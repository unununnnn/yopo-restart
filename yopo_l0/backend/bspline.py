"""均匀三次 B 样条后段。

YOPO 已经选出一条多项式。这里把它拟合成均匀三次 B 样条，只动中间控制点，
用解析梯度下降一条 EGO 风格的代价：平滑、到场景圆柱 / 盒子 / 地面的最近距离、
速度和加速度超过上限。机间椭球项默认关（E-B3）：打开时，对自身中间控制点与
邻机同时刻位置的椭球距离加平方铰链。邻机轨迹来自总线上已提交采样。关上时
不读邻机，代价与加这项之前逐位相同。

E-B3c（``swarm_segment_penalty``，默认关）把同一条椭球铰链改到每段的内点上
算，梯度按三次基函数分回控制点。头三个控制点仍钉死当前 PVA。末端三个在这项
打开时不再钉死：最近点常常落在轨迹尾部，钉死的末端收不到步长。关掉时自由集
和代价都与 E-B3 逐位相同。

E-B4c（``peer_sphere_enabled``，默认关）是另一条线，不改椭球开关。它只在
时间对齐点的 3D 欧氏距离小于 ``peer_sphere_radius_m`` 时加平滑二次/三次铰链，
打在除最后一跨之外的段内采样上。头尾控制点仍按原来的规则钉死，障碍走廊权重
和末段平滑权重不动。关掉时不读邻机，自由集和代价与加这项之前逐位相同。

不复制 ego-planner-swarm。那份代码是 GPLv3，本仓库根目录没有 LICENSE。
代价按论文的角色重写：平滑是控制点三阶差分，碰撞是米制有符号距离上的平方铰链，
可行是速度、加速度超过上限的平方铰链，机间是论文式 (5)(6) 的椭球软障碍。
论文用的是 {p, v} 平面距离和另一组权重，数值对不上，所以权重写在下面，
不假装是论文表里的 λ。椭球的 C、c 是假设，正文只写 c > 1、C 由用户定。

求解器用 scipy 的 L-BFGS-B，迭代上限和时间预算都是硬的。时间预算缺省按评估次数算
（票 L0-DET，见 ``_eval_cap``），不看墙钟；``budget="wall"`` 才按真墙钟截断。失败、超时后没有更安全的点、
或运动学守不住时，交回原来的多项式对象。
"""

from __future__ import annotations

import time
from collections import Counter
from dataclasses import dataclass

import numpy as np

from yopo_l0.scene import Box, Cylinder, Ground

from .config import BackendConfig

# 论文 λ_s=1、λ_c=0.5、λ_d=0.1。那里的碰撞不是米。
# 平方铰链以米计，λ_c 要大到能把分米级的侵入从树干上推开，同时锚住拟合形状。
_LAMBDA_S = 1.0
_LAMBDA_C = 40.0
_LAMBDA_D = 0.1
_LAMBDA_ANCHOR = 0.15
# 机间椭球权重。假设，不是论文 λ_w=0.5：那里的距离量纲对不上米制铰链。
_LAMBDA_W = 40.0
# 椭球距离过近、梯度无定义时的下限。
_SWARM_EPS = 1e-9
# E-B3c：每段内点个数，落在 4–8。取各等分箱的中点，段端点不重复计。
_SWARM_SEGMENT_SAMPLES = 6
# L0 机间碰撞球。peer_sphere_radius 比它多出来的一截是三次过渡带，再往里是二次。
_PEER_COLLISION_BALL_M = 0.70
# 软惩罚比机体半径多出来的一截。铰链在门槛上梯度为零，留 5 cm 才推得动表面。
_CLEARANCE_BUFFER_M = 0.05
_MIN_DURATION_S = 0.6
_MIN_SPANS = 6
_MAX_SPANS = 12
_TARGET_DT_S = 0.22
# 票 L0-DET。空闲 box 上一次代价评估（含梯度）约 2 ms：线程 CPU 计时下 0.020 s 预算
# 被截断的调用停在 8–10 次评估（中位 9），没被截断的跑满 8 次迭代停在 10–11 次。
# 仿真里按这个折算，结果只取决于 (场景卡, seed, 配置)，不取决于机器忙不忙。
_IDLE_EVAL_COST_S = 0.002
# 拟合把五次多项式换成三次样条，净空允许掉这么多仍算「没有更差」。
_CLEARANCE_TOL_M = 0.01
# 计划契约的速度、急动度。加速度用场景 a_max。后段自己的速度软上限是 v_max=6。
_PLAN_V_MAX = 20.0
_PLAN_J_MAX = 20000.0
_SAMPLE_DT_S = 0.02
# 与 core check_derivative_bound 的 DEFAULT_TOL 相同：证出 SAFE 要严格低于 limit - tol。
_KIN_TOL = 1e-9
# 净空的宽相半径。小于这个半径的障碍先算；若最小距离仍压着半径，再扩一圈。
_CULL_CLEARANCE_M = 2.0

# p(u) = [1, u, u^2, u^3] @ _M @ [Q_i, Q_{i+1}, Q_{i+2}, Q_{i+3}]
_M = np.array(
    [
        [1.0, 4.0, 1.0, 0.0],
        [-3.0, 0.0, 3.0, 0.0],
        [3.0, -6.0, 3.0, 0.0],
        [-1.0, 3.0, -3.0, 1.0],
    ],
    dtype=float,
) / 6.0

_U_DYN = (0.0, 0.25, 0.5, 0.75)
_U_POS_LAST = (0.0, 0.25, 0.5, 0.75, 1.0)

_LAYOUT_CACHE: dict[int, tuple[np.ndarray, np.ndarray]] = {}


def summarize_log(rows: list[dict]) -> dict:
    """几次调用合成结果记录里的 backend 块。时间是墙钟，不参与轨迹。"""
    if not rows:
        return {
            "enabled": True,
            "calls": 0,
            "applied": 0,
            "fallback": 0,
            "iterations": None,
            "time_ms": {"p50": None, "p95": None, "max": None},
            "clearance_gain_m": None,
            "reasons": {},
        }
    iters = np.asarray([int(r["iterations"]) for r in rows], dtype=float)
    times = np.asarray([float(r["time_ms"]) for r in rows], dtype=float)
    gains = [
        float(r["clearance_after_m"]) - float(r["clearance_before_m"])
        for r in rows
        if r.get("applied") and r.get("clearance_before_m") is not None and r.get("clearance_after_m") is not None
    ]
    gain = None
    if gains:
        arr = np.asarray(gains, dtype=float)
        gain = {"mean": float(arr.mean()), "min": float(arr.min()), "max": float(arr.max())}
    reasons = Counter(str(r["reason"]) for r in rows)
    return {
        "enabled": True,
        "calls": int(len(rows)),
        "applied": int(sum(1 for r in rows if r.get("applied"))),
        "fallback": int(sum(1 for r in rows if not r.get("applied"))),
        "iterations": {
            "mean": float(iters.mean()),
            "p50": float(np.percentile(iters, 50)),
            "p95": float(np.percentile(iters, 95)),
            "max": int(iters.max()),
            "samples": [int(v) for v in iters],
        },
        "time_ms": {
            "p50": float(np.percentile(times, 50)),
            "p95": float(np.percentile(times, 95)),
            "max": float(times.max()),
            "samples": [float(v) for v in times],
        },
        "clearance_gain_m": gain,
        "reasons": dict(sorted(reasons.items())),
    }


def refine_yopo(traj, cfg: BackendConfig, obstacles, goal=None, neighbors=None):
    """优化一条已选中的 YOPO 多项式。失败时返回同一个对象。

    ``goal`` 留在签名里：本原型不另加终端进度。段内惩罚关掉时，末端 PVA 钉死
    在这条前缀上。打开时末端三个控制点放开，起点 PVA 仍然钉死。
    形状不合法就当这次失败，交回原曲线。
    ``neighbors`` 是邻机已提交轨迹的采样，每份 (t, x, y, z)。只有
    ``cfg.swarm_penalty`` 或 ``cfg.peer_sphere_enabled`` 为真时才读；两个都
    关上时这个参数不影响代价。
    ``cfg.swarm_segment_penalty`` 为真时，椭球项改在段内点上算。该项为假时
    仍用控制点上的椭球项，自由集也不变，与 E-B3 逐位相同。
    ``cfg.peer_sphere_enabled`` 不改自由集，也不改椭球项。
    """
    # 首次 import 不计入这次调用的时间预算。
    import scipy.optimize  # noqa: F401

    t0 = time.perf_counter()
    if goal is not None:
        g = np.asarray(goal, dtype=float)
        if g.shape != (3,) or not np.all(np.isfinite(g)):
            return traj, _record(False, "bad_goal", 0, t0, budget_cut=False)

    try:
        return _refine(traj, cfg, tuple(obstacles or ()), t0, neighbors=neighbors)
    except (ValueError, FloatingPointError, np.linalg.LinAlgError):
        return traj, _record(False, "exception", 0, t0, budget_cut=False)


def _record(
    applied: bool,
    reason: str,
    iterations: int,
    t0: float,
    *,
    budget_cut: bool,
    clearance_before_m: float | None = None,
    clearance_after_m: float | None = None,
    fit_rmse_m: float | None = None,
) -> dict:
    return {
        "applied": bool(applied),
        "reason": reason,
        "iterations": int(iterations),
        "time_ms": (time.perf_counter() - t0) * 1000.0,
        "budget_cut": bool(budget_cut),
        "clearance_before_m": _json_float(clearance_before_m),
        "clearance_after_m": _json_float(clearance_after_m),
        "fit_rmse_m": _json_float(fit_rmse_m),
    }


def _json_float(value) -> float | None:
    if value is None:
        return None
    out = float(value)
    if not np.isfinite(out):
        return None
    return out


def _refine(traj, cfg: BackendConfig, obstacles, t0: float, neighbors=None):
    duration = float(traj.t_end - traj.t_start)
    knots = _choose_knots(duration)
    if knots is None:
        return traj, _record(False, "too_short", 0, t0, budget_cut=False)
    n_span, dt = knots
    p0, v0, a0 = _pva(traj, float(traj.t_start))
    p1, v1, a1 = _pva(traj, float(traj.t_end))
    n_ctrl = n_span + 3
    q_fixed = np.zeros((n_ctrl, 3), dtype=float)
    q_fixed[0], q_fixed[1], q_fixed[2] = _boundary_controls(p0, v0, a0, dt)
    q_fixed[-3], q_fixed[-2], q_fixed[-1] = _boundary_controls(p1, v1, a1, dt)
    swarm = bool(cfg.swarm_penalty)
    # 段内惩罚只在机间项已经打开时替换采样位置。单独打开它不读邻机。
    segment = swarm and bool(cfg.swarm_segment_penalty)
    # 欧氏球不放开末端，也不改椭球采样。关时下面的分支不进入。
    peer = bool(cfg.peer_sphere_enabled)
    # 关：自由集是中间控制点，末端三元组钉死，与 E-B3 相同。
    # 开：末端三个也放开。它们不是当前状态，钉死时尾部最近点推不动。
    free = np.arange(3, n_ctrl if segment else n_ctrl - 3)
    q_fit, rmse = _fit_free(traj, q_fixed, free, n_span, dt)
    if not np.all(np.isfinite(q_fit)):
        return traj, _record(False, "nonfinite", 0, t0, budget_cut=False, fit_rmse_m=rmse)

    d_safe = float(cfg.clearance_m) + _CLEARANCE_BUFFER_M
    idx, wts = _sample_layout(n_span)
    packed = _pack_obstacles(obstacles)
    clr_before = min_signed_distance(traj, obstacles, packed=packed)
    swarm_nbs = tuple(neighbors or ()) if swarm else ()
    # 关时不读 neighbors。开时与椭球分开存，避免椭球关着仍去碰那份采样。
    sphere_nbs = tuple(neighbors or ()) if peer else ()

    def cost_grad(q):
        cost, grad = _cost_grad(
            q,
            q_fit,
            free,
            dt,
            obstacles,
            d_safe,
            float(cfg.v_max),
            float(cfg.a_max),
            idx,
            wts,
            packed=packed,
        )
        if segment:
            sc, sg = _swarm_segment_cost_grad(
                q,
                dt,
                float(traj.t_start),
                swarm_nbs,
                float(cfg.swarm_clearance_m),
                float(cfg.swarm_c),
            )
            # peer 关时原样返回，加法和加这项之前是同一条表达式。
            if not peer:
                return cost + sc, grad + sg
            cost = cost + sc
            grad = grad + sg
        elif swarm:
            sc, sg = _swarm_cost_grad(
                q,
                dt,
                float(traj.t_start),
                swarm_nbs,
                float(cfg.swarm_clearance_m),
                float(cfg.swarm_c),
            )
            if not peer:
                return cost + sc, grad + sg
            cost = cost + sc
            grad = grad + sg
        if peer:
            pc, pg = _peer_sphere_cost_grad(
                q,
                dt,
                float(traj.t_start),
                sphere_nbs,
                float(cfg.peer_sphere_radius_m),
                float(cfg.peer_sphere_weight),
            )
            # 代价和梯度都是 0 时不要做加法，远邻机与关掉逐位相同。
            if pc != 0.0 or np.any(pg):
                return cost + pc, grad + pg
        return cost, grad

    x0 = q_fit[free].reshape(-1).copy()
    x_best, iterations, budget_cut = _minimize(x0, q_fit, free, cost_grad, cfg, t0)
    q_opt = q_fit.copy()
    q_opt[free] = x_best.reshape(-1, 3)

    chosen = None
    chosen_clr = None
    kin_fail = 0
    clear_fail = 0
    for alpha in (1.0, 0.5, 0.25, 0.125, 0.0):
        q = q_opt if alpha == 1.0 else (1.0 - alpha) * q_fit + alpha * q_opt
        built = _to_traj(q, float(traj.t_start), dt, n_span)
        if built is None or not _ends_match(built, (p0, v0, a0), (p1, v1, a1), match_end=not segment):
            kin_fail += 1
            continue
        if not _kin_ok(built, float(cfg.a_max)):
            kin_fail += 1
            continue
        clr = min_signed_distance(built, obstacles, packed=packed)
        if clr + _CLEARANCE_TOL_M < clr_before:
            clear_fail += 1
            continue
        chosen = built
        chosen_clr = clr
        break

    if chosen is None:
        reason = "kinematics" if clear_fail == 0 else "clearance"
        return traj, _record(
            False,
            reason,
            iterations,
            t0,
            budget_cut=budget_cut,
            clearance_before_m=clr_before,
            fit_rmse_m=rmse,
        )
    return chosen, _record(
        True,
        "applied",
        iterations,
        t0,
        budget_cut=budget_cut,
        clearance_before_m=clr_before,
        clearance_after_m=chosen_clr,
        fit_rmse_m=rmse,
    )


def _choose_knots(duration: float) -> tuple[int, float] | None:
    if not np.isfinite(duration) or duration < _MIN_DURATION_S:
        return None
    n_span = int(round(duration / _TARGET_DT_S))
    n_span = max(_MIN_SPANS, min(_MAX_SPANS, n_span))
    return n_span, duration / n_span


def _boundary_controls(p, v, a, dt: float):
    """均匀三次 B 样条在段端点的三个控制点，使位置、速度、加速度等于给定 PVA。"""
    p = np.asarray(p, dtype=float)
    v = np.asarray(v, dtype=float)
    a = np.asarray(a, dtype=float)
    dt2 = dt * dt
    q0 = p - v * dt + a * dt2 / 3.0
    q1 = p - a * dt2 / 6.0
    q2 = p + v * dt + a * dt2 / 3.0
    return q0, q1, q2


def _basis(u: float) -> np.ndarray:
    uu = np.array([1.0, u, u * u, u * u * u], dtype=float)
    return uu @ _M


def _deriv_basis(u: float) -> np.ndarray:
    return np.array([0.0, 1.0, 2.0 * u, 3.0 * u * u], dtype=float) @ _M


def _second_basis(u: float) -> np.ndarray:
    return np.array([0.0, 0.0, 2.0, 6.0 * u], dtype=float) @ _M


_BP_DYN = np.stack([_deriv_basis(u) for u in _U_DYN])
_BPP_DYN = np.stack([_second_basis(u) for u in _U_DYN])
_B_POS = np.stack([_basis(u) for u in _U_DYN])
_B_POS_LAST = np.stack([_basis(u) for u in _U_POS_LAST])
_SEGMENT_U = (np.arange(_SWARM_SEGMENT_SAMPLES, dtype=float) + 0.5) / _SWARM_SEGMENT_SAMPLES
_B_SEGMENT = np.stack([_basis(float(u)) for u in _SEGMENT_U])


def _dyn_samples(n_span: int):
    cached = []
    for span in range(n_span):
        for u in _U_DYN:
            cached.append((span, u, _deriv_basis(u), _second_basis(u)))
    return cached


def _add_limit_grad(grad, span: int, vec: np.ndarray, limit: float, basis: np.ndarray, scale: float) -> float:
    """``max(0, ||vec|| - limit)^2`` 的代价，梯度加进控制点。"""
    nrm = float(np.linalg.norm(vec))
    excess = nrm - float(limit)
    if excess <= 0.0 or nrm < 1e-12:
        return 0.0
    cost = _LAMBDA_D * excess * excess
    dvec = (_LAMBDA_D * 2.0 * excess / nrm) * vec
    grad[span : span + 4] += scale * basis[:, None] * dvec
    return cost


def _sample_layout(n_span: int) -> tuple[np.ndarray, np.ndarray]:
    hit = _LAYOUT_CACHE.get(n_span)
    if hit is not None:
        return hit
    idx = []
    wts = []
    for span in range(n_span):
        basis = _B_POS if span < n_span - 1 else _B_POS_LAST
        for b in basis:
            idx.append((span, span + 1, span + 2, span + 3))
            wts.append(b)
    out = (np.asarray(idx, dtype=int), np.asarray(wts, dtype=float))
    _LAYOUT_CACHE[n_span] = out
    return out


def _fit_free(traj, q_fixed: np.ndarray, free: np.ndarray, n_span: int, dt: float):
    """中间控制点对 YOPO 采样做最小二乘。端点三元组不动。"""
    t0 = float(traj.t_start)
    t1 = float(traj.t_end)
    rows = []
    targets = []
    fixed_offset = []
    free_of = {int(i): k for k, i in enumerate(free)}
    times = []
    us = (0.25, 0.5, 0.75)
    for span in range(n_span):
        for u in us:
            times.append(min(t0 + (span + u) * dt, t1))
    pts = _eval_positions(traj, np.asarray(times, dtype=float))
    k = 0
    for span in range(n_span):
        for u in us:
            b = _basis(u)
            row = np.zeros(len(free), dtype=float)
            offset = np.zeros(3, dtype=float)
            for j in range(4):
                index = span + j
                if index in free_of:
                    row[free_of[index]] = b[j]
                else:
                    offset += b[j] * q_fixed[index]
            rows.append(row)
            targets.append(pts[k])
            fixed_offset.append(offset)
            k += 1
    design = np.asarray(rows, dtype=float)
    rhs = np.asarray(targets, dtype=float) - np.asarray(fixed_offset, dtype=float)
    solved, _, _, _ = np.linalg.lstsq(design, rhs, rcond=None)
    q = q_fixed.copy()
    q[free] = solved.reshape(len(free), 3)
    pred = design @ solved + np.asarray(fixed_offset, dtype=float)
    err = pred - np.asarray(targets, dtype=float)
    rmse = float(np.sqrt(np.mean(np.sum(err * err, axis=1))))
    return q, rmse


def _curve_points(q: np.ndarray, idx: np.ndarray, wts: np.ndarray) -> np.ndarray:
    pts = np.zeros((len(idx), 3), dtype=float)
    for k in range(4):
        pts += wts[:, k, None] * q[idx[:, k]]
    return pts


def _control_windows(q: np.ndarray) -> np.ndarray:
    """(n_span, 4, 3) 滑窗，每段四个控制点。"""
    return np.lib.stride_tricks.sliding_window_view(q, (4, 3))[:, 0, :, :]


def _hinge_limit(grad: np.ndarray, vecs: np.ndarray, limit: float, basis: np.ndarray, scale: float) -> float:
    """vecs (n_span, 4, 3)，basis (4, 4) 是各 u 的基。"""
    nrm = np.linalg.norm(vecs, axis=2)
    excess = nrm - float(limit)
    active = (excess > 0.0) & (nrm > 1e-12)
    if not np.any(active):
        return 0.0
    cost = _LAMBDA_D * float(np.sum(excess[active] ** 2))
    dvec = np.zeros_like(vecs)
    dvec[active] = (_LAMBDA_D * 2.0 * excess[active] / nrm[active])[:, None] * vecs[active]
    contrib = scale * np.einsum("ub,suc->sbc", basis, dvec)
    n_span = vecs.shape[0]
    for k in range(4):
        grad[k : k + n_span] += contrib[:, k, :]
    return cost


def _swarm_cost_grad(q, dt, t_start, neighbors, clearance, c, lam: float = _LAMBDA_W):
    """中间控制点对邻机同时刻位置的椭球平方铰链。

    距离 d = sqrt(dx² + dy² + (c·dz)²)。d < C 时代价是 λ (C - d)²。
    C、c、λ 都是假设，不是论文数值。控制点 i 对应时刻 t_start + (i - 1)·dt
    （均匀三次样条的 Greville 横坐标），夹到曲线区间。只罚可动的中间点。
    没有时间重叠的邻机采样不外推。
    """
    n = int(q.shape[0])
    free = np.arange(3, n - 3)
    grad = np.zeros_like(q)
    if free.size == 0 or not neighbors:
        return 0.0, grad
    t_end = float(t_start) + float(dt) * (n - 3)
    times = np.clip(float(t_start) + (free.astype(float) - 1.0) * float(dt), float(t_start), t_end)
    pts = q[free]
    cost = 0.0
    g_free = np.zeros_like(pts)
    c2 = float(c) * float(c)
    thresh = float(clearance)
    scale_lam = float(lam)
    for raw in neighbors:
        arr = np.asarray(raw, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 4 or arr.shape[0] == 0:
            continue
        if not bool(np.all(np.isfinite(arr))):
            continue
        ts = arr[:, 0]
        mask = (times >= float(ts[0])) & (times <= float(ts[-1]))
        if not bool(np.any(mask)):
            continue
        t = times[mask]
        delta = pts[mask] - np.column_stack((
            np.interp(t, ts, arr[:, 1]),
            np.interp(t, ts, arr[:, 2]),
            np.interp(t, ts, arr[:, 3]),
        ))
        dist = np.sqrt(delta[:, 0] * delta[:, 0] + delta[:, 1] * delta[:, 1] + c2 * delta[:, 2] * delta[:, 2])
        zero = dist <= _SWARM_EPS
        active = (dist < thresh) & ~zero
        if bool(np.any(zero)):
            # 重合时椭球距离的梯度无定义，只加代价，不推梯度。
            cost += scale_lam * float(np.sum((thresh - dist[zero]) ** 2))
        if not bool(np.any(active)):
            continue
        viol = thresh - dist[active]
        cost += scale_lam * float(np.sum(viol * viol))
        push = ((-2.0 * scale_lam * viol) / dist[active])[:, None] * np.column_stack((
            delta[active, 0],
            delta[active, 1],
            c2 * delta[active, 2],
        ))
        idx = np.flatnonzero(mask)[active]
        g_free[idx] += push
    grad[free] = g_free
    return float(cost), grad


def _swarm_segment_cost_grad(q, dt, t_start, neighbors, clearance, c, lam: float = _LAMBDA_W):
    """段内点上的椭球平方铰链，梯度按三次基函数分到控制点。

    每段取 ``_SWARM_SEGMENT_SAMPLES`` 个箱中点（不含段端点，避免相邻段重复）。
    样本位置 p(u) = Σ_j B_j(u) Q_{s+j}。铰链对 p 的梯度乘上 B_j(u) 加回
    Q_{s+j}。头三个控制点仍钉死。末端三个在段内惩罚打开时一并放开，近端样本
    的梯度才能推动轨迹尾部。控制点版只在自由点自己的 Greville 时刻看邻机，
    段内和尾部最近点的代价常常是 0。

    距离、门槛、重合时不推梯度，都和 ``_swarm_cost_grad`` 同一套。C、c、λ
    仍是假设，不是论文数值。没有时间重叠的邻机采样不外推。
    """
    n = int(q.shape[0])
    n_span = n - 3
    grad = np.zeros_like(q)
    if n_span < 1 or not neighbors:
        return 0.0, grad
    windows = _control_windows(q)
    pts = np.einsum("kb,sbc->skc", _B_SEGMENT, windows)
    span = np.arange(n_span, dtype=float)[:, None]
    times = float(t_start) + (span + _SEGMENT_U[None, :]) * float(dt)
    flat_t = times.reshape(-1)
    flat_p = pts.reshape(-1, 3)
    cost = 0.0
    dpos = np.zeros_like(flat_p)
    c2 = float(c) * float(c)
    thresh = float(clearance)
    scale_lam = float(lam)
    for raw in neighbors:
        arr = np.asarray(raw, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 4 or arr.shape[0] == 0:
            continue
        if not bool(np.all(np.isfinite(arr))):
            continue
        ts = arr[:, 0]
        mask = (flat_t >= float(ts[0])) & (flat_t <= float(ts[-1]))
        if not bool(np.any(mask)):
            continue
        t = flat_t[mask]
        delta = flat_p[mask] - np.column_stack((
            np.interp(t, ts, arr[:, 1]),
            np.interp(t, ts, arr[:, 2]),
            np.interp(t, ts, arr[:, 3]),
        ))
        dist = np.sqrt(delta[:, 0] * delta[:, 0] + delta[:, 1] * delta[:, 1] + c2 * delta[:, 2] * delta[:, 2])
        zero = dist <= _SWARM_EPS
        active = (dist < thresh) & ~zero
        if bool(np.any(zero)):
            cost += scale_lam * float(np.sum((thresh - dist[zero]) ** 2))
        if not bool(np.any(active)):
            continue
        viol = thresh - dist[active]
        cost += scale_lam * float(np.sum(viol * viol))
        push = ((-2.0 * scale_lam * viol) / dist[active])[:, None] * np.column_stack((
            delta[active, 0],
            delta[active, 1],
            c2 * delta[active, 2],
        ))
        idx = np.flatnonzero(mask)
        dpos[idx[active]] += push
    d_sk = dpos.reshape(n_span, _SWARM_SEGMENT_SAMPLES, 3)
    contrib = np.einsum("kb,skc->sbc", _B_SEGMENT, d_sk)
    for j in range(4):
        grad[j : j + n_span] += contrib[:, j, :]
    return float(cost), grad


def _peer_sphere_sample(dist: np.ndarray, radius: float, weight: float, band: float):
    """平滑二次/三次铰链。返回每个样本的代价，以及 d(cost)/d(dist)（≤ 0）。

    s = R - d。d >= R 时为 0，球面处函数值、一阶和二阶导数都是 0。
    band = R - 0.70 m（默认 0.15 m）是碰撞球外的三次带：0 < s <= band 用
    w·s³/band，再往里（已经进了 0.70 m 球）用与之 C1 相接的二次
    w·(s² + band·s - band²)。band <= 0 时整段都是 w·s²。
    """
    dist = np.asarray(dist, dtype=float)
    cost = np.zeros(dist.shape, dtype=float)
    dcost = np.zeros(dist.shape, dtype=float)
    inside = dist < float(radius)
    if not np.any(inside):
        return cost, dcost
    s = float(radius) - dist
    if band <= 1e-12:
        cost[inside] = weight * s[inside] * s[inside]
        dcost[inside] = -2.0 * weight * s[inside]
        return cost, dcost
    cubic = inside & (s <= band)
    deep = inside & ~cubic
    if np.any(cubic):
        sc = s[cubic]
        cost[cubic] = weight * (sc * sc * sc) / band
        dcost[cubic] = -(3.0 * weight * (sc * sc) / band)
    if np.any(deep):
        sd = s[deep]
        cost[deep] = weight * (sd * sd + band * sd - band * band)
        dcost[deep] = -(weight * (2.0 * sd + band))
    return cost, dcost


def _peer_sphere_cost_grad(q, dt, t_start, neighbors, radius, weight):
    """除最后一跨外，段内点对邻机同时刻位置的欧氏球铰链。

    采样与 E-B3c 相同：每段 ``_SWARM_SEGMENT_SAMPLES`` 个箱中点，梯度按三次
    基函数分回控制点。最后一跨不进这项，留给平滑项；头三个和尾三个控制点仍由
    调用方钉死，这里不改自由集。没有时间重叠的邻机采样不外推。重合时距离的
    方向无定义，只加代价，不推梯度。
    """
    n = int(q.shape[0])
    n_span = n - 3
    grad = np.zeros_like(q)
    radius = float(radius)
    weight = float(weight)
    band = radius - _PEER_COLLISION_BALL_M
    # 末段（最后一跨）不进机间球，避免去拽钉死的末端 PVA。
    n_pen = n_span - 1
    if n_pen < 1 or not neighbors or not (weight > 0.0) or not (radius > 0.0):
        return 0.0, grad
    windows = _control_windows(q)[:n_pen]
    pts = np.einsum("kb,sbc->skc", _B_SEGMENT, windows)
    span = np.arange(n_pen, dtype=float)[:, None]
    times = float(t_start) + (span + _SEGMENT_U[None, :]) * float(dt)
    flat_t = times.reshape(-1)
    flat_p = pts.reshape(-1, 3)
    cost = 0.0
    dpos = np.zeros_like(flat_p)
    for raw in neighbors:
        arr = np.asarray(raw, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 4 or arr.shape[0] == 0:
            continue
        if not bool(np.all(np.isfinite(arr))):
            continue
        ts = arr[:, 0]
        mask = (flat_t >= float(ts[0])) & (flat_t <= float(ts[-1]))
        if not bool(np.any(mask)):
            continue
        t = flat_t[mask]
        delta = flat_p[mask] - np.column_stack((
            np.interp(t, ts, arr[:, 1]),
            np.interp(t, ts, arr[:, 2]),
            np.interp(t, ts, arr[:, 3]),
        ))
        dist = np.sqrt(np.sum(delta * delta, axis=1))
        sample_cost, dcost_dd = _peer_sphere_sample(dist, radius, weight, band)
        zero = dist <= _SWARM_EPS
        if bool(np.any(zero)):
            # 方向无定义。代价按贴在球心算，梯度留 0。
            zdist = np.zeros(int(np.count_nonzero(zero)), dtype=float)
            zcost, _zd = _peer_sphere_sample(zdist, radius, weight, band)
            sample_cost = sample_cost.copy()
            sample_cost[zero] = zcost
            dcost_dd = dcost_dd.copy()
            dcost_dd[zero] = 0.0
        cost += float(np.sum(sample_cost))
        active = (dcost_dd != 0.0) & ~zero
        if not bool(np.any(active)):
            continue
        push = (dcost_dd[active] / dist[active])[:, None] * delta[active]
        idx = np.flatnonzero(mask)
        dpos[idx[active]] += push
    if cost == 0.0 and not np.any(dpos):
        return 0.0, grad
    d_sk = dpos.reshape(n_pen, _SWARM_SEGMENT_SAMPLES, 3)
    contrib = np.einsum("kb,skc->sbc", _B_SEGMENT, d_sk)
    for j in range(4):
        grad[j : j + n_pen] += contrib[:, j, :]
    return float(cost), grad


def _cost_grad(q, q_fit, free, dt, obstacles, d_safe, v_max, a_max, idx, wts, packed=None):
    n = q.shape[0]
    n_span = n - 3
    grad = np.zeros_like(q)
    cost = 0.0

    q0, q1, q2, q3 = q[:-3], q[1:-2], q[2:-1], q[3:]
    diff = q3 - 3.0 * q2 + 3.0 * q1 - q0
    cost += _LAMBDA_S * float(np.sum(diff * diff))
    chain = _LAMBDA_S * 2.0 * diff
    grad[:-3] -= chain
    grad[1:-2] += 3.0 * chain
    grad[2:-1] -= 3.0 * chain
    grad[3:] += chain

    # 罚的是曲线上的速度和加速度，不是控制点差分。差分是凸包上界，弯一下就被判爆。
    windows = _control_windows(q)
    vel = np.einsum("ub,sbc->suc", _BP_DYN, windows) / dt
    acc = np.einsum("ub,sbc->suc", _BPP_DYN, windows) / (dt * dt)
    cost += _hinge_limit(grad, vel, v_max, _BP_DYN, 1.0 / dt)
    cost += _hinge_limit(grad, acc, a_max, _BPP_DYN, 1.0 / (dt * dt))

    delta = q[free] - q_fit[free]
    cost += _LAMBDA_ANCHOR * float(np.sum(delta * delta))
    grad[free] += _LAMBDA_ANCHOR * 2.0 * delta

    if obstacles:
        pts = _curve_points(q, idx, wts)
        sd, sd_grad = _nearest(pts, obstacles, packed=packed, active_limit=float(d_safe))
        viol = d_safe - sd
        active = viol > 0.0
        if np.any(active):
            cost += _LAMBDA_C * float(np.sum(viol[active] ** 2))
            d_pts = np.zeros_like(pts)
            d_pts[active] = (-2.0 * _LAMBDA_C * viol[active])[:, None] * sd_grad[active]
            for k in range(4):
                np.add.at(grad, idx[:, k], wts[:, k, None] * d_pts)

    return float(cost), grad


def _minimize(x0, q_fit, free, cost_grad, cfg: BackendConfig, t0: float):
    # 走底层 fmin_l_bfgs_b，不走 minimize 包装。同一套 factr / pgtol / maxls / maxcor，
    # 8 次迭代的解与 minimize(..., method="L-BFGS-B") 逐位相同，但包装本身能吃掉十几毫秒，
    # 几乎就是整份 0.02 s 预算。
    from scipy.optimize import fmin_l_bfgs_b

    state = {
        "best_x": np.array(x0, dtype=float, copy=True),
        "best_c": None,
        "evals": 0,
        "cut": False,
    }
    deadline = t0 + float(cfg.time_budget_s)
    wall = cfg.budget == "wall"
    cap = _eval_cap(cfg)

    def over_budget() -> bool:
        if state["evals"] == 0:
            return False
        if wall:
            return time.perf_counter() > deadline
        return state["evals"] >= cap

    def fun(x):
        if over_budget():
            state["cut"] = True
            fill = 0.0 if state["best_c"] is None else state["best_c"]
            return fill, np.zeros_like(x)
        q = q_fit.copy()
        q[free] = np.asarray(x, dtype=float).reshape(-1, 3)
        cost, grad = cost_grad(q)
        g = grad[free].reshape(-1)
        state["evals"] += 1
        if np.isfinite(cost) and np.all(np.isfinite(g)):
            if state["best_c"] is None or cost < state["best_c"]:
                state["best_c"] = float(cost)
                state["best_x"] = np.array(x, dtype=float, copy=True)
            return float(cost), g
        return 1e12, np.zeros_like(x)

    # minimize 的 ftol=1e-12 对应 factr * machine_eps。
    factr = 1e-12 / np.finfo(float).eps
    _x, _f, info = fmin_l_bfgs_b(
        fun,
        np.array(x0, dtype=float, copy=True),
        maxiter=int(cfg.max_iterations),
        factr=factr,
        pgtol=1e-6,
        maxls=16,
        m=8,
    )
    iterations = int(info.get("nit", 0) or 0)
    return state["best_x"], iterations, bool(state["cut"])


def _eval_cap(cfg: BackendConfig) -> int:
    """仿真里的时间预算：空闲机器上这份预算能做几次代价评估。至少 1 次。"""
    return max(1, int(round(float(cfg.time_budget_s) / _IDLE_EVAL_COST_S)))


def _to_traj(q: np.ndarray, t_start: float, dt: float, n_span: int):
    from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment

    if not np.all(np.isfinite(q)):
        return None
    scales = np.array([1.0, 1.0 / dt, 1.0 / dt**2, 1.0 / dt**3], dtype=float)
    segments = []
    for span in range(n_span):
        coeff_u = _M @ q[span : span + 4]
        coeffs = (coeff_u * scales[:, None]).T
        segments.append(PolySegment(coeffs, dt))
    return PiecewisePolynomial(t_start, segments)


def _pva(traj, t: float):
    return tuple(np.asarray(traj.evaluate(t, k), dtype=float) for k in range(3))


def _ends_match(traj, start, end, *, match_end: bool = True) -> bool:
    got0 = _pva(traj, float(traj.t_start))
    pairs = [(got0, start)]
    if match_end:
        pairs.append((_pva(traj, float(traj.t_end)), end))
    for got, want in pairs:
        for a, b in zip(got, want, strict=True):
            if float(np.max(np.abs(a - b))) > 1e-6:
                return False
    return True


def _kin_ok(traj, a_max: float) -> bool:
    """三次段用解析上界；证不出或不三次时退回 core 的 Bernstein 检查。"""
    segs = getattr(traj, "segments", ())
    if segs and all(int(s.coeffs.shape[1]) - 1 <= 3 for s in segs):
        proved = _cubic_kin_ok(traj, float(a_max))
        if proved is not None:
            return proved
    from yopo_swarm_core.collision import Verdict, check_derivative_bound

    limits = ((1, _PLAN_V_MAX), (2, float(a_max)), (3, _PLAN_J_MAX))
    return all(check_derivative_bound(traj, order, lim).verdict is Verdict.SAFE for order, lim in limits)


def _cubic_kin_ok(traj, a_max: float) -> bool | None:
    """三次多项式段上速度、加速度、急动度范数的解析判定。

    返回 True / False 表示已经证出 SAFE 或已经可以拒绝；None 表示交给 core。
    加速度在段上是直线，范数在端点取到最大。急动度是常向量。速度是二次，
    驻点来自 v·a = 0（三次方程）。判定口径与 check_derivative_bound 一致：
    只有严格低于 limit - tol 才算 SAFE。
    """
    a_lim = float(a_max) - _KIN_TOL
    v_lim = _PLAN_V_MAX - _KIN_TOL
    j_lim = _PLAN_J_MAX - _KIN_TOL
    if a_lim <= 0.0 or v_lim <= 0.0 or j_lim <= 0.0:
        return False
    for seg in traj.segments:
        c = np.zeros((3, 4), dtype=float)
        src = np.asarray(seg.coeffs, dtype=float)
        c[:, : src.shape[1]] = src
        duration = float(seg.duration)
        c1, c2, c3 = c[:, 1], c[:, 2], c[:, 3]
        jerk = 6.0 * c3
        if float(np.dot(jerk, jerk)) >= j_lim * j_lim:
            return False
        a0 = 2.0 * c2
        a_end = a0 + 6.0 * c3 * duration
        if float(np.dot(a0, a0)) >= a_lim * a_lim or float(np.dot(a_end, a_end)) >= a_lim * a_lim:
            return False
        v0 = c1
        v_end = c1 + 2.0 * c2 * duration + 3.0 * c3 * duration * duration
        if float(np.dot(v0, v0)) >= v_lim * v_lim or float(np.dot(v_end, v_end)) >= v_lim * v_lim:
            return False
        # (c1 + 2 c2 t + 3 c3 t^2) · (2 c2 + 6 c3 t) = 0
        A = 18.0 * float(np.dot(c3, c3))
        B = 18.0 * float(np.dot(c2, c3))
        C = 6.0 * float(np.dot(c1, c3)) + 4.0 * float(np.dot(c2, c2))
        D = 2.0 * float(np.dot(c1, c2))
        if abs(A) + abs(B) + abs(C) + abs(D) == 0.0:
            continue
        try:
            roots = np.roots((A, B, C, D))
        except np.linalg.LinAlgError:
            return None
        for root in roots:
            if abs(float(np.imag(root))) > 1e-12:
                continue
            t = float(np.real(root))
            if t <= 0.0 or t >= duration:
                continue
            vel = c1 + 2.0 * c2 * t + 3.0 * c3 * t * t
            if float(np.dot(vel, vel)) >= v_lim * v_lim:
                return False
    return True


def min_signed_distance(traj, obstacles, step: float = _SAMPLE_DT_S, packed=None) -> float:
    """轨迹上到障碍表面的最小有符号距离。没有障碍时是正无穷。穿进表面为负。"""
    if not obstacles:
        return float("inf")
    span = float(traj.t_end - traj.t_start)
    n = max(2, int(np.ceil(span / step)) + 1)
    times = np.linspace(float(traj.t_start), float(traj.t_end), n)
    pts = _eval_positions(traj, times)
    sd, _grad = _nearest(pts, obstacles, packed=packed)
    return float(np.min(sd))


def _horner(coeffs: np.ndarray, tau: np.ndarray) -> np.ndarray:
    """coeffs (3, deg+1)，tau (N,)，返回 (N, 3)。"""
    val = np.broadcast_to(np.asarray(coeffs[:, -1], dtype=float), (tau.shape[0], 3)).copy()
    for k in range(coeffs.shape[1] - 2, -1, -1):
        val = val * tau[:, None] + coeffs[:, k]
    return val


def _eval_positions(traj, times: np.ndarray) -> np.ndarray:
    """按 locate 的规则批量求位置：段边界归入后一段，最后边界归入最后一段。"""
    times = np.asarray(times, dtype=float)
    pts = np.empty((len(times), 3), dtype=float)
    segs = traj.segments
    last = len(segs) - 1
    bounds = traj._bounds
    assigned = np.zeros(len(times), dtype=bool)
    for i, seg in enumerate(segs):
        start, end = bounds[i], bounds[i + 1]
        if i == last:
            mask = (times >= start) & (times <= end)
        else:
            mask = (times >= start) & (times < end)
        if not np.any(mask):
            continue
        tau = np.clip(times[mask] - start, 0.0, seg.duration)
        pts[mask] = _horner(np.asarray(seg.coeffs, dtype=float), tau)
        assigned[mask] = True
    if not np.all(assigned):
        for k in np.flatnonzero(~assigned):
            pts[k] = np.asarray(traj.evaluate(float(times[k])), dtype=float)
    return pts


@dataclass
class _PackedObstacles:
    n: int
    up_i: np.ndarray
    up_xy: np.ndarray
    up_r: np.ndarray
    up_zmid: np.ndarray
    up_half: np.ndarray
    cyl_i: np.ndarray
    cyl_c: np.ndarray
    cyl_ax: np.ndarray
    cyl_h: np.ndarray
    cyl_r: np.ndarray
    deg_i: np.ndarray
    deg_a: np.ndarray
    deg_r: np.ndarray
    box_i: np.ndarray
    box_c: np.ndarray
    box_h: np.ndarray
    grd_i: np.ndarray
    grd_z: np.ndarray
    other: tuple


def _pack_obstacles(obstacles) -> _PackedObstacles | None:
    if not obstacles:
        return None
    up_i, up_xy, up_r, up_zmid, up_half = [], [], [], [], []
    cyl_i, cyl_c, cyl_ax, cyl_h, cyl_r = [], [], [], [], []
    deg_i, deg_a, deg_r = [], [], []
    box_i, box_c, box_h = [], [], []
    grd_i, grd_z = [], []
    other = []
    for i, ob in enumerate(obstacles):
        if isinstance(ob, Cylinder):
            a, b = ob.axis_ends()
            ab = b - a
            length = float(np.linalg.norm(ab))
            radius = float(ob.radius)
            if length < 1e-9:
                deg_i.append(i)
                deg_a.append(a)
                deg_r.append(radius)
            elif ob.upright():
                up_i.append(i)
                up_xy.append((float(ob.center_xy[0]), float(ob.center_xy[1])))
                up_r.append(radius)
                z_lo, z_hi = float(ob.z_lo), float(ob.z_hi)
                up_zmid.append(0.5 * (z_lo + z_hi))
                up_half.append(0.5 * (z_hi - z_lo))
            else:
                cyl_i.append(i)
                cyl_c.append(0.5 * (a + b))
                cyl_ax.append(ab / length)
                cyl_h.append(0.5 * length)
                cyl_r.append(radius)
        elif isinstance(ob, Box):
            lo = np.asarray(ob.lo, dtype=float)
            hi = np.asarray(ob.hi, dtype=float)
            box_i.append(i)
            box_c.append(0.5 * (lo + hi))
            box_h.append(0.5 * (hi - lo))
        elif isinstance(ob, Ground):
            grd_i.append(i)
            grd_z.append(float(ob.z))
        else:
            other.append((i, ob))
    empty = np.zeros((0, 3), dtype=float)
    return _PackedObstacles(
        n=len(obstacles),
        up_i=np.asarray(up_i, dtype=int),
        up_xy=np.asarray(up_xy, dtype=float).reshape(-1, 2) if up_xy else np.zeros((0, 2)),
        up_r=np.asarray(up_r, dtype=float),
        up_zmid=np.asarray(up_zmid, dtype=float),
        up_half=np.asarray(up_half, dtype=float),
        cyl_i=np.asarray(cyl_i, dtype=int),
        cyl_c=np.asarray(cyl_c, dtype=float).reshape(-1, 3) if cyl_c else empty.copy(),
        cyl_ax=np.asarray(cyl_ax, dtype=float).reshape(-1, 3) if cyl_ax else empty.copy(),
        cyl_h=np.asarray(cyl_h, dtype=float),
        cyl_r=np.asarray(cyl_r, dtype=float),
        deg_i=np.asarray(deg_i, dtype=int),
        deg_a=np.asarray(deg_a, dtype=float).reshape(-1, 3) if deg_a else empty.copy(),
        deg_r=np.asarray(deg_r, dtype=float),
        box_i=np.asarray(box_i, dtype=int),
        box_c=np.asarray(box_c, dtype=float).reshape(-1, 3) if box_c else empty.copy(),
        box_h=np.asarray(box_h, dtype=float).reshape(-1, 3) if box_h else empty.copy(),
        grd_i=np.asarray(grd_i, dtype=int),
        grd_z=np.asarray(grd_z, dtype=float),
        other=tuple(other),
    )


def _aabb_cyl_lb(lo, hi, xy, radius, zmid, half) -> np.ndarray:
    dx = np.maximum(np.maximum(lo[0] - xy[:, 0], 0.0), xy[:, 0] - hi[0])
    dy = np.maximum(np.maximum(lo[1] - xy[:, 1], 0.0), xy[:, 1] - hi[1])
    dz = np.maximum(np.maximum(lo[2] - (zmid + half), 0.0), (zmid - half) - hi[2])
    return np.sqrt(dx * dx + dy * dy + dz * dz) - radius


def _aabb_box_lb(lo, hi, center, half) -> np.ndarray:
    blo = center - half
    bhi = center + half
    dx = np.maximum(np.maximum(lo[0] - bhi[:, 0], 0.0), blo[:, 0] - hi[0])
    dy = np.maximum(np.maximum(lo[1] - bhi[:, 1], 0.0), blo[:, 1] - hi[1])
    dz = np.maximum(np.maximum(lo[2] - bhi[:, 2], 0.0), blo[:, 2] - hi[2])
    return np.sqrt(dx * dx + dy * dy + dz * dz)


def _aabb_seg_cyl_lb(lo, hi, center, axis, half, radius) -> np.ndarray:
    """直立公式推广：把点 AABB 到有限段的下界再减半径。略松，但不漏。"""
    a = center - axis * half[:, None]
    b = center + axis * half[:, None]
    # 段两端点张成的包围盒再膨胀 0，再减半径，作为下界会偏松（斜柱）。
    slo = np.minimum(a, b)
    shi = np.maximum(a, b)
    dx = np.maximum(np.maximum(lo[0] - shi[:, 0], 0.0), slo[:, 0] - hi[0])
    dy = np.maximum(np.maximum(lo[1] - shi[:, 1], 0.0), slo[:, 1] - hi[1])
    dz = np.maximum(np.maximum(lo[2] - shi[:, 2], 0.0), slo[:, 2] - hi[2])
    return np.sqrt(dx * dx + dy * dy + dz * dz) - radius


def _keep_mask(lb: np.ndarray, radius: float) -> np.ndarray:
    if lb.size == 0:
        return np.zeros(0, dtype=bool)
    return lb < radius


def _scatter_min(best, grad, sd, g, keep_i, take_radius: float | None):
    """把一批障碍的有符号距离并进当前最近场。保持原顺序下「先写入者赢平手」。"""
    if sd.size == 0 or keep_i.size == 0:
        return best, grad
    if take_radius is not None:
        # 只需关心可能低于门槛的障碍；最近若全在门槛外，铰链本来就不亮。
        pass
    idx = np.argmin(sd, axis=1)
    rows = np.arange(sd.shape[0])
    val = sd[rows, idx]
    take = val < best
    best = np.where(take, val, best)
    grad = np.where(take[:, None], g[rows, idx], grad)
    return best, grad


def _upright_sd_grad(points, xy, radius, zmid, half):
    if len(xy) == 0:
        return np.zeros((len(points), 0)), np.zeros((len(points), 0, 3))
    dx = points[:, 0:1] - xy[None, :, 0]
    dy = points[:, 1:2] - xy[None, :, 1]
    axial = points[:, 2:3] - zmid[None, :]
    radial = np.hypot(dx, dy)
    d_rad = radial - radius
    d_ax = np.abs(axial) - half
    outside_len = np.hypot(np.maximum(d_rad, 0.0), np.maximum(d_ax, 0.0))
    inside = np.minimum(np.maximum(d_rad, d_ax), 0.0)
    sd = outside_len + inside
    # 只给每个点的最近柱子算梯度。
    n = len(points)
    win = np.argmin(sd, axis=1)
    rows = np.arange(n)
    rad = radial[rows, win]
    drv = d_rad[rows, win]
    daxv = d_ax[rows, win]
    axv = axial[rows, win]
    olen = np.maximum(outside_len[rows, win], 1e-12)
    hat = np.zeros((n, 3), dtype=float)
    nz = rad > 1e-9
    if np.any(nz):
        hat[nz, 0] = dx[rows, win][nz] / rad[nz]
        hat[nz, 1] = dy[rows, win][nz] / rad[nz]
    if np.any(~nz):
        hat[~nz, 0] = 1.0
    sign_ax = np.where(axv >= 0.0, 1.0, -1.0)
    g_rad = np.zeros(n, dtype=float)
    g_ax = np.zeros(n, dtype=float)
    corner = (drv > 0.0) & (daxv > 0.0)
    side = (drv > 0.0) & (daxv <= 0.0)
    cap = (drv <= 0.0) & (daxv > 0.0)
    inner = (drv <= 0.0) & (daxv <= 0.0)
    g_rad[corner] = drv[corner] / olen[corner]
    g_ax[corner] = sign_ax[corner] * daxv[corner] / olen[corner]
    g_rad[side] = 1.0
    g_ax[cap] = sign_ax[cap]
    inner_side = inner & (drv >= daxv)
    g_rad[inner_side] = 1.0
    g_ax[inner & ~inner_side] = sign_ax[inner & ~inner_side]
    grad = g_rad[:, None] * hat
    grad[:, 2] += g_ax
    # 仍返回全表 sd，给 argmin 平手规则用；梯度只在胜出列有值，其它列不会被选中。
    g_all = np.zeros((n, len(xy), 3), dtype=float)
    g_all[rows, win] = grad
    return sd, g_all


def _general_cyl_sd_grad(points, centers, axes, halfs, radii):
    if len(centers) == 0:
        return np.zeros((len(points), 0)), np.zeros((len(points), 0, 3))
    rel = points[:, None, :] - centers[None, :, :]
    axial = np.einsum("nci,ci->nc", rel, axes)
    radial_vec = rel - axial[:, :, None] * axes[None, :, :]
    radial = np.linalg.norm(radial_vec, axis=2)
    d_rad = radial - radii
    d_ax = np.abs(axial) - halfs
    outside_len = np.hypot(np.maximum(d_rad, 0.0), np.maximum(d_ax, 0.0))
    inside = np.minimum(np.maximum(d_rad, d_ax), 0.0)
    sd = outside_len + inside
    n, c = sd.shape
    win = np.argmin(sd, axis=1) if c else np.zeros(n, dtype=int)
    rows = np.arange(n)
    hat = np.zeros((n, 3), dtype=float)
    if c:
        rad = radial[rows, win]
        rvec = radial_vec[rows, win]
        nz = rad > 1e-9
        hat[nz] = rvec[nz] / rad[nz, None]
        if np.any(~nz):
            ax = axes[win[~nz]]
            aux = np.zeros_like(ax)
            aux[:, 0] = 1.0
            flip = np.abs(ax[:, 0]) >= 0.9
            aux[flip] = (0.0, 1.0, 0.0)
            perp = np.cross(ax, aux)
            perp /= np.linalg.norm(perp, axis=1, keepdims=True)
            hat[~nz] = perp
        drv = d_rad[rows, win]
        daxv = d_ax[rows, win]
        axv = axial[rows, win]
        olen = np.maximum(outside_len[rows, win], 1e-12)
        sign_ax = np.where(axv >= 0.0, 1.0, -1.0)
        g_rad = np.zeros(n, dtype=float)
        g_ax = np.zeros(n, dtype=float)
        corner = (drv > 0.0) & (daxv > 0.0)
        side = (drv > 0.0) & (daxv <= 0.0)
        cap = (drv <= 0.0) & (daxv > 0.0)
        inner = (drv <= 0.0) & (daxv <= 0.0)
        g_rad[corner] = drv[corner] / olen[corner]
        g_ax[corner] = sign_ax[corner] * daxv[corner] / olen[corner]
        g_rad[side] = 1.0
        g_ax[cap] = sign_ax[cap]
        inner_side = inner & (drv >= daxv)
        g_rad[inner_side] = 1.0
        g_ax[inner & ~inner_side] = sign_ax[inner & ~inner_side]
        grad = g_rad[:, None] * hat + g_ax[:, None] * axes[win]
    else:
        grad = hat
    g_all = np.zeros((n, c, 3), dtype=float)
    if c:
        g_all[rows, win] = grad
    return sd, g_all


def _box_sd_grad(points, centers, halfs):
    if len(centers) == 0:
        return np.zeros((len(points), 0)), np.zeros((len(points), 0, 3))
    rel = points[:, None, :] - centers[None, :, :]
    q = np.abs(rel) - halfs
    outside_vec = np.maximum(q, 0.0)
    outside_len = np.linalg.norm(outside_vec, axis=2)
    inside = np.minimum(np.max(q, axis=2), 0.0)
    sd = outside_len + inside
    n, b = sd.shape
    win = np.argmin(sd, axis=1) if b else np.zeros(n, dtype=int)
    rows = np.arange(n)
    sign = np.sign(rel)
    sign = np.where(sign == 0.0, 1.0, sign)
    grad = np.zeros((n, 3), dtype=float)
    if b:
        olen = outside_len[rows, win]
        out = olen > 1e-12
        if np.any(out):
            grad[out] = (outside_vec[rows, win][out] / olen[out, None]) * sign[rows, win][out]
        inn = ~out
        if np.any(inn):
            axis = np.argmax(q[rows, win][inn], axis=1)
            grad[np.flatnonzero(inn), axis] = sign[rows, win][inn, axis]
    g_all = np.zeros((n, b, 3), dtype=float)
    if b:
        g_all[rows, win] = grad
    return sd, g_all


def _nearest_packed(points: np.ndarray, packed: _PackedObstacles, active_limit: float | None):
    n = len(points)
    best = np.full(n, np.inf, dtype=float)
    grad = np.zeros_like(points)
    if packed.n == 0 and not packed.other:
        return best, grad
    lo = points.min(axis=0)
    hi = points.max(axis=0)
    # 铰链只需门槛内的障碍。求全局最小距离时先用 2 m 宽相，不够再扩。
    if active_limit is not None and np.isfinite(active_limit):
        radii = [float(active_limit)]
    else:
        radii = [_CULL_CLEARANCE_M, np.inf]

    def merge(radius: float):
        b = np.full(n, np.inf, dtype=float)
        g = np.zeros_like(points)
        if packed.up_i.size:
            lb = _aabb_cyl_lb(lo, hi, packed.up_xy, packed.up_r, packed.up_zmid, packed.up_half)
            keep = _keep_mask(lb, radius)
            if np.any(keep):
                sd, gg = _upright_sd_grad(
                    points, packed.up_xy[keep], packed.up_r[keep], packed.up_zmid[keep], packed.up_half[keep]
                )
                b, g = _scatter_min(b, g, sd, gg, packed.up_i[keep], radius)
        if packed.cyl_i.size:
            lb = _aabb_seg_cyl_lb(lo, hi, packed.cyl_c, packed.cyl_ax, packed.cyl_h, packed.cyl_r)
            keep = _keep_mask(lb, radius)
            if np.any(keep):
                sd, gg = _general_cyl_sd_grad(
                    points, packed.cyl_c[keep], packed.cyl_ax[keep], packed.cyl_h[keep], packed.cyl_r[keep]
                )
                b, g = _scatter_min(b, g, sd, gg, packed.cyl_i[keep], radius)
        if packed.deg_i.size:
            rel = points[:, None, :] - packed.deg_a[None, :, :]
            dist = np.linalg.norm(rel, axis=2)
            sd = dist - packed.deg_r
            hat = rel / np.maximum(dist, 1e-12)[:, :, None]
            b, g = _scatter_min(b, g, sd, hat, packed.deg_i, radius)
        if packed.box_i.size:
            lb = _aabb_box_lb(lo, hi, packed.box_c, packed.box_h)
            keep = _keep_mask(lb, radius)
            if np.any(keep):
                sd, gg = _box_sd_grad(points, packed.box_c[keep], packed.box_h[keep])
                b, g = _scatter_min(b, g, sd, gg, packed.box_i[keep], radius)
        if packed.grd_i.size:
            sd = points[:, 2:3] - packed.grd_z[None, :]
            gg = np.zeros((n, packed.grd_z.shape[0], 3), dtype=float)
            gg[:, :, 2] = 1.0
            b, g = _scatter_min(b, g, sd, gg, packed.grd_i, radius)
        for _i, ob in packed.other:
            sd = np.array([float(ob.distance(p)) for p in points], dtype=float)[:, None]
            gg = np.zeros((n, 1, 3), dtype=float)
            b, g = _scatter_min(b, g, sd, gg, np.array([_i], dtype=int), radius)
        return b, g

    for radius in radii:
        best, grad = merge(radius)
        # 每个点都要有一个半径内的见证，才能保证该点的最近障碍没被宽相丢掉。
        # 只要全局最小小于半径，只能保证 min(sd) 本身，不够给每个点用。
        if np.all(np.isfinite(best) & (best < radius)):
            return best, grad
    return best, grad


def _nearest(points: np.ndarray, obstacles, packed=None, active_limit: float | None = None):
    if not obstacles:
        return np.full(len(points), np.inf, dtype=float), np.zeros_like(points)
    if packed is None:
        packed = _pack_obstacles(obstacles)
    if packed is None:
        return np.full(len(points), np.inf, dtype=float), np.zeros_like(points)
    return _nearest_packed(points, packed, active_limit)


def _sdf_obstacle(points: np.ndarray, ob):
    if isinstance(ob, Cylinder):
        return _sdf_cylinder(points, ob)
    if isinstance(ob, Box):
        return _sdf_box(points, ob)
    if isinstance(ob, Ground):
        return _sdf_ground(points, ob)
    sd = np.array([float(ob.distance(p)) for p in points], dtype=float)
    return sd, np.zeros_like(points)


def _sdf_ground(points: np.ndarray, ground: Ground):
    sd = points[:, 2] - float(ground.z)
    grad = np.zeros_like(points)
    grad[:, 2] = 1.0
    return sd, grad


def _sdf_box(points: np.ndarray, box: Box):
    lo = np.asarray(box.lo, dtype=float)
    hi = np.asarray(box.hi, dtype=float)
    center = 0.5 * (lo + hi)
    half = 0.5 * (hi - lo)
    rel = points - center
    q = np.abs(rel) - half
    outside_vec = np.maximum(q, 0.0)
    outside_len = np.linalg.norm(outside_vec, axis=1)
    inside = np.minimum(np.max(q, axis=1), 0.0)
    sd = outside_len + inside
    sign = np.sign(rel)
    sign = np.where(sign == 0.0, 1.0, sign)
    grad = np.zeros_like(points)
    out = outside_len > 1e-12
    if np.any(out):
        grad[out] = (outside_vec[out] / outside_len[out, None]) * sign[out]
    inn = ~out
    if np.any(inn):
        rows = np.nonzero(inn)[0]
        axis = np.argmax(q[inn], axis=1)
        grad[rows, axis] = sign[rows, axis]
    return sd, grad


def _sdf_cylinder(points: np.ndarray, cyl: Cylinder):
    a, b = cyl.axis_ends()
    ab = b - a
    length = float(np.linalg.norm(ab))
    radius = float(cyl.radius)
    if length < 1e-9:
        rel = points - a
        dist = np.linalg.norm(rel, axis=1)
        hat = rel / np.maximum(dist, 1e-12)[:, None]
        return dist - radius, hat
    axis = ab / length
    center = 0.5 * (a + b)
    half = 0.5 * length
    rel = points - center
    axial = rel @ axis
    radial_vec = rel - axial[:, None] * axis
    radial = np.linalg.norm(radial_vec, axis=1)
    d_rad = radial - radius
    d_ax = np.abs(axial) - half
    outside_len = np.hypot(np.maximum(d_rad, 0.0), np.maximum(d_ax, 0.0))
    inside = np.minimum(np.maximum(d_rad, d_ax), 0.0)
    sd = outside_len + inside

    hat = np.zeros_like(points)
    nz = radial > 1e-9
    hat[nz] = radial_vec[nz] / radial[nz, None]
    if np.any(~nz):
        aux = np.array([1.0, 0.0, 0.0]) if abs(float(axis[0])) < 0.9 else np.array([0.0, 1.0, 0.0])
        perp = np.cross(axis, aux)
        perp = perp / np.linalg.norm(perp)
        hat[~nz] = perp
    sign_ax = np.where(axial >= 0.0, 1.0, -1.0)
    g_rad = np.zeros(len(points), dtype=float)
    g_ax = np.zeros(len(points), dtype=float)
    corner = (d_rad > 0.0) & (d_ax > 0.0)
    side = (d_rad > 0.0) & (d_ax <= 0.0)
    cap = (d_rad <= 0.0) & (d_ax > 0.0)
    inner = (d_rad <= 0.0) & (d_ax <= 0.0)
    olen = np.maximum(outside_len, 1e-12)
    g_rad[corner] = d_rad[corner] / olen[corner]
    g_ax[corner] = sign_ax[corner] * d_ax[corner] / olen[corner]
    g_rad[side] = 1.0
    g_ax[cap] = sign_ax[cap]
    inner_side = inner & (d_rad >= d_ax)
    g_rad[inner_side] = 1.0
    g_ax[inner & ~inner_side] = sign_ax[inner & ~inner_side]
    grad = g_rad[:, None] * hat + g_ax[:, None] * axis
    return sd, grad
