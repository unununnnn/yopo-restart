"""单机、无邻机的均匀三次 B 样条后段。

YOPO 已经选出一条多项式。这里把它拟合成均匀三次 B 样条，只动中间控制点，
用解析梯度下降一条 EGO 风格的代价：平滑、到场景圆柱 / 盒子 / 地面的最近距离、
速度和加速度超过上限。没有机间项。

不复制 ego-planner-swarm。那份代码是 GPLv3，本仓库根目录没有 LICENSE。
代价按论文的角色重写：平滑是控制点三阶差分，碰撞是米制有符号距离上的平方铰链，
可行是速度、加速度超过上限的平方铰链。论文用的是 {p, v} 平面距离和另一组权重，
数值对不上，所以权重写在下面，不假装是论文表里的 λ。

求解器用 scipy 的 L-BFGS-B，迭代上限和时间预算都是硬的。失败、超时后没有更安全的点、
或运动学守不住时，交回原来的多项式对象。
"""

from __future__ import annotations

import time
from collections import Counter

import numpy as np

from yopo_l0.scene import Box, Cylinder, Ground

from .config import BackendConfig

# 论文 λ_s=1、λ_c=0.5、λ_d=0.1。那里的碰撞不是米。
# 平方铰链以米计，λ_c 要大到能把分米级的侵入从树干上推开，同时锚住拟合形状。
_LAMBDA_S = 1.0
_LAMBDA_C = 40.0
_LAMBDA_D = 0.1
_LAMBDA_ANCHOR = 0.15
# 软惩罚比机体半径多出来的一截。铰链在门槛上梯度为零，留 5 cm 才推得动表面。
_CLEARANCE_BUFFER_M = 0.05
_MIN_DURATION_S = 0.6
_MIN_SPANS = 6
_MAX_SPANS = 12
_TARGET_DT_S = 0.22
# 拟合把五次多项式换成三次样条，净空允许掉这么多仍算「没有更差」。
_CLEARANCE_TOL_M = 0.01
# 计划契约的速度、急动度。加速度用场景 a_max。后段自己的速度软上限是 v_max=6。
_PLAN_V_MAX = 20.0
_PLAN_J_MAX = 20000.0
_SAMPLE_DT_S = 0.02

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


def refine_yopo(traj, cfg: BackendConfig, obstacles, goal=None):
    """优化一条已选中的 YOPO 多项式。失败时返回同一个对象。

    ``goal`` 留在签名里：末端 PVA 钉死在这条前缀上，本原型不另加终端进度。
    形状不合法就当这次失败，交回原曲线。
    """
    # 首次 import 不计入这次调用的时间预算。
    import scipy.optimize  # noqa: F401

    t0 = time.perf_counter()
    if goal is not None:
        g = np.asarray(goal, dtype=float)
        if g.shape != (3,) or not np.all(np.isfinite(g)):
            return traj, _record(False, "bad_goal", 0, t0, budget_cut=False)

    try:
        return _refine(traj, cfg, tuple(obstacles or ()), t0)
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


def _refine(traj, cfg: BackendConfig, obstacles, t0: float):
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
    free = np.arange(3, n_ctrl - 3)
    q_fit, rmse = _fit_free(traj, q_fixed, free, n_span, dt)
    if not np.all(np.isfinite(q_fit)):
        return traj, _record(False, "nonfinite", 0, t0, budget_cut=False, fit_rmse_m=rmse)

    d_safe = float(cfg.clearance_m) + _CLEARANCE_BUFFER_M
    idx, wts = _sample_layout(n_span)
    clr_before = min_signed_distance(traj, obstacles)

    def cost_grad(q):
        return _cost_grad(
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
        )

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
        if built is None or not _ends_match(built, (p0, v0, a0), (p1, v1, a1)):
            kin_fail += 1
            continue
        if not _kin_ok(built, float(cfg.a_max)):
            kin_fail += 1
            continue
        clr = min_signed_distance(built, obstacles)
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


def _dyn_samples(n_span: int):
    cached = []
    for span in range(n_span):
        for u in (0.0, 0.25, 0.5, 0.75):
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
    idx = []
    wts = []
    for span in range(n_span):
        us = (0.0, 0.25, 0.5, 0.75) if span < n_span - 1 else (0.0, 0.25, 0.5, 0.75, 1.0)
        for u in us:
            idx.append((span, span + 1, span + 2, span + 3))
            wts.append(_basis(u))
    return np.asarray(idx, dtype=int), np.asarray(wts, dtype=float)


def _fit_free(traj, q_fixed: np.ndarray, free: np.ndarray, n_span: int, dt: float):
    """中间控制点对 YOPO 采样做最小二乘。端点三元组不动。"""
    t0 = float(traj.t_start)
    t1 = float(traj.t_end)
    rows = []
    targets = []
    fixed_offset = []
    free_of = {int(i): k for k, i in enumerate(free)}
    for span in range(n_span):
        for u in (0.25, 0.5, 0.75):
            t = min(t0 + (span + u) * dt, t1)
            b = _basis(u)
            row = np.zeros(len(free), dtype=float)
            offset = np.zeros(3, dtype=float)
            for k in range(4):
                index = span + k
                if index in free_of:
                    row[free_of[index]] = b[k]
                else:
                    offset += b[k] * q_fixed[index]
            rows.append(row)
            targets.append(np.asarray(traj.evaluate(float(t)), dtype=float))
            fixed_offset.append(offset)
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


def _cost_grad(q, q_fit, free, dt, obstacles, d_safe, v_max, a_max, idx, wts):
    n = q.shape[0]
    grad = np.zeros_like(q)
    cost = 0.0

    for i in range(n - 3):
        diff = q[i + 3] - 3.0 * q[i + 2] + 3.0 * q[i + 1] - q[i]
        cost += _LAMBDA_S * float(diff @ diff)
        chain = _LAMBDA_S * 2.0 * diff
        grad[i] -= chain
        grad[i + 1] += 3.0 * chain
        grad[i + 2] -= 3.0 * chain
        grad[i + 3] += chain

    # 罚的是曲线上的速度和加速度，不是控制点差分。差分是凸包上界，弯一下就被判爆。
    for span, _u, bp, bpp in _dyn_samples(n - 3):
        seg = q[span : span + 4]
        vel = (bp @ seg) / dt
        acc = (bpp @ seg) / (dt * dt)
        cost += _add_limit_grad(grad, span, vel, v_max, bp, 1.0 / dt)
        cost += _add_limit_grad(grad, span, acc, a_max, bpp, 1.0 / (dt * dt))

    for index in free:
        delta = q[index] - q_fit[index]
        cost += _LAMBDA_ANCHOR * float(delta @ delta)
        grad[index] += _LAMBDA_ANCHOR * 2.0 * delta

    if obstacles:
        pts = _curve_points(q, idx, wts)
        sd, sd_grad = _nearest(pts, obstacles)
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
    from scipy.optimize import minimize

    state = {
        "best_x": np.array(x0, dtype=float, copy=True),
        "best_c": None,
        "evals": 0,
        "cut": False,
    }
    deadline = t0 + float(cfg.time_budget_s)

    def fun(x):
        if state["evals"] > 0 and time.perf_counter() > deadline:
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

    res = minimize(
        fun,
        np.array(x0, dtype=float, copy=True),
        method="L-BFGS-B",
        jac=True,
        options={
            "maxiter": int(cfg.max_iterations),
            "ftol": 1e-12,
            "gtol": 1e-6,
            "maxls": 16,
            "maxcor": 8,
        },
    )
    iterations = int(getattr(res, "nit", 0) or 0)
    return state["best_x"], iterations, bool(state["cut"])


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


def _ends_match(traj, start, end) -> bool:
    got0 = _pva(traj, float(traj.t_start))
    got1 = _pva(traj, float(traj.t_end))
    for got, want in ((got0, start), (got1, end)):
        for a, b in zip(got, want, strict=True):
            if float(np.max(np.abs(a - b))) > 1e-6:
                return False
    return True


def _kin_ok(traj, a_max: float) -> bool:
    from yopo_swarm_core.collision import Verdict, check_derivative_bound

    limits = ((1, _PLAN_V_MAX), (2, float(a_max)), (3, _PLAN_J_MAX))
    return all(check_derivative_bound(traj, order, lim).verdict is Verdict.SAFE for order, lim in limits)


def min_signed_distance(traj, obstacles, step: float = _SAMPLE_DT_S) -> float:
    """轨迹上到障碍表面的最小有符号距离。没有障碍时是正无穷。穿进表面为负。"""
    if not obstacles:
        return float("inf")
    span = float(traj.t_end - traj.t_start)
    n = max(2, int(np.ceil(span / step)) + 1)
    times = np.linspace(float(traj.t_start), float(traj.t_end), n)
    pts = np.stack([np.asarray(traj.evaluate(float(t)), dtype=float) for t in times], axis=0)
    sd, _grad = _nearest(pts, obstacles)
    return float(np.min(sd))


def _nearest(points: np.ndarray, obstacles):
    best = np.full(len(points), np.inf, dtype=float)
    grad = np.zeros_like(points)
    for ob in obstacles:
        sd, g = _sdf_obstacle(points, ob)
        take = sd < best
        best = np.where(take, sd, best)
        grad = np.where(take[:, None], g, grad)
    return best, grad


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
