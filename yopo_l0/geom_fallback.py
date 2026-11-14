"""票 D1：提交前的一次性几何回退。默认关。

关着的时候不采样、不往总线上加消息、不写结果字段。开着的时候，在新轨迹
被提交之前沿时间采样一次，对照两件事：

- 场景卡上已有的环境几何，按机体半径膨胀（中心到表面 ≤ body_radius 算冲突，
  与仿真真值碰撞同一口径）。
- 邻机已经广播在总线上的已提交轨迹采样。中心距 < 阈值算冲突。阈值写在场景卡
  ``geometric_fallback.inter_drone_distance_m``，缺省是 2 × body_radius。

只查一遍，不重规划、不换下一条候选。失败则：先前已提交的轨迹若仍落在承诺视界内
（现在时刻还早于它的提交时刻 + commit_horizon，并且还没飞出它的运动段）就继续飞它，
否则按场景动力学 a_max 刹停悬停。原因码和回退次数记在结果字段 ``geometric_fallback``，
检查本身的墙钟单独记 p50/p95。这个字段与 E-C3 的 ``plan_wall_ms`` 无关。

承诺视界沿用票 13a：有舰队时用舰队已经解析好的值（六机 1.75 s，三机卡上的 1.0 s
不抬高）。goto 没有窗口时按 N × 0.25 s + 0.25 s 取下界，不把六机的 1.75 s 写死到三机上。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from yopo_swarm_core.plan import KinematicLimits, state_at
from yopo_swarm_core.protocol.messages import Reason
from yopo_swarm_core.protocol.node import ProtocolNode

KIND = "geom_samples"
TOPIC = "geom_fallback"
FIELD = "geometric_fallback"
SAMPLE_DT_S = 0.02
# 与票 13a 的计划契约一样：只把 a_max 收到动力学，v_max / j_max 仍放宽。
_BRAKE_V_MAX = 20.0
_BRAKE_J_MAX = 20000.0

REASON_OBSTACLE = "GEOM_OBSTACLE"
REASON_INTER = "GEOM_INTER_DRONE"
ACTION_PREVIOUS = "PREVIOUS"
ACTION_BRAKE = "BRAKE"


def brake_from_head(head, now: float, a_max: float):
    """从 head = (p, v, a) 按 a_max 刹停的五次分支。D1 回退与 E-B4b 无解刹停共用。

    已经静止（速度、加速度都不超过 1e-6）或找不到可行分支时返回 None，调用方原地保持。
    """
    from yopo_swarm_core.plan import brake_branch

    _p, v, a = (np.asarray(row, dtype=float) for row in head)
    if float(np.linalg.norm(v)) > 1e-6 or float(np.linalg.norm(a)) > 1e-6:
        limits = KinematicLimits(v_max=_BRAKE_V_MAX, a_max=float(a_max), j_max=_BRAKE_J_MAX)
        return brake_branch(float(now), head, limits)
    return None


def _head_matrix(head) -> np.ndarray:
    rows = [np.asarray(row, dtype=float).reshape(3) for row in head]
    arr = np.vstack(rows)
    if arr.shape != (3, 3) or not bool(np.all(np.isfinite(arr))):
        raise ValueError("head 必须是有限的 (p, v, a)")
    return arr


def _poly_min_z(coeffs_z: np.ndarray, duration: float) -> float:
    """一段 z 多项式在 [0, duration] 上的最小值。端点、导数实根，再加 0.02 s 采样。"""
    from numpy.polynomial.polynomial import polyder, polyroots, polyval

    c = np.asarray(coeffs_z, dtype=float)
    duration = float(duration)
    best = min(float(polyval(0.0, c)), float(polyval(duration, c)))
    deriv = polyder(c)
    if deriv.size:
        for root in np.atleast_1d(polyroots(deriv)):
            if abs(float(np.imag(root))) > 1e-7:
                continue
            x = float(np.real(root))
            if -1e-9 <= x <= duration + 1e-9:
                best = min(best, float(polyval(min(max(x, 0.0), duration), c)))
    steps = max(int(math.ceil(duration / SAMPLE_DT_S)), 1)
    for k in range(steps + 1):
        tau = min(duration, k * SAMPLE_DT_S)
        best = min(best, float(polyval(tau, c)))
    return float(best)


def trajectory_min_z(traj) -> float:
    """整段轨迹采样（含导数根）的最低 z。"""
    best = math.inf
    for seg in traj.segments:
        best = min(best, _poly_min_z(np.asarray(seg.coeffs[2], dtype=float), float(seg.duration)))
    return float(best)


def _peak_accel(traj) -> float:
    best = 0.0
    for t in iter_times(float(traj.t_start), float(traj.t_end), min(SAMPLE_DT_S, 0.01)):
        acc = np.asarray(traj.evaluate(float(t), 2), dtype=float)
        best = max(best, float(np.linalg.norm(acc)))
    return float(best)


def _quintic_segment(head: np.ndarray, end: np.ndarray, duration: float):
    from yopo_swarm_core.plan import quintic

    return quintic(np.asarray(head, dtype=float), np.asarray(end, dtype=float), float(duration))


def _as_traj(now: float, segments):
    from yopo_swarm_core.trajectory import PiecewisePolynomial

    return PiecewisePolynomial(float(now), list(segments))


def _chain_ok(segments, floor: float, tol: float) -> bool:
    if not segments:
        return False
    traj = _as_traj(0.0, segments)
    return trajectory_min_z(traj) >= float(floor) - tol


def ground_safe_brake_from_head(head, now, a_max, min_z):
    """刹停不得下穿 ``min_z``。E-C5d，只在地面闸门打开时调用。

    先取常规 ``brake_from_head``。全段 z 都 ≥ ``min_z`` 就原样返回。
    原本静止且 ``head`` 的高度已经 ≥ ``min_z`` 时返回 None（与常规刹停一样，调用方原地保持）。
    刹停曲线会下穿，或一开始就低于 ``min_z``，则用五次多项式拉起：
    水平停在常规刹停的终点（来不及就停在就近的点），高度收到 ``max(刹停终点 z, min_z)``。
    总时长至少 ``max(0.5, 1.5 * 速率 / a_max)``。垂向先用不超过必要的加速度拉起，
    再在安全高度上把水平速度停掉。采样最低点不低于 ``min(起点 z, min_z)``
    （起点已在线下且垂速向下时，只留反转垂速 unavoidable 的那一点下探）。
    """
    head_m = _head_matrix(head)
    p, v, a = (head_m[i].copy() for i in range(3))
    now = float(now)
    a_max = float(a_max)
    min_z = float(min_z)
    if not math.isfinite(min_z) or not math.isfinite(a_max):
        raise ValueError(f"min_z 与 a_max 必须有限，收到 min_z={min_z!r} a_max={a_max!r}")
    a_lim = a_max if a_max > 1e-6 else 1e-6
    z0 = float(p[2])
    stationary = float(np.linalg.norm(v)) <= 1e-6 and float(np.linalg.norm(a)) <= 1e-6
    regular = brake_from_head(head_m, now, a_lim if a_max > 1e-6 else a_max)
    if stationary and z0 >= min_z - 1e-9:
        return None
    if (
        regular is not None
        and z0 >= min_z - 1e-9
        and trajectory_min_z(regular) >= min_z - 1e-8
    ):
        return regular

    speed = float(np.linalg.norm(v))
    t_min = max(0.5, 1.5 * speed / a_lim)
    if regular is not None:
        stop = np.asarray(regular.evaluate(float(regular.t_end), 0), dtype=float)
        xy_stop = stop[:2].copy()
        z_end = float(stop[2])
    else:
        z_end = z0
        xy_stop = p[:2] + v[:2] * (t_min / 2.0) + a[:2] * (t_min ** 2) / 12.0
    z_target = max(z_end, min_z)
    # 起点已低于 min_z 时，不能再往下；否则地板就是 min_z。
    floor = min_z if z0 >= min_z - 1e-9 else z0
    # 人已经在线下且还在往下掉时，五次必须先跟上这个垂速，最短斜坡会下探大约 |vz|·0.03。
    if z0 >= min_z - 1e-9 or float(v[2]) >= -1e-6:
        tol = 1e-6
    else:
        tol = min(0.03, max(1e-3, -float(v[2]) * 0.03))
    built = _pullup_brake(head_m, now, a_lim, floor, z_target, xy_stop, t_min, tol)
    if built is None:
        built = _pullup_brake(head_m, now, max(a_lim, _az_needed(z0, float(v[2]), floor)), floor, z_target, xy_stop, t_min, tol)
    if built is None:
        # 仍要给出一条不低于地板的五次拉起，水平就停在当前位置。
        built = _pullup_brake(
            head_m, now, max(a_lim, _az_needed(z0, float(v[2]), floor)), floor, max(z_target, floor, z0),
            p[:2].copy(), t_min, max(tol, 5e-3),
        )
    return built


def _az_needed(z0: float, vz: float, floor: float) -> float:
    room = float(z0) - float(floor)
    if vz >= -1e-9 or room <= 1e-4:
        return 0.0
    return float(vz * vz / (2.0 * room))


def _pullup_brake(head, now, a_lim, floor, z_target, xy_stop, t_min, tol):
    """垂向先拉起，再在安全高度上刹停水平。拉不起就返回 None。"""
    if float(np.linalg.norm(head[1])) <= 1e-6 and float(np.linalg.norm(head[2])) <= 1e-6:
        return _rest_climb(head, now, np.asarray(xy_stop, dtype=float), max(float(z_target), float(floor)), t_min, a_lim, floor, tol)
    segments, end = _vertical_rescue(head, a_lim, floor, z_target, tol)
    if segments is None:
        return None
    t_used = sum(float(seg.duration) for seg in segments)
    xy_goal = np.asarray(xy_stop, dtype=float).reshape(2)
    # 救援段水平是滑行。终点离得太远就改停在救援结束时再刹一段能停住的地方。
    coast_xy = end[0, :2]
    if float(np.linalg.norm(xy_goal - coast_xy)) > max(8.0, float(np.linalg.norm(end[1, :2])) * t_min + 1.0):
        xy_goal = coast_xy + end[1, :2] * max(0.5, t_min) / 2.0
    z_goal = max(float(z_target), float(floor))
    # 先把垂向加速度收到 0，高度留在不低于 z_goal 的地方，水平速度先留着。
    eased, end = _ease_vertical(segments, end, z_goal, floor, tol)
    if eased is None:
        return None
    segments = eased
    t_used = sum(float(seg.duration) for seg in segments)
    t_h = max(0.5, t_min - t_used, 1.5 * float(np.linalg.norm(end[1, :2])) / max(float(a_lim), 1e-6))
    horizontal = _horizontal_stop(segments, end, xy_goal, z_goal, floor, tol, t_h, a_lim)
    if horizontal is None:
        return None
    traj = _as_traj(now, horizontal)
    if trajectory_min_z(traj) < float(floor) - tol:
        return None
    return traj


def _rest_climb(head, now, xy, z_goal, t_min, a_lim, floor, tol):
    """静止时一段五次拉到目标高度。时长从下限往上加，直到加速度不超过 a_max。"""
    xy = np.asarray(xy, dtype=float).reshape(2)
    z_goal = max(float(z_goal), float(floor))
    duration = max(float(t_min), 0.5)
    span = abs(z_goal - float(head[0, 2])) + float(np.linalg.norm(xy - head[0, :2]))
    if span > 1e-8:
        # 五次静止到静止的峰值加速度大约是 5.8 * 距离 / T^2。
        duration = max(duration, math.sqrt(5.8 * span / max(float(a_lim), 1e-6)))
    best = None
    for _ in range(10):
        end = np.array(
            [[float(xy[0]), float(xy[1]), z_goal], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            dtype=float,
        )
        seg = _quintic_segment(head, end, duration)
        traj = _as_traj(now, [seg])
        if trajectory_min_z(traj) < float(floor) - tol:
            duration *= 1.25
            continue
        best = traj
        if _peak_accel(traj) <= float(a_lim) * 1.05 + 0.2:
            return traj
        duration *= 1.25
    return best


def _vertical_rescue(head, a_lim, floor, z_target, tol):
    """从当前垂速拉到 vz ≥ 0 且 z 不低于地板。试几档向上加速度，斜坡吃掉的高度也算进去。"""
    vz = float(head[1, 2])
    options = [float(a_lim)]
    if vz < -1e-9:
        need = _az_needed(float(head[0, 2]), vz, floor)
        for mul in (1.0, 1.4, 2.0, 3.0, 5.0, 8.0):
            options.append(max(float(a_lim), need * mul, float(a_lim) * mul))
    elif float(head[0, 2]) < float(z_target) - 1e-9:
        options.append(float(a_lim))
    seen = set()
    for a_up in options:
        key = round(a_up, 5)
        if key in seen:
            continue
        seen.add(key)
        got = _rescue_at(head, a_up, floor, z_target, tol)
        if got is not None and got[0] is not None:
            return got
    return None, None


def _rescue_at(head, a_up, floor, z_target, tol):
    a_cmd = np.array([0.0, 0.0, float(a_up)], dtype=float)
    segments = []
    state = head.copy()
    if float(np.linalg.norm(state[2] - a_cmd)) > 1e-6:
        ramp = _ramp_to_accel(state, a_cmd, floor, tol)
        if ramp is None:
            return None, None
        seg, state = ramp
        segments.append(seg)
    if float(state[1, 2]) < -1e-9 and a_up > 1e-8:
        hold_t = max(1e-3, -float(state[1, 2]) / a_up)
        nxt = _const_accel_state(state, a_cmd, hold_t)
        if float(nxt[0, 2]) < float(floor) - tol:
            return None, None
        segments.append(_quintic_segment(state, nxt, hold_t))
        state = nxt
    elif float(state[0, 2]) < float(z_target) - 1e-6 and a_up > 1e-8:
        # 已经没有向下的速度，但还在目标高度下面：继续以 a_up 拉一段再交给后面收平。
        need = float(z_target) - float(state[0, 2])
        # 0.5*a*t^2 + vz*t = need 的一个够用的时长
        vz_now = max(float(state[1, 2]), 0.0)
        hold_t = max(0.05, (-vz_now + math.sqrt(max(vz_now * vz_now + 2.0 * a_up * need, 0.0))) / a_up)
        nxt = _const_accel_state(state, a_cmd, hold_t)
        segments.append(_quintic_segment(state, nxt, hold_t))
        state = nxt
    if not _chain_ok(segments, floor, tol) and segments:
        return None, None
    return segments, state


def _ramp_to_accel(state, a_cmd, floor, tol):
    """短五次，把加速度收到 a_cmd。取仍不低于地板的最短斜坡。"""
    best = None
    for ramp_t in (0.02, 0.03, 0.04, 0.06, 0.08, 0.12, 0.18, 0.28, 0.4):
        a_avg = 0.5 * (state[2] + a_cmd)
        p2 = state[0] + state[1] * ramp_t + 0.5 * a_avg * ramp_t * ramp_t
        v2 = state[1] + a_avg * ramp_t
        nxt = np.vstack([p2, v2, np.asarray(a_cmd, dtype=float)])
        seg = _quintic_segment(state, nxt, ramp_t)
        if _poly_min_z(np.asarray(seg.coeffs[2], dtype=float), ramp_t) < float(floor) - tol:
            continue
        best = (seg, nxt)
        break
    return best


def _const_accel_state(state, a_cmd, duration: float) -> np.ndarray:
    a_cmd = np.asarray(a_cmd, dtype=float)
    duration = float(duration)
    p2 = state[0] + state[1] * duration + 0.5 * a_cmd * duration * duration
    v2 = state[1] + a_cmd * duration
    return np.vstack([p2, v2, a_cmd.copy()])


def _ease_vertical(segments, state, z_goal, floor, tol):
    """垂速、垂向加速度收到 0，高度收到不低于 z_goal。水平速度保持。"""
    z_now = float(state[0, 2])
    z_keep = max(float(z_goal), z_now, float(floor))
    # 若救援停在目标上方且垂向已经安静，不必再动高度。
    if abs(float(state[1, 2])) <= 1e-6 and abs(float(state[2, 2])) <= 1e-6 and z_now >= float(z_goal) - 1e-6:
        return list(segments), state
    best = None
    for duration in (0.2, 0.3, 0.45, 0.6, 0.8, 1.0, 1.4):
        xy = state[0, :2] + state[1, :2] * duration
        end = np.array(
            [
                [float(xy[0]), float(xy[1]), z_keep],
                [float(state[1, 0]), float(state[1, 1]), 0.0],
                [0.0, 0.0, 0.0],
            ],
            dtype=float,
        )
        seg = _quintic_segment(state, end, duration)
        trial = list(segments) + [seg]
        if _chain_ok(trial, floor, tol):
            best = (trial, end)
            break
    if best is None:
        return None, None
    return best


def _horizontal_stop(segments, state, xy_goal, z_goal, floor, tol, t_h, a_lim):
    """在已经安全的高度上把水平速度停到 xy_goal。拉长时长直到加速度可跟踪且不掉高度。"""
    z_hold = max(float(state[0, 2]), float(z_goal), float(floor))
    duration = max(float(t_h), 0.5)
    best = None
    for _ in range(16):
        end = np.array(
            [
                [float(xy_goal[0]), float(xy_goal[1]), z_hold],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
            dtype=float,
        )
        seg = _quintic_segment(state, end, duration)
        trial = list(segments) + [seg]
        # 只约束这一段水平刹停的加速度。前面的拉起为了不穿地，可以比 a_max 更陡。
        above = trajectory_min_z(_as_traj(0.0, trial)) >= float(floor) - tol
        if above and _peak_accel(_as_traj(0.0, [seg])) <= float(a_lim) * 1.05 + 0.5:
            return trial
        if above:
            best = trial
        duration *= 1.25
    return best


def _a_max(card: dict) -> float:
    dyn = card.get("dynamics") or {}
    if dyn.get("a_max") is None:
        from .dynamics import PointMassLag

        return float(PointMassLag.a_max)
    value = float(dyn["a_max"])
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"动力学 a_max 必须为正，收到 {dyn.get('a_max')}")
    return value


def _obstacles_of(card: dict):
    from .scene import parse_geometry

    geom = card.get("geometry")
    if not isinstance(geom, dict):
        if isinstance(card.get("forest"), dict):
            from .forest import generate_forest_geometry

            geom = generate_forest_geometry(card["forest"])
        else:
            geom = {}
    return parse_geometry(geom, card.get("ground_z"))


def _horizon_of(card: dict, n_agents: int, requested: float | None) -> float:
    from .crossing import resolve_commit_horizon_s

    task = card.get("task") or {}
    window = float(task.get("window_len_s", 0.25))
    if requested is None:
        raw = (card.get("geometric_fallback") or {}).get("commit_horizon_s")
        if raw is None:
            raw = task.get("commit_horizon_s", 1.0)
        requested = float(raw)
    return float(resolve_commit_horizon_s(float(requested), int(n_agents), window))


def iter_times(t0: float, t1: float, dt: float) -> list[float]:
    """含两端的均匀采样。t1 < t0 时退化为一个点。"""
    t0 = float(t0)
    t1 = float(t1)
    dt = float(dt)
    if not dt > 0.0:
        raise ValueError("采样步长必须为正")
    if t1 < t0:
        t1 = t0
    times = [t0]
    n = int(math.floor((t1 - t0) / dt + 1e-9))
    for i in range(1, n + 1):
        times.append(t0 + i * dt)
    if times[-1] < t1 - 1e-9:
        times.append(t1)
    return times


def sample_eval(eval_p, t0: float, t1: float, dt: float) -> list[list[float]]:
    out: list[list[float]] = []
    for t in iter_times(t0, t1, dt):
        p = np.asarray(eval_p(t), dtype=float)
        out.append([float(t), float(p[0]), float(p[1]), float(p[2])])
    return out


def interp_position(series: list[list[float]], t: float) -> np.ndarray | None:
    """在已广播的采样上线性插值。t 落在采样区间外就没有邻机信息，不补。"""
    if not series:
        return None
    if t < series[0][0] - 1e-9 or t > series[-1][0] + 1e-9:
        return None
    lo, hi = 0, len(series)
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    if lo <= 0:
        return np.asarray(series[0][1:], dtype=float)
    if lo >= len(series):
        return np.asarray(series[-1][1:], dtype=float)
    left, right = series[lo - 1], series[lo]
    span = right[0] - left[0]
    if span <= 1e-15:
        return np.asarray(right[1:], dtype=float)
    u = (t - left[0]) / span
    p0 = np.asarray(left[1:], dtype=float)
    p1 = np.asarray(right[1:], dtype=float)
    return (1.0 - u) * p0 + u * p1


def time_stats(samples_ms: list[float]) -> dict:
    if not samples_ms:
        return {"count": 0, "mean": None, "p50": None, "p95": None, "max": None}
    arr = np.asarray(samples_ms, dtype=float)
    return {
        "count": int(arr.size),
        "mean": float(np.mean(arr)),
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(np.max(arr)),
    }


class BrakeHover:
    """刹停后的跟踪对象。有多项式就沿它减速，否则停在 hold。sample 会推进偏航，pva 不会。"""

    def __init__(self, traj, goal, yaw: float, dt: float, hold: np.ndarray):
        self.traj = traj
        self.goal = np.asarray(goal, dtype=float)
        self.yaw = float(yaw)
        self.dt = float(dt)
        self.hold = np.asarray(hold, dtype=float)

    def pva(self, t: float):
        if self.traj is None:
            z = np.zeros(3)
            return self.hold.copy(), z, z.copy()
        tt = min(max(float(t), self.traj.t_start), self.traj.t_end)
        p, v, a = (np.asarray(self.traj.evaluate(tt, k), dtype=float) for k in range(3))
        if float(t) > self.traj.t_end:
            v = np.zeros(3)
            a = np.zeros(3)
        return p, v, a

    def sample(self, t: float):
        from yopo_swarm_core.yaw import calculate_yaw

        p, v, a = self.pva(t)
        self.yaw = float(calculate_yaw(v, self.goal - p, self.yaw, self.dt)[0])
        return p, v, a, self.yaw


@dataclass
class GeomSession:
    enabled: bool
    body_radius_m: float
    inter_drone_m: float
    horizon_s: float
    sample_dt_s: float
    a_max: float
    physics_dt: float
    members: tuple[str, ...]
    obstacles: tuple = ()
    checks: int = 0
    fallbacks: int = 0
    reason_counts: dict = field(default_factory=dict)
    action_counts: dict = field(default_factory=lambda: {ACTION_PREVIOUS: 0, ACTION_BRAKE: 0})
    _times_ms: list = field(default_factory=list)
    _heard: dict = field(default_factory=dict)
    _heard_seq: dict = field(default_factory=dict)
    _meta: dict = field(default_factory=dict)
    _plans: dict = field(default_factory=dict)
    _seq: dict = field(default_factory=dict)
    _outbox: list = field(default_factory=list)
    _overrides: dict = field(default_factory=dict)
    _pending: dict = field(default_factory=dict)

    @classmethod
    def from_card(
        cls,
        card: dict,
        *,
        members: tuple[str, ...],
        horizon: float | None = None,
        physics_dt: float = 0.01,
        obstacles=None,
    ) -> "GeomSession":
        raw = card.get("geometric_fallback") or {}
        enabled = bool(raw.get("enabled", False))
        body = float(card["criteria"]["body_radius_m"])
        if "inter_drone_distance_m" in raw:
            gap = float(raw["inter_drone_distance_m"])
            if not math.isfinite(gap) or gap <= 0.0:
                raise ValueError(f"inter_drone_distance_m 必须为正，收到 {gap}")
        else:
            gap = 2.0 * body
        if not enabled:
            # 不解析视界、不生成森林。缺省路径不能多打一条 13a 的抬高日志。
            return cls(
                enabled=False,
                body_radius_m=body,
                inter_drone_m=gap,
                horizon_s=0.0,
                sample_dt_s=SAMPLE_DT_S,
                a_max=_a_max(card),
                physics_dt=float(physics_dt),
                members=tuple(members),
            )
        if horizon is None:
            requested = raw.get("commit_horizon_s")
            horizon = _horizon_of(card, len(members), None if requested is None else float(requested))
        else:
            horizon = float(horizon)
        if obstacles is None:
            obstacles = _obstacles_of(card)
        return cls(
            enabled=True,
            body_radius_m=body,
            inter_drone_m=gap,
            horizon_s=float(horizon),
            sample_dt_s=SAMPLE_DT_S,
            a_max=_a_max(card),
            physics_dt=float(physics_dt),
            members=tuple(members),
            obstacles=tuple(obstacles),
        )

    def summary(self) -> dict:
        return {
            "enabled": True,
            "body_radius_m": self.body_radius_m,
            "inter_drone_distance_m": self.inter_drone_m,
            "commit_horizon_s": self.horizon_s,
            "sample_dt_s": self.sample_dt_s,
            "a_max": self.a_max,
            "checks": int(self.checks),
            "fallbacks": int(self.fallbacks),
            "reasons": {k: int(v) for k, v in sorted(self.reason_counts.items())},
            "actions": {
                ACTION_PREVIOUS: int(self.action_counts.get(ACTION_PREVIOUS, 0)),
                ACTION_BRAKE: int(self.action_counts.get(ACTION_BRAKE, 0)),
            },
            "check_time_ms": time_stats(self._times_ms),
        }

    def previous_valid(self, drone_id: str, now: float) -> bool:
        """已提交运动段还没飞完。

        段的长度在提交时至少盖住承诺视界（票 13a）。现在还落在这段里，就继续飞它。
        段已经结束，或还没有已提交轨迹，就不能再拿它当回退。不在段中途按
        committed_at + horizon 提前丢掉，否则动力学还在飞这条已提交曲线，下一次提案
        又从它的末端状态接，刹停和协议父计划会对不上。
        """
        meta = self._meta.get(drone_id)
        if meta is None:
            return False
        return float(now) < float(meta["t_end"]) - 1e-12

    def _motion_end(self, plan, now: float) -> float:
        traj = getattr(plan, "traj", None)
        if traj is not None and hasattr(traj, "t_end"):
            return float(traj.t_end)
        return float(now) + self.horizon_s

    def _remember(self, drone_id: str, plan, now: float) -> None:
        self._meta[drone_id] = {"t_end": self._motion_end(plan, now), "committed_at": float(now)}
        self._plans[drone_id] = plan

    def _enqueue(self, drone_id: str, samples: list[list[float]]) -> None:
        seq = int(self._seq.get(drone_id, 0)) + 1
        self._seq[drone_id] = seq
        self._outbox.append((drone_id, seq, samples))

    def flush(self, bus, now: float) -> None:
        if not self.enabled or not self._outbox:
            return
        pending = self._outbox
        self._outbox = []
        for src, seq, samples in pending:
            payload = {"kind": KIND, "owner": src, "seq": int(seq), "samples": samples}
            for dest in self.members:
                if dest == src:
                    continue
                bus.publish(now, src=src, dest=dest, topic=TOPIC, payload=payload)

    def ingest_envelope(self, env) -> None:
        payload = env.payload
        if payload.get("kind") != KIND:
            return
        owner = str(payload.get("owner", env.src))
        seq = int(payload.get("seq", 0))
        if seq < int(self._heard_seq.get(owner, -1)):
            return
        self._heard_seq[owner] = seq
        self._heard[owner] = list(payload.get("samples") or [])

    def _obstacle_hit(self, p: np.ndarray) -> bool:
        radius = self.body_radius_m
        point = np.asarray(p, dtype=float)
        for ob in self.obstacles:
            if ob.distance(point) <= radius + 1e-12:
                return True
        return False

    def _inter_hit(self, owner: str, t: float, p: np.ndarray) -> bool:
        limit = self.inter_drone_m
        point = np.asarray(p, dtype=float)
        for other, series in self._heard.items():
            if other == owner:
                continue
            q = interp_position(series, t)
            if q is None:
                continue
            if float(np.linalg.norm(point - q)) < limit:
                return True
        return False

    def classify(self, owner: str, samples: list[list[float]]) -> list[str]:
        """单遍。两种原因可以同时记下，但不会因此再采样一次。"""
        obstacle = False
        inter = False
        for row in samples:
            t = float(row[0])
            p = row[1:]
            if not obstacle and self._obstacle_hit(p):
                obstacle = True
            if not inter and self._inter_hit(owner, t, p):
                inter = True
            if obstacle and inter:
                break
        reasons = []
        if obstacle:
            reasons.append(REASON_OBSTACLE)
        if inter:
            reasons.append(REASON_INTER)
        return reasons

    def _timed_classify(self, owner: str, samples: list[list[float]]) -> list[str]:
        t0 = time.perf_counter()
        try:
            return self.classify(owner, samples)
        finally:
            self._times_ms.append((time.perf_counter() - t0) * 1000.0)
            self.checks += 1

    def _note_failure(self, reasons: list[str], action: str) -> None:
        self.fallbacks += 1
        for reason in reasons:
            self.reason_counts[reason] = int(self.reason_counts.get(reason, 0)) + 1
        self.action_counts[action] = int(self.action_counts.get(action, 0)) + 1

    def samples_of_plan(self, plan) -> list[list[float]]:
        motion = plan.motion
        t0 = float(motion.t_start)
        t1 = float(motion.t_end) + self.horizon_s

        def ev(t: float):
            return state_at(plan, t)[0]

        return sample_eval(ev, t0, t1, self.sample_dt_s)

    def samples_of_object(self, plan, now: float) -> list[list[float]]:
        traj = getattr(plan, "traj", None)
        if traj is not None and hasattr(traj, "t_start"):
            t0 = float(traj.t_start)
            t1 = float(traj.t_end) + self.horizon_s

            def ev(t: float):
                return plan.pva(t)[0]

            return sample_eval(ev, t0, t1, self.sample_dt_s)
        if hasattr(plan, "pva"):
            t0 = float(now)
            t1 = float(now) + self.horizon_s

            def ev2(t: float):
                return plan.pva(t)[0]

            return sample_eval(ev2, t0, t1, self.sample_dt_s)
        pos = np.asarray(getattr(plan, "position", getattr(plan, "hold", np.zeros(3))), dtype=float)
        t0 = float(now)
        t1 = float(now) + self.horizon_s
        return sample_eval(lambda _t: pos, t0, t1, self.sample_dt_s)

    def note_initial(self, node: ProtocolNode) -> None:
        plan = node.committed.occupancy.plan
        self._meta[node.id] = {"t_end": float(plan.motion.t_end), "committed_at": 0.0}
        self._enqueue(node.id, self.samples_of_plan(plan))

    def review_commit(self, node: ProtocolNode, now: float) -> str:
        if not self.enabled:
            return "accept"
        plan = node.in_flight.record.occupancy.plan
        samples = self.samples_of_plan(plan)
        self._pending[node.id] = samples
        reasons = self._timed_classify(node.id, samples)
        if not reasons:
            return "accept"
        if self.previous_valid(node.id, now):
            self._note_failure(reasons, ACTION_PREVIOUS)
            return "previous"
        self._note_failure(reasons, ACTION_BRAKE)
        return "brake"

    def on_accepted(self, node: ProtocolNode, now: float) -> None:
        plan = node.committed.occupancy.plan
        self._meta[node.id] = {"t_end": float(plan.motion.t_end), "committed_at": float(now)}
        self._overrides.pop(node.id, None)
        samples = self._pending.pop(node.id, None)
        if samples is None:
            samples = self.samples_of_plan(plan)
        self._enqueue(node.id, samples)

    def on_brake(self, node: ProtocolNode, now: float) -> None:
        alt = self._overrides.get(node.id)
        if alt is not None:
            p, v, a = alt.pva(now)
            head = np.array([p, v, a], dtype=float)
            yaw = float(alt.yaw)
        else:
            head = np.array(state_at(node.committed.occupancy.plan, float(now)), dtype=float)
            yaw = 0.0
        brake = self._brake_from_head(head, yaw, now)
        # 刹车不是一次协议提交。承诺视界仍按上一条已提交轨迹计，下一次失败不会把它当成「仍有效」。
        self._overrides[node.id] = brake
        self._enqueue(node.id, self.samples_of_object(brake, now))

    def override_sampler(self, drone_id: str):
        return self._overrides.get(drone_id)

    def _brake_from_head(self, head: np.ndarray, yaw: float, now: float) -> BrakeHover:
        p = np.asarray(head[0], dtype=float)
        traj = brake_from_head(head, now, self.a_max)
        return BrakeHover(traj, p, yaw, self.physics_dt, p)

    def review_goto(self, drone_id: str, proposed, now: float):
        """goto / hover：总线把本机令牌投递回来、即将写进 approved 之前调用。返回实际要飞的对象。"""
        if not self.enabled:
            return proposed
        samples = self.samples_of_object(proposed, now)
        reasons = self._timed_classify(drone_id, samples)
        if not reasons:
            self._remember(drone_id, proposed, now)
            self._enqueue(drone_id, samples)
            return proposed
        if self.previous_valid(drone_id, now):
            self._note_failure(reasons, ACTION_PREVIOUS)
            return self._plans[drone_id]
        self._note_failure(reasons, ACTION_BRAKE)
        brake = self._brake_from_object(proposed, now)
        self._remember(drone_id, brake, now)
        self._enqueue(drone_id, self.samples_of_object(brake, now))
        return brake

    def _brake_from_object(self, proposed, now: float) -> BrakeHover:
        if hasattr(proposed, "pva"):
            p, v, a = proposed.pva(now)
            head = np.array([p, v, a], dtype=float)
            yaw = float(getattr(proposed, "yaw", 0.0))
        else:
            p = np.asarray(getattr(proposed, "position", np.zeros(3)), dtype=float)
            head = np.array([p, np.zeros(3), np.zeros(3)], dtype=float)
            yaw = float(getattr(proposed, "yaw", 0.0))
        return self._brake_from_head(head, yaw, now)


class GuardedNode(ProtocolNode):
    """只在几何回退打开时替换 ProtocolNode。关着仍构造原来的节点。"""

    def __init__(self, *args, geom: GeomSession, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.geom = geom
        self._geom_now = 0.0

    def on_tick(self, now: float):
        self._geom_now = float(now)
        return super().on_tick(now)

    def on_message(self, msg, now: float):
        self._geom_now = float(now)
        return super().on_message(msg, now)

    def finalize_delay_check(self, now: float):
        self._geom_now = float(now)
        return super().finalize_delay_check(now)

    def _commit(self):
        action = self.geom.review_commit(self, self._geom_now)
        if action == "accept":
            msgs = super()._commit()
            self.geom.on_accepted(self, self._geom_now)
            return msgs
        if action == "brake":
            self.geom.on_brake(self, self._geom_now)
        return self._abort(Reason.CONFLICT)


def make_protocol_node(session: GeomSession, *args, **kwargs):
    """关：原样 ProtocolNode。开：提交前先做一次性几何检查。"""
    if not session.enabled:
        return ProtocolNode(*args, **kwargs)
    node = GuardedNode(*args, geom=session, **kwargs)
    session.note_initial(node)
    return node
