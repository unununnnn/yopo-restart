"""规划策略。规划器只拿 Observation：深度帧 + 本机里程计 + 目标 + 仿真时间，没有任何场景几何。

策略开关（POLICIES）：
- `hover`：票 01 悬停。
- `upstream`：上游等价（YOPO-MINCO test_yopo_ros 规则）：从期望状态出发，候选按预测走廊（0.05 m）
  准入、取最高分；无准入候选时本帧悬停、下帧照常重规划。不经任何安全门。
- `certified`：现行全部门：vendored `UavRuntime` + `Planner` + `SafetyGate`，参数同 L1 G2（g2_run.py）。
- `student`：D5 学生网络。默认不选。只有 ``--policy student``（或同等显式选择）才构造网络、读权重。
两条路径共用 vendored 的解码（decode_params / minco_two_piece / corridor_mu_lo / select_action）和推理。
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field

import numpy as np

from .backend.config import config_from_params
from .dynamics import HoverPlan, Plan, State
from .sensor import DepthFrame


@dataclass(frozen=True)
class Observation:
    frame: DepthFrame | None
    state: State  # 里程计（L0 为真值）
    goal: np.ndarray
    now: float

    def digest(self) -> str:
        h = hashlib.sha256()
        h.update(np.float64(self.now).tobytes())
        h.update(np.asarray(self.goal, dtype=np.float64).tobytes())
        s = self.state
        for x in (s.p, s.v, s.a):
            h.update(np.asarray(x, dtype=np.float64).tobytes())
        h.update(np.float64(s.yaw).tobytes())
        if self.frame is not None:
            h.update(np.float64(self.frame.stamp).tobytes())
            h.update(np.asarray(self.frame.K, dtype=np.float64).tobytes())
            h.update(np.ascontiguousarray(self.frame.depth).tobytes())
        return h.hexdigest()


@dataclass(frozen=True)
class PolicyContext:
    """策略构造参数：只含本机身份、时钟步长和场景卡里的策略参数。

    ``obstacles`` / ``a_max`` 只给后段开关。开关关掉时策略不读它们。
    """

    owner: str
    physics_dt: float
    params: dict = field(default_factory=dict)
    obstacles: tuple = ()
    a_max: float = 6.0


def R_yaw(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class HoverPolicy:
    """票 01：在目标点悬停（悬停任务里目标 = 悬停点）。"""

    name = "hover"

    def __init__(self, ctx: PolicyContext):
        self._plan: Plan | None = None
        self.last_code = "HOVER"

    def plan(self, obs: Observation) -> Plan:
        if self._plan is None:
            self._plan = HoverPlan(tuple(float(x) for x in obs.goal), obs.state.yaw)
        return self._plan


class _TrajPlan:
    """跟踪一条 PiecewisePolynomial，结束后在终点静止；偏航按上游 control_pub 用 calculate_yaw 逐 tick 推进。"""

    def __init__(self, traj, goal, yaw0: float, dt: float, brake: bool = False):
        self.traj, self.goal, self.yaw, self.dt, self.brake = traj, np.asarray(goal, float), yaw0, dt, brake

    def pva(self, t: float):
        tr = self.traj
        tt = min(max(t, tr.t_start), tr.t_end)
        p, v, a = (np.asarray(tr.evaluate(tt, k), dtype=float) for k in range(3))
        if t > tr.t_end:
            v, a = np.zeros(3), np.zeros(3)
        return p, v, a

    def sample(self, t: float):
        from yopo_swarm_core.yaw import calculate_yaw

        p, v, a = self.pva(t)
        gd = self.goal - p
        vel_dir = gd if self.brake else v
        self.yaw = float(calculate_yaw(vel_dir, gd, self.yaw, self.dt)[0])
        return p, v, a, self.yaw


# 上游 test_yopo_ros.py 的 safe_radius。非这个值就是偏离 d5e1e60，结果表要标出来。
UPSTREAM_SAFE_RADIUS_M = 0.05
_EMPTY_SOLUTION = ("CORRIDOR_EMPTY_HOVER", "BRAKE_ONLY", "CONFLICT_BRAKE")


def upstream_decoder(safe_radius: float | None = None):
    """复制解码器只改准入半径。默认返回原来的 UPSTREAM_DECODER，金样比较不受影响。"""
    from dataclasses import replace

    from yopo_swarm_core.candidates import UPSTREAM_DECODER

    radius = UPSTREAM_SAFE_RADIUS_M if safe_radius is None else float(safe_radius)
    if not radius > 0.0:
        raise ValueError(f"safe_radius 必须为正，收到 {safe_radius}")
    if abs(radius - float(UPSTREAM_DECODER.safe_radius)) <= 1e-12:
        return UPSTREAM_DECODER
    return replace(UPSTREAM_DECODER, safe_radius=radius)


def read_safe_radius(params: dict | None) -> float:
    raw = ((params or {}).get("upstream") or {}).get("safe_radius", UPSTREAM_SAFE_RADIUS_M)
    radius = float(raw)
    if not radius > 0.0:
        raise ValueError(f"safe_radius 必须为正，收到 {raw}")
    return radius


def read_empty_brake(params: dict | None) -> bool:
    """E-B4b：`policy_params.upstream.empty_brake`。缺省 false。打开即偏离上游 SPEC（无解时原地悬停）。"""
    raw = ((params or {}).get("upstream") or {}).get("empty_brake", False)
    if raw is not True and raw is not False:
        raise ValueError(f"empty_brake 必须是布尔值，收到 {raw!r}")
    return raw


# E-C5b：走廊跟踪误差补偿。默认关。0.08 m 盖的是 E-C5 已观测的 0.058–0.073 m 跟踪差，是假设。
# 机体半径与场景 criteria.body_radius_m、geom_fallback 的表面净空同一口径（中心到表面）。
TRACK_CLEARANCE_MODES = ("pred", "geom", "geom_multi")
TRACK_MARGIN_DEFAULT_M = 0.08
TRACK_BODY_RADIUS_M = 0.35
# 与 geom_fallback.SAMPLE_DT_S 相同。参考只在这些采样点上量表面净空，不取连续最小值。
TRACK_SAMPLE_DT_S = 0.02


def _upstream_block(params: dict | None) -> dict:
    raw = (params or {}).get("upstream") or {}
    if not isinstance(raw, dict):
        raise ValueError(f"policy_params.upstream 必须是对象，收到 {raw!r}")
    return raw


def read_track_clearance(params: dict | None) -> bool:
    """E-C5b：`policy_params.upstream.track_clearance`。缺省 false。非 JSON 布尔（含缺省以外的字符串）报错。"""
    raw = _upstream_block(params).get("track_clearance", False)
    if raw is not True and raw is not False:
        raise ValueError(f"track_clearance 必须是布尔值，收到 {raw!r}")
    return raw


def read_track_clearance_mode(params: dict | None) -> str:
    """`"pred" | "geom" | "geom_multi"`，缺省 `geom_multi`。调用方只在开关打开时读它。"""
    raw = _upstream_block(params).get("track_clearance_mode", "geom_multi")
    if raw not in TRACK_CLEARANCE_MODES:
        raise ValueError(f"track_clearance_mode 必须是 {TRACK_CLEARANCE_MODES} 之一，收到 {raw!r}")
    return str(raw)


def read_track_margin_m(params: dict | None) -> float:
    """非负。缺省 0.08 m。负数、非有限值、布尔值报错。"""
    raw = _upstream_block(params).get("track_margin_m", TRACK_MARGIN_DEFAULT_M)
    if isinstance(raw, bool) or raw is None:
        raise ValueError(f"track_margin_m 必须为非负实数，收到 {raw!r}")
    value = float(raw)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"track_margin_m 必须为非负，收到 {raw!r}")
    return value


def read_track_body_radius_m(params: dict | None) -> float:
    """表面净空里的机体半径。缺省 0.35 m，与场景卡 body_radius_m 对齐。必须为正。"""
    raw = _upstream_block(params).get("track_body_radius_m", TRACK_BODY_RADIUS_M)
    if isinstance(raw, bool) or raw is None:
        raise ValueError(f"track_body_radius_m 必须为正，收到 {raw!r}")
    value = float(raw)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"track_body_radius_m 必须为正，收到 {raw!r}")
    return value


def track_clearance_wants_neighbors(params: dict | None) -> bool:
    """geom_multi 要邻机采样，才能知道本帧是不是多机。其它情况 False，peer_plan 载荷不变。"""
    if not read_track_clearance(params):
        return False
    return read_track_clearance_mode(params) == "geom_multi"


class _ObstacleIndex:
    """ctx.obstacles 的表面净空。直立圆柱 / 地面 / 盒子与 scene.Obstacle.distance 同一公式。"""

    def __init__(self, obstacles):
        from .scene import Box, Cylinder, Ground

        centers, radii, z_lo, z_hi = [], [], [], []
        self.grounds: list[float] = []
        self.boxes: list[Box] = []
        self.others = []
        for ob in obstacles:
            if isinstance(ob, Cylinder) and ob.upright():
                centers.append(ob.center_xy)
                radii.append(ob.radius)
                z_lo.append(ob.z_lo)
                z_hi.append(ob.z_hi)
            elif isinstance(ob, Ground):
                self.grounds.append(float(ob.z))
            elif isinstance(ob, Box):
                self.boxes.append(ob)
            else:
                self.others.append(ob)
        if centers:
            self.center = np.asarray(centers, dtype=np.float64)
            self.radius = np.asarray(radii, dtype=np.float64)
            self.z_lo = np.asarray(z_lo, dtype=np.float64)
            self.z_hi = np.asarray(z_hi, dtype=np.float64)
        else:
            self.center = None

    def min_clearance(self, points: np.ndarray) -> float:
        pts = np.asarray(points, dtype=np.float64)
        if pts.ndim == 1:
            pts = pts.reshape(1, 3)
        if pts.shape[0] == 0:
            return math.inf
        best = math.inf
        if self.center is not None:
            radial = np.hypot(
                pts[:, None, 0] - self.center[None, :, 0],
                pts[:, None, 1] - self.center[None, :, 1],
            ) - self.radius
            radial = np.maximum(radial, 0.0)
            dz = np.maximum(np.maximum(self.z_lo - pts[:, None, 2], 0.0), pts[:, None, 2] - self.z_hi)
            best = min(best, float(np.hypot(radial, dz).min()))
        for z in self.grounds:
            best = min(best, float(np.maximum(pts[:, 2] - z, 0.0).min()))
        for ob in self.boxes:
            lo = np.asarray(ob.lo, dtype=np.float64)
            hi = np.asarray(ob.hi, dtype=np.float64)
            delta = np.maximum(np.maximum(lo - pts, 0.0), pts - hi)
            best = min(best, float(np.linalg.norm(delta, axis=1).min()))
        for ob in self.others:
            for p in pts:
                best = min(best, float(ob.distance(p)))
        return float(best)


def trajectory_sample_points(traj, dt: float = TRACK_SAMPLE_DT_S) -> np.ndarray:
    """从 t_start 到 t_end（含两端）按 dt 采样位置。步长与几何回退相同。"""
    from .geom_fallback import iter_times

    times = iter_times(float(traj.t_start), float(traj.t_end), float(dt))
    return np.stack(
        [np.asarray(traj.evaluate(float(t), 0), dtype=np.float64) for t in times],
        axis=0,
    )


def reference_surface_clearance(traj, obstacles, dt: float = TRACK_SAMPLE_DT_S) -> float:
    """参考轨迹采样点到障碍表面的最小距离。没有障碍时为 +inf。"""
    return _ObstacleIndex(obstacles).min_clearance(trajectory_sample_points(traj, dt))


def deviates_from_upstream(radius: float) -> bool:
    return abs(float(radius) - UPSTREAM_SAFE_RADIUS_M) > 1e-9


def empty_solution_count(plan_codes: dict | None) -> int:
    """无解后的悬停或刹车次数。不含正常的 OK / YOPO_ALT。"""
    codes = plan_codes or {}
    return sum(int(codes.get(name, 0)) for name in _EMPTY_SOLUTION)


class UpstreamPolicy:
    """参数（场景卡 policy_params.upstream）：
    - plan_from_reference（默认 True，上游注释「用位置控制器飞时设 True」；L0/L1 都跟踪位置设定点）
    - safe_radius：走廊准入半径，默认 0.05 m，与上游 d5e1e60 一致。0.35 m 是票 07d 的可选修法。
    - track_clearance（E-C5b，默认 false）：跟踪误差补偿。关掉时不读 ctx.obstacles（backend 另开除外），
      不改解码器，也不改 select_action 的结果。打开即偏离上游。
      - mode `pred`：准入解码器半径改为 safe_radius + track_margin_m。不看几何。
      - mode `geom`：0.05 m 走廊准入之后，按分数从高到低检查参考轨迹（0.02 s 采样到 t_end）
        对障碍表面的最小净空，要求 ≥ track_body_radius_m（默认 0.35）+ track_margin_m（默认 0.08）。
        最高分不合格就看下一个准入候选。全部被拒则走原来的无解分支。
      - mode `geom_multi`（默认）：与 geom 相同，但只在本帧 `_neighbors` 非空时启用。
        单机没有邻机，几何门保持关闭。多机第 0 个规划拍也没有邻机：runner 先 plan 再 bus.poll，
        本拍广播要到下一拍规划才进来，这一拍不启几何门。
      与 E-A1 的顺序：先走廊准入，再几何净空，得到的可行集才交给 `_apply_inter`。
      几何门滤空，或邻机过滤再滤空，都进 `CORRIDOR_EMPTY_HOVER`（empty_brake 开着则从参考头刹停）。
    - arrive_radius_m / arrive_mode：上游进入到达半径（上游 5 m）后停止重规划，`hold_here` 原地保持（上游原样）。
      上游网络不会在目标点减速停住（训练目标是远处目标，goal 截断到 10 m），按 G2 oracle 0.4 m / 0.2 m/s / 1 s
      判到达时几乎总是绕圈超时，所以加一条项目终端规则 `stop_at_goal`：进入半径后用一段五次多项式从期望状态
      停到目标点。默认 stop_at_goal + 1.5 m（待 frank 确认；hold_here 的数字一并报告）。
    """

    name = "upstream"
    DEFAULTS = {
        "plan_from_reference": True,
        "arrive_radius_m": 1.5,
        "arrive_mode": "stop_at_goal",
        "stop_time_s": 2.0,
        "safe_radius": UPSTREAM_SAFE_RADIUS_M,
    }

    def __init__(self, ctx: PolicyContext):
        from .net import InProcessSource

        self.ctx = ctx
        self.p = {**self.DEFAULTS, **ctx.params.get("upstream", {})}
        self.source = InProcessSource()
        self.plan_obj: _TrajPlan | None = None
        self.arrived = False
        self.last_code = ""
        # 缺省关。关掉时不读障碍，也不改这条多项式。
        self._backend = config_from_params(ctx.params, a_max=float(ctx.a_max))
        self.backend_log = [] if self._backend is not None else None
        self._obstacles = tuple(ctx.obstacles) if self._backend is not None else ()
        self._swarm = self._backend is not None and self._backend.swarm_penalty
        # 默认关。关时 _inter 为 None，plan() 不读邻机、不改 select_action 的结果。
        from .inter_drone import inter_drone_config

        self._inter = inter_drone_config(ctx.params)
        # E-B4b：默认关。关时无解分支仍是上游的原地悬停。
        self._empty_brake = read_empty_brake(ctx.params)
        # E-C5b：默认关。关时不读障碍，下面三个数只是占位，plan() 不会用。
        self._track_on = read_track_clearance(ctx.params)
        self._track_mode = "geom_multi"
        self._track_margin = TRACK_MARGIN_DEFAULT_M
        self._track_body = TRACK_BODY_RADIUS_M
        self._track_index: _ObstacleIndex | None = None
        if self._track_on:
            self._track_mode = read_track_clearance_mode(ctx.params)
            self._track_margin = read_track_margin_m(ctx.params)
            self._track_body = read_track_body_radius_m(ctx.params)
            if self._track_mode in ("geom", "geom_multi"):
                self._track_index = _ObstacleIndex(ctx.obstacles)
        self._neighbors: dict[str, np.ndarray] = {}
        self._inter_filtered = 0
        self._inter_penalized = 0
        self._inter_empty = 0

    def plan(self, obs: Observation) -> Plan:
        from yopo_swarm_core.candidates import (
            corridor_mu_lo,
            decode_params,
            minco_two_piece,
            select_action,
        )
        from yopo_swarm_core.plan import hold_traj

        if self._track_on and self._track_mode == "pred":
            # 只抬准入阈值。safe_radius 结果字段仍是场景卡上的数，不把 margin 写回去。
            dec = upstream_decoder(float(self.p["safe_radius"]) + float(self._track_margin))
        else:
            dec = upstream_decoder(self.p["safe_radius"])

        s, now = obs.state, obs.now
        yaw = s.yaw if self.plan_obj is None else self.plan_obj.yaw
        if self.arrived or np.linalg.norm(s.p - obs.goal) < self.p["arrive_radius_m"]:
            # 上游 arrive：不再重规划，保持（只在第一次到达时锁定保持点）
            if not self.arrived:
                self.arrived = True
                if self.p["arrive_mode"] == "hold_here":
                    traj = hold_traj(s.p, now, now + 1e-3)
                else:
                    from yopo_swarm_core.plan import quintic
                    from yopo_swarm_core.trajectory import PiecewisePolynomial

                    head = np.array(self.plan_obj.pva(now)) if self.plan_obj else np.array([s.p, s.v, np.zeros(3)])
                    end = np.array([np.asarray(obs.goal, float), np.zeros(3), np.zeros(3)])
                    traj = PiecewisePolynomial(now, [quintic(head, end, float(self.p["stop_time_s"]))])
                self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=True)
            self.last_code = "ARRIVED_HOLD"
            return self.plan_obj
        if self.plan_obj is None:  # 上游：首帧用里程计初始化期望状态
            head = np.array([s.p, s.v, np.zeros(3)])
        else:
            head = np.array(self.plan_obj.pva(now))
            if not self.p["plan_from_reference"]:
                head[0], head[1] = s.p, s.v
        R = R_yaw(s.yaw)  # 上游用里程计姿态做机体系
        params, score, radius = self.source.forward_raw(obs.frame.depth, head[1], head[2], obs.goal, head[0], R)
        inner_b, tail_b, durations = decode_params(params, dec)
        inner_w = inner_b @ R.T + head[0]
        tail_w = np.einsum("ij,nkj->nki", R, tail_b)
        tail_w[:, 0] += head[0]
        action, brake = select_action(score, inner_w, corridor_mu_lo(radius, dec), dec.safe_mu)
        # 先准入（pred 时阈值已经抬高），再几何净空，最后才是邻机过滤。几何门关着时不进下面这一支。
        geom_ok = None
        if not brake and self._track_geom_now():
            action, brake, geom_ok = self._apply_track_geom(
                score, radius, dec, now, head, tail_w, inner_w, durations
            )
        if not brake and self._inter is not None and self._neighbors:
            action, brake = self._apply_inter(
                score, radius, dec, now, head, tail_w, inner_w, durations, action, brake, geom_ok=geom_ok
            )
        if brake:
            # SPEC：无准入候选时本帧悬停（停在当前位置），下帧照常重规划
            self.last_code = "CORRIDOR_EMPTY_HOVER"
            traj = None
            if self._empty_brake:
                # E-B4b（偏离上游）：不把参考拽回当前位置，从参考头按 a_max 刹停（与 D1 回退同一个函数）。
                from .geom_fallback import brake_from_head

                traj = brake_from_head(head, now, float(self.ctx.a_max))
                if traj is None:
                    traj = hold_traj(head[0], now, now + 1e-3)
            if traj is None:
                traj = hold_traj(s.p, now, now + 1e-3)
        else:
            self.last_code = "OK"
            traj = minco_two_piece(now, head, tail_w[action], inner_w[action], durations[action])
            if self._backend is not None:
                traj = self._apply_backend(traj, obs.goal)
        self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=brake)
        return self.plan_obj

    def _apply_backend(self, traj, goal):
        from .backend.bspline import refine_yopo

        neighbors = list(self._neighbors.values()) if self._swarm else None
        out, rec = refine_yopo(traj, self._backend, self._obstacles, goal, neighbors=neighbors)
        self.backend_log.append(rec)
        return out

    def _track_geom_now(self) -> bool:
        """本帧要不要做参考表面净空。关、pred、以及 geom_multi 没有邻机时都不做。"""
        if not self._track_on or self._track_index is None:
            return False
        if self._track_mode == "geom":
            return True
        if self._track_mode == "geom_multi":
            # 单机：runner 不会投递 peer_plan，_neighbors 一直空，0.05 m 走廊保持原样。
            # 多机第 0 个规划拍：本 tick 的广播要等 plan() 全部返回后才 bus.poll，这里仍空。
            # 下一拍规划（planner 10 Hz 时是 t = 0.1 s）起，邻机采样已经在，几何门才开。
            return bool(self._neighbors)
        return False

    def _keep_neighbor_samples(self) -> bool:
        if self._inter is not None or self._swarm:
            return True
        return bool(self._track_on and self._track_mode == "geom_multi")

    def _apply_track_geom(self, score, radius, dec, now, head, tail_w, inner_w, durations):
        """准入候选按分数从高到低看表面净空。没有合格的就交给无解分支。"""
        from yopo_swarm_core.candidates import minco_two_piece, rank_candidates

        ranking = rank_candidates(np.asarray(score), radius, dec)
        n = int(np.asarray(score).shape[0])
        geom_ok = np.zeros(n, dtype=bool)
        if ranking.corridor_empty or self._track_index is None:
            fallback = int(np.argmax(np.asarray(score)))
            return fallback, True, geom_ok
        limit = float(self._track_body) + float(self._track_margin)
        # 邻机过滤还要在这份可行集上继续筛，所以有邻机时不能在第一名合格后就停。
        keep_rest = self._inter is not None and bool(self._neighbors)
        first = None
        for i in ranking.order:
            try:
                traj_i = minco_two_piece(now, head, tail_w[int(i)], inner_w[int(i)], durations[int(i)])
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                continue
            clearance = self._track_index.min_clearance(trajectory_sample_points(traj_i))
            if clearance + 1e-12 < limit:
                continue
            geom_ok[int(i)] = True
            if first is None:
                first = int(i)
            if not keep_rest:
                break
        if first is None:
            return int(ranking.order[0]), True, geom_ok
        return first, False, geom_ok

    def note_neighbor_samples(self, src: str, samples) -> None:
        """总线上邻机已提交轨迹的采样。E-A1、E-B3 都关、且不是 geom_multi 时丢掉，不进状态。"""
        if not self._keep_neighbor_samples():
            return
        from .inter_drone import as_sample_array

        arr = as_sample_array(samples)
        if arr is not None:
            self._neighbors[str(src)] = arr

    def inter_drone_report(self) -> dict | None:
        if self._inter is None:
            return None
        from .inter_drone import report_block

        return report_block(self._inter, self._inter_filtered, self._inter_penalized, self._inter_empty)

    def _apply_inter(self, score, radius, dec, now, head, tail_w, inner_w, durations, action, brake, geom_ok=None):
        """阈值内的候选过滤或减分。没有可行候选时回到原来的悬停分支。

        ``geom_ok`` 是 E-C5b 已经按表面净空留过的可行集。None 时与以前一样，只看走廊准入。
        """
        from yopo_swarm_core.candidates import corridor_mu_lo, minco_two_piece

        from .inter_drone import clearance_m, inter_pick, sample_motion

        mu = corridor_mu_lo(radius, dec)
        admitted = np.asarray(mu, dtype=np.float64) >= np.float64(dec.safe_mu)
        if geom_ok is not None:
            mask = np.asarray(geom_ok, dtype=bool)
            if mask.shape != admitted.shape:
                raise ValueError("geom_ok 与候选数不一致")
            admitted = admitted & mask
        clear = np.full(int(dec.count), np.inf)
        neighbors = list(self._neighbors.values())
        for i in np.flatnonzero(admitted):
            try:
                traj_i = minco_two_piece(now, head, tail_w[i], inner_w[i], durations[i])
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                continue
            clear[i] = clearance_m(sample_motion(traj_i, now), neighbors)
        pick = inter_pick(score, admitted, clear, self._inter)
        if pick.kind == "unchanged":
            return action, brake
        if pick.kind == "empty":
            # 和走廊为空同一条悬停。不在这里造一条绕开邻机的曲线。
            self._inter_empty += 1
            self._inter_filtered += pick.filtered
            return action, True
        self._inter_filtered += pick.filtered
        self._inter_penalized += pick.penalized
        return int(pick.action), False


class _RuntimePlan:
    """certified：每个物理 tick 向 UavRuntime 取命令（与 L1 的 TrajectorySetpoint 同语义）。"""

    def __init__(self, policy: CertifiedPolicy, hold: np.ndarray, yaw0: float):
        self.policy, self.hold, self.yaw0 = policy, np.asarray(hold, float), yaw0
        self.last = (self.hold, np.zeros(3), np.zeros(3), yaw0)

    def sample(self, t: float):
        rt = self.policy.runtime
        cmd = rt.command(t, self.yaw0, goal_w=self.policy.goal)
        if cmd is not None:
            p, v, a = (np.asarray(x, dtype=float) for x in cmd.pva_enu)
            self.last = (p, v, a, float(rt.last_yaw if rt.last_yaw is not None else self.yaw0))
        elif rt.current is None:
            self.last = (self.hold, np.zeros(3), np.zeros(3), self.yaw0)  # 尚无批准：原地保持
        return self.last


class CertifiedPolicy:
    name = "certified"
    DEFAULTS = {"body_radius": 0.35, "tracking_margin": 0.15, "switch_delay": 2.0, "start_pad_radius": 1.2}

    def __init__(self, ctx: PolicyContext):
        from yopo_swarm_core.planner import Planner
        from yopo_swarm_core.uav import UavRuntime, default_config

        from .net import InProcessSource

        p = {**self.DEFAULTS, **ctx.params.get("certified", {})}
        self.pad = float(p["start_pad_radius"])
        self.cfg = default_config(ctx.owner, p["body_radius"], p["tracking_margin"], p["switch_delay"])
        self.source = InProcessSource()
        self.planner = Planner(self.cfg, self.source)
        self.runtime = UavRuntime(self.planner)
        self.goal: np.ndarray | None = None
        self.plan_obj: _RuntimePlan | None = None
        self.last_code = ""
        self.extra_codes: dict[str, int] = {}

    def plan(self, obs: Observation) -> Plan:
        from yopo_swarm_core.depth import DepthSample, optical_pose
        from yopo_swarm_core.freespace import CameraIntrinsics
        from yopo_swarm_core.planner import VehicleState

        s, now, rt = obs.state, obs.now, self.runtime
        self.goal = np.asarray(obs.goal, float)
        if self.plan_obj is None:
            self.plan_obj = _RuntimePlan(self, s.p, s.yaw)
        f = obs.frame
        h, w = f.depth.shape
        R = R_yaw(s.yaw)
        k = CameraIntrinsics(*f.K, width=w, height=h)
        raw = f.depth.astype(np.float32)
        frame = DepthSample(raw, k, optical_pose(s.p, R), int(round(f.stamp * 1e9))).evidence(self.cfg.space)
        state = VehicleState(s.p.copy(), s.v.copy(), s.a.copy(), R, s.t)
        if rt.current is None:
            if rt.pending is None:
                ok = rt.bootstrap(frame, state, now, self.pad)
                self.last_code = "BOOTSTRAP" if ok else f"BOOTSTRAP_FAIL:{rt.last_error}"
            else:
                self.last_code = "WAIT_SWITCH"
            return self.plan_obj
        o = rt.observation(frame, state, self.goal, now)
        if o is None:
            self.last_code = "WAIT_SWITCH"
            return self.plan_obj
        self.source.bind_depth(raw, frame.stamp)
        parent = o.approved.plan_hash(self.cfg.plan)
        d = self.planner.step(o)
        rt.accept(d, parent, now)
        self.last_code = d.diagnostic.value
        for c in d.candidates:  # 候选漏斗：每个被拒候选的原因
            key = "cand:" + c.code
            self.extra_codes[key] = self.extra_codes.get(key, 0) + 1
        return self.plan_obj



class StudentPolicy:
    """D5：用多头学生网络给出一条连续轨迹。只有显式选 ``student`` 时才会构造。

    参数（policy_params.student，或环境变量 YOPO_STUDENT_WEIGHTS）：
    - weights：检查点路径，仓库外。缺了就拒绝构造。
    - max_range_m：深度归一化，默认 20.0，和教师转换一致。
    - arrive_radius_m / arrive_mode / stop_time_s：与上游相同的终端规则，不是网络学出来的。

    D5b 接口选项。默认值就是 D5 的行为，不传时逐位不变：
    - knots：``"network"``（D5，结点速度取速度头）或 ``"position"``（结点速度由位置结点中心差分得到，与位置自洽）。
    - yaw_source：``"calculate_yaw"``（D5，上游 YOPO 的速度/目标混合，0.5π rad/s）或
      ``"ego_lookahead"``（教师 EGO-Swarm traj_server 的规则：朝 1.0 s 后的轨迹点，位移 ≤ 0.1 m 时保持，π rad/s 限速）。
    - accel_source：``"state"``（D5，里程计加速度）或 ``"reference"``（当前计划在此刻的参考加速度；
      教师记录的加速度是 fake drone 写入的指令加速度）。
    - vertical：``"network"``（D5，高度取网络位置头的 z）或 ``"hold_goal_z"``（低维动作：网络只管水平，
      结点高度按时间常数 vertical_tau_s 指数趋近目标高度）。教师 EGO-Swarm 场景是定高飞行，网络的 z 头没有学到可用的高度控制。
    """

    name = "student"
    DEFAULTS = {
        "weights": None,
        "max_range_m": 20.0,
        "arrive_radius_m": 1.5,
        "arrive_mode": "stop_at_goal",
        "stop_time_s": 2.0,
        "knots": "network",
        "yaw_source": "calculate_yaw",
        "accel_source": "state",
        "vertical": "network",
        "vertical_tau_s": 1.0,
    }
    KNOTS = ("network", "position")
    YAW_SOURCES = ("calculate_yaw", "ego_lookahead")
    ACCEL_SOURCES = ("state", "reference")
    VERTICALS = ("network", "hold_goal_z")

    def __init__(self, ctx: PolicyContext):
        import os

        self.ctx = ctx
        self.p = {**self.DEFAULTS, **ctx.params.get("student", {})}
        weights = self.p.get("weights") or os.environ.get("YOPO_STUDENT_WEIGHTS")
        if not weights:
            raise ValueError("学生策略需要 policy_params.student.weights 或环境变量 YOPO_STUDENT_WEIGHTS")
        from yopo_l0.student import load_checkpoint

        for key, allowed in (("knots", self.KNOTS), ("yaw_source", self.YAW_SOURCES), ("accel_source", self.ACCEL_SOURCES),
                             ("vertical", self.VERTICALS)):
            if self.p[key] not in allowed:
                raise ValueError(f"policy_params.student.{key} 只能是 {allowed}，收到 {self.p[key]!r}")
        self.model, self.meta = load_checkpoint(weights, enabled=True)
        self.plan_obj: _TrajPlan | None = None
        self.arrived = False
        self.last_code = ""
        self._neighbors: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.infer_s: list[float] = []
        self.weights_sha256 = str(self.meta.get("weights_sha256") or "")

    def note_neighbor_odom(self, src: str, odom) -> None:
        """总线上邻机当前里程计。缺字段就丢掉这一条。"""
        if not isinstance(odom, dict) or "p" not in odom or "v" not in odom:
            return
        p = np.asarray(odom["p"], dtype=np.float64).reshape(3)
        v = np.asarray(odom["v"], dtype=np.float64).reshape(3)
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(v))):
            return
        self._neighbors[str(src)] = (p.copy(), v.copy())

    def infer_report(self) -> dict | None:
        if not self.infer_s:
            return {
                "n": 0,
                "mean_ms": None,
                "p50_ms": None,
                "p95_ms": None,
                "max_ms": None,
                "weights_sha256": self.weights_sha256,
            }
        arr = np.asarray(self.infer_s, dtype=np.float64) * 1e3
        return {
            "n": int(arr.size),
            "mean_ms": float(arr.mean()),
            "p50_ms": float(np.percentile(arr, 50)),
            "p95_ms": float(np.percentile(arr, 95)),
            "max_ms": float(arr.max()),
            "weights_sha256": self.weights_sha256,
        }

    def plan(self, obs: Observation) -> Plan:
        import time as time_mod

        from yopo_swarm_core.plan import hold_traj, quintic
        from yopo_swarm_core.trajectory import PiecewisePolynomial

        from yopo_l0.student import (
            HORIZON_DT_S,
            HORIZON_STEPS,
            depth_from_hw,
            pack_ego_state,
            pack_neighbor_features,
            predict,
        )

        s, now = obs.state, obs.now
        yaw = s.yaw if self.plan_obj is None else self.plan_obj.yaw
        if self.arrived or np.linalg.norm(s.p - obs.goal) < self.p["arrive_radius_m"]:
            if not self.arrived:
                self.arrived = True
                if self.p["arrive_mode"] == "hold_here":
                    traj = hold_traj(s.p, now, now + 1e-3)
                else:
                    head = np.array(self.plan_obj.pva(now)) if self.plan_obj else np.array([s.p, s.v, np.zeros(3)])
                    end = np.array([np.asarray(obs.goal, float), np.zeros(3), np.zeros(3)])
                    traj = PiecewisePolynomial(now, [quintic(head, end, float(self.p["stop_time_s"]))])
                self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=True)
            self.last_code = "ARRIVED_HOLD"
            return self.plan_obj
        if obs.frame is None:
            self.last_code = "NO_DEPTH_HOVER"
            traj = hold_traj(s.p, now, now + 1e-3)
            self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=True)
            return self.plan_obj

        depth = depth_from_hw(obs.frame.depth, float(self.p["max_range_m"]))
        accel = s.a
        if self.p["accel_source"] == "reference" and self.plan_obj is not None:
            accel = np.asarray(self.plan_obj.pva(now)[2], dtype=float)
        state = pack_ego_state(s.p, s.v, accel, obs.goal, s.yaw)
        if self._neighbors:
            ids = sorted(self._neighbors)
            nb_p = np.stack([self._neighbors[i][0] for i in ids], axis=0)
            nb_v = np.stack([self._neighbors[i][1] for i in ids], axis=0)
            feat, mask = pack_neighbor_features(s.p, s.v, s.yaw, nb_p, nb_v, k=self.model.cfg.n_neighbors)
        else:
            feat, mask = pack_neighbor_features(
                s.p, s.v, s.yaw, np.zeros((0, 3)), np.zeros((0, 3)), k=self.model.cfg.n_neighbors
            )
        t0 = time_mod.perf_counter()
        pred = predict(self.model, depth, state, feat, mask)
        self.infer_s.append(time_mod.perf_counter() - t0)
        if pred is None:
            self.last_code = "STUDENT_DISABLED_HOVER"
            traj = hold_traj(s.p, now, now + 1e-3)
            self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=True)
            return self.plan_obj

        rot = R_yaw(s.yaw)
        p_b = np.asarray(pred.position_body[0], dtype=np.float64)
        v_b = np.asarray(pred.velocity_body[0], dtype=np.float64)
        if p_b.shape != (HORIZON_STEPS, 3) or v_b.shape != (HORIZON_STEPS, 3) or not (
            np.all(np.isfinite(p_b)) and np.all(np.isfinite(v_b))
        ):
            self.last_code = "STUDENT_BAD_TRAJ_HOVER"
            traj = hold_traj(s.p, now, now + 1e-3)
            self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=True)
            return self.plan_obj
        p_w = s.p + (rot @ p_b.T).T
        if self.p["vertical"] == "hold_goal_z":
            p_w[:, 2], vz = student_vertical_hold(float(s.p[2]), float(np.asarray(obs.goal, float)[2]), HORIZON_STEPS,
                                                  float(HORIZON_DT_S), float(self.p["vertical_tau_s"]))
        if self.p["knots"] == "position":
            v_w = student_knot_velocities(np.asarray(s.p, float), p_w, float(HORIZON_DT_S))
        else:
            v_w = (rot @ v_b.T).T
            if self.p["vertical"] == "hold_goal_z":
                v_w[:, 2] = vz
        head = np.array([np.asarray(s.p, float), np.asarray(s.v, float), np.zeros(3)])
        segs = []
        prev = head
        try:
            for i in range(HORIZON_STEPS):
                end = np.array([p_w[i], v_w[i], np.zeros(3)])
                segs.append(quintic(prev, end, float(HORIZON_DT_S)))
                prev = end
            traj = PiecewisePolynomial(now, segs)
        except (ValueError, np.linalg.LinAlgError, FloatingPointError):
            self.last_code = "STUDENT_FIT_HOVER"
            traj = hold_traj(s.p, now, now + 1e-3)
            self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=True)
            return self.plan_obj
        self.last_code = "OK"
        plan_cls = _EgoLookaheadPlan if self.p["yaw_source"] == "ego_lookahead" else _TrajPlan
        self.plan_obj = plan_cls(traj, obs.goal, yaw, self.ctx.physics_dt, brake=False)
        return self.plan_obj


def student_vertical_hold(z0: float, z_goal: float, steps: int, dt: float, tau_s: float):
    """结点高度和竖直速度：从当前高度按指数趋近目标高度。返回 (z (K,), vz (K,))。"""
    if not tau_s > 0.0:
        raise ValueError(f"vertical_tau_s 必须为正，收到 {tau_s}")
    t = dt * np.arange(1, int(steps) + 1, dtype=float)
    decay = np.exp(-t / float(tau_s))
    z = z_goal + (z0 - z_goal) * decay
    vz = (z_goal - z0) / float(tau_s) * decay
    return z, vz


def student_knot_velocities(p0: np.ndarray, p_w: np.ndarray, dt: float) -> np.ndarray:
    """位置结点 (K, 3) → 结点速度。内部结点用中心差分，末结点用后向差分。起点 p0 不在返回里。"""
    pts = np.vstack([np.asarray(p0, dtype=float).reshape(1, 3), np.asarray(p_w, dtype=float)])
    v = np.empty_like(pts[1:])
    v[:-1] = (pts[2:] - pts[:-2]) / (2.0 * dt)
    v[-1] = (pts[-1] - pts[-2]) / dt
    return v


class _EgoLookaheadPlan(_TrajPlan):
    """偏航按 EGO-Swarm traj_server（92fe9f7 ``calculate_yaw``）：朝 1.0 s 后的轨迹点，
    位移不超过 0.1 m 时保持上一偏航，变化率不超过 π rad/s。只给学生的 ``ego_lookahead`` 用。"""

    LOOKAHEAD_S = 1.0
    YAW_RATE_RAD_S = np.pi

    def sample(self, t: float):
        p, v, a = self.pva(t)
        tr = self.traj
        tf = min(max(t + self.LOOKAHEAD_S, tr.t_start), tr.t_end)
        d = np.asarray(tr.evaluate(tf, 0), dtype=float) - p
        if float(np.linalg.norm(d)) > 0.1:
            diff = (float(np.arctan2(d[1], d[0])) - self.yaw + np.pi) % (2.0 * np.pi) - np.pi
            step = self.YAW_RATE_RAD_S * self.dt
            self.yaw = float((self.yaw + float(np.clip(diff, -step, step)) + np.pi) % (2.0 * np.pi) - np.pi)
        return p, v, a, self.yaw


POLICIES = {"hover": HoverPolicy, "upstream": UpstreamPolicy, "certified": CertifiedPolicy, "student": StudentPolicy}
