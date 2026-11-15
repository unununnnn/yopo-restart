"""影子盾：只记录「本来会不会拒」，不改批准动作。

每个规划拍刷新一次选择门，每个物理 tick 再记 oracle。检查器只读计划，
不调用 ``plan.sample``（``_TrajPlan.sample`` / ``UavRuntime.command`` 会推进航向和时钟）。

接上的 vendored 门：

- 安全证据：``DepthSample.evidence`` + ``FreeSpace.classify_sweep`` / ``planner._env_code``。
  起飞区 ``assume_free`` 与 ``CertifiedPolicy`` 的 start pad 同一半径，不调用 ``certify_sweep``
  （证书会把后面的 UNKNOWN 改成 FREE）。
- 计划契约：``make_plan`` / ``validate_plan``。上游多项式按原样送进去；悬停命令走 ``hover_plan``。
- STOP_SELECT：core 里没有这个符号。等价物是 ``SafetyGate.validate``（不过就回退到刹车）。
  只在影子自己拿得到父计划时调用。certified 的 runtime 只留 current、不留父 Plan，
  对它再调 SafetyGate 会造出假的 ``PARENT_MISMATCH``，所以那条路径只复验契约和环境。
- oracle 跟踪 / 落后：界取 ``FlightOracle`` 默认值，逐 tick 重算。不调用 ``update``
  （会锁存，并把碰撞写进 oracle）。碰撞仍是运行器硬失败。

邻机冲突要等票 05；单机 peers 为空，门照常跑。

``scene_obstacles is None``（默认）时不读场景几何，输出与这条路径改前相同。
传了障碍之后才把场景卡光线投射和净空证书写进影子自己的 ``FreeSpace``，见 ``shadow_env``。
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from yopo_swarm_core.depth import DepthSample, optical_pose
from yopo_swarm_core.freespace import CameraIntrinsics, FreeSpace, SpaceVerdict
from yopo_swarm_core.oracle import FlightOracle
from yopo_swarm_core.plan import Plan, PlanReject, hover_plan, make_plan, state_at, validate_plan
from yopo_swarm_core.planner import ApprovedPlan, GateReject, SafetyGate, _env_code
from yopo_swarm_core.trajectory import PiecewisePolynomial
from yopo_swarm_core.uav import default_config

from .policy import CertifiedPolicy, R_yaw

_oracle = FlightOracle(boxes=(), start=np.zeros(3), goal=np.zeros(3), start_time=0.0)
ORACLE_TRACKING_BOUND = float(_oracle.tracking_bound)
ORACLE_LAG_LIMIT = float(_oracle.lag_limit)
ORACLE_REFERENCE_SPEED = float(_oracle.reference_speed)
ORACLE_MAX_AGE = float(_oracle.max_age)

ENV_NO_FRAME = "ENV:NO_FRAME"
ENV_NONFINITE = "ENV:NONFINITE"
ENV_OCCUPIED = "ENV:OCCUPIED"
ENV_UNKNOWN = "ENV:UNKNOWN"
TRACKING_BOUND = "TRACKING_BOUND"
LAG_VIOLATION = "LAG_VIOLATION"
ODOMETRY_AGE_OR_NONFINITE = "ODOMETRY_AGE_OR_NONFINITE"

SHADOW_RESULT_KEYS = frozenset({
    "shadow",
    "shadow_reject_rate",
    "shadow_reject_histogram",
    "shadow_gates",
    "shadow_drones",
})
SHADOW_TICK_KEYS = frozenset({"would_reject", "reject_codes"})


def approved_pva(plan, t: float):
    """本 tick 动力学要跟踪的位置/速度/加速度。不调用 sample / command。"""
    if hasattr(plan, "pva"):
        return plan.pva(float(t))
    if hasattr(plan, "position"):
        z = np.zeros(3)
        return np.asarray(plan.position, dtype=float), z, z
    rt = getattr(getattr(plan, "policy", None), "runtime", None)
    if rt is not None and rt.current is not None:
        p, v, a = state_at(rt.current.plan, float(t), rt.active_branch)
        return np.asarray(p, float), np.asarray(v, float), np.asarray(a, float)
    hold = getattr(plan, "hold", None)
    z = np.zeros(3)
    if hold is not None:
        return np.asarray(hold, dtype=float), z, z
    return z, z.copy(), z.copy()


def _vec3(x) -> np.ndarray | None:
    try:
        a = np.asarray(x, dtype=float)
    except (TypeError, ValueError):
        return None
    if a.shape != (3,):
        return None
    return a


def _transition(reasons) -> tuple[str, ...]:
    return tuple(f"TRANSITION:{r.value}" for r in reasons)


def _sweep(space: FreeSpace, traj: PiecewisePolynomial, now: float) -> tuple[str, ...]:
    res = space.classify_sweep(traj, now)
    if res.verdict is SpaceVerdict.OCCUPIED:
        return (ENV_OCCUPIED,)
    if res.verdict is SpaceVerdict.UNKNOWN:
        return (ENV_UNKNOWN,)
    return ()


def _env_of(space: FreeSpace, plan: Plan, now: float) -> tuple[str, ...]:
    code = _env_code(space, plan, now)
    return () if code is None else (code,)


@dataclass(frozen=True)
class GateInfo:
    gate: str
    wired: bool
    impl: str
    note: str


GATES = (
    GateInfo(
        "env_evidence",
        True,
        "FreeSpace.classify_sweep",
        "DepthSample.evidence 进独立 FreeSpace；起飞区 assume_free 用 CertifiedPolicy.start_pad_radius。不写证书。",
    ),
    GateInfo(
        "stop_select",
        True,
        "SafetyGate.validate",
        "core 无 STOP_SELECT 符号。上游/hover 对影子父计划调用 SafetyGate；certified 只留 current，跳过父计划复查。",
    ),
    GateInfo(
        "plan_contract",
        True,
        "validate_plan",
        "上游轨迹走 make_plan；悬停走 hover_plan；已批准的 certified Plan 走 validate_plan。",
    ),
    GateInfo(
        "oracle_track_lag",
        True,
        "record_oracle_track_lag",
        "界取 FlightOracle 默认值。不调用 update（锁存，且会把碰撞记进 oracle）。",
    ),
)


def gate_status() -> list[dict]:
    return [{"gate": g.gate, "wired": g.wired, "impl": g.impl, "note": g.note} for g in GATES]


def record_oracle_track_lag(now, stamp, actual, desired, start, goal, start_time) -> tuple[str, ...]:
    """与 FlightOracle.update 的跟踪/落后公式相同（ground_z 为空）。不判碰撞、不锁存。"""
    actual_v, desired_v = _vec3(actual), _vec3(desired)
    start_v, goal_v = _vec3(start), _vec3(goal)
    try:
        now_f, stamp_f, t0 = float(now), float(stamp), float(start_time)
    except (TypeError, ValueError):
        return (ODOMETRY_AGE_OR_NONFINITE,)
    if (
        actual_v is None
        or desired_v is None
        or start_v is None
        or goal_v is None
        or not bool(np.all(np.isfinite([now_f, stamp_f, t0, *actual_v, *desired_v, *start_v, *goal_v])))
        or not 0.0 <= now_f - stamp_f <= ORACLE_MAX_AGE
    ):
        return (ODOMETRY_AGE_OR_NONFINITE,)
    delta = goal_v - start_v
    distance = float(np.linalg.norm(delta))
    direction = delta / max(distance, 1e-12)
    progress = float(np.dot(actual_v - start_v, direction))
    lag = float(min(distance, max(0.0, now_f - t0) * ORACLE_REFERENCE_SPEED) - progress)
    error = float(np.linalg.norm(actual_v - desired_v))
    codes: list[str] = []
    if error > ORACLE_TRACKING_BOUND:
        codes.append(TRACKING_BOUND)
    if lag > ORACLE_LAG_LIMIT:
        codes.append(LAG_VIOLATION)
    return tuple(codes)


class DroneShadow:
    """一架机自己的证据缓冲和父计划链。不碰策略、不碰动力学。"""

    def __init__(self, owner: str, start: np.ndarray, goal: np.ndarray, params: dict, scene_obstacles=None):
        cert = {**CertifiedPolicy.DEFAULTS, **params.get("certified", {})}
        self.cfg = default_config(
            owner, cert["body_radius"], cert["tracking_margin"], cert["switch_delay"],
        )
        self.owner = owner
        self.start = np.asarray(start, dtype=float).copy()
        self.goal = np.asarray(goal, dtype=float).copy()
        self.space = FreeSpace(self.cfg.space)
        self.pad = float(cert["start_pad_radius"])
        if self.pad > 0.0:
            self.space.assume_free(self.start, self.pad, "explicit-start-pad")
        self._scene = None
        if scene_obstacles:
            # 只在研究开关打开时进入。关着不导入、不换 FreeSpace，起飞区证书仍是上面那一份。
            from .shadow_env import SceneEvidence

            self._scene = SceneEvidence(tuple(scene_obstacles), self.start, self.goal)
            kept = list(self.space.assumptions)
            wider = replace(self.cfg.space, max_frames=self.cfg.space.max_frames + self._scene.n_views)
            self.space = FreeSpace(wider)
            self.space.assumptions.extend(kept)
        self.gate = SafetyGate(self.cfg)
        self.parent = hover_plan(owner, self.start, 0.0, self.cfg.plan)
        self.seq = 1
        self.selection: tuple[str, ...] = ()
        self._frame_problem: tuple[str, ...] = ()

    def on_selection(self, plan, frame, state, now: float) -> None:
        self._add_evidence(frame, state)
        if self._scene is not None:
            self._scene.add_frames(self.space, float(now))
        env, contract, stop = self._recheck(plan, float(now))
        self.selection = tuple(dict.fromkeys((*self._frame_problem, *env, *contract, *stop)))

    def on_tick(self, state, cmd_p, now: float) -> tuple[str, ...]:
        oracle = record_oracle_track_lag(
            now, state.t, state.p, cmd_p, self.start, self.goal, 0.0,
        )
        return tuple(dict.fromkeys((*self.selection, *oracle)))

    def _add_evidence(self, frame, state) -> None:
        if frame is None:
            self._frame_problem = (ENV_NO_FRAME,)
            return
        try:
            raw = np.array(frame.depth, dtype=np.float32, copy=True)
            h, w = raw.shape
            k = CameraIntrinsics(*frame.K, width=w, height=h)
            pose = optical_pose(np.asarray(state.p, dtype=float), R_yaw(float(state.yaw)))
            evidence = DepthSample(raw, k, pose, int(round(float(frame.stamp) * 1e9))).evidence(self.cfg.space)
        except (TypeError, ValueError):
            self._frame_problem = (ENV_NONFINITE,)
            return
        self.space.add(evidence)
        self._frame_problem = ()

    def _recheck(self, plan, now: float):
        existing = _executed_certified_plan(plan)
        if existing is not None:
            if self._scene is not None:
                self._scene.certify_plan(self.space, existing)
            return (
                _env_of(self.space, existing, now),
                _transition(validate_plan(existing, self.cfg.plan)),
                (),  # 没有父 Plan 对象，不造 PARENT_MISMATCH
            )
        built = self._build(plan, now)
        if isinstance(built, list):
            motion = getattr(plan, "traj", None)
            if self._scene is not None and isinstance(motion, PiecewisePolynomial):
                self._scene.certify_trajs(self.space, [motion])
            env = _sweep(self.space, motion, now) if isinstance(motion, PiecewisePolynomial) else ()
            return env, _transition(built), ()
        if self._scene is not None:
            self._scene.certify_plan(self.space, built)
        env = _env_of(self.space, built, now)
        contract = _transition(validate_plan(built, self.cfg.plan))
        res = self.gate.validate(built, self.parent, None, self.space, (), now)
        if isinstance(res, ApprovedPlan):
            self.parent = built
            self.seq += 1
            return env, contract, ()
        if isinstance(res, GateReject):
            return env, contract, tuple(res.codes)
        return env, contract, ()

    def _build(self, plan, now: float):
        parent_hash = self.parent.plan_hash(self.cfg.plan)
        traj = getattr(plan, "traj", None)
        if isinstance(traj, PiecewisePolynomial):
            t_switch = float(traj.t_start)
            try:
                head = np.array([traj.evaluate(t_switch, k) for k in range(3)])
            except (TypeError, ValueError):
                return [PlanReject.MALFORMED]
            branches = [t_switch + o for o in self.cfg.branch_offsets]
            return make_plan(
                self.owner, self.seq, parent_hash, traj, t_switch, head, branches, self.cfg.plan,
            )
        pos = getattr(plan, "position", None)
        if pos is None:
            pos = getattr(plan, "hold", None)  # certified 尚未批准时，动力学跟踪的是这块原地保持
        if pos is not None:
            try:
                return hover_plan(
                    self.owner, np.asarray(pos, dtype=float), now, self.cfg.plan, parent=parent_hash,
                )
            except ValueError:
                return [PlanReject.MALFORMED]
        return [PlanReject.MALFORMED]


def _executed_certified_plan(plan):
    """最新一次本机批准（pending 优先于正在执行的 current）。没有父 Plan，调用方不再跑 SafetyGate。"""
    rt = getattr(getattr(plan, "policy", None), "runtime", None)
    if rt is None:
        return None
    token = rt.pending if rt.pending is not None else rt.current
    if token is None:
        return None
    current = token.plan
    return current if isinstance(current, Plan) else None


def aggregate(ticks: list[dict]) -> tuple[float, dict[str, int]]:
    """拒绝率 = 会拒的 tick / 全部 tick。直方图按码计 tick 次数，一 tick 多码各计一次。"""
    hist: dict[str, int] = {}
    rejected = 0
    for t in ticks:
        codes = t.get("reject_codes") or []
        if t.get("would_reject"):
            rejected += 1
        for code in codes:
            hist[code] = hist.get(code, 0) + 1
    n = len(ticks)
    return (0.0 if n == 0 else rejected / n), dict(sorted(hist.items()))


def aggregate_drones(ticks: list[dict], ids: list[str]) -> list[dict]:
    """按无人机 id 的出现顺序各出一行，再由调用方对全部 tick 做总拒绝率。"""
    rows = []
    for uid in ids:
        subset = [t for t in ticks if t.get("uav") == uid]
        rate, hist = aggregate(subset)
        rows.append({"id": uid, "shadow_reject_rate": rate, "shadow_reject_histogram": hist})
    return rows
