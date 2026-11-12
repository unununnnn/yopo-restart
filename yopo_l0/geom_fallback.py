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
