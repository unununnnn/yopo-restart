"""单机规划器（#9）：一帧观测快照 → 经本机批准门（SafetyGate）放行的新计划，或带分层诊断码的拒绝。

流水线：输入校验 → 深度入滚动自由空间 → 候选源（网络或离线脚本源）→ #6 解码并在 t_switch
重构头状态 → 运动前缀（截断）+ 末端制动 → #8 计划契约（固定分支点制动分支、终端占用）
→ #7 环境检查（原始深度证据，UNKNOWN 不放行）→ 邻机占用冲突（#4 连续时间检查）
→ SafetyGate 独立复查 → 批准。

- 规划器只能通过 SafetyGate 产出计划：`ApprovedPlan` 只能由门构造，`Decision` 只接受
  `ApprovedPlan`（I7）。门不信任调用方的前序检查，自己重跑契约、头状态、环境与邻机检查。
- 观测快照里没有地图真值字段；规划器只看深度、位姿、状态、目标、已批准计划和邻机占用。
- 不导入 ROS；离线核心只接受显式时间。
- 每步可交给 `StepLog`（I9）；JSONL 实现与逐位重放在 `replay.py`。
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from typing import Any, Protocol

import numpy as np

from .candidates import (
    UPSTREAM_DECODER,
    CandidateReject,
    DecoderParams,
    Timing,
    build_candidates,
)
from .freespace import DepthFrame, FreeSpace, FreeSpaceConfig, SpaceVerdict
from .plan import (
    CONTINUITY_TOL,
    OccupancySet,
    Plan,
    PlanConfig,
    PlanReject,
    brake_branch,
    canonical_digest,
    conflicts,
    executing_traj,
    make_plan,
    quintic,
    state_at,
    validate_plan,
)
from .trajectory import PiecewisePolynomial, PolySegment

RUN_ASSUMPTIONS = (
    "固定成员",
    "非恶意节点",
    "有效跟踪误差界（tracking_margin）",
    "共同仿真时基",
    "控制器按已批准分支执行",
    "两帧之间及证书有效期内环境静止",
    "障碍尺度不小于像素足迹",
)

__all__ = [
    "RUN_ASSUMPTIONS",
    "ApprovedPlan",
    "CandidateVerdict",
    "Decision",
    "Diagnostic",
    "Fallback",
    "Funnel",
    "GateReject",
    "Observation",
    "PeerOccupancy",
    "PlanReject",
    "Planner",
    "PlannerConfig",
    "SafetyGate",
    "StepLog",
    "VehicleState",
    "straight_plan",
]


# --- 配置 -----------------------------------------------------------------------


@dataclass(frozen=True)
class PlannerConfig:
    plan: PlanConfig
    space: FreeSpaceConfig
    decoder: DecoderParams = UPSTREAM_DECODER
    owner: str = "uav0"
    frame: str = "world-ENU"
    switch_delay: float = 0.1  # t_switch = now + switch_delay
    prefix_time: float = 1.0  # 运动前缀最长时长；之后接制动到静止
    branch_offsets: tuple[float, ...] = (0.0, 0.5)  # 固定分支点（相对 t_switch）
    max_frame_age: float = 0.2
    min_valid_fraction: float = 0.05
    max_skew: float = 0.03
    certificate_slack: float = 0.25

    def __post_init__(self) -> None:
        if self.space.sample_radius < self.plan.tube_radius:
            raise ValueError("free-space sample radius must cover the plan tube radius")
        if self.space.max_age < self.max_frame_age:
            raise ValueError("free-space max_age must be >= max_frame_age")
        if any(not (0.0 <= o <= self.prefix_time) for o in self.branch_offsets):
            raise ValueError("branch offsets must lie within the motion prefix")
        if len(self.branch_offsets) > self.plan.max_branch_points:
            raise ValueError("more branch offsets than max_branch_points")

    def to_canonical(self) -> dict:
        f = float.hex
        sp = self.space
        return {
            "plan": self.plan.to_canonical(),
            "space": {
                k: (f(float(v)) if isinstance(v, float) else v) for k, v in sorted(vars(sp).items())
            },
            "decoder": {
                k: (f(float(v)) if isinstance(v, float) else v)
                for k, v in sorted(vars(self.decoder).items())
            },
            "owner": self.owner,
            "frame": self.frame,
            "switch_delay": f(self.switch_delay),
            "prefix_time": f(self.prefix_time),
            "branch_offsets": [f(x) for x in self.branch_offsets],
            "max_frame_age": f(self.max_frame_age),
            "min_valid_fraction": f(self.min_valid_fraction),
            "max_skew": f(self.max_skew),
            "certificate_slack": f(self.certificate_slack),
        }

    def config_hash(self) -> str:
        return canonical_digest(self.to_canonical())


# --- 输入 -----------------------------------------------------------------------


@dataclass(frozen=True)
class VehicleState:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    R_wb: np.ndarray  # 机体系（x 前、y 左、z 上；与网络坐标一致）→ 世界系
    stamp: float


@dataclass(frozen=True)
class PeerOccupancy:
    owner: str
    version: int
    occupancy: OccupancySet


@dataclass(frozen=True)
class Observation:
    """规划器的全部输入。刻意没有任何地图真值 / 障碍列表字段。"""

    frame: DepthFrame
    state: VehicleState
    goal: np.ndarray
    approved: Plan
    peers: tuple[PeerOccupancy, ...]
    now: float
    frame_id: str = "world-ENU"
    active_branch: int | None = None  # 正在执行已批准计划的哪个制动分支（None = 运动段）


class CandidateSource(Protocol):
    name: str

    def describe(self) -> dict: ...

    def __call__(
        self, obs: Observation, dec: DecoderParams
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]: ...


# --- 输出 -----------------------------------------------------------------------


class StepLog(Protocol):
    """每步决策的记录器（`replay.ReplayLog` 是 JSONL 实现）。"""

    def write_step(
        self, planner: Planner, k: int, obs: Observation, net: Any, decision: Decision
    ) -> None: ...


class Diagnostic(Enum):
    APPROVED = "APPROVED"
    INPUT_MALFORMED = "INPUT_MALFORMED"
    UNKNOWN_FRAME = "UNKNOWN_FRAME"
    STALE_FRAME = "STALE_FRAME"
    EMPTY_FRAME = "EMPTY_FRAME"
    NET_OUTPUT_MALFORMED = "NET_OUTPUT_MALFORMED"
    STATE_IMAGE_SKEW = "STATE_IMAGE_SKEW"
    TIME_ORDER = "TIME_ORDER"
    PARENT_EXPIRED = "PARENT_EXPIRED"
    CORRIDOR_EMPTY = "CORRIDOR_EMPTY"  # 上游 brake=True：没有候选通过预测走廊准入（D3）
    NO_CANDIDATE = "NO_CANDIDATE"
    NO_TRANSITION_CANDIDATE = "NO_TRANSITION_CANDIDATE"
    NO_ENV_SAFE_CANDIDATE = "NO_ENV_SAFE_CANDIDATE"
    NO_PEER_SAFE_CANDIDATE = "NO_PEER_SAFE_CANDIDATE"
    GATE_REJECTED = "GATE_REJECTED"


@dataclass(frozen=True)
class Funnel:
    decoded: int
    transition_ok: int
    env_safe: int
    peer_safe: int
    approved: int


@dataclass(frozen=True)
class CandidateVerdict:
    index: int  # 图像顺序索引
    rank: int  # 检查顺序
    code: str  # OK / TRANSITION:<PlanReject> / ENV:OCCUPIED / ENV:UNKNOWN / PEER:CONFLICT
    detail: str = ""


@dataclass(frozen=True)
class Fallback:
    kind: str  # KEEP_APPROVED（继续已批准计划，含其制动分支与终端保持）/ BRAKE_BRANCH
    plan_hash: str
    t_branch: float | None = None


_GATE_TOKEN = object()


class ApprovedPlan:
    """只有 SafetyGate 能构造的批准凭证。"""

    __slots__ = ("plan", "plan_hash")

    def __init__(self, token: object, plan: Plan, plan_hash: str) -> None:
        if token is not _GATE_TOKEN:
            raise PermissionError("ApprovedPlan can only be issued by SafetyGate")
        self.plan = plan
        self.plan_hash = plan_hash


@dataclass(frozen=True)
class GateReject:
    codes: tuple[str, ...]


@dataclass(frozen=True)
class Decision:
    step: int
    diagnostic: Diagnostic
    approved: ApprovedPlan | None
    fallback: Fallback | None
    funnel: Funnel
    candidates: tuple[CandidateVerdict, ...]

    def __post_init__(self) -> None:
        if self.approved is not None and not isinstance(self.approved, ApprovedPlan):
            raise TypeError("Decision.approved must be an ApprovedPlan issued by SafetyGate")
        if (self.approved is None) == (self.diagnostic is Diagnostic.APPROVED):
            raise ValueError("diagnostic APPROVED iff a plan is approved")

    def canonical(self) -> dict:
        fb = self.fallback
        return {
            "step": self.step,
            "diagnostic": self.diagnostic.value,
            "approved": None
            if self.approved is None
            else {
                "plan_hash": self.approved.plan_hash,
                "plan_seq": self.approved.plan.plan_seq,
                "t_switch": float(self.approved.plan.t_switch).hex(),
            },
            "fallback": None
            if fb is None
            else {
                "kind": fb.kind,
                "plan_hash": fb.plan_hash,
                "t_branch": None if fb.t_branch is None else float(fb.t_branch).hex(),
            },
            "funnel": vars(self.funnel).copy(),
            "candidates": [[c.index, c.rank, c.code, c.detail] for c in self.candidates],
        }


# --- 执行状态与计划构造 -----------------------------------------------------------


def _motion_with_brake(
    cand: PiecewisePolynomial, t_switch: float, prefix_time: float, cfg: PlanConfig
) -> PiecewisePolynomial | None:
    """候选截断到前缀末端，再接一段制动到静止（终端可保持）。"""
    tp = min(t_switch + prefix_time, cand.t_end)
    segs, start = [], cand.t0
    for s in cand.segments:
        if start >= tp:
            break
        segs.append(PolySegment(s.coeffs, min(s.duration, tp - start)))
        start += s.duration
    head = np.array([cand.evaluate(tp, k) for k in range(3)])
    brake = brake_branch(tp, head, cfg.limits)
    if brake is None:
        return None
    return PiecewisePolynomial(t_switch, [*segs, *brake.segments])


def straight_plan(
    approved: Plan,
    t_switch: float,
    target: np.ndarray,
    cfg: PlannerConfig,
    duration: float = 4.0,
) -> Plan | list[PlanReject]:
    """从已批准计划在 t_switch 的状态出发、duration 内直达 target 并静止的计划（构造对照用例）。"""
    head = state_at(approved, t_switch)
    seg = quintic(head, np.array([np.asarray(target, float), np.zeros(3), np.zeros(3)]), duration)
    motion = PiecewisePolynomial(t_switch, [seg])
    return make_plan(
        cfg.owner,
        approved.plan_seq + 1,
        approved.plan_hash(cfg.plan),
        motion,
        t_switch,
        head,
        [t_switch + o for o in cfg.branch_offsets],
        cfg.plan,
    )


def _plan_parts(plan: Plan) -> list[PiecewisePolynomial]:
    return [plan.motion, *(b.traj for b in plan.branches)]


def _env_code(space: FreeSpace, plan: Plan, now: float) -> str | None:
    unknown = None
    for tr in _plan_parts(plan):
        res = space.classify_sweep(tr, now)
        if res.verdict is SpaceVerdict.OCCUPIED:
            return "ENV:OCCUPIED"
        if res.verdict is SpaceVerdict.UNKNOWN:
            unknown = "ENV:UNKNOWN"
    return unknown


def _peer_conflict(plan: Plan, cfg: PlanConfig, peers: Sequence[PeerOccupancy]) -> str | None:
    occ = plan.occupancy(cfg)
    for p in peers:
        if conflicts(occ, p.occupancy):
            return p.owner
    return None


def _finite(*xs: Any) -> bool:
    try:
        return all(bool(np.all(np.isfinite(np.asarray(x, dtype=float)))) for x in xs)
    except (TypeError, ValueError):
        return False


def _is_rotation(r: Any) -> bool:
    r = np.asarray(r, dtype=float)
    return (
        r.shape == (3, 3)
        and _finite(r)
        and np.allclose(r @ r.T, np.eye(3), atol=1e-6)
        and abs(np.linalg.det(r) - 1.0) < 1e-6
    )


# --- 批准门 ---------------------------------------------------------------------


class SafetyGate:
    """本机批准门：独立复查，任何一项不过即拒绝。规划器没有其他产出计划的路径。"""

    def __init__(self, cfg: PlannerConfig) -> None:
        self.cfg = cfg

    def validate(
        self,
        plan: Plan,
        parent: Plan,
        active_branch: int | None,
        space: FreeSpace,
        peers: Sequence[PeerOccupancy],
        now: float,
    ) -> ApprovedPlan | GateReject:
        pc = self.cfg.plan
        if not isinstance(plan, Plan) or not isinstance(parent, Plan):
            return GateReject(("MALFORMED",))
        codes = [f"TRANSITION:{r.value}" for r in validate_plan(plan, pc)]
        if codes:
            return GateReject(tuple(codes))
        if not plan.t_switch > now:
            codes.append("TIME:SWITCH_NOT_IN_FUTURE")
        if plan.owner != self.cfg.owner:
            codes.append("OWNER_MISMATCH")
        if plan.parent_plan_hash != parent.plan_hash(pc):
            codes.append("PARENT_MISMATCH")
        head = state_at(parent, plan.t_switch, active_branch)
        if float(np.max(np.abs(np.asarray(plan.head_pva) - head))) > CONTINUITY_TOL:
            codes.append(f"TRANSITION:{PlanReject.HEAD_DISCONTINUITY.value}")
        env = _env_code(space, plan, now)
        if env is not None:
            codes.append(env)
        if _peer_conflict(plan, pc, peers) is not None:
            codes.append("PEER:CONFLICT")
        if codes:
            return GateReject(tuple(codes))
        return ApprovedPlan(_GATE_TOKEN, plan, plan.plan_hash(pc))


# --- 规划器 ---------------------------------------------------------------------


_CAND_REJECT_DIAG = {
    CandidateReject.MALFORMED: Diagnostic.NET_OUTPUT_MALFORMED,
    CandidateReject.STATE_IMAGE_SKEW: Diagnostic.STATE_IMAGE_SKEW,
    CandidateReject.TIME_ORDER: Diagnostic.TIME_ORDER,
    CandidateReject.PARENT_EXPIRED: Diagnostic.PARENT_EXPIRED,
    CandidateReject.CORRIDOR_EMPTY: Diagnostic.CORRIDOR_EMPTY,
}
_BRAKE_FALLBACK = {
    Diagnostic.INPUT_MALFORMED,
    Diagnostic.UNKNOWN_FRAME,
    Diagnostic.STALE_FRAME,
    Diagnostic.EMPTY_FRAME,
    Diagnostic.CORRIDOR_EMPTY,  # 保真登记 (b)：上游刹车 → 本仓库已认证刹车分支
}


class Planner:
    def __init__(
        self,
        cfg: PlannerConfig,
        source: CandidateSource,
        gate: SafetyGate | None = None,
        log: StepLog | None = None,
    ) -> None:
        self.cfg = cfg
        self.source = source
        self.gate = gate or SafetyGate(cfg)
        self.space = FreeSpace(cfg.space)
        self.log = log
        self._step = 0
        self._assumed: list[dict] = []

    def assume_free(self, center: np.ndarray, radius: float, label: str) -> None:
        """运行假设（例如起飞区无障碍）；会写进日志头。"""
        c = np.asarray(center, dtype=float)
        self.space.assume_free(c, radius, label)
        self._assumed.append(
            {"label": label, "center": [float(x).hex() for x in c], "radius": float(radius).hex()}
        )

    def assumptions(self) -> dict:
        return {"run": list(RUN_ASSUMPTIONS), "free_space": list(self._assumed)}

    # -- 主入口 --
    def step(self, obs: Observation) -> Decision:
        k = self._step
        self._step += 1
        net: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
        decision, net = self._decide(k, obs)
        if self.log is not None:
            self.log.write_step(self, k, obs, net, decision)
        return decision

    def _reject(self, k: int, obs: Observation, diag: Diagnostic, verdicts=(), funnel=None):
        approved = obs.approved if isinstance(obs.approved, Plan) else None
        fb = None
        if approved is not None:
            h = approved.plan_hash(self.cfg.plan)
            fb = Fallback("KEEP_APPROVED", h)
            if diag in _BRAKE_FALLBACK and obs.active_branch is None and _finite(obs.now):
                nxt = [b.t_branch for b in approved.branches if b.t_branch >= obs.now]
                if nxt:
                    fb = Fallback("BRAKE_BRANCH", h, min(nxt))
        return Decision(k, diag, None, fb, funnel or Funnel(0, 0, 0, 0, 0), tuple(verdicts))

    def _input_problem(self, obs: Observation) -> Diagnostic | None:
        if obs.frame_id != self.cfg.frame:
            return Diagnostic.UNKNOWN_FRAME
        st = obs.state
        if (
            not isinstance(obs.frame, DepthFrame)
            or not isinstance(st, VehicleState)
            or not isinstance(obs.approved, Plan)
            or validate_plan(obs.approved, self.cfg.plan)
            or not _finite(st.position, st.velocity, st.acceleration, st.stamp, obs.goal, obs.now)
            or np.shape(st.position) != (3,)
            or np.shape(obs.goal) != (3,)
            or not _is_rotation(st.R_wb)
            or not all(isinstance(p, PeerOccupancy) for p in obs.peers)
            or (
                obs.active_branch is not None
                and not (0 <= obs.active_branch < len(obs.approved.branches))
            )
        ):
            return Diagnostic.INPUT_MALFORMED
        return None

    def _decide(self, k: int, obs: Observation):
        cfg = self.cfg
        bad = self._input_problem(obs)
        if bad is not None:
            return self._reject(k, obs, bad), None
        frame, now = obs.frame, float(obs.now)
        self.space.add(frame)
        if not (0.0 <= now - frame.stamp <= cfg.max_frame_age):
            return self._reject(k, obs, Diagnostic.STALE_FRAME), None
        if float(np.mean(frame.valid)) < cfg.min_valid_fraction:
            return self._reject(k, obs, Diagnostic.EMPTY_FRAME), None

        net = self.source(obs, cfg.decoder)
        params, score, radius = (np.asarray(x, dtype=float) for x in net)
        net = (params, score, radius)
        if not _finite(params, score, radius):
            return self._reject(k, obs, Diagnostic.NET_OUTPUT_MALFORMED), net

        t_switch = now + cfg.switch_delay
        timing = Timing(frame.stamp, obs.state.stamp, now, now, t_switch)
        parent = executing_traj(obs.approved, t_switch, obs.active_branch)
        cands = build_candidates(
            params,
            score,
            radius,
            cfg.decoder,
            parent,
            timing,
            obs.state.R_wb,
            obs.state.position,
            max_skew=cfg.max_skew,
        )
        if isinstance(cands, CandidateReject):
            return self._reject(k, obs, _CAND_REJECT_DIAG[cands]), net
        if not cands:
            return self._reject(k, obs, Diagnostic.NO_CANDIDATE), net

        pc = cfg.plan
        head = state_at(obs.approved, t_switch, obs.active_branch)
        parent_hash = obs.approved.plan_hash(pc)
        branch_times = [t_switch + o for o in cfg.branch_offsets]
        verdicts: list[CandidateVerdict] = []
        passing: list[tuple[int, Plan]] = []
        n_trans = n_env = n_peer = 0
        for rank, c in enumerate(cands):
            motion = _motion_with_brake(c.traj, t_switch, cfg.prefix_time, pc)
            if motion is None:
                code = f"TRANSITION:{PlanReject.BRANCH_UNREACHABLE.value}"
                verdicts.append(CandidateVerdict(c.index, rank, code, "terminal brake"))
                continue
            plan = make_plan(
                cfg.owner,
                obs.approved.plan_seq + 1,
                parent_hash,
                motion,
                t_switch,
                head,
                branch_times,
                pc,
            )
            if isinstance(plan, list):
                code = f"TRANSITION:{plan[0].value}"
                detail = ",".join(r.value for r in plan)
                verdicts.append(CandidateVerdict(c.index, rank, code, detail))
                continue
            n_trans += 1
            env = _env_code(self.space, plan, now)
            if env is not None:
                verdicts.append(CandidateVerdict(c.index, rank, env))
                continue
            n_env += 1
            peer = _peer_conflict(plan, pc, obs.peers)
            if peer is not None:
                verdicts.append(CandidateVerdict(c.index, rank, "PEER:CONFLICT", peer))
                continue
            n_peer += 1
            verdicts.append(CandidateVerdict(c.index, rank, "OK"))
            passing.append((len(verdicts) - 1, plan))

        approved = None
        for vi, plan in passing:
            res = self.gate.validate(
                plan, obs.approved, obs.active_branch, self.space, obs.peers, now
            )
            if isinstance(res, ApprovedPlan):
                approved = res
                break
            v = verdicts[vi]
            verdicts[vi] = CandidateVerdict(v.index, v.rank, v.code, "gate:" + ",".join(res.codes))

        funnel = Funnel(len(cands), n_trans, n_env, n_peer, 1 if approved is not None else 0)
        if approved is None:
            if n_trans == 0:
                diag = Diagnostic.NO_TRANSITION_CANDIDATE
            elif n_env == 0:
                diag = Diagnostic.NO_ENV_SAFE_CANDIDATE
            elif n_peer == 0:
                diag = Diagnostic.NO_PEER_SAFE_CANDIDATE
            else:
                diag = Diagnostic.GATE_REJECTED
            return self._reject(k, obs, diag, verdicts, funnel), net

        for tr in _plan_parts(approved.plan):
            self.space.certify_sweep(tr, now, cfg.certificate_slack, approved.plan_hash[:12])
        return Decision(k, Diagnostic.APPROVED, approved, None, funnel, tuple(verdicts)), net
