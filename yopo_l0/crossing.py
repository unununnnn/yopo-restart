"""票 07：三机交叉避障。队形尽力而为，避障优先。

每机在自己的提案窗口里，用深度图跑上游 YOPO（走廊准入 + 最高分），把编队槽位
融进目标当作偏置。三机和柱林不读场景几何。选出的多项式接到本机已提交计划在 t_start 的
状态上，经轮转窗口和预留 ACK/COMMIT 之后才交给动力学。跟邻机占用冲突就放弃
这次提案，继续飞本机已经批准的计划。没有中央裁决者。

票 13a：六机构造时把 commit_horizon_s 抬到至少一轮窗口 + 0.25 s，计划 a_max 与
L0 动力学相同，禁止再写死 80。票 13c：六机在林带里槽位是软目标。侧向、纵向只在
界内松弛，避障是硬约束；出了林带收紧回原来的刚性槽位前瞻。COMMIT 前对轨迹采样：
中心到树的最小距离必须 ≥ body_radius + margin（默认 0.15 m）。不够就
CLEARANCE_SKIP，不把这条多项式交出去。这是过滤，不引入 CPLEX。不改 safe_radius
默认 0.05 m，也不改 L0 的 kp / kd / tau。

票 13b：穿林的虚拟中心不再按墙钟×speed 开环走。frank 2026-11-09 定进度门控只在林带开、
开阔区关。沿中心投影的进度取 min（progress_percentile 0）或低分位，落后机把时刻表按住；
超前机只提案悬停，或一条 0.3 s 短前缀。悬停和短前缀的刹车用场景卡 a_max（与 13a 计划契约相同）。
不换 YOPO，也不引入外部求解器。没有 forest 的柱林默认仍是开环。
"""

from __future__ import annotations

import copy
import hashlib
import logging
import math
from dataclasses import replace

import numpy as np

from yopo_swarm_core.candidates import (
    decode_params,
    minco_two_piece,
    rank_candidates,
)
from yopo_swarm_core.collision import Verdict, check_derivative_bound
from yopo_swarm_core.plan import (
    BrakeBranch,
    KinematicLimits,
    Plan,
    PlanConfig,
    hold_traj,
    make_plan,
    quintic,
    state_at,
)
from yopo_swarm_core.protocol.messages import Proposal, digest
from yopo_swarm_core.protocol.node import ProtocolConfig, ProtocolNode
from yopo_swarm_core.protocol.plan_occupancy import PlanConflictChecker, PlanOccupancy, hover_occupancy
from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment

from .backend.config import config_from_params
from .criteria import frozen_for_card
from .dynamics import PointMassLag
from .forest import _segment_distance, woods_bounds
from .geom_fallback import KIND, GeomSession, make_protocol_node
from .inter_drone import (
    clearance_m,
    committed_neighbor_samples,
    inter_drone_config,
    inter_pick,
    report_block,
    sample_motion,
)
from .policy import R_yaw, read_safe_radius, upstream_decoder
from .progress import read_progress_settings
from .scene import Box, Cylinder, parse_geometry

# 轨迹采样步长。0.02 s 的弦高按动力学 a_max 计大约几毫米，门按采样到的最小距离判。
_CLEARANCE_DT_S = 0.02

_LOG = logging.getLogger(__name__)

# 下界 N_agents × window_len_s + 0.25。短于下界的场景卡或默认值在构造时抬高。
# 0.25 s 是两次本机 COMMIT 之间多留的一截，避免急刹掉进下一次提交还没生效的空档。
HORIZON_MARGIN_S = 0.25

# 速度、急动度仍放宽，只为计划契约肯收下上游多项式的速度尖峰。
# 加速度必须等于 L0 动力学。写成 80 时，急刹在契约里已经停住，点质量（a_max 6）还在滑。
_PLAN_V_MAX = 20.0
_PLAN_J_MAX = 20000.0
# 超前超过这一截才改提悬停或短前缀。落后机仍走原来的 YOPO。
_LEADER_MARGIN_M = 1.0
_LEADER_PREFIX_S = 0.3


def required_commit_horizon_s(n_agents: int, window_len_s: float) -> float:
    """commit_horizon_s 的下界：N_agents × window_len_s + 0.25。"""
    if isinstance(n_agents, bool) or not isinstance(n_agents, int) or n_agents < 1:
        raise ValueError("N_agents 必须是正整数")
    window = float(window_len_s)
    if not math.isfinite(window) or window <= 0.0:
        raise ValueError("window_len_s 必须为正")
    return n_agents * window + HORIZON_MARGIN_S


def resolve_commit_horizon_s(requested: float, n_agents: int, window_len_s: float) -> float:
    """场景卡或默认值短于下界时抬高，并记一条日志。非正数直接报错。"""
    need = required_commit_horizon_s(n_agents, window_len_s)
    got = float(requested)
    if not math.isfinite(got) or got <= 0.0:
        raise ValueError("commit_horizon_s 必须为正")
    if got < need - 1e-12:
        _LOG.warning(
            "commit_horizon_s %s 小于 N_agents*window_len_s+%.2f=%s（N=%s, window_len_s=%s）；抬高到 %s",
            got,
            HORIZON_MARGIN_S,
            need,
            int(n_agents),
            float(window_len_s),
            need,
        )
        return need
    return got


def dynamics_a_max(dynamics: dict | None) -> float:
    """L0 动力学的 a_max。场景卡没写时用 PointMassLag 的默认值（当前 6）。"""
    dyn = dict(dynamics or {})
    if dyn.get("a_max") is None:
        a_max = float(PointMassLag.a_max)
    else:
        a_max = float(dyn["a_max"])
    if not math.isfinite(a_max) or a_max <= 0.0:
        raise ValueError("动力学 a_max 必须为正")
    return a_max


def plan_limits_for_dynamics(dynamics: dict | None) -> KinematicLimits:
    """急刹和计划契约的 a_max 与 L0 动力学相同，禁止再写死 80。"""
    return KinematicLimits(_PLAN_V_MAX, dynamics_a_max(dynamics), _PLAN_J_MAX)


def _forest_obstacles(card: dict) -> tuple:
    """场景卡里的树干和矮枝。墙和地面不参与净空门。几何没内联时返回空。"""
    geom = card.get("geometry")
    if not isinstance(geom, dict):
        return ()
    obs = []
    for raw in geom.get("cylinders") or []:
        if not str(raw.get("name", "")).startswith("t"):
            continue
        lean = raw.get("lean_xy") or (0.0, 0.0)
        obs.append(
            Cylinder(
                raw["name"],
                (float(raw["center_xy"][0]), float(raw["center_xy"][1])),
                float(raw["radius"]),
                float(raw["z_lo"]),
                float(raw["z_hi"]),
                (float(lean[0]), float(lean[1])),
            )
        )
    for raw in geom.get("boxes") or []:
        if not str(raw.get("name", "")).startswith("t"):
            continue
        obs.append(Box(raw["name"], tuple(raw["lo"]), tuple(raw["hi"])))
    return tuple(obs)


class LocalApproval:
    """本机批准码。只记本机节点的状态，不代表编队级裁决。"""

    def __init__(self) -> None:
        self.last_code = "LOCAL_HOVER"
        self.extra_codes: dict = {}


def _unit_slot(raw) -> np.ndarray:
    slot = np.asarray(raw, dtype=float)
    if slot.shape != (3,):
        raise ValueError(f"slot_body 必须是长度 3，收到 {raw}")
    return slot


def _kin_ok(traj: PiecewisePolynomial, limits: KinematicLimits) -> bool:
    return all(
        check_derivative_bound(traj, order, lim).verdict is Verdict.SAFE
        for order, lim in limits.by_order()
    )


def _prefix(traj: PiecewisePolynomial, t_cut: float) -> PiecewisePolynomial:
    """保留 [t_start, t_cut] 上的原多项式。系数不变，避障形状还是 YOPO 的。"""
    if t_cut >= traj.t_end - 1e-9:
        return traj
    t_cut = max(t_cut, traj.t_start + 0.05)
    segs: list[PolySegment] = []
    t = traj.t0
    for seg in traj.segments:
        end = t + seg.duration
        if end <= t_cut + 1e-12:
            segs.append(PolySegment(np.array(seg.coeffs, dtype=float, copy=True), seg.duration))
            t = end
            continue
        segs.append(PolySegment(np.array(seg.coeffs, dtype=float, copy=True), max(t_cut - t, 1e-3)))
        break
    if not segs:
        raise RuntimeError("YOPO 前缀是空的")
    return PiecewisePolynomial(traj.t0, segs)


def _append_stop(traj: PiecewisePolynomial, limits: KinematicLimits) -> PiecewisePolynomial | None:
    """运动段必须能在终点保持。在 YOPO 末端接一段刹车，接不上就放弃这条曲线。"""
    from yopo_swarm_core.plan import brake_branch

    t1 = traj.t_end
    head = np.array([traj.evaluate(t1, k) for k in range(3)])
    if float(np.linalg.norm(head[1])) <= 1e-6 and float(np.linalg.norm(head[2])) <= 1e-6:
        return traj
    stop = brake_branch(t1, head, limits)
    if stop is None:
        return None
    segs = [PolySegment(np.array(seg.coeffs, dtype=float, copy=True), seg.duration) for seg in traj.segments]
    segs.extend(PolySegment(np.array(seg.coeffs, dtype=float, copy=True), seg.duration) for seg in stop.segments)
    return PiecewisePolynomial(traj.t0, segs)


def _brake_motion(head: np.ndarray, t_start: float, limits: KinematicLimits) -> PiecewisePolynomial | None:
    from yopo_swarm_core.plan import brake_branch

    if float(np.linalg.norm(head[1])) <= 1e-6 and float(np.linalg.norm(head[2])) <= 1e-6:
        return None
    return brake_branch(t_start, head, limits)


def _stop_at(head: np.ndarray, t_start: float, goal: np.ndarray, limits: KinematicLimits) -> PiecewisePolynomial | None:
    """进到达半径之后的终端规则，和上游 stop_at_goal 一样：从当前 PVA 停到槽位。"""
    end = np.array([np.asarray(goal, dtype=float), np.zeros(3), np.zeros(3)])
    duration = 2.0
    while duration <= 8.0:
        traj = PiecewisePolynomial(t_start, [quintic(head, end, duration)])
        if _kin_ok(traj, limits):
            return traj
        duration *= 1.25
    return None


def _clone_traj(traj: PiecewisePolynomial) -> PiecewisePolynomial:
    segs = tuple(PolySegment(np.array(seg.coeffs, dtype=float, copy=True), seg.duration) for seg in traj.segments)
    return PiecewisePolynomial(traj.t0, segs, traj.t_start, traj.t_end)


def _clone_plan(plan: Plan) -> Plan:
    return Plan(
        plan.owner,
        plan.plan_seq,
        plan.parent_plan_hash,
        plan.t_switch,
        np.array(plan.head_pva, dtype=float, copy=True),
        _clone_traj(plan.motion),
        tuple(BrakeBranch(b.t_branch, _clone_traj(b.traj)) for b in plan.branches),
    )


def _clone_occ(occ: PlanOccupancy) -> PlanOccupancy:
    return PlanOccupancy(_clone_plan(occ.plan), occ.cfg, occ.until)


def clone_msg(msg):
    """总线深拷贝不会拆开 PlanOccupancy（它的 __deepcopy__ 返回自身）。先拆成独立计划。"""
    if isinstance(msg, Proposal):
        return replace(msg, occupancy=_clone_occ(msg.occupancy))
    return copy.deepcopy(msg)


class _Track:
    def __init__(self, plan: Plan, goal: np.ndarray, yaw: float, dt: float):
        self.plan = plan
        self.goal = np.asarray(goal, dtype=float)
        self.yaw = float(yaw)
        self.dt = float(dt)

    def pva(self, t: float):
        rows = state_at(self.plan, float(t))
        return tuple(np.asarray(x, dtype=float) for x in rows)

    @property
    def traj(self):
        return self.plan.motion

    def sample(self, t: float):
        from yopo_swarm_core.yaw import calculate_yaw

        p, v, a = self.pva(t)
        self.yaw = float(calculate_yaw(v, self.goal - p, self.yaw, self.dt)[0])
        return p, v, a, self.yaw


class _Rail:
    """直线虚拟中心。航向固定朝 +x，槽位 y 就是世界系左侧。票 08 才转弯、变阵。"""

    def __init__(self, spec: dict):
        origin = np.asarray(spec["origin_enu"], dtype=float)
        if origin.shape != (3,):
            raise ValueError("virtual_center.origin_enu 必须是长度 3")
        self.origin = origin
        self.length = float(spec["length_m"])
        self.speed = float(spec["speed_mps"])
        if self.length <= 0.0 or self.speed <= 0.0:
            raise ValueError("length_m 和 speed_mps 必须为正")
        self.T = self.length / self.speed

    def arc(self, t: float, t_go: float) -> float:
        """开环弧长：墙钟 × speed，夹在 [0, length]。"""
        tau = float(t) - float(t_go)
        if tau <= 0.0:
            return 0.0
        if tau >= self.T:
            return self.length
        return self.speed * tau

    def at(self, s: float, slot: np.ndarray) -> np.ndarray:
        s = float(min(self.length, max(0.0, float(s))))
        sx, sy, sz = (float(x) for x in slot)
        return self.origin + np.array([s + sx, sy, sz])

    def along(self, position: np.ndarray, slot: np.ndarray) -> float:
        """机体位置去掉槽位纵向偏移后，在中心线上的投影弧长。可以是负的。"""
        return float(position[0]) - float(self.origin[0]) - float(slot[0])

    def position(self, t: float, t_go: float, slot: np.ndarray) -> np.ndarray:
        return self.at(self.arc(t, t_go), slot)


class CrossingFleet:
    """三机各跑各的上游 YOPO。槽位只偏置目标；深度决定绕开哪边。"""

    policy_name = "crossing"
    result_key = "crossing"
    uses_network = True

    def __init__(self, card: dict, physics_dt: float, *, record_inputs: bool):
        task = card["task"]
        if task["type"] != "crossing":
            raise ValueError("CrossingFleet 只接受 crossing 任务")
        drones = card["drones"]
        if len(drones) < 1:
            raise ValueError("穿越至少一架飞机")
        self.members = tuple(str(d["id"]) for d in drones)
        if len(set(self.members)) != len(self.members):
            raise ValueError("无人机 id 重复")
        self.slots = {str(d["id"]): _unit_slot(d["slot_body"]) for d in drones}
        self.rail = _Rail(task["virtual_center"])
        self.dt = float(physics_dt)
        self.record_inputs = record_inputs
        self.input_digests: list[str] = []
        self.errors: list[float] = []
        self.deliveries = 0
        self.frozen = frozen_for_card(card, density=(card.get("forest") or {}).get("density"))
        self.radius = float(card["criteria"]["body_radius_m"])
        window = float(task.get("window_len_s", 0.25))
        if not math.isfinite(window) or window <= 0.0:
            raise ValueError("window_len_s 必须为正")
        self.window = window
        # 缺省 1.0 只够三机 × 0.25 s。六机沿用同一张卡时在这里抬高，不改场景卡 JSON。
        self.horizon_requested = float(task.get("commit_horizon_s", 1.0))
        self.horizon = resolve_commit_horizon_s(self.horizon_requested, len(self.members), window)
        self.lookahead_s = float(task.get("lookahead_s", 2.0))
        self.min_ahead_m = float(task.get("min_ahead_m", 5.0))
        self.lateral_gain = float(task.get("lateral_gain", 0.25))
        self.arrive_radius = float(task.get("arrive_radius_m", 1.5))
        self.safe_radius_m = read_safe_radius(card.get("policy_params"))
        self.decoder = upstream_decoder(self.safe_radius_m)
        if min(self.lookahead_s, self.min_ahead_m) <= 0.0:
            raise ValueError("lookahead_s、min_ahead_m 必须为正")
        if not 0.0 <= self.lateral_gain <= 2.0:
            raise ValueError("lateral_gain 必须在 [0, 2]")
        # 净空门的余量。门槛是机体半径 + margin，默认 0.15 m。不进 YOPO 的 safe_radius。
        self.clearance_margin_m = float(task.get("clearance_margin_m", 0.15))
        if self.clearance_margin_m < 0.0:
            raise ValueError(f"clearance_margin_m 不能为负，收到 {self.clearance_margin_m}")
        self.lateral_slack_m = float(task.get("lateral_slack_m", 2.0))
        if self.lateral_slack_m < 0.0:
            raise ValueError(f"lateral_slack_m 不能为负，收到 {self.lateral_slack_m}")
        raw_long = task.get("longitudinal_slack_m", None)
        self.longitudinal_slack_m = self.min_ahead_m if raw_long is None else float(raw_long)
        if self.longitudinal_slack_m <= 0.0:
            raise ValueError(f"longitudinal_slack_m 必须为正，收到 {self.longitudinal_slack_m}")
        self.exit_tighten_m = float(task.get("exit_tighten_m", 4.0))
        if self.exit_tighten_m < 0.0:
            raise ValueError(f"exit_tighten_m 不能为负，收到 {self.exit_tighten_m}")
        # 时刻表从 t=0 走。各机第一次能执行已是自己窗口的下一沿，起步差一两个窗口。
        self.t_go = float(task.get("t_go_s", 0.0))
        # 只有带森林的六机才松槽位、才上净空门。三机穿林和柱林保持原来的槽位前瞻。
        self.in_forest = isinstance(card.get("forest"), dict)
        self._trees = _forest_obstacles(card) if self.in_forest else ()
        # 林带默认门控。柱林没有 forest，默认开环。卡上的布尔盖过默认。
        self.progress_gate, self.progress_percentile = read_progress_settings(
            task, default_gate=self.in_forest
        )
        # 超前机悬停/短前缀用场景加速度。13a 之后计划契约已经是同一组限幅。
        self.gentle_limits = plan_limits_for_dynamics(card.get("dynamics"))
        self._progress = {did: 0.0 for did in self.members}
        spec = task["virtual_center"]
        self.cfg = ProtocolConfig(
            members=self.members,
            window_len=window,
            task_hash=digest(
                {
                    "origin": spec["origin_enu"],
                    "length_m": spec["length_m"],
                    "speed_mps": spec["speed_mps"],
                    "slots": {k: v.tolist() for k, v in self.slots.items()},
                    "window_len_s": window,
                    "commit_horizon_s": self.horizon,
                    "lookahead_s": self.lookahead_s,
                    "min_ahead_m": self.min_ahead_m,
                    "lateral_gain": self.lateral_gain,
                    "arrive_radius_m": self.arrive_radius,
                    "safe_radius_m": self.safe_radius_m,
                    "clearance_margin_m": self.clearance_margin_m,
                    "lateral_slack_m": self.lateral_slack_m,
                    "longitudinal_slack_m": self.longitudinal_slack_m,
                    "exit_tighten_m": self.exit_tighten_m,
                    "progress_gate": self.progress_gate,
                    "progress_percentile": self.progress_percentile,
                }
            ),
            # 冲突跳过是常态，不能三次失败就停飞。
            max_failed_rounds=10000,
            resend_period=None,
            heartbeat_period=None,
            # 票 13d：穿林不打开 Delay Check。δ = 0 时收齐 ACK 当拍提交，轨迹与打开之前一致。
            delay_check_s=0.0,
        )
        self.pcfg = PlanConfig(
            plan_limits_for_dynamics(card.get("dynamics")),
            max_branch_points=2,
            body_radius=self.radius,
            tracking_margin=0.15,
        )
        self.goals = {
            did: self.rail.position(self.t_go + self.rail.T, self.t_go, slot).copy()
            for did, slot in self.slots.items()
        }
        # E-B2：默认关。打开时只优化 _yopo_options 里第一条拟合成功的多项式，
        # 净空门、冲突自检和 COMMIT 仍走原来的函数。
        self._backend = config_from_params(
            card.get("policy_params"),
            a_max=dynamics_a_max(card.get("dynamics")),
            clearance_m=self.radius,
        )
        self.backend_log = [] if self._backend is not None else None
        self._backend_obstacles = ()
        if self._backend is not None and isinstance(card.get("geometry"), dict):
            self._backend_obstacles = parse_geometry(card["geometry"], card.get("ground_z"))
        self.starts = {
            did: self.rail.position(0.0, self.t_go, slot).copy() for did, slot in self.slots.items()
        }
        checker = PlanConflictChecker()
        # 视界用上面已经解析好的 self.horizon，不再走一次 13a 抬高（六机 1.75 s 只抬那一次）。
        self.geom = GeomSession.from_card(
            card, members=self.members, horizon=self.horizon, physics_dt=self.dt,
        )
        self.nodes = {
            did: make_protocol_node(
                self.geom,
                did,
                self.cfg,
                checker=checker,
                planner=self._planner,
                initial=self._initial,
            )
            for did in self.members
        }
        self._follow = {did: {"hash": None, "sampler": None} for did in self.members}
        self._last_rec = {}
        self._sense: dict[str, tuple] = {}
        self._codes: dict[str, dict[str, int]] = {did: {} for did in self.members}
        self._last_code = {did: "LOCAL_HOVER" for did in self.members}
        self.empty_solution = 0
        self._source = None
        # 默认关。关时不读邻机采样，_yopo_options 的排序与改前相同。
        self._inter = inter_drone_config(card.get("policy_params"))
        self._score_node = None
        self._score_t = 0.0
        self._inter_filtered = 0
        self._inter_penalized = 0
        self._inter_empty = 0

    @classmethod
    def from_card(cls, card: dict, physics_dt: float, *, record_inputs: bool = False) -> "CrossingFleet":
        return cls(card, physics_dt, record_inputs=record_inputs)

    @property
    def source(self):
        if self._source is None:
            from .net import InProcessSource

            self._source = InProcessSource()
        return self._source

    def _initial(self, owner: str, cfg: ProtocolConfig) -> PlanOccupancy:
        return hover_occupancy(owner, self.starts[owner], self.pcfg)

    def check_start(self, drone_id: str, start: np.ndarray) -> None:
        expect = self.starts[drone_id]
        if float(np.linalg.norm(np.asarray(start, dtype=float) - expect)) > 1e-6:
            raise ValueError(
                f"{drone_id} 的 start_enu {np.asarray(start, float).tolist()} 不是槽位在虚拟中心起点的位置 {expect.tolist()}"
            )

    def goal_of(self, drone_id: str) -> np.ndarray:
        return self.goals[drone_id].copy()

    def ingest(self, drone_id: str, frame, state) -> None:
        """只收本机深度和航向。头状态稍后从本机已提交计划取，不用里程计位置冒充。

        进度门控另记一个沿中心的标量。这是协同用的投影，不拿来当计划头。
        """
        self._sense[drone_id] = (frame, float(state.yaw))
        self.note_progress(drone_id, state.p)

    def note_progress(self, drone_id: str, position: np.ndarray) -> float:
        s = self.rail.along(np.asarray(position, dtype=float), self.slots[drone_id])
        self._progress[drone_id] = s
        return s

    def gate_s(self) -> float:
        """全队投影进度的低分位。分位 0 就是 min。夹在航段内。"""
        vals = [self._progress[did] for did in self.members if did in self._progress]
        if not vals:
            return 0.0
        s = float(np.percentile(np.asarray(vals, dtype=float), self.progress_percentile))
        if not math.isfinite(s):
            return 0.0
        return float(min(self.rail.length, max(0.0, s)))

    def schedule_s(self, t: float) -> float:
        """门控开：时刻表等于当前低分位。关：仍是墙钟 × speed。"""
        if self.progress_gate:
            return self.gate_s()
        return self.rail.arc(t, self.t_go)

    def _ahead_of_gate(self, drone_id: str, head_p: np.ndarray) -> bool:
        """超前看本机刚观测到的投影，不看 t_start 上的计划头。

        计划头已经沿着上一条曲线往前探。拿它来比的话，落后机也会被当成超前，
        下一扇窗口就把自己的前进提案换成悬停，时刻表走不动。
        """
        if not self.progress_gate:
            return False
        s = self._progress.get(drone_id)
        if s is None:
            s = self.rail.along(np.asarray(head_p, dtype=float), self.slots[drone_id])
        return float(s) > self.gate_s() + _LEADER_MARGIN_M

    def _forest_six(self) -> bool:
        return self.in_forest and len(self.members) >= 6 and bool(self._trees)

    def _required_clearance(self) -> float:
        """轨迹中心到树表面的最小距离。等于机体半径加可配 margin。"""
        return self.radius + self.clearance_margin_m

    def _commit_span(self) -> float:
        """六机轮转长于承诺地平线时，前缀盖到下一轮窗口，少飞末端急刹。"""
        cycle = len(self.members) * self.window
        if self._forest_six() and cycle > self.horizon + 1e-9:
            return cycle
        return self.horizon

    def _elastic_active(self, x: float) -> bool:
        """前方 goto 已经探进林带，且人还没出林。出林后回到刚性槽位。"""
        if not self._forest_six():
            return False
        x_lo, x_hi, _y_lo, _y_hi = woods_bounds()
        x = float(x)
        if x >= x_hi:
            return False
        return (x + self.longitudinal_slack_m) >= x_lo

    def _slack_scale(self, x: float) -> float:
        """林中为 1。出林前 exit_tighten_m 内收到 0，侧向松弛跟着收掉。"""
        _x_lo, x_hi, _y_lo, _y_hi = woods_bounds()
        tighten = self.exit_tighten_m
        x = float(x)
        if x >= x_hi or tighten <= 1e-9:
            return 0.0 if x >= x_hi else 1.0
        if x <= x_hi - tighten:
            return 1.0
        u = (x - (x_hi - tighten)) / tighten
        smooth = u * u * (3.0 - 2.0 * u)
        return 1.0 - smooth

    def _point_clearance(self, p: np.ndarray) -> float:
        if not self._trees:
            return math.inf
        best = math.inf
        point = np.asarray(p, dtype=float)
        for ob in self._trees:
            best = min(best, float(ob.distance(point)))
            if best <= 0.0:
                return best
        return best

    def _segment_clearance(self, a: np.ndarray, b: np.ndarray, *, stop_at: float | None = None) -> float:
        """线段到树的最小距离。树干用轴线线段，矮枝用采样。"""
        if not self._trees:
            return math.inf
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        best = math.inf
        dist = float(np.linalg.norm(b - a))
        n_box = max(2, int(math.ceil(dist / 0.05)) + 1)
        for ob in self._trees:
            if isinstance(ob, Cylinder):
                p0, p1 = ob.axis_ends()
                gap = _segment_distance(a, b, p0, p1) - float(ob.radius)
                best = min(best, gap)
            else:
                for s in np.linspace(0.0, 1.0, n_box):
                    best = min(best, float(ob.distance((1.0 - s) * a + s * b)))
            if stop_at is not None and best < stop_at:
                return best
        return best

    def _trajectory_min_clearance(self, motion: PiecewisePolynomial) -> float:
        """已拟合轨迹上的最小中心距。没有树就是无穷。"""
        if not self._trees:
            return math.inf
        span = float(motion.t_end - motion.t_start)
        n = max(2, int(math.ceil(span / _CLEARANCE_DT_S)) + 1)
        best = math.inf
        need = self._required_clearance()
        for t in np.linspace(float(motion.t_start), float(motion.t_end), n):
            best = min(best, self._point_clearance(np.asarray(motion.evaluate(float(t)), dtype=float)))
            if best < need:
                return best
        return best

    def _clearance_ok(self, motion: PiecewisePolynomial) -> bool:
        """COMMIT 前的硬净空门。三机和柱林不开这扇门。"""
        if not self._forest_six():
            return True
        return self._trajectory_min_clearance(motion) + 1e-9 >= self._required_clearance()

    def _y_band(self, head_y: float, slot_y: float, scale: float) -> tuple[float, float]:
        """侧向松弛：槽位两侧各 lateral_slack，再剪到走廊里，也不一次横跳超过同一界。"""
        _x_lo, _x_hi, y_lo, y_hi = woods_bounds()
        slack = self.lateral_slack_m * float(scale)
        band_lo = max(y_lo, float(slot_y) - slack)
        band_hi = min(y_hi, float(slot_y) + slack)
        near_lo = float(head_y) - slack
        near_hi = float(head_y) + slack
        lo = max(band_lo, near_lo)
        hi = min(band_hi, near_hi)
        if lo > hi + 1e-9:
            edge = min(max(float(head_y), band_lo), band_hi)
            return edge, edge
        return lo, hi

    def _candidate_ys(self, head_y: float, slot_y: float, scale: float) -> list[float]:
        lo, hi = self._y_band(head_y, slot_y, scale)
        raw = [float(slot_y), float(head_y), lo, hi]
        if hi - lo > 1e-6:
            raw.extend(float(y) for y in np.linspace(lo, hi, 9))
        out: list[float] = []
        for y in raw:
            if y < lo - 1e-9 or y > hi + 1e-9:
                continue
            y = min(max(y, lo), hi)
            if all(abs(y - z) > 1e-6 for z in out):
                out.append(y)
        if not out:
            out.append(0.5 * (lo + hi))
        return out

    def _lane_reach(self, head: np.ndarray, y: float, z_goal: float) -> tuple[float, float, np.ndarray]:
        """射线上整段过净空门的最远点。缩短时仍停在同一条射线上，不另画一条斜线。"""
        head = np.asarray(head, dtype=float)
        cap = float(self.longitudinal_slack_m)
        need = self._required_clearance()
        far = np.array([float(head[0]) + cap, float(y), float(z_goal)], dtype=float)

        def at(dist: float) -> np.ndarray:
            return head + (dist / cap) * (far - head)

        if self._segment_clearance(head, far, stop_at=need) + 1e-9 >= need:
            return cap, self._segment_clearance(head, far), far
        lo, hi = 0.0, cap
        for _ in range(12):
            mid = 0.5 * (lo + hi)
            if self._segment_clearance(head, at(mid), stop_at=need) + 1e-9 >= need:
                lo = mid
            else:
                hi = mid
        probe = at(min(1.0, cap))
        return lo, self._segment_clearance(head, probe), at(lo)

    def _search_goals(self, head: np.ndarray, final: np.ndarray, slot_y: float, scale: float) -> list[np.ndarray]:
        """界内挑侧向。能走的优先贴槽位，再取更长的净空；都不能走就挑 1 m 处最干净的。"""
        head = np.asarray(head, dtype=float)
        z_goal = float(final[2])
        cap = float(self.longitudinal_slack_m)
        ranked: list[tuple] = []
        for y in self._candidate_ys(float(head[1]), slot_y, scale):
            reach, probe, point = self._lane_reach(head, y, z_goal)
            far = np.array([float(head[0]) + cap, float(y), z_goal], dtype=float)
            room = float(final[0]) - float(head[0])
            if room <= 1.0:
                dist = min(max(room, 0.0), cap)
                goal = head + (dist / cap) * (far - head)
            elif reach + 1e-9 >= 1.0:
                goal = np.array(point, dtype=float, copy=True)
            else:
                goal = head + (min(1.0, cap) / cap) * (far - head)
            goal = np.array(goal, dtype=float, copy=True)
            if float(goal[0]) > float(final[0]):
                goal[0] = float(final[0])
            goal[2] = z_goal
            if reach + 1e-9 >= 1.0:
                key = (1, -abs(y - float(slot_y)), reach, -abs(y - float(head[1])))
            else:
                key = (0, probe, -abs(y - float(slot_y)), -abs(y - float(head[1])))
            ranked.append((key, goal, float(y)))
        ranked.sort(key=lambda item: item[0], reverse=True)
        goals: list[np.ndarray] = []
        chosen: list[float] = []
        for _key, goal, y in ranked:
            if any(abs(y - prev) <= 1e-6 for prev in chosen):
                continue
            chosen.append(y)
            goals.append(goal)
            if len(goals) == 3:
                break
        if not goals:
            goals.append(np.array([min(float(final[0]), float(head[0]) + 1.0), float(slot_y), z_goal]))
        return goals

    def _elastic_goals(self, head: np.ndarray, final: np.ndarray, slot_y: float) -> list[np.ndarray]:
        """出林收紧。收紧后的射线要是已经进树，就放回界内松弛，避障不让步。"""
        head = np.asarray(head, dtype=float)
        scale = self._slack_scale(float(head[0]))
        goals = self._search_goals(head, final, slot_y, scale)
        need = self._required_clearance()
        if scale < 1.0 - 1e-9 and self._segment_clearance(head, goals[0], stop_at=need) + 1e-9 < need:
            wider = self._search_goals(head, final, slot_y, 1.0)
            wide_gap = self._segment_clearance(head, wider[0])
            tight_gap = self._segment_clearance(head, goals[0])
            if wide_gap + 1e-9 >= need or wide_gap > tight_gap + 1e-9:
                return wider
        return goals

    def goal_for(self, drone_id: str, t: float, head_p: np.ndarray) -> np.ndarray:
        """槽位前瞻和终点槽位的混合，再加一点拉回航线的侧向偏置。

        目标始终在机头前方、终点以内，避免网络看见身后的目标掉头。
        避障不在这里做：进网络的是这个目标和本机深度。
        六机在林带里不走这条混合，槽位只是界内的软目标，见 `_elastic_goals`。
        """
        slot = self.slots[drone_id]
        head_p = np.asarray(head_p, dtype=float)
        final = self.goals[drone_id]
        if self._elastic_active(float(head_p[0])):
            return self._elastic_goals(head_p, final, float(slot[1]))[0]
        s = self.schedule_s(t)
        ahead = self.rail.at(s + self.lookahead_s * self.rail.speed, slot)
        aim_x = min(float(final[0]), max(float(ahead[0]), float(head_p[0]) + self.min_ahead_m))
        aim = np.array([aim_x, float(ahead[1]), float(ahead[2])])
        remain = max(float(final[0] - ahead[0]), 0.0)
        reach = max(self.lookahead_s * self.rail.speed, 1e-6)
        w_final = 1.0 - min(remain / reach, 1.0)
        goal = (1.0 - w_final) * aim + w_final * final
        lane_y = float(self.rail.at(s, slot)[1])
        goal = np.array(goal, dtype=float, copy=True)
        goal[1] += self.lateral_gain * (lane_y - float(head_p[1]))
        goal[2] = float(final[2])
        if float(final[0]) - float(head_p[0]) > 1.0:
            goal[0] = min(float(final[0]), max(float(goal[0]), float(head_p[0]) + 1.0))
        return goal

    def _count(self, drone_id: str, code: str) -> None:
        slot = self._codes[drone_id]
        slot[code] = slot.get(code, 0) + 1
        self._last_code[drone_id] = code

    def _digest(self, drone_id: str, t_start: float, head: np.ndarray, goal: np.ndarray, yaw: float, frame) -> str:
        h = hashlib.sha256()
        h.update(drone_id.encode())
        h.update(np.float64(t_start).tobytes())
        h.update(np.asarray(head, dtype=np.float64).tobytes())
        h.update(np.asarray(goal, dtype=np.float64).tobytes())
        h.update(np.float64(yaw).tobytes())
        h.update(np.float64(frame.stamp).tobytes())
        h.update(np.ascontiguousarray(np.asarray(frame.depth)).tobytes())
        return h.hexdigest()

    def _fit_yopo(self, raw: PiecewisePolynomial) -> PiecewisePolynomial | None:
        """从长到短取过运动学的前缀。系数不变，避障形状还是这条 YOPO 曲线。"""
        total = raw.t_end - raw.t_start
        for horizon in (min(self._commit_span(), total), 0.7, 0.45, 0.3):
            if horizon > total + 1e-9 or horizon < 0.2:
                continue
            try:
                traj = _append_stop(_prefix(raw, raw.t_start + horizon), self.pcfg.limits)
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                continue
            if traj is not None and _kin_ok(traj, self.pcfg.limits):
                return traj
        return None

    def _fit_horizon(self, raw: PiecewisePolynomial, horizon: float, limits: KinematicLimits) -> PiecewisePolynomial | None:
        total = raw.t_end - raw.t_start
        horizon = min(float(horizon), total)
        if horizon < 0.2:
            return None
        try:
            traj = _append_stop(_prefix(raw, raw.t_start + horizon), limits)
        except (ValueError, np.linalg.LinAlgError, FloatingPointError):
            return None
        if traj is not None and _kin_ok(traj, self.pcfg.limits):
            return traj
        return None

    def _fit_short(self, raw: PiecewisePolynomial) -> PiecewisePolynomial | None:
        """超前机的短前缀。尾巴用场景 a_max 停住。"""
        return self._fit_horizon(raw, _LEADER_PREFIX_S, self.gentle_limits)

    def _gentle_hover(self, head: np.ndarray, t_start: float) -> PiecewisePolynomial | None:
        """已经停住就保持。还在走就按场景 a_max 刹停，停点之后由 state_at 保持。"""
        if float(np.linalg.norm(head[1])) <= 1e-6 and float(np.linalg.norm(head[2])) <= 1e-6:
            return hold_traj(head[0], t_start, t_start + max(self._commit_span(), 0.5))
        return _brake_motion(head, t_start, self.gentle_limits)

    def _leader_proposal(self, node: ProtocolNode, t_start: float, head: np.ndarray, goal: np.ndarray, frame, yaw: float):
        """超前机只提悬停，或一条短前缀。不提交盖住下一轮窗口的长曲线。"""
        hover = self._gentle_hover(head, t_start)
        if hover is not None and self._clearance_ok(hover):
            occ, _reject = self._as_plan(node, t_start, head, hover)
            if occ is not None:
                self._count(node.id, "LEADER_HOVER")
                return occ
        try:
            motions, code = self._yopo_options(head, t_start, goal, frame.depth, yaw, short=True)
        except (ValueError, np.linalg.LinAlgError, FloatingPointError):
            motions, code = [], "YOPO_FAIL"
        if code == "CORRIDOR_EMPTY_HOVER":
            self.empty_solution += 1
        for motion in motions:
            if not self._clearance_ok(motion):
                continue
            occ, _reject = self._as_plan(node, t_start, head, motion)
            if occ is not None:
                self._count(node.id, "LEADER_PREFIX")
                return occ
        self._count(node.id, "LEADER_HOLD")
        return None

    def _yopo_options(self, head: np.ndarray, t_start: float, goal: np.ndarray, depth, yaw: float, *, short: bool = False):
        """一次前向。首位是上游 select_action；预留冲突时才按分数试后面的准入候选。"""
        R = R_yaw(yaw)
        params, score, radius = self.source.forward_raw(depth, head[1], head[2], goal, head[0], R)
        dec = self.decoder
        inner_b, tail_b, durations = decode_params(params, dec)
        inner_w = inner_b @ R.T + head[0]
        tail_w = np.einsum("ij,nkj->nki", R, tail_b)
        tail_w[:, 0] += head[0]
        ranking = rank_candidates(score, radius, dec, inner_w)
        if ranking.corridor_empty:
            return [], "CORRIDOR_EMPTY_HOVER"
        ranking = self._apply_inter_rank(ranking, score, head, t_start, tail_w, inner_w, durations, dec, short=short)
        if ranking is None:
            # 可行集空：与走廊为空同一返回值。调用方接着走已有的刹车或保持上一份计划。
            return [], "CORRIDOR_EMPTY_HOVER"
        motions = []
        refined = False
        for index in ranking.order[:6]:
            try:
                raw = minco_two_piece(t_start, head, tail_w[index], inner_w[index], durations[index])
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                continue
            traj = self._fit_short(raw) if short else self._fit_yopo(raw)
            if traj is None:
                continue
            if self._backend is not None and not refined:
                traj = self._apply_backend(traj, goal)
                refined = True
            motions.append(traj)
        if not motions:
            return [], "YOPO_KIN_FALLBACK"
        return motions, "OK"

    def _apply_backend(self, traj: PiecewisePolynomial, goal: np.ndarray) -> PiecewisePolynomial:
        from .backend.bspline import refine_yopo

        neighbors = None
        if self._backend.swarm_penalty and self._score_node is not None:
            neighbors = committed_neighbor_samples(self._score_node, float(traj.t_start))
        out, rec = refine_yopo(traj, self._backend, self._backend_obstacles, goal, neighbors=neighbors)
        self.backend_log.append(rec)
        return out

    def _as_plan(self, node: ProtocolNode, t_start: float, head: np.ndarray, motion: PiecewisePolynomial):
        res = make_plan(
            node.id,
            node.plan_seq + 1,
            node.committed.plan_hash,
            motion,
            t_start,
            head,
            [t_start],
            self.pcfg,
        )
        if not isinstance(res, Plan):
            return None, "CONTRACT_" + "+".join(item.value for item in res)
        occ = PlanOccupancy(res, self.pcfg)
        if self._conflicts_with_live(node, occ):
            return None, "CONFLICT_SKIP"
        return occ, None

    def _conflicts_with_live(self, node: ProtocolNode, occ: PlanOccupancy) -> bool:
        """提案前的自检，口径对齐邻机投票。

        邻机 ACK 时只拿自己的 committed + previous（own_possible）和已 ACK 的预留来比。
        本机的 peers[m].plans 却留着该邻机全部历史计划：截断只砍名义段，刹车分支的 Hold 到 inf，
        被替换的旧计划停点永远挡路。六机穿林里超前机悬停留下的这些停点，正好堵住落后机的通道。
        这里每个邻机只看最后两份已提交计划（等于它的 committed + previous）加预留。三机和柱林不变。
        """
        if not self._forest_six():
            return node.conflicts_with_neighbors(occ)
        for _m, peer in sorted(node.peers.items()):
            recs = list(peer.plans[-2:]) + [r for _, r in sorted(peer.reservations.items())]
            if self.cfg.delay_check_s > 0:
                recs += [r for _, r in sorted(peer.heard.items())]
            if any(node.checker.conflicts(occ, r.occupancy) for r in recs):
                return True
        return False

    def _apply_inter_rank(self, ranking, score, head, t_start, tail_w, inner_w, durations, dec, *, short: bool):
        """开关打开时，用邻机最后一份已提交采样重排准入候选。关或没有采样时原样返回。"""
        if self._inter is None or self._score_node is None:
            return ranking
        peers = committed_neighbor_samples(self._score_node, self._score_t)
        if not peers:
            return ranking
        clear = np.full(int(dec.count), np.inf)
        for index in ranking.order:
            try:
                raw = minco_two_piece(t_start, head, tail_w[index], inner_w[index], durations[index])
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                continue
            fitted = self._fit_short(raw) if short else self._fit_yopo(raw)
            if fitted is None:
                continue
            clear[index] = clearance_m(sample_motion(fitted, t_start), peers)
        pick = inter_pick(score, np.asarray(ranking.admitted, dtype=bool), clear, self._inter)
        if pick.kind == "unchanged":
            return ranking
        if pick.kind == "empty":
            self._inter_empty += 1
            self._inter_filtered += pick.filtered
            return None
        self._inter_filtered += pick.filtered
        self._inter_penalized += pick.penalized
        from yopo_swarm_core.candidates import Ranking

        return Ranking(pick.order, ranking.mu_lo, ranking.admitted, corridor_empty=False)

    def inter_drone_report(self) -> dict | None:
        if self._inter is None:
            return None
        return report_block(self._inter, self._inter_filtered, self._inter_penalized, self._inter_empty)

    def _planner(self, node: ProtocolNode, now: float, t_start: float):
        if self._inter is not None:
            self._score_node = node
            self._score_t = float(t_start)
        parent = node.committed.occupancy.plan
        head = np.array(state_at(parent, t_start), dtype=float, copy=True)
        sense = self._sense.get(node.id)
        if sense is None or sense[0] is None:
            self._count(node.id, "NO_DEPTH")
            return None
        frame, yaw = sense
        final = self.goals[node.id]
        goal = self.goal_for(node.id, t_start, head[0])
        if self.record_inputs:
            self.input_digests.append(self._digest(node.id, t_start, head, goal, yaw, frame))
        parked = float(np.linalg.norm(parent.terminal_point() - final)) <= 0.05 and float(np.linalg.norm(head[1])) <= 0.05
        if parked:
            self._count(node.id, "ARRIVED_HOLD")
            return None
        # 到了终点槽位仍走原来的停住。没到、又超前于低分位的，只许悬停或短前缀。
        if self._ahead_of_gate(node.id, head[0]) and float(np.linalg.norm(head[0] - final)) >= self.arrive_radius:
            return self._leader_proposal(node, t_start, head, goal, frame, yaw)
        code = "OK"
        motions: list[PiecewisePolynomial] = []
        goal_list = [goal]
        arriving = float(np.linalg.norm(head[0] - final)) < self.arrive_radius
        if arriving:
            motion = _stop_at(head, t_start, final, self.pcfg.limits)
            if motion is not None and not self._clearance_ok(motion):
                # 停到槽位的这段已经进树：不提交。继续飞上一份计划。
                self._count(node.id, "CLEARANCE_SKIP")
                return None
            if motion is not None:
                motions = [motion]
                code = "ARRIVED_HOLD"
            else:
                code = "ARRIVE_KIN_FALLBACK"
        elif self._elastic_active(float(head[0][0])):
            goal_list = self._elastic_goals(head[0], final, float(self.slots[node.id][1]))
        occ = None
        reject = None
        used_alt = False
        shifted = False
        if arriving:
            occ, reject, used_alt = self._take_motions(node, t_start, head, motions)
        else:
            for gi, g in enumerate(goal_list):
                try:
                    motions, code = self._yopo_options(head, t_start, g, frame.depth, yaw)
                except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                    motions, code = [], "YOPO_FAIL"
                if code == "CORRIDOR_EMPTY_HOVER":
                    # 和原来一样：这一次前向没有准入走廊，记一次无解。换一条界内目标再试。
                    if gi == 0:
                        self.empty_solution += 1
                    reject = code
                    continue
                occ, reject, used_alt = self._take_motions(node, t_start, head, motions)
                if occ is not None:
                    shifted = gi > 0
                    break
        if occ is None:
            # 最高分和后面几个准入候选都和邻机预留重叠，或净空不够：不硬挤。
            # 能刹车就停在自己的管子里。刹车要是已经进树，也不提交。
            brake = _brake_motion(head, t_start, self.pcfg.limits)
            if brake is not None and not self._clearance_ok(brake):
                brake = None
                reject = "CLEARANCE_SKIP"
            if brake is not None:
                occ, reject2 = self._as_plan(node, t_start, head, brake)
                if occ is not None:
                    self._count(node.id, "CONFLICT_BRAKE" if reject == "CONFLICT_SKIP" else "BRAKE_ONLY")
                    return occ
                reject = reject2 or reject
            self._count(node.id, "CONFLICT_SKIP" if reject == "CONFLICT_SKIP" else (reject or code))
            return None
        if shifted:
            self._count(node.id, "SLOT_SHIFT")
        else:
            self._count(node.id, "YOPO_ALT" if used_alt else code)
        return occ

    def _take_motions(self, node: ProtocolNode, t_start: float, head: np.ndarray, motions: list[PiecewisePolynomial]):
        """净空不够的多项式不进 _as_plan，也就不会被提案、不会 COMMIT。"""
        occ = None
        reject = None
        used_alt = False
        for i, motion in enumerate(motions):
            if not self._clearance_ok(motion):
                reject = "CLEARANCE_SKIP"
                continue
            occ, reject = self._as_plan(node, t_start, head, motion)
            if occ is not None:
                used_alt = i > 0
                break
        return occ, reject, used_alt

    def _emit(self, bus, now: float, src: str, msg) -> None:
        dests = [msg.dest] if getattr(msg, "dest", None) else [m for m in self.members if m != src]
        for dest in dests:
            bus.publish(
                now,
                src=src,
                dest=dest,
                topic="protocol",
                payload={"kind": "protocol", "msg": clone_msg(msg)},
            )

    def exchange(self, bus, now: float) -> None:
        if self.geom.enabled:
            self.geom.flush(bus, now)
        for did in self.members:
            for msg in self.nodes[did].on_tick(now):
                self._emit(bus, now, did, msg)
        for _ in range(8):
            envs = bus.poll(now)
            if not envs:
                return
            for env in envs:
                if env.payload.get("kind") != "protocol":
                    if self.geom.enabled and env.payload.get("kind") == KIND:
                        self.geom.ingest_envelope(env)
                    continue
                self.deliveries += 1
                for reply in self.nodes[env.dest].on_message(env.payload["msg"], now):
                    self._emit(bus, now, env.dest, reply)
        raise RuntimeError("协议消息在一个 tick 内没有投递完")

    def track(self, drone_id: str, now: float):
        rec = self.nodes[drone_id].executing(now)
        self._last_rec[drone_id] = rec
        if self.geom.enabled:
            alt = self.geom.override_sampler(drone_id)
            if alt is not None:
                return alt
        slot = self._follow[drone_id]
        if slot["hash"] != rec.plan_hash:
            yaw = 0.0 if slot["sampler"] is None else slot["sampler"].yaw
            slot["sampler"] = _Track(rec.occupancy.plan, self.goals[drone_id], yaw, self.dt)
            slot["hash"] = rec.plan_hash
        return slot["sampler"]

    def shadow_view(self, drone_id: str):
        rec = self.nodes[drone_id].committed
        slot = self._follow[drone_id]
        if slot.get("commit_hash") == rec.plan_hash:
            return None
        slot["commit_hash"] = rec.plan_hash
        yaw = 0.0 if slot["sampler"] is None else slot["sampler"].yaw
        return _Track(rec.occupancy.plan, self.goals[drone_id], yaw, self.dt)

    def last_record(self, drone_id: str):
        return self._last_rec[drone_id]

    def code(self, drone_id: str) -> str:
        return self._last_code[drone_id]

    def observe(self, drone_id: str, t: float, position: np.ndarray) -> float:
        pos = np.asarray(position, dtype=float)
        self.note_progress(drone_id, pos)
        slot = self.slots[drone_id]
        if self.progress_gate:
            ref = self.rail.at(self.gate_s(), slot)
        else:
            ref = self.rail.position(t, self.t_go, slot)
        err = float(np.linalg.norm(pos - ref))
        self.errors.append(err)
        return err

    def summary(self, arrived: int, n_drones: int) -> dict:
        commits: dict[str, int] = {}
        proposals = acks = nacks = 0
        hashes = {}
        decisions: dict[str, int] = {}
        for did, node in self.nodes.items():
            n_commit = 0
            for event in node.events:
                kind = event["event"]
                if kind == "commit":
                    n_commit += 1
                elif kind == "propose":
                    proposals += 1
                elif kind == "vote":
                    if event.get("decision") == "ACK":
                        acks += 1
                    else:
                        nacks += 1
            commits[did] = n_commit
            hashes[did] = node.committed.plan_hash
            for key, value in self._codes[did].items():
                decisions[key] = decisions.get(key, 0) + int(value)
        arr = np.asarray(self.errors, dtype=float)
        return {
            "error_p95_m": None if arr.size == 0 else float(np.percentile(arr, 95)),
            "error_max_m": None if arr.size == 0 else float(arr.max()),
            "arrival_rate": float(arrived) / float(n_drones),
            "frozen": self.frozen,
            "adjudicator": None,
            "avoidance": "upstream-yopo",
            "goal_bias": "elastic-slot" if self._forest_six() else "slot-lookahead-blend",
            "clearance_margin_m": self.clearance_margin_m,
            "clearance_required_m": self._required_clearance(),
            "commit_span_s": self._commit_span(),
            "progress_gate": self.progress_gate,
            "progress_percentile": self.progress_percentile,
            "gate_s_m": self.gate_s() if self.progress_gate else None,
            "leader_margin_m": _LEADER_MARGIN_M if self.progress_gate else None,
            "lateral_slack_m": self.lateral_slack_m,
            "longitudinal_slack_m": self.longitudinal_slack_m,
            "body_radius_m": self.radius,
            "safe_radius_m": self.safe_radius_m,
            "empty_solution_count": int(self.empty_solution),
            "local_commits": commits,
            "proposals": proposals,
            "acks": acks,
            "nacks": nacks,
            "decisions": dict(sorted(decisions.items())),
            "window_len_s": self.cfg.window_len,
            "commit_horizon_s": self.horizon,
            "lookahead_s": self.lookahead_s,
            "t_go_s": self.t_go,
            "motion_duration_s": self.rail.T,
            "final_slots_enu": {did: self.goals[did].tolist() for did in self.members},
            "committed_plan_hash": hashes,
            "virtual_center": "progress-gated" if self.progress_gate else "straight-rail",
        }


def _crossing_verdict(rows: list[dict]) -> str:
    frozen = all(bool((r.get("crossing") or {}).get("frozen", False)) for r in rows)
    return "判据已冻结" if frozen else "未冻结 / report-only"


def _crossing_title(rows: list[dict], heading: str) -> str:
    return f"### {heading}（{_crossing_verdict(rows)}）"


def format_report(rows: list[dict], *, title: str | None = None) -> str:
    """票 07 / 07b 结果表。碰撞是硬失败；标题按本批是否冻结来写。"""
    rows = [r for r in rows if r.get("task") == "crossing"]
    if not rows:
        return ""
    densities = []
    for row in rows:
        density = row.get("forest_density")
        if density not in densities:
            densities.append(density)
    if len(densities) > 1:
        parts = []
        for density in densities:
            subset = [row for row in rows if row.get("forest_density") == density]
            heading = f"票 07b {density}" if density else "票 07"
            parts.append(format_report(subset, title=_crossing_title(subset, heading)))
        return "\n\n".join(parts)
    if title is None and rows[0].get("forest_density"):
        title = _crossing_title(rows, f"票 07b {rows[0]['forest_density']}")
    lines = [
        title or _crossing_title(rows, "票 07 三机交叉避障"),
        "",
        "净空 = 机体中心到障碍表面的距离 − body_radius。碰撞（障碍或机间）是硬失败，仿真在第一次碰撞处停下。",
        "",
        "| seed | 到达率 | 障碍碰撞 | 机间碰撞 | 最小机间距离 m | 最小障碍净空 m | 队形误差 p95 m | 影子拒绝率 | safe_radius m | 偏离上游 | 无解次数 | 失败原因 |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    obs_sum = inter_sum = 0
    for r in rows:
        block = r.get("crossing") or {}
        radius = float(block.get("body_radius_m", 0.35))
        n = int(r.get("n_drones") or 0)
        got = int(r.get("arrived") or 0)
        rate = r.get("arrival_rate")
        rate_cell = "—" if rate is None else f"{float(rate):.2f} ({got}/{n})"
        if "obstacle_collisions" in block:
            obs = int(block["obstacle_collisions"])
            inter = int(block["inter_drone_collisions"])
        else:
            dist = r.get("min_obstacle_distance_m")
            gap = r.get("min_inter_drone_distance_m")
            obs = int(dist is not None and dist <= radius + 1e-9)
            inter = int(gap is not None and gap <= 2.0 * radius + 1e-9)
        obs_sum += obs
        inter_sum += inter
        clearance = block.get("min_obstacle_clearance_m")
        if clearance is None and r.get("min_obstacle_distance_m") is not None:
            clearance = float(r["min_obstacle_distance_m"]) - radius
        reason = "" if r.get("outcome") == "SUCCESS" else f"{r.get('outcome')} {r.get('reason')}"
        rad = r.get("safe_radius_m")
        if rad is None:
            rad_cell, dev = "—", "—"
        else:
            rad_cell = f"{float(rad):.2f}"
            dev = "偏离上游" if r.get("upstream_deviation") else "否"
        empty = r.get("empty_solution_count")
        empty_cell = "—" if empty is None else str(int(empty))
        lines.append(
            "| {seed} | {rate} | {obs} | {inter} | {gap} | {clear} | {p95} | {shadow} | {rad} | {dev} | {empty} | {why} |".format(
                seed=r.get("seed"),
                rate=rate_cell,
                obs=obs,
                inter=inter,
                gap=_fmt(r.get("min_inter_drone_distance_m")),
                clear=_fmt(clearance),
                p95=_fmt(r.get("formation_error_p95_m")),
                shadow=_fmt(r.get("shadow_reject_rate")),
                rad=rad_cell,
                dev=dev,
                empty=empty_cell,
                why=reason,
            )
        )
    frozen = all(bool((r.get("crossing") or {}).get("frozen", False)) for r in rows)
    lines += [
        "",
        f"合计 {len(rows)} 个 seed：障碍碰撞 {obs_sum}，机间碰撞 {inter_sum}。"
        f"{'判据已冻结' if frozen else '未冻结 / report-only'}。",
    ]
    failed = [r for r in rows if r.get("outcome") != "SUCCESS"]
    if failed:
        lines.append("")
        lines.append("未到达或碰撞的 seed：")
        for r in failed:
            decisions = (r.get("crossing") or {}).get("decisions") or {}
            shown = ", ".join(f"{k}:{v}" for k, v in decisions.items()) or "—"
            lines.append(
                f"- seed {r.get('seed')}: {r.get('outcome')} {r.get('reason')}；"
                f"终点误差 {_fmt(r.get('final_goal_error_m'))} m；决策 {shown}"
            )
    return "\n".join(lines)


def _fmt(x, nd=3):
    return "—" if x is None else f"{float(x):.{nd}f}"
