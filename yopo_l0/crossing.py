"""票 07：三机交叉避障。队形尽力而为，避障优先。

每机在自己的提案窗口里，用深度图跑上游 YOPO（走廊准入 + 最高分），把编队槽位
融进目标当作偏置，不读场景几何。选出的多项式接到本机已提交计划在 t_start 的
状态上，经轮转窗口和预留 ACK/COMMIT 之后才交给动力学。跟邻机占用冲突就放弃
这次提案，继续飞本机已经批准的计划。没有中央裁决者。
"""

from __future__ import annotations

import copy
import hashlib
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
    make_plan,
    quintic,
    state_at,
)
from yopo_swarm_core.protocol.messages import Proposal, digest
from yopo_swarm_core.protocol.node import ProtocolConfig, ProtocolNode
from yopo_swarm_core.protocol.plan_occupancy import PlanConflictChecker, PlanOccupancy, hover_occupancy
from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment

from .policy import R_yaw, read_safe_radius, upstream_decoder

# 上游多项式的速度尖峰可以高出 vel_max。这里的限幅只为了计划契约肯收下那条曲线，
# 不是安全清单；动力学仍用场景卡里的 a_max。收不进就改走刹车，不飞一条没过契约的曲线。
_LIMITS = KinematicLimits(v_max=20.0, a_max=80.0, j_max=20000.0)


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

    def position(self, t: float, t_go: float, slot: np.ndarray) -> np.ndarray:
        tau = float(t) - float(t_go)
        if tau <= 0.0:
            s = 0.0
        elif tau >= self.T:
            s = self.length
        else:
            s = self.speed * tau
        sx, sy, sz = (float(x) for x in slot)
        return self.origin + np.array([s + sx, sy, sz])


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
        self.frozen = bool(card["criteria"].get("frozen", False))
        self.radius = float(card["criteria"]["body_radius_m"])
        window = float(task.get("window_len_s", 0.25))
        if window <= 0.0:
            raise ValueError("window_len_s 必须为正")
        self.window = window
        self.horizon = float(task.get("commit_horizon_s", 1.0))
        self.lookahead_s = float(task.get("lookahead_s", 2.0))
        self.min_ahead_m = float(task.get("min_ahead_m", 5.0))
        self.lateral_gain = float(task.get("lateral_gain", 0.25))
        self.arrive_radius = float(task.get("arrive_radius_m", 1.5))
        self.safe_radius_m = read_safe_radius(card.get("policy_params"))
        self.decoder = upstream_decoder(self.safe_radius_m)
        if min(self.horizon, self.lookahead_s, self.min_ahead_m) <= 0.0:
            raise ValueError("commit_horizon_s、lookahead_s、min_ahead_m 必须为正")
        if not 0.0 <= self.lateral_gain <= 2.0:
            raise ValueError("lateral_gain 必须在 [0, 2]")
        # 时刻表从 t=0 走。各机第一次能执行已是自己窗口的下一沿，起步差一两个窗口。
        self.t_go = float(task.get("t_go_s", 0.0))
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
            _LIMITS,
            max_branch_points=2,
            body_radius=self.radius,
            tracking_margin=0.15,
        )
        self.goals = {
            did: self.rail.position(self.t_go + self.rail.T, self.t_go, slot).copy()
            for did, slot in self.slots.items()
        }
        self.starts = {
            did: self.rail.position(0.0, self.t_go, slot).copy() for did, slot in self.slots.items()
        }
        checker = PlanConflictChecker()
        self.nodes = {
            did: ProtocolNode(
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
        """只收本机深度和航向。头状态稍后从本机已提交计划取，不用里程计位置冒充。"""
        self._sense[drone_id] = (frame, float(state.yaw))

    def goal_for(self, drone_id: str, t: float, head_p: np.ndarray) -> np.ndarray:
        """槽位前瞻和终点槽位的混合，再加一点拉回航线的侧向偏置。

        目标始终在机头前方、终点以内，避免网络看见身后的目标掉头。
        避障不在这里做：进网络的是这个目标和本机深度。
        """
        slot = self.slots[drone_id]
        head_p = np.asarray(head_p, dtype=float)
        final = self.goals[drone_id]
        ahead = self.rail.position(t + self.lookahead_s, self.t_go, slot)
        aim_x = min(float(final[0]), max(float(ahead[0]), float(head_p[0]) + self.min_ahead_m))
        aim = np.array([aim_x, float(ahead[1]), float(ahead[2])])
        remain = max(float(final[0] - ahead[0]), 0.0)
        reach = max(self.lookahead_s * self.rail.speed, 1e-6)
        w_final = 1.0 - min(remain / reach, 1.0)
        goal = (1.0 - w_final) * aim + w_final * final
        lane_y = float(self.rail.position(t, self.t_go, slot)[1])
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
        for horizon in (min(self.horizon, total), 0.7, 0.45, 0.3):
            if horizon > total + 1e-9 or horizon < 0.2:
                continue
            try:
                traj = _append_stop(_prefix(raw, raw.t_start + horizon), self.pcfg.limits)
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                continue
            if traj is not None and _kin_ok(traj, self.pcfg.limits):
                return traj
        return None

    def _yopo_options(self, head: np.ndarray, t_start: float, goal: np.ndarray, depth, yaw: float):
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
        motions = []
        for index in ranking.order[:6]:
            try:
                raw = minco_two_piece(t_start, head, tail_w[index], inner_w[index], durations[index])
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                continue
            traj = self._fit_yopo(raw)
            if traj is not None:
                motions.append(traj)
        if not motions:
            return [], "YOPO_KIN_FALLBACK"
        return motions, "OK"

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
        if node.conflicts_with_neighbors(occ):
            return None, "CONFLICT_SKIP"
        return occ, None

    def _planner(self, node: ProtocolNode, now: float, t_start: float):
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
        code = "OK"
        motions: list[PiecewisePolynomial] = []
        if float(np.linalg.norm(head[0] - final)) < self.arrive_radius:
            motion = _stop_at(head, t_start, final, self.pcfg.limits)
            if motion is not None:
                motions = [motion]
                code = "ARRIVED_HOLD"
            else:
                code = "ARRIVE_KIN_FALLBACK"
        else:
            try:
                motions, code = self._yopo_options(head, t_start, goal, frame.depth, yaw)
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                motions, code = [], "YOPO_FAIL"
            if code == "CORRIDOR_EMPTY_HOVER":
                self.empty_solution += 1
        occ = None
        reject = None
        used_alt = False
        for i, motion in enumerate(motions):
            occ, reject = self._as_plan(node, t_start, head, motion)
            if occ is not None:
                used_alt = i > 0
                break
        if occ is None:
            # 最高分和后面几个准入候选都和邻机预留重叠：不硬挤。能刹车就停在自己的管子里。
            brake = _brake_motion(head, t_start, self.pcfg.limits)
            if brake is not None:
                occ, reject2 = self._as_plan(node, t_start, head, brake)
                if occ is not None:
                    self._count(node.id, "CONFLICT_BRAKE" if reject == "CONFLICT_SKIP" else "BRAKE_ONLY")
                    return occ
                reject = reject2 or reject
            self._count(node.id, "CONFLICT_SKIP" if reject == "CONFLICT_SKIP" else (reject or code))
            return None
        self._count(node.id, "YOPO_ALT" if used_alt else code)
        return occ

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
        for did in self.members:
            for msg in self.nodes[did].on_tick(now):
                self._emit(bus, now, did, msg)
        for _ in range(8):
            envs = bus.poll(now)
            if not envs:
                return
            for env in envs:
                if env.payload.get("kind") != "protocol":
                    continue
                self.deliveries += 1
                for reply in self.nodes[env.dest].on_message(env.payload["msg"], now):
                    self._emit(bus, now, env.dest, reply)
        raise RuntimeError("协议消息在一个 tick 内没有投递完")

    def track(self, drone_id: str, now: float):
        rec = self.nodes[drone_id].executing(now)
        self._last_rec[drone_id] = rec
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
        ref = self.rail.position(t, self.t_go, self.slots[drone_id])
        err = float(np.linalg.norm(np.asarray(position, dtype=float) - ref))
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
            "goal_bias": "slot-lookahead-blend",
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
            "virtual_center": "straight-rail",
        }


def format_report(rows: list[dict], *, title: str | None = None) -> str:
    """票 07 / 07b 结果表。碰撞是硬失败；判据本身未冻结，只报数。"""
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
            parts.append(format_report(
                subset,
                title=f"### {heading}（判据未冻结 / report-only）",
            ))
        return "\n\n".join(parts)
    if title is None and rows[0].get("forest_density"):
        title = f"### 票 07b {rows[0]['forest_density']}（判据未冻结 / report-only）"
    lines = [
        title or "### 票 07 三机交叉避障（判据未冻结 / report-only）",
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
