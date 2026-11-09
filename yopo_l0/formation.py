"""票 06：三机编队保持。虚拟中心沿开阔区「直线 + 平滑转弯 + 直线」走，槽位刚性挂在航向上。

每架飞机一个 vendored `ProtocolNode`（轮转提案窗口 + 预留 ACK/COMMIT/RELEASE）。
运动更新只在本机窗口里提案；邻机用几何占用判冲突。本机只有收齐 N-1 个 ACK 并
在本地 COMMIT 之后，动力学才跟踪这份计划。没有中央裁决者，也不读邻机里程计。

同步出发：成员 i 的提案 t_start = (i+1)·窗口长，末机刚好在 t_go 起步。先提案的飞机
在自己的计划里把 [t_start, t_go) 留成悬停，所以三机按同一套槽位时刻表一起动。
这是写在场景卡里的任务配置，每机各自算一遍，不是运行时的中央调度。
"""

from __future__ import annotations

import copy
import hashlib
import math
from dataclasses import replace

import numpy as np

from yopo_swarm_core.collision import Verdict, check_derivative_bound
from yopo_swarm_core.plan import BrakeBranch, KinematicLimits, Plan, PlanConfig, make_plan, quintic, state_at
from yopo_swarm_core.protocol.messages import Proposal, digest
from yopo_swarm_core.protocol.node import ProtocolConfig, ProtocolNode
from yopo_swarm_core.protocol.plan_occupancy import PlanConflictChecker, PlanOccupancy, hover_occupancy
from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment

# 最小急动度标量曲线 σ(u)=10u³-15u⁴+6u⁵ 的速度峰值（u=1/2）。
_MINJERK_V_PEAK = 1.875
_FIT_TOL_M = 0.02
_MAX_SEGMENTS = 240


class LocalApproval:
    """本机批准码。只记本机节点的状态，不代表编队级裁决。"""

    def __init__(self) -> None:
        self.last_code = "LOCAL_HOVER"
        self.extra_codes: dict = {}


def _smooth01(u: float) -> tuple[float, float, float]:
    """u∈[0,1] 上 0→1 的最小急动度曲线，及其对 u 的一二阶导数。两端值为 0。"""
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    sig = 10.0 * u3 - 15.0 * u4 + 6.0 * u4 * u
    dsig = 30.0 * u2 - 60.0 * u3 + 30.0 * u4
    d2 = 60.0 * u - 180.0 * u2 + 120.0 * u3
    return sig, dsig, d2


def _unit_slot(raw) -> np.ndarray:
    slot = np.asarray(raw, dtype=float)
    if slot.shape != (3,):
        raise ValueError(f"slot_body 必须是长度 3，收到 {raw}")
    return slot


class _CenterPath:
    """水平虚拟中心：直线、航向最小急动度转到新方向、再直线。位置对弧长是 C2 的。"""

    def __init__(self, spec: dict):
        origin = np.asarray(spec["origin_enu"], dtype=float)
        if origin.shape != (3,):
            raise ValueError("virtual_center.origin_enu 必须是长度 3")
        self.origin = origin
        self.straight0 = float(spec["straight_m"])
        self.turn_len = float(spec["turn_m"])
        self.straight1 = float(spec["final_straight_m"])
        if min(self.straight0, self.turn_len, self.straight1) <= 0.0:
            raise ValueError("直线段和转弯段长度必须为正")
        turn = str(spec.get("turn", "left"))
        if turn not in ("left", "right"):
            raise ValueError("turn 只能是 left 或 right")
        sign = 1.0 if turn == "left" else -1.0
        self.turn_angle = sign * math.radians(float(spec.get("turn_deg", 90.0)))
        self.s_turn0 = self.straight0
        self.s_turn1 = self.straight0 + self.turn_len
        self.length = self.s_turn1 + self.straight1
        speed = float(spec["speed_mps"])
        if speed <= 0.0:
            raise ValueError("speed_mps 必须为正")
        self.speed = speed
        self.T = _MINJERK_V_PEAK * self.length / speed
        n = max(2, int(round(self.length / 5e-4)))
        ds = self.length / n
        s = np.linspace(0.0, self.length, n + 1)
        tang = np.stack([self._tangent(float(si)) for si in s], axis=0)
        delta = 0.5 * (tang[:-1] + tang[1:]) * ds
        p = np.empty_like(tang)
        p[0] = self.origin
        p[1:] = np.cumsum(delta, axis=0) + self.origin
        self._s = s
        self._p = p

    def _heading(self, s: float) -> tuple[float, float, float]:
        if s <= self.s_turn0:
            return 0.0, 0.0, 0.0
        if s >= self.s_turn1:
            return self.turn_angle, 0.0, 0.0
        u = (s - self.s_turn0) / self.turn_len
        sig, dsig, d2 = _smooth01(u)
        psi = self.turn_angle * sig
        kappa = self.turn_angle * dsig / self.turn_len
        dkappa = self.turn_angle * d2 / (self.turn_len ** 2)
        return psi, kappa, dkappa

    def _tangent(self, s: float) -> np.ndarray:
        psi, _, _ = self._heading(s)
        return np.array([math.cos(psi), math.sin(psi), 0.0])

    def _center(self, s: float) -> np.ndarray:
        if s <= 0.0:
            return self.origin.copy()
        if s >= self.length:
            return self._p[-1].copy()
        return np.array([float(np.interp(s, self._s, self._p[:, i])) for i in range(3)])

    def _sigma(self, tau: float) -> tuple[float, float, float]:
        if tau <= 0.0:
            return 0.0, 0.0, 0.0
        if tau >= self.T:
            return self.length, 0.0, 0.0
        sig, dsig, d2 = _smooth01(tau / self.T)
        return (
            self.length * sig,
            self.length * dsig / self.T,
            self.length * d2 / (self.T ** 2),
        )

    def state_along(self, s: float, sp: float, sa: float, slot: np.ndarray) -> np.ndarray:
        """弧长 s、速度 sp、加速度 sa 处，槽位的世界系 PVA。κ 是航向对弧长的导数。"""
        tangent, left, kappa, dkappa = self._frame(s)
        sx, sy, sz = (float(x) for x in slot)
        u_vec = sx * left - sy * tangent
        d_off = kappa * u_vec
        du_ds = -kappa * (sx * tangent + sy * left)
        off = sx * tangent + sy * left
        off = np.array([off[0], off[1], sz])
        p = self._center(s) + off
        v = sp * tangent + sp * d_off
        a = (
            sa * tangent
            + (kappa * sp * sp) * left
            + sa * d_off
            + (dkappa * sp * sp) * u_vec
            + (kappa * du_ds * sp * sp)
        )
        return np.stack([p, v, a])

    def _frame(self, s: float):
        psi, kappa, dkappa = self._heading(s)
        tangent = np.array([math.cos(psi), math.sin(psi), 0.0])
        left = np.array([-math.sin(psi), math.cos(psi), 0.0])
        return tangent, left, kappa, dkappa

    def pva_at(self, t: float, t_go: float, slot: np.ndarray) -> np.ndarray:
        if t <= t_go:
            return self.state_along(0.0, 0.0, 0.0, slot)
        if t >= t_go + self.T:
            return self.state_along(self.length, 0.0, 0.0, slot)
        s, sp, sa = self._sigma(t - t_go)
        return self.state_along(s, sp, sa, slot)

    def check_derivatives(self, slot: np.ndarray) -> None:
        """用中心差分对一下槽位速度和加速度，公式写错时立刻失败。"""
        step = 2e-3
        samples = (
            0.4 * self.straight0,
            self.s_turn0 + 0.35 * self.turn_len,
            self.s_turn0 + 0.65 * self.turn_len,
            self.s_turn1 + 0.4 * self.straight1,
        )
        for s in samples:
            if not (step < s < self.length - step):
                continue
            p0 = self.state_along(s - step, 1.0, 0.0, slot)[0]
            p1 = self.state_along(s + step, 1.0, 0.0, slot)[0]
            v = self.state_along(s, 1.0, 0.0, slot)[1]
            if float(np.linalg.norm((p1 - p0) / (2 * step) - v)) > 5e-2:
                raise RuntimeError(f"槽位速度和位置差分不一致，s={s}")
            v0 = self.state_along(s - step, 1.0, 0.0, slot)[1]
            v1 = self.state_along(s + step, 1.0, 0.0, slot)[1]
            a = self.state_along(s, 1.0, 0.0, slot)[2]
            if float(np.linalg.norm((v1 - v0) / (2 * step) - a)) > 0.5:
                raise RuntimeError(f"槽位加速度和速度差分不一致，s={s}")


def _kin_ok(traj: PiecewisePolynomial, limits: KinematicLimits) -> bool:
    return all(
        check_derivative_bound(traj, order, lim).verdict is Verdict.SAFE
        for order, lim in limits.by_order()
    )


def _fit_interval(path: _CenterPath, t0: float, t1: float, t_go: float, slot: np.ndarray, limits: KinematicLimits, depth: int):
    pva0 = path.pva_at(t0, t_go, slot)
    pva1 = path.pva_at(t1, t_go, slot)
    seg = quintic(pva0, pva1, t1 - t0)
    traj = PiecewisePolynomial(t0, [seg])
    tm = 0.5 * (t0 + t1)
    err = float(np.linalg.norm(traj.evaluate(tm) - path.pva_at(tm, t_go, slot)[0]))
    ok = err <= _FIT_TOL_M and _kin_ok(traj, limits)
    if ok:
        return [seg]
    if depth >= 8 or (t1 - t0) <= 0.08:
        raise RuntimeError(f"槽位轨迹拟合失败 t=[{t0}, {t1}] 中点误差 {err:.3f} m，运动学 {'通过' if _kin_ok(traj, limits) else '未通过'}")
    return _fit_interval(path, t0, tm, t_go, slot, limits, depth + 1) + _fit_interval(
        path, tm, t1, t_go, slot, limits, depth + 1
    )


def build_motion(path: _CenterPath, t_start: float, t_go: float, slot: np.ndarray, head: np.ndarray, limits: KinematicLimits):
    """从父计划在 t_start 的 PVA 接到共享时刻表。t_start < t_go 时前面悬停。"""
    head = np.asarray(head, dtype=float)
    at_go = path.pva_at(t_go, t_go, slot)
    if float(np.linalg.norm(at_go[0] - head[0])) > 1e-4 or float(np.linalg.norm(at_go[1])) > 1e-4:
        raise RuntimeError("t_go 处的槽位和父计划悬停点对不上")
    segs: list[PolySegment] = []
    if t_go > t_start + 1e-9:
        coeff = np.zeros((3, 1))
        coeff[:, 0] = head[0]
        segs.append(PolySegment(coeff, t_go - t_start))
    t_end = t_go + path.T
    knots = [t_go]
    t = t_go
    while t < t_end - 1e-9:
        t = min(t_end, t + 0.5)
        knots.append(t)
    # 第一段用父计划的 PVA，避免悬停多项式和解析槽位差 1 个 ulp 就被 HEAD_DISCONTINUITY 拒掉。
    forced = {t_go: np.array(head, dtype=float, copy=True)}
    for a, b in zip(knots, knots[1:]):
        if a in forced:
            pva0 = forced[a]
            pva1 = path.pva_at(b, t_go, slot)
            seg = quintic(pva0, pva1, b - a)
            traj = PiecewisePolynomial(a, [seg])
            err = float(np.linalg.norm(traj.evaluate(0.5 * (a + b)) - path.pva_at(0.5 * (a + b), t_go, slot)[0]))
            if err <= _FIT_TOL_M and _kin_ok(traj, limits):
                segs.append(seg)
                continue
        segs.extend(_fit_interval(path, a, b, t_go, slot, limits, 0))
    if len(segs) > _MAX_SEGMENTS:
        raise RuntimeError(f"槽位多项式段数 {len(segs)} 超过 {_MAX_SEGMENTS}")
    return PiecewisePolynomial(t_start, segs)


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
    """总线深拷贝不会拆开 PlanOccupancy（它的 __deepcopy__ 返回自身）。这里先拆成独立计划。"""
    if isinstance(msg, Proposal):
        return replace(msg, occupancy=_clone_occ(msg.occupancy))
    return copy.deepcopy(msg)


class _Track:
    def __init__(self, plan: Plan, goal: np.ndarray, yaw: float, dt: float):
        self.plan = plan
        self.goal = np.asarray(goal, dtype=float)
        self.yaw = float(yaw)
        self.dt = float(dt)

    def sample(self, t: float):
        from yopo_swarm_core.yaw import calculate_yaw

        pva = state_at(self.plan, float(t))
        p, v, a = (np.asarray(x, dtype=float) for x in pva)
        self.yaw = float(calculate_yaw(v, self.goal - p, self.yaw, self.dt)[0])
        return p, v, a, self.yaw


class FormationFleet:
    def __init__(self, card: dict, physics_dt: float, *, record_inputs: bool):
        task = card["task"]
        if task["type"] != "formation":
            raise ValueError("FormationFleet 只接受 formation 任务")
        drones = card["drones"]
        if len(drones) < 1:
            raise ValueError("编队至少一架飞机")
        self.members = tuple(str(d["id"]) for d in drones)
        if len(set(self.members)) != len(self.members):
            raise ValueError("无人机 id 重复")
        self.slots = {str(d["id"]): _unit_slot(d["slot_body"]) for d in drones}
        self.path = _CenterPath(task["virtual_center"])
        for slot in self.slots.values():
            self.path.check_derivatives(slot)
        self.dt = float(physics_dt)
        self.record_inputs = record_inputs
        self.input_digests: list[str] = []
        self.errors: list[float] = []
        self.deliveries = 0
        self.frozen = bool(card["criteria"].get("frozen", False))
        window = float(task.get("window_len_s", 0.5))
        if window <= 0.0:
            raise ValueError("window_len_s 必须为正")
        # 末机的 t_start 等于全体窗口转完一圈。此前提案的飞机在计划里悬停到这一刻。
        self.t_go = window * len(self.members)
        spec = task["virtual_center"]
        self.cfg = ProtocolConfig(
            members=self.members,
            window_len=window,
            task_hash=digest(
                {
                    "origin": spec["origin_enu"],
                    "straight_m": spec["straight_m"],
                    "turn_m": spec["turn_m"],
                    "final_straight_m": spec["final_straight_m"],
                    "turn": spec.get("turn", "left"),
                    "turn_deg": spec.get("turn_deg", 90.0),
                    "speed_mps": spec["speed_mps"],
                    "slots": {k: v.tolist() for k, v in self.slots.items()},
                    "window_len_s": window,
                }
            ),
            resend_period=None,
            heartbeat_period=None,
        )
        crit = card["criteria"]
        self.pcfg = PlanConfig(
            KinematicLimits(v_max=8.0, a_max=12.0, j_max=200.0),
            max_branch_points=2,
            body_radius=float(crit["body_radius_m"]),
            tracking_margin=0.15,
        )
        self.goals = {
            did: self.path.pva_at(self.t_go + self.path.T, self.t_go, slot)[0].copy()
            for did, slot in self.slots.items()
        }
        self.starts = {
            did: self.path.pva_at(0.0, self.t_go, slot)[0].copy() for did, slot in self.slots.items()
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

    @classmethod
    def from_card(cls, card: dict, physics_dt: float, *, record_inputs: bool = False) -> "FormationFleet":
        return cls(card, physics_dt, record_inputs=record_inputs)

    def _initial(self, owner: str, cfg: ProtocolConfig) -> PlanOccupancy:
        return hover_occupancy(owner, self.starts[owner], self.pcfg)

    def check_start(self, drone_id: str, start: np.ndarray) -> None:
        expect = self.starts[drone_id]
        if float(np.linalg.norm(np.asarray(start, dtype=float) - expect)) > 1e-6:
            raise ValueError(f"{drone_id} 的 start_enu {start.tolist()} 不是槽位在虚拟中心起点的位置 {expect.tolist()}")

    def goal_of(self, drone_id: str) -> np.ndarray:
        return self.goals[drone_id].copy()

    def _digest(self, node: ProtocolNode, t_start: float, head: np.ndarray) -> str:
        h = hashlib.sha256()
        h.update(node.id.encode())
        h.update(np.float64(t_start).tobytes())
        h.update(np.asarray(head, dtype=np.float64).tobytes())
        for peer_id, peer in sorted(node.peers.items()):
            h.update(peer_id.encode())
            for rec in peer.plans:
                h.update(rec.plan_hash.encode())
            for rid in sorted(peer.reservations):
                h.update(rid.encode())
                h.update(peer.reservations[rid].plan_hash.encode())
        return h.hexdigest()

    def _planner(self, node: ProtocolNode, now: float, t_start: float):
        parent = node.committed.occupancy.plan
        head = state_at(parent, t_start)
        if self.record_inputs:
            self.input_digests.append(self._digest(node, t_start, head))
        if float(np.linalg.norm(parent.terminal_point() - self.goals[node.id])) <= 1e-4:
            return None
        if t_start > self.t_go + 1e-9:
            return None  # 错过同步出发，继续执行已批准的悬停，不临时插一条追赶
        motion = build_motion(self.path, t_start, self.t_go, self.slots[node.id], head, self.pcfg.limits)
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
            raise RuntimeError(f"{node.id} 的编队计划没通过计划契约：{res}")
        occ = PlanOccupancy(res, self.pcfg)
        if node.conflicts_with_neighbors(occ):
            return None
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

    def last_record(self, drone_id: str):
        return self._last_rec[drone_id]

    def code(self, drone_id: str) -> str:
        rec = self.last_record(drone_id)
        return "LOCAL_APPROVED" if rec.plan_seq > 0 else "LOCAL_HOVER"

    def observe(self, drone_id: str, t: float, position: np.ndarray) -> float:
        ref = self.path.pva_at(t, self.t_go, self.slots[drone_id])[0]
        err = float(np.linalg.norm(np.asarray(position, dtype=float) - ref))
        self.errors.append(err)
        return err

    def summary(self, arrived: int, n_drones: int) -> dict:
        commits: dict[str, int] = {}
        proposals = acks = nacks = 0
        hashes = {}
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
        arr = np.asarray(self.errors, dtype=float)
        return {
            "error_p95_m": None if arr.size == 0 else float(np.percentile(arr, 95)),
            "error_max_m": None if arr.size == 0 else float(arr.max()),
            "arrival_rate": float(arrived) / float(n_drones),
            "frozen": self.frozen,
            "adjudicator": None,
            "local_commits": commits,
            "proposals": proposals,
            "acks": acks,
            "nacks": nacks,
            "window_len_s": self.cfg.window_len,
            "t_go_s": self.t_go,
            "motion_duration_s": self.path.T,
            "final_slots_enu": {did: self.goals[did].tolist() for did in self.members},
            "committed_plan_hash": hashes,
            "virtual_center": "scheduled",
        }
