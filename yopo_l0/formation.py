"""票 06：三机编队保持。虚拟中心沿开阔区「直线 + 平滑转弯 + 直线」走，槽位刚性挂在航向上。

每架飞机一个 vendored `ProtocolNode`（轮转提案窗口 + 预留 ACK/COMMIT/RELEASE）。
运动更新只在本机窗口里提案；邻机用几何占用判冲突。本机只有收齐 N-1 个 ACK 并
在本地 COMMIT 之后，动力学才跟踪这份计划。没有中央裁决者，也不读邻机里程计。

同步出发：成员 i 的提案 t_start = (i+1)·窗口长，末机刚好在 t_go 起步。先提案的飞机
在自己的计划里把 [t_start, t_go) 留成悬停，所以三机按同一套槽位时刻表一起动。
这是写在场景卡里的任务配置，每机各自算一遍，不是运行时的中央调度。

票 06b：这一轮 ACK 没到齐时，节点在 t_start 超时，按协议 RELEASE 并放弃该提案。
COMMIT/RELEASE 用 resend_period 从总线上重发，免得对端的预留一直占着槽位。
下一轮轮到本机，重新提案一条追赶：从当前悬停点沿本槽弧长接到时刻表（或终点保持），
速度和加速度不超过计划限幅。动力学仍然只跟踪本机已经 COMMIT 的计划。

票 06c：提案和 ACK 原来都不重传，六机一轮要 5 张票，丢 20% 时这一窗基本收不齐。
exchange 在截止前按 0.02 s 重发同一份提案，对端再 ACK。票齐了就停；截止前 0.10 s
也不再重发，避免时延副本掉进下一窗口被当成 NACK。追赶轨迹和跟踪参数不动。

票 13d：新轨走 PROPOSED → TENTATIVE → DELAY_CHECK → COMMITTED。δ_DC = 0.05 s，
等于故障档时延上界。Delay Check 没结束不提交；已提交的安全轨一直留着。
窗口内重发的是同一 reservation，Delay Check 作废后重提的是新 plan_seq。
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

from .criteria import frozen_for_card
from .geom_fallback import KIND, GeomSession, make_protocol_node
from .progress import read_progress_settings

# 最小急动度标量曲线 σ(u)=10u³-15u⁴+6u⁵ 的速度峰值（u=1/2）。
_MINJERK_V_PEAK = 1.875
_FIT_TOL_M = 0.02
_MAX_SEGMENTS = 240
# 窗口内重发同一份提案。0.02 s 在 0.5 s 窗口、截止前 0.10 s 静默时大约 20 次。
# guard 盖住故障档最大时延 0.05 s 的一个来回（提案 + ACK）。
_PROPOSAL_RESEND_PERIOD = 0.02
_PROPOSAL_RESEND_GUARD = 0.10
# RMADER δ_DC。故障档 delay 上界是 0.05 s，取等号。必须小于 window_len_s（0.5）。
_DELAY_CHECK_S = 0.05


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


def _arc_ref(path: _CenterPath, t: float, t_go: float) -> tuple[float, float, float]:
    """时刻表在 t 的弧长、弧长速度、弧长加速度。走完以后停在终点。"""
    if t <= t_go:
        return 0.0, 0.0, 0.0
    if t >= t_go + path.T:
        return path.length, 0.0, 0.0
    return path._sigma(t - t_go)


def _scalar_quintic(p0: float, v0: float, a0: float, p1: float, v1: float, a1: float, duration: float):
    """标量五次，两端 PVA。返回局部时间 u∈[0, duration] 上的 (值, 一阶, 二阶)。"""
    start = np.array([[p0, 0.0, 0.0], [v0, 0.0, 0.0], [a0, 0.0, 0.0]], dtype=float)
    end = np.array([[p1, 0.0, 0.0], [v1, 0.0, 0.0], [a1, 0.0, 0.0]], dtype=float)
    traj = PiecewisePolynomial(0.0, [quintic(start, end, duration)])

    def at(u: float) -> tuple[float, float, float]:
        st = np.stack([traj.evaluate(u, k) for k in range(3)])
        return float(st[0, 0]), float(st[1, 0]), float(st[2, 0])

    return at


def _join_times(t_start: float, t_end: float) -> list[float]:
    """从最早的半秒档开始试接入点，时刻表走完以后继续试终点保持。"""
    times: list[float] = []
    t = t_start + 0.5
    horizon = max(t_end, t_start) + 20.0
    while t <= horizon + 1e-9:
        times.append(t)
        t += 0.5
        if len(times) > 80:
            break
    if t_end > t_start + 0.5 and all(abs(item - t_end) > 1e-6 for item in times):
        times.append(t_end)
        times.sort()
    return times


def _fit_pva_interval(pva_at, t0: float, t1: float, limits: KinematicLimits, depth: int, forced0: np.ndarray | None):
    pva0 = pva_at(t0) if forced0 is None else forced0
    pva1 = pva_at(t1)
    seg = quintic(pva0, pva1, t1 - t0)
    traj = PiecewisePolynomial(t0, [seg])
    tm = 0.5 * (t0 + t1)
    err = float(np.linalg.norm(traj.evaluate(tm) - pva_at(tm)[0]))
    if err <= _FIT_TOL_M and _kin_ok(traj, limits):
        return [seg]
    if depth >= 8 or (t1 - t0) <= 0.08:
        return None
    left = _fit_pva_interval(pva_at, t0, tm, limits, depth + 1, forced0)
    if left is None:
        return None
    right = _fit_pva_interval(pva_at, tm, t1, limits, depth + 1, None)
    if right is None:
        return None
    return left + right


def _fit_profile(pva_at, t_start: float, t_end: float, head: np.ndarray, limits: KinematicLimits):
    if t_end <= t_start + 1e-9:
        return None
    knots = [t_start]
    t = t_start
    while t < t_end - 1e-9:
        t = min(t_end, t + 0.5)
        knots.append(t)
    segs: list[PolySegment] = []
    forced = {t_start: np.array(head, dtype=float, copy=True)}
    for a, b in zip(knots, knots[1:]):
        piece = _fit_pva_interval(pva_at, a, b, limits, 0, forced.get(a))
        if piece is None:
            return None
        if len(segs) + len(piece) > _MAX_SEGMENTS:
            return None
        segs.extend(piece)
    return PiecewisePolynomial(t_start, segs)


def _catchup_motion(path: _CenterPath, t_start: float, t_go: float, t_join: float, slot: np.ndarray, head: np.ndarray, limits: KinematicLimits):
    """沿本槽弧长从悬停接到 t_join 的时刻表，再贴着时刻表飞到终点。接不上就返回 None。"""
    if t_join <= t_start + 0.2:
        return None
    s1, sp1, sa1 = _arc_ref(path, t_join, t_go)
    blend = _scalar_quintic(0.0, 0.0, 0.0, s1, sp1, sa1, t_join - t_start)
    # 轮廓比计划限幅收一档：点质量滞后的加速度上限更紧，反馈还要余量。硬检查仍是 limits。
    v_cap = min(float(limits.v_max), 4.0)
    a_cap = min(float(limits.a_max), 3.0)
    t_motion_end = t_join if t_join >= t_go + path.T - 1e-9 else t_go + path.T

    def arc(t: float) -> tuple[float, float, float]:
        if t >= t_join - 1e-12:
            return _arc_ref(path, t, t_go)
        return blend(t - t_start)

    def pva_at(t: float) -> np.ndarray:
        s, sp, sa = arc(t)
        s = min(max(s, 0.0), path.length)
        return path.state_along(s, sp, sa, slot)

    step = 0.05
    t = t_start
    while t <= t_motion_end + 1e-9:
        s, sp, sa = arc(t)
        if s < -1e-4 or s > path.length + 1e-4 or sp < -1e-3:
            return None
        pva = path.state_along(min(max(s, 0.0), path.length), sp, sa, slot)
        if float(np.linalg.norm(pva[1])) > v_cap or float(np.linalg.norm(pva[2])) > a_cap:
            return None
        t += step
    try:
        return _fit_profile(pva_at, t_start, t_motion_end, head, limits)
    except (RuntimeError, ValueError, np.linalg.LinAlgError):
        return None


def iter_catchup(path: _CenterPath, t_start: float, t_go: float, slot: np.ndarray, head: np.ndarray, limits: KinematicLimits):
    """错过出发、人还停在本槽起点时，按接入时刻从早到晚产出至多三条追赶轨迹。"""
    origin = path.state_along(0.0, 0.0, 0.0, slot)
    head = np.asarray(head, dtype=float)
    if float(np.linalg.norm(origin[0] - head[0])) > 1e-4 or float(np.linalg.norm(head[1])) > 1e-4:
        return
    produced = 0
    t_end = t_go + path.T
    for t_join in _join_times(t_start, t_end):
        motion = _catchup_motion(path, t_start, t_go, t_join, slot, head, limits)
        if motion is None:
            continue
        produced += 1
        yield motion
        if produced >= 3:
            return


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

    def pva(self, t: float):
        """影子盾读批准位置用。不推进航向。"""
        rows = state_at(self.plan, float(t))
        return tuple(np.asarray(x, dtype=float) for x in rows)

    @property
    def traj(self):
        """影子盾按上游轨迹复验契约时读运动段，不改本计划。"""
        return self.plan.motion

    def sample(self, t: float):
        from yopo_swarm_core.yaw import calculate_yaw

        p, v, a = self.pva(t)
        self.yaw = float(calculate_yaw(v, self.goal - p, self.yaw, self.dt)[0])
        return p, v, a, self.yaw


class FormationFleet:
    # 票 10b 传输钩子。None：下面的 exchange 用调用方传入的进程内总线。
    # ROS 形态只在一次 run 期间把话题传输放在这里。06b 若重写 exchange，请保留第一行的 _transport_or_bus。
    transport_override = None

    def __init__(self, card: dict, physics_dt: float, *, record_inputs: bool):
        task = card["task"]
        if task["type"] != "formation":
            raise ValueError("FormationFleet 只接受 formation 任务")
        # frank 2026-11-09：开阔区进度门控关闭。开关写明 false，时刻表仍按墙钟开环走。
        gate, pct = read_progress_settings(task, default_gate=False)
        if gate:
            raise ValueError(
                "开阔区进度门控关闭（frank 2026-11-09）：FormationFleet 保持开环时刻表，progress_gate 必须为 false"
            )
        self.progress_gate = False
        self.progress_percentile = pct
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
        self._off_until: dict[str, float] = {}
        self._last_obs_t = 0.0
        self.deliveries = 0
        self.frozen = frozen_for_card(card)
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
            # 超时后重发 RELEASE/COMMIT。默认关掉的话，丢了的 RELEASE 会让对端预留一直占着本槽。
            resend_period=window,
            heartbeat_period=None,
            delay_check_s=_DELAY_CHECK_S,
            # 默认 3 次失败就停提案。丢包下要一直重新提案才能追上，不能中途 HALT。
            max_failed_rounds=10_000,
        )
        crit = card["criteria"]
        self.goal_tol = float(crit["goal_tolerance_m"])
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
        self.geom = GeomSession.from_card(card, members=self.members, physics_dt=self.dt)
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
        self.proposal_resend_period = _PROPOSAL_RESEND_PERIOD
        self.proposal_resend_guard = _PROPOSAL_RESEND_GUARD
        self._proposal_sent_at = {did: -math.inf for did in self.members}

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
        slot = self.slots[node.id]
        if t_start <= self.t_go + 1e-9:
            motion = build_motion(self.path, t_start, self.t_go, slot, head, self.pcfg.limits)
            return self._offer(node, motion, t_start, head, strict=True)
        # 同步出发那一轮已经过了。只提案本机追赶，对不上限幅或和邻机占用冲突就等下一窗口再要。
        for motion in iter_catchup(self.path, t_start, self.t_go, slot, head, self.pcfg.limits):
            offered = self._offer(node, motion, t_start, head, strict=False)
            if offered is not None:
                return offered
        return None

    def _offer(self, node: ProtocolNode, motion: PiecewisePolynomial, t_start: float, head: np.ndarray, *, strict: bool):
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
            if strict:
                raise RuntimeError(f"{node.id} 的编队计划没通过计划契约：{res}")
            return None
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

    def _transport_or_bus(self, bus):
        """进程内总线，或票 10b 设下的话题传输。不改协议，只换 publish/poll 的那一端。"""
        transport = type(self).transport_override
        if transport is None:
            return bus
        return transport.carrying(bus)

    def _resend_open_proposal(self, bus, node, now: float) -> None:
        """同一 reservation 再发一次，向还没把 ACK 送回来的对端再要一票。不新开一轮。"""
        inflight = node.in_flight
        if inflight is None:
            return
        # 票齐之后只剩 Delay Check 在等时间。再重发会多出 ACK，也会把副本送进下一窗。
        if node._quorum(inflight):
            return
        if now + self.proposal_resend_guard >= inflight.record.t_start:
            return
        last = self._proposal_sent_at[node.id]
        if now - last + 1e-12 < self.proposal_resend_period:
            return
        self._proposal_sent_at[node.id] = now
        msg = replace(inflight.proposal, ctx=node.next_ctx())
        node._log("resend_proposal", rid=msg.reservation_id, t=now)
        self._emit(bus, now, node.id, msg)

    def _pump(self, bus, now: float) -> None:
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

    def exchange(self, bus, now: float) -> None:
        bus = self._transport_or_bus(bus)
        if self.geom.enabled:
            self.geom.flush(bus, now)
        for did in self.members:
            node = self.nodes[did]
            for msg in node.on_tick(now):
                if getattr(msg, "kind", None) == "PROPOSE":
                    self._proposal_sent_at[did] = now
                self._emit(bus, now, did, msg)
            self._resend_open_proposal(bus, node, now)
        # 先把本拍到期的邻机轨迹和 ACK 看完，再决定 Delay Check 能不能提交。
        self._pump(bus, now)
        for did in self.members:
            for msg in self.nodes[did].finalize_delay_check(now):
                self._emit(bus, now, did, msg)
        self._pump(bus, now)

    def track(self, drone_id: str, now: float):
        rec = self.nodes[drone_id].executing(now)
        self._last_rec[drone_id] = rec
        if self.geom.enabled:
            alt = self.geom.override_sampler(drone_id)
            if alt is not None:
                return alt
        slot = self._follow[drone_id]
        changed = slot["hash"] != rec.plan_hash
        if changed:
            yaw = 0.0 if slot["sampler"] is None else slot["sampler"].yaw
            slot["sampler"] = _Track(rec.occupancy.plan, self.goals[drone_id], yaw, self.dt)
            slot["hash"] = rec.plan_hash
        return slot["sampler"]

    def shadow_view(self, drone_id: str):
        """本机刚提交的计划的只读视图。没换代则返回 None。不交给动力学。"""
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
        rec = self.last_record(drone_id)
        return "LOCAL_APPROVED" if rec.plan_seq > 0 else "LOCAL_HOVER"

    def observe(self, drone_id: str, t: float, position: np.ndarray) -> float:
        ref = self.path.pva_at(t, self.t_go, self.slots[drone_id])[0]
        err = float(np.linalg.norm(np.asarray(position, dtype=float) - ref))
        self.errors.append(err)
        self._last_obs_t = max(self._last_obs_t, float(t))
        if err > self.goal_tol:
            self._off_until[drone_id] = float(t)
        return err

    def summary(self, arrived: int, n_drones: int) -> dict:
        commits: dict[str, int] = {}
        aborts: dict[str, int] = {}
        switches: dict[str, float] = {}
        proposals = acks = nacks = 0
        hashes = {}
        for did, node in self.nodes.items():
            n_commit = 0
            n_abort = 0
            for event in node.events:
                kind = event["event"]
                if kind == "commit":
                    n_commit += 1
                elif kind == "abort":
                    n_abort += 1
                elif kind == "propose":
                    proposals += 1
                elif kind == "vote":
                    if event.get("decision") == "ACK":
                        acks += 1
                    else:
                        nacks += 1
            commits[did] = n_commit
            aborts[did] = n_abort
            switches[did] = float(node.committed.t_start)
            hashes[did] = node.committed.plan_hash
        catchup = {}
        for did, node in self.nodes.items():
            t_switch = switches[did]
            late = node.committed.plan_seq > 0 and t_switch > self.t_go + 1e-9
            if node.committed.plan_seq == 0:
                # 超时后一直悬停，没有接回槽位。记到最后一次观测。
                catchup[did] = max(0.0, float(self._last_obs_t) - self.t_go)
            elif late:
                t_back = t_switch
                t_off = self._off_until.get(did)
                if t_off is not None and t_off > t_back:
                    t_back = t_off
                catchup[did] = max(0.0, float(t_back) - self.t_go)
            else:
                catchup[did] = 0.0
        arr = np.asarray(self.errors, dtype=float)
        return {
            "error_p95_m": None if arr.size == 0 else float(np.percentile(arr, 95)),
            "error_max_m": None if arr.size == 0 else float(arr.max()),
            "arrival_rate": float(arrived) / float(n_drones),
            # 追赶时间：同步出发的飞机为 0。错过出发的，是从 t_go 到「晚提交的计划已经在执行，
            # 且队形误差回到 goal_tolerance 以内」的时间。到结束还停在初始悬停上，则记到最后一次观测。
            # 编队值取各机最大。
            "catchup_time_s": max(catchup.values()) if catchup else 0.0,
            "catchup_time_by_drone_s": catchup,
            "local_aborts": aborts,
            "committed_t_switch_s": switches,
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
            "progress_gate": self.progress_gate,
            "progress_percentile": self.progress_percentile,
            "virtual_center": "scheduled",
        }
