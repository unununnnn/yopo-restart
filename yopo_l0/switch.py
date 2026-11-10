"""票 08：一种变阵。横排（line-abreast）↔ 纵列（column），窄门自动触发。

虚拟中心沿开阔区直线走。每机只在自己的轮转窗口里提案，邻机按几何占用回 ACK / NACK，
本机收齐 N-1 个 ACK 并在本地 COMMIT 之后，动力学才跟踪这份新计划。槽位更换是
从父计划在 t_start 的 PVA 接出去的多项式，不是瞬移。没有中央裁决者。

触发：各机用同一份场景几何，量虚拟中心前方走廊里包含航线的净空宽度。
宽度低于阈值就协商切到纵列；驶出窄门、宽度回来再协商切回横排。
几何只决定「要哪一套槽位」，不改已经 COMMIT 的曲线，也不读他机里程计。
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

from .scene import parse_geometry

_LIMITS = KinematicLimits(v_max=8.0, a_max=12.0, j_max=400.0)
_GOAL_MATCH_M = 0.25


class LocalApproval:
    """本机批准码。只记本机节点的状态，不代表编队级裁决。"""

    def __init__(self) -> None:
        self.last_code = "LOCAL_HOVER"
        self.extra_codes: dict = {}


def _smooth01(u: float) -> tuple[float, float, float]:
    """u∈[0,1] 上 0→1 的最小急动度曲线，以及它对 u 的一二阶导数。两端导数为 0。"""
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
        raise ValueError(f"槽位必须是长度 3，收到 {raw}")
    return slot


def _kin_ok(traj: PiecewisePolynomial, limits: KinematicLimits) -> bool:
    return all(
        check_derivative_bound(traj, order, lim).verdict is Verdict.SAFE
        for order, lim in limits.by_order()
    )


def _uniq(times: list[float]) -> list[float]:
    out: list[float] = []
    for t in sorted(times):
        if not out or t > out[-1] + 1e-8:
            out.append(float(t))
    return out


def _densify(knots: list[float], step: float) -> list[float]:
    out = [knots[0]]
    for b in knots[1:]:
        span = b - out[-1]
        n = max(1, int(math.ceil(span / step - 1e-9)))
        a = out[-1]
        for i in range(1, n + 1):
            out.append(a + span * i / n)
    return out


def _merge(intervals: list[tuple[float, float]]) -> list[list[float]]:
    merged: list[list[float]] = []
    for a, b in sorted(intervals):
        if a > b:
            a, b = b, a
        if not merged or a > merged[-1][1] + 1e-9:
            merged.append([a, b])
        else:
            merged[-1][1] = max(merged[-1][1], b)
    return merged


def corridor_width(geometry: dict, x: float, z: float, *, open_width: float, y0: float = 0.0) -> float:
    """x 处、高度 z 上，包含 y0 的横向净空。没有遮挡时返回 open_width（开阔区）。

    只看场景几何里的盒子和竖直圆柱。这是变阵触发用的宽度，不是避障轨迹。
    """
    if open_width <= 0.0:
        raise ValueError("open_width_m 必须为正")
    blocks: list[tuple[float, float]] = []
    for b in geometry.get("boxes") or []:
        lo, hi = b["lo"], b["hi"]
        if lo[0] - 1e-9 <= x <= hi[0] + 1e-9 and lo[2] <= z <= hi[2]:
            blocks.append((float(lo[1]), float(hi[1])))
    for c in geometry.get("cylinders") or []:
        cx, cy = c["center_xy"]
        r = float(c["radius"])
        dx = float(x) - float(cx)
        if abs(dx) > r + 1e-9 or not (float(c["z_lo"]) <= z <= float(c["z_hi"])):
            continue
        half = math.sqrt(max(r * r - dx * dx, 0.0))
        blocks.append((float(cy) - half, float(cy) + half))
    if not blocks:
        return float(open_width)
    gaps: list[tuple[float, float]] = []
    prev: float | None = None
    for a, b in _merge(blocks):
        gaps.append((float("-inf") if prev is None else prev, a))
        prev = b
    gaps.append((float("-inf") if prev is None else prev, float("inf")))
    for lo, hi in gaps:
        if lo - 1e-9 <= y0 <= hi + 1e-9:
            if math.isinf(lo) or math.isinf(hi):
                return float(open_width)
            return float(hi - lo)
    return 0.0


def width_ahead(
    geometry: dict,
    x_back: float,
    x_front: float,
    z: float,
    *,
    open_width: float,
    y0: float = 0.0,
) -> float:
    """[x_back, x_front] 上的最小净空。采样步长 0.25 m，并探进每只障碍的内部。"""
    lo, hi = (x_back, x_front) if x_back <= x_front else (x_front, x_back)
    xs = [lo, hi]
    x = lo
    while x < hi - 1e-9:
        xs.append(x)
        x += 0.25
    for b in geometry.get("boxes") or []:
        blo, bhi = float(b["lo"][0]), float(b["hi"][0])
        if bhi < lo - 1e-9 or blo > hi + 1e-9:
            continue
        for edge in (blo, bhi, 0.5 * (blo + bhi)):
            if lo - 1e-6 <= edge <= hi + 1e-6:
                xs.append(edge)
    for c in geometry.get("cylinders") or []:
        cx = float(c["center_xy"][0])
        r = float(c["radius"])
        for edge in (cx - r, cx, cx + r):
            if lo - 1e-6 <= edge <= hi + 1e-6:
                xs.append(edge)
    return min(corridor_width(geometry, x, z, open_width=open_width, y0=y0) for x in xs)


class _Rail:
    """水平直线虚拟中心：静止 → 平滑加速到巡航 → 巡航 → 平滑刹停。航向固定 +x。"""

    def __init__(self, spec: dict):
        origin = np.asarray(spec["origin_enu"], dtype=float)
        if origin.shape != (3,):
            raise ValueError("virtual_center.origin_enu 必须是长度 3")
        self.origin = origin
        self.length = float(spec["length_m"])
        self.vc = float(spec["speed_mps"])
        self.Ta = float(spec.get("accel_s", 2.0))
        self.Tb = float(spec.get("brake_s", 2.0))
        if min(self.length, self.vc, self.Ta, self.Tb) <= 0.0:
            raise ValueError("length_m、speed_mps、accel_s、brake_s 必须为正")
        # 速度用 smoothstep：两端加速度为 0，加速段路程 = vc·Ta/2。
        self.s_acc = self.vc * self.Ta * 0.5
        self.s_brake = self.vc * self.Tb * 0.5
        if self.s_acc + self.s_brake >= self.length - 1e-6:
            raise ValueError("加速段加减速段已经比全程还长")
        self.s_cruise = self.length - self.s_acc - self.s_brake
        self.t_cruise = self.s_cruise / self.vc
        self.t_brake = self.Ta + self.t_cruise
        self.T = self.t_brake + self.Tb
        self._check()

    def _check(self) -> None:
        step = 1e-3
        for tau in (0.4 * self.Ta, self.Ta, self.Ta + 0.4 * self.t_cruise, self.t_brake, self.T - 0.3 * self.Tb):
            if not (step < tau < self.T - step):
                continue
            p0 = self.center_pva(tau - step)[0]
            p1 = self.center_pva(tau + step)[0]
            v = self.center_pva(tau)[1]
            if float(np.linalg.norm((p1 - p0) / (2 * step) - v)) > 1e-2:
                raise RuntimeError(f"虚拟中心速度和位置差分不一致，tau={tau}")

    def center_pva(self, tau: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if tau <= 0.0:
            return self.origin.copy(), np.zeros(3), np.zeros(3)
        if tau >= self.T:
            return self.origin + np.array([self.length, 0.0, 0.0]), np.zeros(3), np.zeros(3)
        if tau <= self.Ta:
            s = self.vc * (tau**3 / self.Ta**2 - 0.5 * tau**4 / self.Ta**3)
            v = self.vc * (3.0 * tau**2 / self.Ta**2 - 2.0 * tau**3 / self.Ta**3)
            a = self.vc * (6.0 * tau / self.Ta**2 - 6.0 * tau**2 / self.Ta**3)
        elif tau <= self.t_brake:
            s = self.s_acc + self.vc * (tau - self.Ta)
            v = self.vc
            a = 0.0
        else:
            tb = tau - self.t_brake
            s = self.s_acc + self.s_cruise + self.vc * (tb - tb**3 / self.Tb**2 + 0.5 * tb**4 / self.Tb**3)
            v = self.vc * (1.0 - 3.0 * tb**2 / self.Tb**2 + 2.0 * tb**3 / self.Tb**3)
            a = self.vc * (-6.0 * tb / self.Tb**2 + 6.0 * tb**2 / self.Tb**3)
        return (
            self.origin + np.array([s, 0.0, 0.0]),
            np.array([v, 0.0, 0.0]),
            np.array([a, 0.0, 0.0]),
        )

    def body_pva(
        self,
        t: float,
        t_go: float,
        slot_from: np.ndarray,
        slot_to: np.ndarray,
        t_blend: float | None,
        blend_s: float,
    ) -> np.ndarray:
        cp, cv, ca = self.center_pva(float(t) - float(t_go))
        if t_blend is None or blend_s <= 0.0 or float(np.linalg.norm(slot_to - slot_from)) <= 1e-12:
            slot, ds, d2 = slot_from, np.zeros(3), np.zeros(3)
        elif t <= t_blend:
            slot, ds, d2 = slot_from, np.zeros(3), np.zeros(3)
        elif t >= t_blend + blend_s:
            slot, ds, d2 = slot_to, np.zeros(3), np.zeros(3)
        else:
            sig, dsig, d2sig = _smooth01((t - t_blend) / blend_s)
            delta = slot_to - slot_from
            slot = slot_from + sig * delta
            ds = (dsig / blend_s) * delta
            d2 = (d2sig / (blend_s**2)) * delta
        return np.stack([cp + slot, cv + ds, ca + d2])


def build_motion(
    rail: _Rail,
    t_start: float,
    t_go: float,
    head: np.ndarray,
    slot_from: np.ndarray,
    slot_to: np.ndarray,
    t_blend: float | None,
    blend_s: float,
    limits: KinematicLimits,
) -> PiecewisePolynomial | None:
    """从父计划在 t_start 的 PVA 接到槽位时刻表。t_start < t_go 时前面悬停。

    变阵从 t_blend 起用最小急动度把槽位从 slot_from 收到 slot_to，两端速度加速度对齐巡航，
    所以接头没有跳变。拟合失败返回 None，调用方记拒绝码，不飞一条没通过契约的曲线。
    """
    head = np.asarray(head, dtype=float)
    t_end = t_go + rail.T
    if t_end <= t_start + 1e-4:
        return None
    segs: list[PolySegment] = []
    if t_go > t_start + 1e-8:
        coeff = np.zeros((3, 1))
        coeff[:, 0] = head[0]
        segs.append(PolySegment(coeff, t_go - t_start))
        cursor = t_go
    else:
        cursor = t_start
    bounds = [cursor, t_end, t_go + rail.Ta, t_go + rail.t_brake]
    if t_blend is not None and blend_s > 0.0:
        bounds.extend((t_blend, t_blend + blend_s))
    knots = _uniq([t for t in bounds if cursor - 1e-9 <= t <= t_end + 1e-9])
    if knots[0] > cursor + 1e-8:
        knots.insert(0, cursor)
    if knots[-1] < t_end - 1e-8:
        knots.append(t_end)
    dense = _densify(knots, 1.0)

    def pva(t: float) -> np.ndarray:
        return rail.body_pva(t, t_go, slot_from, slot_to, t_blend, blend_s)

    for a, b in zip(dense, dense[1:]):
        if b <= a + 1e-8:
            continue
        if abs(a - t_start) <= 1e-8 and t_go <= t_start + 1e-8:
            p0 = np.array(head, dtype=float, copy=True)
        elif abs(a - t_go) <= 1e-8 and t_go > t_start + 1e-8:
            p0 = np.array([np.asarray(head[0], dtype=float), np.zeros(3), np.zeros(3)])
        else:
            p0 = pva(a)
        p1 = pva(b)
        seg = quintic(p0, p1, b - a)
        piece = PiecewisePolynomial(a, [seg])
        if not _kin_ok(piece, limits):
            return None
        segs.append(seg)
    if not segs:
        return None
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


class SwitchFleet:
    """横排出发，窄门前协商切纵列，出门后再协商切回横排。每机只飞本机 COMMIT 的计划。"""

    policy_name = "switch"
    result_key = "switch"
    uses_network = False

    def __init__(self, card: dict, physics_dt: float, *, record_inputs: bool):
        task = card["task"]
        if task["type"] != "switch":
            raise ValueError("SwitchFleet 只接受 switch 任务")
        drones = card["drones"]
        if len(drones) < 2:
            raise ValueError("变阵至少两架飞机")
        self.members = tuple(str(d["id"]) for d in drones)
        if len(set(self.members)) != len(self.members):
            raise ValueError("无人机 id 重复")
        self.line = {str(d["id"]): _unit_slot(d["slot_body"]) for d in drones}
        raw_col = task.get("column_slots")
        if not isinstance(raw_col, dict):
            raise ValueError("task.column_slots 必须给出每机的纵列槽位")
        missing = [d for d in self.members if d not in raw_col]
        if missing:
            raise ValueError(f"column_slots 缺 {missing}")
        self.column = {d: _unit_slot(raw_col[d]) for d in self.members}
        if not any(float(np.linalg.norm(self.column[d] - self.line[d])) > 1e-6 for d in self.members):
            raise ValueError("纵列槽位和横排槽位完全相同，没有可切换的一对")
        geom = card.get("geometry")
        if not isinstance(geom, dict):
            raise ValueError("变阵触发要读场景几何，请经 scene.load_card 读入（geometry 已内联）")
        self.geometry = geom
        self.rail = _Rail(task["virtual_center"])
        self.dt = float(physics_dt)
        self.record_inputs = record_inputs
        self.input_digests: list[str] = []
        self.errors: list[float] = []
        self.err_before: list[float] = []
        self.err_after: list[float] = []
        self.deliveries = 0
        self.frozen = bool(card["criteria"].get("frozen", False))
        self.radius = float(card["criteria"]["body_radius_m"])
        self.obstacles = parse_geometry(geom, card.get("ground_z"))
        window = float(task.get("window_len_s", 0.5))
        if window <= 0.0:
            raise ValueError("window_len_s 必须为正")
        self.lookahead = float(task.get("lookahead_m", 7.0))
        self.threshold = float(task.get("width_threshold_m", 5.0))
        self.open_width = float(task.get("open_width_m", 20.0))
        self.transition = float(task.get("transition_s", 4.0))
        if min(self.lookahead, self.threshold, self.open_width, self.transition) <= 0.0:
            raise ValueError("lookahead_m、width_threshold_m、open_width_m、transition_s 必须为正")
        # 末机的 t_start 等于全体窗口转完一圈。此前提案的飞机在计划里悬停到这一刻。
        self.t_go = window * len(self.members)
        self.z = float(self.rail.origin[2])
        spec = task["virtual_center"]
        self.cfg = ProtocolConfig(
            members=self.members,
            window_len=window,
            task_hash=digest(
                {
                    "origin": spec["origin_enu"],
                    "length_m": spec["length_m"],
                    "speed_mps": spec["speed_mps"],
                    "accel_s": spec.get("accel_s", 2.0),
                    "brake_s": spec.get("brake_s", 2.0),
                    "line": {k: v.tolist() for k, v in self.line.items()},
                    "column": {k: v.tolist() for k, v in self.column.items()},
                    "window_len_s": window,
                    "lookahead_m": self.lookahead,
                    "width_threshold_m": self.threshold,
                    "open_width_m": self.open_width,
                    "transition_s": self.transition,
                }
            ),
            # 冲突或丢 ACK 是变阵失败的常态，不能三次就停飞。拒绝码留在结果里。
            # delay_check_s 保持默认 0：变阵仍是收齐 ACK 当拍提交。
            max_failed_rounds=10000,
            resend_period=None,
            heartbeat_period=None,
        )
        self.pcfg = PlanConfig(
            _LIMITS,
            max_branch_points=2,
            body_radius=self.radius,
            tracking_margin=0.15,
        )
        t_final = self.t_go + self.rail.T
        self.goals_line = {d: self.rail.body_pva(t_final, self.t_go, self.line[d], self.line[d], None, 0.0)[0].copy() for d in self.members}
        self.goals_column = {
            d: self.rail.body_pva(t_final, self.t_go, self.column[d], self.column[d], None, 0.0)[0].copy() for d in self.members
        }
        # 到达点是变阵对走完之后的横排，不是纵列中途的槽位。
        self.goals = {d: self.goals_line[d].copy() for d in self.members}
        self.starts = {
            d: self.rail.body_pva(0.0, self.t_go, self.line[d], self.line[d], None, 0.0)[0].copy() for d in self.members
        }
        checker = PlanConflictChecker()
        self.nodes = {
            d: ProtocolNode(d, self.cfg, checker=checker, planner=self._planner, initial=self._initial)
            for d in self.members
        }
        self._follow = {d: {"hash": None, "sampler": None} for d in self.members}
        self._last_rec = {}
        self._codes: dict[str, dict[str, int]] = {d: {} for d in self.members}
        self._last_code = {d: "LOCAL_HOVER" for d in self.members}
        self._seen = {d: 0 for d in self.members}
        self._pending: dict[str, dict | None] = {d: None for d in self.members}
        self._phase = {d: "hover" for d in self.members}
        self._blend: dict[str, tuple | None] = {d: None for d in self.members}
        self._win_column: dict | None = None
        self._win_line: dict | None = None
        self._blends: list[dict] = []
        self._pos: dict[str, np.ndarray] = {}
        self._pos_t: dict[str, float] = {}
        self._last_width: float | None = None

    @classmethod
    def from_card(cls, card: dict, physics_dt: float, *, record_inputs: bool = False) -> "SwitchFleet":
        return cls(card, physics_dt, record_inputs=record_inputs)

    def _initial(self, owner: str, cfg: ProtocolConfig) -> PlanOccupancy:
        return hover_occupancy(owner, self.starts[owner], self.pcfg)

    def check_start(self, drone_id: str, start: np.ndarray) -> None:
        expect = self.starts[drone_id]
        if float(np.linalg.norm(np.asarray(start, dtype=float) - expect)) > 1e-6:
            raise ValueError(
                f"{drone_id} 的 start_enu {np.asarray(start, float).tolist()} 不是横排槽位在虚拟中心起点的位置 {expect.tolist()}"
            )

    def goal_of(self, drone_id: str) -> np.ndarray:
        return self.goals[drone_id].copy()

    def width_between(self, x_back: float, x_front: float) -> float:
        return width_ahead(self.geometry, x_back, x_front, self.z, open_width=self.open_width)

    def desired_at(self, t: float) -> str:
        """走廊前方净空低于阈值 → 纵列，否则横排。各机用同一几何，同一时刻得到同一个答案。"""
        cx = float(self.rail.center_pva(t - self.t_go)[0][0])
        rear = min(float(s[0]) for s in self.column.values())
        width = self.width_between(cx + rear, cx + self.lookahead)
        self._last_width = width
        return "column" if width < self.threshold else "line"

    def _count(self, drone_id: str, code: str) -> None:
        slot = self._codes[drone_id]
        slot[code] = slot.get(code, 0) + 1
        self._last_code[drone_id] = code

    def _phase_of(self, drone_id: str, terminal: np.ndarray) -> str:
        terminal = np.asarray(terminal, dtype=float)
        line = self.goals_line[drone_id]
        col = self.goals_column[drone_id]
        if float(np.linalg.norm(col - line)) <= 1e-4:
            # 领机两个队形的终点是同一个点。它不参与槽位重分配。
            return "line" if float(np.linalg.norm(terminal - line)) < _GOAL_MATCH_M else "hover"
        if float(np.linalg.norm(terminal - col)) < _GOAL_MATCH_M:
            return "column"
        if float(np.linalg.norm(terminal - line)) < _GOAL_MATCH_M:
            return "line"
        return "hover"

    def _order(self, desired: str) -> list[str]:
        """要改槽位的飞机。去纵列时后机先动，回横排时靠近领机的先让出中线。"""
        changed = [d for d in self.members if float(np.linalg.norm(self.column[d] - self.line[d])) > 1e-6]
        if desired == "column":
            changed.sort(key=lambda d: (float(self.column[d][0]), self.members.index(d)))
        else:
            changed.sort(key=lambda d: (-float(self.column[d][0]), self.members.index(d)))
        return changed

    def _ready(self, node: ProtocolNode, desired: str, who: str) -> bool:
        order = self._order(desired)
        if who not in order:
            return False
        for pred in order[: order.index(who)]:
            term = node.peers[pred].plans[-1].occupancy.plan.terminal_point()
            if self._phase_of(pred, term) != desired:
                return False
        return True

    def _as_plan(self, node: ProtocolNode, t_start: float, head: np.ndarray, motion: PiecewisePolynomial):
        res = make_plan(
            node.id,
            node.plan_seq + 1,
            node.committed.plan_hash,
            motion,
            t_start,
            head,
            [motion.t_end],
            self.pcfg,
        )
        if not isinstance(res, Plan):
            return None, "SWITCH:CONTRACT"
        occ = PlanOccupancy(res, self.pcfg)
        if node.conflicts_with_neighbors(occ):
            return None, "SWITCH:CONFLICT"
        return occ, None

    def _digest(self, node: ProtocolNode, t_start: float, head: np.ndarray, desired: str) -> str:
        h = hashlib.sha256()
        h.update(node.id.encode())
        h.update(np.float64(t_start).tobytes())
        h.update(np.asarray(head, dtype=np.float64).tobytes())
        h.update(desired.encode())
        h.update(np.float64(0.0 if self._last_width is None else self._last_width).tobytes())
        for peer_id, peer in sorted(node.peers.items()):
            h.update(peer_id.encode())
            for rec in peer.plans:
                h.update(rec.plan_hash.encode())
        return h.hexdigest()

    def _planner(self, node: ProtocolNode, now: float, t_start: float):
        parent = node.committed.occupancy.plan
        head = np.array(state_at(parent, t_start), dtype=float, copy=True)
        phase = self._phase_of(node.id, parent.terminal_point())
        desired = self.desired_at(t_start)
        if self.record_inputs:
            self.input_digests.append(self._digest(node, t_start, head, desired))
        if phase == "hover":
            return self._propose_line(node, t_start, head)
        same_slot = float(np.linalg.norm(self.column[node.id] - self.line[node.id])) <= 1e-6
        if phase == desired or same_slot:
            return None
        if not self._ready(node, desired, node.id):
            return None
        slot_from = self.line[node.id] if phase == "line" else self.column[node.id]
        slot_to = self.column[node.id] if desired == "column" else self.line[node.id]
        direction = "line-to-column" if desired == "column" else "column-to-line"
        motion = build_motion(
            self.rail, t_start, self.t_go, head, slot_from, slot_to, t_start, self.transition, self.pcfg.limits
        )
        if motion is None:
            self._count(node.id, "SWITCH:KIN")
            return None
        occ, code = self._as_plan(node, t_start, head, motion)
        if occ is None:
            # 预检就和邻机已提交的占用重叠：不把这条提案送上总线。拒绝码解释为什么没切成。
            self._count(node.id, code or "SWITCH:CONTRACT")
            return None
        self._pending[node.id] = {
            "kind": "switch",
            "direction": direction,
            "t_start": float(t_start),
            "slot_from": np.array(slot_from, dtype=float, copy=True),
            "slot_to": np.array(slot_to, dtype=float, copy=True),
            "width_m": self._last_width,
        }
        return occ

    def _propose_line(self, node: ProtocolNode, t_start: float, head: np.ndarray):
        if t_start > self.t_go + 1e-6:
            self._count(node.id, "LINE:MISSED")
            return None
        slot = self.line[node.id]
        motion = build_motion(self.rail, t_start, self.t_go, head, slot, slot, None, 0.0, self.pcfg.limits)
        if motion is None:
            self._count(node.id, "LINE:KIN")
            return None
        occ, code = self._as_plan(node, t_start, head, motion)
        if occ is None:
            self._count(node.id, "LINE:CONFLICT" if code == "SWITCH:CONFLICT" else (code or "LINE:KIN"))
            return None
        self._pending[node.id] = {"kind": "line", "t_start": float(t_start)}
        return occ

    def _consume(self, drone_id: str) -> list[dict]:
        node = self.nodes[drone_id]
        evs = node.events[self._seen[drone_id] :]
        self._seen[drone_id] = len(node.events)
        return evs

    def _harvest(self) -> None:
        fresh = {d: self._consume(d) for d in self.members}
        conflict = False
        for evs in fresh.values():
            for e in evs:
                if e.get("event") == "vote" and e.get("reason") == "CONFLICT":
                    conflict = True
        for did, evs in fresh.items():
            pend = self._pending.get(did)
            for e in evs:
                if pend is None:
                    break
                if e.get("event") == "commit":
                    self._note_commit(did, pend)
                    self._pending[did] = None
                    pend = None
                elif e.get("event") == "abort":
                    self._note_abort(did, pend, str(e.get("reason")), conflict)
                    self._pending[did] = None
                    pend = None

    def _note_abort(self, drone_id: str, pend: dict, reason: str, conflict: bool) -> None:
        if pend.get("kind") != "switch":
            if pend.get("kind") == "line":
                self._count(drone_id, "LINE:NO_ACK")
            return
        # 没在截止前收齐 ACK，或对方明确拒绝。冲突优先于「没 ACK」，两条都是可解释的拒绝码。
        if reason == "NACKED" and conflict:
            self._count(drone_id, "SWITCH:CONFLICT")
        else:
            self._count(drone_id, "SWITCH:NO_ACK")

    def _note_commit(self, drone_id: str, pend: dict) -> None:
        if pend.get("kind") == "line":
            self._count(drone_id, "LINE:COMMIT")
            self._phase[drone_id] = "line"
            self._blend[drone_id] = None
            return
        direction = str(pend["direction"])
        t0 = float(pend["t_start"])
        t1 = t0 + self.transition
        slot_from = np.asarray(pend["slot_from"], dtype=float)
        slot_to = np.asarray(pend["slot_to"], dtype=float)
        self._blend[drone_id] = (t0, t1, slot_from, slot_to)
        self._phase[drone_id] = "column" if direction == "line-to-column" else "line"
        self._count(drone_id, "SWITCH:TO_COLUMN" if direction == "line-to-column" else "SWITCH:TO_LINE")
        self._blends.append(
            {
                "drone": drone_id,
                "direction": direction,
                "t_start": t0,
                "t_end": t1,
                "width_m": pend.get("width_m"),
            }
        )
        attr = "_win_column" if direction == "line-to-column" else "_win_line"
        w = getattr(self, attr)
        if w is None:
            w = {
                "direction": direction,
                "t_start": t0,
                "t_end": t1,
                "min_inter_drone_m": None,
                "collisions": 0,
                "commits": [drone_id],
            }
            setattr(self, attr, w)
        else:
            w["t_start"] = min(float(w["t_start"]), t0)
            w["t_end"] = max(float(w["t_end"]), t1)
            w["commits"].append(drone_id)

    def _slot_at(self, drone_id: str, t: float) -> np.ndarray:
        blend = self._blend.get(drone_id)
        if blend is not None:
            t0, t1, s0, s1 = blend
            if t < t0:
                return s0
            if t <= t1 and t1 > t0:
                sig = _smooth01(min(max((t - t0) / (t1 - t0), 0.0), 1.0))[0]
                return s0 + sig * (s1 - s0)
            return s1
        if self._phase.get(drone_id) == "column":
            return self.column[drone_id]
        return self.line[drone_id]

    def _ref_p(self, drone_id: str, t: float, slot: np.ndarray) -> np.ndarray:
        return self.rail.body_pva(t, self.t_go, slot, slot, None, 0.0)[0]

    def _windows(self) -> list[dict]:
        return [w for w in (self._win_column, self._win_line) if w is not None]

    def _note_spacing(self, t: float) -> None:
        pts = [self._pos[d] for d in self.members]
        mind = None
        for i, a in enumerate(pts):
            for b in pts[i + 1 :]:
                dist = float(np.linalg.norm(a - b))
                mind = dist if mind is None else min(mind, dist)
        hit = mind is not None and mind <= 2.0 * self.radius + 1e-9
        if not hit:
            for p in pts:
                if min(o.distance(p) for o in self.obstacles) <= self.radius + 1e-9:
                    hit = True
                    break
        for w in self._windows():
            if w["t_start"] - 1e-9 <= t <= w["t_end"] + 1e-9:
                if mind is not None:
                    prev = w["min_inter_drone_m"]
                    w["min_inter_drone_m"] = mind if prev is None else min(float(prev), mind)
                if hit:
                    w["collisions"] = 1

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
                break
            for env in envs:
                if env.payload.get("kind") != "protocol":
                    continue
                self.deliveries += 1
                for reply in self.nodes[env.dest].on_message(env.payload["msg"], now):
                    self._emit(bus, now, env.dest, reply)
        else:
            raise RuntimeError("协议消息在一个 tick 内没有投递完")
        self._harvest()

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
        position = np.asarray(position, dtype=float)
        slot = self._slot_at(drone_id, t)
        err = float(np.linalg.norm(position - self._ref_p(drone_id, t, slot)))
        self.errors.append(err)
        line_err = float(np.linalg.norm(position - self._ref_p(drone_id, t, self.line[drone_id])))
        if self._win_column is None or t < float(self._win_column["t_start"]):
            if t >= self.t_go:
                self.err_before.append(line_err)
        elif self._win_line is not None and t >= float(self._win_line["t_end"]):
            self.err_after.append(line_err)
        self._pos[drone_id] = position
        self._pos_t[drone_id] = float(t)
        if all(self._pos_t.get(d) == float(t) for d in self.members):
            self._note_spacing(float(t))
        return err

    def summary(self, arrived: int, n_drones: int) -> dict:
        self._harvest()
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
        windows = []
        for w in self._windows():
            windows.append(
                {
                    "direction": w["direction"],
                    "t_start": float(w["t_start"]),
                    "t_end": float(w["t_end"]),
                    "duration_s": float(w["t_end"] - w["t_start"]),
                    "min_inter_drone_m": w["min_inter_drone_m"],
                    "collisions": int(w["collisions"]),
                    "commits": list(w["commits"]),
                }
            )
        mins = [w["min_inter_drone_m"] for w in windows if w["min_inter_drone_m"] is not None]
        arr = np.asarray(self.errors, dtype=float)
        return {
            "error_p95_m": None if arr.size == 0 else float(np.percentile(arr, 95)),
            "error_max_m": None if arr.size == 0 else float(arr.max()),
            "error_p95_before_m": _p95(self.err_before),
            "error_p95_after_m": _p95(self.err_after),
            "arrival_rate": float(arrived) / float(n_drones),
            "frozen": self.frozen,
            "adjudicator": None,
            "pair": "line-abreast<->column",
            "trigger": "corridor-width-ahead",
            "negotiated": "bus-reservation",
            "width_threshold_m": self.threshold,
            "lookahead_m": self.lookahead,
            "open_width_m": self.open_width,
            "transition_s": self.transition,
            "body_radius_m": self.radius,
            "local_commits": commits,
            "proposals": proposals,
            "acks": acks,
            "nacks": nacks,
            "decisions": dict(sorted(decisions.items())),
            "transitions": windows,
            "transition_duration_s": [w["duration_s"] for w in windows],
            "transition_collisions": int(sum(int(w["collisions"]) for w in windows)),
            "min_inter_drone_transition_m": None if not mins else float(min(mins)),
            "blends": list(self._blends),
            "window_len_s": self.cfg.window_len,
            "t_go_s": self.t_go,
            "motion_duration_s": self.rail.T,
            "final_slots_enu": {d: self.goals[d].tolist() for d in self.members},
            "column_slots_enu": {d: self.goals_column[d].tolist() for d in self.members},
            "committed_plan_hash": hashes,
            "virtual_center": "straight-rail",
        }


def _p95(xs: list[float]) -> float | None:
    if not xs:
        return None
    return float(np.percentile(np.asarray(xs, dtype=float), 95))


def format_report(rows: list[dict]) -> str:
    """票 08 结果表。过渡期碰撞是硬失败；判据本身未冻结，只报数。"""
    rows = [r for r in rows if r.get("task") == "switch"]
    if not rows:
        return ""
    lines = [
        "### 票 08 横排↔纵列变阵（判据未冻结 / report-only）",
        "",
        "触发是走廊前方净空低于阈值。过渡期碰撞（障碍或机间）是硬失败；没切成时拒绝码写在失败原因里。",
        "",
        "| seed | 到达率 | 过渡碰撞 | 过渡最小机间距离 m | 过渡时长 s | 变阵前队形误差 p95 m | 变阵后队形误差 p95 m | 影子拒绝率 | 失败原因 |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    col_sum = 0
    for r in rows:
        block = r.get("switch") or {}
        n = int(r.get("n_drones") or 0)
        got = int(r.get("arrived") or 0)
        rate = r.get("arrival_rate")
        if rate is None:
            rate = block.get("arrival_rate")
        rate_cell = "—" if rate is None else f"{float(rate):.2f} ({got}/{n})"
        collisions = int(block.get("transition_collisions") or 0)
        col_sum += collisions
        durations = block.get("transition_duration_s") or []
        dur = "—" if not durations else "+".join(f"{float(x):.2f}" for x in durations)
        why = _failure_text(r, block, collisions)
        lines.append(
            "| {seed} | {rate} | {col} | {gap} | {dur} | {before} | {after} | {shadow} | {why} |".format(
                seed=r.get("seed"),
                rate=rate_cell,
                col=collisions,
                gap=_fmt(block.get("min_inter_drone_transition_m")),
                dur=dur,
                before=_fmt(block.get("error_p95_before_m")),
                after=_fmt(block.get("error_p95_after_m")),
                shadow=_fmt(r.get("shadow_reject_rate")),
                why=why,
            )
        )
    frozen = all(bool((r.get("switch") or {}).get("frozen", False)) for r in rows)
    lines += [
        "",
        f"合计 {len(rows)} 个 seed：过渡碰撞 {col_sum}。"
        f"{'判据已冻结' if frozen else '未冻结 / report-only'}。",
    ]
    failed = [r for r in rows if r.get("outcome") != "SUCCESS" or int((r.get("switch") or {}).get("transition_collisions") or 0) > 0]
    if failed:
        lines.append("")
        lines.append("未到达、碰撞或变阵被拒的 seed：")
        for r in failed:
            block = r.get("switch") or {}
            codes = _switch_codes(block)
            shown = ", ".join(f"{k}:{v}" for k, v in codes.items()) or "—"
            lines.append(
                f"- seed {r.get('seed')}: {r.get('outcome')} {r.get('reason')}；"
                f"终点误差 {_fmt(r.get('final_goal_error_m'))} m；拒绝码 {shown}"
            )
    return "\n".join(lines)


def _switch_codes(block: dict) -> dict:
    decisions = block.get("decisions") or {}
    return {k: v for k, v in decisions.items() if str(k).startswith("SWITCH:") and "TO_" not in str(k) and str(k) != "SWITCH:COMMIT"}


def _failure_text(r: dict, block: dict, collisions: int) -> str:
    parts = []
    if r.get("outcome") != "SUCCESS":
        parts.append(f"{r.get('outcome')} {r.get('reason')}".strip())
    if collisions:
        parts.append("过渡碰撞")
    codes = _switch_codes(block)
    if r.get("outcome") != "SUCCESS" or collisions:
        if codes:
            parts.append(", ".join(f"{k}:{v}" for k, v in codes.items()))
    return " ".join(parts)


def _fmt(x, nd=3):
    return "—" if x is None else f"{float(x):.{nd}f}"
