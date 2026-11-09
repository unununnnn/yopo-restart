"""计划安全契约（ADR-0002）：运动前缀 + 固定分支点制动分支 + 终端占用 + 占用集合 O_i(t)。

- 计划 = 从 t_switch 起的运动前缀（头状态 PVA 必须与父计划在 t_switch 的状态一致，I8）
  + 少量固定分支点上的制动分支（每个都从该点的 PVA 出发、减速到静止、满足动力学限制）
  + 终端保持（运动前缀结束点；在被显式替换前一直有效）。
- 占用集合是各部分跟踪误差管之并：运动前缀、各制动分支及其停点保持、终端保持。
  冲突判定作用于占用集合，不是两条名义轨迹（I14）。
- 动力学与间距检查都用 #4 的保守连续时间检查；UNKNOWN 一律按不通过 / 冲突处理。
- plan_hash 对字段顺序无关，对所有安全相关字段（系数、时长、绝对时间、坐标系/单位/限幅/半径）敏感。
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Any

import numpy as np

from yopo_swarm_core.collision import (
    ConstantMargin,
    Verdict,
    check_derivative_bound,
    check_separation,
)
from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment

CONTINUITY_TOL = 1e-6
HOLD_TOL = 1e-6


class PlanReject(Enum):
    MALFORMED = "MALFORMED"
    NO_BRAKE_BRANCH = "NO_BRAKE_BRANCH"
    TOO_MANY_BRANCH_POINTS = "TOO_MANY_BRANCH_POINTS"
    HEAD_DISCONTINUITY = "HEAD_DISCONTINUITY"
    MOTION_KINEMATICS = "MOTION_KINEMATICS"
    BRANCH_OUTSIDE_PREFIX = "BRANCH_OUTSIDE_PREFIX"
    HOVER_AS_BRAKE = "HOVER_AS_BRAKE"
    BRANCH_DISCONTINUITY = "BRANCH_DISCONTINUITY"
    BRANCH_NOT_STOPPING = "BRANCH_NOT_STOPPING"
    BRANCH_KINEMATICS = "BRANCH_KINEMATICS"
    BRANCH_UNREACHABLE = "BRANCH_UNREACHABLE"
    TERMINAL_NOT_HOLDABLE = "TERMINAL_NOT_HOLDABLE"


@dataclass(frozen=True)
class KinematicLimits:
    v_max: float
    a_max: float
    j_max: float

    def by_order(self) -> tuple[tuple[int, float], ...]:
        return ((1, self.v_max), (2, self.a_max), (3, self.j_max))


@dataclass(frozen=True)
class PlanConfig:
    limits: KinematicLimits
    max_branch_points: int = 4
    body_radius: float = 0.2
    tracking_margin: float = 0.1  # 跟踪误差管半径
    frame: str = "world-ENU"
    units: str = "SI"

    @property
    def tube_radius(self) -> float:
        return self.body_radius + self.tracking_margin

    def to_canonical(self) -> dict:
        return {
            "frame": self.frame,
            "units": self.units,
            "limits": [_f(self.limits.v_max), _f(self.limits.a_max), _f(self.limits.j_max)],
            "max_branch_points": self.max_branch_points,
            "body_radius": _f(self.body_radius),
            "tracking_margin": _f(self.tracking_margin),
        }


# --- 规范化与哈希 -------------------------------------------------------------


def _f(x: float) -> str:
    """浮点数精确规范化（float.hex），任何一位变化都会改变哈希。"""
    return float(x).hex()


def _traj_canonical(tr: PiecewisePolynomial) -> dict:
    return {
        "t0": _f(tr.t0),
        "valid": [_f(tr.t_start), _f(tr.t_end)],
        "segments": [
            {"duration": _f(s.duration), "coeffs": [[_f(c) for c in row] for row in s.coeffs]}
            for s in tr.segments
        ],
    }


def canonical_digest(obj: Any) -> str:
    text = json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(text.encode()).hexdigest()


# --- 多项式构造 ---------------------------------------------------------------


def quintic(start: np.ndarray, end: np.ndarray, duration: float) -> PolySegment:
    """三轴五次多项式，满足两端 PVA（start/end 形状 (3,3)：行 = p, v, a）。"""
    T = float(duration)
    p0, v0, a0 = np.asarray(start, dtype=float)
    p1, v1, a1 = np.asarray(end, dtype=float)
    c0, c1, c2 = p0, v0, a0 / 2
    m = np.array(
        [[T**3, T**4, T**5], [3 * T**2, 4 * T**3, 5 * T**4], [6 * T, 12 * T**2, 20 * T**3]]
    )
    rhs = np.stack([p1 - (c0 + c1 * T + c2 * T**2), v1 - (c1 + 2 * c2 * T), a1 - 2 * c2])
    c345 = np.linalg.solve(m, rhs)
    return PolySegment(np.column_stack([c0, c1, c2, *c345]), T)


def _kinematics_ok(traj: PiecewisePolynomial, limits: KinematicLimits) -> bool:
    return all(
        check_derivative_bound(traj, order, lim).verdict is Verdict.SAFE
        for order, lim in limits.by_order()
    )


def brake_branch(
    t_branch: float,
    head: np.ndarray,
    limits: KinematicLimits,
    max_duration: float = 30.0,
) -> PiecewisePolynomial | None:
    """从 head=(p,v,a) 出发减速到静止的五次分支；时长从物理下界起逐步加长直到保守检查通过。"""
    p, v, a = np.asarray(head, dtype=float)
    speed = float(np.linalg.norm(v))
    T = max(0.2, 1.5 * speed / limits.a_max, float(np.linalg.norm(a)) / limits.j_max)
    while T <= max_duration:
        stop = p + v * T / 2 + a * T**2 / 12  # 自然减速停点
        tr = PiecewisePolynomial(
            t_branch, [quintic(head, np.array([stop, np.zeros(3), np.zeros(3)]), T)]
        )
        if _kinematics_ok(tr, limits):
            return tr
        T *= 1.25
    return None


# --- 计划 ---------------------------------------------------------------------


@dataclass(frozen=True)
class BrakeBranch:
    t_branch: float
    traj: PiecewisePolynomial

    def stop_point(self) -> np.ndarray:
        return self.traj.evaluate(self.traj.t_end)


@dataclass(frozen=True)
class Plan:
    owner: str
    plan_seq: int
    parent_plan_hash: str
    t_switch: float
    head_pva: np.ndarray
    motion: PiecewisePolynomial
    branches: tuple[BrakeBranch, ...]

    @property
    def t_end(self) -> float:
        return self.motion.t_end

    def terminal_point(self) -> np.ndarray:
        return self.motion.evaluate(self.motion.t_end)

    # 修改器（不可变）
    def replace_branches(self, branches: Iterable[BrakeBranch]) -> Plan:
        return replace(self, branches=tuple(branches))

    def with_owner(self, owner: str) -> Plan:
        return replace(self, owner=owner)

    def retimed(self, dt: float) -> Plan:
        return replace(
            self,
            t_switch=self.t_switch + dt,
            motion=self.motion.shifted(dt),
            branches=tuple(BrakeBranch(b.t_branch + dt, b.traj.shifted(dt)) for b in self.branches),
        )

    # 规范化
    def canonical(self, cfg: PlanConfig) -> dict:
        return {
            "owner": self.owner,
            "plan_seq": self.plan_seq,
            "parent_plan_hash": self.parent_plan_hash,
            "t_switch": _f(self.t_switch),
            "head_pva": [[_f(x) for x in row] for row in np.asarray(self.head_pva, dtype=float)],
            "motion": _traj_canonical(self.motion),
            "branches": [
                {"t_branch": _f(b.t_branch), "traj": _traj_canonical(b.traj)} for b in self.branches
            ],
            "terminal": "hold-at-motion-end-until-replaced",
            "config": cfg.to_canonical(),
        }

    def plan_hash(self, cfg: PlanConfig) -> str:
        return canonical_digest(self.canonical(cfg))

    # 占用
    def occupancy(self, cfg: PlanConfig) -> OccupancySet:
        r = cfg.tube_radius
        els: list[Tube | Hold] = [Tube(self.motion, r, "motion")]
        for i, b in enumerate(self.branches):
            els.append(Tube(b.traj, r, f"branch{i}"))
            els.append(Hold(tuple(b.stop_point()), r, b.traj.t_end, math.inf, f"branch{i}-hold"))
        els.append(Hold(tuple(self.terminal_point()), r, self.t_end, math.inf, "terminal"))
        return OccupancySet(tuple(els))

    def nominal_occupancy(self, cfg: PlanConfig) -> OccupancySet:
        """只含名义运动与终端保持（不含制动分支）——仅用于说明 I14，不得用于批准。"""
        r = cfg.tube_radius
        return OccupancySet(
            (
                Tube(self.motion, r, "motion"),
                Hold(tuple(self.terminal_point()), r, self.t_end, math.inf, "terminal"),
            )
        )


# --- 占用集合 -----------------------------------------------------------------


@dataclass(frozen=True)
class Tube:
    traj: PiecewisePolynomial
    radius: float
    label: str = ""

    @property
    def window(self) -> tuple[float, float]:
        return (self.traj.t_start, self.traj.t_end)


@dataclass(frozen=True)
class Hold:
    """静止保持：position 处半径 radius 的球，在 [t_from, t_until) 内（t_until 可为 inf）。"""

    position: tuple[float, float, float]
    radius: float
    t_from: float
    t_until: float = math.inf
    label: str = ""


@dataclass(frozen=True)
class OccupancySet:
    elements: tuple[Tube | Hold, ...] = field(default_factory=tuple)

    def union(self, *others: OccupancySet) -> OccupancySet:
        els = list(self.elements)
        for o in others:
            els += o.elements
        return OccupancySet(tuple(els))

    def truncated_after(self, t: float) -> OccupancySet:
        """去掉 t 之前已结束的元素（仍保留跨越 t 的元素全部内容，保守）。"""
        keep = [
            e for e in self.elements if (e.traj.t_end if isinstance(e, Tube) else e.t_until) > t
        ]
        return OccupancySet(tuple(keep))


def _const_traj(pos: Sequence[float], t0: float, t1: float) -> PiecewisePolynomial:
    c = np.zeros((3, 1))
    c[:, 0] = pos
    return PiecewisePolynomial(t0, [PolySegment(c, t1 - t0)])


def _pair_conflict(a: Tube | Hold, b: Tube | Hold) -> bool:
    r = a.radius + b.radius
    if isinstance(a, Hold) and isinstance(b, Hold):
        if max(a.t_from, b.t_from) >= min(a.t_until, b.t_until):
            return False
        d = float(np.linalg.norm(np.subtract(a.position, b.position)))
        return not d > r  # 相切算冲突
    if isinstance(a, Hold):
        a, b = b, a
    assert isinstance(a, Tube)
    lo, hi = a.window
    if isinstance(b, Hold):
        lo, hi = max(lo, b.t_from), min(hi, b.t_until)
        if hi <= lo:
            return False
        other = _const_traj(b.position, lo, hi)
    else:
        lo, hi = max(lo, b.traj.t_start), min(hi, b.traj.t_end)
        if hi <= lo:
            return False
        other = b.traj
    res = check_separation(a.traj, other, ConstantMargin(r), window=(lo, hi))
    return res.verdict is not Verdict.SAFE  # UNKNOWN 按冲突


def conflicts(a: OccupancySet, b: OccupancySet) -> bool:
    return any(_pair_conflict(x, y) for x in a.elements for y in b.elements)


def conflict_witness(a: OccupancySet, b: OccupancySet) -> tuple[str, str] | None:
    for x in a.elements:
        for y in b.elements:
            if _pair_conflict(x, y):
                return (x.label, y.label)
    return None


# --- 校验与构造 ---------------------------------------------------------------


def _finite(*arrays: Any) -> bool:
    return all(bool(np.all(np.isfinite(np.asarray(x, dtype=float)))) for x in arrays)


def _close(x: np.ndarray, y: np.ndarray, tol: float = CONTINUITY_TOL) -> bool:
    return float(np.max(np.abs(np.asarray(x) - np.asarray(y)))) <= tol


def _pva_at(tr: PiecewisePolynomial, t: float) -> np.ndarray:
    return np.array([tr.evaluate(t, k) for k in range(3)])


def hold_traj(p: np.ndarray, t0: float, t1: float) -> PiecewisePolynomial:
    """在 p 静止保持的常数轨迹，有效区间 [t0, t1]。"""
    c = np.zeros((3, 1))
    c[:, 0] = p
    return PiecewisePolynomial(t0, [PolySegment(c, max(t1 - t0, 1e-6))])


def executing_traj(plan: Plan, t: float, active_branch: int | None = None) -> PiecewisePolynomial:
    """覆盖 t 的执行轨迹：运动段 / 第 active_branch 条制动分支；过了终点是终端保持。"""
    tr = plan.motion if active_branch is None else plan.branches[active_branch].traj
    if t <= tr.t_end:
        return tr
    return hold_traj(tr.evaluate(tr.t_end), tr.t_end, t + 1.0)


def state_at(plan: Plan, t: float, active_branch: int | None = None) -> np.ndarray:
    """按计划执行到 t 时的 (p, v, a)；运动或分支结束后为终端静止状态。"""
    return _pva_at(executing_traj(plan, t, active_branch), t)


def hover_plan(
    owner: str, position: np.ndarray, t0: float, cfg: PlanConfig, parent: str = "genesis"
) -> Plan:
    """原地悬停计划（运动段为静止保持，分支点在 t0）。"""
    p = np.asarray(position, dtype=float)
    head = np.array([p, np.zeros(3), np.zeros(3)])
    res = make_plan(owner, 0, parent, hold_traj(p, t0, t0 + 0.5), t0, head, [t0], cfg)
    if isinstance(res, Plan):
        return res
    raise ValueError(f"hover plan invalid: {res}")


def validate_plan(plan: Plan, cfg: PlanConfig) -> list[PlanReject]:
    """返回全部拒绝原因（空 = 通过）。"""
    head = np.asarray(plan.head_pva, dtype=float)
    if (
        head.shape != (3, 3)
        or not _finite(head, plan.t_switch)
        or not plan.motion.is_well_formed()
        or not all(b.traj.is_well_formed() and _finite(b.t_branch) for b in plan.branches)
    ):
        return [PlanReject.MALFORMED]
    out: list[PlanReject] = []
    m = plan.motion
    if abs(m.t_start - plan.t_switch) > CONTINUITY_TOL or not _close(_pva_at(m, m.t_start), head):
        out.append(PlanReject.HEAD_DISCONTINUITY)
    if not _kinematics_ok(m, cfg.limits):
        out.append(PlanReject.MOTION_KINEMATICS)
    end = _pva_at(m, m.t_end)
    if not (_close(end[1], np.zeros(3), HOLD_TOL) and _close(end[2], np.zeros(3), HOLD_TOL)):
        out.append(PlanReject.TERMINAL_NOT_HOLDABLE)
    if not plan.branches:
        out.append(PlanReject.NO_BRAKE_BRANCH)
    if len(plan.branches) > cfg.max_branch_points:
        out.append(PlanReject.TOO_MANY_BRANCH_POINTS)
    for b in plan.branches:
        out += _validate_branch(m, b, cfg)
    return list(dict.fromkeys(out))


def _validate_branch(m: PiecewisePolynomial, b: BrakeBranch, cfg: PlanConfig) -> list[PlanReject]:
    if not (m.t_start <= b.t_branch <= m.t_end):
        return [PlanReject.BRANCH_OUTSIDE_PREFIX]
    here = _pva_at(m, b.t_branch)
    br0 = _pva_at(b.traj, b.traj.t_start)
    out = []
    moving = float(np.linalg.norm(here[1])) > HOLD_TOL
    if moving and float(np.linalg.norm(br0[1])) <= HOLD_TOL:
        out.append(PlanReject.HOVER_AS_BRAKE)
    elif abs(b.traj.t_start - b.t_branch) > CONTINUITY_TOL or not _close(br0, here):
        out.append(PlanReject.BRANCH_DISCONTINUITY)
    tail = _pva_at(b.traj, b.traj.t_end)
    if not (_close(tail[1], np.zeros(3), HOLD_TOL) and _close(tail[2], np.zeros(3), HOLD_TOL)):
        out.append(PlanReject.BRANCH_NOT_STOPPING)
    if not _kinematics_ok(b.traj, cfg.limits):
        out.append(PlanReject.BRANCH_KINEMATICS)
    return out


def make_plan(
    owner: str,
    plan_seq: int,
    parent_plan_hash: str,
    motion: PiecewisePolynomial,
    t_switch: float,
    head_pva: np.ndarray,
    branch_times: Sequence[float],
    cfg: PlanConfig,
) -> Plan | list[PlanReject]:
    """在给定固定分支点上生成制动分支并整体校验；失败返回拒绝原因列表。"""
    head = np.asarray(head_pva, dtype=float)
    if (
        head.shape != (3, 3)
        or not _finite(head, t_switch, list(branch_times))
        or not motion.is_well_formed()
    ):
        return [PlanReject.MALFORMED]
    if len(branch_times) > cfg.max_branch_points:
        return [PlanReject.TOO_MANY_BRANCH_POINTS]
    branches, rejects = [], []
    for tb in sorted(branch_times):
        if not (motion.t_start <= tb <= motion.t_end):
            rejects.append(PlanReject.BRANCH_OUTSIDE_PREFIX)
            continue
        tr = brake_branch(tb, _pva_at(motion, tb), cfg.limits)
        if tr is None:
            rejects.append(PlanReject.BRANCH_UNREACHABLE)
            continue
        branches.append(BrakeBranch(tb, tr))
    plan = Plan(owner, plan_seq, parent_plan_hash, float(t_switch), head, motion, tuple(branches))
    found = validate_plan(plan, cfg)
    if rejects:
        found = [r for r in found if r is not PlanReject.NO_BRAKE_BRANCH]
    rejects = list(dict.fromkeys(rejects + found))
    return rejects or plan


__all__ = [
    "BrakeBranch",
    "Hold",
    "KinematicLimits",
    "OccupancySet",
    "Plan",
    "PlanConfig",
    "PlanReject",
    "Tube",
    "brake_branch",
    "canonical_digest",
    "conflict_witness",
    "conflicts",
    "executing_traj",
    "hold_traj",
    "hover_plan",
    "make_plan",
    "quintic",
    "state_at",
    "validate_plan",
]
