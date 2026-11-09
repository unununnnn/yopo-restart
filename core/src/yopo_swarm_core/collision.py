"""连续时间成对冲突检查与动力学约束检查（ROS-free）。

方法：在双方分段边界、额外切换时刻处切分公共时间窗；每个子区间把多项式重参数化到
u ∈ [0, 1]，转 Bernstein 基，用控制点的轴对齐包围盒给出保守界；证不出就二分。
预算内无法判定返回 UNKNOWN。擦边相切永远不会被判 SAFE。
"""

from __future__ import annotations

import heapq
import itertools
import math
from collections.abc import Iterable
from dataclasses import dataclass
from enum import Enum
from functools import lru_cache

import numpy as np
from numpy.polynomial import Polynomial

from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment

DEFAULT_TOL = 1e-9
DEFAULT_MAX_DEPTH = 40
DEFAULT_MAX_NODES = 5_000


class Verdict(Enum):
    SAFE = "SAFE"
    UNSAFE = "UNSAFE"
    UNKNOWN = "UNKNOWN"


class Reason(Enum):
    PROVEN = "PROVEN"
    VIOLATION = "VIOLATION"
    NO_COMMON_WINDOW = "NO_COMMON_WINDOW"
    MALFORMED = "MALFORMED"
    BUDGET_EXHAUSTED = "BUDGET_EXHAUSTED"


@dataclass(frozen=True)
class CheckResult:
    verdict: Verdict
    reason: Reason
    witness_time: float | None = None
    # SAFE 时：SAFE 判定的保守界（间距检查为最小距离下界，导数检查为范数上界）
    lower_bound: float | None = None
    upper_bound: float | None = None


# 兼容命名
SeparationResult = CheckResult


# --- 安全余量 ---------------------------------------------------------------


@dataclass(frozen=True)
class ConstantMargin:
    value: float

    def at(self, t: float) -> float:
        return float(self.value)

    def max_over(self, t0: float, t1: float) -> float:
        return float(self.value)


@dataclass(frozen=True)
class LinearMargin:
    """m(t) = value0 + rate (t - t0)。"""

    t0: float
    value0: float
    rate: float

    def at(self, t: float) -> float:
        return self.value0 + self.rate * (t - self.t0)

    def max_over(self, t0: float, t1: float) -> float:
        return max(self.at(t0), self.at(t1))


# --- Bernstein 工具 ---------------------------------------------------------


@lru_cache(maxsize=32)
def _power_to_bernstein_matrix(n: int) -> np.ndarray:
    m = np.zeros((n + 1, n + 1))
    for k in range(n + 1):
        for j in range(k + 1):
            m[k, j] = math.comb(k, j) / math.comb(n, j)
    return m


def _bernstein(coeffs_u: np.ndarray) -> np.ndarray:
    """(3, n+1) power-basis（变量 u∈[0,1]）→ (3, n+1) Bernstein 控制点。"""
    n = coeffs_u.shape[1] - 1
    return coeffs_u @ _power_to_bernstein_matrix(n).T


def _reparam(coeffs: np.ndarray, tau0: float, h: float) -> np.ndarray:
    """p(τ) → q(u) = p(τ0 + h u)，逐轴。"""
    sub = Polynomial([tau0, h])
    rows = [Polynomial(row)(sub).coef for row in coeffs]
    width = coeffs.shape[1]
    return np.array([np.pad(r, (0, width - len(r)))[:width] for r in rows])


@dataclass(frozen=True)
class _Piece:
    seg: PolySegment
    abs_start: float


def _piece_at(traj: PiecewisePolynomial, t_mid: float) -> _Piece:
    i, _ = traj.locate(t_mid)
    return _Piece(traj.segments[i], traj.boundaries()[i])


def _breakpoints(lo: float, hi: float, *groups: Iterable[float]) -> list[float]:
    pts = {lo, hi}
    for g in groups:
        pts.update(t for t in g if lo < t < hi)
    return sorted(pts)


# --- 成对间距 ---------------------------------------------------------------


def check_separation(
    a: PiecewisePolynomial,
    b: PiecewisePolynomial,
    margin: ConstantMargin | LinearMargin,
    window: tuple[float, float] | None = None,
    split_times: Iterable[float] = (),
    max_depth: int = DEFAULT_MAX_DEPTH,
    max_nodes: int = DEFAULT_MAX_NODES,
    tol: float = DEFAULT_TOL,
) -> CheckResult:
    """在整个公共时间窗内判定 ||a(t) - b(t)|| >= margin(t)。"""
    if not (a.is_well_formed() and b.is_well_formed()):
        return CheckResult(Verdict.UNKNOWN, Reason.MALFORMED)

    if window is None:
        lo, hi = max(a.t_start, b.t_start), min(a.t_end, b.t_end)
    else:
        lo, hi = float(window[0]), float(window[1])
        if not (a.covers(lo, hi) and b.covers(lo, hi)):
            return CheckResult(Verdict.UNKNOWN, Reason.NO_COMMON_WINDOW)
    if not (math.isfinite(lo) and math.isfinite(hi)) or hi <= lo:
        return CheckResult(Verdict.UNKNOWN, Reason.NO_COMMON_WINDOW)
    if not (math.isfinite(margin.at(lo)) and math.isfinite(margin.at(hi))):
        return CheckResult(Verdict.UNKNOWN, Reason.MALFORMED)

    split_times = list(split_times)
    pts = _breakpoints(lo, hi, a.boundaries(), b.boundaries(), split_times)

    def rel_bernstein(pa: _Piece, pb: _Piece, s: float, e: float) -> np.ndarray:
        h = e - s
        ca = _reparam(pa.seg.coeffs, s - pa.abs_start, h)
        cb = _reparam(pb.seg.coeffs, s - pb.abs_start, h)
        n = max(ca.shape[1], cb.shape[1])
        rel = np.pad(ca, ((0, 0), (0, n - ca.shape[1]))) - np.pad(
            cb, ((0, 0), (0, n - cb.shape[1]))
        )
        return _bernstein(rel)

    def exact_slack(t: float) -> float:
        """真实距离减余量（用公开的 evaluate 语义：段边界处取后一段）。"""
        return float(np.linalg.norm(a.evaluate(t) - b.evaluate(t))) - margin.at(t)

    unknown = False
    # 断点处逐点精确检查：轨迹在段边界处可能不连续，子区间包络只覆盖单段的闭包。
    for t in pts:
        sl = exact_slack(t)
        if sl < -tol:
            return CheckResult(Verdict.UNSAFE, Reason.VIOLATION, witness_time=t)
        if sl <= tol:
            unknown = True

    heap: list[tuple[float, int, float, float, _Piece, _Piece, int]] = []
    counter = 0
    for s, e in itertools.pairwise(pts):
        mid = 0.5 * (s + e)
        heap.append((0.0, counter, s, e, _piece_at(a, mid), _piece_at(b, mid), 0))
        counter += 1
    heapq.heapify(heap)

    nodes = 0
    min_lb = math.inf
    while heap:
        _, _, s, e, pa, pb, depth = heapq.heappop(heap)
        nodes += 1
        ctrl = rel_bernstein(pa, pb, s, e)
        box_lo, box_hi = ctrl.min(axis=1), ctrl.max(axis=1)
        gap = np.maximum(np.maximum(box_lo, -box_hi), 0.0)
        lb = float(np.linalg.norm(gap))
        if lb > margin.max_over(s, e) + tol:
            min_lb = min(min_lb, lb)
            continue
        mid = 0.5 * (s + e)
        sl = exact_slack(mid)
        if sl < -tol:
            return CheckResult(Verdict.UNSAFE, Reason.VIOLATION, witness_time=mid)
        m_hi = margin.max_over(s, e)
        # 中点落在容差带里只说明这个节点证不出 SAFE，不是区间极值。包络下界仍
        # 远低于余量时必须继续细分，否则采样点之间的碰撞会被提前判成 UNKNOWN。
        # 下界已经贴着余量时，再分也得不到 UNSAFE。
        sample_blocks_safe = sl <= tol
        bound_blocks_unsafe = lb >= m_hi - tol
        budget_done = (
            depth >= max_depth or nodes >= max_nodes or (e - s) <= 1e-15 * max(1.0, abs(s))
        )
        if (sample_blocks_safe and bound_blocks_unsafe) or budget_done:
            unknown = True
            continue
        if sample_blocks_safe:
            unknown = True
        slack = lb - m_hi
        for s2, e2 in ((s, mid), (mid, e)):
            heapq.heappush(heap, (slack, counter, s2, e2, pa, pb, depth + 1))
            counter += 1

    if unknown:
        return CheckResult(Verdict.UNKNOWN, Reason.BUDGET_EXHAUSTED)
    return CheckResult(Verdict.SAFE, Reason.PROVEN, lower_bound=min_lb)


# --- 动力学约束 -------------------------------------------------------------


def check_derivative_bound(
    traj: PiecewisePolynomial,
    order: int,
    limit: float,
    window: tuple[float, float] | None = None,
    max_depth: int = DEFAULT_MAX_DEPTH,
    max_nodes: int = DEFAULT_MAX_NODES,
    tol: float = DEFAULT_TOL,
) -> CheckResult:
    """判定第 order 阶导数的欧氏范数在整个有效期（或 window）内 <= limit。

    SAFE 表示上界严格低于 limit；只在采样点之间超限也会被二分找到。
    """
    if order < 1 or not traj.is_well_formed() or not math.isfinite(limit):
        return CheckResult(Verdict.UNKNOWN, Reason.MALFORMED)
    lo, hi = (traj.t_start, traj.t_end) if window is None else window
    if not traj.covers(lo, hi) or hi <= lo:
        return CheckResult(Verdict.UNKNOWN, Reason.NO_COMMON_WINDOW)

    pts = _breakpoints(lo, hi, traj.boundaries())
    unknown = False
    for t in pts:
        excess = float(np.linalg.norm(traj.evaluate(t, order))) - limit
        if excess > tol:
            return CheckResult(Verdict.UNSAFE, Reason.VIOLATION, witness_time=t)
        if excess >= -tol:
            unknown = True
    stack = []
    for s, e in itertools.pairwise(pts):
        stack.append((s, e, _piece_at(traj, 0.5 * (s + e)), 0))

    nodes = 0
    max_ub = 0.0
    while stack:
        s, e, piece, depth = stack.pop()
        nodes += 1
        dcoef = piece.seg.derivative_coeffs(order)
        ctrl = _bernstein(_reparam(dcoef, s - piece.abs_start, e - s))
        ub = float(np.sqrt(np.sum(np.max(np.abs(ctrl), axis=1) ** 2)))
        if ub < limit - tol:
            max_ub = max(max_ub, ub)
            continue
        mid = 0.5 * (s + e)
        excess = float(np.linalg.norm(traj.evaluate(mid, order))) - limit
        if excess > tol:
            return CheckResult(Verdict.UNSAFE, Reason.VIOLATION, witness_time=mid)
        # 中点贴着限幅只说明证不出 SAFE。上界仍明显高于限幅时，峰值可能在
        # 别的时刻，必须继续细分。
        sample_blocks_safe = excess >= -tol
        bound_blocks_unsafe = ub <= limit + tol
        budget_done = (
            depth >= max_depth or nodes >= max_nodes or (e - s) <= 1e-15 * max(1.0, abs(s))
        )
        if (sample_blocks_safe and bound_blocks_unsafe) or budget_done:
            unknown = True
            continue
        if sample_blocks_safe:
            unknown = True
        stack.append((s, mid, piece, depth + 1))
        stack.append((mid, e, piece, depth + 1))

    if unknown:
        return CheckResult(Verdict.UNKNOWN, Reason.BUDGET_EXHAUSTED)
    return CheckResult(Verdict.SAFE, Reason.PROVEN, upper_bound=max_ub)
