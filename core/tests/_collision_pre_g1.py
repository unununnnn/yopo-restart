"""Frozen pre-55cc533 reference, test-only.

``check_separation`` and ``check_derivative_bound`` are the function bodies at
``e9762b1`` (``git show e9762b1:core/src/yopo_swarm_core/collision.py``). Every
other name is imported from the current ``yopo_swarm_core.collision``.
"""

from __future__ import annotations

import heapq
import itertools
import math
from collections.abc import Iterable

import numpy as np

from yopo_swarm_core.collision import (
    DEFAULT_MAX_DEPTH,
    DEFAULT_MAX_NODES,
    DEFAULT_TOL,
    CheckResult,
    ConstantMargin,
    LinearMargin,
    Reason,
    Verdict,
    _bernstein,
    _breakpoints,
    _Piece,
    _piece_at,
    _reparam,
)
from yopo_swarm_core.trajectory import PiecewisePolynomial


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
        if sl <= tol:
            # 擦边带：真实最小值已不高于 余量+tol，此节点永远证不出 SAFE。
            unknown = True
            continue
        if depth >= max_depth or nodes >= max_nodes or (e - s) <= 1e-15 * max(1.0, abs(s)):
            unknown = True
            continue
        slack = lb - margin.max_over(s, e)
        for s2, e2 in ((s, mid), (mid, e)):
            heapq.heappush(heap, (slack, counter, s2, e2, pa, pb, depth + 1))
            counter += 1

    if unknown:
        return CheckResult(Verdict.UNKNOWN, Reason.BUDGET_EXHAUSTED)
    return CheckResult(Verdict.SAFE, Reason.PROVEN, lower_bound=min_lb)


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
        if excess >= -tol or depth >= max_depth or nodes >= max_nodes:
            unknown = True
            continue
        stack.append((s, mid, piece, depth + 1))
        stack.append((mid, e, piece, depth + 1))

    if unknown:
        return CheckResult(Verdict.UNKNOWN, Reason.BUDGET_EXHAUSTED)
    return CheckResult(Verdict.SAFE, Reason.PROVEN, upper_bound=max_ub)
