"""分段多项式轨迹：绝对时间、显式有效期，ROS-free。

每段系数为 power basis，形状 (3, deg+1)，按段内局部时间 τ ∈ [0, duration]：
p_axis(τ) = Σ_j coeffs[axis, j] τ^j。
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.polynomial import polynomial as P


class TrajectoryExpired(ValueError):
    """在有效期之外求值。不做端点截断。"""


@dataclass(frozen=True)
class PolySegment:
    coeffs: np.ndarray
    duration: float

    def __post_init__(self) -> None:
        c = np.atleast_2d(np.asarray(self.coeffs, dtype=float))
        if c.shape[0] != 3 or c.ndim != 2:
            raise ValueError(f"coeffs must have shape (3, deg+1), got {c.shape}")
        object.__setattr__(self, "coeffs", c)
        object.__setattr__(self, "duration", float(self.duration))

    @property
    def degree(self) -> int:
        return self.coeffs.shape[1] - 1

    def is_well_formed(self) -> bool:
        return (
            math.isfinite(self.duration)
            and self.duration > 0.0
            and bool(np.all(np.isfinite(self.coeffs)))
        )

    def derivative_coeffs(self, order: int) -> np.ndarray:
        if order == 0:
            return self.coeffs
        rows = [
            P.polyder(row, order) if self.degree >= order else np.zeros(1) for row in self.coeffs
        ]
        width = max(len(r) for r in rows)
        return np.array([np.pad(r, (0, width - len(r))) for r in rows])

    def evaluate_local(self, tau: float, order: int = 0) -> np.ndarray:
        return np.array([P.polyval(tau, row) for row in self.derivative_coeffs(order)])


class PiecewisePolynomial:
    def __init__(
        self,
        t0: float,
        segments: Sequence[PolySegment],
        valid_from: float | None = None,
        valid_until: float | None = None,
    ) -> None:
        if not segments:
            raise ValueError("at least one segment required")
        self.t0 = float(t0)
        self.segments: tuple[PolySegment, ...] = tuple(segments)
        bounds = [self.t0]
        for seg in self.segments:
            bounds.append(bounds[-1] + seg.duration)
        self._bounds: tuple[float, ...] = tuple(bounds)
        support_end = bounds[-1]
        self.t_start = self.t0 if valid_from is None else float(valid_from)
        self.t_end = support_end if valid_until is None else float(valid_until)

    # --- structure -------------------------------------------------------
    def support_end(self) -> float:
        return self._bounds[-1]

    def is_well_formed(self) -> bool:
        if not all(s.is_well_formed() for s in self.segments):
            return False
        if not (math.isfinite(self.t0) and math.isfinite(self.t_start)):
            return False
        if not math.isfinite(self.t_end) or self.t_end <= self.t_start:
            return False
        eps = 1e-12 * max(1.0, abs(self.t_end))
        return self.t_start >= self.t0 - eps and self.t_end <= self.support_end() + eps

    def boundaries(self) -> list[float]:
        """各段绝对起止时刻（多项式支撑，与有效期无关）。"""
        return list(self._bounds)

    def with_validity(self, valid_from: float, valid_until: float) -> PiecewisePolynomial:
        return PiecewisePolynomial(self.t0, self.segments, valid_from, valid_until)

    def shifted(self, dt: float) -> PiecewisePolynomial:
        return PiecewisePolynomial(self.t0 + dt, self.segments, self.t_start + dt, self.t_end + dt)

    def covers(self, t0: float, t1: float) -> bool:
        return self.t_start <= t0 and t1 <= self.t_end

    # --- evaluation ------------------------------------------------------
    def locate(self, t: float) -> tuple[int, float]:
        """返回 (段索引, 段内局部时间)。段边界处归入后一段（最后边界归入最后一段）。"""
        if not (self.t_start <= t <= self.t_end):
            raise TrajectoryExpired(f"t={t} outside validity [{self.t_start}, {self.t_end}]")
        last = len(self.segments) - 1
        for i, s in enumerate(self.segments):
            start, end = self._bounds[i], self._bounds[i + 1]
            if t < end or i == last:
                return i, min(max(t - start, 0.0), s.duration)
        raise AssertionError("unreachable")

    def evaluate(self, t: float, order: int = 0) -> np.ndarray:
        i, tau = self.locate(t)
        return self.segments[i].evaluate_local(tau, order)

    def pieces(self, t0: float, t1: float) -> list[tuple[float, float, PolySegment, float]]:
        """把 [t0, t1] 切成落在单段内的子区间：(a, b, segment, segment_abs_start)。"""
        out = []
        for i, s in enumerate(self.segments):
            start, end = self._bounds[i], self._bounds[i + 1]
            a, b = max(start, t0), min(end, t1)
            if b > a:
                out.append((a, b, s, start))
        return out
