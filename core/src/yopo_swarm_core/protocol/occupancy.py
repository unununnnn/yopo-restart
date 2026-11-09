"""占用表示与冲突判定接口。

#5 用抽象占用（时间区间 × 区域标签）；#10 的真实计划占用（plan_occupancy.py）实现同一个
`ConflictChecker` 协议：conflicts(a, b) 返回 True 表示「冲突或无法证明不冲突」。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Protocol


class Occupancy(Protocol):
    def truncated(self, until: float) -> Occupancy: ...
    def end_time(self) -> float: ...
    def execution_end(self) -> float:
        """名义运动被替换的时刻（I1 只要求新旧名义执行期不重叠；回退分支可延续）。"""
        ...

    def is_empty(self) -> bool: ...
    def to_canonical(self) -> Any: ...
    def is_finite(self) -> bool: ...


class ConflictChecker(Protocol):
    def conflicts(self, a: Any, b: Any) -> bool:
        """True = 冲突或无法证明不冲突（UNKNOWN 按冲突处理）。"""
        ...


def _num(x: float) -> Any:
    return "inf" if x == math.inf else ("-inf" if x == -math.inf else x)


@dataclass(frozen=True, order=True)
class Span:
    t0: float
    t1: float
    region: str


@dataclass(frozen=True)
class AbstractOccupancy:
    spans: tuple[Span, ...]

    @staticmethod
    def of(*spans: Span) -> AbstractOccupancy:
        return AbstractOccupancy(tuple(sorted(spans)))

    def truncated(self, until: float) -> AbstractOccupancy:
        out = [Span(s.t0, min(s.t1, until), s.region) for s in self.spans if s.t0 < until]
        return AbstractOccupancy(tuple(s for s in out if s.t1 > s.t0))

    def end_time(self) -> float:
        return max((s.t1 for s in self.spans), default=-math.inf)

    def execution_end(self) -> float:
        return self.end_time()

    def is_empty(self) -> bool:
        return not self.spans

    def is_finite(self) -> bool:
        # 终端占用允许 t1 = +inf；NaN 与 t0 = ±inf 不允许
        return all(
            not math.isnan(s.t0) and not math.isnan(s.t1) and math.isfinite(s.t0) and s.t1 > s.t0
            for s in self.spans
        )

    def to_canonical(self) -> Any:
        return [[_num(s.t0), _num(s.t1), s.region] for s in self.spans]


class AbstractConflictChecker:
    """同一区域、时间区间重叠即冲突。"""

    def conflicts(self, a: AbstractOccupancy, b: AbstractOccupancy) -> bool:
        for x in a.spans:
            for y in b.spans:
                if x.region == y.region and x.t0 < y.t1 and y.t0 < x.t1:
                    return True
        return False
