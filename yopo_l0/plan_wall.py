"""单次规划墙钟：只记录，不改轨迹，不进确定性比较，也不进 gates。

每次网络前向（或以后的后段优化）往当前收集器里记一段墙钟。
`run()` 开头打开收集器，结束时汇总进结果记录的 `plan_wall_ms`。
没有打开收集器时（单测直接调网络）什么都不记，行为与加计时之前相同。
"""

from __future__ import annotations

import time
from collections.abc import Iterator
from contextlib import contextmanager

import numpy as np

_samples_ms: list[float] | None = None

PLAN_WALL_FIELD = "plan_wall_ms"


def begin() -> None:
    global _samples_ms
    _samples_ms = []


def take() -> list[float]:
    global _samples_ms
    out = list(_samples_ms or [])
    _samples_ms = None
    return out


def note_ms(ms: float) -> None:
    """后段优化或其它规划步骤把已经量好的毫秒数记进来。"""
    if _samples_ms is not None:
        _samples_ms.append(float(ms))


@contextmanager
def measure() -> Iterator[None]:
    """包住一次网络前向或后段优化。收集器没打开时仍照常执行，不计时。"""
    if _samples_ms is None:
        yield
        return
    t0 = time.perf_counter()
    try:
        yield
    finally:
        _samples_ms.append((time.perf_counter() - t0) * 1000.0)


def stats(samples_ms: list[float]) -> dict:
    if not samples_ms:
        return {"count": 0, "mean": None, "p50": None, "p95": None, "max": None}
    arr = np.asarray(samples_ms, dtype=float)
    return {
        "count": int(arr.size),
        "mean": float(np.mean(arr)),
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(np.max(arr)),
    }


def p95_ms(row: dict) -> float | None:
    """结果记录里单次规划墙钟的 p95（毫秒）。字段缺失或 count=0 时返回 None。"""
    block = row.get(PLAN_WALL_FIELD)
    if not isinstance(block, dict):
        return None
    val = block.get("p95")
    if val is None:
        return None
    return float(val)
