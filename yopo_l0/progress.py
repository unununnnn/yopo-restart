"""票 13b：场景卡上的进度门控。

林带默认开，开阔区必须关。分位 0 是全队最小值。
"""

from __future__ import annotations

import math


def read_progress_settings(task: dict, *, default_gate: bool) -> tuple[bool, float]:
    """读 progress_gate / progress_percentile。缺省门控由调用方决定，分位默认 0（min）。"""
    if "progress_gate" in task:
        gate = task["progress_gate"]
        if not isinstance(gate, bool):
            raise ValueError("progress_gate 必须是布尔")
    else:
        gate = bool(default_gate)
    raw = task.get("progress_percentile", 0)
    try:
        pct = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"progress_percentile 必须是数字，收到 {raw!r}") from exc
    if not math.isfinite(pct) or pct < 0.0 or pct > 100.0:
        raise ValueError("progress_percentile 必须在 [0, 100]")
    return gate, pct


def read_leader_margin_m(task: dict, *, default: float = 1.0) -> float:
    """读 task.leader_margin_m。缺这个字段时用 default，穿林缺省 1.0。

    与 read_progress_settings 一起在 CrossingFleet 构造时读。缺省值与改前的
    模块常量相同，不改已有场景卡的飞行结果。
    """
    if "leader_margin_m" not in task:
        margin = float(default)
    else:
        raw = task["leader_margin_m"]
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"leader_margin_m 必须是数字，收到 {raw!r}")
        margin = float(raw)
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError(f"leader_margin_m 必须是非负有限数，收到 {margin!r}")
    return margin
