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
