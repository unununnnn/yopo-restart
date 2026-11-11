"""后段开关。纯 Python，不导入 numpy / scipy。

场景卡 ``policy_params.backend.enabled`` 缺省或不是 JSON true 时，调用方拿到 None，
不读几何，也不拟合。已有卡不写这个字段，行为与加后段之前相同。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class BackendConfig:
    """一次后段调用的限额。代价权重写在求解器里，不进场景卡。"""

    a_max: float = 6.0
    v_max: float = 6.0
    max_iterations: int = 8
    time_budget_s: float = 0.020
    clearance_m: float = 0.35

    def __post_init__(self) -> None:
        if not (self.a_max > 0.0 and self.v_max > 0.0 and self.clearance_m > 0.0):
            raise ValueError("a_max、v_max、clearance_m 必须为正")
        if int(self.max_iterations) < 1:
            raise ValueError("max_iterations 至少为 1")
        if not (self.time_budget_s > 0.0):
            raise ValueError("time_budget_s 必须为正")
        object.__setattr__(self, "max_iterations", int(self.max_iterations))


def config_from_params(
    params: dict | None,
    *,
    a_max: float,
    clearance_m: float | None = None,
) -> BackendConfig | None:
    """只有 ``enabled`` 为 JSON true 才返回配置。其它值，包括缺省，都是关。"""
    block = (params or {}).get("backend")
    if not isinstance(block, dict) or block.get("enabled") is not True:
        return None
    clear = block["clearance_m"] if "clearance_m" in block else clearance_m
    return BackendConfig(
        a_max=float(block.get("a_max", a_max)),
        v_max=float(block.get("v_max", 6.0)),
        max_iterations=int(block.get("max_iterations", 8)),
        time_budget_s=float(block.get("time_budget_s", 0.020)),
        clearance_m=0.35 if clear is None else float(clear),
    )
