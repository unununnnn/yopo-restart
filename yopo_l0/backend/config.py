"""后段开关。纯 Python，不导入 numpy / scipy。

场景卡 ``policy_params.backend.enabled`` 缺省或不是 JSON true 时，调用方拿到 None，
不读几何，也不拟合。已有卡不写这个字段，行为与加后段之前相同。

``swarm_penalty`` 是后段里的机间椭球项（E-B3）。只有 ``enabled`` 和
``swarm_penalty`` 都是 JSON true 时才打开。缺省关。关上时不读邻机，也不改代价。
``swarm_clearance_m``（C）和 ``swarm_c``（c）是假设，不是论文复现：正文只写
c > 1、C 由用户定，没有给数值。
"""

from __future__ import annotations

from dataclasses import dataclass

# 假设，不是论文复现。论文只写 c > 1，C 由用户定。
# 椭球距离 d = sqrt(dx² + dy² + (c·dz)²)，d < C 时加平方铰链。
# 竖直半轴是 C/c，比水平半轴短。
DEFAULT_SWARM_CLEARANCE_M = 0.80
DEFAULT_SWARM_C = 2.0


@dataclass(frozen=True)
class BackendConfig:
    """一次后段调用的限额。代价权重写在求解器里，不进场景卡。"""

    a_max: float = 6.0
    v_max: float = 6.0
    max_iterations: int = 8
    time_budget_s: float = 0.020
    clearance_m: float = 0.35
    swarm_penalty: bool = False
    swarm_clearance_m: float = DEFAULT_SWARM_CLEARANCE_M
    swarm_c: float = DEFAULT_SWARM_C

    def __post_init__(self) -> None:
        if not (self.a_max > 0.0 and self.v_max > 0.0 and self.clearance_m > 0.0):
            raise ValueError("a_max、v_max、clearance_m 必须为正")
        if int(self.max_iterations) < 1:
            raise ValueError("max_iterations 至少为 1")
        if not (self.time_budget_s > 0.0):
            raise ValueError("time_budget_s 必须为正")
        if self.swarm_penalty:
            if not (self.swarm_clearance_m > 0.0):
                raise ValueError("swarm_clearance_m 必须为正（假设，论文未给数值）")
            if not (self.swarm_c > 1.0):
                raise ValueError("swarm_c 必须大于 1（假设，论文只写 c > 1）")
        object.__setattr__(self, "max_iterations", int(self.max_iterations))
        object.__setattr__(self, "swarm_penalty", bool(self.swarm_penalty))


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
    swarm = block.get("swarm_penalty") is True
    return BackendConfig(
        a_max=float(block.get("a_max", a_max)),
        v_max=float(block.get("v_max", 6.0)),
        max_iterations=int(block.get("max_iterations", 8)),
        time_budget_s=float(block.get("time_budget_s", 0.020)),
        clearance_m=0.35 if clear is None else float(clear),
        swarm_penalty=swarm,
        swarm_clearance_m=float(block.get("swarm_clearance_m", DEFAULT_SWARM_CLEARANCE_M)),
        swarm_c=float(block.get("swarm_c", DEFAULT_SWARM_C)),
    )


def swarm_penalty_enabled(params: dict | None, *, a_max: float = 6.0) -> bool:
    """后段和机间椭球都打开时为 True。关着或缺省时不读 C、c。"""
    cfg = config_from_params(params, a_max=a_max)
    return cfg is not None and cfg.swarm_penalty
