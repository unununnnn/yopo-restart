"""后段开关。纯 Python，不导入 numpy / scipy。

场景卡 ``policy_params.backend.enabled`` 缺省或不是 JSON true 时，调用方拿到 None，
不读几何，也不拟合。已有卡不写这个字段，行为与加后段之前相同。

``swarm_penalty`` 是后段里的机间椭球项（E-B3）。只有 ``enabled`` 和
``swarm_penalty`` 都是 JSON true 时才打开。缺省关。关上时不读邻机，也不改代价。
``swarm_clearance_m``（C）和 ``swarm_c``（c）是假设，不是论文复现：正文只写
c > 1、C 由用户定，没有给数值。

``swarm_segment_penalty`` 是 E-B3c。缺省关。只有它和 ``swarm_penalty`` 都是
JSON true 时，椭球项才改到每段的内点上算，梯度按 B 样条基函数送回控制点，
并且末端三个控制点不再钉死（头三个仍钉死当前 PVA）。关掉时（缺省、false、
字符串）后段与 E-B3 逐位相同。它自己不打开邻机采样：采样仍由
``swarm_penalty`` 决定，这样不必改策略里的接线。

``peer_sphere_enabled`` 是 E-B4c。缺省关，和椭球开关彼此独立：打开它不改
``swarm_penalty`` / ``swarm_segment_penalty`` 的语义，也不放开末端控制点。
关上时不读邻机，代价与加这项之前逐位相同。只有时间对齐的 3D 欧氏距离小于
``peer_sphere_radius_m``（默认 0.85 m = 0.70 m 碰撞球 + 0.15 m 裕度）才加
平滑二次/三次铰链，权重是 ``peer_sphere_weight``（默认 8）。也可经环境变量
``YOPO_PEER_SPHERE``（1/true/yes/on）在 ``enabled`` 已经为 JSON true 时打开；
半径和权重可用 ``YOPO_PEER_SPHERE_RADIUS_M``、``YOPO_PEER_SPHERE_WEIGHT`` 盖过。

``budget`` 是票 L0-DET。缺省 ``"evals"``：``time_budget_s`` 按空闲机器上每次
代价评估约 2 ms 折成评估次数上限（0.020 s → 10 次），与机器负载无关，同一
(场景卡, seed, 配置) 结果逐位相同。``"wall"`` 用真墙钟截断，留给实时形态
（SITL / 真机）；也可经环境变量 ``YOPO_BACKEND_BUDGET=wall`` 打开。
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

# 假设，不是论文复现。论文只写 c > 1，C 由用户定。
# 椭球距离 d = sqrt(dx² + dy² + (c·dz)²)，d < C 时加平方铰链。
# 竖直半轴是 C/c，比水平半轴短。
DEFAULT_SWARM_CLEARANCE_M = 0.80
DEFAULT_SWARM_C = 2.0
# 0.70 m 是 L0 机间碰撞球（机体半径 0.35 m 的两倍）。0.15 m 是球外的三次过渡带。
DEFAULT_PEER_SPHERE_RADIUS_M = 0.85
DEFAULT_PEER_SPHERE_WEIGHT = 8.0
# 票 L0-DET。仿真按评估次数截断，实时形态按墙钟截断。
BUDGET_MODES = ("evals", "wall")
DEFAULT_BUDGET = "evals"


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
    # E-B3c。关时椭球项仍只打在控制点上，和加这项之前逐位相同。
    swarm_segment_penalty: bool = False
    # E-B4c。关时不读邻机，自由集和椭球项都与加这项之前逐位相同。
    peer_sphere_enabled: bool = False
    peer_sphere_radius_m: float = DEFAULT_PEER_SPHERE_RADIUS_M
    peer_sphere_weight: float = DEFAULT_PEER_SPHERE_WEIGHT
    # 票 L0-DET。"evals"：预算折成评估次数，结果不随负载变。"wall"：真墙钟。
    budget: str = DEFAULT_BUDGET

    def __post_init__(self) -> None:
        if not (self.a_max > 0.0 and self.v_max > 0.0 and self.clearance_m > 0.0):
            raise ValueError("a_max、v_max、clearance_m 必须为正")
        if int(self.max_iterations) < 1:
            raise ValueError("max_iterations 至少为 1")
        if not (self.time_budget_s > 0.0):
            raise ValueError("time_budget_s 必须为正")
        if self.budget not in BUDGET_MODES:
            raise ValueError(f"budget 只能是 {BUDGET_MODES}")
        if self.swarm_penalty:
            if not (self.swarm_clearance_m > 0.0):
                raise ValueError("swarm_clearance_m 必须为正（假设，论文未给数值）")
            if not (self.swarm_c > 1.0):
                raise ValueError("swarm_c 必须大于 1（假设，论文只写 c > 1）")
        object.__setattr__(self, "max_iterations", int(self.max_iterations))
        object.__setattr__(self, "swarm_penalty", bool(self.swarm_penalty))
        object.__setattr__(self, "swarm_segment_penalty", bool(self.swarm_segment_penalty))
        object.__setattr__(self, "peer_sphere_enabled", bool(self.peer_sphere_enabled))
        if self.peer_sphere_enabled:
            radius = float(self.peer_sphere_radius_m)
            weight = float(self.peer_sphere_weight)
            if not (math.isfinite(radius) and radius > 0.0):
                raise ValueError("peer_sphere_radius_m 必须为正（0.70 m 碰撞球 + 安全裕度）")
            if not (math.isfinite(weight) and weight >= 0.0):
                raise ValueError("peer_sphere_weight 必须为非负有限值")


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
    # 只有 JSON true 才打开。缺省、false、字符串 "true" 都保持 E-B3 的控制点惩罚。
    segment = block.get("swarm_segment_penalty") is True
    # 椭球开关不连带打开欧氏球。JSON true，或 YOPO_PEER_SPHERE 为真，才打开。
    sphere = block.get("peer_sphere_enabled") is True or _env_flag("YOPO_PEER_SPHERE")
    budget = os.environ.get("YOPO_BACKEND_BUDGET", "").strip().lower() or block.get("budget", DEFAULT_BUDGET)
    radius = _env_float("YOPO_PEER_SPHERE_RADIUS_M")
    weight = _env_float("YOPO_PEER_SPHERE_WEIGHT")
    return BackendConfig(
        a_max=float(block.get("a_max", a_max)),
        v_max=float(block.get("v_max", 6.0)),
        max_iterations=int(block.get("max_iterations", 8)),
        time_budget_s=float(block.get("time_budget_s", 0.020)),
        clearance_m=0.35 if clear is None else float(clear),
        swarm_penalty=swarm,
        swarm_clearance_m=float(block.get("swarm_clearance_m", DEFAULT_SWARM_CLEARANCE_M)),
        swarm_c=float(block.get("swarm_c", DEFAULT_SWARM_C)),
        swarm_segment_penalty=segment,
        peer_sphere_enabled=sphere,
        peer_sphere_radius_m=float(block.get("peer_sphere_radius_m", DEFAULT_PEER_SPHERE_RADIUS_M))
        if radius is None
        else radius,
        peer_sphere_weight=float(block.get("peer_sphere_weight", DEFAULT_PEER_SPHERE_WEIGHT))
        if weight is None
        else weight,
        budget=budget,
    )


def swarm_penalty_enabled(params: dict | None, *, a_max: float = 6.0) -> bool:
    """后段和机间椭球都打开时为 True。关着或缺省时不读 C、c。"""
    cfg = config_from_params(params, a_max=a_max)
    return cfg is not None and cfg.swarm_penalty


def peer_sphere_enabled(params: dict | None, *, a_max: float = 6.0) -> bool:
    """后段和欧氏碰撞球都打开时为 True。关着时不读半径和权重。"""
    cfg = config_from_params(params, a_max=a_max)
    return cfg is not None and cfg.peer_sphere_enabled


def _env_flag(name: str) -> bool:
    """只有 1/true/yes/on 才算打开。未设置、空串和其它写法都是关，不改默认。"""
    raw = os.environ.get(name)
    if raw is None:
        return False
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str) -> float | None:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return None
    return float(raw)
