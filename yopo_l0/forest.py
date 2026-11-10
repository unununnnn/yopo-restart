"""票 07b：泊松盘树干林。几何只由种子和密度档决定。

沿走廊的净距是变的：入口疏、中段密、密林之后是空地。
密度档只改中段树干表面的最小净距，不改规划参数。

票 13c：六机穿林会读这里生成的树干做弹性槽位和提交前净空门。生成规则不变。
三机和柱林的规划器仍不读这份几何。

- ``dense`` 的中段最小净距落在 1.5–2.0 m（树干表面，不是圆心）。
- 树干半径从三角分布里抽，约 0.10–0.40 m。
- 一部分树干倾斜，一部分在飞行高度伸出矮枝。倾斜或枝干若会破坏最小净距，就丢掉这一笔。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TIERS = ("sparse", "medium", "dense")

# 中段树干表面最小净距。dense 落在 frank 说的 1.5–2.0 m。
_SURFACE_GAP = {"sparse": 3.6, "medium": 2.4, "dense": 1.7}
_RADIUS = (0.10, 0.22, 0.40)  # 三角分布：左端、众数、右端
# 林子在走廊上的位置。入口净距更大，x > WOODS_HI 是空地。
_WOODS_LO = 8.0
_DENSE_LO = 16.0
_WOODS_HI = 30.0
_Y_LO = -8.5
_Y_HI = 8.5
_ENTRY_GAP_SCALE = 2.15


@dataclass(frozen=True)
class _Trunk:
    x: float
    y: float
    radius: float
    height: float
    gap: float
    lean_xy: tuple[float, float]
    branch: tuple[float, float, float, float, float] | None  # 中心 x, y, z，半长 x，半长 y


def _gap_at(x: float, tier: str) -> float | None:
    """这个 x 上新树干要求的表面净距。空地和林子外面返回 None。"""
    if x < _WOODS_LO or x > _WOODS_HI:
        return None
    mid = _SURFACE_GAP[tier]
    if x >= _DENSE_LO:
        return mid
    u = (x - _WOODS_LO) / (_DENSE_LO - _WOODS_LO)
    smooth = u * u * (3.0 - 2.0 * u)
    return mid * _ENTRY_GAP_SCALE + (mid - mid * _ENTRY_GAP_SCALE) * smooth


def _sample_radius(rng: np.random.Generator) -> float:
    return float(rng.triangular(*_RADIUS))


def _segment_distance(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray) -> float:
    """两条线段的最短距离。来自 Real-Time Collision Detection 的 clamp 写法。"""
    u = a1 - a0
    v = b1 - b0
    w = a0 - b0
    a = float(np.dot(u, u))
    b = float(np.dot(u, v))
    c = float(np.dot(v, v))
    d = float(np.dot(u, w))
    e = float(np.dot(v, w))
    denom = a * c - b * b
    s_num, s_den = denom, denom
    t_num, t_den = denom, denom
    if denom < 1e-12:
        s_num = 0.0
        s_den = 1.0
        t_num = e
        t_den = c
    else:
        s_num = b * e - c * d
        t_num = a * e - b * d
        if s_num < 0.0:
            s_num = 0.0
            t_num = e
            t_den = c
        elif s_num > s_den:
            s_num = s_den
            t_num = e + b
            t_den = c
    if t_num < 0.0:
        t_num = 0.0
        if -d < 0.0:
            s_num = 0.0
        elif -d > a:
            s_num = s_den
        else:
            s_num = -d
            s_den = a
    elif t_num > t_den:
        t_num = t_den
        if -d + b < 0.0:
            s_num = 0.0
        elif -d + b > a:
            s_num = s_den
        else:
            s_num = -d + b
            s_den = a
    sc = 0.0 if abs(s_num) < 1e-12 else s_num / s_den
    tc = 0.0 if abs(t_num) < 1e-12 else t_num / t_den
    return float(np.linalg.norm(w + sc * u - tc * v))


def _axis(trunk: _Trunk) -> tuple[np.ndarray, np.ndarray]:
    base = np.array([trunk.x, trunk.y, 0.0])
    top = np.array([trunk.x + trunk.lean_xy[0], trunk.y + trunk.lean_xy[1], trunk.height])
    return base, top


def _surface_gap(a: _Trunk, b: _Trunk) -> float:
    a0, a1 = _axis(a)
    b0, b1 = _axis(b)
    return _segment_distance(a0, a1, b0, b1) - a.radius - b.radius


def _fits(x: float, y: float, radius: float, gap: float, trunks: list[_Trunk]) -> bool:
    for other in trunks:
        need = other.radius + radius + max(gap, other.gap)
        if math.hypot(x - other.x, y - other.y) < need:
            return False
    return True


def _poisson(rng: np.random.Generator, tier: str) -> list[_Trunk]:
    """Bridson 圆环候选。净距随 x 变，树干半径先抽再检查表面净距。"""
    trunks: list[_Trunk] = []
    for _ in range(64):
        x = float(rng.uniform(_DENSE_LO, _WOODS_HI))
        y = float(rng.uniform(_Y_LO, _Y_HI))
        gap = _gap_at(x, tier)
        radius = _sample_radius(rng)
        if gap is not None and _fits(x, y, radius, gap, trunks):
            trunks.append(_Trunk(x, y, radius, 0.0, gap, (0.0, 0.0), None))
            break
    if not trunks:
        raise RuntimeError(f"{tier} 林一个初始点都放不下")
    active = [0]
    while active:
        pick = int(rng.integers(0, len(active)))
        parent = trunks[active[pick]]
        placed = False
        for _ in range(30):
            ang = float(rng.uniform(0.0, 2.0 * math.pi))
            step = parent.radius + 0.40 + parent.gap
            dist = float(rng.uniform(step, 2.0 * step))
            x = parent.x + dist * math.cos(ang)
            y = parent.y + dist * math.sin(ang)
            if not (_Y_LO <= y <= _Y_HI):
                continue
            gap = _gap_at(x, tier)
            if gap is None:
                continue
            radius = _sample_radius(rng)
            if _fits(x, y, radius, gap, trunks):
                trunks.append(_Trunk(x, y, radius, 0.0, gap, (0.0, 0.0), None))
                active.append(len(trunks) - 1)
                placed = True
                break
        if not placed:
            active.pop(pick)
    return trunks


def _with_lean(trunk: _Trunk, lean: tuple[float, float]) -> _Trunk:
    return _Trunk(trunk.x, trunk.y, trunk.radius, trunk.height, trunk.gap, lean, trunk.branch)


def _repair_leans(trunks: list[_Trunk]) -> list[_Trunk]:
    """倾斜是后加的。哪一笔让表面净距小于两棵树里较严的那条，就把那笔倾斜去掉。"""
    leans = [trunk.lean_xy for trunk in trunks]
    for _ in range(2):
        for i, trunk in enumerate(trunks):
            if leans[i] == (0.0, 0.0):
                continue
            trial = _with_lean(trunk, leans[i])
            for j, other in enumerate(trunks):
                if i == j:
                    continue
                gap = _surface_gap(trial, _with_lean(other, leans[j]))
                if gap < min(trunk.gap, other.gap) - 1e-6:
                    leans[i] = (0.0, 0.0)
                    break
    return [_with_lean(trunk, lean) for trunk, lean in zip(trunks, leans, strict=True)]


def _decorate(rng: np.random.Generator, trunks: list[_Trunk]) -> list[_Trunk]:
    """高度、倾斜、矮枝。破坏表面净距的倾斜和枝丢掉。"""
    drafted: list[_Trunk] = []
    branches: list[tuple[float, float, float, float] | None] = []
    for trunk in trunks:
        height = float(rng.uniform(3.4, 6.2))
        lean = (0.0, 0.0)
        if float(rng.random()) < 0.35:
            reach = float(rng.uniform(0.35, 1.15))
            ang = float(rng.uniform(0.0, 2.0 * math.pi))
            lean = (reach * math.cos(ang), reach * math.sin(ang))
        branch = None
        if float(rng.random()) < 0.22:
            ang = float(rng.choice([0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi]))
            length = float(rng.uniform(0.7, 1.2))
            z = float(rng.uniform(1.0, 1.7))
            along_x = ang in (0.0, math.pi)
            sign = -1.0 if ang in (math.pi, 1.5 * math.pi) else 1.0
            thick = 0.16
            reach = trunk.radius + 0.5 * length
            if along_x:
                branch = (trunk.x + sign * reach, trunk.y, z, 0.5 * length, 0.5 * thick)
            else:
                branch = (trunk.x, trunk.y + sign * reach, z, 0.5 * thick, 0.5 * length)
        drafted.append(_Trunk(trunk.x, trunk.y, trunk.radius, height, trunk.gap, lean, None))
        branches.append(branch)
    repaired = _repair_leans(drafted)
    out: list[_Trunk] = []
    for trunk, branch in zip(repaired, branches, strict=True):
        if branch is not None and _branch_hits(branch, [t for t in repaired if t is not trunk]):
            branch = None
        out.append(_Trunk(trunk.x, trunk.y, trunk.radius, trunk.height, trunk.gap, trunk.lean_xy, branch))
    return out


def _branch_hits(branch: tuple[float, float, float, float, float], others: list[_Trunk]) -> bool:
    """矮枝的盒子离别的树干表面要留下 0.3 m，免得两棵树粘在一起。"""
    cx, cy, _z, half_x, half_y = branch
    for other in others:
        dx = max(abs(cx - other.x) - half_x, 0.0)
        dy = max(abs(cy - other.y) - half_y, 0.0)
        if math.hypot(dx, dy) < other.radius + 0.3:
            return True
    return False


def _walls() -> list[dict]:
    return [
        {"name": "ceiling", "lo": [-4.0, -16.2, 8.0], "hi": [56.0, 16.2, 8.2]},
        {"name": "left_wall", "lo": [-4.0, 16.0, 0.0], "hi": [56.0, 16.2, 8.0]},
        {"name": "right_wall", "lo": [-4.0, -16.2, 0.0], "hi": [56.0, -16.0, 8.0]},
        {"name": "end_wall", "lo": [54.0, -16.2, 0.0], "hi": [54.2, 16.2, 8.0]},
        {"name": "back_wall", "lo": [-3.2, -16.2, 0.0], "hi": [-3.0, 16.2, 8.0]},
    ]


def _branch_box(name: str, branch: tuple[float, float, float, float, float]) -> dict:
    cx, cy, z, half_x, half_y = branch
    thick_z = 0.16
    return {
        "name": name,
        "lo": [cx - half_x, cy - half_y, z - 0.5 * thick_z],
        "hi": [cx + half_x, cy + half_y, z + 0.5 * thick_z],
    }


def generate_forest(spec: dict) -> tuple[dict, dict]:
    """返回 (几何 dict, 统计)。同一 spec 两次调用逐位相同。"""
    tier = str(spec.get("density", "medium"))
    if tier not in _SURFACE_GAP:
        raise ValueError(f"density 只能是 {TIERS}，收到 {tier}")
    seed = int(spec.get("seed", 7))
    rng = np.random.default_rng(np.random.SeedSequence([seed, {"sparse": 1, "medium": 2, "dense": 3}[tier]]))
    trunks = _decorate(rng, _poisson(rng, tier))
    cylinders = []
    boxes = _walls()
    n_lean = n_branch = 0
    for i, trunk in enumerate(trunks):
        cylinders.append(
            {
                "name": f"t{i:03d}",
                "center_xy": [trunk.x, trunk.y],
                "radius": trunk.radius,
                "z_lo": 0.0,
                "z_hi": trunk.height,
                "lean_xy": [trunk.lean_xy[0], trunk.lean_xy[1]],
            }
        )
        if trunk.lean_xy != (0.0, 0.0):
            n_lean += 1
        if trunk.branch is not None:
            boxes.append(_branch_box(f"t{i:03d}-br", trunk.branch))
            n_branch += 1
    gaps = [_surface_gap(a, b) for i, a in enumerate(trunks) for b in trunks[i + 1 :]]
    entry = sum(1 for t in trunks if t.x < _DENSE_LO)
    middle = sum(1 for t in trunks if t.x >= _DENSE_LO)
    radii = [t.radius for t in trunks]
    geom = {"world": "crossing-forest", "density": tier, "seed": seed, "boxes": boxes, "cylinders": cylinders}
    stats = {
        "density": tier,
        "seed": seed,
        "n_trunks": len(trunks),
        "n_branches": n_branch,
        "n_lean": n_lean,
        "n_obstacles": len(cylinders) + n_branch,
        "n_entry": entry,
        "n_middle": middle,
        "min_surface_gap_m": None if not gaps else float(min(gaps)),
        "radius_min_m": None if not radii else float(min(radii)),
        "radius_max_m": None if not radii else float(max(radii)),
        "surface_gap_spec_m": _SURFACE_GAP[tier],
    }
    return geom, stats


def woods_x_interval() -> tuple[float, float]:
    """林带在走廊 x 上的范围。入口在这之前，空地在这之后。"""
    return _WOODS_LO, _WOODS_HI


def generate_forest_geometry(spec: dict) -> dict:
    return generate_forest(spec)[0]


def woods_x_interval() -> tuple[float, float]:
    """林带在走廊 x 上的范围。入口在这之前，空地在这之后。"""
    return _WOODS_LO, _WOODS_HI


def woods_bounds() -> tuple[float, float, float, float]:
    """林带 x 范围和走廊 y 范围。弹性槽位的侧向松弛剪在这个 y 区间里。"""
    return _WOODS_LO, _WOODS_HI, _Y_LO, _Y_HI


def card_for_tier(card: dict, tier: str) -> dict:
    """复制场景卡并拨密度档。名字带上档位，结果表才分得开。"""
    if tier not in TIERS:
        raise ValueError(f"density 只能是 {TIERS}，收到 {tier}")
    out = {
        **card,
        "name": f"{card['name']}-{tier}" if not str(card["name"]).endswith(f"-{tier}") else card["name"],
        "forest": {**card["forest"], "density": tier},
    }
    return out
