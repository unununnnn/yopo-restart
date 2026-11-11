#!/usr/bin/env python3
"""现有 L0 场景的障碍面密度（obs/m²），对照 EGO-Swarm 论文 Table II。

只数几何，不跑规划器。林带区域用 yopo_l0.forest 放置树干时的同一组边界；
柱林只数名字以 p_ 开头的盒子，墙和天花板不计入。

用法（仓库根目录，只要 numpy，不需要 torch）：

  python3 scripts/ego-swarm-density.py
  python3 scripts/ego-swarm-density.py --out docs/research/ego-swarm-density.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from yopo_l0.forest import (  # noqa: E402
    _DENSE_LO,
    _Y_HI,
    _Y_LO,
    TIERS,
    generate_forest,
    woods_bounds,
)

# 论文 Table II 原文数字，只作对照，不是本仓库的测量。
# Zhou et al., arXiv:2011.04183v3 (2021-05-05), Table II.
EGO_TABLE_II = [
    {
        "density_obs_per_m2": 0.42,
        "dfly_m": 49.4,
        "vel_m_s": 1.61,
        "dsafe_m": 0.208,
        "tcal_ms": 5.00,
    },
    {
        "density_obs_per_m2": 0.28,
        "dfly_m": 45.4,
        "vel_m_s": 1.59,
        "dsafe_m": 0.22,
        "tcal_ms": 2.87,
    },
    {
        "density_obs_per_m2": 0.14,
        "dfly_m": 43.7,
        "vel_m_s": 1.57,
        "dsafe_m": 0.266,
        "tcal_ms": 1.21,
    },
    {
        "density_obs_per_m2": 0,
        "dfly_m": 42.2,
        "vel_m_s": 1.55,
        "dsafe_m": None,
        "tcal_ms": 0.49,
    },
]

FOREST_SEED = 7
PILLAR_SCENE = ROOT / "scenes" / "crossing-pillars-scene.json"
PILLAR_CARD = ROOT / "scenes" / "crossing-pillars.json"


def _area(x0: float, x1: float, y0: float, y1: float) -> float:
    return (x1 - x0) * (y1 - y0)


def _band(
    x: float,
    y: float,
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    *,
    left_closed: bool,
    right_closed: bool,
) -> bool:
    """点是否落在矩形里。左右是否含端点分开，避免 x == 分界被两段都数。"""
    if y < y0 or y > y1:
        return False
    if x < x0 or (x == x0 and not left_closed):
        return False
    if x > x1 or (x == x1 and not right_closed):
        return False
    return True


def _box_center(box: dict) -> tuple[float, float]:
    lo, hi = box["lo"], box["hi"]
    return (0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]))


def forest_report() -> dict:
    """三档林、forest.seed=7。密度 = 该带里的树干数 / 该带矩形面积。"""
    woods_lo, woods_hi, y_lo, y_hi = woods_bounds()
    if (woods_lo, woods_hi, y_lo, y_hi) != (_band_bounds()):
        raise RuntimeError("woods_bounds 与脚本使用的 y 边界不一致")
    dense_lo = float(_DENSE_LO)
    regions = {
        "woods": {
            "x": [woods_lo, woods_hi],
            "y": [y_lo, y_hi],
            "area_m2": _area(woods_lo, woods_hi, y_lo, y_hi),
            "rule": "树干底面圆心 x 落在林带内（含两端）",
        },
        "entry": {
            "x": [woods_lo, dense_lo],
            "y": [y_lo, y_hi],
            "area_m2": _area(woods_lo, dense_lo, y_lo, y_hi),
            "rule": "与 generate_forest 的 n_entry 相同：圆心 x < _DENSE_LO",
        },
        "middle": {
            "x": [dense_lo, woods_hi],
            "y": [y_lo, y_hi],
            "area_m2": _area(dense_lo, woods_hi, y_lo, y_hi),
            "rule": "与 generate_forest 的 n_middle 相同：圆心 x >= _DENSE_LO",
        },
    }
    tiers = {}
    for tier in TIERS:
        geom, stats = generate_forest({"density": tier, "seed": FOREST_SEED})
        trunks = geom["cylinders"]
        branches = [b for b in geom["boxes"] if str(b["name"]).endswith("-br")]
        trunk_band: dict[str, str] = {}
        for trunk in trunks:
            x, y = trunk["center_xy"]
            if _band(x, y, dense_lo, woods_hi, y_lo, y_hi, left_closed=True, right_closed=True):
                trunk_band[trunk["name"]] = "middle"
            elif _band(
                x, y, woods_lo, dense_lo, y_lo, y_hi, left_closed=True, right_closed=False
            ):
                trunk_band[trunk["name"]] = "entry"
            else:
                raise RuntimeError(f"{tier} 树干 {trunk['name']} 的圆心不在林带矩形里")

        def count_branches(band: str | None) -> int:
            n = 0
            for box in branches:
                parent = str(box["name"]).rsplit("-br", 1)[0]
                where = trunk_band.get(parent)
                if where is None:
                    raise RuntimeError(f"{tier} 矮枝 {box['name']} 没有对应树干")
                if band is None or where == band:
                    n += 1
            return n

        n_entry = sum(1 for band in trunk_band.values() if band == "entry")
        n_middle = sum(1 for band in trunk_band.values() if band == "middle")
        n_woods = len(trunk_band)
        if n_entry + n_middle != n_woods:
            raise RuntimeError(f"{tier} 入口 {n_entry} + 中段 {n_middle} != 林带 {n_woods}")
        if n_entry != stats["n_entry"] or n_middle != stats["n_middle"]:
            raise RuntimeError(f"{tier} 计数与 generate_forest 统计不一致")
        if n_woods != stats["n_trunks"]:
            raise RuntimeError(f"{tier} 林带树干数与 n_trunks 不一致")
        n_br_woods = count_branches(None)
        n_br_entry = count_branches("entry")
        n_br_middle = count_branches("middle")
        if n_br_entry + n_br_middle != n_br_woods:
            raise RuntimeError(f"{tier} 矮枝分段计数对不上")
        if n_br_woods != stats["n_branches"]:
            raise RuntimeError(f"{tier} 矮枝数与 n_branches 不一致")
        outside = 0
        for box in branches:
            x, y = _box_center(box)
            if not _band(x, y, woods_lo, woods_hi, y_lo, y_hi, left_closed=True, right_closed=True):
                outside += 1

        def row(n_trunk: int, n_branch: int, area: float) -> dict:
            return {
                "n_trunks": n_trunk,
                "n_branches": n_branch,
                "area_m2": area,
                "trunks_per_m2": n_trunk / area,
                "trunks_plus_branches_per_m2": (n_trunk + n_branch) / area,
            }

        tiers[tier] = {
            "surface_gap_spec_m": stats["surface_gap_spec_m"],
            "min_surface_gap_m": stats["min_surface_gap_m"],
            "radius_min_m": stats["radius_min_m"],
            "radius_max_m": stats["radius_max_m"],
            "n_lean": stats["n_lean"],
            "n_branch_centers_outside_woods": outside,
            "woods": row(n_woods, n_br_woods, regions["woods"]["area_m2"]),
            "entry": row(n_entry, n_br_entry, regions["entry"]["area_m2"]),
            "middle": row(n_middle, n_br_middle, regions["middle"]["area_m2"]),
        }
    return {"seed": FOREST_SEED, "regions": regions, "tiers": tiers}


def _band_bounds() -> tuple[float, float, float, float]:
    woods_lo, woods_hi, _, _ = woods_bounds()
    return woods_lo, woods_hi, float(_Y_LO), float(_Y_HI)


def pillar_report() -> dict:
    geom = json.loads(PILLAR_SCENE.read_text())
    card = json.loads(PILLAR_CARD.read_text())
    pillars = [b for b in geom["boxes"] if str(b["name"]).startswith("p_")]
    if not pillars:
        raise RuntimeError("柱林场景里没有 p_ 盒子")
    x0 = min(b["lo"][0] for b in pillars)
    x1 = max(b["hi"][0] for b in pillars)
    y0 = min(b["lo"][1] for b in pillars)
    y1 = max(b["hi"][1] for b in pillars)
    aabb_area = _area(x0, x1, y0, y1)
    center = card["task"]["virtual_center"]
    length = float(center["length_m"])
    origin_x = float(center["origin_enu"][0])
    ys = [float(d["start_enu"][1]) for d in card["drones"]]
    y_lo, y_hi = min(ys), max(ys)
    corridor_area = _area(origin_x, origin_x + length, y_lo, y_hi)
    return {
        "scene": card["name"],
        "geometry": "crossing-pillars-scene.json",
        "n_pillars": len(pillars),
        "aabb": {
            "x": [x0, x1],
            "y": [y0, y1],
            "area_m2": aabb_area,
            "pillars_per_m2": len(pillars) / aabb_area,
            "rule": "全部柱子盒子的轴对齐外包矩形，含柱子自身占的面积",
        },
        "slot_corridor": {
            "x": [origin_x, origin_x + length],
            "y": [y_lo, y_hi],
            "area_m2": corridor_area,
            "pillars_per_m2": len(pillars) / corridor_area,
            "rule": "虚拟中心长度 × 最外两架起点的 y。柱子可以落在这个矩形外面",
        },
    }


def build() -> dict:
    return {
        "what": "L0 现有场景的障碍面密度。没有跑规划，也没有按论文参数新生成地图。",
        "flight_comparison_ran": False,
        "flight_not_run_because": (
            "六机林带 seed 0 的整回路墙钟在 docs/results/12e-upstream-vs-shield.md "
            "里是 234.2 s（medium）和 176.0 s（sparse）。论文 Table II 是 10 架、"
            "对向交换、密度 0.14/0.28/0.42 obs/m²、速度上限 2 m/s、机体半径 0.2 m。"
            "forest.py 只有 sparse/medium/dense 三档，写不了这三个密度；"
            "论文也没有给出地图边长、障碍半径和空域交换的圆半径。"
            "用现有卡再跑一遍不会得到可与 Table II 对齐的数字。"
        ),
        "forest": forest_report(),
        "pillars": pillar_report(),
        "ego_table_ii": {
            "source": "Xin Zhou et al., arXiv:2011.04183v3, Table II. 十架，速度上限 2 m/s，机体半径 0.2 m。",
            "rows": EGO_TABLE_II,
            "dsafe_definition": "the closest distance to obstacles during the flight tests",
            "dsafe_is_center_or_surface": "未知",
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="计算 L0 场景障碍面密度")
    parser.add_argument(
        "--out",
        type=Path,
        default=ROOT / "docs" / "research" / "ego-swarm-density.json",
    )
    args = parser.parse_args()
    report = build()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    forest = report["forest"]
    print(f"wrote {args.out}")
    woods_area = forest["regions"]["woods"]["area_m2"]
    middle_area = forest["regions"]["middle"]["area_m2"]
    print(f"woods area_m2={woods_area} middle={middle_area}")
    for tier, row in forest["tiers"].items():
        mid = row["middle"]
        woods = row["woods"]
        print(
            f"{tier}: trunks woods {woods['n_trunks']} ({woods['trunks_per_m2']:.4f}/m2)"
            f" middle {mid['n_trunks']} ({mid['trunks_per_m2']:.4f}/m2)"
            f" gap_spec {row['surface_gap_spec_m']} min_gap {row['min_surface_gap_m']}"
        )
    pillars = report["pillars"]
    print(
        f"pillars n={pillars['n_pillars']}"
        f" aabb {pillars['aabb']['pillars_per_m2']:.4f}/m2"
        f" corridor {pillars['slot_corridor']['pillars_per_m2']:.4f}/m2"
    )


if __name__ == "__main__":
    main()
