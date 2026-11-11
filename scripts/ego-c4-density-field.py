#!/usr/bin/env python3
"""票 E-C4：按论文表 II 的目标密度写三份几何文件。

论文表 II（arXiv:2011.04183v3）没有给出障碍半径和地图边长。
本脚本选定的两个数与 D2 教师管线相同（PR #49 的 scripts/teacher/assumptions.py，
写这份文件时尚未合入 main，所以这里不导入那个模块）：

- 障碍半径 0.20 m
- 正方形边长 20.0 m，面积 400 m²

这是假设，不是论文复现。

密度 = 圆柱个数 / 边长²。0.14、0.28、0.42 个/m² 用整数个数 56、112、168，
避免 0.14 * 400 的二进制误差。圆柱高度 0.0–2.5 m 也沿用那条管线，论文同样没给。

不改 yopo_l0.forest 的 sparse / medium / dense 三档。不跑规划器。

用法（仓库根目录，只要 Python 标准库）：

  python3 scripts/ego-c4-density-field.py
  python3 scripts/ego-c4-density-field.py --check
"""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / "scenes"

# 假设，不是论文复现。与 D2 教师管线选定的数相同。
OBSTACLE_RADIUS_M = 0.20
MAP_SIDE_M = 20.0
OBSTACLE_Z_LO_M = 0.0
OBSTACLE_Z_HI_M = 2.5
DISCLAIMER = "这是假设，不是论文复现"

# 与边长 20 m 相乘后是整数。不用 density * area 的浮点乘法来取个数。
COUNTS = (
    (0.14, 56, "ego-density-014.json"),
    (0.28, 112, "ego-density-028.json"),
    (0.42, 168, "ego-density-042.json"),
)


def place_centers(n: int, seed_text: str) -> tuple[list[tuple[float, float]], int]:
    """圆心落在内正方形里，彼此不重叠。圆柱整体留在边长 20 m 的场内。"""
    radius = OBSTACLE_RADIUS_M
    lo = -MAP_SIDE_M / 2.0 + radius
    hi = MAP_SIDE_M / 2.0 - radius
    min_sep = 2.0 * radius
    rng = random.Random(seed_text)
    centers: list[tuple[float, float]] = []
    attempts = 0
    max_attempts = max(1000, n * 400)
    while len(centers) < n and attempts < max_attempts:
        attempts += 1
        x = rng.uniform(lo, hi)
        y = rng.uniform(lo, hi)
        if any((x - cx) ** 2 + (y - cy) ** 2 < min_sep ** 2 for cx, cy in centers):
            continue
        centers.append((x, y))
    if len(centers) != n:
        raise RuntimeError(f"种子 {seed_text} 只放下 {len(centers)}/{n} 根圆柱")
    return centers, attempts


def build(target_density: float, n: int) -> dict:
    area = MAP_SIDE_M * MAP_SIDE_M
    if n / area != target_density:
        raise RuntimeError(f"{n} / {area} = {n / area}，对不上目标 {target_density}")
    seed_text = f"e-c4-density-seed0-n{n}"
    centers, attempts = place_centers(n, seed_text)
    cylinders = [
        {
            "name": f"obs{i}",
            "center_xy": [x, y],
            "radius": OBSTACLE_RADIUS_M,
            "z_lo": OBSTACLE_Z_LO_M,
            "z_hi": OBSTACLE_Z_HI_M,
        }
        for i, (x, y) in enumerate(centers)
    ]
    tag = f"{target_density:.2f}".replace(".", "")
    return {
        "note": (
            f"{DISCLAIMER}。障碍半径 {OBSTACLE_RADIUS_M:.2f} m，地图边长 {MAP_SIDE_M:.1f} m。"
            "论文表 II 没有给出这两个数。高度 0.0–2.5 m 同样不是论文里的数。"
        ),
        "header": {
            "ticket": "E-C4",
            "disclaimer": DISCLAIMER,
            "paper": "EGO-Swarm 论文表 II（arXiv:2011.04183v3）没有给出障碍半径和地图边长。",
            "obstacle_radius_m": OBSTACLE_RADIUS_M,
            "map_side_m": MAP_SIDE_M,
            "map_area_m2": area,
            "obstacle_z_lo_m": OBSTACLE_Z_LO_M,
            "obstacle_z_hi_m": OBSTACLE_Z_HI_M,
            "target_density_per_m2": target_density,
            "n_obstacles": n,
            "realized_density_per_m2": n / area,
            "density_rule": "圆柱个数 / 边长²。边长 20.0 m，面积 400 m²。",
            "assumption_source": (
                "与 D2 教师管线选定的假设相同：障碍半径 0.20 m，正方形边长 20.0 m，"
                "圆柱高度 0.0–2.5 m。写本文件时 main 上还没有 scripts/teacher/assumptions.py"
                "（PR #49 未合入），数字写在本头里，不导入那个模块。"
            ),
            "placement": (
                "Python random.Random，种子字符串 e-c4-density-seed0-n<个数>。"
                "圆心落在边长 19.6 m 的内正方形（四周各让出半径 0.20 m）。"
                "圆心间距不小于 0.40 m。不给飞行起终点留空。不跑规划器。"
            ),
            "placement_seed": seed_text,
            "placement_attempts": attempts,
            "coordinate_frame": "正方形中心在原点，x、y 各覆盖 [-10, 10] m。",
        },
        "world": f"ego-density-{tag}",
        "boxes": [],
        "cylinders": cylinders,
    }


def write_all() -> list[Path]:
    paths = []
    for density, n, name in COUNTS:
        path = SCENES / name
        path.write_text(json.dumps(build(density, n), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        paths.append(path)
    return paths


def check_all() -> None:
    for density, n, name in COUNTS:
        path = SCENES / name
        got = json.loads(path.read_text(encoding="utf-8"))
        expected = build(density, n)
        if got != expected:
            raise SystemExit(f"{path} 与生成器不一致")
        header = got["header"]
        if header["disclaimer"] != DISCLAIMER:
            raise SystemExit(f"{path} 缺少「{DISCLAIMER}」")
        realized = header["n_obstacles"] / header["map_area_m2"]
        if realized != header["target_density_per_m2"] or header["n_obstacles"] != n:
            raise SystemExit(f"{path} 实测密度对不上")
        print(
            f"{name}: target {header['target_density_per_m2']} "
            f"realized {header['realized_density_per_m2']} "
            f"n {header['n_obstacles']} area {header['map_area_m2']} "
            f"radius {header['obstacle_radius_m']} side {header['map_side_m']}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="写 E-C4 密度场几何")
    parser.add_argument("--check", action="store_true", help="只核对已写入的文件，不覆盖")
    args = parser.parse_args()
    if args.check:
        check_all()
        return
    for path in write_all():
        print(f"wrote {path.relative_to(ROOT)}")
    check_all()


if __name__ == "__main__":
    main()
