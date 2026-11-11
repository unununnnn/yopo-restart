"""批量场景。障碍半径和地图边长的假设写在 assumptions.py 的文件头。

单机没有第二架可以交换：对向和平行的起终点相同，障碍仍按图案编号分开抽。
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .assumptions import (
    BODY_RADIUS_M,
    DENSITIES_PER_M2,
    DRONE_COUNTS,
    FLIGHT_Z_M,
    GOAL_X_M,
    MAP_SIDE_M,
    OBSTACLE_RADIUS_M,
    OBSTACLE_Z_HI_M,
    OBSTACLE_Z_LO_M,
    PATTERNS,
    SEEDS,
    SPAWN_CLEARANCE_EXTRA_M,
    START_X_M,
    lane_ys,
    obstacle_count,
)


def scenario_id(density: float, n_drones: int, pattern: str, seed: int) -> str:
    tag = {0.0: "d000", 0.14: "d014", 0.28: "d028", 0.42: "d042"}[float(density)]
    return f"{tag}-n{n_drones}-{pattern}-s{seed}"


def iter_scenarios():
    for density in DENSITIES_PER_M2:
        for n_drones in DRONE_COUNTS:
            for pattern in PATTERNS:
                for seed in SEEDS:
                    yield make_scenario(density, n_drones, pattern, seed)


def make_scenario(density: float, n_drones: int, pattern: str, seed: int) -> dict:
    if pattern not in PATTERNS:
        raise ValueError(f"图案只支持 parallel 与 headon，收到 {pattern}")
    density = float(density)
    ys = lane_ys(n_drones)
    goal_ys = list(reversed(ys)) if pattern == "headon" else list(ys)
    drones = []
    for i, (y0, y1) in enumerate(zip(ys, goal_ys)):
        drones.append(
            {
                "ros_id": i,
                "id": f"uav{i}",
                "start_enu": [START_X_M, float(y0), FLIGHT_Z_M],
                "goal_enu": [GOAL_X_M, float(y1), FLIGHT_Z_M],
            }
        )
    anchors = [d["start_enu"] for d in drones] + [d["goal_enu"] for d in drones]
    cylinders, attempts, complete = place_cylinders(
        obstacle_count(density),
        anchors,
        seed_key=(obstacle_count(density), n_drones, 0 if pattern == "parallel" else 1, int(seed)),
    )
    n = len(cylinders)
    area = MAP_SIDE_M * MAP_SIDE_M
    return {
        "id": scenario_id(density, n_drones, pattern, seed),
        "density_per_m2": density,
        "n_drones": n_drones,
        "pattern": pattern,
        "seed": int(seed),
        "map_side_m": MAP_SIDE_M,
        "obstacle_radius_m": OBSTACLE_RADIUS_M,
        "body_radius_m": BODY_RADIUS_M,
        "flight_z_m": FLIGHT_Z_M,
        "drones": drones,
        "cylinders": cylinders,
        "n_obstacles": n,
        "actual_density_per_m2": n / area,
        "placement_attempts": attempts,
        "placement_complete": complete,
        "assumption": {
            "map_side_m": MAP_SIDE_M,
            "obstacle_radius_m": OBSTACLE_RADIUS_M,
            "text": "地图边长和障碍半径是假设，不是论文复现。见 assumptions.py 文件头。",
        },
    }


def place_cylinders(n: int, anchors_xyz: list[list[float]], seed_key: tuple[int, int, int, int]):
    """在正方形里放 n 个互不重叠的圆柱，并躲开起点和终点。"""
    radius = OBSTACLE_RADIUS_M
    guard = radius + BODY_RADIUS_M + SPAWN_CLEARANCE_EXTRA_M
    min_sep = 2.0 * radius
    lo = -MAP_SIDE_M / 2.0 + radius
    hi = MAP_SIDE_M / 2.0 - radius
    rng = np.random.default_rng(np.random.SeedSequence(list(seed_key)))
    centers: list[tuple[float, float]] = []
    attempts = 0
    max_attempts = max(1000, n * 400)
    while len(centers) < n and attempts < max_attempts:
        attempts += 1
        x = float(rng.uniform(lo, hi))
        y = float(rng.uniform(lo, hi))
        if any((x - cx) ** 2 + (y - cy) ** 2 < min_sep ** 2 for cx, cy in centers):
            continue
        if any((x - p[0]) ** 2 + (y - p[1]) ** 2 < guard ** 2 for p in anchors_xyz):
            continue
        centers.append((x, y))
    cylinders = []
    for i, (x, y) in enumerate(centers):
        cylinders.append(
            {
                "name": f"obs{i}",
                "center_xy": [x, y],
                "radius": radius,
                "z_lo": OBSTACLE_Z_LO_M,
                "z_hi": OBSTACLE_Z_HI_M,
            }
        )
    return cylinders, attempts, len(cylinders) == n


def write_scenarios(path: Path, scenarios: list[dict] | None = None) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(iter_scenarios()) if scenarios is None else scenarios
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
    return len(rows)


def read_scenarios(path: Path) -> list[dict]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            rows.append(json.loads(line))
    return rows
