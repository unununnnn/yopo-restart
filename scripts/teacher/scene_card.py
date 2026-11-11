"""把同一份圆柱几何写成 L0 能 ``load_card`` 的场景卡。

任务用 goto：每架有自己的 ``goal_enu``。平行是各飞各的 y，对向是把 y 的顺序对调。
不写入 ``scenes/suite.txt``，``criteria.frozen`` 为 false。
"""

from __future__ import annotations

import json
from pathlib import Path

from .assumptions import BODY_RADIUS_M, GOAL_HOLD_S, GOAL_RADIUS_M, GOAL_SPEED_MPS

_REPO = Path(__file__).resolve().parents[2]


def l0_camera_dynamics() -> tuple[dict, dict, dict]:
    template = json.loads((_REPO / "scenes" / "open-3drone.json").read_text(encoding="utf-8"))
    camera = dict(template["camera"])
    # 噪声为 0：同一姿态的光线投射只由几何决定。open-3drone 卡是 0.02。
    camera["noise_std_m"] = 0.0
    return camera, dict(template["dynamics"]), dict(template["rates_hz"])


def write_scene_card(scenario: dict, out_dir: Path) -> Path:
    """写出 ``geometry.json`` 和 ``scene.json``。返回场景卡路径。"""
    out_dir.mkdir(parents=True, exist_ok=True)
    geom = {
        "boxes": [],
        "cylinders": [
            {
                "name": c["name"],
                "center_xy": [float(c["center_xy"][0]), float(c["center_xy"][1])],
                "radius": float(c["radius"]),
                "z_lo": float(c["z_lo"]),
                "z_hi": float(c["z_hi"]),
            }
            for c in scenario["cylinders"]
        ],
    }
    (out_dir / "geometry.json").write_text(json.dumps(geom, indent=2) + "\n", encoding="utf-8")
    camera, dynamics, rates = l0_camera_dynamics()
    card = {
        "name": scenario["id"],
        "geometry": "geometry.json",
        "ground_z": 0.0,
        "drones": [
            {
                "id": d["id"],
                "start_enu": [float(x) for x in d["start_enu"]],
                "goal_enu": [float(x) for x in d["goal_enu"]],
            }
            for d in scenario["drones"]
        ],
        "task": {
            "type": "goto",
            "duration_s": 40.0,
        },
        "criteria": {
            "body_radius_m": BODY_RADIUS_M,
            "goal_tolerance_m": GOAL_RADIUS_M,
            "goal_speed_mps": GOAL_SPEED_MPS,
            "goal_hold_s": GOAL_HOLD_S,
            "frozen": False,
        },
        "camera": camera,
        "dynamics": dynamics,
        "rates_hz": rates,
        "seeds": [int(scenario["seed"])],
        "teacher": {
            "role": "ego-swarm-teacher-scene",
            "assumption": True,
            "density_per_m2": scenario["density_per_m2"],
            "actual_density_per_m2": scenario["actual_density_per_m2"],
            "map_side_m": scenario["map_side_m"],
            "obstacle_radius_m": scenario["obstacle_radius_m"],
            "pattern": scenario["pattern"],
            "body_radius_note": "论文表 II 写 0.2 m。这次的规划器二进制没有打出机体半径，卡上这个数是假设。",
            "noise_std_note": "深度噪声设为 0。scenes/open-3drone.json 是 0.02。",
            "ground_note": "教师点云不含地面。ground_z 只给 L0 的碰撞真值。",
        },
    }
    path = out_dir / "scene.json"
    path.write_text(json.dumps(card, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path
