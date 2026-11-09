"""票 11b：六机穿林的 ROS 2 形态和进程内形态，同 seed 的短前缀一致。

rclpy 导入不了时跳过。全长对账在批跑里做，不放进 regress。
"""

from __future__ import annotations

from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card

from conftest import ROOT
from test_ros2_form import needs_ros

MEDIUM = ROOT / "scenes/crossing-6-forest.json"


def _view(result: dict) -> dict:
    return {
        "outcome": result["outcome"],
        "arrived": result["arrived"],
        "arrival_rate": result.get("arrival_rate"),
        "collision": result["outcome"] == "COLLISION",
        "formation_error_p95_m": result.get("formation_error_p95_m"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "min_obstacle_distance_m": result.get("min_obstacle_distance_m"),
        "forest_density": result.get("forest_density"),
        "n_drones": result["n_drones"],
    }


@needs_ros
def test_ros2_medium_prefix_matches_inprocess():
    from yopo_l0.ros2_run import execute

    card = load_card(MEDIUM)
    cfg = RunConfig(seed=0, policy="crossing", duration_s=0.4, shadow=False)
    proc = run(card, cfg)
    ros = execute(card, cfg)
    assert proc.result["n_drones"] == 6
    assert _view(proc.result) == _view(ros.result)
    assert proc.ticks == ros.ticks
    assert ros.result["form"] == "ros2"
    assert ros.result["policy"] == "crossing"
