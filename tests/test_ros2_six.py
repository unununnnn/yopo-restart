"""票 11a：六机开阔区的 ROS 2 形态和进程内形态，同 seed 结论一致。

rclpy 导入不了时跳过，原因里写 scripts/setup_ros2.sh。
"""

from __future__ import annotations

import json

from yopo_l0.batch import main as batch_main
from yopo_l0.formation import FormationFleet
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card

from conftest import ROOT
from test_ros2_form import needs_ros

CARD = ROOT / "scenes/formation-6-open.json"


def _view(result: dict) -> dict:
    return {
        "outcome": result["outcome"],
        "reason": result["reason"],
        "arrived": result["arrived"],
        "arrival_rate": result.get("arrival_rate"),
        "collision": result["outcome"] == "COLLISION",
        "formation_error_p95_m": result.get("formation_error_p95_m"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "shadow_reject_rate": result.get("shadow_reject_rate"),
    }


@needs_ros
def test_ros2_six_matches_inprocess_for_seed0():
    from yopo_l0.ros2_run import execute

    card = load_card(CARD)
    cfg = RunConfig(seed=0, policy="formation", duration_s=1.0)
    proc = run(card, cfg)
    info: dict = {}
    ros = execute(card, cfg, inspect=info)
    assert info["clock_ok"], info["clock_error"]
    assert info["nodes"] == ["/l0_world", *[f"/uav{i}/drone" for i in range(6)]]
    assert proc.result["n_drones"] == ros.result["n_drones"] == 6
    assert _view(proc.result) == _view(ros.result)
    assert proc.ticks == ros.ticks
    assert ros.result["form"] == "ros2"
    assert info["bus_ros_deliveries"] == ros.result["bus"]["delivered"] == proc.result["bus"]["delivered"]
    assert FormationFleet.transport_override is None


@needs_ros
def test_ros2_batch_tier_none_prints_one_table(tmp_path):
    out = tmp_path / "ros2"
    rc = batch_main([
        "scenes/formation-6-open.json",
        "--form", "ros2",
        "--tiers", "none",
        "--seeds", "0",
        "--duration", "0.4",
        "--no-determinism",
        "--no-shadow",
        "--out", str(out),
    ])
    assert rc == 0
    md = (out / "summary.md").read_text()
    assert "/ none（" in md
    assert "形态 ROS 2" in md
    assert "单次墙钟" in md
    saved = json.loads((out / "results.json").read_text())
    assert saved[0]["form"] == "ros2"
    assert saved[0]["fault_tier"] == "none"
    assert saved[0]["n_drones"] == 6
    assert FormationFleet.transport_override is None
