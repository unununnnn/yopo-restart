"""票 D2：教师场景能落到 L0 场景卡，观测接口能读出深度、状态和邻机。"""

from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import numpy as np
import pytest

from teacher.assumptions import DENSITY_COUNTS, MAP_SIDE_M, OBSTACLE_RADIUS_M
from teacher.dataset import check_record, observation_at, write_dataset
from teacher.launch_xml import launch_xml
from teacher.metrics import evaluate_trajectory, parse_planner_log, success_of
from teacher.scenarios import iter_scenarios, make_scenario
from teacher.scene_card import write_scene_card
from yopo_l0.scene import load_card

from conftest import ROOT


def test_density_counts_match_the_assumed_square():
    assert MAP_SIDE_M == 20.0
    assert OBSTACLE_RADIUS_M == 0.20
    assert DENSITY_COUNTS[0.0] == 0
    assert DENSITY_COUNTS[0.14] == 56
    assert DENSITY_COUNTS[0.28] == 112
    assert DENSITY_COUNTS[0.42] == 168
    assert 56 / 400 == pytest.approx(0.14)
    assert 168 / 400 == pytest.approx(0.42)


def test_grid_is_192_and_headon_swaps_lanes():
    rows = list(iter_scenarios())
    assert len(rows) == 192
    assert len({r["id"] for r in rows}) == 192
    parallel = make_scenario(0.14, 3, "parallel", 0)
    headon = make_scenario(0.14, 3, "headon", 0)
    assert [d["goal_enu"][1] for d in parallel["drones"]] == [d["start_enu"][1] for d in parallel["drones"]]
    assert [d["goal_enu"][1] for d in headon["drones"]] == list(reversed([d["start_enu"][1] for d in headon["drones"]]))
    alone_p = make_scenario(0.0, 1, "parallel", 1)
    alone_h = make_scenario(0.0, 1, "headon", 1)
    assert alone_p["drones"][0]["goal_enu"] == alone_h["drones"][0]["goal_enu"]
    assert alone_p["cylinders"] == []
    dense = make_scenario(0.42, 6, "headon", 3)
    assert dense["placement_complete"] is True
    assert dense["n_obstacles"] == 168
    guard = OBSTACLE_RADIUS_M + 0.20 + 0.30
    for cyl in dense["cylinders"]:
        cx, cy = cyl["center_xy"]
        for drone in dense["drones"]:
            for point in (drone["start_enu"], drone["goal_enu"]):
                assert math.hypot(cx - point[0], cy - point[1]) >= guard - 1e-9


def test_scene_card_loads_and_is_not_a_frozen_gate(tmp_path):
    scenario = make_scenario(0.14, 3, "parallel", 0)
    path = write_scene_card(scenario, tmp_path)
    card = load_card(path)
    assert card.name == scenario["id"]
    assert len(card.obstacles) == 56 + 1  # 圆柱 + 地面
    assert card.card["criteria"]["frozen"] is False
    assert card.card["task"]["type"] == "goto"
    suite = (ROOT / "scenes" / "suite.txt").read_text(encoding="utf-8")
    assert scenario["id"] not in suite
    assert "teacher" not in suite
    start = np.asarray(scenario["drones"][0]["start_enu"], dtype=float)
    assert card.min_distance(start) > 0.20


def test_same_scenario_is_deterministic():
    a = make_scenario(0.28, 6, "headon", 4)
    b = make_scenario(0.28, 6, "headon", 4)
    assert a["cylinders"] == b["cylinders"]
    c = make_scenario(0.28, 6, "parallel", 4)
    assert a["cylinders"] != c["cylinders"]


def test_launch_lists_each_drone():
    scenario = make_scenario(0.0, 6, "parallel", 0)
    root = ET.fromstring(launch_xml(scenario))
    includes = [node for node in root.iter("include")]
    assert len(includes) == 6
    assert "rviz" not in launch_xml(scenario)
    assert "random_forest" not in launch_xml(scenario)


def test_metrics_count_a_hit_and_a_miss():
    geom = {
        "boxes": [],
        "cylinders": [
            {"name": "obs0", "center_xy": [0.0, 0.0], "radius": 0.2, "z_lo": 0.0, "z_hi": 2.5}
        ],
    }
    time_s = np.array([0.0, 0.5, 1.0])
    clear = np.zeros((3, 1, 3))
    clear[:, 0, 0] = 5.0
    clear[:, 0, 2] = 1.0
    velocity = np.zeros_like(clear)
    goals = np.array([[5.0, 0.0, 1.0]])
    starts = np.array([[0.0, 0.0, 1.0]])
    missed = evaluate_trajectory(time_s, clear, velocity, starts, goals, geom)
    assert missed["collision_obstacle"] is False
    assert missed["min_inter_drone_m"] is None
    hit = clear.copy()
    hit[1, 0, 0] = 0.0
    struck = evaluate_trajectory(time_s, hit, velocity, starts, goals, geom)
    assert struck["collision_obstacle"] is True
    pair_pos = np.zeros((2, 2, 3))
    pair_pos[:, :, 2] = 1.0
    pair_pos[:, 1, 1] = 0.1
    pair = evaluate_trajectory(
        np.array([0.0, 0.1]),
        pair_pos,
        np.zeros_like(pair_pos),
        np.zeros((2, 3)),
        np.array([[5.0, 0.0, 1.0], [5.0, 1.0, 1.0]]),
        {"boxes": [], "cylinders": []},
    )
    assert pair["collision_inter_drone"] is True
    assert pair["min_inter_drone_m"] == pytest.approx(0.1)


def test_hold_needs_a_full_second():
    geom = {"boxes": [], "cylinders": []}
    time_s = np.linspace(0.0, 1.0, 11)
    pos = np.zeros((11, 1, 3))
    pos[:, 0] = (9.0, 0.0, 1.0)
    vel = np.zeros_like(pos)
    goals = np.array([[9.0, 0.0, 1.0]])
    starts = np.array([[-9.0, 0.0, 1.0]])
    held = evaluate_trajectory(time_s, pos, vel, starts, goals, geom)
    assert held["arrived"] is True
    assert success_of("arrived", held) is True
    short = evaluate_trajectory(time_s[:3], pos[:3], vel[:3], starts, goals, geom)
    assert short["arrived"] is False
    assert success_of("timeout", {"arrived": None}) is None


def test_planner_log_without_times_is_unknown():
    stats = parse_planner_log("plan_success=0\nno timing\n")
    assert stats["planning_time_mean_s"] is None
    assert stats["plan_success_0"] == 1
    both = "Suddenly discovered obstacles. emergency stop!\n[SAFETY]: from EXEC_TRAJ to EMERGENCY_STOP\n"
    assert parse_planner_log(both)["emergency_stop"] == 1
    colored = "total time:\x1b[42m0.002\x1b[0m,optimize:0.001\n" * 3
    parsed = parse_planner_log(colored)
    assert parsed["planning_time_count"] == 3
    assert parsed["planning_time_mean_s"] == pytest.approx(0.002)


def test_observation_is_depth_state_and_neighbour(tmp_path):
    scenario = make_scenario(0.0, 3, "parallel", 0)
    ys = [d["start_enu"][1] for d in scenario["drones"]]

    def row(x):
        return [[x, y, 1.0] for y in ys]

    raw = {
        "run_status": "arrived",
        "wall_s": 1.0,
        "t0_ros_s": 100.0,
        "sample_hz": 10.0,
        "bspline_import": True,
        "yaw_source": "PositionCommand.yaw；没有指令时为 0",
        "acceleration_source": "odom.twist.angular（fake_drone 写入的指令加速度）",
        "time_s": [0.0, 0.1],
        "position_m": [row(-9.0), row(-8.0)],
        "velocity_mps": [[[0.0, 0.0, 0.0] for _ in ys], [[1.0, 0.0, 0.0] for _ in ys]],
        "acceleration_mps2": [[[0.0, 0.0, 0.0] for _ in ys], [[0.0, 0.0, 0.0] for _ in ys]],
        "yaw_rad": [[0.0, 0.0, 0.0], [0.1, 0.2, 0.0]],
        "bsplines": [
            {
                "topic": "broadcast",
                "drone_id": 1,
                "order": 3,
                "traj_id": 1,
                "start_time_ros_s": 100.0,
                "receive_ros_s": 100.05,
                "knots": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                "pos_pts": [[-9.0, 0.0, 1.0], [-8.0, 0.0, 1.0], [9.0, 0.0, 1.0]],
                "yaw_pts": [0.0],
                "yaw_dt": 0.1,
            }
        ],
    }
    record = write_dataset(tmp_path, scenario, raw, "")
    assert check_record(record) == []
    assert record["metrics"]["min_inter_drone_m"] == pytest.approx(6.0)
    obs, neighbours = observation_at(tmp_path, 0, 1)
    assert obs.frame.depth.shape == (96, 160)
    assert obs.state.p.tolist() == pytest.approx([-8.0, -6.0, 1.0])
    assert obs.goal.tolist() == pytest.approx(scenario["drones"][0]["goal_enu"])
    assert len(neighbours) == 2
    assert neighbours[0]["broadcast"]["pos_pts"][0] == [-9.0, 0.0, 1.0]
    later = {
        **raw["bsplines"][0],
        "receive_ros_s": 100.5,
        "pos_pts": [[0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0]],
    }
    record["bsplines"].append(later)
    (tmp_path / "record.json").write_text(json.dumps(record), encoding="utf-8")
    _, neighbours = observation_at(tmp_path, 0, 1)
    assert neighbours[0]["broadcast"]["pos_pts"][0] == [-9.0, 0.0, 1.0]


def test_empty_log_does_not_invent_a_planning_time(tmp_path):
    scenario = make_scenario(0.0, 1, "parallel", 0)
    raw = {
        "run_status": "launch_failed",
        "time_s": [],
        "bsplines": [],
        "bspline_import": False,
    }
    record = write_dataset(tmp_path, scenario, raw, "")
    assert record["metrics"]["success"] is None
    assert record["metrics"]["planning_time_mean_s"] is None
    assert record["metrics"]["min_inter_drone_m"] is None
