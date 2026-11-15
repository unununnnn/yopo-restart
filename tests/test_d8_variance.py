"""D8：方差与 g2 归因编排的纯函数检查（不跑闭环、不起容器）。"""
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def _mod(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


V = _mod("t_d8_variance", ROOT / "scripts/distill/d8/variance.py")
REC = _mod("t_d8_record", ROOT / "scripts/distill/d5b/ros/dagger_record.py")


def test_select_and_eval_seeds_disjoint():
    for scene, seeds in V.SELECT_SEEDS.items():
        assert not set(seeds) & set(V.EVAL_SEEDS[scene])


def test_speed_cap_off_by_default():
    from yopo_l0.policy import StudentPolicy, read_knot_speed_cap

    assert "knot_speed_cap_mps" not in StudentPolicy.DEFAULTS
    assert read_knot_speed_cap(dict(StudentPolicy.DEFAULTS)) is None
    with pytest.raises(ValueError):
        read_knot_speed_cap({"knot_speed_cap_mps": -1.0})


def test_cap_knot_steps_limits_speed_and_keeps_slow_paths():
    from yopo_l0.policy import cap_horizontal, cap_knot_steps, student_knot_velocities

    p0 = np.zeros(3)
    fast = np.stack([[0.5 * (i + 1), 0.0, 0.1 * i] for i in range(8)])  # 0.5 m / 0.1 s = 5 m/s
    capped = cap_knot_steps(p0, fast, 0.1, 2.0)
    steps = np.diff(np.vstack([p0, capped])[:, :2], axis=0)
    assert np.all(np.linalg.norm(steps, axis=1) <= 0.2 + 1e-9)
    assert np.allclose(capped[:, 2], fast[:, 2])
    v = cap_horizontal(student_knot_velocities(p0, capped, 0.1), 2.0)
    assert np.all(np.linalg.norm(v[:, :2], axis=1) <= 2.0 + 1e-9)
    slow = fast * 0.1
    assert np.array_equal(cap_knot_steps(p0, slow, 0.1, 2.0), slow)


def test_recorder_boxes_only_when_given():
    base = {"cylinders": [{"center_xy": [3.0, 0.0], "radius": 0.5, "z_lo": 0.0, "z_hi": 2.8}]}
    a = REC.cloud_points(base, 0.1, (0, 0, -10))
    b = REC.cloud_points({**base, "boxes": []}, 0.1, (0, 0, -10))
    assert a == b
    pts = REC.box_points([2.7, -0.4, 0.0], [3.3, 0.4, 2.8], 0.1)
    arr = np.asarray(pts)
    assert arr[:, 0].min() == pytest.approx(2.7) and arr[:, 0].max() == pytest.approx(3.3)
    assert arr[:, 1].min() == pytest.approx(-0.4) and arr[:, 2].max() == pytest.approx(2.8)


def test_exact_scenario_changes_only_g2():
    drone = {"ros_id": 0, "id": "uav0", "p": [0, 0, 1], "v": [0, 0, 0], "a": [0, 0, 0], "yaw": 0.0, "goal": [5, 0, 1]}
    job = {"id": "x", "round": 1, "seed": 5, "t": 0.5, "altitude_m": 1.0, "drones": [drone]}
    g2 = V.exact_scenario({**job, "scene": "g2-pillar"})
    assert g2["cylinders"] == [] and {b["name"] for b in g2["boxes"]} >= {"pillar", "left_wall", "right_wall"}
    op = V.exact_scenario({**job, "scene": "open-3drone"})
    assert op == V.d7().teacher_scenario({**job, "scene": "open-3drone"})
    assert "boxes" not in V.d7().teacher_scenario({**job, "scene": "g2-pillar"})


def test_score_orders_by_arrival_then_collisions():
    a = {"rows": [{"arrived": 3, "obstacle": 0, "inter_drone": 0, "final_goal_error_m": 0.1}]}
    b = {"rows": [{"arrived": 2, "obstacle": 0, "inter_drone": 0, "final_goal_error_m": 0.0}]}
    c = {"rows": [{"arrived": 3, "obstacle": 1, "inter_drone": 0, "final_goal_error_m": 0.0}]}
    assert V.score(a) > V.score(b) and V.score(a) > V.score(c)
