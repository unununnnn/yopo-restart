"""票 D6c：开阔场景到不了终点的归因。不改飞行库的默认。"""

from __future__ import annotations

import hashlib
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

ROOT = Path(__file__).resolve().parents[1]
OPEN = ROOT / "scenes" / "open-3drone.json"


def _diagnose():
    name = "d6c_diagnose"
    cached = sys.modules.get(name)
    if cached is not None:
        return cached
    path = ROOT / "scripts" / "distill" / "d6c" / "diagnose.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _weights(tmp_path: Path) -> Path:
    from yopo_l0.student import MultiHeadStudent, StudentConfig, save_checkpoint

    torch.manual_seed(0)
    path, _ = save_checkpoint(MultiHeadStudent(StudentConfig(enabled=True)), tmp_path / "student.pt")
    return path


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_published_open_error_is_outside_both_radii():
    diag = _diagnose()
    reading = diag.published_reading()
    error = reading["published"]["open_final_goal_error_m"]
    assert error > diag.ARRIVE_RADIUS_M
    assert error > diag.GOAL_TOLERANCE_M
    assert reading["open_along_track_if_straight_m"] == pytest.approx(6.0 - error)
    assert reading["g2_mean_speed_mps_if_straight_and_on_axis"] > 1.0


def test_teacher_start_horizon_does_not_move_in_the_first_second():
    forward = _diagnose().teacher_start_forward_m()
    assert forward.shape == (8,)
    assert float(forward[0]) == pytest.approx(0.0, abs=1e-6)
    assert float(forward[4]) < 0.01
    assert float(forward[-1]) > 0.1


def test_cruise_knots_point_along_the_goal():
    diag = _diagnose()
    state = np.array([0, 0, 0, 0, 0, 0, 6.0, 0.0, 0.0], dtype=np.float64)
    forward, speed = diag.cruise_forward()
    traj = diag.trajectory_along_goal(state, forward, speed)
    end = traj.position_body[0, -1]
    assert end[0] == pytest.approx(diag.CRUISE_MPS * 1.6)
    assert abs(float(end[1])) < 1e-6
    assert abs(float(end[2])) < 1e-6


def test_altitude_changes_depth_not_the_goal_vector():
    report = _diagnose().altitude_observation()
    assert report["goal_body_max_abs"] == pytest.approx(0.0, abs=1e-6)
    assert report["goal_body_1_5"][0] == pytest.approx(6.0)
    assert report["goal_body_1_5"][2] == pytest.approx(0.0)
    assert report["depth_changed_frac"] > 0.1


def test_interface_report_matches_r4_and_the_scene_gate():
    from yopo_l0.policy import StudentPolicy

    report = _diagnose().interface_report()
    assert report["d5_defaults"]["knots"] == "network"
    assert report["r4_eval_params"]["vertical"] == "hold_goal_z"
    assert report["r4_eval_params"]["yaw_source"] == "ego_lookahead"
    gate = report["goal_criterion"]
    assert gate["runner_tolerance_m"] == pytest.approx(0.4)
    assert gate["student_arrive_radius_m"] == pytest.approx(StudentPolicy.DEFAULTS["arrive_radius_m"])
    assert gate["open_duration_s"] == pytest.approx(8.0)
    assert StudentPolicy.DEFAULTS["knots"] == "network"


def test_cruise_reaches_and_pause_stays_and_defaults_stay(tmp_path):
    from yopo_l0.policy import StudentPolicy
    import yopo_l0.student as student_mod

    diag = _diagnose()
    weights = _weights(tmp_path)
    before_defaults = dict(StudentPolicy.DEFAULTS)
    before_plan = StudentPolicy.plan
    before_predict = student_mod.predict
    scene_sha = _sha(OPEN)
    base = {
        "params": dict(diag.R4),
        "altitude_m": 1.5,
        "seed": 0,
        "drones": 3,
        "weights": str(weights),
        "duration_s": 8.0,
    }
    pause = diag.run_episode({**base, "name": "pause-r4", "predict": "teacher_start"})
    cruise = diag.run_episode({**base, "name": "cruise-r4", "predict": "cruise"})
    assert pause["outcome"] == "TIMEOUT"
    assert pause["arrived"] == 0
    assert pause["final_goal_error_m"] == pytest.approx(6.0, abs=1e-6)
    assert pause["drones"][0]["final_p"][0] == pytest.approx(0.0, abs=1e-3)
    assert pause["planner"][0]["goal_body"][0] == pytest.approx(6.0)
    assert pause["planner"][0]["p_body_end"][0] < 0.3
    assert cruise["outcome"] == "SUCCESS"
    assert cruise["arrived"] == 3
    assert cruise["final_goal_error_m"] < diag.GOAL_TOLERANCE_M
    assert cruise["drones"][0]["final_p"][2] == pytest.approx(1.5, abs=1e-3)
    assert cruise["planner"][0]["horizon_forward_m"] == pytest.approx(diag.CRUISE_MPS * 1.6, abs=1e-3)
    assert StudentPolicy.DEFAULTS == before_defaults
    assert StudentPolicy.plan is before_plan
    assert student_mod.predict is before_predict
    assert _sha(OPEN) == scene_sha
