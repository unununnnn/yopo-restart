"""D6：飞行高度和 z 平移默认关，关时与原来逐位相同。"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from teacher.altitude_plan import (  # noqa: E402
    D2C_CELLS,
    keep_scene,
    plan_runs,
    runs_for_cell,
    scenes_per_hour,
)
from teacher.d2b_grid import grid_size, iter_d2b_scenarios  # noqa: E402
from teacher.launch_xml import launch_xml  # noqa: E402
from teacher.scenarios import make_scenario  # noqa: E402
from yopo_l0.distill import DistillError, convert_run, load_batch, parse_run_id, save_batch  # noqa: E402

FIXTURE = ROOT / "tests" / "fixtures" / "d3" / "d014-n1-headon-s0"
# 默认 1.0 m、单机、seed 0 的 launch XML。point1–4 的 z 改成跟目标走之后，1.0 m 仍是这个串。
_DEFAULT_LAUNCH_SHA256 = "874f70257ef31d979a6a2291d41d34bd53354a7d5a4e405774c9ae4672dcb874"


def _load_feasibility():
    path = ROOT / "scripts" / "distill" / "d6" / "feasibility.py"
    spec = importlib.util.spec_from_file_location("d6_feasibility", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_zero_shift_matches_the_unshifted_conversion():
    base = convert_run(FIXTURE, max_windows=3)
    zero = convert_run(FIXTURE, max_windows=3, altitude_shift_m=0.0)
    assert base.meta() == zero.meta()
    assert "altitude_shift_m" not in base.meta()
    for name in ("depth", "state", "position_body", "velocity_body", "yaw_rel", "neighbor_rel", "neighbor_mask"):
        np.testing.assert_array_equal(getattr(base, name), getattr(zero, name))
    np.testing.assert_array_equal(base.sample_index, zero.sample_index)
    np.testing.assert_array_equal(base.ego_index, zero.ego_index)


def test_half_meter_shift_rerenders_depth_and_keeps_body_frame(tmp_path):
    base = convert_run(FIXTURE, max_windows=3)
    shifted = convert_run(FIXTURE, max_windows=3, altitude_shift_m=0.5)
    assert shifted.altitude_shift_m == 0.5
    assert shifted.meta()["altitude_shift_m"] == 0.5
    assert not np.array_equal(base.depth, shifted.depth)
    for name in ("state", "position_body", "velocity_body", "yaw_rel", "neighbor_rel", "neighbor_mask"):
        np.testing.assert_allclose(getattr(base, name), getattr(shifted, name), atol=1e-12, rtol=0.0)
    path = save_batch(shifted, tmp_path / "shifted.npz")
    loaded = load_batch(path)
    assert loaded.altitude_shift_m == 0.5
    np.testing.assert_array_equal(loaded.depth, shifted.depth)


def test_non_finite_shift_is_rejected():
    with pytest.raises(DistillError, match="altitude_shift_m"):
        convert_run(FIXTURE, max_windows=1, altitude_shift_m=float("nan"))


def test_cli_without_shift_matches_explicit_zero(tmp_path):
    script = ROOT / "scripts" / "distill" / "teacher_to_tensors.py"
    spec = importlib.util.spec_from_file_location("teacher_to_tensors", script)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    plain = tmp_path / "plain.npz"
    zero = tmp_path / "zero.npz"
    assert mod.main(["--run", str(FIXTURE), "--out", str(plain), "--max-windows", "2"]) == 0
    assert mod.main(["--run", str(FIXTURE), "--out", str(zero), "--max-windows", "2", "--altitude-shift", "0"]) == 0
    a, b = load_batch(plain), load_batch(zero)
    np.testing.assert_array_equal(a.depth, b.depth)
    np.testing.assert_array_equal(a.position_body, b.position_body)
    assert a.meta() == b.meta()


def test_default_flight_z_is_unchanged_and_one_point_five_only_changes_z():
    base = make_scenario(0.14, 3, "headon", 4)
    same = make_scenario(0.14, 3, "headon", 4, flight_z_m=1.0)
    higher = make_scenario(0.14, 3, "headon", 4, flight_z_m=1.5)
    assert base == same
    assert base["id"] == "d014-n3-headon-s4"
    assert higher["id"] == "d014-n3-headon-z150-s4"
    assert higher["flight_z_m"] == 1.5
    assert higher["drones"][0]["start_enu"][2] == 1.5
    assert higher["drones"][0]["goal_enu"][2] == 1.5
    assert higher["cylinders"] == base["cylinders"]
    assert [d["start_enu"][:2] for d in higher["drones"]] == [d["start_enu"][:2] for d in base["drones"]]


def test_launch_xml_at_one_meter_is_the_previous_string():
    import hashlib

    xml = launch_xml(make_scenario(0.0, 1, "parallel", 0))
    assert hashlib.sha256(xml.encode()).hexdigest() == _DEFAULT_LAUNCH_SHA256
    higher = launch_xml(make_scenario(0.0, 1, "parallel", 0, flight_z_m=1.5))
    assert '<arg name="point0_z" value="1.5"/>' in higher
    assert '<arg name="point1_z" value="1.5"/>' in higher
    assert '<param name="init_z" value="1.5"/>' in higher
    assert 'value="1.0"' not in higher.split("traj_server/time_forward")[0]


def test_default_d2b_grid_stays_1792_and_one_point_five_doubles_it():
    assert grid_size(128, True) == 1792
    assert grid_size(128, True, (1.0, 1.5)) == 3584
    default_ids = [row["id"] for row in iter_d2b_scenarios(seed_count=128, include_six_open=True)]
    assert len(default_ids) == 1792
    assert all("z150" not in i for i in default_ids)
    both = list(iter_d2b_scenarios(seed_count=1, include_six_open=True, flight_zs=(1.0, 1.5)))
    assert len(both) == 28
    assert sum(1 for row in both if row["flight_z_m"] == 1.5) == 14
    info = parse_run_id("d014-n3-headon-z150-s0")
    assert info["seed"] == 0
    assert info["scene_key"] == "d014-n3-headon"
    assert info["flight_z_m"] == 1.5
    plain = parse_run_id("d014-n1-headon-s20")
    assert "flight_z_m" not in plain
    assert plain["seed"] == 20


def test_clearance_plan_matches_the_d2c_rates():
    assert len(D2C_CELLS) == 14
    assert sum(c.planned for c in D2C_CELLS) == 1792
    cell = next(c for c in D2C_CELLS if c.key == "d028-n3-parallel")
    assert runs_for_cell(cell, quota=128, fresh=True, cap=None) == 4096
    assert runs_for_cell(cell, quota=128, fresh=False, cap=None) == 3968
    assert runs_for_cell(cell, quota=128, fresh=True, cap=512) == 512
    assert runs_for_cell(cell, quota=128, fresh=False, cap=512) == 384
    headon = next(c for c in D2C_CELLS if c.key == "d028-n3-headon")
    assert headon.concluded == 121
    assert runs_for_cell(headon, quota=128, fresh=True, cap=None) == 1107
    open_cell = next(c for c in D2C_CELLS if c.key == "d000-n6-parallel")
    assert runs_for_cell(open_cell, quota=128, fresh=True, cap=None) == 128
    assert runs_for_cell(open_cell, quota=128, fresh=False, cap=None) == 0
    bare = plan_runs(cap=None)
    capped = plan_runs(cap=512)
    assert bare["new_altitude_runs"] == 7467
    assert bare["backfill_runs"] == 5682
    assert bare["total_runs"] == 13149
    assert capped["new_altitude_runs"] == 3288
    assert capped["backfill_runs"] == 1496
    assert capped["total_runs"] == 4784
    assert scenes_per_hour() == pytest.approx(1249 / 13019.918 * 3600.0)
    assert keep_scene({"success": True, "min_obstacle_clearance_m": 0.0})
    assert not keep_scene({"success": True, "min_obstacle_clearance_m": -0.01})
    assert not keep_scene({"success": False, "min_obstacle_clearance_m": 0.2})


def test_sample_holdout_is_not_the_d5_seed_rule():
    mod = _load_feasibility()
    ids = mod.successful_sample_ids()
    assert len(ids) == 10
    assert all(i.endswith("-s0") for i in ids)
    train, hold = mod.split_sample_ids(ids)
    assert hold == ["d000-n1-headon-s0", "d000-n6-parallel-s0"]
    assert len(train) == 8
    assert set(train).isdisjoint(hold)
