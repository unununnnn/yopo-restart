"""票 D6e：起步结点沿目标位移改写，开关默认关。"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
PIPELINE = ROOT / "scripts" / "distill" / "d6" / "pipeline.py"
PY = sys.executable
SAMPLE = ROOT / "data" / "teacher" / "sample" / "d000-n3-parallel-s0"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _arrays(batch):
    return (
        batch.depth,
        batch.state,
        batch.position_body,
        batch.velocity_body,
        batch.yaw_rel,
        batch.neighbor_rel,
        batch.neighbor_mask,
        batch.time_s,
        batch.sample_index,
        batch.ego_index,
    )


def _write_run(directory: Path, *, still_until_s: float, stop_after_s: float, duration_s: float) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    times = np.arange(0.0, duration_s + 1e-9, 0.1, dtype=np.float64)
    position = np.zeros((times.size, 1, 3), dtype=np.float64)
    velocity = np.zeros_like(position)
    position[:, 0, 2] = 1.0
    for i, t in enumerate(times):
        if t < still_until_s or t >= stop_after_s:
            continue
        position[i, 0, 0] = (t - still_until_s) * 2.0
        velocity[i, 0, 0] = 2.0
    if stop_after_s < duration_s:
        held = (stop_after_s - still_until_s) * 2.0
        position[times >= stop_after_s, 0, 0] = held
    goal = np.array([[20.0, 0.0, 1.0]], dtype=np.float64)
    np.savez_compressed(
        directory / "trajectory.npz",
        time_s=times,
        position_m=position,
        velocity_mps=velocity,
        acceleration_mps2=np.zeros_like(position),
        yaw_rad=np.zeros((times.size, 1), dtype=np.float64),
        goal_m=goal,
    )
    record = {
        "schema": "yopo-teacher-record/1",
        "scenario": {"id": directory.name, "n_drones": 1, "flight_z_m": 1.0},
        "metrics": {"success": True, "min_obstacle_clearance_m": 0.8},
        "run": {"status": "arrived"},
    }
    (directory / "record.json").write_text(json.dumps(record), encoding="utf-8")
    (directory / "geometry.json").write_text(json.dumps({"boxes": [], "cylinders": []}), encoding="utf-8")
    scene = {
        "name": directory.name,
        "geometry": "geometry.json",
        "ground_z": 0.0,
        "drones": [{"id": "uav0", "start_enu": [0.0, 0.0, 1.0], "goal_enu": [20.0, 0.0, 1.0]}],
        "task": {"type": "goto", "duration_s": 5.0},
        "criteria": {"frozen": False, "body_radius_m": 0.2},
        "camera": {
            "width": 160,
            "height": 96,
            "hfov_deg": 90.0,
            "vfov_deg": 60.0,
            "min_range_m": 0.2,
            "max_range_m": 20.0,
            "noise_std_m": 0.0,
            "mount_xyz_body": [0.0, 0.0, 0.0],
        },
        "dynamics": {"model": "point_mass_lag", "tau_s": 0.1, "kp": 4.0, "kd": 3.0, "a_max": 6.0},
        "rates_hz": {"physics": 100, "camera": 30, "planner": 10},
    }
    (directory / "scene.json").write_text(json.dumps(scene), encoding="utf-8")


def test_start_knot_horizon_locks_the_0_2s_displacement():
    from yopo_l0.distill import START_KNOT_SPEED_MPS, DistillError, start_knot_horizon
    from yopo_l0.student import HORIZON_DT_S, HORIZON_STEPS

    goal = np.array([6.0, 0.0, 0.0], dtype=np.float64)
    pos, vel = start_knot_horizon(goal, dt_s=HORIZON_DT_S, n_steps=HORIZON_STEPS, speed_mps=START_KNOT_SPEED_MPS)
    assert pos.shape == (HORIZON_STEPS, 3)
    assert vel.shape == (HORIZON_STEPS, 3)
    assert pos[0, 0] == pytest.approx(START_KNOT_SPEED_MPS * HORIZON_DT_S)
    assert pos[0, 0] == pytest.approx(0.266)
    assert pos[0, 1] == pytest.approx(0.0)
    assert pos[0, 2] == pytest.approx(0.0)
    assert vel[0, 0] == pytest.approx(START_KNOT_SPEED_MPS)
    assert np.allclose(pos[:, 0], START_KNOT_SPEED_MPS * HORIZON_DT_S * np.arange(1, HORIZON_STEPS + 1))
    sideways = np.array([0.0, 4.0, 0.0], dtype=np.float64)
    pos_y, vel_y = start_knot_horizon(sideways, dt_s=0.2, n_steps=8, speed_mps=1.33)
    assert pos_y[0, 1] == pytest.approx(0.266)
    assert pos_y[0, 0] == pytest.approx(0.0)
    zero, zero_v = start_knot_horizon(np.zeros(3), dt_s=0.2, n_steps=8, speed_mps=1.33)
    assert np.array_equal(zero, np.zeros((8, 3)))
    assert np.array_equal(zero_v, np.zeros((8, 3)))
    with pytest.raises(DistillError, match="start_knot_speed_mps"):
        start_knot_horizon(goal, dt_s=0.2, n_steps=8, speed_mps=0.0)
    with pytest.raises(DistillError, match="start_knot_speed_mps"):
        start_knot_horizon(goal, dt_s=0.2, n_steps=8, speed_mps=float("nan"))


def test_switch_off_matches_omitted_argument_byte_for_byte(tmp_path):
    from yopo_l0.distill import convert_run, save_batch

    plain = convert_run(SAMPLE, max_windows=2)
    explicit = convert_run(
        SAMPLE,
        max_windows=2,
        rewrite_start_knots=False,
        start_knot_speed_mps=float("nan"),
        drop_start_still=False,
    )
    assert plain.n_start_knots_rewritten == 0
    assert "n_start_knots_rewritten" not in plain.meta()
    assert "start_knot_speed_mps" not in plain.meta()
    assert plain.meta() == explicit.meta()
    for left, right in zip(_arrays(plain), _arrays(explicit), strict=True):
        assert np.array_equal(left, right)
    left = save_batch(plain, tmp_path / "plain.npz")
    right = save_batch(explicit, tmp_path / "explicit.npz")
    assert left.read_bytes() == right.read_bytes()


def test_switch_on_rewrites_start_knots_and_keeps_later_windows(tmp_path):
    from yopo_l0.distill import START_KNOT_SPEED_MPS, convert_run, start_knot_horizon
    from yopo_l0.student import HORIZON_DT_S, HORIZON_STEPS

    run = tmp_path / "d000-n1-parallel-s1"
    _write_run(run, still_until_s=1.5, stop_after_s=3.2, duration_s=5.0)
    off = convert_run(run)
    on = convert_run(run, rewrite_start_knots=True)
    assert on.n == off.n
    assert on.n_start_knots_rewritten > 0
    assert on.n_start_still_dropped == 0
    assert "n_start_knots_rewritten" in on.meta()
    assert on.meta()["start_knot_first_disp_m"] == pytest.approx(0.266)
    assert on.meta()["start_knot_speed_mps"] == pytest.approx(START_KNOT_SPEED_MPS)
    start_mask = on.time_s < 1.5 - 1e-12
    later_mask = ~start_mask
    assert int(np.count_nonzero(start_mask)) == on.n_start_knots_rewritten
    assert np.any(later_mask)
    for n in np.flatnonzero(start_mask):
        expect_p, expect_v = start_knot_horizon(
            off.state[n, 6:9],
            dt_s=HORIZON_DT_S,
            n_steps=HORIZON_STEPS,
            speed_mps=START_KNOT_SPEED_MPS,
        )
        assert np.allclose(on.position_body[n], expect_p.astype(np.float32))
        assert np.allclose(on.velocity_body[n], expect_v.astype(np.float32))
        assert float(np.linalg.norm(on.position_body[n, 0])) == pytest.approx(0.266)
        assert np.array_equal(on.depth[n], off.depth[n])
        assert np.array_equal(on.state[n], off.state[n])
        assert np.array_equal(on.yaw_rel[n], off.yaw_rel[n])
    for n in np.flatnonzero(later_mask):
        assert np.array_equal(on.position_body[n], off.position_body[n])
        assert np.array_equal(on.velocity_body[n], off.velocity_body[n])
        assert np.array_equal(on.depth[n], off.depth[n])
        assert np.array_equal(on.state[n], off.state[n])
    # 已经在动的轨迹开开关也不改字节。
    moving = tmp_path / "d000-n1-parallel-s2"
    _write_run(moving, still_until_s=0.0, stop_after_s=5.0, duration_s=5.0)
    base = convert_run(moving, max_windows=2)
    same = convert_run(moving, max_windows=2, rewrite_start_knots=True)
    assert same.n_start_knots_rewritten == 0
    assert same.meta() == base.meta()
    for left, right in zip(_arrays(base), _arrays(same), strict=True):
        assert np.array_equal(left, right)


def test_sample_t0_first_knot_gets_along_goal_displacement():
    from yopo_l0.distill import START_KNOT_SPEED_MPS, convert_run
    from yopo_l0.student import HORIZON_DT_S

    off = convert_run(SAMPLE)
    on = convert_run(SAMPLE, rewrite_start_knots=True)
    assert off.n == on.n
    t0 = np.flatnonzero(off.time_s == off.time_s.min())
    assert t0.size == 3
    for n in t0:
        raw = np.linalg.norm(off.position_body[n, 0, :2])
        assert raw < 1e-6
        goal = off.state[n, 6:9]
        direction = goal / np.linalg.norm(goal)
        got = on.position_body[n, 0]
        expect = direction * START_KNOT_SPEED_MPS * HORIZON_DT_S
        assert np.allclose(got, expect.astype(np.float32), atol=1e-6)
        assert float(np.linalg.norm(got)) == pytest.approx(0.266)
        # 水平投影沿目标，不指反向。
        assert float(np.dot(got[:2], goal[:2])) > 0.0


def test_drop_start_still_stays_off_when_rewriting(tmp_path):
    from yopo_l0.distill import convert_run

    run = tmp_path / "d000-n1-parallel-s1"
    _write_run(run, still_until_s=1.5, stop_after_s=3.2, duration_s=5.0)
    off = convert_run(run)
    rewritten = convert_run(run, rewrite_start_knots=True)
    dropped = convert_run(run, drop_start_still=True)
    assert rewritten.n == off.n
    assert rewritten.n_start_still_dropped == 0
    assert dropped.n < off.n
    assert dropped.n_start_knots_rewritten == 0
    both = convert_run(run, drop_start_still=True, rewrite_start_knots=True)
    assert both.n == dropped.n
    assert both.n_start_knots_rewritten == 0
    assert both.n_start_still_dropped == dropped.n_start_still_dropped


def test_pipeline_flag_off_reuses_and_flag_on_rewrites(tmp_path):
    pipe = _load("d6e_pipeline", PIPELINE)
    run = tmp_path / "runs" / "d000-n1-parallel-s1"
    _write_run(run, still_until_s=1.5, stop_after_s=3.2, duration_s=5.0)
    off = tmp_path / "off"
    on = tmp_path / "on"
    common = ["--only-convert", "--teacher-dirs", str(tmp_path / "runs"), "--jobs", "1", "--max-windows", "4"]
    assert pipe.main([*common, "--out", str(off)]) == 0
    manifest = json.loads((off / "convert-manifest.json").read_text(encoding="utf-8"))
    assert manifest["rewrite_start_knots"] is False
    assert manifest["start_knot_speed_mps"] is None
    assert manifest["drop_start_still"] is False
    assert manifest["n_start_knots_rewritten"] == 0
    sha = manifest["files"]["d000-n1-parallel-s1"]["sha256"]
    assert pipe.main([*common, "--out", str(off)]) == 0
    resumed = json.loads((off / "convert-manifest.json").read_text(encoding="utf-8"))
    assert resumed["n_reused"] == 1
    assert resumed["files"]["d000-n1-parallel-s1"]["sha256"] == sha
    assert pipe.main([*common, "--out", str(on), "--rewrite-start-knots"]) == 0
    gated = json.loads((on / "convert-manifest.json").read_text(encoding="utf-8"))
    assert gated["rewrite_start_knots"] is True
    assert gated["start_knot_speed_mps"] == pytest.approx(1.33)
    assert gated["drop_start_still"] is False
    assert gated["n_start_still_dropped"] == 0
    assert gated["n_start_knots_rewritten"] > 0
    assert gated["files"]["d000-n1-parallel-s1"]["sha256"] != sha
