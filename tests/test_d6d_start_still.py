"""票 D6d：起步静止窗开关默认关，开时只删起步结点之前的低速窗。"""

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


def test_keep_after_startup_still_drops_only_the_prefix():
    from yopo_l0.distill import DistillError, keep_after_startup_still

    times = np.arange(0.0, 5.0 + 1e-9, 0.1, dtype=np.float64)
    velocity = np.zeros((times.size, 1, 3), dtype=np.float64)
    moving = (times >= 1.5) & (times < 3.2)
    velocity[moving, 0, 0] = 1.0
    pairs = np.stack([np.arange(times.size, dtype=np.int64), np.zeros(times.size, dtype=np.int64)], axis=1)
    kept, dropped = keep_after_startup_still(pairs, times, velocity, 0.05)
    assert dropped == int(np.count_nonzero(times < 1.5))
    assert kept.shape[0] == times.size - dropped
    assert np.all(times[kept[:, 0]] >= 1.5 - 1e-12)
    late = kept[times[kept[:, 0]] >= 3.2 - 1e-12]
    assert late.shape[0] > 0
    assert np.all(np.linalg.norm(velocity[late[:, 0], late[:, 1]], axis=1) == 0.0)
    with pytest.raises(DistillError, match="start_still_speed_mps"):
        keep_after_startup_still(pairs, times, velocity, -0.1)


def test_parallel_sample_prefix_is_before_the_motion_node():
    from yopo_l0.distill import START_STILL_SPEED_MPS, keep_after_startup_still
    from yopo_l0.student import HORIZON_DT_S, HORIZON_STEPS

    with np.load(SAMPLE / "trajectory.npz") as npz:
        time_s = np.asarray(npz["time_s"], dtype=np.float64)
        velocity = np.asarray(npz["velocity_mps"], dtype=np.float64)
    horizon = float(HORIZON_STEPS) * float(HORIZON_DT_S)
    valid = np.flatnonzero(time_s + horizon <= float(time_s[-1]) + 1e-9)
    n_drones = int(velocity.shape[1])
    pairs = np.array([(int(i), d) for i in valid for d in range(n_drones)], dtype=np.int64)
    speed = np.linalg.norm(velocity, axis=2)
    kept, dropped = keep_after_startup_still(pairs, time_s, velocity, START_STILL_SPEED_MPS)
    assert dropped > 0
    assert not np.any(kept[:, 0] == 0)
    for drone in range(n_drones):
        onset = int(np.flatnonzero(speed[:, drone] >= START_STILL_SPEED_MPS)[0])
        assert float(time_s[onset]) > 1.0
        drone_kept = kept[kept[:, 1] == drone]
        assert drone_kept.shape[0] > 0
        assert int(drone_kept[:, 0].min()) >= onset
        assert float(speed[0, drone]) < START_STILL_SPEED_MPS


def test_switch_off_matches_omitted_argument_byte_for_byte(tmp_path):
    from yopo_l0.distill import convert_run, save_batch

    plain = convert_run(SAMPLE, max_windows=2)
    explicit = convert_run(SAMPLE, max_windows=2, drop_start_still=False, start_still_speed_mps=float("nan"))
    assert plain.n_start_still_dropped == 0
    assert "n_start_still_dropped" not in plain.meta()
    assert plain.meta() == explicit.meta()
    for left, right in zip(_arrays(plain), _arrays(explicit), strict=True):
        assert np.array_equal(left, right)
    left = save_batch(plain, tmp_path / "plain.npz")
    right = save_batch(explicit, tmp_path / "explicit.npz")
    assert left.read_bytes() == right.read_bytes()


def test_switch_on_keeps_moving_rows_and_the_later_stop(tmp_path):
    from yopo_l0.distill import convert_run

    run = tmp_path / "d000-n1-parallel-s1"
    _write_run(run, still_until_s=1.5, stop_after_s=3.2, duration_s=5.0)
    off = convert_run(run)
    on = convert_run(run, drop_start_still=True)
    assert on.n < off.n
    assert on.n_start_still_dropped == off.n - on.n
    assert "n_start_still_dropped" in on.meta()
    assert float(on.time_s.min()) >= 1.5 - 1e-9
    assert np.any(on.time_s >= 3.2 - 1e-9)
    off_key = {(int(i), int(d)): n for n, (i, d) in enumerate(zip(off.sample_index, off.ego_index))}
    for n, (i, d) in enumerate(zip(on.sample_index, on.ego_index)):
        src = off_key[(int(i), int(d))]
        assert np.array_equal(on.depth[n], off.depth[src])
        assert np.array_equal(on.position_body[n], off.position_body[src])
        assert np.array_equal(on.state[n], off.state[src])
    # 已经在动的轨迹开开关也不改字节。
    moving = tmp_path / "d000-n1-parallel-s2"
    _write_run(moving, still_until_s=0.0, stop_after_s=5.0, duration_s=5.0)
    base = convert_run(moving, max_windows=2)
    same = convert_run(moving, max_windows=2, drop_start_still=True)
    assert same.n_start_still_dropped == 0
    assert same.meta() == base.meta()
    for left, right in zip(_arrays(base), _arrays(same), strict=True):
        assert np.array_equal(left, right)


def test_pipeline_flag_off_reuses_and_flag_on_drops(tmp_path):
    pipe = _load("d6d_pipeline", PIPELINE)
    run = tmp_path / "runs" / "d000-n1-parallel-s1"
    _write_run(run, still_until_s=1.5, stop_after_s=3.2, duration_s=5.0)
    off = tmp_path / "off"
    on = tmp_path / "on"
    common = ["--only-convert", "--teacher-dirs", str(tmp_path / "runs"), "--jobs", "1", "--max-windows", "4"]
    assert pipe.main([*common, "--out", str(off)]) == 0
    manifest = json.loads((off / "convert-manifest.json").read_text(encoding="utf-8"))
    assert manifest["drop_start_still"] is False
    assert manifest["start_still_speed_mps"] is None
    assert manifest["n_start_still_dropped"] == 0
    sha = manifest["files"]["d000-n1-parallel-s1"]["sha256"]
    assert pipe.main([*common, "--out", str(off)]) == 0
    resumed = json.loads((off / "convert-manifest.json").read_text(encoding="utf-8"))
    assert resumed["n_reused"] == 1
    assert resumed["files"]["d000-n1-parallel-s1"]["sha256"] == sha
    assert pipe.main([*common, "--out", str(on), "--drop-start-still"]) == 0
    gated = json.loads((on / "convert-manifest.json").read_text(encoding="utf-8"))
    assert gated["drop_start_still"] is True
    assert gated["start_still_speed_mps"] == pytest.approx(0.05)
    assert gated["n_start_still_dropped"] > 0
    # max_windows 把两边都收成 4 窗，但开臂是在删掉起步窗之后再抽，字节不同。
    assert gated["files"]["d000-n1-parallel-s1"]["sha256"] != sha
