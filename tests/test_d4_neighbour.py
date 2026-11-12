"""票 D4：学生干路加邻机输入，默认关，不进飞行环。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from yopo_l0.distill import TENSOR_SCHEMA, convert_run, load_batch
from yopo_l0.student import (
    DEFAULT_ENABLED,
    NEIGHBOR_FEAT_DIM,
    NEIGHBOR_K,
    MultiHeadStudent,
    StudentConfig,
    pack_neighbor_features,
    predict,
    train_smoke,
)

from conftest import ROOT
from test_d3_student import FLIGHT_FILES, _write_run

SAMPLE_N3 = ROOT / "data/teacher/sample/d000-n3-headon-s0"


def _write_placed(
    directory: Path,
    starts: list[tuple[float, float, float]],
    *,
    velocities: list[tuple[float, float, float]] | None = None,
    yaw: float = 0.0,
    duration_s: float = 2.0,
    success: bool = True,
):
    """按给定起点写一条多机直线记录。速度默认沿偏航方向 2 m/s。"""
    n = len(starts)
    if velocities is None:
        heading = np.array([np.cos(yaw), np.sin(yaw), 0.0], dtype=np.float64)
        velocities = [tuple((2.0 * heading).tolist())] * n
    directory.mkdir(parents=True, exist_ok=True)
    times = np.arange(0.0, duration_s + 1e-9, 0.1, dtype=np.float64)
    position = np.zeros((times.size, n, 3), dtype=np.float64)
    velocity = np.zeros((times.size, n, 3), dtype=np.float64)
    acceleration = np.zeros_like(position)
    yaw_all = np.full((times.size, n), yaw, dtype=np.float64)
    goals = np.zeros((n, 3), dtype=np.float64)
    for i, start in enumerate(starts):
        p0 = np.asarray(start, dtype=np.float64)
        v = np.asarray(velocities[i], dtype=np.float64)
        position[:, i, :] = p0 + times[:, None] * v
        velocity[:, i, :] = v
        goals[i] = p0 + v / (np.linalg.norm(v) + 1e-12) * 20.0
        goals[i, 2] = p0[2]
    np.savez_compressed(
        directory / "trajectory.npz",
        time_s=times,
        position_m=position,
        velocity_mps=velocity,
        acceleration_mps2=acceleration,
        yaw_rad=yaw_all,
        goal_m=goals,
    )
    record = {
        "schema": "yopo-teacher-record/1",
        "scenario": {"id": directory.name, "n_drones": n},
        "metrics": {"success": success},
        "run": {"status": "arrived" if success else "stopped"},
    }
    (directory / "record.json").write_text(json.dumps(record), encoding="utf-8")
    (directory / "geometry.json").write_text(json.dumps({"boxes": [], "cylinders": []}), encoding="utf-8")
    scene = {
        "name": directory.name,
        "geometry": "geometry.json",
        "ground_z": 0.0,
        "drones": [
            {
                "id": f"uav{i}",
                "start_enu": [float(starts[i][0]), float(starts[i][1]), float(starts[i][2])],
                "goal_enu": [float(goals[i, 0]), float(goals[i, 1]), float(goals[i, 2])],
            }
            for i in range(n)
        ],
        "task": {"type": "goto", "duration_s": 5.0},
        "criteria": {"frozen": False, "body_radius_m": 0.2},
        "camera": {
            "width": 160,
            "height": 96,
            "hfov_deg": 90.0,
            "vfov_deg": 60.0,
            "min_range_m": 0.2,
            "max_range_m": 20.0,
            "noise_std_m": 0.02,
            "mount_xyz_body": [0.0, 0.0, 0.0],
        },
        "dynamics": {"model": "point_mass_lag", "tau_s": 0.1, "kp": 4.0, "kd": 3.0, "a_max": 6.0},
        "rates_hz": {"physics": 100, "camera": 30, "planner": 10},
    }
    (directory / "scene.json").write_text(json.dumps(scene), encoding="utf-8")


def test_default_is_off_and_predict_returns_none_even_with_neighbors():
    assert DEFAULT_ENABLED is False
    assert StudentConfig().enabled is False
    assert StudentConfig().n_neighbors == NEIGHBOR_K == 3
    model = MultiHeadStudent()
    depth = np.zeros((1, 2, 24, 40), dtype=np.float32)
    state = np.zeros((1, 9), dtype=np.float32)
    neigh = np.ones((1, 3, 6), dtype=np.float32)
    mask = np.ones((1, 3), dtype=np.float32)
    assert predict(model, depth, state, neigh, mask) is None
    assert predict(None, depth, state, neigh, mask) is None
    assert predict(model, depth, state, neigh, mask, enabled=True) is None
    with pytest.raises(RuntimeError, match="默认关"):
        model.forward(depth, state, neigh, mask)


def test_flight_loop_does_not_import_student():
    for rel in FLIGHT_FILES:
        text = (ROOT / rel).read_text(encoding="utf-8")
        assert "yopo_l0.student" not in text, rel
        assert "yopo_l0.distill" not in text, rel
        assert "pack_neighbor_features" not in text, rel
        assert "MultiHeadStudent" not in text, rel
    suite = (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    assert "distill" not in suite
    assert "d000-n3-headon-s0" not in suite


def test_pack_neighbor_features_pads_and_keeps_nearest():
    ego_p = np.zeros(3)
    ego_v = np.zeros(3)
    feat, mask = pack_neighbor_features(ego_p, ego_v, 0.0, np.zeros((0, 3)), np.zeros((0, 3)), k=3)
    assert feat.shape == (3, 6)
    assert mask.shape == (3,)
    np.testing.assert_array_equal(feat, 0)
    np.testing.assert_array_equal(mask, 0)

    # 六架在 +y 上，距离 1、2、4、5、8、10 m。最近三架应是前三个。
    starts = np.array([[0.0, d, 0.0] for d in (1.0, 2.0, 4.0, 5.0, 8.0, 10.0)])
    vels = np.array([[0.1 * (i + 1), 0.0, 0.0] for i in range(6)])
    feat, mask = pack_neighbor_features(ego_p, ego_v, 0.0, starts, vels, k=3)
    np.testing.assert_array_equal(mask, [1.0, 1.0, 1.0])
    np.testing.assert_allclose(feat[0, 0:3], [0.0, 1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(feat[1, 0:3], [0.0, 2.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(feat[2, 0:3], [0.0, 4.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(feat[0, 3:6], [0.1, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(feat[2, 3:6], [0.3, 0.0, 0.0], atol=1e-6)

    # 距离相同按下标。两架都在 3 m 处，先写下标 0。
    tied = np.array([[0.0, 3.0, 0.0], [0.0, -3.0, 0.0]])
    tied_v = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    feat, mask = pack_neighbor_features(ego_p, ego_v, 0.0, tied, tied_v, k=3)
    np.testing.assert_array_equal(mask, [1.0, 1.0, 0.0])
    np.testing.assert_allclose(feat[0, 3:6], [1.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(feat[1, 3:6], [2.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_array_equal(feat[2], 0)


def test_pack_neighbor_features_uses_body_frame():
    # 偏航 π/2 时，世界 +y 是机体 +x。相对速度同样旋转。
    ego_p = np.zeros(3)
    ego_v = np.array([0.0, 1.0, 0.0])
    neigh_p = np.array([[0.0, 3.0, 0.0]])
    neigh_v = np.array([[0.0, 2.0, 0.0]])
    feat, mask = pack_neighbor_features(ego_p, ego_v, np.pi / 2, neigh_p, neigh_v, k=3)
    np.testing.assert_array_equal(mask, [1.0, 0.0, 0.0])
    np.testing.assert_allclose(feat[0, 0:3], [3.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(feat[0, 3:6], [1.0, 0.0, 0.0], atol=1e-6)


def test_convert_two_drones_masks_unused_slots(tmp_path):
    run = tmp_path / "pair"
    _write_placed(run, [(0.0, 0.0, 1.0), (0.0, 3.0, 1.0)], yaw=0.0)
    batch = convert_run(run, max_windows=2)
    assert batch.n == 2
    assert batch.neighbor_rel.shape == (2, 3, 6)
    assert batch.neighbor_mask.shape == (2, 3)
    np.testing.assert_array_equal(batch.neighbor_mask[:, 0], 1)
    np.testing.assert_array_equal(batch.neighbor_mask[:, 1:], 0)
    # 两个视点都看见对方在自己的左侧或右侧 3 m，相对速度为 0。
    for row in range(2):
        np.testing.assert_allclose(np.abs(batch.neighbor_rel[row, 0, 0:3]), [0.0, 3.0, 0.0], atol=1e-5)
        np.testing.assert_allclose(batch.neighbor_rel[row, 0, 3:6], 0.0, atol=1e-5)
        np.testing.assert_array_equal(batch.neighbor_rel[row, 1:], 0)


def test_convert_yawed_pair_puts_neighbor_on_body_x(tmp_path):
    run = tmp_path / "yawed-pair"
    _write_placed(run, [(0.0, 0.0, 1.0), (0.0, 3.0, 1.0)], yaw=np.pi / 2)
    batch = convert_run(run, max_windows=2)
    ego0 = np.flatnonzero(batch.ego_index == 0)[0]
    np.testing.assert_allclose(batch.neighbor_rel[ego0, 0, 0:3], [3.0, 0.0, 0.0], atol=1e-5)


def test_convert_six_drones_keeps_three_nearest(tmp_path):
    starts = [(0.0, 0.0, 1.0)] + [(0.0, d, 1.0) for d in (1.0, 2.0, 4.0, 8.0, 16.0)]
    run = tmp_path / "six"
    _write_placed(run, starts)
    batch = convert_run(run, max_windows=6)
    # max_windows 按（时刻，视点）抽。第一拍六个视点会被均匀抽到。
    ego0 = [i for i, (t, e) in enumerate(zip(batch.sample_index, batch.ego_index)) if t == 0 and e == 0]
    assert ego0, (batch.sample_index, batch.ego_index)
    row = ego0[0]
    np.testing.assert_array_equal(batch.neighbor_mask[row], [1.0, 1.0, 1.0])
    np.testing.assert_allclose(batch.neighbor_rel[row, :, 1], [1.0, 2.0, 4.0], atol=1e-5)


def test_enabled_forward_has_neighbor_branch():
    import torch

    model = MultiHeadStudent(StudentConfig(enabled=True))
    depth = torch.zeros(2, 2, 24, 40)
    state = torch.zeros(2, 9)
    neighbors = torch.zeros(2, 3, 6)
    mask = torch.tensor([[1.0, 0.0, 0.0], [1.0, 1.0, 0.0]])
    neighbors[0, 0, 1] = 3.0
    position, velocity, yaw = model.forward(depth, state, neighbors, mask)
    assert tuple(position.shape) == (2, 8, 3)
    assert tuple(velocity.shape) == (2, 8, 3)
    assert tuple(yaw.shape) == (2, 8)
    named = {name for name, _ in model._module().named_parameters()}
    assert any(name.startswith("neighbor_fc") for name in named)
    # 缺位里的垃圾数乘掩码后必须是 0，前向结果不变。
    dirty = neighbors.clone()
    dirty[:, 2, :] = 99.0
    pos2, vel2, yaw2 = model.forward(depth, state, dirty, mask)
    torch.testing.assert_close(position, pos2)
    torch.testing.assert_close(velocity, vel2)
    torch.testing.assert_close(yaw, yaw2)
    out = predict(model, depth.numpy(), state.numpy(), neighbors.numpy(), mask.numpy())
    assert out is not None
    assert out.position_body.shape == (2, 8, 3)
    assert predict(model, depth.numpy(), state.numpy(), neighbors.numpy(), mask.numpy(), enabled=False) is None


def test_teacher_sample_three_drones_has_two_neighbors():
    batch = convert_run(SAMPLE_N3, max_windows=6)
    assert batch.n == 6
    assert batch.scenario_id == "d000-n3-headon-s0"
    assert batch.neighbor_rel.shape == (6, 3, NEIGHBOR_FEAT_DIM)
    # 三机记录每个视点有两架邻机，第三槽缺位。
    np.testing.assert_array_equal(batch.neighbor_mask[:, 0:2], 1)
    np.testing.assert_array_equal(batch.neighbor_mask[:, 2], 0)
    assert set(batch.ego_index.tolist()) <= {0, 1, 2}
    assert float(np.min(np.linalg.norm(batch.neighbor_rel[:, 0, 0:3], axis=1))) > 0.0


def test_convert_cli_on_three_drone_sample(tmp_path):
    import importlib.util

    script = ROOT / "scripts/distill/teacher_to_tensors.py"
    spec = importlib.util.spec_from_file_location("teacher_to_tensors_d4", script)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    out = tmp_path / "windows.npz"
    assert mod.main(["--run", str(SAMPLE_N3), "--out", str(out), "--max-windows", "3"]) == 0
    loaded = load_batch(out)
    assert loaded.n == 3
    assert loaded.meta()["tensor_schema"] == TENSOR_SCHEMA
    assert loaded.neighbor_rel.shape == (3, 3, 6)
    failed = tmp_path / "failed-n3"
    _write_run(failed, n_drones=3, success=False, with_scene=False)
    assert mod.main(["--run", str(failed), "--out", str(tmp_path / "no.npz")]) == 2
    assert not (tmp_path / "no.npz").exists()


def test_smoke_overfits_windows_with_neighbor_features():
    batch = convert_run(SAMPLE_N3, max_windows=4)
    report = train_smoke(
        batch.depth,
        batch.state,
        batch.position_body,
        batch.velocity_body,
        batch.yaw_rel,
        batch.neighbor_rel,
        batch.neighbor_mask,
        enabled=True,
        steps=200,
        seed=0,
        scenario_id=batch.scenario_id,
    )
    assert report.n_windows == 4
    assert report.scenario_id == "d000-n3-headon-s0"
    assert report.loss_final < report.loss_initial * 0.1
    assert report.position_mse_final < 1e-3
    assert report.overfit is True
