"""票 D3：单机多头连续学生默认关，教师窗口能转，少量成功样本能过拟合。

不跑飞行环，不需要 GPU，不需要 192 个场景。
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from yopo_l0.distill import TENSOR_SCHEMA, DistillError, convert_run, load_batch, save_batch
from yopo_l0.policy import POLICIES
from yopo_l0.student import (
    DEFAULT_ENABLED,
    MultiHeadStudent,
    StudentConfig,
    predict,
    train_smoke,
)

from conftest import ROOT

FIXTURE = ROOT / "tests/fixtures/d3/d014-n1-headon-s0"
FLIGHT_FILES = (
    "yopo_l0/__init__.py",
    "yopo_l0/runner.py",
    "yopo_l0/policy.py",
    "yopo_l0/crossing.py",
    "yopo_l0/run.py",
    "yopo_l0/ros2_run.py",
    "yopo_l0/ros2_nodes.py",
    "yopo_l0/formation.py",
    "yopo_l0/switch.py",
    "yopo_l0/geom_fallback.py",
    "yopo_l0/net.py",
)


def _write_run(
    directory: Path,
    *,
    n_drones: int = 1,
    success=True,
    yaw: float = 0.0,
    schema: str = "yopo-teacher-record/1",
    duration_s: float = 2.0,
    speed: float = 2.0,
    with_scene: bool = True,
):
    directory.mkdir(parents=True, exist_ok=True)
    times = np.arange(0.0, duration_s + 1e-9, 0.1, dtype=np.float64)
    heading = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    origin = np.zeros(3)
    pos = origin + times[:, None] * speed * heading
    vel = np.repeat((speed * heading)[None, :], times.size, axis=0)
    acc = np.zeros_like(pos)
    yaw_row = np.full(times.size, yaw, dtype=np.float64)
    goal = origin + heading * 20.0
    position = np.repeat(pos[:, None, :], n_drones, axis=1)
    velocity = np.repeat(vel[:, None, :], n_drones, axis=1)
    acceleration = np.repeat(acc[:, None, :], n_drones, axis=1)
    yaw_all = np.repeat(yaw_row[:, None], n_drones, axis=1)
    goals = np.repeat(goal[None, :], n_drones, axis=0)
    if n_drones > 1:
        position[:, 1, 1] += 3.0
        goals[1, 1] += 3.0
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
        "schema": schema,
        "scenario": {"id": directory.name, "n_drones": n_drones},
        "metrics": {"success": success},
        "run": {"status": "arrived" if success is True else "stopped"},
    }
    (directory / "record.json").write_text(json.dumps(record), encoding="utf-8")
    if with_scene:
        (directory / "geometry.json").write_text(json.dumps({"boxes": [], "cylinders": []}), encoding="utf-8")
        scene = {
            "name": directory.name,
            "geometry": "geometry.json",
            "ground_z": 0.0,
            "drones": [
                {
                    "id": f"uav{i}",
                    "start_enu": [0.0, 3.0 if i == 1 else 0.0, 1.0],
                    "goal_enu": [float(goals[i, 0]), float(goals[i, 1]), float(goals[i, 2])],
                }
                for i in range(n_drones)
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


def test_student_defaults_off_and_predict_does_not_run():
    assert DEFAULT_ENABLED is False
    assert StudentConfig().enabled is False
    model = MultiHeadStudent()
    assert model.cfg.enabled is False
    depth = np.zeros((1, 2, 24, 40), dtype=np.float32)
    state = np.zeros((1, 9), dtype=np.float32)
    assert predict(model, depth, state) is None
    assert predict(None, depth, state) is None
    assert predict(model, depth, state, enabled=True) is None
    with pytest.raises(RuntimeError, match="默认关"):
        model.forward(depth, state)


def test_train_smoke_refuses_when_disabled():
    depth = np.zeros((1, 2, 24, 40), dtype=np.float32)
    state = np.zeros((1, 9), dtype=np.float32)
    pos = np.zeros((1, 8, 3), dtype=np.float32)
    yaw = np.zeros((1, 8), dtype=np.float32)
    with pytest.raises(RuntimeError, match="enabled=True"):
        train_smoke(depth, state, pos, pos, yaw)


def test_flight_loop_default_is_still_upstream():
    from yopo_l0.runner import RunConfig

    assert set(POLICIES) >= {"hover", "upstream", "certified"}
    assert RunConfig().policy == "upstream"
    # 学生策略是显式选项。除了策略注册和接入点，飞行环源文件仍不点蒸馏管线的名。
    allowed = {"yopo_l0/policy.py", "yopo_l0/runner.py", "yopo_l0/run.py"}
    for rel in FLIGHT_FILES:
        text = (ROOT / rel).read_text(encoding="utf-8")
        assert "yopo_l0.distill" not in text, rel
        if rel not in allowed:
            assert "yopo_l0.student" not in text, rel
            assert "MultiHeadStudent" not in text, rel
    suite = (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    assert "d014-n1-headon-s0" not in suite
    assert "distill" not in suite


def test_enabled_forward_is_one_continuous_trajectory_not_a_score_lattice():
    import torch

    model = MultiHeadStudent(StudentConfig(enabled=True))
    depth = torch.zeros(2, 2, 24, 40)
    state = torch.zeros(2, 9)
    position, velocity, yaw = model.forward(depth, state)
    assert tuple(position.shape) == (2, 8, 3)
    assert tuple(velocity.shape) == (2, 8, 3)
    assert tuple(yaw.shape) == (2, 8)
    # 没有分数头，输出里也没有候选维。
    named = {name for name, _ in model._module().named_parameters()}
    assert any(name.startswith("position_head") for name in named)
    assert any(name.startswith("velocity_head") for name in named)
    assert any(name.startswith("yaw_head") for name in named)
    assert not any("score" in name for name in named)
    out = predict(model, depth.numpy(), state.numpy())
    assert out is not None
    assert out.position_body.shape == (2, 8, 3)
    assert predict(model, depth.numpy(), state.numpy(), enabled=False) is None


def test_straight_flight_maps_into_the_body_frame(tmp_path):
    run = tmp_path / "straight"
    _write_run(run, yaw=0.0)
    batch = convert_run(run, max_windows=1)
    assert batch.n == 1
    assert batch.skipped is None
    assert batch.scenario_id == "straight"
    np.testing.assert_allclose(batch.state[0, 0:3], [2.0, 0.0, 0.0], atol=1e-5)
    np.testing.assert_allclose(batch.state[0, 3:6], 0.0, atol=1e-5)
    np.testing.assert_allclose(batch.state[0, 6:9], [20.0, 0.0, 0.0], atol=1e-5)
    np.testing.assert_allclose(batch.position_body[0, 0], [0.4, 0.0, 0.0], atol=1e-5)
    np.testing.assert_allclose(batch.velocity_body[0, 0], [2.0, 0.0, 0.0], atol=1e-5)
    np.testing.assert_allclose(batch.yaw_rel[0], 0.0, atol=1e-5)
    assert batch.neighbor_rel.shape == (1, 3, 6)
    assert batch.neighbor_mask.shape == (1, 3)
    np.testing.assert_array_equal(batch.neighbor_mask, 0)
    np.testing.assert_array_equal(batch.neighbor_rel, 0)
    np.testing.assert_array_equal(batch.ego_index, [0])
    # 卡上写了 0.02 的噪声。转换强制为 0，同一姿态两次深度相同，空场景没有回波。
    again = convert_run(run, max_windows=1)
    np.testing.assert_array_equal(batch.depth, again.depth)
    assert batch.depth.shape == (1, 2, 24, 40)
    assert float(batch.depth[0, 1].sum()) == 0.0


def test_yawed_flight_puts_forward_motion_on_body_x(tmp_path):
    run = tmp_path / "yawed"
    _write_run(run, yaw=np.pi / 2)
    batch = convert_run(run, max_windows=1)
    np.testing.assert_allclose(batch.state[0, 0:3], [2.0, 0.0, 0.0], atol=1e-5)
    np.testing.assert_allclose(batch.state[0, 6:9], [20.0, 0.0, 0.0], atol=1e-5)
    np.testing.assert_allclose(batch.position_body[0, 0], [0.4, 0.0, 0.0], atol=1e-5)


def test_failed_record_yields_no_windows(tmp_path):
    run = tmp_path / "failed"
    _write_run(run, success=False, with_scene=False)
    batch = convert_run(run)
    assert batch.n == 0
    assert batch.skipped == "success_not_true"
    unknown = tmp_path / "unknown"
    _write_run(unknown, success=None, with_scene=False)
    assert convert_run(unknown).skipped == "success_not_true"


def test_multi_drone_record_is_no_longer_rejected_here(tmp_path):
    # 形状和最近邻选择在 tests/test_d4_neighbour.py。这里只钉住：成功的双机记录能出窗口。
    run = tmp_path / "two"
    _write_run(run, n_drones=2)
    batch = convert_run(run)
    assert batch.n > 0
    assert set(batch.ego_index.tolist()) == {0, 1}
    assert batch.neighbor_mask.shape[1] == 3
    # 每个视点恰好一架邻机，另外两个槽位是缺位。
    np.testing.assert_array_equal(batch.neighbor_mask[:, 0], 1)
    np.testing.assert_array_equal(batch.neighbor_mask[:, 1:], 0)


def test_wrong_schema_is_rejected(tmp_path):
    run = tmp_path / "bad-schema"
    _write_run(run, schema="other", with_scene=False)
    with pytest.raises(DistillError, match="schema"):
        convert_run(run)


def test_fixture_is_one_successful_single_drone_sample():
    record = json.loads((FIXTURE / "record.json").read_text(encoding="utf-8"))
    assert record["schema"] == "yopo-teacher-record/1"
    assert record["scenario"]["id"] == "d014-n1-headon-s0"
    assert record["scenario"]["n_drones"] == 1
    assert record["metrics"]["success"] is True
    assert record["metrics"]["min_obstacle_surface_m"] > 0.20
    assert record["bsplines"] == []
    batch = convert_run(FIXTURE, max_windows=4)
    assert batch.n == 4
    assert batch.scenario_id == "d014-n1-headon-s0"
    assert batch.depth.shape == (4, 2, 24, 40)
    assert batch.state.shape == (4, 9)
    assert batch.position_body.shape == (4, 8, 3)
    assert batch.neighbor_rel.shape == (4, 3, 6)
    np.testing.assert_array_equal(batch.neighbor_mask, 0)
    # 高度 1 m，相机会看到地面或圆柱。四个窗口里应有回波。
    assert float(batch.depth[:, 1].sum()) > 0.0


def test_batch_roundtrip(tmp_path):
    batch = convert_run(FIXTURE, max_windows=2)
    path = save_batch(batch, tmp_path / "windows.npz")
    loaded = load_batch(path)
    assert loaded.n == batch.n
    assert loaded.scenario_id == batch.scenario_id
    assert loaded.meta()["tensor_schema"] == TENSOR_SCHEMA
    np.testing.assert_allclose(loaded.position_body, batch.position_body)
    np.testing.assert_allclose(loaded.depth, batch.depth)
    np.testing.assert_allclose(loaded.state, batch.state)
    np.testing.assert_allclose(loaded.neighbor_rel, batch.neighbor_rel)
    np.testing.assert_allclose(loaded.neighbor_mask, batch.neighbor_mask)
    np.testing.assert_array_equal(loaded.ego_index, batch.ego_index)


def test_convert_cli_on_the_fixture(tmp_path):
    import importlib.util

    script = ROOT / "scripts/distill/teacher_to_tensors.py"
    spec = importlib.util.spec_from_file_location("teacher_to_tensors", script)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    out = tmp_path / "windows.npz"
    assert mod.main(["--run", str(FIXTURE), "--out", str(out), "--max-windows", "2"]) == 0
    assert load_batch(out).n == 2
    failed = tmp_path / "failed-run"
    _write_run(failed, success=False, with_scene=False)
    assert mod.main(["--run", str(failed), "--out", str(tmp_path / "no.npz")]) == 2
    assert not (tmp_path / "no.npz").exists()


def test_smoke_overfits_a_few_windows_from_the_real_sample():
    batch = convert_run(FIXTURE, max_windows=4)
    report = train_smoke(
        batch.depth,
        batch.state,
        batch.position_body,
        batch.velocity_body,
        batch.yaw_rel,
        enabled=True,
        steps=200,
        seed=0,
        scenario_id=batch.scenario_id,
    )
    assert report.n_windows == 4
    assert report.scenario_id == "d014-n1-headon-s0"
    assert report.loss_final < report.loss_initial * 0.1
    assert report.position_mse_final < 1e-3
    assert report.overfit is True
