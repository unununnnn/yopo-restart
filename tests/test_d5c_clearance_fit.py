"""票 D5c：L0 机体半径净空过滤和贴边 / 末段加权。默认关，关时训练损失与 D5/D5b 同一条路径。"""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from yopo_l0.student import trajectory_loss
from yopo_l0.student_fit import (
    FitConfig,
    read_window_clearance,
    train_from_paths,
    uses_unweighted_d5_loss,
    weighted_trajectory_loss,
    window_weights,
)

from conftest import ROOT

SCRIPT = ROOT / "scripts" / "distill" / "d5c" / "filter_and_fit.py"


def _d5c():
    spec = importlib.util.spec_from_file_location("d5c_filter_and_fit", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


D5C = _d5c()


def _heads(n=2, steps=8, seed=0):
    g = torch.Generator().manual_seed(seed)
    mk = lambda *shape: torch.randn(*shape, generator=g)
    return mk(n, steps, 3), mk(n, steps, 3), mk(n, steps), mk(n, steps, 3), mk(n, steps, 3), mk(n, steps)


class _Batch:
    def __init__(self, depth_m, valid, max_range=20.0):
        self.n = len(depth_m)
        self.max_range_m = max_range
        depth = np.zeros((self.n, 2, 4, 4), np.float32)
        for i, (meters, ok) in enumerate(zip(depth_m, valid)):
            depth[i, 0] = meters / max_range
            depth[i, 1] = 1.0 if ok else 0.0
        self.depth = depth


def _write_windows(path: Path, clearance: np.ndarray | None, *, extra: dict | None = None, n: int | None = None):
    if clearance is None:
        count = int(n if n is not None else 1)
    else:
        count = int(np.asarray(clearance).shape[0])
    meta = {
        "tensor_schema": "yopo-student-window/2",
        "scenario_id": "d5c-test",
        "horizon_dt_s": 0.2,
        "max_range_m": 20.0,
        "n_neighbors": 3,
        "skipped": None,
    }
    arrays = {
        "depth": np.zeros((count, 2, 24, 40), np.float32),
        "state": np.zeros((count, 9), np.float32),
        "position_body": np.zeros((count, 8, 3), np.float32),
        "velocity_body": np.zeros((count, 8, 3), np.float32),
        "yaw_rel": np.zeros((count, 8), np.float32),
        "neighbor_rel": np.zeros((count, 3, 6), np.float32),
        "neighbor_mask": np.zeros((count, 3), np.float32),
        "time_s": np.zeros(count, np.float64),
        "sample_index": np.arange(count, dtype=np.int64),
        "ego_index": np.zeros(count, np.int64),
        "meta": np.asarray(json.dumps(meta)),
    }
    if clearance is not None:
        arrays["min_surface_clearance_m"] = np.asarray(clearance)
    if extra:
        arrays.update(extra)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **arrays)


def _scene(tmp: Path, cylinders: list[dict], *, ground_z: float | None = 0.0) -> Path:
    geom = {"boxes": [], "cylinders": cylinders}
    (tmp / "geom.json").write_text(json.dumps(geom), encoding="utf-8")
    card = {"name": "d5c-test", "geometry": "geom.json"}
    if ground_z is not None:
        card["ground_z"] = ground_z
    path = tmp / "scene.json"
    path.write_text(json.dumps(card), encoding="utf-8")
    return path


def _fit(tmp: Path, train: list[Path], hold: list[Path], cfg: FitConfig):
    return train_from_paths(train, hold, tmp / "run", cfg=cfg)


def test_fitconfig_defaults_match_d5_and_d5b():
    cfg = FitConfig()
    assert (
        cfg.lr,
        cfg.batch_size,
        cfg.epochs,
        cfg.seed,
        cfg.num_threads,
        cfg.loss_weights,
        cfg.log_every,
        cfg.planar,
        cfg.near_weight,
        cfg.near_depth_m,
        cfg.shuffle_batches,
    ) == (1e-3, 64, 5, 0, 2, (1.0, 0.25, 0.05), 50, False, 1.0, 1.2, 8)
    explicit = FitConfig(
        min_clearance_m=None,
        clearance_soft_margin_m=0.0,
        clearance_low_weight=1.0,
        endpoint_weight=0.0,
        clearance_missing="keep",
    )
    assert cfg == explicit
    assert uses_unweighted_d5_loss(cfg)
    assert uses_unweighted_d5_loss(FitConfig(min_clearance_m=0.0))
    assert not uses_unweighted_d5_loss(FitConfig(planar=True))
    assert not uses_unweighted_d5_loss(FitConfig(near_weight=3.0))
    assert not uses_unweighted_d5_loss(FitConfig(endpoint_weight=1.0))
    assert not uses_unweighted_d5_loss(FitConfig(clearance_soft_margin_m=0.05, clearance_low_weight=0.25))
    assert uses_unweighted_d5_loss(FitConfig(clearance_soft_margin_m=0.05, clearance_low_weight=1.0))
    with pytest.raises(ValueError, match="clearance_missing"):
        FitConfig(clearance_missing="drop")


def test_endpoint_weight_zero_matches_the_old_loss_and_positive_hits_the_back_half():
    position, velocity, yaw, position_hat, velocity_hat, yaw_hat = _heads()
    ref, ref_pos, _, _ = trajectory_loss(position, velocity, yaw, position_hat, velocity_hat, yaw_hat)
    plain, plain_pos = weighted_trajectory_loss(
        position, velocity, yaw, position_hat, velocity_hat, yaw_hat, torch.ones(2), endpoint_weight=0.0
    )
    omitted, omitted_pos = weighted_trajectory_loss(
        position, velocity, yaw, position_hat, velocity_hat, yaw_hat, torch.ones(2)
    )
    assert float(plain) == float(omitted)
    assert float(plain_pos) == float(omitted_pos)
    assert float(plain) == pytest.approx(float(ref), rel=1e-6)
    assert float(plain_pos) == pytest.approx(float(ref_pos), rel=1e-6)

    zeros = torch.zeros(2, 8, 3)
    yaw0 = torch.zeros(2, 8)
    back = zeros.clone()
    back[:, -1, 0] = 1.0
    front = zeros.clone()
    front[:, 0, 0] = 1.0
    loss_back, _ = weighted_trajectory_loss(
        zeros, zeros, yaw0, back, zeros, yaw0, torch.ones(2), endpoint_weight=3.0
    )
    loss_front, _ = weighted_trajectory_loss(
        zeros, zeros, yaw0, front, zeros, yaw0, torch.ones(2), endpoint_weight=3.0
    )
    # 后一半步权重是 4，前一半是 1。步权重和是 20，位置维是 3。末点误差 1 的损失是 4/60。
    assert float(loss_back) == pytest.approx(4.0 / 60.0)
    assert float(loss_front) == pytest.approx(1.0 / 60.0)
    assert float(loss_back) > float(loss_front)
    same_back, _ = weighted_trajectory_loss(
        zeros, zeros, yaw0, back, zeros, yaw0, torch.ones(2), endpoint_weight=0.0
    )
    same_front, _ = weighted_trajectory_loss(
        zeros, zeros, yaw0, front, zeros, yaw0, torch.ones(2), endpoint_weight=0.0
    )
    assert float(same_back) == float(same_front)
    z_only = back.clone()
    z_only[:, -1, 0] = 0.0
    z_only[:, -1, 2] = 1.0
    planar, _ = weighted_trajectory_loss(
        zeros, zeros, yaw0, z_only, zeros, yaw0, torch.ones(2), planar=True, endpoint_weight=3.0
    )
    assert float(planar) == 0.0


def test_clearance_low_weight_downs_the_near_band_and_defaults_do_not():
    batch = _Batch([0.8, 5.0, 0.5], [True, True, False])
    clearance = np.array([0.02, 0.05, -0.1])
    cfg = FitConfig(
        min_clearance_m=0.0,
        clearance_soft_margin_m=0.05,
        clearance_low_weight=0.25,
        near_weight=3.0,
        near_depth_m=1.2,
    )
    # 文件权重 2。近障只打中第一窗（0.8 m）。贴边打中 0.02 和 -0.1，0.05 刚好在带外。
    got = window_weights(batch, 2.0, cfg, clearance)
    np.testing.assert_allclose(got, [2.0 * 3.0 * 0.25, 2.0, 2.0 * 0.25])
    np.testing.assert_allclose(
        window_weights(batch, 1.0, FitConfig(), clearance=np.array([-1.0, 0.0, 0.01])),
        [1.0, 1.0, 1.0],
    )
    no_floor = window_weights(
        batch,
        1.0,
        FitConfig(clearance_soft_margin_m=0.05, clearance_low_weight=0.25),
        clearance,
    )
    np.testing.assert_allclose(no_floor, [0.25, 1.0, 0.25])


def test_read_clearance_prefers_min_surface_and_reduces_the_horizon(tmp_path):
    path = tmp_path / "both.npz"
    _write_windows(
        path,
        np.array([1.0, 2.0]),
        extra={"clearance": np.array([9.0, 9.0])},
    )
    np.testing.assert_allclose(read_window_clearance(path), [1.0, 2.0])
    horizon = tmp_path / "horizon.npz"
    per_step = np.array([[0.4, 0.1, 0.3], [0.2, 0.8, 0.7]], dtype=np.float64)
    _write_windows(horizon, None, n=2, extra={"clearance": per_step})
    np.testing.assert_allclose(read_window_clearance(horizon), [0.1, 0.2])


def test_default_training_ignores_stored_clearance_and_stays_on_trajectory_loss(tmp_path, monkeypatch):
    import yopo_l0.student_fit as fit_mod

    train = tmp_path / "train.npz"
    hold = tmp_path / "hold.npz"
    _write_windows(train, np.array([-0.2, -0.01, 1.0]))
    _write_windows(hold, np.array([-0.5]))
    called = {"weighted": 0, "plain": 0}
    real_weighted = fit_mod.weighted_trajectory_loss
    real_plain = fit_mod.trajectory_loss

    def wrap_weighted(*args, **kwargs):
        called["weighted"] += 1
        return real_weighted(*args, **kwargs)

    def wrap_plain(*args, **kwargs):
        called["plain"] += 1
        return real_plain(*args, **kwargs)

    monkeypatch.setattr(fit_mod, "weighted_trajectory_loss", wrap_weighted)
    monkeypatch.setattr(fit_mod, "trajectory_loss", wrap_plain)
    summary = _fit(tmp_path, [train], [hold], FitConfig(epochs=1, batch_size=8, num_threads=1))
    assert called["weighted"] == 0
    assert called["plain"] > 0
    assert summary["history"][-1]["n_train_windows_seen"] == 3
    assert summary["holdout"]["n_windows"] == 1


def test_min_clearance_filters_train_and_holdout(tmp_path):
    train = tmp_path / "train.npz"
    hold = tmp_path / "hold.npz"
    _write_windows(train, np.array([-0.05, 0.0, 0.02, 1.15]))
    _write_windows(hold, np.array([-0.01, 0.4]))
    summary = _fit(
        tmp_path,
        [train],
        [hold],
        FitConfig(epochs=1, batch_size=8, num_threads=1, min_clearance_m=0.0),
    )
    assert summary["history"][-1]["n_train_windows_seen"] == 3
    assert summary["holdout"]["n_windows"] == 1
    assert summary["fit"]["min_clearance_m"] == 0.0
    assert uses_unweighted_d5_loss(FitConfig(min_clearance_m=0.0))


def test_missing_clearance_keep_skip_and_error(tmp_path):
    import yopo_l0.student_fit as fit_mod

    bare = tmp_path / "bare.npz"
    safe = tmp_path / "safe.npz"
    hold = tmp_path / "hold.npz"
    _write_windows(bare, None, n=2)
    _write_windows(safe, np.array([0.4]))
    _write_windows(hold, np.array([0.4]))
    kept = _fit(
        tmp_path / "keep",
        [bare],
        [hold],
        FitConfig(epochs=1, batch_size=8, num_threads=1, min_clearance_m=0.0, clearance_missing="keep"),
    )
    assert kept["history"][-1]["n_train_windows_seen"] == 2
    skipped = _fit(
        tmp_path / "skip",
        [bare, safe],
        [hold],
        FitConfig(epochs=1, batch_size=8, num_threads=1, min_clearance_m=0.0, clearance_missing="skip"),
    )
    assert skipped["history"][-1]["n_train_windows_seen"] == 1
    with pytest.raises(ValueError, match="min_surface_clearance_m"):
        fit_mod._prepare_clearance(bare, 2, FitConfig(min_clearance_m=0.0, clearance_missing="error"))
    with pytest.raises(ValueError, match="clearance"):
        _fit(
            tmp_path / "err",
            [bare],
            [hold],
            FitConfig(epochs=1, batch_size=8, num_threads=1, min_clearance_m=0.0, clearance_missing="error"),
        )


def test_surface_distance_matches_scene_objects():
    from yopo_l0.policy import R_yaw
    from yopo_l0.scene import Box, Cylinder, Ground

    cylinder = Cylinder("tree", (1.0, 0.0), 0.25, 0.0, 3.0)
    box = Box("pillar", (0.0, 0.0, 0.0), (1.0, 1.0, 2.0))
    ground = Ground(0.0)
    lean = Cylinder("lean", (0.0, 0.0), 0.2, 0.0, 2.0, lean_xy=(0.4, 0.0))
    points = np.array(
        [
            [1.0, 0.0, 1.5],
            [1.6, 0.0, 1.5],
            [1.0, 0.0, 4.0],
            [2.0, 2.0, 0.5],
            [0.2, 0.2, 1.0],
            [-0.5, 0.3, 1.0],
        ],
        dtype=np.float64,
    )
    obstacles = (cylinder, box, ground, lean)
    got = D5C.surface_rows(points, obstacles)
    for i, point in enumerate(points):
        expect = min(ob.distance(point) for ob in obstacles)
        assert got[i] == pytest.approx(expect, abs=1e-12)
    origin = np.array([[1.0, 2.0, 1.5]])
    yaw = np.array([np.pi / 2])
    body = np.array([[[1.0, 0.0, 0.0]]])
    world = D5C.body_to_world(origin, yaw, body)
    np.testing.assert_allclose(world[0, 0], origin[0] + R_yaw(float(yaw[0])) @ body[0, 0])


def test_filter_recomputes_l0_radius_and_does_not_touch_inputs(tmp_path):
    scene = _scene(
        tmp_path,
        [{"name": "tree", "center_xy": [0.05, 0.0], "radius": 0.02, "z_lo": 0.0, "z_hi": 3.0}],
    )
    # 0.1 s 那一拍贴进细圆柱，0.2 s 的结点在 5 m 外。只看结点会漏掉这一下。
    times = np.arange(0.0, 2.0 + 1e-9, 0.1)
    position = np.zeros((times.size, 1, 3), dtype=np.float64)
    position[:, 0, 0] = 5.0
    position[:, 0, 2] = 1.5
    hit = int(np.where(np.isclose(times, 0.1))[0][0])
    position[hit, 0, 0] = 0.05
    traj = tmp_path / "trajectory.npz"
    np.savez(traj, time_s=times, position_m=position)

    graze = tmp_path / "graze.npz"
    _write_windows(graze, None, n=1, extra={"time_s": np.array([0.0]), "ego_index": np.array([0])})
    far = tmp_path / "far.npz"
    point = np.array([5.0, 5.0, 1.5])
    _write_windows(
        far,
        None,
        n=1,
        extra={"position_world": np.repeat(point.reshape(1, 1, 3), 8, axis=1)},
    )
    before = {p: p.read_bytes() for p in (graze, far, scene)}
    report = D5C.filter_entries(
        [
            {"id": "graze", "path": graze, "scene": scene, "trajectory": traj},
            {"id": "far", "path": far, "scene": scene, "trajectory": None},
        ],
        body_radius_m=0.35,
        margin_m=0.0,
        horizon_s=1.6,
    )
    after = {p: p.read_bytes() for p in before}
    assert after == before
    assert report["inputs_modified"] is False
    assert report["report_only"] is True
    assert report["body_radius_m"] == 0.35
    assert report["threshold_surface_m"] == pytest.approx(0.35)
    by_id = {row["id"]: row for row in report["files"]}
    assert by_id["graze"]["source"] == "trajectory"
    assert by_id["graze"]["n_dropped"] == 1
    assert by_id["graze"]["min_clearance_m"] == pytest.approx(-0.35, abs=1e-9)
    assert by_id["far"]["source"] == "position_world"
    assert by_id["far"]["n_kept"] == 1
    assert by_id["far"]["min_clearance_m"] == pytest.approx(1.15, abs=1e-9)
    assert report["n_dropped"] == 1
    assert report["n_kept"] == 1
    samples = D5C.horizon_samples(times, position[:, 0, :], 0.0, 1.6)
    assert np.any(np.isclose(samples[:, 0], 0.05))
    assert float(samples[:, 0].min()) < 0.2


def test_smoke_cli_runs():
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), "--smoke"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=180,
        check=False,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    payload = json.loads(proc.stdout)
    assert payload["report_only"] is True
    assert payload["smoke"] is True
    assert payload["inputs_modified"] is False
    assert payload["max_recompute_error_m"] < 1e-9
    assert payload["filter"]["n_dropped"] == 1
    assert payload["filter"]["n_kept"] == 3
    assert payload["filter"]["body_radius_m"] == 0.35
    assert payload["filter"]["inputs_modified"] is False
    assert payload["fit"]["planar"] is True
    assert payload["fit"]["epochs_done"] == 1
    assert payload["fit"]["n_train_windows_seen"] == 2
    assert payload["fit"]["holdout_n"] == 1
    assert payload["fit"]["min_clearance_m"] == 0.0
    assert payload["fit"]["clearance_low_weight"] == 0.25
    assert payload["fit"]["endpoint_weight"] == 1.0
