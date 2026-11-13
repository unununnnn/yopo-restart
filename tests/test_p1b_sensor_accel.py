"""P1b：深度相机加速默认关闭。打开后与逐障碍路径在深度和闭环上一致。"""

import numpy as np
import pytest

from conftest import ROOT
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import Box, Cylinder, Ground, load_card
from yopo_l0.sensor import (
    R_BODY_CAM,
    CameraConfig,
    DepthCamera,
    _ray_dirs_cam,
    render,
    sensor_accel_enabled,
)

MIXED = (
    Box("wall", (4.0, -2.0, 0.0), (4.3, 2.0, 3.0)),
    Box("ceil", (-2.0, -2.0, 3.2), (6.0, 2.0, 3.5)),
    Box("near", (0.05, -0.3, 0.6), (0.2, 0.3, 1.8)),
    Box("far", (30.0, -5.0, -1.0), (30.4, 5.0, 6.0)),
    Cylinder("up-a", (3.0, 0.4), 0.3, 0.0, 3.0),
    Cylinder("up-b", (2.2, -0.7), 0.5, 0.0, 2.4),
    Cylinder("lean", (3.4, 1.0), 0.25, 0.0, 3.0, (0.45, -0.15)),
    Ground(0.0),
)

POSES = (
    (np.array([0.0, 0.0, 1.2]), 0.0),
    (np.array([0.6, -0.3, 1.5]), 0.35),
    (np.array([1.4, 0.8, 2.1]), -0.7),
    (np.array([2.2, -1.1, 0.45]), 1.2),
    (np.array([-0.4, 1.6, 2.8]), 2.2),
)


def assert_same_depth(a, b):
    a = np.asarray(a)
    b = np.asarray(b)
    assert a.shape == b.shape and a.dtype == b.dtype
    assert np.array_equal(np.isnan(a), np.isnan(b))
    finite = np.isfinite(a)
    if np.any(finite):
        delta = float(np.max(np.abs(a[finite].astype(np.float64) - b[finite].astype(np.float64))))
        assert delta <= 1e-5
        assert np.array_equal(a[finite], b[finite])


def _fresh_dirs(cfg: CameraConfig) -> np.ndarray:
    fx, fy, cx, cy = cfg.intrinsics
    u, v = np.meshgrid(np.arange(cfg.width) + 0.5, np.arange(cfg.height) + 0.5)
    return np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=-1)


def _capture_seq(cfg, obstacles, seed):
    """调用前设好 ``YOPO_SENSOR_ACCEL``。返回各帧深度，以及之后再抽的一段正态数。"""
    rng = np.random.default_rng(seed)
    cam = DepthCamera(cfg, obstacles, rng)
    frames = [cam.capture(pos, yaw, stamp=i * 0.1).depth for i, (pos, yaw) in enumerate(POSES)]
    return frames, rng.normal(size=6)


def test_sensor_accel_defaults_off(monkeypatch):
    monkeypatch.delenv("YOPO_SENSOR_ACCEL", raising=False)
    assert sensor_accel_enabled() is False
    monkeypatch.setenv("YOPO_SENSOR_ACCEL", "0")
    assert sensor_accel_enabled() is False
    monkeypatch.setenv("YOPO_SENSOR_ACCEL", "1")
    assert sensor_accel_enabled() is True
    monkeypatch.setenv("YOPO_SENSOR_ACCEL", "true")
    assert sensor_accel_enabled() is False


def test_ray_dirs_cached_readonly_and_match_formula():
    cfg = CameraConfig(width=40, height=24, hfov_deg=80.0, vfov_deg=50.0)
    cached = _ray_dirs_cam(cfg)
    assert cached is _ray_dirs_cam(cfg)
    assert cached.flags.writeable is False
    np.testing.assert_array_equal(cached, _fresh_dirs(cfg))
    with pytest.raises(ValueError):
        cached[0, 0, 0] = 0.0


def test_off_matches_fresh_formula(monkeypatch):
    """关掉时深度等于当场重建射线、再逐障碍求交。缓存不改数值。"""
    monkeypatch.delenv("YOPO_SENSOR_ACCEL", raising=False)
    cfg = CameraConfig(width=64, height=48, noise_std_m=0.04, min_range_m=0.2, max_range_m=7.0)
    cam = DepthCamera(cfg, MIXED, np.random.default_rng(3))
    got = cam.capture(np.array([0.4, -0.2, 1.3]), 0.4, 1.5)
    dirs = _fresh_dirs(cfg)
    yaw = 0.4
    c, s = np.cos(yaw), np.sin(yaw)
    R_wb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    cam_pos = np.array([0.4, -0.2, 1.3]) + R_wb @ np.asarray(cfg.mount_xyz_body)
    d = dirs @ (R_wb @ R_BODY_CAM).T
    from yopo_l0.sensor import _hit_box, _hit_cylinder, _hit_ground

    best = np.full(d.shape[:2], np.inf)
    for ob in MIXED:
        if isinstance(ob, Box):
            t = _hit_box(cam_pos, d, ob)
        elif isinstance(ob, Cylinder):
            t = _hit_cylinder(cam_pos, d, ob)
        else:
            t = _hit_ground(cam_pos, d, ob)
        best = np.minimum(best, t)
    rng = np.random.default_rng(3)
    noise = rng.normal(0.0, cfg.noise_std_m, best.shape)
    depth = best + noise
    depth[~np.isfinite(best) | (best > cfg.max_range_m) | (best < cfg.min_range_m)] = np.nan
    assert got.stamp == 1.5
    assert_same_depth(got.depth, depth.astype(np.float32))


def test_on_matches_off_without_noise_over_frames(monkeypatch):
    cfg = CameraConfig(width=96, height=64, noise_std_m=0.0, min_range_m=0.2, max_range_m=6.0)
    monkeypatch.setenv("YOPO_SENSOR_ACCEL", "0")
    off, tail_off = _capture_seq(cfg, MIXED, 11)
    monkeypatch.setenv("YOPO_SENSOR_ACCEL", "1")
    on, tail_on = _capture_seq(cfg, MIXED, 11)
    assert len(off) == len(POSES)
    for a, b in zip(off, on, strict=True):
        assert_same_depth(a, b)
    # 无噪声时不抽 rng，两边剩下的序列相同。
    np.testing.assert_array_equal(tail_off, tail_on)
    # 第一帧贴着近处盒子，回波都低于 min_range，整幅是 NaN。后面的帧又有回波又有空像素。
    assert np.isnan(off[0]).all()
    assert np.isfinite(np.stack(off[1:])).any()
    assert np.isnan(np.stack(off[1:])).any()


def test_on_matches_off_with_noise_same_seed(monkeypatch):
    cfg = CameraConfig(width=80, height=48, noise_std_m=0.05, min_range_m=0.2, max_range_m=8.0)
    monkeypatch.setenv("YOPO_SENSOR_ACCEL", "0")
    off, tail_off = _capture_seq(cfg, MIXED, 19)
    monkeypatch.setenv("YOPO_SENSOR_ACCEL", "1")
    on, tail_on = _capture_seq(cfg, MIXED, 19)
    for a, b in zip(off, on, strict=True):
        assert_same_depth(a, b)
        assert np.array_equal(np.isnan(a), np.isnan(b))
    np.testing.assert_array_equal(tail_off, tail_on)
    # 同 seed 的两帧噪声不是同一张图，说明每帧都抽了一次。
    assert not np.array_equal(off[0], off[1])


def test_pitched_rays_and_scene_cards_match(monkeypatch):
    """斜相机和场景卡上的单帧：无噪声误差为 0，NaN 掩码一致。"""
    rng = np.random.default_rng(4)
    cfg = CameraConfig(width=72, height=40, noise_std_m=0.0, min_range_m=0.15, max_range_m=9.0)
    for _ in range(8):
        pos = rng.uniform([-1.0, -2.0, 0.3], [5.0, 2.0, 3.2])
        yaw = float(rng.uniform(-np.pi, np.pi))
        pitch = float(rng.uniform(-0.5, 0.5))
        c, s = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        R_wb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        R_pitch = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        R = R_wb @ R_pitch @ R_BODY_CAM
        monkeypatch.setenv("YOPO_SENSOR_ACCEL", "0")
        off = render(MIXED, cfg, pos, R, np.random.default_rng(0), 0.0).depth
        monkeypatch.setenv("YOPO_SENSOR_ACCEL", "1")
        on = render(MIXED, cfg, pos, R, np.random.default_rng(0), 0.0).depth
        assert_same_depth(off, on)

    for path, pos, yaw in (
        (ROOT / "scenes/g2-pillar.json", np.array([0.0, 0.0, 1.5]), 0.15),
        (ROOT / "scenes/crossing-6-forest.json", np.array([20.0, 0.0, 1.5]), 0.0),
        (ROOT / "scenes/crossing-forest.json", np.array([12.0, -1.0, 1.6]), -0.4),
    ):
        card = load_card(path)
        quiet = CameraConfig.from_card({**card.card["camera"], "noise_std_m": 0.0})
        noisy = CameraConfig.from_card(card.card["camera"])
        for cfg_i, seed in ((quiet, 2), (noisy, 8)):
            monkeypatch.setenv("YOPO_SENSOR_ACCEL", "0")
            quiet_rng = np.random.default_rng(seed)
            a = DepthCamera(cfg_i, card.obstacles, quiet_rng).capture(pos, yaw, 0.0)
            monkeypatch.setenv("YOPO_SENSOR_ACCEL", "1")
            b = DepthCamera(cfg_i, card.obstacles, np.random.default_rng(seed)).capture(pos, yaw, 0.0)
            assert_same_depth(a.depth, b.depth)


def _trajectory(card):
    out = run(card, RunConfig(seed=0))
    pos = np.asarray([row["p"] for row in out.ticks], dtype=np.float64)
    vel = np.asarray([row["v"] for row in out.ticks], dtype=np.float64)
    codes = [row["plan_code"] for row in out.ticks]
    return out.result, pos, vel, codes


def _assert_same_run(off, on):
    result_off, pos_off, vel_off, codes_off = off
    result_on, pos_on, vel_on, codes_on = on
    assert result_on["outcome"] == result_off["outcome"]
    assert result_on["arrival_time_s"] == result_off["arrival_time_s"]
    assert result_on["ticks"] == result_off["ticks"]
    assert result_on["frames"] == result_off["frames"]
    np.testing.assert_array_equal(pos_on, pos_off)
    np.testing.assert_array_equal(vel_on, vel_off)
    assert codes_on == codes_off


def test_closed_loop_g2_pillar_matches(monkeypatch):
    card = load_card(ROOT / "scenes/g2-pillar.json")
    monkeypatch.setenv("YOPO_SENSOR_ACCEL", "0")
    off = _trajectory(card)
    monkeypatch.setenv("YOPO_SENSOR_ACCEL", "1")
    on = _trajectory(card)
    _assert_same_run(off, on)


def test_closed_loop_crossing_forest_matches(monkeypatch):
    card = load_card(ROOT / "scenes/crossing-forest.json")
    monkeypatch.setenv("YOPO_SENSOR_ACCEL", "0")
    off = _trajectory(card)
    monkeypatch.setenv("YOPO_SENSOR_ACCEL", "1")
    on = _trajectory(card)
    _assert_same_run(off, on)
