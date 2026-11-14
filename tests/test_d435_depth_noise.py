"""E-D435：视差噪声、遮挡阴影、延迟队列，以及关闭时与现路径逐位相同。"""

import hashlib
import json

import numpy as np
import pytest

from yopo_l0.depth_noise import (
    DEF,
    INVALID_NOTE,
    WIRED_INVALID,
    D435Rig,
    LatencyQueue,
    _box_same,
    apply_d435_depth,
    d435_depth,
    d435_from_task,
    fill_invalid,
    incidence_cos,
)
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import Box, SceneCard, load_card
from yopo_l0.sensor import CameraConfig, DepthCamera
from yopo_swarm_core.depth import network_depth

from conftest import ROOT

# 2 m、s=0.08、fx=426、B=0.05 m。σ_Z = Z²·s/(fx·B)。
SIGMA_2M_M = (2.0 ** 2) * 0.08 / (426.0 * 0.05)
FX = 426.0


def _robust(values: np.ndarray) -> float:
    med = np.median(values)
    return float(1.4826 * np.median(np.abs(values - med)))


def _flat_params(**overrides):
    params = dict(
        DEF,
        p_flying=0.0,
        lateral_px=0.0,
        p_hole=0.0,
        blob_holes=0,
        p_outlier=0.0,
        far_drop_z0=50.0,
        far_drop_z1=60.0,
        disp_offset=0.0,
    )
    params.update(overrides)
    return params


def test_box_blur_matches_convolve_when_kernel_fits():
    rng = np.random.default_rng(1)
    image = rng.standard_normal((30, 40))
    radius = 3
    kernel = np.ones(2 * radius + 1) / (2 * radius + 1)
    ref = np.apply_along_axis(lambda row: np.convolve(row, kernel, "same"), 1, image)
    ref = np.apply_along_axis(lambda col: np.convolve(col, kernel, "same"), 0, ref)
    got = _box_same(_box_same(image, radius, 1), radius, 0)
    np.testing.assert_allclose(got, ref, atol=1e-12)


def test_axial_noise_at_2m_matches_theory():
    """平壁、关掉飞点和空洞。RMS 对上 σ≈15 mm，容差 ±20%。"""
    image = np.full((480, 848), 2.0)
    intrinsics = (FX, FX, 424.0, 240.0)
    out = d435_depth(image, intrinsics, np.random.default_rng(0), params=_flat_params())
    center = out[40:-40, 40:-40]
    center = center[center > 0]
    assert center.size > 10000
    sigma = float(center.std())
    assert abs(sigma - SIGMA_2M_M) / SIGMA_2M_M < 0.20
    assert 0.012 < sigma < 0.018


def test_reference_self_check_pole_wall_and_shadow():
    """与调研脚本同一张图、同一套默认：1.5 m 柱、6 m 墙、柱左侧阴影。"""
    rng = np.random.default_rng(0)
    height, width = 480, 848
    ground = np.full((height, width), 6.0)
    ground[150:400, 400:520] = 1.5
    intrinsics = (FX, FX, width / 2, height / 2)
    out = d435_depth(ground, intrinsics, rng, params=dict(DEF))
    assert abs(float((out > 0).mean()) - 0.663) < 0.02
    pole = out[200:350, 430:500]
    pole = pole[pole > 0]
    wall = out[50:120, 600:800]
    wall = wall[wall > 0]
    assert abs(_robust(pole) - 1.5 ** 2 * 0.08 / (FX * 0.05)) / (1.5 ** 2 * 0.08 / (FX * 0.05)) < 0.20
    assert abs(_robust(wall) - 36 * 0.08 / (FX * 0.05)) / (36 * 0.08 / (FX * 0.05)) < 0.20
    clean = d435_depth(
        ground,
        intrinsics,
        np.random.default_rng(1),
        params=_flat_params(),
    )
    shadow = int((clean[300, 380:400] == 0).sum())
    theory = FX * 0.05 * (1 / 1.5 - 1 / 6)
    assert abs(shadow - theory) <= 2.0
    assert shadow == 10


def test_left_invalid_band_width_on_a_flat_wall():
    width = 848
    ground = np.full((64, width), 2.0)
    out = d435_depth(
        ground,
        (FX, FX, width / 2, 32.0),
        np.random.default_rng(0),
        params=_flat_params(),
        noise=False,
        holes=True,
    )
    band = FX * 0.05 / 2.0
    row = out[32]
    invalid = np.where(row <= 0)[0]
    assert invalid.size >= int(np.floor(band)) - 1
    assert invalid.max() < band + 2


def test_same_seed_is_deterministic_and_streams_do_not_cross():
    ground = np.full((48, 64), 2.0)
    intrinsics = (80.0, 48.0, 32.0, 24.0)
    a = apply_d435_depth(ground, intrinsics, np.random.default_rng(4), noise=True, holes=False)
    b = apply_d435_depth(ground, intrinsics, np.random.default_rng(4), noise=True, holes=False)
    assert np.array_equal(a, b, equal_nan=True)
    # 空洞用另一条流。噪声流不变时，没被挖掉的像素与只开噪声相同。
    params = _flat_params(p_outlier=0.0, p_hole=0.0, blob_holes=0, grazing_deg=90.0, far_drop_z0=50.0)
    noise_rng = np.random.default_rng(4)
    holes_rng = np.random.default_rng(9)
    noisy = d435_depth(ground, intrinsics, noise_rng, params=params, noise=True, holes=False)
    both = d435_depth(
        ground,
        intrinsics,
        np.random.default_rng(4),
        holes_rng=holes_rng,
        params=params,
        noise=True,
        holes=True,
        normals_cos=np.ones(ground.shape, np.float32),
    )
    kept = both > 0
    assert np.any(~kept)
    np.testing.assert_allclose(both[kept], noisy[kept], atol=1e-6)


def test_latency_queue_returns_newest_delivered_capture():
    queue = LatencyQueue(
        np.random.default_rng(0),
        mean_ms=20.0,
        jitter_ms=0.0,
        min_ms=20.0,
        p_spike=0.0,
        p_drop=0.0,
    )
    queue.push(0.0, "a", exposure_ms=0.0)
    assert queue.pop(0.0) is None
    assert queue.pop(0.020)[2] == "a"
    queue.push(1.0 / 30.0, "b", exposure_ms=0.0)
    assert queue.pop(1.0 / 30.0)[2] == "a"
    assert queue.pop(0.020 + 1.0 / 30.0)[2] == "b"
    # 后捕获的帧先到：已经送达的最新捕获盖过更早的帧。
    late = LatencyQueue(np.random.default_rng(1), mean_ms=0.0, jitter_ms=0.0, min_ms=0.0, p_spike=0.0, p_drop=0.0)
    late.q.append((0.10, 0.0, "old"))
    late.q.append((0.04, 0.03, "new"))
    assert late.pop(0.05)[2] == "new"
    assert late.pop(0.20)[2] == "new"
    dropped = LatencyQueue(np.random.default_rng(2), p_drop=1.0)
    dropped.push(0.0, "gone", exposure_ms=0.0)
    assert dropped.pop(1.0) is None


def test_invalid_fill_nan_inpaint_and_max_range():
    depth = np.full((5, 5), 2.0, np.float32)
    depth[2, 2] = 0.0
    nan = fill_invalid(depth, "nan", 20.0)
    assert np.isnan(nan[2, 2]) and nan[0, 0] == pytest.approx(2.0)
    filled = fill_invalid(depth, "inpaint", 20.0)
    assert filled[2, 2] == pytest.approx(2.0)
    assert np.isfinite(filled).all()
    far = fill_invalid(depth, "max_range", 20.0)
    assert far[2, 2] == pytest.approx(20.0)
    raw = np.full((96, 160), 2.0, np.float32)
    raw[40, 80] = np.nan
    wired = network_depth(raw)
    assert wired.shape == (1, 1, 96, 160)
    assert float(wired[0, 0, 40, 80]) > 0.05
    assert WIRED_INVALID == "nan"
    assert "network_depth" in INVALID_NOTE


def test_flat_wall_incidence_is_not_grazing():
    cosine = incidence_cos(np.full((96, 160), 2.0), 80.0, 48.0, 80.0, 48.0)
    # 90° 相机看正对的墙，中心接近正入射。视场边缘的入射角仍小于 75° 掠射阈值。
    assert float(cosine[48, 80]) > 0.95
    assert float(cosine.min()) > 0.45


def test_task_switch_defaults_off_and_rejects_bad_values():
    assert d435_from_task(None).active is False
    assert d435_from_task({}).active is False
    assert d435_from_task({"d435_depth": False}).active is False
    assert d435_from_task({"d435_depth": {"enabled": False, "noise": True}}).active is False
    full = d435_from_task({"d435_depth": True})
    assert full.active and full.noise and full.holes and full.latency and full.invalid == "nan"
    stress = d435_from_task({
        "d435_depth": {"enabled": True, "subpix": 0.20, "latency_ms": 50, "holes": False},
    })
    assert stress.active and stress.subpix == pytest.approx(0.20)
    assert stress.latency_ms == pytest.approx(50.0) and stress.holes is False
    with pytest.raises(ValueError):
        d435_from_task({"d435_depth": "true"})
    with pytest.raises(ValueError):
        d435_from_task({"d435_depth": {"enabled": True, "invalid": "zero"}})


def test_spawn_after_generator_does_not_change_the_gaussian_stream():
    parent = np.random.SeedSequence(0)
    first = np.random.default_rng(parent)
    parent.spawn(3)
    drawn = first.normal(size=6)
    again = np.random.default_rng(np.random.SeedSequence(0))
    np.testing.assert_array_equal(drawn, again.normal(size=6))


def test_latency_holds_the_previous_frame_until_it_is_due():
    wall = (Box("w", (4.0, -10.0, -10.0), (4.2, 10.0, 10.0)),)
    cfg = CameraConfig(width=32, height=24, noise_std_m=0.0, hfov_deg=90.0, vfov_deg=60.0)
    spec = d435_from_task({
        "d435_depth": {
            "enabled": True,
            "noise": False,
            "holes": False,
            "latency": True,
            "latency_ms": 50.0,
            "p_drop": 0.0,
        },
    })
    rig = D435Rig.from_seed(spec, np.random.SeedSequence(3))
    cam = DepthCamera(cfg, wall, np.random.default_rng(3), d435=rig)
    first = cam.capture(np.zeros(3), 0.0, 0.0)
    assert np.isnan(first.depth).all()
    assert first.stamp == 0.0
    second = cam.capture(np.zeros(3), 0.0, 0.05)
    assert second.stamp == pytest.approx(0.0)
    center = second.depth[12, 16]
    assert center == pytest.approx(4.0, abs=1e-4)


def _enabled(card, value):
    raw = dict(card.card)
    task = dict(raw["task"])
    task["d435_depth"] = value
    raw["task"] = task
    return SceneCard(card.path, raw, card.obstacles)


def _strip(result: dict) -> dict:
    drop = set(WALL_FIELDS) | {"commit", "scene_card_hash"}
    return {k: v for k, v in result.items() if k not in drop}


def _tick_hash(ticks) -> str:
    blob = json.dumps(ticks, ensure_ascii=False, separators=(",", ":")).encode()
    return hashlib.sha256(blob).hexdigest()


@pytest.mark.parametrize(
    "path",
    ["scenes/g2-pillar.json", "scenes/open-3drone.json"],
)
def test_switch_off_matches_default_bitwise(path):
    """单机 g2-pillar 与三机 open-3drone，seed 0。去掉墙钟、commit 和卡哈希。"""
    card = load_card(ROOT / path)
    cfg = RunConfig(seed=0, policy="upstream", shadow=True, duration_s=0.3, record_planner_inputs=True)
    default = run(card, cfg)
    explicit = run(_enabled(card, False), cfg)
    assert "d435_depth" not in default.result
    assert "d435_depth" not in explicit.result
    assert default.result.get("upstream_deviation") is False
    assert _strip(default.result) == _strip(explicit.result)
    assert default.planner_inputs == explicit.planner_inputs
    assert _tick_hash(default.ticks) == _tick_hash(explicit.ticks)


def test_switch_on_marks_upstream_deviation_and_changes_depth(tmp_path):
    card = load_card(ROOT / "scenes/g2-pillar.json")
    cfg = RunConfig(seed=0, policy="upstream", shadow=False, duration_s=0.3, record_planner_inputs=True)
    off = run(card, cfg)
    on_card = _enabled(card, {"enabled": True, "noise": True, "holes": False, "latency": False})
    on = run(on_card, cfg)
    assert on.result["upstream_deviation"] is True
    block = on.result["d435_depth"]
    assert block["noise"] is True and block["holes"] is False and block["invalid_fill"] == "nan"
    assert "NaN" in block["invalid_fill_note"]
    assert off.planner_inputs != on.planner_inputs
