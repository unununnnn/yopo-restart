"""#7：原始深度 → 局部自由空间 + 扫掠体判定（B05 双管线）。"""

from __future__ import annotations

import numpy as np
import pytest

from yopo_swarm_core.freespace import (
    CameraIntrinsics,
    DepthFrame,
    FreeSpace,
    FreeSpaceConfig,
    SpaceVerdict,
)
from yopo_swarm_core.synthetic import Box, camera_pose_looking_x, render_depth
from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment

K = CameraIntrinsics(fx=100.0, fy=100.0, cx=80.0, cy=60.0, width=160, height=120)
POSE = camera_pose_looking_x(position=(0.0, 0.0, 0.0))
WALL = Box((15.0, -30.0, -30.0), (16.0, 30.0, 30.0))
BLOCK = Box((5.0, -1.0, -1.0), (6.0, 1.0, 1.0))
CFG = FreeSpaceConfig(
    body_radius=0.2, margin=0.1, max_age=0.5, min_range=0.3, max_range=30.0, sample_spacing=0.05
)


def line(p0, p1, t0=0.0, T=2.0):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    c = np.column_stack([p0, (p1 - p0) / T])
    return PiecewisePolynomial(t0, [PolySegment(c, T)])


def frame(boxes=(WALL, BLOCK), stamp=0.0, depth=None, mask=None):
    d = render_depth(boxes, K, POSE, max_range=CFG.max_range) if depth is None else depth
    return (
        DepthFrame.from_raw(d, K, POSE, stamp, CFG)
        if mask is None
        else DepthFrame(d, mask, K, POSE, stamp)
    )


def space(*frames):
    fs = FreeSpace(CFG)
    for f in frames:
        fs.add(f)
    return fs


def test_renderer_sees_block_and_wall():
    d = render_depth((WALL, BLOCK), K, POSE, max_range=30.0)
    assert d[60, 80] == pytest.approx(5.0)
    assert d[5, 5] == pytest.approx(15.0, rel=1e-6)


def test_open_space_in_view_is_free():
    r = space(frame()).classify_sweep(line((1, 0, 0), (3.5, 0, 0)), now=0.1)
    assert r.verdict is SpaceVerdict.FREE


def test_candidate_through_obstacle_is_occupied():
    r = space(frame()).classify_sweep(line((1, 0, 0), (7, 0, 0)), now=0.1)
    assert r.verdict is SpaceVerdict.OCCUPIED
    assert r.first_bad_time is not None


def test_candidate_behind_obstacle_is_unknown():
    r = space(frame()).classify_sweep(line((8, -0.3, 0), (8, 0.3, 0)), now=0.1)
    assert r.verdict is SpaceVerdict.UNKNOWN


def test_outside_field_of_view_is_unknown():
    r = space(frame()).classify_sweep(line((2, 5, 0), (3, 5, 0)), now=0.1)
    assert r.verdict is SpaceVerdict.UNKNOWN


def test_invalid_pixels_do_not_create_free_space_and_filled_values_are_ignored():
    raw = render_depth((WALL, BLOCK), K, POSE, max_range=30.0)
    raw[50:70, 70:90] = np.nan  # 中心一块无回波
    raw[0:10, 0:10] = 0.05  # 低于最小距离
    f = DepthFrame.from_raw(raw, K, POSE, 0.0, CFG)
    assert not f.valid[60, 80] and not f.valid[5, 5]
    r = space(f).classify_sweep(line((1, 0, 0), (3.5, 0, 0)), now=0.1)
    assert r.verdict is SpaceVerdict.UNKNOWN
    # 网络输入用的补洞图：洞里填了“很远”。即使把补洞值塞进来，掩码为 False 的像素也不被采用
    filled = np.where(np.isfinite(raw), raw, 50.0)
    g = DepthFrame(filled, f.valid, K, POSE, 0.0)
    assert (
        space(g).classify_sweep(line((1, 0, 0), (3.5, 0, 0)), now=0.1).verdict
        is SpaceVerdict.UNKNOWN
    )


def test_mask_cannot_mark_non_finite_or_out_of_range_pixels_valid():
    raw = render_depth((WALL,), K, POSE, max_range=30.0)
    raw[0, 0] = np.nan
    with pytest.raises(ValueError):
        DepthFrame(raw, np.ones_like(raw, dtype=bool), K, POSE, 0.0)


def test_expired_observation_is_ignored():
    f = frame(stamp=0.0)
    path = line((1, 0, 0), (3.5, 0, 0))
    assert space(f).classify_sweep(path, now=0.3).verdict is SpaceVerdict.FREE
    assert space(f).classify_sweep(path, now=1.0).verdict is SpaceVerdict.UNKNOWN


def test_rolling_window_keeps_only_recent_frames():
    fs = FreeSpace(FreeSpaceConfig(max_frames=2, max_age=10.0))
    for s in range(5):
        fs.add(frame(stamp=float(s)))
    assert [f.stamp for f in fs.frames] == [3.0, 4.0]


def test_swept_body_edge_collision_is_not_safe():
    """中心线离柱子 0.15 m：点判定安全，但半径 0.2 的机体扫掠体判不安全。"""
    pillar = Box((3.0, 0.15, -2.0), (3.2, 0.35, 2.0))
    f = frame(boxes=(WALL, pillar))
    path = line((1, 0, 0), (2.9, 0, 0), T=1.0).with_validity(0.0, 1.0)
    through = line((1, 0, 0), (4.5, 0, 0))
    thin = FreeSpace(FreeSpaceConfig(body_radius=0.01, margin=0.0, sample_spacing=0.02))
    thin.add(f)
    assert thin.classify_sweep(path, now=0.1).verdict is SpaceVerdict.FREE
    assert thin.classify_sweep(through, now=0.1).verdict is SpaceVerdict.FREE  # 中心线不碰
    r = space(f).classify_sweep(through, now=0.1)  # 机体半径 0.2 + 余量 0.1
    assert r.verdict is SpaceVerdict.OCCUPIED


def test_partially_unknown_path_is_never_free():
    r = space(frame()).classify_sweep(line((1, 0, 0), (2.0, 5.0, 0)), now=0.1)
    assert r.verdict is SpaceVerdict.UNKNOWN


def test_no_frames_means_unknown():
    assert (
        FreeSpace(CFG).classify_sweep(line((1, 0, 0), (2, 0, 0)), now=0.0).verdict
        is SpaceVerdict.UNKNOWN
    )


def test_occupied_anywhere_beats_free_elsewhere():
    """多帧：任一有效帧看到障碍即不算自由。"""
    f1 = frame(boxes=(WALL,), stamp=0.0)
    f2 = frame(boxes=(WALL, BLOCK), stamp=0.1)
    r = space(f1, f2).classify_sweep(line((1, 0, 0), (7, 0, 0)), now=0.2)
    assert r.verdict is SpaceVerdict.OCCUPIED


@pytest.mark.parametrize("seed", range(60))
def test_random_paths_free_verdict_is_sound_against_ground_truth(seed):
    """属性：判 FREE 的路径，其扫掠体与场景真值障碍不相交。"""
    rng = np.random.default_rng(seed)
    boxes = [WALL]
    for _ in range(4):
        lo = rng.uniform([2, -3, -2], [10, 3, 2])
        boxes.append(Box(tuple(lo), tuple(lo + rng.uniform(0.2, 1.5, 3))))
    fs = space(frame(boxes=tuple(boxes)))
    p0 = rng.uniform([0.8, -1, -1], [3, 1, 1])
    p1 = rng.uniform([2, -3, -2], [12, 3, 2])
    path = line(p0, p1)
    r = fs.classify_sweep(path, now=0.1)
    if r.verdict is SpaceVerdict.FREE:
        R = CFG.body_radius + CFG.margin
        for t in np.linspace(path.t_start, path.t_end, 400):
            p = path.evaluate(t)
            for b in boxes:
                assert b.distance(p) > R - 1e-9


# --- 证书（#9 用：已批准计划的扫掠体在静态环境假设下保持空闲） -----------------


def test_certificate_covers_near_camera_region_that_frames_cannot_see():
    fs = FreeSpace(CFG)
    fs.add(frame(stamp=0.0))
    near = np.array([0.1, 0.0, 0.0])  # 在 min_range 内：单帧永远 UNKNOWN
    assert fs.classify_point(near, CFG.sample_radius, 0.1) is SpaceVerdict.UNKNOWN
    fs.assume_free(np.zeros(3), 1.0, "takeoff")
    assert fs.classify_point(near, CFG.sample_radius, 0.1) is SpaceVerdict.FREE


def test_certificate_never_overrides_fresh_occupied_evidence():
    fs = FreeSpace(CFG)
    fs.assume_free(np.array([5.0, 0.0, 0.0]), 3.0, "wrong-assumption")
    fs.add(frame(stamp=0.0))
    assert fs.classify_point(np.array([4.9, 0.0, 0.0]), 0.3, 0.1) is SpaceVerdict.OCCUPIED


def test_certify_sweep_only_records_balls_free_in_frames_with_slack():
    fs = FreeSpace(CFG)
    fs.add(frame(stamp=0.0))
    n = fs.certify_sweep(line((1.0, 0, 0), (3.0, 0, 0)), now=0.1, slack=0.2, label="p1")
    assert n > 0
    fs2 = FreeSpace(CFG)
    fs2.add(frame(stamp=0.0))
    assert fs2.certify_sweep(line((3.0, 0, 0), (5.8, 0, 0)), now=0.1, slack=0.2, label="p2") > 0
    fs2.frames.clear()
    assert fs2.classify_point(np.array([3.2, 0, 0]), CFG.sample_radius, 0.1) is SpaceVerdict.FREE
    assert fs2.classify_point(np.array([5.6, 0, 0]), CFG.sample_radius, 0.1) is SpaceVerdict.UNKNOWN
    # 证书半径带 slack：中心偏离样本 ≤ slack 的同尺寸球被覆盖
    fs.frames.clear()
    p = np.array([2.0, 0.15, 0.0])
    assert fs.classify_point(p, CFG.sample_radius, 0.1) is SpaceVerdict.FREE
    assert (
        fs.classify_point(np.array([2.0, 0.5, 0.0]), CFG.sample_radius, 0.1) is SpaceVerdict.UNKNOWN
    )


def test_rolling_certificates_are_bounded():
    fs = FreeSpace(FreeSpaceConfig(max_certificate_sets=2))
    for i in range(4):
        fs.add_certificate(np.array([[float(i), 0, 0]]), 1.0, f"c{i}")
    assert [c.label for c in fs.certificates] == ["c2", "c3"]
