"""深度相机行为：已知几何给出已知深度；时间戳来自仿真时钟。"""

import numpy as np

from yopo_l0.scene import Box, Cylinder
from yopo_l0.sensor import CameraConfig, DepthCamera


def center(depth):
    h, w = depth.shape
    return depth[h // 2 - 1 : h // 2 + 1, w // 2 - 1 : w // 2 + 1]


def test_box_wall_depth_and_stamp():
    wall = (Box("w", (5.0, -10.0, -10.0), (5.2, 10.0, 10.0)),)
    cam = DepthCamera(CameraConfig(), wall, np.random.default_rng(0))
    f = cam.capture(np.zeros(3), 0.0, stamp=1.25)
    assert f.stamp == 1.25 and f.depth.shape == (96, 160)
    np.testing.assert_allclose(f.depth, 5.0, atol=1e-5)  # 平面墙：z 深度处处 5 m


def test_cylinder_range_fov_and_resolution():
    cyl = (Cylinder("c", (4.0, 0.0), 0.5, 0.0, 3.0),)
    cfg = CameraConfig(width=64, height=48, hfov_deg=60.0, vfov_deg=45.0, max_range_m=10.0)
    f = DepthCamera(cfg, cyl, np.random.default_rng(0)).capture(np.array([0.0, 0.0, 1.5]), 0.0, 0.0)
    assert f.depth.shape == (48, 64)
    np.testing.assert_allclose(center(f.depth), 3.5, atol=0.01)
    assert np.isnan(f.depth[:, 0]).all()  # 视场边缘没有回波
    near = CameraConfig(max_range_m=3.0)
    g = DepthCamera(near, cyl, np.random.default_rng(0)).capture(np.array([0.0, 0.0, 1.5]), 0.0, 0.0)
    assert np.isnan(g.depth).all()  # 超量程


def test_yaw_turns_camera():
    wall = (Box("w", (-0.1, 4.0, -10.0), (0.1, 4.2, 10.0)),)  # 左侧 4 m 外的墙（窄）
    cam = DepthCamera(CameraConfig(), wall, np.random.default_rng(0))
    assert np.isnan(center(cam.capture(np.zeros(3), 0.0, 0.0).depth)).all()
    np.testing.assert_allclose(center(cam.capture(np.zeros(3), np.pi / 2, 0.0).depth), 4.0, atol=1e-4)


def test_noise_is_seeded():
    wall = (Box("w", (5.0, -10.0, -10.0), (5.2, 10.0, 10.0)),)
    cfg = CameraConfig(noise_std_m=0.05)
    a = DepthCamera(cfg, wall, np.random.default_rng(7)).capture(np.zeros(3), 0.0, 0.0).depth
    b = DepthCamera(cfg, wall, np.random.default_rng(7)).capture(np.zeros(3), 0.0, 0.0).depth
    c = DepthCamera(cfg, wall, np.random.default_rng(8)).capture(np.zeros(3), 0.0, 0.0).depth
    assert np.array_equal(a, b) and not np.array_equal(a, c)
    assert 0.03 < float(np.std(a - 5.0)) < 0.07
