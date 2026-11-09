"""光线投射深度相机。相机系：x 右、y 下、z 前；世界系 ENU。深度为 z 深度（米），无回波为 NaN。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .scene import Box, Cylinder, Ground, Obstacle

# 机体 FLU → 相机 optical（右、下、前）
R_BODY_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


@dataclass(frozen=True)
class CameraConfig:
    width: int = 160
    height: int = 96
    hfov_deg: float = 90.0
    vfov_deg: float = 60.0
    min_range_m: float = 0.2
    max_range_m: float = 20.0
    noise_std_m: float = 0.0
    mount_xyz_body: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @classmethod
    def from_card(cls, d: dict) -> "CameraConfig":
        d = dict(d)
        if "mount_xyz_body" in d:
            d["mount_xyz_body"] = tuple(d["mount_xyz_body"])
        return cls(**d)

    @property
    def intrinsics(self) -> tuple[float, float, float, float]:
        fx = (self.width / 2) / np.tan(np.radians(self.hfov_deg) / 2)
        fy = (self.height / 2) / np.tan(np.radians(self.vfov_deg) / 2)
        return float(fx), float(fy), self.width / 2, self.height / 2


@dataclass(frozen=True)
class DepthFrame:
    stamp: float  # 仿真时钟
    depth: np.ndarray  # (H, W) float32
    K: tuple[float, float, float, float]  # fx, fy, cx, cy


def _ray_dirs_cam(cfg: CameraConfig) -> np.ndarray:
    fx, fy, cx, cy = cfg.intrinsics
    u, v = np.meshgrid(np.arange(cfg.width) + 0.5, np.arange(cfg.height) + 0.5)
    return np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=-1)  # z 分量 = 1


def _hit_box(o, d, b: Box):
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (np.asarray(b.lo) - o) / d
        t2 = (np.asarray(b.hi) - o) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
    hit = (tmax >= tmin) & (tmin > 0)
    return np.where(hit, tmin, np.inf)


def _hit_cylinder(o, d, c: Cylinder):
    ox, oy = o[0] - c.center_xy[0], o[1] - c.center_xy[1]
    dx, dy = d[..., 0], d[..., 1]
    a = dx * dx + dy * dy
    b = 2 * (ox * dx + oy * dy)
    cc = ox * ox + oy * oy - c.radius**2
    disc = b * b - 4 * a * cc
    with np.errstate(divide="ignore", invalid="ignore"):
        sq = np.sqrt(np.maximum(disc, 0.0))
        t_side = np.where((disc >= 0) & (a > 0), (-b - sq) / (2 * a), np.inf)
        t_side = np.where(t_side > 0, t_side, np.inf)
        z_side = o[2] + t_side * d[..., 2]
        t_side = np.where((z_side >= c.z_lo) & (z_side <= c.z_hi), t_side, np.inf)
        best = t_side
        for zc in (c.z_lo, c.z_hi):  # 顶面 / 底面
            t = (zc - o[2]) / d[..., 2]
            px, py = ox + t * dx, oy + t * dy
            ok = (t > 0) & (px * px + py * py <= c.radius**2)
            best = np.minimum(best, np.where(ok, t, np.inf))
    return best


def _hit_ground(o, d, g: Ground):
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (g.z - o[2]) / d[..., 2]
    return np.where(t > 0, t, np.inf)


def render(
    obstacles: tuple[Obstacle, ...], cfg: CameraConfig, cam_pos: np.ndarray, R_world_cam: np.ndarray,
    rng: np.random.Generator, stamp: float,
) -> DepthFrame:
    d = _ray_dirs_cam(cfg) @ R_world_cam.T  # 世界系方向，参数 t 即相机 z 深度
    best = np.full(d.shape[:2], np.inf)
    for ob in obstacles:
        if isinstance(ob, Box):
            t = _hit_box(cam_pos, d, ob)
        elif isinstance(ob, Cylinder):
            t = _hit_cylinder(cam_pos, d, ob)
        else:
            t = _hit_ground(cam_pos, d, ob)
        best = np.minimum(best, t)
    # 噪声每帧整幅抽取，抽取次数与几何无关，保证同 seed 同序列
    noise = rng.normal(0.0, cfg.noise_std_m, best.shape) if cfg.noise_std_m > 0 else 0.0
    depth = best + noise
    depth[~np.isfinite(best) | (best > cfg.max_range_m) | (best < cfg.min_range_m)] = np.nan
    return DepthFrame(stamp, depth.astype(np.float32), cfg.intrinsics)


class DepthCamera:
    def __init__(self, cfg: CameraConfig, obstacles: tuple[Obstacle, ...], rng: np.random.Generator):
        self.cfg, self._obstacles, self._rng = cfg, obstacles, rng

    def capture(self, position: np.ndarray, yaw: float, stamp: float) -> DepthFrame:
        c, s = np.cos(yaw), np.sin(yaw)
        R_wb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        cam_pos = np.asarray(position) + R_wb @ np.asarray(self.cfg.mount_xyz_body)
        return render(self._obstacles, self.cfg, cam_pos, R_wb @ R_BODY_CAM, self._rng, stamp)
