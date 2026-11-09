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
    if not c.upright():
        return _hit_cylinder_lean(o, d, c)
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


def _hit_cylinder_lean(o, d, c: Cylinder):
    """有限圆柱，轴线可以从底面圆心斜到顶面圆心。竖直圆柱不走这里。"""
    a, b = c.axis_ends()
    ab = b - a
    length = float(np.linalg.norm(ab))
    axis = ab / length
    origin = np.asarray(o, dtype=float)
    oc = origin - a
    d_ax = np.sum(d * axis, axis=-1)
    d_perp = d - d_ax[..., None] * axis
    oc_ax = float(np.dot(oc, axis))
    oc_perp = oc - oc_ax * axis
    A = np.sum(d_perp * d_perp, axis=-1)
    B = 2.0 * np.sum(oc_perp * d_perp, axis=-1)
    C = float(np.dot(oc_perp, oc_perp) - c.radius ** 2)
    disc = B * B - 4.0 * A * C
    with np.errstate(divide="ignore", invalid="ignore"):
        sq = np.sqrt(np.maximum(disc, 0.0))
        denom = np.where(A > 1e-12, 2.0 * A, np.inf)
        t_near = np.where(disc >= 0.0, (-B - sq) / denom, np.inf)
        t_far = np.where(disc >= 0.0, (-B + sq) / denom, np.inf)

    def on_side(t):
        axial = oc_ax + t * d_ax
        return (t > 0.0) & (axial >= -1e-6) & (axial <= length + 1e-6)

    best = np.where(on_side(t_near), t_near, np.inf)
    best = np.minimum(best, np.where(on_side(t_far), t_far, np.inf))
    for end, outward in ((a, -axis), (b, axis)):
        denom = np.sum(d * outward, axis=-1)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.dot(end - origin, outward) / np.where(np.abs(denom) > 1e-12, denom, np.inf)
        hit = origin + t[..., None] * d
        radial = hit - end
        radial = radial - np.sum(radial * axis, axis=-1, keepdims=True) * axis
        dist = np.linalg.norm(radial, axis=-1)
        best = np.minimum(best, np.where((t > 0.0) & (dist <= c.radius + 1e-7), t, np.inf))
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
