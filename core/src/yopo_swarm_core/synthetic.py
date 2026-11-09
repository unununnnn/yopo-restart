"""合成场景：轴对齐盒子障碍 + 针孔相机深度渲染（测试与离线纵切用，ROS-free）。

相机系：z 前、x 右、y 下。世界系：ENU（x 前、y 左、z 上）。深度图存 z 深度（米）；
无回波像素为 NaN。
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

# 相机系 → 世界系（相机看 +x）：cam x = -world y，cam y = -world z，cam z = world x
_R_WORLD_CAM_X = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


@dataclass(frozen=True)
class Box:
    lo: tuple[float, float, float]
    hi: tuple[float, float, float]

    def distance(self, p: np.ndarray) -> float:
        """点到盒子的欧氏距离（盒内为 0）。"""
        lo, hi = np.asarray(self.lo), np.asarray(self.hi)
        d = np.maximum(np.maximum(lo - p, 0.0), p - hi)
        return float(np.linalg.norm(d))


def camera_pose_looking_x(position: Sequence[float], yaw: float = 0.0) -> np.ndarray:
    """4x4 T_world_cam：相机在 position，光轴沿世界 +x 绕 z 转 yaw。"""
    c, s = np.cos(yaw), np.sin(yaw)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    T = np.eye(4)
    T[:3, :3] = rz @ _R_WORLD_CAM_X
    T[:3, 3] = position
    return T


def render_depth(
    boxes: Sequence[Box], intrinsics, T_world_cam: np.ndarray, max_range: float
) -> np.ndarray:
    k = intrinsics
    u, v = np.meshgrid(np.arange(k.width) + 0.5, np.arange(k.height) + 0.5)
    dirs_cam = np.stack([(u - k.cx) / k.fx, (v - k.cy) / k.fy, np.ones_like(u)], axis=-1)  # z=1
    R, o = T_world_cam[:3, :3], T_world_cam[:3, 3]
    dirs = dirs_cam @ R.T
    best = np.full(u.shape, np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for b in boxes:
            lo, hi = np.asarray(b.lo), np.asarray(b.hi)
            t1 = (lo - o) / dirs
            t2 = (hi - o) / dirs
            tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
            tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
            hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0)
            best = np.where(hit & (tmin < best), tmin, best)
    depth = best  # 方向向量 z 分量为 1，所以参数 t 即 z 深度
    depth[~np.isfinite(depth) | (depth > max_range)] = np.nan
    return depth


__all__ = ["Box", "ScriptedLatticeSource", "camera_pose_looking_x", "render_depth"]


class ScriptedLatticeSource:
    """离线测试用的候选源：不看深度、不知道障碍，只按网格方向给出 15 条“停在 3 m 外”的候选。

    输出格式与网络一致（图像顺序 traj_params (N,14)、score (N,)、radius (N,2nr)），
    分数 = 1 + cos(尾点方向, 机体系目标方向)。poison=True 时注入 NaN（测试网络输出校验）。
    """

    name = "scripted-lattice"

    def __init__(self, reach: float = 3.0, piece_time: float = 1.0, poison: bool = False):
        self.reach = reach
        self.piece_time = piece_time
        self.poison = poison

    def describe(self) -> dict:
        return {"name": self.name, "reach": self.reach, "piece_time": self.piece_time}

    def __call__(self, obs, dec) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        yaw, pitch = dec.grid_angles()
        n = dec.count
        p = np.zeros((n, 14))
        p[:, 2] = (0.5 * self.reach / dec.tail_radio_max) * 2.0 - 1.0
        p[:, 3] = yaw / (np.radians(dec.horizon_fov_deg) / 2.0)
        p[:, 4] = 0.5 * pitch / (np.radians(dec.vertical_fov_deg) / 2.0)
        p[:, 5] = (self.reach / dec.tail_radio_max) * 2.0 - 1.0
        p[:, 12:14] = self.piece_time / (0.5 * dec.segment_time) - 1.0
        goal_b = obs.state.R_wb.T @ (np.asarray(obs.goal, dtype=float) - obs.state.position)
        g = goal_b / (np.linalg.norm(goal_b) + 1e-9)
        tail_dir = np.stack(
            [
                np.cos(0.5 * pitch) * np.cos(yaw),
                np.cos(0.5 * pitch) * np.sin(yaw),
                np.sin(0.5 * pitch),
            ],
            axis=-1,
        )
        score = 1.0 + tail_dir @ g
        radius = np.zeros((n, 2 * dec.radius_num))
        radius[:, : dec.radius_num] = 0.5
        radius[:, dec.radius_num :] = 0.02
        if self.poison:
            p[2, 7] = np.nan
        return p, score, radius
