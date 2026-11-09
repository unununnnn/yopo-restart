"""原始深度双管线：安全证据保留米制数据，网络输入复现固定上游。

对应 YOPO @ d5e1e60 / YOPO/test_yopo_ros.py:218–234。
NS 半径 1 的 numpy 实现在 _inpaint.py；不依赖 ROS、torch 或 OpenCV。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ._inpaint import ns_radius_one
from .freespace import CameraIntrinsics, DepthFrame, FreeSpaceConfig

# Gazebo 相机 link 为 FLU；投影坐标为 optical (右、下、前)。
R_FLU_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
T_BODY_OPTICAL = np.eye(4)
T_BODY_OPTICAL[:3, :3] = R_FLU_OPTICAL
# sdformat14 语义帧图：x500_base 的合并模型帧在根模型 z=.24，必须扣除。
# (x500_depth camera include + sensor pose) - base_link in root model。
T_BODY_OPTICAL[:3, 3] = [0.13233, 0.0, 0.02078]


def decode_image(
    data: bytes, width: int, height: int, step: int, encoding: str, bigendian: bool = False
) -> np.ndarray:
    """sensor_msgs/Image → float32 米；支持行 padding 与消息字节序。"""
    if encoding not in ("32FC1", "16UC1"):
        raise ValueError(f"unsupported depth encoding: {encoding}")
    dt = np.dtype((">" if bigendian else "<") + ("f4" if encoding == "32FC1" else "u2"))
    if width <= 0 or height <= 0 or step < width * dt.itemsize or len(data) != step * height:
        raise ValueError("invalid depth image layout")
    rows = np.frombuffer(data, np.uint8).reshape(height, step)
    depth = np.ascontiguousarray(rows[:, : width * dt.itemsize]).view(dt).reshape(height, width)
    depth = depth.astype(np.float32)
    return depth if encoding == "32FC1" else depth / np.float32(1000)


def network_depth(raw_m: np.ndarray) -> np.ndarray:
    """96×160 最近邻，不裁图；20 m 截断、0.04 m/NaN 掩码、uint8 NS 补洞。"""
    d = np.asarray(raw_m, dtype=np.float32)
    if d.ndim != 2 or min(d.shape) == 0:
        raise ValueError("depth must be a nonempty 2D image")
    # cv2.INTER_NEAREST 使用 floor(dst_index * src_size / dst_size)。
    y = np.floor(np.arange(96) * (d.shape[0] / 96)).astype(int)
    x = np.floor(np.arange(160) * (d.shape[1] / 160)).astype(int)
    normalized = np.minimum(d[np.ix_(y, x)], np.float32(20)) / np.float32(20)
    holes = np.isnan(normalized) | (normalized < np.float32(0.04 / 20))
    # 上游 NaN 转 uint8 为 0；显式处理以避免 numpy RuntimeWarning。
    u8 = np.nan_to_num(normalized * np.float32(255), nan=0, neginf=0).astype(np.uint8)
    filled = ns_radius_one(u8, holes)
    return (filled.astype(np.float32) / np.float32(255))[None, None]


def optical_pose(position_enu, R_enu_flu, T_body_optical=T_BODY_OPTICAL) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R_enu_flu
    T[:3, 3] = position_enu
    return T @ T_body_optical


@dataclass(frozen=True)
class DepthSample:
    raw_m: np.ndarray
    intrinsics: CameraIntrinsics
    T_world_optical: np.ndarray
    stamp_ns: int

    def evidence(self, cfg: FreeSpaceConfig) -> DepthFrame:
        return DepthFrame.from_raw(
            self.raw_m, self.intrinsics, self.T_world_optical, self.stamp_ns / 1e9, cfg
        )

    def save(self, path: Path) -> None:
        k = self.intrinsics
        np.savez_compressed(
            path,
            raw_m=self.raw_m,
            K=np.array([k.fx, k.fy, k.cx, k.cy]),
            T_world_optical=self.T_world_optical,
            stamp_ns=np.int64(self.stamp_ns),
        )

    @classmethod
    def load(cls, path: Path) -> DepthSample:
        with np.load(path, allow_pickle=False) as z:
            d = z["raw_m"].copy()
            k = CameraIntrinsics(*map(float, z["K"]), width=d.shape[1], height=d.shape[0])
            return cls(d, k, z["T_world_optical"].copy(), int(z["stamp_ns"]))
