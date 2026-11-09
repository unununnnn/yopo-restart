"""Y6 用的小型合成训练数据（Y7 出真数据前的替身）：确定性、无随机数、跨平台逐字节一致。

目录布局与上游 ``YOPODataset`` / ``SafetyLoss`` 读取的一致::

    <root>/pointcloud-<i>.ply   竖直圆柱林 + 地面的点云（binary_little_endian float32 x/y/z）
    <root>/pose-<i>.csv         表头 + 每帧 [px, py, pz, qw, qx, qy, qz]
    <root>/<i>/img_<k>.png      16 位深度图（0–20 m → 0–65535），按 k 与 pose 行对齐

深度图由同一组圆柱解析射线求交得到（机体系 x 前 y 左 z 上，水平 90°、竖直 60° 视场）。
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np

DEPTH_MAX = 20.0  # 上游数据约定：int16 0–20 m


@dataclass(frozen=True)
class Forest:
    centers: np.ndarray  # (K, 2) 圆柱中心 xy
    radius: float
    height: float
    extent: tuple[float, float, float, float]  # xmin, xmax, ymin, ymax


def forest(index: int) -> Forest:
    """第 index 张地图：规则网格上按 (i*7 + j*3 + index) 取舍的圆柱，确定性。"""
    xs, ys = np.arange(4.0, 30.0, 3.0), np.arange(-12.0, 12.5, 3.0)
    keep = [
        (x + (j % 2) * 1.5, y)
        for i, x in enumerate(xs)
        for j, y in enumerate(ys)
        if (i * 7 + j * 3 + index) % 4 != 0
    ]
    return Forest(np.array(keep, dtype=np.float64), 0.4, 6.0, (-2.0, 32.0, -14.0, 14.0))


def forest_points(f: Forest, step: float = 0.1) -> np.ndarray:
    """圆柱表面 + 地面点（float32），步长 step（米）。"""
    ang = np.arange(0.0, 2 * np.pi, step / f.radius)
    zs = np.arange(0.0, f.height + 1e-9, step)
    ring = np.stack([np.cos(ang), np.sin(ang)], axis=1) * f.radius
    pts = []
    for c in f.centers:
        xy = np.repeat(ring + c, len(zs), axis=0)
        z = np.tile(zs, len(ring))
        pts.append(np.column_stack([xy, z]))
    gx = np.arange(f.extent[0], f.extent[1] + 1e-9, 0.2)
    gy = np.arange(f.extent[2], f.extent[3] + 1e-9, 0.2)
    g = np.stack(np.meshgrid(gx, gy, indexing="ij"), axis=-1).reshape(-1, 2)
    pts.append(np.column_stack([g, np.zeros(len(g))]))
    return np.concatenate(pts).astype(np.float32)


def write_ply(path: Path, points: np.ndarray) -> None:
    pts = np.ascontiguousarray(points, dtype="<f4")
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(pts)}\n"
        "property float x\nproperty float y\nproperty float z\nend_header\n"
    ).encode("ascii")
    Path(path).write_bytes(header + pts.tobytes())


def write_png16(path: Path, img: np.ndarray) -> None:
    """单通道 16 位 PNG（无 cv2 依赖）；cv2.imread(path, -1) 读回同值 uint16。"""
    a = np.ascontiguousarray(img, dtype=">u2")
    h, w = a.shape
    raw = b"".join(b"\x00" + a[r].tobytes() for r in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", w, h, 16, 0, 0, 0, 0)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(raw, 6))
    Path(path).write_bytes(png + chunk(b"IEND", b""))


def render_depth(
    f: Forest, pos: np.ndarray, yaw: float, width: int, height: int, hfov=90.0, vfov=60.0
) -> np.ndarray:
    """水平机体姿态（只有 yaw）下的解析深度图（沿光轴的 x 深度，米，上限 DEPTH_MAX）。"""
    fx = (width / 2) / np.tan(np.radians(hfov) / 2)
    fy = (height / 2) / np.tan(np.radians(vfov) / 2)
    u = (np.arange(width) + 0.5 - width / 2) / fx
    v = (np.arange(height) + 0.5 - height / 2) / fy
    dy, dz = np.meshgrid(-u, -v)  # 体系 y 左、z 上
    c, s = np.cos(yaw), np.sin(yaw)
    wx, wy = c - s * dy, s + c * dy  # 体系 (1, dy) 旋到世界系；x 分量为 1 ⇒ 参数 t 即 x 深度
    best = np.full(dy.shape, DEPTH_MAX)
    for cx, cy in f.centers:
        ox, oy = pos[0] - cx, pos[1] - cy
        a = wx * wx + wy * wy
        b = 2 * (ox * wx + oy * wy)
        cc = ox * ox + oy * oy - f.radius**2
        disc = b * b - 4 * a * cc
        ok = disc >= 0
        t = np.where(ok, (-b - np.sqrt(np.where(ok, disc, 0.0))) / (2 * a), DEPTH_MAX + 1.0)
        zhit = pos[2] + t * dz
        hit = ok & (t > 0) & (zhit >= 0) & (zhit <= f.height)
        best = np.where(hit & (t < best), t, best)
    tg = np.where(dz < 0, -pos[2] / np.where(dz < 0, dz, -1.0), DEPTH_MAX + 1.0)  # 地面
    best = np.where((tg > 0) & (tg < best), tg, best)
    return np.minimum(best, DEPTH_MAX)


def write_dataset(
    root: Path, maps: int = 1, frames: int = 8, width: int = 160, height: int = 96
) -> Path:
    """写 maps 张地图、每张 frames 帧；位姿沿 x 轴走廊、yaw 在 ±0.3 rad 内确定性摆动。"""
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    for m in range(maps):
        f = forest(m)
        write_ply(root / f"pointcloud-{m}.ply", forest_points(f))
        d = root / str(m)
        d.mkdir(exist_ok=True)
        rows = []
        for k in range(frames):
            pos = np.array([0.5 + 0.25 * k, 0.15 * np.sin(k), 1.2 + 0.1 * (k % 5)])
            yaw = 0.3 * np.sin(0.7 * k)
            depth = render_depth(f, pos, yaw, width, height)
            write_png16(d / f"img_{k}.png", np.round(depth / DEPTH_MAX * 65535.0))
            rows.append([*pos, np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
        lines = ["px,py,pz,qw,qx,qy,qz"] + [",".join(f"{x:.9g}" for x in r) for r in rows]
        (root / f"pose-{m}.csv").write_text("\n".join(lines) + "\n")
    return root
