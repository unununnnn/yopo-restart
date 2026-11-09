"""Y6 训练薄壳：原样调用钉死在 d5e1e60 的上游训练代码（D8）。

本模块只做三件事，不改任何损失、网络或优化器数学：

1. 路径与配置：把上游 ``YOPO/`` 放进 ``sys.path``，把 ``cfg["dataset_path"]`` 指到调用方给的
   绝对路径（上游用 ``os.path.join(<模块目录>/.., dataset_path)``，绝对路径会覆盖前缀）。
2. 设备单例（K9）：上游 ``primitive.py`` 等在 import/构造时按 ``torch.cuda.is_available()``
   固定设备。这里在 import 与构造期间把它钉成调用方选定的设备，并保证一个进程只用一个设备。
3. open3d 替代：上游 ``SafetyLoss`` 只用 ``o3d.io.read_point_cloud`` 读 PLY 点云。装不上 open3d
   （例如 aarch64 推理容器）时注册一个只读 PLY 的最小替身；装得上就用真的 open3d。
"""

from __future__ import annotations

import contextlib
import sys
import types
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import numpy as np

_DEVICE: str | None = None


# --- 1. 路径与配置 ----------------------------------------------------------------


def upstream_yopo_dir(upstream_root: Path | None = None) -> Path:
    """上游 ``YOPO/`` 目录；缺省按 ADR-0006 的固定 SHA 取到缓存。"""
    if upstream_root is None:
        from .baseline import fetch_upstream

        upstream_root = fetch_upstream()
    return Path(upstream_root) / "YOPO"


def add_upstream_to_path(upstream_root: Path | None = None) -> Path:
    yopo = upstream_yopo_dir(upstream_root)
    if str(yopo) not in sys.path:
        sys.path.insert(0, str(yopo))
    return yopo


def configure(
    dataset_path: Path | str, upstream_root: Path | None = None, device: str | None = None
) -> Any:
    """返回上游 ``cfg`` 单例，``dataset_path`` 改成绝对路径；其余键保持上游 traj_opt.yaml。

    ``device`` 在这里就钉住单例：上游 import 期读 CUDA 可用性，调用方随后用的设备必须与之一致。
    """
    add_upstream_to_path(upstream_root)
    install_open3d_fallback()
    with pinned_device(device):
        from config.config import cfg
    cfg["dataset_path"] = str(Path(dataset_path).resolve())
    return cfg


# --- 2. 设备单例（K9）---------------------------------------------------------------


def select_device(device: str | None = None) -> str:
    """本进程唯一的训练设备。首次调用决定（缺省：有 CUDA 用 cuda），之后只能重复同一个值。"""
    global _DEVICE
    import torch

    if device is None:
        device = _DEVICE or ("cuda" if torch.cuda.is_available() else "cpu")
    if device not in ("cpu", "cuda"):
        raise ValueError(f"device must be 'cpu' or 'cuda', got {device!r}")
    if _DEVICE is not None and device != _DEVICE:
        raise RuntimeError(
            f"upstream device singleton already pinned to {_DEVICE!r}; one device per process (K9)"
        )
    if device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("device='cuda' requested but CUDA is not available")
    _DEVICE = device
    return device


@contextlib.contextmanager
def pinned_device(device: str | None = None) -> Iterator[str]:
    """在上下文里让 ``torch.cuda.is_available()`` 与选定设备一致，供上游 import/构造期读取。"""
    import torch

    dev = select_device(device)
    real = torch.cuda.is_available
    torch.cuda.is_available = lambda: dev == "cuda"  # type: ignore[assignment]
    try:
        yield dev
    finally:
        torch.cuda.is_available = real  # type: ignore[assignment]


# --- 3. open3d 替代 ----------------------------------------------------------------

_PLY_TYPES = {
    "char": "i1",
    "int8": "i1",
    "uchar": "u1",
    "uint8": "u1",
    "short": "i2",
    "int16": "i2",
    "ushort": "u2",
    "uint16": "u2",
    "int": "i4",
    "int32": "i4",
    "uint": "u4",
    "uint32": "u4",
    "float": "f4",
    "float32": "f4",
    "double": "f8",
    "float64": "f8",
}


def read_ply_points(path: Path | str) -> np.ndarray:
    """读 PLY ``vertex`` 元素的 x/y/z，返回 (N, 3) float64（与 open3d 一致）。

    支持 ascii / binary_little_endian / binary_big_endian；顶点元素须是第一个元素且无 list 属性
    （PCL ``savePLYFile*`` 写出的点云满足这两点）。
    """
    data = Path(path).read_bytes()
    end = data.find(b"end_header")
    if not data.startswith(b"ply") or end < 0:
        raise ValueError(f"{path}: not a PLY file")
    body_start = data.index(b"\n", end) + 1
    header = data[:end].decode("ascii").splitlines()
    fmt, n, props, seen_vertex = None, 0, [], False
    for line in header:
        tok = line.split()
        if not tok:
            continue
        if tok[0] == "format":
            fmt = tok[1]
        elif tok[0] == "element":
            if seen_vertex:
                break  # 只读第一个元素（vertex）
            if tok[1] != "vertex":
                raise ValueError(f"{path}: first element must be vertex, got {tok[1]}")
            seen_vertex, n = True, int(tok[2])
        elif tok[0] == "property" and seen_vertex:
            if tok[1] == "list":
                raise ValueError(f"{path}: list properties in vertex are not supported")
            props.append((tok[2], _PLY_TYPES[tok[1]]))
    names = [p[0] for p in props]
    if not {"x", "y", "z"} <= set(names):
        raise ValueError(f"{path}: vertex lacks x/y/z")
    if fmt == "ascii":
        rows = data[body_start:].decode("ascii").split("\n")[:n]
        arr = np.array([r.split()[: len(props)] for r in rows], dtype=np.float64).reshape(n, -1)
        cols = {name: arr[:, i] for i, name in enumerate(names)}
    elif fmt in ("binary_little_endian", "binary_big_endian"):
        bo = "<" if fmt == "binary_little_endian" else ">"
        dt = np.dtype([(name, bo + t) for name, t in props])
        rec = np.frombuffer(data, dtype=dt, count=n, offset=body_start)
        cols = {name: rec[name] for name in names}
    else:
        raise ValueError(f"{path}: unsupported PLY format {fmt!r}")
    return np.stack([cols["x"], cols["y"], cols["z"]], axis=1).astype(np.float64)


class _PointCloud:
    """open3d.geometry.PointCloud 的最小只读替身（上游只用 points / get_min_bound / get_max_bound）。"""

    def __init__(self, points: np.ndarray) -> None:
        self.points = points

    def get_min_bound(self) -> np.ndarray:
        return self.points.min(axis=0) if len(self.points) else np.zeros(3)

    def get_max_bound(self) -> np.ndarray:
        return self.points.max(axis=0) if len(self.points) else np.zeros(3)


def make_open3d_stub() -> types.ModuleType:
    mod = types.ModuleType("open3d")
    mod.__dict__["__yopo_swarm_stub__"] = True
    io = types.ModuleType("open3d.io")
    io.read_point_cloud = lambda filename, *a, **k: _PointCloud(read_ply_points(filename))  # type: ignore[attr-defined]
    mod.io = io  # type: ignore[attr-defined]
    return mod


def install_open3d_fallback(force_stub: bool = False) -> bool:
    """open3d 可用就用真的；否则（或 force_stub）注册替身。返回是否用了替身。"""
    if not force_stub:
        existing = sys.modules.get("open3d")
        if existing is not None:
            return bool(getattr(existing, "__yopo_swarm_stub__", False))
        try:
            import open3d  # noqa: F401

            return False
        except ImportError:
            pass
    stub = make_open3d_stub()
    sys.modules["open3d"] = stub
    sys.modules["open3d.io"] = stub.io
    return True
