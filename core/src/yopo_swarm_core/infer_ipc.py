"""每机推理进程的本地 IPC（ADR-0008 + 方案 (c) 拆容器）。

ROS 容器（Humble / Py3.10，无 GPU）里的节点通过 Unix 域套接字把一帧网络输入发给同机的推理容器
（CUDA 13.2 / Py3.12），拿回网络原始输出。一架无人机一个推理进程、一个套接字，不共享队列。

线格式（小端）：
  b"YSI1" | u32 头长度 | 头 JSON（utf-8）| 各数组原始字节（按头里 arrays 的顺序）
  头：{"op": ..., "seq": int, "arrays": [{"name", "dtype", "shape"}...], ...}
请求 op="forward"，数组 depth (1,1,H,W) float32（已是上游网络输入格式）、obs (1,9) float32（机体系 FLU：
速度、加速度、局部目标，未归一化，归一化在服务端按上游 StateTransform 做）。
应答 op="result"，数组 traj_params / score / radius（展平顺序同 baseline.CandidateBatch），
头里带 infer_ms（服务端前向耗时）与 backend。出错时 op="error"，带 reason。
只依赖 numpy 与标准库，ROS 容器可直接用；服务端才需要 torch。
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import sys
import time
from collections.abc import Callable
from pathlib import Path

import numpy as np

MAGIC = b"YSI1"
MAX_HEADER = 1 << 20
MAX_PAYLOAD = 64 << 20
_DTYPES = {"float32", "float64", "int32", "int64", "uint8", "bool"}


class IpcError(RuntimeError):
    pass


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise IpcError("connection closed")
        buf.extend(chunk)
    return bytes(buf)


def send_msg(
    sock: socket.socket, header: dict, arrays: dict[str, np.ndarray] | None = None
) -> None:
    arrays = arrays or {}
    specs, blobs = [], []
    for name, a in arrays.items():
        a = np.ascontiguousarray(a)
        if a.dtype.name not in _DTYPES:
            raise IpcError(f"unsupported dtype {a.dtype} for {name}")
        specs.append({"name": name, "dtype": a.dtype.name, "shape": list(a.shape)})
        blobs.append(a.astype(a.dtype.newbyteorder("<"), copy=False).tobytes())
    head = json.dumps({**header, "arrays": specs}, separators=(",", ":")).encode()
    sock.sendall(MAGIC + struct.pack("<I", len(head)) + head + b"".join(blobs))


def recv_msg(sock: socket.socket) -> tuple[dict, dict[str, np.ndarray]]:
    if _recv_exact(sock, 4) != MAGIC:
        raise IpcError("bad magic")
    (n,) = struct.unpack("<I", _recv_exact(sock, 4))
    if n > MAX_HEADER:
        raise IpcError(f"header too large: {n}")
    header = json.loads(_recv_exact(sock, n))
    arrays, total = {}, 0
    for spec in header.get("arrays", []):
        if spec["dtype"] not in _DTYPES:
            raise IpcError(f"unsupported dtype {spec['dtype']}")
        dt = np.dtype(spec["dtype"]).newbyteorder("<")
        size = int(np.prod(spec["shape"], dtype=np.int64)) * dt.itemsize
        total += size
        if total > MAX_PAYLOAD:
            raise IpcError("payload too large")
        raw = _recv_exact(sock, size)
        arrays[spec["name"]] = (
            np.frombuffer(raw, dtype=dt).reshape(spec["shape"]).astype(dt.newbyteorder("="))
        )
    return header, arrays


ForwardFn = Callable[[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray, np.ndarray]]


def serve(
    path: str | Path, forward: ForwardFn, backend: str, max_requests: int | None = None
) -> None:
    """单客户端串行服务（一机一进程）。客户端断开后等待下一个连接。"""
    path = Path(path)
    if path.exists():
        path.unlink()  # 只删本进程自己的旧套接字文件
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(str(path))
    os.chmod(path, 0o666)  # 跨容器（不同 uid）可连
    srv.listen(1)
    print(f"infer_ipc: listening on {path} backend={backend}", flush=True)
    served = 0
    try:
        while max_requests is None or served < max_requests:
            conn, _ = srv.accept()
            with conn:
                while max_requests is None or served < max_requests:
                    try:
                        header, arrays = recv_msg(conn)
                    except IpcError:
                        break
                    seq = header.get("seq", 0)
                    if header.get("op") != "forward":
                        send_msg(conn, {"op": "error", "seq": seq, "reason": "unknown op"})
                        continue
                    try:
                        t0 = time.perf_counter()
                        p, s, r = forward(arrays["depth"], arrays["obs"])
                        ms = (time.perf_counter() - t0) * 1e3
                        send_msg(
                            conn,
                            {"op": "result", "seq": seq, "infer_ms": ms, "backend": backend},
                            {"traj_params": p, "score": s, "radius": r},
                        )
                    except Exception as e:  # noqa: BLE001 —— 回报给客户端，服务不退出
                        send_msg(
                            conn, {"op": "error", "seq": seq, "reason": f"{type(e).__name__}: {e}"}
                        )
                    served += 1
    finally:
        srv.close()


class IpcClient:
    def __init__(self, path: str | Path, timeout: float = 5.0) -> None:
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect(str(path))
        self.seq = 0
        self.last_header: dict = {}

    def forward(
        self, depth: np.ndarray, obs: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        self.seq += 1
        send_msg(
            self.sock,
            {"op": "forward", "seq": self.seq},
            {"depth": np.asarray(depth, np.float32), "obs": np.asarray(obs, np.float32)},
        )
        header, arrays = recv_msg(self.sock)
        self.last_header = header
        if header.get("seq") != self.seq:
            raise IpcError(f"seq mismatch {header.get('seq')} != {self.seq}")
        if header.get("op") != "result":
            raise IpcError(header.get("reason", "error"))
        return arrays["traj_params"], arrays["score"], arrays["radius"]

    def close(self) -> None:
        self.sock.close()


GOAL_LENGTH = 10.0  # 上游 cfg["goal_length"] = tail_radio_max（config/config.py）
ARRIVE_RADIUS = 5.0  # 上游 callback_odometry：距目标 < 5 m 视为到达


def goal_obs(goal_w, position_w, goal_length: float = GOAL_LENGTH) -> np.ndarray:
    """世界系局部目标偏移，z 优先截断到 goal_length（上游 test_yopo_ros.py:164-170 原样移植）。

    ‖g‖ > goal_length 时先把 z 截到 ±goal_length，再把水平分量缩放到剩余长度，
    使截断后 ‖g‖ = goal_length 且尽量保留高度差。上游 `goal_w = self.goal - self.desire_pos`，
    即 position_w 应取参考（期望）位置。
    """
    goal_w = np.asarray(goal_w, dtype=float) - np.asarray(position_w, dtype=float)
    if np.linalg.norm(goal_w) > goal_length:
        gz = np.clip(goal_w[2], -goal_length, goal_length)
        hxy = goal_w[:2] / (np.linalg.norm(goal_w[:2]) + 1e-9) * np.sqrt(goal_length**2 - gz**2)
        goal_w = np.array([hxy[0], hxy[1], gz])
    return goal_w


def arrived(position_w, goal_w, radius: float = ARRIVE_RADIUS) -> bool:
    """上游 callback_odometry 的到达判定：‖p − goal‖ < radius（严格小于）。"""
    d = np.asarray(position_w, dtype=float) - np.asarray(goal_w, dtype=float)
    return bool(np.linalg.norm(d) < radius)


def body_obs(
    velocity_w, acceleration_w, goal_w, position_w, R_wb, goal_length: float = GOAL_LENGTH
) -> np.ndarray:
    """世界系 ENU 状态 → 网络 9 维观测（机体系 FLU：速度、加速度、局部目标），形状 (1, 9)。

    与上游 `process_odom` 一致：目标偏移先经 `goal_obs` z 优先截断，再转到相机系。
    相机俯仰为 0（上游 pitch_angle_deg 默认 -0），相机系与机体系相同（R_bc = I）。
    """
    Rt = np.asarray(R_wb, dtype=float).T
    v = Rt @ np.asarray(velocity_w, dtype=float)
    a = Rt @ np.asarray(acceleration_w, dtype=float)
    g = Rt @ goal_obs(goal_w, position_w, goal_length)
    return np.concatenate([v, a, g])[None, :].astype(np.float32)


# --- 命令行：推理服务 / 延迟基准 ---------------------------------------------------


def _torch_forward(device: str) -> tuple[ForwardFn, str]:
    import torch

    from yopo_swarm_core import baseline as B

    torch.set_num_threads(1)
    # 与 #11 L1 判据一致：关 TF32（默认 cudnn 卷积走 TF32，Orin 实测与 CPU 金样 traj max_abs 约 1.6e-4）
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cuda.matmul.allow_tf32 = False
    model = B.load_verified_model(B.fetch_upstream())
    net = model._net.eval().to(device)
    st = model._st
    # 上游格点单例在 GPU 可见时把 _Rbp_image_grid 放到 CUDA；预处理统一在 CPU 做，再整体送 device
    # （与 docker/common/golden_gpu.py 的 #11 修复一致，不改上游源码）
    st._Rbp_image_grid = st._Rbp_image_grid.cpu()

    def fwd(depth, obs):
        with torch.no_grad():
            d = torch.from_numpy(np.ascontiguousarray(depth, np.float32)).to(device)
            o = torch.from_numpy(np.ascontiguousarray(obs, np.float32))
            o = st.prepare_input(st.normalize_obs(o)).to(device)
            e, s, r = net(d, o)
            b = e.shape[0]
            p = e.permute(0, 2, 3, 1).reshape(b, -1, e.shape[1])
            r = r.permute(0, 2, 3, 1).reshape(b, -1, r.shape[1])
            return p.cpu().numpy(), s.reshape(b, -1).cpu().numpy(), r.cpu().numpy()

    return fwd, f"torch-{device}-fp32"


def _bench(path: str, n: int, out: Path | None) -> int:
    from yopo_swarm_core import baseline as B

    # 与金样相同的网格（上游 YOPO-MINCO：3×5 候选、下采样 32 → 深度 96×160），客户端侧不加载上游配置
    cfg = {"vertical_num": 3, "horizon_num": 5, "downsample_factor": 32}
    depth, obs = B.fixed_inputs(cfg)
    golden = np.load(B.GOLDEN_PATH) if Path(B.GOLDEN_PATH).exists() else None
    cli = IpcClient(path, timeout=30.0)
    rtt, inf = [], []
    p = s = r = None
    for _ in range(n + 1):
        t0 = time.perf_counter()
        p, s, r = cli.forward(depth, obs)
        rtt.append((time.perf_counter() - t0) * 1e3)
        inf.append(cli.last_header["infer_ms"])
    cli.close()
    rep = {
        "backend": cli.last_header.get("backend"),
        "n": n,
        "python": sys.version.split()[0],
        "rtt_ms_p50_p99": [float(np.percentile(rtt[1:], 50)), float(np.percentile(rtt[1:], 99))],
        "infer_ms_p50_p99": [float(np.percentile(inf[1:], 50)), float(np.percentile(inf[1:], 99))],
    }
    if golden is not None:
        rep["max_abs_vs_cpu_golden"] = {
            "traj_params": float(np.max(np.abs(p - golden["traj_params"]))),
            "score": float(np.max(np.abs(s - golden["score"]))),
            "radius": float(np.max(np.abs(r - golden["radius"]))),
        }
        rep["argmax_equal"] = bool(np.argmax(s) == np.argmax(golden["score"]))
    text = json.dumps(rep, indent=2)
    print(text)
    if out:
        out.write_text(text)
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m yopo_swarm_core.infer_ipc")
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("serve", help="推理服务（推理容器里跑，每机一个）")
    a.add_argument("--socket", default="/ipc/uav0.sock")
    a.add_argument("--device", default="cuda")
    b = sub.add_parser("bench", help="客户端往返延迟 + 与 CPU 金样比较（任一容器）")
    b.add_argument("--socket", default="/ipc/uav0.sock")
    b.add_argument("-n", type=int, default=200)
    b.add_argument("--out", type=Path)
    args = ap.parse_args(argv)
    if args.cmd == "serve":
        fwd, backend = _torch_forward(args.device)
        serve(args.socket, fwd, backend)
        return 0
    return _bench(args.socket, args.n, args.out)


if __name__ == "__main__":
    sys.exit(main())
