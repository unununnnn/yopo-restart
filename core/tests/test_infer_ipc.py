"""方案 (c) 拆容器的推理 IPC：线格式、串行服务、机体系观测。"""

import socket
import tempfile
import threading
from pathlib import Path

import numpy as np
import pytest

from yopo_swarm_core import infer_ipc as ipc


def test_framing_roundtrip_preserves_dtype_shape_values():
    a, b = socket.socketpair()
    arrs = {
        "depth": np.random.default_rng(0).random((1, 1, 96, 160), dtype=np.float32),
        "obs": np.arange(9, dtype=np.float32)[None],
        "flag": np.array([True, False]),
    }
    ipc.send_msg(a, {"op": "forward", "seq": 3}, arrs)
    header, got = ipc.recv_msg(b)
    assert header["op"] == "forward" and header["seq"] == 3
    for k, v in arrs.items():
        assert got[k].dtype == v.dtype and got[k].shape == v.shape
        assert np.array_equal(got[k], v)


def test_bad_magic_and_closed_connection():
    a, b = socket.socketpair()
    a.sendall(b"XXXX\x00\x00\x00\x00")
    with pytest.raises(ipc.IpcError):
        ipc.recv_msg(b)
    a.close()
    with pytest.raises(ipc.IpcError):
        ipc.recv_msg(b)


def _fake_forward(depth, obs):
    if not np.all(np.isfinite(obs)):
        raise ValueError("nan obs")
    n = 15
    p = np.full((1, n, 14), float(depth.mean()), np.float32)
    s = np.arange(n, dtype=np.float32)[None] + obs[0, 0]
    return p, s, np.zeros((1, n, 20), np.float32)


def test_serve_and_client_sequence_and_errors():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "uav0.sock"
        t = threading.Thread(target=ipc.serve, args=(path, _fake_forward, "fake", 3), daemon=True)
        t.start()
        for _ in range(100):
            if path.exists():
                break
            threading.Event().wait(0.01)
        # bind 已经写出套接字文件，listen 可能还没跟上。连早了就是 ConnectionRefused。
        cli = None
        refused: ConnectionRefusedError | None = None
        for _ in range(50):
            try:
                cli = ipc.IpcClient(path)
                break
            except ConnectionRefusedError as exc:
                refused = exc
                threading.Event().wait(0.02)
        if cli is None:
            raise refused  # type: ignore[misc]
        depth = np.ones((1, 1, 96, 160), np.float32)
        p, s, r = cli.forward(depth, np.ones((1, 9), np.float32))
        assert p.shape == (1, 15, 14) and np.all(p == 1.0)
        assert s[0, 0] == 1.0 and r.shape == (1, 15, 20)
        assert cli.last_header["backend"] == "fake" and cli.last_header["infer_ms"] >= 0
        with pytest.raises(ipc.IpcError, match="nan obs"):
            cli.forward(depth, np.full((1, 9), np.nan, np.float32))
        p2, _, _ = cli.forward(depth * 2, np.zeros((1, 9), np.float32))
        assert np.all(p2 == 2.0) and cli.seq == 3
        cli.close()
        t.join(timeout=5)
        assert not t.is_alive()


def test_body_obs_rotates_world_enu_into_flu():
    yaw = np.pi / 2  # 机头朝北（ENU +y）
    R_wb = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    o = ipc.body_obs([0, 1, 0], [0, 0, 1], [1, 5, 2], [1, 1, 1], R_wb)
    assert o.shape == (1, 9) and o.dtype == np.float32
    assert np.allclose(o[0, :3], [1, 0, 0], atol=1e-6)  # 向北飞 = 机体前向
    assert np.allclose(o[0, 3:6], [0, 0, 1], atol=1e-6)
    assert np.allclose(o[0, 6:], [4, 0, 1], atol=1e-6)
