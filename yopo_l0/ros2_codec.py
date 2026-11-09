"""观测的逐位编码。深度用小端 float32 原字节，标量用 float.hex，避免 JSON 十进制改掉末位。

一条规划请求拆成一条 header 和若干深度分片，每片原始字节不超过 OBS_CHUNK_BYTES，
这样单条 ROS 字符串留在常见 DDS 分片上限之下。节点都在本进程里，字节序就是本机。
"""

from __future__ import annotations

import base64
import json
import re

import numpy as np

from .dynamics import State
from .policy import Observation
from .sensor import DepthFrame

OBS_CHUNK_BYTES = 24576
_NS = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def check_drone_id(drone_id: str) -> str:
    """ROS 命名空间按机区分。id 必须是合法的 ROS 名字，编队卡以后沿用这条，不在这里写死 uavN。"""
    if not isinstance(drone_id, str) or _NS.fullmatch(drone_id) is None:
        raise ValueError(f"无人机 id {drone_id!r} 不能作为 ROS 命名空间（需要 [A-Za-z_][A-Za-z0-9_]*）")
    return drone_id


def sim_time_ns(k: int, phys: int) -> int:
    """tick 起点的仿真时间，单位纳秒。用整数除法，不走 k/phys 的二进制小数。"""
    if int(k) < 0 or int(phys) <= 0:
        raise ValueError(f"非法 tick 时钟 k={k} phys={phys}")
    return (int(k) * 1_000_000_000) // int(phys)


def _hex(x: float) -> str:
    return float(x).hex()


def _unhex(text: str) -> float:
    return float.fromhex(text)


def _arr(a) -> list[str]:
    return [_hex(x) for x in np.asarray(a, dtype=np.float64).reshape(-1)]


def _unarr(values: list[str]) -> np.ndarray:
    return np.array([_unhex(x) for x in values], dtype=np.float64)


def encode_observation(obs: Observation, *, drone_id: str, seq: int) -> list[str]:
    """返回按发送顺序排列的 JSON 字符串：先 header，再深度分片。"""
    check_drone_id(drone_id)
    header: dict = {
        "v": 1,
        "kind": "header",
        "seq": int(seq),
        "drone_id": drone_id,
        "now": _hex(obs.now),
        "goal": _arr(obs.goal),
        "state": {
            "t": _hex(obs.state.t),
            "p": _arr(obs.state.p),
            "v": _arr(obs.state.v),
            "a": _arr(obs.state.a),
            "yaw": _hex(obs.state.yaw),
        },
        "frame": None,
        "n_chunks": 0,
    }
    messages = []
    frame = obs.frame
    if frame is None:
        messages.append(json.dumps(header, separators=(",", ":")))
        return messages
    depth = np.asarray(frame.depth)
    if depth.dtype != np.float32 or depth.ndim != 2:
        raise TypeError(f"深度必须是二维 float32，收到 dtype={depth.dtype} ndim={depth.ndim}")
    blob = np.ascontiguousarray(depth, dtype="<f4").tobytes()
    chunks = [blob[i : i + OBS_CHUNK_BYTES] for i in range(0, len(blob), OBS_CHUNK_BYTES)] or []
    if len(chunks) + 1 > 64:
        raise RuntimeError(
            f"深度 {len(blob)} 字节超过锁步队列（64 条，每条 {OBS_CHUNK_BYTES} 字节）。"
            "加大观测 QoS 深度后再跑这张卡。"
        )
    header["frame"] = {
        "stamp": _hex(frame.stamp),
        "K": [_hex(x) for x in frame.K],
        "shape": [int(depth.shape[0]), int(depth.shape[1])],
        "dtype": "float32",
        "nbytes": len(blob),
    }
    header["n_chunks"] = len(chunks)
    messages.append(json.dumps(header, separators=(",", ":")))
    for i, chunk in enumerate(chunks):
        messages.append(json.dumps({
            "v": 1, "kind": "depth", "seq": int(seq), "i": i,
            "b64": base64.b64encode(chunk).decode("ascii"),
        }, separators=(",", ":")))
    return messages


def decode_observation_messages(messages: list[str]) -> Observation:
    if not messages:
        raise ValueError("观测消息是空的")
    header = json.loads(messages[0])
    if header.get("v") != 1 or header.get("kind") != "header":
        raise ValueError("第一条观测消息不是 v1 header")
    n_chunks = int(header["n_chunks"])
    if len(messages) != 1 + n_chunks:
        raise ValueError(f"深度分片数量不对：header 说 {n_chunks}，实际 {len(messages) - 1}")
    st = header["state"]
    state = State(
        t=_unhex(st["t"]),
        p=_unarr(st["p"]),
        v=_unarr(st["v"]),
        a=_unarr(st["a"]),
        yaw=_unhex(st["yaw"]),
    )
    raw_frame = header["frame"]
    if raw_frame is None:
        frame = None
    else:
        parts: list[bytes | None] = [None] * n_chunks
        for text in messages[1:]:
            piece = json.loads(text)
            if piece.get("kind") != "depth" or int(piece["seq"]) != int(header["seq"]):
                raise ValueError("深度分片的 seq 和 header 不一致")
            idx = int(piece["i"])
            if idx < 0 or idx >= n_chunks or parts[idx] is not None:
                raise ValueError(f"深度分片序号重复或越界：{idx}")
            parts[idx] = base64.b64decode(piece["b64"])
        blob = b"".join(parts)  # type: ignore[arg-type]
        if len(blob) != int(raw_frame["nbytes"]):
            raise ValueError(f"深度字节数 {len(blob)} != {raw_frame['nbytes']}")
        h, w = (int(x) for x in raw_frame["shape"])
        if h * w * 4 != len(blob):
            raise ValueError(f"深度形状 {h}x{w} 和字节数 {len(blob)} 对不上")
        depth = np.frombuffer(blob, dtype="<f4").reshape(h, w).copy()
        frame = DepthFrame(
            stamp=_unhex(raw_frame["stamp"]),
            depth=depth,
            K=tuple(_unhex(x) for x in raw_frame["K"]),
        )
    return Observation(frame, state, _unarr(header["goal"]), _unhex(header["now"]))


def same_observation(a: Observation, b: Observation) -> bool:
    """digest 覆盖深度字节、内参、目标、里程计和 now。state.t 不在 digest 里，单独比。"""
    if a.digest() != b.digest() or float(a.state.t) != float(b.state.t):
        return False
    if (a.frame is None) != (b.frame is None):
        return False
    if a.frame is not None and a.frame.depth.dtype != b.frame.depth.dtype:
        return False
    return True
