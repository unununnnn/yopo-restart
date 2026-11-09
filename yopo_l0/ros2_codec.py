"""观测和机间总线信封的逐位编码。深度用小端 float32 原字节，标量用 float.hex，避免 JSON 十进制改掉末位。

一条规划请求拆成一条 header 和若干深度分片，每片原始字节不超过 OBS_CHUNK_BYTES，
这样单条 ROS 字符串留在常见 DDS 分片上限之下。节点都在本进程里，字节序就是本机。

机间总线信封同样拆片。协议消息里的计划和占用走规范形式（float.hex），往返后计划哈希不变。
"""

from __future__ import annotations

import base64
import json
import re

import numpy as np

from yopo_swarm_core.plan import BrakeBranch, KinematicLimits, Plan, PlanConfig
from yopo_swarm_core.protocol.messages import (
    Commit,
    Decision,
    Heartbeat,
    Proposal,
    Reason,
    Release,
    VersionContext,
    Vote,
)
from yopo_swarm_core.protocol.plan_occupancy import PlanOccupancy
from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment

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


def same_float(a: float, b: float) -> bool:
    """含 inf / -0.0。float.hex 相同才算逐位相同。"""
    return _hex(a) == _hex(b)


def _ctx_wire(ctx: VersionContext) -> dict:
    return {
        "member_epoch": int(ctx.member_epoch),
        "task_hash": ctx.task_hash,
        "clock_epoch": int(ctx.clock_epoch),
        "sender_boot_id": int(ctx.sender_boot_id),
        "sender_id": ctx.sender_id,
        "message_seq": int(ctx.message_seq),
    }


def _ctx_from_wire(data: dict) -> VersionContext:
    return VersionContext(
        member_epoch=int(data["member_epoch"]),
        task_hash=str(data["task_hash"]),
        clock_epoch=int(data["clock_epoch"]),
        sender_boot_id=int(data["sender_boot_id"]),
        sender_id=str(data["sender_id"]),
        message_seq=int(data["message_seq"]),
    )


def _traj_from_wire(data: dict) -> PiecewisePolynomial:
    segments = []
    for seg in data["segments"]:
        coeffs = np.array([[_unhex(c) for c in row] for row in seg["coeffs"]], dtype=float)
        segments.append(PolySegment(coeffs, _unhex(seg["duration"])))
    t0 = _unhex(data["t0"])
    t_start, t_end = (_unhex(x) for x in data["valid"])
    return PiecewisePolynomial(t0, segments, t_start, t_end)


def _plan_from_wire(data: dict) -> tuple[Plan, PlanConfig]:
    cfg_c = data["config"]
    limits = KinematicLimits(*(_unhex(x) for x in cfg_c["limits"]))
    cfg = PlanConfig(
        limits,
        max_branch_points=int(cfg_c["max_branch_points"]),
        body_radius=_unhex(cfg_c["body_radius"]),
        tracking_margin=_unhex(cfg_c["tracking_margin"]),
        frame=str(cfg_c["frame"]),
        units=str(cfg_c["units"]),
    )
    branches = tuple(
        BrakeBranch(_unhex(b["t_branch"]), _traj_from_wire(b["traj"])) for b in data["branches"]
    )
    head = np.array([[_unhex(x) for x in row] for row in data["head_pva"]], dtype=float)
    plan = Plan(
        str(data["owner"]),
        int(data["plan_seq"]),
        str(data["parent_plan_hash"]),
        _unhex(data["t_switch"]),
        head,
        _traj_from_wire(data["motion"]),
        branches,
    )
    return plan, cfg


def _occupancy_from_wire(data: dict) -> PlanOccupancy:
    plan, cfg = _plan_from_wire(data["plan"])
    occ = PlanOccupancy(plan, cfg, _unhex(data["until"]))
    if occ.to_canonical() != data:
        raise RuntimeError("占用经总线往返后规范形式变了")
    return occ


def message_to_wire(msg) -> dict:
    """协议消息 → JSON 安全的 dict。浮点全部是 float.hex。"""
    data = {"type": msg.kind, "ctx": _ctx_wire(msg.ctx), "dest": msg.dest}
    if isinstance(msg, Heartbeat):
        return data
    if isinstance(msg, Proposal):
        data.update({
            "reservation_id": msg.reservation_id,
            "proposer_id": msg.proposer_id,
            "plan_seq": int(msg.plan_seq),
            "plan_hash": msg.plan_hash,
            "parent_plan_hash": msg.parent_plan_hash,
            "window_index": int(msg.window_index),
            "t_start": _hex(msg.t_start),
            "validity": [_hex(x) for x in msg.validity],
            "occupancy": msg.occupancy.to_canonical(),
            "dependency_digest": msg.dependency_digest,
        })
        return data
    if isinstance(msg, Vote):
        data.update({
            "reservation_id": msg.reservation_id,
            "proposer_id": msg.proposer_id,
            "plan_hash": msg.plan_hash,
            "validity": [_hex(x) for x in msg.validity],
            "voter_occupancy_version": msg.voter_occupancy_version,
            "dependency_digest": msg.dependency_digest,
            "decision": msg.decision.value,
            "reason": msg.reason.value,
        })
        return data
    if isinstance(msg, (Commit, Release)):
        data.update({
            "reservation_id": msg.reservation_id,
            "plan_hash": msg.plan_hash,
        })
        return data
    raise TypeError(f"不支持的协议消息 {type(msg).__name__}")


def message_from_wire(data: dict):
    ctx = _ctx_from_wire(data["ctx"])
    dest = data.get("dest")
    kind = data["type"]
    if kind == "HEARTBEAT":
        return Heartbeat(ctx=ctx, dest=dest)
    if kind == "PROPOSE":
        occ = _occupancy_from_wire(data["occupancy"])
        msg = Proposal(
            ctx=ctx,
            reservation_id=str(data["reservation_id"]),
            proposer_id=str(data["proposer_id"]),
            plan_seq=int(data["plan_seq"]),
            plan_hash=str(data["plan_hash"]),
            parent_plan_hash=str(data["parent_plan_hash"]),
            window_index=int(data["window_index"]),
            t_start=_unhex(data["t_start"]),
            validity=tuple(_unhex(x) for x in data["validity"]),
            occupancy=occ,
            dependency_digest=str(data["dependency_digest"]),
            dest=dest,
        )
        if msg.validity != (msg.t_start, occ.end_time()):
            raise RuntimeError(f"提案有效期 {msg.validity} 和占用 end_time {occ.end_time()} 不一致")
        if not occ.is_finite():
            raise RuntimeError(f"{msg.proposer_id} 的占用往返后不再满足计划契约")
        return msg
    if kind == "VOTE":
        return Vote(
            ctx=ctx,
            reservation_id=str(data["reservation_id"]),
            proposer_id=str(data["proposer_id"]),
            plan_hash=str(data["plan_hash"]),
            validity=tuple(_unhex(x) for x in data["validity"]),
            voter_occupancy_version=str(data["voter_occupancy_version"]),
            dependency_digest=str(data["dependency_digest"]),
            decision=Decision(data["decision"]),
            reason=Reason(data["reason"]),
            dest=dest,
        )
    if kind == "COMMIT":
        return Commit(
            ctx=ctx,
            reservation_id=str(data["reservation_id"]),
            plan_hash=str(data["plan_hash"]),
            dest=dest,
        )
    if kind == "RELEASE":
        return Release(
            ctx=ctx,
            reservation_id=str(data["reservation_id"]),
            plan_hash=str(data["plan_hash"]),
            dest=dest,
        )
    raise ValueError(f"不认识的协议消息类型 {kind!r}")


def _encode_payload(payload: dict) -> str:
    kind = payload.get("kind")
    if kind == "protocol":
        body = {"kind": "protocol", "msg": message_to_wire(payload["msg"])}
    elif kind in ("self_plan", "peer_plan"):
        body = {"kind": kind, **{k: v for k, v in payload.items() if k != "kind"}}
    else:
        raise TypeError(f"不支持的总线载荷 {kind!r}")
    return json.dumps(body, separators=(",", ":"), ensure_ascii=True)


def _decode_payload(text: str) -> dict:
    body = json.loads(text)
    kind = body.get("kind")
    if kind == "protocol":
        return {"kind": "protocol", "msg": message_from_wire(body["msg"])}
    if kind == "self_plan":
        return {"kind": "self_plan", "token": int(body["token"])}
    if kind == "peer_plan":
        return {"kind": "peer_plan", "code": str(body["code"])}
    raise TypeError(f"不支持的总线载荷 {kind!r}")


def encode_bus_item(item) -> list[str]:
    """一条总线信封：先 header，再载荷分片。分片上带 src/dest，转发不用等拼包。"""
    raw = _encode_payload(item.payload).encode("utf-8")
    chunks = [raw[i : i + OBS_CHUNK_BYTES] for i in range(0, len(raw), OBS_CHUNK_BYTES)]
    if len(chunks) + 1 > 64:
        raise RuntimeError(
            f"总线载荷 {len(raw)} 字节超过锁步队列（64 条，每条 {OBS_CHUNK_BYTES} 字节）。"
        )
    header = {
        "v": 1,
        "kind": "bus",
        "seq": int(item.seq),
        "src": item.src,
        "dest": item.dest,
        "topic": item.topic,
        "sent_at": _hex(item.sent_at),
        "deliver_at": _hex(item.deliver_at),
        "rank": int(item.rank),
        "n_chunks": len(chunks),
    }
    messages = [json.dumps(header, separators=(",", ":"), ensure_ascii=True)]
    for i, chunk in enumerate(chunks):
        messages.append(json.dumps({
            "v": 1,
            "kind": "bus_chunk",
            "seq": int(item.seq),
            "src": item.src,
            "dest": item.dest,
            "i": i,
            "b64": base64.b64encode(chunk).decode("ascii"),
        }, separators=(",", ":"), ensure_ascii=True))
    return messages


def decode_bus_messages(messages: list[str]) -> dict:
    """拼回信封。payload 是新对象，不复用发送侧那一份。"""
    if not messages:
        raise ValueError("总线消息是空的")
    header = json.loads(messages[0])
    if header.get("v") != 1 or header.get("kind") != "bus":
        raise ValueError("第一条总线消息不是 v1 bus header")
    n_chunks = int(header["n_chunks"])
    if len(messages) != 1 + n_chunks:
        raise ValueError(f"总线分片数量不对：header 说 {n_chunks}，实际 {len(messages) - 1}")
    parts: list[bytes | None] = [None] * n_chunks
    for text in messages[1:]:
        piece = json.loads(text)
        if piece.get("kind") != "bus_chunk" or int(piece["seq"]) != int(header["seq"]):
            raise ValueError("总线分片的 seq 和 header 不一致")
        if piece.get("dest") != header["dest"] or piece.get("src") != header["src"]:
            raise ValueError("总线分片的 src/dest 和 header 不一致")
        idx = int(piece["i"])
        if idx < 0 or idx >= n_chunks or parts[idx] is not None:
            raise ValueError(f"总线分片序号重复或越界：{idx}")
        parts[idx] = base64.b64decode(piece["b64"])
    blob = b"".join(parts)  # type: ignore[arg-type]
    return {
        "seq": int(header["seq"]),
        "src": str(header["src"]),
        "dest": str(header["dest"]),
        "topic": str(header["topic"]),
        "sent_at": _unhex(header["sent_at"]),
        "deliver_at": _unhex(header["deliver_at"]),
        "rank": int(header["rank"]),
        "payload": _decode_payload(blob.decode("utf-8")),
    }
