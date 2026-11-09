"""协议消息。字段对应方案书 V0.3 第 8 节的版本上下文与预留上下文。"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, fields
from enum import Enum
from typing import Any, ClassVar

from yopo_swarm_core.protocol.occupancy import _num


class Decision(Enum):
    ACK = "ACK"
    NACK_CONFLICT = "NACK_CONFLICT"
    NACK_STALE = "NACK_STALE"


class Reason(Enum):
    OK = "OK"
    WINDOW = "WINDOW"
    LATE = "LATE"
    CONFLICT = "CONFLICT"
    HALTED = "HALTED"
    CLOCK_EPOCH = "CLOCK_EPOCH"
    MEMBER_EPOCH = "MEMBER_EPOCH"
    TASK_HASH = "TASK_HASH"
    UNKNOWN_SENDER = "UNKNOWN_SENDER"
    STALE_BOOT = "STALE_BOOT"
    PEER_RESTARTED = "PEER_RESTARTED"
    SEQ_REGRESSION = "SEQ_REGRESSION"
    MALFORMED = "MALFORMED"
    UNKNOWN_RESERVATION = "UNKNOWN_RESERVATION"
    VOTE_MISMATCH = "VOTE_MISMATCH"
    DEADLINE = "DEADLINE"
    NACKED = "NACKED"
    NO_CANDIDATE = "NO_CANDIDATE"
    TOO_MANY_FAILURES = "TOO_MANY_FAILURES"
    SELF_RESET = "SELF_RESET"


def canonical(obj: Any) -> Any:
    if isinstance(obj, Enum):
        return obj.value
    if isinstance(obj, float):
        return _num(obj) if math.isfinite(obj) or math.isinf(obj) else "nan"
    if isinstance(obj, tuple | list):
        return [canonical(x) for x in obj]
    if isinstance(obj, dict):
        return {str(k): canonical(v) for k, v in sorted(obj.items())}
    if hasattr(obj, "to_canonical"):
        return obj.to_canonical()
    if hasattr(obj, "__dataclass_fields__"):
        return {f.name: canonical(getattr(obj, f.name)) for f in fields(obj)}
    return obj


def digest(obj: Any) -> str:
    text = json.dumps(canonical(obj), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode()).hexdigest()


@dataclass(frozen=True)
class VersionContext:
    member_epoch: int
    task_hash: str
    clock_epoch: int
    sender_boot_id: int
    sender_id: str
    message_seq: int


@dataclass(frozen=True)
class Message:
    kind: ClassVar[str] = "?"

    def to_json(self) -> dict:
        return {"kind": self.kind, **canonical(self)}

    def numbers(self) -> list[float]:
        return []


@dataclass(frozen=True)
class Heartbeat(Message):
    kind: ClassVar[str] = "HEARTBEAT"
    ctx: VersionContext
    dest: str | None = None


@dataclass(frozen=True)
class Proposal(Message):
    kind: ClassVar[str] = "PROPOSE"
    ctx: VersionContext
    reservation_id: str
    proposer_id: str
    plan_seq: int
    plan_hash: str
    parent_plan_hash: str
    window_index: int
    t_start: float
    validity: tuple[float, float]
    occupancy: Any
    dependency_digest: str
    dest: str | None = None

    def numbers(self) -> list[float]:
        return [self.t_start, self.validity[0]]


@dataclass(frozen=True)
class Vote(Message):
    kind: ClassVar[str] = "VOTE"
    ctx: VersionContext
    reservation_id: str
    proposer_id: str
    plan_hash: str
    validity: tuple[float, float]
    voter_occupancy_version: str
    dependency_digest: str
    decision: Decision
    reason: Reason
    dest: str | None = None


@dataclass(frozen=True)
class Commit(Message):
    kind: ClassVar[str] = "COMMIT"
    ctx: VersionContext
    reservation_id: str
    plan_hash: str
    dest: str | None = None


@dataclass(frozen=True)
class Release(Message):
    kind: ClassVar[str] = "RELEASE"
    ctx: VersionContext
    reservation_id: str
    plan_hash: str
    dest: str | None = None
