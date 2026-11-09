"""L0 进程内消息总线。机间消息只走这里，不共享对象。

故障旋钮对齐 G3 离散事件模型 `yopo_swarm_core.protocol.des.FaultPlan`
（Simulator 的 drop / duplicate / delay）以及 explore 里的 reorder
（可以不按队首投递）。未接入分区、重启、时钟暂停/回拨、任务重置——
那些作用在协议节点上，留给票 06。

旋钮（`BusFaults`）
- `drop`：每条逻辑消息被整段丢弃的概率，∈ [0, 1]。对应 `FaultPlan.drop`。
  先掷丢包，再决定是否复制；丢了就没有拷贝。
- `duplicate`：未丢弃时额外再入队一份拷贝的概率，∈ [0, 1]。对应
  `FaultPlan.duplicate`。两份拷贝各自采样时延、各自掷乱序。
- `delay`：`(lo, hi)` 秒。`hi == lo` 时为固定时延，不消耗随机数；
  `hi > lo` 时在区间上均匀采样。对应 `FaultPlan.delay`（G3 默认
  `(0.01, 0.05)`，注释写明随机时延天然产生乱序）。后发的消息可以更早到期。
  L0 默认 `(0, 0)`，这样零故障时本机计划在同一 tick 投递，单机悬停轨迹不变。
- `reorder`：概率 ∈ [0, 1]。命中且已有在途消息时，把本拷贝的投递序号与当前
  最早的在途消息对调，使它抢到队首（相同时延下也不再保持发送序）。
  对应 G3 explore 的 `"reorder"` 故障（不保证只投递队首）；这里取「始终插到
  当前队首」这一种可复现特例，而不是在袋中均匀抽一条。`reorder=1` 时，
  同一时刻发出的一串消息会按发送序的反序投递。

到期条件是 `deliver_at <= now`（加 1e-12 以吸收仿真步长的二进制误差）。
同一时刻到期的消息按投递序号升序交出。投递序号相对已交出的最大序号倒退，
记一次 `reordered`（时延造成的抢先和 reorder 旋钮造成的插队都算）。

payload 在入队和投递时各深拷贝一次。调用方事后修改字典，或接收方修改
投递结果，都不会改到仍在途的拷贝，也不会改到别的无人机手里的那份。
随机流由调用方注入；运行器使用与传感器 SeedSequence 分开的生成器，
避免拨故障旋钮就改变深度噪声。
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Protocol

import numpy as np


class _Rng(Protocol):
    def random(self) -> float: ...

    def uniform(self, low: float, high: float) -> float: ...


@dataclass(frozen=True)
class BusFaults:
    drop: float = 0.0
    duplicate: float = 0.0
    delay: tuple[float, float] = (0.0, 0.0)
    reorder: float = 0.0

    def __post_init__(self) -> None:
        for name in ("drop", "duplicate", "reorder"):
            value = getattr(self, name)
            if not 0.0 <= float(value) <= 1.0:
                raise ValueError(f"{name} 必须在 [0, 1] 内，收到 {value}")
        lo, hi = self.delay
        if float(lo) < 0.0 or float(hi) < float(lo):
            raise ValueError(f"delay 必须满足 0 <= lo <= hi，收到 {self.delay}")

    @classmethod
    def from_mapping(cls, raw: dict | None) -> "BusFaults":
        if not raw:
            return cls()
        delay = raw.get("delay", (0.0, 0.0))
        return cls(
            drop=float(raw.get("drop", 0.0)),
            duplicate=float(raw.get("duplicate", 0.0)),
            delay=(float(delay[0]), float(delay[1])),
            reorder=float(raw.get("reorder", 0.0)),
        )


@dataclass(frozen=True)
class Envelope:
    seq: int
    src: str
    dest: str
    topic: str
    payload: dict
    sent_at: float
    deliver_at: float


@dataclass
class _Item:
    deliver_at: float
    rank: int
    seq: int
    src: str
    dest: str
    topic: str
    payload: dict
    sent_at: float


class MessageBus:
    """单进程队列。不知道无人机列表；调用方按目的地逐条 publish。"""

    def __init__(self, faults: BusFaults | None = None, *, rng: _Rng | None = None, seed: int = 0):
        self.faults = faults or BusFaults()
        self.rng: _Rng = rng if rng is not None else np.random.default_rng(seed)
        self._q: list[_Item] = []
        self._seq = 0
        self._max_delivered_seq = -1
        self.published = 0
        self.dropped = 0
        self.duplicated = 0
        self.delivered = 0
        self.reordered = 0

    def publish(self, now: float, *, src: str, dest: str, topic: str, payload: dict) -> None:
        self.published += 1
        faults = self.faults
        if faults.drop > 0.0 and float(self.rng.random()) < faults.drop:
            self.dropped += 1
            return
        copies = 2 if faults.duplicate > 0.0 and float(self.rng.random()) < faults.duplicate else 1
        self.duplicated += copies - 1
        lo, hi = faults.delay
        for _ in range(copies):
            delay = float(self.rng.uniform(lo, hi)) if hi > lo else float(lo)
            seq = self._seq
            self._seq += 1
            item = _Item(
                deliver_at=float(now) + delay,
                rank=seq,
                seq=seq,
                src=src,
                dest=dest,
                topic=topic,
                payload=copy.deepcopy(payload),
                sent_at=float(now),
            )
            if faults.reorder > 0.0 and self._q and float(self.rng.random()) < faults.reorder:
                j = min(range(len(self._q)), key=lambda i: (self._q[i].rank, self._q[i].seq))
                item.rank, self._q[j].rank = self._q[j].rank, item.rank
            self._q.append(item)

    def poll(self, now: float) -> list[Envelope]:
        due = [it for it in self._q if it.deliver_at <= float(now) + 1e-12]
        if not due:
            return []
        due.sort(key=lambda it: (it.rank, it.seq))
        due_ids = {id(it) for it in due}
        self._q = [it for it in self._q if id(it) not in due_ids]
        out: list[Envelope] = []
        for it in due:
            if it.seq < self._max_delivered_seq:
                self.reordered += 1
            if it.seq > self._max_delivered_seq:
                self._max_delivered_seq = it.seq
            self.delivered += 1
            out.append(
                Envelope(
                    seq=it.seq,
                    src=it.src,
                    dest=it.dest,
                    topic=it.topic,
                    payload=copy.deepcopy(it.payload),
                    sent_at=it.sent_at,
                    deliver_at=it.deliver_at,
                )
            )
        return out

    def counts(self) -> dict:
        return {
            "published": self.published,
            "dropped": self.dropped,
            "duplicated": self.duplicated,
            "delivered": self.delivered,
            "reordered": self.reordered,
        }
