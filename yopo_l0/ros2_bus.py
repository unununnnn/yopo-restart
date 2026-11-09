"""票 10b：和 MessageBus 同一套故障抽样，但入队前先把信封交给 relay。

relay 由 ROS 形态实现：源节点话题 → 目的节点话题 → 世界。它返回的对象才进锁步队列，
所以 poll 交出去的载荷是话题上回来的那一份，不是进程内深拷贝。
时延仍然只写在 deliver_at 上，relay 不睡墙钟。
"""

from __future__ import annotations

from .bus import MessageBus


class _RelayQueue(list):
    """append 时走 relay。list.extend / list.append 不触发，避免 poll 之后把剩余项再送一遍。"""

    def __init__(self, relay):
        super().__init__()
        self._relay = relay

    def append(self, item):
        list.append(self, self._relay(item))


class Ros2MessageBus(MessageBus):
    """故障与计数跟 MessageBus 相同。队列里只放 relay 交还的信封。"""

    def __init__(self, faults=None, *, relay, rng=None, seed: int = 0):
        super().__init__(faults, rng=rng, seed=seed)
        self._relay = relay
        self.mirror = None  # 运行器手里的那条总线。只抄计数，不让它入队。
        self._q = _RelayQueue(relay)

    def publish(self, now, *, src, dest, topic, payload):
        self._ensure_queue()
        super().publish(now, src=src, dest=dest, topic=topic, payload=payload)
        self._mirror_counts()

    def poll(self, now):
        out = super().poll(now)
        # poll 会把 _q 换成普通 list。下一跳 append 必须还在 RelayQueue 上。
        self._ensure_queue()
        self._mirror_counts()
        return out

    def _ensure_queue(self) -> None:
        if isinstance(self._q, _RelayQueue):
            return
        kept = list(self._q)
        q = _RelayQueue(self._relay)
        for item in kept:
            list.append(q, item)
        self._q = q

    def _mirror_counts(self) -> None:
        host = self.mirror
        if host is None:
            return
        host.published = self.published
        host.dropped = self.dropped
        host.duplicated = self.duplicated
        host.delivered = self.delivered
        host.reordered = self.reordered


class Ros2Transport:
    """FormationFleet.transport_override 指向它。carrying 在第一次 exchange 时接上运行器的随机流。"""

    def __init__(self) -> None:
        self.relay = None
        self.inner: Ros2MessageBus | None = None
        self.topics: list[str] = []

    def carrying(self, host: MessageBus) -> Ros2MessageBus:
        if self.relay is None:
            raise RuntimeError("ROS 机间传输还没有接上节点")
        if self.inner is None:
            self.inner = Ros2MessageBus(host.faults, rng=host.rng, relay=self.relay)
        elif self.inner.rng is not host.rng:
            raise RuntimeError("同一次运行里机间传输接到了另一条总线")
        self.inner.mirror = host
        return self.inner
