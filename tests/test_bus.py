"""票 05：总线外部行为。丢包、乱序必须各有一条；时延和重复一起锁住旋钮。"""

import pytest

from yopo_l0.bus import BusFaults, MessageBus


class _Script:
    """按给定序列回放 random()/uniform() 的单位区间采样。"""

    def __init__(self, values):
        self._values = iter(values)

    def random(self):
        return float(next(self._values))

    def uniform(self, low, high):
        u = float(next(self._values))
        return float(low) + (float(high) - float(low)) * u


def _send_pair(bus: MessageBus):
    bus.publish(0.0, src="uav0", dest="uav0", topic="plan", payload={"n": 1})
    bus.publish(0.0, src="uav0", dest="uav0", topic="plan", payload={"n": 2})


def test_loss_drops_the_message():
    bus = MessageBus(BusFaults(drop=1.0), seed=0)
    bus.publish(0.0, src="uav0", dest="uav1", topic="plan", payload={"n": 1})
    assert bus.poll(0.0) == []
    assert bus.poll(10.0) == []
    assert bus.counts() == {
        "published": 1, "dropped": 1, "duplicated": 0, "delivered": 0, "reordered": 0,
    }


def test_partial_loss_is_seed_deterministic():
    faults = BusFaults(drop=0.5)
    a = MessageBus(faults, seed=4)
    b = MessageBus(faults, seed=4)
    for i in range(8):
        payload = {"n": i}
        a.publish(0.0, src="uav0", dest="uav1", topic="plan", payload=payload)
        b.publish(0.0, src="uav0", dest="uav1", topic="plan", payload=payload)
    got_a = [(m.seq, m.payload["n"]) for m in a.poll(0.0)]
    got_b = [(m.seq, m.payload["n"]) for m in b.poll(0.0)]
    assert got_a == got_b
    assert a.counts() == b.counts()
    assert 0 < a.counts()["dropped"] < 8
    assert a.counts()["delivered"] == 8 - a.counts()["dropped"]


def test_reorder_delivers_later_message_first():
    # reorder=1 且时延为 0：第二条抢到队首，投递序与发送序相反。
    bus = MessageBus(BusFaults(reorder=1.0), seed=0)
    _send_pair(bus)
    got = bus.poll(0.0)
    assert [m.payload["n"] for m in got] == [2, 1]
    assert bus.counts()["reordered"] == 1
    assert bus.counts()["delivered"] == 2


def test_delay_holds_then_reorders_when_the_later_send_is_sooner():
    # 固定脚本：第一条时延 0.9 s，第二条 0.1 s。后发先到。
    bus = MessageBus(BusFaults(delay=(0.0, 1.0)), rng=_Script([0.9, 0.1]))
    _send_pair(bus)
    assert bus.poll(0.0) == []
    early = bus.poll(0.1)
    assert [m.payload["n"] for m in early] == [2]
    assert bus.counts()["reordered"] == 0  # 还没见到被它超过的那条
    late = bus.poll(0.9)
    assert [m.payload["n"] for m in late] == [1]
    assert bus.counts()["reordered"] == 1


def test_fixed_delay_is_not_available_early():
    bus = MessageBus(BusFaults(delay=(0.5, 0.5)), seed=1)
    bus.publish(0.0, src="a", dest="b", topic="plan", payload={"n": 1})
    assert bus.poll(0.5 - 1e-6) == []
    got = bus.poll(0.5)
    assert [m.payload["n"] for m in got] == [1]
    assert bus.counts()["reordered"] == 0


def test_duplicate_delivers_two_independent_copies():
    bus = MessageBus(BusFaults(duplicate=1.0), seed=0)
    payload = {"position": [1.0, 2.0, 3.0]}
    bus.publish(0.0, src="uav0", dest="uav0", topic="plan", payload=payload)
    payload["position"][0] = 9.0  # 入队后改调用方的字典，在途拷贝不变
    got = bus.poll(0.0)
    assert len(got) == 2
    assert bus.counts()["duplicated"] == 1
    assert got[0].payload["position"][0] == 1.0
    got[0].payload["position"][0] = 7.0
    assert got[1].payload["position"] == [1.0, 2.0, 3.0]


def test_zero_fault_keeps_send_order_and_same_seed_matches():
    faults = BusFaults()
    first = MessageBus(faults, seed=2)
    second = MessageBus(faults, seed=2)
    for bus in (first, second):
        _send_pair(bus)
    assert [m.payload["n"] for m in first.poll(0.0)] == [1, 2]
    assert [m.payload["n"] for m in second.poll(0.0)] == [1, 2]
    assert first.counts()["reordered"] == 0
    assert first.counts() == second.counts()


def test_fault_knobs_reject_out_of_range():
    with pytest.raises(ValueError):
        BusFaults(drop=1.1)
    with pytest.raises(ValueError):
        BusFaults(delay=(0.2, 0.1))
    with pytest.raises(ValueError):
        BusFaults(reorder=-0.1)
