"""票 06b：同步出发的 ACK 丢掉以后，该机沿本槽追赶，只飞自己提交的计划。

随机丢包 + 乱序未冻结，只把到达率、追赶时间、队形误差 p95、最小机间距离打出来。
"""

import numpy as np
import pytest

from yopo_l0.bus import BusFaults, MessageBus
from yopo_l0.formation import FormationFleet
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card
from yopo_swarm_core.protocol.messages import Decision, Vote

from conftest import ROOT

CARD = ROOT / "scenes/formation-hold.json"
DRONES = ("uav0", "uav1", "uav2")
FAULT_SCENES = {
    "formation-drop": {"drop": 0.1},
    "formation-duplicate": {"duplicate": 0.5},
    "formation-delay": {"delay": (0.01, 0.05)},
    "formation-reorder": {"reorder": 1.0},
}


def _run(card, **kw):
    return run(card, RunConfig(**kw))


def _drop_departure_acks(monkeypatch, drone_id):
    """只丢掉该机同步出发提案（reservation …/0/1）的 ACK，其余消息照常投递。"""
    rid = f"{drone_id}/0/1"
    orig = MessageBus.publish

    def publish(self, now, *, src, dest, topic, payload):
        msg = payload.get("msg") if isinstance(payload, dict) else None
        if (
            isinstance(msg, Vote)
            and msg.decision is Decision.ACK
            and msg.reservation_id == rid
            and dest == drone_id
        ):
            self.published += 1
            self.dropped += 1
            return None
        return orig(self, now, src=src, dest=dest, topic=topic, payload=payload)

    monkeypatch.setattr(MessageBus, "publish", publish)


def test_catchup_messages_go_through_transport_override(monkeypatch):
    """票 10b 的话题传输要带上超时 RELEASE 和追赶提案，不能退回另一条总线。"""
    card = load_card(CARD)
    seen = []

    class Transport:
        def __init__(self):
            self.bus = None

        def carrying(self, bus):
            if self.bus is not None:
                return self.bus
            orig = bus.publish

            def publish(now, *, src, dest, topic, payload):
                seen.append(payload.get("msg"))
                return orig(now, src=src, dest=dest, topic=topic, payload=payload)

            bus.publish = publish
            self.bus = bus
            return bus

    FormationFleet.transport_override = Transport()
    try:
        _drop_departure_acks(monkeypatch, "uav0")
        out = _run(card, seed=0, shadow=False)
    finally:
        FormationFleet.transport_override = None
    assert out.result["outcome"] == "SUCCESS", out.result
    assert out.result["arrived"] == 3
    proposals = [m for m in seen if getattr(m, "kind", None) == "PROPOSE" and m.proposer_id == "uav0"]
    assert any(m.plan_seq >= 2 and m.t_start > out.result["formation"]["t_go_s"] for m in proposals)
    assert any(getattr(m, "kind", None) == "RELEASE" and m.ctx.sender_id == "uav0" for m in seen)


@pytest.mark.parametrize("drone_id", DRONES)
def test_dropped_departure_ack_rejoins(monkeypatch, drone_id):
    card = load_card(CARD)
    limits = FormationFleet.from_card(card.card, 0.01).pcfg.limits
    radius = float(card.card["criteria"]["body_radius_m"])
    _drop_departure_acks(monkeypatch, drone_id)
    out = _run(card, seed=0)
    r = out.result
    form = r["formation"]
    assert r["outcome"] == "SUCCESS", r
    assert r["arrived"] == 3 and form["arrival_rate"] == 1.0
    assert r["min_inter_drone_distance_m"] > 2.0 * radius
    assert form["local_aborts"][drone_id] >= 1
    assert form["deadline_aborts"][drone_id] == form["local_aborts"][drone_id]
    assert form["local_commits"][drone_id] >= 1
    assert form["committed_t_switch_s"][drone_id] > form["t_go_s"]
    # 这一轮的 ACK 全丢。窗口内补齐请求会再要一次，仍然拿不到，才超时 RELEASE 走追赶。
    assert 1 <= form["fill_requests"][drone_id] <= FormationFleet.FILL_MAX
    for other in DRONES:
        if other == drone_id:
            continue
        assert form["local_aborts"][other] == 0
        assert form["fill_requests"][other] == 0
        assert form["local_commits"][other] == 1
        assert form["committed_t_switch_s"][other] <= form["t_go_s"] + 1e-9
    assert r["bus"]["dropped"] == (len(DRONES) - 1) * (1 + form["fill_requests"][drone_id])
    for row in out.ticks:
        assert row["approved_owner"] == row["uav"]
        assert np.linalg.norm(row["cmd_v"]) <= limits.v_max + 1e-6
        assert np.linalg.norm(row["cmd_a"]) <= limits.a_max + 1e-6


def _drop_first_ack(monkeypatch):
    """每个 (预留, 投票方) 的第一张 ACK 丢掉，随后的补齐请求照常送达。"""
    seen: set[tuple] = set()
    orig = MessageBus.publish

    def publish(self, now, *, src, dest, topic, payload):
        msg = payload.get("msg") if isinstance(payload, dict) else None
        if isinstance(msg, Vote) and msg.decision is Decision.ACK:
            key = (msg.reservation_id, src, dest)
            if key not in seen:
                seen.add(key)
                self.published += 1
                self.dropped += 1
                return None
        return orig(self, now, src=src, dest=dest, topic=topic, payload=payload)

    monkeypatch.setattr(MessageBus, "publish", publish)


def test_fill_request_commits_inside_the_departure_window(monkeypatch):
    """首张 ACK 丢失时，同一 reservation 在 t_start 前补齐，不改成追赶计划。"""
    card = load_card(CARD)
    _drop_first_ack(monkeypatch)
    out = _run(card, seed=0, duration_s=2.0, shadow=False)
    r = out.result
    form = r["formation"]
    assert form["local_commits"] == {did: 1 for did in DRONES}
    assert form["local_aborts"] == {did: 0 for did in DRONES}
    assert form["deadline_aborts"] == {did: 0 for did in DRONES}
    assert all(form["fill_requests"][did] >= 1 for did in DRONES)
    assert all(form["committed_t_switch_s"][did] <= form["t_go_s"] + 1e-9 for did in DRONES)
    assert r["bus"]["dropped"] == 2 * len(DRONES)
    for row in out.ticks:
        assert row["approved_owner"] == row["uav"]
        assert row["approved_seq"] in (0, 1)


def test_random_drop_reorder_reports_catchup():
    """10 个 seed、10% 丢包 + 乱序。到达率不设门，数字写在标准输出。"""
    card = load_card(CARD)
    faults = BusFaults(drop=0.1, reorder=1.0)
    rows = []
    for seed in range(10):
        out = _run(card, seed=seed, bus=faults)
        r = out.result
        form = r["formation"]
        rows.append({
            "seed": seed,
            "outcome": r["outcome"],
            "arrived": r["arrived"],
            "arrival_rate": form["arrival_rate"],
            "catchup_time_s": form["catchup_time_s"],
            "error_p95_m": form["error_p95_m"],
            "min_inter_m": r["min_inter_drone_distance_m"],
            "dropped": r["bus"]["dropped"],
            "reordered": r["bus"]["reordered"],
        })
    n = len(rows)
    arrival = sum(row["arrived"] for row in rows) / (n * 3)
    catch = sum(row["catchup_time_s"] for row in rows) / n
    p95 = [row["error_p95_m"] for row in rows if row["error_p95_m"] is not None]
    dist = [row["min_inter_m"] for row in rows if row["min_inter_m"] is not None]
    lines = [
        "票 06b 报告（10 seed，drop=0.1，reorder=1；未冻结 / report-only）",
        f"到达率 {arrival:.3f}  追赶时间均值 {catch:.3f} s  "
        f"队形误差 p95 均值 {(sum(p95) / len(p95)) if p95 else float('nan'):.4f} m  "
        f"最小机间距离 {min(dist) if dist else float('nan'):.3f} m",
        "seed  outcome  arrived  catchup_s  error_p95_m  min_inter_m",
    ]
    for row in rows:
        p95_text = "—" if row["error_p95_m"] is None else f"{row['error_p95_m']:.4f}"
        dist_text = "—" if row["min_inter_m"] is None else f"{row['min_inter_m']:.3f}"
        lines.append(
            f"{row['seed']:>4}  {row['outcome']:<8}  {row['arrived']:>7}  "
            f"{row['catchup_time_s']:>9.3f}  {p95_text:>11}  {dist_text:>11}"
        )
    report = "\n".join(lines)
    print(report)
    assert report.startswith("票 06b 报告")
    assert sum(row["dropped"] for row in rows) > 0
    assert sum(row["reordered"] for row in rows) > 0
    for row in rows:
        assert row["outcome"] in {"SUCCESS", "TIMEOUT", "COLLISION"}
        assert 0.0 <= row["arrival_rate"] <= 1.0
        assert row["catchup_time_s"] >= 0.0
        assert row["error_p95_m"] is None or row["error_p95_m"] >= 0.0
        assert row["min_inter_m"] is None or row["min_inter_m"] >= 0.0


def test_fault_scenes_expose_bus_knobs():
    for name, expect in FAULT_SCENES.items():
        card = load_card(ROOT / f"scenes/{name}.json")
        faults = BusFaults.from_mapping(card.card["bus"])
        assert card.card["task"]["type"] == "formation"
        assert card.card["criteria"]["frozen"] is False
        for key, value in expect.items():
            assert getattr(faults, key) == value
    listed = (ROOT / "scenes/suite.txt").read_text()
    for name in FAULT_SCENES:
        assert f"scenes/{name}.json@formation" in listed
