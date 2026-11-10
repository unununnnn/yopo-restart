"""票 11a：六机开阔区楔形编队。进程内、零故障要能到终点。

全丢（drop=1）仍然悬停：窗口内重发也收不到 ACK。20% 丢包的补票是票 06c。
"""

import json

import numpy as np

from yopo_l0.batch import resolve_tiers
from yopo_l0.bus import BusFaults
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import Ground, load_card

from conftest import ROOT

CARD = ROOT / "scenes/formation-6-open.json"
TIERS = ROOT / "scenes/formation-6-tiers.json"


def _strip(result):
    return {k: v for k, v in result.items() if k not in WALL_FIELDS}


def test_scene_is_open_wedge_with_seeds_0_through_4():
    raw = json.loads(CARD.read_text())
    assert raw["geometry"] == "open-area.json"
    assert raw["seeds"] == [0, 1, 2, 3, 4]
    assert raw["criteria"]["frozen"] is False
    assert raw["task"]["type"] == "formation"
    assert [d["id"] for d in raw["drones"]] == [f"uav{i}" for i in range(6)]
    card = load_card(CARD)
    assert all(isinstance(o, Ground) for o in card.obstacles)
    slots = [np.asarray(d["slot_body"], dtype=float) for d in raw["drones"]]
    # 楔形：前两架、中间两架、后两架，横向拉开，没有两架落在同一条航迹上。
    assert slots[0][0] == slots[1][0] == 0.0
    assert slots[2][0] == slots[3][0] < 0.0
    assert slots[4][0] == slots[5][0] < slots[2][0]
    ys = [s[1] for s in slots]
    assert len(set(np.round(ys, 6))) == 6
    for i, a in enumerate(slots):
        for b in slots[i + 1 :]:
            assert float(np.linalg.norm(a - b)) >= 3.0 - 1e-9
    for drone in raw["drones"]:
        start = np.asarray(drone["start_enu"], dtype=float)
        assert card.min_distance(start) > raw["criteria"]["body_radius_m"]


def test_tier_list_matches_bus_faults():
    tiers = resolve_tiers(str(TIERS))
    assert [t["id"] for t in tiers] == [
        "none", "drop-5", "drop-20", "duplicate-10", "reorder-delay", "combined",
    ]
    assert [t["label"] for t in tiers] == [
        "none", "drop 5%", "drop 20%", "duplicate 10%", "reorder+delay", "combined",
    ]
    by_id = {t["id"]: BusFaults.from_mapping(t["bus"]) for t in tiers}
    assert by_id["none"] == BusFaults()
    assert by_id["drop-5"] == BusFaults(drop=0.05)
    assert by_id["drop-20"] == BusFaults(drop=0.20)
    assert by_id["duplicate-10"] == BusFaults(duplicate=0.10)
    assert by_id["reorder-delay"] == BusFaults(delay=(0.01, 0.05), reorder=1.0)
    assert by_id["combined"] == BusFaults(drop=0.05, duplicate=0.10, delay=(0.01, 0.05), reorder=1.0)
    assert [t["id"] for t in resolve_tiers("drop-20,none")] == ["drop-20", "none"]


def test_open_six_none_reaches_and_is_deterministic():
    card = load_card(CARD)
    out = run(card, RunConfig(seed=0, policy="formation"))
    again = run(card, RunConfig(seed=0, policy="formation"))
    r = out.result
    assert r["outcome"] == "SUCCESS", r
    assert r["n_drones"] == 6 and r["arrived"] == 6
    assert r["arrival_rate"] == 1.0
    assert r["outcome"] != "COLLISION"
    assert r["formation_error_p95_m"] < 0.2
    assert r["min_inter_drone_distance_m"] > 2.5
    assert r["formation"]["local_commits"] == {f"uav{i}": 1 for i in range(6)}
    assert r["formation"]["adjudicator"] is None
    assert isinstance(r["shadow_reject_rate"], float)
    assert r["wall_time_s"] < 60.0
    assert _strip(r) == _strip(again.result)
    assert out.ticks == again.ticks


def test_dropped_ack_is_recovered_by_resending_the_same_proposal(monkeypatch):
    """第一轮两张 ACK 都丢掉。0.02 s 后重发同一份提案，本窗提交，不开新的 plan_seq。"""
    from yopo_l0.bus import MessageBus
    from yopo_l0.formation import FormationFleet
    from yopo_swarm_core.protocol.messages import Decision, Vote

    fleet = FormationFleet.from_card(load_card(ROOT / "scenes/formation-hold.json").card, 0.01)
    bus = MessageBus()
    dropped = {"n": 0}
    orig = MessageBus.publish

    def publish(self, now, *, src, dest, topic, payload):
        msg = payload.get("msg") if isinstance(payload, dict) else None
        if (
            isinstance(msg, Vote)
            and msg.decision is Decision.ACK
            and msg.reservation_id == "uav0/0/1"
            and dropped["n"] < 2
        ):
            dropped["n"] += 1
            self.published += 1
            self.dropped += 1
            return None
        return orig(self, now, src=src, dest=dest, topic=topic, payload=payload)

    monkeypatch.setattr(MessageBus, "publish", publish)
    fleet.exchange(bus, 0.0)
    node = fleet.nodes["uav0"]
    assert node.committed.plan_seq == 0 and node.in_flight is not None and node.plan_seq == 1
    fleet.exchange(bus, 0.02)
    assert dropped["n"] == 2
    assert node.in_flight is None and node.committed.plan_seq == 1
    assert node.committed.t_start == fleet.cfg.window_len


def test_open_six_resends_the_same_proposal_inside_the_window():
    from yopo_l0.formation import FormationFleet

    fleet = FormationFleet.from_card(load_card(CARD).card, 0.01)
    assert fleet.proposal_resend_period == 0.02
    assert fleet.proposal_resend_guard == 0.10
    assert fleet.cfg.resend_period == fleet.cfg.window_len
    assert "proposal_resend_period" not in fleet.cfg.__dataclass_fields__


def test_total_loss_leaves_the_wedge_hovering():
    # drop=1 时重发也到不了。飞机停在起点，不是规划器把追赶关掉了。
    card = load_card(CARD)
    out = run(card, RunConfig(seed=0, duration_s=2.0, bus=BusFaults(drop=1.0)))
    r = out.result
    assert r["outcome"] == "TIMEOUT"
    assert r["arrival_rate"] == 0.0
    assert r["formation"]["local_commits"] == {f"uav{i}": 0 for i in range(6)}
    assert r["bus"]["dropped"] == r["bus"]["published"] > 0
    starts = {d["id"]: d["start_enu"] for d in card.card["drones"]}
    for did, start in starts.items():
        last = [row["p"] for row in out.ticks if row["uav"] == did][-1]
        assert np.linalg.norm(np.asarray(last) - np.asarray(start)) < 0.05
