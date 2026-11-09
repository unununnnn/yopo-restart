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
    assert form["local_commits"][drone_id] >= 1
    assert form["committed_t_switch_s"][drone_id] > form["t_go_s"]
    for other in DRONES:
        if other == drone_id:
            continue
        assert form["local_aborts"][other] == 0
        assert form["local_commits"][other] == 1
        assert form["committed_t_switch_s"][other] <= form["t_go_s"] + 1e-9
    assert r["bus"]["dropped"] == 2
    for row in out.ticks:
        assert row["approved_owner"] == row["uav"]
        assert np.linalg.norm(row["cmd_v"]) <= limits.v_max + 1e-6
        assert np.linalg.norm(row["cmd_a"]) <= limits.a_max + 1e-6


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
