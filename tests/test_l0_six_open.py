"""票 11a：六机开阔区楔形编队。进程内、零故障要能到终点。丢包仍会悬停，那是票 06b。"""

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
    assert r["formation"]["fill_requests"] == {f"uav{i}": 0 for i in range(6)}
    assert r["formation"]["adjudicator"] is None
    assert isinstance(r["shadow_reject_rate"], float)
    assert r["wall_time_s"] < 60.0
    assert _strip(r) == _strip(again.result)
    assert out.ticks == again.ticks


def test_drop20_seed0_commits_the_departure_plan():
    """票 06c：drop 20% 的 seed 0 原先有飞机停在起点。补齐后六机都在 t_go 前提交自己的计划。"""
    card = load_card(CARD)
    out = run(card, RunConfig(seed=0, policy="formation", shadow=False, bus=BusFaults(drop=0.20)))
    r = out.result
    form = r["formation"]
    assert r["outcome"] == "SUCCESS", r
    assert r["arrived"] == 6 and form["arrival_rate"] == 1.0
    assert r["outcome"] != "COLLISION"
    assert r["min_inter_drone_distance_m"] > 2.0 * card.card["criteria"]["body_radius_m"]
    assert form["local_commits"] == {f"uav{i}": 1 for i in range(6)}
    assert form["local_aborts"] == {f"uav{i}": 0 for i in range(6)}
    assert form["deadline_aborts"] == {f"uav{i}": 0 for i in range(6)}
    assert sum(form["fill_requests"].values()) > 0
    assert all(form["committed_t_switch_s"][f"uav{i}"] <= form["t_go_s"] + 1e-9 for i in range(6))
    for row in out.ticks:
        assert row["approved_owner"] == row["uav"]


def test_total_loss_leaves_the_wedge_hovering():
    # 票 06b 之前收不齐 ACK 就不再追赶。这里只确认现状，不改 formation.py。
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
