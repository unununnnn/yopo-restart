"""票 06：三机开阔区直线 + 转弯，虚拟中心 + 槽位，本机批准。判据已冻结；本文件仍核对行为，不改门槛。"""

import copy
import json
import subprocess

import numpy as np

from yopo_l0.bus import BusFaults
from yopo_l0.render import render_video
from yopo_l0.runner import WALL_FIELDS, RunConfig, run, write
from yopo_l0.scene import load_card

from conftest import ROOT

CARD = ROOT / "scenes/formation-hold.json"
GEOM = json.loads((ROOT / "scenes/open-area.json").read_text())


def strip_wall(result):
    return {k: v for k, v in result.items() if k not in WALL_FIELDS}


def _positions(out, drone_id):
    return [row["p"] for row in out.ticks if row["uav"] == drone_id]


def _run(card, **kw):
    return run(card, RunConfig(**kw))


def test_formation_holds_through_straight_and_turn(tmp_path):
    card = load_card(CARD)
    assert card.card["criteria"]["frozen"] is True
    out = _run(card, seed=0, record_planner_inputs=True)
    again = _run(card, seed=0, record_planner_inputs=True)
    r = out.result
    assert r["outcome"] == "SUCCESS", r
    assert r["reason"] == "GOAL_REACHED_AND_HELD"
    assert r["task"] == "formation" and r["policy"] == "formation"
    assert r["network"] is None and r["tier"] == "L0" and r["shadow"] is True
    assert isinstance(r["shadow_reject_rate"], float)
    assert [row["id"] for row in r["shadow_drones"]] == ["uav0", "uav1", "uav2"]
    assert all(isinstance(row["shadow_reject_rate"], float) for row in r["shadow_drones"])
    assert r["n_drones"] == 3 and r["arrived"] == 3
    form = r["formation"]
    assert form["frozen"] is True and form["adjudicator"] is None
    assert form["virtual_center"] == "scheduled"
    assert form["progress_gate"] is False
    assert form["arrival_rate"] == 1.0
    assert r["arrival_rate"] == form["arrival_rate"]
    assert r["formation_error_p95_m"] == form["error_p95_m"]
    assert form["proposals"] == 3 and form["acks"] == 6 and form["nacks"] == 0
    assert form["local_commits"] == {"uav0": 1, "uav1": 1, "uav2": 1}
    # 回归守卫，不是冻结判据：实测 p95 约 0.03 m、机间约 2 m。散掉或贴上才算坏。
    assert form["error_p95_m"] < 0.1
    assert r["min_inter_drone_distance_m"] > 1.5
    assert r["min_obstacle_distance_m"] > 0.35
    assert r["final_goal_error_m"] <= 0.4
    errs = [row["formation_error_m"] for row in out.ticks]
    assert abs(float(np.percentile(errs, 95)) - form["error_p95_m"]) < 1e-9
    slots = form["final_slots_enu"]
    assert abs(slots["uav1"][0] - slots["uav2"][0]) > 3.0
    assert abs(slots["uav1"][1] - slots["uav2"][1]) < 0.05
    last = {}
    straight = []
    turned = False
    for row in out.ticks:
        last[row["uav"]] = row["p"]
        assert row["approved_owner"] == row["uav"]
        if row["t"] >= form["t_go_s"] + 0.2:
            assert row["approved_seq"] >= 1
            assert row["plan_code"] == "LOCAL_APPROVED"
        if row["uav"] == "uav0" and 1.5 < row["p"][0] < 4.0:
            straight.append(row["p"])
        if row["uav"] == "uav0" and row["p"][1] > 4.0:
            turned = True
    assert straight and max(abs(p[1]) for p in straight) < 0.3
    assert turned
    for did, goal in slots.items():
        assert np.linalg.norm(np.asarray(last[did]) - np.asarray(goal)) <= 0.4
    assert r["bus"]["dropped"] == 0 and r["bus"]["delivered"] == r["bus"]["published"] > 0
    assert out.planner_inputs and all(len(item) == 64 for item in out.planner_inputs)
    assert strip_wall(r) == strip_wall(again.result)
    assert out.ticks == again.ticks and out.planner_inputs == again.planner_inputs
    json.dumps(r)
    log = write(out, tmp_path / "run") / "ticks.jsonl"
    info = render_video(log, tmp_path / "formation-hold.mp4", scene=CARD, fps=10, max_seconds=60)
    assert info.duration_s <= 60
    probe = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "json", str(info.path)],
        capture_output=True, text=True, check=True,
    )
    assert float(json.loads(probe.stdout)["format"]["duration"]) <= 60.5


def _actions(ticks):
    return [(t["cmd_p"], t["cmd_v"], t["cmd_a"], t["p"], t["v"], t["a"]) for t in ticks]


def test_shadow_records_per_drone_and_does_not_change_the_formation():
    card = load_card(CARD)
    kw = dict(seed=0, duration_s=3.0, record_planner_inputs=True)
    off = _run(card, shadow=False, **kw)
    on = _run(card, shadow=True, **kw)
    assert off.result["shadow_reject_rate"] is None and off.result["shadow_drones"] is None
    assert isinstance(on.result["shadow_reject_rate"], float)
    assert [row["id"] for row in on.result["shadow_drones"]] == ["uav0", "uav1", "uav2"]
    assert _actions(off.ticks) == _actions(on.ticks)
    assert off.planner_inputs == on.planner_inputs
    assert off.result["formation"]["error_p95_m"] == on.result["formation"]["error_p95_m"]
    assert off.result["formation"]["local_commits"] == on.result["formation"]["local_commits"]
    rejected = 0
    for row in on.result["shadow_drones"]:
        subset = [t for t in on.ticks if t["uav"] == row["id"]]
        n = sum(bool(t["would_reject"]) for t in subset)
        rejected += n
        assert row["shadow_reject_rate"] == n / len(subset)
    assert on.result["shadow_reject_rate"] == rejected / len(on.ticks)


def test_seed_does_not_change_the_approved_schedule():
    # 计划在 t_go 前就提交，覆盖整段时刻表。前 3 s 的轨迹和计划哈希一致，说明 seed 没进编队。
    card = load_card(CARD)
    a = _run(card, seed=0, duration_s=3.0)
    b = _run(card, seed=2, duration_s=3.0)
    assert a.result["formation"]["committed_plan_hash"] == b.result["formation"]["committed_plan_hash"]
    assert _positions(a, "uav0") == _positions(b, "uav0")
    assert _positions(a, "uav1") == _positions(b, "uav1")
    assert a.result["formation"]["local_commits"] == {"uav0": 1, "uav1": 1, "uav2": 1}


def test_loss_blocks_local_approval(tmp_path):
    card = load_card(CARD)
    cfg = dict(seed=0, duration_s=2.0, bus=BusFaults(drop=1.0))
    lost = _run(card, **cfg)
    again = _run(card, **cfg)
    r = lost.result
    assert r["outcome"] == "TIMEOUT"
    assert r["arrived"] == 0 and r["formation"]["arrival_rate"] == 0.0
    assert r["formation"]["local_commits"] == {"uav0": 0, "uav1": 0, "uav2": 0}
    assert r["formation"]["adjudicator"] is None
    assert r["bus"]["dropped"] == r["bus"]["published"] > 0
    assert r["bus"]["delivered"] == 0
    for did, start in ("uav0", [0.0, 0.0, 1.5]), ("uav1", [0.0, 2.0, 1.5]), ("uav2", [0.0, -2.0, 1.5]):
        last = _positions(lost, did)[-1]
        assert np.linalg.norm(np.asarray(last) - np.asarray(start)) < 0.05
    assert strip_wall(r) == strip_wall(again.result)
    assert lost.ticks == again.ticks


def test_reorder_still_arrives_on_local_commits():
    card = load_card(CARD)
    out = _run(card, seed=0, bus=BusFaults(reorder=1.0))
    r = out.result
    assert r["outcome"] == "SUCCESS", r
    assert r["arrived"] == 3 and r["formation"]["arrival_rate"] == 1.0
    assert r["formation"]["local_commits"] == {"uav0": 1, "uav1": 1, "uav2": 1}
    assert r["bus"]["reordered"] > 0
    assert r["min_inter_drone_distance_m"] > 1.5
    assert r["formation"]["error_p95_m"] < 0.1


def test_geometry_does_not_enter_the_protocol_planner(tmp_path):
    raw = json.loads(CARD.read_text())
    path = tmp_path / "formation-hold.json"
    path.write_text(json.dumps(raw))
    blocked = copy.deepcopy(GEOM)
    blocked["boxes"] = [{"name": "decoy", "lo": [30.0, 30.0, 0.0], "hi": [31.0, 31.0, 2.0]}]
    kw = dict(seed=0, duration_s=2.5, record_planner_inputs=True)
    plain = _run(load_card(path, geometry_override=GEOM), **kw)
    moved = _run(load_card(path, geometry_override=blocked), **kw)
    assert plain.planner_inputs == moved.planner_inputs
    assert plain.result["formation"]["committed_plan_hash"] == moved.result["formation"]["committed_plan_hash"]
    assert _positions(plain, "uav0") == _positions(moved, "uav0")
    assert plain.result["scene_card_hash"] != moved.result["scene_card_hash"]


def test_one_slot_change_does_not_drag_the_others(tmp_path):
    raw = json.loads(CARD.read_text())
    shifted = json.loads(CARD.read_text())
    shifted["drones"][1]["slot_body"] = [0.0, 2.5, 0.0]
    shifted["drones"][1]["start_enu"] = [0.0, 2.5, 1.5]
    path_a = tmp_path / "a.json"
    path_b = tmp_path / "b.json"
    path_a.write_text(json.dumps(raw))
    path_b.write_text(json.dumps(shifted))
    kw = dict(seed=0, duration_s=3.0)
    base = _run(load_card(path_a, geometry_override=GEOM), **kw)
    wing = _run(load_card(path_b, geometry_override=GEOM), **kw)
    assert _positions(base, "uav0") == _positions(wing, "uav0")
    assert _positions(base, "uav2") == _positions(wing, "uav2")
    assert _positions(base, "uav1") != _positions(wing, "uav1")


def test_overlapping_slots_are_not_approved(tmp_path):
    raw = json.loads(CARD.read_text())
    raw["drones"] = [
        {"id": "uav0", "slot_body": [0.0, 0.0, 0.0], "start_enu": [0.0, 0.0, 1.5]},
        {"id": "uav1", "slot_body": [0.0, 0.8, 0.0], "start_enu": [0.0, 0.8, 1.5]},
        {"id": "uav2", "slot_body": [0.0, -0.8, 0.0], "start_enu": [0.0, -0.8, 1.5]},
    ]
    path = tmp_path / "tight.json"
    path.write_text(json.dumps(raw))
    out = _run(load_card(path, geometry_override=GEOM), seed=0, duration_s=2.0)
    r = out.result
    assert r["outcome"] != "COLLISION"
    assert r["min_inter_drone_distance_m"] > 0.7
    assert r["formation"]["local_commits"] == {"uav0": 0, "uav1": 0, "uav2": 0}
    assert r["formation"]["adjudicator"] is None
    for did, start_y in ("uav0", 0.0), ("uav1", 0.8), ("uav2", -0.8):
        last = _positions(out, did)[-1]
        assert abs(last[0]) < 0.05 and abs(last[1] - start_y) < 0.05
