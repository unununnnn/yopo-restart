"""票 07：三机穿柱林。避障走上游 YOPO，槽位只是目标偏置，只飞本机已提交的计划。

碰撞数是硬事实，判据未冻结，测试不拿 10 个 seed 的碰撞数当门。
"""

import copy
import json

import numpy as np
import pytest

from yopo_l0.bus import BusFaults
from yopo_l0.crossing import format_report
from yopo_l0.render import render_top_pngs, render_video
from yopo_l0.runner import WALL_FIELDS, RunConfig, run, write
from yopo_l0.scene import load_card

from conftest import ROOT

CARD = ROOT / "scenes/crossing-pillars.json"
GEOM = json.loads((ROOT / "scenes/crossing-pillars-scene.json").read_text())


def strip_wall(result):
    return {k: v for k, v in result.items() if k not in WALL_FIELDS}


def _positions(out, drone_id):
    return [row["p"] for row in out.ticks if row["uav"] == drone_id]


def _run(card, **kw):
    kw.setdefault("shadow", False)
    return run(card, RunConfig(**kw))


def _open_geometry():
    geom = copy.deepcopy(GEOM)
    geom["boxes"] = [b for b in geom["boxes"] if not str(b["name"]).startswith("p_")]
    return geom


def test_scene_is_a_wide_g2_style_pillar_forest():
    g2 = json.loads((ROOT / "scenes/g2-scene.json").read_text())
    g2_pillar = next(b for b in g2["boxes"] if b["name"] == "pillar")
    g2_size = [hi - lo for lo, hi in zip(g2_pillar["lo"], g2_pillar["hi"])]
    g2_ceil = next(b for b in g2["boxes"] if b["name"] == "ceiling")["lo"][2]
    g2_left = next(b for b in g2["boxes"] if b["name"] == "left_wall")
    g2_right = next(b for b in g2["boxes"] if b["name"] == "right_wall")
    g2_gap = g2_left["lo"][1] - g2_right["hi"][1]
    raw = json.loads(CARD.read_text())
    assert raw["task"]["type"] == "crossing"
    assert raw["criteria"]["frozen"] is True
    assert raw["seeds"] == list(range(10))
    assert [d["id"] for d in raw["drones"]] == ["uav0", "uav1", "uav2"]
    pillars = [b for b in GEOM["boxes"] if str(b["name"]).startswith("p_")]
    assert len(pillars) == 12
    for pillar in pillars:
        size = [hi - lo for lo, hi in zip(pillar["lo"], pillar["hi"])]
        assert size == pytest.approx(g2_size, abs=1e-9)
    assert next(b for b in GEOM["boxes"] if b["name"] == "ceiling")["lo"][2] == g2_ceil
    left = next(b for b in GEOM["boxes"] if b["name"] == "left_wall")
    right = next(b for b in GEOM["boxes"] if b["name"] == "right_wall")
    gap = left["lo"][1] - right["hi"][1]
    assert gap == 28.0
    assert gap > g2_gap
    card = load_card(CARD)
    radius = card.card["criteria"]["body_radius_m"]
    for drone in card.card["drones"]:
        start = np.asarray(drone["start_enu"], dtype=float)
        assert card.min_distance(start) > radius
    specs = []
    for ln in (ROOT / "scenes/suite.txt").read_text().splitlines():
        ln = ln.split("#", 1)[0].strip()
        if ln:
            specs.append(ln)
    assert "scenes/crossing-pillars.json@crossing" in specs


def test_report_table_lists_the_ticket_columns():
    row = {
        "task": "crossing",
        "seed": 3,
        "n_drones": 3,
        "arrived": 2,
        "arrival_rate": 2 / 3,
        "outcome": "COLLISION",
        "reason": "uav1:TRUTH_POSITION",
        "min_inter_drone_distance_m": 1.8,
        "min_obstacle_distance_m": 0.2,
        "formation_error_p95_m": 0.8,
        "shadow_reject_rate": 0.4,
        "final_goal_error_m": 4.0,
        "crossing": {
            "frozen": False,
            "body_radius_m": 0.35,
            "obstacle_collisions": 1,
            "inter_drone_collisions": 0,
            "min_obstacle_clearance_m": -0.15,
            "decisions": {"OK": 4},
        },
    }
    text = format_report([row])
    assert "未冻结 / report-only" in text
    for header in ("到达率", "障碍碰撞", "机间碰撞", "最小机间距离 m", "最小障碍净空 m", "队形误差 p95 m", "影子拒绝率", "safe_radius m", "偏离上游", "无解次数"):
        assert header in text
    assert "0.67 (2/3)" in text
    assert "| 3 | 0.67 (2/3) | 1 | 0 | 1.800 | -0.150 | 0.800 | 0.400 | — | — | — | COLLISION uav1:TRUTH_POSITION |" in text
    assert "seed 3: COLLISION uav1:TRUTH_POSITION" in text


def test_same_seed_is_deterministic_and_each_drone_flies_its_own_plan():
    card = load_card(CARD)
    cfg = dict(seed=0, duration_s=1.2, record_planner_inputs=True)
    out = _run(card, **cfg)
    again = _run(card, **cfg)
    r = out.result
    assert r["task"] == "crossing" and r["policy"] == "crossing"
    assert r["crossing"]["frozen"] is True and r["crossing"]["adjudicator"] is None
    assert r["crossing"]["avoidance"] == "upstream-yopo"
    assert r["crossing"]["goal_bias"] == "slot-lookahead-blend"
    assert r["crossing"]["progress_gate"] is False
    assert r["crossing"]["virtual_center"] == "straight-rail"
    assert r["network"]["upstream_sha"].startswith("d5e1e60")
    assert r["arrival_rate"] == r["crossing"]["arrival_rate"]
    assert r["formation_error_p95_m"] == r["crossing"]["error_p95_m"]
    assert r["plan_codes"] == r["crossing"]["decisions"]
    assert r["crossing"]["decisions"].get("OK", 0) > 0
    assert sum(r["crossing"]["local_commits"].values()) == r["crossing"]["proposals"] > 0
    assert r["crossing"]["nacks"] == 0
    for row in out.ticks:
        assert row["approved_owner"] == row["uav"]
    assert out.planner_inputs and all(len(item) == 64 for item in out.planner_inputs)
    assert strip_wall(r) == strip_wall(again.result)
    assert out.ticks == again.ticks and out.planner_inputs == again.planner_inputs


def test_dropped_acks_leave_the_committed_hover(tmp_path):
    card = load_card(CARD)
    out = _run(card, seed=0, duration_s=1.2, bus=BusFaults(drop=1.0))
    r = out.result
    assert r["outcome"] != "COLLISION"
    assert r["crossing"]["local_commits"] == {"uav0": 0, "uav1": 0, "uav2": 0}
    assert r["crossing"]["adjudicator"] is None
    assert r["bus"]["dropped"] == r["bus"]["published"] > 0
    for did, start in ("uav0", [0.0, 0.0, 1.5]), ("uav1", [0.0, 8.0, 1.5]), ("uav2", [0.0, -8.0, 1.5]):
        last = _positions(out, did)[-1]
        assert np.linalg.norm(np.asarray(last) - np.asarray(start)) < 0.05
        assert all(row["approved_owner"] == did and row["approved_seq"] == 0 for row in out.ticks if row["uav"] == did)


def test_reorder_stays_deterministic_and_local():
    card = load_card(CARD)
    cfg = dict(seed=1, duration_s=1.2, bus=BusFaults(reorder=1.0), record_planner_inputs=True)
    a = _run(card, **cfg)
    b = _run(card, **cfg)
    assert a.result["crossing"]["adjudicator"] is None
    for row in a.ticks:
        assert row["approved_owner"] == row["uav"]
    assert strip_wall(a.result) == strip_wall(b.result)
    assert a.ticks == b.ticks


def test_hidden_geometry_does_not_enter_the_planner(tmp_path):
    raw = json.loads(CARD.read_text())
    path = tmp_path / "crossing-pillars.json"
    path.write_text(json.dumps(raw))
    hidden = copy.deepcopy(GEOM)
    hidden["boxes"] = GEOM["boxes"] + [{"name": "decoy", "lo": [-2.5, -1.0, 0.0], "hi": [-2.0, 1.0, 2.0]}]
    kw = dict(seed=0, duration_s=1.2, record_planner_inputs=True)
    plain = _run(load_card(path, geometry_override=GEOM), **kw)
    moved = _run(load_card(path, geometry_override=hidden), **kw)
    assert plain.planner_inputs == moved.planner_inputs
    assert plain.result["crossing"]["committed_plan_hash"] == moved.result["crossing"]["committed_plan_hash"]
    assert _positions(plain, "uav0") == _positions(moved, "uav0")
    assert plain.result["scene_card_hash"] != moved.result["scene_card_hash"]


def test_pillars_in_view_change_the_yopo_path():
    card = load_card(CARD)
    kw = dict(seed=0, duration_s=4.0, record_planner_inputs=True)
    forest = _run(card, **kw)
    opened = _run(load_card(CARD, geometry_override=_open_geometry()), **kw)
    assert forest.planner_inputs != opened.planner_inputs
    assert _positions(forest, "uav0") != _positions(opened, "uav0")
    assert opened.result["crossing"]["decisions"].get("OK", 0) > 0
    assert _positions(opened, "uav0")[-1][0] > 1.0
    for row in forest.ticks:
        assert row["approved_owner"] == row["uav"]


def test_shadow_records_and_does_not_change_the_crossing():
    card = load_card(CARD)
    kw = dict(seed=0, duration_s=1.2, record_planner_inputs=True)
    off = _run(card, shadow=False, **kw)
    on = _run(card, shadow=True, **kw)
    assert on.result["shadow"] is True and isinstance(on.result["shadow_reject_rate"], float)
    assert [row["id"] for row in on.result["shadow_drones"]] == ["uav0", "uav1", "uav2"]
    assert off.result["crossing"]["committed_plan_hash"] == on.result["crossing"]["committed_plan_hash"]
    assert [(t["p"], t["v"], t["cmd_p"], t["cmd_v"]) for t in off.ticks] == [
        (t["p"], t["v"], t["cmd_p"], t["cmd_v"]) for t in on.ticks
    ]


def test_short_log_renders_realtime_video_and_top_pngs(tmp_path):
    card = load_card(CARD)
    out = _run(card, seed=0, duration_s=0.4)
    log = write(out, tmp_path / "run") / "ticks.jsonl"
    info = render_video(log, tmp_path / "crossing.mp4", scene=CARD, fps=10, max_seconds=60)
    assert info.duration_s <= 60 and info.path.is_file()
    pngs = render_top_pngs(log, tmp_path / "tops", scene=CARD, n=2)
    assert len(pngs) == 2
    assert all(p.suffix == ".png" and p.stat().st_size > 500 for p in pngs)
