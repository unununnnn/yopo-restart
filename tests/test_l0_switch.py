"""票 08：横排 ↔ 纵列。窄门宽度触发，槽位重分配走预留协议，不瞬移。

判据未冻结。过渡期碰撞是硬失败，测试锁定「成功那次是 0」和「失败时有拒绝码」。
"""

import json
import tempfile
from pathlib import Path

import numpy as np

from yopo_l0.bus import MessageBus
from yopo_l0.render import render_top_pngs, render_video
from yopo_l0.runner import WALL_FIELDS, RunConfig, run, write
from yopo_l0.scene import load_card
from yopo_l0.switch import SwitchFleet, corridor_width, format_report
from yopo_swarm_core.protocol.messages import Vote

from conftest import ROOT

CARD = ROOT / "scenes/formation-switch.json"
GEOM = json.loads((ROOT / "scenes/formation-switch-scene.json").read_text())


def strip_wall(result):
    return {k: v for k, v in result.items() if k not in WALL_FIELDS}


def _positions(out, drone_id):
    return [row["p"] for row in out.ticks if row["uav"] == drone_id]


def _at(out, drone_id, t, tol=0.02):
    rows = [row for row in out.ticks if row["uav"] == drone_id and abs(row["t"] - t) <= tol]
    assert rows, (drone_id, t)
    return rows[len(rows) // 2]


def _run(card, **kw):
    kw.setdefault("shadow", False)
    return run(card, RunConfig(**kw))


def test_gate_is_narrow_and_the_card_is_in_the_suite():
    raw = json.loads(CARD.read_text())
    assert raw["task"]["type"] == "switch"
    assert raw["criteria"]["frozen"] is False
    assert raw["seeds"] == list(range(10))
    assert [d["id"] for d in raw["drones"]] == ["uav0", "uav1", "uav2"]
    threshold = raw["task"]["width_threshold_m"]
    open_w = raw["task"]["open_width_m"]
    assert corridor_width(GEOM, 2.0, 1.5, open_width=open_w) == open_w
    gate = corridor_width(GEOM, 12.6, 1.5, open_width=open_w)
    assert gate < threshold < open_w
    assert gate > 2.0 * raw["criteria"]["body_radius_m"]
    card = load_card(CARD)
    radius = card.card["criteria"]["body_radius_m"]
    # 横排翼机落在柱子里，纵列中线过得去。
    assert card.min_distance(np.array([12.6, 2.0, 1.5])) <= radius
    assert card.min_distance(np.array([12.6, -2.0, 1.5])) <= radius
    assert card.min_distance(np.array([12.6, 0.0, 1.5])) > radius
    for drone in card.card["drones"]:
        start = np.asarray(drone["start_enu"], dtype=float)
        assert card.min_distance(start) > radius
    specs = []
    for ln in (ROOT / "scenes/suite.txt").read_text().splitlines():
        ln = ln.split("#", 1)[0].strip()
        if ln:
            specs.append(ln)
    assert "scenes/formation-switch.json@switch" in specs


def test_report_table_lists_the_ticket_columns():
    row = {
        "task": "switch",
        "seed": 4,
        "n_drones": 3,
        "arrived": 2,
        "arrival_rate": 2 / 3,
        "outcome": "COLLISION",
        "reason": "uav2:TRUTH_POSITION",
        "shadow_reject_rate": 0.5,
        "final_goal_error_m": 6.0,
        "switch": {
            "frozen": False,
            "transition_collisions": 1,
            "min_inter_drone_transition_m": 1.2,
            "transition_duration_s": [4.0, 4.5],
            "error_p95_before_m": 0.03,
            "error_p95_after_m": None,
            "decisions": {"SWITCH:CONFLICT": 3, "SWITCH:TO_COLUMN": 1, "LINE:COMMIT": 3},
        },
    }
    text = format_report([row])
    assert "未冻结 / report-only" in text
    for header in (
        "到达率",
        "过渡碰撞",
        "过渡最小机间距离 m",
        "过渡时长 s",
        "变阵前队形误差 p95 m",
        "变阵后队形误差 p95 m",
        "影子拒绝率",
    ):
        assert header in text
    assert "0.67 (2/3)" in text
    assert "SWITCH:CONFLICT:3" in text
    assert "过渡碰撞" in text
    assert "| 4 | 0.67 (2/3) | 1 | 1.200 | 4.00+4.50 | 0.030 | — | 0.500 | COLLISION uav2:TRUTH_POSITION 过渡碰撞 SWITCH:CONFLICT:3 |" in text
    assert "seed 4: COLLISION uav2:TRUTH_POSITION" in text
    ok = {
        "task": "switch",
        "seed": 0,
        "n_drones": 3,
        "arrived": 3,
        "arrival_rate": 1.0,
        "outcome": "SUCCESS",
        "reason": "GOAL_REACHED_AND_HELD",
        "shadow_reject_rate": 0.0,
        "final_goal_error_m": 0.02,
        "switch": {
            "frozen": False,
            "transition_collisions": 0,
            "min_inter_drone_transition_m": 1.5,
            "transition_duration_s": [5.0, 4.5],
            "error_p95_before_m": 0.02,
            "error_p95_after_m": 0.03,
            "decisions": {"SWITCH:TO_COLUMN": 2, "SWITCH:TO_LINE": 2},
        },
    }
    ok_text = format_report([ok])
    assert "| 0 | 1.00 (3/3) | 0 | 1.500 | 5.00+4.50 | 0.020 | 0.030 | 0.000 |  |" in ok_text
    assert "未到达" not in ok_text


def test_switch_pair_is_negotiated_and_arrives(tmp_path):
    card = load_card(CARD)
    out = run(card, RunConfig(seed=0, shadow=True, record_planner_inputs=True))
    again = run(card, RunConfig(seed=0, shadow=True, record_planner_inputs=True))
    r = out.result
    assert r["outcome"] == "SUCCESS", r
    assert r["reason"] == "GOAL_REACHED_AND_HELD"
    assert r["task"] == "switch" and r["policy"] == "switch"
    assert r["arrived"] == 3 and r["arrival_rate"] == 1.0
    assert r["n_drones"] == 3 and r["tier"] == "L0"
    assert isinstance(r["shadow_reject_rate"], float)
    sw = r["switch"]
    assert sw["frozen"] is False and sw["adjudicator"] is None
    assert sw["pair"] == "line-abreast<->column"
    assert sw["trigger"] == "corridor-width-ahead"
    assert sw["negotiated"] == "bus-reservation"
    assert sw["transition_collisions"] == 0
    assert sw["min_inter_drone_transition_m"] > 1.2
    assert r["min_inter_drone_distance_m"] > 1.2
    assert r["min_obstacle_distance_m"] > 0.35
    assert sw["error_p95_before_m"] < 0.1
    assert sw["error_p95_after_m"] < 0.1
    assert sw["local_commits"] == {"uav0": 1, "uav1": 3, "uav2": 3}
    assert sw["proposals"] == 7 and sw["acks"] == 14 and sw["nacks"] == 0
    assert sw["decisions"]["SWITCH:TO_COLUMN"] == 2
    assert sw["decisions"]["SWITCH:TO_LINE"] == 2
    assert "SWITCH:CONFLICT" not in sw["decisions"]
    assert "SWITCH:NO_ACK" not in sw["decisions"]
    directions = [w["direction"] for w in sw["transitions"]]
    assert directions == ["line-to-column", "column-to-line"]
    to_col, to_line = sw["transitions"]
    assert to_col["collisions"] == 0 and to_line["collisions"] == 0
    assert to_line["t_start"] >= to_col["t_end"] - 1e-9
    assert all(w["duration_s"] > 0.0 for w in sw["transitions"])
    # 变阵前是横排，窄门里是纵列，出门后回到横排。
    before = _at(out, "uav1", sw["t_go_s"] + 2.0)
    assert abs(before["p"][1] - 2.0) < 0.2
    before2 = _at(out, "uav2", sw["t_go_s"] + 2.0)
    assert abs(before2["p"][1] + 2.0) < 0.2
    assert abs(before["p"][0] - before2["p"][0]) < 0.3
    hold_t = to_col["t_end"] + 0.5
    xs = {}
    for did in ("uav0", "uav1", "uav2"):
        row = _at(out, did, hold_t)
        assert abs(row["p"][1]) < 0.15
        xs[did] = row["p"][0]
    assert xs["uav0"] > xs["uav1"] + 1.8
    assert xs["uav1"] > xs["uav2"] + 1.8
    after = _at(out, "uav1", to_line["t_end"] + 0.5)
    after2 = _at(out, "uav2", to_line["t_end"] + 0.5)
    assert after["p"][1] > 1.5 and after2["p"][1] < -1.5
    assert abs(after["p"][0] - after2["p"][0]) < 0.4
    prev = {}
    max_step = 0.0
    for row in out.ticks:
        assert row["approved_owner"] == row["uav"]
        if row["t"] >= sw["t_go_s"] + 0.2:
            assert row["approved_seq"] >= 1
        p = row["p"]
        if row["uav"] in prev:
            max_step = max(max_step, float(np.linalg.norm(np.asarray(p) - np.asarray(prev[row["uav"]]))))
        prev[row["uav"]] = p
    assert max_step < 0.06  # 100 Hz 下巡航一步约 1.5 cm；瞬移会是米级
    last_seq = {}
    for row in out.ticks:
        last_seq[row["uav"]] = row["approved_seq"]
    assert last_seq == {"uav0": 1, "uav1": 3, "uav2": 3}
    assert r["bus"]["dropped"] == 0 and r["bus"]["delivered"] == r["bus"]["published"] > 0
    assert out.planner_inputs and all(len(item) == 64 for item in out.planner_inputs)
    assert strip_wall(r) == strip_wall(again.result)
    assert out.ticks == again.ticks and out.planner_inputs == again.planner_inputs
    json.dumps(r)
    log = write(out, tmp_path / "run") / "ticks.jsonl"
    info = render_video(log, tmp_path / "formation-switch.mp4", scene=CARD, fps=10, max_seconds=60)
    assert info.duration_s <= 60 and info.path.is_file() and info.path.stat().st_size > 0
    pngs = render_top_pngs(log, tmp_path / "tops", scene=CARD, n=5)
    assert len(pngs) == 5
    assert all(p.is_file() and p.stat().st_size > 0 for p in pngs)


def test_seed_does_not_move_the_trigger():
    card = load_card(CARD)
    a = _run(card, seed=0, duration_s=8.0)
    b = _run(card, seed=4, duration_s=8.0)
    assert a.result["switch"]["decisions"] == b.result["switch"]["decisions"]
    assert _positions(a, "uav0") == _positions(b, "uav0")
    assert _positions(a, "uav1") == _positions(b, "uav1")
    assert _positions(a, "uav2") == _positions(b, "uav2")
    assert a.result["switch"]["blends"][0]["t_start"] == b.result["switch"]["blends"][0]["t_start"]


def test_open_geometry_stays_in_line(tmp_path):
    geom = json.loads(json.dumps(GEOM))
    geom["boxes"] = []
    card = load_card(CARD, geometry_override=geom)
    out = _run(card, seed=0, duration_s=12.0)
    decisions = out.result["switch"]["decisions"]
    assert "SWITCH:TO_COLUMN" not in decisions
    assert "SWITCH:TO_LINE" not in decisions
    assert out.result["switch"]["transitions"] == []
    row = _at(out, "uav1", 10.0)
    assert abs(row["p"][1] - 2.0) < 0.2


def test_overlapping_column_slots_are_rejected_with_a_code():
    raw = json.loads(CARD.read_text())
    raw["task"]["column_slots"]["uav2"] = [-2.4, 0.0, 0.0]
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        (td / "formation-switch-scene.json").write_text(json.dumps(GEOM))
        card_path = td / "formation-switch.json"
        card_path.write_text(json.dumps(raw))
        out = _run(load_card(card_path), seed=0, duration_s=16.0)
    r = out.result
    assert r["outcome"] == "COLLISION"
    assert r["switch"]["decisions"].get("SWITCH:CONFLICT", 0) >= 1
    assert r["switch"]["decisions"].get("SWITCH:TO_COLUMN", 0) == 1
    text = format_report([r])
    assert "SWITCH:CONFLICT" in text
    assert "硬失败" in text


def test_missing_acks_record_switch_no_ack():
    """变阵提案已经上了总线，ACK 被丢掉，截止前没收齐就记 SWITCH:NO_ACK，不切槽位。"""
    card = load_card(CARD)
    fleet = SwitchFleet.from_card(card.card, 0.01)
    inner = MessageBus()

    class DropVotes:
        def __init__(self, bus, after_t):
            self.bus = bus
            self.after_t = after_t

        def publish(self, now, *, src, dest, topic, payload):
            msg = payload.get("msg")
            if now >= self.after_t and isinstance(msg, Vote):
                return
            self.bus.publish(now, src=src, dest=dest, topic=topic, payload=payload)

        def poll(self, now):
            return self.bus.poll(now)

    # 横排在 t≈1.5 之前提交完。之后丢掉表决，纵列提案收不到 ACK。
    bus = DropVotes(inner, 3.0)
    t = 0.0
    while t <= 8.0 + 1e-9:
        fleet.exchange(bus, t)
        t = round(t + 0.05, 10)
    summary = fleet.summary(0, 3)
    assert summary["decisions"].get("LINE:COMMIT") == 3
    assert summary["decisions"].get("SWITCH:NO_ACK", 0) >= 1
    assert "SWITCH:TO_COLUMN" not in summary["decisions"]
    assert summary["adjudicator"] is None


def test_shadow_records_and_does_not_change_the_switch():
    card = load_card(CARD)
    kw = dict(seed=0, duration_s=4.0, record_planner_inputs=True)
    off = _run(card, shadow=False, **kw)
    on = _run(card, shadow=True, **kw)

    def actions(ticks):
        return [(t["cmd_p"], t["cmd_v"], t["cmd_a"], t["p"], t["v"], t["a"]) for t in ticks]

    assert off.result["shadow_reject_rate"] is None
    assert isinstance(on.result["shadow_reject_rate"], float)
    assert actions(off.ticks) == actions(on.ticks)
    assert off.planner_inputs == on.planner_inputs
    assert off.result["switch"]["local_commits"] == on.result["switch"]["local_commits"]
