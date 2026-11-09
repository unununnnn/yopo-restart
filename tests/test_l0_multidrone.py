"""票 05：三机开阔区各自飞向各自目标。只看结果记录和规划器输入摘要。

证明用 hover（跟踪 goal_enu）。upstream / certified 的计划对象同样只经令牌投递，
单机回归覆盖那两条策略。
"""

import json

from yopo_l0.bus import BusFaults
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import load_card

from conftest import G2_CARD, ROOT

OPEN = ROOT / "scenes/open-3drone.json"
GEOM = json.loads((ROOT / "scenes/open-area.json").read_text())


def strip_wall(r):
    return {k: v for k, v in r.items() if k not in WALL_FIELDS}


def _last_positions(out):
    last = {}
    for tick in out.ticks:
        last[tick["uav"]] = tick["p"]
    return last


def _hover(seed=0, **kw):
    return RunConfig(seed=seed, policy="hover", **kw)


def test_three_drones_reach_own_goals_without_collision():
    out = run(load_card(OPEN), _hover(0))
    r = out.result
    assert r["outcome"] == "SUCCESS"
    assert r["reason"] == "GOAL_REACHED_AND_HELD"
    assert r["n_drones"] == 3 and r["arrived"] == 3
    assert r["policy"] == "hover"
    assert r["min_inter_drone_distance_m"] == 5.0
    assert r["min_inter_drone_distance_m"] > 2 * 0.35
    assert r["min_obstacle_distance_m"] > 0.35
    assert r["final_goal_error_m"] <= 0.4
    pos = _last_positions(out)
    assert abs(pos["uav0"][0] - 6.0) <= 0.4 and abs(pos["uav0"][1]) < 0.05
    assert abs(pos["uav1"][0] - 6.0) <= 0.4 and abs(pos["uav1"][1] - 5.0) < 0.05
    assert abs(pos["uav2"][0] - 6.0) <= 0.4 and abs(pos["uav2"][1] + 5.0) < 0.05
    # 每条计划都经总线：发给全部无人机，其中 n-1 份是他机公告。
    assert r["plans"] > 0
    assert r["bus"]["published"] == r["plans"] * r["n_drones"]
    assert r["bus"]["delivered"] == r["bus"]["published"]
    assert r["bus"]["peer_deliveries"] == r["plans"] * (r["n_drones"] - 1)
    assert r["bus"]["dropped"] == 0 and r["bus"]["reordered"] == 0


def test_three_drone_same_seed_is_deterministic():
    card = load_card(OPEN)
    cfg = _hover(1, record_planner_inputs=True)
    a = run(card, cfg)
    b = run(card, cfg)
    assert strip_wall(a.result) == strip_wall(b.result)
    assert a.ticks == b.ticks
    assert a.planner_inputs == b.planner_inputs
    assert "wall_time_s" not in strip_wall(a.result)


def test_loss_prevents_goal_and_is_deterministic():
    card = load_card(OPEN)
    cfg = _hover(0, duration_s=0.5, bus=BusFaults(drop=1.0))
    lost = run(card, cfg)
    again = run(card, cfg)
    r = lost.result
    assert r["outcome"] == "TIMEOUT"
    assert r["reason"] == "GOAL_NOT_REACHED"
    assert r["arrived"] == 0
    assert r["bus"]["dropped"] == r["bus"]["published"] > 0
    assert r["bus"]["delivered"] == 0
    assert r["bus"]["peer_deliveries"] == 0
    pos = _last_positions(lost)
    assert abs(pos["uav0"][0]) < 0.05
    assert abs(pos["uav1"][1] - 5.0) < 0.05
    assert r["final_goal_error_m"] > 1.0
    assert strip_wall(r) == strip_wall(again.result)
    assert lost.ticks == again.ticks


def test_reorder_still_reaches_goals_and_records_reordering():
    card = load_card(OPEN)
    cfg = _hover(0, bus=BusFaults(reorder=1.0), record_planner_inputs=True)
    a = run(card, cfg)
    b = run(card, cfg)
    r = a.result
    assert r["outcome"] == "SUCCESS"
    assert r["arrived"] == 3
    assert r["bus"]["reordered"] > 0
    assert r["min_inter_drone_distance_m"] > 2 * 0.35
    pos = _last_positions(a)
    assert abs(pos["uav0"][1]) < 0.05
    assert abs(pos["uav1"][1] - 5.0) < 0.05
    assert abs(pos["uav2"][1] + 5.0) < 0.05
    assert strip_wall(r) == strip_wall(b.result)
    assert a.ticks == b.ticks
    assert a.planner_inputs == b.planner_inputs


def test_other_drone_does_not_enter_planner_inputs(tmp_path):
    # 只挪 uav1 的起点和目标。uav0 的规划输入摘要必须逐位不变；uav1 自己的必须变。
    base = json.loads(OPEN.read_text())
    moved = json.loads(OPEN.read_text())
    moved["drones"][1]["start_enu"] = [0.0, 6.0, 1.5]
    moved["drones"][1]["goal_enu"] = [6.0, 6.0, 1.5]
    path_a = tmp_path / "a.json"
    path_b = tmp_path / "b.json"
    path_a.write_text(json.dumps(base))
    path_b.write_text(json.dumps(moved))
    cfg = _hover(0, duration_s=0.5, record_planner_inputs=True)
    a = run(load_card(path_a, geometry_override=GEOM), cfg)
    b = run(load_card(path_b, geometry_override=GEOM), cfg)
    n = 3
    own0_a, own0_b = a.planner_inputs[0::n], b.planner_inputs[0::n]
    own1_a, own1_b = a.planner_inputs[1::n], b.planner_inputs[1::n]
    assert own0_a and own0_a == own0_b
    assert own1_a != own1_b


def test_inter_drone_overlap_is_collision(make_card):
    hit = make_card(
        drones=[
            {"id": "uav0", "start_enu": [0.0, 0.0, 1.5]},
            {"id": "uav1", "start_enu": [0.5, 0.0, 1.5]},
        ],
        task={"type": "hover", "duration_s": 0.2},
    )
    out = run(hit, _hover(0))
    assert out.result["outcome"] == "COLLISION"
    assert out.result["reason"] == "uav0-uav1:INTER_DRONE"
    assert out.result["min_inter_drone_distance_m"] <= 0.7

    clear = make_card(
        drones=[
            {"id": "uav0", "start_enu": [0.0, 0.0, 1.5]},
            {"id": "uav1", "start_enu": [2.0, 0.0, 1.5]},
        ],
        task={"type": "hover", "duration_s": 0.2},
    )
    ok = run(clear, _hover(0))
    assert ok.result["outcome"] == "SUCCESS"
    assert ok.result["min_inter_drone_distance_m"] > 0.7


def test_single_drone_hover_has_no_inter_distance():
    out = run(load_card(G2_CARD), _hover(0, shadow=False))
    r = out.result
    assert r["outcome"] == "SUCCESS"
    assert r["n_drones"] == 1
    assert r["min_inter_drone_distance_m"] is None
    assert r["bus"]["dropped"] == 0
    assert r["bus"]["reordered"] == 0
    assert r["bus"]["peer_deliveries"] == 0
    assert r["bus"]["delivered"] == r["bus"]["published"] == r["plans"]
