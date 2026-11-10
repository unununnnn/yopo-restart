"""票 11b：六机楔形穿 07b 的林。规划参数与三机穿林卡相同，只报数。"""

import json

import numpy as np

from yopo_l0.crossing import CrossingFleet
from yopo_l0.forest import card_for_tier
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card

from conftest import ROOT

MEDIUM = ROOT / "scenes/crossing-6-forest.json"
SPARSE = ROOT / "scenes/crossing-6-forest-sparse.json"
THREE = ROOT / "scenes/crossing-forest.json"

_TASK_KEYS = (
    "type", "duration_s", "window_len_s", "commit_horizon_s", "lookahead_s",
    "min_ahead_m", "lateral_gain", "arrive_radius_m", "t_go_s",
)
_RAIL_KEYS = ("length_m", "speed_mps")
_WEDGE = {
    "uav0": [0.0, 1.5, 0.0],
    "uav1": [0.0, -1.5, 0.0],
    "uav2": [-3.0, 3.5, 0.0],
    "uav3": [-3.0, -3.5, 0.0],
    "uav4": [-6.0, 5.5, 0.0],
    "uav5": [-6.0, -5.5, 0.0],
}


def test_six_wedge_uses_the_three_drone_crossing_parameters():
    six = json.loads(MEDIUM.read_text())
    three = json.loads(THREE.read_text())
    assert six["forest"] == {"density": "medium", "seed": 7}
    assert six["seeds"] == list(range(10))
    assert six["criteria"] == three["criteria"] and six["criteria"]["frozen"] is False
    assert six["camera"] == three["camera"]
    assert six["dynamics"] == three["dynamics"]
    assert six["rates_hz"] == three["rates_hz"]
    for key in _TASK_KEYS:
        assert six["task"][key] == three["task"][key]
    for key in _RAIL_KEYS:
        assert six["task"]["virtual_center"][key] == three["task"]["virtual_center"][key]
    # 后墙在 x=-3。楔形纵深 6 m，虚拟中心放到 x=6，最后一排落在 x=0，离后墙和三机卡一样。
    assert six["task"]["virtual_center"]["origin_enu"] == [6.0, 0.0, 1.5]
    assert [d["id"] for d in six["drones"]] == list(_WEDGE)
    card = load_card(MEDIUM)
    radius = six["criteria"]["body_radius_m"]
    for drone in six["drones"]:
        assert drone["slot_body"] == _WEDGE[drone["id"]]
        start = np.asarray(drone["start_enu"], dtype=float)
        slot = np.asarray(drone["slot_body"], dtype=float)
        assert np.allclose(start, np.array([6.0, 0.0, 1.5]) + slot)
        assert card.min_distance(start) > radius
    sparse = json.loads(SPARSE.read_text())
    assert sparse["forest"] == {"density": "sparse", "seed": 7}
    assert sparse["name"] == "crossing-6-forest-sparse"
    generated = card_for_tier(six, "sparse")
    assert generated["forest"]["density"] == "sparse"
    assert generated["drones"] == six["drones"]
    assert generated["task"] == six["task"]


def test_medium_seed0_short_prefix_stays_clear_of_the_wall():
    card = load_card(MEDIUM)
    cfg = RunConfig(seed=0, policy="crossing", duration_s=0.4, shadow=False)
    out = run(card, cfg)
    again = run(card, cfg)
    r = out.result
    assert r["n_drones"] == 6 and r["task"] == "crossing" and r["policy"] == "crossing"
    assert r["forest_density"] == "medium"
    assert r["outcome"] != "COLLISION"
    assert r["crossing"]["avoidance"] == "upstream-yopo"
    assert r["crossing"]["goal_bias"] == "forest-loose-goto"
    assert r["crossing"]["commit_horizon_s"] == 1.0
    assert r["crossing"]["commit_span_s"] == 1.5
    assert r["crossing"]["adjudicator"] is None
    assert r["min_inter_drone_distance_m"] > 2 * 0.35
    assert out.ticks == again.ticks
    again_result = {k: v for k, v in again.result.items() if k != "wall_time_s"}
    this_result = {k: v for k, v in r.items() if k != "wall_time_s"}
    assert this_result == again_result


def test_woods_goal_is_a_personal_goto_and_three_drones_stay_on_the_slot():
    """林带里目标改到前方净空够的侧向；林外和三机卡仍是原来的槽位前瞻。"""
    six = CrossingFleet.from_card(load_card(MEDIUM).card, 0.01)
    head = np.array([16.0, -4.2, 1.5])
    loose = six.goal_for("uav3", 8.0, head)
    assert abs(float(loose[0]) - 21.0) < 1e-9
    assert abs(float(loose[2]) - 1.5) < 1e-9
    chosen = six._line_clearance(16.0, 20.0, loose[1], 1.5)
    slot_clear = six._line_clearance(16.0, 20.0, float(six.slots["uav3"][1]), 1.5)
    window = [
        six._line_clearance(16.0, 20.0, float(y), 1.5)
        for y in np.linspace(-4.2 - 2.0, -4.2 + 2.0, 9)
    ]
    assert chosen + 1e-9 >= max(slot_clear, max(window)) or chosen >= 0.5 - 1e-6
    early = six.goal_for("uav3", 0.5, np.array([3.2, -4.2, 1.5]))
    assert early[1] > -4.2
    assert six._commit_span() == 1.5
    three = CrossingFleet.from_card(load_card(THREE).card, 0.01)
    kept = three.goal_for("uav0", 8.0, np.array([16.0, 1.0, 1.5]))
    assert kept[1] != 1.0
    assert three._commit_span() == three.horizon
    assert three.summary(0, 3)["goal_bias"] == "slot-lookahead-blend"
    assert three._loose_band(16.0) is False
