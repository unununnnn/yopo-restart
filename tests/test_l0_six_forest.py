"""票 11b：六机楔形穿 07b 的林。规划参数与三机穿林卡相同，只报数。"""

import json

import numpy as np

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
    out = run(card, RunConfig(seed=0, policy="crossing", duration_s=0.4, shadow=False))
    r = out.result
    assert r["n_drones"] == 6 and r["task"] == "crossing" and r["policy"] == "crossing"
    assert r["forest_density"] == "medium"
    assert r["outcome"] != "COLLISION"
    assert r["crossing"]["avoidance"] == "upstream-yopo"
    assert r["crossing"]["adjudicator"] is None
    assert r["min_inter_drone_distance_m"] > 2 * 0.35
