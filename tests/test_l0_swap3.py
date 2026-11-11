"""票 E-C2：三机开阔对向卡只报数，不进套件，不设冻结门。"""

import json
import math

import numpy as np
import pytest

from yopo_l0.criteria import active_gates
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import Ground, load_card

from conftest import ROOT

CARD = ROOT / "scenes/swap-3-open.json"
RADIUS_M = 8.0
HEIGHT_M = 1.5


def test_swap3_card_is_report_only_antipodal_circle():
    raw = json.loads(CARD.read_text())
    assert raw["name"] == "swap-3-open"
    assert raw["geometry"] == "open-area.json"
    assert raw["task"]["type"] == "goto"
    assert raw["seeds"] == [0]
    assert raw["criteria"]["frozen"] is False
    assert "gates" not in raw["criteria"]
    assert "20.3" in raw["note"] and "14.3" in raw["note"]
    assert "不复现" in raw["note"]
    assert raw["dynamics"] == {
        "model": "point_mass_lag",
        "tau_s": 0.1,
        "kp": 4.0,
        "kd": 3.0,
        "a_max": 6.0,
    }
    assert "speed_mps" not in raw["task"]
    starts = []
    for drone in raw["drones"]:
        start = np.asarray(drone["start_enu"], dtype=float)
        goal = np.asarray(drone["goal_enu"], dtype=float)
        assert start[2] == HEIGHT_M and goal[2] == HEIGHT_M
        assert math.hypot(start[0], start[1]) == pytest.approx(RADIUS_M)
        assert math.hypot(goal[0], goal[1]) == pytest.approx(RADIUS_M)
        assert np.allclose(start[:2] + goal[:2], 0.0)
        starts.append(start[:2])
    assert len(raw["drones"]) == 3
    side = RADIUS_M * math.sqrt(3)
    for i in range(3):
        d = float(np.linalg.norm(starts[i] - starts[(i + 1) % 3]))
        assert d == pytest.approx(side)

    card = load_card(CARD)
    assert all(isinstance(o, Ground) for o in card.obstacles)
    assert active_gates(card, policy="upstream") is None
    listed = (ROOT / "scenes/suite.txt").read_text()
    assert "swap-3-open" not in listed


def test_swap3_card_hash_stable_and_short_run_records_gap():
    card = load_card(CARD)
    again = load_card(CARD)
    assert card.hash == again.hash and len(card.hash) == 64
    out = run(card, RunConfig(seed=0, policy="upstream", shadow=False, duration_s=0.2))
    assert out.result["scene"] == "swap-3-open"
    assert out.result["scene_card_hash"] == card.hash
    assert out.result["n_drones"] == 3
    assert out.result["tier"] == "L0"
    assert out.result["policy"] == "upstream"
    # 起点间距 8√3 m，短跑还没飞到圆心。
    assert out.result["min_inter_drone_distance_m"] > 2 * 0.35
    assert out.result["outcome"] != "COLLISION"
