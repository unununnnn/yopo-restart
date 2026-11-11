"""票 E-C4 密度场 L0 seed 0：飞行卡能加载，圆柱数与 E-C4 一致，起终点净空够。"""

import hashlib
import importlib.util
import json
import math
import sys

import numpy as np
import pytest

from yopo_l0.criteria import active_gates
from yopo_l0.scene import Cylinder, Ground, load_card

from conftest import ROOT

DISCLAIMER = "这是假设，不是论文复现"
GEOM_SHA256 = {
    "ego-density-014.json": "5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9",
    "ego-density-028.json": "e43d320ad66ac47732ccb39c1a355a6152d7ff7b0139cae76f5b7222a1f28d60",
    "ego-density-042.json": "ee613b50ab19f9c4f2438f3298f79baab8f88f90ce7ebc36aacd0750ed7f8a97",
}
CARDS = (
    ("014", "swap3", "ego-density-014.json", 56),
    ("014", "solo", "ego-density-014.json", 56),
    ("028", "swap3", "ego-density-028.json", 112),
    ("028", "solo", "ego-density-028.json", 112),
    ("042", "swap3", "ego-density-042.json", 168),
    ("042", "solo", "ego-density-042.json", 168),
)


def _script():
    path = ROOT / "scripts/e-c4-density-l0.py"
    spec = importlib.util.spec_from_file_location("e_c4_density_l0", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _path(tag: str, kind: str):
    return ROOT / "scenes" / f"ego-density-{tag}-{kind}.json"


def test_density_geometry_files_unchanged():
    for name, digest in GEOM_SHA256.items():
        got = hashlib.sha256((ROOT / "scenes" / name).read_bytes()).hexdigest()
        assert got == digest


def test_flight_cards_load_with_e_c4_cylinder_counts_and_clearance():
    mod = _script()
    assert mod.DISCLAIMER == DISCLAIMER
    assert mod.AXIS_CLEARANCE_M == pytest.approx(0.70)
    assert mod.OBSTACLE_RADIUS_M == 0.20
    assert mod.MAP_SIDE_M == 20.0
    expected = mod.expected_cards()
    suite = (ROOT / "scenes/suite.txt").read_text()
    for tag, kind, geom, n in CARDS:
        path = _path(tag, kind)
        raw = json.loads(path.read_text(encoding="utf-8"))
        assert raw == expected[path.name]
        assert raw["name"] == f"ego-density-{tag}-{kind}"
        assert raw["geometry"] == geom
        assert raw["task"]["type"] == "goto"
        assert raw["seeds"] == [0]
        assert raw["criteria"]["frozen"] is False
        assert "gates" not in raw["criteria"]
        assert "policy_params" not in raw
        assert "geometric_fallback" not in raw
        assert raw["note"].startswith("票 E-C4")
        assert DISCLAIMER in raw["note"]
        assert "0.20 m" in raw["note"] and "20.0 m" in raw["note"]
        assert path.name not in suite
        assert "speed_mps" not in raw["task"]
        assert raw["dynamics"]["a_max"] == 6.0

        card = load_card(path)
        cylinders = [o for o in card.obstacles if isinstance(o, Cylinder)]
        assert len(cylinders) == n
        assert all(c.radius == 0.20 for c in cylinders)
        assert any(isinstance(o, Ground) for o in card.obstacles)
        assert card.card["geometry"]["header"]["disclaimer"] == DISCLAIMER
        assert card.card["geometry"]["header"]["n_obstacles"] == n
        assert active_gates(card, policy="upstream") is None
        assert active_gates(card, policy="hover") is None

        centers = [(float(c.center_xy[0]), float(c.center_xy[1])) for c in cylinders]
        for drone in raw["drones"]:
            for key in ("start_enu", "goal_enu"):
                x, y, z = drone[key]
                assert z == 1.5
                dist = min(math.hypot(x - cx, y - cy) for cx, cy in centers)
                assert dist >= mod.AXIS_CLEARANCE_M - 1e-12


def test_swap3_keeps_or_pushes_along_the_same_ray():
    mod = _script()
    for tag, _density, _n, geom in (("014", 0.14, 56, "ego-density-014.json"),
                                    ("028", 0.28, 112, "ego-density-028.json"),
                                    ("042", 0.42, 168, "ego-density-042.json")):
        raw = json.loads(_path(tag, "swap3").read_text(encoding="utf-8"))
        centers = mod.cylinder_centers(mod.load_geometry(geom))
        placed = mod.place_swap3(centers)
        assert len(raw["drones"]) == 3
        for i, (drone, info, (ux, uy)) in enumerate(zip(raw["drones"], placed, mod.SWAP_UNIT)):
            assert drone["id"] == f"uav{i}"
            start = np.asarray(drone["start_enu"], dtype=float)
            goal = np.asarray(drone["goal_enu"], dtype=float)
            assert np.allclose(start[:2] + goal[:2], 0.0)
            assert start[2] == 1.5 and goal[2] == 1.5
            ux, uy = mod._unit(ux, uy)
            r = float(np.hypot(start[0], start[1]))
            assert start[0] == pytest.approx(r * ux)
            assert start[1] == pytest.approx(r * uy)
            if info["pushed_outside"]:
                assert r > 8.0
                assert abs(start[0]) > 10.0 or abs(start[1]) > 10.0
                assert abs(goal[0]) > 10.0 or abs(goal[1]) > 10.0
                assert info["axis_distance_at_8m_start"] < mod.AXIS_CLEARANCE_M or info["axis_distance_at_8m_goal"] < mod.AXIS_CLEARANCE_M
                assert f"{drone['id']} 在 8 m 圆上" in raw["note"]
            else:
                assert r == pytest.approx(8.0)
                assert info["axis_distance_at_8m_start"] >= mod.AXIS_CLEARANCE_M
                assert info["axis_distance_at_8m_goal"] >= mod.AXIS_CLEARANCE_M


def test_solo_keeps_or_pushes_along_x():
    mod = _script()
    for tag, _density, _n, geom in (("014", 0.14, 56, "ego-density-014.json"),
                                    ("028", 0.28, 112, "ego-density-028.json"),
                                    ("042", 0.42, 168, "ego-density-042.json")):
        raw = json.loads(_path(tag, "solo").read_text(encoding="utf-8"))
        centers = mod.cylinder_centers(mod.load_geometry(geom))
        placed = mod.place_solo(centers)
        assert len(raw["drones"]) == 1
        start = raw["drones"][0]["start_enu"]
        goal = raw["drones"][0]["goal_enu"]
        assert start[1] == 0.0 and goal[1] == 0.0
        assert start[0] == pytest.approx(-goal[0])
        if placed["pushed_outside"]:
            assert abs(start[0]) > 10.0 and abs(goal[0]) > 10.0
            assert placed["axis_distance_at_pm9_start"] < mod.AXIS_CLEARANCE_M or placed["axis_distance_at_pm9_goal"] < mod.AXIS_CLEARANCE_M
            assert "移到 20 m 正方形外面" in raw["note"]
        else:
            assert start == [-9.0, 0.0, 1.5]
            assert goal == [9.0, 0.0, 1.5]


def test_combo_overlay_is_in_memory_only_and_matches_e_combo_flags():
    mod = _script()
    path = _path("014", "swap3")
    before = path.read_text(encoding="utf-8")
    raw = json.loads(before)
    off = mod.build_run_card("014", "swap3", "off")
    combo = mod.build_run_card("014", "swap3", "combo")
    assert path.read_text(encoding="utf-8") == before
    assert "policy_params" not in raw and "geometric_fallback" not in raw
    assert "inter_drone_filter" not in json.dumps(off.card)
    assert "geometric_fallback" not in off.card
    upstream = combo.card["policy_params"]["upstream"]
    assert upstream["inter_drone_filter"] is True
    assert upstream["inter_drone_threshold_m"] == 0.70
    assert upstream["inter_drone_penalty"] == 0
    assert combo.card["geometric_fallback"] == {
        "enabled": True,
        "inter_drone_distance_m": 0.7,
        "commit_horizon_s": 1.0,
    }
    assert combo.card["policy_params"]["backend"] == {"enabled": True}
    # 只开 enabled，迭代、预算、净空走代码缺省。
    assert "max_iterations" not in combo.card["policy_params"]["backend"]
    assert "time_budget_s" not in combo.card["policy_params"]["backend"]
    assert "clearance_m" not in combo.card["policy_params"]["backend"]
    assert off.hash != combo.hash
