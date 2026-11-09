"""票 07b：泊松盘树干林。只查几何和卡能加载，不把碰撞数当门。"""

import json

import numpy as np

from yopo_l0.forest import TIERS, _DENSE_LO, _WOODS_HI, _WOODS_LO, card_for_tier, generate_forest
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import Cylinder, load_card
from yopo_l0.sensor import _hit_cylinder, _hit_cylinder_lean

from conftest import ROOT

CARD = ROOT / "scenes/crossing-forest.json"


def _surface_gaps(geom):
    trunks = [c for c in geom["cylinders"]]
    gaps = []
    for i, a in enumerate(trunks):
        ca = Cylinder(a["name"], tuple(a["center_xy"]), a["radius"], a["z_lo"], a["z_hi"], tuple(a["lean_xy"]))
        a0, a1 = ca.axis_ends()
        for b in trunks[i + 1 :]:
            cb = Cylinder(b["name"], tuple(b["center_xy"]), b["radius"], b["z_lo"], b["z_hi"], tuple(b["lean_xy"]))
            b0, b1 = cb.axis_ends()
            from yopo_l0.forest import _segment_distance

            gaps.append(_segment_distance(a0, a1, b0, b1) - a["radius"] - b["radius"])
    return gaps


def test_tiers_are_deterministic_poisson_forests():
    counts = {}
    for tier in TIERS:
        geom, stats = generate_forest({"density": tier, "seed": 7})
        again, _ = generate_forest({"density": tier, "seed": 7})
        assert geom == again
        assert stats["radius_min_m"] >= 0.10 - 1e-9
        assert stats["radius_max_m"] <= 0.40 + 1e-9
        assert stats["min_surface_gap_m"] + 1e-6 >= stats["surface_gap_spec_m"]
        assert min(_surface_gaps(geom)) + 1e-6 >= stats["surface_gap_spec_m"]
        assert all(c["center_xy"][0] <= _WOODS_HI + 1e-6 for c in geom["cylinders"])
        assert all(c["center_xy"][0] >= _WOODS_LO - 1e-6 for c in geom["cylinders"])
        entry_len = _DENSE_LO - _WOODS_LO
        mid_len = _WOODS_HI - _DENSE_LO
        assert stats["n_entry"] / entry_len < stats["n_middle"] / mid_len
        counts[tier] = stats
    # 4×3 柱是 12 个障碍。中档大约 3 倍，密档大约 5 倍，而且一档比一档密。
    assert counts["sparse"]["n_obstacles"] < counts["medium"]["n_obstacles"] < counts["dense"]["n_obstacles"]
    assert 30 <= counts["medium"]["n_obstacles"] <= 50
    assert 36 <= counts["dense"]["n_obstacles"] <= 60
    assert counts["dense"]["n_lean"] >= 1
    assert counts["dense"]["n_branches"] >= 1
    assert counts["dense"]["surface_gap_spec_m"] == 1.7


def test_card_loads_medium_and_other_tiers_differ():
    raw = json.loads(CARD.read_text())
    assert raw["forest"]["density"] == "medium"
    assert raw["seeds"] == list(range(10))
    assert raw["criteria"]["frozen"] is False
    card = load_card(CARD)
    again = load_card(CARD)
    assert card.hash == again.hash
    assert card.card["geometry"]["density"] == "medium"
    n_med = len(card.card["geometry"]["cylinders"])
    dense = card_for_tier(raw, "dense")
    # 密档几何不同，但规划参数和票 07 的窗口、前瞻一样，没有为了过林去改。
    assert dense["task"]["window_len_s"] == raw["task"]["window_len_s"]
    assert dense["task"]["lookahead_s"] == raw["task"]["lookahead_s"]
    assert dense["forest"]["density"] == "dense"
    from yopo_l0.forest import generate_forest_geometry

    n_dense = len(generate_forest_geometry(dense["forest"])["cylinders"])
    assert n_dense > n_med
    radius = card.card["criteria"]["body_radius_m"]
    for drone in card.card["drones"]:
        start = np.asarray(drone["start_enu"], dtype=float)
        assert card.min_distance(start) > radius + 1.0


def test_upright_cylinder_ray_matches_the_old_vertical_path():
    cyl = Cylinder("up", (2.0, 0.0), 0.3, 0.0, 4.0, (0.0, 0.0))
    lean = Cylinder("lean", (2.0, 0.0), 0.3, 0.0, 4.0, (1.2, 0.0))
    origin = np.array([0.0, 0.0, 1.5])
    dirs = np.array([[[1.0, 0.0, 0.0]]])
    assert cyl.upright()
    hit_up = float(_hit_cylinder(origin, dirs, cyl)[0, 0])
    # 斜树的顶在 x=3.2。飞行高度 z=1.5 处轴线在 x=2.45，表面在 x=2.15；竖直表面在 x=1.7。
    hit_lean = float(_hit_cylinder_lean(origin, dirs, lean)[0, 0])
    assert np.isfinite(hit_up) and np.isfinite(hit_lean)
    assert hit_lean > hit_up
    point = np.array([2.45, 0.0, 1.5])
    assert lean.distance(point) < 1e-6
    assert lean.distance(np.array([2.45, 0.5, 1.5])) == np.float64(0.2) or abs(lean.distance(np.array([2.45, 0.5, 1.5])) - 0.2) < 1e-9


def test_short_run_uses_the_crossing_policy_and_records_density():
    card = load_card(CARD)
    out = run(card, RunConfig(seed=0, duration_s=0.4, shadow=False))
    r = out.result
    assert r["task"] == "crossing" and r["policy"] == "crossing"
    assert r["forest_density"] == "medium"
    assert r["crossing"]["adjudicator"] is None
    assert r["outcome"] != "COLLISION"
    for row in out.ticks:
        assert row["approved_owner"] == row["uav"]
