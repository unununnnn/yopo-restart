"""票 07c：密档撞树定位。最小复现是 dense seed 0，不改规划器。"""

import json

from yopo_l0.dense_diag import (
    CAT_CORRIDOR,
    CAT_HIDDEN,
    CAT_HOVER,
    CAT_OTHER,
    CAT_TRACK,
    CATEGORIES,
    _DiagFleet,
    _install,
    classify,
)
from yopo_l0.forest import card_for_tier
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card

from conftest import ROOT


def _facts(**kw):
    base = {
        "plan_code": "OK",
        "plan_clearance_m": 0.2,
        "actual_clearance_m": 0.2,
        "body_radius_m": 0.35,
        "kind": "trunk",
        "in_depth": True,
        "admitted": True,
    }
    base.update(kw)
    return base


def test_categories_follow_the_stated_order():
    assert classify(_facts(plan_code="CORRIDOR_EMPTY_HOVER", plan_clearance_m=1.0)) == CAT_HOVER
    assert classify(_facts(plan_clearance_m=0.5, actual_clearance_m=0.2)) == CAT_TRACK
    assert classify(_facts(kind="branch", in_depth=False, plan_clearance_m=0.1)) == CAT_HIDDEN
    assert classify(_facts(kind="lean", in_depth=False, plan_clearance_m=0.1)) == CAT_HIDDEN
    assert classify(_facts(plan_clearance_m=0.2, in_depth=True, admitted=True)) == CAT_CORRIDOR
    assert classify(_facts(plan_clearance_m=0.2, admitted=False, plan_code="OK")) == CAT_OTHER
    assert set(CATEGORIES) == {CAT_HIDDEN, CAT_CORRIDOR, CAT_HOVER, CAT_TRACK, CAT_OTHER}


def test_diag_logging_does_not_change_the_crossing(tmp_path):
    raw = json.loads((ROOT / "scenes/crossing-forest.json").read_text())
    path = tmp_path / "dense.json"
    path.write_text(json.dumps(card_for_tier(raw, "dense")))
    stock = run(load_card(path), RunConfig(seed=0, duration_s=0.8, shadow=False))
    previous = _install(_DiagFleet)
    try:
        diag = run(load_card(path), RunConfig(seed=0, duration_s=0.8, shadow=False))
    finally:
        _install(previous)
    assert [(t["uav"], t["p"], t["v"]) for t in stock.ticks] == [(t["uav"], t["p"], t["v"]) for t in diag.ticks]
    assert stock.result["plan_codes"] == diag.result["plan_codes"]
    assert stock.result["outcome"] == diag.result["outcome"]


def test_dense_seed0_replays_the_uav0_tree_hit():
    """最小复现：密档 seed 0，uav0 撞上深度里看得见的直立树干，计划净空小于机体半径。"""
    from yopo_l0.dense_diag import run_seed

    facts, _out = run_seed(0)
    assert facts["outcome"] == "COLLISION"
    assert facts["drone"] == "uav0"
    assert facts["reason"] == "uav0:TRUTH_POSITION"
    assert facts["kind"] == "trunk"
    assert facts["in_depth"] is True
    assert facts["plan_code"] == "OK"
    assert facts["plan_clearance_m"] <= facts["body_radius_m"]
    assert facts["category"] == CAT_CORRIDOR
    assert facts["window"]
    assert any(row["code"] == "OK" and row["n_admitted"] for row in facts["window"])
    report = (ROOT / "docs/results/07c-dense-diag.md").read_text()
    assert "走廊 0.05 m 过松" in report
    assert f"seed 0" in report
