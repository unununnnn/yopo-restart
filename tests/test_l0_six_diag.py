"""票 11c：六机撞树分类。定位器不改轨迹。"""

from yopo_l0.six_diag import (
    CAT_CORRIDOR,
    CAT_GOAL,
    CAT_HIDDEN,
    CAT_HOVER,
    CAT_OTHER,
    CAT_TRACK,
    CAT_YIELD,
    CATEGORIES,
    _DiagFleet,
    _install,
    classify,
)
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card

from conftest import ROOT

MEDIUM = ROOT / "scenes/crossing-6-forest.json"


def _facts(**kw):
    base = {
        "plan_code": "OK",
        "plan_clearance_m": 0.2,
        "actual_clearance_m": 0.2,
        "body_radius_m": 0.35,
        "kind": "trunk",
        "in_depth": True,
        "admitted": True,
        "goal_clearance_m": 2.0,
    }
    base.update(kw)
    return base


def test_categories_follow_the_stated_order():
    assert classify(_facts(plan_code="CONFLICT_BRAKE", plan_clearance_m=1.0)) == CAT_YIELD
    assert classify(_facts(plan_code="CORRIDOR_EMPTY_HOVER", plan_clearance_m=1.0)) == CAT_HOVER
    assert classify(_facts(plan_code="BRAKE_ONLY", plan_clearance_m=1.0)) == CAT_HOVER
    assert classify(_facts(plan_clearance_m=0.5, actual_clearance_m=0.2)) == CAT_TRACK
    assert classify(_facts(kind="branch", in_depth=False, plan_clearance_m=0.1)) == CAT_HIDDEN
    assert classify(_facts(goal_clearance_m=0.0, plan_clearance_m=0.2)) == CAT_GOAL
    assert classify(_facts(goal_clearance_m=0.35, plan_clearance_m=0.2)) == CAT_GOAL
    assert classify(_facts(goal_clearance_m=1.0, plan_clearance_m=0.2)) == CAT_CORRIDOR
    assert classify(_facts(plan_clearance_m=0.2, admitted=False)) == CAT_OTHER
    assert CATEGORIES[0] == CAT_YIELD
    assert CAT_CORRIDOR in CATEGORIES and CAT_GOAL in CATEGORIES


def test_diag_logging_does_not_change_the_six_drone_prefix():
    card = load_card(MEDIUM)
    cfg = RunConfig(seed=0, policy="crossing", duration_s=0.4, shadow=False)
    stock = run(card, cfg)
    previous = _install(_DiagFleet)
    try:
        diag = run(card, cfg)
    finally:
        _install(previous)
    assert [(t["uav"], t["p"], t["v"]) for t in stock.ticks] == [(t["uav"], t["p"], t["v"]) for t in diag.ticks]
    assert stock.result["plan_codes"] == diag.result["plan_codes"]
    assert stock.result["outcome"] == diag.result["outcome"]
