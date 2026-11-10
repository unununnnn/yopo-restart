"""票 13a：承诺视界和急刹加速度两条不变量。不改编队目标，也不改进度门控。"""

import copy
import json
import logging
import math

import numpy as np
import pytest

from yopo_l0.crossing import (
    CrossingFleet,
    HORIZON_MARGIN_S,
    dynamics_a_max,
    plan_limits_for_dynamics,
    required_commit_horizon_s,
    resolve_commit_horizon_s,
)
from yopo_l0.dynamics import PointMassLag
from yopo_swarm_core.collision import Verdict, check_derivative_bound
from yopo_swarm_core.plan import KinematicLimits, brake_branch

from conftest import ROOT

PILLARS = ROOT / "scenes/crossing-pillars.json"
SIX = ROOT / "scenes/crossing-6-forest.json"


def _card(path):
    return json.loads(path.read_text())


def test_horizon_lower_bound_is_one_rotation_plus_margin():
    assert HORIZON_MARGIN_S == pytest.approx(0.25)
    assert required_commit_horizon_s(3, 0.25) == pytest.approx(1.0)
    assert required_commit_horizon_s(6, 0.25) == pytest.approx(1.75)
    assert required_commit_horizon_s(6, 0.5) == pytest.approx(3.25)
    assert required_commit_horizon_s(1, 0.25) == pytest.approx(0.5)


def test_short_horizon_is_raised_and_logged(caplog):
    with caplog.at_level(logging.WARNING, logger="yopo_l0.crossing"):
        got = resolve_commit_horizon_s(1.0, 6, 0.25)
    assert got == pytest.approx(1.75)
    assert len(caplog.records) == 1
    assert "抬高到" in caplog.records[0].message
    assert "1.75" in caplog.records[0].message


def test_horizon_on_the_bound_or_above_is_kept(caplog):
    with caplog.at_level(logging.WARNING, logger="yopo_l0.crossing"):
        exact = resolve_commit_horizon_s(1.0, 3, 0.25)
        longer = resolve_commit_horizon_s(2.5, 6, 0.25)
    assert exact == pytest.approx(1.0)
    assert longer == pytest.approx(2.5)
    assert caplog.records == []


@pytest.mark.parametrize("bad", [0.0, -0.1, math.nan, math.inf])
def test_non_positive_horizon_is_rejected(bad):
    with pytest.raises(ValueError):
        resolve_commit_horizon_s(bad, 3, 0.25)


def test_three_drone_card_already_meets_the_horizon_and_uses_dynamics_a_max(caplog):
    card = _card(PILLARS)
    with caplog.at_level(logging.WARNING, logger="yopo_l0.crossing"):
        fleet = CrossingFleet.from_card(card, 0.01)
    assert fleet.horizon_requested == pytest.approx(1.0)
    assert fleet.horizon == pytest.approx(required_commit_horizon_s(3, 0.25))
    assert fleet.pcfg.limits.a_max == pytest.approx(card["dynamics"]["a_max"])
    assert fleet.pcfg.limits.a_max == pytest.approx(6.0)
    assert fleet.pcfg.limits.a_max != pytest.approx(80.0)
    assert caplog.records == []


def test_six_drone_card_horizon_is_raised_without_editing_the_json(caplog):
    card = _card(SIX)
    assert card["task"]["commit_horizon_s"] == pytest.approx(1.0)
    with caplog.at_level(logging.WARNING, logger="yopo_l0.crossing"):
        fleet = CrossingFleet.from_card(card, 0.01)
    assert fleet.horizon_requested == pytest.approx(1.0)
    assert fleet.horizon == pytest.approx(required_commit_horizon_s(6, card["task"]["window_len_s"]))
    assert fleet.horizon > fleet.horizon_requested
    assert fleet.cfg.task_hash
    assert any("抬高到" in rec.message for rec in caplog.records)
    assert fleet.pcfg.limits.a_max == pytest.approx(6.0)


def test_omitted_horizon_uses_the_default_then_the_same_bound(caplog):
    card = _card(SIX)
    del card["task"]["commit_horizon_s"]
    with caplog.at_level(logging.WARNING, logger="yopo_l0.crossing"):
        fleet = CrossingFleet.from_card(card, 0.01)
    assert fleet.horizon_requested == pytest.approx(1.0)
    assert fleet.horizon == pytest.approx(1.75)
    assert caplog.records


def test_explicit_horizon_above_the_bound_is_not_rewritten(caplog):
    card = _card(SIX)
    card["task"]["commit_horizon_s"] = 2.5
    with caplog.at_level(logging.WARNING, logger="yopo_l0.crossing"):
        fleet = CrossingFleet.from_card(card, 0.01)
    assert fleet.horizon == pytest.approx(2.5)
    assert caplog.records == []


def test_plan_a_max_follows_the_scene_dynamics_and_not_a_hardcoded_80():
    assert dynamics_a_max(None) == pytest.approx(PointMassLag.a_max)
    assert dynamics_a_max({}) == pytest.approx(6.0)
    assert plan_limits_for_dynamics(None).a_max == pytest.approx(6.0)
    card = _card(PILLARS)
    card["dynamics"] = {**card["dynamics"], "a_max": 4.5}
    fleet = CrossingFleet.from_card(card, 0.01)
    assert fleet.pcfg.limits.a_max == pytest.approx(4.5)
    assert fleet.pcfg.limits.a_max != pytest.approx(80.0)
    bare = copy.deepcopy(_card(PILLARS))
    del bare["dynamics"]
    assert CrossingFleet.from_card(bare, 0.01).pcfg.limits.a_max == pytest.approx(6.0)


@pytest.mark.parametrize("bad", [0.0, -6.0, math.nan])
def test_non_positive_dynamics_a_max_is_rejected(bad):
    card = _card(PILLARS)
    card["dynamics"] = {**card["dynamics"], "a_max": bad}
    with pytest.raises(ValueError):
        CrossingFleet.from_card(card, 0.01)


def test_emergency_brake_at_dynamics_a_max_covers_a_stop_the_old_80_does_not():
    """11c 的空档：a_max=80 的急刹在契约里很短，按动力学 a_max=6 再查就不成立。"""
    limits = plan_limits_for_dynamics({"a_max": 6.0})
    head = np.array([[0.0, 0.0, 1.5], [3.5, 0.0, 0.0], [0.0, 0.0, 0.0]])
    honest = brake_branch(0.0, head, limits)
    fake = brake_branch(0.0, head, KinematicLimits(v_max=20.0, a_max=80.0, j_max=20000.0))
    assert honest is not None and fake is not None
    assert fake.t_end < honest.t_end
    assert check_derivative_bound(honest, 2, limits.a_max).verdict is Verdict.SAFE
    assert check_derivative_bound(fake, 2, limits.a_max).verdict is not Verdict.SAFE
