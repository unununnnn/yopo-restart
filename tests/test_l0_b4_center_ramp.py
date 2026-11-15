"""票 B4：交叉卡虚拟中心起步斜坡。默认关，打开即偏离上游。"""

import numpy as np
import pytest

from conftest import ROOT
from yopo_l0 import crossing
from yopo_l0.b2_plan_diag import b3_from_fix, fix_deviations, parse_fix
from yopo_l0.scene import load_card

CROSS = ROOT / "scenes/crossing-pillars.json"


def _fleet(params=None):
    card = dict(load_card(CROSS).card)
    if params is not None:
        card["policy_params"] = params
    return crossing.CrossingFleet.from_card(card, 0.01)


def test_ramp_is_off_by_default_and_keeps_the_rail_and_task_hash():
    off = _fleet()
    assert off.rail.ramp_mps2 is None and off.rail.T == pytest.approx(off.rail.length / off.rail.speed)
    assert off.rail.arc(1.0, 0.0) == pytest.approx(off.rail.speed)  # 一步到巡航速度
    on = _fleet({"b3": {"center_ramp_mps2": 3.0}})
    assert on.b3 == {"center_ramp_mps2": 3.0}
    assert on.cfg.task_hash != off.cfg.task_hash
    with pytest.raises(ValueError):
        _fleet({"b3": {"center_ramp_mps2": 7.0}})  # 不能高于动力学 a_max
    with pytest.raises(ValueError):
        _fleet({"b3": {"center_ramp_mps2": 0.0}})


def test_ramp_accelerates_then_cruises_and_ends_at_the_same_goal():
    off, on = _fleet(), _fleet({"b3": {"center_ramp_mps2": 3.0}})
    rail = on.rail
    v, a = rail.speed, 3.0
    tr = v / a
    assert rail.arc(0.5 * tr, 0.0) == pytest.approx(0.5 * a * (0.5 * tr) ** 2)
    # 加速段末速度连续：差分速度等于 speed。
    h = 1e-4
    assert (rail.arc(tr + h, 0.0) - rail.arc(tr - h, 0.0)) / (2 * h) == pytest.approx(v, rel=1e-3)
    assert rail.T == pytest.approx(off.rail.T + 0.5 * tr)
    assert rail.arc(rail.T, 0.0) == pytest.approx(rail.length)
    for did in on.members:
        assert np.allclose(on.goals[did], off.goals[did]) and np.allclose(on.starts[did], off.starts[did])
    # 斜坡中心永远不超前缺省中心。
    ts = np.linspace(0.0, rail.T, 200)
    assert all(rail.arc(t, 0.0) <= off.rail.arc(t, 0.0) + 1e-9 for t in ts)


def test_fix_string_maps_ramp_and_marks_deviation():
    fix = parse_fix("launch=3:1.5,ramp=3")
    assert b3_from_fix(fix) == {"launch_a_max_mps2": 3.0, "launch_s": 1.5, "center_ramp_mps2": 3.0}
    assert "偏离上游：虚拟中心起步按 3 m/s² 斜坡" in fix_deviations(fix)
    with pytest.raises(ValueError):
        parse_fix("ramp=0")



def test_short_l0_flight_with_ramp_marks_upstream_deviation():
    from yopo_l0.b2_plan_diag import apply_b2_overrides
    from yopo_l0.runner import RunConfig, run

    card, _ = apply_b2_overrides(load_card(CROSS), b3={"center_ramp_mps2": 3.0})
    out = run(card, RunConfig(policy="crossing", seed=0, shadow=False, duration_s=1.0)).result
    assert out["upstream_deviation"] is True
    assert out["crossing"]["center_ramp_mps2"] == 3.0
    assert out["center_ramp_deviation"].startswith("偏离上游")
