"""票 B5：斜坡只改编队误差参考，不改规划时刻表。默认关，打开即偏离上游。"""

import numpy as np
import pytest

from conftest import ROOT
from yopo_l0 import crossing
from yopo_l0.b2_plan_diag import apply_b2_overrides, b3_from_fix, fix_deviations, parse_fix
from yopo_l0.scene import load_card

CROSS = ROOT / "scenes/crossing-pillars.json"


def _fleet(params=None):
    card = dict(load_card(CROSS).card)
    if params is not None:
        card["policy_params"] = params
    return crossing.CrossingFleet.from_card(card, 0.01)


def test_ref_ramp_is_off_by_default():
    off = _fleet()
    assert off.ref_rail is off.rail
    on = _fleet({"b3": {"ref_ramp_mps2": 3.0}})
    assert on.cfg.task_hash != off.cfg.task_hash
    with pytest.raises(ValueError):
        _fleet({"b3": {"ref_ramp_mps2": 7.0}})
    with pytest.raises(ValueError):
        _fleet({"b3": {"ref_ramp_mps2": 3.0, "center_ramp_mps2": 3.0}})


def test_ref_ramp_changes_only_the_error_reference():
    off, on = _fleet(), _fleet({"b3": {"ref_ramp_mps2": 3.0}})
    # 规划时刻表、终点和总时长都与缺省相同。
    assert on.rail.ramp_mps2 is None and on.rail.T == off.rail.T
    for t in (0.3, 1.0, 2.0, 5.0):
        assert on.schedule_s(t) == off.schedule_s(t)
    for did in on.members:
        assert np.allclose(on.goals[did], off.goals[did])
    # 参考中心是斜坡：飞机按斜坡起步时，缺省参考算出落后，斜坡参考是 0。
    did = on.members[0]
    p = on.ref_rail.position(0.6, on.t_go, on.slots[did])
    assert on.observe(did, 0.6, p) == pytest.approx(0.0)
    assert off.observe(did, 0.6, p) > 1.0


def test_ref_ramp_fix_and_deviation():
    fix = parse_fix("launch=3:1.5,reframp=3")
    assert b3_from_fix(fix) == {"launch_a_max_mps2": 3.0, "launch_s": 1.5, "ref_ramp_mps2": 3.0}
    assert any("编队误差参考" in d and d.startswith("偏离上游") for d in fix_deviations(fix))


def test_short_l0_flight_with_ref_ramp_marks_upstream_deviation():
    from yopo_l0.runner import RunConfig, run

    card, _ = apply_b2_overrides(load_card(CROSS), b3={"ref_ramp_mps2": 3.0})
    out = run(card, RunConfig(policy="crossing", seed=0, shadow=False, duration_s=1.0)).result
    assert out["upstream_deviation"] is True
    assert out["crossing"]["ref_ramp_mps2"] == 3.0
    assert "规划时刻表不变" in out["center_ramp_deviation"]
