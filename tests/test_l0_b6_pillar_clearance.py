"""票 B6：规划净空过滤用场景柱几何。默认关，打开即偏离上游。"""

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


class _Line:
    """直线匀速运动，只给净空门用（t_start、t_end、evaluate）。"""

    def __init__(self, a, b, t0=0.0, t1=1.0):
        self.a, self.b = np.asarray(a, float), np.asarray(b, float)
        self.t_start, self.t_end = t0, t1

    def evaluate(self, t, order=0):
        u = (t - self.t_start) / (self.t_end - self.t_start)
        return self.a + u * (self.b - self.a)


def _line(a, b):
    return _Line(a, b)


def test_pillar_clearance_is_off_by_default_and_enters_hash_when_on():
    off = _fleet()
    assert off.pillar_clearance is None
    on = _fleet({"b3": {"pillar_clearance_m": 0.5}})
    assert on.pillar_clearance is not None and on.pillar_clearance.margin_m == 0.5
    assert on.cfg.task_hash != off.cfg.task_hash
    for bad in (0.0, -1.0, float("nan")):
        with pytest.raises(ValueError):
            _fleet({"b3": {"pillar_clearance_m": bad}})


def test_gate_rejects_a_motion_that_skims_a_pillar_and_keeps_a_clear_one():
    fleet = _fleet({"b3": {"pillar_clearance_m": 0.5}})
    # 柱 p_c0_r1 占 x 3.7–4.3、y −0.4–0.4。贴着 y=0.9 掠过：中心距 0.5，减机体 0.35 只剩 0.15。
    skim = _line([2.0, 0.9, 1.5], [6.0, 0.9, 1.5])
    clear = _line([2.0, 2.0, 1.5], [6.0, 2.0, 1.5])
    assert not fleet._clearance_ok(skim)
    assert fleet._clearance_ok(clear)
    s = fleet.pillar_clearance.summary()
    assert s["checks"] == 2 and s["rejected"] == 1 and s["deviation"].startswith("偏离上游")
    assert _fleet()._clearance_ok(skim)  # 关时柱林不开净空门


def test_fix_string_and_deviation():
    fix = parse_fix("launch=3:1.5,pillar=0.5")
    assert b3_from_fix(fix) == {"launch_a_max_mps2": 3.0, "launch_s": 1.5, "pillar_clearance_m": 0.5}
    assert any(d.startswith("偏离上游：场景柱几何辅助的规划净空过滤") for d in fix_deviations(fix))
    with pytest.raises(ValueError):
        parse_fix("pillar=0")


def test_short_l0_flight_marks_upstream_deviation():
    from yopo_l0.runner import RunConfig, run

    card, _ = apply_b2_overrides(load_card(CROSS), b3={"pillar_clearance_m": 0.5})
    out = run(card, RunConfig(policy="crossing", seed=0, shadow=False, duration_s=1.0)).result
    assert out["upstream_deviation"] is True
    assert out["crossing"]["pillar_clearance"]["checks"] > 0
    assert out["pillar_clearance_deviation"].startswith("偏离上游")
