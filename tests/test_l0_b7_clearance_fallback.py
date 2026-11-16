"""票 B7：柱净空门把候选全拒时，退回离柱面最远的那个，避免 PLAN_TIMEOUT。"""

import numpy as np

from conftest import ROOT
from yopo_l0 import crossing
from yopo_l0.scene import load_card

CROSS = ROOT / "scenes/crossing-pillars.json"


def _fleet(params=None):
    card = dict(load_card(CROSS).card)
    if params is not None:
        card["policy_params"] = params
    return crossing.CrossingFleet.from_card(card, 0.01)


class _Line:
    def __init__(self, a, b, t0=0.0, t1=1.0):
        self.a, self.b = np.asarray(a, float), np.asarray(b, float)
        self.t_start, self.t_end = t0, t1

    def evaluate(self, t, order=0):
        u = (t - self.t_start) / (self.t_end - self.t_start)
        return self.a + u * (self.b - self.a)


def _line(a, b):
    return _Line(a, b)


# 柱 p_c0_r1 占 x 3.7–4.3、y −0.4–0.4。门限 0.5 m 时：
# y=0.9 净空约 0.15 m（扣采样保守量后更低），y=1.1 约 0.35 m，都过不了门；y=2.0 过得了。
NEAR = _line([2.0, 0.9, 1.5], [6.0, 0.9, 1.5])
MID = _line([2.0, 1.1, 1.5], [6.0, 1.1, 1.5])
FAR = _line([2.0, 1.2, 1.5], [6.0, 1.2, 1.5])
CLEAR = _line([2.0, 2.0, 1.5], [6.0, 2.0, 1.5])


def test_default_off_does_not_fall_back_and_hash_unchanged():
    off = _fleet()
    on = _fleet({"b3": {"pillar_clearance_m": 0.5}})
    assert off.pillar_clearance is None
    assert on.cfg.task_hash != off.cfg.task_hash
    picked = []

    def fake(node, t_start, head, motion):
        picked.append(motion)
        return object(), None

    off._as_plan = fake
    occ, reject, used_alt, used_fallback = off._take_motions(None, 0.0, None, [NEAR, MID])
    assert used_fallback is False and occ is not None and picked == [NEAR]


def test_all_rejected_falls_back_to_farthest_from_pillars():
    """对应 B6 锁步 SITL (a)+0.3 m seed 7：门连拒，没有可用计划，直到 PLAN_TIMEOUT。"""
    fleet = _fleet({"b3": {"pillar_clearance_m": 0.5}})
    speed = fleet.pcfg.limits.v_max
    gate = fleet.pillar_clearance
    assert gate.gap(NEAR, speed) < gate.gap(MID, speed) < gate.gap(FAR, speed) < 0.5
    assert not gate.allows(NEAR, speed) and not gate.allows(MID, speed) and not gate.allows(FAR, speed)
    gate.checks = gate.rejected = 0

    picked = []

    def fake(node, t_start, head, motion):
        picked.append(motion)
        return object(), None

    fleet._as_plan = fake
    # 更远的排在后面，确认不是「取第一个」。
    occ, reject, used_alt, used_fallback = fleet._take_motions(None, 0.0, None, [NEAR, MID, FAR])
    assert used_fallback is True and occ is not None and picked == [FAR]
    s = gate.summary()
    assert s["fallbacks"] == 1 and s["rejected"] == 3 and s["checks"] == 3
    assert "全拒时退回离柱最远候选" in s["deviation"]
    assert s["deviation"].startswith("偏离上游")

    # 再调一次：三个一样的近距候选，仍能退回其中一个，不会交空。
    picked.clear()
    occ, _, _, used_fallback = fleet._take_motions(None, 0.0, None, [NEAR, NEAR, NEAR])
    assert used_fallback is True and occ is not None and picked == [NEAR]
    assert gate.fallbacks == 2


def test_one_clear_candidate_is_taken_without_fallback():
    fleet = _fleet({"b3": {"pillar_clearance_m": 0.5}})
    picked = []

    def fake(node, t_start, head, motion):
        picked.append(motion)
        return object(), None

    fleet._as_plan = fake
    occ, reject, used_alt, used_fallback = fleet._take_motions(None, 0.0, None, [NEAR, CLEAR, FAR])
    assert used_fallback is False and used_alt is True
    assert occ is not None and picked == [CLEAR]
    assert fleet.pillar_clearance.fallbacks == 0
    assert fleet.pillar_clearance.rejected == 1  # 只检查到 CLEAR 就停


def test_fallback_not_used_when_as_plan_fails_on_the_best():
    fleet = _fleet({"b3": {"pillar_clearance_m": 0.5}})

    def fake(node, t_start, head, motion):
        return None, "CONTRACT"

    fleet._as_plan = fake
    occ, reject, used_alt, used_fallback = fleet._take_motions(None, 0.0, None, [NEAR, FAR])
    assert occ is None and used_fallback is False and reject == "CONTRACT"
    assert fleet.pillar_clearance.fallbacks == 0
