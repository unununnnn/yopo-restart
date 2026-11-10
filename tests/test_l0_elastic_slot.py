"""票 13c：林带弹性槽位，COMMIT 前硬净空门。三机和柱林仍是刚性槽位前瞻。"""

import numpy as np
import pytest

from yopo_l0.crossing import CrossingFleet
from yopo_l0.dynamics import PointMassLag
from yopo_l0.scene import Cylinder, load_card
from yopo_swarm_core.plan import quintic
from yopo_swarm_core.trajectory import PiecewisePolynomial

from conftest import ROOT

MEDIUM = ROOT / "scenes/crossing-6-forest.json"
THREE = ROOT / "scenes/crossing-forest.json"


def _rigid_goal(fleet: CrossingFleet, drone_id: str, t: float, head_p: np.ndarray) -> np.ndarray:
    """出林 / 三机用的槽位前瞻。和改弹性槽位之前的公式一致。"""
    slot = fleet.slots[drone_id]
    head_p = np.asarray(head_p, dtype=float)
    final = fleet.goals[drone_id]
    ahead = fleet.rail.position(t + fleet.lookahead_s, fleet.t_go, slot)
    aim_x = min(float(final[0]), max(float(ahead[0]), float(head_p[0]) + fleet.min_ahead_m))
    aim = np.array([aim_x, float(ahead[1]), float(ahead[2])])
    remain = max(float(final[0] - ahead[0]), 0.0)
    reach = max(fleet.lookahead_s * fleet.rail.speed, 1e-6)
    w_final = 1.0 - min(remain / reach, 1.0)
    goal = (1.0 - w_final) * aim + w_final * final
    lane_y = float(fleet.rail.position(t, fleet.t_go, slot)[1])
    goal = np.array(goal, dtype=float, copy=True)
    goal[1] += fleet.lateral_gain * (lane_y - float(head_p[1]))
    goal[2] = float(final[2])
    if float(final[0]) - float(head_p[0]) > 1.0:
        goal[0] = min(float(final[0]), max(float(goal[0]), float(head_p[0]) + 1.0))
    return goal


def _line(y: float = 0.0) -> PiecewisePolynomial:
    head = np.array([[10.0, y, 1.5], [2.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    end = np.array([[14.0, y, 1.5], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    return PiecewisePolynomial(0.0, [quintic(head, end, 1.2)])


class _Frame:
    depth = np.zeros((2, 2), dtype=np.float32)
    stamp = 0.0


def test_margin_defaults_and_rejects_a_negative_value():
    fleet = CrossingFleet.from_card(load_card(MEDIUM).card, 0.01)
    assert fleet.clearance_margin_m == pytest.approx(0.15)
    assert fleet._required_clearance() == pytest.approx(0.35 + 0.15)
    assert fleet.safe_radius_m == pytest.approx(0.05)
    assert (PointMassLag().kp, PointMassLag().kd, PointMassLag().tau_s) == (4.0, 3.0, 0.1)
    raw = load_card(MEDIUM).card
    wider = {**raw, "task": {**raw["task"], "clearance_margin_m": 0.4}}
    other = CrossingFleet.from_card(wider, 0.01)
    assert other.clearance_margin_m == pytest.approx(0.4)
    assert other._required_clearance() == pytest.approx(0.35 + 0.4)
    bad = {**raw, "task": {**raw["task"], "clearance_margin_m": -0.01}}
    with pytest.raises(ValueError):
        CrossingFleet.from_card(bad, 0.01)


def test_clearance_gate_skips_a_trajectory_that_enters_a_tree():
    fleet = CrossingFleet.from_card(load_card(MEDIUM).card, 0.01)
    blocked = _line()
    fleet._trees = (Cylinder("t000", (12.0, 0.0), 0.25, 0.0, 6.0),)
    assert fleet._trajectory_min_clearance(blocked) < fleet._required_clearance()
    assert fleet._clearance_ok(blocked) is False
    clear = _line()
    fleet._trees = (Cylinder("t000", (12.0, 3.0), 0.2, 0.0, 6.0),)
    assert fleet._clearance_ok(clear) is True
    # 0.60 m 过默认 0.50 m 的门，margin 收到 0.30 m 就不够。
    borderline = _line()
    fleet._trees = (Cylinder("t000", (12.0, 0.80), 0.20, 0.0, 6.0),)
    assert fleet._trajectory_min_clearance(borderline) == pytest.approx(0.60, abs=0.03)
    assert fleet._clearance_ok(borderline) is True
    fleet.clearance_margin_m = 0.30
    assert fleet._clearance_ok(borderline) is False
    three = CrossingFleet.from_card(load_card(THREE).card, 0.01)
    three._trees = (Cylinder("t000", (12.0, 0.0), 0.25, 0.0, 6.0),)
    assert three._clearance_ok(blocked) is True
    assert three._commit_span() == three.horizon
    assert three.summary(0, 3)["goal_bias"] == "slot-lookahead-blend"


def test_clearance_skip_does_not_hand_the_curve_to_commit():
    """不可行走的轨迹不进 _as_plan，也就不会被提案或 COMMIT。"""
    fleet = CrossingFleet.from_card(load_card(MEDIUM).card, 0.01)
    motion = _line()
    fleet._trees = (Cylinder("t000", (12.0, 0.0), 0.25, 0.0, 6.0),)
    calls = {"n": 0}

    def _as_plan(*args, **kwargs):
        calls["n"] += 1
        raise AssertionError("净空不够的多项式不该进提交")

    fleet._as_plan = _as_plan
    fleet._yopo_options = lambda *args, **kwargs: ([motion], "OK")
    # 后排 x=0，前方 5 m 还没进林，不走弹性目标，只试这一次前向。
    fleet._sense["uav4"] = (_Frame(), 0.0)
    occ = fleet._planner(fleet.nodes["uav4"], 0.2, 0.2)
    assert occ is None
    assert calls["n"] == 0
    assert fleet._codes["uav4"]["CLEARANCE_SKIP"] >= 1


def test_a_clear_curve_still_reaches_the_commit_check():
    fleet = CrossingFleet.from_card(load_card(MEDIUM).card, 0.01)
    motion = _line()
    fleet._trees = (Cylinder("t000", (12.0, 3.0), 0.2, 0.0, 6.0),)
    calls = {"n": 0}
    real = fleet._as_plan

    def _as_plan(*args, **kwargs):
        calls["n"] += 1
        return real(*args, **kwargs)

    fleet._as_plan = _as_plan
    fleet._yopo_options = lambda *args, **kwargs: ([motion], "OK")
    fleet._sense["uav4"] = (_Frame(), 0.0)
    fleet._planner(fleet.nodes["uav4"], 0.2, 0.2)
    assert calls["n"] == 1


def test_woods_slot_is_soft_and_exit_returns_to_the_rigid_slot():
    six = CrossingFleet.from_card(load_card(MEDIUM).card, 0.01)
    assert six.summary(0, 6)["goal_bias"] == "elastic-slot"
    assert six._commit_span() == pytest.approx(1.5)
    head = np.array([16.0, -3.5, 1.5])
    assert six._elastic_active(16.0) is True
    assert six._slack_scale(16.0) == pytest.approx(1.0)
    assert six._slack_scale(28.0) == pytest.approx(0.5)
    assert six._slack_scale(30.0) == pytest.approx(0.0)
    slot_y = float(six.slots["uav3"][1])
    goals = six._elastic_goals(head, six.goals["uav3"], slot_y)
    goal = goals[0]
    assert np.allclose(goal, six.goal_for("uav3", 8.0, head))
    assert abs(float(goal[1]) - slot_y) <= six.lateral_slack_m + 1e-6
    assert abs(float(goal[1]) - float(head[1])) <= six.lateral_slack_m + 1e-6
    assert float(head[0]) < float(goal[0]) <= float(head[0]) + six.longitudinal_slack_m + 1e-6
    # 时刻表上的槽位已经跑到三十米外。软目标不追那一条穿树的远点。
    timetable = six.rail.position(8.0 + six.lookahead_s, six.t_go, six.slots["uav3"])
    assert float(goal[0]) < float(timetable[0]) - 5.0
    slot_reach, _probe, _point = six._lane_reach(head, slot_y, float(six.goals["uav3"][2]))
    if slot_reach + 1e-9 >= 1.0:
        assert abs(float(goal[1]) - slot_y) < 1e-6
    walkable = False
    for y in six._candidate_ys(float(head[1]), slot_y, 1.0):
        reach, _probe, _point = six._lane_reach(head, y, float(six.goals["uav3"][2]))
        if reach + 1e-9 >= 1.0:
            walkable = True
            break
    if walkable:
        gap = six._segment_clearance(head, goal)
        assert gap + 1e-6 >= six._required_clearance()
    early = np.array([2.0, -4.2, 1.5])
    assert six._elastic_active(2.0) is False
    assert np.allclose(six.goal_for("uav3", 0.4, early), _rigid_goal(six, "uav3", 0.4, early))
    assert six._elastic_active(31.0) is False
    out = np.array([31.0, -3.2, 1.5])
    assert np.allclose(six.goal_for("uav3", 12.0, out), _rigid_goal(six, "uav3", 12.0, out))
    three = CrossingFleet.from_card(load_card(THREE).card, 0.01)
    kept = np.array([16.0, 1.0, 1.5])
    assert three._elastic_active(16.0) is False
    assert np.allclose(three.goal_for("uav0", 8.0, kept), _rigid_goal(three, "uav0", 8.0, kept))
