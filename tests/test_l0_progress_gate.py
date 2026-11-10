"""票 13b：穿林进度门控。开阔区保持开环。不跑完整 seeds。"""

import copy

import numpy as np
import pytest

from yopo_l0.crossing import CrossingFleet
from yopo_l0.formation import FormationFleet
from yopo_l0.progress import read_progress_settings
from yopo_l0.scene import load_card
from yopo_swarm_core.plan import quintic
from yopo_swarm_core.trajectory import PiecewisePolynomial

from conftest import ROOT

PILLARS = ROOT / "scenes/crossing-pillars.json"
FOREST = ROOT / "scenes/crossing-forest.json"
SIX = ROOT / "scenes/crossing-6-forest.json"
HOLD = ROOT / "scenes/formation-hold.json"


def test_percentile_zero_is_the_min_and_leaders_are_those_ahead_of_it():
    fleet = CrossingFleet.from_card(load_card(SIX).card, 0.01)
    assert fleet.progress_gate is True
    assert fleet.progress_percentile == 0.0
    values = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    for did, s in zip(fleet.members, values):
        fleet._progress[did] = s
    assert fleet.gate_s() == 0.0
    fleet.progress_percentile = 20.0
    assert fleet.gate_s() == pytest.approx(10.0)
    fleet.progress_percentile = 0.0
    front = fleet.rail.at(10.0, fleet.slots["uav0"])
    assert fleet.rail.along(front, fleet.slots["uav0"]) == pytest.approx(10.0)
    for did in fleet.members:
        fleet._progress[did] = 0.0
    fleet._progress["uav0"] = 10.0
    assert fleet.gate_s() == 0.0
    assert fleet._ahead_of_gate("uav0", front) is True
    assert fleet._ahead_of_gate("uav5", fleet.rail.at(0.0, fleet.slots["uav5"])) is False
    # 计划头可以已经探到前面。门控仍看观测到的投影，落后机不被误判成超前。
    probed = fleet.rail.at(12.0, fleet.slots["uav5"])
    assert fleet._ahead_of_gate("uav5", probed) is False


def test_open_loop_schedule_ignores_progress_and_gated_goal_uses_the_percentile():
    card = load_card(PILLARS).card
    fleet = CrossingFleet.from_card(card, 0.01)
    assert fleet.progress_gate is False
    assert fleet.schedule_s(2.0) == pytest.approx(7.0)
    for did in fleet.members:
        fleet._progress[did] = 1.0
    assert fleet.schedule_s(2.0) == pytest.approx(7.0)
    gated = copy.deepcopy(card)
    gated["task"]["progress_gate"] = True
    gated["task"]["progress_percentile"] = 0
    hold = CrossingFleet.from_card(gated, 0.01)
    for did in hold.members:
        hold._progress[did] = 1.0
    assert hold.schedule_s(20.0) == pytest.approx(1.0)
    goal = hold.goal_for("uav0", 20.0, np.array([1.0, 0.5, 1.5]))
    # 弧长 1 m + 前瞻 2 s × 3.5 m/s = 8 m。开环在 t=20 已经到终点 19.5 m。
    assert goal[0] == pytest.approx(8.0)
    assert hold.summary(0, 3)["virtual_center"] == "progress-gated"
    assert hold.summary(0, 3)["gate_s_m"] == pytest.approx(1.0)


def test_forest_defaults_on_and_bad_settings_fail():
    card = load_card(FOREST).card
    bare = copy.deepcopy(card)
    bare["task"].pop("progress_gate")
    bare["task"].pop("progress_percentile")
    assert CrossingFleet.from_card(bare, 0.01).progress_gate is True
    pillars = load_card(PILLARS).card
    naked = copy.deepcopy(pillars)
    naked["task"].pop("progress_gate")
    assert CrossingFleet.from_card(naked, 0.01).progress_gate is False
    with pytest.raises(ValueError, match="progress_percentile"):
        read_progress_settings({"progress_percentile": 120}, default_gate=True)
    with pytest.raises(ValueError, match="布尔"):
        read_progress_settings({"progress_gate": 1}, default_gate=False)


def test_leader_hover_stops_gently_and_does_not_call_yopo():
    fleet = CrossingFleet.from_card(load_card(PILLARS).card, 0.01)
    fleet.progress_gate = True
    for did in fleet.members:
        fleet._progress[did] = 0.0

    class Boom:
        def forward_raw(self, *_a, **_k):
            raise AssertionError("悬停提案不应再跑 YOPO")

    fleet._source = Boom()
    head_p = fleet.rail.at(10.0, fleet.slots["uav0"])
    head = np.array([head_p, np.array([2.0, 0.0, 0.0]), np.zeros(3)])
    frame = type("F", (), {"depth": np.zeros((2, 2)), "stamp": 0.0})()
    occ = fleet._leader_proposal(
        fleet.nodes["uav0"], 1.0, head, np.array([15.0, 0.0, 1.5]), frame, 0.0
    )
    assert occ is not None
    assert fleet.code("uav0") == "LEADER_HOVER"
    acc = [
        float(np.linalg.norm(occ.plan.motion.evaluate(float(t), 2)))
        for t in np.linspace(occ.plan.motion.t_start, occ.plan.motion.t_end, 21)
    ]
    assert max(acc) <= fleet.gentle_limits.a_max * 1.01


def test_short_prefix_uses_the_scene_accel_not_the_contract_brake():
    fleet = CrossingFleet.from_card(load_card(PILLARS).card, 0.01)
    head = np.zeros((3, 3))
    head[0] = np.array([0.0, 0.0, 1.5])
    head[1] = np.array([3.0, 0.0, 0.0])
    end = np.array([[8.0, 0.0, 1.5], [3.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    raw = PiecewisePolynomial(0.0, [quintic(head, end, 2.0)])
    short = fleet._fit_short(raw)
    assert short is not None
    assert short.t_end - short.t_start < 2.0
    tail = np.linspace(short.t_start + 0.3, short.t_end, 15)
    acc = [float(np.linalg.norm(short.evaluate(float(t), 2))) for t in tail]
    assert max(acc) <= fleet.gentle_limits.a_max * 1.01


def test_formation_rejects_progress_gate_and_reports_the_switch_off():
    card = load_card(HOLD).card
    fleet = FormationFleet.from_card(card, 0.01)
    assert fleet.progress_gate is False
    assert fleet.summary(0, 3)["progress_gate"] is False
    assert fleet.summary(0, 3)["virtual_center"] == "scheduled"
    bad = copy.deepcopy(card)
    bad["task"]["progress_gate"] = True
    with pytest.raises(ValueError, match="开阔区"):
        FormationFleet.from_card(bad, 0.01)
