"""票 E-PG3：超前机短前缀开关。默认关，关时与当前 main 逐位一致。"""

import copy
import json
from pathlib import Path

import numpy as np
import pytest

from yopo_l0.crossing import CrossingFleet, _LEADER_PREFIX_S
from yopo_l0.progress import LEADER_PREFIX_MAX_S, LEADER_PREFIX_MIN_S, read_leader_prefix
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import SceneCard, load_card
from yopo_swarm_core.plan import quintic
from yopo_swarm_core.trajectory import PiecewisePolynomial

from conftest import ROOT

SIX = ROOT / "scenes/crossing-6-forest.json"
SPARSE = ROOT / "scenes/crossing-6-forest-sparse.json"
PILLARS = ROOT / "scenes/crossing-pillars.json"
FIXTURE = Path(__file__).parent / "fixtures" / "pg2_prechange_crossing6_forest_seed0_result.json"
_STRIP = set(WALL_FIELDS) | {"commit"}


def _canon(obj):
    return json.loads(json.dumps(obj))


def _strip(result):
    return {k: v for k, v in result.items() if k not in _STRIP}


def _raw_coast():
    head = np.zeros((3, 3))
    head[0] = np.array([0.0, 0.0, 1.5])
    head[1] = np.array([3.0, 0.0, 0.0])
    end = np.array([[8.0, 0.0, 1.5], [3.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    return PiecewisePolynomial(0.0, [quintic(head, end, 2.0)])


def _departs_at(raw, fitted) -> float:
    """拟合曲线离开原多项式的时刻。短前缀在这一刻之前系数不变。"""
    step = 0.02
    t = 0.0
    while t <= 1.8 + 1e-9:
        if t >= float(fitted.t_end) - 1e-9:
            return t
        gap = np.linalg.norm(np.asarray(fitted.evaluate(t), dtype=float) - np.asarray(raw.evaluate(t), dtype=float))
        if float(gap) > 1e-3:
            return t
        t += step
    raise AssertionError("短前缀没有离开原多项式")


def _moving_head(fleet):
    head_p = fleet.rail.at(10.0, fleet.slots["uav0"])
    return np.array([head_p, np.array([2.0, 0.0, 0.0]), np.zeros(3)])


def test_scene_cards_omit_the_switch_and_the_default_is_off():
    for path in (SIX, SPARSE):
        raw = json.loads(path.read_text())
        assert raw["task"]["progress_gate"] is False
        assert "leader_prefix" not in raw["task"]
        assert "leader_prefix_s" not in raw["task"]
    assert _LEADER_PREFIX_S == pytest.approx(0.3)
    assert read_leader_prefix({}) == (False, pytest.approx(0.3))
    assert read_leader_prefix({"leader_prefix": False, "leader_prefix_s": 1.0}) == (False, pytest.approx(0.3))
    assert read_leader_prefix({"leader_prefix": True}) == (True, pytest.approx(0.3))
    assert read_leader_prefix({"leader_prefix": True, "leader_prefix_s": 0.7}) == (True, pytest.approx(0.7))
    assert read_leader_prefix({"leader_prefix": True, "leader_prefix_s": LEADER_PREFIX_MIN_S})[1] == pytest.approx(
        LEADER_PREFIX_MIN_S
    )
    assert read_leader_prefix({"leader_prefix": True, "leader_prefix_s": LEADER_PREFIX_MAX_S})[1] == pytest.approx(
        LEADER_PREFIX_MAX_S
    )

    card = copy.deepcopy(load_card(SIX).card)
    fleet = CrossingFleet.from_card(card, 0.01)
    assert fleet.leader_prefix_enabled is False
    assert fleet.leader_prefix_s == pytest.approx(0.3)
    assert fleet.progress_gate is False
    summary = fleet.summary(0, 6)
    assert "leader_prefix" not in summary
    assert "leader_prefix_s" not in summary
    assert summary["leader_margin_m"] is None


def test_switch_off_keeps_the_task_hash_and_switch_on_changes_it():
    base = copy.deepcopy(load_card(SIX).card)
    plain = CrossingFleet.from_card(base, 0.01)

    explicit = copy.deepcopy(base)
    explicit["task"]["leader_prefix"] = False
    explicit["task"]["leader_prefix_s"] = 1.0
    off = CrossingFleet.from_card(explicit, 0.01)
    assert off.leader_prefix_enabled is False
    assert off.leader_prefix_s == pytest.approx(_LEADER_PREFIX_S)
    assert off.cfg.task_hash == plain.cfg.task_hash
    assert "leader_prefix" not in off.summary(0, 6)

    opened = copy.deepcopy(base)
    opened["task"]["progress_gate"] = True
    opened["task"]["leader_prefix"] = True
    default_len = CrossingFleet.from_card(opened, 0.01)
    longer = copy.deepcopy(opened)
    longer["task"]["leader_prefix_s"] = 1.0
    wide = CrossingFleet.from_card(longer, 0.01)
    assert default_len.leader_prefix_enabled is True
    assert default_len.leader_prefix_s == pytest.approx(0.3)
    assert wide.leader_prefix_s == pytest.approx(1.0)
    assert default_len.cfg.task_hash != plain.cfg.task_hash
    assert wide.cfg.task_hash != default_len.cfg.task_hash
    assert default_len.summary(0, 6)["leader_prefix"] is True
    assert default_len.summary(0, 6)["leader_prefix_s"] == pytest.approx(0.3)
    assert wide.summary(0, 6)["leader_prefix_s"] == pytest.approx(1.0)


def test_illegal_leader_prefix_is_rejected():
    card = copy.deepcopy(load_card(SIX).card)
    for raw in (1, 0, "true", "false", None, 1.0):
        with pytest.raises(ValueError, match="leader_prefix"):
            read_leader_prefix({"leader_prefix": raw})
        bad = copy.deepcopy(card)
        bad["task"]["leader_prefix"] = raw
        with pytest.raises(ValueError, match="leader_prefix"):
            CrossingFleet.from_card(bad, 0.01)
    for raw in ("0.7", True, False, None, 0.1, 2.1, -0.3, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="leader_prefix_s"):
            read_leader_prefix({"leader_prefix": True, "leader_prefix_s": raw})
        bad = copy.deepcopy(card)
        bad["task"]["leader_prefix"] = False
        bad["task"]["leader_prefix_s"] = raw
        with pytest.raises(ValueError, match="leader_prefix_s"):
            CrossingFleet.from_card(bad, 0.01)


def test_switch_off_still_hovers_without_calling_yopo_even_if_a_length_is_written():
    card = copy.deepcopy(load_card(PILLARS).card)
    card["task"]["leader_prefix"] = False
    card["task"]["leader_prefix_s"] = 1.0
    fleet = CrossingFleet.from_card(card, 0.01)
    fleet.progress_gate = True

    class Boom:
        def forward_raw(self, *_a, **_k):
            raise AssertionError("开关关着时悬停提案不应再跑 YOPO")

    fleet._source = Boom()
    frame = type("F", (), {"depth": np.zeros((2, 2)), "stamp": 0.0})()
    occ = fleet._leader_proposal(fleet.nodes["uav0"], 1.0, _moving_head(fleet), np.array([15.0, 0.0, 1.5]), frame, 0.0)
    assert occ is not None
    assert fleet.code("uav0") == "LEADER_HOVER"


def test_switch_on_fits_the_requested_prefix_and_ignores_it_when_off():
    fleet = CrossingFleet.from_card(load_card(PILLARS).card, 0.01)
    raw = _raw_coast()
    fleet.leader_prefix_enabled = False
    fleet.leader_prefix_s = 1.0
    off = fleet._fit_short(raw)
    assert off is not None
    off_t = _departs_at(raw, off)

    fleet.leader_prefix_enabled = True
    fleet.leader_prefix_s = 0.3
    same = fleet._fit_short(raw)
    assert same is not None
    same_t = _departs_at(raw, same)
    # 开关关时不采用卡上或属性里的 1.0 s，仍按 0.3 s 截。
    assert same_t == pytest.approx(off_t, abs=0.021)
    assert off_t < 0.5

    fleet.leader_prefix_s = 0.7
    mid = fleet._fit_short(raw)
    fleet.leader_prefix_s = 1.0
    long = fleet._fit_short(raw)
    assert mid is not None and long is not None
    # 刹车接上之后位置还会贴着原曲线一小段，所以比名义长度晚一截。长度差仍是参数差。
    assert _departs_at(raw, mid) - off_t == pytest.approx(0.4, abs=0.05)
    assert _departs_at(raw, long) - _departs_at(raw, mid) == pytest.approx(0.3, abs=0.05)


def test_switch_on_tries_the_prefix_then_waits_or_holds():
    fleet = CrossingFleet.from_card(load_card(PILLARS).card, 0.01)
    fleet.leader_prefix_enabled = True
    fleet.leader_prefix_s = 0.7
    frame = type("F", (), {"depth": np.zeros((2, 2)), "stamp": 0.0})()
    head = _moving_head(fleet)
    goal = np.array([15.0, 0.0, 1.5])
    seen = {}

    def fake_options(head_p, t_start, goal_p, depth, yaw, short=False):
        seen["short"] = short
        motion = fleet._gentle_hover(head_p, t_start)
        return [motion], "OK"

    fleet._yopo_options = fake_options
    occ = fleet._leader_proposal(fleet.nodes["uav0"], 1.0, head, goal, frame, 0.0)
    assert seen["short"] is True
    assert occ is not None
    assert fleet.code("uav0") == "LEADER_PREFIX"

    fleet._codes["uav0"].clear()
    fleet._yopo_options = lambda *_a, **_k: ([], "YOPO_FAIL")
    occ = fleet._leader_proposal(fleet.nodes["uav0"], 1.0, head, goal, frame, 0.0)
    assert occ is not None
    assert fleet.code("uav0") == "LEADER_HOVER"

    fleet._codes["uav0"].clear()
    fleet._gentle_hover = lambda *_a, **_k: None
    occ = fleet._leader_proposal(fleet.nodes["uav0"], 1.0, head, goal, frame, 0.0)
    assert occ is None
    assert fleet.code("uav0") == "LEADER_HOLD"


def test_switch_on_marks_upstream_deviation_and_switch_off_does_not():
    loaded = load_card(SIX)
    off = run(loaded, RunConfig(seed=0, policy="crossing", duration_s=0.4, shadow=False))
    assert off.result["upstream_deviation"] is False
    assert "leader_prefix" not in off.result
    assert "leader_prefix" not in off.result["crossing"]

    card = copy.deepcopy(loaded.card)
    card["task"]["progress_gate"] = True
    card["task"]["leader_prefix"] = True
    card["task"]["leader_prefix_s"] = 0.7
    opened = SceneCard(loaded.path, card, loaded.obstacles)
    on = run(opened, RunConfig(seed=0, policy="crossing", duration_s=0.4, shadow=False))
    assert on.result["upstream_deviation"] is True
    assert on.result["leader_prefix"] is True
    assert on.result["leader_prefix_s"] == pytest.approx(0.7)
    assert on.result["crossing"]["leader_prefix"] is True
    assert on.result["crossing"]["leader_prefix_s"] == pytest.approx(0.7)
    assert on.result["crossing"]["progress_gate"] is True
    disk = json.loads(SIX.read_text())
    assert "leader_prefix" not in disk["task"]
    assert disk["task"]["progress_gate"] is False


def test_switch_off_crossing6_forest_seed0_matches_main():
    """关时与当前 main 的六机林带 seed 0 逐位一致。比较时去掉墙钟和仓库 SHA。"""
    pre = json.loads(FIXTURE.read_text())
    assert pre["scene"] == "crossing-6-forest"
    assert pre["seed"] == 0
    assert pre["crossing"]["progress_gate"] is False
    assert "leader_prefix" not in pre
    assert "leader_prefix" not in pre["crossing"]
    out = run(load_card(SIX), RunConfig(seed=0, policy="crossing", shadow=True))
    assert "leader_prefix" not in out.result
    assert "leader_prefix" not in out.result["crossing"]
    assert out.result["upstream_deviation"] is False
    assert _canon(_strip(out.result)) == _strip(pre)
