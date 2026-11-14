"""票 E-PG2：超前余量从模块常量改成场景卡字段。缺省仍是 1.0。"""

import copy
import json
from pathlib import Path

import numpy as np
import pytest

from yopo_l0.crossing import CrossingFleet, _LEADER_MARGIN_M
from yopo_l0.progress import read_leader_margin_m
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import load_card

from conftest import ROOT

SIX = ROOT / "scenes/crossing-6-forest.json"
SPARSE = ROOT / "scenes/crossing-6-forest-sparse.json"
FIXTURE = Path(__file__).parent / "fixtures" / "pg2_prechange_crossing6_forest_seed0_result.json"
# 仓库 SHA 随这次提交变化，不在轨迹里。墙钟字段同样不比。
_STRIP = set(WALL_FIELDS) | {"commit"}


def _canon(obj):
    return json.loads(json.dumps(obj))


def _strip(result):
    return {k: v for k, v in result.items() if k not in _STRIP}


def _place(fleet, drone_id, arc):
    slot = fleet.slots[drone_id]
    origin_x = float(fleet.rail.origin[0])
    fleet.note_progress(drone_id, np.array([origin_x + float(slot[0]) + arc, 0.0, 1.5]))


def test_scene_cards_still_omit_the_margin_and_keep_the_gate_off():
    for path in (SIX, SPARSE):
        raw = json.loads(path.read_text())
        assert raw["task"]["progress_gate"] is False
        assert "leader_margin_m" not in raw["task"]


def test_missing_margin_is_one_metre_and_three_is_read():
    assert _LEADER_MARGIN_M == 1.0
    assert read_leader_margin_m({}) == 1.0
    assert read_leader_margin_m({"progress_gate": False}) == 1.0
    assert read_leader_margin_m({"leader_margin_m": 3}) == 3.0
    assert read_leader_margin_m({"leader_margin_m": 3.0}) == 3.0
    assert read_leader_margin_m({"leader_margin_m": 0}) == 0.0
    assert read_leader_margin_m({"leader_margin_m": 1}) == 1.0

    card = copy.deepcopy(load_card(SIX).card)
    assert "leader_margin_m" not in card["task"]
    fleet = CrossingFleet.from_card(card, 0.01)
    assert fleet.leader_margin_m == 1.0
    assert fleet.progress_gate is False
    # 门控关时摘要仍不写余量，和改前的 None 相同。
    assert fleet.summary(0, 6)["leader_margin_m"] is None

    opened = copy.deepcopy(card)
    opened["task"]["progress_gate"] = True
    opened["task"]["leader_margin_m"] = 3.0
    opened["task"]["progress_percentile"] = 20
    wide = CrossingFleet.from_card(opened, 0.01)
    assert wide.progress_gate is True
    assert wide.progress_percentile == 20.0
    assert wide.leader_margin_m == 3.0
    assert wide.summary(0, 6)["leader_margin_m"] == 3.0

    for drone_id in fleet.members:
        _place(fleet, drone_id, 0.0)
        _place(wide, drone_id, 0.0)
    _place(fleet, "uav0", 1.5)
    _place(wide, "uav0", 1.5)
    fleet.progress_gate = True
    assert fleet.gate_s() == pytest.approx(0.0)
    assert wide.gate_s() == pytest.approx(0.0)
    head = np.zeros(3)
    assert fleet._ahead_of_gate("uav0", head) is True
    assert wide._ahead_of_gate("uav0", head) is False


def test_explicit_default_keeps_the_task_hash_and_a_wider_margin_changes_it():
    base = copy.deepcopy(load_card(SIX).card)
    same = copy.deepcopy(base)
    same["task"]["leader_margin_m"] = 1.0
    plain = CrossingFleet.from_card(base, 0.01)
    explicit = CrossingFleet.from_card(same, 0.01)
    assert plain.leader_margin_m == explicit.leader_margin_m == 1.0
    assert plain.cfg.task_hash == explicit.cfg.task_hash

    opened = copy.deepcopy(base)
    opened["task"]["progress_gate"] = True
    wider = copy.deepcopy(opened)
    wider["task"]["leader_margin_m"] = 3.0
    assert CrossingFleet.from_card(opened, 0.01).cfg.task_hash != CrossingFleet.from_card(wider, 0.01).cfg.task_hash


def test_illegal_leader_margin_is_rejected():
    card = copy.deepcopy(load_card(SIX).card)
    for raw in ("3", -0.1, True, False, None, float("nan"), float("inf"), float("-inf")):
        with pytest.raises(ValueError, match="leader_margin_m"):
            read_leader_margin_m({"leader_margin_m": raw})
        bad = copy.deepcopy(card)
        bad["task"]["leader_margin_m"] = raw
        with pytest.raises(ValueError, match="leader_margin_m"):
            CrossingFleet.from_card(bad, 0.01)


def test_default_crossing6_forest_seed0_matches_prechange():
    """缺省卡、seed 0 的结果与改余量开关之前相同。比较时去掉墙钟和仓库 SHA。"""
    pre = json.loads(FIXTURE.read_text())
    assert pre["scene"] == "crossing-6-forest"
    assert pre["seed"] == 0
    assert pre["crossing"]["progress_gate"] is False
    assert pre["crossing"]["leader_margin_m"] is None
    out = run(load_card(SIX), RunConfig(seed=0, policy="crossing", shadow=True))
    assert _canon(_strip(out.result)) == _strip(pre)
