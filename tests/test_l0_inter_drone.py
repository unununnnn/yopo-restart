"""E-A1：机间项默认关。关掉时，已有场景卡同 seed 的结果与打开这个开关之前逐字节相同。

比较的是去掉墙钟和 commit 之后的结果记录，加上全部 tick。时长 0.5 s、seed 0。
哈希在改打分之前生成，见 tests/fixtures/ea1_switch_off_sha256.json。
"""

import json
from pathlib import Path

import numpy as np
import pytest

from yopo_l0.inter_drone import (
    InterDroneConfig,
    as_sample_array,
    clearance_m,
    committed_neighbor_samples,
    inter_drone_config,
    inter_pick,
    sample_motion,
)
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import load_card

from conftest import ROOT

FIXTURE = ROOT / "tests/fixtures/ea1_switch_off_sha256.json"
DURATION_S = 0.5
_STRIP = set(WALL_FIELDS) | {"commit"}


def _canonical(out) -> str:
    import hashlib

    result = {k: v for k, v in out.result.items() if k not in _STRIP}
    blob = json.dumps(
        {"result": result, "ticks": out.ticks},
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    )
    return hashlib.sha256(blob.encode()).hexdigest()


def test_switch_off_matches_prechange_bytes():
    rows = json.loads(FIXTURE.read_text())
    assert len(rows) >= 17
    for row in rows:
        card = load_card(ROOT / "scenes" / row["card"])
        out = run(card, RunConfig(seed=0, duration_s=DURATION_S, policy=row["policy"], shadow=True))
        assert "inter_drone_filter" not in out.result
        assert _canonical(out) == row["sha256"], row["card"] + "@" + row["policy"]


def test_explicit_false_matches_absent_key(tmp_path):
    """场景卡写上 inter_drone_filter=false，飞行记录与不写这个键相同（场景卡哈希除外）。"""
    src = json.loads((ROOT / "scenes/open-3drone.json").read_text())
    src["policy_params"] = {
        "upstream": {
            "inter_drone_filter": False,
            "inter_drone_threshold_m": 1.0,
            "inter_drone_penalty": 0.0,
        }
    }
    (tmp_path / "open-area.json").write_text((ROOT / "scenes/open-area.json").read_text())
    path = tmp_path / "open-3drone.json"
    path.write_text(json.dumps(src))
    plain = run(load_card(ROOT / "scenes/open-3drone.json"), RunConfig(seed=0, policy="upstream", duration_s=0.3, shadow=False))
    explicit = run(load_card(path), RunConfig(seed=0, policy="upstream", duration_s=0.3, shadow=False))
    assert "inter_drone_filter" not in plain.result
    assert "inter_drone_filter" not in explicit.result
    assert plain.ticks == explicit.ticks
    a = {k: v for k, v in plain.result.items() if k not in _STRIP | {"scene_card_hash"}}
    b = {k: v for k, v in explicit.result.items() if k not in _STRIP | {"scene_card_hash"}}
    assert a == b
    assert plain.result["scene_card_hash"] != explicit.result["scene_card_hash"]


def test_config_defaults_and_requires_threshold():
    assert inter_drone_config({}) is None
    assert inter_drone_config({"upstream": {}}) is None
    assert inter_drone_config({"upstream": {"inter_drone_filter": False}}) is None
    with pytest.raises(ValueError):
        inter_drone_config({"upstream": {"inter_drone_filter": True}})
    with pytest.raises(ValueError):
        inter_drone_config({"upstream": {"inter_drone_filter": "yes", "inter_drone_threshold_m": 1, "inter_drone_penalty": 0}})
    cfg = inter_drone_config({
        "upstream": {"inter_drone_filter": True, "inter_drone_threshold_m": 0.7, "inter_drone_penalty": 0}
    })
    assert cfg == InterDroneConfig(0.7, 0.0)


def test_penalty_zero_filters_and_empty_uses_no_new_action():
    score = np.array([0.2, 5.0, 4.0, 1.0])
    admitted = np.array([True, True, True, False])
    clear = np.array([3.0, 0.2, 0.5, 0.0])
    cfg = InterDroneConfig(threshold_m=0.7, penalty=0.0)
    pick = inter_pick(score, admitted, clear, cfg)
    assert pick.kind == "adjusted" and pick.action == 0 and pick.order == (0,)
    assert pick.filtered == 2 and pick.penalized == 0
    # 全被滤掉：没有新的动作索引，调用方走原来的无候选悬停。
    clear2 = np.array([0.1, 0.2, 0.3, 9.0])
    empty = inter_pick(score, admitted, clear2, cfg)
    assert empty.kind == "empty" and empty.action is None and empty.empty
    assert empty.filtered == 3


def test_positive_penalty_keeps_candidate_and_can_change_argmax():
    score = np.array([1.0, 3.0])
    admitted = np.array([True, True])
    clear = np.array([2.0, 0.0])
    # 3 - penalty * (1-0)/1 = 3 - 2.5 = 0.5，低于另一条的 1.0。
    cfg = InterDroneConfig(threshold_m=1.0, penalty=2.5)
    pick = inter_pick(score, admitted, clear, cfg)
    assert pick.kind == "adjusted" and pick.action == 0
    assert pick.filtered == 0 and pick.penalized == 1
    assert pick.order == (0, 1)
    # 没有人进入阈值：调用方必须继续用原来的 argmax，不能重排。
    far = inter_pick(score, admitted, np.array([2.0, 1.0]), cfg)
    assert far.kind == "unchanged" and far.action is None


def test_clearance_is_same_time_and_does_not_extrapolate():
    cand = np.array([[0.0, 0.0, 0.0, 1.0], [0.1, 1.0, 0.0, 1.0]])
    nb = np.array([[0.0, 0.0, 0.5, 1.0], [0.1, 1.0, 0.0, 1.0]])
    assert clearance_m(cand, [nb]) == pytest.approx(0.0)
    # 邻机采样还没覆盖 0.1 s 之后，不能把端点外推成冲突。
    early = np.array([[0.0, 0.0, 0.0, 1.0]])
    assert clearance_m(cand, [early]) == pytest.approx(0.0)
    later_only = np.array([[1.0, 0.0, 0.0, 1.0]])
    assert clearance_m(cand, [later_only]) == float("inf")


def test_neighbor_samples_use_latest_commit_only():
    class _Motion:
        def __init__(self, point):
            self.t_start = 0.0
            self.t_end = 0.5
            self.point = np.asarray(point, dtype=float)

        def evaluate(self, t, order=0):
            return self.point

    class _Rec:
        def __init__(self, point):
            self.occupancy = type("O", (), {})()
            self.occupancy.plan = type("P", (), {"motion": _Motion(point)})()

    class _Peer:
        def __init__(self, points):
            self.plans = [_Rec(p) for p in points]

    class _Node:
        def __init__(self):
            self.peers = {"uav1": _Peer([[0.0, 0.0, 1.0], [8.0, 0.0, 1.0]])}

    samples = committed_neighbor_samples(_Node(), 0.0)
    assert len(samples) == 1
    assert samples[0][0, 1] == pytest.approx(8.0)
    assert sample_motion(_Motion([1.0, 2.0, 3.0]), 0.0).shape[1] == 4
    assert as_sample_array([[0.0, 1.0, 2.0]]) is None
    assert as_sample_array([[0.0, 1.0, 2.0, 3.0]]) is not None


def test_switch_on_broadcasts_samples_and_records_fallback_field(tmp_path):
    src = json.loads((ROOT / "scenes/open-3drone.json").read_text())
    src["name"] = "open-3drone-inter"
    src["policy_params"] = {
        "upstream": {
            "inter_drone_filter": True,
            "inter_drone_threshold_m": 0.7,
            "inter_drone_penalty": 0.0,
        }
    }
    (tmp_path / "open-area.json").write_text((ROOT / "scenes/open-area.json").read_text())
    path = tmp_path / "card.json"
    path.write_text(json.dumps(src))
    seen = []
    from yopo_l0.bus import MessageBus

    orig = MessageBus.publish

    def publish(self, now, *, src, dest, topic, payload):
        seen.append(payload)
        return orig(self, now, src=src, dest=dest, topic=topic, payload=payload)

    MessageBus.publish = publish
    try:
        out = run(load_card(path), RunConfig(seed=0, policy="upstream", duration_s=0.25, shadow=False))
    finally:
        MessageBus.publish = orig
    peer = [p for p in seen if p.get("kind") == "peer_plan"]
    assert peer and all("traj_samples" in p for p in peer)
    assert len(peer[0]["traj_samples"]) == 200
    block = out.result["inter_drone_filter"]
    assert block["enabled"] is True
    assert block["threshold_m"] == pytest.approx(0.7)
    assert block["penalty"] == pytest.approx(0.0)
    assert block["fallback"] == "CORRIDOR_EMPTY_HOVER"
    assert block["empty_fallbacks"] >= 0
    assert "safe_radius_m" in out.result and out.result["safe_radius_m"] == pytest.approx(0.05)
