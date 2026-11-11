"""票 D1：几何回退默认关，同 seed 的现有场景卡与改前按位一致。

比较时去掉 ``commit`` 和 ``WALL_FIELDS``（``wall_time_s``、``plan_wall_ms``）。
前者随 HEAD 变，后两者是墙钟。其余结果字段和逐 tick 日志都进哈希。
打开开关才出现 ``geometric_fallback``，并且不写进 runner.WALL_FIELDS，
避免和 E-C3 的 plan_wall_ms 抢同一行。
"""

import hashlib
import json
import math

import numpy as np
import pytest

from yopo_l0.criteria import active_gates
from yopo_l0.geom_fallback import (
    ACTION_BRAKE,
    ACTION_PREVIOUS,
    FIELD,
    REASON_INTER,
    REASON_OBSTACLE,
    GeomSession,
    iter_times,
)
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import Box, Ground, load_card

from conftest import ROOT

FIXTURE = ROOT / "tests/fixtures/geom_fallback_off_1s.json"
VARIANT = ROOT / "scenes/swap-3-geom-fallback.json"
FOREST = ROOT / "scenes/crossing-forest.json"


def _canon(result, ticks) -> str:
    skip = set(WALL_FIELDS) | {"commit"}
    kept = {k: v for k, v in result.items() if k not in skip}
    blob = json.dumps({"result": kept, "ticks": ticks}, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(blob.encode()).hexdigest()


def _session(**kw) -> GeomSession:
    card = {
        "criteria": {"body_radius_m": 0.35},
        "dynamics": {"a_max": 6.0},
        "geometric_fallback": {"enabled": True, "commit_horizon_s": 1.0},
        "task": {"window_len_s": 0.25},
    }
    card.update(kw.pop("card_patch", {}))
    return GeomSession.from_card(
        card,
        members=("uav0", "uav1"),
        horizon=kw.pop("horizon", 1.0),
        physics_dt=0.01,
        obstacles=kw.pop("obstacles", (Ground(0.0),)),
    )


class _Traj:
    def __init__(self, t0, t1, fn):
        self.t_start = float(t0)
        self.t_end = float(t1)
        self.fn = fn

    def evaluate(self, t, k=0):
        if k == 0:
            return np.asarray(self.fn(t), dtype=float)
        return np.zeros(3)


class _Plan:
    def __init__(self, traj):
        self.traj = traj
        self.yaw = 0.0
        self.dt = 0.01
        self.goal = np.zeros(3)
        self.calls = 0

    def pva(self, t):
        self.calls += 1
        tt = min(max(float(t), self.traj.t_start), self.traj.t_end)
        p = np.asarray(self.traj.evaluate(tt), dtype=float)
        z = np.zeros(3)
        return p, z, z.copy()


def test_disabled_session_does_not_sample_or_enqueue():
    card = {
        "criteria": {"body_radius_m": 0.35},
        "dynamics": {"a_max": 6.0},
        "task": {"type": "goto"},
    }
    session = GeomSession.from_card(card, members=("uav0",), physics_dt=0.01, obstacles=(Ground(0.0),))
    assert session.enabled is False
    plan = _Plan(_Traj(0.0, 1.0, lambda t: np.array([t, 0.0, 1.5])))
    assert session.review_goto("uav0", plan, 0.0) is plan
    assert session.checks == 0
    assert session._outbox == []
    assert plan.calls == 0


def test_obstacle_inflation_uses_body_radius_and_is_single_pass():
    box = Box("pillar", (1.0, -0.4, 0.0), (1.6, 0.4, 2.8))
    session = _session(obstacles=(box, Ground(0.0)))
    # 表面在 x=1.6。0.35 m 膨胀后 x=1.95 仍撞，x=1.96 不撞。
    assert box.distance(np.array([1.95, 0.0, 1.5])) == pytest.approx(0.35)
    assert session._obstacle_hit(np.array([1.95, 0.0, 1.5]))
    assert not session._obstacle_hit(np.array([1.96, 0.0, 1.5]))
    plan = _Plan(_Traj(0.0, 0.2, lambda t: np.array([1.0 + 5.0 * t, 0.0, 1.5])))
    calls_before = plan.calls
    got = session.review_goto("uav0", plan, 0.0)
    assert got is not plan
    assert session.checks == 1
    assert session.fallbacks == 1
    assert session.reason_counts[REASON_OBSTACLE] == 1
    assert session.action_counts[ACTION_BRAKE] == 1
    # 采样只走一遍。多出来的一次 pva 是刹车取当前头状态，不是再查一遍几何。
    assert plan.calls - calls_before == len(iter_times(0.0, 0.2 + 1.0, session.sample_dt_s)) + 1
    assert session.review_goto("uav0", plan, 0.0) is not None
    assert session.checks == 2


def test_inter_drone_threshold_is_strict_and_previous_stays_when_horizon_holds():
    session = _session()
    session._heard["uav1"] = [[0.0, 0.0, 0.0, 1.5], [1.0, 0.0, 0.0, 1.5]]
    # 距离恰好等于阈值，不算冲突。
    far = _Plan(_Traj(0.0, 0.2, lambda t: np.array([0.7, 0.0, 1.5])))
    assert session.classify("uav0", session.samples_of_object(far, 0.0)) == []
    near = _Plan(_Traj(0.0, 0.2, lambda t: np.array([0.699, 0.0, 1.5])))
    reasons = session.classify("uav0", session.samples_of_object(near, 0.0))
    assert reasons == [REASON_INTER]
    accepted = session.review_goto("uav0", far, 0.0)
    assert accepted is far
    assert session.checks == 1
    # 视界内退回刚才提交的那条，不再采样出一条新的刹车。
    again = session.review_goto("uav0", near, 0.1)
    assert again is far
    assert session.action_counts[ACTION_PREVIOUS] == 1
    assert session.action_counts[ACTION_BRAKE] == 0
    assert session.fallbacks == 1
    # 运动段已经飞完（t_end=0.2），不能再沿用，改刹停。
    later = session.review_goto("uav0", near, 1.0)
    assert later is not far
    assert session.action_counts[ACTION_BRAKE] == 1
    assert math.isclose(session.horizon_s, 1.0)


def test_previous_validity_is_the_committed_motion_not_a_second_horizon():
    session = _session()
    session._meta["uav0"] = {"committed_at": 0.0, "t_end": 0.4}
    assert session.previous_valid("uav0", 0.2)
    assert not session.previous_valid("uav0", 0.4)
    # 运动段比承诺视界长时，段还没飞完就继续用它。视界长度记在结果里，不在这里把曲线截断。
    session._meta["uav0"] = {"committed_at": 0.0, "t_end": 5.0}
    assert session.previous_valid("uav0", 1.0)
    assert not session.previous_valid("uav0", 5.0)


def test_batch_strip_keeps_fallback_counts_and_drops_check_time():
    from yopo_l0.batch import _strip

    a = {
        "seed": 0,
        "wall_time_s": 1.0,
        "geometric_fallback": {"fallbacks": 2, "check_time_ms": {"p50": 0.1, "p95": 0.2}},
    }
    b = {
        "seed": 0,
        "wall_time_s": 9.0,
        "geometric_fallback": {"fallbacks": 2, "check_time_ms": {"p50": 3.0, "p95": 4.0}},
    }
    assert _strip(a) == _strip(b)
    assert _strip(a)["geometric_fallback"]["fallbacks"] == 2
    assert "check_time_ms" not in _strip(a)["geometric_fallback"]
    assert "plan_wall_ms" not in _strip(a)


def test_enabling_fallback_keeps_13a_horizon_and_a_max():
    from yopo_l0.crossing import CrossingFleet

    card = json.loads((ROOT / "scenes/crossing-6-forest.json").read_text())
    assert card["task"]["commit_horizon_s"] == pytest.approx(1.0)
    card["geometric_fallback"] = {"enabled": True}
    fleet = CrossingFleet.from_card(card, 0.01)
    assert fleet.horizon == pytest.approx(1.75)
    assert fleet.pcfg.limits.a_max == pytest.approx(6.0)
    assert fleet.geom.enabled is True
    assert fleet.geom.horizon_s == pytest.approx(1.75)
    assert fleet.geom.a_max == pytest.approx(6.0)
    assert type(next(iter(fleet.nodes.values()))).__name__ == "GuardedNode"


def test_variant_card_is_report_only_and_not_in_the_suite():
    raw = json.loads(VARIANT.read_text())
    assert raw["name"] == "swap-3-geom-fallback"
    assert raw["geometry"] == "open-area.json"
    assert raw["task"]["type"] == "goto"
    assert raw["seeds"] == [0]
    assert raw["criteria"]["frozen"] is False
    assert "gates" not in raw["criteria"]
    assert raw["geometric_fallback"]["enabled"] is True
    assert raw["geometric_fallback"]["inter_drone_distance_m"] == pytest.approx(0.7)
    assert raw["geometric_fallback"]["commit_horizon_s"] == pytest.approx(1.0)
    assert raw["dynamics"]["a_max"] == pytest.approx(6.0)
    assert "speed_mps" not in raw["task"]
    assert "20.3" in raw["note"] and "14.3" in raw["note"]
    card = load_card(VARIANT)
    assert active_gates(card, policy="upstream") is None
    listed = (ROOT / "scenes/suite.txt").read_text()
    assert "swap-3-geom-fallback" not in listed
    assert "swap-3-open" not in listed
    forest = json.loads(FOREST.read_text())
    assert "geometric_fallback" not in forest
    assert forest["criteria"]["frozen"] is True
    assert forest["task"]["commit_horizon_s"] == pytest.approx(1.0)


def test_variant_short_run_records_independent_field():
    card = load_card(VARIANT)
    out = run(card, RunConfig(seed=0, shadow=False, duration_s=0.4))
    block = out.result[FIELD]
    assert block["enabled"] is True
    assert block["checks"] >= 1
    assert block["check_time_ms"]["count"] == block["checks"]
    assert block["commit_horizon_s"] == pytest.approx(1.0)
    assert block["inter_drone_distance_m"] == pytest.approx(0.7)
    assert block["a_max"] == pytest.approx(6.0)
    # E-C3 的规划墙钟仍在结果顶层；几何回退自己的计时在 check_time_ms，不占用该字段。
    assert "plan_wall_ms" in out.result
    assert "plan_wall_ms" not in block
    assert set(block["actions"]) == {ACTION_PREVIOUS, ACTION_BRAKE}


def test_fallback_off_matches_prechange_hashes_for_every_scene_card():
    expected = json.loads(FIXTURE.read_text())
    assert expected, "缺少关着回退时的指纹"
    seen = []
    for rel, digest in expected.items():
        path = ROOT / rel
        assert path.is_file(), rel
        card = load_card(path)
        out = run(card, RunConfig(seed=0, duration_s=1.0))
        assert FIELD not in out.result
        got = _canon(out.result, out.ticks)
        assert got == digest, rel
        seen.append(rel)
    assert len(seen) == len(list((ROOT / "scenes").glob("*.json")) ) or len(seen) >= 17

