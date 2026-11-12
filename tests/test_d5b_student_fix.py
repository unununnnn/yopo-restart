"""票 D5b：学生接口选项。默认就是 D5 行为；新选项只在显式传参时生效。"""

from __future__ import annotations

import json
import shutil

import numpy as np
import pytest

from yopo_l0.policy import (
    POLICIES,
    Observation,
    PolicyContext,
    StudentPolicy,
    _EgoLookaheadPlan,
    _TrajPlan,
    student_knot_velocities,
    student_vertical_hold,
)
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import load_card

from conftest import ROOT

PILLAR = ROOT / "scenes/g2-pillar.json"


def _weights(tmp_path):
    from yopo_l0.student import MultiHeadStudent, StudentConfig, save_checkpoint
    import torch

    torch.manual_seed(0)
    path, _ = save_checkpoint(MultiHeadStudent(StudentConfig(enabled=True)), tmp_path / "student.pt")
    return path


def _strip(r):
    out = {k: v for k, v in r.items() if k not in WALL_FIELDS}
    out.pop("student_infer", None)
    return out


def test_defaults_are_d5_behaviour():
    d = StudentPolicy.DEFAULTS
    assert d["knots"] == "network"
    assert d["yaw_source"] == "calculate_yaw"
    assert d["accel_source"] == "state"
    assert d["vertical"] == "network"
    assert POLICIES["student"] is StudentPolicy


@pytest.mark.parametrize("key", ["knots", "yaw_source", "accel_source", "vertical"])
def test_unknown_option_is_rejected(tmp_path, key):
    ctx = PolicyContext("uav0", 0.01, {"student": {"weights": str(_weights(tmp_path)), key: "nope"}})
    with pytest.raises(ValueError, match=key):
        StudentPolicy(ctx)


def test_knot_velocities_are_consistent_with_positions():
    dt = 0.2
    p0 = np.array([1.0, 2.0, 1.5])
    v = np.array([1.2, -0.4, 0.0])
    p_w = p0 + np.outer(dt * np.arange(1, 9), v)
    out = student_knot_velocities(p0, p_w, dt)
    assert out.shape == (8, 3)
    np.testing.assert_allclose(out, np.tile(v, (8, 1)), atol=1e-12)


def test_ego_lookahead_yaw_turns_at_most_pi_per_second_and_holds_when_still():
    from yopo_swarm_core.plan import hold_traj, quintic
    from yopo_swarm_core.trajectory import PiecewisePolynomial

    head = np.array([[0.0, 0.0, 1.5], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]])
    end = np.array([[0.0, 3.0, 1.5], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]])
    traj = PiecewisePolynomial(0.0, [quintic(head, end, 3.0)])
    plan = _EgoLookaheadPlan(traj, np.array([0.0, 3.0, 1.5]), 0.0, 0.01)
    _, _, _, y1 = plan.sample(0.0)
    assert y1 == pytest.approx(np.pi * 0.01)
    for k in range(1, 100):
        plan.sample(0.01 * k)
    assert plan.yaw == pytest.approx(np.pi / 2, abs=1e-6)
    still = _EgoLookaheadPlan(hold_traj(np.array([0.0, 0.0, 1.5]), 0.0, 1e-3), np.array([5.0, 0.0, 1.5]), 0.3, 0.01)
    for k in range(10):
        still.sample(0.01 * k)
    assert still.yaw == 0.3


def test_new_options_plan_with_lookahead_and_reference_accel(tmp_path):
    from yopo_l0.dynamics import State
    from yopo_l0.sensor import CameraConfig, DepthCamera

    card = load_card(PILLAR)
    cam = DepthCamera(CameraConfig.from_card(card.card["camera"]), card.obstacles, np.random.default_rng(0))
    params = {"student": {"weights": str(_weights(tmp_path)), "knots": "position", "yaw_source": "ego_lookahead",
                          "accel_source": "reference"}}
    pol = StudentPolicy(PolicyContext("uav0", 0.01, params))
    goal = np.array([5.0, 0.0, 1.5])
    s = State(0.0, np.array([0.0, 0.0, 1.5]), np.zeros(3), np.array([9.0, 9.0, 9.0]), 0.0)
    first = pol.plan(Observation(cam.capture(s.p, s.yaw, 0.0), s, goal, 0.0))
    assert isinstance(first, _EgoLookaheadPlan)
    assert pol.last_code == "OK"
    s2 = State(0.1, np.array([0.01, 0.0, 1.5]), np.array([0.1, 0.0, 0.0]), np.array([9.0, 9.0, 9.0]), 0.0)
    second = pol.plan(Observation(cam.capture(s2.p, s2.yaw, 0.1), s2, goal, 0.1))
    assert isinstance(second, _EgoLookaheadPlan)
    p, v, a = second.pva(0.3)
    assert np.all(np.isfinite(p)) and np.all(np.isfinite(v)) and np.all(np.isfinite(a))


def test_explicit_d5_options_are_bitwise_same_as_defaults(tmp_path, monkeypatch):
    weights = _weights(tmp_path)
    monkeypatch.setenv("YOPO_STUDENT_WEIGHTS", str(weights))
    raw = json.loads(PILLAR.read_text(encoding="utf-8"))
    shutil.copy(PILLAR.parent / raw["geometry"], tmp_path / raw["geometry"])
    explicit = dict(raw)
    explicit["policy_params"] = {**raw.get("policy_params", {}),
                                 "student": {"knots": "network", "yaw_source": "calculate_yaw", "accel_source": "state",
                                             "vertical": "network"}}
    (tmp_path / "explicit.json").write_text(json.dumps(explicit), encoding="utf-8")
    a = run(load_card(PILLAR), RunConfig(seed=0, policy="student", duration_s=0.5, shadow=False))
    b = run(load_card(tmp_path / "explicit.json"), RunConfig(seed=0, policy="student", duration_s=0.5, shadow=False))
    ra, rb = _strip(a.result), _strip(b.result)
    ra.pop("scene_card_hash", None)
    rb.pop("scene_card_hash", None)
    ra.pop("scene", None)
    rb.pop("scene", None)
    assert ra == rb
    assert a.ticks == b.ticks


def test_new_options_change_the_flight(tmp_path, monkeypatch):
    weights = _weights(tmp_path)
    monkeypatch.setenv("YOPO_STUDENT_WEIGHTS", str(weights))
    raw = json.loads(PILLAR.read_text(encoding="utf-8"))
    shutil.copy(PILLAR.parent / raw["geometry"], tmp_path / raw["geometry"])
    fixed = dict(raw)
    fixed["policy_params"] = {**raw.get("policy_params", {}),
                              "student": {"knots": "position", "yaw_source": "ego_lookahead", "accel_source": "reference"}}
    (tmp_path / "fixed.json").write_text(json.dumps(fixed), encoding="utf-8")
    a = run(load_card(PILLAR), RunConfig(seed=0, policy="student", duration_s=0.5, shadow=False))
    b = run(load_card(tmp_path / "fixed.json"), RunConfig(seed=0, policy="student", duration_s=0.5, shadow=False))
    assert a.ticks != b.ticks


def test_vertical_hold_approaches_goal_altitude():
    z, vz = student_vertical_hold(1.0, 1.5, 8, 0.2, 1.0)
    t = 0.2 * np.arange(1, 9)
    np.testing.assert_allclose(z, 1.5 - 0.5 * np.exp(-t), atol=1e-12)
    np.testing.assert_allclose(vz, 0.5 * np.exp(-t), atol=1e-12)
    assert np.all(np.diff(z) > 0) and z[-1] < 1.5
    flat, vflat = student_vertical_hold(1.5, 1.5, 8, 0.2, 1.0)
    assert np.all(flat == 1.5) and np.all(vflat == 0.0)
    with pytest.raises(ValueError):
        student_vertical_hold(1.0, 1.5, 8, 0.2, 0.0)


@pytest.mark.parametrize("knots", ["network", "position"])
def test_vertical_hold_ignores_network_altitude(tmp_path, knots):
    from yopo_l0.dynamics import State
    from yopo_l0.sensor import CameraConfig, DepthCamera

    card = load_card(PILLAR)
    cam = DepthCamera(CameraConfig.from_card(card.card["camera"]), card.obstacles, np.random.default_rng(0))
    params = {"student": {"weights": str(_weights(tmp_path)), "knots": knots, "vertical": "hold_goal_z"}}
    pol = StudentPolicy(PolicyContext("uav0", 0.01, params))
    goal = np.array([5.0, 0.0, 1.5])
    s = State(0.0, np.array([0.0, 0.0, 1.5]), np.zeros(3), np.zeros(3), 0.0)
    plan = pol.plan(Observation(cam.capture(s.p, s.yaw, 0.0), s, goal, 0.0))
    for t in np.linspace(0.0, 1.6, 17):
        p, v, _ = plan.pva(t)
        assert p[2] == pytest.approx(1.5, abs=1e-9)
        assert v[2] == pytest.approx(0.0, abs=1e-9)
