"""票 E-B4b：无解时从参考头刹停的开关。默认关；关时与上游原地悬停相同，开时标偏离上游。"""

import json

import numpy as np
import pytest

import yopo_swarm_core.candidates as candidates
from yopo_l0.dynamics import State
from yopo_l0.geom_fallback import GeomSession, brake_from_head
from yopo_l0.policy import Observation, PolicyContext, UpstreamPolicy, _TrajPlan, read_empty_brake
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import SceneCard, load_card, parse_geometry
from yopo_l0.sensor import CameraConfig, DepthCamera
from yopo_swarm_core.plan import PiecewisePolynomial, quintic

from conftest import ROOT


def test_switch_defaults_off_and_is_strict_bool():
    assert read_empty_brake(None) is False
    assert read_empty_brake({}) is False
    assert read_empty_brake({"upstream": {}}) is False
    assert read_empty_brake({"upstream": {"empty_brake": True}}) is True
    for bad in ("true", 1, None):
        with pytest.raises(ValueError):
            read_empty_brake({"upstream": {"empty_brake": bad}})


def test_no_committed_card_turns_it_on():
    for path in sorted((ROOT / "scenes").glob("*.json")):
        assert "empty_brake" not in path.read_text(encoding="utf-8"), path.name


def test_shared_function_matches_d1_brake():
    head = np.array([[1.0, 2.0, 1.5], [4.0, -1.0, 0.0], [0.5, 0.0, 0.0]])
    card = json.loads((ROOT / "scenes/swap-8-open.json").read_text())
    sess = GeomSession.from_card(card, members=("uav0",), physics_dt=0.01)
    d1 = sess._brake_from_head(head, 0.0, 3.0).traj
    ours = brake_from_head(head, 3.0, 6.0)
    assert ours is not None
    p, v, _a = (np.asarray(ours.evaluate(3.0, k)) for k in range(3))
    np.testing.assert_allclose(p, head[0], atol=1e-9)
    np.testing.assert_allclose(v, head[1], atol=1e-9)
    assert np.linalg.norm(ours.evaluate(ours.t_end, 1)) < 1e-9
    assert d1 is not None
    for t in np.linspace(3.0, ours.t_end, 50):
        np.testing.assert_array_equal(np.asarray(d1.evaluate(float(t), 0)), np.asarray(ours.evaluate(float(t), 0)))
    assert brake_from_head(np.array([[0.0, 0.0, 1.0], [0.0] * 3, [0.0] * 3]), 0.0, 6.0) is None


def _policy_and_obs(on: bool, monkeypatch):
    card = load_card(ROOT / "scenes/swap-8-open.json")
    params = {"upstream": {"empty_brake": True}} if on else {}
    pol = UpstreamPolicy(PolicyContext("uav0", 0.01, params, obstacles=card.obstacles, a_max=6.0))
    head = np.array([[0.0, 0.0, 1.5], [4.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    end = np.array([[8.0, 0.0, 1.5], [4.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    pol.plan_obj = _TrajPlan(PiecewisePolynomial(0.0, [quintic(head, end, 2.0)]), np.array([-8.0, 0, 1.5]), 0.0, 0.01)
    cam = DepthCamera(CameraConfig.from_card(card.card["camera"]), card.obstacles, np.random.default_rng(0))
    p_odom = np.array([1.5, 0.0, 1.5])  # 真值在参考前面 0.5 m
    frame = cam.capture(p_odom, 0.0, 0.5)
    obs = Observation(frame, State(0.5, p_odom, np.array([4.0, 0, 0]), np.zeros(3), 0.0), np.array([-8.0, 0, 1.5]), 0.5)
    # 逼出无解分支：走廊为空
    monkeypatch.setattr(candidates, "select_action", lambda score, *a, **k: (0, True))
    return pol, obs


def test_off_keeps_upstream_hover(monkeypatch):
    pol, obs = _policy_and_obs(False, monkeypatch)
    out = pol.plan(obs)
    assert pol.last_code == "CORRIDOR_EMPTY_HOVER"
    p, v, _a = out.pva(0.5)
    np.testing.assert_allclose(p, obs.state.p)
    np.testing.assert_allclose(v, 0.0)


def test_on_brakes_from_reference_head(monkeypatch):
    pol, obs = _policy_and_obs(True, monkeypatch)
    ref_p, ref_v, _ = pol.plan_obj.pva(0.5)
    out = pol.plan(obs)
    assert pol.last_code == "CORRIDOR_EMPTY_HOVER"
    p, v, _a = out.pva(0.5)
    np.testing.assert_allclose(p, ref_p, atol=1e-9)
    np.testing.assert_allclose(v, ref_v, atol=1e-9)
    ts = np.linspace(out.traj.t_start, out.traj.t_end, 200)
    assert max(float(np.linalg.norm(out.traj.evaluate(float(t), 2))) for t in ts) <= 6.0 + 1e-6
    stop, vend, _ = out.pva(out.traj.t_end + 0.5)
    assert np.linalg.norm(vend) == 0.0
    assert stop[0] - ref_p[0] >= 16.0 / 12.0 - 1e-9  # 至少 v²/(2a)


def _card(on: bool):
    raw = json.loads((ROOT / "scenes/swap-8-open.json").read_text())
    if on:
        raw["policy_params"] = {"upstream": {"empty_brake": True}}
    raw["geometry"] = json.loads((ROOT / "scenes" / raw["geometry"]).read_text())
    return SceneCard(raw["name"], raw, parse_geometry(raw["geometry"], raw.get("ground_z")))


def test_runner_marks_deviation_only_when_on():
    off = run(load_card(ROOT / "scenes/swap-8-open.json"), RunConfig(seed=0, duration_s=0.2)).result
    assert "empty_brake" not in off and off["upstream_deviation"] is False
    on = run(_card(True), RunConfig(seed=0, duration_s=0.2)).result
    assert on["empty_brake"] is True and on["upstream_deviation"] is True
