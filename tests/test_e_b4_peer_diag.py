"""票 E-B4：机间碰撞诊断脚本的结构检查。脚本只读，跑完把换下的东西还原。"""

import importlib.util
import json
import sys

import numpy as np
import pytest

import yopo_l0.backend.bspline as bspline
import yopo_l0.policy as policy
import yopo_l0.runner as runner

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-b4-peer-diag.py"
    spec = importlib.util.spec_from_file_location("e_b4_peer_diag", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_cards_do_not_touch_committed_scenes():
    mod = _script()
    before = {k: (ROOT / v).read_text() for k, v in mod.SCENES.items()}
    off = mod.build_card("swap8", "off")
    a1 = mod.build_card("swap8", "a1", {"upstream": {"inter_drone_threshold_m": 1.0}})
    b2 = mod.build_card("d014-swap3", "b2")
    assert "policy_params" not in off.card
    up = a1.card["policy_params"]["upstream"]
    assert up["inter_drone_filter"] is True and up["inter_drone_threshold_m"] == pytest.approx(1.0)
    assert b2.card["policy_params"]["backend"] == {"enabled": True}
    assert {k: (ROOT / v).read_text() for k, v in mod.SCENES.items()} == before
    suite = (ROOT / "scenes/suite.txt").read_text()
    assert "e-b4" not in suite


def test_layers_report_zero_margin_defaults():
    mod = _script()
    off = mod._layers(mod.build_card("swap8", "off"))
    assert off["collision_line_m"] == pytest.approx(0.70)
    assert off["e_a1"]["enabled"] is False
    assert off["d1"]["enabled"] is False and off["d1"]["threshold_m"] == pytest.approx(0.70)
    assert off["camera"]["renders_peers"] is False and off["shadow"]["peers"] == 0
    a1 = mod._layers(mod.build_card("swap8", "a1"))
    assert a1["e_a1"]["enabled"] is True and a1["e_a1"]["threshold_m"] == pytest.approx(0.70)


def test_note_pairs_records_first_crossing_and_delays_stop():
    from yopo_l0.dynamics import State

    mod = _script()
    log = mod._Log()
    log.stop_after = 0.05
    fn = mod._make_note_pairs(log)

    def drones(t, gap):
        return [
            {"id": "a", "state": State(t, np.array([0.0, 0.0, 1.5]), np.zeros(3), np.zeros(3))},
            {"id": "b", "state": State(t, np.array([gap, 0.0, 1.5]), np.zeros(3), np.zeros(3))},
        ]

    m, out, why = fn(drones(1.0, 0.71), 0.35, None, None, "")
    assert out is None and m == pytest.approx(0.71) and log.cross is None
    m, out, why = fn(drones(1.01, 0.69), 0.35, m, None, "")
    assert out is None and log.cross["t"] == pytest.approx(1.01) and log.cross["pair"] == ["a", "b"]
    m2, out, why = fn(drones(1.03, 0.10), 0.35, m, None, "")
    assert out is None and m2 == pytest.approx(0.69)  # 越线之后不再改截断口径的最小值
    _m, out, why = fn(drones(1.06, 0.10), 0.35, m2, None, "")
    assert out == "COLLISION" and why == "a-b:INTER_DRONE"


def test_traj_vs_neighbors_matches_hand_value():
    from yopo_swarm_core.plan import hold_traj

    mod = _script()
    traj = hold_traj(np.array([0.0, 0.0, 1.5]), 0.0, 1.0)
    nb = np.array([[0.0, 3.0, 4.0, 1.5], [0.5, 0.6, 0.0, 1.5], [1.0, 0.0, 0.0, 2.0]])
    center, ellip = mod._traj_vs_neighbors(traj, [nb], c=2.0)
    assert center == pytest.approx(0.5)
    assert ellip == pytest.approx(0.6)
    assert mod._traj_vs_neighbors(traj, [np.array([[5.0, 0.0, 0.0, 1.5]])]) == (None, None)


def test_proto_brake_starts_from_reference_and_stops():
    from yopo_l0.policy import PolicyContext, UpstreamPolicy, _TrajPlan
    from yopo_swarm_core.plan import PiecewisePolynomial, quintic

    mod = _script()
    pol = UpstreamPolicy(PolicyContext("uav0", 0.01, {}, a_max=6.0))
    head = np.array([[0.0, 0.0, 1.5], [4.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    end = np.array([[8.0, 0.0, 1.5], [4.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    prev = _TrajPlan(PiecewisePolynomial(0.0, [quintic(head, end, 2.0)]), np.array([20.0, 0, 1.5]), 0.0, 0.01)

    class Obs:
        pass

    obs = Obs()
    obs.now = 0.5
    obs.goal = np.array([20.0, 0.0, 1.5])
    obs.state = type("S", (), {"p": np.array([1.5, 0.0, 1.5]), "v": np.array([4.0, 0, 0]), "yaw": 0.0})()
    out = mod._proto_brake(pol, prev, obs)
    p0, v0, _a0 = out.pva(0.5)
    pr, vr, _ar = prev.pva(0.5)
    np.testing.assert_allclose(p0, pr, atol=1e-9)
    np.testing.assert_allclose(v0, vr, atol=1e-9)
    p1, v1, _a1 = out.pva(out.traj.t_end + 0.1)
    assert np.linalg.norm(v1) == 0.0
    # 4 m/s、a_max 6 时停距至少 v²/(2a)
    assert p1[0] - p0[0] >= 16.0 / 12.0 - 1e-9
    ts = np.linspace(out.traj.t_start, out.traj.t_end, 200)
    acc = max(float(np.linalg.norm(out.traj.evaluate(float(t), 2))) for t in ts)
    assert acc <= 6.0 + 1e-6
    assert pol.plan_obj is out


def test_patches_are_restored(monkeypatch):
    mod = _script()
    before = (policy.POLICIES["upstream"], runner._note_pairs, bspline.refine_yopo)
    calls = {}

    def fake_run(card, cfg, **kw):
        calls["n"] = 1
        assert policy.POLICIES["upstream"] is not before[0]
        assert runner._note_pairs is not before[1]
        raise RuntimeError("stop")

    monkeypatch.setattr(runner, "run", fake_run)
    with pytest.raises(RuntimeError):
        mod.diagnose("swap8", "off", 0, window_s=2.0, post_s=1.0)
    assert calls["n"] == 1
    assert (policy.POLICIES["upstream"], runner._note_pairs, bspline.refine_yopo) == before


def test_committed_report_rows_are_json():
    path = ROOT / "docs/research/e-b4-peer-collision-diag-runs.json"
    if not path.exists():
        pytest.skip("报告数据还没写入")
    data = json.loads(path.read_text())
    assert data["disclaimer"]
    for row in data["compare"]:
        assert row["scene"] in ("swap8", "d014-swap3")
        assert row["seed"] in range(10)
