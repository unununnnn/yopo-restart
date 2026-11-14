"""票 E-C5d：地面净空闸门。默认关；关时不读 Ground，轨迹与原来逐位相同。"""

import numpy as np
import pytest

import yopo_l0.policy as policy
from yopo_l0.dynamics import State
from yopo_l0.geom_fallback import brake_from_head, ground_safe_brake_from_head, iter_times, trajectory_min_z
from yopo_l0.policy import (
    GROUND_GUARD_CLEARANCE_M,
    Observation,
    PolicyContext,
    UpstreamPolicy,
    read_ground_guard,
    read_ground_guard_clearance_m,
)
from yopo_l0.scene import Cylinder, Ground
from yopo_l0.sensor import DepthFrame
from yopo_swarm_core.candidates import UPSTREAM_DECODER

from conftest import ROOT

_SAFE = 0.43


def test_default_off_and_cards_do_not_mention_the_switch():
    assert read_ground_guard(None) is False
    assert read_ground_guard({}) is False
    assert read_ground_guard({"upstream": {}}) is False
    assert read_ground_guard({"upstream": {"ground_guard": False}}) is False
    assert read_ground_guard({"upstream": {"ground_guard": True}}) is True
    assert read_ground_guard_clearance_m({}) == pytest.approx(GROUND_GUARD_CLEARANCE_M)
    assert GROUND_GUARD_CLEARANCE_M == pytest.approx(0.43)
    for path in sorted((ROOT / "scenes").glob("*.json")):
        text = path.read_text(encoding="utf-8")
        assert "ground_guard" not in text, path.name
    suite = (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    assert "e-c5d" not in suite and "ground_guard" not in suite


def test_invalid_bool_and_clearance_raise():
    for bad in ("true", 1, None, 0):
        with pytest.raises(ValueError):
            read_ground_guard({"upstream": {"ground_guard": bad}})
        with pytest.raises(ValueError):
            UpstreamPolicy(PolicyContext("uav0", 0.01, {"upstream": {"ground_guard": bad}}))
    for bad in (0, 0.0, -0.1, float("nan"), float("inf"), True, False, "0.43", None):
        with pytest.raises(ValueError):
            read_ground_guard_clearance_m({"upstream": {"ground_guard_clearance_m": bad}})
    with pytest.raises(ValueError):
        UpstreamPolicy(PolicyContext(
            "uav0", 0.01, {"upstream": {"ground_guard": True, "ground_guard_clearance_m": 0.0}},
        ))
    # 关掉时不读净空，非法值不影响构造，也不读障碍。
    class Trap:
        def __iter__(self):
            raise AssertionError("开关关着却读了 obstacles")

    UpstreamPolicy(PolicyContext(
        "uav0", 0.01,
        {"upstream": {"ground_guard": False, "ground_guard_clearance_m": -1}},
        obstacles=Trap(),
    ))


def _sample_z(traj) -> np.ndarray:
    times = iter_times(float(traj.t_start), float(traj.t_end), 0.02)
    return np.array([float(np.asarray(traj.evaluate(t, 0))[2]) for t in times])


def test_ground_safe_brake_stays_above_min_z_or_returns_the_plain_brake():
    high = np.array([[0.0, 0.0, 1.5], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    assert ground_safe_brake_from_head(high, 0.0, 6.0, _SAFE) is None
    assert brake_from_head(high, 0.0, 6.0) is None

    safe_head = np.array([[0.0, 0.0, 2.0], [1.0, 0.0, 0.2], [0.0, 0.0, 0.0]])
    plain = brake_from_head(safe_head, 1.0, 6.0)
    kept = ground_safe_brake_from_head(safe_head, 1.0, 6.0, _SAFE)
    assert plain is not None and kept is not None
    np.testing.assert_allclose(kept.segments[0].coeffs, plain.segments[0].coeffs)
    assert trajectory_min_z(plain) >= _SAFE

    dive = np.array([[0.0, 0.0, 0.8], [2.0, 0.0, -1.5], [0.0, 0.0, 0.0]])
    plain_dive = brake_from_head(dive, 0.0, 6.0)
    assert plain_dive is not None and trajectory_min_z(plain_dive) < _SAFE
    lifted = ground_safe_brake_from_head(dive, 0.0, 6.0, _SAFE)
    assert lifted is not None
    assert float(_sample_z(lifted).min()) >= _SAFE - 1e-6
    end = np.asarray(lifted.evaluate(lifted.t_end, 0), dtype=float)
    vend = np.asarray(lifted.evaluate(lifted.t_end, 1), dtype=float)
    assert end[2] >= _SAFE - 1e-6
    assert float(np.linalg.norm(vend)) < 1e-6

    # 贴地且静止：从当前高度拉到 min_z，不再往下。
    low = np.array([[1.0, 2.0, 0.30], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    climb = ground_safe_brake_from_head(low, 0.2, 6.0, _SAFE)
    zs = _sample_z(climb)
    assert float(zs.min()) >= 0.30 - 1e-6
    assert float(zs[-1]) >= _SAFE - 1e-6


def _frame():
    return DepthFrame(0.5, np.zeros((4, 4), np.float32), (1.0, 1.0, 1.0, 1.0))


def _obs(p, v=(0.0, 0.0, 0.0)):
    state = State(0.5, np.array(p, float), np.array(v, float), np.zeros(3), 0.0)
    return Observation(_frame(), state, np.array([30.0, 0.0, 1.5]), 0.5)


def _stub_rejected(monkeypatch):
    """走廊一个都不准入，迫使策略走无解分支。"""
    n = int(UPSTREAM_DECODER.count)

    def fake_forward(*_a, **_k):
        return (
            np.zeros((n, 14), np.float32),
            np.zeros(n, np.float64),
            np.zeros((n, 2), np.float32),
        )

    def fake_decode(_params, _dec):
        return np.zeros((n, 3)), np.zeros((n, 3, 3)), np.ones((n, 2))

    def fake_mu(_radius, _dec):
        return np.zeros(n, np.float64)

    monkeypatch.setattr("yopo_swarm_core.candidates.decode_params", fake_decode)
    monkeypatch.setattr("yopo_swarm_core.candidates.corridor_mu_lo", fake_mu)
    return fake_forward


def _coeffs(plan):
    return np.concatenate([seg.coeffs.ravel() for seg in plan.traj.segments])


def test_off_matches_explicit_false_and_ignores_ground(monkeypatch):
    class Trap:
        def __iter__(self):
            raise AssertionError("开关关着却读了 obstacles")

    _stub_rejected(monkeypatch)
    bare = UpstreamPolicy(PolicyContext("uav0", 0.01, {}, obstacles=Trap(), a_max=6.0))
    bare.source.forward_raw = _stub_rejected(monkeypatch)
    explicit = UpstreamPolicy(PolicyContext(
        "uav0", 0.01, {"upstream": {"ground_guard": False}}, obstacles=Trap(), a_max=6.0,
    ))
    explicit.source.forward_raw = _stub_rejected(monkeypatch)
    no_ground = UpstreamPolicy(PolicyContext(
        "uav0", 0.01,
        {"upstream": {"ground_guard": True}},
        obstacles=(Cylinder("c", (0.0, 0.0), 0.2, 0.0, 3.0),),
        a_max=6.0,
    ))
    no_ground.source.forward_raw = _stub_rejected(monkeypatch)
    obs = _obs((0.0, 0.0, 0.36))
    a = bare.plan(obs)
    b = explicit.plan(obs)
    c = no_ground.plan(obs)
    assert bare.last_code == explicit.last_code == no_ground.last_code == "CORRIDOR_EMPTY_HOVER"
    np.testing.assert_array_equal(_coeffs(a), _coeffs(b))
    np.testing.assert_array_equal(_coeffs(a), _coeffs(c))
    # 关着：无解就是停在当前位置，包括贴地。
    np.testing.assert_allclose(np.asarray(a.pva(0.5)[0]), [0.0, 0.0, 0.36])


def _stub_two_heights(monkeypatch, head_z: float):
    """高分候选停在 0.20 m，低分候选停在 1.20 m。两条都准入。"""
    n = int(UPSTREAM_DECODER.count)
    head = np.array([0.0, 0.0, head_z])
    positions = np.zeros((n, 3))
    positions[:, 2] = 1.5
    positions[0] = (0.0, 0.0, 0.20)
    positions[1] = (0.0, 0.0, 1.20)
    score = np.full(n, -1.0)
    score[0] = 5.0
    score[1] = 1.0
    mu = np.zeros(n)
    mu[0] = 1.0
    mu[1] = 1.0
    inner_b = positions - head

    def fake_forward(*_a, **_k):
        return np.zeros((n, 14), np.float32), score, np.zeros((n, 2), np.float32)

    def fake_decode(_params, _dec):
        return inner_b.copy(), np.zeros((n, 3, 3)), np.ones((n, 2))

    def fake_mu(_radius, _dec):
        return mu.copy()

    def fake_minco(t0, _head, _tail, inner, _durations):
        from yopo_swarm_core.plan import hold_traj

        return hold_traj(np.asarray(inner, float).reshape(3), float(t0), float(t0) + 0.2)

    monkeypatch.setattr("yopo_swarm_core.candidates.decode_params", fake_decode)
    monkeypatch.setattr("yopo_swarm_core.candidates.corridor_mu_lo", fake_mu)
    monkeypatch.setattr("yopo_swarm_core.candidates.minco_two_piece", fake_minco)
    return fake_forward


def test_guard_skips_a_candidate_that_drops_below_safe_z(monkeypatch):
    _stub_two_heights(monkeypatch, 1.5)
    params = {"upstream": {"ground_guard": True}}
    pol = UpstreamPolicy(PolicyContext("uav0", 0.01, params, obstacles=(Ground(0.0),), a_max=6.0))
    pol.source.forward_raw = _stub_two_heights(monkeypatch, 1.5)
    out = pol.plan(_obs((0.0, 0.0, 1.5)))
    assert pol.last_code == "OK"
    np.testing.assert_allclose(np.asarray(out.pva(0.5)[0]), [0.0, 0.0, 1.20], atol=1e-9)


def test_low_hover_lifts_instead_of_holding_on_the_ground(monkeypatch):
    _stub_rejected(monkeypatch)
    params = {"upstream": {"ground_guard": True, "ground_guard_clearance_m": _SAFE}}
    obstacles = (Ground(0.0), Cylinder("c", (4.0, 0.0), 0.2, 0.0, 3.0))
    pol = UpstreamPolicy(PolicyContext("uav0", 0.01, params, obstacles=obstacles, a_max=6.0))
    pol.source.forward_raw = _stub_rejected(monkeypatch)
    out = pol.plan(_obs((0.0, 0.0, 0.36)))
    assert pol.last_code == "CORRIDOR_EMPTY_HOVER"
    zs = _sample_z(out.traj)
    assert float(zs.min()) >= 0.36 - 1e-6
    assert float(zs[-1]) >= _SAFE - 1e-6
    # 机体半径 0.35。拉起后的参考不再贴地。
    assert float(zs.min()) > 0.35


def test_empty_brake_pullup_does_not_dive_through_min_z(monkeypatch):
    _stub_rejected(monkeypatch)
    params = {"upstream": {"ground_guard": True, "empty_brake": True}}
    pol = UpstreamPolicy(PolicyContext(
        "uav0", 0.01, params, obstacles=(Ground(0.0),), a_max=6.0,
    ))
    pol.source.forward_raw = _stub_rejected(monkeypatch)
    out = pol.plan(_obs((0.0, 0.0, 0.70), v=(1.0, 0.0, -1.2)))
    assert pol.last_code == "CORRIDOR_EMPTY_HOVER"
    assert float(_sample_z(out.traj).min()) >= _SAFE - 1e-6
