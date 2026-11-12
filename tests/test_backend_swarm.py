"""E-B3：后段机间椭球惩罚默认关。关上时与 E-B2 逐位相同；打开才读邻机采样。"""

import json
from pathlib import Path

import numpy as np
import pytest
from yopo_swarm_core.plan import quintic
from yopo_swarm_core.trajectory import PiecewisePolynomial

from yopo_l0.backend.bspline import (
    _boundary_controls,
    _choose_knots,
    _cost_grad,
    _fit_free,
    _pva,
    _sample_layout,
    _swarm_cost_grad,
    refine_yopo,
)
from yopo_l0.backend.config import (
    DEFAULT_SWARM_C,
    DEFAULT_SWARM_CLEARANCE_M,
    BackendConfig,
    config_from_params,
    swarm_penalty_enabled,
)
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import SceneCard, load_card

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / "scenes"


def _line(y: float = 0.0, speed: float = 2.0, duration: float = 1.6, t0: float = 0.0) -> PiecewisePolynomial:
    p0 = np.array([-2.0, y, 1.5])
    head = np.array([p0, np.array([speed, 0.0, 0.0]), np.zeros(3)])
    end = np.array([p0 + head[1] * duration, head[1].copy(), np.zeros(3)])
    return PiecewisePolynomial(t0, [quintic(head, end, duration)])


def _cfg(**kwargs) -> BackendConfig:
    base = dict(a_max=6.0, v_max=6.0, max_iterations=8, time_budget_s=0.05, clearance_m=0.35)
    base.update(kwargs)
    return BackendConfig(**base)


def _samples(traj: PiecewisePolynomial, dt: float = 0.02) -> np.ndarray:
    n = int(round((float(traj.t_end) - float(traj.t_start)) / dt)) + 1
    times = float(traj.t_start) + np.arange(n, dtype=float) * dt
    pos = np.array([np.asarray(traj.evaluate(float(t), 0), dtype=float) for t in times])
    return np.column_stack((times, pos))


def _positions(traj, n: int = 40) -> np.ndarray:
    times = np.linspace(float(traj.t_start), float(traj.t_end), n)
    return np.array([np.asarray(traj.evaluate(float(t), 0), dtype=float) for t in times])


def test_swarm_switch_is_off_unless_json_true():
    assert swarm_penalty_enabled({}) is False
    assert swarm_penalty_enabled({"backend": {"enabled": True}}) is False
    assert swarm_penalty_enabled({"backend": {"enabled": True, "swarm_penalty": False}}) is False
    assert swarm_penalty_enabled({"backend": {"enabled": True, "swarm_penalty": "true"}}) is False
    assert swarm_penalty_enabled({"backend": {"swarm_penalty": True}}) is False
    cfg = config_from_params({"backend": {"enabled": True, "swarm_penalty": True}}, a_max=6.0)
    assert cfg is not None and cfg.swarm_penalty is True
    assert cfg.swarm_clearance_m == pytest.approx(DEFAULT_SWARM_CLEARANCE_M)
    assert cfg.swarm_c == pytest.approx(DEFAULT_SWARM_C)
    custom = config_from_params(
        {"backend": {"enabled": True, "swarm_penalty": True, "swarm_clearance_m": 1.0, "swarm_c": 3.0}},
        a_max=6.0,
    )
    assert custom is not None
    assert custom.swarm_clearance_m == pytest.approx(1.0)
    assert custom.swarm_c == pytest.approx(3.0)


def test_swarm_params_rejected_when_on():
    with pytest.raises(ValueError, match="swarm_clearance_m"):
        BackendConfig(swarm_penalty=True, swarm_clearance_m=0.0)
    with pytest.raises(ValueError, match="swarm_c"):
        BackendConfig(swarm_penalty=True, swarm_c=1.0)


def test_existing_scene_cards_leave_swarm_off():
    seen = 0
    for path in sorted(SCENES.glob("*.json")):
        raw = json.loads(path.read_text())
        if "task" not in raw:
            continue
        seen += 1
        assert swarm_penalty_enabled(raw.get("policy_params")) is False, path.name
    assert seen >= 10
    suite = (SCENES / "suite.txt").read_text()
    assert "swarm_penalty" not in suite


def test_swarm_off_ignores_neighbors_bitwise():
    traj = _line()
    close = _samples(_line(y=0.40))
    off = _cfg()
    out_a, rec_a = refine_yopo(traj, off, ())
    out_b, rec_b = refine_yopo(traj, off, (), neighbors=[close])
    assert rec_a["applied"] is True and rec_b["applied"] is True
    np.testing.assert_array_equal(_positions(out_a), _positions(out_b))
    assert rec_a["iterations"] == rec_b["iterations"]
    assert rec_a["reason"] == rec_b["reason"]


def test_swarm_on_without_neighbors_matches_off():
    traj = _line()
    off = _cfg()
    on = _cfg(swarm_penalty=True)
    out_off, rec_off = refine_yopo(traj, off, ())
    out_on, rec_on = refine_yopo(traj, on, (), neighbors=None)
    assert rec_off["applied"] is True and rec_on["applied"] is True
    np.testing.assert_array_equal(_positions(out_off), _positions(out_on))
    assert rec_off["iterations"] == rec_on["iterations"]


def test_far_neighbor_does_not_change_the_curve():
    traj = _line()
    far = _samples(_line(y=8.0))
    off = _cfg()
    on = _cfg(swarm_penalty=True)
    out_off, rec_off = refine_yopo(traj, off, ())
    out_on, rec_on = refine_yopo(traj, on, (), neighbors=[far])
    assert rec_off["applied"] is True and rec_on["applied"] is True
    np.testing.assert_allclose(_positions(out_off), _positions(out_on), atol=1e-12)


def test_close_neighbor_pushes_the_curve_away():
    traj = _line(y=0.0)
    neighbor = _samples(_line(y=0.40))
    out, rec = refine_yopo(traj, _cfg(swarm_penalty=True, max_iterations=8, time_budget_s=0.10), (), neighbors=[neighbor])
    assert rec["applied"] is True, rec
    mid_t = 0.5 * (float(out.t_start) + float(out.t_end))
    y = float(out.evaluate(mid_t, 0)[1])
    # 邻机在 y=0.40，本机初值 y=0，应被推到更负的一侧。
    assert y < -0.05


def test_swarm_gradient_matches_finite_difference():
    traj = _line(y=0.0)
    n_span, dt = _choose_knots(float(traj.t_end - traj.t_start))
    p0, v0, a0 = _pva(traj, traj.t_start)
    p1, v1, a1 = _pva(traj, traj.t_end)
    n_ctrl = n_span + 3
    q = np.zeros((n_ctrl, 3))
    q[0], q[1], q[2] = _boundary_controls(p0, v0, a0, dt)
    q[-3], q[-2], q[-1] = _boundary_controls(p1, v1, a1, dt)
    free = np.arange(3, n_ctrl - 3)
    q, _rmse = _fit_free(traj, q, free, n_span, dt)
    # 把一个中间点移近邻机，但仍离开铰链和原点，梯度光滑。
    q = q.copy()
    q[int(free[len(free) // 2]), 1] = 0.25
    neighbor = _samples(_line(y=0.40))
    cost, grad = _swarm_cost_grad(q, dt, float(traj.t_start), [neighbor], 0.80, 2.0)
    assert cost > 0.0
    eps = 1e-6
    index = int(free[len(free) // 2])
    for axis in range(3):
        qp, qm = q.copy(), q.copy()
        qp[index, axis] += eps
        qm[index, axis] -= eps
        cp, _ = _swarm_cost_grad(qp, dt, float(traj.t_start), [neighbor], 0.80, 2.0)
        cm, _ = _swarm_cost_grad(qm, dt, float(traj.t_start), [neighbor], 0.80, 2.0)
        numeric = (cp - cm) / (2 * eps)
        assert numeric == pytest.approx(grad[index, axis], rel=1e-3, abs=1e-4)


def test_full_cost_with_swarm_off_matches_plain_cost_grad():
    traj = _line(y=-0.8)
    n_span, dt = _choose_knots(float(traj.t_end - traj.t_start))
    p0, v0, a0 = _pva(traj, traj.t_start)
    p1, v1, a1 = _pva(traj, traj.t_end)
    n_ctrl = n_span + 3
    q = np.zeros((n_ctrl, 3))
    q[0], q[1], q[2] = _boundary_controls(p0, v0, a0, dt)
    q[-3], q[-2], q[-1] = _boundary_controls(p1, v1, a1, dt)
    free = np.arange(3, n_ctrl - 3)
    q, _rmse = _fit_free(traj, q, free, n_span, dt)
    idx, wts = _sample_layout(n_span)
    cost, grad = _cost_grad(q, q, free, dt, (), 0.4, 6.0, 6.0, idx, wts)
    sc, sg = _swarm_cost_grad(q, dt, float(traj.t_start), [], 0.80, 2.0)
    assert sc == 0.0
    np.testing.assert_array_equal(sg, np.zeros_like(q))
    np.testing.assert_array_equal(grad, grad + sg)
    assert np.isfinite(cost)


def test_broadcast_and_backend_stay_off_on_existing_goto_card(monkeypatch):
    published = []

    from yopo_l0.bus import MessageBus

    orig = MessageBus.publish

    def hook(self, now, *, src, dest, topic, payload):
        published.append(dict(payload))
        return orig(self, now, src=src, dest=dest, topic=topic, payload=payload)

    monkeypatch.setattr(MessageBus, "publish", hook)
    card = load_card(SCENES / "swap-3-open.json")
    out = run(card, RunConfig(seed=0, policy="upstream", shadow=False, duration_s=0.3))
    assert "backend" not in out.result
    assert "inter_drone_filter" not in out.result
    plan_payloads = [p for p in published if p.get("kind") == "peer_plan"]
    assert plan_payloads
    assert all("traj_samples" not in p for p in plan_payloads)


def test_swarm_on_broadcasts_samples_and_records_backend(monkeypatch):
    base = load_card(SCENES / "swap-3-open.json")
    raw = json.loads(json.dumps(base.card))
    raw["policy_params"] = {
        "backend": {
            "enabled": True,
            "swarm_penalty": True,
            "swarm_clearance_m": 0.80,
            "swarm_c": 2.0,
        }
    }
    card = SceneCard(base.path, raw, base.obstacles)
    published = []
    from yopo_l0.bus import MessageBus

    orig = MessageBus.publish

    def hook(self, now, *, src, dest, topic, payload):
        published.append(dict(payload))
        return orig(self, now, src=src, dest=dest, topic=topic, payload=payload)

    monkeypatch.setattr(MessageBus, "publish", hook)
    out = run(card, RunConfig(seed=0, policy="upstream", shadow=False, duration_s=0.4))
    assert out.result.get("backend", {}).get("enabled") is True
    assert out.result["backend"]["calls"] >= 1
    plan_payloads = [p for p in published if p.get("kind") == "peer_plan"]
    assert plan_payloads
    assert any(isinstance(p.get("traj_samples"), list) and p["traj_samples"] for p in plan_payloads)
