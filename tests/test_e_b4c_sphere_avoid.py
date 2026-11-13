"""E-B4c：后段欧氏碰撞球默认关。关上时与加这项之前逐位相同。"""

import json
from pathlib import Path

import numpy as np
import pytest
from yopo_swarm_core.plan import quintic
from yopo_swarm_core.trajectory import PiecewisePolynomial

from yopo_l0.backend.bspline import (
    _PEER_COLLISION_BALL_M,
    _boundary_controls,
    _choose_knots,
    _fit_free,
    _peer_sphere_cost_grad,
    _peer_sphere_sample,
    _pva,
    refine_yopo,
)
from yopo_l0.backend.config import (
    DEFAULT_PEER_SPHERE_RADIUS_M,
    DEFAULT_PEER_SPHERE_WEIGHT,
    BackendConfig,
    config_from_params,
    peer_sphere_enabled,
    swarm_penalty_enabled,
)
from yopo_l0.scene import load_card

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / "scenes"


def _line(y: float = 0.0, speed: float = 2.5, duration: float = 1.6) -> PiecewisePolynomial:
    p0 = np.array([-2.0, y, 1.5])
    head = np.array([p0, np.array([speed, 0.0, 0.0]), np.zeros(3)])
    end = np.array([p0 + head[1] * duration, head[1].copy(), np.zeros(3)])
    return PiecewisePolynomial(0.0, [quintic(head, end, duration)])


def _opposing(y: float = 0.30, speed: float = 2.5, duration: float = 1.6) -> PiecewisePolynomial:
    """与 ``_line`` 对向：从 x=+2 飞向 x=-2，横向错开 y。"""
    p0 = np.array([2.0, y, 1.5])
    vel = np.array([-speed, 0.0, 0.0])
    head = np.array([p0, vel, np.zeros(3)])
    end = np.array([p0 + vel * duration, vel.copy(), np.zeros(3)])
    return PiecewisePolynomial(0.0, [quintic(head, end, duration)])


def _cfg(**kwargs) -> BackendConfig:
    base = dict(a_max=6.0, v_max=6.0, max_iterations=8, time_budget_s=0.20, clearance_m=0.35)
    base.update(kwargs)
    return BackendConfig(**base)


def _samples(traj: PiecewisePolynomial, dt: float = 0.02) -> np.ndarray:
    n = int(round((float(traj.t_end) - float(traj.t_start)) / dt)) + 1
    times = float(traj.t_start) + np.arange(n, dtype=float) * dt
    pos = np.array([np.asarray(traj.evaluate(float(t), 0), dtype=float) for t in times])
    return np.column_stack((times, pos))


def _positions(traj, n: int = 41) -> np.ndarray:
    times = np.linspace(float(traj.t_start), float(traj.t_end), n)
    return np.array([np.asarray(traj.evaluate(float(t), 0), dtype=float) for t in times])


def _min_aligned(traj, samples: np.ndarray) -> float:
    times = samples[:, 0]
    mask = (times >= float(traj.t_start) - 1e-12) & (times <= float(traj.t_end) + 1e-12)
    t = np.clip(times[mask], float(traj.t_start), float(traj.t_end))
    pos = np.array([np.asarray(traj.evaluate(float(tt), 0), dtype=float) for tt in t])
    return float(np.linalg.norm(pos - samples[mask, 1:4], axis=1).min())


def _fit(traj):
    n_span, dt = _choose_knots(float(traj.t_end - traj.t_start))
    p0, v0, a0 = _pva(traj, traj.t_start)
    p1, v1, a1 = _pva(traj, traj.t_end)
    n_ctrl = n_span + 3
    q = np.zeros((n_ctrl, 3))
    q[0], q[1], q[2] = _boundary_controls(p0, v0, a0, dt)
    q[-3], q[-2], q[-1] = _boundary_controls(p1, v1, a1, dt)
    free = np.arange(3, n_ctrl - 3)
    q, _rmse = _fit_free(traj, q, free, n_span, dt)
    return q, free, dt, n_span


def test_peer_sphere_defaults_off_and_ignores_non_true():
    cfg = BackendConfig()
    assert cfg.peer_sphere_enabled is False
    assert cfg.peer_sphere_radius_m == pytest.approx(DEFAULT_PEER_SPHERE_RADIUS_M)
    assert cfg.peer_sphere_radius_m == pytest.approx(0.85)
    assert cfg.peer_sphere_weight == pytest.approx(DEFAULT_PEER_SPHERE_WEIGHT)
    assert cfg.peer_sphere_weight == pytest.approx(8.0)
    assert peer_sphere_enabled({}) is False
    assert peer_sphere_enabled({"backend": {"enabled": True}}) is False
    assert peer_sphere_enabled({"backend": {"enabled": True, "peer_sphere_enabled": False}}) is False
    assert peer_sphere_enabled({"backend": {"enabled": True, "peer_sphere_enabled": "true"}}) is False
    # 后段没开时，环境变量也不把求解器叫起来。
    assert peer_sphere_enabled({"backend": {"peer_sphere_enabled": True}}) is False
    # 椭球仍只看自己的开关。
    assert swarm_penalty_enabled({"backend": {"enabled": True, "peer_sphere_enabled": True}}) is False
    on = config_from_params({"backend": {"enabled": True, "peer_sphere_enabled": True}}, a_max=6.0)
    assert on is not None and on.peer_sphere_enabled is True and on.swarm_penalty is False
    assert on.swarm_segment_penalty is False
    assert on.peer_sphere_radius_m == pytest.approx(0.85)
    assert on.peer_sphere_weight == pytest.approx(8.0)
    custom = config_from_params(
        {
            "backend": {
                "enabled": True,
                "peer_sphere_enabled": True,
                "peer_sphere_radius_m": 1.0,
                "peer_sphere_weight": 3.0,
            }
        },
        a_max=6.0,
    )
    assert custom is not None
    assert custom.peer_sphere_radius_m == pytest.approx(1.0)
    assert custom.peer_sphere_weight == pytest.approx(3.0)


def test_peer_sphere_env_and_invalid_radius(monkeypatch):
    monkeypatch.setenv("YOPO_PEER_SPHERE", "1")
    cfg = config_from_params({"backend": {"enabled": True}}, a_max=6.0)
    assert cfg is not None and cfg.peer_sphere_enabled is True
    monkeypatch.setenv("YOPO_PEER_SPHERE", "false")
    off = config_from_params({"backend": {"enabled": True}}, a_max=6.0)
    assert off is not None and off.peer_sphere_enabled is False
    monkeypatch.setenv("YOPO_PEER_SPHERE", "yes")
    monkeypatch.setenv("YOPO_PEER_SPHERE_RADIUS_M", "0.95")
    monkeypatch.setenv("YOPO_PEER_SPHERE_WEIGHT", "4")
    via = config_from_params({"backend": {"enabled": True, "peer_sphere_radius_m": 1.2}}, a_max=6.0)
    assert via is not None and via.peer_sphere_enabled is True
    assert via.peer_sphere_radius_m == pytest.approx(0.95)
    assert via.peer_sphere_weight == pytest.approx(4.0)
    with pytest.raises(ValueError, match="peer_sphere_radius_m"):
        BackendConfig(peer_sphere_enabled=True, peer_sphere_radius_m=0.0)
    with pytest.raises(ValueError, match="peer_sphere_weight"):
        BackendConfig(peer_sphere_enabled=True, peer_sphere_weight=-1.0)


def test_existing_scene_cards_leave_peer_sphere_off():
    seen = 0
    for path in sorted(SCENES.glob("*.json")):
        raw = json.loads(path.read_text())
        if "task" not in raw:
            continue
        seen += 1
        assert peer_sphere_enabled(raw.get("policy_params")) is False, path.name
        blob = json.dumps(raw)
        assert "peer_sphere" not in blob
    assert seen >= 10
    suite = (SCENES / "suite.txt").read_text()
    assert "peer_sphere" not in suite


def test_peer_sphere_off_is_bitwise_with_close_neighbor():
    traj = _line()
    close = _samples(_opposing(y=0.30))
    assert _min_aligned(traj, close) < 0.70
    plain = _cfg()
    explicit = _cfg(peer_sphere_enabled=False)
    out_plain, rec_plain = refine_yopo(traj, plain, ())
    out_near, rec_near = refine_yopo(traj, plain, (), neighbors=[close])
    out_flag, rec_flag = refine_yopo(traj, explicit, (), neighbors=[close])
    assert rec_plain["applied"] and rec_near["applied"] and rec_flag["applied"]
    np.testing.assert_array_equal(_positions(out_plain), _positions(out_near))
    np.testing.assert_array_equal(_positions(out_plain), _positions(out_flag))
    assert rec_plain["iterations"] == rec_near["iterations"] == rec_flag["iterations"]
    assert rec_plain["reason"] == rec_near["reason"] == rec_flag["reason"]


def test_far_neighbor_with_sphere_on_matches_off():
    traj = _line()
    far = _samples(_opposing(y=8.0))
    off, rec_off = refine_yopo(traj, _cfg(), ())
    on, rec_on = refine_yopo(traj, _cfg(peer_sphere_enabled=True), (), neighbors=[far])
    assert rec_off["applied"] and rec_on["applied"]
    np.testing.assert_array_equal(_positions(off), _positions(on))
    assert rec_off["iterations"] == rec_on["iterations"]


def test_hinge_is_cubic_outside_the_ball_and_quadratic_inside():
    radius = 0.85
    weight = 8.0
    band = radius - _PEER_COLLISION_BALL_M
    assert band == pytest.approx(0.15)
    dist = np.array([1.2, 0.85, 0.80, 0.70, 0.40, 0.0])
    cost, dcost = _peer_sphere_sample(dist, radius, weight, band)
    assert cost[0] == 0.0 and cost[1] == 0.0
    assert dcost[0] == 0.0 and dcost[1] == 0.0
    # d = 0.80 在 0.70 与 0.85 之间，s = 0.05，三次带。
    s_shell = 0.05
    assert cost[2] == pytest.approx(weight * s_shell**3 / band)
    assert dcost[2] == pytest.approx(-(3.0 * weight * s_shell**2 / band))
    # d = 0.70 是两段的接点，代价和一阶导连续。
    s_ball = 0.15
    cubic = weight * s_ball**3 / band
    quad = weight * (s_ball**2 + band * s_ball - band**2)
    assert cubic == pytest.approx(quad)
    assert cost[3] == pytest.approx(quad)
    assert dcost[3] == pytest.approx(-(3.0 * weight * band))
    # 球内走二次。
    s_in = 0.45
    assert cost[4] == pytest.approx(weight * (s_in**2 + band * s_in - band**2))
    assert dcost[4] == pytest.approx(-(weight * (2.0 * s_in + band)))
    assert np.all(np.isfinite(cost)) and np.all(np.isfinite(dcost))
    assert cost[5] > cost[4] > cost[3] > cost[2] > 0.0


def test_sphere_gradient_matches_finite_difference_and_is_finite():
    traj = _line(y=0.0)
    q, free, dt, _n_span = _fit(traj)
    # 对向邻机在 y=0.30，参考最近约 0.30 m，落在二次段里，离开球面和原点。
    neighbor = _samples(_opposing(y=0.30))
    assert _min_aligned(traj, neighbor) < 0.70
    cost, grad = _peer_sphere_cost_grad(q, dt, float(traj.t_start), [neighbor], 0.85, 8.0)
    assert cost > 0.0
    assert np.isfinite(cost)
    assert np.all(np.isfinite(grad))
    # 最后一跨被排除，只出现在那一跨里的末端控制点没有梯度。
    # 倒数第二跨仍会把梯度分到尾部三元组的前两个；优化器不步它们。
    assert np.all(grad[-1] == 0.0)
    eps = 1e-6
    index = int(free[len(free) // 2])
    for axis in range(3):
        qp, qm = q.copy(), q.copy()
        qp[index, axis] += eps
        qm[index, axis] -= eps
        cp, gp = _peer_sphere_cost_grad(qp, dt, float(traj.t_start), [neighbor], 0.85, 8.0)
        cm, gm = _peer_sphere_cost_grad(qm, dt, float(traj.t_start), [neighbor], 0.85, 8.0)
        assert np.isfinite(cp) and np.isfinite(cm)
        assert np.all(np.isfinite(gp)) and np.all(np.isfinite(gm))
        numeric = (cp - cm) / (2 * eps)
        assert numeric == pytest.approx(grad[index, axis], rel=1e-3, abs=1e-4)


def test_headon_sphere_pushes_min_distance_without_nan():
    traj = _line(y=0.0, speed=2.5, duration=1.6)
    neighbor = _samples(_opposing(y=0.30, speed=2.5, duration=1.6))
    before = _min_aligned(traj, neighbor)
    assert before < 0.70
    cfg = _cfg(peer_sphere_enabled=True, max_iterations=8, time_budget_s=0.50)
    out, rec = refine_yopo(traj, cfg, (), neighbors=[neighbor])
    assert rec["applied"] is True, rec
    assert rec["reason"] == "applied"
    assert int(rec["iterations"]) >= 1
    pos = _positions(out, n=81)
    assert np.all(np.isfinite(pos))
    after = _min_aligned(out, neighbor)
    assert np.isfinite(after)
    # 参考最近在 0.30 m 附近。球铰链要把同时刻距离明显推开，同时钉住两端 PVA。
    assert after > before + 0.15
    for order in range(3):
        np.testing.assert_allclose(out.evaluate(out.t_start, order), traj.evaluate(traj.t_start, order), atol=1e-6)
        np.testing.assert_allclose(out.evaluate(out.t_end, order), traj.evaluate(traj.t_end, order), atol=1e-6)
    # 再在优化后的曲线上重算一次梯度，确认收敛点没有 NaN。
    q, _free, dt, _n = _fit(out)
    cost, grad = _peer_sphere_cost_grad(q, dt, float(out.t_start), [neighbor], 0.85, 8.0)
    assert np.isfinite(cost)
    assert np.all(np.isfinite(grad))


def test_sphere_on_does_not_enable_segment_endpoint_release():
    """欧氏球打开时末端仍钉死，和只开后段平滑一样。段内椭球开关保持原语义。"""
    traj = _line()
    neighbor = _samples(_opposing(y=0.30))
    out, rec = refine_yopo(traj, _cfg(peer_sphere_enabled=True), (), neighbors=[neighbor])
    assert rec["applied"] is True
    np.testing.assert_allclose(out.evaluate(out.t_end, 0), traj.evaluate(traj.t_end, 0), atol=1e-6)
    np.testing.assert_allclose(out.evaluate(out.t_end, 1), traj.evaluate(traj.t_end, 1), atol=1e-6)
    np.testing.assert_allclose(out.evaluate(out.t_end, 2), traj.evaluate(traj.t_end, 2), atol=1e-6)
    card = load_card(SCENES / "swap-3-open.json")
    assert peer_sphere_enabled(card.card.get("policy_params")) is False
