"""E-B2：后段默认关，打开时才拟合和推障碍。关掉时现有卡同 seed 逐位一致。"""

import json
from pathlib import Path

import numpy as np
import pytest
from yopo_swarm_core.plan import quintic
from yopo_swarm_core.trajectory import PiecewisePolynomial

from yopo_l0.backend.bspline import (
    _M,
    _basis,
    _boundary_controls,
    _cost_grad,
    _sample_layout,
    _sdf_obstacle,
    min_signed_distance,
    refine_yopo,
)
from yopo_l0.backend.config import BackendConfig, config_from_params
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import Box, Cylinder, Ground, load_card

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / "scenes"
NEW_CARDS = {"g2-pillar-bspline.json", "crossing-forest-dense-bspline.json"}


def _line(y: float = 0.0, speed: float = 2.0, duration: float = 1.6) -> PiecewisePolynomial:
    p0 = np.array([-2.0, y, 1.5])
    p1 = np.array([2.0, y, 1.5])
    v = np.array([(p1[0] - p0[0]) / duration, 0.0, 0.0])
    head = np.array([p0, v * 0.0 + np.array([speed, 0.0, 0.0]), np.zeros(3)])
    # 匀速：两端速度相同，加速度为 0。位移必须和速度乘时间一致。
    head = np.array([p0, np.array([speed, 0.0, 0.0]), np.zeros(3)])
    end = np.array([p0 + head[1] * duration, head[1].copy(), np.zeros(3)])
    return PiecewisePolynomial(0.0, [quintic(head, end, duration)])


def _cfg(**kwargs) -> BackendConfig:
    base = dict(a_max=6.0, v_max=6.0, max_iterations=8, time_budget_s=0.05, clearance_m=0.35)
    base.update(kwargs)
    return BackendConfig(**base)


def test_switch_is_off_unless_json_true():
    assert config_from_params({}, a_max=6.0) is None
    assert config_from_params({"backend": {}}, a_max=6.0) is None
    assert config_from_params({"backend": {"enabled": False}}, a_max=6.0) is None
    assert config_from_params({"backend": {"enabled": "true"}}, a_max=6.0) is None
    cfg = config_from_params({"backend": {"enabled": True}}, a_max=6.0, clearance_m=0.4)
    assert cfg is not None
    assert cfg.a_max == 6.0 and cfg.clearance_m == 0.4 and cfg.v_max == 6.0
    assert cfg.max_iterations == 8 and cfg.time_budget_s == pytest.approx(0.020)


def test_existing_scene_cards_leave_backend_off():
    seen = 0
    for path in sorted(SCENES.glob("*.json")):
        raw = json.loads(path.read_text())
        if "task" not in raw or path.name in NEW_CARDS:
            continue
        seen += 1
        assert config_from_params(raw.get("policy_params"), a_max=6.0) is None, path.name
    assert seen >= 10


def test_new_cards_are_the_only_ones_enabled_and_are_not_frozen():
    for name in NEW_CARDS:
        raw = json.loads((SCENES / name).read_text())
        assert raw["criteria"]["frozen"] is False
        assert "gates" not in raw["criteria"]
        assert config_from_params(raw.get("policy_params"), a_max=6.0) is not None
    suite = (SCENES / "suite.txt").read_text()
    assert "bspline" not in suite


def test_too_short_returns_the_same_object():
    traj = _line(duration=0.3)
    out, rec = refine_yopo(traj, _cfg(), ())
    assert out is traj
    assert rec["applied"] is False and rec["reason"] == "too_short"


def test_endpoint_pva_matches_the_input_line():
    traj = _line()
    out, rec = refine_yopo(traj, _cfg(), ())
    assert rec["applied"] is True
    for order in range(3):
        np.testing.assert_allclose(out.evaluate(out.t_start, order), traj.evaluate(traj.t_start, order), atol=1e-6)
        np.testing.assert_allclose(out.evaluate(out.t_end, order), traj.evaluate(traj.t_end, order), atol=1e-6)


def test_basis_and_boundary_controls_reproduce_pva():
    p = np.array([1.0, -2.0, 0.5])
    v = np.array([0.4, 0.1, -0.2])
    a = np.array([0.3, -0.5, 0.2])
    dt = 0.2
    q0, q1, q2 = _boundary_controls(p, v, a, dt)
    # 段起点只用前三个控制点；第四个不进入 u=0 的位置、速度、加速度。
    q = np.stack([q0, q1, q2, np.zeros(3)])
    b0 = _basis(0.0)
    assert b0[3] == pytest.approx(0.0)
    np.testing.assert_allclose(b0 @ q, p, atol=1e-9)

    def dbasis(u: float) -> np.ndarray:
        return np.array([0.0, 1.0, 2.0 * u, 3.0 * u * u]) @ _M

    def d2basis(u: float) -> np.ndarray:
        return np.array([0.0, 0.0, 2.0, 6.0 * u]) @ _M

    np.testing.assert_allclose(dbasis(0.0) @ q / dt, v, atol=1e-9)
    np.testing.assert_allclose(d2basis(0.0) @ q / dt**2, a, atol=1e-9)


def test_sdf_matches_scene_distance_outside():
    cyl = Cylinder("c", (0.0, 0.2), 0.4, 0.0, 3.0, (0.1, -0.05))
    box = Box("b", (1.0, -0.5, 0.0), (1.4, 0.5, 2.0))
    ground = Ground(0.0)
    rng = np.random.default_rng(0)
    pts = rng.normal(size=(40, 3)) * np.array([3.0, 2.0, 2.0]) + np.array([0.0, 0.0, 1.5])
    for ob in (cyl, box, ground):
        sd, _g = _sdf_obstacle(pts, ob)
        for p, s in zip(pts, sd, strict=True):
            dist = ob.distance(p)
            if dist > 1e-6:
                assert s == pytest.approx(dist, abs=1e-6)


def test_cost_gradient_matches_finite_difference_away_from_kinks():
    traj = _line(y=-0.8)
    # 直线在圆柱侧面外，惩罚是光滑的（距离明显大于门槛）。
    cyl = Cylinder("c", (0.0, 0.0), 0.3, 0.0, 3.0)
    out_unused, rec = refine_yopo(traj, _cfg(max_iterations=1), (cyl,))
    assert rec["reason"] in {"applied", "clearance", "kinematics"}
    from yopo_l0.backend.bspline import _choose_knots, _fit_free, _pva

    duration = float(traj.t_end - traj.t_start)
    n_span, dt = _choose_knots(duration)
    p0, v0, a0 = _pva(traj, traj.t_start)
    p1, v1, a1 = _pva(traj, traj.t_end)
    n_ctrl = n_span + 3
    q = np.zeros((n_ctrl, 3))
    q[0], q[1], q[2] = _boundary_controls(p0, v0, a0, dt)
    q[-3], q[-2], q[-1] = _boundary_controls(p1, v1, a1, dt)
    free = np.arange(3, n_ctrl - 3)
    q, _rmse = _fit_free(traj, q, free, n_span, dt)
    q = q.copy()
    q[free[0], 1] -= 0.3
    idx, wts = _sample_layout(n_span)
    cost, grad = _cost_grad(q, q, free, dt, (cyl,), 0.4, 6.0, 6.0, idx, wts)
    eps = 1e-6
    index = int(free[0])
    for axis in range(3):
        qp = q.copy()
        qm = q.copy()
        qp[index, axis] += eps
        qm[index, axis] -= eps
        cp, _ = _cost_grad(qp, q, free, dt, (cyl,), 0.4, 6.0, 6.0, idx, wts)
        cm, _ = _cost_grad(qm, q, free, dt, (cyl,), 0.4, 6.0, 6.0, idx, wts)
        numeric = (cp - cm) / (2 * eps)
        assert numeric == pytest.approx(grad[index, axis], rel=1e-3, abs=1e-4)
    assert np.isfinite(cost)


def test_cylinder_penalty_increases_clearance():
    traj = _line(y=0.0, speed=2.0, duration=1.6)
    # 轴线在 y=0.15，半径 0.45。直线 y=0 穿进圆柱 0.3 m。
    cyl = Cylinder("trunk", (0.0, 0.15), 0.45, 0.0, 4.0)
    before = min_signed_distance(traj, (cyl,))
    assert before < 0.0
    out, rec = refine_yopo(traj, _cfg(max_iterations=8, time_budget_s=0.05), (cyl,))
    assert rec["applied"] is True, rec
    after = min_signed_distance(out, (cyl,))
    assert after > before + 0.05
    # 加速度仍在场景上限内：采样检查，提交契约由 refine 自己挡过。
    times = np.linspace(out.t_start, out.t_end, 40)
    acc = np.array([np.linalg.norm(out.evaluate(float(t), 2)) for t in times])
    assert float(acc.max()) <= 6.0 + 1e-6


def test_iteration_cap_and_time_budget():
    traj = _line()
    cyl = Cylinder("trunk", (0.0, 0.15), 0.45, 0.0, 4.0)
    _out, capped = refine_yopo(traj, _cfg(max_iterations=2, time_budget_s=1.0), (cyl,))
    assert capped["iterations"] <= 2
    _out, cut = refine_yopo(traj, _cfg(max_iterations=8, time_budget_s=1e-4), (cyl,))
    assert cut["budget_cut"] is True
    assert cut["time_ms"] < 50.0


def test_failure_returns_the_original_object(monkeypatch):
    traj = _line()

    def boom(*_a, **_k):
        raise ValueError("solver")

    monkeypatch.setattr("yopo_l0.backend.bspline._minimize", boom)
    out, rec = refine_yopo(traj, _cfg(), (Cylinder("c", (0.0, 0.0), 0.2, 0.0, 2.0),))
    assert out is traj
    assert rec["applied"] is False and rec["reason"] == "exception"


def _task_cards():
    found = []
    for path in sorted(SCENES.glob("*.json")):
        raw = json.loads(path.read_text())
        if "task" in raw and "drones" in raw and path.name not in NEW_CARDS:
            found.append(path)
    return found


def _policy_for(path: Path) -> str:
    task = json.loads(path.read_text())["task"]["type"]
    return {"hover": "hover", "goto": "upstream"}.get(task, "upstream")


def _strip(result):
    return {k: v for k, v in result.items() if k not in WALL_FIELDS}


@pytest.mark.parametrize("path", _task_cards(), ids=lambda p: p.name)
def test_backend_off_is_bitwise_for_existing_cards(path, monkeypatch):
    def boom(*_args, **_kwargs):
        raise AssertionError("后段在开关关闭时被调用")

    monkeypatch.setattr("yopo_l0.backend.bspline.refine_yopo", boom)
    card = load_card(path)
    cfg = RunConfig(seed=0, policy=_policy_for(path), shadow=False, duration_s=0.2)
    first = run(card, cfg)
    second = run(card, cfg)
    assert "backend" not in first.result
    assert _strip(first.result) == _strip(second.result)
    assert first.ticks == second.ticks
