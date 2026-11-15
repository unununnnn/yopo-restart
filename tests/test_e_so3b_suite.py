"""票 E-SO3b：SO3 是场景级 / RunConfig 开关，默认关。关时轨迹与点质量逐位相同。"""

import importlib.util
import json
import sys

import numpy as np
import pytest
from dataclasses import replace

from conftest import ROOT
from yopo_l0.dynamics import HoverPlan, PointMassLag, State, make_model
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import SceneCard, load_card
from yopo_l0.so3 import (
    SOURCE,
    SWITCH_DEFAULTS,
    SO3LagModel,
    point_mass_config,
    switch_config,
)


HOVER = ROOT / "scenes/g2-hover.json"
PILLAR = ROOT / "scenes/g2-pillar.json"
SUITE_CARDS = (
    "scenes/g2-pillar.json",
    "scenes/formation-hold.json",
    "scenes/crossing-pillars.json",
    "scenes/crossing-forest.json",
    "scenes/formation-switch.json",
    "scenes/formation-6-open.json",
    "scenes/crossing-6-forest.json",
    "scenes/crossing-6-forest-sparse.json",
)


def _script():
    path = ROOT / "scripts/e-so3b-suite.py"
    spec = importlib.util.spec_from_file_location("e_so3b_suite", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _track(out):
    rows = []
    for row in out.ticks:
        rows.append([*row["p"], *row["v"], row["yaw"]])
    return np.asarray(rows, dtype=np.float64)


def _legacy_point_mass(model: PointMassLag, s: State, plan, dt: float):
    """加 SO3 之前 PointMassLag.step 的公式。测试把它钉死。"""
    p_ref, v_ref, a_ref, yaw = plan.sample(s.t)
    a_cmd = a_ref + model.kp * (p_ref - s.p) + model.kd * (v_ref - s.v)
    n = float(np.linalg.norm(a_cmd))
    if n > model.a_max:
        a_cmd = a_cmd * (model.a_max / n)
    alpha = 1.0 - np.exp(-dt / model.tau_s)
    a = s.a + alpha * (a_cmd - s.a)
    v = s.v + a * dt
    p = s.p + v * dt
    return p, v, a, yaw


def test_point_mass_step_matches_the_formula_and_source_file():
    text = (ROOT / "yopo_l0/dynamics.py").read_text(encoding="utf-8")
    assert "so3" not in text.lower()
    assert 'MODELS = {"point_mass_lag": PointMassLag}' in text
    model = PointMassLag()
    assert (model.tau_s, model.kp, model.kd, model.a_max) == (0.1, 4.0, 3.0, 6.0)
    s = State(
        0.2,
        np.array([1.0, -0.4, 1.5]),
        np.array([0.3, -0.2, 0.1]),
        np.array([1.0, 0.5, -0.4]),
        0.2,
    )
    plan = HoverPlan((2.0, 0.2, 1.2), 0.4)
    got = model.step(s, plan, 0.01)
    p, v, a, yaw = _legacy_point_mass(model, s, plan, 0.01)
    np.testing.assert_array_equal(got.p, p)
    np.testing.assert_array_equal(got.v, v)
    np.testing.assert_array_equal(got.a, a)
    assert got.yaw == yaw
    assert make_model({"model": "point_mass_lag", "tau_s": 0.1, "kp": 4.0, "kd": 3.0, "a_max": 6.0}).__class__ is PointMassLag


def test_switch_defaults_document_source_and_confidence():
    assert SWITCH_DEFAULTS["attitude_order"] == 1
    assert SWITCH_DEFAULTS["tau_att_s"] == pytest.approx(0.12)
    assert SWITCH_DEFAULTS["tau_thrust_s"] == pytest.approx(0.06)
    assert SWITCH_DEFAULTS["max_tilt_deg"] == pytest.approx(35.0)
    assert SWITCH_DEFAULTS["a_max"] == pytest.approx(6.0)
    assert SWITCH_DEFAULTS["yaw_weight"] == pytest.approx(0.4)
    assert SWITCH_DEFAULTS["roll_rate_max_deg_s"] == pytest.approx(220.0)
    assert SWITCH_DEFAULTS["yaw_rate_max_deg_s"] == pytest.approx(200.0)
    blob = json.dumps(SOURCE, ensure_ascii=False)
    assert "E-SO3" in blob
    assert "置信度低" in blob
    assert "置信度中" in blob
    assert "PX4" in blob
    assert "0.25" in SOURCE["tau_att_s"]
    model = SO3LagModel()
    assert model.attitude_order == 1
    assert model.k_R == pytest.approx(1.0 / 0.12)
    over = switch_config({"a_max": 5.0, "kp": 4.0, "model": "point_mass_lag"})
    assert over["a_max"] == pytest.approx(5.0)
    assert over["attitude_order"] == 1
    assert over["model"] == "so3_lag"
    back = point_mass_config({"kp": 4.0, "a_max": 5.0, "model": "so3_lag"})
    assert back["model"] == "point_mass_lag"
    assert back["a_max"] == pytest.approx(5.0)
    assert back["tau_s"] == pytest.approx(0.1)


def test_so3_limits_lag_and_keeps_vehicles_across_replace():
    wide = SO3LagModel(max_tilt_deg=35, a_max=6, kp=4, kd=3)
    b3, thrust = wide.desired_specific_force(np.array([6.0, 0.0, 0.0]))
    tilt = np.degrees(np.arccos(np.clip(b3[2], -1.0, 1.0)))
    assert tilt == pytest.approx(np.degrees(np.arctan2(6.0, wide.g)), abs=1e-6)
    _b3u, Tu = wide.desired_specific_force(np.array([0.0, 0.0, 100.0]))
    assert Tu == pytest.approx(wide.g + 6.0)
    narrow = SO3LagModel(max_tilt_deg=30, a_max=6)
    b3n, _ = narrow.desired_specific_force(np.array([6.0, 0.0, 0.0]))
    tilt_n = np.degrees(np.arccos(np.clip(b3n[2], -1.0, 1.0)))
    assert tilt_n == pytest.approx(30.0)

    # 默认 yaw_weight=0.4 把偏航角速度压在 200 °/s 以下。权重 1 才会顶到限幅。
    yaw_plan = HoverPlan((0.0, 0.0, 0.0), 1.2)
    limited = SO3LagModel(yaw_weight=1.0)
    s = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    s = limited.step(s, yaw_plan, 0.01)
    cap = np.radians(200.0) * 0.01
    assert abs(s.yaw) == pytest.approx(cap, abs=1e-5)

    pm = PointMassLag()
    far = HoverPlan((4.0, 0.0, 0.0), 0.0)
    pm_s = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    so_s = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    so_m = SO3LagModel()
    for _ in range(30):
        pm_s = pm.step(pm_s, far, 0.01)
        so_s = so_m.step(so_s, far, 0.01)
    assert 0.0 < so_s.p[0] < pm_s.p[0]

    shared = SO3LagModel()
    a = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    b = State(0.0, np.array([3.0, 0.0, 0.0]), np.zeros(3), np.zeros(3), 0.0)
    spin = HoverPlan((0.0, 0.0, 0.0), 0.8)
    hold = HoverPlan((3.0, 0.0, 0.0), 0.0)
    for _ in range(6):
        a = shared.step(a, spin, 0.01)
        a = replace(a, t=round(a.t, 9))
        b = shared.step(b, hold, 0.01)
        b = replace(b, t=round(b.t, 9))
    assert shared.n_vehicles == 2
    assert a.yaw > 0.0
    assert abs(b.yaw) < 1e-9


def test_suite_cards_stay_point_mass():
    for rel in SUITE_CARDS:
        raw = json.loads((ROOT / rel).read_text(encoding="utf-8"))
        assert raw["dynamics"]["model"] == "point_mass_lag"
        assert "so3" not in json.dumps(raw).lower()
        assert raw["criteria"]["frozen"] is True


def test_default_off_is_bitwise_with_explicit_off_and_skips_so3(monkeypatch):
    import yopo_l0.so3 as so3

    def boom(self, *args, **kwargs):
        raise AssertionError("关着的开关不应步进 SO3")

    monkeypatch.setattr(so3.SO3LagModel, "step", boom)
    card = load_card(HOVER)
    kw = dict(seed=1, policy="hover", shadow=False, duration_s=0.2)
    out = run(card, RunConfig(**kw))
    assert out.result["dynamics"] == "point_mass_lag"
    assert "so3" not in out.result
    assert "upstream_deviation" not in out.result

    monkeypatch.undo()
    again = run(card, RunConfig(**kw))
    explicit = run(card, RunConfig(so3=False, **kw))
    np.testing.assert_array_equal(_track(out), _track(again))
    np.testing.assert_array_equal(_track(again), _track(explicit))
    assert set(again.result) == set(explicit.result)
    assert explicit.result["dynamics"] == "point_mass_lag"
    assert "so3" not in explicit.result


def test_runconfig_and_scene_switch_mark_upstream_deviation():
    card = load_card(HOVER)
    kw = dict(seed=0, policy="hover", shadow=False, duration_s=0.2)
    forced = run(card, RunConfig(so3=True, **kw))
    assert forced.result["dynamics"] == "so3_lag"
    assert forced.result["upstream_deviation"] is True
    assert forced.result["so3"]["mark"] == "偏离上游"
    assert forced.result["so3"]["attitude_order"] == 1
    assert forced.result["so3"]["tau_att_s"] == pytest.approx(0.12)
    plain = run(card, RunConfig(**kw))
    # 悬停在平衡点时位置可以很接近。真正拉开的是要动起来的闭环，见下一条。
    assert forced.result["dynamics"] != plain.result["dynamics"]

    raw = dict(card.card)
    raw["dynamics"] = dict(SWITCH_DEFAULTS)
    scene_on = SceneCard(card.path, raw, card.obstacles)
    via_card = run(scene_on, RunConfig(**kw))
    assert via_card.result["dynamics"] == "so3_lag"
    assert via_card.result["upstream_deviation"] is True
    np.testing.assert_array_equal(_track(via_card), _track(forced))
    restored = run(scene_on, RunConfig(so3=False, **kw))
    np.testing.assert_array_equal(_track(restored), _track(plain))
    assert restored.result["dynamics"] == "point_mass_lag"
    assert "so3" not in restored.result
    assert json.loads(HOVER.read_text(encoding="utf-8"))["dynamics"]["model"] == "point_mass_lag"
    with pytest.raises(ValueError):
        run(card, RunConfig(so3=1, **kw))


def test_closed_loop_so3_diverges_from_point_mass():
    card = load_card(PILLAR)
    kw = dict(seed=0, policy="upstream", shadow=False, duration_s=0.3)
    plain = run(card, RunConfig(**kw))
    forced = run(card, RunConfig(so3=True, **kw))
    assert plain.result["dynamics"] == "point_mass_lag"
    assert "so3" not in plain.result
    assert forced.result["upstream_deviation"] is True
    assert not np.array_equal(_track(plain), _track(forced))


def test_script_grid_check_and_resume(tmp_path, monkeypatch):
    mod = _script()
    assert mod.main(["--check"]) == 0
    specs = mod.job_specs()
    assert len(specs) == 93
    assert sum(1 for scene, _seed, tier, _resume in specs if scene == "formation-6-open") == 30
    assert {tier for scene, _seed, tier, _resume in specs if scene == "formation-6-open"} == {
        "none",
        "drop-5",
        "drop-20",
        "duplicate-10",
        "reorder-delay",
        "combined",
    }
    monkeypatch.setattr(mod, "PARTIAL", tmp_path)
    stored = {
        "row_version": mod.ROW_VERSION,
        "params_fingerprint": mod.params_fingerprint(),
        "scene": "g2-pillar",
        "seed": 4,
        "fault_tier": None,
        "arrived": 1,
        "n_drones": 1,
        "outcome": "SUCCESS",
    }
    mod.save_partial(stored)

    def boom(*_args, **_kwargs):
        raise AssertionError("指纹相符的 partial 不应重跑")

    monkeypatch.setattr(mod, "_simulate", boom)
    got = mod.run_one("g2-pillar", 4, None, resume=True)
    assert got["arrived"] == 1
    stale = dict(stored)
    stale["seed"] = 5
    stale["params_fingerprint"] = "0" * 64
    mod.save_partial(stale)
    assert mod.load_partial("g2-pillar", 5, None) is None


def test_gate_report_flags_a_broken_frozen_gate():
    mod = _script()
    row = {
        "scene": "g2-pillar",
        "seed": 0,
        "policy": "upstream",
        "form": "inprocess",
        "forest_density": None,
        "outcome": "TIMEOUT",
        "arrived": 0,
        "n_drones": 1,
        "arrival_time_s": None,
        "task": "goto",
    }
    _gates, failures = mod.evaluate_case(mod.case_by_id("g2-pillar"), [row])
    text = "\n".join(failures)
    assert "arrival_rate" in text
    assert failures
