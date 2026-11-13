"""票 E-C5b：走廊跟踪误差补偿。默认关；关时不读障碍、不改解码器，与主线逐位相同。"""

import importlib.util
import json
import sys

import numpy as np
import pytest

import yopo_l0.policy as policy
from yopo_l0.dynamics import State
from yopo_l0.geom_fallback import SAMPLE_DT_S
from yopo_l0.policy import (
    Observation,
    PolicyContext,
    UpstreamPolicy,
    _TrajPlan,
    read_empty_brake,
    read_track_body_radius_m,
    read_track_clearance,
    read_track_clearance_mode,
    read_track_margin_m,
    reference_surface_clearance,
    track_clearance_wants_neighbors,
)
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import Box, Cylinder, Ground, SceneCard, load_card, parse_geometry
from yopo_l0.sensor import DepthFrame
from yopo_swarm_core.candidates import UPSTREAM_DECODER
from yopo_swarm_core.plan import hold_traj

from conftest import ROOT

_STRIP = set(WALL_FIELDS) | {"commit", "scene_card_hash"}
_LIMIT = 0.35 + 0.08


def test_defaults_off_and_cards_do_not_mention_the_switch():
    assert policy.UPSTREAM_SAFE_RADIUS_M == pytest.approx(0.05)
    assert read_empty_brake({}) is False
    assert read_track_clearance(None) is False
    assert read_track_clearance({}) is False
    assert read_track_clearance({"upstream": {}}) is False
    assert read_track_clearance({"upstream": {"track_clearance": True}}) is True
    assert read_track_clearance({"upstream": {"track_clearance": False}}) is False
    assert read_track_clearance_mode({}) == "geom_multi"
    assert read_track_margin_m({}) == pytest.approx(0.08)
    assert read_track_body_radius_m({}) == pytest.approx(0.35)
    assert policy.TRACK_SAMPLE_DT_S == pytest.approx(SAMPLE_DT_S)
    assert track_clearance_wants_neighbors({}) is False
    assert track_clearance_wants_neighbors({"upstream": {"track_clearance": True}}) is True
    assert track_clearance_wants_neighbors(
        {"upstream": {"track_clearance": True, "track_clearance_mode": "geom"}}
    ) is False
    for path in sorted((ROOT / "scenes").glob("*.json")):
        text = path.read_text(encoding="utf-8")
        for key in ("track_clearance", "track_margin_m", "track_body_radius_m"):
            assert key not in text, path.name
    suite = (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    assert "e-c5b" not in suite and "track_clearance" not in suite


def test_invalid_bool_mode_and_margin_raise():
    for bad in ("true", 1, None):
        with pytest.raises(ValueError):
            read_track_clearance({"upstream": {"track_clearance": bad}})
        with pytest.raises(ValueError):
            UpstreamPolicy(PolicyContext("uav0", 0.01, {"upstream": {"track_clearance": bad}}))
    # 关掉时不读 mode。打开才拒绝非法值。
    UpstreamPolicy(PolicyContext("uav0", 0.01, {"upstream": {"track_clearance_mode": "nope"}}))
    with pytest.raises(ValueError):
        read_track_clearance_mode({"upstream": {"track_clearance_mode": "nope"}})
    with pytest.raises(ValueError):
        UpstreamPolicy(PolicyContext(
            "uav0", 0.01, {"upstream": {"track_clearance": True, "track_clearance_mode": "nope"}},
        ))
    for bad in (-0.01, -1, float("nan"), True):
        with pytest.raises(ValueError):
            read_track_margin_m({"upstream": {"track_margin_m": bad}})
    with pytest.raises(ValueError):
        UpstreamPolicy(PolicyContext(
            "uav0", 0.01, {"upstream": {"track_clearance": True, "track_margin_m": -0.08}},
        ))
    # 0 是非负，允许。负的机体半径不允许。
    ok = UpstreamPolicy(PolicyContext(
        "uav0", 0.01, {"upstream": {"track_clearance": True, "track_margin_m": 0.0, "track_clearance_mode": "pred"}},
    ))
    assert ok._track_margin == pytest.approx(0.0)
    with pytest.raises(ValueError):
        read_track_body_radius_m({"upstream": {"track_body_radius_m": 0.0}})


def test_clearance_matches_scene_distance_and_the_margin_threshold():
    cyl = Cylinder("c", (0.0, 0.0), 0.2, 0.0, 3.0)
    ground = Ground(0.0)
    box = Box("b", (10.0, 10.0, 0.0), (11.0, 12.0, 1.0))
    lean = Cylinder("lean", (3.0, 0.0), 0.2, 0.0, 2.0, lean_xy=(0.4, 0.0))
    obstacles = (cyl, ground, box, lean)
    near = hold_traj(np.array([0.56, 0.0, 1.5]), 0.0, 0.2)
    far = hold_traj(np.array([0.65, 0.0, 1.5]), 0.0, 0.2)
    assert reference_surface_clearance(near, obstacles) == pytest.approx(0.36)
    assert reference_surface_clearance(far, obstacles) == pytest.approx(0.45)
    assert reference_surface_clearance(near, obstacles) < _LIMIT
    assert reference_surface_clearance(far, obstacles) >= _LIMIT
    pts = np.array([
        [0.56, 0.0, 1.5],
        [0.65, 0.1, 0.2],
        [10.5, 11.0, 0.4],
        [3.2, 0.0, 1.0],
        [3.2, 0.0, 3.0],
    ])
    index = policy._ObstacleIndex(obstacles)
    for p in pts:
        expect = min(float(ob.distance(p)) for ob in obstacles)
        assert index.min_clearance(p) == pytest.approx(expect, abs=1e-9)


def _frame():
    return DepthFrame(0.5, np.zeros((4, 4), np.float32), (1.0, 1.0, 1.0, 1.0))


def _obs(p=(0.0, 0.0, 1.5)):
    state = State(0.5, np.array(p, float), np.zeros(3), np.zeros(3), 0.0)
    return Observation(_frame(), state, np.array([30.0, 0.0, 1.5]), 0.5)


def _obstacles():
    return (Cylinder("c", (0.0, 0.0), 0.2, 0.0, 3.0), Ground(0.0))


def _params(mode, **extra):
    upstream = {"track_clearance": True, "track_clearance_mode": mode, "track_margin_m": 0.08}
    upstream.update(extra)
    return {"upstream": upstream}


def _stub(monkeypatch, positions, scores, mu, head0):
    positions = np.asarray(positions, float)
    head0 = np.asarray(head0, float)
    inner_b = positions - head0
    n = len(positions)

    def fake_forward(*_a, **_k):
        return (
            np.zeros((n, 14), np.float32),
            np.asarray(scores, np.float64),
            np.zeros((n, 2), np.float32),
        )

    def fake_decode(_params, _dec):
        return inner_b.copy(), np.zeros((n, 3, 3)), np.ones((n, 2))

    def fake_mu(_radius, _dec):
        return np.asarray(mu, np.float64)

    def fake_minco(t0, _head, _tail, inner, _durations):
        return hold_traj(np.asarray(inner, float).reshape(3), float(t0), float(t0) + 0.2)

    monkeypatch.setattr("yopo_swarm_core.candidates.decode_params", fake_decode)
    monkeypatch.setattr("yopo_swarm_core.candidates.corridor_mu_lo", fake_mu)
    monkeypatch.setattr("yopo_swarm_core.candidates.minco_two_piece", fake_minco)
    return fake_forward


def _lattice(monkeypatch, xs, scores, admitted, head0=(0.0, 0.0, 1.5)):
    """前几个候选按给定净空放置，其余不准入。长度与解码器候选数一致，方便邻机过滤。"""
    n = int(UPSTREAM_DECODER.count)
    positions = np.zeros((n, 3))
    positions[:, 2] = 1.5
    positions[:, 0] = 20.0
    score = np.full(n, -1.0)
    mu = np.zeros(n)
    for i, (x, s, ok) in enumerate(zip(xs, scores, admitted, strict=True)):
        # 圆柱半径 0.2，表面净空 = x - 0.2。
        positions[i, 0] = 0.2 + float(x)
        score[i] = float(s)
        mu[i] = 1.0 if ok else 0.0
    forward = _stub(monkeypatch, positions, score, mu, head0)
    return forward


def _policy(mode, monkeypatch, *, neighbors=False, inter=False, brake=False, xs=(0.36, 0.45), head0=(0.0, 0.0, 1.5)):
    extra = {}
    if inter:
        extra.update(inter_drone_filter=True, inter_drone_threshold_m=0.05, inter_drone_penalty=0)
    if brake:
        extra["empty_brake"] = True
    params = {} if mode is None else _params(mode, **extra)
    obstacles = () if mode is None else _obstacles()
    pol = UpstreamPolicy(PolicyContext("uav0", 0.01, params, obstacles=obstacles, a_max=6.0))
    pol.source.forward_raw = _lattice(
        monkeypatch, xs, scores=(5.0, 1.0), admitted=(True, True), head0=head0,
    )
    if neighbors:
        pol.note_neighbor_samples("uav1", [[0.0, 40.0, 0.0, 1.5], [2.0, 40.0, 0.0, 1.5]])
    return pol


def _at(plan, t=0.5):
    p, _v, _a = plan.pva(t)
    return np.asarray(p, float)


def test_off_does_not_read_obstacles_or_raise_decoder_radius(monkeypatch):
    class Trap:
        def __iter__(self):
            raise AssertionError("开关关着却读了 obstacles")

    pol = UpstreamPolicy(PolicyContext("uav0", 0.01, {}, obstacles=Trap()))
    pol.source.forward_raw = _lattice(monkeypatch, (0.36, 0.45), (5.0, 1.0), (True, True))
    seen = []
    real = policy.upstream_decoder

    def wrapped(radius=None):
        seen.append(None if radius is None else float(radius))
        return real(radius)

    monkeypatch.setattr(policy, "upstream_decoder", wrapped)
    out = pol.plan(_obs())
    assert seen == [pytest.approx(0.05)]
    assert pol.last_code == "OK"
    # 最高分是 0.36 m 那条。关着时几何门不存在，不会改成 0.45 m。
    np.testing.assert_allclose(_at(out), [0.56, 0.0, 1.5], atol=1e-9)
    pred = UpstreamPolicy(PolicyContext(
        "uav0", 0.01, _params("pred"), obstacles=Trap(),
    ))
    assert pred._track_index is None


def test_geom_rejects_036_and_accepts_045(monkeypatch):
    pol = _policy("geom", monkeypatch)
    out = pol.plan(_obs())
    assert pol.last_code == "OK"
    np.testing.assert_allclose(_at(out), [0.65, 0.0, 1.5], atol=1e-9)

    # 参考头在 x=1，候选净空仍按这个头锚定，避免 inner 被平移后变成合格。
    blocked = _policy("geom", monkeypatch, xs=(0.36, 0.36), brake=True, head0=(1.0, 0.0, 1.5))
    blocked.plan_obj = _TrajPlan(hold_traj(np.array([1.0, 0.0, 1.5]), 0.0, 2.0), np.array([30.0, 0, 1.5]), 0.0, 0.01)
    out = blocked.plan(_obs())
    assert blocked.last_code == "CORRIDOR_EMPTY_HOVER"
    # 无解走 empty_brake：停在参考头，不拽回里程计。
    np.testing.assert_allclose(_at(out), [1.0, 0.0, 1.5], atol=1e-9)

    hover = _policy("geom", monkeypatch, xs=(0.36, 0.36), head0=(1.0, 0.0, 1.5))
    hover.plan_obj = _TrajPlan(hold_traj(np.array([1.0, 0.0, 1.5]), 0.0, 2.0), np.array([30.0, 0, 1.5]), 0.0, 0.01)
    out = hover.plan(_obs())
    assert hover.last_code == "CORRIDOR_EMPTY_HOVER"
    np.testing.assert_allclose(_at(out), [0.0, 0.0, 1.5], atol=1e-9)


def test_geom_multi_uses_neighbors_and_pred_only_raises_decoder_radius(monkeypatch):
    solo = _policy("geom_multi", monkeypatch)
    solo.note_neighbor_samples("uav1", [[0.0, 1, 2, 3], [0.1, 1, 2, 3]])
    assert "uav1" in solo._neighbors
    solo._neighbors.clear()
    out = solo.plan(_obs())
    assert solo.last_code == "OK"
    np.testing.assert_allclose(_at(out), [0.56, 0.0, 1.5], atol=1e-9)

    multi = _policy("geom_multi", monkeypatch, neighbors=True)
    out = multi.plan(_obs())
    assert multi.last_code == "OK"
    np.testing.assert_allclose(_at(out), [0.65, 0.0, 1.5], atol=1e-9)

    # geom（不是 geom_multi）在没有 E-A1 时不收邻机采样。
    geom = _policy("geom", monkeypatch)
    geom.note_neighbor_samples("uav1", [[0.0, 1, 2, 3], [0.1, 1, 2, 3]])
    assert geom._neighbors == {}

    seen = []
    real = policy.upstream_decoder

    def wrapped(radius=None):
        seen.append(float(radius))
        return real(radius)

    monkeypatch.setattr(policy, "upstream_decoder", wrapped)
    pred = _policy("pred", monkeypatch)
    out = pred.plan(_obs())
    assert seen == [pytest.approx(0.13)]
    assert pred.last_code == "OK"
    # pred 不看几何，仍飞最高分的 0.36 m。
    np.testing.assert_allclose(_at(out), [0.56, 0.0, 1.5], atol=1e-9)
    assert pred._track_index is None


def test_geom_filter_runs_before_inter_and_does_not_revive_rejected(monkeypatch):
    """0.36 m 最高分被几何门去掉。剩下的 0.45 m 贴着邻机，E-A1 再滤空，走无解。

    邻机阈值 0.05 m，盖不住 0.36 m 与 0.45 m 之间的 0.09 m。若几何门没先筛，
    邻机过滤会留下 0.36 m 那条。
    """
    pol = _policy("geom", monkeypatch, inter=True, neighbors=False)
    # 邻机放在 0.45 m 净空那个点上（x = 0.65）。
    pol.note_neighbor_samples("uav1", [[0.0, 0.65, 0.0, 1.5], [2.0, 0.65, 0.0, 1.5]])
    out = pol.plan(_obs())
    assert pol.last_code == "CORRIDOR_EMPTY_HOVER"
    np.testing.assert_allclose(_at(out), _obs().state.p, atol=1e-9)
    assert pol._inter_empty == 1


def _memory_card(path, upstream):
    raw = json.loads(path.read_text(encoding="utf-8"))
    params = dict(raw.get("policy_params") or {})
    block = dict(params.get("upstream") or {})
    block.update(upstream)
    params["upstream"] = block
    raw["policy_params"] = params
    geom = raw["geometry"]
    if isinstance(geom, str):
        raw["geometry"] = json.loads((path.parent / geom).read_text(encoding="utf-8"))
    return SceneCard(raw["name"], raw, parse_geometry(raw["geometry"], raw.get("ground_z")))


def _result(out):
    return {k: v for k, v in out.result.items() if k not in _STRIP}


def test_switch_off_matches_mainline_ticks():
    path = ROOT / "scenes/ego-density-014-solo.json"
    cfg = RunConfig(seed=0, duration_s=0.4, shadow=False)
    plain = run(load_card(path), cfg)
    explicit = run(_memory_card(path, {
        "track_clearance": False,
        "track_clearance_mode": "nope",
        "track_margin_m": -1.0,
    }), cfg)
    assert "track_clearance" not in plain.result
    assert plain.result["upstream_deviation"] is False
    assert plain.result["safe_radius_m"] == pytest.approx(0.05)
    assert "empty_brake" not in plain.result
    assert plain.ticks == explicit.ticks
    assert _result(plain) == _result(explicit)


def test_runner_marks_deviation_only_when_on_and_geom_multi_hears_neighbors(monkeypatch):
    path = ROOT / "scenes/open-3drone.json"
    cfg = RunConfig(seed=0, duration_s=0.2, shadow=False)
    off = run(load_card(path), cfg).result
    assert "track_clearance" not in off and off["upstream_deviation"] is False
    on = run(_memory_card(path, {
        "track_clearance": True,
        "track_clearance_mode": "geom",
        "track_margin_m": 0.08,
    }), cfg).result
    assert on["track_clearance"] is True
    assert on["track_clearance_mode"] == "geom"
    assert on["track_margin_m"] == pytest.approx(0.08)
    assert on["track_body_radius_m"] == pytest.approx(0.35)
    assert on["safe_radius_m"] == pytest.approx(0.05)
    assert on["upstream_deviation"] is True

    calls = {"n": 0}
    orig = UpstreamPolicy.note_neighbor_samples

    def wrapped(self, src, samples):
        calls["n"] += 1
        return orig(self, src, samples)

    monkeypatch.setattr(UpstreamPolicy, "note_neighbor_samples", wrapped)
    run(load_card(path), RunConfig(seed=0, duration_s=0.05, shadow=False))
    assert calls["n"] == 0
    calls["n"] = 0
    run(_memory_card(path, {
        "track_clearance": True,
        "track_clearance_mode": "geom_multi",
        "track_margin_m": 0.08,
    }), RunConfig(seed=0, duration_s=0.05, shadow=False))
    assert calls["n"] > 0


def _script():
    path = ROOT / "scripts/e-c5b-track-clearance.py"
    spec = importlib.util.spec_from_file_location("e_c5b_track_clearance", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_script_check_labels_and_does_not_touch_suite():
    mod = _script()
    assert mod.ARMS == ("off", "base", "pred", "geom", "geom_multi")
    assert mod.SEEDS == tuple(range(10))
    assert mod.arm_deviates("off") is False
    assert all(mod.arm_deviates(arm) for arm in mod.ARMS if arm != "off")
    assert "偏离上游" not in mod.arm_label("off")
    for arm in ("base", "pred", "geom", "geom_multi"):
        assert "偏离上游" in mod.arm_label(arm)
    assert mod.decoder_radius_m("off") == pytest.approx(0.05)
    assert mod.decoder_radius_m("pred") == pytest.approx(0.13)
    assert mod.decoder_radius_m("geom") == pytest.approx(0.05)
    assert len(mod.job_specs(list(mod.SCENE_ORDER), list(mod.ARMS), list(mod.SEEDS))) == 100
    assert mod.main(["--check"]) == 0
    with pytest.raises(SystemExit):
        mod.main(["--jobs", "5", "--seeds", "0"])
    tickets = (ROOT / "docs/tickets.md").read_text(encoding="utf-8")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "E-C5b" not in tickets and "E-C5b" not in readme


def test_committed_runs_match_tables_when_present():
    path = ROOT / "docs/research/e-c5b-track-clearance-runs.json"
    md_path = ROOT / "docs/research/e-c5b-track-clearance.md"
    if not path.is_file() or not md_path.is_file():
        pytest.skip("报告还没写入")
    mod = _script()
    data = json.loads(path.read_text(encoding="utf-8"))
    rows = data["rows"]
    assert len(rows) == 100
    assert data["disclaimer"]
    assert data["report_only"] is True and data["frozen"] is False
    md = md_path.read_text(encoding="utf-8")
    assert mod.render_summary(rows) in md
    assert mod.render_tree_counts(rows) in md
    assert mod.render_tree_details(rows) in md
    assert mod.render_seeds(rows) in md
    assert "只报数" in md and "未冻结" in md and "偏离上游" in md
    for row in rows:
        assert row["scene"] in mod.SCENE_ORDER
        assert row["arm"] in mod.ARMS
        assert row["seed"] in range(10)
        assert row["report_only"] is True and row["frozen"] is False
        assert bool(row["upstream_deviation"]) is mod.arm_deviates(row["arm"])
        assert float(row["safe_radius_m"]) == pytest.approx(0.05)
        if int(row["tree_collision"]):
            assert row["attribution"]["category"] in mod.load_ec5().TREE_ORDER
