"""票 E-B4d：欧氏球没推开 uav1-uav2 的只读诊断。不改后段。"""

import importlib.util
import json
import sys

import numpy as np
import pytest

import yopo_l0.backend.bspline as bspline
import yopo_l0.crossing as crossing
import yopo_l0.policy as policy
import yopo_l0.runner as runner

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-b4d-sphere-miss.py"
    spec = importlib.util.spec_from_file_location("e_b4d_sphere_miss", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _hinge(**kwargs):
    base = {
        "measured": True,
        "activated": False,
        "cost": 0.0,
        "min_dist_m": None,
        "free_shift_m": 0.0,
        "end_shift_m": 0.0,
        "end_pinned": True,
        "closest_on_excluded_last_span": False,
        "pinned_grad_norm": 0.0,
    }
    base.update(kwargs)
    return base


def test_detection_sees_the_merged_switch():
    mod = _script()
    support = mod.detect_support()
    assert support["supported"] is True
    assert support["config_has_peer_sphere_enabled"] is True
    assert support["config_from_params_honors_flag"] is True
    assert support["refine_yopo_reads_peer_sphere"] is True
    assert support["peer_sphere_cost_grad"] is True
    assert "def _peer_sphere_cost_grad" not in (ROOT / "scripts/e-b4d-sphere-miss.py").read_text()


def test_partial_skips_flight_even_when_the_switch_exists(monkeypatch):
    mod = _script()

    def boom(*_a, **_k):
        raise AssertionError("不应实飞")

    monkeypatch.setattr(runner, "run", boom)
    report = mod.build_partial_report()
    assert report["report_only"] is True
    assert report["status"] == "partial_smoke"
    assert report["flight"] == "skipped"
    assert report["cases"] == []
    assert report["comparison"] is None
    assert report["blocked_on"] is None
    assert [q["status"] for q in report["questions"]] == ["deferred"] * 4
    assert all(q["answer"] is None for q in report["questions"])
    smoke = report["partial_smoke"]
    assert smoke["scene_files_unchanged"] is True
    assert smoke["pair_present"] is True
    assert smoke["uav_ids"] == ["uav0", "uav1", "uav2"]
    assert smoke["body_radius_m"] == pytest.approx(0.35)
    assert smoke["collision_line_m"] == pytest.approx(0.70)
    assert smoke["runner_plan_before_poll"] is True
    assert smoke["runner_plan_before_poll_is_static"] is True
    sphere = smoke["in_memory_arms"]["sphere"]
    geom = smoke["in_memory_arms"]["sphere+track_geom_multi"]
    assert sphere["peer_sphere_enabled"] is True
    assert sphere["peer_sphere_radius_m"] == pytest.approx(0.85)
    assert sphere["peer_sphere_weight"] == pytest.approx(8.0)
    assert sphere["swarm_penalty"] is False
    assert sphere["track_clearance"] is False
    assert geom["track_clearance"] is True and geom["track_clearance_mode"] == "geom_multi"
    cited = report["cited_e_b4c"]
    assert cited["remeasured_here"] is False
    assert cited["sphere"]["seed1"]["min_inter_m"] == "0.694"
    assert cited["sphere"]["seed1"]["reason"] == "uav1-uav2:INTER_DRONE"


def test_partial_marks_missing_when_switch_is_absent(monkeypatch):
    mod = _script()
    monkeypatch.setattr(runner, "run", lambda *_a, **_k: (_ for _ in ()).throw(AssertionError("flew")))
    dead = dict(mod.detect_support())
    dead["supported"] = False
    report = mod.build_partial_report(dead)
    assert report["status"] == "missing_feature"
    assert report["flight"] == "skipped"
    assert report["blocked_on"]["pr"] == 94
    assert report["questions"][0]["status"] == "deferred"


def test_partial_still_skips_when_support_flag_is_forced(monkeypatch):
    mod = _script()
    monkeypatch.setattr(runner, "run", lambda *_a, **_k: (_ for _ in ()).throw(AssertionError("flew")))
    support = dict(mod.detect_support())
    support["supported"] = True
    report = mod.build_partial_report(support)
    assert report["status"] == "partial_smoke"
    assert report["flight"] == "skipped"
    assert report["cases"] == []
    assert report["questions"][0]["status"] == "deferred"


def test_flight_refuses_when_detection_says_missing(monkeypatch):
    mod = _script()
    monkeypatch.setattr(runner, "run", lambda *_a, **_k: (_ for _ in ()).throw(AssertionError("flew")))
    dead = dict(mod.detect_support())
    dead["supported"] = False
    monkeypatch.setattr(mod, "detect_support", lambda: dead)
    with pytest.raises(mod.MissingFeature):
        mod.diagnose_flight()


def test_cards_do_not_touch_scene_files():
    mod = _script()
    path = ROOT / mod.SCENE_PATH
    geom = ROOT / "scenes" / json.loads(path.read_text())["geometry"]
    before = path.read_bytes()
    geom_before = geom.read_bytes()
    card = mod.build_card("sphere")
    assert card.card["policy_params"]["backend"]["peer_sphere_enabled"] is True
    assert path.read_bytes() == before
    assert geom.read_bytes() == geom_before
    with pytest.raises(ValueError):
        mod.build_card("off")


def test_install_restores_hooks():
    mod = _script()
    before = (
        policy.UpstreamPolicy.plan,
        policy.UpstreamPolicy.note_neighbor_samples,
        policy.UpstreamPolicy._apply_backend,
        policy.UpstreamPolicy._keep_neighbor_samples,
        runner.swarm_penalty_enabled,
        bspline.refine_yopo,
        crossing.CrossingFleet._apply_backend,
    )
    restore = mod._Restore()
    try:
        mod._install(restore, mod._Log())
        assert policy.UpstreamPolicy.plan is not before[0]
        assert bspline.refine_yopo is not before[5]
    finally:
        restore.restore()
    after = (
        policy.UpstreamPolicy.plan,
        policy.UpstreamPolicy.note_neighbor_samples,
        policy.UpstreamPolicy._apply_backend,
        policy.UpstreamPolicy._keep_neighbor_samples,
        runner.swarm_penalty_enabled,
        bspline.refine_yopo,
        crossing.CrossingFleet._apply_backend,
    )
    assert after == before


def test_locus_head_mid_tail_and_hover_hold():
    mod = _script()
    assert mod.locus_of(0.0, 0.0, 3.0, "OK") == "head"
    assert mod.locus_of(1.5, 0.0, 3.0, "OK") == "mid"
    assert mod.locus_of(2.5, 0.0, 3.0, "OK") == "tail"
    assert mod.locus_of(1.0, 1.0, 1.001, "CORRIDOR_EMPTY_HOVER") == "hover_hold"
    assert mod.locus_of(1.5, 1.0, 2.0, "CORRIDOR_EMPTY_HOVER") == "mid"


def test_closest_span_finds_planted_neighbor_and_last_span():
    mod = _script()
    q = np.zeros((9, 3))
    q[:, 0] = np.linspace(0.0, 8.0, 9)
    q[:, 2] = 1.5
    dt = 0.25
    close = mod.closest_span(q, dt, 0.0, [])
    assert close["dist_m"] is None
    windows = bspline._control_windows(q)
    pts = np.einsum("kb,sbc->skc", bspline._B_SEGMENT, windows)
    times = (np.arange(6, dtype=float)[:, None] + bspline._SEGMENT_U[None, :]) * dt
    span, k = 1, 3
    p = pts[span, k]
    t = float(times[span, k])
    nb = np.array([[t, p[0], p[1] + 0.4, p[2]]])
    hit = mod.closest_span(q, dt, 0.0, [nb])
    assert hit["span"] == span
    assert hit["dist_m"] == pytest.approx(0.4, abs=1e-6)
    assert hit["on_excluded_last_span"] is False
    last = 5
    p = pts[last, k]
    t = float(times[last, k])
    nb = np.array([[t, p[0], p[1], p[2]]])
    hit = mod.closest_span(q, dt, 0.0, [nb])
    assert hit["span"] == last
    assert hit["on_excluded_last_span"] is True
    assert hit["dist_m"] == pytest.approx(0.0, abs=1e-6)
    hidden = mod.closest_span(q, dt, 0.0, [nb], exclude_last=True)
    assert hidden["on_excluded_last_span"] is False
    assert hidden["span"] != last


def test_polynomial_gap_is_time_aligned():
    mod = _script()

    class Hold:
        t_start = 0.0
        t_end = 1.0

        def evaluate(self, _t, _k):
            return np.array([0.0, 0.0, 1.5])

    nb = np.array([[0.0, 0.3, 0.4, 1.5], [1.0, 0.3, 0.4, 1.5]])
    assert mod.polynomial_gap(Hold(), [nb]) == pytest.approx(0.5)
    assert mod.polynomial_gap(Hold(), []) is None
    later = np.array([[5.0, 0.0, 0.0, 1.5]])
    assert mod.polynomial_gap(Hold(), [later]) is None


def test_neighbor_summary_sees_empty_first_tick_and_later_samples():
    mod = _script()
    plans = [
        {"t": 0.0, "uav": "uav1", "n_neighbors": 0, "is_first": True},
        {"t": 0.0, "uav": "uav2", "n_neighbors": 0, "is_first": True},
        {"t": 1.2, "uav": "uav1", "n_neighbors": 2, "is_first": False},
        {"t": 1.2, "uav": "uav2", "n_neighbors": 2, "is_first": False},
    ]
    refines = [
        {"t": 0.0, "n_neighbors": 0},
        {"t": 1.2, "n_neighbors": 2},
    ]
    notes = [{"n_plans_done": 2}]
    block = mod.summarize_neighbors(plans, refines, notes, 1.5)
    assert block["first_plan_missing_neighbors"] is True
    assert block["first_plan_t"] == pytest.approx(0.0)
    assert block["notes_before_any_plan"] == 0
    assert block["backend_received_neighbors"] is True
    assert block["window_refine_with_neighbors"] == 1
    assert block["window_refine_calls"] == 1
    assert "第 0 个规划拍没有邻机" in block["reading"]
    assert "收到邻机" in block["reading"]


def test_hinge_summary_flags_pinned_tail_and_a_real_push():
    mod = _script()
    blocked = mod.summarize_hinge([{
        "hinge": _hinge(
            activated=True, cost=3.5, min_dist_m=0.62, free_shift_m=0.01,
            end_shift_m=0.0, closest_on_excluded_last_span=True, pinned_grad_norm=0.2,
        ),
    }])
    assert blocked["activated"] is True
    assert blocked["min_dist_when_active_m"] == pytest.approx(0.62)
    assert blocked["max_cost"] == pytest.approx(3.5)
    assert blocked["end_pinned"] is True
    assert blocked["pinned_end_blocks_push"] is True
    assert "最后一跨" in blocked["reading"]

    leaked = mod.summarize_hinge([{
        "hinge": _hinge(
            activated=True, cost=6.9, min_dist_m=0.24, free_shift_m=0.87,
            end_shift_m=0.0, closest_on_excluded_last_span=False, pinned_grad_norm=1e-4,
        ),
    }])
    assert leaked["end_pinned"] is True
    assert leaked["pinned_end_blocks_push"] is False

    pushed = mod.summarize_hinge([{
        "hinge": _hinge(
            activated=True, cost=1.0, min_dist_m=0.66, free_shift_m=0.2,
            end_shift_m=0.0, closest_on_excluded_last_span=False, pinned_grad_norm=0.0,
        ),
    }])
    assert pushed["pinned_end_blocks_push"] is False
    assert pushed["max_free_shift_m"] == pytest.approx(0.2)

    quiet = mod.summarize_hinge([{"hinge": {"measured": False}}])
    assert quiet["activated"] is False
    assert quiet["pinned_end_blocks_push"] is False


def test_collision_summary_splits_hover_and_incoming():
    mod = _script()
    plans = [
        {"t": 0.0, "uav": "uav1", "code": "OK", "traj_t0": 0.0, "traj_t1": 2.0, "is_first": True, "n_neighbors": 0},
        {"t": 0.0, "uav": "uav2", "code": "OK", "traj_t0": 0.0, "traj_t1": 2.0, "is_first": True, "n_neighbors": 0},
        {"t": 1.2, "uav": "uav1", "code": "CORRIDOR_EMPTY_HOVER", "traj_t0": 1.2, "traj_t1": 1.201, "is_first": False, "n_neighbors": 2},
        {"t": 1.2, "uav": "uav2", "code": "OK", "traj_t0": 0.4, "traj_t1": 2.2, "is_first": False, "n_neighbors": 2},
    ]
    ticks = [
        {"t": 1.49, "uav": "uav1", "p": [0.0, 0.0, 1.5]},
        {"t": 1.49, "uav": "uav2", "p": [0.9, 0.0, 1.5]},
        {"t": 1.50, "uav": "uav1", "p": [0.0, 0.0, 1.5]},
        {"t": 1.50, "uav": "uav2", "p": [0.6, 0.0, 1.5]},
    ]
    event = mod.attach_plans(mod.pair_event(ticks), plans)
    block = mod.summarize_collision(event, empty_brake=False)
    assert block["collided"] is True
    assert block["t"] == pytest.approx(1.50)
    assert block["dist_m"] == pytest.approx(0.6)
    assert block["locus"]["uav1"] == "hover_hold"
    assert block["locus"]["uav2"] == "mid"
    assert block["one_side_hover_other_flying"] is True
    assert block["hover_uav"] == "uav1"
    assert block["flying_uav"] == "uav2"
    assert block["empty_brake_enabled"] is False
    assert "empty_brake 关着" in block["reading"]


def test_geom_summary_flags_admitted_reference_inside_the_ball():
    mod = _script()
    refines = [
        {"t": 0.0, "geom_on": False, "pre_ref_min_m": 0.40},
        {"t": 1.2, "geom_on": True, "pre_ref_min_m": 0.55},
        {"t": 1.3, "geom_on": True, "pre_ref_min_m": 0.91},
    ]
    block = mod.summarize_geom(refines, refines[1:])
    assert block["gated_reference_already_below_ball"] is True
    assert block["gated_min_reference_m"] == pytest.approx(0.55)
    assert block["gated_n_below_ball"] == 1
    assert block["any_reference_min_m"] == pytest.approx(0.40)
    assert "已经 < 0.70 m" in block["reading"]
    clear = mod.summarize_geom(
        [{"t": 1.0, "geom_on": True, "pre_ref_min_m": 0.88}],
        [{"t": 1.0, "geom_on": True, "pre_ref_min_m": 0.88}],
    )
    assert clear["gated_reference_already_below_ball"] is False
    idle = mod.summarize_geom([{"t": 0.0, "geom_on": False, "pre_ref_min_m": 0.2}], [])
    assert idle["gated_samples"] == 0
    assert "没有打开" in idle["reading"]


def test_answers_for_uses_the_collision_window():
    mod = _script()
    log = mod._Log()
    log.plans = [
        {"t": 0.0, "uav": "uav1", "n_neighbors": 0, "is_first": True, "code": "OK", "traj_t0": 0.0, "traj_t1": 2.0},
        {"t": 0.0, "uav": "uav2", "n_neighbors": 0, "is_first": True, "code": "OK", "traj_t0": 0.0, "traj_t1": 2.0},
        {"t": 1.2, "uav": "uav1", "n_neighbors": 2, "is_first": False, "code": "OK", "traj_t0": 1.2, "traj_t1": 3.0},
        {"t": 1.2, "uav": "uav2", "n_neighbors": 2, "is_first": False, "code": "OK", "traj_t0": 1.2, "traj_t1": 3.0},
    ]
    log.refines = [
        {"t": 0.0, "n_neighbors": 0, "geom_on": False, "pre_ref_min_m": None, "hinge": _hinge()},
        {
            "t": 1.2, "n_neighbors": 2, "geom_on": True, "pre_ref_min_m": 0.50,
            "hinge": _hinge(activated=True, cost=2.0, min_dist_m=0.64, free_shift_m=0.05),
        },
    ]
    log.notes = [{"n_plans_done": 2}]
    ticks = [
        {"t": 1.4, "uav": "uav1", "p": [0.0, 0.0, 1.0]},
        {"t": 1.4, "uav": "uav2", "p": [0.69, 0.0, 1.0]},
    ]
    answers = mod.answers_for(log, ticks, empty_brake=False)
    assert answers["1"]["first_plan_missing_neighbors"] is True
    assert answers["1"]["backend_received_neighbors"] is True
    assert answers["2"]["activated"] is True
    assert answers["2"]["min_dist_when_active_m"] == pytest.approx(0.64)
    assert answers["3"]["collided"] is True
    assert answers["3"]["locus"]["uav1"] == "head"
    assert answers["4"]["gated_reference_already_below_ball"] is True


def test_committed_report_is_the_diagnosed_run():
    mod = _script()
    path = ROOT / "docs/research/e-b4d-sphere-miss-runs.json"
    md_path = ROOT / "docs/research/e-b4d-sphere-miss.md"
    data = json.loads(path.read_text())
    assert data["report_only"] is True
    assert data["status"] == "diagnosed"
    assert data["flight"] == "ran"
    assert data["feature"]["supported"] is True
    assert data["partial_smoke"] is None
    assert [q["status"] for q in data["questions"]] == ["answered"] * 4
    assert len(data["cases"]) == 4
    by = {(c["arm"], c["seed"]): c for c in data["cases"]}
    miss = by[("sphere", 1)]
    ref = by[("sphere", 0)]
    geom = by[("sphere+track_geom_multi", 1)]
    assert miss["outcome"] == "COLLISION"
    assert miss["reason"] == "uav1-uav2:INTER_DRONE"
    assert miss["answers"]["3"]["collided"] is True
    assert ref["role"] == "success_reference"
    assert geom["outcome"] == "COLLISION"
    assert geom["reason"] == "uav1-uav2:INTER_DRONE"
    for key in ("1", "2", "3", "4"):
        assert miss["answers"][key]["reading"]
    text = md_path.read_text()
    rendered = mod.render_markdown(data)
    # 原文仍是已提交诊断的渲染结果。L0-DET 重跑只加在文末，不改原结论段。
    assert text.startswith(rendered)
    assert "\n## L0-DET 确定性重跑（2026-11-15）\n" in text
    assert text.startswith("# 票 E-B4d")
    for phrase in ("report-only", "只报数", "uav1-uav2", "0.694", "peer_sphere_enabled"):
        assert phrase in text
    assert "Co-authored-by" not in text


def test_diagnosed_markdown_quotes_readings():
    mod = _script()
    case = {
        "arm": "sphere",
        "seed": 1,
        "outcome": "COLLISION",
        "reason": "uav1-uav2:INTER_DRONE",
        "min_inter_drone_distance_m": 0.694,
        "answers": {
            "1": {"reading": "第 0 个规划拍没有邻机。"},
            "2": {"reading": "铰链激活 1 次。"},
            "3": {"reading": "碰撞 t=1.50 s。", "collided": True},
            "4": {"reading": "几何门放行的参考轨迹有 1 次同时刻距离已经 < 0.70 m。"},
        },
    }
    other = {
        "arm": "sphere",
        "seed": 0,
        "outcome": "SUCCESS",
        "reason": "GOAL_REACHED_AND_HELD",
        "min_inter_drone_distance_m": 0.946,
        "answers": {str(i): {"reading": "seed0", "collided": False} for i in range(1, 5)},
    }
    geom1 = dict(case)
    geom1["arm"] = "sphere+track_geom_multi"
    geom0 = dict(other)
    geom0["arm"] = "sphere+track_geom_multi"
    cases = [other, case, geom0, geom1]
    report = {
        "ticket": "E-B4d",
        "report_only": True,
        "disclaimer": mod.DISCLAIMER,
        "status": "diagnosed",
        "flight": "ran",
        "questions": mod._question_rows("diagnosed", None, cases),
        "comparison": {
            "sphere": {
                "seed1_outcome": "COLLISION",
                "seed1_min_inter_m": 0.694,
                "seed0_outcome": "SUCCESS",
                "seed0_min_inter_m": 0.946,
            },
        },
        "partial_smoke": None,
    }
    text = mod.render_markdown(report)
    assert "report-only" in text
    assert "铰链激活 1 次" in text
    assert "第 0 个规划拍没有邻机" in text
    assert "0.694" in text
