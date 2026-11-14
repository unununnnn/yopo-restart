"""票 E-C8：脱困触发、临时目标边界、六机卡不改阈值。不起完整仿真。"""

import importlib.util
import json
import sys

import numpy as np
import pytest

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-c8-escape-replan.py"
    spec = importlib.util.spec_from_file_location("e_c8_escape_replan", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _watched():
    mod = _script()
    return [ROOT / rel for rel in mod._WATCHED]


def test_escape_fires_at_n_and_releases_when_the_corridor_reopens():
    mod = _script()
    state = mod.fresh_escape_state()
    for _ in range(9):
        state, fire, release = mod.observe(state, mod.HOVER, n=10, arrived=False)
        assert fire is False and release is False
    assert state["streak"] == 9
    state, fire, release = mod.observe(state, mod.HOVER, n=10, arrived=False)
    assert fire is True and release is False
    state = mod.commit_fire(state, mod.HOVER, k=10)
    assert state["active"] is True
    assert state["hold"] == 9
    state, fire, release = mod.observe(state, "OK", n=10, arrived=False)
    assert fire is False and release is True
    assert state["active"] is False and state["streak"] == 0 and state["hold"] == 0


def test_failed_escape_can_fire_again_only_after_the_hold():
    mod = _script()
    state = mod.fresh_escape_state()
    state, fire, _release = mod.observe(state, mod.HOVER, n=2, arrived=False)
    assert fire is False
    state, fire, _release = mod.observe(state, mod.HOVER, n=2, arrived=False)
    assert fire is True
    state = mod.commit_fire(state, mod.HOVER, k=3)
    state, fire, release = mod.observe(state, mod.HOVER, n=2, arrived=False)
    assert fire is False and release is False and state["hold"] == 1
    state, fire, release = mod.observe(state, mod.HOVER, n=2, arrived=False)
    assert release is True and fire is False and state["active"] is False
    state, fire, release = mod.observe(state, mod.HOVER, n=2, arrived=False)
    assert fire is True and release is False
    opened = mod.commit_fire(state, "OK", k=3)
    assert opened == {"streak": 0, "hold": 0, "active": False}


def test_arrival_does_not_start_an_escape():
    mod = _script()
    state = {"streak": 40, "hold": 0, "active": False}
    state, fire, release = mod.observe(state, mod.HOVER, n=10, arrived=True)
    assert fire is False and release is False


def test_escape_goal_is_a_bounded_offset_outside_the_arrive_radius():
    mod = _script()
    p = np.array([1.0, 2.0, 1.5])
    before = p.copy()
    goal, info = mod.choose_escape_goal(p, 0.0, None, None, alternate=0)
    assert np.array_equal(p, before)
    assert info["range_m"] > 1.5
    assert info["forward_m"] == pytest.approx(mod.FORWARD_M)
    assert info["lateral_m"] == pytest.approx(mod.LATERAL_M)
    assert info["up_m"] == pytest.approx(0.0)
    assert goal[1] < p[1]

    left, left_info = mod.choose_escape_goal(p, 0.0, None, None, alternate=1)
    assert left[1] > p[1]
    assert left_info["lateral_m"] == pytest.approx(mod.LATERAL_M)

    depth = np.full((96, 160), 1.0)
    depth[:, 128:] = 20.0
    intrinsics = (80.0, 80.0, 80.0, 48.0)
    side, side_info = mod.choose_escape_goal(p, 0.0, depth, intrinsics, alternate=0)
    assert side_info["center"] is False
    assert side_info["sector"] == 4
    assert mod.LATERAL_M - 1e-9 <= side_info["lateral_m"] <= mod.LATERAL_CAP_M + 1e-9
    assert side[1] < p[1]
    assert side_info["range_m"] > 1.5
    assert side_info["up_m"] == pytest.approx(0.0)

    lifted = np.full((96, 160), 5.0)
    lifted[:32, :] = 20.0
    up_goal, up_info = mod.choose_escape_goal(p, 0.0, lifted, intrinsics, alternate=0)
    assert up_info["prefer_up"] is True
    assert up_info["up_m"] == pytest.approx(mod.UP_M)
    assert up_goal[2] == pytest.approx(p[2] + mod.UP_M)
    assert mod.LATERAL_M - 1e-9 <= up_info["lateral_m"] <= mod.LATERAL_CAP_M + 1e-9


def test_keep_ahead_pushes_a_close_goal_outside_the_arrive_radius():
    mod = _script()
    p = np.array([0.0, 0.0, 1.5])
    near = p + np.array([1.0, 0.0, 0.0])
    pushed = mod.keep_ahead(p, near, 0.0)
    assert float(np.linalg.norm(pushed - p)) == pytest.approx(mod.ARRIVE_GUARD_M)
    assert pushed[0] > p[0]
    far = p + np.array([4.0, -1.2, 0.0])
    assert np.allclose(mod.keep_ahead(p, far, 0.0), far)


def test_planner_hover_count_sums_drones_on_planner_ticks_only():
    mod = _script()
    from yopo_l0.runner import _due

    for k in range(30):
        assert mod.planner_due(k) is _due(k, 10, 100)
    ticks = []
    for k in range(20):
        for uav, code in (("uav0", mod.HOVER), ("uav1", "OK" if k < 10 else mod.HOVER)):
            ticks.append({"k": k, "uav": uav, "plan_code": code})
    # 规划拍是 k=0 和 k=10。uav0 两拍都悬停，uav1 只在 k=10 悬停。
    assert mod.planner_hover_count(ticks) == 3
    assert mod.planner_hover_count([]) == 0


def test_cards_keep_geom_thresholds_and_do_not_touch_the_tree():
    mod = _script()
    before = {path: path.read_bytes() for path in _watched()}
    plain = mod.build_card(False)
    sphere = mod.build_card(True)
    ec6 = mod.load_ec6()
    base_up = ec6.build_card("d014-swap6", "geom").card["policy_params"]["upstream"]
    assert plain.card["policy_params"]["upstream"] == base_up
    assert sphere.card["policy_params"]["upstream"] == base_up
    assert "backend" not in plain.card["policy_params"]
    backend = sphere.card["policy_params"]["backend"]
    assert backend["enabled"] is True
    assert backend["peer_sphere_enabled"] is True
    assert backend["peer_sphere_radius_m"] == pytest.approx(0.85)
    assert backend["peer_sphere_weight"] == pytest.approx(8.0)
    assert backend.get("swarm_penalty") is not True
    assert len(plain.card["drones"]) == 6
    assert not (ROOT / "scenes/ego-density-014-swap6.json").exists()
    after = {path: path.read_bytes() for path in before}
    assert after == before


def test_partial_resume_skips_simulation(tmp_path, monkeypatch):
    mod = _script()
    monkeypatch.setattr(mod, "PARTIAL", tmp_path)
    stored = {
        "row_version": mod.ROW_VERSION,
        "scene": "d014-swap6",
        "arm": "n15",
        "seed": 2,
        "arrived": 5,
        "n_drones": 6,
        "obstacle_collision": 0,
        "inter_drone_collision": 0,
        "timeout": 0,
        "avg_speed_mps": 1.7,
        "corridor_empty_hover": 12,
        "escape_triggers": 1,
        "outcome": "SUCCESS",
        "reason": "GOAL_REACHED_AND_HELD",
        "sim_duration_s": 18.0,
        "wall_time_s": 40.0,
        "min_clearance_m": 0.2,
        "min_inter_drone_distance_m": 1.1,
        "max_hover_streak": 15,
    }
    mod.save_partial(stored)

    def boom(*_args, **_kwargs):
        raise AssertionError("断点续跑不应重跑")

    monkeypatch.setattr(mod, "_simulate", boom)
    got = mod.run_one("n15", 2, resume=True)
    assert got["escape_triggers"] == 1
    assert got["corridor_empty_hover"] == 12

    stale = dict(stored)
    stale["row_version"] = mod.ROW_VERSION + 1
    stale["seed"] = 1
    mod.save_partial(stale)
    assert mod.load_partial("n15", 1) is None


def test_summary_mentions_baseline_and_check_does_not_edit_tickets():
    mod = _script()
    assert mod.ARMS == ("n10", "n15", "n20", "n10-sphere", "n15-sphere", "n20-sphere")
    assert [mod.arm_n(arm) for arm in mod.ARMS] == [10, 15, 20, 10, 15, 20]
    assert mod.arm_sphere("n10") is False and mod.arm_sphere("n20-sphere") is True
    assert all(mod.arm_deviates(arm) for arm in mod.ARMS)
    assert "偏离上游" in mod.arm_label("n10")
    assert "E-B4c" in mod.arm_label("n10-sphere")
    rows = [
        {
            "arm": "n10",
            "seed": seed,
            "arrived": 4,
            "n_drones": 6,
            "obstacle_collision": 0,
            "inter_drone_collision": 0,
            "timeout": 0,
            "avg_speed_mps": 1.5,
            "corridor_empty_hover": 10,
            "escape_triggers": 2,
            "escape_up_triggers": 1,
            "escape_side_triggers": 1,
            "outcome": "SUCCESS",
            "reason": "GOAL_REACHED_AND_HELD",
            "sim_duration_s": 20.0,
            "min_clearance_m": 0.1,
            "min_inter_drone_distance_m": 1.0,
            "max_hover_streak": 10,
        }
        for seed in range(5)
    ]
    summary = mod.render_summary(rows)
    assert "E-C7 基线" in summary
    assert "15/30" in summary
    assert "1127" in summary
    assert "20/30" in summary
    text = mod.render_vs_baseline(rows)
    assert "多 5" in text
    assert "少 1127" not in text
    assert "悬停 50（基线 1127，少 1077）" in text
    assert "report-only" in mod.DISCLAIMER and "偏离上游" in mod.DISCLAIMER
    assert len(mod.job_specs(list(mod.ARMS), list(mod.SEEDS))) == 30
    tickets = (ROOT / "docs/tickets.md").read_text(encoding="utf-8")
    assert mod.main(["--check"]) == 0
    assert (ROOT / "docs/tickets.md").read_text(encoding="utf-8") == tickets
    assert "E-C8" not in tickets


def test_committed_runs_match_tables_when_present():
    path = ROOT / "docs/research/e-c8-escape-replan-runs.json"
    md_path = ROOT / "docs/research/e-c8-escape-replan.md"
    if not path.is_file() or not md_path.is_file():
        pytest.skip("报告还没写入")
    mod = _script()
    data = json.loads(path.read_text(encoding="utf-8"))
    rows = data["rows"]
    assert len(rows) == 30
    assert data["report_only"] is True and data["frozen"] is False
    assert data["upstream_deviation"] is True
    assert "偏离上游" in data["disclaimer"] and "report-only" in data["disclaimer"]
    assert data["baseline"]["arrived"] == 15
    assert data["baseline"]["hover"] == 1127
    assert data["baseline"]["avg_speed_mps"] == pytest.approx(1.580)
    md = md_path.read_text(encoding="utf-8")
    assert mod.render_summary(rows) in md
    assert mod.render_seeds(rows) in md
    assert mod.render_vs_baseline(rows) in md
    assert "只报数" in md and "偏离上游" in md and "report-only" in md
    for row in rows:
        assert row["arm"] in mod.ARMS
        assert row["seed"] in range(5)
        assert row["report_only"] is True and row["frozen"] is False
        assert row["upstream_deviation"] is True
        assert float(row["safe_radius_m"]) == pytest.approx(0.05)
        assert row["track_clearance_mode"] == "geom"
        assert float(row["track_margin_m"]) == pytest.approx(0.08)
        assert float(row["inter_threshold_m"]) == pytest.approx(1.0)
        assert row["empty_brake"] is True
        assert bool(row["peer_sphere"]) is mod.arm_sphere(row["arm"])
        assert int(row["escape_n"]) == mod.arm_n(row["arm"])
        assert row["avg_speed_mps"] is None or float(row["avg_speed_mps"]) > 0.0
        kinds = int(row["pillar_collision"]) + int(row["ground_collision"]) + int(row["box_collision"])
        assert kinds == int(row["obstacle_collision"])
        assert kinds + int(row["inter_drone_collision"]) <= 1
