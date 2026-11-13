"""票 E-C6：六机 0.14 对向的内存场景、四档参数、平均速度和断点续跑。"""

import importlib.util
import json
import math
import sys

import numpy as np
import pytest

from yopo_l0.scene import SceneCard, parse_geometry

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-c6-six-density.py"
    spec = importlib.util.spec_from_file_location("e_c6_six_density", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _watched():
    return [
        ROOT / "scenes/crossing-6-forest.json",
        ROOT / "scenes/ego-density-014.json",
        ROOT / "scenes/suite.txt",
        ROOT / "docs/tickets.md",
        ROOT / "README.md",
        ROOT / "scripts/regress.sh",
    ]


def test_swap6_is_symmetric_and_endpoints_do_not_collide():
    mod = _script()
    geom = mod.density_geometry()
    assert geom["header"]["target_density_per_m2"] == pytest.approx(0.14)
    assert len(geom["cylinders"]) == 56
    placed = mod.place_swap6(mod.density_centers(geom))
    assert [d["id"] for d in placed] == [f"uav{i}" for i in range(6)]
    for i, drone in enumerate(placed):
        start = drone["start_enu"]
        goal = drone["goal_enu"]
        assert goal[0] == pytest.approx(-start[0])
        assert goal[1] == pytest.approx(-start[1])
        assert goal[2] == pytest.approx(1.5)
        assert drone["axis_distance_start"] >= 0.70 - 1e-9
        assert drone["axis_distance_goal"] >= 0.70 - 1e-9
        ux, uy = mod.SWAP6_UNIT[i]
        norm = math.hypot(start[0], start[1])
        assert start[0] / norm == pytest.approx(ux)
        assert start[1] / norm == pytest.approx(uy)
        nxt = placed[(i + 1) % 6]["start_enu"]
        dot = (start[0] * nxt[0] + start[1] * nxt[1]) / (
            norm * math.hypot(nxt[0], nxt[1])
        )
        assert dot == pytest.approx(math.cos(math.pi / 3))
        opp = placed[(i + 3) % 6]
        assert opp["start_enu"][0] == pytest.approx(goal[0])
        assert opp["start_enu"][1] == pytest.approx(goal[1])
        assert opp["radius_m"] == pytest.approx(drone["radius_m"])

    card = mod.build_card("d014-swap6", "off")
    assert card.name == "ego-density-014-swap6"
    assert card.card["task"]["type"] == "goto"
    assert "policy_params" not in card.card
    assert not (ROOT / "scenes/ego-density-014-swap6.json").exists()
    mod.assert_endpoints_clear(card)
    body = 0.35
    drones = card.card["drones"]
    for drone in drones:
        for key in ("start_enu", "goal_enu"):
            dist = card.min_distance(np.asarray(drone[key], dtype=float))
            assert dist > body


def test_four_arms_only_touch_the_in_memory_card():
    mod = _script()
    before = {path: path.read_bytes() for path in _watched()}
    for scene in mod.SCENE_ORDER:
        off = mod.build_card(scene, "off")
        base = mod.build_card(scene, "base")
        geom = mod.build_card(scene, "geom")
        multi = mod.build_card(scene, "geom_multi")
        assert "policy_params" not in off.card
        assert len(off.card["drones"]) == 6
        for card, mode in ((base, None), (geom, "geom"), (multi, "geom_multi")):
            up = card.card["policy_params"]["upstream"]
            assert up["inter_drone_filter"] is True
            assert up["inter_drone_threshold_m"] == pytest.approx(1.0)
            assert up["inter_drone_penalty"] == pytest.approx(0)
            assert up["empty_brake"] is True
            assert "safe_radius" not in up
            if mode is None:
                assert "track_clearance" not in up
            else:
                assert up["track_clearance"] is True
                assert up["track_clearance_mode"] == mode
                assert up["track_margin_m"] == pytest.approx(0.08)
        if scene == "crossing-6":
            assert off.card["task"]["type"] == "crossing"
            assert "forest" in off.card
        else:
            assert off.card["task"]["type"] == "goto"
            assert len(off.card["geometry"]["cylinders"]) == 56
    after = {path: path.read_bytes() for path in before}
    assert after == before
    names = sorted(p.name for p in (ROOT / "scenes").glob("*.json"))
    assert "ego-density-014-swap6.json" not in names


def test_avg_speed_keeps_only_positive_finite_samples():
    mod = _script()
    ticks = [
        {"uav": "uav0", "v": [3.0, 0.0, 0.0]},
        {"uav": "uav0", "v": [0.0, 0.0, 0.0]},
        {"uav": "uav1", "v": [0.0, 4.0, 0.0]},
        {"uav": "uav1", "v": [float("nan"), 0.0, 0.0]},
        {"uav": "uav2", "v": [1.0]},
    ]
    assert mod.avg_speed_mps(ticks) == pytest.approx((3.0 + 4.0) / 2.0)
    assert mod.avg_speed_mps([]) is None
    assert mod.avg_speed_mps([{"uav": "uav0", "v": [0.0, 0.0, 0.0]}]) is None
    assert mod.avg_speed_mps(None) is None
    wall = mod.plan_wall_block({"plan_wall_ms": {"count": 4, "mean": 11.5, "p50": 11.0, "p95": 14.25, "max": 16.0}})
    assert wall["mean"] == pytest.approx(11.5)
    assert wall["p95"] == pytest.approx(14.25)
    assert mod.corridor_empty_hover({"plan_codes": {"CORRIDOR_EMPTY_HOVER": 7, "OK": 3}}) == 7
    assert mod.corridor_empty_hover({}) == 0


def test_classify_pillar_ground_and_inter_from_the_hit_tick():
    mod = _script()
    geom = {
        "boxes": [{"name": "wall", "lo": [8.0, -1.0, 0.0], "hi": [9.0, 1.0, 2.0]}],
        "cylinders": [
            {"name": "c", "center_xy": [0.0, 0.0], "radius": 0.2, "z_lo": 0.0, "z_hi": 3.0}
        ],
    }
    card = SceneCard("tiny", {"criteria": {"body_radius_m": 0.35}}, parse_geometry(geom, 0.0))
    pillar_ticks = [{"uav": "uav0", "p": [0.5, 0.0, 1.5], "v": [1.0, 0.0, 0.0], "t": 0.1}]
    hit = mod.classify_collision(
        card, {"outcome": "COLLISION", "reason": "uav0:TRUTH_POSITION"}, pillar_ticks
    )
    assert hit == {
        "pillar_collision": 1,
        "ground_collision": 0,
        "box_collision": 0,
        "inter_drone_collision": 0,
        "hit_kind": "cylinder",
    }
    ground_ticks = [{"uav": "uav0", "p": [5.0, 0.0, 0.1], "v": [0.0, 0.0, -1.0], "t": 0.2}]
    hit_g = mod.classify_collision(
        card, {"outcome": "COLLISION", "reason": "uav0:TRUTH_POSITION"}, ground_ticks
    )
    assert hit_g["ground_collision"] == 1 and hit_g["pillar_collision"] == 0
    box_ticks = [{"uav": "uav1", "p": [8.2, 0.0, 1.0], "v": [1.0, 0.0, 0.0], "t": 0.3}]
    hit_b = mod.classify_collision(
        card, {"outcome": "COLLISION", "reason": "uav1:TRUTH_POSITION"}, box_ticks
    )
    assert hit_b["box_collision"] == 1 and hit_b["pillar_collision"] == 0
    hit_i = mod.classify_collision(
        card, {"outcome": "COLLISION", "reason": "uav0-uav1:INTER_DRONE"}, pillar_ticks
    )
    assert hit_i["inter_drone_collision"] == 1 and hit_i["pillar_collision"] == 0
    assert mod.classify_collision(card, {"outcome": "SUCCESS", "reason": "GOAL_REACHED_AND_HELD"}, pillar_ticks)[
        "hit_kind"
    ] is None


def test_partial_resume_restores_avg_speed_without_resimulating(tmp_path, monkeypatch):
    mod = _script()
    monkeypatch.setattr(mod, "PARTIAL", tmp_path)
    stored = {
        "row_version": mod.ROW_VERSION,
        "scene": "d014-swap6",
        "arm": "geom_multi",
        "seed": 4,
        "outcome": "SUCCESS",
        "arrived": 6,
        "n_drones": 6,
        "pillar_collision": 0,
        "ground_collision": 0,
        "box_collision": 0,
        "inter_drone_collision": 0,
        "avg_speed_mps": 1.25,
        "plan_wall_ms": {"count": 3, "mean": 9.5, "p50": 9.0, "p95": 12.0, "max": 12.5},
        "corridor_empty_hover": 2,
        "sim_duration_s": 12.0,
        "wall_time_s": 30.0,
    }
    mod.save_partial(stored)

    def boom(*_args, **_kwargs):
        raise AssertionError("断点续跑不应重跑")

    monkeypatch.setattr(mod, "_simulate", boom)
    got = mod.run_one("d014-swap6", "geom_multi", 4, resume=True)
    assert got["avg_speed_mps"] == pytest.approx(1.25)
    assert got["plan_wall_ms"]["p95"] == pytest.approx(12.0)
    assert got["corridor_empty_hover"] == 2

    fresh = dict(stored)
    fresh["avg_speed_mps"] = 2.5
    monkeypatch.setattr(mod, "_simulate", lambda *_a, **_k: fresh)
    again = mod.run_one("d014-swap6", "geom_multi", 4, resume=False)
    assert again["avg_speed_mps"] == pytest.approx(2.5)
    assert mod.load_partial("d014-swap6", "geom_multi", 4)["avg_speed_mps"] == pytest.approx(2.5)

    stale = dict(stored)
    stale["row_version"] = mod.ROW_VERSION + 1
    stale["seed"] = 1
    stale["avg_speed_mps"] = 9.0
    mod.save_partial(stale)
    assert mod.load_partial("d014-swap6", "geom_multi", 1) is None


def test_script_check_grid_and_labels_do_not_touch_tickets():
    mod = _script()
    assert mod.ARMS == ("off", "base", "geom", "geom_multi")
    assert mod.SEEDS == (0, 1, 2, 3, 4)
    assert mod.SCENE_ORDER == ("crossing-6", "d014-swap6")
    assert mod.arm_deviates("off") is False
    assert all(mod.arm_deviates(arm) for arm in ("base", "geom", "geom_multi"))
    assert "偏离上游" not in mod.arm_label("off")
    for arm in ("base", "geom", "geom_multi"):
        assert "偏离上游" in mod.arm_label(arm)
    assert len(mod.job_specs(list(mod.SCENE_ORDER), list(mod.ARMS), list(mod.SEEDS))) == 40
    assert "report-only" in mod.DISCLAIMER and "偏离上游" in mod.DISCLAIMER
    assert mod.main(["--check"]) == 0
    with pytest.raises(SystemExit):
        mod.main(["--jobs", "5", "--seeds", "0"])
    tickets = (ROOT / "docs/tickets.md").read_text(encoding="utf-8")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "E-C6" not in tickets and "E-C6" not in readme


def test_committed_runs_match_tables_when_present():
    path = ROOT / "docs/research/e-c6-six-density-runs.json"
    md_path = ROOT / "docs/research/e-c6-six-density.md"
    if not path.is_file() or not md_path.is_file():
        pytest.skip("报告还没写入")
    mod = _script()
    data = json.loads(path.read_text(encoding="utf-8"))
    rows = data["rows"]
    assert len(rows) == 40
    assert data["report_only"] is True and data["frozen"] is False
    assert "偏离上游" in data["disclaimer"] and "report-only" in data["disclaimer"]
    assert data["geom_sha256"] == mod.GEOM_SHA256
    assert len(data["swap6"]) == 6
    md = md_path.read_text(encoding="utf-8")
    assert mod.render_summary(rows) in md
    assert mod.render_seeds(rows) in md
    assert "只报数" in md and "偏离上游" in md and "report-only" in md
    for row in rows:
        assert row["scene"] in mod.SCENE_ORDER
        assert row["arm"] in mod.ARMS
        assert row["seed"] in range(5)
        assert row["report_only"] is True and row["frozen"] is False
        assert bool(row["upstream_deviation"]) is mod.arm_deviates(row["arm"])
        assert float(row["safe_radius_m"]) == pytest.approx(0.05)
        assert "mean" in row["plan_wall_ms"] and "p95" in row["plan_wall_ms"]
        assert row["avg_speed_mps"] is None or float(row["avg_speed_mps"]) > 0.0
        kinds = (
            int(row["pillar_collision"])
            + int(row["ground_collision"])
            + int(row["box_collision"])
            + int(row["inter_drone_collision"])
        )
        assert kinds <= 1
