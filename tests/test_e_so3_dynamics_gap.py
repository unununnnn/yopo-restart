"""票 E-SO3：SO3 滞后只在脚本进程里注册，默认点质量和冻结场景卡不动。"""

import importlib.util
import json
import subprocess
import sys

import pytest

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-so3-dynamics-gap.py"
    spec = importlib.util.spec_from_file_location("e_so3_dynamics_gap", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_point_mass_defaults_and_source_stay_put():
    mod = _script()
    text = (ROOT / "yopo_l0/dynamics.py").read_text(encoding="utf-8")
    assert "so3" not in text.lower()
    assert 'MODELS = {"point_mass_lag": PointMassLag}' in text
    from yopo_l0.dynamics import PointMassLag

    model = PointMassLag()
    assert (model.tau_s, model.kp, model.kd, model.a_max) == (0.1, 4.0, 3.0, 6.0)
    assert mod.POINT_MASS == {
        "model": "point_mass_lag",
        "tau_s": 0.1,
        "kp": 4.0,
        "kd": 3.0,
        "a_max": 6.0,
    }
    assert mod.hover_horizontal_accel(35) > 6.0 > mod.hover_horizontal_accel(30)


def test_so3_step_lag_tilt_and_separate_vehicles():
    mod = _script()
    mod.self_check()
    nominal = mod.ARM_DYNAMICS["so3_nominal"]
    slow = mod.ARM_DYNAMICS["so3_slow"]
    assert nominal["tau_att_s"] == pytest.approx(0.12)
    assert nominal["tau_thrust_s"] == pytest.approx(0.06)
    assert nominal["max_tilt_deg"] == pytest.approx(35.0)
    assert nominal["a_max"] == pytest.approx(6.0)
    assert nominal["attitude_order"] == 2
    assert slow["tau_att_s"] == pytest.approx(0.22)
    assert slow["tau_thrust_s"] == pytest.approx(0.10)
    assert slow["max_tilt_deg"] == pytest.approx(30.0)
    assert slow["a_max"] == pytest.approx(6.0)
    for bad in (
        {"tau_att_s": 0.0},
        {"tau_thrust_s": -0.1},
        {"max_tilt_deg": 90.0},
        {"attitude_order": 3},
        {"a_max": 0.0},
        {"kp": -1.0},
    ):
        with pytest.raises(ValueError):
            mod.SO3LagModel(**bad)


def test_cards_keep_point_mass_on_disk_and_only_memory_swaps_so3():
    mod = _script()
    before = {path: (ROOT / path).read_bytes() for path in mod.FORBIDDEN}
    for extra in (
        "scenes/g2-pillar.json",
        "scenes/crossing-pillars.json",
        "scenes/crossing-forest.json",
        "scenes/crossing-6-forest-sparse.json",
        "scenes/suite.txt",
    ):
        before[extra] = (ROOT / extra).read_bytes()
    mod.install_so3_model()
    import yopo_l0.dynamics as dynamics

    assert dynamics.MODELS["point_mass_lag"] is dynamics.PointMassLag
    assert dynamics.MODELS["so3_lag"] is mod.SO3LagModel
    for scene in mod.SCENE_ORDER:
        disk = mod.load_scene(scene)
        point = mod.build_card(scene, "point_mass")
        assert point.hash == disk.hash
        assert point.card["dynamics"] == mod.POINT_MASS
        assert "so3_lag" not in json.dumps(point.card["dynamics"])
        for arm in ("so3_nominal", "so3_slow"):
            card = mod.build_card(scene, arm)
            assert card.card["dynamics"] == mod.ARM_DYNAMICS[arm]
            assert card.hash != disk.hash
            assert card.card["geometry"] == disk.card["geometry"]
            assert card.card["task"] == disk.card["task"]
            assert card.card["drones"] == disk.card["drones"]
    after = {path: (ROOT / path).read_bytes() for path in before}
    assert after == before
    diff = subprocess.check_output(
        [
            "git", "diff", "--",
            "yopo_l0", "core", "scenes",
            "docs/tickets.md", "README.md",
            "scripts/premerge.sh", "scripts/regress.sh",
        ],
        cwd=ROOT,
    )
    assert diff == b""


def test_grid_labels_config_and_check():
    mod = _script()
    assert mod.ARMS == ("point_mass", "so3_nominal", "so3_slow")
    assert mod.SCENE_ORDER == (
        "g2-pillar",
        "crossing-pillars",
        "crossing-forest",
        "crossing-6-forest-sparse",
    )
    assert mod.SCENES["g2-pillar"]["seeds"] == tuple(range(10))
    assert mod.SCENES["crossing-pillars"]["seeds"] == tuple(range(5))
    assert mod.SCENES["crossing-forest"]["seeds"] == tuple(range(5))
    assert mod.SCENES["crossing-6-forest-sparse"]["seeds"] == tuple(range(3))
    assert len(mod.full_specs()) == 69
    assert mod.arm_deviates("point_mass") is False
    assert mod.arm_deviates("so3_nominal") is True
    assert mod.arm_deviates("so3_slow") is True
    for arm in ("so3_nominal", "so3_slow"):
        label = mod.arm_label(arm)
        assert "report-only" in label
        assert "偏离默认动力学" in label
        assert "L0 点质量不变" in label
        assert mod.arm_mark(arm) == "report-only / 偏离默认动力学"
    assert "偏离默认动力学" not in mod.arm_label("point_mass")
    assert "report-only" in mod.DISCLAIMER and "偏离默认动力学" in mod.DISCLAIMER
    cfg = mod.run_config(3)
    assert cfg.policy == "upstream"
    assert cfg.shadow is True
    assert cfg.safe_radius_m is None
    assert cfg.geometric_fallback is None
    assert mod.main(["--check"]) == 0
    tickets = (ROOT / "docs/tickets.md").read_text(encoding="utf-8")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "E-SO3" not in tickets and "E-SO3" not in readme


def test_partial_resume_skips_resim_when_fingerprint_matches(tmp_path, monkeypatch):
    mod = _script()
    monkeypatch.setattr(mod, "PARTIAL", tmp_path)
    stored = {
        "row_version": mod.ROW_VERSION,
        "params_fingerprint": mod.params_fingerprint(),
        "scene": "g2-pillar",
        "arm": "so3_nominal",
        "seed": 2,
        "outcome": "SUCCESS",
        "arrived": 1,
        "n_drones": 1,
        "collision": 0,
        "min_clearance_m": 0.4,
        "formation_error_p95_m": None,
        "arrival_time_s": 3.5,
        "sim_duration_s": 4.5,
        "wall_time_s": 1.0,
    }
    mod.save_partial(stored)

    def boom(*_args, **_kwargs):
        raise AssertionError("断点续跑不应重跑")

    monkeypatch.setattr(mod, "_simulate", boom)
    got = mod.run_one("g2-pillar", "so3_nominal", 2, resume=True)
    assert got["arrival_time_s"] == pytest.approx(3.5)
    assert got["min_clearance_m"] == pytest.approx(0.4)

    stale = dict(stored)
    stale["seed"] = 3
    stale["params_fingerprint"] = "0" * 64
    mod.save_partial(stale)
    assert mod.load_partial("g2-pillar", "so3_nominal", 3) is None


def test_summary_gap_and_conclusion_use_the_rows():
    mod = _script()
    def row(scene, arm, seed, arrived, n, collision, clearance, p95, arrival):
        return {
            "scene": scene,
            "arm": arm,
            "seed": seed,
            "arrived": arrived,
            "n_drones": n,
            "collision": collision,
            "obstacle_collision": collision,
            "inter_drone_collision": 0,
            "outcome": "SUCCESS" if arrived == n and collision == 0 else "TIMEOUT",
            "reason": "GOAL_REACHED_AND_HELD" if arrived == n else "GOAL_NOT_REACHED",
            "min_clearance_m": clearance,
            "formation_error_p95_m": p95,
            "arrival_time_s": arrival,
            "sim_duration_s": 10.0,
            "positions_sha256": f"{scene}-{arm}-{seed}",
        }

    rows = [
        row("g2-pillar", "point_mass", 0, 1, 1, 0, 0.50, None, 3.0),
        row("g2-pillar", "so3_nominal", 0, 1, 1, 0, 0.40, None, 3.4),
        row("g2-pillar", "so3_slow", 0, 0, 1, 1, -0.02, None, None),
        row("crossing-pillars", "point_mass", 0, 3, 3, 0, 0.45, 1.0, 12.0),
        row("crossing-pillars", "so3_nominal", 0, 3, 3, 0, 0.30, 1.2, 13.0),
        row("crossing-pillars", "so3_slow", 0, 2, 3, 0, 0.20, 1.5, None),
    ]
    summary = mod.render_summary(rows)
    gap = mod.render_gap(rows)
    conclusion = mod.render_conclusion(rows)
    assert "1/1" in summary
    assert "0/1" in summary
    assert "3/3" in summary
    assert "2/3" in summary
    assert "report-only / 偏离默认动力学" in summary
    assert "-0.020" in summary or "-0.02" in summary
    assert "+0" in gap or "-1" in gap
    assert "-1" in gap
    assert "少 1 架次" in conclusion
    assert "多 1 次碰撞" in conclusion
    payload = {"rows": rows, "commit": "abc"}
    md = mod.render_markdown(payload)
    assert summary in md and gap in md and conclusion in md
    assert "偏离默认动力学" in md and "report-only" in md
    assert "MC_ROLL_P" in md and "MPC_TILTMAX_AIR" in md
    assert "e-so3-dynamics-gap-runs.json" in md


def test_committed_runs_match_tables_when_present():
    path = ROOT / "docs/research/e-so3-dynamics-gap-runs.json"
    md_path = ROOT / "docs/research/e-so3-dynamics-gap.md"
    if not path.is_file() or not md_path.is_file():
        pytest.skip("报告还没写入")
    mod = _script()
    data = json.loads(path.read_text(encoding="utf-8"))
    rows = data["rows"]
    assert len(rows) == 69
    assert data["report_only"] is True and data["frozen"] is False
    assert data["params_fingerprint"] == mod.params_fingerprint()
    assert "report-only" in data["disclaimer"] and "偏离默认动力学" in data["disclaimer"]
    md = md_path.read_text(encoding="utf-8")
    assert mod.render_summary(rows) in md
    assert mod.render_gap(rows) in md
    assert mod.render_seeds(rows) in md
    assert mod.render_conclusion(rows) in md
    assert "只报数" in md and "report-only" in md and "偏离默认动力学" in md
    seen = set()
    for row in rows:
        assert row["scene"] in mod.SCENE_ORDER
        assert row["arm"] in mod.ARMS
        assert int(row["seed"]) in mod.SCENES[row["scene"]]["seeds"]
        assert row["report_only"] is True and row["frozen"] is False
        assert bool(row["dynamics_deviation"]) is mod.arm_deviates(row["arm"])
        assert row["mark"] == mod.arm_mark(row["arm"])
        assert float(row["safe_radius_m"]) == pytest.approx(0.05)
        assert row["upstream_deviation"] is False
        assert row["shadow"] is True
        assert row["file_card_hash"] == mod.file_card_hash(row["scene"])
        if row["arm"] == "point_mass":
            assert row["dynamics"] == "point_mass_lag"
            assert row["run_card_hash"] == row["file_card_hash"]
        else:
            assert row["dynamics"] == "so3_lag"
            assert row["run_card_hash"] != row["file_card_hash"]
            assert "report-only" in row["mark"] and "偏离默认动力学" in row["mark"]
        assert int(row["collision"]) == int(row["obstacle_collision"]) + int(row["inter_drone_collision"])
        assert int(row["collision"]) <= 1
        seen.add((row["scene"], row["arm"], int(row["seed"])))
    assert len(seen) == 69
