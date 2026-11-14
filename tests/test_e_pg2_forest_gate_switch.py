"""票 E-PG2：进程内打开门控和余量，汇总表与报告对齐。不起六机全仿真，除了对照金样的那条。"""

import importlib.util
import json
import sys

import pytest

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-pg2-forest-gate-switch.py"
    name = "e_pg2_forest_gate_switch"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def _watched_bytes(mod):
    return {rel: (ROOT / rel).read_bytes() for rel in mod._WATCHED}


def _fake_row(mod, scene, arm_id, seed, *, arrived, timeout, hover, arrival_time, p95=4.5, gap=1.1, clear=0.2):
    arm = mod.arm_by_id(arm_id)
    return {
        "scene_kind": scene,
        "arm": arm_id,
        "params": arm.params,
        "config_status": arm.config_status,
        "seed": seed,
        "arrived": arrived,
        "n_drones": 6,
        "tree_collisions": 0,
        "inter_drone_collisions": 0,
        "timeout": timeout,
        "formation_error_p95_m": p95,
        "min_inter_drone_distance_m": gap,
        "min_clearance_m": clear,
        "arrival_time_s": arrival_time,
        "leader_hover": hover,
        "gate_s_m": None if not arm.progress_gate else 42.0,
        "outcome": "TIMEOUT" if timeout else "SUCCESS",
        "progress_gate": arm.progress_gate,
    }


def test_open_arm_is_the_deviation_and_baseline_is_the_card():
    mod = _script()
    assert [arm.arm_id for arm in mod.ARMS] == ["baseline", "open"]
    assert mod.DEVIATION_LABEL == "report-only / 偏离当前场景卡配置"
    baseline, open_arm = mod.ARMS
    assert baseline.progress_gate is False
    assert baseline.leader_margin_m is None
    assert baseline.config_status == mod.BASELINE_LABEL
    assert baseline.deviates is False
    assert open_arm.progress_gate is True
    assert open_arm.leader_margin_m == pytest.approx(3.0)
    assert open_arm.progress_percentile == pytest.approx(20.0)
    assert open_arm.config_status == mod.DEVIATION_LABEL
    assert open_arm.params == "progress_gate=true, leader_margin_m=3.0, progress_percentile=20"


def test_open_override_stays_in_memory_and_is_read_as_three_metres():
    mod = _script()
    before = _watched_bytes(mod)
    from yopo_l0.crossing import CrossingFleet

    baseline = mod.build_card("medium", "baseline")
    opened = mod.build_card("medium", "open")
    sparse = mod.build_card("sparse", "open")
    assert baseline.card["task"]["progress_gate"] is False
    assert "leader_margin_m" not in baseline.card["task"]
    assert opened.card["task"]["progress_gate"] is True
    assert opened.card["task"]["leader_margin_m"] == pytest.approx(3.0)
    assert opened.card["task"]["progress_percentile"] == 20
    assert opened.hash != baseline.hash
    assert sparse.card["task"]["progress_percentile"] == 20
    fleet = CrossingFleet.from_card(opened.card, 0.01)
    assert fleet.leader_margin_m == pytest.approx(3.0)
    assert fleet.progress_gate is True
    assert fleet.progress_percentile == pytest.approx(20.0)
    assert fleet.summary(0, 6)["leader_margin_m"] == pytest.approx(3.0)
    disk = json.loads((ROOT / "scenes/crossing-6-forest.json").read_text())
    other = json.loads((ROOT / "scenes/crossing-6-forest-sparse.json").read_text())
    assert disk["task"]["progress_gate"] is False
    assert other["task"]["progress_gate"] is False
    assert "leader_margin_m" not in disk["task"]
    assert "leader_margin_m" not in other["task"]
    assert _watched_bytes(mod) == before


def test_mean_arrival_skips_timeouts_and_labels_the_open_arm():
    mod = _script()
    rows = [
        _fake_row(mod, "medium", "baseline", 0, arrived=6, timeout=0, hover=0, arrival_time=26.0, p95=23.0),
        _fake_row(mod, "medium", "open", 0, arrived=6, timeout=0, hover=19, arrival_time=31.0, p95=4.4),
        _fake_row(mod, "medium", "open", 1, arrived=4, timeout=1, hover=40, arrival_time=None, p95=5.9),
    ]
    summary = {item["arm"]: item for item in mod.aggregate(rows)}
    assert summary["open"]["arrived"] == 10
    assert summary["open"]["drones"] == 12
    assert summary["open"]["timeouts"] == 1
    assert summary["open"]["mean_arrival_time_s"] == pytest.approx(31.0)
    assert summary["open"]["arrival_time_n"] == 1
    assert summary["open"]["mean_leader_hover"] == pytest.approx(29.5)
    assert summary["open"]["leader_hover_sum"] == 59
    assert summary["open"]["config_status"] == mod.DEVIATION_LABEL
    assert summary["baseline"]["config_status"] == mod.BASELINE_LABEL
    assert summary["baseline"]["gate_s_m"] == []
    text = mod.render_report_tables(rows)
    assert "10/12" in text
    assert "31.00（1/2）" in text
    assert mod.DEVIATION_LABEL in text
    assert "progress_gate=true, leader_margin_m=3.0, progress_percentile=20" in text
    assert "当前场景卡配置" in text


def test_resume_restores_a_finished_row_without_resimulating(tmp_path, monkeypatch):
    mod = _script()
    monkeypatch.setattr(mod, "PARTIAL", tmp_path)
    stored = _fake_row(mod, "sparse", "open", 4, arrived=6, timeout=0, hover=8, arrival_time=25.0)
    stored["row_version"] = mod.ROW_VERSION
    dest = mod._partial("sparse", "open", 4)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(stored))

    def boom(*_args, **_kwargs):
        raise AssertionError("断点续跑不应重跑")

    monkeypatch.setattr(mod, "build_card", boom)
    got = mod.run_one("sparse", "open", 4, resume=True)
    assert got["arrived"] == 6
    assert got["leader_hover"] == 8
    stored["row_version"] = mod.ROW_VERSION - 1
    dest.write_text(json.dumps(stored))
    with pytest.raises(AssertionError):
        mod.run_one("sparse", "open", 4, resume=True)


def test_report_tables_match_the_runs_json():
    mod = _script()
    path = ROOT / "docs/research/e-pg2-forest-gate-switch-runs.json"
    report = ROOT / "docs/research/e-pg2-forest-gate-switch.md"
    payload = json.loads(path.read_text())
    text = report.read_text()
    rows = payload["rows"]
    seeds = list(payload["seeds"])
    assert seeds == list(range(10))
    assert payload["report_only"] is True
    assert payload["deviation_label"] == mod.DEVIATION_LABEL
    for scene in mod.SCENES:
        for arm in mod.ARMS:
            got = sorted(int(row["seed"]) for row in rows if row["scene_kind"] == scene and row["arm"] == arm.arm_id)
            assert got == seeds
            for row in rows:
                if row["scene_kind"] != scene or row["arm"] != arm.arm_id:
                    continue
                assert row["config_status"] == arm.config_status
                assert row["params"] == arm.params
                assert bool(row["progress_gate"]) is arm.progress_gate
                assert int(row["n_drones"]) == 6
                if arm.deviates:
                    assert row["config_status"] == "report-only / 偏离当前场景卡配置"
                    assert row["deviates_from_scene_card"] is True
                    assert float(row["leader_margin_m"]) == pytest.approx(3.0)
                    assert float(row["progress_percentile"]) == pytest.approx(20.0)
                else:
                    assert row["deviates_from_scene_card"] is False
                    assert row["leader_margin_m"] is None
                    assert row["leader_hover"] == 0
    fresh = mod.aggregate(rows)
    stored = payload["summary"]
    assert [item["arm"] + item["scene"] for item in fresh] == [item["arm"] + item["scene"] for item in stored]
    for got, expect in zip(fresh, stored, strict=True):
        assert got["arrived"] == expect["arrived"]
        assert got["drones"] == expect["drones"]
        assert got["tree_collisions"] == expect["tree_collisions"]
        assert got["inter_drone_collisions"] == expect["inter_drone_collisions"]
        assert got["timeouts"] == expect["timeouts"]
        assert got["config_status"] == expect["config_status"]
        assert got["leader_hover_sum"] == expect["leader_hover_sum"]
    rendered = mod.render_report_tables(rows)
    assert rendered in text
    assert mod.DEVIATION_LABEL in text
    for scene in ("crossing-6-forest.json", "crossing-6-forest-sparse.json"):
        card = json.loads((ROOT / "scenes" / scene).read_text())
        assert card["task"]["progress_gate"] is False
        assert "leader_margin_m" not in card["task"]
