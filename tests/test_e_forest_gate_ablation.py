"""票 E-PG1：内存覆写门控和 margin，汇总表与报告对齐。不起六机全仿真。"""

import importlib.util
import json
import sys

import numpy as np
import pytest

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-forest-gate-ablation.py"
    spec = importlib.util.spec_from_file_location("e_forest_gate_ablation", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _watched_bytes(mod):
    return {rel: (ROOT / rel).read_bytes() for rel in mod._WATCHED}


def _fake_row(mod, scene, arm_id, seed, *, arrived, timeout, hover, arrival_time, p95=2.5, rel=2.0, gap=1.1, clear=0.2):
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
        "relative_formation_p95_m": rel,
        "min_inter_drone_distance_m": gap,
        "min_clearance_m": clear,
        "arrival_time_s": arrival_time,
        "leader_hover": hover,
        "leader_prefix": 1,
        "leader_hold": 0,
        "hover_wait_s": 4.0,
        "gate_s_m": None if not arm.progress_gate else 12.0,
        "outcome": "TIMEOUT" if timeout else "SUCCESS",
        "progress_gate": arm.progress_gate,
    }


def test_arms_match_the_ticket():
    mod = _script()
    assert [arm.arm_id for arm in mod.ARMS] == ["arm0", "arm1", "arm2", "arm3", "arm4", "arm5"]
    assert mod.ARMS[0].progress_gate is False
    assert mod.ARMS[0].leader_margin_m is None
    assert mod.ARMS[0].config_status == mod.BASELINE_LABEL
    expect = [
        (1.0, 0.0),
        (3.0, 0.0),
        (6.0, 0.0),
        (10.0, 0.0),
        (3.0, 20.0),
    ]
    for arm, (margin, pct) in zip(mod.ARMS[1:], expect, strict=True):
        assert arm.progress_gate is True
        assert arm.leader_margin_m == pytest.approx(margin)
        assert arm.progress_percentile == pytest.approx(pct)
        assert arm.config_status == mod.DEVIATION_LABEL
        assert arm.deviates is True
    assert "report-only / 偏离当前场景卡配置" == mod.DEVIATION_LABEL


def test_gate_override_stays_in_memory_and_margin_is_not_a_card_field():
    mod = _script()
    before = _watched_bytes(mod)
    import yopo_l0.crossing as crossing

    source_margin = crossing._LEADER_MARGIN_M
    baseline = mod.build_card("medium", "arm0")
    widened = mod.build_card("medium", "arm2")
    wider = mod.build_card("medium", "arm4")
    quantile = mod.build_card("sparse", "arm5")
    assert baseline.card["task"]["progress_gate"] is False
    assert widened.card["task"]["progress_gate"] is True
    assert widened.card["task"]["progress_percentile"] == 0
    assert wider.card["task"]["progress_percentile"] == 0
    assert quantile.card["task"]["progress_gate"] is True
    assert quantile.card["task"]["progress_percentile"] == 20
    assert "leader_margin" not in json.dumps(widened.card["task"])
    assert widened.hash == wider.hash
    assert widened.hash != baseline.hash
    assert quantile.hash != widened.hash
    disk = json.loads((ROOT / "scenes/crossing-6-forest.json").read_text())
    sparse = json.loads((ROOT / "scenes/crossing-6-forest-sparse.json").read_text())
    assert disk["task"]["progress_gate"] is False
    assert sparse["task"]["progress_gate"] is False
    assert crossing._LEADER_MARGIN_M == source_margin
    assert _watched_bytes(mod) == before


def test_leader_margin_changes_ahead_of_gate_and_is_restored():
    mod = _script()
    import yopo_l0.crossing as crossing
    from yopo_l0.crossing import CrossingFleet

    before = _watched_bytes(mod)
    source_margin = crossing._LEADER_MARGIN_M
    card = mod.build_card("medium", "arm1")
    fleet = CrossingFleet(card.card, 0.01, record_inputs=False)
    assert fleet.progress_gate is True
    assert fleet.progress_percentile == pytest.approx(0.0)

    def place(drone_id, arc):
        slot = fleet.slots[drone_id]
        origin_x = float(fleet.rail.origin[0])
        fleet.note_progress(drone_id, np.array([origin_x + float(slot[0]) + arc, 0.0, 1.5]))

    place("uav0", 1.5)
    assert fleet.gate_s() == pytest.approx(0.0)
    with mod.override_leader_margin(1.0):
        assert crossing._LEADER_MARGIN_M == pytest.approx(1.0)
        assert fleet._ahead_of_gate("uav0", np.zeros(3)) is True
    with mod.override_leader_margin(3.0):
        assert fleet._ahead_of_gate("uav0", np.zeros(3)) is False
    with mod.override_leader_margin(1.0):
        place("uav0", 1.0)
        assert fleet._ahead_of_gate("uav0", np.zeros(3)) is False
        place("uav0", 1.0 + 1e-6)
        assert fleet._ahead_of_gate("uav0", np.zeros(3)) is True

    quantile = CrossingFleet(mod.build_card("medium", "arm5").card, 0.01, record_inputs=False)
    arcs = [0.0, 1.0, 2.0, 3.0, 4.0, 10.0]
    for drone_id, arc in zip(quantile.members, arcs, strict=True):
        slot = quantile.slots[drone_id]
        origin_x = float(quantile.rail.origin[0])
        quantile.note_progress(drone_id, np.array([origin_x + float(slot[0]) + arc, 0.0, 1.5]))
    assert quantile.progress_percentile == pytest.approx(20.0)
    assert quantile.gate_s() == pytest.approx(float(np.percentile(np.asarray(arcs, dtype=float), 20.0)))

    try:
        with mod.override_leader_margin(10.0):
            raise RuntimeError("boom")
    except RuntimeError:
        pass
    assert crossing._LEADER_MARGIN_M == source_margin
    with mod.override_leader_margin(None):
        assert crossing._LEADER_MARGIN_M == source_margin
    assert _watched_bytes(mod) == before


def test_hover_wait_and_relative_p95_use_only_the_samples_they_name():
    mod = _script()
    ticks = [
        {"uav": "uav0", "t": 0.01, "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0], "goal_error_m": 1.0},
        {"uav": "uav0", "t": 0.02, "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0], "goal_error_m": 1.0},
        {"uav": "uav0", "t": 0.03, "v": [1.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0], "goal_error_m": 1.0},
        {"uav": "uav1", "t": 0.01, "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0], "goal_error_m": 0.1},
        {"uav": "uav1", "t": 0.02, "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0], "goal_error_m": 0.1},
        {"uav": "uav1", "t": 0.03, "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0], "goal_error_m": 0.1},
    ]
    assert mod.hover_wait_s(ticks) == pytest.approx(0.01)
    assert mod.hover_wait_s([]) is None

    slots = {f"uav{i}": [0.0, 0.0, 0.0] for i in range(6)}
    spread = []
    for i in range(6):
        pos = [6.0, 0.0, 0.0] if i == 0 else [0.0, 0.0, 0.0]
        spread.append({"t": 0.01, "uav": f"uav{i}", "p": pos, "v": [0.0, 0.0, 0.0], "goal_error_m": 1.0})
    errs = [5.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    assert mod.relative_formation_p95(spread, slots) == pytest.approx(float(np.percentile(errs, 95)))


def test_mean_arrival_skips_timeouts_and_labels_the_gated_arm():
    mod = _script()
    rows = [
        _fake_row(mod, "medium", "arm0", 0, arrived=6, timeout=0, hover=0, arrival_time=26.0, p95=23.0, rel=9.0),
        _fake_row(mod, "medium", "arm1", 0, arrived=6, timeout=0, hover=10, arrival_time=20.0, p95=3.0),
        _fake_row(mod, "medium", "arm1", 1, arrived=4, timeout=1, hover=30, arrival_time=None, p95=4.0),
    ]
    summary = {item["arm"]: item for item in mod.aggregate(rows)}
    assert summary["arm1"]["arrived"] == 10
    assert summary["arm1"]["drones"] == 12
    assert summary["arm1"]["timeouts"] == 1
    assert summary["arm1"]["mean_arrival_time_s"] == pytest.approx(20.0)
    assert summary["arm1"]["arrival_time_n"] == 1
    assert summary["arm1"]["mean_leader_hover"] == pytest.approx(20.0)
    assert summary["arm1"]["config_status"] == mod.DEVIATION_LABEL
    assert summary["arm0"]["config_status"] == mod.BASELINE_LABEL
    assert summary["arm0"]["gate_s_m"] == []
    text = mod.render_report_tables(rows)
    assert "10/12" in text
    assert "20.00（1/2）" in text
    assert mod.DEVIATION_LABEL in text
    assert "progress_gate=true, _LEADER_MARGIN_M=1.0, progress_percentile=0" in text
    assert "当前场景卡配置" in text


def test_resume_restores_a_finished_row_without_resimulating(tmp_path, monkeypatch):
    mod = _script()
    monkeypatch.setattr(mod, "PARTIAL", tmp_path)
    stored = _fake_row(mod, "sparse", "arm3", 4, arrived=6, timeout=0, hover=2, arrival_time=30.0)
    stored["row_version"] = mod.ROW_VERSION
    stored["hover_wait_s"] = 1.5
    dest = mod._partial("sparse", "arm3", 4)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(stored))

    def boom(*_args, **_kwargs):
        raise AssertionError("断点续跑不应重跑")

    monkeypatch.setattr(mod, "build_card", boom)
    got = mod.run_one("sparse", "arm3", 4, resume=True)
    assert got["arrived"] == 6
    assert got["hover_wait_s"] == pytest.approx(1.5)
    stored["row_version"] = mod.ROW_VERSION - 1
    dest.write_text(json.dumps(stored))
    with pytest.raises(AssertionError):
        mod.run_one("sparse", "arm3", 4, resume=True)


def test_report_tables_match_the_runs_json():
    mod = _script()
    path = ROOT / "docs/research/e-forest-gate-ablation-runs.json"
    report = ROOT / "docs/research/e-forest-gate-ablation.md"
    payload = json.loads(path.read_text())
    text = report.read_text()
    rows = payload["rows"]
    seeds = list(payload["seeds"])
    assert seeds == list(range(seeds[0], seeds[-1] + 1))
    assert seeds[0] == 0
    assert len(seeds) >= 5
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
                assert float(row["progress_percentile"]) == pytest.approx(arm.progress_percentile)
                assert int(row["n_drones"]) == 6
                if arm.deviates:
                    assert row["config_status"] == "report-only / 偏离当前场景卡配置"
                    assert row["deviates_from_scene_card"] is True
                    assert float(row["leader_margin_m"]) == pytest.approx(arm.leader_margin_m)
                    assert float(row["leader_margin_applied_m"]) == pytest.approx(arm.leader_margin_m)
                else:
                    assert row["deviates_from_scene_card"] is False
                    assert row["leader_margin_m"] is None
                    assert "偏离当前场景卡配置" not in row["config_status"]
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
    rendered = mod.render_report_tables(rows)
    assert rendered in text
    assert "30/30" in text
    assert mod.DEVIATION_LABEL in text
    for scene in ("crossing-6-forest.json", "crossing-6-forest-sparse.json"):
        card = json.loads((ROOT / "scenes" / scene).read_text())
        assert card["task"]["progress_gate"] is False
