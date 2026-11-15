"""票 E-PG3：批跑脚本的臂、内存覆写和续跑。全仿真在脚本里，不在这里起。"""

import importlib.util
import json
import sys

import pytest

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-pg3-leader-prefix.py"
    name = "e_pg3_leader_prefix"
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


def _fake_row(mod, scene, arm_id, seed, *, arrived, timeout, hover, prefix, arrival_time, p95=4.5, gap=1.1, clear=0.2):
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
        "leader_prefix_count": prefix,
        "gate_s_m": None if not arm.progress_gate else 42.0,
        "outcome": "TIMEOUT" if timeout else "SUCCESS",
        "progress_gate": arm.progress_gate,
        "upstream_deviation": arm.upstream_deviation,
    }


def test_arms_are_the_default_margin_and_three_prefix_lengths():
    mod = _script()
    assert [arm.arm_id for arm in mod.ARMS] == ["baseline", "gate", "prefix03", "prefix07", "prefix10"]
    assert mod.PREFIX_LENGTHS_S == (0.3, 0.7, 1.0)
    baseline, gate, p03, p07, p10 = mod.ARMS
    assert baseline.config_status == mod.BASELINE_LABEL
    assert baseline.upstream_deviation is False
    assert baseline.params == "progress_gate=false"
    assert gate.progress_gate is True and gate.leader_prefix is False
    assert gate.config_status == mod.CARD_LABEL
    assert gate.upstream_deviation is False
    assert gate.params == "progress_gate=true, leader_margin_m=1.0, progress_percentile=0, leader_prefix=false"
    for arm, length in ((p03, 0.3), (p07, 0.7), (p10, 1.0)):
        assert arm.leader_prefix is True
        assert arm.leader_prefix_s == pytest.approx(length)
        assert arm.config_status == "report-only / 偏离上游"
        assert arm.upstream_deviation is True
        assert f"leader_prefix_s={length:.1f}" in arm.params
    assert "leader_margin_m=3.0" not in " ".join(arm.params for arm in mod.ARMS)
    assert "progress_percentile=20" not in " ".join(arm.params for arm in mod.ARMS)


def test_prefix_override_stays_in_memory():
    mod = _script()
    before = _watched_bytes(mod)
    from yopo_l0.crossing import CrossingFleet

    baseline = mod.build_card("medium", "baseline")
    gate = mod.build_card("medium", "gate")
    short = mod.build_card("sparse", "prefix03")
    longer = mod.build_card("medium", "prefix10")
    assert baseline.card["task"]["progress_gate"] is False
    assert "leader_prefix" not in baseline.card["task"]
    assert gate.card["task"]["progress_gate"] is True
    assert "leader_prefix" not in gate.card["task"]
    assert "leader_margin_m" not in gate.card["task"]
    assert short.card["task"]["leader_prefix"] is True
    assert short.card["task"]["leader_prefix_s"] == pytest.approx(0.3)
    assert longer.card["task"]["leader_prefix_s"] == pytest.approx(1.0)
    assert longer.hash != gate.hash
    assert short.hash != longer.hash
    fleet = CrossingFleet.from_card(longer.card, 0.01)
    assert fleet.leader_prefix_enabled is True
    assert fleet.leader_prefix_s == pytest.approx(1.0)
    assert fleet.effective_leader_margin_m() == pytest.approx(1.0)
    assert fleet.summary(0, 6)["leader_prefix"] is True
    gated = CrossingFleet.from_card(gate.card, 0.01)
    assert gated.leader_prefix_enabled is False
    assert "leader_prefix" not in gated.summary(0, 6)
    assert gated.summary(0, 6)["leader_margin_m"] == pytest.approx(1.0)
    disk = json.loads((ROOT / "scenes/crossing-6-forest.json").read_text())
    other = json.loads((ROOT / "scenes/crossing-6-forest-sparse.json").read_text())
    for raw in (disk, other):
        assert raw["task"]["progress_gate"] is False
        assert "leader_prefix" not in raw["task"]
        assert "leader_prefix_s" not in raw["task"]
        assert "leader_margin_m" not in raw["task"]
    assert _watched_bytes(mod) == before


def test_summary_labels_prefix_arms_as_upstream_deviation():
    mod = _script()
    rows = [
        _fake_row(mod, "medium", "baseline", 0, arrived=6, timeout=0, hover=0, prefix=0, arrival_time=26.0, p95=23.0),
        _fake_row(mod, "medium", "gate", 0, arrived=2, timeout=1, hover=100, prefix=0, arrival_time=None, p95=4.6),
        _fake_row(mod, "medium", "prefix07", 0, arrived=6, timeout=0, hover=4, prefix=20, arrival_time=30.0, p95=5.0),
        _fake_row(mod, "medium", "prefix07", 1, arrived=0, timeout=1, hover=10, prefix=8, arrival_time=None, p95=5.2),
    ]
    summary = {item["arm"]: item for item in mod.aggregate(rows)}
    assert summary["prefix07"]["arrived"] == 6
    assert summary["prefix07"]["timeouts"] == 1
    assert summary["prefix07"]["mean_arrival_time_s"] == pytest.approx(30.0)
    assert summary["prefix07"]["arrival_time_n"] == 1
    assert summary["prefix07"]["mean_leader_prefix"] == pytest.approx(14.0)
    assert summary["prefix07"]["leader_prefix_sum"] == 28
    assert summary["prefix07"]["config_status"] == "report-only / 偏离上游"
    assert summary["gate"]["config_status"] == mod.CARD_LABEL
    assert summary["baseline"]["config_status"] == mod.BASELINE_LABEL
    text = mod.render_report_tables(rows)
    assert "6/12" in text
    assert "30.00（1/2）" in text
    assert "report-only / 偏离上游" in text
    assert "leader_prefix_s=0.7" in text


def test_resume_skips_a_finished_prefix_row(tmp_path, monkeypatch):
    mod = _script()
    monkeypatch.setattr(mod, "PARTIAL", tmp_path)
    stored = _fake_row(mod, "sparse", "prefix10", 3, arrived=6, timeout=0, hover=2, prefix=11, arrival_time=28.0)
    stored["row_version"] = mod.ROW_VERSION
    dest = mod._partial("sparse", "prefix10", 3)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(stored))

    def boom(*_args, **_kwargs):
        raise AssertionError("断点续跑不应重跑")

    monkeypatch.setattr(mod, "build_card", boom)
    got = mod.run_one("sparse", "prefix10", 3, resume=True)
    assert got["leader_prefix_count"] == 11
    stored["row_version"] = mod.ROW_VERSION - 1
    dest.write_text(json.dumps(stored))
    with pytest.raises(AssertionError):
        mod.run_one("sparse", "prefix10", 3, resume=True)
