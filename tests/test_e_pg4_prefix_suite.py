"""票 E-PG4：0.7 s 短前缀默认关。关时不写字段，开时写字段并标偏离上游。"""

import importlib.util
import json
import sys

import pytest

from conftest import ROOT
from yopo_l0.crossing import CrossingFleet
from yopo_l0.progress import read_leader_prefix
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card


PILLARS = ROOT / "scenes/crossing-pillars.json"
SIX = ROOT / "scenes/crossing-6-forest.json"
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
    path = ROOT / "scripts/e-pg4-prefix-suite.py"
    spec = importlib.util.spec_from_file_location("e_pg4_prefix_suite", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_switch_defaults_off_and_cards_omit_the_field():
    assert read_leader_prefix({}) == (False, pytest.approx(0.3))
    assert read_leader_prefix({"leader_prefix": False, "leader_prefix_s": 0.7}) == (False, pytest.approx(0.3))
    for rel in SUITE_CARDS:
        raw = json.loads((ROOT / rel).read_text(encoding="utf-8"))
        assert "leader_prefix" not in raw["task"]
        assert "leader_prefix_s" not in raw["task"]
        assert raw["criteria"]["frozen"] is True
    fleet = CrossingFleet.from_card(load_card(SIX).card, 0.01)
    assert fleet.leader_prefix_enabled is False
    summary = fleet.summary(0, 6)
    assert "leader_prefix" not in summary
    assert "leader_prefix_s" not in summary


def test_switch_off_omits_the_field():
    card = load_card(PILLARS)
    out = run(card, RunConfig(seed=0, policy="crossing", shadow=False, duration_s=0.3))
    assert out.result["upstream_deviation"] is False
    assert "leader_prefix" not in out.result
    assert "leader_prefix_s" not in out.result
    assert "leader_prefix" not in out.result["crossing"]
    assert "leader_prefix_s" not in out.result["crossing"]
    disk = json.loads(PILLARS.read_text(encoding="utf-8"))
    assert disk["task"]["progress_gate"] is False
    assert "leader_prefix" not in disk["task"]


def test_switch_on_writes_the_field_and_marks_upstream_deviation():
    mod = _script()
    before = PILLARS.read_bytes()
    card, applied = mod.prepare_run(mod.case_by_id("crossing-pillars"))
    assert applied is True
    assert card.card["task"]["leader_prefix"] is True
    assert card.card["task"]["leader_prefix_s"] == pytest.approx(0.7)
    assert card.card["task"]["progress_gate"] is True
    assert "leader_margin_m" not in card.card["task"]
    assert PILLARS.read_bytes() == before
    out = run(card, RunConfig(seed=0, policy="crossing", shadow=False, duration_s=0.3))
    assert out.result["leader_prefix"] is True
    assert out.result["leader_prefix_s"] == pytest.approx(0.7)
    assert out.result["upstream_deviation"] is True
    assert out.result["crossing"]["leader_prefix"] is True
    assert out.result["crossing"]["leader_prefix_s"] == pytest.approx(0.7)
    assert out.result["crossing"]["progress_gate"] is True
    assert out.result["crossing"]["leader_margin_m"] == pytest.approx(1.0)
    disk = json.loads(PILLARS.read_text(encoding="utf-8"))
    assert "leader_prefix" not in disk["task"]
    assert "leader_prefix_s" not in disk["task"]
    assert disk["task"]["progress_gate"] is False
    assert disk["criteria"]["frozen"] is True


def test_prefix_is_in_memory_only_on_crossing_cards():
    mod = _script()
    assert mod.main(["--check"]) == 0
    for case in mod.CASES:
        raw_before = (ROOT / case["path"]).read_bytes()
        card, applied = mod.prepare_run(case)
        assert (ROOT / case["path"]).read_bytes() == raw_before
        assert applied is (case["policy"] == "crossing")
        task = card.card["task"]
        if applied:
            assert task["leader_prefix"] is True
            assert task["leader_prefix_s"] == pytest.approx(mod.PREFIX_S)
            assert task["progress_gate"] is True
            assert "leader_margin_m" not in task
        else:
            assert "leader_prefix" not in task
            assert "leader_prefix_s" not in task


def test_script_grid_resume_and_eval_budget(tmp_path, monkeypatch):
    mod = _script()
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
    monkeypatch.setenv("YOPO_BACKEND_BUDGET", "wall")
    with pytest.raises(RuntimeError, match="评估次数"):
        mod.assert_eval_budget()
    monkeypatch.delenv("YOPO_BACKEND_BUDGET", raising=False)
    mod.assert_eval_budget()
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


def test_broken_gate_names_follow_the_numbers():
    mod = _script()
    rows = [
        {
            "scene": "g2-pillar",
            "seed": 0,
            "policy": "upstream",
            "form": "inprocess",
            "forest_density": None,
            "arrived": 0,
            "n_drones": 1,
            "outcome": "TIMEOUT",
            "reason": "GOAL_NOT_REACHED",
            "arrival_time_s": None,
            "min_obstacle_distance_m": 1.0,
            "body_radius_m": 0.35,
            "min_clearance_m": 0.65,
            "collision": 0,
        }
    ]
    gates, failures = mod.evaluate_case(mod.case_by_id("g2-pillar"), rows)
    assert gates["arrival_rate"] == pytest.approx(1.0)
    assert "arrival_rate" in mod._broken_names(gates, rows)
    assert "arrival_time_s" in mod._broken_names(gates, rows)
    assert failures
    assert any("arrival_rate" in msg for msg in failures)
