"""票 E-C3：结果记录增加单次规划墙钟。新字段不改轨迹，不进确定性比较，不进 gates。"""

import json
from pathlib import Path

import pytest

from yopo_l0.batch import _fmt_plan_wall, _legacy_section, _strip
from yopo_l0.criteria import _GATE_KIND, active_gates
from yopo_l0.plan_wall import PLAN_WALL_FIELD, p95_ms, stats
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import load_card
from yopo_l0.summary import summarize

from conftest import G2_CARD, ROOT

FIXTURES = Path(__file__).parent / "fixtures"
PRE_RESULT = FIXTURES / "ec3_prechange_g2_pillar_seed0_result.json"
PRE_TICKS = FIXTURES / "ec3_prechange_g2_pillar_seed0_ticks.json"
PILLAR = ROOT / "scenes/g2-pillar.json"
_PRE_CFG = dict(seed=0, policy="upstream", duration_s=0.8, record_planner_inputs=True)


def _canon(obj):
    return json.loads(json.dumps(obj))


def _without_wall(row: dict) -> dict:
    return {k: v for k, v in row.items() if k not in WALL_FIELDS and k != "commit"}


def test_same_seed_result_matches_prechange_after_dropping_new_fields():
    pre = json.loads(PRE_RESULT.read_text())
    pre_ticks = json.loads(PRE_TICKS.read_text())
    assert PLAN_WALL_FIELD not in pre
    out = run(load_card(PILLAR), RunConfig(**_PRE_CFG))
    assert PLAN_WALL_FIELD in out.result
    assert _canon(_without_wall(out.result)) == _without_wall(pre)
    assert _canon(out.ticks) == pre_ticks
    block = out.result[PLAN_WALL_FIELD]
    assert block["count"] == pre["plans"] == 8
    assert block["mean"] > 0
    assert block["p50"] > 0
    assert block["p95"] >= block["p50"]
    assert block["max"] >= block["p95"]


def test_same_seed_rerun_matches_when_wall_fields_are_dropped():
    card = load_card(PILLAR)
    a = run(card, RunConfig(**_PRE_CFG))
    b = run(card, RunConfig(**_PRE_CFG))
    assert a.ticks == b.ticks
    assert a.planner_inputs == b.planner_inputs
    assert _strip(a.result) == _strip(b.result)
    assert PLAN_WALL_FIELD not in _strip(a.result)
    assert "wall_time_s" not in _strip(a.result)


def test_missing_plan_wall_field_still_formats_and_summarizes(tmp_path):
    pre = json.loads(PRE_RESULT.read_text())
    assert PLAN_WALL_FIELD not in pre
    assert p95_ms(pre) is None
    assert _fmt_plan_wall(pre) == "—"
    src = tmp_path / "results.json"
    src.write_text(json.dumps([pre], ensure_ascii=False) + "\n")
    md_path = tmp_path / "SUMMARY.md"
    json_path = tmp_path / "SUMMARY.json"
    summary = summarize(
        [src],
        scenes_dir=ROOT / "scenes",
        md_path=md_path,
        json_path=json_path,
    )
    row = next(r for r in summary["rows"] if r["scene"] == "g2-pillar")
    assert row["plan_wall_ms"] is None
    assert "规划墙钟" not in row["status"]
    md = md_path.read_text()
    assert "规划墙钟 p95 ms（只报数）" in md
    assert "整回路墙钟" in md
    # 旧结果缺字段时，这一格是 —，不能挡冻结状态。
    assert row["status"].startswith("判据已冻结")

    card = load_card(PILLAR)
    lines, gate_ok = _legacy_section(card, "upstream", [pre], wall_limit=60.0)
    text = "\n".join(lines)
    assert "规划墙钟 p95 ms（只报数）" in text
    assert "墙钟 s | 规划墙钟 p95 ms（只报数） |" in text
    assert " | — |" in text
    assert gate_ok is True


def test_hover_without_network_records_empty_plan_wall():
    out = run(load_card(G2_CARD), RunConfig(seed=0, policy="hover", duration_s=0.2, shadow=False))
    assert out.result[PLAN_WALL_FIELD] == {
        "count": 0,
        "mean": None,
        "p50": None,
        "p95": None,
        "max": None,
    }
    assert _fmt_plan_wall(out.result) == "—"


def test_plan_wall_is_not_a_writable_gate():
    assert PLAN_WALL_FIELD not in _GATE_KIND
    assert "wall_time_s" not in _GATE_KIND
    card = load_card(PILLAR)
    gates = active_gates(card, policy="upstream", form="inprocess")
    assert gates is not None
    assert PLAN_WALL_FIELD not in gates
    assert "wall_time_s" not in gates


def test_stats_on_empty_and_populated_samples():
    empty = stats([])
    assert empty == {"count": 0, "mean": None, "p50": None, "p95": None, "max": None}
    filled = stats([1.0, 2.0, 3.0, 4.0, 10.0])
    assert filled["count"] == 5
    assert filled["mean"] == pytest.approx(4.0)
    assert filled["p50"] == pytest.approx(3.0)
    assert filled["max"] == pytest.approx(10.0)
    assert filled["p95"] >= filled["p50"]
