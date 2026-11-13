"""票 E-A2：软让行默认关。关掉时配置和 inter_pick 与 E-A1 逐位相同。"""

import importlib.util
import json
import math
import sys

import numpy as np
import pytest

from yopo_l0.inter_drone import InterDroneConfig, inter_drone_config, inter_pick
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import load_card

from conftest import ROOT

_STRIP = set(WALL_FIELDS) | {"commit", "scene_card_hash"}
_ON = {
    "inter_drone_filter": True,
    "inter_drone_threshold_m": 1.0,
    "inter_drone_penalty": 0,
}


def _cfg(**extra) -> InterDroneConfig:
    upstream = dict(_ON)
    upstream.update(extra)
    cfg = inter_drone_config({"upstream": upstream})
    assert cfg is not None
    return cfg


def test_default_off_matches_legacy_config_and_pick():
    legacy = InterDroneConfig(1.0, 0.0)
    absent = _cfg()
    explicit = _cfg(inter_drone_soft_yield=False, inter_drone_hard_min_m=-1, inter_drone_yield_penalty=True)
    assert absent == legacy
    assert explicit == legacy
    assert absent.soft_yield is False
    assert absent.hard_min_m == pytest.approx(0.75)
    assert absent.yield_penalty == pytest.approx(2.0)
    # 过滤关着时不读软让行，和以前一样返回 None。
    assert inter_drone_config({"upstream": {"inter_drone_soft_yield": "yes"}}) is None
    score = np.array([0.2, 5.0, 4.0, 1.0, 3.0])
    admitted = np.array([True, True, True, False, True])
    cases = [
        np.array([3.0, 0.2, 0.5, 0.0, 1.0]),
        np.array([0.1, 0.2, 0.3, 9.0, 0.4]),
        np.array([2.0, 1.0, 4.0, 0.0, math.inf]),
        np.array([0.8, 0.8, 0.8, 0.8, 0.8]),
    ]
    for clear in cases:
        assert inter_pick(score, admitted, clear, legacy) == inter_pick(score, admitted, clear, absent)
        assert inter_pick(score, admitted, clear, legacy) == inter_pick(score, admitted, clear, explicit)
    penal = InterDroneConfig(threshold_m=1.0, penalty=2.5)
    soft_penal = _cfg(inter_drone_penalty=2.5, inter_drone_soft_yield=True)
    clear = np.array([2.0, 0.0, 0.2, 0.0, 0.4])
    assert inter_pick(score, admitted, clear, penal) == inter_pick(score, admitted, clear, soft_penal)


def test_switch_off_flight_matches_explicit_false(tmp_path):
    src = json.loads((ROOT / "scenes/open-3drone.json").read_text())
    (tmp_path / "open-area.json").write_text((ROOT / "scenes/open-area.json").read_text())
    absent = dict(src)
    absent["policy_params"] = {"upstream": dict(_ON)}
    explicit = dict(src)
    explicit["policy_params"] = {"upstream": {**_ON, "inter_drone_soft_yield": False}}
    path_a = tmp_path / "absent.json"
    path_b = tmp_path / "explicit.json"
    path_a.write_text(json.dumps(absent))
    path_b.write_text(json.dumps(explicit))
    out_a = run(load_card(path_a), RunConfig(seed=0, policy="upstream", duration_s=0.3, shadow=False))
    out_b = run(load_card(path_b), RunConfig(seed=0, policy="upstream", duration_s=0.3, shadow=False))
    ticks_a = out_a.ticks
    ticks_b = out_b.ticks
    assert ticks_a == ticks_b
    left = {k: v for k, v in out_a.result.items() if k not in _STRIP}
    right = {k: v for k, v in out_b.result.items() if k not in _STRIP}
    assert left == right


def test_parameter_validation():
    for bad in ("yes", 1, 0, None):
        with pytest.raises(ValueError):
            _cfg(inter_drone_soft_yield=bad)
    for bad in (0, -0.1, math.nan, math.inf, True, False, "0.75", None):
        with pytest.raises(ValueError):
            _cfg(inter_drone_soft_yield=True, inter_drone_hard_min_m=bad)
    for bad in (-0.01, math.nan, math.inf, True, False, "2", None):
        with pytest.raises(ValueError):
            _cfg(inter_drone_soft_yield=True, inter_drone_yield_penalty=bad)
    cfg = _cfg(inter_drone_soft_yield=True)
    assert cfg.soft_yield is True
    assert cfg.hard_min_m == pytest.approx(0.75)
    assert cfg.yield_penalty == pytest.approx(2.0)
    cfg = _cfg(inter_drone_soft_yield=True, inter_drone_hard_min_m=0.8, inter_drone_yield_penalty=0)
    assert cfg.hard_min_m == pytest.approx(0.8)
    assert cfg.yield_penalty == pytest.approx(0.0)


def test_soft_yield_keeps_margin_and_picks_smaller_index_on_tie():
    cfg = _cfg(inter_drone_soft_yield=True, inter_drone_hard_min_m=0.75, inter_drone_yield_penalty=2.0)
    # 硬过滤还有人留在 1.0 m 外面，软让行不改这一拍。
    score = np.array([0.5, 9.0])
    admitted = np.array([True, True])
    kept = inter_pick(score, admitted, np.array([2.0, 0.2]), cfg)
    assert kept.kind == "adjusted" and kept.action == 0 and kept.filtered == 1 and kept.penalized == 0
    # 全在阈值内。0.74 m 低于下限，滤掉；0.80 与 0.90 按让行权重重排。
    # adj(0.80) = 1 - 2 * 0.20 = 0.60；adj(0.90) = 1.2 - 2 * 0.10 = 1.00。
    score = np.array([9.0, 1.0, 1.2])
    admitted = np.array([True, True, True])
    clear = np.array([0.74, 0.80, 0.90])
    pick = inter_pick(score, admitted, clear, cfg)
    assert pick.kind == "adjusted" and pick.action == 2 and pick.order == (2, 1)
    assert pick.filtered == 1 and pick.penalized == 2 and pick.empty is False
    # 减完之后同分，取索引较小的那个。
    # 1.2 - 2 * 0.20 = 0.80；1.0 - 2 * 0.10 = 0.80。
    tied = inter_pick(
        np.array([1.2, 1.0]),
        np.array([True, True]),
        np.array([0.80, 0.90]),
        cfg,
    )
    assert tied.kind == "adjusted" and tied.action == 0 and tied.order == (0, 1)
    # 下限本身留下。刚好 0.75 m：scale = 0.25，分数 5 - 2 * 0.25 = 4.5。
    edge = inter_pick(np.array([5.0]), np.array([True]), np.array([0.75]), cfg)
    assert edge.kind == "adjusted" and edge.action == 0 and edge.penalized == 1 and edge.filtered == 0


def test_soft_yield_empty_when_everyone_is_inside_hard_min():
    cfg = _cfg(inter_drone_soft_yield=True, inter_drone_hard_min_m=0.75)
    score = np.array([4.0, 9.0, 1.0])
    admitted = np.array([True, True, False])
    clear = np.array([0.74, 0.10, 5.0])
    empty = inter_pick(score, admitted, clear, cfg)
    assert empty.kind == "empty" and empty.action is None and empty.empty
    assert empty.filtered == 2 and empty.penalized == 0
    # 同一组数、软让行关着，仍然是空集。
    off = inter_pick(score, admitted, clear, _cfg())
    assert off.kind == "empty" and off.filtered == 2 and off.penalized == 0
    # 测不到的 +inf 不进硬过滤，软让行也不会把它判空。
    far = inter_pick(
        np.array([1.0, 0.2]),
        np.array([True, True]),
        np.array([0.2, math.inf]),
        cfg,
    )
    assert far.kind == "adjusted" and far.action == 1 and far.filtered == 1


def _script():
    path = ROOT / "scripts/e-a2-inter-soft-yield.py"
    spec = importlib.util.spec_from_file_location("e_a2_inter_soft_yield", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_script_check_and_partial_smoke(tmp_path):
    mod = _script()
    assert mod.ARMS == ("base", "soft_only", "geom_base", "geom_soft")
    assert mod.SCENE_SEEDS["d014-swap3"] == tuple(range(10))
    assert mod.SCENE_SEEDS["d014-swap6"] == tuple(range(5))
    assert "偏离上游" in mod.DISCLAIMER and "report-only" in mod.DISCLAIMER
    for arm in mod.ARMS:
        assert "偏离上游" in mod.arm_label(arm)
    assert mod.main(["--check"]) == 0
    out = tmp_path / "runs.json"
    partial = tmp_path / "partial"
    rc = mod.main([
        "--partial",
        "--scenes", "d014-swap3",
        "--arms", "base",
        "--seeds", "0",
        "--jobs", "1",
        "--duration-s", "0.05",
        "--out", str(out),
        "--partial-dir", str(partial),
    ])
    assert rc == 0
    grain = partial / "d014-swap3-base-0-t0.05.json"
    assert grain.is_file()
    row = json.loads(grain.read_text(encoding="utf-8"))
    assert row["report_only"] is True and row["frozen"] is False and row["upstream_deviation"] is True
    assert row["arm"] == "base" and row["seed"] == 0 and row["n_drones"] == 3
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["report_only"] is True and payload["frozen"] is False
    assert payload["rows"] == [row]
    stamped = grain.stat().st_mtime_ns
    again = mod.main([
        "--partial",
        "--resume",
        "--scenes", "d014-swap3",
        "--arms", "base",
        "--seeds", "0",
        "--jobs", "1",
        "--duration-s", "0.05",
        "--out", str(out),
        "--partial-dir", str(partial),
    ])
    assert again == 0
    assert grain.stat().st_mtime_ns == stamped
    suite = (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    assert "e-a2" not in suite and "soft_yield" not in suite


def test_committed_report_when_present():
    path = ROOT / "docs/research/e-a2-inter-soft-yield-runs.json"
    md_path = ROOT / "docs/research/e-a2-inter-soft-yield.md"
    if not path.is_file() or not md_path.is_file():
        pytest.skip("报告还没写入")
    mod = _script()
    data = json.loads(path.read_text(encoding="utf-8"))
    rows = data["rows"]
    assert data["report_only"] is True and data["frozen"] is False and data["upstream_deviation"] is True
    assert "report-only" in data["disclaimer"] and "偏离上游" in data["disclaimer"]
    assert data["seeds_by_scene"]["d014-swap3"] == list(range(10))
    assert data["seeds_by_scene"]["d014-swap6"] == list(range(5))
    assert len(rows) == 60
    md = md_path.read_text(encoding="utf-8")
    assert mod.render_summary(rows) in md
    assert mod.render_seeds(rows) in md
    assert "report-only" in md and "偏离上游" in md and "只报数" in md
    for row in rows:
        assert row["report_only"] is True and row["frozen"] is False
        assert row["upstream_deviation"] is True
        assert row["scene"] in mod.SCENES and row["arm"] in mod.ARMS
