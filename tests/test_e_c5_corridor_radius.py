"""票 E-C5：走廊半径对照脚本只报数，不改默认，刹停复用 E-B4。"""

import importlib.util
import json
import sys

import pytest

import yopo_l0.policy as policy
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-c5-corridor-radius.py"
    spec = importlib.util.spec_from_file_location("e_c5_corridor_radius", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_grid_labels_and_does_not_touch_defaults_or_suite():
    mod = _script()
    assert mod.RADII == (0.05, 0.20, 0.35)
    assert mod.A1_ORDER == ("off", "fix")
    assert mod.SEEDS == tuple(range(10))
    assert mod.deviates(0.05) is False
    assert mod.deviates(0.20) is True and mod.deviates(0.35) is True
    assert mod.radius_label(0.05) == "safe_radius 0.05 m"
    assert "偏离上游" not in mod.radius_label(0.05)
    assert mod.radius_label(0.20) == "safe_radius 0.20 m（偏离上游）"
    assert mod.radius_label(0.35) == "safe_radius 0.35 m（偏离上游）"
    assert mod.arm_label(0.05, "off") == "safe_radius 0.05 m，E-A1 关"
    assert mod.arm_label(0.35, "fix") == "safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型"
    assert policy.UPSTREAM_SAFE_RADIUS_M == pytest.approx(0.05)
    src = (ROOT / "scripts/e-c5-corridor-radius.py").read_text(encoding="utf-8")
    assert "def _proto_brake" not in src
    assert "_make_policy_cls" in src
    assert "e-b4-peer-diag.py" in src
    specs = mod.job_specs(list(mod.SCENE_ORDER), list(mod.RADII), list(mod.A1_ORDER), list(mod.SEEDS))
    assert len(specs) == 120
    assert mod.run_config(0.05, 3).safe_radius_m is None
    assert mod.run_config(0.20, 3).safe_radius_m == pytest.approx(0.20)
    assert mod.run_config(0.35, 3).safe_radius_m == pytest.approx(0.35)
    suite = (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    tickets = (ROOT / "docs/tickets.md").read_text(encoding="utf-8")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "e-c5" not in suite and "ego-density-014" not in suite
    assert "E-C5" not in tickets and "E-C5" not in readme
    assert mod.main(["--check"]) == 0
    with pytest.raises(SystemExit):
        mod.main(["--jobs", "5", "--seeds", "0"])


def test_fix_parent_is_e_b4_diag_and_cards_stay_on_disk():
    mod = _script()
    eb4 = mod.load_eb4()
    log = eb4._Log()
    parent = mod.policy_parent(eb4, True, log)
    assert parent.__module__ == eb4.__name__
    assert parent.__name__ == "DiagUpstream"
    assert log.proto_brake is True
    assert mod.policy_parent(eb4, False, eb4._Log()) is policy.POLICIES["upstream"]
    paths = [ROOT / rel for rel in mod.SCENES.values()]
    before = {path: path.read_bytes() for path in paths}
    off = mod.build_card(eb4, "d014-swap3", "off")
    solo = mod.build_card(eb4, "d014-solo", "off")
    fix = mod.build_card(eb4, "d014-swap3", "fix")
    assert off.hash == load_card(ROOT / "scenes/ego-density-014-swap3.json").hash
    assert solo.hash == load_card(ROOT / "scenes/ego-density-014-solo.json").hash
    assert "policy_params" not in off.card
    up = fix.card["policy_params"]["upstream"]
    assert up["inter_drone_filter"] is True
    assert up["inter_drone_threshold_m"] == pytest.approx(1.0)
    assert up["inter_drone_penalty"] == 0
    assert "safe_radius" not in up
    assert {path: path.read_bytes() for path in paths} == before


def test_classify_tree_order_keeps_depth_from_overriding():
    mod = _script()
    base = dict(
        plan_clearance_m=0.20,
        fly_clearance_m=0.20,
        actual_clearance_m=0.30,
        body_radius_m=0.35,
        in_depth=True,
        frame_available=True,
        fly_admitted=True,
        fly_code="OK",
        fly_proto_braked=False,
        plan_code="OK",
        proto_braked=False,
    )
    assert mod.classify_tree({**base, "plan_clearance_m": 0.50, "actual_clearance_m": 0.30}) == mod.CAT_TRACK
    assert mod.classify_tree({
        **base, "plan_clearance_m": 0.50, "actual_clearance_m": 0.30, "in_depth": False, "plan_code": "CORRIDOR_EMPTY_HOVER",
    }) == mod.CAT_TRACK
    assert mod.classify_tree(base) == mod.CAT_LOOSE
    assert mod.classify_tree({**base, "in_depth": False}) == mod.CAT_LOOSE
    assert mod.classify_tree({**base, "fly_admitted": False, "in_depth": False}) == mod.CAT_BLIND
    assert mod.classify_tree({
        **base, "fly_clearance_m": None, "fly_admitted": False, "plan_code": "CORRIDOR_EMPTY_HOVER",
        "proto_braked": True, "in_depth": True, "plan_clearance_m": 0.20,
    }) == mod.CAT_BRAKE
    assert mod.classify_tree({
        **base, "fly_clearance_m": None, "fly_admitted": False, "plan_code": "CORRIDOR_EMPTY_HOVER",
        "proto_braked": False, "in_depth": True, "plan_clearance_m": 0.20,
    }) == mod.CAT_HOVER
    assert mod.classify_tree({
        **base, "fly_admitted": False, "in_depth": True, "plan_code": "OK", "plan_clearance_m": 0.20,
    }) == mod.CAT_OTHER
    assert mod.classify_inter({
        "plan_code": "OK", "planned_gap_m": 0.80, "actual_gap_m": 0.64, "filter_enabled": False, "proto_braked": False,
    }) == mod.CAT_TRACK
    assert mod.classify_inter({
        "plan_code": "OK", "planned_gap_m": 0.60, "actual_gap_m": 0.64, "filter_enabled": False, "proto_braked": False,
    }) == mod.CAT_INTER_OFF
    assert mod.classify_inter({
        "plan_code": "OK", "planned_gap_m": 0.60, "actual_gap_m": 0.64, "filter_enabled": True, "proto_braked": False,
    }) == mod.CAT_INTER_ON


def test_trace_hook_does_not_change_upstream_ticks():
    mod = _script()
    card = load_card(ROOT / "scenes/ego-density-014-solo.json")
    cfg = RunConfig(seed=0, duration_s=0.4, shadow=True)
    stock = run(card, cfg)
    eb4 = mod.load_eb4()
    diag_cls, previous, _log = mod.install_trace(eb4, False)
    try:
        hooked = run(load_card(ROOT / "scenes/ego-density-014-solo.json"), cfg)
        traces = mod._traces(diag_cls)
    finally:
        mod.restore_trace(previous)
        diag_cls.instances = []
    assert [(t["uav"], t["p"], t["v"], t["plan_code"], t["cmd_p"]) for t in stock.ticks] == [
        (t["uav"], t["p"], t["v"], t["plan_code"], t["cmd_p"]) for t in hooked.ticks
    ]
    assert stock.result["outcome"] == hooked.result["outcome"]
    assert stock.result["plan_codes"] == hooked.result["plan_codes"]
    assert stock.result["safe_radius_m"] == pytest.approx(0.05)
    assert hooked.result["upstream_deviation"] is False
    assert traces["uav0"]
    assert all(row["safe_radius_m"] == pytest.approx(0.05) for row in traces["uav0"])
    assert policy.POLICIES["upstream"] is previous


def test_committed_runs_match_tables_when_present():
    path = ROOT / "docs/research/e-c5-corridor-radius-runs.json"
    md_path = ROOT / "docs/research/e-c5-corridor-radius.md"
    if not path.is_file() or not md_path.is_file():
        pytest.skip("报告还没写入")
    mod = _script()
    data = json.loads(path.read_text(encoding="utf-8"))
    rows = data["rows"]
    assert len(rows) == 120
    assert data["disclaimer"]
    md = md_path.read_text(encoding="utf-8")
    assert mod.render_summary(rows) in md
    assert mod.render_tree_counts(rows) in md
    assert mod.render_tree_details(rows) in md
    assert mod.render_seeds(rows) in md
    for row in rows:
        assert row["scene"] in mod.SCENE_ORDER
        assert row["seed"] in range(10)
        assert row["a1"] in mod.A1_ORDER
        assert bool(row["upstream_deviation"]) is mod.deviates(row["radius_m"])
        if abs(float(row["radius_m"]) - 0.05) < 1e-9:
            assert "偏离上游" not in mod.radius_label(row["radius_m"])
        else:
            assert "偏离上游" in mod.arm_label(row["radius_m"], row["a1"])
        if int(row["tree_collision"]):
            assert row["attribution"]["category"] in mod.TREE_ORDER
