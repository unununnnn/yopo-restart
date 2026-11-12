"""票 E-C4b：密度场补种子与归因脚本的结构检查。诊断钩子不改轨迹。"""

import importlib.util
import json
import sys

import pytest

from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card

from conftest import ROOT

DISCLAIMER = "这是假设，不是论文复现"


def _script():
    path = ROOT / "scripts/e-c4b-density-seeds.py"
    spec = importlib.util.spec_from_file_location("e_c4b_density_seeds", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _base():
    return _script()._load_base()


def test_reuses_e_c4_cards_and_does_not_touch_suite_or_defaults():
    mod = _script()
    base = _base()
    assert mod.DISCLAIMER == DISCLAIMER
    assert mod.SEEDS == tuple(range(1, 10))
    assert mod.ARMS == ("off", "combo", "radius035")
    assert mod.RADIUS035_M == pytest.approx(0.35)
    suite = (ROOT / "scenes/suite.txt").read_text()
    tickets = (ROOT / "docs/tickets.md").read_text()
    readme = (ROOT / "README.md").read_text()
    src = (ROOT / "scripts/e-c4b-density-seeds.py").read_text(encoding="utf-8")
    assert "docs/tickets.md" not in src or "不改" in src
    assert "write_cards" not in src
    for tag, kind, _geom, _n in (
        ("014", "swap3", "ego-density-014.json", 56),
        ("014", "solo", "ego-density-014.json", 56),
        ("028", "swap3", "ego-density-028.json", 112),
        ("028", "solo", "ego-density-028.json", 112),
        ("042", "swap3", "ego-density-042.json", 168),
        ("042", "solo", "ego-density-042.json", 168),
    ):
        name = f"ego-density-{tag}-{kind}.json"
        assert name not in suite
        path = ROOT / "scenes" / name
        raw = json.loads(path.read_text(encoding="utf-8"))
        assert raw["seeds"] == [0]
        assert raw["criteria"]["frozen"] is False
        assert "policy_params" not in raw
        assert "geometric_fallback" not in raw
        off = mod.build_run_card(base, tag, kind, "off")
        combo = mod.build_run_card(base, tag, kind, "combo")
        wide = mod.build_run_card(base, tag, kind, "radius035")
        assert off.hash == base.build_run_card(tag, kind, "off").hash
        assert combo.hash == base.build_run_card(tag, kind, "combo").hash
        assert wide.hash == off.hash
        assert wide.card.get("policy_params") == off.card.get("policy_params")
        assert "0.35" not in json.dumps(wide.card.get("policy_params") or {})
    assert "e-c4b" not in tickets
    cfg = mod.run_config("radius035", 1)
    assert cfg.safe_radius_m == pytest.approx(0.35)
    assert mod.run_config("off", 1).safe_radius_m is None
    assert "偏离上游" in src
    assert readme  # 读了但不改


def test_combo_overlay_matches_e_c4_and_radius035_is_deviation_only():
    mod = _script()
    base = _base()
    path = ROOT / "scenes/ego-density-014-swap3.json"
    before = path.read_text(encoding="utf-8")
    combo = mod.build_run_card(base, "014", "swap3", "combo")
    off = mod.build_run_card(base, "014", "swap3", "off")
    assert path.read_text(encoding="utf-8") == before
    upstream = combo.card["policy_params"]["upstream"]
    assert upstream["inter_drone_filter"] is True
    assert upstream["inter_drone_threshold_m"] == 0.70
    assert upstream["inter_drone_penalty"] == 0
    assert combo.card["geometric_fallback"] == {
        "enabled": True,
        "inter_drone_distance_m": 0.7,
        "commit_horizon_s": 1.0,
    }
    assert combo.card["policy_params"]["backend"] == {"enabled": True}
    assert "safe_radius" not in (off.card.get("policy_params") or {}).get("upstream", {})
    assert off.hash != combo.hash


def test_classify_tree_and_timeout_follow_stated_order():
    mod = _script()
    tree = dict(
        plan_code="OK",
        plan_clearance_m=0.2,
        actual_clearance_m=0.2,
        body_radius_m=0.35,
        in_depth=True,
        frame_available=True,
        admitted=True,
        safe_radius_m=0.05,
    )
    assert mod.classify_tree({**tree, "plan_code": "CORRIDOR_EMPTY_HOVER", "plan_clearance_m": 1.0}) == mod.CAT_HOVER
    assert mod.classify_tree({**tree, "plan_clearance_m": 0.5, "actual_clearance_m": 0.2}) == mod.CAT_TRACK
    assert mod.classify_tree(tree) == mod.CAT_CORRIDOR_005
    assert mod.classify_tree({**tree, "in_depth": False}) == mod.CAT_CORRIDOR_005
    assert mod.classify_tree({**tree, "safe_radius_m": 0.35}) == mod.CAT_CORRIDOR_035
    assert mod.classify_tree({**tree, "admitted": False, "plan_clearance_m": 0.2, "in_depth": False}) == mod.CAT_HIDDEN
    assert mod.classify_tree({**tree, "admitted": False, "plan_clearance_m": 0.2, "in_depth": True}) == mod.CAT_OTHER
    assert mod.classify_inter({"task": "formation", "filter_enabled": False}) == mod.CAT_FORMATION
    assert mod.classify_inter({"task": "goto", "plan_code": "OK", "planned_gap_m": 0.80, "actual_gap_m": 0.64, "filter_enabled": False}) == mod.CAT_TRACK
    assert mod.classify_inter({"task": "goto", "plan_code": "OK", "planned_gap_m": 0.60, "actual_gap_m": 0.64, "filter_enabled": False}) == mod.CAT_INTER_OFF
    assert mod.classify_inter({"task": "goto", "plan_code": "OK", "planned_gap_m": 0.60, "actual_gap_m": 0.64, "filter_enabled": True}) == mod.CAT_INTER_ON
    assert mod.classify_timeout_drone({"stalled": True}) == mod.CAT_TIMEOUT_STALL
    assert mod.classify_timeout_drone({"stalled": False}) == mod.CAT_TIMEOUT_OTHER
    assert mod.corridor_label(0.05) == "走廊准入 safe_radius 0.05 太松"
    assert mod.corridor_label(0.35) == "走廊准入 safe_radius 0.35 太松"


def test_diag_hook_does_not_change_upstream_ticks():
    mod = _script()
    card = load_card(ROOT / "scenes/ego-density-014-solo.json")
    cfg = RunConfig(seed=0, duration_s=0.5, shadow=True)
    stock = run(card, cfg)
    diag_cls, previous = mod.install_diag()
    try:
        hooked = run(load_card(ROOT / "scenes/ego-density-014-solo.json"), cfg)
        traces = mod.traces_from_instances(diag_cls)
    finally:
        mod.restore_diag(previous)
        diag_cls.instances = []
    assert [(t["uav"], t["p"], t["v"], t["plan_code"]) for t in stock.ticks] == [
        (t["uav"], t["p"], t["v"], t["plan_code"]) for t in hooked.ticks
    ]
    assert stock.result["outcome"] == hooked.result["outcome"]
    assert stock.result["plan_codes"] == hooked.result["plan_codes"]
    assert stock.result["safe_radius_m"] == pytest.approx(0.05)
    assert hooked.result["upstream_deviation"] is False
    assert traces["uav0"]
    assert all("frame" in row for row in traces["uav0"])


def test_jobs_limits_and_seed_default_excludes_zero():
    mod = _script()
    with pytest.raises(SystemExit):
        mod.main(["--jobs-plain", "3", "--seeds", "1"])
    with pytest.raises(SystemExit):
        mod.main(["--jobs-backend", "2", "--seeds", "1", "--arms", "off"])
    src = (ROOT / "scripts/e-c4b-density-seeds.py").read_text(encoding="utf-8")
    assert 'default=",".join(str(s) for s in SEEDS)' in src
    assert "range(1, 10)" in src
