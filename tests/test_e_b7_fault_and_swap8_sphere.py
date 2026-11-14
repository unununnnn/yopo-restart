"""E-B7：六档通讯故障与八机对向的欧氏球只报数。不改前人文件。"""

import importlib.util
import json
import sys

import pytest

from yopo_l0.backend.config import config_from_params, peer_sphere_enabled
from yopo_l0.scene import load_card

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-b7-fault-and-swap8-sphere.py"
    spec = importlib.util.spec_from_file_location("e_b7_fault_and_swap8_sphere", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _watched():
    return [
        ROOT / "docs/tickets.md",
        ROOT / "README.md",
        ROOT / "scripts/premerge.sh",
        ROOT / "scripts/regress.sh",
        ROOT / "scenes/formation-6-open.json",
        ROOT / "scenes/formation-6-tiers.json",
        ROOT / "scenes/swap-8-open.json",
        ROOT / "scenes/suite.txt",
        ROOT / "yopo_l0/policy.py",
        ROOT / "yopo_l0/crossing.py",
        ROOT / "yopo_l0/formation.py",
        ROOT / "yopo_l0/runner.py",
        ROOT / "yopo_l0/backend/bspline.py",
        ROOT / "yopo_l0/backend/config.py",
        ROOT / "core/src/yopo_swarm_core/protocol/node.py",
    ]


def test_grid_labels_and_check_do_not_touch_upstream_files():
    mod = _script()
    before = {path: path.read_bytes() for path in _watched()}
    src = (ROOT / "scripts/e-b7-fault-and-swap8-sphere.py").read_text(encoding="utf-8")
    assert "FormationFleet._planner" not in src
    assert "FormationFleet._offer" not in src
    assert "FormationFleet._apply_backend" not in src
    assert mod.FORMATION_ARMS == ("off", "eb4c", "eb5-r100")
    assert mod.SWAP_ARMS == ("off", "eb4c", "eb5-r100", "eb5-w16")
    assert "eb5-w16" not in mod.FORMATION_ARMS
    assert mod.FORMATION_SEEDS == (0, 1, 2, 3, 4)
    assert mod.SWAP_SEEDS == (0, 1, 2, 3, 4)
    assert mod.tier_ids() == ("none", "drop-5", "drop-20", "duplicate-10", "reorder-delay", "combined")
    assert mod.arm_deviates("off") is False
    assert "偏离上游" not in mod.arm_label("off")
    assert "report-only" not in mod.arm_label("off")
    for arm in ("eb4c", "eb5-r100", "eb5-w16"):
        label = mod.arm_label(arm)
        assert "report-only" in label and "偏离上游" in label
        assert mod.arm_deviates(arm) is True
    assert mod.arm_label("eb4c") == "E-B4c(r=0.85, w=8) report-only / 偏离上游"
    assert mod.arm_label("eb5-r100") == "E-B5(r=1.00, w=8) report-only / 偏离上游"
    assert mod.arm_label("eb5-w16") == "E-B5(r=0.85, w=16) report-only / 偏离上游"
    jobs = mod.iter_jobs(
        list(mod.GROUPS),
        list(mod.FORMATION_ARMS),
        list(mod.SWAP_ARMS),
        list(mod.FORMATION_SEEDS),
        list(mod.SWAP_SEEDS),
        list(mod.tier_ids()),
    )
    assert len(jobs) == 6 * 3 * 5 + 4 * 5
    assert all(not mod.uses_refine_budget(group, arm) for group, _tier, arm, _seed in jobs if group == "formation")
    assert mod.uses_refine_budget("swap8", "eb4c") is True
    assert mod.uses_refine_budget("swap8", "off") is False
    assert "report-only" in mod.DISCLAIMER and "偏离上游" in mod.DISCLAIMER
    assert "FormationFleet" in mod.WIRING and "refine_yopo" in mod.WIRING
    assert mod.main(["--check"]) == 0
    with pytest.raises(SystemExit):
        mod.main(["--jobs-plain", "3", "--check"])
    after = {path: path.read_bytes() for path in before}
    assert after == before
    tickets = (ROOT / "docs/tickets.md").read_text(encoding="utf-8")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "E-B7" not in tickets and "E-B7" not in readme


def test_cards_stay_in_memory_and_sphere_arms_deviate():
    mod = _script()
    before = {path: path.read_bytes() for path in _watched()}
    off_f = mod.build_card("formation", "off")
    loaded = load_card(ROOT / "scenes/formation-6-open.json")
    assert off_f.card["task"]["type"] == "formation"
    assert off_f.hash == loaded.hash
    assert "policy_params" not in off_f.card
    assert "bus" not in off_f.card
    assert peer_sphere_enabled(off_f.card.get("policy_params")) is False

    eb4 = mod.build_card("formation", "eb4c")
    eb5 = mod.build_card("formation", "eb5-r100")
    assert eb4.card["task"]["type"] == "formation"
    assert eb4.hash != off_f.hash
    assert "bus" not in eb4.card
    cfg4 = config_from_params(eb4.card["policy_params"], a_max=6.0)
    cfg5 = config_from_params(eb5.card["policy_params"], a_max=6.0)
    assert cfg4 is not None and cfg5 is not None
    assert cfg4.peer_sphere_enabled is True and cfg4.swarm_penalty is False
    assert cfg4.peer_sphere_radius_m == pytest.approx(0.85)
    assert cfg4.peer_sphere_weight == pytest.approx(8.0)
    assert cfg5.peer_sphere_radius_m == pytest.approx(1.0)
    assert cfg5.peer_sphere_weight == pytest.approx(8.0)
    assert cfg5.swarm_segment_penalty is False

    off_s = mod.build_card("swap8", "off")
    swap = load_card(ROOT / "scenes/swap-8-open.json")
    assert off_s.card["task"]["type"] == "goto"
    assert off_s.hash == swap.hash
    assert len(off_s.card["drones"]) == 8
    wide = mod.build_card("swap8", "eb5-w16")
    cfg_w = config_from_params(wide.card["policy_params"], a_max=6.0)
    assert cfg_w is not None
    assert cfg_w.peer_sphere_radius_m == pytest.approx(0.85)
    assert cfg_w.peer_sphere_weight == pytest.approx(16.0)
    assert cfg_w.swarm_penalty is False
    assert not (ROOT / "scenes/swap-8-open.json").read_text(encoding="utf-8").count("peer_sphere")
    after = {path: path.read_bytes() for path in before}
    assert after == before


def test_wrap_passes_same_tick_samples_only_when_sphere_is_on(monkeypatch):
    mod = _script()
    mod.install_peer_delivery()
    import yopo_l0.policy as policy_mod

    seen = {}

    def spy(traj, cfg, obstacles, goal=None, neighbors=None):
        seen["neighbors"] = neighbors
        seen["radius"] = cfg.peer_sphere_radius_m
        seen["weight"] = cfg.peer_sphere_weight
        seen["enabled"] = cfg.peer_sphere_enabled
        return traj, {"applied": True, "reason": "spy", "budget_cut": False}

    monkeypatch.setattr("yopo_l0.backend.bspline.refine_yopo", spy)
    card = mod.build_card("swap8", "eb5-w16")
    cfg = config_from_params(card.card["policy_params"], a_max=6.0)

    class Holder:
        pass

    holder = Holder()
    holder._backend = cfg
    holder._swarm = False
    holder._neighbors = {"uav1": "sample"}
    holder._obstacles = ()
    holder.backend_log = []
    policy_mod.UpstreamPolicy._apply_backend(holder, object(), object())
    assert seen["neighbors"] == ["sample"]
    assert seen["enabled"] is True
    assert seen["radius"] == pytest.approx(0.85)
    assert seen["weight"] == pytest.approx(16.0)
    assert holder.backend_log[0]["reason"] == "spy"

    seen.clear()
    holder._backend = config_from_params({"backend": {"enabled": True}}, a_max=6.0)
    holder._swarm = False
    holder.backend_log = []
    policy_mod.UpstreamPolicy._apply_backend(holder, object(), object())
    assert seen["neighbors"] is None
    assert seen["enabled"] is False


def test_committed_runs_match_tables_when_present():
    path = ROOT / "docs/research/e-b7-fault-and-swap8-sphere-runs.json"
    md_path = ROOT / "docs/research/e-b7-fault-and-swap8-sphere.md"
    if not path.is_file() or not md_path.is_file():
        pytest.skip("报告还没写入")
    mod = _script()
    data = json.loads(path.read_text(encoding="utf-8"))
    rows = data["rows"]
    assert data["report_only"] is True and data["frozen"] is False
    assert data["formation_calls_refine_yopo"] is False
    assert "偏离上游" in data["disclaimer"] and "report-only" in data["disclaimer"]
    assert "e-b4c-sphere-avoid.py" in data["neighbor_delivery"]
    assert data["formation_seeds"] == [0, 1, 2, 3, 4]
    assert data["swap_seeds"] == [0, 1, 2, 3, 4]
    assert data["tiers"] == list(mod.tier_ids())
    assert len(rows) == len(
        mod.iter_jobs(
            ["formation", "swap8"],
            list(mod.FORMATION_ARMS),
            list(mod.SWAP_ARMS),
            data["formation_seeds"],
            data["swap_seeds"],
            data["tiers"],
        )
    )
    assert data["incomplete"] == 0
    assert data["formation_arm_mismatches"] == mod.formation_arm_mismatches(rows)
    formation = [r for r in rows if r["group"] == "formation"]
    swap = [r for r in rows if r["group"] == "swap8"]
    assert formation and swap
    if formation:
        assert data["formation_arms_match_off"] is (len(data["formation_arm_mismatches"]) == 0)
    md = md_path.read_text(encoding="utf-8")
    assert mod.render_group1(rows) in md
    assert mod.render_group1_seeds(rows) in md
    assert mod.render_group2(rows) in md
    assert mod.render_group2_seeds(rows) in md
    assert "只报数" in md and "偏离上游" in md and "report-only" in md
    for row in rows:
        assert row["report_only"] is True and row["frozen"] is False
        assert row["swarm_penalty"] is False
        assert bool(row["deviates_from_upstream"]) is mod.arm_deviates(row["arm"])
        assert bool(row["peer_sphere_enabled"]) is mod.arm_deviates(row["arm"])
        assert row["arm_label"] == mod.arm_label(row["arm"])
        if row["arm"] == "off":
            assert row["peer_sphere_radius_m"] is None
            assert "偏离上游" not in row["arm_label"]
        else:
            assert "report-only" in row["arm_label"] and "偏离上游" in row["arm_label"]
        if row["group"] == "formation":
            assert row["task"] == "formation"
            assert int(row["backend_budget_calls"]) == 0
            assert row["tier"] in mod.tier_ids()
            assert row["seed"] in range(5)
        else:
            assert row["task"] == "goto"
            assert row["n_drones"] == 8
            assert row["tier"] is None
            assert row["seed"] in range(5)
            if row["arm"] != "off":
                assert int(row["backend_budget_calls"]) > 0
