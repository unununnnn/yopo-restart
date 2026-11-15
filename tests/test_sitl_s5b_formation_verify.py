"""票 S5b：三机 formation-hold 的 PX4 SITL 复验入口与对照卡。假 PX4 / dry-run，不起 Gazebo。"""

import json
import subprocess

from conftest import ROOT
from yopo_l0.scene import load_card
from yopo_l0.sitl_bridge import live_topic_plan

SITL = ROOT / "scripts/sitl"
BASE = ROOT / "scenes/formation-hold.json"
SITL_CARD = ROOT / "scenes/formation-hold-sitl80.json"
RUNS = ROOT / "docs/research/s5b-sitl-formation-verify-runs.json"


def test_sitl_card_only_extends_duration_and_stays_off_suite():
    base = json.loads(BASE.read_text())
    sitl = json.loads(SITL_CARD.read_text())
    assert sitl["name"] == "formation-hold-sitl80"
    assert sitl["task"]["duration_s"] == 80.0
    assert base["task"]["duration_s"] == 40.0
    note = sitl.pop("note")
    assert "duration_s" in note and "suite.txt" in note
    base["name"] = sitl["name"]
    base["task"]["duration_s"] = sitl["task"]["duration_s"]
    assert sitl == base
    suite = (ROOT / "scenes/suite.txt").read_text().lower()
    assert "formation-hold-sitl80" not in suite


def test_dry_run_selects_formation_and_writes_no_px4_params():
    card = load_card(SITL_CARD)
    plan = live_topic_plan(card, positioning="default", seed=0, policy="formation")
    assert plan["policy"] == "formation"
    assert plan["n_drones"] == 3
    assert plan["scene"] == "formation-hold-sitl80"
    assert plan["px4_parameters_written"] == []
    assert "protocol" not in json.dumps(plan.get("topics") or {})


def test_run_script_dry_run_prints_policy_and_scene():
    shell = subprocess.run(
        [
            "bash",
            str(SITL / "run_yopo_l0_sitl.sh"),
            "--dry-run",
            "--n",
            "3",
            "--scene",
            "scenes/formation-hold-sitl80.json",
            "--policy",
            "formation",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert shell.returncode == 0, shell.stderr
    assert "scene=scenes/formation-hold-sitl80.json" in shell.stdout
    assert "policy=formation" in shell.stdout
    assert "n=3" in shell.stdout


def test_verify_runs_are_report_only_and_never_write_px4_params():
    # 数字只报数，不拿 L0 gates 当 SITL 通过条件。这里只核对落盘字段和默认关参。
    assert RUNS.is_file(), "缺少 docs/research/s5b-sitl-formation-verify-runs.json"
    payload = json.loads(RUNS.read_text())
    assert payload["ticket"] == "S5b"
    assert payload["policy"] == "formation"
    assert payload["scene"] == "formation-hold-sitl80"
    assert payload["n_drones"] == 3
    assert "判据未冻结" in payload["disclaimer"] or "只报数" in payload["disclaimer"]
    assert payload["l0"] and payload["sitl"]
    assert [row["seed"] for row in payload["l0"]] == [row["seed"] for row in payload["sitl"]]
    for row in payload["l0"]:
        assert row["tier"] == "L0"
        assert row["policy"] == "formation"
        assert row["arrived"] == 3
        assert row["scene"] == "formation-hold"
    for row in payload["sitl"]:
        assert row["tier"] == "SITL"
        assert row["policy"] == "formation"
        assert row["px4_parameters_written"] == []
        assert row["report_only"] is True
        assert row["criteria_frozen"] is False
        assert "formation_t0_s" in row
        assert "arrival_time_s" in row
        assert "rtf" in row
        assert "landed" in row
        assert "formation_error_p95_m" in row
        assert "min_inter_drone_distance_m" in row
