"""票 SH5：SITL 重测影子对齐开关。默认关，不改 0.15 m，不起 Gazebo。"""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys

from conftest import ROOT
from yopo_l0.runner import RunConfig
from yopo_l0.shadow import ORACLE_TRACKING_BOUND
from yopo_l0.shadow_sitl import shadow_enabled
from yopo_l0.sitl_bridge import LiveFleetConfig

SITL = ROOT / "scripts/sitl"


def _load_sh5():
    path = ROOT / "scripts/sh5-sitl-shadow-align.py"
    spec = importlib.util.spec_from_file_location("sh5_sitl_shadow_align", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _dry_run(extra: list[str]) -> str:
    proc = subprocess.run(
        [
            "bash",
            str(SITL / "run_yopo_l0_sitl.sh"),
            "--dry-run",
            "--n",
            "3",
            "--scene",
            "scenes/crossing-pillars-sitl80.json",
            "--policy",
            "crossing",
            *extra,
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert proc.returncode == 0, proc.stderr
    return proc.stdout


def _flag_map(stdout: str) -> dict[str, str]:
    out = {}
    for line in stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            out[key] = value
    return out


def test_research_switches_default_off_and_bound_stays_015():
    cfg = LiveFleetConfig()
    run = RunConfig()
    assert cfg.shadow_env_evidence is False
    assert cfg.fleet_shadow is False
    assert cfg.shadow_contract_align is False
    assert cfg.shadow_tracking_align is False
    assert run.shadow_env_evidence is False
    assert run.shadow_contract_align is False
    assert run.shadow_tracking_align is False
    assert ORACLE_TRACKING_BOUND == 0.15
    assert shadow_enabled(cfg, "crossing") is False
    assert shadow_enabled(cfg, "upstream") is True


def test_dry_run_defaults_keep_all_research_switches_off():
    flags = _flag_map(_dry_run([]))
    assert flags["lockstep"] == "0"
    assert flags["fleet_shadow"] == "0"
    assert flags["shadow_env_evidence"] == "0"
    assert flags["shadow_contract_align"] == "0"
    assert flags["shadow_tracking_align"] == "0"


def test_dry_run_four_arms_match_sh5_argv():
    sh5 = _load_sh5()
    expected = {
        "default": {
            "lockstep": "1",
            "fleet_shadow": "1",
            "shadow_env_evidence": "0",
            "shadow_contract_align": "0",
            "shadow_tracking_align": "0",
        },
        "env": {
            "lockstep": "1",
            "fleet_shadow": "1",
            "shadow_env_evidence": "1",
            "shadow_contract_align": "0",
            "shadow_tracking_align": "0",
        },
        "env_contract": {
            "lockstep": "1",
            "fleet_shadow": "1",
            "shadow_env_evidence": "1",
            "shadow_contract_align": "1",
            "shadow_tracking_align": "0",
        },
        "env_contract_tracking": {
            "lockstep": "1",
            "fleet_shadow": "1",
            "shadow_env_evidence": "1",
            "shadow_contract_align": "1",
            "shadow_tracking_align": "1",
        },
    }
    for arm, want in expected.items():
        argv = sh5.sitl_argv("crossing-pillars-sitl80", 0, arm)
        assert argv[:2] == ["bash", str(ROOT / "scripts/sitl/run_yopo_l0_sitl.sh")]
        assert "--lockstep" in argv
        assert "--fleet-shadow" in argv
        assert ("--shadow-env-evidence" in argv) is (want["shadow_env_evidence"] == "1")
        assert ("--shadow-contract-align" in argv) is (want["shadow_contract_align"] == "1")
        assert ("--shadow-tracking-align" in argv) is (want["shadow_tracking_align"] == "1")
    extras = {
        "default": ["--lockstep", "--fleet-shadow"],
        "env": ["--lockstep", "--fleet-shadow", "--shadow-env-evidence"],
        "env_contract": [
            "--lockstep",
            "--fleet-shadow",
            "--shadow-env-evidence",
            "--shadow-contract-align",
        ],
        "env_contract_tracking": [
            "--lockstep",
            "--fleet-shadow",
            "--shadow-env-evidence",
            "--shadow-contract-align",
            "--shadow-tracking-align",
        ],
    }
    for arm, extra in extras.items():
        flags = _flag_map(_dry_run(extra))
        assert flags == {**flags, **expected[arm]}
        for key, value in expected[arm].items():
            assert flags[key] == value, (arm, key, flags)


def test_compact_row_and_aggregate_four_arms(tmp_path):
    sh5 = _load_sh5()
    result = {
        "outcome": "SUCCESS",
        "reason": "GOAL_REACHED_AND_HELD",
        "arrived": 3,
        "n_drones": 3,
        "collisions": 0,
        "min_surface_clearance_plan_m": 0.4,
        "px4_parameters_written": [],
        "lockstep": {"clock_ahead": 0},
        "upstream_deviation": True,
        "shadow_env_evidence": True,
        "shadow_contract_align": True,
        "shadow_tracking_align": True,
        "shadow_reject_rate": 0.2,
        "shadow_attribution": {
            "ticks": 1000,
            "would_reject": 200,
            "would_reject_rate": 0.2,
            "env_reject": 10,
            "env_reject_rate": 0.01,
            "histogram": {"TRACKING_BOUND": 400, "ENV:OCCUPIED": 10, "TRANSITION:MOTION_KINEMATICS": 200},
            "family_ticks": {"TRACKING": 400, "ENV": 10, "TRANSITION": 200},
        },
    }
    row = sh5.compact_row(
        scene="crossing-pillars-sitl80",
        seed=2,
        arm="env_contract_tracking",
        run_dir="/workspace/yopo-ops/sh5/sitl/runs/demo",
        rc=0,
        result=result,
        network="yopo-sitl-demo",
    )
    assert row["tracking_bound_count"] == 400
    assert row["would_reject_rate"] == 0.2
    assert row["env_reject_rate"] == 0.01
    assert row["shadow_tracking_align"] is True
    assert row["px4_parameters_written"] == []
    assert row["clock_ahead"] == 0

    off = dict(result)
    off["upstream_deviation"] = False
    off["shadow_env_evidence"] = False
    off["shadow_contract_align"] = False
    off["shadow_tracking_align"] = False
    off["shadow_reject_rate"] = 1.0
    off["shadow_attribution"] = {
        "ticks": 1000,
        "would_reject": 1000,
        "would_reject_rate": 1.0,
        "env_reject": 900,
        "env_reject_rate": 0.9,
        "histogram": {"ENV:UNKNOWN": 900, "TRACKING_BOUND": 500},
        "family_ticks": {"ENV": 900, "TRACKING": 500},
    }
    off_row = sh5.compact_row(
        scene="crossing-pillars-sitl80",
        seed=2,
        arm="default",
        run_dir="/workspace/yopo-ops/sh5/sitl/runs/demo-off",
        rc=0,
        result=off,
        network="yopo-sitl-demo",
    )
    dest = tmp_path / "sitl"
    dest.mkdir()
    (dest / "crossing-pillars-sitl80-seed2-env_contract_tracking.json").write_text(
        json.dumps(row, ensure_ascii=False, indent=2) + "\n"
    )
    (dest / "crossing-pillars-sitl80-seed2-default.json").write_text(
        json.dumps(off_row, ensure_ascii=False, indent=2) + "\n"
    )
    rows = sh5.collect_rows(dest)
    assert [r["arm"] for r in rows] == ["default", "env_contract_tracking"]
    payload = sh5.build_payload(rows)
    arms = payload["scenes"]["crossing-pillars-sitl80"]["arms"]
    default_sum = arms["default"]["summary"]
    on_sum = arms["env_contract_tracking"]["summary"]
    assert default_sum["would_reject_rate"] == 1.0
    assert default_sum["env_reject_rate"] == 0.9
    assert default_sum["tracking_bound_count"] == 500
    assert on_sum["would_reject_rate"] == 0.2
    assert on_sum["tracking_bound_count"] == 400
    assert on_sum["success_trips"] == 1
    assert payload["note"].startswith("票 SH5")


def test_parse_launcher_output_extracts_run_dir_and_network():
    sh5 = _load_sh5()
    run_dir, network = sh5.parse_launcher_output(
        "hello\nrun_dir /workspace/yopo-ops/sh5/sitl/runs/20261116T010000-yopo-n3 rc=0\n"
        "sitl_network yopo-sitl-20261116T010000-1-2 owned=1\n"
    )
    assert run_dir.endswith("20261116T010000-yopo-n3")
    assert network == "yopo-sitl-20261116T010000-1-2"
    assert sh5.parse_launcher_output("nope\n") == (None, None)


def test_attribution_respects_tracking_align_would_reject():
    from yopo_l0.shadow_sitl import attribution

    ticks = [
        {"uav": "uav0", "would_reject": False, "reject_codes": ["TRACKING_BOUND"]},
        {"uav": "uav0", "would_reject": True, "reject_codes": ["ENV:OCCUPIED", "TRACKING_BOUND"]},
        {"uav": "uav0", "would_reject": False, "reject_codes": []},
        {"uav": "uav0", "would_reject": True, "reject_codes": ["TRANSITION:MOTION_KINEMATICS"]},
    ]
    attr = attribution(ticks)
    assert attr["ticks"] == 4
    assert attr["would_reject"] == 2
    assert attr["would_reject_rate"] == 0.5
    assert attr["tracking_bound_count"] if False else attr["histogram"]["TRACKING_BOUND"] == 2
    assert attr["env_reject"] == 1
    # 缺 would_reject 字段时，仍按是否有码计算（兼容旧日志）。
    legacy = attribution([{"reject_codes": ["TRACKING_BOUND"]}])
    assert legacy["would_reject"] == 1
