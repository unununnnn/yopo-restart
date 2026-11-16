"""票 BAT1：只测日志统计，不起 Gazebo、不写 PX4 参数。"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def _load():
    path = REPO / "scripts" / "sitl" / "bat1_collect.py"
    spec = importlib.util.spec_from_file_location("bat1_collect", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


_mod = _load()
STALE_BATTERY_S = _mod.STALE_BATTERY_S
collect_run = _mod.collect_run
count_console = _mod.count_console
stopped_at_arm = _mod.stopped_at_arm


def test_stale_window_matches_px4_battery_check() -> None:
    assert STALE_BATTERY_S == 5.0


def test_console_counts_unhealthy_and_failsafe() -> None:
    text = """
INFO  [commander] Armed by external command
INFO  [commander] Takeoff detected
WARN  [health_and_arming_checks] Preflight Fail: Battery unhealthy
INFO  [commander] Landing at current position
WARN  [health_and_arming_checks] Preflight Fail: Battery unhealthy
INFO  [tone_alarm] battery warning (fast)
WARN  [failsafe] Failsafe activated
"""
    assert count_console(text) == {
        "battery_unhealthy": 2,
        "failsafe_activated": 1,
        "battery_warning": 1,
    }


def test_stopped_at_arm_requires_last_sample_near_arm_and_flight_continuing() -> None:
    assert stopped_at_arm(arm_s=21.408, bat_last_s=21.416, duration_s=49.8) is True
    # 电池一直发到收尾：不是停在解锁。
    assert stopped_at_arm(arm_s=16.436, bat_last_s=49.6, duration_s=49.8) is False
    # 飞行没走到 5 s 门槛，不算。
    assert stopped_at_arm(arm_s=21.408, bat_last_s=21.416, duration_s=24.0) is False
    assert stopped_at_arm(None, 21.4, 40.0) is False


def test_collect_run_reads_console_from_a_run_dir(tmp_path: Path) -> None:
    (tmp_path / "px4-1.log").write_text(
        "INFO  [commander] Armed by external command\n", encoding="utf-8"
    )
    (tmp_path / "px4-2.log").write_text(
        "WARN  [health_and_arming_checks] Preflight Fail: Battery unhealthy\n"
        "WARN  [health_and_arming_checks] Preflight Fail: Battery unhealthy\n",
        encoding="utf-8",
    )
    (tmp_path / "result.json").write_text(
        json.dumps(
            {
                "scene": "crossing-pillars-sitl80",
                "outcome": "SUCCESS",
                "px4_parameters_written": [],
                "lockstep": {
                    "enabled": True,
                    "mechanism": "gz WorldControl pause+multi_step=1 per physics step + PX4 motor_speed handshake",
                    "t_engage_s": 15.408,
                },
            }
        ),
        encoding="utf-8",
    )
    out = collect_run(tmp_path)
    assert out["scene"] == "crossing-pillars-sitl80"
    assert out["lockstep_enabled"] is True
    assert out["px4_parameters_written"] == []
    assert out["console_unhealthy_vehicles"] == 1
    assert out["console_failsafe_vehicles"] == 0
    assert out["vehicles"][1]["console"]["battery_unhealthy"] == 2
