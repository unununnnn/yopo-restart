"""票 SH2：SITL 桥上的影子盾环境证据开关。默认关，打开不改设定点。假 PX4，不起 Gazebo。"""

import json
import subprocess

import numpy as np

from conftest import ROOT
from yopo_l0.scene import load_card
from yopo_l0.shadow_sitl import ENV_RESULT_KEYS, shadow_enabled
from yopo_l0.sitl_bridge import (
    FakePx4Vehicle,
    LiveFleetConfig,
    VehicleInbox,
    live_topic_plan,
    make_live_session,
    odometry_message_from_odometry,
)

DT = 0.02
CROSS = ROOT / "scenes/crossing-pillars-sitl80.json"
G2 = ROOT / "scenes/g2-pillar-sitl80.json"
SITL = ROOT / "scripts/sitl"


def _inbox(session, positions, *, armed=True, landed=False, t=0.0):
    out = {}
    for slot, p in zip(session.slots, positions):
        vehicle = FakePx4Vehicle(p)
        out[slot.ident["namespace"]] = VehicleInbox(
            odometry=odometry_message_from_odometry(vehicle.odometry(), int(round(t * 1e6))),
            armed=armed,
            offboard=True,
            landed=landed,
        )
    return out


def _starts(session):
    return [np.array(slot.bridge.start_enu, dtype=float) for slot in session.slots]


def _setpoints(session, t0: float, steps: int):
    starts = _starts(session)
    rows = []
    t = float(t0)
    for _ in range(steps):
        inbox = _inbox(session, starts, t=t)
        cmds = session.step(t, inbox)
        rows.append(
            [
                None if cmd.setpoint is None else json.dumps(cmd.setpoint, sort_keys=True)
                for cmd in cmds
            ]
        )
        t = round(t + DT, 10)
    return rows, t


def test_defaults_keep_crossing_shadow_off_and_add_no_fields():
    cfg = LiveFleetConfig()
    assert cfg.shadow_env_evidence is False
    assert cfg.fleet_shadow is False
    assert shadow_enabled(cfg, "upstream") is True
    assert shadow_enabled(cfg, "crossing") is False
    session = make_live_session(load_card(CROSS), seed=0, policy="crossing")
    assert session.shadow is False
    assert all(slot.shadow is None for slot in session.slots)
    starts = _starts(session)
    session.step(0.0, _inbox(session, starts))
    result = session.result()
    assert result["shadow"] is False
    assert result["shadow_reject_rate"] is None
    for key in ENV_RESULT_KEYS:
        assert key not in result
    assert "upstream_deviation" not in result


def test_dry_run_default_omits_evidence_and_flag_on_marks_deviation():
    plan = live_topic_plan(load_card(CROSS), policy="crossing")
    assert plan["shadow"] is False
    assert "shadow_env_evidence" not in plan
    shell = subprocess.run(
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
            "--lockstep",
            "--shadow-env-evidence",
            "--fleet-shadow",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert shell.returncode == 0, shell.stderr
    assert "shadow_env_evidence=1" in shell.stdout
    assert "fleet_shadow=1" in shell.stdout
    off = subprocess.run(
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
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert off.returncode == 0, off.stderr
    assert "shadow_env_evidence=0" in off.stdout
    assert "fleet_shadow=0" in off.stdout


def test_fleet_shadow_records_rejects_without_evidence_fields():
    session = make_live_session(
        load_card(CROSS),
        seed=0,
        policy="crossing",
        live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card", fleet_shadow=True),
    )
    assert session.shadow is True
    assert all(slot.shadow is not None for slot in session.slots)
    _setpoints(session, 0.0, 40)
    result = session.result()
    assert result["shadow"] is True
    assert isinstance(result["shadow_reject_rate"], float)
    assert 0.0 <= result["shadow_reject_rate"] <= 1.0
    assert result["shadow_reject_histogram"] is not None
    for key in ENV_RESULT_KEYS:
        assert key not in result
    assert "upstream_deviation" not in result
    assert result["px4_parameters_written"] == []


def test_evidence_switch_does_not_change_setpoints_and_tags_deviation():
    card = load_card(CROSS)
    live_off = LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card", fleet_shadow=True)
    live_on = LiveFleetConfig(
        initial_phase="PLAN",
        depth_source="scene-card",
        fleet_shadow=True,
        shadow_env_evidence=True,
    )
    off = make_live_session(card, seed=1, policy="crossing", live=live_off)
    on = make_live_session(card, seed=1, policy="crossing", live=live_on)
    rows_off, _ = _setpoints(off, 0.0, 50)
    rows_on, _ = _setpoints(on, 0.0, 50)
    assert rows_off == rows_on
    result = on.result()
    assert result["shadow_env_evidence"] is True
    assert result["upstream_deviation"] is True
    assert result["shadow_env_views"] >= 1
    assert result["shadow_env_clearance_evals"] >= 0
    off_result = off.result()
    for key in ENV_RESULT_KEYS:
        assert key not in off_result
    # 假 PX4 只在起点停 1 s，打开证据后环境码应减少，不必在这一小段前缀上清零。
    hist_on = result["shadow_reject_histogram"] or {}
    hist_off = off_result["shadow_reject_histogram"] or {}
    env_on = sum(v for k, v in hist_on.items() if str(k).startswith("ENV:"))
    env_off = sum(v for k, v in hist_off.items() if str(k).startswith("ENV:"))
    assert env_off > 0
    assert env_on < env_off


def test_upstream_default_unchanged_and_evidence_optional():
    session = make_live_session(
        load_card(G2),
        seed=0,
        live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card"),
    )
    assert session.shadow is True
    assert session.policy == "upstream"
    start = _starts(session)
    session.step(0.0, _inbox(session, start))
    result = session.result()
    assert result["shadow"] is True
    for key in ENV_RESULT_KEYS:
        assert key not in result

    on = make_live_session(
        load_card(G2),
        seed=0,
        live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card", shadow_env_evidence=True),
    )
    on.step(0.0, _inbox(on, _starts(on)))
    tagged = on.result()
    assert tagged["shadow_env_evidence"] is True
    assert tagged["upstream_deviation"] is True
    assert tagged["shadow_env_views"] >= 1


def test_shadow_false_wins_over_evidence_flag():
    session = make_live_session(
        load_card(CROSS),
        seed=0,
        policy="crossing",
        live=LiveFleetConfig(initial_phase="PLAN", shadow=False, shadow_env_evidence=True, fleet_shadow=True),
    )
    assert session.shadow is False
    assert all(slot.shadow is None for slot in session.slots)
