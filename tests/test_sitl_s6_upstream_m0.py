"""票 S6：M0 g2-pillar 的单机 SITL 对照卡，以及 upstream + 影子盾只读记录。"""

import importlib.util
import json
import subprocess
import sys

import numpy as np
import pytest

from conftest import ROOT
from yopo_l0.scene import load_card
from yopo_l0.sitl_bridge import (
    FakePx4Vehicle,
    LiveFleetConfig,
    VehicleInbox,
    live_topic_plan,
    make_live_session,
    odometry_message_from_odometry,
)

SITL = ROOT / "scripts/sitl"
BASE = ROOT / "scenes/g2-pillar.json"
SITL_CARD = ROOT / "scenes/g2-pillar-sitl80.json"
DT = 0.02


def _load_live():
    spec = importlib.util.spec_from_file_location("sitl_s6_yopo_l0_live", SITL / "yopo_l0_live.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _inbox(session, positions, velocity=None, *, armed=True, offboard=True, landed=False):
    inbox = {}
    vel = np.zeros(3) if velocity is None else np.asarray(velocity, dtype=float)
    for slot, position in zip(session.slots, positions):
        vehicle = FakePx4Vehicle(position, vel)
        inbox[slot.ident["namespace"]] = VehicleInbox(
            odometry=odometry_message_from_odometry(vehicle.odometry(), int(round(0 * 1e6))),
            armed=armed,
            offboard=offboard,
            landed=landed,
        )
    return inbox


def test_sitl_card_only_extends_duration():
    base = json.loads(BASE.read_text())
    sitl = json.loads(SITL_CARD.read_text())
    assert sitl["name"] == "g2-pillar-sitl80"
    assert sitl["task"]["duration_s"] == 80.0
    assert base["task"]["duration_s"] == 45.0
    note = sitl.pop("note")
    assert "duration_s" in note and "suite.txt" in note
    base["name"] = sitl["name"]
    base["task"]["duration_s"] = sitl["task"]["duration_s"]
    assert sitl == base
    suite = (ROOT / "scenes/suite.txt").read_text().lower()
    assert "g2-pillar-sitl80" not in suite
    assert "g2-pillar-sitl" not in suite


def test_dry_run_defaults_to_upstream_with_readonly_shadow():
    card = load_card(SITL_CARD)
    plan = live_topic_plan(card, positioning="default", seed=0, policy="upstream")
    assert plan["policy"] == "upstream"
    assert plan["shadow"] is True
    assert plan["n_drones"] == 1
    assert plan["scene"] == "g2-pillar-sitl80"
    assert plan["px4_parameters_written"] == []
    assert plan["namespaces"] == [""]


def test_takeoff_handover_and_shadow_ticks_are_recorded():
    session = make_live_session(
        load_card(SITL_CARD),
        seed=0,
        live=LiveFleetConfig(initial_phase="WARMUP", depth_source="scene-card", warmup_s=0.04),
    )
    assert session.policy == "upstream"
    assert session.shadow is True
    assert session.slots[0].shadow is not None
    start = np.array(session.slots[0].bridge.start_enu, dtype=float)
    hover = start.copy()
    # 先在地面（z 保持场景卡起点），走完预热和解锁。
    t = 0.0
    inbox = _inbox(session, [start], armed=False, offboard=False)
    while t < 0.2 + 1e-9:
        session.step(t, inbox)
        t = round(t + DT, 10)
    assert session.slots[0].phase == "ARMING"
    inbox = _inbox(session, [start], armed=True, offboard=True)
    session.step(t, inbox)
    t = round(t + DT, 10)
    assert session.slots[0].phase == "TAKEOFF"
    # 停在悬停点并保持 settle_hold_s。
    inbox = _inbox(session, [hover])
    end = t + 1.2
    while t < end + 1e-9:
        session.step(t, inbox)
        t = round(t + DT, 10)
    assert session.slots[0].phase == "PLAN"
    handover = session.slots[0].takeoff_handover_s
    assert handover is not None
    assert handover == pytest.approx(1.24, abs=0.05)
    # 再走几拍规划，影子盾应记下 would_reject。
    while t < handover + 0.3 + 1e-9:
        session.step(t, inbox)
        t = round(t + DT, 10)
    result = session.result()
    assert result["policy"] == "upstream"
    assert result["shadow"] is True
    assert result["takeoff_handover_s"] == pytest.approx(handover)
    assert result["collisions"] == 0
    assert result["rtf"] is not None
    assert isinstance(result["shadow_reject_rate"], float)
    assert 0.0 <= result["shadow_reject_rate"] <= 1.0
    assert result["shadow_drones"][0]["id"] == "uav0"
    assert result["px4_parameters_written"] == []
    assert result["drones"][0]["takeoff_handover_s"] == pytest.approx(handover)


def test_plan_start_has_zero_handover_and_no_shadow_when_disabled():
    session = make_live_session(
        load_card(SITL_CARD),
        seed=0,
        live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card", shadow=False),
    )
    assert session.shadow is False
    start = np.array(session.slots[0].bridge.start_enu, dtype=float)
    inbox = _inbox(session, [start])
    session.step(0.0, inbox)
    assert session.slots[0].takeoff_handover_s == 0.0
    result = session.result()
    assert result["shadow"] is False
    assert result["shadow_reject_rate"] is None
    assert result["takeoff_handover_s"] == 0.0


def test_run_script_dry_run_prints_policy_and_scene():
    shell = subprocess.run(
        [
            "bash",
            str(SITL / "run_yopo_l0_sitl.sh"),
            "--dry-run",
            "--n",
            "1",
            "--scene",
            "scenes/g2-pillar-sitl80.json",
            "--policy",
            "upstream",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert shell.returncode == 0, shell.stderr
    assert "scene=scenes/g2-pillar-sitl80.json" in shell.stdout
    assert "policy=upstream" in shell.stdout
    assert "n=1" in shell.stdout
