"""票 S5a：SITL 会话里的编队命令序列。假 PX4，不起 Gazebo。"""

import importlib.util
import json
import subprocess
import sys

import numpy as np
import pytest

from yopo_swarm_core import px4

from conftest import ROOT
from yopo_l0.formation import FormationFleet
from yopo_l0.scene import load_card
from yopo_l0.sitl_bridge import (
    FakePx4Vehicle,
    LiveFleetConfig,
    VehicleInbox,
    make_live_session,
    odometry_message_from_odometry,
    resolve_live_policy,
)

SITL = ROOT / "scripts/sitl"
HOLD = ROOT / "scenes/formation-hold.json"
SITL_CARD = ROOT / "scenes/formation-hold-sitl80.json"
G2 = ROOT / "scenes/g2-pillar.json"
DT = 0.02


def _load_live():
    spec = importlib.util.spec_from_file_location("sitl_s5a_yopo_l0_live", SITL / "yopo_l0_live.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _session(card=None):
    return make_live_session(
        load_card(card or SITL_CARD),
        seed=0,
        policy="formation",
        live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card", warmup_s=5.0),
    )


def _inbox(session, positions, velocity=None):
    inbox = {}
    vel = np.zeros(3) if velocity is None else np.asarray(velocity, dtype=float)
    for slot, position in zip(session.slots, positions):
        vehicle = FakePx4Vehicle(position, vel)
        inbox[slot.ident["namespace"]] = VehicleInbox(
            odometry=odometry_message_from_odometry(vehicle.odometry(), int(round(0 * 1e6))),
            armed=True,
            offboard=True,
            landed=False,
        )
    return inbox


def _starts(session):
    return [np.zeros(3) + np.array([0.0, 0.0, slot.bridge.start_enu[2]]) for slot in session.slots]


def _world(cmd, slot):
    return np.asarray(px4.ned_to_enu(cmd.setpoint["position"]), dtype=float) + slot.offset_enu


def _step_grid(session, positions, t_end):
    inbox = _inbox(session, positions)
    t = 0.0
    last = None
    while t < t_end + 1e-9:
        last = session.step(t, inbox)
        t = round(t + DT, 10)
    return last


def test_sitl_card_only_extends_duration():
    base = json.loads(HOLD.read_text())
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
    assert "sitl" not in suite


def test_policy_must_match_the_card():
    hold = load_card(HOLD)
    g2 = load_card(G2)
    assert resolve_live_policy(hold, "formation") == "formation"
    assert resolve_live_policy(g2, "upstream") == "upstream"
    with pytest.raises(ValueError, match="policy=formation"):
        make_live_session(g2, policy="formation")
    with pytest.raises(ValueError, match="formation"):
        make_live_session(hold)
    upstream = make_live_session(g2, seed=0)
    assert upstream.policy == "upstream"
    assert upstream.fleet is None
    assert upstream.slots[0].bridge.policy is not None


def test_formation_commands_follow_the_committed_schedule_on_the_inprocess_bus():
    assert FormationFleet.transport_override is None
    session = _session()
    assert session.policy == "formation"
    assert session.fleet is not None and session.bus is not None
    assert FormationFleet.transport_override is None
    for slot in session.slots:
        assert slot.bridge.policy is None
        assert np.linalg.norm(slot.bridge.goal - slot.bridge.start_enu) > 5.0
        assert np.allclose(slot.bridge.goal, session.fleet.goal_of(slot.bridge.drone_id))
    positions = _starts(session)
    early = session.step(0.0, _inbox(session, positions))
    published = session.bus.published
    assert published > 0 and session.bus.delivered > 0
    assert session.step(0.0, _inbox(session, positions))[0].phase == "PLAN"
    assert session.bus.published == published
    for cmd, slot in zip(early, session.slots):
        assert cmd.phase == "PLAN"
        assert cmd.heartbeat is True
        assert cmd.vehicle_commands == ()
        assert cmd.planned is False
        assert cmd.depth_source == "formation"
        assert np.allclose(cmd.setpoint["velocity"], 0.0, atol=1e-6)
        assert np.allclose(_world(cmd, slot), slot.bridge.start_enu, atol=1e-5)
        assert "protocol" not in "".join(cmd.topics.values())
    _step_grid(session, positions, 1.0)
    for cmd, slot in zip(session._last_cmds, session.slots):
        assert float(_world(cmd, slot)[0]) < 0.05
    jumped = _session()
    jumped.step(0.0, _inbox(jumped, _starts(jumped)))
    jumped.step(6.0, _inbox(jumped, _starts(jumped)))
    stepped = _session()
    cmds = _step_grid(stepped, _starts(stepped), 6.0)
    summary = stepped.fleet.summary(0, 3)
    assert summary["local_commits"] == {"uav0": 1, "uav1": 1, "uav2": 1}
    assert summary["proposals"] == 3 and summary["acks"] == 6 and summary["nacks"] == 0
    for cmd, other, slot, jumped_slot in zip(cmds, jumped._last_cmds, stepped.slots, jumped.slots):
        assert cmd.planned is True
        assert cmd.vehicle_commands == ()
        assert stepped.fleet.code(slot.bridge.drone_id) == "LOCAL_APPROVED"
        world = _world(cmd, slot)
        assert np.allclose(world, _world(other, jumped_slot), atol=1e-6)
        sampler = slot.formation_sampler
        p, v, _a = sampler.pva(stepped._formation_t)
        assert np.allclose(world, p, atol=1e-6)
        assert np.allclose(px4.ned_to_enu(cmd.setpoint["velocity"]), v, atol=1e-6)
        assert float(world[0]) > 1.0
    assert float(_world(cmds[1], stepped.slots[1])[1]) == pytest.approx(2.0, abs=0.3)
    tracked = stepped.stream_local_setpoint(stepped.slots[0].ident["namespace"], 8.0)
    assert tracked is not None
    ahead = np.asarray(px4.ned_to_enu(tracked["position"]), dtype=float) + stepped.slots[0].offset_enu
    assert float(ahead[0]) > float(_world(cmds[0], stepped.slots[0])[0])
    assert float(np.linalg.norm(tracked["velocity"])) > 0.2
    mode, held = _load_live().streamed_setpoint(cmds[0], tracked)
    assert mode["timestamp"] == tracked["timestamp"]
    assert held["velocity"] == tracked["velocity"]
    assert held["acceleration"] == tracked["acceleration"]
    result = stepped.result()
    assert result["policy"] == "formation"
    assert result["inter_drone_transport"] == "inprocess"
    assert result["px4_parameters_written"] == []
    assert result["formation"]["t_go_s"] == pytest.approx(1.5)
    assert result["bus"]["delivered"] > 0
    assert all(sum(slot.bridge.plan_counts.values()) == 0 for slot in stepped.slots)
    assert result["min_inter_drone_distance_m"] == pytest.approx(2.0)
    assert FormationFleet.transport_override is None


def test_result_before_first_plan_has_no_plan_code():
    session = _session()
    result = session.result()
    assert result["outcome"] == "RUNNING"
    assert all(drone["plan_code"] is None for drone in result["drones"])
    assert all(drone["id"] in ("uav0", "uav1", "uav2") for drone in result["drones"])


def test_fake_px4_tracks_the_straight_segment_without_closing_the_gap():
    session = _session()
    vehicles = [FakePx4Vehicle(np.array([0.0, 0.0, 1.5])) for _ in session.slots]
    t = 0.0
    while t < 8.0 + 1e-9:
        inbox = {}
        for slot, vehicle in zip(session.slots, vehicles):
            inbox[slot.ident["namespace"]] = VehicleInbox(
                odometry=odometry_message_from_odometry(vehicle.odometry(), int(round(t * 1e6))),
                armed=True,
                offboard=True,
                landed=False,
            )
        commands = session.step(t, inbox)
        for vehicle, command in zip(vehicles, commands):
            assert command.vehicle_commands == ()
            vehicle.step(command.setpoint, DT)
        t = round(t + DT, 10)
    worlds = []
    for slot, vehicle in zip(session.slots, vehicles):
        local = np.asarray(px4.ned_to_enu(vehicle.p_ned), dtype=float)
        worlds.append(local + slot.offset_enu)
    assert session.fail_reason == ""
    assert float(worlds[0][0]) > 1.0
    assert abs(float(worlds[1][1]) - 2.0) < 0.5
    gaps = [float(np.linalg.norm(worlds[i] - worlds[j])) for i in range(3) for j in range(i + 1, 3)]
    result = session.result()
    assert min(gaps) > 1.5
    assert result["min_inter_drone_distance_m"] > 1.5
    assert result["formation_error_p95_m"] < 0.5
    assert result["outcome"] == "RUNNING"


def test_goal_hold_emits_nav_land_and_does_not_write_parameters():
    session = _session()
    tol = session.goal_tol
    assert tol == pytest.approx(0.4)
    assert session.goal_speed == pytest.approx(0.2)
    assert session.goal_hold == pytest.approx(1.0)
    positions = [slot.bridge.goal - slot.offset_enu for slot in session.slots]
    t = 0.0
    saw_land = False
    while t < 1.2 + 1e-9:
        commands = session.step(t, _inbox(session, positions))
        for command in commands:
            assert command.mav_sys_id in (2, 3, 4)
            for fields in command.vehicle_commands:
                assert int(fields["command"]) == 21
                assert fields["target_system"] == command.mav_sys_id
                saw_land = True
        t = round(t + DT, 10)
    assert saw_land
    assert all(slot.arrival is not None for slot in session.slots)
    # 上锁且已落地才收尾。循环里的 inbox 仍是 armed。
    inbox = _inbox(session, positions)
    for box in inbox.values():
        box.landed = True
        box.armed = False
    done = session.step(t, inbox)
    assert all(command.phase == "DONE" for command in done)
    result = session.result()
    assert result["outcome"] == "SUCCESS"
    assert result["reason"] == "GOAL_REACHED_AND_HELD"
    assert result["ok"] is True
    assert result["arrived"] == 3
    assert result["landed"] is True
    assert result["px4_parameters_written"] == []
    assert result["formation_t0_s"] == pytest.approx(0.0)


def test_dry_run_lists_formation_without_a_protocol_topic(tmp_path):
    mod = _load_live()
    out = tmp_path / "dry.json"
    rc = mod.main([
        "--dry-run",
        "--scene", "scenes/formation-hold-sitl80.json",
        "--n", "3",
        "--policy", "formation",
        "--out", str(out),
    ])
    assert rc == 0
    data = json.loads(out.read_text())
    assert data["dry_run"] is True
    assert data["policy"] == "formation"
    assert data["inter_drone_transport"] == "inprocess"
    assert data["n_drones"] == 3
    assert data["px4_parameters_written"] == []
    assert data["namespaces"] == ["px4_1", "px4_2", "px4_3"]
    blob = json.dumps(data["topics"])
    assert "protocol" not in blob
    assert "/px4_1/fmu/in/trajectory_setpoint" in blob
    bad = mod.main([
        "--dry-run",
        "--scene", "scenes/formation-hold.json",
        "--policy", "upstream",
        "--out", str(tmp_path / "bad.json"),
    ])
    assert bad == 2
    mismatch = mod.main([
        "--dry-run",
        "--scene", "scenes/g2-pillar.json",
        "--policy", "formation",
        "--out", str(tmp_path / "g2.json"),
    ])
    assert mismatch == 2
    shell = subprocess.run(
        [
            "bash", str(SITL / "run_yopo_l0_sitl.sh"),
            "--dry-run", "--n", "3",
            "--scene", "scenes/formation-hold-sitl80.json",
            "--policy", "formation",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert shell.returncode == 0, shell.stderr
    assert "policy=formation" in shell.stdout
    assert "scene=scenes/formation-hold-sitl80.json" in shell.stdout
    assert "n=3" in shell.stdout
    default = subprocess.run(
        ["bash", str(SITL / "run_yopo_l0_sitl.sh"), "--dry-run", "--n", "1"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert default.returncode == 0, default.stderr
    assert "policy=upstream" in default.stdout
    rejected = subprocess.run(
        ["bash", str(SITL / "run_yopo_l0_sitl.sh"), "--dry-run", "--n", "3", "--policy", "no-such-policy"],  # 票 S7 起 crossing / switch 合法
        capture_output=True,
        text=True,
        check=False,
    )
    assert rejected.returncode == 2
    container = (SITL / "in_container.sh").read_text()
    mission = (SITL / "exec_mission.sh").read_text()
    entry = (SITL / "run_yopo_l0_sitl.sh").read_text()
    assert '--policy "${YOPO_L0_POLICY:-upstream}"' in container
    assert '--policy "${YOPO_L0_POLICY:-upstream}"' in mission
    assert 'YOPO_L0_POLICY="$POLICY"' in entry
    assert "PX4_PARAM_" not in entry
    assert "param set" not in entry
