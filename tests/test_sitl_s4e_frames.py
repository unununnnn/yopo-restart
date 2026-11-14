"""票 S4e：三机本地里程计抬到场景卡世界系，结果以到达为准。不连 PX4。"""

import importlib.util
import math
import subprocess
import sys
import time

import numpy as np
import pytest

from yopo_swarm_core import px4

from conftest import ROOT
from yopo_l0.scene import load_card
from yopo_l0.sitl_bridge import (
    FakePx4Vehicle,
    LiveFleetConfig,
    VehicleInbox,
    apply_world_offset,
    decode_vehicle_odometry,
    frame_note,
    make_live_session,
    odometry_message_from_odometry,
    outcome_exit_code,
    realtime_factor,
    setpoint_to_local,
    sitl_line_spawn_xy,
)

SITL = ROOT / "scripts/sitl"
SWAP = ROOT / "scenes/swap-3-open.json"
G2 = ROOT / "scenes/g2-pillar.json"


def _load_live():
    spec = importlib.util.spec_from_file_location("sitl_s4e_yopo_l0_live", SITL / "yopo_l0_live.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _fleet():
    spec = importlib.util.spec_from_file_location("sitl_s4e_fleet", SITL / "fleet.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _msg(position, velocity=None, yaw=0.0, timestamp_us=0):
    vehicle = FakePx4Vehicle(position, velocity, yaw_enu=yaw)
    return odometry_message_from_odometry(vehicle.odometry(), timestamp_us)


def _session(card, phase):
    return make_live_session(
        card,
        seed=0,
        live=LiveFleetConfig(initial_phase=phase, depth_source="scene-card", warmup_s=5.0),
    )


def _inbox(session, positions, velocity=None, yaw=0.0, timestamp_us=0, **flags):
    inbox = {}
    for slot, position in zip(session.slots, positions):
        inbox[slot.ident["namespace"]] = VehicleInbox(
            odometry=_msg(position, velocity, yaw=yaw, timestamp_us=timestamp_us),
            armed=flags.get("armed", True),
            offboard=flags.get("offboard", True),
            landed=flags.get("landed", False),
        )
    return inbox


def test_world_offset_shifts_position_and_leaves_velocity():
    decoded = decode_vehicle_odometry(_msg([0.1, -0.2, 1.2], [0.3, 0.0, -0.05], yaw=0.2), now_s=0.0)
    world = apply_world_offset(decoded, [8.0, 0.0, 0.0])
    assert np.allclose(world["position_enu"], [8.1, -0.2, 1.2])
    assert np.allclose(world["velocity_enu"], decoded["velocity_enu"])
    assert np.allclose(world["acceleration_enu"], decoded["acceleration_enu"])
    assert world["yaw_enu"] == pytest.approx(decoded["yaw_enu"])
    assert np.allclose(world["position_ned"], np.asarray(decoded["position_ned"]) + px4.enu_to_ned([8.0, 0.0, 0.0]))
    assert np.allclose(px4.ned_to_enu(world["position_ned"]), world["position_enu"])
    assert np.allclose(decoded["position_enu"], [0.1, -0.2, 1.2])


def test_setpoint_to_local_subtracts_only_finite_position():
    original = {
        "position": [float("nan"), 8.0, -1.5],
        "velocity": [0.0, 1.25, 0.0],
        "acceleration": [0.0, 0.0, -0.2],
        "yaw": 0.4,
        "jerk": [float("nan")] * 3,
    }
    local = setpoint_to_local(original, [8.0, 0.0, 0.0])
    assert math.isnan(local["position"][0])
    assert local["position"][1] == pytest.approx(0.0)
    assert local["position"][2] == pytest.approx(-1.5)
    assert local["velocity"] == [0.0, 1.25, 0.0]
    assert local["acceleration"] == [0.0, 0.0, -0.2]
    assert local["yaw"] == 0.4
    assert math.isnan(local["jerk"][0])
    assert original["position"][1] == 8.0
    assert setpoint_to_local(None, [1.0, 0.0, 0.0]) is None


def test_three_local_odometries_become_scene_starts_and_setpoints_shrink():
    """三架假里程计都在自己的原点附近。世界系落到场景卡起点，设定点减回本地系。"""
    card = load_card(SWAP)
    session = _session(card, "WARMUP")
    velocity = np.array([0.25, -0.05, 0.0])
    yaw = 0.2
    locals_ = [
        np.array([0.10, -0.05, 0.50]),
        np.array([-0.20, 0.15, 0.55]),
        np.array([0.05, 0.08, 0.60]),
    ]
    commands = session.step(0.0, _inbox(session, locals_, velocity, yaw=yaw))
    assert session.fail_reason == ""
    worlds = []
    for slot, command, local in zip(session.slots, commands, locals_):
        offset = np.array([slot.bridge.start_enu[0], slot.bridge.start_enu[1], 0.0])
        assert np.allclose(slot.offset_enu, offset)
        world = local + offset
        worlds.append(world)
        assert np.allclose(command.position_enu, world, atol=1e-9)
        assert np.allclose(command.setpoint["position"], px4.enu_to_ned(local), atol=1e-9)
        assert np.allclose(command.setpoint["velocity"], 0.0, atol=1e-12)
        assert command.setpoint["yaw"] == pytest.approx(px4.yaw_enu_to_ned(yaw))
        assert np.allclose(slot.velocity, velocity)
    gaps = [
        float(np.linalg.norm(worlds[i] - worlds[j]))
        for i in range(3)
        for j in range(i + 1, 3)
    ]
    result = session.result()
    assert result["min_inter_drone_distance_m"] == pytest.approx(min(gaps))
    assert result["min_surface_clearance_m"] == pytest.approx(0.50 - 0.35)
    assert result["min_inter_drone_distance_m"] > 0.7
    assert "以场景卡为准" in result["frame_note"]
    assert "(8, 0)" in result["frame_note"]
    assert "(0, 3)" in result["frame_note"] and "(0, 6)" in result["frame_note"]
    for row, slot in zip(result["drones"], session.slots):
        assert row["world_offset_enu"] == pytest.approx(list(slot.offset_enu))


def test_takeoff_at_local_origins_is_not_inter_drone():
    card = load_card(SWAP)
    session = _session(card, "WARMUP")
    for slot in session.slots:
        slot.phase = "TAKEOFF"
    locals_ = [np.array([0.0, 0.0, 1.0]) for _ in session.slots]
    session.step(0.0, _inbox(session, locals_))
    assert session.fail_reason == ""
    assert all(slot.phase == "TAKEOFF" for slot in session.slots)
    result = session.result()
    assert result["outcome"] == "RUNNING"
    assert result["ok"] is False
    assert result["terminal"] is False
    assert result["min_inter_drone_distance_m"] == pytest.approx(8.0 * math.sqrt(3.0))
    assert outcome_exit_code(result) == 1


def test_g2_offset_is_identity_and_matches_the_line_spawn():
    card = load_card(G2)
    assert frame_note(card) == ""
    assert sitl_line_spawn_xy(1) == [(0.0, 0.0)]
    session = _session(card, "WARMUP")
    local = np.array([0.2, -0.1, 0.4])
    command = session.step(0.0, _inbox(session, [local]))[0]
    assert np.allclose(session.slots[0].offset_enu, 0.0)
    assert np.allclose(command.position_enu, local)
    assert np.allclose(command.setpoint["position"], px4.enu_to_ned(local), atol=1e-12)
    assert session.result()["frame_note"] == ""


def test_result_success_requires_every_arrival_and_exit_code_follows():
    session = _session(load_card(SWAP), "PLAN")
    fresh = session.result()
    assert fresh["outcome"] == "RUNNING"
    assert fresh["ok"] is False
    assert fresh["terminal"] is False
    assert fresh["rtf"] is None
    assert fresh["min_inter_drone_distance_m"] is None
    assert fresh["min_surface_clearance_m"] is None
    assert outcome_exit_code(fresh) == 1

    for slot in session.slots:
        slot.arrival = 1.0
        slot.phase = "PLAN"
    holding = session.result()
    assert holding["outcome"] == "RUNNING"
    assert holding["ok"] is False
    assert outcome_exit_code(holding) == 1

    for slot in session.slots:
        slot.phase = "DONE"
        slot.landed_done = True
    done = session.result()
    assert done["outcome"] == "SUCCESS"
    assert done["reason"] == "GOAL_REACHED_AND_HELD"
    assert done["ok"] is True
    assert done["terminal"] is True
    assert done["arrived"] == 3
    assert outcome_exit_code(done) == 0
    assert outcome_exit_code({"outcome": "SUCCESS", "ok": False}) == 1

    for slot in session.slots:
        slot.arrival = None
    missed = session.result()
    assert missed["outcome"] == "FAIL"
    assert missed["reason"] == "GOAL_NOT_REACHED"
    assert missed["ok"] is False
    assert outcome_exit_code(missed) == 1


def test_inter_drone_survives_landing_and_is_not_rewritten():
    card = load_card(SWAP)
    session = _session(card, "WARMUP")
    for slot in session.slots:
        slot.phase = "TAKEOFF"
    locals_ = [np.array([-slot.offset_enu[0], -slot.offset_enu[1], 1.0]) for slot in session.slots]
    session.step(0.0, _inbox(session, locals_, timestamp_us=0))
    assert session.fail_reason == "uav0-uav1:INTER_DRONE"
    assert all(slot.phase == "FAIL" for slot in session.slots)
    assert sum(sum(slot.bridge.plan_counts.values()) for slot in session.slots) == 0
    flying = session.result()
    assert flying["outcome"] == "FAIL"
    assert flying["reason"] == "uav0-uav1:INTER_DRONE"
    assert flying["arrived"] == 0
    assert flying["ok"] is False
    assert flying["terminal"] is False
    assert flying["min_inter_drone_distance_m"] == pytest.approx(0.0, abs=1e-9)
    assert outcome_exit_code(flying) == 1

    session.step(0.05, _inbox(session, locals_, timestamp_us=50000, landed=True, armed=False))
    assert all(slot.phase == "FAIL" for slot in session.slots)
    assert all(slot.landed_done for slot in session.slots)
    session._fail("uav2:PLAN_TIMEOUT", 1.0)
    assert session.fail_reason == "uav0-uav1:INTER_DRONE"
    assert all(slot.land_sent for slot in session.slots)
    for slot in session.slots:
        slot.phase = "DONE"
        slot.arrival = 1.0
    landed = session.result()
    assert landed["outcome"] == "FAIL"
    assert landed["reason"] == "uav0-uav1:INTER_DRONE"
    assert landed["ok"] is False
    assert landed["terminal"] is True
    assert landed["landed"] is True
    assert landed["arrived"] == 3
    assert outcome_exit_code(landed) == 1


def test_rtf_is_sim_over_wall():
    assert realtime_factor(None, 1.0) is None
    assert realtime_factor(1.0, 0.0) is None
    assert realtime_factor(2.0, 4.0) == pytest.approx(0.5)
    session = _session(load_card(G2), "WARMUP")
    session._sim0 = 10.0
    session._sim1 = 12.0
    session._wall0 = time.perf_counter() - 4.0
    result = session.result()
    assert result["sim_time_s"] == pytest.approx(2.0)
    assert result["rtf"] == pytest.approx(2.0 / result["wall_time_s"])
    assert result["rtf"] == pytest.approx(0.5, rel=0.02)


def test_scene_card_spawn_replaces_the_s2_line_without_changing_the_default():
    fleet = _fleet()
    shell = fleet.format_shell(3)
    assert [line for line in shell.splitlines() if line.startswith("pose=")] == ["pose=0,0", "pose=0,3", "pose=0,6"]
    assert fleet.SPAWN_SPACING_M == 3.0
    assert sitl_line_spawn_xy(3) == [(0.0, 0.0), (0.0, 3.0), (0.0, 6.0)]
    assert [tuple(spec.spawn_xy_m) for spec in fleet.fleet_specs(3)] == sitl_line_spawn_xy(3)
    rewritten = fleet.apply_scene_spawn_poses(shell, SWAP, 3)
    assert fleet.format_shell(3) == shell
    poses = [line.split("=", 1)[1] for line in rewritten.splitlines() if line.startswith("pose=")]
    parsed = [tuple(float(part) for part in pose.split(",")) for pose in poses]
    starts = [(8.0, 0.0), (-4.0, 6.928203230275509), (-4.0, -6.928203230275509)]
    for got, want in zip(parsed, starts, strict=True):
        assert got == pytest.approx(want, abs=1e-8)
    assert parsed != [(0.0, 0.0), (0.0, 3.0), (0.0, 6.0)]
    assert "namespace=px4_1" in rewritten and "namespace=px4_3" in rewritten
    assert "PX4_PARAM_" not in rewritten
    with pytest.raises(ValueError):
        fleet.apply_scene_spawn_poses(shell, SWAP, 2)

    container = (SITL / "in_container.sh").read_text()
    assert 'yopo-l0" && "$N" -gt 1' in container
    assert "yopo-poses" in container
    assert 'fleet.py --n "$N" --format shell' in container
    subprocess.run(["bash", "-n", str(SITL / "in_container.sh")], check=True)


def test_diagonal_spawn_offset_roundtrips_through_ned_without_swapping_axes():
    """uav1 的起点不在坐标轴上。偏置减错轴的话，本地目标不会落在直径对侧。"""
    offset = np.array([-4.0, 6.928203230275509, 0.0])
    goal = np.array([4.0, -6.928203230275509, 1.5])
    local_enu = goal - offset
    assert local_enu == pytest.approx([8.0, -13.856406460551018, 1.5])
    setpoint = {
        "position": [float(x) for x in px4.enu_to_ned(goal)],
        "velocity": [0.0, 1.0, 0.0],
        "acceleration": [0.0, 0.0, 0.0],
        "yaw": px4.yaw_enu_to_ned(0.0),
    }
    local = setpoint_to_local(setpoint, offset)
    assert local["position"] == pytest.approx([float(x) for x in px4.enu_to_ned(local_enu)])
    assert local["velocity"] == [0.0, 1.0, 0.0]
    assert local["yaw"] == pytest.approx(math.pi / 2.0)
    # 生成点只写 x,y。不在 PX4_GZ_MODEL_POSE 里再塞一个 yaw，避免和里程计航向叠一次。
    fleet = _fleet()
    rewritten = fleet.apply_scene_spawn_poses(fleet.format_shell(3), SWAP, 3)
    poses = [line.split("=", 1)[1] for line in rewritten.splitlines() if line.startswith("pose=")]
    assert poses
    assert all(pose.count(",") == 1 for pose in poses)


def test_mission_clock_ignores_gazebo_boot_time():
    live = _load_live()
    clock = live.MissionClock(40.0)
    assert clock.elapsed(23.0) == pytest.approx(0.0)
    assert clock.expired(40.0) is False
    assert clock.elapsed(42.78) == pytest.approx(19.78)
    assert clock.expired(63.0) is False
    assert clock.elapsed(63.1) == pytest.approx(40.1)
    assert clock.expired(63.1) is True
    assert clock.origin == pytest.approx(23.0)
    with pytest.raises(ValueError):
        live.MissionClock(0.0)


def test_offboard_stream_repeats_setpoints_but_not_vehicle_commands():
    live = _load_live()
    assert live.SETPOINT_PERIOD_S == pytest.approx(0.05)
    assert live.SETPOINT_PERIOD_S < 1.0

    class _Cmd:
        def __init__(self, heartbeat, offboard, setpoint):
            self.heartbeat = heartbeat
            self.offboard = offboard
            self.setpoint = setpoint
            self.vehicle_commands = ({"command": 21},)

    fresh = {
        "position": [0.0, 1.0, -1.5],
        "velocity": [1.0, 0.0, -0.4],
        "acceleration": [0.2, 0.0, 0.0],
        "yaw": 1.2,
    }
    mode, setpoint = live.offboard_stream(_Cmd(True, {"mode": 1}, fresh))
    assert mode == {"mode": 1}
    assert setpoint["position"] == [0.0, 1.0, -1.5]
    assert setpoint["yaw"] == 1.2
    assert setpoint["velocity"] == [0.0, 0.0, 0.0]
    assert setpoint["acceleration"] == [0.0, 0.0, 0.0]
    assert fresh["velocity"] == [1.0, 0.0, -0.4]
    assert live.offboard_stream(_Cmd(False, {"mode": 1}, {"position": [1.0, 0.0, 0.0]})) == (None, None)
    text = (SITL / "yopo_l0_live.py").read_text()
    assert "offboard_stream(command)" in text
    assert "MissionClock(max_s)" in text
    assert "sitl-battery.env" not in text


def test_feedforward_is_ned_once_and_z_up_is_negative():
    """速度、加速度只做一次 ENU→NED。往东是 NED +y，往上是 NED -z。偏置不改这三项。"""
    from yopo_l0.sitl_bridge import trajectory_setpoint_ned

    setpoint = trajectory_setpoint_ned([8.0, 0.0, 1.5], [1.0, 0.0, 0.4], [0.0, 0.0, -0.2], 0.0, 0.0)
    assert setpoint["position"] == pytest.approx([0.0, 8.0, -1.5])
    assert setpoint["velocity"] == pytest.approx([0.0, 1.0, -0.4])
    assert setpoint["acceleration"] == pytest.approx([0.0, 0.0, 0.2])
    assert setpoint["yaw"] == pytest.approx(math.pi / 2.0)
    local = setpoint_to_local(setpoint, [8.0, 0.0, 0.0])
    assert local["position"] == pytest.approx([0.0, 0.0, -1.5])
    assert local["velocity"] == pytest.approx(setpoint["velocity"])
    assert local["acceleration"] == pytest.approx(setpoint["acceleration"])


def test_local_setpoint_perfect_track_stays_in_the_world_frame():
    """假 PX4 把本地 NED 设定点当成完美跟踪。世界系 = 本地 ENU + 场景卡起点。

    偏置加两次或减两次时，t=0 的悬停点不会落回场景卡起点。
    随后几秒 uav2 会朝目标收敛；uav0 开头往北是规划轨迹本身，不是坐标反号。
    """
    card = load_card(SWAP)
    session = _session(card, "PLAN")
    state = [np.array([0.0, 0.0, 1.5]) for _ in session.slots]
    vel = [np.zeros(3) for _ in session.slots]
    yaw = [0.0 for _ in session.slots]
    dt = 0.2
    errors = []
    for k in range(21):
        t = round(k * dt, 10)
        inbox = {}
        for i, slot in enumerate(session.slots):
            vehicle = FakePx4Vehicle(state[i], vel[i], yaw_enu=yaw[i])
            inbox[slot.ident["namespace"]] = VehicleInbox(
                odometry=odometry_message_from_odometry(vehicle.odometry(), int(round(t * 1e6))),
                armed=True,
                offboard=True,
                landed=False,
            )
        commands = session.step(t, inbox)
        step_err = []
        for i, (slot, command) in enumerate(zip(session.slots, commands)):
            setpoint = command.setpoint
            assert setpoint is not None
            local_enu = np.asarray(px4.ned_to_enu(setpoint["position"]), dtype=float)
            world = local_enu + slot.offset_enu
            if k == 0:
                assert world == pytest.approx(slot.bridge.start_enu)
                assert setpoint["position"][2] == pytest.approx(-1.5)
                assert local_enu[2] == pytest.approx(1.5)
            step_err.append(float(np.linalg.norm(world - slot.bridge.goal)))
            state[i] = local_enu
            vel[i] = np.asarray(px4.ned_to_enu(setpoint["velocity"]), dtype=float)
            if math.isfinite(float(setpoint["yaw"])):
                yaw[i] = float(px4.yaw_ned_to_enu(float(setpoint["yaw"])))
        errors.append(step_err)
    assert errors[0][0] > 10.0
    assert errors[-1][2] < 2.0
    assert sum(errors[-1]) < sum(errors[0])
    # 不是三架一起被推到东边：uav2 的世界系 x 停在目标附近，而不是箱上那次的 23 m。
    uav2 = state[2] + session.slots[2].offset_enu
    assert uav2[0] == pytest.approx(4.0, abs=2.0)
    assert uav2[2] > 0.0
    trace = session.result()["drones"][0]["trace"]
    assert trace
    assert trace[0]["odometry_world_enu"] == pytest.approx([8.0, 0.0, 1.5])
    assert trace[0]["odometry_local_enu"] == pytest.approx([0.0, 0.0, 1.5])
    assert trace[0]["setpoint_local_ned"][2] == pytest.approx(-1.5)
    assert trace[0]["setpoint_world_enu"][2] == pytest.approx(1.5)


def test_stalled_setpoint_holds_at_the_goal_instead_of_passing_it():
    """第四轮的数：设定点已经在目标后面，或者飞机进了 1.5 m，就改成目标点悬停。"""
    from yopo_l0.sitl_bridge import clamp_stalled_setpoint

    goal = np.array([-8.0, 0.0, 1.5])
    start = np.array([8.0, 0.0, 1.5])
    here = np.array([7.945, -0.018, 1.417])
    past = np.array([-8.824, 0.069, 3.130])
    p, v, a, hold = clamp_stalled_setpoint(here, past, np.array([-4.0, 0.0, 0.2]), np.zeros(3), goal, 1.5, start)
    assert hold
    assert p == pytest.approx(goal)
    assert v == pytest.approx(0.0)
    assert a == pytest.approx(0.0)
    short = np.array([0.0, 0.0, 1.5])
    p2, v2, _, hold2 = clamp_stalled_setpoint(here, short, np.array([-1.0, 0.0, 0.0]), np.zeros(3), goal, 1.5, start)
    assert not hold2
    assert p2 == pytest.approx(short)
    assert v2[0] == pytest.approx(-1.0)
    inside = goal + np.array([1.0, 0.0, 0.0])
    p3, v3, _, hold3 = clamp_stalled_setpoint(inside, past, np.ones(3), np.ones(3), goal, 1.5, start)
    assert hold3 and p3 == pytest.approx(goal) and v3 == pytest.approx(0.0)
    # 已经冲过目标 3.7 m，也要拉回目标，而不是继续往前发。
    overshot = np.array([-11.735, 0.104, 2.052])
    p4, _, _, hold4 = clamp_stalled_setpoint(overshot, overshot + np.array([-1.0, 0.0, 0.0]), np.zeros(3), np.zeros(3), goal, 1.5, start)
    assert hold4 and p4 == pytest.approx(goal)


def test_live_arrival_limits_match_the_scene_card():
    from yopo_l0.sitl_bridge import _goal_limits

    card = load_card(SWAP)
    tol, speed, hold, body = _goal_limits(card)
    assert (tol, speed, hold, body) == (0.4, 0.2, 1.0, 0.35)
    session = _session(card, "PLAN")
    assert session.goal_tol == tol and session.goal_speed == speed and session.goal_hold == hold
    assert session.slots[0].bridge._arrive_radius() == pytest.approx(1.5)


def test_yaw_catches_up_toward_the_goal_when_the_loop_stalls():
    """正常 0.02 s 节拍偏航几乎不动。间隔 1.5 s 时期望航向转到目标，下一拍的速度朝西而不是朝东。"""
    from yopo_l0.sitl_bridge import SitlBridgeConfig, YopoSitlBridge

    card = load_card(SWAP)
    cfg = SitlBridgeConfig(
        drone_id="uav0", seed=0, drone_index=0, n_drones=3,
        goal_enu=(-8.0, 0.0, 1.5), hover_alt_m=1.5, planner_dt=1.0,
    )

    def run(times):
        bridge = YopoSitlBridge(card, cfg)
        vehicle = FakePx4Vehicle(np.array([8.0, 0.0, 1.5]), np.zeros(3), yaw_enu=0.0)
        odom = vehicle.odometry()
        last = None
        for t in times:
            last = bridge.command(
                odom["position_ned"], odom["velocity_ned"], t,
                q_ned_frd=odom["q_ned_frd"], acceleration_ned=odom["acceleration_ned"],
            )
        return last

    fine = run([0.0, 0.02])
    assert abs(fine.yaw_enu) < 0.2
    # 1.5 s 处补航向并重规划，1.9 s 只采样、不再重规划，速度应朝西（目标在 -x）。
    stalled = run([0.0, 1.5, 1.9])
    assert abs(px4.wrap_pi(stalled.yaw_enu)) > 1.0
    assert stalled.v_enu[0] < 0.0
    # 1.2 s 的间隔不能把设定点发到目标西边。
    jumped = run([0.0, 1.2])
    assert jumped.p_enu[0] >= -8.0 - 1e-6
    assert float(np.linalg.norm(jumped.p_enu - np.array([-8.0, 0.0, 1.5]))) <= float(
        np.linalg.norm(np.array([8.0, 0.0, 1.5]) - np.array([-8.0, 0.0, 1.5]))
    )


def test_yopo_poses_cli_rewrites_a_layout_file(tmp_path):
    fleet = _fleet()
    layout = tmp_path / "layout.txt"
    layout.write_text(fleet.format_shell(3))
    proc = subprocess.run(
        [
            sys.executable,
            str(SITL / "fleet.py"),
            "yopo-poses",
            "--scene",
            str(SWAP),
            "--n",
            "3",
            "--layout",
            str(layout),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
    poses = [line for line in proc.stdout.splitlines() if line.startswith("pose=")]
    assert poses[0] == "pose=8,0"
    assert float(poses[1].split(",")[1]) == pytest.approx(6.928203230275509, abs=1e-8)
    bad = subprocess.run(
        [
            sys.executable,
            str(SITL / "fleet.py"),
            "yopo-poses",
            "--scene",
            str(SWAP),
            "--n",
            "2",
            "--layout",
            str(layout),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert bad.returncode == 2
    assert "2" in bad.stderr
