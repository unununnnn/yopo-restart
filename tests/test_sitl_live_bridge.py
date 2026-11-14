"""票 S4b：ROS 消息字段上的 YOPO L0 ↔ PX4 闭环。不需要 docker / rclpy。"""

import ast
import importlib.util
import json
import math
import subprocess
import sys

import numpy as np
import pytest

from yopo_swarm_core import px4

from conftest import ROOT
from yopo_l0.scene import load_card
from yopo_l0.sensor import CameraConfig
from yopo_l0.sitl_bridge import (
    SCHEMA,
    SCHEMA_LIVE,
    VEHICLE_CMD_NAV_LAND,
    FakePx4Vehicle,
    LiveFleetConfig,
    VehicleInbox,
    YopoSitlBridge,
    SitlBridgeConfig,
    decode_depth_image,
    decode_vehicle_odometry,
    depth_from_obstacle_summary,
    encode_depth_image,
    live_topic_plan,
    make_live_session,
    normalize_positioning,
    odometry_message_from_odometry,
    command_from_ros,
    roundtrip_abs,
    run_closed_loop,
    run_mock_ros_closed_loop,
    vehicle_identity,
    vehicle_topics,
)

SITL = ROOT / "scripts/sitl"


def _load(name):
    spec = importlib.util.spec_from_file_location(f"sitl_s4b_{name}", SITL / name)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _odom_message(position_enu, velocity_enu=None, yaw_enu=0.0, timestamp_us=0):
    vehicle = FakePx4Vehicle(position_enu, velocity_enu, yaw_enu=yaw_enu)
    return odometry_message_from_odometry(vehicle.odometry(), timestamp_us), vehicle


def test_live_sources_do_not_import_rclpy_at_module_level():
    bridge = (ROOT / "yopo_l0/sitl_bridge.py").read_text()
    offline = (SITL / "run_yopo_l0.py").read_text()
    live = (SITL / "yopo_l0_live.py").read_text()
    for text in (bridge, offline):
        assert "import rclpy" not in text
        assert "from rclpy" not in text
    tree = ast.parse(live)
    for node in tree.body:
        if isinstance(node, ast.Import):
            assert all(alias.name != "rclpy" for alias in node.names)
        if isinstance(node, ast.ImportFrom):
            assert node.module != "rclpy"
    assert "def _run_ros" in live


def test_vehicle_topics_isolate_namespaces_and_roundtrip_ned():
    single = vehicle_topics("")
    assert single["vehicle_odometry"] == "/fmu/out/vehicle_odometry"
    assert single["trajectory_setpoint"] == "/fmu/in/trajectory_setpoint"
    assert single["depth_image"] == "/yopo/depth_image"
    first = vehicle_identity(0, 3)
    second = vehicle_identity(1, 3)
    assert first["namespace"] == "px4_1" and first["mav_sys_id"] == 2 and first["gcs_port"] == 18571
    assert second["namespace"] == "px4_2" and second["mav_sys_id"] == 3
    topics_a = vehicle_topics(first["namespace"])
    topics_b = vehicle_topics(second["namespace"])
    assert topics_a["vehicle_odometry"] == "/px4_1/fmu/out/vehicle_odometry"
    assert topics_b["trajectory_setpoint"] == "/px4_2/fmu/in/trajectory_setpoint"
    assert topics_a["obstacle_summary"] != topics_b["obstacle_summary"]
    assert topics_a["vehicle_status"].endswith("vehicle_status_v1")

    msg, _ = _odom_message([1.25, -0.5, 1.5], [0.2, -0.1, 0.0], yaw_enu=0.3, timestamp_us=20000)
    decoded = decode_vehicle_odometry(msg, now_s=0.02)
    assert np.allclose(decoded["position_enu"], [1.25, -0.5, 1.5], atol=1e-12)
    assert np.allclose(decoded["velocity_enu"], [0.2, -0.1, 0.0], atol=1e-12)
    assert abs(px4.wrap_pi(decoded["yaw_enu"] - 0.3)) < 1e-12
    assert roundtrip_abs(decoded["position_enu"]) < 1e-12
    assert roundtrip_abs(decoded["velocity_enu"]) < 1e-12
    back = px4.ned_to_enu(decoded["position_ned"])
    assert np.max(np.abs(back - decoded["position_enu"])) < 1e-12
    bad = dict(msg)
    bad["pose_frame"] = 2
    with pytest.raises(ValueError):
        decode_vehicle_odometry(bad, now_s=0.02)


def test_depth_image_and_obstacle_summary_become_depth_frames():
    from yopo_l0.sensor import DepthFrame

    depth = np.array([[1.5, np.nan], [0.0, 4.0]], dtype=np.float32)
    frame = DepthFrame(0.2, depth, (10.0, 11.0, 1.0, 2.0))
    encoded = encode_depth_image(frame)
    decoded = decode_depth_image(encoded)
    assert decoded.depth.shape == (2, 2)
    assert decoded.depth[0, 0] == pytest.approx(1.5)
    assert math.isnan(float(decoded.depth[0, 1]))
    assert decoded.K == pytest.approx((10.0, 11.0, 1.0, 2.0))
    mm = np.array([[5000, 0], [1000, 2500]], dtype="<u2")
    range_frame = decode_depth_image(
        {
            "height": 2,
            "width": 2,
            "encoding": "16UC1",
            "is_bigendian": 0,
            "data": mm.tobytes(),
            "K": (1.0, 1.0, 1.0, 1.0),
            "stamp_s": 0.0,
        }
    )
    assert range_frame.depth[0, 0] == pytest.approx(5.0)
    assert math.isnan(float(range_frame.depth[0, 1]))
    assert range_frame.depth[1, 0] == pytest.approx(1.0)

    camera = CameraConfig(width=32, height=24, hfov_deg=90.0, vfov_deg=60.0, min_range_m=0.2, max_range_m=20.0)
    summary = depth_from_obstacle_summary(
        {"points_enu": [[5.0, 0.0, 1.5]]},
        np.array([0.0, 0.0, 1.5]),
        0.0,
        camera,
        0.0,
    )
    finite = np.isfinite(summary.depth)
    assert int(finite.sum()) >= 1
    rows, cols = np.nonzero(finite)
    assert np.min(np.abs(cols - camera.width / 2.0)) < camera.width / 4.0
    assert float(np.nanmin(summary.depth)) == pytest.approx(5.0, abs=0.5)
    fan = depth_from_obstacle_summary(
        json.dumps({"ranges_m": [4.0], "bearings_rad": [0.0]}),
        np.array([0.0, 0.0, 1.5]),
        0.0,
        camera,
        1.0,
    )
    assert np.isfinite(fan.depth).any()


def test_one_ros_step_matches_direct_bridge_and_setpoint_roundtrip():
    card = load_card(ROOT / "scenes/g2-pillar.json")
    cfg = SitlBridgeConfig(drone_id="uav0", seed=0, goal_enu=(5.0, 0.0, 1.5), hover_alt_m=1.5)
    direct = YopoSitlBridge(card, cfg)
    via_ros = YopoSitlBridge(card, cfg)
    msg, vehicle = _odom_message(direct.start_enu, yaw_enu=0.0, timestamp_us=0)
    preview = via_ros.camera.capture(direct.start_enu, 0.0, 0.0)
    out_ros, source = command_from_ros(via_ros, msg, 0.0, depth=encode_depth_image(preview), depth_source="depth")
    odom = vehicle.odometry()
    out_direct = direct.command(
        odom["position_ned"], odom["velocity_ned"], 0.0, q_ned_frd=odom["q_ned_frd"], yaw_ned=odom["yaw_ned"],
        acceleration_ned=odom["acceleration_ned"],
    )
    assert source == "ros-depth"
    assert np.allclose(out_ros.p_enu, out_direct.p_enu, atol=1e-12)
    assert np.allclose(out_ros.v_enu, out_direct.v_enu, atol=1e-12)
    assert np.allclose(out_ros.a_enu, out_direct.a_enu, atol=1e-12)
    assert abs(px4.wrap_pi(out_ros.yaw_enu - out_direct.yaw_enu)) < 1e-12
    assert out_ros.roundtrip_error_m < 1e-12
    assert np.max(np.abs(px4.ned_to_enu(out_ros.setpoint["position"]) - out_ros.p_enu)) < 1e-12
    assert via_ros.last_observation is not None
    assert np.allclose(via_ros.last_observation.state.p, direct.start_enu, atol=1e-12)
    assert via_ros.last_observation.frame.depth.shape == preview.depth.shape


def test_namespaces_do_not_share_odometry_or_commands():
    card = load_card(ROOT / "scenes/swap-3-open.json")
    session = make_live_session(
        card, seed=0, live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card", positioning="default")
    )
    assert [slot.ident["namespace"] for slot in session.slots] == ["px4_1", "px4_2", "px4_3"]
    inbox = {}
    expected = {}
    for slot in session.slots:
        start = slot.bridge.start_enu.copy()
        if slot.ident["namespace"] == "px4_1":
            start = start + np.array([3.0, 0.0, 0.0])
        expected[slot.ident["namespace"]] = start
        msg, _ = _odom_message(start, timestamp_us=0)
        inbox[slot.ident["namespace"]] = VehicleInbox(odometry=msg, armed=True, offboard=True, landed=False)
    inbox["px4_9"] = inbox["px4_1"]
    commands = session.step(0.0, inbox)
    by_ns = {command.namespace: command for command in commands}
    assert set(by_ns) == {"px4_1", "px4_2", "px4_3"}
    for ns, command in by_ns.items():
        assert command.topics["vehicle_odometry"].startswith(f"/{ns}/")
        assert command.topics["trajectory_setpoint"].startswith(f"/{ns}/")
        assert np.allclose(command.position_enu, expected[ns], atol=1e-9)
        assert command.mav_sys_id == {"px4_1": 2, "px4_2": 3, "px4_3": 4}[ns]
    assert not np.allclose(by_ns["px4_1"].position_enu, by_ns["px4_2"].position_enu, atol=1e-6)
    assert session.px4_parameters_written == []


def test_rtk_switch_is_recorded_without_parameter_writes():
    with pytest.raises(ValueError):
        normalize_positioning("gps")
    card = load_card(ROOT / "scenes/g2-pillar.json")
    plan = live_topic_plan(card, positioning="rtk", seed=0)
    assert plan["positioning"] == "rtk"
    assert plan["px4_parameters_written"] == []
    assert plan["namespaces"] == [""]
    assert plan["mav_sys_ids"] == [1]
    assert plan["dry_run"] is True
    assert plan["fake_px4"] is False
    session = make_live_session(card, live=LiveFleetConfig(positioning="rtk", initial_phase="PLAN"))
    assert session.cfg.positioning == "rtk"
    assert session.px4_parameters_written == []
    assert session.result()["positioning"] == "rtk"


def test_goal_hold_emits_land_and_finishes_disarmed():
    card = load_card(ROOT / "scenes/g2-pillar.json")
    session = make_live_session(
        card,
        seed=0,
        live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card", positioning="rtk", plan_timeout_s=30.0),
    )
    goal = session.slots[0].bridge.goal
    landed_cmds = []
    done = False
    inbox = None
    t = 0.0
    for _ in range(80):
        msg, _ = _odom_message(goal, np.zeros(3), timestamp_us=int(round(t * 1e6)))
        inbox = VehicleInbox(odometry=msg, armed=True, offboard=True, landed=False)
        if landed_cmds:
            inbox.landed = True
            inbox.armed = False
        commands = session.step(t, {"": inbox})
        for command in commands:
            for fields in command.vehicle_commands:
                if int(fields["command"]) == VEHICLE_CMD_NAV_LAND:
                    landed_cmds.append(fields)
                    assert fields["target_system"] == 1
        if commands and all(command.phase == "DONE" for command in commands):
            done = True
            break
        t = round(t + 0.02, 10)
    assert landed_cmds, "到达保持后应发出 NAV_LAND"
    assert done
    result = session.result()
    assert result["ok"] is True
    assert result["landed"] is True
    assert result["outcome"] == "SUCCESS"
    assert result["arrival_rate"] == 1.0
    assert result["positioning"] == "rtk"
    assert result["px4_parameters_written"] == []
    assert result["enu_ned_roundtrip_max_error_m"] < 1e-12
    assert result["land_commands_sent"] >= 1


def test_arm_commands_use_each_vehicle_sys_id():
    card = load_card(ROOT / "scenes/swap-3-open.json")
    session = make_live_session(
        card,
        seed=0,
        live=LiveFleetConfig(initial_phase="WARMUP", warmup_s=0.02, positioning="default"),
    )
    inbox = {}
    for slot in session.slots:
        msg, _ = _odom_message(np.array([slot.bridge.start_enu[0], slot.bridge.start_enu[1], 0.0]), timestamp_us=0)
        inbox[slot.ident["namespace"]] = VehicleInbox(odometry=msg, armed=False, offboard=False, landed=True)
    seen = {}
    t = 0.0
    index = {slot.ident["namespace"]: slot for slot in session.slots}
    for _ in range(8):
        for ns, box in inbox.items():
            ground = index[ns].bridge.start_enu * np.array([1.0, 1.0, 0.0])
            msg, _ = _odom_message(ground, timestamp_us=int(round(t * 1e6)))
            box.odometry = msg
        commands = session.step(t, inbox)
        for command in commands:
            assert command.heartbeat
            for fields in command.vehicle_commands:
                seen.setdefault(command.namespace, []).append(int(fields["command"]))
                assert fields["target_system"] == command.mav_sys_id
                assert command.topics["vehicle_command"] == f"/{command.namespace}/fmu/in/vehicle_command"
        t = round(t + 0.02, 10)
    assert set(seen) == {"px4_1", "px4_2", "px4_3"}
    for codes in seen.values():
        assert 176 in codes and 400 in codes
    assert session.px4_parameters_written == []


def test_g2_mock_ros_matches_fake_px4_then_lands():
    card = load_card(ROOT / "scenes/g2-pillar.json")
    direct = run_closed_loop(card, seed=0, max_s=15.0, partial=False)
    mocked = run_mock_ros_closed_loop(card, seed=0, max_s=15.0, partial=False, positioning="default")
    assert direct["schema"] == SCHEMA
    assert mocked["schema"] == SCHEMA_LIVE
    assert mocked["adapter"] == "mock-ros" and mocked["fake_px4"] is True
    assert mocked["outcome"] == direct["outcome"] == "SUCCESS"
    assert mocked["arrival_time_s"] == pytest.approx(direct["arrival_time_s"])
    assert mocked["arrival_rate"] == 1.0
    assert mocked["collisions"] == 0
    assert mocked["min_surface_clearance_m"] == pytest.approx(direct["min_surface_clearance_m"])
    assert mocked["enu_ned_roundtrip_max_error_m"] < 1e-12
    assert mocked["enu_ned_roundtrip_max_velocity_mps"] < 1e-12
    assert mocked["namespaces"] == [""]
    assert mocked["topics"][""]["vehicle_odometry"] == "/fmu/out/vehicle_odometry"
    assert mocked["positioning"] == "default"
    assert mocked["px4_parameters_written"] == []
    assert mocked["ok"] is True and mocked["landed"] is True and mocked["finish"] == "NAV_LAND"
    assert len(mocked["land_commands"]) == 1
    assert mocked["land_commands"][0]["command"] == VEHICLE_CMD_NAV_LAND
    assert mocked["land_commands"][0]["target_system"] == 1
    assert "positioning" not in direct


def test_swap3_mock_ros_partial_keeps_namespaces_and_rtk_flag():
    card = load_card(ROOT / "scenes/swap-3-open.json")
    result = run_mock_ros_closed_loop(
        card, seed=0, max_s=15.0, partial=True, partial_s=0.3, positioning="rtk"
    )
    assert result["n_drones"] == 3
    assert result["namespaces"] == ["px4_1", "px4_2", "px4_3"]
    assert result["mav_sys_ids"] == [2, 3, 4]
    assert result["gcs_ports"] == [18571, 18572, 18573]
    assert result["topics"]["px4_1"]["vehicle_odometry"] == "/px4_1/fmu/out/vehicle_odometry"
    assert result["topics"]["px4_3"]["trajectory_setpoint"] == "/px4_3/fmu/in/trajectory_setpoint"
    assert result["topics"]["px4_2"]["depth_image"] == "/px4_2/yopo/depth_image"
    assert len({result["topics"][ns]["obstacle_summary"] for ns in result["namespaces"]}) == 3
    assert result["positioning"] == "rtk"
    assert result["px4_parameters_written"] == []
    assert result["collisions"] == 0
    assert result["neighbor_sample_deliveries"] > 0
    assert result["land_commands"] == []
    assert result["landed"] is False
    assert result["outcome"] in ("PARTIAL", "SUCCESS")
    assert result["enu_ned_roundtrip_max_error_m"] < 1e-12
    direct = run_closed_loop(card, seed=0, max_s=15.0, partial=True, partial_s=0.3)
    assert result["min_inter_drone_distance_m"] == pytest.approx(direct["min_inter_drone_distance_m"])


def test_fake_px4_cli_stays_on_s4_schema(tmp_path, capsys):
    mod = _load("run_yopo_l0.py")
    out = tmp_path / "fake.json"
    rc = mod.main(
        ["--fake-px4", "--scene", "scenes/g2-pillar.json", "--partial", "--positioning", "rtk", "--out", str(out)]
    )
    assert rc == 0
    data = json.loads(out.read_text())
    assert json.loads(capsys.readouterr().out) == data
    assert data["schema"] == SCHEMA
    assert data["adapter"] == "fake-px4"
    assert "positioning" not in data
    assert "land_commands" not in data


def test_mock_ros_cli_records_positioning(tmp_path, capsys):
    mod = _load("run_yopo_l0.py")
    out = tmp_path / "mock.json"
    rc = mod.main(
        [
            "--mock-ros",
            "--positioning",
            "rtk",
            "--scene",
            "scenes/g2-pillar.json",
            "--partial",
            "--out",
            str(out),
        ]
    )
    assert rc == 0
    data = json.loads(out.read_text())
    assert json.loads(capsys.readouterr().out)["schema"] == data["schema"] == SCHEMA_LIVE
    assert data["positioning"] == "rtk"
    assert data["px4_parameters_written"] == []
    assert data["fake_px4"] is True
    assert data["namespaces"] == [""]


def test_live_script_dry_run_lists_swap_namespaces(tmp_path, capsys):
    mod = _load("yopo_l0_live.py")
    out = tmp_path / "dry.json"
    rc = mod.main(
        [
            "--dry-run",
            "--scene",
            "scenes/swap-3-open.json",
            "--n",
            "3",
            "--positioning",
            "rtk",
            "--seed",
            "0",
            "--out",
            str(out),
        ]
    )
    assert rc == 0
    data = json.loads(out.read_text())
    printed = json.loads(capsys.readouterr().out)
    assert printed == data
    assert data["dry_run"] is True
    assert data["positioning"] == "rtk"
    assert data["px4_parameters_written"] == []
    assert data["namespaces"] == ["px4_1", "px4_2", "px4_3"]
    assert data["topics"]["px4_1"]["vehicle_odometry"] == "/px4_1/fmu/out/vehicle_odometry"
    assert data["drone_ids"] == ["uav0", "uav1", "uav2"]
    bad = mod.main(["--dry-run", "--scene", "scenes/g2-pillar.json", "--n", "3", "--out", str(tmp_path / "bad.json")])
    assert bad == 2


def test_shell_entry_is_opt_in_and_leaves_offboard_default():
    for name in ("run_yopo_l0_sitl.sh", "in_container.sh", "exec_mission.sh"):
        subprocess.run(["bash", "-n", str(SITL / name)], check=True)
    dry = subprocess.run(
        ["bash", str(SITL / "run_yopo_l0_sitl.sh"), "--dry-run", "--n", "3", "--positioning", "rtk"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert dry.returncode == 0, dry.stderr
    text = dry.stdout
    assert "planner=yopo-l0" in text
    assert "scene=scenes/swap-3-open.json" in text
    assert "n=3" in text
    assert "positioning=rtk" in text
    one = subprocess.run(
        ["bash", str(SITL / "run_yopo_l0_sitl.sh"), "--dry-run", "--n", "1"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert one.returncode == 0
    assert "scene=scenes/g2-pillar.json" in one.stdout
    assert "positioning=default" in one.stdout
    bad = subprocess.run(
        ["bash", str(SITL / "run_yopo_l0_sitl.sh"), "--dry-run", "--n", "4"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert bad.returncode == 2
    container = (SITL / "in_container.sh").read_text()
    mission = (SITL / "exec_mission.sh").read_text()
    entry = (SITL / "run_yopo_l0_sitl.sh").read_text()
    assert 'YOPO_SITL_PLANNER:-offboard' in container
    assert 'offboard_mission.py --out "$OUT/result.json" --wall-timeout' in container
    assert 'env -u YOPO_SITL_MISSION_MODE' in container
    assert 'PLANNER" == "yopo-l0"' in container
    entry_code = "\n".join(line for line in entry.splitlines() if not line.strip().startswith("#"))
    assert "offboard_mission.py" not in entry_code
    assert "yopo_l0_live.py" in mission
    assert 'fleet.py run --out /out/result.json --n "${YOPO_SITL_N:?}"' in mission
    for path in (SITL / "run_yopo_l0_sitl.sh", SITL / "yopo_l0_live.py", ROOT / "yopo_l0/sitl_bridge.py"):
        code = "\n".join(line.strip() for line in path.read_text().splitlines() if not line.strip().startswith("#"))
        assert "PX4_PARAM_" not in code
        assert "param set" not in code
