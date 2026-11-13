"""票 S4：YOPO L0 ↔ 假 PX4 的 ENU/NED 闭环。不需要 docker / rclpy。"""

import importlib.util
import json
import math
import sys

import numpy as np
import pytest

from yopo_swarm_core import px4

from conftest import ROOT
from yopo_l0.dynamics import HoverPlan, PointMassLag, State
from yopo_l0.policy import PolicyContext, UpstreamPolicy
from yopo_l0.scene import load_card
from yopo_l0.sitl_bridge import (
    SCHEMA,
    FakePx4Vehicle,
    NeighborEnu,
    SitlBridgeConfig,
    YopoSitlBridge,
    enu_state_from_ned,
    roundtrip_abs,
    offboard_command,
    run_closed_loop,
    trajectory_setpoint_ned,
)

SITL = ROOT / "scripts/sitl"


def _load_cli():
    spec = importlib.util.spec_from_file_location("sitl_s4_run_yopo_l0", SITL / "run_yopo_l0.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_module_does_not_import_rclpy():
    bridge = (ROOT / "yopo_l0/sitl_bridge.py").read_text()
    script = (SITL / "run_yopo_l0.py").read_text()
    for text in (bridge, script):
        assert "import rclpy" not in text
        assert "from rclpy" not in text
    assert "from yopo_swarm_core import px4" in bridge


def test_enu_ned_roundtrip_error_below_1e_12():
    rng = np.random.default_rng(0)
    for _ in range(32):
        vec = rng.normal(size=3) * 20.0
        assert roundtrip_abs(vec) < 1e-12
        assert np.max(np.abs(px4.ned_to_enu(px4.enu_to_ned(vec)) - vec)) < 1e-12
    sp = trajectory_setpoint_ned([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], 0.0, 0.0)
    assert sp["position"] == pytest.approx(px4.enu_to_ned([1.0, 0.0, 0.0]))
    assert sp["velocity"] == pytest.approx(px4.enu_to_ned([0.0, 1.0, 0.0]))
    assert sp["acceleration"] == pytest.approx(px4.enu_to_ned([0.0, 0.0, 1.0]))
    assert sp["yaw"] == pytest.approx(px4.yaw_enu_to_ned(0.0))
    back = px4.ned_to_enu(sp["position"])
    assert np.max(np.abs(back - np.array([1.0, 0.0, 0.0]))) < 1e-12


def test_fake_px4_matches_point_mass_lag_after_ned_roundtrip():
    start = np.array([1.0, -2.0, 1.5])
    goal = np.array([4.0, 0.5, 1.5])
    yaw0 = 0.3
    veh = FakePx4Vehicle(start, yaw_enu=yaw0)
    st = State(0.0, start.copy(), np.zeros(3), np.zeros(3), yaw0)
    lag = PointMassLag()
    plan = HoverPlan(tuple(float(x) for x in goal), yaw0)
    dt = 0.02
    for k in range(40):
        t = k * dt
        p, v, a, yaw = plan.sample(t)
        veh.step(trajectory_setpoint_ned(p, v, a, yaw, t), dt)
        st = lag.step(st, plan, dt)
        odom = veh.odometry()
        p_enu, v_enu, a_enu, yaw_enu = enu_state_from_ned(
            odom["position_ned"],
            odom["velocity_ned"],
            t + dt,
            q_ned_frd=odom["q_ned_frd"],
            acceleration_ned=odom["acceleration_ned"],
        )
        assert np.allclose(p_enu, st.p, atol=1e-9)
        assert np.allclose(v_enu, st.v, atol=1e-9)
        assert np.allclose(a_enu, st.a, atol=1e-9)
        assert abs(px4.wrap_pi(yaw_enu - st.yaw)) < 1e-9
        assert roundtrip_abs(p_enu) < 1e-12


def test_quaternion_and_yaw_ned_are_the_same_enu_state():
    veh = FakePx4Vehicle([0.0, 0.0, 1.5], velocity_enu=[0.4, -0.2, 0.0], yaw_enu=0.0)
    odom = veh.odometry()
    via_q = enu_state_from_ned(
        odom["position_ned"], odom["velocity_ned"], 0.0, q_ned_frd=odom["q_ned_frd"]
    )
    via_yaw = enu_state_from_ned(
        odom["position_ned"], odom["velocity_ned"], 0.0, yaw_ned=odom["yaw_ned"]
    )
    assert np.allclose(via_q[0], [0.0, 0.0, 1.5], atol=1e-12)
    assert np.allclose(via_q[0], via_yaw[0], atol=1e-12)
    assert np.allclose(via_q[1], via_yaw[1], atol=1e-12)
    assert abs(px4.wrap_pi(via_q[3] - via_yaw[3])) < 1e-12
    assert abs(px4.wrap_pi(via_q[3] - 0.0)) < 1e-12


def test_one_step_matches_upstream_policy_and_px4_setpoint():
    card = load_card(ROOT / "scenes/g2-pillar.json")
    cfg = SitlBridgeConfig(
        drone_id="uav0",
        seed=0,
        goal_enu=(5.0, 0.0, 1.5),
        hover_alt_m=1.5,
        policy_params={},
    )
    bridge = YopoSitlBridge(card, cfg)
    veh = FakePx4Vehicle(bridge.start_enu, yaw_enu=0.0)
    odom = veh.odometry()
    out = bridge.command(
        odom["position_ned"],
        odom["velocity_ned"],
        0.0,
        q_ned_frd=odom["q_ned_frd"],
        yaw_ned=odom["yaw_ned"],
    )
    fresh = UpstreamPolicy(
        PolicyContext("uav0", cfg.physics_dt, {}, obstacles=card.obstacles, a_max=cfg.a_max)
    )
    plan = fresh.plan(bridge.last_observation)
    ep, ev, ea, ey = plan.sample(0.0)
    assert np.allclose(out.p_enu, ep, atol=1e-12)
    assert np.allclose(out.v_enu, ev, atol=1e-12)
    assert np.allclose(out.a_enu, ea, atol=1e-12)
    assert abs(px4.wrap_pi(out.yaw_enu - float(ey))) < 1e-12
    expect = px4.trajectory_setpoint(np.stack([np.asarray(ep, float), np.asarray(ev, float), np.asarray(ea, float)]), float(ey), 0)
    assert out.setpoint["position"] == pytest.approx(expect["position"])
    assert out.setpoint["velocity"] == pytest.approx(expect["velocity"])
    assert out.setpoint["acceleration"] == pytest.approx(expect["acceleration"])
    assert out.setpoint["yaw"] == pytest.approx(expect["yaw"])
    assert out.offboard["position"] and out.offboard["velocity"] and out.offboard["acceleration"]
    assert out.roundtrip_error_m < 1e-12
    assert np.max(np.abs(px4.ned_to_enu(out.setpoint["position"]) - out.p_enu)) < 1e-12
    cmd = offboard_command(out, "px4_1")
    assert cmd.namespace == "px4_1"
    assert cmd.trajectory_setpoint is out.setpoint
    assert cmd.offboard_control_mode is out.offboard
    assert out.planned is True
    again = bridge.command(
        odom["position_ned"], odom["velocity_ned"], 0.0, q_ned_frd=odom["q_ned_frd"]
    )
    assert again is out


def test_neighbor_samples_reach_upstream_when_filter_is_on():
    card = load_card(ROOT / "scenes/swap-3-open.json")
    params = {
        "upstream": {
            "inter_drone_filter": True,
            "inter_drone_threshold_m": 0.7,
            "inter_drone_penalty": 0.0,
        }
    }
    cfg = SitlBridgeConfig(
        drone_id="uav0",
        seed=0,
        drone_index=0,
        n_drones=3,
        goal_enu=tuple(card.card["drones"][0]["goal_enu"]),
        policy_params=params,
    )
    bridge = YopoSitlBridge(card, cfg)
    samples = [[0.0, -8.0, 0.2, 1.5], [0.02, -7.9, 0.2, 1.5], [0.04, -7.8, 0.2, 1.5]]
    bridge.set_neighbors(
        (NeighborEnu("uav1", np.array([-4.0, 6.9, 1.5]), np.zeros(3), samples),)
    )
    veh = FakePx4Vehicle(bridge.start_enu, yaw_enu=0.0)
    odom = veh.odometry()
    out = bridge.command(
        odom["position_ned"], odom["velocity_ned"], 0.0, yaw_ned=odom["yaw_ned"]
    )
    assert out.planned
    assert bridge.neighbor_sample_deliveries == 1
    assert "uav1" in bridge.policy._neighbors
    assert bridge.neighbors[0].drone_id == "uav1"


def test_g2_pillar_seed0_arrives_without_collision():
    card = load_card(ROOT / "scenes/g2-pillar.json")
    result = run_closed_loop(card, seed=0, max_s=15.0, partial=False)
    assert result["schema"] == SCHEMA
    assert result["report_only"] is True
    assert result["criteria_frozen"] is False
    assert result["outcome"] == "SUCCESS"
    assert result["arrival_rate"] == 1.0
    assert result["collisions"] == 0
    assert result["min_surface_clearance_m"] > 0.0
    assert result["enu_ned_roundtrip_max_error_m"] < 1e-12
    assert result["enu_ned_roundtrip_max_velocity_mps"] < 1e-12
    assert result["enu_ned_roundtrip_max_acceleration_mps2"] < 1e-12
    assert result["setpoint_hz"] == 50.0
    assert result["planner_hz"] == 10.0
    assert result["drones"][0]["arrived"] is True


def test_swap3_partial_exchanges_neighbors_without_collision():
    card = load_card(ROOT / "scenes/swap-3-open.json")
    result = run_closed_loop(card, seed=0, max_s=15.0, partial=True, partial_s=0.3)
    assert result["n_drones"] == 3
    assert result["partial"] is True
    assert result["collisions"] == 0
    assert result["neighbor_sample_deliveries"] > 0
    assert result["min_inter_drone_distance_m"] > 0.7
    assert result["enu_ned_roundtrip_max_error_m"] < 1e-12
    assert result["outcome"] in ("PARTIAL", "SUCCESS")
    assert 0.0 <= result["arrival_rate"] <= 1.0


def test_partial_cli_writes_valid_json(tmp_path, capsys):
    mod = _load_cli()
    out = tmp_path / "result.json"
    rc = mod.main(
        [
            "--scene",
            "scenes/g2-pillar.json",
            "--seed",
            "0",
            "--max-s",
            "15",
            "--partial",
            "--out",
            str(out),
        ]
    )
    assert rc == 0
    data = json.loads(out.read_text())
    printed = json.loads(capsys.readouterr().out)
    assert printed == data
    assert data["schema"] == SCHEMA
    assert data["report_only"] is True
    assert data["criteria_frozen"] is False
    assert data["scene"] == "g2-pillar"
    assert data["seed"] == 0
    assert data["partial"] is True
    assert data["max_s"] == 15.0
    assert data["collisions"] == 0
    assert data["enu_ned_roundtrip_max_error_m"] < 1e-12
    assert isinstance(data["arrival_rate"], float)
    assert math.isfinite(data["min_surface_clearance_m"])
    assert data["horizon_s"] == pytest.approx(0.5)
    assert data["sim_duration_s"] == pytest.approx(0.5)
