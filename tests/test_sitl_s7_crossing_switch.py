"""票 S7：实飞会话接 crossing / switch，飞行段净空口径和机间时钟偏差。假 PX4，不起 Gazebo。"""

import json
import subprocess

import numpy as np
import pytest

from conftest import ROOT
from yopo_l0.scene import load_card
from yopo_l0.sitl_bridge import (
    ClockSkewMeter,
    FakePx4Vehicle,
    LiveFleetConfig,
    VehicleInbox,
    live_topic_plan,
    make_live_session,
    odometry_message_from_odometry,
    resolve_live_policy,
)

DT = 0.02
CROSS = ROOT / "scenes/crossing-pillars.json"
CROSS_SITL = ROOT / "scenes/crossing-pillars-sitl80.json"
SWITCH = ROOT / "scenes/formation-switch.json"
SWITCH_SITL = ROOT / "scenes/formation-switch-sitl80.json"
G2 = ROOT / "scenes/g2-pillar.json"


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


@pytest.mark.parametrize("base,sitl", [(CROSS, CROSS_SITL), (SWITCH, SWITCH_SITL)])
def test_sitl_cards_only_extend_duration_and_stay_off_suite(base, sitl):
    a = json.loads(base.read_text())
    b = json.loads(sitl.read_text())
    assert b["name"] == a["name"] + "-sitl80"
    assert b["task"]["duration_s"] == 80.0
    note = b.pop("note")
    assert "duration_s" in note and "suite.txt" in note
    a["name"] = b["name"]
    a["task"]["duration_s"] = 80.0
    assert a == b
    suite = (ROOT / "scenes/suite.txt").read_text()
    assert b["name"] not in suite


def test_policy_must_match_the_card_task():
    assert resolve_live_policy(load_card(CROSS), "crossing") == "crossing"
    assert resolve_live_policy(load_card(SWITCH), "switch") == "switch"
    with pytest.raises(ValueError, match="policy=crossing"):
        resolve_live_policy(load_card(G2), "crossing")
    with pytest.raises(ValueError, match="switch 场景卡需要 policy=switch"):
        resolve_live_policy(load_card(SWITCH), "upstream")
    with pytest.raises(ValueError, match="crossing 场景卡需要 policy=crossing"):
        resolve_live_policy(load_card(CROSS), "upstream")
    with pytest.raises(ValueError, match="policy=formation 只适用于"):
        resolve_live_policy(load_card(CROSS), "formation")


@pytest.mark.parametrize("card,policy", [(CROSS_SITL, "crossing"), (SWITCH_SITL, "switch")])
def test_dry_run_lists_fleet_policy_without_px4_params(card, policy):
    plan = live_topic_plan(load_card(card), policy=policy)
    assert plan["policy"] == policy
    assert plan["n_drones"] == 3
    assert plan["inter_drone_transport"] == "inprocess"
    assert plan["px4_parameters_written"] == []
    assert plan["shadow"] is False
    shell = subprocess.run(
        ["bash", str(ROOT / "scripts/sitl/run_yopo_l0_sitl.sh"), "--dry-run", "--n", "3",
         "--scene", str(card.relative_to(ROOT)), "--policy", policy],
        cwd=ROOT, check=False, capture_output=True, text=True,
    )
    assert shell.returncode == 0, shell.stderr
    assert f"policy={policy}" in shell.stdout


def test_run_script_rejects_unknown_policy():
    shell = subprocess.run(
        ["bash", str(ROOT / "scripts/sitl/run_yopo_l0_sitl.sh"), "--dry-run", "--policy", "bogus"],
        cwd=ROOT, check=False, capture_output=True, text=True,
    )
    assert shell.returncode == 2


def test_switch_session_flies_to_the_slots_on_fake_px4():
    session = make_live_session(
        load_card(SWITCH_SITL), seed=0, policy="switch",
        live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card", plan_timeout_s=60.0),
    )
    assert session.fleet is not None and session.bus is not None
    assert all(slot.bridge.policy is None for slot in session.slots)
    vehicles = [FakePx4Vehicle(np.array([0.0, 0.0, 1.5])) for _ in session.slots]
    t = 0.0
    while t < 40.0 and not all(slot.arrival is not None for slot in session.slots):
        inbox = {
            slot.ident["namespace"]: VehicleInbox(
                odometry=odometry_message_from_odometry(v.odometry(), int(round(t * 1e6))),
                armed=True, offboard=True, landed=False,
            )
            for slot, v in zip(session.slots, vehicles)
        }
        for v, cmd in zip(vehicles, session.step(t, inbox)):
            if cmd.setpoint is not None:
                v.step(cmd.setpoint, DT)
        t = round(t + DT, 10)
    result = session.result()
    assert session.fail_reason == ""
    assert result["arrived"] == 3
    assert result["policy"] == "switch"
    assert "switch" in result and "formation" not in result
    assert result["formation_error_p95_m"] < 0.2
    assert result["min_inter_drone_distance_plan_m"] > 1.2
    assert result["px4_parameters_written"] == []


def test_crossing_session_feeds_scene_card_depth_into_the_protocol():
    session = make_live_session(
        load_card(CROSS_SITL), seed=0, policy="crossing",
        live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card"),
    )
    starts = [np.array([0.0, 0.0, 1.5]) for _ in session.slots]
    seen = []
    orig = session.fleet.ingest

    def spy(did, frame, state):
        seen.append((did, frame, state))
        return orig(did, frame, state)

    session.fleet.ingest = spy
    session.step(0.0, _inbox(session, starts))
    assert {did for did, _, _ in seen} == {"uav0", "uav1", "uav2"}
    for did, frame, state in seen:
        slot = next(s for s in session.slots if s.bridge.drone_id == did)
        # 里程计加回场景卡起点偏置后再渲染，位置在卡的世界系里。
        assert np.allclose(state.p, slot.bridge.start_enu, atol=1e-6)
        assert np.asarray(frame.depth).ndim == 2
    assert session.result()["depth_sources"] == ["scene-card"]


def test_flight_clearance_excludes_the_landing_segment():
    session = make_live_session(
        load_card(SWITCH_SITL), seed=0, policy="switch",
        live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card"),
    )
    high = [np.array([0.0, 0.0, 1.5]) for _ in session.slots]
    session.step(0.0, _inbox(session, high))
    plan = session.result()
    assert plan["min_surface_clearance_plan_m"] == pytest.approx(plan["min_surface_clearance_m"])
    assert plan["min_surface_clearance_flight_m"] == pytest.approx(plan["min_surface_clearance_m"])
    for slot in session.slots:
        slot.phase = "LAND"
    low = [np.array([0.0, 0.0, 0.40]) for _ in session.slots]
    session.step(DT, _inbox(session, low, t=DT))
    after = session.result()
    # 旧字段语义不变：降落段贴地也算。新字段只看飞行段。
    assert after["min_surface_clearance_m"] == pytest.approx(0.40 - 0.35, abs=1e-6)
    assert after["min_surface_clearance_plan_m"] == pytest.approx(plan["min_surface_clearance_plan_m"])
    assert after["min_surface_clearance_flight_m"] == pytest.approx(plan["min_surface_clearance_flight_m"])
    assert after["min_inter_drone_distance_plan_m"] == pytest.approx(plan["min_inter_drone_distance_plan_m"])
    assert "降落" in after["clearance_note"]


def test_clock_skew_meter_reports_offset_jitter_and_inter_vehicle_spread():
    meter = ClockSkewMeter()
    assert meter.summary() is None
    meter.note(None, {"px4_1": 1.0})
    assert meter.summary() is None
    meter.note(10.0, {"px4_1": 9.98, "px4_2": 9.96, "px4_3": None})
    meter.note(10.02, {"px4_1": 10.0, "px4_2": 10.02})
    s = meter.summary()
    assert s["per_vehicle"]["px4_1"]["samples"] == 2
    assert s["per_vehicle"]["px4_1"]["offset_range_s"] == pytest.approx(0.0, abs=1e-9)
    assert s["per_vehicle"]["px4_2"]["offset_range_s"] == pytest.approx(0.04)
    assert s["inter_vehicle_spread_max_s"] == pytest.approx(0.02)
    assert "px4_3" not in s["per_vehicle"]


def test_lazy_depth_frame_renders_once_on_first_depth_read():
    from types import SimpleNamespace

    from yopo_l0.sitl_bridge import _LazyDepthFrame

    calls = []

    class Cam:
        cfg = SimpleNamespace(intrinsics=(1.0, 1.0, 0.5, 0.5))

        def capture(self, p, yaw, stamp):
            calls.append((tuple(p), yaw, stamp))
            return SimpleNamespace(depth=np.ones((2, 3), dtype=np.float32))

    frame = _LazyDepthFrame(Cam(), np.array([1.0, 2.0, 1.5]), 0.3, 4.0)
    assert frame.stamp == 4.0 and frame.K == (1.0, 1.0, 0.5, 0.5)
    assert not frame.rendered and calls == []
    assert frame.depth.shape == (2, 3)
    assert frame.depth.shape == (2, 3)
    assert calls == [((1.0, 2.0, 1.5), 0.3, 4.0)]
    assert frame.rendered


def test_clock_skew_meter_skips_zero_odometry_stamps():
    from yopo_l0.sitl_bridge import ClockSkewMeter

    meter = ClockSkewMeter()
    meter.note(10.0, {"px4_1": 1000.0, "px4_2": 1000.1})
    meter.note(10.02, {"px4_1": 1000.02, "px4_2": 0.0})
    out = meter.summary()
    assert out["zero_stamps_skipped"] == 1
    assert out["per_vehicle"]["px4_2"]["samples"] == 1
    assert out["inter_vehicle_spread_max_s"] < 0.2
