"""票 B2：交叉卡规划净空归因。不起 Gazebo，也不连 ROS。"""

import json
import os
import subprocess
import sys

import numpy as np
import pytest

from conftest import ROOT
from yopo_l0 import crossing
from yopo_l0.b2_plan_diag import PlanDiag, _split, apply_b2_overrides, parse_fix
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card

CROSS = ROOT / "scenes/crossing-pillars.json"
CROSS_SITL = ROOT / "scenes/crossing-pillars-sitl80.json"


def _run_l0(seconds, *, diag):
    card = load_card(CROSS)
    holder = {}
    original = crossing.CrossingFleet.from_card.__func__

    def from_card(cls, card_dict, physics_dt, *, record_inputs=False):
        fleet = original(cls, card_dict, physics_dt, record_inputs=record_inputs)
        if diag:
            holder["d"] = PlanDiag(fleet, card.min_distance, 0.35)
        return fleet

    crossing.CrossingFleet.from_card = classmethod(from_card)
    try:
        out = run(card, RunConfig(seed=0, shadow=False, duration_s=seconds))
    finally:
        crossing.CrossingFleet.from_card = classmethod(original)
    return out, holder.get("d")


def test_recorder_does_not_change_the_flight():
    """装上记录器，L0 每拍位置逐位相同；记录器确实记到了提案和每拍。"""
    plain, _ = _run_l0(2.5, diag=False)
    seen, d = _run_l0(2.5, diag=True)
    assert [r["p"] for r in plain.ticks] == [r["p"] for r in seen.ticks]
    s = d.summary()
    assert s["planner_calls"] > 0 and s["proposals"] > 0
    assert s["per_drone"]["uav0"]["ticks"] > 0
    assert s["plan_clear_min_m"] is not None and s["plan_clear_min_m"] > 0.0
    # L0 深度帧和规划同一拍，年龄不超过一个 30 Hz 周期。
    assert s["at_plan_frame_age_s"]["max"] <= 1.0 / 30.0 + 1e-9


def test_split_along_and_lateral():
    along, lat = _split(np.array([1.0, 2.0, 0.0]), np.array([1.0, 0.0]))
    assert along == pytest.approx(1.0) and lat == pytest.approx(2.0)
    along, lat = _split(np.array([0.0, -0.5, 0.0]), np.array([0.0, 0.0]))
    assert along == 0.0 and lat == pytest.approx(0.5)


def test_overrides_default_off_keep_card_hash():
    card = load_card(CROSS_SITL)
    same, dev = apply_b2_overrides(card)
    assert same is card and dev == []
    wide, dev = apply_b2_overrides(card, safe_radius=0.35)
    assert wide.card["policy_params"]["upstream"]["safe_radius"] == 0.35
    assert dev and dev[0].startswith("偏离上游")
    assert card.hash == load_card(CROSS_SITL).hash  # 原卡没被改
    with pytest.raises(ValueError):
        apply_b2_overrides(card, safe_radius=0.0)


def _live(*argv, env=None):
    e = {k: v for k, v in os.environ.items() if not k.startswith(("YOPO_B2_", "YOPO_SITL_", "YOPO_AGENT_"))}
    e.update(env or {})
    e["PYTHONPATH"] = f"{ROOT}:{ROOT / 'core/src'}"
    return subprocess.run(
        [sys.executable, str(ROOT / "scripts/sitl/yopo_l0_live.py"), *argv],
        cwd=ROOT, env=e, capture_output=True, text=True, check=False,
    )


def test_dry_run_b2_switches_default_off(tmp_path):
    out = tmp_path / "p.json"
    r = _live("--scene", str(CROSS_SITL), "--policy", "crossing", "--dry-run", "--out", str(out))
    assert r.returncode == 0, r.stderr
    assert json.loads(out.read_text())["b2"] == {"plan_diag": False, "deviations": [], "fix": {}}
    r = _live("--scene", str(CROSS_SITL), "--policy", "crossing", "--dry-run", "--out", str(out),
              env={"YOPO_B2_PLAN_DIAG": "1", "YOPO_B2_SAFE_RADIUS": "0.35"})
    assert r.returncode == 0, r.stderr
    b2 = json.loads(out.read_text())["b2"]
    assert b2["plan_diag"] is True and b2["deviations"] == ["偏离上游：safe_radius=0.35"]
    r = _live("--scene", str(CROSS_SITL), "--policy", "crossing", "--dry-run", "--out", str(out),
              env={"YOPO_B2_FIX": "lead=0.1"})
    b2 = json.loads(out.read_text())["b2"]
    assert b2["fix"] == {"setpoint_lead_s": 0.1} and b2["deviations"] == ["偏离上游：设定点提前 0.1 s"]
    assert _live("--scene", str(CROSS_SITL), "--policy", "crossing", "--dry-run", "--out", str(out),
                 env={"YOPO_B2_FIX": "lead=2"}).returncode == 2
    assert _live("--scene", str(CROSS_SITL), "--policy", "crossing", "--dry-run", "--out", str(out),
                 env={"YOPO_B2_FIX": "wat=1"}).returncode == 2
    bad = _live("--scene", str(CROSS_SITL), "--policy", "crossing", "--dry-run", "--out", str(out), "--safe-radius", "-1")
    assert bad.returncode == 2


def test_shell_entry_passes_b2_env_through():
    text = (ROOT / "scripts/sitl/run_yopo_l0_sitl.sh").read_text()
    # sudo docker 不继承调用者环境：必须显式带值。
    assert text.count('-e YOPO_B2_PLAN_DIAG="${YOPO_B2_PLAN_DIAG:-}"') == 2
    assert text.count('-e YOPO_B2_FIX="${YOPO_B2_FIX:-}"') == 2


def test_b2_does_not_touch_px4_parameters():
    for rel in ("yopo_l0/b2_plan_diag.py", "scripts/b2/l0_plan_diag.py"):
        text = (ROOT / rel).read_text()
        assert "param set" not in text and "PX4_PARAM_" not in text
        for prefix in ("COM_", "NAV_", "BAT_"):
            assert f'"{prefix}' not in text and f"'{prefix}" not in text


def _fake_inbox(session, positions):
    from yopo_l0.sitl_bridge import FakePx4Vehicle, VehicleInbox, odometry_message_from_odometry

    out = {}
    for slot, p in zip(session.slots, positions):
        vehicle = FakePx4Vehicle(p)
        out[slot.ident["namespace"]] = VehicleInbox(
            odometry=odometry_message_from_odometry(vehicle.odometry(), 0), armed=True, offboard=True, landed=False,
        )
    return out


@pytest.mark.parametrize("lead", [0.0, 0.1])
def test_setpoint_lead_samples_the_committed_plan_ahead(lead):
    """lead=0 与改前同一点；lead>0 时设定点是已提交计划在 t+lead 上的位置。计划本身不受影响。"""
    from yopo_l0.sitl_bridge import LiveFleetConfig, make_live_session

    session = make_live_session(
        load_card(CROSS_SITL), seed=0, policy="crossing",
        live=LiveFleetConfig(initial_phase="PLAN", depth_source="scene-card", setpoint_lead_s=lead),
    )
    starts = [np.asarray(s.bridge.start_enu, dtype=float) for s in session.slots]
    checked = 0
    for k in range(60):
        cmds = session.step(round(k * 0.02, 10), _fake_inbox(session, starts))
        t = session._formation_t
        if t is None:
            continue
        for slot, cmd in zip(session.slots, cmds):
            sampler = slot.formation_sampler
            if sampler is None or cmd.setpoint is None:
                continue
            p = np.asarray(sampler.pva(t + lead)[0]) - np.asarray(slot.offset_enu)
            ned = np.array([p[1], p[0], -p[2]])
            assert np.allclose(np.asarray(cmd.setpoint["position"], dtype=float), ned, atol=1e-5)
            checked += 1
    assert checked > 50


def test_setpoint_lead_is_bounded():
    from yopo_l0.sitl_bridge import LiveFleetConfig

    with pytest.raises(ValueError):
        LiveFleetConfig(setpoint_lead_s=0.6)
    with pytest.raises(ValueError):
        LiveFleetConfig(setpoint_lead_s=-0.1)
    assert parse_fix("") == {} and parse_fix("lead=0.12") == {"setpoint_lead_s": 0.12}


def _fleet(params=None):
    card = dict(load_card(CROSS).card)
    if params is not None:
        card["policy_params"] = params
    return crossing.CrossingFleet.from_card(card, 0.01)


def test_odom_head_is_off_by_default_and_keeps_the_task_hash():
    off = _fleet()
    assert off.head_from_odom_m is None
    on = _fleet({"b2": {"head_from_odom_m": 0.3}})
    assert on.head_from_odom_m == 0.3
    assert off.cfg.task_hash != on.cfg.task_hash  # 打开才进 task_hash
    assert _fleet({}).cfg.task_hash == off.cfg.task_hash
    with pytest.raises(ValueError):
        _fleet({"b2": {"head_from_odom_m": -1}})


def test_odom_head_replaces_the_head_only_beyond_the_threshold():
    from types import SimpleNamespace

    fleet = _fleet({"b2": {"head_from_odom_m": 0.3}})
    head = np.array([[1.0, 0.0, 1.5], [2.0, 0.0, 0.0], [0.5, 0.0, 0.0]])
    frame = SimpleNamespace(stamp=0.0, depth=np.zeros((2, 2)))
    near = SimpleNamespace(p=np.array([0.5, 0.1, 1.5]), v=np.array([2.0, 0.0, 0.0]), yaw=0.0)
    fleet.ingest("uav0", frame, near)
    # 外推 0.25 s：0.5 + 2*0.25 = 1.0，离计划头 0.1 m，不换头。
    assert np.array_equal(fleet._odom_head("uav0", head, 0.0, 0.25), head)
    far = SimpleNamespace(p=np.array([0.5, 0.6, 1.5]), v=np.array([2.0, 0.4, 0.0]), yaw=0.0)
    fleet.ingest("uav0", frame, far)
    out = fleet._odom_head("uav0", head, 0.0, 0.25)
    assert np.allclose(out[0], [1.0, 0.7, 1.5]) and np.allclose(out[1], [2.0, 0.4, 0.0])
    assert np.allclose(out[2], 0.0) and fleet.head_from_odom_count == 1


def test_odomhead_fix_parses_and_marks_deviation(tmp_path):
    assert parse_fix("odomhead=0.3,lead=0.1") == {"head_from_odom_m": 0.3, "setpoint_lead_s": 0.1}
    out = tmp_path / "p.json"
    r = _live("--scene", str(CROSS_SITL), "--policy", "crossing", "--dry-run", "--out", str(out),
              env={"YOPO_B2_FIX": "odomhead=0.3"})
    assert r.returncode == 0, r.stderr
    b2 = json.loads(out.read_text())["b2"]
    assert b2["deviations"] == ["偏离上游：偏差超过 0.3 m 时从里程计起规划"]


def test_stiffen_default_returns_the_same_arrays_and_clips():
    from yopo_l0.b2_plan_diag import stiffen as _stiffen

    p, v = np.array([1.0, 0.0, 1.5]), np.array([2.0, 0.0, 0.0])
    state = {"position_enu": [0.0, 0.5, 1.5], "velocity_enu": [1.0, 0.0, 0.0]}
    p0, v0 = _stiffen(p, v, state, 0.0, 0.0)
    assert p0 is p and v0 is v
    p1, v1 = _stiffen(p, v, state, 1.0, 0.5)
    assert np.allclose(p1, [2.0, -0.5, 1.5]) and np.allclose(v1, [2.5, 0.0, 0.0])
    far = {"position_enu": [-10.0, 0.0, 1.5], "velocity_enu": [2.0, 0.0, 0.0]}
    p2, _ = _stiffen(p, v, far, 5.0, 0.0)
    assert np.linalg.norm(p2 - p) == pytest.approx(1.5)
    assert parse_fix("stiff=1.3:0.7") == {"setpoint_gain_p": 1.3, "setpoint_gain_v": 0.7}
    with pytest.raises(ValueError):
        parse_fix("stiff=0:0")
