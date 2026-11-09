from dataclasses import replace

import numpy as np
import pytest

from yopo_swarm_core.depth import DepthSample, optical_pose
from yopo_swarm_core.freespace import CameraIntrinsics
from yopo_swarm_core.planner import Diagnostic, Planner, VehicleState
from yopo_swarm_core.synthetic import ScriptedLatticeSource
from yopo_swarm_core.uav import InputBuffer, UavRuntime, default_config, namespace


def setup_runtime():
    cfg = replace(default_config("px4_1"), switch_delay=0.125)
    planner = Planner(cfg, ScriptedLatticeSource(reach=0.2))
    rt = UavRuntime(planner)
    state = VehicleState(np.zeros(3), np.zeros(3), np.zeros(3), np.eye(3), 0.0)
    sample = DepthSample(
        np.full((96, 160), 10.0, np.float32),
        CameraIntrinsics(100.0, 100.0, 80.0, 48.0, 160, 96),
        optical_pose(state.position, state.R_wb),
        0,
    )
    frame = sample.evidence(cfg.space)
    assert rt.bootstrap(frame, state, 0.0, 2.0)
    return rt, frame, state


def test_only_gate_approved_plan_can_produce_commands():
    rt, frame, state = setup_runtime()
    assert rt.current is None
    assert rt.command(0.1) is None
    assert rt.command(0.125).plan_hash == rt.current.plan_hash
    # 新鲜占用证据优先于显式起飞区假设；初始保持也不能旁路门。
    blocked = UavRuntime(Planner(rt.planner.cfg, ScriptedLatticeSource()))
    from yopo_swarm_core.freespace import DepthFrame

    pose = frame.T.copy()
    pose[:3, 3] = 0
    occupied = DepthFrame.from_raw(
        np.full_like(frame.depth, 0.31), frame.k, pose, 0.0, rt.planner.cfg.space
    )
    assert not blocked.bootstrap(occupied, state, 0.0, 2.0)
    assert blocked.command(1.0) is None


def test_pause_and_clock_reset_never_publish_new_commands():
    rt, _, _ = setup_runtime()
    assert rt.command(0.125) is not None
    assert all(rt.command(0.125) is None for _ in range(100))
    assert rt.command(0.2) is not None
    assert rt.command(0.1) is None
    assert rt.clock_fault
    assert rt.command(5.0) is None


def test_late_worker_result_cannot_execute_retroactively():
    rt, frame, state = setup_runtime()
    rt.command(0.125)
    frame.stamp = 0.125
    state = replace(state, stamp=0.125)
    obs = rt.observation(frame, state, [1.0, 0.0, 0.0], 0.125)
    decision = rt.planner.step(obs)
    assert decision.diagnostic is Diagnostic.APPROVED
    parent = rt.current.plan_hash
    assert not rt.accept(decision, parent, decision.approved.plan.t_switch)
    assert rt.last_error == "MISSED_SWITCH"
    assert rt.current.plan_hash == parent


def test_stale_frame_selects_future_approved_brake_and_samples_it_at_branch_time():
    rt, frame, state = setup_runtime()
    rt.command(0.125)
    frame.stamp = 0.125
    state = replace(state, stamp=0.125)
    obs = rt.observation(frame, state, [1.0, 0.0, 0.0], 0.125)
    d = rt.planner.step(obs)
    assert rt.accept(d, rt.current.plan_hash, 0.125)
    rt.command(d.approved.plan.t_switch)
    now = rt.current.plan.t_switch + 0.125
    obs = rt.observation(frame, state, [1.0, 0.0, 0.0], now)
    frame.stamp = -1.0
    d = rt.planner.step(obs)
    assert d.diagnostic is Diagnostic.STALE_FRAME
    assert d.fallback.kind == "BRAKE_BRANCH"
    rt.accept(d, rt.current.plan_hash, now)
    assert rt.command(now).pva_enu.shape == (3, 3)
    assert rt.active_branch is None
    assert rt.command(d.fallback.t_branch) is not None
    assert rt.active_branch is not None


def test_timestamp_pairing_rejects_epoch_or_out_of_order_odometry():
    cfg = default_config("px4_1")
    b = InputBuffer(cfg)
    b.odometry([1, 2, 3], [1, 0, 0, 0], [0, 0, 0], 1_000_000)
    b.odometry([9, 9, 9], [1, 0, 0, 0], [0, 0, 0], 999_000)
    assert len(b.states) == 1
    k = CameraIntrinsics(1, 1, 1, 1, 2, 2)
    assert b.pair(np.ones((2, 2)), k, 1_000_000_000) is not None
    assert b.pair(np.ones((2, 2)), k, 1_792_000_000_000_000_000) is None


def test_initial_hold_waits_for_stable_actual_estimator_reference():
    b = InputBuffer(default_config("px4_1"))
    for t in np.linspace(0.0, 1.1, 112):
        b.odometry([0.0, 0.0, -0.1 * t], [1, 0, 0, 0], [0, 0, 0], int(t * 1e6))
    assert not b.stationary(1.1)
    for t in np.linspace(1.2, 2.4, 122):
        b.odometry([0.0, 0.0, -0.11], [1, 0, 0, 0], [0, 0, 0], int(t * 1e6))
    assert b.stationary(2.4)
    assert not b.stationary(3.0)


def test_ipc_source_removes_batch_and_preserves_raw_depth_below_safety_min_range():
    from yopo_swarm_core.uav import IpcSource

    rt, frame, state = setup_runtime()
    rt.command(0.125)
    obs = rt.observation(frame, state, [1.0, 0.0, 0.0], 0.125)
    source = IpcSource("unused-test-socket")

    class Client:
        def __init__(self):
            self.last_header = {"backend": "fixture"}

        def forward(self, depth, body):
            assert np.all(depth > 0)  # .2 m 在安全掩码之外，但不是网络的 .04 m 洞。
            assert body.shape == (1, 9)
            return np.zeros((1, 15, 14)), np.zeros((1, 15)), np.zeros((1, 15, 20))

    source.client = Client()
    source.bind_depth(np.full((96, 160), 0.2, np.float32), frame.stamp)
    p, s, r = source(obs, rt.planner.cfg.decoder)
    assert (p.shape, s.shape, r.shape) == ((15, 14), (15,), (15, 20))


@pytest.mark.parametrize("value", ["", "/", "uav1", "px4_-1", "px4_1/foo"])
def test_no_global_or_ambiguous_vehicle_namespace(value):
    with pytest.raises(ValueError):
        namespace(value)


def test_command_yaw_follows_upstream_calculate_yaw_and_rate_limit():
    """#34 / Y1：给出目标时航向按上游 control_pub 计算，NED 字段由 ENU 换算，且 |yaw_dot| ≤ 0.5π。"""
    import math

    from yopo_swarm_core.px4 import yaw_enu_to_ned
    from yopo_swarm_core.yaw import CTRL_DT, MAX_YAW_RATE, calculate_yaw

    rt, _, _ = setup_runtime()
    goal = np.array([0.0, 20.0, 1.0])  # 正左方：期望航向 +π/2（ENU）
    legacy = rt.command(0.125, 0.0)
    assert legacy.setpoint["yaw"] == yaw_enu_to_ned(0.0) and math.isnan(legacy.setpoint["yawspeed"])

    rt2, _, _ = setup_runtime()
    c = rt2.command(0.125, 0.0, goal_w=goal)
    want, want_dot = calculate_yaw(c.pva_enu[1], goal - c.pva_enu[0], 0.0, CTRL_DT)
    assert c.setpoint["yaw"] == pytest.approx(yaw_enu_to_ned(want), abs=1e-12)
    assert c.setpoint["yawspeed"] == pytest.approx(-want_dot, abs=1e-12)
    last, t = rt2.last_yaw, 0.125
    for k in range(1, 200):
        t2 = 0.125 + 0.01 * k
        c = rt2.command(t2, 0.0, goal_w=goal)
        w, wd = calculate_yaw(c.pva_enu[1], goal - c.pva_enu[0], last, t2 - t)
        assert rt2.last_yaw == pytest.approx(float(w), abs=1e-12)
        assert abs(wd) <= MAX_YAW_RATE * math.pi + 1e-9
        last, t = rt2.last_yaw, t2
    assert rt2.last_yaw == pytest.approx(math.pi / 2, abs=1e-6)  # 1.99 s × 0.5π rad/s 足以转到位
