from dataclasses import replace

import numpy as np

from yopo_swarm_core.depth import optical_pose
from yopo_swarm_core.freespace import CameraIntrinsics, DepthFrame, FreeSpace, SpaceVerdict
from yopo_swarm_core.planner import Planner, VehicleState
from yopo_swarm_core.synthetic import Box, ScriptedLatticeSource, render_depth
from yopo_swarm_core.takeoff import TakeoffContract
from yopo_swarm_core.uav import UavRuntime, default_config


def fixture(extra=()):
    cfg = default_config("px4_1", 0.35, 0.15, 0.125)
    state = VehicleState(np.zeros(3), np.zeros(3), np.zeros(3), np.eye(3), 0.0)
    k = CameraIntrinsics(70.0, 70.0, 80.0, 48.0, 160, 96)
    boxes = (
        Box((-10.0, -10.0, -0.4), (10.0, 10.0, -0.24)),
        Box((5.0, -10.0, -0.24), (5.2, 10.0, 4.0)),
        *extra,
    )
    pose = optical_pose(state.position, state.R_wb)
    raw = render_depth(boxes, k, pose, 20.0)
    frame = DepthFrame.from_raw(raw, k, pose, 0.0, cfg.space)
    rt = UavRuntime(Planner(cfg, ScriptedLatticeSource()), TakeoffContract())
    return rt, frame, state


def test_ground_occupancy_retained_but_bounded_support_takeoff_can_be_approved():
    rt, frame, state = fixture()
    ordinary = FreeSpace(rt.planner.cfg.space)
    ordinary.add(frame)
    assert ordinary.classify_point(np.array([0.2, 0.0, 0.0]), 0.525, 0.0) is SpaceVerdict.OCCUPIED
    original = frame.depth.copy()
    assert rt.bootstrap(frame, state, 0.0, 1.2)
    np.testing.assert_array_equal(original, frame.depth)
    assert rt.command(0.125) is not None
    assert rt.observation(frame, state, [5.0, 0.0, 1.5], 0.125) is None
    frame.stamp = 0.125
    rt.update_takeoff(frame, replace(state, stamp=0.125), 0.125, True, True)
    assert rt.pending is not None
    np.testing.assert_allclose(
        rt.pending.plan.terminal_point()[2], rt.takeoff.ground_z + 0.525 + 0.3, rtol=0.0, atol=1e-12
    )
    assert (
        rt.takeoff.space.classify_point(np.array([0.0, 0.0, -0.1]), 0.525, 0.0)
        is SpaceVerdict.OCCUPIED
    )
    assert (
        rt.takeoff.space.classify_point(np.array([0.3, 0.0, 0.0]), 0.525, 0.0)
        is SpaceVerdict.OCCUPIED
    )


def test_support_never_exempts_a_low_obstacle_and_rejects_missing_floor():
    rt, frame, state = fixture((Box((0.46, -0.12, -0.24), (0.58, 0.12, 0.12)),))
    assert not rt.bootstrap(frame, state, 0.0, 1.2)
    assert rt.command(1.0) is None
    rt, frame, state = fixture()
    no_floor = DepthFrame.from_raw(
        np.full_like(frame.depth, 10.0), frame.k, frame.T, 0.0, rt.planner.cfg.space
    )
    assert not rt.bootstrap(no_floor, state, 0.0, 1.2)
    assert rt.command(1.0) is None


def test_takeoff_handoff_requires_stable_actual_altitude_and_restores_full_ground_check():
    rt, frame, state = fixture()
    assert rt.bootstrap(frame, state, 0.0, 1.2)
    rt.command(0.125)
    frame.stamp = 0.125
    rt.update_takeoff(frame, replace(state, stamp=0.125), 0.125, True, True)
    climb = rt.pending.plan
    rt.command(climb.t_switch)
    end = climb.t_end
    frame.stamp = end
    rt.command(end)
    rt.update_takeoff(frame, replace(state, stamp=end), end, True, True)
    assert not rt.takeoff.ready
    high = replace(state, position=climb.terminal_point(), stamp=end)
    pose = optical_pose(high.position, high.R_wb)
    raw = render_depth(
        (
            Box((-10.0, -10.0, -0.4), (10.0, 10.0, -0.24)),
            Box((5.0, -10.0, -0.24), (5.2, 10.0, 4.0)),
        ),
        frame.k,
        pose,
        20.0,
    )
    frame = DepthFrame.from_raw(raw, frame.k, pose, end, rt.planner.cfg.space)
    rt.update_takeoff(frame, high, end, True, True)
    frame.stamp = end + 1.1
    rt.update_takeoff(frame, replace(high, stamp=end + 1.1), end + 1.1, True, True)
    assert not rt.takeoff.ready  # 两次观测之间长间隔不能算连续稳定。
    for dt in np.arange(1.2, 2.3, 0.1):
        frame.stamp = end + dt
        rt.update_takeoff(frame, replace(high, stamp=end + dt), end + dt, True, True)
    assert rt.takeoff.ready
    assert rt.pending.plan.plan_seq == climb.plan_seq + 1
    assert (
        rt.planner.space.classify_point(np.array([1.2, 0.0, 0.0]), 0.525, frame.stamp)
        is SpaceVerdict.OCCUPIED
    )
    rt.command(end + 2.4)
    assert (
        rt.observation(frame, replace(high, stamp=end + 2.2), [5.0, 0.0, 1.5], end + 2.4)
        is not None
    )


def test_stale_inputs_and_disarmed_state_never_start_climb():
    rt, frame, state = fixture()
    assert rt.bootstrap(frame, state, 0.0, 1.2)
    rt.command(0.125)
    rt.update_takeoff(frame, state, 0.125, False, True)
    assert rt.pending is None
    rt.update_takeoff(frame, state, 1.0, True, True)
    assert rt.pending is None


def test_late_takeoff_approval_cannot_jump_over_the_px4_switch_time():
    rt, frame, state = fixture()
    assert rt.bootstrap(frame, state, 0.0, 1.2)
    rt.command(0.125)
    frame.stamp = 0.125
    token = rt.takeoff.propose(rt, frame, replace(state, stamp=0.125), 0.125, True, True)
    parent = rt.current.plan_hash
    assert token is not None
    assert not rt.takeoff.accept(rt, token, parent, 0.125, False, True)
    assert not rt.takeoff.accept(rt, token, parent, 0.125, True, False)
    assert not rt.takeoff.accept(rt, token, parent, token.plan.t_switch)
    assert rt.takeoff.phase == "HOLD"
    assert rt.current.plan_hash == parent
    assert rt.pending is None


def test_async_handoff_cannot_use_a_stability_window_broken_during_approval():
    rt, frame, state = fixture()
    assert rt.bootstrap(frame, state, 0.0, 1.2)
    rt.command(0.125)
    frame.stamp = 0.125
    rt.update_takeoff(frame, replace(state, stamp=0.125), 0.125, True, True)
    climb = rt.pending.plan
    rt.command(climb.t_switch)
    end = climb.t_end
    rt.command(end)
    high = replace(state, position=climb.terminal_point(), stamp=end)
    pose = optical_pose(high.position, high.R_wb)
    raw = render_depth(
        (
            Box((-10.0, -10.0, -0.4), (10.0, 10.0, -0.24)),
            Box((5.0, -10.0, -0.24), (5.2, 10.0, 4.0)),
        ),
        frame.k,
        pose,
        20.0,
    )
    frame = DepthFrame.from_raw(raw, frame.k, pose, end, rt.planner.cfg.space)
    for dt in np.linspace(0.0, 1.1, 12):
        now = end + dt
        frame.stamp = now
        high = replace(high, stamp=now)
        rt.takeoff.observe(rt, high, frame.stamp, now, True, True)
    token = rt.takeoff.propose(rt, frame, high, now, True, True)
    assert token is not None
    unstable = replace(high, position=high.position + np.array([0.0, 0.0, 0.06]), stamp=now + 0.05)
    rt.takeoff.observe(rt, unstable, now + 0.05, now + 0.05, True, True)
    assert not rt.takeoff.accept(rt, token, rt.current.plan_hash, now + 0.05, True, True)
    assert not rt.takeoff.ready
    assert rt.pending is None
