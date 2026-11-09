"""ADR-0011：PX4 边界的坐标系转换与 px4_msgs 字段。"""

import math

import numpy as np
import pytest
from hypothesis import given
from hypothesis import strategies as st

from yopo_swarm_core import px4

finite = st.floats(-1e3, 1e3, allow_nan=False)
vec3 = st.tuples(finite, finite, finite)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def test_axes_map_per_px4_convention():
    # ENU 东 → NED y（东）；ENU 北 → NED x；ENU 上 → NED -z
    assert np.allclose(px4.enu_to_ned([1, 0, 0]), [0, 1, 0])
    assert np.allclose(px4.enu_to_ned([0, 1, 0]), [1, 0, 0])
    assert np.allclose(px4.enu_to_ned([0, 0, 1]), [0, 0, -1])
    # FLU 左 → FRD -右；上 → -下
    assert np.allclose(px4.flu_to_frd([1, 2, 3]), [1, -2, -3])


@given(vec3)
def test_conversions_are_involutions(v):
    assert np.allclose(px4.ned_to_enu(px4.enu_to_ned(v)), v)
    assert np.allclose(px4.frd_to_flu(px4.flu_to_frd(v)), v)


@pytest.mark.parametrize(
    ("enu", "ned"),
    [(0.0, math.pi / 2), (math.pi / 2, 0.0), (math.pi, -math.pi / 2), (-math.pi / 2, math.pi)],
)
def test_yaw(enu, ned):
    # ENU 朝东(0) = NED 朝东(π/2)；ENU 朝北(π/2) = NED 0
    assert math.isclose(abs(px4.wrap_pi(px4.yaw_enu_to_ned(enu) - ned)), 0.0, abs_tol=1e-12)
    assert math.isclose(abs(px4.wrap_pi(px4.yaw_ned_to_enu(ned) - enu)), 0.0, abs_tol=1e-12)


@given(st.floats(-math.pi, math.pi), st.floats(-0.5, 0.5), st.floats(-0.5, 0.5))
def test_attitude_roundtrip_and_yaw_consistency(yaw, pitch, roll):
    # 在 NED/FRD 里构造 PX4 姿态（ZYX 欧拉），转到 ENU/FLU 再回来
    def rx(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    def ry(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    R_ned = rz(yaw) @ ry(pitch) @ rx(roll)
    q = px4.rot_to_quat(R_ned)
    assert np.allclose(px4.quat_to_rot(q), R_ned, atol=1e-9)
    R_enu = px4.R_enu_flu_from_ned_frd(R_ned)
    assert np.allclose(R_enu @ R_enu.T, np.eye(3), atol=1e-9)
    assert math.isclose(np.linalg.det(R_enu), 1.0, abs_tol=1e-9)
    assert np.allclose(px4.R_ned_frd_from_enu_flu(R_enu), R_ned, atol=1e-9)
    # 机头方向（FLU x / FRD x）在两系里指向同一物理方向
    assert np.allclose(px4.ned_to_enu(R_ned[:, 0]), R_enu[:, 0], atol=1e-9)
    # 航向换算与姿态换算一致
    yaw_enu = math.atan2(R_enu[1, 0], R_enu[0, 0])
    if abs(math.cos(pitch)) > 1e-6:
        assert abs(px4.wrap_pi(px4.yaw_ned_to_enu(yaw) - yaw_enu)) < 1e-9


def test_level_north_heading_is_enu_yaw_90deg():
    st_ = px4.odometry_to_enu([1, 2, -3], [1, 0, 0, 0], [0.5, 0, 0], 123)
    assert np.allclose(st_.position, [2, 1, 3])
    assert np.allclose(st_.velocity, [0, 0.5, 0])
    assert np.allclose(st_.R_wb, rz(math.pi / 2), atol=1e-12)
    assert st_.stamp_us == 123


def test_odometry_rejects_nan_and_unknown_frames():
    with pytest.raises(ValueError):
        px4.odometry_to_enu([math.nan, 0, 0], [1, 0, 0, 0], [0, 0, 0], 0)
    with pytest.raises(ValueError):
        px4.odometry_to_enu([0, 0, 0], [1, 0, 0, 0], [0, 0, 0], 0, pose_frame=2)


def test_trajectory_setpoint_fields():
    pva = np.array([[1.0, 2.0, 3.0], [0.1, 0.2, 0.3], [0.0, 0.0, -1.0]])
    m = px4.trajectory_setpoint(pva, yaw_enu=0.0, timestamp_us=7, yawspeed_enu=0.2)
    assert m["position"] == [2.0, 1.0, -3.0]
    assert m["velocity"] == pytest.approx([0.2, 0.1, -0.3])
    assert m["acceleration"] == [0.0, 0.0, 1.0]
    assert all(math.isnan(x) for x in m["jerk"])
    assert m["yaw"] == pytest.approx(math.pi / 2)
    assert m["yawspeed"] == pytest.approx(-0.2)
    assert math.isnan(px4.trajectory_setpoint(pva, None, 0)["yaw"])
    with pytest.raises(ValueError):
        px4.trajectory_setpoint(np.full((3, 3), math.nan), 0.0, 0)


def test_topics_follow_px4_message_versions():
    assert px4.topic("in", "trajectory_setpoint") == "/fmu/in/trajectory_setpoint"
    assert px4.topic("out", "vehicle_status") == "/fmu/out/vehicle_status_v1"
    assert px4.topic("out", "vehicle_local_position", "px4_1") == (
        "/px4_1/fmu/out/vehicle_local_position_v1"
    )
    with pytest.raises(ValueError):
        px4.topic("up", "x")


def test_commands():
    m = px4.offboard_mode_command(5, target_system=2)
    assert (m["command"], m["param1"], m["param2"], m["target_system"]) == (176, 1.0, 6.0, 2)
    assert px4.arm_command(5)["command"] == 400 and px4.arm_command(5, arm=False)["param1"] == 0.0
    ocm = px4.offboard_control_mode(9)
    assert ocm["position"] and ocm["velocity"] and ocm["acceleration"] and not ocm["attitude"]
