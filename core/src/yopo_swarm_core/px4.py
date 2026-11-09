"""PX4 边界（ADR-0011）：ROS-free 的坐标系转换与 px4_msgs 字段构造。

仿真飞控固定为 PX4 v1.17.0 SITL + uXRCE-DDS（Micro-XRCE-DDS-Agent v2.4.3）+ px4_msgs release/1.17。
PX4 的约定（px4_msgs 注释原文）：
- 世界系 NED（x 北、y 东、z 下），机体系 FRD（x 前、y 右、z 下）。
- 四元数 [w, x, y, z]，Hamilton，表示 FRD 机体 → NED 世界的旋转（VehicleAttitude.q / VehicleOdometry.q）。
- TrajectorySetpoint：NED 下的 position / velocity / acceleration / jerk，yaw 为 NED 航向（-π..π），NaN = 不控制。
- 外部控制必须持续发 OffboardControlMode（> 2 Hz），再切 offboard（VehicleCommand 176，param1=1，param2=6）。
- /fmu/out/* 话题用 BEST_EFFORT + TRANSIENT_LOCAL + KEEP_LAST QoS 订阅；MESSAGE_VERSION>0 的消息话题带 `_vN` 后缀。

规划核心内部保持 ENU 世界系 / FLU 机体系（上游 YOPO 网络按 FLU 训练，金样与 G0–G3 测试据此建立），
与 PX4 的转换只在这里做一次，ROS 节点不得自己再写一份。
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

PX4_VERSION = "v1.17.0"
PX4_MSGS_BRANCH = "release/1.17"
XRCE_AGENT_VERSION = "v2.4.3"
XRCE_DEFAULT_PORT = 8888

# px4_msgs release/1.17 的 MESSAGE_VERSION（决定话题后缀）
MESSAGE_VERSION = {
    "trajectory_setpoint": 0,
    "offboard_control_mode": 0,
    "vehicle_command": 0,
    "vehicle_command_ack": 0,
    "vehicle_odometry": 0,
    "vehicle_attitude": 0,
    "vehicle_local_position": 1,
    "vehicle_status": 1,
    "vehicle_land_detected": 0,
}

VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
POSE_FRAME_NED = 1
VELOCITY_FRAME_NED = 1

# ENU ↔ NED 与 FLU ↔ FRD 都是对合（自身的逆）
_ENU_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_FLU_FRD = np.diag([1.0, -1.0, -1.0])


def topic(direction: str, name: str, namespace: str = "") -> str:
    """PX4 uXRCE-DDS 话题名：[/ns]/fmu/{in|out}/name[_vN]。"""
    if direction not in ("in", "out"):
        raise ValueError(f"direction must be 'in' or 'out', got {direction!r}")
    v = MESSAGE_VERSION.get(name, 0)
    ns = namespace.strip("/")
    return (f"/{ns}" if ns else "") + f"/fmu/{direction}/{name}" + (f"_v{v}" if v else "")


def enu_to_ned(v: Sequence[float]) -> np.ndarray:
    return _ENU_NED @ np.asarray(v, dtype=float)


def ned_to_enu(v: Sequence[float]) -> np.ndarray:
    return _ENU_NED @ np.asarray(v, dtype=float)


def flu_to_frd(v: Sequence[float]) -> np.ndarray:
    return _FLU_FRD @ np.asarray(v, dtype=float)


def frd_to_flu(v: Sequence[float]) -> np.ndarray:
    return _FLU_FRD @ np.asarray(v, dtype=float)


def wrap_pi(a: float) -> float:
    return math.remainder(a, 2.0 * math.pi)


def yaw_enu_to_ned(yaw_enu: float) -> float:
    """ENU 航向（从东逆时针）→ NED 航向（从北顺时针）。"""
    return wrap_pi(math.pi / 2.0 - yaw_enu)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    return wrap_pi(math.pi / 2.0 - yaw_ned)


def quat_to_rot(q: Sequence[float]) -> np.ndarray:
    """[w, x, y, z]（Hamilton，单位化）→ 旋转矩阵。"""
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 → [w, x, y, z]，w ≥ 0。"""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q if q[0] >= 0 else -q


def R_enu_flu_from_ned_frd(R_ned_frd: np.ndarray) -> np.ndarray:
    """PX4 姿态（FRD→NED）→ 规划核心用的 R_wb（FLU→ENU）。"""
    return _ENU_NED @ np.asarray(R_ned_frd, dtype=float) @ _FLU_FRD


def R_ned_frd_from_enu_flu(R_enu_flu: np.ndarray) -> np.ndarray:
    return _ENU_NED @ np.asarray(R_enu_flu, dtype=float) @ _FLU_FRD


@dataclass(frozen=True)
class EnuState:
    """从 PX4 VehicleOdometry 转来的规划核心状态（世界 ENU，R_wb: FLU→ENU）。"""

    position: np.ndarray
    velocity: np.ndarray
    R_wb: np.ndarray
    stamp_us: int


def odometry_to_enu(
    position_ned: Sequence[float],
    q_ned_frd: Sequence[float],
    velocity_ned: Sequence[float],
    timestamp_us: int,
    pose_frame: int = POSE_FRAME_NED,
    velocity_frame: int = VELOCITY_FRAME_NED,
) -> EnuState:
    """VehicleOdometry → EnuState。只接受 NED 位姿与 NED 速度；其他帧或 NaN 直接拒绝，不猜。"""
    if pose_frame != POSE_FRAME_NED or velocity_frame != VELOCITY_FRAME_NED:
        raise ValueError(f"unsupported odometry frames pose={pose_frame} vel={velocity_frame}")
    p, q, v = (np.asarray(x, dtype=float) for x in (position_ned, q_ned_frd, velocity_ned))
    if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q)) and np.all(np.isfinite(v))):
        raise ValueError("odometry contains NaN/inf")
    return EnuState(
        position=ned_to_enu(p),
        velocity=ned_to_enu(v),
        R_wb=R_enu_flu_from_ned_frd(quat_to_rot(q)),
        stamp_us=int(timestamp_us),
    )


def trajectory_setpoint(
    pva_enu: np.ndarray, yaw_enu: float | None, timestamp_us: int, yawspeed_enu: float | None = None
) -> dict:
    """规划核心 (p, v, a)（ENU，形状 (3,3)）→ px4_msgs/TrajectorySetpoint 字段（NED）。

    yaw 为 None 时填 NaN（PX4 不控航向）。jerk 只用于日志，填 NaN。
    """
    pva = np.asarray(pva_enu, dtype=float)
    if pva.shape != (3, 3) or not np.all(np.isfinite(pva)):
        raise ValueError(f"pva must be finite with shape (3,3), got {pva.shape}")
    nan3 = [math.nan] * 3
    return {
        "timestamp": int(timestamp_us),
        "position": [float(x) for x in enu_to_ned(pva[0])],
        "velocity": [float(x) for x in enu_to_ned(pva[1])],
        "acceleration": [float(x) for x in enu_to_ned(pva[2])],
        "jerk": nan3,
        "yaw": math.nan if yaw_enu is None else yaw_enu_to_ned(yaw_enu),
        # ENU 绕上轴逆时针为正 → NED 绕下轴顺时针为正：数值取反
        "yawspeed": math.nan if yawspeed_enu is None else -float(yawspeed_enu),
    }


def offboard_control_mode(timestamp_us: int) -> dict:
    """位置 + 速度 + 加速度前馈的 offboard 心跳（TrajectorySetpoint 三项都填）。"""
    return {
        "timestamp": int(timestamp_us),
        "position": True,
        "velocity": True,
        "acceleration": True,
        "attitude": False,
        "body_rate": False,
        "thrust_and_torque": False,
        "direct_actuator": False,
    }


def vehicle_command(
    command: int, timestamp_us: int, target_system: int = 1, **params: float
) -> dict:
    """VehicleCommand 字段；param1..param7 用关键字给，其余为 0。"""
    msg = {f"param{i}": 0.0 for i in range(1, 8)}
    for k, val in params.items():
        if k not in msg:
            raise ValueError(f"unknown param {k}")
        msg[k] = float(val)
    msg.update(
        timestamp=int(timestamp_us),
        command=int(command),
        target_system=int(target_system),
        target_component=1,
        source_system=1,
        source_component=1,
        from_external=True,
    )
    return msg


def offboard_mode_command(timestamp_us: int, target_system: int = 1) -> dict:
    return vehicle_command(
        VEHICLE_CMD_DO_SET_MODE,
        timestamp_us,
        target_system,
        param1=1.0,
        param2=float(PX4_CUSTOM_MAIN_MODE_OFFBOARD),
    )


def arm_command(timestamp_us: int, target_system: int = 1, arm: bool = True) -> dict:
    return vehicle_command(
        VEHICLE_CMD_COMPONENT_ARM_DISARM, timestamp_us, target_system, param1=1.0 if arm else 0.0
    )
