"""飞行航向（#34 / Y1）：原样移植 YOPO-MINCO d5e1e60 `YOPO/policy/poly_solver.py:139-170`。

`calculate_yaw` 把速度方向与目标方向混合，目标权重随航向误差增大（90° 时两者等权），
再按 `max_yaw_rate·π` rad/s 限速（默认 0.5·π rad/s，不是 0.5 rad/s）。
运算顺序与上游逐行一致，float64 结果与上游逐位相同；由
`core/tests/golden/minco_node_d5e1e60.npz` 锁定（ADR-0006 2026-10-26 修订）。

坐标系：上游在 FLU 世界系里算 atan2(y, x)；本仓库世界系是 ENU，同为右手、z 向上，
所以同一公式直接给出 ENU 航向（x 轴为 0，逆时针为正），再由 `px4` 模块转换到 NED。
"""

from __future__ import annotations

import numpy as np

MAX_YAW_RATE = 0.5  # 上游 calculate_yaw 默认值，单位 π rad/s
CTRL_DT = 0.02  # 上游 test_yopo_ros.py ctrl_dt（50 Hz 控制定时器）


def wrap_to_pi(angle):
    """将角度限制在 [-pi, pi)。"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def calculate_yaw(vel_dir, goal_dir, last_yaw, dt, max_yaw_rate=MAX_YAW_RATE):
    """返回 (yaw, yaw_dot)。goal_dir 为目标减当前（期望）位置；距目标 ≤ 0.5 m 时保持 last_yaw。"""
    vel_dir = vel_dir / (np.linalg.norm(vel_dir) + 1e-5)
    goal_dist = np.linalg.norm(goal_dir)
    goal_dir = goal_dir / (goal_dist + 1e-5)

    goal_yaw = np.arctan2(goal_dir[1], goal_dir[0])
    delta_yaw = wrap_to_pi(goal_yaw - last_yaw)
    weight = 2 * abs(delta_yaw) / np.pi

    dir_des = vel_dir + weight * goal_dir
    yaw_desired = np.arctan2(dir_des[1], dir_des[0]) if goal_dist > 0.5 else last_yaw

    yaw_diff = wrap_to_pi(yaw_desired - last_yaw)
    max_yaw_change = max_yaw_rate * np.pi * dt
    yaw_change = np.clip(yaw_diff, -max_yaw_change, max_yaw_change)

    yaw = wrap_to_pi(last_yaw + yaw_change)
    yawdot = yaw_change / dt
    return yaw, yawdot
