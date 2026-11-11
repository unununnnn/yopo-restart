"""从采样轨迹和规划日志计算到达、碰撞、间距、规划时间。

测不到的量保持 None。文档里把 None 写成「未知」。
障碍距离用 ``yopo_l0.scene`` 的圆柱/地面公式，和 L0 场景卡是同一把尺子。
碰撞线：表面距离 <= 机体半径，机间中心距 <= 两倍机体半径。机体半径是假设。
"""

from __future__ import annotations

import math
import re
from decimal import Decimal

import numpy as np

from yopo_l0.scene import parse_geometry

from .assumptions import BODY_RADIUS_M, GOAL_HOLD_S, GOAL_RADIUS_M, GOAL_SPEED_MPS, SAMPLE_HZ


def _as_float_list(raws: list[str]) -> list[float]:
    return [float(Decimal(s)) for s in raws]


def parse_planner_log(text: str) -> dict:
    """解析 ego_planner 打到标准输出的 ``total time`` 和 ``plan_success``。"""
    colored = re.findall(r"total time:\x1b\[42m([0-9.eE+-]+)", text)
    raws = colored or re.findall(r"total time:([0-9.eE+-]+)", text)
    per_drone: dict[str, list[str]] = {}
    for match in re.finditer(
        r"\[drone_(\d+)_ego_planner_node[^\]]*\][^\n]*?total time:(?:\x1b\[42m)?([0-9.eE+-]+)",
        text,
    ):
        per_drone.setdefault(match.group(1), []).append(match.group(2))
    vals = _as_float_list(raws)
    out = {
        "planning_time_s": vals,
        "planning_time_count": len(vals),
        "planning_time_mean_s": None,
        "planning_time_p95_s": None,
        "planning_time_min_s": None,
        "planning_time_max_s": None,
        "planning_time_by_drone_mean_s": None,
        "plan_success_0": text.count("plan_success=0"),
        "plan_success_1": text.count("plan_success=1"),
        # 同一件事会先打 Suddenly discovered，再打一行 to EMERGENCY_STOP。只数前一行。
        "emergency_stop": text.count("Suddenly discovered obstacles. emergency stop!"),
    }
    if vals:
        ordered = sorted(vals)
        idx = math.ceil(0.95 * len(ordered)) - 1
        out["planning_time_mean_s"] = float(sum(vals) / len(vals))
        out["planning_time_p95_s"] = ordered[idx]
        out["planning_time_min_s"] = ordered[0]
        out["planning_time_max_s"] = ordered[-1]
    if per_drone and sum(len(v) for v in per_drone.values()) == len(vals) and vals:
        out["planning_time_by_drone_mean_s"] = {
            k: float(sum(_as_float_list(v)) / len(v)) for k, v in sorted(per_drone.items())
        }
    return out


def evaluate_trajectory(
    time_s: np.ndarray,
    position_m: np.ndarray,
    velocity_mps: np.ndarray,
    starts: np.ndarray,
    goals: np.ndarray,
    geometry: dict,
    *,
    body_radius_m: float = BODY_RADIUS_M,
    ground_z: float = 0.0,
) -> dict:
    """``position_m`` 形状 (T, N, 3)。没有样本时，到达和碰撞都是 None。"""
    time_s = np.asarray(time_s, dtype=float)
    position_m = np.asarray(position_m, dtype=float)
    velocity_mps = np.asarray(velocity_mps, dtype=float)
    goals = np.asarray(goals, dtype=float)
    starts = np.asarray(starts, dtype=float)
    if position_m.size == 0 or time_s.size == 0:
        return {
            "arrived": None,
            "collision_obstacle": None,
            "collision_inter_drone": None,
            "min_inter_drone_m": None,
            "min_inter_drone_inflight_m": None,
            "min_obstacle_surface_m": None,
            "min_obstacle_clearance_m": None,
            "final_goal_distance_m": None,
            "final_speed_mps": None,
        }
    t = position_m.shape[0]
    n = position_m.shape[1]
    obstacles = parse_geometry(geometry, ground_z)
    min_surface = math.inf
    for i in range(t):
        for k in range(n):
            dist = min(o.distance(position_m[i, k]) for o in obstacles)
            if dist < min_surface:
                min_surface = dist
    min_pair = None
    min_pair_flight = None
    collision_inter = False
    if n >= 2:
        min_pair = math.inf
        min_pair_flight = math.inf
        for i in range(t):
            moved = False
            for k in range(n):
                if float(np.linalg.norm(position_m[i, k] - starts[k])) > 1.0:
                    moved = True
            for a in range(n):
                for b in range(a + 1, n):
                    dist = float(np.linalg.norm(position_m[i, a] - position_m[i, b]))
                    if dist < min_pair:
                        min_pair = dist
                    if moved and dist < min_pair_flight:
                        min_pair_flight = dist
                    if dist <= 2.0 * body_radius_m:
                        collision_inter = True
        if min_pair_flight is math.inf:
            min_pair_flight = None
    clearance = None if min_surface is math.inf else float(min_surface - body_radius_m)
    collision_obstacle = bool(min_surface <= body_radius_m + 1e-9)
    final_goal = [float(np.linalg.norm(position_m[-1, k] - goals[k])) for k in range(n)]
    final_speed = [float(np.linalg.norm(velocity_mps[-1, k])) for k in range(n)]
    arrived = _held(time_s, position_m, velocity_mps, goals)
    return {
        "arrived": arrived,
        "collision_obstacle": collision_obstacle,
        "collision_inter_drone": collision_inter,
        "min_inter_drone_m": None if min_pair is None else float(min_pair),
        "min_inter_drone_inflight_m": None if min_pair_flight is None else float(min_pair_flight),
        "min_obstacle_surface_m": None if min_surface is math.inf else float(min_surface),
        "min_obstacle_clearance_m": clearance,
        "final_goal_distance_m": final_goal,
        "final_speed_mps": final_speed,
    }


def _held(time_s, position_m, velocity_mps, goals) -> bool:
    """一段连续样本都在到达球内、速度也低于阈值，并且跨过保持时间。

    采样是 10 Hz，允许少一拍，避免相位刚好卡在 0.9 s 上被判失败。
    """
    n = position_m.shape[1]
    count = int(time_s.shape[0])
    if count == 0:
        return False
    inside = []
    for i in range(count):
        ok = True
        for k in range(n):
            dist = float(np.linalg.norm(position_m[i, k] - goals[k]))
            speed = float(np.linalg.norm(velocity_mps[i, k]))
            if dist > GOAL_RADIUS_M or speed > GOAL_SPEED_MPS:
                ok = False
                break
        inside.append(ok)
    need = GOAL_HOLD_S - (1.0 / SAMPLE_HZ)
    i = 0
    while i < count:
        if not inside[i]:
            i += 1
            continue
        j = i
        while j + 1 < count and inside[j + 1]:
            j += 1
        if float(time_s[j] - time_s[i]) + 1e-9 >= need:
            return True
        i = j + 1
    return False


def success_of(run_status: str, trajectory: dict) -> bool | None:
    """有轨迹才下结论。进程没留下样本时是 None（未知），不是失败。"""
    if trajectory.get("arrived") is None:
        return None
    if run_status not in ("arrived", "timeout", "stopped"):
        return None
    return bool(
        trajectory["arrived"]
        and not trajectory["collision_obstacle"]
        and not trajectory["collision_inter_drone"]
    )
