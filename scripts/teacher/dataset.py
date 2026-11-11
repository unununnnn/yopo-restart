"""把一次教师运行写成 L0 能读的 npz + record.json。

深度图不放进 npz。``observation_at`` 用场景卡和该拍的位姿，走
``yopo_l0.sensor.DepthCamera`` 现算。这样训练和评估看见的是同一份圆柱。
邻机有两份：该拍的状态，以及到该拍为止最后一条广播 B 样条。
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from yopo_l0.dynamics import State
from yopo_l0.policy import Observation
from yopo_l0.scene import load_card
from yopo_l0.sensor import CameraConfig, DepthCamera

from .assumptions import SCHEMA_VERSION
from .metrics import evaluate_trajectory, parse_planner_log, success_of
from .scene_card import write_scene_card


def geometry_of(scenario: dict) -> dict:
    return {
        "boxes": [],
        "cylinders": [
            {
                "name": c["name"],
                "center_xy": [float(c["center_xy"][0]), float(c["center_xy"][1])],
                "radius": float(c["radius"]),
                "z_lo": float(c["z_lo"]),
                "z_hi": float(c["z_hi"]),
            }
            for c in scenario["cylinders"]
        ],
    }


def write_dataset(run_dir: Path, scenario: dict, raw: dict, log_text: str) -> dict:
    """读容器留下的 raw_record，写出 trajectory.npz、record.json、scene.json。"""
    time_s = np.asarray(raw.get("time_s") or [], dtype=np.float64)
    position = np.asarray(raw.get("position_m") or [], dtype=np.float64)
    velocity = np.asarray(raw.get("velocity_mps") or [], dtype=np.float64)
    acceleration = np.asarray(raw.get("acceleration_mps2") or [], dtype=np.float64)
    yaw = np.asarray(raw.get("yaw_rad") or [], dtype=np.float64)
    if time_s.size:
        position = position.reshape(time_s.shape[0], scenario["n_drones"], 3)
        velocity = velocity.reshape(time_s.shape[0], scenario["n_drones"], 3)
        acceleration = acceleration.reshape(time_s.shape[0], scenario["n_drones"], 3)
        yaw = yaw.reshape(time_s.shape[0], scenario["n_drones"])
    else:
        n = scenario["n_drones"]
        position = np.zeros((0, n, 3))
        velocity = np.zeros((0, n, 3))
        acceleration = np.zeros((0, n, 3))
        yaw = np.zeros((0, n))
    starts = np.asarray([d["start_enu"] for d in scenario["drones"]], dtype=np.float64)
    goals = np.asarray([d["goal_enu"] for d in scenario["drones"]], dtype=np.float64)
    geom = geometry_of(scenario)
    centers = np.asarray([c["center_xy"] for c in scenario["cylinders"]], dtype=np.float64).reshape(-1, 2)
    radii = np.asarray([c["radius"] for c in scenario["cylinders"]], dtype=np.float64)
    z_lo = np.asarray([c["z_lo"] for c in scenario["cylinders"]], dtype=np.float64)
    z_hi = np.asarray([c["z_hi"] for c in scenario["cylinders"]], dtype=np.float64)
    np.savez_compressed(
        run_dir / "trajectory.npz",
        time_s=time_s,
        position_m=position,
        velocity_mps=velocity,
        acceleration_mps2=acceleration,
        yaw_rad=yaw,
        drone_ros_id=np.asarray([d["ros_id"] for d in scenario["drones"]], dtype=np.int32),
        start_m=starts,
        goal_m=goals,
        cylinder_center_xy_m=centers,
        cylinder_radius_m=radii,
        cylinder_z_lo_m=z_lo,
        cylinder_z_hi_m=z_hi,
    )
    traj = evaluate_trajectory(time_s, position, velocity, starts, goals, geom)
    log_stats = parse_planner_log(log_text)
    status = raw.get("run_status") or "stopped"
    success = success_of(status, traj)
    record = {
        "schema": SCHEMA_VERSION,
        "scenario": scenario,
        "run": {
            "status": status,
            "wall_s": raw.get("wall_s"),
            "t0_ros_s": raw.get("t0_ros_s"),
            "sample_hz": raw.get("sample_hz"),
            "bspline_import": raw.get("bspline_import"),
            "yaw_source": raw.get("yaw_source"),
            "acceleration_source": raw.get("acceleration_source"),
            "n_broadcast": sum(1 for b in raw.get("bsplines") or [] if b.get("topic") == "broadcast"),
            "n_local_bspline": sum(1 for b in raw.get("bsplines") or [] if b.get("topic") == "local"),
        },
        "metrics": {
            "success": success,
            **traj,
            **log_stats,
            "body_radius_m_assumption": scenario["body_radius_m"],
        },
        "bsplines": raw.get("bsplines") or [],
        "null_means": "未知",
    }
    (run_dir / "record.json").write_text(
        json.dumps(record, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_scene_card(scenario, run_dir)
    return record


def load_arrays(run_dir: Path):
    npz = np.load(run_dir / "trajectory.npz")
    record = json.loads((run_dir / "record.json").read_text(encoding="utf-8"))
    return npz, record


def observation_at(run_dir: Path, drone_index: int, sample_index: int, *, depth_seed: int = 0):
    """返回 L0 ``Observation``，以及邻机状态和当时有效的广播 B 样条。

    深度是 L0 光线投射，不是教师容器里的点云渲染。
    """
    npz, record = load_arrays(run_dir)
    card = load_card(run_dir / "scene.json")
    time_s = npz["time_s"]
    if sample_index < 0 or sample_index >= time_s.shape[0]:
        raise IndexError(f"sample_index={sample_index}，样本数 {time_s.shape[0]}")
    n = npz["position_m"].shape[1]
    if drone_index < 0 or drone_index >= n:
        raise IndexError(f"drone_index={drone_index}，机数 {n}")
    cam_cfg = CameraConfig.from_card(card.card["camera"])
    rng = np.random.default_rng(depth_seed)
    camera = DepthCamera(cam_cfg, card.obstacles, rng)
    now = float(time_s[sample_index])
    p = np.asarray(npz["position_m"][sample_index, drone_index], dtype=float)
    v = np.asarray(npz["velocity_mps"][sample_index, drone_index], dtype=float)
    a = np.asarray(npz["acceleration_mps2"][sample_index, drone_index], dtype=float)
    yaw = float(npz["yaw_rad"][sample_index, drone_index])
    goal = np.asarray(npz["goal_m"][drone_index], dtype=float)
    frame = camera.capture(p, yaw, now)
    state = State(now, p, v, a, yaw)
    obs = Observation(frame, state, goal, now)
    t0 = record["run"].get("t0_ros_s")
    query_ros = None if t0 is None else float(t0) + now
    neighbours = []
    for j in range(n):
        if j == drone_index:
            continue
        neighbours.append(
            {
                "drone_index": j,
                "ros_id": int(npz["drone_ros_id"][j]),
                "p": np.asarray(npz["position_m"][sample_index, j], dtype=float),
                "v": np.asarray(npz["velocity_mps"][sample_index, j], dtype=float),
                "a": np.asarray(npz["acceleration_mps2"][sample_index, j], dtype=float),
                "yaw": float(npz["yaw_rad"][sample_index, j]),
                "broadcast": _latest_broadcast(record["bsplines"], int(npz["drone_ros_id"][j]), query_ros),
            }
        )
    return obs, neighbours


def _latest_broadcast(bsplines: list[dict], ros_id: int, query_ros: float | None):
    chosen = None
    for msg in bsplines:
        if msg.get("topic") != "broadcast":
            continue
        if int(msg.get("drone_id", -1)) != ros_id:
            continue
        if query_ros is not None and msg.get("receive_ros_s") is not None:
            if float(msg["receive_ros_s"]) > query_ros + 1e-9:
                continue
        chosen = msg
    if chosen is None:
        return None
    return {
        "order": chosen.get("order"),
        "traj_id": chosen.get("traj_id"),
        "knots": chosen.get("knots"),
        "pos_pts": chosen.get("pos_pts"),
        "yaw_pts": chosen.get("yaw_pts"),
        "yaw_dt": chosen.get("yaw_dt"),
        "start_time_ros_s": chosen.get("start_time_ros_s"),
    }


def check_record(record: dict) -> list[str]:
    """缺字段就返回说明。不替代 schema.json，只抓会让读取失败的洞。"""
    missing = []
    for key in ("schema", "scenario", "run", "metrics", "bsplines"):
        if key not in record:
            missing.append(key)
    metrics = record.get("metrics") or {}
    for key in (
        "success",
        "arrived",
        "collision_obstacle",
        "collision_inter_drone",
        "min_inter_drone_m",
        "min_obstacle_surface_m",
        "planning_time_mean_s",
        "planning_time_count",
    ):
        if key not in metrics:
            missing.append(f"metrics.{key}")
    scenario = record.get("scenario") or {}
    for key in ("id", "density_per_m2", "n_drones", "pattern", "cylinders", "drones", "assumption"):
        if key not in scenario:
            missing.append(f"scenario.{key}")
    return missing
