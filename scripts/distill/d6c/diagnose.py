#!/usr/bin/env python3
"""D6c：解释 D6b 学生在开阔三机平行里到不了终点。只报数，不改飞行库。

权重不在仓库里。这里用两样能复现的东西走 ``eval_altitudes.retarget_card`` 同一条闭环：

- 参考轨迹：``predict`` 换成朝目标的直线，或换成教师起步那一窗的位置结点。
  后面的旋转、``knots=position``、``ego_lookahead``、``hold_goal_z`` 仍是 ``StudentPolicy.plan``。
- 仓库样本上 1 个 epoch 的小权重（D5b r4 的 ``FitConfig``，场次少，不是 D6b 那份 ``student.pt``）。

    .venv/bin/python scripts/distill/d6c/diagnose.py --out /tmp/d6c-diagnosis
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

OPEN_CARD = ROOT / "scenes" / "open-3drone.json"
SAMPLE_ROOT = ROOT / "data" / "teacher" / "sample"
TEACHER_PARALLEL = SAMPLE_ROOT / "d000-n3-parallel-s0" / "trajectory.npz"

# 从 yopo-restart-bot/d6b-eval-results @ 3ff89c6 的 d6b-eval-runs.json 读出的数。
# 本脚本不重飞那份权重。派生量在 published_reading() 里现算。
PUBLISHED = {
    "source": "yopo-restart-bot/d6b-eval-results docs/research/d6b-eval-runs.json",
    "commit": "3ff89c6",
    "weights_sha256": "e4bd211662b478066f3611ac8ca6e9cbc6b13445dfdaef82440c04bc53e1f3f0",
    "holdout_position_xy_rmse_m": 0.0658,
    "open_goal_distance_m": 6.0,
    "open_duration_s": 8.0,
    "open_final_goal_error_m": 4.042491172677189,
    "open_min_inter_drone_distance_m": 3.9766344748665308,
    "open_start_separation_m": 5.0,
    "open_seeds": 10,
    "open_altitudes_m": [1.0, 1.5],
    "open_error_unique_per_altitude": 1,
    "open_error_spread_m": 0.0,
    "swap_goal_distance_m": 16.0,
    "swap_duration_s": 40.0,
    "swap_final_goal_error_m": 6.313362368630015,
    "swap_arrived_per_seed": 1,
    "g2_goal_x_m": 5.0,
    "g2_altitude_m": 1.5,
    "g2_seed": 0,
    "g2_final_goal_error_m": 2.680721867276408,
    "g2_sim_duration_s": 1.31,
    "g2_outcome": "COLLISION",
}

R4 = {
    "knots": "position",
    "yaw_source": "ego_lookahead",
    "accel_source": "reference",
    "vertical": "hold_goal_z",
}
D5 = {
    "knots": "network",
    "yaw_source": "calculate_yaw",
    "accel_source": "state",
    "vertical": "network",
}
CRUISE_MPS = 1.33
ARRIVE_RADIUS_M = 1.5
GOAL_TOLERANCE_M = 0.4


def _load(name: str, path: Path):
    cached = sys.modules.get(name)
    if cached is not None and getattr(cached, "__file__", None) == str(path):
        return cached
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"无法加载 {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _eval_altitudes():
    return _load("d6b_eval_altitudes", ROOT / "scripts" / "distill" / "d6" / "eval_altitudes.py")


def _feasibility():
    return _load("d6_feasibility", ROOT / "scripts" / "distill" / "d6" / "feasibility.py")


def published_reading() -> dict:
    """由已发表的终点误差和场景几何现算。直线假设写在字段名里。"""
    open_closed = PUBLISHED["open_goal_distance_m"] - PUBLISHED["open_final_goal_error_m"]
    swap_closed = PUBLISHED["swap_goal_distance_m"] - PUBLISHED["swap_final_goal_error_m"]
    g2_along = PUBLISHED["g2_goal_x_m"] - PUBLISHED["g2_final_goal_error_m"]
    return {
        "published": dict(PUBLISHED),
        "open_along_track_if_straight_m": open_closed,
        "open_mean_closing_mps_if_straight": open_closed / PUBLISHED["open_duration_s"],
        "swap_along_track_if_straight_m": swap_closed,
        "swap_mean_closing_mps_if_straight": swap_closed / PUBLISHED["swap_duration_s"],
        "g2_along_track_if_straight_and_on_axis_m": g2_along,
        "g2_mean_speed_mps_if_straight_and_on_axis": g2_along / PUBLISHED["g2_sim_duration_s"],
        "note": (
            "open / swap 的沿轨距离是「航程减终点误差」。"
            "只有最差那一架沿直线飞向目标时，它才等于走过的路程。"
            "g2 再假设侧向位移是 0、高度保持在目标上。"
            "已发表的 json 没有逐机位置，这两条都是读数之后的换算，不是轨迹测量。"
        ),
    }


def interface_report() -> dict:
    """D5b r4 成功闭环和 D6b 评测在代码里的差。不跑仿真。"""
    from yopo_l0.policy import StudentPolicy
    from yopo_l0.scene import load_card

    card = load_card(OPEN_CARD)
    crit = card.card["criteria"]
    drone = card.card["drones"][0]
    start = drone["start_enu"]
    goal = drone["goal_enu"]
    return {
        "d5_defaults": {key: StudentPolicy.DEFAULTS[key] for key in D5},
        "r4_eval_params": dict(R4),
        "d5b_r4_how": (
            "scripts/distill/d5b/rollout.py 的 variant=repo 把 student_params 写进 "
            "StudentPolicy.DEFAULTS。r4 的四个值见 docs/research/d5b-student-fix-runs.json。"
            "场景卡不改高度，开阔三机平行的 z 本来就是 1.5。"
        ),
        "d6b_how": (
            "scripts/distill/d6/eval_altitudes.py 的 retarget_card 把起终点和悬停点的 z 改成目标高度，"
            "并把同一组 r4 参数写进这次内存卡的 policy_params.student。不改 DEFAULTS，不写场景文件。"
        ),
        "same_four_keys": R4 == {
            "knots": "position",
            "yaw_source": "ego_lookahead",
            "accel_source": "reference",
            "vertical": "hold_goal_z",
        },
        "observation": (
            "两边都走 pack_ego_state：机体系速度、加速度、目标相对位置。"
            "深度都是观测图缩到 24×40 再除以 max_range_m。邻机是上一规划周期的里程计。"
        ),
        "frame": (
            "位置结点先乘 R_yaw 到世界系。knots=position 时速度由世界系位置中心差分得到，不用速度头。"
            "ego_lookahead 朝 1 秒后的轨迹点转偏航，限速 π rad/s。"
        ),
        "altitude": (
            "D5b r4 不平移 z。D6b 在 1.5 m 上把 z 写成 1.5，和卡上的数相同；在 1.0 m 上只改 z。"
            "起终点一起改，目标相对位置的 z 仍是 0。变的是深度里的地面。"
        ),
        "goal_criterion": {
            "runner_tolerance_m": float(crit["goal_tolerance_m"]),
            "runner_speed_mps": float(crit["goal_speed_mps"]),
            "runner_hold_s": float(crit["goal_hold_s"]),
            "student_arrive_radius_m": float(StudentPolicy.DEFAULTS["arrive_radius_m"]),
            "student_arrive_mode": StudentPolicy.DEFAULTS["arrive_mode"],
            "open_start_enu": [float(x) for x in start],
            "open_goal_enu": [float(x) for x in goal],
            "open_duration_s": float(card.card["task"]["duration_s"]),
            "note": (
                "到达成功看场景卡：误差 ≤ 0.4 m、速度 ≤ 0.2 m/s、保持 1 s。"
                "学生自己在 1.5 m 内换成 stop_at_goal，不再问网络。"
                "D6b 开阔平行的终点误差 4.04 m，两种半径都还没进。"
            ),
        },
    }


def teacher_start_forward_m() -> np.ndarray:
    """d000-n3-parallel-s0 第 0 架、t=0 起未来 8 个 0.2 s 结点的前向位移（米）。

    偏航约是 0，航程沿 +x。教师在第一秒几乎不动，1.6 s 才走出零点几米。
    """
    from yopo_l0.student import HORIZON_DT_S, HORIZON_STEPS

    with np.load(TEACHER_PARALLEL) as npz:
        time_s = np.asarray(npz["time_s"], dtype=np.float64)
        position = np.asarray(npz["position_m"], dtype=np.float64)
    knots = float(time_s[0]) + HORIZON_DT_S * np.arange(1, HORIZON_STEPS + 1)
    x = np.interp(knots, time_s, position[:, 0, 0])
    return (x - float(position[0, 0, 0])).astype(np.float64)


def label_speeds(sample_root: Path = SAMPLE_ROOT) -> dict:
    """成功样本里，水平目标距离落到各档时的水平速度。"""
    rows_d = []
    rows_s = []
    n_runs = 0
    for record_path in sorted(sample_root.glob("*/record.json")):
        record = json.loads(record_path.read_text(encoding="utf-8"))
        if record.get("metrics", {}).get("success") is not True:
            continue
        with np.load(record_path.parent / "trajectory.npz") as npz:
            position = np.asarray(npz["position_m"], dtype=np.float64)
            velocity = np.asarray(npz["velocity_mps"], dtype=np.float64)
            goal = np.asarray(npz["goal_m"], dtype=np.float64)
        n_runs += 1
        for drone in range(position.shape[1]):
            dist = np.linalg.norm(position[:, drone, :2] - goal[drone, :2], axis=1)
            speed = np.linalg.norm(velocity[:, drone, :2], axis=1)
            rows_d.append(dist)
            rows_s.append(speed)
    dist = np.concatenate(rows_d)
    speed = np.concatenate(rows_s)
    bins = []
    for lo, hi in ((5.5, 6.5), (0.0, 1.0), (16.0, 20.0)):
        mask = (dist >= lo) & (dist < hi)
        bins.append({
            "goal_distance_m": [lo, hi],
            "n": int(mask.sum()),
            "speed_median_mps": float(np.median(speed[mask])) if mask.any() else None,
        })
    forward = teacher_start_forward_m()
    return {
        "n_success_runs": n_runs,
        "bins": bins,
        "teacher_start_forward_m": [float(x) for x in forward],
        "teacher_start_file": str(TEACHER_PARALLEL.relative_to(ROOT)),
    }


def _as_batch(state) -> np.ndarray:
    state = np.asarray(state, dtype=np.float64)
    if state.ndim == 1:
        state = state.reshape(1, -1)
    return state


def trajectory_along_goal(state, forward_m: np.ndarray, speed_mps: np.ndarray | None = None):
    """把一维前向位移安到目标相对位置的方向上。返回 ContinuousTrajectory。"""
    from yopo_l0.student import HORIZON_STEPS, ContinuousTrajectory

    state = _as_batch(state)
    goal = state[:, 6:9]
    dist = np.linalg.norm(goal, axis=1, keepdims=True)
    direction = np.divide(goal, np.maximum(dist, 1e-9))
    direction = np.where(dist > 1e-6, direction, 0.0)
    forward = np.asarray(forward_m, dtype=np.float64).reshape(1, HORIZON_STEPS, 1)
    pos = direction.reshape(-1, 1, 3) * forward
    if speed_mps is None:
        vel = np.zeros_like(pos)
    else:
        speed = np.asarray(speed_mps, dtype=np.float64).reshape(1, HORIZON_STEPS, 1)
        vel = direction.reshape(-1, 1, 3) * speed
    yaw = np.zeros((state.shape[0], HORIZON_STEPS), dtype=np.float32)
    return ContinuousTrajectory(
        position_body=pos.astype(np.float32),
        velocity_body=vel.astype(np.float32),
        yaw_rel=yaw,
    )


def cruise_forward(speed_mps: float = CRUISE_MPS) -> tuple[np.ndarray, np.ndarray]:
    from yopo_l0.student import HORIZON_DT_S, HORIZON_STEPS

    times = HORIZON_DT_S * np.arange(1, HORIZON_STEPS + 1, dtype=np.float64)
    return speed_mps * times, np.full(HORIZON_STEPS, speed_mps, dtype=np.float64)


def make_predict(kind: str):
    """替换 yopo_l0.student.predict。kind 是 cruise 或 teacher_start。"""
    if kind == "cruise":
        forward, speed = cruise_forward()
    elif kind == "teacher_start":
        forward, speed = teacher_start_forward_m(), None
    else:
        raise ValueError(f"未知参考轨迹 {kind}")

    def predict(model, depth, state, neighbors=None, neighbor_mask=None, *, enabled=None):
        if enabled is False:
            return None
        return trajectory_along_goal(state, forward, speed)

    return predict


def _round_list(values, digits=4) -> list[float]:
    return [round(float(x), digits) for x in np.asarray(values, dtype=float).reshape(-1)]


def first_observation(altitude_m: float) -> dict:
    """开阔三机平行 uav0 的第一帧。噪声关掉，只看高度把深度和目标向量改了多少。"""
    from yopo_l0.scene import load_card
    from yopo_l0.sensor import CameraConfig, DepthCamera
    from yopo_l0.student import depth_from_hw, pack_ego_state

    card = load_card(OPEN_CARD)
    raw = json.loads(json.dumps(card.card))
    altitude = float(altitude_m)
    for drone in raw["drones"]:
        for key in ("start_enu", "goal_enu", "hover_enu"):
            point = drone.get(key)
            if isinstance(point, list) and len(point) >= 3:
                drone[key] = [float(point[0]), float(point[1]), altitude]
    raw["camera"] = dict(raw["camera"])
    raw["camera"]["noise_std_m"] = 0.0
    drone = raw["drones"][0]
    position = np.asarray(drone["start_enu"], dtype=np.float64)
    goal = np.asarray(drone["goal_enu"], dtype=np.float64)
    camera = DepthCamera(CameraConfig.from_card(raw["camera"]), card.obstacles, np.random.default_rng(0))
    frame = camera.capture(position, 0.0, 0.0)
    depth = depth_from_hw(frame.depth, float(raw["camera"]["max_range_m"]))
    state = pack_ego_state(position, np.zeros(3), np.zeros(3), goal, 0.0)
    return {
        "altitude_m": altitude,
        "goal_body": _round_list(state[6:9], 6),
        "depth": depth,
        "depth_finite_frac": float(np.mean(np.isfinite(frame.depth))),
    }


def altitude_observation() -> dict:
    low = first_observation(1.0)
    high = first_observation(1.5)
    delta = np.abs(low["depth"].astype(np.float64) - high["depth"].astype(np.float64))
    goal_delta = np.max(np.abs(np.asarray(low["goal_body"]) - np.asarray(high["goal_body"])))
    return {
        "noise_std_m": 0.0,
        "goal_body_1_0": low["goal_body"],
        "goal_body_1_5": high["goal_body"],
        "goal_body_max_abs": float(goal_delta),
        "depth_mean_abs": float(delta.mean()),
        "depth_changed_frac": float(np.count_nonzero(delta) / delta.size),
        "depth_finite_frac_1_0": low["depth_finite_frac"],
        "depth_finite_frac_1_5": high["depth_finite_frac"],
    }


def _jsonable(value):
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.integer):
        return int(value)
    return value


def train_one_epoch(out_dir: Path, epochs: int = 1) -> dict:
    """仓库成功样本，D5b r4 的 FitConfig，epoch 数由参数给定。默认 1。"""
    from yopo_l0.student_fit import FitConfig, train_from_paths

    feas = _feasibility()
    data = out_dir / "sample"
    manifest_path = data / "manifest.json"
    if not manifest_path.is_file():
        feas.convert_samples(data, 0.0, SAMPLE_ROOT)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    windows = data / "windows"
    train_paths = [windows / f"{name}.npz" for name in manifest["train_ids"]]
    hold_paths = [windows / f"{name}.npz" for name in manifest["hold_ids"]]
    fit = dict(feas.R4_FIT)
    fit["epochs"] = int(epochs)
    train_dir = out_dir / "train"
    summary = train_from_paths(train_paths, hold_paths, train_dir, cfg=FitConfig(**fit))
    summary["n_success_runs"] = manifest["n_success_runs"]
    summary["n_windows"] = manifest["n_windows"]
    summary["train_ids"] = manifest["train_ids"]
    summary["hold_ids"] = manifest["hold_ids"]
    return summary


def _keep_trace(spec: dict) -> bool:
    """种子 0 的对照场保留每一次规划拍。其余场只留汇总。"""
    if int(spec["seed"]) != 0:
        return False
    if spec["name"] in {"pause-r4", "trained-solo"}:
        return True
    return spec["name"] in {"cruise-r4", "trained-r4"} and float(spec["altitude_m"]) in (1.0, 1.5)


def _summarize(spec: dict, result: dict, rows: list[dict], worker_s: float) -> dict:
    by_id: dict[str, list[dict]] = {}
    for row in rows:
        by_id.setdefault(row["id"], []).append(row)
    drones = []
    for drone_id, items in by_id.items():
        errors = [float(item["goal_error_m"]) for item in items]
        speeds = [float(np.linalg.norm(item["v"])) for item in items]
        horizons = [float(item["horizon_forward_m"]) for item in items if "horizon_forward_m" in item]
        early = [float(np.linalg.norm(item["p_body_0"])) for item in items if "p_body_0" in item]
        drones.append({
            "id": drone_id,
            "final_p": items[-1]["p"],
            "final_goal_error_m": errors[-1],
            "min_goal_error_m": min(errors),
            "max_speed_mps": max(speeds) if speeds else None,
            "horizon_forward_median_m": float(np.median(horizons)) if horizons else None,
            "first_knot_median_m": float(np.median(early)) if early else None,
            "entered_arrive_radius": bool(min(errors) < ARRIVE_RADIUS_M),
            "entered_goal_tolerance": bool(min(errors) < GOAL_TOLERANCE_M),
            "final_code": items[-1]["code"],
            "final_yaw": items[-1]["yaw"],
            "n_plans": len(items),
        })
    return {
        "name": spec["name"],
        "predict": spec["predict"],
        "params": spec["params"],
        "altitude_m": float(spec["altitude_m"]),
        "seed": int(spec["seed"]),
        "n_drones": int(result.get("n_drones") or 0),
        "outcome": result.get("outcome"),
        "reason": result.get("reason"),
        "arrived": int(result.get("arrived") or 0),
        "final_goal_error_m": result.get("final_goal_error_m"),
        "sim_duration_s": result.get("sim_duration_s"),
        "min_obstacle_distance_m": result.get("min_obstacle_distance_m"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "worker_s": worker_s,
        "drones": drones,
    }


def _specs(weights: str) -> list[dict]:
    specs = []

    def add(name, predict, params, altitude, seed, drones=3):
        specs.append({
            "name": name,
            "predict": predict,
            "params": dict(params),
            "altitude_m": float(altitude),
            "seed": int(seed),
            "drones": int(drones),
            "weights": weights,
            "duration_s": None,
        })

    for altitude in (1.0, 1.5):
        for seed in (0, 1, 2):
            add("cruise-r4", "cruise", R4, altitude, seed)
            add("trained-r4", "network", R4, altitude, seed)
    add("pause-r4", "teacher_start", R4, 1.5, 0)
    add("trained-solo", "network", R4, 1.5, 0, drones=1)
    for name, params in (
        ("cruise-d5", D5),
        ("cruise-vertical-network", {**R4, "vertical": "network"}),
        ("cruise-knots-network", {**R4, "knots": "network"}),
        ("cruise-yaw-calculate", {**R4, "yaw_source": "calculate_yaw"}),
        ("cruise-accel-state", {**R4, "accel_source": "state"}),
    ):
        add(name, "cruise", params, 1.5, 0)
    for name, params in (
        ("trained-vertical-network", {**R4, "vertical": "network"}),
        ("trained-knots-network", {**R4, "knots": "network"}),
        ("trained-yaw-calculate", {**R4, "yaw_source": "calculate_yaw"}),
        ("trained-accel-state", {**R4, "accel_source": "state"}),
    ):
        add(name, "network", params, 1.5, 0)
    return specs


def run_episode(spec: dict) -> dict:
    """一场闭环。接口参数写在内存卡上，predict 替换在 finally 里还原。"""
    wanted = dict(spec["params"])
    import os

    os.environ["OMP_NUM_THREADS"] = "1"
    import torch

    torch.set_num_threads(1)
    from yopo_l0.policy import StudentPolicy
    from yopo_l0.runner import RunConfig, run
    from yopo_l0.scene import SceneCard, load_card
    from yopo_l0.student import HORIZON_STEPS

    defaults = dict(StudentPolicy.DEFAULTS)
    eval_mod = _eval_altitudes()
    card = eval_mod.retarget_card(load_card(OPEN_CARD), float(spec["altitude_m"]), "off", spec["weights"])
    raw = json.loads(json.dumps(card.card))
    raw["policy_params"]["student"].update(wanted)
    if int(spec.get("drones") or 3) == 1:
        raw["drones"] = [raw["drones"][0]]
    card = SceneCard(card.path, raw, card.obstacles)
    bucket: list[dict] = []
    rows: list[dict] = []
    inner = make_predict(spec["predict"]) if spec["predict"] != "network" else None
    import yopo_l0.student as student_mod

    original_predict = student_mod.predict
    original_plan = StudentPolicy.plan

    def predict(model, depth, state, neighbors=None, neighbor_mask=None, *, enabled=None):
        fn = original_predict if inner is None else inner
        out = fn(model, depth, state, neighbors, neighbor_mask, enabled=enabled)
        position = None if out is None else np.asarray(out.position_body, dtype=np.float32)
        velocity = None if out is None else np.asarray(out.velocity_body, dtype=np.float32)
        bucket.append({
            "state": np.asarray(state, dtype=np.float32).copy(),
            "position_body": None if position is None else position.copy(),
            "velocity_body": None if velocity is None else velocity.copy(),
        })
        return out

    def plan(self, obs):
        before = len(bucket)
        result = original_plan(self, obs)
        call = bucket[before] if len(bucket) > before else None
        position = np.asarray(obs.state.p, dtype=float)
        goal = np.asarray(obs.goal, dtype=float)
        row = {
            "t": round(float(obs.now), 4),
            "id": self.ctx.owner,
            "p": _round_list(position, 4),
            "v": _round_list(obs.state.v, 4),
            "yaw": round(float(obs.state.yaw), 4),
            "goal_error_m": round(float(np.linalg.norm(position - goal)), 4),
            "code": self.last_code,
            "n_neighbors": len(self._neighbors),
        }
        if call is not None and call["position_body"] is not None:
            body = call["position_body"]
            body = body[0] if body.ndim == 3 else body
            vel = call["velocity_body"]
            vel = vel[0] if vel.ndim == 3 else vel
            state = np.asarray(call["state"], dtype=float).reshape(-1)
            row["goal_body"] = _round_list(state[6:9], 4)
            row["p_body_0"] = _round_list(body[0], 4)
            row["p_body_end"] = _round_list(body[HORIZON_STEPS - 1], 4)
            row["horizon_forward_m"] = round(float(np.linalg.norm(body[HORIZON_STEPS - 1])), 4)
            row["v_body_end_mps"] = round(float(np.linalg.norm(vel[HORIZON_STEPS - 1])), 4)
        rows.append(row)
        return result

    student_mod.predict = predict
    StudentPolicy.plan = plan
    started = time.perf_counter()
    try:
        output = run(
            card,
            RunConfig(
                seed=int(spec["seed"]),
                policy="student",
                shadow=False,
                duration_s=None if spec.get("duration_s") is None else float(spec["duration_s"]),
            ),
        )
    finally:
        student_mod.predict = original_predict
        StudentPolicy.plan = original_plan
    if dict(StudentPolicy.DEFAULTS) != defaults:
        raise RuntimeError("StudentPolicy.DEFAULTS 被这场闭环改掉了")
    summary = _summarize(spec, output.result, rows, time.perf_counter() - started)
    if _keep_trace(spec):
        summary["planner"] = rows
    print(
        f"{spec['name']} z={spec['altitude_m']} seed={spec['seed']} drones={spec.get('drones', 3)} "
        f"{summary['outcome']} err={summary['final_goal_error_m']} arrived={summary['arrived']}",
        flush=True,
    )
    return summary


def run_study(out_dir: Path, epochs: int = 1) -> dict:
    from yopo_l0.policy import StudentPolicy
    from yopo_l0.student import checkpoint_sha256

    out_dir.mkdir(parents=True, exist_ok=True)
    defaults = dict(StudentPolicy.DEFAULTS)
    trained = train_one_epoch(out_dir, epochs=epochs)
    weights = str(trained["weights"])
    episodes = [run_episode(spec) for spec in _specs(weights)]
    if dict(StudentPolicy.DEFAULTS) != defaults:
        raise RuntimeError("整次诊断结束时 DEFAULTS 和开始不一致")
    payload = {
        "report_only": True,
        "disclaimer": (
            "report-only。小权重是仓库样本上现训的，不是 D6b 的 student.pt。"
            "参考轨迹替换的是 predict，StudentPolicy.plan 的接口分支没有改。"
            "不修改 StudentPolicy.DEFAULTS，不改场景卡文件。"
        ),
        "published_reading": published_reading(),
        "interface": interface_report(),
        "labels": label_speeds(),
        "altitude_observation": altitude_observation(),
        "train": {
            "epochs": int(epochs),
            "weights_sha256": checkpoint_sha256(weights),
            "n_success_runs": trained.get("n_success_runs"),
            "n_windows": trained.get("n_windows"),
            "train_ids": trained.get("train_ids"),
            "hold_ids": trained.get("hold_ids"),
            "holdout": trained.get("holdout"),
            "fit": trained.get("fit"),
            "wall_s": trained.get("wall_s"),
        },
        "cruise_mps": CRUISE_MPS,
        "episodes": episodes,
    }
    dest = out_dir / "d6c-diagnosis-runs.json"
    dest.write_text(json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {dest}", flush=True)
    return payload


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="D6c：开阔三机平行到不了终点的归因")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=1)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if int(args.epochs) < 1:
        raise SystemExit("--epochs 至少为 1")
    run_study(args.out, epochs=int(args.epochs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
