"""把一条 EGO-Swarm 教师记录转成单机学生的窗口张量。

读 ``yopo-teacher-record/1``：``trajectory.npz`` 的位姿，``record.json`` 的成败，
``scene.json`` 现算深度。不导入 ``scripts/teacher``，也不读广播 B 样条。
邻机输入是 D4。多机记录直接拒绝，不把第 0 架切出来当单机。

深度按场景卡的视场角光线投射，空间尺寸用学生的 24×40，不是卡上的 96×160。
噪声固定为 0。无回波写成两个通道的 0。
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from yopo_l0.policy import R_yaw
from yopo_l0.scene import load_card
from yopo_l0.sensor import CameraConfig, DepthCamera
from yopo_l0.student import (
    DEPTH_HEIGHT,
    DEPTH_WIDTH,
    HORIZON_DT_S,
    HORIZON_STEPS,
    STATE_DIM,
    StudentConfig,
)

TEACHER_SCHEMA = "yopo-teacher-record/1"
TENSOR_SCHEMA = "yopo-student-window/1"


class DistillError(ValueError):
    """记录缺字段、模式不对，或者机数不是 1。"""


@dataclass(frozen=True)
class WindowBatch:
    depth: np.ndarray  # (N, 2, H, W) float32
    state: np.ndarray  # (N, 9) float32
    position_body: np.ndarray  # (N, K, 3) float32
    velocity_body: np.ndarray  # (N, K, 3) float32
    yaw_rel: np.ndarray  # (N, K) float32
    time_s: np.ndarray  # (N,) float64
    sample_index: np.ndarray  # (N,) int64
    scenario_id: str
    skipped: str | None
    horizon_dt_s: float
    max_range_m: float

    @property
    def n(self) -> int:
        return int(self.state.shape[0])

    def meta(self) -> dict:
        return {
            "tensor_schema": TENSOR_SCHEMA,
            "teacher_schema": TEACHER_SCHEMA,
            "scenario_id": self.scenario_id,
            "n_windows": self.n,
            "skipped": self.skipped,
            "horizon_steps": int(self.position_body.shape[1]) if self.position_body.ndim == 3 else HORIZON_STEPS,
            "horizon_dt_s": self.horizon_dt_s,
            "depth_hw": [DEPTH_HEIGHT, DEPTH_WIDTH],
            "state_dim": STATE_DIM,
            "max_range_m": self.max_range_m,
            "heads": ["position_body", "velocity_body", "yaw_rel"],
            "depth_note": (
                "按场景卡视场角现算，分辨率是学生的 24×40，不是教师卡的 96×160。"
                "通道 0 是深度/max_range，无回波为 0。通道 1 是有效掩码。"
                "噪声固定为 0。"
            ),
            "state_note": "机体坐标系：速度 3、加速度 3、目标相对位置 3。不含邻机。",
            "bsplines": "未读",
        }


def _empty(scenario_id: str, skipped: str, dt: float, max_range: float) -> WindowBatch:
    k = HORIZON_STEPS
    return WindowBatch(
        depth=np.zeros((0, 2, DEPTH_HEIGHT, DEPTH_WIDTH), dtype=np.float32),
        state=np.zeros((0, STATE_DIM), dtype=np.float32),
        position_body=np.zeros((0, k, 3), dtype=np.float32),
        velocity_body=np.zeros((0, k, 3), dtype=np.float32),
        yaw_rel=np.zeros((0, k), dtype=np.float32),
        time_s=np.zeros((0,), dtype=np.float64),
        sample_index=np.zeros((0,), dtype=np.int64),
        scenario_id=scenario_id,
        skipped=skipped,
        horizon_dt_s=dt,
        max_range_m=max_range,
    )


def _load_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise DistillError(f"缺少 {path.name}：{path}") from exc
    except json.JSONDecodeError as exc:
        raise DistillError(f"{path} 不是 JSON：{exc}") from exc


def _require_keys(npz, keys: tuple[str, ...], path: Path):
    missing = [k for k in keys if k not in npz.files]
    if missing:
        raise DistillError(f"{path} 缺少数组 {missing}")


def wrap_angle(angle: np.ndarray) -> np.ndarray:
    return (np.asarray(angle, dtype=np.float64) + np.pi) % (2.0 * np.pi) - np.pi


def pack_depth(depth_m: np.ndarray, max_range_m: float) -> np.ndarray:
    """(H, W) 米，NaN 为无回波。返回 (2, H, W)：归一化深度和有效掩码。"""
    depth_m = np.asarray(depth_m, dtype=np.float32)
    valid = np.isfinite(depth_m)
    values = np.zeros(depth_m.shape, dtype=np.float32)
    if max_range_m <= 0:
        raise DistillError(f"max_range_m 必须为正，收到 {max_range_m}")
    np.divide(depth_m, np.float32(max_range_m), out=values, where=valid)
    np.clip(values, 0.0, 1.0, out=values)
    return np.stack([values, valid.astype(np.float32)], axis=0)


def _interp_rows(times: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    if values.ndim == 1:
        return np.interp(query, times, values).astype(np.float64)
    out = np.empty((query.shape[0], values.shape[1]), dtype=np.float64)
    for c in range(values.shape[1]):
        out[:, c] = np.interp(query, times, values[:, c])
    return out


def _pick(count: int, max_windows: int | None) -> np.ndarray:
    idx = np.arange(count, dtype=np.int64)
    if max_windows is None or count <= max_windows:
        return idx
    if max_windows < 1:
        raise DistillError(f"max_windows 至少为 1，收到 {max_windows}")
    chosen = np.unique(np.round(np.linspace(0, count - 1, int(max_windows))).astype(np.int64))
    return chosen


def student_camera(card_camera: dict, cfg: StudentConfig) -> CameraConfig:
    base = CameraConfig.from_card(card_camera)
    return CameraConfig(
        width=int(cfg.depth_width),
        height=int(cfg.depth_height),
        hfov_deg=base.hfov_deg,
        vfov_deg=base.vfov_deg,
        min_range_m=base.min_range_m,
        max_range_m=base.max_range_m,
        noise_std_m=0.0,
        mount_xyz_body=base.mount_xyz_body,
    )


def convert_run(
    run_dir: Path | str,
    *,
    max_windows: int | None = None,
    cfg: StudentConfig | None = None,
) -> WindowBatch:
    """成功的单机记录 → 窗口。``metrics.success`` 不是 true 时返回空批次，不渲染。"""
    cfg = cfg or StudentConfig()
    if cfg.horizon_steps != HORIZON_STEPS or cfg.depth_height != DEPTH_HEIGHT or cfg.depth_width != DEPTH_WIDTH:
        raise DistillError("本票的转换只产出固定的 8 步、24×40 窗口。")
    run_dir = Path(run_dir)
    record_path = run_dir / "record.json"
    traj_path = run_dir / "trajectory.npz"
    record = _load_json(record_path)
    schema = record.get("schema")
    if schema != TEACHER_SCHEMA:
        raise DistillError(f"{record_path} 的 schema 是 {schema!r}，本转换只读 {TEACHER_SCHEMA}")
    scenario = record.get("scenario") or {}
    scenario_id = str(scenario.get("id") or run_dir.name)
    metrics = record.get("metrics")
    if not isinstance(metrics, dict) or "success" not in metrics:
        raise DistillError(f"{record_path} 缺少 metrics.success")
    if not traj_path.is_file():
        raise DistillError(f"缺少 trajectory.npz：{traj_path}")
    with np.load(traj_path) as npz:
        _require_keys(
            npz,
            (
                "time_s",
                "position_m",
                "velocity_mps",
                "acceleration_mps2",
                "yaw_rad",
                "goal_m",
            ),
            traj_path,
        )
        time_s = np.asarray(npz["time_s"], dtype=np.float64)
        position = np.asarray(npz["position_m"], dtype=np.float64)
        velocity = np.asarray(npz["velocity_mps"], dtype=np.float64)
        acceleration = np.asarray(npz["acceleration_mps2"], dtype=np.float64)
        yaw = np.asarray(npz["yaw_rad"], dtype=np.float64)
        goal = np.asarray(npz["goal_m"], dtype=np.float64)

    if time_s.ndim != 1 or position.ndim != 3 or position.shape[0] != time_s.shape[0] or position.shape[2] != 3:
        raise DistillError(f"{traj_path} 的 position_m 形状无法和 time_s 对齐：{position.shape} / {time_s.shape}")
    n_drones = int(position.shape[1])
    if velocity.shape != position.shape or acceleration.shape != position.shape:
        raise DistillError(f"{traj_path} 的速度或加速度形状与位置不一致")
    if yaw.shape != (time_s.shape[0], n_drones):
        raise DistillError(f"{traj_path} 的 yaw_rad 形状是 {yaw.shape}，应为 {(time_s.shape[0], n_drones)}")
    if goal.shape != (n_drones, 3):
        raise DistillError(f"{traj_path} 的 goal_m 形状是 {goal.shape}")
    declared = scenario.get("n_drones")
    if declared is not None and int(declared) != n_drones:
        raise DistillError(f"scenario.n_drones={declared} 与轨迹机数 {n_drones} 不一致")
    if n_drones != 1:
        raise DistillError(
            f"{scenario_id} 有 {n_drones} 架。本票只转换单机。邻机输入是 D4，不把第 0 架切出来。"
        )
    if time_s.size and np.any(np.diff(time_s) <= 0):
        raise DistillError(f"{traj_path} 的 time_s 不是严格递增")
    if metrics.get("success") is not True:
        return _empty(scenario_id, "success_not_true", cfg.horizon_dt_s, 20.0)

    scene_path = run_dir / "scene.json"
    if not scene_path.is_file():
        raise DistillError(f"成功记录缺少 scene.json，无法现算深度：{scene_path}")
    card = load_card(scene_path)
    if "camera" not in card.card:
        raise DistillError(f"{scene_path} 没有 camera")
    camera_cfg = student_camera(card.card["camera"], cfg)
    # 噪声已是 0。种子不影响深度，只是 DepthCamera 要一个 rng。
    camera = DepthCamera(camera_cfg, card.obstacles, np.random.default_rng(0))
    horizon = cfg.horizon_s
    if time_s.size == 0 or float(time_s[-1] - time_s[0]) + 1e-9 < horizon:
        return _empty(scenario_id, "horizon_longer_than_record", cfg.horizon_dt_s, camera_cfg.max_range_m)
    valid = np.flatnonzero(time_s + horizon <= float(time_s[-1]) + 1e-9)
    if valid.size == 0:
        return _empty(scenario_id, "horizon_longer_than_record", cfg.horizon_dt_s, camera_cfg.max_range_m)
    chosen = valid[_pick(int(valid.size), max_windows)]

    yaw_i = yaw[:, 0]
    if yaw_i.size >= 2 and float(np.max(np.abs(np.diff(yaw_i)))) > np.pi:
        # 相邻样本的原始差超过 π 时，线性插值会抄近路的反方向。包装后的角差看不出来。
        raise DistillError(f"{traj_path} 的偏航在相邻样本之间跳了超过 π，本票的线性插值不能用")

    depths = []
    states = []
    positions = []
    velocities = []
    yaws = []
    times = []
    dt = float(cfg.horizon_dt_s)
    k = int(cfg.horizon_steps)
    goal0 = goal[0]
    for index in chosen:
        i = int(index)
        t_i = float(time_s[i])
        yaw0 = float(yaw_i[i])
        rot = R_yaw(yaw0)
        world_to_body = rot.T
        p0 = position[i, 0]
        state = np.concatenate(
            [
                world_to_body @ velocity[i, 0],
                world_to_body @ acceleration[i, 0],
                world_to_body @ (goal0 - p0),
            ]
        )
        knots = t_i + dt * np.arange(1, k + 1, dtype=np.float64)
        p_w = _interp_rows(time_s, position[:, 0, :], knots)
        v_w = _interp_rows(time_s, velocity[:, 0, :], knots)
        y_w = _interp_rows(time_s, yaw_i, knots)
        p_b = (world_to_body @ (p_w - p0).T).T
        v_b = (world_to_body @ v_w.T).T
        frame = camera.capture(p0, yaw0, t_i)
        depths.append(pack_depth(frame.depth, camera_cfg.max_range_m))
        states.append(state.astype(np.float32))
        positions.append(p_b.astype(np.float32))
        velocities.append(v_b.astype(np.float32))
        yaws.append(wrap_angle(y_w - yaw0).astype(np.float32))
        times.append(t_i)
    return WindowBatch(
        depth=np.stack(depths).astype(np.float32),
        state=np.stack(states).astype(np.float32),
        position_body=np.stack(positions).astype(np.float32),
        velocity_body=np.stack(velocities).astype(np.float32),
        yaw_rel=np.stack(yaws).astype(np.float32),
        time_s=np.asarray(times, dtype=np.float64),
        sample_index=chosen.astype(np.int64),
        scenario_id=scenario_id,
        skipped=None,
        horizon_dt_s=dt,
        max_range_m=float(camera_cfg.max_range_m),
    )


def save_batch(batch: WindowBatch, path: Path | str) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        depth=batch.depth,
        state=batch.state,
        position_body=batch.position_body,
        velocity_body=batch.velocity_body,
        yaw_rel=batch.yaw_rel,
        time_s=batch.time_s,
        sample_index=batch.sample_index,
        meta=np.asarray(json.dumps(batch.meta(), ensure_ascii=False)),
    )
    return path


def load_batch(path: Path | str) -> WindowBatch:
    path = Path(path)
    with np.load(path, allow_pickle=False) as npz:
        meta = json.loads(str(npz["meta"]))
        if meta.get("tensor_schema") != TENSOR_SCHEMA:
            raise DistillError(f"{path} 的 tensor_schema 不是 {TENSOR_SCHEMA}")
        return WindowBatch(
            depth=np.asarray(npz["depth"], dtype=np.float32),
            state=np.asarray(npz["state"], dtype=np.float32),
            position_body=np.asarray(npz["position_body"], dtype=np.float32),
            velocity_body=np.asarray(npz["velocity_body"], dtype=np.float32),
            yaw_rel=np.asarray(npz["yaw_rel"], dtype=np.float32),
            time_s=np.asarray(npz["time_s"], dtype=np.float64),
            sample_index=np.asarray(npz["sample_index"], dtype=np.int64),
            scenario_id=str(meta.get("scenario_id") or ""),
            skipped=meta.get("skipped"),
            horizon_dt_s=float(meta["horizon_dt_s"]),
            max_range_m=float(meta["max_range_m"]),
        )
