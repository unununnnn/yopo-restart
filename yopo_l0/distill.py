"""把一条 EGO-Swarm 教师记录转成学生的窗口张量。

读 ``yopo-teacher-record/1``：``trajectory.npz`` 的位姿，``record.json`` 的成败，
``scene.json`` 现算深度。不导入 ``scripts/teacher``。不把广播 B 样条系数喂进网络。

多机记录按每一架自己的视点出窗口。邻机特征是同一拍世界系位置、速度变到机体系后，
取最近 K 架的相对位置和相对速度，缺位补零加掩码。这和总线上能广播的量同类。
本转换按记录里的同步里程计取值，按零时延、无丢包来假设，不模拟总线故障。

深度按场景卡的视场角光线投射，空间尺寸用学生的 24×40，不是卡上的 96×160。
噪声固定为 0。无回波写成两个通道的 0。邻机不画进深度，只走邻机通道。
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
    NEIGHBOR_FEAT_DIM,
    NEIGHBOR_K,
    STATE_DIM,
    StudentConfig,
    pack_neighbor_features,
)

TEACHER_SCHEMA = "yopo-teacher-record/1"
TENSOR_SCHEMA = "yopo-student-window/2"


class DistillError(ValueError):
    """记录缺字段、模式不对，或者轨迹对不齐。"""


@dataclass(frozen=True)
class WindowBatch:
    depth: np.ndarray  # (N, 2, H, W) float32
    state: np.ndarray  # (N, 9) float32
    position_body: np.ndarray  # (N, Hzn, 3) float32
    velocity_body: np.ndarray  # (N, Hzn, 3) float32
    yaw_rel: np.ndarray  # (N, Hzn) float32
    neighbor_rel: np.ndarray  # (N, K, 6) float32
    neighbor_mask: np.ndarray  # (N, K) float32
    time_s: np.ndarray  # (N,) float64
    sample_index: np.ndarray  # (N,) int64
    ego_index: np.ndarray  # (N,) int64
    scenario_id: str
    skipped: str | None
    horizon_dt_s: float
    max_range_m: float
    n_neighbors: int
    # 0 表示不平移。非 0 时位置和目标的世界系 z 加了这么多米，深度按 L0 相机重渲。
    altitude_shift_m: float = 0.0

    @property
    def n(self) -> int:
        return int(self.state.shape[0])

    def meta(self) -> dict:
        out = {
            "tensor_schema": TENSOR_SCHEMA,
            "teacher_schema": TEACHER_SCHEMA,
            "scenario_id": self.scenario_id,
            "n_windows": self.n,
            "skipped": self.skipped,
            "horizon_steps": int(self.position_body.shape[1]) if self.position_body.ndim == 3 else HORIZON_STEPS,
            "horizon_dt_s": self.horizon_dt_s,
            "depth_hw": [DEPTH_HEIGHT, DEPTH_WIDTH],
            "state_dim": STATE_DIM,
            "n_neighbors": self.n_neighbors,
            "neighbor_feat_dim": NEIGHBOR_FEAT_DIM,
            "max_range_m": self.max_range_m,
            "heads": ["position_body", "velocity_body", "yaw_rel"],
            "depth_note": (
                "按场景卡视场角现算，分辨率是学生的 24×40，不是教师卡的 96×160。"
                "通道 0 是深度/max_range，无回波为 0。通道 1 是有效掩码。"
                "噪声固定为 0。邻机不画进深度。"
            ),
            "state_note": "机体坐标系：速度 3、加速度 3、目标相对位置 3。",
            "neighbor_note": (
                "同一拍世界系位置、速度变到机体系：相对位置 3、相对速度 3。"
                "按距离取最近 K 架，缺位补零，掩码 1 表示有邻机。"
                "按同步里程计、零时延无丢包取值（假设）。不读 B 样条系数。"
            ),
            "bsplines": "未把系数写入张量",
        }
        if self.altitude_shift_m != 0.0:
            out["altitude_shift_m"] = float(self.altitude_shift_m)
            out["altitude_note"] = (
                "位置和目标的世界系 z 平移了这么多米，再按 L0 相机重渲深度。"
                "速度、加速度、偏航不改。障碍和地面不平移。"
            )
        return out


def _empty(
    scenario_id: str,
    skipped: str,
    dt: float,
    max_range: float,
    *,
    n_neighbors: int = NEIGHBOR_K,
) -> WindowBatch:
    k = HORIZON_STEPS
    nk = int(n_neighbors)
    return WindowBatch(
        depth=np.zeros((0, 2, DEPTH_HEIGHT, DEPTH_WIDTH), dtype=np.float32),
        state=np.zeros((0, STATE_DIM), dtype=np.float32),
        position_body=np.zeros((0, k, 3), dtype=np.float32),
        velocity_body=np.zeros((0, k, 3), dtype=np.float32),
        yaw_rel=np.zeros((0, k), dtype=np.float32),
        neighbor_rel=np.zeros((0, nk, NEIGHBOR_FEAT_DIM), dtype=np.float32),
        neighbor_mask=np.zeros((0, nk), dtype=np.float32),
        time_s=np.zeros((0,), dtype=np.float64),
        sample_index=np.zeros((0,), dtype=np.int64),
        ego_index=np.zeros((0,), dtype=np.int64),
        scenario_id=scenario_id,
        skipped=skipped,
        horizon_dt_s=dt,
        max_range_m=max_range,
        n_neighbors=nk,
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
    altitude_shift_m: float = 0.0,
) -> WindowBatch:
    """成功记录 → 窗口。每一架都是一个视点。``metrics.success`` 不是 true 时返回空批次，不渲染。

    ``altitude_shift_m`` 默认 0。为 0 时不改数组，输出与不传这个参数逐位相同。
    非 0 时把位置和目标的世界系 z 加上这个数，深度按平移后的位姿用 L0 相机重渲。
    速度、加速度、偏航、障碍和地面都不改。机体坐标里的相对量因此不变，变的是深度。
    """
    cfg = cfg or StudentConfig()
    shift = float(altitude_shift_m)
    if not np.isfinite(shift):
        raise DistillError(f"altitude_shift_m 必须是有限数，收到 {altitude_shift_m!r}")
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
    if shift != 0.0:
        position = np.array(position, copy=True)
        goal = np.array(goal, copy=True)
        position[:, :, 2] += shift
        goal[:, 2] += shift
    declared = scenario.get("n_drones")
    if declared is not None and int(declared) != n_drones:
        raise DistillError(f"scenario.n_drones={declared} 与轨迹机数 {n_drones} 不一致")
    if n_drones < 1:
        raise DistillError(f"{traj_path} 的机数是 {n_drones}")
    if time_s.size and np.any(np.diff(time_s) <= 0):
        raise DistillError(f"{traj_path} 的 time_s 不是严格递增")
    if metrics.get("success") is not True:
        return _empty(scenario_id, "success_not_true", cfg.horizon_dt_s, 20.0, n_neighbors=cfg.n_neighbors)

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
        return _empty(
            scenario_id, "horizon_longer_than_record", cfg.horizon_dt_s, camera_cfg.max_range_m,
            n_neighbors=cfg.n_neighbors,
        )
    valid = np.flatnonzero(time_s + horizon <= float(time_s[-1]) + 1e-9)
    if valid.size == 0:
        return _empty(
            scenario_id, "horizon_longer_than_record", cfg.horizon_dt_s, camera_cfg.max_range_m,
            n_neighbors=cfg.n_neighbors,
        )
    pairs = np.array([(int(i), d) for i in valid for d in range(n_drones)], dtype=np.int64)
    chosen = pairs[_pick(int(pairs.shape[0]), max_windows)]

    for d in range(n_drones):
        yaw_d = yaw[:, d]
        if yaw_d.size >= 2 and float(np.max(np.abs(np.diff(yaw_d)))) > np.pi:
            # 相邻样本的原始差超过 π 时，线性插值会抄近路的反方向。包装后的角差看不出来。
            raise DistillError(
                f"{traj_path} 第 {d} 架的偏航在相邻样本之间跳了超过 π，本票的线性插值不能用"
            )

    depths = []
    states = []
    positions = []
    velocities = []
    yaws = []
    neighbors = []
    neighbor_masks = []
    times = []
    sample_ids = []
    ego_ids = []
    dt = float(cfg.horizon_dt_s)
    k = int(cfg.horizon_steps)
    n_k = int(cfg.n_neighbors)
    for pair in chosen:
        i = int(pair[0])
        d = int(pair[1])
        t_i = float(time_s[i])
        yaw0 = float(yaw[i, d])
        rot = R_yaw(yaw0)
        world_to_body = rot.T
        p0 = position[i, d]
        state = np.concatenate(
            [
                world_to_body @ velocity[i, d],
                world_to_body @ acceleration[i, d],
                world_to_body @ (goal[d] - p0),
            ]
        )
        knots = t_i + dt * np.arange(1, k + 1, dtype=np.float64)
        p_w = _interp_rows(time_s, position[:, d, :], knots)
        v_w = _interp_rows(time_s, velocity[:, d, :], knots)
        y_w = _interp_rows(time_s, yaw[:, d], knots)
        p_b = (world_to_body @ (p_w - p0).T).T
        v_b = (world_to_body @ v_w.T).T
        others = [j for j in range(n_drones) if j != d]
        if others:
            feat, mask = pack_neighbor_features(
                p0,
                velocity[i, d],
                yaw0,
                position[i, others, :],
                velocity[i, others, :],
                k=n_k,
            )
        else:
            feat, mask = pack_neighbor_features(
                p0,
                velocity[i, d],
                yaw0,
                np.zeros((0, 3)),
                np.zeros((0, 3)),
                k=n_k,
            )
        frame = camera.capture(p0, yaw0, t_i)
        depths.append(pack_depth(frame.depth, camera_cfg.max_range_m))
        states.append(state.astype(np.float32))
        positions.append(p_b.astype(np.float32))
        velocities.append(v_b.astype(np.float32))
        yaws.append(wrap_angle(y_w - yaw0).astype(np.float32))
        neighbors.append(feat)
        neighbor_masks.append(mask)
        times.append(t_i)
        sample_ids.append(i)
        ego_ids.append(d)
    return WindowBatch(
        depth=np.stack(depths).astype(np.float32),
        state=np.stack(states).astype(np.float32),
        position_body=np.stack(positions).astype(np.float32),
        velocity_body=np.stack(velocities).astype(np.float32),
        yaw_rel=np.stack(yaws).astype(np.float32),
        neighbor_rel=np.stack(neighbors).astype(np.float32),
        neighbor_mask=np.stack(neighbor_masks).astype(np.float32),
        time_s=np.asarray(times, dtype=np.float64),
        sample_index=np.asarray(sample_ids, dtype=np.int64),
        ego_index=np.asarray(ego_ids, dtype=np.int64),
        scenario_id=scenario_id,
        skipped=None,
        horizon_dt_s=dt,
        max_range_m=float(camera_cfg.max_range_m),
        n_neighbors=n_k,
        altitude_shift_m=shift,
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
        neighbor_rel=batch.neighbor_rel,
        neighbor_mask=batch.neighbor_mask,
        time_s=batch.time_s,
        sample_index=batch.sample_index,
        ego_index=batch.ego_index,
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
            neighbor_rel=np.asarray(npz["neighbor_rel"], dtype=np.float32),
            neighbor_mask=np.asarray(npz["neighbor_mask"], dtype=np.float32),
            time_s=np.asarray(npz["time_s"], dtype=np.float64),
            sample_index=np.asarray(npz["sample_index"], dtype=np.int64),
            ego_index=np.asarray(npz["ego_index"], dtype=np.int64),
            scenario_id=str(meta.get("scenario_id") or ""),
            skipped=meta.get("skipped"),
            horizon_dt_s=float(meta["horizon_dt_s"]),
            max_range_m=float(meta["max_range_m"]),
            n_neighbors=int(meta.get("n_neighbors", NEIGHBOR_K)),
            altitude_shift_m=float(meta.get("altitude_shift_m", 0.0)),
        )


HOLD_SEED_MOD = 5


def parse_run_id(run_id: str) -> dict:
    """解析 d000-n3-headon-s12 这类教师场次编号。"""
    import re

    m = re.fullmatch(r"d(\d+)-n(\d+)-(headon|parallel)(?:-z(\d+))?-s(\d+)", str(run_id))
    if m is None:
        raise DistillError(f"无法解析场次编号：{run_id}")
    density_code = m.group(1)
    density = int(density_code) / 100.0
    info = {
        "id": str(run_id),
        "density_code": density_code,
        "density_per_m2": density,
        "n_drones": int(m.group(2)),
        "pattern": m.group(3),
        "seed": int(m.group(5)),
        "scene_key": f"d{density_code}-n{m.group(2)}-{m.group(3)}",
    }
    if m.group(4) is not None:
        info["flight_z_m"] = int(m.group(4)) / 100.0
    return info


def is_holdout_seed(seed: int, *, mod: int = HOLD_SEED_MOD) -> bool:
    """按种子取模分留出，同一场次不会同时进训练集和留出集。"""
    if int(mod) < 2:
        raise DistillError(f"留出取模至少为 2，收到 {mod}")
    return int(seed) % int(mod) == 0


def split_runs_by_seed(run_ids, *, mod: int = HOLD_SEED_MOD) -> tuple[list[str], list[str]]:
    """按场景键分组、再按种子划训练/留出。返回 (train_ids, hold_ids)。"""
    train: list[str] = []
    hold: list[str] = []
    for raw in run_ids:
        info = parse_run_id(raw)
        if is_holdout_seed(info["seed"], mod=mod):
            hold.append(info["id"])
        else:
            train.append(info["id"])
    return train, hold
