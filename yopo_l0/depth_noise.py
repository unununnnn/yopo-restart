"""D435 深度噪声、结构化空洞和延迟队列。默认关。

视差域轴向噪声 ``σ_Z = Z² · s / (fx · B)``，``s = 0.08`` px，``B = 0.05`` m。
``fx`` 用调用方传入的相机内参（仿真相机不是 848×480 时，米制噪声按该焦距放大）。
848×480、HFOV 90° 时 ``fx ≈ 426`` px，2 m 处 ``σ ≈ 15`` mm。

无效像素：模型内部用 0。交给策略之前按 ``invalid`` 处理。

- ``nan``（接线默认）：与 ``sensor.render`` 的无回波相同。网络入口
  ``network_depth`` 再做上游 NS radius-1 inpaint（YOPO ``test_yopo_ros``）。
  安全证据和影子盾仍把这些像素当未知。
- ``inpaint``：4 邻域均值，最多 8 轮，填不上的改为 ``max_range``。
- ``max_range``：无效像素直接填相机最大量程。

噪声、空洞/阴影、延迟三条可以单独开。模拟里三条用不同的随机流，
开空洞不会挪动轴向噪声的随机数。每架飞机、每个回合的视差偏置只抽一次。
延迟是按捕获时间戳的队列：策略读到的是已经送达、捕获时间最新的那一帧。

置信度低、仍按调研表放进默认值的有：飞点概率 ``p_flying=0.3``、
掠射角空洞阈值 75°、相关长度 3 px、丢帧率 0.5%，以及 Σ≈21 ms 里
Orin 额外的约 5 ms。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

# 调研表的推荐默认。随机化范围不在这里抽，除非调用方改字段。
DEF = dict(
    fx=426.0,
    baseline=0.050,
    subpix=0.08,
    corr_px=3.0,
    disp_offset=0.0,
    disp_quant=1 / 32,
    min_z_m=0.195,
    z_max_valid=10.0,
    far_drop_z0=4.0,
    far_drop_z1=8.0,
    p_far_min=0.3,
    lateral_px=1.0,
    p_flying=0.3,
    p_hole=0.01,
    blob_holes=3,
    blob_r=(2, 12),
    p_outlier=0.003,
    grazing_deg=75.0,
    depth_unit=0.001,
)

INVALID_MODES = ("nan", "inpaint", "max_range")
# 本研究和运行器接线用 nan。另外两种留着，方便对照「先填再送给网络」。
WIRED_INVALID = "nan"
INVALID_NOTE = (
    "无效像素为 NaN，与 sensor.render 无回波约定相同。"
    "进入网络时由 network_depth 做上游 NS radius-1 inpaint"
    "（YOPO test_yopo_ros）。安全证据和影子盾仍把 NaN 当未知。"
    "可选 invalid=inpaint（4 邻域均值，最多 8 轮，剩余填 max_range）"
    "或 invalid=max_range（无效像素填相机最大量程）。本开关接线用的是 nan。"
)
INPAINT_ROUNDS = 8

# 曝光中点之后的输送：驱动约 18 ms，再加半个曝光。室内曝光取调研表的约 8 ms。
LATENCY_MEAN_MS = 18.0
LATENCY_JITTER_MS = 3.0
LATENCY_MIN_MS = 10.0
LATENCY_P_SPIKE = 0.005
LATENCY_SPIKE_MS = (20.0, 60.0)
LATENCY_P_DROP = 0.005
LATENCY_EXPOSURE_MS = 8.0


def _flag(block: dict, key: str, default: bool) -> bool:
    if key not in block:
        return default
    value = block[key]
    if value is not True and value is not False:
        raise ValueError(f"task.d435_depth.{key} 必须是布尔值，收到 {value!r}")
    return bool(value)


def _finite(block: dict, key: str, default: float, *, lower: float | None = 0.0) -> float:
    if key not in block or block[key] is None:
        return float(default)
    raw = block[key]
    if isinstance(raw, bool):
        raise ValueError(f"task.d435_depth.{key} 必须是实数，收到 {raw!r}")
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"task.d435_depth.{key} 必须是实数，收到 {raw!r}") from exc
    if not np.isfinite(value) or (lower is not None and value < lower):
        raise ValueError(f"task.d435_depth.{key} 必须 ≥ {lower} 且有限，收到 {raw!r}")
    return value


@dataclass(frozen=True)
class D435Config:
    """``task.d435_depth``。``enabled`` 缺省 false。三条子开关在总开关打开时缺省 true。"""

    enabled: bool = False
    noise: bool = True
    holes: bool = True
    latency: bool = True
    subpix: float = DEF["subpix"]
    latency_ms: float | None = None
    invalid: str = WIRED_INVALID
    p_flying: float = DEF["p_flying"]
    p_hole: float = DEF["p_hole"]
    grazing_deg: float = DEF["grazing_deg"]
    p_drop: float = LATENCY_P_DROP

    @property
    def active(self) -> bool:
        return bool(self.enabled) and (self.noise or self.holes or self.latency)

    def public_dict(self) -> dict:
        return {
            "enabled": True,
            "noise": self.noise,
            "holes": self.holes,
            "latency": self.latency,
            "subpix": self.subpix,
            "latency_ms": self.latency_ms,
            "invalid_fill": self.invalid,
            "invalid_fill_note": INVALID_NOTE,
            "p_flying": self.p_flying,
            "grazing_deg": self.grazing_deg,
            "p_drop": self.p_drop,
        }


def d435_from_task(task: dict | None) -> D435Config:
    """缺省、false、空任务都是关。``true`` 表示三条子开关按默认全开。"""
    if not isinstance(task, dict):
        return D435Config()
    return d435_from_value(task.get("d435_depth", False))


def d435_from_value(raw) -> D435Config:
    if raw is False or raw is None:
        return D435Config(enabled=False)
    if raw is True:
        return D435Config(enabled=True)
    if not isinstance(raw, dict):
        raise ValueError(f"task.d435_depth 必须是布尔或对象，收到 {raw!r}")
    enabled = _flag(raw, "enabled", False)
    invalid = raw.get("invalid", WIRED_INVALID)
    if invalid not in INVALID_MODES:
        raise ValueError(f"task.d435_depth.invalid 必须是 {INVALID_MODES} 之一，收到 {invalid!r}")
    latency_ms = None
    if "latency_ms" in raw and raw["latency_ms"] is not None:
        latency_ms = _finite(raw, "latency_ms", 0.0, lower=0.0)
    p_drop = _finite(raw, "p_drop", LATENCY_P_DROP, lower=0.0)
    if p_drop > 1.0:
        raise ValueError(f"task.d435_depth.p_drop 不能大于 1，收到 {raw.get('p_drop')!r}")
    grazing = _finite(raw, "grazing_deg", DEF["grazing_deg"], lower=0.0)
    if grazing > 90.0:
        raise ValueError(f"task.d435_depth.grazing_deg 不能大于 90，收到 {raw.get('grazing_deg')!r}")
    return D435Config(
        enabled=enabled,
        noise=_flag(raw, "noise", True),
        holes=_flag(raw, "holes", True),
        latency=_flag(raw, "latency", True),
        subpix=_finite(raw, "subpix", DEF["subpix"], lower=0.0),
        latency_ms=latency_ms,
        invalid=str(invalid),
        p_flying=_finite(raw, "p_flying", DEF["p_flying"], lower=0.0),
        p_hole=_finite(raw, "p_hole", DEF["p_hole"], lower=0.0),
        grazing_deg=grazing,
        p_drop=p_drop,
    )


def _box_same(values: np.ndarray, radius: int, axis: int) -> np.ndarray:
    """零填充的均匀盒式滤波，长度不变。与 ``np.convolve(..., mode='same')`` 一致。"""
    if radius <= 0:
        return values
    moved = np.moveaxis(values, axis, -1)
    width = moved.shape[-1]
    padded = np.pad(moved, [(0, 0)] * (moved.ndim - 1) + [(radius, radius)])
    cumulative = np.cumsum(padded, axis=-1)
    kernel = 2 * radius + 1
    end = cumulative[..., kernel - 1 : kernel - 1 + width]
    start = np.empty_like(end)
    start[..., 0] = 0
    if width > 1:
        start[..., 1:] = cumulative[..., : width - 1]
    return np.moveaxis((end - start) / kernel, -1, axis)


def _corr_noise(shape, sigma: float, corr_px: float, rng: np.random.Generator) -> np.ndarray:
    field = rng.standard_normal(shape)
    if corr_px <= 0.5:
        return field * sigma
    radius = int(max(1, round(corr_px)))
    field = _box_same(field, radius, 1)
    field = _box_same(field, radius, 0)
    return field / (field.std() + 1e-9) * sigma


def incidence_cos(z_m: np.ndarray, fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    """由真值深度估计入射角余弦。未知或退化的像素记 1（不额外挖洞）。

    掠射角系数没有 D435 的公开实测，置信度低。平壁正对相机时余弦接近 1。
    """
    z = np.asarray(z_m, dtype=np.float64)
    finite = np.isfinite(z) & (z > 0)
    height, width = z.shape
    uu, vv = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
    depth = np.where(finite, z, np.nan)
    points = np.stack([(uu - cx) / fx * depth, (vv - cy) / fy * depth, depth], axis=-1)
    du = np.zeros_like(points)
    dv = np.zeros_like(points)
    if width >= 3:
        du[:, 1:-1] = points[:, 2:] - points[:, :-2]
    if height >= 3:
        dv[1:-1] = points[2:] - points[:-2]
    normal = np.cross(dv, du)
    norm = np.linalg.norm(normal, axis=-1, keepdims=True)
    normal = normal / np.maximum(norm, 1e-12)
    ray_norm = np.linalg.norm(points, axis=-1, keepdims=True)
    ray = points / np.maximum(ray_norm, 1e-12)
    cosine = np.abs(np.sum(normal * ray, axis=-1))
    # 差分在图像边上是 0，法向长度也是 0。那不是掠射，不能挖洞。
    unknown = (~finite) | (norm[..., 0] < 1e-6) | ~np.isfinite(cosine)
    cosine[unknown] = 1.0
    return np.clip(cosine, 0.0, 1.0).astype(np.float32)


def d435_depth(
    z_gt: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    rng: np.random.Generator,
    *,
    holes_rng: np.random.Generator | None = None,
    params: dict | None = None,
    noise: bool = True,
    holes: bool = True,
    normals_cos: np.ndarray | None = None,
) -> np.ndarray:
    """真值深度（米）→ 加噪深度。0 表示无效。``intrinsics`` 是 fx, fy, cx, cy。

    ``rng`` 只服务轴向/横向/飞点。``holes_rng`` 缺省时空洞接着用同一个生成器，
    和调研脚本的自测同一条随机流。模拟器把两个生成器分开。
    """
    spec = dict(DEF)
    if params:
        spec.update(params)
    hole_rng = rng if holes_rng is None else holes_rng
    height, width = z_gt.shape
    fx = float(intrinsics[0])
    baseline = float(spec["baseline"])
    fb = fx * baseline
    z = np.where(np.isfinite(z_gt) & (z_gt > 0), np.asarray(z_gt, dtype=np.float64), np.inf)
    disp = fb / z
    disp_j = disp
    measured = disp
    if noise:
        shift = rng.normal(0.0, float(spec["lateral_px"]), (height, width))
        columns = np.clip(np.round(np.arange(width)[None, :] + shift), 0, width - 1).astype(int)
        disp_j = disp[np.arange(height)[:, None], columns]
        jump = np.abs(np.diff(disp_j, axis=1, prepend=disp_j[:, :1]))
        edge = jump > 0.08 * np.maximum(disp_j, 1e-6) + 1.0
        edge = edge | np.roll(edge, 1, axis=1)
        fly = edge & (rng.random((height, width)) < float(spec["p_flying"]))
        left = np.roll(disp_j, 1, axis=1)
        mix = rng.random((height, width))
        disp_j = np.where(fly, mix * disp_j + (1.0 - mix) * left, disp_j)
        measured = disp_j + _corr_noise((height, width), float(spec["subpix"]), float(spec["corr_px"]), rng)
        measured = measured + float(spec["disp_offset"])
        quant = float(spec["disp_quant"])
        measured = np.round(measured / quant) * quant
    valid = (measured > 0) & np.isfinite(disp_j) & (disp_j > 0)
    if holes:
        max_disp = fb / float(spec["min_z_m"])
        valid &= measured < max_disp
        band = np.arange(width)[None, :] < (fb / np.where(np.isfinite(z), z, 1e9))
        valid &= ~band
        right = np.arange(width)[None, :] - disp_j
        run_min = np.minimum.accumulate(right[:, ::-1], axis=1)[:, ::-1]
        occluded = right > np.roll(run_min, -1, axis=1) + 0.5
        occluded[:, -1] = False
        valid &= ~occluded
    depth = np.where(valid, fb / np.maximum(measured, 1e-6), 0.0)
    if holes:
        near = float(spec["far_drop_z0"])
        far = float(spec["far_drop_z1"])
        keep_prob = np.interp(depth, [near, far], [1.0, float(spec["p_far_min"])])
        depth = np.where((hole_rng.random((height, width)) < keep_prob) & (depth < float(spec["z_max_valid"])), depth, 0.0)
        if normals_cos is not None:
            angle = np.degrees(np.arccos(np.clip(normals_cos, 0.0, 1.0)))
            span = 90.0 - float(spec["grazing_deg"])
            probability = np.clip((angle - float(spec["grazing_deg"])) / span, 0.0, 1.0) if span > 0 else (angle >= 90.0)
            depth[hole_rng.random((height, width)) < probability] = 0.0
        depth[hole_rng.random((height, width)) < float(spec["p_hole"])] = 0.0
        yy, xx = np.mgrid[0:height, 0:width]
        for _ in range(int(hole_rng.poisson(float(spec["blob_holes"])))):
            cy = int(hole_rng.integers(0, height))
            cx = int(hole_rng.integers(0, width))
            radius = int(hole_rng.integers(int(spec["blob_r"][0]), int(spec["blob_r"][1])))
            depth[(yy - cy) ** 2 + (xx - cx) ** 2 < radius * radius] = 0.0
        outlier = hole_rng.random((height, width)) < float(spec["p_outlier"])
        count = int(outlier.sum())
        if count:
            depth[outlier] = hole_rng.uniform(0.2, 1.5, count)
    if noise:
        unit = float(spec["depth_unit"])
        depth = np.round(depth / unit) * unit
    return depth.astype(np.float32)


def fill_invalid(depth: np.ndarray, mode: str, max_range_m: float) -> np.ndarray:
    """0 和非有限值按 ``mode`` 处理。``inpaint`` 填不上的像素改为 ``max_range_m``。"""
    if mode not in INVALID_MODES:
        raise ValueError(f"invalid 必须是 {INVALID_MODES} 之一，收到 {mode!r}")
    values = np.array(depth, dtype=np.float32, copy=True)
    invalid = (~np.isfinite(values)) | (values <= 0)
    if mode == "nan":
        values[invalid] = np.nan
        return values
    if mode == "max_range":
        values[invalid] = np.float32(max_range_m)
        return values
    filled = values.copy()
    filled[invalid] = np.nan
    for _ in range(INPAINT_ROUNDS):
        padded = np.pad(filled, 1, mode="constant", constant_values=np.nan)
        total = np.zeros(filled.shape, np.float32)
        count = np.zeros(filled.shape, np.float32)
        for src_y, src_x in ((0, 1), (1, 0), (1, 2), (2, 1)):
            neighbor = padded[src_y : src_y + filled.shape[0], src_x : src_x + filled.shape[1]]
            ok = np.isfinite(neighbor)
            total += np.where(ok, neighbor, 0.0)
            count += ok
        holes = ~np.isfinite(filled)
        writable = holes & (count > 0)
        if not np.any(writable):
            break
        filled[writable] = total[writable] / count[writable]
    filled[~np.isfinite(filled)] = np.float32(max_range_m)
    return filled.astype(np.float32)


def finalize_depth(depth: np.ndarray, mode: str, min_range_m: float, max_range_m: float) -> np.ndarray:
    """先按无效约定填充，再套相机量程。非 nan 模式把量程外的像素填成最大量程。"""
    values = fill_invalid(depth, mode, max_range_m)
    with np.errstate(invalid="ignore"):
        outside = (~np.isfinite(values)) | (values < min_range_m) | (values > max_range_m)
    if mode == "nan":
        values[outside] = np.nan
    else:
        values[outside] = np.float32(max_range_m)
    return values.astype(np.float32)


def apply_d435_depth(
    z_gt: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    rng: np.random.Generator,
    *,
    params: dict | None = None,
    noise: bool = True,
    holes: bool = True,
    invalid: str = WIRED_INVALID,
    min_range_m: float = 0.2,
    max_range_m: float = 20.0,
    normals_cos: np.ndarray | None = None,
    holes_rng: np.random.Generator | None = None,
) -> np.ndarray:
    """真值深度 + 内参 + rng → 加噪深度。无效像素按 ``invalid``。"""
    raw = d435_depth(
        z_gt,
        intrinsics,
        rng,
        holes_rng=holes_rng,
        params=params,
        noise=noise,
        holes=holes,
        normals_cos=normals_cos,
    )
    return finalize_depth(raw, invalid, min_range_m, max_range_m)


class LatencyQueue:
    """捕获时刻 ``t_cap`` 的帧在 ``t_cap + delay`` 送达。读取方拿已送达里最新的一帧。"""

    def __init__(
        self,
        rng: np.random.Generator,
        mean_ms: float = LATENCY_MEAN_MS,
        jitter_ms: float = LATENCY_JITTER_MS,
        min_ms: float = LATENCY_MIN_MS,
        p_spike: float = LATENCY_P_SPIKE,
        spike_ms: tuple[float, float] = LATENCY_SPIKE_MS,
        p_drop: float = LATENCY_P_DROP,
    ):
        self.q: deque = deque()
        self.rng = rng
        self.mean = float(mean_ms)
        self.jitter = float(jitter_ms)
        self.min_ms = float(min_ms)
        self.p_spike = float(p_spike)
        self.spike_ms = (float(spike_ms[0]), float(spike_ms[1]))
        self.p_drop = float(p_drop)
        self.latest = None

    def push(self, t_cap: float, frame, exposure_ms: float = LATENCY_EXPOSURE_MS) -> None:
        if self.rng.random() < self.p_drop:
            return
        delay = max(self.min_ms, float(self.rng.normal(self.mean, self.jitter))) + 0.5 * float(exposure_ms)
        if self.rng.random() < self.p_spike:
            delay += float(self.rng.uniform(*self.spike_ms))
        self.q.append((t_cap + delay / 1000.0, t_cap, frame))

    def pop(self, t_now: float):
        ready = [item for item in self.q if item[0] <= t_now]
        self.q = deque(item for item in self.q if item[0] > t_now)
        if ready:
            # 同一次取出的帧里留捕获时间最新的。晚到的旧帧不能把已经在用的更新帧换回去。
            candidate = max(ready, key=lambda item: item[1])
            if self.latest is None or candidate[1] >= self.latest[1]:
                self.latest = candidate
        return self.latest


def _classic_depth(best: np.ndarray, cfg, rng: np.random.Generator) -> np.ndarray:
    """与 ``sensor.render`` 在开关关闭时的三行相同，延迟臂仍消耗原来的高斯随机数。"""
    noise = rng.normal(0.0, cfg.noise_std_m, best.shape) if cfg.noise_std_m > 0 else 0.0
    depth = best + noise
    depth[~np.isfinite(best) | (best > cfg.max_range_m) | (best < cfg.min_range_m)] = np.nan
    return depth.astype(np.float32)


class D435Rig:
    """一架相机、一个回合。三条随机流互不干扰。视差偏置在构造时抽一次。"""

    def __init__(self, cfg: D435Config, noise_rng: np.random.Generator, holes_rng: np.random.Generator, latency_rng: np.random.Generator):
        self.cfg = cfg
        self.noise_rng = noise_rng
        self.holes_rng = holes_rng
        self.params = dict(DEF)
        self.params["subpix"] = cfg.subpix
        self.params["p_flying"] = cfg.p_flying
        self.params["p_hole"] = cfg.p_hole
        self.params["grazing_deg"] = cfg.grazing_deg
        if cfg.noise:
            self.params["disp_offset"] = float(np.clip(noise_rng.normal(0.0, 0.1), -0.25, 0.25))
        else:
            self.params["disp_offset"] = 0.0
        self.exposure_ms = 0.0 if cfg.latency_ms is not None else LATENCY_EXPOSURE_MS
        self.queue = None
        if cfg.latency:
            if cfg.latency_ms is None:
                self.queue = LatencyQueue(latency_rng, p_drop=cfg.p_drop)
            else:
                fixed = float(cfg.latency_ms)
                self.queue = LatencyQueue(
                    latency_rng,
                    mean_ms=fixed,
                    jitter_ms=0.0,
                    min_ms=fixed,
                    p_spike=0.0,
                    spike_ms=(0.0, 0.0),
                    p_drop=cfg.p_drop,
                )

    @classmethod
    def from_seed(cls, cfg: D435Config, seed) -> "D435Rig | None":
        """``seed`` 是这架相机的 ``SeedSequence``。关着返回 None，不抽随机数。"""
        if not cfg.active:
            return None
        noise_ss, holes_ss, latency_ss = seed.spawn(3)
        return cls(
            cfg,
            np.random.default_rng(noise_ss),
            np.random.default_rng(holes_ss),
            np.random.default_rng(latency_ss),
        )

    def finish(self, best: np.ndarray, cfg, rng: np.random.Generator, stamp: float) -> tuple[np.ndarray, float]:
        """返回 ``(depth, stamp)``。延迟打开时 stamp 是更早那一帧的捕获时刻。"""
        if self.cfg.noise or self.cfg.holes:
            ground = np.where(np.isfinite(best) & (best > 0), best, np.inf)
            normals = incidence_cos(ground, *cfg.intrinsics) if self.cfg.holes else None
            raw = d435_depth(
                ground,
                cfg.intrinsics,
                self.noise_rng,
                holes_rng=self.holes_rng,
                params=self.params,
                noise=self.cfg.noise,
                holes=self.cfg.holes,
                normals_cos=normals,
            )
            depth = finalize_depth(raw, self.cfg.invalid, cfg.min_range_m, cfg.max_range_m)
            out_stamp = float(stamp)
        else:
            depth = _classic_depth(best, cfg, rng)
            out_stamp = float(stamp)
        if self.queue is None:
            return depth, out_stamp
        self.queue.push(out_stamp, depth, exposure_ms=self.exposure_ms)
        delivered = self.queue.pop(float(stamp))
        if delivered is None:
            return np.full(depth.shape, np.nan, np.float32), float(stamp)
        _arrival, captured, frame = delivered
        return np.array(frame, dtype=np.float32, copy=True), float(captured)
