"""D435 深度噪声、结构化空洞和延迟队列。默认关。

视差域轴向噪声 ``σ_Z = Z² · s / (fx · B)``，``s = 0.08`` px，``B = 0.05`` m。
``profile`` 缺省 ``scene``：``fx`` 用调用方传入的相机内参。仿真相机不是
848×480 时，米制噪声和按像素计的空洞（块状洞、相关长度）按该焦距放大。
``profile=native`` 用原生 D435 的角分辨率（848×480、``fx ≈ 426`` px，2 m 处
``σ ≈ 15`` mm）：把真值最近邻抬到这个网格，在视差域加噪并挖空洞，再按视差
平均重采样回原来的分辨率。缺省 ``scene`` 不走这条路径。

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

import functools
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
# gt 不进 fill_invalid：它要用同一像素的真值，不能只看深度图。
FILL_MODES = INVALID_MODES + ("gt",)
PROFILES = ("scene", "native")
# 原生 D435 深度：848×480，fx=fy≈426。像素角 1/426 rad。profile=native 用这个角分辨率。
NATIVE_FX = 426.0
NATIVE_WIDTH = 848
NATIVE_HEIGHT = 480
# 重采样时，一个网络像素里有效原生像素不足这一比例就仍算空洞。
NATIVE_VALID_FRACTION = 0.5
# 本研究和运行器接线用 nan。另外两种留着，方便对照「先填再送给网络」。
WIRED_INVALID = "nan"
INVALID_NOTE = (
    "无效像素为 NaN，与 sensor.render 无回波约定相同。"
    "进入网络时由 network_depth 做上游 NS radius-1 inpaint"
    "（YOPO test_yopo_ros）。安全证据和影子盾仍把 NaN 当未知。"
    "可选 invalid=inpaint（4 邻域均值，最多 8 轮，剩余填 max_range）"
    "或 invalid=max_range（无效像素填相机最大量程）。本开关接线用的是 nan。"
)
GT_INVALID_NOTE = (
    "空洞像素用这一帧的真值深度填上，再交给策略。"
    "这些像素是有限深度，network_depth 不会把它们当洞做 NS inpaint。"
    "真值本身无效的像素仍是 NaN。假近点不在这次填补里。这是对照，不是 D435 的输出。"
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
    profile: str = "scene"
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
            "profile": self.profile,
            "invalid_fill": self.invalid,
            "invalid_fill_note": GT_INVALID_NOTE if self.invalid == "gt" else INVALID_NOTE,
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
    if invalid not in FILL_MODES:
        raise ValueError(f"task.d435_depth.invalid 必须是 {FILL_MODES} 之一，收到 {invalid!r}")
    if "profile" not in raw or raw["profile"] is None:
        profile = "scene"
    else:
        profile = raw["profile"]
        if profile not in PROFILES:
            raise ValueError(f"task.d435_depth.profile 必须是 {PROFILES} 之一，收到 {profile!r}")
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
        profile=str(profile),
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


@functools.lru_cache(maxsize=8)
def _ray_scales(height: int, width: int, fx_u: int, fy_u: int, cx_u: int, cy_u: int):
    """像素射线的 x/z、y/z。只读，同一相机会反复用。"""
    fx = fx_u / 1_000_000.0
    fy = fy_u / 1_000_000.0
    cx = cx_u / 1_000_000.0
    cy = cy_u / 1_000_000.0
    horizontal = np.ascontiguousarray((np.arange(width) + 0.5 - cx) / fx)
    vertical = np.ascontiguousarray((np.arange(height) + 0.5 - cy) / fy)
    horizontal.setflags(write=False)
    vertical.setflags(write=False)
    return horizontal, vertical


def incidence_cos(z_m: np.ndarray, fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    """由真值深度估计入射角余弦。未知或退化的像素记 1（不额外挖洞）。

    掠射角系数没有 D435 的公开实测，置信度低。平壁正对相机时余弦接近 1。
    手写叉乘和模长，结果与按点堆叠再 ``np.cross`` 相同，原生分辨率上少做几次大数组拷贝。
    """
    z = np.asarray(z_m, dtype=np.float64)
    finite = np.isfinite(z) & (z > 0)
    height, width = z.shape
    horizontal, vertical = _ray_scales(
        height,
        width,
        int(round(float(fx) * 1_000_000.0)),
        int(round(float(fy) * 1_000_000.0)),
        int(round(float(cx) * 1_000_000.0)),
        int(round(float(cy) * 1_000_000.0)),
    )
    depth = np.where(finite, z, np.nan)
    x = horizontal * depth
    y = vertical[:, None] * depth
    dx = np.zeros_like(depth)
    dy = np.zeros_like(depth)
    dz = np.zeros_like(depth)
    ex = np.zeros_like(depth)
    ey = np.zeros_like(depth)
    ez = np.zeros_like(depth)
    if width >= 3:
        dx[:, 1:-1] = x[:, 2:] - x[:, :-2]
        dy[:, 1:-1] = y[:, 2:] - y[:, :-2]
        dz[:, 1:-1] = depth[:, 2:] - depth[:, :-2]
    if height >= 3:
        ex[1:-1] = x[2:] - x[:-2]
        ey[1:-1] = y[2:] - y[:-2]
        ez[1:-1] = depth[2:] - depth[:-2]
    # normal = dv × du。边上差分是 0，法向长度也是 0，那不是掠射。
    nx = ey * dz - ez * dy
    ny = ez * dx - ex * dz
    nz = ex * dy - ey * dx
    norm = np.sqrt(nx * nx + ny * ny + nz * nz)
    ray = np.sqrt(x * x + y * y + depth * depth)
    cosine = np.abs(nx * x + ny * y + nz * depth) / np.maximum(norm, 1e-12) / np.maximum(ray, 1e-12)
    unknown = (~finite) | (norm < 1e-6) | ~np.isfinite(cosine)
    cosine = np.where(unknown, 1.0, cosine)
    return np.clip(cosine, 0.0, 1.0).astype(np.float32)


@functools.lru_cache(maxsize=8)
def _blob_axes(height: int, width: int):
    yy, xx = np.mgrid[0:height, 0:width]
    yy.setflags(write=False)
    xx.setflags(write=False)
    return yy, xx


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
        columns = np.clip(np.round(np.arange(width) + shift), 0, width - 1).astype(np.intp)
        disp_j = np.take_along_axis(disp, columns, axis=1)
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
        yy, xx = _blob_axes(height, width)
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


def native_sensor() -> dict:
    """名义 D435 深度相机。``profile=native`` 用它的像素角，不把每张图都裁成这块画幅。"""
    return {
        "width": NATIVE_WIDTH,
        "height": NATIVE_HEIGHT,
        "fx": NATIVE_FX,
        "baseline_m": float(DEF["baseline"]),
        "subpix": float(DEF["subpix"]),
    }


def _key_u(value: float) -> int:
    return int(round(float(value) * 1_000_000.0))


@functools.lru_cache(maxsize=8)
def _native_grid(height: int, width: int, fx_u: int, fy_u: int, cx_u: int, cy_u: int):
    """同一视场、原生 D435 角分辨率。返回的数组只读。"""
    fx = fx_u / 1_000_000.0
    fy = fy_u / 1_000_000.0
    cx = cx_u / 1_000_000.0
    cy = cy_u / 1_000_000.0
    native_w = max(1, int(round(width * NATIVE_FX / fx)))
    native_h = max(1, int(round(height * NATIVE_FX / fy)))
    native_fx = fx * native_w / float(width)
    native_fy = fy * native_h / float(height)
    native_cx = cx * native_w / float(width)
    native_cy = cy * native_h / float(height)
    rows = np.arange(native_h)
    cols = np.arange(native_w)
    ys = np.minimum(np.floor(rows * (height / native_h)).astype(np.intp), height - 1)
    xs = np.minimum(np.floor(cols * (width / native_w)).astype(np.intp), width - 1)
    flat = np.ascontiguousarray((ys[:, None] * width + xs[None, :]).astype(np.intp).ravel())
    expect = np.bincount(flat, minlength=height * width).reshape(height, width).astype(np.float64)
    for array in (ys, xs, flat, expect):
        array.setflags(write=False)
    return (native_h, native_w, native_fx, native_fy, native_cx, native_cy, ys, xs, flat, expect)


def _resample_disparity(
    depth: np.ndarray,
    flat: np.ndarray,
    expect: np.ndarray,
    native_fx: float,
    baseline: float,
    height: int,
    width: int,
    *,
    quantize: bool,
    unit: float,
) -> np.ndarray:
    """原生视差平均回网络分辨率。有效像素不足一半的格子仍是 0。"""
    fb = float(native_fx) * float(baseline)
    valid = np.isfinite(depth) & (depth > 0)
    disparity = np.zeros(depth.shape, np.float64)
    disparity[valid] = fb / np.asarray(depth, dtype=np.float64)[valid]
    total = np.bincount(flat, weights=disparity.ravel(), minlength=height * width)
    count = np.bincount(flat, weights=valid.reshape(-1).astype(np.float64), minlength=height * width)
    out = np.zeros(height * width, np.float64)
    good = (count >= NATIVE_VALID_FRACTION * expect.reshape(-1)) & (count > 0)
    out[good] = fb / (total[good] / count[good])
    if quantize and unit > 0:
        out[good] = np.round(out[good] / unit) * unit
    return out.reshape(height, width).astype(np.float32)


def native_d435_depth(
    z_gt: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    rng: np.random.Generator,
    *,
    holes_rng: np.random.Generator | None = None,
    params: dict | None = None,
    noise: bool = True,
    holes: bool = True,
) -> np.ndarray:
    """在原生 D435 网格上加噪，再重采样回 ``z_gt`` 的分辨率。0 表示无效。

    网格已经是 848×480、fx=426 时不再重采样，和 ``d435_depth`` 同一条路径。
    调用方传入的法向是粗分辨率的，这里不用；空洞打开时在原生网格上重算。
    """
    spec = dict(DEF)
    if params:
        spec.update(params)
    height, width = z_gt.shape
    fx, fy, cx, cy = (float(v) for v in intrinsics)
    native_h, native_w, native_fx, native_fy, native_cx, native_cy, ys, xs, flat, expect = _native_grid(
        height, width, _key_u(fx), _key_u(fy), _key_u(cx), _key_u(cy),
    )
    if native_h == height and native_w == width:
        normals = incidence_cos(z_gt, fx, fy, cx, cy) if holes else None
        return d435_depth(
            z_gt,
            intrinsics,
            rng,
            holes_rng=holes_rng,
            params=spec,
            noise=noise,
            holes=holes,
            normals_cos=normals,
        )
    lifted = np.asarray(z_gt)[np.ix_(ys, xs)]
    normals = incidence_cos(lifted, native_fx, native_fy, native_cx, native_cy) if holes else None
    raw = d435_depth(
        lifted,
        (native_fx, native_fy, native_cx, native_cy),
        rng,
        holes_rng=holes_rng,
        params=spec,
        noise=noise,
        holes=holes,
        normals_cos=normals,
    )
    return _resample_disparity(
        raw,
        flat,
        expect,
        native_fx,
        float(spec["baseline"]),
        height,
        width,
        quantize=bool(noise),
        unit=float(spec["depth_unit"]),
    )


def fill_holes_with_gt(depth: np.ndarray, z_gt: np.ndarray) -> np.ndarray:
    """无效像素改成同一位置的真值。真值也无效的仍留 0。不抽随机数。"""
    out = np.array(depth, dtype=np.float32, copy=True)
    gt = np.asarray(z_gt, dtype=np.float32)
    if gt.shape != out.shape:
        raise ValueError(f"真值深度形状 {gt.shape} 与加噪深度 {out.shape} 不一致")
    invalid = (~np.isfinite(out)) | (out <= 0)
    usable = np.isfinite(gt) & (gt > 0) & ~np.isinf(gt)
    out[invalid & usable] = gt[invalid & usable]
    return out


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
    profile: str = "scene",
) -> np.ndarray:
    """真值深度 + 内参 + rng → 加噪深度。无效像素按 ``invalid``。"""
    if profile not in PROFILES:
        raise ValueError(f"profile 必须是 {PROFILES} 之一，收到 {profile!r}")
    if profile == "native":
        raw = native_d435_depth(
            z_gt,
            intrinsics,
            rng,
            holes_rng=holes_rng,
            params=params,
            noise=noise,
            holes=holes,
        )
    else:
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
    mode = invalid
    if mode == "gt":
        raw = fill_holes_with_gt(raw, z_gt)
        mode = "nan"
    return finalize_depth(raw, mode, min_range_m, max_range_m)


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
            if self.cfg.profile == "native":
                raw = native_d435_depth(
                    ground,
                    cfg.intrinsics,
                    self.noise_rng,
                    holes_rng=self.holes_rng,
                    params=self.params,
                    noise=self.cfg.noise,
                    holes=self.cfg.holes,
                )
            else:
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
            mode = self.cfg.invalid
            if mode == "gt":
                raw = fill_holes_with_gt(raw, ground)
                mode = "nan"
            depth = finalize_depth(raw, mode, cfg.min_range_m, cfg.max_range_m)
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
