"""安全证据：原始深度 → 局部滚动自由空间 + 扫掠体判定（ROS-free，B05 双管线）。

- 只用原始深度与有效掩码；网络输入用的补洞图不进入本模块（掩码为 False 的像素值一律不用）。
- 一个球（中心 c，半径 r）在某帧中：
  - OCCUPIED：该帧有有效像素的表面点落在球内；
  - FREE：球整体在相机前方且超过最小距离、投影包围框全在图像内、框内每个像素都有效
    且观测深度大于球的最远 z（球整体位于观测表面之前，被有效射线覆盖）；
  - 否则 UNKNOWN（遮挡后方、视场外、无回波、过近）。
- 多帧合并：任一未过期帧 OCCUPIED → OCCUPIED；否则任一未过期帧 FREE → FREE；否则 UNKNOWN。
  过期帧（now - stamp > max_age）不参与。假设：两帧之间环境静止。
- 扫掠体：按速度上界取样，保证相邻样本间位移 ≤ sample_spacing，
  每个样本球半径 = body_radius + margin + sample_spacing/2，样本球之并覆盖整个扫掠体。
- UNKNOWN 永远不按 FREE 放行；唯一例外是“证书”：
  - 已批准计划批准时，其扫掠样本以放大半径（r + slack）在新鲜帧中判 FREE 的球记为证书；
  - 起飞区假设（`assume_free`）也是证书，必须写进运行假设；
  - 证书只把“各帧都 UNKNOWN”的球升为 FREE（被某个证书球完整包含才算），任何新鲜帧的
    OCCUPIED 证据都优先于证书。依据是静态环境假设；用途是覆盖相机最小距离内与身后的区域。
- 分辨率假设：障碍在像素足迹以上尺度（小于像素间距的细障碍可能漏检），记录在运行假设中。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from yopo_swarm_core.collision import Verdict, check_derivative_bound
from yopo_swarm_core.trajectory import PiecewisePolynomial


class SpaceVerdict(Enum):
    FREE = "FREE"
    OCCUPIED = "OCCUPIED"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass(frozen=True)
class FreeSpaceConfig:
    body_radius: float = 0.2
    margin: float = 0.1
    max_age: float = 0.5  # 秒；超过即失效
    min_range: float = 0.3
    max_range: float = 30.0
    sample_spacing: float = 0.05
    max_frames: int = 4
    max_samples: int = 20_000
    max_certificate_sets: int = 8  # 滚动保留的已批准计划证书组数（起飞区假设另存，不滚动）

    @property
    def sample_radius(self) -> float:
        return self.body_radius + self.margin + 0.5 * self.sample_spacing


class DepthFrame:
    """一帧原始深度（z 深度，米）+ 有效掩码 + 内参 + 采集时刻外参 T_world_cam + 时间戳。"""

    def __init__(
        self,
        depth: np.ndarray,
        valid: np.ndarray,
        intrinsics: CameraIntrinsics,
        T_world_cam: np.ndarray,
        stamp: float,
    ) -> None:
        depth = np.asarray(depth, dtype=float)
        valid = np.asarray(valid, dtype=bool)
        k = intrinsics
        if depth.shape != (k.height, k.width) or valid.shape != depth.shape:
            raise ValueError("depth/mask shape does not match intrinsics")
        if np.any(valid & ~(np.isfinite(depth) & (depth > 0))):
            raise ValueError("mask marks non-finite or non-positive depth as valid")
        T = np.asarray(T_world_cam, dtype=float)
        if T.shape != (4, 4) or not np.all(np.isfinite(T)) or not math.isfinite(stamp):
            raise ValueError("bad pose or stamp")
        self.depth = np.where(valid, depth, np.nan)  # 无效像素的值（含补洞值）直接丢掉
        self.valid = valid
        self.k = intrinsics
        self.T = T
        self.R = T[:3, :3]
        self.o = T[:3, 3]
        self.stamp = float(stamp)
        u, v = np.meshgrid(np.arange(k.width) + 0.5, np.arange(k.height) + 0.5)
        pts_cam = np.stack(
            [(u - k.cx) / k.fx * self.depth, (v - k.cy) / k.fy * self.depth, self.depth], axis=-1
        )
        self.points_cam = pts_cam  # 无效像素为 NaN
        self._points_by_z = None

    @classmethod
    def from_raw(
        cls,
        depth: np.ndarray,
        intrinsics: CameraIntrinsics,
        T_world_cam: np.ndarray,
        stamp: float,
        cfg: FreeSpaceConfig,
    ) -> DepthFrame:
        d = np.asarray(depth, dtype=float)
        with np.errstate(invalid="ignore"):
            valid = np.isfinite(d) & (d >= cfg.min_range) & (d <= cfg.max_range)
        return cls(d, valid, intrinsics, T_world_cam, stamp)

    def classify_ball(self, center: np.ndarray, radius: float, min_range: float) -> SpaceVerdict:
        k = self.k
        c = self.R.T @ (np.asarray(center, dtype=float) - self.o)
        # 先查占用证据：包围框内有效表面点落在球内
        corners = c + radius * np.array(
            [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float
        )
        if np.min(corners[:, 2]) <= min_range:
            return self._occupied_near(c, radius) or SpaceVerdict.UNKNOWN
        us = k.fx * corners[:, 0] / corners[:, 2] + k.cx
        vs = k.fy * corners[:, 1] / corners[:, 2] + k.cy
        u0, u1 = math.floor(us.min()), math.ceil(us.max())
        v0, v1 = math.floor(vs.min()), math.ceil(vs.max())
        inside = u0 >= 0 and v0 >= 0 and u1 <= k.width and v1 <= k.height
        cu0, cu1 = max(u0, 0), min(max(u1, u0 + 1), k.width)
        cv0, cv1 = max(v0, 0), min(max(v1, v0 + 1), k.height)
        if cu0 < cu1 and cv0 < cv1:
            pts = self.points_cam[cv0:cv1, cu0:cu1].reshape(-1, 3)
            ok = np.isfinite(pts[:, 2])
            if ok.any() and np.min(np.linalg.norm(pts[ok] - c, axis=1)) <= radius:
                return SpaceVerdict.OCCUPIED
        if not inside:
            return SpaceVerdict.UNKNOWN
        patch = self.depth[v0:v1, u0:u1]
        if patch.size == 0 or not np.all(self.valid[v0:v1, u0:u1]):
            return SpaceVerdict.UNKNOWN
        if np.min(patch) > c[2] + radius:
            return SpaceVerdict.FREE
        return SpaceVerdict.UNKNOWN

    def _occupied_near(self, c: np.ndarray, radius: float) -> SpaceVerdict | None:
        # 按光轴深度索引，只排除 z 距离严格大于半径的点；球形判定本身不变。
        # 延迟构造放在规划工作线程，避免每帧给 ROS 接收回调增加排序开销。
        if self._points_by_z is None:
            pts = self.points_cam.reshape(-1, 3)
            pts = pts[np.isfinite(pts[:, 2])]
            self._points_by_z = pts[np.argsort(pts[:, 2])]
        all_points = self._points_by_z
        lo = np.searchsorted(all_points[:, 2], np.nextafter(c[2] - radius, -np.inf), side="left")
        hi = np.searchsorted(all_points[:, 2], np.nextafter(c[2] + radius, np.inf), side="right")
        pts = all_points[lo:hi]
        if len(pts) and np.min(np.linalg.norm(pts - c, axis=1)) <= radius:
            return SpaceVerdict.OCCUPIED
        return None


@dataclass(frozen=True)
class Certificate:
    centers: np.ndarray  # (M, 3)
    radius: float
    label: str

    def contains(self, p: np.ndarray, r: float) -> bool:
        if r > self.radius or len(self.centers) == 0:
            return False
        d = np.linalg.norm(self.centers - np.asarray(p, dtype=float), axis=1)
        return bool(np.min(d) <= self.radius - r)


@dataclass
class SweepResult:
    verdict: SpaceVerdict
    first_bad_time: float | None = None
    samples: int = 0
    reason: str = ""


@dataclass
class FreeSpace:
    """局部滚动自由空间（最近 max_frames 帧）；不是全局地图。"""

    cfg: FreeSpaceConfig = field(default_factory=FreeSpaceConfig)
    frames: deque = field(default_factory=deque)
    certificates: deque = field(default_factory=deque)
    assumptions: list = field(default_factory=list)

    def add(self, frame: DepthFrame) -> None:
        self.frames.append(frame)
        while len(self.frames) > self.cfg.max_frames:
            self.frames.popleft()

    def fresh_frames(self, now: float) -> list[DepthFrame]:
        return [f for f in self.frames if 0.0 <= now - f.stamp <= self.cfg.max_age]

    def assume_free(self, center: np.ndarray, radius: float, label: str) -> None:
        """运行假设：该球内无障碍（如起飞区）。调用方须把它写进日志头。"""
        c = np.asarray(center, dtype=float).reshape(1, 3)
        self.assumptions.append(Certificate(c, float(radius), label))

    def add_certificate(self, centers: np.ndarray, radius: float, label: str) -> None:
        self.certificates.append(
            Certificate(np.asarray(centers, dtype=float).reshape(-1, 3), float(radius), label)
        )
        while len(self.certificates) > self.cfg.max_certificate_sets:
            self.certificates.popleft()

    def _certified(self, p: np.ndarray, radius: float) -> bool:
        return any(c.contains(p, radius) for c in (*self.assumptions, *self.certificates))

    def _frames_verdict(self, p: np.ndarray, radius: float, now: float) -> SpaceVerdict:
        verdicts = [f.classify_ball(p, radius, self.cfg.min_range) for f in self.fresh_frames(now)]
        if SpaceVerdict.OCCUPIED in verdicts:
            return SpaceVerdict.OCCUPIED
        if SpaceVerdict.FREE in verdicts:
            return SpaceVerdict.FREE
        return SpaceVerdict.UNKNOWN

    def classify_point(self, p: np.ndarray, radius: float, now: float) -> SpaceVerdict:
        v = self._frames_verdict(p, radius, now)
        if v is SpaceVerdict.UNKNOWN and self._certified(p, radius):
            return SpaceVerdict.FREE
        return v

    def _sample_times(self, traj: PiecewisePolynomial) -> np.ndarray | str:
        if not traj.is_well_formed():
            return "MALFORMED"
        vb = check_derivative_bound(traj, 1, 1e9)
        if vb.verdict is not Verdict.SAFE or vb.upper_bound is None:
            return "SPEED_BOUND"
        span = traj.t_end - traj.t_start
        n = max(2, math.ceil(vb.upper_bound * span / self.cfg.sample_spacing) + 1)
        if n > self.cfg.max_samples:
            return "TOO_MANY_SAMPLES"
        return np.linspace(traj.t_start, traj.t_end, n)

    def certify_sweep(self, traj: PiecewisePolynomial, now: float, slack: float, label: str) -> int:
        """把扫掠样本中放大半径 (r + slack) 在新鲜帧中判 FREE 的球记为一组证书；返回记录数。"""
        times = self._sample_times(traj)
        if isinstance(times, str):
            return 0
        r = self.cfg.sample_radius + slack
        centers = [
            p
            for p in (traj.evaluate(float(t)) for t in times)
            if self._frames_verdict(p, r, now) is SpaceVerdict.FREE
        ]
        if centers:
            self.add_certificate(np.array(centers), r, label)
        return len(centers)

    def classify_sweep(self, traj: PiecewisePolynomial, now: float) -> SweepResult:
        if not traj.is_well_formed():
            return SweepResult(SpaceVerdict.UNKNOWN, reason="MALFORMED")
        if not self.fresh_frames(now):
            return SweepResult(SpaceVerdict.UNKNOWN, reason="NO_FRESH_OBSERVATION")
        times = self._sample_times(traj)
        if isinstance(times, str):
            return SweepResult(SpaceVerdict.UNKNOWN, reason=times)
        n = len(times)
        r = self.cfg.sample_radius
        result = SpaceVerdict.FREE
        first_unknown = None
        for t in times:
            v = self.classify_point(traj.evaluate(float(t)), r, now)
            if v is SpaceVerdict.OCCUPIED:
                return SweepResult(SpaceVerdict.OCCUPIED, float(t), n, "OCCUPIED")
            if v is SpaceVerdict.UNKNOWN and first_unknown is None:
                first_unknown = float(t)
                result = SpaceVerdict.UNKNOWN
        return SweepResult(
            result, first_unknown, n, "UNKNOWN" if first_unknown is not None else "FREE"
        )


__all__ = [
    "CameraIntrinsics",
    "Certificate",
    "DepthFrame",
    "FreeSpace",
    "FreeSpaceConfig",
    "SpaceVerdict",
    "SweepResult",
]
