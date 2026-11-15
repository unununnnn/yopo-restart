"""影子盾的场景证据（研究开关，默认关）。

L0 前视深度进 ``FreeSpace`` 之后，扫掠样本只要落在相机最小量程内、身后、视场外，
或者包围盒里有无效像素，``classify_ball`` 就是 UNKNOWN。``classify_sweep`` 又要求
整条运动和制动分支都 FREE，于是离开起飞区的计划整段变成 ``ENV:UNKNOWN``。
起飞区 ``assume_free`` 只盖起点附近，而且没有新鲜帧时 ``classify_sweep`` 直接返回
``NO_FRESH_OBSERVATION``，证书根本不会被问起。

本模块不改 ``core/``。打开时做两件事，都走 core 已有接口：

- 用 ``yopo_l0.sensor.render`` 对场景卡几何打几张无噪声深度，按 ``DepthFrame.from_raw``
  放进影子自己的 ``FreeSpace``。表面点落在样本球里时，判定仍是 ``ENV:OCCUPIED``。
- 对 ``FreeSpace._sample_times`` 采到的、到障碍表面的净空不小于样本半径的球，
  调用 ``add_certificate``。证书只把「各帧都是 UNKNOWN」的球升为 FREE，碰不到
  OCCUPIED。净空不够的球不写证书，贴近障碍的扫掠仍然会被标出。

帧数和采样点数都是定数，没有墙钟预算。关着时调用方不构造本对象。
"""

from __future__ import annotations

import copy
import math

import numpy as np

from yopo_swarm_core.freespace import CameraIntrinsics, DepthFrame, FreeSpace
from yopo_swarm_core.plan import Plan
from yopo_swarm_core.trajectory import PiecewisePolynomial

from .scene import Box, Cylinder, Obstacle
from .sensor import CameraConfig, render

# 比样本半径多出来的这一截只用来躲开浮点贴边。小于它的净空不写证书。
_CERT_EPS_M = 1e-3
# 证据相机拉到包围盒外面，近表面要落在 FreeSpace 最小量程之外。
_PULLBACK_M = 1.5
_MIN_RANGE_M = 0.3
_MAX_RANGE_M = 30.0
_FOOTPRINT_M = 0.025
_MAX_WIDTH = 640
_MAX_HEIGHT = 480


def _look_rotation(forward: np.ndarray) -> np.ndarray:
    """相机轴在世界系的列：x 右、y 下、z 前。"""
    f = np.asarray(forward, dtype=float)
    f = f / np.linalg.norm(f)
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(f, world_up))) > 0.95:
        right = np.array([1.0, 0.0, 0.0])
    else:
        right = np.cross(f, world_up)
        right = right / np.linalg.norm(right)
    down = np.cross(f, right)
    down = down / np.linalg.norm(down)
    right = np.cross(down, f)
    right = right / np.linalg.norm(right)
    return np.column_stack([right, down, f])


def _solid_bounds(obstacles: tuple[Obstacle, ...]) -> tuple[np.ndarray, np.ndarray] | None:
    lows, highs = [], []
    for ob in obstacles:
        if isinstance(ob, Box):
            lows.append(np.asarray(ob.lo, dtype=float))
            highs.append(np.asarray(ob.hi, dtype=float))
        elif isinstance(ob, Cylinder):
            a, b = ob.axis_ends()
            pad = np.array([ob.radius, ob.radius, 0.0])
            lows.append(np.minimum(a, b) - pad)
            highs.append(np.maximum(a, b) + pad)
    if not lows:
        return None
    return np.min(np.stack(lows), axis=0), np.max(np.stack(highs), axis=0)


def _route_bounds(start: np.ndarray, goal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """没有墙和柱时，用起点到终点再外扩一截。编队转弯的横向鼓包落在这截里。"""
    lo = np.minimum(start, goal) - np.array([12.0, 12.0, 2.0])
    hi = np.maximum(start, goal) + np.array([12.0, 12.0, 2.0])
    lo[2] = min(float(lo[2]), 0.0)
    hi[2] = max(float(hi[2]), 4.0)
    return lo, hi


def _corners(lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    return np.array(
        [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])],
        dtype=float,
    )


def _fit_fov(origin: np.ndarray, R: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> CameraConfig:
    cam = (_corners(lo, hi) - origin) @ R
    z = cam[:, 2]
    front = z > 0.4
    if not np.any(front):
        front = np.ones(len(z), dtype=bool)
    zf = np.maximum(z[front], 1e-3)
    tx = float(np.max(np.abs(cam[front, 0]) / zf)) * 1.2
    ty = float(np.max(np.abs(cam[front, 1]) / zf)) * 1.2
    tx = min(max(tx, math.tan(math.radians(25.0))), math.tan(math.radians(60.0)))
    ty = min(max(ty, math.tan(math.radians(20.0))), math.tan(math.radians(55.0)))
    z_ref = float(np.median(zf))
    fx = max(z_ref / _FOOTPRINT_M, 40.0)
    fy = max(z_ref / _FOOTPRINT_M, 40.0)
    width = int(np.clip(math.ceil(2.0 * fx * tx), 160, _MAX_WIDTH))
    height = int(np.clip(math.ceil(2.0 * fy * ty), 120, _MAX_HEIGHT))
    return CameraConfig(
        width=width,
        height=height,
        hfov_deg=math.degrees(2.0 * math.atan(tx)),
        vfov_deg=math.degrees(2.0 * math.atan(ty)),
        min_range_m=_MIN_RANGE_M,
        max_range_m=_MAX_RANGE_M,
        noise_std_m=0.0,
    )


def _view_templates(lo: np.ndarray, hi: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
    center = 0.5 * (lo + hi)
    half = 0.5 * (hi - lo)
    forwards = (
        np.array([1.0, 0.0, 0.0]),
        np.array([-1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, -1.0, 0.0]),
        np.array([0.0, 0.0, -1.0]),
    )
    out = []
    for forward in forwards:
        along = float(abs(np.dot(half, np.abs(forward))))
        origin = center - forward * (along + _PULLBACK_M)
        out.append((origin, _look_rotation(forward)))
    return out


class SceneEvidence:
    """一架影子盾自己的几何证据。深度只渲染一次，之后按拍改时间戳。"""

    def __init__(self, obstacles: tuple[Obstacle, ...], start: np.ndarray, goal: np.ndarray):
        self.obstacles = tuple(obstacles)
        self.clearance_evals = 0
        self.certificate_centers = 0
        bounds = _solid_bounds(self.obstacles)
        if bounds is None:
            bounds = _route_bounds(np.asarray(start, dtype=float), np.asarray(goal, dtype=float))
        lo, hi = bounds
        rng = np.random.default_rng(0)
        cached: list[DepthFrame] = []
        for origin, rot in _view_templates(lo, hi):
            cfg = _fit_fov(origin, rot, lo, hi)
            shot = render(self.obstacles, cfg, origin, rot, rng, 0.0, None)
            T = np.eye(4)
            T[:3, :3] = rot
            T[:3, 3] = origin
            k = CameraIntrinsics(*shot.K, width=cfg.width, height=cfg.height)
            # 用 FreeSpace 自己的量程做有效掩码。噪声为 0，rng 不会被抽出。
            cached.append(
                DepthFrame.from_raw(
                    shot.depth, k, T, 0.0, _space_cfg_match(),
                )
            )
        self._cached = tuple(cached)
        self.n_views = len(self._cached)

    def add_frames(self, space: FreeSpace, now: float) -> None:
        """同一张深度，时间戳改成这一拍。数组共用，不重新求交。"""
        stamp = float(now)
        for src in self._cached:
            frame = copy.copy(src)
            frame.stamp = stamp
            space.add(frame)

    def certify_plan(self, space: FreeSpace, plan: Plan) -> None:
        self.certify_trajs(space, [plan.motion, *(b.traj for b in plan.branches)])

    def certify_trajs(self, space: FreeSpace, trajs: list[PiecewisePolynomial]) -> None:
        radius = float(space.cfg.sample_radius)
        need = radius + _CERT_EPS_M
        centers: list[np.ndarray] = []
        for traj in trajs:
            times = space._sample_times(traj)
            if isinstance(times, str):
                continue
            for t in times:
                p = np.asarray(traj.evaluate(float(t)), dtype=float)
                if self._clearance(p) >= need:
                    centers.append(p)
        if not centers:
            return
        space.add_certificate(np.stack(centers), need, "scene-card-free")
        self.certificate_centers += len(centers)

    def _clearance(self, p: np.ndarray) -> float:
        self.clearance_evals += len(self.obstacles)
        best = math.inf
        for ob in self.obstacles:
            best = min(best, ob.distance(p))
        return best


def _space_cfg_match():
    """``from_raw`` 只用 min_range / max_range。与 ``FreeSpaceConfig`` 缺省一致。"""
    from yopo_swarm_core.freespace import FreeSpaceConfig

    return FreeSpaceConfig(min_range=_MIN_RANGE_M, max_range=_MAX_RANGE_M)
