"""有界支撑面起飞契约；仅起飞阶段使用，#9 永远使用完整球形环境检查。

前视相机无法证明机体上方/身后自由：显式假设半径为 pad_radius 的垂直起飞柱无障碍。
原始深度不修改；任何非支撑面占用仍具有否决权。支撑接触只许可在初始 XY 的误差管内、
不低于初始接触高度的中心。安全高度 = 实测支撑面 + 扫掠半径 + 两倍跟踪余量。
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from .freespace import FreeSpace, SpaceVerdict
from .plan import hover_plan
from .planner import ApprovedPlan, straight_plan


def world_points(frame):
    points = frame.points_cam.reshape(-1, 3)
    return points[np.isfinite(points[:, 2])] @ frame.R.T + frame.o


class SupportSpace(FreeSpace):
    """保留并标记地面接触证据；这不是在线规划的自由空间实例。"""

    def __init__(self, cfg, start, ground_z, target, pad_radius, plane_band):
        super().__init__(cfg)
        self.start = np.asarray(start).copy()
        self.ground_z, self.target = ground_z, np.asarray(target).copy()
        self.pad_radius, self.plane_band = pad_radius, plane_band
        self._points = {}
        self.support_points = 0
        # 球证书覆盖垂直柱；每个请求球仍需完全包含在证书球中。
        for z in np.linspace(start[2], target[2], 33):
            self.assume_free(np.array([*start[:2], z]), pad_radius, "explicit-takeoff-column")

    def add(self, frame):
        super().add(frame)
        points = world_points(frame)
        support = (np.abs(points[:, 2] - self.ground_z) <= self.plane_band) & (
            np.linalg.norm(points[:, :2] - self.start[:2], axis=1) <= self.pad_radius
        )
        self.support_points = int(support.sum())
        # 不改 frame / mask；独立缓存接触分类，原始地面证据一直保留。
        self._points[id(frame)] = points[~support]
        self._points = {id(f): self._points[id(f)] for f in self.frames}

    def classify_point(self, p, radius, now):
        p = np.asarray(p)
        if (
            np.linalg.norm(p[:2] - self.start[:2]) > self.cfg.margin
            or p[2] < self.start[2] - self.plane_band
            or p[2] > self.target[2] + self.plane_band
            or radius > self.cfg.sample_radius + 1e-12
        ):
            return SpaceVerdict.OCCUPIED
        return super().classify_point(p, radius, now)

    def _frames_verdict(self, p, radius, now):
        for frame in self.fresh_frames(now):
            pts = self._points[id(frame)]
            if len(pts) and np.min(np.linalg.norm(pts - p, axis=1)) <= radius:
                return SpaceVerdict.OCCUPIED
        return SpaceVerdict.UNKNOWN


@dataclass
class TakeoffContract:
    duration: float = 16.0  # 配置值；覆盖 PX4 默认 3 s 起飞 ramp，保持原跟踪界。
    plane_band: float = 0.01
    settle_time: float = 1.0
    position_tolerance: float = 0.05
    speed_tolerance: float = 0.1
    phase: str = "INITIAL"
    ground_z: float | None = None
    target: np.ndarray | None = None
    space: SupportSpace | None = None
    stable_since: float | None = None
    last_stable_check: float | None = None
    ready: bool = False
    approvals: list = field(default_factory=list)

    def observe(self, rt, state, frame_stamp, now, armed, offboard):
        """发布线程持续维护实际稳定窗口；审批线程忙时也不能漏过失稳。"""
        cfg = rt.planner.cfg
        if (
            self.phase not in ("CLIMB", "SETTLE")
            or rt.current is None
            or rt.pending is not None
            or now < rt.current.plan.t_end
        ):
            return
        self.phase = "SETTLE"
        stable = (
            armed
            and offboard
            and not rt.clock_fault
            and 0 <= now - state.stamp <= cfg.max_frame_age
            and 0 <= now - frame_stamp <= cfg.max_frame_age
            and np.linalg.norm(state.position - self.target) <= self.position_tolerance
            and state.position[2]
            >= self.ground_z + cfg.space.sample_radius + cfg.plan.tracking_margin
            and np.linalg.norm(state.velocity) <= self.speed_tolerance
        )
        if not stable:
            self.stable_since = self.last_stable_check = None
            return
        if (
            self.last_stable_check is None
            or not 0 <= now - self.last_stable_check <= cfg.max_frame_age
        ):
            self.stable_since = now
        self.last_stable_check = now

    def initialize(self, frame, state, cfg, pad_radius):
        points = world_points(frame)
        # x500 已落地 base_link 到支撑面的标定范围；不用 oracle 地图/位姿。
        near = points[
            (points[:, 2] >= state.position[2] - 0.30)
            & (points[:, 2] <= state.position[2] - 0.18)
            & (np.linalg.norm(points[:, :2] - state.position[:2], axis=1) <= pad_radius)
        ]
        if len(near) < 50:
            return False
        A = np.column_stack([near[:, :2] - state.position[:2], np.ones(len(near))])
        plane, _, rank, _ = np.linalg.lstsq(A, near[:, 2], rcond=None)
        if (
            rank < 3
            or np.linalg.norm(plane[:2]) > 0.01
            or np.max(np.abs(A @ plane - near[:, 2])) > self.plane_band
        ):
            return False
        self.ground_z = float(plane[2])
        self.target = state.position.copy()
        self.target[2] = self.ground_z + cfg.space.sample_radius + 2 * cfg.plan.tracking_margin
        if self.target[2] <= state.position[2] or pad_radius <= cfg.space.sample_radius:
            return False
        self.space = SupportSpace(
            cfg.space, state.position, self.ground_z, self.target, pad_radius, self.plane_band
        )
        self.phase = "HOLD"
        return True

    def propose(self, rt, frame, state, now, armed, offboard):
        cfg = rt.planner.cfg
        if (
            self.ready
            or self.space is None
            or rt.current is None
            or rt.pending is not None
            or rt.clock_fault
        ):
            return
        if not (
            armed
            and offboard
            and 0 <= now - frame.stamp <= cfg.max_frame_age
            and 0 <= now - state.stamp <= cfg.max_frame_age
            and abs(state.stamp - frame.stamp) <= cfg.max_skew
            and np.mean(frame.valid) >= cfg.min_valid_fraction
        ):
            return
        self.space.add(frame)
        if self.phase == "HOLD":
            plan = straight_plan(
                rt.current.plan, now + cfg.switch_delay, self.target, cfg, duration=self.duration
            )
            if isinstance(plan, list):
                rt.last_error = "TAKEOFF_TRANSITION"
                return
            token = rt.planner.gate.validate(plan, rt.current.plan, None, self.space, (), now)
            if isinstance(token, ApprovedPlan):
                return token
            else:
                rt.last_error = ",".join(token.codes)
            return
        if now < rt.current.plan.t_end:
            return
        if self.stable_since is None or now - self.stable_since < self.settle_time:
            return
        # 恢复完整球形环境证据，经同一 SafetyGate 再批准空中保持后才释放 #9。
        if not rt.planner.space.assumptions:
            rt.planner.assume_free(
                self.target, self.space.pad_radius, "explicit-airborne-start-pad"
            )
        rt.planner.space.add(frame)
        hold = replace(
            hover_plan(
                cfg.owner,
                self.target,
                now + cfg.switch_delay,
                cfg.plan,
                parent=rt.current.plan_hash,
            ),
            plan_seq=rt.current.plan.plan_seq + 1,
        )
        token = rt.planner.gate.validate(hold, rt.current.plan, None, rt.planner.space, (), now)
        if isinstance(token, ApprovedPlan):
            return token
        else:
            rt.last_error = ",".join(token.codes)

    def accept(self, rt, token, parent_hash, now, armed=False, offboard=False):
        if (
            not isinstance(token, ApprovedPlan)
            or rt.clock_fault
            or rt.current is None
            or rt.current.plan_hash != parent_hash
            or token.plan.parent_plan_hash != parent_hash
            or rt.pending is not None
            or token.plan.t_switch <= now
        ):
            rt.last_error = "TAKEOFF_LATE_OR_PARENT_CHANGED"
            return False
        if not (armed and offboard):
            rt.last_error = "TAKEOFF_MODE_OR_ARM_CHANGED"
            return False
        if self.phase == "SETTLE" and (
            self.stable_since is None
            or self.last_stable_check is None
            or now - self.stable_since < self.settle_time
            or not 0 <= now - self.last_stable_check <= rt.planner.cfg.max_frame_age
        ):
            rt.last_error = "TAKEOFF_NO_LONGER_STABLE"
            return False
        rt.pending = token
        self.approvals.append(token)
        if self.phase == "HOLD":
            self.phase = "CLIMB"
        else:
            self.ready, self.phase = True, "READY"
        return True

    def describe(self):
        return {
            "phase": self.phase,
            "ground_z": self.ground_z,
            "target": None if self.target is None else self.target.tolist(),
            "duration_s": self.duration,
            "plane_band_m": self.plane_band,
            "settle_time_s": self.settle_time,
            "stable_since": self.stable_since,
            "last_stable_check": self.last_stable_check,
            "support_points": 0 if self.space is None else self.space.support_points,
            "pad_radius_m": None if self.space is None else self.space.pad_radius,
            "position_tolerance_m": self.position_tolerance,
            "speed_tolerance_mps": self.speed_tolerance,
        }
