"""仅评估端使用的 G2 oracle；绝不传入 Planner / Observation / UavRuntime。"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from .synthetic import Box


class Outcome(Enum):
    SUCCESS = "SUCCESS"
    COLLISION = "COLLISION"
    ORACLE_PAUSE = "ORACLE_PAUSE"
    LAG_VIOLATION = "LAG_VIOLATION"


@dataclass
class FlightOracle:
    boxes: tuple[Box, ...]
    start: np.ndarray
    goal: np.ndarray
    start_time: float
    body_radius: float = 0.35
    tracking_bound: float = 0.15
    lag_limit: float = 1.5
    reference_speed: float = 0.15
    max_age: float = 0.2
    goal_tolerance: float = 0.4
    samples: list[dict] = field(default_factory=list)
    outcome: Outcome | None = None
    reason: str = ""
    goal_since: float | None = None
    ground_z: float | None = None
    support_center: np.ndarray | None = None
    support_radius: float = 0.15
    support_min_z: float | None = None
    airborne: bool = False
    mission_start_time: float | None = None

    def stop(self, outcome: Outcome, reason: str):
        if self.outcome is None:
            self.outcome, self.reason = outcome, reason
        return self.outcome

    def update(
        self,
        now: float,
        actual: np.ndarray,
        stamp: float,
        desired: np.ndarray,
        *,
        armed: bool = False,
        speed: float = 0.0,
        mission_ready: bool = True,
        phase: str = "PLANNING",
    ):
        if self.outcome is not None:
            return self.outcome
        actual, desired = np.asarray(actual), np.asarray(desired)
        if (
            not np.all(np.isfinite([now, stamp, *actual, *desired]))
            or not 0 <= now - stamp <= self.max_age
        ):
            return self.stop(Outcome.ORACLE_PAUSE, "ODOMETRY_AGE_OR_NONFINITE")
        delta = self.goal - self.start
        distance = np.linalg.norm(delta)
        direction = delta / max(distance, 1e-12)
        progress = float(np.dot(actual - self.start, direction))
        if mission_ready and self.mission_start_time is None:
            self.mission_start_time = now
        reference_start = self.start_time if self.ground_z is None else self.mission_start_time
        lag = (
            float(min(distance, max(0.0, now - reference_start) * self.reference_speed) - progress)
            if reference_start is not None
            else 0.0
        )
        error = float(np.linalg.norm(actual - desired))
        self.samples.append(
            {
                "time": now,
                "actual": actual.tolist(),
                "desired": desired.tolist(),
                "state_stamp": stamp,
                "tracking_error_m": error,
                "lag_m": lag,
                "phase": phase,
                "armed": armed,
                "speed_mps": speed,
            }
        )
        # 碰撞只看实际 PX4 位置，绝不对名义轨迹判碰撞。
        if any(box.distance(actual) <= self.body_radius for box in self.boxes):
            return self.stop(Outcome.COLLISION, "PX4_ACTUAL_POSITION")
        if self.ground_z is not None:
            if actual[2] - self.ground_z > self.body_radius + self.tracking_bound:
                self.airborne = True
            supported = (
                not self.airborne
                and not mission_ready
                and self.support_center is not None
                and self.support_min_z is not None
                and actual[2] >= self.support_min_z
                and np.linalg.norm(actual[:2] - self.support_center[:2]) <= self.support_radius
            )
            if actual[2] - self.ground_z <= self.body_radius and not supported:
                return self.stop(Outcome.COLLISION, "GROUND_PX4_ACTUAL_POSITION")
        if error > self.tracking_bound:
            return self.stop(Outcome.ORACLE_PAUSE, "TRACKING_BOUND")
        if lag > self.lag_limit:
            return self.stop(Outcome.LAG_VIOLATION, "P0_TIME_REFERENCE")
        if (
            mission_ready
            and armed
            and np.linalg.norm(actual - self.goal) <= self.goal_tolerance
            and speed <= 0.2
        ):
            if self.goal_since is None:
                self.goal_since = now
            if now - self.goal_since >= 1.0:
                return self.stop(Outcome.SUCCESS, "GOAL_REACHED_AND_HELD")
        else:
            self.goal_since = None
        return None

    def report(self):
        errors = [x["tracking_error_m"] for x in self.samples]
        return {
            "outcome": None if self.outcome is None else self.outcome.value,
            "reason": self.reason,
            "tracking_error_m_p50_max": None
            if not errors
            else [float(np.median(errors)), max(errors)],
            "tracking_error_m_p50_p95_max": None
            if not errors
            else np.quantile(errors, [0.5, 0.95, 1.0]).tolist(),
            "samples": self.samples,
        }
