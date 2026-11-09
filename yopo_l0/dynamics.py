"""L0 动力学：接口 step(state, approved_plan, dt) -> state。首版点质量 + 一阶加速度滞后。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class State:
    t: float
    p: np.ndarray
    v: np.ndarray
    a: np.ndarray
    yaw: float = 0.0


class Plan(Protocol):
    """本机批准的计划：按仿真时间给参考位置/速度/加速度/偏航（与 L1 TrajectorySetpoint 同语义）。"""

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]: ...


@dataclass(frozen=True)
class HoverPlan:
    position: tuple[float, float, float]
    yaw: float = 0.0

    def sample(self, t: float):
        z = np.zeros(3)
        return np.asarray(self.position, dtype=float), z, z, self.yaw


@dataclass(frozen=True)
class PointMassLag:
    tau_s: float = 0.1
    kp: float = 4.0
    kd: float = 3.0
    a_max: float = 6.0

    def step(self, s: State, plan: Plan, dt: float) -> State:
        p_ref, v_ref, a_ref, yaw = plan.sample(s.t)
        a_cmd = a_ref + self.kp * (p_ref - s.p) + self.kd * (v_ref - s.v)
        n = float(np.linalg.norm(a_cmd))
        if n > self.a_max:
            a_cmd = a_cmd * (self.a_max / n)
        alpha = 1.0 - np.exp(-dt / self.tau_s)
        a = s.a + alpha * (a_cmd - s.a)
        v = s.v + a * dt
        p = s.p + v * dt
        return State(s.t + dt, p, v, a, yaw)


MODELS = {"point_mass_lag": PointMassLag}


def make_model(cfg: dict):
    cfg = dict(cfg)
    return MODELS[cfg.pop("model")](**cfg)
