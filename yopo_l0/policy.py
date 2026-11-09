"""规划策略。规划器只拿 Observation：深度帧 + 本机里程计 + 目标 + 仿真时间，没有任何场景几何。

票 02 在 POLICIES 里加 upstream / certified。
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass

import numpy as np

from .dynamics import HoverPlan, Plan, State
from .sensor import DepthFrame


@dataclass(frozen=True)
class Observation:
    frame: DepthFrame | None
    state: State  # 里程计（L0 为真值）
    goal: np.ndarray
    now: float

    def digest(self) -> str:
        h = hashlib.sha256()
        h.update(np.float64(self.now).tobytes())
        h.update(np.asarray(self.goal, dtype=np.float64).tobytes())
        s = self.state
        for x in (s.p, s.v, s.a):
            h.update(np.asarray(x, dtype=np.float64).tobytes())
        h.update(np.float64(s.yaw).tobytes())
        if self.frame is not None:
            h.update(np.float64(self.frame.stamp).tobytes())
            h.update(np.asarray(self.frame.K, dtype=np.float64).tobytes())
            h.update(np.ascontiguousarray(self.frame.depth).tobytes())
        return h.hexdigest()


class HoverPolicy:
    """票 01：在起点上方悬停。第一次规划锁定当前位置。"""

    name = "hover"

    def __init__(self):
        self._plan: Plan | None = None

    def plan(self, obs: Observation) -> Plan:
        if self._plan is None:
            self._plan = HoverPlan(tuple(float(x) for x in obs.goal), obs.state.yaw)
        return self._plan


POLICIES = {"hover": HoverPolicy}
