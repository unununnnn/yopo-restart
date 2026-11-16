"""B6（默认关）：用场景柱几何过滤拟合后的候选，属于几何真值辅助实验。"""

import math

import numpy as np


class PillarClearance:
    def __init__(self, geometry: dict, margin_m: float, body_radius_m: float):
        pillars = [b for b in geometry.get("boxes", []) if str(b.get("name", "")).startswith("p_")]
        if not pillars:
            raise ValueError("pillar_clearance_m 只用于含 p_ 柱子的 crossing 几何")
        self.lo = np.asarray([b["lo"] for b in pillars], dtype=float)
        self.hi = np.asarray([b["hi"] for b in pillars], dtype=float)
        self.margin_m = margin_m
        self.body_radius_m = body_radius_m
        self.checks = 0
        self.rejected = 0

    def allows(self, motion, speed_limit: float) -> bool:
        # 距离函数是 1-Lipschitz；用轨迹限速 × 半个采样间隔覆盖采样点之间的运动。
        # 检查整条拟合计划（含停止尾），不把未拟合 raw 的远端混进来。
        span = float(motion.t_end - motion.t_start)
        n = max(2, int(math.ceil(span / 0.02)) + 1)
        times = np.linspace(float(motion.t_start), float(motion.t_end), n)
        points = np.asarray([motion.evaluate(float(t)) for t in times])
        delta = np.maximum(np.maximum(self.lo[None] - points[:, None], 0.0), points[:, None] - self.hi[None])
        gap = float(np.linalg.norm(delta, axis=-1).min()) - self.body_radius_m
        lower = gap - float(speed_limit) * span / (2 * (n - 1))
        ok = math.isfinite(lower) and lower + 1e-9 >= self.margin_m
        self.checks += 1
        self.rejected += int(not ok)
        return ok

    def summary(self) -> dict:
        return {
            "margin_m": self.margin_m,
            "checks": self.checks,
            "rejected": self.rejected,
            "source": "scene p_ boxes (oracle)",
            "deviation": "偏离上游：场景柱几何辅助的规划净空过滤",
        }
