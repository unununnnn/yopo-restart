"""票 B2：交叉卡规划净空归因的只读记录器。默认不装，装上也不改任何选择。

L0 和 SITL 用同一个记录器，对照「规划输入」：

* 规划时刻：本机里程计（深度视点）和本机正在执行的计划在同一时刻差多少（跟踪误差），
  拆成沿航向和侧向；计划头（t_start 上的计划状态）离深度视点多远。
* 深度帧和规划时刻的对齐：最新一帧深度的年龄（规划时刻减帧时刻）。
* 每次前向留下几条过了运动学的候选（``ranking.order[:6]`` 里拟合成功的条数）。
* 每份提案的几何净空：计划运动段上到场景卡障碍表面的最小距离减机体半径。
* 每拍：设定点（已提交计划）的净空、里程计的净空和两者之差。

所有时刻都用协议钟（L0 从 0 起；SITL 从全体进入 PLAN 起）。里程计记的是 ingest 进来的那份。
"""

from __future__ import annotations

import dataclasses
import math
from typing import Callable

import numpy as np

from yopo_swarm_core.plan import state_at

SCHEMA = "yopo-restart/b2-plan-diag/1"
_SAMPLE_DT = 0.02
_MAX_EVENTS = 4000


def _r(x, nd=4):
    return None if x is None or not math.isfinite(float(x)) else round(float(x), nd)


def _split(delta: np.ndarray, heading_xy: np.ndarray) -> tuple[float, float]:
    """水平差拆成沿航向和侧向（左正）。航向长度为 0 时全算侧向的模。"""
    d = np.asarray(delta, dtype=float)[:2]
    n = float(np.linalg.norm(heading_xy))
    if n < 1e-6:
        return 0.0, float(np.linalg.norm(d))
    u = np.asarray(heading_xy, dtype=float) / n
    along = float(d @ u)
    lat = float(u[0] * d[1] - u[1] * d[0])
    return along, lat


def _stats(vals: list[float]) -> dict | None:
    v = [float(x) for x in vals if x is not None and math.isfinite(float(x))]
    if not v:
        return None
    a = np.asarray(v)
    return {
        "n": len(v),
        "median": _r(np.median(a)),
        "p95": _r(np.percentile(a, 95)),
        "max": _r(a.max()),
        "min": _r(a.min()),
    }


class PlanDiag:
    """装在一个 CrossingFleet 实例上。``distance`` 是场景卡点到障碍表面的距离。"""

    def __init__(self, fleet, distance: Callable[[np.ndarray], float], body_radius: float, *, abs_offset: Callable[[], float] | None = None):
        self.fleet = fleet
        self.distance = distance
        self.body = float(body_radius)
        # SITL：ingest 的状态时刻是绝对仿真钟，协议钟要减去 PLAN 起点。L0 两者相同。
        self.abs_offset = abs_offset or (lambda: 0.0)
        self.state: dict[str, tuple[np.ndarray, np.ndarray, float]] = {}
        self.frame_pose: dict[str, tuple[int, np.ndarray, float]] = {}
        self.plans: list[dict] = []
        self.ticks: dict[str, list[dict]] = {did: [] for did in fleet.members}
        self._options_n: int | None = None
        self._install()

    # ---- 装钩子（实例属性，不改类） ----
    def _install(self) -> None:
        fleet = self.fleet
        ingest = fleet.ingest
        track = fleet.track
        options = fleet._yopo_options

        def ingest_hook(drone_id, frame, state):
            p = np.asarray(state.p, dtype=float).copy()
            v = np.asarray(state.v, dtype=float).copy()
            t = float(state.t) - float(self.abs_offset())
            self.state[drone_id] = (p, v, t)
            if frame is not None:
                prev = self.frame_pose.get(drone_id)
                if prev is None or prev[0] != id(frame):
                    pose = getattr(frame, "_position", None)
                    pose = p if pose is None else np.asarray(pose, dtype=float).copy()
                    self.frame_pose[drone_id] = (id(frame), pose, float(frame.stamp) - float(self.abs_offset()))
            return ingest(drone_id, frame, state)

        def track_hook(drone_id, now):
            out = track(drone_id, now)
            self._tick(drone_id, float(now))
            return out

        def options_hook(*args, **kwargs):
            motions, code = options(*args, **kwargs)
            self._options_n = len(motions)
            return motions, code

        fleet.ingest = ingest_hook
        fleet.track = track_hook
        fleet._yopo_options = options_hook
        for did, node in fleet.nodes.items():
            planner = node.planner

            def plan_hook(n, now, t_start, _planner=planner, _did=did):
                self._options_n = None
                parent = n.committed.occupancy.plan
                occ = _planner(n, now, t_start)
                self._plan(_did, float(now), float(t_start), parent, occ)
                return occ

            node.planner = plan_hook

    # ---- 记录 ----
    def _clear(self, p) -> float:
        return float(self.distance(np.asarray(p, dtype=float))) - self.body

    def _motion_clearance(self, plan) -> tuple[float, float]:
        tr = plan.motion
        t0, t1 = float(tr.t_start), float(tr.t_end)
        n = max(2, int(math.ceil((t1 - t0) / _SAMPLE_DT)) + 1)
        best, at = math.inf, t0
        for t in np.linspace(t0, t1, n):
            c = self._clear(tr.evaluate(float(t)))
            if c < best:
                best, at = c, float(t)
        return best, at

    def _plan(self, did: str, now: float, t_start: float, parent, occ) -> None:
        if len(self.plans) >= _MAX_EVENTS:
            return
        st = self.state.get(did)
        fr = self.frame_pose.get(did)
        row = {"uav": did, "now": _r(now, 3), "t_start": _r(t_start, 3), "code": self.fleet.code(did),
               "options": self._options_n, "proposed": occ is not None}
        p_now, v_now, _a = state_at(parent, now)
        head = state_at(parent, t_start)[0]
        heading = np.asarray(v_now, dtype=float)[:2]
        if st is not None:
            odom_p, odom_v, odom_t = st
            along, lat = _split(odom_p - p_now, heading)
            row.update(
                odom_age_s=_r(now - odom_t, 3),
                track_err_m=_r(np.linalg.norm(odom_p - p_now)),
                track_along_m=_r(along),
                track_lat_m=_r(lat),
                speed_plan=_r(np.linalg.norm(v_now)),
                speed_odom=_r(np.linalg.norm(odom_v)),
                odom_clear_m=_r(self._clear(odom_p)),
            )
        if fr is not None:
            _fid, pose, stamp = fr
            along, lat = _split(head - pose, heading)
            row.update(frame_age_s=_r(now - stamp, 3), head_from_view_along_m=_r(along), head_from_view_lat_m=_r(lat))
        if occ is not None:
            c, at = self._motion_clearance(occ.plan)
            row.update(plan_clear_m=_r(c), plan_clear_at=_r(at, 3), plan_t_end=_r(occ.plan.t_end, 3))
        self.plans.append(row)

    def _tick(self, did: str, now: float) -> None:
        st = self.state.get(did)
        if st is None:
            return
        rows = self.ticks[did]
        if len(rows) >= _MAX_EVENTS:
            return
        rec = self.fleet._last_rec.get(did)
        if rec is None:
            return
        sp, sv, _sa = state_at(rec.occupancy.plan, now)
        odom_p = st[0]
        along, lat = _split(odom_p - sp, np.asarray(sv)[:2])
        rows.append({
            "t": round(now, 3),
            "sp_clear": self._clear(sp),
            "odom_clear": self._clear(odom_p),
            "err": float(np.linalg.norm(odom_p - sp)),
            "along": along,
            "lat": lat,
            "speed": float(np.linalg.norm(sv)),
            "x": float(odom_p[0]),
        })

    # ---- 汇总 ----
    def summary(self) -> dict:
        per = {}
        for did, rows in self.ticks.items():
            moving = [r for r in rows if r["speed"] > 0.5]
            worst = min(rows, key=lambda r: r["odom_clear"]) if rows else None
            worst_sp = min(rows, key=lambda r: r["sp_clear"]) if rows else None
            per[did] = {
                "ticks": len(rows),
                "track_err_moving": _stats([r["err"] for r in moving]),
                "track_lat_abs_moving": _stats([abs(r["lat"]) for r in moving]),
                "track_along_moving": _stats([r["along"] for r in moving]),
                "min_setpoint_clear_m": None if worst_sp is None else _r(worst_sp["sp_clear"]),
                "min_setpoint_clear_t": None if worst_sp is None else worst_sp["t"],
                "min_odom_clear_m": None if worst is None else _r(worst["odom_clear"]),
                "at_min_odom": None if worst is None else {
                    "t": worst["t"], "x": _r(worst["x"], 3), "setpoint_clear_m": _r(worst["sp_clear"]),
                    "err_m": _r(worst["err"]), "along_m": _r(worst["along"]), "lat_m": _r(worst["lat"]),
                    "speed": _r(worst["speed"], 3)},
            }
        proposed = [p for p in self.plans if p.get("proposed")]
        moving_plans = [p for p in self.plans if (p.get("speed_plan") or 0.0) > 0.5]
        return {
            "schema": SCHEMA,
            "per_drone": per,
            "planner_calls": len(self.plans),
            "proposals": len(proposed),
            "plan_clear_min_m": _r(min((p["plan_clear_m"] for p in proposed if p.get("plan_clear_m") is not None), default=math.inf)),
            "plan_clear": _stats([p.get("plan_clear_m") for p in proposed]),
            "at_plan_track_err": _stats([p.get("track_err_m") for p in moving_plans]),
            "at_plan_track_lat_abs": _stats([abs(p["track_lat_m"]) for p in moving_plans if p.get("track_lat_m") is not None]),
            "at_plan_head_from_view_lat_abs": _stats([abs(p["head_from_view_lat_m"]) for p in moving_plans if p.get("head_from_view_lat_m") is not None]),
            "at_plan_head_from_view_along": _stats([p.get("head_from_view_along_m") for p in moving_plans]),
            "at_plan_frame_age_s": _stats([p.get("frame_age_s") for p in self.plans]),
            "at_plan_odom_age_s": _stats([p.get("odom_age_s") for p in self.plans]),
            "options_per_call": _stats([p.get("options") for p in self.plans if p.get("options") is not None]),
            "plans": self.plans,
        }


def apply_b2_overrides(card, *, safe_radius: float | None = None):
    """在内存里盖过场景卡的研究开关。都缺省时原样返回同一个对象，卡哈希不变。

    返回 (card, deviations)。deviations 非空就是「偏离上游」。
    """
    deviations: list[str] = []
    if safe_radius is None:
        return card, deviations
    radius = float(safe_radius)
    if not radius > 0.0 or not math.isfinite(radius):
        raise ValueError(f"--safe-radius 必须为正，收到 {safe_radius}")
    params = dict(card.card.get("policy_params") or {})
    upstream = dict(params.get("upstream") or {})
    upstream["safe_radius"] = radius
    params["upstream"] = upstream
    deviations.append(f"偏离上游：safe_radius={radius:g}")
    return dataclasses.replace(card, card={**card.card, "policy_params": params}), deviations
