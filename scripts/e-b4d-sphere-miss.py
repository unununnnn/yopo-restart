#!/usr/bin/env python3
"""票 E-B4d：014-swap3 seed 1 上欧氏球没把 uav1-uav2 推开。只报数。

E-B4c（PR #94，分支 ``yopo-restart-bot/e-b4c-sphere-avoid``）在
``ego-density-014-swap3`` seed 1 上仍是 ``uav1-uav2`` 相撞，最小距 0.694 m。
seed 0 是成功参照。本脚本不改后段，也不把 #94 的源码拷进来。

main 还没有 ``peer_sphere_enabled`` 时，``--partial`` 跳过实飞，JSON 标
``missing_feature``。开关进了 ``BackendConfig`` 且 ``refine_yopo`` 会读它之后，
去掉 ``--partial`` 才逐 tick 回答下面四问（对照 seed 0）：

1. 撞前 1.0 s 后段有没有收到邻机采样；第 0 个规划拍是不是空的
   （runner 先 ``plan`` 再 ``bus.poll``）。
2. 铰链有没有亮。亮的时候记三维距、代价值、控制点位移，以及末端钉死
   是不是让最后一跨推不动。
3. 碰撞落在当时轨迹的头 / 中 / 尾；是不是一侧已经
   ``CORRIDOR_EMPTY_HOVER`` / ``empty_brake``，另一侧还在飞过来。
4. 叠 ``geom_multi`` 为什么没解开：几何门放行的参考轨迹自己是不是已经 < 0.70 m。

欧氏球的邻机采样跟 E-B4c 的对照脚本一样，只在本进程里送进 ``refine_yopo``。
``policy.py`` 仍只在 ``swarm_penalty`` 时送采样，这里不改那个文件。
"""

from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SCENE = "ego-density-014-swap3"
SCENE_PATH = "scenes/ego-density-014-swap3.json"
ARMS = ("sphere", "sphere+track_geom_multi")
SEEDS = (0, 1)
PAIR = ("uav1", "uav2")
BALL_M = 0.70
RADIUS_M = 0.85
WEIGHT = 8.0
WINDOW_S = 1.0
END_SHIFT_EPS = 1e-6
PINNED_GRAD_EPS = 1e-8
PR_NUMBER = 94
PR_BRANCH = "yopo-restart-bot/e-b4c-sphere-avoid"
JSON_PATH = ROOT / "docs/research/e-b4d-sphere-miss-runs.json"
MD_PATH = ROOT / "docs/research/e-b4d-sphere-miss.md"

DISCLAIMER = (
    "report-only。只报数。不改 yopo_l0 后段、不冻结、不进 scenes/suite.txt，不设 gates。"
    "本票不拷贝 PR #94 的欧氏球实现。main 尚未带 peer_sphere_enabled 时跳过实飞，标 missing_feature。"
    "开关合入后再用同一脚本逐 tick 回答 014-swap3 seed 1（对照 seed 0）。"
)

# 数字引自 PR #94 的报告，本分支没有重测。
CITED = {
    "source": "PR #94 docs/research/e-b4c-sphere-avoid.md",
    "remeasured_here": False,
    "scene": SCENE,
    "sphere": {
        "seed0": {"outcome": "SUCCESS", "min_inter_m": "0.946", "arrival_time_s": "14.76"},
        "seed1": {"outcome": "COLLISION", "reason": "uav1-uav2:INTER_DRONE", "min_inter_m": "0.694"},
    },
    "sphere_track_geom_multi": {
        "seed0": {"outcome": "SUCCESS", "min_inter_m": "0.943", "arrival_time_s": "8.60"},
        "seed1": {"outcome": "COLLISION", "reason": "uav1-uav2:INTER_DRONE", "min_inter_m": "0.694"},
    },
}

QUESTION_SPEC = (
    (1, "neighbor_samples", "撞前 1.0 s 内后段是否收到邻机采样；第 0 个规划拍是否缺邻机"),
    (2, "hinge", "铰链是否激活；激活时的三维距、代价、控制点位移、末端是否因钉死无法上推"),
    (3, "locus", "碰撞在轨迹头/中/尾；是否一侧 CORRIDOR_EMPTY_HOVER / empty_brake、另一侧还在飞"),
    (4, "geom_multi", "几何门放行的参考轨迹自身是否已经 < 0.70 m"),
)


class MissingFeature(RuntimeError):
    """当前进程里的后段还没有欧氏球，不能做实飞诊断。"""


class _Log:
    def __init__(self):
        self.plans: list[dict] = []
        self.refines: list[dict] = []
        self.notes: list[dict] = []
        self.n_plans = 0


class _Restore:
    def __init__(self):
        self._saved: list[tuple[object, str, object]] = []

    def set(self, obj, name: str, value) -> None:
        self._saved.append((obj, name, getattr(obj, name)))
        setattr(obj, name, value)

    def restore(self) -> None:
        for obj, name, old in reversed(self._saved):
            setattr(obj, name, old)
        self._saved.clear()


def _deep(raw):
    return json.loads(json.dumps(raw))


def detect_support() -> dict:
    """看 ``BackendConfig`` / ``refine_yopo`` 认不认 ``peer_sphere_enabled``。

    字段在、``config_from_params`` 真的把它打开、并且 ``refine_yopo``（或它调用的
    ``_refine``）源码读这个字段，三者都成立才算支持。缺一就不飞。
    """
    from yopo_l0.backend import bspline as bs
    from yopo_l0.backend.config import BackendConfig

    fields = getattr(BackendConfig, "__dataclass_fields__", {})
    has_field = "peer_sphere_enabled" in fields
    honors = False
    if has_field:
        from yopo_l0.backend.config import config_from_params

        try:
            cfg = config_from_params(
                {"backend": {"enabled": True, "peer_sphere_enabled": True}},
                a_max=6.0,
            )
        except (TypeError, ValueError):
            cfg = None
        honors = cfg is not None and bool(getattr(cfg, "peer_sphere_enabled", False))
    chunks = []
    for fn in (bs.refine_yopo, getattr(bs, "_refine", None)):
        if fn is None:
            continue
        try:
            chunks.append(inspect.getsource(fn))
        except OSError:
            continue
    reads = any("peer_sphere_enabled" in chunk for chunk in chunks)
    return {
        "supported": bool(has_field and honors and reads),
        "config_has_peer_sphere_enabled": bool(has_field),
        "config_from_params_honors_flag": bool(honors),
        "refine_yopo_reads_peer_sphere": bool(reads),
        "peer_sphere_cost_grad": bool(hasattr(bs, "_peer_sphere_cost_grad")),
    }


def plan_before_poll() -> bool:
    """``run`` 的源码里，本 tick 的 ``plan`` 是否写在 ``bus.poll`` 前面。不飞。

    读文件而不是 ``inspect.getsource(run)``，这样测试把 ``runner.run`` 换成桩时
    仍检查仓库里的运行器，而不是桩函数。
    """
    src = (ROOT / "yopo_l0/runner.py").read_text(encoding="utf-8")
    start = src.find("\ndef run(")
    body = src if start < 0 else src[start:]
    plan_at = body.find('dr["policy"].plan(')
    poll_at = body.find("bus.poll(")
    return 0 <= plan_at < poll_at


def build_card(arm: str):
    """进程内覆盖。不写 ``scenes/``。"""
    from yopo_l0.scene import SceneCard, load_card

    if arm not in ARMS:
        raise ValueError(arm)
    loaded = load_card(ROOT / SCENE_PATH)
    raw = _deep(loaded.card)
    params = dict(raw.get("policy_params") or {})
    backend = dict(params.get("backend") or {})
    backend.update({
        "enabled": True,
        "peer_sphere_enabled": True,
        "peer_sphere_radius_m": RADIUS_M,
        "peer_sphere_weight": WEIGHT,
    })
    backend.pop("swarm_penalty", None)
    backend.pop("swarm_segment_penalty", None)
    params["backend"] = backend
    if arm == "sphere+track_geom_multi":
        upstream = dict(params.get("upstream") or {})
        upstream["track_clearance"] = True
        upstream["track_clearance_mode"] = "geom_multi"
        params["upstream"] = upstream
    raw["policy_params"] = params
    return SceneCard(loaded.path, raw, loaded.obstacles)


def _arm_flags(card) -> dict:
    params = card.card.get("policy_params") or {}
    backend = params.get("backend") or {}
    upstream = params.get("upstream") or {}
    return {
        "backend_enabled": backend.get("enabled") is True,
        "peer_sphere_enabled": backend.get("peer_sphere_enabled") is True,
        "peer_sphere_radius_m": backend.get("peer_sphere_radius_m"),
        "peer_sphere_weight": backend.get("peer_sphere_weight"),
        "swarm_penalty": backend.get("swarm_penalty") is True,
        "track_clearance": upstream.get("track_clearance") is True,
        "track_clearance_mode": upstream.get("track_clearance_mode"),
    }


def target_block() -> dict:
    return {
        "scene": SCENE,
        "scene_path": SCENE_PATH,
        "miss_seed": 1,
        "reference_seed": 0,
        "pair": list(PAIR),
        "collision_ball_m": BALL_M,
        "sphere_radius_m": RADIUS_M,
        "sphere_weight": WEIGHT,
        "window_s": WINDOW_S,
        "arms": list(ARMS),
    }


def _fmt(value, digits: int = 3) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{digits}f}"


def locus_of(t, t0, t1, code) -> str:
    """碰撞时刻落在这条已提交轨迹的哪一段。

    ``CORRIDOR_EMPTY_HOVER`` 且时长 < 0.05 s 是原地保持，单独标 ``hover_hold``。
    其余按 ``(t - t0) / (t1 - t0)``：``[0, 1/3)`` 头，``[1/3, 2/3)`` 中，``[2/3, 1]`` 尾。
    """
    if t is None or t0 is None or t1 is None:
        return "unknown"
    dur = float(t1) - float(t0)
    if code == "CORRIDOR_EMPTY_HOVER" and dur < 0.05:
        return "hover_hold"
    if dur <= 1e-9:
        return "degenerate"
    frac = (float(t) - float(t0)) / dur
    if frac < 0.0:
        return "before"
    if frac < 1.0 / 3.0:
        return "head"
    if frac < 2.0 / 3.0:
        return "mid"
    if frac <= 1.0 + 1e-9:
        return "tail"
    return "after"


def sample_traj(traj, dt: float = 0.02) -> np.ndarray:
    t0 = float(traj.t_start)
    t1 = float(traj.t_end)
    if t1 < t0:
        t0, t1 = t1, t0
    span = t1 - t0
    if span < 1e-9:
        ts = np.array([t0], dtype=float)
    else:
        n = max(int(round(span / dt)), 1)
        ts = t0 + np.arange(n + 1, dtype=float) * (span / n)
        ts[-1] = t1
    pts = np.stack([np.asarray(traj.evaluate(float(t), 0), dtype=float) for t in ts], axis=0)
    return np.column_stack((ts, pts))


def _neighbor_arrays(neighbors) -> list[np.ndarray]:
    out = []
    for raw in neighbors or ():
        arr = np.asarray(raw, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 4 or arr.shape[0] == 0:
            continue
        if not bool(np.all(np.isfinite(arr))):
            continue
        out.append(arr)
    return out


def polynomial_gap(traj, neighbors) -> float | None:
    """参考多项式与邻机采样的同时刻最小中心距。没有重叠就是 None。不外推。"""
    from yopo_l0.inter_drone import clearance_m

    arrays = _neighbor_arrays(neighbors)
    if not arrays:
        return None
    gap = float(clearance_m(sample_traj(traj), arrays))
    if not math.isfinite(gap):
        return None
    return gap


def closest_span(q, dt: float, t_start: float, neighbors, *, exclude_last: bool = False) -> dict:
    """均匀三次样条段内采样（与后段机间项同一组箱中点）上的最近邻机。

    ``exclude_last`` 时丢掉最后一跨。欧氏球铰链不看那一跨。
    """
    from yopo_l0.backend import bspline as bs

    ctrl = np.asarray(q, dtype=float)
    n_span = int(ctrl.shape[0]) - 3
    if n_span < 1:
        return {"dist_m": None, "span": None, "on_excluded_last_span": False}
    windows = bs._control_windows(ctrl)
    pts = np.einsum("kb,sbc->skc", bs._B_SEGMENT, windows)
    span_i = np.arange(n_span, dtype=float)[:, None]
    times = float(t_start) + (span_i + bs._SEGMENT_U[None, :]) * float(dt)
    arrays = _neighbor_arrays(neighbors)
    if not arrays:
        return {"dist_m": None, "span": None, "on_excluded_last_span": False}
    last = n_span - 1
    stop = last if exclude_last and n_span > 1 else n_span
    best = math.inf
    best_span = None
    for s in range(stop):
        for k in range(pts.shape[1]):
            t = float(times[s, k])
            p = pts[s, k]
            for arr in arrays:
                ts = arr[:, 0]
                if t < float(ts[0]) or t > float(ts[-1]):
                    continue
                nb = np.array([
                    np.interp(t, ts, arr[:, 1]),
                    np.interp(t, ts, arr[:, 2]),
                    np.interp(t, ts, arr[:, 3]),
                ], dtype=float)
                dist = float(np.linalg.norm(p - nb))
                if dist < best:
                    best = dist
                    best_span = s
    if best_span is None:
        return {"dist_m": None, "span": None, "on_excluded_last_span": False}
    return {
        "dist_m": float(best),
        "span": int(best_span),
        "on_excluded_last_span": bool(best_span == last),
    }


def _fit_controls(bs, traj, free: np.ndarray, n_span: int, dt: float) -> np.ndarray:
    n_ctrl = n_span + 3
    q = np.zeros((n_ctrl, 3), dtype=float)
    p0, v0, a0 = bs._pva(traj, float(traj.t_start))
    p1, v1, a1 = bs._pva(traj, float(traj.t_end))
    q[0], q[1], q[2] = bs._boundary_controls(p0, v0, a0, dt)
    q[-3], q[-2], q[-1] = bs._boundary_controls(p1, v1, a1, dt)
    fitted, _rmse = bs._fit_free(traj, q, free, n_span, dt)
    return fitted


def hinge_metrics(traj_in, traj_out, neighbors, cfg) -> dict:
    """用后段自己的拟合和 ``_peer_sphere_cost_grad`` 量铰链。函数不在就标未测。"""
    from yopo_l0.backend import bspline as bs

    cost_fn = getattr(bs, "_peer_sphere_cost_grad", None)
    if cost_fn is None or not bool(getattr(cfg, "peer_sphere_enabled", False)):
        return {"measured": False, "activated": False, "end_pinned": None}
    duration = float(traj_in.t_end - traj_in.t_start)
    knots = bs._choose_knots(duration)
    if knots is None:
        return {"measured": False, "too_short": True, "activated": False, "end_pinned": True}
    n_span, dt = knots
    n_ctrl = n_span + 3
    segment = bool(getattr(cfg, "swarm_penalty", False) and getattr(cfg, "swarm_segment_penalty", False))
    free = np.arange(3, n_ctrl if segment else n_ctrl - 3)
    q_in = _fit_controls(bs, traj_in, free, n_span, dt)
    q_out = _fit_controls(bs, traj_out, free, n_span, dt)
    if not np.all(np.isfinite(q_in)) or not np.all(np.isfinite(q_out)):
        return {"measured": False, "activated": False, "end_pinned": None, "nonfinite": True}
    radius = float(getattr(cfg, "peer_sphere_radius_m", RADIUS_M))
    weight = float(getattr(cfg, "peer_sphere_weight", WEIGHT))
    nbs = tuple(_neighbor_arrays(neighbors))
    t0 = float(traj_in.t_start)
    cost_in, grad_in = cost_fn(q_in, dt, t0, nbs, radius, weight)
    cost_out, _grad_out = cost_fn(q_out, dt, t0, nbs, radius, weight)
    close = closest_span(q_in, dt, t0, nbs)
    penalized = closest_span(q_in, dt, t0, nbs, exclude_last=True)
    end_pinned = bool(free.size == 0 or int(np.max(free)) < n_ctrl - 3)
    end_shift = float(np.linalg.norm(q_out[-3:] - q_in[-3:]))
    free_shift = float(np.linalg.norm((q_out[free] - q_in[free]).ravel())) if free.size else 0.0
    pinned_grad = float(np.linalg.norm(grad_in[-3:]))
    return {
        "measured": True,
        "activated": bool(float(cost_in) > 0.0 or float(cost_out) > 0.0),
        "cost": float(cost_in),
        "cost_after": float(cost_out),
        "min_dist_m": close["dist_m"],
        "penalized_min_dist_m": penalized["dist_m"],
        "closest_span": close["span"],
        "closest_on_excluded_last_span": bool(close["on_excluded_last_span"]),
        "end_pinned": end_pinned,
        "end_shift_m": end_shift,
        "free_shift_m": free_shift,
        "pinned_grad_norm": pinned_grad,
    }


def _in_window(t, event_t: float | None, window: float = WINDOW_S) -> bool:
    if event_t is None or t is None:
        return False
    return (float(event_t) - window - 1e-9) <= float(t) <= float(event_t) + 1e-9


def pair_event(ticks, pair=PAIR, ball: float = BALL_M) -> dict:
    """第一次中心距 <= 碰撞球的时刻。没撞则取全程最小距离的时刻。"""
    a, b = pair
    by_t: dict[float, dict] = {}
    order: list[float] = []
    for row in ticks or ():
        if row.get("uav") not in (a, b):
            continue
        t = float(row["t"])
        bucket = by_t.get(t)
        if bucket is None:
            bucket = {}
            by_t[t] = bucket
            order.append(t)
        bucket[row["uav"]] = row
    best_t = None
    best_d = math.inf
    first = None
    for t in order:
        bucket = by_t[t]
        if a not in bucket or b not in bucket:
            continue
        dist = float(np.linalg.norm(np.asarray(bucket[a]["p"], float) - np.asarray(bucket[b]["p"], float)))
        if dist < best_d:
            best_d = dist
            best_t = t
        if first is None and dist <= float(ball) + 1e-9:
            first = (t, dist)
    if first is not None:
        t, dist = first
        collided = True
    else:
        t, dist = best_t, (None if best_t is None else best_d)
        collided = False
    return {"t": t, "dist_m": dist, "collided": collided, "pair": [a, b]}


def attach_plans(event: dict, plans: list[dict]) -> dict:
    out = dict(event)
    uavs = {}
    t = event.get("t")
    for uav in event.get("pair") or PAIR:
        if t is None:
            uavs[uav] = {"code": None, "traj_t0": None, "traj_t1": None}
            continue
        prev = [p for p in plans if p.get("uav") == uav and p.get("t") is not None and float(p["t"]) <= float(t) + 1e-12]
        if not prev:
            uavs[uav] = {"code": None, "traj_t0": None, "traj_t1": None}
            continue
        last = max(prev, key=lambda p: float(p["t"]))
        uavs[uav] = {
            "code": last.get("code"),
            "traj_t0": last.get("traj_t0"),
            "traj_t1": last.get("traj_t1"),
        }
    out["uavs"] = uavs
    return out


def summarize_neighbors(plans, refines, notes, event_t) -> dict:
    first = [p for p in plans if p.get("is_first")]
    window_plans = [p for p in plans if _in_window(p.get("t"), event_t)]
    window_refines = [r for r in refines if _in_window(r.get("t"), event_t)]
    with_nb = [r for r in window_refines if int(r.get("n_neighbors") or 0) > 0]
    block = {
        "first_plan_count": len(first),
        "first_plan_t": None if not first else min(float(p["t"]) for p in first),
        "first_plan_missing_neighbors": bool(first) and all(int(p.get("n_neighbors") or 0) == 0 for p in first),
        "notes_before_any_plan": sum(1 for n in notes if int(n.get("n_plans_done") or 0) == 0),
        "window_s": WINDOW_S,
        "window_plan_count": len(window_plans),
        "window_plans_missing_neighbors": sum(1 for p in window_plans if int(p.get("n_neighbors") or 0) == 0),
        "window_refine_calls": len(window_refines),
        "window_refine_with_neighbors": len(with_nb),
        "backend_received_neighbors": bool(with_nb),
    }
    if block["first_plan_missing_neighbors"]:
        head = "第 0 个规划拍没有邻机。"
    elif not first:
        head = "没有规划记录。"
    else:
        head = "第 0 个规划拍已经有邻机。"
    if not window_refines:
        tail = f"事件前 {WINDOW_S:.1f} s 内没有后段调用。"
    elif block["backend_received_neighbors"]:
        tail = (
            f"事件前 {WINDOW_S:.1f} s 内后段收到邻机的调用 "
            f"{block['window_refine_with_neighbors']}/{block['window_refine_calls']}。"
        )
    else:
        tail = f"事件前 {WINDOW_S:.1f} s 内后段没有收到邻机采样（refine {block['window_refine_calls']} 次）。"
    if block["notes_before_any_plan"]:
        tail += f" 第一次 plan 之前已经有 {block['notes_before_any_plan']} 条邻机采样。"
    else:
        tail += " 第一次 plan 之前没有邻机采样。"
    block["reading"] = head + tail
    return block


def _hinge_of(row: dict) -> dict:
    hinge = row.get("hinge") or {}
    return hinge if hinge.get("measured") else {}


def summarize_hinge(refines) -> dict:
    measured = [r for r in refines if (r.get("hinge") or {}).get("measured")]
    active = [r for r in measured if r["hinge"].get("activated")]

    def vals(rows, key):
        out = []
        for row in rows:
            value = row["hinge"].get(key)
            if value is not None:
                out.append(float(value))
        return out

    end_shifts = vals(measured, "end_shift_m")
    free_shifts = vals(measured, "free_shift_m")
    costs = vals(active, "cost")
    active_dist = vals(active, "min_dist_m")
    all_dist = vals(measured, "min_dist_m")
    pinned_grads = vals(measured, "pinned_grad_norm")
    if not measured:
        end_pinned = None
    else:
        end_pinned = all(bool(r["hinge"].get("end_pinned")) for r in measured)
    last_span = [r for r in measured if r["hinge"].get("closest_on_excluded_last_span")]
    inside = [
        r for r in measured
        if r["hinge"].get("min_dist_m") is not None and float(r["hinge"]["min_dist_m"]) < BALL_M
    ]
    blocked = False
    if inside and all(bool(r["hinge"].get("end_pinned")) for r in inside):
        shifts = [float(r["hinge"].get("end_shift_m") or 0.0) for r in inside]
        if max(shifts) <= END_SHIFT_EPS:
            blocked = any(
                bool(r["hinge"].get("closest_on_excluded_last_span"))
                or float(r["hinge"].get("pinned_grad_norm") or 0.0) > PINNED_GRAD_EPS
                for r in inside
            )
    block = {
        "measured_calls": len(measured),
        "activated": bool(active),
        "activated_calls": len(active),
        "min_dist_when_active_m": None if not active_dist else min(active_dist),
        "min_dist_all_m": None if not all_dist else min(all_dist),
        "max_cost": None if not costs else max(costs),
        "max_free_shift_m": None if not free_shifts else max(free_shifts),
        "max_end_shift_m": None if not end_shifts else max(end_shifts),
        "end_pinned": end_pinned,
        "closest_on_excluded_last_span_calls": len(last_span),
        "max_pinned_grad_norm": None if not pinned_grads else max(pinned_grads),
        "pinned_end_blocks_push": bool(blocked),
    }
    if not measured:
        text = "窗口内没有可测的欧氏球调用。"
    else:
        if block["activated"]:
            text = (
                f"铰链激活 {block['activated_calls']} 次，"
                f"激活时最小三维距 {_fmt(block['min_dist_when_active_m'])} m，"
                f"最大代价 {_fmt(block['max_cost'])}，"
                f"自由控制点最大位移 {_fmt(block['max_free_shift_m'])} m。"
            )
        else:
            text = "铰链没有激活（代价为 0）。"
        if block["end_pinned"]:
            text += f"末端三个控制点钉死，末端位移最大 {_fmt(block['max_end_shift_m'])} m。"
        elif block["end_pinned"] is False:
            text += "末端控制点不在钉死集里。"
        if block["pinned_end_blocks_push"]:
            text += "球内的最近点落在钉死末端或被排除的最后一跨上，末端没有位移。"
        elif block["closest_on_excluded_last_span_calls"]:
            text += f"有 {block['closest_on_excluded_last_span_calls']} 次最近点在被排除的最后一跨。"
    block["reading"] = text
    return block


def summarize_collision(event: dict, *, empty_brake: bool) -> dict:
    uavs = event.get("uavs") or {}
    locus = {}
    codes = {}
    for uav, info in uavs.items():
        code = info.get("code")
        codes[uav] = code
        locus[uav] = locus_of(event.get("t"), info.get("traj_t0"), info.get("traj_t1"), code)
    hover = [u for u, code in codes.items() if code == "CORRIDOR_EMPTY_HOVER"]
    flying = [u for u, code in codes.items() if code not in ("CORRIDOR_EMPTY_HOVER", "ARRIVED_HOLD", None)]
    one_side = len(hover) == 1 and len(flying) == 1
    block = {
        "collided": bool(event.get("collided")),
        "t": event.get("t"),
        "dist_m": event.get("dist_m"),
        "pair": list(event.get("pair") or PAIR),
        "locus": locus,
        "plan_code": codes,
        "empty_brake_enabled": bool(empty_brake),
        "one_side_hover_other_flying": bool(one_side),
        "hover_uav": hover[0] if len(hover) == 1 else None,
        "flying_uav": flying[0] if len(flying) == 1 else None,
    }
    if block["t"] is None:
        block["reading"] = "没有这对飞机的同时刻位置。"
        return block
    kind = "碰撞" if block["collided"] else "最近接近"
    loc = "，".join(f"{u}={locus[u]}" for u in locus)
    code = "，".join(f"{u}={codes[u]}" for u in codes)
    side = "一侧是 CORRIDOR_EMPTY_HOVER、另一侧仍在飞。" if one_side else "不是一侧悬停、另一侧飞过来。"
    brake = "empty_brake 开着。" if block["empty_brake_enabled"] else "empty_brake 关着。"
    block["reading"] = (
        f"{kind} t={_fmt(block['t'], 2)} s，距离 {_fmt(block['dist_m'])} m。"
        f"轨迹位置 {loc}。规划码 {code}。{side}{brake}"
    )
    return block


def _gap_stats(rows, key: str = "pre_ref_min_m") -> dict:
    gaps = [float(r[key]) for r in rows if r.get(key) is not None]
    below = [g for g in gaps if g < BALL_M]
    return {
        "samples": len(gaps),
        "min_m": None if not gaps else min(gaps),
        "n_below_ball": len(below),
        "already_below_ball": bool(below),
    }


def summarize_geom(refines, window_refines) -> dict:
    gated = [r for r in refines if r.get("geom_on")]
    window_gated = [r for r in window_refines if r.get("geom_on")]
    any_stats = _gap_stats(refines)
    gated_stats = _gap_stats(gated)
    window_stats = _gap_stats(window_gated)
    block = {
        "gated_samples": gated_stats["samples"],
        "gated_min_reference_m": gated_stats["min_m"],
        "gated_n_below_ball": gated_stats["n_below_ball"],
        "gated_reference_already_below_ball": gated_stats["already_below_ball"],
        "window_gated_samples": window_stats["samples"],
        "window_gated_min_reference_m": window_stats["min_m"],
        "window_gated_reference_already_below_ball": window_stats["already_below_ball"],
        "any_samples": any_stats["samples"],
        "any_reference_min_m": any_stats["min_m"],
        "any_n_below_ball": any_stats["n_below_ball"],
        "any_reference_below_ball": any_stats["already_below_ball"],
    }
    if block["gated_samples"] == 0:
        text = "几何门在这次运行里没有打开过（geom_multi 要等邻机采样；第 0 拍没有）。"
    elif block["gated_reference_already_below_ball"]:
        text = (
            f"几何门放行的参考轨迹有 {block['gated_n_below_ball']} 次同时刻距离已经 < {BALL_M:.2f} m，"
            f"最小 {_fmt(block['gated_min_reference_m'])} m。"
        )
    else:
        text = f"几何门放行的参考轨迹没有进 {BALL_M:.2f} m 球，最小 {_fmt(block['gated_min_reference_m'])} m。"
    if block["any_reference_below_ball"]:
        text += f" 不论门开没开，参考轨迹自身最小 {_fmt(block['any_reference_min_m'])} m，已经 < {BALL_M:.2f} m。"
    block["reading"] = text
    return block


def answers_for(log: _Log, ticks, *, empty_brake: bool) -> dict:
    event = attach_plans(pair_event(ticks), log.plans)
    window_refines = [r for r in log.refines if _in_window(r.get("t"), event.get("t"))]
    return {
        "1": summarize_neighbors(log.plans, log.refines, log.notes, event.get("t")),
        "2": summarize_hinge(window_refines),
        "3": summarize_collision(event, empty_brake=empty_brake),
        "4": summarize_geom(log.refines, window_refines),
    }


def compare_cases(cases: list[dict]) -> dict:
    by = {(c["arm"], int(c["seed"])): c for c in cases}
    out = {}
    for arm in ARMS:
        miss = by[(arm, 1)]
        ref = by[(arm, 0)]
        out[arm] = {
            "seed1_outcome": miss["outcome"],
            "seed1_reason": miss["reason"],
            "seed1_min_inter_m": miss["min_inter_drone_distance_m"],
            "seed0_outcome": ref["outcome"],
            "seed0_min_inter_m": ref["min_inter_drone_distance_m"],
            "seed1_collided": miss["answers"]["3"]["collided"],
            "seed0_collided": ref["answers"]["3"]["collided"],
            "both_first_plan_missing_neighbors": bool(
                miss["answers"]["1"]["first_plan_missing_neighbors"]
                and ref["answers"]["1"]["first_plan_missing_neighbors"]
            ),
            "seed1_backend_received_neighbors": miss["answers"]["1"]["backend_received_neighbors"],
            "seed0_backend_received_neighbors": ref["answers"]["1"]["backend_received_neighbors"],
            "seed1_hinge_activated": miss["answers"]["2"]["activated"],
            "seed0_hinge_activated": ref["answers"]["2"]["activated"],
            "seed1_pinned_end_blocks_push": miss["answers"]["2"]["pinned_end_blocks_push"],
            "seed1_one_side_hover": miss["answers"]["3"]["one_side_hover_other_flying"],
            "seed1_geom_reference_below_ball": miss["answers"]["4"]["gated_reference_already_below_ball"],
            "seed1_any_reference_below_ball": miss["answers"]["4"]["any_reference_below_ball"],
        }
    return out


def _question_rows(status: str, reason: str | None, cases: list[dict]) -> list[dict]:
    if status != "diagnosed":
        return [
            {"id": qid, "key": key, "prompt": prompt, "status": "deferred", "answer": None, "reason": reason}
            for qid, key, prompt in QUESTION_SPEC
        ]
    by = {(c["arm"], int(c["seed"])): c["answers"] for c in cases}
    rows = []
    for qid, key, prompt in QUESTION_SPEC:
        slot = str(qid)
        rows.append({
            "id": qid,
            "key": key,
            "prompt": prompt,
            "status": "answered",
            "reason": None,
            "answer": {
                "sphere_seed1": by[("sphere", 1)][slot],
                "sphere_seed0": by[("sphere", 0)][slot],
                "geom_multi_seed1": by[("sphere+track_geom_multi", 1)][slot],
                "geom_multi_seed0": by[("sphere+track_geom_multi", 0)][slot],
            },
        })
    return rows


def _smoke(support: dict) -> dict:
    path = ROOT / SCENE_PATH
    before = path.read_bytes()
    sphere = build_card("sphere")
    geom = build_card("sphere+track_geom_multi")
    after = path.read_bytes()
    ids = [d["id"] for d in sphere.card["drones"]]
    return {
        "scene_loads": True,
        "scene_files_unchanged": before == after,
        "scene_sha256": hashlib.sha256(after).hexdigest(),
        "uav_ids": ids,
        "pair_present": "uav1" in ids and "uav2" in ids,
        "body_radius_m": float(sphere.card["criteria"]["body_radius_m"]),
        "collision_line_m": BALL_M,
        "runner_plan_before_poll": plan_before_poll(),
        "runner_plan_before_poll_is_static": True,
        "in_memory_arms": {"sphere": _arm_flags(sphere), "sphere+track_geom_multi": _arm_flags(geom)},
        "feature": support,
    }


def _deferred_reason(support: dict) -> str:
    if support["supported"]:
        return "--partial 只做冒烟，不跑 014-swap3。去掉 --partial 才逐 tick 诊断 seed 0 和 seed 1。"
    return (
        "main 的 BackendConfig / refine_yopo 还没有可用的 peer_sphere_enabled。"
        "实飞诊断待 PR #94 合入后再跑（不要 --partial）。"
    )


def build_partial_report(support: dict | None = None) -> dict:
    """不调用 ``runner.run``。开关不在时标 ``missing_feature``。"""
    support = detect_support() if support is None else support
    status = "partial_smoke" if support["supported"] else "missing_feature"
    return {
        "ticket": "E-B4d",
        "report_only": True,
        "disclaimer": DISCLAIMER,
        "status": status,
        "flight": "skipped",
        "feature": support,
        "blocked_on": None if support["supported"] else {"pr": PR_NUMBER, "branch": PR_BRANCH},
        "target": target_block(),
        "cited_e_b4c": CITED,
        "questions": _question_rows(status, _deferred_reason(support), []),
        "comparison": None,
        "cases": [],
        "partial_smoke": _smoke(support),
    }


def _install(restore: _Restore, log: _Log) -> None:
    """欧氏球打开时把邻机送进后段，并记下每次 plan / refine。用完由 restore 还原。"""
    import yopo_l0.backend.bspline as bs
    import yopo_l0.crossing as crossing_mod
    import yopo_l0.policy as policy_mod
    import yopo_l0.runner as runner_mod

    ctx = {"uav": None, "geom_on": False}

    def sphere_on(params, **kwargs):
        from yopo_l0.backend.config import config_from_params

        cfg = config_from_params(params, a_max=float(kwargs.get("a_max", 6.0)))
        return cfg is not None and bool(getattr(cfg, "peer_sphere_enabled", False))

    orig_enabled = runner_mod.swarm_penalty_enabled

    def enabled(params, **kwargs):
        if orig_enabled(params, **kwargs):
            return True
        return bool(sphere_on(params, **kwargs))

    restore.set(runner_mod, "swarm_penalty_enabled", enabled)

    orig_keep = policy_mod.UpstreamPolicy._keep_neighbor_samples

    def keep(self):
        if orig_keep(self):
            return True
        cfg = getattr(self, "_backend", None)
        return cfg is not None and bool(getattr(cfg, "peer_sphere_enabled", False))

    restore.set(policy_mod.UpstreamPolicy, "_keep_neighbor_samples", keep)

    def apply(self, traj, goal):
        from yopo_l0.backend.bspline import refine_yopo

        cfg = self._backend
        use = bool(getattr(self, "_swarm", False)) or (
            cfg is not None and bool(getattr(cfg, "peer_sphere_enabled", False))
        )
        neighbors = list(self._neighbors.values()) if use else None
        out, rec = refine_yopo(traj, cfg, self._obstacles, goal, neighbors=neighbors)
        self.backend_log.append(rec)
        return out

    restore.set(policy_mod.UpstreamPolicy, "_apply_backend", apply)

    def cross_apply(self, traj, goal):
        from yopo_l0.backend.bspline import refine_yopo
        from yopo_l0.inter_drone import committed_neighbor_samples

        cfg = self._backend
        neighbors = None
        wants = cfg is not None and (
            bool(getattr(cfg, "swarm_penalty", False)) or bool(getattr(cfg, "peer_sphere_enabled", False))
        )
        if wants and getattr(self, "_score_node", None) is not None:
            neighbors = committed_neighbor_samples(self._score_node, float(traj.t_start))
        out, rec = refine_yopo(traj, cfg, self._backend_obstacles, goal, neighbors=neighbors)
        self.backend_log.append(rec)
        return out

    restore.set(crossing_mod.CrossingFleet, "_apply_backend", cross_apply)

    orig_refine = bs.refine_yopo

    def wrapped_refine(traj, cfg, obstacles, goal=None, neighbors=None):
        nbs = list(neighbors) if neighbors else []
        try:
            pre = polynomial_gap(traj, nbs)
        except Exception:
            pre = None
        out, rec = orig_refine(traj, cfg, obstacles, goal, neighbors=neighbors)
        try:
            post = polynomial_gap(out, nbs)
        except Exception:
            post = None
        try:
            hinge = hinge_metrics(traj, out, nbs, cfg)
        except Exception as exc:
            hinge = {"measured": False, "activated": False, "end_pinned": None, "error": type(exc).__name__}
        log.refines.append({
            "uav": ctx["uav"],
            "t": float(traj.t_start),
            "n_neighbors": len(_neighbor_arrays(nbs)),
            "neighbors_is_none": neighbors is None,
            "geom_on": bool(ctx["geom_on"]),
            "pre_ref_min_m": pre,
            "post_ref_min_m": post,
            "applied": bool(rec.get("applied")),
            "reason": rec.get("reason"),
            "hinge": hinge,
        })
        return out, rec

    restore.set(bs, "refine_yopo", wrapped_refine)

    orig_plan = policy_mod.UpstreamPolicy.plan
    seen: set[str] = set()

    def plan(self, obs):
        keys = sorted(getattr(self, "_neighbors", {}))
        geom_fn = getattr(self, "_track_geom_now", None)
        geom_on = bool(geom_fn()) if geom_fn is not None else False
        owner = self.ctx.owner
        first = owner not in seen
        seen.add(owner)
        ctx["uav"] = owner
        ctx["geom_on"] = geom_on
        try:
            out = orig_plan(self, obs)
        finally:
            log.n_plans += 1
            ctx["uav"] = None
            ctx["geom_on"] = False
        traj = getattr(out, "traj", None)
        log.plans.append({
            "uav": owner,
            "t": float(obs.now),
            "code": self.last_code,
            "n_neighbors": len(keys),
            "neighbor_ids": keys,
            "is_first": first,
            "geom_on": geom_on,
            "traj_t0": None if traj is None else float(traj.t_start),
            "traj_t1": None if traj is None else float(traj.t_end),
        })
        return out

    restore.set(policy_mod.UpstreamPolicy, "plan", plan)

    orig_note = policy_mod.UpstreamPolicy.note_neighbor_samples

    def note(self, src, samples):
        arr = np.asarray(samples, dtype=float)
        t0 = None
        if arr.ndim == 2 and arr.shape[0] and arr.shape[1] >= 1:
            t0 = float(arr[0, 0])
        log.notes.append({
            "dest": self.ctx.owner,
            "src": str(src),
            "t0": t0,
            "n_plans_done": int(log.n_plans),
        })
        return orig_note(self, src, samples)

    restore.set(policy_mod.UpstreamPolicy, "note_neighbor_samples", note)


def run_one(arm: str, seed: int) -> dict:
    from yopo_l0.runner import RunConfig, run

    log = _Log()
    card = build_card(arm)
    restore = _Restore()
    try:
        _install(restore, log)
        out = run(card, RunConfig(seed=int(seed), shadow=True))
    finally:
        restore.restore()
    empty = bool(out.result.get("empty_brake", False))
    return {
        "arm": arm,
        "seed": int(seed),
        "role": "miss" if int(seed) == 1 else "success_reference",
        "outcome": out.result.get("outcome"),
        "reason": out.result.get("reason"),
        "arrived": out.result.get("arrived"),
        "n_drones": out.result.get("n_drones"),
        "min_inter_drone_distance_m": out.result.get("min_inter_drone_distance_m"),
        "sim_duration_s": out.result.get("sim_duration_s"),
        "arrival_time_s": out.result.get("arrival_time_s"),
        "empty_brake": empty,
        "answers": answers_for(log, out.ticks, empty_brake=empty),
    }


def diagnose_flight(support: dict | None = None) -> dict:
    live = detect_support()
    if not live["supported"]:
        raise MissingFeature("peer_sphere_enabled 不可用，拒绝实飞。用 --partial。")
    if support is None:
        support = live
    cases = [run_one(arm, seed) for arm in ARMS for seed in SEEDS]
    return {
        "ticket": "E-B4d",
        "report_only": True,
        "disclaimer": DISCLAIMER,
        "status": "diagnosed",
        "flight": "ran",
        "feature": support,
        "blocked_on": None,
        "target": target_block(),
        "cited_e_b4c": CITED,
        "questions": _question_rows("diagnosed", None, cases),
        "comparison": compare_cases(cases),
        "cases": cases,
        "partial_smoke": None,
    }


def build_flight_report(support: dict | None = None) -> dict:
    return diagnose_flight(support)


def _cited_lines() -> list[str]:
    lines = [
        "数字引自 PR #94 的 `docs/research/e-b4c-sphere-avoid.md`，本分支没有重测。",
        "",
        "| 臂 | seed | 结果 | 最小机间 m | 到达时间 s |",
        "|---|---|---|---|---|",
    ]
    rows = (
        ("sphere", "0", CITED["sphere"]["seed0"]),
        ("sphere", "1", CITED["sphere"]["seed1"]),
        ("sphere+track_geom_multi", "0", CITED["sphere_track_geom_multi"]["seed0"]),
        ("sphere+track_geom_multi", "1", CITED["sphere_track_geom_multi"]["seed1"]),
    )
    for arm, seed, row in rows:
        lines.append(
            f"| {arm} | {seed} | {row['outcome']} | {row['min_inter_m']} | {row.get('arrival_time_s', '—')} |"
        )
    return lines


def _question_section(report: dict) -> list[str]:
    lines = ["## 四问", ""]
    for row in report["questions"]:
        lines.append(f"{row['id']}. {row['prompt']}")
        lines.append("")
        if row["status"] != "answered" or not row.get("answer"):
            lines.append(f"状态：`{row['status']}`。{row.get('reason') or ''}")
            lines.append("")
            continue
        answer = row["answer"]
        for label, key in (
            ("sphere seed 1", "sphere_seed1"),
            ("sphere seed 0", "sphere_seed0"),
            ("geom_multi seed 1", "geom_multi_seed1"),
            ("geom_multi seed 0", "geom_multi_seed0"),
        ):
            reading = (answer.get(key) or {}).get("reading")
            if reading:
                lines.append(f"- {label}：{reading}")
        lines.append("")
    return lines


def render_markdown(report: dict) -> str:
    status = report["status"]
    lines = [
        "# 票 E-B4d：014-swap3 seed 1 欧氏球未推开",
        "",
        "report-only。只报数。不改 `yopo_l0/`、`core/`、`scenes/`，不冻结，不进 `scenes/suite.txt`。",
        "",
        report["disclaimer"],
        "",
        f"状态：`{status}`。实飞：`{report['flight']}`。",
        "",
    ]
    if status == "missing_feature":
        blocked = report.get("blocked_on") or {}
        lines += [
            "## 为什么这次没有逐 tick 的数",
            "",
            "本分支从 main 拉出。`yopo_l0.backend.config.BackendConfig` 没有 `peer_sphere_enabled`，"
            "`refine_yopo` 也不读这个字段，`config_from_params` 不会把它打开。"
            f"按票面跳过 `ego-density-014-swap3` 的实飞，JSON 标 `missing_feature`。"
            "不把 PR "
            f"[#{blocked.get('pr', PR_NUMBER)}](https://github.com/unununnnn/yopo-restart/pull/{blocked.get('pr', PR_NUMBER)})"
            f"（`{blocked.get('branch', PR_BRANCH)}`）的后段源码拷进本 PR。",
            "",
            "待该 PR 合入 main 之后，在合入后的树上去掉 `--partial` 重跑，才会记下 seed 1 和 seed 0 的逐 tick。",
            "",
        ]
    elif status == "partial_smoke":
        lines += [
            "## 冒烟",
            "",
            "开关已经能用。`--partial` 仍不飞，只确认探测和进程内卡片。去掉 `--partial` 才跑 seed 0 与 seed 1。",
            "",
        ]
    lines += ["## 引自 E-B4c，本分支未重测", ""]
    lines += _cited_lines()
    lines += ["", "seed 1 两臂都是 `uav1-uav2` 相撞，最小距 0.694 m，贴着 0.70 m 碰撞球。叠 `geom_multi` 没有解开。", ""]
    lines += _question_section(report)
    if status == "diagnosed" and report.get("comparison"):
        lines += ["## 对照", "", "| 臂 | seed 1 结果 | seed 1 最小机间 m | seed 0 结果 | seed 0 最小机间 m |", "|---|---|---|---|---|"]
        for arm, row in report["comparison"].items():
            lines.append(
                f"| {arm} | {row['seed1_outcome']} | {_fmt(row['seed1_min_inter_m'])} | "
                f"{row['seed0_outcome']} | {_fmt(row['seed0_min_inter_m'])} |"
            )
        lines.append("")
    smoke = report.get("partial_smoke")
    if smoke:
        ids = "、".join(smoke["uav_ids"])
        if smoke["runner_plan_before_poll"]:
            order = "运行器源码里，`plan` 在 `bus.poll` 之前"
        else:
            order = "运行器源码里，`plan` 不在 `bus.poll` 之前"
        lines += [
            "## 本次冒烟",
            "",
            f"- 场景卡能读，飞机是 {ids}，机体半径 {_fmt(smoke['body_radius_m'], 2)} m，碰撞线 {_fmt(smoke['collision_line_m'], 2)} m。",
            f"- 场景文件未改（sha256 `{smoke['scene_sha256']}`）。",
            f"- {order}。这是静态检查，不是 seed 1 的逐 tick 记录。",
            "- 进程内卡片写了 `peer_sphere_enabled`，半径 0.85 m、权重 8。当前 `config_from_params` 不认这个字段时，不会拿它去飞。",
            "",
        ]
    lines += [
        "## 开关可用时脚本记什么",
        "",
        "进程内包一层，和 E-B4c 对照脚本一样：欧氏球打开时把邻机采样送进 `refine_yopo`，椭球项仍关。"
        "不改 `policy.py`。然后对 seed 0（成功参照）和 seed 1（`uav1-uav2`）各跑 `sphere` 与 `sphere+track_geom_multi`。",
        "",
        "- 每次 `plan` 记下当时 `_neighbors` 是否为空。第 0 拍应为空：本 tick 的广播要等全部 `plan` 返回后才 `bus.poll`。",
        "- 每次 `refine_yopo` 记下传入的邻机份数。撞前 1.0 s（没撞则是最近时刻前 1.0 s）里有没有非空采样。",
        "- 铰链用后段自己的 `_peer_sphere_cost_grad` 量。记三维距、代价、自由控制点位移、末端三控制点位移。"
        "末端钉死且球内最近点落在被排除的最后一跨（或梯度打在钉死的控制点上）时，标末端推不动。",
        "- 碰撞时刻相对当时轨迹：头 / 中 / 尾。`CORRIDOR_EMPTY_HOVER` 且轨迹短于 0.05 s 标 `hover_hold`。"
        "本票的两臂都不开 `empty_brake`。",
        "- `geom_multi` 只看参考轨迹对障碍表面的净空，不看机间距离。记下几何门打开时放行的参考多项式与邻机的同时刻最小距，看它是不是已经 < 0.70 m。",
        "",
        "## 怎么重跑",
        "",
        "```bash",
        ".venv/bin/python scripts/e-b4d-sphere-miss.py --partial --write",
        ".venv/bin/python scripts/e-b4d-sphere-miss.py --write",
        ".venv/bin/python -m pytest tests/test_e_b4d_sphere_miss.py",
        "```",
        "",
        "`--partial` 永远不飞。第二条只在 `peer_sphere_enabled` 可用时跑完四粒并覆盖这份报告。",
        "",
    ]
    return "\n".join(lines)


def write_report(report: dict, json_path: Path, md_path: Path) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    md_path.write_text(render_markdown(report), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="票 E-B4d：014-swap3 seed 1 欧氏球未推开，只报数")
    parser.add_argument("--partial", action="store_true", help="最小冒烟。不跑实飞；开关不在时标 missing_feature")
    parser.add_argument("--write", action="store_true", help="把 JSON 和 Markdown 写到 docs/research")
    parser.add_argument("--json", default=str(JSON_PATH))
    parser.add_argument("--md", default=str(MD_PATH))
    args = parser.parse_args(argv)
    support = detect_support()
    if args.partial or not support["supported"]:
        report = build_partial_report(support)
    else:
        report = build_flight_report(support)
    if args.write:
        write_report(report, Path(args.json), Path(args.md))
    else:
        print(json.dumps({"status": report["status"], "flight": report["flight"], "report_only": True}, ensure_ascii=False))
    print(f"status={report['status']} flight={report['flight']} report-only", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
