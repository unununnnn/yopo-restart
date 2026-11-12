#!/usr/bin/env python3
"""票 E-C5：走廊准入 safe_radius 对照。只报数。

在密度 0.14 的三机对向（d014-swap3）和单机穿越上，seeds 0–9，对照
safe_radius ∈ {0.05, 0.20, 0.35}，每一档再配 E-A1 关，或 E-A1 阈值 1.0 m
加上 scripts/e-b4-peer-diag.py 里的刹停原型（_proto_brake）。

0.05 m 是上游 d5e1e60 的默认，不标偏离。0.20 m 和 0.35 m 标「偏离上游」。
半径走 RunConfig.safe_radius_m，不写进场景卡。0.05 m 臂不传这个字段，
让运行器走缺省。E-A1 和刹停只在进程里，不改 policy.py 的默认悬停。

撞树归因（互斥，按这个顺序）：

1. 跟踪误差：最近 0.5 s 参考点到树的净空仍大于机体半径，机体已经进去。
2. 准入松：这 0.5 s 里有一段已准入的飞行计划，参考点净空 ≤ 机体半径。
3. 深度感知盲区：订下撞击前最后一条计划时，深度图里没有这件障碍。
4. 无解刹停后撞 / 无解悬停后撞：走廊空了，参考是刹停原型或上游原地悬停。
5. 其他。

深度图里没有这件障碍时，不覆盖已经判成跟踪或准入松的主因，只记在事实上。
这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。
不改 core/、已冻门、scenes/suite.txt、docs/tickets.md、README，也不改任何默认值。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import sys
import time
from pathlib import Path

# 每个进程一条推理线程。轨迹不靠墙钟，多进程只缩短整批时间。
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-c5-corridor-radius" / "partial"
GEOM_NAME = "ego-density-014.json"
GEOM_SHA256 = "5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9"

SCENES = {
    "d014-swap3": "scenes/ego-density-014-swap3.json",
    "d014-solo": "scenes/ego-density-014-solo.json",
}
SCENE_ORDER = ("d014-swap3", "d014-solo")
SCENE_LABEL = {"d014-swap3": "014 对向", "d014-solo": "014 单机"}
RADII = (0.05, 0.20, 0.35)
A1_ORDER = ("off", "fix")
SEEDS = tuple(range(10))
BODY_RADIUS_M = 0.35
UPSTREAM_SAFE_RADIUS_M = 0.05
INTER_LIMIT_M = 0.70

DISCLAIMER = (
    "只报数。不改默认 safe_radius 0.05 m。0.20 m 与 0.35 m 偏离上游 d5e1e60。"
    "刹停是 scripts/e-b4-peer-diag.py 的进程内原型，不是已合入开关。"
    "这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。"
)

CAT_LOOSE = "准入松"
CAT_TRACK = "跟踪误差"
CAT_BLIND = "深度感知盲区"
CAT_BRAKE = "无解刹停后撞"
CAT_HOVER = "无解悬停后撞"
CAT_OTHER = "其他"
CAT_INTER_OFF = "机间过滤未开"
CAT_INTER_ON = "机间过滤未挡住"
CAT_TIMEOUT_STALL = "超时卡死"
CAT_TIMEOUT_OTHER = "超时仍在前进"
TREE_ORDER = (CAT_TRACK, CAT_LOOSE, CAT_BLIND, CAT_BRAKE, CAT_HOVER, CAT_OTHER)

_HOVER_CODES = frozenset({
    "CORRIDOR_EMPTY_HOVER",
    "BRAKE_ONLY",
    "CONFLICT_BRAKE",
    "YOPO_KIN_FALLBACK",
    "YOPO_FAIL",
    "ARRIVE_KIN_FALLBACK",
})
_FLY_CODES = frozenset({"OK", "YOPO_ALT"})


def deviates(radius: float) -> bool:
    from yopo_l0.policy import deviates_from_upstream

    return bool(deviates_from_upstream(float(radius)))


def radius_label(radius: float) -> str:
    text = f"safe_radius {float(radius):.2f} m"
    if deviates(radius):
        text += "（偏离上游）"
    return text


def a1_label(a1: str) -> str:
    if a1 == "off":
        return "E-A1 关"
    if a1 == "fix":
        return "E-A1 1.0 m + 刹停原型"
    raise ValueError(a1)


def arm_label(radius: float, a1: str) -> str:
    return f"{radius_label(radius)}，{a1_label(a1)}"


def load_eb4():
    """加载 E-B4 诊断脚本。刹停用它的 _proto_brake，不在本文件再写一份。"""
    name = "e_b4_peer_diag_for_c5"
    if name in sys.modules:
        mod = sys.modules[name]
    else:
        path = ROOT / "scripts" / "e-b4-peer-diag.py"
        spec = importlib.util.spec_from_file_location(name, path)
        if spec is None or spec.loader is None:
            raise SystemExit(f"无法加载 {path}")
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
    mod.SCENES["d014-solo"] = SCENES["d014-solo"]
    return mod


def policy_parent(eb4, proto_brake: bool, log):
    """刹停臂的父类就是 E-B4 的 DiagUpstream，plan() 里调用的是那份 _proto_brake。"""
    import yopo_l0.policy as policy_mod

    base = policy_mod.POLICIES["upstream"]
    if proto_brake:
        log.proto_brake = True
        return eb4._make_policy_cls(base, log)
    return base


def build_card(eb4, scene: str, a1: str):
    if scene not in SCENES:
        raise ValueError(scene)
    eb4.SCENES["d014-solo"] = SCENES["d014-solo"]
    if a1 == "off":
        return eb4.build_card(scene, "off")
    if a1 == "fix":
        return eb4.build_card(scene, "a1", {"upstream": {"inter_drone_threshold_m": 1.0}})
    raise ValueError(a1)


def run_config(radius: float, seed: int):
    """0.05 m 不覆盖，运行器缺省就是上游 0.05 m。其余半径只进 RunConfig。"""
    from yopo_l0.runner import RunConfig

    kwargs = {"seed": int(seed), "shadow": True}
    if deviates(radius):
        kwargs["safe_radius_m"] = float(radius)
    return RunConfig(**kwargs)


def install_trace(eb4, proto_brake: bool):
    """只记录走廊准入和深度图。选择仍走原来的 plan()；刹停只在原型臂上替换无解分支。"""
    import yopo_l0.policy as policy_mod
    from yopo_l0.dense_diag import implied_radius_m
    from yopo_swarm_core.candidates import corridor_mu_lo, select_action

    log = eb4._Log()
    parent = policy_parent(eb4, proto_brake, log)
    previous = policy_mod.POLICIES["upstream"]

    class TraceUpstream(parent):
        instances: list = []

        def __init__(self, ctx):
            super().__init__(ctx)
            self.drone_id = ctx.owner
            self.trace: list[dict] = []
            self._last_fwd = None
            orig = self.source.forward_raw

            def wrapped(*args, **kwargs):
                out = orig(*args, **kwargs)
                self._last_fwd = out
                return out

            self.source.forward_raw = wrapped
            TraceUpstream.instances.append(self)

        def plan(self, obs):
            self._last_fwd = None
            out = super().plan(obs)
            n_admitted = None
            tightest = None
            prefilter_r = None
            fwd = self._last_fwd
            if fwd is not None:
                _params, score, radius = fwd
                dec = policy_mod.upstream_decoder(self.p["safe_radius"])
                mu = np.asarray(corridor_mu_lo(radius, dec), dtype=np.float64)
                admitted = mu >= np.float64(dec.safe_mu)
                n_admitted = int(admitted.sum())
                if n_admitted:
                    tightest = float(implied_radius_m(float(mu[admitted].min())))
                action, prefilter_brake = select_action(score, None, mu, float(dec.safe_mu))
                if not prefilter_brake:
                    prefilter_r = float(implied_radius_m(float(mu[int(action)])))
            braked = bool(proto_brake and self.last_code == "CORRIDOR_EMPTY_HOVER")
            state = obs.state
            self.trace.append({
                "drone": self.drone_id,
                "now": float(obs.now),
                "code": self.last_code,
                "p": np.asarray(state.p, dtype=float).copy(),
                "yaw": float(state.yaw),
                "frame": obs.frame,
                "n_admitted": n_admitted,
                "tightest_admitted_radius_m": tightest,
                "prefilter_chosen_radius_m": prefilter_r,
                "proto_braked": braked,
                "safe_radius_m": float(self.p["safe_radius"]),
            })
            return out

    TraceUpstream.__name__ = "TraceUpstream"
    TraceUpstream.instances = []
    policy_mod.POLICIES["upstream"] = TraceUpstream
    return TraceUpstream, previous, log


def restore_trace(previous) -> None:
    import yopo_l0.policy as policy_mod

    policy_mod.POLICIES["upstream"] = previous


def _jsonable(value):
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        return float(value)
    return value


def _rows_for(ticks, drone: str) -> list[dict]:
    return [row for row in ticks if row["uav"] == drone]


def _trace_at(trace: list[dict], t: float) -> dict | None:
    picked = None
    for row in trace:
        if float(row["now"]) <= t + 1e-9:
            picked = row
        else:
            break
    return picked


def _last_fly(trace: list[dict], t: float) -> dict | None:
    picked = None
    for row in trace:
        if float(row["now"]) > t + 1e-9:
            break
        if row.get("code") in _FLY_CODES:
            picked = row
    return picked


def _cylinders(card):
    from yopo_l0.scene import Cylinder, Ground

    upright = []
    other = []
    for ob in card.obstacles:
        if isinstance(ob, Ground):
            continue
        if isinstance(ob, Cylinder) and ob.upright():
            upright.append(ob)
        else:
            other.append(ob)
    return upright, other


def _min_clearance(points: np.ndarray, upright, other) -> tuple[float, object]:
    if len(points) == 0:
        return math.inf, None
    best = math.inf
    best_ob = None
    for ob in upright:
        radial = np.hypot(points[:, 0] - ob.center_xy[0], points[:, 1] - ob.center_xy[1]) - ob.radius
        radial = np.maximum(radial, 0.0)
        dz = np.maximum(np.maximum(ob.z_lo - points[:, 2], 0.0), points[:, 2] - ob.z_hi)
        dist = np.hypot(radial, dz)
        idx = int(np.argmin(dist))
        val = float(dist[idx])
        if val < best:
            best, best_ob = val, ob
    if other:
        for p in points:
            for ob in other:
                val = float(ob.distance(p))
                if val < best:
                    best, best_ob = val, ob
    return best, best_ob


def min_tree_clearance(card, ticks) -> float | None:
    if not ticks:
        return None
    pts = np.asarray([row["p"] for row in ticks], dtype=float)
    dist, _ob = _min_clearance(pts, *_cylinders(card))
    if not math.isfinite(dist):
        return None
    return float(dist)


def _nearest_at(card, position: np.ndarray):
    dist, ob = _min_clearance(np.asarray(position, dtype=float).reshape(1, 3), *_cylinders(card))
    return ob, dist


def _cmd_clearance(card, rows: list[dict]) -> float:
    if not rows:
        return math.inf
    pts = np.asarray([row["cmd_p"] for row in rows], dtype=float)
    dist, _ob = _min_clearance(pts, *_cylinders(card))
    return dist


def _window(rows: list[dict], t_hit: float, span: float = 0.5) -> list[dict]:
    picked = [row for row in rows if t_hit - span - 1e-12 <= float(row["t"]) <= t_hit + 1e-12]
    return picked or (rows[-1:] if rows else [])


def classify_tree(facts: dict) -> str:
    """跟踪 → 准入松 → 深度看不到 → 刹停 / 悬停 → 其他。"""
    body = float(facts.get("body_radius_m", BODY_RADIUS_M))
    plan_c = facts.get("plan_clearance_m")
    actual = facts.get("actual_clearance_m")
    if (
        plan_c is not None
        and actual is not None
        and float(plan_c) > body
        and float(actual) <= body + 1e-9
    ):
        return CAT_TRACK
    fly_c = facts.get("fly_clearance_m")
    if (
        fly_c is not None
        and float(fly_c) <= body + 1e-9
        and facts.get("fly_admitted")
        and facts.get("fly_code") in _FLY_CODES
        and not facts.get("fly_proto_braked")
    ):
        return CAT_LOOSE
    if facts.get("frame_available") and facts.get("in_depth") is False:
        return CAT_BLIND
    if facts.get("proto_braked"):
        return CAT_BRAKE
    if facts.get("plan_code") in _HOVER_CODES:
        return CAT_HOVER
    return CAT_OTHER


def classify_inter(facts: dict) -> str:
    if facts.get("plan_code") in _HOVER_CODES:
        return CAT_HOVER if not facts.get("proto_braked") else CAT_BRAKE
    planned = facts.get("planned_gap_m")
    actual = facts.get("actual_gap_m")
    if planned is not None and actual is not None and float(planned) > INTER_LIMIT_M >= float(actual):
        return CAT_TRACK
    if facts.get("filter_enabled"):
        return CAT_INTER_ON
    return CAT_INTER_OFF


def _depth_flag(card, flown: dict | None, obstacle) -> tuple[bool, bool | None]:
    from yopo_l0.dense_diag import depth_sees
    from yopo_l0.sensor import CameraConfig

    frame = None if flown is None else flown.get("frame")
    if frame is None or obstacle is None:
        return False, None
    cam = CameraConfig.from_card(card.card["camera"])
    seen = bool(depth_sees(
        frame,
        np.asarray(flown["p"], dtype=float),
        float(flown["yaw"]),
        cam,
        obstacle,
    ))
    return True, seen


def _diagnose_tree(card, result, ticks, traces, drone: str) -> dict:
    rows = _rows_for(ticks, drone)
    hit = rows[-1]
    t_hit = float(hit["t"])
    p_hit = np.asarray(hit["p"], dtype=float)
    obstacle, actual = _nearest_at(card, p_hit)
    window = _window(rows, t_hit)
    plan_c = _cmd_clearance(card, window)
    fly_rows = [row for row in window if row.get("plan_code") in _FLY_CODES]
    fly_c = None if not fly_rows else _cmd_clearance(card, fly_rows)
    trace = traces.get(drone) or []
    flown = _trace_at(trace, t_hit)
    fly = _last_fly(trace, t_hit)
    frame_available, in_depth = _depth_flag(card, flown, obstacle)
    _fly_frame, fly_in_depth = _depth_flag(card, fly, obstacle)
    body = float(card.card["criteria"]["body_radius_m"])
    facts = {
        "kind": "tree",
        "drone": drone,
        "t_hit_s": t_hit,
        "obstacle": None if obstacle is None else getattr(obstacle, "name", None),
        "actual_clearance_m": None if not math.isfinite(actual) else float(actual),
        "plan_clearance_m": None if not math.isfinite(plan_c) else float(plan_c),
        "fly_clearance_m": None if fly_c is None or not math.isfinite(fly_c) else float(fly_c),
        "tracking_error_m": float(np.linalg.norm(p_hit - np.asarray(hit["cmd_p"], dtype=float))),
        "speed_mps": float(np.linalg.norm(hit["v"])),
        "in_depth": in_depth,
        "fly_in_depth": fly_in_depth,
        "frame_available": frame_available,
        "plan_code": None if flown is None else flown.get("code"),
        "proto_braked": bool(flown and flown.get("proto_braked")),
        "fly_code": None if fly is None else fly.get("code"),
        "fly_admitted": bool(fly and (fly.get("n_admitted") or 0) > 0),
        "fly_proto_braked": bool(fly and fly.get("proto_braked")),
        "n_admitted": None if flown is None else flown.get("n_admitted"),
        "tightest_admitted_radius_m": None if flown is None else flown.get("tightest_admitted_radius_m"),
        "fly_tightest_admitted_radius_m": None if fly is None else fly.get("tightest_admitted_radius_m"),
        "prefilter_chosen_radius_m": None if flown is None else flown.get("prefilter_chosen_radius_m"),
        "safe_radius_m": float(result.get("safe_radius_m") or UPSTREAM_SAFE_RADIUS_M),
        "body_radius_m": body,
        "tick_plan_code": hit.get("plan_code"),
    }
    facts["category"] = classify_tree(facts)
    return facts


def _diagnose_inter(result, ticks, traces, reason: str) -> dict:
    left, right = reason.split(":", 1)[0].split("-", 1)
    hit_a, hit_b = _rows_for(ticks, left)[-1], _rows_for(ticks, right)[-1]
    t_hit = float(hit_a["t"])
    actual = float(np.linalg.norm(np.asarray(hit_a["p"], dtype=float) - np.asarray(hit_b["p"], dtype=float)))
    planned = float(np.linalg.norm(
        np.asarray(hit_a["cmd_p"], dtype=float) - np.asarray(hit_b["cmd_p"], dtype=float)
    ))
    flown_a = _trace_at(traces.get(left) or [], t_hit)
    flown_b = _trace_at(traces.get(right) or [], t_hit)
    code = None if flown_a is None else flown_a.get("code")
    braked = bool(flown_a and flown_a.get("proto_braked"))
    if code not in _HOVER_CODES and flown_b and flown_b.get("code") in _HOVER_CODES:
        code = flown_b.get("code")
        braked = bool(flown_b.get("proto_braked"))
    filt = result.get("inter_drone_filter") or {}
    facts = {
        "kind": "inter",
        "drone": f"{left}-{right}",
        "drones": [left, right],
        "t_hit_s": t_hit,
        "actual_gap_m": actual,
        "planned_gap_m": planned,
        "tracking_error_m": max(
            float(np.linalg.norm(np.asarray(hit_a["p"], dtype=float) - np.asarray(hit_a["cmd_p"], dtype=float))),
            float(np.linalg.norm(np.asarray(hit_b["p"], dtype=float) - np.asarray(hit_b["cmd_p"], dtype=float))),
        ),
        "speed_mps": max(float(np.linalg.norm(hit_a["v"])), float(np.linalg.norm(hit_b["v"]))),
        "plan_code": code,
        "proto_braked": braked,
        "filter_enabled": bool(filt),
        "threshold_m": filt.get("threshold_m"),
        "safe_radius_m": float(result.get("safe_radius_m") or UPSTREAM_SAFE_RADIUS_M),
    }
    facts["category"] = classify_inter(facts)
    return facts


def _mean_speed(rows: list[dict]) -> float:
    if not rows:
        return 0.0
    return float(np.mean([np.linalg.norm(row["v"]) for row in rows]))


def _progress_m(rows: list[dict]) -> float:
    if len(rows) < 2:
        return 0.0
    return float(rows[0]["goal_error_m"]) - float(rows[-1]["goal_error_m"])


def _drone_arrived(rows: list[dict], gtol: float, gspeed: float, ghold: float) -> bool:
    started = None
    for row in rows:
        t = float(row["t"])
        speed = float(np.linalg.norm(row["v"]))
        if float(row["goal_error_m"]) <= gtol and speed <= gspeed:
            if started is None:
                started = t
            elif t - started >= ghold - 1e-9:
                return True
        else:
            started = None
    return False


def _diagnose_timeout(card, result, ticks, traces) -> dict:
    crit = card.card["criteria"]
    gtol = float(crit["goal_tolerance_m"])
    gspeed = float(crit["goal_speed_mps"])
    ghold = float(crit["goal_hold_s"])
    per = []
    for drone in [d["id"] for d in card.card["drones"]]:
        rows = _rows_for(ticks, drone)
        if not rows or _drone_arrived(rows, gtol, gspeed, ghold):
            continue
        t_end = float(rows[-1]["t"])
        span = 5.0
        window = [row for row in rows if float(row["t"]) >= t_end - span - 1e-12]
        trace = [row for row in (traces.get(drone) or []) if float(row["now"]) >= t_end - span]
        hover_n = sum(1 for row in trace if row.get("code") in _HOVER_CODES or row.get("proto_braked"))
        stalled = bool(
            (_progress_m(window) < 0.5 and _mean_speed(window) < 0.20)
            or (trace and hover_n >= max(1, (len(trace) + 1) // 2))
        )
        per.append({
            "drone": drone,
            "goal_error_m": float(rows[-1]["goal_error_m"]),
            "progress_m": _progress_m(window),
            "mean_speed_mps": _mean_speed(window),
            "hover_plans": hover_n,
            "plans_in_window": len(trace),
            "stalled": stalled,
            "plan_code": None if not trace else trace[-1].get("code"),
            "proto_braked": bool(trace and trace[-1].get("proto_braked")),
        })
    category = CAT_TIMEOUT_STALL if any(item["stalled"] for item in per) else CAT_TIMEOUT_OTHER
    return {"kind": "timeout", "category": category, "per_drone": per}


def diagnose(card, result, ticks, traces) -> dict | None:
    outcome = result.get("outcome")
    reason = str(result.get("reason") or "")
    if outcome == "COLLISION" and "TRUTH_POSITION" in reason:
        return _diagnose_tree(card, result, ticks, traces, reason.split(":", 1)[0])
    if outcome == "COLLISION" and "INTER_DRONE" in reason:
        return _diagnose_inter(result, ticks, traces, reason)
    if outcome == "TIMEOUT":
        return _diagnose_timeout(card, result, ticks, traces)
    return None


def _drop_frames(traces: dict) -> dict[str, list]:
    out = {}
    for drone, rows in traces.items():
        out[drone] = rows
    return out


def _partial_path(scene: str, radius: float, a1: str, seed: int) -> Path:
    return PARTIAL / f"{scene}-r{float(radius):.2f}-{a1}-{int(seed)}.json"


def _traces(cls) -> dict[str, list]:
    out: dict[str, list] = {}
    for inst in getattr(cls, "instances", []):
        out[inst.drone_id] = list(inst.trace)
    return out


def _file_hash(scene: str) -> str:
    from yopo_l0.scene import load_card

    return load_card(ROOT / SCENES[scene]).hash


def run_one(scene: str, radius: float, a1: str, seed: int, *, resume: bool) -> dict:
    dest = _partial_path(scene, radius, a1, seed)
    if resume and dest.is_file():
        return json.loads(dest.read_text(encoding="utf-8"))
    from yopo_l0.runner import run

    eb4 = load_eb4()
    card = build_card(eb4, scene, a1)
    proto = a1 == "fix"
    cfg = run_config(radius, seed)
    diag_cls, previous, _log = install_trace(eb4, proto)
    t0 = time.perf_counter()
    try:
        out = run(card, cfg)
        traces = _traces(diag_cls)
        attr = diagnose(card, out.result, out.ticks, traces)
        tree_m = min_tree_clearance(card, out.ticks)
        proto_brakes = sum(1 for rows in traces.values() for row in rows if row.get("proto_braked"))
    finally:
        restore_trace(previous)
        diag_cls.instances = []
    res = out.result
    got = float(res["safe_radius_m"])
    if abs(got - float(radius)) > 1e-9:
        raise RuntimeError(f"safe_radius 读回 {got}，臂是 {radius}")
    if bool(res["upstream_deviation"]) != deviates(radius):
        raise RuntimeError(f"偏离标记 {res['upstream_deviation']} 与半径 {radius} 不一致")
    reason = str(res["reason"])
    filt = res.get("inter_drone_filter") or {}
    row = {
        "scene": scene,
        "radius_m": float(radius),
        "a1": a1,
        "seed": int(seed),
        "proto_brake": proto,
        "upstream_deviation": bool(res["upstream_deviation"]),
        "safe_radius_m": got,
        "scene_card_hash": res["scene_card_hash"],
        "file_hash": _file_hash(scene),
        "outcome": res["outcome"],
        "reason": reason,
        "arrived": int(res["arrived"]),
        "n_drones": int(res["n_drones"]),
        "tree_collision": int(res["outcome"] == "COLLISION" and "TRUTH_POSITION" in reason),
        "inter_drone_collision": int(res["outcome"] == "COLLISION" and "INTER_DRONE" in reason),
        "timeout": int(res["outcome"] == "TIMEOUT"),
        "min_inter_drone_distance_m": res["min_inter_drone_distance_m"],
        "min_obstacle_distance_m": res["min_obstacle_distance_m"],
        "min_tree_clearance_m": tree_m,
        "sim_duration_s": res["sim_duration_s"],
        "arrival_time_s": res["arrival_time_s"],
        "plan_codes": res.get("plan_codes"),
        "empty_solution_count": res.get("empty_solution_count"),
        "inter_threshold_m": filt.get("threshold_m"),
        "inter_filtered": filt.get("filtered_candidates"),
        "inter_empty_fallbacks": filt.get("empty_fallbacks"),
        "proto_brakes": int(proto_brakes),
        "attribution": attr,
        "wall_time_s": res["wall_time_s"],
        "worker_s": time.perf_counter() - t0,
    }
    row = _jsonable(row)
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)
    gap = row["min_inter_drone_distance_m"]
    gap_s = "—" if gap is None else f"{float(gap):.3f}"
    tree = row["min_tree_clearance_m"]
    tree_s = "—" if tree is None else f"{float(tree):.3f}"
    cat = (row.get("attribution") or {}).get("category") or "—"
    print(
        f"{scene} {arm_label(radius, a1)} seed={seed} {row['outcome']} "
        f"{row['arrived']}/{row['n_drones']} tree={row['tree_collision']} "
        f"inter={row['inter_drone_collision']} gap={gap_s} tree_clear={tree_s} "
        f"sim={float(row['sim_duration_s']):.2f} wall={float(row['wall_time_s']):.1f}s cat={cat}",
        flush=True,
    )
    return row


def _worker(spec: tuple) -> dict:
    try:
        os.nice(10)
    except OSError:
        pass
    scene, radius, a1, seed, resume = spec
    return run_one(scene, float(radius), str(a1), int(seed), resume=bool(resume))


def job_specs(scenes, radii, a1s, seeds, *, resume: bool = True) -> list[tuple]:
    return [
        (scene, float(radius), a1, int(seed), bool(resume))
        for a1 in a1s
        for radius in radii
        for scene in scenes
        for seed in seeds
    ]


def _run_group(specs: list[tuple], jobs: int) -> list[dict]:
    if jobs <= 1 or len(specs) <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)


def _rng(values: list[float]) -> str:
    if not values:
        return "—"
    lo, hi = min(values), max(values)
    if abs(lo - hi) < 5e-4:
        return f"{lo:.3f}"
    return f"{lo:.3f}–{hi:.3f}"


def _group(rows: list[dict], scene: str, radius: float, a1: str) -> list[dict]:
    return [
        r for r in rows
        if r["scene"] == scene and abs(float(r["radius_m"]) - float(radius)) < 1e-9 and r["a1"] == a1
    ]


def render_summary(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | 到达 | 撞树 | 机间碰撞 | 超时 | 最小树净空 m | 最小机间 m |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for scene in SCENE_ORDER:
        for radius in RADII:
            for a1 in A1_ORDER:
                group = _group(rows, scene, radius, a1)
                if not group:
                    continue
                drones = sum(int(r["n_drones"]) for r in group)
                arrived = sum(int(r["arrived"]) for r in group)
                tree = sum(int(r["tree_collision"]) for r in group)
                inter = sum(int(r["inter_drone_collision"]) for r in group)
                timeout = sum(int(r["timeout"]) for r in group)
                trees = [float(r["min_tree_clearance_m"]) for r in group if r.get("min_tree_clearance_m") is not None]
                gaps = [
                    float(r["min_inter_drone_distance_m"])
                    for r in group
                    if r.get("min_inter_drone_distance_m") is not None
                ]
                lines.append(
                    f"| {SCENE_LABEL[scene]} | {arm_label(radius, a1)} | {arrived}/{drones} | "
                    f"{tree} | {inter} | {timeout} | {_rng(trees)} | {_rng(gaps)} |"
                )
    return "\n".join(lines)


def render_seeds(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | seed | 结果 | 原因 | 到达 | 最小树净空 m | 最小机间 m | 仿真 s | 刹停拍 |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for scene in SCENE_ORDER:
        for radius in RADII:
            for a1 in A1_ORDER:
                for row in sorted(_group(rows, scene, radius, a1), key=lambda r: int(r["seed"])):
                    gap = row["min_inter_drone_distance_m"]
                    gap_s = "—" if gap is None else f"{float(gap):.3f}"
                    tree = row["min_tree_clearance_m"]
                    tree_s = "—" if tree is None else f"{float(tree):.3f}"
                    lines.append(
                        f"| {SCENE_LABEL[scene]} | {arm_label(radius, a1)} | {row['seed']} | "
                        f"{row['outcome']} | `{row['reason']}` | {row['arrived']}/{row['n_drones']} | "
                        f"{tree_s} | {gap_s} | {float(row['sim_duration_s']):.2f} | {int(row['proto_brakes'])} |"
                    )
    return "\n".join(lines)


def _tree_rows(rows: list[dict]) -> list[dict]:
    return [r for r in rows if int(r.get("tree_collision") or 0) == 1]


def render_tree_counts(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | 撞树 | 跟踪误差 | 准入松 | 深度感知盲区 | 无解刹停后撞 | 无解悬停后撞 | 其他 |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for scene in SCENE_ORDER:
        for radius in RADII:
            for a1 in A1_ORDER:
                group = [r for r in _group(rows, scene, radius, a1) if int(r.get("tree_collision") or 0) == 1]
                if not _group(rows, scene, radius, a1):
                    continue
                counts = {name: 0 for name in TREE_ORDER}
                for row in group:
                    cat = (row.get("attribution") or {}).get("category")
                    if cat in counts:
                        counts[cat] += 1
                    else:
                        counts[CAT_OTHER] += 1
                lines.append(
                    f"| {SCENE_LABEL[scene]} | {arm_label(radius, a1)} | {len(group)} | "
                    + " | ".join(str(counts[name]) for name in TREE_ORDER)
                    + " |"
                )
    return "\n".join(lines)


def render_tree_details(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | seed | 飞机 | 主因 | 计划净空 m | 飞行段净空 m | 真值净空 m | 跟踪差 m | 深度里有 | 最紧已准入走廊 m | 规划码 |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    trees = sorted(
        _tree_rows(rows),
        key=lambda r: (SCENE_ORDER.index(r["scene"]), float(r["radius_m"]), A1_ORDER.index(r["a1"]), int(r["seed"])),
    )
    for row in trees:
        attr = row.get("attribution") or {}
        seen = attr.get("in_depth")
        seen_s = "—" if seen is None else ("有" if seen else "没有")
        tight = attr.get("fly_tightest_admitted_radius_m")
        if tight is None:
            tight = attr.get("tightest_admitted_radius_m")
        tight_s = "—" if tight is None else f"{float(tight):.3f}"

        def cell(key):
            val = attr.get(key)
            return "—" if val is None else f"{float(val):.3f}"

        lines.append(
            f"| {SCENE_LABEL[row['scene']]} | {arm_label(row['radius_m'], row['a1'])} | {row['seed']} | "
            f"{attr.get('drone') or '—'} | {attr.get('category') or '—'} | {cell('plan_clearance_m')} | "
            f"{cell('fly_clearance_m')} | {cell('actual_clearance_m')} | {cell('tracking_error_m')} | "
            f"{seen_s} | {tight_s} | `{attr.get('plan_code') or '—'}` |"
        )
    if len(lines) == 2:
        lines.append("| — | — | — | — | — | — | — | — | — | — | — | — |")
    return "\n".join(lines)


def _check() -> int:
    import hashlib

    blob = (ROOT / "scenes" / GEOM_NAME).read_bytes()
    got = hashlib.sha256(blob).hexdigest()
    if got != GEOM_SHA256:
        raise SystemExit(f"{GEOM_NAME} sha256 变了：{got}")
    eb4 = load_eb4()
    before = {rel: (ROOT / rel).read_bytes() for rel in SCENES.values()}
    before[f"scenes/{GEOM_NAME}"] = blob
    for scene in SCENE_ORDER:
        off = build_card(eb4, scene, "off")
        fix = build_card(eb4, scene, "fix")
        file_card_hash = _file_hash(scene)
        if off.hash != file_card_hash:
            raise SystemExit(f"{scene} 全关臂哈希与场景文件不一致")
        up = fix.card["policy_params"]["upstream"]
        if up.get("inter_drone_filter") is not True or abs(float(up["inter_drone_threshold_m"]) - 1.0) > 1e-12:
            raise SystemExit(f"{scene} 修复臂的 E-A1 不是 1.0 m")
        if "safe_radius" in up:
            raise SystemExit(f"{scene} 把 safe_radius 写进了进程内场景卡")
        if run_config(0.05, 0).safe_radius_m is not None:
            raise SystemExit("0.05 m 臂不应覆盖 RunConfig.safe_radius_m")
        if abs(float(run_config(0.20, 0).safe_radius_m) - 0.20) > 1e-12:
            raise SystemExit("0.20 m 臂没有放进 RunConfig")
    after = {rel: (ROOT / rel).read_bytes() for rel in before}
    if after != before:
        raise SystemExit("检查场景卡时改写了文件")
    suite = (ROOT / "scenes" / "suite.txt").read_text(encoding="utf-8")
    if "e-c5" in suite or "ego-density-014" in suite:
        raise SystemExit("scenes/suite.txt 被写进了本票的卡")
    print("e-c5 check ok")
    return 0


def _head_commit() -> str:
    import subprocess

    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scenes", default=",".join(SCENE_ORDER))
    ap.add_argument("--radii", default=",".join(f"{r:.2f}" for r in RADII))
    ap.add_argument("--a1", default=",".join(A1_ORDER))
    ap.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--out", default="docs/research/e-c5-corridor-radius-runs.json")
    ap.add_argument("--no-resume", action="store_true")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)
    if args.check:
        return _check()
    if args.jobs not in (1, 2, 3, 4):
        raise SystemExit("jobs 只能是 1 到 4")
    scenes = [s for s in args.scenes.split(",") if s]
    radii = [float(s) for s in args.radii.split(",") if s]
    a1s = [s for s in args.a1.split(",") if s]
    seeds = [int(s) for s in args.seeds.split(",") if s != ""]
    unknown = (
        [s for s in scenes if s not in SCENES]
        + [s for s in a1s if s not in A1_ORDER]
        + [r for r in radii if not any(abs(r - known) < 1e-9 for known in RADII)]
    )
    if unknown:
        raise SystemExit(f"未知场景、半径或臂：{unknown}")
    _check()
    resume = not args.no_resume
    specs = job_specs(scenes, radii, a1s, seeds, resume=resume)
    print(f"e-c5 start n={len(specs)} jobs={args.jobs} resume={resume}", flush=True)
    rows = _run_group(specs, int(args.jobs))
    rows.sort(key=lambda r: (
        SCENE_ORDER.index(r["scene"]),
        RADII.index(float(r["radius_m"])) if float(r["radius_m"]) in RADII else 99,
        A1_ORDER.index(r["a1"]),
        int(r["seed"]),
    ))
    payload = {
        "disclaimer": DISCLAIMER,
        "commit": _head_commit(),
        "scenes": scenes,
        "radii_m": radii,
        "a1": a1s,
        "seeds": seeds,
        "jobs": int(args.jobs),
        "attribution_order": list(TREE_ORDER),
        "rows": rows,
        "note": DISCLAIMER,
    }
    dest = Path(args.out)
    if not dest.is_absolute():
        dest = ROOT / dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(render_summary(rows))
    print(render_tree_counts(rows))
    print(f"写入 {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
