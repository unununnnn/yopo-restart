#!/usr/bin/env python3
"""E-C4b：密度场 L0 补跑 seeds 1–9，失败归因，并加 safe_radius 0.35 对照臂。只报数。

复用 scripts/e-c4-density-l0.py 的飞行卡和 off / combo 覆盖，不改那份脚本，
也不改已提交的场景卡。seed 0 的 off / combo 会重跑以便归因，并与已合入的
docs/research/e-c4-density-l0-runs.json 对照（全关臂同 seed 下，到达、碰撞、
最小距离和仿真时长不该变）。

三臂：
  off        默认全关，场景卡上不写开关。
  combo      E-A1 + D1 + E-B2，覆盖方式与 scripts/e-c4-density-l0.py 相同。
  radius035  全关 + RunConfig.safe_radius_m=0.35，结果标「偏离上游」。
             不把 0.35 写进已提交的场景卡。

不改默认，不冻结，不进 scenes/suite.txt。不改 yopo_l0/backend/、core/。
这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。

诊断钩子只记录 UpstreamPolicy 的前向与深度图，不改选择。combo 后段预算
按墙钟截断，combo 臂必须单进程。全程最多两个进程。
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

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-c4b-density-seeds" / "partial"
SEED0_SOURCE = ROOT / "docs/research/e-c4-density-l0-runs.json"

ARMS = ("off", "combo", "radius035")
KINDS = ("swap3", "solo")
SEEDS = tuple(range(1, 10))
BODY_RADIUS_M = 0.35
INTER_LIMIT_M = 0.70
UPSTREAM_SAFE_RADIUS_M = 0.05
RADIUS035_M = 0.35
DISCLAIMER = "这是假设，不是论文复现"

CAT_HOVER = "无解悬停后撞"
CAT_TRACK = "速度/跟踪偏差"
CAT_HIDDEN = "感知深度看不到"
CAT_CORRIDOR_005 = "走廊准入 safe_radius 0.05 太松"
CAT_CORRIDOR_035 = "走廊准入 safe_radius 0.35 太松"
CAT_FORMATION = "编队层"
CAT_INTER_OFF = "机间过滤未开"
CAT_INTER_ON = "机间过滤未挡住"
CAT_TIMEOUT_STALL = "超时卡死"
CAT_TIMEOUT_OTHER = "超时仍在前进"
CAT_OTHER = "其他"

# 撞树判定顺序与票 07c 同一思路：无解悬停、跟踪、走廊过松，再才是深度图里没有。
# 计划本身已经进机体且过了准入时，记走廊过松；in_depth 仍作为事实记下，不抢主因。
# 编队层只在 formation 任务上成立；本票卡都是 goto。
TREE_ORDER = (CAT_HOVER, CAT_TRACK, CAT_CORRIDOR_005, CAT_CORRIDOR_035, CAT_HIDDEN, CAT_OTHER)
INTER_ORDER = (CAT_FORMATION, CAT_HOVER, CAT_TRACK, CAT_INTER_OFF, CAT_INTER_ON, CAT_OTHER)
TIMEOUT_ORDER = (CAT_TIMEOUT_STALL, CAT_TIMEOUT_OTHER)

_HOVER_CODES = frozenset({
    "CORRIDOR_EMPTY_HOVER",
    "BRAKE_ONLY",
    "CONFLICT_BRAKE",
    "YOPO_KIN_FALLBACK",
    "YOPO_FAIL",
    "ARRIVE_KIN_FALLBACK",
})
_FLY_CODES = frozenset({"OK", "YOPO_ALT"})


def _load_base():
    path = ROOT / "scripts/e-c4-density-l0.py"
    spec = importlib.util.spec_from_file_location("e_c4_density_l0", path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"无法加载 {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _loadavg() -> dict:
    one, five, fifteen = os.getloadavg()
    return {"1": one, "5": five, "15": fifteen}


def corridor_label(safe_radius_m: float) -> str:
    if abs(float(safe_radius_m) - UPSTREAM_SAFE_RADIUS_M) <= 1e-9:
        return CAT_CORRIDOR_005
    if abs(float(safe_radius_m) - RADIUS035_M) <= 1e-9:
        return CAT_CORRIDOR_035
    return f"走廊准入 safe_radius {float(safe_radius_m):g} 太松"


def nearest_obstacle(obstacles, position: np.ndarray):
    from yopo_l0.scene import Ground

    best = None
    best_d = math.inf
    for ob in obstacles:
        if isinstance(ob, Ground):
            continue
        dist = float(ob.distance(np.asarray(position, dtype=float)))
        if dist < best_d:
            best, best_d = ob, dist
    return best, best_d


def _rows_for(ticks: list[dict], drone: str) -> list[dict]:
    return [row for row in ticks if row["uav"] == drone]


def _at_or_before(rows: list[dict], t: float) -> dict | None:
    picked = None
    for row in rows:
        if float(row["t"]) <= t + 1e-9:
            picked = row
        else:
            break
    return picked


def _mean_speed(rows: list[dict]) -> float:
    if not rows:
        return 0.0
    return float(np.mean([np.linalg.norm(row["v"]) for row in rows]))


def _progress_m(rows: list[dict]) -> float:
    if len(rows) < 2:
        return 0.0
    return float(rows[0]["goal_error_m"]) - float(rows[-1]["goal_error_m"])


def drone_arrived(ticks: list[dict], drone: str, *, gtol=0.4, gspeed=0.2, ghold=1.0) -> bool:
    started = None
    last_t = None
    for row in _rows_for(ticks, drone):
        t = float(row["t"])
        speed = float(np.linalg.norm(row["v"]))
        if float(row["goal_error_m"]) <= gtol and speed <= gspeed:
            if started is None:
                started = t
            last_t = t
            if last_t - started >= ghold - 1e-9:
                return True
        else:
            started = None
    return False


def install_diag():
    """用子类替换 POLICIES['upstream']。选择仍走 UpstreamPolicy.plan。"""
    import yopo_l0.policy as policy_mod
    from yopo_swarm_core.candidates import corridor_mu_lo

    base = policy_mod.UpstreamPolicy

    class DiagUpstream(base):
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
            DiagUpstream.instances.append(self)

        def plan(self, obs):
            self._last_fwd = None
            out = super().plan(obs)
            fwd = self._last_fwd
            n_admitted = None
            corridor_empty = None
            mu_lo_min = None
            safe_mu = None
            if fwd is not None:
                _params, _score, radius = fwd
                dec = policy_mod.upstream_decoder(self.p["safe_radius"])
                mu = corridor_mu_lo(radius, dec)
                safe_mu = float(dec.safe_mu)
                admitted = np.asarray(mu, dtype=np.float64) >= np.float64(dec.safe_mu)
                n_admitted = int(admitted.sum())
                corridor_empty = bool(n_admitted == 0)
                mu_lo_min = None if n_admitted == 0 else float(np.asarray(mu)[admitted].min())
            s = obs.state
            self.trace.append(
                {
                    "drone": self.drone_id,
                    "now": float(obs.now),
                    "code": self.last_code,
                    "p": np.asarray(s.p, dtype=float).tolist(),
                    "yaw": float(s.yaw),
                    "frame": obs.frame,
                    "n_admitted": n_admitted,
                    "corridor_empty": corridor_empty,
                    "mu_lo_min_admitted": mu_lo_min,
                    "safe_mu": safe_mu,
                    "safe_radius_m": float(self.p["safe_radius"]),
                }
            )
            return out

    previous = policy_mod.POLICIES["upstream"]
    policy_mod.POLICIES["upstream"] = DiagUpstream
    DiagUpstream.instances = []
    return DiagUpstream, previous


def restore_diag(previous) -> None:
    import yopo_l0.policy as policy_mod

    policy_mod.POLICIES["upstream"] = previous


def _flown_trace(trace: list[dict], t_hit: float) -> dict | None:
    window = [row for row in trace if float(row["now"]) <= t_hit + 1e-9]
    return window[-1] if window else None


def classify_tree(facts: dict) -> str:
    """悬停 → 跟踪 → 走廊过松 → 深度看不到 → 其他。"""
    if facts.get("plan_code") in _HOVER_CODES:
        return CAT_HOVER
    plan_c = facts.get("plan_clearance_m")
    actual_c = facts.get("actual_clearance_m")
    body = facts.get("body_radius_m", BODY_RADIUS_M)
    if plan_c is not None and actual_c is not None and plan_c > body and actual_c <= body:
        return CAT_TRACK
    if (
        plan_c is not None
        and plan_c <= body
        and facts.get("admitted")
        and facts.get("plan_code") in _FLY_CODES
    ):
        return corridor_label(facts.get("safe_radius_m", UPSTREAM_SAFE_RADIUS_M))
    if facts.get("frame_available") and facts.get("in_depth") is False:
        return CAT_HIDDEN
    return CAT_OTHER


def classify_inter(facts: dict) -> str:
    if facts.get("task") == "formation":
        return CAT_FORMATION
    if facts.get("plan_code") in _HOVER_CODES:
        return CAT_HOVER
    planned = facts.get("planned_gap_m")
    actual = facts.get("actual_gap_m")
    if planned is not None and actual is not None and planned > INTER_LIMIT_M and actual <= INTER_LIMIT_M:
        return CAT_TRACK
    if facts.get("filter_enabled"):
        return CAT_INTER_ON
    return CAT_INTER_OFF


def classify_timeout_drone(facts: dict) -> str:
    if facts.get("stalled"):
        return CAT_TIMEOUT_STALL
    return CAT_TIMEOUT_OTHER


def _cmd_clearance(rows: list[dict], obstacles, t0: float, t1: float) -> tuple[float, np.ndarray]:
    window = [row for row in rows if t0 - 1e-12 <= float(row["t"]) <= t1 + 1e-12]
    if not window:
        window = rows[-1:] if rows else []
    best_d = math.inf
    best_p = np.zeros(3)
    for row in window:
        p = np.asarray(row["cmd_p"], dtype=float)
        _ob, dist = nearest_obstacle(obstacles, p)
        if dist < best_d:
            best_d, best_p = dist, p
    return best_d, best_p


def _diagnose_tree(card, result, ticks, traces, drone: str) -> dict:
    from yopo_l0.dense_diag import depth_sees
    from yopo_l0.sensor import CameraConfig

    rows = _rows_for(ticks, drone)
    hit = rows[-1]
    t_hit = float(hit["t"])
    p_hit = np.asarray(hit["p"], dtype=float)
    obstacle, actual = nearest_obstacle(card.obstacles, p_hit)
    plan_clear, plan_p = _cmd_clearance(rows, card.obstacles, max(0.0, t_hit - 0.5), t_hit)
    flown = _flown_trace(traces.get(drone) or [], t_hit)
    in_depth = None
    frame_available = bool(flown and flown.get("frame") is not None and obstacle is not None)
    if frame_available:
        cam = CameraConfig.from_card(card.card["camera"])
        in_depth = bool(
            depth_sees(
                flown["frame"],
                np.asarray(flown["p"], dtype=float),
                float(flown["yaw"]),
                cam,
                obstacle,
            )
        )
    body = float(card.card["criteria"]["body_radius_m"])
    facts = {
        "kind": "tree",
        "drone": drone,
        "t_hit_s": t_hit,
        "obstacle": None if obstacle is None else getattr(obstacle, "name", None),
        "actual_clearance_m": actual if actual is not math.inf else None,
        "plan_clearance_m": None if plan_clear is math.inf else float(plan_clear),
        "tracking_error_m": float(np.linalg.norm(p_hit - np.asarray(hit["cmd_p"], dtype=float))),
        "cmd_plan_error_m": float(np.linalg.norm(p_hit - plan_p)),
        "speed_mps": float(np.linalg.norm(hit["v"])),
        "in_depth": in_depth,
        "frame_available": frame_available,
        "plan_code": None if flown is None else flown.get("code"),
        "admitted": bool(flown and (flown.get("n_admitted") or 0) > 0 and not flown.get("corridor_empty")),
        "n_admitted": None if flown is None else flown.get("n_admitted"),
        "safe_radius_m": float(result.get("safe_radius_m") or UPSTREAM_SAFE_RADIUS_M),
        "body_radius_m": body,
        "task": result.get("task"),
    }
    facts["category"] = classify_tree(facts)
    return facts


def _diagnose_inter(result, ticks, traces, reason: str) -> dict:
    left, right = reason.split(":", 1)[0].split("-", 1)
    rows_a = _rows_for(ticks, left)
    rows_b = _rows_for(ticks, right)
    hit_a, hit_b = rows_a[-1], rows_b[-1]
    t_hit = float(hit_a["t"])
    actual = float(np.linalg.norm(np.asarray(hit_a["p"], dtype=float) - np.asarray(hit_b["p"], dtype=float)))
    planned = float(np.linalg.norm(np.asarray(hit_a["cmd_p"], dtype=float) - np.asarray(hit_b["cmd_p"], dtype=float)))
    flown_a = _flown_trace(traces.get(left) or [], t_hit)
    flown_b = _flown_trace(traces.get(right) or [], t_hit)
    code = flown_a.get("code") if flown_a else None
    if code not in _HOVER_CODES and flown_b and flown_b.get("code") in _HOVER_CODES:
        code = flown_b.get("code")
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
        "filter_enabled": bool(filt),
        "filtered_candidates": int(filt.get("filtered_candidates") or 0),
        "empty_fallbacks": int(filt.get("empty_fallbacks") or 0),
        "task": result.get("task"),
        "safe_radius_m": float(result.get("safe_radius_m") or UPSTREAM_SAFE_RADIUS_M),
    }
    facts["category"] = classify_inter(facts)
    return facts


def _window(rows: list[dict], t_end: float, span: float = 5.0) -> list[dict]:
    t0 = max(0.0, t_end - span)
    if rows:
        dur = float(rows[-1]["t"]) - float(rows[0]["t"])
        if dur < span * 2 and dur > 0:
            t0 = max(0.0, t_end - max(span, dur / 2.0))
    return [row for row in rows if float(row["t"]) >= t0 - 1e-12]


def _diagnose_timeout(result, ticks, traces, n_drones: int) -> dict:
    drones = [f"uav{i}" for i in range(n_drones)]
    failed = []
    per = []
    for drone in drones:
        if drone_arrived(ticks, drone):
            continue
        rows = _rows_for(ticks, drone)
        if not rows:
            continue
        t_end = float(rows[-1]["t"])
        window = _window(rows, t_end)
        trace = [row for row in (traces.get(drone) or []) if float(row["now"]) >= t_end - 5.0]
        hover_n = sum(1 for row in trace if row.get("code") in _HOVER_CODES)
        stalled = bool(
            (_progress_m(window) < 0.5 and _mean_speed(window) < 0.20)
            or (trace and hover_n >= max(1, (len(trace) + 1) // 2))
        )
        item = {
            "kind": "timeout",
            "drone": drone,
            "t_end_s": t_end,
            "goal_error_m": float(rows[-1]["goal_error_m"]),
            "progress_m": _progress_m(window),
            "mean_speed_mps": _mean_speed(window),
            "hover_plans": hover_n,
            "plans_in_window": len(trace),
            "stalled": stalled,
            "plan_code": None if not trace else trace[-1].get("code"),
            "task": result.get("task"),
        }
        item["category"] = classify_timeout_drone(item)
        per.append(item)
        failed.append(drone)
    if any(item["category"] == CAT_TIMEOUT_STALL for item in per):
        category = CAT_TIMEOUT_STALL
    elif per:
        category = CAT_TIMEOUT_OTHER
    else:
        category = CAT_TIMEOUT_OTHER
    return {
        "kind": "timeout",
        "drones": failed,
        "category": category,
        "per_drone": per,
    }


def diagnose(card, result, ticks, traces: dict[str, list]) -> dict:
    outcome = result.get("outcome")
    reason = str(result.get("reason") or "")
    if outcome == "COLLISION" and "TRUTH_POSITION" in reason:
        drone = reason.split(":", 1)[0]
        return _diagnose_tree(card, result, ticks, traces, drone)
    if outcome == "COLLISION" and "INTER_DRONE" in reason:
        return _diagnose_inter(result, ticks, traces, reason)
    if outcome == "TIMEOUT":
        return _diagnose_timeout(result, ticks, traces, int(result["n_drones"]))
    return {"kind": None, "category": None}


def compact_attribution(attr: dict | None) -> dict | None:
    if not attr:
        return None
    skip = {"frame"}
    out = {}
    for key, value in attr.items():
        if key == "per_drone":
            out[key] = [{k: v for k, v in row.items() if k not in skip} for row in value]
        else:
            out[key] = value
    return out


def traces_from_instances(cls) -> dict[str, list]:
    out: dict[str, list] = {}
    for inst in getattr(cls, "instances", []):
        out[inst.drone_id] = list(inst.trace)
    return out


def build_run_card(base, tag: str, kind: str, arm: str):
    if arm == "radius035":
        return base.build_run_card(tag, kind, "off")
    return base.build_run_card(tag, kind, arm)


def run_config(arm: str, seed: int):
    from yopo_l0.runner import RunConfig

    if arm == "radius035":
        return RunConfig(seed=int(seed), shadow=True, safe_radius_m=RADIUS035_M)
    return RunConfig(seed=int(seed), shadow=True)


def _partial(scene: str, arm: str, seed: int) -> Path:
    return PARTIAL / f"{scene}-{arm}-{seed}.json"


def run_one(base, tag: str, kind: str, arm: str, seed: int, *, resume: bool) -> dict:
    key = base.scene_key(tag, kind)
    dest = _partial(key, arm, seed)
    if resume and dest.is_file():
        return json.loads(dest.read_text(encoding="utf-8"))
    from yopo_l0.runner import run

    card = build_run_card(base, tag, kind, arm)
    wrapped = base._budget_wrap()
    cfg = run_config(arm, seed)
    diag_cls, previous = install_diag()
    t0 = time.perf_counter()
    try:
        out = run(card, cfg)
        traces = traces_from_instances(diag_cls)
        attr = diagnose(card, out.result, out.ticks, traces)
    finally:
        restore_diag(previous)
        diag_cls.instances = []
    row = base._row(
        key,
        arm,
        out.result,
        out.ticks,
        budget_calls=int(wrapped.calls),
        budget_cuts=int(wrapped.cuts),
    )
    row["density_tag"] = tag
    row["kind"] = kind
    row["worker_s"] = time.perf_counter() - t0
    row["safe_radius_m"] = out.result.get("safe_radius_m")
    row["upstream_deviation"] = bool(out.result.get("upstream_deviation"))
    row["attribution"] = compact_attribution(attr)
    row["loadavg_before"] = getattr(run_one, "_load_before", None)
    row["loadavg_after"] = _loadavg()
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)
    cat = (row.get("attribution") or {}).get("category") or "—"
    gap = row["min_inter_drone_distance_m"]
    gap_s = "—" if gap is None else f"{float(gap):.3f}"
    dist = row["min_obstacle_distance_m"]
    dist_s = "—" if dist is None else f"{float(dist):.3f}"
    print(
        f"{key} {arm} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"tree={row['tree_collisions']} inter={row['inter_drone_collisions']} "
        f"gap={gap_s} obst={dist_s} sim={row['sim_duration_s']} "
        f"wall={row['wall_time_s']:.1f}s cat={cat}",
        flush=True,
    )
    return row


def _worker(spec: tuple) -> dict:
    base = _load_base()
    tag, kind, arm, seed, resume = spec
    run_one._load_before = _loadavg()
    return run_one(base, tag, kind, arm, int(seed), resume=bool(resume))


def _run_group(specs: list[tuple], jobs: int) -> list[dict]:
    if jobs <= 1 or len(specs) <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)


def _f(value, nd: int) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{nd}f}"


def _arm_label(arm: str) -> str:
    return {
        "off": "全关",
        "combo": "三个一起",
        "radius035": "safe_radius 0.35（偏离上游）",
    }[arm]


def count_categories(rows: list[dict]) -> list[tuple[str, int]]:
    counts: dict[str, int] = {}
    for row in rows:
        attr = row.get("attribution") or {}
        if row.get("outcome") in {"COLLISION", "TIMEOUT"}:
            cat = attr.get("category")
            if not cat:
                continue
            counts[cat] = counts.get(cat, 0) + 1
    return sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))


def summarize(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | 到达 | 撞树 | 机间碰撞 | 超时 | 最小障碍距离 m | 最小机间 m |",
        "|---|---|---|---|---|---|---|---|",
    ]
    base = _load_base()
    order = [base.scene_key(tag, kind) for tag, *_ in base.DENSITIES for kind in KINDS]
    for key in order:
        for arm in ARMS:
            group = [r for r in rows if r["scene_kind"] == key and r["arm"] == arm]
            if not group:
                continue
            drones = sum(int(r["n_drones"]) for r in group)
            arrived = sum(int(r["arrived"]) for r in group)
            tree = sum(int(r["tree_collisions"]) for r in group)
            inter = sum(int(r["inter_drone_collisions"]) for r in group)
            timeout = sum(int(r["timeout"]) for r in group)
            obst = [float(r["min_obstacle_distance_m"]) for r in group if r.get("min_obstacle_distance_m") is not None]
            gaps = [
                float(r["min_inter_drone_distance_m"])
                for r in group
                if r.get("min_inter_drone_distance_m") is not None
            ]
            lo, hi = (min(obst), max(obst)) if obst else (None, None)
            obst_s = "—" if lo is None else (_f(lo, 3) if abs(lo - hi) < 5e-4 else f"{_f(lo, 3)}–{_f(hi, 3)}")
            glo, ghi = (min(gaps), max(gaps)) if gaps else (None, None)
            gap_s = "—" if glo is None else (_f(glo, 3) if abs(glo - ghi) < 5e-4 else f"{_f(glo, 3)}–{_f(ghi, 3)}")
            lines.append(
                f"| {key} | {_arm_label(arm)} | {arrived}/{drones} | {tree} | "
                f"{inter} | {timeout} | {obst_s} | {gap_s} |"
            )
    lines.append("")
    lines.append("归因（失败次数）：")
    for name, n in count_categories(rows):
        lines.append(f"- {name}：{n}")
    return "\n".join(lines)


def copy_seed0_reference() -> list[dict]:
    if not SEED0_SOURCE.is_file():
        return []
    payload = json.loads(SEED0_SOURCE.read_text(encoding="utf-8"))
    rows = []
    for raw in payload.get("rows") or []:
        if int(raw.get("seed", -1)) != 0:
            continue
        if raw.get("arm") not in ("off", "combo"):
            continue
        row = dict(raw)
        row["copied_from"] = "docs/research/e-c4-density-l0-runs.json"
        rows.append(row)
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--densities", default="014,028,042")
    ap.add_argument("--kinds", default="swap3,solo")
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    ap.add_argument("--jobs-plain", type=int, default=2)
    ap.add_argument("--jobs-backend", type=int, default=1)
    ap.add_argument("--out", default="docs/research/e-c4b-density-seeds-runs.json")
    ap.add_argument("--no-resume", action="store_true")
    a = ap.parse_args(argv)
    if a.jobs_plain < 1 or a.jobs_plain > 2:
        raise SystemExit("jobs-plain 只能是 1 或 2")
    if a.jobs_backend != 1:
        raise SystemExit("combo 臂的后段预算按墙钟截断，jobs-backend 必须是 1")
    tags = [s for s in a.densities.split(",") if s]
    kinds = [s for s in a.kinds.split(",") if s]
    arms = [s for s in a.arms.split(",") if s]
    seeds = [int(s) for s in a.seeds.split(",") if s != ""]
    base = _load_base()
    known_tags = {t[0] for t in base.DENSITIES}
    unknown = (
        [s for s in tags if s not in known_tags]
        + [s for s in kinds if s not in KINDS]
        + [s for s in arms if s not in ARMS]
    )
    if unknown:
        raise SystemExit(f"未知密度、卡种或臂：{unknown}")
    import hashlib

    for name, digest in base.GEOM_SHA256.items():
        got = hashlib.sha256((ROOT / "scenes" / name).read_bytes()).hexdigest()
        if got != digest:
            raise SystemExit(f"{name} sha256 变了：{got}")
    resume = not a.no_resume
    from yopo_swarm_core.baseline import fetch_upstream

    fetch_upstream()
    PARTIAL.mkdir(parents=True, exist_ok=True)
    plain = [arm for arm in arms if arm != "combo"]
    backend = [arm for arm in arms if arm == "combo"]
    rows: list[dict] = []
    if plain:
        specs = [(tag, kind, arm, seed, resume) for arm in plain for tag in tags for kind in kinds for seed in seeds]
        print(f"plain start loadavg={_loadavg()} n={len(specs)} jobs={a.jobs_plain}", flush=True)
        rows.extend(_run_group(specs, int(a.jobs_plain)))
    if backend:
        specs = [(tag, kind, "combo", seed, resume) for tag in tags for kind in kinds for seed in seeds]
        print(f"combo start loadavg={_loadavg()} n={len(specs)} jobs={a.jobs_backend}", flush=True)
        rows.extend(_run_group(specs, int(a.jobs_backend)))
    rows.sort(key=lambda r: (r["density_tag"], r["kind"], ARMS.index(r["arm"]), int(r["seed"])))
    payload = {
        "rows": rows,
        "shadow": True,
        "jobs_plain": int(a.jobs_plain),
        "jobs_backend": int(a.jobs_backend),
        "seeds_run": seeds,
        "arms": arms,
        "seed0_reference": {
            "source": "docs/research/e-c4-density-l0-runs.json",
            "note": "已合入 E-C4 密度场 seed 0 的 off/combo，只作对照。本脚本若包含 seed 0，数字以本脚本重跑为准，并做归因。",
            "rows": copy_seed0_reference(),
        },
        "attribution_order": {
            "tree": list(TREE_ORDER),
            "inter": list(INTER_ORDER),
            "timeout": list(TIMEOUT_ORDER),
        },
        "note": (
            "只报数，不改默认，不冻结。墙钟不参与轨迹。"
            "backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。"
            "radius035 是 safe_radius 0.35 m，偏离上游 d5e1e60。"
            f"{DISCLAIMER}（障碍半径 {base.OBSTACLE_RADIUS_M:.2f} m，边长 {base.MAP_SIDE_M:.1f} m）。"
        ),
    }
    dest = Path(a.out)
    if not dest.is_absolute():
        dest = ROOT / dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(summarize(rows))
    print(f"写入 {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
