#!/usr/bin/env python3
"""票 SH4：用已有 SH2/SH2b 日志和 L0 对照 TRACKING_BOUND 修正开关。只报数。

L0 在已打开环境证据和 SH3 契约修正的基础上再开/关 shadow_tracking_align。
g2-hover-lift 另报四组开关，因为只有这张卡在 L0 上会记下 TRACKING_BOUND。
不改判据，不改设定点，不放宽 0.15 m。墙钟不进结论。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "sh4-tracking-bound"
DOC_SH2 = ROOT / "docs" / "research" / "sh2-sitl-shadow-evidence-runs.json"
DOC = ROOT / "docs" / "research" / "sh4-tracking-bound-runs.json"
L0_AFTER_SH3 = (
    ("g2-pillar", "scenes/g2-pillar.json", None, list(range(10))),
    ("crossing-pillars", "scenes/crossing-pillars.json", None, list(range(10))),
)
LIFT = ("g2-hover-lift", "scenes/g2-hover-lift.json", "hover", list(range(10)))
FOCUS_CODES = (
    "TIME:SWITCH_NOT_IN_FUTURE",
    "TRANSITION:HEAD_DISCONTINUITY",
    "TRANSITION:TERMINAL_NOT_HOLDABLE",
    "TRANSITION:MOTION_KINEMATICS",
    "TRANSITION:BRANCH_UNREACHABLE",
    "TRACKING_BOUND",
    "LAG_VIOLATION",
    "ENV:OCCUPIED",
    "ENV:UNKNOWN",
)


def _traj_digest(ticks: list[dict]) -> str:
    h = hashlib.sha256()
    for t in ticks:
        blob = json.dumps(
            [t["uav"], t["p"], t["v"], t["a"], t["yaw"], t["cmd_p"], t["cmd_v"], t["cmd_a"]],
            separators=(",", ":"),
        )
        h.update(blob.encode())
    return h.hexdigest()


def _write(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(row, ensure_ascii=False, indent=2) + "\n")


def _p95(vals: list[float]) -> float | None:
    if not vals:
        return None
    return float(np.percentile(np.asarray(vals, dtype=float), 95))


def _split(delta, heading_xy) -> tuple[float, float]:
    d = np.asarray(delta, dtype=float)[:2]
    n = float(np.linalg.norm(heading_xy))
    if n < 1e-6:
        return 0.0, float(np.linalg.norm(d))
    u = np.asarray(heading_xy, dtype=float) / n
    along = float(d @ u)
    lat = float(u[0] * d[1] - u[1] * d[0])
    return along, lat


def _arm_record(result: dict, ticks: list[dict], body: float) -> dict:
    from yopo_l0.shadow import aggregate
    from yopo_l0.shadow_sitl import attribution

    rate, hist = aggregate(ticks)
    track_eucl, track_lat = [], []
    for tick in ticks:
        p = tick.get("p")
        cmd = tick.get("cmd_p")
        if p is None or cmd is None:
            continue
        delta = np.asarray(p, dtype=float) - np.asarray(cmd, dtype=float)
        track_eucl.append(float(np.linalg.norm(delta)))
        heading = np.asarray(tick.get("cmd_v") or (0, 0, 0), dtype=float)[:2]
        _, side = _split(delta, heading)
        track_lat.append(abs(side))
    gap = result.get("min_inter_drone_distance_m")
    obs = int(float(result["min_obstacle_distance_m"]) <= body + 1e-9)
    inter = int(gap is not None and float(gap) <= 2.0 * body + 1e-9)
    return {
        "outcome": result["outcome"],
        "reason": result["reason"],
        "arrived": result["arrived"],
        "n_drones": result["n_drones"],
        "arrival_time_s": result["arrival_time_s"],
        "min_obstacle_distance_m": result["min_obstacle_distance_m"],
        "min_inter_drone_distance_m": result["min_inter_drone_distance_m"],
        "collisions": {
            "obstacle": obs,
            "inter": inter,
            "outcome_collision": int(result["outcome"] == "COLLISION"),
        },
        "plan_codes": result.get("plan_codes"),
        "upstream_deviation": bool(result.get("upstream_deviation")),
        "shadow_env_evidence": bool(result.get("shadow_env_evidence", False)),
        "shadow_contract_align": bool(result.get("shadow_contract_align", False)),
        "shadow_tracking_align": bool(result.get("shadow_tracking_align", False)),
        "shadow_reject_rate": rate,
        "shadow_reject_histogram": hist,
        "traj_sha256": _traj_digest(ticks),
        "attribution": attribution(ticks),
        "focus_counts": {c: int(hist.get(c, 0)) for c in FOCUS_CODES},
        "tracking_euclid_p95_m": _p95(track_eucl),
        "tracking_lateral_p95_m": _p95(track_lat),
        "tracking_over_015": sum(1 for e in track_eucl if e > 0.15),
        "tracking_samples": len(track_eucl),
    }


def _run_cfg(seed: int, policy: str | None, **flags):
    from yopo_l0.runner import RunConfig

    kw = dict(seed=seed, shadow=True, **flags)
    if policy:
        kw["policy"] = policy
    return RunConfig(**kw)


def run_l0_after_sh3(scene_name: str, scene_path: str, policy: str | None, seed: int) -> dict:
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    from yopo_l0.runner import run
    from yopo_l0.scene import load_card

    card = load_card(ROOT / scene_path)
    body = float(card.card["criteria"]["body_radius_m"])
    common = dict(shadow_env_evidence=True, shadow_contract_align=True)
    off = run(card, _run_cfg(seed, policy, **common, shadow_tracking_align=False))
    on = run(card, _run_cfg(seed, policy, **common, shadow_tracking_align=True))
    same_traj = _traj_digest(off.ticks) == _traj_digest(on.ticks)
    if not same_traj or off.result["outcome"] != on.result["outcome"] or off.result["arrived"] != on.result["arrived"]:
        raise RuntimeError(f"{scene_name} seed {seed} 两臂轨迹或到达不一致")
    return {
        "layer": "l0",
        "scene": scene_name,
        "scene_path": scene_path,
        "scene_card_hash": card.hash,
        "seed": seed,
        "same_traj": same_traj,
        "off": _arm_record(off.result, off.ticks, body),
        "on": _arm_record(on.result, on.ticks, body),
    }


def run_l0_lift(seed: int) -> dict:
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    from yopo_l0.runner import run
    from yopo_l0.scene import load_card

    name, path, policy, _ = LIFT
    card = load_card(ROOT / path)
    body = float(card.card["criteria"]["body_radius_m"])
    arms = {
        "base": run(card, _run_cfg(seed, policy)),
        "contract": run(card, _run_cfg(seed, policy, shadow_contract_align=True)),
        "tracking": run(card, _run_cfg(seed, policy, shadow_tracking_align=True)),
        "both": run(card, _run_cfg(seed, policy, shadow_contract_align=True, shadow_tracking_align=True)),
    }
    digests = {k: _traj_digest(v.ticks) for k, v in arms.items()}
    if len(set(digests.values())) != 1:
        raise RuntimeError(f"lift seed {seed} 各臂轨迹不一致：{digests}")
    return {
        "layer": "l0-lift",
        "scene": name,
        "scene_path": path,
        "scene_card_hash": card.hash,
        "seed": seed,
        "same_traj": True,
        **{k: _arm_record(v.result, v.ticks, body) for k, v in arms.items()},
    }


def _aggregate_pair(rows: list[dict]) -> dict:
    n = len(rows)
    def _sum(arm, key):
        return sum(r[arm]["focus_counts"].get(key, 0) for r in rows)

    out = {
        "n": n,
        "arrived_off": [r["off"]["arrived"] for r in rows],
        "arrived_on": [r["on"]["arrived"] for r in rows],
        "outcomes_off": [r["off"]["outcome"] for r in rows],
        "outcomes_on": [r["on"]["outcome"] for r in rows],
        "would_reject_off": [r["off"]["shadow_reject_rate"] for r in rows],
        "would_reject_on": [r["on"]["shadow_reject_rate"] for r in rows],
        "would_reject_off_mean": None if not rows else sum(r["off"]["shadow_reject_rate"] for r in rows) / n,
        "would_reject_on_mean": None if not rows else sum(r["on"]["shadow_reject_rate"] for r in rows) / n,
        "same_traj": all(r["same_traj"] for r in rows),
        "focus_off": {c: _sum("off", c) for c in FOCUS_CODES},
        "focus_on": {c: _sum("on", c) for c in FOCUS_CODES},
        "hist_off": {},
        "hist_on": {},
    }
    for arm, dest in (("off", "hist_off"), ("on", "hist_on")):
        acc = {}
        for r in rows:
            for k, v in (r[arm].get("shadow_reject_histogram") or {}).items():
                acc[k] = acc.get(k, 0) + int(v)
        out[dest] = dict(sorted(acc.items()))
    return out


def _aggregate_lift(rows: list[dict]) -> dict:
    n = len(rows)
    out = {"n": n, "same_traj": all(r["same_traj"] for r in rows), "arms": {}}
    for arm in ("base", "contract", "tracking", "both"):
        wr = [r[arm]["shadow_reject_rate"] for r in rows]
        acc = {}
        focus = {c: 0 for c in FOCUS_CODES}
        for r in rows:
            for k, v in (r[arm].get("shadow_reject_histogram") or {}).items():
                acc[k] = acc.get(k, 0) + int(v)
            for c in FOCUS_CODES:
                focus[c] += r[arm]["focus_counts"].get(c, 0)
        out["arms"][arm] = {
            "would_reject": wr,
            "would_reject_mean": sum(wr) / n if n else None,
            "outcomes": [r[arm]["outcome"] for r in rows],
            "histogram": dict(sorted(acc.items())),
            "focus": focus,
        }
    return out


def _trace_tracking(result: dict) -> dict:
    eucl: list[float] = []
    lat: list[float] = []
    n = n_over = 0
    for drone in result.get("drones") or []:
        for row in drone.get("trace") or []:
            odom = row.get("odometry_world_enu")
            sp = row.get("setpoint_world_enu")
            if odom is None or sp is None:
                continue
            odom = np.asarray(odom, dtype=float)
            sp = np.asarray(sp, dtype=float)
            if odom.shape != (3,) or sp.shape != (3,):
                continue
            err = float(np.linalg.norm(odom - sp))
            ev = row.get("setpoint_velocity_enu")
            heading = np.asarray(ev if ev is not None else (0, 0), dtype=float)[:2]
            _, side = _split(odom - sp, heading)
            eucl.append(err)
            lat.append(abs(side))
            n += 1
            if err > 0.15:
                n_over += 1
    return {
        "n_samples": n,
        "euclid_p95_m": _p95(eucl),
        "lateral_p95_m": _p95(lat),
        "euclid_max_m": None if not eucl else float(max(eucl)),
        "over_015": n_over,
        "over_015_rate": None if n == 0 else n_over / n,
    }


def cmd_logs(_args) -> int:
    doc = json.loads(DOC_SH2.read_text())
    pairs = doc["sitl_lockstep"]["pairs"]
    rows = []
    for seed in ("0", "1", "2", "3", "4"):
        for arm in ("off", "on"):
            rec = pairs[seed][arm]
            result = rec["result"]
            attr = result.get("shadow_attribution") or {}
            hist = attr.get("histogram") or result.get("shadow_reject_histogram") or {}
            track = _trace_tracking(result)
            rows.append({
                "seed": int(seed),
                "arm": arm,
                "shadow_env_evidence": bool(rec.get("shadow_env_evidence", arm == "on")),
                "run_dir": rec.get("run_dir"),
                "isolated_rerun": bool((rec.get("sh2b") or {}).get("isolated_rerun")),
                "voided_run": (rec.get("sh2b") or {}).get("voided_run"),
                "outcome": result.get("outcome"),
                "reason": result.get("reason"),
                "arrived": result.get("arrived"),
                "collisions": result.get("collisions"),
                "ticks": attr.get("ticks"),
                "would_reject_rate": attr.get("would_reject_rate", result.get("shadow_reject_rate")),
                "env_reject_rate": attr.get("env_reject_rate"),
                "tracking_bound_ticks": int(hist.get("TRACKING_BOUND", 0)),
                "histogram": hist,
                "trace_tracking": track,
            })
    payload = {
        "note": "票 SH4：从 SH2/SH2b 已有 result.json 的稀疏 trace 重算跟踪误差；TRACKING_BOUND 计数来自当时的影子直方图。trace 是降采样，不是规划时刻口径。",
        "generated": time.strftime("%Y-%m-%d %H:%M:%S UTC-3"),
        "tracking_bound_m": 0.15,
        "rows": rows,
    }
    _write(OUT / "sitl-logs.json", payload)
    print(json.dumps({
        "n": len(rows),
        "lateral_p95": [r["trace_tracking"]["lateral_p95_m"] for r in rows],
        "euclid_p95": [r["trace_tracking"]["euclid_p95_m"] for r in rows],
        "tracking_bound_ticks": [r["tracking_bound_ticks"] for r in rows],
        "would_reject": [r["would_reject_rate"] for r in rows],
    }, ensure_ascii=False, indent=2))
    return 0


def _l0_job(kind, name, path, policy, seed):
    if kind == "lift":
        return run_l0_lift(seed)
    return run_l0_after_sh3(name, path, policy, seed)


def cmd_l0(args) -> int:
    workers = max(1, int(args.workers))
    jobs = []
    for name, path, policy, seeds in L0_AFTER_SH3:
        if args.scene and args.scene != name:
            continue
        for seed in seeds:
            if args.seed is not None and seed != args.seed:
                continue
            jobs.append(("after", name, path, policy, seed))
    if not args.scene or args.scene == LIFT[0]:
        for seed in LIFT[3]:
            if args.seed is not None and seed != args.seed:
                continue
            jobs.append(("lift", LIFT[0], LIFT[1], LIFT[2], seed))
    print(f"l0 jobs={len(jobs)} workers={workers}", flush=True)

    rows = []
    if workers == 1:
        for job in jobs:
            row = _l0_job(*job)
            rows.append(row)
            print(f"done {row['scene']} seed {row['seed']}", flush=True)
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futs = {pool.submit(_l0_job, *job): job for job in jobs}
            for fut in as_completed(futs):
                row = fut.result()
                rows.append(row)
                print(f"done {row['scene']} seed {row['seed']}", flush=True)
    rows.sort(key=lambda r: (r["scene"], r["seed"]))
    after = [r for r in rows if r["layer"] == "l0"]
    lift = [r for r in rows if r["layer"] == "l0-lift"]
    payload = {
        "note": "票 SH4：L0 开关对照。after_sh3 两臂都开环境证据和契约修正；on 再开 tracking_align。lift 另报四组。",
        "generated": time.strftime("%Y-%m-%d %H:%M:%S UTC-3"),
        "after_sh3": after,
        "after_sh3_summary": {
            name: _aggregate_pair([r for r in after if r["scene"] == name])
            for name in dict.fromkeys(r["scene"] for r in after)
        },
        "lift": lift,
        "lift_summary": _aggregate_lift(lift) if lift else {},
    }
    _write(OUT / "l0.json", payload)
    print(json.dumps({
        "after_sh3": {k: {"wr_off": v["would_reject_off_mean"], "wr_on": v["would_reject_on_mean"], "track_off": v["focus_off"]["TRACKING_BOUND"], "track_on": v["focus_on"]["TRACKING_BOUND"]} for k, v in payload["after_sh3_summary"].items()},
        "lift": {arm: {"wr": v["would_reject_mean"], "track": v["focus"]["TRACKING_BOUND"]} for arm, v in (payload["lift_summary"].get("arms") or {}).items()},
    }, ensure_ascii=False, indent=2))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="票 SH4：TRACKING_BOUND 误报判定（只报数）")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("logs").set_defaults(func=cmd_logs)
    l0 = sub.add_parser("l0")
    l0.add_argument("--workers", type=int, default=2)
    l0.add_argument("--scene", choices=[s[0] for s in L0_AFTER_SH3] + [LIFT[0]], default=None)
    l0.add_argument("--seed", type=int, default=None)
    l0.set_defaults(func=cmd_l0)
    args = ap.parse_args(argv)
    return int(args.func(args) or 0)


if __name__ == "__main__":
    raise SystemExit(main())
