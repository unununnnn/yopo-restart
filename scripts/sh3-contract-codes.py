#!/usr/bin/env python3
"""票 SH3：L0 g2-pillar / crossing-pillars 契约码修正开关对照。只报数。

两臂都打开 shadow_env_evidence，否则 ENV:UNKNOWN 会把 would_reject 打满，看不出契约修正。
默认关的一臂不做契约对齐。打开的一臂按上游语义过滤误报码，偏离上游。
不改判据，不改设定点。墙钟不进结论。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "sh3-contract-codes"
DOC = ROOT / "docs" / "research" / "sh3-contract-codes-runs.json"
L0_SCENES = (
    ("g2-pillar", "scenes/g2-pillar.json", list(range(10))),
    ("crossing-pillars", "scenes/crossing-pillars.json", list(range(10))),
)
FOCUS_CODES = (
    "TIME:SWITCH_NOT_IN_FUTURE",
    "TRANSITION:HEAD_DISCONTINUITY",
    "TRANSITION:TERMINAL_NOT_HOLDABLE",
    "TRANSITION:MOTION_KINEMATICS",
    "TRANSITION:BRANCH_UNREACHABLE",
    "TRACKING_BOUND",
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


def _arm_record(result: dict, ticks: list[dict], body: float) -> dict:
    from yopo_l0.shadow import aggregate
    from yopo_l0.shadow_sitl import attribution

    rate, hist = aggregate(ticks)
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
        "shadow_reject_rate": rate,
        "shadow_reject_histogram": hist,
        "traj_sha256": _traj_digest(ticks),
        "attribution": attribution(ticks),
        "focus_counts": {c: int(hist.get(c, 0)) for c in FOCUS_CODES},
    }


def run_l0_pair(scene_name: str, scene_path: str, seed: int) -> dict:
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    from yopo_l0.runner import RunConfig, run
    from yopo_l0.scene import load_card

    card = load_card(ROOT / scene_path)
    body = float(card.card["criteria"]["body_radius_m"])
    # 两臂都开场景证据，隔离契约码。对齐开关才是本票对照。
    off = run(card, RunConfig(seed=seed, shadow=True, shadow_env_evidence=True, shadow_contract_align=False))
    on = run(card, RunConfig(seed=seed, shadow=True, shadow_env_evidence=True, shadow_contract_align=True))
    same_traj = _traj_digest(off.ticks) == _traj_digest(on.ticks)
    same_flight = (
        off.result["outcome"] == on.result["outcome"]
        and off.result["arrived"] == on.result["arrived"]
        and off.result["arrival_time_s"] == on.result["arrival_time_s"]
        and off.result["min_obstacle_distance_m"] == on.result["min_obstacle_distance_m"]
        and off.result["min_inter_drone_distance_m"] == on.result["min_inter_drone_distance_m"]
        and off.result["plan_codes"] == on.result["plan_codes"]
    )
    if not (same_traj and same_flight):
        raise RuntimeError(f"{scene_name} seed {seed} 两臂轨迹或到达/碰撞不一致")
    return {
        "layer": "l0",
        "scene": scene_name,
        "scene_path": scene_path,
        "scene_card_hash": card.hash,
        "seed": seed,
        "same_traj": same_traj,
        "same_flight": same_flight,
        "off": _arm_record(off.result, off.ticks, body),
        "on": _arm_record(on.result, on.ticks, body),
    }


def _aggregate(rows: list[dict]) -> dict:
    n = len(rows)
    def _sum_arm(arm: str, key: str):
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
        "focus_off": {c: _sum_arm("off", c) for c in FOCUS_CODES},
        "focus_on": {c: _sum_arm("on", c) for c in FOCUS_CODES},
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


def cmd_l0(args) -> int:
    jobs = []
    for name, path, seeds in L0_SCENES:
        for seed in seeds:
            jobs.append((name, path, seed))
    if args.scene:
        jobs = [j for j in jobs if j[0] == args.scene]
    if args.seed is not None:
        jobs = [j for j in jobs if j[2] == args.seed]
    workers = max(1, int(args.workers))
    print(f"l0 jobs={len(jobs)} workers={workers}", flush=True)
    rows = []
    failures = []
    if workers == 1:
        for job in jobs:
            try:
                row = run_l0_pair(*job)
                dest = OUT / row["scene"] / f"seed{row['seed']}.json"
                _write(dest, row)
                rows.append(row)
                print(
                    f"ok {row['scene']} seed {row['seed']} "
                    f"reject {row['off']['shadow_reject_rate']:.3f} -> {row['on']['shadow_reject_rate']:.3f} "
                    f"{row['off']['outcome']} arrived={row['off']['arrived']}",
                    flush=True,
                )
            except Exception as exc:
                failures.append({"job": job, "error": f"{type(exc).__name__}: {exc}"})
                traceback.print_exc()
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futs = {pool.submit(run_l0_pair, *job): job for job in jobs}
            for fut in as_completed(futs):
                job = futs[fut]
                try:
                    row = fut.result()
                    dest = OUT / row["scene"] / f"seed{row['seed']}.json"
                    _write(dest, row)
                    rows.append(row)
                    print(
                        f"ok {row['scene']} seed {row['seed']} "
                        f"reject {row['off']['shadow_reject_rate']:.3f} -> {row['on']['shadow_reject_rate']:.3f} "
                        f"{row['off']['outcome']} arrived={row['off']['arrived']}",
                        flush=True,
                    )
                except Exception as exc:
                    failures.append({"job": job, "error": f"{type(exc).__name__}: {exc}"})
                    traceback.print_exc()
    rows.sort(key=lambda r: (r["scene"], r["seed"]))
    summary = {name: _aggregate([r for r in rows if r["scene"] == name]) for name, *_ in L0_SCENES}
    payload = {
        "note": "票 SH3 L0 契约码修正对照。两臂都开场景证据。打开 shadow_contract_align 偏离上游。只报数。",
        "rows": rows,
        "summary": summary,
        "failures": failures,
    }
    _write(OUT / "l0-summary.json", payload)
    print(json.dumps({"summary": summary, "failures": failures}, ensure_ascii=False, indent=2))
    return 0 if not failures else 1


def cmd_danger(_args) -> int:
    """故意让轨迹穿过 g2 柱子，确认修正开关打开后影子盾仍拒绝。"""
    import numpy as np

    from yopo_l0.scene import Box, Ground
    from yopo_l0.sensor import CameraConfig, DepthCamera
    from yopo_l0.shadow import ENV_OCCUPIED, DroneShadow
    from yopo_swarm_core.plan import quintic
    from yopo_swarm_core.trajectory import PiecewisePolynomial

    pillar = (
        Box("pillar", (2.6, -0.4, 0.0), (3.2, 0.4, 2.8)),
        Ground(0.0),
    )
    origin = np.array([0.0, 0.0, 1.5])
    dest = np.array([5.0, 0.0, 1.5])
    z = np.zeros(3)
    # 5 m / 4 s 的静止到静止五次多项式，峰值速度和加速度都在上游限幅内。
    # 这样拒绝只来自穿柱占用，而不是误报的运动学码。
    traj = PiecewisePolynomial(0.0, [quintic(np.stack([origin, z, z]), np.stack([dest, z, z]), 4.0)])

    class Wrap:
        def __init__(self, traj):
            self.traj = traj

    class State:
        def __init__(self, p):
            self.p = np.asarray(p, dtype=float)
            self.yaw = 0.0
            self.t = 0.2

    cam = DepthCamera(
        CameraConfig(width=160, height=120, hfov_deg=80.0, vfov_deg=60.0, noise_std_m=0.0),
        pillar,
        __import__("numpy").random.default_rng(0),
    )
    frame = cam.capture(origin, 0.0, 0.2)
    shadow = DroneShadow("uav0", origin, dest, {}, scene_obstacles=pillar, contract_align=True)
    shadow.on_selection(Wrap(traj), frame, State(origin), 0.2)
    codes = shadow.on_tick(State(origin), origin, 0.2)
    row = {
        "case": "straight-through-g2-pillar",
        "contract_align": True,
        "scene_evidence": True,
        "origin": origin.tolist(),
        "dest": dest.tolist(),
        "selection": list(shadow.selection),
        "tick_codes": list(codes),
        "would_reject": bool(codes),
        "occupied": ENV_OCCUPIED in codes,
    }
    _write(OUT / "danger.json", row)
    print(json.dumps(row, ensure_ascii=False, indent=2))
    if not row["occupied"] or not row["would_reject"]:
        raise SystemExit("危险卡未被拒绝")
    return 0


def cmd_merge(_args) -> int:
    l0 = json.loads((OUT / "l0-summary.json").read_text()) if (OUT / "l0-summary.json").exists() else {}
    danger = json.loads((OUT / "danger.json").read_text()) if (OUT / "danger.json").exists() else {}
    payload = {
        "note": "票 SH3：影子盾契约码误报判定（只报数）。打开 shadow_contract_align 偏离上游。",
        "l0": l0,
        "danger": danger,
    }
    DOC.parent.mkdir(parents=True, exist_ok=True)
    DOC.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
    print(f"wrote {DOC}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="票 SH3 L0 契约码对照")
    sub = ap.add_subparsers(dest="cmd", required=True)
    l0 = sub.add_parser("l0")
    l0.add_argument("--workers", type=int, default=2)
    l0.add_argument("--scene", choices=[s[0] for s in L0_SCENES], default=None)
    l0.add_argument("--seed", type=int, default=None)
    l0.set_defaults(func=cmd_l0)
    d = sub.add_parser("danger")
    d.set_defaults(func=cmd_danger)
    m = sub.add_parser("merge")
    m.set_defaults(func=cmd_merge)
    args = ap.parse_args(argv)
    return int(args.func(args) or 0)


if __name__ == "__main__":
    raise SystemExit(main())
