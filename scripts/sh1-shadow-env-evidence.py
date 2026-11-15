#!/usr/bin/env python3
"""票 SH1：影子盾环境证据开关的两臂对照。只报数。

默认关的那一臂不读场景几何。打开的一臂把场景卡光线投射交给影子盾，偏离上游。
两臂的到达、碰撞和轨迹必须相同。墙钟不进结论。评估次数记的是净空查询，不是秒表。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import traceback
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

# 每个进程一个线程。必须在 import numpy / torch 之前。
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "sh1-shadow-env"
SCENES = (
    ("g2-pillar", "scenes/g2-pillar.json", list(range(10))),
    ("crossing-pillars", "scenes/crossing-pillars.json", list(range(10))),
    ("formation-hold", "scenes/formation-hold.json", [0, 1, 2]),
)
NEAR_M = 0.70
OPEN_M = 1.00


def _traj_digest(ticks: list[dict]) -> str:
    h = hashlib.sha256()
    for t in ticks:
        blob = json.dumps(
            [t["uav"], t["p"], t["v"], t["a"], t["yaw"], t["cmd_p"], t["cmd_v"], t["cmd_a"]],
            separators=(",", ":"),
        )
        h.update(blob.encode())
    return h.hexdigest()


def _env_codes(codes) -> bool:
    return any(str(c).startswith("ENV:") for c in (codes or []))


def _slice_rate(ticks: list[dict], pred) -> dict:
    subset = [t for t in ticks if pred(t)]
    n = len(subset)
    if n == 0:
        return {"n": 0, "would_reject": None, "env_reject": None}
    wr = sum(bool(t["would_reject"]) for t in subset) / n
    env = sum(_env_codes(t["reject_codes"]) for t in subset) / n
    return {"n": n, "would_reject": wr, "env_reject": env}


def _histogram(ticks: list[dict]) -> dict[str, int]:
    hist: dict[str, int] = {}
    for t in ticks:
        for code in t.get("reject_codes") or []:
            hist[code] = hist.get(code, 0) + 1
    return dict(sorted(hist.items()))


def _bins(ticks: list[dict], width: float = 0.5) -> list[dict]:
    if not ticks:
        return []
    last = max(float(t["t"]) for t in ticks)
    edges = np.arange(0.0, last + width, width)
    rows = []
    for i in range(len(edges) - 1):
        lo, hi = float(edges[i]), float(edges[i + 1])
        subset = [t for t in ticks if lo <= float(t["t"]) < hi or (i == len(edges) - 2 and float(t["t"]) == hi)]
        if not subset:
            continue
        n = len(subset)
        rows.append({
            "t0": round(lo, 3),
            "t1": round(hi, 3),
            "n": n,
            "would_reject": sum(bool(t["would_reject"]) for t in subset) / n,
            "env_reject": sum(_env_codes(t["reject_codes"]) for t in subset) / n,
            "min_obstacle_m": min(float(t["min_obstacle_m"]) for t in subset),
            "histogram": _histogram(subset),
        })
    return rows


def _collisions(result: dict, body: float) -> dict:
    gap = result.get("min_inter_drone_distance_m")
    obs = int(float(result["min_obstacle_distance_m"]) <= body + 1e-9)
    inter = int(gap is not None and float(gap) <= 2.0 * body + 1e-9)
    return {"obstacle": obs, "inter": inter, "outcome_collision": int(result["outcome"] == "COLLISION")}


def _arm_record(result: dict, ticks: list[dict], body: float, diagnosis: dict | None) -> dict:
    n = len(ticks)
    env_n = sum(_env_codes(t["reject_codes"]) for t in ticks)
    row = {
        "outcome": result["outcome"],
        "reason": result["reason"],
        "arrived": result["arrived"],
        "n_drones": result["n_drones"],
        "arrival_time_s": result["arrival_time_s"],
        "min_obstacle_distance_m": result["min_obstacle_distance_m"],
        "min_inter_drone_distance_m": result["min_inter_drone_distance_m"],
        "collisions": _collisions(result, body),
        "ticks": n,
        "shadow_reject_rate": result["shadow_reject_rate"],
        "shadow_reject_histogram": result["shadow_reject_histogram"],
        "env_reject_rate": None if n == 0 else env_n / n,
        "by_clearance": {
            "open_gt_1m": _slice_rate(ticks, lambda t: float(t["min_obstacle_m"]) > OPEN_M),
            "near_le_0p70m": _slice_rate(ticks, lambda t: float(t["min_obstacle_m"]) <= NEAR_M),
        },
        "time_bins": _bins(ticks),
        "plan_codes": result.get("plan_codes"),
        "upstream_deviation": bool(result.get("upstream_deviation")),
        "shadow_env_evidence": bool(result.get("shadow_env_evidence", False)),
        "shadow_env_views": result.get("shadow_env_views"),
        "shadow_env_clearance_evals": result.get("shadow_env_clearance_evals"),
        "shadow_env_certificate_centers": result.get("shadow_env_certificate_centers"),
        "traj_sha256": _traj_digest(ticks),
    }
    if diagnosis is not None:
        row["sweep_diagnosis"] = diagnosis
    return row


def _frame_tag(frame, p: np.ndarray, radius: float, min_range: float) -> str:
    c = frame.R.T @ (np.asarray(p, dtype=float) - frame.o)
    signs = np.array([[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])
    corners = c + radius * signs
    if float(np.min(corners[:, 2])) <= min_range:
        return "behind_camera" if float(c[2]) <= 0.0 else "within_min_range"
    k = frame.k
    us = k.fx * corners[:, 0] / corners[:, 2] + k.cx
    vs = k.fy * corners[:, 1] / corners[:, 2] + k.cy
    u0, u1 = int(np.floor(us.min())), int(np.ceil(us.max()))
    v0, v1 = int(np.floor(vs.min())), int(np.ceil(vs.max()))
    if not (u0 >= 0 and v0 >= 0 and u1 <= k.width and v1 <= k.height):
        return "outside_fov"
    if u1 <= u0 or v1 <= v0 or not bool(np.all(frame.valid[v0:v1, u0:u1])):
        return "invalid_pixels"
    if float(np.min(frame.depth[v0:v1, u0:u1])) > float(c[2]) + radius:
        return "free"
    return "behind_surface"


def _diagnose_factory():
    """包住 classify_sweep，只计数，不改返回值。"""
    from yopo_swarm_core.freespace import FreeSpace, SpaceVerdict

    reasons: Counter[str] = Counter()
    buckets: Counter[str] = Counter()
    examples: list[dict] = []
    orig = FreeSpace.classify_sweep

    def wrapped(self, traj, now):
        res = orig(self, traj, now)
        reasons[res.reason] += 1
        if res.verdict is SpaceVerdict.UNKNOWN and res.first_bad_time is not None:
            p = np.asarray(traj.evaluate(float(res.first_bad_time)), dtype=float)
            frames = self.fresh_frames(float(now))
            if not frames:
                tag = "NO_FRESH_OBSERVATION"
            else:
                tag = _frame_tag(frames[-1], p, self.cfg.sample_radius, self.cfg.min_range)
            buckets[tag] += 1
            if len(examples) < 6 and reasons["UNKNOWN"] in (1, 50, 200):
                cam = frames[-1].o if frames else None
                examples.append({
                    "t": float(res.first_bad_time),
                    "p": [round(float(x), 4) for x in p],
                    "tag": tag,
                    "cam_dist_m": None if cam is None else round(float(np.linalg.norm(p - cam)), 4),
                })
        elif res.verdict is SpaceVerdict.UNKNOWN:
            buckets[res.reason] += 1
        return res

    def install():
        FreeSpace.classify_sweep = wrapped

    def remove():
        FreeSpace.classify_sweep = orig

    def payload():
        return {
            "classify_sweep_calls": dict(reasons),
            "unknown_first_sample": dict(buckets),
            "examples": examples,
            "note": "计数是 classify_sweep 调用，不是 tick。一次选择会扫运动段和制动分支，门里还会再扫一遍。",
        }

    return install, remove, payload


def _run_arm(card, seed: int, enabled: bool, diagnose: bool):
    from yopo_l0.runner import RunConfig, run

    diagnosis = None
    install = remove = None
    if diagnose:
        install, remove, payload = _diagnose_factory()
        install()
    try:
        out = run(card, RunConfig(seed=seed, shadow=True, shadow_env_evidence=enabled))
    finally:
        if remove is not None:
            remove()
            diagnosis = payload()
    return out, diagnosis


def run_pair(scene_name: str, scene_path: str, seed: int) -> dict:
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    from yopo_l0.scene import load_card

    card = load_card(ROOT / scene_path)
    body = float(card.card["criteria"]["body_radius_m"])
    diagnose = scene_name == "g2-pillar" and seed == 0
    off, diagnosis = _run_arm(card, seed, False, diagnose)
    on, _ = _run_arm(card, seed, True, False)
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
        "scene": scene_name,
        "scene_path": scene_path,
        "scene_card_hash": card.hash,
        "seed": seed,
        "body_radius_m": body,
        "arms_match": True,
        "off": _arm_record(off.result, off.ticks, body, diagnosis),
        "on": _arm_record(on.result, on.ticks, body, None),
    }


def _write(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(row, ensure_ascii=False, indent=2) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", nargs="*", default=None, help="只跑这些场景名，缺省三张卡")
    ap.add_argument("--seeds", nargs="*", type=int, default=None)
    ap.add_argument("--jobs", type=int, default=4)
    args = ap.parse_args()
    jobs = []
    for name, path, seeds in SCENES:
        if args.scenes and name not in args.scenes:
            continue
        for seed in seeds:
            if args.seeds and seed not in args.seeds:
                continue
            jobs.append((name, path, seed))
    OUT.mkdir(parents=True, exist_ok=True)
    pending = []
    for name, path, seed in jobs:
        dest = OUT / f"{name}-seed{seed}.json"
        if dest.exists():
            print(f"skip {dest.name}", flush=True)
            continue
        pending.append((name, path, seed, dest))
    if pending:
        with ProcessPoolExecutor(max_workers=max(1, args.jobs)) as pool:
            futs = {pool.submit(run_pair, n, p, s): (n, s, dest) for n, p, s, dest in pending}
            for fut in as_completed(futs):
                name, seed, dest = futs[fut]
                try:
                    row = fut.result()
                except Exception:
                    print(f"FAIL {name} seed {seed}\n{traceback.format_exc()}", flush=True)
                    raise
                _write(dest, row)
                off_r = row["off"]["shadow_reject_rate"]
                on_r = row["on"]["shadow_reject_rate"]
                print(
                    f"done {name} seed {seed} would_reject {off_r:.3f} -> {on_r:.3f} "
                    f"env {row['off']['env_reject_rate']:.3f} -> {row['on']['env_reject_rate']:.3f}",
                    flush=True,
                )
    print(f"rows in {OUT}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
