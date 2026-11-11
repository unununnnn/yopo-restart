#!/usr/bin/env python3
"""票 D1：三机密林回退开/关（seed 0–9）和三机对向变体（seed 0）。

不改场景卡文件。密林用 card_for_tier 拨密度，回退开关走 RunConfig，
所以 crossing-forest.json 的判据和哈希算法不变。对向变体是
scenes/swap-3-geom-fallback.json，只报数。

输出 JSON 写到 --out（默认 out/d1-geometric-fallback/runs.json）。
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
from pathlib import Path

from yopo_l0.forest import card_for_tier
from yopo_l0.geom_fallback import FIELD
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card

ROOT = Path(__file__).resolve().parents[1]
FOREST = ROOT / "scenes/crossing-forest.json"
VARIANT = ROOT / "scenes/swap-3-geom-fallback.json"


def _write_dense_card() -> Path:
    raw = json.loads(FOREST.read_text())
    dense = card_for_tier(raw, "dense")
    path = ROOT / "out" / "d1-geometric-fallback" / "crossing-forest-dense.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(dense, ensure_ascii=False, indent=2) + "\n")
    return path


def _dense_card():
    path = ROOT / "out" / "d1-geometric-fallback" / "crossing-forest-dense.json"
    return load_card(path)


def _row(card, result: dict) -> dict:
    body = float(card.card["criteria"]["body_radius_m"])
    dist = result.get("min_obstacle_distance_m")
    clearance = None if dist is None else float(dist) - body
    block = result.get("crossing") or {}
    if block.get("min_obstacle_clearance_m") is not None:
        clearance = float(block["min_obstacle_clearance_m"])
    geom = result.get(FIELD) or {}
    timing = geom.get("check_time_ms") or {}
    n = int(result["n_drones"])
    arrived = int(result["arrived"])
    return {
        "scene": result["scene"],
        "seed": int(result["seed"]),
        "fallback": bool(geom.get("enabled", False)),
        "outcome": result["outcome"],
        "reason": result["reason"],
        "arrived": arrived,
        "n_drones": n,
        "arrival": f"{arrived}/{n}",
        "arrival_time_s": result.get("arrival_time_s"),
        "collision": int(result["outcome"] == "COLLISION"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "min_obstacle_distance_m": dist,
        "min_obstacle_clearance_m": clearance,
        "fallbacks": int(geom.get("fallbacks", 0)),
        "checks": int(geom.get("checks", 0)),
        "reasons": geom.get("reasons") or {},
        "actions": geom.get("actions") or {},
        "check_p50_ms": timing.get("p50"),
        "check_p95_ms": timing.get("p95"),
        "sim_duration_s": result["sim_duration_s"],
        "final_goal_error_m": result.get("final_goal_error_m"),
        "commit": result["commit"],
        "scene_card_hash": result["scene_card_hash"],
        "commit_horizon_s": geom.get("commit_horizon_s"),
        "inter_drone_distance_m": geom.get("inter_drone_distance_m"),
        "a_max": geom.get("a_max"),
        "local_commits": (block.get("local_commits") if block else None),
    }


def _run_one(spec: tuple) -> dict:
    kind, seed, enabled = spec
    if kind == "dense":
        card = _dense_card()
        cfg = RunConfig(seed=int(seed), geometric_fallback=bool(enabled))
    else:
        card = load_card(VARIANT)
        cfg = RunConfig(seed=int(seed))
    out = run(card, cfg)
    row = _row(card, out.result)
    row["kind"] = kind
    return row


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="0,1,2,3,4,5,6,7,8,9")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--out", default="out/d1-geometric-fallback/runs.json")
    ap.add_argument("--variant-only", action="store_true")
    ap.add_argument("--dense-only", action="store_true")
    a = ap.parse_args(argv)
    seeds = [int(s) for s in a.seeds.split(",") if s != ""]
    if not a.variant_only:
        _write_dense_card()
    specs = []
    if not a.variant_only:
        for enabled in (False, True):
            for seed in seeds:
                specs.append(("dense", seed, enabled))
    if not a.dense_only:
        specs.append(("variant", 0, True))
    if a.jobs <= 1:
        rows = [_run_one(spec) for spec in specs]
    else:
        with mp.get_context("spawn").Pool(a.jobs) as pool:
            rows = pool.map(_run_one, specs, chunksize=1)
    dest = Path(a.out)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(rows, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
