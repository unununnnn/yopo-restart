"""E-B2 只报数：g2 柱子与三机密林，后段开 / 关，种子 0..9。

不进 suite.txt，不设冻结门。关的一列是同一几何、同一任务，只把开关关掉。
"""

from __future__ import annotations

import json
import multiprocessing as mp
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BODY_R = 0.35


def _jobs() -> list[dict]:
    jobs = []
    for seed in range(10):
        jobs.append({"label": "g2-off", "seed": seed, "scene": "scenes/g2-pillar.json", "enabled": None})
        jobs.append({"label": "g2-on", "seed": seed, "scene": "scenes/g2-pillar-bspline.json", "enabled": None})
        jobs.append({"label": "forest-off", "seed": seed, "scene": "scenes/crossing-forest-dense-bspline.json", "enabled": False})
        jobs.append({"label": "forest-on", "seed": seed, "scene": "scenes/crossing-forest-dense-bspline.json", "enabled": True})
    return jobs


def _chunks(items: list[dict], n: int) -> list[list[dict]]:
    buckets: list[list[dict]] = [[] for _ in range(n)]
    for i, item in enumerate(items):
        buckets[i % n].append(item)
    return [b for b in buckets if b]


def _load(job: dict):
    from yopo_l0.scene import load_card

    path = ROOT / job["scene"]
    if job["enabled"] is None:
        return load_card(path)
    raw = json.loads(path.read_text())
    params = dict(raw.get("policy_params") or {})
    backend = dict(params.get("backend") or {})
    backend["enabled"] = bool(job["enabled"])
    params["backend"] = backend
    raw["policy_params"] = params
    raw["name"] = f"{raw['name']}-{'on' if job['enabled'] else 'off'}"
    tmp = Path(tempfile.mkdtemp(prefix="e-b2-")) / "card.json"
    tmp.write_text(json.dumps(raw))
    return load_card(tmp)


def _compact(job: dict, result: dict) -> dict:
    dist = result.get("min_obstacle_distance_m")
    clearance = None if dist is None else float(dist) - BODY_R
    gap = result.get("min_inter_drone_distance_m")
    block = result.get("crossing") or {}
    if "obstacle_collisions" in block:
        obstacle_hit = int(block["obstacle_collisions"])
        inter_hit = int(block["inter_drone_collisions"])
    else:
        obstacle_hit = int(dist is not None and dist <= BODY_R + 1e-9)
        inter_hit = int(gap is not None and gap <= 2.0 * BODY_R + 1e-9)
    return {
        "label": job["label"],
        "seed": int(job["seed"]),
        "scene": result.get("scene"),
        "outcome": result.get("outcome"),
        "arrived": result.get("arrived"),
        "n_drones": result.get("n_drones"),
        "arrival_time_s": result.get("arrival_time_s"),
        "min_obstacle_distance_m": dist,
        "min_obstacle_clearance_m": clearance,
        "obstacle_collisions": obstacle_hit,
        "inter_drone_collisions": inter_hit,
        "min_inter_drone_distance_m": gap,
        "sim_duration_s": result.get("sim_duration_s"),
        "wall_time_s": result.get("wall_time_s"),
        "backend": result.get("backend"),
        "shadow_reject_rate": result.get("shadow_reject_rate"),
    }


def _worker(jobs: list[dict]) -> list[dict]:
    from yopo_l0.runner import RunConfig, run

    rows = []
    for job in jobs:
        card = _load(job)
        out = run(card, RunConfig(seed=int(job["seed"]), shadow=True))
        row = _compact(job, out.result)
        rows.append(row)
        partial = Path("/tmp/e-b2")
        partial.mkdir(exist_ok=True)
        (partial / f"{row['label']}-{row['seed']}.json").write_text(json.dumps(row))
        print(
            f"{row['label']} seed={row['seed']} outcome={row['outcome']} "
            f"arrived={row['arrived']}/{row['n_drones']} hit={row['obstacle_collisions']} "
            f"clear={row['min_obstacle_clearance_m']} wall={row['wall_time_s']:.1f}s",
            flush=True,
        )
    return rows


def _pct(values: list[float], q: float) -> float | None:
    if not values:
        return None
    import numpy as np

    return float(np.percentile(np.asarray(values, dtype=float), q))


def _summary(rows: list[dict]) -> dict:
    labels = []
    for row in rows:
        if row["label"] not in labels:
            labels.append(row["label"])
    out = {}
    for label in labels:
        group = [r for r in rows if r["label"] == label]
        drones = sum(int(r["n_drones"] or 0) for r in group)
        arrived = sum(int(r["arrived"] or 0) for r in group)
        hits = sum(int(r["obstacle_collisions"]) for r in group)
        inter = sum(int(r["inter_drone_collisions"]) for r in group)
        clears = [float(r["min_obstacle_clearance_m"]) for r in group if r["min_obstacle_clearance_m"] is not None]
        times: list[float] = []
        iters: list[float] = []
        calls = applied = 0
        for r in group:
            backend = r.get("backend") or {}
            calls += int(backend.get("calls") or 0)
            applied += int(backend.get("applied") or 0)
            block_t = backend.get("time_ms") or {}
            block_i = backend.get("iterations") or {}
            times.extend(float(v) for v in (block_t.get("samples") or []))
            iters.extend(float(v) for v in (block_i.get("samples") or []))
        out[label] = {
            "runs": len(group),
            "arrived": arrived,
            "drones": drones,
            "obstacle_collisions": hits,
            "inter_drone_collisions": inter,
            "min_clearance_m": None if not clears else min(clears),
            "backend_calls": calls,
            "backend_applied": applied,
            "backend_time_ms_p50": _pct(times, 50),
            "backend_time_ms_p95": _pct(times, 95),
            "iterations_p50": _pct(iters, 50),
            "iterations_mean": None if not iters else float(sum(iters) / len(iters)),
        }
    return out


def main() -> None:
    jobs = _jobs()
    # 两个进程。后段预算是墙钟，再挤就会把迭代截短，开/关对比跟着变。
    ctx = mp.get_context("spawn")
    with ctx.Pool(2) as pool:
        parts = pool.map(_worker, _chunks(jobs, 2))
    rows = [row for part in parts for row in part]
    rows.sort(key=lambda r: (r["label"], r["seed"]))
    payload = {"rows": rows, "summary": _summary(rows)}
    for row in rows:
        block = row.get("backend")
        if not isinstance(block, dict):
            continue
        for key in ("time_ms", "iterations"):
            part = block.get(key)
            if isinstance(part, dict):
                part.pop("samples", None)
    dest = ROOT / "docs" / "research" / "e-b2-bspline-runs.json"
    dest.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(payload["summary"], indent=2, ensure_ascii=False))
    print(f"写入 {dest}")


if __name__ == "__main__":
    main()
