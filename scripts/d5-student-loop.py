#!/usr/bin/env python3
"""D5：学生策略对上游，只报数。

场景：g2 柱子、开阔三机对向、开阔三机平行、密度 0.14 单机。seeds 0–9。
不改默认，不冻结，不进 scenes/suite.txt。撞得多也如实报。
学生权重通过环境变量 YOPO_STUDENT_WEIGHTS 传入，不改场景卡。

    .venv/bin/python scripts/d5-student-loop.py \\
        --weights /workspace/yopo-teacher-data/d5/student.pt \\
        --out /workspace/yopo-teacher-data/d5/eval
"""

from __future__ import annotations

import argparse
import json
import os
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

SCENES = (
    ("g2-pillar", ROOT / "scenes/g2-pillar.json", "g2 柱子"),
    ("swap-3-open", ROOT / "scenes/swap-3-open.json", "开阔三机对向"),
    ("open-3drone", ROOT / "scenes/open-3drone.json", "开阔三机平行"),
    ("ego-density-014-solo", ROOT / "scenes/ego-density-014-solo.json", "密度 0.14 单机"),
)
ARMS = ("upstream", "student")
SEEDS = tuple(range(10))
BODY_RADIUS_M = 0.35
DISCLAIMER = "只报数，不冻结新判据。密度场的地图边长和障碍半径是假设，不是论文复现。"


def _classify(result: dict) -> str:
    outcome = result.get("outcome")
    reason = str(result.get("reason") or "")
    if outcome == "SUCCESS":
        return "arrived"
    if outcome == "COLLISION" and "INTER_DRONE" in reason:
        return "inter_drone"
    if outcome == "COLLISION":
        return "obstacle"
    if outcome == "TIMEOUT":
        return "timeout"
    return str(outcome or "unknown")


def _row(key: str, label: str, policy: str, seed: int, result: dict, worker_s: float) -> dict:
    min_obs = result.get("min_obstacle_distance_m")
    min_inter = result.get("min_inter_drone_distance_m")
    clearance = None if min_obs is None else float(min_obs) - BODY_RADIUS_M
    kind = _classify(result)
    n = int(result.get("n_drones") or 0)
    arrived = int(result.get("arrived") or 0)
    return {
        "scene": key,
        "label": label,
        "policy": policy,
        "seed": int(seed),
        "outcome": result.get("outcome"),
        "reason": result.get("reason"),
        "kind": kind,
        "n_drones": n,
        "arrived": arrived,
        "tree_collisions": int(kind == "obstacle"),
        "inter_drone_collisions": int(kind == "inter_drone"),
        "timeouts": int(kind == "timeout"),
        "arrival_time_s": result.get("arrival_time_s"),
        "min_obstacle_distance_m": min_obs,
        "min_clearance_m": clearance,
        "min_inter_drone_distance_m": min_inter,
        "final_goal_error_m": result.get("final_goal_error_m"),
        "student_infer": result.get("student_infer"),
        "network": result.get("network"),
        "scene_card_hash": result.get("scene_card_hash"),
        "commit": result.get("commit"),
        "sim_duration_s": result.get("sim_duration_s"),
        "wall_time_s": result.get("wall_time_s"),
        "worker_s": worker_s,
        "report_only": True,
        "disclaimer": DISCLAIMER,
    }


def _partial(out_dir: Path, key: str, policy: str, seed: int) -> Path:
    return out_dir / "partial" / f"{key}__{policy}__s{seed}.json"


def run_one(key: str, card_path: str, label: str, policy: str, seed: int, weights: str | None, out_dir: str, resume: bool) -> dict:
    dest = _partial(Path(out_dir), key, policy, int(seed))
    if resume and dest.is_file():
        return json.loads(dest.read_text(encoding="utf-8"))
    if policy == "student":
        if not weights:
            raise SystemExit("学生臂需要 --weights")
        os.environ["YOPO_STUDENT_WEIGHTS"] = weights
    elif "YOPO_STUDENT_WEIGHTS" in os.environ:
        # 上游臂不读这个变量。清掉以免测试之外的路径误用。
        os.environ.pop("YOPO_STUDENT_WEIGHTS", None)
    from yopo_l0.runner import RunConfig, run
    from yopo_l0.scene import load_card

    card = load_card(card_path)
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(seed), policy=policy, shadow=True))
    row = _row(key, label, policy, seed, out.result, time.perf_counter() - t0)
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)
    infer = row.get("student_infer") or {}
    infer_s = "—" if infer.get("p95_ms") is None else f"{float(infer['p95_ms']):.2f}"
    print(
        f"{key} {policy} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"tree={row['tree_collisions']} inter={row['inter_drone_collisions']} "
        f"clear={_fmt(row['min_clearance_m'], 3)} inter_m={_fmt(row['min_inter_drone_distance_m'], 3)} "
        f"infer_p95_ms={infer_s} wall={row['wall_time_s']:.1f}s",
        flush=True,
    )
    return row


def _fmt(value, nd: int) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{nd}f}"


def _worker(spec: tuple) -> dict:
    return run_one(*spec)


def _aggregate(rows: list[dict]) -> list[dict]:
    from collections import defaultdict

    groups = defaultdict(list)
    for row in rows:
        groups[(row["scene"], row["policy"])].append(row)
    out = []
    for (scene, policy), items in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        n = len(items)
        drones = max(int(r["n_drones"]) for r in items)
        arrived = sum(int(r["arrived"]) for r in items)
        denom = drones * n
        tree = sum(int(r["tree_collisions"]) for r in items)
        inter = sum(int(r["inter_drone_collisions"]) for r in items)
        timeout = sum(int(r["timeouts"]) for r in items)
        clears = [float(r["min_clearance_m"]) for r in items if r.get("min_clearance_m") is not None]
        gaps = [float(r["min_inter_drone_distance_m"]) for r in items if r.get("min_inter_drone_distance_m") is not None]
        infer = [r.get("student_infer") or {} for r in items]
        p95s = [float(x["p95_ms"]) for x in infer if x.get("p95_ms") is not None]
        means = [float(x["mean_ms"]) for x in infer if x.get("mean_ms") is not None]
        label = items[0].get("label") or scene
        out.append({
            "scene": scene,
            "label": label,
            "policy": policy,
            "seeds": n,
            "arrived": arrived,
            "denom": denom,
            "tree_collisions": tree,
            "inter_drone_collisions": inter,
            "timeouts": timeout,
            "min_clearance_m": min(clears) if clears else None,
            "min_inter_drone_distance_m": min(gaps) if gaps else None,
            "infer_mean_ms": sum(means) / len(means) if means else None,
            "infer_p95_ms": max(p95s) if p95s else None,
        })
    return out


def summarize(groups: list[dict], holdout: dict | None) -> str:
    lines = [
        DISCLAIMER,
        "",
        "| 场景 | 策略 | 到达 | 撞树/障碍 | 机间碰撞 | 超时 | 最小净空 m | 最小机间 m | 推理均值 ms | 推理 p95 ms |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for g in groups:
        lines.append(
            f"| {g['label']} | {g['policy']} | {g['arrived']}/{g['denom']} | {g['tree_collisions']} | "
            f"{g['inter_drone_collisions']} | {g['timeouts']} | {_fmt(g['min_clearance_m'], 3)} | "
            f"{_fmt(g['min_inter_drone_distance_m'], 3)} | {_fmt(g['infer_mean_ms'], 2)} | "
            f"{_fmt(g['infer_p95_ms'], 2)} |"
        )
    if holdout:
        lines += [
            "",
            "留出集轨迹误差（窗口监督，不是飞行闭环）：",
            "",
            f"- 窗口数：{holdout.get('n_windows')}",
            f"- 位置 MSE：{holdout.get('position_mse')}",
            f"- 位置 RMSE m：{holdout.get('position_rmse_m')}",
            f"- 速度 MSE：{holdout.get('velocity_mse')}",
            f"- 偏航 MSE：{holdout.get('yaw_mse')}",
        ]
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="D5 学生 vs 上游，只报数")
    parser.add_argument("--weights", type=Path, default=Path("/workspace/yopo-teacher-data/d5/student.pt"))
    parser.add_argument("--out", type=Path, default=Path("/workspace/yopo-teacher-data/d5/eval"))
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--seeds", default="0-9")
    parser.add_argument("--no-resume", action="store_true")
    parser.add_argument("--holdout", type=Path, default=None, help="可选 train-summary.json，把留出误差抄进总表")
    args = parser.parse_args(argv)
    if args.seeds == "0-9":
        seeds = list(SEEDS)
    else:
        seeds = [int(x) for x in args.seeds.split(",") if x.strip() != ""]
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    weights = Path(args.weights)
    specs = []
    for key, path, label in SCENES:
        for policy in ARMS:
            for seed in seeds:
                w = str(weights) if policy == "student" else None
                specs.append((key, str(path), label, policy, int(seed), w, str(out_dir), not args.no_resume))
    jobs = max(1, min(int(args.jobs), 2))
    rows: list[dict] = []
    print(f"共 {len(specs)} 场，jobs={jobs}", flush=True)
    if jobs == 1:
        for spec in specs:
            rows.append(run_one(*spec))
    else:
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            futs = [pool.submit(_worker, spec) for spec in specs]
            for fut in as_completed(futs):
                rows.append(fut.result())
    rows.sort(key=lambda r: (r["scene"], r["policy"], int(r["seed"])))
    holdout = None
    hold_path = args.holdout
    if hold_path is None:
        cand = weights.parent / "train-summary.json"
        hold_path = cand if cand.is_file() else None
    if hold_path is not None and Path(hold_path).is_file():
        summary = json.loads(Path(hold_path).read_text(encoding="utf-8"))
        holdout = summary.get("holdout")
    groups = _aggregate(rows)
    table = summarize(groups, holdout)
    payload = {
        "schema": "yopo-d5-student-loop/1",
        "report_only": True,
        "disclaimer": DISCLAIMER,
        "weights": str(weights) if weights.is_file() else None,
        "holdout": holdout,
        "groups": groups,
        "runs": rows,
        "table": table,
    }
    dest = out_dir / "d5-student-loop-runs.json"
    dest.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (out_dir / "table.txt").write_text(table, encoding="utf-8")
    print(table, end="")
    print(f"写出 {dest}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
