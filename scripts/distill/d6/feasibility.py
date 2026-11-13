#!/usr/bin/env python3
"""D6 可行性：把仓库里 1.0 m 的教师样本做高度平移，重渲深度，按 D5b r4 的超参缩小数据规模训练。

不启动 EGO-Swarm 容器。新选项是 convert_run 的 altitude_shift_m，默认 0。
评测用 D5b r4 的学生接口（knots=position、yaw_source=ego_lookahead、
accel_source=reference、vertical=hold_goal_z），只在这个脚本里打开，不改默认。

    .venv/bin/python scripts/distill/d6/feasibility.py all --out /workspace/yopo-teacher-data/d6
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SAMPLE = ROOT / "data" / "teacher" / "sample"
# 与 D5b r4 评测同一组接口。默认策略仍是 D5，这里只在本进程覆盖。
R4_EVAL_PARAMS = {
    "knots": "position",
    "yaw_source": "ego_lookahead",
    "accel_source": "reference",
    "vertical": "hold_goal_z",
}
# 与 scripts/distill/d5b/train_r4.py 的 FitConfig 相同。缩小的是场次数，不是这些数。
R4_FIT = {
    "lr": 1e-3,
    "batch_size": 64,
    "epochs": 5,
    "seed": 0,
    "num_threads": 2,
    "loss_weights": (1.0, 0.25, 0.05),
    "planar": True,
    "near_weight": 3.0,
    "near_depth_m": 1.2,
    "shuffle_batches": 64,
}
SCENES = (
    ("open-3drone", ROOT / "scenes" / "open-3drone.json", "开阔三机平行"),
    ("swap-3-open", ROOT / "scenes" / "swap-3-open.json", "开阔三机对向"),
    ("g2-pillar", ROOT / "scenes" / "g2-pillar.json", "g2 柱子"),
)
SEEDS = (0, 1, 2, 3, 4)
BODY_RADIUS_M = 0.35
KINEMATIC = ("state", "position_body", "velocity_body", "yaw_rel", "neighbor_rel", "neighbor_mask")


def split_sample_ids(ids: list[str]) -> tuple[list[str], list[str]]:
    """仓库样本全是 seed 0。D5 的 seed % 5 == 0 会把每一场都放进留出。

    缩小规模改为：编号排序后，下标 % 5 == 0 留出，其余训练。同一场的窗口不拆开。
    """
    ordered = sorted(ids)
    hold = [i for n, i in enumerate(ordered) if n % 5 == 0]
    train = [i for i in ordered if i not in set(hold)]
    return train, hold


def successful_sample_ids(sample_root: Path = SAMPLE) -> list[str]:
    ids = []
    for record_path in sorted(sample_root.glob("*/record.json")):
        record = json.loads(record_path.read_text(encoding="utf-8"))
        if record.get("metrics", {}).get("success") is True:
            ids.append(record_path.parent.name)
    return ids


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _convert_call(run_dir: Path, shift: float):
    from yopo_l0.distill import convert_run

    if shift == 0.0:
        return convert_run(run_dir)
    return convert_run(run_dir, altitude_shift_m=shift)


def convert_samples(out_dir: Path, shift: float, sample_root: Path = SAMPLE) -> dict:
    from yopo_l0.distill import save_batch

    out_dir.mkdir(parents=True, exist_ok=True)
    win_dir = out_dir / "windows"
    win_dir.mkdir(parents=True, exist_ok=True)
    ids = successful_sample_ids(sample_root)
    train_ids, hold_ids = split_sample_ids(ids)
    files = {}
    t0 = time.perf_counter()
    n_windows = 0
    for run_id in ids:
        batch = _convert_call(sample_root / run_id, shift)
        if batch.n == 0:
            raise SystemExit(f"{run_id} 成功场没有窗口：{batch.skipped}")
        path = save_batch(batch, win_dir / f"{run_id}.npz")
        files[run_id] = {"path": str(path), "sha256": _sha256(path), "n": batch.n}
        n_windows += batch.n
        print(f"convert shift={shift} {run_id} n={batch.n}", flush=True)
    manifest = {
        "schema": "yopo-student-d6-sample/1",
        "altitude_shift_m": shift,
        "sample_root": str(sample_root),
        "n_success_runs": len(ids),
        "n_windows": n_windows,
        "hold_rule": "样本全是 seed 0。排序后下标 % 5 == 0 留出。不是 D5 的 seed % 5 == 0。",
        "train_ids": train_ids,
        "hold_ids": hold_ids,
        "files": files,
        "wall_s": time.perf_counter() - t0,
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def compare_windows(dir_a: Path, dir_b: Path) -> dict:
    """同一批样本、两种平移。机体坐标应只差舍入，深度应不同。"""
    from yopo_l0.distill import load_batch

    man_a = json.loads((dir_a / "manifest.json").read_text(encoding="utf-8"))
    rows = []
    kin_max = 0.0
    depth_abs = 0.0
    depth_n = 0
    depth_changed = 0
    for run_id in man_a["train_ids"] + man_a["hold_ids"]:
        a = load_batch(dir_a / "windows" / f"{run_id}.npz")
        b = load_batch(dir_b / "windows" / f"{run_id}.npz")
        scene_max = 0.0
        for name in KINEMATIC:
            diff = np.max(np.abs(getattr(a, name).astype(np.float64) - getattr(b, name).astype(np.float64)))
            scene_max = max(scene_max, float(diff))
        kin_max = max(kin_max, scene_max)
        delta = np.abs(a.depth.astype(np.float64) - b.depth.astype(np.float64))
        depth_abs += float(delta.sum())
        depth_n += int(delta.size)
        depth_changed += int(np.count_nonzero(delta))
        rows.append({
            "id": run_id,
            "n": a.n,
            "kinematic_max_abs": scene_max,
            "depth_changed_frac": float(np.count_nonzero(delta) / delta.size),
        })
    return {
        "kinematic_max_abs": kin_max,
        "depth_mean_abs": depth_abs / depth_n if depth_n else None,
        "depth_changed_frac": depth_changed / depth_n if depth_n else None,
        "runs": rows,
    }


def train_student(data_dir: Path, out_dir: Path) -> dict:
    from yopo_l0.student import load_checkpoint
    from yopo_l0.student_fit import FitConfig, train_from_paths

    manifest = json.loads((data_dir / "manifest.json").read_text(encoding="utf-8"))
    win = data_dir / "windows"
    train_paths = [win / f"{i}.npz" for i in manifest["train_ids"]]
    hold_paths = [win / f"{i}.npz" for i in manifest["hold_ids"]]
    cfg = FitConfig(**R4_FIT)
    summary = train_from_paths(train_paths, hold_paths, out_dir, cfg=cfg)
    weights = Path(summary["weights"])
    model, _meta = load_checkpoint(weights, enabled=True)
    n_param = sum(int(p.numel()) for p in model._module().parameters())
    summary["n_parameters"] = n_param
    summary["altitude_shift_m"] = manifest["altitude_shift_m"]
    summary["train_ids"] = manifest["train_ids"]
    summary["hold_ids"] = manifest["hold_ids"]
    summary["n_train_windows"] = sum(manifest["files"][i]["n"] for i in manifest["train_ids"])
    summary["n_hold_windows"] = sum(manifest["files"][i]["n"] for i in manifest["hold_ids"])
    (out_dir / "d6-train.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "weights_sha256": summary.get("weights_sha256"),
        "n_parameters": n_param,
        "holdout": summary.get("holdout"),
        "wall_s": summary.get("wall_s"),
    }, ensure_ascii=False), flush=True)
    return summary


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


def eval_one(spec: dict) -> dict:
    """单场闭环。在子进程里把学生接口改成 r4 的四个值，不写场景卡。"""
    os.environ["OMP_NUM_THREADS"] = "1"
    os.environ["YOPO_STUDENT_WEIGHTS"] = spec["weights"]
    import torch

    torch.set_num_threads(1)
    from yopo_l0.policy import StudentPolicy
    from yopo_l0.runner import RunConfig, run
    from yopo_l0.scene import load_card

    StudentPolicy.DEFAULTS = {**StudentPolicy.DEFAULTS, **R4_EVAL_PARAMS}
    t0 = time.perf_counter()
    out = run(load_card(spec["card"]), RunConfig(seed=int(spec["seed"]), policy="student", shadow=True))
    result = out.result
    kind = _classify(result)
    min_obs = result.get("min_obstacle_distance_m")
    clearance = None if min_obs is None else float(min_obs) - BODY_RADIUS_M
    row = {
        "scene": spec["scene"],
        "label": spec["label"],
        "seed": int(spec["seed"]),
        "outcome": result.get("outcome"),
        "reason": result.get("reason"),
        "kind": kind,
        "n_drones": int(result.get("n_drones") or 0),
        "arrived": int(result.get("arrived") or 0),
        "obstacle": int(kind == "obstacle"),
        "inter_drone": int(kind == "inter_drone"),
        "timeout": int(kind == "timeout"),
        "min_obstacle_distance_m": min_obs,
        "min_clearance_m": clearance,
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "final_goal_error_m": result.get("final_goal_error_m"),
        "sim_duration_s": result.get("sim_duration_s"),
        "wall_time_s": result.get("wall_time_s"),
        "worker_s": time.perf_counter() - t0,
        "student_params": R4_EVAL_PARAMS,
        "weights": spec["weights"],
    }
    dest = Path(spec["dest"])
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)
    print(
        f"{row['scene']} seed={row['seed']} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"clear={row['min_clearance_m']} wall={row['worker_s']:.1f}s",
        flush=True,
    )
    return row


def _aggregate(rows: list[dict]) -> list[dict]:
    from collections import defaultdict

    groups: dict[str, list[dict]] = defaultdict(list)
    for row in rows:
        groups[row["scene"]].append(row)
    order = [key for key, _path, _label in SCENES]
    out = []
    for key in order:
        items = groups.get(key) or []
        if not items:
            continue
        n = len(items)
        drones = max(int(r["n_drones"]) for r in items)
        clears = [float(r["min_clearance_m"]) for r in items if r.get("min_clearance_m") is not None]
        out.append({
            "scene": key,
            "label": items[0]["label"],
            "seeds": n,
            "arrived": sum(int(r["arrived"]) for r in items),
            "denom": drones * n,
            "obstacle": sum(int(r["obstacle"]) for r in items),
            "inter_drone": sum(int(r["inter_drone"]) for r in items),
            "timeout": sum(int(r["timeout"]) for r in items),
            "min_clearance_m": min(clears) if clears else None,
        })
    return out


def eval_student(weights: Path, out_dir: Path, *, jobs: int) -> dict:
    import multiprocessing
    from concurrent.futures import ProcessPoolExecutor, as_completed

    out_dir.mkdir(parents=True, exist_ok=True)
    specs = []
    rows = []
    for key, card, label in SCENES:
        for seed in SEEDS:
            dest = out_dir / f"{key}-s{seed}.json"
            if dest.is_file():
                rows.append(json.loads(dest.read_text(encoding="utf-8")))
                continue
            specs.append({
                "scene": key,
                "label": label,
                "card": str(card),
                "seed": seed,
                "weights": str(weights),
                "dest": str(dest),
            })
    if specs:
        ctx = multiprocessing.get_context("spawn")
        with ProcessPoolExecutor(max_workers=max(1, jobs), mp_context=ctx) as pool:
            futs = [pool.submit(eval_one, spec) for spec in specs]
            for fut in as_completed(futs):
                rows.append(fut.result())
    rows.sort(key=lambda r: (r["scene"], r["seed"]))
    summary = {
        "weights": str(weights),
        "weights_sha256": _sha256(weights),
        "student_params": R4_EVAL_PARAMS,
        "seeds": list(SEEDS),
        "rows": rows,
        "groups": _aggregate(rows),
        "report_only": True,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for group in summary["groups"]:
        print(json.dumps(group, ensure_ascii=False), flush=True)
    return summary


def _tag(shift: float) -> str:
    return f"shift{int(round(float(shift) * 100)):03d}"


def run_all(out_root: Path, *, jobs: int, shifts: tuple[float, ...]) -> dict:
    reports = {}
    for shift in shifts:
        tag = _tag(shift)
        data = out_root / tag
        train_dir = data / "train"
        if not (data / "manifest.json").is_file():
            convert_samples(data, shift)
        if not (train_dir / "student.pt").is_file():
            train_student(data, train_dir)
        eval_dir = data / "eval"
        if not (eval_dir / "summary.json").is_file():
            weights = train_dir / "student.pt"
            eval_student(weights, eval_dir, jobs=jobs)
        reports[tag] = {
            "shift": shift,
            "manifest": str(data / "manifest.json"),
            "train": str(train_dir / "d6-train.json"),
            "eval": str(eval_dir / "summary.json"),
        }
    compare = None
    if (out_root / "shift000" / "manifest.json").is_file() and (out_root / "shift050" / "manifest.json").is_file():
        compare = compare_windows(out_root / "shift000", out_root / "shift050")
        (out_root / "shift-compare.json").write_text(
            json.dumps(compare, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    payload = {"reports": reports, "compare": compare}
    (out_root / "feasibility.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return payload


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="D6 高度平移可行性实验")
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_all = sub.add_parser("all")
    p_all.add_argument("--out", type=Path, default=Path("/workspace/yopo-teacher-data/d6"))
    p_all.add_argument("--jobs", type=int, default=2)
    p_conv = sub.add_parser("convert")
    p_conv.add_argument("--shift", type=float, required=True)
    p_conv.add_argument("--out", type=Path, required=True)
    p_train = sub.add_parser("train")
    p_train.add_argument("--data", type=Path, required=True)
    p_train.add_argument("--out", type=Path, required=True)
    p_eval = sub.add_parser("eval")
    p_eval.add_argument("--weights", type=Path, required=True)
    p_eval.add_argument("--out", type=Path, required=True)
    p_eval.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args(argv)
    if args.cmd == "all":
        run_all(args.out, jobs=args.jobs, shifts=(0.0, 0.5))
    elif args.cmd == "convert":
        convert_samples(args.out, float(args.shift))
    elif args.cmd == "train":
        train_student(args.data, args.out)
    elif args.cmd == "eval":
        eval_student(args.weights, args.out, jobs=args.jobs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
