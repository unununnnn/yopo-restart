#!/usr/bin/env python3
"""D6e：起步结点沿目标位移改写的小规模对照。只报数。

数据是仓库 ``data/teacher/sample`` 里成功的场，不是 box 上的全量 d6b。
这些场种子都是 0，留出用可行性脚本的下标 % 5，不用 D5 的 ``seed % 5``
（那条会把样本全部放进留出）。超参是 D5b r4 的 FitConfig，epoch 仍是 5。
关臂不改写标签；开臂改写起步静止段的位置、速度结点。不打开 D6d 的删窗开关。

闭环走 ``eval_altitudes.retarget_card``：1.0 m 与 1.5 m，``track_clearance=off``，
影子盾开。不改 ``StudentPolicy.DEFAULTS``，不写场景卡。

另外用 D6c 那种沿目标直线替换 ``predict``，确认改写后的 0.2 s 结点（0.266 m）
能让直线参考在开阔三机上到达。

    .venv/bin/python scripts/distill/d6e/compare.py all \\
        --out /workspace/yopo-ops/d6e --jobs 6
"""

from __future__ import annotations

import argparse
import importlib.util
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
GOAL_SPEED_MPS = 0.2
GOAL_TOLERANCE_M = 0.4
OPEN_REF_SEEDS = [0, 1, 2]
PROTOCOL_SEEDS = {
    "open-3drone": list(range(10)),
    "g2-pillar": list(range(5)),
}
DISCLAIMER = (
    "只报数。report-only。"
    "起步结点改写开关默认关，这里的开臂只存在于这次转换。"
    "不打开 D6d 的删起步窗开关。"
    "不改 StudentPolicy 的默认，不改场景卡，不把 track_clearance 写成库默认。"
    "全量 d2b / d6-z150 不在这台机器上，数字来自仓库里的成功样本。"
    "打开后结果标「偏离上游」。"
)


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    if spec is None or spec.loader is None:
        raise SystemExit(f"无法加载 {path}")
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _pipeline():
    return _load("d6e_pipeline", ROOT / "scripts" / "distill" / "d6" / "pipeline.py")


def _feasibility():
    return _load("d6e_feasibility", ROOT / "scripts" / "distill" / "d6" / "feasibility.py")


def _eval():
    return _load("d6e_eval_altitudes", ROOT / "scripts" / "distill" / "d6" / "eval_altitudes.py")


def _diagnose():
    return _load("d6e_d6c_diagnose", ROOT / "scripts" / "distill" / "d6c" / "diagnose.py")


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    body = _pipeline()._jsonable(payload)
    temporary.write_text(json.dumps(body, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def convert_arm(out_dir: Path, *, rewrite: bool, jobs: int) -> dict:
    pipe = _pipeline()
    rows = pipe.scan_roots([SAMPLE], None)
    manifest = pipe.convert_records(
        rows,
        out_dir,
        jobs=int(jobs),
        max_windows=None,
        limit=None,
        min_clearance_m=None,
        teacher_dirs=[SAMPLE],
        drop_start_still=False,
        rewrite_start_knots=bool(rewrite),
        start_knot_speed_mps=float(pipe.START_KNOT_SPEED_MPS),
    )
    return manifest


def train_arm(manifest: dict, out_dir: Path) -> dict:
    from yopo_l0.student_fit import FitConfig, train_from_paths

    pipe = _pipeline()
    feas = _feasibility()
    files = manifest["files"]
    train_ids, hold_ids = feas.split_sample_ids(list(files))
    if not train_ids:
        raise SystemExit("训练场次是空的")
    train_paths = [Path(files[run_id]["path"]) for run_id in train_ids]
    hold_paths = [Path(files[run_id]["path"]) for run_id in hold_ids]
    fit = dict(pipe.R4_FIT)
    train_dir = out_dir / "train"
    with pipe._TupleJson():
        summary = train_from_paths(train_paths, hold_paths, train_dir, cfg=FitConfig(**fit))
    summary["train_ids"] = train_ids
    summary["hold_ids"] = hold_ids
    summary["n_train_windows"] = sum(int(files[i]["n"]) for i in train_ids)
    summary["n_hold_windows"] = sum(int(files[i]["n"]) for i in hold_ids)
    summary["n_start_still_dropped"] = int(manifest.get("n_start_still_dropped") or 0)
    summary["drop_start_still"] = bool(manifest.get("drop_start_still"))
    summary["n_start_knots_rewritten"] = int(manifest.get("n_start_knots_rewritten") or 0)
    summary["rewrite_start_knots"] = bool(manifest.get("rewrite_start_knots"))
    summary["start_knot_speed_mps"] = manifest.get("start_knot_speed_mps")
    summary["fit_used"] = fit
    _write_json(train_dir / "d6e-train.json", summary)
    return summary


def _hover(ticks: list[dict]) -> dict:
    by_id: dict[str, list[dict]] = {}
    for row in ticks:
        by_id.setdefault(str(row["uav"]), []).append(row)
    drones = []
    for drone_id, rows in by_id.items():
        speeds = [float(np.linalg.norm(item["v"])) for item in rows]
        errors = [float(item["goal_error_m"]) for item in rows]
        hover_ticks = sum(
            1
            for speed, error in zip(speeds, errors)
            if speed <= GOAL_SPEED_MPS and error > GOAL_TOLERANCE_M
        )
        final_error = errors[-1] if errors else None
        max_speed = max(speeds) if speeds else None
        drones.append({
            "id": drone_id,
            "n_ticks": len(rows),
            "hover_ticks": hover_ticks,
            "hover_frac": (hover_ticks / len(rows)) if rows else None,
            "max_speed_mps": max_speed,
            "final_goal_error_m": final_error,
            "never_left_hover": bool(
                max_speed is not None
                and max_speed <= GOAL_SPEED_MPS
                and final_error is not None
                and final_error > GOAL_TOLERANCE_M
            ),
        })
    fracs = [float(item["hover_frac"]) for item in drones if item["hover_frac"] is not None]
    return {
        "hover_frac": float(np.mean(fracs)) if fracs else None,
        "n_never_left_hover": sum(1 for item in drones if item["never_left_hover"]),
        "max_speed_mps": max((float(item["max_speed_mps"]) for item in drones if item["max_speed_mps"] is not None), default=None),
        "drones": drones,
    }


def eval_one(spec: dict) -> dict:
    """单场闭环。学生参数写在内存卡上。"""
    os.environ["OMP_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    import torch

    torch.set_num_threads(1)
    from yopo_l0.runner import RunConfig, run
    from yopo_l0.scene import load_card

    ev = _eval()
    card = ev.retarget_card(load_card(spec["card"]), spec["altitude_m"], "off", spec["weights"])
    started = time.perf_counter()
    output = run(
        card,
        RunConfig(seed=int(spec["seed"]), policy="student", shadow=True, duration_s=None),
    )
    result = output.result
    kind = ev._classify(result)
    distance = result.get("min_obstacle_distance_m")
    clearance = None if distance is None else float(distance) - ev.BODY_RADIUS_M
    hover = _hover(output.ticks)
    row = {
        "arm": spec["arm"],
        "scene": spec["scene"],
        "label": spec["label"],
        "altitude_m": float(spec["altitude_m"]),
        "seed": int(spec["seed"]),
        "outcome": result.get("outcome"),
        "reason": result.get("reason"),
        "kind": kind,
        "n_drones": int(result.get("n_drones") or 0),
        "arrived": int(result.get("arrived") or 0),
        "obstacle": int(kind == "obstacle"),
        "inter_drone": int(kind == "inter_drone"),
        "timeout": int(kind == "timeout"),
        "min_clearance_m": clearance,
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "final_goal_error_m": result.get("final_goal_error_m"),
        "sim_duration_s": result.get("sim_duration_s"),
        "hover_frac": hover["hover_frac"],
        "n_never_left_hover": hover["n_never_left_hover"],
        "max_speed_mps": hover["max_speed_mps"],
        "worker_s": time.perf_counter() - started,
        "report_only": True,
        "deviates_upstream": bool(spec.get("deviates_upstream")),
    }
    print(
        f"{row['arm']} {row['scene']} z={row['altitude_m']} seed={row['seed']} "
        f"{row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"hover={row['hover_frac']:.3f} vmax={row['max_speed_mps']}",
        flush=True,
    )
    return row


def _map(fn, items: list, jobs: int) -> list:
    if len(items) <= 1 or jobs <= 1:
        return [fn(item) for item in items]
    import multiprocessing
    from concurrent.futures import ProcessPoolExecutor, as_completed

    ctx = multiprocessing.get_context("spawn")
    with ProcessPoolExecutor(max_workers=max(1, min(int(jobs), len(items))), mp_context=ctx) as pool:
        futures = [pool.submit(fn, item) for item in items]
        return [future.result() for future in as_completed(futures)]


def _seeds_for(scene: str, override: list[int] | None) -> list[int]:
    if override is not None:
        return list(override)
    return list(PROTOCOL_SEEDS.get(scene, list(range(10))))


def evaluate(
    out_dir: Path,
    weights: dict[str, Path],
    *,
    scenes: list[str],
    seeds: list[int] | None,
    altitudes: list[float],
    jobs: int,
) -> list[dict]:
    ev = _eval()
    chosen = [item for item in ev.SCENES if item[0] in set(scenes)]
    missing = sorted(set(scenes) - {item[0] for item in chosen})
    if missing:
        known = ", ".join(item[0] for item in ev.SCENES)
        raise SystemExit(f"未知场景 {missing}。可选：{known}")
    specs = []
    for arm, path in weights.items():
        for altitude in altitudes:
            for name, card, label in chosen:
                for seed in _seeds_for(name, seeds):
                    specs.append({
                        "arm": arm,
                        "scene": name,
                        "label": label,
                        "card": str(card),
                        "altitude_m": float(altitude),
                        "seed": int(seed),
                        "weights": str(path),
                        "deviates_upstream": arm == "on",
                    })
    rows = _map(eval_one, specs, jobs)
    rows.sort(key=lambda item: (item["arm"], item["scene"], float(item["altitude_m"]), item["seed"]))
    _write_json(out_dir / "eval-rows.json", {"report_only": True, "rows": rows})
    return rows


def eval_ref_one(spec: dict) -> dict:
    """子进程入口。在进程内加载 D6c 诊断脚本，避免动态模块不能 pickle。"""
    return _diagnose().run_episode(spec)


def eval_straight_reference(weights: Path, *, altitudes: list[float], seeds: list[int], jobs: int) -> list[dict]:
    """用改写后的 0.2 s 结点同一条直线参考替换 predict，验证仍能到达。"""
    diag = _diagnose()
    specs = []
    for altitude in altitudes:
        for seed in seeds:
            specs.append({
                "name": "rewrite-cruise",
                "predict": "cruise",
                "params": dict(diag.R4),
                "altitude_m": float(altitude),
                "seed": int(seed),
                "weights": str(weights),
                "drones": 3,
            })
    rows = _map(eval_ref_one, specs, jobs)
    rows.sort(key=lambda item: (float(item["altitude_m"]), int(item["seed"])))
    return rows


def _median(values: list[float]) -> float | None:
    if not values:
        return None
    return float(np.median(np.asarray(values, dtype=np.float64)))


def aggregate(rows: list[dict]) -> list[dict]:
    groups: dict[tuple, list[dict]] = {}
    for row in rows:
        groups.setdefault((row["arm"], row["scene"], float(row["altitude_m"])), []).append(row)
    out = []
    for key in sorted(groups, key=lambda item: (item[0], item[1], item[2])):
        items = groups[key]
        drones = max(int(item["n_drones"]) for item in items)
        errors = [float(item["final_goal_error_m"]) for item in items if item.get("final_goal_error_m") is not None]
        hovers = [float(item["hover_frac"]) for item in items if item.get("hover_frac") is not None]
        speeds = [float(item["max_speed_mps"]) for item in items if item.get("max_speed_mps") is not None]
        out.append({
            "arm": key[0],
            "scene": key[1],
            "label": items[0]["label"],
            "altitude_m": key[2],
            "seeds": len(items),
            "arrived": sum(int(item["arrived"]) for item in items),
            "denom": drones * len(items),
            "obstacle": sum(int(item["obstacle"]) for item in items),
            "inter_drone": sum(int(item["inter_drone"]) for item in items),
            "timeout": sum(int(item["timeout"]) for item in items),
            "hover_frac_mean": float(np.mean(hovers)) if hovers else None,
            "n_never_left_hover": sum(int(item["n_never_left_hover"]) for item in items),
            "n_drones": drones * len(items),
            "max_speed_median_mps": _median(speeds),
            "final_goal_error_median_m": _median(errors),
            "deviates_upstream": any(bool(item.get("deviates_upstream")) for item in items),
        })
    return out


def _brief_train(summary: dict) -> dict:
    hold = summary.get("holdout") or {}
    return {
        "weights": summary.get("weights"),
        "weights_sha256": summary.get("weights_sha256"),
        "epochs_done": summary.get("epochs_done"),
        "wall_s": summary.get("wall_s"),
        "train_ids": summary.get("train_ids"),
        "hold_ids": summary.get("hold_ids"),
        "n_train_windows": summary.get("n_train_windows"),
        "n_hold_windows": summary.get("n_hold_windows"),
        "n_start_still_dropped": summary.get("n_start_still_dropped"),
        "drop_start_still": summary.get("drop_start_still"),
        "n_start_knots_rewritten": summary.get("n_start_knots_rewritten"),
        "rewrite_start_knots": summary.get("rewrite_start_knots"),
        "start_knot_speed_mps": summary.get("start_knot_speed_mps"),
        "fit_used": summary.get("fit_used"),
        "holdout": hold,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="D6e：样本规模上对照起步结点沿目标改写")
    parser.add_argument("mode", choices=("convert", "train", "eval", "all"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--seeds", type=int, nargs="+", default=None, help="缺省按协议：开阔 0–9，g2 0–4")
    parser.add_argument("--altitudes", type=float, nargs="+", default=[1.0, 1.5])
    parser.add_argument("--scenes", nargs="+", default=["open-3drone", "g2-pillar"])
    parser.add_argument("--ref-seeds", type=int, nargs="+", default=OPEN_REF_SEEDS)
    parser.add_argument("--skip-ref", action="store_true", help="跳过直线参考对照")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    arms = {"off": out / "off", "on": out / "on"}
    trained = {}
    if args.mode in ("convert", "train", "all"):
        for name, rewrite in (("off", False), ("on", True)):
            print(f"转换 {name} rewrite_start_knots={rewrite} drop_start_still=False", flush=True)
            manifest = convert_arm(arms[name], rewrite=rewrite, jobs=int(args.jobs))
            _write_json(arms[name] / "convert-note.json", {
                "n_converted": manifest["n_converted"],
                "n_windows": manifest["n_windows"],
                "n_start_still_dropped": manifest["n_start_still_dropped"],
                "drop_start_still": manifest["drop_start_still"],
                "n_start_knots_rewritten": manifest["n_start_knots_rewritten"],
                "rewrite_start_knots": manifest["rewrite_start_knots"],
                "start_knot_speed_mps": manifest["start_knot_speed_mps"],
                "train_ids_pipeline": manifest["train_ids"],
                "hold_ids_pipeline": manifest["hold_ids"],
            })
            if args.mode == "convert":
                continue
            print(f"训练 {name}", flush=True)
            trained[name] = train_arm(manifest, arms[name])
            print(json.dumps({
                "arm": name,
                "sha256": trained[name].get("weights_sha256"),
                "n_train_windows": trained[name].get("n_train_windows"),
                "n_start_knots_rewritten": trained[name].get("n_start_knots_rewritten"),
                "holdout": (trained[name].get("holdout") or {}).get("position_xy_rmse_m"),
                "wall_s": trained[name].get("wall_s"),
            }, ensure_ascii=False), flush=True)
    rows = []
    if args.mode in ("eval", "all"):
        weights = {}
        for name, directory in arms.items():
            path = directory / "train" / "student.pt"
            if not path.is_file():
                raise SystemExit(f"没有权重：{path}")
            weights[name] = path
        from yopo_l0.policy import StudentPolicy

        defaults_before = dict(StudentPolicy.DEFAULTS)
        rows = evaluate(
            out,
            weights,
            scenes=list(args.scenes),
            seeds=None if args.seeds is None else list(args.seeds),
            altitudes=list(args.altitudes),
            jobs=int(args.jobs),
        )
        if dict(StudentPolicy.DEFAULTS) != defaults_before:
            raise SystemExit("StudentPolicy.DEFAULTS 被改了")
    ref_rows = []
    if args.mode in ("eval", "all") and not args.skip_ref:
        off_weights = arms["off"] / "train" / "student.pt"
        print("直线参考：改写后的 0.2 s 结点（1.33 m/s，0.266 m）替换 predict", flush=True)
        from yopo_l0.policy import StudentPolicy

        defaults_before = dict(StudentPolicy.DEFAULTS)
        ref_rows = eval_straight_reference(
            off_weights,
            altitudes=list(args.altitudes),
            seeds=list(args.ref_seeds),
            jobs=int(args.jobs),
        )
        if dict(StudentPolicy.DEFAULTS) != defaults_before:
            raise SystemExit("StudentPolicy.DEFAULTS 被直线参考改了")
        _write_json(out / "ref-rows.json", {"report_only": True, "rows": ref_rows, "deviates_upstream": True})
    if trained or rows or ref_rows:
        note = {
            "disclaimer": DISCLAIMER,
            "scale": "data/teacher/sample 的成功场。留出是排序后下标 % 5 == 0。",
            "hover_rule": (
                "悬停 tick：速度 ≤ 0.2 m/s 且离目标 > 0.4 m。"
                "hover_frac 是各机这种 tick 的比例再对机平均，组内再对场平均。"
                "never_left_hover：整场最大速度 ≤ 0.2 m/s 且终点误差 > 0.4 m。"
            ),
            "rewrite_rule": (
                "开臂把起步静止段窗口的位置、速度结点改写成沿目标 1.33 m/s 的匀速短移位。"
                "0.2 s 结点位移 0.266 m。不删窗。关臂与改前逐位相同。"
            ),
            "arms": {name: _brief_train(summary) for name, summary in trained.items()},
            "groups": aggregate(rows) if rows else [],
            "straight_reference": ref_rows,
            "scenes": list(args.scenes),
            "seeds": None if args.seeds is None else list(args.seeds),
            "seed_protocol": {name: _seeds_for(name, None if args.seeds is None else list(args.seeds)) for name in args.scenes},
            "altitudes_m": list(args.altitudes),
        }
        _write_json(out / "d6e-summary.json", note)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
