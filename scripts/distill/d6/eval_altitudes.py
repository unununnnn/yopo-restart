#!/usr/bin/env python3
"""D6b：同一份学生权重，在 1.0 m 和 1.5 m 上各飞一遍，只报数。

场景覆盖开阔三机平行、开阔三机对向、g2 柱子、密度 0.14 单机、密度 0.14 三机对向。
闭环接口用 D5b r4（knots=position、yaw_source=ego_lookahead、accel_source=reference、
vertical=hold_goal_z），写在这次评测的内存场景卡上，不改 ``StudentPolicy.DEFAULTS``。

``--track-clearance off`` 不写 ``track_clearance``（库默认 false）。
``geom_multi`` 只在内存卡上打开 E-C5b 的那一项，方便和跟踪补偿对照。
学生策略不读 ``policy_params.upstream``，所以这一臂不会改飞法；表上若两臂相同，就是这个原因。

    .venv/bin/python scripts/distill/d6/eval_altitudes.py \\
        --weights /workspace/yopo-teacher-data/d6b/train/student.pt \\
        --out /workspace/yopo-teacher-data/d6b/eval \\
        --altitudes 1.0 1.5 --seeds 0 1 2 3 4 5 6 7 8 9 \\
        --track-clearance off
"""

from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SCENES = (
    ("open-3drone", ROOT / "scenes" / "open-3drone.json", "开阔三机平行"),
    ("swap-3-open", ROOT / "scenes" / "swap-3-open.json", "开阔三机对向"),
    ("g2-pillar", ROOT / "scenes" / "g2-pillar.json", "g2 柱子"),
    ("ego-density-014-solo", ROOT / "scenes" / "ego-density-014-solo.json", "密度 0.14 单机"),
    ("ego-density-014-swap3", ROOT / "scenes" / "ego-density-014-swap3.json", "密度 0.14 三机对向"),
)
BODY_RADIUS_M = 0.35
DISCLAIMER = (
    "report-only。本文件只报数。"
    "不修改 StudentPolicy 的默认（knots=network、yaw_source=calculate_yaw、"
    "accel_source=state、vertical=network），不改场景卡文件，"
    "不把 track_clearance 写成库默认（库默认仍是 false）。"
    "geom_multi 只写在这次评测的内存场景卡上。"
    "学生策略不读 policy_params.upstream.track_clearance，这一臂不改变飞法，用来对照默认没被改写。"
)


def _pipeline():
    name = "d6b_pipeline"
    cached = sys.modules.get(name)
    if cached is not None and hasattr(cached, "R4_EVAL_PARAMS"):
        return cached
    path = Path(__file__).resolve().parent / "pipeline.py"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"无法加载 {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def r4_eval_params() -> dict:
    return dict(_pipeline().R4_EVAL_PARAMS)


def library_defaults() -> dict:
    """读当前进程里的库默认。本脚本不赋值。"""
    from yopo_l0.policy import StudentPolicy, read_track_clearance

    defaults = StudentPolicy.DEFAULTS
    return {
        "student_knots": defaults["knots"],
        "student_yaw_source": defaults["yaw_source"],
        "student_accel_source": defaults["accel_source"],
        "student_vertical": defaults["vertical"],
        "track_clearance": read_track_clearance({}),
        "changed": False,
    }


def retarget_card(card, altitude_m: float, track_mode: str, weights: str):
    """复制一张卡：起终点高度改成目标值，学生接口改成 r4。不写磁盘。"""
    from yopo_l0.policy import TRACK_MARGIN_DEFAULT_M
    from yopo_l0.scene import SceneCard

    if track_mode not in ("off", "geom_multi"):
        raise ValueError(f"track_clearance 只能是 off 或 geom_multi，收到 {track_mode}")
    altitude = float(altitude_m)
    if not np.isfinite(altitude):
        raise ValueError(f"高度必须是有限数，收到 {altitude_m}")
    raw = copy.deepcopy(card.card)
    for drone in raw.get("drones") or []:
        for key in ("start_enu", "goal_enu", "hover_enu"):
            point = drone.get(key)
            if isinstance(point, list) and len(point) >= 3:
                drone[key] = [float(point[0]), float(point[1]), altitude]
    params = dict(raw.get("policy_params") or {})
    student = dict(params.get("student") or {})
    student.update(r4_eval_params())
    student["weights"] = str(weights)
    params["student"] = student
    if track_mode == "geom_multi":
        upstream = dict(params.get("upstream") or {})
        upstream["track_clearance"] = True
        upstream["track_clearance_mode"] = "geom_multi"
        upstream["track_margin_m"] = float(TRACK_MARGIN_DEFAULT_M)
        params["upstream"] = upstream
    raw["policy_params"] = params
    return SceneCard(card.path, raw, card.obstacles)


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


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def eval_one(spec: dict) -> dict:
    """单场闭环。学生参数在卡上，不改类默认。"""
    from yopo_l0.runner import RunConfig, run
    from yopo_l0.scene import load_card

    card = retarget_card(load_card(spec["card"]), spec["altitude_m"], spec["track"], spec["weights"])
    started = time.perf_counter()
    duration = spec.get("duration_s")
    output = run(
        card,
        RunConfig(
            seed=int(spec["seed"]),
            policy="student",
            shadow=bool(spec.get("shadow", True)),
            duration_s=None if duration is None else float(duration),
        ),
    )
    result = output.result
    kind = _classify(result)
    distance = result.get("min_obstacle_distance_m")
    clearance = None if distance is None else float(distance) - BODY_RADIUS_M
    student = card.card["policy_params"]["student"]
    upstream = (card.card.get("policy_params") or {}).get("upstream") or {}
    row = {
        "scene": spec["scene"],
        "label": spec["label"],
        "altitude_m": float(spec["altitude_m"]),
        "track_clearance": spec["track"],
        "track_clearance_on_card": upstream.get("track_clearance", False) is True,
        "track_clearance_mode_on_card": upstream.get("track_clearance_mode"),
        "student_reads_track_clearance": False,
        "seed": int(spec["seed"]),
        "outcome": result.get("outcome"),
        "reason": result.get("reason"),
        "kind": kind,
        "n_drones": int(result.get("n_drones") or 0),
        "arrived": int(result.get("arrived") or 0),
        "obstacle": int(kind == "obstacle"),
        "inter_drone": int(kind == "inter_drone"),
        "timeout": int(kind == "timeout"),
        "min_obstacle_distance_m": distance,
        "min_clearance_m": clearance,
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "final_goal_error_m": result.get("final_goal_error_m"),
        "sim_duration_s": result.get("sim_duration_s"),
        "wall_time_s": result.get("wall_time_s"),
        "worker_s": time.perf_counter() - started,
        "student_params": {key: student[key] for key in r4_eval_params()},
        "weights": spec["weights"],
        "report_only": True,
    }
    dest = Path(spec["dest"])
    _write_json(dest, row)
    print(
        f"{row['scene']} z={row['altitude_m']} track={row['track_clearance']} seed={row['seed']} "
        f"{row['outcome']} {row['arrived']}/{row['n_drones']} clear={row['min_clearance_m']}",
        flush=True,
    )
    return row


def _median(values: list[float]) -> float | None:
    if not values:
        return None
    return float(np.median(np.asarray(values, dtype=np.float64)))


def aggregate(rows: list[dict], scene_order: list[str], altitudes: list[float], tracks: list[str]) -> list[dict]:
    groups: dict[tuple, list[dict]] = defaultdict(list)
    for row in rows:
        groups[(row["scene"], float(row["altitude_m"]), row["track_clearance"])].append(row)
    out = []
    for track in tracks:
        for altitude in altitudes:
            for scene in scene_order:
                items = groups.get((scene, float(altitude), track)) or []
                if not items:
                    continue
                drones = max(int(item["n_drones"]) for item in items)
                clears = [float(item["min_clearance_m"]) for item in items if item.get("min_clearance_m") is not None]
                errors = [float(item["final_goal_error_m"]) for item in items if item.get("final_goal_error_m") is not None]
                out.append({
                    "scene": scene,
                    "label": items[0]["label"],
                    "altitude_m": float(altitude),
                    "track_clearance": track,
                    "seeds": len(items),
                    "arrived": sum(int(item["arrived"]) for item in items),
                    "denom": drones * len(items),
                    "obstacle": sum(int(item["obstacle"]) for item in items),
                    "inter_drone": sum(int(item["inter_drone"]) for item in items),
                    "timeout": sum(int(item["timeout"]) for item in items),
                    "min_clearance_m": min(clears) if clears else None,
                    "final_goal_error_median_m": _median(errors),
                })
    return out


def _fmt_opt(value, digits=3, signed=False) -> str:
    if value is None:
        return "—"
    if signed:
        return f"{float(value):+.{digits}f}"
    return f"{float(value):.{digits}f}"


def render_markdown(summary: dict) -> str:
    lines = [
        "# D6b 双高度闭环（report-only）",
        "",
        f"> {DISCLAIMER}",
        "",
        f"- 权重 `{summary['weights']}`",
        f"- sha256 `{summary.get('weights_sha256')}`",
        f"- 学生接口 `{json.dumps(summary['student_params'], ensure_ascii=False)}`",
        f"- 库默认 `{json.dumps(summary['library_defaults'], ensure_ascii=False)}`",
        "",
        "| 场景 | 高度 m | track_clearance | 到达 | 撞障场 | 机间场 | 超时场 | 最小净空 m | 终点误差中位数 m |",
        "| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for group in summary["groups"]:
        lines.append(
            "| {label} | {altitude:.1f} | {track} | {arrived}/{denom} | {obstacle} | {inter} | {timeout} | {clear} | {err} |".format(
                label=group["label"],
                altitude=float(group["altitude_m"]),
                track=group["track_clearance"],
                arrived=group["arrived"],
                denom=group["denom"],
                obstacle=group["obstacle"],
                inter=group["inter_drone"],
                timeout=group["timeout"],
                clear=_fmt_opt(group["min_clearance_m"], signed=True),
                err=_fmt_opt(group["final_goal_error_median_m"]),
            )
        )
    lines.extend(["", "## 高度对照", ""])
    altitudes = summary["altitudes_m"]
    header = "| 场景 | track_clearance | " + " | ".join(f"{float(item):.1f} m" for item in altitudes) + " |"
    rule = "| --- | --- | " + " | ".join("---" for _ in altitudes) + " |"
    lines.extend([header, rule])
    by_key = {(item["scene"], item["track_clearance"], float(item["altitude_m"])): item for item in summary["groups"]}
    seen = []
    for group in summary["groups"]:
        key = (group["scene"], group["track_clearance"])
        if key in seen:
            continue
        seen.append(key)
        cells = []
        for altitude in altitudes:
            item = by_key.get((group["scene"], group["track_clearance"], float(altitude)))
            if item is None:
                cells.append("—")
            else:
                cells.append(
                    f"{item['arrived']}/{item['denom']}，撞 {item['obstacle']}，超时 {item['timeout']}，净空 {_fmt_opt(item['min_clearance_m'], signed=True)}"
                )
        lines.append("| {label} | {track} | {cells} |".format(
            label=group["label"],
            track=group["track_clearance"],
            cells=" | ".join(cells),
        ))
    lines.append("")
    return "\n".join(lines)


def _recipe_mismatch(weights: Path) -> dict:
    """检查点里若记了别的接口，写进报告。评测仍用 r4。"""
    try:
        from yopo_l0.student import load_checkpoint

        _model, meta = load_checkpoint(weights, enabled=True)
    except (OSError, ValueError):
        return {}
    stored = ((meta.get("d6b") or {}).get("eval_params") or {})
    if not stored:
        return {}
    expected = r4_eval_params()
    return {key: stored.get(key) for key in expected if stored.get(key) != expected[key]}


def evaluate(weights: Path, out_dir: Path, *, altitudes, seeds, tracks, scenes, jobs: int, duration_s, shadow: bool) -> dict:
    import hashlib

    from yopo_l0.student import checkpoint_sha256

    weights = Path(weights)
    if not weights.is_file():
        raise SystemExit(f"没有检查点：{weights}")
    out_dir = Path(out_dir)
    row_dir = out_dir / "rows"
    row_dir.mkdir(parents=True, exist_ok=True)
    chosen = [item for item in SCENES if item[0] in set(scenes)]
    missing = sorted(set(scenes) - {item[0] for item in SCENES})
    if missing:
        known = ", ".join(item[0] for item in SCENES)
        raise SystemExit(f"未知场景 {missing}。可选：{known}")
    for _name, path, _label in chosen:
        if not path.is_file():
            raise SystemExit(f"场景卡不在：{path}")
    specs = []
    rows = []
    for altitude in altitudes:
        for track in tracks:
            for name, path, label in chosen:
                for seed in seeds:
                    tag = f"{name}-z{int(round(float(altitude) * 100)):03d}-{track}-s{int(seed)}"
                    dest = row_dir / f"{tag}.json"
                    spec = {
                        "scene": name,
                        "label": label,
                        "card": str(path),
                        "altitude_m": float(altitude),
                        "track": track,
                        "seed": int(seed),
                        "weights": str(weights),
                        "dest": str(dest),
                        "duration_s": duration_s,
                        "shadow": shadow,
                    }
                    if dest.is_file():
                        rows.append(json.loads(dest.read_text(encoding="utf-8")))
                        continue
                    specs.append(spec)
    if specs:
        if jobs <= 1 or len(specs) == 1:
            for spec in specs:
                rows.append(eval_one(spec))
        else:
            import multiprocessing
            from concurrent.futures import ProcessPoolExecutor, as_completed

            ctx = multiprocessing.get_context("spawn")
            with ProcessPoolExecutor(max_workers=max(1, int(jobs)), mp_context=ctx) as pool:
                futures = [pool.submit(eval_one, spec) for spec in specs]
                for future in as_completed(futures):
                    rows.append(future.result())
    rows.sort(key=lambda item: (item["track_clearance"], float(item["altitude_m"]), item["scene"], item["seed"]))
    scene_order = [item[0] for item in chosen]
    summary = {
        "report_only": True,
        "disclaimer": DISCLAIMER,
        "weights": str(weights),
        "weights_sha256": checkpoint_sha256(weights),
        "student_params": r4_eval_params(),
        "library_defaults": library_defaults(),
        "recipe_mismatch": _recipe_mismatch(weights),
        "altitudes_m": [float(item) for item in altitudes],
        "seeds": [int(item) for item in seeds],
        "track_clearance": list(tracks),
        "scenes": scene_order,
        "duration_s": duration_s,
        "shadow": bool(shadow),
        "body_radius_m": BODY_RADIUS_M,
        "scene_files_sha256": {
            name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name, path, _label in chosen
        },
        "rows": rows,
        "groups": aggregate(rows, scene_order, [float(item) for item in altitudes], list(tracks)),
    }
    _write_json(out_dir / "eval-altitudes.json", summary)
    (out_dir / "eval-altitudes.md").write_text(render_markdown(summary), encoding="utf-8")
    for group in summary["groups"]:
        print(json.dumps(group, ensure_ascii=False), flush=True)
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="D6b：1.0 m 与 1.5 m 学生闭环，只报数")
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--altitudes", type=float, nargs="+", default=[1.0, 1.5])
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(10)))
    parser.add_argument("--track-clearance", nargs="+", choices=("off", "geom_multi"), default=["off"])
    parser.add_argument("--scenes", nargs="+", default=[item[0] for item in SCENES])
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--duration-s", type=float, default=None, help="盖过场景卡时长。缺省用卡上的 duration_s")
    parser.add_argument("--no-shadow", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if int(args.jobs) < 1:
        raise SystemExit("--jobs 至少为 1")
    for altitude in args.altitudes:
        if not np.isfinite(float(altitude)):
            raise SystemExit(f"高度必须是有限数，收到 {altitude}")
    if args.duration_s is not None and (not np.isfinite(float(args.duration_s)) or float(args.duration_s) <= 0.0):
        raise SystemExit("--duration-s 必须为正")
    evaluate(
        args.weights,
        args.out,
        altitudes=args.altitudes,
        seeds=args.seeds,
        tracks=args.track_clearance,
        scenes=args.scenes,
        jobs=int(args.jobs),
        duration_s=None if args.duration_s is None else float(args.duration_s),
        shadow=not args.no_shadow,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
