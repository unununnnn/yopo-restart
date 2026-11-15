#!/usr/bin/env python3
"""D6b：把 1.0 m（d2b）和 1.5 m（d6-z150）的教师记录转成窗口，再按 D5b r4 混合训练。

扫描多个教师根目录里已经写完的 ``yopo-teacher-record/1``。成功场转成每场一个 ``.npz``，
可中断后续跑。训练走 ``yopo_l0.student_fit.train_from_paths``，不改它的默认。
``knots`` / ``yaw_source`` / ``accel_source`` / ``vertical`` 是闭环接口，写进检查点，
本脚本不改 ``StudentPolicy.DEFAULTS``。``--drop-start-still`` 与 ``--rewrite-start-knots`` 默认关；关时窗口与不传逐位相同。

    .venv/bin/python scripts/distill/d6/pipeline.py all \\
        --teacher-dirs /workspace/yopo-teacher-data/d2b/runs \\
                       /workspace/yopo-teacher-data/d6-z150/runs \\
        --out /workspace/yopo-teacher-data/d6b --jobs 2

    .venv/bin/python scripts/distill/d6/pipeline.py --only-convert \\
        --teacher-dirs /workspace/yopo-teacher-data/d2b/runs \\
                       /workspace/yopo-teacher-data/d6-z150/runs \\
        --out /workspace/yopo-teacher-data/d6b

    .venv/bin/python scripts/distill/d6/pipeline.py --only-train \\
        --out /workspace/yopo-teacher-data/d6b --balance altitude
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

from yopo_l0.distill import (  # noqa: E402
    START_KNOT_SPEED_MPS,
    START_STILL_SPEED_MPS,
    TEACHER_SCHEMA,
    DistillError,
    parse_run_id,
)

# 与 scripts/distill/d5b/train_r4.py、scripts/distill/d6/feasibility.py 的 FitConfig 相同。
# 闭环四项不进 FitConfig，评测脚本按同一组打开，库默认仍是 D5。
R4_EVAL_PARAMS = {
    "knots": "position",
    "yaw_source": "ego_lookahead",
    "accel_source": "reference",
    "vertical": "hold_goal_z",
}
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
MANIFEST_NAME = "convert-manifest.json"
STATUS_NAME = "convert-status.jsonl"
SCHEMA = "yopo-student-d6b-convert/1"
ASSUMPTION = "地图边长和障碍半径是假设，不是论文复现。"
START_STILL_RULE = (
    "关（默认）：不按起步速度删窗，窗口与不传这个开关逐位相同。"
    "开：丢掉速度模长 < start_still_speed_mps、且早于该架起步结点的窗。"
    "起步结点是该架速度第一次达到阈值的轨迹样本。阈值默认 0.05 m/s。"
    "结点之后再慢下来的窗保留。教师样本上这段零位移大约 1.2–1.6 s，"
    "盖住 t=0 前 1 s 位置结点位移为 0 的那些窗。"
)
START_KNOT_RULE = (
    "关（默认）：不改标签，窗口与不传这个开关逐位相同。"
    "开：把起步静止段窗口的位置、速度结点改写成沿目标方向的匀速短移位，不删窗。"
    "起步段判据与起步静止过滤相同，速度阈值固定 0.05 m/s。"
    "默认速度 1.33 m/s，与 D6c 直线参考相同，0.2 s 结点沿目标位移是 0.266 m。"
    "已经在动的窗口不改。不要把 --drop-start-still 默认打开。"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def z_key(value: float | None) -> str:
    """高度分组键。1.0 与 1.5 写成一位小数。"""
    if value is None:
        return "unknown"
    number = float(value)
    if not np.isfinite(number):
        return "unknown"
    return f"{number:.1f}"


def parse_min_clearance(text: str) -> float | None:
    """``0`` / ``off``：不再按净空过滤。正数：``min_obstacle_clearance_m`` 必须大于等于该值。

    教师自己的成功线已经是净空 ≥ 0。设 0 不会把这些成功场再丢掉。
    """
    raw = str(text).strip().lower()
    if raw in {"off", "none", "false"}:
        return None
    try:
        value = float(raw)
    except ValueError as exc:
        raise SystemExit(f"--min-clearance-m 无法解析：{text}") from exc
    if not np.isfinite(value) or value < 0.0:
        raise SystemExit(f"--min-clearance-m 必须是 off 或非负有限数，收到 {text}")
    if value == 0.0:
        return None
    return value


def resolve_flight_z(record: dict, run_id: str, parsed: dict | None = None) -> tuple[float, str, bool]:
    """高度优先用记录里的 ``scenario.flight_z_m``，否则从编号的 ``z150`` 读，都没有则 1.0 m。

    返回 ``(米, 来源, 记录和编号是否不一致)``。来源是 ``record`` / ``run_id`` / ``default_1.0``。
    """
    scenario = record.get("scenario") if isinstance(record, dict) else None
    if not isinstance(scenario, dict):
        scenario = {}
    recorded = _finite_or_none(scenario.get("flight_z_m"))
    if parsed is None:
        try:
            parsed = parse_run_id(run_id)
        except DistillError:
            parsed = None
    from_id = None
    if isinstance(parsed, dict) and "flight_z_m" in parsed:
        from_id = float(parsed["flight_z_m"])
    mismatch = recorded is not None and from_id is not None and abs(recorded - from_id) > 1e-6
    if recorded is not None:
        return recorded, "record", mismatch
    if from_id is not None:
        return from_id, "run_id", False
    return 1.0, "default_1.0", False


def _finite_or_none(value) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(number):
        return None
    return number


def discover_run_dirs(root: Path) -> list[Path]:
    """``root`` 自己或其中某一层有 ``record.json`` 即为一场。找到一场就不再往下走。"""
    root = Path(root)
    if not root.is_dir():
        raise SystemExit(f"教师目录不存在：{root}")
    if (root / "record.json").is_file():
        return [root]
    found: list[Path] = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames.sort()
        if "record.json" in filenames:
            found.append(Path(dirpath))
            dirnames.clear()
    return found


def _clearance_value(metrics: dict) -> tuple[float | None, str | None]:
    if "min_obstacle_clearance_m" not in metrics or metrics.get("min_obstacle_clearance_m") is None:
        return None, "missing"
    number = _finite_or_none(metrics.get("min_obstacle_clearance_m"))
    if number is None:
        return None, "bad"
    return number, None


def inspect_run(run_dir: Path, source_root: Path, min_clearance_m: float | None) -> dict:
    """读一场记录并归类。不渲染深度。"""
    run_dir = Path(run_dir)
    run_id = run_dir.name
    row = {
        "id": run_id,
        "run_dir": str(run_dir),
        "source_dir": str(Path(source_root)),
        "flight_z_m": None,
        "flight_z_source": None,
        "z_mismatch": False,
        "scene_key": run_id,
        "status": "bad_json",
        "clearance_m": None,
        "success": None,
        "detail": "",
    }
    record_path = run_dir / "record.json"
    try:
        record = json.loads(record_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        row["detail"] = f"record.json 读不出：{exc}"
        return row
    if not isinstance(record, dict):
        row["detail"] = "record.json 不是对象"
        return row
    parsed = None
    try:
        parsed = parse_run_id(run_id)
        row["scene_key"] = parsed["scene_key"]
    except DistillError:
        parsed = None
    height, source, mismatch = resolve_flight_z(record, run_id, parsed)
    row["flight_z_m"] = height
    row["flight_z_source"] = source
    row["z_mismatch"] = mismatch
    if record.get("schema") != TEACHER_SCHEMA:
        row["status"] = "bad_schema"
        row["detail"] = f"schema={record.get('schema')!r}"
        return row
    if parsed is None:
        row["status"] = "bad_id"
        row["detail"] = "场次编号不符合 dNNN-nN-(headon|parallel)[-zHHH]-sN"
        return row
    metrics = record.get("metrics")
    if not isinstance(metrics, dict) or "success" not in metrics:
        row["status"] = "incomplete"
        row["detail"] = "缺少 metrics.success，这场还没写完"
        return row
    if metrics.get("success") is not True:
        row["status"] = "not_success"
        row["success"] = False
        row["detail"] = "metrics.success 不是 true"
        return row
    row["success"] = True
    clearance, clearance_problem = _clearance_value(metrics)
    row["clearance_m"] = clearance
    if min_clearance_m is not None:
        if clearance_problem is not None or clearance is None or clearance < float(min_clearance_m):
            row["status"] = "filtered"
            row["detail"] = "净空低于门槛或写得不对"
            return row
    row["status"] = "eligible"
    return row


def scan_roots(roots: list[Path], min_clearance_m: float | None) -> list[dict]:
    """按 ``--teacher-dirs`` 的顺序扫描。同一编号只保留先扫到的那场。"""
    seen: dict[str, dict] = {}
    rows: list[dict] = []
    for root in roots:
        for run_dir in discover_run_dirs(root):
            row = inspect_run(run_dir, root, min_clearance_m)
            previous = seen.get(row["id"])
            if previous is not None:
                row = {
                    **row,
                    "status": "duplicate",
                    "detail": f"与 {previous['run_dir']} 编号重复，保留先扫到的",
                }
            else:
                seen[row["id"]] = row
            rows.append(row)
    return rows


def _empty_bucket() -> dict:
    return {
        "n_records": 0,
        "n_success": 0,
        "n_filtered": 0,
        "n_converted": 0,
        "n_windows": 0,
    }


def _bump(bucket: dict, row: dict, windows: int | None) -> None:
    bucket["n_records"] += 1
    if row["status"] != "duplicate" and row.get("success") is True:
        bucket["n_success"] += 1
    if row["status"] == "filtered":
        bucket["n_filtered"] += 1
    if windows is not None:
        bucket["n_converted"] += 1
        bucket["n_windows"] += int(windows)


def _convert_one(spec: dict) -> dict:
    """子进程入口。只捕转换错误，别把一场坏数据变成整批中断。"""
    root = Path(__file__).resolve().parents[3]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from yopo_l0.distill import DistillError, convert_run, save_batch

    started = time.perf_counter()
    run_id = spec["id"]
    still = _still_fields(spec)
    try:
        kwargs = {}
        if spec.get("max_windows") is not None:
            kwargs["max_windows"] = int(spec["max_windows"])
        if still["drop_start_still"]:
            kwargs["drop_start_still"] = True
            kwargs["start_still_speed_mps"] = float(still["start_still_speed_mps"])
        if still.get("rewrite_start_knots"):
            kwargs["rewrite_start_knots"] = True
            kwargs["start_knot_speed_mps"] = float(still["start_knot_speed_mps"])
        batch = convert_run(Path(spec["run_dir"]), **kwargs)
    except DistillError as exc:
        return {
            "id": run_id,
            "ok": False,
            "error": str(exc),
            "wall_s": time.perf_counter() - started,
            "max_windows": spec.get("max_windows"),
            **still,
        }
    if batch.n == 0:
        return {
            "id": run_id,
            "ok": False,
            "error": batch.skipped or "no_windows",
            "n": 0,
            "n_start_still_dropped": int(batch.n_start_still_dropped),
            "n_start_knots_rewritten": int(batch.n_start_knots_rewritten),
            "wall_s": time.perf_counter() - started,
            "max_windows": spec.get("max_windows"),
            **still,
        }
    path = save_batch(batch, Path(spec["out_path"]))
    return {
        "id": run_id,
        "ok": True,
        "n": int(batch.n),
        "n_start_still_dropped": int(batch.n_start_still_dropped),
        "n_start_knots_rewritten": int(batch.n_start_knots_rewritten),
        "path": str(Path(path).resolve()),
        "sha256": sha256_file(path),
        "bytes": Path(path).stat().st_size,
        "wall_s": time.perf_counter() - started,
        "max_windows": spec.get("max_windows"),
        **still,
    }


def _still_fields(spec: dict) -> dict:
    """状态行里的开关。关时不写速度，续跑才能和旧状态行对上。"""
    enabled = bool(spec.get("drop_start_still", False))
    if not enabled:
        out = {"drop_start_still": False}
    else:
        out = {
            "drop_start_still": True,
            "start_still_speed_mps": float(spec["start_still_speed_mps"]),
        }
    if bool(spec.get("rewrite_start_knots", False)):
        out["rewrite_start_knots"] = True
        out["start_knot_speed_mps"] = float(spec["start_knot_speed_mps"])
    return out


def _still_token(record: dict) -> tuple:
    """旧状态行没有这个键时当作关。"""
    if not bool(record.get("drop_start_still", False)):
        drop = (False, None)
    else:
        speed = record.get("start_still_speed_mps")
        drop = (True, None if speed is None else float(speed))
    if not bool(record.get("rewrite_start_knots", False)):
        rewrite = (False, None)
    else:
        speed = record.get("start_knot_speed_mps")
        rewrite = (True, None if speed is None else float(speed))
    return (drop, rewrite)


def _reusable(
    record: dict,
    windows: Path,
    max_windows: int | None,
    drop_start_still: bool,
    start_still_speed_mps: float | None,
    rewrite_start_knots: bool = False,
    start_knot_speed_mps: float | None = None,
) -> bool:
    if not record.get("ok") or not record.get("id"):
        return False
    if record.get("max_windows") != max_windows:
        return False
    drop = (False, None)
    if drop_start_still:
        drop = (True, float(start_still_speed_mps))
    rewrite = (False, None)
    if rewrite_start_knots:
        rewrite = (True, float(start_knot_speed_mps))
    if _still_token(record) != (drop, rewrite):
        return False
    target = windows / f"{record['id']}.npz"
    if not target.is_file():
        return False
    return record.get("sha256") == sha256_file(target) and record.get("path")


def _load_status(path: Path) -> dict[str, dict]:
    done: dict[str, dict] = {}
    if not path.is_file():
        return done
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        if record.get("id"):
            done[str(record["id"])] = record
    return done


def _map_jobs(fn, items: list, jobs: int) -> list:
    if len(items) <= 1 or jobs <= 1:
        return [fn(item) for item in items]
    import multiprocessing
    from concurrent.futures import ProcessPoolExecutor, as_completed

    ctx = multiprocessing.get_context("fork")
    workers = max(1, min(int(jobs), len(items)))
    with ProcessPoolExecutor(max_workers=workers, mp_context=ctx) as pool:
        futures = [pool.submit(fn, item) for item in items]
        return [future.result() for future in as_completed(futures)]


def convert_records(
    rows: list[dict],
    out_dir: Path,
    *,
    jobs: int,
    max_windows: int | None,
    limit: int | None,
    min_clearance_m: float | None,
    teacher_dirs: list[Path],
    drop_start_still: bool = False,
    start_still_speed_mps: float = START_STILL_SPEED_MPS,
    rewrite_start_knots: bool = False,
    start_knot_speed_mps: float = START_KNOT_SPEED_MPS,
) -> dict:
    """把合格场写成窗口，并写出 manifest。已有且 sha 对得上的场次跳过。

    ``drop_start_still`` 与 ``rewrite_start_knots`` 默认关。关时不把对应参数传进
    ``convert_run``，窗口与改前逐位相同。开时续跑还要比对阈值或改写速度，
    避免把没改过的 npz 当成已经改过。
    """
    from yopo_l0.distill import split_runs_by_seed

    out_dir = Path(out_dir)
    windows = out_dir / "windows"
    windows.mkdir(parents=True, exist_ok=True)
    status_path = out_dir / STATUS_NAME
    done = _load_status(status_path)
    eligible = [row for row in rows if row["status"] == "eligible"]
    if limit is not None:
        eligible = sorted(eligible, key=lambda row: row["id"])[: int(limit)]
        allowed = {row["id"] for row in eligible}
    else:
        allowed = None

    reused: dict[str, dict] = {}
    todo = []
    for row in eligible:
        previous = done.get(row["id"])
        if previous is not None and _reusable(
            previous,
            windows,
            max_windows,
            bool(drop_start_still),
            start_still_speed_mps,
            bool(rewrite_start_knots),
            start_knot_speed_mps,
        ):
            reused[row["id"]] = previous
            continue
        todo.append({
            "id": row["id"],
            "run_dir": row["run_dir"],
            "out_path": str(windows / f"{row['id']}.npz"),
            "max_windows": max_windows,
            "drop_start_still": bool(drop_start_still),
            "start_still_speed_mps": float(start_still_speed_mps),
            "rewrite_start_knots": bool(rewrite_start_knots),
            "start_knot_speed_mps": float(start_knot_speed_mps),
        })
    print(
        f"扫描 {len(rows)} 场，合格 {len(eligible)}，续跑 {len(reused)}，待转 {len(todo)}，jobs={max(1, int(jobs))}",
        flush=True,
    )
    fresh: dict[str, dict] = {}
    if todo:
        with status_path.open("a", encoding="utf-8") as handle:
            for record in _map_jobs(_convert_one, todo, int(jobs)):
                handle.write(json.dumps(record, ensure_ascii=False) + "\n")
                handle.flush()
                fresh[record["id"]] = record
                if record.get("ok"):
                    print(f"ok {record['id']} n={record['n']} {record['wall_s']:.2f}s", flush=True)
                else:
                    print(f"fail {record['id']} {record.get('error')}", flush=True)

    converted: dict[str, dict] = {}
    failures = []
    for row in eligible:
        record = fresh.get(row["id"]) or reused.get(row["id"])
        if record is None or not record.get("ok"):
            error = None if record is None else record.get("error")
            failures.append({"id": row["id"], "error": error or "未转换"})
            continue
        converted[row["id"]] = {
            "path": str(Path(record["path"]).resolve()),
            "sha256": record["sha256"],
            "n": int(record["n"]),
            "n_start_still_dropped": int(record.get("n_start_still_dropped") or 0),
            "n_start_knots_rewritten": int(record.get("n_start_knots_rewritten") or 0),
            "flight_z_m": row["flight_z_m"],
            "flight_z_source": row["flight_z_source"],
            "z_mismatch": bool(row["z_mismatch"]),
            "scene_key": row["scene_key"],
            "source_dir": row["source_dir"],
            "clearance_m": row["clearance_m"],
            "reused": row["id"] in reused and row["id"] not in fresh,
        }

    files = {}
    for run_id in sorted(converted):
        files[run_id] = {key: value for key, value in converted[run_id].items() if key != "reused"}
    train_ids, hold_ids = split_runs_by_seed(list(files))
    train_ids = sorted(train_ids)
    hold_ids = sorted(hold_ids)

    total = _empty_bucket()
    by_altitude: dict[str, dict] = {}
    by_scene: dict[str, dict] = {}
    counts = {
        "n_bad_json": 0,
        "n_bad_schema": 0,
        "n_bad_id": 0,
        "n_incomplete": 0,
        "n_not_success": 0,
        "n_duplicate": 0,
        "n_eligible": 0,
        "n_limited": 0,
        "n_z_mismatch": 0,
    }
    for row in rows:
        windows_n = files[row["id"]]["n"] if row["id"] in files else None
        limited = allowed is not None and row["status"] == "eligible" and row["id"] not in allowed
        _bump(total, row, windows_n)
        height = z_key(row.get("flight_z_m"))
        by_altitude.setdefault(height, _empty_bucket())
        _bump(by_altitude[height], row, windows_n)
        scene_bucket = by_scene.setdefault(row["scene_key"], {})
        scene_bucket.setdefault(height, _empty_bucket())
        _bump(scene_bucket[height], row, windows_n)
        status = row["status"]
        if status in {"bad_json", "bad_schema", "bad_id", "incomplete", "not_success", "duplicate", "eligible"}:
            counts[f"n_{status}"] += 1
        if limited:
            counts["n_limited"] += 1
        if row.get("z_mismatch"):
            counts["n_z_mismatch"] += 1

    manifest = {
        "schema": SCHEMA,
        "assumption": ASSUMPTION,
        "teacher_schema": TEACHER_SCHEMA,
        "teacher_dirs": [str(Path(path)) for path in teacher_dirs],
        "min_clearance_m": 0.0 if min_clearance_m is None else float(min_clearance_m),
        "clearance_filter": min_clearance_m is not None,
        "clearance_note": (
            "clearance_filter 为 false 时（--min-clearance-m 0 或 off）只要求 success==true。"
            "为 true 时再要求 metrics.min_obstacle_clearance_m >= min_clearance_m。"
        ),
        "max_windows": max_windows,
        "drop_start_still": bool(drop_start_still),
        "start_still_speed_mps": float(start_still_speed_mps) if drop_start_still else None,
        "start_still_rule": START_STILL_RULE,
        "n_start_still_dropped": sum(int(item.get("n_start_still_dropped") or 0) for item in files.values()),
        "rewrite_start_knots": bool(rewrite_start_knots),
        "start_knot_speed_mps": float(start_knot_speed_mps) if rewrite_start_knots else None,
        "start_knot_rule": START_KNOT_RULE,
        "n_start_knots_rewritten": sum(int(item.get("n_start_knots_rewritten") or 0) for item in files.values()),
        "sharding": "每场一个 npz，文件名是场次编号。sha 没变就跳过，可以断点续跑。",
        "hold_seed_mod": 5,
        "hold_rule": "seed % 5 == 0 进留出，其余进训练。1.0 m 和 1.5 m 同一颗种子一起进同一边。",
        "n_records": total["n_records"],
        "n_success": total["n_success"],
        "n_filtered": total["n_filtered"],
        "n_converted": total["n_converted"],
        "n_windows": total["n_windows"],
        "n_reused": sum(1 for item in converted.values() if item["reused"]),
        "n_convert_failed": len(failures),
        "n_train_runs": len(train_ids),
        "n_hold_runs": len(hold_ids),
        **counts,
        "by_altitude": by_altitude,
        "by_scene": by_scene,
        "train_ids": train_ids,
        "hold_ids": hold_ids,
        "failures": failures,
        "files": files,
    }
    _write_json(out_dir / MANIFEST_NAME, manifest)
    print(
        f"完成 转换 {manifest['n_converted']} 场，窗口 {manifest['n_windows']}，"
        f"过滤 {manifest['n_filtered']}，失败 {manifest['n_convert_failed']}，"
        f"训练 {len(train_ids)}，留出 {len(hold_ids)}",
        flush=True,
    )
    return manifest


def balance_train_ids(train_ids: list[str], files: dict, seed: int) -> tuple[list[str], dict]:
    """每个高度留下同样多场训练。场次少的一边全留，多的一边按种子抽。"""
    groups: dict[str, list[str]] = {}
    for run_id in train_ids:
        groups.setdefault(z_key(files[run_id]["flight_z_m"]), []).append(run_id)
    for height in groups:
        groups[height] = sorted(groups[height])
    populated = {height: ids for height, ids in groups.items() if ids}
    detail = {
        height: {"available": len(ids), "kept": len(ids)}
        for height, ids in groups.items()
    }
    if len(populated) < 2:
        return list(train_ids), {
            "mode": "altitude",
            "applied": False,
            "reason": "single_altitude",
            "by_altitude": detail,
        }
    target = min(len(ids) for ids in populated.values())
    if target <= 0:
        return list(train_ids), {
            "mode": "altitude",
            "applied": False,
            "reason": "empty_altitude",
            "by_altitude": detail,
        }
    rng = np.random.default_rng(int(seed))
    kept: list[str] = []
    for height in sorted(populated):
        ids = populated[height]
        if len(ids) > target:
            picked = sorted(int(index) for index in rng.choice(len(ids), size=target, replace=False))
            chosen = [ids[index] for index in picked]
        else:
            chosen = list(ids)
        kept.extend(chosen)
        detail[height] = {"available": len(ids), "kept": len(chosen)}
    return sorted(kept), {
        "mode": "altitude",
        "applied": True,
        "runs_per_altitude": target,
        "by_altitude": detail,
    }


def altitude_path_weights(ids: list[str], files: dict, paths: list[Path]) -> dict[str, float]:
    """抽完场次之后，若各高度的窗口数仍不同，把路径权重设成各高度总权重相等。"""
    mass: dict[str, int] = {}
    for run_id in ids:
        height = z_key(files[run_id]["flight_z_m"])
        mass[height] = mass.get(height, 0) + int(files[run_id]["n"])
    if len(mass) < 2 or any(value <= 0 for value in mass.values()):
        return {}
    weights = {}
    for run_id, path in zip(ids, paths):
        height = z_key(files[run_id]["flight_z_m"])
        weights[str(path)] = 1.0 / float(mass[height])
    return weights


def _distribution(ids: list[str], files: dict) -> dict:
    out: dict[str, dict] = {}
    for run_id in ids:
        height = z_key(files[run_id]["flight_z_m"])
        slot = out.setdefault(height, {"runs": 0, "windows": 0})
        slot["runs"] += 1
        slot["windows"] += int(files[run_id]["n"])
    return out


def manifest_summary(manifest: dict) -> dict:
    """检查点里放摘要，不把逐文件 sha 再抄一遍。"""
    keys = (
        "schema",
        "n_records",
        "n_success",
        "n_filtered",
        "n_converted",
        "n_windows",
        "n_train_runs",
        "n_hold_runs",
        "n_convert_failed",
        "by_altitude",
        "by_scene",
        "min_clearance_m",
        "clearance_filter",
        "teacher_dirs",
        "hold_rule",
        "drop_start_still",
        "start_still_speed_mps",
        "n_start_still_dropped",
        "rewrite_start_knots",
        "start_knot_speed_mps",
        "n_start_knots_rewritten",
    )
    return {key: manifest.get(key) for key in keys}


def _jsonable(value):
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, (np.floating, float)):
        return float(value)
    if isinstance(value, (np.integer, int)) and not isinstance(value, bool):
        return int(value)
    return value


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


class _TupleJson:
    """``train_from_paths`` 会把 FitConfig.loss_weights 这个元组写进 json。

    不改 ``student_fit.py``。只在这次调用期间让 ``json.dumps`` 接受元组，调用结束恢复。
    """

    def __enter__(self):
        import json as json_mod

        self._json = json_mod
        self._dumps = json_mod.dumps

        def dumps(obj, *args, **kwargs):
            if "default" not in kwargs:
                def default(item):
                    if isinstance(item, tuple):
                        return list(item)
                    raise TypeError(f"{type(item).__name__} 不能写成 JSON")

                kwargs["default"] = default
            return self._dumps(obj, *args, **kwargs)

        json_mod.dumps = dumps
        return self

    def __exit__(self, exc_type, exc, tb):
        self._json.dumps = self._dumps
        return False


def _stamp_checkpoint(path: Path, block: dict) -> dict:
    from yopo_l0.student import load_checkpoint, save_checkpoint

    model, meta = load_checkpoint(path, enabled=True)
    meta.pop("weights_sha256", None)
    meta.pop("weights_path", None)
    meta["d6b"] = _jsonable(block)
    save_checkpoint(model, path, meta)
    _model, stamped = load_checkpoint(path, enabled=True)
    return stamped


def train_from_manifest(manifest: dict, out_dir: Path, *, balance: str, epochs: int, seed: int, threads: int) -> dict:
    """按 manifest 的训练/留出名单训练。``balance=altitude`` 先把两边的场数抽齐。"""
    from yopo_l0.student_fit import FitConfig, train_from_paths

    files = manifest.get("files") or {}
    if not files:
        raise SystemExit("manifest 里没有窗口，先转换再训练")
    train_ids = [run_id for run_id in manifest.get("train_ids") or [] if run_id in files]
    hold_ids = [run_id for run_id in manifest.get("hold_ids") or [] if run_id in files]
    if balance == "altitude":
        train_ids, balance_info = balance_train_ids(train_ids, files, seed)
    elif balance == "full":
        balance_info = {"mode": "full", "applied": False, "train_runs": len(train_ids)}
    else:
        raise SystemExit(f"未知的 --balance：{balance}")
    if not train_ids:
        raise SystemExit("训练场次是空的")
    train_paths = [Path(files[run_id]["path"]).resolve() for run_id in train_ids]
    hold_paths = [Path(files[run_id]["path"]).resolve() for run_id in hold_ids]
    path_weights = {}
    if balance == "altitude" and balance_info.get("applied"):
        path_weights = altitude_path_weights(train_ids, files, train_paths)
    fit_kwargs = dict(R4_FIT)
    fit_kwargs["epochs"] = int(epochs)
    fit_kwargs["seed"] = int(seed)
    fit_kwargs["num_threads"] = int(threads)
    cfg = FitConfig(**fit_kwargs)
    train_dir = Path(out_dir) / "train"
    train_dir.mkdir(parents=True, exist_ok=True)
    with _TupleJson():
        summary = train_from_paths(
            train_paths,
            hold_paths,
            train_dir,
            cfg=cfg,
            path_weights=path_weights or None,
        )
    weights = Path(summary["weights"]) if summary.get("weights") else train_dir / "student.pt"
    if not weights.is_file():
        raise SystemExit(f"训练没有写出检查点：{weights}")
    block = {
        "ticket": "D6b",
        "recipe_name": "d5b-r4",
        "eval_params": dict(R4_EVAL_PARAMS),
        "recipe_fit": dict(R4_FIT),
        "fit_used": fit_kwargs,
        "balance": balance_info,
        "altitude_distribution": {
            "train": _distribution(train_ids, files),
            "hold": _distribution(hold_ids, files),
        },
        "manifest_summary": manifest_summary(manifest),
        "assumption": ASSUMPTION,
    }
    stamped = _stamp_checkpoint(weights, block)
    payload = {
        "weights": str(weights),
        "weights_sha256": stamped.get("weights_sha256"),
        "d6b": block,
        "holdout": summary.get("holdout"),
        "epochs_done": summary.get("epochs_done"),
        "wall_s": summary.get("wall_s"),
    }
    _write_json(train_dir / "d6b-train.json", payload)
    print(
        json.dumps({
            "weights_sha256": payload["weights_sha256"],
            "balance": balance_info.get("mode"),
            "train_altitudes": payload["d6b"]["altitude_distribution"]["train"],
            "epochs_done": payload["epochs_done"],
        }, ensure_ascii=False),
        flush=True,
    )
    return payload


def load_manifest(out_dir: Path) -> dict:
    path = Path(out_dir) / MANIFEST_NAME
    if not path.is_file():
        raise SystemExit(f"没有清单：{path}。先跑转换。")
    return json.loads(path.read_text(encoding="utf-8"))


def resolve_mode(mode: str | None, only_convert: bool, only_train: bool) -> str:
    chosen = []
    if only_convert:
        chosen.append("convert")
    if only_train:
        chosen.append("train")
    if len(chosen) > 1:
        raise SystemExit("--only-convert 和 --only-train 只能选一个")
    if chosen and mode not in (None, "all", chosen[0]):
        raise SystemExit(f"位置参数 {mode} 和 --only-* 冲突")
    if chosen:
        return chosen[0]
    if mode in ("convert", "train", "all"):
        return mode
    raise SystemExit("需要 all、--only-convert 或 --only-train")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="D6b：1.0 m + 1.5 m 教师窗口与 D5b r4 混合训练")
    parser.add_argument("mode", nargs="?", choices=("all", "convert", "train"), default=None)
    parser.add_argument("--only-convert", action="store_true", help="只扫描并转换，不训练")
    parser.add_argument("--only-train", action="store_true", help="只按已有 manifest 训练")
    parser.add_argument("--teacher-dirs", type=Path, nargs="+", default=None, help="教师场次根目录，可给多个")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument(
        "--min-clearance-m",
        default="0",
        help="0 或 off：不按净空再过滤。正数：成功场还要 min_obstacle_clearance_m >= 该值",
    )
    parser.add_argument("--max-windows", type=int, default=None, help="每场最多留多少窗。缺省全留")
    parser.add_argument(
        "--drop-start-still",
        action="store_true",
        help="丢掉起步结点之前的低速窗。默认关，关时窗口与不传逐位相同",
    )
    parser.add_argument(
        "--start-still-speed-mps",
        type=float,
        default=START_STILL_SPEED_MPS,
        help="起步结点的速度阈值（m/s）。只在 --drop-start-still 时生效，默认 0.05",
    )
    parser.add_argument(
        "--rewrite-start-knots",
        action="store_true",
        help="把起步静止段的位置、速度结点改写成沿目标的短移位。默认关，关时与不传逐位相同",
    )
    parser.add_argument(
        "--start-knot-speed-mps",
        type=float,
        default=START_KNOT_SPEED_MPS,
        help="改写用的沿目标速度（m/s）。只在 --rewrite-start-knots 时生效，默认 1.33",
    )
    parser.add_argument("--limit", type=int, default=None, help="最多转换多少场合格记录，按编号排序截断")
    parser.add_argument("--balance", choices=("full", "altitude"), default="full")
    parser.add_argument("--epochs", type=int, default=None, help="缺省 5，与 D5b r4 相同")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--threads", type=int, default=2)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    mode = resolve_mode(args.mode, args.only_convert, args.only_train)
    if args.max_windows is not None and int(args.max_windows) < 1:
        raise SystemExit("--max-windows 至少为 1")
    if args.epochs is not None and int(args.epochs) < 1:
        raise SystemExit("--epochs 至少为 1")
    if int(args.jobs) < 1:
        raise SystemExit("--jobs 至少为 1")
    clearance = parse_min_clearance(str(args.min_clearance_m))
    drop_start_still = bool(args.drop_start_still)
    start_still_speed = float(args.start_still_speed_mps)
    if drop_start_still and (not np.isfinite(start_still_speed) or start_still_speed < 0.0):
        raise SystemExit(f"--start-still-speed-mps 必须是非负有限数，收到 {args.start_still_speed_mps}")
    rewrite_start_knots = bool(args.rewrite_start_knots)
    start_knot_speed = float(args.start_knot_speed_mps)
    if rewrite_start_knots and (not np.isfinite(start_knot_speed) or start_knot_speed <= 0.0):
        raise SystemExit(f"--start-knot-speed-mps 必须是正有限数，收到 {args.start_knot_speed_mps}")
    out_dir = Path(args.out)
    manifest = None
    if mode in ("convert", "all"):
        if not args.teacher_dirs:
            raise SystemExit("转换需要 --teacher-dirs")
        rows = scan_roots([Path(path) for path in args.teacher_dirs], clearance)
        manifest = convert_records(
            rows,
            out_dir,
            jobs=int(args.jobs),
            max_windows=None if args.max_windows is None else int(args.max_windows),
            limit=None if args.limit is None else int(args.limit),
            min_clearance_m=clearance,
            teacher_dirs=[Path(path) for path in args.teacher_dirs],
            drop_start_still=drop_start_still,
            start_still_speed_mps=start_still_speed,
            rewrite_start_knots=rewrite_start_knots,
            start_knot_speed_mps=start_knot_speed,
        )
    if mode in ("train", "all"):
        if manifest is None:
            manifest = load_manifest(out_dir)
        epochs = int(R4_FIT["epochs"] if args.epochs is None else args.epochs)
        train_from_manifest(
            manifest,
            out_dir,
            balance=args.balance,
            epochs=epochs,
            seed=int(args.seed),
            threads=int(args.threads),
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
