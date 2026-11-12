#!/usr/bin/env python3
"""把冻结的 D2b 成功教师场次转成学生窗口张量。

只读 /workspace/yopo-teacher-data/d2b/ 在 2026-11-12 01:26:35 UTC-3 冻结的 663 场，
其中成功 635 场。每场一个 npz，已存在且同 sha 的跳过，可中断续跑。

    .venv/bin/python scripts/distill/convert_d2b.py \\
        --jobs 2 --out /workspace/yopo-teacher-data/d5
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

CUTOFF_DIR = Path("/workspace/yopo-teacher-data/d2b/snapshots/20261112T012635")
WORK = Path("/workspace/yopo-teacher-data/d2b")
CUTOFF_LOCAL = "2026-11-12 01:26:35 -0300"


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_frozen(cutoff: Path = CUTOFF_DIR, work: Path = WORK) -> dict:
    cutoff_json = json.loads((cutoff / "cutoff.json").read_text(encoding="utf-8"))
    if cutoff_json.get("cutoff_local") != CUTOFF_LOCAL:
        raise SystemExit(f"cutoff.json 不是冻结时刻 {CUTOFF_LOCAL}：{cutoff_json}")
    ids = [line.strip() for line in (cutoff / "cutoff-ids.txt").read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(ids) != int(cutoff_json["n_ids"]):
        raise SystemExit(f"cutoff-ids 条数 {len(ids)} 与 cutoff.json 的 n_ids 不一致")
    batch = json.loads((cutoff / "d2b-teacher-batch.json").read_text(encoding="utf-8"))
    if batch.get("cutoff_local") != CUTOFF_LOCAL:
        raise SystemExit("d2b-teacher-batch.json 的冻结时刻对不上")
    by_id = {r["id"]: r for r in batch["runs"]}
    missing = [i for i in ids if i not in by_id]
    if missing:
        raise SystemExit(f"批次清单缺少冻结编号，例如 {missing[:3]}")
    success = [i for i in ids if by_id[i].get("success") is True]
    failed = [i for i in ids if by_id[i].get("success") is not True]
    return {
        "cutoff": cutoff_json,
        "ids": ids,
        "success_ids": success,
        "failed_ids": failed,
        "runs": by_id,
        "batch": batch,
        "work": work,
    }


def _convert_one(args: tuple) -> dict:
    run_id, run_dir, out_path = args
    from yopo_l0.distill import DistillError, convert_run, save_batch

    t0 = time.perf_counter()
    try:
        batch = convert_run(Path(run_dir))
    except DistillError as exc:
        return {"id": run_id, "ok": False, "error": str(exc), "wall_s": time.perf_counter() - t0}
    if batch.n == 0:
        return {
            "id": run_id,
            "ok": False,
            "error": batch.skipped or "no_windows",
            "n": 0,
            "wall_s": time.perf_counter() - t0,
        }
    path = save_batch(batch, Path(out_path))
    sha = _sha256(path)
    return {
        "id": run_id,
        "ok": True,
        "n": batch.n,
        "path": str(path),
        "sha256": sha,
        "bytes": path.stat().st_size,
        "skipped": batch.skipped,
        "wall_s": time.perf_counter() - t0,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="冻结的 D2b 成功场次 → 学生窗口")
    parser.add_argument("--work", type=Path, default=WORK)
    parser.add_argument("--cutoff", type=Path, default=CUTOFF_DIR)
    parser.add_argument("--out", type=Path, default=Path("/workspace/yopo-teacher-data/d5"))
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--limit", type=int, default=None)
    args = parser.parse_args(argv)

    frozen = load_frozen(args.cutoff, args.work)
    out = Path(args.out)
    win_dir = out / "windows"
    win_dir.mkdir(parents=True, exist_ok=True)
    status_path = out / "convert-status.jsonl"
    done: dict[str, dict] = {}
    if status_path.is_file():
        for line in status_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            rec = json.loads(line)
            if rec.get("ok") and rec.get("id"):
                target = win_dir / f"{rec['id']}.npz"
                if target.is_file() and rec.get("sha256") == _sha256(target):
                    done[rec["id"]] = rec

    todo = []
    ids = list(frozen["success_ids"])
    if args.limit is not None:
        ids = ids[: int(args.limit)]
    for run_id in ids:
        if run_id in done:
            continue
        run_dir = Path(args.work) / "runs" / run_id
        if not (run_dir / "trajectory.npz").is_file():
            print(f"缺轨迹：{run_dir}", file=sys.stderr)
            return 1
        todo.append((run_id, str(run_dir), str(win_dir / f"{run_id}.npz")))

    jobs = max(1, min(int(args.jobs), 2))
    print(f"冻结成功 {len(frozen['success_ids'])}，已转 {len(done)}，待转 {len(todo)}，jobs={jobs}", flush=True)
    if todo:
        with status_path.open("a", encoding="utf-8") as fh, ProcessPoolExecutor(max_workers=jobs) as pool:
            futs = [pool.submit(_convert_one, item) for item in todo]
            for fut in as_completed(futs):
                rec = fut.result()
                fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
                fh.flush()
                if rec.get("ok"):
                    done[rec["id"]] = rec
                    print(f"ok {rec['id']} n={rec['n']} {rec['wall_s']:.2f}s", flush=True)
                else:
                    print(f"fail {rec['id']} {rec.get('error')}", flush=True)
                    return 1

    from yopo_l0.distill import split_runs_by_seed

    train_ids, hold_ids = split_runs_by_seed([rec["id"] for rec in done.values()])
    manifest = {
        "schema": "yopo-student-d5-convert/1",
        "cutoff_local": CUTOFF_LOCAL,
        "cutoff_zone": "America/Argentina/San_Juan (UTC-3)",
        "assumption": "地图边长和障碍半径是假设，不是论文复现。",
        "source_work": str(args.work),
        "n_frozen": len(frozen["ids"]),
        "n_success": len(frozen["success_ids"]),
        "n_converted": len(done),
        "n_windows": sum(int(r["n"]) for r in done.values()),
        "hold_seed_mod": 5,
        "hold_rule": "seed % 5 == 0 进留出，其余进训练。同一场次不会跨集合。",
        "train_ids": train_ids,
        "hold_ids": hold_ids,
        "n_train_runs": len(train_ids),
        "n_hold_runs": len(hold_ids),
        "files": {k: {"path": v["path"], "sha256": v["sha256"], "n": v["n"]} for k, v in sorted(done.items())},
    }
    (out / "convert-manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        f"完成 {manifest['n_converted']} 场，{manifest['n_windows']} 个窗口，"
        f"训练 {len(train_ids)}，留出 {len(hold_ids)}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
