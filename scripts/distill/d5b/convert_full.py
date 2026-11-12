#!/usr/bin/env python3
"""D5b：把 D2b 全量 1792 场里 metrics.success 为真的场次转成学生窗口（可续跑）。
源：/workspace/yopo-teacher-data/d2b/runs/；输出：/workspace/yopo-teacher-data/d5b/windows-base/
同时写清单（源文件和窗口的 sha256）以及与 D5 冻结 663 场的差异。"""
import hashlib, json, sys, os, time
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
WT = os.environ.get("YOPO_WT", "/workspace/wt/yopo-restart/yopo-restart-bot-d5-student-loop")
sys.path.insert(0, WT)
RUNS = Path("/workspace/yopo-teacher-data/d2b/runs")
OUT = Path("/workspace/yopo-teacher-data/d5b")
FROZEN = Path("/workspace/yopo-teacher-data/d2b/snapshots/20261112T012635")

def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for c in iter(lambda: f.read(1 << 20), b""): h.update(c)
    return h.hexdigest()

def one(rid):
    from yopo_l0.distill import convert_run, save_batch, DistillError
    t0 = time.time(); rd = RUNS / rid
    try:
        b = convert_run(rd)
    except DistillError as e:
        return {"id": rid, "ok": False, "error": str(e)}
    if b.n == 0:
        return {"id": rid, "ok": False, "error": b.skipped}
    p = save_batch(b, OUT / "windows-base" / f"{rid}.npz")
    return {"id": rid, "ok": True, "n": b.n, "path": str(p), "sha256": sha(p),
            "src_record_sha256": sha(rd / "record.json"), "src_traj_sha256": sha(rd / "trajectory.npz"), "wall_s": time.time() - t0}

def main():
    from yopo_l0.distill import parse_run_id, split_runs_by_seed
    ids_all = sorted(p.name for p in RUNS.iterdir() if (p / "record.json").is_file())
    succ, cells = [], Counter(); cell_succ = Counter()
    for rid in ids_all:
        r = json.loads((RUNS / rid / "record.json").read_text())
        key = parse_run_id(rid)["scene_key"]; cells[key] += 1
        if (r.get("metrics") or {}).get("success") is True:
            succ.append(rid); cell_succ[key] += 1
    status = OUT / "convert-base-status.jsonl"; done = {}; skipped = []
    if status.is_file():
        for l in status.read_text().splitlines():
            if l.strip():
                x = json.loads(l)
                if x.get("ok") and Path(x["path"]).is_file(): done[x["id"]] = x
    todo = [r for r in succ if r not in done]
    print(f"全量 {len(ids_all)}，成功 {len(succ)}，已转 {len(done)}，待转 {len(todo)}", flush=True)
    with status.open("a") as fh, ProcessPoolExecutor(max_workers=2) as pool:
        for fut in as_completed([pool.submit(one, r) for r in todo]):
            x = fut.result(); fh.write(json.dumps(x) + "\n"); fh.flush()
            if x.get("ok"): done[x["id"]] = x
            else: skipped.append(x)
    frozen_ids = set(l.strip() for l in (FROZEN / "cutoff-ids.txt").read_text().splitlines() if l.strip())
    d5_man = json.loads(Path("/workspace/yopo-teacher-data/d5/convert-manifest.json").read_text())
    d5_ids = set(d5_man["files"])
    train, hold = split_runs_by_seed(sorted(done))
    per_cell = {}
    for k in sorted(cells):
        conv = [r for r in done if parse_run_id(r)["scene_key"] == k]
        per_cell[k] = {"runs": cells[k], "success": cell_succ[k], "converted": len(conv),
                       "windows": sum(done[r]["n"] for r in conv), "in_d5": sum(1 for r in conv if r in d5_ids)}
    man = {"schema": "yopo-student-d5b-convert/1", "source": str(RUNS), "n_runs": len(ids_all), "n_success": len(succ),
           "n_converted": len(done), "n_windows": sum(x["n"] for x in done.values()),
           "skipped": {x["id"]: x.get("error") for x in skipped},
           "filter": "只转 record.json 的 metrics.success 为 true 的场次（D2 的成功判据已含机体半径 0.20 m 的障碍碰撞，碰到即不成功）。",
           "vs_d5": {"d5_frozen_ids": len(frozen_ids), "d5_converted": len(d5_ids), "new_success_not_in_d5": len(set(done) - d5_ids),
                      "d5_ids_missing_now": sorted(d5_ids - set(done))},
           "hold_rule": "seed % 5 == 0 进留出，同 D5", "train_ids": train, "hold_ids": hold, "per_cell": per_cell,
           "files": {k: {kk: v[kk] for kk in ("path", "sha256", "n", "src_record_sha256", "src_traj_sha256")} for k, v in sorted(done.items())},
           "assumption": "地图边长和障碍半径是假设，不是论文复现。"}
    (OUT / "convert-base-manifest.json").write_text(json.dumps(man, ensure_ascii=False, indent=1))
    print("完成", man["n_converted"], man["n_windows"], flush=True)

if __name__ == "__main__":
    main()
