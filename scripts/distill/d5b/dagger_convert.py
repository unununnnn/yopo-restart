#!/usr/bin/env python3
"""D5b：重标注场次 → 窗口，按净空过滤。只留未来 1.6 s 内到障碍表面净空 ≥ 0（机体半径 0.20 m 已扣）、机间 ≥ 0.40 m 的窗口。"""
import hashlib, json, os, sys
from pathlib import Path
import numpy as np
WT = os.environ.get("YOPO_WT", "/workspace/wt/yopo-restart/yopo-restart-bot-d5-student-loop")
sys.path.insert(0, WT)
from yopo_l0.distill import convert_run, save_batch, WindowBatch
from dataclasses import replace
RUNS = Path("/workspace/yopo-teacher-data/d5b/relabel/runs")
HZN = 1.6

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def filt(rd, b):
    tr = np.load(rd / "trajectory.npz")
    T = tr["time_s"]; cl = tr["clearance_m"]; P = tr["position_m"]
    keep = np.zeros(b.n, bool)
    for w in range(b.n):
        t = b.time_s[w]; d = int(b.ego_index[w])
        m = (T >= t - 1e-6) & (T <= t + HZN + 1e-6)
        ok = bool(np.all(cl[m, d] >= 0.0))
        for j in range(P.shape[1]):
            if j != d and np.any(np.linalg.norm(P[m, d] - P[m, j], axis=1) < 0.40):
                ok = False
        keep[w] = ok
    return keep

def main(round_tag, out_dir):
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    rows = {}; cells = {}
    for rd in sorted(RUNS.glob(f"{round_tag}-*")):
        rec = json.loads((rd / "record.json").read_text()) if (rd / "record.json").is_file() else None
        if rec is None: continue
        base = rec["scenario"]["base"]; cell = base.rsplit("-s", 1)[0]
        c = cells.setdefault(cell, {"runs": 0, "usable_runs": 0, "windows_all": 0, "windows_kept": 0})
        c["runs"] += 1
        if (rec.get("metrics") or {}).get("success") is not True: continue
        dest = out / f"{rd.name}.npz"
        b = convert_run(rd)
        if b.n == 0: continue
        k = filt(rd, b)
        c["usable_runs"] += 1; c["windows_all"] += int(b.n); c["windows_kept"] += int(k.sum())
        if not k.any(): continue
        fb = WindowBatch(**{f: (getattr(b, f)[k] if isinstance(getattr(b, f), np.ndarray) else getattr(b, f)) for f in b.__dataclass_fields__})
        save_batch(fb, dest)
        rows[rd.name] = {"path": str(dest), "sha256": sha(dest), "n": int(k.sum()), "base": base, "src_record_sha256": sha(rd / "record.json")}
    man = {"round": round_tag, "filter": "未来 1.6 s 净空 ≥ 0（机体半径 0.20 m 已扣），机间 ≥ 0.40 m；只用记录跨度 ≥ 1.8 s 的重标注场",
           "per_cell": cells, "n_files": len(rows), "n_windows": sum(r["n"] for r in rows.values()), "files": rows,
           "note": "密集格里 EGO-Swarm 自己也常擦树（D2c），重标注在那些格不可靠，只能靠净空过滤兜底。"}
    (out / "manifest.json").write_text(json.dumps(man, ensure_ascii=False, indent=1))
    print(round_tag, man["n_files"], man["n_windows"], json.dumps(cells, ensure_ascii=False))

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
