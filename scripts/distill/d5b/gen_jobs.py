#!/usr/bin/env python3
"""从学生闭环记录的状态挑重标注起点。用法 gen_jobs.py ROUND ROLLOUT_DIR OUT.jsonl"""
import json, sys
from pathlib import Path
import numpy as np
rnd, rdir, outp = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])
D2B = Path("/workspace/yopo-teacher-data/d2b/runs")
EVERY_S, T0, CAP = 1.0, 0.5, 12
jobs = []
for f in sorted(rdir.glob("*.json")):
    r = json.loads(f.read_text()); base = f.stem
    scn = json.loads((D2B / base / "scenario.json").read_text())
    ids = [d["id"] for d in scn["drones"]]
    by_t = {}
    for s in r["states"]:
        by_t.setdefault(round(s["t"], 2), {})[s["id"]] = s
    picked = 0
    for t in sorted(by_t):
        if t < T0 - 1e-6 or abs((t - T0) / EVERY_S - round((t - T0) / EVERY_S)) > 1e-3: continue
        row = by_t[t]
        if set(row) != set(ids): continue
        ok = True
        for s in row.values():
            p = np.asarray(s["p"])
            if s["code"] != "OK" or abs(p[0]) > 11.0 or abs(p[1]) > 11.0 or not (0.4 <= p[2] <= 3.4): ok = False
            for c in scn["cylinders"]:
                if np.hypot(p[0] - c["center_xy"][0], p[1] - c["center_xy"][1]) - c["radius"] < 0.25: ok = False
            if np.linalg.norm(np.asarray(s["goal"]) - p) < 1.0: ok = False
        if not ok: continue
        drones = [{"ros_id": i, **{k: row[d][k] for k in ("p", "v", "a", "yaw", "goal")}} for i, d in enumerate(ids)]
        jobs.append({"id": f"{rnd}-{base}-t{t:.1f}", "base": base, "t": t, "round": rnd, "drones": drones})
        picked += 1
        if picked >= CAP: break
outp.write_text("".join(json.dumps(j) + "\n" for j in jobs))
print(rnd, len(jobs))
