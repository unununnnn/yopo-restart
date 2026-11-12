#!/usr/bin/env python3
"""D5b：四场景 × seeds 0–9 学生闭环，只报数。用法 eval.py WEIGHTS OUTDIR [variant]"""
import json, os, subprocess, sys
from pathlib import Path
import numpy as np
W, OUT = sys.argv[1], Path(sys.argv[2]); VAR = sys.argv[3] if len(sys.argv) > 3 else "b"
SP = sys.argv[4] if len(sys.argv) > 4 else "{}"; os.environ["D5B_STUDENT_PARAMS"] = SP
WT = os.environ.get("YOPO_WT", "/workspace/wt/yopo-restart/yopo-restart-bot-d5-student-loop")
PY = f"{WT}/.venv/bin/python"
SCENES = [("g2-pillar", "g2 柱子"), ("swap-3-open", "开阔三机对向"), ("open-3drone", "开阔三机平行"), ("ego-density-014-solo", "密度 0.14 单机")]
OUT.mkdir(parents=True, exist_ok=True)
jobs = [f"{sc} {s}" for sc, _ in SCENES for s in range(10) if not (OUT / f"{sc}-s{s}.json").is_file()]
if jobs:
    cmd = (f"xargs -P 3 -L 1 bash -c 'nice -n 10 timeout 2400 {PY} /workspace/yopo-ops/d5b/rollout.py --card {WT}/scenes/$0.json "
           f"--seed $1 --weights {W} --variant {VAR} --out {OUT}/$0-s$1.json > {OUT}/$0-s$1.log 2>&1'")
    subprocess.run(cmd, shell=True, input="\n".join(jobs) + "\n", text=True)
rows = []
for sc, label in SCENES:
    R = [json.loads((OUT / f"{sc}-s{s}.json").read_text()) for s in range(10) if (OUT / f"{sc}-s{s}.json").is_file()]
    n = max([r["n_drones"] or 0 for r in R] or [0])
    kind = lambda r: "arrived" if r["outcome"] == "SUCCESS" else ("inter" if "INTER_DRONE" in str(r["reason"]) else ("obstacle" if r["outcome"] == "COLLISION" else "timeout"))
    cl = [r["min_obstacle_distance_m"] - 0.35 for r in R if r.get("min_obstacle_distance_m") is not None]
    gap = [r["min_inter_drone_distance_m"] for r in R if r.get("min_inter_drone_distance_m") is not None]
    rows.append({"scene": sc, "label": label, "seeds": len(R), "arrived": sum(int(r["arrived"] or 0) for r in R), "denom": n * len(R),
                 "obstacle": sum(kind(r) == "obstacle" for r in R), "inter": sum(kind(r) == "inter" for r in R),
                 "timeout": sum(kind(r) == "timeout" for r in R), "min_clearance_m": min(cl) if cl else None,
                 "min_inter_m": min(gap) if gap else None,
                 "final_goal_err_median_m": float(np.median([r["final_goal_error_m"] for r in R if r.get("final_goal_error_m") is not None])) if R else None})
(OUT / "summary.json").write_text(json.dumps({"weights": W, "variant": VAR, "student_params": json.loads(SP), "wt": WT, "rows": rows, "report_only": True}, ensure_ascii=False, indent=1))
for r in rows: print(json.dumps(r, ensure_ascii=False))
