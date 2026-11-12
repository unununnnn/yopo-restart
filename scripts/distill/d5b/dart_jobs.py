#!/usr/bin/env python3
"""D5b DART 式扰动：从教师成功轨迹上取状态，加水平位置/速度/偏航扰动，并把起终点高度设成 1.0 或 1.5 m，
交给 EGO-Swarm 从扰动后的状态接手，得到纠偏示范。用法 dart_jobs.py TAG PER_CELL OUT.jsonl"""
import json, sys
from pathlib import Path
import numpy as np
tag, per_cell, outp = sys.argv[1], int(sys.argv[2]), Path(sys.argv[3])
D2B = Path("/workspace/yopo-teacher-data/d2b/runs")
man = json.loads(Path("/workspace/yopo-teacher-data/d5b/convert-base-manifest.json").read_text())
train = set(man["train_ids"])
rng = np.random.default_rng(20261112)
CELLS = ["d000-n1-parallel", "d000-n1-headon", "d000-n3-parallel", "d000-n3-headon",
         "d014-n1-parallel", "d014-n1-headon", "d014-n3-parallel", "d014-n3-headon"]
SIG = {"pos_lat": 0.4, "vel_lat": 0.6, "vel_lon": 0.3, "yaw": 0.35}
jobs = []
for cell in CELLS:
    runs = sorted(r for r in train if r.startswith(cell + "-s"))
    made = 0; tries = 0
    while made < per_cell and tries < per_cell * 20:
        tries += 1
        rid = runs[int(rng.integers(len(runs)))]
        scn = json.loads((D2B / rid / "scenario.json").read_text())
        tr = np.load(D2B / rid / "trajectory.npz")
        T = tr["time_s"]; P = tr["position_m"]; V = tr["velocity_mps"]; A = tr["acceleration_mps2"]; Y = tr["yaw_rad"]
        if T[-1] < 4.0: continue
        i = int(rng.integers(5, max(6, len(T) - 20)))
        alt = float(rng.choice([1.0, 1.5]))
        drones = []; ok = True
        for d in range(P.shape[1]):
            yaw = float(Y[i, d]) + float(rng.normal(0, SIG["yaw"]))
            lat = np.array([-np.sin(Y[i, d]), np.cos(Y[i, d]), 0.0]); lon = np.array([np.cos(Y[i, d]), np.sin(Y[i, d]), 0.0])
            p = P[i, d] + lat * rng.normal(0, SIG["pos_lat"]); p[2] = alt
            v = V[i, d] + lat * rng.normal(0, SIG["vel_lat"]) + lon * rng.normal(0, SIG["vel_lon"]); v[2] = 0.0
            g = np.array(scn["drones"][d]["goal_enu"], float); g[2] = alt
            if abs(p[0]) > 11 or abs(p[1]) > 11 or np.linalg.norm(g[:2] - p[:2]) < 1.5: ok = False
            for c in scn["cylinders"]:
                if np.hypot(p[0] - c["center_xy"][0], p[1] - c["center_xy"][1]) - c["radius"] < 0.35: ok = False
            drones.append({"ros_id": d, "p": p.tolist(), "v": v.tolist(), "a": A[i, d].tolist(), "yaw": yaw, "goal": g.tolist()})
        for a in range(len(drones)):
            for b in range(a + 1, len(drones)):
                if np.linalg.norm(np.array(drones[a]["p"]) - np.array(drones[b]["p"])) < 0.8: ok = False
        if not ok: continue
        jobs.append({"id": f"{tag}-{rid}-i{i}-z{alt:.1f}", "base": rid, "t": float(T[i]), "round": tag, "drones": drones,
                     "dart": {"sigma": SIG, "alt_m": alt}})
        made += 1
outp.write_text("".join(json.dumps(j) + "\n" for j in jobs))
print(tag, len(jobs))
