#!/usr/bin/env python3
"""D5b DAgger 重标注：EGO-Swarm 从学生走到的状态（p, v, a, yaw）起飞，记录之后的教师轨迹。

用法: relabel.py JOBS.jsonl --container-index K --stride 3
每个 job: {"id", "base", "t", "drones": [{"ros_id", "p", "v", "a", "yaw", "goal"}]}
已有 record.json 的跳过。地图边长和障碍半径沿用 D2 假设，不是论文复现。
"""
import argparse, json, math, os, subprocess, sys, time
from pathlib import Path
import numpy as np
WT = os.environ.get("YOPO_WT", "/workspace/wt/yopo-restart/yopo-restart-bot-d5-student-loop")
sys.path.insert(0, str(Path(WT) / "scripts"))
from teacher.launch_xml import launch_xml  # noqa: E402

WORK = Path("/workspace/yopo-teacher-data/d5b/relabel")
D2B = Path("/workspace/yopo-teacher-data/d2b/runs")
BODY_R = 0.20  # 教师机体半径假设（D2）
AFTER_S = 4.0


def clearance(scn, P):
    """P (T, D, 3) → (T, D) 到最近圆柱表面的水平距离（不含机体半径）。无障碍为 +inf。"""
    out = np.full(P.shape[:2], np.inf)
    for c in scn["cylinders"]:
        cx, cy = c["center_xy"]; r = c["radius"]
        d = np.hypot(P[..., 0] - cx, P[..., 1] - cy) - r
        inz = (P[..., 2] >= c["z_lo"] - BODY_R) & (P[..., 2] <= c["z_hi"] + BODY_R)
        out = np.where(inz, np.minimum(out, d), out)
    return out


def run_job(job, container):
    rd = WORK / "runs" / job["id"]
    if (rd / "record.json").is_file():
        return "skip"
    rd.mkdir(parents=True, exist_ok=True)
    base = json.loads((D2B / job["base"] / "scenario.json").read_text())
    scn = dict(base)
    scn["id"] = job["id"]
    if "cylinders" in job:
        scn["cylinders"] = job["cylinders"]
    drones = []
    for bd, jd in zip(base["drones"], job["drones"]):
        assert int(bd["ros_id"]) == int(jd["ros_id"])
        drones.append({**bd, "start_enu": [float(x) for x in jd["p"]], "goal_enu": [float(x) for x in jd["goal"]],
                       "seed_state": {k: jd[k] for k in ("p", "v", "a", "yaw")}})
    scn["drones"] = drones
    scn["record_after_handover_s"] = AFTER_S
    scn["dagger"] = {"base": job["base"], "t_student_s": job["t"], "round": job.get("round")}
    (rd / "scenario.json").write_text(json.dumps(scn))
    (rd / "teacher.launch").write_text(launch_xml(scn))
    remote = f"/teacher-runs/runs/{job['id']}"
    script = f"""#!/bin/bash
set +u
source /opt/ros/noetic/setup.bash
source /ego/devel/setup.bash
set -u
export PYTHONUNBUFFERED=1
python3 /teacher-runs/kill_ros.py
roscore > {remote}/roscore.log 2>&1 &
for _i in $(seq 1 25); do rosparam list >/dev/null 2>&1 && break; sleep 0.2; done
python3 -u /ws/dagger_record.py --scenario {remote}/scenario.json --out {remote} --timeout 25 > {remote}/record.log 2>&1 &
rec=$!
sleep 1
LD_PRELOAD=/teacher-runs/linebuf.so roslaunch {remote}/teacher.launch > {remote}/roslaunch.log 2>&1 &
launch=$!
deadline=$((SECONDS + 40))
while [ "$SECONDS" -lt "$deadline" ]; do
  [ -f {remote}/done.json ] && break
  kill -0 "$launch" 2>/dev/null || break
  sleep 0.3
done
kill -INT "$rec" 2>/dev/null || true
for _i in $(seq 1 20); do [ -f {remote}/done.json ] && break; sleep 0.3; done
kill -INT "$launch" 2>/dev/null || true
sleep 1
kill -KILL "$rec" "$launch" 2>/dev/null || true
python3 /teacher-runs/kill_ros.py
chmod -R a+rwX {remote} || true
"""
    (rd / "run.sh").write_text(script); (rd / "run.sh").chmod(0o755)
    t0 = time.time()
    subprocess.run(["sudo", "-n", "docker", "exec", container, "bash", f"{remote}/run.sh"], timeout=120)
    subprocess.run(["sudo", "-n", "chmod", "-R", "a+rwX", str(rd)], check=False)
    raw_p = rd / "raw_record.json"
    rec = {"schema": "yopo-teacher-record/1", "scenario": {"id": job["id"], "n_drones": len(drones), "base": job["base"]},
           "dagger": scn["dagger"], "wall_s": time.time() - t0}
    if not raw_p.is_file():
        rec["metrics"] = {"success": None, "why": "no_raw"}
        (rd / "record.json").write_text(json.dumps(rec)); return "noraw"
    raw = json.loads(raw_p.read_text())
    T = np.asarray(raw["time_s"], float)
    hand = raw.get("handover_ros_s") or {}
    if len(hand) < len(drones) or T.size == 0 or raw.get("t0_ros_s") is None:
        rec["metrics"] = {"success": None, "why": f"no_handover status={raw.get('run_status')} n_hand={len(hand)}"}
        (rd / "record.json").write_text(json.dumps(rec)); return "nohand"
    t_h = max(hand.values()) - float(raw["t0_ros_s"])
    keep = T >= t_h - 1e-6
    T = T[keep] - T[keep][0]
    P = np.asarray(raw["position_m"], float)[keep]; V = np.asarray(raw["velocity_mps"], float)[keep]
    A = np.asarray(raw["acceleration_mps2"], float)[keep]; Y = np.asarray(raw["yaw_rad"], float)[keep]
    G = np.asarray([d["goal_enu"] for d in drones], float)
    cl = clearance(scn, P) - BODY_R
    inter = np.inf
    for i in range(P.shape[1]):
        for j in range(i + 1, P.shape[1]):
            inter = min(inter, float(np.linalg.norm(P[:, i] - P[:, j], axis=1).min()))
    # 偏航按 D2 记录，换成连续角，避免插值抄近路
    Y = np.unwrap(Y, axis=0)
    np.savez_compressed(rd / "trajectory.npz", time_s=T, position_m=P, velocity_mps=V, acceleration_mps2=A, yaw_rad=Y,
                        goal_m=G, clearance_m=cl)
    # 起飞时的状态差（检查教师是否真从学生状态出发）
    p0err = float(max(np.linalg.norm(P[0, k] - np.asarray(jd["p"])) for k, jd in enumerate(job["drones"])))
    v0err = float(max(np.linalg.norm(V[0, k] - np.asarray(jd["v"])) for k, jd in enumerate(job["drones"])))
    ok = bool(T[-1] >= 1.6 + 0.2)
    rec["metrics"] = {"success": ok, "min_clearance_m": float(np.min(cl)) if np.isfinite(cl).any() else None,
                      "min_inter_m": inter if np.isfinite(inter) else None, "span_s": float(T[-1]),
                      "handover_s": t_h, "p0_err_m": p0err, "v0_err_mps": v0err, "status": raw.get("run_status")}
    (rd / "record.json").write_text(json.dumps(rec))
    # 场景卡：沿用教师卡，换起点
    card = json.loads((D2B / job["base"] / "scene.json").read_text())
    card["name"] = job["id"]
    card["drones"] = [{**cd, "start_enu": d["start_enu"], "goal_enu": d["goal_enu"]} for cd, d in zip(card["drones"], drones)]
    card["geometry"] = "geometry.json"
    (rd / "scene.json").write_text(json.dumps(card))
    import shutil
    shutil.copy(D2B / job["base"] / "geometry.json", rd / "geometry.json")
    return "ok"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("jobs"); ap.add_argument("--container-index", type=int, default=0); ap.add_argument("--stride", type=int, default=1)
    a = ap.parse_args()
    jobs = [json.loads(l) for l in Path(a.jobs).read_text().splitlines() if l.strip()]
    cont = f"ego-teacher-d5b-{a.container_index}"
    for k, job in enumerate(jobs):
        if k % a.stride != a.container_index:
            continue
        try:
            r = run_job(job, cont)
        except Exception as e:  # 单场失败不拖垮整批
            r = f"err {e!r}"
        print(time.strftime("%H:%M:%S"), job["id"], r, flush=True)


if __name__ == "__main__":
    main()
