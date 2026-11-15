#!/usr/bin/env python3
"""D7：学生 DAgger 在策略重标注，小规模（与 D6e 同量级）。只报数。

起点是 D6e 关臂的学生：仓库 10 场成功样本、D5b r4 FitConfig、5 epoch。每一轮：

1. rollout：当前学生在开阔三机与 g2 柱子的训练种子上闭环飞（1.0 m 与 1.5 m）。
   训练种子和评估种子不重叠。β = 0：执行的是学生自己的动作，教师不进闭环。
2. jobs：在学生走到的状态上每 0.5 s 取一个重标注起点。
3. relabel：EGO-Swarm（yopo-teacher-d2b:92fe9f7）从这些状态（p、v、a、yaw）接手，记 4 s。
4. convert：重标注轨迹转窗口，按 L0 判据过滤（未来 1.6 s 障碍净空 ≥ 机体半径 0.35 m，机间 ≥ 0.70 m）。
5. train：基础训练窗 + 第 1..k 轮全部重标注窗，从头训，同一套超参，等权。
6. eval：同 D6e 口径，开阔三机 seeds 0–9、g2 seeds 0–4，1.0 m 与 1.5 m。

库默认不改：不改 ``StudentPolicy.DEFAULTS``、不改场景卡、不改 ``convert_run`` 的默认。
本脚本整体是只报数的研究开关，不跑就不生效；结果标「偏离上游」。
g2 的柱子在教师侧近似成外接圆柱（中心 (3.0, 0)，半径 0.5 m，高 0–2.8 m），墙不发给教师。
这是假设，不是论文复现。

    .venv/bin/python scripts/distill/d7/dagger.py all --out /workspace/yopo-teacher-data/d7
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SCENES = {
    "open-3drone": ROOT / "scenes" / "open-3drone.json",
    "g2-pillar": ROOT / "scenes" / "g2-pillar.json",
}
ALTITUDES = (1.0, 1.5)
EVAL_SEEDS = {"open-3drone": list(range(10)), "g2-pillar": list(range(5))}
TRAIN_SEEDS = {
    1: {"open-3drone": [10, 11, 12], "g2-pillar": [5, 6]},
    2: {"open-3drone": [13, 14, 15], "g2-pillar": [7, 8]},
    3: {"open-3drone": [16, 17, 18], "g2-pillar": [9, 10]},
}
G2_TEACHER_CYLINDERS = [{"center_xy": [3.0, 0.0], "radius": 0.5, "z_lo": 0.0, "z_hi": 2.8}]
EVERY_S = 0.5
T0_S = 0.5
CAP_PER_ROLLOUT = 15
GOAL_SKIP_M = 0.5
L0_BODY_M = 0.35
L0_INTER_M = 0.70
HORIZON_S = 1.6
AFTER_S = 4.0
EGO_DIR = Path("/home/box/ext/ego-planner-swarm")
IMAGE = "yopo-teacher-d2b:92fe9f7"
RECORDER_DIR = ROOT / "scripts" / "distill" / "d5b" / "ros"
N_CONTAINERS = 3
DISCLAIMER = (
    "只报数。report-only。偏离上游。DAgger 只在本脚本里发生，库默认不改。"
    "教师是 EGO-Swarm 92fe9f7，从学生状态接手；g2 柱子在教师侧是外接圆柱近似，墙不发给教师，"
    "这是假设，不是论文复现。规模与 D6e 同量级，不是全量 D6b。"
)


def _load(name: str, path: Path):
    cached = sys.modules.get(name)
    if cached is not None:
        return cached
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _d6d():
    return _load("d7_d6d_compare", ROOT / "scripts" / "distill" / "d6d" / "compare.py")


def _eval():
    return _load("d7_eval_altitudes", ROOT / "scripts" / "distill" / "d6" / "eval_altitudes.py")


def _pipe():
    return _load("d7_pipeline", ROOT / "scripts" / "distill" / "d6" / "pipeline.py")


def _write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(_pipe()._jsonable(payload), ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    tmp.replace(path)


def _sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _map(fn, items, jobs):
    return _d6d()._map(fn, items, jobs)


# ---------- 起点 ----------

def build_base(out: Path, jobs: int) -> dict:
    d6d = _d6d()
    base = out / "base"
    manifest = d6d.convert_arm(base, drop=False, jobs=jobs)

    feas = _load("d7_feasibility", ROOT / "scripts" / "distill" / "d6" / "feasibility.py")
    train_ids, hold_ids = feas.split_sample_ids(list(manifest["files"]))
    info = {
        "train_paths": [manifest["files"][i]["path"] for i in train_ids],
        "hold_paths": [manifest["files"][i]["path"] for i in hold_ids],
        "train_ids": train_ids,
        "hold_ids": hold_ids,
        "n_train_windows": sum(int(manifest["files"][i]["n"]) for i in train_ids),
        "n_hold_windows": sum(int(manifest["files"][i]["n"]) for i in hold_ids),
                "drop_start_still": False,
    }
    _write_json(base / "base.json", info)
    return info


def train_round(out: Path, k: int) -> dict:
    from yopo_l0.student_fit import FitConfig, train_from_paths

    base = json.loads((out / "base" / "base.json").read_text())
    extra = []
    for r in range(1, k + 1):
        man = out / f"r{r}" / "windows" / "manifest.json"
        if not man.is_file():
            raise SystemExit(f"缺少第 {r} 轮重标注窗：{man}")
        extra.extend(item["path"] for item in json.loads(man.read_text())["files"].values())
    train_paths = [Path(p) for p in base["train_paths"]] + [Path(p) for p in extra]
    hold_paths = [Path(p) for p in base["hold_paths"]]
    pipe = _pipe()
    fit = dict(pipe.R4_FIT)
    tdir = out / f"r{k}" / "train"
    with pipe._TupleJson():
        summary = train_from_paths(train_paths, hold_paths, tdir, cfg=FitConfig(**fit))
    n_extra = 0
    for r in range(1, k + 1):
        n_extra += int(json.loads((out / f"r{r}" / "windows" / "manifest.json").read_text())["n_windows"])
    summary["round"] = k
    summary["n_base_train_windows"] = int(base["n_train_windows"])
    summary["n_dagger_windows"] = n_extra
    summary["n_dagger_files"] = len(extra)
    summary["fit_used"] = fit
    _write_json(tdir / "d7-train.json", summary)
    return summary


# ---------- 闭环采样 ----------

def rollout_one(spec: dict) -> dict:
    os.environ["OMP_NUM_THREADS"] = "1"
    import torch

    torch.set_num_threads(1)
    from yopo_l0.runner import RunConfig, run
    from yopo_l0.scene import load_card

    ev = _eval()
    card = ev.retarget_card(load_card(spec["card"]), spec["altitude_m"], "off", spec["weights"])
    output = run(card, RunConfig(seed=int(spec["seed"]), policy="student", shadow=False, duration_s=None))
    goals = {d["id"]: [float(x) for x in d["goal_enu"]] for d in card.card["drones"]}
    states = []
    for row in output.ticks:
        if int(row["k"]) % 10 != 9:  # 每 0.1 s 留一拍（tick 的 t 是步后时刻）
            continue
        states.append({
            "t": round(float(row["t"]), 2), "id": row["uav"], "p": row["p"], "v": row["v"], "a": row["a"],
            "yaw": float(row["yaw"]), "goal": goals[row["uav"]], "min_obstacle_m": float(row["min_obstacle_m"]),
        })
    result = output.result
    body = {
        "scene": spec["scene"], "altitude_m": spec["altitude_m"], "seed": spec["seed"], "round": spec["round"],
        "weights": spec["weights"], "outcome": result.get("outcome"), "reason": result.get("reason"),
        "arrived": result.get("arrived"), "n_drones": result.get("n_drones"),
        "final_goal_error_m": result.get("final_goal_error_m"), "sim_duration_s": result.get("sim_duration_s"),
        "drone_ids": [d["id"] for d in card.card["drones"]], "states": states,
    }
    Path(spec["out"]).write_text(json.dumps(body), encoding="utf-8")
    print(f"rollout r{spec['round']} {spec['scene']} z={spec['altitude_m']} seed={spec['seed']} {body['outcome']} "
          f"{body['arrived']}/{body['n_drones']}", flush=True)
    return {k: v for k, v in body.items() if k != "states"}


def rollouts(out: Path, k: int, weights: Path, jobs: int) -> list[dict]:
    rdir = out / f"r{k}" / "rollouts"
    rdir.mkdir(parents=True, exist_ok=True)
    specs = []
    for scene, seeds in TRAIN_SEEDS[k].items():
        for alt in ALTITUDES:
            for seed in seeds:
                path = rdir / f"{scene}-z{alt:.1f}-s{seed}.json"
                if path.is_file():
                    continue
                specs.append({"scene": scene, "card": str(SCENES[scene]), "altitude_m": float(alt), "seed": int(seed),
                              "round": k, "weights": str(weights), "out": str(path)})
    _map(rollout_one, specs, jobs)
    return [{k2: v for k2, v in json.loads(p.read_text()).items() if k2 != "states"} for p in sorted(rdir.glob("*.json"))]


def _state_ok(scene: str, s: dict) -> bool:
    p = np.asarray(s["p"], float)
    if not (0.4 <= p[2] <= 3.4) or abs(p[0]) > 11.0 or abs(p[1]) > 11.0:
        return False
    if s["min_obstacle_m"] < L0_BODY_M + 0.10:
        return False
    if scene == "g2-pillar" and (abs(p[1]) > 3.0 or not (-2.8 <= p[0] <= 9.8)):
        return False
    if np.linalg.norm(np.asarray(s["goal"], float) - p) < GOAL_SKIP_M:
        return False
    return True


def make_jobs(out: Path, k: int) -> list[dict]:
    jobs = []
    for f in sorted((out / f"r{k}" / "rollouts").glob("*.json")):
        r = json.loads(f.read_text())
        ids = r["drone_ids"]
        by_t: dict[float, dict] = {}
        for s in r["states"]:
            by_t.setdefault(round(s["t"], 2), {})[s["id"]] = s
        picked = 0
        for t in sorted(by_t):
            q = (t - T0_S) / EVERY_S
            if t < T0_S - 1e-6 or abs(q - round(q)) > 1e-3:
                continue
            row = by_t[t]
            if set(row) != set(ids) or not all(_state_ok(r["scene"], row[i]) for i in ids):
                continue
            jobs.append({
                "id": f"r{k}-{f.stem}-t{t:.1f}", "round": k, "scene": r["scene"], "altitude_m": r["altitude_m"],
                "seed": r["seed"], "t": t,
                "drones": [{"ros_id": n, "id": i, **{key: row[i][key] for key in ("p", "v", "a", "yaw", "goal")}}
                           for n, i in enumerate(ids)],
            })
            picked += 1
            if picked >= CAP_PER_ROLLOUT:
                break
    path = out / f"r{k}" / "jobs.jsonl"
    path.write_text("".join(json.dumps(j) + "\n" for j in jobs), encoding="utf-8")
    return jobs


# ---------- 教师重标注 ----------

def _docker(*args, check=True, capture=False):
    return subprocess.run(["sudo", "-n", "docker", *args], check=check, capture_output=capture, text=True)


def ensure_containers(work: Path) -> list[str]:
    work.mkdir(parents=True, exist_ok=True)
    helper = work / "kill_ros.py"
    if not helper.is_file():
        shutil.copy(ROOT / "scripts" / "distill" / "d7" / "kill_ros.py", helper)
    if not (work / "linebuf.so").is_file():
        raise SystemExit(f"缺少 {work}/linebuf.so：先跑 scripts/teacher/d2b.sh build --work {work}")
    names = []
    for i in range(N_CONTAINERS):
        name = f"ego-teacher-d7-{i}"
        if _docker("inspect", name, check=False, capture=True).returncode != 0:
            _docker("run", "-d", "--init", "--name", name,
                    "-v", f"{EGO_DIR}:/ego", "-v", f"{RECORDER_DIR}:/ws:ro", "-v", f"{work}:/teacher-runs",
                    "-w", "/ego", "--memory", "2200m", "--cpus", "2", "--pids-limit", "8192",
                    IMAGE, "sleep", "infinity")
        else:
            _docker("start", name, check=False)
        names.append(name)
    return names


def teacher_scenario(job: dict) -> dict:
    cylinders = G2_TEACHER_CYLINDERS if job["scene"] == "g2-pillar" else []
    drones = []
    for d in job["drones"]:
        drones.append({
            "ros_id": int(d["ros_id"]), "id": d["id"],
            "start_enu": [float(x) for x in d["p"]], "goal_enu": [float(x) for x in d["goal"]],
            "seed_state": {key: d[key] for key in ("p", "v", "a", "yaw")},
        })
    return {
        "id": job["id"], "density_per_m2": 0.0, "n_drones": len(drones), "pattern": job["scene"], "seed": job["seed"],
        "map_side_m": 20.0, "obstacle_radius_m": 0.2, "body_radius_m": 0.2, "flight_z_m": job["altitude_m"],
        "drones": drones, "cylinders": cylinders, "n_obstacles": len(cylinders),
        "record_after_handover_s": AFTER_S,
        "dagger": {"round": job["round"], "scene": job["scene"], "t_student_s": job["t"], "seed": job["seed"]},
        "assumption": "g2 柱子在教师侧是外接圆柱近似，墙不发给教师。这是假设，不是论文复现。",
    }


RUN_SH = """#!/bin/bash
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


def relabel_one(job: dict, container: str, work: Path) -> str:
    from teacher.launch_xml import launch_xml

    rd = work / "runs" / job["id"]
    if (rd / "record.json").is_file():
        return "skip"
    rd.mkdir(parents=True, exist_ok=True)
    scn = teacher_scenario(job)
    (rd / "scenario.json").write_text(json.dumps(scn))
    (rd / "teacher.launch").write_text(launch_xml(scn))
    remote = f"/teacher-runs/runs/{job['id']}"
    (rd / "run.sh").write_text(RUN_SH.format(remote=remote))
    (rd / "run.sh").chmod(0o755)
    t0 = time.time()
    subprocess.run(["sudo", "-n", "docker", "exec", container, "bash", f"{remote}/run.sh"], timeout=120)
    subprocess.run(["sudo", "-n", "chmod", "-R", "a+rwX", str(rd)], check=False)
    rec = {"schema": "yopo-teacher-record/1", "scenario": {"id": job["id"], "n_drones": len(scn["drones"])},
           "dagger": scn["dagger"], "wall_s": time.time() - t0}
    raw_p = rd / "raw_record.json"
    if not raw_p.is_file():
        rec["metrics"] = {"success": None, "why": "no_raw"}
        (rd / "record.json").write_text(json.dumps(rec))
        return "noraw"
    raw = json.loads(raw_p.read_text())
    T = np.asarray(raw["time_s"], float)
    hand = raw.get("handover_ros_s") or {}
    if len(hand) < len(scn["drones"]) or T.size == 0 or raw.get("t0_ros_s") is None:
        rec["metrics"] = {"success": None, "why": f"no_handover status={raw.get('run_status')} n_hand={len(hand)}"}
        (rd / "record.json").write_text(json.dumps(rec))
        return "nohand"
    t_h = max(hand.values()) - float(raw["t0_ros_s"])
    keep = T >= t_h - 1e-6
    T = T[keep] - T[keep][0]
    P = np.asarray(raw["position_m"], float)[keep]
    V = np.asarray(raw["velocity_mps"], float)[keep]
    A = np.asarray(raw["acceleration_mps2"], float)[keep]
    Y = np.unwrap(np.asarray(raw["yaw_rad"], float)[keep], axis=0)
    G = np.asarray([d["goal_enu"] for d in scn["drones"]], float)
    np.savez_compressed(rd / "trajectory.npz", time_s=T, position_m=P, velocity_mps=V, acceleration_mps2=A,
                        yaw_rad=Y, goal_m=G)
    p0err = float(max(np.linalg.norm(P[0, n] - np.asarray(d["p"])) for n, d in enumerate(job["drones"])))
    v0err = float(max(np.linalg.norm(V[0, n] - np.asarray(d["v"])) for n, d in enumerate(job["drones"])))
    ok = bool(T.size and T[-1] >= HORIZON_S + 0.2)
    rec["metrics"] = {"success": ok, "span_s": float(T[-1]) if T.size else 0.0, "handover_s": t_h,
                      "p0_err_m": p0err, "v0_err_mps": v0err, "status": raw.get("run_status")}
    (rd / "record.json").write_text(json.dumps(rec))
    # L0 场景卡：同一张卡、同一高度，起点换成学生状态。深度按 L0 障碍现算。
    card = json.loads(Path(SCENES[job["scene"]]).read_text())
    geom = card["geometry"]
    shutil.copy(Path(SCENES[job["scene"]]).parent / geom, rd / geom)
    card["name"] = job["id"]
    card["drones"] = [{**cd, "start_enu": [float(x) for x in d["p"]], "goal_enu": [float(x) for x in d["goal"]]}
                      for cd, d in zip(card["drones"], job["drones"])]
    (rd / "scene.json").write_text(json.dumps(card))
    return "ok"


def _relabel_worker(args) -> list[str]:
    jobs, container, work = args
    sys.path.insert(0, str(ROOT / "scripts"))
    out = []
    for job in jobs:
        try:
            r = relabel_one(job, container, Path(work))
        except Exception as exc:  # 单场失败不拖垮整批
            r = f"err {exc!r}"
        print(time.strftime("%H:%M:%S"), container, job["id"], r, flush=True)
        out.append(r)
    return out


def relabel(out: Path, k: int) -> dict:
    from concurrent.futures import ThreadPoolExecutor

    work = out / "relabel"
    names = ensure_containers(work)
    jobs = [json.loads(line) for line in (out / f"r{k}" / "jobs.jsonl").read_text().splitlines() if line.strip()]
    chunks = [(jobs[i::len(names)], names[i], str(work)) for i in range(len(names))]
    with ThreadPoolExecutor(max_workers=len(names)) as pool:
        results = [r for part in pool.map(_relabel_worker, chunks) for r in part]
    counts: dict[str, int] = {}
    for r in results:
        key = r if r in ("ok", "skip", "noraw", "nohand") else "err"
        counts[key] = counts.get(key, 0) + 1
    return {"n_jobs": len(jobs), "counts": counts}


# ---------- 转窗口 ----------

def _filter(rd: Path, batch) -> np.ndarray:
    from yopo_l0.scene import load_card

    card = load_card(rd / "scene.json")
    with np.load(rd / "trajectory.npz") as tr:
        T = np.asarray(tr["time_s"]); P = np.asarray(tr["position_m"])
    clear = np.array([[card.min_distance(P[i, d]) for d in range(P.shape[1])] for i in range(P.shape[0])])
    keep = np.zeros(batch.n, bool)
    for w in range(batch.n):
        t = float(batch.time_s[w]); d = int(batch.ego_index[w])
        m = (T >= t - 1e-6) & (T <= t + HORIZON_S + 1e-6)
        ok = bool(np.all(clear[m, d] >= L0_BODY_M))
        for j in range(P.shape[1]):
            if j != d and np.any(np.linalg.norm(P[m, d] - P[m, j], axis=1) < L0_INTER_M):
                ok = False
        keep[w] = ok
    return keep


def convert_round(out: Path, k: int) -> dict:
    from yopo_l0.distill import WindowBatch, convert_run, save_batch

    wdir = out / f"r{k}" / "windows"
    wdir.mkdir(parents=True, exist_ok=True)
    files = {}
    stats = {"runs": 0, "usable_runs": 0, "windows_all": 0, "windows_kept": 0, "by_scene": {}}
    for rd in sorted((out / "relabel" / "runs").glob(f"r{k}-*")):
        rec_p = rd / "record.json"
        if not rec_p.is_file():
            continue
        rec = json.loads(rec_p.read_text())
        scene = rec["dagger"]["scene"]
        sc = stats["by_scene"].setdefault(scene, {"runs": 0, "usable_runs": 0, "windows_all": 0, "windows_kept": 0,
                                                  "p0_err_m": [], "v0_err_mps": []})
        stats["runs"] += 1; sc["runs"] += 1
        met = rec.get("metrics") or {}
        if met.get("success") is not True:
            continue
        sc["p0_err_m"].append(met["p0_err_m"]); sc["v0_err_mps"].append(met["v0_err_mps"])
        batch = convert_run(rd)
        if batch.n == 0:
            continue
        keep = _filter(rd, batch)
        stats["usable_runs"] += 1; sc["usable_runs"] += 1
        stats["windows_all"] += int(batch.n); sc["windows_all"] += int(batch.n)
        stats["windows_kept"] += int(keep.sum()); sc["windows_kept"] += int(keep.sum())
        if not keep.any():
            continue
        fb = WindowBatch(**{f: (getattr(batch, f)[keep] if isinstance(getattr(batch, f), np.ndarray) else getattr(batch, f))
                            for f in batch.__dataclass_fields__})
        dest = wdir / f"{rd.name}.npz"
        save_batch(fb, dest)
        files[rd.name] = {"path": str(dest), "sha256": _sha(dest), "n": int(keep.sum()), "scene": scene}
    for sc in stats["by_scene"].values():
        for key in ("p0_err_m", "v0_err_mps"):
            vals = sc.pop(key)
            sc[f"{key}_median"] = float(np.median(vals)) if vals else None
    man = {"round": k, "filter": "未来 1.6 s：L0 障碍净空 ≥ 0.35 m，机间 ≥ 0.70 m；重标注跨度 ≥ 1.8 s",
           "stats": stats, "n_files": len(files), "n_windows": sum(v["n"] for v in files.values()), "files": files}
    _write_json(wdir / "manifest.json", man)
    return man


# ---------- 评测 ----------

def eval_worker(spec: dict) -> dict:
    """子进程入口。进程内加载 D6d 的单场评测，避免动态模块不能 pickle。"""
    return _d6d().eval_one(spec)


def evaluate(out: Path, tag: str, weights: Path, jobs: int) -> dict:
    d6d = _d6d()
    ev = _eval()
    specs = []
    for alt in ALTITUDES:
        for name, card, label in ev.SCENES:
            if name not in EVAL_SEEDS:
                continue
            for seed in EVAL_SEEDS[name]:
                specs.append({"arm": tag, "scene": name, "label": label, "card": str(card), "altitude_m": float(alt),
                              "seed": int(seed), "weights": str(weights), "deviates_upstream": True})
    rows = _map(eval_worker, specs, jobs)
    rows.sort(key=lambda r: (r["scene"], float(r["altitude_m"]), r["seed"]))
    groups = d6d.aggregate(rows)
    body = {"tag": tag, "weights": str(weights), "weights_sha256": _sha(weights), "rows": rows, "groups": groups}
    _write_json(out / f"eval-{tag}.json", body)
    return body


def open_arrived(groups: list[dict]) -> int:
    return sum(int(g["arrived"]) for g in groups if g["scene"] == "open-3drone")


# ---------- 编排 ----------

def _marker(out: Path, name: str) -> Path:
    return out / "markers" / name


def step(out: Path, name: str, fn):
    m = _marker(out, name)
    if m.is_file():
        print(f"skip {name}", flush=True)
        return json.loads(m.read_text())
    print(f"start {name} {time.strftime('%F %T')}", flush=True)
    res = fn()
    m.parent.mkdir(parents=True, exist_ok=True)
    _write_json(m, res if isinstance(res, (dict, list)) else {"result": res})
    print(f"done {name} {time.strftime('%F %T')}", flush=True)
    return json.loads(m.read_text())


def run_all(out: Path, jobs: int, rounds: int) -> int:
    from yopo_l0.policy import StudentPolicy

    defaults = dict(StudentPolicy.DEFAULTS)
    out.mkdir(parents=True, exist_ok=True)
    step(out, "base", lambda: build_base(out, jobs))

    def train_r0():
        from yopo_l0.student_fit import FitConfig, train_from_paths

        base = json.loads((out / "base" / "base.json").read_text())
        pipe = _pipe()
        with pipe._TupleJson():
            s = train_from_paths([Path(p) for p in base["train_paths"]], [Path(p) for p in base["hold_paths"]],
                                 out / "r0" / "train", cfg=FitConfig(**dict(pipe.R4_FIT)))
        s["round"] = 0; s["n_dagger_windows"] = 0; s["n_base_train_windows"] = base["n_train_windows"]
        _write_json(out / "r0" / "train" / "d7-train.json", s)
        return {"weights_sha256": s.get("weights_sha256"), "holdout": s.get("holdout")}

    step(out, "train-r0", train_r0)
    ev0 = step(out, "eval-r0", lambda: {"groups": evaluate(out, "r0", out / "r0" / "train" / "student.pt", jobs)["groups"]})
    last = ev0
    for k in range(1, rounds + 1):
        prev_w = out / f"r{k-1}" / "train" / "student.pt"
        step(out, f"rollouts-r{k}", lambda: {"rollouts": rollouts(out, k, prev_w, jobs)})
        step(out, f"jobs-r{k}", lambda: {"n_jobs": len(make_jobs(out, k))})
        step(out, f"relabel-r{k}", lambda: relabel(out, k))
        step(out, f"convert-r{k}", lambda: {key: v for key, v in convert_round(out, k).items() if key != "files"})
        step(out, f"train-r{k}", lambda: {key: v for key, v in train_round(out, k).items()
                                          if key in ("weights_sha256", "holdout", "n_dagger_windows", "n_base_train_windows", "wall_s")})
        last = step(out, f"eval-r{k}", lambda: {"groups": evaluate(out, f"r{k}", out / f"r{k}" / "train" / "student.pt", jobs)["groups"]})
        print(f"r{k} 开阔到达 {open_arrived(last['groups'])}/60", flush=True)
    if dict(StudentPolicy.DEFAULTS) != defaults:
        raise SystemExit("StudentPolicy.DEFAULTS 被改了")
    print("ALL-DONE", flush=True)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="D7：学生 DAgger 在策略重标注（只报数）")
    ap.add_argument("mode", choices=("all",))
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--rounds", type=int, default=3)
    a = ap.parse_args(argv)
    return run_all(a.out, a.jobs, a.rounds)


if __name__ == "__main__":
    raise SystemExit(main())
