#!/usr/bin/env python3
"""D8：DAgger 方差与 g2 退化归因（只报数，偏离上游）。

只读 D7 的产物（/workspace/yopo-teacher-data/d7），不改官方 d6b student.pt。
每个阶段一个完成标记 <out>/markers/<name>，结果写 <out>/results/<name>.json；中断后重跑同一命令即可续跑。

阶段：
- train-<arm>-s<seed>：r0 / r2 / r3 用 D7 同一份数据换训练种子重训；g2w 是 r3 数据 + g2 重标注窗加权；
  exact 是 r3 数据里的 g2 重标注窗换成「教师侧精确几何」重标注的版本。
- eval-<arm>-s<seed>[-cap]：口径同 D7（开阔 0–9、g2 0–4，1.0 / 1.5 m）。-cap 是同一权重打开结点水平速度上限。
- select-<arm>-s<seed>：候选在训练种子（开阔 10–18、g2 5–10）上闭环，用来挑检查点，不看评估种子。
- probe-arrival：D7 r2 学生（s0）开阔场的到达判定单查，逐架记速度曲线。
- relabel-exact-r<k> / convert-exact-r<k>：D7 各轮 g2 作业原样重跑教师，柱子发方柱，墙也发。
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
D7_OUT = Path("/workspace/yopo-teacher-data/d7")
OFFICIAL = Path("/workspace/yopo-teacher-data/d6b/train/student.pt")
OFFICIAL_SHA = "e4bd211662b478066f3611ac8ca6e9cbc6b13445dfdaef82440c04bc53e1f3f0"
TRAIN_FIT_SEEDS = (0, 1, 2)
SELECT_SEEDS = {"open-3drone": list(range(10, 19)), "g2-pillar": list(range(5, 11))}
EVAL_SEEDS = {"open-3drone": list(range(10)), "g2-pillar": list(range(5))}
ALTITUDES = (1.0, 1.5)
SPEED_CAP_MPS = 2.0  # 教师 EGO-Swarm max_vel（scripts/teacher/assumptions.py MAX_VEL_MPS）
G2_WEIGHT = 4.0
EARLY_S = 2.0
CONTAINER_PREFIX = "ego-teacher-d8"
DISCLAIMER = ("D8 只报数，偏离上游：训练种子、场景加权、结点速度上限、教师侧精确几何都是本票的对照臂，"
              "库默认（StudentPolicy.DEFAULTS、knot_speed_cap_mps 缺省为关）不变。")


def _load(name: str, path: Path):
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def d7():
    return _load("d8_d7_dagger", ROOT / "scripts" / "distill" / "d7" / "dagger.py")


def d6d():
    return d7()._d6d()


def _write_json(path: Path, payload) -> None:
    d7()._write_json(path, payload)


def _sha(path: Path) -> str:
    return d7()._sha(path)


# ---------- 数据集 ----------

def _d7_files(k: int, out: Path = D7_OUT) -> dict:
    return json.loads((out / f"r{k}" / "windows" / "manifest.json").read_text())["files"]


def dataset(arm: str, out: Path) -> tuple[list[Path], list[Path], dict, dict]:
    """返回 (训练路径, 留出路径, path_weights, 说明)。"""
    base = json.loads((D7_OUT / "base" / "base.json").read_text())
    train = [Path(p) for p in base["train_paths"]]
    hold = [Path(p) for p in base["hold_paths"]]
    weights: dict = {}
    rounds = {"r0": 0, "r2": 2, "r3": 3, "g2w": 3, "exact": 3}[arm]
    n = {"open-3drone": 0, "g2-pillar": 0}
    for k in range(1, rounds + 1):
        files = _d7_files(k)
        if arm == "exact":
            files = {key: v for key, v in files.items() if v["scene"] != "g2-pillar"}
            ex = out / "exact" / f"r{k}" / "windows" / "manifest.json"
            if not ex.is_file():
                raise SystemExit(f"缺少精确几何重标注窗：{ex}")
            files.update(json.loads(ex.read_text())["files"])
        for v in files.values():
            train.append(Path(v["path"]))
            n[v["scene"]] += int(v["n"])
            if arm == "g2w" and v["scene"] == "g2-pillar":
                weights[str(Path(v["path"]))] = G2_WEIGHT
    info = {"arm": arm, "rounds": rounds, "n_base_train_windows": int(base["n_train_windows"]),
            "n_dagger_windows_by_scene": n, "g2_weight": G2_WEIGHT if arm == "g2w" else 1.0}
    return train, hold, weights, info


def train_arm(out: Path, arm: str, seed: int) -> dict:
    from yopo_l0.student_fit import FitConfig, train_from_paths

    train, hold, weights, info = dataset(arm, out)
    pipe = d7()._pipe()
    fit = dict(pipe.R4_FIT)
    fit["seed"] = int(seed)
    tdir = out / "train" / f"{arm}-s{seed}"
    if (tdir / "student.pt").exists():
        raise SystemExit(f"{tdir} 已有权重但没有完成标记，先手工检查")
    with pipe._TupleJson():
        s = train_from_paths(train, hold, tdir, cfg=FitConfig(**fit), path_weights=weights or None)
    return {"arm": arm, "seed": seed, "weights": str(tdir / "student.pt"), "weights_sha256": s.get("weights_sha256"),
            "holdout": s.get("holdout"), "wall_s": s.get("wall_s"), "fit_used": fit, **info}


# ---------- 评估 ----------

def eval_one(spec: dict) -> dict:
    """单场闭环，口径同 D6d / D7 的 eval_one（影子盾开、track_clearance 关）；可选注入学生参数。"""
    os.environ["OMP_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    import torch

    torch.set_num_threads(1)
    from yopo_l0.runner import RunConfig, run
    from yopo_l0.scene import load_card

    cmp = d6d()
    ev = cmp._eval()
    card = ev.retarget_card(load_card(spec["card"]), spec["altitude_m"], "off", spec["weights"])
    if spec.get("student_extra"):
        card.card["policy_params"]["student"].update(spec["student_extra"])
    started = time.perf_counter()
    output = run(card, RunConfig(seed=int(spec["seed"]), policy="student", shadow=True, duration_s=None))
    result = output.result
    kind = ev._classify(result)
    distance = result.get("min_obstacle_distance_m")
    hover = cmp._hover(output.ticks)
    early = [float(np.linalg.norm(r["v"])) for r in output.ticks if float(r["t"]) <= EARLY_S + 1e-9]
    timing = drone_timing(card, output.ticks)
    row = {
        "arm": spec["arm"], "scene": spec["scene"], "label": spec["label"], "altitude_m": float(spec["altitude_m"]),
        "seed": int(spec["seed"]), "outcome": result.get("outcome"), "reason": result.get("reason"), "kind": kind,
        "n_drones": int(result.get("n_drones") or 0), "arrived": int(result.get("arrived") or 0),
        "obstacle": int(kind == "obstacle"), "inter_drone": int(kind == "inter_drone"), "timeout": int(kind == "timeout"),
        "min_clearance_m": None if distance is None else float(distance) - ev.BODY_RADIUS_M,
        "final_goal_error_m": result.get("final_goal_error_m"), "sim_duration_s": result.get("sim_duration_s"),
        "hover_frac": hover["hover_frac"], "n_never_left_hover": hover["n_never_left_hover"],
        "max_speed_mps": hover["max_speed_mps"], "early_max_speed_mps": max(early) if early else None,
        "worker_s": time.perf_counter() - started, "report_only": True, "drone_timing": timing,
    }
    print(f"{row['arm']} {row['scene']} z={row['altitude_m']} seed={row['seed']} {row['outcome']} "
          f"{row['arrived']}/{row['n_drones']} vmax2s={row['early_max_speed_mps']}", flush=True)
    return row


ARRIVE_RADIUS_M = 1.5  # StudentPolicy 的 arrive_radius_m（库默认，r4 评估不改）


def drone_timing(card, ticks) -> list[dict]:
    """D9：逐架首次进入 1.5 m 到达半径、首次进入判据容差的时刻，以及按判据的最长连续保持。只报数。"""
    crit = card.card["criteria"]
    gtol, gspeed = float(crit["goal_tolerance_m"]), float(crit["goal_speed_mps"])
    out = []
    for d in card.card["drones"]:
        goal = np.asarray(d["goal_enu"], float)
        t_r = t_tol = None
        best = 0.0
        start = None
        for r in ticks:
            if r["uav"] != d["id"]:
                continue
            t = float(r["t"])
            err = float(np.linalg.norm(np.asarray(r["p"], float) - goal))
            if t_r is None and err <= ARRIVE_RADIUS_M:
                t_r = t
            if t_tol is None and err <= gtol:
                t_tol = t
            if err <= gtol and float(np.linalg.norm(r["v"])) <= gspeed:
                start = t if start is None else start
                best = max(best, t - start)
            else:
                start = None
        out.append({"id": d["id"], "t_arrive_radius_s": t_r, "t_goal_tol_s": t_tol, "longest_hold_s": best})
    return out


def evaluate(arm_tag: str, weights: Path, seeds: dict, jobs: int, extra: dict | None = None) -> dict:
    ev = d6d()._eval()
    specs = []
    for alt in ALTITUDES:
        for name, card, label in ev.SCENES:
            for seed in seeds.get(name, []):
                specs.append({"arm": arm_tag, "scene": name, "label": label, "card": str(card), "altitude_m": float(alt),
                              "seed": int(seed), "weights": str(weights), "student_extra": extra or {}})
    rows = d6d()._map(eval_one, specs, jobs)
    rows.sort(key=lambda r: (r["scene"], float(r["altitude_m"]), r["seed"]))
    groups = d6d().aggregate(rows)
    for g in groups:
        sub = [r for r in rows if r["scene"] == g["scene"] and float(r["altitude_m"]) == float(g["altitude_m"])]
        e = [r["early_max_speed_mps"] for r in sub if r["early_max_speed_mps"] is not None]
        g["early_max_speed_median_mps"] = float(np.median(e)) if e else None
        c = [r["sim_duration_s"] for r in sub if r["obstacle"]]
        g["collision_t_median_s"] = float(np.median(c)) if c else None
    return {"arm": arm_tag, "weights": str(weights), "weights_sha256": _sha(weights), "student_extra": extra or {},
            "seeds": seeds, "rows": rows, "groups": groups}


def score(body: dict) -> tuple:
    """挑检查点的分数：到达架数多者优先，其次碰撞少，其次终点误差中位数小。"""
    rows = body["rows"]
    arrived = sum(int(r["arrived"]) for r in rows)
    coll = sum(int(r["obstacle"]) + int(r["inter_drone"]) for r in rows)
    errs = [float(r["final_goal_error_m"]) for r in rows if r.get("final_goal_error_m") is not None]
    return (arrived, -coll, -float(np.median(errs)) if errs else 0.0)


# ---------- 到达判定单查 ----------

def probe_arrival(out: Path, weights: Path, seeds=range(10)) -> dict:
    from yopo_l0.runner import RunConfig, run
    from yopo_l0.scene import load_card

    ev = d6d()._eval()
    card0 = load_card(str(d7().SCENES["open-3drone"]))
    crit = card0.card["criteria"]
    gtol, gspeed, ghold = float(crit["goal_tolerance_m"]), float(crit["goal_speed_mps"]), float(crit["goal_hold_s"])
    dur = float(card0.card["task"]["duration_s"])
    cases = []
    for alt in ALTITUDES:
        for seed in seeds:
            card = ev.retarget_card(load_card(str(d7().SCENES["open-3drone"])), alt, "off", str(weights))
            output = run(card, RunConfig(seed=int(seed), policy="student", shadow=True, duration_s=None))
            goals = {d["id"]: np.asarray(d["goal_enu"], float) for d in card.card["drones"]}
            drones = []
            for did, goal in goals.items():
                rows = [r for r in output.ticks if r["uav"] == did]
                t = np.array([float(r["t"]) for r in rows])
                sp = np.array([float(np.linalg.norm(r["v"])) for r in rows])
                er = np.array([float(np.linalg.norm(np.asarray(r["p"], float) - goal)) for r in rows])
                ok = (er <= gtol) & (sp <= gspeed)
                best, run_start, best_start = 0.0, None, None
                for i in range(len(t)):
                    if ok[i]:
                        run_start = t[i] if run_start is None else run_start
                        if t[i] - run_start > best:
                            best, best_start = t[i] - run_start, run_start
                    else:
                        run_start = None
                inz = np.where(er <= gtol)[0]
                t_in = float(t[inz[0]]) if inz.size else None
                after = slice(int(inz[0]), None) if inz.size else slice(0, 0)
                exits = int(np.sum(np.diff(ok[after].astype(int)) == -1)) if inz.size else 0
                spd_after = sp[after]
                k10 = [i for i in range(len(t)) if abs(t[i] * 10 - round(t[i] * 10)) < 1e-6]
                drones.append({
                    "id": did, "arrived_by_rule": bool(best >= ghold - 1e-9),
                    "t_enter_tol_s": t_in, "first_ok_s": float(t[ok][0]) if ok.any() else None,
                    "longest_hold_s": float(best), "longest_hold_start_s": best_start,
                    "ok_breaks_after_entry": exits,
                    "speed_at_entry_mps": float(sp[inz[0]]) if inz.size else None,
                    "speed_max_after_entry_mps": float(spd_after.max()) if spd_after.size else None,
                    "speed_min_after_entry_mps": float(spd_after.min()) if spd_after.size else None,
                    "final_speed_mps": float(sp[-1]), "final_err_m": float(er[-1]),
                    "err_min_m": float(er.min()),
                    "curve_10hz": [{"t": round(float(t[i]), 2), "speed_mps": round(float(sp[i]), 4),
                                    "err_m": round(float(er[i]), 4)} for i in k10],
                })
            cases.append({"altitude_m": alt, "seed": int(seed), "outcome": output.result.get("outcome"),
                          "arrived": output.result.get("arrived"), "drones": drones})
            print("probe", alt, seed, output.result.get("outcome"),
                  [(d["id"], d["arrived_by_rule"], d["t_enter_tol_s"], round(d["longest_hold_s"], 2)) for d in drones],
                  flush=True)
    return {"weights": str(weights), "weights_sha256": _sha(weights),
            "rule": {"goal_tolerance_m": gtol, "goal_speed_mps": gspeed, "goal_hold_s": ghold, "duration_s": dur,
                     "note": "runner：每个物理步（100 Hz）距目标 ≤ 容差且三维速度 ≤ 上限，连续保持 goal_hold 才算到达"},
            "cases": cases}


# ---------- 教师侧精确几何 ----------

def g2_boxes() -> list[dict]:
    card = json.loads(Path(d7().SCENES["g2-pillar"]).read_text())
    geom = json.loads((Path(d7().SCENES["g2-pillar"]).parent / card["geometry"]).read_text())
    return [{"name": b["name"], "lo": [float(x) for x in b["lo"]], "hi": [float(x) for x in b["hi"]]} for b in geom["boxes"]]


def exact_scenario(job: dict) -> dict:
    scn = d7().teacher_scenario(job)
    if job["scene"] != "g2-pillar":
        return scn
    scn["cylinders"] = []
    scn["boxes"] = g2_boxes()
    scn["n_obstacles"] = len(scn["boxes"])
    scn["assumption"] = "D8 对照①：g2 在教师侧按场景卡原样发方柱和墙（实心盒点云，0.1 m 栅格）。"
    return scn


def relabel_exact(out: Path, k: int) -> dict:
    import shutil
    from concurrent.futures import ThreadPoolExecutor

    m = d7()
    xo = out / "exact"
    work = xo / "relabel"
    work.mkdir(parents=True, exist_ok=True)
    if not (work / "linebuf.so").is_file():
        shutil.copy(D7_OUT / "relabel" / "linebuf.so", work / "linebuf.so")
    names = m.ensure_containers(work, prefix=CONTAINER_PREFIX)
    jobs = [json.loads(x) for x in (D7_OUT / f"r{k}" / "jobs.jsonl").read_text().splitlines() if x.strip()]
    jobs = [j for j in jobs if j["scene"] == "g2-pillar"]
    chunks = [(jobs[i::len(names)], names[i], str(work), exact_scenario) for i in range(len(names))]
    with ThreadPoolExecutor(max_workers=len(names)) as pool:
        res = [r for part in pool.map(m._relabel_worker, chunks) for r in part]
    counts: dict = {}
    for r in res:
        key = r if r in ("ok", "skip", "noraw", "nohand") else "err"
        counts[key] = counts.get(key, 0) + 1
    return {"round": k, "n_jobs": len(jobs), "counts": counts}


def convert_exact(out: Path, k: int) -> dict:
    man = d7().convert_round(out / "exact", k)
    return {key: v for key, v in man.items() if key != "files"}


def teacher_diff(out: Path) -> dict:
    """同一 g2 作业，圆柱近似与精确几何两版教师轨迹的差别（重标注后 1.6 s 内）。"""
    rows = []
    for k in (1, 2, 3):
        for rd in sorted((out / "exact" / "relabel" / "runs").glob(f"r{k}-*")):
            a = D7_OUT / "relabel" / "runs" / rd.name / "trajectory.npz"
            b = rd / "trajectory.npz"
            if not (a.is_file() and b.is_file()):
                continue
            with np.load(a) as A, np.load(b) as B:
                ta, pa = np.asarray(A["time_s"]), np.asarray(A["position_m"])
                tb, pb = np.asarray(B["time_s"]), np.asarray(B["position_m"])
            n = min(int(np.searchsorted(ta, 1.6)), int(np.searchsorted(tb, 1.6)), len(ta), len(tb))
            if n < 2:
                continue
            d = np.linalg.norm(pa[:n, 0, :2] - pb[:n, 0, :2], axis=1)
            rows.append({"run": rd.name, "max_xy_diff_m": float(d.max()), "end_xy_diff_m": float(d[-1]),
                         "min_y_abs_circle_m": float(np.abs(pa[:n, 0, 1]).max()),
                         "min_y_abs_exact_m": float(np.abs(pb[:n, 0, 1]).max())})
    md = [r["max_xy_diff_m"] for r in rows]
    return {"n_pairs": len(rows), "max_xy_diff_median_m": float(np.median(md)) if md else None,
            "max_xy_diff_p90_m": float(np.percentile(md, 90)) if md else None,
            "max_xy_diff_max_m": float(np.max(md)) if md else None, "rows": rows}


# ---------- 编排 ----------

def step(out: Path, name: str, fn):
    m = out / "markers" / name
    if m.is_file():
        print(f"skip {name}", flush=True)
        return json.loads((out / "results" / f"{name}.json").read_text())
    print(f"start {name} {time.strftime('%F %T')}", flush=True)
    res = fn()
    _write_json(out / "results" / f"{name}.json", res)
    m.parent.mkdir(parents=True, exist_ok=True)
    m.write_text(time.strftime("%F %T") + "\n")
    print(f"done {name} {time.strftime('%F %T')}", flush=True)
    return res


def _check_official() -> None:
    if _sha(OFFICIAL) != OFFICIAL_SHA:
        raise SystemExit("官方 d6b student.pt 的 sha256 变了，停")


def run_all(out: Path, jobs: int, only: set[str] | None = None) -> int:
    from yopo_l0.policy import StudentPolicy

    defaults = dict(StudentPolicy.DEFAULTS)
    _check_official()
    out.mkdir(parents=True, exist_ok=True)
    want = (lambda s: only is None or s in only)
    w = lambda arm, s: out / "train" / f"{arm}-s{s}" / "student.pt"  # noqa: E731

    if want("variance"):
        for arm in ("r2", "r3", "r0"):
            for s in TRAIN_FIT_SEEDS:
                step(out, f"train-{arm}-s{s}", lambda: train_arm(out, arm, s))
                step(out, f"eval-{arm}-s{s}", lambda: evaluate(f"{arm}-s{s}", w(arm, s), EVAL_SEEDS, jobs))
    if want("probe"):
        step(out, "probe-arrival", lambda: probe_arrival(out, w("r2", 0)))
    if want("cap"):
        for s in TRAIN_FIT_SEEDS:
            step(out, f"eval-r3-s{s}-cap", lambda: evaluate(f"r3-s{s}-cap", w("r3", s), EVAL_SEEDS, jobs,
                                                             {"knot_speed_cap_mps": SPEED_CAP_MPS}))
    if want("g2w"):
        for s in TRAIN_FIT_SEEDS:
            step(out, f"train-g2w-s{s}", lambda: train_arm(out, "g2w", s))
            step(out, f"eval-g2w-s{s}", lambda: evaluate(f"g2w-s{s}", w("g2w", s), EVAL_SEEDS, jobs))
    if want("select"):
        for arm in ("r2", "r3"):
            for s in TRAIN_FIT_SEEDS:
                step(out, f"select-{arm}-s{s}", lambda: evaluate(f"sel-{arm}-s{s}", w(arm, s), SELECT_SEEDS, jobs))
    if want("exact"):
        for k in (1, 2, 3):
            step(out, f"relabel-exact-r{k}", lambda: relabel_exact(out, k))
            step(out, f"convert-exact-r{k}", lambda: convert_exact(out, k))
        step(out, "teacher-diff", lambda: teacher_diff(out))
        for s in TRAIN_FIT_SEEDS:
            step(out, f"train-exact-s{s}", lambda: train_arm(out, "exact", s))
            step(out, f"eval-exact-s{s}", lambda: evaluate(f"exact-s{s}", w("exact", s), EVAL_SEEDS, jobs))
    if dict(StudentPolicy.DEFAULTS) != defaults:
        raise SystemExit("StudentPolicy.DEFAULTS 被改了")
    _check_official()
    print("ALL-DONE", flush=True)
    return 0


CELLS = [("open-3drone", 1.0), ("open-3drone", 1.5), ("g2-pillar", 1.0), ("g2-pillar", 1.5)]
METRICS = ("arrived", "obstacle", "inter_drone", "timeout", "hover_frac_mean", "final_goal_error_median_m",
           "max_speed_median_mps", "early_max_speed_median_mps", "collision_t_median_s")


def _res(out: Path, name: str):
    f = out / "results" / f"{name}.json"
    return json.loads(f.read_text()) if (out / "markers" / name).is_file() and f.is_file() else None


def _cell(groups: list[dict], scene: str, alt: float) -> dict | None:
    for g in groups:
        if g["scene"] == scene and float(g["altitude_m"]) == alt:
            return g
    return None


def spread(evals: list[dict]) -> list[dict]:
    """多个训练种子的同一臂：每格每个指标的逐种子值、均值、极差。"""
    rows = []
    for scene, alt in CELLS:
        cells = [_cell(e["groups"], scene, alt) for e in evals]
        if any(c is None for c in cells):
            continue
        row = {"scene": scene, "altitude_m": alt, "denom": cells[0]["denom"], "seeds": len(cells)}
        for m in METRICS:
            vals = [c.get(m) for c in cells]
            if any(v is None for v in vals):
                row[m] = {"values": vals}
                continue
            vals = [float(v) for v in vals]
            row[m] = {"values": vals, "mean": float(np.mean(vals)), "min": min(vals), "max": max(vals),
                      "range": max(vals) - min(vals)}
        rows.append(row)
    return rows


def report(out: Path, dest: Path) -> dict:
    arms = {}
    for arm in ("r0", "r2", "r3", "r3-cap", "g2w", "exact"):
        tr_arm, cap = ("r3", True) if arm == "r3-cap" else (arm, False)
        evals, trains = [], []
        for s in TRAIN_FIT_SEEDS:
            e = _res(out, f"eval-{tr_arm}-s{s}" + ("-cap" if cap else ""))
            t = _res(out, f"train-{tr_arm}-s{s}")
            if e is None or t is None:
                continue
            evals.append(e)
            trains.append({k: t.get(k) for k in ("seed", "weights_sha256", "holdout", "n_dagger_windows_by_scene",
                                                 "g2_weight", "n_base_train_windows")})
        if evals:
            arms[arm] = {"seeds": [t["seed"] for t in trains], "train": trains,
                         "student_extra": evals[0]["student_extra"], "cells": spread(evals),
                         "per_seed_groups": [e["groups"] for e in evals]}
    select = {}
    for arm in ("r2", "r3"):
        cands = []
        for s in TRAIN_FIT_SEEDS:
            e = _res(out, f"select-{arm}-s{s}")
            if e is not None:
                cands.append({"seed": s, "score": list(score(e)), "groups": e["groups"]})
        if len(cands) == len(TRAIN_FIT_SEEDS):
            best = max(cands, key=lambda c: tuple(c["score"]))
            ev = _res(out, f"eval-{arm}-s{best['seed']}")
            select[arm] = {"candidates": cands, "picked_seed": best["seed"],
                           "picked_eval_groups": ev["groups"] if ev else None}
    probe = _res(out, "probe-arrival")
    if probe:
        for case in probe["cases"]:
            for d in case["drones"]:
                if d["arrived_by_rule"] or not (case["seed"] == 0):
                    d.pop("curve_10hz", None)
                else:
                    d["curve_10hz"] = [c for c in d["curve_10hz"] if c["t"] >= 4.0]
    body = {"title": "票 D8：DAgger 方差与 g2 退化归因（只报数）", "disclaimer": DISCLAIMER,
            "d7_data": str(D7_OUT), "fit_seeds": list(TRAIN_FIT_SEEDS), "eval_seeds": EVAL_SEEDS,
            "select_seeds": SELECT_SEEDS, "altitudes_m": list(ALTITUDES), "speed_cap_mps": SPEED_CAP_MPS,
            "g2_weight": G2_WEIGHT, "arms": arms, "select": select, "probe_arrival": probe,
            "exact_relabel": {f"r{k}": _res(out, f"relabel-exact-r{k}") for k in (1, 2, 3)},
            "exact_convert": {f"r{k}": _res(out, f"convert-exact-r{k}") for k in (1, 2, 3)},
            "teacher_diff": {k: v for k, v in (_res(out, "teacher-diff") or {}).items() if k != "rows"}}
    _write_json(dest, body)
    return body


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="D8：DAgger 方差与 g2 退化归因（只报数）")
    ap.add_argument("mode", choices=("all", "report"))
    ap.add_argument("--dest", type=Path, default=ROOT / "docs" / "research" / "d8-dagger-variance-runs.json")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--only", default="", help="逗号分隔：variance,probe,cap,g2w,select,exact；缺省全跑")
    a = ap.parse_args(argv)
    only = {x for x in a.only.split(",") if x} or None
    if a.mode == "report":
        report(a.out, a.dest)
        return 0
    return run_all(a.out, a.jobs, only)


if __name__ == "__main__":
    raise SystemExit(main())
