#!/usr/bin/env python3
"""D9：限速与 g2 加权 5 种子确认（只报数，偏离上游）。

数据是 D7 的 r3 数据（基础 + r1–r3 重标注窗），训练配方同 D8（D5b r4 FitConfig，只换 seed）。
- 权重：ctrl（r3 数据）与 g2w（g2 重标注窗 ×4），训练种子 0–4，各从头训。
- 评估：每份权重分别在 限速 关 / 1.5 / 2.0 / 2.5 m/s 下评估；限速只在评估时打开，同一份权重配对。
- 四个主臂：对照 = ctrl·关，限速 = ctrl·2.0，加权 = g2w·关，合并 = g2w·2.0。
每步一个完成标记 <out>/markers/<name>，结果在 <out>/results/<name>.json；中断后重跑同一命令即可续跑。
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
FIT_SEEDS = (0, 1, 2, 3, 4)
TRAIN_ARMS = {"ctrl": "r3", "g2w": "g2w"}  # D9 名 → D8 dataset 名
CAPS = (None, 1.5, 2.0, 2.5)
MAIN_ARMS = {"对照": ("ctrl", None), "限速 2.0": ("ctrl", 2.0), "加权 ×4": ("g2w", None), "限速 + 加权": ("g2w", 2.0)}
N_BOOT = 20000
DISCLAIMER = ("D9 只报数，偏离上游：限速（policy_params.student.knot_speed_cap_mps）与 g2 加权都只在本脚本里打开，"
              "库默认不变；判据不变。")


def _load(name: str, path: Path):
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def d8():
    return _load("d9_d8_variance", ROOT / "scripts" / "distill" / "d8" / "variance.py")


def cap_tag(cap) -> str:
    return "off" if cap is None else f"{cap:.1f}"


def eval_worker(spec: dict) -> dict:
    """子进程入口：主模块里的函数才能被 spawn 池 pickle。"""
    return d8().eval_one(spec)


def evaluate(tag: str, weights: Path, cap) -> dict:
    m = d8()
    ev = m.d6d()._eval()
    extra = {} if cap is None else {"knot_speed_cap_mps": float(cap)}
    specs = []
    for alt in m.ALTITUDES:
        for name, card, label in ev.SCENES:
            for seed in m.EVAL_SEEDS.get(name, []):
                specs.append({"arm": tag, "scene": name, "label": label, "card": str(card), "altitude_m": float(alt),
                              "seed": int(seed), "weights": str(weights), "student_extra": extra})
    rows = m.d6d()._map(eval_worker, specs, JOBS)
    rows.sort(key=lambda r: (r["scene"], float(r["altitude_m"]), r["seed"]))
    return {"arm": tag, "weights": str(weights), "weights_sha256": m._sha(weights), "student_extra": extra,
            "rows": rows, "groups": m.d6d().aggregate(rows)}


JOBS = 6


def step(out: Path, name: str, fn):
    return d8().step(out, name, fn)


def run_all(out: Path) -> int:
    from yopo_l0.policy import StudentPolicy

    m = d8()
    defaults = dict(StudentPolicy.DEFAULTS)
    m._check_official()
    out.mkdir(parents=True, exist_ok=True)
    for s in FIT_SEEDS:  # 先把权重都训完（每个约 10 s），再评估
        for arm, ds in TRAIN_ARMS.items():
            step(out, f"train-{arm}-s{s}", lambda: m.train_arm(out, ds, s))
    order = [(arm, None) for arm in TRAIN_ARMS] + [(arm, 2.0) for arm in TRAIN_ARMS] + \
            [("ctrl", 1.5), ("ctrl", 2.5), ("g2w", 1.5), ("g2w", 2.5)]
    for arm, cap in order:  # 主臂先跑完，扫描后跑
        for s in FIT_SEEDS:
            w = out / "train" / f"{TRAIN_ARMS[arm]}-s{s}" / "student.pt"
            step(out, f"eval-{arm}-s{s}-cap{cap_tag(cap)}", lambda: evaluate(f"{arm}-s{s}-cap{cap_tag(cap)}", w, cap))
    if dict(StudentPolicy.DEFAULTS) != defaults:
        raise SystemExit("StudentPolicy.DEFAULTS 被改了")
    m._check_official()
    print("ALL-DONE", flush=True)
    return 0


# ---------- 统计 ----------

def sign_test(diffs) -> dict:
    """双侧精确符号检验；差为 0 的种子去掉。"""
    pos = sum(1 for d in diffs if d > 0)
    neg = sum(1 for d in diffs if d < 0)
    n = pos + neg
    if n == 0:
        return {"n_pos": 0, "n_neg": 0, "n_tie": len(diffs), "p_two_sided": 1.0}
    k = min(pos, neg)
    p = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n * 2
    return {"n_pos": pos, "n_neg": neg, "n_tie": len(diffs) - n, "p_two_sided": min(1.0, p)}


def bootstrap_ci(values, n_boot: int = N_BOOT, seed: int = 0) -> dict:
    """种子层面的自助法：对种子有放回重抽，均值的 95% 百分位区间。"""
    v = np.asarray(values, float)
    rng = np.random.default_rng(seed)
    means = v[rng.integers(0, len(v), size=(n_boot, len(v)))].mean(axis=1)
    return {"mean": float(v.mean()), "ci95": [float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))],
            "n_boot": n_boot}


def seed_metrics(body: dict) -> dict:
    rows = body["rows"]
    o = [r for r in rows if r["scene"] == "open-3drone"]
    g = [r for r in rows if r["scene"] == "g2-pillar"]
    uav2 = [t["t_arrive_radius_s"] for r in o for t in r.get("drone_timing", []) if t["id"] == "uav2"]
    return {
        "open_arrived": sum(int(r["arrived"]) for r in o), "open_denom": sum(int(r["n_drones"]) for r in o),
        "g2_arrived": sum(int(r["arrived"]) for r in g), "g2_denom": sum(int(r["n_drones"]) for r in g),
        "obstacle": sum(int(r["obstacle"]) for r in rows), "inter_drone": sum(int(r["inter_drone"]) for r in rows),
        "timeout": sum(int(r["timeout"]) for r in rows),
        "open_all3_within_0p05": sum(1 for r in o if r.get("final_goal_error_m") is not None
                                     and float(r["final_goal_error_m"]) <= 0.05),
        "uav2_t_arrive_radius_median_s": float(np.median([x for x in uav2 if x is not None])) if any(
            x is not None for x in uav2) else None,
        "uav2_never_in_radius": sum(1 for x in uav2 if x is None),
    }


def _res(out: Path, name: str):
    return d8()._res(out, name)


def report(out: Path, dest: Path) -> dict:
    per = {}
    for arm in TRAIN_ARMS:
        for cap in CAPS:
            for s in FIT_SEEDS:
                b = _res(out, f"eval-{arm}-s{s}-cap{cap_tag(cap)}")
                if b is not None:
                    per[(arm, cap_tag(cap), s)] = {"metrics": seed_metrics(b), "groups": b["groups"]}
    combos = {}
    for arm in TRAIN_ARMS:
        for cap in CAPS:
            ss = [s for s in FIT_SEEDS if (arm, cap_tag(cap), s) in per]
            if not ss:
                continue
            ms = [per[(arm, cap_tag(cap), s)]["metrics"] for s in ss]
            tot = {k: sum(x[k] for x in ms) for k in ("open_arrived", "open_denom", "g2_arrived", "g2_denom",
                                                     "obstacle", "inter_drone", "timeout", "open_all3_within_0p05",
                                                     "uav2_never_in_radius")}
            combos[f"{arm}/cap{cap_tag(cap)}"] = {"seeds": ss, "per_seed": ms, "total": tot,
                                                  "groups_per_seed": [per[(arm, cap_tag(cap), s)]["groups"] for s in ss]}
    comps = []

    def compare(name, a, b, paired_note):
        if a not in combos or b not in combos or combos[a]["seeds"] != combos[b]["seeds"]:
            return
        out_c = {"name": name, "a": a, "b": b, "pairing": paired_note, "metrics": {}}
        for key in ("open_arrived", "g2_arrived", "obstacle", "inter_drone"):
            diffs = [y[key] - x[key] for x, y in zip(combos[a]["per_seed"], combos[b]["per_seed"])]
            out_c["metrics"][key] = {"diffs": diffs, "sign": sign_test(diffs), "bootstrap": bootstrap_ci(diffs)}
        comps.append(out_c)

    compare("限速 2.0 − 对照", "ctrl/capoff", "ctrl/cap2.0", "同一权重，只开关限速")
    compare("限速 1.5 − 对照", "ctrl/capoff", "ctrl/cap1.5", "同一权重，只开关限速")
    compare("限速 2.5 − 对照", "ctrl/capoff", "ctrl/cap2.5", "同一权重，只开关限速")
    compare("加权 ×4 − 对照", "ctrl/capoff", "g2w/capoff", "同一训练种子、不同数据权重（重训，不是严格配对）")
    compare("限速 + 加权 − 加权", "g2w/capoff", "g2w/cap2.0", "同一权重，只开关限速")
    compare("限速 + 加权 − 对照", "ctrl/capoff", "g2w/cap2.0", "同一训练种子（重训，不是严格配对）")
    compare("限速 + 加权 − 限速", "ctrl/cap2.0", "g2w/cap2.0", "同一训练种子（重训，不是严格配对）")
    compare("加权·限速 1.5 − 加权", "g2w/capoff", "g2w/cap1.5", "同一权重，只开关限速")
    compare("加权·限速 2.5 − 加权", "g2w/capoff", "g2w/cap2.5", "同一权重，只开关限速")
    trains = {f"{arm}-s{s}": {k: (_res(out, f"train-{arm}-s{s}") or {}).get(k)
                              for k in ("seed", "weights_sha256", "holdout", "g2_weight", "n_dagger_windows_by_scene")}
              for arm in TRAIN_ARMS for s in FIT_SEEDS}
    body = {"title": "票 D9：限速与 g2 加权 5 种子确认（只报数）", "disclaimer": DISCLAIMER,
            "fit_seeds": list(FIT_SEEDS), "eval_seeds": d8().EVAL_SEEDS, "altitudes_m": list(d8().ALTITUDES),
            "caps": [cap_tag(c) for c in CAPS], "g2_weight": d8().G2_WEIGHT, "main_arms": {k: f"{a}/cap{cap_tag(c)}"
                                                                                       for k, (a, c) in MAIN_ARMS.items()},
            "stats": {"sign_test": "双侧精确符号检验，种子为单位，差为 0 去掉", "bootstrap": f"种子层面有放回重抽 {N_BOOT} 次，均值 95% 百分位区间"},
            "train": trains, "combos": combos, "comparisons": comps}
    d8()._write_json(dest, body)
    return body


def main(argv=None) -> int:
    global JOBS
    ap = argparse.ArgumentParser(description="D9：限速与 g2 加权 5 种子确认（只报数）")
    ap.add_argument("mode", choices=("all", "report"))
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--dest", type=Path, default=ROOT / "docs" / "research" / "d9-cap-weight-runs.json")
    a = ap.parse_args(argv)
    JOBS = a.jobs
    if a.mode == "report":
        report(a.out, a.dest)
        return 0
    return run_all(a.out)


if __name__ == "__main__":
    raise SystemExit(main())
