#!/usr/bin/env python3
"""八机开阔对向 swap-8-open 补跑种子 1–9，只报数。

不改 scenes/swap-8-open.json 的 seeds 列表（那张卡仍只声明 seed 0）。
种子走 CLI / RunConfig。不改 scripts/e-combo-a1-d1-b2.py，也不改
docs/research/e-combo-a1-d1-b2.md。开关拨法直接复用该脚本的 build_card：
off（全关）、ea1、combo（E-A1+D1+E-B2）。

不把卡写入 scenes/suite.txt，不冻结，不改默认开关。
没有后段的臂两个进程；combo 带后段，一个进程，避免两份 L-BFGS 抢 0.02 s 墙钟预算。
"""

from __future__ import annotations

import argparse
import os
import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ARMS = ("off", "ea1", "combo")
SCENE = "swap8"
SEEDS = tuple(range(1, 10))
PARTIAL = ROOT / "out" / "e-combo-swap8" / "partial"
# 已合入对照报告里的 seed 0，只抄录，不重跑。
SEED0_SOURCE = ROOT / "docs/research/e-combo-a1-d1-b2-runs.json"


def _load_combo():
    path = ROOT / "scripts/e-combo-a1-d1-b2.py"
    spec = importlib.util.spec_from_file_location("e_combo_a1_d1_b2", path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"无法加载 {path}")
    mod = importlib.util.module_from_spec(spec)
    # 让同目录脚本里的相对导入能找到自己。
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    mod.PARTIAL = PARTIAL
    return mod


def _copy_seed0() -> list[dict]:
    if not SEED0_SOURCE.is_file():
        return []
    payload = json.loads(SEED0_SOURCE.read_text())
    rows = []
    for raw in payload.get("rows") or []:
        if raw.get("scene_kind") != SCENE or int(raw.get("seed", -1)) != 0:
            continue
        if raw.get("arm") not in ARMS:
            continue
        row = dict(raw)
        row["copied_from"] = "docs/research/e-combo-a1-d1-b2-runs.json"
        rows.append(row)
    rows.sort(key=lambda r: ARMS.index(r["arm"]))
    return rows


def _f(value, nd: int) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{nd}f}"


def _rng(values: list[float], nd: int) -> str:
    if not values:
        return "—"
    lo, hi = min(values), max(values)
    if abs(lo - hi) < 10 ** (-nd) / 2:
        return _f(lo, nd)
    return f"{_f(lo, nd)}–{_f(hi, nd)}"


def summarize(rows: list[dict], *, title: str) -> str:
    lines = [f"## {title}", ""]
    lines.append("| 臂 | 到达 | 机间碰撞 | 超时 | 最小机间 m | 最小净空 m | 仿真 s |")
    lines.append("|---|---|---|---|---|---|---|")
    for arm in ARMS:
        group = [r for r in rows if r["arm"] == arm]
        if not group:
            continue
        drones = sum(int(r["n_drones"]) for r in group)
        arrived = sum(int(r["arrived"]) for r in group)
        inter = sum(int(r["inter_drone_collisions"]) for r in group)
        timeouts = sum(int(r["timeout"]) for r in group)
        gaps = [
            float(r["min_inter_drone_distance_m"])
            for r in group
            if r.get("min_inter_drone_distance_m") is not None
        ]
        clears = [float(r["min_clearance_m"]) for r in group if r.get("min_clearance_m") is not None]
        sims = [float(r["sim_duration_s"]) for r in group if r.get("sim_duration_s") is not None]
        lines.append(
            f"| {arm} | {arrived}/{drones} | {inter} | {timeouts} | "
            f"{_rng(gaps, 3)} | {_rng(clears, 3)} | {_rng(sims, 2)} |"
        )
    lines.append("")
    lines.append("| seed | 臂 | 结果 | 到达 | 原因 | 最小机间 m | 最小净空 m | 仿真 s | 墙钟 s |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for seed in sorted({int(r["seed"]) for r in rows}):
        for arm in ARMS:
            group = [r for r in rows if int(r["seed"]) == seed and r["arm"] == arm]
            if not group:
                continue
            r = group[0]
            lines.append(
                f"| {seed} | {arm} | {r.get('outcome')} | {r['arrived']}/{r['n_drones']} | "
                f"{r.get('reason') or '—'} | {_f(r.get('min_inter_drone_distance_m'), 3)} | "
                f"{_f(r.get('min_clearance_m'), 3)} | {_f(r.get('sim_duration_s'), 2)} | "
                f"{_f(r.get('wall_time_s'), 1)} |"
            )
    lines.append("")
    return "\n".join(lines)


def _loadavg() -> dict:
    one, five, fifteen = os.getloadavg()
    return {"1": one, "5": five, "15": fifteen}


def _worker(spec: tuple) -> dict:
    """顶层函数，供 spawn 子进程反序列化。动态加载的 combo._worker 过不了 pickle。"""
    scene, arm, seed, resume = spec
    before = _loadavg()
    row = _load_combo().run_one(scene, arm, int(seed), resume=bool(resume))
    row["loadavg_before"] = before
    row["loadavg_after"] = _loadavg()
    return row


def _run_group(arms: list[str], seeds: list[int], *, jobs: int, resume: bool) -> list[dict]:
    specs = [(SCENE, arm, seed, resume) for arm in arms for seed in seeds]
    if jobs <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)



def _load_partials(arms: list[str], seeds: list[int]) -> list[dict]:
    rows = []
    for arm in arms:
        for seed in seeds:
            dest = PARTIAL / f"{SCENE}-{arm}-{seed}.json"
            if dest.is_file():
                rows.append(json.loads(dest.read_text()))
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    ap.add_argument("--jobs-plain", type=int, default=2)
    ap.add_argument("--jobs-backend", type=int, default=1)
    ap.add_argument("--out", default="docs/research/e-combo-swap8-seeds-runs.json")
    ap.add_argument("--no-resume", action="store_true")
    a = ap.parse_args(argv)
    arms = [s for s in a.arms.split(",") if s]
    seeds = [int(s) for s in a.seeds.split(",") if s != ""]
    unknown = [s for s in arms if s not in ARMS]
    if unknown:
        raise SystemExit(f"未知臂：{unknown}")
    if any(s == 0 for s in seeds):
        raise SystemExit("本脚本只补种子 1–9。seed 0 从已合入对照报告抄录，不重跑。")
    resume = not a.no_resume
    combo = _load_combo()
    # 两个进程同时第一次加载权重时会抢同一个上游缓存目录。
    from yopo_swarm_core.baseline import fetch_upstream

    fetch_upstream()
    PARTIAL.mkdir(parents=True, exist_ok=True)
    plain = [arm for arm in arms if arm != "combo"]
    backend = [arm for arm in arms if arm == "combo"]
    rows: list[dict] = []
    if plain:
        rows.extend(_run_group(plain, seeds, jobs=int(a.jobs_plain), resume=resume))
    if backend:
        print(f"combo start loadavg={_loadavg()}", flush=True)
        rows.extend(_run_group(backend, seeds, jobs=int(a.jobs_backend), resume=resume))
    # 把已经落盘的 off/ea1/combo 都收进来，方便只重跑 combo 时仍写出完整 json。
    rows = _load_partials(list(ARMS), seeds)
    rows.sort(key=lambda r: (ARMS.index(r["arm"]), int(r["seed"])))
    seed0 = _copy_seed0()
    payload = {
        "scene": "scenes/swap-8-open.json",
        "scene_kind": SCENE,
        "arms": list(arms),
        "seeds_run": seeds,
        "seed0": {
            "source": "docs/research/e-combo-a1-d1-b2-runs.json",
            "note": "已合入对照报告的 seed 0，只抄录，没有在本脚本里重跑。",
            "rows": seed0,
        },
        "rows": rows,
        "shadow": True,
        "jobs_plain": int(a.jobs_plain),
        "jobs_backend": int(a.jobs_backend),
        "note": "墙钟不参与轨迹。combo 的 backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。",
    }
    dest = Path(a.out)
    if not dest.is_absolute():
        dest = ROOT / dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    if seed0:
        print(summarize(seed0, title="swap8 seed 0（抄自已合入对照，未重跑）"))
    print(summarize(rows, title="swap8 seeds 1–9（本脚本实测）"))
    print(f"写入 {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
