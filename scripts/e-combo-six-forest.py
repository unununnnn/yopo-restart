#!/usr/bin/env python3
"""六机林带上把 E-A1、D1、E-B2 三个开关一起只报数。

不改默认关，不改已冻卡，不进 scenes/suite.txt，不改
scripts/e-combo-a1-d1-b2.py 和 docs/research/e-combo-a1-d1-b2.md。
开关拨法和那份三机脚本相同，只是底卡换成已有的六机林带卡。

臂：
  off    全关。medium 是 scenes/crossing-6-forest.json，
         sparse 是 scenes/crossing-6-forest-sparse.json。
  ea1    policy_params.upstream.inter_drone_filter，阈值 0.70 m，惩罚 0。
  d1     geometric_fallback.enabled，机间 0.70 m，承诺视界 1.0 s。
         这一臂用 RunConfig 盖过，场景卡哈希与 off 相同。
  eb2    policy_params.backend.enabled = true，其余后段限额用代码里的缺省。
  combo  上面三项一起开。combo 的回退写在进程内的卡上，不改磁盘上的卡。

影子盾沿 RunConfig 缺省：开，只记录，不改批准动作。
后段预算是墙钟（缺省 0.02 s）。没有后段的臂可以两个进程。
带后段的臂用 --jobs-backend 1，并且不要同时再跑别的重活：截止时间按墙钟切迭代，
多挤一下闭环结果就会变。

本票结果整表标 report-only，不挡合入。已冻结的到达、碰撞、机间门一个都不改。
队形误差 p95 按结果里的 formation_error_p95_m（官方口径）只报数，不新冻结。
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-combo-six-forest" / "partial"
BODY_R = 0.35

# 与 E-A1 / 三机联合对照同一组数：滤掉，不减分。0.70 m 是机体半径的两倍，也是机间碰撞线。
_EA1 = {
    "inter_drone_filter": True,
    "inter_drone_threshold_m": 0.70,
    "inter_drone_penalty": 0,
}
# 与 D1 的对向变体、三机联合对照同一组数。
_D1 = {
    "enabled": True,
    "inter_drone_distance_m": 0.7,
    "commit_horizon_s": 1.0,
}

ARMS = ("off", "ea1", "d1", "eb2", "combo")
SCENES = ("medium", "sparse")
SCENE_CARDS = {
    "medium": ROOT / "scenes/crossing-6-forest.json",
    "sparse": ROOT / "scenes/crossing-6-forest-sparse.json",
}
BACKEND_ARMS = frozenset({"eb2", "combo"})

# 13e E+F 合入后、SUMMARY 在 f5985e4335fc 上记下的全关逐种子数。
# 场景卡后来加了 criteria.frozen / gates，哈希从 13e 那一版变成了 SUMMARY 这一版，
# 规划路径不应受这两项影响。off 对不上就如实写，不改旧报告。
_OFF_HASH = {
    "medium": "657efeb6fae1a62ddc1b061d4cf04a3013f660d2e57b28f70a42d5c23c1db9ee",
    "sparse": "7e3328908444d4615c34e217fbb871fec2bed1e2fcb451f8738c13ba2b24762a",
}
# arrived, min_inter, min_clearance, formation_error_p95, arrival_time
_OFF_ANCHORS = {
    "medium": {
        0: (6, 1.293, 0.306, 23.472, 26.08),
        1: (6, 1.392, 0.161, 23.813, 28.13),
        2: (6, 1.148, 0.202, 24.591, 26.90),
        3: (6, 1.189, 0.239, 24.195, 29.08),
        4: (6, 1.111, 0.194, 23.041, 28.40),
        5: (6, 1.260, 0.126, 26.298, 33.57),
        6: (6, 1.224, 0.191, 24.213, 32.07),
        7: (6, 1.248, 0.218, 26.108, 26.90),
        8: (6, 1.361, 0.170, 24.769, 26.00),
        9: (6, 1.187, 0.180, 25.937, 28.94),
    },
    "sparse": {
        0: (6, 0.959, 0.315, 20.976, 25.40),
        1: (6, 0.982, 0.178, 18.725, 23.24),
        2: (6, 1.242, 0.322, 20.661, 30.31),
        3: (6, 0.972, 0.380, 19.778, 31.58),
        4: (6, 1.322, 0.192, 20.842, 24.83),
        5: (6, 1.039, 0.253, 20.075, 24.40),
        6: (6, 1.012, 0.402, 21.695, 24.81),
        7: (6, 1.002, 0.186, 20.476, 29.30),
        8: (6, 1.043, 0.401, 22.922, 27.48),
        9: (6, 1.053, 0.158, 21.293, 26.06),
    },
}


def _deep(raw: dict) -> dict:
    return json.loads(json.dumps(raw))


def _base(scene: str) -> dict:
    return json.loads(SCENE_CARDS[scene].read_text())


def _with_flags(raw: dict, arm: str) -> dict:
    """只加这一臂要开的字段。不改 criteria，不改任务。"""
    out = _deep(raw)
    if arm in ("ea1", "combo"):
        params = dict(out.get("policy_params") or {})
        upstream = dict(params.get("upstream") or {})
        upstream.update(_EA1)
        params["upstream"] = upstream
        out["policy_params"] = params
    if arm in ("eb2", "combo"):
        params = dict(out.get("policy_params") or {})
        backend = dict(params.get("backend") or {})
        backend["enabled"] = True
        params["backend"] = backend
        out["policy_params"] = params
    if arm in ("d1", "combo"):
        out["geometric_fallback"] = dict(_D1)
    return out


def _materialize(raw: dict):
    """与 load_card 相同的内联几何，好让场景卡哈希口径一致。"""
    from yopo_l0.scene import SceneCard, parse_geometry

    card = _deep(raw)
    if isinstance(card.get("forest"), dict):
        from yopo_l0.forest import generate_forest_geometry

        geom = generate_forest_geometry(card["forest"])
    else:
        geom = json.loads((ROOT / "scenes" / card["geometry"]).read_text())
    card = {**card, "geometry": geom}
    return SceneCard(card["name"], card, parse_geometry(geom, card.get("ground_z")))


def build_card(scene: str, arm: str):
    """返回 (SceneCard, geometric_fallback 覆盖或 None)。

    d1 只用 RunConfig 打开回退，卡哈希与 off 相同，和三机密林 D1 臂一样。
    combo 的回退写在卡上。磁盘上的六机卡不改。
    """
    from yopo_l0.scene import load_card

    if arm == "off":
        return load_card(SCENE_CARDS[scene]), None
    base = _base(scene)
    if arm == "d1":
        return _materialize(base), True
    flagged = _with_flags(base, arm)
    return _materialize(flagged), None


def _deadlock(result: dict, ticks: list[dict]) -> bool:
    """E-A1 报告里的口径，只记在本报告，不进门。

    TIMEOUT，末 2 s 每架速度都不超过 0.2 m/s，且至少一架离目标超过 0.4 m。
    """
    if result.get("outcome") != "TIMEOUT" or not ticks:
        return False
    t_end = max(float(row["t"]) for row in ticks)
    window = [row for row in ticks if float(row["t"]) >= t_end - 2.0 - 1e-9]
    speeds: dict[str, float] = {}
    for row in window:
        v = row["v"]
        speed = (float(v[0]) ** 2 + float(v[1]) ** 2 + float(v[2]) ** 2) ** 0.5
        uid = str(row["uav"])
        speeds[uid] = max(speeds.get(uid, 0.0), speed)
    if not speeds or any(speed > 0.2 + 1e-9 for speed in speeds.values()):
        return False
    last: dict[str, float] = {}
    for row in ticks:
        last[str(row["uav"])] = float(row["goal_error_m"])
    return any(err > 0.4 for err in last.values())


def _budget_wrap():
    """数一下后段有没有被墙钟预算截断。包装本身不改曲线。"""
    import yopo_l0.backend.bspline as bspline

    current = bspline.refine_yopo
    if getattr(current, "_combo_wrapped", False):
        current.cuts = 0
        current.calls = 0
        return current
    orig = current

    def wrapped(*args, **kwargs):
        out, rec = orig(*args, **kwargs)
        wrapped.calls += 1
        wrapped.cuts += int(bool(rec.get("budget_cut")))
        return out, rec

    wrapped.calls = 0
    wrapped.cuts = 0
    wrapped._combo_wrapped = True
    bspline.refine_yopo = wrapped
    return wrapped


def _drop_samples(block: dict | None) -> dict | None:
    if not isinstance(block, dict):
        return None
    out = dict(block)
    for key in ("time_ms", "iterations"):
        part = out.get(key)
        if isinstance(part, dict):
            kept = dict(part)
            kept.pop("samples", None)
            out[key] = kept
    return out


def _row(scene: str, arm: str, result: dict, ticks: list[dict], *, budget_calls: int, budget_cuts: int) -> dict:
    dist = result.get("min_obstacle_distance_m")
    clearance = None if dist is None else float(dist) - BODY_R
    block = result.get("crossing") or {}
    if block.get("min_obstacle_clearance_m") is not None:
        clearance = float(block["min_obstacle_clearance_m"])
    reason = result.get("reason") or ""
    outcome = result.get("outcome")
    tree = int(outcome == "COLLISION" and "TRUTH_POSITION" in str(reason))
    inter = int(outcome == "COLLISION" and "INTER_DRONE" in str(reason))
    if "obstacle_collisions" in block:
        # 穿林按真值停在第一次碰撞，和上面的 0/1 一致。这里留着对照。
        tree = int(block.get("obstacle_collisions") or 0)
        inter = int(block.get("inter_drone_collisions") or 0)
    geom = result.get("geometric_fallback") or {}
    timing = geom.get("check_time_ms") or {}
    filt = result.get("inter_drone_filter") or {}
    plan = result.get("plan_wall_ms") or {}
    return {
        "scene_kind": scene,
        "arm": arm,
        "scene": result.get("scene"),
        "seed": int(result["seed"]),
        "outcome": outcome,
        "reason": reason,
        "arrived": int(result["arrived"]),
        "n_drones": int(result["n_drones"]),
        "arrival_rate": result.get("arrival_rate"),
        "arrival_time_s": result.get("arrival_time_s"),
        "tree_collisions": tree,
        "inter_drone_collisions": inter,
        "timeout": int(outcome == "TIMEOUT"),
        "deadlock": _deadlock(result, ticks),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "min_obstacle_distance_m": dist,
        "min_clearance_m": clearance,
        "formation_error_p95_m": result.get("formation_error_p95_m"),
        "sim_duration_s": result.get("sim_duration_s"),
        "final_goal_error_m": result.get("final_goal_error_m"),
        "wall_time_s": result.get("wall_time_s"),
        "plan_wall_ms": {
            "count": plan.get("count"),
            "mean": plan.get("mean"),
            "p50": plan.get("p50"),
            "p95": plan.get("p95"),
            "max": plan.get("max"),
        },
        "fallback_checks": int(geom.get("checks") or 0),
        "fallbacks": int(geom.get("fallbacks") or 0),
        "fallback_reasons": geom.get("reasons") or {},
        "fallback_actions": geom.get("actions") or {},
        "check_p50_ms": timing.get("p50"),
        "check_p95_ms": timing.get("p95"),
        "check_max_ms": timing.get("max"),
        "filtered_candidates": int(filt.get("filtered_candidates") or 0),
        "empty_fallbacks": int(filt.get("empty_fallbacks") or 0),
        "inter_drone_filter": filt or None,
        "backend": _drop_samples(result.get("backend")),
        "backend_budget_calls": int(budget_calls),
        "backend_budget_cuts": int(budget_cuts),
        "shadow": result.get("shadow"),
        "shadow_reject_rate": result.get("shadow_reject_rate"),
        "scene_card_hash": result.get("scene_card_hash"),
        "commit": result.get("commit"),
        "empty_solution_count": result.get("empty_solution_count"),
    }


def _partial(scene: str, arm: str, seed: int) -> Path:
    return PARTIAL / f"{scene}-{arm}-{seed}.json"


def run_one(scene: str, arm: str, seed: int, *, resume: bool) -> dict:
    dest = _partial(scene, arm, seed)
    if resume and dest.is_file():
        return json.loads(dest.read_text())
    from yopo_l0.runner import RunConfig, run

    card, geom_flag = build_card(scene, arm)
    wrapped = _budget_wrap()
    cfg = RunConfig(seed=int(seed), shadow=True, geometric_fallback=geom_flag)
    t0 = time.perf_counter()
    out = run(card, cfg)
    row = _row(
        scene,
        arm,
        out.result,
        out.ticks,
        budget_calls=int(wrapped.calls),
        budget_cuts=int(wrapped.cuts),
    )
    row["worker_s"] = time.perf_counter() - t0
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False))
    tmp.replace(dest)
    print(
        f"{scene} {arm} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"tree={row['tree_collisions']} inter={row['inter_drone_collisions']} "
        f"gap={row['min_inter_drone_distance_m']} clear={row['min_clearance_m']} "
        f"p95={row['formation_error_p95_m']} sim={row['sim_duration_s']} "
        f"wall={row['wall_time_s']:.1f}s",
        flush=True,
    )
    return row


def _worker(spec: tuple) -> dict:
    scene, arm, seed, resume = spec
    return run_one(scene, arm, int(seed), resume=bool(resume))


def _specs(scenes: list[str], arms: list[str], seeds: list[int], resume: bool) -> list[tuple]:
    order = {"off": 0, "ea1": 1, "d1": 2, "eb2": 3, "combo": 4}
    specs = [(scene, arm, seed, resume) for scene in scenes for arm in arms for seed in seeds]
    specs.sort(key=lambda item: (order[item[1]], 0 if item[0] == "medium" else 1, item[2]))
    return specs


def _run_specs(specs: list[tuple], jobs: int) -> list[dict]:
    if jobs <= 1 or len(specs) <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)


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


def summarize(rows: list[dict]) -> str:
    lines = []
    for scene in SCENES:
        if not any(r["scene_kind"] == scene for r in rows):
            continue
        lines.append(f"## {scene}")
        lines.append("")
        lines.append(
            "| 臂 | 到达 | 撞树 | 机间碰撞 | 超时 | 死锁 | 最小机间 m | 最小净空 m | "
            "队形误差 p95 m | plan_wall p95 ms | 回退检查 p50/p95 ms | 后段 p50/p95 ms | 后段截断 |"
        )
        lines.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
        for arm in ARMS:
            group = [r for r in rows if r["scene_kind"] == scene and r["arm"] == arm]
            if not group:
                continue
            drones = sum(int(r["n_drones"]) for r in group)
            arrived = sum(int(r["arrived"]) for r in group)
            trees = sum(int(r["tree_collisions"]) for r in group)
            inter = sum(int(r["inter_drone_collisions"]) for r in group)
            timeouts = sum(int(r["timeout"]) for r in group)
            dead = sum(int(bool(r["deadlock"])) for r in group)
            gaps = [float(r["min_inter_drone_distance_m"]) for r in group if r["min_inter_drone_distance_m"] is not None]
            clears = [float(r["min_clearance_m"]) for r in group if r["min_clearance_m"] is not None]
            p95s = [float(r["formation_error_p95_m"]) for r in group if r.get("formation_error_p95_m") is not None]
            walls = [float(r["plan_wall_ms"]["p95"]) for r in group if (r.get("plan_wall_ms") or {}).get("p95") is not None]
            c50 = [float(r["check_p50_ms"]) for r in group if r.get("check_p50_ms") is not None]
            c95 = [float(r["check_p95_ms"]) for r in group if r.get("check_p95_ms") is not None]
            b50, b95 = [], []
            cuts = calls = 0
            for r in group:
                cuts += int(r.get("backend_budget_cuts") or 0)
                calls += int(r.get("backend_budget_calls") or 0)
                backend = r.get("backend") or {}
                t = backend.get("time_ms") or {}
                if t.get("p50") is not None:
                    b50.append(float(t["p50"]))
                if t.get("p95") is not None:
                    b95.append(float(t["p95"]))
            check = "—"
            if c50 and c95:
                check = f"{_rng(c50, 1)} / {_rng(c95, 1)}"
            backend_t = "—"
            if b50 and b95:
                backend_t = f"{_rng(b50, 1)} / {_rng(b95, 1)}"
            lines.append(
                f"| {arm} | {arrived}/{drones} | {trees} | {inter} | {timeouts} | {dead} | "
                f"{_rng(gaps, 3)} | {_rng(clears, 3)} | {_rng(p95s, 3)} | {_rng(walls, 1)} | "
                f"{check} | {backend_t} | {cuts}/{calls} |"
            )
        lines.append("")
    return "\n".join(lines)


def _anchors(rows: list[dict]) -> list[str]:
    """和 13e E+F / SUMMARY 里的全关对照。差一截就说明这台机器上的 off 不是那一列。"""
    notes = []
    by = {(r["scene_kind"], r["arm"], int(r["seed"])): r for r in rows}

    def near(row, key, expect, tol, label):
        got = row.get(key)
        ok = got is not None and abs(float(got) - float(expect)) <= tol
        notes.append(f"{'OK' if ok else 'DIFF'} {label}: got {got} expect {expect}")

    for scene, expect_hash in _OFF_HASH.items():
        sample = next((r for r in rows if r["scene_kind"] == scene and r["arm"] == "off"), None)
        if sample is None:
            continue
        got = sample.get("scene_card_hash")
        ok = got == expect_hash
        notes.append(f"{'OK' if ok else 'DIFF'} {scene} off hash: got {got} expect {expect_hash}")

    for scene, seeds in _OFF_ANCHORS.items():
        for seed, (arrived, gap, clear, p95, t_arr) in seeds.items():
            row = by.get((scene, "off", seed))
            if row is None:
                continue
            label = f"{scene} off seed{seed}"
            ok_arr = int(row["arrived"]) == arrived and int(row["n_drones"]) == 6
            notes.append(
                f"{'OK' if ok_arr else 'DIFF'} {label} arrived {row['arrived']}/{row['n_drones']} "
                f"expect {arrived}/6 outcome {row['outcome']} reason {row.get('reason')}"
            )
            near(row, "min_inter_drone_distance_m", gap, 0.001, f"{label} gap")
            near(row, "min_clearance_m", clear, 0.001, f"{label} clearance")
            near(row, "formation_error_p95_m", p95, 0.001, f"{label} p95")
            near(row, "arrival_time_s", t_arr, 0.015, f"{label} arrival_time")
            tree = int(row.get("tree_collisions") or 0)
            inter = int(row.get("inter_drone_collisions") or 0)
            ok_col = tree == 0 and inter == 0
            notes.append(f"{'OK' if ok_col else 'DIFF'} {label} collisions tree={tree} inter={inter} expect 0 0")
    return notes


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", default="medium,sparse")
    ap.add_argument("--arms", default="off,combo")
    ap.add_argument("--seeds", default="0,1,2,3,4,5,6,7,8,9")
    ap.add_argument("--jobs-plain", type=int, default=2)
    ap.add_argument("--jobs-backend", type=int, default=1)
    ap.add_argument("--out", default="docs/research/e-combo-six-forest-runs.json")
    ap.add_argument("--no-resume", action="store_true")
    a = ap.parse_args(argv)
    scenes = [s for s in a.scenes.split(",") if s]
    arms = [s for s in a.arms.split(",") if s]
    seeds = [int(s) for s in a.seeds.split(",") if s != ""]
    unknown = [s for s in scenes if s not in SCENES] + [s for s in arms if s not in ARMS]
    if unknown:
        raise SystemExit(f"未知场景或臂：{unknown}")
    resume = not a.no_resume
    # 两个进程同时第一次加载权重时会抢同一个上游缓存目录，后启动的那个 cwd 会被删掉。
    # 父进程先取好，工作进程只读缓存。
    from yopo_swarm_core.baseline import fetch_upstream

    fetch_upstream()
    specs = _specs(scenes, arms, seeds, resume)
    plain = [s for s in specs if s[1] not in BACKEND_ARMS]
    backend = [s for s in specs if s[1] in BACKEND_ARMS]
    rows = []
    if plain:
        rows.extend(_run_specs(plain, int(a.jobs_plain)))
    if backend:
        rows.extend(_run_specs(backend, int(a.jobs_backend)))
    rows.sort(key=lambda r: (r["scene_kind"], ARMS.index(r["arm"]), int(r["seed"])))
    payload = {
        "rows": rows,
        "shadow": True,
        "jobs_plain": int(a.jobs_plain),
        "jobs_backend": int(a.jobs_backend),
        "note": "墙钟不参与轨迹，也不进已冻门。backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。整表 report-only。",
    }
    dest = Path(a.out)
    if not dest.is_absolute():
        dest = ROOT / dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    print(summarize(rows))
    for line in _anchors(rows):
        print(line)
    print(f"写入 {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
