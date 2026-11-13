#!/usr/bin/env python3
"""E-B3c 小批量只报数：段内机间惩罚 vs 控制点版 vs 全关，seeds 0–9。

场景：密度 0.14 三机对向（scenes/ego-density-014-swap3.json）、八机开阔对向
（scenes/swap-8-open.json）。不改这两张卡，开关在进程里覆盖。

臂：
  off  默认全关。
  b3   E-B3：后段打开，机间椭球打在控制点上。C=0.80 m、c=2.0 是假设，不是论文复现。
  b3c  E-B3c：同样的椭球，改在每段内点上算，梯度按 B 样条基函数送回自由控制点。

不把卡写入 scenes/suite.txt，不冻结，不改默认开关，不改 yopo_l0/policy.py。
后段预算是墙钟（0.02 s）。带后段的臂默认一个进程；off 可以两个。
每粒结果落在 out/e-b3c-segment-penalty/partial/，可续跑。
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from yopo_l0.experiment import comma_ints, comma_list, markdown_table, run_completion_jobs, unknown_values, write_json

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-b3c-segment-penalty" / "partial"
BODY_R = 0.35
ARMS = ("off", "b3", "b3c")
SCENES = ("d014-swap3", "swap8")
SEEDS = tuple(range(10))
DISCLAIMER = "这是假设，不是论文复现"

# 假设，不是论文复现。论文只写 c > 1，C 由用户定，没有给数值。
# 椭球距离 d = sqrt(dx² + dy² + (c·dz)²)，d < C 时加平方铰链。竖直半轴 C/c = 0.40 m。
_B3 = {
    "enabled": True,
    "swarm_penalty": True,
    "swarm_clearance_m": 0.80,
    "swarm_c": 2.0,
}
_B3C = {**_B3, "swarm_segment_penalty": True}


def _deep(raw: dict) -> dict:
    return json.loads(json.dumps(raw))


def _with_flags(raw: dict, arm: str) -> dict:
    out = _deep(raw)
    if arm == "off":
        return out
    params = dict(out.get("policy_params") or {})
    backend = dict(params.get("backend") or {})
    if arm == "b3":
        backend.update(_B3)
    elif arm == "b3c":
        backend.update(_B3C)
    else:
        raise ValueError(arm)
    params["backend"] = backend
    out["policy_params"] = params
    return out


def _materialize(raw: dict):
    from yopo_l0.scene import SceneCard, parse_geometry

    card = _deep(raw)
    geom = card.get("geometry")
    if isinstance(geom, str):
        geom = json.loads((ROOT / "scenes" / geom).read_text(encoding="utf-8"))
        card = {**card, "geometry": geom}
    return SceneCard(card["name"], card, parse_geometry(geom, card.get("ground_z")))


def build_card(scene: str, arm: str):
    from yopo_l0.scene import load_card

    if scene == "d014-swap3":
        path = ROOT / "scenes/ego-density-014-swap3.json"
    elif scene == "swap8":
        path = ROOT / "scenes/swap-8-open.json"
    else:
        raise ValueError(scene)
    if arm == "off":
        return load_card(path)
    raw = json.loads(path.read_text(encoding="utf-8"))
    return _materialize(_with_flags(raw, arm))


def _budget_wrap():
    import yopo_l0.backend.bspline as bspline

    current = bspline.refine_yopo
    if getattr(current, "_eb3c_wrapped", False):
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
    wrapped._eb3c_wrapped = True
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


def _row(scene: str, arm: str, result: dict, *, budget_calls: int, budget_cuts: int) -> dict:
    dist = result.get("min_obstacle_distance_m")
    reason = result.get("reason") or ""
    outcome = result.get("outcome")
    backend = _drop_samples(result.get("backend"))
    time_ms = (backend or {}).get("time_ms") or {}
    penalty = _B3C if arm == "b3c" else _B3 if arm == "b3" else None
    return {
        "scene_kind": scene,
        "arm": arm,
        "scene": result.get("scene"),
        "seed": int(result["seed"]),
        "outcome": outcome,
        "reason": reason,
        "arrived": int(result["arrived"]),
        "n_drones": int(result["n_drones"]),
        "arrival_time_s": result.get("arrival_time_s"),
        "tree_collisions": int(outcome == "COLLISION" and "TRUTH_POSITION" in str(reason)),
        "inter_drone_collisions": int(outcome == "COLLISION" and "INTER_DRONE" in str(reason)),
        "timeout": int(outcome == "TIMEOUT"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "min_obstacle_distance_m": dist,
        "min_clearance_m": None if dist is None else float(dist) - BODY_R,
        "sim_duration_s": result.get("sim_duration_s"),
        "final_goal_error_m": result.get("final_goal_error_m"),
        "wall_time_s": result.get("wall_time_s"),
        "backend": backend,
        "backend_time_p50_ms": time_ms.get("p50"),
        "backend_time_p95_ms": time_ms.get("p95"),
        "backend_budget_calls": int(budget_calls),
        "backend_budget_cuts": int(budget_cuts),
        "scene_card_hash": result.get("scene_card_hash"),
        "commit": result.get("commit"),
        "disclaimer": DISCLAIMER,
        "swarm_clearance_m": None if penalty is None else penalty["swarm_clearance_m"],
        "swarm_c": None if penalty is None else penalty["swarm_c"],
        "swarm_segment_penalty": arm == "b3c",
    }


def _partial(scene: str, arm: str, seed: int) -> Path:
    return PARTIAL / f"{scene}-{arm}-{seed}.json"


def run_one(scene: str, arm: str, seed: int, *, resume: bool) -> dict:
    dest = _partial(scene, arm, seed)
    if resume and dest.is_file():
        return json.loads(dest.read_text(encoding="utf-8"))
    from yopo_l0.runner import RunConfig, run

    card = build_card(scene, arm)
    wrapped = _budget_wrap()
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(seed), shadow=True))
    row = _row(scene, arm, out.result, budget_calls=int(wrapped.calls), budget_cuts=int(wrapped.cuts))
    row["worker_s"] = time.perf_counter() - t0
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)
    gap = row["min_inter_drone_distance_m"]
    gap_s = "—" if gap is None else f"{float(gap):.3f}"
    p50 = row["backend_time_p50_ms"]
    p95 = row["backend_time_p95_ms"]
    p50_s = "—" if p50 is None else f"{float(p50):.1f}"
    p95_s = "—" if p95 is None else f"{float(p95):.1f}"
    print(
        f"{scene} {arm} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"tree={row['tree_collisions']} inter={row['inter_drone_collisions']} "
        f"gap={gap_s} backend_p50={p50_s}ms p95={p95_s}ms cuts={row['backend_budget_cuts']}/"
        f"{row['backend_budget_calls']} wall={row['wall_time_s']:.1f}s",
        flush=True,
    )
    return row


def _job(spec):
    scene, arm, seed, resume = spec
    return run_one(scene, arm, int(seed), resume=bool(resume))


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
    summary_rows = []
    scenes = []
    for row in rows:
        if row["scene_kind"] not in scenes:
            scenes.append(row["scene_kind"])
    for scene in scenes:
        for arm in ARMS:
            group = [r for r in rows if r["scene_kind"] == scene and r["arm"] == arm]
            if not group:
                continue
            arrived = sum(int(r["arrived"]) for r in group)
            total = sum(int(r["n_drones"]) for r in group)
            tree = sum(int(r["tree_collisions"]) for r in group)
            inter = sum(int(r["inter_drone_collisions"]) for r in group)
            timeout = sum(int(r["timeout"]) for r in group)
            gaps = [float(r["min_inter_drone_distance_m"]) for r in group if r.get("min_inter_drone_distance_m") is not None]
            p50 = [float(r["backend_time_p50_ms"]) for r in group if r.get("backend_time_p50_ms") is not None]
            p95 = [float(r["backend_time_p95_ms"]) for r in group if r.get("backend_time_p95_ms") is not None]
            cuts = sum(int(r.get("backend_budget_cuts") or 0) for r in group)
            calls = sum(int(r.get("backend_budget_calls") or 0) for r in group)
            backend_t = "—" if not p50 else f"{_rng(p50, 1)} / {_rng(p95, 1)}"
            summary_rows.append(
                f"| {scene} | {arm} | {arrived}/{total} | {tree} | {inter} | {timeout} | "
                f"{_rng(gaps, 3)} | {backend_t} | {cuts}/{calls} |"
            )
    lines.append(markdown_table(
        "| 场景 | 臂 | 到达 | 撞树 | 机间碰撞 | 超时 | 最小机间 m | 后段 p50 / p95 ms | 预算截断 |",
        "|---|---|---|---|---|---|---|---|---|",
        summary_rows,
    ))
    lines.append("")
    lines.append("### 分种子")
    lines.append("")
    lines.append("| 场景 | 臂 | seed | 结果 | 到达 | 撞树 | 机间 | 最小机间 m | 后段 p50 ms | 后段 p95 ms |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        lines.append(
            f"| {r['scene_kind']} | {r['arm']} | {r['seed']} | {r.get('outcome') or '—'} | "
            f"{r['arrived']}/{r['n_drones']} | {r['tree_collisions']} | {r['inter_drone_collisions']} | "
            f"{_f(r.get('min_inter_drone_distance_m'), 3)} | {_f(r.get('backend_time_p50_ms'), 1)} | "
            f"{_f(r.get('backend_time_p95_ms'), 1)} |"
        )
    return "\n".join(lines) + "\n"


def _load_partials(scenes: list[str], arms: list[str], seeds: list[int]) -> list[dict]:
    rows = []
    for scene in scenes:
        for arm in arms:
            for seed in seeds:
                path = _partial(scene, arm, seed)
                if path.is_file():
                    rows.append(json.loads(path.read_text(encoding="utf-8")))
    return rows


def _run_group(scenes: list[str], arms: list[str], seeds: list[int], *, jobs: int, resume: bool) -> list[dict]:
    specs = [(scene, arm, seed, resume) for scene in scenes for arm in arms for seed in seeds]
    return run_completion_jobs(specs, _job, jobs)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", default=",".join(SCENES))
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    ap.add_argument("--jobs-plain", type=int, default=2)
    ap.add_argument("--jobs-backend", type=int, default=1)
    ap.add_argument("--out", default="docs/research/e-b3c-segment-penalty-runs.json")
    ap.add_argument("--no-resume", action="store_true")
    a = ap.parse_args(argv)
    scenes = comma_list(a.scenes)
    arms = comma_list(a.arms)
    seeds = comma_ints(a.seeds)
    unknown = unknown_values(scenes, SCENES) + unknown_values(arms, ARMS)
    if unknown:
        raise SystemExit(f"未知场景或臂: {unknown}")
    resume = not a.no_resume
    if int(a.jobs_plain) > 2 or int(a.jobs_backend) > 2:
        raise SystemExit("最多 2 个进程")

    from yopo_swarm_core.baseline import fetch_upstream

    fetch_upstream()
    plain = [arm for arm in arms if arm == "off"]
    backend = [arm for arm in arms if arm in ("b3", "b3c")]
    if plain:
        _run_group(scenes, plain, seeds, jobs=max(1, int(a.jobs_plain)), resume=resume)
    if backend:
        _run_group(scenes, backend, seeds, jobs=max(1, int(a.jobs_backend)), resume=resume)

    rows = _load_partials(scenes, list(ARMS), seeds)
    rows.sort(key=lambda r: (SCENES.index(r["scene_kind"]), ARMS.index(r["arm"]), int(r["seed"])))
    expected = len(scenes) * len(arms) * len(seeds)
    got = len([r for r in rows if r["scene_kind"] in scenes and r["arm"] in arms and int(r["seed"]) in seeds])
    payload = {
        "disclaimer": DISCLAIMER,
        "swarm_clearance_m": _B3["swarm_clearance_m"],
        "swarm_c": _B3["swarm_c"],
        "segment_samples_per_span": 6,
        "note": (
            "C、c 是假设，不是论文复现。墙钟不参与轨迹，但后段 0.02 s 预算按墙钟截断。"
            "backend_budget_cuts 是这次截断的次数。b3 的椭球打在控制点上；"
            "b3c 改在每段 6 个内点上，梯度按 B 样条基函数送回自由控制点。"
        ),
        "scenes": scenes,
        "arms": arms,
        "seeds": seeds,
        "completed": got,
        "expected": expected,
        "incomplete": expected - got,
        "rows": [r for r in rows if r["scene_kind"] in scenes and r["arm"] in arms and int(r["seed"]) in seeds],
    }
    out = ROOT / a.out
    write_json(out, payload)
    print(summarize(payload["rows"], title="E-B3c 段内惩罚 vs E-B3 vs 全关"))
    print(f"wrote {out} ({payload['completed']}/{payload['expected']})")
    return 0 if payload["incomplete"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
