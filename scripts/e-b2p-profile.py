#!/usr/bin/env python3
"""E-B2p：剖析后段单次调用。默认不改代码，只读当前求解器。"""

from __future__ import annotations

import argparse
import cProfile
import json
import pstats
import time
from collections import Counter
from io import StringIO
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def _line(p0, p1, speed=3.0):
    from yopo_swarm_core.plan import quintic
    from yopo_swarm_core.trajectory import PiecewisePolynomial

    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    delta = p1 - p0
    dist = float(np.linalg.norm(delta))
    duration = max(0.8, dist / speed)
    v = delta / duration
    head = np.array([p0, v, np.zeros(3)])
    end = np.array([p1, v, np.zeros(3)])
    return PiecewisePolynomial(0.0, [quintic(head, end, duration)])


def _load_obs(tag: str):
    from yopo_l0.scene import load_card

    name = {
        "014": "ego-density-014-solo.json",
        "028": "ego-density-028-solo.json",
        "042": "ego-density-042-solo.json",
        "g2": "g2-pillar-bspline.json",
        "dense": "crossing-forest-dense-bspline.json",
    }[tag]
    return load_card(ROOT / "scenes" / name).obstacles


def _cfg(budget=0.020, iters=8, **kw):
    from yopo_l0.backend.config import BackendConfig

    d = dict(a_max=6.0, v_max=6.0, max_iterations=iters, time_budget_s=budget, clearance_m=0.35)
    d.update(kw)
    return BackendConfig(**d)


def _stage_breakdown(traj, obstacles, repeats=5):
    import yopo_l0.backend.bspline as B
    from yopo_l0.backend.bspline import (
        _choose_knots,
        _boundary_controls,
        _fit_free,
        _pva,
        _sample_layout,
        _cost_grad,
        min_signed_distance,
        _to_traj,
        _kin_ok,
        _ends_match,
        _dyn_samples,
    )

    duration = float(traj.t_end - traj.t_start)
    n_span, dt = _choose_knots(duration)
    times = {k: [] for k in ("pva_fit", "layout", "clr_before", "dyn_samples", "cost_grad", "sdf", "smooth_dyn_anchor", "to_traj", "kin_ok", "clr_after")}

    p0 = v0 = a0 = p1 = v1 = a1 = None
    q_fit = free = idx = wts = None
    d_safe = 0.35 + B._CLEARANCE_BUFFER_M
    for _ in range(repeats):
        t0 = time.perf_counter()
        p0, v0, a0 = _pva(traj, float(traj.t_start))
        p1, v1, a1 = _pva(traj, float(traj.t_end))
        n_ctrl = n_span + 3
        q_fixed = np.zeros((n_ctrl, 3), dtype=float)
        q_fixed[0], q_fixed[1], q_fixed[2] = _boundary_controls(p0, v0, a0, dt)
        q_fixed[-3], q_fixed[-2], q_fixed[-1] = _boundary_controls(p1, v1, a1, dt)
        free = np.arange(3, n_ctrl - 3)
        q_fit, _rmse = _fit_free(traj, q_fixed, free, n_span, dt)
        times["pva_fit"].append((time.perf_counter() - t0) * 1000)

        t0 = time.perf_counter()
        idx, wts = _sample_layout(n_span)
        times["layout"].append((time.perf_counter() - t0) * 1000)

        t0 = time.perf_counter()
        min_signed_distance(traj, obstacles)
        times["clr_before"].append((time.perf_counter() - t0) * 1000)

        t0 = time.perf_counter()
        _dyn_samples(n_span)
        times["dyn_samples"].append((time.perf_counter() - t0) * 1000)

        t0 = time.perf_counter()
        _cost_grad(q_fit, q_fit, free, dt, obstacles, d_safe, 6.0, 6.0, idx, wts)
        times["cost_grad"].append((time.perf_counter() - t0) * 1000)

        # split sdf vs rest of one cost
        t0 = time.perf_counter()
        pts = B._curve_points(q_fit, idx, wts)
        B._nearest(pts, obstacles)
        times["sdf"].append((time.perf_counter() - t0) * 1000)

        t0 = time.perf_counter()
        _cost_grad(q_fit, q_fit, free, dt, (), d_safe, 6.0, 6.0, idx, wts)
        times["smooth_dyn_anchor"].append((time.perf_counter() - t0) * 1000)

        t0 = time.perf_counter()
        built = _to_traj(q_fit, float(traj.t_start), dt, n_span)
        times["to_traj"].append((time.perf_counter() - t0) * 1000)

        t0 = time.perf_counter()
        _ends_match(built, (p0, v0, a0), (p1, v1, a1))
        _kin_ok(built, 6.0)
        times["kin_ok"].append((time.perf_counter() - t0) * 1000)

        t0 = time.perf_counter()
        min_signed_distance(built, obstacles)
        times["clr_after"].append((time.perf_counter() - t0) * 1000)

    def summ(xs):
        a = np.asarray(xs, dtype=float)
        return {"mean_ms": float(a.mean()), "min_ms": float(a.min()), "max_ms": float(a.max())}

    return {
        "duration_s": duration,
        "n_span": int(n_span),
        "n_free": int(len(free)),
        "n_obstacles": int(len(obstacles)),
        "n_samples_cost": int(len(idx)),
        "n_samples_clearance": int(max(2, int(np.ceil(duration / B._SAMPLE_DT_S)) + 1)),
        "stages_ms": {k: summ(v) for k, v in times.items()},
    }


def _call_stats(traj, obstacles, budget, iters, repeats=8, label=""):
    from yopo_l0.backend.bspline import refine_yopo

    cfg = _cfg(budget=budget, iters=iters)
    rows = []
    for _ in range(repeats):
        _out, rec = refine_yopo(traj, cfg, obstacles)
        rows.append(rec)
    times = np.array([r["time_ms"] for r in rows], dtype=float)
    it = np.array([r["iterations"] for r in rows], dtype=float)
    cuts = sum(1 for r in rows if r["budget_cut"])
    return {
        "label": label,
        "budget_s": budget,
        "max_iterations": iters,
        "repeats": repeats,
        "time_ms": {"p50": float(np.percentile(times, 50)), "p95": float(np.percentile(times, 95)), "mean": float(times.mean()), "max": float(times.max())},
        "iterations": {"p50": float(np.percentile(it, 50)), "p95": float(np.percentile(it, 95)), "mean": float(it.mean()), "max": int(it.max())},
        "budget_cut_rate": cuts / repeats,
        "reasons": dict(Counter(r["reason"] for r in rows)),
        "applied_rate": sum(1 for r in rows if r["applied"]) / repeats,
        "clearance_before_m": rows[-1].get("clearance_before_m"),
        "clearance_after_m": rows[-1].get("clearance_after_m"),
        "last": {k: rows[-1][k] for k in ("applied", "reason", "iterations", "time_ms", "budget_cut", "clearance_before_m", "clearance_after_m", "fit_rmse_m")},
    }


def _cprofile_cost(traj, obstacles, n=20):
    import yopo_l0.backend.bspline as B

    duration = float(traj.t_end - traj.t_start)
    n_span, dt = B._choose_knots(duration)
    p0, v0, a0 = B._pva(traj, float(traj.t_start))
    p1, v1, a1 = B._pva(traj, float(traj.t_end))
    n_ctrl = n_span + 3
    q_fixed = np.zeros((n_ctrl, 3), dtype=float)
    q_fixed[0], q_fixed[1], q_fixed[2] = B._boundary_controls(p0, v0, a0, dt)
    q_fixed[-3], q_fixed[-2], q_fixed[-1] = B._boundary_controls(p1, v1, a1, dt)
    free = np.arange(3, n_ctrl - 3)
    q_fit, _ = B._fit_free(traj, q_fixed, free, n_span, dt)
    idx, wts = B._sample_layout(n_span)
    d_safe = 0.35 + B._CLEARANCE_BUFFER_M
    pr = cProfile.Profile()
    pr.enable()
    for _ in range(n):
        B._cost_grad(q_fit, q_fit, free, dt, obstacles, d_safe, 6.0, 6.0, idx, wts)
        B.min_signed_distance(traj, obstacles)
    pr.disable()
    buf = StringIO()
    stats = pstats.Stats(pr, stream=buf).sort_stats("tottime")
    stats.print_stats(25)
    return buf.getvalue()


def _cprofile_refine(traj, obstacles, budget=0.02, iters=8, n=5):
    from yopo_l0.backend.bspline import refine_yopo

    cfg = _cfg(budget=budget, iters=iters)
    # warmup import
    refine_yopo(traj, cfg, obstacles)
    pr = cProfile.Profile()
    pr.enable()
    for _ in range(n):
        refine_yopo(traj, cfg, obstacles)
    pr.disable()
    buf = StringIO()
    stats = pstats.Stats(pr, stream=buf).sort_stats("tottime")
    stats.print_stats(30)
    return buf.getvalue()


def _real_calls(tag: str, duration_s: float, max_keep: int):
    """短跑密度场，截住前几条真实 refine 输入。"""
    import json
    from yopo_l0.backend import bspline
    from yopo_l0.scene import load_card
    from yopo_l0.runner import RunConfig, run

    captured = []
    orig = bspline.refine_yopo

    def hook(traj, cfg, obstacles, goal=None):
        if len(captured) < max_keep:
            captured.append(
                {
                    "duration_s": float(traj.t_end - traj.t_start),
                    "n_obs": len(tuple(obstacles or ())),
                    "t_start": float(traj.t_start),
                    "traj": traj,
                    "obstacles": tuple(obstacles or ()),
                    "goal": None if goal is None else np.asarray(goal, dtype=float).copy(),
                }
            )
        return orig(traj, cfg, obstacles, goal)

    bspline.refine_yopo = hook
    try:
        path = ROOT / "scenes" / f"ego-density-{tag}-solo.json"
        raw = json.loads(path.read_text())
        raw = json.loads(json.dumps(raw))
        params = dict(raw.get("policy_params") or {})
        backend = dict(params.get("backend") or {})
        backend["enabled"] = True
        params["backend"] = backend
        raw["policy_params"] = params
        tmp = ROOT / "out" / "e-b2p" / f"tmp-{tag}-solo-backend.json"
        tmp.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(json.dumps(raw))
        card = load_card(tmp)
        out = run(card, RunConfig(seed=0, shadow=True, duration_s=duration_s, geometric_fallback=False))
        result = out.result
    finally:
        bspline.refine_yopo = orig
        try:
            tmp.unlink()
        except Exception:
            pass
    backend = result.get("backend") or {}
    return captured, {
        "calls": backend.get("calls"),
        "applied": backend.get("applied"),
        "fallback": backend.get("fallback"),
        "iterations": {k: v for k, v in (backend.get("iterations") or {}).items() if k != "samples"},
        "time_ms": {k: v for k, v in (backend.get("time_ms") or {}).items() if k != "samples"},
        "reasons": backend.get("reasons"),
        "budget_cuts": sum(1 for r in (backend.get("iterations") or {}).get("samples", []) or []),
        "arrived": result.get("arrived"),
        "tree_collisions": result.get("tree_collisions") or result.get("collisions"),
        "wall_time_s": result.get("wall_time_s"),
        "raw_budget": {
            "cuts": int(sum(1 for rec in (result.get("backend") or {}).get("iterations", {}).get("samples", []) or [])),
        },
        "log_budget_cut_rate": None,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tags", default="014,028,042")
    ap.add_argument("--repeats", type=int, default=7)
    ap.add_argument("--capture-s", type=float, default=2.0, help="真实短跑时长，0 表示跳过")
    ap.add_argument("--out", default="out/e-b2p/profile-before.json")
    args = ap.parse_args()

    report = {"synthetic": [], "cprofile": {}, "real": []}
    for tag in args.tags.split(","):
        tag = tag.strip()
        obs = _load_obs(tag)
        # 一条穿场的 2 s 左右匀速直线，和 YOPO 两段前缀同量级。
        if tag in {"014", "028", "042"}:
            traj = _line((-8.0, 0.2, 1.5), (-2.0, -0.1, 1.5), speed=3.0)
        elif tag == "g2":
            traj = _line((-2.0, 0.0, 1.5), (2.0, 0.0, 1.5), speed=2.0)
        else:
            traj = _line((-4.0, 0.0, 1.5), (2.0, 0.3, 1.5), speed=2.5)
        stages = _stage_breakdown(traj, obs, repeats=max(3, args.repeats))
        def_budget = _call_stats(traj, obs, 0.020, 8, repeats=args.repeats, label=f"{tag}-synthetic-budget")
        relaxed = _call_stats(traj, obs, 1.0, 8, repeats=args.repeats, label=f"{tag}-synthetic-relaxed")
        report["synthetic"].append({"tag": tag, "stages": stages, "budget": def_budget, "relaxed": relaxed})
        if tag in {"042", "014"}:
            report["cprofile"][tag + "_cost"] = _cprofile_cost(traj, obs, n=10)
            report["cprofile"][tag + "_refine"] = _cprofile_refine(traj, obs, budget=0.02, n=4)

    if args.capture_s > 0:
        for tag in [t.strip() for t in args.tags.split(",") if t.strip() in {"014", "028", "042"}]:
            captured, summary = _real_calls(tag, args.capture_s, max_keep=8)
            rows = []
            for i, c in enumerate(captured):
                st = _stage_breakdown(c["traj"], c["obstacles"], repeats=3)
                b = _call_stats(c["traj"], c["obstacles"], 0.020, 8, repeats=4, label=f"{tag}-real{i}-budget")
                r = _call_stats(c["traj"], c["obstacles"], 1.0, 8, repeats=3, label=f"{tag}-real{i}-relaxed")
                rows.append({"duration_s": c["duration_s"], "n_obs": c["n_obs"], "stages": st, "budget": b, "relaxed": r})
            report["real"].append({"tag": tag, "run": summary, "n_captured": len(captured), "calls": rows})

    out = ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    # cProfile 文本太大，一起写入旁边的 txt；json 里只留标志。
    txt = out.with_suffix(".cprofile.txt")
    with txt.open("w") as f:
        for k, v in report["cprofile"].items():
            f.write(f"\n===== {k} =====\n")
            f.write(v)
    report["cprofile"] = {k: f"(see {txt.name})" for k in report["cprofile"]}
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps({
        "out": str(out),
        "synthetic": [
            {
                "tag": s["tag"],
                "n_obs": s["stages"]["n_obstacles"],
                "duration_s": s["stages"]["duration_s"],
                "stages_mean_ms": {k: v["mean_ms"] for k, v in s["stages"]["stages_ms"].items()},
                "budget": s["budget"]["time_ms"] | {"cut": s["budget"]["budget_cut_rate"], "iters": s["budget"]["iterations"]},
                "relaxed": s["relaxed"]["time_ms"] | {"cut": s["relaxed"]["budget_cut_rate"], "iters": s["relaxed"]["iterations"]},
            }
            for s in report["synthetic"]
        ],
        "real": [
            {"tag": r["tag"], "run_time_ms": r["run"].get("time_ms"), "run_iters": r["run"].get("iterations"), "n_captured": r["n_captured"]}
            for r in report["real"]
        ],
    }, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
