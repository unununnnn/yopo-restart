#!/usr/bin/env python3
"""票 E-B2r：用新缺省重跑密林和 014 对向。只报数。

库缺省已是 budget="evals"、0.020 s → 20 次。本脚本不写 time_budget_s，
也不把 budget 设成 wall。密林用场景卡上已经打开的后段。014 对向按
scripts/e-c4-density-l0.py 的 combo 盖上 E-A1 和 D1，后段只写 enabled=true，
次数走缺省。打开后段的这两张表标 report-only / 偏离上游，不是把开关改成默认开。

--check 不跑仿真。它核对缺省是 20 次、显式传入的秒数不被改成 20、
冻结套件和 SUMMARY 里的卡都不打开后段。
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-b2r-evals-default" / "partial"
BODY_R = 0.35
ROW_VERSION = 1

CASES = {
    "forest": {
        "title": "E-B2 三机密林，后段开，评估次数走库缺省",
        "scene": "scenes/crossing-forest-dense-bspline.json",
        "seeds": tuple(range(10)),
    },
    "c4b014": {
        "title": "E-C4b 014 对向，固定 E-A1 + D1，后段次数走库缺省",
        "scene": "scenes/ego-density-014-swap3.json",
        "seeds": tuple(range(10)),
    },
}

WATCHED = (
    "scenes/suite.txt",
    "scenes/crossing-forest-dense-bspline.json",
    "scenes/ego-density-014-swap3.json",
    "scenes/g2-pillar-bspline.json",
    "README.md",
    "docs/tickets.md",
    "docs/results/SUMMARY.md",
    "docs/results/SUMMARY.json",
    "scripts/premerge.sh",
    "scripts/regress.sh",
    "scripts/sitl/Dockerfile",
    "core",
)

_ENV_CLEAR = (
    "YOPO_BACKEND_BUDGET",
    "YOPO_PEER_SPHERE",
    "YOPO_PEER_SPHERE_RADIUS_M",
    "YOPO_PEER_SPHERE_WEIGHT",
)


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    if path.is_dir():
        for item in sorted(p for p in path.rglob("*") if p.is_file()):
            h.update(str(item.relative_to(path)).encode())
            h.update(item.read_bytes())
        return h.hexdigest()
    h.update(path.read_bytes())
    return h.hexdigest()


def watched_hashes() -> dict[str, str]:
    return {rel: _sha256(ROOT / rel) for rel in WATCHED}


def _clear_budget_env() -> None:
    for key in _ENV_CLEAR:
        os.environ.pop(key, None)


def _commit() -> str:
    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()


def _pct(values: list[float], q: float) -> float | None:
    if not values:
        return None
    import numpy as np

    return float(np.percentile(np.asarray(values, dtype=float), q))


def _suite_paths() -> list[Path]:
    paths = []
    for line in (ROOT / "scenes" / "suite.txt").read_text(encoding="utf-8").splitlines():
        body = line.split("#", 1)[0].strip()
        if not body:
            continue
        paths.append(ROOT / body.split("@", 1)[0])
    return paths


def _c4_module():
    path = ROOT / "scripts" / "e-c4-density-l0.py"
    spec = importlib.util.spec_from_file_location("e_c4_density_l0_b2r", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _scene_card(case: str):
    """不写 time_budget_s。次数就是库缺省。"""
    from yopo_l0.scene import load_card

    spec = CASES[case]
    if case == "forest":
        return load_card(ROOT / spec["scene"])
    if case == "c4b014":
        return _c4_module().build_run_card("014", "swap3", "combo")
    raise KeyError(case)


def _install_observer() -> None:
    """补上评估次数和截断次数。只活在这个进程里，不改库文件，也不改曲线。"""
    import yopo_l0.backend.bspline as bspline

    if getattr(bspline, "_b2r_installed", False):
        return
    orig_min = bspline._minimize
    orig_record = bspline._record
    orig_sum = bspline.summarize_log

    def min_wrap(x0, q_fit, free, cost_grad, cfg, t0):
        counted = {"n": 0}

        def wrapped(q):
            counted["n"] += 1
            return cost_grad(q)

        out = orig_min(x0, q_fit, free, wrapped, cfg, t0)
        min_wrap.last = counted["n"]
        return out

    min_wrap.last = None

    def rec_wrap(*args, **kwargs):
        rec = orig_record(*args, **kwargs)
        rec["evals"] = min_wrap.last
        min_wrap.last = None
        return rec

    def sum_wrap(rows):
        out = orig_sum(rows)
        evals = [int(r["evals"]) for r in rows if isinstance(r.get("evals"), int)]
        iters = [int(r.get("iterations") or 0) for r in rows]
        hist: dict[str, int] = {}
        for value in iters:
            key = str(value)
            hist[key] = hist.get(key, 0) + 1
        out["budget_cuts"] = int(sum(1 for r in rows if r.get("budget_cut")))
        out["evals"] = {
            "n": len(evals),
            "min": None if not evals else int(min(evals)),
            "max": None if not evals else int(max(evals)),
            "mean": None if not evals else float(sum(evals) / len(evals)),
            "p50": _pct([float(v) for v in evals], 50),
        }
        out["iteration_hist"] = hist
        return out

    bspline._minimize = min_wrap
    bspline._record = rec_wrap
    bspline.summarize_log = sum_wrap
    bspline._b2r_installed = True


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


def _compact(case: str, result: dict, wall_s: float) -> dict:
    dist = result.get("min_obstacle_distance_m")
    clearance = None if dist is None else float(dist) - BODY_R
    gap = result.get("min_inter_drone_distance_m")
    block = result.get("crossing") if isinstance(result.get("crossing"), dict) else {}
    if "obstacle_collisions" in block:
        obstacle_hit = int(block["obstacle_collisions"])
        inter_hit = int(block["inter_drone_collisions"])
    else:
        obstacle_hit = int(dist is not None and dist <= BODY_R + 1e-9)
        inter_hit = int(gap is not None and gap <= 2.0 * BODY_R + 1e-9)
    backend = _drop_samples(result.get("backend"))
    iters = (backend or {}).get("iterations") or {}
    evals = (backend or {}).get("evals") or {}
    calls = int((backend or {}).get("calls") or 0)
    cuts = int((backend or {}).get("budget_cuts") or 0)
    return {
        "version": ROW_VERSION,
        "case": case,
        "seed": int(result.get("seed")),
        "scene": result.get("scene"),
        "outcome": result.get("outcome"),
        "reason": result.get("reason"),
        "arrived": int(result.get("arrived") or 0),
        "n_drones": int(result.get("n_drones") or 0),
        "timeout": int(result.get("outcome") == "TIMEOUT"),
        "arrival_time_s": result.get("arrival_time_s"),
        "sim_duration_s": result.get("sim_duration_s"),
        "min_obstacle_distance_m": dist,
        "min_obstacle_clearance_m": clearance,
        "obstacle_collisions": obstacle_hit,
        "inter_drone_collisions": inter_hit,
        "min_inter_drone_distance_m": gap,
        "wall_time_s": wall_s,
        "backend_calls": calls,
        "backend_applied": int((backend or {}).get("applied") or 0),
        "backend_budget_cuts": cuts,
        "truncated": bool(cuts > 0),
        "iterations_p50": iters.get("p50"),
        "iterations_mean": iters.get("mean"),
        "iteration_hist": (backend or {}).get("iteration_hist") or {},
        "evals_p50": evals.get("p50"),
        "evals_max": evals.get("max"),
        "eval_cap": 20,
        "time_budget_s_written": False,
    }


def _worker(job: dict) -> dict:
    _clear_budget_env()
    os.environ["OMP_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    _install_observer()
    from yopo_l0.backend.bspline import _eval_cap
    from yopo_l0.backend.config import config_from_params
    from yopo_l0.runner import RunConfig, run

    card = _scene_card(job["case"])
    backend = (card.card.get("policy_params") or {}).get("backend") or {}
    if "time_budget_s" in backend or backend.get("budget") not in (None, "evals"):
        raise RuntimeError(f"{job['case']} 写了预算，不是库缺省: {backend}")
    cfg = config_from_params(card.card.get("policy_params"), a_max=6.0)
    if cfg is None or cfg.budget != "evals" or _eval_cap(cfg) != 20:
        raise RuntimeError(f"{job['case']} 缺省不是 20 次: {cfg}")
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(job["seed"]), shadow=True, shadow_env_evidence=False))
    row = _compact(job["case"], out.result, time.perf_counter() - t0)
    PARTIAL.mkdir(parents=True, exist_ok=True)
    dest = PARTIAL / f"{row['case']}-seed{row['seed']}.json"
    dest.write_text(json.dumps(row, ensure_ascii=False) + "\n", encoding="utf-8")
    print(
        f"{row['case']} seed={row['seed']} outcome={row['outcome']} reason={row['reason']} "
        f"arrived={row['arrived']}/{row['n_drones']} hit={row['obstacle_collisions']} "
        f"inter={row['inter_drone_collisions']} timeout={row['timeout']} "
        f"clear={row['min_obstacle_clearance_m']} cuts={row['backend_budget_cuts']}/{row['backend_calls']} "
        f"iter_p50={row['iterations_p50']} eval_max={row['evals_max']} wall={row['wall_time_s']:.1f}s",
        flush=True,
    )
    return row


def _load_partial(case: str, seed: int) -> dict | None:
    path = PARTIAL / f"{case}-seed{seed}.json"
    if not path.is_file():
        return None
    row = json.loads(path.read_text(encoding="utf-8"))
    if int(row.get("version", 0)) != ROW_VERSION:
        return None
    if row.get("case") != case or int(row.get("seed")) != seed:
        return None
    if row.get("eval_cap") != 20 or row.get("time_budget_s_written") is not False:
        return None
    return row


def _summarize(rows: list[dict]) -> dict:
    groups: dict[str, list[dict]] = {}
    for row in rows:
        groups.setdefault(row["case"], []).append(row)
    out = {}
    for case, group in sorted(groups.items()):
        drones = sum(int(r["n_drones"]) for r in group)
        arrived = sum(int(r["arrived"]) for r in group)
        clears = [float(r["min_obstacle_clearance_m"]) for r in group if r["min_obstacle_clearance_m"] is not None]
        pooled: list[float] = []
        for row in group:
            for key, count in (row.get("iteration_hist") or {}).items():
                pooled.extend([float(key)] * int(count))
        eval_max = [int(r["evals_max"]) for r in group if r.get("evals_max") is not None]
        out[case] = {
            "case": case,
            "title": CASES[case]["title"],
            "runs": len(group),
            "arrived": arrived,
            "drones": drones,
            "obstacle_collisions": sum(int(r["obstacle_collisions"]) for r in group),
            "inter_drone_collisions": sum(int(r["inter_drone_collisions"]) for r in group),
            "timeouts": sum(int(r["timeout"]) for r in group),
            "min_clearance_m": None if not clears else min(clears),
            "iterations_p50": _pct(pooled, 50),
            "iterations_mean": None if not pooled else float(sum(pooled) / len(pooled)),
            "evals_max": None if not eval_max else max(eval_max),
            "backend_calls": sum(int(r["backend_calls"]) for r in group),
            "backend_budget_cuts": sum(int(r["backend_budget_cuts"]) for r in group),
            "runs_with_any_cut": sum(1 for r in group if r["truncated"]),
        }
    return out


def check() -> int:
    """干跑：缺省 20 次、显式预算、冻结套件不走后段。不跑仿真。"""
    _clear_budget_env()
    from yopo_l0.backend.bspline import _IDLE_EVAL_COST_S, _eval_cap
    from yopo_l0.backend.config import DEFAULT_BUDGET, BackendConfig, config_from_params
    from yopo_l0.scene import load_card

    before = watched_hashes()
    assert DEFAULT_BUDGET == "evals"
    assert _IDLE_EVAL_COST_S == 0.001
    bare = BackendConfig()
    assert bare.budget == "evals" and bare.time_budget_s == 0.020
    assert _eval_cap(bare) == 20
    assert config_from_params({}, a_max=6.0) is None
    assert config_from_params({"backend": {"enabled": False}}, a_max=6.0) is None
    default_on = config_from_params({"backend": {"enabled": True}}, a_max=6.0)
    assert default_on is not None and _eval_cap(default_on) == 20
    # 显式传入的秒数按同一比例折算，不会被改成 20。
    assert _eval_cap(BackendConfig(time_budget_s=0.010)) == 10
    assert _eval_cap(BackendConfig(time_budget_s=0.040)) == 40
    assert _eval_cap(BackendConfig(time_budget_s=1e-4)) == 1
    assert _eval_cap(BackendConfig(time_budget_s=5.0)) == 5000
    explicit = config_from_params({"backend": {"enabled": True, "time_budget_s": 0.010}}, a_max=6.0)
    assert explicit is not None and _eval_cap(explicit) == 10

    suite_names = set()
    for path in _suite_paths():
        raw = json.loads(path.read_text(encoding="utf-8"))
        assert config_from_params(raw.get("policy_params"), a_max=6.0) is None, path.name
        assert "bspline" not in path.name
        suite_names.add(path.name)

    summary = json.loads((ROOT / "docs" / "results" / "SUMMARY.json").read_text(encoding="utf-8"))
    summary_scenes = []
    for row in summary["rows"]:
        name = Path(row["scene_path"]).name
        summary_scenes.append(name)
        assert name in suite_names, name
        card = load_card(ROOT / "scenes" / name)
        assert config_from_params(card.card.get("policy_params"), a_max=6.0) is None, name

    forest = json.loads((ROOT / CASES["forest"]["scene"]).read_text(encoding="utf-8"))
    disk_backend = forest["policy_params"]["backend"]
    assert disk_backend.get("enabled") is True
    assert "time_budget_s" not in disk_backend
    assert "budget" not in disk_backend
    forest_card = _scene_card("forest")
    forest_cfg = config_from_params(forest_card.card.get("policy_params"), a_max=6.0)
    assert forest_cfg is not None and _eval_cap(forest_cfg) == 20

    c4 = _scene_card("c4b014")
    block = c4.card["policy_params"]["backend"]
    assert block.get("enabled") is True
    assert "time_budget_s" not in block
    assert "budget" not in block
    assert c4.card["geometric_fallback"]["enabled"] is True
    assert c4.card["policy_params"]["upstream"]["inter_drone_filter"] is True
    c4_cfg = config_from_params(c4.card.get("policy_params"), a_max=6.0)
    assert c4_cfg is not None and c4_cfg.budget == "evals" and _eval_cap(c4_cfg) == 20
    assert c4_cfg.swarm_penalty is False and c4_cfg.peer_sphere_enabled is False

    after = watched_hashes()
    assert before == after, "干跑改写了受保护文件"
    print(
        json.dumps(
            {
                "check": "ok",
                "default_budget": "evals",
                "default_time_budget_s": 0.020,
                "default_eval_cap": 20,
                "idle_eval_cost_s": 0.001,
                "explicit_0_010s_evals": 10,
                "suite_backend_off": sorted(suite_names),
                "summary_scenes_backend_off": sorted(set(summary_scenes)),
                "summary_would_change": False,
                "simulation": False,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="干跑：不跑仿真")
    parser.add_argument("--case", default="forest,c4b014")
    parser.add_argument("--seeds", default="", help="缺省 0–9。逗号分隔")
    parser.add_argument("--jobs", type=int, default=3)
    parser.add_argument("--no-resume", action="store_true")
    parser.add_argument(
        "--out",
        type=Path,
        default=ROOT / "docs" / "research" / "e-b2r-evals-default-runs.json",
    )
    args = parser.parse_args(argv)
    if args.check:
        return check()
    cases = [part.strip() for part in args.case.split(",") if part.strip()]
    for case in cases:
        if case not in CASES:
            raise SystemExit(f"未知 case: {case}")
    if args.jobs < 1:
        raise SystemExit("--jobs 至少为 1")
    _clear_budget_env()
    if os.environ.get("YOPO_BACKEND_BUDGET", "").strip().lower() == "wall":
        raise SystemExit("本票不用墙钟预算")

    jobs = []
    rows: list[dict] = []
    seed_override = tuple(int(part) for part in args.seeds.split(",") if part != "")
    for case in cases:
        seeds = seed_override or CASES[case]["seeds"]
        for seed in seeds:
            if not args.no_resume:
                cached = _load_partial(case, int(seed))
                if cached is not None:
                    rows.append(cached)
                    continue
            jobs.append({"case": case, "seed": int(seed)})
    if jobs:
        import multiprocessing as mp

        from yopo_swarm_core.baseline import fetch_upstream

        fetch_upstream()
        ctx = mp.get_context("spawn")
        workers = min(args.jobs, len(jobs))
        with ctx.Pool(workers) as pool:
            rows.extend(pool.map(_worker, jobs))
    rows.sort(key=lambda r: (r["case"], r["seed"]))
    payload = {
        "title": "票 E-B2r：后段评估预算缺省 10→20",
        "note": (
            "只报数。report-only。未冻结。后段研究开关仍默认关。"
            "打开后段之后的缺省是 budget=evals，0.020 s 对应 20 次。"
            "本文件的两次重跑都没有写 time_budget_s。"
            "密林用场景卡上已经打开的后段。014 对向固定 E-A1 + D1，只让后段走新缺省。"
            "截断指后段评估次数上限把这一次求解停下，不是仿真超时。"
            "冻结套件不打开后段，SUMMARY 不因这次缺省改变。"
            "旧研究文档里按 10 次报的数字没有改。"
        ),
        "commit": _commit(),
        "budget_mode": "evals",
        "time_budget_s_default": 0.020,
        "idle_eval_cost_s": 0.001,
        "default_eval_cap": 20,
        "time_budget_s_written": False,
        "max_iterations_unchanged": 8,
        "cases": {name: {"title": CASES[name]["title"], "scene": CASES[name]["scene"], "seeds": list(CASES[name]["seeds"])} for name in cases},
        "rows": rows,
        "summary": _summarize(rows),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload["summary"], indent=2, ensure_ascii=False))
    print(f"写入 {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
