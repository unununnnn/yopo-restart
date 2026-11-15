#!/usr/bin/env python3
"""票 E-B2q：密林开后段的评估次数预算诊断。只报数。

对照三臂以上，全部只活在本进程的场景卡里，不写回磁盘：

  off   后段关。对照，不是新默认。
  e6    后段开，budget=evals，6 次。探旧文迭代 p50=4 附近的次数。
  e10   后段开，budget=evals，10 次。显式 time_budget_s，不是缺省。
  e20   后段开，budget=evals，20 次。票 E-B2r 之后这是库缺省（0.020 s）。
  e40   后段开，budget=evals，40 次。

不用墙钟做预算。打开后段时标「偏离上游」。
不改 core/、场景卡、suite、README、premerge、regress、
SITL Dockerfile、docs/tickets.md、已冻门。诊断原文里的数字不改。

``--check`` 核对库缺省是 20 次、各臂仍是写明的次数、磁盘文件没被改写，不跑仿真。
票 E-B2r 把 0.020 s 改成 20 次之后，各臂改用显式秒数保住原来的 6/10/20/40。
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-b2q-evals-diag" / "partial"
BODY_R = 0.35
ROW_VERSION = 1
# 与 yopo_l0/backend/bspline.py 的 _IDLE_EVAL_COST_S 同一口径。
# 票 E-B2r 之后是 0.001：0.020 s → 20 次。诊断当时按 0.002 把 0.020 s 写成 10 次。
IDLE_EVAL_COST_S = 0.001
DEFAULT_EVALS = 20
UPSTREAM = "report-only / 偏离上游"

# 次数 → time_budget_s。round(s / IDLE_EVAL_COST_S) 必须落回这个次数。
# 显式传入，不靠缺省。E-B2r 之后 10 次是 0.010 s，不再是 0.020 s。
EVAL_BUDGET_S = {6: 0.006, 10: 0.010, 20: 0.020, 40: 0.040}

CASES = {
    "forest": {
        "title": "E-B2 三机密林",
        "scene": "scenes/crossing-forest-dense-bspline.json",
        "seeds": tuple(range(10)),
        "base": "eb2",
    },
    "c4b014": {
        "title": "E-C4b 014 对向（固定 E-A1 + D1，只变后段次数）",
        "scene": "scenes/ego-density-014-swap3.json",
        "seeds": tuple(range(10)),
        "base": "c4b-combo",
    },
    "b6medium": {
        "title": "E-B6 六机 medium（开球臂用 E-B4c 默认球，只变后段次数）",
        "scene": "scenes/crossing-6-forest.json",
        "seeds": tuple(range(5)),
        "base": "b6-sphere",
    },
}

# 与 scripts/e-c4-density-l0.py 的 combo 覆盖相同。不写进场景卡。
_EA1 = {
    "inter_drone_filter": True,
    "inter_drone_threshold_m": 0.70,
    "inter_drone_penalty": 0,
}
_D1 = {
    "enabled": True,
    "inter_drone_distance_m": 0.7,
    "commit_horizon_s": 1.0,
}
_B4C_SPHERE = {"peer_sphere_enabled": True, "peer_sphere_radius_m": 0.85, "peer_sphere_weight": 8.0}

WATCHED = (
    "scenes/crossing-forest-dense-bspline.json",
    "scenes/ego-density-014-swap3.json",
    "scenes/ego-density-014.json",
    "scenes/crossing-6-forest.json",
    "scenes/suite.txt",
    "README.md",
    "docs/tickets.md",
    "scripts/premerge.sh",
    "scripts/regress.sh",
    "scripts/sitl/Dockerfile",
    "yopo_l0/backend/config.py",
    "yopo_l0/backend/bspline.py",
    "core",
)

_ENV_CLEAR = (
    "YOPO_BACKEND_BUDGET",
    "YOPO_PEER_SPHERE",
    "YOPO_PEER_SPHERE_RADIUS_M",
    "YOPO_PEER_SPHERE_WEIGHT",
)


def arms_table() -> dict[str, dict]:
    """诊断臂。off 不算新默认；打开后段的臂都标偏离上游。"""
    rows = {
        "off": {
            "enabled": False,
            "evals": None,
            "label": "后段关（对照，不是新默认）",
            "upstream_deviation": False,
        }
    }
    notes = {
        6: "探旧文迭代 p50=4 附近的次数，显式秒数，不是缺省",
        10: "显式 0.010 s，不是缺省。诊断当时这是缺省",
        20: "显式 0.020 s。票 E-B2r 之后这也是库缺省",
        40: "显式 0.040 s，不是缺省",
    }
    for n, note in notes.items():
        rows[f"e{n}"] = {
            "enabled": True,
            "evals": n,
            "time_budget_s": EVAL_BUDGET_S[n],
            "budget": "evals",
            "label": f"{UPSTREAM}（后段开，budget=evals，{n} 次；{note}）",
            "upstream_deviation": True,
        }
    return rows


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


def _scene_card(case: str, arm_id: str):
    """进程内盖一层。不写场景 JSON。"""
    from yopo_l0.scene import SceneCard, load_card

    spec = CASES[case]
    arm = arms_table()[arm_id]
    loaded = load_card(ROOT / spec["scene"])
    raw = copy.deepcopy(loaded.card)
    base = spec["base"]
    if base == "c4b-combo":
        params = dict(raw.get("policy_params") or {})
        upstream = dict(params.get("upstream") or {})
        upstream.update(_EA1)
        params["upstream"] = upstream
        raw["policy_params"] = params
        raw["geometric_fallback"] = dict(_D1)
    params = dict(raw.get("policy_params") or {})
    backend = dict(params.get("backend") or {})
    if not arm["enabled"]:
        backend["enabled"] = False
        backend.pop("time_budget_s", None)
        backend.pop("budget", None)
        if base == "b6-sphere":
            backend.pop("peer_sphere_enabled", None)
            backend.pop("peer_sphere_radius_m", None)
            backend.pop("peer_sphere_weight", None)
    else:
        backend["enabled"] = True
        backend["budget"] = "evals"
        # 写上秒数，让这一臂的次数不跟着库缺省走。E-B2r 之后缺省 0.020 s 是 20 次。
        backend["time_budget_s"] = float(arm["time_budget_s"])
        backend.pop("swarm_penalty", None)
        backend.pop("swarm_segment_penalty", None)
        if base == "b6-sphere":
            backend.update(_B4C_SPHERE)
        else:
            backend.pop("peer_sphere_enabled", None)
    params["backend"] = backend
    raw["policy_params"] = params
    name = raw.get("name", case)
    raw["name"] = f"{name}-{arm_id}"
    return SceneCard(loaded.path, raw, loaded.obstacles)


def _install_eval_counter() -> None:
    """给这次调用补上评估次数和截断次数。只活在诊断进程里，不改库文件。"""
    import yopo_l0.backend.bspline as bspline

    if getattr(bspline, "_b2q_installed", False):
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
    bspline._b2q_installed = True


def _install_b6_peer_delivery() -> None:
    """与 scripts/e-b6-suite-sphere.py 相同：欧氏球打开才把邻机送进后段。"""
    import importlib.util

    path = ROOT / "scripts" / "e-b6-suite-sphere.py"
    spec = importlib.util.spec_from_file_location("e_b6_suite_sphere_b2q", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.clear_sphere_env()
    mod.install_peer_delivery()


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


def _fingerprint(result: dict) -> str:
    crossing = result.get("crossing") if isinstance(result.get("crossing"), dict) else {}
    payload = {
        "outcome": result.get("outcome"),
        "reason": result.get("reason"),
        "arrived": result.get("arrived"),
        "sim_duration_s": result.get("sim_duration_s"),
        "arrival_time_s": result.get("arrival_time_s"),
        "min_obstacle_distance_m": result.get("min_obstacle_distance_m"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "committed_plan_hash": crossing.get("committed_plan_hash"),
    }
    blob = json.dumps(payload, sort_keys=True, ensure_ascii=False, default=str)
    return hashlib.sha256(blob.encode()).hexdigest()


def _compact(case: str, arm_id: str, result: dict, wall_s: float) -> dict:
    arm = arms_table()[arm_id]
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
        "arm": arm_id,
        "label": arm["label"],
        "upstream_deviation": arm["upstream_deviation"],
        "budget": None if not arm["enabled"] else "evals",
        "eval_cap": arm["evals"],
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
        "iterations_max": iters.get("max"),
        "iteration_hist": (backend or {}).get("iteration_hist") or {},
        "evals_p50": evals.get("p50"),
        "evals_mean": evals.get("mean"),
        "evals_max": evals.get("max"),
        "fingerprint": _fingerprint(result),
        "backend": backend,
    }


def _worker(job: dict) -> dict:
    _clear_budget_env()
    os.environ["OMP_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    _install_eval_counter()
    if job["case"] == "b6medium" and job["arm"] != "off":
        _install_b6_peer_delivery()
    from yopo_l0.runner import RunConfig, run

    card = _scene_card(job["case"], job["arm"])
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(job["seed"]), shadow=True))
    row = _compact(job["case"], job["arm"], out.result, time.perf_counter() - t0)
    PARTIAL.mkdir(parents=True, exist_ok=True)
    dest = PARTIAL / f"{row['case']}-{row['arm']}-seed{row['seed']}.json"
    dest.write_text(json.dumps(row, ensure_ascii=False) + "\n")
    print(
        f"{row['case']} {row['arm']} seed={row['seed']} outcome={row['outcome']} "
        f"arrived={row['arrived']}/{row['n_drones']} hit={row['obstacle_collisions']} "
        f"inter={row['inter_drone_collisions']} timeout={row['timeout']} "
        f"clear={row['min_obstacle_clearance_m']} cuts={row['backend_budget_cuts']}/{row['backend_calls']} "
        f"iter_p50={row['iterations_p50']} eval_p50={row['evals_p50']} wall={row['wall_time_s']:.1f}s",
        flush=True,
    )
    return row


def _load_partial(case: str, arm_id: str, seed: int) -> dict | None:
    path = PARTIAL / f"{case}-{arm_id}-seed{seed}.json"
    if not path.is_file():
        return None
    row = json.loads(path.read_text())
    if int(row.get("version", 0)) != ROW_VERSION:
        return None
    if row.get("arm") != arm_id or row.get("case") != case or int(row.get("seed")) != seed:
        return None
    return row


def _summarize(rows: list[dict]) -> dict:
    groups: dict[tuple[str, str], list[dict]] = {}
    for row in rows:
        groups.setdefault((row["case"], row["arm"]), []).append(row)
    out = {}
    for (case, arm_id), group in sorted(groups.items()):
        drones = sum(int(r["n_drones"]) for r in group)
        arrived = sum(int(r["arrived"]) for r in group)
        hits = sum(int(r["obstacle_collisions"]) for r in group)
        inter = sum(int(r["inter_drone_collisions"]) for r in group)
        timeouts = sum(int(r["timeout"]) for r in group)
        clears = [float(r["min_obstacle_clearance_m"]) for r in group if r["min_obstacle_clearance_m"] is not None]
        iters = [float(r["iterations_p50"]) for r in group if r.get("iterations_p50") is not None]
        calls = sum(int(r["backend_calls"]) for r in group)
        cuts = sum(int(r["backend_budget_cuts"]) for r in group)
        truncated_runs = sum(1 for r in group if r["truncated"])
        pooled: list[float] = []
        for row in group:
            for key, count in (row.get("iteration_hist") or {}).items():
                pooled.extend([float(key)] * int(count))
        eval_max = [int(r["evals_max"]) for r in group if r.get("evals_max") is not None]
        out[f"{case}/{arm_id}"] = {
            "case": case,
            "arm": arm_id,
            "label": group[0]["label"],
            "runs": len(group),
            "arrived": arrived,
            "drones": drones,
            "obstacle_collisions": hits,
            "inter_drone_collisions": inter,
            "timeouts": timeouts,
            "min_clearance_m": None if not clears else min(clears),
            "iterations_p50": _pct(pooled, 50),
            "iterations_mean": None if not pooled else float(sum(pooled) / len(pooled)),
            "iterations_p50_of_seed_p50": _pct(iters, 50),
            "evals_max": None if not eval_max else max(eval_max),
            "backend_calls": calls,
            "backend_budget_cuts": cuts,
            "runs_with_any_cut": truncated_runs,
        }
    return out


def _per_seed(rows: list[dict]) -> list[dict]:
    """同一 case、同一 seed 上各臂并排，用来归因。"""
    by_case: dict[str, dict[int, dict[str, dict]]] = {}
    for row in rows:
        by_case.setdefault(row["case"], {}).setdefault(int(row["seed"]), {})[row["arm"]] = row
    table = []
    for case, seeds in sorted(by_case.items()):
        for seed, arms in sorted(seeds.items()):
            e10 = arms.get("e10")
            e20 = arms.get("e20")
            item = {"case": case, "seed": seed, "arms": {}}
            for arm_id, row in sorted(arms.items()):
                same = None if e10 is None or arm_id == "e10" else row["fingerprint"] == e10["fingerprint"]
                same20 = None if e20 is None or arm_id == "e20" else row["fingerprint"] == e20["fingerprint"]
                item["arms"][arm_id] = {
                    "arrived": row["arrived"],
                    "n_drones": row["n_drones"],
                    "outcome": row["outcome"],
                    "reason": row["reason"],
                    "obstacle_collisions": row["obstacle_collisions"],
                    "inter_drone_collisions": row["inter_drone_collisions"],
                    "timeout": row["timeout"],
                    "min_clearance_m": row["min_obstacle_clearance_m"],
                    "iterations_p50": row["iterations_p50"],
                    "evals_p50": row["evals_p50"],
                    "evals_max": row["evals_max"],
                    "budget_cuts": row["backend_budget_cuts"],
                    "backend_calls": row["backend_calls"],
                    "truncated": row["truncated"],
                    "same_fingerprint_as_e10": same,
                    "same_fingerprint_as_e20": same20,
                }
            table.append(item)
    return table


def check(cases: list[str]) -> int:
    """干跑：缺省次数、臂定义、磁盘文件。不跑仿真。"""
    _clear_budget_env()
    from yopo_l0.backend.bspline import _eval_cap
    from yopo_l0.backend.config import DEFAULT_BUDGET, BackendConfig, config_from_params

    before = watched_hashes()
    assert DEFAULT_BUDGET == "evals"
    bare = BackendConfig()
    assert bare.budget == "evals"
    assert bare.time_budget_s == 0.020
    assert _eval_cap(bare) == DEFAULT_EVALS
    assert config_from_params({}, a_max=6.0) is None
    assert config_from_params({"backend": {"enabled": False}}, a_max=6.0) is None
    default_on = config_from_params({"backend": {"enabled": True}}, a_max=6.0)
    assert default_on is not None and default_on.budget == "evals" and _eval_cap(default_on) == DEFAULT_EVALS

    disk = json.loads((ROOT / CASES["forest"]["scene"]).read_text())
    disk_backend = disk["policy_params"]["backend"]
    assert disk_backend.get("enabled") is True
    assert "time_budget_s" not in disk_backend
    assert "budget" not in disk_backend
    assert "peer_sphere_enabled" not in disk_backend

    for case in cases:
        for arm_id, arm in arms_table().items():
            card = _scene_card(case, arm_id)
            cfg = config_from_params(card.card.get("policy_params"), a_max=6.0)
            if not arm["enabled"]:
                assert cfg is None, (case, arm_id)
                continue
            assert cfg is not None
            assert cfg.budget == "evals"
            assert cfg.budget != "wall"
            assert _eval_cap(cfg) == arm["evals"], (case, arm_id, _eval_cap(cfg), arm["evals"])
            if case == "b6medium":
                assert cfg.peer_sphere_enabled is True
                assert cfg.peer_sphere_radius_m == 0.85
            else:
                assert cfg.peer_sphere_enabled is False
            if case == "c4b014":
                assert card.card["geometric_fallback"]["enabled"] is True
                assert card.card["policy_params"]["upstream"]["inter_drone_filter"] is True
    after = watched_hashes()
    assert before == after, "干跑改写了受保护文件"
    print(
        json.dumps(
            {
                "check": "ok",
                "default_budget": "evals",
                "default_eval_cap": DEFAULT_EVALS,
                "arms": {k: {"evals": v["evals"], "label": v["label"]} for k, v in arms_table().items()},
                "cases": cases,
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
    parser.add_argument("--case", default="forest", help="forest、c4b014、b6medium，或逗号分隔")
    parser.add_argument("--arms", default="off,e6,e10,e20,e40")
    parser.add_argument("--seeds", default="", help="缺省用该 case 的种子。逗号分隔")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--no-resume", action="store_true")
    parser.add_argument(
        "--out",
        type=Path,
        default=ROOT / "docs" / "research" / "e-b2q-evals-diag-runs.json",
    )
    args = parser.parse_args(argv)
    cases = [part.strip() for part in args.case.split(",") if part.strip()]
    for case in cases:
        if case not in CASES:
            raise SystemExit(f"未知 case: {case}")
    if args.check:
        return check(cases)
    arm_ids = [part.strip() for part in args.arms.split(",") if part.strip()]
    known = arms_table()
    for arm_id in arm_ids:
        if arm_id not in known:
            raise SystemExit(f"未知臂: {arm_id}")
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
        for arm_id in arm_ids:
            for seed in seeds:
                if not args.no_resume:
                    cached = _load_partial(case, arm_id, int(seed))
                    if cached is not None:
                        rows.append(cached)
                        continue
                jobs.append({"case": case, "arm": arm_id, "seed": int(seed)})
    if jobs:
        import multiprocessing as mp

        from yopo_swarm_core.baseline import fetch_upstream

        # 权重缓存只建一次。两个进程同时 git init 会抢同一个 .partial。
        fetch_upstream()
        ctx = mp.get_context("spawn")
        # 评估次数预算不随负载变，并行只影响墙钟。墙钟不进对照。
        workers = min(args.jobs, len(jobs))
        with ctx.Pool(workers) as pool:
            rows.extend(pool.map(_worker, jobs))
    rows.sort(key=lambda r: (r["case"], r["arm"], r["seed"]))
    payload = {
        "title": "票 E-B2q：密林开后段评估次数预算诊断",
        "note": (
            "只报数。report-only。未冻结。库缺省在票 E-B2r 之后是 budget=evals、20 次评估。"
            "本诊断的 6/10/20/40 次都写了显式 time_budget_s，原文表格里的数字不改。"
            "off 是后段关的对照。打开后段的臂都标偏离上游。"
            "不用墙钟做预算。c4b014 固定 E-A1 + D1，只变后段次数；b6medium 的开后段臂用 E-B4c 默认球。"
            "截断指后段评估次数上限把这一次求解停下，不是仿真超时。"
        ),
        "commit": _commit(),
        "starting_ref": "765e2cb6a01123a236ff00d3892d8ae99a806a65",
        "budget_mode": "evals",
        "idle_eval_cost_s": IDLE_EVAL_COST_S,
        "default_eval_cap": DEFAULT_EVALS,
        "max_iterations_unchanged": 8,
        "arms": arms_table(),
        "cases": {name: CASES[name] for name in cases},
        "rows": rows,
        "summary": _summarize(rows),
        "per_seed": _per_seed(rows),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(payload["summary"], indent=2, ensure_ascii=False))
    print(f"写入 {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
