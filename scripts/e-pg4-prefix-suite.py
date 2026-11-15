#!/usr/bin/env python3
"""票 E-PG4：把 E-PG3 的 0.7 s 短前缀在已冻 M0–M4 套件上打开重测。只报数。

不改场景卡，不改默认。穿林/柱林在进程里写 task.leader_prefix 与 leader_prefix_s=0.7，
并把门控打开（六机林带和柱林的卡上原本是关的；三机林带卡上本来就是开的）。
编队和单机 goto 不读这个字段，那些行按场景卡默认重跑。
后段不打开。库缺省预算是评估次数；本脚本拒绝 YOPO_BACKEND_BUDGET=wall。
每粒种子写到 out/e-pg4-prefix-suite/partial/，中断后原样再跑会续上。
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import sys
import time
import traceback
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "e-pg4-prefix-suite"
PARTIAL = OUT / "partial"
SPOT_PATH = OUT / "spot-off.json"
JSON_PATH = ROOT / "docs" / "research" / "e-pg4-prefix-suite-runs.json"
MD_PATH = ROOT / "docs" / "research" / "e-pg4-prefix-suite.md"
SUMMARY_PATH = ROOT / "docs" / "results" / "SUMMARY.json"
PG3_PATH = ROOT / "docs" / "research" / "e-pg3-leader-prefix-runs.json"
TIERS_PATH = ROOT / "scenes" / "formation-6-tiers.json"
SPOT_FIXTURE = ROOT / "tests" / "fixtures" / "pg2_prechange_crossing6_forest_seed0_result.json"

# 改开关、种子清单或行格式时一起改，旧 partial 就不会被拿来用。
MODEL_REV = "e-pg4-prefix07-gate-on-2026-11-15"
ROW_VERSION = 1
PREFIX_S = 0.7
# E-PG3 prefix07：门控开，余量缺省 1.0（不写卡），分位 0，短前缀 0.7 s。
LEADER_MARGIN_M = 1.0

DISCLAIMER = (
    "只报数。report-only。task.leader_prefix 默认关。本表把 E-PG3 里 0.7 s 那一档"
    "（progress_gate 开、leader_margin_m 缺省 1.0、progress_percentile 0、"
    "leader_prefix_s=0.7）在进程里打开后重跑已冻 M0–M4，标偏离上游。"
    "不把开关写回场景卡，不改默认，不改 core/、yopo_l0/crossing.py、yopo_l0/progress.py、"
    "scenes/suite.txt、已冻门、docs/tickets.md、README.md、scripts/premerge.sh、"
    "scripts/regress.sh、scripts/sitl/Dockerfile。后段不打开；预算按评估次数，不用墙钟截断。"
    "破门不挡合入。未冻结，不进 scenes/suite.txt，不设新门，不改已冻门。"
)

CASES = (
    {"id": "g2-pillar", "path": "scenes/g2-pillar.json", "policy": "upstream", "seeds": tuple(range(10)), "milestone": "M0", "tiers": False, "prefix": False},
    {"id": "formation-hold", "path": "scenes/formation-hold.json", "policy": "formation", "seeds": tuple(range(3)), "milestone": "M1", "tiers": False, "prefix": False},
    {"id": "crossing-pillars", "path": "scenes/crossing-pillars.json", "policy": "crossing", "seeds": tuple(range(10)), "milestone": "M2", "tiers": False, "prefix": True},
    {"id": "crossing-forest", "path": "scenes/crossing-forest.json", "policy": "crossing", "seeds": tuple(range(10)), "milestone": "M2", "tiers": False, "prefix": True},
    {"id": "formation-switch", "path": "scenes/formation-switch.json", "policy": "switch", "seeds": tuple(range(10)), "milestone": "M3", "tiers": False, "prefix": False},
    {"id": "formation-6-open", "path": "scenes/formation-6-open.json", "policy": "formation", "seeds": tuple(range(5)), "milestone": "M4", "tiers": True, "prefix": False},
    {"id": "crossing-6-forest", "path": "scenes/crossing-6-forest.json", "policy": "crossing", "seeds": tuple(range(10)), "milestone": "M4", "tiers": False, "prefix": True},
    {"id": "crossing-6-forest-sparse", "path": "scenes/crossing-6-forest-sparse.json", "policy": "crossing", "seeds": tuple(range(10)), "milestone": "M4", "tiers": False, "prefix": True},
)

OPEN_AREA = {"formation-hold", "formation-6-open"}
SWITCH_METRICS = (
    "error_p95_before_m",
    "error_p95_after_m",
    "min_inter_drone_transition_m",
    "transition_collisions",
)
WATCHED = (
    "scenes/suite.txt",
    "README.md",
    "scripts/premerge.sh",
    "scripts/regress.sh",
    "scripts/sitl/Dockerfile",
    "docs/tickets.md",
    "yopo_l0/crossing.py",
    "yopo_l0/progress.py",
)


def case_by_id(scene: str) -> dict:
    for case in CASES:
        if case["id"] == scene:
            return case
    raise KeyError(scene)


def load_tiers() -> list[dict]:
    raw = json.loads(TIERS_PATH.read_text(encoding="utf-8"))
    tiers = raw["tiers"] if isinstance(raw, dict) else raw
    if len(tiers) != 6:
        raise RuntimeError(f"formation-6 应有六档，读到 {len(tiers)}")
    return list(tiers)


def assert_eval_budget() -> None:
    """后段预算按评估次数。墙钟模式会让同一粒种子的轨迹随负载变。"""
    raw = os.environ.get("YOPO_BACKEND_BUDGET", "").strip().lower()
    if raw in {"", "evals"}:
        return
    raise RuntimeError(f"后段预算必须按评估次数，拒绝 YOPO_BACKEND_BUDGET={raw!r}")


def params_fingerprint() -> str:
    blob = {
        "model_rev": MODEL_REV,
        "row_version": ROW_VERSION,
        "prefix_s": PREFIX_S,
        "leader_margin_m": LEADER_MARGIN_M,
        "budget": "evals",
        "cases": [
            {
                "id": case["id"],
                "seeds": list(case["seeds"]),
                "tiers": case["tiers"],
                "policy": case["policy"],
                "prefix": case["prefix"],
            }
            for case in CASES
        ],
        "tier_ids": [tier["id"] for tier in load_tiers()],
    }
    raw = json.dumps(blob, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(raw).hexdigest()


def code_fingerprint() -> str:
    """关时抽查只在 yopo_l0 与 core 没变时复用。本票不改这两处。"""
    h = hashlib.sha256()
    for folder in (ROOT / "yopo_l0", ROOT / "core" / "src"):
        for path in sorted(p for p in folder.rglob("*.py") if "__pycache__" not in p.parts):
            h.update(path.relative_to(ROOT).as_posix().encode())
            h.update(b"\0")
            h.update(path.read_bytes())
    return h.hexdigest()


def job_specs(scenes: tuple[str, ...] | None = None, seeds: tuple[int, ...] | None = None, *, resume: bool = True) -> list[tuple]:
    wanted = set(scenes) if scenes is not None else None
    wanted_seeds = None if seeds is None else set(int(s) for s in seeds)
    tiers = load_tiers()
    specs = []
    for case in CASES:
        if wanted is not None and case["id"] not in wanted:
            continue
        use_seeds = [int(s) for s in case["seeds"] if wanted_seeds is None or int(s) in wanted_seeds]
        if case["tiers"]:
            for tier in tiers:
                for seed in use_seeds:
                    specs.append((case["id"], int(seed), str(tier["id"]), bool(resume)))
        else:
            for seed in use_seeds:
                specs.append((case["id"], int(seed), None, bool(resume)))
    return specs


def partial_path(scene: str, seed: int, tier: str | None) -> Path:
    tag = tier or "notier"
    return PARTIAL / f"{scene}__{tag}__seed{int(seed)}.json"


def _jsonable(obj):
    import math

    import numpy as np

    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.floating):
        value = float(obj)
        return None if not math.isfinite(value) else value
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, float):
        return None if not math.isfinite(obj) else obj
    return obj


def load_partial(scene: str, seed: int, tier: str | None) -> dict | None:
    dest = partial_path(scene, seed, tier)
    if not dest.is_file():
        return None
    row = json.loads(dest.read_text(encoding="utf-8"))
    if int(row.get("row_version", 0)) != ROW_VERSION:
        return None
    if row.get("params_fingerprint") != params_fingerprint():
        return None
    return row


def save_partial(row: dict) -> None:
    dest = partial_path(row["scene"], int(row["seed"]), row.get("fault_tier"))
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(_jsonable(row), ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)


def _bus_for(tier: str | None):
    if tier is None:
        return None
    from yopo_l0.bus import BusFaults

    for item in load_tiers():
        if item["id"] == tier:
            return BusFaults.from_mapping(item.get("bus"))
    raise KeyError(tier)


def prepare_run(case: dict):
    """返回 (SceneCard, 是否打开了短前缀)。改动只在内存里。"""
    from yopo_l0.scene import SceneCard, load_card

    loaded = load_card(ROOT / case["path"])
    if not case["prefix"]:
        task = loaded.card["task"]
        if "leader_prefix" in task or "leader_prefix_s" in task:
            raise RuntimeError(f"{case['id']} 的场景卡已经写了 leader_prefix")
        return loaded, False
    if loaded.card["task"].get("type") != "crossing":
        raise RuntimeError(f"{case['id']} 不是 crossing，不能套短前缀")
    card = copy.deepcopy(loaded.card)
    task = card["task"]
    if "leader_margin_m" in task or "leader_prefix" in task or "leader_prefix_s" in task:
        raise RuntimeError(f"{case['id']} 的卡上已经有短前缀或余量字段")
    task["progress_gate"] = True
    task["leader_prefix"] = True
    task["leader_prefix_s"] = PREFIX_S
    return SceneCard(loaded.path, card, loaded.obstacles), True


def _split_collision(result: dict) -> tuple[int, int]:
    block = result.get("crossing") or {}
    if "obstacle_collisions" in block or "inter_drone_collisions" in block:
        return int(block.get("obstacle_collisions") or 0), int(block.get("inter_drone_collisions") or 0)
    reason = str(result.get("reason") or "")
    if result.get("outcome") != "COLLISION":
        return 0, 0
    if "INTER_DRONE" in reason:
        return 0, 1
    return 1, 0


def _decision_count(result: dict, key: str) -> int:
    block = result.get("crossing") or {}
    decisions = block.get("decisions") or result.get("plan_codes") or {}
    return int(decisions.get(key) or 0)


def _check_result(case: dict, result: dict, *, prefix: bool) -> None:
    if result.get("backend") is not None:
        raise RuntimeError(f"{case['id']} 打开了后段，本票不跑后段")
    if prefix:
        if result.get("leader_prefix") is not True or result.get("upstream_deviation") is not True:
            raise RuntimeError(f"{case['id']} 打开短前缀后没有写字段或没有标偏离上游")
        if abs(float(result.get("leader_prefix_s")) - PREFIX_S) > 1e-9:
            raise RuntimeError(f"{case['id']} 前缀长度实际 {result.get('leader_prefix_s')}")
        block = result.get("crossing") or {}
        if block.get("leader_prefix") is not True or block.get("progress_gate") is not True:
            raise RuntimeError(f"{case['id']} 的 crossing 块没有标上短前缀或门控")
        if abs(float(block.get("leader_prefix_s")) - PREFIX_S) > 1e-9:
            raise RuntimeError(f"{case['id']} crossing 前缀长度实际 {block.get('leader_prefix_s')}")
        margin = block.get("leader_margin_m")
        if margin is None or abs(float(margin) - LEADER_MARGIN_M) > 1e-9:
            raise RuntimeError(f"{case['id']} 缺省余量应为 {LEADER_MARGIN_M}，收到 {margin}")
        return
    if "leader_prefix" in result or "leader_prefix_s" in result:
        raise RuntimeError(f"{case['id']} 没有打开短前缀，结果里却有字段")
    block = result.get("crossing") or {}
    if "leader_prefix" in block or "leader_prefix_s" in block:
        raise RuntimeError(f"{case['id']} 没有打开短前缀，crossing 里却有字段")
    if result.get("upstream_deviation") is True:
        raise RuntimeError(f"{case['id']} 没有打开短前缀，却标了偏离上游")


def _simulate(scene: str, seed: int, tier: str | None) -> dict:
    assert_eval_budget()
    from yopo_l0.runner import RunConfig, run

    case = case_by_id(scene)
    card, prefix = prepare_run(case)
    disk = json.loads((ROOT / case["path"]).read_text(encoding="utf-8"))
    if "leader_prefix" in disk.get("task", {}) or "leader_prefix_s" in disk.get("task", {}):
        raise RuntimeError(f"{scene} 的磁盘卡被写上了 leader_prefix")
    if disk.get("criteria", {}).get("frozen") is not True:
        raise RuntimeError(f"{scene} 的 frozen 标志被动过")
    from yopo_l0.scene import load_card

    file_hash = load_card(ROOT / case["path"]).hash
    cfg = RunConfig(
        seed=int(seed),
        policy=case["policy"],
        shadow=True,
        bus=_bus_for(tier),
    )
    out = run(card, cfg)
    res = out.result
    _check_result(case, res, prefix=prefix)
    if not prefix and res.get("scene_card_hash") != file_hash:
        raise RuntimeError(f"{scene} 关着开关时场景卡哈希变了")
    body = float(card.card["criteria"]["body_radius_m"])
    surface = float(res["min_obstacle_distance_m"])
    obstacle, inter = _split_collision(res)
    switch = res.get("switch") or {}
    switch_keep = {key: switch[key] for key in SWITCH_METRICS if key in switch}
    cross = res.get("crossing") or {}
    row = {
        "row_version": ROW_VERSION,
        "params_fingerprint": params_fingerprint(),
        "model_rev": MODEL_REV,
        "scene": scene,
        "milestone": case["milestone"],
        "seed": int(seed),
        "fault_tier": tier,
        "form": "inprocess",
        "policy": res.get("policy"),
        "tier": res.get("tier"),
        "shadow": bool(res.get("shadow")),
        "task": res.get("task"),
        "forest_density": res.get("forest_density"),
        "prefix_applied": prefix,
        "mark": "偏离上游" if prefix else "场景卡默认",
        "upstream_deviation": bool(res.get("upstream_deviation")),
        "leader_prefix": True if prefix else None,
        "leader_prefix_s": PREFIX_S if prefix else None,
        "progress_gate": cross.get("progress_gate") if prefix else None,
        "leader_margin_m": cross.get("leader_margin_m") if prefix else None,
        "leader_prefix_count": _decision_count(res, "LEADER_PREFIX") if prefix else 0,
        "leader_hover": _decision_count(res, "LEADER_HOVER") if prefix else 0,
        "gate_s_m": cross.get("gate_s_m") if prefix else None,
        "outcome": res.get("outcome"),
        "reason": str(res.get("reason") or ""),
        "arrived": int(res.get("arrived") or 0),
        "n_drones": int(res.get("n_drones") or 0),
        "arrival_time_s": res.get("arrival_time_s"),
        "min_obstacle_distance_m": surface,
        "min_clearance_m": surface - body,
        "body_radius_m": body,
        "clearance_is_ground": scene in OPEN_AREA,
        "min_inter_drone_distance_m": res.get("min_inter_drone_distance_m"),
        "formation_error_p95_m": res.get("formation_error_p95_m"),
        "switch": switch_keep or None,
        "obstacle_collision": obstacle,
        "inter_drone_collision": inter,
        "collision": obstacle + inter,
        "sim_duration_s": float(res.get("sim_duration_s") or 0.0),
        "wall_time_s": float(res.get("wall_time_s") or 0.0),
        "scene_card_hash": res.get("scene_card_hash"),
        "file_card_hash": file_hash,
        "commit": res.get("commit"),
        "backend_enabled": False,
        "backend_budget": "evals",
    }
    if prefix:
        # 碰撞门按 crossing 块的计数，和 SUMMARY 同一条路径。
        row["crossing"] = {
            "obstacle_collisions": obstacle,
            "inter_drone_collisions": inter,
        }
    return _jsonable(row)


def run_one(scene: str, seed: int, tier: str | None, *, resume: bool) -> dict:
    if resume:
        cached = load_partial(scene, seed, tier)
        if cached is not None:
            label = f"{scene} {tier or '-'} seed={seed}"
            print(f"{label} resume", flush=True)
            return cached
    row = _simulate(scene, seed, tier)
    save_partial(row)
    arrival = row.get("arrival_time_s")
    arrival_s = "—" if arrival is None else f"{float(arrival):.2f}"
    print(
        f"{scene} {tier or '-'} seed={row['seed']} {row['outcome']} "
        f"{row['arrived']}/{row['n_drones']} coll={row['collision']} "
        f"clear={float(row['min_clearance_m']):.3f} t={arrival_s} "
        f"prefix={row['leader_prefix_count']} "
        f"sim={float(row['sim_duration_s']):.2f} wall={float(row['wall_time_s']):.1f}s",
        flush=True,
    )
    return row


def _worker(spec: tuple) -> dict:
    try:
        os.nice(10)
    except OSError:
        pass
    scene, seed, tier, resume = spec
    try:
        return run_one(str(scene), int(seed), tier, resume=bool(resume))
    except Exception:
        print(f"FAIL {scene} {tier} seed={seed}", flush=True)
        traceback.print_exc()
        raise


def _run_group(specs: list[tuple], jobs: int) -> list[dict]:
    if jobs <= 1 or len(specs) <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)


def _diff_json(got, exp, prefix=""):
    diffs = []
    if isinstance(got, dict) and isinstance(exp, dict):
        for key in sorted(set(got) | set(exp)):
            path = f"{prefix}.{key}" if prefix else str(key)
            if key not in got or key not in exp:
                diffs.append((path, got.get(key, "<missing>"), exp.get(key, "<missing>")))
            else:
                diffs.extend(_diff_json(got[key], exp[key], path))
        return diffs
    if isinstance(got, list) and isinstance(exp, list):
        if got != exp:
            diffs.append((prefix, got, exp))
        return diffs
    if isinstance(got, (int, float)) and isinstance(exp, (int, float)) and not isinstance(got, bool) and not isinstance(exp, bool):
        if abs(float(got) - float(exp)) > 1e-9:
            diffs.append((prefix, got, exp))
        return diffs
    if got != exp:
        diffs.append((prefix, got, exp))
    return diffs


def spot_off(*, resume: bool) -> dict:
    """关时抽查已冻 crossing-6-forest seed 0，与当前 main 的结果逐位一致。

    比较时去掉墙钟和仓库 SHA，与 E-PG3 同一条夹具。
    """
    finger = code_fingerprint()
    if resume and SPOT_PATH.is_file():
        stored = json.loads(SPOT_PATH.read_text(encoding="utf-8"))
        if stored.get("matched") is True and stored.get("code_fingerprint") == finger:
            print("spot-off resume matched", flush=True)
            return stored
    from yopo_l0.runner import WALL_FIELDS, RunConfig, run
    from yopo_l0.scene import load_card

    pre = json.loads(SPOT_FIXTURE.read_text(encoding="utf-8"))
    if pre.get("scene") != "crossing-6-forest" or int(pre.get("seed")) != 0:
        raise RuntimeError("抽查夹具不是 crossing-6-forest seed 0")
    if "leader_prefix" in pre or "leader_prefix" in (pre.get("crossing") or {}):
        raise RuntimeError("抽查夹具里已经有 leader_prefix")
    card = load_card(ROOT / "scenes/crossing-6-forest.json")
    if card.card["task"].get("progress_gate") is not False or "leader_prefix" in card.card["task"]:
        raise RuntimeError("六机林带卡的默认被改过")
    print("spot-off crossing-6-forest seed=0 switch off", flush=True)
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=0, policy="crossing", shadow=True))
    elapsed = time.perf_counter() - t0
    if "leader_prefix" in out.result or "leader_prefix" in out.result.get("crossing", {}):
        raise RuntimeError("开关关着，结果里却有 leader_prefix")
    if out.result.get("upstream_deviation") is True:
        raise RuntimeError("开关关着却标了偏离上游")
    strip = set(WALL_FIELDS) | {"commit"}
    got = {k: v for k, v in json.loads(json.dumps(_jsonable(out.result))).items() if k not in strip}
    exp = {k: v for k, v in pre.items() if k not in strip}
    diffs = _diff_json(got, exp)
    record = {
        "scene": "crossing-6-forest",
        "seed": 0,
        "matched": not diffs,
        "n_diffs": len(diffs),
        "diffs_head": [
            {"path": path, "got": _jsonable(a), "expected": _jsonable(b)} for path, a, b in diffs[:30]
        ],
        "stripped": sorted(strip),
        "fixture": str(SPOT_FIXTURE.relative_to(ROOT)),
        "code_fingerprint": finger,
        "wall_time_s": elapsed,
        "arrived": out.result.get("arrived"),
        "outcome": out.result.get("outcome"),
        "upstream_deviation": out.result.get("upstream_deviation"),
    }
    SPOT_PATH.parent.mkdir(parents=True, exist_ok=True)
    SPOT_PATH.write_text(json.dumps(_jsonable(record), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if diffs:
        preview = "; ".join(f"{path}" for path, _a, _b in diffs[:12])
        raise RuntimeError(f"关时 crossing-6-forest seed 0 与 main 不一致，差 {len(diffs)} 处：{preview}")
    print(f"spot-off matched main wall={elapsed:.1f}s", flush=True)
    return record


def watched_snapshot() -> dict[str, str]:
    files = list(WATCHED) + [case["path"] for case in CASES]
    out = {}
    for rel in files:
        out[rel] = hashlib.sha256((ROOT / rel).read_bytes()).hexdigest()
    h = hashlib.sha256()
    for path in sorted(p for p in (ROOT / "core").rglob("*") if p.is_file() and "__pycache__" not in p.parts):
        h.update(path.relative_to(ROOT).as_posix().encode())
        h.update(b"\0")
        h.update(path.read_bytes())
    out["core/"] = h.hexdigest()
    return out


def assert_watched(before: dict[str, str]) -> None:
    after = watched_snapshot()
    changed = [key for key in before if before[key] != after.get(key)]
    if changed:
        raise RuntimeError("禁止改动的文件变了：" + ", ".join(changed))


def _broken_names(gates: dict, rows: list[dict]) -> list[str]:
    from yopo_l0.criteria import _GATE_KIND, _holds, collision_count, metric_value

    broken = []
    for name, threshold in gates.items():
        kind = _GATE_KIND[name]
        failed = False
        for row in rows:
            value = metric_value(row, name)
            if value is None or not _holds(kind, float(value), float(threshold)):
                failed = True
                break
        if not failed and name == "collisions":
            total = sum(collision_count(row) for row in rows)
            failed = not _holds(kind, float(total), float(threshold))
        if not failed and name == "arrival_rate":
            arrived = sum(int(row.get("arrived") or 0) for row in rows)
            slots = sum(int(row.get("n_drones") or 0) for row in rows)
            overall = None if not slots else arrived / slots
            failed = overall is None or not _holds(kind, float(overall), float(threshold))
        if failed:
            broken.append(name)
    return broken


def evaluate_case(case: dict, rows: list[dict]) -> tuple[dict | None, list[str]]:
    from yopo_l0.criteria import active_gates, evaluate_gates
    from yopo_l0.scene import load_card

    if not rows:
        return None, [f"{case['id']}：没有结果"]
    card = load_card(ROOT / case["path"])
    sample = rows[0]
    density = sample.get("forest_density")
    if density is None:
        density = (card.card.get("forest") or {}).get("density")
    gates = active_gates(
        card,
        policy=sample.get("policy") or case["policy"],
        form=sample.get("form") or "inprocess",
        density=density,
    )
    if not gates:
        return None, []
    label = case["id"] if not sample.get("fault_tier") else f"{case['id']} {sample['fault_tier']}"
    return gates, evaluate_gates(rows, gates, label=label)


def _range(values: list[float]) -> dict | None:
    if not values:
        return None
    return {"min": min(values), "max": max(values), "mean": sum(values) / len(values), "n": len(values)}


def _fmt_range(span: dict | None, nd: int = 3) -> str:
    if not span:
        return "—"
    lo, hi = float(span["min"]), float(span["max"])
    if abs(hi - lo) < 0.5 * 10 ** (-nd):
        return f"{lo:.{nd}f}"
    return f"{lo:.{nd}f}–{hi:.{nd}f}"


def _fmt_opt(value, nd: int = 3) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{nd}f}"


def _fmt_time(span: dict | None, n_runs: int) -> str:
    if not span:
        return f"—（0/{n_runs} 次有全体到达）"
    return (
        f"{float(span['mean']):.3f}（{float(span['min']):.2f}–{float(span['max']):.2f}，"
        f"{int(span['n'])}/{n_runs} 次有全体到达）"
    )


def _fmt_clear(value, ground: bool) -> str:
    text = _fmt_opt(value)
    if ground and value is not None:
        return f"离地 {text}"
    return text


def _fmt_p95(stats: dict) -> str:
    overall = _fmt_range(stats.get("formation_error_p95_m"))
    before = stats.get("error_p95_before_m")
    after = stats.get("error_p95_after_m")
    if before or after:
        return f"前 {_fmt_range(before)} / 后 {_fmt_range(after)}（全程 {overall}）"
    return overall


def _fmt_inter(stats: dict) -> str:
    text = _fmt_opt(stats.get("min_inter_drone_distance_m"))
    trans = stats.get("min_inter_drone_transition_m")
    if isinstance(trans, dict):
        shown = _fmt_opt(trans.get("min"))
        if shown != "—":
            return f"{text}（过渡 {shown}）"
    elif trans is not None:
        return f"{text}（过渡 {_fmt_opt(trans)}）"
    return text


def _rerun_stats(rows: list[dict]) -> dict:
    p95s = [float(r["formation_error_p95_m"]) for r in rows if r.get("formation_error_p95_m") is not None]
    inters = [float(r["min_inter_drone_distance_m"]) for r in rows if r.get("min_inter_drone_distance_m") is not None]
    clears = [float(r["min_clearance_m"]) for r in rows if r.get("min_clearance_m") is not None]
    times = [float(r["arrival_time_s"]) for r in rows if r.get("arrival_time_s") is not None]
    before = [float(v) for r in rows if (v := (r.get("switch") or {}).get("error_p95_before_m")) is not None]
    after = [float(v) for r in rows if (v := (r.get("switch") or {}).get("error_p95_after_m")) is not None]
    trans = [float(v) for r in rows if (v := (r.get("switch") or {}).get("min_inter_drone_transition_m")) is not None]
    arrived = sum(int(r.get("arrived") or 0) for r in rows)
    slots = sum(int(r.get("n_drones") or 0) for r in rows)
    return {
        "arrivals": f"{arrived}/{slots}" if slots else "—",
        "arrived": arrived,
        "slots": slots,
        "collisions": sum(int(r.get("collision") or 0) for r in rows),
        "obstacle_collisions": sum(int(r.get("obstacle_collision") or 0) for r in rows),
        "inter_drone_collisions": sum(int(r.get("inter_drone_collision") or 0) for r in rows),
        "formation_error_p95_m": _range(p95s),
        "error_p95_before_m": _range(before),
        "error_p95_after_m": _range(after),
        "min_inter_drone_distance_m": None if not inters else min(inters),
        "min_inter_drone_transition_m": None if not trans else min(trans),
        "min_clearance_m": None if not clears else min(clears),
        "clearance_is_ground": bool(rows[0].get("clearance_is_ground")),
        "arrival_time_s": _range(times),
        "n_runs": len(rows),
        "leader_prefix_count": sum(int(r.get("leader_prefix_count") or 0) for r in rows),
        "leader_hover": sum(int(r.get("leader_hover") or 0) for r in rows),
        "prefix_applied": bool(rows[0].get("prefix_applied")),
        "mark": rows[0].get("mark"),
    }


def _summary_by_key() -> dict[tuple, dict]:
    payload = json.loads(SUMMARY_PATH.read_text(encoding="utf-8"))
    out = {}
    for row in payload["rows"]:
        out[(row["scene"], row.get("fault_tier"))] = row
    return out


def _summary_view(row: dict) -> dict:
    return {
        "arrivals": row.get("arrivals"),
        "collisions": row.get("collisions"),
        "formation_error_p95_m": row.get("formation_error_p95_m"),
        "error_p95_before_m": row.get("error_p95_before_m"),
        "error_p95_after_m": row.get("error_p95_after_m"),
        "min_inter_drone_distance_m": row.get("min_inter_drone_distance_m"),
        "min_inter_drone_transition_m": row.get("min_inter_drone_transition_m"),
        "min_clearance_m": row.get("min_obstacle_clearance_m"),
        "clearance_is_ground": bool(row.get("clearance_is_ground")),
        "arrival_time_s": row.get("arrival_time_s"),
        "transition_collisions": row.get("transition_collisions"),
        "n_runs": row.get("n_seeds"),
        "seeds": row.get("seeds"),
        "gates": row.get("gates"),
        "status": row.get("status"),
        "scene_card_hash": row.get("scene_card_hash"),
    }


def _summary_gate_broken(view: dict) -> list[str]:
    gates = view.get("gates") or {}
    broken = []

    def upper(name: str, span) -> None:
        if name not in gates:
            return
        if not isinstance(span, dict) or float(span["max"]) - 1e-9 > float(gates[name]):
            broken.append(name)

    def lower_value(name: str, value) -> None:
        if name not in gates:
            return
        if value is None or float(value) + 1e-9 < float(gates[name]):
            broken.append(name)

    arrivals = str(view.get("arrivals") or "")
    if "arrival_rate" in gates:
        if "/" not in arrivals:
            broken.append("arrival_rate")
        else:
            num, den = arrivals.split("/", 1)
            rate = None if float(den) == 0.0 else float(num) / float(den)
            if rate is None or rate + 1e-9 < float(gates["arrival_rate"]):
                broken.append("arrival_rate")
    if "collisions" in gates and float(view.get("collisions") or 0) - 1e-9 > float(gates["collisions"]):
        broken.append("collisions")
    if "transition_collisions" in gates:
        value = view.get("transition_collisions")
        if value is None or float(value) - 1e-9 > float(gates["transition_collisions"]):
            broken.append("transition_collisions")
    upper("arrival_time_s", view.get("arrival_time_s"))
    upper("formation_error_p95_m", view.get("formation_error_p95_m"))
    upper("error_p95_before_m", view.get("error_p95_before_m"))
    upper("error_p95_after_m", view.get("error_p95_after_m"))
    lower_value("min_inter_drone_distance_m", view.get("min_inter_drone_distance_m"))
    trans = view.get("min_inter_drone_transition_m")
    if isinstance(trans, dict):
        lower_value("min_inter_drone_transition_m", trans.get("min"))
    else:
        lower_value("min_inter_drone_transition_m", trans)
    lower_value("min_obstacle_clearance_m", view.get("min_clearance_m"))
    return broken


def _coll_text(stats: dict) -> str:
    n = int(stats["collisions"])
    if n == 0:
        return str(n)
    return f"{n}（障碍 {int(stats['obstacle_collisions'])}，机间 {int(stats['inter_drone_collisions'])}）"


def _group_rows(rows: list[dict]) -> list[tuple[dict, str | None, list[dict]]]:
    grouped: dict[tuple, list[dict]] = {}
    order = []
    for row in rows:
        key = (row["scene"], row.get("fault_tier"))
        if key not in grouped:
            grouped[key] = []
            order.append(key)
        grouped[key].append(row)
    out = []
    for scene, tier in order:
        group = sorted(grouped[(scene, tier)], key=lambda r: int(r["seed"]))
        out.append((case_by_id(scene), tier, group))
    return out


def build_comparisons(rows: list[dict]) -> list[dict]:
    summary = _summary_by_key()
    comparisons = []
    for case, tier, group in _group_rows(rows):
        key = (case["id"], tier)
        base = summary.get(key)
        if base is None:
            raise RuntimeError(f"SUMMARY 没有 {case['id']} tier={tier}")
        gates, failures = evaluate_case(case, group)
        broken = _broken_names(gates, group) if gates else []
        file_hashes = sorted({str(r.get("file_card_hash") or "") for r in group})
        comparisons.append(
            {
                "scene": case["id"],
                "milestone": case["milestone"],
                "fault_tier": tier,
                "policy": group[0].get("policy"),
                "forest_density": group[0].get("forest_density"),
                "prefix_applied": bool(group[0].get("prefix_applied")),
                "mark": group[0].get("mark"),
                "seeds": [int(r["seed"]) for r in group],
                "gates": gates,
                "gate_failures": failures,
                "broken_gates": broken,
                "file_card_hash": file_hashes[0] if len(file_hashes) == 1 else file_hashes,
                "point_mass": _summary_view(base),
                "rerun": _rerun_stats(group),
            }
        )
    return comparisons


def _scene_label(item: dict) -> str:
    name = item["scene"]
    density = item.get("forest_density")
    if density:
        name = f"{name}（{density}）"
    return name


def _tier_label(tier: str | None) -> str:
    return "—" if tier is None else tier


def _config_label(item: dict) -> str:
    if item.get("prefix_applied"):
        return "0.7 s 短前缀（偏离上游）"
    return "场景卡默认（短前缀不作用）"


def _pg3_prefix07() -> dict[tuple[str, int], dict]:
    payload = json.loads(PG3_PATH.read_text(encoding="utf-8"))
    out = {}
    for row in payload["rows"]:
        if row.get("arm") != "prefix07":
            continue
        out[(str(row["scene"]), int(row["seed"]))] = row
    return out


def _same_num(a, b, tol: float = 1e-6) -> bool:
    if a is None or b is None:
        return a is None and b is None
    return abs(float(a) - float(b)) <= tol


def pg3_confirm(rows: list[dict]) -> list[dict]:
    """六机林带与 E-PG3 prefix07 对照。本环境重跑，不抄数。"""
    ref = _pg3_prefix07()
    out = []
    for row in rows:
        if row["scene"] not in {"crossing-6-forest", "crossing-6-forest-sparse"}:
            continue
        if not row.get("prefix_applied"):
            continue
        other = ref.get((row["scene"], int(row["seed"])))
        if other is None:
            raise RuntimeError(f"E-PG3 没有 {row['scene']} seed {row['seed']} 的 prefix07")
        fields = {
            "arrived": (int(row["arrived"]), int(other["arrived"])),
            "obstacle_collision": (int(row["obstacle_collision"]), int(other["tree_collisions"])),
            "inter_drone_collision": (int(row["inter_drone_collision"]), int(other["inter_drone_collisions"])),
            "formation_error_p95_m": (row.get("formation_error_p95_m"), other.get("formation_error_p95_m")),
            "min_inter_drone_distance_m": (row.get("min_inter_drone_distance_m"), other.get("min_inter_drone_distance_m")),
            "min_clearance_m": (row.get("min_clearance_m"), other.get("min_clearance_m")),
            "arrival_time_s": (row.get("arrival_time_s"), other.get("arrival_time_s")),
            "leader_prefix_count": (int(row.get("leader_prefix_count") or 0), int(other.get("leader_prefix_count") or 0)),
        }
        mismatched = [name for name, (a, b) in fields.items() if not _same_num(a, b)]
        out.append(
            {
                "scene": row["scene"],
                "seed": int(row["seed"]),
                "matched": not mismatched,
                "mismatched": mismatched,
                "ours": {name: pair[0] for name, pair in fields.items()},
                "pg3": {name: pair[1] for name, pair in fields.items()},
            }
        )
    return out


def _hash_note(comparisons: list[dict]) -> str:
    bits = []
    for item in comparisons:
        if item.get("fault_tier") not in (None, "none"):
            continue
        got = item.get("file_card_hash")
        exp = (item.get("point_mass") or {}).get("scene_card_hash")
        if isinstance(got, str) and isinstance(exp, str) and got != exp:
            bits.append(f"{item['scene']} 文件哈希 {got[:12]} 与 SUMMARY {exp[:12]} 不同")
    if not bits:
        return "这八张卡的文件哈希与 SUMMARY 的 scene_card_hash 一致。"
    return "场景卡哈希有差：" + "；".join(bits) + "。"


def _conclusion(comparisons: list[dict], confirm: list[dict]) -> list[str]:
    broken = [item for item in comparisons if item["gate_failures"]]
    held = [item for item in comparisons if item.get("gates") and not item["gate_failures"]]
    lines = ["## 结论", ""]
    if not broken:
        lines.append(f"这 {len(comparisons)} 行的冻结门在这次重跑里都还成立。")
    else:
        bits = []
        for item in broken:
            label = f"{item['milestone']} {_scene_label(item)}"
            if item.get("fault_tier"):
                label += f" / {item['fault_tier']}"
            rerun = item["rerun"]
            pm = item["point_mass"]
            names = "、".join(item["broken_gates"]) or "见上面的句子"
            how = "短前缀打开" if item.get("prefix_applied") else "短前缀不作用"
            bits.append(
                f"{label}（{how}）破了 {names}：到达 {rerun['arrivals']}（点质量 {pm['arrivals']}），"
                f"碰撞 {rerun['collisions']}（点质量 {pm['collisions']}）。"
            )
        lines.append(f"冻结门破了 {len(broken)} 行，其余 {len(held)} 行仍过。" + "".join(bits))
    opened = [item for item in comparisons if item.get("prefix_applied")]
    idle = [item for item in comparisons if not item.get("prefix_applied")]
    lines.append(
        f"短前缀打开的是 {len(opened)} 行穿林/柱林，结果标偏离上游。"
        f"另外 {len(idle)} 行是编队或单机 goto，不读 task.leader_prefix，按场景卡默认重跑，不标偏离上游。"
    )
    if confirm:
        bad = [item for item in confirm if not item["matched"]]
        if not bad:
            lines.append(
                "两张六机林带 seeds 0–9 在本环境重跑后，到达、碰撞、队形 p95、最小机间、最小净空、到达时间、"
                "LEADER_PREFIX 次数与 E-PG3 prefix07 一致。这是重跑，不是从 E-PG3 抄来的。"
            )
        else:
            lines.append(
                f"两张六机林带相对 E-PG3 prefix07 有 {len(bad)} 个种子不一致，见上面的对照。数字以本环境重跑为准。"
            )
    lines.append("这些数只报数，不改默认，也不改已冻门。破了的门不挡合入。")
    lines.append("")
    return lines


def render_markdown(payload: dict) -> str:
    lines = [
        "# 票 E-PG4：0.7 s 短前缀在冻结套件上重测（只报数）",
        "",
        DISCLAIMER,
        "",
        "点质量一列来自 [SUMMARY](../results/SUMMARY.md)。"
        "短前缀一列是本票打开开关后的重跑；编队和单机那几行开关不作用，重跑的是场景卡默认。"
        "净空 = 机体中心到障碍表面的距离 − 0.35 m。开阔区写「离地」。"
        "队形误差 p95 多个 seed 写出最小到最大；最小机间和最小净空取最紧的那一次。"
        "到达时间是有全体到达的运行的 `arrival_time_s` 均值，括号里是最小到最大，以及有全体到达的运行次数。"
        "墙钟不进门。后段没有打开，预算模式是评估次数。",
        "",
        _hash_note(payload["comparisons"]),
        "",
        "## 开关",
        "",
        "已有接口是场景卡 `task.leader_prefix` / `task.leader_prefix_s`，由 `CrossingFleet` 读取。"
        "默认关。关时不写这两个字段，超前机仍先悬停，悬停接不上才退到 0.3 s。"
        f"本票在进程里对四张 crossing 卡写成 `leader_prefix=true`、`leader_prefix_s={PREFIX_S}`，"
        "并把 `progress_gate` 设为 true。`leader_margin_m` 不写，缺省 1.0。`progress_percentile` 不改，缺省 0。"
        "这是 E-PG3 prefix07 那一档：两张六机林带在 50 s 内 60/60 到达、碰撞 0、机间不低于 0.70 m。"
        "磁盘上的场景卡不变。编队卡若把门控写成 true，`FormationFleet` 会拒绝，所以那几行保持场景卡默认。",
        "",
        "打开后结果写 `leader_prefix`、`leader_prefix_s`，并标「偏离上游」。关时这两个字段都不出现。",
        "",
        f"模型修订 `{payload['model_rev']}`。跑数 commit `{payload.get('commit')}`。",
        "",
        "## 关时抽查",
        "",
    ]
    spot = payload.get("spot_off") or {}
    if spot.get("matched"):
        lines.append(
            "已冻 `crossing-6-forest` seed 0、开关关、影子盾开，与当前 main 的结果逐位一致。"
            f"比较时去掉了 {', '.join(spot.get('stripped') or [])}。"
            f"夹具是 `{spot.get('fixture')}`。到达 {spot.get('arrived')}/6，结果 `{spot.get('outcome')}`，"
            "结果里没有 `leader_prefix`，也没有标偏离上游。"
        )
    else:
        lines.append("关时抽查没有通过。见 `spot_off`。")
    lines.append("")
    lines += [
        "## 和 SUMMARY 逐行对照",
        "",
        "| 里程碑 | 场景 | 档 | 配置 | 到达 | 碰撞 | 最小机间 m | 最小净空 m | 队形误差 p95 m | 到达时间 s | 冻结门 |",
        "|---|---|---|---|---:|---:|---|---|---|---|---|",
    ]
    for item in payload["comparisons"]:
        pm = item["point_mass"]
        rerun = item["rerun"]
        scene = _scene_label(item)
        tier = _tier_label(item.get("fault_tier"))
        milestone = item["milestone"]
        pm_time = pm.get("arrival_time_s")
        if isinstance(pm_time, dict) and "n" not in pm_time:
            pm_time = {**pm_time, "n": pm.get("n_runs")}
        pm_broken = _summary_gate_broken(pm)
        pm_gate = "通过" if not pm_broken else "破：" + "、".join(pm_broken)
        gate = "通过" if not item["gate_failures"] else "破：" + "、".join(item["broken_gates"] or ["见下列出的句子"])
        lines.append(
            "| {ms} | {scene} | {tier} | 点质量（SUMMARY） | {arr} | {col} | {inter} | {clear} | {p95} | {time} | {gate} |".format(
                ms=milestone,
                scene=scene,
                tier=tier,
                arr=pm["arrivals"],
                col=pm["collisions"],
                inter=_fmt_inter(pm),
                clear=_fmt_clear(pm.get("min_clearance_m"), pm.get("clearance_is_ground")),
                p95=_fmt_p95(pm),
                time=_fmt_time(pm_time, int(pm.get("n_runs") or 0)),
                gate=pm_gate,
            )
        )
        lines.append(
            "| {ms} | {scene} | {tier} | {cfg} | {arr} | {col} | {inter} | {clear} | {p95} | {time} | {gate} |".format(
                ms=milestone,
                scene=scene,
                tier=tier,
                cfg=_config_label(item),
                arr=rerun["arrivals"],
                col=_coll_text(rerun),
                inter=_fmt_inter(rerun),
                clear=_fmt_clear(rerun.get("min_clearance_m"), rerun.get("clearance_is_ground")),
                p95=_fmt_p95(rerun),
                time=_fmt_time(rerun.get("arrival_time_s"), int(rerun["n_runs"])),
                gate=gate,
            )
        )
    lines.append("")
    lines += ["## 哪些冻结门会破", ""]
    broken = [item for item in payload["comparisons"] if item["gate_failures"]]
    held = [item for item in payload["comparisons"] if item["gates"] and not item["gate_failures"]]
    if not broken:
        lines.append("这 13 行在这次重跑下，场景卡里的冻结门都还成立。")
    else:
        lines.append("下面这些行的冻结门在这次重跑里不成立。点质量在 SUMMARY 里这些门是过的。破门不挡合入。")
        lines.append("")
        for item in broken:
            label = f"{item['milestone']} {_scene_label(item)} / {_tier_label(item.get('fault_tier'))}"
            how = "0.7 s 短前缀（偏离上游）" if item.get("prefix_applied") else "场景卡默认（短前缀不作用）"
            lines.append(f"### {label}")
            lines.append("")
            lines.append(f"配置：{how}。门槛：`{json.dumps(item['gates'], ensure_ascii=False)}`。")
            lines.append("")
            for msg in item["gate_failures"]:
                lines.append(f"- {msg}")
            lines.append("")
    if held:
        names = "、".join(f"{_scene_label(item)}/{_tier_label(item.get('fault_tier'))}" for item in held)
        lines.append(f"仍然成立的冻结行：{names}。")
    lines.append("")
    lines += [
        "## 六机林带与 E-PG3 对照",
        "",
        "E-PG3 prefix07 已经报过这两张卡 seeds 0–9。这里在本票环境重跑，用来确认，不把那张表抄进来当结果。",
        "",
    ]
    confirm = payload.get("pg3_confirm") or []
    bad = [item for item in confirm if not item["matched"]]
    if confirm and not bad:
        lines.append(
            f"{len(confirm)} 个种子的到达、障碍碰撞、机间碰撞、队形 p95、最小机间、最小净空、到达时间、"
            "LEADER_PREFIX 次数都与 E-PG3 prefix07 一致。"
        )
    elif bad:
        lines.append(f"{len(bad)} 个种子与 E-PG3 prefix07 不一致：")
        lines.append("")
        lines.append("| 场景 | seed | 不一致的字段 | 本票 | E-PG3 |")
        lines.append("|---|---:|---|---|---|")
        for item in bad:
            for name in item["mismatched"]:
                lines.append(
                    f"| {item['scene']} | {item['seed']} | {name} | {item['ours'].get(name)} | {item['pg3'].get(name)} |"
                )
    else:
        lines.append("没有六机林带的重跑行。")
    lines.append("")
    lines.extend(_conclusion(payload["comparisons"], confirm))
    lines += [
        "## 分种子",
        "",
        "| 场景 | 档 | seed | 配置 | 结果 | 到达 | 碰撞 | 最小净空 m | 最小机间 m | 队形 p95 m | 到达时间 s |",
        "|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in payload["rows"]:
        p95 = row.get("formation_error_p95_m")
        p95_s = "—" if p95 is None else f"{float(p95):.3f}"
        switch = row.get("switch") or {}
        if switch.get("error_p95_before_m") is not None or switch.get("error_p95_after_m") is not None:
            before = switch.get("error_p95_before_m")
            after = switch.get("error_p95_after_m")
            p95_s = (
                f"前 {'—' if before is None else f'{float(before):.3f}'} / "
                f"后 {'—' if after is None else f'{float(after):.3f}'}（全程 {p95_s}）"
            )
        inter = row.get("min_inter_drone_distance_m")
        inter_s = "—" if inter is None else f"{float(inter):.3f}"
        if switch.get("min_inter_drone_transition_m") is not None:
            inter_s = f"{inter_s}（过渡 {float(switch['min_inter_drone_transition_m']):.3f}）"
        arrival = row.get("arrival_time_s")
        arrival_s = "—" if arrival is None else f"{float(arrival):.2f}"
        cfg = "0.7 s" if row.get("prefix_applied") else "默认"
        lines.append(
            "| {scene} | {tier} | {seed} | {cfg} | {outcome} | {arr} | {col} | {clear} | {inter} | {p95} | {arrival} |".format(
                scene=row["scene"],
                tier=_tier_label(row.get("fault_tier")),
                seed=int(row["seed"]),
                cfg=cfg,
                outcome=row.get("outcome"),
                arr=f"{row['arrived']}/{row['n_drones']}",
                col=int(row.get("collision") or 0),
                clear=f"{float(row['min_clearance_m']):.3f}",
                inter=inter_s,
                p95=p95_s,
                arrival=arrival_s,
            )
        )
    lines += [
        "",
        "## 复现",
        "",
        "```",
        ".venv/bin/python scripts/e-pg4-prefix-suite.py --jobs 2",
        ".venv/bin/python -m pytest tests/test_e_pg4_prefix_suite.py -q",
        "PYTHONPATH=.:core/src .venv/bin/python -m pytest -q -n 4 -m 'not slow' tests",
        "```",
        "",
        "缺省从 `out/e-pg4-prefix-suite/partial/` 续跑。`--no-resume` 忽略已有 partial。"
        "开关或种子清单变了，指纹对不上的 partial 不会被拿来用。"
        "关时抽查写在 `out/e-pg4-prefix-suite/spot-off.json`，代码指纹没变就不再跑。"
        "不要设置 `YOPO_BACKEND_BUDGET=wall`。本套件不打开后段。",
        "",
    ]
    return "\n".join(lines)


def _case_index(scene: str) -> int:
    for i, case in enumerate(CASES):
        if case["id"] == scene:
            return i
    return len(CASES)


def _tier_index(tier: str | None) -> int:
    if tier is None:
        return -1
    ids = [item["id"] for item in load_tiers()]
    try:
        return ids.index(tier)
    except ValueError:
        return len(ids)


def build_payload(rows: list[dict], *, jobs: int, spot: dict | None) -> dict:
    rows = sorted(rows, key=lambda r: (_case_index(r["scene"]), _tier_index(r.get("fault_tier")), int(r["seed"])))
    commits = sorted({str(r.get("commit") or "") for r in rows})
    confirm = pg3_confirm(rows)
    return {
        "title": "票 E-PG4：0.7 s 短前缀在冻结套件上重测，只报数",
        "disclaimer": DISCLAIMER,
        "report_only": True,
        "frozen": False,
        "mark": "偏离上游",
        "model_rev": MODEL_REV,
        "row_version": ROW_VERSION,
        "params_fingerprint": params_fingerprint(),
        "prefix_s": PREFIX_S,
        "leader_margin_m": LEADER_MARGIN_M,
        "backend_budget": "evals",
        "commit": commits[0] if len(commits) == 1 else None,
        "commits": commits,
        "summary_commit": "5524cab42584ea54df54bbf38fc77e26117066dd",
        "pg3_source": "docs/research/e-pg3-leader-prefix-runs.json",
        "jobs": int(jobs),
        "n_runs": len(rows),
        "spot_off": spot,
        "pg3_confirm": confirm,
        "comparisons": build_comparisons(rows),
        "rows": rows,
    }


def write_docs(payload: dict) -> None:
    JSON_PATH.parent.mkdir(parents=True, exist_ok=True)
    JSON_PATH.write_text(json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    MD_PATH.write_text(render_markdown(payload), encoding="utf-8")


def run_check() -> int:
    from yopo_l0.progress import read_leader_prefix

    assert_eval_budget()
    enabled, length = read_leader_prefix({})
    if enabled or abs(length - 0.3) > 1e-12:
        raise RuntimeError(f"短前缀默认应为关、0.3 s，读到 {enabled} {length}")
    if abs(PREFIX_S - 0.7) > 1e-12:
        raise RuntimeError("本票的前缀长度应为 0.7 s")
    if len(job_specs()) != 93:
        raise RuntimeError(f"套件应是 93 次运行，得到 {len(job_specs())}")
    prefix_ids = [case["id"] for case in CASES if case["prefix"]]
    if prefix_ids != ["crossing-pillars", "crossing-forest", "crossing-6-forest", "crossing-6-forest-sparse"]:
        raise RuntimeError(f"短前缀只应开在四张 crossing 卡上，得到 {prefix_ids}")
    for case in CASES:
        raw = json.loads((ROOT / case["path"]).read_text(encoding="utf-8"))
        task = raw.get("task") or {}
        if "leader_prefix" in task or "leader_prefix_s" in task or "leader_margin_m" in task:
            raise RuntimeError(f"{case['id']} 的场景卡写了短前缀或余量")
        if raw.get("criteria", {}).get("frozen") is not True:
            raise RuntimeError(f"{case['id']} 的 frozen 标志被动过")
        card, applied = prepare_run(case)
        if applied is not case["prefix"]:
            raise RuntimeError(f"{case['id']} 的开关准备和清单不一致")
        disk = json.loads((ROOT / case["path"]).read_text(encoding="utf-8"))
        if disk != raw:
            raise RuntimeError(f"{case['id']} 准备开关时改了磁盘")
        if applied:
            if card.card["task"].get("leader_prefix") is not True:
                raise RuntimeError(f"{case['id']} 内存卡没有打开 leader_prefix")
            if abs(float(card.card["task"]["leader_prefix_s"]) - PREFIX_S) > 1e-12:
                raise RuntimeError(f"{case['id']} 内存卡前缀长度不对")
            if card.card["task"].get("progress_gate") is not True:
                raise RuntimeError(f"{case['id']} 内存卡门控没有打开")
            if "leader_margin_m" in card.card["task"]:
                raise RuntimeError(f"{case['id']} 不该把余量写进内存卡")
        elif "leader_prefix" in card.card["task"]:
            raise RuntimeError(f"{case['id']} 不作用的卡被写上了 leader_prefix")
    print("e-pg4 check ok", flush=True)
    return 0


def _parse_scenes(text: str) -> tuple[str, ...] | None:
    text = text.strip()
    if not text:
        return None
    names = tuple(part.strip() for part in text.split(",") if part.strip())
    known = {case["id"] for case in CASES}
    for name in names:
        if name not in known:
            raise SystemExit(f"未知场景 {name}")
    return names


def _parse_seeds(text: str) -> tuple[int, ...] | None:
    text = text.strip()
    if not text:
        return None
    return tuple(int(part) for part in text.split(",") if part.strip())


def _load_rows(specs: list[tuple]) -> list[dict]:
    rows = []
    for scene, seed, tier, _resume in specs:
        cached = load_partial(scene, seed, tier)
        if cached is None:
            raise SystemExit(f"缺少 partial：{scene} {tier} seed={seed}")
        rows.append(cached)
    return rows


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="E-PG4 冻结套件打开 0.7 s 短前缀，只报数")
    parser.add_argument("--check", action="store_true", help="只检查开关默认、场景卡和套件清单")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--scenes", default="", help="逗号分隔的场景 id，缺省整套")
    parser.add_argument("--seeds", default="", help="逗号分隔的种子，缺省各场景自己的冻结种子")
    parser.add_argument("--no-resume", action="store_true")
    parser.add_argument("--no-write", action="store_true")
    parser.add_argument("--render-only", action="store_true", help="用已有 partial 重写报告")
    parser.add_argument("--skip-spot", action="store_true", help="跳过关时逐位抽查")
    parser.add_argument("--spot-only", action="store_true", help="只跑关时逐位抽查")
    args = parser.parse_args(argv)
    if args.jobs < 1:
        raise SystemExit("--jobs 至少为 1")
    assert_eval_budget()
    if args.check:
        return run_check()
    scenes = _parse_scenes(args.scenes)
    seeds = _parse_seeds(args.seeds)
    full = scenes is None and seeds is None
    before = watched_snapshot()
    spot = None
    if not args.skip_spot and (full or args.spot_only):
        spot = spot_off(resume=not args.no_resume)
    if args.spot_only:
        assert_watched(before)
        return 0
    specs = job_specs(scenes, seeds, resume=not args.no_resume)
    if args.render_only:
        rows = _load_rows(specs)
        if full and not args.no_write:
            if spot is None and SPOT_PATH.is_file():
                spot = json.loads(SPOT_PATH.read_text(encoding="utf-8"))
            payload = build_payload(rows, jobs=args.jobs, spot=spot)
            write_docs(payload)
            print(f"wrote {MD_PATH} and {JSON_PATH}", flush=True)
        assert_watched(before)
        return 0
    print(f"E-PG4 runs={len(specs)} jobs={args.jobs} resume={not args.no_resume}", flush=True)
    rows = _run_group(specs, args.jobs)
    assert_watched(before)
    if full and not args.no_write:
        payload = build_payload(rows, jobs=args.jobs, spot=spot)
        write_docs(payload)
        print(f"wrote {MD_PATH} and {JSON_PATH}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
