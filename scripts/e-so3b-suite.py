#!/usr/bin/env python3
"""票 E-SO3b：一阶 SO3 开关打开后，重跑 M0–M4 冻结套件。只报数。

不改场景卡默认值，不改已冻门。点质量数字读 docs/results/SUMMARY.json，本脚本只跑 SO3。
每粒种子写到 out/e-so3b-suite/partial/，中断后原样再跑会续上。
"""

from __future__ import annotations

import argparse
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
PARTIAL = ROOT / "out" / "e-so3b-suite" / "partial"
JSON_PATH = ROOT / "docs" / "research" / "e-so3b-suite-runs.json"
MD_PATH = ROOT / "docs" / "research" / "e-so3b-suite.md"
SUMMARY_PATH = ROOT / "docs" / "results" / "SUMMARY.json"
TIERS_PATH = ROOT / "scenes" / "formation-6-tiers.json"

# 改积分、开关默认参数或种子清单时一起改，旧 partial 就不会被拿来用。
MODEL_REV = "so3b-att1-e-so3-nominal-px4rate-2026-11-15"
ROW_VERSION = 1

DISCLAIMER = (
    "只报数。report-only。SO3 开关默认关；本表是打开后的冻结套件，标偏离上游。"
    "参数来自 E-SO3 标称档和 PX4 公开默认值，置信度写在模型一节，不是本仓库的 SITL 辨识。"
    "不改 core/、场景卡默认值、scenes/suite.txt、已冻门、docs/tickets.md、README.md、"
    "scripts/premerge.sh、scripts/regress.sh、scripts/sitl/Dockerfile。"
    "不重复 E-SO3 已经量化的二阶两档。未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。"
)

CASES = (
    {"id": "g2-pillar", "path": "scenes/g2-pillar.json", "policy": "upstream", "seeds": tuple(range(10)), "milestone": "M0", "tiers": False},
    {"id": "formation-hold", "path": "scenes/formation-hold.json", "policy": "formation", "seeds": tuple(range(3)), "milestone": "M1", "tiers": False},
    {"id": "crossing-pillars", "path": "scenes/crossing-pillars.json", "policy": "crossing", "seeds": tuple(range(10)), "milestone": "M2", "tiers": False},
    {"id": "crossing-forest", "path": "scenes/crossing-forest.json", "policy": "crossing", "seeds": tuple(range(10)), "milestone": "M2", "tiers": False},
    {"id": "formation-switch", "path": "scenes/formation-switch.json", "policy": "switch", "seeds": tuple(range(10)), "milestone": "M3", "tiers": False},
    {"id": "formation-6-open", "path": "scenes/formation-6-open.json", "policy": "formation", "seeds": tuple(range(5)), "milestone": "M4", "tiers": True},
    {"id": "crossing-6-forest", "path": "scenes/crossing-6-forest.json", "policy": "crossing", "seeds": tuple(range(10)), "milestone": "M4", "tiers": False},
    {"id": "crossing-6-forest-sparse", "path": "scenes/crossing-6-forest-sparse.json", "policy": "crossing", "seeds": tuple(range(10)), "milestone": "M4", "tiers": False},
)

OPEN_AREA = {"formation-hold", "formation-6-open"}
SWITCH_METRICS = (
    "error_p95_before_m",
    "error_p95_after_m",
    "min_inter_drone_transition_m",
    "transition_collisions",
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


def params_fingerprint() -> str:
    from yopo_l0.so3 import SWITCH_DEFAULTS

    blob = {
        "model_rev": MODEL_REV,
        "row_version": ROW_VERSION,
        "switch": SWITCH_DEFAULTS,
        "cases": [
            {"id": case["id"], "seeds": list(case["seeds"]), "tiers": case["tiers"], "policy": case["policy"]}
            for case in CASES
        ],
        "tier_ids": [tier["id"] for tier in load_tiers()],
    }
    raw = json.dumps(blob, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(raw).hexdigest()


def job_specs(scenes: tuple[str, ...] | None = None, *, resume: bool = True) -> list[tuple]:
    wanted = set(scenes) if scenes is not None else None
    tiers = load_tiers()
    specs = []
    for case in CASES:
        if wanted is not None and case["id"] not in wanted:
            continue
        if case["tiers"]:
            for tier in tiers:
                for seed in case["seeds"]:
                    specs.append((case["id"], int(seed), str(tier["id"]), bool(resume)))
        else:
            for seed in case["seeds"]:
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


def _split_collision(result: dict) -> tuple[int, int]:
    reason = str(result.get("reason") or "")
    if result.get("outcome") != "COLLISION":
        return 0, 0
    if "INTER_DRONE" in reason:
        return 0, 1
    return 1, 0


def _simulate(scene: str, seed: int, tier: str | None) -> dict:
    from yopo_l0.runner import RunConfig, run
    from yopo_l0.scene import load_card

    case = case_by_id(scene)
    card = load_card(ROOT / case["path"])
    file_hash = card.hash
    if card.card["dynamics"].get("model") != "point_mass_lag":
        raise RuntimeError(f"{scene} 的场景卡动力学不是 point_mass_lag")
    cfg = RunConfig(
        seed=int(seed),
        policy=case["policy"],
        shadow=True,
        so3=True,
        bus=_bus_for(tier),
    )
    out = run(card, cfg)
    res = out.result
    if res.get("dynamics") != "so3_lag":
        raise RuntimeError(f"{scene} seed {seed} 动力学读回 {res.get('dynamics')}")
    if res.get("upstream_deviation") is not True:
        raise RuntimeError(f"{scene} seed {seed} 打开 SO3 后没有标偏离上游")
    so3 = res.get("so3") or {}
    if so3.get("mark") != "偏离上游" or int(so3.get("attitude_order") or 0) != 1:
        raise RuntimeError(f"{scene} seed {seed} 的 so3 记录不对：{so3}")
    if res.get("scene_card_hash") != file_hash:
        raise RuntimeError(f"{scene} 运行改写了场景卡哈希")
    body = float(card.card["criteria"]["body_radius_m"])
    surface = float(res["min_obstacle_distance_m"])
    obstacle, inter = _split_collision(res)
    switch = res.get("switch") or {}
    switch_keep = {key: switch[key] for key in SWITCH_METRICS if key in switch}
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
        "dynamics": res.get("dynamics"),
        "so3": so3,
        "upstream_deviation": True,
        "mark": "偏离上游",
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
        "safe_radius_m": res.get("safe_radius_m"),
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
    tier_s = tier or "-"
    print(
        f"{scene} {tier_s} seed={row['seed']} {row['outcome']} "
        f"{row['arrived']}/{row['n_drones']} coll={row['collision']} "
        f"clear={float(row['min_clearance_m']):.3f} t={arrival_s} "
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


def warmup() -> None:
    from yopo_l0.net import load_model

    load_model()


def _broken_names(gates: dict, rows: list[dict]) -> list[str]:
    """按门槛重算哪些指标没过。缺失也算破。不靠失败句子里的子串。"""
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
    if isinstance(trans, dict) or trans is not None and not isinstance(trans, dict):
        if isinstance(trans, dict):
            shown = _fmt_opt(trans.get("min"))
        else:
            shown = _fmt_opt(trans)
        if shown != "—":
            return f"{text}（过渡 {shown}）"
    return text


def _so3_stats(rows: list[dict]) -> dict:
    p95s = [float(r["formation_error_p95_m"]) for r in rows if r.get("formation_error_p95_m") is not None]
    inters = [float(r["min_inter_drone_distance_m"]) for r in rows if r.get("min_inter_drone_distance_m") is not None]
    clears = [float(r["min_clearance_m"]) for r in rows if r.get("min_clearance_m") is not None]
    times = [float(r["arrival_time_s"]) for r in rows if r.get("arrival_time_s") is not None]
    before = [
        float(v)
        for r in rows
        if (v := (r.get("switch") or {}).get("error_p95_before_m")) is not None
    ]
    after = [
        float(v)
        for r in rows
        if (v := (r.get("switch") or {}).get("error_p95_after_m")) is not None
    ]
    trans = [
        float(v)
        for r in rows
        if (v := (r.get("switch") or {}).get("min_inter_drone_transition_m")) is not None
    ]
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
    }


def _summary_by_key() -> dict[tuple, dict]:
    payload = json.loads(SUMMARY_PATH.read_text(encoding="utf-8"))
    out = {}
    for row in payload["rows"]:
        out[(row["scene"], row.get("fault_tier"))] = row
    return out


def _summary_view(row: dict) -> dict:
    p95 = row.get("formation_error_p95_m")
    return {
        "arrivals": row.get("arrivals"),
        "collisions": row.get("collisions"),
        "formation_error_p95_m": p95,
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
    }


def _summary_gate_broken(view: dict) -> list[str]:
    """用 SUMMARY 的汇总行核对冻结门。上界看范围内的最大，下界看最紧的那个数。"""
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


def _coll_text(stats: dict, *, split: bool) -> str:
    n = int(stats["collisions"])
    if not split or n == 0:
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
        comparisons.append(
            {
                "scene": case["id"],
                "milestone": case["milestone"],
                "fault_tier": tier,
                "policy": group[0].get("policy"),
                "forest_density": group[0].get("forest_density"),
                "seeds": [int(r["seed"]) for r in group],
                "gates": gates,
                "gate_failures": failures,
                "broken_gates": broken,
                "point_mass": _summary_view(base),
                "so3": _so3_stats(group),
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


def _find(comparisons: list[dict], scene: str, tier: str | None = None) -> dict:
    for item in comparisons:
        if item["scene"] == scene and item.get("fault_tier") == tier:
            return item
    raise KeyError((scene, tier))


def _conclusion(comparisons: list[dict]) -> list[str]:
    broken = [item for item in comparisons if item["gate_failures"]]
    held = [item for item in comparisons if item.get("gates") and not item["gate_failures"]]
    lines = ["## 结论", ""]
    if not broken:
        lines.append(f"这 {len(comparisons)} 行的冻结门在一阶 SO3 下都还成立。碰撞在每一行都是 0。")
    else:
        bits = []
        for item in broken:
            label = f"{item['milestone']} {_scene_label(item)}"
            if item.get("fault_tier"):
                label += f" / {item['fault_tier']}"
            so3 = item["so3"]
            pm = item["point_mass"]
            names = "、".join(item["broken_gates"]) or "见上面的句子"
            bits.append(
                f"{label} 破了 {names}：到达 {so3['arrivals']}（点质量 {pm['arrivals']}），"
                f"碰撞 {so3['collisions']}（点质量 {pm['collisions']}）。"
            )
        coll = sum(int(item["so3"]["collisions"]) for item in comparisons)
        lines.append(
            f"冻结门破了 {len(broken)} 行，其余 {len(held)} 行仍过。"
            + "".join(bits)
            + f"SO3 碰撞合计 {coll}。"
        )
    forest = _find(comparisons, "crossing-forest")
    six = _find(comparisons, "crossing-6-forest")
    sparse = _find(comparisons, "crossing-6-forest-sparse")
    g2 = _find(comparisons, "g2-pillar")
    lines.append(
        "三机中等林最小净空从 "
        f"{_fmt_opt(forest['point_mass'].get('min_clearance_m'))} m 到 "
        f"{_fmt_opt(forest['so3'].get('min_clearance_m'))} m，"
        "六机中等林从 "
        f"{_fmt_opt(six['point_mass'].get('min_clearance_m'))} m 到 "
        f"{_fmt_opt(six['so3'].get('min_clearance_m'))} m。"
        "这两项在对应卡上只报数，不进冻结门。"
        "六机稀疏没到齐的那一次碰撞仍是 0；这一档队形 p95 的下沿盖过了这次没走完的轨迹，不能读成队形变好。"
        "G2 到达时间从 "
        f"{float(g2['point_mass']['arrival_time_s']['mean']):.3f} s 到 "
        f"{float(g2['so3']['arrival_time_s']['mean']):.3f} s，"
        "仍低于 5.0 s 的门。"
        "这些数只报数，不改默认动力学，也不改已冻门。"
    )
    lines.append("")
    return lines


def render_markdown(payload: dict) -> str:
    lines = [
        "# 票 E-SO3b：冻结套件上的一阶 SO3，只报数",
        "",
        DISCLAIMER,
        "",
        "点质量一列来自 [SUMMARY](../results/SUMMARY.md)（commit `5524cab42584`，场景卡哈希与当前这八张卡一致）。"
        "SO3 一列是本票打开开关后的重跑。净空 = 机体中心到障碍表面的距离 − 0.35 m。"
        "开阔区写「离地」。队形误差 p95 多个 seed 写出最小到最大；最小机间和最小净空取最紧的那一次。"
        "到达时间是有全体到达的运行的 `arrival_time_s` 均值，括号里是最小到最大，以及有全体到达的运行次数。"
        "E-SO3 已经报过二阶标称档和慢姿态档在 G2、三机柱群、三机林、六机稀疏上的差距，这里不重复那张表。",
        "",
        "## 模型",
        "",
        "开关在 `RunConfig.so3`，也可以把场景卡 `dynamics.model` 写成 `so3_lag`。默认关。"
        "关时运行器仍调用 `make_model(场景卡动力学)`，不导入 `yopo_l0/so3.py`。"
        "打开后再标「偏离上游」。本套件用 `RunConfig.so3=True`，不改场景卡文件。",
        "",
        "姿态是一阶：角速度命令 `ω = clip(−e_R / τ_att)`，误差是 Lee, Leok, McClamroch, CDC 2010 的 SO(3) vee。"
        "推力对比力命令做一阶滞后，再限制在 `[0, g+a_max]`。倾角限制在锥内。滚转、俯仰、偏航角速度分开限幅。"
        "外环和点质量同一式，`kp/kd/a_max` 沿用场景卡（这八张卡都是 4 / 3 / 6）。"
        "深度相机仍只读水平偏航，倾角不进光线。",
        "",
        "| 参数 | 本开关 | 来源与置信度 |",
        "|---|---|---|",
    ]
    from yopo_l0.so3 import SOURCE, SWITCH_DEFAULTS

    lines.append(f"| `attitude_order` | {SWITCH_DEFAULTS['attitude_order']} | {SOURCE['attitude_order']} |")
    lines.append(f"| `tau_att_s` | {SWITCH_DEFAULTS['tau_att_s']} s | {SOURCE['tau_att_s']} |")
    lines.append(f"| `tau_thrust_s` | {SWITCH_DEFAULTS['tau_thrust_s']} s | {SOURCE['tau_thrust_s']} |")
    lines.append(f"| `max_tilt_deg` | {SWITCH_DEFAULTS['max_tilt_deg']} ° | {SOURCE['max_tilt_deg']} |")
    lines.append(
        f"| 角速率 | {SWITCH_DEFAULTS['roll_rate_max_deg_s']:.0f} / "
        f"{SWITCH_DEFAULTS['pitch_rate_max_deg_s']:.0f} / "
        f"{SWITCH_DEFAULTS['yaw_rate_max_deg_s']:.0f} °/s | {SOURCE['rate_limits_deg_s']} |"
    )
    lines.append(f"| `yaw_weight` | {SWITCH_DEFAULTS['yaw_weight']} | {SOURCE['yaw_weight']} |")
    lines.append(f"| 推力幅值 | [0, g+a_max] | {SOURCE['thrust_limit']} |")
    lines.append(f"| 外环 | kp={SWITCH_DEFAULTS['kp']}，kd={SWITCH_DEFAULTS['kd']}，a_max={SWITCH_DEFAULTS['a_max']} | {SOURCE['outer_loop']} |")
    lines.append(f"| `g` | {SWITCH_DEFAULTS['g']} | {SOURCE['g']} |")
    lines.append("")
    lines.append(f"模型修订 `{payload['model_rev']}`。跑数 commit `{payload['commit']}`。")
    lines.append("")
    lines += [
        "## 和 SUMMARY 逐行对照",
        "",
        "| 里程碑 | 场景 | 档 | 动力学 | 到达 | 碰撞 | 最小机间 m | 最小净空 m | 队形误差 p95 m | 到达时间 s | 冻结门 |",
        "|---|---|---|---|---:|---:|---|---|---|---|---|",
    ]
    for item in payload["comparisons"]:
        pm = item["point_mass"]
        so3 = item["so3"]
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
            "| {ms} | {scene} | {tier} | SO3 一阶（偏离上游） | {arr} | {col} | {inter} | {clear} | {p95} | {time} | {gate} |".format(
                ms=milestone,
                scene=scene,
                tier=tier,
                arr=so3["arrivals"],
                col=_coll_text(so3, split=True),
                inter=_fmt_inter(so3),
                clear=_fmt_clear(so3.get("min_clearance_m"), so3.get("clearance_is_ground")),
                p95=_fmt_p95(so3),
                time=_fmt_time(so3.get("arrival_time_s"), int(so3["n_runs"])),
                gate=gate,
            )
        )
    lines.append("")
    lines += [
        "## 哪些冻结门会破",
        "",
    ]
    broken = [item for item in payload["comparisons"] if item["gate_failures"]]
    held = [item for item in payload["comparisons"] if item["gates"] and not item["gate_failures"]]
    if not broken:
        lines.append("这 13 行在 SO3 一阶下，场景卡里的冻结门都还成立。")
    else:
        lines.append("下面这些行的冻结门在 SO3 一阶下不成立。点质量在 SUMMARY 里这些门是过的。")
        lines.append("")
        for item in broken:
            label = f"{item['milestone']} {_scene_label(item)} / {_tier_label(item.get('fault_tier'))}"
            lines.append(f"### {label}")
            lines.append("")
            lines.append(f"门槛：`{json.dumps(item['gates'], ensure_ascii=False)}`。")
            lines.append("")
            for msg in item["gate_failures"]:
                lines.append(f"- {msg}")
            lines.append("")
    if held:
        names = "、".join(f"{_scene_label(item)}/{_tier_label(item.get('fault_tier'))}" for item in held)
        lines.append(f"仍然成立的冻结行：{names}。")
    lines.append("")
    lines.extend(_conclusion(payload["comparisons"]))
    lines += [
        "",
        "## 分种子",
        "",
        "| 场景 | 档 | seed | 结果 | 原因 | 到达 | 碰撞 | 最小净空 m | 最小机间 m | 队形 p95 m | 到达时间 s | 仿真 s |",
        "|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|---:|",
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
        lines.append(
            "| {scene} | {tier} | {seed} | {outcome} | `{reason}` | {arr} | {col} | {clear} | {inter} | {p95} | {arrival} | {sim} |".format(
                scene=row["scene"],
                tier=_tier_label(row.get("fault_tier")),
                seed=int(row["seed"]),
                outcome=row.get("outcome"),
                reason=str(row.get("reason") or "").replace("|", "\\|"),
                arr=f"{row['arrived']}/{row['n_drones']}",
                col=int(row.get("collision") or 0),
                clear=f"{float(row['min_clearance_m']):.3f}",
                inter=inter_s,
                p95=p95_s,
                arrival=arrival_s,
                sim=f"{float(row['sim_duration_s']):.2f}",
            )
        )
    lines += [
        "",
        "## 复现",
        "",
        "```",
        ".venv/bin/python scripts/e-so3b-suite.py --jobs 2",
        ".venv/bin/python -m pytest tests/test_e_so3b_suite.py -q",
        "PYTHONPATH=.:core/src .venv/bin/python -m pytest -q tests",
        "```",
        "",
        "缺省从 `out/e-so3b-suite/partial/` 续跑。`--no-resume` 忽略已有 partial。"
        "模型或种子清单变了，指纹对不上的 partial 不会被拿来用。`--jobs` 可以降到 1。",
        "",
    ]
    return "\n".join(lines)


def build_payload(rows: list[dict], *, jobs: int) -> dict:
    from yopo_l0.so3 import SWITCH_DEFAULTS

    rows = sorted(
        rows,
        key=lambda r: (_case_index(r["scene"]), _tier_index(r.get("fault_tier")), int(r["seed"])),
    )
    commits = sorted({str(r.get("commit") or "") for r in rows})
    return {
        "title": "票 E-SO3b：冻结套件上的一阶 SO3，只报数",
        "disclaimer": DISCLAIMER,
        "report_only": True,
        "frozen": False,
        "mark": "偏离上游",
        "model_rev": MODEL_REV,
        "row_version": ROW_VERSION,
        "params_fingerprint": params_fingerprint(),
        "switch": SWITCH_DEFAULTS,
        "commit": commits[0] if len(commits) == 1 else None,
        "commits": commits,
        "summary_commit": "5524cab42584ea54df54bbf38fc77e26117066dd",
        "jobs": int(jobs),
        "n_runs": len(rows),
        "comparisons": build_comparisons(rows),
        "rows": rows,
    }


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


def write_docs(payload: dict) -> None:
    JSON_PATH.write_text(json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    MD_PATH.write_text(render_markdown(payload), encoding="utf-8")


def run_check() -> int:
    from yopo_l0.dynamics import PointMassLag
    from yopo_l0.runner import RunConfig
    from yopo_l0.so3 import SWITCH_DEFAULTS

    text = (ROOT / "yopo_l0/dynamics.py").read_text(encoding="utf-8")
    if "so3" in text.lower():
        raise RuntimeError("dynamics.py 不应出现 so3；点质量路径保持原文件")
    if "point_mass_lag" not in text or PointMassLag is None:
        raise RuntimeError("点质量模型不在了")
    if RunConfig().so3 is not None:
        raise RuntimeError("RunConfig.so3 默认必须是 None")
    if SWITCH_DEFAULTS["attitude_order"] != 1:
        raise RuntimeError("本票的开关应是一阶姿态")
    if len(job_specs()) != 93:
        raise RuntimeError(f"套件应是 93 次运行，得到 {len(job_specs())}")
    for case in CASES:
        raw = json.loads((ROOT / case["path"]).read_text(encoding="utf-8"))
        dyn = raw.get("dynamics") or {}
        if dyn.get("model") != "point_mass_lag":
            raise RuntimeError(f"{case['id']} 的动力学被改成了 {dyn.get('model')}")
        if "so3" in json.dumps(raw).lower():
            raise RuntimeError(f"{case['id']} 的场景卡里出现了 so3")
        if raw.get("criteria", {}).get("frozen") is not True:
            raise RuntimeError(f"{case['id']} 的 frozen 标志被动过")
    print("e-so3b check ok", flush=True)
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


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="E-SO3b 冻结套件打开一阶 SO3，只报数")
    parser.add_argument("--check", action="store_true", help="只检查开关默认和套件清单")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--scenes", default="", help="逗号分隔的场景 id，缺省整套")
    parser.add_argument("--no-resume", action="store_true")
    parser.add_argument("--no-write", action="store_true")
    parser.add_argument("--render-only", action="store_true", help="用已有 partial 重写报告")
    parser.add_argument("--no-warmup", action="store_true")
    args = parser.parse_args(argv)
    if args.jobs < 1:
        raise SystemExit("--jobs 至少为 1")
    if args.check:
        return run_check()
    scenes = _parse_scenes(args.scenes)
    specs = job_specs(scenes, resume=not args.no_resume)
    if args.render_only:
        rows = []
        for scene, seed, tier, _resume in specs:
            cached = load_partial(scene, seed, tier)
            if cached is None:
                raise SystemExit(f"缺少 partial：{scene} {tier} seed={seed}")
            rows.append(cached)
        if scenes is None and not args.no_write:
            payload = build_payload(rows, jobs=args.jobs)
            write_docs(payload)
            print(f"wrote {MD_PATH} and {JSON_PATH}", flush=True)
        return 0
    if not args.no_warmup:
        print("warmup YOPO weights", flush=True)
        warmup()
    print(f"E-SO3b runs={len(specs)} jobs={args.jobs} resume={not args.no_resume}", flush=True)
    rows = _run_group(specs, args.jobs)
    if scenes is None and not args.no_write:
        payload = build_payload(rows, jobs=args.jobs)
        write_docs(payload)
        print(f"wrote {MD_PATH} and {JSON_PATH}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
