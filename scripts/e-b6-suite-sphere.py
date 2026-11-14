#!/usr/bin/env python3
"""E-B6：欧氏碰撞球在常规 M2/M4 套件上的兼容评估。只报数。

四张已冻场景卡用各自的默认策略（不改磁盘上的卡，不改默认开关）：

- M2 ``crossing-pillars``、``crossing-forest``，seeds 0–9
- M4 ``crossing-6-forest``、``crossing-6-forest-sparse``，seeds 0–4

M4 单粒在 E-B4c 的六机 medium 上墙钟约 220–295 s。四臂 × 两张卡 × seeds 0–9
会把后段臂按墙钟预算串行拉得很长，所以六机只跑 0–4。同场景四臂种子完全对齐。

臂（后段半径 / 权重只在进程内的卡上盖过）：

- baseline：``peer_sphere_enabled`` 不开。上游默认 + 影子盾。
- b4c：E-B4c 默认球，r = 0.85 m，w = 8。report-only / 偏离上游。
- b5-radius：E-B5 扩半径，r = 1.00 m，w = 8。report-only / 偏离上游。
- b5-weight：E-B5 加权重，r = 0.85 m，w = 16。report-only / 偏离上游。

开球必须同时把后段 ``enabled`` 设为 JSON true，否则求解器不起来。其余字段
（椭球、safe_radius、empty_brake、track_clearance、任务、判据）保持场景卡原样。

``crossing.py`` 仍只在 ``swarm_penalty`` 时把邻机送进 ``refine_yopo``。本脚本
不改 ``yopo_l0/``，只在进程里包一层：欧氏球打开时把本规划拍的
``committed_neighbor_samples`` 送进去，椭球项仍关。四张卡都是 CrossingFleet。

后段预算仍是墙钟 0.02 s。带后段的臂默认一个进程。每粒落在
``out/e-b6-suite-sphere/partial/``，可续跑。
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-b6-suite-sphere" / "partial"
BODY_R = 0.35
DEVIATION_LABEL = "report-only / 偏离上游"
ENV_KEYS = (
    "YOPO_PEER_SPHERE",
    "YOPO_PEER_SPHERE_RADIUS_M",
    "YOPO_PEER_SPHERE_WEIGHT",
)

# (臂名, 半径 m, 权重)。baseline 不在这里。
ARM_SPEC = {
    "b4c": {"radius_m": 0.85, "weight": 8.0, "level": "E-B4c 默认球"},
    "b5-radius": {"radius_m": 1.00, "weight": 8.0, "level": "E-B5 扩半径"},
    "b5-weight": {"radius_m": 0.85, "weight": 16.0, "level": "E-B5 加权重"},
}
ARMS = ("baseline", "b4c", "b5-radius", "b5-weight")
SPHERE_ARMS = tuple(ARM_SPEC)

# (名字, 场景卡, 默认种子, 里程碑)
SCENES = (
    ("crossing-pillars", "scenes/crossing-pillars.json", tuple(range(10)), "M2"),
    ("crossing-forest", "scenes/crossing-forest.json", tuple(range(10)), "M2"),
    ("crossing-6-forest", "scenes/crossing-6-forest.json", tuple(range(5)), "M4"),
    ("crossing-6-forest-sparse", "scenes/crossing-6-forest-sparse.json", tuple(range(5)), "M4"),
)
SCENE_NAMES = tuple(item[0] for item in SCENES)
SCENE_PATH = {name: path for name, path, _seeds, _tier in SCENES}
SCENE_SEEDS = {name: seeds for name, _path, seeds, _tier in SCENES}
SCENE_TIER = {name: tier for name, _path, _seeds, tier in SCENES}

# 已冻门，只用来在报告里对照，不改场景卡，也不设新门。
# upper：本臂各种子里的最大值不得超过。lower：最紧的一粒不得低于。
FROZEN_GATES = {
    "crossing-pillars": {
        "arrival_rate": ("lower", 1.0),
        "collisions": ("upper", 0),
        "formation_error_p95_m": ("upper", 6.0),
        "min_inter_drone_distance_m": ("lower", 2.0),
        "min_obstacle_clearance_m": ("lower", 0.40),
    },
    "crossing-forest": {
        "arrival_rate": ("lower", 1.0),
        "collisions": ("upper", 0),
        "formation_error_p95_m": ("upper", 6.0),
        "min_inter_drone_distance_m": ("lower", 2.0),
    },
    "crossing-6-forest": {
        "arrival_rate": ("lower", 1.0),
        "collisions": ("upper", 0),
        "min_inter_drone_distance_m": ("lower", 0.70),
    },
    "crossing-6-forest-sparse": {
        "arrival_rate": ("lower", 1.0),
        "collisions": ("upper", 0),
        "min_inter_drone_distance_m": ("lower", 0.70),
    },
}

DISCLAIMER = (
    "只报数。report-only。不改 yopo_l0/、core/、scenes/、docs/tickets.md、README.md、"
    "scripts/premerge.sh、scripts/regress.sh，不进 scenes/suite.txt，不改已冻门。"
    "开球三臂同时打开后段平滑，并标 report-only / 偏离上游（上游 d5e1e60 没有这颗欧氏球）。"
    "基线是各场景卡默认策略加影子盾，peer_sphere_enabled 保持关。"
    "M2 seeds 0–9；M4 六机林带 seeds 0–4，同场景四臂对齐。"
    "0.85 m = 0.70 m 碰撞球 + 0.15 m 裕度。1.00 m 与权重 16 是本票的对照档，不是新默认。"
)
SEED_NOTE = (
    "M2 两张卡 seeds 0–9。M4 六机林带单粒墙钟在 E-B4c 的 crossing-6-forest 上约 220–295 s，"
    "四臂全跑 seeds 0–9 时后段臂必须按墙钟预算串行，本票改为 seeds 0–4。"
    "同场景四臂种子完全对齐。不传 --seeds 时用上述默认；传了则所有选中场景共用这一列。"
)

_INSTALLED = False
_ORIG: dict = {}


def clear_sphere_env() -> None:
    """环境变量会盖过卡上的半径和权重。本票的四臂只认卡。"""
    for key in ENV_KEYS:
        os.environ.pop(key, None)


def _deep(raw: dict) -> dict:
    return json.loads(json.dumps(raw))


def sphere_on(arm: str) -> bool:
    if arm not in ARMS:
        raise ValueError(arm)
    return arm in ARM_SPEC


def deviation_label(arm: str) -> str:
    return DEVIATION_LABEL if sphere_on(arm) else "否"


def seeds_for(scene: str, override: str | None) -> tuple[int, ...]:
    if scene not in SCENE_SEEDS:
        raise ValueError(scene)
    if override is None:
        return SCENE_SEEDS[scene]
    return tuple(int(part) for part in override.split(",") if part != "")


def build_card(scene: str, arm: str):
    """进程内盖一层。baseline 原样返回 load_card 的结果。"""
    from yopo_l0.scene import SceneCard, load_card

    if scene not in SCENE_PATH:
        raise ValueError(scene)
    if arm not in ARMS:
        raise ValueError(arm)
    loaded = load_card(ROOT / SCENE_PATH[scene])
    if arm == "baseline":
        return loaded
    raw = _deep(loaded.card)
    params = dict(raw.get("policy_params") or {})
    backend = dict(params.get("backend") or {})
    spec = ARM_SPEC[arm]
    backend["enabled"] = True
    backend["peer_sphere_enabled"] = True
    backend["peer_sphere_radius_m"] = spec["radius_m"]
    backend["peer_sphere_weight"] = spec["weight"]
    backend.pop("swarm_penalty", None)
    backend.pop("swarm_segment_penalty", None)
    params["backend"] = backend
    raw["policy_params"] = params
    return SceneCard(loaded.path, raw, loaded.obstacles)


def install_peer_delivery() -> None:
    """欧氏球打开时把本规划拍的邻机采样送进后段。椭球项仍只看 swarm_penalty。

    不改 policy.py / crossing.py 的文件。两个开关都关时，传进 refine_yopo 的邻机
    仍是 None，和原来的调用相同。
    """
    global _INSTALLED
    if _INSTALLED:
        return
    import yopo_l0.crossing as crossing_mod
    import yopo_l0.policy as policy_mod
    import yopo_l0.runner as runner_mod
    from yopo_l0.backend.config import peer_sphere_enabled as sphere_flag

    _ORIG["runner"] = runner_mod.swarm_penalty_enabled
    _ORIG["keep"] = policy_mod.UpstreamPolicy._keep_neighbor_samples
    _ORIG["apply"] = policy_mod.UpstreamPolicy._apply_backend
    _ORIG["cross"] = crossing_mod.CrossingFleet._apply_backend

    orig_enabled = runner_mod.swarm_penalty_enabled

    def enabled(params, **kwargs):
        if orig_enabled(params, **kwargs):
            return True
        return bool(sphere_flag(params, **kwargs))

    runner_mod.swarm_penalty_enabled = enabled

    orig_keep = policy_mod.UpstreamPolicy._keep_neighbor_samples

    def keep(self):
        if orig_keep(self):
            return True
        cfg = getattr(self, "_backend", None)
        return cfg is not None and bool(cfg.peer_sphere_enabled)

    policy_mod.UpstreamPolicy._keep_neighbor_samples = keep

    def apply(self, traj, goal):
        from yopo_l0.backend.bspline import refine_yopo

        cfg = self._backend
        use = bool(self._swarm) or (cfg is not None and bool(cfg.peer_sphere_enabled))
        neighbors = list(self._neighbors.values()) if use else None
        out, rec = refine_yopo(traj, cfg, self._obstacles, goal, neighbors=neighbors)
        self.backend_log.append(rec)
        return out

    policy_mod.UpstreamPolicy._apply_backend = apply

    def cross_apply(self, traj, goal):
        from yopo_l0.backend.bspline import refine_yopo
        from yopo_l0.inter_drone import committed_neighbor_samples

        cfg = self._backend
        neighbors = None
        wants = cfg is not None and (bool(cfg.swarm_penalty) or bool(cfg.peer_sphere_enabled))
        if wants and getattr(self, "_score_node", None) is not None:
            neighbors = committed_neighbor_samples(self._score_node, float(traj.t_start))
        out, rec = refine_yopo(traj, cfg, self._backend_obstacles, goal, neighbors=neighbors)
        self.backend_log.append(rec)
        return out

    crossing_mod.CrossingFleet._apply_backend = cross_apply
    _INSTALLED = True


def restore_peer_delivery() -> None:
    """测试用。把进程内的包装撤掉。"""
    global _INSTALLED
    if not _INSTALLED:
        return
    import yopo_l0.crossing as crossing_mod
    import yopo_l0.policy as policy_mod
    import yopo_l0.runner as runner_mod

    runner_mod.swarm_penalty_enabled = _ORIG["runner"]
    policy_mod.UpstreamPolicy._keep_neighbor_samples = _ORIG["keep"]
    policy_mod.UpstreamPolicy._apply_backend = _ORIG["apply"]
    crossing_mod.CrossingFleet._apply_backend = _ORIG["cross"]
    _INSTALLED = False


def _pin_threads() -> None:
    import torch

    torch.set_num_threads(1)


def _budget_wrap():
    import yopo_l0.backend.bspline as bspline

    current = bspline.refine_yopo
    if getattr(current, "_eb6_wrapped", False):
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
    wrapped._eb6_wrapped = True
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


def _counts(result: dict) -> tuple[int, int, float | None]:
    """障碍碰撞、机间碰撞、最小障碍净空。仿真在第一次碰撞停下。"""
    reason = result.get("reason") or ""
    outcome = result.get("outcome")
    block = result.get("crossing") or {}
    obs = int(outcome == "COLLISION" and "TRUTH_POSITION" in str(reason))
    inter = int(outcome == "COLLISION" and "INTER_DRONE" in str(reason))
    if "obstacle_collisions" in block:
        obs = int(block.get("obstacle_collisions") or 0)
        inter = int(block.get("inter_drone_collisions") or 0)
    dist = result.get("min_obstacle_distance_m")
    clearance = None if dist is None else float(dist) - BODY_R
    if block.get("min_obstacle_clearance_m") is not None:
        clearance = float(block["min_obstacle_clearance_m"])
    return obs, inter, clearance


def _row(scene: str, arm: str, result: dict, *, budget_calls: int, budget_cuts: int) -> dict:
    obs, inter, clearance = _counts(result)
    outcome = result.get("outcome")
    backend = _drop_samples(result.get("backend"))
    time_ms = (backend or {}).get("time_ms") or {}
    on = sphere_on(arm)
    spec = ARM_SPEC.get(arm) or {}
    return {
        "scene_kind": scene,
        "tier": SCENE_TIER[scene],
        "arm": arm,
        "arm_level": "基线" if arm == "baseline" else spec["level"],
        "scene": result.get("scene"),
        "seed": int(result["seed"]),
        "outcome": outcome,
        "reason": result.get("reason") or "",
        "arrived": int(result["arrived"]),
        "n_drones": int(result["n_drones"]),
        "arrival_rate": result.get("arrival_rate"),
        "arrival_time_s": result.get("arrival_time_s"),
        "obstacle_collisions": obs,
        "inter_drone_collisions": inter,
        "timeout": int(outcome == "TIMEOUT"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "min_obstacle_distance_m": result.get("min_obstacle_distance_m"),
        "min_obstacle_clearance_m": clearance,
        "formation_error_p95_m": result.get("formation_error_p95_m"),
        "sim_duration_s": result.get("sim_duration_s"),
        "final_goal_error_m": result.get("final_goal_error_m"),
        "wall_time_s": result.get("wall_time_s"),
        "backend": backend,
        "backend_time_p50_ms": time_ms.get("p50"),
        "backend_time_p95_ms": time_ms.get("p95"),
        "backend_budget_calls": int(budget_calls),
        "backend_budget_cuts": int(budget_cuts),
        "shadow": result.get("shadow_reject_rate") is not None or result.get("shadow") is not None,
        "shadow_reject_rate": result.get("shadow_reject_rate"),
        "scene_card_hash": result.get("scene_card_hash"),
        "commit": result.get("commit"),
        "safe_radius_m": result.get("safe_radius_m"),
        "runner_upstream_deviation": result.get("upstream_deviation"),
        "report_only": True,
        "deviates_from_upstream": on,
        "deviation_label": deviation_label(arm),
        "peer_sphere_enabled": on,
        "peer_sphere_radius_m": spec.get("radius_m"),
        "peer_sphere_weight": spec.get("weight"),
        "disclaimer": DISCLAIMER,
    }


def _partial(scene: str, arm: str, seed: int) -> Path:
    return PARTIAL / f"{scene}__{arm}__{seed}.json"


def run_one(scene: str, arm: str, seed: int, *, resume: bool) -> dict:
    dest = _partial(scene, arm, seed)
    if resume and dest.is_file():
        return json.loads(dest.read_text(encoding="utf-8"))
    from yopo_l0.runner import RunConfig, run

    clear_sphere_env()
    _pin_threads()
    install_peer_delivery()
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
    print(
        f"{scene} {arm} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"obs={row['obstacle_collisions']} inter={row['inter_drone_collisions']} "
        f"timeout={row['timeout']} gap={_f(row['min_inter_drone_distance_m'], 3)} "
        f"clear={_f(row['min_obstacle_clearance_m'], 3)} p95={_f(row['formation_error_p95_m'], 3)} "
        f"arrive={_f(row['arrival_time_s'], 2)} cuts={row['backend_budget_cuts']}/{row['backend_budget_calls']} "
        f"wall={row['wall_time_s']:.1f}s",
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


def _nums(group: list[dict], key: str) -> list[float]:
    return [float(row[key]) for row in group if row.get(key) is not None]


def aggregate_arm(group: list[dict]) -> dict:
    arrived = sum(int(row["arrived"]) for row in group)
    drones = sum(int(row["n_drones"]) for row in group)
    times = _nums(group, "arrival_time_s")
    p95s = _nums(group, "formation_error_p95_m")
    gaps = _nums(group, "min_inter_drone_distance_m")
    clears = _nums(group, "min_obstacle_clearance_m")
    return {
        "n_runs": len(group),
        "arrived": arrived,
        "drones": drones,
        "arrival_rate": None if drones == 0 else arrived / drones,
        "obstacle_collisions": sum(int(row["obstacle_collisions"]) for row in group),
        "inter_drone_collisions": sum(int(row["inter_drone_collisions"]) for row in group),
        "timeout": sum(int(row["timeout"]) for row in group),
        "formation_error_p95_m": None if not p95s else {"min": min(p95s), "max": max(p95s)},
        "min_inter_drone_distance_m": None if not gaps else min(gaps),
        "min_inter_drone_distance_range_m": None if not gaps else {"min": min(gaps), "max": max(gaps)},
        "min_obstacle_clearance_m": None if not clears else min(clears),
        "min_obstacle_clearance_range_m": None if not clears else {"min": min(clears), "max": max(clears)},
        "mean_arrival_time_s": None if not times else sum(times) / len(times),
        "n_arrival_times": len(times),
        "deviates_from_upstream": bool(group and group[0]["deviates_from_upstream"]),
        "deviation_label": group[0]["deviation_label"] if group else "否",
        "peer_sphere_radius_m": group[0].get("peer_sphere_radius_m") if group else None,
        "peer_sphere_weight": group[0].get("peer_sphere_weight") if group else None,
    }


def _gate_value(stats: dict, name: str):
    if name == "arrival_rate":
        return stats["arrival_rate"]
    if name == "collisions":
        return int(stats["obstacle_collisions"]) + int(stats["inter_drone_collisions"])
    if name == "formation_error_p95_m":
        block = stats["formation_error_p95_m"]
        return None if block is None else block["max"]
    return stats.get(name)


def gate_check(scene: str, stats: dict) -> dict:
    """对照该卡已冻门。缺测的指标记成未通过，避免把空表写成过门。"""
    failed = []
    for name, (direction, limit) in FROZEN_GATES[scene].items():
        got = _gate_value(stats, name)
        if got is None:
            failed.append(f"{name} 缺失")
            continue
        if direction == "upper" and float(got) > float(limit) + 1e-9:
            failed.append(f"{name} {got} > {limit}")
        if direction == "lower" and float(got) < float(limit) - 1e-9:
            failed.append(f"{name} {got} < {limit}")
    return {"passes_frozen_gates": not failed, "failed": failed}


def _worse_than_baseline(base: dict, arm: dict) -> list[str]:
    """到达、碰撞、超时变差，或基线守住的已冻门这臂没守住。"""
    reasons = []
    if int(arm["arrived"]) < int(base["arrived"]):
        reasons.append(f"到达 {arm['arrived']}/{arm['drones']} < 基线 {base['arrived']}/{base['drones']}")
    if int(arm["obstacle_collisions"]) > int(base["obstacle_collisions"]):
        reasons.append(f"障碍碰撞 {arm['obstacle_collisions']} > 基线 {base['obstacle_collisions']}")
    if int(arm["inter_drone_collisions"]) > int(base["inter_drone_collisions"]):
        reasons.append(f"机间碰撞 {arm['inter_drone_collisions']} > 基线 {base['inter_drone_collisions']}")
    if int(arm["timeout"]) > int(base["timeout"]):
        reasons.append(f"超时 {arm['timeout']} > 基线 {base['timeout']}")
    if base["frozen_gates"]["passes_frozen_gates"]:
        for item in arm["frozen_gates"]["failed"]:
            reasons.append(f"已冻门 {item}")
    return reasons


def summarize_stats(rows: list[dict]) -> list[dict]:
    """按场景、臂汇总。只包含行里实际出现的组合。"""
    out = []
    scenes = [name for name in SCENE_NAMES if any(row["scene_kind"] == name for row in rows)]
    for scene in scenes:
        grouped = {}
        for arm in ARMS:
            group = [row for row in rows if row["scene_kind"] == scene and row["arm"] == arm]
            if not group:
                continue
            stats = aggregate_arm(group)
            stats["scene"] = scene
            stats["tier"] = SCENE_TIER[scene]
            stats["arm"] = arm
            stats["seeds"] = sorted(int(row["seed"]) for row in group)
            stats["frozen_gates"] = gate_check(scene, stats)
            grouped[arm] = stats
        base = grouped.get("baseline")
        for arm, stats in grouped.items():
            if arm == "baseline" or base is None:
                stats["regression_vs_baseline"] = []
                stats["zero_regression_vs_baseline"] = True
            else:
                reasons = _worse_than_baseline(base, stats)
                stats["regression_vs_baseline"] = reasons
                stats["zero_regression_vs_baseline"] = not reasons
            out.append(stats)
    return out


def format_summary(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 臂 | 标注 | 到达 | 障碍碰撞 | 机间碰撞 | 超时 | 队形误差 p95 m | 最小机间 m | 最小障碍净空 m | 平均到达时间 s |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for stats in summarize_stats(rows):
        p95 = stats["formation_error_p95_m"]
        p95_cell = "—" if p95 is None else _rng([p95["min"], p95["max"]], 3)
        times = stats["mean_arrival_time_s"]
        if times is None:
            time_cell = "—"
        else:
            time_cell = _f(times, 2)
            if int(stats["n_arrival_times"]) != int(stats["n_runs"]):
                time_cell += f" ({stats['n_arrival_times']}/{stats['n_runs']})"
        lines.append(
            f"| {stats['scene']} | {stats['arm']} | {stats['deviation_label']} | "
            f"{stats['arrived']}/{stats['drones']} | {stats['obstacle_collisions']} | "
            f"{stats['inter_drone_collisions']} | {stats['timeout']} | {p95_cell} | "
            f"{_rng(_range_pair(stats['min_inter_drone_distance_range_m']), 3)} | "
            f"{_rng(_range_pair(stats['min_obstacle_clearance_range_m']), 3)} | {time_cell} |"
        )
    return "\n".join(lines) + "\n"


def _range_pair(block: dict | None) -> list[float]:
    if not block:
        return []
    return [float(block["min"]), float(block["max"])]


def _load_partials(jobs: list[tuple[str, str, int]]) -> list[dict]:
    rows = []
    for scene, arm, seed in jobs:
        path = _partial(scene, arm, seed)
        if path.is_file():
            rows.append(json.loads(path.read_text(encoding="utf-8")))
    return rows


def _run_group(jobs: list[tuple[str, str, int]], *, workers: int, resume: bool) -> None:
    specs = [(scene, arm, seed, resume) for scene, arm, seed in jobs]
    if not specs:
        return
    if workers <= 1:
        for spec in specs:
            _job(spec)
        return
    from concurrent.futures import ProcessPoolExecutor, as_completed

    with ProcessPoolExecutor(max_workers=workers) as pool:
        futs = [pool.submit(_job, spec) for spec in specs]
        for fut in as_completed(futs):
            fut.result()


def _parse_list(text: str, allowed: tuple[str, ...], what: str) -> list[str]:
    items = [part for part in text.split(",") if part]
    unknown = [part for part in items if part not in allowed]
    if unknown:
        raise SystemExit(f"未知{what}: {unknown}")
    return items


def build_jobs(scenes: list[str], arms: list[str], seeds_override: str | None) -> list[tuple[str, str, int]]:
    jobs = []
    for scene in scenes:
        seeds = seeds_for(scene, seeds_override)
        for arm in arms:
            for seed in seeds:
                jobs.append((scene, arm, int(seed)))
    return jobs


def write_payload(rows: list[dict], jobs: list[tuple[str, str, int]], *, scenes: list[str], arms: list[str], seeds_override: str | None, out: Path) -> dict:
    rows = sorted(rows, key=lambda row: (SCENE_NAMES.index(row["scene_kind"]), ARMS.index(row["arm"]), int(row["seed"])))
    payload = {
        "disclaimer": DISCLAIMER,
        "seed_note": SEED_NOTE,
        "report_only": True,
        "default_off": True,
        "deviates_from_upstream_when_enabled": True,
        "deviation_label": DEVIATION_LABEL,
        "arms": {
            "baseline": {"peer_sphere_enabled": False, "deviation_label": "否"},
            **{
                name: {
                    "peer_sphere_enabled": True,
                    "peer_sphere_radius_m": spec["radius_m"],
                    "peer_sphere_weight": spec["weight"],
                    "level": spec["level"],
                    "deviation_label": DEVIATION_LABEL,
                }
                for name, spec in ARM_SPEC.items()
            },
        },
        "note": (
            "开球臂把后段 enabled 与 peer_sphere 一起打开，邻机采样在进程内送进 refine_yopo。"
            "椭球 swarm_penalty / swarm_segment_penalty 保持关。"
            "最小机间距离是运行器第一次越过 0.70 m 那一拍的采样；没撞则是全程最小。"
            "最小障碍净空 = 机体中心到障碍表面的距离 − 0.35 m。"
            "队形误差 p95 是结果里的 formation_error_p95_m（官方口径）。"
            "平均到达时间只平均整队都到达的 arrival_time_s。"
            "Crossing 的运行器 upstream_deviation 只看 safe_radius，开球时它仍可能是 false；"
            "本票以 deviation_label 为准。"
            "墙钟不参与轨迹，但后段 0.02 s 预算按墙钟截断。"
        ),
        "scenes": scenes,
        "arms_run": arms,
        "seeds_by_scene": {scene: list(seeds_for(scene, seeds_override)) for scene in scenes},
        "completed": len(rows),
        "expected": len(jobs),
        "incomplete": len(jobs) - len(rows),
        "summary": summarize_stats(rows),
        "rows": rows,
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return payload


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="E-B6 常规 M2/M4 套件上的欧氏球对照，只报数")
    ap.add_argument("--scenes", default=",".join(SCENE_NAMES))
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=None, help="盖过每张卡自己的种子。缺省 M2 为 0–9，M4 为 0–4。")
    ap.add_argument("--jobs-plain", type=int, default=2)
    ap.add_argument("--jobs-backend", type=int, default=1)
    ap.add_argument("--out", default="docs/research/e-b6-suite-sphere-runs.json")
    ap.add_argument("--no-resume", action="store_true")
    ap.add_argument("--summarize-only", action="store_true")
    args = ap.parse_args(argv)
    scenes = _parse_list(args.scenes, SCENE_NAMES, "场景")
    arms = _parse_list(args.arms, ARMS, "臂")
    if int(args.jobs_plain) > 2 or int(args.jobs_backend) > 2:
        raise SystemExit("最多 2 个进程，避免墙钟预算把后段迭代截短")
    if int(args.jobs_plain) < 1 or int(args.jobs_backend) < 1:
        raise SystemExit("进程数至少为 1")
    jobs = build_jobs(scenes, arms, args.seeds)
    resume = not args.no_resume
    if not args.summarize_only:
        from yopo_swarm_core.baseline import fetch_upstream

        fetch_upstream()
        plain = [job for job in jobs if job[1] == "baseline"]
        backend = [job for job in jobs if job[1] != "baseline"]
        if plain:
            _run_group(plain, workers=max(1, int(args.jobs_plain)), resume=resume)
        if backend:
            _run_group(backend, workers=max(1, int(args.jobs_backend)), resume=resume)
    rows = _load_partials(jobs)
    payload = write_payload(
        rows,
        jobs,
        scenes=scenes,
        arms=arms,
        seeds_override=args.seeds,
        out=ROOT / args.out,
    )
    print(format_summary(rows))
    print(f"wrote {ROOT / args.out} ({payload['completed']}/{payload['expected']})")
    return 0 if payload["incomplete"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
