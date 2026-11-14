#!/usr/bin/env python3
"""E-B7 只报数：六机编队六档通讯故障，以及八机开阔对向，开欧氏碰撞球。

不改 yopo_l0/、core/、scenes/、docs/tickets.md、README、scripts/premerge.sh、scripts/regress.sh。
开球只写在进程内的场景卡上。邻机采样进后段的包装与 scripts/e-b4c-sphere-avoid.py 相同，
不改 policy.py / crossing.py 的文件。

编队 FormationFleet 不调用 refine_yopo，运行器也不为它建 UpstreamPolicy。
这层包装不会给编队新加后段。六档故障下开球因此不能靠邻机采样去推开或误排斥。
swap-8-open 是 goto / upstream，包装会把本 tick 广播、本 tick poll 到的采样留到下一次 plan，
再送进 refine_yopo。第 0 个规划拍仍然没有邻机。椭球项保持关闭。

开球臂都是 report-only / 偏离上游。未冻结，不进 scenes/suite.txt，不设 gates。
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
PARTIAL = ROOT / "out" / "e-b7-fault-and-swap8-sphere" / "partial"
FORMATION_SCENE = "scenes/formation-6-open.json"
SWAP_SCENE = "scenes/swap-8-open.json"
TIERS_PATH = "scenes/formation-6-tiers.json"
ROW_VERSION = 1

# 半径、权重。None 是关球，不写 backend。
ARM_SPEC: dict[str, dict | None] = {
    "off": None,
    "eb4c": {"ticket": "E-B4c", "radius": 0.85, "weight": 8.0},
    "eb5-r100": {"ticket": "E-B5", "radius": 1.00, "weight": 8.0},
    "eb5-w16": {"ticket": "E-B5", "radius": 0.85, "weight": 16.0},
}
FORMATION_ARMS = ("off", "eb4c", "eb5-r100")
SWAP_ARMS = ("off", "eb4c", "eb5-r100", "eb5-w16")
# 六档 × 三臂。编队不跑网络，种子放到 0–4，与场景卡 seeds 以及 M4 冻结表一致。
FORMATION_SEEDS = (0, 1, 2, 3, 4)
SWAP_SEEDS = (0, 1, 2, 3, 4)
GROUPS = ("formation", "swap8")

DISCLAIMER = (
    "只报数。report-only。peer_sphere_enabled 默认关，不改 swarm_penalty / swarm_segment_penalty 的语义。"
    "打开欧氏球即偏离上游 d5e1e60。未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。"
    "E-B4c 半径 0.85 m = 0.70 m 碰撞球 + 0.15 m 裕度，权重 8。"
    "E-B5 的 1.00 m / 权重 8，以及 0.85 m / 权重 16，同样只在本进程的卡上。"
    "这不是论文复现。"
)
WIRING = (
    "进程内包装与 scripts/e-b4c-sphere-avoid.py 的 install_peer_delivery 相同："
    "欧氏球打开时，runner.swarm_penalty_enabled 也视为要广播轨迹，"
    "UpstreamPolicy 留下同 tick poll 到的邻机采样，并在下一次 refine_yopo 送进去；"
    "CrossingFleet._apply_backend 同样在欧氏球打开时送已提交采样。两个开关都关时，送进后段的邻机仍是 None。"
    "不改 policy.py、crossing.py、formation.py 的文件，也不给 FormationFleet 新加 refine_yopo 调用点。"
    "formation 任务走预留协议，不进 YOPO。开球字段写在内存卡的 policy_params.backend 上，编队规划器不读。"
    "因此六档丢包、复制、乱序和时延不能通过过期采样让欧氏球去改队形或误排斥。"
    "swap-8-open 是 goto，包装会把采样送进后段。第 0 个规划拍策略还看不到邻机。"
)
_INSTALLED = False


def load_tiers() -> list[dict]:
    raw = json.loads((ROOT / TIERS_PATH).read_text(encoding="utf-8"))
    tiers = raw["tiers"] if isinstance(raw, dict) else raw
    if not tiers:
        raise SystemExit(f"{TIERS_PATH} 没有档位")
    return list(tiers)


def tier_ids() -> tuple[str, ...]:
    return tuple(str(tier["id"]) for tier in load_tiers())


def arm_spec(arm: str) -> dict | None:
    if arm not in ARM_SPEC:
        raise ValueError(arm)
    return ARM_SPEC[arm]


def arm_deviates(arm: str) -> bool:
    return arm_spec(arm) is not None


def arm_label(arm: str) -> str:
    spec = arm_spec(arm)
    if spec is None:
        return "off（关球）"
    radius = float(spec["radius"])
    weight = float(spec["weight"])
    weight_s = str(int(weight)) if weight == int(weight) else f"{weight:g}"
    return f"{spec['ticket']}(r={radius:.2f}, w={weight_s}) report-only / 偏离上游"


def _deep(raw: dict) -> dict:
    return json.loads(json.dumps(raw))


def build_card(group: str, arm: str):
    """进程内覆盖。不写 scenes/。关球臂返回加载器给出的原卡。"""
    from yopo_l0.scene import SceneCard, load_card

    if group == "formation":
        path = ROOT / FORMATION_SCENE
    elif group == "swap8":
        path = ROOT / SWAP_SCENE
    else:
        raise ValueError(group)
    loaded = load_card(path)
    spec = arm_spec(arm)
    if spec is None:
        return loaded
    raw = _deep(loaded.card)
    params = dict(raw.get("policy_params") or {})
    backend = dict(params.get("backend") or {})
    backend["enabled"] = True
    backend["peer_sphere_enabled"] = True
    backend["peer_sphere_radius_m"] = float(spec["radius"])
    backend["peer_sphere_weight"] = float(spec["weight"])
    backend.pop("swarm_penalty", None)
    backend.pop("swarm_segment_penalty", None)
    params["backend"] = backend
    raw["policy_params"] = params
    return SceneCard(loaded.path, raw, loaded.obstacles)


def install_peer_delivery() -> None:
    """欧氏球打开时把邻机采样送进已有的 refine_yopo。椭球项仍只看 swarm_penalty。

    与 scripts/e-b4c-sphere-avoid.py 同一层包装。不改编队规划器。
    """
    global _INSTALLED
    if _INSTALLED:
        return
    import yopo_l0.crossing as crossing_mod
    import yopo_l0.policy as policy_mod
    import yopo_l0.runner as runner_mod
    from yopo_l0.backend.config import peer_sphere_enabled as sphere_on

    orig_enabled = runner_mod.swarm_penalty_enabled

    def enabled(params, **kwargs):
        if orig_enabled(params, **kwargs):
            return True
        return bool(sphere_on(params, **kwargs))

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


def _budget_wrap():
    import yopo_l0.backend.bspline as bspline

    current = bspline.refine_yopo
    if getattr(current, "_eb7_wrapped", False):
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
    wrapped._eb7_wrapped = True
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


def iter_jobs(
    groups: list[str],
    formation_arms: list[str],
    swap_arms: list[str],
    formation_seeds: list[int],
    swap_seeds: list[int],
    tiers: list[str],
) -> list[tuple[str, str | None, str, int]]:
    jobs: list[tuple[str, str | None, str, int]] = []
    if "formation" in groups:
        for tier in tiers:
            for arm in formation_arms:
                for seed in formation_seeds:
                    jobs.append(("formation", tier, arm, int(seed)))
    if "swap8" in groups:
        for arm in swap_arms:
            for seed in swap_seeds:
                jobs.append(("swap8", None, arm, int(seed)))
    return jobs


def uses_refine_budget(group: str, arm: str) -> bool:
    """只有八机开球会进后段的 0.02 s 墙钟预算。编队开球不调用 refine_yopo。"""
    return group == "swap8" and arm_deviates(arm)


def _partial(group: str, tier: str | None, arm: str, seed: int) -> Path:
    tier_id = "na" if tier is None else tier
    return PARTIAL / f"{group}-{tier_id}-{arm}-{seed}.json"


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


def _row(group: str, tier: str | None, arm: str, result: dict, *, budget_calls: int, budget_cuts: int) -> dict:
    reason = result.get("reason") or ""
    outcome = result.get("outcome")
    backend = _drop_samples(result.get("backend"))
    time_ms = (backend or {}).get("time_ms") or {}
    spec = arm_spec(arm)
    bus = result.get("bus") or {}
    deviates = arm_deviates(arm)
    return {
        "row_version": ROW_VERSION,
        "group": group,
        "tier": tier,
        "arm": arm,
        "arm_label": arm_label(arm),
        "scene": result.get("scene"),
        "seed": int(result["seed"]),
        "outcome": outcome,
        "reason": reason,
        "arrived": int(result["arrived"]),
        "n_drones": int(result["n_drones"]),
        "arrival_time_s": result.get("arrival_time_s"),
        "formation_error_p95_m": result.get("formation_error_p95_m"),
        "obstacle_collisions": int(outcome == "COLLISION" and "TRUTH_POSITION" in str(reason)),
        "inter_drone_collisions": int(outcome == "COLLISION" and "INTER_DRONE" in str(reason)),
        "timeout": int(outcome == "TIMEOUT"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "min_obstacle_distance_m": result.get("min_obstacle_distance_m"),
        "sim_duration_s": result.get("sim_duration_s"),
        "final_goal_error_m": result.get("final_goal_error_m"),
        "wall_time_s": result.get("wall_time_s"),
        "backend_time_p50_ms": time_ms.get("p50"),
        "backend_time_p95_ms": time_ms.get("p95"),
        "backend_calls": None if backend is None else int(backend.get("calls") or 0),
        "backend_budget_calls": int(budget_calls),
        "backend_budget_cuts": int(budget_cuts),
        "bus_published": bus.get("published"),
        "bus_dropped": bus.get("dropped"),
        "bus_duplicated": bus.get("duplicated"),
        "bus_delivered": bus.get("delivered"),
        "bus_reordered": bus.get("reordered"),
        "scene_card_hash": result.get("scene_card_hash"),
        "commit": result.get("commit"),
        "policy": result.get("policy"),
        "task": result.get("task"),
        "report_only": True,
        "frozen": False,
        "deviates_from_upstream": deviates,
        "runner_upstream_deviation": result.get("upstream_deviation"),
        "peer_sphere_enabled": deviates,
        "peer_sphere_radius_m": None if spec is None else float(spec["radius"]),
        "peer_sphere_weight": None if spec is None else float(spec["weight"]),
        "swarm_penalty": False,
        "disclaimer": DISCLAIMER,
    }


def run_one(group: str, tier: str | None, arm: str, seed: int, *, resume: bool) -> dict:
    dest = _partial(group, tier, arm, seed)
    if resume and dest.is_file():
        row = json.loads(dest.read_text(encoding="utf-8"))
        if int(row.get("row_version", 0)) == ROW_VERSION:
            return row
    from yopo_l0.bus import BusFaults
    from yopo_l0.runner import RunConfig, run

    install_peer_delivery()
    card = build_card(group, arm)
    bus = None
    policy = "upstream"
    if group == "formation":
        policy = "formation"
        by_id = {str(item["id"]): item for item in load_tiers()}
        if tier not in by_id:
            raise ValueError(tier)
        bus = BusFaults.from_mapping(by_id[tier].get("bus"))
    wrapped = _budget_wrap()
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(seed), policy=policy, shadow=True, bus=bus))
    row = _row(
        group,
        tier,
        arm,
        out.result,
        budget_calls=int(wrapped.calls),
        budget_cuts=int(wrapped.cuts),
    )
    row["worker_s"] = time.perf_counter() - t0
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)
    gap = row["min_inter_drone_distance_m"]
    p95 = row["formation_error_p95_m"]
    print(
        f"{group} tier={tier or '—'} {arm} seed={seed} {row['outcome']} "
        f"{row['arrived']}/{row['n_drones']} inter={row['inter_drone_collisions']} "
        f"gap={_f(gap, 3)} p95={_f(p95, 3)} arrive={_f(row['arrival_time_s'], 2)} "
        f"refine={row['backend_budget_calls']} cuts={row['backend_budget_cuts']} "
        f"drop={row['bus_dropped']} wall={_f(row['wall_time_s'], 1)}s",
        flush=True,
    )
    return row


def _job(spec):
    group, tier, arm, seed, resume = spec
    return run_one(group, tier, arm, int(seed), resume=bool(resume))


def _run_group(jobs: list[tuple[str, str | None, str, int]], *, workers: int, resume: bool) -> None:
    specs = [(group, tier, arm, seed, resume) for group, tier, arm, seed in jobs]
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


def _load_partials(jobs: list[tuple[str, str | None, str, int]]) -> list[dict]:
    rows = []
    for group, tier, arm, seed in jobs:
        path = _partial(group, tier, arm, seed)
        if path.is_file():
            row = json.loads(path.read_text(encoding="utf-8"))
            if int(row.get("row_version", 0)) == ROW_VERSION:
                rows.append(row)
    return rows


def _sort_key(row: dict):
    group = row["group"]
    tiers = tier_ids()
    tier = row.get("tier")
    tier_rank = 99 if tier is None else tiers.index(str(tier))
    arms = FORMATION_ARMS if group == "formation" else SWAP_ARMS
    return (GROUPS.index(group), tier_rank, arms.index(row["arm"]), int(row["seed"]))


def formation_flight_key(row: dict) -> tuple:
    """和关球比轨迹结果时用。墙钟、哈希、总线计数不在这里。"""
    return (
        row.get("outcome"),
        row.get("reason") or "",
        int(row["arrived"]),
        int(row["n_drones"]),
        row.get("arrival_time_s"),
        row.get("formation_error_p95_m"),
        row.get("min_inter_drone_distance_m"),
        int(row["obstacle_collisions"]),
        int(row["inter_drone_collisions"]),
        int(row["timeout"]),
        row.get("sim_duration_s"),
        int(row.get("backend_budget_calls") or 0),
    )


def formation_arm_mismatches(rows: list[dict]) -> list[dict]:
    """同一档位、同一种子，开球臂与关球的飞行结果不一致就记一条。"""
    off = {
        (r["tier"], int(r["seed"])): r
        for r in rows
        if r["group"] == "formation" and r["arm"] == "off"
    }
    bad = []
    for row in rows:
        if row["group"] != "formation" or row["arm"] == "off":
            continue
        base = off.get((row["tier"], int(row["seed"])))
        if base is None:
            bad.append({"tier": row["tier"], "arm": row["arm"], "seed": row["seed"], "missing_off": True})
            continue
        if formation_flight_key(row) != formation_flight_key(base):
            bad.append(
                {
                    "tier": row["tier"],
                    "arm": row["arm"],
                    "seed": int(row["seed"]),
                    "off": formation_flight_key(base),
                    "sphere": formation_flight_key(row),
                }
            )
    return bad


def _collisions(row: dict) -> int:
    return int(row["obstacle_collisions"]) + int(row["inter_drone_collisions"])


def _arrival_cell(times: list[float], n_seeds: int) -> str:
    if not times:
        return f"—（0/{n_seeds} 粒）"
    mean = sum(times) / len(times)
    if len(times) == 1 or abs(max(times) - min(times)) < 0.005:
        return f"{mean:.2f}（{len(times)}/{n_seeds} 粒）"
    return (
        f"{mean:.2f}（{len(times)}/{n_seeds} 粒，范围 {_f(min(times), 2)}–{_f(max(times), 2)}）"
    )


def render_group1(rows: list[dict]) -> str:
    lines = [
        "| 档位 | 臂 | 到达 | 碰撞 | 队形误差 p95 m | 最小机间 m | 后段调用 | 丢包 |",
        "|---|---|---|---|---|---|---|---|",
    ]
    present = [r for r in rows if r["group"] == "formation"]
    tiers = []
    for row in present:
        if row["tier"] not in tiers:
            tiers.append(row["tier"])
    for tier in tiers:
        for arm in FORMATION_ARMS:
            group = [r for r in present if r["tier"] == tier and r["arm"] == arm]
            if not group:
                continue
            arrived = sum(int(r["arrived"]) for r in group)
            total = sum(int(r["n_drones"]) for r in group)
            hits = sum(_collisions(r) for r in group)
            p95 = [float(r["formation_error_p95_m"]) for r in group if r.get("formation_error_p95_m") is not None]
            gaps = [float(r["min_inter_drone_distance_m"]) for r in group if r.get("min_inter_drone_distance_m") is not None]
            calls = sum(int(r.get("backend_budget_calls") or 0) for r in group)
            drops = [float(r["bus_dropped"]) for r in group if r.get("bus_dropped") is not None]
            lines.append(
                f"| {tier} | {arm_label(arm)} | {arrived}/{total} | {hits} | {_rng(p95, 3)} | "
                f"{_rng(gaps, 3)} | {calls} | {_rng(drops, 0)} |"
            )
    return "\n".join(lines)


def render_group1_seeds(rows: list[dict]) -> str:
    lines = [
        "| 档位 | 臂 | seed | 结果 | 到达 | 队形误差 p95 m | 最小机间 m | 到达时间 s | 后段调用 | 丢包 | 乱序 |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    present = [r for r in rows if r["group"] == "formation"]
    present.sort(key=_sort_key)
    for row in present:
        lines.append(
            f"| {row['tier']} | {row['arm_label']} | {int(row['seed'])} | {row['outcome']} | "
            f"{row['arrived']}/{row['n_drones']} | {_f(row.get('formation_error_p95_m'), 3)} | "
            f"{_f(row.get('min_inter_drone_distance_m'), 3)} | {_f(row.get('arrival_time_s'), 2)} | "
            f"{int(row.get('backend_budget_calls') or 0)} | {row.get('bus_dropped')} | {row.get('bus_reordered')} |"
        )
    return "\n".join(lines)


def render_group2(rows: list[dict]) -> str:
    lines = [
        "| 臂 | 到达 | 机间碰撞 | 超时 | 最小机间 m | 平均到达时间 s | 后段调用 | 预算截断 |",
        "|---|---|---|---|---|---|---|---|",
    ]
    present = [r for r in rows if r["group"] == "swap8"]
    for arm in SWAP_ARMS:
        group = [r for r in present if r["arm"] == arm]
        if not group:
            continue
        arrived = sum(int(r["arrived"]) for r in group)
        total = sum(int(r["n_drones"]) for r in group)
        inter = sum(int(r["inter_drone_collisions"]) for r in group)
        timeout = sum(int(r["timeout"]) for r in group)
        gaps = [float(r["min_inter_drone_distance_m"]) for r in group if r.get("min_inter_drone_distance_m") is not None]
        times = [float(r["arrival_time_s"]) for r in group if r.get("arrival_time_s") is not None]
        calls = sum(int(r.get("backend_budget_calls") or 0) for r in group)
        cuts = sum(int(r.get("backend_budget_cuts") or 0) for r in group)
        lines.append(
            f"| {arm_label(arm)} | {arrived}/{total} | {inter} | {timeout} | {_rng(gaps, 3)} | "
            f"{_arrival_cell(times, len(group))} | {calls} | {cuts}/{calls} |"
        )
    return "\n".join(lines)


def render_group2_seeds(rows: list[dict]) -> str:
    lines = [
        "| 臂 | seed | 结果 | 原因 | 到达 | 机间 | 最小机间 m | 到达时间 s | 后段 p50 / p95 ms | 截断 | 墙钟 s |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    present = [r for r in rows if r["group"] == "swap8"]
    present.sort(key=_sort_key)
    for row in present:
        calls = int(row.get("backend_budget_calls") or 0)
        cuts = int(row.get("backend_budget_cuts") or 0)
        if row.get("backend_time_p50_ms") is None:
            backend_t = "—"
        else:
            backend_t = f"{_f(row.get('backend_time_p50_ms'), 1)} / {_f(row.get('backend_time_p95_ms'), 1)}"
        lines.append(
            f"| {row['arm_label']} | {int(row['seed'])} | {row['outcome']} | {row.get('reason') or '—'} | "
            f"{row['arrived']}/{row['n_drones']} | {int(row['inter_drone_collisions'])} | "
            f"{_f(row.get('min_inter_drone_distance_m'), 3)} | {_f(row.get('arrival_time_s'), 2)} | "
            f"{backend_t} | {cuts}/{calls} | {_f(row.get('wall_time_s'), 1)} |"
        )
    return "\n".join(lines)


def _parse_ints(text: str) -> list[int]:
    return [int(s) for s in text.split(",") if s != ""]


def _parse_names(text: str) -> list[str]:
    return [s for s in text.split(",") if s]


def main(argv: list[str] | None = None) -> int:
    tiers_all = load_tiers()
    ap = argparse.ArgumentParser()
    ap.add_argument("--groups", default=",".join(GROUPS))
    ap.add_argument("--formation-arms", default=",".join(FORMATION_ARMS))
    ap.add_argument("--swap-arms", default=",".join(SWAP_ARMS))
    ap.add_argument("--formation-seeds", default=",".join(str(s) for s in FORMATION_SEEDS))
    ap.add_argument("--swap-seeds", default=",".join(str(s) for s in SWAP_SEEDS))
    ap.add_argument("--tiers", default=",".join(tier_ids()))
    ap.add_argument("--jobs-plain", type=int, default=2)
    ap.add_argument("--jobs-backend", type=int, default=1)
    ap.add_argument("--out", default="docs/research/e-b7-fault-and-swap8-sphere-runs.json")
    ap.add_argument("--no-resume", action="store_true")
    ap.add_argument("--check", action="store_true", help="只检查任务网格，不跑仿真")
    a = ap.parse_args(argv)

    groups = _parse_names(a.groups)
    formation_arms = _parse_names(a.formation_arms)
    swap_arms = _parse_names(a.swap_arms)
    formation_seeds = _parse_ints(a.formation_seeds)
    swap_seeds = _parse_ints(a.swap_seeds)
    wanted = _parse_names(a.tiers)
    known = {str(tier["id"]): tier for tier in tiers_all}
    unknown = (
        [name for name in groups if name not in GROUPS]
        + [name for name in formation_arms if name not in FORMATION_ARMS]
        + [name for name in swap_arms if name not in SWAP_ARMS]
        + [name for name in wanted if name not in known]
    )
    if unknown:
        raise SystemExit(f"未知分组、臂或档位: {unknown}")
    if int(a.jobs_plain) > 2 or int(a.jobs_backend) > 2:
        raise SystemExit("最多 2 个进程，避免墙钟预算把后段迭代截短")
    if int(a.jobs_plain) < 1 or int(a.jobs_backend) < 1:
        raise SystemExit("进程数至少为 1")
    jobs = iter_jobs(groups, formation_arms, swap_arms, formation_seeds, swap_seeds, wanted)
    if a.check:
        print(f"jobs {len(jobs)}")
        return 0

    resume = not a.no_resume
    from yopo_swarm_core.baseline import fetch_upstream

    fetch_upstream()
    plain = [job for job in jobs if not uses_refine_budget(job[0], job[2])]
    backend = [job for job in jobs if uses_refine_budget(job[0], job[2])]
    if plain:
        _run_group(plain, workers=max(1, int(a.jobs_plain)), resume=resume)
    if backend:
        _run_group(backend, workers=max(1, int(a.jobs_backend)), resume=resume)

    rows = _load_partials(jobs)
    rows.sort(key=_sort_key)
    mismatches = formation_arm_mismatches(rows)
    payload = {
        "disclaimer": DISCLAIMER,
        "wiring": WIRING,
        "report_only": True,
        "frozen": False,
        "default_off": True,
        "deviates_from_upstream_when_enabled": True,
        "formation_calls_refine_yopo": False,
        "collision_ball_m": 0.70,
        "neighbor_delivery": (
            "in-process wrap of runner.swarm_penalty_enabled, "
            "UpstreamPolicy._keep_neighbor_samples, UpstreamPolicy._apply_backend, "
            "CrossingFleet._apply_backend; same pattern as scripts/e-b4c-sphere-avoid.py"
        ),
        "note": (
            "开球臂都标 report-only / 偏离上游。"
            "编队不调用 refine_yopo，后段调用应为 0，飞行结果应与同档同种子的关球一致。"
            "八机对向的平均到达时间只对 arrival_time_s 非空的种子取平均，也就是六架都进到达判据的那些种子；"
            "没全到的种子不进平均。机间碰撞一粒种子最多记 1，仿真在第一次中心距 ≤ 0.70 m 时停下。"
            "最小机间：没撞是全程最小，撞了是停下那一拍的采样。"
            "墙钟不参与轨迹。八机开球的后段 0.02 s 预算按墙钟截断，这两臂用一个进程。"
            "编队开球不进后段，和关球一起最多两个进程。"
        ),
        "groups": groups,
        "formation_scene": FORMATION_SCENE,
        "swap_scene": SWAP_SCENE,
        "tiers_file": TIERS_PATH,
        "formation_arms": formation_arms,
        "swap_arms": swap_arms,
        "formation_seeds": formation_seeds,
        "swap_seeds": swap_seeds,
        "tiers": wanted,
        "completed": len(rows),
        "expected": len(jobs),
        "incomplete": len(jobs) - len(rows),
        "formation_arm_mismatches": mismatches,
        "formation_arms_match_off": len(mismatches) == 0 and any(r["group"] == "formation" for r in rows),
        "rows": rows,
    }
    out = ROOT / a.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(render_group1(rows))
    print()
    print(render_group2(rows))
    print(f"formation_arms_match_off={payload['formation_arms_match_off']} mismatches={len(mismatches)}")
    print(f"wrote {out} ({payload['completed']}/{payload['expected']})")
    return 0 if payload["incomplete"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
