#!/usr/bin/env python3
"""E-C4 三档密度场 L0 seed 0，只报数。

几何已经在 main：scenes/ego-density-014.json / 028 / 042。
本脚本生成飞行卡、在进程里覆盖开关、跑 seed 0，并写出 runs json。

两臂：
  off    默认全关，场景卡上不写开关。
  combo  E-A1 + D1 + E-B2，覆盖方式与 scripts/e-combo-a1-d1-b2.py 相同。
         不把开关写进已提交的场景卡。

不改默认，不冻结，不进 scenes/suite.txt。不跑 10 机，不为填论文表 II 去跑全种子。
这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。

后段预算按墙钟截断。本脚本默认 --jobs 1，跑 combo 时不要同时再跑别的重活。
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-c4-density-l0" / "partial"

BODY_RADIUS_M = 0.35
OBSTACLE_RADIUS_M = 0.20
CLEARANCE_MARGIN_M = 0.15
AXIS_CLEARANCE_M = 0.70  # BODY + 障碍半径 + 余量，写成字面量避免 0.35+0.20+0.15 的二进制尾差
MAP_SIDE_M = 20.0
MAP_HALF_M = MAP_SIDE_M / 2.0
NOMINAL_SWAP_RADIUS_M = 8.0
SOLO_NOMINAL_X_M = 9.0
HEIGHT_M = 1.5
RADIUS_PAD_M = 0.05
DISCLAIMER = "这是假设，不是论文复现"

# 与 swap-3-open.json 同一组方位：0°、120°、240°。
SWAP_UNIT = (
    (1.0, 0.0),
    (-0.5, math.sqrt(3.0) / 2.0),
    (-0.5, -math.sqrt(3.0) / 2.0),
)

DENSITIES = (
    ("014", 0.14, 56, "ego-density-014.json"),
    ("028", 0.28, 112, "ego-density-028.json"),
    ("042", 0.42, 168, "ego-density-042.json"),
)

# 几何文件字节 sha256，与 docs/research/e-c4-eight-swap-density.md 一致。本票不得改这三份文件。
GEOM_SHA256 = {
    "ego-density-014.json": "5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9",
    "ego-density-028.json": "e43d320ad66ac47732ccb39c1a355a6152d7ff7b0139cae76f5b7222a1f28d60",
    "ego-density-042.json": "ee613b50ab19f9c4f2438f3298f79baab8f88f90ce7ebc36aacd0750ed7f8a97",
}

ARMS = ("off", "combo")
KINDS = ("swap3", "solo")

# 与 E-A1 报告同一组数：滤掉，不减分。0.70 m 是机体半径的两倍，也是机间碰撞线。
_EA1 = {
    "inter_drone_filter": True,
    "inter_drone_threshold_m": 0.70,
    "inter_drone_penalty": 0,
}
# 与 D1 的对向变体同一组数。
_D1 = {
    "enabled": True,
    "inter_drone_distance_m": 0.7,
    "commit_horizon_s": 1.0,
}

CAMERA = {
    "width": 160, "height": 96, "hfov_deg": 90.0, "vfov_deg": 60.0,
    "min_range_m": 0.2, "max_range_m": 20.0, "noise_std_m": 0.02,
    "mount_xyz_body": [0.13, 0.0, 0.02],
}
DYNAMICS = {"model": "point_mass_lag", "tau_s": 0.1, "kp": 4.0, "kd": 3.0, "a_max": 6.0}
RATES = {"physics": 100, "camera": 30, "planner": 10}
CRITERIA = {
    "body_radius_m": BODY_RADIUS_M,
    "goal_tolerance_m": 0.4,
    "goal_speed_mps": 0.2,
    "goal_hold_s": 1.0,
    "frozen": False,
}


def _deep(raw: dict) -> dict:
    return json.loads(json.dumps(raw))


def cylinder_centers(geom: dict) -> list[tuple[float, float]]:
    return [(float(c["center_xy"][0]), float(c["center_xy"][1])) for c in geom["cylinders"]]


def load_geometry(name: str) -> dict:
    return json.loads((ROOT / "scenes" / name).read_text(encoding="utf-8"))


def axis_distance(x: float, y: float, centers: list[tuple[float, float]]) -> float:
    return min(math.hypot(x - cx, y - cy) for cx, cy in centers)


def outside_square(x: float, y: float) -> bool:
    return abs(x) > MAP_HALF_M or abs(y) > MAP_HALF_M


def _unit(ux: float, uy: float) -> tuple[float, float]:
    n = math.hypot(ux, uy)
    if n <= 0.0:
        raise ValueError("零向量")
    return ux / n, uy / n


def _bisect_min_ok(lo: float, hi: float, ok, steps: int = 80) -> float:
    if ok(lo):
        return lo
    if not ok(hi):
        raise RuntimeError(f"在 {hi} 仍找不到净空足够的位置")
    a, b = lo, hi
    for _ in range(steps):
        mid = (a + b) / 2.0
        if ok(mid):
            b = mid
        else:
            a = mid
    return b


def first_clear_swap_radius(ux: float, uy: float, centers: list[tuple[float, float]]) -> tuple[float, float, float]:
    """返回 (写入卡的半径, 刚满足净空的半径, 正方形边界半径)。"""
    ux, uy = _unit(ux, uy)

    def ok(r: float) -> bool:
        sx, sy = r * ux, r * uy
        gx, gy = -sx, -sy
        return (
            outside_square(sx, sy)
            and outside_square(gx, gy)
            and axis_distance(sx, sy, centers) >= AXIS_CLEARANCE_M - 1e-12
            and axis_distance(gx, gy, centers) >= AXIS_CLEARANCE_M - 1e-12
        )

    r_edge = MAP_HALF_M / max(abs(ux), abs(uy))
    r_min = _bisect_min_ok(r_edge + 1e-12, r_edge + 8.0, ok)
    return r_min + RADIUS_PAD_M, r_min, r_edge


def first_clear_solo_x(centers: list[tuple[float, float]]) -> tuple[float, float]:
    """返回 (写入卡的 |x|, 刚满足净空的 |x|)。"""

    def ok(x: float) -> bool:
        return (
            x > MAP_HALF_M
            and axis_distance(-x, 0.0, centers) >= AXIS_CLEARANCE_M - 1e-12
            and axis_distance(x, 0.0, centers) >= AXIS_CLEARANCE_M - 1e-12
        )

    x_min = _bisect_min_ok(MAP_HALF_M + 1e-12, MAP_HALF_M + 8.0, ok)
    return x_min + RADIUS_PAD_M, x_min


def place_swap3(centers: list[tuple[float, float]]) -> list[dict]:
    drones = []
    for i, (ux, uy) in enumerate(SWAP_UNIT):
        ux, uy = _unit(ux, uy)
        x8, y8 = NOMINAL_SWAP_RADIUS_M * ux, NOMINAL_SWAP_RADIUS_M * uy
        d_start = axis_distance(x8, y8, centers)
        d_goal = axis_distance(-x8, -y8, centers)
        keep = d_start >= AXIS_CLEARANCE_M - 1e-12 and d_goal >= AXIS_CLEARANCE_M - 1e-12
        if keep:
            r = NOMINAL_SWAP_RADIUS_M
            sx, sy = x8, y8
            pushed = False
            r_min = r_edge = None
        else:
            r, r_min, r_edge = first_clear_swap_radius(ux, uy, centers)
            sx, sy = r * ux, r * uy
            pushed = True
        drones.append(
            {
                "id": f"uav{i}",
                "start_enu": [sx, sy, HEIGHT_M],
                "goal_enu": [-sx, -sy, HEIGHT_M],
                "nominal_radius_m": NOMINAL_SWAP_RADIUS_M,
                "radius_m": r,
                "pushed_outside": pushed,
                "axis_distance_at_8m_start": d_start,
                "axis_distance_at_8m_goal": d_goal,
                "axis_distance_start": axis_distance(sx, sy, centers),
                "axis_distance_goal": axis_distance(-sx, -sy, centers),
                "r_min_clear_m": r_min,
                "r_square_edge_m": r_edge,
            }
        )
    return drones


def place_solo(centers: list[tuple[float, float]]) -> dict:
    x0 = SOLO_NOMINAL_X_M
    d_start = axis_distance(-x0, 0.0, centers)
    d_goal = axis_distance(x0, 0.0, centers)
    keep = d_start >= AXIS_CLEARANCE_M - 1e-12 and d_goal >= AXIS_CLEARANCE_M - 1e-12
    if keep:
        x, x_min, pushed = x0, None, False
    else:
        x, x_min = first_clear_solo_x(centers)
        pushed = True
    return {
        "id": "uav0",
        "start_enu": [-x, 0.0, HEIGHT_M],
        "goal_enu": [x, 0.0, HEIGHT_M],
        "nominal_x_m": SOLO_NOMINAL_X_M,
        "x_m": x,
        "pushed_outside": pushed,
        "axis_distance_at_pm9_start": d_start,
        "axis_distance_at_pm9_goal": d_goal,
        "axis_distance_start": axis_distance(-x, 0.0, centers),
        "axis_distance_goal": axis_distance(x, 0.0, centers),
        "x_min_clear_m": x_min,
    }


def card_name(tag: str, kind: str) -> str:
    return f"ego-density-{tag}-{kind}"


def card_path(tag: str, kind: str) -> Path:
    return ROOT / "scenes" / f"{card_name(tag, kind)}.json"


def _swap_note(tag: str, density: float, n: int, geom: str, drones: list[dict]) -> str:
    moved = [d for d in drones if d["pushed_outside"]]
    kept = [d for d in drones if not d["pushed_outside"]]
    parts = [
        f"票 E-C4 密度场 L0 seed 0：三机 goto 对向，几何是 scenes/{geom}（{n} 根圆柱，目标密度 {density} 个/m²）。",
        f"{DISCLAIMER}（障碍半径 {OBSTACLE_RADIUS_M:.2f} m，边长 {MAP_SIDE_M:.1f} m）。论文表 II 没有给出这两个数。",
        "起点名义上均匀分布在半径 8.0 m、高度 1.5 m 的圆上，各自飞向直径对侧，方位与 scenes/swap-3-open.json 相同。",
        "起终点必须在自由空间：机体中心到最近圆柱轴的水平距离至少 0.70 m（机体半径 0.35 m + 障碍半径 0.20 m + 余量 0.15 m）。",
    ]
    if moved:
        bits = []
        for d in moved:
            bits.append(
                f"{d['id']} 在 8 m 圆上到最近轴是 {d['axis_distance_at_8m_start']:.3f} m / "
                f"{d['axis_distance_at_8m_goal']:.3f} m，低于 0.70 m，"
                f"因此沿同一条射线把起终点移到 20 m 正方形外面（半径 {d['radius_m']:.3f} m，"
                f"刚满足净空后再加 {RADIUS_PAD_M:.2f} m）"
            )
        parts.append("；".join(bits) + "。")
    if kept:
        ids = "、".join(d["id"] for d in kept)
        parts.append(f"{ids} 在 8 m 圆上已经满足 0.70 m，仍留在圆上。")
    parts.append(
        "只跑 seed 0，不进 scenes/suite.txt，criteria.frozen=false，不设 gates。"
        "开关不写进本卡：默认全关是一臂，E-A1、D1、E-B2 全开由 scripts/e-c4-density-l0.py 在进程里覆盖，写法与 scripts/e-combo-a1-d1-b2.py 相同。"
        "本卡不是 10 机，也不为填写论文表 II 去跑全种子。"
        "速度与 a_max 用本仓库已有的点质量设定：场景卡无 speed_mps（goto 任务如此），dynamics.a_max=6.0、tau_s=0.1、kp=4.0、kd=3.0。"
    )
    return "".join(parts)


def _solo_note(tag: str, density: float, n: int, geom: str, placed: dict) -> str:
    parts = [
        f"票 E-C4 密度场 L0 seed 0：单机 goto 穿越，几何是 scenes/{geom}（{n} 根圆柱，目标密度 {density} 个/m²）。",
        f"{DISCLAIMER}（障碍半径 {OBSTACLE_RADIUS_M:.2f} m，边长 {MAP_SIDE_M:.1f} m）。论文表 II 没有给出这两个数。",
        "名义航线与 D2 教师单机相同：y = 0，x 从 −9 m 到 +9 m；高度用本仓库 L0 的 1.5 m，不是教师的 1.0 m。",
        "对照的是同一组密度假设，不是复现教师轨迹。D2 教师在密度 0.42 时单机严格成功 9/16。",
        "机体半径和到达口径用 L0 已有设定（0.35 m，0.4 m / 0.2 m/s / 1 s），不是教师的 0.20 m / 0.5 m / 0.3 m/s。",
        "起终点必须在自由空间：机体中心到最近圆柱轴的水平距离至少 0.70 m（机体半径 0.35 m + 障碍半径 0.20 m + 余量 0.15 m）。",
    ]
    if placed["pushed_outside"]:
        parts.append(
            f"x = ±9 m 时到最近轴是 {placed['axis_distance_at_pm9_start']:.3f} m / "
            f"{placed['axis_distance_at_pm9_goal']:.3f} m，低于 0.70 m，"
            f"因此沿 x 轴把起终点移到 20 m 正方形外面（|x| = {placed['x_m']:.3f} m，"
            f"刚满足净空后再加 {RADIUS_PAD_M:.2f} m）。"
        )
    else:
        parts.append("x = ±9 m 已经满足 0.70 m，仍用这对起终点。")
    parts.append(
        "只跑 seed 0，不进 scenes/suite.txt，criteria.frozen=false，不设 gates。"
        "开关不写进本卡：默认全关是一臂，E-A1、D1、E-B2 全开由 scripts/e-c4-density-l0.py 在进程里覆盖，写法与 scripts/e-combo-a1-d1-b2.py 相同。"
        "本卡不是 10 机，也不为填写论文表 II 去跑全种子。"
        "速度与 a_max 用本仓库已有的点质量设定：场景卡无 speed_mps（goto 任务如此），dynamics.a_max=6.0、tau_s=0.1、kp=4.0、kd=3.0。"
    )
    return "".join(parts)


def _coord(v: float) -> float:
    # 避免 0.0 的反向写成 -0.0，和 swap-3-open.json 一样写 0.0。
    x = float(v)
    return 0.0 if x == 0.0 else x


def _drone_public(d: dict) -> dict:
    return {
        "id": d["id"],
        "start_enu": [_coord(d["start_enu"][0]), _coord(d["start_enu"][1]), _coord(d["start_enu"][2])],
        "goal_enu": [_coord(d["goal_enu"][0]), _coord(d["goal_enu"][1]), _coord(d["goal_enu"][2])],
    }


def build_card(tag: str, density: float, n: int, geom: str, kind: str, centers: list[tuple[float, float]]) -> dict:
    name = card_name(tag, kind)
    if kind == "swap3":
        placed = place_swap3(centers)
        note = _swap_note(tag, density, n, geom, placed)
        drones = [_drone_public(d) for d in placed]
        duration = 40.0
    elif kind == "solo":
        placed = place_solo(centers)
        note = _solo_note(tag, density, n, geom, placed)
        drones = [_drone_public(placed)]
        duration = 50.0
    else:
        raise ValueError(kind)
    return {
        "name": name,
        "note": note,
        "geometry": geom,
        "ground_z": 0.0,
        "drones": drones,
        "task": {"type": "goto", "duration_s": duration},
        "criteria": dict(CRITERIA),
        "camera": dict(CAMERA),
        "dynamics": dict(DYNAMICS),
        "rates_hz": dict(RATES),
        "seeds": [0],
    }


def _dumps_card(card: dict) -> str:
    """接近 swap-3-open.json 的排版：无人机、相机各占一行。"""
    drones = ",\n    ".join(json.dumps(d, ensure_ascii=False) for d in card["drones"])
    criteria = json.dumps(card["criteria"], indent=2, ensure_ascii=False)
    criteria = "\n".join(
        ("  " + line if i else line) for i, line in enumerate(criteria.splitlines())
    )
    camera = json.dumps(card["camera"], ensure_ascii=False)
    dynamics = json.dumps(card["dynamics"], ensure_ascii=False)
    rates = json.dumps(card["rates_hz"], ensure_ascii=False)
    note = json.dumps(card["note"], ensure_ascii=False)
    return (
        "{\n"
        f'  "name": {json.dumps(card["name"])},\n'
        f'  "note": {note},\n'
        f'  "geometry": {json.dumps(card["geometry"])},\n'
        f'  "ground_z": 0.0,\n'
        f'  "drones": [\n    {drones}\n  ],\n'
        f'  "task": {json.dumps(card["task"], ensure_ascii=False)},\n'
        f'  "criteria": {criteria},\n'
        f'  "camera": {camera},\n'
        f'  "dynamics": {dynamics},\n'
        f'  "rates_hz": {rates},\n'
        f'  "seeds": [0]\n'
        "}\n"
    )


def expected_cards() -> dict[str, dict]:
    out = {}
    for tag, density, n, geom in DENSITIES:
        centers = cylinder_centers(load_geometry(geom))
        if len(centers) != n:
            raise RuntimeError(f"{geom} 圆柱数 {len(centers)}，期望 {n}")
        for kind in KINDS:
            card = build_card(tag, density, n, geom, kind, centers)
            out[card_path(tag, kind).name] = card
    return out


def write_cards() -> list[Path]:
    written = []
    for name, card in expected_cards().items():
        path = ROOT / "scenes" / name
        path.write_text(_dumps_card(card), encoding="utf-8")
        written.append(path)
    return written


def _with_flags(raw: dict, arm: str) -> dict:
    """只加这一臂要开的字段。不改 criteria，不改任务。与 e-combo-a1-d1-b2.py 相同。"""
    out = _deep(raw)
    if arm == "combo":
        params = dict(out.get("policy_params") or {})
        upstream = dict(params.get("upstream") or {})
        upstream.update(_EA1)
        params["upstream"] = upstream
        backend = dict(params.get("backend") or {})
        backend["enabled"] = True
        params["backend"] = backend
        out["policy_params"] = params
        out["geometric_fallback"] = dict(_D1)
    return out


def _materialize(raw: dict):
    from yopo_l0.scene import SceneCard, parse_geometry

    card = _deep(raw)
    geom = json.loads((ROOT / "scenes" / card["geometry"]).read_text(encoding="utf-8"))
    card = {**card, "geometry": geom}
    return SceneCard(card["name"], card, parse_geometry(geom, card.get("ground_z")))


def build_run_card(tag: str, kind: str, arm: str):
    raw = json.loads(card_path(tag, kind).read_text(encoding="utf-8"))
    if arm == "off":
        from yopo_l0.scene import load_card

        return load_card(card_path(tag, kind))
    if arm == "combo":
        return _materialize(_with_flags(raw, arm))
    raise ValueError(arm)


def _budget_wrap():
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
    clearance = None if dist is None else float(dist) - BODY_RADIUS_M
    reason = result.get("reason") or ""
    outcome = result.get("outcome")
    tree = int(outcome == "COLLISION" and "TRUTH_POSITION" in str(reason))
    inter = int(outcome == "COLLISION" and "INTER_DRONE" in str(reason))
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
        "arrival_time_s": result.get("arrival_time_s"),
        "tree_collisions": tree,
        "inter_drone_collisions": inter,
        "timeout": int(outcome == "TIMEOUT"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "min_obstacle_distance_m": dist,
        "min_clearance_m": clearance,
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


def scene_key(tag: str, kind: str) -> str:
    return f"{tag}-{kind}"


def run_one(tag: str, kind: str, arm: str, seed: int, *, resume: bool) -> dict:
    key = scene_key(tag, kind)
    dest = _partial(key, arm, seed)
    if resume and dest.is_file():
        return json.loads(dest.read_text(encoding="utf-8"))
    from yopo_l0.runner import RunConfig, run

    card = build_run_card(tag, kind, arm)
    wrapped = _budget_wrap()
    cfg = RunConfig(seed=int(seed), shadow=True)
    t0 = time.perf_counter()
    out = run(card, cfg)
    row = _row(
        key,
        arm,
        out.result,
        out.ticks,
        budget_calls=int(wrapped.calls),
        budget_cuts=int(wrapped.cuts),
    )
    row["density_tag"] = tag
    row["kind"] = kind
    row["worker_s"] = time.perf_counter() - t0
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)
    gap = row["min_inter_drone_distance_m"]
    gap_s = "—" if gap is None else f"{float(gap):.3f}"
    dist = row["min_obstacle_distance_m"]
    dist_s = "—" if dist is None else f"{float(dist):.3f}"
    print(
        f"{key} {arm} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"tree={row['tree_collisions']} inter={row['inter_drone_collisions']} "
        f"gap={gap_s} obst={dist_s} sim={row['sim_duration_s']} wall={row['wall_time_s']:.1f}s",
        flush=True,
    )
    return row


def _worker(spec: tuple) -> dict:
    tag, kind, arm, seed, resume = spec
    return run_one(tag, kind, arm, int(seed), resume=bool(resume))


def _f(value, nd: int) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{nd}f}"


def summarize(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | 到达 | 撞树 | 机间碰撞 | 结果 | 最小障碍距离 m | 最小机间 m | 仿真 s | 墙钟 s |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    order = [scene_key(tag, kind) for tag, *_ in DENSITIES for kind in KINDS]
    for key in order:
        for arm in ARMS:
            group = [r for r in rows if r["scene_kind"] == key and r["arm"] == arm]
            if not group:
                continue
            r = group[0]
            lines.append(
                f"| {key} | {arm} | {r['arrived']}/{r['n_drones']} | {r['tree_collisions']} | "
                f"{r['inter_drone_collisions']} | {r['outcome']} | {_f(r['min_obstacle_distance_m'], 3)} | "
                f"{_f(r['min_inter_drone_distance_m'], 3)} | {_f(r['sim_duration_s'], 2)} | "
                f"{_f(r['wall_time_s'], 1)} |"
            )
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write-cards", action="store_true", help="按净空规则写出六张飞行卡")
    ap.add_argument("--check-cards", action="store_true", help="核对已提交的飞行卡与净空规则")
    ap.add_argument("--run", action="store_true", help="跑 L0 seed 0 的两臂")
    ap.add_argument("--densities", default="014,028,042")
    ap.add_argument("--kinds", default="swap3,solo")
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default="0")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--out", default="docs/research/e-c4-density-l0-runs.json")
    ap.add_argument("--no-resume", action="store_true")
    a = ap.parse_args(argv)
    if a.write_cards:
        paths = write_cards()
        for path in paths:
            print(f"写入 {path.relative_to(ROOT)}")
        if not a.run and not a.check_cards:
            return 0
    if a.check_cards or a.run or not a.write_cards:
        import hashlib

        for name, digest in GEOM_SHA256.items():
            got = hashlib.sha256((ROOT / "scenes" / name).read_bytes()).hexdigest()
            if got != digest:
                raise SystemExit(f"{name} sha256 变了：{got}")
        expected = expected_cards()
        for name, card in expected.items():
            path = ROOT / "scenes" / name
            if not path.is_file():
                raise SystemExit(f"缺少 {name}")
            raw = json.loads(path.read_text(encoding="utf-8"))
            if raw != card:
                raise SystemExit(f"{name} 与净空规则不一致")
            if DISCLAIMER not in raw["note"]:
                raise SystemExit(f"{name} 缺少声明")
        print("飞行卡与三份几何文件核对通过")
        if not a.run:
            return 0
    tags = [s for s in a.densities.split(",") if s]
    kinds = [s for s in a.kinds.split(",") if s]
    arms = [s for s in a.arms.split(",") if s]
    seeds = [int(s) for s in a.seeds.split(",") if s != ""]
    known_tags = {t[0] for t in DENSITIES}
    unknown = [s for s in tags if s not in known_tags] + [s for s in kinds if s not in KINDS] + [s for s in arms if s not in ARMS]
    if unknown:
        raise SystemExit(f"未知密度、卡种或臂：{unknown}")
    if a.jobs > 1 and "combo" in arms:
        raise SystemExit("combo 臂的后段预算按墙钟截断，必须 --jobs 1")
    resume = not a.no_resume
    from yopo_swarm_core.baseline import fetch_upstream

    fetch_upstream()
    # 先跑全关，再跑会吃墙钟预算的 combo。
    specs = []
    for arm in arms:
        for tag in tags:
            for kind in kinds:
                for seed in seeds:
                    specs.append((tag, kind, arm, seed, resume))
    if a.jobs <= 1:
        rows = [_worker(spec) for spec in specs]
    else:
        import multiprocessing as mp

        with mp.get_context("spawn").Pool(a.jobs) as pool:
            rows = pool.map(_worker, specs, chunksize=1)
    rows.sort(key=lambda r: (r["density_tag"], r["kind"], ARMS.index(r["arm"]), int(r["seed"])))
    payload = {
        "rows": rows,
        "shadow": True,
        "jobs": int(a.jobs),
        "note": (
            "只报数，不改默认，不冻结。墙钟不参与轨迹。"
            "backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。"
            f"{DISCLAIMER}（障碍半径 {OBSTACLE_RADIUS_M:.2f} m，边长 {MAP_SIDE_M:.1f} m）。"
        ),
    }
    dest = Path(a.out)
    if not dest.is_absolute():
        dest = ROOT / dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(summarize(rows))
    print(f"写入 {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
