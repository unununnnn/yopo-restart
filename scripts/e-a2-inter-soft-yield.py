#!/usr/bin/env python3
"""票 E-A2：机间硬过滤的软让行。只报数。

E-C6 在 014 六机对向里看到：E-A1 惩罚取 0（或再叠加 track_clearance=geom）时，
4 s 视界内所有准入候选都进了 1.0 m，可行集被整票清空，`CORRIDOR_EMPTY_HOVER`
堆到上千次，平均速度腰斩，对向超时。

本脚本在进程里改参数，不改场景卡，对比四档。全部偏离上游，report-only：

- base：E-A1 硬过滤，阈值 1.0 m，惩罚 0，empty_brake
- soft_only：base + soft_yield，hard_min_m 0.75，让行权重 2.0
- geom_base：base + track_clearance mode=geom，margin 0.08 m
- geom_soft：geom_base + 同一组软让行

014 三机对向用 scenes/ego-density-014-swap3.json，seeds 0–9。
014 六机对向不写盘：几何仍是 scenes/ego-density-014.json，六架沿直径对向，
净空外推与 E-C4 的三机同一套函数。seeds 0–4。

`--partial` 逐粒落盘，`--resume` 跳过已经落盘的粒。不改默认开关，
不进 scenes/suite.txt，不设 gates。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-a2-inter-soft-yield" / "partial"
GEOM_NAME = "ego-density-014.json"
GEOM_SHA256 = "5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9"
SWAP3_FILE = "scenes/ego-density-014-swap3.json"
BODY_RADIUS_M = 0.35
MARGIN_M = 0.08
THRESHOLD_M = 1.0
HARD_MIN_M = 0.75
YIELD_PENALTY = 2.0
HOVER_CODE = "CORRIDOR_EMPTY_HOVER"

# 与三机 0° / 120° / 240° 同一组数，中间再插 60° / 180° / 300°。
_S3 = math.sqrt(3.0) / 2.0
SWAP6_UNIT = (
    (1.0, 0.0),
    (0.5, _S3),
    (-0.5, _S3),
    (-1.0, 0.0),
    (-0.5, -_S3),
    (0.5, -_S3),
)

SCENES = ("d014-swap3", "d014-swap6")
SCENE_SEEDS = {
    "d014-swap3": tuple(range(10)),
    "d014-swap6": tuple(range(5)),
}
SCENE_LABEL = {"d014-swap3": "014 三机对向", "d014-swap6": "014 六机对向"}
ARMS = ("base", "soft_only", "geom_base", "geom_soft")

DISCLAIMER = (
    "只报数。report-only。不改默认 inter_drone_soft_yield false、empty_brake false、"
    "track_clearance false。四档都打开 E-A1（阈值 1.0 m、惩罚 0）和 empty_brake；"
    "geom 两档再打开 track_clearance mode=geom、margin 0.08 m。全部偏离上游。"
    "soft_yield 默认关。hard_min_m 0.75 m 高于 0.70 m 机间碰撞球。"
    "这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。"
    "未冻结，不进 scenes/suite.txt，不设 gates。"
)

A1 = {
    "inter_drone_filter": True,
    "inter_drone_threshold_m": THRESHOLD_M,
    "inter_drone_penalty": 0,
}
SOFT = {
    "inter_drone_soft_yield": True,
    "inter_drone_hard_min_m": HARD_MIN_M,
    "inter_drone_yield_penalty": YIELD_PENALTY,
}
GEOM = {
    "track_clearance": True,
    "track_clearance_mode": "geom",
    "track_margin_m": MARGIN_M,
}


def load_ec4():
    name = "e_c4_density_l0_for_a2"
    if name in sys.modules:
        return sys.modules[name]
    path = ROOT / "scripts" / "e-c4-density-l0.py"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"无法加载 {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def arm_label(arm: str) -> str:
    labels = {
        "base": "E-A1 硬过滤 + empty_brake（偏离上游）",
        "soft_only": "E-A1 + 软让行 0.75 m + empty_brake（偏离上游）",
        "geom_base": "geom 0.08 m + E-A1 硬过滤 + empty_brake（偏离上游）",
        "geom_soft": "geom 0.08 m + 软让行 0.75 m + empty_brake（偏离上游）",
    }
    if arm not in labels:
        raise ValueError(arm)
    return labels[arm]


def _public_drone(d: dict) -> dict:
    ec4 = load_ec4()
    return {
        "id": d["id"],
        "start_enu": [ec4._coord(v) for v in d["start_enu"]],
        "goal_enu": [ec4._coord(v) for v in d["goal_enu"]],
    }


def place_swap6(centers: list[tuple[float, float]]) -> list[dict]:
    """六架直径对向。净空不够就沿射线推到正方形外，和 E-C4 三机同一套函数。"""
    ec4 = load_ec4()
    drones = []
    for i, (ux, uy) in enumerate(SWAP6_UNIT):
        ux, uy = ec4._unit(ux, uy)
        x8, y8 = ec4.NOMINAL_SWAP_RADIUS_M * ux, ec4.NOMINAL_SWAP_RADIUS_M * uy
        d_start = ec4.axis_distance(x8, y8, centers)
        d_goal = ec4.axis_distance(-x8, -y8, centers)
        keep = (
            d_start >= ec4.AXIS_CLEARANCE_M - 1e-12
            and d_goal >= ec4.AXIS_CLEARANCE_M - 1e-12
        )
        if keep:
            r = ec4.NOMINAL_SWAP_RADIUS_M
            sx, sy = x8, y8
        else:
            r, _r_min, _r_edge = ec4.first_clear_swap_radius(ux, uy, centers)
            sx, sy = r * ux, r * uy
        drones.append({
            "id": f"uav{i}",
            "start_enu": [sx, sy, ec4.HEIGHT_M],
            "goal_enu": [-sx, -sy, ec4.HEIGHT_M],
            "radius_m": r,
            "axis_distance_start": ec4.axis_distance(sx, sy, centers),
            "axis_distance_goal": ec4.axis_distance(-sx, -sy, centers),
        })
    return drones


def swap6_raw() -> dict:
    ec4 = load_ec4()
    centers = ec4.cylinder_centers(ec4.load_geometry(GEOM_NAME))
    placed = place_swap6(centers)
    return {
        "name": "ego-density-014-swap6",
        "note": (
            "票 E-A2 内存卡：014 六机 goto 对向，几何是 scenes/ego-density-014.json。"
            "六架在半径 8.0 m 的圆上每隔 60° 飞向直径对侧；0°/120°/240° 与三机卡同一方位。"
            "起终点到最近圆柱轴至少 0.70 m，不够就沿射线推到 20 m 正方形外，"
            "外推函数与 scripts/e-c4-density-l0.py 的三机相同。不写进 scenes/，"
            "不进 suite.txt。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。"
        ),
        "geometry": GEOM_NAME,
        "ground_z": 0.0,
        "drones": [_public_drone(d) for d in placed],
        "task": {"type": "goto", "duration_s": 40.0},
        "criteria": dict(ec4.CRITERIA),
        "camera": dict(ec4.CAMERA),
        "dynamics": dict(ec4.DYNAMICS),
        "rates_hz": dict(ec4.RATES),
        "seeds": [0],
    }


def _upstream_for(arm: str) -> dict:
    upstream = dict(A1)
    upstream["empty_brake"] = True
    if arm in ("soft_only", "geom_soft"):
        upstream.update(SOFT)
    if arm in ("geom_base", "geom_soft"):
        upstream.update(GEOM)
    if arm not in ARMS:
        raise ValueError(arm)
    return upstream


def _materialize(raw: dict):
    from yopo_l0.scene import SceneCard, parse_geometry

    card = json.loads(json.dumps(raw))
    geom = card.get("geometry")
    if isinstance(geom, str):
        card["geometry"] = json.loads((ROOT / "scenes" / geom).read_text(encoding="utf-8"))
    return SceneCard(card["name"], card, parse_geometry(card["geometry"], card.get("ground_z")))


def build_card(scene: str, arm: str):
    if scene == "d014-swap3":
        raw = json.loads((ROOT / SWAP3_FILE).read_text(encoding="utf-8"))
    elif scene == "d014-swap6":
        raw = swap6_raw()
    else:
        raise ValueError(scene)
    params = dict(raw.get("policy_params") or {})
    params["upstream"] = _upstream_for(arm)
    raw["policy_params"] = params
    return _materialize(raw)


def _mean_speed(ticks: list[dict]) -> float:
    if not ticks:
        return 0.0
    return float(np.mean([np.linalg.norm(row["v"]) for row in ticks]))


def _hover_count(ticks: list[dict], physics: int, planner: int) -> int:
    """每个规划拍、每一架记一次。物理拍重复的同一份规划码不累加。"""
    n = 0
    for row in ticks:
        k = int(row["k"])
        if k != 0 and (k * planner) // physics == ((k - 1) * planner) // physics:
            continue
        if row.get("plan_code") == HOVER_CODE:
            n += 1
    return n


def _hit_kind(card, result: dict, ticks: list[dict]) -> str:
    outcome = result.get("outcome")
    reason = str(result.get("reason") or "")
    if outcome == "TIMEOUT":
        return "timeout"
    if outcome == "COLLISION" and "INTER_DRONE" in reason:
        return "inter"
    if outcome != "COLLISION" or "TRUTH_POSITION" not in reason:
        return "none"
    from yopo_l0.scene import Cylinder, Ground

    drone = reason.split(":", 1)[0]
    rows = [row for row in ticks if row["uav"] == drone]
    if not rows:
        return "pillar"
    p = np.asarray(rows[-1]["p"], dtype=float)
    ground = math.inf
    pillar = math.inf
    for ob in card.obstacles:
        dist = float(ob.distance(p))
        if isinstance(ob, Ground):
            ground = min(ground, dist)
        elif isinstance(ob, Cylinder):
            pillar = min(pillar, dist)
    return "ground" if ground <= pillar else "pillar"


def _partial_path(partial_dir: Path, scene: str, arm: str, seed: int, duration_s: float | None) -> Path:
    suffix = "" if duration_s is None else f"-t{duration_s:g}"
    return partial_dir / f"{scene}-{arm}-{int(seed)}{suffix}.json"


def _duration_token(duration_s: float | None):
    return None if duration_s is None else float(duration_s)


def run_one(
    scene: str,
    arm: str,
    seed: int,
    *,
    partial: bool,
    resume: bool,
    duration_s: float | None,
    partial_dir: Path,
) -> dict:
    dest = _partial_path(partial_dir, scene, arm, seed, duration_s)
    token = _duration_token(duration_s)
    if resume and dest.is_file():
        cached = json.loads(dest.read_text(encoding="utf-8"))
        if cached.get("duration_s_override") == token:
            return cached
    from yopo_l0.runner import RunConfig, run

    card = build_card(scene, arm)
    rates = card.card["rates_hz"]
    t0 = time.perf_counter()
    out = run(
        card,
        RunConfig(seed=int(seed), shadow=False, duration_s=duration_s),
    )
    res = out.result
    kind = _hit_kind(card, res, out.ticks)
    filt = res.get("inter_drone_filter") or {}
    wall = res.get("plan_wall_ms") or {}
    p95 = wall.get("p95")
    row = {
        "scene": scene,
        "arm": arm,
        "seed": int(seed),
        "report_only": True,
        "frozen": False,
        "upstream_deviation": True,
        "duration_s_override": token,
        "scene_card_hash": res["scene_card_hash"],
        "outcome": res["outcome"],
        "reason": res["reason"],
        "arrived": int(res["arrived"]),
        "n_drones": int(res["n_drones"]),
        "collisions": int(kind in ("pillar", "ground", "inter")),
        "pillar": int(kind == "pillar"),
        "ground": int(kind == "ground"),
        "inter": int(kind == "inter"),
        "timeout": int(kind == "timeout"),
        "mean_speed_mps": _mean_speed(out.ticks),
        "speed_samples": len(out.ticks),
        "corridor_empty_hover": _hover_count(out.ticks, int(rates["physics"]), int(rates["planner"])),
        "min_inter_drone_distance_m": res["min_inter_drone_distance_m"],
        "plan_wall_ms_p95": None if p95 is None else float(p95),
        "empty_fallbacks": int(filt.get("empty_fallbacks") or 0),
        "penalized_candidates": int(filt.get("penalized_candidates") or 0),
        "filtered_candidates": int(filt.get("filtered_candidates") or 0),
        "sim_duration_s": float(res["sim_duration_s"]),
        "min_obstacle_distance_m": res["min_obstacle_distance_m"],
        "safe_radius_m": float(res["safe_radius_m"]),
        "empty_brake": res.get("empty_brake"),
        "track_clearance_mode": res.get("track_clearance_mode"),
        "wall_time_s": float(res["wall_time_s"]),
        "worker_s": time.perf_counter() - t0,
    }
    if bool(res["upstream_deviation"]) is not True:
        raise RuntimeError(f"{scene} {arm} 没有标偏离上游")
    if res.get("empty_brake") is not True:
        raise RuntimeError(f"{scene} {arm} 没有打开 empty_brake")
    if abs(float(res["safe_radius_m"]) - 0.05) > 1e-9:
        raise RuntimeError(f"{scene} {arm} safe_radius 读回 {res['safe_radius_m']}")
    if partial or resume:
        dest.parent.mkdir(parents=True, exist_ok=True)
        tmp = dest.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(row, ensure_ascii=False), encoding="utf-8")
        tmp.replace(dest)
    gap = row["min_inter_drone_distance_m"]
    gap_s = "—" if gap is None else f"{float(gap):.3f}"
    print(
        f"{scene} {arm} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"hit={kind} hover={row['corridor_empty_hover']} v={row['mean_speed_mps']:.2f} "
        f"gap={gap_s} p95={row['plan_wall_ms_p95']} sim={row['sim_duration_s']:.2f} "
        f"wall={row['wall_time_s']:.1f}s",
        flush=True,
    )
    return row


def _worker(spec: tuple) -> dict:
    try:
        os.nice(10)
    except OSError:
        pass
    scene, arm, seed, partial, resume, duration_s, partial_dir = spec
    return run_one(
        str(scene),
        str(arm),
        int(seed),
        partial=bool(partial),
        resume=bool(resume),
        duration_s=None if duration_s is None else float(duration_s),
        partial_dir=Path(partial_dir),
    )


def seeds_for(scene: str, override: list[int] | None) -> tuple[int, ...]:
    if override is not None:
        return tuple(int(s) for s in override)
    return SCENE_SEEDS[scene]


def job_specs(scenes, arms, seeds_by_scene, *, partial: bool, resume: bool, duration_s, partial_dir: Path):
    return [
        (scene, arm, int(seed), bool(partial), bool(resume), duration_s, str(partial_dir))
        for scene in scenes
        for arm in arms
        for seed in seeds_by_scene[scene]
    ]


def _run_group(specs: list[tuple], jobs: int) -> list[dict]:
    if jobs <= 1 or len(specs) <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)


def _group(rows: list[dict], scene: str, arm: str) -> list[dict]:
    return [r for r in rows if r["scene"] == scene and r["arm"] == arm]


def _weighted_speed(group: list[dict]) -> float | None:
    samples = sum(int(r["speed_samples"]) for r in group)
    if samples <= 0:
        return None
    total = sum(float(r["mean_speed_mps"]) * int(r["speed_samples"]) for r in group)
    return total / samples


def _rng(values: list[float], digits: int) -> str:
    if not values:
        return "—"
    lo, hi = min(values), max(values)
    if abs(lo - hi) < 5 * 10 ** (-(digits + 1)):
        return f"{lo:.{digits}f}"
    return f"{lo:.{digits}f}–{hi:.{digits}f}"


def render_summary(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 臂 | 偏离上游 | 到达 | 总碰撞 | 撞柱 | 撞地 | 机间 | 超时 | 平均速度 m/s | CORRIDOR_EMPTY_HOVER | 最小机间 m | plan_wall_ms p95 |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for scene in SCENES:
        for arm in ARMS:
            group = _group(rows, scene, arm)
            if not group:
                continue
            drones = sum(int(r["n_drones"]) for r in group)
            arrived = sum(int(r["arrived"]) for r in group)
            collisions = sum(int(r["collisions"]) for r in group)
            pillar = sum(int(r["pillar"]) for r in group)
            ground = sum(int(r["ground"]) for r in group)
            inter = sum(int(r["inter"]) for r in group)
            timeout = sum(int(r["timeout"]) for r in group)
            hover = sum(int(r["corridor_empty_hover"]) for r in group)
            speed = _weighted_speed(group)
            speed_s = "—" if speed is None else f"{speed:.2f}"
            gaps = [
                float(r["min_inter_drone_distance_m"])
                for r in group
                if r.get("min_inter_drone_distance_m") is not None
            ]
            p95s = [float(r["plan_wall_ms_p95"]) for r in group if r.get("plan_wall_ms_p95") is not None]
            lines.append(
                f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | 偏离上游 | {arrived}/{drones} | "
                f"{collisions} | {pillar} | {ground} | {inter} | {timeout} | {speed_s} | {hover} | "
                f"{_rng(gaps, 3)} | {_rng(p95s, 1)} |"
            )
    return "\n".join(lines)


def render_seeds(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 臂 | seed | 结果 | 到达 | 撞柱 | 撞地 | 机间 | 超时 | 平均速度 m/s | CORRIDOR_EMPTY_HOVER | 最小机间 m | plan_wall_ms p95 |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for scene in SCENES:
        for arm in ARMS:
            for row in sorted(_group(rows, scene, arm), key=lambda r: int(r["seed"])):
                gap = row["min_inter_drone_distance_m"]
                gap_s = "—" if gap is None else f"{float(gap):.3f}"
                p95 = row["plan_wall_ms_p95"]
                p95_s = "—" if p95 is None else f"{float(p95):.1f}"
                lines.append(
                    f"| {SCENE_LABEL[scene]} | {arm_label(row['arm'])} | {row['seed']} | "
                    f"{row['outcome']} | {row['arrived']}/{row['n_drones']} | {row['pillar']} | "
                    f"{row['ground']} | {row['inter']} | {row['timeout']} | "
                    f"{float(row['mean_speed_mps']):.2f} | {row['corridor_empty_hover']} | "
                    f"{gap_s} | {p95_s} |"
                )
    return "\n".join(lines)


def _check() -> int:
    import hashlib

    from yopo_l0.inter_drone import InterDroneConfig, inter_drone_config
    from yopo_l0.policy import read_empty_brake, read_track_clearance

    blob = (ROOT / "scenes" / GEOM_NAME).read_bytes()
    if hashlib.sha256(blob).hexdigest() != GEOM_SHA256:
        raise SystemExit(f"{GEOM_NAME} sha256 变了")
    if read_empty_brake({}) or read_track_clearance({}):
        raise SystemExit("empty_brake 或 track_clearance 默认被打开了")
    off = inter_drone_config({
        "upstream": {
            "inter_drone_filter": True,
            "inter_drone_threshold_m": 1.0,
            "inter_drone_penalty": 0,
        }
    })
    if off != InterDroneConfig(1.0, 0.0):
        raise SystemExit("软让行缺省时配置与 E-A1 不一致")
    watched = {
        SWAP3_FILE: (ROOT / SWAP3_FILE).read_bytes(),
        f"scenes/{GEOM_NAME}": blob,
        "scenes/suite.txt": (ROOT / "scenes" / "suite.txt").read_bytes(),
    }
    ec4 = load_ec4()
    centers = ec4.cylinder_centers(ec4.load_geometry(GEOM_NAME))
    placed = place_swap6(centers)
    if len(placed) != 6:
        raise SystemExit("六机对向不是 6 架")
    disk = json.loads(watched[SWAP3_FILE])
    for src, idx in ((0, 0), (1, 2), (2, 4)):
        for key in ("start_enu", "goal_enu"):
            got = _public_drone(placed[idx])[key]
            exp = disk["drones"][src][key]
            if any(abs(float(a) - float(b)) > 1e-9 for a, b in zip(got, exp)):
                raise SystemExit(f"六机 {idx} 的 {key} 与三机 uav{src} 不一致：{got} vs {exp}")
    for d in placed:
        if d["axis_distance_start"] < ec4.AXIS_CLEARANCE_M - 1e-9:
            raise SystemExit(f"{d['id']} 起点净空不足")
        if d["axis_distance_goal"] < ec4.AXIS_CLEARANCE_M - 1e-9:
            raise SystemExit(f"{d['id']} 终点净空不足")
    for scene in SCENES:
        for arm in ARMS:
            card = build_card(scene, arm)
            up = card.card["policy_params"]["upstream"]
            if up.get("empty_brake") is not True or up.get("inter_drone_penalty") != 0:
                raise SystemExit(f"{scene} {arm} 没有硬过滤和 empty_brake")
            if abs(float(up["inter_drone_threshold_m"]) - THRESHOLD_M) > 1e-12:
                raise SystemExit(f"{scene} {arm} 阈值不是 1.0 m")
            if "safe_radius" in up:
                raise SystemExit(f"{scene} {arm} 把 safe_radius 写进了进程内卡")
            soft = arm in ("soft_only", "geom_soft")
            if bool(up.get("inter_drone_soft_yield")) is not soft:
                raise SystemExit(f"{scene} {arm} 软让行开关不对")
            if soft and abs(float(up["inter_drone_hard_min_m"]) - HARD_MIN_M) > 1e-12:
                raise SystemExit(f"{scene} {arm} hard_min 不是 0.75 m")
            geom = arm in ("geom_base", "geom_soft")
            if geom:
                if up.get("track_clearance") is not True or up.get("track_clearance_mode") != "geom":
                    raise SystemExit(f"{scene} {arm} 不是 geom")
                if abs(float(up["track_margin_m"]) - MARGIN_M) > 1e-12:
                    raise SystemExit(f"{scene} {arm} margin 不是 0.08 m")
            elif "track_clearance" in up:
                raise SystemExit(f"{scene} {arm} 多写了 track_clearance")
    after = {rel: (ROOT / rel).read_bytes() for rel in watched}
    if after != watched:
        raise SystemExit("检查时改写了场景文件")
    suite = watched["scenes/suite.txt"].decode()
    if "e-a2" in suite or "soft_yield" in suite:
        raise SystemExit("scenes/suite.txt 被写进了本票")
    print("e-a2 check ok")
    return 0


def _head_commit() -> str:
    import subprocess

    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def _parse_seeds(text: str | None) -> list[int] | None:
    if text is None:
        return None
    return [int(s) for s in text.split(",") if s != ""]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scenes", default=",".join(SCENES))
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=None, help="覆盖两张卡的种子。缺省时三机 0–9，六机 0–4")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--out", default="docs/research/e-a2-inter-soft-yield-runs.json")
    ap.add_argument("--partial", action="store_true", help="每粒跑完就落盘")
    ap.add_argument("--resume", action="store_true", help="已落盘的粒直接读回")
    ap.add_argument("--partial-dir", default=None)
    ap.add_argument("--duration-s", type=float, default=None, help="盖过场景卡时长，冒烟用")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)
    if args.check:
        return _check()
    if args.jobs not in (1, 2, 3, 4):
        raise SystemExit("jobs 只能是 1 到 4")
    if args.duration_s is not None and (not math.isfinite(args.duration_s) or args.duration_s <= 0.0):
        raise SystemExit("--duration-s 必须为正")
    scenes = [s for s in args.scenes.split(",") if s]
    arms = [s for s in args.arms.split(",") if s]
    unknown = [s for s in scenes if s not in SCENES] + [s for s in arms if s not in ARMS]
    if unknown:
        raise SystemExit(f"未知场景或臂：{unknown}")
    override = _parse_seeds(args.seeds)
    seeds_by_scene = {scene: seeds_for(scene, override) for scene in scenes}
    partial_dir = Path(args.partial_dir) if args.partial_dir else PARTIAL
    if not partial_dir.is_absolute():
        partial_dir = ROOT / partial_dir
    _check()
    specs = job_specs(
        scenes,
        arms,
        seeds_by_scene,
        partial=bool(args.partial),
        resume=bool(args.resume),
        duration_s=args.duration_s,
        partial_dir=partial_dir,
    )
    print(
        f"e-a2 start n={len(specs)} jobs={args.jobs} partial={bool(args.partial)} "
        f"resume={bool(args.resume)}",
        flush=True,
    )
    rows = _run_group(specs, int(args.jobs))
    rows.sort(key=lambda r: (
        SCENES.index(r["scene"]),
        ARMS.index(r["arm"]),
        int(r["seed"]),
    ))
    payload = {
        "disclaimer": DISCLAIMER,
        "commit": _head_commit(),
        "report_only": True,
        "frozen": False,
        "upstream_deviation": True,
        "scenes": scenes,
        "arms": arms,
        "seeds_by_scene": {scene: list(seeds_by_scene[scene]) for scene in scenes},
        "jobs": int(args.jobs),
        "threshold_m": THRESHOLD_M,
        "hard_min_m": HARD_MIN_M,
        "yield_penalty": YIELD_PENALTY,
        "margin_m": MARGIN_M,
        "body_radius_m": BODY_RADIUS_M,
        "shadow": False,
        "rows": rows,
        "note": DISCLAIMER,
    }
    dest = Path(args.out)
    if not dest.is_absolute():
        dest = ROOT / dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(render_summary(rows))
    print(f"写入 {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
