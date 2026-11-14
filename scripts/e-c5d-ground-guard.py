#!/usr/bin/env python3
"""票 E-C5d：地面净空闸门。只报数。

四档都在进程里改参数，不改场景卡。先 014-swap3 seeds 0–9，再 028-swap3，再 028-solo。

- geom（关）：E-C5c 的 base + track geom（E-A1 1.0 m、empty_brake 开），ground_guard 关
- geom + ground_guard：同一几何门和 E-A1，empty_brake 关，ground_guard 开
- geom + empty_brake + ground_guard：几何门、E-A1、empty_brake、ground_guard 都开
- geom_multi + empty_brake + ground_guard：模式换成 geom_multi，其余与上一档相同

全部偏离上游，report-only。不改默认、不进 scenes/suite.txt、不设 gates。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-c5d-ground-guard" / "partial"

SCENES = {
    "d014-swap3": "scenes/ego-density-014-swap3.json",
    "d028-swap3": "scenes/ego-density-028-swap3.json",
    "d028-solo": "scenes/ego-density-028-solo.json",
}
SCENE_ORDER = ("d014-swap3", "d028-swap3", "d028-solo")
SCENE_LABEL = {
    "d014-swap3": "014 对向",
    "d028-swap3": "028 对向",
    "d028-solo": "028 单机",
}
ARMS = ("geom", "geom_guard", "geom_brake_guard", "multi_brake_guard")
SEEDS = tuple(range(10))
BODY_RADIUS_M = 0.35
MARGIN_M = 0.08
CLEARANCE_M = 0.43

DISCLAIMER = (
    "只报数（report-only）。不改默认 safe_radius 0.05 m、empty_brake false、"
    "track_clearance false、ground_guard false。"
    "四档都打开了 track_clearance，偏离上游。"
    "ground_guard 打开也偏离上游。未冻结，不进 scenes/suite.txt，不设 gates。"
    "这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。"
)

A1 = {
    "inter_drone_filter": True,
    "inter_drone_threshold_m": 1.0,
    "inter_drone_penalty": 0,
}


def arm_upstream(arm: str) -> dict:
    if arm not in ARMS:
        raise ValueError(arm)
    upstream = dict(A1)
    upstream["track_clearance"] = True
    upstream["track_margin_m"] = MARGIN_M
    if arm == "geom":
        upstream["track_clearance_mode"] = "geom"
        upstream["empty_brake"] = True
        upstream["ground_guard"] = False
    elif arm == "geom_guard":
        upstream["track_clearance_mode"] = "geom"
        upstream["empty_brake"] = False
        upstream["ground_guard"] = True
        upstream["ground_guard_clearance_m"] = CLEARANCE_M
    elif arm == "geom_brake_guard":
        upstream["track_clearance_mode"] = "geom"
        upstream["empty_brake"] = True
        upstream["ground_guard"] = True
        upstream["ground_guard_clearance_m"] = CLEARANCE_M
    else:
        upstream["track_clearance_mode"] = "geom_multi"
        upstream["empty_brake"] = True
        upstream["ground_guard"] = True
        upstream["ground_guard_clearance_m"] = CLEARANCE_M
    return upstream


def arm_label(arm: str) -> str:
    return {
        "geom": "geom（关，E-C5c 的 geom：E-A1 + empty_brake，ground_guard 关）",
        "geom_guard": "geom + ground_guard（empty_brake 关）",
        "geom_brake_guard": "geom + empty_brake + ground_guard",
        "multi_brake_guard": "geom_multi + empty_brake + ground_guard",
    }[arm]


def build_card(scene: str, arm: str):
    from yopo_l0.scene import SceneCard, parse_geometry

    if scene not in SCENES:
        raise ValueError(scene)
    path = ROOT / SCENES[scene]
    raw = json.loads(path.read_text(encoding="utf-8"))
    params = dict(raw.get("policy_params") or {})
    upstream = dict(params.get("upstream") or {})
    upstream.update(arm_upstream(arm))
    if "safe_radius" in upstream:
        raise RuntimeError("对照不写 safe_radius，0.05 m 走运行器缺省")
    params["upstream"] = upstream
    raw["policy_params"] = params
    card = json.loads(json.dumps(raw))
    geom = card.get("geometry")
    if isinstance(geom, str):
        card["geometry"] = json.loads((ROOT / "scenes" / geom).read_text(encoding="utf-8"))
    return SceneCard(card["name"], card, parse_geometry(card["geometry"], card.get("ground_z")))


def _partial_path(scene: str, arm: str, seed: int) -> Path:
    return PARTIAL / f"{scene}-{arm}-{int(seed)}.json"


def _clearances(card, ticks: list[dict]) -> dict:
    from yopo_l0.scene import Ground

    grounds = [ob for ob in card.obstacles if isinstance(ob, Ground)]
    others = [ob for ob in card.obstacles if not isinstance(ob, Ground)]
    min_g = math.inf
    min_o = math.inf
    per: dict[str, float] = {}
    pillar = 0
    ground = 0
    for row in ticks:
        p = np.asarray(row["p"], dtype=float)
        drone = str(row["uav"])
        if grounds:
            g = min(max(float(p[2]) - float(ob.z), 0.0) for ob in grounds)
        else:
            g = math.inf
        o = min((float(ob.distance(p)) for ob in others), default=math.inf)
        min_g = min(min_g, g)
        min_o = min(min_o, o)
        per[drone] = min(per.get(drone, math.inf), g)
        if g <= BODY_RADIUS_M + 1e-9:
            ground = 1
        if o <= BODY_RADIUS_M + 1e-9:
            pillar = 1
    return {
        "min_ground_clearance_m": None if not math.isfinite(min_g) else float(min_g),
        "min_cylinder_clearance_m": None if not math.isfinite(min_o) else float(min_o),
        "drone_ground_m": {k: float(v) for k, v in sorted(per.items())},
        "pillar_collision": int(pillar),
        "ground_collision": int(ground),
    }


def run_one(scene: str, arm: str, seed: int, *, resume: bool) -> dict:
    dest = _partial_path(scene, arm, seed)
    if resume and dest.is_file():
        return json.loads(dest.read_text(encoding="utf-8"))
    from yopo_l0.runner import RunConfig, run

    card = build_card(scene, arm)
    spec = arm_upstream(arm)
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(seed), shadow=True))
    res = out.result
    gaps = _clearances(card, out.ticks)
    reason = str(res["reason"])
    collided = res["outcome"] == "COLLISION"
    # 没撞上时，距离进了机体半径才算。运行器会在第一次真值碰撞停下。
    if not collided:
        gaps["pillar_collision"] = 0
        gaps["ground_collision"] = 0
    uav1 = gaps["drone_ground_m"].get("uav1")
    row = {
        "scene": scene,
        "arm": arm,
        "seed": int(seed),
        "report_only": True,
        "frozen": False,
        "upstream_deviation": True,
        "deviates_from_upstream": True,
        "ground_guard": bool(spec["ground_guard"]),
        "empty_brake": bool(spec["empty_brake"]),
        "track_clearance_mode": spec["track_clearance_mode"],
        "ground_guard_clearance_m": CLEARANCE_M if spec["ground_guard"] else None,
        "outcome": res["outcome"],
        "reason": reason,
        "arrived": int(res["arrived"]),
        "n_drones": int(res["n_drones"]),
        "collision": int(collided),
        "pillar_collision": int(gaps["pillar_collision"]) if collided else 0,
        "ground_collision": int(gaps["ground_collision"]) if collided else 0,
        "inter_drone_collision": int(collided and "INTER_DRONE" in reason),
        "timeout": int(res["outcome"] == "TIMEOUT"),
        "min_ground_clearance_m": gaps["min_ground_clearance_m"],
        "min_cylinder_clearance_m": gaps["min_cylinder_clearance_m"],
        "min_inter_drone_distance_m": res["min_inter_drone_distance_m"],
        "min_obstacle_distance_m": res["min_obstacle_distance_m"],
        "sim_duration_s": res["sim_duration_s"],
        "drone_ground_m": gaps["drone_ground_m"],
        "uav1_ground_m": uav1,
        "uav1_ground_hit": None if uav1 is None else int(uav1 <= BODY_RADIUS_M + 1e-9),
        "scene_card_hash": res["scene_card_hash"],
        "safe_radius_m": res.get("safe_radius_m"),
        "runner_upstream_deviation": bool(res.get("upstream_deviation")),
        "wall_time_s": res["wall_time_s"],
        "worker_s": time.perf_counter() - t0,
    }
    if abs(float(row["safe_radius_m"]) - 0.05) > 1e-9:
        raise RuntimeError(f"safe_radius 读回 {row['safe_radius_m']}")
    if row["runner_upstream_deviation"] is not True:
        raise RuntimeError("track_clearance 开着却没有标偏离上游")
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)
    gap = row["min_inter_drone_distance_m"]
    gap_s = "—" if gap is None else f"{float(gap):.3f}"
    gz = row["min_ground_clearance_m"]
    gz_s = "—" if gz is None else f"{float(gz):.3f}"
    print(
        f"{scene} {arm} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"pillar={row['pillar_collision']} ground={row['ground_collision']} "
        f"inter={row['inter_drone_collision']} timeout={row['timeout']} "
        f"gz={gz_s} gap={gap_s} uav1={row['uav1_ground_m']} "
        f"sim={float(row['sim_duration_s']):.2f} wall={float(row['wall_time_s']):.1f}s",
        flush=True,
    )
    return row


def _worker(spec: tuple) -> dict:
    try:
        os.nice(10)
    except OSError:
        pass
    scene, arm, seed, resume = spec
    return run_one(str(scene), str(arm), int(seed), resume=bool(resume))


def job_specs(scenes, arms, seeds, *, resume: bool) -> list[tuple]:
    return [
        (scene, arm, int(seed), bool(resume))
        for scene in scenes
        for arm in arms
        for seed in seeds
    ]


def _run_group(specs: list[tuple], jobs: int) -> list[dict]:
    if jobs <= 1 or len(specs) <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)


def _rng(values: list[float]) -> str:
    if not values:
        return "—"
    lo, hi = min(values), max(values)
    if abs(lo - hi) < 5e-4:
        return f"{lo:.3f}"
    return f"{lo:.3f}–{hi:.3f}"


def _group(rows, scene, arm):
    return [r for r in rows if r["scene"] == scene and r["arm"] == arm]


def render_summary(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | 偏离上游 | 到达 | 总碰撞 | 撞柱 | 撞地 | 机间碰撞 | 超时 | 最小地面净空 m | 最小机间 m |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for scene in SCENE_ORDER:
        for arm in ARMS:
            group = _group(rows, scene, arm)
            if not group:
                continue
            drones = sum(int(r["n_drones"]) for r in group)
            arrived = sum(int(r["arrived"]) for r in group)
            collision = sum(int(r["collision"]) for r in group)
            pillar = sum(int(r["pillar_collision"]) for r in group)
            ground = sum(int(r["ground_collision"]) for r in group)
            inter = sum(int(r["inter_drone_collision"]) for r in group)
            timeout = sum(int(r["timeout"]) for r in group)
            grounds = [float(r["min_ground_clearance_m"]) for r in group if r.get("min_ground_clearance_m") is not None]
            gaps = [
                float(r["min_inter_drone_distance_m"])
                for r in group
                if r.get("min_inter_drone_distance_m") is not None
            ]
            lines.append(
                f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | 偏离上游 | {arrived}/{drones} | "
                f"{collision} | {pillar} | {ground} | {inter} | {timeout} | {_rng(grounds)} | {_rng(gaps)} |"
            )
    return "\n".join(lines)


def render_seeds(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | seed | 结果 | 原因 | 到达 | 撞柱 | 撞地 | 机间 | 超时 | 最小地面净空 m | 最小机间 m | uav1 地面 m |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for scene in SCENE_ORDER:
        for arm in ARMS:
            for row in sorted(_group(rows, scene, arm), key=lambda r: int(r["seed"])):
                gap = row["min_inter_drone_distance_m"]
                gap_s = "—" if gap is None else f"{float(gap):.3f}"
                gz = row["min_ground_clearance_m"]
                gz_s = "—" if gz is None else f"{float(gz):.3f}"
                u1 = row.get("uav1_ground_m")
                u1_s = "—" if u1 is None else f"{float(u1):.3f}"
                lines.append(
                    f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | {row['seed']} | {row['outcome']} | "
                    f"`{row['reason']}` | {row['arrived']}/{row['n_drones']} | "
                    f"{row['pillar_collision']} | {row['ground_collision']} | {row['inter_drone_collision']} | "
                    f"{row['timeout']} | {gz_s} | {gap_s} | {u1_s} |"
                )
    return "\n".join(lines)


def render_focus(rows: list[dict]) -> str:
    lines = [
        "| 臂 | seed | 结果 | 原因 | uav1 最小地面净空 m | uav1 撞地 |",
        "|---|---|---|---|---|---|",
    ]
    focus = [
        r for r in rows
        if r["scene"] == "d014-swap3" and int(r["seed"]) in (2, 9)
    ]
    focus.sort(key=lambda r: (ARMS.index(r["arm"]), int(r["seed"])))
    for row in focus:
        u1 = row.get("uav1_ground_m")
        u1_s = "—" if u1 is None else f"{float(u1):.3f}"
        hit = row.get("uav1_ground_hit")
        hit_s = "—" if hit is None else ("是" if int(hit) else "否")
        lines.append(
            f"| {arm_label(row['arm'])} | {row['seed']} | {row['outcome']} | `{row['reason']}` | {u1_s} | {hit_s} |"
        )
    return "\n".join(lines)


def _check() -> int:
    from yopo_l0.policy import read_empty_brake, read_ground_guard, read_track_clearance

    if read_ground_guard({}) or read_track_clearance({}) or read_empty_brake({}):
        raise SystemExit("默认开关被改了")
    before = {rel: (ROOT / rel).read_bytes() for rel in SCENES.values()}
    for scene in SCENE_ORDER:
        for arm in ARMS:
            card = build_card(scene, arm)
            up = card.card["policy_params"]["upstream"]
            spec = arm_upstream(arm)
            if up.get("ground_guard") is not spec["ground_guard"]:
                raise SystemExit(f"{scene} {arm} ground_guard 不一致")
            if up.get("empty_brake") is not spec["empty_brake"]:
                raise SystemExit(f"{scene} {arm} empty_brake 不一致")
            if up.get("track_clearance_mode") != spec["track_clearance_mode"]:
                raise SystemExit(f"{scene} {arm} 模式不一致")
            if "safe_radius" in up:
                raise SystemExit("把 safe_radius 写进了进程内场景卡")
    after = {rel: (ROOT / rel).read_bytes() for rel in before}
    if after != before:
        raise SystemExit("检查场景卡时改写了文件")
    suite = (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    if "e-c5d" in suite or "ground_guard" in suite:
        raise SystemExit("scenes/suite.txt 被写进了本票的卡")
    print("e-c5d check ok")
    return 0


def _head_commit() -> str:
    import subprocess

    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scenes", default=",".join(SCENE_ORDER))
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--out", default="docs/research/e-c5d-ground-guard-runs.json")
    ap.add_argument("--no-resume", action="store_true")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)
    if args.check:
        return _check()
    if args.jobs not in (1, 2, 3, 4):
        raise SystemExit("jobs 只能是 1 到 4")
    scenes = [s for s in args.scenes.split(",") if s]
    arms = [s for s in args.arms.split(",") if s]
    seeds = [int(s) for s in args.seeds.split(",") if s != ""]
    unknown = [s for s in scenes if s not in SCENES] + [s for s in arms if s not in ARMS]
    if unknown:
        raise SystemExit(f"未知场景或臂：{unknown}")
    _check()
    resume = not args.no_resume
    specs = job_specs(scenes, arms, seeds, resume=resume)
    print(f"e-c5d start n={len(specs)} jobs={args.jobs} resume={resume}", flush=True)
    rows = _run_group(specs, int(args.jobs))
    rows.sort(key=lambda r: (
        SCENE_ORDER.index(r["scene"]) if r["scene"] in SCENE_ORDER else 99,
        ARMS.index(r["arm"]) if r["arm"] in ARMS else 99,
        int(r["seed"]),
    ))
    payload = {
        "disclaimer": DISCLAIMER,
        "commit": _head_commit(),
        "report_only": True,
        "frozen": False,
        "deviates_from_upstream": True,
        "scenes": scenes,
        "arms": arms,
        "arm_labels": {arm: arm_label(arm) for arm in arms},
        "seeds": seeds,
        "jobs": int(args.jobs),
        "body_radius_m": BODY_RADIUS_M,
        "ground_guard_clearance_m": CLEARANCE_M,
        "rows": rows,
        "summary_md": render_summary(rows),
        "focus_md": render_focus(rows),
        "note": DISCLAIMER,
    }
    dest = Path(args.out)
    if not dest.is_absolute():
        dest = ROOT / dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(render_summary(rows), flush=True)
    print(render_focus(rows), flush=True)
    print(f"wrote {dest} rows={len(rows)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
