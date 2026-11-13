#!/usr/bin/env python3
"""票 E-C5b：走廊跟踪误差补偿。只报数。

在密度 0.14 的三机对向和单机上，seeds 0–9，五组都在进程里改参数，不改场景卡：

- off：默认全关（safe_radius 0.05 m，E-A1 关，empty_brake 关，track_clearance 关）
- base：0.05 m + E-A1 阈值 1.0 m、惩罚 0 + empty_brake
- pred：base + track_clearance，mode=pred，margin=0.08 m（解码器半径 0.13 m）
- geom：base + mode=geom，margin=0.08 m（表面净空 ≥ 0.35 + 0.08）
- geom_multi：base + mode=geom_multi。没有邻机（单机，以及多机第 0 个规划拍）不启几何门

撞树归因复用 scripts/e-c5-corridor-radius.py。empty_brake 开着且规划码是
CORRIDOR_EMPTY_HOVER 时，记成 E-C5 表里的「无解刹停」（那里叫 proto_braked）。
这是已合入的开关，不是 E-B4 的进程内原型。

0.08 m 是假设，用来盖 E-C5 记下的 0.058–0.073 m 跟踪差。不是论文复现
（障碍半径 0.20 m，边长 20.0 m）。不改 core/、已冻门、scenes/suite.txt、
docs/tickets.md、README，也不改任何默认值。结果不进冻结判据。
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
PARTIAL = ROOT / "out" / "e-c5b-track-clearance" / "partial"
GEOM_NAME = "ego-density-014.json"
GEOM_SHA256 = "5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9"

SCENES = {
    "d014-swap3": "scenes/ego-density-014-swap3.json",
    "d014-solo": "scenes/ego-density-014-solo.json",
}
SCENE_ORDER = ("d014-swap3", "d014-solo")
SCENE_LABEL = {"d014-swap3": "014 对向", "d014-solo": "014 单机"}
ARMS = ("off", "base", "pred", "geom", "geom_multi")
SEEDS = tuple(range(10))
BODY_RADIUS_M = 0.35
MARGIN_M = 0.08
UPSTREAM_SAFE_RADIUS_M = 0.05

DISCLAIMER = (
    "只报数。不改默认 safe_radius 0.05 m、empty_brake false、track_clearance false。"
    "E-A1、empty_brake、track_clearance 打开即偏离上游 d5e1e60。"
    "track_margin 0.08 m 是假设，用来盖已观测的 0.058–0.073 m 跟踪差。"
    "这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。"
    "未冻结，不进 scenes/suite.txt，不设 gates。"
)

A1 = {
    "inter_drone_filter": True,
    "inter_drone_threshold_m": 1.0,
    "inter_drone_penalty": 0,
}


def load_ec5():
    name = "e_c5_corridor_radius_for_c5b"
    if name in sys.modules:
        return sys.modules[name]
    path = ROOT / "scripts" / "e-c5-corridor-radius.py"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"无法加载 {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def arm_deviates(arm: str) -> bool:
    return arm != "off"


def arm_label(arm: str) -> str:
    labels = {
        "off": "默认全关",
        "base": "0.05 m + E-A1 1.0 m + empty_brake（偏离上游）",
        "pred": "base + track pred 0.08 m（偏离上游）",
        "geom": "base + track geom 0.08 m（偏离上游）",
        "geom_multi": "base + track geom_multi 0.08 m（偏离上游）",
    }
    if arm not in labels:
        raise ValueError(arm)
    return labels[arm]


def decoder_radius_m(arm: str) -> float:
    """pred 把解码器半径抬到 0.13 m。结果字段 safe_radius_m 仍是 0.05 m。"""
    if arm == "pred":
        return UPSTREAM_SAFE_RADIUS_M + MARGIN_M
    return UPSTREAM_SAFE_RADIUS_M


def build_card(scene: str, arm: str):
    from yopo_l0.scene import SceneCard, load_card, parse_geometry

    if scene not in SCENES:
        raise ValueError(scene)
    path = ROOT / SCENES[scene]
    if arm == "off":
        return load_card(path)
    if arm not in ARMS:
        raise ValueError(arm)
    raw = json.loads(path.read_text(encoding="utf-8"))
    params = dict(raw.get("policy_params") or {})
    upstream = dict(params.get("upstream") or {})
    upstream.update(A1)
    upstream["empty_brake"] = True
    if arm == "pred":
        upstream.update(track_clearance=True, track_clearance_mode="pred", track_margin_m=MARGIN_M)
    elif arm == "geom":
        upstream.update(track_clearance=True, track_clearance_mode="geom", track_margin_m=MARGIN_M)
    elif arm == "geom_multi":
        upstream.update(track_clearance=True, track_clearance_mode="geom_multi", track_margin_m=MARGIN_M)
    elif arm != "base":
        raise ValueError(arm)
    if "safe_radius" in upstream:
        raise RuntimeError("对照不写 safe_radius，0.05 m 走运行器缺省")
    params["upstream"] = upstream
    raw["policy_params"] = params
    card = json.loads(json.dumps(raw))
    geom = card.get("geometry")
    if isinstance(geom, str):
        card["geometry"] = json.loads((ROOT / "scenes" / geom).read_text(encoding="utf-8"))
    return SceneCard(card["name"], card, parse_geometry(card["geometry"], card.get("ground_z")))


def install_trace():
    """只记录走廊和深度图。选择仍走 UpstreamPolicy.plan，刹停走已合入的 empty_brake。"""
    import yopo_l0.policy as policy_mod
    from yopo_l0.dense_diag import implied_radius_m
    from yopo_swarm_core.candidates import corridor_mu_lo, select_action

    ec5 = load_ec5()
    previous = policy_mod.POLICIES["upstream"]

    class TraceUpstream(previous):
        instances: list = []

        def __init__(self, ctx):
            super().__init__(ctx)
            self.drone_id = ctx.owner
            self.trace: list[dict] = []
            self._last_fwd = None
            orig = self.source.forward_raw

            def wrapped(*args, **kwargs):
                out = orig(*args, **kwargs)
                self._last_fwd = out
                return out

            self.source.forward_raw = wrapped
            TraceUpstream.instances.append(self)

        def plan(self, obs):
            self._last_fwd = None
            out = super().plan(obs)
            n_admitted = None
            tightest = None
            prefilter_r = None
            fwd = self._last_fwd
            if fwd is not None:
                _params, score, radius = fwd
                dec = policy_mod.upstream_decoder(self.p["safe_radius"])
                if self._track_on and self._track_mode == "pred":
                    dec = policy_mod.upstream_decoder(float(self.p["safe_radius"]) + float(self._track_margin))
                mu = np.asarray(corridor_mu_lo(radius, dec), dtype=np.float64)
                admitted = mu >= np.float64(dec.safe_mu)
                n_admitted = int(admitted.sum())
                if n_admitted:
                    tightest = float(implied_radius_m(float(mu[admitted].min())))
                action, prefilter_brake = select_action(score, None, mu, float(dec.safe_mu))
                if not prefilter_brake:
                    prefilter_r = float(implied_radius_m(float(mu[int(action)])))
            braked = bool(self._empty_brake and self.last_code == "CORRIDOR_EMPTY_HOVER")
            state = obs.state
            self.trace.append({
                "drone": self.drone_id,
                "now": float(obs.now),
                "code": self.last_code,
                "p": np.asarray(state.p, dtype=float).copy(),
                "yaw": float(state.yaw),
                "frame": obs.frame,
                "n_admitted": n_admitted,
                "tightest_admitted_radius_m": tightest,
                "prefilter_chosen_radius_m": prefilter_r,
                "proto_braked": braked,
                "safe_radius_m": float(self.p["safe_radius"]),
            })
            return out

    TraceUpstream.__name__ = "TraceUpstream"
    TraceUpstream.instances = []
    policy_mod.POLICIES["upstream"] = TraceUpstream
    return TraceUpstream, previous, ec5


def restore_trace(previous) -> None:
    import yopo_l0.policy as policy_mod

    policy_mod.POLICIES["upstream"] = previous


def _traces(cls) -> dict[str, list]:
    out: dict[str, list] = {}
    for inst in getattr(cls, "instances", []):
        out[inst.drone_id] = list(inst.trace)
    return out


def _file_hash(scene: str) -> str:
    from yopo_l0.scene import load_card

    return load_card(ROOT / SCENES[scene]).hash


def _partial_path(scene: str, arm: str, seed: int) -> Path:
    return PARTIAL / f"{scene}-{arm}-{int(seed)}.json"


def cylinder_hit(row: dict) -> int:
    tree = row.get("min_tree_clearance_m")
    if tree is None or not math.isfinite(float(tree)):
        return 0
    return int(float(tree) <= BODY_RADIUS_M + 1e-9)


def run_one(scene: str, arm: str, seed: int, *, resume: bool) -> dict:
    dest = _partial_path(scene, arm, seed)
    if resume and dest.is_file():
        return json.loads(dest.read_text(encoding="utf-8"))
    from yopo_l0.runner import RunConfig, run

    card = build_card(scene, arm)
    diag_cls, previous, ec5 = install_trace()
    t0 = time.perf_counter()
    try:
        out = run(card, RunConfig(seed=int(seed), shadow=True))
        traces = _traces(diag_cls)
        attr = ec5.diagnose(card, out.result, out.ticks, traces)
        tree_m = ec5.min_tree_clearance(card, out.ticks)
        brake_plans = sum(1 for rows in traces.values() for row in rows if row.get("proto_braked"))
    finally:
        restore_trace(previous)
        diag_cls.instances = []
    res = out.result
    got = float(res["safe_radius_m"])
    if abs(got - UPSTREAM_SAFE_RADIUS_M) > 1e-9:
        raise RuntimeError(f"safe_radius 读回 {got}，本票五组都保持 0.05 m")
    if bool(res["upstream_deviation"]) != arm_deviates(arm):
        raise RuntimeError(f"{arm} 偏离标记 {res['upstream_deviation']} 与开关不一致")
    if arm == "off":
        if "track_clearance" in res or "empty_brake" in res:
            raise RuntimeError("全关臂不应写出 track_clearance / empty_brake")
    else:
        if res.get("empty_brake") is not True:
            raise RuntimeError(f"{arm} 没有打开 empty_brake")
    if arm in ("pred", "geom", "geom_multi"):
        if res.get("track_clearance") is not True or res.get("track_clearance_mode") != arm:
            raise RuntimeError(f"{arm} 的 track_clearance 读回 {res.get('track_clearance_mode')}")
        if abs(float(res.get("track_margin_m")) - MARGIN_M) > 1e-12:
            raise RuntimeError(f"{arm} margin 读回 {res.get('track_margin_m')}")
    reason = str(res["reason"])
    filt = res.get("inter_drone_filter") or {}
    row = {
        "scene": scene,
        "arm": arm,
        "seed": int(seed),
        "upstream_deviation": bool(res["upstream_deviation"]),
        "report_only": True,
        "frozen": False,
        "safe_radius_m": got,
        "decoder_radius_m": decoder_radius_m(arm),
        "track_margin_m": None if arm in ("off", "base") else MARGIN_M,
        "track_clearance_mode": None if arm in ("off", "base") else arm,
        "scene_card_hash": res["scene_card_hash"],
        "file_hash": _file_hash(scene),
        "outcome": res["outcome"],
        "reason": reason,
        "arrived": int(res["arrived"]),
        "n_drones": int(res["n_drones"]),
        "tree_collision": int(res["outcome"] == "COLLISION" and "TRUTH_POSITION" in reason),
        "cylinder_hit": None,  # 下面用全程圆柱净空填
        "inter_drone_collision": int(res["outcome"] == "COLLISION" and "INTER_DRONE" in reason),
        "timeout": int(res["outcome"] == "TIMEOUT"),
        "min_inter_drone_distance_m": res["min_inter_drone_distance_m"],
        "min_obstacle_distance_m": res["min_obstacle_distance_m"],
        "min_tree_clearance_m": tree_m,
        "sim_duration_s": res["sim_duration_s"],
        "arrival_time_s": res["arrival_time_s"],
        "plan_codes": res.get("plan_codes"),
        "empty_solution_count": res.get("empty_solution_count"),
        "inter_threshold_m": filt.get("threshold_m"),
        "inter_filtered": filt.get("filtered_candidates"),
        "inter_empty_fallbacks": filt.get("empty_fallbacks"),
        "brake_plans": int(brake_plans),
        "attribution": attr,
        "wall_time_s": res["wall_time_s"],
        "worker_s": time.perf_counter() - t0,
    }
    row["cylinder_hit"] = cylinder_hit(row)
    row = load_ec5()._jsonable(row)
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)
    gap = row["min_inter_drone_distance_m"]
    gap_s = "—" if gap is None else f"{float(gap):.3f}"
    tree = row["min_tree_clearance_m"]
    tree_s = "—" if tree is None else f"{float(tree):.3f}"
    cat = (row.get("attribution") or {}).get("category") or "—"
    print(
        f"{scene} {arm_label(arm)} seed={seed} {row['outcome']} "
        f"{row['arrived']}/{row['n_drones']} tree={row['tree_collision']} "
        f"cyl={row['cylinder_hit']} inter={row['inter_drone_collision']} "
        f"gap={gap_s} tree_clear={tree_s} sim={float(row['sim_duration_s']):.2f} "
        f"wall={float(row['wall_time_s']):.1f}s cat={cat}",
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


def job_specs(scenes, arms, seeds, *, resume: bool = True) -> list[tuple]:
    return [
        (scene, arm, int(seed), bool(resume))
        for arm in arms
        for scene in scenes
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


def _group(rows: list[dict], scene: str, arm: str) -> list[dict]:
    return [r for r in rows if r["scene"] == scene and r["arm"] == arm]


def render_summary(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | 偏离上游 | 到达 | 撞树 | 撞圆柱 | 机间碰撞 | 超时 | 最小树净空 m | 最小机间 m |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for scene in SCENE_ORDER:
        for arm in ARMS:
            group = _group(rows, scene, arm)
            if not group:
                continue
            drones = sum(int(r["n_drones"]) for r in group)
            arrived = sum(int(r["arrived"]) for r in group)
            tree = sum(int(r["tree_collision"]) for r in group)
            cyl = sum(int(r.get("cylinder_hit") or 0) for r in group)
            inter = sum(int(r["inter_drone_collision"]) for r in group)
            timeout = sum(int(r["timeout"]) for r in group)
            trees = [float(r["min_tree_clearance_m"]) for r in group if r.get("min_tree_clearance_m") is not None]
            gaps = [
                float(r["min_inter_drone_distance_m"])
                for r in group
                if r.get("min_inter_drone_distance_m") is not None
            ]
            dev = "偏离上游" if arm_deviates(arm) else "否"
            lines.append(
                f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | {dev} | {arrived}/{drones} | "
                f"{tree} | {cyl} | {inter} | {timeout} | {_rng(trees)} | {_rng(gaps)} |"
            )
    return "\n".join(lines)


def render_tree_counts(rows: list[dict]) -> str:
    ec5 = load_ec5()
    lines = [
        "| 卡 | 臂 | 撞树 | 跟踪误差 | 准入松 | 深度感知盲区 | 无解刹停后撞 | 无解悬停后撞 | 其他 |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for scene in SCENE_ORDER:
        for arm in ARMS:
            have = _group(rows, scene, arm)
            if not have:
                continue
            group = [r for r in have if int(r.get("tree_collision") or 0) == 1]
            counts = {name: 0 for name in ec5.TREE_ORDER}
            for row in group:
                cat = (row.get("attribution") or {}).get("category")
                if cat in counts:
                    counts[cat] += 1
                else:
                    counts[ec5.CAT_OTHER] += 1
            lines.append(
                f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | {len(group)} | "
                + " | ".join(str(counts[name]) for name in ec5.TREE_ORDER)
                + " |"
            )
    return "\n".join(lines)


def render_tree_details(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | seed | 飞机 | 主因 | 计划净空 m | 飞行段净空 m | 真值净空 m | 跟踪差 m | 深度里有 | 最紧已准入走廊 m | 规划码 |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    trees = sorted(
        [r for r in rows if int(r.get("tree_collision") or 0) == 1],
        key=lambda r: (SCENE_ORDER.index(r["scene"]), ARMS.index(r["arm"]), int(r["seed"])),
    )
    for row in trees:
        attr = row.get("attribution") or {}
        seen = attr.get("in_depth")
        seen_s = "—" if seen is None else ("有" if seen else "没有")
        tight = attr.get("fly_tightest_admitted_radius_m")
        if tight is None:
            tight = attr.get("tightest_admitted_radius_m")
        tight_s = "—" if tight is None else f"{float(tight):.3f}"

        def cell(key, _attr=attr):
            val = _attr.get(key)
            return "—" if val is None else f"{float(val):.3f}"

        lines.append(
            f"| {SCENE_LABEL[row['scene']]} | {arm_label(row['arm'])} | {row['seed']} | "
            f"{attr.get('drone') or '—'} | {attr.get('category') or '—'} | {cell('plan_clearance_m')} | "
            f"{cell('fly_clearance_m')} | {cell('actual_clearance_m')} | {cell('tracking_error_m')} | "
            f"{seen_s} | {tight_s} | `{attr.get('plan_code') or '—'}` |"
        )
    if len(lines) == 2:
        lines.append("| — | — | — | — | — | — | — | — | — | — | — | — |")
    return "\n".join(lines)


def render_seeds(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | seed | 结果 | 原因 | 到达 | 最小树净空 m | 最小机间 m | 仿真 s | 刹停拍 |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for scene in SCENE_ORDER:
        for arm in ARMS:
            for row in sorted(_group(rows, scene, arm), key=lambda r: int(r["seed"])):
                gap = row["min_inter_drone_distance_m"]
                gap_s = "—" if gap is None else f"{float(gap):.3f}"
                tree = row["min_tree_clearance_m"]
                tree_s = "—" if tree is None else f"{float(tree):.3f}"
                lines.append(
                    f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | {row['seed']} | "
                    f"{row['outcome']} | `{row['reason']}` | {row['arrived']}/{row['n_drones']} | "
                    f"{tree_s} | {gap_s} | {float(row['sim_duration_s']):.2f} | {int(row['brake_plans'])} |"
                )
    return "\n".join(lines)


def _check() -> int:
    import hashlib

    from yopo_l0.policy import UPSTREAM_SAFE_RADIUS_M as radius
    from yopo_l0.policy import read_empty_brake, read_track_clearance

    blob = (ROOT / "scenes" / GEOM_NAME).read_bytes()
    got = hashlib.sha256(blob).hexdigest()
    if got != GEOM_SHA256:
        raise SystemExit(f"{GEOM_NAME} sha256 变了：{got}")
    if abs(float(radius) - 0.05) > 1e-12 or read_track_clearance({}) or read_empty_brake({}):
        raise SystemExit("默认开关被改了")
    before = {rel: (ROOT / rel).read_bytes() for rel in SCENES.values()}
    before[f"scenes/{GEOM_NAME}"] = blob
    for scene in SCENE_ORDER:
        off = build_card(scene, "off")
        base = build_card(scene, "base")
        pred = build_card(scene, "pred")
        geom = build_card(scene, "geom")
        multi = build_card(scene, "geom_multi")
        if off.hash != _file_hash(scene):
            raise SystemExit(f"{scene} 全关臂哈希与场景文件不一致")
        if "policy_params" in off.card:
            raise SystemExit(f"{scene} 全关臂写了 policy_params")
        for card, mode in ((pred, "pred"), (geom, "geom"), (multi, "geom_multi")):
            up = card.card["policy_params"]["upstream"]
            if up.get("track_clearance") is not True or up.get("track_clearance_mode") != mode:
                raise SystemExit(f"{scene} {mode} 没写进进程内卡")
            if abs(float(up["track_margin_m"]) - MARGIN_M) > 1e-12:
                raise SystemExit(f"{scene} {mode} margin 不是 0.08")
            if "safe_radius" in up:
                raise SystemExit(f"{scene} 把 safe_radius 写进了进程内场景卡")
            if up.get("empty_brake") is not True or up.get("inter_drone_filter") is not True:
                raise SystemExit(f"{scene} {mode} 没有 base 的 E-A1 和 empty_brake")
            if abs(float(up["inter_drone_threshold_m"]) - 1.0) > 1e-12:
                raise SystemExit(f"{scene} E-A1 阈值不是 1.0 m")
        up = base.card["policy_params"]["upstream"]
        if "track_clearance" in up or "safe_radius" in up:
            raise SystemExit(f"{scene} base 臂多写了 track_clearance 或 safe_radius")
    after = {rel: (ROOT / rel).read_bytes() for rel in before}
    if after != before:
        raise SystemExit("检查场景卡时改写了文件")
    suite = (ROOT / "scenes" / "suite.txt").read_text(encoding="utf-8")
    if "e-c5b" in suite or "track_clearance" in suite:
        raise SystemExit("scenes/suite.txt 被写进了本票的卡")
    ec5 = load_ec5()
    if not hasattr(ec5, "diagnose") or not hasattr(ec5, "classify_tree"):
        raise SystemExit("没有复用 E-C5 归因")
    print("e-c5b check ok")
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
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--out", default="docs/research/e-c5b-track-clearance-runs.json")
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
    print(f"e-c5b start n={len(specs)} jobs={args.jobs} resume={resume}", flush=True)
    rows = _run_group(specs, int(args.jobs))
    rows.sort(key=lambda r: (
        SCENE_ORDER.index(r["scene"]),
        ARMS.index(r["arm"]),
        int(r["seed"]),
    ))
    ec5 = load_ec5()
    payload = {
        "disclaimer": DISCLAIMER,
        "commit": _head_commit(),
        "report_only": True,
        "frozen": False,
        "scenes": scenes,
        "arms": arms,
        "seeds": seeds,
        "jobs": int(args.jobs),
        "margin_m": MARGIN_M,
        "body_radius_m": BODY_RADIUS_M,
        "attribution_order": list(ec5.TREE_ORDER),
        "rows": rows,
        "note": DISCLAIMER,
    }
    dest = Path(args.out)
    if not dest.is_absolute():
        dest = ROOT / dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(render_summary(rows))
    print(render_tree_counts(rows))
    print(f"写入 {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
