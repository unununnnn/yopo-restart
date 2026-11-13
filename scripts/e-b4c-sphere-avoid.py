#!/usr/bin/env python3
"""E-B4c 小批量只报数：0.70 m 欧氏碰撞球 vs 只开后段 vs 全关。

场景（不改卡，开关在进程里覆盖）：

- swap-3-open，seeds 0–4
- crossing-pillars，seeds 0–4
- crossing-forest，seeds 0–4
- crossing-6-forest，seeds 0–2
- ego-density-014-swap3，seeds 0–4

臂：

- off：默认全关
- bspline_only：只开后段平滑，不开机间
- sphere：后段 + peer_sphere_enabled（半径 0.85 m，权重 8）
- sphere+track_geom_multi：sphere 再加 E-C5b 的 geom_multi

peer_sphere 默认关，不改椭球开关语义。现有接线只在 swarm_penalty 时把邻机
采样送进 refine_yopo。本脚本不改 policy.py / crossing.py，只在进程里包一层，
让欧氏球拿到采样，同时不打开椭球项。穿林/柱林走 CrossingFleet，不读
track_clearance；那一臂在这三张卡上应与 sphere 相同。

每粒落在 out/e-b4c-sphere-avoid/partial/，可续跑。后段预算仍是墙钟 0.02 s。
带后段的臂默认一个进程。
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
PARTIAL = ROOT / "out" / "e-b4c-sphere-avoid" / "partial"
BODY_R = 0.35
ARMS = ("off", "bspline_only", "sphere", "sphere+track_geom_multi")
# (名字, 场景卡, 默认种子)
SCENES = (
    ("swap-3-open", "scenes/swap-3-open.json", tuple(range(5))),
    ("crossing-pillars", "scenes/crossing-pillars.json", tuple(range(5))),
    ("crossing-forest", "scenes/crossing-forest.json", tuple(range(5))),
    ("crossing-6-forest", "scenes/crossing-6-forest.json", tuple(range(3))),
    ("ego-density-014-swap3", "scenes/ego-density-014-swap3.json", tuple(range(5))),
)
SCENE_NAMES = tuple(item[0] for item in SCENES)
SCENE_PATH = {name: path for name, path, _seeds in SCENES}
SCENE_SEEDS = {name: seeds for name, _path, seeds in SCENES}
DISCLAIMER = (
    "只报数。peer_sphere_enabled 默认关，不改 swarm_penalty / swarm_segment_penalty 的语义。"
    "打开欧氏球或 track_clearance 即偏离上游 d5e1e60。未冻结，不进 scenes/suite.txt，不设 gates。"
    "半径 0.85 m = 0.70 m 碰撞球 + 0.15 m 裕度。这不是论文复现。"
)
SPHERE = {
    "enabled": True,
    "peer_sphere_enabled": True,
    "peer_sphere_radius_m": 0.85,
    "peer_sphere_weight": 8.0,
}
_INSTALLED = False


def _deep(raw: dict) -> dict:
    return json.loads(json.dumps(raw))


def build_card(scene: str, arm: str):
    from yopo_l0.scene import SceneCard, load_card

    if scene not in SCENE_PATH:
        raise ValueError(scene)
    if arm not in ARMS:
        raise ValueError(arm)
    loaded = load_card(ROOT / SCENE_PATH[scene])
    if arm == "off":
        return loaded
    raw = _deep(loaded.card)
    params = dict(raw.get("policy_params") or {})
    backend = dict(params.get("backend") or {})
    if arm == "bspline_only":
        backend["enabled"] = True
        backend.pop("peer_sphere_enabled", None)
        backend.pop("swarm_penalty", None)
        backend.pop("swarm_segment_penalty", None)
    elif arm in ("sphere", "sphere+track_geom_multi"):
        backend.update(SPHERE)
        backend.pop("swarm_penalty", None)
        backend.pop("swarm_segment_penalty", None)
    else:
        raise ValueError(arm)
    params["backend"] = backend
    if arm == "sphere+track_geom_multi":
        upstream = dict(params.get("upstream") or {})
        upstream["track_clearance"] = True
        upstream["track_clearance_mode"] = "geom_multi"
        params["upstream"] = upstream
    raw["policy_params"] = params
    return SceneCard(loaded.path, raw, loaded.obstacles)


def install_peer_delivery() -> None:
    """欧氏球打开时把邻机采样送进后段。椭球项仍只看 swarm_penalty。

    policy.py 和 crossing.py 不改。包一层之后，两个开关都关时传进
    refine_yopo 的邻机仍是 None，和原来的调用相同。
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
    if getattr(current, "_eb4c_wrapped", False):
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
    wrapped._eb4c_wrapped = True
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


def _row(scene: str, arm: str, result: dict, *, budget_calls: int, budget_cuts: int) -> dict:
    reason = result.get("reason") or ""
    outcome = result.get("outcome")
    backend = _drop_samples(result.get("backend"))
    time_ms = (backend or {}).get("time_ms") or {}
    sphere = arm in ("sphere", "sphere+track_geom_multi")
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
        "obstacle_collisions": int(outcome == "COLLISION" and "TRUTH_POSITION" in str(reason)),
        "inter_drone_collisions": int(outcome == "COLLISION" and "INTER_DRONE" in str(reason)),
        "timeout": int(outcome == "TIMEOUT"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "min_obstacle_distance_m": result.get("min_obstacle_distance_m"),
        "sim_duration_s": result.get("sim_duration_s"),
        "final_goal_error_m": result.get("final_goal_error_m"),
        "wall_time_s": result.get("wall_time_s"),
        "backend": backend,
        "backend_time_p50_ms": time_ms.get("p50"),
        "backend_time_p95_ms": time_ms.get("p95"),
        "backend_budget_calls": int(budget_calls),
        "backend_budget_cuts": int(budget_cuts),
        "scene_card_hash": result.get("scene_card_hash"),
        "commit": result.get("commit"),
        "deviates_from_upstream": arm != "off",
        "runner_upstream_deviation": result.get("upstream_deviation"),
        "track_clearance_mode": result.get("track_clearance_mode"),
        "peer_sphere_enabled": sphere,
        "peer_sphere_radius_m": SPHERE["peer_sphere_radius_m"] if sphere else None,
        "peer_sphere_weight": SPHERE["peer_sphere_weight"] if sphere else None,
        "disclaimer": DISCLAIMER,
    }


def _partial(scene: str, arm: str, seed: int) -> Path:
    safe = arm.replace("+", "_")
    return PARTIAL / f"{scene}-{safe}-{seed}.json"


def run_one(scene: str, arm: str, seed: int, *, resume: bool) -> dict:
    dest = _partial(scene, arm, seed)
    if resume and dest.is_file():
        return json.loads(dest.read_text(encoding="utf-8"))
    from yopo_l0.runner import RunConfig, run

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
    gap = row["min_inter_drone_distance_m"]
    gap_s = "—" if gap is None else f"{float(gap):.3f}"
    arr = row["arrival_time_s"]
    arr_s = "—" if arr is None else f"{float(arr):.2f}"
    print(
        f"{scene} {arm} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"obs={row['obstacle_collisions']} inter={row['inter_drone_collisions']} "
        f"gap={gap_s} arrive={arr_s}s cuts={row['backend_budget_cuts']}/{row['backend_budget_calls']} "
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


def summarize(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 臂 | 到达 | 障碍碰撞 | 机间碰撞 | 超时 | 最小机间 m | 到达时间 s | 后段 p50 / p95 ms | 预算截断 |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    scenes = []
    for row in rows:
        if row["scene_kind"] not in scenes:
            scenes.append(row["scene_kind"])
    for scene in scenes:
        for arm in ARMS:
            group = [r for r in rows if r["scene_kind"] == scene and r["arm"] == arm]
            if not group:
                continue
            arrived = sum(int(r["arrived"]) for r in group)
            total = sum(int(r["n_drones"]) for r in group)
            obs = sum(int(r["obstacle_collisions"]) for r in group)
            inter = sum(int(r["inter_drone_collisions"]) for r in group)
            timeout = sum(int(r["timeout"]) for r in group)
            gaps = [float(r["min_inter_drone_distance_m"]) for r in group if r.get("min_inter_drone_distance_m") is not None]
            times = [float(r["arrival_time_s"]) for r in group if r.get("arrival_time_s") is not None]
            p50 = [float(r["backend_time_p50_ms"]) for r in group if r.get("backend_time_p50_ms") is not None]
            p95 = [float(r["backend_time_p95_ms"]) for r in group if r.get("backend_time_p95_ms") is not None]
            cuts = sum(int(r.get("backend_budget_cuts") or 0) for r in group)
            calls = sum(int(r.get("backend_budget_calls") or 0) for r in group)
            backend_t = "—" if not p50 else f"{_rng(p50, 1)} / {_rng(p95, 1)}"
            lines.append(
                f"| {scene} | {arm} | {arrived}/{total} | {obs} | {inter} | {timeout} | "
                f"{_rng(gaps, 3)} | {_rng(times, 2)} | {backend_t} | {cuts}/{calls} |"
            )
    return "\n".join(lines) + "\n"


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


def _parse_seeds(text: str | None, scene: str) -> tuple[int, ...]:
    if text is None:
        return SCENE_SEEDS[scene]
    return tuple(int(s) for s in text.split(",") if s != "")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", default=",".join(SCENE_NAMES))
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=None, help="盖过每张卡自己的种子。缺省用票面种子。")
    ap.add_argument("--jobs-plain", type=int, default=2)
    ap.add_argument("--jobs-backend", type=int, default=1)
    ap.add_argument("--out", default="docs/research/e-b4c-sphere-avoid-runs.json")
    ap.add_argument("--no-resume", action="store_true")
    a = ap.parse_args(argv)
    scenes = [s for s in a.scenes.split(",") if s]
    arms = [s for s in a.arms.split(",") if s]
    unknown = [s for s in scenes if s not in SCENE_NAMES] + [s for s in arms if s not in ARMS]
    if unknown:
        raise SystemExit(f"未知场景或臂: {unknown}")
    if int(a.jobs_plain) > 2 or int(a.jobs_backend) > 2:
        raise SystemExit("最多 2 个进程，避免墙钟预算把后段迭代截短")
    resume = not a.no_resume
    jobs = []
    for scene in scenes:
        seeds = _parse_seeds(a.seeds, scene)
        for arm in arms:
            for seed in seeds:
                jobs.append((scene, arm, int(seed)))

    from yopo_swarm_core.baseline import fetch_upstream

    fetch_upstream()
    plain = [job for job in jobs if job[1] == "off"]
    backend = [job for job in jobs if job[1] != "off"]
    if plain:
        _run_group(plain, workers=max(1, int(a.jobs_plain)), resume=resume)
    if backend:
        _run_group(backend, workers=max(1, int(a.jobs_backend)), resume=resume)

    rows = _load_partials(jobs)
    rows.sort(key=lambda r: (SCENE_NAMES.index(r["scene_kind"]), ARMS.index(r["arm"]), int(r["seed"])))
    payload = {
        "disclaimer": DISCLAIMER,
        "peer_sphere_radius_m": SPHERE["peer_sphere_radius_m"],
        "peer_sphere_weight": SPHERE["peer_sphere_weight"],
        "collision_ball_m": 0.70,
        "default_off": True,
        "report_only": True,
        "deviates_from_upstream_when_enabled": True,
        "note": (
            "欧氏球默认关。铰链只在时间对齐的 3D 距离 < 0.85 m 时亮："
            "0.70–0.85 m 是三次过渡，小于 0.70 m 接二次。"
            "惩罚打在除最后一跨外的段内采样上，末端控制点仍钉死，障碍走廊权重不动。"
            "最小机间距离是运行器第一次越过 0.70 m 那一拍的采样；没撞则是全程最小。"
            "crossing 不读 track_clearance，sphere+track_geom_multi 在柱林/穿林上不另改几何门。"
            "墙钟不参与轨迹，但后段 0.02 s 预算按墙钟截断。"
        ),
        "scenes": scenes,
        "arms": arms,
        "seeds_by_scene": {scene: list(_parse_seeds(a.seeds, scene)) for scene in scenes},
        "completed": len(rows),
        "expected": len(jobs),
        "incomplete": len(jobs) - len(rows),
        "rows": rows,
    }
    out = ROOT / a.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(summarize(rows))
    print(f"wrote {out} ({payload['completed']}/{payload['expected']})")
    return 0 if payload["incomplete"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
