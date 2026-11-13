#!/usr/bin/env python3
"""票 V3：E-B4c 欧氏碰撞球开关前后的俯视对比。只出图，不改算法。

两档都在进程里盖参数，不写场景卡：

- ``base``：后段 ``enabled=true``，``peer_sphere_enabled=false``
- ``sphere``：后段开，并且 ``peer_sphere_enabled=true``，半径 0.85 m，权重 8

默认三粒（``--cases`` 可覆盖）：

- ``scenes/swap-3-open.json`` seed 0
- ``scenes/ego-density-014-swap3.json`` seed 0
- ``scenes/ego-density-014-swap3.json`` seed 1（E-B4c 全程里剩下的 ``uav1-uav2`` 碰撞）

当前检出没有 ``BackendConfig.peer_sphere_enabled`` 时，去掉 ``--partial`` 会直接失败。
加上 ``--partial`` 仍用现有后段飞 ``base``，``sphere`` 档不飞，在 ``summary.json`` 里标 ``missing``。
开关在代码里之后，两档都会飞；邻机采样由本脚本在进程里送进 ``refine_yopo``，不改 ``policy.py``。

俯视对比写 ``.gif``。ffmpeg 在 PATH 里再写 ``.mp4``。每粒一张静图，全部粒再拼一张总览 ``.png``。

用法::

    .venv/bin/python scripts/v3-sphere-replay.py --partial --out /tmp/v3-smoke
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation, font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

DEFAULT_OUT = Path("/tmp/v3-sphere-replay")
PARTIAL_DURATION_S = 2.0
COLLIDE_M = 0.70
SPHERE_RADIUS_M = 0.85
SPHERE_WEIGHT = 8.0
BODY_RADIUS_M = 0.35
MISSING_SWITCH = (
    "当前代码没有 peer_sphere_enabled（E-B4c，PR #94，分支 yopo-restart-bot/e-b4c-sphere-avoid）。"
    "sphere 档跳过，summary 里标 missing。后段 base 仍用现有后端飞。"
    "完整对比等该开关进入检出后，去掉 --partial 再跑。"
)
FULL_SWITCH_ERROR = (
    "当前代码没有 peer_sphere_enabled，不能做 base / sphere 完整对比。"
    "这个开关在 PR #94（分支 yopo-restart-bot/e-b4c-sphere-avoid）。"
    "可先加 --partial：后段照常打开，sphere 档在 summary.json 里标成 missing。"
)
_SPHERE_ENV = ("YOPO_PEER_SPHERE", "YOPO_PEER_SPHERE_RADIUS_M", "YOPO_PEER_SPHERE_WEIGHT")

SCENE_PATHS = {
    "swap-3-open": "scenes/swap-3-open.json",
    "ego-density-014-swap3": "scenes/ego-density-014-swap3.json",
}
SCENE_TITLE = {
    "swap-3-open": "开阔三机对向",
    "ego-density-014-swap3": "密度 0.14 三机对向",
}
SCENE_ALIASES = {
    "swap-3-open": "swap-3-open",
    "swap3-open": "swap-3-open",
    "scenes/swap-3-open.json": "swap-3-open",
    "ego-density-014-swap3": "ego-density-014-swap3",
    "d014-swap3": "ego-density-014-swap3",
    "swap3": "ego-density-014-swap3",
    "scenes/ego-density-014-swap3.json": "ego-density-014-swap3",
}
BASE_ARM = {
    "id": "base",
    "title": "base（后段开，球关）",
    "short": "base",
    "peer_sphere_enabled": False,
}
SPHERE_ARM = {
    "id": "sphere",
    "title": "sphere（半径 0.85 m）",
    "short": "sphere",
    "peer_sphere_enabled": True,
}
ARM_ORDER = ("base", "sphere")
COLORS = ("#d62728", "#1f77b4", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf", "#e377c2", "#8c564b")
_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)
_FONT_READY = False
_INSTALLED = False


class SphereSwitchMissing(RuntimeError):
    """要开欧氏球，但 BackendConfig 上还没有这个字段。"""


def peer_sphere_available() -> bool:
    """检出里的后段配置是否已经有 E-B4c 的开关。不改任何文件。"""
    from yopo_l0.backend.config import BackendConfig

    fields = getattr(BackendConfig, "__dataclass_fields__", {})
    return "peer_sphere_enabled" in fields


def isolate_sphere_env() -> None:
    """对比只认场景卡字段。环境变量会把两档一起打开，这里清掉。"""
    for key in _SPHERE_ENV:
        os.environ.pop(key, None)


def default_cases() -> list[dict]:
    return [
        {"scene": "swap-3-open", "path": SCENE_PATHS["swap-3-open"], "seed": 0, "title": SCENE_TITLE["swap-3-open"]},
        {
            "scene": "ego-density-014-swap3",
            "path": SCENE_PATHS["ego-density-014-swap3"],
            "seed": 0,
            "title": SCENE_TITLE["ego-density-014-swap3"],
        },
        {
            "scene": "ego-density-014-swap3",
            "path": SCENE_PATHS["ego-density-014-swap3"],
            "seed": 1,
            "title": SCENE_TITLE["ego-density-014-swap3"],
        },
    ]


def resolve_scene(key: str) -> tuple[str, str]:
    alias = SCENE_ALIASES.get(key) or SCENE_ALIASES.get(key.removesuffix(".json"))
    if alias is not None:
        return alias, SCENE_PATHS[alias]
    path = Path(key)
    if not path.is_file():
        path = REPO / key
    if not path.is_file():
        known = ", ".join(sorted(SCENE_ALIASES))
        raise SystemExit(f"未知场景 {key!r}。已知别名：{known}，或给一个存在的场景卡路径")
    rel = path.resolve().relative_to(REPO).as_posix() if path.resolve().is_relative_to(REPO) else str(path.resolve())
    return path.stem, rel


def resolve_cases(text: str | None) -> list[dict]:
    if not text:
        return default_cases()
    chosen: list[dict] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" not in part:
            raise SystemExit(f"用例要写成 场景:seed，收到 {part!r}")
        scene_key, seed_text = part.rsplit(":", 1)
        try:
            seed = int(seed_text)
        except ValueError as exc:
            raise SystemExit(f"seed 不是整数：{part!r}") from exc
        scene, rel = resolve_scene(scene_key.strip())
        chosen.append({"scene": scene, "path": rel, "seed": seed, "title": SCENE_TITLE.get(scene, scene)})
    if not chosen:
        raise SystemExit("`--cases` 是空的")
    return chosen


def prepare_font() -> None:
    global _FONT_READY
    if _FONT_READY:
        return
    for path in _FONT_CANDIDATES:
        if not os.path.isfile(path):
            continue
        try:
            font_manager.fontManager.addfont(path)
            name = font_manager.FontProperties(fname=path).get_name()
        except (OSError, RuntimeError, ValueError):
            continue
        plt.rcParams["font.family"] = "sans-serif"
        plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        _FONT_READY = True
        return
    _FONT_READY = True


def build_card(scene_path: Path, arm: dict):
    """进程内盖后段和欧氏球。不写场景文件。"""
    from yopo_l0.scene import SceneCard, load_card

    isolate_sphere_env()
    if arm["id"] == "sphere" and not peer_sphere_available():
        raise SphereSwitchMissing(MISSING_SWITCH)
    loaded = load_card(scene_path)
    raw = json.loads(json.dumps(loaded.card))
    params = dict(raw.get("policy_params") or {})
    backend = dict(params.get("backend") or {})
    backend["enabled"] = True
    backend.pop("swarm_penalty", None)
    backend.pop("swarm_segment_penalty", None)
    if peer_sphere_available():
        if arm["id"] == "sphere":
            backend["peer_sphere_enabled"] = True
            backend["peer_sphere_radius_m"] = SPHERE_RADIUS_M
            backend["peer_sphere_weight"] = SPHERE_WEIGHT
        else:
            backend["peer_sphere_enabled"] = False
            backend.pop("peer_sphere_radius_m", None)
            backend.pop("peer_sphere_weight", None)
    else:
        backend.pop("peer_sphere_enabled", None)
        backend.pop("peer_sphere_radius_m", None)
        backend.pop("peer_sphere_weight", None)
    params["backend"] = backend
    raw["policy_params"] = params
    return SceneCard(loaded.path, raw, loaded.obstacles)


def install_peer_delivery() -> bool:
    """欧氏球打开时把邻机采样送进后段。椭球项仍只看 swarm_penalty。

    policy.py / crossing.py 不改文件。两个开关都关时，传进 refine_yopo 的邻机仍是 None。
    没有 peer_sphere_enabled 字段时什么都不包。
    """
    global _INSTALLED
    if _INSTALLED:
        return True
    if not peer_sphere_available():
        return False
    import yopo_l0.backend.config as cfg_mod
    import yopo_l0.crossing as crossing_mod
    import yopo_l0.policy as policy_mod
    import yopo_l0.runner as runner_mod

    sphere_on = getattr(cfg_mod, "peer_sphere_enabled", None)

    def sphere_from_params(params, **kwargs) -> bool:
        if sphere_on is not None:
            return bool(sphere_on(params, **kwargs))
        cfg = cfg_mod.config_from_params(params, **kwargs)
        return cfg is not None and bool(getattr(cfg, "peer_sphere_enabled", False))

    orig_enabled = runner_mod.swarm_penalty_enabled

    def enabled(params, **kwargs):
        if orig_enabled(params, **kwargs):
            return True
        return sphere_from_params(params, **kwargs)

    runner_mod.swarm_penalty_enabled = enabled
    orig_keep = policy_mod.UpstreamPolicy._keep_neighbor_samples

    def keep(self):
        if orig_keep(self):
            return True
        cfg = getattr(self, "_backend", None)
        return cfg is not None and bool(getattr(cfg, "peer_sphere_enabled", False))

    policy_mod.UpstreamPolicy._keep_neighbor_samples = keep

    def apply(self, traj, goal):
        from yopo_l0.backend.bspline import refine_yopo

        cfg = self._backend
        use = bool(getattr(self, "_swarm", False)) or (
            cfg is not None and bool(getattr(cfg, "peer_sphere_enabled", False))
        )
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
        wants = cfg is not None and (
            bool(getattr(cfg, "swarm_penalty", False)) or bool(getattr(cfg, "peer_sphere_enabled", False))
        )
        if wants and getattr(self, "_score_node", None) is not None:
            neighbors = committed_neighbor_samples(self._score_node, float(traj.t_start))
        out, rec = refine_yopo(traj, cfg, self._backend_obstacles, goal, neighbors=neighbors)
        self.backend_log.append(rec)
        return out

    crossing_mod.CrossingFleet._apply_backend = cross_apply
    _INSTALLED = True
    return True


def pair_from_reason(reason: str | None) -> list[str] | None:
    if not reason or "INTER_DRONE" not in str(reason):
        return None
    left = str(reason).split(":", 1)[0]
    parts = left.split("-")
    if len(parts) != 2 or not parts[0] or not parts[1]:
        return None
    return sorted(parts)


def analyze_inter(times: np.ndarray, uavs: list[str], tracks: dict[str, np.ndarray], reason: str | None, collide_m: float) -> dict:
    """全程最小机间，以及中心距进入碰撞线的对。reason 里的 INTER_DRONE 也会留下。"""
    best: dict[tuple[str, str], dict] = {}
    n = len(times)
    for i in range(n):
        for a in range(len(uavs)):
            pa = tracks[uavs[a]][i]
            if not np.isfinite(pa).all():
                continue
            for b in range(a + 1, len(uavs)):
                pb = tracks[uavs[b]][i]
                if not np.isfinite(pb).all():
                    continue
                dist = float(np.linalg.norm(pa - pb))
                key = tuple(sorted((uavs[a], uavs[b])))
                prev = best.get(key)
                if prev is None or dist < prev["min_distance_m"]:
                    mid = ((pa + pb) / 2.0).tolist()
                    best[key] = {
                        "pair": [key[0], key[1]],
                        "min_distance_m": dist,
                        "t_s": float(times[i]),
                        "mid_xy": [float(mid[0]), float(mid[1])],
                    }
    if not best:
        named = pair_from_reason(reason)
        pairs = []
        if named is not None:
            pairs.append({"pair": named, "min_distance_m": None, "t_s": None, "mid_xy": None})
        return {"min_inter_drone_distance_m": None, "collision_pairs": pairs, "closest_pair": None}
    closest = min(best.values(), key=lambda row: (row["min_distance_m"], row["pair"]))
    colliding = [row for row in best.values() if row["min_distance_m"] <= float(collide_m) + 1e-12]
    named = pair_from_reason(reason)
    if named is not None and not any(row["pair"] == named for row in colliding):
        extra = best.get(tuple(named))
        colliding.append(extra if extra is not None else {"pair": named, "min_distance_m": None, "t_s": None, "mid_xy": None})
    colliding.sort(key=lambda row: (row["min_distance_m"] is None, row["min_distance_m"] or 0.0, row["pair"]))
    return {
        "min_inter_drone_distance_m": float(closest["min_distance_m"]),
        "collision_pairs": colliding,
        "closest_pair": closest,
    }


def load_ticks(path: Path) -> list[dict]:
    rows: list[dict] = []
    text = path.read_text(encoding="utf-8")
    for lineno, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}:{lineno} 不是 JSON：{exc}") from exc
    if not rows:
        raise ValueError(f"日志是空的：{path}")
    return rows


def align_ticks(ticks: list[dict]) -> tuple[np.ndarray, list[str], dict[str, np.ndarray]]:
    by_k: dict[int, dict] = {}
    for row in ticks:
        try:
            k = int(row["k"])
            t = float(row["t"])
            uav = str(row["uav"])
            p = row["p"]
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"无法解析 tick：{row!r}") from exc
        if not isinstance(p, (list, tuple)) or len(p) != 3:
            raise ValueError(f"tick 的 p 必须是长度 3：{row!r}")
        slot = by_k.setdefault(k, {"t": t, "uavs": {}})
        slot["t"] = t
        slot["uavs"][uav] = np.asarray(p, dtype=float)
    if not by_k:
        raise ValueError("ticks 里没有对得齐的帧")
    ks = sorted(by_k)
    uavs: list[str] = []
    for k in ks:
        for uav in by_k[k]["uavs"]:
            if uav not in uavs:
                uavs.append(uav)
    tracks = {uav: np.full((len(ks), 3), np.nan) for uav in uavs}
    times = np.empty(len(ks))
    for i, k in enumerate(ks):
        times[i] = by_k[k]["t"]
        for uav, p in by_k[k]["uavs"].items():
            tracks[uav][i] = p
    return times, uavs, tracks


def persist_log(log_dir: Path, result: dict, ticks: list[dict]) -> None:
    log_dir.mkdir(parents=True, exist_ok=True)
    (log_dir / "result.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    with (log_dir / "ticks.jsonl").open("w", encoding="utf-8") as handle:
        for row in ticks:
            handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")


def run_sim(scene_path: Path, arm: dict, seed: int, duration: float | None, log_dir: Path) -> tuple[dict, list[dict], float]:
    from yopo_l0.runner import RunConfig, run

    card = build_card(scene_path, arm)
    log_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(seed), shadow=True, duration_s=duration))
    wall = time.perf_counter() - t0
    persist_log(log_dir, out.result, out.ticks)
    return out.result, out.ticks, wall


@dataclass
class Clip:
    scene: str
    scene_path: str
    title: str
    arm: dict
    seed: int
    status: str
    times: np.ndarray
    uavs: list[str]
    tracks: dict[str, np.ndarray]
    ground_z: float
    body_radius: float
    collide_m: float
    obstacles: tuple
    goals: dict[str, list[float]]
    result: dict
    min_inter_m: float | None
    collision_pairs: list[dict] | None
    closest_pair: dict | None
    log_dir: Path | None
    wall_time_s: float | None
    missing: str | None


def _card_view(scene_path: Path) -> tuple[object, float, float, tuple, dict[str, list[float]]]:
    from yopo_l0.scene import load_card

    card = load_card(scene_path)
    ground = card.card.get("ground_z")
    ground_z = 0.0 if ground is None else float(ground)
    radius = float(card.card["criteria"]["body_radius_m"])
    goals = {str(drone["id"]): [float(x) for x in drone["goal_enu"]] for drone in card.card["drones"]}
    return card, ground_z, radius, card.obstacles, goals


def clip_from_run(case: dict, arm: dict, scene_path: Path, result: dict, ticks: list[dict], log_dir: Path, wall: float | None) -> Clip:
    _card, ground_z, radius, obstacles, goals = _card_view(scene_path)
    times, uavs, tracks = align_ticks(ticks)
    collide_m = 2.0 * radius
    report = analyze_inter(times, uavs, tracks, result.get("reason"), collide_m)
    return Clip(
        scene=case["scene"],
        scene_path=case["path"],
        title=case["title"],
        arm=dict(arm),
        seed=int(case["seed"]),
        status="ok",
        times=times,
        uavs=uavs,
        tracks=tracks,
        ground_z=ground_z,
        body_radius=radius,
        collide_m=collide_m,
        obstacles=obstacles,
        goals=goals,
        result=result,
        min_inter_m=report["min_inter_drone_distance_m"],
        collision_pairs=report["collision_pairs"],
        closest_pair=report["closest_pair"],
        log_dir=log_dir,
        wall_time_s=wall,
        missing=None,
    )


def missing_clip(case: dict, arm: dict, scene_path: Path) -> Clip:
    _card, ground_z, radius, obstacles, goals = _card_view(scene_path)
    titled = dict(arm)
    titled["title"] = "sphere（开关缺失）"
    titled["short"] = "sphere 缺失"
    return Clip(
        scene=case["scene"],
        scene_path=case["path"],
        title=case["title"],
        arm=titled,
        seed=int(case["seed"]),
        status="missing",
        times=np.asarray([0.0]),
        uavs=[],
        tracks={},
        ground_z=ground_z,
        body_radius=radius,
        collide_m=2.0 * radius,
        obstacles=obstacles,
        goals=goals,
        result={"outcome": None, "reason": None, "arrived": None, "n_drones": None, "sim_duration_s": None},
        min_inter_m=None,
        collision_pairs=None,
        closest_pair=None,
        log_dir=None,
        wall_time_s=None,
        missing=MISSING_SWITCH,
    )


def scene_file(case: dict) -> Path:
    raw = Path(case["path"])
    if raw.is_absolute():
        return raw
    return REPO / raw


def ensure_clip(case: dict, arm: dict, duration: float | None, out: Path, reuse: bool) -> Clip:
    scene_path = scene_file(case)
    if arm.get("status") == "missing":
        print(f"跳过 {case['scene']} sphere seed={case['seed']}：{MISSING_SWITCH}", flush=True)
        return missing_clip(case, arm, scene_path)
    log_dir = out / "logs" / f"{case['scene']}-{arm['id']}-seed{case['seed']}"
    ticks_path = log_dir / "ticks.jsonl"
    result_path = log_dir / "result.json"
    wall: float | None = None
    if reuse and ticks_path.is_file() and result_path.is_file():
        print(f"复用日志 {log_dir}", flush=True)
        ticks = load_ticks(ticks_path)
        result = json.loads(result_path.read_text(encoding="utf-8"))
    else:
        dur = "场景卡" if duration is None else f"{duration:g} s"
        print(f"开跑 {case['scene']} {arm['id']} seed={case['seed']} 时长={dur}", flush=True)
        result, ticks, wall = run_sim(scene_path, arm, int(case["seed"]), duration, log_dir)
        print(
            f"  完成 {case['scene']} {arm['id']} seed={case['seed']}  outcome={result.get('outcome')}  "
            f"到达 {result.get('arrived')}/{result.get('n_drones')}  墙钟 {wall:.1f} s",
            flush=True,
        )
    return clip_from_run(case, arm, scene_path, result, ticks, log_dir, wall)


def _span(lo: float, hi: float, minimum: float) -> tuple[float, float]:
    if hi - lo >= minimum:
        return lo, hi
    mid = (lo + hi) / 2.0
    return mid - minimum / 2.0, mid + minimum / 2.0


def view_limits(clips: list[Clip]) -> tuple[tuple[float, float], tuple[float, float]]:
    from yopo_l0.scene import Box, Cylinder

    xs: list[float] = []
    ys: list[float] = []
    for clip in clips:
        for track in clip.tracks.values():
            finite = track[np.isfinite(track).all(axis=1)]
            if len(finite):
                xs.extend([float(finite[:, 0].min()), float(finite[:, 0].max())])
                ys.extend([float(finite[:, 1].min()), float(finite[:, 1].max())])
        for goal in clip.goals.values():
            xs.append(float(goal[0]))
            ys.append(float(goal[1]))
    if not xs:
        return (-12.0, 12.0), (-12.0, 12.0)
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    reach = 2.0
    for clip in clips:
        for ob in clip.obstacles:
            if isinstance(ob, Cylinder):
                cx, cy = ob.section_xy(1.5)
                if xmin - reach <= cx <= xmax + reach and ymin - reach <= cy <= ymax + reach:
                    r = float(ob.radius)
                    xs.extend([cx - r, cx + r])
                    ys.extend([cy - r, cy + r])
            elif isinstance(ob, Box):
                if xmin - reach <= float(ob.hi[0]) and xmax + reach >= float(ob.lo[0]) and ymin - reach <= float(ob.hi[1]) and ymax + reach >= float(ob.lo[1]):
                    xs.extend([float(ob.lo[0]), float(ob.hi[0])])
                    ys.extend([float(ob.lo[1]), float(ob.hi[1])])
    pad = 1.6
    return _span(min(xs) - pad, max(xs) + pad, 8.0), _span(min(ys) - pad, max(ys) + pad, 8.0)


def draw_obstacles(ax, obstacles) -> None:
    from yopo_l0.scene import Box, Cylinder

    tree_kw = dict(facecolor="#9cbb7a", edgecolor="#4d6b32", lw=0.4, alpha=0.9, zorder=1)
    box_kw = dict(facecolor="#c8c8c8", edgecolor="#666666", lw=0.4, zorder=0)
    for ob in obstacles:
        if isinstance(ob, Cylinder):
            cx, cy = ob.section_xy(1.5)
            ax.add_patch(Circle((cx, cy), ob.radius, **tree_kw))
        elif isinstance(ob, Box):
            x, y, _z = ob.lo
            ax.add_patch(Rectangle((x, y), float(ob.hi[0] - ob.lo[0]), float(ob.hi[1] - ob.lo[1]), **box_kw))


def draw_goals(ax, clip: Clip) -> None:
    for i, (uav, goal) in enumerate(clip.goals.items()):
        color = COLORS[i % len(COLORS)]
        ax.plot([goal[0]], [goal[1]], marker="x", color=color, ms=6, mew=1.2, alpha=0.75, zorder=2)


def style_xy(ax, xlim, ylim) -> None:
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.grid(True, alpha=0.28)
    ax.set_facecolor("#fbfbf7")


def index_at(times: np.ndarray, t: float) -> int:
    if len(times) == 0:
        return 0
    if t >= float(times[-1]):
        return int(len(times) - 1)
    if t <= float(times[0]):
        return 0
    return int(np.searchsorted(times, t, side="right") - 1)


def closest_index(clip: Clip) -> int:
    if clip.closest_pair is None or clip.status != "ok":
        return int(len(clip.times) - 1) if len(clip.times) else 0
    return index_at(clip.times, float(clip.closest_pair["t_s"]))


def _frame_pair(clip: Clip, idx: int):
    best = None
    pair = None
    pa = pb = None
    for a in range(len(clip.uavs)):
        left = clip.tracks[clip.uavs[a]][idx]
        if not np.isfinite(left).all():
            continue
        for b in range(a + 1, len(clip.uavs)):
            right = clip.tracks[clip.uavs[b]][idx]
            if not np.isfinite(right).all():
                continue
            dist = float(np.linalg.norm(left - right))
            if best is None or dist < best:
                best = dist
                pair = (clip.uavs[a], clip.uavs[b])
                pa, pb = left, right
    return best, pair, pa, pb


def column_artists(ax, clip: Clip) -> dict:
    if clip.status != "ok":
        ax.text(
            0.5,
            0.55,
            "sphere 档缺失\n检出里还没有 peer_sphere_enabled\n这一档没有飞行",
            transform=ax.transAxes,
            ha="center",
            va="center",
            fontsize=10,
            color="#9b0000",
            linespacing=1.45,
            zorder=8,
            bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor="#e0b4b4", alpha=0.92),
        )
        return {"empty": True}
    trails, bodies, dots, labels, halos = [], [], [], [], []
    show_halo = bool(clip.arm.get("peer_sphere_enabled"))
    for i, uav in enumerate(clip.uavs):
        color = COLORS[i % len(COLORS)]
        (trail,) = ax.plot([], [], color=color, lw=1.4, zorder=3, solid_capstyle="round")
        body = Circle((0, 0), clip.body_radius, fill=True, facecolor=color, alpha=0.22, ec=color, lw=1.1, zorder=4)
        ax.add_patch(body)
        halo = None
        if show_halo:
            halo = Circle((0, 0), SPHERE_RADIUS_M, fill=False, ec=color, lw=0.7, ls="--", alpha=0.8, zorder=3)
            ax.add_patch(halo)
        (dot,) = ax.plot([], [], "o", color=color, ms=3.2, zorder=5)
        label = ax.annotate(
            uav[3:] if uav.startswith("uav") else uav,
            xy=(0, 0),
            xytext=(4, 4),
            textcoords="offset points",
            color=color,
            fontsize=8,
            zorder=6,
        )
        trails.append(trail)
        bodies.append(body)
        dots.append(dot)
        labels.append(label)
        halos.append(halo)
    (seg,) = ax.plot([], [], color="#c00000", lw=1.5, zorder=7)
    star = ax.scatter([], [], marker="*", s=80, c="#9b0000", zorder=8)
    return {
        "empty": False,
        "trails": trails,
        "bodies": bodies,
        "dots": dots,
        "labels": labels,
        "halos": halos,
        "seg": seg,
        "star": star,
    }


def apply_column(clip: Clip, artists: dict, idx: int, *, full: bool) -> None:
    if artists.get("empty") or clip.status != "ok" or not clip.uavs:
        return
    idx = int(max(0, min(idx, len(clip.times) - 1)))
    end = len(clip.times) if full else idx + 1
    for i, uav in enumerate(clip.uavs):
        trail = clip.tracks[uav][:end]
        finite = np.isfinite(trail).all(axis=1)
        artists["trails"][i].set_data(trail[finite, 0], trail[finite, 1])
        p = clip.tracks[uav][idx]
        visible = bool(np.isfinite(p).all())
        artists["bodies"][i].set_visible(visible)
        artists["labels"][i].set_visible(visible)
        if artists["halos"][i] is not None:
            artists["halos"][i].set_visible(visible)
        if visible:
            center = (float(p[0]), float(p[1]))
            artists["bodies"][i].set_center(center)
            if artists["halos"][i] is not None:
                artists["halos"][i].set_center(center)
            artists["dots"][i].set_data([p[0]], [p[1]])
            artists["labels"][i].xy = center
        else:
            artists["dots"][i].set_data([], [])
    dist, _pair, pa, pb = _frame_pair(clip, idx)
    if dist is not None and dist <= clip.collide_m + 1e-9:
        artists["seg"].set_data([pa[0], pb[0]], [pa[1], pb[1]])
    else:
        artists["seg"].set_data([], [])
    if full and clip.closest_pair and clip.closest_pair.get("mid_xy"):
        artists["star"].set_offsets([clip.closest_pair["mid_xy"]])
    else:
        artists["star"].set_offsets(np.empty((0, 2)))


def fmt_m(value: float | None) -> str:
    if value is None:
        return "—"
    return f"{float(value):.3f} m"


def fmt_pairs(pairs: list[dict] | None) -> str:
    if pairs is None:
        return "—"
    if not pairs:
        return "无"
    bits = []
    for row in pairs:
        dist = row.get("min_distance_m")
        dist_s = "—" if dist is None else f"{float(dist):.3f} m"
        bits.append(f"{row['pair'][0]}-{row['pair'][1]} {dist_s}")
    return "，".join(bits)


def panel_caption(clip: Clip) -> str:
    if clip.status != "ok":
        return "sphere 档缺失\noutcome —\n到达 —\n最小机间 —\n碰撞对 —"
    arrived = clip.result.get("arrived")
    n = clip.result.get("n_drones")
    arrived_s = "—" if arrived is None else f"{arrived}/{n}"
    return (
        f"{clip.result.get('outcome')}\n"
        f"到达 {arrived_s}\n"
        f"最小机间 {fmt_m(clip.min_inter_m)}\n"
        f"碰撞对 {fmt_pairs(clip.collision_pairs)}"
    )


def hud_text(clips: list[Clip], idx_of, *, t_now: float | None = None) -> str:
    title = clips[0].title
    if t_now is None:
        t_now = max(float(clip.times[idx_of(clip)]) for clip in clips if len(clip.times))
    lines = [f"{title}  seed {clips[0].seed}    t = {float(t_now):.2f} s    碰撞线 {COLLIDE_M:.2f} m"]
    for clip in clips:
        if clip.status != "ok":
            lines.append(f"{clip.arm['short']}    开关缺失    到达 —    机间 —    碰撞对 —")
            continue
        idx = idx_of(clip)
        dist, pair, _pa, _pb = _frame_pair(clip, idx)
        ended = t_now > float(clip.times[-1]) + 1e-9
        tag = "（已结束，定格）" if ended else ""
        pair_s = "—" if pair is None else f"{pair[0]}-{pair[1]}"
        arrived = clip.result.get("arrived")
        n = clip.result.get("n_drones") or len(clip.uavs)
        lines.append(
            f"{clip.arm['short']}    当前机间 {fmt_m(dist)} ({pair_s})    "
            f"全程最小 {fmt_m(clip.min_inter_m)}    到达 {arrived}/{n}    "
            f"{clip.result.get('outcome')}    碰撞对 {fmt_pairs(clip.collision_pairs)}{tag}"
        )
    return "\n".join(lines)


def _new_figure(clips: list[Clip], *, dpi: float):
    prepare_font()
    n = len(clips)
    fig = plt.figure(figsize=(5.2 * n, 6.4), dpi=dpi)
    fig.patch.set_facecolor("white")
    grid = fig.add_gridspec(1, n, wspace=0.22, left=0.06, right=0.985, top=0.74, bottom=0.10)
    xlim, ylim = view_limits(clips)
    columns = []
    for i, clip in enumerate(clips):
        ax = fig.add_subplot(grid[0, i])
        draw_obstacles(ax, clip.obstacles)
        draw_goals(ax, clip)
        style_xy(ax, xlim, ylim)
        ax.set_title(clip.arm["title"], fontsize=11, pad=6)
        columns.append((ax, column_artists(ax, clip)))
    handles = [
        Line2D([0], [0], marker="o", color="#4d6b32", markerfacecolor="#9cbb7a", ls="none", ms=7, label="树"),
        Line2D([0], [0], color="#444444", lw=1.3, label="轨迹"),
        Line2D([0], [0], marker="x", color="#444444", ls="none", ms=6, label="终点"),
        Line2D([0], [0], marker="*", color="#9b0000", ls="none", ms=8, label="全程最近"),
        Line2D([0], [0], color="#c00000", lw=1.4, label=f"碰撞线 {COLLIDE_M:.2f} m"),
        Line2D([0], [0], color="#444444", lw=0.8, ls="--", label=f"欧氏球半径 {SPHERE_RADIUS_M:.2f} m"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=3, fontsize=8, frameon=False, bbox_to_anchor=(0.5, 0.01))
    hud = fig.text(0.04, 0.985, "", va="top", ha="left", fontsize=9, linespacing=1.35)
    return fig, columns, hud


def render_still(clips: list[Clip], png: Path) -> Path:
    fig, columns, hud = _new_figure(clips, dpi=120)
    try:
        def idx_of(clip: Clip) -> int:
            return closest_index(clip)

        for clip, (_ax, artists) in zip(clips, columns, strict=True):
            apply_column(clip, artists, idx_of(clip), full=True)
            _ax.text(
                0.02,
                0.02,
                panel_caption(clip),
                transform=_ax.transAxes,
                va="bottom",
                ha="left",
                fontsize=8,
                color="#1b1b1b",
                bbox=dict(boxstyle="round,pad=0.25", facecolor="white", edgecolor="#cccccc", alpha=0.9),
                zorder=9,
            )
        hud.set_text(hud_text(clips, idx_of))
        png.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(png)
    finally:
        plt.close(fig)
    return png


def render_overview(groups: list[list[Clip]], png: Path) -> Path:
    prepare_font()
    rows = len(groups)
    fig, axes = plt.subplots(rows, 2, figsize=(11.2, 5.1 * rows), dpi=110, squeeze=False)
    fig.patch.set_facecolor("white")
    fig.suptitle("E-B4c 欧氏碰撞球开关  俯视总览", fontsize=14, y=0.995)
    try:
        for r, clips in enumerate(groups):
            xlim, ylim = view_limits(clips)
            for c, clip in enumerate(clips):
                ax = axes[r, c]
                draw_obstacles(ax, clip.obstacles)
                draw_goals(ax, clip)
                style_xy(ax, xlim, ylim)
                ax.set_title(f"{clip.title}  seed {clip.seed}  {clip.arm['short']}", fontsize=10)
                artists = column_artists(ax, clip)
                apply_column(clip, artists, closest_index(clip), full=True)
                ax.text(
                    0.02,
                    0.02,
                    panel_caption(clip),
                    transform=ax.transAxes,
                    va="bottom",
                    ha="left",
                    fontsize=7.5,
                    color="#1b1b1b",
                    bbox=dict(boxstyle="round,pad=0.25", facecolor="white", edgecolor="#cccccc", alpha=0.9),
                    zorder=9,
                )
        fig.tight_layout(rect=(0, 0, 1, 0.97))
        png.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(png)
    finally:
        plt.close(fig)
    return png


def playback_times(duration_s: float, fps: float, max_seconds: float) -> np.ndarray:
    if duration_s <= 0:
        return np.asarray([0.0, 0.0])
    play = min(max(float(duration_s), 1.0 / fps), float(max_seconds))
    n = max(2, int(round(play * fps)))
    return np.linspace(0.0, float(duration_s), n)


def render_animation(clips: list[Clip], gif: Path, mp4: Path | None, *, fps: float, max_seconds: float) -> tuple[Path, Path | None, float]:
    finite = [clip for clip in clips if clip.status == "ok" and len(clip.times)]
    if not finite:
        raise RuntimeError("没有可以回放的轨迹")
    t0 = min(float(clip.times[0]) for clip in finite)
    t1 = max(float(clip.times[-1]) for clip in finite)
    grid = playback_times(max(t1 - t0, 0.0), fps, max_seconds)
    play_s = len(grid) / float(fps)
    if play_s > float(max_seconds) + 1e-6:
        raise RuntimeError(f"成片时长 {play_s:.3f}s 超过上限 {max_seconds}s")
    gif.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = animation.FFMpegWriter.isAvailable()
    pillow = animation.PillowWriter.isAvailable()
    if not ffmpeg and not pillow:
        raise RuntimeError("写 gif 需要 ffmpeg 或 pillow")

    def draw_into(writer, dest: Path) -> None:
        fig, columns, hud = _new_figure(clips, dpi=100)
        try:
            with writer.saving(fig, str(dest), dpi=100):
                for t in grid:
                    t_abs = t0 + float(t)

                    def idx_of(clip: Clip, t_abs=t_abs) -> int:
                        if clip.status != "ok" or len(clip.times) == 0:
                            return 0
                        return index_at(clip.times, t_abs)

                    for clip, (_ax, artists) in zip(clips, columns, strict=True):
                        apply_column(clip, artists, idx_of(clip), full=False)
                    hud.set_text(hud_text(clips, idx_of, t_now=t_abs))
                    writer.grab_frame()
        except Exception:
            dest.unlink(missing_ok=True)
            raise
        finally:
            plt.close(fig)

    if ffmpeg and mp4 is not None:
        mp4.parent.mkdir(parents=True, exist_ok=True)
        writer = animation.FFMpegWriter(
            fps=float(fps),
            codec="libx264",
            extra_args=["-pix_fmt", "yuv420p", "-preset", "veryfast", "-crf", "23"],
        )
        draw_into(writer, mp4)
        mp4_to_gif(mp4, gif, fps=fps)
        return gif, mp4, play_s
    writer = animation.PillowWriter(fps=float(fps))
    draw_into(writer, gif)
    return gif, None, play_s


def mp4_to_gif(mp4: Path, gif: Path, *, fps: float, width: int = 960) -> Path:
    gif.parent.mkdir(parents=True, exist_ok=True)
    vf = (
        f"fps={fps:.4g},scale={int(width)}:-1:flags=lanczos,"
        "split[s0][s1];[s0]palettegen=max_colors=128:stats_mode=full[p];"
        "[s1][p]paletteuse=dither=bayer:bayer_scale=5"
    )
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-i", str(mp4), "-vf", vf, "-an", str(gif)]
    subprocess.run(cmd, check=True)
    return gif


def rel_to(path: Path, out: Path) -> str:
    try:
        return path.resolve().relative_to(out.resolve()).as_posix()
    except ValueError:
        return str(path)


def case_stem(case: dict) -> str:
    return f"{case['scene']}-seed{int(case['seed'])}"


def run_record(clip: Clip, files: dict[str, str | None]) -> dict:
    ok = clip.status == "ok"
    arrived = clip.result.get("arrived") if ok else None
    n = clip.result.get("n_drones") if ok else None
    enabled = None if not ok else bool(clip.arm.get("peer_sphere_enabled"))
    return {
        "id": f"{clip.scene}-{clip.arm['id']}-seed{clip.seed}",
        "scene": clip.scene,
        "scene_path": clip.scene_path,
        "title": clip.title,
        "arm": "sphere" if clip.arm["id"] == "sphere" else clip.arm["id"],
        "arm_title": clip.arm["title"],
        "seed": int(clip.seed),
        "status": clip.status,
        "missing": clip.missing,
        "backend_enabled": True if ok else None,
        "peer_sphere_enabled": enabled,
        "peer_sphere_radius_m": SPHERE_RADIUS_M if enabled else None,
        "peer_sphere_weight": SPHERE_WEIGHT if enabled else None,
        "outcome": clip.result.get("outcome") if ok else None,
        "reason": clip.result.get("reason") if ok else None,
        "arrived": None if arrived is None else int(arrived),
        "n_drones": None if n is None else int(n),
        "min_inter_drone_distance_m": clip.min_inter_m,
        "collision_pairs": clip.collision_pairs,
        "closest_pair": clip.closest_pair if ok else None,
        "sim_duration_s": clip.result.get("sim_duration_s") if ok else None,
        "wall_time_s": clip.wall_time_s,
        "log_dir": None if clip.log_dir is None else rel_to(clip.log_dir, clip.log_dir.parents[1]),
        "files": files,
    }


def write_summary(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def planned_arms(available: bool) -> list[dict]:
    base = dict(BASE_ARM)
    base["status"] = "ok"
    sphere = dict(SPHERE_ARM)
    if available:
        sphere["status"] = "ok"
    else:
        sphere["status"] = "missing"
        sphere["title"] = "sphere（开关缺失）"
        sphere["short"] = "sphere 缺失"
    return [base, sphere]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="成片、日志和 summary.json 的目录")
    parser.add_argument("--cases", default=None, help="逗号分隔的 场景:seed。缺省是票面三粒")
    parser.add_argument("--partial", action="store_true", help=f"冒烟：仿真时长收到 {PARTIAL_DURATION_S:g} s；缺开关时跳过 sphere")
    parser.add_argument("--duration", type=float, default=None, help="覆盖仿真秒数。与 --partial 一起用时仍允许缺开关")
    parser.add_argument("--reuse-logs", action="store_true", help="复用 --out/logs 里已有的 ticks.jsonl 和 result.json")
    parser.add_argument("--fps", type=float, default=10)
    parser.add_argument("--max-seconds", type=float, default=15, help="成片播放时长上限")
    args = parser.parse_args(argv)

    if not (0 < args.max_seconds <= 15):
        print("成片时长上限必须在 (0, 15] 秒", file=sys.stderr)
        return 2
    if args.fps <= 0:
        print("fps 必须为正", file=sys.stderr)
        return 2
    if args.duration is not None and not (args.duration > 0):
        print("duration 必须为正", file=sys.stderr)
        return 2

    isolate_sphere_env()
    available = peer_sphere_available()
    partial = bool(args.partial)
    if args.duration is not None:
        duration: float | None = float(args.duration)
    elif partial:
        duration = float(PARTIAL_DURATION_S)
    else:
        duration = None
    if not available and not partial:
        print(FULL_SWITCH_ERROR, file=sys.stderr)
        return 2
    if not animation.FFMpegWriter.isAvailable() and not animation.PillowWriter.isAvailable():
        print("写 gif 需要 ffmpeg 或 pillow", file=sys.stderr)
        return 2
    if available:
        install_peer_delivery()

    cases = resolve_cases(args.cases)
    arms = planned_arms(available)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    print(
        f"out={out}  partial={partial}  duration={duration if duration is not None else '场景卡'}  "
        f"peer_sphere_available={available}",
        flush=True,
    )

    groups: list[list[Clip]] = []
    for case in cases:
        clips = [ensure_clip(case, arm, duration, out, args.reuse_logs) for arm in arms]
        groups.append(clips)

    compare = out / "compare"
    artifacts: list[dict] = []
    runs: list[dict] = []
    ffmpeg = animation.FFMpegWriter.isAvailable()
    for case, clips in zip(cases, groups, strict=True):
        stem = case_stem(case)
        png = compare / f"{stem}.png"
        gif = compare / f"{stem}.gif"
        mp4 = compare / f"{stem}.mp4"
        print(f"静图 {png.name}", flush=True)
        render_still(clips, png)
        print(f"动图 {gif.name}", flush=True)
        _gif, written_mp4, play_s = render_animation(
            clips,
            gif,
            mp4 if ffmpeg else None,
            fps=float(args.fps),
            max_seconds=float(args.max_seconds),
        )
        files = {
            "gif": rel_to(gif, out),
            "png": rel_to(png, out),
            "mp4": None if written_mp4 is None else rel_to(written_mp4, out),
            "overview": "compare/overview.png",
        }
        artifacts.append({"path": files["png"], "kind": "png", "scene": case["scene"], "seed": case["seed"], "duration_s": None, "note": f"{case['title']} 俯视静图"})
        artifacts.append({"path": files["gif"], "kind": "gif", "scene": case["scene"], "seed": case["seed"], "duration_s": play_s, "note": f"{case['title']} 俯视对比"})
        if files["mp4"]:
            artifacts.append({"path": files["mp4"], "kind": "mp4", "scene": case["scene"], "seed": case["seed"], "duration_s": play_s, "note": f"{case['title']} 俯视对比"})
        for clip in clips:
            runs.append(run_record(clip, dict(files)))

    overview = compare / "overview.png"
    print(f"总览 {overview.name}", flush=True)
    render_overview(groups, overview)
    artifacts.append({"path": "compare/overview.png", "kind": "png", "scene": None, "seed": None, "duration_s": None, "note": "全部用例俯视总览"})

    summary = {
        "ticket": "V3",
        "title": "E-B4c 欧氏碰撞球开关对比回放",
        "report_only": True,
        "frozen": False,
        "viz_only": True,
        "partial": partial,
        "duration_s": duration,
        "fps": float(args.fps),
        "max_seconds": float(args.max_seconds),
        "peer_sphere_available": available,
        "peer_sphere_radius_m": SPHERE_RADIUS_M,
        "peer_sphere_weight": SPHERE_WEIGHT,
        "collision_m": COLLIDE_M,
        "body_radius_m": BODY_RADIUS_M,
        "arms": list(ARM_ORDER),
        "cases": [{"scene": case["scene"], "scene_path": case["path"], "seed": int(case["seed"]), "title": case["title"]} for case in cases],
        "runs": runs,
        "artifacts": artifacts,
        "note": None if available else MISSING_SWITCH,
    }
    summary_path = out / "summary.json"
    write_summary(summary_path, summary)
    print(f"写入 {summary_path}", flush=True)
    for run in runs:
        gap = run["min_inter_drone_distance_m"]
        gap_s = "—" if gap is None else f"{gap:.3f}"
        print(
            f"数字 {run['id']}: status={run['status']} {run['outcome']}  "
            f"到达 {run['arrived']}/{run['n_drones']}  最小机间 {gap_s} m  "
            f"碰撞对 {fmt_pairs(run['collision_pairs'])}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
