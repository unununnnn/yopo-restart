#!/usr/bin/env python3
"""票 V2：穿林跟踪净空门前后对比回放。只出图，不改算法。

三档都在进程里盖参数，不改场景卡，也不改 ``track_clearance`` / ``empty_brake`` 的默认值：

- ``base``：``track_clearance=False``，全关，不写 ``policy_params``
- ``geom``：``track_clearance=True``，``track_clearance_mode="geom"``，余量用缺省 0.08 m
- ``geom+empty_brake``：geom 再加 ``empty_brake=True``

默认场景（``--scenes`` / ``--seed`` 可覆盖），都是 seed 0：

- ``scenes/ego-density-014-solo.json``
- ``scenes/ego-density-014-swap3.json``

每帧写入 ``frames.jsonl``：位置、速度、``plan_code``、最小表面净空、最小机间距离。
俯视是 x-y 上的障碍圆柱和轨迹，侧视是 t-z 高度和地面净空。并排对比。
ffmpeg 可用时写 ``.mp4`` 再转 ``.gif``；否则用 pillow 写 ``.gif``。总览 ``.png`` 始终写。

``--partial`` 把仿真收到 2 s，用来冒烟。已有 ``logs/*/ticks.jsonl`` 时加 ``--reuse-logs`` 只重渲。

用法::

    .venv/bin/python scripts/v2-clearance-replay.py --partial --out /tmp/v2-smoke
"""

from __future__ import annotations

import argparse
import json
import math
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
from matplotlib.patches import Circle

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

DEFAULT_OUT = Path("/workspace/yopo-ops/viz-v2")
PARTIAL_DURATION_S = 2.0
TRACK_MARGIN_M = 0.08
BODY_RADIUS_M = 0.35

SCENES = {
    "solo": "scenes/ego-density-014-solo.json",
    "swap3": "scenes/ego-density-014-swap3.json",
}
SCENE_TITLE = {
    "solo": "密度 0.14 单机穿林",
    "swap3": "密度 0.14 三机对向穿林",
}
SCENE_ALIASES = {
    "solo": "solo",
    "d014-solo": "solo",
    "ego-density-014-solo": "solo",
    "scenes/ego-density-014-solo.json": "solo",
    "swap3": "swap3",
    "d014-swap3": "swap3",
    "ego-density-014-swap3": "swap3",
    "scenes/ego-density-014-swap3.json": "swap3",
}
ARMS = (
    {
        "id": "base",
        "title": "base（全关）",
        "short": "base",
        "track_clearance": False,
        "track_clearance_mode": None,
        "empty_brake": False,
    },
    {
        "id": "geom",
        "title": "geom",
        "short": "geom",
        "track_clearance": True,
        "track_clearance_mode": "geom",
        "empty_brake": False,
    },
    {
        "id": "geom+empty_brake",
        "title": "geom + empty_brake",
        "short": "geom+刹停",
        "track_clearance": True,
        "track_clearance_mode": "geom",
        "empty_brake": True,
    },
)
COLORS = ("#d62728", "#1f77b4", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf", "#e377c2", "#8c564b")
_FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)
_FONT_READY = False


def arm_by_id(arm_id: str) -> dict:
    for arm in ARMS:
        if arm["id"] == arm_id:
            return arm
    raise KeyError(arm_id)


def upstream_block(arm: dict) -> dict | None:
    """全关返回 None，调用方不写 policy_params。余量只在几何门打开时写上，数值等于代码缺省。"""
    if not arm["track_clearance"] and not arm["empty_brake"]:
        return None
    block: dict = {}
    if arm["track_clearance"]:
        block["track_clearance"] = True
        block["track_clearance_mode"] = arm["track_clearance_mode"]
        block["track_margin_m"] = TRACK_MARGIN_M
    if arm["empty_brake"]:
        block["empty_brake"] = True
    return block


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


def resolve_scenes(text: str | None) -> list[tuple[str, Path]]:
    raw = ["solo", "swap3"] if not text else [part.strip() for part in text.split(",") if part.strip()]
    if not raw:
        raise SystemExit("`--scenes` 是空的")
    chosen: list[tuple[str, Path]] = []
    seen: set[str] = set()
    for key in raw:
        alias = SCENE_ALIASES.get(key) or SCENE_ALIASES.get(key.removesuffix(".json"))
        if alias is not None:
            scene_id = alias
            path = REPO / SCENES[alias]
        else:
            path = Path(key)
            if not path.is_file():
                path = REPO / key
            if not path.is_file():
                known = ", ".join(sorted(SCENE_ALIASES))
                raise SystemExit(f"未知场景 {key!r}。已知别名：{known}，或给一个存在的场景卡路径")
            scene_id = path.stem
            path = path.resolve()
        if scene_id in seen:
            continue
        seen.add(scene_id)
        if not path.is_file():
            raise SystemExit(f"场景卡不存在：{path}")
        chosen.append((scene_id, path))
    return chosen


def build_card(scene_path: Path, arm: dict):
    """进程内盖开关。不写场景文件。base 走 load_card，哈希与文件一致。"""
    from yopo_l0.scene import SceneCard, load_card, parse_geometry

    base = load_card(scene_path)
    block = upstream_block(arm)
    if block is None:
        return base
    card = json.loads(json.dumps(base.card))
    params = dict(card.get("policy_params") or {})
    upstream = dict(params.get("upstream") or {})
    upstream.update(block)
    params["upstream"] = upstream
    card["policy_params"] = params
    return SceneCard(base.path, card, parse_geometry(card["geometry"], card.get("ground_z")))


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


def _as_vec3(value, what: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{what} 必须是长度 3，收到 {value!r}")
    return [float(value[0]), float(value[1]), float(value[2])]


def frames_from_ticks(ticks: list[dict]) -> list[dict]:
    """按物理拍对齐。一拍一行：每机位置、速度、plan_code、表面净空，以及这一拍的最小机间距离。"""
    by_k: dict[int, dict] = {}
    for row in ticks:
        try:
            k = int(row["k"])
            t = float(row["t"])
            uav = str(row["uav"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"无法解析 tick：{row!r}") from exc
        slot = by_k.setdefault(k, {"k": k, "t": t, "drones": {}})
        slot["t"] = t
        code = row.get("plan_code")
        slot["drones"][uav] = {
            "uav": uav,
            "p": _as_vec3(row.get("p"), "p"),
            "v": _as_vec3(row.get("v"), "v"),
            "plan_code": "" if code is None else str(code),
            "min_surface_clearance_m": float(row["min_obstacle_m"]),
        }
    frames: list[dict] = []
    for k in sorted(by_k):
        slot = by_k[k]
        drones = [slot["drones"][uid] for uid in sorted(slot["drones"])]
        if not drones:
            continue
        surface = min(item["min_surface_clearance_m"] for item in drones)
        inter = _min_inter(drones)
        frames.append(
            {
                "k": int(k),
                "t": float(slot["t"]),
                "min_surface_clearance_m": float(surface),
                "min_inter_drone_distance_m": inter,
                "drones": drones,
            }
        )
    if not frames:
        raise ValueError("ticks 里没有对得齐的帧")
    return frames


def _min_inter(drones: list[dict]) -> float | None:
    if len(drones) < 2:
        return None
    best: float | None = None
    for i, left in enumerate(drones):
        pa = np.asarray(left["p"], dtype=float)
        for right in drones[i + 1 :]:
            dist = float(np.linalg.norm(pa - np.asarray(right["p"], dtype=float)))
            best = dist if best is None else min(best, dist)
    return best


def metrics_from_frames(frames: list[dict]) -> dict:
    if not frames:
        raise ValueError("没有帧")
    surface = min(float(frame["min_surface_clearance_m"]) for frame in frames)
    zs = [float(drone["p"][2]) for frame in frames for drone in frame["drones"]]
    inter_values = [float(frame["min_inter_drone_distance_m"]) for frame in frames if frame["min_inter_drone_distance_m"] is not None]
    return {
        "min_obstacle_clearance_m": float(surface),
        "min_ground_z_m": float(min(zs)),
        "min_inter_drone_distance_m": None if not inter_values else float(min(inter_values)),
    }


def write_frames(path: Path, frames: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for frame in frames:
            handle.write(json.dumps(frame, ensure_ascii=False, separators=(",", ":")) + "\n")


@dataclass
class Clip:
    scene_id: str
    scene_path: str
    arm: dict
    seed: int
    times: np.ndarray
    uavs: list[str]
    tracks: dict[str, np.ndarray]
    surface: np.ndarray
    inter: np.ndarray
    ground_z: float
    body_radius: float
    obstacles: tuple
    goals: dict[str, list[float]]
    result: dict
    metrics: dict
    log_dir: Path
    wall_time_s: float | None


def clip_from_frames(scene_id: str, scene_path: Path, arm: dict, seed: int, frames: list[dict], result: dict, log_dir: Path, wall_time_s: float | None) -> Clip:
    from yopo_l0.scene import load_card

    card = load_card(scene_path)
    uavs = []
    for frame in frames:
        for drone in frame["drones"]:
            if drone["uav"] not in uavs:
                uavs.append(drone["uav"])
    n = len(frames)
    tracks = {uav: np.full((n, 3), np.nan) for uav in uavs}
    times = np.empty(n)
    surface = np.empty(n)
    inter = np.full(n, np.nan)
    for i, frame in enumerate(frames):
        times[i] = float(frame["t"])
        surface[i] = float(frame["min_surface_clearance_m"])
        if frame["min_inter_drone_distance_m"] is not None:
            inter[i] = float(frame["min_inter_drone_distance_m"])
        for drone in frame["drones"]:
            tracks[drone["uav"]][i] = drone["p"]
    ground = card.card.get("ground_z")
    ground_z = 0.0 if ground is None else float(ground)
    radius = float(card.card["criteria"]["body_radius_m"])
    goals = {str(drone["id"]): [float(x) for x in drone["goal_enu"]] for drone in card.card["drones"]}
    return Clip(
        scene_id=scene_id,
        scene_path=str(scene_path.relative_to(REPO)) if scene_path.is_relative_to(REPO) else str(scene_path),
        arm=arm,
        seed=int(seed),
        times=times,
        uavs=uavs,
        tracks=tracks,
        surface=surface,
        inter=inter,
        ground_z=ground_z,
        body_radius=radius,
        obstacles=card.obstacles,
        goals=goals,
        result=result,
        metrics=metrics_from_frames(frames),
        log_dir=log_dir,
        wall_time_s=wall_time_s,
    )


def run_sim(scene_path: Path, arm: dict, seed: int, duration: float | None, log_dir: Path) -> tuple[dict, list[dict], float]:
    from yopo_l0.runner import RunConfig, run, write

    card = build_card(scene_path, arm)
    log_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(seed), shadow=True, duration_s=duration))
    wall = time.perf_counter() - t0
    write(out, log_dir)
    return out.result, out.ticks, wall


def ensure_clip(scene_id: str, scene_path: Path, arm: dict, seed: int, duration: float | None, out: Path, reuse: bool) -> Clip:
    log_dir = out / "logs" / f"{scene_id}-{arm['id']}-seed{seed}"
    ticks_path = log_dir / "ticks.jsonl"
    result_path = log_dir / "result.json"
    wall: float | None = None
    if reuse and ticks_path.is_file() and result_path.is_file():
        print(f"复用日志 {log_dir}", flush=True)
        ticks = load_ticks(ticks_path)
        result = json.loads(result_path.read_text(encoding="utf-8"))
    else:
        dur = "全程" if duration is None else f"{duration:g} s"
        print(f"开跑 {scene_id} {arm['id']} seed={seed} 时长={dur}", flush=True)
        result, ticks, wall = run_sim(scene_path, arm, seed, duration, log_dir)
        print(
            f"  完成 {scene_id} {arm['id']}  outcome={result.get('outcome')}  "
            f"到达 {result.get('arrived')}/{result.get('n_drones')}  "
            f"仿真 {result.get('sim_duration_s')} s  墙钟 {wall:.1f} s",
            flush=True,
        )
    frames = frames_from_ticks(ticks)
    write_frames(log_dir / "frames.jsonl", frames)
    return clip_from_frames(scene_id, scene_path, arm, seed, frames, result, log_dir, wall)


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
    if not xs:
        return (-8.0, 8.0), (-8.0, 8.0)
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    reach = 3.0
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
    pad = 1.4
    return _span(min(xs) - pad, max(xs) + pad, 8.0), _span(min(ys) - pad, max(ys) + pad, 8.0)


def z_limits(clips: list[Clip]) -> tuple[float, float]:
    vals = [clip.ground_z for clip in clips]
    for clip in clips:
        for track in clip.tracks.values():
            finite = track[np.isfinite(track).all(axis=1)]
            if len(finite):
                vals.append(float(finite[:, 2].min()))
                vals.append(float(finite[:, 2].max()))
                vals.append(float(finite[:, 2].min()) - clip.ground_z)
                vals.append(float(finite[:, 2].max()) - clip.ground_z)
    lo, hi = min(vals), max(vals)
    return lo - 0.35, hi + 0.45


def draw_obstacles(ax, obstacles) -> None:
    from yopo_l0.scene import Box, Cylinder
    from matplotlib.patches import Rectangle

    tree_kw = dict(facecolor="#9cbb7a", edgecolor="#4d6b32", lw=0.4, alpha=0.9, zorder=1)
    box_kw = dict(facecolor="#c8c8c8", edgecolor="#666666", lw=0.4, zorder=0)
    for ob in obstacles:
        if isinstance(ob, Cylinder):
            cx, cy = ob.section_xy(1.5)
            ax.add_patch(Circle((cx, cy), ob.radius, **tree_kw))
        elif isinstance(ob, Box):
            x, y, _z = ob.lo
            dx = float(ob.hi[0] - ob.lo[0])
            dy = float(ob.hi[1] - ob.lo[1])
            ax.add_patch(Rectangle((x, y), dx, dy, **box_kw))


def draw_goals(ax, clip: Clip) -> None:
    for i, uav in enumerate(clip.uavs):
        goal = clip.goals.get(uav)
        if goal is None:
            continue
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


def style_tz(ax, xlim, ylim, ground_z: float) -> None:
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_xlabel("t / s")
    ax.set_ylabel("m")
    ax.grid(True, alpha=0.28)
    ax.set_facecolor("#fbfbf7")
    ax.axhline(ground_z, color="#8a8a8a", lw=0.8, zorder=0)


def add_legend(fig) -> None:
    handles = [
        Line2D([0], [0], marker="o", color="#4d6b32", markerfacecolor="#9cbb7a", ls="none", ms=7, label="树"),
        Line2D([0], [0], color="#444444", lw=1.3, label="轨迹"),
        Line2D([0], [0], marker="x", color="#444444", ls="none", ms=6, label="终点"),
        Line2D([0], [0], color="#333333", lw=1.3, label="高度 z"),
        Line2D([0], [0], color="#333333", lw=1.2, ls="--", label="地面净空"),
        Line2D([0], [0], color="#8a8a8a", lw=0.8, label="地面"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=6, fontsize=8, frameon=False, bbox_to_anchor=(0.5, 0.755))


def column_artists(ax_xy, ax_tz, clip: Clip) -> dict:
    trails = []
    bodies = []
    dots = []
    labels = []
    z_lines = []
    clear_lines = []
    for i, uav in enumerate(clip.uavs):
        color = COLORS[i % len(COLORS)]
        (trail,) = ax_xy.plot([], [], color=color, lw=1.4, zorder=3, solid_capstyle="round")
        body = Circle((0, 0), clip.body_radius, fill=True, facecolor=color, alpha=0.22, ec=color, lw=1.1, zorder=4)
        ax_xy.add_patch(body)
        (dot,) = ax_xy.plot([], [], "o", color=color, ms=3.2, zorder=5)
        label = ax_xy.annotate(
            uav[3:] if uav.startswith("uav") else uav,
            xy=(0, 0),
            xytext=(4, 4),
            textcoords="offset points",
            color=color,
            fontsize=8,
            zorder=6,
        )
        (z_line,) = ax_tz.plot([], [], color=color, lw=1.35, zorder=3)
        (clear_line,) = ax_tz.plot([], [], color=color, lw=1.15, ls="--", zorder=4)
        trails.append(trail)
        bodies.append(body)
        dots.append(dot)
        labels.append(label)
        z_lines.append(z_line)
        clear_lines.append(clear_line)
    (vline,) = ax_tz.plot([], [], color="#555555", lw=0.7, ls=":", zorder=2)
    star = ax_xy.scatter([], [], marker="*", s=70, c="#9b0000", zorder=7)
    if abs(clip.ground_z) < 1e-9:
        ax_tz.text(0.98, 0.06, "地面 z=0，实线与虚线重合", transform=ax_tz.transAxes, ha="right", va="bottom", fontsize=7, color="#555555")
    return {
        "trails": trails,
        "bodies": bodies,
        "dots": dots,
        "labels": labels,
        "z_lines": z_lines,
        "clear_lines": clear_lines,
        "fills": [],
        "tz": ax_tz,
        "colors": [COLORS[i % len(COLORS)] for i in range(len(clip.uavs))],
        "vline": vline,
        "star": star,
    }


def apply_column(clip: Clip, artists: dict, idx: int, *, full: bool) -> None:
    idx = int(max(0, min(idx, len(clip.times) - 1)))
    end = len(clip.times) if full else idx + 1
    tight = int(np.nanargmin(clip.surface))
    for i, uav in enumerate(clip.uavs):
        trail = clip.tracks[uav][:end]
        finite = np.isfinite(trail).all(axis=1)
        artists["trails"][i].set_data(trail[finite, 0], trail[finite, 1])
        times = clip.times[:end]
        zz = clip.tracks[uav][:end, 2]
        good = np.isfinite(zz)
        artists["z_lines"][i].set_data(times[good], zz[good])
        artists["clear_lines"][i].set_data(times[good], zz[good] - clip.ground_z)
        p = clip.tracks[uav][idx]
        visible = bool(np.isfinite(p).all())
        artists["bodies"][i].set_visible(visible)
        artists["labels"][i].set_visible(visible)
        if visible:
            artists["bodies"][i].set_center((float(p[0]), float(p[1])))
            artists["dots"][i].set_data([p[0]], [p[1]])
            artists["labels"][i].xy = (float(p[0]), float(p[1]))
            close = bool(np.isfinite(clip.surface[idx]) and clip.surface[idx] <= clip.body_radius + 1e-9)
            artists["bodies"][i].set_edgecolor("#9b0000" if close else COLORS[i % len(COLORS)])
        else:
            artists["dots"][i].set_data([], [])
    for coll in artists["fills"]:
        coll.remove()
    artists["fills"].clear()
    for i, uav in enumerate(clip.uavs):
        times = clip.times[:end]
        zz = clip.tracks[uav][:end, 2]
        good = np.isfinite(zz) & np.isfinite(times)
        if not good.any():
            continue
        artists["fills"].append(
            artists["tz"].fill_between(
                times[good],
                np.full(int(good.sum()), clip.ground_z),
                zz[good],
                color=artists["colors"][i],
                alpha=0.16,
                zorder=1,
                linewidth=0,
            )
        )
    t = float(clip.times[idx])
    z0, z1 = artists["vline"].axes.get_ylim()
    artists["vline"].set_data([t, t], [z0, z1])
    if full and 0 <= tight < len(clip.times):
        # 总览把全程最紧的那一拍标出来。多机时取这一拍里表面净空最小的那架。
        mark = _tight_xy(clip, tight)
        artists["star"].set_offsets([mark] if mark is not None else np.empty((0, 2)))
    else:
        artists["star"].set_offsets(np.empty((0, 2)))


def _tight_xy(clip: Clip, idx: int) -> list[float] | None:
    best = None
    best_d = None
    for uav in clip.uavs:
        p = clip.tracks[uav][idx]
        if not np.isfinite(p).all():
            continue
        # 帧上的表面净空是全机最小，星标放在该拍最低的那架，贴地和贴树都看得见。
        z = float(p[2])
        if best is None or z < best_d:
            best = [float(p[0]), float(p[1])]
            best_d = z
    return best


def fmt_inter(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return "—"
    return f"{value:.3f} m"


def _running(clip: Clip, idx: int) -> tuple[float, float, float | None]:
    idx = int(max(0, min(idx, len(clip.times) - 1)))
    surf = clip.surface[: idx + 1]
    d_obs = float(np.nanmin(surf)) if np.isfinite(surf).any() else float("nan")
    zs = [
        float(np.nanmin(clip.tracks[uav][: idx + 1, 2]))
        for uav in clip.uavs
        if np.isfinite(clip.tracks[uav][: idx + 1, 2]).any()
    ]
    zmin = min(zs) if zs else float("nan")
    inter_vals = clip.inter[: idx + 1]
    inter = float(np.nanmin(inter_vals)) if np.isfinite(inter_vals).any() else None
    return d_obs, zmin, inter


def hud_text(clips: list[Clip], idx_of, *, t_now: float | None = None) -> str:
    scene = SCENE_TITLE.get(clips[0].scene_id, clips[0].scene_id)
    t = t_now if t_now is not None else max(float(clip.times[idx_of(clip)]) for clip in clips)
    lines = [f"{scene}  seed {clips[0].seed}    t = {t:.2f} s"]
    for clip in clips:
        idx = idx_of(clip)
        d_obs, zmin, inter = _running(clip, idx)
        arrived = clip.result.get("arrived")
        n = clip.result.get("n_drones") or len(clip.uavs)
        ended = t_now is not None and t_now > float(clip.times[-1]) + 1e-9
        tag = "（已结束，定格）" if ended else ""
        lines.append(
            f"{clip.arm['short']}    表面净空 {d_obs:.3f} m    最低高度 {zmin:.3f} m    "
            f"机间 {fmt_inter(inter)}    到达 {arrived}/{n}    {clip.result.get('outcome')}{tag}"
        )
    return "\n".join(lines)


def overview_text(clips: list[Clip]) -> str:
    scene = SCENE_TITLE.get(clips[0].scene_id, clips[0].scene_id)
    lines = [f"{scene}  seed {clips[0].seed}    关键帧总览（星标是表面净空最小的一拍，数字是整段）"]
    for clip in clips:
        d_obs, zmin, inter = _running(clip, len(clip.times) - 1)
        tight = float(clip.times[int(np.nanargmin(clip.surface))])
        arrived = clip.result.get("arrived")
        n = clip.result.get("n_drones") or len(clip.uavs)
        lines.append(
            f"{clip.arm['short']}    t* = {tight:.2f} s    表面净空 {d_obs:.3f} m    "
            f"最低高度 {zmin:.3f} m    机间 {fmt_inter(inter)}    到达 {arrived}/{n}    {clip.result.get('outcome')}"
        )
    return "\n".join(lines)


def hud_color(clips: list[Clip], idx_of) -> str:
    for clip in clips:
        idx = idx_of(clip)
        surf = clip.surface[: idx + 1]
        if np.isfinite(surf).any() and float(np.nanmin(surf)) <= clip.body_radius + 1e-9:
            return "#9b0000"
        inter_vals = clip.inter[: idx + 1]
        if np.isfinite(inter_vals).any() and float(np.nanmin(inter_vals)) <= 2.0 * clip.body_radius + 1e-9:
            return "#9b0000"
    return "#1b1b1b"


def playback_times(duration_s: float, fps: float, max_seconds: float) -> np.ndarray:
    if duration_s <= 0:
        return np.asarray([0.0])
    play = min(max(float(duration_s), 1.0 / fps), float(max_seconds))
    n = max(2, int(round(play * fps)))
    return np.linspace(0.0, float(duration_s), n)


def index_at(times: np.ndarray, t: float) -> int:
    if t >= float(times[-1]):
        return int(len(times) - 1)
    if t <= float(times[0]):
        return 0
    return int(np.searchsorted(times, t, side="right") - 1)


def _new_figure(clips: list[Clip], *, dpi: float):
    prepare_font()
    n = len(clips)
    fig = plt.figure(figsize=(4.8 * n, 8.8), dpi=dpi)
    fig.patch.set_facecolor("white")
    # 顶上留给四行读数，图例夹在读数和坐标轴之间，避免叠在一起。
    grid = fig.add_gridspec(2, n, height_ratios=[1.35, 1.0], hspace=0.36, wspace=0.28, left=0.05, right=0.985, top=0.72, bottom=0.07)
    xlim, ylim = view_limits(clips)
    t_max = max(float(clip.times[-1]) for clip in clips)
    zlim = z_limits(clips)
    columns = []
    for i, clip in enumerate(clips):
        ax_xy = fig.add_subplot(grid[0, i])
        ax_tz = fig.add_subplot(grid[1, i])
        draw_obstacles(ax_xy, clip.obstacles)
        draw_goals(ax_xy, clip)
        style_xy(ax_xy, xlim, ylim)
        style_tz(ax_tz, (0.0, t_max * 1.02 if t_max > 0 else 1.0), zlim, clip.ground_z)
        ax_xy.set_title(clip.arm["title"], fontsize=11, pad=6)
        columns.append((ax_xy, ax_tz, column_artists(ax_xy, ax_tz, clip)))
    add_legend(fig)
    hud = fig.text(0.05, 0.985, "", va="top", ha="left", fontsize=9, linespacing=1.4)
    return fig, columns, hud


def render_overview(clips: list[Clip], png: Path) -> Path:
    fig, columns, hud = _new_figure(clips, dpi=120)
    try:
        def idx_of(clip: Clip) -> int:
            return int(np.nanargmin(clip.surface))

        for clip, (_xy, _tz, artists) in zip(clips, columns, strict=True):
            apply_column(clip, artists, idx_of(clip), full=True)
        hud.set_text(overview_text(clips))
        hud.set_color(hud_color(clips, lambda clip: len(clip.times) - 1))
        png.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(png)
    finally:
        plt.close(fig)
    return png


def _grab(fig, columns, hud, clips: list[Clip], t_abs: float, writer) -> None:
    def idx_of(clip: Clip) -> int:
        return index_at(clip.times, t_abs)

    for clip, (_xy, _tz, artists) in zip(clips, columns, strict=True):
        apply_column(clip, artists, idx_of(clip), full=False)
    hud.set_text(hud_text(clips, idx_of, t_now=t_abs))
    hud.set_color(hud_color(clips, idx_of))
    writer.grab_frame()


def render_animation(clips: list[Clip], gif: Path, mp4: Path | None, *, fps: float, max_seconds: float) -> tuple[Path, Path | None, float]:
    t0 = min(float(clip.times[0]) for clip in clips)
    t1 = max(float(clip.times[-1]) for clip in clips)
    grid = playback_times(max(t1 - t0, 0.0), fps, max_seconds)
    play_s = len(grid) / float(fps)
    if play_s > float(max_seconds) + 1e-6:
        raise RuntimeError(f"成片时长 {play_s:.3f}s 超过上限 {max_seconds}s")
    gif.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = animation.FFMpegWriter.isAvailable()
    pillow = animation.PillowWriter.isAvailable()
    if not ffmpeg and not pillow:
        raise RuntimeError("写 gif 需要 ffmpeg 或 pillow")

    def draw_into(writer, dest: Path):
        fig, columns, hud = _new_figure(clips, dpi=100)
        try:
            with writer.saving(fig, str(dest), dpi=100):
                for t in grid:
                    _grab(fig, columns, hud, clips, t0 + float(t), writer)
        except Exception:
            dest.unlink(missing_ok=True)
            raise
        finally:
            plt.close(fig)

    written_mp4: Path | None = None
    if ffmpeg and mp4 is not None:
        mp4.parent.mkdir(parents=True, exist_ok=True)
        writer = animation.FFMpegWriter(
            fps=float(fps),
            codec="libx264",
            extra_args=["-pix_fmt", "yuv420p", "-preset", "veryfast", "-crf", "23"],
        )
        draw_into(writer, mp4)
        written_mp4 = mp4
        mp4_to_gif(mp4, gif, fps=fps)
        return gif, written_mp4, play_s
    writer = animation.PillowWriter(fps=float(fps))
    draw_into(writer, gif)
    return gif, None, play_s


def mp4_to_gif(mp4: Path, gif: Path, *, fps: float, width: int = 1080) -> Path:
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


def run_record(clip: Clip, files: dict[str, str | None]) -> dict:
    arm = clip.arm
    arrived = clip.result.get("arrived")
    return {
        "id": f"{clip.scene_id}-{arm['id']}-seed{clip.seed}",
        "scene": clip.scene_id,
        "scene_path": clip.scene_path,
        "arm": arm["id"],
        "title": arm["title"],
        "seed": clip.seed,
        "track_clearance": bool(arm["track_clearance"]),
        "track_clearance_mode": arm["track_clearance_mode"],
        "empty_brake": bool(arm["empty_brake"]),
        "track_margin_m": TRACK_MARGIN_M if arm["track_clearance"] else None,
        "upstream_deviation": bool(arm["track_clearance"] or arm["empty_brake"]),
        "outcome": clip.result.get("outcome"),
        "reason": clip.result.get("reason"),
        "arrived": None if arrived is None else int(arrived),
        "n_drones": int(clip.result.get("n_drones") or len(clip.uavs)),
        "min_obstacle_clearance_m": clip.metrics["min_obstacle_clearance_m"],
        "min_ground_z_m": clip.metrics["min_ground_z_m"],
        "min_inter_drone_distance_m": clip.metrics["min_inter_drone_distance_m"],
        "sim_duration_s": clip.result.get("sim_duration_s"),
        "wall_time_s": clip.wall_time_s,
        "log_dir": rel_to(clip.log_dir, clip.log_dir.parents[1]),
        "files": files,
    }


def write_summary(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="成片、日志和 summary.json 的目录")
    parser.add_argument("--scenes", default=None, help="逗号分隔。默认 solo,swap3。可写别名或场景卡路径")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--partial", action="store_true", help=f"冒烟：仿真时长收到 {PARTIAL_DURATION_S:g} s")
    parser.add_argument("--duration", type=float, default=None, help="覆盖仿真秒数。给出时优先于 --partial 和场景卡")
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
    if not animation.FFMpegWriter.isAvailable() and not animation.PillowWriter.isAvailable():
        print("写 gif 需要 ffmpeg 或 pillow", file=sys.stderr)
        return 2

    if args.duration is not None:
        duration: float | None = float(args.duration)
        partial = False
    elif args.partial:
        duration = float(PARTIAL_DURATION_S)
        partial = True
    else:
        duration = None
        partial = False

    scenes = resolve_scenes(args.scenes)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    print(f"out={out}  seed={args.seed}  partial={partial}  duration={duration if duration is not None else '场景卡'}", flush=True)

    grouped: list[tuple[str, list[Clip]]] = []
    for scene_id, scene_path in scenes:
        clips = [ensure_clip(scene_id, scene_path, arm, args.seed, duration, out, args.reuse_logs) for arm in ARMS]
        grouped.append((scene_id, clips))

    compare = out / "compare"
    artifacts: list[dict] = []
    runs: list[dict] = []
    for scene_id, clips in grouped:
        stem = f"{scene_id}-seed{clips[0].seed}"
        png = compare / f"{stem}.png"
        gif = compare / f"{stem}.gif"
        mp4 = compare / f"{stem}.mp4"
        print(f"总览 {png.name}", flush=True)
        render_overview(clips, png)
        print(f"动图 {gif.name}", flush=True)
        _gif, written_mp4, play_s = render_animation(
            clips,
            gif,
            mp4 if animation.FFMpegWriter.isAvailable() else None,
            fps=float(args.fps),
            max_seconds=float(args.max_seconds),
        )
        files = {
            "gif": rel_to(gif, out),
            "png": rel_to(png, out),
            "mp4": None if written_mp4 is None else rel_to(written_mp4, out),
        }
        note = SCENE_TITLE.get(scene_id, scene_id)
        artifacts.append({"path": files["png"], "kind": "png", "scene": scene_id, "duration_s": None, "note": f"{note} 关键帧总览"})
        artifacts.append({"path": files["gif"], "kind": "gif", "scene": scene_id, "duration_s": play_s, "note": f"{note} 并排对比"})
        if files["mp4"]:
            artifacts.append({"path": files["mp4"], "kind": "mp4", "scene": scene_id, "duration_s": play_s, "note": f"{note} 并排对比"})
        for clip in clips:
            run_files = dict(files)
            run_files["frames"] = rel_to(clip.log_dir / "frames.jsonl", out)
            runs.append(run_record(clip, run_files))

    summary = {
        "ticket": "V2",
        "title": "穿林跟踪净空门前后对比回放",
        "report_only": True,
        "frozen": False,
        "viz_only": True,
        "seed": int(args.seed),
        "partial": bool(args.partial),
        "duration_s": duration,
        "fps": float(args.fps),
        "max_seconds": float(args.max_seconds),
        "body_radius_m": BODY_RADIUS_M,
        "track_margin_m": TRACK_MARGIN_M,
        "clearance_definition": "min_obstacle_clearance_m 是机体中心到最近表面（含地面）的距离，与 ticks.min_obstacle_m 同一口径。碰撞线是 body_radius 0.35 m。min_ground_z_m 是全程最低的机体中心高度。",
        "arms": [arm["id"] for arm in ARMS],
        "scenes": [scene_id for scene_id, _path in scenes],
        "runs": runs,
        "artifacts": artifacts,
    }
    summary_path = out / "summary.json"
    write_summary(summary_path, summary)
    print(f"写入 {summary_path}", flush=True)
    for run in runs:
        inter = run["min_inter_drone_distance_m"]
        inter_s = "—" if inter is None else f"{inter:.3f}"
        print(
            f"数字 {run['id']}: {run['outcome']}  到达 {run['arrived']}/{run['n_drones']}  "
            f"表面净空 {run['min_obstacle_clearance_m']:.3f} m  最低高度 {run['min_ground_z_m']:.3f} m  机间 {inter_s}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
