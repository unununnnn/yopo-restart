#!/usr/bin/env python3
"""票 V1：E-B4 机间碰撞修复前后的俯视回放。只出图，不改算法。

四段默认素材（均为 seed 0）：

- 八机对向 ``swap8``、密度 0.14 三机对向 ``d014-swap3`` 各两段；
- 全关臂越线后再飞 ``--post-s`` 秒（默认 1 s），让对穿看得见；
- 修复臂与诊断对照相同：E-A1 门槛 1.0 m，加 ``--proto-brake``。

仿真调用 E-B4 的 ``scripts/e-b4-peer-diag.py``（``--diag-root``，合入后也可用本仓库），
不改 core、运行器或场景卡。成片播放不超过 ``--max-seconds``（默认 15 s）。

用法::

    nice -n 15 <venv>/bin/python scripts/viz-e-b4-replay.py \\
        --diag-root /path/to/e-b4-worktree \\
        --out /workspace/yopo-ops/viz

已有 ``logs/*/ticks.jsonl`` 时加 ``--reuse-logs`` 只重渲。默认单进程。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation, font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle

REPO = Path(__file__).resolve().parents[1]
COLLIDE_M = 0.70
BODY_R = 0.35
DEFAULT_OUT = Path("/workspace/yopo-ops/viz")
DEFAULT_DIAG = Path("/workspace/wt/yopo-restart/yopo-restart-bot-e-b4-peer-collision-diag")
SCENES = {
    "swap8": "scenes/swap-8-open.json",
    "d014-swap3": "scenes/ego-density-014-swap3.json",
}
SCENE_TITLE = {
    "swap8": "八机对向",
    "d014-swap3": "密度 0.14 三机对向",
}
COLORS = (
    "#d62728",
    "#1f77b4",
    "#2ca02c",
    "#9467bd",
    "#ff7f0e",
    "#17becf",
    "#e377c2",
    "#8c564b",
)
_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)
_FONT_READY = False


def clip_specs(seed: int, post_s: float) -> list[dict]:
    extra = {"upstream": {"inter_drone_threshold_m": 1.0}}
    return [
        {
            "id": "swap8-off",
            "scene": "swap8",
            "arm": "off",
            "proto_brake": False,
            "extra": None,
            "post_s": float(post_s),
            "seed": int(seed),
            "title": "全关（越线后再飞 1 s）" if post_s == 1.0 else f"全关（越线后再飞 {post_s:g} s）",
            "short": "全关",
        },
        {
            "id": "swap8-fix",
            "scene": "swap8",
            "arm": "a1",
            "proto_brake": True,
            "extra": extra,
            "post_s": float(post_s),
            "seed": int(seed),
            "title": "修复（E-A1 1.0 m + 刹停）",
            "short": "修复",
        },
        {
            "id": "d014-swap3-off",
            "scene": "d014-swap3",
            "arm": "off",
            "proto_brake": False,
            "extra": None,
            "post_s": float(post_s),
            "seed": int(seed),
            "title": "全关（越线后再飞 1 s）" if post_s == 1.0 else f"全关（越线后再飞 {post_s:g} s）",
            "short": "全关",
        },
        {
            "id": "d014-swap3-fix",
            "scene": "d014-swap3",
            "arm": "a1",
            "proto_brake": True,
            "extra": extra,
            "post_s": float(post_s),
            "seed": int(seed),
            "title": "修复（E-A1 1.0 m + 刹停）",
            "short": "修复",
        },
    ]


def default_diag_root() -> Path:
    if (REPO / "scripts" / "e-b4-peer-diag.py").is_file():
        return REPO
    if (DEFAULT_DIAG / "scripts" / "e-b4-peer-diag.py").is_file():
        return DEFAULT_DIAG
    return REPO


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


def load_diag(diag_root: Path):
    path = diag_root / "scripts" / "e-b4-peer-diag.py"
    if not path.is_file():
        raise SystemExit(f"找不到 E-B4 诊断脚本：{path}（请传 --diag-root）")
    # 不用 --code-root 这个名字，避免诊断脚本在 import 时误读本脚本的 argv。
    root = str(diag_root.resolve())
    if sys.path[:1] != [root]:
        sys.path.insert(0, root)
    spec = importlib.util.spec_from_file_location("e_b4_peer_diag", path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"无法加载 {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load_ticks(path: Path) -> list[dict]:
    rows: list[dict] = []
    text = path.read_text()
    for lineno, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError as e:
            raise ValueError(f"{path}:{lineno} 不是 JSON：{e}") from e
    if not rows:
        raise ValueError(f"日志是空的：{path}")
    return rows


def align(ticks: list[dict]):
    by_k: dict[int, dict] = {}
    for row in ticks:
        k = int(row["k"])
        uav = str(row["uav"])
        t = float(row["t"])
        p = row["p"]
        if not isinstance(p, (list, tuple)) or len(p) != 3:
            raise ValueError(f"tick 的 p 必须是长度 3：{row!r}")
        slot = by_k.setdefault(k, {"t": t, "uavs": {}})
        slot["t"] = t
        slot["uavs"][uav] = np.asarray(p, dtype=float)
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


def _min_pair(uavs: list[str], tracks: dict[str, np.ndarray], i: int):
    best = None
    pair = None
    pa = pb = None
    for a in range(len(uavs)):
        pa_ = tracks[uavs[a]][i]
        if not np.isfinite(pa_).all():
            continue
        for b in range(a + 1, len(uavs)):
            pb_ = tracks[uavs[b]][i]
            if not np.isfinite(pb_).all():
                continue
            d = float(np.linalg.norm(pa_ - pb_))
            if best is None or d < best:
                best, pair, pa, pb = d, (uavs[a], uavs[b]), pa_.copy(), pb_.copy()
    return best, pair, pa, pb


def analyze_tracks(times: np.ndarray, uavs: list[str], tracks: dict[str, np.ndarray]) -> dict:
    n = len(times)
    live = np.full(n, np.nan)
    first_cross = None
    closest = None
    for i in range(n):
        d, pair, pa, pb = _min_pair(uavs, tracks, i)
        if d is None:
            continue
        live[i] = d
        mid = ((pa + pb) / 2.0).tolist()
        rec = {
            "t_s": float(times[i]),
            "index": i,
            "pair": list(pair),
            "dist_m": d,
            "mid_xy": [float(mid[0]), float(mid[1])],
            "pa_xy": [float(pa[0]), float(pa[1])],
            "pb_xy": [float(pb[0]), float(pb[1])],
        }
        if closest is None or d < closest["dist_m"]:
            closest = rec
        if first_cross is None and d <= COLLIDE_M + 1e-12:
            first_cross = rec
    return {"live_min_m": live, "first_cross": first_cross, "closest": closest}


def drone_no(uav: str) -> str:
    return uav[3:] if uav.startswith("uav") else uav


@dataclass
class ClipData:
    spec: dict
    times: np.ndarray
    uavs: list[str]
    tracks: dict[str, np.ndarray]
    analysis: dict
    meta: dict
    card: object
    log_dir: Path
    xlim: tuple[float, float]
    ylim: tuple[float, float]


def bounds_for(tracks: dict[str, np.ndarray], card, pad: float = 2.5) -> tuple[tuple[float, float], tuple[float, float]]:
    pts = [tr[np.isfinite(tr).all(axis=1)] for tr in tracks.values()]
    pts = [p for p in pts if len(p)]
    xs, ys = [], []
    if pts:
        arr = np.concatenate(pts, axis=0)
        xs.extend([float(arr[:, 0].min()), float(arr[:, 0].max())])
        ys.extend([float(arr[:, 1].min()), float(arr[:, 1].max())])
    if card is not None:
        for d in card.card["drones"]:
            for key in ("start_enu", "goal_enu"):
                p = d[key]
                xs.append(float(p[0]))
                ys.append(float(p[1]))
        from yopo_l0.scene import Box, Cylinder

        for ob in card.obstacles:
            if isinstance(ob, Box):
                xs.extend([float(ob.lo[0]), float(ob.hi[0])])
                ys.extend([float(ob.lo[1]), float(ob.hi[1])])
            elif isinstance(ob, Cylinder):
                cx, cy = ob.section_xy(1.5)
                r = float(ob.radius)
                xs.extend([cx - r, cx + r])
                ys.extend([cy - r, cy + r])
    if not xs:
        return (-10.0, 10.0), (-10.0, 10.0)
    return (min(xs) - pad, max(xs) + pad), (min(ys) - pad, max(ys) + pad)


def union_limits(items: list[ClipData]) -> tuple[tuple[float, float], tuple[float, float]]:
    xmin = min(c.xlim[0] for c in items)
    xmax = max(c.xlim[1] for c in items)
    ymin = min(c.ylim[0] for c in items)
    ymax = max(c.ylim[1] for c in items)
    return (xmin, xmax), (ymin, ymax)


def draw_obstacles(ax, obstacles) -> None:
    from yopo_l0.scene import Box, Cylinder

    tree_kw = dict(facecolor="#9cbb7a", edgecolor="#4d6b32", lw=0.4, alpha=0.85, zorder=1)
    box_kw = dict(facecolor="#c8c8c8", edgecolor="#666666", lw=0.5, zorder=0)
    saw_tree = False
    for ob in obstacles:
        if isinstance(ob, Box):
            x, y, _z = ob.lo
            dx, dy, _dz = (np.asarray(ob.hi) - np.asarray(ob.lo)).tolist()
            ax.add_patch(Rectangle((x, y), dx, dy, **box_kw))
        elif isinstance(ob, Cylinder):
            cx, cy = ob.section_xy(1.5)
            ax.add_patch(Circle((cx, cy), ob.radius, **tree_kw))
            saw_tree = True
    if saw_tree:
        ax._eb4_saw_tree = True  # type: ignore[attr-defined]


def style_ax(ax, xlim, ylim) -> None:
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.grid(True, alpha=0.28)
    ax.set_facecolor("#fbfbf7")


def draw_goals(ax, card, uavs: list[str]) -> None:
    if card is None:
        return
    by_id = {d["id"]: d for d in card.card["drones"]}
    for i, uav in enumerate(uavs):
        d = by_id.get(uav)
        if d is None:
            continue
        color = COLORS[i % len(COLORS)]
        g = d["goal_enu"]
        ax.plot([g[0]], [g[1]], marker="x", color=color, ms=6, mew=1.2, alpha=0.7, zorder=2)


def frame_artists(ax, clip: ClipData):
    artists = {}
    radius = BODY_R
    if clip.card is not None:
        radius = float(clip.card.card["criteria"]["body_radius_m"])
    trails, bodies, labels, dots = [], [], [], []
    for i, uav in enumerate(clip.uavs):
        color = COLORS[i % len(COLORS)]
        (tr,) = ax.plot([], [], color=color, lw=1.35, zorder=3, solid_capstyle="round")
        body = Circle((0, 0), radius, fill=True, facecolor=color, alpha=0.22, ec=color, lw=1.15, zorder=4)
        ax.add_patch(body)
        (dot,) = ax.plot([], [], "o", color=color, ms=3.5, zorder=5)
        lab = ax.annotate(
            drone_no(uav),
            xy=(0, 0),
            xytext=(5, 5),
            textcoords="offset points",
            color=color,
            fontsize=8,
            fontweight="bold",
            zorder=6,
        )
        trails.append(tr)
        bodies.append(body)
        labels.append(lab)
        dots.append(dot)
    (seg,) = ax.plot([], [], color="#c00000", lw=1.6, zorder=7)
    cross_mark = ax.scatter([], [], marker="x", s=70, c="#c00000", linewidths=1.8, zorder=8)
    close_mark = ax.scatter([], [], marker="*", s=90, c="#9b0000", linewidths=0.6, zorder=8)
    cross_txt = ax.text(0, 0, "", color="#c00000", fontsize=7.5, ha="left", va="bottom", zorder=9)
    close_txt = ax.text(0, 0, "", color="#9b0000", fontsize=7.5, ha="left", va="top", zorder=9)
    artists.update(
        trails=trails,
        bodies=bodies,
        labels=labels,
        dots=dots,
        seg=seg,
        cross_mark=cross_mark,
        close_mark=close_mark,
        cross_txt=cross_txt,
        close_txt=close_txt,
        radius=radius,
    )
    return artists


def apply_frame(clip: ClipData, artists: dict, idx: int) -> None:
    idx = int(max(0, min(idx, len(clip.times) - 1)))
    for i, uav in enumerate(clip.uavs):
        trail = clip.tracks[uav][: idx + 1]
        finite = np.isfinite(trail).all(axis=1)
        artists["trails"][i].set_data(trail[finite, 0], trail[finite, 1])
        p = clip.tracks[uav][idx]
        visible = bool(np.isfinite(p).all())
        artists["bodies"][i].set_visible(visible)
        artists["labels"][i].set_visible(visible)
        if visible:
            artists["bodies"][i].set_center((float(p[0]), float(p[1])))
            artists["dots"][i].set_data([p[0]], [p[1]])
            artists["labels"][i].xy = (float(p[0]), float(p[1]))
        else:
            artists["dots"][i].set_data([], [])
    live = clip.analysis["live_min_m"]
    d_now = float(live[idx]) if np.isfinite(live[idx]) else float("nan")
    # 当前最近一对：红线标出；越线后一直画。
    _, pair, pa, pb = _min_pair(clip.uavs, clip.tracks, idx)
    if pair is not None and d_now <= COLLIDE_M + 0.15:
        artists["seg"].set_data([pa[0], pb[0]], [pa[1], pb[1]])
    else:
        artists["seg"].set_data([], [])
    t = float(clip.times[idx])
    fc = clip.analysis["first_cross"]
    if fc is not None and t + 1e-12 >= fc["t_s"]:
        artists["cross_mark"].set_offsets([fc["mid_xy"]])
        artists["cross_txt"].set_position((fc["mid_xy"][0] + 0.25, fc["mid_xy"][1] + 0.25))
        artists["cross_txt"].set_text(f"越线 {fc['dist_m']:.3f} m\n{drone_no(fc['pair'][0])}-{drone_no(fc['pair'][1])}")
        artists["cross_txt"].set_visible(True)
    else:
        artists["cross_mark"].set_offsets(np.empty((0, 2)))
        artists["cross_txt"].set_visible(False)
    cl = clip.analysis["closest"]
    # 最近点与越线点几乎重合时只标越线，避免两行叠在一起。
    show_close = (
        cl is not None
        and t + 1e-12 >= cl["t_s"]
        and (fc is None or abs(cl["dist_m"] - fc["dist_m"]) > 0.02 or abs(cl["t_s"] - fc["t_s"]) > 0.05)
    )
    if show_close:
        artists["close_mark"].set_offsets([cl["mid_xy"]])
        artists["close_txt"].set_position((cl["mid_xy"][0] + 0.25, cl["mid_xy"][1] - 0.25))
        artists["close_txt"].set_text(f"最近 {cl['dist_m']:.3f} m\n{drone_no(cl['pair'][0])}-{drone_no(cl['pair'][1])}")
        artists["close_txt"].set_visible(True)
    else:
        artists["close_mark"].set_offsets(np.empty((0, 2)))
        artists["close_txt"].set_visible(False)


def hud_text(clip: ClipData, idx: int) -> str:
    idx = int(max(0, min(idx, len(clip.times) - 1)))
    live = clip.analysis["live_min_m"]
    d_now = float(live[idx]) if np.isfinite(live[idx]) else float("nan")
    d_run = float(np.nanmin(live[: idx + 1])) if np.isfinite(live[: idx + 1]).any() else float("nan")
    t = float(clip.times[idx])
    arrived = clip.meta.get("arrived")
    n = clip.meta.get("n_drones") or len(clip.uavs)
    scene = SCENE_TITLE[clip.spec["scene"]]
    line1 = f"{scene}  {clip.spec['title']}  seed {clip.spec['seed']}"
    line2 = f"t = {t:.2f} s    当前机间 {d_now:.3f} m    全程最小 {d_run:.3f} m    到达 {arrived}/{n}"
    return line1 + "\n" + line2


def hud_color(clip: ClipData, idx: int) -> str:
    idx = int(max(0, min(idx, len(clip.times) - 1)))
    live = clip.analysis["live_min_m"]
    d_run = float(np.nanmin(live[: idx + 1])) if np.isfinite(live[: idx + 1]).any() else float("nan")
    if np.isfinite(d_run) and d_run <= COLLIDE_M:
        return "#9b0000"
    return "#1b1b1b"


def playback_times(duration_s: float, fps: float, max_seconds: float) -> np.ndarray:
    if duration_s <= 0:
        return np.asarray([0.0])
    play = min(max(float(duration_s), 1.0 / fps), float(max_seconds))
    n = max(1, int(round(play * fps)))
    return np.linspace(0.0, float(duration_s), n)


def index_at(times: np.ndarray, t: float) -> int:
    if t >= float(times[-1]):
        return int(len(times) - 1)
    if t <= float(times[0]):
        return 0
    return int(np.searchsorted(times, t, side="right") - 1)


def add_legend(ax, has_tree: bool) -> None:
    handles = [
        Line2D([0], [0], color="#444444", lw=1.3, label="轨迹"),
        Line2D([0], [0], marker="x", color="#444444", ls="none", ms=6, label="终点"),
        Line2D([0], [0], marker="x", color="#c00000", ls="none", ms=7, label="首次越线（0.70 m）"),
        Line2D([0], [0], marker="*", color="#9b0000", ls="none", ms=8, label="全程最近"),
    ]
    if has_tree:
        handles.insert(0, Line2D([0], [0], marker="o", color="#4d6b32", markerfacecolor="#9cbb7a", ls="none", ms=8, label="树"))
    ax.legend(handles=handles, loc="upper right", fontsize=7, framealpha=0.88)


def render_top_video(clip: ClipData, out_mp4: Path, *, fps: float, max_seconds: float) -> dict:
    prepare_font()
    duration = float(clip.times[-1] - clip.times[0]) if len(clip.times) > 1 else 0.0
    t_grid = playback_times(duration if duration > 0 else float(clip.times[-1]), fps, max_seconds)
    play_s = len(t_grid) / float(fps)
    if play_s > float(max_seconds) + 1e-6:
        raise RuntimeError(f"成片时长 {play_s:.3f}s 超过上限 {max_seconds}s")
    sped = duration > play_s + 1.0 / float(fps)
    fig = plt.figure(figsize=(8.0, 8.8), dpi=100)
    fig.patch.set_facecolor("white")
    ax = fig.add_axes([0.09, 0.06, 0.86, 0.80])
    hud = fig.text(0.05, 0.975, "", va="top", ha="left", fontsize=10.5, linespacing=1.35)
    try:
        draw_obstacles(ax, clip.card.obstacles if clip.card is not None else ())
        draw_goals(ax, clip.card, clip.uavs)
        style_ax(ax, clip.xlim, clip.ylim)
        add_legend(ax, bool(getattr(ax, "_eb4_saw_tree", False)))
        arts = frame_artists(ax, clip)
        out_mp4.parent.mkdir(parents=True, exist_ok=True)
        writer = animation.FFMpegWriter(fps=float(fps), codec="libx264", extra_args=["-pix_fmt", "yuv420p", "-preset", "veryfast", "-crf", "20"])
        with writer.saving(fig, str(out_mp4), dpi=100):
            for t in t_grid:
                idx = index_at(clip.times, float(t) + float(clip.times[0]))
                apply_frame(clip, arts, idx)
                text = hud_text(clip, idx)
                if sped:
                    text += f"    （{duration / play_s:.2f}×）"
                hud.set_text(text)
                hud.set_color(hud_color(clip, idx))
                writer.grab_frame()
    except Exception:
        out_mp4.unlink(missing_ok=True)
        raise
    finally:
        plt.close(fig)
    return {"path": out_mp4, "n_frames": len(t_grid), "duration_s": play_s, "sped_up": sped}


def render_compare_video(left: ClipData, right: ClipData, out_mp4: Path, *, fps: float, max_seconds: float) -> dict:
    prepare_font()
    t0 = min(float(left.times[0]), float(right.times[0]))
    t1 = max(float(left.times[-1]), float(right.times[-1]))
    duration = t1 - t0
    t_grid = playback_times(duration, fps, max_seconds)
    play_s = len(t_grid) / float(fps)
    if play_s > float(max_seconds) + 1e-6:
        raise RuntimeError(f"成片时长 {play_s:.3f}s 超过上限 {max_seconds}s")
    sped = duration > play_s + 1.0 / float(fps)
    xlim, ylim = union_limits([left, right])
    fig = plt.figure(figsize=(16.0, 8.6), dpi=100)
    fig.patch.set_facecolor("white")
    ax_l = fig.add_axes([0.045, 0.06, 0.445, 0.78])
    ax_r = fig.add_axes([0.545, 0.06, 0.445, 0.78])
    hud = fig.text(0.02, 0.975, "", va="top", ha="left", fontsize=11, linespacing=1.4)
    try:
        xlims, arts = [], []
        for ax, clip in ((ax_l, left), (ax_r, right)):
            draw_obstacles(ax, clip.card.obstacles if clip.card is not None else ())
            draw_goals(ax, clip.card, clip.uavs)
            style_ax(ax, xlim, ylim)
            ax.set_title(clip.spec["title"], fontsize=12, pad=8)
            add_legend(ax, bool(getattr(ax, "_eb4_saw_tree", False)))
            arts.append(frame_artists(ax, clip))
            xlims.append(clip)
        scene = SCENE_TITLE[left.spec["scene"]]
        out_mp4.parent.mkdir(parents=True, exist_ok=True)
        writer = animation.FFMpegWriter(fps=float(fps), codec="libx264", extra_args=["-pix_fmt", "yuv420p", "-preset", "veryfast", "-crf", "20"])
        with writer.saving(fig, str(out_mp4), dpi=100):
            for t in t_grid:
                lines = [f"{scene}  修复前后对照  seed {left.spec['seed']}    t = {t0 + float(t):.2f} s"]
                if sped:
                    lines[0] += f"    （{duration / play_s:.2f}×）"
                color = "#1b1b1b"
                for clip, art in zip((left, right), arts, strict=True):
                    idx = index_at(clip.times, t0 + float(t))
                    apply_frame(clip, art, idx)
                    live = clip.analysis["live_min_m"]
                    d_now = float(live[idx]) if np.isfinite(live[idx]) else float("nan")
                    d_run = float(np.nanmin(live[: idx + 1])) if np.isfinite(live[: idx + 1]).any() else float("nan")
                    arrived = clip.meta.get("arrived")
                    n = clip.meta.get("n_drones") or len(clip.uavs)
                    ended = t0 + float(t) > float(clip.times[-1]) + 1e-9
                    tag = "（已结束，定格）" if ended else ""
                    lines.append(
                        f"{clip.spec['short']}    当前机间 {d_now:.3f} m    全程最小 {d_run:.3f} m    到达 {arrived}/{n}{tag}"
                    )
                    if np.isfinite(d_run) and d_run <= COLLIDE_M:
                        color = "#9b0000"
                hud.set_text("\n".join(lines))
                hud.set_color(color)
                writer.grab_frame()
    except Exception:
        out_mp4.unlink(missing_ok=True)
        raise
    finally:
        plt.close(fig)
    return {"path": out_mp4, "n_frames": len(t_grid), "duration_s": play_s, "sped_up": sped}


def _still_panel(ax, clip: ClipData, idx: int, xlim, ylim, title: str) -> None:
    draw_obstacles(ax, clip.card.obstacles if clip.card is not None else ())
    draw_goals(ax, clip.card, clip.uavs)
    style_ax(ax, xlim, ylim)
    ax.set_title(title, fontsize=11, pad=6)
    arts = frame_artists(ax, clip)
    apply_frame(clip, arts, idx)
    add_legend(ax, bool(getattr(ax, "_eb4_saw_tree", False)))
    live = clip.analysis["live_min_m"]
    d_now = float(live[idx]) if np.isfinite(live[idx]) else float("nan")
    d_min = float(np.nanmin(live)) if np.isfinite(live).any() else float("nan")
    arrived = clip.meta.get("arrived")
    n = clip.meta.get("n_drones") or len(clip.uavs)
    t = float(clip.times[idx])
    ax.text(
        0.02,
        0.02,
        f"t = {t:.2f} s\n当前机间 {d_now:.3f} m\n全程最小 {d_min:.3f} m\n到达 {arrived}/{n}",
        transform=ax.transAxes,
        va="bottom",
        ha="left",
        fontsize=8,
        color="#9b0000" if np.isfinite(d_min) and d_min <= COLLIDE_M else "#1b1b1b",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white", edgecolor="#cccccc", alpha=0.9),
        zorder=10,
    )


def render_compare_png(left: ClipData, right: ClipData, out_png: Path) -> Path:
    prepare_font()
    xlim, ylim = union_limits([left, right])
    # 左列全关：越线时刻 + 全程最近（对穿）；右列修复：同一时刻 + 结束。
    t_cross = left.analysis["first_cross"]["t_s"] if left.analysis["first_cross"] else float(left.times[-1])
    t_close = left.analysis["closest"]["t_s"] if left.analysis["closest"] else float(left.times[-1])
    rows = [
        ("首次越线附近", t_cross),
        ("全关最近 / 修复同刻", t_close),
        ("各自结束", None),
    ]
    fig, axes = plt.subplots(len(rows), 2, figsize=(11.5, 5.2 * len(rows)), dpi=120)
    fig.patch.set_facecolor("white")
    scene = SCENE_TITLE[left.spec["scene"]]
    fig.suptitle(f"{scene}  seed {left.spec['seed']}  修复前后俯视对照", fontsize=14, y=0.995)
    try:
        for r, (label, t) in enumerate(rows):
            for c, clip in enumerate((left, right)):
                if t is None:
                    idx = len(clip.times) - 1
                else:
                    idx = index_at(clip.times, float(t))
                title = f"{clip.spec['short']} · {label}"
                _still_panel(axes[r, c], clip, idx, xlim, ylim, title)
        fig.tight_layout(rect=(0, 0, 1, 0.98))
        out_png.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_png, bbox_inches="tight")
    finally:
        plt.close(fig)
    return out_png


def render_all_png(clips: list[ClipData], out_png: Path) -> Path:
    prepare_font()
    # 两行两列：上行八机，下行三机；左全关、右修复，定格在全关最近或结束。
    by_id = {c.spec["id"]: c for c in clips}
    rows = [
        ("swap8-off", "swap8-fix"),
        ("d014-swap3-off", "d014-swap3-fix"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 12.2), dpi=130)
    fig.patch.set_facecolor("white")
    fig.suptitle("E-B4 机间碰撞修复前后  俯视对照  seed 0", fontsize=15, y=0.995)
    try:
        for r, (lid, rid) in enumerate(rows):
            left, right = by_id[lid], by_id[rid]
            xlim, ylim = union_limits([left, right])
            t_close = left.analysis["closest"]["t_s"] if left.analysis["closest"] else float(left.times[-1])
            for c, clip, t in ((0, left, t_close), (1, right, t_close)):
                idx = index_at(clip.times, float(t)) if c == 0 else len(clip.times) - 1
                # 修复臂用结束时刻，轨迹完整，对穿对比更清楚；全关用最近时刻。
                if clip.spec["id"].endswith("-fix"):
                    idx = len(clip.times) - 1
                title = f"{SCENE_TITLE[clip.spec['scene']]}  {clip.spec['short']}"
                _still_panel(axes[r, c], clip, idx, xlim, ylim, title)
        fig.tight_layout(rect=(0, 0, 1, 0.98))
        out_png.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_png, bbox_inches="tight")
    finally:
        plt.close(fig)
    return out_png


def mp4_to_gif(mp4: Path, gif: Path, *, fps: float, width: int = 720) -> Path:
    gif.parent.mkdir(parents=True, exist_ok=True)
    vf = (
        f"fps={fps:.4g},scale={int(width)}:-1:flags=lanczos,"
        "split[s0][s1];[s0]palettegen=max_colors=128:stats_mode=full[p];"
        "[s1][p]paletteuse=dither=bayer:bayer_scale=5"
    )
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-i", str(mp4), "-vf", vf, "-an", str(gif)]
    subprocess.run(cmd, check=True)
    return gif


def probe_duration(path: Path) -> float:
    p = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "default=nw=1:nk=1", str(path)],
        capture_output=True,
        text=True,
        check=True,
    )
    return float(p.stdout.strip())


def run_sim(diag, spec: dict, log_dir: Path) -> dict:
    import yopo_l0.backend.bspline as bs
    import yopo_l0.policy as P
    import yopo_l0.runner as R

    log_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.perf_counter()
    card = diag.build_card(spec["scene"], spec["arm"], spec["extra"])
    log = diag._Log()
    log.stop_after = float(spec["post_s"])
    log.proto_brake = bool(spec["proto_brake"])
    log.dump = None
    orig_cls = P.POLICIES["upstream"]
    orig_pairs = R._note_pairs
    P.POLICIES["upstream"] = diag._make_policy_cls(orig_cls, log)
    R._note_pairs = diag._make_note_pairs(log)
    orig_refine = diag._wrap_refine(log)
    try:
        out = R.run(card, R.RunConfig(seed=int(spec["seed"]), shadow=True))
    finally:
        P.POLICIES["upstream"] = orig_cls
        R._note_pairs = orig_pairs
        bs.refine_yopo = orig_refine
    R.write(out, log_dir)
    return {
        "result": out.result,
        "cross": None if log.cross is None else {"t": log.cross["t"], "pair": list(log.cross["pair"]), "dist": log.cross["dist"], "reason": log.cross["reason"]},
        "wall_time_s": time.perf_counter() - t0,
        "card": card,
    }


def write_meta(spec: dict, log_dir: Path, run_info: dict | None, analysis: dict, times: np.ndarray, uavs: list[str], diag_root: Path) -> dict:
    result = {}
    if (log_dir / "result.json").is_file():
        result = json.loads((log_dir / "result.json").read_text())
    if run_info and "result" in run_info:
        result = run_info["result"]
    live = analysis["live_min_m"]
    true_min = float(np.nanmin(live)) if np.isfinite(live).any() else None
    censored = None
    if analysis["first_cross"] is not None:
        censored = analysis["first_cross"]["dist_m"]
    elif true_min is not None:
        censored = true_min
    meta = {
        "id": spec["id"],
        "scene": spec["scene"],
        "scene_card": SCENES[spec["scene"]],
        "title": spec["title"],
        "arm": spec["arm"],
        "proto_brake": bool(spec["proto_brake"]),
        "extra": spec["extra"],
        "post_s": spec["post_s"],
        "seed": spec["seed"],
        "diag_root": str(diag_root),
        "outcome": result.get("outcome"),
        "reason": result.get("reason"),
        "arrived": result.get("arrived"),
        "n_drones": result.get("n_drones") or len(uavs),
        "sim_duration_s": result.get("sim_duration_s") if result.get("sim_duration_s") is not None else (float(times[-1]) if len(times) else None),
        "min_inter_drone_distance_m": true_min,
        "min_inter_censored_m": censored,
        "runner_min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "first_cross": analysis["first_cross"],
        "closest": analysis["closest"],
        "n_ticks": int(len(times)),
    }
    if run_info and "cross" in run_info:
        meta["diag_cross"] = run_info["cross"]
        meta["wall_time_s"] = run_info.get("wall_time_s")
    (log_dir / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    return meta


def load_clip(spec: dict, log_dir: Path, diag_root: Path, card=None) -> ClipData:
    ticks = load_ticks(log_dir / "ticks.jsonl")
    times, uavs, tracks = align(ticks)
    analysis = analyze_tracks(times, uavs, tracks)
    if card is None:
        from yopo_l0.scene import load_card

        card = load_card(diag_root / SCENES[spec["scene"]])
    meta_path = log_dir / "meta.json"
    if meta_path.is_file():
        meta = json.loads(meta_path.read_text())
        # 用当前轨迹重算最小距离，避免旧 meta 和 ticks 不一致。
        meta["min_inter_drone_distance_m"] = float(np.nanmin(analysis["live_min_m"])) if np.isfinite(analysis["live_min_m"]).any() else None
        meta["first_cross"] = analysis["first_cross"]
        meta["closest"] = analysis["closest"]
    else:
        meta = write_meta(spec, log_dir, None, analysis, times, uavs, diag_root)
    xlim, ylim = bounds_for(tracks, card)
    return ClipData(spec, times, uavs, tracks, analysis, meta, card, log_dir, xlim, ylim)


def ensure_clip(diag, spec: dict, log_dir: Path, diag_root: Path, reuse: bool) -> ClipData:
    ticks_path = log_dir / "ticks.jsonl"
    if reuse and ticks_path.is_file():
        print(f"复用日志 {ticks_path}", flush=True)
        return load_clip(spec, log_dir, diag_root)
    print(f"开跑 {spec['id']}  scene={spec['scene']} arm={spec['arm']} proto_brake={spec['proto_brake']} seed={spec['seed']}", flush=True)
    info = run_sim(diag, spec, log_dir)
    ticks = load_ticks(log_dir / "ticks.jsonl")
    times, uavs, tracks = align(ticks)
    analysis = analyze_tracks(times, uavs, tracks)
    write_meta(spec, log_dir, info, analysis, times, uavs, diag_root)
    clip = load_clip(spec, log_dir, diag_root, card=info["card"])
    print(
        f"  完成 {spec['id']}  outcome={clip.meta.get('outcome')}  "
        f"最小机间 {clip.meta.get('min_inter_drone_distance_m'):.4f} m  "
        f"到达 {clip.meta.get('arrived')}/{clip.meta.get('n_drones')}  "
        f"时长 {clip.meta.get('sim_duration_s')} s  墙钟 {info['wall_time_s']:.1f} s",
        flush=True,
    )
    return clip


def fmt_size(n: int) -> str:
    if n >= 1048576:
        return f"{n / 1048576:.1f} MiB"
    if n >= 1024:
        return f"{n / 1024:.1f} KiB"
    return f"{n} B"


def write_index(out: Path, clips: list[ClipData], artifacts: list[dict], diag_root: Path, diag_sha: str) -> Path:
    lines = [
        "# E-B4 机间碰撞修复前后回放（票 V1）",
        "",
        "只出图，不改算法。全关臂越线后再飞 1 s；修复臂为 E-A1 门槛 1.0 m + 刹停原型。",
        "",
        "| 项 | 值 |",
        "|---|---|",
        f"| 诊断代码 | `{diag_root}` `{diag_sha}` |",
        "| seed | 0 |",
        "| 碰撞线 | 0.70 m（2 × 机体半径 0.35 m） |",
        "| 播放时长上限 | 15 s |",
        "",
        "## 各段关键数字",
        "",
        "| 段 | 结果 | 到达 | 最小机间 m | 截断最小机间 m（首次越线） | 仿真时长 s | 越线时刻 s | 最近时刻 s |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for c in clips:
        fc, cl = c.analysis["first_cross"], c.analysis["closest"]
        lines.append(
            "| {id} | {out} | {arr}/{n} | {mn:.4f} | {cen} | {dur:.2f} | {ct} | {lt} |".format(
                id=c.spec["id"],
                out=c.meta.get("outcome"),
                arr=c.meta.get("arrived"),
                n=c.meta.get("n_drones"),
                mn=c.meta.get("min_inter_drone_distance_m") or float("nan"),
                cen="—" if fc is None else f"{fc['dist_m']:.4f}",
                dur=c.meta.get("sim_duration_s") or float("nan"),
                ct="—" if fc is None else f"{fc['t_s']:.2f}（{drone_no(fc['pair'][0])}-{drone_no(fc['pair'][1])}）",
                lt="—" if cl is None else f"{cl['t_s']:.2f}（{drone_no(cl['pair'][0])}-{drone_no(cl['pair'][1])}）",
            )
        )
    lines += [
        "",
        "全关臂的「最小机间」是越线后再飞 1 s 期间的真实最小值，不是运行器在 0.70 m 处截断记下的值。",
        "",
        "## 文件",
        "",
        "| 文件 | 时长 s | 大小 | 说明 |",
        "|---|---|---|---|",
    ]
    for art in artifacts:
        path = Path(art["path"])
        rel = path.relative_to(out) if path.is_relative_to(out) else path
        dur = art.get("duration_s")
        dur_s = "—" if dur is None else f"{float(dur):.2f}"
        size = fmt_size(path.stat().st_size) if path.is_file() else "（缺失）"
        lines.append(f"| `{rel}` | {dur_s} | {size} | {art.get('note', '')} |")
    lines.append("")
    text = "\n".join(lines) + "\n"
    path = out / "README.md"
    path.write_text(text)
    return path


def git_sha(root: Path) -> str:
    try:
        p = subprocess.run(["git", "rev-parse", "--short=7", "HEAD"], cwd=root, capture_output=True, text=True, check=True)
        return p.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--diag-root", default=None, help="含 scripts/e-b4-peer-diag.py 的检出（默认本仓库或 E-B4 worktree）")
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="成片和日志目录，默认 /workspace/yopo-ops/viz")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--post-s", type=float, default=1.0, help="全关臂越线后再飞多久")
    ap.add_argument("--fps", type=float, default=10)
    ap.add_argument("--max-seconds", type=float, default=15, help="成片播放时长上限")
    ap.add_argument("--reuse-logs", action="store_true", help="复用 --out/logs 里已有的 ticks.jsonl")
    ap.add_argument("--clips", default=None, help="只跑这些段，逗号分隔，例如 swap8-off,swap8-fix")
    ap.add_argument("--skip-gif", action="store_true")
    ap.add_argument("--gif-width", type=int, default=720)
    args = ap.parse_args(argv)

    if not (0 < args.max_seconds <= 15):
        print("成片时长上限必须在 (0, 15] 秒", file=sys.stderr)
        return 2
    if args.fps <= 0:
        print("fps 必须为正", file=sys.stderr)
        return 2
    if not animation.FFMpegWriter.isAvailable():
        print("渲染 mp4 需要 ffmpeg 在 PATH 里", file=sys.stderr)
        return 2

    diag_root = Path(args.diag_root).resolve() if args.diag_root else default_diag_root()
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    specs = clip_specs(args.seed, args.post_s)
    if args.clips:
        want = {s.strip() for s in args.clips.split(",") if s.strip()}
        unknown = want - {s["id"] for s in specs}
        if unknown:
            print(f"未知段：{', '.join(sorted(unknown))}", file=sys.stderr)
            return 2
        specs = [s for s in specs if s["id"] in want]
        # 保持默认顺序。
        order = {s["id"]: i for i, s in enumerate(clip_specs(args.seed, args.post_s))}
        specs.sort(key=lambda s: order[s["id"]])

    print(f"diag-root={diag_root}  out={out}  sha={git_sha(diag_root)}", flush=True)
    diag = load_diag(diag_root)
    clips: list[ClipData] = []
    for spec in specs:
        log_dir = out / "logs" / f"{spec['id']}-seed{spec['seed']}"
        clips.append(ensure_clip(diag, spec, log_dir, diag_root, args.reuse_logs))

    artifacts: list[dict] = []
    clip_dir = out / "clips"
    cmp_dir = out / "compare"
    for clip in clips:
        mp4 = clip_dir / f"{clip.spec['id']}-seed{clip.spec['seed']}.mp4"
        print(f"渲染 {mp4.name}", flush=True)
        info = render_top_video(clip, mp4, fps=args.fps, max_seconds=args.max_seconds)
        dur = probe_duration(mp4)
        if dur > args.max_seconds + 0.5:
            raise SystemExit(f"{mp4} 时长 {dur:.3f}s 超过上限 {args.max_seconds}s")
        artifacts.append({"path": mp4, "duration_s": dur, "note": f"俯视回放 · {clip.spec['title']}"})
        if not args.skip_gif:
            gif = clip_dir / f"{clip.spec['id']}-seed{clip.spec['seed']}.gif"
            mp4_to_gif(mp4, gif, fps=args.fps, width=args.gif_width)
            artifacts.append({"path": gif, "duration_s": dur, "note": "同上，GIF"})

    by_scene: dict[str, list[ClipData]] = {}
    for c in clips:
        by_scene.setdefault(c.spec["scene"], []).append(c)
    for scene, group in by_scene.items():
        if len(group) != 2:
            continue
        left = next(c for c in group if c.spec["id"].endswith("-off"))
        right = next(c for c in group if c.spec["id"].endswith("-fix"))
        png = cmp_dir / f"{scene}-seed{left.spec['seed']}.png"
        print(f"对照静图 {png.name}", flush=True)
        render_compare_png(left, right, png)
        artifacts.append({"path": png, "duration_s": None, "note": f"{SCENE_TITLE[scene]} 左右对照静图（三行时刻）"})
        mp4 = cmp_dir / f"{scene}-seed{left.spec['seed']}.mp4"
        print(f"对照动图 {mp4.name}", flush=True)
        info = render_compare_video(left, right, mp4, fps=args.fps, max_seconds=args.max_seconds)
        dur = probe_duration(mp4)
        if dur > args.max_seconds + 0.5:
            raise SystemExit(f"{mp4} 时长 {dur:.3f}s 超过上限 {args.max_seconds}s")
        artifacts.append({"path": mp4, "duration_s": dur, "note": f"{SCENE_TITLE[scene]} 左右对照"})
        if not args.skip_gif:
            gif = cmp_dir / f"{scene}-seed{left.spec['seed']}.gif"
            # 对照画面更宽，GIF 放宽到 1280。
            mp4_to_gif(mp4, gif, fps=args.fps, width=min(1280, args.gif_width * 2))
            artifacts.append({"path": gif, "duration_s": dur, "note": "同上，GIF"})

    if len(clips) == 4:
        png = cmp_dir / f"all-seed{clips[0].spec['seed']}.png"
        print(f"总对照 {png.name}", flush=True)
        render_all_png(clips, png)
        artifacts.append({"path": png, "duration_s": None, "note": "两场景四段总对照（左全关、右修复）"})

    metrics = {
        "diag_root": str(diag_root),
        "diag_commit": git_sha(diag_root),
        "seed": args.seed,
        "post_s": args.post_s,
        "clips": [
            {
                "id": c.spec["id"],
                "scene": c.spec["scene"],
                "title": c.spec["title"],
                "outcome": c.meta.get("outcome"),
                "reason": c.meta.get("reason"),
                "arrived": c.meta.get("arrived"),
                "n_drones": c.meta.get("n_drones"),
                "min_inter_drone_distance_m": c.meta.get("min_inter_drone_distance_m"),
                "min_inter_censored_m": c.meta.get("min_inter_censored_m"),
                "sim_duration_s": c.meta.get("sim_duration_s"),
                "first_cross": c.analysis["first_cross"],
                "closest": c.analysis["closest"],
            }
            for c in clips
        ],
        "artifacts": [{"path": str(a["path"]), "duration_s": a.get("duration_s"), "note": a.get("note")} for a in artifacts],
    }
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2, ensure_ascii=False) + "\n")
    write_index(out, clips, artifacts, diag_root, git_sha(diag_root))
    print(f"写入 {out / 'metrics.json'} 和 {out / 'README.md'}", flush=True)
    for c in clips:
        print(
            f"数字 {c.spec['id']}: 到达 {c.meta.get('arrived')}/{c.meta.get('n_drones')}  "
            f"最小机间 {c.meta.get('min_inter_drone_distance_m'):.4f} m  "
            f"outcome={c.meta.get('outcome')}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
