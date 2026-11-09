"""离线视频：从逐 tick 日志渲染俯视 + 侧视 mp4，播放时长 ≤60 s。

    python -m yopo_l0.render --log out/.../ticks.jsonl --out out/.../preview.mp4
    python -m yopo_l0.render --log out/.../ticks.jsonl --scene scenes/pillar-forest.json --out out/.../preview.mp4

仿真短于 60 s 时按实时播放；更长则抽帧，使成片不超过 --max-seconds（上限 60）。
需要系统里的 ffmpeg。障碍几何只用于画图，不进规划器。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.patches import Circle, Rectangle

from .scene import Box, Cylinder, Ground, load_card

COLORS = ("#d85a30", "#2a6fdb", "#2f9e44", "#7048c6")
_FONT_READY = False


@dataclass(frozen=True)
class RenderInfo:
    path: Path
    n_frames: int
    fps: float
    duration_s: float
    sped_up: bool


def select_frame_indices(n_samples: int, sim_duration_s: float, *, fps: float, max_seconds: float) -> np.ndarray:
    """抽出成片帧下标。len(result) / fps ≤ max_seconds，且 max_seconds 不得超过 60。"""
    if n_samples < 1:
        raise ValueError("日志是空的")
    if fps <= 0:
        raise ValueError("fps 必须为正")
    if not (0 < max_seconds <= 60):
        raise ValueError("视频时长上限必须在 (0, 60] 秒")
    cap = max(1, int(np.floor(max_seconds * fps + 1e-9)))
    if n_samples == 1:
        return np.asarray([0], dtype=int)
    play = min(max(float(sim_duration_s), 1.0 / fps), float(max_seconds))
    n_frames = int(round(play * fps))
    n_frames = max(1, min(n_samples, n_frames, cap))
    raw = np.rint(np.linspace(0, n_samples - 1, n_frames))
    out = np.unique(np.clip(raw, 0, n_samples - 1).astype(int))
    return out


def _ensure_cjk_font() -> None:
    global _FONT_READY
    if _FONT_READY:
        return
    from matplotlib import font_manager

    for path in (
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    ):
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
        break
    _FONT_READY = True


def _load_ticks(path: Path) -> list[dict]:
    rows: list[dict] = []
    try:
        text = path.read_text()
    except OSError as e:
        raise ValueError(f"读不到日志 {path}：{e}") from e
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


def _align(ticks: list[dict]):
    by_k: dict[int, dict] = {}
    for row in ticks:
        try:
            k = int(row["k"])
            uav = str(row["uav"])
            t = float(row["t"])
            p = row["p"]
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f"无法解析 tick：{row!r}") from e
        if not isinstance(p, (list, tuple)) or len(p) != 3:
            raise ValueError(f"tick 的 p 必须是长度 3：{row!r}")
        slot = by_k.setdefault(k, {"t": t, "uavs": {}})
        slot["t"] = t
        slot["uavs"][uav] = (float(p[0]), float(p[1]), float(p[2]))
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


def _add_obstacles(ax_top, ax_side, obstacles) -> float | None:
    ground_z = None
    for ob in obstacles:
        if isinstance(ob, Box):
            x, y, z = ob.lo
            dx, dy, dz = (np.asarray(ob.hi) - np.asarray(ob.lo)).tolist()
            kw = dict(facecolor="#c8c8c8", edgecolor="#666666", lw=0.6, zorder=0)
            ax_top.add_patch(Rectangle((x, y), dx, dy, **kw))
            ax_side.add_patch(Rectangle((x, z), dx, dz, **kw))
        elif isinstance(ob, Cylinder):
            cx, cy = ob.center_xy
            kw = dict(facecolor="#a8a8a8", edgecolor="#555555", lw=0.6, zorder=1)
            ax_top.add_patch(Circle((cx, cy), ob.radius, **kw))
            ax_side.add_patch(Rectangle((cx - ob.radius, ob.z_lo), 2 * ob.radius, ob.z_hi - ob.z_lo, **kw))
        elif isinstance(ob, Ground):
            ground_z = ob.z
            ax_side.axhline(ob.z, color="#888888", lw=0.8, zorder=0)
    return ground_z


def render_video(
    log: str | Path,
    out: str | Path,
    *,
    scene: str | Path | None = None,
    fps: float = 10,
    max_seconds: float = 60,
) -> RenderInfo:
    if not animation.FFMpegWriter.isAvailable():
        raise RuntimeError("渲染 mp4 需要 ffmpeg 在 PATH 里")
    _ensure_cjk_font()
    log_path, out_path = Path(log), Path(out)
    times, uavs, tracks = _align(_load_ticks(log_path))
    pts = np.concatenate([tracks[u] for u in uavs], axis=0)
    pts = pts[np.isfinite(pts).all(axis=1)]
    if len(pts) == 0:
        raise ValueError(f"日志里没有有效位置：{log_path}")

    card = load_card(scene) if scene is not None else None
    name = card.name if card is not None else log_path.stem
    obstacles = card.obstacles if card is not None else ()
    radius = float(card.card["criteria"]["body_radius_m"]) if card is not None else 0.35
    sim_duration = float(times[-1] - times[0]) if len(times) > 1 else 0.0
    indices = select_frame_indices(len(times), sim_duration, fps=float(fps), max_seconds=float(max_seconds))
    duration_s = len(indices) / float(fps)
    if duration_s > float(max_seconds) + 1e-6:
        raise RuntimeError(f"成片时长 {duration_s:.3f}s 超过上限 {max_seconds}s")
    sped_up = sim_duration > duration_s + 1.0 / float(fps)

    pad = 4.0
    xmin, xmax = float(pts[:, 0].min()) - pad, float(pts[:, 0].max()) + pad
    ymin, ymax = float(pts[:, 1].min()) - pad, float(pts[:, 1].max()) + pad

    fig, (ax_top, ax_side) = plt.subplots(2, 1, figsize=(8.0, 6.0), dpi=100)
    try:
        fig.patch.set_facecolor("white")
        ground_z = _add_obstacles(ax_top, ax_side, obstacles)
        z_floor = 0.0 if ground_z is None else ground_z
        zmin = min(float(pts[:, 2].min()), z_floor) - 0.4
        zmax = float(pts[:, 2].max()) + 1.5
        ax_top.set_xlim(xmin, xmax)
        ax_top.set_ylim(ymin, ymax)
        ax_side.set_xlim(xmin, xmax)
        ax_side.set_ylim(zmin, zmax)
        ax_top.set_aspect("equal", adjustable="box")
        ax_side.set_aspect("equal", adjustable="box")
        ax_top.set_title("俯视")
        ax_side.set_title("侧视")
        ax_top.set_xlabel("x / m")
        ax_top.set_ylabel("y / m")
        ax_side.set_xlabel("x / m")
        ax_side.set_ylabel("z / m")
        ax_top.grid(True, alpha=0.3)
        ax_side.grid(True, alpha=0.3)

        artists = []
        for i, uav in enumerate(uavs):
            color = COLORS[i % len(COLORS)]
            trail_t, = ax_top.plot([], [], color=color, lw=1.4, label=uav, zorder=3)
            trail_s, = ax_side.plot([], [], color=color, lw=1.4, zorder=3)
            mark_t, = ax_top.plot([], [], "o", color=color, ms=5, zorder=4)
            mark_s, = ax_side.plot([], [], "o", color=color, ms=5, zorder=4)
            body_t = Circle((0, 0), radius, fill=False, ec=color, lw=1.0, zorder=4)
            body_s = Circle((0, 0), radius, fill=False, ec=color, lw=1.0, zorder=4)
            ax_top.add_patch(body_t)
            ax_side.add_patch(body_s)
            artists.append((trail_t, trail_s, mark_t, mark_s, body_t, body_s))
        if len(uavs) > 1:
            ax_top.legend(loc="upper right", fontsize=8)
        title = fig.suptitle("", fontsize=11)
        fig.subplots_adjust(hspace=0.45, top=0.88)

        out_path.parent.mkdir(parents=True, exist_ok=True)
        writer = animation.FFMpegWriter(fps=float(fps), codec="libx264", extra_args=["-pix_fmt", "yuv420p"])
        try:
            with writer.saving(fig, str(out_path), dpi=100):
                for src in indices:
                    for uav, arts in zip(uavs, artists, strict=True):
                        trail = tracks[uav][: src + 1]
                        arts[0].set_data(trail[:, 0], trail[:, 1])
                        arts[1].set_data(trail[:, 0], trail[:, 2])
                        p = tracks[uav][src]
                        visible = bool(np.isfinite(p).all())
                        arts[2].set_data([p[0]] if visible else [], [p[1]] if visible else [])
                        arts[3].set_data([p[0]] if visible else [], [p[2]] if visible else [])
                        arts[4].set_visible(visible)
                        arts[5].set_visible(visible)
                        if visible:
                            arts[4].set_center((float(p[0]), float(p[1])))
                            arts[5].set_center((float(p[0]), float(p[2])))
                    note = f"{name}  t={float(times[src]):.2f}s"
                    if sped_up:
                        note += f"  （{sim_duration / duration_s:.1f}×）"
                    title.set_text(note)
                    writer.grab_frame()
        except Exception:
            out_path.unlink(missing_ok=True)
            raise
    finally:
        plt.close(fig)

    return RenderInfo(out_path, len(indices), float(fps), duration_s, sped_up)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="从 ticks.jsonl 离线渲染俯视+侧视视频（播放 ≤60s）")
    ap.add_argument("--log", required=True, help="逐 tick 日志（ticks.jsonl）")
    ap.add_argument("--out", required=True, help="输出 mp4")
    ap.add_argument("--scene", default=None, help="场景卡 JSON，用于画障碍；不参与规划")
    ap.add_argument("--fps", type=float, default=10)
    ap.add_argument("--max-seconds", type=float, default=60, help="成片时长上限，最大 60")
    a = ap.parse_args(argv)
    try:
        info = render_video(a.log, a.out, scene=a.scene, fps=a.fps, max_seconds=a.max_seconds)
    except (ValueError, RuntimeError) as e:
        print(e, file=sys.stderr)
        return 2
    print(f"写入 {info.path}（{info.n_frames} 帧，{info.duration_s:.2f}s）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
