"""P1b：深度渲染关 vs ``YOPO_SENSOR_ACCEL=1``。

单帧用 ``crossing-6-forest`` 的障碍（中段密度带一帧，另加起点一帧）。
闭环是 ``g2-pillar``、``crossing-forest``、``crossing-6-forest``，seed 0，影子盾开。

``--partial`` 少重复几次，并把闭环截到 ``--partial-s`` 秒，用来确认脚本还能跑。
完整数字不要用 partial 的结果。
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import sys
import time
from pathlib import Path

import numpy as np

from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import Box, Cylinder, Ground, load_card
from yopo_l0.sensor import CameraConfig, DepthCamera, _ray_dirs_cam

ROOT = Path(__file__).resolve().parents[1]
SCENES = (
    "scenes/g2-pillar.json",
    "scenes/crossing-forest.json",
    "scenes/crossing-6-forest.json",
)
# 林带中段。crossing-6-forest 的密度档是 medium，x∈[16, 30] 用中段净距。
WOODS_POSE = (np.array([20.0, 0.0, 1.5]), 0.0)
START_POSE = (np.array([6.0, 1.5, 1.5]), 0.0)


def _stats(samples_s: list[float]) -> dict:
    arr = np.asarray(samples_s, dtype=np.float64) * 1e3
    return {
        "n": int(arr.size),
        "min_ms": float(arr.min()),
        "mean_ms": float(arr.mean()),
        "p50_ms": float(np.percentile(arr, 50)),
        "p95_ms": float(np.percentile(arr, 95)),
        "max_ms": float(arr.max()),
    }


def _blas() -> str:
    path = Path("/lib/x86_64-linux-gnu/libblas.so.3")
    try:
        return os.path.realpath(path)
    except OSError:
        return "unknown"


def _cpu() -> dict:
    model = "unknown"
    cpus = 0
    for line in Path("/proc/cpuinfo").read_text().splitlines():
        if line.startswith("model name") and model == "unknown":
            model = line.split(":", 1)[1].strip()
        if line.startswith("processor"):
            cpus += 1
    mem_kib = 0
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemTotal:"):
            mem_kib = int(line.split()[1])
            break
    return {
        "model": model,
        "cpus": cpus,
        "mem_gib": round(mem_kib / (1024 * 1024), 2),
        "uname": platform.platform(),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "blas": _blas(),
        "OMP_NUM_THREADS": os.environ.get("OMP_NUM_THREADS"),
        "MKL_NUM_THREADS": os.environ.get("MKL_NUM_THREADS"),
        "OPENBLAS_NUM_THREADS": os.environ.get("OPENBLAS_NUM_THREADS"),
        "loadavg": [float(x) for x in Path("/proc/loadavg").read_text().split()[:3]],
    }


def _census(card) -> dict:
    obs = card.obstacles
    return {
        "boxes": sum(isinstance(ob, Box) for ob in obs),
        "upright_cylinders": sum(isinstance(ob, Cylinder) and ob.upright() for ob in obs),
        "lean_cylinders": sum(isinstance(ob, Cylinder) and not ob.upright() for ob in obs),
        "grounds": sum(isinstance(ob, Ground) for ob in obs),
        "camera": card.card["camera"],
    }


def _time_capture(card, pos, yaw, accel: str, repeats: int) -> dict:
    os.environ["YOPO_SENSOR_ACCEL"] = accel
    cfg = CameraConfig.from_card(card.card["camera"])
    cam = DepthCamera(cfg, card.obstacles, np.random.default_rng(0))
    for _ in range(2):
        cam.capture(pos, yaw, 0.0)
    samples = []
    for i in range(repeats):
        t0 = time.perf_counter()
        cam.capture(pos, yaw, float(i))
        samples.append(time.perf_counter() - t0)
    return _stats(samples)


def _grid_ms(cfg: CameraConfig, repeats: int = 20) -> dict:
    """不走缓存的网格重建，对照缓存命中。"""
    fresh = _ray_dirs_cam.__wrapped__
    for _ in range(2):
        fresh(cfg)
    cold = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        fresh(cfg)
        cold.append(time.perf_counter() - t0)
    warm = []
    _ray_dirs_cam(cfg)
    for _ in range(repeats):
        t0 = time.perf_counter()
        _ray_dirs_cam(cfg)
        warm.append(time.perf_counter() - t0)
    return {"rebuild_ms": _stats(cold), "cached_ms": _stats(warm)}


def _positions(out) -> np.ndarray:
    return np.asarray([row["p"] for row in out.ticks], dtype=np.float64)


def _closed_loop(scene: str, accel: str, duration_s: float | None) -> dict:
    os.environ["YOPO_SENSOR_ACCEL"] = accel
    card = load_card(ROOT / scene)
    samples: list[float] = []
    orig = DepthCamera.capture

    def wrapped(self, position, yaw, stamp):
        t0 = time.perf_counter()
        try:
            return orig(self, position, yaw, stamp)
        finally:
            samples.append(time.perf_counter() - t0)

    DepthCamera.capture = wrapped
    try:
        out = run(card, RunConfig(seed=0, duration_s=duration_s))
    finally:
        DepthCamera.capture = orig
    result = out.result
    return {
        "accel": accel,
        "outcome": result["outcome"],
        "arrival_time_s": result["arrival_time_s"],
        "ticks": result["ticks"],
        "frames": result["frames"],
        "wall_time_s": result["wall_time_s"],
        "render_ms": _stats(samples) if samples else None,
        "positions": _positions(out),
    }


def _pair(off: dict, on: dict) -> dict:
    pos_off, pos_on = off.pop("positions"), on.pop("positions")
    same_shape = pos_off.shape == pos_on.shape
    max_abs = None
    if same_shape and pos_off.size:
        max_abs = float(np.max(np.abs(pos_on - pos_off)))
    return {
        "off": off,
        "on": on,
        "same_tick_count": bool(same_shape and off["ticks"] == on["ticks"]),
        "max_abs_position_m": max_abs,
        "same_outcome": (
            off["outcome"] == on["outcome"] and off["arrival_time_s"] == on["arrival_time_s"]
        ),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--partial", action="store_true", help="少次单帧，闭环只用 --partial-s")
    ap.add_argument("--partial-s", type=float, default=0.4)
    ap.add_argument("--repeats", type=int, default=None)
    args = ap.parse_args()
    repeats = args.repeats if args.repeats is not None else (5 if args.partial else 30)
    duration = args.partial_s if args.partial else None

    forest = load_card(ROOT / "scenes/crossing-6-forest.json")
    g2 = load_card(ROOT / "scenes/g2-pillar.json")
    single = {}
    for name, (pos, yaw) in (("woods", WOODS_POSE), ("start", START_POSE)):
        single[name] = {
            "pose_enu": [float(x) for x in pos] + [float(yaw)],
            "off": _time_capture(forest, pos, yaw, "0", repeats),
            "on": _time_capture(forest, pos, yaw, "1", repeats),
        }
    loops = {}
    for scene in SCENES:
        print(f"closed loop {scene} duration={duration}", file=sys.stderr, flush=True)
        off = _closed_loop(scene, "0", duration)
        on = _closed_loop(scene, "1", duration)
        pair = _pair(off, on)
        loops[Path(scene).stem] = pair
        print(
            f"done {scene} wall {pair['off']['wall_time_s']:.1f}s -> {pair['on']['wall_time_s']:.1f}s"
            f" pos {pair['max_abs_position_m']}",
            file=sys.stderr,
            flush=True,
        )

    try:
        import torch

        torch_threads = int(torch.get_num_threads())
        torch_version = torch.__version__
    except Exception:
        torch_threads = None
        torch_version = None

    report = {
        "partial": bool(args.partial),
        "repeats": repeats,
        "closed_loop_duration_s": duration,
        "machine": {**_cpu(), "torch": torch_version, "torch_num_threads": torch_threads},
        "crossing_6_forest": _census(forest),
        "ray_grid": {
            "crossing-6-forest": _grid_ms(CameraConfig.from_card(forest.card["camera"])),
            "g2-pillar": _grid_ms(CameraConfig.from_card(g2.card["camera"])),
        },
        "single_frame": single,
        "closed_loop": loops,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
