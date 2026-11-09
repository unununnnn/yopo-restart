"""唯一测试接缝：场景卡 → 运行器 → 结果记录 + 逐 tick 日志。

锁步：每 tick 依次 传感器（到期才拍）→ 规划（到期才规划，墙钟不计入仿真时间）→ 动力学 → 碰撞真值。
"""

from __future__ import annotations

import json
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .dynamics import State, make_model
from .policy import POLICIES, Observation
from .scene import SceneCard
from .sensor import CameraConfig, DepthCamera

WALL_FIELDS = frozenset({"wall_time_s"})  # 确定性比较时排除的字段
REPO = Path(__file__).resolve().parents[1]


@dataclass
class RunConfig:
    seed: int = 0
    policy: str = "hover"
    shadow: bool = False  # 票 03 接入；当前只写进结果记录
    tier: str = "L0"
    duration_s: float | None = None  # None = 用场景卡 task.duration_s
    record_planner_inputs: bool = False


@dataclass
class RunOutput:
    result: dict
    ticks: list[dict]
    planner_inputs: list[str] = field(default_factory=list)  # Observation 摘要序列


def git_commit() -> str:
    try:
        sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPO, capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=REPO, capture_output=True, text=True).stdout.strip()
        return sha + ("-dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _due(k: int, rate: int, physics: int) -> bool:
    return k == 0 or (k * rate) // physics != ((k - 1) * rate) // physics


def run(card: SceneCard, cfg: RunConfig) -> RunOutput:
    if cfg.tier != "L0":
        raise ValueError("本运行器只实现 L0")
    wall0 = time.perf_counter()
    c = card.card
    rates = c["rates_hz"]
    phys, cam_hz, plan_hz = int(rates["physics"]), int(rates["camera"]), int(rates["planner"])
    dt = 1.0 / phys
    duration = float(cfg.duration_s if cfg.duration_s is not None else c["task"]["duration_s"])
    n_ticks = int(round(duration * phys))
    radius = float(c["criteria"]["body_radius_m"])
    tol = float(c["criteria"]["hover_tolerance_m"])
    model = make_model(c["dynamics"])
    cam_cfg = CameraConfig.from_card(c["camera"])

    root = np.random.SeedSequence(cfg.seed)
    drones = []
    for d, ss in zip(c["drones"], root.spawn(len(c["drones"])), strict=True):
        start = np.asarray(d["start_enu"], dtype=float)
        drones.append({
            "id": d["id"],
            "goal": np.asarray(d.get("hover_enu", start), dtype=float),  # 悬停点，默认 = 起点
            "state": State(0.0, start.copy(), np.zeros(3), np.zeros(3), 0.0),
            "camera": DepthCamera(cam_cfg, card.obstacles, np.random.default_rng(ss)),
            "policy": POLICIES[cfg.policy](),
            "frame": None, "plan": None,
            "min_obs": card.min_distance(start), "max_err": 0.0,
        })

    ticks: list[dict] = []
    inputs: list[str] = []
    outcome, reason = None, ""
    frames = plans = 0
    k = 0
    for k in range(n_ticks):
        now = k / phys
        for dr in drones:
            s = dr["state"]
            if _due(k, cam_hz, phys):
                dr["frame"] = dr["camera"].capture(s.p, s.yaw, now)
                frames += 1
            if _due(k, plan_hz, phys) or dr["plan"] is None:
                obs = Observation(dr["frame"], s, dr["goal"].copy(), now)
                if cfg.record_planner_inputs:
                    inputs.append(obs.digest())
                dr["plan"] = dr["policy"].plan(obs)
                plans += 1
            s = model.step(s, dr["plan"], dt)
            dr["state"] = s
            dist = card.min_distance(s.p)
            err = float(np.linalg.norm(s.p - dr["goal"]))
            dr["min_obs"] = min(dr["min_obs"], dist)
            dr["max_err"] = max(dr["max_err"], err)
            ticks.append({
                "k": k, "t": round(s.t, 9), "uav": dr["id"], "p": s.p.tolist(), "v": s.v.tolist(),
                "a": s.a.tolist(), "frame_stamp": None if dr["frame"] is None else dr["frame"].stamp,
                "min_obstacle_m": dist, "hover_error_m": err,
            })
            if dist <= radius:
                outcome, reason = "COLLISION", f"{dr['id']}:TRUTH_POSITION"
        if outcome:
            break
    if outcome is None:
        worst = max(d["max_err"] for d in drones)
        outcome, reason = ("SUCCESS", "HOVER_HELD") if worst <= tol else ("HOVER_DRIFT", "HOVER_TOLERANCE")

    result = {
        "commit": git_commit(),
        "scene": card.name,
        "scene_card_hash": card.hash,
        "seed": cfg.seed,
        "tier": cfg.tier,
        "policy": cfg.policy,
        "shadow": cfg.shadow,
        "dynamics": c["dynamics"]["model"],
        "n_drones": len(drones),
        "outcome": outcome,
        "reason": reason,
        "arrival_time_s": None,  # 票 02
        "min_obstacle_distance_m": min(d["min_obs"] for d in drones),
        "max_hover_error_m": max(d["max_err"] for d in drones),
        "min_inter_drone_distance_m": None,  # 票 05
        "shadow_reject_rate": None,  # 票 03
        "sim_duration_s": (k + 1) / phys,
        "ticks": k + 1,
        "frames": frames,
        "plans": plans,
        "wall_time_s": time.perf_counter() - wall0,
    }
    return RunOutput(result, ticks, inputs)


def write(out: RunOutput, out_dir: str | Path) -> Path:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "result.json").write_text(json.dumps(out.result, indent=2, ensure_ascii=False) + "\n")
    with (out_dir / "ticks.jsonl").open("w") as f:
        for t in out.ticks:
            f.write(json.dumps(t, separators=(",", ":")) + "\n")
    return out_dir
