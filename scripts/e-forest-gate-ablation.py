#!/usr/bin/env python3
"""票 E-PG1：六机林带进度门控的 _LEADER_MARGIN_M 与分位数消融。只报数。

依据 docs/adr/0015-ghost-brake-stops.md 与 docs/LIMITATIONS.md：六机林带卡为避免
按 min 爬行太慢，当前写 progress_gate=false。本脚本在进程内读入
scenes/crossing-6-forest.json 与 scenes/crossing-6-forest-sparse.json，在内存里
改 task.progress_gate / task.progress_percentile，并改 yopo_l0.crossing 模块全局
_LEADER_MARGIN_M。不写场景 JSON，不改 yopo_l0/、core/、docs/tickets.md、README.md、
scripts/premerge.sh、scripts/regress.sh。

臂（两张卡、同一组种子）：

  arm0  基线。progress_gate=false，不改 _LEADER_MARGIN_M。当前场景卡配置。
  arm1  progress_gate=true，_LEADER_MARGIN_M=1.0，progress_percentile=0。
        ADR 0015 已知的按 min 慢爬复核。
  arm2  progress_gate=true，_LEADER_MARGIN_M=3.0，progress_percentile=0。
  arm3  progress_gate=true，_LEADER_MARGIN_M=6.0，progress_percentile=0。
  arm4  progress_gate=true，_LEADER_MARGIN_M=10.0，progress_percentile=0。
  arm5  progress_gate=true，_LEADER_MARGIN_M=3.0，progress_percentile=20。

arm1–arm5 都标「report-only / 偏离当前场景卡配置」。margin 不是场景卡字段，
所以 arm1–arm4 的场景卡哈希相同，差别只在进程内的 _LEADER_MARGIN_M。
_ahead_of_gate 和 summary 都在调用时读这个模块全局。

默认种子 0–4（每张卡 5 粒，六臂对齐）。0–9 用 --seeds 显式打开。
影子盾开，只记录，不改批准动作。没有后段，墙钟不进门。

队形误差 p95 是结果里的 formation_error_p95_m。门控开时对着门控弧长上的槽位，
门控关时对着墙钟 × speed，两列不能直接比大小。相对队形 p95 与门控无关：每 10 个
物理拍取一次全队（位置 − 槽位）的平均当中心，误差是各机到这个中心的距离，再取 p95。
悬停等待时长是每架在速度 ≤ 0.2 m/s 且离目标 > 0.4 m 的仿真时间，六架再平均。
LEADER_HOVER / LEADER_PREFIX / LEADER_HOLD 是提案窗口计数，一次是一架在自己的窗口里
选了该码。平均到达时间只平均 6/6 完全到达的种子。
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import time
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path

# 必须在 numpy / torch 之前。三个进程各用一个线程，避免 OpenMP 把 4 个核抢满。
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-forest-gate-ablation" / "partial"
BODY_R = 0.35
ROW_VERSION = 1
DEVIATION_LABEL = "report-only / 偏离当前场景卡配置"
BASELINE_LABEL = "当前场景卡配置"
HOVER_SPEED_MPS = 0.2
HOVER_GOAL_M = 0.4

_WATCHED = (
    "scenes/crossing-6-forest.json",
    "scenes/crossing-6-forest-sparse.json",
    "yopo_l0/crossing.py",
    "docs/tickets.md",
    "README.md",
    "scripts/premerge.sh",
    "scripts/regress.sh",
)

SCENES = ("medium", "sparse")
SCENE_CARDS = {
    "medium": ROOT / "scenes/crossing-6-forest.json",
    "sparse": ROOT / "scenes/crossing-6-forest-sparse.json",
}


@dataclass(frozen=True)
class Arm:
    arm_id: str
    title: str
    progress_gate: bool
    leader_margin_m: float | None
    progress_percentile: float

    @property
    def deviates(self) -> bool:
        """门控打开就偏离了六机林带卡上的 progress_gate=false。"""
        return self.progress_gate

    @property
    def config_status(self) -> str:
        if self.deviates:
            return DEVIATION_LABEL
        return BASELINE_LABEL

    @property
    def params(self) -> str:
        if not self.progress_gate:
            return "progress_gate=false"
        pct = self.progress_percentile
        pct_txt = str(int(pct)) if float(pct).is_integer() else str(pct)
        return (
            f"progress_gate=true, _LEADER_MARGIN_M={float(self.leader_margin_m):.1f}, "
            f"progress_percentile={pct_txt}"
        )


ARMS = (
    Arm("arm0", "基线", False, None, 0.0),
    Arm("arm1", "门控开 margin 1.0 分位 0", True, 1.0, 0.0),
    Arm("arm2", "门控开 margin 3.0 分位 0", True, 3.0, 0.0),
    Arm("arm3", "门控开 margin 6.0 分位 0", True, 6.0, 0.0),
    Arm("arm4", "门控开 margin 10.0 分位 0", True, 10.0, 0.0),
    Arm("arm5", "门控开 margin 3.0 分位 20", True, 3.0, 20.0),
)
ARM_BY_ID = {arm.arm_id: arm for arm in ARMS}

# 13e E+F / SUMMARY 在 f5985e4335fc 附近记下的全关逐种子数。只用来核对 arm0，对不上就照实写。
_OFF_ANCHORS = {
    "medium": {
        0: (6, 1.293, 0.306, 23.472, 26.08),
        1: (6, 1.392, 0.161, 23.813, 28.13),
        2: (6, 1.148, 0.202, 24.591, 26.90),
        3: (6, 1.189, 0.239, 24.195, 29.08),
        4: (6, 1.111, 0.194, 23.041, 28.40),
        5: (6, 1.260, 0.126, 26.298, 33.57),
        6: (6, 1.224, 0.191, 24.213, 32.07),
        7: (6, 1.248, 0.218, 26.108, 26.90),
        8: (6, 1.361, 0.170, 24.769, 26.00),
        9: (6, 1.187, 0.180, 25.937, 28.94),
    },
    "sparse": {
        0: (6, 0.959, 0.315, 20.976, 25.40),
        1: (6, 0.982, 0.178, 18.725, 23.24),
        2: (6, 1.242, 0.322, 20.661, 30.31),
        3: (6, 0.972, 0.380, 19.778, 31.58),
        4: (6, 1.322, 0.192, 20.842, 24.83),
        5: (6, 1.039, 0.253, 20.075, 24.40),
        6: (6, 1.012, 0.402, 21.695, 24.81),
        7: (6, 1.002, 0.186, 20.476, 29.30),
        8: (6, 1.043, 0.401, 22.922, 27.48),
        9: (6, 1.053, 0.158, 21.293, 26.06),
    },
}


def arm_by_id(arm_id: str) -> Arm:
    try:
        return ARM_BY_ID[arm_id]
    except KeyError as exc:
        raise KeyError(f"未知臂 {arm_id}") from exc


def watched_bytes() -> dict[str, bytes]:
    return {rel: (ROOT / rel).read_bytes() for rel in _WATCHED}


def load_scene_raw(scene: str) -> dict:
    """读磁盘上的场景卡。六机林带卡必须仍是 progress_gate=false、分位 0。"""
    if scene not in SCENE_CARDS:
        raise KeyError(f"未知场景 {scene}")
    path = SCENE_CARDS[scene]
    raw = json.loads(path.read_text())
    task = raw["task"]
    if task.get("progress_gate") is not False:
        raise RuntimeError(f"{path.name} 的 progress_gate 已不是 false，基线口径变了")
    if int(task.get("progress_percentile", 0)) != 0:
        raise RuntimeError(f"{path.name} 的 progress_percentile 已不是 0，基线口径变了")
    return raw


def build_card(scene: str, arm: Arm | str):
    """返回 SceneCard。偏离臂只改内存里的 task，磁盘上的卡仍是 load_scene_raw 读到的那份。"""
    from yopo_l0.forest import generate_forest_geometry
    from yopo_l0.scene import SceneCard, load_card, parse_geometry

    if isinstance(arm, str):
        arm = arm_by_id(arm)
    path = SCENE_CARDS[scene]
    if not arm.deviates:
        card = load_card(path)
        if card.card["task"]["progress_gate"] is not False:
            raise RuntimeError("load_card 后门控不是 false")
        return card
    cloned = copy.deepcopy(load_scene_raw(scene))
    pct = float(arm.progress_percentile)
    cloned["task"]["progress_gate"] = True
    cloned["task"]["progress_percentile"] = int(pct) if pct.is_integer() else pct
    geom = generate_forest_geometry(cloned["forest"])
    merged = {**cloned, "geometry": geom}
    return SceneCard(str(path), merged, parse_geometry(geom, merged.get("ground_z")))


@contextmanager
def override_leader_margin(margin: float | None):
    """进程内改 yopo_l0.crossing._LEADER_MARGIN_M。margin 为 None 时保持源码里的值。

    退出时一定写回。_ahead_of_gate 和 CrossingFleet.summary 都在调用时读这个全局。
    """
    import yopo_l0.crossing as crossing

    previous = crossing._LEADER_MARGIN_M
    try:
        if margin is not None:
            value = float(margin)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"_LEADER_MARGIN_M 必须是非负有限数，收到 {margin}")
            crossing._LEADER_MARGIN_M = value
        yield float(crossing._LEADER_MARGIN_M)
    finally:
        crossing._LEADER_MARGIN_M = previous


def _limit_threads() -> None:
    try:
        import torch

        torch.set_num_threads(1)
    except Exception:
        return


def hover_wait_s(ticks: list[dict] | None) -> float | None:
    """每架「还没进目标球且几乎停住」的仿真时间，再对飞机取平均。

    速度 ≤ 0.2 m/s 且 goal_error > 0.4 m 才计入。步长用相邻拍的 t 差，缺第一拍时用 0.01 s。
    """
    if not ticks:
        return None
    by: dict[str, list[dict]] = {}
    for row in ticks:
        by.setdefault(str(row["uav"]), []).append(row)
    if not by:
        return None
    totals = []
    for rows in by.values():
        rows.sort(key=lambda item: float(item["t"]))
        acc = 0.0
        prev_t = None
        for row in rows:
            t = float(row["t"])
            vel = row["v"]
            speed = math.sqrt(sum(float(x) * float(x) for x in vel))
            waiting = speed <= HOVER_SPEED_MPS + 1e-12 and float(row["goal_error_m"]) > HOVER_GOAL_M
            if prev_t is None:
                dt = 0.01
            else:
                dt = t - prev_t
                if dt <= 0.0 or dt > 0.05:
                    dt = 0.01
            prev_t = t
            if waiting:
                acc += dt
        totals.append(acc)
    return sum(totals) / len(totals)


def relative_formation_p95(ticks: list[dict] | None, slots: dict[str, list]) -> float | None:
    """与门控无关的相对队形 p95。每 10 个物理拍采样一次。"""
    if not ticks:
        return None
    import numpy as np

    by_t: dict[float, list[dict]] = {}
    for row in ticks:
        by_t.setdefault(float(row["t"]), []).append(row)
    times = sorted(by_t)
    if not times:
        return None
    errs: list[float] = []
    for stamp in times[::10]:
        rows = by_t[stamp]
        if len(rows) < 2:
            continue
        implied = []
        for row in rows:
            slot = slots[str(row["uav"])]
            pos = row["p"]
            implied.append(
                (
                    float(pos[0]) - float(slot[0]),
                    float(pos[1]) - float(slot[1]),
                    float(pos[2]) - float(slot[2]),
                )
            )
        n = float(len(implied))
        center = tuple(sum(item[axis] for item in implied) / n for axis in range(3))
        for item in implied:
            errs.append(math.sqrt(sum((item[axis] - center[axis]) ** 2 for axis in range(3))))
    if not errs:
        return None
    return float(np.percentile(np.asarray(errs, dtype=float), 95))


def _decision_count(result: dict, key: str) -> int:
    block = result.get("crossing") or {}
    decisions = block.get("decisions") or result.get("plan_codes") or {}
    return int(decisions.get(key) or 0)


def _check_applied(arm: Arm, result: dict, applied_margin: float) -> None:
    block = result.get("crossing") or {}
    if bool(block.get("progress_gate")) is not arm.progress_gate:
        raise RuntimeError(
            f"{arm.arm_id} progress_gate 实际 {block.get('progress_gate')}，期望 {arm.progress_gate}"
        )
    got_pct = block.get("progress_percentile")
    if got_pct is None or abs(float(got_pct) - float(arm.progress_percentile)) > 1e-9:
        raise RuntimeError(f"{arm.arm_id} progress_percentile 实际 {got_pct}，期望 {arm.progress_percentile}")
    got_margin = block.get("leader_margin_m")
    if arm.progress_gate:
        if got_margin is None or abs(float(got_margin) - float(arm.leader_margin_m)) > 1e-9:
            raise RuntimeError(f"{arm.arm_id} leader_margin_m 实际 {got_margin}，期望 {arm.leader_margin_m}")
        if abs(float(got_margin) - float(applied_margin)) > 1e-9:
            raise RuntimeError(f"{arm.arm_id} 进程内 margin {applied_margin} 与 summary {got_margin} 不一致")
    elif got_margin is not None:
        raise RuntimeError(f"{arm.arm_id} 门控关时 summary 不应写入 leader_margin_m，收到 {got_margin}")


def _row(scene: str, arm: Arm, result: dict, *, applied_margin: float) -> dict:
    block = result.get("crossing") or {}
    dist = result.get("min_obstacle_distance_m")
    clearance = None if dist is None else float(dist) - BODY_R
    if block.get("min_obstacle_clearance_m") is not None:
        clearance = float(block["min_obstacle_clearance_m"])
    reason = result.get("reason") or ""
    outcome = result.get("outcome")
    tree = int(outcome == "COLLISION" and "TRUTH_POSITION" in str(reason))
    inter = int(outcome == "COLLISION" and "INTER_DRONE" in str(reason))
    return {
        "row_version": ROW_VERSION,
        "scene_kind": scene,
        "arm": arm.arm_id,
        "title": arm.title,
        "params": arm.params,
        "config_status": arm.config_status,
        "deviates_from_scene_card": arm.deviates,
        "report_only": True,
        "scene": result.get("scene"),
        "seed": int(result["seed"]),
        "outcome": outcome,
        "reason": reason,
        "arrived": int(result["arrived"]),
        "n_drones": int(result["n_drones"]),
        "arrival_rate": result.get("arrival_rate"),
        "arrival_time_s": result.get("arrival_time_s"),
        "tree_collisions": tree,
        "inter_drone_collisions": inter,
        "timeout": int(outcome == "TIMEOUT"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "min_obstacle_distance_m": dist,
        "min_clearance_m": clearance,
        "formation_error_p95_m": result.get("formation_error_p95_m"),
        "sim_duration_s": result.get("sim_duration_s"),
        "final_goal_error_m": result.get("final_goal_error_m"),
        "wall_time_s": result.get("wall_time_s"),
        "leader_hover": _decision_count(result, "LEADER_HOVER"),
        "leader_prefix": _decision_count(result, "LEADER_PREFIX"),
        "leader_hold": _decision_count(result, "LEADER_HOLD"),
        "decisions": dict(block.get("decisions") or {}),
        "progress_gate": bool(block.get("progress_gate")),
        "progress_percentile": block.get("progress_percentile"),
        "leader_margin_m": block.get("leader_margin_m"),
        "leader_margin_override_m": None if arm.leader_margin_m is None else float(arm.leader_margin_m),
        "leader_margin_applied_m": float(applied_margin),
        "gate_s_m": block.get("gate_s_m"),
        "scene_card_hash": result.get("scene_card_hash"),
        "commit": result.get("commit"),
        "empty_solution_count": result.get("empty_solution_count"),
    }


def _finish_row(row: dict, ticks: list[dict], slots: dict[str, list]) -> dict:
    out = dict(row)
    out.pop("_ticks_for_metrics", None)
    out["hover_wait_s"] = hover_wait_s(ticks)
    out["relative_formation_p95_m"] = relative_formation_p95(ticks, slots)
    return out


def _partial(scene: str, arm_id: str, seed: int) -> Path:
    return PARTIAL / f"{scene}-{arm_id}-{int(seed)}.json"


def run_one(scene: str, arm_id: str, seed: int, *, resume: bool) -> dict:
    dest = _partial(scene, arm_id, seed)
    if resume and dest.is_file():
        stored = json.loads(dest.read_text())
        if int(stored.get("row_version", -1)) == ROW_VERSION and "hover_wait_s" in stored:
            return stored
    arm = arm_by_id(arm_id)
    _limit_threads()
    card = build_card(scene, arm)
    slots = {str(drone["id"]): list(drone["slot_body"]) for drone in card.card["drones"]}
    from yopo_l0.runner import RunConfig, run

    t0 = time.perf_counter()
    with override_leader_margin(arm.leader_margin_m) as applied:
        out = run(card, RunConfig(seed=int(seed), shadow=True))
        _check_applied(arm, out.result, applied)
        drafted = _row(scene, arm, out.result, applied_margin=applied)
        ticks = out.ticks
    row = _finish_row(drafted, ticks, slots)
    row["worker_s"] = time.perf_counter() - t0
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False))
    tmp.replace(dest)
    print(
        f"{scene} {arm.arm_id} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"tree={row['tree_collisions']} inter={row['inter_drone_collisions']} "
        f"gap={row['min_inter_drone_distance_m']} clear={row['min_clearance_m']} "
        f"p95={row['formation_error_p95_m']} rel={row['relative_formation_p95_m']} "
        f"hover={row['leader_hover']} prefix={row['leader_prefix']} hold={row['leader_hold']} "
        f"wait={None if row['hover_wait_s'] is None else round(row['hover_wait_s'], 2)} "
        f"gate={row['gate_s_m']} margin={row['leader_margin_m']} "
        f"sim={row['sim_duration_s']} wall={row['wall_time_s']:.1f}s",
        flush=True,
    )
    return row


def _worker(spec: tuple) -> dict:
    scene, arm_id, seed, resume = spec
    return run_one(scene, arm_id, int(seed), resume=bool(resume))


def _specs(scenes: list[str], arms: list[str], seeds: list[int], resume: bool) -> list[tuple]:
    wanted_scenes = set(scenes)
    wanted_arms = set(arms)
    specs = []
    for arm in ARMS:
        if arm.arm_id not in wanted_arms:
            continue
        for scene in SCENES:
            if scene not in wanted_scenes:
                continue
            for seed in seeds:
                specs.append((scene, arm.arm_id, int(seed), resume))
    return specs


def _run_specs(specs: list[tuple], jobs: int) -> list[dict]:
    if jobs <= 1 or len(specs) <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)


def _nums(group: list[dict], key: str) -> list[float]:
    return [float(row[key]) for row in group if row.get(key) is not None]


def _same(group: list[dict], key: str):
    values = [row[key] for row in group]
    if any(value != values[0] for value in values):
        raise ValueError(f"{key} 在同一臂内不一致：{values}")
    return values[0]


def _mean(values: list[float]) -> float | None:
    if not values:
        return None
    return sum(values) / len(values)


def aggregate(rows: list[dict]) -> list[dict]:
    """按场景、臂汇总。缺臂就跳过，方便只跑了一部分时也能出表。"""
    out = []
    for scene in SCENES:
        for arm in ARMS:
            group = [row for row in rows if row["scene_kind"] == scene and row["arm"] == arm.arm_id]
            if not group:
                continue
            group = sorted(group, key=lambda row: int(row["seed"]))
            times = _nums(group, "arrival_time_s")
            gaps = _nums(group, "min_inter_drone_distance_m")
            clears = _nums(group, "min_clearance_m")
            hovers = [int(row["leader_hover"]) for row in group]
            prefixes = [int(row["leader_prefix"]) for row in group]
            holds = [int(row["leader_hold"]) for row in group]
            waits = _nums(group, "hover_wait_s")
            out.append(
                {
                    "scene": scene,
                    "arm": arm.arm_id,
                    "title": arm.title,
                    "params": _same(group, "params"),
                    "config_status": _same(group, "config_status"),
                    "seeds": [int(row["seed"]) for row in group],
                    "arrived": sum(int(row["arrived"]) for row in group),
                    "drones": sum(int(row["n_drones"]) for row in group),
                    "tree_collisions": sum(int(row["tree_collisions"]) for row in group),
                    "inter_drone_collisions": sum(int(row["inter_drone_collisions"]) for row in group),
                    "timeouts": sum(int(row["timeout"]) for row in group),
                    "formation_error_p95_m": _nums(group, "formation_error_p95_m"),
                    "relative_formation_p95_m": _nums(group, "relative_formation_p95_m"),
                    "min_inter_drone_distance_m": None if not gaps else min(gaps),
                    "min_inter_range_m": gaps,
                    "min_clearance_m": None if not clears else min(clears),
                    "clearance_range_m": clears,
                    "mean_arrival_time_s": _mean(times),
                    "arrival_time_n": len(times),
                    "n_runs": len(group),
                    "leader_hover_sum": sum(hovers),
                    "leader_prefix_sum": sum(prefixes),
                    "leader_hold_sum": sum(holds),
                    "mean_leader_hover": _mean([float(v) for v in hovers]),
                    "mean_leader_prefix": _mean([float(v) for v in prefixes]),
                    "mean_leader_hold": _mean([float(v) for v in holds]),
                    "mean_hover_wait_s": _mean(waits),
                    "gate_s_m": _nums(group, "gate_s_m"),
                }
            )
    return out


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


def _arrival_cell(summary: dict) -> str:
    if summary["mean_arrival_time_s"] is None:
        return "—"
    text = _f(summary["mean_arrival_time_s"], 2)
    if int(summary["arrival_time_n"]) != int(summary["n_runs"]):
        text += f"（{int(summary['arrival_time_n'])}/{int(summary['n_runs'])}）"
    return text


def _hit(tree: int, inter: int) -> str:
    if tree == 0 and inter == 0:
        return "0"
    parts = []
    if tree:
        parts.append(f"障碍 {tree}")
    if inter:
        parts.append(f"机间 {inter}")
    return "，".join(parts)


def format_summary_table(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 臂 | 参数 | 标注 | 到达 | 障碍碰撞 | 机间碰撞 | 超时 | 队形误差 p95 m | 相对队形 p95 m | 最小机间 m | 最小净空 m | 平均到达时间 s | 平均 LEADER_HOVER | 平均 LEADER_PREFIX | 平均 LEADER_HOLD | 平均悬停等待 s | 结束门控弧长 m |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for summary in aggregate(rows):
        lines.append(
            "| {scene} | {arm} | {params} | {status} | {arrived}/{drones} | {tree} | {inter} | {timeouts} | "
            "{p95} | {rel} | {gap} | {clear} | {arrival} | {hover} | {prefix} | {hold} | {wait} | {gate} |".format(
                scene=summary["scene"],
                arm=summary["arm"],
                params=summary["params"],
                status=summary["config_status"],
                arrived=summary["arrived"],
                drones=summary["drones"],
                tree=summary["tree_collisions"],
                inter=summary["inter_drone_collisions"],
                timeouts=summary["timeouts"],
                p95=_rng(summary["formation_error_p95_m"], 3),
                rel=_rng(summary["relative_formation_p95_m"], 3),
                gap=_rng(summary["min_inter_range_m"], 3),
                clear=_rng(summary["clearance_range_m"], 3),
                arrival=_arrival_cell(summary),
                hover=_f(summary["mean_leader_hover"], 1),
                prefix=_f(summary["mean_leader_prefix"], 1),
                hold=_f(summary["mean_leader_hold"], 1),
                wait=_f(summary["mean_hover_wait_s"], 2),
                gate=_rng(summary["gate_s_m"], 2),
            )
        )
    return "\n".join(lines)


def format_seed_table(rows: list[dict]) -> str:
    order = {arm.arm_id: index for index, arm in enumerate(ARMS)}
    scene_order = {name: index for index, name in enumerate(SCENES)}
    chosen = sorted(
        rows,
        key=lambda row: (scene_order[row["scene_kind"]], order[row["arm"]], int(row["seed"])),
    )
    lines = [
        "| 场景 | 臂 | seed | 到达 | 结果 | 碰撞 | 队形误差 p95 m | 相对队形 p95 m | 最小机间 m | 最小净空 m | 到达时间 s | LEADER_HOVER | LEADER_PREFIX | LEADER_HOLD | 悬停等待 s | 门控弧长 m |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for row in chosen:
        lines.append(
            "| {scene} | {arm} | {seed} | {arrived}/{n} | {outcome} | {hit} | {p95} | {rel} | {gap} | {clear} | "
            "{arrival} | {hover} | {prefix} | {hold} | {wait} | {gate} |".format(
                scene=row["scene_kind"],
                arm=row["arm"],
                seed=int(row["seed"]),
                arrived=int(row["arrived"]),
                n=int(row["n_drones"]),
                outcome=row["outcome"],
                hit=_hit(int(row["tree_collisions"]), int(row["inter_drone_collisions"])),
                p95=_f(row.get("formation_error_p95_m"), 3),
                rel=_f(row.get("relative_formation_p95_m"), 3),
                gap=_f(row.get("min_inter_drone_distance_m"), 3),
                clear=_f(row.get("min_clearance_m"), 3),
                arrival=_f(row.get("arrival_time_s"), 2),
                hover=int(row["leader_hover"]),
                prefix=int(row["leader_prefix"]),
                hold=int(row["leader_hold"]),
                wait=_f(row.get("hover_wait_s"), 2),
                gate=_f(row.get("gate_s_m"), 2),
            )
        )
    return "\n".join(lines)


def render_report_tables(rows: list[dict]) -> str:
    return "### 合计\n\n" + format_summary_table(rows) + "\n\n### 逐种子\n\n" + format_seed_table(rows)


def baseline_anchor_notes(rows: list[dict]) -> list[str]:
    """arm0 与 13e E+F / SUMMARY 逐种子对照。差一截就说明这台机器上的基线不是那一列。"""
    notes = []
    by = {(row["scene_kind"], row["arm"], int(row["seed"])): row for row in rows}

    def near(row: dict, key: str, expect: float, tol: float, label: str) -> None:
        got = row.get(key)
        ok = got is not None and abs(float(got) - float(expect)) <= tol
        notes.append(f"{'OK' if ok else 'DIFF'} {label}: got {got} expect {expect}")

    for scene, seeds in _OFF_ANCHORS.items():
        for seed, (arrived, gap, clear, p95, t_arr) in seeds.items():
            row = by.get((scene, "arm0", seed))
            if row is None:
                continue
            label = f"{scene} arm0 seed{seed}"
            ok_arr = int(row["arrived"]) == arrived and int(row["n_drones"]) == 6
            notes.append(
                f"{'OK' if ok_arr else 'DIFF'} {label} arrived {row['arrived']}/{row['n_drones']} "
                f"expect {arrived}/6 outcome {row['outcome']} reason {row.get('reason')}"
            )
            near(row, "min_inter_drone_distance_m", gap, 0.001, f"{label} gap")
            near(row, "min_clearance_m", clear, 0.001, f"{label} clearance")
            near(row, "formation_error_p95_m", p95, 0.001, f"{label} p95")
            near(row, "arrival_time_s", t_arr, 0.015, f"{label} arrival_time")
            tree = int(row.get("tree_collisions") or 0)
            inter = int(row.get("inter_drone_collisions") or 0)
            ok_col = tree == 0 and inter == 0
            notes.append(f"{'OK' if ok_col else 'DIFF'} {label} collisions tree={tree} inter={inter} expect 0 0")
    return notes


def _git_head() -> str:
    import subprocess

    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="E-PG1 六机林带进度门控消融（只报数）")
    parser.add_argument("--scenes", default="medium,sparse")
    parser.add_argument("--arms", default=",".join(arm.arm_id for arm in ARMS))
    parser.add_argument("--seeds", default="0,1,2,3,4")
    parser.add_argument("--jobs", type=int, default=3)
    parser.add_argument("--out", default="docs/research/e-forest-gate-ablation-runs.json")
    parser.add_argument("--no-resume", action="store_true")
    args = parser.parse_args(argv)
    scenes = [item for item in args.scenes.split(",") if item]
    arms = [item for item in args.arms.split(",") if item]
    seeds = [int(item) for item in args.seeds.split(",") if item != ""]
    unknown = [item for item in scenes if item not in SCENES] + [item for item in arms if item not in ARM_BY_ID]
    if unknown:
        raise SystemExit(f"未知场景或臂：{unknown}")
    if any(seed < 0 for seed in seeds):
        raise SystemExit(f"种子必须非负：{seeds}")
    before = watched_bytes()
    resume = not args.no_resume
    from yopo_swarm_core.baseline import fetch_upstream

    fetch_upstream()
    specs = _specs(scenes, arms, seeds, resume)
    rows = _run_specs(specs, int(args.jobs))
    after = watched_bytes()
    if before != after:
        changed = [rel for rel in _WATCHED if before[rel] != after[rel]]
        raise SystemExit(f"跑数改了磁盘文件：{changed}")
    arm_order = [arm.arm_id for arm in ARMS]
    rows.sort(key=lambda row: (SCENES.index(row["scene_kind"]), arm_order.index(row["arm"]), int(row["seed"])))
    notes = baseline_anchor_notes(rows)
    payload = {
        "ticket": "E-PG1",
        "report_only": True,
        "deviation_label": DEVIATION_LABEL,
        "planner_commit": _git_head(),
        "seeds": seeds,
        "scenes": scenes,
        "arms": [
            {
                "arm": arm.arm_id,
                "title": arm.title,
                "params": arm.params,
                "config_status": arm.config_status,
                "progress_gate": arm.progress_gate,
                "leader_margin_m": arm.leader_margin_m,
                "progress_percentile": arm.progress_percentile,
            }
            for arm in ARMS
            if arm.arm_id in arms
        ],
        "jobs": int(args.jobs),
        "metric_notes": {
            "arrival": "到达架次 / 该臂该场景全部种子的架次。",
            "collisions": "仿真因 COLLISION 停下才计。障碍看 TRUTH_POSITION，机间看 INTER_DRONE。一次运行最多记 1。",
            "timeout": "outcome 为 TIMEOUT 的种子数。",
            "formation_error_p95_m": "结果 formation_error_p95_m。门控开对着门控弧长上的槽位，门控关对着墙钟×speed。",
            "relative_formation_p95_m": "每 10 个物理拍，全队（位置−槽位）的平均为中心，各机到中心的距离取 p95。",
            "min_inter_drone_distance_m": "各种子最小机间中心距的最小值，单位 m。表里写范围，左端即这个最小值。",
            "min_clearance_m": "机体中心到障碍表面的距离减去 body_radius 0.35 m。",
            "mean_arrival_time_s": "6/6 完全到达的种子的 arrival_time_s 算术平均。没有完全到达则为 null。",
            "leader_counts": "LEADER_HOVER / LEADER_PREFIX / LEADER_HOLD 是提案窗口次数。平均是各种子次数的算术平均。",
            "hover_wait_s": "每架速度≤0.2 m/s 且离目标>0.4 m 的仿真时间，六架再平均，各种子再平均。",
            "gate_s_m": "结束时门控弧长。门控关为 null。航段长度 42 m。",
            "config_status": "progress_gate=true 的臂标 report-only / 偏离当前场景卡配置。",
        },
        "baseline_anchor_notes": notes,
        "summary": aggregate(rows),
        "rows": rows,
        "note": "整表 report-only。arm1–arm5 偏离当前场景卡（卡上 progress_gate=false）。margin 只改进程内全局，不写进场景卡。墙钟不进门。",
    }
    dest = Path(args.out)
    if not dest.is_absolute():
        dest = ROOT / dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    print(render_report_tables(rows))
    for line in notes:
        print(line)
    print(f"写入 {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
