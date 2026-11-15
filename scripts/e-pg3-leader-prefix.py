#!/usr/bin/env python3
"""票 E-PG3：超前机改走短前缀。只报数。

进程内读入 scenes/crossing-6-forest.json 与 scenes/crossing-6-forest-sparse.json。
不写场景 JSON，不改 core/、docs/tickets.md、README.md、scripts/premerge.sh、
scripts/regress.sh、scenes/suite.txt，也不改已冻门。

不重复 E-PG1 的余量 3/6/10 m 和分位 20，也不重复 E-PG2 的打开臂
（leader_margin_m=3.0、progress_percentile=20）。门控开的对照臂用 E-PG2 的缺省余量
1.0 m、分位 0。seeds 0–4 的这一格与 E-PG1 arm1 同配置，留在同一提交上当对照。

臂（两张卡、种子 0–9）：

  baseline   磁盘上的卡。progress_gate=false。当前场景卡配置。
  gate       progress_gate=true，余量缺省 1.0，分位 0，短前缀开关关。
             标「report-only / 偏离当前场景卡配置」。
  prefix03   门控开，再打开 leader_prefix，前缀 0.3 s。标「report-only / 偏离上游」。
  prefix07   同上，前缀 0.7 s。
  prefix10   同上，前缀 1.0 s。

影子盾开，只记录，不改批准动作。逐粒落在 out/e-pg3-leader-prefix/partial/，默认可续跑。
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# 只在直接跑这个脚本时把 BLAS 收成一线程。pytest 导入本文件时不要改环境。
if sys.argv and sys.argv[0].endswith("e-pg3-leader-prefix.py"):
    os.environ["OMP_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    os.environ["OPENBLAS_NUM_THREADS"] = "1"

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-pg3-leader-prefix" / "partial"
BODY_R = 0.35
ROW_VERSION = 1
UPSTREAM_LABEL = "report-only / 偏离上游"
CARD_LABEL = "report-only / 偏离当前场景卡配置"
BASELINE_LABEL = "当前场景卡配置"

_WATCHED = (
    "scenes/crossing-6-forest.json",
    "scenes/crossing-6-forest-sparse.json",
    "scenes/suite.txt",
    "yopo_l0/crossing.py",
    "yopo_l0/progress.py",
    "yopo_l0/runner.py",
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
PREFIX_LENGTHS_S = (0.3, 0.7, 1.0)

# 13e E+F / SUMMARY。只核对 baseline。
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

# E-PG1 arm1，seeds 0–4。到达、官方 p95、最小机间、最小净空、到达时间（超时为 None）、门控弧长。
# 这一格不重测余量，只用来看同一配置在本提交上是否还是那一列。
_GATE_ANCHORS = {
    "medium": {
        0: (2, 4.613, 1.030, 0.157, None, 40.66),
        1: (0, 4.666, 1.035, 0.149, None, 35.52),
        2: (0, 4.556, 1.065, 0.161, None, 34.85),
        3: (0, 4.179, 1.103, 0.527, None, 10.79),
        4: (0, 4.464, 1.153, 0.237, None, 35.91),
    },
    "sparse": {
        0: (6, 4.843, 1.007, 0.148, 47.78, 41.99),
        1: (2, 4.348, 0.996, 0.482, None, 41.98),
        2: (2, 4.658, 0.997, 0.489, None, 40.12),
        3: (0, 4.895, 1.046, 0.318, None, 38.82),
        4: (6, 4.852, 1.032, 0.386, 46.51, 41.98),
    },
}


@dataclass(frozen=True)
class Arm:
    arm_id: str
    title: str
    progress_gate: bool
    leader_prefix: bool
    leader_prefix_s: float | None

    @property
    def deviates_from_scene_card(self) -> bool:
        return self.progress_gate

    @property
    def upstream_deviation(self) -> bool:
        """短前缀开关打开才标偏离上游。门控本身是场景卡字段，不标这一列。"""
        return self.leader_prefix

    @property
    def config_status(self) -> str:
        if self.leader_prefix:
            return UPSTREAM_LABEL
        if self.progress_gate:
            return CARD_LABEL
        return BASELINE_LABEL

    @property
    def params(self) -> str:
        if not self.progress_gate and not self.leader_prefix:
            return "progress_gate=false"
        if self.leader_prefix:
            length = float(self.leader_prefix_s)
            return (
                "progress_gate=true, leader_margin_m=1.0, progress_percentile=0, "
                f"leader_prefix=true, leader_prefix_s={length:.1f}"
            )
        return "progress_gate=true, leader_margin_m=1.0, progress_percentile=0, leader_prefix=false"


def _prefix_arm(length: float) -> Arm:
    token = f"{length:.1f}".replace(".", "")
    return Arm(
        f"prefix{token}",
        f"门控开，短前缀 {length:.1f} s",
        True,
        True,
        float(length),
    )


ARMS = (
    Arm("baseline", "基线", False, False, None),
    Arm("gate", "门控开，缺省余量 1.0", True, False, None),
    *tuple(_prefix_arm(length) for length in PREFIX_LENGTHS_S),
)
ARM_BY_ID = {arm.arm_id: arm for arm in ARMS}


def arm_by_id(arm_id: str) -> Arm:
    try:
        return ARM_BY_ID[arm_id]
    except KeyError as exc:
        raise KeyError(f"未知臂 {arm_id}") from exc


def watched_bytes() -> dict[str, bytes]:
    return {rel: (ROOT / rel).read_bytes() for rel in _WATCHED}


def load_scene_raw(scene: str) -> dict:
    """磁盘上的六机林带卡必须仍是门控关，且没有余量和短前缀字段。"""
    if scene not in SCENE_CARDS:
        raise KeyError(f"未知场景 {scene}")
    path = SCENE_CARDS[scene]
    raw = json.loads(path.read_text())
    task = raw["task"]
    if task.get("progress_gate") is not False:
        raise RuntimeError(f"{path.name} 的 progress_gate 已不是 false，基线口径变了")
    if int(task.get("progress_percentile", 0)) != 0:
        raise RuntimeError(f"{path.name} 的 progress_percentile 已不是 0，基线口径变了")
    for key in ("leader_margin_m", "leader_prefix", "leader_prefix_s"):
        if key in task:
            raise RuntimeError(f"{path.name} 已写了 {key}，基线口径变了")
    return raw


def build_card(scene: str, arm: Arm | str):
    """返回 SceneCard。改动只在内存里。"""
    from yopo_l0.forest import generate_forest_geometry
    from yopo_l0.scene import SceneCard, load_card, parse_geometry

    if isinstance(arm, str):
        arm = arm_by_id(arm)
    path = SCENE_CARDS[scene]
    if not arm.progress_gate and not arm.leader_prefix:
        card = load_card(path)
        task = card.card["task"]
        if task["progress_gate"] is not False or "leader_prefix" in task or "leader_margin_m" in task:
            raise RuntimeError("load_card 后基线卡被改过")
        return card
    cloned = copy.deepcopy(load_scene_raw(scene))
    cloned["task"]["progress_gate"] = True
    if arm.leader_prefix:
        cloned["task"]["leader_prefix"] = True
        cloned["task"]["leader_prefix_s"] = float(arm.leader_prefix_s)
    geom = generate_forest_geometry(cloned["forest"])
    merged = {**cloned, "geometry": geom}
    return SceneCard(str(path), merged, parse_geometry(geom, merged.get("ground_z")))


def _limit_threads() -> None:
    try:
        import torch

        torch.set_num_threads(1)
    except Exception:
        return


def _decision_count(result: dict, key: str) -> int:
    block = result.get("crossing") or {}
    decisions = block.get("decisions") or result.get("plan_codes") or {}
    return int(decisions.get(key) or 0)


def _collision_flags(result: dict) -> tuple[int, int]:
    block = result.get("crossing") or {}
    if "obstacle_collisions" in block or "inter_drone_collisions" in block:
        return int(block.get("obstacle_collisions") or 0), int(block.get("inter_drone_collisions") or 0)
    reason = str(result.get("reason") or "")
    outcome = result.get("outcome")
    tree = int(outcome == "COLLISION" and "TRUTH_POSITION" in reason)
    inter = int(outcome == "COLLISION" and "INTER_DRONE" in reason)
    return tree, inter


def _check_applied(arm: Arm, result: dict) -> None:
    block = result.get("crossing") or {}
    if bool(block.get("progress_gate")) is not arm.progress_gate:
        raise RuntimeError(f"{arm.arm_id} progress_gate 实际 {block.get('progress_gate')}，期望 {arm.progress_gate}")
    if arm.progress_gate:
        got_margin = block.get("leader_margin_m")
        if got_margin is None or abs(float(got_margin) - 1.0) > 1e-9:
            raise RuntimeError(f"{arm.arm_id} 缺省余量应为 1.0，收到 {got_margin}")
        if abs(float(block.get("progress_percentile")) - 0.0) > 1e-9:
            raise RuntimeError(f"{arm.arm_id} 分位应为 0，收到 {block.get('progress_percentile')}")
    elif block.get("leader_margin_m") is not None:
        raise RuntimeError(f"{arm.arm_id} 门控关时不应写入 leader_margin_m")
    if arm.leader_prefix:
        if block.get("leader_prefix") is not True or result.get("leader_prefix") is not True:
            raise RuntimeError(f"{arm.arm_id} 短前缀开关没有标上")
        if abs(float(block.get("leader_prefix_s")) - float(arm.leader_prefix_s)) > 1e-9:
            raise RuntimeError(f"{arm.arm_id} 前缀长度实际 {block.get('leader_prefix_s')}")
        if result.get("upstream_deviation") is not True:
            raise RuntimeError(f"{arm.arm_id} 打开短前缀却没有标偏离上游")
    else:
        if "leader_prefix" in block or "leader_prefix" in result or "leader_prefix_s" in block:
            raise RuntimeError(f"{arm.arm_id} 开关关着，结果里却有短前缀字段")
        if result.get("upstream_deviation") is not False:
            raise RuntimeError(f"{arm.arm_id} 开关关着却标了偏离上游")


def _clearance(result: dict) -> float | None:
    block = result.get("crossing") or {}
    if block.get("min_obstacle_clearance_m") is not None:
        return float(block["min_obstacle_clearance_m"])
    dist = result.get("min_obstacle_distance_m")
    if dist is None:
        return None
    return float(dist) - BODY_R


def _row(scene: str, arm: Arm, result: dict) -> dict:
    tree, inter = _collision_flags(result)
    outcome = result.get("outcome")
    return {
        "row_version": ROW_VERSION,
        "scene_kind": scene,
        "arm": arm.arm_id,
        "title": arm.title,
        "params": arm.params,
        "config_status": arm.config_status,
        "deviates_from_scene_card": arm.deviates_from_scene_card,
        "upstream_deviation": bool(result.get("upstream_deviation")),
        "report_only": True,
        "scene": result.get("scene"),
        "seed": int(result["seed"]),
        "outcome": outcome,
        "reason": result.get("reason") or "",
        "arrived": int(result["arrived"]),
        "n_drones": int(result["n_drones"]),
        "arrival_rate": result.get("arrival_rate"),
        "arrival_time_s": result.get("arrival_time_s"),
        "tree_collisions": tree,
        "inter_drone_collisions": inter,
        "timeout": int(outcome == "TIMEOUT"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "min_clearance_m": _clearance(result),
        "formation_error_p95_m": result.get("formation_error_p95_m"),
        "sim_duration_s": result.get("sim_duration_s"),
        "final_goal_error_m": result.get("final_goal_error_m"),
        "wall_time_s": result.get("wall_time_s"),
        "leader_hover": _decision_count(result, "LEADER_HOVER"),
        "leader_prefix_count": _decision_count(result, "LEADER_PREFIX"),
        "leader_hold": _decision_count(result, "LEADER_HOLD"),
        "decisions": dict((result.get("crossing") or {}).get("decisions") or {}),
        "progress_gate": bool((result.get("crossing") or {}).get("progress_gate")),
        "progress_percentile": (result.get("crossing") or {}).get("progress_percentile"),
        "leader_margin_m": (result.get("crossing") or {}).get("leader_margin_m"),
        "leader_prefix_s": (result.get("crossing") or {}).get("leader_prefix_s"),
        "gate_s_m": (result.get("crossing") or {}).get("gate_s_m"),
        "scene_card_hash": result.get("scene_card_hash"),
        "commit": result.get("commit"),
        "empty_solution_count": result.get("empty_solution_count"),
    }


def _partial(scene: str, arm_id: str, seed: int) -> Path:
    return PARTIAL / f"{scene}-{arm_id}-{int(seed)}.json"


def run_one(scene: str, arm_id: str, seed: int, *, resume: bool) -> dict:
    dest = _partial(scene, arm_id, seed)
    if resume and dest.is_file():
        stored = json.loads(dest.read_text())
        if int(stored.get("row_version", -1)) == ROW_VERSION and "leader_prefix_count" in stored:
            return stored
    arm = arm_by_id(arm_id)
    _limit_threads()
    card = build_card(scene, arm)
    from yopo_l0.crossing import CrossingFleet
    from yopo_l0.runner import RunConfig, run

    fleet = CrossingFleet.from_card(card.card, 0.01)
    if fleet.leader_prefix_enabled is not arm.leader_prefix:
        raise RuntimeError(f"{arm.arm_id} 短前缀开关读成 {fleet.leader_prefix_enabled}")
    if arm.leader_prefix and abs(float(fleet.leader_prefix_s) - float(arm.leader_prefix_s)) > 1e-9:
        raise RuntimeError(f"{arm.arm_id} 前缀长度读成 {fleet.leader_prefix_s}")
    if arm.progress_gate and (not fleet.progress_gate or abs(float(fleet.effective_leader_margin_m()) - 1.0) > 1e-9):
        raise RuntimeError(f"{arm.arm_id} 门控或余量没有按缺省 1.0 打开")
    if not arm.progress_gate and (fleet.progress_gate or fleet.leader_prefix_enabled):
        raise RuntimeError("基线应是门控关、短前缀关")
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(seed), policy="crossing", shadow=True))
    _check_applied(arm, out.result)
    row = _row(scene, arm, out.result)
    row["worker_s"] = time.perf_counter() - t0
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False))
    tmp.replace(dest)
    print(
        f"{scene} {arm.arm_id} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"tree={row['tree_collisions']} inter={row['inter_drone_collisions']} timeout={row['timeout']} "
        f"gap={row['min_inter_drone_distance_m']} clear={row['min_clearance_m']} "
        f"p95={row['formation_error_p95_m']} hover={row['leader_hover']} prefix={row['leader_prefix_count']} "
        f"gate={row['gate_s_m']} dev={row['upstream_deviation']} "
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
            prefixes = [int(row["leader_prefix_count"]) for row in group]
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
                    "min_inter_drone_distance_m": None if not gaps else min(gaps),
                    "min_inter_range_m": gaps,
                    "min_clearance_m": None if not clears else min(clears),
                    "clearance_range_m": clears,
                    "mean_arrival_time_s": _mean(times),
                    "arrival_time_n": len(times),
                    "n_runs": len(group),
                    "leader_hover_sum": sum(hovers),
                    "mean_leader_hover": _mean([float(v) for v in hovers]),
                    "leader_prefix_sum": sum(prefixes),
                    "mean_leader_prefix": _mean([float(v) for v in prefixes]),
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


def format_summary_table(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 臂 | 参数 | 标注 | 到达 | 障碍碰撞 | 机间碰撞 | 超时 | 队形误差 p95 m | 最小机间 m | 最小净空 m | 平均到达时间 s | LEADER_HOVER | LEADER_PREFIX | 结束门控弧长 m |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for summary in aggregate(rows):
        lines.append(
            "| {scene} | {arm} | {params} | {status} | {arrived}/{drones} | {tree} | {inter} | {timeouts} | "
            "{p95} | {gap} | {clear} | {arrival} | {hover} | {prefix} | {gate} |".format(
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
                gap=_rng(summary["min_inter_range_m"], 3),
                clear=_rng(summary["clearance_range_m"], 3),
                arrival=_arrival_cell(summary),
                hover=_f(summary["mean_leader_hover"], 1),
                prefix=_f(summary["mean_leader_prefix"], 1),
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
        "| 场景 | 臂 | seed | 到达 | 结果 | 障碍碰撞 | 机间碰撞 | 超时 | 队形误差 p95 m | 最小机间 m | 最小净空 m | 到达时间 s | LEADER_HOVER | LEADER_PREFIX | 门控弧长 m |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for row in chosen:
        lines.append(
            "| {scene} | {arm} | {seed} | {arrived}/{n} | {outcome} | {tree} | {inter} | {timeout} | {p95} | {gap} | {clear} | "
            "{arrival} | {hover} | {prefix} | {gate} |".format(
                scene=row["scene_kind"],
                arm=row["arm"],
                seed=int(row["seed"]),
                arrived=int(row["arrived"]),
                n=int(row["n_drones"]),
                outcome=row["outcome"],
                tree=int(row["tree_collisions"]),
                inter=int(row["inter_drone_collisions"]),
                timeout=int(row["timeout"]),
                p95=_f(row.get("formation_error_p95_m"), 3),
                gap=_f(row.get("min_inter_drone_distance_m"), 3),
                clear=_f(row.get("min_clearance_m"), 3),
                arrival=_f(row.get("arrival_time_s"), 2),
                hover=int(row["leader_hover"]),
                prefix=int(row["leader_prefix_count"]),
                gate=_f(row.get("gate_s_m"), 2),
            )
        )
    return "\n".join(lines)


def render_report_tables(rows: list[dict]) -> str:
    return "### 合计\n\n" + format_summary_table(rows) + "\n\n### 逐种子\n\n" + format_seed_table(rows)


def _near(notes: list[str], row: dict, key: str, expect, tol: float, label: str) -> None:
    got = row.get(key)
    if expect is None:
        ok = got is None
    else:
        ok = got is not None and abs(float(got) - float(expect)) <= tol
    notes.append(f"{'OK' if ok else 'DIFF'} {label}: got {got} expect {expect}")


def baseline_anchor_notes(rows: list[dict]) -> list[str]:
    notes = []
    by = {(row["scene_kind"], row["arm"], int(row["seed"])): row for row in rows}
    for scene, seeds in _OFF_ANCHORS.items():
        for seed, (arrived, gap, clear, p95, t_arr) in seeds.items():
            row = by.get((scene, "baseline", seed))
            if row is None:
                continue
            label = f"{scene} baseline seed{seed}"
            ok_arr = int(row["arrived"]) == arrived and int(row["n_drones"]) == 6
            notes.append(
                f"{'OK' if ok_arr else 'DIFF'} {label} arrived {row['arrived']}/{row['n_drones']} "
                f"expect {arrived}/6 outcome {row['outcome']}"
            )
            _near(notes, row, "min_inter_drone_distance_m", gap, 0.001, f"{label} gap")
            _near(notes, row, "min_clearance_m", clear, 0.001, f"{label} clearance")
            _near(notes, row, "formation_error_p95_m", p95, 0.001, f"{label} p95")
            _near(notes, row, "arrival_time_s", t_arr, 0.015, f"{label} arrival_time")
    return notes


def gate_anchor_notes(rows: list[dict]) -> list[str]:
    """对照 E-PG1 arm1 的 seeds 0–4。对不上就照实写，不据此改表。"""
    notes = []
    by = {(row["scene_kind"], row["arm"], int(row["seed"])): row for row in rows}
    for scene, seeds in _GATE_ANCHORS.items():
        for seed, (arrived, p95, gap, clear, t_arr, gate_s) in seeds.items():
            row = by.get((scene, "gate", seed))
            if row is None:
                continue
            label = f"{scene} gate seed{seed}"
            ok_arr = int(row["arrived"]) == arrived
            notes.append(f"{'OK' if ok_arr else 'DIFF'} {label} arrived {row['arrived']} expect {arrived}")
            _near(notes, row, "formation_error_p95_m", p95, 0.001, f"{label} p95")
            _near(notes, row, "min_inter_drone_distance_m", gap, 0.001, f"{label} gap")
            _near(notes, row, "min_clearance_m", clear, 0.001, f"{label} clearance")
            _near(notes, row, "arrival_time_s", t_arr, 0.015, f"{label} arrival_time")
            _near(notes, row, "gate_s_m", gate_s, 0.02, f"{label} gate_s")
    return notes


def _git_head() -> str:
    import subprocess

    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()


def build_payload(rows: list[dict], *, seeds: list[int], scenes: list[str], arms: list[str], jobs: int) -> dict:
    return {
        "ticket": "E-PG3",
        "report_only": True,
        "title": "票 E-PG3：超前机改走短前缀，只报数",
        "note": (
            "整表 report-only。baseline 是当前场景卡。gate 偏离当前场景卡（progress_gate=false），"
            "余量是 E-PG2 缺省 1.0，分位 0，短前缀关。prefix 三臂打开 leader_prefix，标偏离上游。"
            "不改场景卡，不重复余量 3/6/10 和分位 20。墙钟不进门。"
        ),
        "upstream_label": UPSTREAM_LABEL,
        "planner_commit": _git_head(),
        "seeds": seeds,
        "scenes": scenes,
        "prefix_lengths_s": list(PREFIX_LENGTHS_S),
        "arms": [
            {
                "arm": arm.arm_id,
                "title": arm.title,
                "params": arm.params,
                "config_status": arm.config_status,
                "progress_gate": arm.progress_gate,
                "leader_prefix": arm.leader_prefix,
                "leader_prefix_s": arm.leader_prefix_s,
                "upstream_deviation": arm.upstream_deviation,
            }
            for arm in ARMS
            if arm.arm_id in arms
        ],
        "jobs": int(jobs),
        "metric_notes": {
            "arrival": "到达架次 / 该臂该场景全部种子的架次。六机 seeds 0–9 的分母是 60。",
            "collisions": "crossing 块里的 obstacle_collisions 与 inter_drone_collisions，按种子相加。一次运行各至多 1。",
            "timeout": "outcome 为 TIMEOUT 的种子数。",
            "formation_error_p95_m": "结果 formation_error_p95_m。门控开对着门控弧长上的槽位，门控关对着墙钟×speed。",
            "min_inter_drone_distance_m": "各种子最小机间中心距。表里写范围，左端是最小值。已冻门是 ≥ 0.70 m。",
            "min_clearance_m": "机体中心到障碍表面的距离减去 body_radius 0.35 m。",
            "mean_arrival_time_s": "6/6 完全到达的种子的 arrival_time_s 算术平均。没有完全到达则为 null。",
            "leader_prefix": "LEADER_PREFIX 是提案窗口次数。开关关时这一列应接近 0。",
            "config_status": "短前缀打开标 report-only / 偏离上游。只开门控、短前缀关，标偏离当前场景卡配置。",
        },
        "baseline_anchor_notes": baseline_anchor_notes(rows),
        "gate_anchor_notes": gate_anchor_notes(rows),
        "summary": aggregate(rows),
        "rows": rows,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="E-PG3 超前机短前缀（只报数）")
    parser.add_argument("--scenes", default="medium,sparse")
    parser.add_argument("--arms", default=",".join(arm.arm_id for arm in ARMS))
    parser.add_argument("--seeds", default="0,1,2,3,4,5,6,7,8,9")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--out", default="docs/research/e-pg3-leader-prefix-runs.json")
    parser.add_argument("--no-resume", action="store_true")
    parser.add_argument("--from-partial", action="store_true", help="只汇总已有 partial，不起新仿真")
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
    if args.from_partial:
        rows = []
        for scene, arm_id, seed, _resume in _specs(scenes, arms, seeds, True):
            path = _partial(scene, arm_id, seed)
            if not path.is_file():
                raise SystemExit(f"缺少 partial：{scene} {arm_id} seed={seed}")
            rows.append(json.loads(path.read_text()))
    else:
        from yopo_swarm_core.baseline import fetch_upstream

        fetch_upstream()
        specs = _specs(scenes, arms, seeds, resume)
        print(f"e-pg3 start n={len(specs)} jobs={args.jobs} resume={resume}", flush=True)
        rows = _run_specs(specs, int(args.jobs))
    after = watched_bytes()
    if before != after:
        changed = [rel for rel in _WATCHED if before[rel] != after[rel]]
        raise SystemExit(f"跑数改了磁盘文件：{changed}")
    arm_order = [arm.arm_id for arm in ARMS]
    rows.sort(key=lambda row: (SCENES.index(row["scene_kind"]), arm_order.index(row["arm"]), int(row["seed"])))
    payload = build_payload(rows, seeds=seeds, scenes=scenes, arms=arms, jobs=int(args.jobs))
    dest = Path(args.out)
    if not dest.is_absolute():
        dest = ROOT / dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    print(render_report_tables(rows))
    for line in payload["baseline_anchor_notes"]:
        print(line)
    for line in payload["gate_anchor_notes"]:
        print(line)
    print(f"写入 {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
