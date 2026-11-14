#!/usr/bin/env python3
"""票 E-PG2：六机林带进度门控的超前余量开关。只报数。

进程内读入 scenes/crossing-6-forest.json 与 scenes/crossing-6-forest-sparse.json，
在内存里改 task.progress_gate、task.leader_margin_m、task.progress_percentile。
不写场景 JSON，不改 core/、docs/tickets.md、README.md、scripts/premerge.sh、
scripts/regress.sh、scenes/suite.txt，也不改已冻门。

臂（两张卡、同一组种子 0–9）：

  baseline  磁盘上的卡。progress_gate=false。当前场景卡配置。
  open      progress_gate=true，leader_margin_m=3.0，progress_percentile=20。
            标「report-only / 偏离当前场景卡配置」。

余量从场景卡字段读，不再改模块全局。缺省仍是 1.0；本脚本的 open 臂显式写成 3.0。
影子盾开，只记录，不改批准动作。逐粒落在 out/e-pg2-forest-gate-switch/partial/，
默认可续跑。
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

# 只在直接跑这个脚本时把 BLAS 收成一线程。pytest 导入本文件时不要改环境，
# 免得和缺省 seed 0 的对照跑不在同一组线程上。spawn 出来的子进程继承这里设好的环境。
if sys.argv and sys.argv[0].endswith("e-pg2-forest-gate-switch.py"):
    os.environ["OMP_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    os.environ["OPENBLAS_NUM_THREADS"] = "1"

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-pg2-forest-gate-switch" / "partial"
BODY_R = 0.35
ROW_VERSION = 1
DEVIATION_LABEL = "report-only / 偏离当前场景卡配置"
BASELINE_LABEL = "当前场景卡配置"

_WATCHED = (
    "scenes/crossing-6-forest.json",
    "scenes/crossing-6-forest-sparse.json",
    "scenes/suite.txt",
    "yopo_l0/crossing.py",
    "yopo_l0/progress.py",
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

# 13e E+F / SUMMARY 记下的全关逐种子数。只核对 baseline，对不上就照实写。
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


@dataclass(frozen=True)
class Arm:
    arm_id: str
    title: str
    progress_gate: bool
    leader_margin_m: float | None
    progress_percentile: float

    @property
    def deviates(self) -> bool:
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
        margin = float(self.leader_margin_m)
        margin_txt = f"{margin:.1f}"
        return (
            f"progress_gate=true, leader_margin_m={margin_txt}, progress_percentile={pct_txt}"
        )


ARMS = (
    Arm("baseline", "基线", False, None, 0.0),
    Arm("open", "门控开余量 3.0 分位 20", True, 3.0, 20.0),
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
    """读磁盘上的场景卡。六机林带卡必须仍是 progress_gate=false，且没有 leader_margin_m。"""
    if scene not in SCENE_CARDS:
        raise KeyError(f"未知场景 {scene}")
    path = SCENE_CARDS[scene]
    raw = json.loads(path.read_text())
    task = raw["task"]
    if task.get("progress_gate") is not False:
        raise RuntimeError(f"{path.name} 的 progress_gate 已不是 false，基线口径变了")
    if int(task.get("progress_percentile", 0)) != 0:
        raise RuntimeError(f"{path.name} 的 progress_percentile 已不是 0，基线口径变了")
    if "leader_margin_m" in task:
        raise RuntimeError(f"{path.name} 已写了 leader_margin_m，基线口径变了")
    return raw


def build_card(scene: str, arm: Arm | str):
    """返回 SceneCard。open 只改内存里的 task，磁盘上的卡仍是 load_scene_raw 读到的那份。"""
    from yopo_l0.forest import generate_forest_geometry
    from yopo_l0.scene import SceneCard, load_card, parse_geometry

    if isinstance(arm, str):
        arm = arm_by_id(arm)
    path = SCENE_CARDS[scene]
    if not arm.deviates:
        card = load_card(path)
        task = card.card["task"]
        if task["progress_gate"] is not False or "leader_margin_m" in task:
            raise RuntimeError("load_card 后基线卡被改过")
        return card
    cloned = copy.deepcopy(load_scene_raw(scene))
    pct = float(arm.progress_percentile)
    cloned["task"]["progress_gate"] = True
    cloned["task"]["leader_margin_m"] = float(arm.leader_margin_m)
    cloned["task"]["progress_percentile"] = int(pct) if pct.is_integer() else pct
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
        raise RuntimeError(
            f"{arm.arm_id} progress_gate 实际 {block.get('progress_gate')}，期望 {arm.progress_gate}"
        )
    got_pct = block.get("progress_percentile")
    expect_pct = 0.0 if not arm.progress_gate else float(arm.progress_percentile)
    if got_pct is None or abs(float(got_pct) - expect_pct) > 1e-9:
        raise RuntimeError(f"{arm.arm_id} progress_percentile 实际 {got_pct}，期望 {expect_pct}")
    got_margin = block.get("leader_margin_m")
    if arm.progress_gate:
        if got_margin is None or abs(float(got_margin) - float(arm.leader_margin_m)) > 1e-9:
            raise RuntimeError(f"{arm.arm_id} leader_margin_m 实际 {got_margin}，期望 {arm.leader_margin_m}")
    elif got_margin is not None:
        raise RuntimeError(f"{arm.arm_id} 门控关时 summary 不应写入 leader_margin_m，收到 {got_margin}")


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
        "deviates_from_scene_card": arm.deviates,
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
        "decisions": dict((result.get("crossing") or {}).get("decisions") or {}),
        "progress_gate": bool((result.get("crossing") or {}).get("progress_gate")),
        "progress_percentile": (result.get("crossing") or {}).get("progress_percentile"),
        "leader_margin_m": (result.get("crossing") or {}).get("leader_margin_m"),
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
        if int(stored.get("row_version", -1)) == ROW_VERSION and "leader_hover" in stored:
            return stored
    arm = arm_by_id(arm_id)
    _limit_threads()
    card = build_card(scene, arm)
    from yopo_l0.crossing import CrossingFleet
    from yopo_l0.runner import RunConfig, run

    fleet = CrossingFleet.from_card(card.card, 0.01)
    if arm.progress_gate:
        if fleet.leader_margin_m != float(arm.leader_margin_m) or fleet.progress_percentile != float(arm.progress_percentile):
            raise RuntimeError(
                f"{arm.arm_id} 没有读到卡上的余量或分位："
                f"margin={fleet.leader_margin_m} pct={fleet.progress_percentile}"
            )
    elif fleet.leader_margin_m != 1.0 or fleet.progress_gate:
        raise RuntimeError(f"基线应是缺省余量 1.0 且门控关，收到 margin={fleet.leader_margin_m} gate={fleet.progress_gate}")
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
        f"p95={row['formation_error_p95_m']} hover={row['leader_hover']} "
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
    """按场景、臂汇总。缺臂就跳过。"""
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
        "| 场景 | 臂 | 参数 | 标注 | 到达 | 障碍碰撞 | 机间碰撞 | 超时 | 队形误差 p95 m | 最小机间 m | 最小净空 m | 平均到达时间 s | LEADER_HOVER | 结束门控弧长 m |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for summary in aggregate(rows):
        lines.append(
            "| {scene} | {arm} | {params} | {status} | {arrived}/{drones} | {tree} | {inter} | {timeouts} | "
            "{p95} | {gap} | {clear} | {arrival} | {hover} | {gate} |".format(
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
        "| 场景 | 臂 | seed | 到达 | 结果 | 障碍碰撞 | 机间碰撞 | 超时 | 队形误差 p95 m | 最小机间 m | 最小净空 m | 到达时间 s | LEADER_HOVER | 门控弧长 m |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for row in chosen:
        lines.append(
            "| {scene} | {arm} | {seed} | {arrived}/{n} | {outcome} | {tree} | {inter} | {timeout} | {p95} | {gap} | {clear} | "
            "{arrival} | {hover} | {gate} |".format(
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
                gate=_f(row.get("gate_s_m"), 2),
            )
        )
    return "\n".join(lines)


def render_report_tables(rows: list[dict]) -> str:
    return "### 合计\n\n" + format_summary_table(rows) + "\n\n### 逐种子\n\n" + format_seed_table(rows)


def baseline_anchor_notes(rows: list[dict]) -> list[str]:
    """baseline 与 13e E+F / SUMMARY 逐种子对照。差一截就说明这台机器上的基线不是那一列。"""
    notes = []
    by = {(row["scene_kind"], row["arm"], int(row["seed"])): row for row in rows}

    def near(row: dict, key: str, expect: float, tol: float, label: str) -> None:
        got = row.get(key)
        ok = got is not None and abs(float(got) - float(expect)) <= tol
        notes.append(f"{'OK' if ok else 'DIFF'} {label}: got {got} expect {expect}")

    for scene, seeds in _OFF_ANCHORS.items():
        for seed, (arrived, gap, clear, p95, t_arr) in seeds.items():
            row = by.get((scene, "baseline", seed))
            if row is None:
                continue
            label = f"{scene} baseline seed{seed}"
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


def build_payload(rows: list[dict], *, seeds: list[int], scenes: list[str], arms: list[str], jobs: int) -> dict:
    notes = baseline_anchor_notes(rows)
    return {
        "ticket": "E-PG2",
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
        "jobs": int(jobs),
        "metric_notes": {
            "arrival": "到达架次 / 该臂该场景全部种子的架次。六机 seeds 0–9 的分母是 60。",
            "collisions": "crossing 块里的 obstacle_collisions 与 inter_drone_collisions，按种子相加。一次运行各至多 1。",
            "timeout": "outcome 为 TIMEOUT 的种子数。",
            "formation_error_p95_m": "结果 formation_error_p95_m。门控开对着门控弧长上的槽位，门控关对着墙钟×speed。六机林带卡没有把这一列冻成门。三机穿林卡的队形门是 6.0 m。",
            "min_inter_drone_distance_m": "各种子最小机间中心距的最小值，单位 m。表里写范围，左端即这个最小值。已冻门是 ≥ 0.70 m。",
            "min_clearance_m": "机体中心到障碍表面的距离减去 body_radius 0.35 m。",
            "mean_arrival_time_s": "6/6 完全到达的种子的 arrival_time_s 算术平均。没有完全到达则为 null。",
            "leader_hover": "LEADER_HOVER 是提案窗口次数。表上的平均数是各种子次数的算术平均。",
            "gate_s_m": "结束时门控弧长。门控关为 null。航段长度 42 m。",
            "config_status": "progress_gate=true 的臂标 report-only / 偏离当前场景卡配置。",
        },
        "baseline_anchor_notes": notes,
        "summary": aggregate(rows),
        "rows": rows,
        "note": "整表 report-only。open 偏离当前场景卡（卡上 progress_gate=false，没有 leader_margin_m）。余量写在进程内的 task.leader_margin_m，不写回场景卡。墙钟不进门。",
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="E-PG2 六机林带进度门控余量开关（只报数）")
    parser.add_argument("--scenes", default="medium,sparse")
    parser.add_argument("--arms", default=",".join(arm.arm_id for arm in ARMS))
    parser.add_argument("--seeds", default="0,1,2,3,4,5,6,7,8,9")
    parser.add_argument("--jobs", type=int, default=3)
    parser.add_argument("--out", default="docs/research/e-pg2-forest-gate-switch-runs.json")
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
            row = json.loads(_partial(scene, arm_id, seed).read_text()) if _partial(scene, arm_id, seed).is_file() else None
            if row is None:
                raise SystemExit(f"缺少 partial：{scene} {arm_id} seed={seed}")
            rows.append(row)
    else:
        from yopo_swarm_core.baseline import fetch_upstream

        fetch_upstream()
        specs = _specs(scenes, arms, seeds, resume)
        print(
            f"e-pg2 start n={len(specs)} jobs={args.jobs} resume={resume} label={DEVIATION_LABEL}",
            flush=True,
        )
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
    print(f"写入 {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
