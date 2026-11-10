"""票 11c：六机楔形穿林撞树定位。只记录，不改 CrossingFleet 的选择。

每个失败 seed 归一条，按这个顺序（先看当时在飞的那条计划）：

1. 预约总线让行停在树旁：在飞的是冲突刹车（邻机预留把 YOPO 候选挡住之后承诺的刹车）。
2. 无解悬停：在飞的是走廊空 / 刹车 / 运动学退回，不是让行。
3. 跟踪滞后：参考点离障碍仍大于机体半径，机体进了半径。
4. 矮枝/斜干不在深度图里。
5. 队形目标点落进树：这条计划用的目标点离树不超过机体半径，计划本身又进了机体。
6. 走廊准入过松：目标不在树里，但 0.05 m 走廊准入的计划进了机体。
7. 其他。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from yopo_swarm_core.candidates import (
    UPSTREAM_DECODER,
    corridor_mu_lo,
    decode_params,
    rank_candidates,
)
from yopo_swarm_core.plan import state_at

from .crossing import CrossingFleet
from .dense_diag import (
    _FLY_CODES,
    _HOVER_CODES,
    _plan_clearance,
    _tree_kind,
    depth_sees,
    implied_radius_m,
    nearest_tree,
)
from .policy import R_yaw
from .runner import RunConfig, run
from .scene import SceneCard, load_card
from .sensor import CameraConfig

CAT_YIELD = "预约总线让行停在树旁"
CAT_HOVER = "无解悬停"
CAT_TRACK = "跟踪滞后"
CAT_HIDDEN = "矮枝/斜干不在深度图里"
CAT_GOAL = "队形目标点落进树"
CAT_CORRIDOR = "走廊准入过松"
CAT_OTHER = "其他"
CATEGORIES = (CAT_YIELD, CAT_HOVER, CAT_TRACK, CAT_HIDDEN, CAT_GOAL, CAT_CORRIDOR, CAT_OTHER)

_HOVER_NO_YIELD = _HOVER_CODES - {"CONFLICT_BRAKE"}


def classify(facts: dict) -> str:
    """facts 见 diagnose_output。顺序就是模块说明里的七类。"""
    if facts["plan_code"] == "CONFLICT_BRAKE":
        return CAT_YIELD
    if facts["plan_code"] in _HOVER_NO_YIELD:
        return CAT_HOVER
    if facts["plan_clearance_m"] > facts["body_radius_m"] and facts["actual_clearance_m"] <= facts["body_radius_m"]:
        return CAT_TRACK
    if facts["kind"] in ("branch", "lean") and not facts["in_depth"]:
        return CAT_HIDDEN
    goal_in_tree = (
        facts.get("goal_clearance_m") is not None and facts["goal_clearance_m"] <= facts["body_radius_m"]
    )
    clips = (
        facts["plan_clearance_m"] <= facts["body_radius_m"]
        and facts.get("admitted")
        and facts["plan_code"] in _FLY_CODES
    )
    if clips and goal_in_tree:
        return CAT_GOAL
    if clips:
        return CAT_CORRIDOR
    return CAT_OTHER


def _tree_clearance(obstacles, point) -> float:
    _ob, dist = nearest_tree(obstacles, np.asarray(point, dtype=float))
    return dist


class _DiagFleet(CrossingFleet):
    """多记每次提案的目标点和拒绝原因。选择仍走 CrossingFleet._planner。"""

    def __init__(self, card, physics_dt, *, record_inputs: bool):
        super().__init__(card, physics_dt, record_inputs=record_inputs)
        self.trace: list[dict] = []
        self._now = 0.0
        self._t0 = 0.0
        self._goal = None
        self._reject = None
        self._saw_conflict = False
        self._pose: dict[str, tuple] = {}

    def ingest(self, drone_id, frame, state):
        super().ingest(drone_id, frame, state)
        self._pose[drone_id] = (
            np.array(state.p, dtype=float, copy=True),
            float(state.yaw),
            frame,
        )

    def goal_for(self, drone_id, t, head_p):
        goal = super().goal_for(drone_id, t, head_p)
        self._goal = np.array(goal, dtype=float, copy=True)
        return goal

    def _as_plan(self, node, t_start, head, motion):
        occ, reject = super()._as_plan(node, t_start, head, motion)
        if reject:
            self._reject = reject
        if reject == "CONFLICT_SKIP":
            self._saw_conflict = True
        return occ, reject

    def _planner(self, node, now, t_start):
        self._now = float(now)
        self._t0 = float(t_start)
        self._goal = None
        self._reject = None
        self._saw_conflict = False
        return super()._planner(node, now, t_start)

    def _count(self, drone_id, code):
        super()._count(drone_id, code)
        pose = self._pose.get(drone_id)
        self.trace.append({
            "drone": drone_id,
            "now": self._now,
            "t_start": self._t0,
            "code": code,
            "reject": self._reject,
            "saw_conflict": self._saw_conflict,
            "goal": None if self._goal is None else self._goal.tolist(),
            "p": None if pose is None else pose[0].tolist(),
            "yaw": None if pose is None else pose[1],
            "frame": None if pose is None else pose[2],
        })


def _install(fleet_cls):
    import yopo_l0.crossing as crossing

    previous = crossing.CrossingFleet
    crossing.CrossingFleet = fleet_cls
    return previous


_DiagFleet._last = None


def _capture_init(base_init):
    def wrapped(self, *args, **kwargs):
        base_init(self, *args, **kwargs)
        if isinstance(self, _DiagFleet):
            _DiagFleet._last = self
    return wrapped


_DiagFleet.__init__ = _capture_init(_DiagFleet.__init__)


def _admission(source, head, goal, depth, yaw) -> dict:
    params, score, radius = source.forward_raw(depth, head[1], head[2], goal, head[0], R_yaw(yaw))
    inner_b, _tail_b, _durations = decode_params(params, UPSTREAM_DECODER)
    inner_w = inner_b @ R_yaw(yaw).T + head[0]
    ranking = rank_candidates(score, radius, UPSTREAM_DECODER, inner_w)
    mu = corridor_mu_lo(radius, UPSTREAM_DECODER)
    admitted = np.asarray(ranking.admitted, dtype=bool)
    n_adm = int(admitted.sum())
    selected = None if ranking.corridor_empty or not ranking.order else int(ranking.order[0])
    mu_sel = None if selected is None else float(mu[selected])
    return {
        "n_candidates": int(UPSTREAM_DECODER.count),
        "n_admitted": n_adm,
        "mu_lo_selected": mu_sel,
        "implied_r_m": None if mu_sel is None else implied_radius_m(mu_sel),
        "admitted": n_adm > 0 and not bool(ranking.corridor_empty),
    }


def _executing_row(trace, drone: str, t_hit: float):
    active = [
        row for row in trace
        if row["drone"] == drone and row["t_start"] <= t_hit + 1e-9 and row["code"] in _FLY_CODES | _HOVER_CODES
    ]
    return active[-1] if active else None


def _clearance(result: dict, body: float):
    dist = result.get("min_obstacle_distance_m")
    if dist is None:
        return None
    return float(dist) - body


def _frame_table(rows: list[dict], t_hit: float) -> list[dict]:
    """撞前 2 s 的仿真帧，大约每 0.5 s 留一帧，外加撞上的那一帧。"""
    picked = []
    next_t = t_hit - 2.0
    for row in rows:
        t = float(row["t"])
        if t < t_hit - 2.0 - 1e-9:
            continue
        if t + 1e-9 >= t_hit or t + 1e-9 >= next_t:
            p = np.asarray(row["p"], dtype=float)
            cmd = np.asarray(row.get("cmd_p") or p, dtype=float)
            picked.append({
                "t": round(t, 3),
                "p": [round(float(x), 3) for x in p],
                "cmd_p": [round(float(x), 3) for x in cmd],
                "track_m": round(float(np.linalg.norm(p - cmd)), 3),
                "speed_mps": round(float(np.linalg.norm(row["v"])), 3),
                "min_obstacle_m": round(float(row["min_obstacle_m"]), 3),
                "plan_code": row.get("plan_code"),
                "formation_error_m": None if row.get("formation_error_m") is None else round(float(row["formation_error_m"]), 3),
            })
            next_t = t + 0.5
    return picked


def _abort_reasons(fleet, drone: str) -> dict:
    counts: dict[str, int] = {}
    for event in fleet.nodes[drone].events:
        if event.get("event") != "abort":
            continue
        reason = str(event.get("reason"))
        counts[reason] = counts.get(reason, 0) + 1
    return counts


def _metrics(result: dict, body: float) -> dict:
    return {
        "seed": result["seed"],
        "outcome": result["outcome"],
        "reason": result.get("reason"),
        "arrived": result.get("arrived"),
        "arrival_rate": result.get("arrival_rate"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "min_clearance_m": _clearance(result, body),
        "formation_error_p95_m": result.get("formation_error_p95_m"),
        "plan_codes": result.get("plan_codes"),
        "wall_time_s": result.get("wall_time_s"),
        "sim_duration_s": result.get("sim_duration_s"),
        "inter_drone_collisions": (result.get("crossing") or {}).get("inter_drone_collisions"),
        "obstacle_collisions": (result.get("crossing") or {}).get("obstacle_collisions"),
    }


def diagnose_output(card: SceneCard, out, fleet: _DiagFleet) -> dict:
    result = out.result
    body = float(card.card["criteria"]["body_radius_m"])
    base = _metrics(result, body)
    reason = str(result.get("reason") or "")
    if result["outcome"] != "COLLISION":
        base.update({
            "drone": None,
            "category": None,
            "window": [],
            "frames": [],
        })
        return base
    drone = reason.split(":", 1)[0] if ":" in reason else "uav0"
    rows = [row for row in out.ticks if row["uav"] == drone]
    hit = rows[-1]
    t_hit = float(hit["t"])
    p_hit = np.asarray(hit["p"], dtype=float)
    obstacle, actual = nearest_tree(card.obstacles, p_hit)
    plan = fleet.last_record(drone).occupancy.plan
    if obstacle is None:
        plan_clear, plan_p = math.inf, p_hit
    else:
        plan_clear, plan_p = _plan_clearance(plan, obstacle, max(0.0, t_hit - 0.5), t_hit)
    flown = _executing_row(fleet.trace, drone, t_hit)
    window = [
        row for row in fleet.trace
        if row["drone"] == drone and t_hit - 2.0 <= row["now"] <= t_hit + 1e-9
    ]
    if flown is not None and all(row is not flown for row in window):
        window = [flown, *window]
    admission = {"n_candidates": None, "n_admitted": None, "mu_lo_selected": None, "implied_r_m": None, "admitted": False}
    in_depth = False
    goal_clear = None
    if flown and flown.get("goal") is not None:
        goal_clear = _tree_clearance(card.obstacles, flown["goal"])
    if flown and flown["frame"] is not None and flown["p"] is not None and flown["goal"] is not None:
        if obstacle is not None:
            cam = CameraConfig.from_card(card.card["camera"])
            in_depth = depth_sees(
                flown["frame"], np.asarray(flown["p"], dtype=float), float(flown["yaw"]), cam, obstacle
            )
        head = np.array(state_at(plan, float(flown["t_start"])), dtype=float)
        try:
            admission = _admission(
                fleet.source, head, np.asarray(flown["goal"], dtype=float), flown["frame"].depth, float(flown["yaw"])
            )
        except (ValueError, np.linalg.LinAlgError, FloatingPointError):
            pass
    ref_clear = _tree_clearance(card.obstacles, fleet.rail.position(t_hit, fleet.t_go, fleet.slots[drone]))
    cmd = np.asarray(hit.get("cmd_p") or p_hit, dtype=float)
    cmd_clear = None if obstacle is None else float(obstacle.distance(cmd))
    speeds = [
        float(np.linalg.norm(row["v"]))
        for row in rows
        if t_hit - 1.0 <= float(row["t"]) <= t_hit + 1e-9
    ]
    facts = {
        **base,
        "drone": drone,
        "t_hit_s": t_hit,
        "kind": None if obstacle is None else _tree_kind(obstacle),
        "obstacle": None if obstacle is None else obstacle.name,
        "actual_clearance_m": actual,
        "plan_clearance_m": plan_clear,
        "tracking_error_m": float(np.linalg.norm(p_hit - plan_p)),
        "cmd_track_m": float(np.linalg.norm(p_hit - cmd)),
        "cmd_p": [round(float(x), 3) for x in cmd],
        "cmd_clearance_m": cmd_clear,
        "plan_t_end": float(plan.t_end),
        "past_plan": bool(t_hit > float(plan.t_end) + 1e-9),
        "in_depth": in_depth,
        "plan_code": None if flown is None else flown["code"],
        "reject": None if flown is None else flown.get("reject"),
        "saw_conflict": bool(flown and flown.get("saw_conflict")),
        "admitted": bool(admission.get("admitted")),
        "goal": None if flown is None else flown.get("goal"),
        "goal_clearance_m": goal_clear,
        "slot_clearance_m": ref_clear,
        "formation_error_m": hit.get("formation_error_m"),
        "speed_mps": float(np.linalg.norm(hit["v"])),
        "speed_p50_last_1s": None if not speeds else float(np.median(speeds)),
        "mu_lo_selected": admission.get("mu_lo_selected"),
        "implied_r_m": admission.get("implied_r_m"),
        "n_admitted": admission.get("n_admitted"),
        "n_candidates": admission.get("n_candidates"),
        "body_radius_m": body,
        "p": [round(float(x), 3) for x in p_hit],
        "drone_codes": dict(fleet._codes.get(drone, {})),
        "window": window,
        "frames": _frame_table(rows, t_hit),
        "aborts": _abort_reasons(fleet, drone),
    }
    facts["category"] = classify(facts)
    return facts


def _fmt(x, nd=3):
    return "—" if x is None else f"{float(x):.{nd}f}"


def window_table(facts: dict) -> str:
    if facts.get("category") is None:
        return (
            f"seed {facts['seed']}  {facts['outcome']}  {facts.get('reason') or ''}  "
            f"到达 {facts.get('arrived')}  净空 {_fmt(facts.get('min_clearance_m'))} m  "
            f"机间 {_fmt(facts.get('min_inter_drone_distance_m'))} m  "
            f"队形 p95 {_fmt(facts.get('formation_error_p95_m'))} m"
        )
    lines = [
        f"seed {facts['seed']}  t={facts['t_hit_s']:.2f}s  {facts['reason']}  "
        f"障碍 {facts['obstacle']}（{facts['kind']}）  类别：{facts['category']}",
        "",
        "| t | t_start | 规划码 | 拒绝 | 让行过 |",
        "|---|---|---|---|---|",
    ]
    for row in facts["window"]:
        lines.append(
            f"| {row['now']:.2f} | {row['t_start']:.2f} | {row['code']} | {row.get('reject') or '—'} | "
            f"{'是' if row.get('saw_conflict') else '否'} |"
        )
    if not facts["window"]:
        lines.append("| — | — | — | — |")
    lines.append("")
    lines.append(
        f"计划净空 {_fmt(facts['plan_clearance_m'])} m，机体净空 {_fmt(facts['actual_clearance_m'])} m，"
        f"跟踪差 {_fmt(facts['tracking_error_m'])} m，对参考点 {_fmt(facts.get('cmd_track_m'))} m"
        f"（参考 {facts.get('cmd_p')}），目标净空 {_fmt(facts['goal_clearance_m'])} m，"
        f"当时槽位净空 {_fmt(facts['slot_clearance_m'])} m，队形误差 {_fmt(facts['formation_error_m'])} m，"
        f"速度 {_fmt(facts['speed_mps'], 2)} m/s（前 1 s 中位 {_fmt(facts['speed_p50_last_1s'], 2)}），"
        f"深度里{'有' if facts['in_depth'] else '没有'}这件障碍，在飞计划 {facts['plan_code']}，"
        f"计划结束 t={_fmt(facts.get('plan_t_end'), 2)}，"
        f"{'已经进入终端保持' if facts.get('past_plan') else '仍在运动段'}，"
        f"参考点净空 {_fmt(facts.get('cmd_clearance_m'))} m，"
        f"准入 {facts['n_admitted']}/{facts['n_candidates']}，选中 μ_lo {_fmt(facts['mu_lo_selected'], 4)}，"
        f"折合半径 {_fmt(facts['implied_r_m'])} m。位置 {facts['p']}。"
    )
    lines.append("")
    lines.append("| t | 位置 | 参考 | 对参考 | 速度 | 障碍距离 | 规划码 | 队形误差 |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for frame in facts["frames"]:
        lines.append(
            f"| {frame['t']:.2f} | {frame['p']} | {frame['cmd_p']} | {frame['track_m']:.3f} | "
            f"{frame['speed_mps']:.2f} | {frame['min_obstacle_m']:.3f} | {frame['plan_code']} | "
            f"{frame['formation_error_m']} |"
        )
    lines.append("")
    lines.append(
        f"本机规划码累计 {facts['drone_codes']}，全体 {facts.get('plan_codes')}，中止 {facts['aborts'] or '无'}。"
    )
    return "\n".join(lines)


def _public_facts(facts: dict) -> dict:
    window = [{k: v for k, v in row.items() if k != "frame"} for row in facts.get("window") or []]
    out = {k: v for k, v in facts.items() if k != "window"}
    out["window"] = window
    return out


def summarize(rows: list[dict], *, title: str) -> str:
    counts = {name: 0 for name in CATEGORIES}
    for row in rows:
        if row.get("category") in counts:
            counts[row["category"]] += 1
    lines = [
        f"# {title}",
        "",
        "规划器没有改。影子盾关。类别按在飞计划归一条。",
        "目标净空是该次提案送进 YOPO 的目标点到最近树表面的距离。折合半径是 μ 的反函数，不是真值净空。",
        "",
        "| 类别 | 次数 |",
        "|---|---|",
    ]
    for name in CATEGORIES:
        lines.append(f"| {name} | {counts[name]} |")
    failed = [row for row in rows if row.get("outcome") == "COLLISION"]
    lines.append("")
    lines.append(f"碰撞 {len(failed)} / {len(rows)}。")
    clear = [row for row in rows if row.get("outcome") != "COLLISION"]
    if clear:
        lines.append("没有撞树的 seed：" + "、".join(f"{row['seed']}（{row['outcome']}）" for row in clear) + "。")
    lines.append("")
    lines.append("## 撞前")
    lines.append("")
    for row in failed:
        lines.append(window_table(row))
        lines.append("")
    return "\n".join(lines)


def run_seed(path: Path, seed: int, *, shadow: bool = False):
    previous = _install(_DiagFleet)
    try:
        card = load_card(path)
        out = run(card, RunConfig(seed=seed, policy="crossing", shadow=shadow))
        fleet = _DiagFleet._last
        facts = diagnose_output(card, out, fleet)
    finally:
        _install(previous)
    return facts, out


def main(argv=None) -> int:
    import argparse

    ap = argparse.ArgumentParser(description="票 11c 六机穿林撞树定位")
    ap.add_argument("--scene", default="scenes/crossing-6-forest.json")
    ap.add_argument("--seeds", default="2,3,5,7,8,9")
    ap.add_argument("--out", default="")
    ap.add_argument("--json-out", default="")
    a = ap.parse_args(argv)
    path = Path(a.scene)
    rows = []
    for seed in [int(s) for s in a.seeds.split(",") if s != ""]:
        facts, _out = run_seed(path, seed)
        print(window_table(facts), flush=True)
        print("---", flush=True)
        rows.append(_public_facts(facts))
    if a.out:
        text = summarize(rows, title=f"票 11c 定位 {path.name} seeds {a.seeds}")
        dest = Path(a.out)
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(text + "\n")
        blob = a.json_out or str(dest.with_suffix(".json"))
        Path(blob).write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n")
        print(f"写入 {dest}")
    elif a.json_out:
        Path(a.json_out).write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
