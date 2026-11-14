#!/usr/bin/env python3
"""E-B8：crossing-pillars 开欧氏球越已冻门两粒归因。只报数。

E-B6 在柱林 seeds 0–9 上看到开球三档都是 30/30、0 碰撞，但 seed 3 最小净空
0.365 m（已冻门 0.40 m），seed 6 最小机间 1.208 m（已冻门 2.0 m）。三档逐种子
相同，E-B6 写开球必须同时打开后段，路径变化跟着后段走。本票只追这两粒。

四臂（不改磁盘上的卡，开关在进程里盖过）：

- baseline：场景卡默认。后段关，球关。
- bspline_only：只把后段 ``enabled`` 设为 JSON true。球关。后段可以独立打开。
- b4c：后段 + E-B4c 默认球，r = 0.85 m，w = 8。
- b4c-no-obstacle：与 b4c 同一张进程内的卡，但把求解器里的障碍铰链权重
  ``_LAMBDA_C`` 写成 0。平滑、速度加速度、锚点和欧氏球仍在。接受检查
  （新曲线净空不能比输入多项式差过 1 cm）不看这个权重，仍然留着。

种子只有 crossing-pillars 的 3 和 6。影子盾开。后段预算仍是墙钟 0.02 s。
四臂串行，避免墙钟预算互相截短。

越门时刻：seed 3 是真值净空第一次低于 0.40 m，seed 6 是真值机间第一次低于
2.0 m。没有越过时，窗口改放在该指标全程最紧的那一拍，并标明未越门。
窗口是这一拍前后 1 s 的每个物理 tick（100 Hz），记下规划码、当前在飞的那次
后段是否被预算截断、那次求解里球铰链是否激活、参考指令对障碍表面和邻机
参考指令的同时刻距离。

``crossing.py`` 仍只在椭球项打开时把邻机送进 ``refine_yopo``。本脚本不改
``yopo_l0/``，只在进程里包一层：欧氏球打开时送本规划拍的邻机采样。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from collections import Counter
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-b8-pillars-sphere" / "partial"
SCENE = "crossing-pillars"
SCENE_PATH = "scenes/crossing-pillars.json"
SEEDS = (3, 6)
ARMS = ("baseline", "bspline_only", "b4c", "b4c-no-obstacle")
BODY_R = 0.35
CLEARANCE_DOOR_M = 0.40
INTER_DOOR_M = 2.0
WINDOW_S = 1.0
SAME_M = 1e-3
PHYSICS_DT_S = 0.01
DEVIATION_LABEL = "report-only / 偏离上游"
ENV_KEYS = (
    "YOPO_PEER_SPHERE",
    "YOPO_PEER_SPHERE_RADIUS_M",
    "YOPO_PEER_SPHERE_WEIGHT",
)
SPHERE = {
    "enabled": True,
    "peer_sphere_enabled": True,
    "peer_sphere_radius_m": 0.85,
    "peer_sphere_weight": 8.0,
}
# E-B6 正文里这两粒的基线和开球档。用来对照，不是本票的通过线。
EB6 = {
    (3, "baseline"): {"clearance": 0.915, "inter": 3.506, "arrival": 7.76, "p95": 4.139},
    (3, "b4c"): {"clearance": 0.365, "inter": 3.681, "arrival": 7.02, "p95": 4.190},
    (6, "baseline"): {"clearance": 0.828, "inter": 3.646, "arrival": 7.07, "p95": 4.249},
    (6, "b4c"): {"clearance": 0.489, "inter": 1.208, "arrival": 7.26, "p95": 4.940},
}
DOOR_OF = {3: "clearance", 6: "inter"}
CAUSE_ZH = {
    "rear": "越门来自后段平滑。",
    "sphere": "越门来自球项。",
    "budget": "越门跟着后段预算截断走。",
    "mixed": "后段平滑和球项都改了这道门的最紧值。",
    "none": "这道门没有被越过。",
    "baseline": "基线已经越过这道门。",
    "unclear": "四臂对不上单一来源。",
}
DISCLAIMER = (
    "只报数。report-only / 偏离上游。不改 yopo_l0/、core/、scenes/、docs/tickets.md、"
    "README.md、scripts/premerge.sh、scripts/regress.sh，不进 scenes/suite.txt，不改已有门。"
    "后段可以独立打开。开后段或开球都标 report-only / 偏离上游（上游 d5e1e60 的场景卡"
    "没有打开后段，也没有这颗欧氏球）。基线是场景卡默认策略加影子盾。"
    "种子只有 crossing-pillars 的 3 与 6。0.85 m = 0.70 m 碰撞球 + 0.15 m 裕度。"
    "关障碍项只把求解器障碍铰链权重写成 0，不改接受检查。"
)

_PEER_ON = False
_PEER_ORIG: dict = {}
_TRACE_ON = False
_TRACE_ORIG: dict = {}
_LAMBDA_ORIG: float | None = None
_TRACE: dict = {
    "events": [],
    "drone": None,
    "now": None,
    "hinge": False,
    "hinge_cost": 0.0,
}


def clear_sphere_env() -> None:
    """环境变量会盖过卡上的半径和权重。四臂只认卡。"""
    for key in ENV_KEYS:
        os.environ.pop(key, None)


def _deep(raw: dict) -> dict:
    return json.loads(json.dumps(raw))


def sphere_on(arm: str) -> bool:
    if arm not in ARMS:
        raise ValueError(arm)
    return arm in ("b4c", "b4c-no-obstacle")


def obstacle_term_on(arm: str) -> bool:
    if arm not in ARMS:
        raise ValueError(arm)
    return arm != "b4c-no-obstacle"


def backend_on(arm: str) -> bool:
    if arm not in ARMS:
        raise ValueError(arm)
    return arm != "baseline"


def deviation_label(arm: str) -> str:
    if arm not in ARMS:
        raise ValueError(arm)
    return "否" if arm == "baseline" else DEVIATION_LABEL


def door_limit(seed: int) -> tuple[str, float]:
    kind = DOOR_OF[int(seed)]
    if kind == "clearance":
        return kind, CLEARANCE_DOOR_M
    return kind, INTER_DOOR_M


def build_card(arm: str):
    """进程内盖一层。baseline 原样返回 load_card 的结果。

    b4c 与 b4c-no-obstacle 的卡相同。障碍项开关不写进卡，只改进程里的权重。
    """
    from yopo_l0.scene import SceneCard, load_card

    if arm not in ARMS:
        raise ValueError(arm)
    loaded = load_card(ROOT / SCENE_PATH)
    if arm == "baseline":
        return loaded
    raw = _deep(loaded.card)
    params = dict(raw.get("policy_params") or {})
    backend = dict(params.get("backend") or {})
    if arm == "bspline_only":
        backend["enabled"] = True
        backend.pop("peer_sphere_enabled", None)
        backend.pop("peer_sphere_radius_m", None)
        backend.pop("peer_sphere_weight", None)
        backend.pop("swarm_penalty", None)
        backend.pop("swarm_segment_penalty", None)
    else:
        backend.update(SPHERE)
        backend.pop("swarm_penalty", None)
        backend.pop("swarm_segment_penalty", None)
    params["backend"] = backend
    raw["policy_params"] = params
    return SceneCard(loaded.path, raw, loaded.obstacles)


def install_peer_delivery() -> None:
    """欧氏球打开时把本规划拍的邻机采样送进后段。椭球项仍只看 swarm_penalty。"""
    global _PEER_ON
    if _PEER_ON:
        return
    import yopo_l0.crossing as crossing_mod
    import yopo_l0.policy as policy_mod
    import yopo_l0.runner as runner_mod
    from yopo_l0.backend.config import peer_sphere_enabled as sphere_flag

    _PEER_ORIG["runner"] = runner_mod.swarm_penalty_enabled
    _PEER_ORIG["keep"] = policy_mod.UpstreamPolicy._keep_neighbor_samples
    _PEER_ORIG["apply"] = policy_mod.UpstreamPolicy._apply_backend
    _PEER_ORIG["cross"] = crossing_mod.CrossingFleet._apply_backend

    orig_enabled = runner_mod.swarm_penalty_enabled

    def enabled(params, **kwargs):
        if orig_enabled(params, **kwargs):
            return True
        return bool(sphere_flag(params, **kwargs))

    runner_mod.swarm_penalty_enabled = enabled

    orig_keep = policy_mod.UpstreamPolicy._keep_neighbor_samples

    def keep(self):
        if orig_keep(self):
            return True
        cfg = getattr(self, "_backend", None)
        return cfg is not None and bool(cfg.peer_sphere_enabled)

    policy_mod.UpstreamPolicy._keep_neighbor_samples = keep

    def apply(self, traj, goal):
        from yopo_l0.backend.bspline import refine_yopo

        cfg = self._backend
        use = bool(self._swarm) or (cfg is not None and bool(cfg.peer_sphere_enabled))
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
        wants = cfg is not None and (bool(cfg.swarm_penalty) or bool(cfg.peer_sphere_enabled))
        if wants and getattr(self, "_score_node", None) is not None:
            neighbors = committed_neighbor_samples(self._score_node, float(traj.t_start))
        out, rec = refine_yopo(traj, cfg, self._backend_obstacles, goal, neighbors=neighbors)
        self.backend_log.append(rec)
        return out

    crossing_mod.CrossingFleet._apply_backend = cross_apply
    _PEER_ON = True


def restore_peer_delivery() -> None:
    global _PEER_ON
    if not _PEER_ON:
        return
    import yopo_l0.crossing as crossing_mod
    import yopo_l0.policy as policy_mod
    import yopo_l0.runner as runner_mod

    runner_mod.swarm_penalty_enabled = _PEER_ORIG["runner"]
    policy_mod.UpstreamPolicy._keep_neighbor_samples = _PEER_ORIG["keep"]
    policy_mod.UpstreamPolicy._apply_backend = _PEER_ORIG["apply"]
    crossing_mod.CrossingFleet._apply_backend = _PEER_ORIG["cross"]
    _PEER_ON = False


def push_obstacle_term(enabled: bool) -> float:
    """返回改之前的权重。enabled 为假时写成 0，为真时回到本进程第一次见到的原值。"""
    global _LAMBDA_ORIG
    import yopo_l0.backend.bspline as bspline

    if _LAMBDA_ORIG is None:
        _LAMBDA_ORIG = float(bspline._LAMBDA_C)
    prev = float(bspline._LAMBDA_C)
    bspline._LAMBDA_C = _LAMBDA_ORIG if enabled else 0.0
    return prev


def pop_obstacle_term(prev: float) -> None:
    import yopo_l0.backend.bspline as bspline

    bspline._LAMBDA_C = float(prev)


def reset_trace() -> None:
    _TRACE["events"] = []
    _TRACE["drone"] = None
    _TRACE["now"] = None
    _TRACE["hinge"] = False
    _TRACE["hinge_cost"] = 0.0


def install_trace() -> None:
    """记下每次 refine 的无人机、仿真时刻、预算截断，以及求解过程里球铰链是否出过非零代价。"""
    global _TRACE_ON
    if _TRACE_ON:
        return
    import yopo_l0.backend.bspline as bspline
    import yopo_l0.crossing as crossing_mod

    _TRACE_ORIG["refine"] = bspline.refine_yopo
    _TRACE_ORIG["peer"] = bspline._peer_sphere_cost_grad
    _TRACE_ORIG["planner"] = crossing_mod.CrossingFleet._planner
    orig_refine = bspline.refine_yopo
    orig_peer = bspline._peer_sphere_cost_grad
    orig_planner = crossing_mod.CrossingFleet._planner

    def peer(*args, **kwargs):
        cost, grad = orig_peer(*args, **kwargs)
        if cost != 0.0 or (grad is not None and bool(grad.any())):
            _TRACE["hinge"] = True
            if float(cost) > float(_TRACE["hinge_cost"]):
                _TRACE["hinge_cost"] = float(cost)
        return cost, grad

    def refine(traj, cfg, obstacles, goal=None, neighbors=None):
        import yopo_l0.backend.bspline as bspline_mod

        _TRACE["hinge"] = False
        _TRACE["hinge_cost"] = 0.0
        out, rec = orig_refine(traj, cfg, obstacles, goal, neighbors=neighbors)
        threshold = float(cfg.clearance_m) + float(bspline_mod._CLEARANCE_BUFFER_M)
        signed = rec.get("clearance_after_m") if rec.get("applied") else rec.get("clearance_before_m")
        hinge_geom = signed is not None and float(signed) < threshold - 1e-12
        _TRACE["events"].append(
            {
                "drone": _TRACE["drone"],
                "t_sim": None if _TRACE["now"] is None else float(_TRACE["now"]),
                "t_start": float(traj.t_start),
                "applied": bool(rec.get("applied")),
                "budget_cut": bool(rec.get("budget_cut")),
                "reason": rec.get("reason"),
                "iterations": None if rec.get("iterations") is None else int(rec["iterations"]),
                "sphere_hinge": bool(_TRACE["hinge"]),
                "sphere_hinge_cost": float(_TRACE["hinge_cost"]),
                "clearance_before_m": rec.get("clearance_before_m"),
                "clearance_after_m": rec.get("clearance_after_m"),
                "obstacle_threshold_m": threshold,
                "obstacle_hinge": bool(hinge_geom),
                "obstacle_term": float(bspline_mod._LAMBDA_C) != 0.0,
                "output_peer_m": _horizon_peer(out, neighbors),
            }
        )
        return out, rec

    def planner(self, node, now, t_start):
        prev = (_TRACE["drone"], _TRACE["now"])
        _TRACE["drone"] = node.id
        _TRACE["now"] = float(now)
        try:
            return orig_planner(self, node, now, t_start)
        finally:
            _TRACE["drone"], _TRACE["now"] = prev

    bspline._peer_sphere_cost_grad = peer
    bspline.refine_yopo = refine
    crossing_mod.CrossingFleet._planner = planner
    _TRACE_ON = True


def restore_trace() -> None:
    global _TRACE_ON
    if not _TRACE_ON:
        return
    import yopo_l0.backend.bspline as bspline
    import yopo_l0.crossing as crossing_mod

    bspline.refine_yopo = _TRACE_ORIG["refine"]
    bspline._peer_sphere_cost_grad = _TRACE_ORIG["peer"]
    crossing_mod.CrossingFleet._planner = _TRACE_ORIG["planner"]
    _TRACE_ON = False


def _horizon_peer(traj, neighbors) -> float | None:
    """输出曲线对邻机采样的同时刻最近距离。没有邻机或没有时间重叠时是 null。"""
    if not neighbors:
        return None
    import numpy as np

    from yopo_l0.inter_drone import clearance_m, sample_motion

    arrays = []
    for raw in neighbors:
        arr = np.asarray(raw, dtype=float)
        if arr.ndim == 2 and arr.shape[1] == 4 and arr.shape[0] > 0 and bool(np.all(np.isfinite(arr))):
            arrays.append(arr)
    if not arrays:
        return None
    dist = clearance_m(sample_motion(traj, float(traj.t_start)), arrays)
    if not math.isfinite(dist):
        return None
    return float(dist)


def _pair_min(pos: dict) -> tuple[float | None, str | None]:
    import numpy as np

    best = None
    who = None
    ids = sorted(pos)
    for i, left in enumerate(ids):
        for right in ids[i + 1 :]:
            dist = float(np.linalg.norm(pos[left] - pos[right]))
            if best is None or dist < best:
                best = dist
                who = f"{left}-{right}"
    return best, who


def _peer_of(uid: str, pos: dict) -> tuple[float | None, str | None]:
    import numpy as np

    best = None
    who = None
    for other, point in pos.items():
        if other == uid:
            continue
        dist = float(np.linalg.norm(pos[uid] - point))
        if best is None or dist < best:
            best = dist
            who = other
    return best, who


def build_frames(ticks: list[dict], card) -> list[dict]:
    """每个物理 tick 合成一帧。参考距离用该拍的指令位置，真值用积分后的位置。"""
    by_k: dict[int, list[dict]] = {}
    for row in ticks:
        by_k.setdefault(int(row["k"]), []).append(row)
    frames = []
    order = [d["id"] for d in card.card["drones"]]
    for k in sorted(by_k):
        rows = sorted(by_k[k], key=lambda row: order.index(row["uav"]) if row["uav"] in order else 99)
        import numpy as np

        pos = {row["uav"]: np.asarray(row["p"], dtype=float) for row in rows}
        cmd = {row["uav"]: np.asarray(row["cmd_p"], dtype=float) for row in rows}
        truth_inter, truth_pair = _pair_min(pos)
        drones = []
        for row in rows:
            uid = row["uav"]
            surface = float(card.min_distance(cmd[uid]))
            ref_peer, ref_who = _peer_of(uid, cmd)
            drones.append(
                {
                    "uav": uid,
                    "plan_code": row.get("plan_code"),
                    "ref_obstacle_m": surface,
                    "ref_clearance_m": surface - BODY_R,
                    "ref_peer_m": ref_peer,
                    "ref_peer": ref_who,
                    "truth_obstacle_m": float(row["min_obstacle_m"]),
                    "truth_clearance_m": float(row["min_obstacle_m"]) - BODY_R,
                }
            )
        clearances = [item["truth_clearance_m"] for item in drones]
        frames.append(
            {
                "k": int(k),
                "t": float(rows[0]["t"]),
                "truth_clearance_m": None if not clearances else min(clearances),
                "truth_inter_m": truth_inter,
                "truth_inter_pair": truth_pair,
                "drones": drones,
            }
        )
    return frames


def _metric_key(kind: str) -> str:
    return "truth_clearance_m" if kind == "clearance" else "truth_inter_m"


def choose_event(frames: list[dict], kind: str, limit: float, official_min: float | None) -> dict:
    """第一次低于门的时刻。官方最小值来自运行器（含起点）。没越过就用最紧的那一拍。"""
    key = _metric_key(kind)
    hit = None
    for frame in frames:
        value = frame.get(key)
        if value is not None and float(value) < float(limit) - 1e-9:
            hit = frame
            break
    tight = None
    for frame in frames:
        value = frame.get(key)
        if value is None:
            continue
        if tight is None or float(value) < float(tight[key]):
            tight = frame
    # 越门与否跟运行器的全程最小走，和已冻门同一口径。时刻用第一次低于门的 tick。
    crossed = official_min is not None and float(official_min) < float(limit) - 1e-9
    if crossed and hit is not None:
        event_t = float(hit["t"])
    elif crossed:
        event_t = 0.0
    elif tight is not None:
        event_t = float(tight["t"])
    else:
        event_t = None
    return {
        "crossed": bool(crossed),
        "event_t": event_t,
        "tight_t": None if tight is None else float(tight["t"]),
        "tight_value": None if tight is None else float(tight[key]),
    }


def _tight_window(frames: list[dict], event_t: float | None, tight_t: float | None) -> list[dict] | None:
    """全程最紧的那一拍若落在越门窗口外面，再记它前后 1 s。"""
    if event_t is None or tight_t is None:
        return None
    if abs(float(tight_t) - float(event_t)) <= WINDOW_S + 1e-9:
        return None
    return slice_window(frames, tight_t)


def slice_window(frames: list[dict], event_t: float | None, half: float = WINDOW_S) -> list[dict]:
    if event_t is None:
        return []
    lo = float(event_t) - float(half)
    hi = float(event_t) + float(half)
    return [frame for frame in frames if lo - 1e-9 <= float(frame["t"]) <= hi + 1e-9]


def attach_refine(frames: list[dict], events: list[dict]) -> None:
    """每一拍挂上该机最近一次已经接受的后段。没跑过后段时截断和铰链都是 null。"""
    by_drone: dict[str, list[dict]] = {}
    for event in events:
        if event.get("drone") is None or event.get("t_sim") is None:
            continue
        by_drone.setdefault(str(event["drone"]), []).append(event)
    for hist in by_drone.values():
        hist.sort(key=lambda event: (float(event["t_sim"]), float(event["t_start"])))
    for frame in frames:
        t = float(frame["t"])
        for drone in frame["drones"]:
            hist = by_drone.get(drone["uav"], [])
            chosen = None
            for event in hist:
                if float(event["t_sim"]) <= t + 1e-9 and event.get("applied"):
                    chosen = event
            if chosen is None:
                drone["budget_cut"] = None
                drone["sphere_hinge"] = None
                drone["obstacle_hinge"] = None
                drone["refine_t"] = None
                drone["refine_applied"] = None
            else:
                drone["budget_cut"] = bool(chosen["budget_cut"])
                drone["sphere_hinge"] = bool(chosen["sphere_hinge"])
                drone["obstacle_hinge"] = bool(chosen["obstacle_hinge"])
                drone["refine_t"] = float(chosen["t_sim"])
                drone["refine_applied"] = True


def window_digest(window: list[dict], events: list[dict], event_t: float | None) -> dict:
    codes: Counter[str] = Counter()
    ref_clear = []
    ref_peer = []
    for frame in window:
        for drone in frame["drones"]:
            if drone.get("plan_code"):
                codes[str(drone["plan_code"])] += 1
            if drone.get("ref_clearance_m") is not None:
                ref_clear.append(float(drone["ref_clearance_m"]))
            if drone.get("ref_peer_m") is not None:
                ref_peer.append(float(drone["ref_peer_m"]))
    lo = None if event_t is None else float(event_t) - WINDOW_S
    hi = None if event_t is None else float(event_t) + WINDOW_S
    in_window = []
    if lo is not None:
        for event in events:
            if event.get("t_sim") is None:
                continue
            if lo - 1e-9 <= float(event["t_sim"]) <= hi + 1e-9:
                in_window.append(event)
    return {
        "n_ticks": len(window),
        "plan_codes": dict(sorted(codes.items())),
        "ref_clearance_min_m": None if not ref_clear else min(ref_clear),
        "ref_peer_min_m": None if not ref_peer else min(ref_peer),
        "budget_cuts": sum(1 for event in in_window if event.get("budget_cut")),
        "sphere_hinges": sum(1 for event in in_window if event.get("sphere_hinge")),
        "sphere_hinges_applied": sum(1 for event in in_window if event.get("sphere_hinge") and event.get("applied")),
        "obstacle_hinges": sum(
            1 for event in in_window if event.get("obstacle_hinge") and event.get("obstacle_term") and event.get("applied")
        ),
    }


def _counts(result: dict) -> tuple[int, int, float | None]:
    reason = result.get("reason") or ""
    outcome = result.get("outcome")
    block = result.get("crossing") or {}
    obs = int(outcome == "COLLISION" and "TRUTH_POSITION" in str(reason))
    inter = int(outcome == "COLLISION" and "INTER_DRONE" in str(reason))
    if "obstacle_collisions" in block:
        obs = int(block.get("obstacle_collisions") or 0)
        inter = int(block.get("inter_drone_collisions") or 0)
    clearance = None
    if block.get("min_obstacle_clearance_m") is not None:
        clearance = float(block["min_obstacle_clearance_m"])
    elif result.get("min_obstacle_distance_m") is not None:
        clearance = float(result["min_obstacle_distance_m"]) - BODY_R
    return obs, inter, clearance


def _pin_threads() -> None:
    import torch

    torch.set_num_threads(1)


def _drop_samples(block: dict | None) -> dict | None:
    if not isinstance(block, dict):
        return None
    out = dict(block)
    for key in ("time_ms", "iterations"):
        part = out.get(key)
        if isinstance(part, dict):
            kept = dict(part)
            kept.pop("samples", None)
            out[key] = kept
    return out


def _event_counts(events: list[dict]) -> dict:
    applied = [event for event in events if event.get("applied")]
    return {
        "backend_calls": len(events),
        "backend_applied": len(applied),
        "backend_budget_cuts": sum(1 for event in events if event.get("budget_cut")),
        "sphere_hinge_calls": sum(1 for event in events if event.get("sphere_hinge")),
        "sphere_hinge_applied": sum(1 for event in events if event.get("sphere_hinge") and event.get("applied")),
        "obstacle_hinge_applied": sum(
            1 for event in events if event.get("obstacle_hinge") and event.get("obstacle_term") and event.get("applied")
        ),
    }


def _official_min(kind: str, clearance: float | None, inter: float | None) -> float | None:
    return clearance if kind == "clearance" else inter


def _window_coverage(
    window: list[dict],
    event_t: float | None,
    sim_t_start: float | None,
    sim_t_end: float | None,
) -> str:
    if event_t is None or not window:
        return "empty"
    first = float(window[0]["t"])
    last = float(window[-1]["t"])
    want_lo = float(event_t) - WINDOW_S
    want_hi = float(event_t) + WINDOW_S
    if sim_t_start is not None:
        want_lo = max(want_lo, float(sim_t_start))
    if sim_t_end is not None:
        want_hi = min(want_hi, float(sim_t_end))
    start_ok = first <= want_lo + PHYSICS_DT_S + 1e-6
    end_ok = last + 1e-6 >= want_hi - PHYSICS_DT_S
    if start_ok and end_ok:
        return "full"
    if not start_ok and end_ok:
        return "start"
    if start_ok and not end_ok:
        return "end"
    return "both"


def run_one(arm: str, seed: int, *, resume: bool) -> dict:
    dest = PARTIAL / f"{SCENE}__{arm}__{seed}.json"
    if resume and dest.is_file():
        return json.loads(dest.read_text(encoding="utf-8"))
    from yopo_l0.runner import RunConfig, run

    clear_sphere_env()
    _pin_threads()
    install_peer_delivery()
    install_trace()
    reset_trace()
    card = build_card(arm)
    weight = push_obstacle_term(obstacle_term_on(arm))
    try:
        t0 = time.perf_counter()
        out = run(card, RunConfig(seed=int(seed), shadow=True))
        events = list(_TRACE["events"])
        frames = build_frames(out.ticks, card)
        attach_refine(frames, events)
        kind, limit = door_limit(int(seed))
        obs, inter_hits, clearance = _counts(out.result)
        inter = out.result.get("min_inter_drone_distance_m")
        inter_f = None if inter is None else float(inter)
        official = _official_min(kind, clearance, inter_f)
        event = choose_event(frames, kind, limit, official)
        window = slice_window(frames, event["event_t"])
        tight_window = _tight_window(frames, event["event_t"], event["tight_t"])
        other_kind = "inter" if kind == "clearance" else "clearance"
        other_limit = INTER_DOOR_M if other_kind == "inter" else CLEARANCE_DOOR_M
        other_official = _official_min(other_kind, clearance, inter_f)
        counts = _event_counts(events)
        row = {
            "scene_kind": SCENE,
            "arm": arm,
            "arm_level": {
                "baseline": "基线",
                "bspline_only": "只开后段",
                "b4c": "后段+球 E-B4c",
                "b4c-no-obstacle": "后段+球、关障碍项",
            }[arm],
            "seed": int(seed),
            "door": kind,
            "door_limit_m": limit,
            "door_crossed": bool(event["crossed"]),
            "door_min": official,
            "event_t": event["event_t"],
            "tight_t": event["tight_t"],
            "tight_value": event["tight_value"],
            "other_door": other_kind,
            "other_door_limit_m": other_limit,
            "other_door_crossed": bool(
                other_official is not None and float(other_official) < other_limit - 1e-9
            ),
            "outcome": out.result.get("outcome"),
            "reason": out.result.get("reason") or "",
            "arrived": int(out.result["arrived"]),
            "n_drones": int(out.result["n_drones"]),
            "arrival_time_s": out.result.get("arrival_time_s"),
            "obstacle_collisions": obs,
            "inter_drone_collisions": inter_hits,
            "timeout": int(out.result.get("outcome") == "TIMEOUT"),
            "min_inter_drone_distance_m": inter_f,
            "min_obstacle_clearance_m": clearance,
            "formation_error_p95_m": out.result.get("formation_error_p95_m"),
            "sim_duration_s": out.result.get("sim_duration_s"),
            "wall_time_s": out.result.get("wall_time_s"),
            "worker_s": time.perf_counter() - t0,
            "scene_card_hash": out.result.get("scene_card_hash"),
            "commit": out.result.get("commit"),
            "plan_codes": out.result.get("plan_codes") or {},
            "backend": _drop_samples(out.result.get("backend")),
            "runner_upstream_deviation": out.result.get("upstream_deviation"),
            "report_only": True,
            "deviates_from_upstream": arm != "baseline",
            "deviation_label": deviation_label(arm),
            "peer_sphere_enabled": sphere_on(arm),
            "peer_sphere_radius_m": SPHERE["peer_sphere_radius_m"] if sphere_on(arm) else None,
            "peer_sphere_weight": SPHERE["peer_sphere_weight"] if sphere_on(arm) else None,
            "obstacle_term": obstacle_term_on(arm),
            "backend_enabled": backend_on(arm),
            **counts,
            "window_coverage": _window_coverage(window, event["event_t"], _first_t(frames), _last_t(frames)),
            "window_summary": window_digest(window, events, event["event_t"]),
            "window": window,
            "tight_window": tight_window,
            "tight_window_summary": None
            if tight_window is None
            else window_digest(tight_window, events, event["tight_t"]),
            "events": events,
            "disclaimer": DISCLAIMER,
        }
    finally:
        pop_obstacle_term(weight)
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)
    print(
        f"{arm} seed={seed} {row['outcome']} door={kind} crossed={row['door_crossed']} "
        f"t={_f(row['event_t'], 2)} clear={_f(clearance, 3)} inter={_f(inter_f, 3)} "
        f"hinge={row['sphere_hinge_applied']}/{row['sphere_hinge_calls']} "
        f"obs_hinge={row['obstacle_hinge_applied']} cuts={row['backend_budget_cuts']}/{row['backend_calls']} "
        f"wall={float(row['wall_time_s'] or 0):.1f}s",
        flush=True,
    )
    return row


def _first_t(frames: list[dict]) -> float | None:
    if not frames:
        return None
    return float(frames[0]["t"])


def _last_t(frames: list[dict]) -> float | None:
    if not frames:
        return None
    return float(frames[-1]["t"])


def _same_door(left: dict, right: dict) -> bool:
    if bool(left["door_crossed"]) != bool(right["door_crossed"]):
        return False
    if left.get("door_min") is None or right.get("door_min") is None:
        return left.get("door_min") is None and right.get("door_min") is None
    return abs(float(left["door_min"]) - float(right["door_min"])) <= SAME_M


def attribute_seed(by_arm: dict) -> dict:
    """按这道门的越门与否和最紧值，把四臂收成一个来源。

    1 mm 以内且越门与否相同，视为同一结果。球铰链次数只数已经接受的后段。
    """
    base = by_arm["baseline"]
    smooth = by_arm["bspline_only"]
    sphere = by_arm["b4c"]
    naked = by_arm["b4c-no-obstacle"]
    obstacle_moves = not _same_door(sphere, naked)
    hinge = int(sphere.get("sphere_hinge_applied") or 0)
    cuts_smooth = int(smooth.get("backend_budget_cuts") or 0)
    cuts_sphere = int(sphere.get("backend_budget_cuts") or 0)
    base_c = bool(base["door_crossed"])
    smooth_c = bool(smooth["door_crossed"])
    sphere_c = bool(sphere["door_crossed"])

    if not base_c and not smooth_c and not sphere_c:
        cause = "none"
    elif base_c and _same_door(base, smooth) and _same_door(base, sphere):
        cause = "baseline"
    elif (not base_c) and smooth_c and sphere_c and _same_door(smooth, sphere):
        cause = "rear"
    elif (not base_c) and smooth_c and sphere_c:
        if hinge > 0:
            cause = "mixed"
        elif cuts_smooth != cuts_sphere:
            cause = "budget"
        else:
            cause = "unclear"
    elif (not base_c) and (not smooth_c) and sphere_c:
        if hinge > 0:
            cause = "sphere"
        elif cuts_smooth != cuts_sphere:
            cause = "budget"
        else:
            cause = "unclear"
    elif (not base_c) and smooth_c and (not sphere_c):
        cause = "sphere" if hinge > 0 else "unclear"
    else:
        cause = "unclear"
    return {
        "cause": cause,
        "cause_zh": CAUSE_ZH[cause],
        "obstacle_moves": obstacle_moves,
        "sphere_hinge_applied": hinge,
        "bspline_matches_b4c": _same_door(smooth, sphere),
        "baseline_crossed": base_c,
        "bspline_crossed": smooth_c,
        "b4c_crossed": sphere_c,
        "no_obstacle_crossed": bool(naked["door_crossed"]),
    }


def attribute_rows(rows: list[dict]) -> dict:
    out = {}
    for seed in SEEDS:
        group = {arm: _find(rows, seed, arm) for arm in ARMS}
        if any(item is None for item in group.values()):
            continue
        out[str(seed)] = attribute_seed(group)
    return out


def _find(rows: list[dict], seed: int, arm: str) -> dict | None:
    for row in rows:
        if int(row["seed"]) == int(seed) and row["arm"] == arm:
            return row
    return None


def _f(value, nd: int) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{nd}f}"


def _yn(flag: bool) -> str:
    return "是" if flag else "否"


def format_outcome_table(rows: list[dict]) -> str:
    lines = [
        "| seed | 臂 | 标注 | 到达 | 障碍碰撞 | 机间碰撞 | 最小净空 m | 最小机间 m | 队形误差 p95 m | 到达时间 s | 本门 | 越门 | 时刻 s |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for seed in SEEDS:
        for arm in ARMS:
            row = _find(rows, seed, arm)
            if row is None:
                continue
            lines.append(
                f"| {seed} | {arm} | {row['deviation_label']} | {row['arrived']}/{row['n_drones']} | "
                f"{row['obstacle_collisions']} | {row['inter_drone_collisions']} | "
                f"{_f(row['min_obstacle_clearance_m'], 3)} | {_f(row['min_inter_drone_distance_m'], 3)} | "
                f"{_f(row['formation_error_p95_m'], 3)} | {_f(row['arrival_time_s'], 2)} | "
                f"{row['door']} { _f(row['door_limit_m'], 2) } | {_yn(row['door_crossed'])} | {_f(row['event_t'], 2)} |"
            )
    return "\n".join(lines)


def format_window_table(rows: list[dict]) -> str:
    lines = [
        "| seed | 臂 | 窗内 tick | 窗 | 窗内参考净空最小 m | 窗内参考机间最小 m | 窗内截断 | 窗内球铰链 | 全程球铰链 | 全程障碍铰链 | 全程截断 |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for seed in SEEDS:
        for arm in ARMS:
            row = _find(rows, seed, arm)
            if row is None:
                continue
            summary = row.get("window_summary") or {}
            lines.append(
                f"| {seed} | {arm} | {summary.get('n_ticks', 0)} | {row.get('window_coverage')} | "
                f"{_f(summary.get('ref_clearance_min_m'), 3)} | {_f(summary.get('ref_peer_min_m'), 3)} | "
                f"{summary.get('budget_cuts', 0)} | {summary.get('sphere_hinges_applied', 0)} | "
                f"{row.get('sphere_hinge_applied', 0)} | {row.get('obstacle_hinge_applied', 0)} | "
                f"{row.get('backend_budget_cuts', 0)}/{row.get('backend_calls', 0)} |"
            )
    return "\n".join(lines)


def _crossing_snapshot(row: dict) -> list[str]:
    event_t = row.get("event_t")
    if event_t is None:
        return []
    frame = None
    best = None
    for item in row.get("window") or []:
        gap = abs(float(item["t"]) - float(event_t))
        if best is None or gap < best:
            best = gap
            frame = item
    if frame is None:
        return []
    lines = [
        f"seed {row['seed']} {row['arm']} t={_f(frame['t'], 2)} "
        f"真值净空 {_f(frame['truth_clearance_m'], 3)} m，真值机间 {_f(frame['truth_inter_m'], 3)} m"
        + (f"（{frame['truth_inter_pair']}）" if frame.get("truth_inter_pair") else "")
        + "。"
    ]
    for drone in frame["drones"]:
        lines.append(
            f"- {drone['uav']} 规划码 `{drone.get('plan_code')}`，截断 {_cell_flag(drone.get('budget_cut'))}，"
            f"球铰链 {_cell_flag(drone.get('sphere_hinge'))}，"
            f"参考净空 {_f(drone.get('ref_clearance_m'), 3)} m，参考机间 {_f(drone.get('ref_peer_m'), 3)} m。"
        )
    return lines


def _cell_flag(value) -> str:
    if value is None:
        return "—"
    return "是" if value else "否"


def _obstacle_sentence(attribution: dict, rows: list[dict], seed: int) -> str:
    sphere = _find(rows, seed, "b4c")
    naked = _find(rows, seed, "b4c-no-obstacle")
    if sphere is None or naked is None:
        return ""
    if attribution["obstacle_moves"]:
        return (
            f"关掉障碍项之后，本门最紧值从 {_f(sphere['door_min'], 3)} m 变成 {_f(naked['door_min'], 3)} m，"
            f"越门从 {_yn(sphere['door_crossed'])} 变成 {_yn(naked['door_crossed'])}。"
        )
    return "关掉障碍项之后，越门与否相同，最紧值差不超过 1 mm。"


def _eb6_sentence(rows: list[dict]) -> str:
    bits = []
    for seed, arm, label in ((3, "b4c", "seed 3 净空"), (6, "b4c", "seed 6 机间")):
        row = _find(rows, seed, arm)
        ref = EB6[(seed, arm)]
        if row is None:
            continue
        got = row["min_obstacle_clearance_m"] if arm == "b4c" and seed == 3 else row["min_inter_drone_distance_m"]
        if seed == 3:
            got = row["min_obstacle_clearance_m"]
            prev = ref["clearance"]
        else:
            got = row["min_inter_drone_distance_m"]
            prev = ref["inter"]
        bits.append(f"{label} {_f(got, 3)} m（E-B6 记下 {_f(prev, 3)} m）")
    if not bits:
        return ""
    return "本票后段加球测得 " + "，".join(bits) + "。"


def _frame_near(frames: list[dict] | None, t: float | None) -> dict | None:
    if not frames or t is None:
        return None
    return min(frames, key=lambda frame: abs(float(frame["t"]) - float(t)))


def _cut_times(row: dict) -> list[float]:
    times = []
    for event in row.get("events") or []:
        if event.get("budget_cut") and event.get("t_sim") is not None:
            times.append(float(event["t_sim"]))
    return times


def _evidence_lines(rows: list[dict]) -> list[str]:
    lines = []
    opened = [row for row in rows if row["arm"] != "baseline"]
    if opened and all(int(row.get("sphere_hinge_applied") or 0) == 0 for row in opened):
        lines.append(f"开了后段的 {len(opened)} 条记录里，已经接受的后段上球铰链次数都是 0。")
    if opened and all(int(row.get("obstacle_hinge_applied") or 0) == 0 for row in opened):
        lines.append(
            "已经接受的后段上，障碍铰链次数也是 0。"
            "障碍铰链要曲线到障碍表面的距离低于 0.40 m 才有代价；已冻净空门 0.40 m 对应表面距离 0.75 m。"
        )
    smooth3 = _find(rows, 3, "bspline_only")
    sphere3 = _find(rows, 3, "b4c")
    if smooth3 is not None and smooth3.get("window"):
        truth = min(float(frame["truth_clearance_m"]) for frame in smooth3["window"])
        ref = min(float(drone["ref_clearance_m"]) for frame in smooth3["window"] for drone in frame["drones"])
        gap = (
            "窗口里的指令参考没有低于 0.40 m，低于门的是机体相对这条指令的跟踪。"
            if truth < CLEARANCE_DOOR_M - 1e-9 <= ref
            else ""
        )
        lines.append(
            f"seed 3 只开后段的越门窗口里，真值净空最紧 {_f(truth, 3)} m，指令参考净空最紧 {_f(ref, 3)} m。{gap}"
        )
        codes = (smooth3.get("window_summary") or {}).get("plan_codes") or {}
        if codes:
            listed = "、".join(f"{name} {count}" for name, count in codes.items())
            lines.append(f"这扇窗口的规划码按无人机乘物理 tick 计，是 {listed}。")
    if sphere3 is not None and smooth3 is not None and _cut_times(sphere3):
        lines.append(
            f"seed 3 后段加球有 {sphere3['backend_budget_cuts']} 次预算截断，时刻 "
            + "、".join(_f(t, 2) for t in _cut_times(sphere3))
            + f" s，越门窗口里的截断是 {(sphere3.get('window_summary') or {}).get('budget_cuts', 0)}。"
            f"到达时间 {_f(sphere3['arrival_time_s'], 2)} s，只开后段是 {_f(smooth3['arrival_time_s'], 2)} s。"
            "两臂的净空和机间相同。"
        )
    smooth6 = _find(rows, 6, "bspline_only")
    if smooth6 is not None and smooth6.get("event_t") is not None and smooth6.get("tight_t") is not None:
        if abs(float(smooth6["tight_t"]) - float(smooth6["event_t"])) > WINDOW_S:
            lines.append(
                f"seed 6 第一次低于 2.0 m 在 {_f(smooth6['event_t'], 2)} s。"
                f"全程最小 {_f(smooth6['door_min'], 3)} m 在 {_f(smooth6['tight_t'], 2)} s，落在这扇 1 s 窗口外面。"
                + ("最紧那一拍前后 1 s 记在 `tight_window`。" if smooth6.get("tight_window") else "")
            )
        codes = (smooth6.get("window_summary") or {}).get("plan_codes") or {}
        if codes:
            listed = "、".join(f"{name} {count}" for name, count in codes.items())
            lines.append(f"seed 6 越门窗口的规划码按无人机乘物理 tick 计，是 {listed}。")
    return lines


def render_markdown(payload: dict) -> str:
    rows = payload["rows"]
    attribution = payload.get("attribution") or {}
    lines = [
        "# E-B8：crossing-pillars 开欧氏球越已冻门两粒归因",
        "",
        "只报数。report-only / 偏离上游。没有改 `yopo_l0/`、`core/`、`scenes/`、`docs/tickets.md`、"
        "README、`scripts/premerge.sh`、`scripts/regress.sh`，没有进 `scenes/suite.txt`，没有改已冻门。",
        "",
        "数字来自 `scripts/e-b8-pillars-sphere.py`。越门时刻前后 1 s 的逐 tick 在 "
        "[e-b8-pillars-sphere-runs.json](e-b8-pillars-sphere-runs.json) 的 `window`。"
        "全程最紧若落在这扇窗口外面，另记在 `tight_window`。"
        "E-B6（PR #115，`docs/research/e-b6-suite-sphere.md`）在柱林开球三档上看到 seed 3 净空 0.365 m、"
        "seed 6 机间 1.208 m，三档逐种子相同，并写路径变化跟着后段平滑走。"
        "基线全套 seeds 0–9 的最紧值是净空 0.528 m、机间 3.350 m。本票只跑这两粒、这四臂。",
        "",
        "## 结论",
        "",
    ]
    for seed in SEEDS:
        block = attribution.get(str(seed))
        kind, limit = door_limit(seed)
        door_name = "障碍净空" if kind == "clearance" else "机间距离"
        if block is None:
            lines.append(f"seed {seed} 的{door_name}门（{limit:.2f} m）还没有四臂齐全的记录。")
            lines.append("")
            continue
        lines.append(
            f"seed {seed} 的{door_name}门（{limit:.2f} m）：{block['cause_zh']}"
            f"{_obstacle_sentence(block, rows, seed)}"
            f"后段加球已接受的球铰链次数是 {block['sphere_hinge_applied']}。"
        )
        lines.append("")
    eb6 = _eb6_sentence(rows)
    if eb6:
        lines.append(eb6)
        lines.append("")
    lines.extend(_evidence_lines(rows))
    if lines[-1] != "":
        lines.append("")
    other_bits = []
    for row in rows:
        if row.get("other_door_crossed"):
            other_bits.append(f"seed {row['seed']} {row['arm']} 还越过了{row['other_door']}门")
    if other_bits:
        lines.append("另外：" + "；".join(other_bits) + "。")
        lines.append("")
    lines.append(format_outcome_table(rows))
    lines.append("")
    lines.append(format_window_table(rows))
    lines.append("")
    lines.append("## 臂")
    lines.append("")
    lines.append("后段可以独立打开：`bspline_only` 只把 `policy_params.backend.enabled` 写成 JSON true，不写 `peer_sphere_enabled`。")
    lines.append("`b4c` 与 `b4c-no-obstacle` 的进程内场景卡相同，半径 0.85 m，权重 8。")
    lines.append("关障碍项是把 `yopo_l0.backend.bspline._LAMBDA_C` 在该臂运行期间写成 0。")
    lines.append("CrossingFleet 把机体半径 0.35 m 当作后段 `clearance_m`，再加求解器里的 0.05 m 缓冲，障碍铰链的表面距离门槛是 0.40 m。")
    lines.append("已冻净空门 0.40 m 是表面距离减去机体半径，对应表面距离 0.75 m，在这道门槛外面。")
    lines.append("球铰链只在求解过程中欧氏代价或梯度非零时算激活，半径 0.85 m。")
    lines.append("邻机采样仍按 E-B4c 的写法，在进程里送进 `refine_yopo`，椭球项保持关。见 [e-b4c-sphere-avoid.md](e-b4c-sphere-avoid.md)。")
    lines.append("")
    lines.append("| 臂 | 后段 | 欧氏球 | 障碍铰链权重 | 标注 |")
    lines.append("|---|---|---|---|---|")
    lines.append("| baseline | 关 | 关 | 原值（求解器不跑） | 否 |")
    lines.append("| bspline_only | 开 | 关 | 原值 40 | report-only / 偏离上游 |")
    lines.append("| b4c | 开 | 开，r=0.85，w=8 | 原值 40 | report-only / 偏离上游 |")
    lines.append("| b4c-no-obstacle | 开 | 开，r=0.85，w=8 | 0 | report-only / 偏离上游 |")
    lines.append("")
    lines.append("Crossing 运行器的 `upstream_deviation` 只看 `safe_radius`。四臂的安全半径仍是场景卡上的 0.05 m，那个字段可以是 false。本票以臂上的标注为准。")
    lines.append("后段 0.02 s 预算按墙钟截断，四臂串行。窗内截断是越门时刻前后 1 s 里 `budget_cut` 的后段次数；全程截断是整段飞行。")
    lines.append("")
    lines.append("## 越门那一拍")
    lines.append("")
    lines.append("窗口里每个物理 tick 都在 json 的 `window`。下面是离越门时刻最近的那一拍。没越门时，这一拍是该指标全程最紧的地方。")
    lines.append("参考净空和参考机间是该拍指令位置的同时刻距离：净空 = 指令到障碍表面的距离 − 0.35 m，机间是本机指令到邻机指令的距离。")
    lines.append("截断和球铰链来自该机最近一次已经接受的后段；基线没有后段，这两格是 —。")
    lines.append("")
    for seed in SEEDS:
        for arm in ARMS:
            row = _find(rows, seed, arm)
            if row is None:
                continue
            for line in _crossing_snapshot(row):
                lines.append(line)
            tight = row.get("tight_t")
            event_t = row.get("event_t")
            if tight is not None and event_t is not None and abs(float(tight) - float(event_t)) > 0.02:
                source = row.get("tight_window") or row.get("window")
                frame = _frame_near(source, tight)
                if frame is not None and abs(float(frame["t"]) - float(tight)) <= 0.02:
                    lines.append(
                        f"seed {row['seed']} {row['arm']} 最紧 t={_f(frame['t'], 2)} "
                        f"真值净空 {_f(frame['truth_clearance_m'], 3)} m，真值机间 {_f(frame['truth_inter_m'], 3)} m"
                        + (f"（{frame['truth_inter_pair']}）" if frame.get("truth_inter_pair") else "")
                        + "。"
                    )
                    for drone in frame["drones"]:
                        lines.append(
                            f"- {drone['uav']} 规划码 `{drone.get('plan_code')}`，截断 {_cell_flag(drone.get('budget_cut'))}，"
                            f"球铰链 {_cell_flag(drone.get('sphere_hinge'))}，"
                            f"参考净空 {_f(drone.get('ref_clearance_m'), 3)} m，参考机间 {_f(drone.get('ref_peer_m'), 3)} m。"
                        )
            lines.append("")
    lines.append("## 怎么读这张表")
    lines.append("")
    lines.append("种子 3 的本门是净空，种子 6 的本门是机间。越门时刻是真值第一次低于门的物理 tick。")
    lines.append("只开后段和后段加球在这道门上越门与否相同、最紧值差不超过 1 mm，归到后段平滑。")
    lines.append("只开后段守住、后段加球越过、并且已接受的后段里球铰链激活过，归到球项。")
    lines.append("球铰链没有激活、两臂预算截断次数不同，归到后段 0.02 s 墙钟预算。")
    lines.append("两臂都越过但最紧值差超过 1 mm，球铰链又激活过，写成后段和球项都改了最紧值。")
    lines.append("关障碍项与后段加球差超过 1 mm 时，结论句里单独写障碍项把最紧值挪了多少。障碍项没有单独当成越门来源：它只在后段已经打开的前提下改代价。")
    lines.append("")
    lines.append("```bash")
    lines.append(".venv/bin/python scripts/e-b8-pillars-sphere.py")
    lines.append(".venv/bin/python -m pytest -q tests/test_e_b8_pillars_sphere.py")
    lines.append("```")
    lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def _partial(arm: str, seed: int) -> Path:
    return PARTIAL / f"{SCENE}__{arm}__{seed}.json"


def _load_partials(jobs: list[tuple[str, int]]) -> list[dict]:
    rows = []
    for arm, seed in jobs:
        path = _partial(arm, seed)
        if path.is_file():
            rows.append(json.loads(path.read_text(encoding="utf-8")))
    return rows


def build_jobs(arms: list[str], seeds: list[int]) -> list[tuple[str, int]]:
    return [(arm, int(seed)) for arm in arms for seed in seeds]


def write_payload(rows: list[dict], jobs: list[tuple[str, int]], out: Path, doc: Path) -> dict:
    order = {arm: i for i, arm in enumerate(ARMS)}
    rows = sorted(rows, key=lambda row: (int(row["seed"]), order[row["arm"]]))
    payload = {
        "disclaimer": DISCLAIMER,
        "report_only": True,
        "deviation_label": DEVIATION_LABEL,
        "scene": SCENE,
        "seeds": list(SEEDS),
        "arms": list(ARMS),
        "doors": {
            "3": {"metric": "clearance", "limit_m": CLEARANCE_DOOR_M},
            "6": {"metric": "inter", "limit_m": INTER_DOOR_M},
        },
        "window_s": WINDOW_S,
        "same_m": SAME_M,
        "body_radius_m": BODY_R,
        "note": (
            "window 是越门时刻前后 1 s 的每个物理 tick（标称 100 Hz）。"
            "tight_window 只在全程最紧落在这扇窗口外面时写下，同样是前后 1 s。"
            "budget_cut 与 sphere_hinge 挂的是该机最近一次 applied 的后段；基线为 null。"
            "ref_clearance_m 与 ref_peer_m 是该拍指令位置的同时刻距离。"
            "sphere_hinge 在求解过程中欧氏代价或梯度非零时为 true。"
            "obstacle_hinge 表示接受曲线（没接受时用输入曲线）的最小表面距离低于 clearance_m + 0.05。"
            "door_min 用运行器的全程最小，和已冻门同一口径。"
        ),
        "completed": len(rows),
        "expected": len(jobs),
        "incomplete": len(jobs) - len(rows),
        "attribution": attribute_rows(rows),
        "rows": rows,
    }
    text = json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="utf-8")
    doc.write_text(render_markdown(payload), encoding="utf-8")
    return payload


def _parse_arms(text: str) -> list[str]:
    items = [part for part in text.split(",") if part]
    unknown = [part for part in items if part not in ARMS]
    if unknown:
        raise SystemExit(f"未知臂: {unknown}")
    return items


def _parse_seeds(text: str) -> list[int]:
    seeds = [int(part) for part in text.split(",") if part != ""]
    unknown = [seed for seed in seeds if seed not in SEEDS]
    if unknown:
        raise SystemExit(f"本票只跑种子 {list(SEEDS)}，收到 {unknown}")
    return seeds


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="E-B8 柱林两粒开球越门归因，只报数")
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=",".join(str(seed) for seed in SEEDS))
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--out", default="docs/research/e-b8-pillars-sphere-runs.json")
    ap.add_argument("--doc", default="docs/research/e-b8-pillars-sphere.md")
    ap.add_argument("--no-resume", action="store_true")
    ap.add_argument("--summarize-only", action="store_true")
    args = ap.parse_args(argv)
    if int(args.jobs) != 1:
        raise SystemExit("本票串行。后段 0.02 s 预算按墙钟截断，多于 1 个进程会把截断次数写进归因。")
    arms = _parse_arms(args.arms)
    seeds = _parse_seeds(args.seeds)
    jobs = build_jobs(arms, seeds)
    if not args.summarize_only:
        from yopo_swarm_core.baseline import fetch_upstream

        fetch_upstream()
        for arm, seed in jobs:
            run_one(arm, seed, resume=not args.no_resume)
    rows = _load_partials(jobs)
    payload = write_payload(rows, jobs, ROOT / args.out, ROOT / args.doc)
    print(format_outcome_table(rows))
    print()
    print(format_window_table(rows))
    for seed, block in payload["attribution"].items():
        print(f"seed {seed}: {block['cause']} {block['cause_zh']}")
    print(f"wrote {ROOT / args.out} and {ROOT / args.doc} ({payload['completed']}/{payload['expected']})")
    return 0 if payload["incomplete"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
