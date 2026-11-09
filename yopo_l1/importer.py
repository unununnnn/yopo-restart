"""import_l1(log_dir) -> 写入 log_dir/result.json，并返回这份记录。

记录和 yopo_l0 的 result.json 同一套字段，tier 固定为 L1，另加：

- ``takeoff_handoff_s``：从日志起点到起飞交接（规划器接手）的秒数。日志没写就是 null
- ``rtf``：仿真秒 / 墙钟秒。日志给了显式值就用显式值，否则用 sim_duration_s / wall_time_s

能读的日志（都在 log_dir 顶层或下一层）：

- ``run.json``：export_l1 写的身份（场景卡哈希、策略、影子盾、判据）
- ``scene.json``：盒子，用来算障碍距离和碰撞
- ``*.jsonl`` / ``*.jsonl.gz``：schema ``yopo-swarm/planner-log/1`` 的规划日志。
  影子盾开着时，diagnostic 不是 APPROVED 的步计为会拒
- ``ticks.jsonl``：和 L0 相同的逐 tick 记录（有的话优先于规划日志的位置和拒绝码）
- ``l1-flight.json`` 或 ``flight.json``：显式覆盖到达、碰撞、起飞交接、RTF。合成夹具用 ``label: synthetic``
- ``wall_time_s.txt`` / ``takeoff_handoff_s.txt``：抽检脚本写的单行浮点
"""

from __future__ import annotations

import argparse
import base64
import gzip
import json
from pathlib import Path

import numpy as np

from yopo_l0.runner import git_commit
from yopo_l0.scene import Ground, parse_geometry
from yopo_l0.shadow import aggregate, aggregate_drones

from .errors import L1LogError

PLANNER_SCHEMA = "yopo-swarm/planner-log/1"
# 与 yopo_swarm_core.oracle.FlightOracle 的默认值一致，只用于「还在垫上不算落地碰撞」。
_TRACKING_BOUND = 0.15
_SUPPORT_RADIUS = 0.15

_OVERRIDE_KEYS = (
    "outcome",
    "reason",
    "task",
    "arrived",
    "arrival_time_s",
    "min_obstacle_distance_m",
    "max_hover_error_m",
    "final_goal_error_m",
    "min_inter_drone_distance_m",
    "sim_duration_s",
    "ticks",
    "frames",
    "plans",
    "wall_time_s",
    "takeoff_handoff_s",
    "rtf",
    "plan_codes",
    "shadow_reject_rate",
    "shadow_reject_histogram",
    "shadow_drones",
    "commit",
    "seed",
    "policy",
    "shadow",
    "scene",
    "scene_card_hash",
    "n_drones",
    "dynamics",
    "network",
)


def _unarr(d: dict) -> np.ndarray:
    a = np.frombuffer(base64.b64decode(d["b64"]), dtype=np.dtype(d["dtype"]))
    return a.reshape(d["shape"]).copy()


def _unhex(s: str) -> float:
    return float.fromhex(s)


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _rows(path: Path):
    opener = gzip.open if path.name.endswith(".gz") else open
    with opener(path, "rt", encoding="utf-8") as fh:
        for line in fh:
            if line.strip():
                yield json.loads(line)


def _find(log_dir: Path, names: tuple[str, ...]) -> list[Path]:
    found = []
    for name in names:
        found.extend(log_dir.glob(name))
        found.extend(p for p in log_dir.glob(f"*/{name}") if p not in found)
    return sorted(set(found))


def _one(paths: list[Path], what: str) -> Path | None:
    if not paths:
        return None
    if len(paths) > 1:
        raise L1LogError(f"{what} 有多份：" + ", ".join(str(p) for p in paths))
    return paths[0]


def _float_file(path: Path | None) -> float | None:
    if path is None or not path.is_file():
        return None
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return None
    return float(text.split()[0])


def _planner_steps(path: Path) -> tuple[dict, list[dict]]:
    rows = _rows(path)
    header = next(rows, None)
    if not header or header.get("kind") != "header" or header.get("schema") != PLANNER_SCHEMA:
        raise L1LogError(f"{path} 不是 {PLANNER_SCHEMA} 规划日志")
    steps = []
    for row in rows:
        if row.get("kind") not in (None, "step"):
            continue
        obs = row.get("observation")
        decision = row.get("decision") or {}
        sample = None
        if obs and obs.get("state"):
            sample = {
                "t": _unhex(obs["now"]),
                "p": _unarr(obs["state"]["position"]).astype(float),
                "v": _unarr(obs["state"]["velocity"]).astype(float),
                "goal": _unarr(obs["goal"]).astype(float),
                "has_frame": obs.get("frame") is not None,
            }
        steps.append(
            {
                "diagnostic": decision.get("diagnostic") or "",
                "sample": sample,
                "has_frame": bool(sample and sample["has_frame"]),
            }
        )
    return header, steps


def _tick_samples(path: Path) -> list[dict]:
    samples = []
    for row in _rows(path):
        if "p" not in row:
            continue
            samples.append(
            {
                "t": float(row.get("t", 0.0)),
                "p": np.asarray(row["p"], dtype=float),
                "v": np.asarray(row.get("v", [0, 0, 0]), dtype=float),
                "uav": row.get("uav") or "uav0",
                "would_reject": row.get("would_reject"),
                "reject_codes": row.get("reject_codes"),
                "has_frame": row.get("frame_stamp") is not None,
            }
        )
    return samples


def _mission_goals(run: dict | None, owners: list[str], log_goals: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    if run and run.get("drones"):
        goals = {}
        for i, d in enumerate(run["drones"]):
            key = owners[i] if i < len(owners) else d["id"]
            goals[d["id"]] = np.asarray(d["goal_enu"], dtype=float)
            goals[key] = goals[d["id"]]
        return goals
    return log_goals


def _ids(run: dict | None, owners: list[str]) -> list[str]:
    if run and run.get("drones") and len(run["drones"]) == len(owners):
        return [d["id"] for d in run["drones"]]
    if run and run.get("drones") and len(owners) <= 1:
        return [d["id"] for d in run["drones"]]
    return owners or ["uav0"]


def _score(samples_by_uav: dict[str, list[dict]], goals: dict[str, np.ndarray], run: dict | None, obstacles) -> dict:
    crit = (run or {}).get("criteria") or {}
    radius = float(crit.get("body_radius_m", 0.35))
    gtol = float(crit.get("goal_tolerance_m", 0.4))
    gspeed = float(crit.get("goal_speed_mps", 0.2))
    ghold = float(crit.get("goal_hold_s", 1.0))
    htol = float(crit.get("hover_tolerance_m", 0.4))
    task = (run or {}).get("task") or "goto"
    ground_z = float((run or {}).get("ground_z", 0.0))
    outcome, reason = None, ""
    arrivals = []
    min_obs = None
    max_err = 0.0
    final_err = 0.0
    t0 = min(s["t"] for rows in samples_by_uav.values() for s in rows)
    t1 = t0
    for uid, rows in samples_by_uav.items():
        goal = goals.get(uid)
        if goal is None:
            goal = rows[-1].get("goal")
        if goal is None:
            raise L1LogError(f"{uid} 没有目标点（run.json 或规划日志都没写）")
        goal = np.asarray(goal, dtype=float)
        start_xy = rows[0]["p"][:2]
        airborne = False
        goal_since = None
        arrival = None
        for s in rows:
            t1 = max(t1, s["t"])
            p, v = s["p"], s["v"]
            dist = min(o.distance(p) for o in obstacles) if obstacles else float("inf")
            min_obs = dist if min_obs is None else min(min_obs, dist)
            err = float(np.linalg.norm(p - goal))
            max_err = max(max_err, err)
            if p[2] - ground_z > radius + _TRACKING_BOUND:
                airborne = True
            supported = (not airborne) and float(np.linalg.norm(p[:2] - start_xy)) <= _SUPPORT_RADIUS
            hit_box = any(not isinstance(o, Ground) and o.distance(p) <= radius for o in obstacles)
            hit_ground = p[2] - ground_z <= radius and not supported
            if outcome is None and (hit_box or hit_ground):
                outcome, reason = "COLLISION", f"{uid}:TRUTH_POSITION"
            rel = s["t"] - t0
            if task == "goto" and arrival is None and outcome is None:
                if err <= gtol and float(np.linalg.norm(v)) <= gspeed:
                    goal_since = rel if goal_since is None else goal_since
                    if rel - goal_since >= ghold - 1e-9:
                        arrival = goal_since
                else:
                    goal_since = None
        final_err = max(final_err, float(np.linalg.norm(rows[-1]["p"] - goal)))
        arrivals.append(arrival)
    n = len(samples_by_uav)
    inter = None
    if n >= 2:
        # 每个采样时刻用各机当时的最新位置。机间碰撞优先于「还没到」.
        times = sorted({s["t"] for rows in samples_by_uav.values() for s in rows})
        last = {uid: rows[0]["p"] for uid, rows in samples_by_uav.items()}
        idx = {uid: 0 for uid in samples_by_uav}
        uids = list(samples_by_uav)
        for t in times:
            for uid in uids:
                rows = samples_by_uav[uid]
                while idx[uid] + 1 < len(rows) and rows[idx[uid] + 1]["t"] <= t:
                    idx[uid] += 1
                last[uid] = rows[idx[uid]]["p"]
            for i, a in enumerate(uids):
                for b in uids[i + 1 :]:
                    dist = float(np.linalg.norm(last[a] - last[b]))
                    inter = dist if inter is None else min(inter, dist)
                    if outcome is None and dist <= 2.0 * radius:
                        outcome, reason = "COLLISION", f"{a}-{b}:INTER_DRONE"
    if outcome is None and task == "goto":
        if arrivals and all(a is not None for a in arrivals):
            outcome, reason = "SUCCESS", "GOAL_REACHED_AND_HELD"
        else:
            outcome, reason = "TIMEOUT", "GOAL_NOT_REACHED"
    if outcome is None:
        outcome, reason = ("SUCCESS", "HOVER_HELD") if max_err <= htol else ("HOVER_DRIFT", "HOVER_TOLERANCE")
    return {
        "outcome": outcome,
        "reason": reason,
        "task": task,
        "arrived": sum(a is not None for a in arrivals) if task == "goto" else n,
        "arrival_time_s": (max(arrivals) if task == "goto" and arrivals and all(a is not None for a in arrivals) else None),
        "min_obstacle_distance_m": min_obs,
        "max_hover_error_m": None if task == "goto" else max_err,
        "final_goal_error_m": final_err,
        "min_inter_drone_distance_m": inter,
        "sim_duration_s": t1 - t0,
        "n_samples": sum(len(v) for v in samples_by_uav.values()),
    }


def _empty_bus() -> dict:
    return {
        "drop": 0.0,
        "duplicate": 0.0,
        "delay_s": [0.0, 0.0],
        "reorder": 0.0,
        "published": 0,
        "dropped": 0,
        "duplicated": 0,
        "delivered": 0,
        "reordered": 0,
        "peer_deliveries": 0,
    }


def import_l1(log_dir: str | Path) -> dict:
    log_dir = Path(log_dir)
    if not log_dir.is_dir():
        raise L1LogError(f"没有日志目录 {log_dir}")
    run_path = _one(_find(log_dir, ("run.json",)), "run.json")
    flight_path = _one(_find(log_dir, ("l1-flight.json", "flight.json")), "飞行摘要")
    scene_path = _one(_find(log_dir, ("scene.json",)), "scene.json")
    tick_path = _one(_find(log_dir, ("ticks.jsonl",)), "ticks.jsonl")
    planner_paths = [
        p
        for p in _find(log_dir, ("*.jsonl", "*.jsonl.gz"))
        if p.name != "ticks.jsonl" and _looks_like_planner(p)
    ]
    run = _load_json(run_path) if run_path else None
    flight = _load_json(flight_path) if flight_path else None
    if run is None and flight is None and not planner_paths and tick_path is None:
        raise L1LogError(
            f"{log_dir} 里没有 run.json、l1-flight.json、ticks.jsonl，也没有 {PLANNER_SCHEMA} 规划日志"
        )
    if scene_path is None and run and run.get("scene_file"):
        candidate = log_dir / run["scene_file"]
        if candidate.is_file():
            scene_path = candidate
    scene = _load_json(scene_path) if scene_path else {"boxes": []}
    ground_z = float((run or {}).get("ground_z", 0.0))
    obstacles = parse_geometry(scene, ground_z)

    headers = []
    planner_steps: list[tuple[str, dict]] = []
    for i, path in enumerate(planner_paths):
        header, steps = _planner_steps(path)
        headers.append(header)
        owner = ((header.get("config") or {}).get("owner")) or f"px4_{i + 1}"
        for step in steps:
            planner_steps.append((owner, step))

    samples_by: dict[str, list[dict]] = {}
    shadow_ticks: list[dict] = []
    frames = plans = 0
    tick_rows = _tick_samples(tick_path) if tick_path is not None else []
    if tick_rows:
        for s in tick_rows:
            samples_by.setdefault(s["uav"], []).append(s)
            if s["would_reject"] is not None:
                shadow_ticks.append(
                    {"uav": s["uav"], "would_reject": s["would_reject"], "reject_codes": s["reject_codes"] or []}
                )
        plans = len(tick_rows)
        frames = sum(1 for s in tick_rows if s["has_frame"])
    elif planner_steps:
        for owner, step in planner_steps:
            plans += 1
            if step["has_frame"]:
                frames += 1
            sample = step["sample"]
            if sample is None:
                continue
            sample = {**sample, "uav": owner}
            samples_by.setdefault(owner, []).append(sample)
        shadow_ticks = [
            {
                "uav": owner,
                "would_reject": step["diagnostic"] != "APPROVED",
                "reject_codes": [] if step["diagnostic"] == "APPROVED" else [step["diagnostic"]],
            }
            for owner, step in planner_steps
        ]
    owners = list(dict.fromkeys(owner for owner, _ in planner_steps)) or list(samples_by)
    ids = _ids(run, owners)
    # 单机日志的 owner（px4_1）对齐到场景卡的 id（uav0），影子记录和卡用同一个名字。
    if len(ids) == len(samples_by) == 1:
        only = next(iter(samples_by))
        if only != ids[0]:
            samples_by[ids[0]] = samples_by.pop(only)
            for t in shadow_ticks:
                if t["uav"] == only:
                    t["uav"] = ids[0]
    goals = _mission_goals(run, owners, {uid: rows[0]["goal"] for uid, rows in samples_by.items() if rows and rows[0].get("goal") is not None})
    scored = _score(samples_by, goals, run, obstacles) if samples_by else None

    plan_codes: dict[str, int] = {}
    for _owner, step in planner_steps:
        code = step["diagnostic"] or "UNKNOWN"
        plan_codes[code] = plan_codes.get(code, 0) + 1

    if flight and "shadow" in flight:
        shadow_on = bool(flight["shadow"])
    elif run and "shadow" in run:
        shadow_on = bool(run["shadow"])
    else:
        shadow_on = False
    rate = hist = drones_shadow = None
    if shadow_on and shadow_ticks:
        rate, hist = aggregate(shadow_ticks)
        drones_shadow = aggregate_drones(shadow_ticks, ids if ids else sorted({t["uav"] for t in shadow_ticks}))

    network = None
    if headers:
        src = headers[0].get("source") or {}
        network = {
            "upstream_sha": src.get("upstream_sha"),
            "weights_sha256": src.get("weights_sha256"),
            "backend": src.get("name"),
            "l1_code_sha": headers[0].get("code_sha"),
        }

    wall = _float_file(_one(_find(log_dir, ("wall_time_s.txt",)), "wall_time_s.txt"))
    handoff = _float_file(_one(_find(log_dir, ("takeoff_handoff_s.txt",)), "takeoff_handoff_s.txt"))
    sim = None if scored is None else scored["sim_duration_s"]
    rtf = None
    if sim is not None and wall not in (None, 0.0):
        rtf = sim / wall

    identity = run or {}
    result = {
        "commit": identity.get("commit") or git_commit(),
        "scene": identity.get("scene"),
        "scene_card_hash": identity.get("scene_card_hash"),
        "seed": identity.get("seed"),
        "tier": "L1",
        "policy": identity.get("policy"),
        "shadow": shadow_on,
        "dynamics": "px4_sitl",
        "n_drones": len(ids) if ids else (1 if samples_by else None),
        "outcome": None if scored is None else scored["outcome"],
        "reason": "" if scored is None else scored["reason"],
        "task": identity.get("task") if identity.get("task") else (None if scored is None else scored["task"]),
        "arrived": None if scored is None else scored["arrived"],
        "arrival_time_s": None if scored is None else scored["arrival_time_s"],
        "min_obstacle_distance_m": None if scored is None else scored["min_obstacle_distance_m"],
        "max_hover_error_m": None if scored is None else scored["max_hover_error_m"],
        "final_goal_error_m": None if scored is None else scored["final_goal_error_m"],
        "min_inter_drone_distance_m": None if scored is None else scored["min_inter_drone_distance_m"],
        "shadow_reject_rate": rate,
        "shadow_reject_histogram": hist,
        "shadow_gates": None,
        "shadow_drones": drones_shadow,
        "plan_codes": plan_codes,
        "network": network,
        "sim_duration_s": sim if sim is not None else 0.0,
        "ticks": 0 if scored is None else scored["n_samples"],
        "frames": frames,
        "plans": plans,
        "bus": _empty_bus(),
        "wall_time_s": wall,
        "takeoff_handoff_s": handoff,
        "rtf": rtf,
    }
    if flight:
        for key in _OVERRIDE_KEYS:
            if key in flight:
                result[key] = flight[key]
        if "rtf" not in flight and result.get("sim_duration_s") not in (None, 0) and result.get("wall_time_s") not in (None, 0):
            result["rtf"] = float(result["sim_duration_s"]) / float(result["wall_time_s"])
    result["tier"] = "L1"
    if not result["shadow"]:
        result["shadow_reject_rate"] = None
        result["shadow_reject_histogram"] = None
        result["shadow_gates"] = None
        result["shadow_drones"] = None
    if not result.get("scene") or not result.get("scene_card_hash"):
        raise L1LogError("日志里没有场景卡名或哈希。把 export_l1 的 run.json 放进日志目录，或在飞行摘要里写 scene / scene_card_hash")
    if result.get("seed") is None or result.get("policy") is None:
        raise L1LogError("日志里没有 seed 或 policy。它们在 export_l1 的 run.json 里")
    (log_dir / "result.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return result


def _looks_like_planner(path: Path) -> bool:
    try:
        header = next(_rows(path), None)
    except (OSError, json.JSONDecodeError, gzip.BadGzipFile):
        return False
    return bool(header and header.get("schema") == PLANNER_SCHEMA)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="把 L1 日志目录收成与 L0 相同的 result.json（tier=L1）")
    ap.add_argument("log_dir")
    a = ap.parse_args(argv)
    result = import_l1(a.log_dir)
    print(json.dumps({"tier": result["tier"], "outcome": result["outcome"], "result": str(Path(a.log_dir) / "result.json")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
