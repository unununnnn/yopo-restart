"""唯一测试接缝：场景卡 → 运行器 → 结果记录 + 逐 tick 日志。

锁步：每 tick 全体传感 → 全体规划 → 总线投递 → 动力学 → 碰撞真值。墙钟不计入仿真时间。

计划不直接交给动力学。策略返回的对象留在本机私有槽（upstream 的偏航、certified 的
runtime 都挂在这个对象上，零故障时必须原对象进 step）。总线上只放普通 dict：
本机一条 `self_plan` 令牌，他机一条不含对象身份的 `peer_plan` 公告。
令牌被延迟、丢失、重复或乱序后，动力学才改用对应的槽；他机公告不写入批准计划，
也不进入 Observation。没有已投递的本机计划时，动力学保持当前点。
"""

from __future__ import annotations

import json
import subprocess
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from .bus import BusFaults, MessageBus
from .dynamics import HoverPlan, State, make_model
from .policy import POLICIES, Observation, PolicyContext
from .scene import SceneCard
from .sensor import CameraConfig, DepthCamera
from .shadow import DroneShadow, aggregate, aggregate_drones, approved_pva, gate_status

WALL_FIELDS = frozenset({"wall_time_s"})  # 确定性比较时排除的字段
REPO = Path(__file__).resolve().parents[1]
# 与传感器 SeedSequence(cfg.seed) 分开，拨总线旋钮不改深度噪声。
_BUS_SEED_SALT = 0xB05


@dataclass
class RunConfig:
    seed: int = 0
    policy: str = "upstream"  # L0 默认 upstream + 影子盾（frank 拍板 / 父代理代定）；L1 不变
    shadow: bool = True  # 默认开。只记录会否拒绝，不改批准动作
    tier: str = "L0"
    duration_s: float | None = None  # None = 用场景卡 task.duration_s
    record_planner_inputs: bool = False
    bus: BusFaults | None = None  # None = 场景卡 bus，缺省零故障


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


def _net_provenance(policy: str):
    if policy in ("upstream", "certified"):
        from .net import provenance

        return provenance()
    return None


def _due(k: int, rate: int, physics: int) -> bool:
    return k == 0 or (k * rate) // physics != ((k - 1) * rate) // physics


def _faults(card: SceneCard, cfg: RunConfig) -> BusFaults:
    if cfg.bus is not None:
        return cfg.bus
    return BusFaults.from_mapping(card.card.get("bus"))


def _note_pairs(drones: list[dict], radius: float, min_inter: float | None, outcome, reason):
    """两架机体半径 r 的飞机，中心距 <= 2r 判机间碰撞。"""
    if len(drones) < 2:
        return min_inter, outcome, reason
    for i, a in enumerate(drones):
        pa = a["state"].p
        for b in drones[i + 1 :]:
            dist = float(np.linalg.norm(pa - b["state"].p))
            min_inter = dist if min_inter is None else min(min_inter, dist)
            if outcome is None and dist <= 2.0 * radius:
                outcome, reason = "COLLISION", f"{a['id']}-{b['id']}:INTER_DRONE"
    return min_inter, outcome, reason


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
    task = c["task"]
    crit = c["criteria"]
    radius = float(crit["body_radius_m"])
    goto = task["type"] == "goto"
    if goto:
        gtol, gspeed, ghold = float(crit["goal_tolerance_m"]), float(crit["goal_speed_mps"]), float(crit["goal_hold_s"])
    else:
        tol = float(crit["hover_tolerance_m"])
    model = make_model(c["dynamics"])
    cam_cfg = CameraConfig.from_card(c["camera"])

    root = np.random.SeedSequence(cfg.seed)
    drones = []
    for d, ss in zip(c["drones"], root.spawn(len(c["drones"])), strict=True):
        start = np.asarray(d["start_enu"], dtype=float)
        goal = np.asarray(d["goal_enu"] if goto else d.get("hover_enu", start), dtype=float)
        ctx = PolicyContext(d["id"], dt, dict(c.get("policy_params", {})))
        drones.append({
            "id": d["id"],
            "goal": goal,  # goto：目标点；hover：悬停点，默认 = 起点
            "state": State(0.0, start.copy(), np.zeros(3), np.zeros(3), 0.0),
            "camera": DepthCamera(cam_cfg, card.obstacles, np.random.default_rng(ss)),
            "policy": POLICIES[cfg.policy](ctx),
            "codes": {}, "goal_since": None, "arrival": None,
            "frame": None, "plan": None, "approved": None,
            "min_obs": card.min_distance(start), "max_err": 0.0,
            "shadow": (
                DroneShadow(d["id"], start, goal, dict(c.get("policy_params", {}))) if cfg.shadow else None
            ),
        })
    by_id = {dr["id"]: dr for dr in drones}
    faults = _faults(card, cfg)
    bus = MessageBus(faults, rng=np.random.default_rng(np.random.SeedSequence([cfg.seed, _BUS_SEED_SALT])))
    slots: list = []  # 本机计划对象。不下发到他机，也不放进总线 payload。

    ticks: list[dict] = []
    inputs: list[str] = []
    outcome, reason = None, ""
    frames = plans = peer_deliveries = 0
    min_inter: float | None = None
    min_inter, outcome, reason = _note_pairs(drones, radius, min_inter, outcome, reason)
    ran = False
    k = 0
    if outcome is None:
        for k in range(n_ticks):
            ran = True
            now = k / phys
            for dr in drones:
                s = dr["state"]
                if _due(k, cam_hz, phys):
                    dr["frame"] = dr["camera"].capture(s.p, s.yaw, now)
                    frames += 1
                fresh = False
                if _due(k, plan_hz, phys) or dr["plan"] is None:
                    # 目标是本机数组的拷贝。Observation 里没有他机状态，也没有几何。
                    obs = Observation(dr["frame"], s, dr["goal"].copy(), now)
                    if cfg.record_planner_inputs:
                        inputs.append(obs.digest())
                    proposed = dr["policy"].plan(obs)
                    dr["plan"] = proposed
                    code = dr["policy"].last_code
                    dr["codes"][code] = dr["codes"].get(code, 0) + 1
                    plans += 1
                    fresh = True
                    slots.append(proposed)
                    token = len(slots) - 1
                    for dest in drones:
                        if dest["id"] == dr["id"]:
                            payload = {"kind": "self_plan", "token": token}
                        else:
                            payload = {"kind": "peer_plan", "code": code}
                        bus.publish(now, src=dr["id"], dest=dest["id"], topic="plan", payload=payload)
                # 只读复验本机刚选出的计划。不碰总线，也不改 proposed。
                if dr["shadow"] is not None and fresh:
                    dr["shadow"].on_selection(dr["plan"], dr["frame"], s, now)
            for msg in bus.poll(now):
                kind = msg.payload.get("kind")
                if kind == "self_plan" and msg.src == msg.dest and msg.dest in by_id:
                    by_id[msg.dest]["approved"] = slots[int(msg.payload["token"])]
                else:
                    peer_deliveries += 1
            for dr in drones:
                s = dr["state"]
                plan = dr["approved"]
                if plan is None:
                    plan = HoverPlan(tuple(float(x) for x in s.p), float(s.yaw))
                # 批准位置：不调用 sample。_TrajPlan.sample / command 会改航向和时钟。
                cmd_p, cmd_v, cmd_a = approved_pva(plan, s.t)
                codes = None
                if dr["shadow"] is not None:
                    codes = dr["shadow"].on_tick(s, cmd_p, now)
                s = model.step(s, plan, dt)
                s = replace(s, t=(k + 1) / phys)  # 时间戳只取仿真时钟整数 tick，不累加浮点
                dr["state"] = s
                dist = card.min_distance(s.p)
                err = float(np.linalg.norm(s.p - dr["goal"]))
                dr["min_obs"] = min(dr["min_obs"], dist)
                dr["max_err"] = max(dr["max_err"], err)
                ticks.append({
                    "k": k, "t": round(s.t, 9), "uav": dr["id"], "p": s.p.tolist(), "v": s.v.tolist(),
                    "a": s.a.tolist(), "frame_stamp": None if dr["frame"] is None else dr["frame"].stamp,
                    "min_obstacle_m": dist, "goal_error_m": err, "yaw": s.yaw,
                    "plan_code": dr["policy"].last_code,
                    "cmd_p": [float(x) for x in np.asarray(cmd_p, dtype=float)],
                    "cmd_v": [float(x) for x in np.asarray(cmd_v, dtype=float)],
                    "cmd_a": [float(x) for x in np.asarray(cmd_a, dtype=float)],
                    # 关：null。开：空列表表示查了且不拒。
                    "would_reject": None if codes is None else bool(codes),
                    "reject_codes": None if codes is None else list(codes),
                })
                if outcome is None and dist <= radius:
                    outcome, reason = "COLLISION", f"{dr['id']}:TRUTH_POSITION"
                elif goto and dr["arrival"] is None and outcome is None:
                    # G2 oracle 口径：距目标 ≤ 0.4 m 且速度 ≤ 0.2 m/s，连续保持 1 s
                    if err <= gtol and float(np.linalg.norm(s.v)) <= gspeed:
                        dr["goal_since"] = s.t if dr["goal_since"] is None else dr["goal_since"]
                        if s.t - dr["goal_since"] >= ghold - 1e-9:
                            dr["arrival"] = dr["goal_since"]
                    else:
                        dr["goal_since"] = None
            min_inter, outcome, reason = _note_pairs(drones, radius, min_inter, outcome, reason)
            if outcome:
                break
            if goto and all(d["arrival"] is not None for d in drones):
                outcome, reason = "SUCCESS", "GOAL_REACHED_AND_HELD"
                break
    if outcome is None and goto:
        outcome, reason = "TIMEOUT", "GOAL_NOT_REACHED"
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
        "task": task["type"],
        "arrived": sum(d["arrival"] is not None for d in drones),
        "arrival_time_s": (max(d["arrival"] for d in drones) if goto and all(d["arrival"] is not None for d in drones) else None),
        "min_obstacle_distance_m": min(d["min_obs"] for d in drones),
        "max_hover_error_m": None if goto else max(d["max_err"] for d in drones),
        "final_goal_error_m": max(float(np.linalg.norm(d["state"].p - d["goal"])) for d in drones),
        "min_inter_drone_distance_m": min_inter,  # 单机为 None
        # 关 = null，不是 0。0.0 只表示「开了，但没有任何 tick 会拒」。
        "shadow_reject_rate": None,
        "shadow_reject_histogram": None,
        "shadow_gates": None,
        "shadow_drones": None,  # 开：每机一行；关：null
        "plan_codes": {k: v for d in drones for k, v in sorted(
            {**d["codes"], **getattr(d["policy"], "extra_codes", {})}.items())},
        "network": _net_provenance(cfg.policy),
        "sim_duration_s": (k + 1) / phys if ran else 0.0,
        "ticks": k + 1 if ran else 0,
        "frames": frames,
        "plans": plans,
        "bus": {
            "drop": faults.drop,
            "duplicate": faults.duplicate,
            "delay_s": [float(faults.delay[0]), float(faults.delay[1])],
            "reorder": faults.reorder,
            **bus.counts(),
            "peer_deliveries": peer_deliveries,
        },
        "wall_time_s": time.perf_counter() - wall0,
    }
    if cfg.shadow:
        rate, hist = aggregate(ticks)
        result["shadow_reject_rate"] = rate
        result["shadow_reject_histogram"] = hist
        result["shadow_gates"] = gate_status()
        result["shadow_drones"] = aggregate_drones(ticks, [d["id"] for d in drones])
    return RunOutput(result, ticks, inputs)


def write(out: RunOutput, out_dir: str | Path) -> Path:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "result.json").write_text(json.dumps(out.result, indent=2, ensure_ascii=False) + "\n")
    with (out_dir / "ticks.jsonl").open("w") as f:
        for t in out.ticks:
            f.write(json.dumps(t, separators=(",", ":")) + "\n")
    return out_dir
