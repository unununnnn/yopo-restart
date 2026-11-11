"""唯一测试接缝：场景卡 → 运行器 → 结果记录 + 逐 tick 日志。

锁步：每 tick 全体传感 → 全体规划 → 总线投递 → 动力学 → 碰撞真值。墙钟不计入仿真时间。

计划不直接交给动力学。策略返回的对象留在本机私有槽（upstream 的偏航、certified 的
runtime 都挂在这个对象上，零故障时必须原对象进 step）。总线上只放普通 dict：
本机一条 `self_plan` 令牌，他机一条不含对象身份的 `peer_plan` 公告。
令牌被延迟、丢失、重复或乱序后，动力学才改用对应的槽；他机公告不写入批准计划，
也不进入 Observation。没有已投递的本机计划时，动力学保持当前点。

票 10 的 ROS 2 形态不另写一套循环。`bridge is None`（默认）时下面两个调用点都不进入，
进程内结果与加钩子之前相同。桥只做两件事：每个 tick 传感之前 `begin_tick(k, now)`，
规划那一步 `plan(drone_id, obs)` 代替本函数直接调用 `policy.plan`。
`plan` 必须调用该机字典里的同一个策略实例并返回它交出的计划对象，
这样 `last_code` / `extra_codes` 和动力学 `sample` 的对象身份都不变。
调用方在 `run` 返回后调用 `bridge.close()`（本函数不调用）。
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
from .geom_fallback import FIELD as GEOM_FIELD
from .geom_fallback import KIND as GEOM_KIND
from .geom_fallback import GeomSession
from .policy import (
    POLICIES,
    Observation,
    PolicyContext,
    deviates_from_upstream,
    empty_solution_count,
    read_safe_radius,
)
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
    safe_radius_m: float | None = None  # None = 场景卡，缺省 0.05 m（上游 d5e1e60）
    # None = 场景卡 geometric_fallback.enabled，缺省关。True/False 盖过场景卡，不改卡哈希。
    geometric_fallback: bool | None = None


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


def _card_for_run(card_dict: dict, cfg: RunConfig) -> dict:
    """CLI 的 safe_radius / 几何回退盖过场景卡。不改传入的卡，场景卡哈希保持原样。

    两个开关都缺省时返回原字典，避免多一次拷贝。
    """
    if cfg.safe_radius_m is None and cfg.geometric_fallback is None:
        return card_dict
    out = card_dict
    if cfg.safe_radius_m is not None:
        params = dict(out.get("policy_params") or {})
        upstream = dict(params.get("upstream") or {})
        upstream["safe_radius"] = float(cfg.safe_radius_m)
        params["upstream"] = upstream
        out = {**out, "policy_params": params}
    if cfg.geometric_fallback is not None:
        block = dict(out.get("geometric_fallback") or {})
        block["enabled"] = bool(cfg.geometric_fallback)
        out = {**out, "geometric_fallback": block}
    return out


def run(card: SceneCard, cfg: RunConfig, *, bridge=None) -> RunOutput:
    if cfg.tier != "L0":
        raise ValueError("本运行器只实现 L0")
    wall0 = time.perf_counter()
    c = card.card
    tuned = _card_for_run(c, cfg)
    rates = c["rates_hz"]
    phys, cam_hz, plan_hz = int(rates["physics"]), int(rates["camera"]), int(rates["planner"])
    dt = 1.0 / phys
    duration = float(cfg.duration_s if cfg.duration_s is not None else c["task"]["duration_s"])
    n_ticks = int(round(duration * phys))
    task = c["task"]
    crit = c["criteria"]
    radius = float(crit["body_radius_m"])
    # formation 走票 06 的本机预留协议，不进 YOPO。
    # crossing 走同一套预留，但提案内容是本机深度上的上游 YOPO，槽位只偏置目标。
    # switch 走同一套预留：窄门宽度触发横排↔纵列，槽位重分配经 ACK/COMMIT，不瞬移。
    # 判到达的口径和 goto 一样。
    fleet = None
    approval_cls = None
    if task["type"] == "formation":
        from .formation import FormationFleet, LocalApproval

        fleet = FormationFleet.from_card(tuned, dt, record_inputs=cfg.record_planner_inputs)
        approval_cls = LocalApproval
    elif task["type"] == "crossing":
        from .crossing import CrossingFleet
        from .crossing import LocalApproval as CrossingApproval

        fleet = CrossingFleet.from_card(tuned, dt, record_inputs=cfg.record_planner_inputs)
        approval_cls = CrossingApproval
    elif task["type"] == "switch":
        from .switch import SwitchFleet
        from .switch import LocalApproval as SwitchApproval

        fleet = SwitchFleet.from_card(tuned, dt, record_inputs=cfg.record_planner_inputs)
        approval_cls = SwitchApproval
    track_goal = task["type"] in ("goto", "formation", "crossing", "switch")
    if track_goal:
        gtol, gspeed, ghold = float(crit["goal_tolerance_m"]), float(crit["goal_speed_mps"]), float(crit["goal_hold_s"])
    else:
        tol = float(crit["hover_tolerance_m"])
    model = make_model(c["dynamics"])
    cam_cfg = CameraConfig.from_card(c["camera"])

    root = np.random.SeedSequence(cfg.seed)
    drones = []
    for d, ss in zip(c["drones"], root.spawn(len(c["drones"])), strict=True):
        start = np.asarray(d["start_enu"], dtype=float)
        if fleet is not None:
            fleet.check_start(d["id"], start)
            goal = fleet.goal_of(d["id"])
            policy = approval_cls()
        else:
            goal = np.asarray(d["goal_enu"] if track_goal else d.get("hover_enu", start), dtype=float)
            ctx = PolicyContext(d["id"], dt, dict(tuned.get("policy_params", {})))
            policy = POLICIES[cfg.policy](ctx)
        drones.append({
            "id": d["id"],
            "goal": goal,  # goto / formation：目标点；hover：悬停点，默认 = 起点
            "state": State(0.0, start.copy(), np.zeros(3), np.zeros(3), 0.0),
            "camera": DepthCamera(cam_cfg, card.obstacles, np.random.default_rng(ss)),
            "policy": policy,
            "codes": {}, "goal_since": None, "arrival": None,
            "frame": None, "plan": None, "approved": None,
            "min_obs": card.min_distance(start), "max_err": 0.0,
            "shadow": (
                DroneShadow(d["id"], start, goal, dict(c.get("policy_params", {}))) if cfg.shadow else None
            ),
        })
    by_id = {dr["id"]: dr for dr in drones}
    # goto / hover 没有协议节点。关着时不采样。舰队自己持有会话，这里不再建一个。
    geom = None
    if fleet is None:
        geom = GeomSession.from_card(
            tuned,
            members=tuple(d["id"] for d in c["drones"]),
            physics_dt=dt,
            obstacles=card.obstacles,
        )
    faults = _faults(card, cfg)
    bus = MessageBus(faults, rng=np.random.default_rng(np.random.SeedSequence([cfg.seed, _BUS_SEED_SALT])))
    slots: list = []  # 本机计划对象。不下发到他机，也不放进总线 payload。
    # 票 10：策略实例已经建好。None 时不进入。
    if bridge is not None:
        bridge.bind(drones)

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
            if geom is not None and geom.enabled:
                geom.flush(bus, now)
            if bridge is not None:
                bridge.begin_tick(k, now)
            for dr in drones:
                s = dr["state"]
                if _due(k, cam_hz, phys):
                    dr["frame"] = dr["camera"].capture(s.p, s.yaw, now)
                    frames += 1
                fresh = False
                if fleet is None and (_due(k, plan_hz, phys) or dr["plan"] is None):
                    # 目标是本机数组的拷贝。Observation 里没有他机状态，也没有几何。
                    obs = Observation(dr["frame"], s, dr["goal"].copy(), now)
                    if cfg.record_planner_inputs:
                        inputs.append(obs.digest())
                    if bridge is None:
                        proposed = dr["policy"].plan(obs)
                    else:
                        proposed = bridge.plan(dr["id"], obs)
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
                if fleet is None and dr["shadow"] is not None and fresh:
                    dr["shadow"].on_selection(dr["plan"], dr["frame"], s, now)
            if fleet is None:
                for msg in bus.poll(now):
                    kind = msg.payload.get("kind")
                    if geom is not None and geom.enabled and kind == GEOM_KIND:
                        geom.ingest_envelope(msg)
                        continue
                    if kind == "self_plan" and msg.src == msg.dest and msg.dest in by_id:
                        proposed = slots[int(msg.payload["token"])]
                        if geom is not None and geom.enabled:
                            chosen = geom.review_goto(msg.dest, proposed, now)
                            # 回退之后，下一次前向要从正在飞的曲线取头状态，而不是从被拒的那条。
                            policy = by_id[msg.dest]["policy"]
                            if chosen is not proposed and hasattr(policy, "plan_obj"):
                                policy.plan_obj = chosen
                            proposed = chosen
                        by_id[msg.dest]["approved"] = proposed
                    else:
                        peer_deliveries += 1
            else:
                # 提案 / ACK / COMMIT 只经总线。本机 COMMIT 之后 executing() 才换计划。
                # crossing 在这里收本机深度。编队保持没有 ingest，不进这条。
                ingest = getattr(fleet, "ingest", None)
                if ingest is not None:
                    for dr in drones:
                        if dr["frame"] is not None:
                            ingest(dr["id"], dr["frame"], dr["state"])
                fleet.exchange(bus, now)
            for dr in drones:
                s = dr["state"]
                if fleet is not None:
                    dr["approved"] = fleet.track(dr["id"], now)
                    dr["policy"].last_code = fleet.code(dr["id"])
                    if _due(k, plan_hz, phys):
                        code = dr["policy"].last_code
                        dr["codes"][code] = dr["codes"].get(code, 0) + 1
                    # 复验刚在本地提交的计划。视图不进动力学，影子也不改那份计划。
                    view = fleet.shadow_view(dr["id"])
                    if dr["shadow"] is not None and view is not None:
                        dr["shadow"].on_selection(view, dr["frame"], s, now)
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
                row = {
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
                }
                if fleet is not None:
                    rec = fleet.last_record(dr["id"])
                    row["approved_owner"] = rec.owner
                    row["approved_seq"] = int(rec.plan_seq)
                    row["formation_error_m"] = fleet.observe(dr["id"], s.t, s.p)
                ticks.append(row)
                if outcome is None and dist <= radius:
                    outcome, reason = "COLLISION", f"{dr['id']}:TRUTH_POSITION"
                elif track_goal and dr["arrival"] is None and outcome is None:
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
            if track_goal and all(d["arrival"] is not None for d in drones):
                outcome, reason = "SUCCESS", "GOAL_REACHED_AND_HELD"
                break
    if outcome is None and track_goal:
        outcome, reason = "TIMEOUT", "GOAL_NOT_REACHED"
    if outcome is None:
        worst = max(d["max_err"] for d in drones)
        outcome, reason = ("SUCCESS", "HOVER_HELD") if worst <= tol else ("HOVER_DRIFT", "HOVER_TOLERANCE")

    arrived = sum(d["arrival"] is not None for d in drones)
    fleet_block = None
    policy_name = cfg.policy
    if fleet is not None:
        fleet_block = fleet.summary(arrived, len(drones))
        plans = fleet_block["proposals"]
        peer_deliveries = fleet.deliveries
        policy_name = getattr(fleet, "policy_name", "formation")
        if cfg.record_planner_inputs:
            inputs.extend(fleet.input_digests)
    net_name = "upstream" if fleet is not None and getattr(fleet, "uses_network", False) else policy_name
    result = {
        "commit": git_commit(),
        "scene": card.name,
        "scene_card_hash": card.hash,
        "seed": cfg.seed,
        "tier": cfg.tier,
        "policy": policy_name,
        "shadow": cfg.shadow,
        "dynamics": c["dynamics"]["model"],
        "n_drones": len(drones),
        "outcome": outcome,
        "reason": reason,
        "task": task["type"],
        "arrived": arrived,
        "arrival_time_s": (max(d["arrival"] for d in drones) if track_goal and arrived == len(drones) else None),
        "min_obstacle_distance_m": min(d["min_obs"] for d in drones),
        "max_hover_error_m": None if track_goal else max(d["max_err"] for d in drones),
        "final_goal_error_m": max(float(np.linalg.norm(d["state"].p - d["goal"])) for d in drones),
        "min_inter_drone_distance_m": min_inter,  # 单机为 None
        # 关 = null，不是 0。0.0 只表示「开了，但没有任何 tick 会拒」。
        "shadow_reject_rate": None,
        "shadow_reject_histogram": None,
        "shadow_gates": None,
        "shadow_drones": None,  # 开：每机一行；关：null
        "plan_codes": {k: v for d in drones for k, v in sorted(
            {**d["codes"], **getattr(d["policy"], "extra_codes", {})}.items())},
        "network": _net_provenance(net_name),
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
    if fleet is not None and fleet.geom.enabled:
        result[GEOM_FIELD] = fleet.geom.summary()
    elif geom is not None and geom.enabled:
        result[GEOM_FIELD] = geom.summary()
    if isinstance(c.get("forest"), dict):
        result["forest_density"] = str(c["forest"].get("density", "medium"))
    if cfg.shadow:
        rate, hist = aggregate(ticks)
        result["shadow_reject_rate"] = rate
        result["shadow_reject_histogram"] = hist
        result["shadow_gates"] = gate_status()
        result["shadow_drones"] = aggregate_drones(ticks, [d["id"] for d in drones])
    if fleet_block is not None:
        if fleet_block.get("body_radius_m") is not None:
            body = float(fleet_block["body_radius_m"])
            fleet_block["min_obstacle_clearance_m"] = float(result["min_obstacle_distance_m"] - body)
            gap = result["min_inter_drone_distance_m"]
            fleet_block["obstacle_collisions"] = int(result["min_obstacle_distance_m"] <= body + 1e-9)
            fleet_block["inter_drone_collisions"] = int(gap is not None and gap <= 2.0 * body + 1e-9)
        if fleet_block.get("decisions"):
            result["plan_codes"] = dict(fleet_block["decisions"])
        result[getattr(fleet, "result_key", "formation")] = fleet_block
        result["formation_error_p95_m"] = fleet_block["error_p95_m"]
        result["arrival_rate"] = fleet_block["arrival_rate"]
    uses_corridor = cfg.policy == "upstream" or (
        fleet is not None and getattr(fleet, "uses_network", False)
    )
    if uses_corridor:
        radius_m = read_safe_radius(tuned.get("policy_params"))
        result["safe_radius_m"] = radius_m
        result["upstream_deviation"] = deviates_from_upstream(radius_m)
        if fleet is not None and hasattr(fleet, "empty_solution"):
            result["empty_solution_count"] = int(fleet.empty_solution)
        else:
            result["empty_solution_count"] = empty_solution_count(result["plan_codes"])
    return RunOutput(result, ticks, inputs)


def write(out: RunOutput, out_dir: str | Path) -> Path:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "result.json").write_text(json.dumps(out.result, indent=2, ensure_ascii=False) + "\n")
    with (out_dir / "ticks.jsonl").open("w") as f:
        for t in out.ticks:
            f.write(json.dumps(t, separators=(",", ":")) + "\n")
    return out_dir
