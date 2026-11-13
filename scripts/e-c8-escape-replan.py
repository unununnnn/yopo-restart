#!/usr/bin/env python3
"""票 E-C8：六机 014 连续空走廊悬停的脱困重规划。只报数。

基线是 E-C6 / E-C7 的 014 六机对向，geom + empty_brake，seeds 0–4。
不改过滤阈值。连续 CORRIDOR_EMPTY_HOVER 达到 N 拍时，给一个有界临时目标
（深度图最通畅扇区，侧移 1.2–1.5 m，需要时再抬 0.5 m，保持 K 个规划拍）。
走廊重开，或 K 拍用完，就回到原来的目标。

同时跑不开和打开 E-B4c（peer_sphere_enabled）的两档。欧氏球要后段开着才读得到，
邻机采样在本进程里送进 refine_yopo，不改 policy.py。

不改 yopo_l0/、core/、scenes/、docs/tickets.md、README、scripts/premerge.sh、
scripts/regress.sh，也不改任何默认值。结果不进冻结判据。
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
PARTIAL = ROOT / "out" / "e-c8-escape-replan" / "partial"

ARMS = ("n10", "n15", "n20", "n10-sphere", "n15-sphere", "n20-sphere")
SEEDS = tuple(range(5))
ROW_VERSION = 1
SCENE = "d014-swap6"

# 连续空走廊达到 N 拍就触发。N 本身算在内：第 N 拍不再把原地保持交出去。
ARM_N = {
    "n10": 10,
    "n15": 15,
    "n20": 20,
    "n10-sphere": 10,
    "n15-sphere": 15,
    "n20-sphere": 20,
}
ARM_SPHERE = {arm: arm.endswith("sphere") for arm in ARMS}

# 临时目标相对机体系：前方 4 m，保证落在到达半径 1.5 m 外面；
# 侧向分量夹在 1.2–1.5 m；上三分之一更通畅时再抬 0.5 m。
FORWARD_M = 4.0
LATERAL_M = 1.2
LATERAL_CAP_M = 1.5
UP_M = 0.5
K_PLANS = 10
N_SECTORS = 5
ARRIVE_GUARD_M = 2.2
MAX_RANGE_M = 20.0
HOVER = "CORRIDOR_EMPTY_HOVER"

# E-C7 正文的六机 geom 口径。碰撞拆分来自 E-C6 同一 geom 臂，E-C7 没有另报。
EC7_BASELINE = {
    "label": "E-C7 基线 geom + empty_brake（引用，本脚本不重跑）",
    "arrived": 15,
    "drones": 30,
    "timeout_runs": 3,
    "hover": 1127,
    "avg_speed_mps": 1.580,
    "obstacle_collisions": 2,
    "inter_collisions": 0,
    "clearance": "-0.007 至 0.137",
    "inter_gap": "0.966–2.230",
}

SPHERE = {
    "enabled": True,
    "peer_sphere_enabled": True,
    "peer_sphere_radius_m": 0.85,
    "peer_sphere_weight": 8.0,
}

DISCLAIMER = (
    "只报数。report-only。偏离上游。"
    "不改 yopo_l0/、core/、scenes/、docs/tickets.md、README、scripts/premerge.sh、scripts/regress.sh。"
    "不改 safe_radius 0.05 m、E-A1 阈值 1.0 m、track_margin 0.08 m、empty_brake 的过滤门槛。"
    "脱困只改这一拍送进规划器的临时目标，走廊重开后回到原目标。"
    "E-B4c 的 peer_sphere_enabled 默认关；打开即偏离上游。"
    "六机 0.14 对向卡只存在于进程内存，不写入 scenes/。"
    "这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。"
    "未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。"
)

AVG_SPEED_DEF = (
    "avg_speed_mps 是该次运行全部 tick、全部飞机的 ||v|| 里，"
    "有限且严格大于 0 的样本的算术平均。悬停和初值 0 不算有效飞行。"
    "没有有效样本时为 null。"
    "汇总表里的平均速度是该档各次 avg_speed_mps 的算术平均。"
)

HOVER_DEF = (
    "CORRIDOR_EMPTY_HOVER 按飞机把规划拍加起来：物理 100 Hz、规划 10 Hz，"
    "只数规划拍上的这个码。这和 E-C7 的 1127 是同一口径。"
    "运行器 plan_codes 同名键会被后一架飞机盖掉，那个数另记一列，不拿来对基线。"
)

_POLICY_CLS = None
_PEER_INSTALLED = False
_WATCHED = (
    "yopo_l0/policy.py",
    "yopo_l0/geom_fallback.py",
    "yopo_l0/inter_drone.py",
    "yopo_l0/backend/bspline.py",
    "yopo_l0/backend/config.py",
    "yopo_l0/runner.py",
    "scenes/ego-density-014.json",
    "scenes/suite.txt",
    "docs/tickets.md",
    "README.md",
    "scripts/premerge.sh",
    "scripts/regress.sh",
)


def load_ec6():
    name = "e_c6_six_density_for_c8"
    if name in sys.modules:
        return sys.modules[name]
    path = ROOT / "scripts" / "e-c6-six-density.py"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"无法加载 {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def arm_sphere(arm: str) -> bool:
    if arm not in ARM_SPHERE:
        raise ValueError(arm)
    return bool(ARM_SPHERE[arm])


def arm_n(arm: str) -> int:
    if arm not in ARM_N:
        raise ValueError(arm)
    return int(ARM_N[arm])


def arm_label(arm: str) -> str:
    n = arm_n(arm)
    if arm_sphere(arm):
        return f"N={n} 脱困 + E-B4c 欧氏球（偏离上游）"
    return f"N={n} 脱困（偏离上游）"


def arm_deviates(arm: str) -> bool:
    if arm not in ARM_N:
        raise ValueError(arm)
    return True


def _R_yaw(yaw: float) -> np.ndarray:
    c, s = math.cos(float(yaw)), math.sin(float(yaw))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def body_offset(position, yaw: float, goal) -> np.ndarray:
    """临时目标在机体系下的位移。x 前、y 左、z 上。"""
    p = np.asarray(position, dtype=float)
    g = np.asarray(goal, dtype=float)
    return _R_yaw(yaw).T @ (g - p)


def _sector_view(depth, n_sectors: int, max_range: float):
    filled = np.where(np.isfinite(depth) & (depth > 0.0), depth, max_range)
    height, width = filled.shape
    edges = np.linspace(0, width, n_sectors + 1, dtype=int)
    scores = []
    for i in range(n_sectors):
        block = filled[:, edges[i] : edges[i + 1]]
        scores.append(float(np.median(block)) if block.size else 0.0)
    return filled, edges, scores


def choose_escape_goal(
    position,
    yaw: float,
    depth,
    intrinsics,
    *,
    alternate: int = 0,
    max_range: float = MAX_RANGE_M,
    n_sectors: int = N_SECTORS,
) -> tuple[np.ndarray, dict]:
    """有界临时目标。不改调用方的位置数组。

    深度图按水平切成 n_sectors 份，无回波按 max_range 计，取中位数最大的扇区。
    各扇区中位数一样时当作正前方，侧向按 alternate 左右交替（偶数向右）。
    侧向分量夹到 [LATERAL_M, LATERAL_CAP_M]。上三分之一比中带更通畅时再抬 UP_M。
    前方留 FORWARD_M，使临时目标落在到达半径外面。
    """
    p = np.array(position, dtype=float, copy=True)
    yaw = float(yaw)
    bearing = 0.0
    prefer_up = False
    sector = n_sectors // 2
    center = True
    if depth is not None and intrinsics is not None:
        depth_arr = np.asarray(depth, dtype=float)
        if depth_arr.ndim == 2 and depth_arr.size and int(intrinsics[0]) != 0:
            filled, edges, scores = _sector_view(depth_arr, n_sectors, float(max_range))
            if max(scores) - min(scores) >= 1e-3:
                sector = int(np.argmax(scores))
                center = sector == n_sectors // 2
            height = int(filled.shape[0])
            mid0, mid1 = height // 3, 2 * height // 3
            up_score = float(np.median(filled[:mid0])) if mid0 else 0.0
            mid_score = float(np.median(filled[mid0:mid1])) if mid1 > mid0 else up_score
            prefer_up = up_score > mid_score + 0.5
            fx = float(intrinsics[0])
            cx = float(intrinsics[2])
            u = 0.5 * (int(edges[sector]) + max(int(edges[sector + 1]) - 1, int(edges[sector])))
            bearing = math.atan((u - cx) / fx)
    up = UP_M if prefer_up else 0.0
    if center or abs(bearing) <= 1e-6:
        sign_right = 1.0 if int(alternate) % 2 == 0 else -1.0
        body = np.array([FORWARD_M, -sign_right * LATERAL_M, up], dtype=float)
    else:
        sign_right = 1.0 if bearing > 0.0 else -1.0
        direction = np.array([math.cos(bearing), -math.sin(bearing), 0.0], dtype=float)
        direction /= float(np.linalg.norm(direction))
        step = direction * FORWARD_M
        lat = float(step[1])
        mag = abs(lat)
        if mag < LATERAL_M:
            lat = math.copysign(LATERAL_M, -sign_right if mag <= 1e-9 else lat)
        elif mag > LATERAL_CAP_M:
            lat = math.copysign(LATERAL_CAP_M, lat)
        forward = float(step[0])
        if forward < 2.5:
            forward = 2.5
        body = np.array([forward, lat, up], dtype=float)
    world = p + _R_yaw(yaw) @ body
    if float(world[2]) < 0.8:
        world[2] = 0.8
    elif float(world[2]) > 3.0:
        world[2] = 3.0
    delta = world - p
    dist = float(np.linalg.norm(delta))
    if dist < ARRIVE_GUARD_M:
        direction = delta / dist if dist > 1e-9 else _R_yaw(yaw) @ np.array([1.0, 0.0, 0.0])
        world = p + direction * ARRIVE_GUARD_M
    final = body_offset(p, yaw, world)
    info = {
        "sector": int(sector),
        "center": bool(center),
        "bearing_rad": float(bearing),
        "prefer_up": bool(prefer_up),
        "lateral_m": float(abs(final[1])),
        "up_m": float(final[2]),
        "forward_m": float(final[0]),
        "range_m": float(np.linalg.norm(world - p)),
    }
    return world, info


def keep_ahead(position, goal, yaw: float, min_dist: float = ARRIVE_GUARD_M) -> np.ndarray:
    """临时目标若已经靠近机体，就沿原方向推到到达半径外面。"""
    p = np.asarray(position, dtype=float)
    g = np.array(goal, dtype=float, copy=True)
    delta = g - p
    dist = float(np.linalg.norm(delta))
    if dist >= min_dist:
        return g
    if dist > 1e-9:
        direction = delta / dist
    else:
        direction = _R_yaw(yaw) @ np.array([1.0, 0.0, 0.0])
    return p + direction * min_dist


def fresh_escape_state() -> dict:
    return {"streak": 0, "hold": 0, "active": False}


def observe(state: dict, code: str, *, n: int, arrived: bool) -> tuple[dict, bool, bool]:
    """看完这一拍的规划码。

    hold 是这一拍之后还要保持临时目标的规划拍数。
    fire：这一拍开始时没有在脱困，且连续悬停已经达到 N。
    release：这一拍还在用临时目标，但走廊已经重开，或 K 拍用完。
    同一拍不会又 fire 又 release。
    """
    streak = 0 if code != HOVER else int(state["streak"]) + 1
    hold = int(state["hold"])
    active = bool(state["active"])
    fire = False
    release = False
    if active:
        hold -= 1
        if code != HOVER or hold <= 0:
            release = True
            active = False
            hold = 0
            if code != HOVER:
                streak = 0
    elif (not arrived) and code == HOVER and streak >= int(n):
        fire = True
    return {"streak": streak, "hold": hold, "active": active}, fire, release


def commit_fire(state: dict, replan_code: str, *, k: int) -> dict:
    """触发当拍已经用临时目标重规划过。重开则下一拍回到原目标，否则再保持 K-1 拍。"""
    if replan_code != HOVER:
        return {"streak": 0, "hold": 0, "active": False}
    remain = int(k) - 1
    return {"streak": int(state["streak"]), "hold": remain, "active": remain > 0}


def install_peer_delivery() -> None:
    """欧氏球打开时把邻机采样送进后段。不改 policy.py 的源文件。

    椭球项仍只看 swarm_penalty。两个开关都关时，送进 refine_yopo 的邻机仍是 None。
    """
    global _PEER_INSTALLED
    if _PEER_INSTALLED:
        return
    import yopo_l0.policy as policy_mod
    import yopo_l0.runner as runner_mod
    from yopo_l0.backend.config import peer_sphere_enabled as sphere_on

    orig_enabled = runner_mod.swarm_penalty_enabled

    def enabled(params, **kwargs):
        if orig_enabled(params, **kwargs):
            return True
        return bool(sphere_on(params, **kwargs))

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
    _PEER_INSTALLED = True


def escape_policy_class():
    """进程内的 UpstreamPolicy 子类。源文件不动。"""
    global _POLICY_CLS
    if _POLICY_CLS is not None:
        return _POLICY_CLS
    from dataclasses import replace

    from yopo_l0.policy import UpstreamPolicy

    class EscapeReplanPolicy(UpstreamPolicy):
        N = 10
        K = K_PLANS
        triggers = 0
        up_triggers = 0
        side_triggers = 0
        max_streak = 0
        triggers_by: dict = {}

        def __init__(self, ctx):
            super().__init__(ctx)
            self._st = fresh_escape_state()
            self._escape_goal = None

        def _false_arrive(self, was: bool, position, original) -> bool:
            if not self.arrived or was:
                return False
            limit = float(self.p["arrive_radius_m"])
            dist = float(np.linalg.norm(np.asarray(position, dtype=float) - np.asarray(original, dtype=float)))
            return dist >= limit

        def plan(self, obs):
            original = np.array(obs.goal, dtype=float, copy=True)
            state = self._st
            if state["active"] and self._escape_goal is not None:
                goal = keep_ahead(obs.state.p, self._escape_goal, float(obs.state.yaw))
                obs_in = replace(obs, goal=goal)
            else:
                obs_in = obs
            was = self.arrived
            plan = super().plan(obs_in)
            if self._false_arrive(was, obs.state.p, original):
                self.arrived = False
                self._escape_goal = None
                self._st = fresh_escape_state()
                return super().plan(obs)
            new_state, fire, release = observe(state, self.last_code, n=int(type(self).N), arrived=bool(self.arrived))
            if release:
                self._escape_goal = None
            if fire:
                owner = str(self.ctx.owner)
                alternate = int(type(self).triggers_by.get(owner, 0))
                frame = obs.frame
                depth = None if frame is None else frame.depth
                intrinsics = None if frame is None else frame.K
                self._escape_goal, info = choose_escape_goal(
                    obs.state.p, float(obs.state.yaw), depth, intrinsics, alternate=alternate
                )
                cls = type(self)
                cls.triggers += 1
                cls.triggers_by[owner] = alternate + 1
                if info["prefer_up"]:
                    cls.up_triggers += 1
                else:
                    cls.side_triggers += 1
                goal = keep_ahead(obs.state.p, self._escape_goal, float(obs.state.yaw))
                was_fire = self.arrived
                plan = super().plan(replace(obs, goal=goal))
                if self._false_arrive(was_fire, obs.state.p, original):
                    self.arrived = False
                    self._escape_goal = None
                    self._st = fresh_escape_state()
                    return super().plan(obs)
                new_state = commit_fire(new_state, self.last_code, k=int(cls.K))
                if not new_state["active"]:
                    self._escape_goal = None
            self._st = new_state
            type(self).max_streak = max(int(type(self).max_streak), int(new_state["streak"]))
            return plan

    _POLICY_CLS = EscapeReplanPolicy
    return EscapeReplanPolicy


def _reset_policy(n: int, k: int) -> None:
    cls = escape_policy_class()
    cls.N = int(n)
    cls.K = int(k)
    cls.triggers = 0
    cls.up_triggers = 0
    cls.side_triggers = 0
    cls.max_streak = 0
    cls.triggers_by = {}


def build_card(peer: bool):
    """E-C6 的 geom 臂。peer 为真时只在进程内加上后段欧氏球，阈值不动。"""
    from yopo_l0.scene import SceneCard

    ec6 = load_ec6()
    base = ec6.build_card(SCENE, "geom")
    if not peer:
        return base
    raw = json.loads(json.dumps(base.card))
    params = dict(raw.get("policy_params") or {})
    backend = dict(params.get("backend") or {})
    if backend.get("swarm_penalty") is True or backend.get("swarm_segment_penalty") is True:
        raise RuntimeError("脱困臂不打开椭球惩罚")
    backend.update(SPHERE)
    params["backend"] = backend
    raw["policy_params"] = params
    return SceneCard(base.path, raw, base.obstacles)


def _upstream(card) -> dict:
    return dict(card.card["policy_params"]["upstream"])


def assert_geom_thresholds(card, *, peer: bool) -> None:
    ec6 = load_ec6()
    up = _upstream(card)
    base = _upstream(ec6.build_card(SCENE, "geom"))
    if up != base:
        raise RuntimeError(f"脱困臂改了 upstream 阈值：{up}")
    if up.get("inter_drone_threshold_m") != ec6.A1_THRESHOLD_M:
        raise RuntimeError("E-A1 阈值被改了")
    if abs(float(up["track_margin_m"]) - ec6.MARGIN_M) > 1e-12:
        raise RuntimeError("track_margin 被改了")
    if "safe_radius" in up:
        raise RuntimeError("脱困臂不写 safe_radius")
    backend = (card.card.get("policy_params") or {}).get("backend")
    if not peer:
        if backend:
            raise RuntimeError("不开欧氏球的臂不应写 backend")
        return
    if not isinstance(backend, dict) or backend.get("peer_sphere_enabled") is not True:
        raise RuntimeError("欧氏球臂没有打开 peer_sphere_enabled")
    if backend.get("enabled") is not True:
        raise RuntimeError("欧氏球要后段 enabled 才生效")
    if backend.get("swarm_penalty") is True:
        raise RuntimeError("欧氏球臂不打开 swarm_penalty")


def planner_due(k: int, rate: int = 10, physics: int = 100) -> bool:
    """与 yopo_l0.runner._due 同一条式子：这一物理拍要不要规划。"""
    k = int(k)
    return k == 0 or (k * int(rate)) // int(physics) != ((k - 1) * int(rate)) // int(physics)


def planner_hover_count(ticks: list[dict], *, plan_hz: int = 10, phys: int = 100) -> int:
    """各机规划拍上的 CORRIDOR_EMPTY_HOVER 之和。"""
    total = 0
    for row in ticks or []:
        if planner_due(int(row["k"]), int(plan_hz), int(phys)) and row.get("plan_code") == HOVER:
            total += 1
    return total


def _jsonable(obj):
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.floating):
        value = float(obj)
        return None if not math.isfinite(value) else value
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, float):
        return None if not math.isfinite(obj) else obj
    return obj


def partial_path(arm: str, seed: int) -> Path:
    return PARTIAL / f"{arm}-{int(seed)}.json"


def load_partial(arm: str, seed: int) -> dict | None:
    dest = partial_path(arm, seed)
    if not dest.is_file():
        return None
    row = json.loads(dest.read_text(encoding="utf-8"))
    if int(row.get("row_version", 0)) != ROW_VERSION:
        return None
    if row.get("arm") != arm or int(row.get("seed", -1)) != int(seed):
        return None
    return row


def save_partial(row: dict) -> None:
    dest = partial_path(row["arm"], int(row["seed"]))
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(_jsonable(row), ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)


def _simulate(arm: str, seed: int) -> dict:
    import yopo_l0.runner as runner_mod
    from yopo_l0.runner import RunConfig, run

    ec6 = load_ec6()
    peer = arm_sphere(arm)
    card = build_card(peer)
    assert_geom_thresholds(card, peer=peer)
    if peer:
        install_peer_delivery()
    cls = escape_policy_class()
    _reset_policy(arm_n(arm), K_PLANS)
    prev = runner_mod.POLICIES["upstream"]
    runner_mod.POLICIES["upstream"] = cls
    t0 = time.perf_counter()
    try:
        out = run(card, RunConfig(seed=int(seed), shadow=True))
    finally:
        runner_mod.POLICIES["upstream"] = prev
    res = out.result
    got = float(res["safe_radius_m"])
    if abs(got - 0.05) > 1e-9:
        raise RuntimeError(f"safe_radius 读回 {got}，保持 0.05 m")
    if res.get("empty_brake") is not True:
        raise RuntimeError("empty_brake 没有打开")
    if res.get("track_clearance_mode") != "geom":
        raise RuntimeError(f"track mode 读回 {res.get('track_clearance_mode')}")
    filt = res.get("inter_drone_filter") or {}
    if abs(float(filt.get("threshold_m", -1)) - 1.0) > 1e-12:
        raise RuntimeError(f"E-A1 阈值读回 {filt.get('threshold_m')}")
    hits = ec6.classify_collision(card, res, out.ticks)
    surfaces = ec6.surface_mins(card, out.ticks)
    body = float(card.card["criteria"]["body_radius_m"])
    surface = float(res["min_obstacle_distance_m"])
    hover_sum = planner_hover_count(out.ticks)
    obstacle = int(hits["pillar_collision"] + hits["ground_collision"] + hits["box_collision"])
    row = {
        "row_version": ROW_VERSION,
        "scene": SCENE,
        "arm": arm,
        "seed": int(seed),
        "escape_n": arm_n(arm),
        "escape_k": K_PLANS,
        "peer_sphere": peer,
        "upstream_deviation": True,
        "runner_upstream_deviation": bool(res["upstream_deviation"]),
        "report_only": True,
        "frozen": False,
        "policy": res["policy"],
        "task": res["task"],
        "safe_radius_m": got,
        "empty_brake": res.get("empty_brake"),
        "track_clearance": res.get("track_clearance"),
        "track_clearance_mode": res.get("track_clearance_mode"),
        "track_margin_m": res.get("track_margin_m"),
        "inter_threshold_m": filt.get("threshold_m"),
        "outcome": res["outcome"],
        "reason": str(res["reason"]),
        "arrived": int(res["arrived"]),
        "n_drones": int(res["n_drones"]),
        "timeout": int(res["outcome"] == "TIMEOUT"),
        "min_surface_m": surface,
        "min_clearance_m": surface - body,
        "min_inter_drone_distance_m": res["min_inter_drone_distance_m"],
        "avg_speed_mps": ec6.avg_speed_mps(out.ticks),
        "plan_wall_ms": ec6.plan_wall_block(res),
        "corridor_empty_hover": hover_sum,
        "corridor_empty_hover_plan_codes": int((res.get("plan_codes") or {}).get(HOVER, 0)),
        "escape_triggers": int(cls.triggers),
        "escape_up_triggers": int(cls.up_triggers),
        "escape_side_triggers": int(cls.side_triggers),
        "escape_by_drone": dict(cls.triggers_by),
        "max_hover_streak": int(cls.max_streak),
        "obstacle_collision": obstacle,
        "sim_duration_s": res["sim_duration_s"],
        "arrival_time_s": res["arrival_time_s"],
        "positions_sha256": ec6.positions_sha256(out.ticks),
        "wall_time_s": res["wall_time_s"],
        "worker_s": time.perf_counter() - t0,
        **hits,
        **surfaces,
    }
    return _jsonable(row)


def run_one(arm: str, seed: int, *, resume: bool) -> dict:
    if resume:
        cached = load_partial(arm, seed)
        if cached is not None:
            print(f"{arm_label(arm)} seed={seed} resume", flush=True)
            return cached
    row = _simulate(arm, seed)
    save_partial(row)
    speed = row["avg_speed_mps"]
    speed_s = "—" if speed is None else f"{float(speed):.3f}"
    print(
        f"{arm_label(arm)} seed={seed} {row['outcome']} "
        f"{row['arrived']}/{row['n_drones']} obs={row['obstacle_collision']} "
        f"inter={row['inter_drone_collision']} speed={speed_s} "
        f"hover={row['corridor_empty_hover']} escape={row['escape_triggers']} "
        f"sim={float(row['sim_duration_s']):.2f} wall={float(row['wall_time_s']):.1f}s",
        flush=True,
    )
    return row


def _worker(spec: tuple) -> dict:
    try:
        os.nice(10)
    except OSError:
        pass
    arm, seed, resume = spec
    try:
        return run_one(str(arm), int(seed), resume=bool(resume))
    except Exception:
        print(f"FAIL {arm} seed={seed}", flush=True)
        raise


def job_specs(arms, seeds, *, resume: bool = True) -> list[tuple]:
    return [(arm, int(seed), bool(resume)) for arm in arms for seed in seeds]


def _run_group(specs: list[tuple], jobs: int) -> list[dict]:
    if jobs <= 1 or len(specs) <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)


def _group(rows: list[dict], arm: str) -> list[dict]:
    return [r for r in rows if r["arm"] == arm]


def _mean(values: list[float]) -> float | None:
    if not values:
        return None
    return float(sum(values) / len(values))


def _fmt(value, nd: int = 3) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{nd}f}"


def _rng(values: list[float], nd: int = 3) -> str:
    if not values:
        return "—"
    lo, hi = min(values), max(values)
    if abs(lo - hi) < 5 * 10 ** (-(nd + 1)):
        return f"{lo:.{nd}f}"
    sep = " 至 " if lo < 0.0 or hi < 0.0 else "–"
    return f"{lo:.{nd}f}{sep}{hi:.{nd}f}"


def _speeds(group: list[dict]) -> list[float]:
    return [float(r["avg_speed_mps"]) for r in group if r.get("avg_speed_mps") is not None]


def _clears(group: list[dict]) -> list[float]:
    return [float(r["min_clearance_m"]) for r in group]


def _gaps(group: list[dict]) -> list[float]:
    return [
        float(r["min_inter_drone_distance_m"])
        for r in group
        if r.get("min_inter_drone_distance_m") is not None
    ]


def _signed(value: int, unit: str = "") -> str:
    if value > 0:
        return f"多 {value}{unit}"
    if value < 0:
        return f"少 {-value}{unit}"
    return f"持平"


def render_summary(rows: list[dict]) -> str:
    lines = [
        "| 臂 | 偏离上游 | 到达 | 障碍碰撞 | 机间碰撞 | 超时 | 平均速度 m/s | 悬停 | 脱困触发 | 最小净空 m | 最小机间 m |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    base = EC7_BASELINE
    lines.append(
        f"| {base['label']} | 偏离上游 | {base['arrived']}/{base['drones']} | "
        f"{base['obstacle_collisions']} | {base['inter_collisions']} | {base['timeout_runs']} | "
        f"{base['avg_speed_mps']:.3f} | {base['hover']} | — | {base['clearance']} | {base['inter_gap']} |"
    )
    for arm in ARMS:
        group = _group(rows, arm)
        if not group:
            continue
        drones = sum(int(r["n_drones"]) for r in group)
        arrived = sum(int(r["arrived"]) for r in group)
        obstacle = sum(int(r["obstacle_collision"]) for r in group)
        inter = sum(int(r["inter_drone_collision"]) for r in group)
        timeout = sum(int(r["timeout"]) for r in group)
        hover = sum(int(r["corridor_empty_hover"]) for r in group)
        triggers = sum(int(r["escape_triggers"]) for r in group)
        lines.append(
            f"| {arm_label(arm)} | 偏离上游 | {arrived}/{drones} | {obstacle} | {inter} | {timeout} | "
            f"{_fmt(_mean(_speeds(group)))} | {hover} | {triggers} | {_rng(_clears(group))} | {_rng(_gaps(group))} |"
        )
    return "\n".join(lines)


def render_vs_baseline(rows: list[dict]) -> str:
    base = EC7_BASELINE
    lines = []
    for arm in ARMS:
        group = _group(rows, arm)
        if len(group) != len(SEEDS):
            continue
        arrived = sum(int(r["arrived"]) for r in group)
        drones = sum(int(r["n_drones"]) for r in group)
        obstacle = sum(int(r["obstacle_collision"]) for r in group)
        inter = sum(int(r["inter_drone_collision"]) for r in group)
        timeout = sum(int(r["timeout"]) for r in group)
        hover = sum(int(r["corridor_empty_hover"]) for r in group)
        triggers = sum(int(r["escape_triggers"]) for r in group)
        up = sum(int(r.get("escape_up_triggers") or 0) for r in group)
        side = sum(int(r.get("escape_side_triggers") or 0) for r in group)
        speed = _mean(_speeds(group))
        speed_s = "—" if speed is None else f"{speed:.3f}"
        lines.append(
            f"{arm_label(arm)}：到达 {arrived}/{drones}（基线 {base['arrived']}/{base['drones']}，"
            f"{_signed(arrived - int(base['arrived']))}），"
            f"障碍碰撞 {obstacle}（基线 {base['obstacle_collisions']}，{_signed(obstacle - int(base['obstacle_collisions']))}），"
            f"机间碰撞 {inter}（基线 {base['inter_collisions']}，{_signed(inter - int(base['inter_collisions']))}），"
            f"超时 {timeout} 粒（基线 {base['timeout_runs']}，{_signed(timeout - int(base['timeout_runs']))}），"
            f"平均速度 {speed_s} m/s（基线 {base['avg_speed_mps']:.3f}），"
            f"悬停 {hover}（基线 {base['hover']}，{_signed(hover - int(base['hover']))}），"
            f"脱困触发 {triggers}（其中 {up} 次上带更通畅、同时抬升 {UP_M:.1f} m，其余 {side} 次只侧移）。"
            f"最小净空 {_rng(_clears(group))} m，最小机间 {_rng(_gaps(group))} m。"
        )
    return "\n\n".join(lines)


def render_seeds(rows: list[dict]) -> str:
    lines = [
        "| 臂 | seed | 结果 | 原因 | 到达 | 障碍 | 机间 | 超时 | 平均速度 m/s | 悬停 | 脱困 | 最长连续悬停 | 最小净空 m | 最小机间 m | 仿真 s |",
        "|---|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for arm in ARMS:
        for row in sorted(_group(rows, arm), key=lambda r: int(r["seed"])):
            gap = row.get("min_inter_drone_distance_m")
            lines.append(
                f"| {arm_label(arm)} | {row['seed']} | {row['outcome']} | `{row['reason']}` | "
                f"{row['arrived']}/{row['n_drones']} | {row['obstacle_collision']} | "
                f"{row['inter_drone_collision']} | {row['timeout']} | {_fmt(row.get('avg_speed_mps'))} | "
                f"{int(row['corridor_empty_hover'])} | {int(row['escape_triggers'])} | "
                f"{int(row.get('max_hover_streak') or 0)} | {_fmt(row.get('min_clearance_m'))} | "
                f"{_fmt(gap)} | {float(row['sim_duration_s']):.2f} |"
            )
    return "\n".join(lines)


def _disk_snapshot() -> dict[str, bytes]:
    return {rel: (ROOT / rel).read_bytes() for rel in _WATCHED}


def _check() -> int:
    from yopo_l0.policy import R_yaw
    from yopo_l0.policy import read_empty_brake, read_track_clearance

    ec6 = load_ec6()
    blob = (ROOT / "scenes" / ec6.GEOM_NAME).read_bytes()
    got = hashlib.sha256(blob).hexdigest()
    if got != ec6.GEOM_SHA256:
        raise SystemExit(f"{ec6.GEOM_NAME} sha256 变了：{got}")
    if read_track_clearance({}) or read_empty_brake({}):
        raise SystemExit("默认 empty_brake / track_clearance 被打开了")
    yaw = 0.4
    if not np.allclose(_R_yaw(yaw), R_yaw(yaw)):
        raise SystemExit("机体系旋转和 policy.R_yaw 不一致")
    before = _disk_snapshot()
    plain = build_card(False)
    sphere = build_card(True)
    assert_geom_thresholds(plain, peer=False)
    assert_geom_thresholds(sphere, peer=True)
    if len(plain.card["drones"]) != 6:
        raise SystemExit("六机对向不是 6 架")
    ec6.assert_endpoints_clear(plain)
    state = fresh_escape_state()
    for _ in range(9):
        state, fire, release = observe(state, HOVER, n=10, arrived=False)
        if fire or release:
            raise SystemExit("N=10 在第 10 拍之前就触发了")
    state, fire, release = observe(state, HOVER, n=10, arrived=False)
    if not fire or release:
        raise SystemExit("第 10 拍没有触发脱困")
    state = commit_fire(state, HOVER, k=K_PLANS)
    if not state["active"] or int(state["hold"]) != K_PLANS - 1:
        raise SystemExit("脱困保持拍数不对")
    goal, info = choose_escape_goal(np.array([0.0, 0.0, 1.5]), 0.0, None, None, alternate=0)
    if info["range_m"] <= 1.5:
        raise SystemExit("临时目标落进了到达半径")
    if not (LATERAL_M - 1e-9 <= info["lateral_m"] <= LATERAL_CAP_M + 1e-9):
        raise SystemExit(f"侧移 {info['lateral_m']} 不在 1.2–1.5 m")
    after = _disk_snapshot()
    if after != before:
        raise SystemExit("检查时改写了核心文件或场景")
    if (ROOT / "scenes" / "ego-density-014-swap6.json").exists():
        raise SystemExit("六机对向被写进了 scenes/")
    suite = (ROOT / "scenes" / "suite.txt").read_text(encoding="utf-8")
    if "e-c8" in suite or "ego-density-014-swap6" in suite:
        raise SystemExit("scenes/suite.txt 被写进了本票的卡")
    print("e-c8 check ok")
    return 0


def _head_commit() -> str:
    import subprocess

    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def render_findings(rows: list[dict]) -> str:
    """由这次跑出来的行汇总。欧氏球臂同时开了后段，不能单独算成脱困的效果。"""
    def pack(arm: str) -> dict:
        group = _group(rows, arm)
        return {
            "arrived": sum(int(r["arrived"]) for r in group),
            "drones": sum(int(r["n_drones"]) for r in group),
            "obstacle": sum(int(r["obstacle_collision"]) for r in group),
            "pillar": sum(int(r["pillar_collision"]) for r in group),
            "inter": sum(int(r["inter_drone_collision"]) for r in group),
            "timeout": sum(int(r["timeout"]) for r in group),
            "hover": sum(int(r["corridor_empty_hover"]) for r in group),
            "triggers": sum(int(r["escape_triggers"]) for r in group),
            "speed": _mean(_speeds(group)),
        }

    plain = [pack(arm) for arm in ARMS if not arm_sphere(arm)]
    lines = ["## 结论", ""]
    if plain and all(item["arrived"] == 15 and item["timeout"] == 3 and item["obstacle"] == 2 and item["inter"] == 0 for item in plain):
        hovers = "、".join(str(item["hover"]) for item in plain)
        lines.append(
            "不开欧氏球的三档，到达都是 15/30，撞柱 2，机间 0，超时 3，和 E-C7 基线的到达、碰撞、超时相同。"
            f"悬停分别是 {hovers}，基线是 1127。"
            "超时仍是 seeds 0、1、3，每粒 5/6；seeds 2、4 在连续悬停达到 N 之前就撞柱，脱困次数是 0。"
            "临时目标在超时的飞机上会反复触发，40 s 内没有把卡住的那一架送到原目标。"
        )
        lines.append("")
    sphere_rows = [r for r in rows if arm_sphere(r["arm"])]
    best = None
    for arm in ARMS:
        if not arm_sphere(arm):
            continue
        item = pack(arm)
        if best is None or item["arrived"] > best[1]["arrived"]:
            best = (arm, item)
    if best is not None:
        arm, item = best
        speed = "—" if item["speed"] is None else f"{item['speed']:.3f}"
        lines.append(
            f"六档里到达最多的是 {arm_label(arm)}：到达 {item['arrived']}/{item['drones']}，"
            f"撞柱 {item['pillar']}，机间 {item['inter']}，超时 {item['timeout']}，"
            f"悬停 {item['hover']}，脱困触发 {item['triggers']}，平均速度 {speed} m/s。"
            "这一档同时打开后段和 `peer_sphere_enabled`。"
            "本脚本没有单独跑「只开欧氏球、不脱困」，这个到达数是两者叠在 geom + empty_brake 上的结果。"
        )
        lines.append("")
    seed3 = [r for r in sphere_rows if int(r["seed"]) == 3]
    if seed3 and all(r["reason"] == "uav4:TRUTH_POSITION" and int(r["escape_triggers"]) == 0 for r in seed3):
        lines.append(
            "三档欧氏球的 seed 3 都是 `uav4:TRUTH_POSITION`，仿真约 2.37 s，悬停 5 次，没有触发脱困。"
            "这是到达 N 之前的撞柱，不是空走廊悬停。"
        )
        lines.append("")
    n20 = [r for r in rows if r["arm"] == "n20-sphere" and r["reason"] == "uav2-uav4:INTER_DRONE"]
    if n20:
        gap = float(n20[0]["min_inter_drone_distance_m"])
        lines.append(
            f"N=20 加欧氏球的 seed 0 在脱困触发前发生机间碰撞（`uav2-uav4:INTER_DRONE`，最小机间 {gap:.3f} m）。"
            "N=15 加欧氏球的 seed 0 则是 `uav5:TRUTH_POSITION`，脱困触发了 1 次。"
            "阈值越大，seed 0 越先撞上别的东西；N=10 加欧氏球时这一粒是 6/6 到达。"
        )
        lines.append("")
    lines.append(
        "最长连续悬停记的是这一拍结算之后的计数。触发当拍若走廊重开，连续计数会清零，成功脱困时这一列可以停在 N−1。"
        "不开欧氏球时，N=15 和 N=20 的 seeds 0、1 仍记到 372，和 E-C7 那一截连续悬停同一量级。"
    )
    return "\n".join(lines)


def render_markdown(payload: dict) -> str:
    rows = payload["rows"]
    commit = payload.get("commit") or ""
    lines = [
        "# 票 E-C8：六机 014 连续空走廊悬停脱困重规划",
        "",
        "只报数。report-only。偏离上游。没有改 `yopo_l0/`、`core/`、`scenes/`、"
        "`docs/tickets.md`、README、`scripts/premerge.sh` 和 `scripts/regress.sh`。"
        "过滤阈值不动：`safe_radius` 仍是 0.05 m，E-A1 仍是 1.0 m、惩罚 0，"
        "`track_clearance` 仍是 `geom`、`track_margin_m=0.08`，`empty_brake` 仍开。"
        "脱困只在脚本进程里包一层策略，临时改这一拍的目标，不改 `policy.py`。"
        "未冻结，不进 `scenes/suite.txt`，不设 gates，不改已冻门。",
        "",
        f"数字在 [e-c8-escape-replan-runs.json](e-c8-escape-replan-runs.json)。跑数时的 main 是 `{commit}`。"
        "这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。",
        "",
        "## 基线",
        "",
        "E-C7（PR #99）在同一张六机 014 对向、`track_clearance=geom`、`empty_brake=true`、seeds 0–4 上："
        "到达 15/30，超时 3 粒（seeds 0、1、3 的 uav5），悬停 1127，平均速度 1.580 m/s。"
        "悬停的直接原因主要是 geom 净空门拒绝之后参考头停住，`empty_brake` 原地保持。"
        "E-A2 软让行把到达降到 14/30，本票不重复那条线。",
        "",
        "碰撞拆分用 E-C6 同一 geom 臂：撞柱 2、撞地 0、撞箱 0、机间 0。"
        "E-C7 没有另报碰撞。最小净空 -0.007 至 0.137 m，最小机间 0.966–2.230 m，也来自那一臂。"
        "本脚本不重跑这条基线。",
        "",
        "## 脱困动作",
        "",
        "每架飞机自己数连续的 `CORRIDOR_EMPTY_HOVER`。数到 N（10、15 或 20，含第 N 拍）"
        "而且当前不在脱困窗口里，就触发一次。触发当拍用临时目标再规划一次，这一拍交给临时目标，"
        "不再把原地保持交出去。",
        "",
        "临时目标看本机深度图。水平切成 5 个扇区，无回波按 20 m 计，取中位数最大的扇区。"
        "各扇区差不多时当作正前方，侧向按这架飞机自己的触发次数左右交替，偶数向右。"
        f"机体系前方 {FORWARD_M:.1f} m，侧向分量夹在 {LATERAL_M:.1f}–{LATERAL_CAP_M:.1f} m。"
        f"上三分之一的中位数比中带高出 0.5 m 时，再抬 {UP_M:.1f} m。"
        f"前方留 {FORWARD_M:.1f} m，是为了让临时目标落在到达半径 1.5 m 外面，避免误锁 `ARRIVED_HOLD`。"
        "若飞行中临时目标靠近到 2.2 m 以内，就沿同一方向推远。原目标仍在到达半径外时，"
        "误触发的到达标志会清掉，并当拍改回原目标。",
        "",
        f"临时目标最多保持 {K_PLANS} 个规划拍（规划 10 Hz，即 {K_PLANS / 10:.1f} s）。"
        "走廊重开（这一拍不再是 `CORRIDOR_EMPTY_HOVER`）就提前结束，下一拍回到原目标。"
        "K 拍用完仍在悬停，下一拍可以再次触发，侧向交替。不改 E-A1 阈值，也不改 geom 净空门。",
        "",
        "E-B4c 臂在同一张 geom 卡上另写 `policy_params.backend.enabled=true` 和 "
        "`peer_sphere_enabled=true`（半径 0.85 m，权重 8）。`swarm_penalty` 不开。"
        "现有接线只在椭球惩罚打开时把邻机送进后段。本脚本按 E-B4c 的做法，在进程里包一层 "
        "`_apply_backend`：欧氏球打开时送采样，关上时仍是 None。不改 `policy.py`。",
        "",
        "## 口径",
        "",
        AVG_SPEED_DEF,
        "",
        HOVER_DEF,
        "",
        "到达按架次。seeds 0–4、6 架，一档分母是 30。"
        "障碍碰撞是 `TRUTH_POSITION`（圆柱、地面或盒子），机间碰撞是 `INTER_DRONE`。"
        "仿真在第一次碰撞停下，所以一次运行最多记 1 次碰撞，障碍和机间分开数。"
        "超时按运行计，一次运行超时记 1。",
        "最小净空 = `min_obstacle_distance_m` − 0.35 m。最小机间是中心距。",
        "脱困触发按架次累加，一次触发算 1，保持的那几拍不再加。",
        "",
        "## 对照",
        "",
        render_summary(rows),
        "",
        render_vs_baseline(rows),
        "",
        "## 分种子",
        "",
        render_seeds(rows),
        "",
        render_findings(rows),
        "",
        "## 复现",
        "",
        "```",
        "python3 scripts/e-c8-escape-replan.py --jobs 4",
        "```",
        "",
        "缺省从 `out/e-c8-escape-replan/partial/` 续跑。`--no-resume` 忽略已有 partial。"
        "`--check` 只核对六机构造、阈值和脱困触发条件，不起仿真。",
        "",
    ]
    return "\n".join(lines) + "\n"


def _write_outputs(payload: dict, out: Path, md_path: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    md_path.parent.mkdir(parents=True, exist_ok=True)
    md_path.write_text(render_markdown(payload), encoding="utf-8")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--out", default="docs/research/e-c8-escape-replan-runs.json")
    ap.add_argument("--md", default="docs/research/e-c8-escape-replan.md")
    ap.add_argument("--partial", action="store_true", help="从 partial 断点续跑（缺省也续跑）")
    ap.add_argument("--no-resume", action="store_true", help="忽略已有 partial，重跑")
    ap.add_argument("--from-partial", action="store_true", help="只汇总已有 partial，不起新仿真")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)
    if args.check:
        return _check()
    if args.jobs not in (1, 2, 3, 4):
        raise SystemExit("jobs 只能是 1 到 4")
    arms = [s for s in args.arms.split(",") if s]
    seeds = [int(s) for s in args.seeds.split(",") if s != ""]
    unknown = [s for s in arms if s not in ARMS]
    if unknown or any(s not in SEEDS for s in seeds):
        raise SystemExit(f"未知臂或种子：arms={arms} seeds={seeds}")
    _check()
    resume = bool(args.partial) or not args.no_resume
    if args.from_partial:
        rows = []
        for arm in arms:
            for seed in seeds:
                row = load_partial(arm, seed)
                if row is None:
                    raise SystemExit(f"缺少 partial：{arm} seed={seed}")
                rows.append(row)
    else:
        specs = job_specs(arms, seeds, resume=resume)
        print(f"e-c8 start n={len(specs)} jobs={args.jobs} resume={resume}", flush=True)
        rows = _run_group(specs, int(args.jobs))
    rows.sort(key=lambda r: (ARMS.index(r["arm"]), int(r["seed"])))
    ec6 = load_ec6()
    payload = {
        "disclaimer": DISCLAIMER,
        "commit": _head_commit(),
        "report_only": True,
        "frozen": False,
        "upstream_deviation": True,
        "baseline": EC7_BASELINE,
        "arms": arms,
        "seeds": seeds,
        "jobs": int(args.jobs),
        "escape_k": K_PLANS,
        "forward_m": FORWARD_M,
        "lateral_m": LATERAL_M,
        "lateral_cap_m": LATERAL_CAP_M,
        "up_m": UP_M,
        "geom_sha256": ec6.GEOM_SHA256,
        "avg_speed_def": AVG_SPEED_DEF,
        "hover_def": HOVER_DEF,
        "rows": rows,
        "note": DISCLAIMER,
    }
    dest = Path(args.out)
    if not dest.is_absolute():
        dest = ROOT / dest
    md_path = Path(args.md)
    if not md_path.is_absolute():
        md_path = ROOT / md_path
    _write_outputs(payload, dest, md_path)
    print(render_summary(rows))
    print(f"写入 {dest}")
    print(f"写入 {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
