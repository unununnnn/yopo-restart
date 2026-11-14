#!/usr/bin/env python3
"""票 E-C8b：六机 014 欧氏球 × 几何门 × 脱困重规划消融。只报数。

场景只活在本进程的内存里：障碍布局取 ``scenes/ego-density-014-swap3.json``
解析后的几何（即 ``scenes/ego-density-014.json`` 的 56 根圆柱），六机对向
起终点与 ``scripts/e-c6-six-density.py`` 的 swap6 相同。不写 ``scenes/``。

六臂，seeds 0–4。每一臂都偏离上游，report-only：

- A：``track_clearance=geom`` + ``empty_brake``。无 E-B4c，无脱困。
- B：A 再加 E-B4c（半径 0.85 m，权重 8.0）。无脱困。这是 E-C8 缺的孤立对照。
- C：只开 E-B4c（半径 0.85 m，权重 8.0）。无 geom，无 empty_brake，无脱困。
- D：A 加 E-C8（连续 10 拍空走廊才脱困）加 E-B4c（0.85 m / 8.0）。
- E：与 D 相同，球半径改成 E-B5 的 1.00 m，权重仍是 8.0。
- F：与 D 相同，半径仍是 0.85 m，权重改成 E-B5 的 16.0。

E-A1（``inter_drone_filter``）保持关。``safe_radius`` 不写，运行器仍是 0.05 m。
``track_margin_m`` 写成 0.08，这是几何门读者的缺省，也是 E-C5b 用来盖
0.058–0.073 m 跟踪差的假设。

欧氏球的邻机采样沿 ``scripts/e-b4c-sphere-avoid.py`` 的进程内包装送进
``refine_yopo``：不改 ``policy.py``。运行器仍是本 tick 全部 ``plan()`` 之后
才 ``bus.poll``，后段用到的是上一规划拍已经提交的采样。

脱困也只包在进程里。单机连续 ``CORRIDOR_EMPTY_HOVER`` 满 10 个规划拍后，
按当时深度图最通畅扇区给一个有界临时目标：机体前 4.0 m，侧向 1.2–1.5 m。
这个点最多用 10 个规划拍。其间规划码不再是空走廊，就立刻回到原目标。
任务到达仍只认场景卡上的原目标。

不改 ``yopo_l0/``、``core/``、``scenes/``、``docs/tickets.md``、README、
``scripts/premerge.sh``、``scripts/regress.sh``，也不改任何默认值。
未冻结，不进 ``scenes/suite.txt``，不设 gates。这不是论文复现。
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

import numpy as np

def _limit_blas_threads() -> None:
    """只在真正开跑时限制 BLAS 线程。被 pytest import 时不改进程环境。"""
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")


def _clear_sphere_env() -> None:
    """半径和权重以本票的卡为准，不吃进程里残留的 YOPO_PEER_SPHERE*。"""
    for key in ("YOPO_PEER_SPHERE", "YOPO_PEER_SPHERE_RADIUS_M", "YOPO_PEER_SPHERE_WEIGHT"):
        os.environ.pop(key, None)


ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-c8b-ablation" / "partial"
SWAP3_REL = "scenes/ego-density-014-swap3.json"
GEOM_NAME = "ego-density-014.json"
GEOM_SHA256 = "5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9"
SCENE = "d014-swap6"
SCENE_LABEL = "014 六机对向"
SWAP6_NAME = "ego-density-014-swap6"
ARMS = ("A", "B", "C", "D", "E", "F")
SEEDS = tuple(range(5))
ROW_VERSION = 1

BODY_RADIUS_M = 0.35
MARGIN_M = 0.08
SAFE_RADIUS_M = 0.05
OPEN_RANGE_M = 20.0
HFOV_DEG = 90.0
N_SECTORS = 9
ESCAPE_TRIGGER_TICKS = 10
ESCAPE_HOLD_TICKS = 10
ESCAPE_FORWARD_M = 4.0
ESCAPE_LATERAL_LO_M = 1.2
ESCAPE_LATERAL_HI_M = 1.5
ARRIVE_PUSH_SLACK_M = 0.05
ARRIVE_PUSH_AHEAD_M = 0.5
B4C_RADIUS_M = 0.85
B4C_WEIGHT = 8.0
B5_RADIUS_M = 1.00
B5_WEIGHT = 16.0
MAX_JOBS = 2

# 扇区从图像左到右。图像右是相机 +x，也是机体右（FLU 的 -y）。
_ESCAPE_INSTALLED = False
_PEER_INSTALLED = False
_ORIG_PLAN = None
_PEER_SAVED = None
_ESCAPE_STATS: dict[str, dict] = {}

DISCLAIMER = (
    "只报数。report-only。不改默认 safe_radius 0.05 m、empty_brake false、"
    "track_clearance false、peer_sphere_enabled false、inter_drone_filter 关。"
    "track_clearance=geom、empty_brake、peer_sphere_enabled、脱困重规划打开即偏离上游。"
    "track_margin 0.08 m 是几何门读者的缺省，也是假设，用来盖已观测的 0.058–0.073 m 跟踪差。"
    "六机 0.14 对向卡只存在于进程内存，障碍布局来自 scenes/ego-density-014-swap3.json，不写入 scenes/。"
    "E-A1 不打开。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。"
    "未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。"
)

AVG_SPEED_DEF = (
    "avg_speed_mps 是该次运行全部 tick、全部飞机的 ||v|| 里，"
    "有限且严格大于 0 的样本的算术平均。悬停和初值 0 不算有效飞行。"
    "没有有效样本时为 null。"
    "汇总表里的平均速度是该档各次 avg_speed_mps 的算术平均。"
)

HOVER_DEF = (
    "空走廊拍是全部飞机在规划拍上 plan_code 为 CORRIDOR_EMPTY_HOVER 的次数之和。"
    "规划拍与运行器 _due 相同：physics 100 Hz、planner 10 Hz 时即 k 能被 10 整除。"
    "empty_brake 打开时，这一拍是从参考头刹停，不是上游原地悬停。"
)

ESCAPE_DEF = (
    "脱困触发：该机连续空走廊拍达到 10 之后，下一批规划才改用临时目标。"
    "触发的那一拍本身仍飞原目标。临时目标在触发时刻的机体坐标里是前向 4.0 m、"
    "侧向 1.2–1.5 m（图像右侧为负），高度保持触发时刻的 z。"
    "侧向幅度 = 1.2 + 0.3 × min(|方位角| / 45°, 1)。正前方打平时取 1.2 m，"
    "偏向更通畅的一侧；左右一样通则向机体左侧。"
    "最通畅扇区是水平视场 9 等分后，地平线中间三分之一行的深度中位数最大的一档。"
    "无回波按相机最远 20 m 计。这个点最多保持 10 个规划拍；"
    "其间规划码不再是 CORRIDOR_EMPTY_HOVER，就清掉临时目标，下一拍回到原目标。"
    "10 拍用完仍是空走廊，也回到原目标，并重新累计连续空走廊。"
    "若机体已经进入临时目标的到达半径（缺省 1.5 m），沿触发时的瞄准方向把规划目标"
    "推到到达半径外 0.5 m，避免策略把脱困点锁成终点。任务到达只看原目标。"
)


def load_ec6():
    name = "e_c6_six_density_for_c8b"
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


def arm_spec(arm: str) -> dict:
    """返回这一臂相对上游多开的开关。sphere 是 (半径, 权重) 或 None。"""
    if arm not in ARMS:
        raise ValueError(arm)
    specs = {
        "A": {"geom": True, "brake": True, "sphere": None, "escape": False},
        "B": {"geom": True, "brake": True, "sphere": (B4C_RADIUS_M, B4C_WEIGHT), "escape": False},
        "C": {"geom": False, "brake": False, "sphere": (B4C_RADIUS_M, B4C_WEIGHT), "escape": False},
        "D": {"geom": True, "brake": True, "sphere": (B4C_RADIUS_M, B4C_WEIGHT), "escape": True},
        "E": {"geom": True, "brake": True, "sphere": (B5_RADIUS_M, B4C_WEIGHT), "escape": True},
        "F": {"geom": True, "brake": True, "sphere": (B4C_RADIUS_M, B5_WEIGHT), "escape": True},
    }
    return specs[arm]


def arm_deviates(arm: str) -> bool:
    """六臂都至少打开 geom、empty_brake、欧氏球、脱困其中之一。"""
    spec = arm_spec(arm)
    return bool(spec["geom"] or spec["brake"] or spec["sphere"] is not None or spec["escape"])


def arm_label(arm: str) -> str:
    labels = {
        "A": "A geom + empty_brake（无 E-B4c，无脱困；偏离上游 / report-only）",
        "B": "B geom + empty_brake + E-B4c r=0.85 w=8.0（无脱困；偏离上游 / report-only）",
        "C": "C E-B4c r=0.85 w=8.0 单独（偏离上游 / report-only）",
        "D": "D geom + empty_brake + E-C8 N=10 + E-B4c r=0.85 w=8.0（偏离上游 / report-only）",
        "E": "E geom + empty_brake + E-C8 N=10 + E-B5 r=1.00 w=8.0（偏离上游 / report-only）",
        "F": "F geom + empty_brake + E-C8 N=10 + E-B5 r=0.85 w=16.0（偏离上游 / report-only）",
    }
    if arm not in labels:
        raise ValueError(arm)
    return labels[arm]


def arm_config(arm: str) -> dict:
    spec = arm_spec(arm)
    sphere = spec["sphere"]
    return {
        "id": arm,
        "label": arm_label(arm),
        "track_clearance": bool(spec["geom"]),
        "track_clearance_mode": "geom" if spec["geom"] else None,
        "track_margin_m": MARGIN_M if spec["geom"] else None,
        "empty_brake": bool(spec["brake"]),
        "peer_sphere_enabled": sphere is not None,
        "peer_sphere_radius_m": None if sphere is None else float(sphere[0]),
        "peer_sphere_weight": None if sphere is None else float(sphere[1]),
        "escape_replan": bool(spec["escape"]),
        "escape_trigger_ticks": ESCAPE_TRIGGER_TICKS if spec["escape"] else None,
        "escape_hold_ticks": ESCAPE_HOLD_TICKS if spec["escape"] else None,
        "escape_forward_m": ESCAPE_FORWARD_M if spec["escape"] else None,
        "escape_lateral_lo_m": ESCAPE_LATERAL_LO_M if spec["escape"] else None,
        "escape_lateral_hi_m": ESCAPE_LATERAL_HI_M if spec["escape"] else None,
        "inter_drone_filter": False,
        "safe_radius_m": SAFE_RADIUS_M,
        "report_only": True,
        "deviates_from_upstream": True,
    }


def _load_swap3():
    from yopo_l0.scene import load_card

    return load_card(ROOT / SWAP3_REL)


def swap3_geometry() -> dict:
    """swap3 解析后的几何字典。圆柱与 ego-density-014.json 是同一份。"""
    return _load_swap3().card["geometry"]


def _coord(v: float) -> float:
    x = float(v)
    return 0.0 if x == 0.0 else x


def _drone_public(d: dict) -> dict:
    return {
        "id": d["id"],
        "start_enu": [_coord(d["start_enu"][0]), _coord(d["start_enu"][1]), _coord(d["start_enu"][2])],
        "goal_enu": [_coord(d["goal_enu"][0]), _coord(d["goal_enu"][1]), _coord(d["goal_enu"][2])],
    }


def placed_drones() -> list[dict]:
    geom = swap3_geometry()
    centers = [(float(c["center_xy"][0]), float(c["center_xy"][1])) for c in geom["cylinders"]]
    return load_ec6().place_swap6(centers)


def swap6_placement() -> list[dict]:
    out = []
    for d in placed_drones():
        out.append(
            {
                "id": d["id"],
                "start_enu": [_coord(x) for x in d["start_enu"]],
                "goal_enu": [_coord(x) for x in d["goal_enu"]],
                "radius_m": d["radius_m"],
                "pushed_outside": d["pushed_outside"],
                "axis_distance_start": d["axis_distance_start"],
                "axis_distance_goal": d["axis_distance_goal"],
            }
        )
    return out


def _swap6_note(placed: list[dict]) -> str:
    moved = [d for d in placed if d["pushed_outside"]]
    kept = [d for d in placed if not d["pushed_outside"]]
    parts = [
        "票 E-C8b：六机 goto 对向。障碍布局来自 scenes/ego-density-014-swap3.json 解析后的几何，",
        "也就是 scenes/ego-density-014.json（56 根圆柱，目标密度 0.14 个/m²）。",
        "这张卡只在 scripts/e-c8b-ablation.py 的进程内存里构造，不写入 scenes/。",
        "起终点规则与 scripts/e-c6-six-density.py 的六机对向相同。",
        "report-only。track_clearance=geom、empty_brake、peer_sphere_enabled、脱困重规划都偏离上游。",
        "E-A1 不写。safe_radius 不写。",
    ]
    if moved:
        bits = []
        for d in moved:
            bits.append(
                f"{d['id']} 在 8 m 圆上轴距不够，沿射线推到半径 {d['radius_m']:.3f} m"
            )
        parts.append("；".join(bits) + "。")
    if kept:
        parts.append(f"{'、'.join(d['id'] for d in kept)} 留在 8 m 圆上。")
    parts.append("seeds 0–4 由 RunConfig 传入。不进 scenes/suite.txt，criteria.frozen=false。")
    return "".join(parts)


def swap6_raw() -> dict:
    base = _load_swap3()
    placed = placed_drones()
    raw = base.card
    return {
        "name": SWAP6_NAME,
        "note": _swap6_note(placed),
        "geometry": raw["geometry"],
        "ground_z": raw.get("ground_z", 0.0),
        "drones": [_drone_public(d) for d in placed],
        "task": dict(raw["task"]),
        "criteria": dict(raw["criteria"]),
        "camera": dict(raw["camera"]),
        "dynamics": dict(raw["dynamics"]),
        "rates_hz": dict(raw["rates_hz"]),
        "seeds": [0],
    }


def apply_arm(raw: dict, arm: str) -> dict:
    spec = arm_spec(arm)
    out = json.loads(json.dumps(raw))
    if "policy_params" in out:
        raise RuntimeError("底卡不应自带 policy_params")
    params: dict = {}
    if spec["geom"] or spec["brake"]:
        upstream: dict = {}
        if spec["brake"]:
            upstream["empty_brake"] = True
        if spec["geom"]:
            upstream["track_clearance"] = True
            upstream["track_clearance_mode"] = "geom"
            upstream["track_margin_m"] = MARGIN_M
        if "safe_radius" in upstream or "inter_drone_filter" in upstream:
            raise RuntimeError("本票不写 safe_radius，也不开 E-A1")
        params["upstream"] = upstream
    if spec["sphere"] is not None:
        radius, weight = spec["sphere"]
        params["backend"] = {
            "enabled": True,
            "peer_sphere_enabled": True,
            "peer_sphere_radius_m": float(radius),
            "peer_sphere_weight": float(weight),
        }
    research = {
        "ticket": "E-C8b",
        "report_only": True,
        "deviates_from_upstream": True,
        "escape_replan": bool(spec["escape"]),
    }
    if spec["escape"]:
        research.update(
            escape_trigger_ticks=ESCAPE_TRIGGER_TICKS,
            escape_hold_ticks=ESCAPE_HOLD_TICKS,
            escape_forward_m=ESCAPE_FORWARD_M,
            escape_lateral_lo_m=ESCAPE_LATERAL_LO_M,
            escape_lateral_hi_m=ESCAPE_LATERAL_HI_M,
        )
    params["research"] = research
    out["policy_params"] = params
    return out


def assert_endpoints_clear(card) -> None:
    body = float(card.card["criteria"]["body_radius_m"])
    drones = card.card["drones"]
    if len(drones) != 6:
        raise RuntimeError(f"六机对向应有 6 架，收到 {len(drones)}")
    starts = []
    goals = []
    for d in drones:
        start = np.asarray(d["start_enu"], dtype=float)
        goal = np.asarray(d["goal_enu"], dtype=float)
        for label, p in (("起点", start), ("目标", goal)):
            dist = float(card.min_distance(p))
            if not dist > body + 1e-9:
                raise RuntimeError(f"{d['id']} {label} 到障碍表面 {dist:.4f} m，没有超过机体 {body} m")
        if float(np.linalg.norm(goal[:2] + start[:2])) > 1e-6 or abs(float(goal[2] - start[2])) > 1e-9:
            raise RuntimeError(f"{d['id']} 的目标不是起点的对径")
        starts.append(start)
        goals.append(goal)
    for label, points in (("起点", starts), ("目标", goals)):
        for i, a in enumerate(points):
            for b in points[i + 1 :]:
                gap = float(np.linalg.norm(a - b))
                if gap <= 2.0 * body + 1e-9:
                    raise RuntimeError(f"{label} 中心距 {gap:.4f} m，没有超过 {2.0 * body:.2f} m")


def build_card(arm: str):
    from yopo_l0.scene import SceneCard, parse_geometry

    raw = apply_arm(swap6_raw(), arm)
    card = SceneCard(raw["name"], raw, parse_geometry(raw["geometry"], raw.get("ground_z")))
    assert_endpoints_clear(card)
    if (ROOT / "scenes" / f"{SWAP6_NAME}.json").exists():
        raise RuntimeError("六机卡不应写到 scenes/")
    return card


def horizon_rows(height: int) -> tuple[int, int]:
    """地平线附近的中间三分之一行。用整数除法，避免 2/3 的二进制尾差。"""
    h = int(height)
    return h // 3, (2 * h) // 3


def sector_edges(width: int, n: int = N_SECTORS) -> np.ndarray:
    if int(width) < int(n):
        raise ValueError(f"宽度 {width} 小于扇区数 {n}")
    return np.linspace(0.0, float(width), int(n) + 1)


def sector_scores(depth: np.ndarray, *, open_range: float = OPEN_RANGE_M) -> np.ndarray:
    """每个扇区在地平线带上的深度中位数。无回波当作 open_range（通畅）。"""
    image = np.asarray(depth, dtype=float)
    if image.ndim != 2:
        raise ValueError(f"深度图应为 (H, W)，收到 {image.shape}")
    height, width = image.shape
    r0, r1 = horizon_rows(height)
    band = image[r0:r1]
    if band.size == 0:
        band = image
    filled = np.where(np.isfinite(band), band, float(open_range))
    edges = sector_edges(width)
    scores = np.empty(N_SECTORS, dtype=float)
    for i in range(N_SECTORS):
        c0 = int(math.floor(edges[i]))
        c1 = int(math.floor(edges[i + 1]))
        if i == N_SECTORS - 1:
            c1 = width
        sl = filled[:, c0:c1]
        scores[i] = float(np.median(sl)) if sl.size else -math.inf
    return scores


def sector_bearing_rad(index: int, width: int, hfov_deg: float = HFOV_DEG) -> float:
    """扇区中心相对光轴的方位角，弧度。正值指向图像右（机体右）。"""
    edges = sector_edges(width)
    mid = 0.5 * (float(edges[index]) + float(edges[index + 1]))
    u = mid / float(width)
    return (u - 0.5) * math.radians(float(hfov_deg))


def choose_sector(scores: np.ndarray, bearings: np.ndarray) -> int:
    """中位数最大。并列时更靠近正前方，再并列时取更左的扇区。"""
    best_i = 0
    best_key = None
    for i, score in enumerate(scores):
        key = (float(score), -abs(float(bearings[i])), -i)
        if best_key is None or key > best_key:
            best_key = key
            best_i = i
    return best_i


def lateral_m(bearing_rad: float, scores: np.ndarray, index: int, hfov_deg: float = HFOV_DEG) -> float:
    """机体 y，左正右负。幅度落在 [1.2, 1.5]。"""
    half = math.radians(float(hfov_deg)) / 2.0
    bearing = float(bearing_rad)
    if abs(bearing) <= 1e-12:
        left = float(scores[index - 1]) if index > 0 else -math.inf
        right = float(scores[index + 1]) if index + 1 < len(scores) else -math.inf
        sign = -1.0 if right > left + 1e-12 else 1.0
        return sign * ESCAPE_LATERAL_LO_M
    frac = min(abs(bearing) / half, 1.0) if half > 0.0 else 0.0
    mag = ESCAPE_LATERAL_LO_M + (ESCAPE_LATERAL_HI_M - ESCAPE_LATERAL_LO_M) * frac
    sign = -1.0 if bearing > 0.0 else 1.0
    return sign * mag


def body_to_world(position, yaw: float, forward: float, lateral: float) -> np.ndarray:
    c, s = math.cos(float(yaw)), math.sin(float(yaw))
    p = np.asarray(position, dtype=float)
    dx = c * float(forward) - s * float(lateral)
    dy = s * float(forward) + c * float(lateral)
    return np.array([p[0] + dx, p[1] + dy, p[2]], dtype=float)


def world_offset_body(position, yaw: float, goal) -> tuple[float, float]:
    """把世界系位移投回机体前向 / 左向，给测试和报告核对用。"""
    c, s = math.cos(float(yaw)), math.sin(float(yaw))
    p = np.asarray(position, dtype=float)
    g = np.asarray(goal, dtype=float)
    dx, dy = float(g[0] - p[0]), float(g[1] - p[1])
    forward = c * dx + s * dy
    lateral = -s * dx + c * dy
    return forward, lateral


def temp_goal_from_depth(depth, position, yaw: float) -> np.ndarray:
    """深度图最通畅扇区上的有界临时目标。没有深度图时向左前方 4.0 / 1.2。"""
    p = np.asarray(position, dtype=float)
    if depth is None:
        lateral = ESCAPE_LATERAL_LO_M
    else:
        image = np.asarray(depth, dtype=float)
        scores = sector_scores(image)
        bearings = np.array(
            [sector_bearing_rad(i, image.shape[1]) for i in range(N_SECTORS)], dtype=float
        )
        index = choose_sector(scores, bearings)
        lateral = lateral_m(float(bearings[index]), scores, index)
    goal = body_to_world(p, yaw, ESCAPE_FORWARD_M, lateral)
    if not np.all(np.isfinite(goal)):
        raise RuntimeError("临时目标不是有限数")
    return goal


class EscapeTracker:
    """单机脱困状态。触发的那一拍不消耗保持计数，下一拍才开始用临时目标。"""

    def __init__(self, trigger: int = ESCAPE_TRIGGER_TICKS, hold: int = ESCAPE_HOLD_TICKS):
        self.trigger = int(trigger)
        self.hold = int(hold)
        self.streak = 0
        self.hold_left = 0
        self.temp_goal: np.ndarray | None = None
        self.aim: np.ndarray | None = None
        self.triggers = 0
        self.held = 0
        self.reopened = 0
        self.expired = 0

    def active(self) -> bool:
        return self.hold_left > 0 and self.temp_goal is not None and self.aim is not None

    def observe(self, code: str, make_goal) -> None:
        if self.hold_left > 0:
            self.hold_left -= 1
            self.held += 1
            if code != "CORRIDOR_EMPTY_HOVER":
                self.hold_left = 0
                self.temp_goal = None
                self.aim = None
                self.streak = 0
                self.reopened += 1
            elif self.hold_left == 0:
                self.temp_goal = None
                self.aim = None
                self.streak = 0
                self.expired += 1
            return
        if code == "CORRIDOR_EMPTY_HOVER":
            self.streak += 1
            if self.streak >= self.trigger:
                goal, aim = make_goal()
                self.temp_goal = np.asarray(goal, dtype=float).copy()
                self.aim = np.asarray(aim, dtype=float).copy()
                self.hold_left = self.hold
                self.streak = 0
                self.triggers += 1
        else:
            self.streak = 0

    def snapshot(self) -> dict:
        return {
            "triggers": int(self.triggers),
            "held_ticks": int(self.held),
            "reopened": int(self.reopened),
            "expired": int(self.expired),
        }


def planning_goal(position, temp_goal, aim, arrive_radius: float) -> np.ndarray:
    """离临时目标还远就用它本身。已经进了到达半径就沿瞄准方向推出去。"""
    position = np.asarray(position, dtype=float)
    temp_goal = np.asarray(temp_goal, dtype=float)
    dist = float(np.linalg.norm(temp_goal - position))
    if dist >= float(arrive_radius) + ARRIVE_PUSH_SLACK_M:
        return temp_goal.copy()
    direction = np.asarray(aim, dtype=float)
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        direction = np.array([1.0, 0.0, 0.0])
        norm = 1.0
    return position + direction / norm * (float(arrive_radius) + ARRIVE_PUSH_AHEAD_M)


def _assign_goal(obs, goal) -> None:
    goal = np.asarray(goal, dtype=float)
    arr = np.asarray(obs.goal, dtype=float)
    if arr.shape != (3,) or goal.shape != (3,):
        raise RuntimeError(f"目标形状应为 (3,)，收到 {arr.shape} / {goal.shape}")
    if arr.flags.writeable:
        arr[:] = goal
        return
    object.__setattr__(obs, "goal", goal.copy())


def _escape_enabled(policy) -> bool:
    research = (getattr(policy, "ctx", None) and policy.ctx.params or {}).get("research") or {}
    return research.get("escape_replan") is True


def _plan_with_escape(policy, obs, orig_plan):
    tracker = getattr(policy, "_ec8", None)
    if tracker is None:
        tracker = EscapeTracker()
        policy._ec8 = tracker
    used_temp = tracker.active()
    if used_temp:
        goal = planning_goal(
            obs.state.p,
            tracker.temp_goal,
            tracker.aim,
            float(policy.p["arrive_radius_m"]),
        )
        _assign_goal(obs, goal)
    out = orig_plan(policy, obs)
    code = str(policy.last_code)

    def make_goal():
        depth = None if obs.frame is None else obs.frame.depth
        goal = temp_goal_from_depth(depth, obs.state.p, obs.state.yaw)
        aim = goal - np.asarray(obs.state.p, dtype=float)
        return goal, aim

    tracker.observe(code, make_goal)
    if used_temp and getattr(policy, "arrived", False) and code == "ARRIVED_HOLD":
        # 临时目标不是任务终点。解开到达锁，下一拍才能回到原目标。
        policy.arrived = False
    owner = str(policy.ctx.owner)
    _ESCAPE_STATS[owner] = tracker.snapshot()
    return out


def install_escape() -> None:
    """只在卡上写了 escape_replan=true 时改目标。其它策略调用原 plan。"""
    global _ESCAPE_INSTALLED, _ORIG_PLAN
    if _ESCAPE_INSTALLED:
        return
    import yopo_l0.policy as policy_mod

    _ORIG_PLAN = policy_mod.UpstreamPolicy.plan

    def plan(self, obs):
        if not _escape_enabled(self):
            return _ORIG_PLAN(self, obs)
        return _plan_with_escape(self, obs, _ORIG_PLAN)

    policy_mod.UpstreamPolicy.plan = plan
    _ESCAPE_INSTALLED = True


def uninstall_escape() -> None:
    """把 plan 还回包装之前。pytest 里装过之后必须拆掉，避免影响后面的用例。"""
    global _ESCAPE_INSTALLED, _ORIG_PLAN
    if not _ESCAPE_INSTALLED:
        return
    import yopo_l0.policy as policy_mod

    policy_mod.UpstreamPolicy.plan = _ORIG_PLAN
    _ORIG_PLAN = None
    _ESCAPE_INSTALLED = False


def install_peer_delivery() -> None:
    """欧氏球打开时把邻机采样送进后段。椭球项仍只看 swarm_penalty。

    与 scripts/e-b4c-sphere-avoid.py 同一层包装。policy.py 不改。
    两个开关都关时，传进 refine_yopo 的邻机仍是 None。
    """
    global _PEER_INSTALLED, _PEER_SAVED
    if _PEER_INSTALLED:
        return
    import yopo_l0.crossing as crossing_mod
    import yopo_l0.policy as policy_mod
    import yopo_l0.runner as runner_mod
    from yopo_l0.backend.config import peer_sphere_enabled as sphere_on

    orig_enabled = runner_mod.swarm_penalty_enabled
    orig_keep = policy_mod.UpstreamPolicy._keep_neighbor_samples
    orig_apply = policy_mod.UpstreamPolicy._apply_backend
    orig_cross = crossing_mod.CrossingFleet._apply_backend
    _PEER_SAVED = (orig_enabled, orig_keep, orig_apply, orig_cross)

    def enabled(params, **kwargs):
        if orig_enabled(params, **kwargs):
            return True
        return bool(sphere_on(params, **kwargs))

    runner_mod.swarm_penalty_enabled = enabled

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
    _PEER_INSTALLED = True


def uninstall_peer_delivery() -> None:
    """把邻机采样包装还回。pytest 里装过之后必须拆掉。"""
    global _PEER_INSTALLED, _PEER_SAVED
    if not _PEER_INSTALLED or _PEER_SAVED is None:
        return
    import yopo_l0.crossing as crossing_mod
    import yopo_l0.policy as policy_mod
    import yopo_l0.runner as runner_mod

    enabled, keep, apply, cross = _PEER_SAVED
    runner_mod.swarm_penalty_enabled = enabled
    policy_mod.UpstreamPolicy._keep_neighbor_samples = keep
    policy_mod.UpstreamPolicy._apply_backend = apply
    crossing_mod.CrossingFleet._apply_backend = cross
    _PEER_SAVED = None
    _PEER_INSTALLED = False


def _budget_wrap():
    import yopo_l0.backend.bspline as bspline

    current = bspline.refine_yopo
    if getattr(current, "_ec8b_wrapped", False):
        current.cuts = 0
        current.calls = 0
        return current
    orig = current

    def wrapped(*args, **kwargs):
        out, rec = orig(*args, **kwargs)
        wrapped.calls += 1
        wrapped.cuts += int(bool(rec.get("budget_cut")))
        return out, rec

    wrapped.calls = 0
    wrapped.cuts = 0
    wrapped._ec8b_wrapped = True
    bspline.refine_yopo = wrapped
    return wrapped


def _reset_escape_stats() -> None:
    _ESCAPE_STATS.clear()


def escape_totals() -> dict:
    rows = dict(_ESCAPE_STATS)
    keys = ("triggers", "held_ticks", "reopened", "expired")

    def add(name: str) -> int:
        return int(sum(int(row.get(name, 0)) for row in rows.values()))

    return {name: add(name) for name in keys} | {"by_drone": rows}


def avg_speed_mps(ticks: list[dict] | None) -> float | None:
    speeds: list[float] = []
    for row in ticks or []:
        v = row.get("v")
        if not isinstance(v, (list, tuple)) or len(v) != 3:
            continue
        try:
            speed = float(np.linalg.norm(np.asarray(v, dtype=float)))
        except (TypeError, ValueError):
            continue
        if math.isfinite(speed) and speed > 0.0:
            speeds.append(speed)
    if not speeds:
        return None
    return float(sum(speeds) / len(speeds))


def plan_wall_block(result: dict) -> dict:
    block = result.get("plan_wall_ms")
    if not isinstance(block, dict):
        return {"count": 0, "mean": None, "p50": None, "p95": None, "max": None}
    return {
        "count": int(block.get("count") or 0),
        "mean": block.get("mean"),
        "p50": block.get("p50"),
        "p95": block.get("p95"),
        "max": block.get("max"),
    }


def _is_plan_tick(k: int, *, physics: int = 100, planner: int = 10) -> bool:
    if k == 0:
        return True
    return (k * planner) // physics != ((k - 1) * planner) // physics


def corridor_empty_hover(ticks: list[dict] | None, *, physics: int = 100, planner: int = 10) -> int:
    count = 0
    for row in ticks or []:
        if not _is_plan_tick(int(row["k"]), physics=physics, planner=planner):
            continue
        if row.get("plan_code") == "CORRIDOR_EMPTY_HOVER":
            count += 1
    return count


def nearest_obstacle(card, position):
    p = np.asarray(position, dtype=float)
    best = None
    best_d = math.inf
    for ob in card.obstacles:
        dist = float(ob.distance(p))
        if dist < best_d:
            best, best_d = ob, dist
    return best, best_d


def classify_collision(card, result: dict, ticks: list[dict]) -> dict:
    """一次运行至多一种碰撞。仿真在第一次碰撞停下。"""
    from yopo_l0.scene import Box, Cylinder, Ground

    pillar = ground = box = inter = 0
    kind = None
    hit_uav = None
    hit_obstacle = None
    outcome = result.get("outcome")
    reason = str(result.get("reason") or "")
    if outcome == "COLLISION" and "INTER_DRONE" in reason:
        inter = 1
        kind = "inter"
        hit_uav = reason.split(":", 1)[0]
    elif outcome == "COLLISION" and "TRUTH_POSITION" in reason:
        drone = reason.split(":", 1)[0]
        hit_uav = drone
        rows = [row for row in ticks if row.get("uav") == drone]
        if not rows:
            raise RuntimeError(f"{drone} 碰撞但没有 tick")
        ob, _dist = nearest_obstacle(card, rows[-1]["p"])
        hit_obstacle = getattr(ob, "name", type(ob).__name__)
        if isinstance(ob, Ground):
            ground = 1
            kind = "ground"
        elif isinstance(ob, Cylinder):
            pillar = 1
            kind = "cylinder"
        elif isinstance(ob, Box):
            box = 1
            kind = "box"
        else:
            raise RuntimeError(f"未知障碍 {type(ob)!r}")
    return {
        "pillar_collision": pillar,
        "ground_collision": ground,
        "box_collision": box,
        "inter_drone_collision": inter,
        "hit_kind": kind,
        "hit_uav": hit_uav,
        "hit_obstacle": hit_obstacle,
    }


def surface_mins(card, ticks: list[dict]) -> dict:
    from yopo_l0.scene import Cylinder, Ground

    pillar = math.inf
    ground = math.inf
    for row in ticks:
        p = np.asarray(row["p"], dtype=float)
        for ob in card.obstacles:
            dist = float(ob.distance(p))
            if isinstance(ob, Cylinder) and dist < pillar:
                pillar = dist
            elif isinstance(ob, Ground) and dist < ground:
                ground = dist

    def fin(value: float):
        return None if not math.isfinite(value) else float(value)

    return {"min_pillar_surface_m": fin(pillar), "min_ground_surface_m": fin(ground)}


def positions_sha256(ticks: list[dict]) -> str:
    digest = hashlib.sha256()
    for row in ticks:
        digest.update(str(row["uav"]).encode())
        digest.update(np.asarray(row["p"], dtype=np.float64).tobytes())
        digest.update(np.float64(row["t"]).tobytes())
    return digest.hexdigest()


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
    return PARTIAL / f"{SCENE}-{arm}-{int(seed)}.json"


def load_partial(arm: str, seed: int) -> dict | None:
    dest = partial_path(arm, seed)
    if not dest.is_file():
        return None
    row = json.loads(dest.read_text(encoding="utf-8"))
    if int(row.get("row_version", 0)) != ROW_VERSION:
        return None
    return row


def save_partial(row: dict) -> None:
    dest = partial_path(row["arm"], int(row["seed"]))
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(_jsonable(row), ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)


def _check_result_flags(arm: str, res: dict) -> None:
    spec = arm_spec(arm)
    got = float(res["safe_radius_m"])
    if abs(got - SAFE_RADIUS_M) > 1e-9:
        raise RuntimeError(f"{arm} safe_radius 读回 {got}")
    if spec["brake"]:
        if res.get("empty_brake") is not True:
            raise RuntimeError(f"{arm} 没有打开 empty_brake")
    elif "empty_brake" in res:
        raise RuntimeError(f"{arm} 不应写出 empty_brake")
    if spec["geom"]:
        if res.get("track_clearance") is not True or res.get("track_clearance_mode") != "geom":
            raise RuntimeError(f"{arm} 的 track_clearance 读回 {res.get('track_clearance_mode')}")
        if abs(float(res.get("track_margin_m")) - MARGIN_M) > 1e-12:
            raise RuntimeError(f"{arm} margin 读回 {res.get('track_margin_m')}")
    elif "track_clearance" in res:
        raise RuntimeError(f"{arm} 不应写出 track_clearance")
    if "inter_drone_filter" in res:
        raise RuntimeError(f"{arm} 不应打开 E-A1")
    if spec["sphere"] is not None and not isinstance(res.get("backend"), dict):
        raise RuntimeError(f"{arm} 打开了欧氏球但没有后段记录")
    if spec["sphere"] is None and res.get("backend") is not None:
        raise RuntimeError(f"{arm} 没有后段却写出了 backend")
    runner_dev = bool(res["upstream_deviation"])
    expect_runner = bool(spec["geom"] or spec["brake"])
    if runner_dev != expect_runner:
        raise RuntimeError(f"{arm} 运行器偏离标记 {runner_dev}，期望 {expect_runner}")


def _simulate(arm: str, seed: int) -> dict:
    from yopo_l0.runner import RunConfig, run

    _reset_escape_stats()
    install_peer_delivery()
    install_escape()
    card = build_card(arm)
    wrapped = _budget_wrap()
    rates = card.card["rates_hz"]
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(seed), shadow=True))
    res = out.result
    _check_result_flags(arm, res)
    hits = classify_collision(card, res, out.ticks)
    surfaces = surface_mins(card, out.ticks)
    body = float(card.card["criteria"]["body_radius_m"])
    surface = float(res["min_obstacle_distance_m"])
    spec = arm_spec(arm)
    sphere = spec["sphere"]
    escape = escape_totals()
    if not spec["escape"] and any(int(escape[k]) for k in ("triggers", "held_ticks", "reopened", "expired")):
        raise RuntimeError(f"{arm} 没有脱困却记下了 {escape}")
    cfg = arm_config(arm)
    row = {
        "row_version": ROW_VERSION,
        "scene": SCENE,
        "scene_label": SCENE_LABEL,
        "arm": arm,
        "arm_label": arm_label(arm),
        "seed": int(seed),
        "upstream_deviation": True,
        "runner_upstream_deviation": bool(res["upstream_deviation"]),
        "report_only": True,
        "frozen": False,
        "deviation_mark": "偏离上游 / report-only",
        "policy": res["policy"],
        "task": res["task"],
        "safe_radius_m": float(res["safe_radius_m"]),
        "empty_brake": bool(spec["brake"]),
        "track_clearance": bool(spec["geom"]),
        "track_clearance_mode": "geom" if spec["geom"] else None,
        "track_margin_m": MARGIN_M if spec["geom"] else None,
        "peer_sphere_enabled": sphere is not None,
        "peer_sphere_radius_m": None if sphere is None else float(sphere[0]),
        "peer_sphere_weight": None if sphere is None else float(sphere[1]),
        "escape_replan": bool(spec["escape"]),
        "inter_drone_filter": False,
        "config": cfg,
        "scene_card_hash": res["scene_card_hash"],
        "outcome": res["outcome"],
        "reason": str(res["reason"]),
        "arrived": int(res["arrived"]),
        "n_drones": int(res["n_drones"]),
        "timeout": int(res["outcome"] == "TIMEOUT"),
        "min_surface_m": surface,
        "min_clearance_m": surface - body,
        "min_inter_drone_distance_m": res["min_inter_drone_distance_m"],
        "avg_speed_mps": avg_speed_mps(out.ticks),
        "plan_wall_ms": plan_wall_block(res),
        "corridor_empty_hover": corridor_empty_hover(
            out.ticks, physics=int(rates["physics"]), planner=int(rates["planner"])
        ),
        "escape_triggers": int(escape["triggers"]),
        "escape_held_ticks": int(escape["held_ticks"]),
        "escape_reopened": int(escape["reopened"]),
        "escape_expired": int(escape["expired"]),
        "escape_by_drone": escape["by_drone"],
        "sim_duration_s": res["sim_duration_s"],
        "arrival_time_s": res["arrival_time_s"],
        "positions_sha256": positions_sha256(out.ticks),
        "backend_budget_calls": int(wrapped.calls),
        "backend_budget_cuts": int(wrapped.cuts),
        "wall_time_s": res["wall_time_s"],
        "worker_s": time.perf_counter() - t0,
        "commit": res.get("commit"),
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
    def _num(key: str, default: float = 0.0) -> float:
        value = row.get(key, default)
        return default if value is None else float(value)

    print(
        f"{arm_label(arm)} seed={seed} {row.get('outcome')} {row.get('arrived')}/{row.get('n_drones')} "
        f"pillar={int(row.get('pillar_collision') or 0)} ground={int(row.get('ground_collision') or 0)} "
        f"inter={int(row.get('inter_drone_collision') or 0)} timeout={int(row.get('timeout') or 0)} "
        f"speed={speed_s} hover={int(row.get('corridor_empty_hover') or 0)} "
        f"escape={int(row.get('escape_triggers') or 0)}/{int(row.get('escape_held_ticks') or 0)} "
        f"sim={_num('sim_duration_s'):.2f} wall={_num('wall_time_s'):.1f}s "
        f"cuts={int(row.get('backend_budget_cuts') or 0)}/{int(row.get('backend_budget_calls') or 0)}",
        flush=True,
    )
    return row


def _worker(spec: tuple) -> dict:
    _limit_blas_threads()
    _clear_sphere_env()
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


def _run_group(specs: list[tuple], jobs: int) -> None:
    if not specs:
        return
    if jobs <= 1 or len(specs) <= 1:
        for spec in specs:
            _worker(spec)
        return
    import multiprocessing as mp

    errors = []
    with mp.get_context("spawn").Pool(jobs) as pool:
        pending = [(spec, pool.apply_async(_worker, (spec,))) for spec in specs]
        for spec, result in pending:
            try:
                result.get()
            except Exception as exc:  # noqa: BLE001 - 记下来，等这一组都结束再抛
                errors.append((spec, exc))
    if errors:
        spec, exc = errors[0]
        raise RuntimeError(f"{len(errors)} 个种子失败，最先记下的是 {spec}: {exc}") from exc


def _group(rows: list[dict], arm: str) -> list[dict]:
    return [r for r in rows if r["arm"] == arm]


def _rng(values: list[float], nd: int = 3) -> str:
    if not values:
        return "—"
    lo, hi = min(values), max(values)
    if abs(lo - hi) < 5 * 10 ** (-(nd + 1)):
        return f"{lo:.{nd}f}"
    sep = " 至 " if lo < 0.0 or hi < 0.0 else "–"
    return f"{lo:.{nd}f}{sep}{hi:.{nd}f}"


def _mean(values: list[float]) -> float | None:
    if not values:
        return None
    return float(sum(values) / len(values))


def _fmt(value, nd: int = 3) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{nd}f}"


def _speeds(group: list[dict]) -> list[float]:
    return [float(r["avg_speed_mps"]) for r in group if r.get("avg_speed_mps") is not None]


def _wall(group: list[dict], key: str) -> list[float]:
    out = []
    for row in group:
        value = (row.get("plan_wall_ms") or {}).get(key)
        if value is not None:
            out.append(float(value))
    return out


def render_summary(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 臂 | 标记 | 到达 | 撞柱 | 撞地 | 机间 | 超时 | 平均速度 m/s | 最小净空 m | 最小机间 m | 空走廊拍 | 脱困触发 | 脱困保持拍 | 重开回原目标 | 脱困到期 | 预算截断 |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for arm in ARMS:
        group = _group(rows, arm)
        if not group:
            continue
        drones = sum(int(r["n_drones"]) for r in group)
        arrived = sum(int(r["arrived"]) for r in group)
        pillar = sum(int(r["pillar_collision"]) for r in group)
        ground = sum(int(r["ground_collision"]) for r in group)
        inter = sum(int(r["inter_drone_collision"]) for r in group)
        timeout = sum(int(r["timeout"]) for r in group)
        hover = sum(int(r["corridor_empty_hover"]) for r in group)
        triggers = sum(int(r["escape_triggers"]) for r in group)
        held = sum(int(r["escape_held_ticks"]) for r in group)
        reopened = sum(int(r["escape_reopened"]) for r in group)
        expired = sum(int(r["escape_expired"]) for r in group)
        cuts = sum(int(r.get("backend_budget_cuts") or 0) for r in group)
        calls = sum(int(r.get("backend_budget_calls") or 0) for r in group)
        clears = [float(r["min_clearance_m"]) for r in group]
        gaps = [
            float(r["min_inter_drone_distance_m"])
            for r in group
            if r.get("min_inter_drone_distance_m") is not None
        ]
        speed = _mean(_speeds(group))
        lines.append(
            f"| {SCENE_LABEL} | {arm_label(arm)} | 偏离上游 / report-only | {arrived}/{drones} | "
            f"{pillar} | {ground} | {inter} | {timeout} | {_fmt(speed, 3)} | {_rng(clears, 3)} | "
            f"{_rng(gaps, 3)} | {hover} | {triggers} | {held} | {reopened} | {expired} | {cuts}/{calls} |"
        )
    return "\n".join(lines) + "\n"


def render_seeds(rows: list[dict]) -> str:
    lines = [
        "| 臂 | seed | 结果 | 原因 | 到达 | 撞柱 | 撞地 | 机间 | 超时 | 平均速度 m/s | plan 均值 ms | plan p95 ms | 最小净空 m | 最小机间 m | 空走廊拍 | 脱困触发 | 脱困保持 | 重开 | 到期 | 仿真 s |",
        "|---|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    by_key = {(r["arm"], int(r["seed"])): r for r in rows}
    for arm in ARMS:
        for seed in SEEDS:
            row = by_key.get((arm, seed))
            if row is None:
                continue
            wall = row.get("plan_wall_ms") or {}
            detail = str(row["reason"])
            if row.get("hit_obstacle"):
                detail = f"{detail} {row['hit_obstacle']}"
            lines.append(
                f"| {arm_label(arm)} | {seed} | {row['outcome']} | `{detail}` | "
                f"{int(row['arrived'])}/{int(row['n_drones'])} | {int(row['pillar_collision'])} | "
                f"{int(row['ground_collision'])} | {int(row['inter_drone_collision'])} | "
                f"{int(row['timeout'])} | {_fmt(row.get('avg_speed_mps'), 3)} | "
                f"{_fmt(wall.get('mean'), 2)} | {_fmt(wall.get('p95'), 2)} | "
                f"{_fmt(row.get('min_clearance_m'), 3)} | "
                f"{_fmt(row.get('min_inter_drone_distance_m'), 3)} | "
                f"{int(row['corridor_empty_hover'])} | {int(row['escape_triggers'])} | "
                f"{int(row['escape_held_ticks'])} | {int(row['escape_reopened'])} | "
                f"{int(row['escape_expired'])} | {_fmt(row.get('sim_duration_s'), 2)} |"
            )
    return "\n".join(lines) + "\n"


def render_placement(placed: list[dict] | None = None) -> str:
    placed = swap6_placement() if placed is None else placed
    lines = [
        "| 飞机 | 半径 m | 推出正方形 | 起点轴距 m | 目标轴距 m | 起点 | 目标 |",
        "|---|---:|---|---:|---:|---|---|",
    ]
    for d in placed:
        start = ", ".join(f"{float(x):.3f}" for x in d["start_enu"])
        goal = ", ".join(f"{float(x):.3f}" for x in d["goal_enu"])
        pushed = "是" if d["pushed_outside"] else "否"
        lines.append(
            f"| {d['id']} | {float(d['radius_m']):.3f} | {pushed} | "
            f"{float(d['axis_distance_start']):.3f} | {float(d['axis_distance_goal']):.3f} | "
            f"{start} | {goal} |"
        )
    return "\n".join(lines) + "\n"


def _load_partials(jobs: list[tuple]) -> list[dict]:
    rows = []
    for arm, seed in jobs:
        row = load_partial(arm, seed)
        if row is not None:
            rows.append(row)
    return rows


def _write_payload(jobs: list[tuple], out_path: Path) -> dict:
    rows = _load_partials(jobs)
    rows.sort(key=lambda r: (ARMS.index(r["arm"]), int(r["seed"])))
    payload = {
        "disclaimer": DISCLAIMER,
        "report_only": True,
        "frozen": False,
        "deviates_from_upstream": True,
        "scene": SCENE,
        "scene_label": SCENE_LABEL,
        "scene_source": SWAP3_REL,
        "geometry_file": GEOM_NAME,
        "geom_sha256": GEOM_SHA256,
        "swap6_name": SWAP6_NAME,
        "arms": list(ARMS),
        "arm_configs": {arm: arm_config(arm) for arm in ARMS},
        "seeds": list(SEEDS),
        "margin_m": MARGIN_M,
        "body_radius_m": BODY_RADIUS_M,
        "safe_radius_m": SAFE_RADIUS_M,
        "inter_drone_filter": False,
        "escape": {
            "trigger_ticks": ESCAPE_TRIGGER_TICKS,
            "hold_ticks": ESCAPE_HOLD_TICKS,
            "forward_m": ESCAPE_FORWARD_M,
            "lateral_lo_m": ESCAPE_LATERAL_LO_M,
            "lateral_hi_m": ESCAPE_LATERAL_HI_M,
            "sectors": N_SECTORS,
            "hfov_deg": HFOV_DEG,
            "open_range_m": OPEN_RANGE_M,
            "definition": ESCAPE_DEF,
        },
        "avg_speed_def": AVG_SPEED_DEF,
        "hover_def": HOVER_DEF,
        "peer_delivery": (
            "与 scripts/e-b4c-sphere-avoid.py 相同：进程内包一层，欧氏球打开时才把邻机采样送进 "
            "refine_yopo。运行器在本 tick 全部 plan() 之后才 bus.poll，后段用的是上一规划拍已提交的采样。"
            "不改 policy.py。"
        ),
        "swap6": swap6_placement(),
        "completed": len(rows),
        "expected": len(jobs),
        "incomplete": len(jobs) - len(rows),
        "rows": rows,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(render_summary(rows))
    print(f"wrote {out_path} ({payload['completed']}/{payload['expected']})")
    return payload


def _parse_list(text: str | None, allowed: tuple[str, ...] | tuple[int, ...], fallback):
    if text is None:
        return list(fallback)
    items = []
    for part in text.split(","):
        if part == "":
            continue
        if allowed and isinstance(allowed[0], int):
            items.append(int(part))
        else:
            items.append(part)
    return items


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="E-C8b 六机 014 消融，只报数")
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    ap.add_argument("--jobs", type=int, default=MAX_JOBS)
    ap.add_argument("--out", default="docs/research/e-c8b-ablation-runs.json")
    ap.add_argument("--no-resume", action="store_true")
    ap.add_argument("--check", action="store_true", help="只检查内存卡和六臂开关，不跑仿真")
    a = ap.parse_args(argv)
    if int(a.jobs) > MAX_JOBS:
        raise SystemExit(f"最多 {MAX_JOBS} 个进程，避免墙钟预算把后段迭代截短")
    if int(a.jobs) < 1:
        raise SystemExit("--jobs 至少为 1")
    arms = _parse_list(a.arms, ARMS, ARMS)
    seeds = _parse_list(a.seeds, SEEDS, SEEDS)
    unknown = [arm for arm in arms if arm not in ARMS]
    if unknown or any(int(s) < 0 for s in seeds):
        raise SystemExit(f"未知臂或种子: {unknown or seeds}")
    if a.check:
        blob = (ROOT / "scenes" / GEOM_NAME).read_bytes()
        digest = hashlib.sha256(blob).hexdigest()
        if digest != GEOM_SHA256:
            raise SystemExit(f"{GEOM_NAME} sha256 {digest} 与票面不符")
        raw_swap3 = json.loads((ROOT / SWAP3_REL).read_text(encoding="utf-8"))
        if raw_swap3.get("geometry") != GEOM_NAME:
            raise SystemExit("swap3 的几何不是 ego-density-014.json")
        for arm in ARMS:
            card = build_card(arm)
            if card.card["task"]["type"] != "goto":
                raise SystemExit(arm)
            if len(card.card["geometry"]["cylinders"]) != 56:
                raise SystemExit("柱子不是 56 根")
        print(render_placement())
        for arm in ARMS:
            cfg = arm_config(arm)
            print(
                f"{arm_label(arm)} geom={cfg['track_clearance']} brake={cfg['empty_brake']} "
                f"sphere={cfg['peer_sphere_radius_m']}/{cfg['peer_sphere_weight']} "
                f"escape={cfg['escape_replan']}",
                flush=True,
            )
        print("check ok", flush=True)
        return 0

    _limit_blas_threads()
    _clear_sphere_env()
    from yopo_swarm_core.baseline import fetch_upstream

    fetch_upstream()
    resume = not a.no_resume
    jobs = [(arm, int(seed)) for arm in arms for seed in seeds]
    specs = job_specs(arms, seeds, resume=resume)
    plain = [spec for spec in specs if arm_spec(spec[0])["sphere"] is None]
    backend = [spec for spec in specs if arm_spec(spec[0])["sphere"] is not None]
    out = ROOT / a.out
    try:
        _run_group(plain, int(a.jobs))
        _run_group(backend, int(a.jobs))
    finally:
        payload = _write_payload(jobs, out)
    return 0 if payload["incomplete"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
