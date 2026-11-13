#!/usr/bin/env python3
"""票 E-C6：六机密度场 0.14。只报数。

两个场景，seeds 0–4，四档都在进程里改参数，不改场景文件：

- crossing-6：已有的 scenes/crossing-6-forest.json（六机穿林）
- d014-swap6：在 scenes/ego-density-014.json 的柱子上，按圆周对称放 6 架对向。
  只存在于这次进程的内存里，不写 scenes/

四档：

- off：默认上游 + 影子盾。不写 policy_params
- base：E-A1 阈值 1.0 m、惩罚 0，加上 empty_brake（偏离上游）
- geom：base + track_clearance mode=geom，margin=0.08 m（偏离上游）
- geom_multi：base + track_clearance mode=geom_multi，margin=0.08 m（偏离上游）

穿林规划器只读 E-A1。empty_brake 和 track_clearance 写在进程内卡上，
CrossingFleet 不读，结果里也不会出现这两个字段。014 六机对向是 goto，三张开关都进 UpstreamPolicy。

0.08 m 是假设，用来盖 E-C5 记下的 0.058–0.073 m 跟踪差。不是论文复现
（障碍半径 0.20 m，边长 20.0 m）。不改 core/、已冻门、scenes/、docs/tickets.md、
README、scripts/regress.sh，也不改任何默认值。结果不进冻结判据。
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
PARTIAL = ROOT / "out" / "e-c6-six-density" / "partial"
GEOM_NAME = "ego-density-014.json"
GEOM_SHA256 = "5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9"
CROSSING_REL = "scenes/crossing-6-forest.json"

SCENE_ORDER = ("crossing-6", "d014-swap6")
SCENE_LABEL = {"crossing-6": "六机林带", "d014-swap6": "014 六机对向"}
ARMS = ("off", "base", "geom", "geom_multi")
SEEDS = tuple(range(5))
ROW_VERSION = 1

BODY_RADIUS_M = 0.35
OBSTACLE_RADIUS_M = 0.20
AXIS_CLEARANCE_M = 0.70  # 机体 0.35 + 障碍半径 0.20 + 余量 0.15
MARGIN_M = 0.08
A1_THRESHOLD_M = 1.0
A1_PENALTY = 0
HEIGHT_M = 1.5
NOMINAL_SWAP_RADIUS_M = 8.0
MAP_SIDE_M = 20.0
VALID_SPEED_EPS_MPS = 0.0
SWAP6_NAME = "ego-density-014-swap6"

# 60° 一档。0° 与 swap-3 的 uav0 相同。对径是 i+3。
_S3 = math.sqrt(3.0) / 2.0
SWAP6_UNIT = (
    (1.0, 0.0),
    (0.5, _S3),
    (-0.5, _S3),
    (-1.0, 0.0),
    (-0.5, -_S3),
    (0.5, -_S3),
)

DISCLAIMER = (
    "只报数。report-only。不改默认 safe_radius 0.05 m、empty_brake false、"
    "track_clearance false、inter_drone_filter 关。"
    "E-A1 阈值 1.0 m、empty_brake、track_clearance 打开即偏离上游 d5e1e60。"
    "track_margin 0.08 m 是假设，用来盖已观测的 0.058–0.073 m 跟踪差。"
    "六机 0.14 对向卡只存在于进程内存，不写入 scenes/。"
    "这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。"
    "未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。"
)

AVG_SPEED_DEF = (
    "avg_speed_mps 是该次运行全部 tick、全部飞机的 ||v|| 里，"
    "有限且严格大于 0 的样本的算术平均。悬停和初值 0 不算有效飞行。"
    "没有有效样本时为 null。"
)

A1 = {
    "inter_drone_filter": True,
    "inter_drone_threshold_m": A1_THRESHOLD_M,
    "inter_drone_penalty": A1_PENALTY,
}


def load_ec4():
    name = "e_c4_density_l0_for_c6"
    if name in sys.modules:
        return sys.modules[name]
    path = ROOT / "scripts" / "e-c4-density-l0.py"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"无法加载 {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def arm_deviates(arm: str) -> bool:
    return arm != "off"


def arm_label(arm: str) -> str:
    labels = {
        "off": "默认上游 + 影子盾",
        "base": "E-A1 1.0 m + empty_brake（偏离上游）",
        "geom": "base + track geom 0.08 m（偏离上游）",
        "geom_multi": "base + track geom_multi 0.08 m（偏离上游）",
    }
    if arm not in labels:
        raise ValueError(arm)
    return labels[arm]


def density_geometry() -> dict:
    return json.loads((ROOT / "scenes" / GEOM_NAME).read_text(encoding="utf-8"))


def density_centers(geom: dict | None = None) -> list[tuple[float, float]]:
    geom = density_geometry() if geom is None else geom
    return [(float(c["center_xy"][0]), float(c["center_xy"][1])) for c in geom["cylinders"]]


def place_swap6(centers: list[tuple[float, float]]) -> list[dict]:
    """6 架均匀放在名义半径 8 m 的圆上，各自飞向对径。

    起终点到最近圆柱轴的水平距离必须 ≥ 0.70 m。不够就沿同一条射线推到
    20 m 正方形外面，刚满足净空后再加 0.05 m。与 E-C4 的三机对向同一条规则。
    """
    ec4 = load_ec4()
    drones = []
    for i, (ux, uy) in enumerate(SWAP6_UNIT):
        ux, uy = ec4._unit(ux, uy)
        x8, y8 = NOMINAL_SWAP_RADIUS_M * ux, NOMINAL_SWAP_RADIUS_M * uy
        d_start = ec4.axis_distance(x8, y8, centers)
        d_goal = ec4.axis_distance(-x8, -y8, centers)
        keep = d_start >= AXIS_CLEARANCE_M - 1e-12 and d_goal >= AXIS_CLEARANCE_M - 1e-12
        if keep:
            radius = NOMINAL_SWAP_RADIUS_M
            sx, sy = x8, y8
            pushed = False
            r_min = r_edge = None
        else:
            radius, r_min, r_edge = ec4.first_clear_swap_radius(ux, uy, centers)
            sx, sy = radius * ux, radius * uy
            pushed = True
        start_axis = ec4.axis_distance(sx, sy, centers)
        goal_axis = ec4.axis_distance(-sx, -sy, centers)
        if start_axis < AXIS_CLEARANCE_M - 1e-9 or goal_axis < AXIS_CLEARANCE_M - 1e-9:
            raise RuntimeError(f"uav{i} 起终点轴距 {start_axis:.4f} / {goal_axis:.4f} 低于 0.70 m")
        drones.append(
            {
                "id": f"uav{i}",
                "start_enu": [sx, sy, HEIGHT_M],
                "goal_enu": [-sx, -sy, HEIGHT_M],
                "nominal_radius_m": NOMINAL_SWAP_RADIUS_M,
                "radius_m": radius,
                "pushed_outside": pushed,
                "axis_distance_at_8m_start": d_start,
                "axis_distance_at_8m_goal": d_goal,
                "axis_distance_start": start_axis,
                "axis_distance_goal": goal_axis,
                "r_min_clear_m": r_min,
                "r_square_edge_m": r_edge,
            }
        )
    return drones


def _coord(v: float) -> float:
    x = float(v)
    return 0.0 if x == 0.0 else x


def _drone_public(d: dict) -> dict:
    return {
        "id": d["id"],
        "start_enu": [_coord(d["start_enu"][0]), _coord(d["start_enu"][1]), _coord(d["start_enu"][2])],
        "goal_enu": [_coord(d["goal_enu"][0]), _coord(d["goal_enu"][1]), _coord(d["goal_enu"][2])],
    }


def _swap6_note(placed: list[dict]) -> str:
    moved = [d for d in placed if d["pushed_outside"]]
    kept = [d for d in placed if not d["pushed_outside"]]
    parts = [
        "票 E-C6：六机 goto 对向，几何是 scenes/ego-density-014.json（56 根圆柱，目标密度 0.14 个/m²）。",
        "这张卡只在 scripts/e-c6-six-density.py 的进程内存里构造，不写入 scenes/。",
        f"这是假设，不是论文复现（障碍半径 {OBSTACLE_RADIUS_M:.2f} m，边长 {MAP_SIDE_M:.1f} m）。",
        "起点名义上均匀分布在半径 8.0 m、高度 1.5 m 的圆上，方位间隔 60°，各自飞向直径对侧。",
        "起终点必须在自由空间：机体中心到最近圆柱轴的水平距离至少 0.70 m"
        "（机体半径 0.35 m + 障碍半径 0.20 m + 余量 0.15 m）。",
    ]
    if moved:
        bits = []
        for d in moved:
            bits.append(
                f"{d['id']} 在 8 m 圆上到最近轴是 {d['axis_distance_at_8m_start']:.3f} m / "
                f"{d['axis_distance_at_8m_goal']:.3f} m，低于 0.70 m，"
                f"因此沿同一条射线把起终点移到 20 m 正方形外面（半径 {d['radius_m']:.3f} m，"
                f"刚满足净空后再加 0.05 m）"
            )
        parts.append("；".join(bits) + "。")
    if kept:
        ids = "、".join(d["id"] for d in kept)
        parts.append(f"{ids} 在 8 m 圆上已经满足 0.70 m，仍留在圆上。")
    parts.append(
        "脚本跑 seeds 0–4。本卡 seeds 只声明 0，与 E-C4 的密度卡一样；种子由 RunConfig 传入。"
        "不进 scenes/suite.txt，criteria.frozen=false，不设 gates。"
        "开关不写进磁盘上的任何卡：off 不写 policy_params，其余三档由本脚本在进程里覆盖。"
        "速度与 a_max 用本仓库已有的点质量设定：场景卡无 speed_mps，dynamics.a_max=6.0、tau_s=0.1、kp=4.0、kd=3.0。"
    )
    return "".join(parts)


def swap6_raw() -> dict:
    ec4 = load_ec4()
    geom = density_geometry()
    placed = place_swap6(density_centers(geom))
    return {
        "name": SWAP6_NAME,
        "note": _swap6_note(placed),
        "geometry": geom,
        "ground_z": 0.0,
        "drones": [_drone_public(d) for d in placed],
        "task": {"type": "goto", "duration_s": 40.0},
        "criteria": dict(ec4.CRITERIA),
        "camera": dict(ec4.CAMERA),
        "dynamics": dict(ec4.DYNAMICS),
        "rates_hz": dict(ec4.RATES),
        "seeds": [0],
    }


def swap6_placement() -> list[dict]:
    """给报告用的起终点，不含几何。"""
    placed = place_swap6(density_centers())
    out = []
    for d in placed:
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


def apply_upstream(raw: dict, arm: str) -> dict:
    """深拷贝后只加这一档要开的字段。不写 safe_radius。"""
    if arm not in ARMS:
        raise ValueError(arm)
    out = json.loads(json.dumps(raw))
    if arm == "off":
        if "policy_params" in out:
            raise RuntimeError("底卡不应自带 policy_params")
        return out
    params = dict(out.get("policy_params") or {})
    upstream = dict(params.get("upstream") or {})
    upstream.update(A1)
    upstream["empty_brake"] = True
    if arm == "geom":
        upstream.update(track_clearance=True, track_clearance_mode="geom", track_margin_m=MARGIN_M)
    elif arm == "geom_multi":
        upstream.update(
            track_clearance=True, track_clearance_mode="geom_multi", track_margin_m=MARGIN_M
        )
    elif arm != "base":
        raise ValueError(arm)
    if "safe_radius" in upstream:
        raise RuntimeError("对照不写 safe_radius，0.05 m 走运行器缺省")
    if "track_clearance" in upstream and arm == "base":
        raise RuntimeError("base 臂不写 track_clearance")
    params["upstream"] = upstream
    out["policy_params"] = params
    return out


def build_card(scene: str, arm: str):
    from yopo_l0.scene import SceneCard, load_card, parse_geometry

    if arm not in ARMS:
        raise ValueError(arm)
    if scene == "crossing-6":
        base = load_card(ROOT / CROSSING_REL)
        if arm == "off":
            if "policy_params" in base.card:
                raise RuntimeError("六机林带底卡不应自带 policy_params")
            return base
        raw = apply_upstream(base.card, arm)
        return SceneCard(base.path, raw, base.obstacles)
    if scene == "d014-swap6":
        raw = apply_upstream(swap6_raw(), arm)
        card = SceneCard(raw["name"], raw, parse_geometry(raw["geometry"], raw.get("ground_z")))
        assert_endpoints_clear(card)
        return card
    raise ValueError(scene)


def assert_endpoints_clear(card) -> None:
    """起点、目标与柱子和彼此都不碰。机体半径取卡上的 0.35 m。"""
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
    _assert_separation(starts, 2.0 * body, "起点")
    _assert_separation(goals, 2.0 * body, "目标")


def _assert_separation(points: list[np.ndarray], limit: float, label: str) -> None:
    for i, a in enumerate(points):
        for b in points[i + 1 :]:
            gap = float(np.linalg.norm(a - b))
            if gap <= limit + 1e-9:
                raise RuntimeError(f"{label} 中心距 {gap:.4f} m，没有超过 {limit:.2f} m")


def avg_speed_mps(ticks: list[dict] | None) -> float | None:
    """全程有效飞行平均速度。见 AVG_SPEED_DEF。"""
    speeds: list[float] = []
    for row in ticks or []:
        v = row.get("v")
        if not isinstance(v, (list, tuple)) or len(v) != 3:
            continue
        try:
            speed = float(np.linalg.norm(np.asarray(v, dtype=float)))
        except (TypeError, ValueError):
            continue
        if math.isfinite(speed) and speed > VALID_SPEED_EPS_MPS:
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


def corridor_empty_hover(result: dict) -> int:
    codes = result.get("plan_codes") or {}
    return int(codes.get("CORRIDOR_EMPTY_HOVER", 0))


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
    outcome = result.get("outcome")
    reason = str(result.get("reason") or "")
    if outcome == "COLLISION" and "INTER_DRONE" in reason:
        inter = 1
        kind = "inter"
    elif outcome == "COLLISION" and "TRUTH_POSITION" in reason:
        drone = reason.split(":", 1)[0]
        rows = [row for row in ticks if row.get("uav") == drone]
        if not rows:
            raise RuntimeError(f"{drone} 碰撞但没有 tick")
        ob, _dist = nearest_obstacle(card, rows[-1]["p"])
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
    }


def surface_mins(card, ticks: list[dict]) -> dict:
    from yopo_l0.scene import Cylinder, Ground

    pillar = math.inf
    ground = math.inf
    for row in ticks:
        p = np.asarray(row["p"], dtype=float)
        for ob in card.obstacles:
            dist = float(ob.distance(p))
            if isinstance(ob, Cylinder):
                if dist < pillar:
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


def partial_path(scene: str, arm: str, seed: int) -> Path:
    return PARTIAL / f"{scene}-{arm}-{int(seed)}.json"


def load_partial(scene: str, arm: str, seed: int) -> dict | None:
    dest = partial_path(scene, arm, seed)
    if not dest.is_file():
        return None
    row = json.loads(dest.read_text(encoding="utf-8"))
    if int(row.get("row_version", 0)) != ROW_VERSION:
        return None
    return row


def save_partial(row: dict) -> None:
    dest = partial_path(row["scene"], row["arm"], int(row["seed"]))
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(_jsonable(row), ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)


def _crossing_file_hash() -> str:
    from yopo_l0.scene import load_card

    return load_card(ROOT / CROSSING_REL).hash


def _simulate(scene: str, arm: str, seed: int) -> dict:
    from yopo_l0.runner import RunConfig, run

    card = build_card(scene, arm)
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(seed), shadow=True))
    res = out.result
    got = float(res["safe_radius_m"])
    if abs(got - 0.05) > 1e-9:
        raise RuntimeError(f"safe_radius 读回 {got}，四档都保持 0.05 m")
    hits = classify_collision(card, res, out.ticks)
    surfaces = surface_mins(card, out.ticks)
    body = float(card.card["criteria"]["body_radius_m"])
    surface = float(res["min_obstacle_distance_m"])
    runner_dev = bool(res["upstream_deviation"])
    if scene == "d014-swap6":
        if runner_dev != arm_deviates(arm):
            raise RuntimeError(f"{arm} 偏离标记 {runner_dev} 与开关不一致")
        if arm == "off":
            if "empty_brake" in res or "track_clearance" in res or "inter_drone_filter" in res:
                raise RuntimeError("全关臂不应写出 empty_brake / track_clearance / inter_drone_filter")
        else:
            if res.get("empty_brake") is not True:
                raise RuntimeError(f"{arm} 没有打开 empty_brake")
            filt = res.get("inter_drone_filter") or {}
            if abs(float(filt.get("threshold_m", -1)) - A1_THRESHOLD_M) > 1e-12:
                raise RuntimeError(f"{arm} E-A1 阈值读回 {filt.get('threshold_m')}")
        if arm in ("geom", "geom_multi"):
            if res.get("track_clearance") is not True or res.get("track_clearance_mode") != arm:
                raise RuntimeError(f"{arm} 的 track_clearance 读回 {res.get('track_clearance_mode')}")
            if abs(float(res.get("track_margin_m")) - MARGIN_M) > 1e-12:
                raise RuntimeError(f"{arm} margin 读回 {res.get('track_margin_m')}")
        if arm == "base" and "track_clearance" in res:
            raise RuntimeError("base 臂写出了 track_clearance")
    elif scene == "crossing-6":
        if runner_dev:
            raise RuntimeError("穿林半径仍是 0.05 m，运行器不应因本票开关标偏离")
        if "empty_brake" in res or "track_clearance" in res:
            raise RuntimeError("穿林结果不应写出 goto 专用的 empty_brake / track_clearance")
        if arm == "off":
            if "inter_drone_filter" in res:
                raise RuntimeError("穿林全关臂不应写出 inter_drone_filter")
        else:
            filt = res.get("inter_drone_filter") or {}
            if filt.get("enabled") is not True:
                raise RuntimeError(f"穿林 {arm} 没有读到 E-A1")
            if abs(float(filt.get("threshold_m")) - A1_THRESHOLD_M) > 1e-12:
                raise RuntimeError(f"穿林 {arm} 阈值读回 {filt.get('threshold_m')}")
    else:
        raise ValueError(scene)
    filt = res.get("inter_drone_filter") or {}
    row = {
        "row_version": ROW_VERSION,
        "scene": scene,
        "arm": arm,
        "seed": int(seed),
        "upstream_deviation": arm_deviates(arm),
        "runner_upstream_deviation": runner_dev,
        "report_only": True,
        "frozen": False,
        "policy": res["policy"],
        "task": res["task"],
        "safe_radius_m": got,
        "empty_brake": res.get("empty_brake"),
        "track_clearance": res.get("track_clearance"),
        "track_clearance_mode": res.get("track_clearance_mode"),
        "track_margin_m": res.get("track_margin_m"),
        "scene_card_hash": res["scene_card_hash"],
        "file_hash": _crossing_file_hash() if scene == "crossing-6" else None,
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
        "corridor_empty_hover": corridor_empty_hover(res),
        "sim_duration_s": res["sim_duration_s"],
        "arrival_time_s": res["arrival_time_s"],
        "positions_sha256": positions_sha256(out.ticks),
        "inter_threshold_m": filt.get("threshold_m"),
        "inter_filtered": filt.get("filtered_candidates"),
        "inter_empty_fallbacks": filt.get("empty_fallbacks"),
        "wall_time_s": res["wall_time_s"],
        "worker_s": time.perf_counter() - t0,
        **hits,
        **surfaces,
    }
    return _jsonable(row)


def run_one(scene: str, arm: str, seed: int, *, resume: bool) -> dict:
    if resume:
        cached = load_partial(scene, arm, seed)
        if cached is not None:
            print(f"{scene} {arm_label(arm)} seed={seed} resume", flush=True)
            return cached
    row = _simulate(scene, arm, seed)
    save_partial(row)
    speed = row["avg_speed_mps"]
    speed_s = "—" if speed is None else f"{float(speed):.3f}"
    wall = row["plan_wall_ms"]
    mean_s = "—" if wall.get("mean") is None else f"{float(wall['mean']):.2f}"
    p95_s = "—" if wall.get("p95") is None else f"{float(wall['p95']):.2f}"
    print(
        f"{scene} {arm_label(arm)} seed={seed} {row['outcome']} "
        f"{row['arrived']}/{row['n_drones']} pillar={row['pillar_collision']} "
        f"ground={row['ground_collision']} box={row['box_collision']} "
        f"inter={row['inter_drone_collision']} speed={speed_s} "
        f"plan={mean_s}/{p95_s} hover={row['corridor_empty_hover']} "
        f"sim={float(row['sim_duration_s']):.2f} wall={float(row['wall_time_s']):.1f}s",
        flush=True,
    )
    return row


def _worker(spec: tuple) -> dict:
    try:
        os.nice(10)
    except OSError:
        pass
    scene, arm, seed, resume = spec
    try:
        return run_one(str(scene), str(arm), int(seed), resume=bool(resume))
    except Exception:
        print(f"FAIL {scene} {arm} seed={seed}", flush=True)
        raise


def job_specs(scenes, arms, seeds, *, resume: bool = True) -> list[tuple]:
    return [
        (scene, arm, int(seed), bool(resume))
        for arm in arms
        for scene in scenes
        for seed in seeds
    ]


def _run_group(specs: list[tuple], jobs: int) -> list[dict]:
    if jobs <= 1 or len(specs) <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)


def _group(rows: list[dict], scene: str, arm: str) -> list[dict]:
    return [r for r in rows if r["scene"] == scene and r["arm"] == arm]


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


def _wall_means(group: list[dict]) -> list[float]:
    out = []
    for row in group:
        value = (row.get("plan_wall_ms") or {}).get("mean")
        if value is not None:
            out.append(float(value))
    return out


def _wall_p95(group: list[dict]) -> list[float]:
    out = []
    for row in group:
        value = (row.get("plan_wall_ms") or {}).get("p95")
        if value is not None:
            out.append(float(value))
    return out


def render_summary(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 臂 | 偏离上游 | 到达 | 撞柱 | 撞地 | 撞箱 | 机间 | 超时 | 平均速度 m/s | plan_wall 均值 ms | plan_wall p95 ms | 最小净空 m | 最小机间 m | CORRIDOR_EMPTY_HOVER |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---:|",
    ]
    for scene in SCENE_ORDER:
        for arm in ARMS:
            group = _group(rows, scene, arm)
            if not group:
                continue
            drones = sum(int(r["n_drones"]) for r in group)
            arrived = sum(int(r["arrived"]) for r in group)
            pillar = sum(int(r["pillar_collision"]) for r in group)
            ground = sum(int(r["ground_collision"]) for r in group)
            box = sum(int(r["box_collision"]) for r in group)
            inter = sum(int(r["inter_drone_collision"]) for r in group)
            timeout = sum(int(r["timeout"]) for r in group)
            hover = sum(int(r["corridor_empty_hover"]) for r in group)
            clears = [float(r["min_clearance_m"]) for r in group]
            gaps = [
                float(r["min_inter_drone_distance_m"])
                for r in group
                if r.get("min_inter_drone_distance_m") is not None
            ]
            dev = "偏离上游" if arm_deviates(arm) else "否"
            lines.append(
                f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | {dev} | {arrived}/{drones} | "
                f"{pillar} | {ground} | {box} | {inter} | {timeout} | {_fmt(_mean(_speeds(group)))} | "
                f"{_fmt(_mean(_wall_means(group)), 2)} | {_fmt(_mean(_wall_p95(group)), 2)} | "
                f"{_rng(clears)} | {_rng(gaps)} | {hover} |"
            )
    return "\n".join(lines)


def render_seeds(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 臂 | seed | 结果 | 原因 | 到达 | 撞柱 | 撞地 | 机间 | 平均速度 m/s | plan 均值 ms | plan p95 ms | 最小净空 m | 最小机间 m | CORRIDOR_EMPTY_HOVER | 仿真 s |",
        "|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for scene in SCENE_ORDER:
        for arm in ARMS:
            for row in sorted(_group(rows, scene, arm), key=lambda r: int(r["seed"])):
                wall = row.get("plan_wall_ms") or {}
                gap = row.get("min_inter_drone_distance_m")
                lines.append(
                    f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | {row['seed']} | {row['outcome']} | "
                    f"`{row['reason']}` | {row['arrived']}/{row['n_drones']} | "
                    f"{row['pillar_collision']} | {row['ground_collision']} | {row['inter_drone_collision']} | "
                    f"{_fmt(row.get('avg_speed_mps'))} | {_fmt(wall.get('mean'), 2)} | {_fmt(wall.get('p95'), 2)} | "
                    f"{_fmt(row.get('min_clearance_m'))} | {_fmt(gap)} | {int(row['corridor_empty_hover'])} | "
                    f"{float(row['sim_duration_s']):.2f} |"
                )
    return "\n".join(lines)


def _hashes(rows: list[dict], scene: str, arm: str) -> dict[int, str]:
    group = sorted(_group(rows, scene, arm), key=lambda r: int(r["seed"]))
    return {int(r["seed"]): str(r.get("positions_sha256")) for r in group}


def _hash_notes(rows: list[dict]) -> list[str]:
    notes = []
    for scene in SCENE_ORDER:
        maps = {arm: _hashes(rows, scene, arm) for arm in ARMS}
        if any(len(maps[arm]) != len(SEEDS) for arm in ARMS):
            continue
        label = SCENE_LABEL[scene]
        off, base, geom, multi = (maps[arm] for arm in ARMS)
        changed = [s for s in SEEDS if base.get(s) != off.get(s)]
        if base == geom == multi:
            if not changed:
                notes.append(f"{label}四档五个种子的位置哈希都相同。")
            else:
                seeds = "、".join(str(s) for s in changed)
                notes.append(
                    f"{label}的 base、geom、geom_multi 五个种子位置哈希相同。"
                    f"相对全关，只有 seed {seeds} 的轨迹不同。"
                )
        elif geom == multi:
            notes.append(f"{label}的 geom 与 geom_multi 五个种子位置哈希相同，并且与全关、base 都不同。")
        else:
            notes.append(f"{label}四档的位置哈希不完全相同，以表里的数为准。")
    return notes


def _runner_deviation_sentence(rows: list[dict]) -> str:
    bits = []
    for scene in SCENE_ORDER:
        for arm in ARMS:
            group = _group(rows, scene, arm)
            if not group:
                continue
            flags = sorted({bool(r["runner_upstream_deviation"]) for r in group})
            shown = "、".join("true" if flag else "false" for flag in flags)
            bits.append(f"{SCENE_LABEL[scene]} / {arm} = {shown}")
    return "运行器字段 upstream_deviation：" + "；".join(bits) + "。"


def render_placement(placed: list[dict]) -> str:
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
    return "\n".join(lines)


def render_markdown(payload: dict) -> str:
    rows = payload["rows"]
    placed = payload.get("swap6") or []
    commit = payload.get("commit") or ""
    lines = [
        "# 票 E-C6：六机密度场 0.14，只报数",
        "",
        "只报数。report-only。没有改 `yopo_l0/` 的默认、`core/`、已冻门、`scenes/`、"
        "`docs/tickets.md`、README 和 `scripts/regress.sh`。"
        "`safe_radius` 仍是 0.05 m，`empty_brake` 仍是 false，`track_clearance` 仍是 false，"
        "机间过滤仍是关。打开 E-A1、`empty_brake` 或 `track_clearance` 的臂标「偏离上游」。"
        "0.08 m 是假设，用来盖 [E-C5](e-c5-corridor-radius.md) 记下的 0.058–0.073 m 跟踪差。"
        "未冻结，不进 `scenes/suite.txt`，不设 gates，不改已冻门。",
        "",
        f"数字在 [e-c6-six-density-runs.json](e-c6-six-density-runs.json)。跑数 commit 是 `{commit}`。"
        "这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。",
        "",
        "## 口径",
        "",
        AVG_SPEED_DEF
        + " 汇总表里的平均速度是该档各次 `avg_speed_mps` 的算术平均。",
        "",
        "`plan_wall_ms` 是运行器记下的网络前向墙钟，不含解码和打分之后的时间。"
        "汇总表的均值 / p95 是各次运行自己的 mean / p95 再取算术平均，单位毫秒。"
        "它是这台机器上的墙钟，不进门。",
        "",
        "到达按架次。seeds 0–4、每场景 6 架，一档的分母是 30。"
        "撞柱、撞地、撞箱、机间碰撞按运行器，仿真在第一次碰撞停下，所以一次运行最多记 1。"
        "撞柱是 `TRUTH_POSITION` 且碰撞时刻最近的障碍是圆柱。撞地是最近的为地面。"
        "撞箱是最近的为盒子（林带的墙或枝）。机间是 `INTER_DRONE`。",
        "",
        "最小净空 = `min_obstacle_distance_m` − 0.35 m，是机体中心到最近表面的距离再减去机体半径。"
        "最小机间是中心距。",
        "",
        "`CORRIDOR_EMPTY_HOVER` 是 `plan_codes` 里这个码的次数。"
        "014 对向在 `empty_brake` 打开时，这个码就是从参考头刹停的规划次数；全关时它是上游原地悬停。"
        "六机林带里它是走廊为空的次数，随后的 `BRAKE_ONLY` / `CONFLICT_BRAKE` 不计入这一列。",
        "",
        "## 结论",
        "",
        _runner_deviation_sentence(rows),
        "",
        "六机林带是 crossing。CrossingFleet 只读 E-A1，不读 `empty_brake` 和 `track_clearance`，"
        "所以运行器的 `upstream_deviation` 在四档都保持 false（半径仍是 0.05 m）。"
        "014 六机对向是 goto，base / geom / geom_multi 的运行器标记为 true。臂上的「偏离上游」看开关有没有打开，不看这个运行器标记。",
        "",
        "穿林三档偏离臂的 E-A1 都是阈值 1.0 m、惩罚 0。"
        "`empty_brake` 和 `track_clearance` 写在进程内卡上，CrossingFleet 不读。"
        "`plan_wall_ms` 仍是各次前向的墙钟，不要求逐毫秒相同。",
        "",
        *(_hash_notes(rows)),
        "",
        render_summary(rows),
        "",
        "## 场景和臂",
        "",
        "| 场景 | 来源 | 任务 | 机数 | 种子 |",
        "|---|---|---|---:|---|",
        "| 六机林带 | `scenes/crossing-6-forest.json` | crossing | 6 | 0–4 |",
        "| 014 六机对向 | 进程内存，几何 `scenes/ego-density-014.json` | goto，40 s | 6 | 0–4 |",
        "",
        f"几何 `scenes/{GEOM_NAME}` 的 sha256 是 `{GEOM_SHA256}`。柱子 56 根，目标密度 0.14 个/m²。",
        "六机对向的名义圆半径 8.0 m，高度 1.5 m，方位间隔 60°，目标是起点的对径。"
        "轴距低于 0.70 m 时沿射线推到 20 m 正方形外，规则与 [E-C4](e-c4-density-l0.md) 的三机对向相同。",
        "",
        render_placement(placed),
        "",
        "| 臂 | 进程里多出来的 | 偏离上游 | 谁真的读这些开关 |",
        "|---|---|---|---|",
        "| 默认上游 + 影子盾 | 没有。不写 `policy_params`。影子盾开，只记录 | 否 | — |",
        "| E-A1 1.0 m + empty_brake（偏离上游） | `inter_drone_filter`，阈值 1.0 m，惩罚 0；`empty_brake=true` | 偏离上游 | 林带只读 E-A1。对向两样都读 |",
        "| base + track geom 0.08 m（偏离上游） | 再加上 `track_clearance=true`，`mode=geom`，`track_margin_m=0.08` | 偏离上游 | 林带仍只读 E-A1。对向三样都读 |",
        "| base + track geom_multi 0.08 m（偏离上游） | `mode=geom_multi`。只在本帧有邻机时做几何门 | 偏离上游 | 林带仍只读 E-A1。对向三样都读 |",
        "",
        "`safe_radius` 四档都不覆盖，运行器缺省 0.05 m。参数不写进场景文件。",
        "",
        "## 分种子",
        "",
        render_seeds(rows),
        "",
        "## 复现",
        "",
        "```",
        ".venv/bin/python scripts/e-c6-six-density.py --partial --jobs 4",
        "```",
        "",
        "`--partial` 从 `out/e-c6-six-density/partial/` 续跑已经完成的种子。缺省也续跑。`--no-resume` 忽略已有 partial。",
        "",
    ]
    return "\n".join(lines)


def _disk_snapshot() -> dict[str, bytes]:
    rels = [
        CROSSING_REL,
        f"scenes/{GEOM_NAME}",
        "scenes/suite.txt",
        "docs/tickets.md",
        "README.md",
        "scripts/regress.sh",
    ]
    return {rel: (ROOT / rel).read_bytes() for rel in rels}


def _check() -> int:
    from yopo_l0.policy import UPSTREAM_SAFE_RADIUS_M as radius
    from yopo_l0.policy import read_empty_brake, read_track_clearance
    from yopo_l0.scene import load_card

    blob = (ROOT / "scenes" / GEOM_NAME).read_bytes()
    got = hashlib.sha256(blob).hexdigest()
    if got != GEOM_SHA256:
        raise SystemExit(f"{GEOM_NAME} sha256 变了：{got}")
    if abs(float(radius) - 0.05) > 1e-12 or read_track_clearance({}) or read_empty_brake({}):
        raise SystemExit("默认开关被改了")
    before = _disk_snapshot()
    off_cross = build_card("crossing-6", "off")
    if off_cross.hash != load_card(ROOT / CROSSING_REL).hash:
        raise SystemExit("六机林带全关臂哈希与场景文件不一致")
    if len(off_cross.card["drones"]) != 6:
        raise SystemExit("六机林带不是 6 架")
    placed = place_swap6(density_centers())
    if len(placed) != 6:
        raise SystemExit("六机对向不是 6 架")
    for d in placed:
        if d["axis_distance_start"] < AXIS_CLEARANCE_M - 1e-9:
            raise SystemExit(f"{d['id']} 起点轴距不够")
        if d["axis_distance_goal"] < AXIS_CLEARANCE_M - 1e-9:
            raise SystemExit(f"{d['id']} 目标轴距不够")
    for scene in SCENE_ORDER:
        off = build_card(scene, "off")
        base = build_card(scene, "base")
        geom = build_card(scene, "geom")
        multi = build_card(scene, "geom_multi")
        if "policy_params" in off.card:
            raise SystemExit(f"{scene} 全关臂写了 policy_params")
        for card, mode in ((geom, "geom"), (multi, "geom_multi")):
            up = card.card["policy_params"]["upstream"]
            if up.get("track_clearance") is not True or up.get("track_clearance_mode") != mode:
                raise SystemExit(f"{scene} {mode} 没写进进程内卡")
            if abs(float(up["track_margin_m"]) - MARGIN_M) > 1e-12:
                raise SystemExit(f"{scene} {mode} margin 不是 0.08")
            if up.get("empty_brake") is not True or up.get("inter_drone_filter") is not True:
                raise SystemExit(f"{scene} {mode} 没有 base 的 E-A1 和 empty_brake")
            if abs(float(up["inter_drone_threshold_m"]) - A1_THRESHOLD_M) > 1e-12:
                raise SystemExit(f"{scene} E-A1 阈值不是 1.0 m")
            if "safe_radius" in up:
                raise SystemExit(f"{scene} 把 safe_radius 写进了进程内场景卡")
        up = base.card["policy_params"]["upstream"]
        if "track_clearance" in up or "safe_radius" in up:
            raise SystemExit(f"{scene} base 臂多写了 track_clearance 或 safe_radius")
        if up.get("empty_brake") is not True:
            raise SystemExit(f"{scene} base 没有 empty_brake")
    after = _disk_snapshot()
    if after != before:
        raise SystemExit("检查场景卡时改写了文件")
    suite = (ROOT / "scenes" / "suite.txt").read_text(encoding="utf-8")
    if "e-c6" in suite or "ego-density-014-swap6" in suite:
        raise SystemExit("scenes/suite.txt 被写进了本票的卡")
    if (ROOT / "scenes" / "ego-density-014-swap6.json").exists():
        raise SystemExit("六机对向被写进了 scenes/")
    print("e-c6 check ok")
    return 0


def _head_commit() -> str:
    import subprocess

    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scenes", default=",".join(SCENE_ORDER))
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--out", default="docs/research/e-c6-six-density-runs.json")
    ap.add_argument("--md", default="docs/research/e-c6-six-density.md")
    ap.add_argument("--partial", action="store_true", help="从 partial 断点续跑（缺省也续跑）")
    ap.add_argument("--no-resume", action="store_true", help="忽略已有 partial，重跑")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)
    if args.check:
        return _check()
    if args.jobs not in (1, 2, 3, 4):
        raise SystemExit("jobs 只能是 1 到 4")
    scenes = [s for s in args.scenes.split(",") if s]
    arms = [s for s in args.arms.split(",") if s]
    seeds = [int(s) for s in args.seeds.split(",") if s != ""]
    unknown = [s for s in scenes if s not in SCENE_ORDER] + [s for s in arms if s not in ARMS]
    if unknown:
        raise SystemExit(f"未知场景或臂：{unknown}")
    _check()
    resume = bool(args.partial) or not args.no_resume
    specs = job_specs(scenes, arms, seeds, resume=resume)
    print(f"e-c6 start n={len(specs)} jobs={args.jobs} resume={resume}", flush=True)
    rows = _run_group(specs, int(args.jobs))
    rows.sort(key=lambda r: (SCENE_ORDER.index(r["scene"]), ARMS.index(r["arm"]), int(r["seed"])))
    payload = {
        "disclaimer": DISCLAIMER,
        "commit": _head_commit(),
        "report_only": True,
        "frozen": False,
        "scenes": scenes,
        "arms": arms,
        "seeds": seeds,
        "jobs": int(args.jobs),
        "margin_m": MARGIN_M,
        "body_radius_m": BODY_RADIUS_M,
        "a1_threshold_m": A1_THRESHOLD_M,
        "valid_speed_eps_mps": VALID_SPEED_EPS_MPS,
        "avg_speed_def": AVG_SPEED_DEF,
        "geom_sha256": GEOM_SHA256,
        "swap6_name": SWAP6_NAME,
        "swap6": swap6_placement(),
        "rows": rows,
        "note": DISCLAIMER,
    }
    dest = Path(args.out)
    if not dest.is_absolute():
        dest = ROOT / dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    md_path = Path(args.md)
    if not md_path.is_absolute():
        md_path = ROOT / md_path
    md_path.write_text(render_markdown(payload), encoding="utf-8")
    print(render_summary(rows))
    print(f"写入 {dest}")
    print(f"写入 {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
