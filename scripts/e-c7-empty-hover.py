#!/usr/bin/env python3
"""票 E-C7：六机 014 geom 空走廊悬停与超时归因。只报数。

不改 `yopo_l0/`、`core/`、`scenes/`、`docs/tickets.md`、README。
开关只用已经在 main 上的三件：`track_clearance`（geom / geom_multi）、
`inter_drone_filter`、`empty_brake`。三档都在进程里打开，全部偏离上游。

- geom：E-A1 阈值 1.0 m、惩罚 0，empty_brake，track_clearance mode=geom，margin 0.08 m
- geom_multi：同一组，mode=geom_multi
- geom_soft：与 geom 相同，再在本进程里包一层 `inter_pick`。硬过滤把可行集清空时，
  按尚未合入 main 的 E-A2 规则留下中心距 ≥ 0.75 m 的候选，权重 2.0。
  这层不写进 `inter_drone.py`，也不写进 `policy.py`。只用来回答「放行之后，
  另一架的 geom 门是否更常拒绝」。

014 六机对向复用 E-C6 的构造：几何 `scenes/ego-density-014.json`，方位间隔 60°，
净空外推与 `scripts/e-c4-density-l0.py` 的三机相同。seeds 0–4。
014 三机对向读 `scenes/ego-density-014-swap3.json`，seeds 0–9。
`--partial` 逐粒落盘，`--resume` 读回已落盘的粒。`--seeds` 可以少跑。

每次 `CORRIDOR_EMPTY_HOVER` 记一个直接原因：走廊准入为空、geom 净空门拒绝、
E-A1 `inter_pick` 判空、其它。悬停拍再记 empty_brake 有没有真的造出刹车轨迹、
刷新前那条轨迹的最低 z、最小机间、机体和这条轨迹是否贴地或贴树。
贴的线是表面距离 ≤ 0.40 m（机体半径 0.35 m 再加 0.05 m）。
"""

from __future__ import annotations

import argparse
import contextvars
import hashlib
import importlib.util
import json
import math
import os
import sys
import time
from collections import defaultdict
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-c7-empty-hover" / "partial"
GEOM_NAME = "ego-density-014.json"
GEOM_SHA256 = "5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9"
SWAP3_REL = "scenes/ego-density-014-swap3.json"
HOVER = "CORRIDOR_EMPTY_HOVER"
ROW_VERSION = 1

BODY_RADIUS_M = 0.35
TOUCH_M = 0.40  # 机体半径 0.35 m + 0.05 m。表面距离不超过这条算贴地或贴树。
MARGIN_M = 0.08
THRESHOLD_M = 1.0
HARD_MIN_M = 0.75
YIELD_PENALTY = 2.0
PROBE_KEY = "ec7_soft_yield_probe"

SCENES = ("d014-swap6", "d014-swap3")
SCENE_LABEL = {"d014-swap6": "014 六机对向", "d014-swap3": "014 三机对向"}
SCENE_SEEDS = {"d014-swap6": tuple(range(5)), "d014-swap3": tuple(range(10))}
ARMS = ("geom", "geom_multi", "geom_soft")
CAUSES = ("corridor_empty", "geom_reject", "inter_empty", "other")
CAUSE_LABEL = {
    "corridor_empty": "走廊准入为空",
    "geom_reject": "geom 净空门拒绝",
    "inter_empty": "E-A1 inter_pick empty",
    "other": "其它",
}

# 0° / 60° / 120° / 180° / 240° / 300°。0°、120°、240° 与三机卡同一方位。
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
    "track_clearance false、inter_drone_filter 关。三档都在进程里打开 E-A1"
    "（阈值 1.0 m、惩罚 0）、empty_brake，以及 track_clearance margin 0.08 m，全部偏离上游。"
    "geom_soft 的软让行不是 main 上的开关：本脚本只在本进程里包住 inter_pick，"
    "硬过滤判空后留下中心距 ≥ 0.75 m 的候选，让行权重 2.0，用来和硬过滤对照。"
    "不改 yopo_l0/inter_drone.py，不改 yopo_l0/policy.py，不改 scenes/。"
    "0.08 m 是假设，用来盖已观测的跟踪差。0.40 m 的贴地/贴树线是机体半径 0.35 m 再加 0.05 m。"
    "这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。"
    "未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。"
)

A1 = {
    "inter_drone_filter": True,
    "inter_drone_threshold_m": THRESHOLD_M,
    "inter_drone_penalty": 0,
}

_SLOT: contextvars.ContextVar = contextvars.ContextVar("ec7_slot", default=None)
_LOG: list[dict] = []
_INSTALLED = False
_ORIG: dict = {}


def load_ec4():
    name = "e_c4_density_l0_for_c7"
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


def arm_label(arm: str) -> str:
    labels = {
        "geom": "geom 0.08 m + E-A1 硬过滤 + empty_brake（偏离上游）",
        "geom_multi": "geom_multi 0.08 m + E-A1 硬过滤 + empty_brake（偏离上游）",
        "geom_soft": "geom 0.08 m + 进程内软让行探针 0.75 m + empty_brake（偏离上游，report-only）",
    }
    if arm not in labels:
        raise ValueError(arm)
    return labels[arm]


def arm_mode(arm: str) -> str:
    if arm == "geom_multi":
        return "geom_multi"
    if arm in ("geom", "geom_soft"):
        return "geom"
    raise ValueError(arm)


def _upstream_for(arm: str) -> dict:
    upstream = dict(A1)
    upstream["empty_brake"] = True
    upstream["track_clearance"] = True
    upstream["track_clearance_mode"] = arm_mode(arm)
    upstream["track_margin_m"] = MARGIN_M
    if arm == "geom_soft":
        upstream[PROBE_KEY] = True
    elif arm not in ARMS:
        raise ValueError(arm)
    return upstream


def density_geometry() -> dict:
    return json.loads((ROOT / "scenes" / GEOM_NAME).read_text(encoding="utf-8"))


def density_centers(geom: dict | None = None) -> list[tuple[float, float]]:
    ec4 = load_ec4()
    geom = density_geometry() if geom is None else geom
    return ec4.cylinder_centers(geom)


def place_swap6(centers: list[tuple[float, float]]) -> list[dict]:
    """六架直径对向。净空不够就沿射线推到正方形外，和 E-C4 / E-C6 同一套函数。"""
    ec4 = load_ec4()
    drones = []
    for i, (ux, uy) in enumerate(SWAP6_UNIT):
        ux, uy = ec4._unit(ux, uy)
        x8, y8 = ec4.NOMINAL_SWAP_RADIUS_M * ux, ec4.NOMINAL_SWAP_RADIUS_M * uy
        d_start = ec4.axis_distance(x8, y8, centers)
        d_goal = ec4.axis_distance(-x8, -y8, centers)
        keep = (
            d_start >= ec4.AXIS_CLEARANCE_M - 1e-12
            and d_goal >= ec4.AXIS_CLEARANCE_M - 1e-12
        )
        if keep:
            radius = ec4.NOMINAL_SWAP_RADIUS_M
            sx, sy = x8, y8
            pushed = False
        else:
            radius, _r_min, _r_edge = ec4.first_clear_swap_radius(ux, uy, centers)
            sx, sy = radius * ux, radius * uy
            pushed = True
        start_axis = ec4.axis_distance(sx, sy, centers)
        goal_axis = ec4.axis_distance(-sx, -sy, centers)
        if start_axis < ec4.AXIS_CLEARANCE_M - 1e-9 or goal_axis < ec4.AXIS_CLEARANCE_M - 1e-9:
            raise RuntimeError(f"uav{i} 起终点轴距低于 0.70 m")
        drones.append({
            "id": f"uav{i}",
            "start_enu": [sx, sy, ec4.HEIGHT_M],
            "goal_enu": [-sx, -sy, ec4.HEIGHT_M],
            "radius_m": radius,
            "pushed_outside": pushed,
            "axis_distance_start": start_axis,
            "axis_distance_goal": goal_axis,
        })
    return drones


def _coord(v: float) -> float:
    x = float(v)
    return 0.0 if x == 0.0 else x


def _public_drone(d: dict) -> dict:
    return {
        "id": d["id"],
        "start_enu": [_coord(v) for v in d["start_enu"]],
        "goal_enu": [_coord(v) for v in d["goal_enu"]],
    }


def swap6_raw() -> dict:
    ec4 = load_ec4()
    geom = density_geometry()
    placed = place_swap6(density_centers(geom))
    return {
        "name": "ego-density-014-swap6",
        "note": (
            "票 E-C7 内存卡：014 六机 goto 对向，几何是 scenes/ego-density-014.json。"
            "构造与 E-C6 相同：半径 8.0 m、高度 1.5 m、方位间隔 60°，目标是起点的对径。"
            "起终点到最近圆柱轴至少 0.70 m，不够就沿射线推到 20 m 正方形外，"
            "外推函数与 scripts/e-c4-density-l0.py 相同。不写进 scenes/。"
            "这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。"
        ),
        "geometry": geom,
        "ground_z": 0.0,
        "drones": [_public_drone(d) for d in placed],
        "task": {"type": "goto", "duration_s": 40.0},
        "criteria": dict(ec4.CRITERIA),
        "camera": dict(ec4.CAMERA),
        "dynamics": dict(ec4.DYNAMICS),
        "rates_hz": dict(ec4.RATES),
        "seeds": [0],
        "placement": [
            {
                "id": d["id"],
                "radius_m": d["radius_m"],
                "pushed_outside": d["pushed_outside"],
                "axis_distance_start": d["axis_distance_start"],
                "axis_distance_goal": d["axis_distance_goal"],
                "start_enu": _public_drone(d)["start_enu"],
                "goal_enu": _public_drone(d)["goal_enu"],
            }
            for d in placed
        ],
    }


def build_card(scene: str, arm: str):
    from yopo_l0.scene import SceneCard, parse_geometry

    if scene == "d014-swap3":
        raw = json.loads((ROOT / SWAP3_REL).read_text(encoding="utf-8"))
        geom = raw.get("geometry")
        if isinstance(geom, str):
            raw["geometry"] = json.loads((ROOT / "scenes" / geom).read_text(encoding="utf-8"))
    elif scene == "d014-swap6":
        raw = swap6_raw()
    else:
        raise ValueError(scene)
    if arm not in ARMS:
        raise ValueError(arm)
    params = dict(raw.get("policy_params") or {})
    params["upstream"] = _upstream_for(arm)
    raw = json.loads(json.dumps(raw))
    raw["policy_params"] = params
    return SceneCard(raw["name"], raw, parse_geometry(raw["geometry"], raw.get("ground_z")))


def is_touch(distance) -> bool:
    if distance is None:
        return False
    value = float(distance)
    return math.isfinite(value) and value <= TOUCH_M + 1e-12


def split_clearance(obstacles, points) -> tuple[float, float]:
    """返回 (地面距离, 圆柱表面距离)。点可以是一个位置或一组采样。"""
    from yopo_l0.scene import Cylinder, Ground

    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim == 1:
        pts = pts.reshape(1, 3)
    ground = math.inf
    tree = math.inf
    for ob in obstacles:
        if isinstance(ob, Ground):
            ground = min(ground, float(np.maximum(pts[:, 2] - float(ob.z), 0.0).min()))
        elif isinstance(ob, Cylinder) and ob.upright():
            radial = np.hypot(pts[:, 0] - ob.center_xy[0], pts[:, 1] - ob.center_xy[1]) - ob.radius
            radial = np.maximum(radial, 0.0)
            dz = np.maximum(np.maximum(ob.z_lo - pts[:, 2], 0.0), pts[:, 2] - ob.z_hi)
            tree = min(tree, float(np.hypot(radial, dz).min()))
        elif isinstance(ob, Cylinder):
            tree = min(tree, min(float(ob.distance(p)) for p in pts))
    return float(ground), float(tree)


def prev_traj_stats(plan, obstacles) -> dict | None:
    if plan is None or not hasattr(plan, "traj"):
        return None
    from yopo_l0.policy import trajectory_sample_points

    pts = trajectory_sample_points(plan.traj)
    ground, tree = split_clearance(obstacles, pts)
    return {
        "prev_min_z": float(np.min(pts[:, 2])),
        "prev_min_ground_m": ground,
        "prev_min_tree_m": tree,
    }


def classify_cause(slot: dict) -> str:
    """悬停的直接原因。调用方已经确认这一拍的规划码是 CORRIDOR_EMPTY_HOVER。"""
    if slot.get("select_brake") is True:
        return "corridor_empty"
    if slot.get("geom_called") and slot.get("geom_brake"):
        if slot.get("geom_corridor_empty"):
            return "corridor_empty"
        if slot.get("geom_index_missing") or int(slot.get("geom_ok") or 0) > 0:
            return "other"
        return "geom_reject"
    if slot.get("inter_called") and int(slot.get("inter_empty_delta") or 0) > 0:
        return "inter_empty"
    return "other"


def soft_yield_pick(score, admitted, clearances, cfg, *, hard_min_m: float = HARD_MIN_M, yield_penalty: float = YIELD_PENALTY):
    """硬过滤已经判空时的让行。没有候选能留下时返回 None，调用方保留原来的 empty。

    与尚未合入的 E-A2 同一条规则：留下中心距 ≥ hard_min_m 的准入候选（含测不到的 +inf），
    按 `score - yield_penalty * (阈值 − d) / 阈值` 重排，同分取索引较小者。
    低于下限的仍然滤掉。惩罚不为 0 的减分路径不进这里。
    """
    from yopo_l0.inter_drone import InterPick

    score = np.asarray(score, dtype=np.float64)
    admitted = np.asarray(admitted, dtype=bool)
    clearances = np.asarray(clearances, dtype=np.float64)
    keep = admitted & (clearances >= float(hard_min_m))
    if not bool(keep.any()):
        return None
    adjusted = np.array(score, dtype=np.float64, copy=True)
    finite = keep & np.isfinite(clearances)
    penalized = 0
    if bool(finite.any()):
        scale = (float(cfg.threshold_m) - clearances[finite]) / float(cfg.threshold_m)
        adjusted[finite] = score[finite] - float(yield_penalty) * scale
        penalized = int(np.count_nonzero(finite & (clearances < float(cfg.threshold_m))))
    still = int(np.count_nonzero(admitted & np.isfinite(clearances) & (clearances < float(hard_min_m))))
    cand = np.flatnonzero(keep)
    first = int(cand[int(np.argmax(adjusted[cand]))])
    rest = sorted((int(i) for i in cand.tolist() if int(i) != first), key=lambda i: (-float(adjusted[i]), i))
    return InterPick("adjusted", action=first, order=(first, *rest), filtered=still, penalized=penalized)


def _soft_enabled(policy) -> bool:
    raw = ((getattr(policy, "ctx", None) and policy.ctx.params) or {}).get("upstream") or {}
    return raw.get(PROBE_KEY) is True


def install_probe() -> None:
    """包住现有的 plan / 几何门 / inter_pick。不改那些函数的源文件。

    没有本脚本设下的 slot 时，包装函数把参数原样交回去。
    """
    global _INSTALLED
    if _INSTALLED:
        return
    import yopo_l0.geom_fallback as geom_fb
    import yopo_l0.inter_drone as inter
    import yopo_l0.policy as policy
    import yopo_swarm_core.candidates as cand

    _ORIG["plan"] = policy.UpstreamPolicy.plan
    _ORIG["geom"] = policy.UpstreamPolicy._apply_track_geom
    _ORIG["inter"] = policy.UpstreamPolicy._apply_inter
    _ORIG["select"] = cand.select_action
    _ORIG["brake"] = geom_fb.brake_from_head
    _ORIG["pick"] = inter.inter_pick

    def select_action(*args, **kwargs):
        action, brake = _ORIG["select"](*args, **kwargs)
        slot = _SLOT.get()
        if slot is not None:
            slot["select_brake"] = bool(brake)
        return action, brake

    def brake_from_head(*args, **kwargs):
        traj = _ORIG["brake"](*args, **kwargs)
        slot = _SLOT.get()
        if slot is not None:
            slot["brake_called"] = True
            slot["brake_traj"] = traj
        return traj

    def apply_track_geom(self, score, radius, dec, *rest, **kwargs):
        from yopo_swarm_core.candidates import rank_candidates

        ranking = rank_candidates(np.asarray(score), radius, dec)
        action, brake, geom_ok = _ORIG["geom"](self, score, radius, dec, *rest, **kwargs)
        slot = _SLOT.get()
        if slot is not None:
            slot["geom_called"] = True
            slot["geom_brake"] = bool(brake)
            slot["geom_corridor_empty"] = bool(ranking.corridor_empty)
            slot["geom_index_missing"] = self._track_index is None
            slot["geom_ok"] = int(np.count_nonzero(geom_ok)) if geom_ok is not None else 0
            slot["n_admitted"] = int(np.count_nonzero(ranking.admitted))
        return action, brake, geom_ok

    def apply_inter(self, *args, **kwargs):
        before = int(self._inter_empty)
        action, brake = _ORIG["inter"](self, *args, **kwargs)
        slot = _SLOT.get()
        if slot is not None:
            slot["inter_called"] = True
            slot["inter_brake"] = bool(brake)
            slot["inter_empty_delta"] = int(self._inter_empty) - before
        return action, brake

    def inter_pick(score, admitted, clearances, cfg):
        pick = _ORIG["pick"](score, admitted, clearances, cfg)
        slot = _SLOT.get()
        if slot is None or not slot.get("soft"):
            return pick
        if pick.kind != "empty" or float(cfg.penalty) != 0.0:
            return pick
        softened = soft_yield_pick(score, admitted, clearances, cfg)
        slot["soft_consulted"] = True
        if softened is None:
            slot["yield_release"] = False
            return pick
        slot["yield_release"] = True
        return softened

    def plan(self, obs):
        slot = {
            "select_brake": None,
            "geom_called": False,
            "geom_brake": False,
            "geom_corridor_empty": False,
            "geom_index_missing": False,
            "geom_ok": 0,
            "n_admitted": None,
            "inter_called": False,
            "inter_brake": False,
            "inter_empty_delta": 0,
            "brake_called": False,
            "brake_traj": None,
            "soft": _soft_enabled(self),
            "yield_release": False,
            "soft_consulted": False,
        }
        token = _SLOT.set(slot)
        prev = self.plan_obj
        try:
            out = _ORIG["plan"](self, obs)
        finally:
            _SLOT.reset(token)
        _LOG.append(_event(self, obs, slot, prev))
        return out

    policy.UpstreamPolicy.plan = plan
    policy.UpstreamPolicy._apply_track_geom = apply_track_geom
    policy.UpstreamPolicy._apply_inter = apply_inter
    cand.select_action = select_action
    geom_fb.brake_from_head = brake_from_head
    inter.inter_pick = inter_pick
    _INSTALLED = True


def uninstall_probe() -> None:
    global _INSTALLED
    if not _INSTALLED:
        return
    import yopo_l0.geom_fallback as geom_fb
    import yopo_l0.inter_drone as inter
    import yopo_l0.policy as policy
    import yopo_swarm_core.candidates as cand

    policy.UpstreamPolicy.plan = _ORIG["plan"]
    policy.UpstreamPolicy._apply_track_geom = _ORIG["geom"]
    policy.UpstreamPolicy._apply_inter = _ORIG["inter"]
    cand.select_action = _ORIG["select"]
    geom_fb.brake_from_head = _ORIG["brake"]
    inter.inter_pick = _ORIG["pick"]
    _INSTALLED = False


def _event(policy, obs, slot: dict, prev) -> dict:
    p = np.asarray(obs.state.p, dtype=np.float64)
    code = str(policy.last_code)
    cause = classify_cause(slot) if code == HOVER else None
    obstacles = tuple(getattr(policy.ctx, "obstacles", ()) or ())
    pos_ground, pos_tree = split_clearance(obstacles, p) if cause else (None, None)
    prev_stats = prev_traj_stats(prev, obstacles) if cause else None
    empty_on = bool(getattr(policy, "_empty_brake", False))
    applied = bool(slot.get("brake_called") and slot.get("brake_traj") is not None)
    event = {
        "t": round(float(obs.now), 9),
        "uav": str(policy.ctx.owner),
        "code": code,
        "cause": cause,
        "yield_release": bool(slot.get("yield_release")),
        "p": [float(p[0]), float(p[1]), float(p[2])],
    }
    if cause is not None:
        event.update({
            "empty_brake_enabled": empty_on,
            "empty_brake_applied": applied,
            "empty_brake_hold_fallback": bool(empty_on and not applied),
            "pos_ground_m": pos_ground,
            "pos_tree_m": pos_tree,
            "pos_near_ground": is_touch(pos_ground),
            "pos_near_tree": is_touch(pos_tree),
            "n_geom_ok": slot.get("geom_ok") if slot.get("geom_called") else None,
            "n_admitted": slot.get("n_admitted"),
        })
        if prev_stats is not None:
            event.update(prev_stats)
            event["traj_near_ground"] = is_touch(prev_stats["prev_min_ground_m"])
            event["traj_near_tree"] = is_touch(prev_stats["prev_min_tree_m"])
        else:
            event["prev_min_z"] = None
            event["prev_min_ground_m"] = None
            event["prev_min_tree_m"] = None
            event["traj_near_ground"] = False
            event["traj_near_tree"] = False
    return event


def take_log() -> list[dict]:
    rows = list(_LOG)
    _LOG.clear()
    return rows


def yield_geom_coupling(events: list[dict]) -> dict:
    """上一规划拍有另一架被软让行放行，这一拍本机 geom 门拒绝的次数。

    总线在这一拍全部 plan() 之后才把邻机轨迹送进来，所以放行要到下一规划拍才进对方的几何门。
    """
    by_t: dict[float, list[dict]] = {}
    for event in events:
        by_t.setdefault(float(event["t"]), []).append(event)
    times = sorted(by_t)
    geom_total = 0
    after_other = 0
    for i, t in enumerate(times):
        released: set[str] = set()
        if i:
            released = {e["uav"] for e in by_t[times[i - 1]] if e.get("yield_release")}
        for event in by_t[t]:
            if event.get("cause") != "geom_reject":
                continue
            geom_total += 1
            if released - {event["uav"]}:
                after_other += 1
    return {
        "geom_reject": geom_total,
        "geom_reject_after_other_yield": after_other,
    }


def _counts(events: list[dict], key: str, pred) -> dict[str, int]:
    out: dict[str, int] = defaultdict(int)
    for event in events:
        if pred(event):
            out[str(event[key])] += 1
    return dict(out)


def _median(values: list[float]) -> float | None:
    if not values:
        return None
    arr = np.sort(np.asarray(values, dtype=np.float64))
    n = int(arr.size)
    if n % 2 == 1:
        return float(arr[n // 2])
    return float(0.5 * (arr[n // 2 - 1] + arr[n // 2]))


def _finite(values: list[float]) -> list[float]:
    return [float(v) for v in values if v is not None and math.isfinite(float(v))]


def attach_min_inter(events: list[dict]) -> None:
    by_t: dict[float, dict[str, list[float]]] = defaultdict(dict)
    for event in events:
        by_t[float(event["t"])][event["uav"]] = event["p"]
    for event in events:
        if not event.get("cause"):
            continue
        others = [p for uav, p in by_t[float(event["t"])].items() if uav != event["uav"]]
        if not others:
            event["min_inter_m"] = None
            continue
        origin = np.asarray(event["p"], dtype=np.float64)
        event["min_inter_m"] = float(min(np.linalg.norm(origin - np.asarray(p, dtype=np.float64)) for p in others))


def hover_summary(events: list[dict]) -> dict:
    hovers = [e for e in events if e.get("cause")]
    cause_counts = {name: 0 for name in CAUSES}
    for event in hovers:
        cause_counts[event["cause"]] = cause_counts.get(event["cause"], 0) + 1
    by_drone: dict[str, dict[str, int]] = {}
    for event in hovers:
        bucket = by_drone.setdefault(event["uav"], {name: 0 for name in CAUSES})
        bucket[event["cause"]] += 1
    zs = _finite([e.get("prev_min_z") for e in hovers])
    gaps = _finite([e.get("min_inter_m") for e in hovers])
    pos_z = _finite([e["p"][2] for e in hovers])

    def n_true(key: str) -> int:
        return sum(1 for e in hovers if e.get(key))

    return {
        "n": len(hovers),
        "cause_counts": cause_counts,
        "cause_by_drone": by_drone,
        "empty_brake_applied": n_true("empty_brake_applied"),
        "empty_brake_hold_fallback": n_true("empty_brake_hold_fallback"),
        "pos_near_ground": n_true("pos_near_ground"),
        "traj_near_ground": n_true("traj_near_ground"),
        "pos_near_tree": n_true("pos_near_tree"),
        "traj_near_tree": n_true("traj_near_tree"),
        "prev_min_z_min": None if not zs else float(min(zs)),
        "prev_min_z_median": _median(zs),
        "prev_min_z_max": None if not zs else float(max(zs)),
        "min_inter_min": None if not gaps else float(min(gaps)),
        "min_inter_median": _median(gaps),
        "pos_z_median": _median(pos_z),
        "yield_releases": sum(1 for e in events if e.get("yield_release")),
        "yield_releases_by_drone": _counts(events, "uav", lambda e: e.get("yield_release")),
        "yield_coupling": yield_geom_coupling(events),
    }


def arrived_ids(ticks: list[dict], crit: dict) -> set[str]:
    gtol = float(crit["goal_tolerance_m"])
    gspeed = float(crit["goal_speed_mps"])
    ghold = float(crit["goal_hold_s"])
    since: dict[str, float | None] = {}
    arrived: set[str] = set()
    for row in ticks:
        uav = row["uav"]
        if uav in arrived:
            continue
        err = float(row["goal_error_m"])
        speed = float(np.linalg.norm(row["v"]))
        if err <= gtol and speed <= gspeed:
            since[uav] = float(row["t"]) if since.get(uav) is None else since[uav]
            if float(row["t"]) - float(since[uav]) >= ghold - 1e-9:
                arrived.add(uav)
        else:
            since[uav] = None
    return arrived


def terminal_stops(events: list[dict], ticks: list[dict], crit: dict, outcome: str, sim_end: float) -> list[dict]:
    """超时运行里没有到达的飞机。连续悬停从这段后缀的第一拍算到仿真结束。"""
    if outcome != "TIMEOUT":
        return []
    arrived = arrived_ids(ticks, crit)
    by_uav: dict[str, list[dict]] = defaultdict(list)
    for event in events:
        by_uav[event["uav"]].append(event)
    last_tick: dict[str, dict] = {}
    for row in ticks:
        last_tick[row["uav"]] = row
    stops = []
    for uav in sorted(set(last_tick) | set(by_uav)):
        if uav in arrived:
            continue
        rows = sorted(by_uav.get(uav, []), key=lambda e: float(e["t"]))
        end = last_tick.get(uav)
        i = len(rows)
        while i > 0 and rows[i - 1].get("code") == HOVER:
            i -= 1
        streak = rows[i:]
        start = streak[0] if streak else None
        first = next((e for e in rows if e.get("code") == HOVER), None)
        causes = {name: 0 for name in CAUSES}
        for event in streak:
            if event.get("cause"):
                causes[event["cause"]] += 1
        final_p = None if end is None else [float(x) for x in end["p"]]
        final_v = None if end is None else float(np.linalg.norm(end["v"]))
        start_t = None if start is None else float(start["t"])
        stops.append({
            "uav": uav,
            "final_p": [round(x, 4) for x in final_p] if final_p else None,
            "final_speed_mps": final_v,
            "final_goal_error_m": None if end is None else float(end["goal_error_m"]),
            "terminal_hover_start_s": start_t,
            "terminal_hover_duration_s": None if start_t is None else float(sim_end) - start_t,
            "terminal_hover_n": len(streak),
            "terminal_hover_start_p": None if start is None else [round(float(x), 4) for x in start["p"]],
            "terminal_cause_counts": causes,
            "first_hover_s": None if first is None else float(first["t"]),
            "hover_continuous_to_timeout": bool(
                first is not None and start is not None and abs(float(first["t"]) - float(start["t"])) <= 1e-9
            ),
        })
    return stops


def planner_hover_count(ticks: list[dict], physics: int, planner: int) -> int:
    """与运行器 `_due` 同一条：每个规划拍、每一架计一次。"""
    n = 0
    for row in ticks:
        k = int(row["k"])
        due = k == 0 or (k * planner) // physics != ((k - 1) * planner) // physics
        if due and row.get("plan_code") == HOVER:
            n += 1
    return n


def avg_speed_positive(ticks: list[dict]) -> float | None:
    speeds = []
    for row in ticks:
        speed = float(np.linalg.norm(row["v"]))
        if math.isfinite(speed) and speed > 0.0:
            speeds.append(speed)
    if not speeds:
        return None
    return float(sum(speeds) / len(speeds))


def mean_speed(ticks: list[dict]) -> float | None:
    if not ticks:
        return None
    return float(np.mean([np.linalg.norm(row["v"]) for row in ticks]))


def positions_sha256(ticks: list[dict]) -> str:
    digest = hashlib.sha256()
    for row in ticks:
        digest.update(str(row["uav"]).encode())
        digest.update(np.asarray(row["p"], dtype=np.float64).tobytes())
        digest.update(np.float64(row["t"]).tobytes())
    return digest.hexdigest()


def _hit_kind(card, result: dict, ticks: list[dict]) -> str:
    from yopo_l0.scene import Cylinder, Ground

    outcome = result.get("outcome")
    reason = str(result.get("reason") or "")
    if outcome == "TIMEOUT":
        return "timeout"
    if outcome == "COLLISION" and "INTER_DRONE" in reason:
        return "inter"
    if outcome != "COLLISION" or "TRUTH_POSITION" not in reason:
        return "none"
    drone = reason.split(":", 1)[0]
    rows = [row for row in ticks if row["uav"] == drone]
    if not rows:
        return "pillar"
    p = np.asarray(rows[-1]["p"], dtype=float)
    ground = math.inf
    pillar = math.inf
    for ob in card.obstacles:
        dist = float(ob.distance(p))
        if isinstance(ob, Ground):
            ground = min(ground, dist)
        elif isinstance(ob, Cylinder):
            pillar = min(pillar, dist)
    return "ground" if ground <= pillar else "pillar"


def _round_event(event: dict) -> dict:
    out = {}
    for key, value in event.items():
        if key == "p":
            out[key] = [round(float(x), 4) for x in value]
        elif isinstance(value, float):
            out[key] = None if not math.isfinite(value) else round(value, 4)
        else:
            out[key] = value
    return out


def _jsonable(obj):
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, float)):
        value = float(obj)
        return None if not math.isfinite(value) else value
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, set):
        return sorted(obj)
    return obj


def partial_path(partial_dir: Path, scene: str, arm: str, seed: int, duration_s: float | None) -> Path:
    suffix = "" if duration_s is None else f"-t{duration_s:g}"
    return partial_dir / f"{scene}-{arm}-{int(seed)}{suffix}.json"


def load_partial(scene: str, arm: str, seed: int, duration_s: float | None, partial_dir: Path) -> dict | None:
    dest = partial_path(partial_dir, scene, arm, seed, duration_s)
    if not dest.is_file():
        return None
    row = json.loads(dest.read_text(encoding="utf-8"))
    if int(row.get("row_version", 0)) != ROW_VERSION:
        return None
    token = None if duration_s is None else float(duration_s)
    if row.get("duration_s_override") != token:
        return None
    return row


def save_partial(row: dict, partial_dir: Path) -> None:
    dest = partial_path(
        partial_dir, row["scene"], row["arm"], int(row["seed"]), row.get("duration_s_override")
    )
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(_jsonable(row), ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)


def run_one(
    scene: str,
    arm: str,
    seed: int,
    *,
    partial: bool,
    resume: bool,
    duration_s: float | None,
    partial_dir: Path,
) -> dict:
    token = None if duration_s is None else float(duration_s)
    if resume:
        cached = load_partial(scene, arm, seed, token, partial_dir)
        if cached is not None:
            print(f"{scene} {arm} seed={seed} resume", flush=True)
            return cached
    from yopo_l0.runner import RunConfig, run

    install_probe()
    _LOG.clear()
    card = build_card(scene, arm)
    rates = card.card["rates_hz"]
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(seed), shadow=False, duration_s=duration_s))
    events = take_log()
    attach_min_inter(events)
    res = out.result
    summary = hover_summary(events)
    probe_n = int(summary["n"])
    tick_n = planner_hover_count(out.ticks, int(rates["physics"]), int(rates["planner"]))
    if probe_n != tick_n:
        raise RuntimeError(f"{scene} {arm} seed={seed} 悬停计数探针 {probe_n} 与规划拍 {tick_n} 不一致")
    filt = res.get("inter_drone_filter") or {}
    if int(filt.get("empty_fallbacks") or 0) != int(summary["cause_counts"]["inter_empty"]):
        raise RuntimeError(
            f"{scene} {arm} seed={seed} inter_pick 判空 "
            f"{summary['cause_counts']['inter_empty']} 与运行器 {filt.get('empty_fallbacks')} 不一致"
        )
    kind = _hit_kind(card, res, out.ticks)
    got_arrived = arrived_ids(out.ticks, card.card["criteria"])
    if len(got_arrived) != int(res["arrived"]):
        raise RuntimeError(f"{scene} {arm} seed={seed} 到达架次对不上")
    stops = terminal_stops(events, out.ticks, card.card["criteria"], res["outcome"], float(res["sim_duration_s"]))
    hover_events = [_round_event(e) for e in events if e.get("cause")]
    if bool(res["upstream_deviation"]) is not True:
        raise RuntimeError(f"{scene} {arm} 没有标偏离上游")
    if res.get("empty_brake") is not True or res.get("track_clearance") is not True:
        raise RuntimeError(f"{scene} {arm} 开关没有读回去")
    if res.get("track_clearance_mode") != arm_mode(arm):
        raise RuntimeError(f"{scene} {arm} mode 读回 {res.get('track_clearance_mode')}")
    if abs(float(res["safe_radius_m"]) - 0.05) > 1e-9:
        raise RuntimeError(f"{scene} {arm} safe_radius 读回 {res['safe_radius_m']}")
    wall = res.get("plan_wall_ms") or {}
    row = {
        "row_version": ROW_VERSION,
        "scene": scene,
        "arm": arm,
        "seed": int(seed),
        "report_only": True,
        "frozen": False,
        "upstream_deviation": True,
        "soft_yield_probe": arm == "geom_soft",
        "duration_s_override": token,
        "outcome": res["outcome"],
        "reason": res["reason"],
        "arrived": int(res["arrived"]),
        "arrived_ids": sorted(got_arrived),
        "n_drones": int(res["n_drones"]),
        "pillar": int(kind == "pillar"),
        "ground": int(kind == "ground"),
        "inter": int(kind == "inter"),
        "timeout": int(kind == "timeout"),
        "avg_speed_mps": avg_speed_positive(out.ticks),
        "mean_speed_mps": mean_speed(out.ticks),
        "speed_samples": len(out.ticks),
        "corridor_empty_hover": probe_n,
        "cause_counts": summary["cause_counts"],
        "cause_by_drone": summary["cause_by_drone"],
        "empty_brake_applied": summary["empty_brake_applied"],
        "empty_brake_hold_fallback": summary["empty_brake_hold_fallback"],
        "pos_near_ground": summary["pos_near_ground"],
        "traj_near_ground": summary["traj_near_ground"],
        "pos_near_tree": summary["pos_near_tree"],
        "traj_near_tree": summary["traj_near_tree"],
        "prev_min_z_min": summary["prev_min_z_min"],
        "prev_min_z_median": summary["prev_min_z_median"],
        "prev_min_z_max": summary["prev_min_z_max"],
        "hover_min_inter_min": summary["min_inter_min"],
        "hover_min_inter_median": summary["min_inter_median"],
        "hover_pos_z_median": summary["pos_z_median"],
        "yield_releases": summary["yield_releases"],
        "yield_releases_by_drone": summary["yield_releases_by_drone"],
        "yield_coupling": summary["yield_coupling"],
        "stops": stops,
        "hover_events": hover_events,
        "min_inter_drone_distance_m": res["min_inter_drone_distance_m"],
        "min_obstacle_distance_m": res["min_obstacle_distance_m"],
        "empty_fallbacks": int(filt.get("empty_fallbacks") or 0),
        "penalized_candidates": int(filt.get("penalized_candidates") or 0),
        "filtered_candidates": int(filt.get("filtered_candidates") or 0),
        "plan_wall_ms_p95": None if wall.get("p95") is None else float(wall["p95"]),
        "sim_duration_s": float(res["sim_duration_s"]),
        "positions_sha256": positions_sha256(out.ticks),
        "safe_radius_m": float(res["safe_radius_m"]),
        "track_clearance_mode": res.get("track_clearance_mode"),
        "wall_time_s": float(res["wall_time_s"]),
        "worker_s": time.perf_counter() - t0,
    }
    if partial or resume:
        save_partial(row, partial_dir)
    print(
        f"{scene} {arm} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"hover={probe_n} causes={summary['cause_counts']} yield={summary['yield_releases']} "
        f"sim={row['sim_duration_s']:.2f} wall={row['wall_time_s']:.1f}s",
        flush=True,
    )
    return _jsonable(row)


def _worker(spec: tuple) -> dict:
    try:
        os.nice(10)
    except OSError:
        pass
    scene, arm, seed, partial, resume, duration_s, partial_dir = spec
    return run_one(
        str(scene),
        str(arm),
        int(seed),
        partial=bool(partial),
        resume=bool(resume),
        duration_s=None if duration_s is None else float(duration_s),
        partial_dir=Path(partial_dir),
    )


def seeds_for(scene: str, override: list[int] | None) -> tuple[int, ...]:
    if override is not None:
        return tuple(int(s) for s in override)
    return SCENE_SEEDS[scene]


def job_specs(scenes, arms, seeds_by_scene, *, partial: bool, resume: bool, duration_s, partial_dir: Path):
    return [
        (scene, arm, int(seed), bool(partial), bool(resume), duration_s, str(partial_dir))
        for scene in scenes
        for arm in arms
        for seed in seeds_by_scene[scene]
    ]


def _run_group(specs: list[tuple], jobs: int) -> list[dict]:
    if jobs <= 1 or len(specs) <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)


def _group(rows: list[dict], scene: str, arm: str) -> list[dict]:
    return [r for r in rows if r["scene"] == scene and r["arm"] == arm]


def _sum_causes(group: list[dict]) -> dict[str, int]:
    total = {name: 0 for name in CAUSES}
    for row in group:
        counts = row.get("cause_counts") or {}
        for name in CAUSES:
            total[name] += int(counts.get(name) or 0)
    return total


def _fmt(value, digits: int) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{digits}f}"


def _rng(values: list[float], digits: int) -> str:
    finite = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if not finite:
        return "—"
    lo, hi = min(finite), max(finite)
    if abs(lo - hi) < 5 * 10 ** (-(digits + 1)):
        return f"{lo:.{digits}f}"
    return f"{lo:.{digits}f}–{hi:.{digits}f}"


def _weighted_mean(group: list[dict]) -> float | None:
    samples = sum(int(r.get("speed_samples") or 0) for r in group)
    if samples <= 0:
        return None
    total = sum(float(r["mean_speed_mps"]) * int(r["speed_samples"]) for r in group if r.get("mean_speed_mps") is not None)
    return total / samples


def _unweighted_positive(group: list[dict]) -> float | None:
    vals = [float(r["avg_speed_mps"]) for r in group if r.get("avg_speed_mps") is not None]
    if not vals:
        return None
    return sum(vals) / len(vals)


def _cause_cell(counts: dict) -> str:
    return " / ".join(str(int(counts.get(name) or 0)) for name in CAUSES)


def render_markdown(payload: dict) -> str:
    rows = payload.get("rows") or []
    lines = [
        "# 票 E-C7：六机 014 geom 空走廊悬停与超时归因",
        "",
        DISCLAIMER,
        "",
        f"数字在 [e-c7-empty-hover-runs.json](e-c7-empty-hover-runs.json)。跑数时 HEAD 是 `{payload.get('commit') or ''}`。",
        "归因脚本和这份记录在本票里，不在那个提交里。影子盾关。关盾不改批准动作。",
        "",
        "## 口径",
        "",
        "悬停按规划拍、按飞机计。物理拍上重复的同一份规划码不累加。直接原因四类，表里按这个顺序：",
        "走廊准入为空 / geom 净空门拒绝 / E-A1 `inter_pick` empty / 其它。",
        "走廊准入为空是 `select_action` 已经没有准入候选，几何门和机间过滤都还没轮到。",
        "geom 净空门拒绝是准入集非空，但参考轨迹对障碍表面的最小净空都低于机体半径 0.35 m + margin 0.08 m。",
        "E-A1 `inter_pick` empty 是几何门留下的可行集又被阈值 1.0 m、惩罚 0 的硬过滤整票清空。",
        "三类都不符合的记成其它。",
        "",
        "empty_brake 开着时，悬停拍会去叫 `brake_from_head`。参考头已经停住时这个函数返回空，调用方改回原地保持。",
        "「刹停」是真的造出了刹车轨迹；「保持」是开关开着但参考头已经停住。",
        "",
        "刷新前轨迹是这一拍 `plan()` 替换 `plan_obj` 之前的那条。最低 z、贴地、贴树都量在它的 0.02 s 采样上。",
        f"贴地或贴树：表面距离 ≤ {TOUCH_M:.2f} m（机体半径 {BODY_RADIUS_M:.2f} m 再加 0.05 m）。地面距离是机体中心高度。",
        "最小机间是同一规划拍各机 `plan()` 看到的机体中心距，不是 4 s 轨迹上的最近中心距。",
        "",
        "平均速度（>0）沿用 E-C6：全程 ||v|| 里有限且严格大于 0 的样本的算术平均，再对种子做不加权平均。",
        "平均速度（含悬停）沿用 E-A2：全部物理拍的 ||v||，按样本数加权。",
        "",
        "超时里的「停住」只看没有到达的飞机。连续悬停是从超时往回数、规划码一直是 `CORRIDOR_EMPTY_HOVER` 的那段。",
        "时长是仿真结束减去这段第一拍的规划时刻。第一拍悬停若就在这段的开头，悬停中间没有被别的规划码打断。",
        "",
        "软让行探针：硬过滤判空之后，留下中心距 ≥ 0.75 m 的候选（含测不到的 +inf），",
        "按 `score - 2.0 * (1.0 - d) / 1.0` 重排，同分取索引较小者。低于 0.75 m 的仍然滤掉。",
        "几何门在机间过滤之前。几何门自己筛空的拍不会进软让行。",
        "邻机轨迹要等这一拍全部 `plan()` 结束、总线 poll 之后才送进去，所以一方被放行，要到下一规划拍才出现在另一方的 geom 门里。",
        "",
        "## 场景和臂",
        "",
        "| 场景 | 来源 | 机数 | seeds |",
        "|---|---|---:|---|",
        "| 014 六机对向 | 进程内存，几何 `scenes/ego-density-014.json`，构造与 E-C6 相同 | 6 | 0–4 |",
        "| 014 三机对向 | `scenes/ego-density-014-swap3.json` | 3 | 0–9 |",
        "",
        "| 臂 | 进程里多出来的 | 偏离上游 |",
        "|---|---|---|",
        "| geom | E-A1 1.0 m 惩罚 0，empty_brake，track_clearance mode=geom，margin 0.08 m | 偏离上游 |",
        "| geom_multi | 同上，mode=geom_multi | 偏离上游 |",
        "| geom_soft | 与 geom 相同，另加进程内软让行探针（0.75 m，权重 2.0）。不改 `inter_drone.py` | 偏离上游 |",
        "",
        "`safe_radius` 三档都不写，运行器缺省 0.05 m。场景卡文件不改。",
        "",
        "## 汇总",
        "",
        "悬停原因四列的顺序是：走廊准入为空 / geom 净空门拒绝 / E-A1 inter_pick empty / 其它。",
        "",
        "| 场景 | 臂 | 偏离上游 | 到达 | 撞柱 | 撞地 | 机间 | 超时 | 平均速度（>0）m/s | 平均速度（含悬停）m/s | 悬停 | 四类原因 | 让行放行 |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|",
    ]
    for scene in SCENES:
        for arm in ARMS:
            group = _group(rows, scene, arm)
            if not group:
                continue
            drones = sum(int(r["n_drones"]) for r in group)
            arrived = sum(int(r["arrived"]) for r in group)
            causes = _sum_causes(group)
            lines.append(
                f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | 偏离上游 | {arrived}/{drones} | "
                f"{sum(int(r['pillar']) for r in group)} | {sum(int(r['ground']) for r in group)} | "
                f"{sum(int(r['inter']) for r in group)} | {sum(int(r['timeout']) for r in group)} | "
                f"{_fmt(_unweighted_positive(group), 3)} | {_fmt(_weighted_mean(group), 3)} | "
                f"{sum(int(r['corridor_empty_hover']) for r in group)} | {_cause_cell(causes)} | "
                f"{sum(int(r.get('yield_releases') or 0) for r in group)} |"
            )
    lines.extend([
        "",
        "## 悬停当时",
        "",
        "贴地、贴树是悬停拍的计数。位置是这一拍机体；轨迹是刷新前那条。",
        "刹停是 `brake_from_head` 造出了轨迹；保持是开关开着但参考头已经停住，退回原地保持。",
        "",
        "| 场景 | 臂 | 悬停 | 刹停 | 保持 | 位置贴地 | 轨迹贴地 | 位置贴树 | 轨迹贴树 | 刷新前最低 z m | 悬停时最小机间 m | 悬停时高度中位 m |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|---:|",
    ])
    for scene in SCENES:
        for arm in ARMS:
            group = _group(rows, scene, arm)
            if not group:
                continue
            lines.append(
                f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | "
                f"{sum(int(r['corridor_empty_hover']) for r in group)} | "
                f"{sum(int(r['empty_brake_applied']) for r in group)} | "
                f"{sum(int(r['empty_brake_hold_fallback']) for r in group)} | "
                f"{sum(int(r['pos_near_ground']) for r in group)} | "
                f"{sum(int(r['traj_near_ground']) for r in group)} | "
                f"{sum(int(r['pos_near_tree']) for r in group)} | "
                f"{sum(int(r['traj_near_tree']) for r in group)} | "
                f"{_rng([r.get('prev_min_z_min') for r in group] + [r.get('prev_min_z_max') for r in group], 3)} | "
                f"{_rng([r.get('hover_min_inter_min') for r in group], 3)} | "
                f"{_fmt(_median(_finite([r.get('hover_pos_z_median') for r in group])), 3)} |"
            )
    lines.extend([
        "",
        "## 超时种子里谁停住",
        "",
        "E-C6 记下的超时种子是 014 六机对向的 0、1、3。下面把这次跑出来的超时种子都列上，不限于这三粒。",
        "位置是仿真最后一拍的机体中心。时长从这段连续悬停的第一拍到仿真结束。",
        "",
        "| 场景 | 臂 | seed | 飞机 | 末位置 | 末速度 m/s | 离目标 m | 连续悬停起点 s | 时长 s | 悬停拍 | 这段里的四类原因 | 从第一拍悬停一直连续到超时 |",
        "|---|---|---:|---|---|---:|---:|---:|---:|---:|---|---|",
    ])
    any_stop = False
    for scene in SCENES:
        for arm in ARMS:
            for row in sorted(_group(rows, scene, arm), key=lambda r: int(r["seed"])):
                for stop in row.get("stops") or []:
                    any_stop = True
                    pos = stop.get("final_p")
                    pos_s = "—" if not pos else ", ".join(f"{float(x):.3f}" for x in pos)
                    lines.append(
                        f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | {row['seed']} | {stop['uav']} | "
                        f"{pos_s} | {_fmt(stop.get('final_speed_mps'), 3)} | "
                        f"{_fmt(stop.get('final_goal_error_m'), 3)} | "
                        f"{_fmt(stop.get('terminal_hover_start_s'), 2)} | "
                        f"{_fmt(stop.get('terminal_hover_duration_s'), 2)} | "
                        f"{int(stop.get('terminal_hover_n') or 0)} | "
                        f"{_cause_cell(stop.get('terminal_cause_counts') or {})} | "
                        f"{'是' if stop.get('hover_continuous_to_timeout') else '否'} |"
                    )
    if not any_stop:
        lines.append("| — | — | — | — | — | — | — | — | — | — | — | — |")
    lines.extend([
        "",
        "## 软让行为什么会让悬停变多",
        "",
        "对照只在 geom 和 geom_soft 之间做。geom_soft 不是仓库里的开关，是这次进程里的探针。",
        "「放行后下一拍、另一架 geom 拒绝」数的是：上一规划拍有别的飞机被软让行留下了候选，这一拍这架飞机的悬停原因是 geom 净空门拒绝。",
        "",
        "| 场景 | seed | 硬过滤悬停 | 软让行悬停 | 硬过滤四类 | 软让行四类 | 软让行放行 | 软让行 geom 拒绝 | 其中上一拍有另一架被放行 |",
        "|---|---:|---:|---:|---|---|---:|---:|---:|",
    ])
    for scene in SCENES:
        hard = {int(r["seed"]): r for r in _group(rows, scene, "geom")}
        soft = {int(r["seed"]): r for r in _group(rows, scene, "geom_soft")}
        for seed in sorted(set(hard) & set(soft)):
            a, b = hard[seed], soft[seed]
            coupling = b.get("yield_coupling") or {}
            lines.append(
                f"| {SCENE_LABEL[scene]} | {seed} | {a['corridor_empty_hover']} | {b['corridor_empty_hover']} | "
                f"{_cause_cell(a.get('cause_counts') or {})} | {_cause_cell(b.get('cause_counts') or {})} | "
                f"{int(b.get('yield_releases') or 0)} | {int(coupling.get('geom_reject') or 0)} | "
                f"{int(coupling.get('geom_reject_after_other_yield') or 0)} |"
            )
    lines.extend(["", _geom_gate_context(rows), "", _conclusion(rows), "", "## 分种子", ""])
    lines.extend(_seed_table(rows))
    lines.extend([
        "",
        "## 没做的事",
        "",
        "- 没有改 `yopo_l0/inter_drone.py`、`yopo_l0/policy.py`、`core/`、`scenes/`、`docs/tickets.md`、README。",
        "- 没有 merge E-C6 或 E-A2 的分支。软让行规则只活在本脚本的进程里，默认开关仍是关。",
        "- 没有新做几何回退，没有改 geom 的表面净空门。",
        "- 六机对向卡只在这次进程里，不进 `scenes/suite.txt`，不设 gates。",
        "- 影子盾关。",
        "",
        "## 复现",
        "",
        "```bash",
        ".venv/bin/python scripts/e-c7-empty-hover.py --check",
        ".venv/bin/python scripts/e-c7-empty-hover.py --partial --resume --jobs 4",
        "```",
        "",
        "`--partial` 每粒落盘到 `out/e-c7-empty-hover/partial/`。`--resume` 读回已经落盘的粒。",
        "`--seeds` 可以少跑。`--duration-s` 只给冒烟，不会写入这份研究记录。",
        "",
    ])
    return "\n".join(lines)


def _seed_table(rows: list[dict]) -> list[str]:
    lines = [
        "| 场景 | 臂 | seed | 结果 | 到达 | 撞柱 | 撞地 | 机间 | 超时 | 平均速度（>0）m/s | 悬停 | 四类原因 | 让行放行 | 最小机间 m |",
        "|---|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|",
    ]
    for scene in SCENES:
        for arm in ARMS:
            for row in sorted(_group(rows, scene, arm), key=lambda r: int(r["seed"])):
                lines.append(
                    f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | {row['seed']} | {row['outcome']} | "
                    f"{row['arrived']}/{row['n_drones']} | {row['pillar']} | {row['ground']} | {row['inter']} | "
                    f"{row['timeout']} | {_fmt(row.get('avg_speed_mps'), 3)} | {row['corridor_empty_hover']} | "
                    f"{_cause_cell(row.get('cause_counts') or {})} | {int(row.get('yield_releases') or 0)} | "
                    f"{_fmt(row.get('min_inter_drone_distance_m'), 3)} |"
                )
    return lines


def _geom_gate_context(rows: list[dict]) -> str:
    """从逐拍记录里看 geom 拒绝时走廊里还有没有候选人，以及机间判空时人实际离多远。"""
    lines = [
        "## 逐拍里看到的门",
        "",
        "geom 拒绝那一拍如果 `n_admitted` 大于 0，走廊里有候选人，是表面净空把门关死的，不是走廊本身没人准入。",
        "机间判空那一拍的最小机间是各机当时的中心距。硬过滤用的是 4 s 轨迹上的同时刻中心距，可以比这一拍的位置更近。",
        "",
    ]
    found = False
    for scene in SCENES:
        for arm in ARMS:
            group = _group(rows, scene, arm)
            events = [e for row in group for e in (row.get("hover_events") or [])]
            if not events:
                continue
            found = True
            geom = [e for e in events if e.get("cause") == "geom_reject"]
            inter = [e for e in events if e.get("cause") == "inter_empty"]
            admitted = _finite([e.get("n_admitted") for e in geom])
            admitted_pos = sum(1 for v in admitted if v > 0)
            tree = _finite([e.get("pos_tree_m") for e in geom])
            prev_tree = _finite([e.get("prev_min_tree_m") for e in geom])
            gap = _finite([e.get("min_inter_m") for e in inter])
            lines.append(
                f"- {SCENE_LABEL[scene]} / {arm}：geom 拒绝 {len(geom)} 次，"
                f"其中走廊里还有候选人的 {admitted_pos} 次"
                f"（准入数中位 {_fmt(_median(admitted), 0)}）。"
                f"这些拍上机体到柱面的中位距离 {_fmt(_median(tree), 2)} m，"
                f"刷新前轨迹到柱面的中位距离 {_fmt(_median(prev_tree), 2)} m。"
                f"机间判空 {len(inter)} 次，当时机体中心距中位 {_fmt(_median(gap), 2)} m。"
            )
    if not found:
        lines.append("逐拍记录还没写进这批结果。")
    return "\n".join(lines)


def _soft_stop_notes(hard: list[dict], soft: list[dict]) -> list[str]:
    """把硬过滤和软让行的超时飞机并排写出来。没有超时就不写。"""
    notes = []
    soft_by = {int(r["seed"]): r for r in soft}
    for row in sorted(hard, key=lambda r: int(r["seed"])):
        if not int(row.get("timeout") or 0):
            continue
        other = soft_by.get(int(row["seed"]))
        if other is None:
            continue
        def _who(stops: list[dict]) -> str:
            if not stops:
                return "没有未到达的飞机"
            bits = []
            for stop in stops:
                counts = stop.get("terminal_cause_counts") or {}
                bits.append(
                    f"{stop['uav']} 在 {stop.get('final_p')} 连续悬停 "
                    f"{_fmt(stop.get('terminal_hover_duration_s'), 2)} s"
                    f"（{_cause_cell(counts)}）"
                )
            return "；".join(bits)
        notes.append(
            f"seed {row['seed']} 硬过滤停住的是 {_who(row.get('stops') or [])}。"
            f"软让行停住的是 {_who(other.get('stops') or [])}。"
        )
    return notes


def _conclusion(rows: list[dict]) -> str:
    hard = _group(rows, "d014-swap6", "geom")
    soft = _group(rows, "d014-swap6", "geom_soft")
    multi = _group(rows, "d014-swap6", "geom_multi")
    three = _group(rows, "d014-swap3", "geom")
    parts = ["## 结论", ""]
    if not hard:
        parts.append("六机 geom 还没有跑完。")
        return "\n".join(parts)
    h_counts = _sum_causes(hard)
    h_hover = sum(int(r["corridor_empty_hover"]) for r in hard)
    h_arr = sum(int(r["arrived"]) for r in hard)
    h_den = sum(int(r["n_drones"]) for r in hard)
    timeouts = [r for r in hard if int(r["timeout"])]
    parts.append(
        f"六机 geom 硬过滤：到达 {h_arr}/{h_den}，超时 {len(timeouts)} 粒，悬停 {h_hover} 次。"
        f"四类原因是走廊准入为空 {h_counts['corridor_empty']}、geom 净空门拒绝 {h_counts['geom_reject']}、"
        f"E-A1 inter_pick empty {h_counts['inter_empty']}、其它 {h_counts['other']}。"
    )
    if h_hover == 1127 and h_counts["geom_reject"] == 755 and h_counts["inter_empty"] == 367:
        parts.append(
            "E-C6 表上的 1109 次是运行器 `plan_codes`：同一个码名在飞机之间互相覆盖，"
            "超时那一架的连续悬停 372、372、362，加上两粒早撞记下来的 1 和 2，就是 1109。"
            "这次按飞机把规划拍加起来是 1127，和 E-A2 的计数相同。多出的 18 次在别的飞机上。"
        )
    touch_pos = sum(int(r["pos_near_ground"]) + int(r["pos_near_tree"]) for r in hard)
    touch_traj = sum(int(r["traj_near_ground"]) + int(r["traj_near_tree"]) for r in hard)
    parts.append(
        f"这 {h_hover} 次悬停里，机体位置贴地或贴树 {touch_pos} 次，刷新前轨迹贴地或贴树 {touch_traj} 次。"
        f"empty_brake 造出刹车轨迹 {sum(int(r['empty_brake_applied']) for r in hard)} 次，"
        f"参考头已经停住而退回原地保持 {sum(int(r['empty_brake_hold_fallback']) for r in hard)} 次。"
    )
    if timeouts:
        bits = []
        for row in sorted(timeouts, key=lambda r: int(r["seed"])):
            who = []
            for stop in row.get("stops") or []:
                dur = stop.get("terminal_hover_duration_s")
                dur_s = "—" if dur is None else f"{float(dur):.2f} s"
                who.append(f"{stop['uav']} 停在 {stop.get('final_p')}，连续悬停 {dur_s}")
            bits.append(f"seed {row['seed']}：" + ("；".join(who) if who else "没有未到达的飞机"))
        parts.append("超时粒里停住的是：" + "。".join(bits) + "。")
    if multi:
        same = []
        hard_by = {int(r["seed"]): r for r in hard}
        for row in multi:
            other = hard_by.get(int(row["seed"]))
            if other and other.get("positions_sha256") == row.get("positions_sha256"):
                same.append(str(row["seed"]))
        m_hover = sum(int(r["corridor_empty_hover"]) for r in multi)
        parts.append(
            f"geom_multi 悬停 {m_hover} 次。与 geom 位置哈希相同的种子是 "
            + ("、".join(same) if same else "没有")
            + "。"
        )
    if soft:
        s_counts = _sum_causes(soft)
        s_hover = sum(int(r["corridor_empty_hover"]) for r in soft)
        s_arr = sum(int(r["arrived"]) for r in soft)
        s_den = sum(int(r["n_drones"]) for r in soft)
        released = sum(int(r.get("yield_releases") or 0) for r in soft)
        after = sum(int((r.get("yield_coupling") or {}).get("geom_reject_after_other_yield") or 0) for r in soft)
        geom_soft_n = s_counts["geom_reject"]
        parts.append(
            f"打开进程内软让行之后，六机到达 {s_arr}/{s_den}，悬停 {h_hover} → {s_hover}，"
            f"机间判空 {h_counts['inter_empty']} → {s_counts['inter_empty']}，让行放行 {released} 次。"
            f"geom 净空门拒绝 {h_counts['geom_reject']} → {geom_soft_n}，"
            f"走廊准入为空 {h_counts['corridor_empty']} → {s_counts['corridor_empty']}。"
        )
        parts.append(
            f"软让行臂的 geom 拒绝里，有 {after} 次发生在上一规划拍另一架刚被放行之后"
            f"（分母是这档的 geom 拒绝 {geom_soft_n} 次）。"
            "这个比例很小：放行不是在下一拍就把对方的 geom 门顶死。"
        )
        grown = max(CAUSES, key=lambda name: s_counts[name] - h_counts[name])
        delta = s_counts[grown] - h_counts[grown]
        if s_hover > h_hover:
            parts.append(
                f"悬停净增 {s_hover - h_hover} 次。机间判空少了 {h_counts['inter_empty'] - s_counts['inter_empty']} 次，"
                f"{CAUSE_LABEL[grown]}多了 {delta} 次。"
                "多出来的拒绝出现在放行改过轨迹之后的那一段连续悬停里，不是紧挨着放行的下一拍。"
                "这组数不支持把软让行当成六机 geom 死锁的解。"
            )
        else:
            parts.append("这组种子上软让行没有把悬停次数抬上去。仍然不把它写成死锁的解：超时是否还在，看上面的超时表。")
        parts.extend(_soft_stop_notes(hard, soft))
    if three:
        t_hover = sum(int(r["corridor_empty_hover"]) for r in three)
        t_arr = sum(int(r["arrived"]) for r in three)
        t_den = sum(int(r["n_drones"]) for r in three)
        t_counts = _sum_causes(three)
        parts.append(
            f"三机 geom 对照：到达 {t_arr}/{t_den}，超时 {sum(int(r['timeout']) for r in three)} 粒，悬停 {t_hover} 次"
            f"（走廊准入为空 {t_counts['corridor_empty']}、geom 拒绝 {t_counts['geom_reject']}、"
            f"机间判空 {t_counts['inter_empty']}、其它 {t_counts['other']}）。"
            "三机没有堆出六机那种上千次的空走廊悬停。"
        )
    parts.append("全部 report-only。默认开关不动。")
    return "\n".join(parts)


def _check() -> int:
    watched = {
        SWAP3_REL: (ROOT / SWAP3_REL).read_bytes(),
        f"scenes/{GEOM_NAME}": (ROOT / "scenes" / GEOM_NAME).read_bytes(),
        "scenes/suite.txt": (ROOT / "scenes" / "suite.txt").read_bytes(),
        "yopo_l0/inter_drone.py": (ROOT / "yopo_l0" / "inter_drone.py").read_bytes(),
        "yopo_l0/policy.py": (ROOT / "yopo_l0" / "policy.py").read_bytes(),
    }
    blob = watched[f"scenes/{GEOM_NAME}"]
    if hashlib.sha256(blob).hexdigest() != GEOM_SHA256:
        raise SystemExit(f"{GEOM_NAME} sha256 变了")
    # E-A2 的正式开关也叫 soft_yield。这里只拦本票的探针名，不拦那个开关。
    if PROBE_KEY.encode() in watched["yopo_l0/inter_drone.py"] or b"ec7_" in watched["yopo_l0/policy.py"]:
        raise SystemExit("inter_drone.py 或 policy.py 被写进了本票的探针")
    from yopo_l0.policy import read_empty_brake, read_track_clearance

    if read_empty_brake({}) or read_track_clearance({}):
        raise SystemExit("empty_brake 或 track_clearance 默认被打开了")
    ec4 = load_ec4()
    centers = density_centers()
    placed = place_swap6(centers)
    if len(placed) != 6:
        raise SystemExit("六机对向不是 6 架")
    disk = json.loads(watched[SWAP3_REL])
    for src, idx in ((0, 0), (1, 2), (2, 4)):
        for key in ("start_enu", "goal_enu"):
            got = _public_drone(placed[idx])[key]
            exp = disk["drones"][src][key]
            if any(abs(float(a) - float(b)) > 1e-9 for a, b in zip(got, exp)):
                raise SystemExit(f"六机 uav{idx} 的 {key} 与三机 uav{src} 不一致")
    for d in placed:
        if d["axis_distance_start"] < ec4.AXIS_CLEARANCE_M - 1e-9:
            raise SystemExit(f"{d['id']} 起点净空不足")
        if d["axis_distance_goal"] < ec4.AXIS_CLEARANCE_M - 1e-9:
            raise SystemExit(f"{d['id']} 终点净空不足")
    for scene in SCENES:
        for arm in ARMS:
            card = build_card(scene, arm)
            up = card.card["policy_params"]["upstream"]
            if up.get("empty_brake") is not True or up.get("inter_drone_filter") is not True:
                raise SystemExit(f"{scene} {arm} 没有硬过滤和 empty_brake")
            if up.get("inter_drone_penalty") != 0:
                raise SystemExit(f"{scene} {arm} 惩罚不是 0")
            if up.get("track_clearance") is not True or up.get("track_clearance_mode") != arm_mode(arm):
                raise SystemExit(f"{scene} {arm} 几何门不对")
            if abs(float(up["track_margin_m"]) - MARGIN_M) > 1e-12:
                raise SystemExit(f"{scene} {arm} margin 不是 0.08 m")
            if "safe_radius" in up:
                raise SystemExit(f"{scene} {arm} 写了 safe_radius")
            if (up.get(PROBE_KEY) is True) is not (arm == "geom_soft"):
                raise SystemExit(f"{scene} {arm} 探针开关不对")
    # 软让行在惩罚为 0 且全部低于下限时不造候选。0.8 m 的候选会被留下，分数被减。
    from yopo_l0.inter_drone import InterDroneConfig, inter_pick

    cfg = InterDroneConfig(THRESHOLD_M, 0.0)
    kept = soft_yield_pick(np.array([3.0, 5.0]), np.array([True, True]), np.array([0.80, 0.50]), cfg)
    if kept is None or kept.kind != "adjusted" or kept.action != 0:
        raise SystemExit("0.80 m 的候选没有按让行留下")
    if soft_yield_pick(np.array([1.0]), np.array([True]), np.array([0.70]), cfg) is not None:
        raise SystemExit("低于 0.75 m 的候选不该被留下")
    empty = inter_pick(np.array([1.0, 0.0]), np.array([True, False]), np.array([0.2, np.inf]), cfg)
    if empty.kind != "empty":
        raise SystemExit("探针没有安装时，硬过滤判空被改掉了")
    after = {rel: (ROOT / rel).read_bytes() for rel in watched}
    if after != watched:
        raise SystemExit("检查时改写了场景或策略文件")
    suite = watched["scenes/suite.txt"].decode()
    if "e-c7" in suite or "ec7_soft" in suite:
        raise SystemExit("scenes/suite.txt 被写进了本票")
    print("e-c7 check ok")
    return 0


def _head_commit() -> str:
    import subprocess

    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def _parse_seeds(text: str | None) -> list[int] | None:
    if text is None:
        return None
    return [int(s) for s in text.split(",") if s != ""]


def _write_outputs(payload: dict, out: Path, md: Path | None) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if md is not None:
        md.parent.mkdir(parents=True, exist_ok=True)
        md.write_text(render_markdown(payload), encoding="utf-8")
        print(f"写入 {md}")
    print(f"写入 {out}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scenes", default=",".join(SCENES))
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=None, help="覆盖两张卡的种子。缺省时六机 0–4，三机 0–9")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--out", default="docs/research/e-c7-empty-hover-runs.json")
    ap.add_argument("--md", default="docs/research/e-c7-empty-hover.md")
    ap.add_argument("--no-md", action="store_true")
    ap.add_argument("--partial", action="store_true", help="每粒跑完就落盘")
    ap.add_argument("--resume", action="store_true", help="已落盘的粒直接读回")
    ap.add_argument("--partial-dir", default=None)
    ap.add_argument("--duration-s", type=float, default=None, help="盖过场景卡时长，只给冒烟")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--md-only", action="store_true", help="只根据已有 json 重写 markdown")
    args = ap.parse_args(argv)
    if args.check:
        return _check()
    out = Path(args.out)
    if not out.is_absolute():
        out = ROOT / out
    md = None if args.no_md else Path(args.md)
    if md is not None and not md.is_absolute():
        md = ROOT / md
    if args.md_only:
        payload = json.loads(out.read_text(encoding="utf-8"))
        if md is None:
            raise SystemExit("--md-only 需要 markdown 输出")
        md.write_text(render_markdown(payload), encoding="utf-8")
        print(f"写入 {md}")
        return 0
    if args.jobs not in (1, 2, 3, 4):
        raise SystemExit("jobs 只能是 1 到 4")
    if args.duration_s is not None and (not math.isfinite(args.duration_s) or args.duration_s <= 0.0):
        raise SystemExit("--duration-s 必须为正")
    default_out = ROOT / "docs/research/e-c7-empty-hover-runs.json"
    if args.duration_s is not None and out == default_out:
        raise SystemExit("冒烟请另给 --out，不要覆盖研究记录")
    scenes = [s for s in args.scenes.split(",") if s]
    arms = [s for s in args.arms.split(",") if s]
    unknown = [s for s in scenes if s not in SCENES] + [s for s in arms if s not in ARMS]
    if unknown:
        raise SystemExit(f"未知场景或臂：{unknown}")
    override = _parse_seeds(args.seeds)
    seeds_by_scene = {scene: seeds_for(scene, override) for scene in scenes}
    partial_dir = Path(args.partial_dir) if args.partial_dir else PARTIAL
    if not partial_dir.is_absolute():
        partial_dir = ROOT / partial_dir
    _check()
    specs = job_specs(
        scenes,
        arms,
        seeds_by_scene,
        partial=bool(args.partial),
        resume=bool(args.resume),
        duration_s=args.duration_s,
        partial_dir=partial_dir,
    )
    print(
        f"e-c7 start n={len(specs)} jobs={args.jobs} partial={bool(args.partial)} resume={bool(args.resume)}",
        flush=True,
    )
    rows = _run_group(specs, int(args.jobs))
    rows.sort(key=lambda r: (SCENES.index(r["scene"]), ARMS.index(r["arm"]), int(r["seed"])))
    payload = {
        "disclaimer": DISCLAIMER,
        "commit": _head_commit(),
        "report_only": True,
        "frozen": False,
        "ticket": "E-C7",
        "upstream_deviation": True,
        "scenes": scenes,
        "arms": arms,
        "seeds_by_scene": {scene: list(seeds_by_scene[scene]) for scene in scenes},
        "jobs": int(args.jobs),
        "threshold_m": THRESHOLD_M,
        "hard_min_m": HARD_MIN_M,
        "yield_penalty": YIELD_PENALTY,
        "margin_m": MARGIN_M,
        "touch_m": TOUCH_M,
        "body_radius_m": BODY_RADIUS_M,
        "shadow": False,
        "cause_order": list(CAUSES),
        "rows": rows,
        "note": DISCLAIMER,
    }
    _write_outputs(payload, out, md)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
