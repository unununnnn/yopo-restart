#!/usr/bin/env python3
"""票 E-C5c：E-C5b 地面盲区归因，以及密度 0.28 五档对照。只报数。

诊断固定为 `scenes/ego-density-014-swap3.json` 上 seed 2 与 seed 9、
`geom` / `geom_multi` 的 `uav1`。密度对照是 `ego-density-028-swap3` 与
`ego-density-028-solo`，seeds 0–9，五档参数与 E-C5b 相同，只在进程里改：

- off：默认全关（safe_radius 0.05 m，E-A1 关，empty_brake 关，track_clearance 关）
- base：0.05 m + E-A1 阈值 1.0 m、惩罚 0 + empty_brake
- pred：base + track_clearance，mode=pred，margin=0.08 m（解码器半径 0.13 m）
- geom：base + mode=geom，margin=0.08 m（表面净空 ≥ 0.35 + 0.08）
- geom_multi：base + mode=geom_multi。没有邻机时不启几何门

逐粒落在 `out/e-c5c-blindspot-dense/partial/`，重跑默认为断点续跑。
不导入 `scripts/e-c5-corridor-radius.py`、`scripts/e-c5b-track-clearance.py`
或尚未合入的 `yopo_l0.experiment`。不改已有文件，不改默认值。
"""

from __future__ import annotations

import argparse
import hashlib
import inspect
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
PARTIAL = ROOT / "out" / "e-c5c-blindspot-dense" / "partial"
SCHEMA = "e-c5c/1"

SCENES = {
    "d014-swap3": "scenes/ego-density-014-swap3.json",
    "d028-swap3": "scenes/ego-density-028-swap3.json",
    "d028-solo": "scenes/ego-density-028-solo.json",
}
DENSE_ORDER = ("d028-swap3", "d028-solo")
DIAG_SCENE = "d014-swap3"
DIAG_ARMS = ("geom", "geom_multi")
DIAG_SEEDS = (2, 9)
DIAG_DRONE = "uav1"
SCENE_LABEL = {
    "d014-swap3": "014 对向",
    "d028-swap3": "028 对向",
    "d028-solo": "028 单机",
}
ARMS = ("off", "base", "pred", "geom", "geom_multi")
SEEDS = tuple(range(10))
BODY_RADIUS_M = 0.35
MARGIN_M = 0.08
GEOM_LIMIT_M = BODY_RADIUS_M + MARGIN_M
UPSTREAM_SAFE_RADIUS_M = 0.05
LOW_TRUNK_Z_M = 0.8
PREIMPACT_S = 1.0
PLAN_KEEP = 16
GROUND_PIXEL_STRIDE = 2
GROUND_PIXEL_M = 0.15

DISCLAIMER = (
    "只报数（report-only）。不改默认 safe_radius 0.05 m、empty_brake false、track_clearance false。"
    "E-A1、empty_brake、track_clearance 打开即偏离上游。"
    "track_margin 0.08 m 是假设，用来盖 E-C5 已观测的 0.058–0.073 m 跟踪差。"
    "这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。"
    "未冻结，不进 scenes/suite.txt，不设 gates。"
    "密度 0.28 的五档对照只报数，不据此改规划器。"
)

A1 = {
    "inter_drone_filter": True,
    "inter_drone_threshold_m": 1.0,
    "inter_drone_penalty": 0,
}

_PLAN_KEYS = (
    "plan_code",
    "geom_gate_active",
    "n_neighbors",
    "ref_is_brake",
    "ref_min_z_m",
    "ref_ground_clearance_m",
    "ref_cylinder_clearance_m",
    "ref_surface_clearance_m",
    "depth_min_valid_m",
    "depth_n_valid",
    "ground_pixels",
    "ground_pixel_stride",
    "ec5_cylinder_name",
    "ec5_cylinder_distance_m",
    "ec5_cylinder_in_depth",
)


def arm_deviates(arm: str) -> bool:
    return arm != "off"


def arm_label(arm: str) -> str:
    labels = {
        "off": "默认全关",
        "base": "0.05 m + E-A1 1.0 m + empty_brake（偏离上游）",
        "pred": "base + track pred 0.08 m（偏离上游）",
        "geom": "base + track geom 0.08 m（偏离上游）",
        "geom_multi": "base + track geom_multi 0.08 m（偏离上游）",
    }
    if arm not in labels:
        raise ValueError(arm)
    return labels[arm]


def decoder_radius_m(arm: str) -> float:
    """pred 把解码器半径抬到 0.13 m。结果字段 safe_radius_m 仍是 0.05 m。"""
    if arm == "pred":
        return UPSTREAM_SAFE_RADIUS_M + MARGIN_M
    return UPSTREAM_SAFE_RADIUS_M


def build_card(scene: str, arm: str):
    from yopo_l0.scene import SceneCard, load_card, parse_geometry

    if scene not in SCENES:
        raise ValueError(scene)
    path = ROOT / SCENES[scene]
    if arm == "off":
        return load_card(path)
    if arm not in ARMS:
        raise ValueError(arm)
    raw = json.loads(path.read_text(encoding="utf-8"))
    params = dict(raw.get("policy_params") or {})
    upstream = dict(params.get("upstream") or {})
    upstream.update(A1)
    upstream["empty_brake"] = True
    if arm == "pred":
        upstream.update(track_clearance=True, track_clearance_mode="pred", track_margin_m=MARGIN_M)
    elif arm == "geom":
        upstream.update(track_clearance=True, track_clearance_mode="geom", track_margin_m=MARGIN_M)
    elif arm == "geom_multi":
        upstream.update(
            track_clearance=True, track_clearance_mode="geom_multi", track_margin_m=MARGIN_M
        )
    elif arm != "base":
        raise ValueError(arm)
    if "safe_radius" in upstream:
        raise RuntimeError("对照不写 safe_radius，0.05 m 走运行器缺省")
    params["upstream"] = upstream
    raw["policy_params"] = params
    card = json.loads(json.dumps(raw))
    geom = card.get("geometry")
    if isinstance(geom, str):
        card["geometry"] = json.loads((ROOT / "scenes" / geom).read_text(encoding="utf-8"))
    return SceneCard(card["name"], card, parse_geometry(card["geometry"], card.get("ground_z")))


def _file_hash(scene: str) -> str:
    from yopo_l0.scene import load_card

    return load_card(ROOT / SCENES[scene]).hash


class NearQuery:
    """表面距离。直立圆柱公式与 scene.Cylinder.distance 相同，地面是高度差。"""

    def __init__(self, obstacles):
        from yopo_l0.scene import Box, Cylinder, Ground

        self.grounds = []
        self.cylinders = []
        self.boxes = []
        self.others = []
        for ob in obstacles:
            if isinstance(ob, Ground):
                self.grounds.append(ob)
            elif isinstance(ob, Cylinder) and ob.upright():
                self.cylinders.append(ob)
            elif isinstance(ob, Box):
                self.boxes.append(ob)
            else:
                self.others.append(ob)
        if self.cylinders:
            self.center = np.asarray([c.center_xy for c in self.cylinders], dtype=np.float64)
            self.radius = np.asarray([c.radius for c in self.cylinders], dtype=np.float64)
            self.z_lo = np.asarray([c.z_lo for c in self.cylinders], dtype=np.float64)
            self.z_hi = np.asarray([c.z_hi for c in self.cylinders], dtype=np.float64)
        else:
            self.center = None

    def cylinder_distances(self, points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float64)
        if pts.ndim == 1:
            pts = pts.reshape(1, 3)
        if self.center is None or pts.shape[0] == 0:
            return np.full(pts.shape[0], np.inf)
        radial = np.hypot(
            pts[:, None, 0] - self.center[None, :, 0],
            pts[:, None, 1] - self.center[None, :, 1],
        ) - self.radius
        radial = np.maximum(radial, 0.0)
        dz = np.maximum(
            np.maximum(self.z_lo - pts[:, None, 2], 0.0),
            pts[:, None, 2] - self.z_hi,
        )
        return np.hypot(radial, dz).min(axis=1)

    def ground_clearance(self, p: np.ndarray) -> float:
        if not self.grounds:
            return math.inf
        z = float(np.asarray(p, dtype=float)[2])
        return min(max(z - float(g.z), 0.0) for g in self.grounds)

    def box_clearance(self, p: np.ndarray) -> float:
        if not self.boxes:
            return math.inf
        point = np.asarray(p, dtype=float)
        return min(float(ob.distance(point)) for ob in self.boxes)

    def nearest_cylinder(self, p: np.ndarray):
        if self.center is None:
            return None, math.inf
        point = np.asarray(p, dtype=np.float64).reshape(1, 3)
        radial = np.hypot(
            point[0, 0] - self.center[:, 0],
            point[0, 1] - self.center[:, 1],
        ) - self.radius
        radial = np.maximum(radial, 0.0)
        dz = np.maximum(np.maximum(self.z_lo - point[0, 2], 0.0), point[0, 2] - self.z_hi)
        dist = np.hypot(radial, dz)
        idx = int(np.argmin(dist))
        return self.cylinders[idx], float(dist[idx])

    def nearest(self, p: np.ndarray) -> dict:
        point = np.asarray(p, dtype=np.float64).reshape(3)
        found = []
        if self.center is not None:
            radial_vec = point[:2] - self.center
            gap = np.hypot(radial_vec[:, 0], radial_vec[:, 1]) - self.radius
            dz = np.where(
                point[2] < self.z_lo,
                self.z_lo - point[2],
                np.where(point[2] > self.z_hi, point[2] - self.z_hi, 0.0),
            )
            dist = np.hypot(np.maximum(gap, 0.0), dz)
            idx = int(np.argmin(dist))
            direction = radial_vec[idx]
            norm = float(np.hypot(direction[0], direction[1]))
            if norm < 1e-12:
                direction = np.array([1.0, 0.0])
                norm = 1.0
            surf_xy = self.center[idx] + direction / norm * float(self.radius[idx])
            cz = float(np.clip(point[2], self.z_lo[idx], self.z_hi[idx]))
            if abs(float(dz[idx])) < 1e-15:
                closest = np.array([surf_xy[0], surf_xy[1], float(point[2])])
            elif float(gap[idx]) > 0.0:
                closest = np.array([surf_xy[0], surf_xy[1], cz])
            else:
                closest = np.array([point[0], point[1], cz])
            found.append((
                float(dist[idx]),
                1,
                "cylinder",
                self.cylinders[idx].name,
                closest,
                {
                    "z_lo": float(self.z_lo[idx]),
                    "z_hi": float(self.z_hi[idx]),
                    "closest_z_m": float(closest[2]),
                },
            ))
        for ground in self.grounds:
            dist = max(float(point[2]) - float(ground.z), 0.0)
            closest = np.array([point[0], point[1], float(ground.z)])
            found.append((
                dist,
                0,
                "ground",
                ground.name,
                closest,
                {"ground_z": float(ground.z), "closest_z_m": float(ground.z)},
            ))
        for box in self.boxes:
            dist = float(box.distance(point))
            lo = np.asarray(box.lo, dtype=float)
            hi = np.asarray(box.hi, dtype=float)
            closest = np.minimum(np.maximum(point, lo), hi)
            found.append((
                dist,
                2,
                "box",
                box.name,
                closest,
                {"closest_z_m": float(closest[2])},
            ))
        for ob in self.others:
            dist = float(ob.distance(point))
            found.append((dist, 3, type(ob).__name__.lower(), getattr(ob, "name", "other"), point.copy(), {}))
        if not found:
            return {
                "nearest_kind": "—",
                "nearest_raw_kind": None,
                "nearest_name": None,
                "nearest_distance_m": None,
                "closest_enu": None,
                "closest_z_m": None,
            }
        dist, _prio, raw, name, closest, extra = min(found, key=lambda item: (item[0], item[1]))
        closest_z = extra.get("closest_z_m")
        if raw == "ground":
            kind = "Ground"
        elif raw == "cylinder" and closest_z is not None and float(closest_z) <= LOW_TRUNK_Z_M:
            kind = "低处树干"
        elif raw == "cylinder":
            kind = "圆柱"
        elif raw == "box":
            kind = "盒子"
        else:
            kind = raw
        return {
            "nearest_kind": kind,
            "nearest_raw_kind": raw,
            "nearest_name": name,
            "nearest_distance_m": float(dist),
            "closest_enu": [float(x) for x in closest],
            "closest_z_m": None if closest_z is None else float(closest_z),
            **{k: v for k, v in extra.items() if k != "closest_z_m"},
        }


def flight_path_pitch(velocity) -> float | None:
    """速度方向的俯仰。点质量没有姿态俯仰，这个角不进深度相机。"""
    v = np.asarray(velocity, dtype=float).reshape(3)
    horiz = float(np.hypot(v[0], v[1]))
    if horiz < 1e-3 and abs(float(v[2])) < 1e-3:
        return None
    return float(math.atan2(float(v[2]), horiz))


def camera_attitude() -> dict:
    """深度相机只吃偏航。State 也没有俯仰字段。"""
    from yopo_l0.dynamics import State
    from yopo_l0.sensor import DepthCamera

    params = list(inspect.signature(DepthCamera.capture).parameters)
    fields = list(getattr(State, "__dataclass_fields__", {}))
    return {
        "capture_params": params,
        "camera_uses_pitch": "pitch" in params,
        "state_has_pitch": "pitch" in fields,
        "attitude_pitch_rad": 0.0,
    }


def _ground_z(query: NearQuery) -> float:
    if not query.grounds:
        return 0.0
    return float(query.grounds[0].z)


def camera_origin(position, yaw: float, cam) -> np.ndarray:
    from yopo_l0.policy import R_yaw

    mount = np.asarray(cam.mount_xyz_body, dtype=float)
    return np.asarray(position, dtype=float) + R_yaw(float(yaw)) @ mount


def vfov_coverage(cam, origin: np.ndarray, ground_z: float) -> dict:
    half = float(cam.vfov_deg) / 2.0
    half_rad = math.radians(half)
    cam_h = float(origin[2]) - float(ground_z)
    if cam_h <= 0.0 or half <= 0.0 or half >= 90.0:
        enters = None
    else:
        enters = cam_h / math.tan(half_rad)
    return {
        "vfov_deg": float(cam.vfov_deg),
        "vfov_half_deg": half,
        "optical_axis_elevation_deg": 0.0,
        "lower_edge_elevation_deg": -half,
        "upper_edge_elevation_deg": half,
        "camera_height_above_ground_m": cam_h,
        "ground_enters_horizontal_m": enters,
        "nadir_in_vfov": bool(half >= 90.0 - 1e-9),
    }


def point_bearing(cam, origin: np.ndarray, yaw: float, world_point) -> dict:
    """接触点在相机系里的俯仰。光轴前方才谈得上是否落进 VFOV。"""
    from yopo_l0.policy import R_yaw
    from yopo_l0.sensor import R_BODY_CAM

    rel = (R_yaw(float(yaw)) @ R_BODY_CAM).T @ (np.asarray(world_point, dtype=float) - origin)
    in_front = bool(float(rel[2]) > 1e-6)
    elevation = None
    in_vfov = False
    if in_front:
        elevation = float(math.degrees(math.atan2(-float(rel[1]), float(rel[2]))))
        azimuth = float(math.degrees(math.atan2(float(rel[0]), float(rel[2]))))
        in_vfov = abs(elevation) <= float(cam.vfov_deg) / 2.0 + 1e-6 and abs(azimuth) <= float(cam.hfov_deg) / 2.0 + 1e-6
    return {
        "closest_in_front": in_front,
        "closest_elevation_deg": elevation,
        "closest_in_vfov": in_vfov,
    }


def body_delta(position, yaw: float, world_point) -> np.ndarray:
    from yopo_l0.policy import R_yaw

    rel = np.asarray(world_point, dtype=float) - np.asarray(position, dtype=float)
    return R_yaw(float(yaw)).T @ rel


_RAYS: dict[tuple, np.ndarray] = {}


def _ray_dirs(cam) -> np.ndarray:
    key = (int(cam.width), int(cam.height), float(cam.hfov_deg), float(cam.vfov_deg))
    cached = _RAYS.get(key)
    if cached is None:
        from yopo_l0.sensor import _ray_dirs_cam

        cached = np.asarray(_ray_dirs_cam(cam), dtype=np.float64)
        _RAYS[key] = cached
    return cached


def analyze_depth(frame, position, yaw: float, cam, query: NearQuery) -> dict:
    """最小有效深度、隔点地面像素，以及 E-C5 同款的「最近圆柱在不在图里」。"""
    from yopo_l0.dense_diag import depth_sees
    from yopo_l0.policy import R_yaw
    from yopo_l0.sensor import R_BODY_CAM

    depth = np.asarray(frame.depth, dtype=np.float64)
    valid = np.isfinite(depth)
    n_valid = int(valid.sum())
    min_valid = None if n_valid == 0 else float(depth[valid].min())
    cyl, cyl_dist = query.nearest_cylinder(position)
    in_depth = None
    if cyl is not None:
        in_depth = bool(depth_sees(
            frame,
            np.asarray(position, dtype=float),
            float(yaw),
            cam,
            cyl,
        )) if n_valid else False
    ground_pixels = 0
    if n_valid and query.grounds:
        flat = depth.ravel()
        sub_d = flat[::GROUND_PIXEL_STRIDE]
        dirs = _ray_dirs(cam).reshape(-1, 3)[::GROUND_PIXEL_STRIDE]
        ok = np.isfinite(sub_d)
        if np.any(ok):
            rot = R_yaw(float(yaw)) @ R_BODY_CAM
            origin = camera_origin(position, yaw, cam)
            pts = origin + sub_d[ok, None] * (dirs[ok] @ rot.T)
            gdist = np.full(pts.shape[0], np.inf)
            for ground in query.grounds:
                gdist = np.minimum(gdist, np.maximum(pts[:, 2] - float(ground.z), 0.0))
            other = query.cylinder_distances(pts)
            for box in query.boxes:
                lo = np.asarray(box.lo, dtype=float)
                hi = np.asarray(box.hi, dtype=float)
                delta = np.maximum(np.maximum(lo - pts, 0.0), pts - hi)
                other = np.minimum(other, np.linalg.norm(delta, axis=1))
            ground_pixels = int(np.sum((gdist <= GROUND_PIXEL_M) & (gdist + 1e-6 < other)))
    return {
        "depth_min_valid_m": min_valid,
        "depth_n_valid": n_valid,
        "ground_pixels": ground_pixels,
        "ground_pixel_stride": GROUND_PIXEL_STRIDE,
        "ec5_cylinder_name": None if cyl is None else cyl.name,
        "ec5_cylinder_distance_m": None if cyl is None or not math.isfinite(cyl_dist) else float(cyl_dist),
        "ec5_cylinder_in_depth": in_depth,
    }


def split_reference(traj, query: NearQuery, obstacles) -> dict:
    from yopo_l0.policy import reference_surface_clearance, trajectory_sample_points

    pts = trajectory_sample_points(traj)
    if len(pts) == 0:
        return {
            "ref_min_z_m": None,
            "ref_ground_clearance_m": None,
            "ref_cylinder_clearance_m": None,
            "ref_surface_clearance_m": None,
        }
    ground = math.inf
    for item in query.grounds:
        ground = min(ground, float(np.maximum(pts[:, 2] - float(item.z), 0.0).min()))
    cyl = query.cylinder_distances(pts)
    surface = float(reference_surface_clearance(traj, obstacles))
    return {
        "ref_min_z_m": float(np.min(pts[:, 2])),
        "ref_ground_clearance_m": None if not math.isfinite(ground) else float(ground),
        "ref_cylinder_clearance_m": None if not np.isfinite(cyl).any() else float(np.min(cyl)),
        "ref_surface_clearance_m": None if not math.isfinite(surface) else surface,
    }


def geom_gate_sees_ground() -> dict:
    """几何门的净空含地面。0.40 m 悬停会被 0.43 m 门槛拒，0.50 m 会过，没有另设 z_min。"""
    from yopo_l0.policy import reference_surface_clearance
    from yopo_l0.scene import Cylinder, Ground
    from yopo_swarm_core.plan import hold_traj

    ground = Ground(0.0)
    far = Cylinder("far", (30.0, 0.0), 0.2, 0.0, 2.5)
    low = hold_traj(np.array([0.0, 0.0, 0.40]), 0.0, 0.2)
    high = hold_traj(np.array([0.0, 0.0, 0.50]), 0.0, 0.2)
    low_with = float(reference_surface_clearance(low, (far, ground)))
    low_without = float(reference_surface_clearance(low, (far,)))
    passed = float(reference_surface_clearance(high, (far, ground)))
    includes = low_with < 1.0 and low_without > 10.0
    separate = not (abs(low_with - 0.40) < 1e-6 and abs(passed - 0.50) < 1e-6)
    return {
        "low_with_ground_m": low_with,
        "low_without_ground_m": low_without,
        "pass_clearance_m": passed,
        "probe_low_z_m": 0.40,
        "probe_pass_z_m": 0.50,
        "limit_m": GEOM_LIMIT_M,
        "includes_ground": bool(includes),
        "low_rejected": bool(low_with + 1e-12 < GEOM_LIMIT_M),
        "pass_accepted": bool(passed + 1e-12 >= GEOM_LIMIT_M),
        "separate_z_min": bool(separate),
    }


def _on_tenth(t: float) -> bool:
    return abs(float(t) * 10.0 - round(float(t) * 10.0)) < 1e-6


def select_preimpact_ticks(ticks, drone: str, pre_s: float = PREIMPACT_S) -> list[dict]:
    rows = [row for row in ticks if row.get("uav") == drone]
    if not rows:
        return []
    t_hit = float(rows[-1]["t"])
    window = [row for row in rows if float(row["t"]) >= t_hit - float(pre_s) - 1e-12]
    if not window:
        window = rows[-1:]
    kept = [row for row in window if _on_tenth(float(row["t"]))]
    if not kept or kept[-1] is not window[-1]:
        kept.append(window[-1])
    return kept


def _latest_plan(plans: list[dict], t: float) -> dict | None:
    picked = None
    for row in plans:
        if float(row["t"]) <= t + 1e-9:
            picked = row
        else:
            break
    return picked


def sample_from_tick(row: dict, query: NearQuery, cam, plans: list[dict]) -> dict:
    p = np.asarray(row["p"], dtype=float).reshape(3)
    v = np.asarray(row.get("v") or (0.0, 0.0, 0.0), dtype=float).reshape(3)
    yaw = float(row.get("yaw") or 0.0)
    near = query.nearest(p)
    origin = camera_origin(p, yaw, cam)
    coverage = vfov_coverage(cam, origin, _ground_z(query))
    bearing = {"closest_in_front": False, "closest_elevation_deg": None, "closest_in_vfov": False}
    body = None
    if near.get("closest_enu") is not None:
        bearing = point_bearing(cam, origin, yaw, near["closest_enu"])
        body = body_delta(p, yaw, near["closest_enu"])
    plan = _latest_plan(plans, float(row["t"]))
    sample = {
        "t": float(row["t"]),
        "p": [float(x) for x in p],
        "v": [float(x) for x in v],
        "yaw_rad": yaw,
        "attitude_pitch_rad": 0.0,
        "flight_path_pitch_rad": flight_path_pitch(v),
        "camera_uses_pitch": False,
        "nearest_kind": near["nearest_kind"],
        "nearest_raw_kind": near["nearest_raw_kind"],
        "nearest_name": near["nearest_name"],
        "nearest_distance_m": near["nearest_distance_m"],
        "closest_z_m": near.get("closest_z_m"),
        "cylinder_clearance_m": _finite(float(query.cylinder_distances(p)[0])),
        "ground_clearance_m": _finite(query.ground_clearance(p)),
        "closest_body_xyz": None if body is None else [float(x) for x in body],
        "closest_body_z_m": None if body is None else float(body[2]),
        "altitude_m": float(p[2]),
        "plan_t": None if plan is None else float(plan["t"]),
        **bearing,
        **coverage,
    }
    for key in _PLAN_KEYS:
        sample[key] = None if plan is None else plan.get(key)
    return sample


def diagnosis_from_run(card, ticks, plans, *, drone: str = DIAG_DRONE, pre_s: float = PREIMPACT_S) -> dict:
    from yopo_l0.sensor import CameraConfig

    query = NearQuery(card.obstacles)
    cam = CameraConfig.from_card(card.card["camera"])
    ordered = sorted(plans or [], key=lambda item: float(item["t"]))
    samples = [sample_from_tick(row, query, cam, ordered) for row in select_preimpact_ticks(ticks, drone, pre_s)]
    return {
        "drone": drone,
        "preimpact_s": float(pre_s),
        "samples": samples,
    }


def collision_flags(result: dict, ticks, query: NearQuery, body: float = BODY_RADIUS_M) -> dict:
    outcome = result.get("outcome")
    reason = str(result.get("reason") or "")
    pillar = 0
    ground = 0
    if outcome == "COLLISION" and "TRUTH_POSITION" in reason:
        drone = reason.split(":", 1)[0]
        rows = [row for row in ticks if row.get("uav") == drone]
        if rows:
            p = np.asarray(rows[-1]["p"], dtype=float)
            cyl = float(query.cylinder_distances(p)[0])
            box = query.box_clearance(p)
            other = min((float(ob.distance(p)) for ob in query.others), default=math.inf)
            if min(cyl, box, other) <= body + 1e-9:
                pillar = 1
            if query.ground_clearance(p) <= body + 1e-9:
                ground = 1
    return {
        "pillar_collision": pillar,
        "ground_collision": ground,
        "inter_drone_collision": int(outcome == "COLLISION" and "INTER_DRONE" in reason),
        "timeout": int(outcome == "TIMEOUT"),
    }


def _pitch_deg(samples: list[dict]) -> float | None:
    vals = [float(row["flight_path_pitch_rad"]) for row in samples if row.get("flight_path_pitch_rad") is not None]
    if not vals:
        return None
    return float(math.degrees(min(vals)))


def explain_blindspot(cases: list[dict], camera_info: dict, gate_probe: dict) -> dict:
    """用撞点样本回答：为什么深度图被记成没有障碍。"""
    impacts = []
    for case in cases:
        samples = case.get("samples") or []
        if samples:
            impacts.append((case, samples[-1]))
    cam_level = (not camera_info.get("camera_uses_pitch")) and (not camera_info.get("state_has_pitch"))
    includes = bool(gate_probe.get("includes_ground"))
    separate = bool(gate_probe.get("separate_z_min"))
    nadir_out = bool(impacts) and all(not row.get("nadir_in_vfov") for _case, row in impacts)
    images_nonempty = bool(impacts) and all(int(row.get("depth_n_valid") or 0) > 0 for _case, row in impacts)
    cylinders_unseen = bool(impacts) and all(row.get("ec5_cylinder_in_depth") is False for _case, row in impacts)
    all_ground = bool(impacts) and all(row.get("nearest_kind") == "Ground" for _case, row in impacts)
    causes = {
        "attitude_pitch_insufficient": False if cam_level else None,
        "vfov_misses_body_nadir": nadir_out,
        "geom_ignores_ground": not includes,
        "geom_checks_ground": includes,
        "separate_z_min": separate,
        "depth_flag_targets_cylinder_not_image": bool(all_ground and cylinders_unseen and images_nonempty),
    }
    if not impacts:
        text = "没有撞点样本，回答不了深度图为什么显示没有障碍。"
        return {"headline": text, "text": text, "causes": causes, "per_case": []}
    kinds = sorted({row.get("nearest_kind") or "—" for _case, row in impacts})
    if all_ground:
        headline = f"{len(impacts)} 次撞点的最近表面都是 Ground，不是低处树干。"
    elif kinds == ["低处树干"]:
        headline = f"{len(impacts)} 次撞点的最近表面都是低处树干。"
    else:
        headline = f"{len(impacts)} 次撞点的最近表面是 {'、'.join(kinds)}。"
    parts = [headline]
    if cam_level:
        pitches = [_pitch_deg(case.get("samples") or []) for case, _row in impacts]
        pitched = [v for v in pitches if v is not None]
        if pitched:
            pitch_s = f"撞前 {PREIMPACT_S:.0f} s 轨迹俯仰最低 {min(pitched):.1f}°"
        else:
            pitch_s = "撞前水平速度几乎为 0，没有轨迹俯仰"
        parts.append(
            "不是俯仰不够。点质量 State 没有俯仰，DepthCamera.capture 只收 position、yaw、stamp，"
            f"姿态俯仰记 0。{pitch_s}，这个角不转相机，光轴保持水平。"
        )
    else:
        parts.append("相机或状态里出现了俯仰参数，不能再用「光轴保持水平」来解释。")
    if nadir_out:
        heights = [float(row["camera_height_above_ground_m"]) for _case, row in impacts if row.get("camera_height_above_ground_m") is not None]
        enters = [float(row["ground_enters_horizontal_m"]) for _case, row in impacts if row.get("ground_enters_horizontal_m") is not None]
        height_s = "—" if not heights else f"{min(heights):.3f}–{max(heights):.3f}" if max(heights) - min(heights) > 5e-4 else f"{heights[0]:.3f}"
        enter_s = "—" if not enters else f"{min(enters):.3f}–{max(enters):.3f}" if max(enters) - min(enters) > 5e-4 else f"{enters[0]:.3f}"
        parts.append(
            f"VFOV {impacts[0][1].get('vfov_deg')}°，下沿 {impacts[0][1].get('lower_edge_elevation_deg')}°。"
            "正下方地面的俯角是 90°，nadir 不在视场内。"
            f"撞点相机离地 {height_s} m，前方地面要到水平距离 {enter_s} m 才进入下沿。"
            "机体正下方的接触点在光轴后方或正下，不在这张前向图里。"
        )
    if includes and not separate:
        parts.append(
            "track_clearance=geom 检查了 Ground，不是只查柱或盒。"
            f"远处有一根柱时，0.40 m 悬停的表面净空是 {float(gate_probe['low_with_ground_m']):.3f} m，"
            f"去掉地面后是 {float(gate_probe['low_without_ground_m']):.3f} m。"
            f"0.40 m 低于门槛 {GEOM_LIMIT_M:.2f} m，会被拒；0.50 m 悬停的净空是 "
            f"{float(gate_probe['pass_clearance_m']):.3f} m，门会放行。"
            "净空等于离地高度，没有单独的 z_min。门看的是参考轨迹采样，不是机体真值。"
        )
    elif not includes:
        parts.append("几何门的表面净空没有随地面下降，相当于没查 Ground。")
    else:
        parts.append("几何门把地面算进去了，但净空不等于离地高度，不能排除另有一道 z_min。")
    per_case = []
    lines = []
    for case, row in impacts:
        ground_m = row.get("ground_clearance_m")
        cyl_m = row.get("cylinder_clearance_m")
        ref_g = row.get("ref_ground_clearance_m")
        code = row.get("plan_code")
        pixels = row.get("ground_pixels")
        seen = row.get("ec5_cylinder_in_depth")
        if int(row.get("depth_n_valid") or 0) <= 0:
            depth_s = "深度图没有有效回波。"
        elif seen is False:
            depth_s = (
                f"深度图有回波，最小有效深度 {_num(row.get('depth_min_valid_m'))} m。"
                "E-C5 的「没有」是 depth_sees(最近直立圆柱) 为假，不是整幅图没有障碍。"
            )
        elif seen is True:
            depth_s = "最近直立圆柱落在深度图里，和 E-C5 记下的「没有」不一致。"
        else:
            depth_s = "这次没有圆柱可供 depth_sees。"
        if pixels is None:
            pixel_s = ""
        elif int(pixels) > 0:
            pixel_s = f"隔 {GROUND_PIXEL_STRIDE} 像素能数到 {int(pixels)} 个地面回波。"
        else:
            pixel_s = f"隔 {GROUND_PIXEL_STRIDE} 像素没有落到地面上的回波，下沿可能被树干挡住。"
        if code == "CORRIDOR_EMPTY_HOVER":
            gate_s = (
                f"撞前规划码是 `CORRIDOR_EMPTY_HOVER`，刹车轨迹不经过几何门。"
                f"这条刹车参考的地面净空是 {_num(ref_g)} m，真值地面距离 {_num(ground_m)} m。"
            )
        elif code == "OK" and ref_g is not None and float(ref_g) + 1e-9 >= GEOM_LIMIT_M and ground_m is not None and float(ground_m) <= BODY_RADIUS_M + 1e-9:
            gate_s = (
                f"最后一条 OK 参考的地面净空 {_num(ref_g)} m ≥ {GEOM_LIMIT_M:.2f} m，门放行；"
                f"真值地面距离 {_num(ground_m)} m ≤ {BODY_RADIUS_M:.2f} m。差在跟踪。"
            )
        else:
            gate_s = (
                f"撞前规划码 `{code or '—'}`，参考地面净空 {_num(ref_g)} m，"
                f"真值地面距离 {_num(ground_m)} m，圆柱表面 {_num(cyl_m)} m。"
            )
        brake_refs = [
            float(sample["ref_ground_clearance_m"])
            for sample in (case.get("samples") or [])
            if sample.get("plan_code") == "CORRIDOR_EMPTY_HOVER" and sample.get("ref_ground_clearance_m") is not None
        ]
        if brake_refs and min(brake_refs) + 1e-12 < GEOM_LIMIT_M:
            gate_s += (
                f"撞前 {PREIMPACT_S:.0f} s 内刹停参考的地面净空最低 {_num(min(brake_refs))} m，"
                "几何门不检查这条刹车轨迹。"
            )
        line = (
            f"{arm_label(case['arm'])} seed {case['seed']} {case.get('drone') or DIAG_DRONE}："
            f"最近表面是 {row.get('nearest_kind')}（{row.get('nearest_name') or '—'}），"
            f"{depth_s}{pixel_s}{gate_s}"
        )
        lines.append(line)
        per_case.append({
            "scene": case.get("scene"),
            "arm": case.get("arm"),
            "seed": case.get("seed"),
            "drone": case.get("drone"),
            "nearest_kind": row.get("nearest_kind"),
            "ground_clearance_m": ground_m,
            "cylinder_clearance_m": cyl_m,
            "flight_path_pitch_deg_min": _pitch_deg(case.get("samples") or []),
            "attitude_pitch_rad": 0.0,
            "nadir_in_vfov": row.get("nadir_in_vfov"),
            "closest_in_vfov": row.get("closest_in_vfov"),
            "depth_min_valid_m": row.get("depth_min_valid_m"),
            "depth_n_valid": row.get("depth_n_valid"),
            "ground_pixels": pixels,
            "ec5_cylinder_in_depth": seen,
            "ref_ground_clearance_m": ref_g,
            "ref_surface_clearance_m": row.get("ref_surface_clearance_m"),
            "plan_code": code,
            "geom_gate_active": row.get("geom_gate_active"),
        })
    parts.extend(lines)
    return {"headline": headline, "text": "\n\n".join(parts), "causes": causes, "per_case": per_case}


def _finite(value: float) -> float | None:
    if value is None or not math.isfinite(float(value)):
        return None
    return float(value)


def _jsonable(value):
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        return float(value)
    return value


def _partial_path(scene: str, arm: str, seed: int) -> Path:
    return PARTIAL / f"{scene}-{arm}-{int(seed)}.json"


def _write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(path)


def install_trace(card, *, diagnose: bool):
    import yopo_l0.policy as policy_mod
    from yopo_l0.sensor import CameraConfig

    previous = policy_mod.POLICIES["upstream"]
    query = NearQuery(card.obstacles)
    cam = CameraConfig.from_card(card.card["camera"])

    class TraceUpstream(previous):
        brake_plans = 0
        hover_plans = 0
        instances: list = []
        want_depth = False
        focus = DIAG_DRONE
        query = None
        cam = None

        def __init__(self, ctx):
            super().__init__(ctx)
            self.drone_id = ctx.owner
            self.plans: list[dict] = []
            TraceUpstream.instances.append(self)

        def plan(self, obs):
            gate = bool(self._track_geom_now()) if self._track_on else False
            out = super().plan(obs)
            if self.last_code == "CORRIDOR_EMPTY_HOVER":
                type(self).hover_plans += 1
                if self._empty_brake:
                    type(self).brake_plans += 1
            if type(self).want_depth and self.drone_id == type(self).focus:
                self.plans.append(self._snap(obs, gate))
                if len(self.plans) > PLAN_KEEP:
                    del self.plans[:-PLAN_KEEP]
            return out

        def _snap(self, obs, gate: bool) -> dict:
            state = obs.state
            rec = {
                "t": float(obs.now),
                "plan_code": self.last_code,
                "geom_gate_active": bool(gate),
                "n_neighbors": len(self._neighbors),
                "ref_is_brake": bool(getattr(self.plan_obj, "brake", False)),
            }
            traj = None if self.plan_obj is None else getattr(self.plan_obj, "traj", None)
            if traj is not None:
                rec.update(split_reference(traj, type(self).query, self.ctx.obstacles))
            frame = getattr(obs, "frame", None)
            if frame is not None and getattr(frame, "depth", None) is not None:
                rec.update(analyze_depth(
                    frame,
                    np.asarray(state.p, dtype=float),
                    float(state.yaw),
                    type(self).cam,
                    type(self).query,
                ))
            return _jsonable(rec)

    TraceUpstream.want_depth = bool(diagnose)
    TraceUpstream.focus = DIAG_DRONE
    TraceUpstream.query = query
    TraceUpstream.cam = cam
    TraceUpstream.brake_plans = 0
    TraceUpstream.hover_plans = 0
    TraceUpstream.instances = []
    policy_mod.POLICIES["upstream"] = TraceUpstream
    return TraceUpstream, previous


def restore_trace(previous) -> None:
    import yopo_l0.policy as policy_mod

    policy_mod.POLICIES["upstream"] = previous


def _metrics(scene: str, arm: str, seed: int, card, out, query: NearQuery, brakes: int, hovers: int, worker_s: float) -> dict:
    res = out.result
    got = float(res["safe_radius_m"])
    if abs(got - UPSTREAM_SAFE_RADIUS_M) > 1e-9:
        raise RuntimeError(f"safe_radius 读回 {got}，本票五档都保持 0.05 m")
    if bool(res["upstream_deviation"]) != arm_deviates(arm):
        raise RuntimeError(f"{arm} 偏离标记 {res['upstream_deviation']} 与开关不一致")
    if arm == "off":
        if "track_clearance" in res or "empty_brake" in res:
            raise RuntimeError("全关臂不应写出 track_clearance / empty_brake")
    elif res.get("empty_brake") is not True:
        raise RuntimeError(f"{arm} 没有打开 empty_brake")
    if arm in ("pred", "geom", "geom_multi"):
        if res.get("track_clearance") is not True or res.get("track_clearance_mode") != arm:
            raise RuntimeError(f"{arm} 的 track_clearance 读回 {res.get('track_clearance_mode')}")
        if abs(float(res.get("track_margin_m")) - MARGIN_M) > 1e-12:
            raise RuntimeError(f"{arm} margin 读回 {res.get('track_margin_m')}")
    flags = collision_flags(res, out.ticks, query)
    cyl = query.cylinder_distances(np.asarray([row["p"] for row in out.ticks], dtype=float)) if out.ticks else np.array([])
    cyl_m = None if len(cyl) == 0 or not np.isfinite(cyl).any() else float(np.min(cyl))
    return {
        "scene": scene,
        "arm": arm,
        "seed": int(seed),
        "upstream_deviation": bool(res["upstream_deviation"]),
        "report_only": True,
        "frozen": False,
        "safe_radius_m": got,
        "decoder_radius_m": decoder_radius_m(arm),
        "track_margin_m": None if arm in ("off", "base") else MARGIN_M,
        "track_clearance_mode": None if arm in ("off", "base") else arm,
        "empty_brake": arm != "off",
        "scene_card_hash": res["scene_card_hash"],
        "file_hash": _file_hash(scene),
        "outcome": res["outcome"],
        "reason": str(res["reason"]),
        "arrived": int(res["arrived"]),
        "n_drones": int(res["n_drones"]),
        "arrival_time_s": res["arrival_time_s"],
        "min_clearance_m": res["min_obstacle_distance_m"],
        "min_cylinder_clearance_m": cyl_m,
        "min_inter_drone_distance_m": res["min_inter_drone_distance_m"],
        "brake_plans": int(brakes),
        "corridor_empty_hover_plans": int(hovers),
        "sim_duration_s": res["sim_duration_s"],
        "wall_time_s": res["wall_time_s"],
        "worker_s": worker_s,
        **flags,
    }


def _execute(scene: str, arm: str, seed: int, *, diagnose: bool) -> dict:
    from yopo_l0.runner import RunConfig, run

    card = build_card(scene, arm)
    query = NearQuery(card.obstacles)
    cls, previous = install_trace(card, diagnose=diagnose)
    t0 = time.perf_counter()
    try:
        out = run(card, RunConfig(seed=int(seed), shadow=True))
        brakes = int(cls.brake_plans)
        hovers = int(cls.hover_plans)
        plans = []
        if diagnose:
            for inst in cls.instances:
                if inst.drone_id == DIAG_DRONE:
                    plans = list(inst.plans)
    finally:
        restore_trace(previous)
        cls.instances = []
    row = _metrics(scene, arm, seed, card, out, query, brakes, hovers, time.perf_counter() - t0)
    if diagnose:
        diag = diagnosis_from_run(card, out.ticks, plans, drone=DIAG_DRONE)
        row["diagnosis"] = diag
        impact = (diag.get("samples") or [None])[-1]
        kind = None if impact is None else impact.get("nearest_kind")
    else:
        kind = None
    gap = row["min_inter_drone_distance_m"]
    gap_s = "—" if gap is None else f"{float(gap):.3f}"
    extra = "" if kind is None else f" nearest={kind}"
    print(
        f"{scene} {arm_label(arm)} seed={seed} {row['outcome']} "
        f"{row['arrived']}/{row['n_drones']} pillar={row['pillar_collision']} "
        f"ground={row['ground_collision']} inter={row['inter_drone_collision']} "
        f"gap={gap_s} clear={_num(row['min_clearance_m'])} "
        f"brake={row['brake_plans']} sim={float(row['sim_duration_s']):.2f} "
        f"wall={float(row['wall_time_s']):.1f}s{extra}",
        flush=True,
    )
    return row


def run_one(scene: str, arm: str, seed: int, *, diagnose: bool = False, resume: bool = True) -> dict:
    dest = _partial_path(scene, arm, seed)
    if resume and dest.is_file():
        return json.loads(dest.read_text(encoding="utf-8"))
    row = _jsonable(_execute(scene, arm, seed, diagnose=diagnose))
    _write_json(dest, row)
    return row


def _worker(spec: tuple) -> dict:
    try:
        os.nice(10)
    except OSError:
        pass
    scene, arm, seed, diagnose, resume = spec
    return run_one(str(scene), str(arm), int(seed), diagnose=bool(diagnose), resume=bool(resume))


def job_specs(scenes, arms, seeds, *, diagnose: bool = True, resume: bool = True) -> list[tuple]:
    specs = []
    if diagnose:
        for arm in DIAG_ARMS:
            if arm not in arms:
                continue
            for seed in DIAG_SEEDS:
                specs.append((DIAG_SCENE, arm, int(seed), True, bool(resume)))
    for arm in arms:
        for scene in scenes:
            if scene == DIAG_SCENE:
                continue
            for seed in seeds:
                specs.append((scene, arm, int(seed), False, bool(resume)))
    return specs


def _run_group(specs: list[tuple], jobs: int) -> list[dict]:
    if jobs <= 1 or len(specs) <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)


def _num(value, nd=3) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{nd}f}"


def _deg(value) -> str:
    if value is None:
        return "—"
    return f"{math.degrees(float(value)):.1f}"


def _yn(value) -> str:
    if value is None:
        return "—"
    return "是" if value else "否"


def _rng(values: list[float]) -> str:
    if not values:
        return "—"
    lo, hi = min(values), max(values)
    if abs(lo - hi) < 5e-4:
        return f"{lo:.3f}"
    return f"{lo:.3f}–{hi:.3f}"


def _group(rows: list[dict], scene: str, arm: str) -> list[dict]:
    return [row for row in rows if row.get("scene") == scene and row.get("arm") == arm]


def render_summary(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | 偏离上游 | 到达 | 撞柱 | 撞地 | 机间碰撞 | 超时 | 到达时间 s | CORRIDOR_EMPTY_HOVER 刹停 | 最小净空 m | 最小圆柱净空 m |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    scenes = [scene for scene in DENSE_ORDER if any(row.get("scene") == scene for row in rows)]
    for scene in scenes:
        for arm in ARMS:
            group = _group(rows, scene, arm)
            if not group:
                continue
            drones = sum(int(row["n_drones"]) for row in group)
            arrived = sum(int(row["arrived"]) for row in group)
            pillar = sum(int(row.get("pillar_collision") or 0) for row in group)
            ground = sum(int(row.get("ground_collision") or 0) for row in group)
            inter = sum(int(row.get("inter_drone_collision") or 0) for row in group)
            timeout = sum(int(row.get("timeout") or 0) for row in group)
            brakes = sum(int(row.get("brake_plans") or 0) for row in group)
            times = [float(row["arrival_time_s"]) for row in group if row.get("arrival_time_s") is not None]
            clears = [float(row["min_clearance_m"]) for row in group if row.get("min_clearance_m") is not None]
            cyls = [float(row["min_cylinder_clearance_m"]) for row in group if row.get("min_cylinder_clearance_m") is not None]
            dev = "偏离上游" if arm_deviates(arm) else "否"
            lines.append(
                f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | {dev} | {arrived}/{drones} | "
                f"{pillar} | {ground} | {inter} | {timeout} | {_rng(times)} | {brakes} | "
                f"{_rng(clears)} | {_rng(cyls)} |"
            )
    return "\n".join(lines)


def render_seeds(rows: list[dict]) -> str:
    lines = [
        "| 卡 | 臂 | seed | 结果 | 原因 | 到达 | 到达时间 s | 撞柱 | 撞地 | 机间 | 刹停 | 空走廊码 | 最小净空 m | 最小圆柱净空 m | 仿真 s |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    scenes = [scene for scene in DENSE_ORDER if any(row.get("scene") == scene for row in rows)]
    for scene in scenes:
        for arm in ARMS:
            for row in sorted(_group(rows, scene, arm), key=lambda item: int(item["seed"])):
                lines.append(
                    f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | {row['seed']} | {row['outcome']} | "
                    f"`{row['reason']}` | {row['arrived']}/{row['n_drones']} | {_num(row.get('arrival_time_s'), 2)} | "
                    f"{int(row.get('pillar_collision') or 0)} | {int(row.get('ground_collision') or 0)} | "
                    f"{int(row.get('inter_drone_collision') or 0)} | {int(row.get('brake_plans') or 0)} | "
                    f"{int(row.get('corridor_empty_hover_plans') or 0)} | {_num(row.get('min_clearance_m'))} | "
                    f"{_num(row.get('min_cylinder_clearance_m'))} | {_num(row.get('sim_duration_s'), 2)} |"
                )
    return "\n".join(lines)


def render_diagnosis_tables(cases: list[dict]) -> str:
    if not cases:
        return "这次没有诊断样本。"
    summary = [
        "| 卡 | 臂 | seed | 飞机 | 最近表面 | 地面 m | 圆柱 m | 机体系 z | 轨迹俯仰° | 姿态俯仰° | 正下方在 VFOV | 最小深度 m | 地面像素 | 柱在图里 | 参考地面净空 m | 规划码 |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    blocks = []
    ordered = sorted(cases, key=lambda case: (DIAG_ARMS.index(case["arm"]) if case.get("arm") in DIAG_ARMS else 9, int(case.get("seed") or 0)))
    for case in ordered:
        samples = case.get("samples") or []
        impact = samples[-1] if samples else None
        if impact is None:
            continue
        summary.append(
            f"| {SCENE_LABEL.get(case.get('scene'), case.get('scene'))} | {arm_label(case['arm'])} | "
            f"{case['seed']} | {case.get('drone') or DIAG_DRONE} | {impact.get('nearest_kind')} | "
            f"{_num(impact.get('ground_clearance_m'))} | {_num(impact.get('cylinder_clearance_m'))} | "
            f"{_num(impact.get('closest_body_z_m'))} | {_num(_pitch_deg(case.get('samples') or []), 1)} | 0.0 | "
            f"{_yn(impact.get('closest_in_vfov'))} | {_num(impact.get('depth_min_valid_m'))} | "
            f"{'—' if impact.get('ground_pixels') is None else int(impact['ground_pixels'])} | "
            f"{_yn(impact.get('ec5_cylinder_in_depth'))} | {_num(impact.get('ref_ground_clearance_m'))} | "
            f"`{impact.get('plan_code') or '—'}` |"
        )
        lines = [
            f"### {SCENE_LABEL.get(case.get('scene'), case.get('scene'))} / {arm_label(case['arm'])} / seed {case['seed']} / {case.get('drone') or DIAG_DRONE}",
            "",
            "| t | x | y | z | vx | vy | vz | 轨迹俯仰° | 偏航° | 最近 | 距离 m | 机体系 z | 最小深度 m | VFOV 下沿° | 地面像素 | 柱在图里 | 参考地面净空 m | 码 |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
        ]
        for row in samples:
            p = row.get("p") or [None, None, None]
            v = row.get("v") or [None, None, None]
            lines.append(
                f"| {_num(row.get('t'), 2)} | {_num(p[0])} | {_num(p[1])} | {_num(p[2])} | "
                f"{_num(v[0])} | {_num(v[1])} | {_num(v[2])} | {_deg(row.get('flight_path_pitch_rad'))} | "
                f"{_deg(row.get('yaw_rad'))} | {row.get('nearest_kind')} | {_num(row.get('nearest_distance_m'))} | "
                f"{_num(row.get('closest_body_z_m'))} | {_num(row.get('depth_min_valid_m'))} | "
                f"{_num(row.get('lower_edge_elevation_deg'), 1)} | "
                f"{'—' if row.get('ground_pixels') is None else int(row['ground_pixels'])} | "
                f"{_yn(row.get('ec5_cylinder_in_depth'))} | {_num(row.get('ref_ground_clearance_m'))} | "
                f"`{row.get('plan_code') or '—'}` |"
            )
        blocks.append("\n".join(lines))
    return "\n".join(summary) + "\n\n" + "\n\n".join(blocks)


def render_markdown(payload: dict) -> str:
    rows = payload.get("rows") or []
    cases = payload.get("diagnosis") or []
    answer = payload.get("answer") or {}
    commit = payload.get("commit") or ""
    lines = [
        "# 票 E-C5c：地面盲区归因与密度 0.28 对照，只报数",
        "",
        DISCLAIMER,
        "",
        "没有改 `yopo_l0/policy.py`、`core/`、`scenes/`、`docs/tickets.md`、`README.md` 和 `scripts/regress.sh`。"
        "五档都在进程里覆盖，场景卡文件哈希不变。`safe_radius` 仍是 0.05 m。"
        "打开 E-A1、`empty_brake` 或 `track_clearance` 的臂标「偏离上游」。未冻结，不进 `scenes/suite.txt`，不设 gates。",
        "",
        f"数字在 [e-c5c-blindspot-dense-runs.json](e-c5c-blindspot-dense-runs.json)。跑数 commit 是 `{commit}`。"
        "这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。report-only。",
        "",
        "## 地面盲区",
        "",
        "回放 `scenes/ego-density-014-swap3.json` 上 seed 2 与 seed 9、`geom` 与 `geom_multi` 的 `uav1`。"
        f"表里是撞前 {PREIMPACT_S:.0f} s、每 0.1 s 一拍，再加上撞上的那一拍。"
        "轨迹俯仰是 `atan2(vz, 水平速度)`。姿态俯仰是 0：点质量没有这个状态，深度相机也不读它。"
        "机体系 z 是接触点相对机体原点的上向分量。VFOV 下沿是光轴以下的半角，这张卡是 −30°。"
        "地面像素是整幅深度隔 2 个像素之后、离地不超过 0.15 m 且比柱和盒更近的点数。"
        "「柱在图里」沿用 E-C5 的 `depth_sees`：看的是最近直立圆柱，不看 Ground。",
        "",
        answer.get("text") or "没有诊断结论。",
        "",
        render_diagnosis_tables(cases),
        "",
        "## 密度 0.28",
        "",
        "卡是 `scenes/ego-density-028-swap3.json`（3 机）和 `scenes/ego-density-028-solo.json`（1 机），seeds 0–9。"
        "几何是 `scenes/ego-density-028.json`，112 根圆柱。五档参数与 E-C5b 相同。"
        "到达按架次。撞柱、撞地、机间碰撞按运行器第一次停下的那一下，一次运行里柱和地面可以同时进机体半径。"
        "撞柱是非地面表面（这些卡里是圆柱）距离 ≤ 0.35 m。撞地是到 Ground 的距离 ≤ 0.35 m。"
        "到达时间只在全员到达时有值，汇总是这些种子的范围。"
        "`CORRIDOR_EMPTY_HOVER` 刹停只在 `empty_brake` 打开时计数。全关臂的同名规划码是原地悬停，刹停记 0，原始次数在分种子表的「空走廊码」。"
        "最小净空是 `min_obstacle_distance_m`，含地面。最小圆柱净空不含地面。单机没有机间距离。",
        "",
        render_summary(rows),
        "",
        "## 分种子",
        "",
        render_seeds(rows),
        "",
    ]
    return "\n".join(lines)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _check() -> int:
    from yopo_l0.policy import UPSTREAM_SAFE_RADIUS_M as radius
    from yopo_l0.policy import read_empty_brake, read_track_clearance

    if abs(float(radius) - 0.05) > 1e-12 or read_track_clearance({}) or read_empty_brake({}):
        raise SystemExit("默认开关被改了")
    info = camera_attitude()
    if info["camera_uses_pitch"] or info["state_has_pitch"] or "pitch" in info["capture_params"]:
        raise SystemExit(f"深度相机或状态多了俯仰：{info}")
    probe = geom_gate_sees_ground()
    if not probe["includes_ground"] or probe["separate_z_min"]:
        raise SystemExit(f"几何门的地面检查变了：{probe}")
    if not probe["low_rejected"] or not probe["pass_accepted"]:
        raise SystemExit(f"0.43 m 门槛和地面净空对不上：{probe}")
    before = {rel: (ROOT / rel).read_bytes() for rel in SCENES.values()}
    for scene in SCENES:
        off = build_card(scene, "off")
        if off.hash != _file_hash(scene):
            raise SystemExit(f"{scene} 全关臂哈希与场景文件不一致")
        if "policy_params" in off.card:
            raise SystemExit(f"{scene} 全关臂写了 policy_params")
        for arm, mode in (("pred", "pred"), ("geom", "geom"), ("geom_multi", "geom_multi")):
            card = build_card(scene, arm)
            up = card.card["policy_params"]["upstream"]
            if up.get("track_clearance") is not True or up.get("track_clearance_mode") != mode:
                raise SystemExit(f"{scene} {mode} 没写进进程内卡")
            if abs(float(up["track_margin_m"]) - MARGIN_M) > 1e-12:
                raise SystemExit(f"{scene} {mode} margin 不是 0.08")
            if "safe_radius" in up or up.get("empty_brake") is not True or up.get("inter_drone_filter") is not True:
                raise SystemExit(f"{scene} {mode} 的 base 开关不对")
            if abs(float(up["inter_drone_threshold_m"]) - 1.0) > 1e-12 or up.get("inter_drone_penalty") != 0:
                raise SystemExit(f"{scene} E-A1 不是阈值 1.0 m、惩罚 0")
        base = build_card(scene, "base")
        up = base.card["policy_params"]["upstream"]
        if "track_clearance" in up or "safe_radius" in up:
            raise SystemExit(f"{scene} base 臂多写了 track_clearance 或 safe_radius")
    if {rel: (ROOT / rel).read_bytes() for rel in before} != before:
        raise SystemExit("检查场景卡时改写了文件")
    suite = (ROOT / "scenes" / "suite.txt").read_text(encoding="utf-8")
    if "e-c5c" in suite or "ego-density-028" in suite:
        raise SystemExit("scenes/suite.txt 被写进了本票的卡")
    print("e-c5c check ok")
    return 0


def _head_commit() -> str:
    import subprocess

    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def assemble_payload(rows: list[dict], *, commit: str, jobs: int, scenes, arms, seeds) -> dict:
    diagnosis = []
    dense = []
    for row in rows:
        if row.get("diagnosis") and row.get("scene") == DIAG_SCENE and row.get("arm") in DIAG_ARMS and int(row.get("seed")) in DIAG_SEEDS:
            case = {
                "scene": row["scene"],
                "arm": row["arm"],
                "seed": int(row["seed"]),
                "drone": DIAG_DRONE,
                "outcome": row.get("outcome"),
                "reason": row.get("reason"),
                "arrived": row.get("arrived"),
                "n_drones": row.get("n_drones"),
                "pillar_collision": row.get("pillar_collision"),
                "ground_collision": row.get("ground_collision"),
                "min_clearance_m": row.get("min_clearance_m"),
                "min_cylinder_clearance_m": row.get("min_cylinder_clearance_m"),
                "brake_plans": row.get("brake_plans"),
                "sim_duration_s": row.get("sim_duration_s"),
                "samples": row["diagnosis"].get("samples") or [],
            }
            diagnosis.append(case)
        elif row.get("scene") in DENSE_ORDER:
            dense.append({k: v for k, v in row.items() if k != "diagnosis"})
    diagnosis.sort(key=lambda case: (DIAG_ARMS.index(case["arm"]), int(case["seed"])))
    dense.sort(key=lambda row: (DENSE_ORDER.index(row["scene"]), ARMS.index(row["arm"]), int(row["seed"])))
    camera = camera_attitude()
    probe = geom_gate_sees_ground()
    answer = explain_blindspot(diagnosis, camera, probe)
    return {
        "schema": SCHEMA,
        "disclaimer": DISCLAIMER,
        "commit": commit,
        "report_only": True,
        "frozen": False,
        "upstream_deviation": True,
        "scenes": list(scenes),
        "arms": list(arms),
        "seeds": [int(s) for s in seeds],
        "jobs": int(jobs),
        "margin_m": MARGIN_M,
        "body_radius_m": BODY_RADIUS_M,
        "geom_limit_m": GEOM_LIMIT_M,
        "low_trunk_z_m": LOW_TRUNK_Z_M,
        "camera": camera,
        "geom_gate": probe,
        "geometry_sha256": {
            "scenes/ego-density-014.json": _sha256(ROOT / "scenes" / "ego-density-014.json"),
            "scenes/ego-density-028.json": _sha256(ROOT / "scenes" / "ego-density-028.json"),
        },
        "answer": answer,
        "diagnosis": diagnosis,
        "rows": dense,
        "note": DISCLAIMER,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scenes", default=",".join(DENSE_ORDER))
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--out", default="docs/research/e-c5c-blindspot-dense-runs.json")
    ap.add_argument("--md", default="docs/research/e-c5c-blindspot-dense.md")
    ap.add_argument("--no-resume", action="store_true")
    ap.add_argument("--no-diag", action="store_true")
    ap.add_argument("--diag-only", action="store_true")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)
    if args.check:
        return _check()
    if args.jobs not in (1, 2, 3, 4):
        raise SystemExit("jobs 只能是 1 到 4")
    if args.no_diag and args.diag_only:
        raise SystemExit("--no-diag 和 --diag-only 不能一起用")
    scenes = [] if args.diag_only else [s for s in args.scenes.split(",") if s]
    arms = [s for s in args.arms.split(",") if s]
    seeds = [int(s) for s in args.seeds.split(",") if s != ""]
    unknown = [s for s in scenes if s not in SCENES] + [s for s in arms if s not in ARMS]
    if unknown:
        raise SystemExit(f"未知场景或臂：{unknown}")
    _check()
    resume = not args.no_resume
    specs = job_specs(scenes, arms, seeds, diagnose=not args.no_diag, resume=resume)
    print(f"e-c5c start n={len(specs)} jobs={args.jobs} resume={resume}", flush=True)
    ran = _run_group(specs, int(args.jobs))
    by_key = {(row["scene"], row["arm"], int(row["seed"])): row for row in ran}
    rows = [by_key[key] for key in sorted(by_key, key=lambda item: (str(item[0]), ARMS.index(item[1]), item[2]))]
    payload = assemble_payload(rows, commit=_head_commit(), jobs=int(args.jobs), scenes=scenes, arms=arms, seeds=seeds)
    dest = Path(args.out)
    md_path = Path(args.md)
    if not dest.is_absolute():
        dest = ROOT / dest
    if not md_path.is_absolute():
        md_path = ROOT / md_path
    _write_json(dest, payload)
    md_path.parent.mkdir(parents=True, exist_ok=True)
    md_path.write_text(render_markdown(payload), encoding="utf-8")
    print(payload["answer"]["headline"])
    print(render_summary(payload["rows"]))
    print(f"写入 {dest}")
    print(f"写入 {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
