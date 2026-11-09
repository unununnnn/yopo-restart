"""场景卡与几何。几何只给仿真（传感器、碰撞真值），绝不进规划器。"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Box:
    name: str
    lo: tuple[float, float, float]
    hi: tuple[float, float, float]

    def distance(self, p: np.ndarray) -> float:
        lo, hi = np.asarray(self.lo), np.asarray(self.hi)
        return float(np.linalg.norm(np.maximum(np.maximum(lo - p, 0.0), p - hi)))


@dataclass(frozen=True)
class Cylinder:
    """竖直圆柱：底面圆心 (x, y, z_lo)，半径 r，高到 z_hi。"""

    name: str
    center_xy: tuple[float, float]
    radius: float
    z_lo: float
    z_hi: float

    def distance(self, p: np.ndarray) -> float:
        dr = max(float(np.hypot(p[0] - self.center_xy[0], p[1] - self.center_xy[1])) - self.radius, 0.0)
        dz = max(self.z_lo - p[2], 0.0, p[2] - self.z_hi)
        return float(np.hypot(dr, dz))


@dataclass(frozen=True)
class Ground:
    z: float

    name: str = "ground"

    def distance(self, p: np.ndarray) -> float:
        return max(float(p[2]) - self.z, 0.0)


Obstacle = Box | Cylinder | Ground


@dataclass(frozen=True)
class SceneCard:
    path: str
    card: dict  # 场景卡原文（geometry 已内联为 dict）
    obstacles: tuple[Obstacle, ...]

    @property
    def name(self) -> str:
        return self.card["name"]

    @property
    def hash(self) -> str:
        blob = json.dumps(self.card, sort_keys=True, separators=(",", ":")).encode()
        return hashlib.sha256(blob).hexdigest()

    def min_distance(self, p: np.ndarray) -> float:
        return min(o.distance(p) for o in self.obstacles)


def parse_geometry(geom: dict, ground_z: float | None) -> tuple[Obstacle, ...]:
    obs: list[Obstacle] = [Box(b["name"], tuple(b["lo"]), tuple(b["hi"])) for b in geom.get("boxes", [])]
    obs += [
        Cylinder(c["name"], tuple(c["center_xy"]), float(c["radius"]), float(c["z_lo"]), float(c["z_hi"]))
        for c in geom.get("cylinders", [])
    ]
    if ground_z is not None:
        obs.append(Ground(float(ground_z)))
    return tuple(obs)


def load_card(path: str | Path, geometry_override: dict | None = None) -> SceneCard:
    """读场景卡。geometry_override 只给测试用（换几何）。"""
    path = Path(path)
    card = json.loads(path.read_text())
    geom = geometry_override
    if geom is None:
        geom = json.loads((path.parent / card["geometry"]).read_text())
    card = {**card, "geometry": geom}
    return SceneCard(str(path), card, parse_geometry(geom, card.get("ground_z")))
