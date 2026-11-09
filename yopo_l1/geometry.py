"""只放行 L1 已经能表达的几何。

``sim/px4/g2-scene.json``（scenes/g2-scene.json 是它的原样副本）的障碍是轴对齐盒子：
``name`` / ``lo`` / ``hi``。圆柱、网格、球，以及 lo/hi 对不上的盒子，现有 Gazebo 世界没有对应物。
地面在 z=0。机体半径沿用 g2_run.py 冻结的 0.35 m，卡上写了别的数就拒绝，避免抽检改安全清单。
"""

from __future__ import annotations

import math

from yopo_l0.scene import SceneCard

from .errors import L1GeometryError, L1LaunchError
from .launch_contract import FROZEN_BODY_RADIUS_M, FROZEN_SAFETY, L1_POLICIES

_GEOM_KEYS = {"world", "goal_enu", "boxes", "cylinders"}
_BOX_KEYS = {"name", "lo", "hi"}


def _vec3(value, what: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise L1GeometryError(f"{what} 必须是 3 个数")
    out = []
    for x in value:
        try:
            fx = float(x)
        except (TypeError, ValueError) as exc:
            raise L1GeometryError(f"{what} 必须是 3 个数") from exc
        if not math.isfinite(fx):
            raise L1GeometryError(f"{what} 必须是有限数")
        out.append(fx)
    return out


def check_card(card: SceneCard, policy: str) -> dict:
    """返回写入 scene.json 的对象：world、goal_enu、boxes。"""
    if policy not in L1_POLICIES:
        raise L1LaunchError(f"L1 抽检策略只能是 upstream 或 certified，收到 {policy}")
    c = card.card
    geom = c.get("geometry")
    if not isinstance(geom, dict):
        raise L1GeometryError("场景卡几何没有内联成对象，L1 无法对照 sim/px4/g2-scene.json")
    extra = set(geom) - _GEOM_KEYS
    if extra:
        raise L1GeometryError(
            "L1 现有世界只表达 sim/px4/g2-scene.json 的轴对齐盒子，不认识字段 " + ", ".join(sorted(extra))
        )
    cylinders = geom.get("cylinders") or []
    if cylinders:
        names = [str(cy.get("name", "?")) for cy in cylinders[:8]]
        more = "…" if len(cylinders) > 8 else ""
        raise L1GeometryError(
            "L1 现有 Gazebo 世界只表达轴对齐盒子（sim/px4/g2-scene.json 的 name/lo/hi）。"
            f"本卡还有圆柱：{', '.join(names)}{more}"
        )
    ground = c.get("ground_z", 0.0)
    if float(ground) != 0.0:
        raise L1GeometryError(f"L1 地面在 z=0，本卡 ground_z={ground}")
    radius = c.get("criteria", {}).get("body_radius_m")
    if radius is None or not math.isclose(float(radius), FROZEN_BODY_RADIUS_M):
        raise L1GeometryError(
            f"机体半径 {radius} 和 g2_run.py 冻结的 {FROZEN_BODY_RADIUS_M} 不一致。抽检不改安全清单"
        )
    given = (c.get("policy_params") or {}).get("certified") or {}
    unknown = set(given) - set(FROZEN_SAFETY)
    if unknown:
        raise L1GeometryError(
            "场景卡要改 g2_run.py 里没有的安全清单项：" + ", ".join(sorted(unknown)) + "。抽检不改这些数"
        )
    for key, frozen in FROZEN_SAFETY.items():
        if key in given and not math.isclose(float(given[key]), float(frozen)):
            raise L1GeometryError(
                f"L1 安全清单数值不能改：场景卡 certified.{key}={given[key]}，g2_run.py 冻结值是 {frozen}"
            )
    boxes = []
    for b in geom.get("boxes") or []:
        if not isinstance(b, dict):
            raise L1GeometryError("boxes 里有不是对象的条目")
        odd = set(b) - _BOX_KEYS
        name = str(b.get("name", "?"))
        if odd:
            raise L1GeometryError(f"盒子 {name} 有 L1 不认识的字段 {', '.join(sorted(odd))}")
        if not isinstance(b.get("name"), str) or not b["name"]:
            raise L1GeometryError("每个盒子都要有名字")
        lo, hi = _vec3(b.get("lo"), f"盒子 {name} 的 lo"), _vec3(b.get("hi"), f"盒子 {name} 的 hi")
        if any(a > c_ for a, c_ in zip(lo, hi)):
            raise L1GeometryError(f"盒子 {name} 的 lo 超过了 hi")
        boxes.append({"name": b["name"], "lo": lo, "hi": hi})
    world = geom.get("world", card.name)
    if not isinstance(world, str) or not world:
        raise L1GeometryError("world 必须是非空字符串")
    drones = c.get("drones") or []
    if not drones:
        raise L1GeometryError("场景卡没有无人机")
    task = (c.get("task") or {}).get("type")
    if task not in ("goto", "hover"):
        raise L1GeometryError(f"L1 抽检不跑任务类型 {task}")
    missions = []
    for d in drones:
        start = _vec3(d.get("start_enu"), f"{d.get('id', '?')} 的 start_enu")
        if task == "goto":
            if "goal_enu" not in d:
                raise L1GeometryError(f"{d.get('id', '?')} 是 goto 但没有 goal_enu")
            goal = _vec3(d["goal_enu"], f"{d['id']} 的 goal_enu")
        else:
            goal = _vec3(d.get("hover_enu", d.get("start_enu")), f"{d.get('id', '?')} 的悬停点")
        missions.append({"id": d["id"], "start_enu": start, "goal_enu": goal})
    return {
        "world": world,
        "goal_enu": missions[0]["goal_enu"],
        "boxes": boxes,
        "missions": missions,
        "task": task,
    }
