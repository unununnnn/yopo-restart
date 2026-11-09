"""yopo-swarm（只读参考）里 L1 的启动入口和世界格式。

本环境的 GitHub 安装读不到私有仓库 https://github.com/unununnnn/yopo-swarm
（API 返回 404）。下面的路径来自该仓库 @00b85bf 已经原样抄进本仓库的文件：

- 启动文件：``docs/ref/minco-port-map.md`` 第 23 行，ys 目标
  ``ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py``
  命令：``ros2 launch yopo_swarm_px4 uavs.launch.py``
- 世界：``scenes/README.md``，``sim/px4/g2-scene.json``
  字段是 ``world``、``goal_enu``、``boxes[{name, lo, hi}]``
- 安全清单数值的出处：``yopo_l0/policy.py`` 注明 certified 默认值
  「参数同 L1 G2（g2_run.py）」。抽检不改这个文件，也不把这些数当启动参数传下去。
  命名空间规则见 ``yopo_swarm_core.uav.namespace``：``px4_[0-9]+``。
  vendored 规划日志的 owner 是 ``px4_1``。

抽检机上读得到那份 launch 文件时，参数名按 ``DeclareLaunchArgument`` 对齐。
读不到时用规范名。对不上就失败，避免把栈不认识的名字传给 ``ros2 launch``，
也避免悄悄漏掉 policy / shadow。
"""

from __future__ import annotations

import re
from pathlib import Path

from .errors import L1LaunchError

PACKAGE = "yopo_swarm_px4"
LAUNCH_FILE = "uavs.launch.py"
LAUNCH_PATH = "ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py"
WORLD_SCHEMA = "sim/px4/g2-scene.json"
SAFETY_SOURCE = "g2_run.py"

# yopo_l0/policy.py CertifiedPolicy.DEFAULTS，注释写明与 L1 g2_run.py 相同。
FROZEN_SAFETY = {
    "body_radius": 0.35,
    "tracking_margin": 0.15,
    "switch_delay": 2.0,
    "start_pad_radius": 1.2,
}
FROZEN_BODY_RADIUS_M = FROZEN_SAFETY["body_radius"]

# 规范名在前。同一个语义命中多个声明时用靠前的那个，避免把 JSON 路径绑到 Gazebo 的 world 名上。
ALIASES = {
    "scene": ("scene", "scene_file", "scene_json", "g2_scene", "world_file", "world_json", "world"),
    "seed": ("seed",),
    "policy": ("policy", "planner_policy", "strategy"),
    "shadow": ("shadow", "shadow_shield", "shadow_mode", "enable_shadow"),
    "start_enu": ("start_enu", "start", "initial_pose", "spawn_pose"),
    "goal_enu": ("goal_enu", "goal", "goal_pose"),
}

_DECLARED = re.compile(
    r"""DeclareLaunchArgument\(\s*(?:name\s*=\s*)?(['"])(?P<name>[^'"]+)\1""",
    re.M,
)

L1_POLICIES = ("upstream", "certified")


def declared_arguments(launch_py: Path) -> list[str]:
    """按出现顺序读 DeclareLaunchArgument 的名字。不执行 launch 文件（这里没有 ROS）。"""
    text = Path(launch_py).read_text(encoding="utf-8")
    names = [m.group("name") for m in _DECLARED.finditer(text)]
    if not names:
        raise L1LaunchError(f"{launch_py} 里没有 DeclareLaunchArgument，无法对齐启动参数")
    return names


def bind_launch_args(semantic: dict[str, str], declared: list[str] | None) -> dict[str, str]:
    """semantic 的键是规范名。declared 为 None 时原样返回规范名。"""
    if declared is None:
        return dict(semantic)
    have = set(declared)
    bound: dict[str, str] = {}
    missing: list[str] = []
    for key, value in semantic.items():
        name = next((alias for alias in ALIASES[key] if alias in have), None)
        if name is None:
            missing.append(f"{key}（试过 {', '.join(ALIASES[key])}）")
        else:
            bound[name] = value
    if missing:
        raise L1LaunchError(
            "启动文件没有这些参数的声明："
            + "；".join(missing)
            + "。文件里声明了："
            + ", ".join(declared)
        )
    return bound
