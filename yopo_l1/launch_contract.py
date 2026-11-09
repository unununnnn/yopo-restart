"""L1 在 Orin 上的启动入口，以及可选的 ros2 launch 绑定。

frank 在 Orin 上确认过：

- ``uavs.launch.py`` 只声明 ``vehicles`` / ``ipc_root`` / ``log_root`` /
  ``body_radius`` / ``tracking_margin`` / ``code_sha``。没有 scene、seed、policy、
  shadow、start_enu、goal_enu。抽检不再要求这些声明存在，也不把它们塞给
  ``ros2 launch``。
- 宿主没有 ROS 2 Humble，也没有 ``ros2_ws/install``。真 SITL 是 Docker Compose
  + ``sim/px4/start.sh`` + ``scripts/g2_run.py``，入口是 ``scripts/run_g2.sh``。
- 默认后端因此是 ``run_g2``：``bash scripts/run_g2.sh nominal``，种子只经
  ``YOPO_SIM_SEED``。这条路径是已部署的 certified 栈，不接受 ``--policy upstream``。
- 显式 ``backend=ros2`` 且 launch 文件真声明了那六个语义名时，才按
  ``DeclareLaunchArgument`` 绑定。对不上就失败，避免把栈不认识的名字传下去。

世界仍是 ``sim/px4/g2-scene.json``。安全清单数值仍以 ``g2_run.py`` 为准，
抽检不下发、不改这些数。
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

BACKEND_RUN_G2 = "run_g2"
BACKEND_ROS2 = "ros2"
RUN_G2_SCRIPT = "scripts/run_g2.sh"
RUN_G2_MODE = "nominal"
SEED_ENV = "YOPO_SIM_SEED"
EXECUTED_POLICY = "deployed-certified"
JOB_CARD_SCHEMA = "yopo-restart/l1-job-card/1"
# 调度器墙钟。含镜像拉起和 nominal G2，不是场景卡里的 45 s 仿真时长。
JOB_TIMEOUT_S = 1800
CONTAINERS = ("yopo-sim-g2", "yopo-swarm:orin-infer-cu13")
SWARM_REPO = "https://github.com/unununnnn/yopo-swarm"
RESTART_REPO = "https://github.com/unununnnn/yopo-restart"

# Orin 上 uavs.launch.py 实际声明的名字（frank 确认）。记录用，不往下传值。
ORIN_LAUNCH_ARGS = (
    "vehicles",
    "ipc_root",
    "log_root",
    "body_radius",
    "tracking_margin",
    "code_sha",
)

# yopo_l0/policy.py CertifiedPolicy.DEFAULTS，注释写明与 L1 g2_run.py 相同。
FROZEN_SAFETY = {
    "body_radius": 0.35,
    "tracking_margin": 0.15,
    "switch_delay": 2.0,
    "start_pad_radius": 1.2,
}
FROZEN_BODY_RADIUS_M = FROZEN_SAFETY["body_radius"]

POLICY_GAP = (
    "今日 scripts/run_g2.sh 与 scripts/g2_run.py 是已部署的 certified 栈，"
    "不接受 --policy upstream。本表 L1 策略是 deployed-certified，不是 upstream。"
    "把 upstream 移植进 L1 不在票 09b：那要改 yopo-swarm，本仓库不动那个仓库。"
)

PYTHON_CHOICE = (
    "抽检不在 Orin 宿主上创建 venv，也不安装 torch（aarch64 没有 torch 2.4.1 cpu wheel）。"
    "export / import / report 只需要 numpy：优先宿主已能 import numpy 的 python3；"
    "没有时用已经存在的镜像 yopo-swarm:orin-infer-cu13，挂上本仓库跑，不往宿主装依赖。"
)

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


def launch_covers_spotcheck(declared: list[str] | None) -> bool:
    """六个抽检语义是否都能在这份 launch 文件里找到声明。"""
    if not declared:
        return False
    have = set(declared)
    return all(any(alias in have for alias in ALIASES[key]) for key in ALIASES)


def resolve_backend(requested: str | None, declared: list[str] | None) -> str:
    """默认 run_g2。只有显式 ros2，或 launch 文件真声明了全部语义名时，才走 ros2。"""
    if requested in (None, ""):
        if launch_covers_spotcheck(declared):
            return BACKEND_ROS2
        return BACKEND_RUN_G2
    if requested in (BACKEND_RUN_G2, BACKEND_ROS2):
        return requested
    raise L1LaunchError(f"不认识的 L1 后端 {requested}（只能是 {BACKEND_RUN_G2} 或 {BACKEND_ROS2}）")


def bind_launch_args(semantic: dict[str, str], declared: list[str] | None) -> dict[str, str]:
    """semantic 的键是规范名。declared 为 None 时原样返回规范名。缺声明就失败。

    只给 ros2 后端用。run_g2 不调用这里。
    """
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
