"""教师数据的几何假设。

论文表 II（arXiv:2011.04183v3）没有给出障碍半径和地图边长。
下面两个数是本管线选定的假设，不是论文复现：

- 障碍半径 OBSTACLE_RADIUS_M = 0.20 m。竖直圆柱，半径固定，不是上游
  ``swarm.launch`` 里 [0.5, 0.7] m 的随机半径。
- 地图边长 MAP_SIDE_M = 20.0 m。正方形障碍场，面积 400 m²。上游
  ``swarm.launch`` 的地图是 36 m × 20 m，论文没有把那个矩形写成 obs/m²。

密度 = 圆柱个数 / 边长²。0、0.14、0.28、0.42 个/m² 对应 0、56、112、168 根。
机体半径 0.2 m 来自论文表 II 的文字。这次要跑的 ``ego_planner_node`` 不打印
碰撞用的机体半径，记录里的碰撞判定用 0.2 m，并标成假设。
"""

from __future__ import annotations

EGO_COMMIT = "92fe9f7227b2da819133eb8e0e8c7fc000f6ae20"
EGO_REPO = "https://github.com/ZJU-FAST-Lab/ego-planner-swarm"
EGO_IMAGE = "osrf/ros:noetic-desktop-focal"
CONTAINER = "ego-teacher"

# 假设，不是论文复现。见本文件顶部说明。
MAP_SIDE_M = 20.0
OBSTACLE_RADIUS_M = 0.20
OBSTACLE_Z_LO_M = 0.0
OBSTACLE_Z_HI_M = 2.5

# 论文表 II 写了 0.2 m。二进制没有打出这个数，碰撞线用它，标成假设。
BODY_RADIUS_M = 0.20

# 放置时，圆心离起点/终点的水平距离至少是障碍半径 + 机体半径 + 这一段余量。
SPAWN_CLEARANCE_EXTRA_M = 0.30

# 沿 x 穿过障碍场。±9 落在边长 20 m 的场内，两端各留 1 m。高度与上游目标 z 相同。
# 上游 swarm.launch 的 x 是 -20 到 20、起点 z=0.1。这里改短，是为了小批量能跑完，
# 同样标成假设。
START_X_M = -9.0
GOAL_X_M = 9.0
FLIGHT_Z_M = 1.0

# 论文没有到达半径。0.5 m / 0.3 m/s / 1.0 s 是本管线的到达定义，写进场景卡，
# 让教师统计和 L0 的 goto 判据用同一组数。
GOAL_RADIUS_M = 0.5
GOAL_SPEED_MPS = 0.3
GOAL_HOLD_S = 1.0

# 规划器栅格要比障碍场大，才能包住 ±9 m 的起终点。数值取自本管线，不是论文。
PLANNER_MAP_X_M = 24.0
PLANNER_MAP_Y_M = 24.0
PLANNER_MAP_Z_M = 4.0

# 上游 run_in_sim.launch 与论文表 II 的速度上限一致。加速度是 launch 里的 3.0，
# 论文表 II 没有给加速度。
MAX_VEL_MPS = 2.0
MAX_ACC_MPS2 = 3.0
PLANNING_HORIZON_M = 7.5

SAMPLE_HZ = 10.0
# 18 m 航程、速度上限 2 m/s，加上绕行和六机顺序起步。到点就停，这是上限。
TIMEOUT_S = 50.0

CLOUD_RESOLUTION_M = 0.1
# 密度为 0 时仍发一个感知范围外的点，避免上游 CPU 渲染节点对空云建 kdtree。
# 这个点不写进 L0 场景卡。
EMPTY_CLOUD_SENTINEL_M = (0.0, 0.0, -10.0)

DENSITIES_PER_M2 = (0.0, 0.14, 0.28, 0.42)
# 与边长 20 m 相乘后是整数。用这张表，避免 0.14 * 400 的二进制误差。
DENSITY_COUNTS = {0.0: 0, 0.14: 56, 0.28: 112, 0.42: 168}
DRONE_COUNTS = (1, 3, 6)
PATTERNS = ("parallel", "headon")
# 4 × 3 × 2 × 8 = 192。200 不能被 24 整除，小批量取每一格同样多的 8 个种子。
SEEDS = tuple(range(8))

SCHEMA_VERSION = "yopo-teacher-record/1"


def obstacle_count(density_per_m2: float) -> int:
    if density_per_m2 in DENSITY_COUNTS:
        return DENSITY_COUNTS[float(density_per_m2)]
    return int(round(float(density_per_m2) * MAP_SIDE_M * MAP_SIDE_M))


def lane_ys(n_drones: int) -> list[float]:
    """横向槽位。1 架在 y=0；3 架 ±6；6 架从 -7.5 到 7.5、间隔 3。"""
    if n_drones == 1:
        return [0.0]
    if n_drones == 3:
        outer = 6.0
    elif n_drones == 6:
        outer = 7.5
    else:
        raise ValueError(f"机数只支持 1、3、6，收到 {n_drones}")
    return [-outer + (2.0 * outer) * i / (n_drones - 1) for i in range(n_drones)]
