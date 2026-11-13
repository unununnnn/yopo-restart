# 票 S4b：YOPO L0 规划桥接进 PX4 / Gazebo 容器

在已合入的 S4 桥（`yopo_l0/sitl_bridge.py`、`scripts/sitl/run_yopo_l0.py`）上，把同一条 ENU↔NED 边界接到 ROS 2 消息字段，并给容器一条可单独打开的实飞入口。判据未冻结。下面的数字是 **report-only**：`report_only=true`，`criteria_frozen=false`。

不改 `yopo_l0/policy.py`、`core/`、场景卡、`offboard_mission.py`，也不改 `smoke.sh` / `up.sh` 的默认行为。`YOPO_SITL_PLANNER` 缺省是 `offboard`，单机 RTK 冒烟仍走原来的 `offboard_mission.py`。

## 改动文件

- `yopo_l0/sitl_bridge.py`：里程计 / 深度图 / 障碍摘要的消息字段转换，多机 namespace，到达后的 `NAV_LAND` 收尾。仍不导入 `rclpy`。
- `scripts/sitl/run_yopo_l0.py`：`--fake-px4`（缺省就是这个模式）和 `--mock-ros`。
- `scripts/sitl/yopo_l0_live.py`：容器内订阅 / 发布。`rclpy` 只在非 `--dry-run` 时导入。
- `scripts/sitl/run_yopo_l0_sitl.sh`：宿主机一键入口。
- `scripts/sitl/in_container.sh`、`scripts/sitl/exec_mission.sh`：仅当 `YOPO_SITL_PLANNER=yopo-l0` 时改走规划桥。
- `scripts/sitl/README.md`：补上这条入口。
- `tests/test_sitl_live_bridge.py`：离线假 PX4 与 mock ROS 消息闭环。

轴映射只调用 `yopo_swarm_core.px4`（ADR-0011）：`odometry_to_enu`、`enu_to_ned`、`ned_to_enu`、`trajectory_setpoint`、`offboard_control_mode`、`vehicle_command`。桥里没有第二套符号反转，也没有参数写入。`positioning=rtk` 只记进结果；厘米级定位仍由容器里已有的 `positioning-rtk.env` 白名单在启动时注入。

## 消息

每机只读自己的 namespace。1 机 `g2-pillar` 是 instance 0，namespace 为空：

| 方向 | 话题 |
|---|---|
| 订阅 | `/fmu/out/vehicle_odometry` |
| 订阅（可选） | `/yopo/depth_image`、`/yopo/obstacle_summary` |
| 发布 | `/fmu/in/trajectory_setpoint`、`/fmu/in/offboard_control_mode`、`/fmu/in/vehicle_command` |

3 机 `swap-3-open` 与 `fleet.py` 一致：`px4_1`、`px4_2`、`px4_3`，MAV system 2、3、4，GCS 端口 18571、18572、18573。话题是 `/px4_1/fmu/out/vehicle_odometry` 这种形式。`px4_9` 一类多出来的 namespace 不会写进这三架的状态。

`VehicleOdometry` 的 `position` / `velocity` / `q` 原样进 `odometry_to_enu`，再组成 L0 `Observation`。深度图接受 32FC1（米）和 16UC1（毫米，0 为无回波）。障碍摘要是 ENU 点，或 `ranges_m` + `bearings_rad` 的水平扇面，按 `DepthCamera` 的光学轴投成一帧 z 深度。两边都缺、且 `depth_source=auto` 时，这一拍退回场景卡相机。设定点是 NED 的位置、速度、加速度和航向。

到达沿用场景卡的 0.4 m / 0.2 m/s / 保持 1 s。全体到达并保持之后发 `MAV_CMD_NAV_LAND`（命令号 21），目标 system 用该机的 MAV id。假 PX4 不积分降落过程；「已落地且上锁」是收尾状态。解锁、offboard 和 GCS 心跳在实飞脚本里，桥不改 PX4 参数。

## 怎么跑

离线假 PX4（与 S4 相同，schema `yopo-restart/s4-sitl-bridge/1`）：

```bash
.venv/bin/python scripts/sitl/run_yopo_l0.py --fake-px4 --scene scenes/g2-pillar.json --seed 0 --max-s 15
.venv/bin/python scripts/sitl/run_yopo_l0.py --scene scenes/g2-pillar.json --partial
```

离线，但里程计和深度先编成 ROS 消息再解回桥（schema `yopo-restart/s4b-sitl-live/1`）：

```bash
.venv/bin/python scripts/sitl/run_yopo_l0.py --mock-ros --positioning default \
  --scene scenes/g2-pillar.json --seed 0 --max-s 15
.venv/bin/python scripts/sitl/run_yopo_l0.py --mock-ros --positioning rtk \
  --scene scenes/swap-3-open.json --partial
```

容器实飞（不经过 `offboard_mission.py` 的默认冒烟）。镜像里需要能导入规划栈；本环境没有把 torch 打进 SITL 镜像，所以这里没有容器实测：

```bash
scripts/sitl/run_yopo_l0_sitl.sh --dry-run --n 1
scripts/sitl/run_yopo_l0_sitl.sh --n 1
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --positioning rtk
```

`--dry-run` 只打印 `planner=yopo-l0`、场景、机数和定位模式。1 机默认 `scenes/g2-pillar.json`，3 机默认 `scenes/swap-3-open.json`。容器已经由 `up.sh` 拉起时，脚本 `docker exec` 进 `exec_mission.sh`；否则自己起一趟仿真，任务段只跑 `yopo_l0_live.py`。

## g2-pillar seed 0（report-only）

假 PX4，消息字段走一遍再进桥。和直接 `run_closed_loop` 的到达时刻、净空、终点一致。

| 项 | 值 |
|---|---|
| schema | `yopo-restart/s4b-sitl-live/1` |
| adapter / fake_px4 | `mock-ros` / true |
| positioning | `default` |
| px4_parameters_written | `[]` |
| 结果 | `SUCCESS` / `GOAL_REACHED_AND_HELD`，随后 `finish=NAV_LAND` |
| 到达 | 1/1，到达率 1.0，到达时刻 4.28 s，仿真 5.28 s |
| 碰撞 | 0 |
| 最小表面净空 | 0.4690522308430273 m（中心到表面 0.8190522308430273 m） |
| 终点位置 ENU | [5.011095171486111, -0.011326804195948488, 1.5000553682468127] |
| 终点目标误差 | 0.015855673724011723 m |
| ENU↔NED 往返 | 位置 0 m，速度 0 m/s，加速度 0 m/s² |
| 规划 | 53 次：`OK` 21，`ARRIVED_HOLD` 32 |
| 降落 | 1 条命令，`command=21`，`target_system=1`，`landed=true`，`ok=true` |
| namespace | 空字符串，话题 `/fmu/out/vehicle_odometry` |

`scenes/swap-3-open.json` seed 0、`--mock-ros --positioning rtk --partial` 且前缀 0.3 s：`PARTIAL`，到达率 0，碰撞 0，最小机间距离 13.804184374201498 m，最小表面净空 1.15 m，邻机采样递交 12 次，往返误差 0。namespace 是 `px4_1`、`px4_2`、`px4_3`。`landed=false`，`land_commands=[]`，`px4_parameters_written=[]`。

## 本环境实测

```bash
PYTHONPATH=".:core/src" .venv/bin/python -m pytest tests/test_sitl*.py -q --tb=line
bash -n scripts/sitl/*.sh
```

本机用的解释器是 `/tmp/yopo-venv/bin/python`（Python 3.12，numpy 2.5.4，torch 2.4.1+cpu），`PYTHONPATH` 指向仓库根和 `core/src`。

| 命令 | 结果 |
|---|---|
| `pytest tests/test_sitl*.py -q --tb=line` | **50 passed** in 19.64s |
| `bash -n scripts/sitl/*.sh` | 退出码 0 |

`tests/test_sitl_live_bridge.py` 覆盖 ENU↔NED 往返、深度图与障碍摘要、单步设定点和直接 `command` 一致、`px4_1`/`px4_2`/`px4_3` 隔离、`positioning=rtk` 只记结果、目标保持后降落并在上锁后 `DONE`、每机解锁命令的 system id、`g2-pillar` 全长闭环与 S4 假 PX4 对齐、三机 `swap` 前缀、`--fake-px4` 仍是 S4 schema，以及 shell dry-run。没有在本环境里拉起 PX4 / Gazebo 容器。
