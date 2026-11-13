# 票 S4：YOPO L0 与 PX4 SITL 的 ENU/NED 闭环桥

只新增桥接层和离线校验。不改 `scripts/sitl/offboard_mission.py`、`fleet.py`、`smoke.sh`，也不改 `yopo_l0/policy.py`、`core/` 和已有场景卡。

判据未冻结。下面的数字是 **report-only**：`result.json` 里 `report_only=true`、`criteria_frozen=false`。`g2-pillar` 场景卡自己的 `criteria.frozen` 仍是 true，本票不拿那扇门当本报告的通过条件，也不改卡。

## 接口

规划在 ENU，PX4 的 `VehicleOdometry` / `TrajectorySetpoint` 在 NED。轴映射和航向只调用 `yopo_swarm_core.px4`（`core/src/yopo_swarm_core/px4.py`，模块头写的 ADR-0011 边界）：`odometry_to_enu`、`ned_to_enu`、`enu_to_ned`、`yaw_ned_to_enu`、`yaw_enu_to_ned`、`trajectory_setpoint`、`offboard_control_mode`。桥里没有第二套符号反转。本模块不导入 `rclpy`。

`SitlBridgeConfig` 给出一架飞机的周期和策略参数：`physics_dt`（默认 0.02 s，50 Hz 设定点）、`planner_dt`（默认 0.1 s，10 Hz 重规划）、`hover_alt_m`、`goal_enu`、`policy_params`，以及动力学增益 `tau_s` / `kp` / `kd` / `a_max`（默认与 `PointMassLag` 相同）。从场景卡跑时，高度、目标和增益取自该机的起点、`goal_enu` 和 `dynamics`。

`YopoSitlBridge.command(position_ned, velocity_ned, now_s, q_ned_frd=..., yaw_ned=...)`：

1. 用 `px4` 把 NED 里程计转成 ENU，构成 `State`。给了四元数就走 `odometry_to_enu`，否则用 `yaw_ned`。两个都缺则拒绝。
2. 到了 `planner_dt`（第一拍立刻规划）用 `DepthCamera` 对场景卡里的已知障碍渲染深度，调用 `UpstreamPolicy.plan`。返回的对象就是策略自己的 `_TrajPlan`。
3. 每个 `physics_dt` 对这份计划采样 `(p_enu, v_enu, a_enu, yaw_enu)`，再 `px4.trajectory_setpoint` 写成 NED 的位置、速度、前馈加速度和偏航，并带上 `offboard_control_mode`（位置+速度+加速度）。同一个 `now_s` 再调一次不重复采样，避免 `_TrajPlan.sample` 把偏航推进两次。

邻机是扩展口，不是第二套规划器。`set_neighbors` 收下邻机的 ENU 位置/速度和可选轨迹采样。下一次 `plan` 之前，采样交给已有的 `UpstreamPolicy.note_neighbor_samples`。机间过滤、后段惩罚、`geom_multi` 都关着时，策略按原实现把采样丢掉，端口本身仍然记下这次递交。多机闭环是多份 `YopoSitlBridge`：全体先规划，再交换本拍广播，下一拍规划才看得到，和 L0 运行器同一拍内互相看不见是对齐的。

`FakePx4Vehicle` 在 NED 里跟踪设定点。加速度一阶滞后，再用新加速度积分速度和位置，增益与 `PointMassLag` 相同。设定点已经是 NED，这里不再做轴映射。水平四元数由 NED 航向在 NED 里构成。它是容器前的对照模型，不是 PX4 估计器。

`OffboardCommand` 把一次 `command` 收成 `trajectory_setpoint`、`offboard_control_mode` 和 namespace。这是以后容器侧唯一要填进 `px4_msgs` 的对象。

离线入口是 `run_closed_loop` 和 `scripts/sitl/run_yopo_l0.py`：

```bash
.venv/bin/python scripts/sitl/run_yopo_l0.py \
  --scene scenes/g2-pillar.json --seed 0 --max-s 15 --out /tmp/s4-g2.json
.venv/bin/python scripts/sitl/run_yopo_l0.py \
  --scene scenes/swap-3-open.json --seed 0 --max-s 15 --partial
```

`--partial` 只跑 `min(--max-s, 0.5)` 秒前缀。`--out` 把同一份 JSON 写到文件，并打印到 stdout。schema 是 `yopo-restart/s4-sitl-bridge/1`。到达率、最小表面净空（中心到表面的距离减去 `body_radius_m`）、ENU↔NED 位置往返最大误差都在这份 JSON 里。碰撞按场景卡机体半径：障碍是中心到表面 ≤ 半径，机间是中心距 ≤ 两倍半径。到达沿用卡上的 0.4 m / 0.2 m/s / 保持 1 s。

和 L0 `runner.run` 的差别要记在这里：设定点是 50 Hz，不是场景卡物理步的 100 Hz；深度在每次重规划时由已知几何渲染，不是 Gazebo 相机；偏航只取水平航向，不吃俯仰和滚转。可选的 `policy_params` 原样传进 `PolicyContext`，默认上游行为不变。

## 等 S3 合入之后怎么接容器

S3 还没在这棵树上。现有的容器胶水是 `scripts/sitl/offboard_mission.py`（S1 单机任务，S2 的三机悬停走 `fleet.py` 的 namespace）。本票不改那些文件。

S3 合入后，ROS 循环仍留在那个脚本（或它的后继）里，不进 `yopo_l0/sitl_bridge.py`：

- 订阅各机 `VehicleOdometry`（`px4.topic("out", "vehicle_odometry", namespace)`）。把 `position`、`velocity`、`q` 原样交给 `YopoSitlBridge.command`。不要在 ROS 侧先转成 ENU 再转回 NED。
- 用返回的 `OffboardCommand` 填已经存在的 `TrajectorySetpoint` 和 `OffboardControlMode` publisher。字段已经是 `px4.trajectory_setpoint` / `px4.offboard_control_mode` 的字典。心跳继续高于 2 Hz；离线环是 50 Hz。
- 三机 namespace 与 `fleet.py` 一致时用 `px4_1`、`px4_2`、`px4_3`。`run_closed_loop` 在多机时把 `SitlBridgeConfig.px4_namespace` 设成 `px4_{i+1}`，单机是空字符串（instance 0，没有 namespace）。
- 解锁、offboard 模式、GCS 心跳、降落仍是那份脚本里的 PX4 命令。桥只出设定点，不发 `VehicleCommand`，也不改 PX4 参数。
- 真机深度要换掉「用场景卡几何渲染」。策略接口仍然是 `Observation` 里的深度图；换相机不应再写一套 ENU/NED。

在那之前，`FakePx4Vehicle` 就是闭环的另一端。

## g2-pillar seed 0（report-only）

命令是上面的第一条，`--max-s 15`，没有 `--partial`。假 PX4，不是容器里的 PX4。

| 项 | 值 |
|---|---|
| schema | `yopo-restart/s4-sitl-bridge/1` |
| report_only / criteria_frozen | true / false |
| 场景卡 | `g2-pillar`，哈希 `9d385f4cfce543922961865d06ceeedd44afdddb52c706bf6e3da9929431d8ae` |
| 结果 | `SUCCESS` / `GOAL_REACHED_AND_HELD` |
| 到达 | 1/1，到达率 1.0，到达时刻 4.28 s，仿真停在 5.28 s（含 1 s 保持） |
| 碰撞 | 0 |
| 最小表面净空 | 0.4690522308430273 m（中心到表面 0.8190522308430273 m，机体半径 0.35 m） |
| 终点位置 ENU | [5.011095171486111, -0.011326804195948488, 1.5000553682468127] |
| 终点目标误差 | 0.015855673724011723 m |
| ENU↔NED 往返最大误差 | 位置 0 m，速度 0 m/s，加速度 0 m/s² |
| 规划 | 53 次：`OK` 21，`ARRIVED_HOLD` 32 |
| 周期 | 设定点 50 Hz（0.02 s），规划 10 Hz（0.1 s） |
| 适配器 | `fake-px4`，转换 `yopo_swarm_core.px4` |

往返误差是 0，因为 `px4` 的 ENU↔NED 只交换轴并改一个符号，float64 里这是精确的。测试仍要求 `< 1e-12`。这次墙钟大约 3.6 s，不进判据。

同一份代码上，`scenes/swap-3-open.json` seed 0、`--partial` 且前缀收到 0.3 s（三机对向，只报数，没有飞完）：`PARTIAL`，到达率 0，碰撞 0，最小机间距离 13.804184374201498 m，最小表面净空 1.15 m（开阔地面：1.5 − 0.35），邻机采样递交 12 次，位置往返误差 0。默认机间过滤是关的，所以这些采样按 `UpstreamPolicy` 的原规则不进入选轨。

单测：`.venv/bin/python -m pytest tests/test_sitl_bridge.py`。覆盖往返误差、与 `UpstreamPolicy` 同一次观测的单步采样一致、假 PX4 与 `PointMassLag` 在 ENU 下对齐、`g2-pillar` seed 0 到达且无碰撞，以及 `--partial` 写出的 JSON。
