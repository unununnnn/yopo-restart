---
Status: accepted
---
# YOPO 变体：YOPO-MINCO @ d5e1e60，自写 ROS2 封装，复用 YOPO-ROS2 的 Controller / 消息 / 仿真移植

候选生成器以 TJU YOPO 的 YOPO-MINCO 分支 `d5e1e60f83cc991602f142d9b6b96204a24d6b9f` 为准（两段五次 MINCO、走廊输出、通过筛选的候选取 argmax(score)），网络与权重不改（Q4）。dairycowp/YOPO-ROS2（`d2fc4fc`）移植的是旧 YOPO-Simple（argmin、9 维终端状态、单段 Poly5），与方案书的上游核查不一致，因此**不沿用其网络与推理节点**；只复用它已经移植到 ROS2 的 SO(3) 动力学/控制器、quadrotor_msgs 和（若 ADR-0005 选中）传感器仿真。推理与后处理放在 ROS-free 核心，ROS2 层只做薄封装和每机命名空间。

## Consequences

上游是 ROS1 节点，ROS2 封装需自写；任何把 YOPO-Simple 行为（argmin、Poly5）带进来的改动都违反本 ADR。

## 修订（2026-10-24，ADR-0011）

控制侧改为 PX4：规划结果经 `yopo_swarm_core.px4` 转成 NED 的 `px4_msgs/TrajectorySetpoint`，由 PX4 位置控制器跟踪。不再复用 YOPO-ROS2 的 SO(3) 控制器 / `quadrotor_msgs` / 其传感器仿真。网络、权重、推理与解码语义不变。

#13 封装落地：`core.uav` 只采样 SafetyGate 批准凭证，ROS 节点无规划逻辑；初始保持也需
本机批准，暂停时不发布新命令。IPC 的 batch=1 在候选源边界变为 #9 的无 batch 输入。
详见 [ROS2 节点契约与验收](../deploy/ros2-uav-node.md)。

## 修订（2026-10-26，#33）：移植主线与保真登记

本修订在本 ADR 上追加，不取代本 ADR。上游基准仍是 YOPO-MINCO `d5e1e60f83cc991602f142d9b6b96204a24d6b9f`，权重仍是 `epoch50.pth`（sha256 `2d85e712…f62f`）。逐文件映射见 [MINCO 移植映射表](../ref/minco-port-map.md)，该表共 27 行，另含决策 D1–D10 和工单 Y0–Y9（#33–#42）。

**移植形态：**
- 按 dairycowp/YOPO-ROS2 的形态移植 MINCO d5e1e60：算法留在 ROS-free 核心，ROS 2 只是一层薄 rclpy 节点。
- 节点行为用 unununnnn/yopo-ros2 的时间契约修复加固，包括源时间戳、深度回调隔离、30 Hz 槽位门控和重复时间戳活性检测。
- 上游代码按钉定 SHA 复用，不重写。训练走 `train/` shim（#39），传感器仿真按上游构建（#40）。

**保真登记：**
与上游不同的地方只能是下表所列的几项，每项都写明理由和锁定它的测试或工单。

| 偏差 | 理由 | 锁定 |
|---|---|---|
| (a) 解码的头状态取自参考计划（t_switch 处的 PVA），不取实测里程计 | 上游对位置控制的建议做法 | `test_candidates.py::test_reconstructed_head_matches_parent_pva_at_t_switch` |
| (b) 通过筛选的候选为空时，走本仓库已认证的刹车分支，而不是"保持上一条命令" | PX4 需要可认证的停止 | `test_planner.py::test_stale_frame_rejected_and_falls_back_to_brake_branch`；严格准入（无候选准入 → `CORRIDOR_EMPTY`）：`test_planner.py::test_corridor_empty_rejected_and_falls_back_to_brake_branch` |
| (c) 选出候选之后再做 ENV 证明和连续时间碰撞检查 | 最终安全过滤 | `test_planner.py::test_gate_independently_rejects_unsafe_plan`；ENV 见 #23 / #37 |
| (d) 用 `px4_msgs/TrajectorySetpoint` 替代 `PositionCommand` | ADR-0011 | `test_px4.py::test_trajectory_setpoint_fields` |
| (e) 命令按仿真时间采样，替代固定 dt 回放 | 确定性 | `test_uav.py::test_stale_frame_selects_future_approved_brake_and_samples_it_at_branch_time` |
| (f) 目标高度可配置，不硬编码 2 m | 任务需要 | #34 |
| (g) 可选的时长拉伸重定时，只在 D10 下启用 | 运动学可行性 | #38（D10） |

其余节点决策全部与上游一致：
- 选择准入：μ_lo ≥ safe_mu，再取 argmax(score) 和 topk 连续性；
- 航向：`calculate_yaw`，速率 ≤ 0.5π rad/s；
- 目标裁剪：z 优先，goal_length = 10；
- 到达半径：5 m。

这些决策由 `core/tests/golden/minco_node_d5e1e60.npz` 锁定，该金样由 #34 生成，来自上游 `test_yopo_ros.py` 去掉 ROS 后的 ≥200 帧回放（`scripts/make_node_golden.py`，`plan_from_reference=True` 即 D2；比对见 `core/tests/test_node_golden.py`）。

准入比较固定按 float64 进行：上游 `safe_mu` 是 `np.float64`，在 NumPy ≥ 2 下 float32 的 μ_lo 按 float64 比较；NumPy 1.x（Orin 上为 1.26）的按值转换会降到 float32，于是 μ_lo 恰为 float32(safe_mu) 的 1 ulp 边界上会多准入。本仓库不随 NumPy 版本变化，取更保守的 float64 语义（`test_node_golden.py::test_admission_boundary_is_float64_like_upstream`）。

不在上表中的偏差都违反本 ADR。要新增偏差，必须先在这里登记，再改代码。

`core/tests/test_upstream_pin.py` 负责防止钉定版本漂移：
- 离线用例核对 SHA、权重哈希和金样元数据；
- 带 `upstream` 标记的用例联网确认 YOPO-MINCO 分支 head 仍为 d5e1e60。
