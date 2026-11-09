# 上游参考与固定 SHA

> 2026-10-24 只读调查：仅克隆、阅读源码和计算哈希。**未编译、未运行、未加载权重、未测 ROS2/CUDA/性能。** 下列「行为」结论均来自读源码，不是运行结果。

## 固定提交

| 仓库 | 分支 | SHA | 提交时间 (UTC+8) | 许可 |
|---|---|---|---|---|
| [dairycowp/YOPO-ROS2](https://github.com/dairycowp/YOPO-ROS2) | main | `d2fc4fcdeb75445cc1d38e5ecce92b629ddbd3b1` | 2026-09-07 | MIT（© 2025 dairycow） |
| [TJU-Aerial-Robotics/YOPO](https://github.com/TJU-Aerial-Robotics/YOPO) | YOPO-MINCO | `d5e1e60f83cc991602f142d9b6b96204a24d6b9f` | 2026-09-09 | MIT（© 2024 TJU-Aerial-Robotics） |
| 同上 | YOPO-Simple（默认分支） | `bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132` | 2026-08-14 | MIT |
| 同上 | main（论文稳定版） | `881e1c549122efa5a2402b862f70a198e6e93baa` | 2025-05-10 | MIT |

上游另有 tag `v1.0`、`hardware`，以及 Releases 页提供的其他预训练模型（未查看）。

## 权重

| 来源 | 路径 | sha256 |
|---|---|---|
| YOPO-MINCO @ d5e1e60 | `YOPO/saved/YOPO_1/epoch50.pth` | `2d85e712c8874d11cd4e523777698eff78914fdb8f1ffb5068b037f8f972f62f` |
| YOPO-Simple @ bd8a7f9 | `YOPO/saved/YOPO_1/epoch50.pth` | `09ec31094ed83e09702efc1facf18076564ebbe1afc164960c41036e16ba229f` |
| YOPO-ROS2 @ d2fc4fc | `YOPO/saved/YOPO_1/epoch50.pth` | `5ec1a8a636ae3069acdd3a90f7d4cd1de8f1cbb46745e368532fd80bed810eab` |
| YOPO-ROS2 @ d2fc4fc | `YOPO/saved/YOPO_3/epoch1.pth`（移植者自训，1 epoch） | `66546cc508628045ad678f83778ae5ad2ea7079b0e748ef2f389d8aeac26bdd6` |

- 三份 `epoch50.pth` 互不相同。YOPO-ROS2 那份的 git blob 与上游 YOPO-Simple 在 2025-06-24 前后（`e24865b` / `cc13b0e`）的版本一致，即**移植基于较旧的 YOPO-Simple 快照**（约 2025-06 至 2025-10 之间），不是当前 YOPO-Simple，更不是 YOPO-MINCO。
- 权重直接放在 MIT 仓库内，仓库没有单独的权重许可声明。按仓库 MIT 处理是合理推断，但**不是明确授权**；正式交付前建议向上游确认或在报告中注明。
- 上游 README：预训练权重在 6 m/s 下训练，0–6 m/s 表现平滑；训练相机 16:9、90° FOV。

## YOPO-ROS2 结构（读源码）

- 目标：YOPO-Simple 的 ROS 2 版本（README 原文）。未写明 ROS2 发行版。线索：提交的 `.pyc` 为 cpython-310、CMake 硬编码 `gcc-11/g++-11` → 推断为 **Ubuntu 22.04 + Humble**（未证实）。
- 三个 colcon 工作区，分开构建：
  - `Controller/`：`cmake_utils`、`quadrotor_msgs`、`uav_utils`、`so3_control`（依赖 `mavros_msgs`）、`so3_quadrotor_simulator`（SO(3) 四旋翼动力学，ODE 积分）。launch 为 `.launch.py`（也残留 ROS1 `.launch`）。
  - `Simulator/`：`sensor_simulator`。`sensor_simulator_cuda` 为推荐版，CMake 中 **`find_package(CUDA 12.8 EXACT REQUIRED)`** 并自动探测 GPU 架构；CPU 版 `sensor_simulator` 标注「已弃用」但仍在 CMake 中。随机地图由 `seed` 决定（森林/柱子/迷宫等）。
  - `YOPO/`：纯 Python 脚本 `test_yopo_ros.py`（rclpy 节点），不是 ament 包，需 `source Controller/install` 才能找到 `quadrotor_msgs`。
- Python 依赖：`torch==2.4.1+cu118`、numpy 1.22.3、scipy 1.10.1、opencv 4.11、open3d 0.19 等。注意 torch 用 CUDA 11.8 轮子，而传感器仿真要求 CUDA 12.8。
- 推理链路（`test_yopo_ros.py`）：
  1. 订阅 `/sim/odom`、`/depth_image`、`/move_base_simple/goal`（全局绝对话题名，无命名空间）。
  2. 深度图缩放到 160×96，`cv2.inpaint` 补洞后送网络；`cuda` 不可用时自动回退 CPU。可选 TensorRT。
  3. 网络输出 15 个候选（5×3 网格）的 9 维终端状态（体坐标 P/V/A）+ 分数，**取 `argmin(score)`（越小越好）**。
  4. 每轴一条 `Poly5Solver` 单段五次多项式，从当前里程计或上一参考（`plan_from_reference`）出发。
  5. 墙钟定时器按 `ctrl_dt` 采样并发布 `PositionCommand` 到 `/so3_control/pos_cmd`。
- 时间：节点用 `time.time()` 和墙钟定时器，未见 `use_sim_time` 或 `/clock` 发布方。
- 无任何候选几何验证、回退、机间约束。

## YOPO-MINCO（方案书核查的分支，读源码）

- ROS1（`rospy`），README 写「Ubuntu 20.04 + Jetson Orin/Xavier NX」测试。
- 轨迹为两段五次 MINCO（`MincoTraj().solve(head_pva, tail_pva, inner, durations)`），网络另预测安全走廊；推理对通过走廊筛选的候选取 **`argmax(score)`**。候选网格同为 5×3。
- 构建与使用方式与 YOPO-Simple 相同，共用同一套 CUDA 传感器仿真与 SO(3) 仿真。

## 与 V0.3 方案的对照

| 方案要求 | YOPO-ROS2 现状 | 结论 |
|---|---|---|
| Q4 不改网络与权重；正式基线固定源码/权重/配置 SHA | 移植的是旧 YOPO-Simple 网络与权重，非 MINCO | **冲突**：方案第 2 节的上游核查（argmax、14 维、MINCO 两段、走廊）全部针对 MINCO；采用 YOPO-ROS2 意味着换成 Simple（argmin、9 维、单段 Poly5、无走廊）。需决策 |
| Q5 原生 ROS2 Humble | 已是 ROS2（推断 Humble），但 YOPO 部分是脚本不是包 | 基本一致；需包化 |
| ROS-free 算法核心，与消息层解耦 | 推理、后处理、多项式、控制采样全在一个 rclpy 节点里 | **冲突**：需拆出 ROS-free 核心（G1） |
| 每机独立推理、独立命名空间 | 全局话题名，单实例 | **冲突**：需命名空间化；可多开进程实现每机独立实例 |
| 无中央选择器 | 单机，不涉及 | 无冲突，但不能在其上加共享批推理选择 |
| 共享世界、三个独立相机、正确遮挡 | 传感器仿真每进程订阅一个 odom、渲染一个相机；地图由 seed 决定 | **部分冲突**：多实例同 seed 可共享静态世界，但**其他无人机不会出现在深度图中**（无机间遮挡） |
| `/clock`、暂停、步进、重置、重复 seed | 墙钟驱动，无 `/clock`；有 seed | **冲突**：需补仿真时间与步进语义 |
| 控制适配输出跟踪误差，而非瞬移 | SO(3) 动力学 + 控制器，非瞬移 | 一致，可作为最小动力学契约起点 |
| 先安全回退再轨迹提交；未来切换时刻 PVA | 无回退；起点为当前状态或当前参考 | 需新开发（G2） |
| 仅软件仿真、可离线固定依赖 | 依赖 CUDA 12.8 EXACT + NVIDIA GPU；torch cu118 | **环境冲突**：开发 box 无 GPU（见下） |

## 开发 box 环境（2026-10-24 实测）

- OS：Debian GNU/Linux 13 (trixie)，x86_64，运行在容器内（`systemd-detect-virt` = container-other）。
- CPU 8 核 Intel Xeon；内存 15 GiB（可用约 10 GiB）；无 swap；`/` 剩余约 35 GiB。
- **无 NVIDIA GPU**：没有 `nvidia-smi`、没有 `/dev/nvidia*`、没有 nvcc。
- 未安装 ROS2（无 `/opt/ros`）；Humble 官方二进制只支持 Ubuntu 22.04，不支持 Debian 13。
- 无 docker / podman；有 sudo、user namespace 可用、存在 `/dev/kvm`、有 `uv`、Python 3.13.5。apt 源列表为空，容器工具能否安装、能否在此容器内运行**未验证**。
