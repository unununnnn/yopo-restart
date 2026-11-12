# scripts/sitl：box 上的单机 PX4 SITL（票 S1，report-only）

headless 跑一个 PX4 实例（gz_x500，Gazebo Harmonic 服务端），用 ROS 2 + px4_msgs 走 offboard：
起飞到 2.5 m → 悬停 10 s（仿真时间）→ 飞到悬停点 +(5, 3, 0) m（ENU）→ PX4 自己的 NAV_LAND。
setpoint 和状态的 ENU↔NED 转换只用 `core/src/yopo_swarm_core/px4.py`（ADR-0011）。

```bash
scripts/sitl/up.sh --bg       # 构建镜像（幂等；后台，打印 PID 文件和日志）
scripts/sitl/up.sh --status   # ready / building / missing
scripts/sitl/smoke.sh         # 跑一次冒烟，打印结果表；--bg 后台跑
```

- 版本钉死：PX4 v1.17.0（d6f12ad）、px4_msgs release/1.17（86d8239）、Micro-XRCE-DDS-Agent v2.4.3、ROS 2 Humble。
  镜像标签带 Dockerfile 的哈希，内容不变就复用。
- 参数：每次全新状态目录，只用官方 airframe 默认值；脚本里不 `param set`、不设 `PX4_PARAM_*`（`tests/test_sitl_s1.py` 守着）。
  每次运行把与默认不同的参数只读导出到 `px4-params-changed.txt`（只有 PX4 SITL 启动脚本自己写的传感器标定 ID）。
- SITL airframe 默认 `NAV_DLL_ACT=2`，没有地面站时预检报 “No connection to the GCS”。不改参数，
  而是像 QGC 一样每秒向 UDP 18570 发一个 GCS HEARTBEAT。
- 资源：构建和仿真都 nice 15，PX4 编译 `j=4`，只允许 1 个 PX4 实例（同名容器在跑就拒绝）。
- 输出：`/workspace/yopo-ops/sitl/runs/<时间>/`，含 `result.json`、`summary.md`、`px4.log`、`agent.log`。
- 误差两套口径：默认相对 PX4 自身估计（VehicleOdometry，飞控实际跟踪的量）；“Gazebo 真值”含估计器误差。
- report-only：判据未冻结，不进 `scenes/suite.txt`，不进 regress。
- box 网络：packages.ros.org 证书不匹配、osrfoundation 的 http 返回 500，所以 ROS 直接用
  `ros:humble-ros-base-jammy`，Gazebo 源改用 https。
