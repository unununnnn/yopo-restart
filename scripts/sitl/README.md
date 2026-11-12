# scripts/sitl：box 上的单机 PX4 SITL（票 S1，report-only）

headless 跑一个 PX4 实例（gz_x500，Gazebo Harmonic 服务端），用 ROS 2 + px4_msgs 走 offboard：
起飞到 2.5 m → 悬停 10 s（仿真时间）→ 飞到悬停点 +(5, 3, 0) m（ENU）→ PX4 自己的 NAV_LAND。
setpoint 和状态的 ENU↔NED 转换只用 `core/src/yopo_swarm_core/px4.py`（ADR-0011）。

```bash
scripts/sitl/up.sh --bg       # 构建镜像（幂等；后台，打印 PID 文件和日志）
scripts/sitl/up.sh --status   # ready / building / missing
scripts/sitl/smoke.sh         # 跑一次冒烟，打印结果表；--bg 后台跑
scripts/sitl/smoke.sh --positioning rtk   # 厘米级 RTK 定位源（默认关）
scripts/sitl/compare.sh       # 默认与 RTK 各跑 2 次，出对照表
```

- 版本钉死：PX4 v1.17.0（d6f12ad）、px4_msgs release/1.17（86d8239）、Micro-XRCE-DDS-Agent v2.4.3、ROS 2 Humble。
  镜像标签带 Dockerfile 的哈希，内容不变就复用。
- 参数：每次全新状态目录。默认路径（`--positioning default`）只用官方 airframe 默认值，脚本里不 `param set`、不设 `PX4_PARAM_*`。
  `--positioning rtk` 只通过 `positioning-rtk.env` 注入白名单定位/估计参数（`SIM_GZ_GPS_*`、`EKF2_GPS_P_NOISE`、`EKF2_GPS_V_NOISE`、`EKF2_HGT_REF`），
  经 PX4 官方 `PX4_PARAM_*` 在启动时生效；不改安全参数。测试守着白名单。
  每次运行把与默认不同的参数只读导出到 `px4-params-changed.txt`。
- 镜像标签为 `yopo-restart-sitl:s1b-<Dockerfile 哈希>`，不覆盖 S1 的 `s1-` 标签。
  Dockerfile 在编译前给 GZBridge 打补丁：把写死的 GPS 噪声/fix/eph/epv 变成参数，默认值仍是 v1.17.0 的 0.8 m / fix=3 / eph=0.9 / epv=1.78。
  Gazebo x500 的 NavSat 本身没有噪声；米级误差来自 PX4 `addGpsNoise`。RTK 模式把幅度降到 2 cm，并报 `fix_type=6`。
- SITL airframe 默认 `NAV_DLL_ACT=2`，没有地面站时预检报 “No connection to the GCS”。不改参数，
  而是像 QGC 一样每秒向 UDP 18570 发一个 GCS HEARTBEAT。
- 资源：构建和仿真都 nice 15，PX4 编译 `j=4`，只允许 1 个 PX4 实例（同名容器在跑就拒绝）。
- 输出：`/workspace/yopo-ops/sitl/runs/<时间>/`，含 `result.json`、`summary.md`、`px4.log`、`agent.log`。
- 误差两套口径：默认相对 PX4 自身估计（VehicleOdometry，飞控实际跟踪的量）；“Gazebo 真值”含估计器误差。
  另报悬停段估计高度相对真值的偏差（同一世界系的均值 / RMS / 最大）。
- report-only：判据未冻结，不进 `scenes/suite.txt`，不进 regress。
- box 网络：packages.ros.org 证书不匹配、osrfoundation 的 http 返回 500，所以 ROS 直接用
  `ros:humble-ros-base-jammy`，Gazebo 源改用 https。
