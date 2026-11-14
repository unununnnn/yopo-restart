# scripts/sitl：box 上的 PX4 SITL（票 S1 单机，票 S2 三机骨架，report-only）

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

## 票 S2：三机骨架（report-only，判据未冻结）

在同一套镜像上再拉三架 x500。不另改 PX4 安全参数；`--positioning rtk` 仍只注入 S1b 白名单。
本说明没有实测数字。误差写在每次运行的 `result.json` 里，在 box 上跑完才有。

```bash
scripts/sitl/up.sh --n 3          # 镜像没有就先构建，然后拉起 3 架（headless，不起飞）
scripts/sitl/up.sh --n 3 --status
scripts/sitl/up.sh --n 3 --down
scripts/sitl/smoke.sh --n 3       # 三机各自起飞到 1.5 m 悬停点，采样后再降落
scripts/sitl/smoke.sh --n 3 --positioning rtk
```

`smoke.sh --n 3` 如果发现 `up.sh --n 3` 的容器还在，就在里面补跑任务；否则自己起一趟仿真并飞完。
`--n` 省略时仍是 S1 单机任务（2.5 m、平移、降落），不是这条 1.5 m 悬停。

身份按 PX4 v1.17 的 posix `rcS`，脚本不写这些参数：

| 机 | `px4 -i` | MAV_SYS_ID | uXRCE namespace | Gazebo 模型 | GCS UDP | 生成点（x,y） |
|---|---|---|---|---|---|---|
| 1 | 1 | 2 | `px4_1` | `x500_1` | 18571 | 0, 0 |
| 2 | 2 | 3 | `px4_2` | `x500_2` | 18572 | 0, 3 |
| 3 | 3 | 4 | `px4_3` | `x500_3` | 18573 | 0, 6 |

- instance 0 没有 namespace，所以三机从 instance 1 起，namespace 才是 `px4_1`..`px4_3`。`MAV_SYS_ID` 因此是 2、3、4。
- 一个 Micro-XRCE-DDS Agent（端口 8888），一个 DDS 域（`ROS_DOMAIN_ID=0`，三机相同）。不设 `PX4_UXRCE_DDS_NS`。
- 第一架拉起 Gazebo，其余 `PX4_GZ_STANDALONE=1` 接进同一个世界。`HEADLESS=1`。生成点在 y 上隔 3 m，避免叠在一起。
- 每机：起飞到自己起飞点上方 1.5 m → 悬停 10 s（仿真时间，只为采样）→ PX4 自己的 NAV_LAND。不飞 S1 那个平移点。
- `result.json` 的 schema 是 `yopo-restart/s2-sitl-fleet/1`。`report_only=true`，`criteria_frozen=false`。
  `ok` 只表示每机都落地并上锁。悬停高度误差（估计 / Gazebo 真值）和估计相对真值的高度偏差按机记录，不设门。
- 仍是一次只起一趟仿真。容器名 `yopo-restart-sitl-n3`。输出在 `/workspace/yopo-ops/sitl/runs/<时间>-n3/`。
- 不进 `scenes/suite.txt`，不进 regress 的场景批跑。单测只覆盖布局、任务状态机和 `result.json` / 表的生成。

## 票 S3：三机协同点到点与错高对飞（report-only，判据未冻结）

在 S2 的三机进程上加任务，不改 PX4 安全参数，不改 `yopo_l0/` 或 `core/`。
ENU↔NED 仍只走 `yopo_swarm_core.px4`。默认 `--mission hover`，所以不带这个参数时 S1 单机和 S2 三机悬停与以前相同。

```bash
scripts/sitl/smoke.sh --n 3 --mission goto    # 每机在 1.5 m 上飞到悬停点 +(6, 0, 0) m
scripts/sitl/smoke.sh --n 3 --mission swap    # 三机错高 1.5 / 2.2 / 2.9 m，两端换位
```

容器里仿真已经起来时，直接跑任务（`YOPO_SITL_MISSION=0` 仍然只留着仿真、不起飞）：

```bash
YOPO_SITL_N=3 YOPO_SITL_MISSION_MODE=swap \
  python3 /repo/scripts/sitl/fleet.py run --out /out/result.json --n 3 --mission swap
```

`up.sh --n 3` 拉起的容器还在时，`smoke.sh` 会把 `YOPO_SITL_MISSION_MODE` 传进 `exec_mission.sh`。那个脚本不读 `--mission`，`fleet.py` 自己看环境变量。

- `hover`：每机起飞到本机起飞点上方 1.5 m，悬停 10 s，NAV_LAND。不进 GOTO。
- `goto`：悬停之后一起进入 GOTO，目标是悬停点 + `(6, 0, 0)` m（相对起飞点的水平偏移，高度不变）。
- `swap`：只用于 3 架。巡航高度按机体顺序是 1.5 / 2.2 / 2.9 m。两端两架交换 y（0 ↔ 6），中间一架留在 y=3，只站在 2.2 m 这一层。层间距 0.7 m，避免还没接规划器时在 SITL 里对冲相撞。
- 路点在起飞前做同步直线的 3D 间隔检查（`min_geometric_separation_m`）。间隔小于 0.7 m 的计划直接拒绝，不会发给飞控。
- `result.json`：hover 仍是 `yopo-restart/s2-sitl-fleet/1`。goto / swap 是 `yopo-restart/s3-sitl-crossing/1`。
  两者都写 `report_only: true`、`criteria_frozen: false`。
  每机有 `goto_err_m`、`goto_err_truth_m`、`hover_est_truth_alt_bias_mean_m`（hover 任务里到点误差是空的）。
  多机全程真值最小机间距离是 `min_inter_drone_m`，来自各机 trace，或 `/out/px4_<instance>/trajectory.csv`。
- 单机 `--n 1` 不传 `--mission`（或 hover）时仍走 S1，不会变成 1.5 m 原地降落。单机 `goto` 使用同一个 +6 m 偏移。单机不能 `swap`。
- 不进 `scenes/suite.txt`，不进 regress。数字只在 box 上跑完才有，这里不写实测值。

## 票 S4b：YOPO L0 规划桥接进容器（report-only）

默认的 `smoke.sh` / `up.sh` 不设 `YOPO_SITL_PLANNER`，仍走 `offboard_mission.py` / `fleet.py`。
实飞入口是单独的脚本，只传场景、机数、种子和定位模式：

```bash
scripts/sitl/run_yopo_l0_sitl.sh --n 1
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --positioning rtk
scripts/sitl/run_yopo_l0_sitl.sh --dry-run --n 3 --positioning rtk
```

1 机默认 `scenes/g2-pillar.json`，话题在 `/fmu/...`。3 机默认 `scenes/swap-3-open.json`，namespace 是 `px4_1`、`px4_2`、`px4_3`。
`--positioning rtk` 仍只走原来的 `positioning-rtk.env` 白名单，规划桥本身不写飞控参数。
宿主机缓存 `~/.cache/yopo-swarm` 若已存在，只读挂到容器 `/yopo-cache`。容器以 root 跑，
`docker run` 设置 `safe.directory=*`，避免 git 把这份缓存当成 dubious ownership 后再往只读挂载写 `.partial`。
预检只在第一次全体通过时锁存；着陆上锁后 PX4 拉低 `pre_flight_checks_pass` 不会停 step，也不记成启动预检失败。
任务结束时把 `/out` 的属主改回调用者，和 `smoke.sh` 一样。
离线假 PX4 仍是 `scripts/sitl/run_yopo_l0.py`（`--fake-px4` 与缺省相同）。说明和实测数字在 `docs/research/s4b-sitl-live.md`。

## 票 S4c：YOPO L0 运行时分层镜像（report-only）

`scripts/sitl/Dockerfile` 仍然只有 ROS 2 Humble、PX4、Gazebo、Micro-XRCE-DDS-Agent 和 `px4_msgs`。
不往那份文件里加 `torch`，S1/S1b 的镜像哈希不变。YOPO 运行时在第二层，基础镜像要先由 `up.sh` 准备好：

```bash
scripts/sitl/build_yopo_image.sh --print-image
scripts/sitl/build_yopo_image.sh --status
scripts/sitl/build_yopo_image.sh            # 基础镜像没有就退出，提示先跑 up.sh，不重编 PX4
scripts/sitl/build_yopo_image.sh --bg       # setsid 后台构建，PID 和日志在 $SITL_STATE
```

标签是 `yopo-restart-sitl:yopo-<Dockerfile 与 Dockerfile.yopo 拼接后的哈希>`。
`run_yopo_l0_sitl.sh` 在这层镜像已经存在时改用它。`--image` 或 `YOPO_SITL_IMAGE` 显式指定时用指定标签。
`docker run` 之前检查这个标签在不在。`--dry-run` 仍打印原来的 planner、场景、机数、定位和种子，并多一行 `image=`。
`up.sh` / `smoke.sh` 不读这层镜像。说明在 [s4c-sitl-yopo-image.md](../../docs/research/s4c-sitl-yopo-image.md)。

