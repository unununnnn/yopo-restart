# D2：EGO-Swarm 教师数据

把仓库外的 [ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 提交 `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20` 当成教师，无头批量飞，记下轨迹，写成 L0 能读的 npz 和场景卡。源码在 `~/ext/ego-planner-swarm`。没有放进本仓库，也没有加子模块。

不改本仓库的规划器，不改冻结门，不改 `README`，不改 `docs/tickets.md`。也没有改那份外部源码。

测不到的量写「未知」。下面凡是本管线自己定的数，都写「假设」。

## 1. 假设

论文表 II（arXiv:2011.04183v3）没有给障碍半径和地图边长。这两个数写在 [assumptions.py](../../scripts/teacher/assumptions.py) 的文件头，是本管线选的，不是论文复现：

| 量 | 值 | 出处 |
|---|---|---|
| 障碍半径 | 0.20 m，竖直圆柱，半径固定 | 假设 |
| 地图边长 | 20.0 m，正方形，面积 400 m² | 假设 |
| 密度 0 / 0.14 / 0.28 / 0.42 的圆柱数 | 0 / 56 / 112 / 168 | 个数 = 密度 × 400，四档都是整数 |
| 机体半径 | 0.20 m | 论文表 II 写了 0.2 m。这次的 `ego_planner_node` 不打印碰撞用的机体半径，判定用这个数，标成假设 |
| 航程 | x 从 −9 m 到 +9 m，高度 1.0 m | 假设。上游 `swarm.launch` 是 x 从 −20 到 20、起点 z = 0.1。这里缩短，小批量才能跑完 |
| 横向槽位 | 1 架 y = 0；3 架 −6、0、6；6 架 −7.5 到 7.5、间隔 3 m | 假设 |
| 到达 | 目标 0.5 m 内，速度 ≤ 0.3 m/s，连续保持 1.0 s | 假设。论文没有到达半径 |
| 规划栅格 | 24 × 24 × 4 m | 假设。要比 20 m 的障碍场大，才能包住 x = ±9 |
| 采样 | 10 Hz | 假设 |

速度上限 2.0 m/s 和规划地平线 7.5 m 写在该提交的 `run_in_sim.launch` 里。论文表 II 的速度上限也是 2 m/s。加速度 3.0 m/s² 只在 launch 里，论文表 II 没有写。

单机没有第二架可以交换。对向和平行的起终点相同，障碍仍按图案编号分开抽，所以不是同一份地图飞两次。

圆柱圆心离起点和终点的水平距离至少是 0.20 + 0.20 + 0.30 m。圆柱彼此圆心距至少 0.40 m，圆盘落在正方形里面。放不满个数时 `placement_complete` 为 false，实际密度按放进去的个数除以 400。这一批 192 个场景都放满了。

## 2. 几何为什么不交给 random_forest

该提交的 `random_forest_sensing.cpp` 用 `std::random_device` 做种子。`swarm.launch` 里的 `ObstacleShape/seed` 没有被读。同一组参数两次地图不一样，L0 卡对不上教师看见的障碍。

圆柱在 `scripts/teacher/scenarios.py` 里用 `numpy` 的 `SeedSequence` 生成。容器里的 `ros_record.py` 把这些圆柱栅格化成点云，发到 `/map_generator/global_cloud`。不启动 `random_forest`。点云分辨率 0.1 m，和上游体素叶子一样。

密度为 0 时仍发一个点 `(0, 0, −10)`。上游 CPU 渲染节点对空云建 kdtree。这个点在 5 m 感知范围外，不写进 L0 场景卡。

教师点云没有地面。L0 卡的 `ground_z = 0` 只参加 L0 的碰撞真值。飞行高度 1 m 时，离地面大约 1 m。

`local_sensing` 的 `ENABLE_CUDA` 在该提交里是 false，所以深度节点实际编的是 `pointcloud_render_node`。规划器吃局部点云。L0 的深度不从这里拷：读数据时按场景卡用 `yopo_l0.sensor.DepthCamera` 现算。训练和评估看见的是同一批圆柱。

## 3. 场景格

4 个密度 × 3 种机数 × 2 种图案 × 8 个种子（0 到 7）= 192。200 不能被 24 整除。小批量取每一格同样多的 8 个种子，而不是 200。

图案：

- `parallel`：每架的目标 y 等于自己的起点 y。
- `headon`：目标 y 把起点 y 的顺序对调。

## 4. 容器里的教师

镜像 `osrf/ros:noetic-desktop-focal`，digest `sha256:85ff4e2f691f7eb5673835afe7e70cd49e552e3cdb8b1938b7c54b647d93b99a`。和 E-C1 那次是同一个 digest。

动力学是 `poscmd_2_odom`，也就是 README 第 5 节说的 `fake_drone`：指令直接写成里程计。没有 `quadrotor_simulator_so3`。不启动 rviz，也不启动 `odom_visualization`。

编译顺序：先 `catkin_make -j2 multi_map_server_generate_messages_cpp`，再 `catkin_make -j2`。E-C1 记下第一次完整的 `catkin_make -j1` 会缺 `MultiOccupancyGrid.h`。这次用 `-j2`，不是 E-C1 记录里的 `-j1`。没有改 CMake。

每个场景自己起一个 `roscore`，飞完就停。到点（位置和速度都进到达门，并保持 1 s）就结束，否则 50 s 上限。`use_sim_time` 没有打开。npz 里的时间是相对第一拍的容器墙钟，不是论文的仿真钟。

## 5. 记了什么

| 量 | 从哪来 |
|---|---|
| 位置、速度 | `/drone_i_visual_slam/odom` |
| 加速度 | 同一条里程计的 `twist.angular`。该提交的 `poscmd_2_odom.cpp` 把指令加速度写在这里 |
| 偏航 | `PositionCommand.yaw`。没有指令时记 0。里程计四元数是由加速度倾出来的姿态，不用它当航向 |
| 邻机广播 | `/broadcast_bspline` 的阶次、knots、控制点、yaw |
| 本机样条 | `/drone_i_planning/bspline`，同一套字段 |
| 障碍 | 生成时的圆柱，原样进 npz 和场景卡 |
| 规划时间 | 日志里 `total time:` 后面的秒数。颜色码按 E-C1 的方式剥掉。一条都没有时，均值、分位数写未知 |
| 急停 | `Suddenly discovered obstacles. emergency stop!` 和 `to EMERGENCY_STOP` 的次数 |

日志行没有稳定地标上节点名时，分机的规划时间均值是未知。这一列只在能把每一行归到某一架、而且行数和总条数一致时才填。

`total time` 是不是论文表 II 的 tcal，论文和 README 都没有写。这里只报这次日志里的数，不把它叫做 tcal。

## 6. 成功、碰撞、间距

和 L0 的 goto 用同一把尺子，距离函数是 `yopo_l0.scene`。

- 到达：一段连续样本里，每架都在目标 0.5 m 内，速度也不超过 0.3 m/s，时间跨度至少 1.0 s 减去一拍（10 Hz 的相位）。跨度不够就是没到达。
- 障碍碰撞：表面距离 ≤ 0.20 m。地面算在里面。
- 机间碰撞：中心距 ≤ 0.40 m。
- 成功：到达，并且两种碰撞都没有。
- 没有样本（进程没起来，或没有一拍凑齐全部里程计）：成功、到达、碰撞都是未知，不记成失败。
- 单机没有机间距离，`min_inter_drone_m` 是未知。单机的机间碰撞记 false，因为没有一对。

上游 `advanced_param.xml` 的 `obstacles_inflation` 是 0.099 m。教师可以离表面 0.16 m 仍不打印急停，但按假设的 0.20 m 机体半径，这一下算碰撞。成功率为 0 的格子要和「最近表面距离」一起看，不能单独读成规划器报了急停。

最小机间距离有两列：全部样本，以及至少有一架离自己起点超过 1 m 的那些样本。1 m 只是这次脚本的标记，不是论文参数。没有这种样本时，飞行中的那一列是未知。

## 7. L0 能读的格式

一个场景一个目录：

- `trajectory.npz`：`time_s`、`position_m`、`velocity_mps`、`acceleration_mps2`、`yaw_rad`，以及圆柱的圆心、半径、高度。形状写在 [schema.json](../../scripts/teacher/schema.json)。
- `record.json`：模式 `yopo-teacher-record/1`。里面是场景原文、运行状态、指标、广播和本机 B 样条。JSON 的 null 表示未知。
- `scene.json` + `geometry.json`：`yopo_l0.scene.load_card` 能读。任务是 `goto`，每架有自己的 `goal_enu`。`criteria.frozen` 是 false。不写入 `scenes/suite.txt`。

`observation_at(目录, 飞机下标, 样本下标)` 返回：

- 深度：L0 光线投射，默认 96×160，米，无回波是 NaN。场景卡把 `noise_std_m` 设成 0，所以同一姿态只由几何决定。`scenes/open-3drone.json` 是 0.02，这里故意不同，写在卡的 `teacher.noise_std_note`。
- 状态：该拍的位置、速度、加速度、偏航，包成 `yopo_l0.policy.Observation`。
- 邻机：其它飞机该拍的状态，加上接收时间不晚于该拍的最后一条 `/broadcast_bspline`。还没有广播时，这一项是未知（null）。

相机、动力学、频率从 `scenes/open-3drone.json` 抄来，只改了噪声。机体半径用上面的 0.20 m，不是那张卡的 0.35 m。

## 8. 小批量统计

192 个场景都有样本，未知是 0。格子是「成功数 / 有结论的场景数」。

| 密度 obs/m² | 1 架 | 3 架 | 6 架 | 该密度合计 |
|---|---|---|---|---|
| 0 | 16/16 | 16/16 | 16/16 | 48/48 |
| 0.14 | 12/16 | 6/16 | 2/16 | 20/48 |
| 0.28 | 12/16 | 1/16 | 0/16 | 13/48 |
| 0.42 | 9/16 | 0/16 | 0/16 | 9/48 |

| 机数 | 成功率 |
|---|---|
| 1 | 49/64 |
| 3 | 23/64 |
| 6 | 18/64 |

| 图案 | 成功率 |
|---|---|
| 平行 | 42/96 |
| 对向 | 48/96 |

合计 90/192。102 场失败里，101 场已经进了到达门，但表面距离 ≤ 0.20 m，按假设的机体半径记成障碍碰撞。机间碰撞是 0。剩下 1 场是 `d042-n6-headon-s5`：采样跨度 19.8 s 之后里程计不再刷新，墙钟 90.2 s 时按超时停下，有两架离目标还超过 1 m，同时也擦到了障碍。密度 0 的最近表面距离是地面，48 场都在 0.9998 m 到 1.0000 m 之间。

失败场里，表面距离最小 0.015 m，中位 0.128 m，最大 0.199 m。上游 launch 的 `obstacles_inflation` 是 0.099 m，所以这些数里有一部分教师自己没有当碰撞。

急停日志 `Suddenly discovered obstacles. emergency stop!` 出现 2 次，分别在 `d042-n1-parallel-s5` 和 `d028-n6-parallel-s6`。下一行都是切到 `EMERGENCY_STOP`。计数只取前一行，避免一件事算两次。

规划时间是日志里的 `total time`。`plan_success=1` 有 10318 次，解析出的 `total time` 有 10317 条。差的那 1 条对不上行，耗时未知。10317 条的算术平均是 0.000430008 s，也就是 0.430 ms。p95 用下标 `ceil(0.95 × 10317) - 1 = 9801`，值是日志里的 0.0014 s，也就是 1.4 ms。最小 4.9225e-05 s，最大 0.045 s。分机均值未知：这些行没有稳定的节点前缀，192 场都归不到某一架。

`run --limit 192` 的墙钟是 5181.0 s，192 / 5181.0 × 3600 = 133.4 场景/小时。其中 `d028-n3-parallel-s1` 和 `d042-n1-parallel-s7` 各挂了约 299 s 和 315 s，采样只有 7.2 s 和 1.5 s，主机的 75 s 上限没有把它们按时杀掉。这两场后来重跑，墙钟 19.2 s 和 18.5 s，都到达了，但表面距离低于 0.20 m，成功仍是 false。把原来的两段墙钟换成重跑的，总墙钟是 4604.8 s，192 / 4604.8 × 3600 = 150.1 场景/小时。表用的是重跑之后的成败。

逐场数字在 [d2-teacher-batch.json](d2-teacher-batch.json)。全量轨迹在 `/tmp/teacher-d2/runs`，不进 git。种子 0 的 24 场在 `data/teacher/sample/`，大约 9.6 MB。

## 9. 机器

| 项 | 值 |
|---|---|
| 机器型号 | 未知。`/sys/class/dmi/id` 不存在 |
| 虚拟化 | KVM，full（`lscpu` 的 Hypervisor vendor、Virtualization type） |
| CPU | Intel(R) Xeon(R) Processor，family 6，model 207，stepping 2，1 路 4 核，每核 1 线程，`cpu MHz` 2400.000，BogoMIPS 4800.00 |
| 内存 | MemTotal 16387328 kB |
| 宿主机 | Ubuntu 24.04.5 LTS，内核 `6.12.113+`，`x86_64` |
| Docker | 29.1.3。没有 systemd，守护进程是手工拉起的 |
| 容器 | `osrf/ros:noetic-desktop-focal`，digest `sha256:85ff4e2f691f7eb5673835afe7e70cd49e552e3cdb8b1938b7c54b647d93b99a` |
| 教师提交 | `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20` |

论文的基准机是 i7-9700KF。和这台 CPU 的对应关系未知。

## 10. 再跑一次

一条命令是 `scripts/teacher/generate.sh`。它不启动 dockerd。这台机器没有 systemd，守护进程是手工拉起的：

```bash
sudo dockerd --host=unix:///var/run/docker.sock --iptables=false --storage-driver=overlay2
```

然后：

```bash
scripts/teacher/generate.sh --help
scripts/teacher/generate.sh all --dry-run
scripts/teacher/generate.sh all --limit 192
```

`all` 会克隆（若还没有）、编译、写出 192 个场景、逐个飞、把种子 0 的 24 个拷到 `data/teacher/sample/`、并把汇总写到 `docs/research/d2-teacher-batch.json`。已经有 `record.json` 的场景会跳过。要重飞加上 `--no-resume`。只要某几个：

```bash
scripts/teacher/generate.sh run --id d000-n1-parallel-s0
scripts/teacher/generate.sh run --id d042-n6-headon-s0 --timeout 70
```

全量不进仓库。上面的 `all --limit 192` 就是生成命令。更大的网格改 `assumptions.py` 里的 `SEEDS` 再跑同一条命令，输出仍在 `--work`（默认 `/tmp/teacher-d2`）下面。

小样本是种子 0 的每一格，外加 `schema.json`。深度不在文件里，用 `observation_at` 现算。
