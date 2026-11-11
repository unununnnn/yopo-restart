# EGO-Swarm 外部基线（E-C1）

这次在本仓库之外编译并运行 [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 的提交 `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20`（2025-03-08，提交说明 Update README.md）。源码在 `~/ext/ego-planner-swarm`，没有放进本仓库，也没有加子模块。

表里的出处只有三种：论文指 arXiv:2011.04183v3（2021-05-05）的正文、表 I、表 II、图 9；README 指上述提交里的 `README.md`；本次运行指下面这一次无头 `swarm.launch`。某一格三个出处都没有写到的数，值写「未知」。不把 launch 参数换算成论文的密度或平均速度。

计时、节点列表和里程计来自同一次运行：容器里先启动里程计采样，再 `roslaunch ego_planner swarm.launch`，脚本在 launch 起来之后 sleep 120 秒，然后发 SIGINT。在此之前单独探过一次没有显示器的 `simple_run.launch`，那次的数字不进表。

## 1. 运行环境

| 项 | 值 | 出处 |
|---|---|---|
| 机器型号 | 未知。`/sys/class/dmi/id` 不存在，读不到 product_name | 本次运行 |
| 虚拟化 | KVM，full | 本次运行（`lscpu` 的 Hypervisor vendor、Virtualization type） |
| CPU | `Intel(R) Xeon(R) Processor`，vendor GenuineIntel，family 6，model 207，stepping 2，1 路 4 核，每核 1 线程，`cpu MHz` 2400.000，BogoMIPS 4800.00 | 本次运行（`/proc/cpuinfo`、`lscpu`） |
| 内存 | MemTotal 16387328 kB | 本次运行（`/proc/meminfo`） |
| 宿主机 | Ubuntu 24.04.5 LTS，内核 `6.12.113+`，`x86_64` | 本次运行 |
| 宿主机上的 ROS | 没有。`/opt/ros` 不存在 | 本次运行 |
| 容器镜像 | `osrf/ros:noetic-desktop-focal`，digest `sha256:85ff4e2f691f7eb5673835afe7e70cd49e552e3cdb8b1938b7c54b647d93b99a` | 本次运行 |
| 容器系统 | Ubuntu 20.04.6 LTS，gcc 9.4.0，`ros-noetic-desktop` 1.5.0-1focal.20250521.014654 | 本次运行 |
| Docker | 29.1.3。这台机器没有 systemd，`systemctl start docker` 失败后，用 `dockerd --host=unix:///var/run/docker.sock --iptables=false --storage-driver=overlay2` 拉起。容器 `--network=host` | 本次运行 |
| 论文基准机 | i7-9700KF | 论文第 VI 节 |
| 论文机与这次 CPU 的对应 | 未知。两边的型号字符串不同，没有换算 | 论文、本次运行 |

README 第 2 节写的编译环境是 Ubuntu 16.04、18.04 或 20.04，并且已经装好 ros-desktop-full。这次宿主机是 24.04，ROS 装在 20.04 容器里。用的元包是 `ros-noetic-desktop`，不是 desktop-full。

## 2. 编译

README Quick Start 的命令是 `sudo apt-get install libarmadillo-dev`，然后 `catkin_make -j1`，没有 `-DCMAKE_BUILD_TYPE`。第 2 节的标准编译写的是 `catkin_make -DCMAKE_BUILD_TYPE=Release -j1`。这次按 Quick Start，没有在命令行加 `CMAKE_BUILD_TYPE`。

容器里在 `libarmadillo-dev` 之外还装了这些包，因为 desktop 镜像里没有它们，第一次配置会缺头文件和库：`ros-noetic-pcl-ros`、`ros-noetic-pcl-conversions`、`ros-noetic-cv-bridge`、`ros-noetic-image-transport`、`ros-noetic-tf`、`ros-noetic-laser-geometry`、`ros-noetic-nodelet`、`ros-noetic-cmake-modules`、`libpcl-dev`、`libeigen3-dev`。

| 项 | 值 | 出处 |
|---|---|---|
| 第一次 `catkin_make -j1` | 失败。`multi_map_visualization.cc:5:10: fatal error: multi_map_server/MultiOccupancyGrid.h: No such file or directory` | 本次运行 |
| 该提交里的依赖目标名 | `src/uav_simulator/Utils/multi_map_server/CMakeLists.txt` 把 `multi_map_visualization` 依赖写成 `multi_map_server_messages_cpp`。生成 C++ 头的目标名是 `multi_map_server_generate_messages_cpp` | 本次运行（读该提交的 CMakeLists） |
| 接着执行的命令 | `catkin_make -j1 multi_map_server_generate_messages_cpp`，然后再次 `catkin_make -j1`。退出码 0 | 本次运行 |
| 顶层 `CMAKE_BUILD_TYPE` | 空字符串 | 本次运行（`build/CMakeCache.txt`） |
| `ego_planner_node` 的 CXX_FLAGS | `-O3 -Wall -g -std=c++11 -std=c++14` | 本次运行（`build/planner/plan_manage/CMakeFiles/ego_planner_node.dir/flags.make`） |
| 这些 `-O3` 从哪来 | `src/planner/plan_manage/CMakeLists.txt` 自己写了 `set(CMAKE_BUILD_TYPE "Release")` 和 `set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")` | 本次运行（读该提交的文件） |

没有改上游源码。

## 3. 启动文件和动力学

README Quick Start 的最后一条是 `roslaunch ego_planner simple_run.launch`。该提交的 `simple_run.launch` 包含 `rviz.launch` 和 `swarm.launch`。`rviz.launch` 里的 rviz 节点带 `required="true"`。

没有设置 `DISPLAY` 时，单独跑 `simple_run.launch`：rviz 进程退出码 -6，roslaunch 打印 `REQUIRED process [rviz-2] has died!`，随后整次 launch 停掉。把 rviz 再单独拉起来，stderr 是：

```text
qt.qpa.xcb: could not connect to display
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "" even though it was found.
This application failed to start because no Qt platform plugin could be initialized.
```

所以 Quick Start 这条命令在这台没有显示器的机器上没有把规划器留住。下面的数字来自不包含 rviz 的 `roslaunch ego_planner swarm.launch`。

`swarm.launch` 包含 10 次 `run_in_sim.launch`，`drone_id` 从 0 到 9。起点 x 都是 -20、z 都是 0.1，y 从 -9 到 9、间隔 2。目标 x 都是 20、z 都是 1，y 从 9 到 -9、间隔 -2。也就是沿 x 飞过地图，同时把 y 的顺序对调。论文表 I 是 8 架在圆上交换。论文表 II 是 10 架从地图一边到另一边，并在地图中央对向。这次 launch 文件没有写圆半径，也没有写 obs/m²。不把这次叫做表 I 或表 II 的复现。

README 第 5 节：默认用 `fake_drone` 里的做法，把命令直接变成里程计；`quadrotor_simulator_so3` 和 `so3_control` 在 `simulator.xml` 里是注释掉的。

| 项 | 值 | 出处 |
|---|---|---|
| 这次计时用的 launch | `ego_planner swarm.launch` | 本次运行 |
| Quick Start 写的 launch | `ego_planner simple_run.launch` | README |
| `simple_run` 在无 DISPLAY 时 | rviz 退出码 -6，launch 停掉 | 本次运行 |
| 默认动力学 | README 写默认 `fake_drone` | README 第 5 节 |
| 这次进程 | 10 个 `/drone_*_poscmd_2_odom`，10 个 `/drone_*_ego_planner_node`。`rosnode list` 里没有 `quadrotor_simulator_so3`，也没有 `so3` | 本次运行 |
| 机数 | 10 | 本次运行；与 `swarm.launch` 里 drone_id 0–9 一致 |
| 论文表 I 机数 | 8 | 论文 |
| 论文表 II 机数 | 10 | 论文第 VI.B.2 节 |
| 速度上限参数 | `max_vel` 2.0，`max_acc` 3.0 | 本次运行（roslaunch 参数打印）。同一组数写在该提交的 `run_in_sim.launch` |
| 规划地平线参数 | `planning_horizon` 7.5 | 本次运行的参数打印；论文第 VI 节也写 7.5 m |
| 论文表 II 速度上限 | 2 m/s | 论文 |
| 这次的平均速度 | 未知。没有沿轨迹积分，也不把位移除以时间 | 本次运行 |
| 论文表 II 平均速度 | 密度 0.42 / 0.28 / 0.14 / 0 对应 1.61 / 1.59 / 1.57 / 1.55 m/s | 论文表 II |
| 机体半径 | 未知。这次的日志和 `swarm.launch` 没有打出碰撞用的机体半径。`odom_visualization` 的 `robot_scale` 是 1.0，那是显示缩放 | 本次运行 |
| 论文表 II 机体半径 | 0.2 m | 论文 |
| 地图参数 | `map/x_size` 36，`map/y_size` 20，`map/z_size` 3，`map/obs_num` 200，`map/circle_num` 200，`ObstacleShape/seed` 1 | 本次运行读 `swarm.launch` |
| 这次的障碍密度 obs/m² | 未知。launch 给的是个数和边长，没有写成 obs/m²，论文也没有说明圆柱和圆环怎样一起计入密度 | 论文、README、本次运行 |
| 论文表 II 密度 | 0.42、0.28、0.14、0 obs/m² | 论文表 II |

为了让管道里的 `cout` 按行刷出，这次在 launch 前设置了 `LD_PRELOAD=/tmp/linebuf.so`。这个库只调用 `setvbuf` 把 stdout/stderr 改成行缓冲。`total time` 是进程里 `ros::Duration::toSec()` 的差，不读这个库。

## 4. 日志里的规划时间

成功一次重规划后，`planner_manager.cpp` 打印一行：

```text
total time:<秒>,optimize:<秒>,refine:<秒>,avg_time=<秒>
```

秒数两侧有 ANSI 颜色码。单位是秒。下面的毫秒是这些秒乘 1000。

`plan_success=0` 出现 32 次。失败的分支在打印 `total time` 之前返回，所以这 32 次没有对应的 `total time`，它们的耗时未知。`plan_success=1` 与 `total time` 都是 401 次。`IN SWARM MODE, REFINE DISABLED!` 出现 9 次。合并后的日志里各机输出交错，没有把某一行归到某一架。下面是 401 行 `total time` 放在一起的统计。

p95 的算法：把日志里印出的十进制数升序排列，取下标 `ceil(0.95 × 401) - 1 = 380`（从 0 起，也就是第 381 个）。

| 项 | 值 | 出处 |
|---|---|---|
| `total time` 条数 | 401 | 本次运行 |
| 401 个数之和 | 0.689247664 秒（按日志里的十进制文本相加） | 本次运行 |
| 均值 | 0.689247664 / 401 = 0.001718822104738154613466334165 秒 = 1.718822104738154613466334165 毫秒 | 本次运行 |
| p95 | 日志文本 `0.0063` 秒 = 6.3 毫秒 | 本次运行 |
| 最小 | 日志文本 `6.4e-05` 秒 | 本次运行 |
| 最大 | 日志文本 `0.042` 秒 | 本次运行 |
| 失败的 32 次耗时 | 未知 | 本次运行 |
| 分机均值 | 未知 | 本次运行 |
| 论文表 I，本方法 tcal | 0.514 ms。表注：这是单机局部重规划时间；DMPC 的 0.668 s 和 RBP 的 1.18 s 是整段轨迹的总时间 | 论文表 I |
| 论文表 II tcal | 5.00、2.87、1.21、0.49 ms，对应密度 0.42、0.28、0.14、0 | 论文表 II |
| 论文图 9 | 本方法 2 条候选、0.017 ms；Fast-Planner 5 条、7.63 ms。图注写明这只是前端找极小的时间 | 论文图 9 |
| 这次的 `total time` 是不是论文的 tcal | 未知。论文没有写这行日志，README 也没有写 | 论文、README |

论文第 VI 节的权重是 `λs = 1.0`，`λc = λw = λt = 0.5`，`λd = 0.1`。这次参数打印里有 `optimization/lambda_smooth` 1.0、`lambda_collision` 0.5、`lambda_feasibility` 0.1、`lambda_fitness` 1.0、`optimization/swarm_clearance` 0.5。这些名字和论文符号是不是一一对应，论文和 README 都没有写，记为未知。

## 5. 碰撞和到达指示

| 项 | 值 | 出处 |
|---|---|---|
| `Suddenly discovered obstacles. emergency stop!` | 0 次 | 本次运行 |
| `Depth Lost! EMERGENCY_STOP` | 0 次 | 本次运行 |
| 状态迁移到 `EMERGENCY_STOP` | 0 次 | 本次运行 |
| 日志里的 `collision` | 10 次，全部是参数名 `optimization/lambda_collision: 0.5`，每个规划器一行 | 本次运行 |
| `Close to goal` | 0 次。源码里这行在起点离局部目标小于 0.2 m 时打印，然后这次重规划返回失败 | 本次运行；0.2 m 来自该提交的 `reboundReplan` |
| 到达计数 | 未知。日志没有到达计数 | 本次运行 |
| 与障碍物的最近距离 | 未知。没有对点云或栅格做距离 | 本次运行 |
| 实际相撞次数 | 未知。上面几行指示都是 0，但程序没有打印「撞上了」的计数 | 本次运行 |
| 论文表 I 碰撞次数 | 本方法每架 0。同表 ORCA 是 0.375 | 论文表 I |
| 论文表 II 碰撞次数 | 未知。正文写每架都规划出平滑、安全的轨迹，没有给次数 | 论文 |
| 论文表 II 的 dsafe | 0.208、0.22、0.266 m，密度 0 那一格是 `/`。正文写飞行中离障碍的最近距离。算的是机体中心还是表面，未知 | 论文 |

## 6. 里程计：目标距离和机间距离

采样程序订阅 `/drone_0_visual_slam/odom` 到 `/drone_9_visual_slam/odom`。脚本请求 20 Hz。某一拍要同时具备 10 个位置，并且每个位置的墙钟年龄都不超过 0.2 秒，才记为一个同步样本。0.2 秒和 20 Hz 是这次脚本写的，不是论文或 README 的参数。机间距离是这一拍上 45 对位置的欧氏距离里最小的那个，再对全部同步样本取最小。目标是 `swarm.launch` 里该机的 `target_x/y/z`。

同步样本 2379 个。其中 2327 个样本里至少有一架离自己的起点超过 1.0 m。1.0 m 只用来给样本打「已经离开起点」的标记。全体样本的最小机间距离和这 2327 个样本上的最小机间距离是同一个数。

第一条观察日志的 ros 时间是 1794375339.660103，当时已经有 87 个样本，10 架离起点大约 2.6 m 到 3.3 m，运行中的最小机间距离是 1.0196 m。最后一条观察日志是 1794375450.410249，样本数 2302。两条日志相差 110.750146 秒。json 里的最终样本数是 2379，比最后一条观察日志多出来的那一段没有单独的时间戳。`use_sim_time` 没有打开，这些 ros 时间是容器的墙钟。

| 项 | 值 | 出处 |
|---|---|---|
| 最小机间距离 | 0.7557993214529912 m，发生在 drone 4 与 drone 5，ros 时间 1794375342.454349 | 本次运行 |
| 起点相邻两架的 y 间隔 | 2 m（launch 里 y 为 -9,-7,…,9） | 本次运行读 `swarm.launch` |
| 论文的最小机间距离 | 未知 | 论文 |
| 飞行距离 dfly | 未知 | 本次运行 |
| 飞行时间 tfly | 未知。没有把「第一次进入某个半径」记成到达时刻 | 本次运行 |
| 论文表 I 本方法 | 平均飞行距离 20.3 m，飞行时间 14.3 s，每架碰撞 0，tcal 0.514 ms | 论文表 I |
| 圆的半径、这次交换用的最大速度和最大加速度 | 未知。正文只说除了最大速度和最大加速度，其余用各方法的默认参数 | 论文 |

各机到自己目标的最小欧氏距离，以及窗口结束时由最后一拍坐标重算的距离：

| 机 | 采样中的最小目标距离 m | 结束时的目标距离 m | 结束时位置 x, y, z | 出处 |
|---|---|---|---|---|
| 0 | 0.05955784816096655 | 0.059557848160966556 | 19.94415675375258, 8.979323464366582, 0.9989276193237817 | 本次运行 |
| 1 | 0.1280482145825344 | 0.1280482145825344 | 19.878303166876844, 6.960256667499538, 0.9974128031912657 | 本次运行 |
| 2 | 0.028281631654423402 | 0.028281631654423402 | 19.97226614097952, 4.994479866514714, 0.9995396966507878 | 本次运行 |
| 3 | 0.17765592522906742 | 0.17765592522906742 | 19.823517953521943, 2.9819029876885947, 0.9906078125384032 | 本次运行 |
| 4 | 0.00808230034137024 | 0.00808230034137024 | 19.998189173103587, 1.0078766507842811, 1.000053452612871 | 本次运行 |
| 5 | 0.16620642784433526 | 0.16620642784433526 | 19.834383433009258, -0.9865137932305742, 0.9962782232808145 | 本次运行 |
| 6 | 0.007226461226634943 | 0.012603023069440406 | 20.011334486206714, -3.0034854693181536, 0.9957318485792005 | 本次运行 |
| 7 | 0.04295043979434982 | 0.04295043979434982 | 19.95729770980226, -4.995469377543474, 0.9991466826192161 | 本次运行 |
| 8 | 0.11946123797017386 | 0.11946123797017386 | 19.883505086943593, -6.974268111310299, 0.9938524383852155 | 本次运行 |
| 9 | 0.1209760421104842 | 0.1209760421104842 | 19.890697218784013, -8.948208825450543, 0.9975960403590589 | 本次运行 |

这 10 个最小距离都小于 0.2 m。0.2 m 是源码里 `Close to goal` 的阈值，不是论文的到达定义。日志里该字符串出现 0 次。6 号机结束时比采样中的最小值远。其余 9 架的结束距离与最小距离相同，或只差一个双精度末位（0 号机）。

离起点的最大距离（采样中，不是路径长度）：0 号 43.81323774693516 m，1 号 42.260775897173694 m，2 号 41.212629859222965 m，3 号 40.280130682811304 m，4 号 40.058667002157044 m，5 号 39.89395512094815 m，6 号 40.46913713797204 m，7 号 41.19834357233363 m，8 号 42.27023916952265 m，9 号 43.75170400446538 m。出处是本次运行。不把这些数当作论文的 dfly。

## 7. 再跑一次

命令在 [scripts/ext/ego_swarm_baseline.sh](../../scripts/ext/ego_swarm_baseline.sh)。它把仓库克隆到仓库外面，按上面的两步 `catkin_make -j1` 编译，再无头跑 `swarm.launch`。不把 ego-planner-swarm 加进本仓库。

```bash
scripts/ext/ego_swarm_baseline.sh --help
scripts/ext/ego_swarm_baseline.sh build
scripts/ext/ego_swarm_baseline.sh run --seconds 120
```

本仓库的规划器和已有场景卡没有改。
