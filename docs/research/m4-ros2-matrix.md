# 票 M4-ROS2：formation-6-open 六档 ROS 2 矩阵（只报数）

只报数。report-only。ROS 2 形态仍是 L0 世界和 L0 锁步时钟：每机规划在 ROS 2 节点里，`/clock` 发仿真时间。进程内六档已经冻结；这次不改 `criteria.frozen`、`criteria.gates`、`gate_forms`，也不进 `scenes/suite.txt`。

跑数时 HEAD 是 `8678b92986596c1e22081b2121f8ec52478bc500`（当前 main）。相对这个提交，本票只加了跑数脚本和这份记录，没有改 `yopo_l0/`、`core/`、场景卡。场景卡哈希 `a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d`，与 [SUMMARY](../results/SUMMARY.md) 里 formation-6-open 的 `a803d8021c77` 相同。影子盾开，策略 `formation`，层级 L0。ROS 2 发行版是 Jazzy。逐粒记录在 [m4-ros2-matrix-runs.json](m4-ros2-matrix-runs.json)。

## 结论

六档 × seeds 0–4，共 30 粒，全部 `SUCCESS`。和 [SUMMARY.md](../results/SUMMARY.md) 的进程内六档逐格对上：到达 30/30，碰撞 0，队形误差 p95 0.066 m，最小机间距离 2.980 m。到达时间 20.13 s。SUMMARY.md 的表上没有到达时间这一列；同一批 [SUMMARY.json](../results/SUMMARY.json) 里六档的 `arrival_time_s` 都是 min = max = mean = 20.13。

六档五行都对得上。时钟、QoS、投递时序下的轨迹与进程内相同，进程内已冻门保持原样。

原始值相同。30 粒的队形误差 p95 都是 `0.06595466175453953`，最小机间都是 `2.979793631499256`，到达时间都是 `20.13`。SUMMARY.json 里进程内六档的 p95 与到达时间同样是 min = max，最小机间是这一个数。显示口径沿用总表：p95 和机间三位小数，到达时间两位小数。

| 档位 | 形态 | 到达 | 碰撞 | 队形误差 p95 m | 最小机间 m | 到达时间 s | 与进程内 |
|---|---|---|---|---|---|---|---|
| none | ROS 2 | 30/30 | 0 | 0.066 | 2.980 | 20.13 | 一致 |
| drop-5 | ROS 2 | 30/30 | 0 | 0.066 | 2.980 | 20.13 | 一致 |
| drop-20 | ROS 2 | 30/30 | 0 | 0.066 | 2.980 | 20.13 | 一致 |
| duplicate-10 | ROS 2 | 30/30 | 0 | 0.066 | 2.980 | 20.13 | 一致 |
| reorder-delay | ROS 2 | 30/30 | 0 | 0.066 | 2.980 | 20.13 | 一致 |
| combined | ROS 2 | 30/30 | 0 | 0.066 | 2.980 | 20.13 | 一致 |

进程内对照行就是 SUMMARY.md 的六行（form = inprocess，seeds 0–4，影子盾开，commit `5524cab42584`）。六行的到达、碰撞、p95、最小机间与上表相同；到达时间取 SUMMARY.json 的 `arrival_time_s`。

30 粒的提交时刻也相同：`uav0`–`uav5` 的 `committed_t_switch_s` 依次是 0.5、1.0、1.5、2.0、2.5、3.0 s。每架 `local_commits = 1`，`local_aborts = 0`，`nacks = 0`，`catchup_time_s = 0`。已提交计划哈希在 30 粒里是同一组。仿真时长都是 21.13 s。碰撞按总表同一口径：`outcome == COLLISION` 记 1，否则 0。30 粒都是 `GOAL_REACHED_AND_HELD`。

| 飞机 | 已提交计划哈希 |
|---|---|
| uav0 | `bba6fe1f18820323e6ac54ce92fc7ce34265a71229738d7e468f24533ebf7496` |
| uav1 | `338b0858dd0b4528cc17ea9a177757b2c7972c0d78229996290ea6fe3233041b` |
| uav2 | `27d9be85b42e9c218c5c38dd99e17ba754975614eba688f228bb550c06de598c` |
| uav3 | `d7f7006e0c161a751f53979dd881ca25075b3989eb77ea3fe00c3309af7501cc` |
| uav4 | `b260bf4117d2c6687e20bd1f9b80e648ed6a7ab94c89493de92e205afc9b57b1` |
| uav5 | `9184228a4cb081d8d284153a8beacef64be144109f95cff1d5e5a4ca6fea4bbb` |

墙钟只报数，不进这张对账。单次墙钟 24.0–36.2 s，低于档位批跑的 120 s 上限。none 24.2–25.4 s，drop-5 24.0–25.3 s，drop-20 24.5–26.3 s，duplicate-10 25.2–27.4 s，reorder-delay 31.8–33.8 s，combined 34.0–36.2 s。影子拒绝率 30 粒都是 1.000，仍然只报数。

## 时钟、QoS、投递

30 粒 `clock_ok` 都是 true，`clock_error` 为空，`clock_samples` 都是 2113。2113 = 21.13 s × 100 Hz，和仿真时长同一拍。`/clock` 的 QoS 是 BEST_EFFORT、KEEP_LAST、depth 1、VOLATILE，与 rclpy 订阅 `/clock` 的档一致，`use_sim_time` 接得上。锁步话题（观测、计划、`bus_tx` / `bus_rx` / `bus_done`）是 RELIABLE、KEEP_LAST、depth 64、VOLATILE。

每一粒里，走完 `/{src}/bus_tx` → `/{dest}/bus_rx` → `/{dest}/bus_done` 的次数等于 `published - dropped + duplicated`。故障模型留下的每一份拷贝都走完这三跳。none、drop-5、drop-20、duplicate-10 上，这个次数等于 `delivered`。reorder-delay 和 combined 上，`delivered` 少一截：时延写在 `deliver_at` 上，relay 不睡墙钟，仿真在 21.13 s 停下时还有几条的投递时刻还在后面。这是档位自己的仿真时间，轨迹五格仍然与进程内相同。

总线计数随种子变化，五格指标不变。各档范围：

| 档位 | 丢包 | 复制 | 乱序 |
|---|---|---|---|
| none | 0 | 0 | 0 |
| drop-5 | 53–67 | 0 | 0 |
| drop-20 | 244–308 | 0 | 0 |
| duplicate-10 | 0 | 111–137 | 0 |
| reorder-delay | 0 | 0 | 941–1122 |
| combined | 68–81 | 131–156 | 975–1094 |

这五组范围，以及每一粒的 `published` / `dropped` / `duplicated` / `delivered` / `reordered`，与 [E-B7](e-b7-fault-and-swap8-sphere.md) 进程内关球臂在同一张场景卡上的记录逐粒相同。E-B7 的关球跑在 `b6fff8266dcc`，场景卡哈希与这次相同。总表那一批没有留下逐粒总线计数，所以总线这一列对的是 E-B7，五格指标对的是 SUMMARY。

[06c](../results/06c-drop20.md) 那一次 ROS 2 对账（drop 20%、seed 0）的轨迹就是现在这格：6/6，20.13 s，机间 2.980 m，p95 0.066 m，碰撞 0。它写下的总线 `1346 / 258 / 1088` 是当时那棵树的计数。当前 main 上同一粒是 `1380 / 268 / 1112`，与 E-B7 的进程内关球相同，轨迹五格仍是总表上的数。

分种子的总线计数：

| 档位 | seed | 发布 | 丢包 | 复制 | 送达 | 乱序 | 提案 | ACK |
|---|---|---|---|---|---|---|---|---|
| none | 0 | 1305 | 0 | 0 | 1305 | 0 | 6 | 30 |
| none | 1 | 1305 | 0 | 0 | 1305 | 0 | 6 | 30 |
| none | 2 | 1305 | 0 | 0 | 1305 | 0 | 6 | 30 |
| none | 3 | 1305 | 0 | 0 | 1305 | 0 | 6 | 30 |
| none | 4 | 1305 | 0 | 0 | 1305 | 0 | 6 | 30 |
| drop-5 | 0 | 1324 | 54 | 0 | 1270 | 0 | 6 | 39 |
| drop-5 | 1 | 1352 | 67 | 0 | 1285 | 0 | 6 | 52 |
| drop-5 | 2 | 1305 | 58 | 0 | 1247 | 0 | 6 | 30 |
| drop-5 | 3 | 1315 | 53 | 0 | 1262 | 0 | 6 | 35 |
| drop-5 | 4 | 1331 | 66 | 0 | 1265 | 0 | 6 | 41 |
| drop-20 | 0 | 1380 | 268 | 0 | 1112 | 0 | 6 | 60 |
| drop-20 | 1 | 1388 | 308 | 0 | 1080 | 0 | 6 | 63 |
| drop-20 | 2 | 1329 | 244 | 0 | 1085 | 0 | 6 | 39 |
| drop-20 | 3 | 1385 | 249 | 0 | 1136 | 0 | 6 | 65 |
| drop-20 | 4 | 1401 | 283 | 0 | 1118 | 0 | 6 | 71 |
| duplicate-10 | 0 | 1305 | 0 | 117 | 1422 | 0 | 6 | 30 |
| duplicate-10 | 1 | 1305 | 0 | 137 | 1442 | 0 | 6 | 30 |
| duplicate-10 | 2 | 1305 | 0 | 111 | 1416 | 0 | 6 | 30 |
| duplicate-10 | 3 | 1305 | 0 | 111 | 1416 | 0 | 6 | 30 |
| duplicate-10 | 4 | 1305 | 0 | 127 | 1432 | 0 | 6 | 30 |
| reorder-delay | 0 | 1525 | 0 | 0 | 1495 | 1122 | 6 | 130 |
| reorder-delay | 1 | 1523 | 0 | 0 | 1493 | 1065 | 6 | 128 |
| reorder-delay | 2 | 1520 | 0 | 0 | 1499 | 941 | 6 | 130 |
| reorder-delay | 3 | 1519 | 0 | 0 | 1489 | 1095 | 6 | 129 |
| reorder-delay | 4 | 1494 | 0 | 0 | 1480 | 966 | 6 | 119 |
| combined | 0 | 1521 | 76 | 145 | 1567 | 979 | 6 | 126 |
| combined | 1 | 1530 | 75 | 156 | 1591 | 1028 | 6 | 130 |
| combined | 2 | 1524 | 81 | 149 | 1578 | 975 | 6 | 134 |
| combined | 3 | 1519 | 70 | 142 | 1567 | 1044 | 6 | 129 |
| combined | 4 | 1511 | 68 | 131 | 1547 | 1094 | 6 | 121 |

提案在 30 粒里都是 6：每架一轮，第一圈就提交。ACK 比 30 多出来的，是窗口内重发同一份提案之后对端再投的票。

## 环境

Ubuntu 24.04.5。`scripts/setup_ros2.sh` 装的是 Jazzy：`ros-jazzy-ros-base`、`ros-jazzy-rclpy`，以及脚本里列出的 `std-msgs`、`rosgraph-msgs`、`python3-yaml`。装完之后 `.venv/bin/python` 可以 `import rclpy`（`/opt/ros/jazzy/lib/python3.12/site-packages/rclpy`）。动态库写在 `/etc/ld.so.conf.d/ros2-jazzy.conf`。

## 复现

```bash
scripts/setup_ros2.sh
make venv
PYTHONPATH=.:core/src .venv/bin/python scripts/m4-ros2-matrix.py
```

每粒单独起一个进程，写到 `out/m4-ros2-matrix/partial/<tier>/seed<N>.json`，写完才算这一粒完成。再跑同一条命令会跳过场景卡哈希和 `sim_rev` 都对得上的粒。`--no-resume` 忽略已有 partial。`--assemble` 只根据 partial 重写 `docs/research/m4-ros2-matrix-runs.json`。ROS 2 保持单进程，不开多份 rclpy。

`sim_rev` 当前是 `m4-ros2-matrix-1`。改了会影响轨迹或对账字段的代码时要一起改这个字符串，旧 partial 才不会被当成这次的结果。

本票没有 ROS 2 路径上的代码修复。30 粒在现有 `yopo_l0.ros2_run` 上跑完。

记录时间：2026-11-15（UTC）。
