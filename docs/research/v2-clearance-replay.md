# 票 V2：穿林跟踪净空门前后对比回放，只出图

只出图。没有改 `yopo_l0/` 的默认、`core/`、已冻门、`scenes/`、`docs/tickets.md` 和 README。`track_clearance` 仍是 false，`empty_brake` 仍是 false。打开其中任何一个都标偏离上游。未冻结，不进 `scenes/suite.txt`，不设 gates。

脚本是 [v2-clearance-replay.py](../../scripts/v2-clearance-replay.py)。它不改选择逻辑，只在进程里盖开关，把三档的轨迹和净空画在一起。全程十粒的表仍以 [E-C5b](e-c5b-track-clearance.md) 为准。下面的数字是 2026-11-13 的一次 `--partial` 冒烟：seed 0，仿真前缀 2.0 s，不是飞完场景卡。

```bash
.venv/bin/python scripts/v2-clearance-replay.py --partial --out /tmp/v2-smoke
```

这次实跑大约 59 s（含出图）。`summary.json`、`compare/*.gif`、`compare/*.png`、`compare/*.mp4` 和每档 `logs/*/frames.jsonl` 都在这个目录里，不进仓库。

## 三档

参数不写进场景卡。`safe_radius` 不覆盖，运行器缺省 0.05 m。几何门的余量写成缺省的 0.08 m，机体半径仍是 0.35 m。

| 臂 | 进程里多出来的 | 偏离上游 |
|---|---|---|
| base（全关） | 没有。不写 `policy_params` | 否 |
| geom | `track_clearance=true`，`track_clearance_mode="geom"`，`track_margin_m=0.08` | 偏离上游 |
| geom + empty_brake | geom 再加 `empty_brake=true` | 偏离上游 |

[E-C5b](e-c5b-track-clearance.md) 的 geom 列同时还开着 E-A1（阈值 1.0 m）和 `empty_brake`。本脚本的 geom 只开几何门。单机没有邻机，E-A1 本来也不改轨迹。`empty_brake` 只在规划码变成 `CORRIDOR_EMPTY_HOVER` 时，把原地悬停换成从参考头刹停。

默认两张卡，可用 `--scenes` / `--seed` 换：

| 卡 | 文件 | 机数 |
|---|---|---|
| solo | `scenes/ego-density-014-solo.json` | 1 |
| swap3 | `scenes/ego-density-014-swap3.json` | 3 |

## 冒烟读数

`--partial` 把仿真收到 2.0 s。两张卡的任务时长是 50 s 和 40 s，所以六档的 `outcome` 都是 `TIMEOUT`，`reason` 是 `GOAL_NOT_REACHED`，`arrived` 是 0。这是前缀，不是到达率。

`min_obstacle_clearance_m` 是机体中心到最近表面的距离，含地面，和 `ticks.jsonl` 的 `min_obstacle_m` 同一口径。碰撞线是机体半径 0.35 m。`min_ground_z_m` 是全程最低的机体中心高度。地面在 z = 0，所以地面净空等于高度。这 2 s 里最低高度大约 1.0 m，比树的表面净空大，最小表面净空来自树，不来自地面。单机没有机间距离。

| 卡 | 臂 | 结果 | 到达 | 最小表面净空 m | 最低高度 m | 最小机间 m |
|---|---|---|---|---|---|---|
| solo | base | `TIMEOUT` | 0/1 | 0.491 | 1.030 | — |
| solo | geom | `TIMEOUT` | 0/1 | 0.502 | 1.060 | — |
| solo | geom + empty_brake | `TIMEOUT` | 0/1 | 0.502 | 1.060 | — |
| swap3 | base | `TIMEOUT` | 0/3 | 0.849 | 1.002 | 6.978 |
| swap3 | geom | `TIMEOUT` | 0/3 | 0.849 | 1.002 | 6.978 |
| swap3 | geom + empty_brake | `TIMEOUT` | 0/3 | 0.849 | 1.002 | 6.978 |

单机 base 的 0.491 m、geom 的 0.502 m，和 E-C5b 里 seed 0 全程最小树净空对得上（全关 0.491 m，geom 0.502 m）。最紧的那一下在前 2 s 里已经出现。单机两档 geom 的终点也相同，2 s 内规划码都是 `OK`，刹停没有分叉。E-C5b 单机 seed 0 的刹停拍也是 0。

三机的三个最小读数相同，是因为这 2 s 里最紧的表面、最低的高度和最近的机间都不在被改道的那一架上。终点已经分开。seed 0、t = 2.0 s 时 uav0 的位置和规划码：

| 臂 | uav0 终点 (x, y, z) m | 末拍 `plan_code` | `CORRIDOR_EMPTY_HOVER` |
|---|---|---|---|
| base | (6.377, 1.623, 1.192) | `OK` | 0 拍 |
| geom | (7.532, 2.069, 1.182) | `CORRIDOR_EMPTY_HOVER` | 20 拍（0.20 s） |
| geom + empty_brake | (7.491, 2.058, 1.172) | `OK` | 10 拍（0.10 s） |

uav1、uav2 在三档里的终点相同。几何门已经让 uav0 换了走廊；无解的那几拍上，刹停臂和悬停臂的终点差大约 4 cm。E-C5b 全关对向 seed 0 的机间碰撞在 2.60 s，这次前缀停在 2.00 s，机间还是 6.978 m，碰撞还没发生。

每帧 `frames.jsonl` 记着该拍每架的 `p`、`v`、`plan_code`、`min_surface_clearance_m`，以及这一拍的 `min_inter_drone_distance_m`（单机是 null）。

## 图

每个场景一张并排图，三列是 base、geom、geom + empty_brake。

- 上排是俯视。绿圆是飞行高度上的树干截面，折线是轨迹，叉是终点，星标是该档表面净空最小的那一拍。
- 下排是侧视。实线是机体中心高度 z(t)，虚线是地面净空 z − z_ground，填充是两者之间。这张卡地面 z = 0，实线和虚线重合，填充就是离地高度。读数写在图顶。

ffmpeg 在 PATH 里时写 `.mp4`，再用它转 `.gif`。没有 ffmpeg 时改用 pillow 写 `.gif`。总览 `.png` 两种环境都写。成片播放不超过 15 s。这次前缀是 2.0 s，10 fps，按实时播放。

| 文件 | 说明 |
|---|---|
| `compare/solo-seed0.png` | 单机关键帧总览 |
| `compare/solo-seed0.gif` | 单机并排动图 |
| `compare/solo-seed0.mp4` | 有 ffmpeg 时的同一条动图 |
| `compare/swap3-seed0.png` | 三机关键帧总览 |
| `compare/swap3-seed0.gif` | 三机并排动图 |
| `compare/swap3-seed0.mp4` | 有 ffmpeg 时的同一条动图 |
| `summary.json` | 每档 `outcome`、`arrived`、`min_obstacle_clearance_m`、`min_ground_z_m`、`min_inter_drone_distance_m` 和上面这些路径 |

`--reuse-logs` 只重渲，不重飞。去掉 `--partial` 才按场景卡飞完（单机 50 s，对向 40 s）。全程不在这次冒烟里。

## 怎么重跑

```bash
.venv/bin/python -m pytest -q tests/test_v2_clearance_replay.py
.venv/bin/python scripts/v2-clearance-replay.py --partial --out /tmp/v2-smoke
```

测试用 `tmp_path` 跑同一条 `--partial`（solo 和 swap3，seed 0），检查 gif、png 非空，以及 `summary.json` 里六档字段齐全。场景卡的字节不被改写。
