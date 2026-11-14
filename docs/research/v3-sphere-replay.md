# 票 V3：E-B4c 欧氏碰撞球开关对比回放，只出图

只出图。没有改 `yopo_l0/`、`core/`、已有脚本、`scenes/`、`docs/tickets.md` 和 README。后段和欧氏球都不写进场景卡。未冻结，不进 `scenes/suite.txt`，不设 gates。

脚本是 [v3-sphere-replay.py](../../scripts/v3-sphere-replay.py)。它在进程里盖两档开关，把俯视轨迹画在一起。开关和全程数字在 [E-B4c](e-b4c-sphere-avoid.md)（[PR #94](https://github.com/unununnnn/yopo-restart/pull/94)，已在 main 的 `f15354a`）。下面的表是 2026-11-13 的一次 `--partial` 冒烟：仿真前缀 2.0 s，跑在已经合入 main 的检出 `097bb79` 上。`peer_sphere_available` 是 true，base 和 sphere 都飞了。

```bash
.venv/bin/python scripts/v3-sphere-replay.py --partial --out /tmp/v3-smoke-both
```

这次实跑大约 93 s。六档墙钟是 11.7 s、13.5 s、13.5 s、16.5 s、13.6 s、16.5 s，其余是出图。`summary.json`、`compare/*.gif`、`compare/*.png`、`compare/*.mp4` 和 `logs/*/ticks.jsonl` 都在这个目录里，不进仓库。

## 两档

参数不写进场景卡。两档都不打开 `swarm_penalty` / `swarm_segment_penalty`。半径 0.85 m 是 0.70 m 碰撞球加 0.15 m 裕度，权重 8。碰撞线按机体半径 0.35 m 的两倍，0.70 m。

| 臂 | 进程里多出来的 |
|---|---|
| base（后段开，球关） | `backend.enabled=true`，`peer_sphere_enabled=false` |
| sphere（半径 0.85 m） | 后段开，并且 `peer_sphere_enabled=true`，半径 0.85 m，权重 8 |

打开后段就是在用 E-B2 的平滑。运行器结果里的 `upstream_deviation` 仍是 false：那个字段只看 `safe_radius`、`empty_brake` 和 `track_clearance`。这三粒的 `safe_radius_m` 是 0.05。

`policy.py` 和 `crossing.py` 仍只在椭球项打开时把邻机采样送进 `refine_yopo`。脚本在检出里已经有 `peer_sphere_enabled` 时，会在进程里包一层，让欧氏球拿到采样，同时不打开椭球项。这次包装已经装上。

检出里没有这个字段时，不带 `--partial` 会直接退出，码是 2。`--partial` 仍用现有后段飞 base，sphere 档在 `summary.json` 里标 `missing`。本分支的 main 已经带上 E-B4c，所以这次冒烟没有走那条路径。

环境变量 `YOPO_PEER_SPHERE`、`YOPO_PEER_SPHERE_RADIUS_M`、`YOPO_PEER_SPHERE_WEIGHT` 在脚本里会被清掉，避免两档被一起打开。半径和权重以本脚本写进内存场景卡的 0.85 m 和 8 为准。

## 三粒

| 卡 | 文件 | seed | 说明 |
|---|---|---|---|
| 开阔三机对向 | `scenes/swap-3-open.json` | 0 | 票面默认 |
| 密度 0.14 三机对向 | `scenes/ego-density-014-swap3.json` | 0 | 票面默认 |
| 密度 0.14 三机对向 | `scenes/ego-density-014-swap3.json` | 1 | E-B4c 全程里剩下的 `uav1-uav2` 碰撞粒 |

可用 `--cases swap-3-open:0,ego-density-014-swap3:1` 换粒。

## 冒烟读数

`--partial` 把仿真收到 2.0 s。两张卡的任务时长都是 40 s，所以六档的 `outcome` 都是 `TIMEOUT`，`reason` 是 `GOAL_NOT_REACHED`，`arrived` 是 0/3。这是前缀，不是到达率。

`min_inter_drone_distance_m` 从 `ticks.jsonl` 的机体中心距重算。碰撞对是中心距进入 0.70 m 的机对；`reason` 里的 `INTER_DRONE` 也会留下。这 2 s 里六档都还没有碰撞对。最近一对都出现在前缀的最后一拍 t = 2.00 s，机对是 `uav1-uav2`。

| 卡 | seed | 臂 | 状态 | 结果 | 到达 | 最小机间 m | 碰撞对 | 最近一对 |
|---|---|---|---|---|---|---|---|---|
| swap-3-open | 0 | base | ok | `TIMEOUT` | 0/3 | 2.553 | 无 | `uav1-uav2` 2.553 m，t = 2.00 s |
| swap-3-open | 0 | sphere | ok | `TIMEOUT` | 0/3 | 2.420 | 无 | `uav1-uav2` 2.420 m，t = 2.00 s |
| ego-density-014-swap3 | 0 | base | ok | `TIMEOUT` | 0/3 | 7.413 | 无 | `uav1-uav2` 7.413 m，t = 2.00 s |
| ego-density-014-swap3 | 0 | sphere | ok | `TIMEOUT` | 0/3 | 7.297 | 无 | `uav1-uav2` 7.297 m，t = 2.00 s |
| ego-density-014-swap3 | 1 | base | ok | `TIMEOUT` | 0/3 | 7.462 | 无 | `uav1-uav2` 7.462 m，t = 2.00 s |
| ego-density-014-swap3 | 1 | sphere | ok | `TIMEOUT` | 0/3 | 7.317 | 无 | `uav1-uav2` 7.317 m，t = 2.00 s |

后段耗时（墙钟预算仍是 0.02 s，最多 8 次迭代）。sphere 的 p50 比 base 高，邻机采样已经进了后段。

| 卡 | seed | 臂 | 调用 | 采纳 / 运动学回退 | p50 / p95 ms |
|---|---|---|---|---|---|
| swap-3-open | 0 | base | 60 | 58 / 2 | 3.4 / 4.3 |
| swap-3-open | 0 | sphere | 60 | 58 / 2 | 4.8 / 6.8 |
| ego-density-014-swap3 | 0 | base | 60 | 58 / 2 | 4.7 / 7.0 |
| ego-density-014-swap3 | 0 | sphere | 60 | 58 / 2 | 6.1 / 8.9 |
| ego-density-014-swap3 | 1 | base | 60 | 59 / 1 | 4.7 / 7.4 |
| ego-density-014-swap3 | 1 | sphere | 60 | 59 / 1 | 6.1 / 8.5 |

这 2 s 停在对穿之前。机体中心距还在 2.4 m 以上，0.85 m 的铰链还没到执行轨迹的最近点上。sphere 的前缀最小机间比 base 略小（开阔 2.420 m 对 2.553 m，密度场 7.297 m / 7.317 m 对 7.413 m / 7.462 m），是规划时域里的邻机项已经在改曲线，不是全程避碰的结果。

[E-B4c](e-b4c-sphere-avoid.md) 的全程是另一张表：`swap-3-open` seeds 0–4 上 sphere 把到达从 0/15 拉到 15/15、机间碰撞 0；`ego-density-014-swap3` 是 12/15，seed 1 仍是 `uav1-uav2` 0.694 m。去掉 `--partial` 才按场景卡的 40 s 飞完，才能在同一套俯视图里对着看那张表。

## 图

每个用例一张并排俯视图。左列是 base，右列是 sphere。绿圆是飞行高度上的树干，折线是轨迹，叉是终点，星标是该档全程最近的那一拍。sphere 列的机体周围是半径 0.85 m 的虚线圆。

读数写在图顶：当前机间、全程最小、到达、结果、碰撞对。成片播放不超过 15 s。这次前缀是 2.0 s，10 fps，按实时播放。ffmpeg 在 PATH 里，所以同一条动图有 `.mp4` 和 `.gif`。没有 ffmpeg 时改用 pillow 写 `.gif`。总览 `.png` 两种环境都写。

| 文件 | 说明 |
|---|---|
| `compare/swap-3-open-seed0.png` | 开阔三机 seed 0 俯视静图 |
| `compare/swap-3-open-seed0.gif` | 同一粒的俯视动图 |
| `compare/swap-3-open-seed0.mp4` | 有 ffmpeg 时的同一条动图 |
| `compare/ego-density-014-swap3-seed0.png` | 密度 0.14 三机 seed 0 |
| `compare/ego-density-014-swap3-seed0.gif` | 同一粒的俯视动图 |
| `compare/ego-density-014-swap3-seed0.mp4` | 有 ffmpeg 时的同一条动图 |
| `compare/ego-density-014-swap3-seed1.png` | 密度 0.14 三机 seed 1 |
| `compare/ego-density-014-swap3-seed1.gif` | 同一粒的俯视动图 |
| `compare/ego-density-014-swap3-seed1.mp4` | 有 ffmpeg 时的同一条动图 |
| `compare/overview.png` | 三粒两档的俯视总览 |
| `summary.json` | 每档 `outcome`、`arrived`、`min_inter_drone_distance_m`、`collision_pairs` |

`--reuse-logs` 只重渲已经飞过的档，不重飞。

## 怎么重跑

```bash
.venv/bin/python -m pytest -q tests/test_v3_sphere_replay.py
.venv/bin/python scripts/v3-sphere-replay.py --partial --out /tmp/v3-smoke-both
```

测试不跑完整闭环。它用假轨迹走 `--partial`，检查 `summary.json` 的字段、gif / png 的文件头，以及绘图函数能在 `tmp_path` 里写出图。场景卡的字节不被改写。缺开关时，不带 `--partial` 的调用返回 2，并且一次仿真都不启动。
