# 票 D1：提交前的一次性几何回退

L0 闭环在提交新轨迹之前做一遍几何检查。默认关。关着的时候不采样、不往总线上加消息、不写结果字段，现有场景卡同 seed 的结果与改前按位一致（去掉会随 HEAD 变的 `commit` 和整回路墙钟 `wall_time_s`）。

这一遍只采样，不重规划，也不换下一条候选。失败就继续飞上一条已提交轨迹，条件是当前时刻还落在它的运动段里。运动段已经结束，或者还没有已提交轨迹，就按场景动力学 `a_max` 刹停悬停。原因码和回退次数记在结果字段 `geometric_fallback`。检查本身的墙钟是这个字段里的 `check_time_ms`，不进 `runner.WALL_FIELDS`，也不使用 E-C3 正在加的 `plan_wall_ms`。

影子盾仍只记录，不拦截。冻结门、`scenes/suite.txt`、已有场景卡的 `criteria` 都没有改。三机穿林的承诺视界仍是卡上的 1.0 s；六机构造时仍抬到 1.75 s，计划 `a_max` 仍是 6。这两条由 `tests/test_geom_fallback.py` 里的构造期断言守住，本票没有用六机把开关打开重跑。

## 检查做什么

采样步长 0.02 s，含轨迹两端，再顺着末端保持点多覆盖一个承诺视界，让邻机能看见已经停住的位置。

- 静态障碍：场景卡已有的几何（加载时内联的那一份），按机体半径膨胀。中心到表面的距离 ≤ `body_radius`（0.35 m）算冲突，和仿真真值碰撞同一口径。地面也算。
- 邻机：总线上邻机广播的已提交采样。中心距 < 阈值算冲突。阈值在场景卡 `geometric_fallback.inter_drone_distance_m`，缺省 2 × 机体半径 = 0.70 m。相等不算冲突。真值碰撞仍是中心距 ≤ 0.70 m。

原因码：`GEOM_OBSTACLE`、`GEOM_INTER_DRONE`。动作：`PREVIOUS`（留下旧轨迹）、`BRAKE`（刹停悬停）。一次检查可以同时记下两种原因，但不会再采样、再选一条候选。

「仍有效」指现在还早于这条已提交运动段的 `t_end`。段的长度在提交时已经按票 13a 盖住承诺视界。不在段中途按「提交时刻 + 视界」把曲线截掉，否则动力学还在飞这条已提交曲线，下一次提案又从它的末端接上，刹停和协议父计划会对不上。

## 场景

两张都只报数。都不进 `scenes/suite.txt`，都不设新的冻结门。

| 项 | 三机密林 | 三机对向变体 |
|---|---|---|
| 卡 | `scenes/crossing-forest.json` 用 `card_for_tier(..., "dense")` 拨密度。文件本身仍是 medium，`criteria` 未改 | `scenes/swap-3-geom-fallback.json`。几何与 E-C2 对向卡相同：半径 8.0 m、高度 1.5 m 的圆，飞向直径对侧。E-C2 原卡在 PR #40，尚未合入 main，所以本票单独放一张变体，不改那张未合入的文件 |
| 开关 | `RunConfig.geometric_fallback` 开 / 关。关时结果里没有 `geometric_fallback` 字段 | 卡上 `enabled: true`，阈值 0.70 m，`commit_horizon_s` 1.0 s |
| seed | 0–9 | 0 |
| 策略 | crossing（任务类型决定） | upstream，影子盾开 |
| 动力学 | `a_max=6`，`tau_s=0.1`，`kp=4`，`kd=3` | 同左。goto 没有 `speed_mps` |
| 承诺视界 | 三机 × 0.25 s + 0.25 s = 1.0 s，不抬高 | 卡上写 1.0 s。不是六机的 1.75 s |
| 本卡不复现 | — | EGO-Swarm 论文表 I 的 20.3 m / 14.3 s。论文圆半径和速度上限未知 |

对向变体不复现论文那两个数。8.0 m 是 E-C2 选定的半径，不是从 20.3 m / 14.3 s 反推的。

## 出处

| 项 | 值 |
|---|---|
| 跑数 commit | `0b1eff7dd2ce73811bd84ce8b8164845eaa80e23`（分支 `cursor/d1-geometric-fallback-1e34`，跑的时候工作区干净。记下结果的后一次提交 sha 不同，代码与这次跑数相同） |
| 层级 | L0 |
| 密林场景卡哈希 | `f1f6cd9a2c7938a5cedd9ecfcb270d7017145c7b820b9498c2d1608fc8e61e9d`（内联 dense 几何之后，与结果记录的 `scene_card_hash` 相同。开和关是同一份哈希） |
| 对向变体哈希 | `1421a056f804645a33639e2453a13121db0ea8fd4840069f407586835a0d9ebb` |
| 网络 | 上游一次前向，权重未改。本票不改选择规则，也不改 `safe_radius` 默认 0.05 m |

检查耗时是墙钟，不参与确定性比较。开着开关时，去掉 `check_time_ms` 之后，同 seed 重跑的轨迹字段应一致。关着开关时，连这个字段都不出现。

## 三机密林：开 vs 关

关：到达 3/30，撞树 9 次，都是 `uav0:TRUTH_POSITION`。没有机间碰撞。唯一全到达的是 seed 5，26.81 s，最小净空 0.287 m，最小机间 5.304 m。这和 [对照文档](ego-swarm-comparison.md) 里引用的 12e 密林列一致（到达 3/30、撞树 9、seed 5 为 26.81 s、净空 0.287 m、机间 4.688–5.510 m）。本票关着回退的这一列没有把那 9 次撞树改掉。

开：到达 18/30（seed 0、1、2、3、5、9 各 3/3）。撞树 1 次（seed 6，仍是 `uav0:TRUTH_POSITION`）。另外 3 个 seed（4、7、8）跑满 50 s，结果 `TIMEOUT` / `GOAL_NOT_REACHED`，碰撞 0。没有机间碰撞。没有三机挤在一起停住的死锁：这三档的最小机间距离仍是 5.365–5.510 m。uav0 的本地提交明显少（seed 4 是 15 次，seed 7 和 8 是 11 次），另外两架各有 66–67 次提交。终点误差分别是 21.32 m、25.84 m、25.83 m。如实记成超时未到达。

seed 5 打开之后回退次数是 0，到达时间、净空、机间距离和本地提交次数与关掉时相同。检查跑了 104 次，但没有拒绝任何一条。

| 项 | 关 | 开 |
|---|---|---|
| 到达 | 3/30 | 18/30 |
| 撞树的 run | 9 | 1 |
| 机间碰撞 | 0 | 0 |
| 超时未到达 | 0 | 3（seed 4、7、8） |
| 最小机间距离 m | 4.688–5.510 | 2.009–5.510 |
| 最小净空 m | −0.027 到 0.287（9 次为负） | −0.008 到 0.425（只有 seed 6 为负） |
| 回退次数 | 0 | 0–56，全部是 `GEOM_OBSTACLE`。密林没有记到 `GEOM_INTER_DRONE` |
| 检查 p50 / p95 | 没有这个字段 | 见下表，大约 15–21 ms / 22–25 ms |
| commit | `0b1eff7dd2ce73811bd84ce8b8164845eaa80e23` | 同左 |
| 场景卡哈希 | `f1f6cd9a2c7938a5cedd9ecfcb270d7017145c7b820b9498c2d1608fc8e61e9d` | 同左 |

净空 = 机体中心到障碍表面的距离 − 0.35 m。负值就是已经穿过表面。碰撞列按 run 计：仿真在第一次碰撞处停下，所以一档最多 1。

### 关掉

| seed | 结果 | 到达 | 原因 | 最小机间 m | 最小净空 m | 回退 | 仿真 s |
|---|---|---|---|---|---|---|---|
| 0 | COLLISION | 0/3 | `uav0:TRUTH_POSITION` | 4.688 | −0.015 | 0 | 6.35 |
| 1 | COLLISION | 0/3 | `uav0:TRUTH_POSITION` | 5.472 | −0.027 | 0 | 9.41 |
| 2 | COLLISION | 0/3 | `uav0:TRUTH_POSITION` | 5.358 | −0.015 | 0 | 9.41 |
| 3 | COLLISION | 0/3 | `uav0:TRUTH_POSITION` | 5.422 | −0.014 | 0 | 8.72 |
| 4 | COLLISION | 0/3 | `uav0:TRUTH_POSITION` | 5.510 | −0.026 | 0 | 9.42 |
| 5 | SUCCESS | 3/3 | — | 5.304 | 0.287 | 0 | 27.81 |
| 6 | COLLISION | 0/3 | `uav0:TRUTH_POSITION` | 4.721 | −0.009 | 0 | 6.33 |
| 7 | COLLISION | 0/3 | `uav0:TRUTH_POSITION` | 5.365 | −0.015 | 0 | 9.43 |
| 8 | COLLISION | 0/3 | `uav0:TRUTH_POSITION` | 5.431 | −0.006 | 0 | 9.42 |
| 9 | COLLISION | 0/3 | `uav0:TRUTH_POSITION` | 4.812 | −0.009 | 0 | 6.19 |

到达时间只在 seed 5 有数：26.81 s。

### 打开

| seed | 结果 | 到达 | 原因 | 最小机间 m | 最小净空 m | 回退 | 动作 | 检查 p50 ms | 检查 p95 ms | 仿真 s |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | SUCCESS | 3/3 | — | 4.546 | 0.064 | 1 | PREVIOUS 1 | 21.19 | 24.88 | 27.04 |
| 1 | SUCCESS | 3/3 | — | 5.472 | 0.092 | 3 | PREVIOUS 3 | 20.67 | 24.77 | 27.13 |
| 2 | SUCCESS | 3/3 | — | 2.009 | 0.259 | 4 | PREVIOUS 1，BRAKE 3 | 20.53 | 24.62 | 29.66 |
| 3 | SUCCESS | 3/3 | — | 3.871 | 0.052 | 24 | BRAKE 24 | 15.67 | 25.02 | 42.27 |
| 4 | TIMEOUT | 0/3 | `GOAL_NOT_REACHED` | 5.510 | 0.002 | 52 | PREVIOUS 3，BRAKE 49 | 15.40 | 23.38 | 50.00 |
| 5 | SUCCESS | 3/3 | — | 5.304 | 0.287 | 0 | — | 20.50 | 24.47 | 27.81 |
| 6 | COLLISION | 0/3 | `uav0:TRUTH_POSITION` | 4.460 | −0.008 | 1 | PREVIOUS 1 | 21.11 | 24.24 | 10.12 |
| 7 | TIMEOUT | 0/3 | `GOAL_NOT_REACHED` | 5.365 | 0.425 | 56 | PREVIOUS 1，BRAKE 55 | 15.39 | 22.49 | 50.00 |
| 8 | TIMEOUT | 0/3 | `GOAL_NOT_REACHED` | 5.431 | 0.307 | 56 | PREVIOUS 1，BRAKE 55 | 15.41 | 23.00 | 50.00 |
| 9 | SUCCESS | 3/3 | — | 4.754 | 0.034 | 4 | PREVIOUS 4 | 20.34 | 24.82 | 28.31 |

打开之后的到达时间：seed 0 为 26.04 s，1 为 26.13 s，2 为 28.66 s，3 为 41.27 s，5 为 26.81 s，9 为 27.31 s。

seed 6 仍撞树。净空 −0.008 m，比关掉时的 −0.009 m 略晚（仿真 10.12 s，关掉是 6.33 s），但真值已经穿过表面。回退只有 1 次，动作是留下旧轨迹。0.02 s 采样可以漏掉采样点之间的擦碰，已提交曲线在采样上过了门之后，跟踪误差也可以把机体再推进表面零点几厘米。本票没有把这两种来源拆开。这一次碰撞照实留下。

seed 4 的未碰撞净空只有 0.002 m。门是按采样点判的，飞出来的净空可以比 0 紧得多。

## 三机对向变体（seed 0，回退开）

到达 0/3。仿真 2.66 s 时机间碰撞，`uav0-uav2:INTER_DRONE`。最小机间距离 0.697 m。最小障碍距离 1.500 m，净空 1.150 m（开阔区只有地面，高度一直在 1.5 m 附近）。不是死锁：碰撞时飞机还在飞，终点误差 9.668 m。

回退 30 次，全部是 `GEOM_INTER_DRONE`，动作全部是 `PREVIOUS`，刹停 0 次。检查 81 次，p50 1.10 ms，p95 1.41 ms。承诺视界 1.0 s，阈值 0.70 m，`a_max` 6。commit `0b1eff7dd2ce73811bd84ce8b8164845eaa80e23`。场景卡哈希 `1421a056f804645a33639e2453a13121db0ea8fd4840069f407586835a0d9ebb`。

E-C2 在没有这道回退时，upstream 也是机间碰撞（2.16 s，`uav1-uav2`，最小机间 0.631 m，那次跑在 PR #40 的分支上）。本票打开之后碰撞晚了约 0.5 s，最近距离从 0.631 m 放到 0.697 m，相撞的一对也不一样，但真值碰撞还在。30 次检查都拒绝了新轨迹，又都留下了已经提交的旧曲线。旧曲线是在邻机采样还没广播出来时提交的，之后的检查不再改它。单遍、不迭代，就得到这个结果。

## 测试

`tests/test_geom_fallback.py`：

- 关掉时不采样、不入队。
- 障碍按机体半径膨胀，距离等于半径算冲突；机间距离等于阈值不算冲突，小于阈值才算。
- 失败只查一遍。运动段还没结束就留下旧轨迹；段飞完了才刹停。
- 六机卡即使写上 `geometric_fallback.enabled=true`，构造出的承诺视界仍是 1.75 s，计划 `a_max` 仍是 6，节点才换成带检查的那一个。三机卡不抬高。
- 变体卡 `frozen=false`、没有 gates、不在 `suite.txt`。`crossing-forest.json` 没有 `geometric_fallback` 字段，`criteria.frozen` 仍是 true。
- 现有 17 张能加载的场景卡，seed 0、仿真 1.0 s、回退关，结果记录和 tick 日志的 sha256 与改前指纹 `tests/fixtures/geom_fallback_off_1s.json` 一致。指纹去掉了 `commit` 和 `wall_time_s`。

同一工作区还跑过：`tests/test_l0_freeze.py`、`tests/test_l0_six_batch.py`、`tests/test_l0_horizon_invariant.py`、`tests/test_l0_hover.py`、`tests/test_l0_forest.py`、`tests/test_shadow.py`、`tests/test_l0_crossing.py`、`tests/test_l0_formation.py`、`tests/test_l0_multidrone.py`、`tests/test_policies.py`、`tests/test_bus.py`、`tests/test_l0_safe_radius.py`、`tests/test_premerge.py`、`tests/test_l0_progress_gate.py`、`tests/test_l0_elastic_slot.py`、`tests/test_l0_ghost_stops.py`、`tests/test_l0_switch.py`、`tests/test_l0_six_forest.py`、`tests/test_l0_six_open.py`。这些都通过。

## 还没做的

- 对向碰撞还在。回退不会把已经提交的曲线从邻机上推开。
- 密林 seed 6 的撞树还在。采样间隙和跟踪误差没有分开记账。
- seed 4、7、8 是超时，不是撞树，也不是机间距离掉到 0.70 m 的死锁。50 s 不够它们到达。
- 密林上的检查墙钟是十几到二十几毫秒，对向开阔区大约 1 ms。这是 Python 里对场景几何做点采样的耗时，不是网络前向，也不是论文表 II 的优化器毫秒。不进门。
- 六机林带没有打开这个开关。13a 的 1.75 s 只在构造期核对过。
- 没有改 README，也没有改 `docs/tickets.md`。

## 怎么重跑

```bash
make venv
git rev-parse HEAD
.venv/bin/python scripts/d1-geometric-fallback.py --jobs 4 --out out/d1-geometric-fallback/runs.json
```

脚本先把密林拨档写到 `out/`（不进 git），再对 seed 0–9 各跑关和开，最后跑对向变体 seed 0。墙钟会变。关着回退时，密林的到达、碰撞、机间距离和净空不应随墙钟变。打开之后，`check_time_ms` 会变，回退次数和轨迹结果在同 seed 下不应变。
