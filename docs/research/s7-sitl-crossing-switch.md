# 票 S7：三机交叉与变阵 SITL 对照（M3 L1 抽检，只报数）

本票按 SPEC 做 M3 的 L1 抽检：同一套三机 PX4 SITL 入口跑交叉卡 `crossing-pillars` 和变阵卡 `formation-switch`，seeds 0–4，各自和 L0 同卡同 seed 对照；修 S6 发现的净空统计口径；最后给出方向 B（PX4 多实例锁步）要不要启动的建议。

只报数，判据未冻结。SITL 的 `result.json` 仍是 `report_only=true`、`criteria_frozen=false`。不改 `core/`、`scenes/suite.txt`、`criteria.frozen`、`README.md`、`scripts/premerge.sh`、`scripts/regress.sh`、`scripts/sitl/Dockerfile`、`docs/tickets.md`，也不写 PX4 的 `COM_` / `NAV_` / `BAT_` 参数。每趟的 `px4_parameters_written` 都是空列表。逐 seed 数字在 `s7-sitl-crossing-switch-runs.json`。

## 入口改动（最小必要）

S5a/S5b 的三机入口只接 `formation`。本票把交叉和变阵接进同一入口，默认行为不变：

| 文件 | 改动 |
|---|---|
| `yopo_l0/sitl_bridge.py` | `LIVE_POLICIES` 增加 `crossing`、`switch`；`FLEET_POLICIES` 三种编队类策略共用 `_fleet_for`，policy 必须和场景卡 `task.type` 同名。crossing 需要的深度帧由 `_fleet_ingest` 按场景卡相机频率、用 PX4 里程计位姿生成（`depth_source="scene-card"`）。新增 `ClockSkewMeter` 和净空新字段（见下）。 |
| `scripts/sitl/yopo_l0_live.py` | `--policy` 接受 `crossing` / `switch`；编队类策略的 PLAN 超时 60 s（其他仍 45 s）；输出 `clock_skew`。 |
| `scripts/sitl/run_yopo_l0_sitl.sh` | 策略检查改为 `upstream|formation|crossing|switch`，用法文本同步。 |
| `scenes/crossing-pillars-sitl80.json`、`scenes/formation-switch-sitl80.json` | 与基础卡只差名字、一句 note 和 `task.duration_s`（45 → 80、32 → 80，含起飞降落）。不进 `scenes/suite.txt`。 |
| `tests/test_sitl_s5a_formation.py` | S5a 断言 shell 入口拒绝 `--policy crossing`；本票起 crossing 合法，改为拒绝不存在的策略名，断言意图（未知策略退出 2）不变。 |
| `tests/test_sitl_s7_crossing_switch.py` | 12 个测试：卡只延长时长、policy 与卡匹配、两张卡的 dry-run、fake PX4 上变阵 3/3、crossing 深度来自世界系位姿、净空口径、时钟偏差、按需渲染。 |

```bash
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/formation-switch-sitl80.json --policy switch --seed 0
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/crossing-pillars-sitl80.json --policy crossing --seed 0
```

crossing 第一轮全部失败后，我测了 fake PX4 上的代理耗时：仿真 8.1 s 用墙钟 17.0 s，其中约一半是每机 30 Hz 的深度渲染（606 帧，约 18 ms/帧）。协议只在出提案时读 `frame.depth`。于是把深度帧改成 `_LazyDepthFrame`：按相机频率记下位姿和时刻，读 `depth` 时才渲染。同一 fake 冒烟降到墙钟 5.6 s，结果不变（3/3，到达 7.06 s）。`yopo_l0/net.py` 已经把 torch 设成单线程，`OMP_NUM_THREADS=1` 也测不出差别，所以没有加线程透传。下表里「逐帧深度」是改前，「按需深度」是改后。

## 环境与时段

镜像：`yopo-restart-sitl:s1b-23bc262f44d8`（PX4 v1.17.0 SITL + Gazebo Harmonic）和 `yopo-restart-sitl:yopo-81d084c04186`。定位 `default`。所有时间都是 2026-11-15 UTC-3。

| 臂 | 时段 | 主机负载（1 min loadavg，8 核） |
|---|---|---|
| L0 两张卡 | 10:03–10:05 | — |
| SITL 变阵 | 10:06–10:12 | 未记录 |
| SITL 交叉（逐帧深度） | 10:12–10:28 | 未记录 |
| SITL 交叉（按需深度） | 10:32–10:51 | 中位 11.07，最高 22.57 |
| SITL 变阵（同负载对照） | 10:51–10:57 | 中位 10.37，最高 22.21 |

box 是多 agent 共享机。按需深度臂和对照臂期间，另一个 agent 的 3 个 `ego-teacher` 容器和 merge-queue 的 premerge pytest 在同时跑，我没有动它们。对照臂就是为了把「主机负载」和「代理负载」分开：同样的主机负载下，再跑一遍变阵。

## 对照表

L0 到达从 PLAN 起算。SITL 用「任务钟到达 − `formation_t0_s`」和 L0 比（`formation_t0_s` 是三机全部进入 PLAN 的那一拍）。区间写「中位（最小–最大）」。L0 的「碰撞」写 0，表示结局是 SUCCESS，没有碰撞终止。

### 变阵 formation-switch

| 指标 | L0 | SITL | SITL 同负载对照 |
|---|---|---|---|
| 成功 / 到达 | 5/5，15/15 | 5/5，15/15 | 5/5，15/15 |
| 碰撞 | 0 | 0 | 0 |
| 交接后到达 / s | 27.0（全同） | 27.224（27.188–27.268） | 27.22（27.18–27.348） |
| 队形误差 p95 / m | 0.0356 | 0.3221（0.31–0.3466） | 0.3141（0.3065–0.3214） |
| 最小机间距 / m | 1.5364 | 1.372（1.2875–1.5135） | 1.4264（1.3235–1.5533） |
| RTF | —（点质量） | 0.9974（0.9891–0.9995） | 0.9988（0.9919–0.9995） |
| 机间时间戳极差 中位 / p95 / s | — | 0.0956 / 0.2216 | 0.0855 / 0.2339 |
| 机间时间戳极差 最大 / s | — | 1.124（0.594–1.932） | 1.126–1.129（2 个 seed 有效*） |
| PLAN 段最小净空 / m | 1.0427（全程） | 0.6613（0.6156–0.6866） | 0.6598（0.6302–0.6829） |
| PX4 `Accel #0 fail: TIMEOUT` 次数 | — | 合计 1 | 合计 2 |
| 提案 / nack | — | 每趟 7 / 0 | 每趟 7 / 0 |

\* 对照臂 seeds 0、2、4 的 `clock_skew` 把 PX4 退出时时间戳为 0 的里程计也算进去了，最大值落在 1.79e9 s，不可用；中位和 p95 不受影响。代码已修（`zero_stamps_skipped`，见测试），runs.json 里这几格写 `null` 并标 `skew_zero_stamp_contaminated=true`。

### 交叉 crossing-pillars

| 指标 | L0 | SITL 逐帧深度 | SITL 按需深度 |
|---|---|---|---|
| 成功 / 到达 | 5/5，15/15 | 0/5，1/15 | 0/5，0/15 |
| 终止原因 | 全部 GOAL_REACHED_AND_HELD | MAX_S×2，uav0:PLAN_TIMEOUT×2，uav1:TAKEOFF_TIMEOUT×1 | uav0:PLAN_TIMEOUT×2，MAX_S×2，PREFLIGHT px4_3×1 |
| 碰撞 | 0 | 0 | 0 |
| 交接后到达 / s | 7.06（7.05–7.76） | 无 | 无 |
| 队形误差 p95 / m | 4.3418（4.1387–4.6095） | 19.5089（4 趟） | 19.5292（4 趟） |
| 最小机间距 / m | 7.2811（3.4909–7.3073） | 7.9121（2.8563–7.9578） | 7.9328（7.8848–7.9534，4 趟） |
| RTF | — | 0.914（0.4382–0.9973） | 0.7259（0.4631–0.9896，4 趟） |
| 机间时间戳极差 中位 / p95 / s | — | 0.1555 / 0.2776 | 0.1348 / 0.2541 |
| 机间时间戳极差 最大 / s | — | 25.96（0.315–181.061） | 40.83（12.497–251.063） |
| 单机里程计时间戳落后仿真钟（极差最大）/ s | — | 0.34–240.94 | 30.242–243.451 |
| PX4 `Accel #0 fail: TIMEOUT` 次数 | — | 合计 15 | 合计 17 |
| 提案 / nack | — | 0–268 / 0 | 0–278 / 0 |

交叉卡的队形误差 p95 在 L0 本来就大（冻结门 6.0 m）；SITL 的 19.5 m 是没飞出去，等于离终点槽位的距离。最小机间距在 SITL 反而更大，原因也是没飞到交叉区。

按需深度臂 seed 0 的过程：三机都在约 10 s 进入 PLAN。uav0、uav2 的设定点一直停在悬停点；uav1 的设定点往前走，里程计却停在原地，约 31 s 后才跳到 (19.7, 10.6, 2.3)。px4_1 记录 `Accel #0 fail: TIMEOUT`，随后 `Landing at current position`。里程计时间戳比仿真钟最多落后 243 s。

## 净空统计口径

S6 指出 `min_surface_clearance_m` 把降落段也算进去：高度刚过机体半径时，地面距离把它压到接近 0。本票保留旧字段语义不变（所有离地样本，含降落），新增三个字段，并在 `result.json` 写 `clearance_note`：

| 字段 | 统计范围 |
|---|---|
| `min_surface_clearance_m`（旧） | 所有离地样本，含起飞离地和降落 |
| `min_surface_clearance_flight_m` | 只算 TAKEOFF / PLAN 段，不算降落 |
| `min_surface_clearance_plan_m` | 只算 PLAN 段 |
| `min_inter_drone_distance_plan_m` | 两机都在 PLAN 段时的机间距 |

测试 `test_flight_clearance_excludes_the_landing_segment` 构造了降落到 z=0.40 的场景：旧字段 0.05 m，飞行段字段不受影响。实测里飞行段字段多数仍接近 0，因为 TAKEOFF 从地面开始（变阵 0.0033–0.0219 m；对照 seed 0 旧字段 0.0017、飞行段 0.0248，差别来自降落段）。真正能和 L0 比的是 PLAN 段字段：变阵 0.6156–0.6866 m，L0 1.0427 m。

## 方向 B 建议

**推荐：启动方向 B（PX4 多实例锁步），但先用一张卡限定范围。** 第一张验收卡是 `crossing-pillars-sitl80`：在同样的共享机负载下跑到 L0 同级（5/5）。变阵、编队保持这类轻代理卡，继续用现有多机 SITL。

理由，全部来自本票数字：

1. **PX4 没有和仿真钟锁步。** 里程计时间戳的中位偏移是 1.7947e9 s，也就是墙钟 epoch，不是 gz 仿真钟。PX4 按墙钟要传感器数据；gz 或代理一慢，PX4 就判超时。
2. **轻代理时，现有 SITL 已经够用。** 变阵 10/10 成功；RTF ≥ 0.9891；机间时间戳极差中位 0.06–0.10 s、p95 约 0.22 s。和 L0 的差距可解释：交接后到达 27.22 s 对 27.0 s（+0.8%）；最小机间距 1.29–1.55 m 对 1.536 m；队形 p95 0.31 m 对 0.036 m（PX4 跟踪误差）。同负载对照（loadavg 中位 10.4）下结果几乎一样，所以主机负载本身没有把多机 SITL 压垮。
3. **重代理（网络 + 深度在环）时，现有 SITL 不可用，而且原因正是不锁步。** 交叉卡 0/10（两臂合计）；RTF 掉到 0.44–0.48；PX4 报加速度计超时合计 32 次，进入失效保护降落；里程计落后仿真钟最多 243 s；机间时间戳极差最大到 251 s。L0 同卡 5/5、7.06 s 到达。
4. **只靠代理侧提速补不上。** 按需深度把代理耗时砍掉约 2/3（fake 冒烟墙钟 17.0 → 5.6 s），交叉卡仍是 0/5。在同一台共享机上，代理的算力占用会随别人的负载起伏，墙钟 SITL 没有余量。锁步之后，代理和 PX4 都等仿真钟，算得慢只拉低 RTF，不会改变飞行结果。

不足：没有在空闲机上单独跑交叉卡，所以分不出「代理自身」和「共享机峰值负载」各占多少。锁步正好让这个区分不再重要。

## 回归

2026-11-15 11:13–11:37 UTC-3，提交 `578ec12`（已合并 origin/main 的 #133、#134）：

| 项 | 结果 |
|---|---|
| `scripts/regress.sh` | pytest=通过 批跑=通过；L0 pytest 870 passed / 27 skipped；core pytest 453 passed / 3 skipped / 4 deselected；总耗时 1406.3 s |
| 批跑冻结卡 | crossing-pillars 10/10（7.27 s）、formation-switch 10/10（27.00 s）、formation-hold 3/3、g2-pillar@upstream 10/10，碰撞 0 |
| `pytest tests/test_sitl*.py` | 111 passed |

第一次回归在修 S5a 断言之前跑，L0 pytest 1 failed / 869 passed（`test_dry_run_lists_formation_without_a_protocol_topic`）。修后整套重跑，上表就是重跑结果。
