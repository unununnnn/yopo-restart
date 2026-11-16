# 票 SH4：TRACKING_BOUND 误报判定（只报数）

只报数。report-only。判据未冻结。不改 `core/`、`scenes/suite.txt`、已冻门、安全清单和 PX4 参数。影子盾仍然只记录，不改批准动作。研究开关 `shadow_tracking_align` 默认关；打开的结果标「偏离上游」。不放宽 `FlightOracle.tracking_bound`（0.15 m）去盖掉真问题。墙钟不进结论。本票没有新跑 SITL：#151 premerge 结束前只用已有 SH2 / SH2b 日志和 L0。

数字在 [sh4-tracking-bound-runs.json](sh4-tracking-bound-runs.json)。

## 结论

SITL 上打开环境证据之后，`would_reject` 仍有 0.997（SH2b 开臂 5 趟合计），直方图里最多的是 `TRACKING_BOUND`。SH3 已经判定：把这条码当成「计划危险」是误报，因为它把认证闭环界套到未承诺该界的上游 / PX4 栈上；0.15 m 本身不改。

触发条件是机体位置与批准位置的欧氏距离大于 0.15 m（`yopo_l0/shadow.py` `record_oracle_track_lag`，界取 `FlightOracle.tracking_bound`）。L0 上点质量几乎贴着计划飞：本票在 ticks 上量到的横向 p95，g2-pillar 中位数 0.074 m，crossing-pillars 中位数 0.059 m，和票 B2 规划时刻的 0.06 m 同量级，很少越过 0.15 m。锁步 SITL 不是同一套跟踪：B2 规划时刻横向 p95 是 0.40–0.66 m，B3 / B4 后续臂到过 0.88 m，所以票面对照写成 0.40–0.88 m 对 0.06 m。SH2 / SH2b 的 `result.json` 里降采样 trace 的欧氏误差 p95 仍是 1.49–1.77 m，和 B2 的全程跟踪误差（0.6–2.25 m）同量级，远大于 0.15 m。成功到达的趟同样会被这条码打满。

打开 `shadow_tracking_align` 后，0.15 m 超界仍写入码表和直方图，但不单独把 `would_reject` 打满。不改阈值。真正的贴柱仍走 `ENV:OCCUPIED`；沿路径落后超过 1.5 m 仍走 `LAG_VIOLATION`。这两条都不是把 0.15 m 放宽。

L0 上 g2-pillar、crossing-pillars 在已经打开环境证据和 SH3 契约修正之后，`TRACKING_BOUND` 本就是 0，`would_reject` 平均仍是 0.0114 和 0.0176，开本开关数字不变。会记下跟踪码的是 `g2-hover-lift`：10 个种子共 730 次 `TRACKING_BOUND`，但每次都和 `TIME:SWITCH_NOT_IN_FUTURE`、`TRANSITION:HEAD_DISCONTINUITY` 叠在一起，只开跟踪开关 `would_reject` 仍是 1.000。契约修正单独打开会把跟踪测量从码表拿掉，`would_reject` 变成 0。两个开关一起开时，730 次 0.15 m 超界还在直方图里，`would_reject` 仍是 0。轨迹两臂相同，10/10 成功。

## 判定

| 项 | 说明 |
|---|---|
| 触发 | `\|\|actual − desired\|\| > FlightOracle.tracking_bound`，默认 0.15 m，严格大于。`desired` 是本拍批准位置（`approved_pva`），不是规划净空。 |
| 代码位置 | `yopo_l0/shadow.py` `record_oracle_track_lag`；界来自 `core/src/yopo_swarm_core/oracle.py` `FlightOracle.tracking_bound`。影子只重算，不调用 `FlightOracle.update`（那会锁存并把碰撞写进 oracle）。 |
| 最小复现 | `actual` 与 `desired` 相距 0.15 m 不记；相距 0.16 m 记下 `TRACKING_BOUND`。`tests/test_shadow_tracking_align.py` 和 `tests/test_shadow.py` `test_oracle_lag_and_tracking_codes_match_flight_oracle_bounds`。 |
| 判定 | 当作「计划危险」、打进 `would_reject` 是误报。0.15 m 是认证栈给自己跟飞的界，上游点质量和 PX4 位置环都没有承诺这个数。超界本身仍是真实测量，要记下来。 |
| 不是误报的部分 | 轨迹穿过柱子（`ENV:OCCUPIED`）；沿起点到终点的进度落后超过 `lag_limit` 1.5 m（`LAG_VIOLATION`，参考速度 0.15 m/s）。开关不改这两条。 |

## 开关

`RunConfig.shadow_tracking_align` 默认 `False`。命令行 `--shadow-tracking-align`，SITL 还可设 `YOPO_SHADOW_TRACKING_ALIGN=1`。关时不加字段，`would_reject` 仍是「有拒绝码就为真」，跟踪界仍是 0.15 m。

打开时只改影子记录，不改设定点：

- `TRACKING_BOUND` 仍按 0.15 m 计算，并留在 `reject_codes` 和直方图里。
- `would_reject` 忽略单独的 `TRACKING_BOUND`。同一 tick 上还有环境码、契约码或 `LAG_VIOLATION`，该 tick 仍会拒。
- 不把 0.15 m 改成 0.40 m 或 0.88 m。SITL 横向 p95 已经经常超过 0.15 m，放宽阈值只会把测量和判定混在一起。

SH3 的 `shadow_contract_align` 单独打开时，仍会从返回码里拿掉 `TRACKING_BOUND`（保持当时的行为）。本开关打开后，测量留在码表里，由 `codes_would_reject` 决定会不会拒。两个开关可以同时开。

## L0 对照（seeds 0–9）

### 已开环境证据和契约修正的 g2-pillar / crossing-pillars

两臂都开 `shadow_env_evidence` 和 `shadow_contract_align`。打开跟踪对齐的一臂再标一次偏离上游。轨迹摘要两臂相同。碰撞都是 0。

| 场景 | 到达 | 碰撞 | would_reject 关 | would_reject 开 | `TRACKING_BOUND` 关 / 开 | 横向 p95 中位数 m | 留下的码（两臂相同） |
|---|---|---|---:|---:|---|---:|---|
| g2-pillar | 10/10 | 0 | 0.0114 | 0.0114 | 0 / 0 | 0.074 | `TRANSITION:MOTION_KINEMATICS` 60 |
| crossing-pillars | 10/10（每趟 3/3） | 0 | 0.0176 | 0.0176 | 0 / 0 | 0.059 | `ENV:OCCUPIED` 225，`TRANSITION:MOTION_KINEMATICS` 225 |

这两张卡在 L0 上本来就几乎不越过 0.15 m，所以本开关没有可改的 `would_reject`。留下的运动学码和 crossing 上的占用是 SH3 已经留下的真问题，不是跟踪界。

### g2-hover-lift

这张卡会抬升，L0 欧氏跟踪 p95 0.217 m，10 个种子各记下 73 次 `TRACKING_BOUND`（合计 730）。默认策略 `hover`，不开场景证据。四组开关，轨迹相同，全部 SUCCESS。

| 臂 | 契约修正 | 跟踪修正 | would_reject | `TRACKING_BOUND` | 其他码 |
|---|---|---|---:|---:|---|
| base | 关 | 关 | 1.000 | 730 | `TIME:SWITCH_NOT_IN_FUTURE` 10000，`TRANSITION:HEAD_DISCONTINUITY` 10000 |
| tracking | 关 | 开 | 1.000 | 730 | 同上。跟踪码从未单独出现，关会拒率不变。 |
| contract | 开 | 关 | 0.000 | 0 | 空。SH3 会从码表拿掉跟踪测量。 |
| both | 开 | 开 | 0.000 | 730 | 只剩 0.15 m 超界记录，不再打满 `would_reject`。 |

## SH2 / SH2b 日志

用票 SH2 / SH2b 已经写入 `result.json` 的直方图和稀疏 trace，不再重跑容器。开臂合计（seeds 0–4，种子 3、4 用隔离重跑）9759 tick，`would_reject` 0.997，`TRACKING_BOUND` 6445。这是只开环境证据、还没有开 SH3 / SH4 开关时的数。跟踪码与契约码大量重叠，没有逐 tick 日志就无法从直方图反推「只去掉跟踪之后」的精确会拒率；本票没有为此新开 SITL。

降采样 trace（每机大约 80–90 点）的欧氏误差 p95 在 1.49–1.77 m，10 趟里每趟都有约四分之一到三分之一的样本超过 0.15 m。这个口径粗于 B2 的规划时刻横向 p95，但足够说明：SITL 跟飞经常越过认证的 0.15 m，而同一趟仍可以 3/3 到达。

## 危险卡

单元复现，开关打开：

- 机体偏离批准点 0.16 m，无障碍：记下 `TRACKING_BOUND`，`would_reject` 为假。0.15 m 整好不触发。
- 沿路径落后 3 m（20 s × 0.15 m/s，超过 `lag_limit` 1.5 m）：记下 `LAG_VIOLATION`，`would_reject` 仍为真。
- 4 s 内走完 5 m、穿过 g2 柱子、峰值在上游限幅内：选择门给出 `ENV:OCCUPIED`，`would_reject` 仍为真。没有靠放宽 0.15 m 把穿柱放过。
