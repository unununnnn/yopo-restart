# 票 E-PG2：六机林带进度门控的超前余量做成开关

只报数。report-only。`_LEADER_MARGIN_M` 不再只是模块常量。`CrossingFleet` 在读 `progress_gate` 的同一处读 `task.leader_margin_m`，缺省 1.0。缺这个字段时，六机林带 seed 0 的结果与改前相同（比较时去掉墙钟和仓库 SHA）。没有改 `scenes/*.json`、`core/`、`scripts/regress.sh`、`scripts/premerge.sh`、`scenes/suite.txt`、README、`docs/tickets.md`，也没有改已冻门。场景卡仍是 `progress_gate=false`，卡上没有 `leader_margin_m`。

数字在 [e-pg2-forest-gate-switch-runs.json](e-pg2-forest-gate-switch-runs.json)。跑数时的 HEAD 是 `517c9b9904b007aa24d57bfb86baa27901af16cb`，工作区已经带上本票的读取。三进程，每个进程一个 OpenMP 线程。40 次墙钟相加 8409.7 s。墙钟不进门。

依据是 [ADR 0015](../adr/0015-ghost-brake-stops.md)：余量 1.0 m、分位 0 会按 min 慢爬；余量 3.0 m、分位 20 在 seeds 0–4 上把两张卡都恢复到 30/30，medium 的官方队形 p95 是 4.437–5.921 m。本票把这组余量做成场景卡字段，再在 seeds 0–9 上和基线对齐。打开的臂在进程里写 `progress_gate=true`、`leader_margin_m=3.0`、`progress_percentile=20`，整列标「report-only / 偏离当前场景卡配置」。

## 结论

seeds 0–9 上，打开臂两张卡都是 60/60 架次到达，障碍碰撞 0，机间碰撞 0，超时 0。基线同样是 60/60，碰撞 0，超时 0。

六机林带卡冻住的门是到达、碰撞和最小机间距离 ≥ 0.70 m。这三扇门在打开臂上守住了。最小机间 medium 是 0.998 m，sparse 是 0.982 m。净空最紧的是 medium open seed 0 的 0.121 m，只报数。

队形 p95 没有写进六机林带卡的 `criteria.gates`。拿三机穿林卡的 6.0 m 队形门来量，seeds 0–9 没有整段守住：medium open seed 8 是 6.086 m，sparse open seed 3 是 7.648 m。其余 18 粒都在 6.0 m 以下。seeds 0–4 的 medium 仍是 4.437–5.921 m，和先前那一臂相同。打开臂的结束门控弧长是 41.99–42.00 m，官方 p95 对着这条弧长上的槽位，比基线对着墙钟 × speed 的 18–26 m 小。门控还在起作用，平均 `LEADER_HOVER` 是 medium 19.5 次、sparse 8.5 次。

平均到达时间只平均 6/6。打开臂 medium 31.95 s，基线 28.61 s。打开臂 sparse 26.37 s，基线 26.74 s。

基线 seeds 0–9 与 [13e](../results/13e-ghost-stops.md) E+F / SUMMARY 的到达、机间、净空、官方 p95、到达时间逐种子对照，120 条都在 0.001 m / 0.015 s 内，没有 DIFF。

本表不把 `leader_margin_m=3.0` 写回场景卡，也不新冻队形门。

## 臂

| 臂 | 进程内设置 | 标注 |
|---|---|---|
| baseline | 磁盘上的卡，`progress_gate=false`。不写 `leader_margin_m`，构造时仍是缺省 1.0，门控关时摘要里这一格是空。 | 当前场景卡配置 |
| open | `progress_gate=true`，`leader_margin_m=3.0`，`progress_percentile=20` | report-only / 偏离当前场景卡配置 |

影子盾开，只记录，不改批准动作。没有后段。

## 口径

到达按架次。分母是该臂该场景的 10 × 6 = 60。碰撞读 `crossing` 块里的 `obstacle_collisions` 和 `inter_drone_collisions`，一次运行各至多 1。超时是 `outcome` 为 `TIMEOUT` 的种子数。这次 40 次的碰撞和超时都是 0。

队形误差 p95 是结果里的 `formation_error_p95_m`。门控开时对着门控弧长上的槽位，门控关时对着墙钟 × speed。两列口径不同。最小机间是中心距，表里写各种子的范围，左端是最小值。最小净空是机体中心到障碍表面的距离减去机体半径 0.35 m。平均到达时间只平均 6/6 的 `arrival_time_s`。`LEADER_HOVER` 是提案窗口次数，表上是 10 个种子的算术平均。结束门控弧长是 `gate_s_m`。航段长度 42 m。门控关时这一列是 —。

### 合计

| 场景 | 臂 | 参数 | 标注 | 到达 | 障碍碰撞 | 机间碰撞 | 超时 | 队形误差 p95 m | 最小机间 m | 最小净空 m | 平均到达时间 s | LEADER_HOVER | 结束门控弧长 m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| medium | baseline | progress_gate=false | 当前场景卡配置 | 60/60 | 0 | 0 | 0 | 23.041–26.298 | 1.111–1.392 | 0.126–0.306 | 28.61 | 0.0 | — |
| medium | open | progress_gate=true, leader_margin_m=3.0, progress_percentile=20 | report-only / 偏离当前场景卡配置 | 60/60 | 0 | 0 | 0 | 4.437–6.086 | 0.998–1.101 | 0.121–0.215 | 31.95 | 19.5 | 41.99–42.00 |
| sparse | baseline | progress_gate=false | 当前场景卡配置 | 60/60 | 0 | 0 | 0 | 18.725–22.922 | 0.959–1.322 | 0.158–0.402 | 26.74 | 0.0 | — |
| sparse | open | progress_gate=true, leader_margin_m=3.0, progress_percentile=20 | report-only / 偏离当前场景卡配置 | 60/60 | 0 | 0 | 0 | 3.190–7.648 | 0.982–1.210 | 0.158–0.627 | 26.37 | 8.5 | 41.99–42.00 |

### 逐种子

| 场景 | 臂 | seed | 到达 | 结果 | 障碍碰撞 | 机间碰撞 | 超时 | 队形误差 p95 m | 最小机间 m | 最小净空 m | 到达时间 s | LEADER_HOVER | 门控弧长 m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| medium | baseline | 0 | 6/6 | SUCCESS | 0 | 0 | 0 | 23.472 | 1.293 | 0.306 | 26.08 | 0 | — |
| medium | baseline | 1 | 6/6 | SUCCESS | 0 | 0 | 0 | 23.813 | 1.392 | 0.161 | 28.13 | 0 | — |
| medium | baseline | 2 | 6/6 | SUCCESS | 0 | 0 | 0 | 24.591 | 1.148 | 0.202 | 26.90 | 0 | — |
| medium | baseline | 3 | 6/6 | SUCCESS | 0 | 0 | 0 | 24.195 | 1.189 | 0.239 | 29.08 | 0 | — |
| medium | baseline | 4 | 6/6 | SUCCESS | 0 | 0 | 0 | 23.041 | 1.111 | 0.194 | 28.40 | 0 | — |
| medium | baseline | 5 | 6/6 | SUCCESS | 0 | 0 | 0 | 26.298 | 1.260 | 0.126 | 33.57 | 0 | — |
| medium | baseline | 6 | 6/6 | SUCCESS | 0 | 0 | 0 | 24.213 | 1.224 | 0.191 | 32.07 | 0 | — |
| medium | baseline | 7 | 6/6 | SUCCESS | 0 | 0 | 0 | 26.108 | 1.248 | 0.218 | 26.90 | 0 | — |
| medium | baseline | 8 | 6/6 | SUCCESS | 0 | 0 | 0 | 24.769 | 1.361 | 0.170 | 26.00 | 0 | — |
| medium | baseline | 9 | 6/6 | SUCCESS | 0 | 0 | 0 | 25.937 | 1.187 | 0.180 | 28.94 | 0 | — |
| medium | open | 0 | 6/6 | SUCCESS | 0 | 0 | 0 | 5.921 | 1.101 | 0.121 | 33.37 | 19 | 42.00 |
| medium | open | 1 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.437 | 1.037 | 0.131 | 32.47 | 23 | 42.00 |
| medium | open | 2 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.488 | 1.025 | 0.175 | 30.55 | 20 | 42.00 |
| medium | open | 3 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.856 | 1.009 | 0.205 | 31.99 | 20 | 42.00 |
| medium | open | 4 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.783 | 0.998 | 0.172 | 30.41 | 15 | 42.00 |
| medium | open | 5 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.535 | 1.045 | 0.126 | 33.51 | 25 | 41.99 |
| medium | open | 6 | 6/6 | SUCCESS | 0 | 0 | 0 | 5.061 | 1.026 | 0.187 | 31.92 | 18 | 42.00 |
| medium | open | 7 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.995 | 1.070 | 0.147 | 32.00 | 23 | 41.99 |
| medium | open | 8 | 6/6 | SUCCESS | 0 | 0 | 0 | 6.086 | 1.069 | 0.215 | 33.42 | 17 | 42.00 |
| medium | open | 9 | 6/6 | SUCCESS | 0 | 0 | 0 | 5.312 | 1.025 | 0.180 | 29.85 | 15 | 42.00 |
| sparse | baseline | 0 | 6/6 | SUCCESS | 0 | 0 | 0 | 20.976 | 0.959 | 0.315 | 25.40 | 0 | — |
| sparse | baseline | 1 | 6/6 | SUCCESS | 0 | 0 | 0 | 18.725 | 0.982 | 0.178 | 23.24 | 0 | — |
| sparse | baseline | 2 | 6/6 | SUCCESS | 0 | 0 | 0 | 20.661 | 1.242 | 0.322 | 30.31 | 0 | — |
| sparse | baseline | 3 | 6/6 | SUCCESS | 0 | 0 | 0 | 19.778 | 0.972 | 0.380 | 31.58 | 0 | — |
| sparse | baseline | 4 | 6/6 | SUCCESS | 0 | 0 | 0 | 20.842 | 1.322 | 0.192 | 24.83 | 0 | — |
| sparse | baseline | 5 | 6/6 | SUCCESS | 0 | 0 | 0 | 20.075 | 1.039 | 0.253 | 24.40 | 0 | — |
| sparse | baseline | 6 | 6/6 | SUCCESS | 0 | 0 | 0 | 21.695 | 1.012 | 0.402 | 24.81 | 0 | — |
| sparse | baseline | 7 | 6/6 | SUCCESS | 0 | 0 | 0 | 20.476 | 1.002 | 0.186 | 29.30 | 0 | — |
| sparse | baseline | 8 | 6/6 | SUCCESS | 0 | 0 | 0 | 22.922 | 1.043 | 0.401 | 27.48 | 0 | — |
| sparse | baseline | 9 | 6/6 | SUCCESS | 0 | 0 | 0 | 21.293 | 1.053 | 0.158 | 26.06 | 0 | — |
| sparse | open | 0 | 6/6 | SUCCESS | 0 | 0 | 0 | 3.190 | 1.068 | 0.627 | 22.90 | 3 | 41.99 |
| sparse | open | 1 | 6/6 | SUCCESS | 0 | 0 | 0 | 3.810 | 0.982 | 0.178 | 25.97 | 8 | 42.00 |
| sparse | open | 2 | 6/6 | SUCCESS | 0 | 0 | 0 | 3.286 | 0.997 | 0.290 | 26.03 | 7 | 42.00 |
| sparse | open | 3 | 6/6 | SUCCESS | 0 | 0 | 0 | 7.648 | 1.021 | 0.226 | 28.01 | 13 | 42.00 |
| sparse | open | 4 | 6/6 | SUCCESS | 0 | 0 | 0 | 3.464 | 1.008 | 0.465 | 25.26 | 9 | 42.00 |
| sparse | open | 5 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.559 | 1.210 | 0.253 | 26.52 | 6 | 42.00 |
| sparse | open | 6 | 6/6 | SUCCESS | 0 | 0 | 0 | 5.091 | 1.014 | 0.595 | 29.44 | 17 | 42.00 |
| sparse | open | 7 | 6/6 | SUCCESS | 0 | 0 | 0 | 3.203 | 1.002 | 0.186 | 25.70 | 4 | 42.00 |
| sparse | open | 8 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.415 | 1.044 | 0.527 | 28.27 | 15 | 41.99 |
| sparse | open | 9 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.219 | 1.042 | 0.158 | 25.58 | 3 | 42.00 |
