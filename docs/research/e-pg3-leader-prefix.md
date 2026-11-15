# 票 E-PG3：超前机改走短前缀

只报数。report-only。`task.leader_prefix` 默认关。关时超前机仍先悬停，悬停接不上才退到 0.3 s 短前缀；`crossing-6-forest` seed 0 的结果与当前 main 逐位相同（比较时去掉墙钟和仓库 SHA）。打开后，门控判成超前的飞机先飞 `task.leader_prefix_s` 那么长的一段，接不上再等待（悬停）或续接上一份计划，结果标「偏离上游」。没有改 `scenes/*.json`、`core/`、`scripts/regress.sh`、`scripts/premerge.sh`、`scenes/suite.txt`、README、`docs/tickets.md`，也没有改已冻门。场景卡仍是 `progress_gate=false`，卡上没有 `leader_prefix`、`leader_prefix_s`、`leader_margin_m`。

数字在 [e-pg3-leader-prefix-runs.json](e-pg3-leader-prefix-runs.json)。跑数时的 HEAD 是 `defbd08984e68b769424e08d40dfa9c0f21c940a`。三进程，每个进程一个 OpenMP 线程。100 次墙钟相加 28240.2 s。墙钟不进门。逐粒落在 `out/e-pg3-leader-prefix/partial/`，可续跑。

依据是 [ADR 0015](../adr/0015-ghost-brake-stops.md) 和 [LIMITATIONS](../LIMITATIONS.md) 第 4 条：只修自检、门控仍开时，按 min 爬，50 s 内飞不完。可选方向里，放大余量和分位已经由 [E-PG1](e-forest-gate-ablation.md) 与 [E-PG2](e-pg2-forest-gate-switch.md) 测过。本票不重复余量 3 / 6 / 10 m，也不重复 E-PG2 的打开臂（`leader_margin_m=3.0`、`progress_percentile=20`）。门控开的对照用 E-PG2 的缺省余量 1.0 m、分位 0。前缀长度取 0.3 s（原来的 `_LEADER_PREFIX_S`，这次改成优先提交）、0.7 s（`_fit_yopo` 里已有的一档）和 1.0 s（短于六机一轮窗口 1.5 s）。

## 结论

seeds 0–9 上，0.7 s 短前缀两张卡都是 60/60 架次到达，障碍碰撞 0，机间碰撞 0，超时 0。基线同样是 60/60，碰撞 0，超时 0。0.3 s 仍飞不完：medium 21/60、9 个种子超时，sparse 20/60、8 个种子超时。1.0 s 在 medium 是 60/60，sparse 是 59/60，缺的是 sparse seed 2（5/6，结束门控弧长 12.62 m）。

只开门控、短前缀关，就是 E-PG1 arm1 的慢爬。medium 2/60，10 个种子全部超时，平均 `LEADER_HOVER` 125.6 次，`LEADER_PREFIX` 是 0。sparse 36/60，5 个种子超时，平均悬停 103.8 次，前缀仍是 0。seeds 0–4 与 E-PG1 arm1 的到达、官方 p95、机间、净空、到达时间、门控弧长逐条对照，没有 DIFF。seeds 5–9 是这一格新跑的：medium 五粒都是 0/6，sparse 里 seed 5、6、9 为 6/6，seed 7 为 2/6，seed 8 为 0/6，结束弧长停在 15.07 m。

六机林带卡冻住的门是到达、碰撞和最小机间距离 ≥ 0.70 m。0.7 s 这两扇到达和碰撞，以及机间门，在两张卡上都守住了。最小机间 medium 是 1.063 m（seed 8），sparse 是 0.954 m（seed 0）。1.0 s 的最小机间是 medium 0.985 m、sparse 0.854 m（seed 9），也在 0.70 m 以上；sparse 的到达不满格。0.3 s 和只开门控的到达不满格。净空最紧的是 sparse 1.0 s seed 8 的 0.092 m，其次是 sparse 0.7 s seed 5 的 0.114 m，只报数。100 次障碍碰撞和机间碰撞都是 0。

队形 p95 没有写进六机林带卡的 `criteria.gates`。门控开时它对着门控弧长上的槽位，所以比基线对着墙钟 × speed 的 18–26 m 小。拿三机穿林卡的 6.0 m 队形门来量，0.7 s 两张卡的 20 粒都在 6.0 m 以下：medium 4.260–5.525 m，sparse 3.785–5.471 m。1.0 s 没有整段守住：medium seed 3 是 10.878 m，seed 7 是 9.637 m；sparse 到达的 9 粒是 4.991–9.178 m，超时的 seed 2 对着停住的弧长，官方 p95 到 29.384 m。

平均到达时间只平均 6/6。0.7 s 是 medium 42.71 s、sparse 36.84 s，都长于基线的 28.61 s 和 26.74 s。1.0 s 的 medium 是 33.43 s，sparse 到达的 9 粒平均 31.36 s。0.3 s 能到齐的很少：medium 只有 seed 3 的 48.73 s，sparse 是 seed 1 的 46.50 s 和 seed 9 的 45.60 s。只开门控时，sparse 到齐的 5 粒平均 46.33 s，medium 没有 6/6。

短前缀打开以后，提案从悬停换成前缀。0.7 s 的平均 `LEADER_PREFIX` 是 medium 102.1 次、sparse 74.8 次，平均 `LEADER_HOVER` 降到 0.9 次和 2.3 次。1.0 s 更少，medium 79.1 次、sparse 69.2 次。0.3 s 的前缀次数最高（medium 113.1、sparse 122.5），门控弧长仍有停在 19–30 m 的种子。结束弧长在 0.7 s 和 medium 的 1.0 s 上贴着航段末端 42 m。

基线 seeds 0–9 与 [13e](../results/13e-ghost-stops.md) E+F / SUMMARY 的到达、机间、净空、官方 p95、到达时间逐种子对照，没有 DIFF。

本表不把 `leader_prefix` 写回场景卡，也不新冻队形门或净空门。开关保持默认关。

## 臂

| 臂 | 进程内设置 | 标注 |
|---|---|---|
| baseline | 磁盘上的卡，`progress_gate=false`。不写 `leader_prefix`。 | 当前场景卡配置 |
| gate | `progress_gate=true`，余量缺省 1.0，分位 0，`leader_prefix` 关 | report-only / 偏离当前场景卡配置 |
| prefix03 | 门控开，`leader_prefix=true`，`leader_prefix_s=0.3` | report-only / 偏离上游 |
| prefix07 | 门控开，`leader_prefix=true`，`leader_prefix_s=0.7` | report-only / 偏离上游 |
| prefix10 | 门控开，`leader_prefix=true`，`leader_prefix_s=1.0` | report-only / 偏离上游 |

影子盾开，只记录，不改批准动作。没有后段。

## 口径

到达按架次。分母是该臂该场景的 10 × 6 = 60。碰撞读 `crossing` 块里的 `obstacle_collisions` 和 `inter_drone_collisions`，一次运行各至多 1。超时是 `outcome` 为 `TIMEOUT` 的种子数。

队形误差 p95 是结果里的 `formation_error_p95_m`。门控开时对着门控弧长上的槽位，门控关时对着墙钟 × speed。两列口径不同。最小机间是中心距，表里写各种子的范围，左端是最小值。最小净空是机体中心到障碍表面的距离减去机体半径 0.35 m。平均到达时间只平均 6/6 的 `arrival_time_s`，不满 10 粒时括号里写到达齐的种子数。`LEADER_HOVER` 和 `LEADER_PREFIX` 是提案窗口次数，表上是 10 个种子的算术平均。结束门控弧长是 `gate_s_m`。航段长度 42 m。门控关时这一列是 —。

### 合计

| 场景 | 臂 | 参数 | 标注 | 到达 | 障碍碰撞 | 机间碰撞 | 超时 | 队形误差 p95 m | 最小机间 m | 最小净空 m | 平均到达时间 s | LEADER_HOVER | LEADER_PREFIX | 结束门控弧长 m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| medium | baseline | progress_gate=false | 当前场景卡配置 | 60/60 | 0 | 0 | 0 | 23.041–26.298 | 1.111–1.392 | 0.126–0.306 | 28.61 | 0.0 | 0.0 | — |
| medium | gate | progress_gate=true, leader_margin_m=1.0, progress_percentile=0, leader_prefix=false | report-only / 偏离当前场景卡配置 | 2/60 | 0 | 0 | 10 | 3.979–4.984 | 0.934–1.153 | 0.136–0.527 | — | 125.6 | 0.0 | 10.79–40.66 |
| medium | prefix03 | progress_gate=true, leader_margin_m=1.0, progress_percentile=0, leader_prefix=true, leader_prefix_s=0.3 | report-only / 偏离上游 | 21/60 | 0 | 0 | 9 | 4.449–5.863 | 1.008–1.214 | 0.152–0.208 | 48.73（1/10） | 0.2 | 113.1 | 23.42–42.00 |
| medium | prefix07 | progress_gate=true, leader_margin_m=1.0, progress_percentile=0, leader_prefix=true, leader_prefix_s=0.7 | report-only / 偏离上游 | 60/60 | 0 | 0 | 0 | 4.260–5.525 | 1.063–1.765 | 0.122–0.303 | 42.71 | 0.9 | 102.1 | 41.98–42.00 |
| medium | prefix10 | progress_gate=true, leader_margin_m=1.0, progress_percentile=0, leader_prefix=true, leader_prefix_s=1.0 | report-only / 偏离上游 | 60/60 | 0 | 0 | 0 | 4.849–10.878 | 0.985–1.653 | 0.140–0.235 | 33.43 | 1.9 | 79.1 | 41.99–42.00 |
| sparse | baseline | progress_gate=false | 当前场景卡配置 | 60/60 | 0 | 0 | 0 | 18.725–22.922 | 0.959–1.322 | 0.158–0.402 | 26.74 | 0.0 | 0.0 | — |
| sparse | gate | progress_gate=true, leader_margin_m=1.0, progress_percentile=0, leader_prefix=false | report-only / 偏离当前场景卡配置 | 36/60 | 0 | 0 | 5 | 4.264–5.281 | 0.957–1.046 | 0.148–0.612 | 46.33（5/10） | 103.8 | 0.0 | 15.07–42.00 |
| sparse | prefix03 | progress_gate=true, leader_margin_m=1.0, progress_percentile=0, leader_prefix=true, leader_prefix_s=0.3 | report-only / 偏离上游 | 20/60 | 0 | 0 | 8 | 4.548–7.009 | 0.981–1.203 | 0.143–0.322 | 46.05（2/10） | 0.5 | 122.5 | 19.38–42.00 |
| sparse | prefix07 | progress_gate=true, leader_margin_m=1.0, progress_percentile=0, leader_prefix=true, leader_prefix_s=0.7 | report-only / 偏离上游 | 60/60 | 0 | 0 | 0 | 3.785–5.471 | 0.954–1.072 | 0.114–0.401 | 36.84 | 2.3 | 74.8 | 41.98–42.00 |
| sparse | prefix10 | progress_gate=true, leader_margin_m=1.0, progress_percentile=0, leader_prefix=true, leader_prefix_s=1.0 | report-only / 偏离上游 | 59/60 | 0 | 0 | 1 | 4.991–29.384 | 0.854–1.156 | 0.092–0.472 | 31.36（9/10） | 1.1 | 69.2 | 12.62–42.00 |

### 逐种子

| 场景 | 臂 | seed | 到达 | 结果 | 障碍碰撞 | 机间碰撞 | 超时 | 队形误差 p95 m | 最小机间 m | 最小净空 m | 到达时间 s | LEADER_HOVER | LEADER_PREFIX | 门控弧长 m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| medium | baseline | 0 | 6/6 | SUCCESS | 0 | 0 | 0 | 23.472 | 1.293 | 0.306 | 26.08 | 0 | 0 | — |
| medium | baseline | 1 | 6/6 | SUCCESS | 0 | 0 | 0 | 23.813 | 1.392 | 0.161 | 28.13 | 0 | 0 | — |
| medium | baseline | 2 | 6/6 | SUCCESS | 0 | 0 | 0 | 24.591 | 1.148 | 0.202 | 26.90 | 0 | 0 | — |
| medium | baseline | 3 | 6/6 | SUCCESS | 0 | 0 | 0 | 24.195 | 1.189 | 0.239 | 29.08 | 0 | 0 | — |
| medium | baseline | 4 | 6/6 | SUCCESS | 0 | 0 | 0 | 23.041 | 1.111 | 0.194 | 28.40 | 0 | 0 | — |
| medium | baseline | 5 | 6/6 | SUCCESS | 0 | 0 | 0 | 26.298 | 1.260 | 0.126 | 33.57 | 0 | 0 | — |
| medium | baseline | 6 | 6/6 | SUCCESS | 0 | 0 | 0 | 24.213 | 1.224 | 0.191 | 32.07 | 0 | 0 | — |
| medium | baseline | 7 | 6/6 | SUCCESS | 0 | 0 | 0 | 26.108 | 1.248 | 0.218 | 26.90 | 0 | 0 | — |
| medium | baseline | 8 | 6/6 | SUCCESS | 0 | 0 | 0 | 24.769 | 1.361 | 0.170 | 26.00 | 0 | 0 | — |
| medium | baseline | 9 | 6/6 | SUCCESS | 0 | 0 | 0 | 25.937 | 1.187 | 0.180 | 28.94 | 0 | 0 | — |
| medium | gate | 0 | 2/6 | TIMEOUT | 0 | 0 | 1 | 4.613 | 1.030 | 0.157 | — | 102 | 0 | 40.66 |
| medium | gate | 1 | 0/6 | TIMEOUT | 0 | 0 | 1 | 4.666 | 1.035 | 0.149 | — | 116 | 0 | 35.52 |
| medium | gate | 2 | 0/6 | TIMEOUT | 0 | 0 | 1 | 4.556 | 1.065 | 0.161 | — | 124 | 0 | 34.85 |
| medium | gate | 3 | 0/6 | TIMEOUT | 0 | 0 | 1 | 4.179 | 1.103 | 0.527 | — | 148 | 0 | 10.79 |
| medium | gate | 4 | 0/6 | TIMEOUT | 0 | 0 | 1 | 4.464 | 1.153 | 0.237 | — | 121 | 0 | 35.91 |
| medium | gate | 5 | 0/6 | TIMEOUT | 0 | 0 | 1 | 4.075 | 0.962 | 0.159 | — | 124 | 0 | 31.20 |
| medium | gate | 6 | 0/6 | TIMEOUT | 0 | 0 | 1 | 3.979 | 1.052 | 0.136 | — | 113 | 0 | 39.13 |
| medium | gate | 7 | 0/6 | TIMEOUT | 0 | 0 | 1 | 4.459 | 0.940 | 0.294 | — | 140 | 0 | 14.91 |
| medium | gate | 8 | 0/6 | TIMEOUT | 0 | 0 | 1 | 4.984 | 1.036 | 0.197 | — | 146 | 0 | 13.76 |
| medium | gate | 9 | 0/6 | TIMEOUT | 0 | 0 | 1 | 4.387 | 0.934 | 0.169 | — | 122 | 0 | 35.67 |
| medium | prefix03 | 0 | 2/6 | TIMEOUT | 0 | 0 | 1 | 4.991 | 1.008 | 0.166 | — | 0 | 118 | 39.71 |
| medium | prefix03 | 1 | 0/6 | TIMEOUT | 0 | 0 | 1 | 4.694 | 1.077 | 0.153 | — | 0 | 115 | 41.45 |
| medium | prefix03 | 2 | 1/6 | TIMEOUT | 0 | 0 | 1 | 5.179 | 1.068 | 0.168 | — | 0 | 113 | 40.91 |
| medium | prefix03 | 3 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.878 | 1.053 | 0.188 | 48.73 | 0 | 101 | 42.00 |
| medium | prefix03 | 4 | 5/6 | TIMEOUT | 0 | 0 | 1 | 4.866 | 1.125 | 0.167 | — | 0 | 99 | 41.99 |
| medium | prefix03 | 5 | 0/6 | TIMEOUT | 0 | 0 | 1 | 4.449 | 1.010 | 0.172 | — | 0 | 125 | 30.09 |
| medium | prefix03 | 6 | 2/6 | TIMEOUT | 0 | 0 | 1 | 4.841 | 1.079 | 0.152 | — | 0 | 113 | 40.78 |
| medium | prefix03 | 7 | 0/6 | TIMEOUT | 0 | 0 | 1 | 5.863 | 1.017 | 0.171 | — | 1 | 130 | 23.42 |
| medium | prefix03 | 8 | 2/6 | TIMEOUT | 0 | 0 | 1 | 4.990 | 1.061 | 0.168 | — | 1 | 113 | 40.80 |
| medium | prefix03 | 9 | 3/6 | TIMEOUT | 0 | 0 | 1 | 5.062 | 1.214 | 0.208 | — | 0 | 104 | 41.68 |
| medium | prefix07 | 0 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.260 | 1.765 | 0.303 | 41.90 | 1 | 91 | 41.99 |
| medium | prefix07 | 1 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.939 | 1.456 | 0.160 | 46.69 | 1 | 119 | 42.00 |
| medium | prefix07 | 2 | 6/6 | SUCCESS | 0 | 0 | 0 | 5.525 | 1.471 | 0.241 | 44.42 | 0 | 114 | 41.99 |
| medium | prefix07 | 3 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.540 | 1.305 | 0.289 | 40.48 | 1 | 91 | 41.99 |
| medium | prefix07 | 4 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.273 | 1.427 | 0.232 | 41.94 | 0 | 103 | 41.99 |
| medium | prefix07 | 5 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.424 | 1.468 | 0.214 | 42.26 | 1 | 100 | 41.99 |
| medium | prefix07 | 6 | 6/6 | SUCCESS | 0 | 0 | 0 | 5.398 | 1.401 | 0.125 | 42.75 | 3 | 103 | 42.00 |
| medium | prefix07 | 7 | 6/6 | SUCCESS | 0 | 0 | 0 | 5.062 | 1.119 | 0.177 | 43.98 | 2 | 104 | 41.99 |
| medium | prefix07 | 8 | 6/6 | SUCCESS | 0 | 0 | 0 | 5.001 | 1.063 | 0.187 | 42.71 | 0 | 106 | 42.00 |
| medium | prefix07 | 9 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.401 | 1.268 | 0.122 | 40.01 | 0 | 90 | 41.98 |
| medium | prefix10 | 0 | 6/6 | SUCCESS | 0 | 0 | 0 | 7.072 | 1.015 | 0.155 | 35.31 | 1 | 90 | 41.99 |
| medium | prefix10 | 1 | 6/6 | SUCCESS | 0 | 0 | 0 | 7.182 | 1.653 | 0.193 | 31.80 | 1 | 75 | 41.99 |
| medium | prefix10 | 2 | 6/6 | SUCCESS | 0 | 0 | 0 | 7.156 | 1.107 | 0.140 | 33.46 | 2 | 77 | 41.99 |
| medium | prefix10 | 3 | 6/6 | SUCCESS | 0 | 0 | 0 | 10.878 | 0.985 | 0.162 | 38.81 | 4 | 99 | 41.99 |
| medium | prefix10 | 4 | 6/6 | SUCCESS | 0 | 0 | 0 | 6.362 | 1.208 | 0.159 | 33.28 | 1 | 78 | 42.00 |
| medium | prefix10 | 5 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.849 | 1.277 | 0.147 | 32.97 | 1 | 75 | 41.99 |
| medium | prefix10 | 6 | 6/6 | SUCCESS | 0 | 0 | 0 | 7.804 | 1.528 | 0.192 | 33.53 | 2 | 80 | 42.00 |
| medium | prefix10 | 7 | 6/6 | SUCCESS | 0 | 0 | 0 | 9.637 | 1.496 | 0.173 | 33.26 | 2 | 81 | 41.99 |
| medium | prefix10 | 8 | 6/6 | SUCCESS | 0 | 0 | 0 | 5.387 | 1.389 | 0.235 | 31.49 | 1 | 77 | 41.99 |
| medium | prefix10 | 9 | 6/6 | SUCCESS | 0 | 0 | 0 | 6.242 | 1.220 | 0.148 | 30.43 | 4 | 59 | 41.99 |
| sparse | baseline | 0 | 6/6 | SUCCESS | 0 | 0 | 0 | 20.976 | 0.959 | 0.315 | 25.40 | 0 | 0 | — |
| sparse | baseline | 1 | 6/6 | SUCCESS | 0 | 0 | 0 | 18.725 | 0.982 | 0.178 | 23.24 | 0 | 0 | — |
| sparse | baseline | 2 | 6/6 | SUCCESS | 0 | 0 | 0 | 20.661 | 1.242 | 0.322 | 30.31 | 0 | 0 | — |
| sparse | baseline | 3 | 6/6 | SUCCESS | 0 | 0 | 0 | 19.778 | 0.972 | 0.380 | 31.58 | 0 | 0 | — |
| sparse | baseline | 4 | 6/6 | SUCCESS | 0 | 0 | 0 | 20.842 | 1.322 | 0.192 | 24.83 | 0 | 0 | — |
| sparse | baseline | 5 | 6/6 | SUCCESS | 0 | 0 | 0 | 20.075 | 1.039 | 0.253 | 24.40 | 0 | 0 | — |
| sparse | baseline | 6 | 6/6 | SUCCESS | 0 | 0 | 0 | 21.695 | 1.012 | 0.402 | 24.81 | 0 | 0 | — |
| sparse | baseline | 7 | 6/6 | SUCCESS | 0 | 0 | 0 | 20.476 | 1.002 | 0.186 | 29.30 | 0 | 0 | — |
| sparse | baseline | 8 | 6/6 | SUCCESS | 0 | 0 | 0 | 22.922 | 1.043 | 0.401 | 27.48 | 0 | 0 | — |
| sparse | baseline | 9 | 6/6 | SUCCESS | 0 | 0 | 0 | 21.293 | 1.053 | 0.158 | 26.06 | 0 | 0 | — |
| sparse | gate | 0 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.843 | 1.007 | 0.148 | 47.78 | 91 | 0 | 41.99 |
| sparse | gate | 1 | 2/6 | TIMEOUT | 0 | 0 | 1 | 4.348 | 0.996 | 0.482 | — | 109 | 0 | 41.98 |
| sparse | gate | 2 | 2/6 | TIMEOUT | 0 | 0 | 1 | 4.658 | 0.997 | 0.489 | — | 113 | 0 | 40.12 |
| sparse | gate | 3 | 0/6 | TIMEOUT | 0 | 0 | 1 | 4.895 | 1.046 | 0.318 | — | 119 | 0 | 38.82 |
| sparse | gate | 4 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.852 | 1.032 | 0.386 | 46.51 | 92 | 0 | 41.98 |
| sparse | gate | 5 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.616 | 0.978 | 0.245 | 42.75 | 75 | 0 | 42.00 |
| sparse | gate | 6 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.264 | 1.037 | 0.274 | 47.08 | 92 | 0 | 42.00 |
| sparse | gate | 7 | 2/6 | TIMEOUT | 0 | 0 | 1 | 4.588 | 0.987 | 0.386 | — | 107 | 0 | 41.91 |
| sparse | gate | 8 | 0/6 | TIMEOUT | 0 | 0 | 1 | 5.281 | 0.957 | 0.612 | — | 143 | 0 | 15.07 |
| sparse | gate | 9 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.384 | 1.018 | 0.311 | 47.51 | 97 | 0 | 42.00 |
| sparse | prefix03 | 0 | 1/6 | TIMEOUT | 0 | 0 | 1 | 4.808 | 1.001 | 0.308 | — | 1 | 126 | 38.96 |
| sparse | prefix03 | 1 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.779 | 1.005 | 0.143 | 46.50 | 0 | 103 | 41.99 |
| sparse | prefix03 | 2 | 2/6 | TIMEOUT | 0 | 0 | 1 | 5.378 | 1.020 | 0.145 | — | 2 | 108 | 41.98 |
| sparse | prefix03 | 3 | 0/6 | TIMEOUT | 0 | 0 | 1 | 6.730 | 0.984 | 0.310 | — | 0 | 141 | 19.39 |
| sparse | prefix03 | 4 | 0/6 | TIMEOUT | 0 | 0 | 1 | 7.009 | 1.021 | 0.162 | — | 0 | 138 | 21.17 |
| sparse | prefix03 | 5 | 0/6 | TIMEOUT | 0 | 0 | 1 | 4.729 | 1.046 | 0.170 | — | 0 | 141 | 21.15 |
| sparse | prefix03 | 6 | 5/6 | TIMEOUT | 0 | 0 | 1 | 4.790 | 1.203 | 0.232 | — | 1 | 107 | 42.00 |
| sparse | prefix03 | 7 | 0/6 | TIMEOUT | 0 | 0 | 1 | 5.494 | 1.078 | 0.322 | — | 0 | 122 | 40.93 |
| sparse | prefix03 | 8 | 0/6 | TIMEOUT | 0 | 0 | 1 | 5.445 | 0.981 | 0.170 | — | 0 | 143 | 19.38 |
| sparse | prefix03 | 9 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.548 | 1.042 | 0.181 | 45.60 | 1 | 96 | 42.00 |
| sparse | prefix07 | 0 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.107 | 0.954 | 0.365 | 38.03 | 3 | 75 | 41.98 |
| sparse | prefix07 | 1 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.228 | 1.006 | 0.231 | 37.26 | 3 | 77 | 42.00 |
| sparse | prefix07 | 2 | 6/6 | SUCCESS | 0 | 0 | 0 | 5.395 | 1.022 | 0.401 | 39.04 | 2 | 88 | 41.99 |
| sparse | prefix07 | 3 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.106 | 1.035 | 0.179 | 34.27 | 0 | 65 | 42.00 |
| sparse | prefix07 | 4 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.923 | 1.010 | 0.317 | 37.21 | 2 | 76 | 42.00 |
| sparse | prefix07 | 5 | 6/6 | SUCCESS | 0 | 0 | 0 | 5.471 | 0.968 | 0.114 | 40.73 | 2 | 89 | 42.00 |
| sparse | prefix07 | 6 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.312 | 1.072 | 0.372 | 35.55 | 5 | 69 | 41.99 |
| sparse | prefix07 | 7 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.003 | 1.003 | 0.241 | 35.11 | 1 | 72 | 42.00 |
| sparse | prefix07 | 8 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.232 | 1.004 | 0.150 | 36.97 | 3 | 75 | 42.00 |
| sparse | prefix07 | 9 | 6/6 | SUCCESS | 0 | 0 | 0 | 3.785 | 1.015 | 0.198 | 34.23 | 2 | 62 | 42.00 |
| sparse | prefix10 | 0 | 6/6 | SUCCESS | 0 | 0 | 0 | 6.633 | 1.074 | 0.472 | 29.50 | 1 | 63 | 42.00 |
| sparse | prefix10 | 1 | 6/6 | SUCCESS | 0 | 0 | 0 | 6.262 | 1.028 | 0.182 | 29.49 | 1 | 59 | 41.99 |
| sparse | prefix10 | 2 | 5/6 | TIMEOUT | 0 | 0 | 1 | 29.384 | 1.058 | 0.314 | — | 1 | 94 | 12.62 |
| sparse | prefix10 | 3 | 6/6 | SUCCESS | 0 | 0 | 0 | 7.945 | 1.029 | 0.312 | 32.49 | 1 | 75 | 41.99 |
| sparse | prefix10 | 4 | 6/6 | SUCCESS | 0 | 0 | 0 | 7.039 | 1.003 | 0.146 | 34.26 | 3 | 74 | 41.99 |
| sparse | prefix10 | 5 | 6/6 | SUCCESS | 0 | 0 | 0 | 8.328 | 1.156 | 0.344 | 30.46 | 0 | 65 | 42.00 |
| sparse | prefix10 | 6 | 6/6 | SUCCESS | 0 | 0 | 0 | 5.700 | 1.045 | 0.403 | 34.03 | 2 | 74 | 42.00 |
| sparse | prefix10 | 7 | 6/6 | SUCCESS | 0 | 0 | 0 | 6.744 | 1.079 | 0.282 | 27.65 | 1 | 56 | 41.98 |
| sparse | prefix10 | 8 | 6/6 | SUCCESS | 0 | 0 | 0 | 9.178 | 1.046 | 0.092 | 34.77 | 0 | 75 | 42.00 |
| sparse | prefix10 | 9 | 6/6 | SUCCESS | 0 | 0 | 0 | 4.991 | 0.854 | 0.201 | 29.55 | 1 | 57 | 42.00 |
