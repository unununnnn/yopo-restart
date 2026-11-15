# 票 E-PG4：0.7 s 短前缀在冻结套件上重测（只报数）

只报数。report-only。task.leader_prefix 默认关。本表把 E-PG3 里 0.7 s 那一档（progress_gate 开、leader_margin_m 缺省 1.0、progress_percentile 0、leader_prefix_s=0.7）在进程里打开后重跑已冻 M0–M4，标偏离上游。不把开关写回场景卡，不改默认，不改 core/、yopo_l0/crossing.py、yopo_l0/progress.py、scenes/suite.txt、已冻门、docs/tickets.md、README.md、scripts/premerge.sh、scripts/regress.sh、scripts/sitl/Dockerfile。后段不打开；预算按评估次数，不用墙钟截断。破门不挡合入。未冻结，不进 scenes/suite.txt，不设新门，不改已冻门。

点质量一列来自 [SUMMARY](../results/SUMMARY.md)。短前缀一列是本票打开开关后的重跑；编队和单机那几行开关不作用，重跑的是场景卡默认。净空 = 机体中心到障碍表面的距离 − 0.35 m。开阔区写「离地」。队形误差 p95 多个 seed 写出最小到最大；最小机间和最小净空取最紧的那一次。到达时间是有全体到达的运行的 `arrival_time_s` 均值，括号里是最小到最大，以及有全体到达的运行次数。墙钟不进门。后段没有打开，预算模式是评估次数。

这八张卡的文件哈希与 SUMMARY 的 scene_card_hash 一致。

## 开关

已有接口是场景卡 `task.leader_prefix` / `task.leader_prefix_s`，由 `CrossingFleet` 读取。默认关。关时不写这两个字段，超前机仍先悬停，悬停接不上才退到 0.3 s。本票在进程里对四张 crossing 卡写成 `leader_prefix=true`、`leader_prefix_s=0.7`，并把 `progress_gate` 设为 true。`leader_margin_m` 不写，缺省 1.0。`progress_percentile` 不改，缺省 0。这是 E-PG3 prefix07 那一档：两张六机林带在 50 s 内 60/60 到达、碰撞 0、机间不低于 0.70 m。磁盘上的场景卡不变。编队卡若把门控写成 true，`FormationFleet` 会拒绝，所以那几行保持场景卡默认。

打开后结果写 `leader_prefix`、`leader_prefix_s`，并标「偏离上游」。关时这两个字段都不出现。

模型修订 `e-pg4-prefix07-gate-on-2026-11-15`。跑数 commit `ee2c97ff37a8012b028fbd8926c898ee66bb1ac8`。

## 关时抽查

已冻 `crossing-6-forest` seed 0、开关关、影子盾开，与当前 main 的结果逐位一致。比较时去掉了 commit, plan_wall_ms, wall_time_s。夹具是 `tests/fixtures/pg2_prechange_crossing6_forest_seed0_result.json`。到达 6/6，结果 `SUCCESS`，结果里没有 `leader_prefix`，也没有标偏离上游。

## 和 SUMMARY 逐行对照

| 里程碑 | 场景 | 档 | 配置 | 到达 | 碰撞 | 最小机间 m | 最小净空 m | 队形误差 p95 m | 到达时间 s | 冻结门 |
|---|---|---|---|---:|---:|---|---|---|---|---|
| M0 | g2-pillar | — | 点质量（SUMMARY） | 10/10 | 0 | — | 0.478 | — | 4.271（4.26–4.28，10/10 次有全体到达） | 通过 |
| M0 | g2-pillar | — | 场景卡默认（短前缀不作用） | 10/10 | 0 | — | 0.478 | — | 4.271（4.26–4.28，10/10 次有全体到达） | 通过 |
| M1 | formation-hold | — | 点质量（SUMMARY） | 9/9 | 0 | 1.987 | 离地 1.150 | 0.027 | 18.630（18.63–18.63，3/3 次有全体到达） | 通过 |
| M1 | formation-hold | — | 场景卡默认（短前缀不作用） | 9/9 | 0 | 1.987 | 离地 1.150 | 0.027 | 18.630（18.63–18.63，3/3 次有全体到达） | 通过 |
| M2 | crossing-pillars | — | 点质量（SUMMARY） | 30/30 | 0 | 3.350 | 0.528 | 4.130–4.610 | 7.270（7.05–7.77，10/10 次有全体到达） | 通过 |
| M2 | crossing-pillars | — | 0.7 s 短前缀（偏离上游） | 30/30 | 0 | 3.049 | 0.672 | 2.633–3.343 | 8.046（7.69–8.32，10/10 次有全体到达） | 通过 |
| M2 | crossing-forest（medium） | — | 点质量（SUMMARY） | 30/30 | 0 | 4.602 | 0.260 | 4.094–4.818 | 23.235（20.77–24.42，10/10 次有全体到达） | 通过 |
| M2 | crossing-forest（medium） | — | 0.7 s 短前缀（偏离上游） | 24/30 | 2（障碍 2，机间 0） | 4.034 | -0.005 | 2.199–2.796 | 14.520（14.28–15.13，8/10 次有全体到达） | 破：arrival_rate、collisions |
| M3 | formation-switch | — | 点质量（SUMMARY） | 30/30 | 0 | 1.536（过渡 1.536） | 0.693 | 前 0.025 / 后 0.025（全程 0.036） | 27.000（27.00–27.00，10/10 次有全体到达） | 通过 |
| M3 | formation-switch | — | 场景卡默认（短前缀不作用） | 30/30 | 0 | 1.536（过渡 1.536） | 0.693 | 前 0.025 / 后 0.025（全程 0.036） | 27.000（27.00–27.00，10/10 次有全体到达） | 通过 |
| M4 | formation-6-open | none | 点质量（SUMMARY） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | none | 场景卡默认（短前缀不作用） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | drop-5 | 点质量（SUMMARY） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | drop-5 | 场景卡默认（短前缀不作用） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | drop-20 | 点质量（SUMMARY） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | drop-20 | 场景卡默认（短前缀不作用） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | duplicate-10 | 点质量（SUMMARY） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | duplicate-10 | 场景卡默认（短前缀不作用） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | reorder-delay | 点质量（SUMMARY） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | reorder-delay | 场景卡默认（短前缀不作用） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | combined | 点质量（SUMMARY） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | combined | 场景卡默认（短前缀不作用） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | crossing-6-forest（medium） | — | 点质量（SUMMARY） | 60/60 | 0 | 1.111 | 0.126 | 23.041–26.298 | 28.607（26.00–33.57，10/10 次有全体到达） | 通过 |
| M4 | crossing-6-forest（medium） | — | 0.7 s 短前缀（偏离上游） | 60/60 | 0 | 1.063 | 0.122 | 4.260–5.525 | 42.714（40.01–46.69，10/10 次有全体到达） | 通过 |
| M4 | crossing-6-forest-sparse（sparse） | — | 点质量（SUMMARY） | 60/60 | 0 | 0.959 | 0.158 | 18.725–22.922 | 26.741（23.24–31.58，10/10 次有全体到达） | 通过 |
| M4 | crossing-6-forest-sparse（sparse） | — | 0.7 s 短前缀（偏离上游） | 60/60 | 0 | 0.954 | 0.114 | 3.785–5.471 | 36.840（34.23–40.73，10/10 次有全体到达） | 通过 |

## 哪些冻结门会破

下面这些行的冻结门在这次重跑里不成立。点质量在 SUMMARY 里这些门是过的。破门不挡合入。

### M2 crossing-forest（medium） / —

配置：0.7 s 短前缀（偏离上游）。门槛：`{"arrival_rate": 1.0, "collisions": 0.0, "formation_error_p95_m": 6.0, "min_inter_drone_distance_m": 2.0}`。

- crossing-forest：seed 1 arrival_rate 0.000 < 1.000
- crossing-forest：seed 1 collisions 1 > 0
- crossing-forest：seed 8 arrival_rate 0.000 < 1.000
- crossing-forest：seed 8 collisions 1 > 0
- crossing-forest：合计到达率 arrival_rate 0.800 < 1.000
- crossing-forest：合计 collisions 2 > 0

仍然成立的冻结行：g2-pillar/—、formation-hold/—、crossing-pillars/—、formation-switch/—、formation-6-open/none、formation-6-open/drop-5、formation-6-open/drop-20、formation-6-open/duplicate-10、formation-6-open/reorder-delay、formation-6-open/combined、crossing-6-forest（medium）/—、crossing-6-forest-sparse（sparse）/—。

## 六机林带与 E-PG3 对照

E-PG3 prefix07 已经报过这两张卡 seeds 0–9。这里在本票环境重跑，用来确认，不把那张表抄进来当结果。

20 个种子的到达、障碍碰撞、机间碰撞、队形 p95、最小机间、最小净空、到达时间、LEADER_PREFIX 次数都与 E-PG3 prefix07 一致。

## 结论

冻结门破了 1 行，其余 12 行仍过。M2 crossing-forest（medium）（短前缀打开）破了 arrival_rate、collisions：到达 24/30（点质量 30/30），碰撞 2（点质量 0）。
短前缀打开的是 4 行穿林/柱林，结果标偏离上游。另外 9 行是编队或单机 goto，不读 task.leader_prefix，按场景卡默认重跑，不标偏离上游。
两张六机林带 seeds 0–9 在本环境重跑后，到达、碰撞、队形 p95、最小机间、最小净空、到达时间、LEADER_PREFIX 次数与 E-PG3 prefix07 一致。这是重跑，不是从 E-PG3 抄来的。
这些数只报数，不改默认，也不改已冻门。破了的门不挡合入。

## 分种子

| 场景 | 档 | seed | 配置 | 结果 | 到达 | 碰撞 | 最小净空 m | 最小机间 m | 队形 p95 m | 到达时间 s |
|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|
| g2-pillar | — | 0 | 默认 | SUCCESS | 1/1 | 0 | 0.479 | — | — | 4.26 |
| g2-pillar | — | 1 | 默认 | SUCCESS | 1/1 | 0 | 0.479 | — | — | 4.27 |
| g2-pillar | — | 2 | 默认 | SUCCESS | 1/1 | 0 | 0.479 | — | — | 4.26 |
| g2-pillar | — | 3 | 默认 | SUCCESS | 1/1 | 0 | 0.479 | — | — | 4.26 |
| g2-pillar | — | 4 | 默认 | SUCCESS | 1/1 | 0 | 0.479 | — | — | 4.27 |
| g2-pillar | — | 5 | 默认 | SUCCESS | 1/1 | 0 | 0.478 | — | — | 4.28 |
| g2-pillar | — | 6 | 默认 | SUCCESS | 1/1 | 0 | 0.478 | — | — | 4.28 |
| g2-pillar | — | 7 | 默认 | SUCCESS | 1/1 | 0 | 0.478 | — | — | 4.28 |
| g2-pillar | — | 8 | 默认 | SUCCESS | 1/1 | 0 | 0.478 | — | — | 4.28 |
| g2-pillar | — | 9 | 默认 | SUCCESS | 1/1 | 0 | 0.479 | — | — | 4.27 |
| formation-hold | — | 0 | 默认 | SUCCESS | 3/3 | 0 | 1.150 | 1.987 | 0.027 | 18.63 |
| formation-hold | — | 1 | 默认 | SUCCESS | 3/3 | 0 | 1.150 | 1.987 | 0.027 | 18.63 |
| formation-hold | — | 2 | 默认 | SUCCESS | 3/3 | 0 | 1.150 | 1.987 | 0.027 | 18.63 |
| crossing-pillars | — | 0 | 0.7 s | SUCCESS | 3/3 | 0 | 0.944 | 7.298 | 2.951 | 7.75 |
| crossing-pillars | — | 1 | 0.7 s | SUCCESS | 3/3 | 0 | 0.836 | 7.310 | 2.962 | 8.32 |
| crossing-pillars | — | 2 | 0.7 s | SUCCESS | 3/3 | 0 | 0.913 | 7.289 | 2.962 | 7.75 |
| crossing-pillars | — | 3 | 0.7 s | SUCCESS | 3/3 | 0 | 0.976 | 7.240 | 2.866 | 8.09 |
| crossing-pillars | — | 4 | 0.7 s | SUCCESS | 3/3 | 0 | 0.852 | 7.319 | 2.974 | 8.32 |
| crossing-pillars | — | 5 | 0.7 s | SUCCESS | 3/3 | 0 | 0.889 | 3.839 | 3.142 | 7.69 |
| crossing-pillars | — | 6 | 0.7 s | SUCCESS | 3/3 | 0 | 0.893 | 3.049 | 3.343 | 7.97 |
| crossing-pillars | — | 7 | 0.7 s | SUCCESS | 3/3 | 0 | 0.672 | 6.896 | 3.240 | 7.94 |
| crossing-pillars | — | 8 | 0.7 s | SUCCESS | 3/3 | 0 | 0.866 | 7.619 | 2.633 | 8.32 |
| crossing-pillars | — | 9 | 0.7 s | SUCCESS | 3/3 | 0 | 0.852 | 7.600 | 2.816 | 8.31 |
| crossing-forest | — | 0 | 0.7 s | SUCCESS | 3/3 | 0 | 0.615 | 4.034 | 2.777 | 15.13 |
| crossing-forest | — | 1 | 0.7 s | COLLISION | 0/3 | 1 | -0.003 | 5.183 | 2.265 | — |
| crossing-forest | — | 2 | 0.7 s | SUCCESS | 3/3 | 0 | 0.782 | 5.162 | 2.232 | 14.55 |
| crossing-forest | — | 3 | 0.7 s | SUCCESS | 3/3 | 0 | 0.672 | 4.606 | 2.443 | 14.36 |
| crossing-forest | — | 4 | 0.7 s | SUCCESS | 3/3 | 0 | 0.515 | 4.218 | 2.796 | 14.55 |
| crossing-forest | — | 5 | 0.7 s | SUCCESS | 3/3 | 0 | 0.445 | 4.076 | 2.668 | 14.65 |
| crossing-forest | — | 6 | 0.7 s | SUCCESS | 3/3 | 0 | 0.540 | 4.342 | 2.438 | 14.28 |
| crossing-forest | — | 7 | 0.7 s | SUCCESS | 3/3 | 0 | 0.250 | 4.051 | 2.199 | 14.34 |
| crossing-forest | — | 8 | 0.7 s | COLLISION | 0/3 | 1 | -0.005 | 4.694 | 2.305 | — |
| crossing-forest | — | 9 | 0.7 s | SUCCESS | 3/3 | 0 | 0.530 | 4.892 | 2.269 | 14.30 |
| formation-switch | — | 0 | 默认 | SUCCESS | 3/3 | 0 | 0.693 | 1.536（过渡 1.536） | 前 0.025 / 后 0.025（全程 0.036） | 27.00 |
| formation-switch | — | 1 | 默认 | SUCCESS | 3/3 | 0 | 0.693 | 1.536（过渡 1.536） | 前 0.025 / 后 0.025（全程 0.036） | 27.00 |
| formation-switch | — | 2 | 默认 | SUCCESS | 3/3 | 0 | 0.693 | 1.536（过渡 1.536） | 前 0.025 / 后 0.025（全程 0.036） | 27.00 |
| formation-switch | — | 3 | 默认 | SUCCESS | 3/3 | 0 | 0.693 | 1.536（过渡 1.536） | 前 0.025 / 后 0.025（全程 0.036） | 27.00 |
| formation-switch | — | 4 | 默认 | SUCCESS | 3/3 | 0 | 0.693 | 1.536（过渡 1.536） | 前 0.025 / 后 0.025（全程 0.036） | 27.00 |
| formation-switch | — | 5 | 默认 | SUCCESS | 3/3 | 0 | 0.693 | 1.536（过渡 1.536） | 前 0.025 / 后 0.025（全程 0.036） | 27.00 |
| formation-switch | — | 6 | 默认 | SUCCESS | 3/3 | 0 | 0.693 | 1.536（过渡 1.536） | 前 0.025 / 后 0.025（全程 0.036） | 27.00 |
| formation-switch | — | 7 | 默认 | SUCCESS | 3/3 | 0 | 0.693 | 1.536（过渡 1.536） | 前 0.025 / 后 0.025（全程 0.036） | 27.00 |
| formation-switch | — | 8 | 默认 | SUCCESS | 3/3 | 0 | 0.693 | 1.536（过渡 1.536） | 前 0.025 / 后 0.025（全程 0.036） | 27.00 |
| formation-switch | — | 9 | 默认 | SUCCESS | 3/3 | 0 | 0.693 | 1.536（过渡 1.536） | 前 0.025 / 后 0.025（全程 0.036） | 27.00 |
| formation-6-open | none | 0 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | none | 1 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | none | 2 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | none | 3 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | none | 4 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | drop-5 | 0 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | drop-5 | 1 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | drop-5 | 2 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | drop-5 | 3 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | drop-5 | 4 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | drop-20 | 0 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | drop-20 | 1 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | drop-20 | 2 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | drop-20 | 3 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | drop-20 | 4 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | duplicate-10 | 0 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | duplicate-10 | 1 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | duplicate-10 | 2 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | duplicate-10 | 3 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | duplicate-10 | 4 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | reorder-delay | 0 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | reorder-delay | 1 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | reorder-delay | 2 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | reorder-delay | 3 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | reorder-delay | 4 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | combined | 0 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | combined | 1 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | combined | 2 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | combined | 3 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| formation-6-open | combined | 4 | 默认 | SUCCESS | 6/6 | 0 | 1.150 | 2.980 | 0.066 | 20.13 |
| crossing-6-forest | — | 0 | 0.7 s | SUCCESS | 6/6 | 0 | 0.303 | 1.765 | 4.260 | 41.90 |
| crossing-6-forest | — | 1 | 0.7 s | SUCCESS | 6/6 | 0 | 0.160 | 1.456 | 4.939 | 46.69 |
| crossing-6-forest | — | 2 | 0.7 s | SUCCESS | 6/6 | 0 | 0.241 | 1.471 | 5.525 | 44.42 |
| crossing-6-forest | — | 3 | 0.7 s | SUCCESS | 6/6 | 0 | 0.289 | 1.305 | 4.540 | 40.48 |
| crossing-6-forest | — | 4 | 0.7 s | SUCCESS | 6/6 | 0 | 0.232 | 1.427 | 4.273 | 41.94 |
| crossing-6-forest | — | 5 | 0.7 s | SUCCESS | 6/6 | 0 | 0.214 | 1.468 | 4.424 | 42.26 |
| crossing-6-forest | — | 6 | 0.7 s | SUCCESS | 6/6 | 0 | 0.125 | 1.401 | 5.398 | 42.75 |
| crossing-6-forest | — | 7 | 0.7 s | SUCCESS | 6/6 | 0 | 0.177 | 1.119 | 5.062 | 43.98 |
| crossing-6-forest | — | 8 | 0.7 s | SUCCESS | 6/6 | 0 | 0.187 | 1.063 | 5.001 | 42.71 |
| crossing-6-forest | — | 9 | 0.7 s | SUCCESS | 6/6 | 0 | 0.122 | 1.268 | 4.401 | 40.01 |
| crossing-6-forest-sparse | — | 0 | 0.7 s | SUCCESS | 6/6 | 0 | 0.365 | 0.954 | 4.107 | 38.03 |
| crossing-6-forest-sparse | — | 1 | 0.7 s | SUCCESS | 6/6 | 0 | 0.231 | 1.006 | 4.228 | 37.26 |
| crossing-6-forest-sparse | — | 2 | 0.7 s | SUCCESS | 6/6 | 0 | 0.401 | 1.022 | 5.395 | 39.04 |
| crossing-6-forest-sparse | — | 3 | 0.7 s | SUCCESS | 6/6 | 0 | 0.179 | 1.035 | 4.106 | 34.27 |
| crossing-6-forest-sparse | — | 4 | 0.7 s | SUCCESS | 6/6 | 0 | 0.317 | 1.010 | 4.923 | 37.21 |
| crossing-6-forest-sparse | — | 5 | 0.7 s | SUCCESS | 6/6 | 0 | 0.114 | 0.968 | 5.471 | 40.73 |
| crossing-6-forest-sparse | — | 6 | 0.7 s | SUCCESS | 6/6 | 0 | 0.372 | 1.072 | 4.312 | 35.55 |
| crossing-6-forest-sparse | — | 7 | 0.7 s | SUCCESS | 6/6 | 0 | 0.241 | 1.003 | 4.003 | 35.11 |
| crossing-6-forest-sparse | — | 8 | 0.7 s | SUCCESS | 6/6 | 0 | 0.150 | 1.004 | 4.232 | 36.97 |
| crossing-6-forest-sparse | — | 9 | 0.7 s | SUCCESS | 6/6 | 0 | 0.198 | 1.015 | 3.785 | 34.23 |

## 复现

```
.venv/bin/python scripts/e-pg4-prefix-suite.py --jobs 2
.venv/bin/python -m pytest tests/test_e_pg4_prefix_suite.py -q
PYTHONPATH=.:core/src .venv/bin/python -m pytest -q -n 4 -m 'not slow' tests
```

缺省从 `out/e-pg4-prefix-suite/partial/` 续跑。`--no-resume` 忽略已有 partial。开关或种子清单变了，指纹对不上的 partial 不会被拿来用。关时抽查写在 `out/e-pg4-prefix-suite/spot-off.json`，代码指纹没变就不再跑。不要设置 `YOPO_BACKEND_BUDGET=wall`。本套件不打开后段。
