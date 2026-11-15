# 票 E-SO3b：冻结套件上的一阶 SO3，只报数

只报数。report-only。SO3 开关默认关；本表是打开后的冻结套件，标偏离上游。参数来自 E-SO3 标称档和 PX4 公开默认值，置信度写在模型一节，不是本仓库的 SITL 辨识。不改 core/、场景卡默认值、scenes/suite.txt、已冻门、docs/tickets.md、README.md、scripts/premerge.sh、scripts/regress.sh、scripts/sitl/Dockerfile。不重复 E-SO3 已经量化的二阶两档。未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。

点质量一列来自 [SUMMARY](../results/SUMMARY.md)（commit `5524cab42584`，场景卡哈希与当前这八张卡一致）。SO3 一列是本票打开开关后的重跑。净空 = 机体中心到障碍表面的距离 − 0.35 m。开阔区写「离地」。队形误差 p95 多个 seed 写出最小到最大；最小机间和最小净空取最紧的那一次。到达时间是有全体到达的运行的 `arrival_time_s` 均值，括号里是最小到最大，以及有全体到达的运行次数。E-SO3 已经报过二阶标称档和慢姿态档在 G2、三机柱群、三机林、六机稀疏上的差距，这里不重复那张表。

## 模型

开关在 `RunConfig.so3`，也可以把场景卡 `dynamics.model` 写成 `so3_lag`。默认关。关时运行器仍调用 `make_model(场景卡动力学)`，不导入 `yopo_l0/so3.py`。打开后再标「偏离上游」。本套件用 `RunConfig.so3=True`，不改场景卡文件。

姿态是一阶：角速度命令 `ω = clip(−e_R / τ_att)`，误差是 Lee, Leok, McClamroch, CDC 2010 的 SO(3) vee。推力对比力命令做一阶滞后，再限制在 `[0, g+a_max]`。倾角限制在锥内。滚转、俯仰、偏航角速度分开限幅。外环和点质量同一式，`kp/kd/a_max` 沿用场景卡（这八张卡都是 4 / 3 / 6）。深度相机仍只读水平偏航，倾角不进光线。

| 参数 | 本开关 | 来源与置信度 |
|---|---|---|
| `attitude_order` | 1 | 1。本票写明姿态环一阶滞后：ω = −e_R / τ_att，再做角速率限幅。E-SO3 已经量化过二阶（自然频率 1/τ、阻尼 1/√2）。本开关不重复那张表。 |
| `tau_att_s` | 0.12 s | 0.12 s，E-SO3 标称档（docs/research/e-so3-dynamics-gap.md）。票面假设，不是本仓库的 SITL 辨识，置信度低。PX4 当前 main（docs.px4.io/main，2026-11-13）MC_ROLL_P 与 MC_PITCH_P 默认 4.0，速率环若理想，一阶读数是 0.25 s。本开关不用 0.25 s，避免和已经用过的标称时间常数再换一套。 |
| `tau_thrust_s` | 0.06 s | 0.06 s，E-SO3 标称档的一阶比力滞后。票面假设，量级是小型多旋翼电机的几十毫秒，置信度低。PX4 没有单独的推力时间常数。 |
| `max_tilt_deg` | 35.0 ° | 35°，E-SO3 标称档。PX4 MPC_TILTMAX_AIR 默认 45°。本开关比文档默认更紧，置信度低。保持高度时水平加速度上限 g·tan(35°)≈6.87 m/s²，高于 a_max=6，所以纯水平 6 m/s² 不会被这只锥截住。 |
| 角速率 | 220 / 220 / 200 °/s | 滚转/俯仰 220 °/s，偏航 200 °/s。同一页 MC_ROLLRATE_MAX、MC_PITCHRATE_MAX、MC_YAWRATE_MAX 的默认。公开参数表，不是本机辨识，置信度中。 |
| `yaw_weight` | 0.4 | 0.4，PX4 MC_YAW_WEIGHT 默认（docs.px4.io/main，2026-11-13）。只乘在姿态误差的偏航分量上。公开参数表，不是本机辨识，置信度中。 |
| 推力幅值 | [0, g+a_max] | 比力限制在 [0, g + a_max]。这是 E-SO3 模型里的幅值帽，不是电机实测，置信度低。推力通道除此之外还有上面的一阶滞后。 |
| 外环 | kp=4.0，kd=3.0，a_max=6.0 | kp=4、kd=3、a_max=6 与 PointMassLag 相同，并优先沿用场景卡上的这三个数。PX4 的 MPC_XY_P / MPC_XY_VEL_P_ACC 量纲不同，本开关不把外环换成那一组，避免把增益差和滞后差混在一次对照里。 |
| `g` | 9.80665 | 9.80665 m/s²，PX4 CONSTANTS_ONE_G。常数，置信度高。 |

模型修订 `so3b-att1-e-so3-nominal-px4rate-2026-11-15`。跑数 commit `3fcd029ad1f0131fe6b058235fa434cb3a93cf59`。

## 和 SUMMARY 逐行对照

| 里程碑 | 场景 | 档 | 动力学 | 到达 | 碰撞 | 最小机间 m | 最小净空 m | 队形误差 p95 m | 到达时间 s | 冻结门 |
|---|---|---|---|---:|---:|---|---|---|---|---|
| M0 | g2-pillar | — | 点质量（SUMMARY） | 10/10 | 0 | — | 0.478 | — | 4.271（4.26–4.28，10/10 次有全体到达） | 通过 |
| M0 | g2-pillar | — | SO3 一阶（偏离上游） | 10/10 | 0 | — | 0.479 | — | 4.436（4.43–4.44，10/10 次有全体到达） | 通过 |
| M1 | formation-hold | — | 点质量（SUMMARY） | 9/9 | 0 | 1.987 | 离地 1.150 | 0.027 | 18.630（18.63–18.63，3/3 次有全体到达） | 通过 |
| M1 | formation-hold | — | SO3 一阶（偏离上游） | 9/9 | 0 | 1.984 | 离地 1.149 | 0.031 | 18.630（18.63–18.63，3/3 次有全体到达） | 通过 |
| M2 | crossing-pillars | — | 点质量（SUMMARY） | 30/30 | 0 | 3.350 | 0.528 | 4.130–4.610 | 7.270（7.05–7.77，10/10 次有全体到达） | 通过 |
| M2 | crossing-pillars | — | SO3 一阶（偏离上游） | 30/30 | 0 | 3.258 | 0.696 | 4.209–4.456 | 7.094（7.03–7.33，10/10 次有全体到达） | 通过 |
| M2 | crossing-forest（medium） | — | 点质量（SUMMARY） | 30/30 | 0 | 4.602 | 0.260 | 4.094–4.818 | 23.235（20.77–24.42，10/10 次有全体到达） | 通过 |
| M2 | crossing-forest（medium） | — | SO3 一阶（偏离上游） | 30/30 | 0 | 3.833 | 0.224 | 4.049–4.822 | 23.885（22.19–25.17，10/10 次有全体到达） | 通过 |
| M3 | formation-switch | — | 点质量（SUMMARY） | 30/30 | 0 | 1.536（过渡 1.536） | 0.693 | 前 0.025 / 后 0.025（全程 0.036） | 27.000（27.00–27.00，10/10 次有全体到达） | 通过 |
| M3 | formation-switch | — | SO3 一阶（偏离上游） | 30/30 | 0 | 1.537（过渡 1.537） | 0.691 | 前 0.029 / 后 0.030（全程 0.042） | 27.000（27.00–27.00，10/10 次有全体到达） | 通过 |
| M4 | formation-6-open | none | 点质量（SUMMARY） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | none | SO3 一阶（偏离上游） | 30/30 | 0 | 2.975 | 离地 1.143 | 0.077 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | drop-5 | 点质量（SUMMARY） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | drop-5 | SO3 一阶（偏离上游） | 30/30 | 0 | 2.975 | 离地 1.143 | 0.077 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | drop-20 | 点质量（SUMMARY） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | drop-20 | SO3 一阶（偏离上游） | 30/30 | 0 | 2.975 | 离地 1.143 | 0.077 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | duplicate-10 | 点质量（SUMMARY） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | duplicate-10 | SO3 一阶（偏离上游） | 30/30 | 0 | 2.975 | 离地 1.143 | 0.077 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | reorder-delay | 点质量（SUMMARY） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | reorder-delay | SO3 一阶（偏离上游） | 30/30 | 0 | 2.975 | 离地 1.143 | 0.077 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | combined | 点质量（SUMMARY） | 30/30 | 0 | 2.980 | 离地 1.150 | 0.066 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | formation-6-open | combined | SO3 一阶（偏离上游） | 30/30 | 0 | 2.975 | 离地 1.143 | 0.077 | 20.130（20.13–20.13，5/5 次有全体到达） | 通过 |
| M4 | crossing-6-forest（medium） | — | 点质量（SUMMARY） | 60/60 | 0 | 1.111 | 0.126 | 23.041–26.298 | 28.607（26.00–33.57，10/10 次有全体到达） | 通过 |
| M4 | crossing-6-forest（medium） | — | SO3 一阶（偏离上游） | 60/60 | 0 | 0.941 | 0.059 | 23.553–25.987 | 31.085（24.81–35.81，10/10 次有全体到达） | 通过 |
| M4 | crossing-6-forest-sparse（sparse） | — | 点质量（SUMMARY） | 60/60 | 0 | 0.959 | 0.158 | 18.725–22.922 | 26.741（23.24–31.58，10/10 次有全体到达） | 通过 |
| M4 | crossing-6-forest-sparse（sparse） | — | SO3 一阶（偏离上游） | 59/60 | 0 | 0.949 | 0.166 | 18.174–22.952 | 28.057（24.96–32.28，9/10 次有全体到达） | 破：arrival_rate |

## 哪些冻结门会破

下面这些行的冻结门在 SO3 一阶下不成立。点质量在 SUMMARY 里这些门是过的。

### M4 crossing-6-forest-sparse（sparse） / —

门槛：`{"arrival_rate": 1.0, "collisions": 0.0, "min_inter_drone_distance_m": 0.7}`。

- crossing-6-forest-sparse：seed 7 arrival_rate 0.833 < 1.000
- crossing-6-forest-sparse：合计到达率 arrival_rate 0.983 < 1.000

仍然成立的冻结行：g2-pillar/—、formation-hold/—、crossing-pillars/—、crossing-forest（medium）/—、formation-switch/—、formation-6-open/none、formation-6-open/drop-5、formation-6-open/drop-20、formation-6-open/duplicate-10、formation-6-open/reorder-delay、formation-6-open/combined、crossing-6-forest（medium）/—。

## 结论

冻结门破了 1 行，其余 12 行仍过。M4 crossing-6-forest-sparse（sparse） 破了 arrival_rate：到达 59/60（点质量 60/60），碰撞 0（点质量 0）。SO3 碰撞合计 0。
三机中等林最小净空从 0.260 m 到 0.224 m，六机中等林从 0.126 m 到 0.059 m。这两项在对应卡上只报数，不进冻结门。六机稀疏没到齐的那一次碰撞仍是 0；这一档队形 p95 的下沿盖过了这次没走完的轨迹，不能读成队形变好。G2 到达时间从 4.271 s 到 4.436 s，仍低于 5.0 s 的门。这些数只报数，不改默认动力学，也不改已冻门。


## 分种子

| 场景 | 档 | seed | 结果 | 原因 | 到达 | 碰撞 | 最小净空 m | 最小机间 m | 队形 p95 m | 到达时间 s | 仿真 s |
|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|---:|
| g2-pillar | — | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.479 | — | — | 4.44 | 5.44 |
| g2-pillar | — | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.479 | — | — | 4.44 | 5.44 |
| g2-pillar | — | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.480 | — | — | 4.44 | 5.44 |
| g2-pillar | — | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.479 | — | — | 4.44 | 5.44 |
| g2-pillar | — | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.480 | — | — | 4.44 | 5.44 |
| g2-pillar | — | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.484 | — | — | 4.43 | 5.43 |
| g2-pillar | — | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.484 | — | — | 4.43 | 5.43 |
| g2-pillar | — | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.484 | — | — | 4.43 | 5.43 |
| g2-pillar | — | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.484 | — | — | 4.43 | 5.43 |
| g2-pillar | — | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.480 | — | — | 4.44 | 5.44 |
| formation-hold | — | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 1.149 | 1.984 | 0.031 | 18.63 | 19.63 |
| formation-hold | — | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 1.149 | 1.984 | 0.031 | 18.63 | 19.63 |
| formation-hold | — | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 1.149 | 1.984 | 0.031 | 18.63 | 19.63 |
| crossing-pillars | — | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 1.025 | 7.274 | 4.402 | 7.03 | 8.03 |
| crossing-pillars | — | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.696 | 7.353 | 4.211 | 7.03 | 8.03 |
| crossing-pillars | — | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 1.046 | 7.253 | 4.209 | 7.03 | 8.03 |
| crossing-pillars | — | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 1.016 | 3.588 | 4.419 | 7.10 | 8.10 |
| crossing-pillars | — | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.995 | 3.610 | 4.413 | 7.09 | 8.09 |
| crossing-pillars | — | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 1.010 | 3.258 | 4.456 | 7.10 | 8.10 |
| crossing-pillars | — | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 1.021 | 3.615 | 4.450 | 7.10 | 8.10 |
| crossing-pillars | — | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.988 | 3.571 | 4.402 | 7.10 | 8.10 |
| crossing-pillars | — | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 1.048 | 7.454 | 4.209 | 7.03 | 8.03 |
| crossing-pillars | — | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.984 | 7.432 | 4.417 | 7.33 | 8.33 |
| crossing-forest | — | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.299 | 4.289 | 4.235 | 22.51 | 23.51 |
| crossing-forest | — | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.329 | 7.295 | 4.292 | 24.35 | 25.35 |
| crossing-forest | — | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.372 | 3.833 | 4.822 | 24.08 | 25.08 |
| crossing-forest | — | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.386 | 7.137 | 4.725 | 25.14 | 26.14 |
| crossing-forest | — | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.309 | 4.015 | 4.526 | 25.13 | 26.13 |
| crossing-forest | — | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.336 | 4.069 | 4.481 | 23.85 | 24.85 |
| crossing-forest | — | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.310 | 4.417 | 4.049 | 25.17 | 26.17 |
| crossing-forest | — | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.224 | 7.288 | 4.125 | 24.06 | 25.06 |
| crossing-forest | — | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.337 | 7.358 | 4.430 | 22.37 | 23.37 |
| crossing-forest | — | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.373 | 3.975 | 4.106 | 22.19 | 23.19 |
| formation-switch | — | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.691 | 1.537（过渡 1.537） | 前 0.029 / 后 0.030（全程 0.042） | 27.00 | 28.00 |
| formation-switch | — | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.691 | 1.537（过渡 1.537） | 前 0.029 / 后 0.030（全程 0.042） | 27.00 | 28.00 |
| formation-switch | — | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.691 | 1.537（过渡 1.537） | 前 0.029 / 后 0.030（全程 0.042） | 27.00 | 28.00 |
| formation-switch | — | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.691 | 1.537（过渡 1.537） | 前 0.029 / 后 0.030（全程 0.042） | 27.00 | 28.00 |
| formation-switch | — | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.691 | 1.537（过渡 1.537） | 前 0.029 / 后 0.030（全程 0.042） | 27.00 | 28.00 |
| formation-switch | — | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.691 | 1.537（过渡 1.537） | 前 0.029 / 后 0.030（全程 0.042） | 27.00 | 28.00 |
| formation-switch | — | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.691 | 1.537（过渡 1.537） | 前 0.029 / 后 0.030（全程 0.042） | 27.00 | 28.00 |
| formation-switch | — | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.691 | 1.537（过渡 1.537） | 前 0.029 / 后 0.030（全程 0.042） | 27.00 | 28.00 |
| formation-switch | — | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.691 | 1.537（过渡 1.537） | 前 0.029 / 后 0.030（全程 0.042） | 27.00 | 28.00 |
| formation-switch | — | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.691 | 1.537（过渡 1.537） | 前 0.029 / 后 0.030（全程 0.042） | 27.00 | 28.00 |
| formation-6-open | none | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | none | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | none | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | none | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | none | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | drop-5 | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | drop-5 | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | drop-5 | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | drop-5 | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | drop-5 | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | drop-20 | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | drop-20 | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | drop-20 | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | drop-20 | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | drop-20 | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | duplicate-10 | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | duplicate-10 | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | duplicate-10 | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | duplicate-10 | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | duplicate-10 | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | reorder-delay | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | reorder-delay | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | reorder-delay | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | reorder-delay | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | reorder-delay | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | combined | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | combined | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | combined | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | combined | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| formation-6-open | combined | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 1.143 | 2.975 | 0.077 | 20.13 | 21.13 |
| crossing-6-forest | — | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.217 | 1.177 | 23.638 | 29.30 | 30.30 |
| crossing-6-forest | — | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.102 | 1.135 | 23.708 | 35.81 | 36.81 |
| crossing-6-forest | — | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.122 | 1.031 | 23.553 | 35.04 | 36.04 |
| crossing-6-forest | — | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.059 | 1.039 | 23.799 | 33.56 | 34.56 |
| crossing-6-forest | — | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.173 | 1.099 | 24.812 | 29.96 | 30.96 |
| crossing-6-forest | — | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.130 | 0.978 | 23.802 | 24.81 | 25.81 |
| crossing-6-forest | — | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.199 | 1.161 | 24.875 | 27.98 | 28.98 |
| crossing-6-forest | — | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.180 | 1.144 | 25.987 | 33.33 | 34.33 |
| crossing-6-forest | — | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.136 | 0.987 | 23.576 | 30.49 | 31.49 |
| crossing-6-forest | — | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.169 | 0.941 | 23.994 | 30.57 | 31.57 |
| crossing-6-forest-sparse | — | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.216 | 1.072 | 22.952 | 29.26 | 30.26 |
| crossing-6-forest-sparse | — | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.204 | 1.059 | 19.199 | 29.27 | 30.27 |
| crossing-6-forest-sparse | — | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.218 | 1.002 | 20.603 | 32.28 | 33.28 |
| crossing-6-forest-sparse | — | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.166 | 1.051 | 19.680 | 27.53 | 28.53 |
| crossing-6-forest-sparse | — | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.296 | 0.949 | 22.866 | 26.08 | 27.08 |
| crossing-6-forest-sparse | — | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.306 | 0.971 | 21.635 | 27.81 | 28.81 |
| crossing-6-forest-sparse | — | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.360 | 1.043 | 20.410 | 29.02 | 30.02 |
| crossing-6-forest-sparse | — | 7 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0.346 | 1.061 | 18.174 | — | 50.00 |
| crossing-6-forest-sparse | — | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.179 | 1.067 | 22.076 | 26.30 | 27.30 |
| crossing-6-forest-sparse | — | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.515 | 1.047 | 19.533 | 24.96 | 25.96 |

## 复现

```
.venv/bin/python scripts/e-so3b-suite.py --jobs 2
.venv/bin/python -m pytest tests/test_e_so3b_suite.py -q
PYTHONPATH=.:core/src .venv/bin/python -m pytest -q tests
```

缺省从 `out/e-so3b-suite/partial/` 续跑。`--no-resume` 忽略已有 partial。模型或种子清单变了，指纹对不上的 partial 不会被拿来用。`--jobs` 可以降到 1。
