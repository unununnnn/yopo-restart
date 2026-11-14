# 票 E-SO3：L0 SO3 四旋翼滞后 vs 点质量，只报数

只报数。report-only。SO3 两臂是偏离默认动力学（L0 点质量不变）。不改 yopo_l0/ 的默认动力学，不改 core/、scenes/、docs/tickets.md、README.md、scripts/premerge.sh、scripts/regress.sh。规划与策略保持场景卡默认：upstream（柱群和穿林由 CrossingFleet 走上游 YOPO），影子盾开。外环 kp/kd/a_max 与 PointMassLag 相同。τ_att、τ_thrust 和倾角是票面给定的假设，不是本仓库的 SITL 辨识。未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。

依据是 [SPEC](../SPEC.md) 第 47 行（动力学可换，SO3 后加）和第 73 行（接口是「给定当前状态和批准计划，推进 dt」；首版点质量加一阶加速度滞后），以及 [已知限制](../LIMITATIONS.md) 第 102 行：默认仍是点质量、`a_max=6`，换模型之后到达和净空要重测。

数字在 [e-so3-dynamics-gap-runs.json](e-so3-dynamics-gap-runs.json)。跑数 commit 是 `15d9ef7ee32a24264b2f4fb593ff867bf6c9eaae`。

## 模型

外环和 `PointMassLag` 用同一组数：`kp=4`、`kd=3`、`a_max=6`。
PX4 当前 main 参数表（docs.px4.io/main，2026-11-13）里，`MPC_XY_P` 默认 0.95（米/秒每米），`MPC_XY_VEL_P_ACC` 默认 1.8（米/秒²每米/秒），`MPC_ACC_HOR` 默认 3 m/s²，`MPC_ACC_HOR_MAX` 默认 5 m/s²。
这些数和 L0 的加速度外环不是同一个量纲。本票不把外环换成 PX4 默认，避免把增益差和滞后差混在一次对照里。

标称档：`tau_att_s=0.12`，`tau_thrust_s=0.06`，`max_tilt_deg=35`，`a_max=6`，姿态阶数 2。
慢姿态档：`tau_att_s=0.22`，`tau_thrust_s=0.1`，`max_tilt_deg=30`，`a_max=6`，姿态阶数 2。

姿态是二阶环：自然频率 ω_n = 1/τ_att，阻尼 ζ = 1/√2。
速率命令增益 k_R = 1/(τ_att √2)，速率再滞后 τ_att/√2。
同一页参数表里 `MC_ROLL_P` 和 `MC_PITCH_P` 的默认是 4.0（弧度/秒每弧度）。
速率环若理想，一阶读数是 1/4.0 = 0.25 s。旧版参数表常见 6.5，对应约 0.154 s。
标称 0.12 s（ω_n ≈ 8.33 rad/s）比这两个倒数都快，是票面给定的小飞机标称，不是把某一个 PX4 参数取倒数。
慢档 0.22 s（ω_n ≈ 4.55 rad/s）靠近当前默认 0.25 s 的一阶读数。
PX4 没有单独的推力时间常数。0.06 s 和 0.10 s 是票面给定的一阶比力滞后，量级是小型多旋翼电机的几十毫秒。本仓库没有做 SITL 辨识。

`MPC_TILTMAX_AIR` 默认 45°。两档都更紧。
保持高度时，35° 的水平加速度上限是 g·tan(35°) = 6.87 m/s²，高于 6。
30° 是 5.66 m/s²，低于 6。
水平加速度 6 m/s²、垂直加速度 0 需要的倾角约 31.46°。
所以标称档在 a_max=6 的水平命令上不会被倾角锥截住，慢档会被截到大约 5.7 m/s²。

角速度限幅用同一页的默认：滚转/俯仰 220 °/s，偏航 200 °/s。
`MC_YAW_WEIGHT` 默认 0.4，两档 SO3 都乘在姿态误差的偏航分量上。点质量没有这个状态，它每个物理步把偏航设成计划值。
重力取 9.80665 m/s²。几何跟踪用 Lee, Leok, McClamroch, CDC 2010 的 SO(3) 姿态误差。
平动是 a = T b_3 − g e_z。L0 深度相机只读 `State.yaw`，倾角不改变光线方向。

SO3 状态（旋转、角速度、比力）记在脚本进程里，按 `step` 返回的位置数组认飞机。不写入 `State` 的新字段，也不改 `yopo_l0/dynamics.py`。注册键是 `so3_lag`。

## 口径

到达率按架次：该档全部 seed 的 `arrived` 之和 / `n_drones` 之和。碰撞按运行次数。仿真在第一次碰撞停下，所以一次运行最多 1。括号里分开障碍和机间。

最小净空 = `min_obstacle_distance_m` − 机体半径 0.35 m。表里写该档各次的范围，左端是最紧的那一次。负值表示中心已经进了机体半径。

队形误差 p95 是运行器的 `formation_error_p95_m`。多个 seed 写出最小到最大。单机 G2 没有这个字段，写 —。

平均到达时间是该档里 `arrival_time_s` 非空的运行的算术平均。运行器在全体飞机都进入保持时才写这个字段，值是各机进入保持的时刻的最大值。有一机没进保持则为空，不进平均。括号里的分母是该档的运行次数。

差距表是 SO3 减去点质量。最紧净空差为负，表示 SO3 更贴障碍。队形 p95 差用的是各档最差种子。平均到达时间差为正，表示 SO3 更晚。

## 场景和臂

| 场景 | 文件 | 里程碑 | 种子 | 任务 |
|---|---|---|---|---|
| G2 柱子 | `scenes/g2-pillar.json` | M0 | 0–9 | 单机 goto |
| 三机柱群 | `scenes/crossing-pillars.json` | M2 | 0–4 | 柱群 |
| 三机中等林带 | `scenes/crossing-forest.json` | M2 | 0–4 | 中等林带 |
| 六机稀疏林带 | `scenes/crossing-6-forest-sparse.json` | M4 | 0–2 | 稀疏林带 |

三臂共用同一份场景文件。点质量臂不改 `dynamics`。SO3 只在进程内存里把 `dynamics.model` 换成 `so3_lag` 并写上该档参数。文件哈希见 JSON 的 `file_card_hash`。影子盾开，只记录。`safe_radius` 仍是 0.05 m，运行器的 `upstream_deviation` 保持 false。

| 臂 | 动力学 | 标记 |
|---|---|---|
| PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | `point_mass_lag`，τ=0.1 s | 默认动力学 |
| SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | `so3_lag`，τ_att=0.12 s，τ_thrust=0.06 s，倾角 35° | report-only / 偏离默认动力学 |
| SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | `so3_lag`，τ_att=0.22 s，τ_thrust=0.10 s，倾角 30° | report-only / 偏离默认动力学 |

## 汇总

| 场景 | 里程碑 | 臂 | 标记 | 到达率 | 碰撞 | 最小净空 m | 队形误差 p95 m | 平均到达时间 s |
|---|---|---|---|---:|---:|---|---|---:|
| G2 柱子 | M0 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 默认动力学 | 10/10 | 0（障碍 0，机间 0） | 0.478–0.479 | — | 4.27（10/10 次有全体到达） |
| G2 柱子 | M0 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | report-only / 偏离默认动力学 | 10/10 | 0（障碍 0，机间 0） | 0.618–0.643 | — | 4.59（10/10 次有全体到达） |
| G2 柱子 | M0 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | report-only / 偏离默认动力学 | 10/10 | 0（障碍 0，机间 0） | 0.845–0.858 | — | 11.18（10/10 次有全体到达） |
| 三机柱群 | M2 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 默认动力学 | 15/15 | 0（障碍 0，机间 0） | 0.662–1.011 | 4.139–4.610 | 7.20（5/5 次有全体到达） |
| 三机柱群 | M2 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | report-only / 偏离默认动力学 | 15/15 | 0（障碍 0，机间 0） | 0.371–1.001 | 4.415–4.757 | 7.07（5/5 次有全体到达） |
| 三机柱群 | M2 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | report-only / 偏离默认动力学 | 15/15 | 0（障碍 0，机间 0） | 0.628–0.914 | 4.217–4.559 | 17.02（5/5 次有全体到达） |
| 三机中等林带 | M2 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 默认动力学 | 15/15 | 0（障碍 0，机间 0） | 0.260–0.445 | 4.112–4.574 | 23.09（5/5 次有全体到达） |
| 三机中等林带 | M2 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | report-only / 偏离默认动力学 | 15/15 | 0（障碍 0，机间 0） | 0.223–0.497 | 4.516–5.093 | 23.91（5/5 次有全体到达） |
| 三机中等林带 | M2 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | report-only / 偏离默认动力学 | 12/15 | 1（障碍 1，机间 0） | -0.014 至 0.164 | 6.294–7.188 | 38.10（4/5 次有全体到达） |
| 六机稀疏林带 | M4 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 默认动力学 | 18/18 | 0（障碍 0，机间 0） | 0.178–0.322 | 18.725–20.976 | 26.32（3/3 次有全体到达） |
| 六机稀疏林带 | M4 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | report-only / 偏离默认动力学 | 12/18 | 1（障碍 1，机间 0） | -0.002 至 0.490 | 20.557–22.916 | 31.87（2/3 次有全体到达） |
| 六机稀疏林带 | M4 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | report-only / 偏离默认动力学 | 0/18 | 3（障碍 2，机间 1） | -0.009 至 0.153 | 11.634–14.040 | — |

## 相对点质量

| 场景 | SO3 臂 | 到达架次差 | 碰撞差 | 最紧净空差 m | 队形 p95 最差差 m | 平均到达时间差 s |
|---|---|---:|---:|---:|---:|---:|
| G2 柱子 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | +0 | +0 | +0.140 | — | +0.32 |
| G2 柱子 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | +0 | +0 | +0.367 | — | +6.90 |
| 三机柱群 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | +0 | +0 | -0.290 | +0.148 | -0.13 |
| 三机柱群 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | +0 | +0 | -0.034 | -0.051 | +9.82 |
| 三机中等林带 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | +0 | +0 | -0.036 | +0.519 | +0.83 |
| 三机中等林带 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | -3 | +1 | -0.274 | +2.614 | +15.01 |
| 六机稀疏林带 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | -6 | +1 | -0.180 | +1.940 | +5.55 |
| 六机稀疏林带 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | -18 | +3 | -0.187 | -6.936 | — |

## 结论

1. G2 柱子，标称档：到达 10/10（点质量 10/10），碰撞 0（点质量 0），最紧净空差 +0.140 m，队形 p95 最差差 不适用，平均到达时间差 +0.32 s。
2. G2 柱子，慢姿态档：到达 10/10（点质量 10/10），碰撞 0（点质量 0），最紧净空差 +0.367 m，队形 p95 最差差 不适用，平均到达时间差 +6.90 s。
3. 三机柱群，标称档：到达 15/15（点质量 15/15），碰撞 0（点质量 0），最紧净空差 -0.290 m，队形 p95 最差差 +0.148 m，平均到达时间差 -0.13 s。
4. 三机柱群，慢姿态档：到达 15/15（点质量 15/15），碰撞 0（点质量 0），最紧净空差 -0.034 m，队形 p95 最差差 -0.051 m，平均到达时间差 +9.82 s。
5. 三机中等林带，标称档：到达 15/15（点质量 15/15），碰撞 0（点质量 0），最紧净空差 -0.036 m，队形 p95 最差差 +0.519 m，平均到达时间差 +0.83 s。
6. 三机中等林带，慢姿态档：到达 12/15（点质量 15/15），碰撞 1（点质量 0），最紧净空差 -0.274 m，队形 p95 最差差 +2.614 m，平均到达时间差 +15.01 s。
7. 六机稀疏林带，标称档：到达 12/18（点质量 18/18），碰撞 1（点质量 0），最紧净空差 -0.180 m，队形 p95 最差差 +1.940 m，平均到达时间差 +5.55 s。
8. 六机稀疏林带，慢姿态档：到达 0/18（点质量 18/18），碰撞 3（点质量 0），最紧净空差 -0.187 m，队形 p95 最差差 -6.936 m，平均到达时间差 不适用。

点质量在这些格子上更乐观：三机中等林带 / 慢姿态档 少 3 架次；六机稀疏林带 / 标称档 少 6 架次；六机稀疏林带 / 慢姿态档 少 18 架次；三机中等林带 / 慢姿态档 多 1 次碰撞；六机稀疏林带 / 标称档 多 1 次碰撞；六机稀疏林带 / 慢姿态档 多 3 次碰撞。
标称档（τ_att=0.12 s，倾角 35°）在 G2 和两张三机卡上到达架次与点质量相同，碰撞仍是 0。G2 到达 10/10，碰撞 0，最紧净空差 +0.140 m，平均到达时间差 +0.32 s。三机柱群 到达 15/15，碰撞 0，最紧净空差 -0.290 m，平均到达时间差 -0.13 s。柱群这一格是标称档把净空收得最紧的地方。三机中等林带 到达 15/15，碰撞 0，最紧净空差 -0.036 m，平均到达时间差 +0.83 s。六机稀疏林带 到达 12/18，碰撞 1，最紧净空差 -0.180 m，平均到达时间差 +5.55 s。
慢姿态档（τ_att=0.22 s，倾角 30°）在 G2 和三机柱群上仍然全部到达、碰撞 0，到达时间拉长。G2 到达 10/10，碰撞 0，最紧净空差 +0.367 m，平均到达时间差 +6.90 s。三机柱群 到达 15/15，碰撞 0，最紧净空差 -0.034 m，平均到达时间差 +9.82 s。三机中等林带 到达 12/15，碰撞 1，最紧净空差 -0.274 m，平均到达时间差 +15.01 s。六机稀疏林带 到达 0/18，碰撞 3，最紧净空差 -0.187 m，平均到达时间差 不适用。
慢姿态档六机的队形 p95 最差差是 -6.936 m。这几轮在碰撞时停下，p95 只覆盖撞上之前的短轨迹，不能读成队形变好。
合起来：标称档不改单机和三机的到达率；三机柱群的最紧净空被点质量算松，六机稀疏少到达并多出碰撞。慢姿态档把到达时间拉长，并在三机林和六机稀疏上撞出来。点质量对单机绕柱和三机到达率不算明显乐观，对六机稀疏和慢姿态穿林是乐观的。这些数只报数，不改默认动力学，也不改已冻门。冻结表里的秒数和米数不能直接当成 SO3 的结果。
深度相机仍只吃水平偏航，倾角不进光线。上面的净空和到达差来自闭环里的位置、速度和偏航滞后。

## 分种子

| 场景 | 臂 | seed | 结果 | 原因 | 到达 | 碰撞 | 最小净空 m | 队形 p95 m | 到达时间 s | 仿真 s |
|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|
| G2 柱子 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.479 | — | 4.26 | 5.26 |
| G2 柱子 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.479 | — | 4.27 | 5.27 |
| G2 柱子 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.479 | — | 4.26 | 5.26 |
| G2 柱子 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.479 | — | 4.26 | 5.26 |
| G2 柱子 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.479 | — | 4.27 | 5.27 |
| G2 柱子 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.478 | — | 4.28 | 5.28 |
| G2 柱子 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.478 | — | 4.28 | 5.28 |
| G2 柱子 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.478 | — | 4.28 | 5.28 |
| G2 柱子 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.478 | — | 4.28 | 5.28 |
| G2 柱子 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.479 | — | 4.27 | 5.27 |
| G2 柱子 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.618 | — | 4.59 | 5.59 |
| G2 柱子 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.618 | — | 4.59 | 5.59 |
| G2 柱子 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.618 | — | 4.59 | 5.59 |
| G2 柱子 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.618 | — | 4.59 | 5.59 |
| G2 柱子 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.618 | — | 4.59 | 5.59 |
| G2 柱子 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.643 | — | 4.59 | 5.59 |
| G2 柱子 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.643 | — | 4.59 | 5.59 |
| G2 柱子 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.643 | — | 4.59 | 5.59 |
| G2 柱子 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.643 | — | 4.59 | 5.59 |
| G2 柱子 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.618 | — | 4.59 | 5.59 |
| G2 柱子 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.846 | — | 13.34 | 14.34 |
| G2 柱子 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.845 | — | 13.34 | 14.34 |
| G2 柱子 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.845 | — | 13.33 | 14.33 |
| G2 柱子 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.845 | — | 13.33 | 14.33 |
| G2 柱子 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.846 | — | 13.34 | 14.34 |
| G2 柱子 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.858 | — | 8.85 | 9.85 |
| G2 柱子 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.857 | — | 8.84 | 9.84 |
| G2 柱子 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.858 | — | 8.85 | 9.85 |
| G2 柱子 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.858 | — | 8.84 | 9.84 |
| G2 柱子 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0.845 | — | 9.69 | 10.69 |
| 三机柱群 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.662 | 4.610 | 7.06 | 8.06 |
| 三机柱群 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.666 | 4.342 | 7.06 | 8.06 |
| 三机柱群 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 1.011 | 4.471 | 7.06 | 8.06 |
| 三机柱群 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.915 | 4.139 | 7.76 | 8.76 |
| 三机柱群 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.914 | 4.205 | 7.05 | 8.05 |
| 三机柱群 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.413 | 4.436 | 6.93 | 7.93 |
| 三机柱群 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.697 | 4.676 | 6.93 | 7.93 |
| 三机柱群 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.795 | 4.757 | 6.92 | 7.92 |
| 三机柱群 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 1.001 | 4.477 | 6.98 | 7.98 |
| 三机柱群 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.371 | 4.415 | 7.57 | 8.57 |
| 三机柱群 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.708 | 4.217 | 17.32 | 18.32 |
| 三机柱群 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.628 | 4.336 | 17.33 | 18.33 |
| 三机柱群 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.737 | 4.559 | 16.19 | 17.19 |
| 三机柱群 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.914 | 4.431 | 17.22 | 18.22 |
| 三机柱群 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.843 | 4.328 | 17.03 | 18.03 |
| 三机中等林带 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.316 | 4.466 | 23.32 | 24.32 |
| 三机中等林带 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.265 | 4.129 | 23.59 | 24.59 |
| 三机中等林带 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.260 | 4.112 | 20.77 | 21.77 |
| 三机中等林带 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.445 | 4.236 | 23.63 | 24.63 |
| 三机中等林带 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.304 | 4.574 | 24.12 | 25.12 |
| 三机中等林带 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.450 | 4.516 | 22.09 | 23.09 |
| 三机中等林带 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.252 | 5.093 | 25.74 | 26.74 |
| 三机中等林带 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.497 | 4.593 | 22.86 | 23.86 |
| 三机中等林带 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.414 | 4.795 | 25.57 | 26.57 |
| 三机中等林带 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.223 | 4.626 | 23.31 | 24.31 |
| 三机中等林带 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.159 | 6.734 | 39.55 | 40.55 |
| 三机中等林带 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.143 | 6.294 | 37.05 | 38.05 |
| 三机中等林带 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.157 | 6.679 | 37.76 | 38.76 |
| 三机中等林带 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0.164 | 6.967 | 38.03 | 39.03 |
| 三机中等林带 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 4 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 1 | -0.014 | 7.188 | — | 13.19 |
| 六机稀疏林带 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.315 | 20.976 | 25.40 | 26.40 |
| 六机稀疏林带 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.178 | 18.725 | 23.24 | 24.24 |
| 六机稀疏林带 | PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.322 | 20.661 | 30.31 | 31.31 |
| 六机稀疏林带 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.137 | 20.557 | 29.06 | 30.06 |
| 六机稀疏林带 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0.490 | 20.806 | 34.67 | 35.67 |
| 六机稀疏林带 | SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变） | 2 | COLLISION | `uav2:TRUTH_POSITION` | 0/6 | 1 | -0.002 | 22.916 | — | 17.99 |
| 六机稀疏林带 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 0 | COLLISION | `uav3:TRUTH_POSITION` | 0/6 | 1 | -0.009 | 14.040 | — | 8.63 |
| 六机稀疏林带 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 1 | COLLISION | `uav3-uav5:INTER_DRONE` | 0/6 | 1 | 0.153 | 12.235 | — | 7.76 |
| 六机稀疏林带 | SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变） | 2 | COLLISION | `uav3:TRUTH_POSITION` | 0/6 | 1 | -0.001 | 11.634 | — | 6.91 |

## 复现

```
.venv/bin/python scripts/e-so3-dynamics-gap.py --jobs 4
.venv/bin/python -m pytest tests/test_e_so3_dynamics_gap.py -q
python3 scripts/check_doc_links.py
```

`--partial` 从 `out/e-so3-dynamics-gap/partial/` 续跑已经完成的种子。缺省也续跑。`--no-resume` 忽略已有 partial。模型方程或臂参数变了，指纹对不上的 partial 不会被拿来用。
