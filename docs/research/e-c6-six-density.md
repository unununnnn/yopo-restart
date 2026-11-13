# 票 E-C6：六机密度场 0.14，只报数

只报数。report-only。没有改 `yopo_l0/` 的默认、`core/`、已冻门、`scenes/`、`docs/tickets.md`、README 和 `scripts/regress.sh`。`safe_radius` 仍是 0.05 m，`empty_brake` 仍是 false，`track_clearance` 仍是 false，机间过滤仍是关。打开 E-A1、`empty_brake` 或 `track_clearance` 的臂标「偏离上游」。0.08 m 是假设，用来盖 [E-C5](e-c5-corridor-radius.md) 记下的 0.058–0.073 m 跟踪差。未冻结，不进 `scenes/suite.txt`，不设 gates，不改已冻门。

数字在 [e-c6-six-density-runs.json](e-c6-six-density-runs.json)。跑数 commit 是 `030adc893b9db03e50e647a157869011979c9e93`。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。

## 口径

avg_speed_mps 是该次运行全部 tick、全部飞机的 ||v|| 里，有限且严格大于 0 的样本的算术平均。悬停和初值 0 不算有效飞行。没有有效样本时为 null。 汇总表里的平均速度是该档各次 `avg_speed_mps` 的算术平均。

`plan_wall_ms` 是运行器记下的网络前向墙钟，不含解码和打分之后的时间。汇总表的均值 / p95 是各次运行自己的 mean / p95 再取算术平均，单位毫秒。它是这台机器上的墙钟，不进门。

到达按架次。seeds 0–4、每场景 6 架，一档的分母是 30。撞柱、撞地、撞箱、机间碰撞按运行器，仿真在第一次碰撞停下，所以一次运行最多记 1。撞柱是 `TRUTH_POSITION` 且碰撞时刻最近的障碍是圆柱。撞地是最近的为地面。撞箱是最近的为盒子（林带的墙或枝）。机间是 `INTER_DRONE`。

最小净空 = `min_obstacle_distance_m` − 0.35 m，是机体中心到最近表面的距离再减去机体半径。最小机间是中心距。

`CORRIDOR_EMPTY_HOVER` 是 `plan_codes` 里这个码的次数。014 对向在 `empty_brake` 打开时，这个码就是从参考头刹停的规划次数；全关时它是上游原地悬停。六机林带里它是走廊为空的次数，随后的 `BRAKE_ONLY` / `CONFLICT_BRAKE` 不计入这一列。

## 结论

运行器字段 upstream_deviation：六机林带 / off = false；六机林带 / base = false；六机林带 / geom = false；六机林带 / geom_multi = false；014 六机对向 / off = false；014 六机对向 / base = true；014 六机对向 / geom = true；014 六机对向 / geom_multi = true。

六机林带是 crossing。CrossingFleet 只读 E-A1，不读 `empty_brake` 和 `track_clearance`，所以运行器的 `upstream_deviation` 在四档都保持 false（半径仍是 0.05 m）。014 六机对向是 goto，base / geom / geom_multi 的运行器标记为 true。臂上的「偏离上游」看开关有没有打开，不看这个运行器标记。

穿林三档偏离臂的 E-A1 都是阈值 1.0 m、惩罚 0。`empty_brake` 和 `track_clearance` 写在进程内卡上，CrossingFleet 不读。`plan_wall_ms` 仍是各次前向的墙钟，不要求逐毫秒相同。

六机林带的 base、geom、geom_multi 五个种子位置哈希相同。相对全关，只有 seed 4 的轨迹不同。
014 六机对向的 geom 与 geom_multi 五个种子位置哈希相同，并且与全关、base 都不同。

| 场景 | 臂 | 偏离上游 | 到达 | 撞柱 | 撞地 | 撞箱 | 机间 | 超时 | 平均速度 m/s | plan_wall 均值 ms | plan_wall p95 ms | 最小净空 m | 最小机间 m | CORRIDOR_EMPTY_HOVER |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---:|
| 六机林带 | 默认上游 + 影子盾 | 否 | 30/30 | 0 | 0 | 0 | 0 | 0 | 1.716 | 12.58 | 15.88 | 0.161–0.306 | 1.111–1.392 | 0 |
| 六机林带 | E-A1 1.0 m + empty_brake（偏离上游） | 偏离上游 | 30/30 | 0 | 0 | 0 | 0 | 0 | 1.765 | 13.17 | 21.90 | 0.161–0.306 | 1.037–1.392 | 0 |
| 六机林带 | base + track geom 0.08 m（偏离上游） | 偏离上游 | 30/30 | 0 | 0 | 0 | 0 | 0 | 1.765 | 13.48 | 27.68 | 0.161–0.306 | 1.037–1.392 | 0 |
| 六机林带 | base + track geom_multi 0.08 m（偏离上游） | 偏离上游 | 30/30 | 0 | 0 | 0 | 0 | 0 | 1.765 | 11.93 | 16.86 | 0.161–0.306 | 1.037–1.392 | 0 |
| 014 六机对向 | 默认上游 + 影子盾 | 否 | 0/30 | 0 | 0 | 0 | 5 | 0 | 3.314 | 11.00 | 12.51 | 0.145–0.202 | 0.660–0.688 | 0 |
| 014 六机对向 | E-A1 1.0 m + empty_brake（偏离上游） | 偏离上游 | 0/30 | 5 | 0 | 0 | 0 | 0 | 3.273 | 9.16 | 10.41 | -0.030 至 -0.005 | 1.152–4.323 | 4 |
| 014 六机对向 | base + track geom 0.08 m（偏离上游） | 偏离上游 | 15/30 | 2 | 0 | 0 | 0 | 3 | 1.580 | 10.52 | 11.76 | -0.007 至 0.137 | 0.966–2.230 | 1109 |
| 014 六机对向 | base + track geom_multi 0.08 m（偏离上游） | 偏离上游 | 15/30 | 2 | 0 | 0 | 0 | 3 | 1.580 | 10.36 | 11.59 | -0.007 至 0.137 | 0.966–2.230 | 1109 |

## 场景和臂

| 场景 | 来源 | 任务 | 机数 | 种子 |
|---|---|---|---:|---|
| 六机林带 | `scenes/crossing-6-forest.json` | crossing | 6 | 0–4 |
| 014 六机对向 | 进程内存，几何 `scenes/ego-density-014.json` | goto，40 s | 6 | 0–4 |

几何 `scenes/ego-density-014.json` 的 sha256 是 `5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9`。柱子 56 根，目标密度 0.14 个/m²。
六机对向的名义圆半径 8.0 m，高度 1.5 m，方位间隔 60°，目标是起点的对径。轴距低于 0.70 m 时沿射线推到 20 m 正方形外，规则与 [E-C4](e-c4-density-l0.md) 的三机对向相同。

| 飞机 | 半径 m | 推出正方形 | 起点轴距 m | 目标轴距 m | 起点 | 目标 |
|---|---:|---|---:|---:|---|---|
| uav0 | 8.000 | 否 | 2.874 | 1.016 | 8.000, 0.000, 1.500 | -8.000, 0.000, 1.500 |
| uav1 | 8.000 | 否 | 1.851 | 1.434 | 4.000, 6.928, 1.500 | -4.000, -6.928, 1.500 |
| uav2 | 11.597 | 是 | 2.173 | 1.863 | -5.799, 10.043, 1.500 | 5.799, -10.043, 1.500 |
| uav3 | 8.000 | 否 | 1.016 | 2.874 | -8.000, 0.000, 1.500 | 8.000, 0.000, 1.500 |
| uav4 | 8.000 | 否 | 1.434 | 1.851 | -4.000, -6.928, 1.500 | 4.000, 6.928, 1.500 |
| uav5 | 11.597 | 是 | 1.863 | 2.173 | 5.799, -10.043, 1.500 | -5.799, 10.043, 1.500 |

| 臂 | 进程里多出来的 | 偏离上游 | 谁真的读这些开关 |
|---|---|---|---|
| 默认上游 + 影子盾 | 没有。不写 `policy_params`。影子盾开，只记录 | 否 | — |
| E-A1 1.0 m + empty_brake（偏离上游） | `inter_drone_filter`，阈值 1.0 m，惩罚 0；`empty_brake=true` | 偏离上游 | 林带只读 E-A1。对向两样都读 |
| base + track geom 0.08 m（偏离上游） | 再加上 `track_clearance=true`，`mode=geom`，`track_margin_m=0.08` | 偏离上游 | 林带仍只读 E-A1。对向三样都读 |
| base + track geom_multi 0.08 m（偏离上游） | `mode=geom_multi`。只在本帧有邻机时做几何门 | 偏离上游 | 林带仍只读 E-A1。对向三样都读 |

`safe_radius` 四档都不覆盖，运行器缺省 0.05 m。参数不写进场景文件。

## 分种子

| 场景 | 臂 | seed | 结果 | 原因 | 到达 | 撞柱 | 撞地 | 机间 | 平均速度 m/s | plan 均值 ms | plan p95 ms | 最小净空 m | 最小机间 m | CORRIDOR_EMPTY_HOVER | 仿真 s |
|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 六机林带 | 默认上游 + 影子盾 | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.870 | 11.92 | 13.27 | 0.306 | 1.293 | 0 | 27.08 |
| 六机林带 | 默认上游 + 影子盾 | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.651 | 11.83 | 13.83 | 0.161 | 1.392 | 0 | 29.13 |
| 六机林带 | 默认上游 + 影子盾 | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.731 | 12.00 | 12.83 | 0.202 | 1.148 | 0 | 27.90 |
| 六机林带 | 默认上游 + 影子盾 | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.670 | 14.27 | 16.55 | 0.239 | 1.189 | 0 | 30.08 |
| 六机林带 | 默认上游 + 影子盾 | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.658 | 12.91 | 22.93 | 0.194 | 1.111 | 0 | 29.40 |
| 六机林带 | E-A1 1.0 m + empty_brake（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.870 | 12.99 | 17.58 | 0.306 | 1.293 | 0 | 27.08 |
| 六机林带 | E-A1 1.0 m + empty_brake（偏离上游） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.651 | 12.96 | 17.88 | 0.161 | 1.392 | 0 | 29.13 |
| 六机林带 | E-A1 1.0 m + empty_brake（偏离上游） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.731 | 12.59 | 17.99 | 0.202 | 1.148 | 0 | 27.90 |
| 六机林带 | E-A1 1.0 m + empty_brake（偏离上游） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.670 | 13.56 | 27.38 | 0.239 | 1.189 | 0 | 30.08 |
| 六机林带 | E-A1 1.0 m + empty_brake（偏离上游） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.902 | 13.72 | 28.70 | 0.194 | 1.037 | 0 | 25.78 |
| 六机林带 | base + track geom 0.08 m（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.870 | 12.50 | 21.10 | 0.306 | 1.293 | 0 | 27.08 |
| 六机林带 | base + track geom 0.08 m（偏离上游） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.651 | 12.81 | 27.25 | 0.161 | 1.392 | 0 | 29.13 |
| 六机林带 | base + track geom 0.08 m（偏离上游） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.731 | 13.55 | 27.38 | 0.202 | 1.148 | 0 | 27.90 |
| 六机林带 | base + track geom 0.08 m（偏离上游） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.670 | 15.13 | 33.90 | 0.239 | 1.189 | 0 | 30.08 |
| 六机林带 | base + track geom 0.08 m（偏离上游） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.902 | 13.42 | 28.76 | 0.194 | 1.037 | 0 | 25.78 |
| 六机林带 | base + track geom_multi 0.08 m（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.870 | 12.49 | 26.69 | 0.306 | 1.293 | 0 | 27.08 |
| 六机林带 | base + track geom_multi 0.08 m（偏离上游） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.651 | 12.23 | 20.24 | 0.161 | 1.392 | 0 | 29.13 |
| 六机林带 | base + track geom_multi 0.08 m（偏离上游） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.731 | 11.65 | 12.37 | 0.202 | 1.148 | 0 | 27.90 |
| 六机林带 | base + track geom_multi 0.08 m（偏离上游） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.670 | 11.60 | 12.29 | 0.239 | 1.189 | 0 | 30.08 |
| 六机林带 | base + track geom_multi 0.08 m（偏离上游） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 1.902 | 11.70 | 12.69 | 0.194 | 1.037 | 0 | 25.78 |
| 014 六机对向 | 默认上游 + 影子盾 | 0 | COLLISION | `uav2-uav3:INTER_DRONE` | 0/6 | 0 | 0 | 1 | 3.268 | 11.06 | 13.95 | 0.172 | 0.688 | 0 | 2.28 |
| 014 六机对向 | 默认上游 + 影子盾 | 1 | COLLISION | `uav2-uav3:INTER_DRONE` | 0/6 | 0 | 0 | 1 | 3.251 | 10.88 | 12.01 | 0.154 | 0.667 | 0 | 2.28 |
| 014 六机对向 | 默认上游 + 影子盾 | 2 | COLLISION | `uav2-uav4:INTER_DRONE` | 0/6 | 0 | 0 | 1 | 3.571 | 11.21 | 11.83 | 0.145 | 0.660 | 0 | 2.60 |
| 014 六机对向 | 默认上游 + 影子盾 | 3 | COLLISION | `uav2-uav3:INTER_DRONE` | 0/6 | 0 | 0 | 1 | 3.235 | 11.41 | 13.30 | 0.178 | 0.675 | 0 | 2.29 |
| 014 六机对向 | 默认上游 + 影子盾 | 4 | COLLISION | `uav2-uav3:INTER_DRONE` | 0/6 | 0 | 0 | 1 | 3.246 | 10.45 | 11.47 | 0.202 | 0.682 | 0 | 2.26 |
| 014 六机对向 | E-A1 1.0 m + empty_brake（偏离上游） | 0 | COLLISION | `uav0:TRUTH_POSITION` | 0/6 | 1 | 0 | 0 | 3.166 | 9.41 | 11.23 | -0.005 | 1.857 | 3 | 2.58 |
| 014 六机对向 | E-A1 1.0 m + empty_brake（偏离上游） | 1 | COLLISION | `uav5:TRUTH_POSITION` | 0/6 | 1 | 0 | 0 | 3.838 | 9.26 | 11.10 | -0.021 | 1.165 | 0 | 3.65 |
| 014 六机对向 | E-A1 1.0 m + empty_brake（偏离上游） | 2 | COLLISION | `uav5:TRUTH_POSITION` | 0/6 | 1 | 0 | 0 | 3.741 | 9.11 | 10.10 | -0.008 | 1.152 | 0 | 3.59 |
| 014 六机对向 | E-A1 1.0 m + empty_brake（偏离上游） | 3 | COLLISION | `uav0:TRUTH_POSITION` | 0/6 | 1 | 0 | 0 | 3.172 | 9.18 | 10.08 | -0.009 | 2.686 | 0 | 2.71 |
| 014 六机对向 | E-A1 1.0 m + empty_brake（偏离上游） | 4 | COLLISION | `uav3:TRUTH_POSITION` | 0/6 | 1 | 0 | 0 | 2.448 | 8.85 | 9.51 | -0.030 | 4.323 | 1 | 1.85 |
| 014 六机对向 | base + track geom 0.08 m（偏离上游） | 0 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 0.564 | 10.66 | 11.64 | 0.137 | 1.187 | 372 | 40.00 |
| 014 六机对向 | base + track geom 0.08 m（偏离上游） | 1 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 0.708 | 10.56 | 12.02 | 0.112 | 0.966 | 372 | 40.00 |
| 014 六机对向 | base + track geom 0.08 m（偏离上游） | 2 | COLLISION | `uav4:TRUTH_POSITION` | 0/6 | 1 | 0 | 0 | 2.869 | 10.76 | 11.36 | -0.007 | 2.230 | 1 | 2.27 |
| 014 六机对向 | base + track geom 0.08 m（偏离上游） | 3 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 0.734 | 10.19 | 11.88 | 0.137 | 1.076 | 362 | 40.00 |
| 014 六机对向 | base + track geom 0.08 m（偏离上游） | 4 | COLLISION | `uav3:TRUTH_POSITION` | 0/6 | 1 | 0 | 0 | 3.026 | 10.40 | 11.89 | -0.003 | 1.253 | 2 | 2.55 |
| 014 六机对向 | base + track geom_multi 0.08 m（偏离上游） | 0 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 0.564 | 10.49 | 11.82 | 0.137 | 1.187 | 372 | 40.00 |
| 014 六机对向 | base + track geom_multi 0.08 m（偏离上游） | 1 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 0.708 | 10.40 | 11.84 | 0.112 | 0.966 | 372 | 40.00 |
| 014 六机对向 | base + track geom_multi 0.08 m（偏离上游） | 2 | COLLISION | `uav4:TRUTH_POSITION` | 0/6 | 1 | 0 | 0 | 2.869 | 9.98 | 11.14 | -0.007 | 2.230 | 1 | 2.27 |
| 014 六机对向 | base + track geom_multi 0.08 m（偏离上游） | 3 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 0.734 | 10.64 | 11.75 | 0.137 | 1.076 | 362 | 40.00 |
| 014 六机对向 | base + track geom_multi 0.08 m（偏离上游） | 4 | COLLISION | `uav3:TRUTH_POSITION` | 0/6 | 1 | 0 | 0 | 3.026 | 10.28 | 11.37 | -0.003 | 1.253 | 2 | 2.55 |

## 复现

```
.venv/bin/python scripts/e-c6-six-density.py --partial --jobs 4
```

`--partial` 从 `out/e-c6-six-density/partial/` 续跑已经完成的种子。缺省也续跑。`--no-resume` 忽略已有 partial。
