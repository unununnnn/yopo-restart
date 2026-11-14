# 票 E-C8b：六机 014 欧氏球 × 几何门 × 脱困重规划，只报数

只报数。report-only。没有改 `yopo_l0/` 的默认、`core/`、已冻门、`scenes/`、`docs/tickets.md`、README、`scripts/premerge.sh` 和 `scripts/regress.sh`。`safe_radius` 仍是 0.05 m，`empty_brake` 仍是 false，`track_clearance` 仍是 false，`peer_sphere_enabled` 仍是 false，机间过滤仍是关。六臂全部偏离上游。0.08 m 是几何门读者的缺省，也是假设，用来盖 [E-C5](e-c5-corridor-radius.md) 记下的 0.058–0.073 m 跟踪差。未冻结，不进 `scenes/suite.txt`，不设 gates，不改已冻门。

数字在 [e-c8b-ablation-runs.json](e-c8b-ablation-runs.json)。跑数时运行器读到的 HEAD 是 `0a6563f78e0c478158fcc0997c073de979d4a937`。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。

## 结论

先前独立报告里的 24/30，这六臂都没有出现。票面那一档「geom + empty_brake + N=10 脱困 + 默认 E-B4c」是臂 D：到达 20/30，撞柱 1，机间 0，超时 4。

到达主要来自 E-B4c，不来自脱困。

- 臂 C 只开默认 E-B4c（半径 0.85 m，权重 8.0）。到达 30/30，撞柱 0，机间 0，超时 0。这和先前观察到的「014 六机对向只开默认欧氏球是 30/30」一致。最小净空 0.001–0.086 m，最小机间 0.857–0.943 m。
- 臂 A 只有 `track_clearance=geom` 和 `empty_brake`，没有欧氏球，也没有脱困。到达 0/30。五粒都是机间碰撞，最小机间 0.660–0.688 m，低于 0.70 m。几何门加刹停自己没有把飞机送到终点。
- 臂 B 是臂 A 加上默认 E-B4c，仍然没有脱困。这是先前 E-C8 缺的孤立对照。到达 20/30，撞柱 1，机间 0，超时 4。机间碰撞没有了。相对臂 C 的 30/30，少掉的 10 架是几何门加上去之后的：四粒超时各到 5/6，seed 2 在仿真 4.11 s 由 `uav0` 撞上圆柱 `obs19`。
- 臂 D 在臂 B 上加 N=10 脱困。到达、撞柱、机间、超时与臂 B 相同，仍是 20/30、撞柱 1、超时 4。脱困触发 77 次，保持 727 拍，走廊重开 0 次，到期 72 次。seed 0、2、3、4 的位置哈希与臂 B 相同：临时目标没有把走廊重开，空走廊之后仍是从参考头刹停，轨迹不变。seed 1 的哈希不同；臂 B 这一粒有 1 次后段预算截断（1/337），臂 D 是 0 次。两粒都是超时 5/6。

所以叠在几何门上的那 20 次到达是 E-B4c 拿掉机间碰撞之后留下来的，不是脱困补上的。脱困没有把 20/30 推到 24/30，也没有推到 30/30。

加强半径或权重都不能消除那一次撞柱，也都到不了 30/30。

- 臂 E 把半径改成 1.00 m，权重仍是 8.0，脱困仍开。到达 21/30，撞柱仍是 1，超时 3。撞柱还是 seed 2 的 `uav0` / `obs19`，仿真 4.08 s。多出来的 1 架是 seed 4：6/6 到达，这一粒脱困有 1 次走廊重开。其余三粒超时仍是 5/6。
- 臂 F 半径仍是 0.85 m，权重改成 16.0，脱困仍开。到达 20/30，撞柱 1，机间 0，超时 4，走廊重开 0。撞柱还是 seed 2 的 `uav0` / `obs19`。权重加大没有拿掉这根柱，也没有减少超时。

后段预算截断合计：臂 B 1/1726，臂 F 1/1802，其余四臂 0。不是上面到达差的原因。

## 口径

到达按架次。seeds 0–4、每粒 6 架，一臂的分母是 30。撞柱、撞地、机间按运行器，仿真在第一次碰撞停下，所以一次运行最多记 1。撞柱是 `TRUTH_POSITION` 且碰撞时刻最近的障碍是圆柱。机间是 `INTER_DRONE`。超时是 `TIMEOUT` 的运行次数。

最小净空 = `min_obstacle_distance_m` − 0.35 m。最小机间是中心距。汇总表里的平均速度是该档各次 `avg_speed_mps` 的算术平均。`avg_speed_mps` 只平均严格大于 0 的有限速度。

空走廊拍是全部飞机在规划拍上 `plan_code` 为 `CORRIDOR_EMPTY_HOVER` 的次数之和。规划拍与运行器一致：physics 100 Hz、planner 10 Hz 时即 `k` 能被 10 整除。`empty_brake` 打开时，这一拍是从参考头刹停。

脱困触发、保持、重开、到期按规划拍，六架合计。触发的那一拍仍飞原目标，下一拍才用临时目标。保持满 10 拍或走廊重开（规划码不再是 `CORRIDOR_EMPTY_HOVER`）之后回到原目标。任务到达只认场景卡上的原目标。

预算截断是后段 `refine_yopo` 因为 0.02 s 墙钟预算提前停的次数，分母是调用次数。

运行器字段 `upstream_deviation`：臂 A、B、D、E、F 是 true，因为开了 `empty_brake` 和 `track_clearance`。臂 C 是 false。运行器不把 `peer_sphere_enabled` 写进这个字段。臂 C 打开了欧氏球，仍是偏离上游 / report-only。六臂的标记列都写「偏离上游 / report-only」。

E-A1（`inter_drone_filter`）六臂都关。这和 [E-C6](e-c6-six-density.md) 的 geom 列不是同一档：那边的 geom 还开着阈值 1.0 m 的机间过滤。

## 汇总

| 场景 | 臂 | 标记 | 到达 | 撞柱 | 撞地 | 机间 | 超时 | 平均速度 m/s | 最小净空 m | 最小机间 m | 空走廊拍 | 脱困触发 | 脱困保持拍 | 重开回原目标 | 脱困到期 | 预算截断 |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---:|
| 014 六机对向 | A geom + empty_brake（无 E-B4c，无脱困；偏离上游 / report-only） | 偏离上游 / report-only | 0/30 | 0 | 0 | 5 | 0 | 3.194 | 0.193–0.259 | 0.660–0.688 | 6 | 0 | 0 | 0 | 0 | 0/0 |
| 014 六机对向 | B geom + empty_brake + E-B4c r=0.85 w=8.0（无脱困；偏离上游 / report-only） | 偏离上游 / report-only | 20/30 | 1 | 0 | 0 | 4 | 1.182 | -0.003 至 0.191 | 0.908–0.960 | 1501 | 0 | 0 | 0 | 0 | 1/1726 |
| 014 六机对向 | C E-B4c r=0.85 w=8.0 单独（偏离上游 / report-only） | 偏离上游 / report-only | 30/30 | 0 | 0 | 0 | 0 | 1.824 | 0.001–0.086 | 0.857–0.943 | 1 | 0 | 0 | 0 | 0 | 0/2328 |
| 014 六机对向 | D geom + empty_brake + E-C8 N=10 + E-B4c r=0.85 w=8.0（偏离上游 / report-only） | 偏离上游 / report-only | 20/30 | 1 | 0 | 0 | 4 | 1.193 | -0.003 至 0.191 | 0.908–0.960 | 1501 | 77 | 727 | 0 | 72 | 0/1793 |
| 014 六机对向 | E geom + empty_brake + E-C8 N=10 + E-B5 r=1.00 w=8.0（偏离上游 / report-only） | 偏离上游 / report-only | 21/30 | 1 | 0 | 0 | 3 | 1.471 | -0.004 至 0.168 | 0.902–1.009 | 1149 | 59 | 550 | 2 | 53 | 0/1956 |
| 014 六机对向 | F geom + empty_brake + E-C8 N=10 + E-B5 r=0.85 w=16.0（偏离上游 / report-only） | 偏离上游 / report-only | 20/30 | 1 | 0 | 0 | 4 | 1.194 | -0.001 至 0.189 | 0.862–0.971 | 1502 | 77 | 726 | 0 | 72 | 1/1802 |

## 场景和臂

场景只有一张：014 六机对向，goto，40 s，seeds 0–4。障碍布局取 `scenes/ego-density-014-swap3.json` 解析后的几何，也就是 `scenes/ego-density-014.json`。sha256 是 `5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9`。柱子 56 根，目标密度 0.14 个/m²。卡只在 `scripts/e-c8b-ablation.py` 的进程内存里构造，不写入 `scenes/`。起终点与 [E-C6](e-c6-six-density.md) 的六机对向相同。

| 飞机 | 半径 m | 推出正方形 | 起点轴距 m | 目标轴距 m | 起点 | 目标 |
|---|---:|---|---:|---:|---|---|
| uav0 | 8.000 | 否 | 2.874 | 1.016 | 8.000, 0.000, 1.500 | -8.000, 0.000, 1.500 |
| uav1 | 8.000 | 否 | 1.851 | 1.434 | 4.000, 6.928, 1.500 | -4.000, -6.928, 1.500 |
| uav2 | 11.597 | 是 | 2.173 | 1.863 | -5.799, 10.043, 1.500 | 5.799, -10.043, 1.500 |
| uav3 | 8.000 | 否 | 1.016 | 2.874 | -8.000, 0.000, 1.500 | 8.000, 0.000, 1.500 |
| uav4 | 8.000 | 否 | 1.434 | 1.851 | -4.000, -6.928, 1.500 | 4.000, 6.928, 1.500 |
| uav5 | 11.597 | 是 | 1.863 | 2.173 | 5.799, -10.043, 1.500 | -5.799, 10.043, 1.500 |

| 臂 | 进程里多出来的 | 偏离上游 |
|---|---|---|
| A | `track_clearance=true`，`mode=geom`，`track_margin_m=0.08`；`empty_brake=true`。无后段，无脱困 | 偏离上游 / report-only |
| B | A 再加 `backend.enabled` 与 `peer_sphere_enabled`，半径 0.85 m，权重 8.0。无脱困 | 偏离上游 / report-only |
| C | 只加默认 E-B4c：半径 0.85 m，权重 8.0。无 geom，无 `empty_brake`，无脱困 | 偏离上游 / report-only |
| D | B 再加 E-C8：连续 10 个空走廊规划拍之后，向深度图最通畅扇区给临时目标，最多保持 10 拍 | 偏离上游 / report-only |
| E | 与 D 相同，半径改成 1.00 m，权重仍是 8.0 | 偏离上游 / report-only |
| F | 与 D 相同，半径仍是 0.85 m，权重改成 16.0 | 偏离上游 / report-only |

`safe_radius` 六臂都不写，运行器缺省 0.05 m。不写 `inter_drone_filter`。欧氏球的邻机采样沿 [E-B4c](e-b4c-sphere-avoid.md) 的进程内包装送进 `refine_yopo`：不改 `policy.py`。运行器仍是本 tick 全部 `plan()` 之后才 `bus.poll`，后段用到的是上一规划拍已经提交的采样。椭球开关保持关。

脱困也只包在进程里，不进 `yopo_l0/`。水平视场按 9 个扇区，地平线中间三分之一行的深度中位数最大的一档算最通畅，无回波按相机最远 20 m 计。临时目标在触发时刻的机体坐标里是前向 4.0 m、侧向 1.2–1.5 m（图像右侧为负，幅度随方位角从 1.2 m 增到 1.5 m；正前方打平则取 1.2 m，偏向更通畅的一侧）。高度保持触发时刻的 z。若机体已经进入这个点的到达半径（缺省 1.5 m），沿触发时的瞄准方向把规划目标推到到达半径外 0.5 m，避免策略把脱困点锁成终点。

## 分种子

| 臂 | seed | 结果 | 原因 | 到达 | 撞柱 | 撞地 | 机间 | 超时 | 平均速度 m/s | plan 均值 ms | plan p95 ms | 最小净空 m | 最小机间 m | 空走廊拍 | 脱困触发 | 脱困保持 | 重开 | 到期 | 仿真 s |
|---|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| A geom + empty_brake（无 E-B4c，无脱困；偏离上游 / report-only） | 0 | COLLISION | `uav2-uav3:INTER_DRONE` | 0/6 | 0 | 0 | 1 | 0 | 3.152 | 9.14 | 11.42 | 0.193 | 0.688 | 3 | 0 | 0 | 0 | 0 | 2.28 |
| A geom + empty_brake（无 E-B4c，无脱困；偏离上游 / report-only） | 1 | COLLISION | `uav2-uav3:INTER_DRONE` | 0/6 | 0 | 0 | 1 | 0 | 3.155 | 9.19 | 10.97 | 0.236 | 0.667 | 0 | 0 | 0 | 0 | 0 | 2.28 |
| A geom + empty_brake（无 E-B4c，无脱困；偏离上游 / report-only） | 2 | COLLISION | `uav1-uav4:INTER_DRONE` | 0/6 | 0 | 0 | 1 | 0 | 3.339 | 9.15 | 10.93 | 0.246 | 0.660 | 1 | 0 | 0 | 0 | 0 | 2.48 |
| A geom + empty_brake（无 E-B4c，无脱困；偏离上游 / report-only） | 3 | COLLISION | `uav2-uav3:INTER_DRONE` | 0/6 | 0 | 0 | 1 | 0 | 3.177 | 9.22 | 10.31 | 0.229 | 0.675 | 1 | 0 | 0 | 0 | 0 | 2.29 |
| A geom + empty_brake（无 E-B4c，无脱困；偏离上游 / report-only） | 4 | COLLISION | `uav2-uav3:INTER_DRONE` | 0/6 | 0 | 0 | 1 | 0 | 3.147 | 8.60 | 9.14 | 0.259 | 0.682 | 1 | 0 | 0 | 0 | 0 | 2.26 |
| B geom + empty_brake + E-B4c r=0.85 w=8.0（无脱困；偏离上游 / report-only） | 0 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.588 | 11.20 | 12.36 | 0.191 | 0.928 | 372 | 0 | 0 | 0 | 0 | 40.00 |
| B geom + empty_brake + E-B4c r=0.85 w=8.0（无脱困；偏离上游 / report-only） | 1 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.552 | 11.31 | 12.35 | 0.157 | 0.927 | 372 | 0 | 0 | 0 | 0 | 40.00 |
| B geom + empty_brake + E-B4c r=0.85 w=8.0（无脱困；偏离上游 / report-only） | 2 | COLLISION | `uav0:TRUTH_POSITION obs19` | 0/6 | 1 | 0 | 0 | 0 | 3.581 | 10.42 | 11.66 | -0.003 | 0.960 | 13 | 0 | 0 | 0 | 0 | 4.11 |
| B geom + empty_brake + E-B4c r=0.85 w=8.0（无脱困；偏离上游 / report-only） | 3 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.577 | 11.18 | 12.01 | 0.177 | 0.908 | 372 | 0 | 0 | 0 | 0 | 40.00 |
| B geom + empty_brake + E-B4c r=0.85 w=8.0（无脱困；偏离上游 / report-only） | 4 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.610 | 10.75 | 11.92 | 0.177 | 0.928 | 372 | 0 | 0 | 0 | 0 | 40.00 |
| C E-B4c r=0.85 w=8.0 单独（偏离上游 / report-only） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 0 | 1.667 | 11.60 | 12.85 | 0.024 | 0.857 | 1 | 0 | 0 | 0 | 0 | 17.91 |
| C E-B4c r=0.85 w=8.0 单独（偏离上游 / report-only） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 0 | 1.904 | 10.18 | 11.78 | 0.024 | 0.929 | 0 | 0 | 0 | 0 | 0 | 13.88 |
| C E-B4c r=0.85 w=8.0 单独（偏离上游 / report-only） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 0 | 1.842 | 9.85 | 11.61 | 0.025 | 0.874 | 0 | 0 | 0 | 0 | 0 | 14.28 |
| C E-B4c r=0.85 w=8.0 单独（偏离上游 / report-only） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 0 | 1.829 | 10.23 | 11.99 | 0.086 | 0.905 | 0 | 0 | 0 | 0 | 0 | 15.94 |
| C E-B4c r=0.85 w=8.0 单独（偏离上游 / report-only） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 0 | 1.878 | 10.91 | 12.12 | 0.001 | 0.943 | 0 | 0 | 0 | 0 | 0 | 15.73 |
| D geom + empty_brake + E-C8 N=10 + E-B4c r=0.85 w=8.0（偏离上游 / report-only） | 0 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.588 | 10.36 | 12.02 | 0.191 | 0.928 | 372 | 19 | 181 | 0 | 18 | 40.00 |
| D geom + empty_brake + E-C8 N=10 + E-B4c r=0.85 w=8.0（偏离上游 / report-only） | 1 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.610 | 10.97 | 12.42 | 0.157 | 0.930 | 372 | 19 | 181 | 0 | 18 | 40.00 |
| D geom + empty_brake + E-C8 N=10 + E-B4c r=0.85 w=8.0（偏离上游 / report-only） | 2 | COLLISION | `uav0:TRUTH_POSITION obs19` | 0/6 | 1 | 0 | 0 | 0 | 3.581 | 10.76 | 12.16 | -0.003 | 0.960 | 13 | 1 | 3 | 0 | 0 | 4.11 |
| D geom + empty_brake + E-C8 N=10 + E-B4c r=0.85 w=8.0（偏离上游 / report-only） | 3 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.577 | 11.64 | 12.10 | 0.177 | 0.908 | 372 | 19 | 181 | 0 | 18 | 40.00 |
| D geom + empty_brake + E-C8 N=10 + E-B4c r=0.85 w=8.0（偏离上游 / report-only） | 4 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.610 | 11.62 | 12.20 | 0.177 | 0.928 | 372 | 19 | 181 | 0 | 18 | 40.00 |
| E geom + empty_brake + E-C8 N=10 + E-B5 r=1.00 w=8.0（偏离上游 / report-only） | 0 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.600 | 11.52 | 12.10 | 0.162 | 0.902 | 373 | 19 | 181 | 1 | 17 | 40.00 |
| E geom + empty_brake + E-C8 N=10 + E-B5 r=1.00 w=8.0（偏离上游 / report-only） | 1 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.699 | 11.60 | 12.21 | 0.159 | 0.985 | 377 | 19 | 181 | 0 | 18 | 40.00 |
| E geom + empty_brake + E-C8 N=10 + E-B5 r=1.00 w=8.0（偏离上游 / report-only） | 2 | COLLISION | `uav0:TRUTH_POSITION obs19` | 0/6 | 1 | 0 | 0 | 0 | 3.574 | 10.44 | 11.82 | -0.004 | 0.978 | 12 | 1 | 2 | 0 | 0 | 4.08 |
| E geom + empty_brake + E-C8 N=10 + E-B5 r=1.00 w=8.0（偏离上游 / report-only） | 3 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.594 | 10.58 | 11.93 | 0.158 | 1.004 | 372 | 19 | 181 | 0 | 18 | 40.00 |
| E geom + empty_brake + E-C8 N=10 + E-B5 r=1.00 w=8.0（偏离上游 / report-only） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 6/6 | 0 | 0 | 0 | 0 | 1.889 | 9.42 | 11.65 | 0.168 | 1.009 | 15 | 1 | 5 | 1 | 0 | 14.28 |
| F geom + empty_brake + E-C8 N=10 + E-B5 r=0.85 w=16.0（偏离上游 / report-only） | 0 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.578 | 10.28 | 12.18 | 0.133 | 0.961 | 371 | 19 | 181 | 0 | 18 | 40.00 |
| F geom + empty_brake + E-C8 N=10 + E-B5 r=0.85 w=16.0（偏离上游 / report-only） | 1 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.609 | 10.27 | 13.05 | 0.189 | 0.936 | 375 | 19 | 181 | 0 | 18 | 40.00 |
| F geom + empty_brake + E-C8 N=10 + E-B5 r=0.85 w=16.0（偏离上游 / report-only） | 2 | COLLISION | `uav0:TRUTH_POSITION obs19` | 0/6 | 1 | 0 | 0 | 0 | 3.578 | 11.93 | 14.69 | -0.001 | 0.881 | 12 | 1 | 2 | 0 | 0 | 4.07 |
| F geom + empty_brake + E-C8 N=10 + E-B5 r=0.85 w=16.0（偏离上游 / report-only） | 3 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.608 | 11.27 | 12.41 | 0.176 | 0.862 | 372 | 19 | 181 | 0 | 18 | 40.00 |
| F geom + empty_brake + E-C8 N=10 + E-B5 r=0.85 w=16.0（偏离上游 / report-only） | 4 | TIMEOUT | `GOAL_NOT_REACHED` | 5/6 | 0 | 0 | 0 | 1 | 0.596 | 11.11 | 12.06 | 0.177 | 0.971 | 372 | 19 | 181 | 0 | 18 | 40.00 |

## 复现

```
.venv/bin/python scripts/e-c8b-ablation.py --jobs 2
```

`--jobs` 最多 2，避免墙钟预算把后段迭代截短。缺省从 `out/e-c8b-ablation/partial/` 续跑。`--no-resume` 忽略已有 partial。`--check` 只检查内存卡和六臂开关，不跑仿真。
