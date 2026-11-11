# 票 12e：上游策略 vs 影子盾

方向 D 素材。比较的是同一张场景卡、同一个 commit 上：

- **upstream**（穿林 / 编队任务里结果记录的策略名是 `crossing` / `formation`）：规划按上游等价规则选候选，影子盾只记录「这一拍本该不该拒」，不改批准动作。
- **certified**：现行安全门真正拦截。只接在单机 `goto` / `hover` 上。仓库没有把影子盾改成硬拦的开关。

穿林和编队任务由任务类型决定规划路径，`--policy certified` 会被忽略，不能当成多机硬门对照。

## 出处

| 项 | 值 |
|---|---|
| 代码 commit | `f5985e4335fcc00a4e19c861dcd1a05e0113f797`（跑数时的 HEAD；结果记录无 `-dirty`） |
| 层级 | L0 |
| 与当前 main 的关系 | 跑数后 main 已合入 12a/12c/12d（文档和脚本），不改规划器与 `core/`，与本表数字对应的树相同 |
| 形态 | 进程内 |
| 跑数时间 | 2026-11-10 22:42–23:50（America/La_Paz，UTC-4）；写表于 23:51 |
| 到达口径 | ≤ 0.4 m 且 ≤ 0.2 m/s，保持 1 s；机体半径 0.35 m |
| 机间碰撞线 | 0.70 m（两倍机体半径） |
| 净空 | 机体中心到障碍表面的距离 − 0.35 m |
| 影子盾 | 开（对照列）；关的核验见「不干预」节 |
| 数字来源 | 本票脚本 `scripts/12e-upstream-vs-shield.py` 在干净工作区上的实测，原始记录 [12e-runs.json](12e-runs.json) |

墙钟随机器负载变，不参与确定性比较。同 seed 重跑只要求除墙钟外的结果一致。

## 怎么读

- 影子拒绝率是全体 tick 里 `would_reject` 的比例。一 tick 多个拒绝码在直方图里各计一次。
- 拒绝不改 `outcome`，也不进碰撞计数。碰撞是几何真值。
- certified 对照不是里程碑通过线，继续只报数（ADR 0010、ADR 0016）。

## 总表

| 场景 | 策略 | 层级 | seeds | 到达 | 碰撞 | 到达时间 s | 影子拒绝率 | 主要拒绝码 | 最小净空 m | 最小机间 m | 墙钟 s | commit |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| g2-pillar @ upstream + 影子盾开 | upstream | L0 | 0,1,2,3,4,5,6,7,8,9 | 10/10 | 0 | 4.27（4.26–4.28） | 1.000 | ENV:UNKNOWN:5271, TIME:SWITCH_NOT_IN_FUTURE:3171, TRANSITION:HEAD_DISCONTINUITY:3171 | 0.478–0.479 | — | 14.5（13.9–15.7） | `f5985e4335fc` |
| g2-pillar @ certified + 影子盾开（硬门拦截） | certified | L0 | 0,1,2,3,4,5,6,7,8,9 | 0/10 | 0 | — | 0.778 | LAG_VIOLATION:34990 | 1.150 | — | 804.5（759.3–896.0） | `f5985e4335fc` |
| crossing-pillars @ crossing + 影子盾开 | crossing | L0 | 0,1,2,3,4,5,6,7,8,9 | 30/30 | 0 | 7.27（7.05–7.77） | 1.000 | ENV:UNKNOWN:24060, TRANSITION:MOTION_KINEMATICS:13725, TRANSITION:BRANCH_UNREACHABLE:11250 | 0.528–1.106 | 3.350–7.510 | 18.6（17.9–19.4） | `f5985e4335fc` |
| crossing-forest medium @ crossing + 影子盾开 | crossing | L0 | 0,1,2,3,4,5,6,7,8,9 | 30/30 | 0 | 23.23（20.77–24.42） | 1.000 | ENV:UNKNOWN:71955, TRANSITION:HEAD_DISCONTINUITY:33930, TRANSITION:MOTION_KINEMATICS:27600 | 0.260–0.450 | 4.602–7.469 | 96.4（85.3–103.1） | `f5985e4335fc` |
| crossing-forest sparse @ crossing + 影子盾开 | crossing | L0 | 0,1,2,3,4,5,6,7,8,9 | 30/30 | 0 | 23.50（22.09–26.80） | 1.000 | ENV:UNKNOWN:72744, TRANSITION:HEAD_DISCONTINUITY:34794, TRANSITION:MOTION_KINEMATICS:27075 | 0.468–0.985 | 4.312–7.554 | 78.6（70.9–92.1） | `f5985e4335fc` |
| crossing-forest dense @ crossing + 影子盾开 | crossing | L0 | 0,1,2,3,4,5,6,7,8,9 | 3/30 | 9 | 26.81 | 1.000 | ENV:UNKNOWN:29429, TRANSITION:MOTION_KINEMATICS:12863, TRANSITION:BRANCH_UNREACHABLE:12638 | -0.027–0.287 | 4.688–5.510 | 62.4（38.3–171.8） | `f5985e4335fc` |
| formation-hold @ formation + 影子盾开 | formation | L0 | 0,1,2 | 9/9 | 0 | 18.63 | 1.000 | ENV:UNKNOWN:17172, TIME:SWITCH_NOT_IN_FUTURE:495 | 1.150 | 1.987 | 5.2（4.8–5.5） | `f5985e4335fc` |
| crossing-6-forest medium @ crossing + 影子盾开 | crossing | L0 | 0 | 6/6 | 0 | 26.08 | 1.000 | ENV:UNKNOWN:15723, TRANSITION:HEAD_DISCONTINUITY:10323, TRANSITION:MOTION_KINEMATICS:4950 | 0.306 | 1.293 | 234.2 | `f5985e4335fc` |
| crossing-6-forest-sparse @ crossing + 影子盾开 | crossing | L0 | 0 | 6/6 | 0 | 25.40 | 1.000 | ENV:UNKNOWN:15465, TRANSITION:HEAD_DISCONTINUITY:9915, TRANSITION:MOTION_KINEMATICS:4950 | 0.315 | 0.959 | 176.0 | `f5985e4335fc` |

## 不干预：影子盾开/关动作序列逐位一致

仓库已有 `tests/test_shadow.py`：悬停、三机开阔区、g2-pillar 的 upstream 与 certified（短时）在同 seed 下开/关盾，批准动作、规划输入和去掉影子字段后的结果记录一致。本票复跑该文件，并通过；再给本票代表卡补了短时开/关核验。没有再做各卡全时长的开/关对照，以免把穿林和六机的墙钟再翻一倍。短时核验覆盖批准动作序列，机制是逐 tick 的，与全时长同一条路径。

| 检查 | 结果 |
|---|---|
| `pytest tests/test_shadow.py` | 通过 |
| 短时 g2-pillar @ upstream seed 0（0.5 s，50 tick） | 动作一致，去掉影子字段后一致 |
| 短时 crossing-pillars @ crossing seed 0（1.0 s，300 tick） | 动作一致，去掉影子字段后一致 |
| 短时 crossing-forest @ crossing seed 0（1.0 s，300 tick） | 动作一致，去掉影子字段后一致 |
| 短时 formation-hold @ formation seed 0（1.0 s，300 tick） | 动作一致，去掉影子字段后一致 |
| 短时 crossing-6-forest @ crossing seed 0（1.0 s，600 tick） | 动作一致，去掉影子字段后一致 |

结论：当前 main 上影子盾仍然只记录、不改动作。同 seed 开/关盾，动作序列逐位一致。

## 分场景

### g2-pillar @ upstream + 影子盾开

层级 L0，策略 `upstream`，影子盾开，commit `f5985e4335fcc00a4e19c861dcd1a05e0113f797`。到达 10/10，碰撞 0。场景卡哈希 `9d385f4cfce543922961865d06ceeedd44afdddb52c706bf6e3da9929431d8ae`。

| seed | 结果 | 到达 | 到达时间 s | 最小障碍距离 m | 最小净空 m | 最小机间距离 m | 影子拒绝率 | 主要拒绝码 | 仿真 s | 墙钟 s |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | SUCCESS | 1/1 | 4.26 | 0.829 | 0.479 | — | 1.000 | ENV:UNKNOWN:526, TIME:SWITCH_NOT_IN_FUTURE:316, TRANSITION:HEAD_DISCONTINUITY:316 | 5.26 | 15.5 |
| 1 | SUCCESS | 1/1 | 4.27 | 0.829 | 0.479 | — | 1.000 | ENV:UNKNOWN:527, TIME:SWITCH_NOT_IN_FUTURE:317, TRANSITION:HEAD_DISCONTINUITY:317 | 5.27 | 15.7 |
| 2 | SUCCESS | 1/1 | 4.26 | 0.829 | 0.479 | — | 1.000 | ENV:UNKNOWN:526, TIME:SWITCH_NOT_IN_FUTURE:316, TRANSITION:HEAD_DISCONTINUITY:316 | 5.26 | 14.4 |
| 3 | SUCCESS | 1/1 | 4.26 | 0.829 | 0.479 | — | 1.000 | ENV:UNKNOWN:526, TIME:SWITCH_NOT_IN_FUTURE:316, TRANSITION:HEAD_DISCONTINUITY:316 | 5.26 | 14.4 |
| 4 | SUCCESS | 1/1 | 4.27 | 0.829 | 0.479 | — | 1.000 | ENV:UNKNOWN:527, TIME:SWITCH_NOT_IN_FUTURE:317, TRANSITION:HEAD_DISCONTINUITY:317 | 5.27 | 14.4 |
| 5 | SUCCESS | 1/1 | 4.28 | 0.828 | 0.478 | — | 1.000 | ENV:UNKNOWN:528, TIME:SWITCH_NOT_IN_FUTURE:318, TRANSITION:HEAD_DISCONTINUITY:318 | 5.28 | 14.3 |
| 6 | SUCCESS | 1/1 | 4.28 | 0.828 | 0.478 | — | 1.000 | ENV:UNKNOWN:528, TIME:SWITCH_NOT_IN_FUTURE:318, TRANSITION:HEAD_DISCONTINUITY:318 | 5.28 | 14.1 |
| 7 | SUCCESS | 1/1 | 4.28 | 0.828 | 0.478 | — | 1.000 | ENV:UNKNOWN:528, TIME:SWITCH_NOT_IN_FUTURE:318, TRANSITION:HEAD_DISCONTINUITY:318 | 5.28 | 14.3 |
| 8 | SUCCESS | 1/1 | 4.28 | 0.828 | 0.478 | — | 1.000 | ENV:UNKNOWN:528, TIME:SWITCH_NOT_IN_FUTURE:318, TRANSITION:HEAD_DISCONTINUITY:318 | 5.28 | 13.9 |
| 9 | SUCCESS | 1/1 | 4.27 | 0.829 | 0.479 | — | 1.000 | ENV:UNKNOWN:527, TIME:SWITCH_NOT_IN_FUTURE:317, TRANSITION:HEAD_DISCONTINUITY:317 | 5.27 | 14.3 |

拒绝码合计（所有 seed、一 tick 多码各计一次）：

| 拒绝码 | 次数 | 占直方图合计 |
|---|---|---|
| `ENV:UNKNOWN` | 5271 | 0.384 |
| `TIME:SWITCH_NOT_IN_FUTURE` | 3171 | 0.231 |
| `TRANSITION:HEAD_DISCONTINUITY` | 3171 | 0.231 |
| `TRANSITION:TERMINAL_NOT_HOLDABLE` | 2100 | 0.153 |

### g2-pillar @ certified + 影子盾开（硬门拦截）

层级 L0，策略 `certified`，影子盾开，commit `f5985e4335fcc00a4e19c861dcd1a05e0113f797`。到达 0/10，碰撞 0。场景卡哈希 `9d385f4cfce543922961865d06ceeedd44afdddb52c706bf6e3da9929431d8ae`。

| seed | 结果 | 到达 | 到达时间 s | 最小障碍距离 m | 最小净空 m | 最小机间距离 m | 影子拒绝率 | 主要拒绝码 | 仿真 s | 墙钟 s |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | TIMEOUT | 0/1 | — | 1.500 | 1.150 | — | 0.778 | LAG_VIOLATION:3499 | 45.00 | 896.0 |
| 1 | TIMEOUT | 0/1 | — | 1.500 | 1.150 | — | 0.778 | LAG_VIOLATION:3499 | 45.00 | 885.2 |
| 2 | TIMEOUT | 0/1 | — | 1.500 | 1.150 | — | 0.778 | LAG_VIOLATION:3499 | 45.00 | 814.0 |
| 3 | TIMEOUT | 0/1 | — | 1.500 | 1.150 | — | 0.778 | LAG_VIOLATION:3499 | 45.00 | 807.2 |
| 4 | TIMEOUT | 0/1 | — | 1.500 | 1.150 | — | 0.778 | LAG_VIOLATION:3499 | 45.00 | 759.3 |
| 5 | TIMEOUT | 0/1 | — | 1.500 | 1.150 | — | 0.778 | LAG_VIOLATION:3499 | 45.00 | 760.9 |
| 6 | TIMEOUT | 0/1 | — | 1.500 | 1.150 | — | 0.778 | LAG_VIOLATION:3499 | 45.00 | 785.2 |
| 7 | TIMEOUT | 0/1 | — | 1.500 | 1.150 | — | 0.778 | LAG_VIOLATION:3499 | 45.00 | 786.0 |
| 8 | TIMEOUT | 0/1 | — | 1.500 | 1.150 | — | 0.778 | LAG_VIOLATION:3499 | 45.00 | 776.8 |
| 9 | TIMEOUT | 0/1 | — | 1.500 | 1.150 | — | 0.778 | LAG_VIOLATION:3499 | 45.00 | 774.1 |

拒绝码合计（所有 seed、一 tick 多码各计一次）：

| 拒绝码 | 次数 | 占直方图合计 |
|---|---|---|
| `LAG_VIOLATION` | 34990 | 1.000 |

### crossing-pillars @ crossing + 影子盾开

层级 L0，策略 `crossing`，影子盾开，commit `f5985e4335fcc00a4e19c861dcd1a05e0113f797`。到达 30/30，碰撞 0。场景卡哈希 `1d39afbd780740b9e925569a70b27b670d73184d9b487d3adb2473fdf1ed279e`。

| seed | 结果 | 到达 | 到达时间 s | 最小障碍距离 m | 最小净空 m | 最小机间距离 m | 影子拒绝率 | 主要拒绝码 | 仿真 s | 墙钟 s |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | SUCCESS | 3/3 | 7.06 | 1.012 | 0.662 | 7.284 | 1.000 | ENV:UNKNOWN:2343, TRANSITION:MOTION_KINEMATICS:1350, TRANSITION:BRANCH_UNREACHABLE:1125 | 8.06 | 19.2 |
| 1 | SUCCESS | 3/3 | 7.06 | 1.016 | 0.666 | 7.281 | 1.000 | ENV:UNKNOWN:2343, TRANSITION:MOTION_KINEMATICS:1350, TRANSITION:BRANCH_UNREACHABLE:1125 | 8.06 | 19.0 |
| 2 | SUCCESS | 3/3 | 7.06 | 1.361 | 1.011 | 7.307 | 1.000 | ENV:UNKNOWN:2343, TRANSITION:MOTION_KINEMATICS:1350, TRANSITION:BRANCH_UNREACHABLE:1125 | 8.06 | 18.0 |
| 3 | SUCCESS | 3/3 | 7.76 | 1.265 | 0.915 | 3.506 | 1.000 | ENV:UNKNOWN:2553, TRANSITION:MOTION_KINEMATICS:1425, TRANSITION:HEAD_DISCONTINUITY:1128 | 8.76 | 19.3 |
| 4 | SUCCESS | 3/3 | 7.05 | 1.264 | 0.914 | 3.491 | 1.000 | ENV:UNKNOWN:2340, TRANSITION:MOTION_KINEMATICS:1350, TRANSITION:BRANCH_UNREACHABLE:1125 | 8.05 | 18.1 |
| 5 | SUCCESS | 3/3 | 7.77 | 1.266 | 0.916 | 3.350 | 1.000 | ENV:UNKNOWN:2556, TRANSITION:MOTION_KINEMATICS:1425, TRANSITION:HEAD_DISCONTINUITY:1131 | 8.77 | 19.4 |
| 6 | SUCCESS | 3/3 | 7.07 | 1.178 | 0.828 | 3.646 | 1.000 | ENV:UNKNOWN:2346, TRANSITION:MOTION_KINEMATICS:1350, TRANSITION:BRANCH_UNREACHABLE:1125 | 8.07 | 18.0 |
| 7 | SUCCESS | 3/3 | 7.76 | 0.878 | 0.528 | 3.542 | 1.000 | ENV:UNKNOWN:2553, TRANSITION:MOTION_KINEMATICS:1425, TRANSITION:HEAD_DISCONTINUITY:1128 | 8.76 | 19.1 |
| 8 | SUCCESS | 3/3 | 7.06 | 1.456 | 1.106 | 7.510 | 1.000 | ENV:UNKNOWN:2343, TRANSITION:MOTION_KINEMATICS:1350, TRANSITION:BRANCH_UNREACHABLE:1125 | 8.06 | 18.0 |
| 9 | SUCCESS | 3/3 | 7.05 | 1.244 | 0.894 | 7.381 | 1.000 | ENV:UNKNOWN:2340, TRANSITION:MOTION_KINEMATICS:1350, TRANSITION:BRANCH_UNREACHABLE:1125 | 8.05 | 17.9 |

拒绝码合计（所有 seed、一 tick 多码各计一次）：

| 拒绝码 | 次数 | 占直方图合计 |
|---|---|---|
| `ENV:UNKNOWN` | 24060 | 0.400 |
| `TRANSITION:MOTION_KINEMATICS` | 13725 | 0.228 |
| `TRANSITION:BRANCH_UNREACHABLE` | 11250 | 0.187 |
| `TRANSITION:HEAD_DISCONTINUITY` | 10335 | 0.172 |
| `TIME:SWITCH_NOT_IN_FUTURE` | 750 | 0.012 |

### crossing-forest medium @ crossing + 影子盾开

层级 L0，策略 `crossing`，影子盾开，commit `f5985e4335fcc00a4e19c861dcd1a05e0113f797`。到达 30/30，碰撞 0。场景卡哈希 `e66b39061158c23e4c8e8c3fd53bae40c799b3dabef6ef32dad0b94eb265fa0c`。

| seed | 结果 | 到达 | 到达时间 s | 最小障碍距离 m | 最小净空 m | 最小机间距离 m | 影子拒绝率 | 主要拒绝码 | 仿真 s | 墙钟 s |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | SUCCESS | 3/3 | 23.32 | 0.666 | 0.316 | 7.300 | 1.000 | ENV:UNKNOWN:7221, TRANSITION:HEAD_DISCONTINUITY:3096, TRANSITION:MOTION_KINEMATICS:3000 | 24.32 | 94.2 |
| 1 | SUCCESS | 3/3 | 23.59 | 0.615 | 0.265 | 7.388 | 1.000 | ENV:UNKNOWN:7302, TRANSITION:HEAD_DISCONTINUITY:3852, TRANSITION:MOTION_KINEMATICS:2325 | 24.59 | 96.2 |
| 2 | SUCCESS | 3/3 | 20.77 | 0.610 | 0.260 | 7.170 | 1.000 | ENV:UNKNOWN:6456, TRANSITION:HEAD_DISCONTINUITY:3081, TRANSITION:MOTION_KINEMATICS:2400 | 21.77 | 85.3 |
| 3 | SUCCESS | 3/3 | 23.63 | 0.795 | 0.445 | 6.622 | 1.000 | ENV:UNKNOWN:7314, TRANSITION:HEAD_DISCONTINUITY:3339, TRANSITION:MOTION_KINEMATICS:3075 | 24.63 | 95.3 |
| 4 | SUCCESS | 3/3 | 24.12 | 0.654 | 0.304 | 7.356 | 1.000 | ENV:UNKNOWN:7461, TRANSITION:HEAD_DISCONTINUITY:3336, TRANSITION:MOTION_KINEMATICS:3000 | 25.12 | 98.4 |
| 5 | SUCCESS | 3/3 | 22.13 | 0.800 | 0.450 | 4.602 | 1.000 | ENV:UNKNOWN:6864, TRANSITION:HEAD_DISCONTINUITY:3189, TRANSITION:MOTION_KINEMATICS:2625 | 23.13 | 90.9 |
| 6 | SUCCESS | 3/3 | 22.61 | 0.629 | 0.279 | 7.409 | 1.000 | ENV:UNKNOWN:7008, TRANSITION:HEAD_DISCONTINUITY:3333, TRANSITION:MOTION_KINEMATICS:2700 | 23.61 | 97.6 |
| 7 | SUCCESS | 3/3 | 24.12 | 0.783 | 0.433 | 7.469 | 1.000 | ENV:UNKNOWN:7461, TRANSITION:HEAD_DISCONTINUITY:3336, TRANSITION:MOTION_KINEMATICS:3150 | 25.12 | 102.7 |
| 8 | SUCCESS | 3/3 | 24.42 | 0.654 | 0.304 | 7.232 | 1.000 | ENV:UNKNOWN:7551, TRANSITION:HEAD_DISCONTINUITY:3726, TRANSITION:MOTION_KINEMATICS:2700 | 25.42 | 103.1 |
| 9 | SUCCESS | 3/3 | 23.64 | 0.734 | 0.384 | 7.157 | 1.000 | ENV:UNKNOWN:7317, TRANSITION:HEAD_DISCONTINUITY:3642, TRANSITION:MOTION_KINEMATICS:2625 | 24.64 | 99.9 |

拒绝码合计（所有 seed、一 tick 多码各计一次）：

| 拒绝码 | 次数 | 占直方图合计 |
|---|---|---|
| `ENV:UNKNOWN` | 71955 | 0.414 |
| `TRANSITION:HEAD_DISCONTINUITY` | 33930 | 0.195 |
| `TRANSITION:MOTION_KINEMATICS` | 27600 | 0.159 |
| `TRANSITION:BRANCH_UNREACHABLE` | 24900 | 0.143 |
| `TRANSITION:BRANCH_OUTSIDE_PREFIX` | 8175 | 0.047 |
| `TRACKING_BOUND` | 6478 | 0.037 |
| `TIME:SWITCH_NOT_IN_FUTURE` | 750 | 0.004 |

### crossing-forest sparse @ crossing + 影子盾开

层级 L0，策略 `crossing`，影子盾开，commit `f5985e4335fcc00a4e19c861dcd1a05e0113f797`。到达 30/30，碰撞 0。场景卡哈希 `dc619ccd0e64a92973f894e9003a595701e8a765d3a090f15a5c52c829cb0966`。

| seed | 结果 | 到达 | 到达时间 s | 最小障碍距离 m | 最小净空 m | 最小机间距离 m | 影子拒绝率 | 主要拒绝码 | 仿真 s | 墙钟 s |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | SUCCESS | 3/3 | 24.91 | 1.056 | 0.706 | 5.418 | 1.000 | ENV:UNKNOWN:7698, TRANSITION:HEAD_DISCONTINUITY:4098, TRANSITION:MOTION_KINEMATICS:2550 | 25.91 | 80.9 |
| 1 | SUCCESS | 3/3 | 24.09 | 1.069 | 0.719 | 5.099 | 1.000 | ENV:UNKNOWN:7452, TRANSITION:HEAD_DISCONTINUITY:3552, TRANSITION:MOTION_KINEMATICS:2775 | 25.09 | 76.7 |
| 2 | SUCCESS | 3/3 | 23.54 | 0.884 | 0.534 | 5.803 | 1.000 | ENV:UNKNOWN:7287, TRANSITION:HEAD_DISCONTINUITY:3687, TRANSITION:MOTION_KINEMATICS:2550 | 24.54 | 81.4 |
| 3 | SUCCESS | 3/3 | 22.11 | 0.818 | 0.468 | 5.426 | 1.000 | ENV:UNKNOWN:6858, TRANSITION:HEAD_DISCONTINUITY:3033, TRANSITION:MOTION_KINEMATICS:2850 | 23.11 | 77.1 |
| 4 | SUCCESS | 3/3 | 22.09 | 1.335 | 0.985 | 7.554 | 1.000 | ENV:UNKNOWN:6852, TRANSITION:HEAD_DISCONTINUITY:3252, TRANSITION:MOTION_KINEMATICS:2550 | 23.09 | 74.8 |
| 5 | SUCCESS | 3/3 | 23.63 | 1.259 | 0.909 | 7.299 | 1.000 | ENV:UNKNOWN:7314, TRANSITION:HEAD_DISCONTINUITY:3564, TRANSITION:MOTION_KINEMATICS:2700 | 24.63 | 79.2 |
| 6 | SUCCESS | 3/3 | 23.61 | 0.891 | 0.541 | 5.338 | 1.000 | ENV:UNKNOWN:7308, TRANSITION:HEAD_DISCONTINUITY:3483, TRANSITION:MOTION_KINEMATICS:2625 | 24.61 | 75.2 |
| 7 | SUCCESS | 3/3 | 22.09 | 0.890 | 0.540 | 5.003 | 1.000 | ENV:UNKNOWN:6852, TRANSITION:MOTION_KINEMATICS:3000, TRANSITION:HEAD_DISCONTINUITY:2727 | 23.09 | 70.9 |
| 8 | SUCCESS | 3/3 | 22.11 | 1.157 | 0.807 | 5.313 | 1.000 | ENV:UNKNOWN:6858, TRANSITION:HEAD_DISCONTINUITY:3033, TRANSITION:MOTION_KINEMATICS:2700 | 23.11 | 77.3 |
| 9 | SUCCESS | 3/3 | 26.80 | 1.175 | 0.825 | 4.312 | 1.000 | ENV:UNKNOWN:8265, TRANSITION:HEAD_DISCONTINUITY:4365, TRANSITION:MOTION_KINEMATICS:2775 | 27.80 | 92.1 |

拒绝码合计（所有 seed、一 tick 多码各计一次）：

| 拒绝码 | 次数 | 占直方图合计 |
|---|---|---|
| `ENV:UNKNOWN` | 72744 | 0.417 |
| `TRANSITION:HEAD_DISCONTINUITY` | 34794 | 0.200 |
| `TRANSITION:MOTION_KINEMATICS` | 27075 | 0.155 |
| `TRANSITION:BRANCH_UNREACHABLE` | 24675 | 0.142 |
| `TRANSITION:BRANCH_OUTSIDE_PREFIX` | 8625 | 0.049 |
| `TRACKING_BOUND` | 5678 | 0.033 |
| `TIME:SWITCH_NOT_IN_FUTURE` | 750 | 0.004 |

### crossing-forest dense @ crossing + 影子盾开

层级 L0，策略 `crossing`，影子盾开，commit `f5985e4335fcc00a4e19c861dcd1a05e0113f797`。到达 3/30，碰撞 9。场景卡哈希 `f1f6cd9a2c7938a5cedd9ecfcb270d7017145c7b820b9498c2d1608fc8e61e9d`。

| seed | 结果 | 到达 | 到达时间 s | 最小障碍距离 m | 最小净空 m | 最小机间距离 m | 影子拒绝率 | 主要拒绝码 | 仿真 s | 墙钟 s |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | COLLISION | 0/3 | — | 0.335 | -0.015 | 4.688 | 1.000 | ENV:UNKNOWN:1720, TRANSITION:BRANCH_UNREACHABLE:835, TRANSITION:MOTION_KINEMATICS:835 | 6.35 | 39.0 |
| 1 | COLLISION | 0/3 | — | 0.323 | -0.027 | 5.472 | 1.000 | ENV:UNKNOWN:2707, TRANSITION:BRANCH_UNREACHABLE:1466, TRANSITION:MOTION_KINEMATICS:1466 | 9.41 | 54.9 |
| 2 | COLLISION | 0/3 | — | 0.335 | -0.015 | 5.358 | 1.000 | ENV:UNKNOWN:2707, TRANSITION:BRANCH_UNREACHABLE:1391, TRANSITION:MOTION_KINEMATICS:1391 | 9.41 | 52.7 |
| 3 | COLLISION | 0/3 | — | 0.336 | -0.014 | 5.422 | 1.000 | ENV:UNKNOWN:2494, TRANSITION:BRANCH_UNREACHABLE:1144, TRANSITION:MOTION_KINEMATICS:1144 | 8.72 | 49.1 |
| 4 | COLLISION | 0/3 | — | 0.324 | -0.026 | 5.510 | 1.000 | ENV:UNKNOWN:2709, TRANSITION:BRANCH_UNREACHABLE:1317, TRANSITION:MOTION_KINEMATICS:1317 | 9.42 | 53.9 |
| 5 | SUCCESS | 3/3 | 26.81 | 0.637 | 0.287 | 5.304 | 1.000 | ENV:UNKNOWN:8268, TRANSITION:HEAD_DISCONTINUITY:4593, TRANSITION:MOTION_KINEMATICS:2475 | 27.81 | 171.8 |
| 6 | COLLISION | 0/3 | — | 0.341 | -0.009 | 4.721 | 1.000 | ENV:UNKNOWN:1716, TRANSITION:BRANCH_UNREACHABLE:833, TRANSITION:MOTION_KINEMATICS:833 | 6.33 | 38.3 |
| 7 | COLLISION | 0/3 | — | 0.335 | -0.015 | 5.365 | 1.000 | ENV:UNKNOWN:2711, TRANSITION:BRANCH_UNREACHABLE:1243, TRANSITION:MOTION_KINEMATICS:1243 | 9.43 | 59.7 |
| 8 | COLLISION | 0/3 | — | 0.344 | -0.006 | 5.431 | 1.000 | ENV:UNKNOWN:2709, TRANSITION:MOTION_KINEMATICS:1259, TRANSITION:BRANCH_UNREACHABLE:1184 | 9.42 | 65.0 |
| 9 | COLLISION | 0/3 | — | 0.341 | -0.009 | 4.812 | 1.000 | ENV:UNKNOWN:1688, TRANSITION:BRANCH_UNREACHABLE:900, TRANSITION:MOTION_KINEMATICS:900 | 6.19 | 39.6 |

拒绝码合计（所有 seed、一 tick 多码各计一次）：

| 拒绝码 | 次数 | 占直方图合计 |
|---|---|---|
| `ENV:UNKNOWN` | 29429 | 0.395 |
| `TRANSITION:MOTION_KINEMATICS` | 12863 | 0.172 |
| `TRANSITION:BRANCH_UNREACHABLE` | 12638 | 0.169 |
| `TRANSITION:HEAD_DISCONTINUITY` | 10950 | 0.147 |
| `TRANSITION:BRANCH_OUTSIDE_PREFIX` | 3934 | 0.053 |
| `TRACKING_BOUND` | 3449 | 0.046 |
| `TIME:SWITCH_NOT_IN_FUTURE` | 750 | 0.010 |
| `ENV:OCCUPIED` | 568 | 0.008 |

### formation-hold @ formation + 影子盾开

层级 L0，策略 `formation`，影子盾开，commit `f5985e4335fcc00a4e19c861dcd1a05e0113f797`。到达 9/9，碰撞 0。场景卡哈希 `1b1894f5864a2f5f5fb0e06b7d44ec490acf8d012957829f0f599e19951c365e`。 场景卡只有 seeds 0–2。

| seed | 结果 | 到达 | 到达时间 s | 最小障碍距离 m | 最小净空 m | 最小机间距离 m | 影子拒绝率 | 主要拒绝码 | 仿真 s | 墙钟 s |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | SUCCESS | 3/3 | 18.63 | 1.500 | 1.150 | 1.987 | 1.000 | ENV:UNKNOWN:5724, TIME:SWITCH_NOT_IN_FUTURE:165 | 19.63 | 5.4 |
| 1 | SUCCESS | 3/3 | 18.63 | 1.500 | 1.150 | 1.987 | 1.000 | ENV:UNKNOWN:5724, TIME:SWITCH_NOT_IN_FUTURE:165 | 19.63 | 5.5 |
| 2 | SUCCESS | 3/3 | 18.63 | 1.500 | 1.150 | 1.987 | 1.000 | ENV:UNKNOWN:5724, TIME:SWITCH_NOT_IN_FUTURE:165 | 19.63 | 4.8 |

拒绝码合计（所有 seed、一 tick 多码各计一次）：

| 拒绝码 | 次数 | 占直方图合计 |
|---|---|---|
| `ENV:UNKNOWN` | 17172 | 0.972 |
| `TIME:SWITCH_NOT_IN_FUTURE` | 495 | 0.028 |

### crossing-6-forest medium @ crossing + 影子盾开

层级 L0，策略 `crossing`，影子盾开，commit `f5985e4335fcc00a4e19c861dcd1a05e0113f797`。到达 6/6，碰撞 0。场景卡哈希 `657efeb6fae1a62ddc1b061d4cf04a3013f660d2e57b28f70a42d5c23c1db9ee`。

| seed | 结果 | 到达 | 到达时间 s | 最小障碍距离 m | 最小净空 m | 最小机间距离 m | 影子拒绝率 | 主要拒绝码 | 仿真 s | 墙钟 s |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | SUCCESS | 6/6 | 26.08 | 0.656 | 0.306 | 1.293 | 1.000 | ENV:UNKNOWN:15723, TRANSITION:HEAD_DISCONTINUITY:10323, TRANSITION:MOTION_KINEMATICS:4950 | 27.08 | 234.2 |

拒绝码合计（所有 seed、一 tick 多码各计一次）：

| 拒绝码 | 次数 | 占直方图合计 |
|---|---|---|
| `ENV:UNKNOWN` | 15723 | 0.460 |
| `TRANSITION:HEAD_DISCONTINUITY` | 10323 | 0.302 |
| `TRANSITION:MOTION_KINEMATICS` | 4950 | 0.145 |
| `TRANSITION:BRANCH_UNREACHABLE` | 1950 | 0.057 |
| `TRACKING_BOUND` | 829 | 0.024 |
| `TIME:SWITCH_NOT_IN_FUTURE` | 375 | 0.011 |

### crossing-6-forest-sparse @ crossing + 影子盾开

层级 L0，策略 `crossing`，影子盾开，commit `f5985e4335fcc00a4e19c861dcd1a05e0113f797`。到达 6/6，碰撞 0。场景卡哈希 `7e3328908444d4615c34e217fbb871fec2bed1e2fcb451f8738c13ba2b24762a`。

| seed | 结果 | 到达 | 到达时间 s | 最小障碍距离 m | 最小净空 m | 最小机间距离 m | 影子拒绝率 | 主要拒绝码 | 仿真 s | 墙钟 s |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | SUCCESS | 6/6 | 25.40 | 0.665 | 0.315 | 0.959 | 1.000 | ENV:UNKNOWN:15465, TRANSITION:HEAD_DISCONTINUITY:9915, TRANSITION:MOTION_KINEMATICS:4950 | 26.40 | 176.0 |

拒绝码合计（所有 seed、一 tick 多码各计一次）：

| 拒绝码 | 次数 | 占直方图合计 |
|---|---|---|
| `ENV:UNKNOWN` | 15465 | 0.458 |
| `TRANSITION:HEAD_DISCONTINUITY` | 9915 | 0.293 |
| `TRANSITION:MOTION_KINEMATICS` | 4950 | 0.146 |
| `TRANSITION:BRANCH_UNREACHABLE` | 1800 | 0.053 |
| `TRACKING_BOUND` | 834 | 0.025 |
| `TRANSITION:BRANCH_OUTSIDE_PREFIX` | 450 | 0.013 |
| `TIME:SWITCH_NOT_IN_FUTURE` | 375 | 0.011 |

## 说明

1. **没有影子盾拦截模式。** `--shadow off` 只是不写拒绝字段；`--shadow on` 不改动作。要把现行门变成硬拦，只能改用 `certified` 策略。
2. **certified 只覆盖单机。** `yopo_l0.policy.POLICIES` 里的 `certified` 走 vendored `UavRuntime` + `SafetyGate`。多机 crossing / formation / switch 在运行器里走编队协议和上游 YOPO 提案，结果里的 `policy` 是任务名，不是 CLI 的 `--policy`。
3. **g2-pillar @ certified 到不了。** 硬门在 ENV 证明处把候选全部拦下，计划码是 `NO_ENV_SAFE_CANDIDATE` 和 `cand:ENV:UNKNOWN`，飞机不离开起点（最小障碍距离一直是离地 1.500 m）。这是硬门行为（ADR 0010）。同一几何的 upstream 10/10 到达、碰撞 0；影子直方图里也会记 `ENV:UNKNOWN`，但不改动作。certified 开着影子盾时，直方图主要是 `LAG_VIOLATION`：飞机停在起点，oracle 落后界被触发，这是记录，不是第二道硬拦。
4. **拒绝率高并不表示飞行失败。** L0 上 `ENV:UNKNOWN`、`TIME:SWITCH_NOT_IN_FUTURE` 等是影子记录，冻结表把影子拒绝率列为只报数（ADR 0016）。

## 裁剪

- 六机林带 medium / sparse 只跑了 seed 0（任务要求至少 seed 0）。单次墙钟 234.2 s / 176.0 s，超过 60 s，按 suite 惯例没有加跑 seeds 1–9。
- formation-hold 场景卡只有 seeds 0–2，三个都跑了。
- 没有再做各卡全时长的影子盾开/关对照。`tests/test_shadow.py` 8 项通过，代表卡 seed 0 的短时动作序列逐位一致。
- 其余代表卡（含 g2-pillar @ certified 的 10 个 seed、三机林带疏/中/密各 10 个 seed）都按计划跑完，没有再裁。

## 怎么重跑

`out/` 在 `.gitignore` 里。脚本把批跑写到 `out/12e/`，再把汇总 JSON 和本页写进 `docs/results/`。

```bash
make venv
git rev-parse HEAD
.venv/bin/python scripts/12e-upstream-vs-shield.py --jobs 2
```

常用旗标：`--certified-seeds 0` 只跑 certified 的 seed 0；`--six-seeds 0,1` 给六机林带加 seed；`--skip-certified` 跳过 certified；`--report-only` 只根据已有 `out/12e/` 重写本页。

结果记录里的 `commit` 应与本页出处一致（commit 随 HEAD 变）。墙钟列会随机器变。本页对应 `f5985e4335fc`。
