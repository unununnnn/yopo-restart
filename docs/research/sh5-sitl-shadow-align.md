# 票 SH5：SITL 重测影子对齐开关（只报数）

只报数。report-only。判据未冻结。不改 `core/`、`scenes/suite.txt`、已冻门、安全清单和 PX4 参数。影子盾仍然只记录，不改设定点。三个研究开关 `shadow_env_evidence`、`shadow_contract_align`、`shadow_tracking_align` 默认仍关；打开的结果标「偏离上游」。不放宽 `FlightOracle.tracking_bound`（0.15 m）。墙钟不进结论。

数字在 [sh5-sitl-shadow-align-runs.json](sh5-sitl-shadow-align-runs.json)。

## 结论

在隔离网络的锁步 SITL `crossing-pillars-sitl80` seeds 0–4 上，SH3 / SH4 的对齐开关效果与 L0 判断一致：缺省臂 `would_reject` 是 1.000，环境拒绝率 0.9046；只开环境证据后环境拒绝率降到 0.0144，`would_reject` 仍是 0.9931；再加上契约对齐，`would_reject` 降到 0.0519，直方图里不再出现 `TRACKING_BOUND`；三个开关一起开时，0.15 m 超界回到码表（7861 次），但 `would_reject` 只有 0.0203，留下的是 `TRANSITION:MOTION_KINEMATICS` 和少量 `ENV:OCCUPIED`。0.15 m 阈值没有改。开关默认仍关。

到达和碰撞在同一张卡、同一个种子的不同趟之间仍会翻，不能写成开关的效果。缺省 3/5（9/15 到达，碰撞 2），开证据 2/5（6/15，碰撞 3），契约对齐 3/5（9/15，碰撞 2），再加跟踪对齐 3/5（11/15，碰撞 1）。这和票 SH2、B1 以后锁步交叉卡「同 seed 会翻」是同一类噪声。

时间允许时补跑了 `g2-pillar-sitl80` seeds 0–4（单机、policy `upstream`）。四臂都是 5/5 到达、碰撞 0。缺省环境拒绝率 1.000，开证据后是 0；`would_reject` 在缺省和只开证据时仍是 1.000，契约对齐后降到 0.0125（只剩 `LAG_VIOLATION` 21 次，`TRACKING_BOUND` 从码表拿掉），三个开关一起开时 0.15 m 超界回到直方图（1115 次），`would_reject` 是 0.0655，全部来自 `LAG_VIOLATION`（108 次），跟踪超界没有计入会拒。沿路径落后超过 1.5 m 按票 SH4 不是误报。两臂 `LAG_VIOLATION` 次数不同，是锁步同 seed 轨迹起伏，不是跟踪开关注入了新的拒绝。

本票修了一处只影响报告的归因：`shadow_sitl.attribution` 在 tick 已经带 `would_reject` 时，不再用「有拒绝码就拒」把仍记在码表里的 `TRACKING_BOUND` 打回去。`result.shadow_reject_rate`（`shadow.aggregate`）本来就只看 `tick.would_reject`。默认关时两条路径数字相同，行为不变。

`px4_parameters_written` 全程是空列表。锁步 `clock_ahead` 都是 0。每趟都走官方启动脚本自建的 `yopo-sitl-…` 隔离网。

## 方法

入口：

```bash
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/crossing-pillars-sitl80.json --policy crossing --seed 0 --lockstep --fleet-shadow
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/crossing-pillars-sitl80.json --policy crossing --seed 0 --lockstep --fleet-shadow --shadow-env-evidence
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/crossing-pillars-sitl80.json --policy crossing --seed 0 --lockstep --fleet-shadow --shadow-env-evidence --shadow-contract-align
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/crossing-pillars-sitl80.json --policy crossing --seed 0 --lockstep --fleet-shadow --shadow-env-evidence --shadow-contract-align --shadow-tracking-align
```

镜像 `yopo-restart-sitl:yopo-81d084c04186`。四臂都开 `fleet_shadow`，以便记下影子归因（与票 SH2 关臂相同）。`fleet_shadow` 本身不写 `upstream_deviation`。三个研究开关按臂打开，打开即偏离上游：

| 臂 | `shadow_env_evidence` | `shadow_contract_align` | `shadow_tracking_align` |
|---|---|---|---|
| default | 关 | 关 | 关 |
| env | 开 | 关 | 关 |
| env_contract | 开 | 开 | 关 |
| env_contract_tracking | 开 | 开 | 开 |

打开契约对齐时，仍会从返回码里拿掉 `TRACKING_BOUND`（保持票 SH3 的行为）。再打开跟踪对齐后，0.15 m 超界留在直方图里，但不单独打满 `would_reject`。阈值本身不改。真正的贴柱仍走 `ENV:OCCUPIED`，沿路径落后仍走 `LAG_VIOLATION`。

## 锁步 SITL：crossing-pillars-sitl80 seeds 0–4

### 各臂合计

| 臂 | 成功趟 | 到达 | 碰撞 | tick | would_reject | env 拒绝率 | `TRACKING_BOUND` | 开臂主要剩余码 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| default | 3/5 | 9/15 | 2 | 7086 | 1.0000 | 0.9046 | 4542 | `ENV:UNKNOWN` 6372，`TRACKING_BOUND` 4542，`TRANSITION:HEAD_DISCONTINUITY` 4100，`TRANSITION:MOTION_KINEMATICS` 2796，`TRANSITION:BRANCH_UNREACHABLE` 2080，`ENV:NO_FRAME` 190，`TIME:SWITCH_NOT_IN_FUTURE` 190，`ENV:OCCUPIED` 38 |
| env | 2/5 | 6/15 | 3 | 5226 | 0.9931 | 0.0144 | 3363 | `TRACKING_BOUND` 3363，`TRANSITION:HEAD_DISCONTINUITY` 2702，`TRANSITION:MOTION_KINEMATICS` 2296，`TRANSITION:BRANCH_UNREACHABLE` 1618，`TIME:SWITCH_NOT_IN_FUTURE` 190，`ENV:OCCUPIED` 37，`ENV:UNKNOWN` 38 |
| env_contract | 3/5 | 9/15 | 2 | 8649 | 0.0519 | 0.0129 | 0 | `TRANSITION:MOTION_KINEMATICS` 337，`ENV:OCCUPIED` 74，`ENV:UNKNOWN` 38 |
| env_contract_tracking | 3/5 | 11/15 | 1 | 16603 | 0.0203 | 0.0045 | 7861 | `TRACKING_BOUND` 7861，`TRANSITION:MOTION_KINEMATICS` 262，`ENV:OCCUPIED` 75 |

开证据后环境拒绝率与票 SH2b 开臂的 0.013 同量级（本票 0.0144）。再开契约对齐，`would_reject` 从约 0.99 降到 0.0519，和 L0 上 SH3 把契约误报拿掉之后的方向一致；SITL 留下的运动学码和占用比 L0 crossing 的 0.0176 略高。跟踪对齐把 `TRACKING_BOUND` 还回直方图，但会拒率再降到 0.0203：7861 次 0.15 m 超界不再单独打满 `would_reject`。

跟踪对齐臂的 tick 合计偏多，主要是种子 3 以 `uav1:PLAN_TIMEOUT` 跑到 9001 tick（到达 2/3，碰撞 0），会拒 149 次全部是 `TRANSITION:MOTION_KINEMATICS`，同时记下 3739 次 `TRACKING_BOUND`。

### 逐趟

| seed | 臂 | 结果 | 到达 | 碰撞 | tick | would_reject | env 拒绝率 | `TRACKING_BOUND` | 直方图要点 |
|---:|---|---|---:|---:|---:|---:|---:|---:|---|
| 0 | default | SUCCESS | 3/3 | 0 | 2070 | 1.0000 | 0.9449 | 1346 | `ENV:UNKNOWN` 1956，契约码和跟踪码打满 |
| 0 | env | FAIL uav1:OBSTACLE | 0/3 | 1 | 285 | 1.0000 | 0.0000 | 148 | 短路程；环境码已清 |
| 0 | env_contract | FAIL uav0:OBSTACLE | 0/3 | 1 | 252 | 0.0000 | 0.0000 | 0 | 直方图空：契约误报拿掉后，撞柱前没有记下占用 |
| 0 | env_contract_tracking | SUCCESS | 3/3 | 0 | 2889 | 0.0523 | 0.0260 | 1514 | `ENV:OCCUPIED` 75，`MOTION_KINEMATICS` 76 |
| 1 | default | FAIL uav0:OBSTACLE | 0/3 | 1 | 252 | 1.0000 | 1.0000 | 146 | 短路程，`ENV:UNKNOWN` 打满 |
| 1 | env | SUCCESS | 3/3 | 0 | 2841 | 0.9873 | 0.0264 | 1739 | 环境码几乎清掉，契约码和跟踪码仍打满 |
| 1 | env_contract | SUCCESS | 3/3 | 0 | 2907 | 0.0643 | 0.0385 | 0 | `ENV:OCCUPIED` 74，`ENV:UNKNOWN` 38，`MOTION_KINEMATICS` 75 |
| 1 | env_contract_tracking | SUCCESS | 3/3 | 0 | 1563 | 0.0000 | 0.0000 | 1172 | 只剩 0.15 m 超界记录 |
| 2 | default | SUCCESS | 3/3 | 0 | 2898 | 1.0000 | 0.9224 | 1763 | `ENV:UNKNOWN` 2635，另有 38 次 `ENV:OCCUPIED` |
| 2 | env | FAIL uav0:OBSTACLE | 0/3 | 1 | 252 | 1.0000 | 0.0000 | 114 | 短路程；环境码已清 |
| 2 | env_contract | SUCCESS | 3/3 | 0 | 2568 | 0.0292 | 0.0000 | 0 | 只剩 `MOTION_KINEMATICS` 75 |
| 2 | env_contract_tracking | FAIL uav0:OBSTACLE | 0/3 | 1 | 249 | 0.0000 | 0.0000 | 108 | 短路程；只记跟踪超界，会拒为 0 |
| 3 | default | FAIL uav2:OBSTACLE | 0/3 | 1 | 333 | 1.0000 | 1.0000 | 192 | 短路程，`ENV:UNKNOWN` 打满 |
| 3 | env | FAIL uav0:OBSTACLE | 0/3 | 1 | 261 | 1.0000 | 0.0000 | 118 | 短路程；环境码已清 |
| 3 | env_contract | FAIL uav1:OBSTACLE | 0/3 | 1 | 285 | 0.0000 | 0.0000 | 0 | 直方图空，同种子 0 契约臂 |
| 3 | env_contract_tracking | FAIL uav1:PLAN_TIMEOUT | 2/3 | 0 | 9001 | 0.0166 | 0.0000 | 3739 | `MOTION_KINEMATICS` 149；跟踪超界未计入会拒 |
| 4 | default | SUCCESS | 3/3 | 0 | 1533 | 1.0000 | 0.7802 | 1095 | `ENV:UNKNOWN` 1196 |
| 4 | env | SUCCESS | 3/3 | 0 | 1587 | 1.0000 | 0.0000 | 1244 | 环境码 0，契约码和跟踪码打满 |
| 4 | env_contract | SUCCESS | 3/3 | 0 | 2637 | 0.0709 | 0.0000 | 0 | 只剩 `MOTION_KINEMATICS` 187 |
| 4 | env_contract_tracking | SUCCESS | 3/3 | 0 | 2901 | 0.0128 | 0.0000 | 1328 | `MOTION_KINEMATICS` 37 |

短路程失败趟（大约 250–330 tick）在契约对齐之后，直方图经常是空的，或只剩跟踪超界且会拒为 0：影子在这些 tick 上没有记到 `ENV:OCCUPIED`，撞柱是 PX4 跟踪和规划滞后，不是本票要改的设定点。这不能读成「对齐开关放过了穿柱」。成功飞完的趟里，穿柱或超上游限幅仍然留下 `ENV:OCCUPIED` 或 `TRANSITION:MOTION_KINEMATICS`，并计入 `would_reject`。


## 锁步 SITL：g2-pillar-sitl80 seeds 0–4

单机，policy `upstream`。镜像、隔离网络和「不写 PX4 参数」与交叉卡相同。四臂都开 `fleet_shadow`。全部 5/5 成功到达，碰撞 0，`clock_ahead` 都是 0，`px4_parameters_written` 都是空列表。

### 各臂合计

| 臂 | 成功趟 | 到达 | 碰撞 | tick | would_reject | env 拒绝率 | `TRACKING_BOUND` | 开臂主要剩余码 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| default | 5/5 | 5/5 | 0 | 1658 | 1.0000 | 1.0000 | 1135 | `ENV:UNKNOWN` 1658，`TIME:SWITCH_NOT_IN_FUTURE` 1123，`TRANSITION:HEAD_DISCONTINUITY` 1123，`TRACKING_BOUND` 1135，`TRANSITION:TERMINAL_NOT_HOLDABLE` 535，`LAG_VIOLATION` 50 |
| env | 5/5 | 5/5 | 0 | 1670 | 1.0000 | 0.0000 | 1199 | `TIME:SWITCH_NOT_IN_FUTURE` 1140，`TRANSITION:HEAD_DISCONTINUITY` 1140，`TRACKING_BOUND` 1199，`TRANSITION:TERMINAL_NOT_HOLDABLE` 530，`LAG_VIOLATION` 53 |
| env_contract | 5/5 | 5/5 | 0 | 1684 | 0.0125 | 0.0000 | 0 | `LAG_VIOLATION` 21 |
| env_contract_tracking | 5/5 | 5/5 | 0 | 1649 | 0.0655 | 0.0000 | 1115 | `LAG_VIOLATION` 108，`TRACKING_BOUND` 1115 |

开证据后环境码清零，与票 SH1 / SH3 的 L0 g2 一致。契约对齐把 `TIME` / `HEAD` / `TERMINAL_NOT_HOLDABLE` 和码表里的 `TRACKING_BOUND` 拿掉，会拒率从 1.000 降到 0.0125。再开跟踪对齐后，0.15 m 超界重新出现在直方图，但 `would_reject` 等于 `LAG_VIOLATION` 的票数（108 / 1649），没有把跟踪超界算进去。这张卡在 SITL 上留下的真拒绝是沿路径落后，不是 L0 上那 60 个 `MOTION_KINEMATICS`。

### 逐趟

| seed | 臂 | 结果 | would_reject | env 拒绝率 | `TRACKING_BOUND` | tick | 开臂直方图 |
|---:|---|---|---:|---:|---:|---:|---|
| 0 | default | SUCCESS 1/1 | 1.0000 | 1.0000 | 257 | 333 | 环境码和契约码打满 |
| 0 | env | SUCCESS 1/1 | 1.0000 | 0.0000 | 214 | 335 | 环境码 0 |
| 0 | env_contract | SUCCESS 1/1 | 0.0000 | 0.0000 | 0 | 339 | 空 |
| 0 | env_contract_tracking | SUCCESS 1/1 | 0.0871 | 0.0000 | 256 | 333 | `LAG_VIOLATION` 29，`TRACKING_BOUND` 256 |
| 1 | default | SUCCESS 1/1 | 1.0000 | 1.0000 | 229 | 327 | 环境码和契约码打满 |
| 1 | env | SUCCESS 1/1 | 1.0000 | 0.0000 | 231 | 336 | 另有 `LAG_VIOLATION` 19 |
| 1 | env_contract | SUCCESS 1/1 | 0.0303 | 0.0000 | 0 | 330 | `LAG_VIOLATION` 10 |
| 1 | env_contract_tracking | SUCCESS 1/1 | 0.0725 | 0.0000 | 221 | 331 | `LAG_VIOLATION` 24，`TRACKING_BOUND` 221 |
| 2 | default | SUCCESS 1/1 | 1.0000 | 1.0000 | 257 | 335 | 环境码和契约码打满 |
| 2 | env | SUCCESS 1/1 | 1.0000 | 0.0000 | 241 | 331 | 另有 `LAG_VIOLATION` 24 |
| 2 | env_contract | SUCCESS 1/1 | 0.0000 | 0.0000 | 0 | 334 | 空 |
| 2 | env_contract_tracking | SUCCESS 1/1 | 0.0826 | 0.0000 | 222 | 327 | `LAG_VIOLATION` 27，`TRACKING_BOUND` 222 |
| 3 | default | SUCCESS 1/1 | 1.0000 | 1.0000 | 223 | 333 | 另有 `LAG_VIOLATION` 25 |
| 3 | env | SUCCESS 1/1 | 1.0000 | 0.0000 | 285 | 337 | 环境码 0 |
| 3 | env_contract | SUCCESS 1/1 | 0.0000 | 0.0000 | 0 | 333 | 空 |
| 3 | env_contract_tracking | SUCCESS 1/1 | 0.0395 | 0.0000 | 171 | 329 | `LAG_VIOLATION` 13，`TRACKING_BOUND` 171 |
| 4 | default | SUCCESS 1/1 | 1.0000 | 1.0000 | 169 | 330 | 另有 `LAG_VIOLATION` 25 |
| 4 | env | SUCCESS 1/1 | 1.0000 | 0.0000 | 228 | 331 | 另有 `LAG_VIOLATION` 10 |
| 4 | env_contract | SUCCESS 1/1 | 0.0316 | 0.0000 | 0 | 348 | `LAG_VIOLATION` 11 |
| 4 | env_contract_tracking | SUCCESS 1/1 | 0.0456 | 0.0000 | 245 | 329 | `LAG_VIOLATION` 15，`TRACKING_BOUND` 245 |

## pytest

（全量回归数字待填。）新增 `tests/test_sitl_sh5_shadow_align.py`：三个研究开关默认关、0.15 m 阈值不变、官方启动脚本四臂 dry-run 与本票命令行一致、汇总行会算 `would_reject` / 环境拒绝率 / `TRACKING_BOUND` 次数；归因在跟踪对齐下尊重 `tick.would_reject`。
