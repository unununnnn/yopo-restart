# 票 SH3：影子盾契约码误报判定（只报数）

只报数。report-only。判据未冻结。不改 `core/`、`scenes/suite.txt`、已冻门、安全清单和 PX4 参数。影子盾仍然只记录，不改批准动作。研究开关 `shadow_contract_align` 默认关；打开的结果标「偏离上游」。不放宽认证阈值去盖掉真问题。墙钟不进结论。

数字在 [sh3-contract-codes-runs.json](sh3-contract-codes-runs.json)。对照实验两臂都打开 `shadow_env_evidence`，否则 `ENV:UNKNOWN` 会把 `would_reject` 打满，看不出契约修正的效果。场景证据本身也偏离上游，见票 SH1 / SH2。

## 结论

这些卡在 L0 上是 10/10 到达、碰撞 0，但影子盾的 `would_reject` 被契约码打满。根因是影子把已经在飞的上游多项式，按认证 `Plan` 的语义去包一层，而不是轨迹真的危险。

打开修正开关后，g2-pillar 的平均 `would_reject` 从 1.000 降到 0.0114，crossing-pillars 从 1.000 降到 0.0176。`TIME:SWITCH_NOT_IN_FUTURE`、`TRANSITION:HEAD_DISCONTINUITY`、`TRANSITION:TERMINAL_NOT_HOLDABLE`、因认证 `v_max=4` 造不出刹车的 `TRANSITION:BRANCH_UNREACHABLE` 都降到 0。留下的是按上游限幅（`v_max=6`、`a_max=6`）仍然超限的少量 `TRANSITION:MOTION_KINEMATICS`，以及 crossing 上个别种子的 `ENV:OCCUPIED`。对齐时 `a_max` 从认证默认 8 收到上游的 6，所以 g2-pillar 上会新出现 60 个运动学 tick：这是更严，不是放宽。

故意构造的限幅内穿柱轨迹，开关打开后仍被拒绝，码只有 `ENV:OCCUPIED`。

两臂到达时刻、碰撞、轨迹摘要逐粒相同。碰撞都是 0。L0 这两张卡没有 `TRACKING_BOUND`；该码的判定来自单元复现和票 SH2 的 SITL 计数。

## 判定

影子在 `yopo_l0/shadow.py` 的 `_build` 里用 `t_switch = traj.t_start`、分支点 `t_switch+{0,0.5}` 调用 `make_plan`，再用 `SafetyGate` 复查。认证限幅来自 `default_config`：`KinematicLimits(4.0, 8.0, 80.0)`（`core/src/yopo_swarm_core/uav.py`）。上游 YOPO 后端和这两张卡的 L0 动力学是 `v_max=6`、`a_max=6`，没有单独的 `j_max`。`j_max` 对齐后仍用认证默认 80，没有放宽。

| 码 | 判定 | 代码位置 | 最小复现 |
|---|---|---|---|
| `TIME:SWITCH_NOT_IN_FUTURE` | 误报 | `core/.../planner.py` `SafetyGate.validate`：`if not plan.t_switch > now` | `hover_plan(..., t0=0)` 在 `now=0` 交给安全门。认证规则是「提议的切换必须在未来」；影子是在复查已经在飞的运动，`t_switch` 等于轨迹起点，不会大于 `now`。 |
| `TRANSITION:HEAD_DISCONTINUITY` | 对父计划路径是误报；运动起点与声明头不一致仍是真问题 | `core/.../plan.py` `validate_plan`（运动起点 PVA）；`planner.py` `SafetyGate`（`plan.head_pva` 对父计划在 `t_switch` 的状态，容差 `CONTINUITY_TOL`） | 一条 `t_start=1`、从 `x=2` 出发的多项式，父计划是原点悬停，门给出该码，头状态差 2 m。影子把声明头取成轨迹起点，`validate_plan` 这条通常不过。安全门因 `TIME:SWITCH` 从未批准，父计划卡在初始悬停，中途计划的头对不上。对齐后改跟上一条运动比：连续的重规划不记，真跳跃仍记。 |
| `TRANSITION:TERMINAL_NOT_HOLDABLE` | 误报 | `core/.../plan.py` `validate_plan`：终点 `\|v\|` 或 `\|a\|` 大于 `HOLD_TOL`（`1e-6`） | 静止到 `x=3`、终点速度 1 m/s 的五次多项式。上游是滚动时域片段，并不保证在 `t_end` 收成悬停。g2-pillar seed 0 上终点残留速度大约 0.04–0.08 m/s，远大于 `1e-6`，也远不够危险。不改 `HOLD_TOL`。 |
| `TRANSITION:MOTION_KINEMATICS` | 只超过认证 `v_max=4`、未超过上游 `v_max=6` 的是误报；超过上游限幅是真问题 | `core/.../plan.py` `_kinematics_ok` ← `validate_plan` | 起点速度 5 m/s 的五次多项式：认证限幅下失败，上游限幅下通过。起点 7 m/s 的对齐后仍失败。crossing-pillars seed 0 峰值速度 5.40 m/s，超过 4、不超过 6。 |
| `TRANSITION:BRANCH_UNREACHABLE` | 只因认证 `v_max=4` 造不出刹车的是误报；上游限幅下仍造不出是真问题 | `core/.../plan.py` `make_plan`：`brake_branch(...) is None` | 与上一条同一条 5 m/s 轨迹：认证限幅下分支不可达，上游限幅下可刹。起点 20 m/s 的对齐后仍失败。crossing seed 0 第 20 次选择，认证刹车失败，上游限幅下两条分支都成功。 |
| `TRACKING_BOUND`（0.15 m） | 当作「计划危险」是误报：把认证闭环界套到未承诺该界的上游 / PX4 栈上。0.15 m 本身不改，超界仍是真实测量 | `yopo_l0/shadow.py` `record_oracle_track_lag`：`||actual-desired|| > FlightOracle.tracking_bound`（默认 0.15 m，`core/.../oracle.py`） | `actual` 与 `desired` 相距 0.16 m 即记该码。L0 这两张卡计数为 0。票 SH2 的锁步 SITL 上大量出现，部分种子仍成功到达。对齐后不再把它算进 `would_reject`，也不把阈值放宽。 |

## 开关

`RunConfig.shadow_contract_align` 默认 `False`。命令行 `--shadow-contract-align`。关时契约检查仍用认证默认限幅和 `SafetyGate`，结果里不加 `shadow_contract_align` 字段。

打开时只改影子记录，不改设定点：

- `v_max` / `a_max` 换成上游和 L0 动力学的值（默认都是 6）。`j_max` 仍是 80。
- 不把「切换必须在未来」套到已经在执行的轨迹上，也不拿从未获批的初始悬停做父计划 I8。
- 不要求滚动时域片段的终点可悬停。不改 `HOLD_TOL`。
- `TRACKING_BOUND` 仍按 0.15 m 计算，但不计入 `would_reject`。
- 跟上一条运动在 `t_switch` 处做连续性检查；超过上游限幅、真跳跃、`ENV:OCCUPIED` 仍拒绝。

## L0 对照（seeds 0–9）

两臂都开场景证据。打开对齐的一臂标偏离上游。轨迹摘要两臂相同。碰撞都是 0。`would_reject` 是会拒的 tick 占全部 tick 的比例。码计数是各种子 tick 次数之和。

### g2-pillar

场景卡哈希 `9d385f4cfce543922961865d06ceeedd44afdddb52c706bf6e3da9929431d8ae`。10/10 到达，碰撞 0。`would_reject` 平均 1.000 → 0.0114。

| seed | 到达 | would_reject 关 | would_reject 开 | 开臂直方图 |
|---:|---:|---:|---:|---|
| 0 | 1/1 | 1.000 | 0.019 | `MOTION_KINEMATICS` 10 |
| 1 | 1/1 | 1.000 | 0.019 | `MOTION_KINEMATICS` 10 |
| 2 | 1/1 | 1.000 | 0.019 | `MOTION_KINEMATICS` 10 |
| 3 | 1/1 | 1.000 | 0.019 | `MOTION_KINEMATICS` 10 |
| 4 | 1/1 | 1.000 | 0.019 | `MOTION_KINEMATICS` 10 |
| 5 | 1/1 | 1.000 | 0.000 | （空） |
| 6 | 1/1 | 1.000 | 0.000 | （空） |
| 7 | 1/1 | 1.000 | 0.000 | （空） |
| 8 | 1/1 | 1.000 | 0.000 | （空） |
| 9 | 1/1 | 1.000 | 0.019 | `MOTION_KINEMATICS` 10 |

| 码 | 关（10 种子合计） | 开 |
|---|---:|---:|
| `TIME:SWITCH_NOT_IN_FUTURE` | 3171 | 0 |
| `TRANSITION:HEAD_DISCONTINUITY` | 3171 | 0 |
| `TRANSITION:TERMINAL_NOT_HOLDABLE` | 2100 | 0 |
| `TRANSITION:MOTION_KINEMATICS` | 0 | 60 |
| `TRANSITION:BRANCH_UNREACHABLE` | 0 | 0 |
| `TRACKING_BOUND` | 0 | 0 |

开臂留下的 60 个 `MOTION_KINEMATICS` 来自 6 个种子各 10 个 tick。关臂上这些 tick 已被 `TIME` / `HEAD` 打满，认证 `a_max=8` 没有单独记运动学。对齐后按上游 `a_max=6` 检查，这 60 个 tick 是真超上游加速度，留下来。

### crossing-pillars

场景卡哈希 `1d39afbd780740b9e925569a70b27b670d73184d9b487d3adb2473fdf1ed279e`。10/10 到达（每种子 3/3），碰撞 0。`would_reject` 平均 1.000 → 0.0176。

| seed | 到达 | min_obs m | would_reject 关 | would_reject 开 | 开臂直方图 |
|---:|---:|---:|---:|---:|---|
| 0 | 3/3 | 1.012 | 1.000 | 0.031 | `ENV:OCCUPIED` 75 |
| 1 | 3/3 | 1.016 | 1.000 | 0.031 | `ENV:OCCUPIED` 75 |
| 2 | 3/3 | 1.361 | 1.000 | 0.000 | （空） |
| 3 | 3/3 | 1.265 | 1.000 | 0.029 | `MOTION_KINEMATICS` 75 |
| 4 | 3/3 | 1.264 | 1.000 | 0.000 | （空） |
| 5 | 3/3 | 1.266 | 1.000 | 0.029 | `MOTION_KINEMATICS` 75 |
| 6 | 3/3 | 1.178 | 1.000 | 0.000 | （空） |
| 7 | 3/3 | 0.878 | 1.000 | 0.057 | `ENV:OCCUPIED` 75，`MOTION_KINEMATICS` 75 |
| 8 | 3/3 | 1.456 | 1.000 | 0.000 | （空） |
| 9 | 3/3 | 1.244 | 1.000 | 0.000 | （空） |

| 码 | 关（10 种子合计） | 开 |
|---|---:|---:|
| `TIME:SWITCH_NOT_IN_FUTURE` | 750 | 0 |
| `TRANSITION:HEAD_DISCONTINUITY` | 10335 | 0 |
| `TRANSITION:TERMINAL_NOT_HOLDABLE` | 0 | 0 |
| `TRANSITION:MOTION_KINEMATICS` | 13725 | 225 |
| `TRANSITION:BRANCH_UNREACHABLE` | 11250 | 0 |
| `TRACKING_BOUND` | 0 | 0 |
| `ENV:OCCUPIED` | 0 | 225 |

关臂上的 `MOTION_KINEMATICS` 和 `BRANCH_UNREACHABLE` 几乎都是认证 `v_max=4` 套到峰值约 5.4 m/s 的上游轨迹上。对齐后分支码清零，运动学只剩 225 个 tick（3 个种子各 75 个）。`ENV:OCCUPIED` 出现在种子 0、1、7，共 225 个 tick：机体中心到表面最近 0.878–1.016 m，L0 碰撞判据未触发，影子按占用管仍标占用。这不是契约误报，开关没有盖掉。

## 危险卡

一条从 `(0, 0, 1.5)` 到 `(5, 0, 1.5)`、时长 4 s 的静止到静止五次多项式，穿过 g2 柱子 `(2.6, -0.4, 0.0)–(3.2, 0.4, 2.8)`。峰值速度和加速度都在上游限幅内。打开 `shadow_contract_align`，并传入场景几何：

- `would_reject`：是
- 选择门和 tick 的码：只有 `ENV:OCCUPIED`
- 没有 `TIME:SWITCH_NOT_IN_FUTURE`、`TRANSITION:HEAD_DISCONTINUITY`、`TRANSITION:TERMINAL_NOT_HOLDABLE`、`TRANSITION:MOTION_KINEMATICS`、`TRANSITION:BRANCH_UNREACHABLE`

同样条件下，平行于柱子、侧向约 2.2 m 的轨迹没有 `ENV:OCCUPIED`，见票 SH1 的单元测试。

## 回归

`PYTHONPATH=$PWD scripts/regress.sh`：L0 **935 passed，27 skipped**；core **453 passed，3 skipped，4 deselected**；批跑通过。新增 `tests/test_shadow_contract_align.py`：开关默认关、打开不改设定点并标偏离上游、误报码在对齐后消失、超过上游限幅和真头跳跃仍拒绝、限幅内穿柱只剩 `ENV:OCCUPIED`。
