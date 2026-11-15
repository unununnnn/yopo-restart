# 票 SH2：影子盾 ENV 证据接 SITL 与拒绝码归因（只报数）

只报数。report-only。判据未冻结。不改 `core/`、`scenes/suite.txt`、已冻门、安全清单和 PX4 参数。影子盾仍然只记录，不改设定点。研究开关 `shadow_env_evidence` 默认关；打开的结果标「偏离上游」。编队类 SITL 策略还要打开 `fleet_shadow` 才会记影子盾（默认关，避免改 crossing / switch 的缺省结果字段）。墙钟不进结论。

数字在 [sh2-sitl-shadow-evidence-runs.json](sh2-sitl-shadow-evidence-runs.json)。

## 结论

`shadow_env_evidence` 已经接到 `sitl_bridge`。桥接主循环只调用 `yopo_l0/shadow_sitl.py` 的钩子：要不要开影子、要不要把场景卡障碍交给 `DroneShadow`、编队类策略用编队钟和已提交计划、结果里写哪些字段。打开时标「偏离上游」。关着不加这些字段，upstream 缺省仍然只在 S6 那条路上开影子盾。

L0 上 g2-pillar 和 crossing-pillars 的结论与 SH1 一致。打开开关后环境拒绝率都变成 0，`would_reject` 仍是 1.000。剩下的码是计划契约，条数与关臂相同。关掉环境码之后，这些契约码仍然覆盖全部会拒的 tick（`reject_only_env` 在关臂上就是 0：环境码从来不是单独把 `would_reject` 打满的那一个）。

锁步 SITL 的 `crossing-pillars-sitl80` seeds 0–4 上，影子盾第一次接到交叉策略。成功飞完且打开证据的种子（0、1），环境码清零；`would_reject` 仍是 0.987–1.000，靠契约码和跟踪码打满。SITL 比 L0 多出大量 `TRACKING_BOUND`：机体和批准位置的差超过 FlightOracle 的 0.15 m，这是 PX4 跟踪，不是环境。撞障碍的打开臂出现了 `ENV:OCCUPIED`（seed 2 开臂 11 次，seed 3 开臂 38 次）。SH1 在 L0 飞行直方图里没有这个码，因为那几张卡的轨迹都在样本球外面。

两臂飞行不是逐 tick 相同。B1 同一张卡、同一个种子的到达时间跨次本来就在十几秒到二十几秒之间起伏，有的批次成功、有的批次擦柱。本票不把到达时间差写成开关的效果。种子 4 关臂在进入 PLAN、影子开始记 tick 之前就撞了；开臂在预热阶段锁步握手超时，没有影子样本。这两趟不进入拒绝码合计。

环境证据仍然用不了 `would_reject` 去分「门导致的失败」和「动力学或中间件导致的失败」。契约包装还在：上游多项式的 `t_switch` 取轨迹起点，一起飞就可能记下 `TIME:SWITCH_NOT_IN_FUTURE` 和 `TRANSITION:HEAD_DISCONTINUITY`；穿柱规划还经常过不了运动学和制动分支。跟踪码只说明 PX4 没跟上批准点。

## 开关与钩子

`LiveFleetConfig.shadow_env_evidence` 和 `fleet_shadow` 默认 `False`。命令行 `--shadow-env-evidence`、`--fleet-shadow`，也可以用 `YOPO_SHADOW_ENV_EVIDENCE`、`YOPO_FLEET_SHADOW`。关着时 crossing 的 dry-run 仍然是 `shadow=false`，结果不加证据字段。

打开证据时，`DroneShadow` 收到场景卡障碍，走 SH1 的 `shadow_env.SceneEvidence`：五张无噪声深度和净空证书。编队类策略用编队钟（全体进入 PLAN 起算）复验已提交计划，和 L0 的 `fleet.shadow_view` 同一条路径。

影子观察只读已经渲染好的场景卡深度。不提前渲染 `LazyDepthFrame`，以免消耗相机噪声序列、改变后续提案。还没有前视帧时，打开证据不记 `ENV:NO_FRAME`（五张几何视图已经进 `FreeSpace`）；关证据则沿用上一张已渲染帧，否则才是 `ENV:NO_FRAME`。

## L0 对照

场景卡与 SH1 相同：`scenes/g2-pillar.json`、`scenes/crossing-pillars.json`，seeds 0–9。两臂到达、碰撞和轨迹摘要相同。碰撞都是 0。

### g2-pillar

10/10 到达，到达时刻 4.26–4.28 s，碰撞 0。合计 5271 tick。

| 臂 | would_reject | env 拒绝率 | `ENV:UNKNOWN` | `TIME:SWITCH_NOT_IN_FUTURE` | `TRANSITION:HEAD_DISCONTINUITY` | `TRANSITION:TERMINAL_NOT_HOLDABLE` | 只靠环境码拒绝 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 关 | 1.000 | 1.000 | 5271 | 3171 | 3171 | 2100 | 0 |
| 开 | 1.000 | 0.000 | 0 | 3171 | 3171 | 2100 | 0 |

seed 0 打开后的直方图与 SH1 相同：`TIME:SWITCH_NOT_IN_FUTURE` 316，`TRANSITION:HEAD_DISCONTINUITY` 316，`TRANSITION:TERMINAL_NOT_HOLDABLE` 210。

`TRANSITION:TERMINAL_NOT_HOLDABLE` 和 `TIME` / `HEAD_DISCONTINUITY` 在时间上是分段的：末端停不住的码出现在前半段，后半段换成开关时刻不在未来、头状态对不上父计划。两类加起来正好 5271，把全部 tick 打满。

### crossing-pillars

30/30 架到达，到达时刻 7.05–7.77 s，碰撞 0。合计 24810 tick。关臂环境拒绝率 0.969–0.971（起飞区证书能盖住前缀里那几十个没有 `ENV:UNKNOWN` 的 tick）。

| 臂 | would_reject | env 拒绝率 | `ENV:UNKNOWN` | `TRANSITION:MOTION_KINEMATICS` | `TRANSITION:BRANCH_UNREACHABLE` | `TRANSITION:HEAD_DISCONTINUITY` | `TIME:SWITCH_NOT_IN_FUTURE` |
|---|---:|---:|---:|---:|---:|---:|---:|
| 关 | 1.000 | 0.969–0.971 | 24060 | 13725 | 11250 | 10335 | 750 |
| 开 | 1.000 | 0.000 | 0 | 13725 | 11250 | 10335 | 750 |

seed 0 打开后：`TRANSITION:MOTION_KINEMATICS` 1350，`TRANSITION:BRANCH_UNREACHABLE` 1125，`TRANSITION:HEAD_DISCONTINUITY` 993，`TIME:SWITCH_NOT_IN_FUTURE` 75。与 SH1 相同。穿柱的上游多项式经常超过缺省运动学限制（v_max=4 m/s、a_max=8 m/s²、j_max=80 m/s³），制动分支也生成不出。

## 锁步 SITL：crossing-pillars-sitl80 seeds 0–4

入口：

```bash
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/crossing-pillars-sitl80.json --policy crossing --seed 0 --lockstep --fleet-shadow
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/crossing-pillars-sitl80.json --policy crossing --seed 0 --lockstep --fleet-shadow --shadow-env-evidence
```

镜像 `yopo-restart-sitl:yopo-81d084c04186`。不写 PX4 参数：`px4_parameters_written` 全程是空列表。锁步 `clock_ahead` 都是 0。影子在编队钟上记 tick，起飞段不记。

| seed | 臂 | 结果 | 到达 | 碰撞 | 规划净空 m | tick | would_reject | env 拒绝率 | `ENV:UNKNOWN` | `ENV:OCCUPIED` | `ENV:NO_FRAME` | `TRACKING_BOUND` |
|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 关 | SUCCESS | 3/3 | 0 | 0.306 | 1569 | 1.000 | 0.903 | 1417 | 0 | 38 | 1065 |
| 0 | 开 | SUCCESS | 3/3 | 0 | 1.016 | 2646 | 0.987 | 0.000 | 0 | 0 | 0 | 1562 |
| 1 | 关 | SUCCESS | 3/3 | 0 | 0.290 | 2583 | 1.000 | 0.956 | 2469 | 0 | 38 | 1643 |
| 1 | 开 | SUCCESS | 3/3 | 0 | 0.023 | 2586 | 1.000 | 0.000 | 0 | 0 | 0 | 1960 |
| 2 | 关 | SUCCESS | 3/3 | 0 | 0.295 | 1800 | 1.000 | 0.937 | 1686 | 0 | 38 | 1527 |
| 2 | 开 | FAIL uav2:OBSTACLE | 0/3 | 1 | −0.085 | 333 | 1.000 | 0.144 | 37 | 11 | 0 | 199 |
| 3 | 关 | FAIL uav1:OBSTACLE | 0/3 | 1 | −0.001 | 282 | 1.000 | 1.000 | 282 | 0 | 38 | 142 |
| 3 | 开 | FAIL uav0:OBSTACLE | 0/3 | 1 | −0.025 | 276 | 1.000 | 0.272 | 37 | 38 | 0 | 146 |
| 4 | 关 | FAIL uav2:OBSTACLE | 0/3 | 1 | （未进入 PLAN） | 0 | — | — | — | — | — | — |
| 4 | 开 | FAIL LOCKSTEP_TIMEOUT | 0/3 | 0 | （预热握手超时） | 0 | — | — | — | — | — | — |

有影子 tick 的 8 趟合计：

| 臂 | 趟数 | tick | would_reject 率 | env 拒绝率 | 主要剩余码 |
|---|---:|---:|---:|---:|---|
| 关 | 4 | 6234 | 1.000 | 0.939 | `ENV:UNKNOWN` 5854，`TRANSITION:HEAD_DISCONTINUITY` 3662，`TRACKING_BOUND` 4377，`TRANSITION:MOTION_KINEMATICS` 2420，`TRANSITION:BRANCH_UNREACHABLE` 1817，`TIME:SWITCH_NOT_IN_FUTURE` 152，`ENV:NO_FRAME` 152 |
| 开 | 4 | 5841 | 0.994 | 0.021 | `TRANSITION:HEAD_DISCONTINUITY` 3617，`TRACKING_BOUND` 3867，`TRANSITION:MOTION_KINEMATICS` 2033，`TRANSITION:BRANCH_UNREACHABLE` 1507，`TIME:SWITCH_NOT_IN_FUTURE` 153，`ENV:UNKNOWN` 74，`ENV:OCCUPIED` 49 |

打开后环境码几乎只剩撞障碍的那两趟。成功的 seed 0、1 开臂环境码是 0。seed 0 开臂 `would_reject` 0.987，是 34 个 tick 既没有契约码也没有跟踪码；去掉环境码后，剩下的拒绝仍然全部能被契约码或跟踪码解释（`non_env_covers_would_reject` 为真）。关臂的 `ENV:NO_FRAME` 只有进入 PLAN 后、第一张前视深度渲染之前的那 38 个 tick（三机合计，每机约 13 拍）。

关臂撞柱时没有 `ENV:OCCUPIED`，因为影子看不到场景几何，扫掠只能给出 `ENV:UNKNOWN`。打开后，计划已经擦进表面的 tick 会被标成占用。seed 1 开臂规划净空 0.023 m，小于证书半径 0.525 m，但这一趟没有记到 `ENV:OCCUPIED`：贴近发生在跟踪和规划净空口径上，不表示每一个贴近样本都会在影子的计划扫掠里被标出。

## 码的触发条件

这些条件是现有 `core` 判据的字面含义。本票只用来归因，没有改阈值。

| 码 | 触发条件 |
|---|---|
| `ENV:NO_FRAME` | 这一拍影子没有深度帧。关证据且还没有已渲染的前视帧时会出现。 |
| `ENV:UNKNOWN` | 扫掠里有样本既不是 FREE 也不是 OCCUPIED。关证据时，相机后方和最小量程以内通常落在这里。 |
| `ENV:OCCUPIED` | `classify_sweep` 认为扫掠球碰到已观测到的表面。 |
| `TIME:SWITCH_NOT_IN_FUTURE` | 包装后的计划 `t_switch` 不大于当前影子时钟。上游多项式的 `t_switch` 取轨迹起点，计划一开始跑就会记下这个码。 |
| `TRANSITION:HEAD_DISCONTINUITY` | 运动段起点的位置/速度/加速度与声明的 `head_pva` 对不上，或与父计划在 `t_switch` 的状态差超过 1e-6。 |
| `TRANSITION:TERMINAL_NOT_HOLDABLE` | 运动段末端速度或加速度的绝对值超过 1e-6，不能当悬停收住。 |
| `TRANSITION:MOTION_KINEMATICS` | 运动段超过计划运动学限制（缺省 v_max=4 m/s、a_max=8 m/s²、j_max=80 m/s³）。 |
| `TRANSITION:BRANCH_UNREACHABLE` | 在某个制动分支时刻，按同样限制生成不出刹停曲线。 |
| `TRACKING_BOUND` | 机体位置与批准位置的欧氏距离超过 `FlightOracle.tracking_bound`（0.15 m）。 |
| `LAG_VIOLATION` | 沿起点到终点的进度落后超过 1.5 m；参考速度 0.15 m/s。本票的 L0 和这组 SITL 直方图里没有出现。 |

## pytest

见合入前 `scripts/regress.sh` 的记录。新增 `tests/test_sitl_sh2_shadow_evidence.py`：开关默认关、打开不改设定点、缺省 crossing 仍不开影子、打开才记拒绝并标偏离上游、不提前渲染懒深度。
