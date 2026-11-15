# 票 SH1：影子盾环境证据接入与拒绝率归因（只报数）

只报数。report-only。判据未冻结。不改 `core/`、`scenes/suite.txt`、已冻门、安全清单和 PX4 参数。影子盾仍然只记录，不改批准动作。研究开关 `shadow_env_evidence` 默认关；打开的结果标「偏离上游」。关臂与 `765e2cb` 上 g2-pillar seed 0 的拒绝直方图一致。墙钟不进结论。打开时多出来的代价按净空查询次数计，不按秒表。

数字在 [sh1-shadow-env-evidence-runs.json](sh1-shadow-env-evidence-runs.json)。

## 结论

`ENV:UNKNOWN` 不是缺环境地图、缺占据栅格文件，也不是深度时间戳过期。L0 已经把前视深度交进 `DepthSample.evidence`。g2-pillar seed 0 关着时 526 个 tick 全部是 `ENV:UNKNOWN`，没有 `ENV:NO_FRAME`。`classify_sweep` 被调用 213 次：211 次 `UNKNOWN`，2 次 `FREE`。`UNKNOWN` 的第一处样本里，197 次在相机后方，14 次在自由空间最小量程 0.3 m 以内。`core` 的 `classify_ball` 对这两类球给不出 `FREE`。扫掠只要有一个这样的样本，整段运动加上制动分支就是 `ENV:UNKNOWN`。

起飞区 `assume_free` 半径 1.2 m。查询球半径是 0.525 m（机体 0.35 + 跟踪裕度 0.15 + 采样间距 0.05 的一半），证书只抬得起球心距起点不超过 0.675 m 的样本。那 2 次 `FREE` 是这一小段。影子盾按原设计不调用 `certify_sweep`。

打开开关后，三张卡的环境拒绝率都变成 0。`would_reject` 只有 formation-hold 从 1.000 降到 0.028；g2-pillar 和 crossing-pillars 仍是 1.000。剩下的码是计划契约，不是环境：`TIME:SWITCH_NOT_IN_FUTURE`、`TRANSITION:HEAD_DISCONTINUITY`、`TRANSITION:TERMINAL_NOT_HOLDABLE`，穿柱还有 `TRANSITION:MOTION_KINEMATICS` 和 `TRANSITION:BRANCH_UNREACHABLE`。这些码在关臂上就有，打开后条数不变。

这三张卡的飞行没有进入样本球。最近的机体中心到表面：g2-pillar 0.8275–0.8294 m（表面净空 0.4775–0.4794 m，与 S6 的 L0 净空同一列），crossing-pillars 0.8776–1.4560 m，formation-hold 全程 1.5000 m（开阔地面）。都大于 0.525 m，所以飞行直方图里没有 `ENV:OCCUPIED`。单元测试里，轨迹穿进柱子时打开开关得到 `ENV:OCCUPIED`；平行于柱子、侧向约 2.2 m 的轨迹没有环境码。关着时那条平行轨迹仍是 `ENV:UNKNOWN`。

两臂到达时刻、碰撞、轨迹摘要逐粒相同。碰撞都是 0。

## 根因（g2-pillar seed 0，开关关）

入口是 `yopo_l0/runner.py` 每个规划拍调用 `DroneShadow.on_selection`，每个物理 tick 调用 `on_tick`。环境码来自 `core` 的 `planner._env_code` → `FreeSpace.classify_sweep`。深度是本拍前视光线投射，经 `DepthSample.evidence` 变成 `DepthFrame`。帧是新鲜的：诊断里没有 `NO_FRESH_OBSERVATION`。

| 项 | 数 |
|---|---:|
| tick | 526 |
| `would_reject` | 1.000 |
| `ENV:UNKNOWN` | 526 |
| `TIME:SWITCH_NOT_IN_FUTURE` | 316 |
| `TRANSITION:HEAD_DISCONTINUITY` | 316 |
| `TRANSITION:TERMINAL_NOT_HOLDABLE` | 210 |
| `classify_sweep` 调用 | 213 |
| 其中 `UNKNOWN` | 211 |
| 其中 `FREE` | 2 |
| 第一处未知样本在相机后方 | 197 |
| 第一处未知样本在最小量程内 | 14 |

0.5 s 一档，关臂每一档的环境拒绝率都是 1.000。1.0–2.0 s 机体中心最近，最小 0.8290 m，环境拒绝率仍是 1.000。打开后同一档的环境拒绝率是 0，`would_reject` 仍是 1.000，码只剩 `TRANSITION:TERMINAL_NOT_HOLDABLE`。2.5 s 之后码换成 `TIME:SWITCH_NOT_IN_FUTURE` 和 `TRANSITION:HEAD_DISCONTINUITY`，环境拒绝率仍是 0。逐档直方图在 runs.json 的 `seed0_time_bins_off` / `seed0_time_bins_on`。

例子：第一处未知样本在 t = 0.735 s，位置 (0.495, -0.462, 1.445)，离相机 0.592 m，标签 `within_min_range`。后面两次第一处样本在相机后方，离相机 1.224 m 和 1.526 m。

## 开关

`RunConfig.shadow_env_evidence` 默认 `False`。命令行 `--shadow-env-evidence`。关时不把场景障碍传进 `DroneShadow`，结果里不加 `shadow_env_evidence`、`shadow_env_views`、`shadow_env_clearance_evals`、`shadow_env_certificate_centers`。

打开时在 `yopo_l0/shadow_env.py`（不改 `core/`）做两件事：

- 用已有的 `yopo_l0.sensor.render` 对场景卡几何打 5 张无噪声深度（±X、±Y、朝下），`DepthFrame.from_raw` 放进影子自己的 `FreeSpace`。表面点落在样本球里时，码仍是 `ENV:OCCUPIED`。
- 对 `FreeSpace._sample_times` 采到的、中心到表面不小于 0.526 m 的球调用 `add_certificate`。证书只把各帧都是 `UNKNOWN` 的球升成 `FREE`，不覆盖 `OCCUPIED`。

深度只渲染一次，之后按拍改时间戳。g2-pillar 的净空查询是 52003–52731 次，crossing-pillars 是 100800–104292 次，formation-hold 是 2929 次。

## 两臂对照

到达时刻两臂相同，下表只写一列。碰撞两臂都是 0。`min_obs` 是机体中心到表面的最小距离，单位 m。`env` 是带任一 `ENV:` 码的 tick 比例。

### g2-pillar

场景卡哈希 `9d385f4cfce543922961865d06ceeedd44afdddb52c706bf6e3da9929431d8ae`。10/10 到达，碰撞 0。

| seed | 到达 s | 碰撞 | min_obs m | would_reject 关 | would_reject 开 | env 关 | env 开 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 4.26 | 0 | 0.8290 | 1.000 | 1.000 | 1.000 | 0.000 |
| 1 | 4.27 | 0 | 0.8289 | 1.000 | 1.000 | 1.000 | 0.000 |
| 2 | 4.26 | 0 | 0.8289 | 1.000 | 1.000 | 1.000 | 0.000 |
| 3 | 4.26 | 0 | 0.8287 | 1.000 | 1.000 | 1.000 | 0.000 |
| 4 | 4.27 | 0 | 0.8294 | 1.000 | 1.000 | 1.000 | 0.000 |
| 5 | 4.28 | 0 | 0.8280 | 1.000 | 1.000 | 1.000 | 0.000 |
| 6 | 4.28 | 0 | 0.8278 | 1.000 | 1.000 | 1.000 | 0.000 |
| 7 | 4.28 | 0 | 0.8275 | 1.000 | 1.000 | 1.000 | 0.000 |
| 8 | 4.28 | 0 | 0.8277 | 1.000 | 1.000 | 1.000 | 0.000 |
| 9 | 4.27 | 0 | 0.8289 | 1.000 | 1.000 | 1.000 | 0.000 |

打开后的直方图只剩契约码，条数与关臂相同。seed 0：`TIME:SWITCH_NOT_IN_FUTURE` 316，`TRANSITION:HEAD_DISCONTINUITY` 316，`TRANSITION:TERMINAL_NOT_HOLDABLE` 210。中心距大于 1 m 的 tick，环境拒绝率从 1.000 降到 0.000，`would_reject` 仍是 1.000。中心距 ≤ 0.70 m 的 tick 数是 0。

### crossing-pillars

场景卡哈希 `1d39afbd780740b9e925569a70b27b670d73184d9b487d3adb2473fdf1ed279e`。30/30 架到达，碰撞 0。`gap` 是最小机间中心距。

| seed | 到达 s | 碰撞 | min_obs m | gap m | would_reject 关 | would_reject 开 | env 关 | env 开 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 7.06 | 0 | 1.012 | 7.284 | 1.000 | 1.000 | 0.969 | 0.000 |
| 1 | 7.06 | 0 | 1.016 | 7.281 | 1.000 | 1.000 | 0.969 | 0.000 |
| 2 | 7.06 | 0 | 1.361 | 7.307 | 1.000 | 1.000 | 0.969 | 0.000 |
| 3 | 7.76 | 0 | 1.265 | 3.506 | 1.000 | 1.000 | 0.971 | 0.000 |
| 4 | 7.05 | 0 | 1.264 | 3.491 | 1.000 | 1.000 | 0.969 | 0.000 |
| 5 | 7.77 | 0 | 1.266 | 3.350 | 1.000 | 1.000 | 0.971 | 0.000 |
| 6 | 7.07 | 0 | 1.178 | 3.646 | 1.000 | 1.000 | 0.969 | 0.000 |
| 7 | 7.76 | 0 | 0.878 | 3.542 | 1.000 | 1.000 | 0.971 | 0.000 |
| 8 | 7.06 | 0 | 1.456 | 7.510 | 1.000 | 1.000 | 0.969 | 0.000 |
| 9 | 7.05 | 0 | 1.244 | 7.381 | 1.000 | 1.000 | 0.969 | 0.000 |

关臂的 `ENV:UNKNOWN` 不是每个 tick 都有（seed 0 是 2343/2418）。缺的那几十个 tick 落在起飞区证书能盖住的前缀里。打开后环境码清零，`would_reject` 仍是 1.000。seed 0 打开后的直方图：`TRANSITION:MOTION_KINEMATICS` 1350，`TRANSITION:BRANCH_UNREACHABLE` 1125，`TRANSITION:HEAD_DISCONTINUITY` 993，`TIME:SWITCH_NOT_IN_FUTURE` 75。中心距 ≤ 0.70 m 的 tick 数是 0。

### formation-hold

场景卡哈希 `1b1894f5864a2f5f5fb0e06b7d44ec490acf8d012957829f0f599e19951c365e`。冻结表种子 0、1、2。9/9 架到达，到达时刻 18.63 s，碰撞 0，最小机间 1.987 m，最小障碍距离 1.500 m。三粒完全相同。

| seed | 到达 s | 碰撞 | would_reject 关 | would_reject 开 | env 关 | env 开 |
|---:|---:|---:|---:|---:|---:|---:|
| 0 | 18.63 | 0 | 1.000 | 0.028 | 0.972 | 0.000 |
| 1 | 18.63 | 0 | 1.000 | 0.028 | 0.972 | 0.000 |
| 2 | 18.63 | 0 | 1.000 | 0.028 | 0.972 | 0.000 |

打开后只剩 `TIME:SWITCH_NOT_IN_FUTURE` 165 次，5889 个 tick 里占 0.028。这 165 次落在 0–1.1 s：0–0.5 s 的拒绝率 0.701，0.5–1.0 s 是 0.373，1.0–1.5 s 是 0.040，之后到 18.63 s 是 0。开阔段的 `would_reject` 在这张卡上是掉下去的。没有贴近障碍的段可标。

## 还缺什么，这个信号才能用到 SITL

1. 开关没有接到 `yopo_l0/sitl_bridge.py`。SITL 仍用前视场景卡深度，`DroneShadow` 不读几何证据。本次不改 SITL 路径，所以 S6 那组 1.000 不会因这次提交变化。
2. 就算接上，g2 和 crossing 的 `would_reject` 在 L0 上已经被契约码打满。影子把上游多项式包成 `make_plan`，`t_switch` 取轨迹起点，起点不在 `now` 之后就记 `TIME:SWITCH_NOT_IN_FUTURE`，头状态对不上父计划就记 `TRANSITION:HEAD_DISCONTINUITY`，末端停不住就记 `TRANSITION:TERMINAL_NOT_HOLDABLE`。穿柱还过不了运动学和制动分支。这些码与环境证据无关。S6 的 SITL 直方图在 `ENV:UNKNOWN` 之外还有 `TRACKING_BOUND` 和 `LAG_VIOLATION`。环境码清零以后，`would_reject` 仍分不开「门导致的失败」和「动力学或中间件导致的失败」，除非契约复验不再把上游多项式的包装差当成每一拍的拒绝。
3. 这三张卡的轨迹都在样本球外面，飞行数据里没有「贴近障碍仍被标出」。要在 SITL 上看到 `ENV:OCCUPIED`，扫掠得到达表面 0.525 m 以内，并且证据帧里要有那个表面点。S6 的最小净空被降落段压到接近 0，不能直接当成飞行段贴障。本次没有重跑 SITL。
4. 打开的证据用的是场景卡几何，不是机载深度。真机或 Gazebo 相机仍然看不见身后和最小量程以内。那部分要另有证书来源，不能靠把这次开关原样当成机载感知。

## pytest

`PYTHONPATH=.:core/src pytest tests -q -n 4 -m "not slow"`：**847 passed，29 skipped，3 failed**（8 分 30 秒）。3 条失败随后单独重跑通过：`test_demo.py` 和 `test_render_delivery.py` 当时停在 `uv: not found`，装上 `uv` 后 2 passed；`test_e_so3_dynamics_gap.py` 在 `git diff yopo_l0` 非空时失败，改动进索引后通过。合计 850 passed，29 skipped。新增 `tests/test_shadow_env_evidence.py` 4 条在里面。
