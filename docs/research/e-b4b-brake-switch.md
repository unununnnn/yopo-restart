# 票 E-B4b：无解时从参考头刹停（开关，默认关）

frank 2026-11-12 10:27（America/Argentina/San_Juan）拍板：把 [E-B4](e-b4-peer-collision-diag.md) 的刹停原型做成正式开关，默认关；配合 E-A1 门槛 1.0 m；开着时结果标「偏离上游」。

不改 `core/`、冻结门、`scenes/suite.txt`、`docs/tickets.md`、README，不改任何场景卡，不改默认值，不冻结新判据。

## 开关

`policy_params.upstream.empty_brake`，必须是 JSON 布尔，缺省 `false`。

- 关：无准入候选（`CORRIDOR_EMPTY_HOVER`，含 E-A1 滤空）时仍是上游 SPEC 的「本帧悬停（停在当前位置）」，`hold_traj(s.p)`。
- 开：从本拍的参考头 (p, v, a) 按动力学 `a_max` 刹停。刹停曲线由 `yopo_l0.geom_fallback.brake_from_head` 给出，它就是 D1 回退原来的 `_brake_from_head` 抽出来的同一段代码（core `brake_branch`，v_max 20、j_max 20000），D1 现在也调用它。已经静止时原地保持在参考头。
- 开着时结果多一个 `empty_brake: true`，`upstream_deviation` 记为 `true`。关着时不加字段。只有 goto / hover 的 `upstream` 策略读这个开关。

E-A1 门槛 1.0 m（2 × (0.35 + 0.15)）不是新代码，写在场景卡 `policy_params.upstream.inter_drone_threshold_m`。已提交的卡都不写这两项，对照在进程里覆盖。

```json
"policy_params": {"upstream": {"inter_drone_filter": true, "inter_drone_threshold_m": 1.0, "inter_drone_penalty": 0, "empty_brake": true}}
```

## 关：suite 逐位不变

`scenes/suite.txt` 全部卡，`f5cedba`（导出目录）与本分支各跑一遍 `yopo_l0.batch --no-determinism --jobs 1`，逐个比 `result.json` 与 `ticks.jsonl`（去掉墙钟字段和 commit）：`runs=68 diff=0`。

## 开：swap8 与 d014 seeds 0–9（只报数）

<!-- 由 out/e-b4b/make_doc.py 生成 -->
| 卡 | 臂 | 到达（架次） | 机间碰撞 | 撞树 | 超时 | 最小机间 m | 最小障碍 m |
|---|---|---|---|---|---|---|---|
| swap8 | 全关 | 0/80 | 10 | 0 | 0 | 0.695–0.696 | 1.500–1.500 |
| swap8 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 80/80 | 0 | 0 | 0 | 1.069–1.077 | 1.146–1.459 |
| d014-swap3 | 全关 | 0/30 | 10 | 0 | 0 | 0.622–0.698 | 0.486–0.569 |
| d014-swap3 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 14/30 | 0 | 6 | 0 | 1.099–2.633 | 0.324–0.566 |

开臂与 E-B4 诊断里进程内原型的修复臂逐粒比较（结果、原因、到达、规划码、最小机间 / 障碍距离、仿真时长）：20/20 粒相同；关臂与 E-B4 全关 20/20 粒相同。d014 开臂的撞树都是 `TRUTH_POSITION`，与 [e-c4b](e-c4b-density-seeds.md) 里 safe_radius 0.05 m「走廊过松」同一类，本票不处理。

| 卡 | 臂 | seed | 结果 | 原因 | 到达 | 最小机间 m | 与 E-B4 原型同粒相同 |
|---|---|---|---|---|---|---|---|
| d014-swap3 | 全关 | 0 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.634 | 是 |
| d014-swap3 | 全关 | 1 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.639 | 是 |
| d014-swap3 | 全关 | 2 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.629 | 是 |
| d014-swap3 | 全关 | 3 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.698 | 是 |
| d014-swap3 | 全关 | 4 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.622 | 是 |
| d014-swap3 | 全关 | 5 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.684 | 是 |
| d014-swap3 | 全关 | 6 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.683 | 是 |
| d014-swap3 | 全关 | 7 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.658 | 是 |
| d014-swap3 | 全关 | 8 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.666 | 是 |
| d014-swap3 | 全关 | 9 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.629 | 是 |
| d014-swap3 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 1.653 | 是 |
| d014-swap3 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 1.310 | 是 |
| d014-swap3 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 2 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1.134 | 是 |
| d014-swap3 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 3 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 2.411 | 是 |
| d014-swap3 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 4 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 2.633 | 是 |
| d014-swap3 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 1.167 | 是 |
| d014-swap3 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 6 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 1.281 | 是 |
| d014-swap3 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 7 | COLLISION | `uav0:TRUTH_POSITION` | 2/3 | 1.136 | 是 |
| d014-swap3 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 1.308 | 是 |
| d014-swap3 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1.099 | 是 |
| swap8 | 全关 | 0 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 是 |
| swap8 | 全关 | 1 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 是 |
| swap8 | 全关 | 2 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 是 |
| swap8 | 全关 | 3 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.696 | 是 |
| swap8 | 全关 | 4 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 是 |
| swap8 | 全关 | 5 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 是 |
| swap8 | 全关 | 6 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.696 | 是 |
| swap8 | 全关 | 7 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 是 |
| swap8 | 全关 | 8 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 是 |
| swap8 | 全关 | 9 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 是 |
| swap8 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.077 | 是 |
| swap8 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.069 | 是 |
| swap8 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.069 | 是 |
| swap8 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.070 | 是 |
| swap8 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.070 | 是 |
| swap8 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.070 | 是 |
| swap8 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.075 | 是 |
| swap8 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.076 | 是 |
| swap8 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.076 | 是 |
| swap8 | 开：E-A1 1.0 m + `empty_brake`（偏离上游） | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.069 | 是 |

## 怎么重跑

```bash
.venv/bin/python -m pytest -q tests/test_e_b4b_brake_switch.py
.venv/bin/python scripts/e-b4-peer-diag.py --scene swap8 --arm a1 \
  --extra '{"upstream":{"inter_drone_threshold_m":1.0,"empty_brake":true}}' --compare --seed 0
```
