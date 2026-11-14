# 票 E-C5d：地面净空闸门与刹停高度保护

只报数（report-only）。默认仍然是 `ground_guard=false`、`empty_brake=false`、`track_clearance=false`、`safe_radius=0.05` m。本票四档都打开了 `track_clearance`，因此全部偏离上游。`ground_guard` 打开也偏离上游。没有改 `yopo_l0/sensor.py`、`yopo_l0/backend/`、`yopo_l0/crossing.py`、`core/`、`scenes/suite.txt`、`docs/tickets.md`、`README.md`。未冻结，不进 `scenes/suite.txt`，不设 gates。

`ground_guard_clearance_m` 缺省 0.43 m，等于机体半径 0.35 m 加 0.08 m。0.08 m 仍是 [E-C5](e-c5-corridor-radius.md) 那条跟踪差假设。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。

数字在 [e-c5d-ground-guard-runs.json](e-c5d-ground-guard-runs.json)。跑数时 `HEAD` 是 `3c9a5e4b4f4c9631d23b45abcccf0d2ff0c54f29`，闸门代码在同一工作区里，和这份记录一起提交。

## 结论

1. **014 对向 seed 2 与 seed 9 的 `uav1` 撞地，在三档打开 `ground_guard` 的臂上都没有了。** 关着的臂与 E-C5c 对得上：两次都是 `uav1:TRUTH_POSITION`，地面净空 0.349 m 和 0.347 m，都低于机体半径 0.35 m，圆柱净空仍大于 0.35 m。`geom + ground_guard`（`empty_brake` 关）把 seed 9 送到终点，`uav1` 最低离地 0.523 m；seed 2 不再撞地（`uav1` 0.388 m），改成撞柱。`geom + empty_brake + ground_guard` 和 `geom_multi + empty_brake + ground_guard` 这两档数字相同：seed 2 三机都到，`uav1` 最低离地 0.622 m；seed 9 改成撞柱，`uav1` 离地 0.596 m。

2. **014 对向到达架次没有增加，撞地换成了别的碰撞。** 四档都是 24/30。关着的臂是 2 次撞地。`empty_brake` 关、只开地面闸门：撞柱 1、机间 1（seed 5，`uav1-uav2`，机间 0.656 m）。两档 `empty_brake` 开着再加闸门：撞柱 2，机间 0，最小地面净空 0.431–0.666 m。闸门把参考从地面上拉开，没有多送出一架。

3. **028 对向要连着 `empty_brake` 才把撞地清掉。** 关着的臂与 E-C5c 的 geom 一致：到达 13/30，撞柱 1，撞地 4，超时 2。只开闸门、关掉 `empty_brake`：到达降到 6/30，撞地还剩 seed 0 的 `uav2`（0.348 m），撞柱升到 5，超时升到 4。`geom + empty_brake + ground_guard` 到达 18/30，撞地 0，撞柱 2，超时 5。`geom_multi` 同一组合到达 14/30，撞地 0，撞柱 3，超时 6。无解时若仍用会下冲的 `brake_from_head`，闸门挡不住已经贴地的真值；换成 `ground_safe_brake_from_head` 之后，这张卡上的撞地是 0。代价是多了超时。

4. **028 单机没有撞地。`geom_multi` 在单机上不查树，闸门补不了这件事。** `geom` 关着是 8/10，两档把地面闸门加在 `geom` 上是 9/10 和 8/10，撞柱 0、撞地 0。`geom_multi + empty_brake + ground_guard` 是 2/10，撞柱 7，撞地 0，超时 1。单机没有邻机，`geom_multi` 的树净空门全程关闭，只剩地面闸门和 0.05 m 走廊，柱还是会撞上。这和 E-C5c「单机 `geom_multi` 等于没开几何门」是同一条原因。

5. **默认关，没有 Ground 就不起作用。** `ground_guard=false` 时 `UpstreamPolicy` 不读 Ground，无解分支仍是原来的原地悬停或 `brake_from_head`。场景里没有 Ground 时，开关打开也不改候选、不改刹停。

## 开关

`policy_params.upstream`：

| 键 | 缺省 | 打开之后 |
|---|---|---|
| `ground_guard` | `false` | 非布尔值报错。有 Ground 时 `safe_z = max(Ground.z) + ground_guard_clearance_m` |
| `ground_guard_clearance_m` | `0.43` | 非正、非有限、布尔值报错。关着时不读 |

候选：采样最低点要 ≥ `safe_z`。真值已经低于 `safe_z` 时，只留净上升、末点高于 `safe_z` 或高于 `max(真值 z, 参考头 z) + 0.02` m 的候选，否则当无解。无解且 `empty_brake` 开着，走 `ground_safe_brake_from_head`，刹停曲线不得下穿 `safe_z`。`empty_brake` 关着且真值或参考头低于 `safe_z` 时，从该状态五次拉到 `safe_z` 悬停，不再 `hold_traj` 贴地。

## 卡和臂

| 卡 | 文件 | 机数 | 窗口 |
|---|---|---|---|
| 014 对向 | `scenes/ego-density-014-swap3.json` | 3 | 40 s |
| 028 对向 | `scenes/ego-density-028-swap3.json` | 3 | 40 s |
| 028 单机 | `scenes/ego-density-028-solo.json` | 1 | 50 s |

四档都带 E-A1（阈值 1.0 m，惩罚 0）和 `track_clearance`（margin 0.08 m）。`safe_radius` 不写进进程内卡，仍是 0.05 m。影子盾开，只记录。seeds 0–9。

| 臂 | `track_clearance_mode` | `empty_brake` | `ground_guard` | 偏离上游 |
|---|---|---|---|---|
| geom（关） | `geom` | 开 | 关 | 偏离上游 |
| geom + ground_guard | `geom` | 关 | 开 | 偏离上游 |
| geom + empty_brake + ground_guard | `geom` | 开 | 开 | 偏离上游 |
| geom_multi + empty_brake + ground_guard | `geom_multi` | 开 | 开 | 偏离上游 |

「关」指地面闸门关。这一档就是 E-C5c 的 base + track geom：E-A1 和 `empty_brake` 都开着，用来对照那两次撞地。第二档把 `empty_brake` 关掉，只留贴地拉升。后两档无解走带高度保护的刹停。

## 汇总

到达按架次。总碰撞是 `COLLISION` 的运行次数。撞柱是非地面表面距离 ≤ 0.35 m，撞地是到 Ground 的距离 ≤ 0.35 m，一次运行里可以同时记下。机间是 `INTER_DRONE`。最小地面净空是全程中心离地高度。最小机间是中心距，单机写 —。

| 卡 | 臂 | 偏离上游 | 到达 | 总碰撞 | 撞柱 | 撞地 | 机间碰撞 | 超时 | 最小地面净空 m | 最小机间 m |
|---|---|---|---|---|---|---|---|---|---|---|
| 014 对向 | geom（关） | 偏离上游 | 24/30 | 2 | 0 | 2 | 0 | 0 | 0.347–0.666 | 0.856–1.327 |
| 014 对向 | geom + ground_guard | 偏离上游 | 24/30 | 2 | 1 | 0 | 1 | 0 | 0.360–0.705 | 0.656–2.020 |
| 014 对向 | geom + empty_brake + ground_guard | 偏离上游 | 24/30 | 2 | 2 | 0 | 0 | 0 | 0.431–0.666 | 0.856–1.327 |
| 014 对向 | geom_multi + empty_brake + ground_guard | 偏离上游 | 24/30 | 2 | 2 | 0 | 0 | 0 | 0.431–0.666 | 0.856–1.327 |
| 028 对向 | geom（关） | 偏离上游 | 13/30 | 5 | 1 | 4 | 0 | 2 | 0.348–0.657 | 1.076–1.680 |
| 028 对向 | geom + ground_guard | 偏离上游 | 6/30 | 6 | 5 | 1 | 0 | 4 | 0.348–0.857 | 0.976–9.177 |
| 028 对向 | geom + empty_brake + ground_guard | 偏离上游 | 18/30 | 2 | 2 | 0 | 0 | 5 | 0.426–0.843 | 0.703–1.698 |
| 028 对向 | geom_multi + empty_brake + ground_guard | 偏离上游 | 14/30 | 3 | 3 | 0 | 0 | 6 | 0.539–0.686 | 0.953–1.566 |
| 028 单机 | geom（关） | 偏离上游 | 8/10 | 0 | 0 | 0 | 0 | 2 | 0.431–0.756 | — |
| 028 单机 | geom + ground_guard | 偏离上游 | 9/10 | 0 | 0 | 0 | 0 | 1 | 0.410–0.756 | — |
| 028 单机 | geom + empty_brake + ground_guard | 偏离上游 | 8/10 | 0 | 0 | 0 | 0 | 2 | 0.431–0.756 | — |
| 028 单机 | geom_multi + empty_brake + ground_guard | 偏离上游 | 2/10 | 7 | 7 | 0 | 0 | 1 | 0.408–0.528 | — |

014 对向关着的臂：到达 24/30，撞地正好是 seed 2 与 seed 9，机间 0.856–1.327 m，与 E-C5b / E-C5c 的 geom 一致。028 对向关着的臂：13/30、撞柱 1、撞地 4、超时 2，与 E-C5c 的 geom 一致。

## 014 对向 seed 2 与 seed 9 的 uav1

撞地指该机全程地面净空 ≤ 0.35 m。

| 臂 | seed | 结果 | 原因 | uav1 最小地面净空 m | uav1 撞地 |
|---|---|---|---|---|---|
| geom（关） | 2 | COLLISION | `uav1:TRUTH_POSITION` | 0.349 | 是 |
| geom（关） | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0.347 | 是 |
| geom + ground_guard | 2 | COLLISION | `uav1:TRUTH_POSITION` | 0.388 | 否 |
| geom + ground_guard | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 0.523 | 否 |
| geom + empty_brake + ground_guard | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 0.622 | 否 |
| geom + empty_brake + ground_guard | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0.596 | 否 |
| geom_multi + empty_brake + ground_guard | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 0.622 | 否 |
| geom_multi + empty_brake + ground_guard | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0.596 | 否 |

打开闸门之后，这两粒的 `uav1` 都不再进入 0.35 m。seed 2 在两档刹停保护上整次任务到达。seed 9 在那两档改撞圆柱，离地 0.596 m。

## 还在的撞地

七次撞地里六次在闸门关闭的臂上。唯一还开着闸门的是 028 对向 `geom + ground_guard` seed 0：`uav2:TRUTH_POSITION`，地面 0.348 m。这一档 `empty_brake` 是关的，无解时只在真值或参考头已经低于 `safe_z` 才拉升。同一粒在 `geom + empty_brake + ground_guard` 上不再撞地，结果是超时，最低离地 0.443 m。

| 卡 | 臂 | seed | 原因 | 地面 m |
|---|---|---|---|---|
| 014 对向 | geom（关） | 2 | `uav1:TRUTH_POSITION` | 0.349 |
| 014 对向 | geom（关） | 9 | `uav1:TRUTH_POSITION` | 0.347 |
| 028 对向 | geom（关） | 0 | `uav1:TRUTH_POSITION` | 0.349 |
| 028 对向 | geom（关） | 4 | `uav2:TRUTH_POSITION` | 0.349 |
| 028 对向 | geom（关） | 6 | `uav2:TRUTH_POSITION` | 0.350 |
| 028 对向 | geom（关） | 7 | `uav2:TRUTH_POSITION` | 0.348 |
| 028 对向 | geom + ground_guard | 0 | `uav2:TRUTH_POSITION` | 0.348 |

## 分种子

| 卡 | 臂 | seed | 结果 | 原因 | 到达 | 撞柱 | 撞地 | 机间 | 超时 | 最小地面净空 m | 最小机间 m | uav1 地面 m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 014 对向 | geom（关） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.375 | 1.195 | 0.375 |
| 014 对向 | geom（关） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.600 | 1.298 | 0.600 |
| 014 对向 | geom（关） | 2 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0 | 1 | 0 | 0 | 0.349 | 1.134 | 0.349 |
| 014 对向 | geom（关） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.464 | 1.242 | 0.464 |
| 014 对向 | geom（关） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.567 | 1.278 | 0.610 |
| 014 对向 | geom（关） | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.666 | 0.856 | 0.897 |
| 014 对向 | geom（关） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.545 | 1.327 | 0.601 |
| 014 对向 | geom（关） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.431 | 1.076 | 0.431 |
| 014 对向 | geom（关） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.549 | 1.109 | 0.555 |
| 014 对向 | geom（关） | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0 | 1 | 0 | 0 | 0.347 | 1.159 | 0.347 |
| 014 对向 | geom + ground_guard | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.360 | 1.195 | 0.360 |
| 014 对向 | geom + ground_guard | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.526 | 1.298 | 0.600 |
| 014 对向 | geom + ground_guard | 2 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.388 | 1.134 | 0.388 |
| 014 对向 | geom + ground_guard | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.540 | 1.280 | 0.540 |
| 014 对向 | geom + ground_guard | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.610 | 1.278 | 0.610 |
| 014 对向 | geom + ground_guard | 5 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0 | 0 | 1 | 0 | 0.705 | 0.656 | 1.249 |
| 014 对向 | geom + ground_guard | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.545 | 1.327 | 0.601 |
| 014 对向 | geom + ground_guard | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.459 | 2.020 | 0.459 |
| 014 对向 | geom + ground_guard | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.549 | 1.109 | 0.555 |
| 014 对向 | geom + ground_guard | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.523 | 1.159 | 0.523 |
| 014 对向 | geom + empty_brake + ground_guard | 0 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.634 | 1.195 | 0.634 |
| 014 对向 | geom + empty_brake + ground_guard | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.600 | 1.298 | 0.600 |
| 014 对向 | geom + empty_brake + ground_guard | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.578 | 1.134 | 0.622 |
| 014 对向 | geom + empty_brake + ground_guard | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.464 | 1.242 | 0.464 |
| 014 对向 | geom + empty_brake + ground_guard | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.567 | 1.278 | 0.610 |
| 014 对向 | geom + empty_brake + ground_guard | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.666 | 0.856 | 0.897 |
| 014 对向 | geom + empty_brake + ground_guard | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.545 | 1.327 | 0.601 |
| 014 对向 | geom + empty_brake + ground_guard | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.431 | 1.076 | 0.431 |
| 014 对向 | geom + empty_brake + ground_guard | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.549 | 1.109 | 0.555 |
| 014 对向 | geom + empty_brake + ground_guard | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.596 | 1.159 | 0.596 |
| 014 对向 | geom_multi + empty_brake + ground_guard | 0 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.634 | 1.195 | 0.634 |
| 014 对向 | geom_multi + empty_brake + ground_guard | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.600 | 1.298 | 0.600 |
| 014 对向 | geom_multi + empty_brake + ground_guard | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.578 | 1.134 | 0.622 |
| 014 对向 | geom_multi + empty_brake + ground_guard | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.464 | 1.242 | 0.464 |
| 014 对向 | geom_multi + empty_brake + ground_guard | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.567 | 1.278 | 0.610 |
| 014 对向 | geom_multi + empty_brake + ground_guard | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.666 | 0.856 | 0.897 |
| 014 对向 | geom_multi + empty_brake + ground_guard | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.545 | 1.327 | 0.601 |
| 014 对向 | geom_multi + empty_brake + ground_guard | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.431 | 1.076 | 0.431 |
| 014 对向 | geom_multi + empty_brake + ground_guard | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.549 | 1.109 | 0.555 |
| 014 对向 | geom_multi + empty_brake + ground_guard | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.596 | 1.159 | 0.596 |
| 028 对向 | geom（关） | 0 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0 | 1 | 0 | 0 | 0.349 | 1.207 | 0.349 |
| 028 对向 | geom（关） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.483 | 1.279 | 0.690 |
| 028 对向 | geom（关） | 2 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.409 | 1.076 | 1.113 |
| 028 对向 | geom（关） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.426 | 1.412 | 0.426 |
| 028 对向 | geom（关） | 4 | COLLISION | `uav2:TRUTH_POSITION` | 0/3 | 0 | 1 | 0 | 0 | 0.349 | 1.288 | 1.060 |
| 028 对向 | geom（关） | 5 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.512 | 1.602 | 1.113 |
| 028 对向 | geom（关） | 6 | COLLISION | `uav2:TRUTH_POSITION` | 0/3 | 0 | 1 | 0 | 0 | 0.350 | 1.646 | 1.113 |
| 028 对向 | geom（关） | 7 | COLLISION | `uav2:TRUTH_POSITION` | 0/3 | 0 | 1 | 0 | 0 | 0.348 | 1.680 | 1.113 |
| 028 对向 | geom（关） | 8 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.657 | 1.551 | 1.115 |
| 028 对向 | geom（关） | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.448 | 1.091 | 0.506 |
| 028 对向 | geom + ground_guard | 0 | COLLISION | `uav2:TRUTH_POSITION` | 0/3 | 0 | 1 | 0 | 0 | 0.348 | 2.128 | 1.060 |
| 028 对向 | geom + ground_guard | 1 | COLLISION | `uav2:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.787 | 1.027 | 1.149 |
| 028 对向 | geom + ground_guard | 2 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.788 | 1.132 | 1.113 |
| 028 对向 | geom + ground_guard | 3 | TIMEOUT | `GOAL_NOT_REACHED` | 1/3 | 0 | 0 | 0 | 1 | 0.492 | 1.065 | 0.915 |
| 028 对向 | geom + ground_guard | 4 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.503 | 1.230 | 1.113 |
| 028 对向 | geom + ground_guard | 5 | TIMEOUT | `GOAL_NOT_REACHED` | 1/3 | 0 | 0 | 0 | 1 | 0.494 | 0.976 | 1.113 |
| 028 对向 | geom + ground_guard | 6 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.392 | 0.980 | 1.113 |
| 028 对向 | geom + ground_guard | 7 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.629 | 1.680 | 1.113 |
| 028 对向 | geom + ground_guard | 8 | COLLISION | `uav2:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.857 | 9.177 | 1.115 |
| 028 对向 | geom + ground_guard | 9 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.567 | 1.134 | 0.654 |
| 028 对向 | geom + empty_brake + ground_guard | 0 | TIMEOUT | `GOAL_NOT_REACHED` | 1/3 | 0 | 0 | 0 | 1 | 0.443 | 0.703 | 1.060 |
| 028 对向 | geom + empty_brake + ground_guard | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.483 | 1.279 | 0.690 |
| 028 对向 | geom + empty_brake + ground_guard | 2 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.675 | 0.997 | 1.113 |
| 028 对向 | geom + empty_brake + ground_guard | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.426 | 1.412 | 0.426 |
| 028 对向 | geom + empty_brake + ground_guard | 4 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.503 | 1.250 | 1.060 |
| 028 对向 | geom + empty_brake + ground_guard | 5 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.843 | 1.602 | 1.113 |
| 028 对向 | geom + empty_brake + ground_guard | 6 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.560 | 1.473 | 1.113 |
| 028 对向 | geom + empty_brake + ground_guard | 7 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.629 | 1.698 | 1.113 |
| 028 对向 | geom + empty_brake + ground_guard | 8 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.657 | 1.551 | 1.115 |
| 028 对向 | geom + empty_brake + ground_guard | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.506 | 1.056 | 0.506 |
| 028 对向 | geom_multi + empty_brake + ground_guard | 0 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.589 | 0.953 | 0.652 |
| 028 对向 | geom_multi + empty_brake + ground_guard | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0 | 0 | 0 | 0 | 0.651 | 1.100 | 0.948 |
| 028 对向 | geom_multi + empty_brake + ground_guard | 2 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.675 | 1.365 | 1.001 |
| 028 对向 | geom_multi + empty_brake + ground_guard | 3 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.539 | 1.566 | 0.867 |
| 028 对向 | geom_multi + empty_brake + ground_guard | 4 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.565 | 1.392 | 1.102 |
| 028 对向 | geom_multi + empty_brake + ground_guard | 5 | TIMEOUT | `GOAL_NOT_REACHED` | 1/3 | 0 | 0 | 0 | 1 | 0.579 | 1.396 | 1.002 |
| 028 对向 | geom_multi + empty_brake + ground_guard | 6 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.560 | 1.314 | 1.024 |
| 028 对向 | geom_multi + empty_brake + ground_guard | 7 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.629 | 1.057 | 1.081 |
| 028 对向 | geom_multi + empty_brake + ground_guard | 8 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1 | 0 | 0 | 0 | 0.673 | 1.444 | 0.944 |
| 028 对向 | geom_multi + empty_brake + ground_guard | 9 | TIMEOUT | `GOAL_NOT_REACHED` | 2/3 | 0 | 0 | 0 | 1 | 0.686 | 1.463 | 0.996 |
| 028 单机 | geom（关） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.515 | — | — |
| 028 单机 | geom（关） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.524 | — | — |
| 028 单机 | geom（关） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.756 | — | — |
| 028 单机 | geom（关） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.464 | — | — |
| 028 单机 | geom（关） | 4 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 0 | 0 | 0 | 1 | 0.499 | — | — |
| 028 单机 | geom（关） | 5 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 0 | 0 | 0 | 1 | 0.460 | — | — |
| 028 单机 | geom（关） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.486 | — | — |
| 028 单机 | geom（关） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.720 | — | — |
| 028 单机 | geom（关） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.576 | — | — |
| 028 单机 | geom（关） | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.431 | — | — |
| 028 单机 | geom + ground_guard | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.515 | — | — |
| 028 单机 | geom + ground_guard | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.503 | — | — |
| 028 单机 | geom + ground_guard | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.756 | — | — |
| 028 单机 | geom + ground_guard | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.526 | — | — |
| 028 单机 | geom + ground_guard | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.600 | — | — |
| 028 单机 | geom + ground_guard | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.556 | — | — |
| 028 单机 | geom + ground_guard | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.410 | — | — |
| 028 单机 | geom + ground_guard | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.720 | — | — |
| 028 单机 | geom + ground_guard | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.582 | — | — |
| 028 单机 | geom + ground_guard | 9 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 0 | 0 | 0 | 1 | 0.453 | — | — |
| 028 单机 | geom + empty_brake + ground_guard | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.515 | — | — |
| 028 单机 | geom + empty_brake + ground_guard | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.524 | — | — |
| 028 单机 | geom + empty_brake + ground_guard | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.756 | — | — |
| 028 单机 | geom + empty_brake + ground_guard | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.464 | — | — |
| 028 单机 | geom + empty_brake + ground_guard | 4 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 0 | 0 | 0 | 1 | 0.499 | — | — |
| 028 单机 | geom + empty_brake + ground_guard | 5 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 0 | 0 | 0 | 1 | 0.460 | — | — |
| 028 单机 | geom + empty_brake + ground_guard | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.486 | — | — |
| 028 单机 | geom + empty_brake + ground_guard | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.720 | — | — |
| 028 单机 | geom + empty_brake + ground_guard | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.576 | — | — |
| 028 单机 | geom + empty_brake + ground_guard | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.431 | — | — |
| 028 单机 | geom_multi + empty_brake + ground_guard | 0 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 1 | 0 | 0 | 0 | 0.408 | — | — |
| 028 单机 | geom_multi + empty_brake + ground_guard | 1 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 1 | 0 | 0 | 0 | 0.455 | — | — |
| 028 单机 | geom_multi + empty_brake + ground_guard | 2 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 1 | 0 | 0 | 0 | 0.445 | — | — |
| 028 单机 | geom_multi + empty_brake + ground_guard | 3 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 1 | 0 | 0 | 0 | 0.433 | — | — |
| 028 单机 | geom_multi + empty_brake + ground_guard | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.474 | — | — |
| 028 单机 | geom_multi + empty_brake + ground_guard | 5 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 1 | 0 | 0 | 0 | 0.528 | — | — |
| 028 单机 | geom_multi + empty_brake + ground_guard | 6 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 1 | 0 | 0 | 0 | 0.462 | — | — |
| 028 单机 | geom_multi + empty_brake + ground_guard | 7 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 1 | 0 | 0 | 0 | 0.413 | — | — |
| 028 单机 | geom_multi + empty_brake + ground_guard | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0 | 0 | 0 | 0 | 0.479 | — | — |
| 028 单机 | geom_multi + empty_brake + ground_guard | 9 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 0 | 0 | 0 | 1 | 0.451 | — | — |
