# 票 E-C5b：走廊跟踪误差补偿，只报数

只报数。没有改 `yopo_l0/` 的默认、`core/`、已冻门、`scenes/suite.txt`、`docs/tickets.md` 和 README。`safe_radius` 仍是 0.05 m，`empty_brake` 仍是 false，`track_clearance` 仍是 false。打开 E-A1、`empty_brake` 或 `track_clearance` 的臂标「偏离上游」。0.08 m 是假设，用来盖 [E-C5](e-c5-corridor-radius.md) 记下的 0.058–0.073 m 跟踪差。未冻结，不进 `scenes/suite.txt`，不设 gates。

数字在 [e-c5b-track-clearance-runs.json](e-c5b-track-clearance-runs.json)。跑数 commit 是 `bdc188227183ba0ae5b77fd71775b9c2d9a586d0`。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。

## 结论

1. **全关和 base 对得上 E-C5。** 014 对向全关仍是到达 0/30、机间碰撞 10/10、圆柱净空 0.486–0.569 m、机间 0.622–0.698 m。base 是 0.05 m + E-A1 阈值 1.0 m、惩罚 0 + 已合入的 `empty_brake`（不再用 E-B4 的进程内刹停原型）：到达 14/30，`TRUTH_POSITION` 6，机间 0，最小机间 1.099–2.633 m。其中 4 次撞圆柱，参考净空 0.363–0.395 m，高出机体 0.013–0.045 m，跟踪差 0.058–0.073 m，主因都是跟踪误差。单机全关和 base 都是 9/10。唯一的 `TRUTH_POSITION` 是 seed 1，圆柱净空 0.505 m，中心离地面约 0.35 m，主因「其他」。两臂的障碍距离和仿真时长相同，因为没有邻机，E-A1 不改轨迹。

2. **`pred` 把解码器半径抬到 0.13 m，不治本，单机更差。** 结果字段里的 `safe_radius_m` 仍是 0.05 m，偏离标记来自开关，不来自半径本身。对向到达 17/30，撞圆柱从 4 次降到 2 次（seed 4、9），参考净空 0.372 m 和 0.424 m，跟踪差 0.078 m 和 0.103 m，主因仍是跟踪。单机从 9/10 降到 6/10，新出现 4 次撞圆柱（seed 1、2、5、8）。这 4 次的参考净空是 0.306–0.319 m，已经小于机体半径 0.35 m，主因是准入松。预测走廊加 0.08 m 换了一条几何上更贴树的参考，没有把表面净空补到机体加跟踪差。

3. **`geom` 把对向的撞圆柱打到 0，单机 10/10，没有停在起点。** 准入仍是 0.05 m。候选按分数从高到低看参考轨迹（0.02 s 采样到 `t_end`）对障碍表面的最小净空，要求 ≥ 0.35 + 0.08 = 0.43 m。不合格就看下一个准入候选；全部不合格走 `CORRIDOR_EMPTY_HOVER`，本臂同时开了 `empty_brake`，从参考头刹停。对向到达 24/30，撞树 2，撞圆柱 0，机间 0，圆柱净空 0.398–0.552 m，机间 0.856–1.327 m。剩下的两次 `TRUTH_POSITION`（seed 2、9）圆柱净空是 0.446 m 和 0.398 m，中心离地面 0.349 m 和 0.347 m，主因记成深度感知盲区。单机 10/10，最小树净空 0.403–0.528 m。这和把 `safe_radius` 收到 0.35 m 不是同一件事：0.35 m 把预测走廊收空，单机会停在起点；这里 0.05 m 的准入还留着合格候选。单机 seed 6 有 1 拍无解刹停，其余种子刹停拍是 0。对向每粒刹停拍 0–7，没有停在起点。

4. **`geom_multi` 在三机上和 `geom` 十粒相同，在单机上和全关相同。** 几何门只在本帧 `_neighbors` 非空时开。三机十粒的到达、原因、树净空、机间、仿真时长和刹停拍与 `geom` 一致，第 0 个规划拍不启几何门没有改这张卡的结果。单机没有邻机，几何门全程关闭：到达 9/10，seed 1 仍是圆柱净空 0.505 m 的「其他」，仿真时长和树净空与全关、base 相同。多机补跟踪差，单机保持 0.05 m 走廊。

5. **和 E-A1 的顺序。** 先按走廊准入（`pred` 用 0.13 m 的解码器，其余用 0.05 m），再按表面净空筛可行集，然后才把这份可行集交给 `_apply_inter`。几何门滤空，或邻机过滤再滤空，都进原来的无解分支。对向 `geom` 的 seed 5 最小机间 0.856 m，高于 0.70 m 碰撞线，也低于 1.0 m 的 E-A1 门槛。

默认仍是全关。本票不改它。

## 第 0 拍

`runner` 在同一 tick 里先让每架 `plan()`，再 `bus.poll`。总线默认时延是 0，邻机采样要等这次规划返回之后才进 `note_neighbor_samples`。所以多机第 0 个规划拍（`k = 0`）`_neighbors` 仍是空的，`geom_multi` 这一拍不启几何门，和单机一样。下一拍规划在 planner 10 Hz 时是 t = 0.1 s，上一拍的 `peer_plan` 已经带了 `traj_samples`，几何门才开。

邻机采样在 E-A1 或 E-B3 打开时本来就会广播。`track_clearance` 打开且 mode 为 `geom_multi` 时，即使这两项都关，`note_neighbor_samples` 也收样本，运行器也会把轨迹采样放进 `peer_plan`。`pred` 和 `geom` 不靠邻机决定开不开门。单机没有 `peer_plan`，`_neighbors` 一直是空的。

## 卡和臂

| 卡 | 文件 | 机数 | 场景卡哈希（文件，五组相同） |
|---|---|---|---|
| 014 对向 | `scenes/ego-density-014-swap3.json` | 3 | `1157ac52b54f051a51f6f60086ae1ed788ff2d6661b485ceb9ee384cd699d352` |
| 014 单机 | `scenes/ego-density-014-solo.json` | 1 | `59871fbb9b736657cd564eb41c389e39ee9136c6ce8a37a657db68cb481b29c2` |

两张卡的文件哈希与 [E-C5](e-c5-corridor-radius.md) 相同。几何 `scenes/ego-density-014.json` 的 sha256 仍是 `5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9`。

| 臂 | 进程里多出来的 | 偏离上游 |
|---|---|---|
| 默认全关 | 没有。不写 `policy_params` | 否 |
| 0.05 m + E-A1 1.0 m + empty_brake | `inter_drone_filter`，阈值 1.0 m，惩罚 0；`empty_brake=true` | 偏离上游 |
| base + track pred 0.08 m | 上面再加上 `track_clearance=true`，`mode=pred`，`track_margin_m=0.08`。解码器半径是 0.13 m | 偏离上游 |
| base + track geom 0.08 m | `mode=geom`。表面净空 ≥ 0.35 + 0.08 m | 偏离上游 |
| base + track geom_multi 0.08 m | `mode=geom_multi`。只在本帧有邻机时做同一道几何门 | 偏离上游 |

参数不写进场景卡。`safe_radius` 五组都不覆盖，运行器缺省 0.05 m。进程内卡的哈希（seed 0 那一行）：对向 base `9ebd4e9a4b0a08871193239bf4ad4f6d6f73c000ea95f54a3bf84a43675f138b`，pred `bcb7dacf4147d70d8214bbfc6757c9cdd0d0243ad6c9140f21b0839b2902d480`，geom `f1018f81494c836d25609da7bf60afb25ca4f466d2f8d039cce963ef207321ae`，geom_multi `be1823725cdddfbbd188376ef80805c082941bb4de622a555dbc451576d607ba`。单机 base `e3ca47a9b95ab4234d27dc7cbfdac9be51d2f4cc39e98465f9d29d7146d95e0a`，pred `a979419744f8df79a0a31cbbca1a65ccf382f990f0b44aed4fabccd135153306`，geom `5b6c4b7b65507b72bcd0d66983bdcfc4248f0e5875ef0083d8998fc6d4fd8e23`，geom_multi `93d6e7f0bf46616f039ce9f2330ff9d1aaf9553259ae8ce64b6191a805c681ac`。影子盾开，只记录。seeds 0–9。飞行卡本身仍只声明 seed 0。

## 汇总

到达按架次。撞树、机间碰撞按运行器，仿真在第一次碰撞停下，所以一次运行最多记 1。这里的「撞树」是 `TRUTH_POSITION`：机体中心到任一表面（含地面）≤ 0.35 m。撞圆柱是全程圆柱表面净空 ≤ 0.35 m，不含地面。最小树净空只量圆柱表面。最小机间是中心距。单机写 —。偏离上游列只看开关，不看半径。

| 卡 | 臂 | 偏离上游 | 到达 | 撞树 | 撞圆柱 | 机间碰撞 | 超时 | 最小树净空 m | 最小机间 m |
|---|---|---|---|---|---|---|---|---|---|
| 014 对向 | 默认全关 | 否 | 0/30 | 0 | 0 | 10 | 0 | 0.486–0.569 | 0.622–0.698 |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 偏离上游 | 14/30 | 6 | 4 | 0 | 0 | 0.324–0.566 | 1.099–2.633 |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 偏离上游 | 17/30 | 5 | 2 | 0 | 0 | 0.344–0.490 | 1.121–1.357 |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 偏离上游 | 24/30 | 2 | 0 | 0 | 0 | 0.398–0.552 | 0.856–1.327 |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 偏离上游 | 24/30 | 2 | 0 | 0 | 0 | 0.398–0.552 | 0.856–1.327 |
| 014 单机 | 默认全关 | 否 | 9/10 | 1 | 0 | 0 | 0 | 0.385–0.561 | — |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 偏离上游 | 9/10 | 1 | 0 | 0 | 0 | 0.385–0.561 | — |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 偏离上游 | 6/10 | 4 | 4 | 0 | 0 | 0.347–0.474 | — |
| 014 单机 | base + track geom 0.08 m（偏离上游） | 偏离上游 | 10/10 | 0 | 0 | 0 | 0 | 0.403–0.528 | — |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 偏离上游 | 9/10 | 1 | 0 | 0 | 0 | 0.385–0.561 | — |

对向全关的机间 0.622–0.698 m、base 的 14/30、障碍树净空 0.324–0.566 m、机间 1.099–2.633 m，与 E-C5 的 0.05 m 全关列和刹停列相同。seed 0 的 base 仍是到达 3/3，机间 1.653 m，树净空 0.519 m，仿真 16.32 s。

## 撞树怎么归因

复用 [E-C5](e-c5-corridor-radius.md) 的顺序。互斥。深度图里没有这件障碍时，不盖过已经判成跟踪或准入松的那条。`empty_brake` 开着且该次规划码是 `CORRIDOR_EMPTY_HOVER` 时，记成「无解刹停后撞」。全关臂没有刹停，空走廊仍是上游原地悬停。

| 卡 | 臂 | 撞树 | 跟踪误差 | 准入松 | 深度感知盲区 | 无解刹停后撞 | 无解悬停后撞 | 其他 |
|---|---|---|---|---|---|---|---|---|
| 014 对向 | 默认全关 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 6 | 4 | 0 | 1 | 0 | 0 | 1 |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 5 | 2 | 0 | 3 | 0 | 0 | 0 |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 2 | 0 | 0 | 2 | 0 | 0 | 0 |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 2 | 0 | 0 | 2 | 0 | 0 | 0 |
| 014 单机 | 默认全关 | 1 | 0 | 0 | 0 | 0 | 0 | 1 |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 1 | 0 | 0 | 0 | 0 | 0 | 1 |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 4 | 0 | 4 | 0 | 0 | 0 | 0 |
| 014 单机 | base + track geom 0.08 m（偏离上游） | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 1 | 0 | 0 | 0 | 0 | 0 | 1 |

对向 base 的 4 次跟踪误差就是 E-C5 那 4 次撞圆柱。`geom` 和 `geom_multi` 的 2 次撞树都记成深度感知盲区，全程圆柱净空仍大于 0.35 m。`pred` 单机的 4 次撞树都是准入松，也是撞圆柱。

撞圆柱（全程圆柱净空 ≤ 0.35 m）这一列：

| 卡 | 臂 | seed | 参考净空 m | 高出 0.35 m | 圆柱真值 m | 跟踪差 m | 深度里有 | 主因 |
|---|---|---|---|---|---|---|---|---|
| 014 对向 | base（偏离上游） | 3 | 0.395 | 0.045 | 0.346 | 0.073 | 有 | 跟踪误差 |
| 014 对向 | base（偏离上游） | 4 | 0.387 | 0.037 | 0.347 | 0.069 | 有 | 跟踪误差 |
| 014 对向 | base（偏离上游） | 6 | 0.366 | 0.016 | 0.324 | 0.058 | 没有 | 跟踪误差 |
| 014 对向 | base（偏离上游） | 7 | 0.363 | 0.013 | 0.341 | 0.063 | 有 | 跟踪误差 |
| 014 对向 | pred（偏离上游） | 4 | 0.372 | 0.022 | 0.344 | 0.078 | 有 | 跟踪误差 |
| 014 对向 | pred（偏离上游） | 9 | 0.424 | 0.074 | 0.346 | 0.103 | 有 | 跟踪误差 |
| 014 单机 | pred（偏离上游） | 1 | 0.310 | −0.040 | 0.347 | 0.073 | 没有 | 准入松 |
| 014 单机 | pred（偏离上游） | 2 | 0.319 | −0.031 | 0.350 | 0.070 | 没有 | 准入松 |
| 014 单机 | pred（偏离上游） | 5 | 0.306 | −0.044 | 0.350 | 0.074 | 有 | 准入松 |
| 014 单机 | pred（偏离上游） | 8 | 0.311 | −0.039 | 0.347 | 0.074 | 没有 | 准入松 |

`geom` 和 `geom_multi` 没有撞圆柱。对向这两臂剩下的两次 `TRUTH_POSITION`：

| 卡 | 臂 | seed | 飞机 | 主因 | 计划净空 m | 圆柱真值 m | 地面附近的障碍距离 m | 跟踪差 m | 深度里有 | 规划码 |
|---|---|---|---|---|---|---|---|---|---|---|
| 014 对向 | geom 与 geom_multi（偏离上游） | 2 | uav1 | 深度感知盲区 | 0.456 | 0.446 | 0.349 | 0.108 | 没有 | `OK` |
| 014 对向 | geom 与 geom_multi（偏离上游） | 9 | uav1 | 深度感知盲区 | 0.435 | 0.398 | 0.347 | 0.103 | 没有 | `CORRIDOR_EMPTY_HOVER` |

全部 `TRUTH_POSITION` 的归因明细：

| 卡 | 臂 | seed | 飞机 | 主因 | 计划净空 m | 飞行段净空 m | 真值净空 m | 跟踪差 m | 深度里有 | 最紧已准入走廊 m | 规划码 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 2 | uav1 | 深度感知盲区 | 0.437 | 0.437 | 0.390 | 0.131 | 没有 | 0.058 | `OK` |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 3 | uav0 | 跟踪误差 | 0.395 | 0.395 | 0.346 | 0.073 | 有 | 0.105 | `OK` |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 4 | uav0 | 跟踪误差 | 0.387 | 0.387 | 0.347 | 0.069 | 有 | 0.164 | `OK` |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 6 | uav0 | 跟踪误差 | 0.366 | 0.366 | 0.324 | 0.058 | 没有 | 0.164 | `OK` |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 7 | uav0 | 跟踪误差 | 0.363 | 0.363 | 0.341 | 0.063 | 有 | 0.053 | `OK` |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 9 | uav1 | 其他 | 0.696 | 0.696 | 0.612 | 0.114 | 有 | 0.064 | `OK` |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 2 | uav1 | 深度感知盲区 | 0.437 | 0.437 | 0.390 | 0.131 | 没有 | 0.256 | `OK` |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 4 | uav0 | 跟踪误差 | 0.372 | 0.372 | 0.344 | 0.078 | 有 | 0.137 | `OK` |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 5 | uav1 | 深度感知盲区 | 0.547 | 0.547 | 0.930 | 0.106 | 没有 | 0.273 | `OK` |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 8 | uav1 | 深度感知盲区 | 0.460 | 0.460 | 0.787 | 0.076 | 没有 | 0.245 | `OK` |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 9 | uav1 | 跟踪误差 | 0.424 | 0.424 | 0.346 | 0.103 | 有 | 0.139 | `OK` |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 2 | uav1 | 深度感知盲区 | 0.456 | 0.456 | 0.464 | 0.108 | 没有 | 0.076 | `OK` |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 9 | uav1 | 深度感知盲区 | 0.435 | — | 0.398 | 0.103 | 没有 | 0.123 | `CORRIDOR_EMPTY_HOVER` |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 2 | uav1 | 深度感知盲区 | 0.456 | 0.456 | 0.464 | 0.108 | 没有 | 0.076 | `OK` |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 9 | uav1 | 深度感知盲区 | 0.435 | — | 0.398 | 0.103 | 没有 | 0.123 | `CORRIDOR_EMPTY_HOVER` |
| 014 单机 | 默认全关 | 1 | uav0 | 其他 | 0.533 | 0.533 | 0.801 | 0.112 | 有 | 0.056 | `OK` |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 1 | uav0 | 其他 | 0.533 | 0.533 | 0.801 | 0.112 | 有 | 0.056 | `OK` |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 1 | uav0 | 准入松 | 0.310 | 0.310 | 0.347 | 0.073 | 没有 | 0.147 | `OK` |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 2 | uav0 | 准入松 | 0.319 | 0.319 | 0.350 | 0.070 | 没有 | 0.139 | `OK` |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 5 | uav0 | 准入松 | 0.306 | 0.306 | 0.350 | 0.074 | 有 | 0.142 | `OK` |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 8 | uav0 | 准入松 | 0.311 | 0.311 | 0.347 | 0.074 | 没有 | 0.141 | `OK` |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 1 | uav0 | 其他 | 0.533 | 0.533 | 0.801 | 0.112 | 有 | 0.056 | `OK` |

## 分种子

刹停拍是 `empty_brake` 且规划码为 `CORRIDOR_EMPTY_HOVER` 的规划次数，三架合计。全关臂是 0。

| 卡 | 臂 | seed | 结果 | 原因 | 到达 | 最小树净空 m | 最小机间 m | 仿真 s | 刹停拍 |
|---|---|---|---|---|---|---|---|---|---|
| 014 对向 | 默认全关 | 0 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.486 | 0.634 | 2.60 | 0 |
| 014 对向 | 默认全关 | 1 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.531 | 0.639 | 2.61 | 0 |
| 014 对向 | 默认全关 | 2 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.569 | 0.629 | 2.60 | 0 |
| 014 对向 | 默认全关 | 3 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.556 | 0.698 | 2.60 | 0 |
| 014 对向 | 默认全关 | 4 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.550 | 0.622 | 2.61 | 0 |
| 014 对向 | 默认全关 | 5 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.546 | 0.684 | 2.61 | 0 |
| 014 对向 | 默认全关 | 6 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.517 | 0.683 | 2.61 | 0 |
| 014 对向 | 默认全关 | 7 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.553 | 0.658 | 2.60 | 0 |
| 014 对向 | 默认全关 | 8 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.530 | 0.666 | 2.60 | 0 |
| 014 对向 | 默认全关 | 9 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.536 | 0.629 | 2.61 | 0 |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.519 | 1.653 | 16.32 | 0 |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.357 | 1.310 | 8.89 | 0 |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 2 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.378 | 1.134 | 4.41 | 0 |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 3 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 0.346 | 2.411 | 2.66 | 0 |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 4 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 0.347 | 2.633 | 2.63 | 0 |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.354 | 1.167 | 16.72 | 1 |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 6 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 0.324 | 1.281 | 2.88 | 1 |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 7 | COLLISION | `uav0:TRUTH_POSITION` | 2/3 | 0.341 | 1.136 | 8.65 | 0 |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.566 | 1.308 | 16.72 | 0 |
| 014 对向 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.523 | 1.099 | 4.35 | 2 |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.420 | 1.178 | 9.66 | 0 |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.393 | 1.289 | 15.23 | 1 |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 2 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.378 | 1.134 | 4.41 | 0 |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.442 | 1.357 | 9.94 | 0 |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 4 | COLLISION | `uav0:TRUTH_POSITION` | 2/3 | 0.344 | 1.197 | 9.71 | 2 |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 5 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.449 | 1.121 | 4.51 | 1 |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.490 | 1.272 | 17.97 | 0 |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.456 | 1.136 | 17.21 | 0 |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 8 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.405 | 1.134 | 4.70 | 1 |
| 014 对向 | base + track pred 0.08 m（偏离上游） | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.346 | 1.159 | 4.49 | 3 |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.469 | 1.195 | 9.69 | 5 |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.481 | 1.298 | 17.32 | 1 |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 2 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.446 | 1.134 | 4.56 | 4 |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.407 | 1.242 | 18.07 | 4 |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.446 | 1.278 | 9.76 | 1 |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.552 | 0.856 | 17.02 | 3 |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.438 | 1.327 | 9.06 | 0 |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.449 | 1.076 | 18.16 | 7 |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.548 | 1.109 | 8.97 | 0 |
| 014 对向 | base + track geom 0.08 m（偏离上游） | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.398 | 1.159 | 4.63 | 7 |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.469 | 1.195 | 9.69 | 5 |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.481 | 1.298 | 17.32 | 1 |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 2 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.446 | 1.134 | 4.56 | 4 |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.407 | 1.242 | 18.07 | 4 |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.446 | 1.278 | 9.76 | 1 |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.552 | 0.856 | 17.02 | 3 |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.438 | 1.327 | 9.06 | 0 |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.449 | 1.076 | 18.16 | 7 |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.548 | 1.109 | 8.97 | 0 |
| 014 对向 | base + track geom_multi 0.08 m（偏离上游） | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.398 | 1.159 | 4.63 | 7 |
| 014 单机 | 默认全关 | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.491 | — | 13.49 | 0 |
| 014 单机 | 默认全关 | 1 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 0.505 | — | 4.40 | 0 |
| 014 单机 | 默认全关 | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.499 | — | 13.39 | 0 |
| 014 单机 | 默认全关 | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.385 | — | 13.38 | 0 |
| 014 单机 | 默认全关 | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.561 | — | 13.58 | 0 |
| 014 单机 | 默认全关 | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.471 | — | 13.49 | 0 |
| 014 单机 | 默认全关 | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.491 | — | 13.50 | 0 |
| 014 单机 | 默认全关 | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.389 | — | 13.69 | 0 |
| 014 单机 | 默认全关 | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.466 | — | 13.50 | 0 |
| 014 单机 | 默认全关 | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.539 | — | 13.49 | 0 |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.491 | — | 13.49 | 0 |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 1 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 0.505 | — | 4.40 | 0 |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.499 | — | 13.39 | 0 |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.385 | — | 13.38 | 0 |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.561 | — | 13.58 | 0 |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.471 | — | 13.49 | 0 |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.491 | — | 13.50 | 0 |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.389 | — | 13.69 | 0 |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.466 | — | 13.50 | 0 |
| 014 单机 | 0.05 m + E-A1 1.0 m + empty_brake（偏离上游） | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.539 | — | 13.49 | 0 |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.474 | — | 9.85 | 0 |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 1 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 0.347 | — | 0.94 | 0 |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 2 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 0.350 | — | 0.94 | 0 |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.365 | — | 13.39 | 0 |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.350 | — | 13.49 | 0 |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 5 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 0.350 | — | 0.90 | 0 |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.365 | — | 13.48 | 0 |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.459 | — | 9.97 | 0 |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 8 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 0.347 | — | 0.93 | 0 |
| 014 单机 | base + track pred 0.08 m（偏离上游） | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.441 | — | 10.99 | 0 |
| 014 单机 | base + track geom 0.08 m（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.502 | — | 9.49 | 0 |
| 014 单机 | base + track geom 0.08 m（偏离上游） | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.522 | — | 13.87 | 0 |
| 014 单机 | base + track geom 0.08 m（偏离上游） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.527 | — | 9.76 | 0 |
| 014 单机 | base + track geom 0.08 m（偏离上游） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.521 | — | 13.39 | 0 |
| 014 单机 | base + track geom 0.08 m（偏离上游） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.458 | — | 13.20 | 0 |
| 014 单机 | base + track geom 0.08 m（偏离上游） | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.503 | — | 13.50 | 0 |
| 014 单机 | base + track geom 0.08 m（偏离上游） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.528 | — | 13.48 | 1 |
| 014 单机 | base + track geom 0.08 m（偏离上游） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.528 | — | 13.69 | 0 |
| 014 单机 | base + track geom 0.08 m（偏离上游） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.403 | — | 13.49 | 0 |
| 014 单机 | base + track geom 0.08 m（偏离上游） | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.475 | — | 13.39 | 0 |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.491 | — | 13.49 | 0 |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 1 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 0.505 | — | 4.40 | 0 |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.499 | — | 13.39 | 0 |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.385 | — | 13.38 | 0 |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.561 | — | 13.58 | 0 |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.471 | — | 13.49 | 0 |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.491 | — | 13.50 | 0 |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.389 | — | 13.69 | 0 |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.466 | — | 13.50 | 0 |
| 014 单机 | base + track geom_multi 0.08 m（偏离上游） | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.539 | — | 13.49 | 0 |

## 设定

| 项 | 值 |
|---|---|
| 跑数 commit | `bdc188227183ba0ae5b77fd71775b9c2d9a586d0` |
| 层级 | L0，`upstream`，影子盾开，只记录 |
| 脚本 | [e-c5b-track-clearance.py](../../scripts/e-c5b-track-clearance.py)。归因调用 [e-c5-corridor-radius.py](../../scripts/e-c5-corridor-radius.py) 的 `diagnose` / `classify_tree` |
| 开关 | 进程内覆盖。关时与现网逐位相同。开时标偏离上游 |
| seeds | 0–9 |
| 跑法 | `--jobs 4`，`nice 10`，每进程 1 条推理线程。轨迹不按墙钟截断 |
| 时间 | 2026-11-13 00:41–01:02 UTC |
| 单次墙钟合计 | 4851 s。墙钟不参与轨迹，不进门 |
| 报告 | 只报数。`report_only=true`，`frozen=false` |

## 已冻卡

结果不写入 `scenes/suite.txt`，不设 `criteria.frozen`，不设 gates，也不改已有冻结卡的判据。不把 `track_clearance` 或 0.35 m 改成默认。

## 怎么重跑

```bash
make venv
git rev-parse HEAD
.venv/bin/python -m pytest -q tests/test_e_c5b_track_clearance.py
.venv/bin/python scripts/e-c5b-track-clearance.py --check
.venv/bin/python -u scripts/e-c5b-track-clearance.py --jobs 2 --out docs/research/e-c5b-track-clearance-runs.json
```

`--check` 核对几何 sha256、场景文件没被改写、全关臂不写 `policy_params`、三档 track 臂带上 base 的 E-A1 和 `empty_brake`、默认开关仍是关。部分结果在 `out/e-c5b-track-clearance/partial/`，默认可续跑。全关臂在同 seed 下，到达、碰撞、最小距离和仿真时长不该变。没有后段墙钟预算。
