# 票 E-C5：走廊准入 safe_radius 对照，只报数

只报数。没有改 `yopo_l0/` 的默认、`core/`、已冻门、`scenes/suite.txt`、`docs/tickets.md` 和 README，也没有把 0.20 m 或 0.35 m 写成默认。0.05 m 仍是上游 d5e1e60 的走廊准入半径。0.20 m 和 0.35 m 在臂名上标「偏离上游」。刹停用的是 [e-b4-peer-diag.py](../../scripts/e-b4-peer-diag.py) 里的 `_proto_brake`，只在本脚本进程里替换无解分支，不是已合入的开关。

数字在 [e-c5-corridor-radius-runs.json](e-c5-corridor-radius-runs.json)。跑数 commit 是 `3f3d1ba8c7417daabe707421cc2d107c65ff7e12`。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。

## 结论

1. **E-A1 关着时，收紧走廊挡不住三机对穿。** 014 对向三档半径都是到达 0/30、机间碰撞 10/10、运行器撞树 0。圆柱净空随半径升高：0.05 m 是 0.486–0.569 m，0.20 m 是 0.545–0.850 m，0.35 m 是 0.822–0.849 m。机间仍是 0.621–0.699 m。越线那一拍的参考机间距是 0.717–0.852 m，真值低 0.17 m 左右（跟踪差 0.167–0.175 m），计划还在 0.70 m 线外。这和 [E-B4](e-b4-peer-collision-diag.md) 全关列、[E-C4b](e-c4b-density-seeds.md) 的 0.05 / 0.35 全关列是同一件事。

2. **E-B4 那 14/30 里，真正撞进圆柱的 4 次都是跟踪误差，不是准入已经把机体放进树里。** 0.05 m + E-A1 1.0 m + 刹停原型重跑与 E-B4 修复臂一致：到达 14/30，`TRUTH_POSITION` 6，机间 0，最小机间 1.099–2.633 m。这 6 次里，全程圆柱净空 ≤ 0.35 m 的是 seed 3、4、6、7。参考净空 0.363–0.395 m，只比机体半径多 0.013–0.045 m，跟踪差 0.058–0.073 m，余量小于跟踪差。seed 6 的深度图里没有这棵树，参考仍在机体外 0.016 m，主因仍记跟踪。seed 2 和 seed 9 的圆柱净空是 0.378 m 和 0.523 m，运行器是在中心离地面 0.348 m 时停下。按「飞行段参考净空 ≤ 0.35 m」这条，0.05 m 这一臂的准入松是 0。

3. **同一张三机卡，半径收到 0.35 m（偏离上游）之后是 30/30。** 撞树 0，机间 0，圆柱净空 0.390–0.715 m（最近是 seed 9 的 0.390 m），机间 1.188–1.251 m。走廊空的时候按 a_max 刹停，三架合计每粒 7–17 拍，没有停在起点。

4. **0.20 m（偏离上游）不是中间的改善。** 三机到达 15/30，比 0.05 m 多 1 架次。圆柱真正进去 2 次：seed 3 在刹停上跟踪进去（参考 0.466 m，真值 0.349 m，跟踪差 0.197 m）；seed 4 是这一整批里唯一的准入松（参考净空 0.344 m，深度图里有这棵树）。最小机间降到 seed 5 的 0.837 m，仍高于 0.70 m 碰撞线，也低于 1.0 m 门槛。

5. **单机上收紧半径会把走廊收空。** 0.05 m 两臂都是 9/10。唯一的 `TRUTH_POSITION` 是 seed 1，圆柱净空 0.505 m，地面 0.349 m，和 E-C4b 记的「其他」是同一次；两臂的障碍距离和仿真时长相同（0.349 m，4.40 s），因为没有邻机，过滤不改轨迹。0.35 m 两臂都是 0/10、超时 10，最后 5 s 进度 0、速度 0，50/50 拍是 `CORRIDOR_EMPTY_HOVER`，净空锁在起点 1.190 m，与 E-C4b 的 0.35 单机相同。0.20 m 大部分也是这样停在起点。关着的臂 seed 6 真撞圆柱（参考 0.367 m，真值 0.347 m，跟踪差 0.093 m，深度里有），刹停臂上同一次数字相同；刹停臂的 seed 5 到达，关着的臂这一粒是悬停后降到地面（圆柱净空 0.513 m）。

三机对向在 E-A1 1.0 m + 刹停下，0.05 m 留下的圆柱碰撞是参考只在机体外 1–4 cm，盖不住 6–7 cm 的跟踪。收到机体半径 0.35 m 之后这张卡 30/30，但同一档在单机上 10/10 停在起点。默认仍是 0.05 m。本票不改它。

## 卡和臂

| 卡 | 文件 | 机数 | 场景卡哈希（文件，两档半径相同） |
|---|---|---|---|
| 014 对向 | `scenes/ego-density-014-swap3.json` | 3 | `1157ac52b54f051a51f6f60086ae1ed788ff2d6661b485ceb9ee384cd699d352` |
| 014 单机 | `scenes/ego-density-014-solo.json` | 1 | `59871fbb9b736657cd564eb41c389e39ee9136c6ce8a37a657db68cb481b29c2` |

两张卡的文件哈希与 [E-C4](e-c4-density-l0.md) 全关列相同。几何 `scenes/ego-density-014.json` 的 sha256 仍是 `5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9`。

| 臂 | 进程里多出来的 | 偏离上游 |
|---|---|---|
| safe_radius 0.05 m，E-A1 关 | 没有。`RunConfig.safe_radius_m` 不传，运行器缺省 0.05 m | 否 |
| safe_radius 0.20 m，E-A1 关 | `RunConfig.safe_radius_m=0.20` | 偏离上游 |
| safe_radius 0.35 m，E-A1 关 | `RunConfig.safe_radius_m=0.35` | 偏离上游 |
| 上面三档 + E-A1 1.0 m + 刹停原型 | `inter_drone_filter`，阈值 1.0 m，惩罚 0；无解时走 E-B4 `_proto_brake`（从上一份参考头按 a_max 刹停） | 半径不是 0.05 m 才标 |

半径不写进场景卡。E-A1 只在进程内的卡上，三档半径共用同一份内存哈希：对向 `981d2e4121cbedb483303ae07ca9b6f4c4e9db2dd908ab99eda6daf97245fa81`，单机 `fdfb43c77110c184c623a9293cb48546cd8531a61852936a4098c4a83e0751fa`。影子盾开，只记录。seeds 0–9，飞行卡本身仍只声明 seed 0。

## 汇总

到达按架次。撞树、机间碰撞按运行器，仿真在第一次碰撞停下，所以一次运行最多记 1。这里的「撞树」是 `TRUTH_POSITION`：机体中心到任一表面（含地面）≤ 0.35 m。最小树净空只量圆柱表面，不含地面，所以这两列可以不一致。最小机间是中心距。单机写 —。

| 卡 | 臂 | 到达 | 撞树 | 机间碰撞 | 超时 | 最小树净空 m | 最小机间 m |
|---|---|---|---|---|---|---|---|
| 014 对向 | safe_radius 0.05 m，E-A1 关 | 0/30 | 0 | 10 | 0 | 0.486–0.569 | 0.622–0.698 |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 14/30 | 6 | 0 | 0 | 0.324–0.566 | 1.099–2.633 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 关 | 0/30 | 0 | 10 | 0 | 0.545–0.850 | 0.621–0.698 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 15/30 | 5 | 0 | 0 | 0.336–0.563 | 0.837–1.303 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 关 | 0/30 | 0 | 10 | 0 | 0.822–0.849 | 0.643–0.699 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 30/30 | 0 | 0 | 0 | 0.390–0.715 | 1.188–1.251 |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 9/10 | 1 | 0 | 0 | 0.385–0.561 | — |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 9/10 | 1 | 0 | 0 | 0.385–0.561 | — |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 0/10 | 2 | 0 | 8 | 0.347–1.190 | — |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 1/10 | 1 | 0 | 8 | 0.347–1.190 | — |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 关 | 0/10 | 0 | 0 | 10 | 1.190 | — |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 0/10 | 0 | 0 | 10 | 1.190 | — |

0.05 m 对向全关的机间 0.622–0.698 m、0.35 m 全关的圆柱净空 0.822–0.849 m 和机间 0.643–0.699 m，与 E-B4、E-C4b 已写下的范围相同。0.05 m + 刹停的 14/30、障碍 0.324–0.566 m、机间 1.099–2.633 m 与 E-B4 修复臂相同；seed 0 仍是到达 3/3，机间 1.6529 m，障碍 0.5189 m，仿真 16.32 s。

## 撞树怎么归因

互斥，按这个顺序。深度图里没有这件障碍时，不盖过已经判成跟踪或准入松的那条。

1. **跟踪误差**：最近 0.5 s 的参考点到圆柱表面仍大于机体半径 0.35 m，机体已经 ≤ 0.35 m。
2. **准入松**：这 0.5 s 里有一段已准入的 `OK` 计划，参考净空 ≤ 0.35 m。
3. **深度感知盲区**：撞击前最后一条计划的深度图里没有最近那棵圆柱。
4. **无解刹停后撞 / 无解悬停后撞**：走廊空了。刹停臂走原型，关着的臂仍是上游 `hold_traj`。
5. **其他**。

「最紧已准入走廊」是该次前向里已准入候选的最小预测半径（μ 的反函数），不是飞出去那条多项式的几何净空。几何净空看「计划净空」。

| 卡 | 臂 | 撞树 | 跟踪误差 | 准入松 | 深度感知盲区 | 无解刹停后撞 | 无解悬停后撞 | 其他 |
|---|---|---|---|---|---|---|---|---|
| 014 对向 | safe_radius 0.05 m，E-A1 关 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 6 | 4 | 0 | 1 | 0 | 0 | 1 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 关 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 5 | 1 | 1 | 2 | 0 | 0 | 1 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 关 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 1 | 0 | 0 | 0 | 0 | 0 | 1 |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 1 | 0 | 0 | 0 | 0 | 0 | 1 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 2 | 1 | 0 | 0 | 0 | 1 | 0 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 1 | 1 | 0 | 0 | 0 | 0 | 0 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 关 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

16 次 `TRUTH_POSITION` 里，8 次全程圆柱净空仍大于 0.35 m，停下来是因为中心离地面约 0.346–0.350 m。另外 8 次圆柱净空 ≤ 0.35 m，才是撞树：

| 卡 | 臂 | seed | 参考净空 m | 高出 0.35 m | 圆柱真值 m | 跟踪差 m | 深度里有 | 主因 |
|---|---|---|---|---|---|---|---|---|
| 014 对向 | 0.05 m + 刹停 | 3 | 0.395 | 0.045 | 0.346 | 0.073 | 有 | 跟踪误差 |
| 014 对向 | 0.05 m + 刹停 | 4 | 0.387 | 0.037 | 0.347 | 0.069 | 有 | 跟踪误差 |
| 014 对向 | 0.05 m + 刹停 | 6 | 0.366 | 0.016 | 0.324 | 0.058 | 没有 | 跟踪误差 |
| 014 对向 | 0.05 m + 刹停 | 7 | 0.363 | 0.013 | 0.341 | 0.063 | 有 | 跟踪误差 |
| 014 对向 | 0.20 m + 刹停（偏离上游） | 3 | 0.466 | 0.116 | 0.349 | 0.197 | 有 | 跟踪误差（刹停中） |
| 014 对向 | 0.20 m + 刹停（偏离上游） | 4 | 0.344 | −0.006 | 0.336 | 0.146 | 有 | 准入松 |
| 014 单机 | 0.20 m，E-A1 关和刹停各一次（偏离上游） | 6 | 0.367 | 0.017 | 0.347 | 0.093 | 有 | 跟踪误差 |

0.05 m 的 4 次撞圆柱，参考只多出 0.013–0.045 m，跟踪差 0.058–0.073 m。已准入候选里最紧的预测半径是 0.053–0.164 m，刚过 0.05 m 的门，但飞出去的参考几何净空是 0.363–0.395 m。这两个数不是同一个量。seed 6 深度图里没有树，主因不改成盲区，因为参考还在机体外面。

判成「深度感知盲区」的 3 次（对向 0.05 m seed 2，0.20 m seed 0 和 9）最近圆柱不在深度图里，但这 3 次的全程圆柱净空是 0.378 m、0.465 m、0.441 m，运行器停下时地面已经到了 0.346–0.350 m。按圆柱碰撞算，盲区是 0。判成「其他」的 4 次同样是圆柱还在 0.420–0.801 m、地面已经到 0.35 m 附近，其中单机 0.05 m seed 1 两臂是同一次。0.20 m 单机 seed 5（E-A1 关）是无解悬停之后中心降到地面，圆柱净空 0.513 m。

0.35 m + 刹停在对向上没有 `TRUTH_POSITION`。单机 0.35 m 不是撞树，是 10/10 超时卡死：进度 0 m，均速 0，净空 1.190 m。0.20 m 单机 16 次超时里 14 次同样卡在起点；两臂的 seed 1 仍在往前飞（关：末段目标误差 0.104 m、均速 2.48 m/s；刹停：目标误差 4.20 m、均速 3.40 m/s），时限到了，没有稳住。

| 卡 | 臂 | seed | 飞机 | 主因 | 计划净空 m | 飞行段净空 m | 真值净空 m | 跟踪差 m | 深度里有 | 最紧已准入走廊 m | 规划码 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 2 | uav1 | 深度感知盲区 | 0.437 | 0.437 | 0.390 | 0.131 | 没有 | 0.058 | `OK` |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 3 | uav0 | 跟踪误差 | 0.395 | 0.395 | 0.346 | 0.073 | 有 | 0.105 | `OK` |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 4 | uav0 | 跟踪误差 | 0.387 | 0.387 | 0.347 | 0.069 | 有 | 0.164 | `OK` |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 6 | uav0 | 跟踪误差 | 0.366 | 0.366 | 0.324 | 0.058 | 没有 | 0.164 | `OK` |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 7 | uav0 | 跟踪误差 | 0.363 | 0.363 | 0.341 | 0.063 | 有 | 0.053 | `OK` |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 9 | uav1 | 其他 | 0.696 | 0.696 | 0.612 | 0.114 | 有 | 0.064 | `OK` |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 0 | uav1 | 深度感知盲区 | 0.482 | 0.482 | 0.560 | 0.076 | 没有 | 0.270 | `OK` |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 3 | uav0 | 跟踪误差 | 0.466 | — | 0.349 | 0.197 | 有 | 0.243 | `CORRIDOR_EMPTY_HOVER` |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 4 | uav0 | 准入松 | 0.344 | 0.344 | 0.336 | 0.146 | 有 | 0.242 | `OK` |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 7 | uav1 | 其他 | 0.469 | 0.469 | 0.420 | 0.079 | 有 | 0.212 | `OK` |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 9 | uav1 | 深度感知盲区 | 0.466 | 0.466 | 0.466 | 0.084 | 没有 | 0.267 | `OK` |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 1 | uav0 | 其他 | 0.533 | 0.533 | 0.801 | 0.112 | 有 | 0.056 | `OK` |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 1 | uav0 | 其他 | 0.533 | 0.533 | 0.801 | 0.112 | 有 | 0.056 | `OK` |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 5 | uav0 | 无解悬停后撞 | 0.545 | 0.545 | 0.582 | 0.019 | 有 | 0.204 | `CORRIDOR_EMPTY_HOVER` |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 6 | uav0 | 跟踪误差 | 0.367 | 0.367 | 0.347 | 0.093 | 有 | 0.290 | `OK` |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 6 | uav0 | 跟踪误差 | 0.367 | 0.367 | 0.347 | 0.093 | 有 | 0.290 | `OK` |

## 分种子

| 卡 | 臂 | seed | 结果 | 原因 | 到达 | 最小树净空 m | 最小机间 m | 仿真 s | 刹停拍 |
|---|---|---|---|---|---|---|---|---|---|
| 014 对向 | safe_radius 0.05 m，E-A1 关 | 0 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.486 | 0.634 | 2.60 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 关 | 1 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.531 | 0.639 | 2.61 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 关 | 2 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.569 | 0.629 | 2.60 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 关 | 3 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.556 | 0.698 | 2.60 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 关 | 4 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.550 | 0.622 | 2.61 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 关 | 5 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.546 | 0.684 | 2.61 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 关 | 6 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.517 | 0.683 | 2.61 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 关 | 7 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.553 | 0.658 | 2.60 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 关 | 8 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.530 | 0.666 | 2.60 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 关 | 9 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.536 | 0.629 | 2.61 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.519 | 1.653 | 16.32 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.357 | 1.310 | 8.89 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 2 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.378 | 1.134 | 4.41 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 3 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 0.346 | 2.411 | 2.66 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 4 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 0.347 | 2.633 | 2.63 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.354 | 1.167 | 16.72 | 1 |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 6 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 0.324 | 1.281 | 2.88 | 1 |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 7 | COLLISION | `uav0:TRUTH_POSITION` | 2/3 | 0.341 | 1.136 | 8.65 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.566 | 1.308 | 16.72 | 0 |
| 014 对向 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.523 | 1.099 | 4.35 | 2 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 关 | 0 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.849 | 0.634 | 2.60 | 0 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 关 | 1 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.839 | 0.639 | 2.61 | 0 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 关 | 2 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.576 | 0.621 | 2.61 | 0 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 关 | 3 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.781 | 0.698 | 2.60 | 0 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 关 | 4 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.836 | 0.622 | 2.61 | 0 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 关 | 5 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.842 | 0.684 | 2.61 | 0 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 关 | 6 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.844 | 0.683 | 2.61 | 0 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 关 | 7 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.850 | 0.658 | 2.60 | 0 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 关 | 8 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.545 | 0.666 | 2.60 | 0 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 关 | 9 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.690 | 0.629 | 2.61 | 0 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 0 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.465 | 1.147 | 4.58 | 1 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.528 | 1.275 | 11.90 | 2 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.563 | 1.099 | 11.83 | 2 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 3 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 0.349 | 1.303 | 2.98 | 7 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 4 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 0.336 | 1.284 | 4.38 | 8 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.449 | 0.837 | 10.56 | 5 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.427 | 1.151 | 10.42 | 7 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 7 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.420 | 1.076 | 4.47 | 2 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.438 | 1.128 | 11.60 | 2 |
| 014 对向 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.441 | 1.071 | 4.54 | 8 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 关 | 0 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.837 | 0.697 | 2.67 | 0 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 关 | 1 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.828 | 0.686 | 2.67 | 0 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 关 | 2 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.841 | 0.699 | 2.67 | 0 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 关 | 3 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.849 | 0.688 | 2.67 | 0 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 关 | 4 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.833 | 0.643 | 2.67 | 0 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 关 | 5 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.848 | 0.666 | 2.68 | 0 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 关 | 6 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.826 | 0.671 | 2.68 | 0 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 关 | 7 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.826 | 0.683 | 2.67 | 0 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 关 | 8 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.840 | 0.664 | 2.67 | 0 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 关 | 9 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.822 | 0.660 | 2.68 | 0 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.532 | 1.249 | 10.32 | 8 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.715 | 1.251 | 11.20 | 11 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.663 | 1.215 | 11.68 | 11 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.424 | 1.221 | 11.76 | 17 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.578 | 1.228 | 10.86 | 9 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.426 | 1.244 | 10.74 | 9 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.599 | 1.198 | 11.16 | 13 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.467 | 1.251 | 10.77 | 7 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.517 | 1.216 | 9.65 | 13 |
| 014 对向 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.390 | 1.188 | 10.83 | 10 |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.491 | — | 13.49 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 1 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 0.505 | — | 4.40 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.499 | — | 13.39 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.385 | — | 13.38 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.561 | — | 13.58 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.471 | — | 13.49 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.491 | — | 13.50 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.389 | — | 13.69 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.466 | — | 13.50 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 关 | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.539 | — | 13.49 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.491 | — | 13.49 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 1 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 0.505 | — | 4.40 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.499 | — | 13.39 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.385 | — | 13.38 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.561 | — | 13.58 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.471 | — | 13.49 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.491 | — | 13.50 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.389 | — | 13.69 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.466 | — | 13.50 | 0 |
| 014 单机 | safe_radius 0.05 m，E-A1 1.0 m + 刹停原型 | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.539 | — | 13.49 | 0 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 0 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 1 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 0.468 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 2 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 3 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 4 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 5 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 0.513 | — | 29.54 | 0 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 6 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 0.347 | — | 2.44 | 0 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 7 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 8 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 关 | 9 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 0 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 1 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 0.500 | — | 50.00 | 414 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 2 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 3 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 4 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 1/1 | 0.557 | — | 38.28 | 246 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 6 | COLLISION | `uav0:TRUTH_POSITION` | 0/1 | 0.347 | — | 2.44 | 15 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 7 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 8 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.20 m（偏离上游），E-A1 1.0 m + 刹停原型 | 9 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 关 | 0 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 关 | 1 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 关 | 2 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 关 | 3 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 关 | 4 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 关 | 5 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 关 | 6 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 关 | 7 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 关 | 8 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 关 | 9 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 0 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 0 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 1 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 2 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 3 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 4 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 5 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 6 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 7 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 8 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |
| 014 单机 | safe_radius 0.35 m（偏离上游），E-A1 1.0 m + 刹停原型 | 9 | TIMEOUT | `GOAL_NOT_REACHED` | 0/1 | 1.190 | — | 50.00 | 500 |

## 设定

| 项 | 值 |
|---|---|
| 跑数 commit | `3f3d1ba8c7417daabe707421cc2d107c65ff7e12` |
| 层级 | L0，`upstream`，影子盾开，只记录 |
| 脚本 | [e-c5-corridor-radius.py](../../scripts/e-c5-corridor-radius.py)，刹停调用 [e-b4-peer-diag.py](../../scripts/e-b4-peer-diag.py) 的 `_make_policy_cls` / `_proto_brake` |
| seeds | 0–9 |
| 跑法 | `--jobs 4`，`nice 10`，每进程 1 条推理线程。轨迹不按墙钟截断 |
| 时间 | 2026-11-12 14:47–15:10 UTC |
| 单次墙钟合计 | 5464 s。墙钟不参与轨迹，不进门 |

## 已冻卡

结果不写入 `scenes/suite.txt`，不设 `criteria.frozen`，不设 gates，也不改已有冻结卡的判据。不把 0.20 m 或 0.35 m 改成默认。

## 怎么重跑

```bash
make venv
git rev-parse HEAD
.venv/bin/python -m pytest -q tests/test_e_c5_corridor_radius.py
.venv/bin/python scripts/e-c5-corridor-radius.py --check
.venv/bin/python -u scripts/e-c5-corridor-radius.py --jobs 2 --out docs/research/e-c5-corridor-radius-runs.json
```

`--check` 只核对几何 sha256、场景文件没被改写、0.05 m 臂不覆盖 `RunConfig`。部分结果在 `out/e-c5-corridor-radius/partial/`，默认可续跑。0.05 m 全关臂在同 seed 下，到达、碰撞、最小距离和仿真时长不该变。没有后段墙钟预算。
