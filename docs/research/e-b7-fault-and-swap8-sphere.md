# E-B7：八机对向与 M4 六档通讯故障下的欧氏碰撞球

只报数。report-only。没有改 `yopo_l0/`、`core/`、`scenes/`、`docs/tickets.md`、README、`scripts/premerge.sh`、`scripts/regress.sh`。开球只写在进程内的场景卡上，三档开球臂都标「report-only / 偏离上游」。未冻结，不进 `scenes/suite.txt`，不设 gates，不改已冻门。

数字来自 [e-b7-fault-and-swap8-sphere.py](../../scripts/e-b7-fault-and-swap8-sphere.py)，跑数时 HEAD `b6fff8266dcc56ebf67039c0a5c0b195d24ec121`。逐粒记录在 [e-b7-fault-and-swap8-sphere-runs.json](e-b7-fault-and-swap8-sphere-runs.json)。邻机采样进后段的包装与 [E-B4c](e-b4c-sphere-avoid.md) 的 `install_peer_delivery` 相同。

## 结论

第一组是 `scenes/formation-6-open.json`，策略 `formation`，故障档来自 `scenes/formation-6-tiers.json`，seeds 0–4。关球、E-B4c（r=0.85, w=8）、E-B5（r=1.00, w=8）三臂，在同一档、同一种子上，到达、碰撞、队形误差 p95、最小机间、到达时间、仿真时长、后段调用都相同。后段调用全是 0。六档的到达都是 30/30，碰撞 0，队形误差 p95 0.066 m，最小机间 2.980 m，到达时间 20.13 s。

丢包、复制、乱序按档位发生了，而且三臂的总线计数逐种子相同。drop 5% 丢 53–67 条，drop 20% 丢 244–308 条，duplicate 10% 复制 111–137 条，reorder+delay 乱序 941–1122 条，combined 三样都有。开球没有因为这些延迟或丢包去改队形，也没有误排斥。

编队 `FormationFleet` 不调用 `refine_yopo`，运行器也不为它建 `UpstreamPolicy`。包装只改已有的 `UpstreamPolicy._apply_backend` 和 `CrossingFleet._apply_backend`，不给编队新加后段。内存卡上的 `peer_sphere_enabled` 编队规划器不读。故障改的是预留协议的投递，欧氏球在这条回路里没有邻机采样可用。

这组关球数字与 [M4 总表](../results/SUMMARY.md) 的 30/30、碰撞 0、队形误差 p95 0.066 m、最小机间 2.980 m 一致。本票不把这些数再送进 gates。

第二组是 `scenes/swap-8-open.json`，seeds 0–4，零故障，goto / upstream。关球 5/5 机间碰撞，到达 0/40，最小机间 0.695–0.696 m，五粒都是 `uav3-uav4:INTER_DRONE`，仿真 2.09 s。平均到达时间没有样本。

E-B4c（r=0.85, w=8）report-only / 偏离上游：到达 8/40。只有 seed 4 到了，8/8，最小机间 0.709 m，到达时间 6.96 s，仿真 7.96 s。另外四粒仍是 `uav4-uav5:INTER_DRONE`，最小机间 0.694–0.699 m。平均到达时间 6.96 s，只含这 1 粒。

E-B5（r=1.00, w=8）report-only / 偏离上游：到达 0/40，5 次机间碰撞，最小机间 0.688–0.698 m。seed 0–3 仍是 `uav3-uav4`，seed 4 换成 `uav3-uav6`，0.688 m。半径放到 1.00 m 之后，seed 4 没有保住。

E-B5（r=0.85, w=16）report-only / 偏离上游：到达 0/40，5 次机间碰撞，最小机间 0.687–0.696 m。权重加到 16 也没有解开任何种子。最紧的是 seed 0，0.687 m。

四臂的后段预算截断都是 0。八机开球用一个进程。这组差异不是 0.02 s 墙钟预算把迭代截短。

[E-B4c](e-b4c-sphere-avoid.md) 在三机 `swap-3-open` 上把 5 个种子都解开了。八机沿直径同时过圆心时，同一套默认球只解开 1/5 个种子。半径 1.00 m 和权重 16 都没有比（0.85 m, 权重 8）解开更多种子。

## 接线

进程内包装与 `scripts/e-b4c-sphere-avoid.py` 的 `install_peer_delivery` 相同。欧氏球打开时，`runner.swarm_penalty_enabled` 也视为要广播轨迹；`UpstreamPolicy` 留下本 tick poll 到的邻机采样，下一次 `plan` 送进 `refine_yopo`；`CrossingFleet._apply_backend` 在欧氏球打开时送已提交采样。两个开关都关时，送进后段的邻机仍是 None。第 0 个规划拍策略还看不到邻机。`swarm_penalty` 和 `swarm_segment_penalty` 保持关闭，末端控制点仍钉死。

不改 `policy.py`、`crossing.py`、`formation.py` 的文件。`formation` 任务走预留协议，不进 YOPO。

| 臂 | 分组 | 进程内 `policy_params.backend` |
|---|---|---|
| off（关球） | 两组 | 不写。沿用场景卡 |
| E-B4c(r=0.85, w=8) report-only / 偏离上游 | 两组 | `enabled=true`，`peer_sphere_enabled=true`，半径 0.85 m，权重 8 |
| E-B5(r=1.00, w=8) report-only / 偏离上游 | 两组 | 同上，半径 1.00 m，权重 8 |
| E-B5(r=0.85, w=16) report-only / 偏离上游 | 只在八机 | 同上，半径 0.85 m，权重 16 |

0.85 m = 0.70 m 碰撞球 + 0.15 m 裕度，沿用 E-B4c。1.00 m 和权重 16 是本票要对照的另外两档，同样只在进程里。铰链形状见 [E-B4c](e-b4c-sphere-avoid.md)。

## 对照怎么跑

```bash
.venv/bin/python scripts/e-b7-fault-and-swap8-sphere.py
```

编队和关球最多两个进程。八机开球一个进程，避免两份 L-BFGS 抢 0.02 s 墙钟预算。每粒落在 `out/e-b7-fault-and-swap8-sphere/partial/`，可续跑。影子盾开，仿真时长用场景卡。

编队种子 0–4，与场景卡 `seeds` 以及 M4 总表一致。六档 id 是 `none`、`drop-5`、`drop-20`、`duplicate-10`、`reorder-delay`、`combined`。故障经 `RunConfig.bus` 注入，不写回场景卡。八机种子 0–4；`scenes/swap-8-open.json` 本身只声明 seed 0，多出来的种子走 `RunConfig`。

关球的场景卡哈希与文件一致。编队关球 `a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d`。八机关球 `489946bf0db1fde539414dfd51abd5381e795984a191016cde93c5440f7a3853`。开球多了 `policy_params.backend`，哈希不同，只存在于这次进程的内存里。

编队开球哈希：E-B4c `66eac555a320ad126d94610a78ff5a6dec8f3432f80e482ceac4438aa707bac6`，E-B5 半径 1.00 m `87b3db71554b2eabdaa9f2e43b13019f1b20f0e85dd737c2ac0d80c1a1a8ba82`。八机开球哈希：E-B4c `58151f19f2e2b6d80c102748311efa058687800c9e831852f3f35de33d5aa1c1`，半径 1.00 m `b5f7f43c0db12b256f7df2cd501b0ad024e6be26841ac3a7fc2ad02b5c22c94d`，权重 16 `f1b00a93058f862a58110f6ca6aeb1282166e0ec519e9af874581266fd2c1625`。

## 第一组：六机编队 × 六档故障

到达按架次计，分母是 5 个种子 × 6 架。碰撞是障碍碰撞与机间碰撞之和，这组都是 0。队形误差 p95 和最小机间在五个种子上相同，所以表上是一个数。丢包是各种子 `bus.dropped` 的最小到最大。后段调用是包装记到的 `refine_yopo` 次数。

| 档位 | 臂 | 到达 | 碰撞 | 队形误差 p95 m | 最小机间 m | 后段调用 | 丢包 |
|---|---|---|---|---|---|---|---|
| none | off（关球） | 30/30 | 0 | 0.066 | 2.980 | 0 | 0 |
| none | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 30/30 | 0 | 0.066 | 2.980 | 0 | 0 |
| none | E-B5(r=1.00, w=8) report-only / 偏离上游 | 30/30 | 0 | 0.066 | 2.980 | 0 | 0 |
| drop-5 | off（关球） | 30/30 | 0 | 0.066 | 2.980 | 0 | 53–67 |
| drop-5 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 30/30 | 0 | 0.066 | 2.980 | 0 | 53–67 |
| drop-5 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 30/30 | 0 | 0.066 | 2.980 | 0 | 53–67 |
| drop-20 | off（关球） | 30/30 | 0 | 0.066 | 2.980 | 0 | 244–308 |
| drop-20 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 30/30 | 0 | 0.066 | 2.980 | 0 | 244–308 |
| drop-20 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 30/30 | 0 | 0.066 | 2.980 | 0 | 244–308 |
| duplicate-10 | off（关球） | 30/30 | 0 | 0.066 | 2.980 | 0 | 0 |
| duplicate-10 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 30/30 | 0 | 0.066 | 2.980 | 0 | 0 |
| duplicate-10 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 30/30 | 0 | 0.066 | 2.980 | 0 | 0 |
| reorder-delay | off（关球） | 30/30 | 0 | 0.066 | 2.980 | 0 | 0 |
| reorder-delay | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 30/30 | 0 | 0.066 | 2.980 | 0 | 0 |
| reorder-delay | E-B5(r=1.00, w=8) report-only / 偏离上游 | 30/30 | 0 | 0.066 | 2.980 | 0 | 0 |
| combined | off（关球） | 30/30 | 0 | 0.066 | 2.980 | 0 | 68–81 |
| combined | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 30/30 | 0 | 0.066 | 2.980 | 0 | 68–81 |
| combined | E-B5(r=1.00, w=8) report-only / 偏离上游 | 30/30 | 0 | 0.066 | 2.980 | 0 | 68–81 |


duplicate 10% 的丢包是 0，复制条数是 111–137。reorder+delay 的丢包是 0，乱序条数是 941–1122。combined 同时有丢包 68–81、复制 131–156、乱序 975–1094。三臂在每个种子上的这三项计数相同。

### 分种子

到达时间单位 s。乱序是 `bus.reordered`。墙钟不进这张表。

| 档位 | 臂 | seed | 结果 | 到达 | 队形误差 p95 m | 最小机间 m | 到达时间 s | 后段调用 | 丢包 | 乱序 |
|---|---|---|---|---|---|---|---|---|---|---|
| none | off（关球） | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | off（关球） | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | off（关球） | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | off（关球） | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | off（关球） | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | E-B5(r=1.00, w=8) report-only / 偏离上游 | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | E-B5(r=1.00, w=8) report-only / 偏离上游 | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | E-B5(r=1.00, w=8) report-only / 偏离上游 | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | E-B5(r=1.00, w=8) report-only / 偏离上游 | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| none | E-B5(r=1.00, w=8) report-only / 偏离上游 | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| drop-5 | off（关球） | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 54 | 0 |
| drop-5 | off（关球） | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 67 | 0 |
| drop-5 | off（关球） | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 58 | 0 |
| drop-5 | off（关球） | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 53 | 0 |
| drop-5 | off（关球） | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 66 | 0 |
| drop-5 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 54 | 0 |
| drop-5 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 67 | 0 |
| drop-5 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 58 | 0 |
| drop-5 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 53 | 0 |
| drop-5 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 66 | 0 |
| drop-5 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 54 | 0 |
| drop-5 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 67 | 0 |
| drop-5 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 58 | 0 |
| drop-5 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 53 | 0 |
| drop-5 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 66 | 0 |
| drop-20 | off（关球） | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 268 | 0 |
| drop-20 | off（关球） | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 308 | 0 |
| drop-20 | off（关球） | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 244 | 0 |
| drop-20 | off（关球） | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 249 | 0 |
| drop-20 | off（关球） | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 283 | 0 |
| drop-20 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 268 | 0 |
| drop-20 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 308 | 0 |
| drop-20 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 244 | 0 |
| drop-20 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 249 | 0 |
| drop-20 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 283 | 0 |
| drop-20 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 268 | 0 |
| drop-20 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 308 | 0 |
| drop-20 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 244 | 0 |
| drop-20 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 249 | 0 |
| drop-20 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 283 | 0 |
| duplicate-10 | off（关球） | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | off（关球） | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | off（关球） | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | off（关球） | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | off（关球） | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| duplicate-10 | E-B5(r=1.00, w=8) report-only / 偏离上游 | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 0 |
| reorder-delay | off（关球） | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 1122 |
| reorder-delay | off（关球） | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 1065 |
| reorder-delay | off（关球） | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 941 |
| reorder-delay | off（关球） | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 1095 |
| reorder-delay | off（关球） | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 966 |
| reorder-delay | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 1122 |
| reorder-delay | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 1065 |
| reorder-delay | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 941 |
| reorder-delay | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 1095 |
| reorder-delay | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 966 |
| reorder-delay | E-B5(r=1.00, w=8) report-only / 偏离上游 | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 1122 |
| reorder-delay | E-B5(r=1.00, w=8) report-only / 偏离上游 | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 1065 |
| reorder-delay | E-B5(r=1.00, w=8) report-only / 偏离上游 | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 941 |
| reorder-delay | E-B5(r=1.00, w=8) report-only / 偏离上游 | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 1095 |
| reorder-delay | E-B5(r=1.00, w=8) report-only / 偏离上游 | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 0 | 966 |
| combined | off（关球） | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 76 | 979 |
| combined | off（关球） | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 75 | 1028 |
| combined | off（关球） | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 81 | 975 |
| combined | off（关球） | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 70 | 1044 |
| combined | off（关球） | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 68 | 1094 |
| combined | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 76 | 979 |
| combined | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 75 | 1028 |
| combined | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 81 | 975 |
| combined | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 70 | 1044 |
| combined | E-B4c(r=0.85, w=8) report-only / 偏离上游 | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 68 | 1094 |
| combined | E-B5(r=1.00, w=8) report-only / 偏离上游 | 0 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 76 | 979 |
| combined | E-B5(r=1.00, w=8) report-only / 偏离上游 | 1 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 75 | 1028 |
| combined | E-B5(r=1.00, w=8) report-only / 偏离上游 | 2 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 81 | 975 |
| combined | E-B5(r=1.00, w=8) report-only / 偏离上游 | 3 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 70 | 1044 |
| combined | E-B5(r=1.00, w=8) report-only / 偏离上游 | 4 | SUCCESS | 6/6 | 0.066 | 2.980 | 20.13 | 0 | 68 | 1094 |


## 第二组：八机开阔对向

到达按架次计，分母是 5 个种子 × 8 架。机间碰撞一粒种子最多记 1，仿真在第一次中心距 ≤ 0.70 m 时停下。平均到达时间只对 `arrival_time_s` 非空的种子取平均，也就是八架都进到达判据的那些种子。最小机间：没撞是全程最小，撞了是停下那一拍的采样。

| 臂 | 到达 | 机间碰撞 | 超时 | 最小机间 m | 平均到达时间 s | 后段调用 | 预算截断 |
|---|---|---|---|---|---|---|---|
| off（关球） | 0/40 | 5 | 0 | 0.695–0.696 | —（0/5 粒） | 0 | 0/0 |
| E-B4c(r=0.85, w=8) report-only / 偏离上游 | 8/40 | 4 | 0 | 0.694–0.709 | 6.96（1/5 粒） | 1032 | 0/1032 |
| E-B5(r=1.00, w=8) report-only / 偏离上游 | 0/40 | 5 | 0 | 0.688–0.698 | —（0/5 粒） | 888 | 0/888 |
| E-B5(r=0.85, w=16) report-only / 偏离上游 | 0/40 | 5 | 0 | 0.687–0.696 | —（0/5 粒） | 888 | 0/888 |


### 分种子

后段时间是墙钟，单位 ms。整回路墙钟只作这次机器上的耗时，不进门。

| 臂 | seed | 结果 | 原因 | 到达 | 机间 | 最小机间 m | 到达时间 s | 后段 p50 / p95 ms | 截断 | 墙钟 s |
|---|---|---|---|---|---|---|---|---|---|---|
| off（关球） | 0 | COLLISION | uav3-uav4:INTER_DRONE | 0/8 | 1 | 0.695 | — | — | 0/0 | 29.4 |
| off（关球） | 1 | COLLISION | uav3-uav4:INTER_DRONE | 0/8 | 1 | 0.695 | — | — | 0/0 | 29.2 |
| off（关球） | 2 | COLLISION | uav3-uav4:INTER_DRONE | 0/8 | 1 | 0.695 | — | — | 0/0 | 28.1 |
| off（关球） | 3 | COLLISION | uav3-uav4:INTER_DRONE | 0/8 | 1 | 0.696 | — | — | 0/0 | 28.2 |
| off（关球） | 4 | COLLISION | uav3-uav4:INTER_DRONE | 0/8 | 1 | 0.695 | — | — | 0/0 | 28.2 |
| E-B4c(r=0.85, w=8) report-only / 偏离上游 | 0 | COLLISION | uav4-uav5:INTER_DRONE | 0/8 | 1 | 0.697 | — | 7.8 / 9.9 | 0/176 | 64.9 |
| E-B4c(r=0.85, w=8) report-only / 偏离上游 | 1 | COLLISION | uav4-uav5:INTER_DRONE | 0/8 | 1 | 0.697 | — | 7.7 / 9.5 | 0/176 | 63.1 |
| E-B4c(r=0.85, w=8) report-only / 偏离上游 | 2 | COLLISION | uav4-uav5:INTER_DRONE | 0/8 | 1 | 0.699 | — | 7.8 / 9.5 | 0/184 | 66.0 |
| E-B4c(r=0.85, w=8) report-only / 偏离上游 | 3 | COLLISION | uav4-uav5:INTER_DRONE | 0/8 | 1 | 0.694 | — | 7.7 / 9.5 | 0/176 | 63.4 |
| E-B4c(r=0.85, w=8) report-only / 偏离上游 | 4 | SUCCESS | GOAL_REACHED_AND_HELD | 8/8 | 0 | 0.709 | 6.96 | 6.2 / 9.0 | 0/320 | 199.8 |
| E-B5(r=1.00, w=8) report-only / 偏离上游 | 0 | COLLISION | uav3-uav4:INTER_DRONE | 0/8 | 1 | 0.691 | — | 7.7 / 9.1 | 0/176 | 63.1 |
| E-B5(r=1.00, w=8) report-only / 偏离上游 | 1 | COLLISION | uav3-uav4:INTER_DRONE | 0/8 | 1 | 0.694 | — | 7.7 / 9.5 | 0/176 | 63.3 |
| E-B5(r=1.00, w=8) report-only / 偏离上游 | 2 | COLLISION | uav3-uav4:INTER_DRONE | 0/8 | 1 | 0.696 | — | 7.7 / 9.2 | 0/176 | 62.8 |
| E-B5(r=1.00, w=8) report-only / 偏离上游 | 3 | COLLISION | uav3-uav4:INTER_DRONE | 0/8 | 1 | 0.698 | — | 7.7 / 9.4 | 0/176 | 63.5 |
| E-B5(r=1.00, w=8) report-only / 偏离上游 | 4 | COLLISION | uav3-uav6:INTER_DRONE | 0/8 | 1 | 0.688 | — | 7.7 / 9.5 | 0/184 | 66.3 |
| E-B5(r=0.85, w=16) report-only / 偏离上游 | 0 | COLLISION | uav3-uav4:INTER_DRONE | 0/8 | 1 | 0.687 | — | 7.4 / 9.3 | 0/176 | 63.1 |
| E-B5(r=0.85, w=16) report-only / 偏离上游 | 1 | COLLISION | uav4-uav5:INTER_DRONE | 0/8 | 1 | 0.695 | — | 7.4 / 9.1 | 0/176 | 62.8 |
| E-B5(r=0.85, w=16) report-only / 偏离上游 | 2 | COLLISION | uav3-uav4:INTER_DRONE | 0/8 | 1 | 0.695 | — | 7.5 / 9.0 | 0/176 | 62.9 |
| E-B5(r=0.85, w=16) report-only / 偏离上游 | 3 | COLLISION | uav4-uav5:INTER_DRONE | 0/8 | 1 | 0.688 | — | 7.4 / 9.1 | 0/184 | 66.3 |
| E-B5(r=0.85, w=16) report-only / 偏离上游 | 4 | COLLISION | uav3-uav4:INTER_DRONE | 0/8 | 1 | 0.696 | — | 7.3 / 8.9 | 0/176 | 62.6 |


## 单测

`tests/test_e_b7_fault_and_swap8_sphere.py` 检查任务网格、开球标签里的 report-only / 偏离上游、内存卡不写 `scenes/`、包装只在欧氏球打开时把采样送进 `refine_yopo`，以及本页四张表与 json 逐字一致。不重跑这 110 粒。

```bash
.venv/bin/python -m pytest -q tests/test_e_b7_fault_and_swap8_sphere.py
```

## L0-DET 确定性重跑（2026-11-15）

当前 main `046d25a`。后段缺省 `budget="evals"`（0.020 s → 10 次评估）。两组、臂、种子、六档与上面的命令相同；`--jobs-plain 2 --jobs-backend 2`。连跑两遍，到达、碰撞、距离、后段调用和预算截断逐位相同。第二遍的 `commit` 带 `-dirty`，是写文档时的工作区标记，不是轨迹差。

第一组编队六档与上文逐档相同：到达 30/30，碰撞 0，队形误差 p95 0.066 m，最小机间 2.980 m，到达时间 20.13 s，后段调用 0。丢包、复制、乱序的范围也没变（drop 5% 仍是 53–67，drop 20% 仍是 244–308，combined 仍是 68–81）。编队不调用 `refine_yopo`，评估次数预算碰不到这组。

| 臂 | 旧到达 | 新到达 | 旧机间碰撞 | 新机间碰撞 | 旧最小机间 m | 新最小机间 m | 新预算截断 |
|---|---|---|---|---|---|---|---|
| 关球 | 0/40 | 0/40 | 5 | 5 | 0.695–0.696 | 0.695–0.696 | 0/0 |
| E-B4c（0.85 m，权重 8） | 8/40 | 0/40 | 4 | 5 | 0.694–0.709 | 0.682–0.699 | 654/896 |
| E-B5（1.00 m，权重 8） | 0/40 | 0/40 | 5 | 5 | 0.688–0.698 | 0.641–0.685 | 685/920 |
| E-B5（0.85 m，权重 16） | 0/40 | 0/40 | 5 | 5 | 0.687–0.696 | 0.683–0.699 | 612/872 |

结论变了。上文写默认球只解开 seed 4（8/8，最小机间 0.709 m），另外四粒仍相撞。这次五粒都是机间碰撞，seed 4 的最小机间是 0.682 m，没有解开的种子。半径 1.00 m 和权重 16 仍是 0/40，没有比默认球解开更多，这一点还在，但比较的基准从「解开 1/5」变成「解开 0/5」。编队「开球不改队形、后段调用为 0」没变。
