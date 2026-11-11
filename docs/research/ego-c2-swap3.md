# 票 E-C2：三机开阔对向（只报数）

这张卡看现有 L0 点质量在「目标对调、沿直径穿过圆心」时会不会机间碰撞。规划器没有改。场景卡不进 `scenes/suite.txt`，`criteria.frozen` 为 false，没有 gates。墙钟入表，不进门。

## 场景

三架在开阔区，起点均匀分布在半径 8.0 m、高度 1.5 m 的圆上（方位 0° / 120° / 240°），各自飞向直径对侧。直径路程 16.0 m，起点间距 \(8\sqrt{3}\) ≈ 13.856 m。几何是 `open-area.json`，只有地面。

本卡不复现 EGO-Swarm 论文表 I 的 20.3 m / 14.3 s。论文的圆半径和速度上限未知。8.0 m 是本票选定的半径，不是从那两个数字反推的。

种子只跑 0。判据未冻结，到达、碰撞、最小机间距离只报数。

## 设定

速度和 `a_max` 用本仓库已有的点质量设定，没有为这张卡另写上限。

| 项 | 值 |
|---|---|
| 动力学 | `point_mass_lag`，`tau_s=0.1`，`kp=4.0`，`kd=3.0`，`a_max=6.0` |
| 场景卡速度 | goto 任务没有 `speed_mps`（与 `open-3drone.json`、`g2-pillar.json` 相同） |
| 上游解码器 | `UPSTREAM_DECODER.vel_max=6.0`、`acc_max=6.0`（上游 `d5e1e60`） |
| 机体半径 | 0.35 m。机间碰撞线是中心距 ≤ 0.70 m |
| 到达口径 | ≤ 0.4 m 且 ≤ 0.2 m/s，保持 1 s |
| 影子盾 | 开。只记录会否拒绝，不改批准动作 |
| 形态 | 进程内 |

L0 默认策略是 `upstream` 加影子盾（[ADR 0007](../adr/0007-l0-default-upstream-shadow.md)）。另外跑了 `@hover`：`HoverPlan` 把对向目标当悬停点，点质量 PD 沿直径直飞，没有 `vel_max`。这是直行对照，不是默认策略。

## 出处

| 项 | 值 |
|---|---|
| 跑数 commit | `c7a75efb15e3317e511dae21e2264ca6059d44e9`（分支 `ec2-swap-3`，工作区干净。合入 main 后的 squash sha 会不同） |
| 层级 | L0 |
| 场景卡 | `scenes/swap-3-open.json` |
| 场景卡哈希 | `28c41923662d93a33a5d707bc717fc8070c9c6f8020d66be60de842ced659d6e` |
| seed | 0 |
| 网络（upstream） | 上游 `d5e1e60f83cc991602f142d9b6b96204a24d6b9f`，权重 sha256 `2d85e712c8874d11cd4e523777698eff78914fdb8f1ffb5068b037f8f972f62f`，`torch-cpu-fp32` |

场景卡哈希是场景卡加上内联几何之后的规范 JSON 的 sha256，与结果记录里的 `scene_card_hash` 相同。墙钟是这一次运行的测量，不参与确定性比较。同 seed 重跑时，除墙钟外的结果记录和 tick 日志一致。

## 结果

两条策略在 seed 0 都是机间碰撞，到达 0/3。没有死锁：碰撞时刻飞机仍在飞，速度分别约为 5.7–5.9 m/s（upstream）和 9.5 m/s（hover）。没有改规划器。

| 策略 | 结果 | 到达 | 碰撞 | 原因 | 最小机间距离 m | 仿真 s | 墙钟 s | 测得最大速度 m/s | 终点误差 m | 规划码 |
|---|---|---|---|---|---|---|---|---|---|---|
| upstream（默认） | COLLISION | 0/3 | 1 | `uav1-uav2:INTER_DRONE` | 0.631 | 2.16 | 12.1 | 5.937 | 12.227 | OK:22 |
| hover（直行对照） | COLLISION | 0/3 | 1 | `uav0-uav1:INTER_DRONE` | 0.670 | 1.68 | 0.8 | 9.509 | 8.387 | HOVER:17 |

最小机间距离取结果记录的 `min_inter_drone_distance_m`。测得最大速度是从该次 `ticks.jsonl` 里各机速度模长取到的最大值，不是场景卡字段。

hover 在仿真 1.68 s 时，三机仍保持等边靠近圆心，高度一直是 1.5 m。三对中心距同时落到 0.670 m，都低于 0.70 m；结果记录按遍历顺序记成 `uav0-uav1`。碰撞时三机速度模长都是 9.509 m/s，沿各自直径指向圆心。`HoverPlan` 没有速度上限，点质量只受 `a_max=6` 限制，所以速度可以超过上游解码器的 6 m/s。

upstream 在仿真 2.16 s 时，`uav1` 与 `uav2` 在圆心附近相撞，中心距 0.631 m，高度约 2.45–2.50 m。当时 `uav0` 的水平半径仍是 4.74 m，与这两架的距离是 4.111 m 和 4.727 m。三条轨迹没有同时走到圆心。测得最大速度 5.937 m/s，没有超过解码器的 `vel_max=6.0`。规划码全是 `OK`，没有无解悬停。goto 路径的 `Observation` 里没有他机状态，上游一次前向也不会把邻机写进代价。

影子拒绝率两次都是 1.000。盾不改动作，拒绝码不解释这次碰撞。

对照文档里的 E-A1（机间过滤开关，默认关）以「对向卡上出现机间碰撞」为开工条件。本票两条策略都出现了机间碰撞。本文件只记录这个事实，不改 `docs/tickets.md`，也不打开那个开关。

## 已冻卡

本卡不写入 `scenes/suite.txt`。已冻卡的批跑结果不因本票的场景卡或结果文档而改变。合入前由 `scripts/premerge.sh` 再跑回归确认。

## 怎么重跑

```bash
make venv
git rev-parse HEAD
.venv/bin/python -c 'from yopo_l0.scene import load_card; print(load_card("scenes/swap-3-open.json").hash)'
.venv/bin/python -m yopo_l0.batch \
  scenes/swap-3-open.json@upstream \
  scenes/swap-3-open.json@hover \
  --seeds 0 --out out/ego-c2-swap3
```

干净工作区上，结果记录的 `commit` 应是当时的 `HEAD`。墙钟会变。到达、碰撞、最小机间距离、仿真时长和规划码在同 seed 下不该变。
