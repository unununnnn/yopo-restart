# 已知限制（G7）

G7 的口径是「一键启动与可追溯结果」。这份文档是 [M5 交付包](SPEC.md) 里的已知限制：写清 L0 是哪一层、和上游有哪些已经登记的偏离、哪些数字只报数、L1 抽检卡在哪、合入前回归覆盖到哪一步，以及朝真实仿真还缺什么。

每次运行都要带层级（L0 / L1）、策略（`upstream` / `certified`）和影子盾开/关。不要把 L0 的点质量结果当成 PX4 或 Gazebo 的结果（[SPEC](SPEC.md) 用户故事 2，以及「L0 过于乐观」那条风险）。

下面出现的数字都指向结果文档或 ADR，不是编的。

## L0 是什么，不是什么

[SPEC](SPEC.md) 把仿真分成两层：

- **L0**：box 上 ROS-free、锁步、确定性的轻量仿真层。
- **L1**：现有 PX4 SITL + Gazebo 保真层。

L0 已经落地的构造是：

- **动力学**：点质量加一阶加速度滞后（`PointMassLag`）。场景卡和跟踪器的 `a_max` 都是 6。急刹不得再写死计划 `a_max=80`，见 [ADR 0011](adr/0011-brake-gap-horizon-and-amax.md)。这不是 PX4 位置环，也还没有 SPEC 里后加的 SO3 模型。
- **传感器**：按相机内外参做光线投射，输出深度图；量程、视场、分辨率、噪声、帧率可配，时间戳由仿真时钟给。规划器只拿 `Observation`（深度帧、里程计、目标、时间），拿不到场景几何真值。这不是渲染级相机：纹理和光照在 SPEC 的 Out of Scope 里。
- **多机总线**：每机一个独立规划实例和独立传感器。机间消息只走进程内消息总线，没有共享内存捷径；总线可以注入延迟、丢失、重复、乱序。
- **锁步时钟**：每 tick 依次是传感器 → 规划 → 进程内总线投递 → 动力学。规划消耗的墙钟不计入仿真时间。规划预算、帧龄、有效期一律按仿真时间判定。同 commit、同场景卡、同 seed，两次运行的结果记录逐字段一致（墙钟字段除外）。

L0 **不等价于** L1，也不是实飞：

- 它不经过 PX4 飞控，不经过 Gazebo 的传感器和物理，也没有起飞交接耗时和 RTF。结果记录里的 `tier` 是 `L0`。
- [票 10](tickets.md) 的 ROS 2 形态仍然是 L0 世界和 L0 锁步时钟，只是把每机规划放到 ROS 2 节点里，用 `/clock` 发仿真时间。不要把它读成 L1。
- 里程计是真值。没有 PX4 估计误差，也没有实机噪声以外、场景卡里没写的那一类传感缺陷。
- 实飞、改安全清单数值、改 PX4 参数，都在 SPEC 的 Out of Scope 里。

## 偏离上游

和 YOPO-MINCO 上游不一致的构造都记在 ADR 里。口头说「upstream 过了」时，要连这些偏离一起读。默认 `safe_radius` 0.05 m 与上游 d5e1e60 相同，**不是**偏离；设成别的值才标「偏离上游」，见 [ADR 0006 (h)](adr/0006-yopo-minco-baseline.md) 和 [07d](results/07d-safe-radius.md)。本仓库场景用的都是 0.05 m。

下面各条用对应 ADR 的决定原文（或原文要点），不另写一套说法。

| 项 | 决定（ADR 原文） | ADR |
|---|---|---|
| 终端规则 | 「L0 的 `upstream` 默认终端是项目规则 `stop_at_goal`，半径 1.5 m，停车段 2 s。不用上游原样的 `hold_here`。」上游半径是 5 m。在 G2 oracle（距目标 ≤ 0.4 m、速度 ≤ 0.2 m/s、连续保持 1 s）下，`hold_here` 到不了终点。 | [0009](adr/0009-stop-at-goal-vs-hold-here.md) |
| 承诺视界与急刹加速度 | 「构造 `CrossingFleet` 时强制 `commit_horizon_s ≥ N_agents × window_len_s + 0.25`。」「急刹和计划契约的 `a_max` 等于该卡的 L0 动力学 `a_max`。卡上没写时用 `PointMassLag.a_max`（当前 6）。禁止再写死计划 `a_max=80`。」六机视界因此抬到 1.75 s；三机卡上写的 1.0 s 已在下界上，构造时不抬高。 | [0011](adr/0011-brake-gap-horizon-and-amax.md) |
| Delay Check | 「编队把 δ_DC 设为 0.05 s」。收齐 ACK 并且到达 `broadcast_t + δ_DC` 之后才 COMMIT。「穿林和变阵显式保持 `delay_check_s = 0`。YOPO 的候选、解码和 `safe_radius` 都不改。」不换 YOPO，也不引入 Gurobi。上游协议没有这段等待。 | [0012](adr/0012-rmader-dual-track.md) |
| 进度门控 | 「穿林（`CrossingFleet`，场景卡带 `forest`）默认 `progress_gate=true`。」「门控开时，虚拟中心不再用墙钟 × `speed_mps`。」「开阔区 `FormationFleet` 的 `progress_gate` 固定为 false。」票 13e 起，「六机 medium / sparse 穿林卡写 `progress_gate=false`」，门控代码和开关保留。三机穿林卡仍写 true，柱林仍默认关。 | [0013](adr/0013-forest-progress-gate.md)、[0015](adr/0015-ghost-brake-stops.md) |
| 弹性槽位与净空门 | 「只对带 `forest` 且不少于六机的 `CrossingFleet` 生效。」「林带内……槽位是软目标。」「COMMIT 前对已拟合轨迹按 0.02 s 采样。中心到树干或矮枝表面的最小距离必须 ≥ `body_radius + clearance_margin_m`。margin 默认 0.15 m……不够就记 `CLEARANCE_SKIP`，不把这条多项式交给 `_as_plan`，也就不会提案、不会 COMMIT。」「这是过滤，不是优化。」不引入 CPLEX。 | [0014](adr/0014-elastic-slot-clearance-gate.md) |
| 幽灵刹车停点 | 「六机林带（`_forest_six()`）的提案前自检改为 `_conflicts_with_live()`：每个邻机只看 `peer.plans[-2:]`，也就是它的 committed + previous，加上它的预留（`delay_check_s > 0` 时再加 heard）。这与邻机投票时用的集合一致。三机穿林和柱林仍走 `conflicts_with_neighbors()`，行为不变。」六机林带卡同时暂关进度门控，见上。 | [0015](adr/0015-ghost-brake-stops.md) |
| 无解时的刹车 | 「L0 `UpstreamPolicy` 在走廊为空时，本帧停在当前里程计（`hold_traj`，码 `CORRIDOR_EMPTY_HOVER`），下一帧重规划。上游 d5e1e60 的 `control_pub` 在 `self.brake` 时不采样这条新轨迹，重复上一条 `PositionCommand`……crossing 无解且契约允许时提交一段运动学刹车（`BRAKE_ONLY`），否则留在上一条已提交计划。」 | [0006 (i)](adr/0006-yopo-minco-baseline.md) |

另外两件容易误读、但本身不是「改了上游选候选规则」的决定：

- L0 默认 `upstream` + 影子盾开。影子盾只记录会否拒绝，不改批准动作。要看现行全部门，必须显式 `@certified`。见 [ADR 0007](adr/0007-l0-default-upstream-shadow.md)。
- L0 上 `certified` 的 `ENV:UNKNOWN` / `NO_ENV_SAFE_CANDIDATE` / 到达 0 写进表，不作为合入失败，也不把 UNKNOWN 改成 FREE。见 [ADR 0010](adr/0010-certified-env-unknown-reported.md)。

## 只报数、不挡合入

[ADR 0016](adr/0016-m0-m4-freeze-recommendation.md)（accepted，frank 2026-11-10 批准；M2 队形门 6.0 m）打开了已冻指标的批跑退出码。[ADR 0008](adr/0008-report-only-until-frozen.md) 仍然管还没冻的项：这些数字每次都写进表，失败不把 `yopo_l0.batch` / `scripts/regress.sh` 置成非 0。全表和出处见 [票 11 冻结建议表](results/11-freeze-recommendation.md)。建议表里的数字来自 main `8ee3ca1d11056b653c659acba311c5e19482b134`（票 13e）。

继续只报数的是：

| 项 | 已有数字 | 出处 |
|---|---|---|
| 所有墙钟 | 随机器负载变，同 seed 也可以差几秒到几倍；不参与确定性比较。六机林带单次已经超过 60 s。 | [冻结表](results/11-freeze-recommendation.md)、[ADR 0016](adr/0016-m0-m4-freeze-recommendation.md) |
| certified 对照 | 同一张 G2 柱子卡、10 seed、全时长：到达 0/10，碰撞 0，结果都是 `TIMEOUT`。规划码是 `ENV:UNKNOWN` / `NO_ENV_SAFE_CANDIDATE`，飞机留在起点。这是对照臂，不是里程碑通过线。同一 commit 上 `upstream` + `stop_at_goal` 是 10/10、碰撞 0。 | [ADR 0010](adr/0010-certified-env-unknown-reported.md)、[M0 样例](results/m0-g2-pillar.md) |
| 三机林 sparse / dense | 07d（13a / 13b 之前，默认 `safe_radius` 0.05 m）：sparse 10/10、碰撞 0；dense 1/10、撞树 9。两档都没有在冻结用的 main `8ee3ca1` 上重跑。密档本来就不是通过线，不要冻成门。`crossing-forest` 只对 medium 开门。 | [07d](results/07d-safe-radius.md)、[冻结表](results/11-freeze-recommendation.md)、[ADR 0016](adr/0016-m0-m4-freeze-recommendation.md) |
| 三机林 medium 的数值净空 | 当前 main 重跑最紧 0.260 m，不够支撑柱林那道 0.40 m 的门。碰撞 = 0 和到达 10/10 已冻结。 | [冻结表](results/11-freeze-recommendation.md)（`out/t11-m2-forest`） |
| 六机林带的官方 p95 和相对 p95 | 官方 p95：medium 23.04–26.30 m，sparse 18.73–22.92 m。相对 p95（每拍以全队平均位置为中心，再取各 seed p95 的中位数）：medium 9.29 m，sparse 6.17 m。[ADR 0015](adr/0015-ghost-brake-stops.md) 写成 9.3 m / 6.2 m，是同一组数的四舍五入。林带卡 `progress_gate=false`，官方 p95 对着墙钟 × speed 的槽位，和 M1 的相对槽位不是一个口径。相对 p95 还高于 13b 写过的「争取低于 5 m」。 | [冻结表](results/11-freeze-recommendation.md)、[13e](results/13e-ghost-stops.md)、[ADR 0015](adr/0015-ghost-brake-stops.md) |
| 六机林带的数值净空 | 最紧 0.126 m（medium）/ 0.158 m（sparse）。COMMIT 前计划净空门是 0.50 m，跟踪差会吃掉一截。已冻的是碰撞 = 0，以及最小机间距离 ≥ 0.70 m。 | [冻结表](results/11-freeze-recommendation.md)、[13e](results/13e-ghost-stops.md)、[ADR 0015](adr/0015-ghost-brake-stops.md) |
| M4 的 ROS 2 全档矩阵 | 只对过一次：drop 20%、seed 0，与进程内一致（6/6，到达时间 20.13 s，机间 2.980 m，p95 0.066 m，碰撞 0）。全档全 seed 的 ROS 2 矩阵没有在当前 main 重跑。进程内六档已冻结；ROS 2 全矩阵另开票。 | [冻结表](results/11-freeze-recommendation.md)、[ADR 0016](adr/0016-m0-m4-freeze-recommendation.md) |
| M3 的 L1 抽检 | 没有新方案的 L0 / L1 对照数字。票 09 已关闭，见下一节。M3 先按 L0 变阵冻。 | [冻结表](results/11-freeze-recommendation.md)、[ADR 0016](adr/0016-m0-m4-freeze-recommendation.md)、[tickets](tickets.md) |
| 影子拒绝率 | L0 上开盾时多为 1.000。那是「若开硬门会拒」的记录，不改动作，不能当成失败门。 | [冻结表](results/11-freeze-recommendation.md)、[ADR 0007](adr/0007-l0-default-upstream-shadow.md) |
| 六机柱林 | 未测。单次墙钟超过 60 s，不进套件，没有当前 main 的表。 | [冻结表](results/11-freeze-recommendation.md)、[ADR 0016](adr/0016-m0-m4-freeze-recommendation.md) |

已冻的到达、碰撞和距离门见冻结表和场景卡 `criteria.gates`。那些门不在这里放宽，也不在这里重写。

## 票 09 已关闭，以及它留下的缺口

[票 09](tickets.md) 是 SPEC 里 M3 后半的 L1 抽检通道：同一张 M0 场景卡在现有 PX4 SITL 栈上跑一次，产出 L0 / L1 对照表。frank 2026-11-09 关闭了这张票。

关闭原因写在票列表里：09b 跑的是旧仓 yopo-swarm 已部署的 `deployed-certified` 栈，不验证本仓库的新方案（L0 默认 `upstream` + 影子盾）。L1 要等新方案接到 PX4 SITL 之后另开票。09a / 09b 的通道代码保留。

已知缺口（同一段票注）：

- 这条路径仍是已部署的 certified 栈，不接受 `--policy upstream`。对照表写 `policy=deployed-certified`（导出时是 `policy_requested=upstream`、`policy_executed=deployed-certified`）。
- 把 upstream 移植进 L1 要等以后的 yopo-swarm 票。本仓库不改那个仓库。
- 不要在这条已关闭的工单上重跑 Orin，也不要改 PX4 参数或安全清单数值。

因此 M3 按 SPEC 仍缺一次新方案的 L1 抽检。当前没有可引用的 L0 / L1 对照数字，这一项继续只报数。

## 合入前回归里的六机林带

票 13f 把 `crossing-6-forest.json@crossing@0` 和 `crossing-6-forest-sparse.json@crossing@0` 写进了 `scenes/suite.txt`，合入前回归不再漏跑这两张卡。见 [tickets.md](tickets.md) 的 13f 条。

限制是：

- **只跑 seed 0。** 单次墙钟约 216 s（medium）和 156 s（sparse），超过「单次超过 60 s 不进套件」的旧线，所以按 07b 惯例只收 seed 0。10 seed 全量仍用 README 里的批跑命令，不在每次合入前跑。
- 有更细的墙钟出处：[PR #31](https://github.com/unununnnn/yopo-restart/pull/31) 正文写 medium seed 0 墙钟 216.2 s、sparse seed 0 墙钟 156.4 s，到达都是 6/6，障碍和机间碰撞都是 0；同一次本机回归的批跑套件墙钟是 613 s（已含这两张卡的 seed 0）。再加上 L0 / core 的 pytest，整次 `scripts/premerge.sh` / `scripts/regress.sh` 要十几到二十分钟。
- 10 seed 的到达、碰撞门已经在 13e 上冻过：medium 60/60、sparse 60/60，碰撞 0，见 [13e](results/13e-ghost-stops.md) 和 [ADR 0016](adr/0016-m0-m4-freeze-recommendation.md)。套件里的 seed 0 是回归冒烟，不是把 10 seed 换成 1 seed。

## 朝真实仿真的下一步

这些是限制带来的后续工作，不是这张文档要改的代码。

1. **另开票，把本仓库的 `upstream` + 影子盾接到 PX4 SITL。** 不要重开已关闭的票 09，也不要在 Orin 上重跑旧仓 `deployed-certified`。接到之后，用同一张 M0 场景卡出 L0 / L1 对照（到达、碰撞、拒绝率、起飞交接耗时、RTF），再决定要不要启动 SPEC 里的方向 B（PX4 锁步多实例）。L1 的安全清单数值和 PX4 参数仍然不动。本仓库仍然不改 yopo-swarm。
2. **另开票跑 M4 的 ROS 2 全档全 seed 矩阵。** 现在只有 drop 20%、seed 0 的一次对账。进程内六档已经冻结，不能拿那一次对账代替全矩阵。
3. **L0 动力学若要更接近 PX4 的跟踪滞后，再加 SPEC 里后写的 SO3（或带一阶延迟的加速度跟踪之外的模型）。** 当前默认仍是点质量、`a_max=6`。换模型之后，L0 的到达和净空数字要重测，不能沿用现有冻结表里的秒数和米数。
4. **重新打开六机林带进度门控之前，先解决按 min 爬行太慢的问题。** [ADR 0015](adr/0015-ghost-brake-stops.md) 写过：只修自检、门控仍开时，medium 4/60、sparse 33/60，门控能推进到 27–42 m，但 50 s 内飞不完。可选方向（超前机改走短前缀，或放大 `_LEADER_MARGIN_M`）都还没有测。
5. **实飞仍不在范围内。** 在 L1 对照跑通之前，不要把 L0 视频或到达率读成「飞机已经在 SITL 里飞起来了」。
