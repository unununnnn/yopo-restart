# 票 E-B4：机间碰撞为什么在所有臂上都「只差一点」（诊断，只报数）

只报数。没有改 `yopo_l0/`、`core/`、已冻门、`scenes/suite.txt`、`docs/tickets.md` 和 README，也没有改任何开关的默认值，没有冻结新判据。本票只新增一个只读诊断脚本 [e-b4-peer-diag.py](../../scripts/e-b4-peer-diag.py)、它的测试和这篇报告。修复臂是脚本进程里的原型，不是已合入的开关（见「修复」一节）。

数字在 [e-b4-peer-collision-diag-runs.json](e-b4-peer-collision-diag-runs.json)。E-B3 臂用的是还没合入的 E-B3 worktree 代码（`--code-root`），脚本本身不改那份代码。

## 结论

1. **「只差一点」是截断造成的读数，不是擦边。** 运行器在第一次中心距 ≤ 0.70 m 的那个 tick 就停（`runner.py:111`、`runner.py:374`），所以记下的最小机间距离只是越线那一拍的采样，比 0.70 m 少「接近速度 × 0.01 s」。swap8 全关接近速度 2.98 m/s，一拍 0.030 m，所以每粒都是 0.695 m；d014 全关接近速度 8.08 m/s，一拍 0.081 m，所以散在 0.62–0.70 m。让仿真越线后再飞 1 s（脚本里换掉 `_note_pairs`，越线前轨迹与运行器逐 tick 相同），swap8 全关这一对最小到 0.511 m，8 架里任意一对最小 0.090 m，28 对里 12 对进了 0.70 m；d014 全关这一对最小 0.426 m。它们会直接穿过去。
2. **全关和 E-B2 两臂上，没有任何一层看得见邻机。** 深度相机只渲染场景静态障碍（`runner.py:207`、`sensor.py:162`），Observation 只有深度、本机状态、目标和时间（`policy.py:24-28`），影子盾调 `SafetyGate.validate` 时邻机传的是空元组（`shadow.py:241`），goto 没有协议节点（`runner.py:217`），E-A1、D1 默认关。邻机在视场里的规划拍有 33 次（d014 全关），深度图在邻机像素上一次也没有打到邻机。按同样的时序重建「如果广播会拿到的邻机计划」，两机自己的计划在 2 s 窗口里预测分离只有 0.03–0.51 m：计划本身就是对穿的。
3. **E-B3 臂里唯一看得见邻机的是后段，但它的椭球项够不着最近点。** 不是 0.02 s 预算：把预算放到 60 s、迭代放到 100，d014 最近中心距仍是 0.095 m（原样 0.099 m），swap8 是 0.039 m（原样 0.038 m）。离线复算 d014 的 35 次「邻机进了 C」的后段调用：21 次最近点落在两端钉死的控制点上（头尾各约 2·dt ≈ 0.44 s 不可动）；另外 14 次最近点虽在可动区间，但椭球项只在控制点的 Greville 时刻采样，控制点间隔 0.21–0.23 s，接近速度约 7 m/s 时两点之间相对位移约 1.5 m，曲线最近 0.09–0.64 m 时控制点上量到的最近只有 0.14–1.06 m，其中 4 次代价为 0。
4. **E-A1 打开时（阈值 0.70 m），两处让它失效：门槛没有余量（H1），无解悬停把参考点拽回当前位置（H3，进而 H2）。** `CORRIDOR_EMPTY_HOVER` 用 `hold_traj(s.p, …)`（`policy.py:228`）把参考瞬间改成当前位置、速度 0，飞机在 4 m/s 上刹不住，参考与真值差到 1.24 m（swap8）/ 1.90 m（d014）；下一拍又从这个参考头出发（`policy.py:213`，`plan_from_reference=True`），广播出去的计划一直落后真值约 1 m。E-A1 因此预测分离 0.72–0.81 m（过门槛），真值 0.39 m。把悬停换成按 a_max 从参考头刹停（原型）后，跟踪差回到 0.13 m，但 swap8 seed 0 仍以 0.693 m 擦线：这一下就是门槛 0.70 m 正好等于碰撞线。

根因：前端选择层看不见邻机（E-A1 默认关），后段的邻机项在结构上够不着最近点；E-A1 打开时，零余量门槛和「无解即原地悬停」的参考跳变又让它漏过。修复原型（E-A1 阈值 1.0 m + 无解时从参考头刹停）在两张卡 seeds 0–9 上的数字见「修复对照」。

## 假设逐个看

| 假设 | 结论 | 证据 |
|---|---|---|
| H1 门槛 = 碰撞距离，没有余量 | 在三个对照臂上不是主因（那几层根本没开）；在 E-A1 臂上成立 | 全关：相机、Observation、影子盾都不看邻机（见上）。E-A1 门槛 0.70 m（`scripts/e-c4-density-l0.py:67`、`scripts/e-combo-a1-d1-b2.py:40`，判 `clearances < threshold`，`inter_drone.py:89`）；D1 缺省 `2 × body_radius` = 0.70 m（`geom_fallback.py:223`）；两者都是同时刻中心距，余量 0。E-B3 C = 0.80 m，比碰撞线多 0.10 m。swap8 E-A1+刹停原型 seed 0：跟踪差最大 0.133 m，计划预测 0.74 m，越线时接近速度 0.60 m/s，越线后一对最小 0.693 m |
| H2 邻机计划过时 | 三个对照臂上不成立；E-A1 臂上作为 H3 的后果成立 | 零故障总线，邻机计划年龄恒为 0.1 s（一个规划周期：运行器先全体规划、后投递，`runner.py:257-308`）。全关 swap8 最后一拍预测 0.502 / 0.506 m，真值 0.511 m；E-B3 d014 预测 0.368 m，真值 0.477 m。预测比真值还近，过时不会把「本来分得开」变成相撞。E-A1 swap8：悬停后预测 0.797 / 0.813 m，真值 0.393 m |
| H3 跟踪层吃掉了避让 | 三个对照臂上不是主因；E-A1 臂上是主因之一 | 全关 / E-B2 / E-B3 窗口内跟踪差最大 0.152–0.174 m（点质量一阶滞后，kp 4 / kd 3）。越线时参考距 0.75–0.79 m、真值 0.63–0.69 m，但参考距之后自己也降到 0.28–0.50 m（越线后再飞 1 s）：完美跟踪也只晚撞几拍。E-A1 臂：悬停后跟踪差 1.242 m（swap8）/ 1.899 m（d014），越线后参考距 0.908 / 1.710 m 而真值 0.393 / 0.540 m |
| H4 对称死锁、互让后漂进去 | 不成立 | 三个对照臂窗口内 400/400 个 tick 都是 `OK`，没有悬停；越线时两机速度 5.76–5.94 m/s，都在加速，没人让。只有 E-A1+刹停原型上出现慢速擦线（接近速度 0.60 m/s），那是 H1 的零余量，不是互等 |

## seed 0 逐臂诊断

窗口是越线前 2 s，逐物理步（100 Hz）记在 runs.json 的 `diagnostics.<tag>.window_ticks`；逐规划拍的预测 / 实际分离在 `plan_events`。「越线后」是脚本让仿真多飞 1 s 的读数，只用来看不截断时的距离，不进结果表。

| 卡 | 臂 | 越线时刻 s | 那一对 | 记下的最小机间 m | 前一拍 m | 接近速度 m/s | 越线后这一对最小 m | 越线后任一对最小 m | 窗口跟踪差最大 m | 预测分离（窗口）m |
|---|---|---|---|---|---|---|---|---|---|---|
| swap8 | 全关 | 2.09 | uav3-uav4 | 0.695 | 0.725 | 2.98 | 0.511 | 0.090 | 0.152 | 0.031–1.340 |
| swap8 | E-B2 | 2.08 | uav3-uav4 | 0.687 | 0.728 | 4.03 | 0.309 | 0.068 | 0.152 | 0.040–1.342 |
| swap8 | E-B3 | 2.10 | uav3-uav4 | 0.674 | 0.709 | 3.41 | 0.451 | 0.226 | 0.154 | 0.039–0.496 |
| swap8 | E-A1 0.70 | 1.77 | uav0-uav1 | 0.661 | 0.711 | 4.90 | 0.393 | 0.219 | 1.242 | 0.712–5.625 |
| swap8 | E-A1 0.70 + 刹停原型 | 2.38 | uav3-uav5 | 0.696 | 0.705 | 0.60 | 0.693 | 0.693 | 0.133 | 0.553–4.595 |
| d014 | 全关 | 2.60 | uav1-uav2 | 0.634 | 0.719 | 8.08 | 0.426 | 0.426 | 0.174 | 0.185–0.958 |
| d014 | E-B2 | 2.64 | uav1-uav2 | 0.630 | 0.717 | 8.33 | 0.401 | 0.401 | 0.169 | 0.174–0.993 |
| d014 | E-B3 | 2.65 | uav1-uav2 | 0.628 | 0.705 | 7.26 | 0.468 | 0.468 | 0.164 | 0.099–0.986 |
| d014 | E-A1 0.70 | 2.82 | uav1-uav2 | 0.698 | 0.746 | 4.57 | 0.540 | 0.540 | 1.899 | 0.705–4.708 |
| d014 | E-A1 0.70 + 刹停原型 | 2.77 | uav1-uav2 | 0.690 | 0.751 | 5.91 | 0.508 | 0.508 | 0.199 | 0.430–4.708 |

全关两张卡的越线时刻、那一对和记下的距离，与 E-B3 对照（未合入，E-B3 worktree 的 `docs/research/e-b3-swarm-penalty.md`）里全关 seed 0（0.695 m、0.634 m）和 [e-c4b](e-c4b-density-seeds.md) 的 014 对向 seed 0（`uav1-uav2:INTER_DRONE`，0.634219779248544 m，2.60 s）相同。E-B2 / E-B3 后段按墙钟截断，同一粒每次跑的数字会变（本票见到的差到 7 cm）；本表 E-B3 是一次重跑（0.674 / 0.628 m），E-B3 报告里同粒是 0.693 / 0.697 m。

各层在这两张卡上看到的门槛（脚本从场景卡和代码缺省读出，`layers` 字段）：

| 层 | 全关 / E-B2 | E-B3 | E-A1 臂 |
|---|---|---|---|
| 深度相机 | 不渲染邻机 | 同左 | 同左 |
| Observation / YOPO 前端 | 没有邻机 | 同左 | 同左；E-A1 在已准入候选上按 0.70 m 过滤 |
| 影子盾 | peers = () | 同左 | 同左 |
| D1 几何回退 | 关（缺省门槛 0.70 m） | 关 | 关 |
| 后段 | 关 / 开，无邻机项 | 开，椭球 C = 0.80 m、c = 2.0，只罚中间控制点 | 关 |
| 碰撞线 | 0.70 m | 0.70 m | 0.70 m |

### E-B3 为什么推不动

| 读数 | d014 seed 0 | swap8 seed 0 |
|---|---|---|
| 窗口里邻机进了 C 的后段调用 | 37 / 60 | 140 / 160 |
| 其中被 0.02 s 预算截断 | 51 / 60 | 128 / 160 |
| 后段前 / 后最近中心距 m | 0.099 / 0.099 | 0.020 / 0.038 |
| 去掉预算、8 次迭代 | 0.098 | 0.036 |
| 去掉预算、100 次迭代 | 0.095 | 0.039 |

d014 的 35 次调用离线复算（`--dump-refine` + `--analyze-dump`，读 `bspline.py` 的内部函数，不改它）：

- 曲线最近点都在 2.68–2.74 s，控制点间隔 dt = 0.208–0.235 s；
- 21 次最近点落在钉死区（头三个控制点由头状态定、尾三个由末端 PVA 定，`bspline.py:188-190`，E-B3 worktree），其中 18 次椭球代价为 0；
- 14 次最近点在可动区间，曲线最近 0.086–0.643 m，可动控制点在同一时刻离邻机 0.141–1.058 m，4 次椭球代价为 0。椭球项只在控制点 Greville 时刻采样（`bspline.py:436`），相对速度 7 m/s 时一个间隔约 1.5 m。

这一节说的是 E-B3 的写法够不着最近点，不是 C、λ 调小了。本票没有改 E-B3，也没有试别的 C。

## 修复

两处都小，都能做成默认关的开关：

1. **无解时从参考头刹停**（`policy_params.upstream.empty_brake`，拟定名，偏离上游）：`CORRIDOR_EMPTY_HOVER` 不再 `hold_traj(s.p)`，而是从上一份参考的 (p, v, a) 用 core 的 `brake_branch` 按 a_max 刹停，和 D1 `_brake_from_head` 是同一个函数（`geom_fallback.py:458-466`）。上游 SPEC 写的是「本帧悬停（停在当前位置）」，所以打开即偏离上游。
2. **E-A1 门槛留跟踪余量**：不用改代码，场景卡上 `inter_drone_threshold_m = 1.0`，即 2 × (机体半径 0.35 + `tracking_margin` 0.15)。0.15 m 是仓库里已有的 `CertifiedPolicy.DEFAULTS["tracking_margin"]`（`policy.py:310`），也和本票测到的正常飞行跟踪差 0.15–0.17 m 同一量级。这是假设，不冻结。

第 1 处要改 `yopo_l0/policy.py`，E-B3 也在改这个文件，还没合入。按票面要求，本票不并行改它：对照里的修复臂是脚本进程里的原型（`--proto-brake`，把 `UpstreamPolicy` 换成只在无解分支上改计划的子类），E-B3 合入后再开 E-B4b 把它做成开关。

### 修复对照（seeds 0–9，只报数）

四臂都跑完 seeds 0–9。「E-A1 1.0 m」就是上游原样的无解悬停（`hold_traj(s.p)`）加 1.0 m 门槛，用来和刹停比。表由 `--collect … --render-md` 生成。撞树都是 `TRUTH_POSITION`，与 [e-c4b](e-c4b-density-seeds.md) 里 safe_radius 0.05 m「走廊过松」同一类，不是机间问题，本票不处理。

消融结论：

- swap8 要两处一起改。只放宽门槛（上游悬停）48/80 到达，仍有 4 粒机间碰撞（seed 0、6、7、8，都是 `uav3-uav5`，0.693–0.695 m）；只换刹停、门槛仍 0.70 m 是 0/80，10 粒全部机间碰撞，0.671–0.699 m 擦线；两处一起 80/80，最小机间 1.069–1.077 m。
- d014 上机间碰撞只靠门槛就去掉了：1.0 m 两臂都是 0；0.70 m + 刹停还剩 1 粒（seed 0，0.690 m）。剩下的失败都是撞树：上游悬停 7 粒，修复 6 粒，0.70 m + 刹停 3 粒。门槛放大以后绕行更多，撞树反而多于 0.70 m 臂，所以 d014 的到达是 0.70 m + 刹停最高（20/30），修复 14/30、上游悬停 11/30、全关 0/30。
- 刹停原型在 d014 上每粒只触发 0–2 拍，修复和上游悬停有 5 粒结果完全相同；swap8 上每粒触发 6–10 拍，是它把 48/80 变成 80/80。

<!-- e-b4-compare:start -->
| 卡 | 臂 | seeds | 到达（架次） | 机间碰撞 | 撞树 | 超时 | 最小机间 m | 最小障碍 m | 刹停原型触发（规划拍） |
|---|---|---|---|---|---|---|---|---|---|
| swap8 | 全关 | 0–9 | 0/80 | 10 | 0 | 0 | 0.695–0.696 | 1.500–1.500 | — |
| swap8 | E-A1 1.0 m | 0–9 | 48/80 | 4 | 0 | 0 | 0.694–0.815 | 1.463–1.500 | — |
| swap8 | E-A1 0.70 m + 刹停原型 | 0–9 | 0/80 | 10 | 0 | 0 | 0.671–0.699 | 1.457–1.500 | 8–13 |
| swap8 | 修复：E-A1 1.0 m + 刹停原型 | 0–9 | 80/80 | 0 | 0 | 0 | 1.069–1.077 | 1.146–1.459 | 6–10 |
| d014-swap3 | 全关 | 0–9 | 0/30 | 10 | 0 | 0 | 0.622–0.698 | 0.486–0.569 | — |
| d014-swap3 | E-A1 1.0 m | 0–9 | 11/30 | 0 | 7 | 0 | 1.099–2.633 | 0.341–0.566 | — |
| d014-swap3 | E-A1 0.70 m + 刹停原型 | 0–9 | 20/30 | 1 | 3 | 0 | 0.690–1.010 | 0.341–0.568 | 0–2 |
| d014-swap3 | 修复：E-A1 1.0 m + 刹停原型 | 0–9 | 14/30 | 0 | 6 | 0 | 1.099–2.633 | 0.324–0.566 | 0–2 |

| 卡 | 臂 | seed | 结果 | 原因 | 到达 | 最小机间 m | 最小障碍 m | 仿真 s |
|---|---|---|---|---|---|---|---|---|
| swap8 | 全关 | 0 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 1.500 | 2.09 |
| swap8 | 全关 | 1 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 1.500 | 2.09 |
| swap8 | 全关 | 2 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 1.500 | 2.09 |
| swap8 | 全关 | 3 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.696 | 1.500 | 2.09 |
| swap8 | 全关 | 4 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 1.500 | 2.09 |
| swap8 | 全关 | 5 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 1.500 | 2.09 |
| swap8 | 全关 | 6 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.696 | 1.500 | 2.09 |
| swap8 | 全关 | 7 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 1.500 | 2.09 |
| swap8 | 全关 | 8 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 1.500 | 2.09 |
| swap8 | 全关 | 9 | COLLISION | `uav3-uav4:INTER_DRONE` | 0/8 | 0.695 | 1.500 | 2.09 |
| swap8 | E-A1 1.0 m | 0 | COLLISION | `uav3-uav5:INTER_DRONE` | 0/8 | 0.694 | 1.500 | 2.82 |
| swap8 | E-A1 1.0 m | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 0.814 | 1.463 | 9.20 |
| swap8 | E-A1 1.0 m | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 0.812 | 1.463 | 8.96 |
| swap8 | E-A1 1.0 m | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 0.814 | 1.463 | 9.20 |
| swap8 | E-A1 1.0 m | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 0.811 | 1.463 | 8.96 |
| swap8 | E-A1 1.0 m | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 0.815 | 1.463 | 9.20 |
| swap8 | E-A1 1.0 m | 6 | COLLISION | `uav3-uav5:INTER_DRONE` | 0/8 | 0.695 | 1.500 | 2.82 |
| swap8 | E-A1 1.0 m | 7 | COLLISION | `uav3-uav5:INTER_DRONE` | 0/8 | 0.694 | 1.500 | 2.82 |
| swap8 | E-A1 1.0 m | 8 | COLLISION | `uav3-uav5:INTER_DRONE` | 0/8 | 0.695 | 1.500 | 2.82 |
| swap8 | E-A1 1.0 m | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 0.812 | 1.463 | 8.96 |
| swap8 | E-A1 0.70 m + 刹停原型 | 0 | COLLISION | `uav3-uav5:INTER_DRONE` | 0/8 | 0.696 | 1.500 | 2.38 |
| swap8 | E-A1 0.70 m + 刹停原型 | 1 | COLLISION | `uav0-uav6:INTER_DRONE` | 0/8 | 0.671 | 1.457 | 2.81 |
| swap8 | E-A1 0.70 m + 刹停原型 | 2 | COLLISION | `uav0-uav6:INTER_DRONE` | 0/8 | 0.674 | 1.457 | 2.81 |
| swap8 | E-A1 0.70 m + 刹停原型 | 3 | COLLISION | `uav0-uav6:INTER_DRONE` | 0/8 | 0.673 | 1.457 | 2.81 |
| swap8 | E-A1 0.70 m + 刹停原型 | 4 | COLLISION | `uav0-uav6:INTER_DRONE` | 0/8 | 0.672 | 1.458 | 2.81 |
| swap8 | E-A1 0.70 m + 刹停原型 | 5 | COLLISION | `uav0-uav6:INTER_DRONE` | 0/8 | 0.673 | 1.458 | 2.81 |
| swap8 | E-A1 0.70 m + 刹停原型 | 6 | COLLISION | `uav3-uav5:INTER_DRONE` | 0/8 | 0.698 | 1.500 | 2.38 |
| swap8 | E-A1 0.70 m + 刹停原型 | 7 | COLLISION | `uav3-uav5:INTER_DRONE` | 0/8 | 0.697 | 1.500 | 2.38 |
| swap8 | E-A1 0.70 m + 刹停原型 | 8 | COLLISION | `uav3-uav5:INTER_DRONE` | 0/8 | 0.699 | 1.500 | 2.38 |
| swap8 | E-A1 0.70 m + 刹停原型 | 9 | COLLISION | `uav0-uav6:INTER_DRONE` | 0/8 | 0.672 | 1.457 | 2.81 |
| swap8 | 修复：E-A1 1.0 m + 刹停原型 | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.077 | 1.459 | 8.86 |
| swap8 | 修复：E-A1 1.0 m + 刹停原型 | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.069 | 1.146 | 8.78 |
| swap8 | 修复：E-A1 1.0 m + 刹停原型 | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.069 | 1.146 | 8.78 |
| swap8 | 修复：E-A1 1.0 m + 刹停原型 | 3 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.070 | 1.146 | 8.78 |
| swap8 | 修复：E-A1 1.0 m + 刹停原型 | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.070 | 1.146 | 8.78 |
| swap8 | 修复：E-A1 1.0 m + 刹停原型 | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.070 | 1.146 | 8.78 |
| swap8 | 修复：E-A1 1.0 m + 刹停原型 | 6 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.075 | 1.459 | 8.86 |
| swap8 | 修复：E-A1 1.0 m + 刹停原型 | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.076 | 1.459 | 8.86 |
| swap8 | 修复：E-A1 1.0 m + 刹停原型 | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.076 | 1.459 | 8.86 |
| swap8 | 修复：E-A1 1.0 m + 刹停原型 | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 8/8 | 1.069 | 1.146 | 8.78 |
| d014-swap3 | 全关 | 0 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.634 | 0.486 | 2.60 |
| d014-swap3 | 全关 | 1 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.638 | 0.531 | 2.61 |
| d014-swap3 | 全关 | 2 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.629 | 0.569 | 2.60 |
| d014-swap3 | 全关 | 3 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.698 | 0.556 | 2.60 |
| d014-swap3 | 全关 | 4 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.622 | 0.550 | 2.61 |
| d014-swap3 | 全关 | 5 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.684 | 0.546 | 2.61 |
| d014-swap3 | 全关 | 6 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.683 | 0.517 | 2.61 |
| d014-swap3 | 全关 | 7 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.658 | 0.553 | 2.60 |
| d014-swap3 | 全关 | 8 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.666 | 0.530 | 2.60 |
| d014-swap3 | 全关 | 9 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.629 | 0.536 | 2.61 |
| d014-swap3 | E-A1 1.0 m | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 1.653 | 0.519 | 16.32 |
| d014-swap3 | E-A1 1.0 m | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 1.310 | 0.357 | 8.89 |
| d014-swap3 | E-A1 1.0 m | 2 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1.134 | 0.348 | 4.41 |
| d014-swap3 | E-A1 1.0 m | 3 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 2.411 | 0.346 | 2.66 |
| d014-swap3 | E-A1 1.0 m | 4 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 2.633 | 0.347 | 2.63 |
| d014-swap3 | E-A1 1.0 m | 5 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1.167 | 0.347 | 4.49 |
| d014-swap3 | E-A1 1.0 m | 6 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 1.272 | 0.341 | 3.00 |
| d014-swap3 | E-A1 1.0 m | 7 | COLLISION | `uav0:TRUTH_POSITION` | 2/3 | 1.136 | 0.341 | 8.65 |
| d014-swap3 | E-A1 1.0 m | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 1.308 | 0.566 | 16.72 |
| d014-swap3 | E-A1 1.0 m | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1.099 | 0.346 | 4.36 |
| d014-swap3 | E-A1 0.70 m + 刹停原型 | 0 | COLLISION | `uav1-uav2:INTER_DRONE` | 0/3 | 0.690 | 0.568 | 2.77 |
| d014-swap3 | E-A1 0.70 m + 刹停原型 | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.984 | 0.531 | 8.34 |
| d014-swap3 | E-A1 0.70 m + 刹停原型 | 2 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.926 | 0.426 | 16.18 |
| d014-swap3 | E-A1 0.70 m + 刹停原型 | 3 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1.010 | 0.348 | 4.12 |
| d014-swap3 | E-A1 0.70 m + 刹停原型 | 4 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.916 | 0.373 | 16.72 |
| d014-swap3 | E-A1 0.70 m + 刹停原型 | 5 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 0.785 | 0.341 | 4.32 |
| d014-swap3 | E-A1 0.70 m + 刹停原型 | 6 | COLLISION | `uav0:TRUTH_POSITION` | 2/3 | 0.783 | 0.347 | 8.69 |
| d014-swap3 | E-A1 0.70 m + 刹停原型 | 7 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.928 | 0.443 | 8.82 |
| d014-swap3 | E-A1 0.70 m + 刹停原型 | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.828 | 0.480 | 16.46 |
| d014-swap3 | E-A1 0.70 m + 刹停原型 | 9 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 0.908 | 0.555 | 8.97 |
| d014-swap3 | 修复：E-A1 1.0 m + 刹停原型 | 0 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 1.653 | 0.519 | 16.32 |
| d014-swap3 | 修复：E-A1 1.0 m + 刹停原型 | 1 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 1.310 | 0.357 | 8.89 |
| d014-swap3 | 修复：E-A1 1.0 m + 刹停原型 | 2 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1.134 | 0.348 | 4.41 |
| d014-swap3 | 修复：E-A1 1.0 m + 刹停原型 | 3 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 2.411 | 0.346 | 2.66 |
| d014-swap3 | 修复：E-A1 1.0 m + 刹停原型 | 4 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 2.633 | 0.347 | 2.63 |
| d014-swap3 | 修复：E-A1 1.0 m + 刹停原型 | 5 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 1.167 | 0.354 | 16.72 |
| d014-swap3 | 修复：E-A1 1.0 m + 刹停原型 | 6 | COLLISION | `uav0:TRUTH_POSITION` | 0/3 | 1.281 | 0.324 | 2.88 |
| d014-swap3 | 修复：E-A1 1.0 m + 刹停原型 | 7 | COLLISION | `uav0:TRUTH_POSITION` | 2/3 | 1.136 | 0.341 | 8.65 |
| d014-swap3 | 修复：E-A1 1.0 m + 刹停原型 | 8 | SUCCESS | `GOAL_REACHED_AND_HELD` | 3/3 | 1.308 | 0.566 | 16.72 |
| d014-swap3 | 修复：E-A1 1.0 m + 刹停原型 | 9 | COLLISION | `uav1:TRUTH_POSITION` | 0/3 | 1.099 | 0.348 | 4.35 |
<!-- e-b4-compare:end -->

## 设定

| 项 | 值 |
|---|---|
| 代码 | main `ea62b30`（本分支起点）；E-B3 臂用 `/workspace/wt/yopo-restart/yopo-restart-bot-e-b3-swarm-penalty` 未提交的改动 |
| 层级 | L0，`upstream`，影子盾开（只记录） |
| 卡 | `scenes/swap-8-open.json`、`scenes/ego-density-014-swap3.json`，未改 |
| 臂的覆盖 | 只在进程里改 `policy_params`，不写进 `scenes/`。E-B2 `backend.enabled`；E-B3 再加 `swarm_penalty`、C 0.80、c 2.0（与 E-B3 对照同一组假设）；E-A1 `inter_drone_filter`、阈值 0.70 m、权重 0（与 E-C4 / 联合对照同一组）；修复臂 E-A1 阈值 1.0 m + `--proto-brake` |
| 跑法 | `nice -n 10`，最多 2 个进程，setsid nohup，逐粒落 `out/e-b4/diag/`，可续跑 |
| 时间 | 2026-11-12 05:06 起（America/Argentina/San_Juan） |

## 怎么重跑

```bash
make venv
# seed 0 诊断（越线后多飞 1 s，只用来看不截断的距离）
.venv/bin/python scripts/e-b4-peer-diag.py --scene swap8 --arm off --seed 0 --out out/e-b4/diag/swap8-off-0.json
.venv/bin/python scripts/e-b4-peer-diag.py --scene d014-swap3 --arm a1 --seed 0 --proto-brake --out out/e-b4/diag/d014-a1brake-0.json
# E-B3（未合入时指向 E-B3 worktree 的代码）
EB3=/workspace/wt/yopo-restart/yopo-restart-bot-e-b3-swarm-penalty
$EB3/.venv/bin/python scripts/e-b4-peer-diag.py --code-root $EB3 --scene d014-swap3 --arm b3 --seed 0 \
  --dump-refine out/e-b4/d014-b3-refine.pkl --out out/e-b4/diag/d014-b3r-0.json
$EB3/.venv/bin/python scripts/e-b4-peer-diag.py --code-root $EB3 --analyze-dump out/e-b4/d014-b3-refine.pkl --out out/e-b4/d014-b3-refine-analysis.json
# 对照（运行器原样判碰撞）
.venv/bin/python scripts/e-b4-peer-diag.py --scene swap8 --arm off --seed 3 --compare --out out/e-b4/diag/cmp-swap8-off-3.json
.venv/bin/python scripts/e-b4-peer-diag.py --scene swap8 --arm a1 --extra '{"upstream":{"inter_drone_threshold_m":1.0}}' \
  --proto-brake --seed 3 --compare --out out/e-b4/diag/cmp-swap8-fix-3.json
```

全关臂在同 seed 下逐位确定。E-B2 / E-B3 后段按墙钟截断，数字会随机器负载变（同一粒差到 7 cm）。
