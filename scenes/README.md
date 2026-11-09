# 场景

- `g2-scene.json`：原样复制自 yopo-swarm `sim/px4/g2-scene.json`（@00b85bf），与 L1 Gazebo 世界同一份几何。不要手改；改几何就新建文件。
- `g2-hover.json`：票 01 场景卡，单机在起点上方 1.5 m 悬停。
- `g2-hover-lift.json`：同几何，从 1.2 m 起到 1.5 m 悬停点（检验动力学跟踪）。
- `g2-pillar.json`：票 02 / M0，单机从 (0,0,1.5) 飞到 G2 目标 (5,0,1.5)，绕柱；判据未冻结（`criteria.frozen=false`）。几何仍是 `g2-scene.json`。套件里用 `@upstream`。
- `open-area.json`：开阔区，无墙无柱，只有场景卡里的地面。
- `open-3drone.json`：票 05。三机沿三条平行线飞向各自的 `goal_enu`（横向 5 m）。套件里用 `@hover`。判据未冻结。
- `formation-hold.json`：票 06 / M1。三机按虚拟中心（直线 + 平滑左转 + 直线）和机体槽位保持队形。运动更新走轮转提案窗口和预留协议，每机只用本机批准的计划。判据未冻结（`criteria.frozen=false`）。
- `formation-drop.json` / `formation-duplicate.json` / `formation-delay.json` / `formation-reorder.json`：票 06b。同一编队任务，分别打开总线上的丢包、重复、时延、乱序。错过同步出发的飞机沿本槽追赶。判据未冻结。
- `crossing-pillars.json` + `crossing-pillars-scene.json`：票 07 / M2。三机穿柱林。柱子是 G2 那根盒子柱（0.6 × 0.8 × 2.8 m），4 列 × 3 行，列距 3.5 m，行距 8 m，落在三条航线上。走廊内宽 28 m（左右墙内侧面 y=±14），比 G2 的 6.6 m 宽，三机各自绕柱也不会占住邻机的航线。天花板 z=4.0。队形槽位只作为 YOPO 的目标偏置，避障优先；每机只飞本机 COMMIT 的计划。判据未冻结（`criteria.frozen=false`）。seeds 0–9。套件里用 `@crossing`。
- `crossing-forest.json`：票 07b。没有几何文件，`forest.density` 为 `sparse` / `medium` / `dense`，树的位置由 `yopo_l0.forest` 按 `forest.seed` 做泊松盘采样。入口疏、中段密、密林后是空地。密档中段树干表面最小净距 1.7 m（在 1.5–2.0 m 里）。树干半径约 0.10–0.40 m，一部分倾斜，一部分在飞行高度有矮枝。规划参数与票 07 相同。默认 medium。seeds 0–9 只改深度噪声，不改树。判据未冻结。
- `formation-switch.json` + `formation-switch-scene.json`：票 08 / M3 前半。开阔区中间一道窄门（两根柱，净空 2.1 m）。三机横排出发，前方净空低于 `width_threshold_m` 就经预留协议切到纵列，出门后再切回横排。每机只飞本机 COMMIT 的计划。判据未冻结（`criteria.frozen=false`）。seeds 0–9。套件里用 `@switch`。
- `pillar-forest.json` + `pillar-forest-scene.json`：柱林。12 根竖直圆柱，半径 0.3 m，高 2.8 m（与 G2 那根柱子同高），4 列 × 3 行，列距 2.5 m、行距 2.4 m，最近柱心 (3, 0)。天花板 z=4.0，与 G2 一致。当前任务是悬停。三机穿越在 `crossing-pillars.json`（票 07），不改这份圆柱林。
- `narrow-corridor.json` + `narrow-corridor-scene.json`：窄廊。左右墙内侧面 y=±0.8，内宽 1.6 m（G2 内宽 6.6 m）。当前任务是悬停。
- `suite.txt`：`卡@策略` 或 `卡@策略@seed` 列表，`scripts/regress.sh` 与 `scripts/nightly.sh` 共用。含 `g2-pillar.json@upstream`、`open-3drone.json@hover`、`formation-hold.json@formation`、票 06b 的四张总线故障卡、`crossing-pillars.json@crossing`、`formation-switch.json@switch` 和 `crossing-forest.json@crossing@0`（medium，只跑 seed 0）。

场景卡字段：`geometry`（相对路径；带 `forest` 的卡没有这份文件，加载时按种子生成）、`ground_z`、`drones[].start_enu`（hover 任务可选 `hover_enu`；goto 任务用 `goal_enu`；formation / crossing / switch 任务用 `slot_body`，起点必须等于虚拟中心起点处的槽位）、`task`（`hover`、`goto`、`formation`、`crossing` 或 `switch`）、`criteria`、`camera`、`dynamics`、`rates_hz`、`seeds`。可选 `forest`（`density` 为 `sparse` / `medium` / `dense`，`seed` 只决定树的位置）和 `bus`：`drop`、`duplicate`、`delay`（`[lo, hi]` 秒）、`reorder`，含义见 `yopo_l0/bus.py`。`RunConfig.bus` 覆盖场景卡。
goto 到达沿用 G2 oracle：`goal_tolerance_m` / `goal_speed_mps` / `goal_hold_s`。机间碰撞看中心距是否 ≤ 两倍 `body_radius_m`。
场景卡哈希 = 场景卡 + 内联几何的规范 JSON 的 sha256，写进结果记录。

柱林和窄廊复用 `g2-hover.json` 的相机、动力学、判据，任务是 hover。开阔区几何用票 05 的空世界，不另抄一份。
