# 场景

- `g2-scene.json`：原样复制自 yopo-swarm `sim/px4/g2-scene.json`（@00b85bf），与 L1 Gazebo 世界同一份几何。不要手改；改几何就新建文件。
- `g2-hover.json`：票 01 场景卡，单机在起点上方 1.5 m 悬停。
- `g2-hover-lift.json`：同几何，从 1.2 m 起到 1.5 m 悬停点（检验动力学跟踪）。
- `g2-pillar.json`：票 02 / M0，单机从 (0,0,1.5) 飞到 G2 目标 (5,0,1.5)，绕柱；判据未冻结（`criteria.frozen=false`）。几何仍是 `g2-scene.json`。套件里用 `@upstream`。
- `open-area.json`：开阔区，无墙无柱，只有场景卡里的地面。
- `open-3drone.json`：票 05。三机沿三条平行线飞向各自的 `goal_enu`（横向 5 m）。套件里用 `@hover`。判据未冻结。
- `formation-hold.json`：票 06 / M1。三机按虚拟中心（直线 + 平滑左转 + 直线）和机体槽位保持队形。运动更新走轮转提案窗口和预留协议，每机只用本机批准的计划。判据未冻结（`criteria.frozen=false`）。
- `formation-drop.json` / `formation-duplicate.json` / `formation-delay.json` / `formation-reorder.json`：票 06b。同一编队任务，分别打开总线上的丢包、重复、时延、乱序。错过同步出发的飞机沿本槽追赶。判据未冻结。
- `pillar-forest.json` + `pillar-forest-scene.json`：柱林。12 根竖直圆柱，半径 0.3 m，高 2.8 m（与 G2 那根柱子同高），4 列 × 3 行，列距 2.5 m、行距 2.4 m，最近柱心 (3, 0)。天花板 z=4.0，与 G2 一致。当前任务是悬停；穿越是票 07。
- `narrow-corridor.json` + `narrow-corridor-scene.json`：窄廊。左右墙内侧面 y=±0.8，内宽 1.6 m（G2 内宽 6.6 m）。当前任务是悬停。
- `suite.txt`：`卡@策略` 列表，`scripts/regress.sh` 与 `scripts/nightly.sh` 共用。含 `g2-pillar.json@upstream`、`open-3drone.json@hover`、`formation-hold.json@formation`，以及票 06b 的四张总线故障卡。

场景卡字段：`geometry`（相对路径）、`ground_z`、`drones[].start_enu`（hover 任务可选 `hover_enu`；goto 任务用 `goal_enu`；formation 任务用 `slot_body`，起点必须等于虚拟中心起点处的槽位）、`task`（`hover`、`goto` 或 `formation`）、`criteria`、`camera`、`dynamics`、`rates_hz`、`seeds`。可选 `bus`：`drop`、`duplicate`、`delay`（`[lo, hi]` 秒）、`reorder`，含义见 `yopo_l0/bus.py`。`RunConfig.bus` 覆盖场景卡。
goto 到达沿用 G2 oracle：`goal_tolerance_m` / `goal_speed_mps` / `goal_hold_s`。机间碰撞看中心距是否 ≤ 两倍 `body_radius_m`。
场景卡哈希 = 场景卡 + 内联几何的规范 JSON 的 sha256，写进结果记录。

柱林和窄廊复用 `g2-hover.json` 的相机、动力学、判据，任务是 hover。开阔区几何用票 05 的空世界，不另抄一份。
