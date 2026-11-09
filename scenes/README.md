# 场景

- `g2-scene.json`：原样复制自 yopo-swarm `sim/px4/g2-scene.json`（@00b85bf），与 L1 Gazebo 世界同一份几何。不要手改；改几何就新建文件。
- `g2-hover.json`：票 01 场景卡，单机在起点上方 1.5 m 悬停。
- `g2-hover-lift.json`：同几何，从 1.2 m 起到 1.5 m 悬停点（检验动力学跟踪）。
- `g2-pillar.json`：票 02 / M0，单机从 (0,0,1.5) 飞到 G2 目标 (5,0,1.5)，绕柱；判据未冻结（`criteria.frozen=false`）。几何仍是 `g2-scene.json`。套件里用 `@upstream`。
- `open-area.json`：开阔区，无墙无柱，只有场景卡里的地面。
- `open-3drone.json`：票 05。三机沿三条平行线飞向各自的 `goal_enu`（横向 5 m）。套件里用 `@hover`。判据未冻结。
- `formation-hold.json`：票 06 / M1。三机按虚拟中心（直线 + 平滑左转 + 直线）和机体槽位保持队形。运动更新走轮转提案窗口和预留协议，每机只用本机批准的计划。判据未冻结（`criteria.frozen=false`）。
- `formation-6-open.json`：票 11a / M4 开阔区。六机楔形，虚拟中心与三机卡相同。两架在前方（横向 ±1.5 m），中间两架后退 3 m（横向 ±3.5 m），最后两架再退 3 m（横向 ±5.5 m）。没有一架正跟在另一架的航迹上：正后方的槽位会穿过前机刹车分支的保持点，邻机占用把提案判成冲突，后机就一直悬停。这是场景几何的选择，不改 `formation.py`。seeds 0–4。判据未冻结。进程内、零故障、seed 0 的墙钟低于 60 s，所以 `suite.txt` 里有这一条。故障档位在 `formation-6-tiers.json`（none / drop 5% / drop 20% / duplicate 10% / reorder+delay / combined）。柱林六机不在本卡。
- `pillar-forest.json` + `pillar-forest-scene.json`：柱林。12 根竖直圆柱，半径 0.3 m，高 2.8 m（与 G2 那根柱子同高），4 列 × 3 行，列距 2.5 m、行距 2.4 m，最近柱心 (3, 0)。天花板 z=4.0，与 G2 一致。当前任务是悬停；穿越是票 07。
- `narrow-corridor.json` + `narrow-corridor-scene.json`：窄廊。左右墙内侧面 y=±0.8，内宽 1.6 m（G2 内宽 6.6 m）。当前任务是悬停。
- `suite.txt`：`卡@策略` 列表，`scripts/regress.sh` 与 `scripts/nightly.sh` 共用。含 `g2-pillar.json@upstream`、`open-3drone.json@hover`、`formation-hold.json@formation` 和 `formation-6-open.json@formation`（票 11a，仅进程内零故障）。

场景卡字段：`geometry`（相对路径）、`ground_z`、`drones[].start_enu`（hover 任务可选 `hover_enu`；goto 任务用 `goal_enu`；formation 任务用 `slot_body`，起点必须等于虚拟中心起点处的槽位）、`task`（`hover`、`goto` 或 `formation`）、`criteria`、`camera`、`dynamics`、`rates_hz`、`seeds`。可选 `bus`：`drop`、`duplicate`、`delay`（`[lo, hi]` 秒）、`reorder`，含义见 `yopo_l0/bus.py`。`RunConfig.bus` 覆盖场景卡。
goto 到达沿用 G2 oracle：`goal_tolerance_m` / `goal_speed_mps` / `goal_hold_s`。机间碰撞看中心距是否 ≤ 两倍 `body_radius_m`。
场景卡哈希 = 场景卡 + 内联几何的规范 JSON 的 sha256，写进结果记录。

柱林和窄廊复用 `g2-hover.json` 的相机、动力学、判据，任务是 hover。开阔区几何用票 05 的空世界，不另抄一份。
