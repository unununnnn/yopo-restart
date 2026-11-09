# 场景

- `g2-scene.json`：原样复制自 yopo-swarm `sim/px4/g2-scene.json`（@00b85bf），与 L1 Gazebo 世界同一份几何。不要手改；改几何就新建文件。
- `g2-hover.json`：票 01 场景卡，单机在起点上方 1.5 m 悬停。
- `g2-hover-lift.json`：同几何，从 1.2 m 起到 1.5 m 悬停点（检验动力学跟踪）。
- `g2-pillar.json`：票 02 / M0，单机从 (0,0,1.5) 飞到 G2 目标 (5,0,1.5)，绕柱；判据未冻结（`criteria.frozen=false`）。
- `open-area.json`：开阔区，无墙无柱，只有场景卡里的地面。
- `open-3drone.json`：票 05。三机沿三条平行线飞向各自的 `goal_enu`（横向 5 m）。验收跑 `--policy hover`（点质量跟踪该点）。判据未冻结。

场景卡字段：`geometry`（相对路径）、`ground_z`、`drones[].start_enu`（hover 任务可选 `hover_enu`；goto 任务用 `goal_enu`）、`task`（`hover` 或 `goto`）、`criteria`、`camera`、`dynamics`、`rates_hz`、`seeds`。可选 `bus`：`drop`、`duplicate`、`delay`（`[lo, hi]` 秒）、`reorder`，含义见 `yopo_l0/bus.py`。`RunConfig.bus` 覆盖场景卡。
goto 到达沿用 G2 oracle：`goal_tolerance_m` / `goal_speed_mps` / `goal_hold_s`。机间碰撞看中心距是否 ≤ 两倍 `body_radius_m`。
场景卡哈希 = 场景卡 + 内联几何的规范 JSON 的 sha256，写进结果记录。
