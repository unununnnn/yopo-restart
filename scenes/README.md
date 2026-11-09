# 场景

- `g2-scene.json`：原样复制自 yopo-swarm `sim/px4/g2-scene.json`（@00b85bf），与 L1 Gazebo 世界同一份几何。不要手改；改几何就新建文件。
- `g2-hover.json`：票 01 场景卡，单机在起点上方 1.5 m 悬停。
- `g2-hover-lift.json`：同几何，从 1.2 m 起到 1.5 m 悬停点（检验动力学跟踪）。
- `g2-pillar.json`：票 02 / M0，单机从 (0,0,1.5) 飞到 G2 目标 (5,0,1.5)，绕柱；判据未冻结（`criteria.frozen=false`）。

场景卡字段：`geometry`（相对路径）、`ground_z`、`drones[].start_enu`（可选 `hover_enu`）、`task`、`criteria`、`camera`、`dynamics`、`rates_hz`、`seeds`。
场景卡哈希 = 场景卡 + 内联几何的规范 JSON 的 sha256，写进结果记录。
