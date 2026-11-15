# E-B3 收口

E-B3（`f5cedba`，PR #65）和 E-B3c（`41e820f`，PR #76）的实测没有把对向卡上的机间碰撞降下来。这条实现线到此收口。数字只来自 [e-b3-swarm-penalty.md](e-b3-swarm-penalty.md) 和 [e-b3c-segment-penalty.md](e-b3c-segment-penalty.md)。本票只改文档和票表，不改代码、测试、默认值，也不冻结新判据。

## 实测

种子 0–9。d014-swap3 是 `scenes/ego-density-014-swap3.json`，到达分母 30。swap8 是 `scenes/swap-8-open.json`，到达分母 80。`off` / `b2` / `b3` 在 E-B3 对照里，`off` / `b3` / `b3c` 在 E-B3c 对照里。两张卡上这些臂都是机间碰撞 10/10。d014-swap3 到达 0/30，swap8 到达 0/80。

swap8 上，E-B3c 对照里 b3 的最小机间距离下沿是 0.673 m。b3c 的下沿更低：seed 5 表上写成 0.625 m，精确值 0.6247 m（`uav3-uav6`）。表上的最小机间距离是运行器第一次越过 0.70 m 那一拍的采样。

## 根因

E-B4 曾把 E-B3 推不动归到采样位置：惩罚打在控制点上，够不着段内或钉死端点上的最近点。E-B3c 把同一条椭球铰链改到每段 6 个内点（等分箱中点，不含段端点）。碰撞数仍是 10/10。采样位置这一缺口已经排除。

剩下的核心缺口是惩罚形状和碰撞线不是同一个球。椭球 `d = sqrt(dx² + dy² + (c·dz)²)`，`c = 2`，铰链在 `d < 0.80` m 时才亮。竖直半轴是 `C/c = 0.40` m，短于 L0 机间碰撞球半径 0.70 m。纯竖直拉开 0.40 m 时铰链关掉，欧氏距离仍是 0.40 m，机体中心还在 0.70 m 的碰撞球里面。梯度的 z 分量带 `c² = 4`，优化器更愿意往上或往下躲。躲完椭球，球还没出。

## 结论

E-B3 实现线正式收口。不再开后续调参票，也不再开在这条椭球上叠加欧氏铰链的实现票。若将来在后段做机间避碰，另起新线，直接对准 0.70 m 欧氏碰撞球。

## 开关保留

现有代码留下，不作为死代码删除。`policy_params.backend.enabled`、`swarm_penalty`、`swarm_segment_penalty` 继续全部默认关。已有场景卡不写这些字段。`tests/test_backend_swarm.py` 的 `test_swarm_off_ignores_neighbors_bitwise` 守住机间开关关闭时，把邻机采样传进 `refine_yopo` 与不传逐位相同。`test_segment_flag_off_matches_control_point_refine` 守住段内开关关掉时与只开 E-B3 逐位相同。`test_existing_scene_cards_leave_swarm_off` 和 `test_existing_scene_cards_leave_segment_penalty_off` 守住已有场景卡和 `suite.txt` 不写这些字段。

## L0-DET 确定性重跑（2026-11-15）

数字来自 [e-b3-swarm-penalty.md](e-b3-swarm-penalty.md) 和 [e-b3c-segment-penalty.md](e-b3c-segment-penalty.md) 文末的重跑，不另开仿真。两张卡、各臂仍是机间碰撞 10/10，到达仍是 0。收口「椭球线没有把对向撞机降下来」没变。

变的是 swap8 上 b3c 的那一个下沿。上文写 seed 5 表上 0.625 m、精确值 0.6247 m，比 b3 的 0.673 m 更低。重跑里 seed 5 是 0.699 m，b3c 下沿 0.677 m，与 b3 下沿相同。采样位置这一缺口仍被排除：碰撞数没有因为改到段内就降下来。
