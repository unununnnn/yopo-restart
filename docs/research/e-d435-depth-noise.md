# 票 E-D435：D435 深度噪声与延迟下的 YOPO L0

只报数。report-only。``d435_depth`` 默认关，场景卡、``scenes/suite.txt``、已冻门都没改。打开即偏离上游：上游权重在真值深度上训练（右目关闭）。无效像素保持 NaN，网络侧用上游 NS inpaint；本表没有用 inpaint 预填，也没有填最大量程。飞点概率 0.3、掠射角阈值 75°、视差噪声相关长度 3 px、丢帧率 0.5%，以及约 21 ms 里 Orin 额外的约 5 ms，置信度低。仿真相机不是 848×480。噪声用该相机的 fx，米制 σ 比原生 D435 大。编队任务不读深度，轨迹不该因这只开关改变；影子盾仍会看着深度图。影子盾只记录 would_reject，不改批准的计划。墙钟不进结论。

## 接在哪里

深度只在 `yopo_l0/sensor.py` 的 `render` 进策略之前改一次。
`d435 is None` 时仍是原来的高斯噪声三行，随机数抽取次数不变。
运行器读 `task.d435_depth`。缺省、`false`、或 `enabled: false` 都不建噪声状态，结果里也不加字段。
名字登记在 `yopo_l0/experiment.py` 的 `RESEARCH_SWITCHES`，收口脚本据此拒绝场景卡把该键写成 JSON true。
本票没有改 `yopo_l0/crossing.py` 和 `yopo_l0/sitl_bridge.py`。交叉任务读的仍是相机交出去的那一帧。

## 模型和开关

- 轴向噪声在视差域：`σ_Z = Z² · s / (fx · B)`，`B = 0.05` m，默认 `s = 0.08`。`fx` 用场景卡相机，不把图放大到 848×480 再缩回来。
- 每架飞机每个回合抽一次视差偏置 `N(0, 0.1)` px，截断到 ±0.25 px。噪声、空洞、延迟用三条独立随机流。
- 空洞/阴影：最小距离 0.195 m、左侧无效带、前景遮挡阴影、4–8 m 掉点、10 m 截断、掠射角、1% 随机洞、块状洞、0.3% 假近点。
- 延迟：捕获时刻入队，送达后再给策略。默认 `max(10, N(18, 3)) + 0.5×8` ms，0.5% 尖峰加 20–60 ms，0.5% 丢帧。帧龄是锁步里的采样保持，不再另抽 `U(0, 1/fps)`。
- 压测臂把输送延迟固定成 50 ms（抖动和尖峰为 0，丢帧率仍是 0.5%），`s = 0.20`。
- 无效像素：`nan`。`network_depth` 对 NaN 和小于 0.04 m 的像素做上游 NS radius-1 inpaint。安全证据和影子盾仍把 NaN 当未知。`inpaint` 和 `max_range` 两个填充留在 `yopo_l0/depth_noise.py`，这次没打开。

置信度低、仍然用了调研表默认值的参数：飞点概率 `p_flying = 0.3`，掠射角阈值 75°，视差噪声相关长度 3 px，丢帧率 0.5%，Orin 额外延迟约 5 ms（含在大约 21 ms 的合计里）。

原生 D435 在 848×480、`fx ≈ 426` 时，2 m 处 σ ≈ 15 mm。下表是这些场景卡自己的焦距，噪声按这个焦距算。

| 卡 | 任务 | 机数 | 时长 s | 分辨率 | fx px | 2 m 处 σ mm（s=0.08） | 2 m 处 σ mm（s=0.20） |
|---|---|---:|---:|---|---:|---:|---:|
| g2 柱子单机 | goto | 1 | 45 | 320×240 | 216.2 | 29.6 | 74.0 |
| 密度 0.14 单机 | goto | 1 | 50 | 160×96 | 80.0 | 80.0 | 200.0 |
| crossing-pillars 三机 | crossing | 3 | 45 | 160×96 | 80.0 | 80.0 | 200.0 |
| 六机林带 sparse | crossing | 6 | 50 | 160×96 | 80.0 | 80.0 | 200.0 |
| 开阔三机编队 | formation | 3 | 40 | 160×96 | 80.0 | 80.0 | 200.0 |

## 结果

seeds 0–9，每格 10 个种子。到达是到达飞机 / 飞机数。障碍碰撞、机间碰撞是发生了碰撞的运行次数 / 种子数（仿真在第一次碰撞停下，一次运行最多记 1）。净空 = 机体中心到障碍表面的距离 − 机体半径。最小净空是种子里最差的一次，中位净空是这些种子的中位数。平均到达时间只平均已经整队到达的种子。影子盾介入是 `would_reject` 的 tick 合计。

| 卡 | 臂 | 偏离上游 | 到达 | 障碍碰撞 | 机间碰撞 | 最小净空 m | 中位净空 m | 最小机间 m | 越门 | 平均到达时间 s | 影子盾介入 |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---:|---:|
| g2 柱子单机 | 基线（关） | 否 | 10/10 (1.00) | 0/10 | 0/10 | 0.478 | 0.479 | — | 否 | 4.271 | 5271 |
| g2 柱子单机 | 只开噪声 | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.385 | 0.462 | — | 否 | 4.040 | 5040 |
| g2 柱子单机 | 噪声 + 空洞/阴影 | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.357 | 0.456 | — | 否 | 3.847 | 4847 |
| g2 柱子单机 | 全开（含延迟） | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.399 | 0.497 | — | 否 | 4.199 | 5199 |
| g2 柱子单机 | 压测 s=0.20、延迟 50 ms | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.184 | 0.466 | — | 否 | 3.996 | 4996 |
| 密度 0.14 单机 | 基线（关） | 否 | 9/10 (0.90) | 1/10 | 0/10 | -0.001 | 0.112 | — | 未冻结 | 12.501 | 12591 |
| 密度 0.14 单机 | 只开噪声 | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.067 | 0.178 | — | 未冻结 | 8.996 | 9996 |
| 密度 0.14 单机 | 噪声 + 空洞/阴影 | 偏离上游 | 0/10 (0.00) | 10/10 | 0/10 | -0.014 | -0.002 | — | 未冻结 | — | 2201 |
| 密度 0.14 单机 | 全开（含延迟） | 偏离上游 | 0/10 (0.00) | 10/10 | 0/10 | -0.016 | -0.002 | — | 未冻结 | — | 1798 |
| 密度 0.14 单机 | 压测 s=0.20、延迟 50 ms | 偏离上游 | 0/10 (0.00) | 10/10 | 0/10 | -0.022 | -0.004 | — | 未冻结 | — | 2362 |
| crossing-pillars 三机 | 基线（关） | 否 | 30/30 (1.00) | 0/10 | 0/10 | 0.528 | 0.904 | 3.350 | 否 | 7.270 | 24810 |
| crossing-pillars 三机 | 只开噪声 | 偏离上游 | 30/30 (1.00) | 0/10 | 0/10 | 0.741 | 0.876 | 2.826 | 否 | 7.518 | 25554 |
| crossing-pillars 三机 | 噪声 + 空洞/阴影 | 偏离上游 | 18/30 (0.60) | 4/10 | 0/10 | -0.032 | 0.032 | 3.219 | 是（24） | 9.405 | 23019 |
| crossing-pillars 三机 | 全开（含延迟） | 偏离上游 | 15/30 (0.50) | 5/10 | 0/10 | -0.012 | 0.088 | 3.553 | 是（24） | 9.588 | 21318 |
| crossing-pillars 三机 | 压测 s=0.20、延迟 50 ms | 偏离上游 | 24/30 (0.80) | 2/10 | 0/10 | -0.014 | 0.372 | 3.907 | 是（15） | 8.856 | 25761 |
| 六机林带 sparse | 基线（关） | 否 | 60/60 (1.00) | 0/10 | 0/10 | 0.158 | 0.284 | 0.959 | 否 | 26.741 | 166446 |
| 六机林带 sparse | 只开噪声 | 偏离上游 | 60/60 (1.00) | 0/10 | 0/10 | 0.150 | 0.265 | 0.960 | 否 | 32.036 | 198216 |
| 六机林带 sparse | 噪声 + 空洞/阴影 | 偏离上游 | 4/60 (0.07) | 9/10 | 0/10 | -0.004 | -0.001 | 0.970 | 是（21） | — | 89418 |
| 六机林带 sparse | 全开（含延迟） | 偏离上游 | 0/60 (0.00) | 10/10 | 0/10 | -0.003 | -0.001 | 1.041 | 是（22） | — | 92946 |
| 六机林带 sparse | 压测 s=0.20、延迟 50 ms | 偏离上游 | 56/60 (0.93) | 1/10 | 0/10 | -0.001 | 0.121 | 0.989 | 是（6） | 39.244 | 253494 |
| 开阔三机编队 | 基线（关） | 否 | 30/30 (1.00) | 0/10 | 0/10 | 1.150 | 1.150 | 1.987 | 否 | 18.630 | 58890 |
| 开阔三机编队 | 只开噪声 | 偏离上游 | 30/30 (1.00) | 0/10 | 0/10 | 1.150 | 1.150 | 1.987 | 否 | 18.630 | 58890 |
| 开阔三机编队 | 噪声 + 空洞/阴影 | 偏离上游 | 30/30 (1.00) | 0/10 | 0/10 | 1.150 | 1.150 | 1.987 | 否 | 18.630 | 58890 |
| 开阔三机编队 | 全开（含延迟） | 偏离上游 | 30/30 (1.00) | 0/10 | 0/10 | 1.150 | 1.150 | 1.987 | 否 | 18.630 | 58890 |
| 开阔三机编队 | 压测 s=0.20、延迟 50 ms | 偏离上游 | 30/30 (1.00) | 0/10 | 0/10 | 1.150 | 1.150 | 1.987 | 否 | 18.630 | 58890 |

### 已冻门

- `crossing-pillars/all`：crossing-pillars：seed 1 arrival_rate 0.000 < 1.000
- `crossing-pillars/all`：crossing-pillars：seed 1 collisions 1 > 0
- `crossing-pillars/all`：crossing-pillars：seed 1 formation_error_p95_m 7.702 > 6.000
- `crossing-pillars/all`：crossing-pillars：seed 1 min_obstacle_clearance_m -0.002 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 2 arrival_rate 0.000 < 1.000
- `crossing-pillars/all`：crossing-pillars：seed 2 collisions 1 > 0
- `crossing-pillars/all`：crossing-pillars：seed 2 min_obstacle_clearance_m -0.006 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 3 arrival_rate 0.000 < 1.000
- `crossing-pillars/all`：crossing-pillars：seed 3 collisions 1 > 0
- `crossing-pillars/all`：crossing-pillars：seed 3 min_obstacle_clearance_m -0.012 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 4 formation_error_p95_m 6.987 > 6.000
- `crossing-pillars/all`：crossing-pillars：seed 4 min_obstacle_clearance_m 0.177 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 6 arrival_rate 0.000 < 1.000
- `crossing-pillars/all`：crossing-pillars：seed 6 collisions 1 > 0
- `crossing-pillars/all`：crossing-pillars：seed 6 formation_error_p95_m 6.416 > 6.000
- `crossing-pillars/all`：crossing-pillars：seed 6 min_obstacle_clearance_m -0.004 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 7 min_obstacle_clearance_m 0.362 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 8 arrival_rate 0.000 < 1.000
- `crossing-pillars/all`：crossing-pillars：seed 8 collisions 1 > 0
- `crossing-pillars/all`：crossing-pillars：seed 8 formation_error_p95_m 7.282 > 6.000
- `crossing-pillars/all`：crossing-pillars：seed 8 min_obstacle_clearance_m -0.002 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 9 min_obstacle_clearance_m 0.279 < 0.400
- `crossing-pillars/all`：crossing-pillars：合计到达率 arrival_rate 0.500 < 1.000
- `crossing-pillars/all`：crossing-pillars：合计 collisions 5 > 0
- `crossing-pillars/holes`：crossing-pillars：seed 0 min_obstacle_clearance_m 0.151 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 1 formation_error_p95_m 6.168 > 6.000
- `crossing-pillars/holes`：crossing-pillars：seed 1 min_obstacle_clearance_m 0.062 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 2 formation_error_p95_m 7.899 > 6.000
- `crossing-pillars/holes`：crossing-pillars：seed 2 min_obstacle_clearance_m 0.228 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 3 arrival_rate 0.000 < 1.000
- `crossing-pillars/holes`：crossing-pillars：seed 3 collisions 1 > 0
- `crossing-pillars/holes`：crossing-pillars：seed 3 formation_error_p95_m 9.369 > 6.000
- `crossing-pillars/holes`：crossing-pillars：seed 3 min_obstacle_clearance_m -0.002 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 4 arrival_rate 0.000 < 1.000
- `crossing-pillars/holes`：crossing-pillars：seed 4 collisions 1 > 0
- `crossing-pillars/holes`：crossing-pillars：seed 4 formation_error_p95_m 7.372 > 6.000
- `crossing-pillars/holes`：crossing-pillars：seed 4 min_obstacle_clearance_m -0.032 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 6 arrival_rate 0.000 < 1.000
- `crossing-pillars/holes`：crossing-pillars：seed 6 collisions 1 > 0
- `crossing-pillars/holes`：crossing-pillars：seed 6 min_obstacle_clearance_m -0.002 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 7 arrival_rate 0.000 < 1.000
- `crossing-pillars/holes`：crossing-pillars：seed 7 collisions 1 > 0
- `crossing-pillars/holes`：crossing-pillars：seed 7 formation_error_p95_m 6.944 > 6.000
- `crossing-pillars/holes`：crossing-pillars：seed 7 min_obstacle_clearance_m -0.000 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 8 formation_error_p95_m 7.316 > 6.000
- `crossing-pillars/holes`：crossing-pillars：seed 8 min_obstacle_clearance_m 0.001 < 0.400
- `crossing-pillars/holes`：crossing-pillars：合计到达率 arrival_rate 0.600 < 1.000
- `crossing-pillars/holes`：crossing-pillars：合计 collisions 4 > 0
- `crossing-pillars/stress`：crossing-pillars：seed 0 arrival_rate 0.000 < 1.000
- `crossing-pillars/stress`：crossing-pillars：seed 0 collisions 1 > 0
- `crossing-pillars/stress`：crossing-pillars：seed 0 formation_error_p95_m 6.081 > 6.000
- `crossing-pillars/stress`：crossing-pillars：seed 0 min_obstacle_clearance_m -0.001 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 2 formation_error_p95_m 7.797 > 6.000
- `crossing-pillars/stress`：crossing-pillars：seed 3 arrival_rate 0.000 < 1.000
- `crossing-pillars/stress`：crossing-pillars：seed 3 collisions 1 > 0
- `crossing-pillars/stress`：crossing-pillars：seed 3 min_obstacle_clearance_m -0.014 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 4 min_obstacle_clearance_m 0.021 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 5 formation_error_p95_m 6.642 > 6.000
- `crossing-pillars/stress`：crossing-pillars：seed 6 min_obstacle_clearance_m 0.389 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 7 min_obstacle_clearance_m 0.356 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 9 min_obstacle_clearance_m 0.322 < 0.400
- `crossing-pillars/stress`：crossing-pillars：合计到达率 arrival_rate 0.800 < 1.000
- `crossing-pillars/stress`：crossing-pillars：合计 collisions 2 > 0
- `six-sparse/all`：crossing-6-forest-sparse：seed 0 arrival_rate 0.000 < 1.000
- `six-sparse/all`：crossing-6-forest-sparse：seed 0 collisions 1 > 0
- `six-sparse/all`：crossing-6-forest-sparse：seed 1 arrival_rate 0.000 < 1.000
- `six-sparse/all`：crossing-6-forest-sparse：seed 1 collisions 1 > 0
- `six-sparse/all`：crossing-6-forest-sparse：seed 2 arrival_rate 0.000 < 1.000
- `six-sparse/all`：crossing-6-forest-sparse：seed 2 collisions 1 > 0
- `six-sparse/all`：crossing-6-forest-sparse：seed 3 arrival_rate 0.000 < 1.000
- `six-sparse/all`：crossing-6-forest-sparse：seed 3 collisions 1 > 0
- `six-sparse/all`：crossing-6-forest-sparse：seed 4 arrival_rate 0.000 < 1.000
- `six-sparse/all`：crossing-6-forest-sparse：seed 4 collisions 1 > 0
- `six-sparse/all`：crossing-6-forest-sparse：seed 5 arrival_rate 0.000 < 1.000
- `six-sparse/all`：crossing-6-forest-sparse：seed 5 collisions 1 > 0
- `six-sparse/all`：crossing-6-forest-sparse：seed 6 arrival_rate 0.000 < 1.000
- `six-sparse/all`：crossing-6-forest-sparse：seed 6 collisions 1 > 0
- `six-sparse/all`：crossing-6-forest-sparse：seed 7 arrival_rate 0.000 < 1.000
- `six-sparse/all`：crossing-6-forest-sparse：seed 7 collisions 1 > 0
- `six-sparse/all`：crossing-6-forest-sparse：seed 8 arrival_rate 0.000 < 1.000
- `six-sparse/all`：crossing-6-forest-sparse：seed 8 collisions 1 > 0
- `six-sparse/all`：crossing-6-forest-sparse：seed 9 arrival_rate 0.000 < 1.000
- `six-sparse/all`：crossing-6-forest-sparse：seed 9 collisions 1 > 0
- `six-sparse/all`：crossing-6-forest-sparse：合计到达率 arrival_rate 0.000 < 1.000
- `six-sparse/all`：crossing-6-forest-sparse：合计 collisions 10 > 0
- `six-sparse/holes`：crossing-6-forest-sparse：seed 0 arrival_rate 0.667 < 1.000
- `six-sparse/holes`：crossing-6-forest-sparse：seed 1 arrival_rate 0.000 < 1.000
- `six-sparse/holes`：crossing-6-forest-sparse：seed 1 collisions 1 > 0
- `six-sparse/holes`：crossing-6-forest-sparse：seed 2 arrival_rate 0.000 < 1.000
- `six-sparse/holes`：crossing-6-forest-sparse：seed 2 collisions 1 > 0
- `six-sparse/holes`：crossing-6-forest-sparse：seed 3 arrival_rate 0.000 < 1.000
- `six-sparse/holes`：crossing-6-forest-sparse：seed 3 collisions 1 > 0
- `six-sparse/holes`：crossing-6-forest-sparse：seed 4 arrival_rate 0.000 < 1.000
- `six-sparse/holes`：crossing-6-forest-sparse：seed 4 collisions 1 > 0
- `six-sparse/holes`：crossing-6-forest-sparse：seed 5 arrival_rate 0.000 < 1.000
- `six-sparse/holes`：crossing-6-forest-sparse：seed 5 collisions 1 > 0
- `six-sparse/holes`：crossing-6-forest-sparse：seed 6 arrival_rate 0.000 < 1.000
- `six-sparse/holes`：crossing-6-forest-sparse：seed 6 collisions 1 > 0
- `six-sparse/holes`：crossing-6-forest-sparse：seed 7 arrival_rate 0.000 < 1.000
- `six-sparse/holes`：crossing-6-forest-sparse：seed 7 collisions 1 > 0
- `six-sparse/holes`：crossing-6-forest-sparse：seed 8 arrival_rate 0.000 < 1.000
- `six-sparse/holes`：crossing-6-forest-sparse：seed 8 collisions 1 > 0
- `six-sparse/holes`：crossing-6-forest-sparse：seed 9 arrival_rate 0.000 < 1.000
- `six-sparse/holes`：crossing-6-forest-sparse：seed 9 collisions 1 > 0
- `six-sparse/holes`：crossing-6-forest-sparse：合计到达率 arrival_rate 0.067 < 1.000
- `six-sparse/holes`：crossing-6-forest-sparse：合计 collisions 9 > 0
- `six-sparse/stress`：crossing-6-forest-sparse：seed 1 arrival_rate 0.833 < 1.000
- `six-sparse/stress`：crossing-6-forest-sparse：seed 2 arrival_rate 0.667 < 1.000
- `six-sparse/stress`：crossing-6-forest-sparse：seed 2 collisions 1 > 0
- `six-sparse/stress`：crossing-6-forest-sparse：seed 3 arrival_rate 0.833 < 1.000
- `six-sparse/stress`：crossing-6-forest-sparse：合计到达率 arrival_rate 0.933 < 1.000
- `six-sparse/stress`：crossing-6-forest-sparse：合计 collisions 1 > 0

## 结论

下面两套到达率都只含四张走 YOPO 的卡：g2 柱子、密度 0.14 单机、crossing-pillars、六机林带 sparse。开阔三机编队不读深度，不进合计。「按飞机」把到达飞机数除以飞机×种子，六机卡权重更大。「按卡」是四张卡到达率的简单平均，每张卡一票。

按飞机：基线 0.991，只开噪声 1.000，噪声+空洞/阴影 0.291，全开 0.227，压测 0.818。

按卡：基线 0.975，只开噪声 1.000，噪声+空洞/阴影 0.417，全开 0.375，压测 0.683。

按飞机分段：只加视差噪声：0.991 → 1.000（变化 +0.009）；再加空洞/阴影：1.000 → 0.291（变化 -0.709）；再加约 21 ms 延迟：0.291 → 0.227（变化 -0.064）。

三段里按飞机到达率下降最多的是「再加空洞/阴影」，1.000 → 0.291（变化 -0.709）。

按卡分段：只加视差噪声：0.975 → 1.000（变化 +0.025）；再加空洞/阴影：1.000 → 0.417（变化 -0.583）；再加约 21 ms 延迟：0.417 → 0.375（变化 -0.042）。按卡到达率下降最多的是「再加空洞/阴影」，1.000 → 0.417（变化 -0.583）。

全开相对基线，按飞机到达率 0.991 → 0.227（变化 -0.764）。这是上游权重在这些仿真相机和这套 D435 模型下的掉幅，不是 848×480 实机的直接读数。2 m 处的米制噪声见上表：160×96、90° 的卡大约 80 mm，g2 的 320×240、73° 大约 30 mm；原生 D435 在 fx≈426 时是 15 mm。

压测（s=0.20，延迟固定 50 ms，丢帧率仍是 0.5%）相对全开：0.227 → 0.818（变化 +0.591）。

压测不是默认全开再加一档噪声：s、延迟和每回合视差偏置都换了。密度 0.14 全开 0/10 (0.00)，压测 0/10 (0.00)。六机林带 sparse 全开 0/60 (0.00)，压测 56/60 (0.93)。合计回升不能读成噪声越大越安全。

开阔三机编队（formation-hold，不读深度）：基线（关） 30/30 (1.00)；只开噪声 30/30 (1.00)；噪声 + 空洞/阴影 30/30 (1.00)；全开（含延迟） 30/30 (1.00)；压测 s=0.20、延迟 50 ms 30/30 (1.00)。

这张编队卡基线和全开的影子盾会拒 tick 都是 58890，到达、净空、机间、平均到达时间也相同。编队规划不读深度。开阔场景上，这次加噪没有改到影子盾记下的拒绝次数。

逐粒记录在 [e-d435-depth-noise-runs.json](e-d435-depth-noise-runs.json)。

## 复现

```
python3 scripts/e-d435-depth-noise.py --jobs 4
```

中断后原样再跑会读 `out/e-d435-depth-noise/partial/`。`--fresh` 忽略已落盘的粒。
