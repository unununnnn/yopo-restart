# 票 E-D435b：原生 D435 焦距下的深度噪声

只报数。report-only。``d435_depth`` 默认关，``profile`` 缺省 ``scene``。场景卡、``scenes/suite.txt``、已冻门都没改。打开即偏离上游：上游权重在真值深度上训练（右目关闭）。``profile=native`` 按原生 D435（848×480，fx≈426 px，B=0.05 m，s=0.08）在视差域加噪，再重采样到场景相机分辨率。无效像素默认仍是 NaN，网络侧走上游 NS inpaint。真值填是对照：空洞像素写成这一帧的真值，网络不再把这些像素当洞。假近点两种填法都留着。飞点概率 0.3、掠射角阈值 75°、相关长度 3 px、丢帧率 0.5%，以及约 21 ms 里 Orin 额外的约 5 ms，置信度低。影子盾只记录 would_reject，不改批准的计划。墙钟不进结论。

依赖票 E-D435（PR #122，分支 `yopo-restart-bot/e-d435-depth-noise`）。本分支先合并了 `origin/main`。
E-D435 的原表在 [e-d435-depth-noise.md](e-d435-depth-noise.md)。

## 模型和开关

- `task.d435_depth` 缺省关。关时不建噪声状态，高斯随机流与加这只开关之前相同。
- `profile` 缺省 `scene`：视差噪声和按像素计的空洞用场景相机的 fx。不写 `profile` 时，深度与 E-D435 的 scene 路径逐元素相同。
- `profile=native`：名义传感器 848×480、fx=426、B=0.050 m。同一视场按这个像素角抬成细网格，调用原来的视差噪声和空洞，再把视差平均重采样回场景分辨率。
- 重采样时，格子里有效原生像素不足一半，输出仍是空洞。块状洞、1% 随机洞、相关长度 3 px 都在细网格的像素上，缩回来变小。
- 遮挡阴影宽度约 `fx · B · (1/z_near − 1/z_far)` 像素，角宽是 `B · (1/z_near − 1/z_far)`，跟把 fx 换成场景焦距再在粗网格上算，是同一个角。
- 无效像素默认 `nan`。`network_depth` 对 NaN 和小于 0.04 m 的像素做上游 NS radius-1 inpaint。
- `invalid=gt`：空洞像素改成这一帧的真值，再交给策略。这些像素是有限深度，NS 不会把它们当洞。假近点（约 0.3%）不在这次填补里。
- 名字仍登记在 `yopo_l0/experiment.py` 的 `RESEARCH_SWITCHES`。场景卡没有把 `d435_depth` 写成 JSON true。

## 平壁 σ 和合成图上的空洞比例

σ 是 2 m 平壁、关掉飞点和空洞、视差偏置为 0、种子 0，裁掉四周 8 像素后的标准差。公式 σ 是 `Z² · s / (fx · B)`，native 公式用 fx=426，还没有做重采样平均。空洞比例是半幅 1.5 m、其余 6 m 的合成图，种子固定。三张 160×96、90° 的卡共用第二行的相机。

| 相机 | 分辨率 | fx px | scene 公式 σ mm | scene 实测 σ mm | native 公式 σ mm | native 重采样后 σ mm | scene 空洞比例 | scene 只几何 | native 空洞比例 | native 只几何 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| g2 柱子单机 | 320×240 | 216.2 | 29.6 | 29.9 | 15.0 | 14.0 | 0.341 | 0.028 | 0.136 | 0.022 |
| 密度 0.14 单机 | 160×96 | 80.0 | 80.0 | 82.5 | 15.0 | 11.5 | 0.340 | 0.029 | 0.070 | 0.019 |

## native 飞行结果

seeds 0–9。到达是到达飞机 / 飞机数。障碍碰撞、机间碰撞是发生了碰撞的运行次数 / 种子数（仿真在第一次碰撞停下，一次运行最多记 1）。最小障碍距离是种子里最小的机体中心到障碍表面的距离。最小净空是该距离减去机体半径后再取种子里最差的一次。这四张卡的机体半径是 0.35 m。中位净空是各种子最小净空的中位数。越门只报告，本票不改已冻门。

| 卡 | 臂 | 偏离上游 | 到达 | 障碍碰撞 | 机间碰撞 | 最小障碍距离 m | 最小净空 m | 中位净空 m | 越门 |
|---|---|---|---:|---:|---:|---:|---:|---:|---|
| g2 柱子单机 | 只开噪声 | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.796 | 0.446 | 0.472 | 否 |
| g2 柱子单机 | 噪声 + 空洞/阴影 | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.739 | 0.389 | 0.467 | 否 |
| g2 柱子单机 | 全开（含延迟） | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.772 | 0.422 | 0.534 | 否 |
| g2 柱子单机 | 压测 s=0.20、延迟 50 ms | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.758 | 0.408 | 0.503 | 否 |
| g2 柱子单机 | 噪声 + 空洞，空洞填真值 | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.782 | 0.432 | 0.454 | 否 |
| 密度 0.14 单机 | 只开噪声 | 偏离上游 | 9/10 (0.90) | 1/10 | 0/10 | 0.344 | -0.006 | 0.147 | 未冻结 |
| 密度 0.14 单机 | 噪声 + 空洞/阴影 | 偏离上游 | 8/10 (0.80) | 2/10 | 0/10 | 0.347 | -0.003 | 0.162 | 未冻结 |
| 密度 0.14 单机 | 全开（含延迟） | 偏离上游 | 7/10 (0.70) | 3/10 | 0/10 | 0.345 | -0.005 | 0.087 | 未冻结 |
| 密度 0.14 单机 | 压测 s=0.20、延迟 50 ms | 偏离上游 | 5/10 (0.50) | 5/10 | 0/10 | 0.338 | -0.012 | 0.035 | 未冻结 |
| 密度 0.14 单机 | 噪声 + 空洞，空洞填真值 | 偏离上游 | 7/10 (0.70) | 3/10 | 0/10 | 0.341 | -0.009 | 0.016 | 未冻结 |
| crossing-pillars 三机 | 只开噪声 | 偏离上游 | 30/30 (1.00) | 0/10 | 0/10 | 1.342 | 0.992 | 1.008 | 否 |
| crossing-pillars 三机 | 噪声 + 空洞/阴影 | 偏离上游 | 27/30 (0.90) | 1/10 | 0/10 | 0.344 | -0.006 | 0.318 | 是（14） |
| crossing-pillars 三机 | 全开（含延迟） | 偏离上游 | 30/30 (1.00) | 0/10 | 0/10 | 0.409 | 0.059 | 0.349 | 是（15） |
| crossing-pillars 三机 | 压测 s=0.20、延迟 50 ms | 偏离上游 | 18/30 (0.60) | 4/10 | 0/10 | 0.330 | -0.020 | 0.082 | 是（24） |
| crossing-pillars 三机 | 噪声 + 空洞，空洞填真值 | 偏离上游 | 30/30 (1.00) | 0/10 | 0/10 | 1.074 | 0.724 | 0.880 | 否 |
| 六机林带 sparse | 只开噪声 | 偏离上游 | 60/60 (1.00) | 0/10 | 0/10 | 0.495 | 0.145 | 0.244 | 否 |
| 六机林带 sparse | 噪声 + 空洞/阴影 | 偏离上游 | 60/60 (1.00) | 0/10 | 0/10 | 0.527 | 0.177 | 0.343 | 否 |
| 六机林带 sparse | 全开（含延迟） | 偏离上游 | 60/60 (1.00) | 0/10 | 0/10 | 0.502 | 0.152 | 0.222 | 否 |
| 六机林带 sparse | 压测 s=0.20、延迟 50 ms | 偏离上游 | 60/60 (1.00) | 0/10 | 0/10 | 0.484 | 0.134 | 0.331 | 否 |
| 六机林带 sparse | 噪声 + 空洞，空洞填真值 | 偏离上游 | 60/60 (1.00) | 0/10 | 0/10 | 0.495 | 0.145 | 0.225 | 否 |

## 与 E-D435 原表对照

E-D435 列来自 [e-d435-depth-noise-runs.json](e-d435-depth-noise-runs.json)，profile 是 scene。native 列是本票。基线没有 native 臂。最小障碍距离是各种子 `min_obstacle_distance_m` 的最小值。

| 卡 | 臂 | E-D435 到达 | native 到达 | E-D435 障碍碰撞 | native 障碍碰撞 | E-D435 最小障碍距离 m | native 最小障碍距离 m | E-D435 最小净空 m | native 最小净空 m |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| g2 柱子单机 | E-D435 基线（关） | 10/10 (1.00) | — | 0/10 | — | 0.828 | — | 0.478 | — |
| g2 柱子单机 | 只开噪声 | 10/10 (1.00) | 10/10 (1.00) | 0/10 | 0/10 | 0.735 | 0.796 | 0.385 | 0.446 |
| g2 柱子单机 | 噪声 + 空洞/阴影 | 10/10 (1.00) | 10/10 (1.00) | 0/10 | 0/10 | 0.707 | 0.739 | 0.357 | 0.389 |
| g2 柱子单机 | 全开（含延迟） | 10/10 (1.00) | 10/10 (1.00) | 0/10 | 0/10 | 0.749 | 0.772 | 0.399 | 0.422 |
| g2 柱子单机 | 压测 s=0.20、延迟 50 ms | 10/10 (1.00) | 10/10 (1.00) | 0/10 | 0/10 | 0.534 | 0.758 | 0.184 | 0.408 |
| 密度 0.14 单机 | E-D435 基线（关） | 9/10 (0.90) | — | 1/10 | — | 0.349 | — | -0.001 | — |
| 密度 0.14 单机 | 只开噪声 | 10/10 (1.00) | 9/10 (0.90) | 0/10 | 1/10 | 0.417 | 0.344 | 0.067 | -0.006 |
| 密度 0.14 单机 | 噪声 + 空洞/阴影 | 0/10 (0.00) | 8/10 (0.80) | 10/10 | 2/10 | 0.336 | 0.347 | -0.014 | -0.003 |
| 密度 0.14 单机 | 全开（含延迟） | 0/10 (0.00) | 7/10 (0.70) | 10/10 | 3/10 | 0.334 | 0.345 | -0.016 | -0.005 |
| 密度 0.14 单机 | 压测 s=0.20、延迟 50 ms | 0/10 (0.00) | 5/10 (0.50) | 10/10 | 5/10 | 0.328 | 0.338 | -0.022 | -0.012 |
| crossing-pillars 三机 | E-D435 基线（关） | 30/30 (1.00) | — | 0/10 | — | 0.878 | — | 0.528 | — |
| crossing-pillars 三机 | 只开噪声 | 30/30 (1.00) | 30/30 (1.00) | 0/10 | 0/10 | 1.091 | 1.342 | 0.741 | 0.992 |
| crossing-pillars 三机 | 噪声 + 空洞/阴影 | 18/30 (0.60) | 27/30 (0.90) | 4/10 | 1/10 | 0.318 | 0.344 | -0.032 | -0.006 |
| crossing-pillars 三机 | 全开（含延迟） | 15/30 (0.50) | 30/30 (1.00) | 5/10 | 0/10 | 0.338 | 0.409 | -0.012 | 0.059 |
| crossing-pillars 三机 | 压测 s=0.20、延迟 50 ms | 24/30 (0.80) | 18/30 (0.60) | 2/10 | 4/10 | 0.336 | 0.330 | -0.014 | -0.020 |
| 六机林带 sparse | E-D435 基线（关） | 60/60 (1.00) | — | 0/10 | — | 0.508 | — | 0.158 | — |
| 六机林带 sparse | 只开噪声 | 60/60 (1.00) | 60/60 (1.00) | 0/10 | 0/10 | 0.500 | 0.495 | 0.150 | 0.145 |
| 六机林带 sparse | 噪声 + 空洞/阴影 | 4/60 (0.07) | 60/60 (1.00) | 9/10 | 0/10 | 0.346 | 0.527 | -0.004 | 0.177 |
| 六机林带 sparse | 全开（含延迟） | 0/60 (0.00) | 60/60 (1.00) | 10/10 | 0/10 | 0.347 | 0.502 | -0.003 | 0.152 |
| 六机林带 sparse | 压测 s=0.20、延迟 50 ms | 56/60 (0.93) | 60/60 (1.00) | 1/10 | 0/10 | 0.349 | 0.484 | -0.001 | 0.134 |

## 空洞填真值，对照 NS inpaint

scene 两行是 E-D435 的焦距，本分支重跑。native 两行是原生像素角。NS 是 `invalid=nan`：空洞留给 `network_depth` 做 NS inpaint。真值填是 `invalid=gt`。噪声、遮挡、随机洞、块状洞、假近点两边相同。真值填只替换深度 ≤ 0 的像素。

| 卡 | 臂 | 偏离上游 | 到达 | 障碍碰撞 | 机间碰撞 | 最小障碍距离 m | 最小净空 m | 中位净空 m | 越门 |
|---|---|---|---:|---:|---:|---:|---:|---:|---|
| g2 柱子单机 | scene 噪声 + 空洞/阴影（NS inpaint） | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.707 | 0.357 | 0.456 | 否 |
| g2 柱子单机 | scene 噪声 + 空洞，空洞填真值 | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.747 | 0.397 | 0.461 | 否 |
| g2 柱子单机 | 噪声 + 空洞/阴影 | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.739 | 0.389 | 0.467 | 否 |
| g2 柱子单机 | 噪声 + 空洞，空洞填真值 | 偏离上游 | 10/10 (1.00) | 0/10 | 0/10 | 0.782 | 0.432 | 0.454 | 否 |
| 密度 0.14 单机 | scene 噪声 + 空洞/阴影（NS inpaint） | 偏离上游 | 0/10 (0.00) | 10/10 | 0/10 | 0.336 | -0.014 | -0.002 | 未冻结 |
| 密度 0.14 单机 | scene 噪声 + 空洞，空洞填真值 | 偏离上游 | 0/10 (0.00) | 10/10 | 0/10 | 0.337 | -0.013 | -0.002 | 未冻结 |
| 密度 0.14 单机 | 噪声 + 空洞/阴影 | 偏离上游 | 8/10 (0.80) | 2/10 | 0/10 | 0.347 | -0.003 | 0.162 | 未冻结 |
| 密度 0.14 单机 | 噪声 + 空洞，空洞填真值 | 偏离上游 | 7/10 (0.70) | 3/10 | 0/10 | 0.341 | -0.009 | 0.016 | 未冻结 |
| crossing-pillars 三机 | scene 噪声 + 空洞/阴影（NS inpaint） | 偏离上游 | 18/30 (0.60) | 4/10 | 0/10 | 0.318 | -0.032 | 0.032 | 是（24） |
| crossing-pillars 三机 | scene 噪声 + 空洞，空洞填真值 | 偏离上游 | 30/30 (1.00) | 0/10 | 0/10 | 0.606 | 0.256 | 0.657 | 是（2） |
| crossing-pillars 三机 | 噪声 + 空洞/阴影 | 偏离上游 | 27/30 (0.90) | 1/10 | 0/10 | 0.344 | -0.006 | 0.318 | 是（14） |
| crossing-pillars 三机 | 噪声 + 空洞，空洞填真值 | 偏离上游 | 30/30 (1.00) | 0/10 | 0/10 | 1.074 | 0.724 | 0.880 | 否 |
| 六机林带 sparse | scene 噪声 + 空洞/阴影（NS inpaint） | 偏离上游 | 4/60 (0.07) | 9/10 | 0/10 | 0.346 | -0.004 | -0.001 | 是（21） |
| 六机林带 sparse | scene 噪声 + 空洞，空洞填真值 | 偏离上游 | 56/60 (0.93) | 0/10 | 0/10 | 0.459 | 0.109 | 0.142 | 是（5） |
| 六机林带 sparse | 噪声 + 空洞/阴影 | 偏离上游 | 60/60 (1.00) | 0/10 | 0/10 | 0.527 | 0.177 | 0.343 | 否 |
| 六机林带 sparse | 噪声 + 空洞，空洞填真值 | 偏离上游 | 60/60 (1.00) | 0/10 | 0/10 | 0.495 | 0.145 | 0.225 | 否 |

### 已冻门

- `crossing-pillars/all`：crossing-pillars：seed 0 formation_error_p95_m 6.079 > 6.000
- `crossing-pillars/all`：crossing-pillars：seed 0 min_obstacle_clearance_m 0.293 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 1 formation_error_p95_m 8.437 > 6.000
- `crossing-pillars/all`：crossing-pillars：seed 1 min_obstacle_clearance_m 0.253 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 2 min_obstacle_clearance_m 0.377 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 3 formation_error_p95_m 6.299 > 6.000
- `crossing-pillars/all`：crossing-pillars：seed 3 min_obstacle_clearance_m 0.343 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 4 min_obstacle_clearance_m 0.354 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 5 min_inter_drone_distance_m 1.606 < 2.000
- `crossing-pillars/all`：crossing-pillars：seed 5 min_obstacle_clearance_m 0.059 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 6 min_obstacle_clearance_m 0.234 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 7 formation_error_p95_m 7.241 > 6.000
- `crossing-pillars/all`：crossing-pillars：seed 7 min_obstacle_clearance_m 0.370 < 0.400
- `crossing-pillars/all`：crossing-pillars：seed 8 formation_error_p95_m 6.252 > 6.000
- `crossing-pillars/all`：crossing-pillars：seed 8 min_obstacle_clearance_m 0.388 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 0 formation_error_p95_m 6.621 > 6.000
- `crossing-pillars/holes`：crossing-pillars：seed 0 min_obstacle_clearance_m 0.221 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 1 min_obstacle_clearance_m 0.353 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 2 formation_error_p95_m 6.457 > 6.000
- `crossing-pillars/holes`：crossing-pillars：seed 3 arrival_rate 0.000 < 1.000
- `crossing-pillars/holes`：crossing-pillars：seed 3 collisions 1 > 0
- `crossing-pillars/holes`：crossing-pillars：seed 3 formation_error_p95_m 6.336 > 6.000
- `crossing-pillars/holes`：crossing-pillars：seed 3 min_obstacle_clearance_m -0.006 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 4 min_obstacle_clearance_m 0.283 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 6 formation_error_p95_m 6.319 > 6.000
- `crossing-pillars/holes`：crossing-pillars：seed 6 min_obstacle_clearance_m 0.109 < 0.400
- `crossing-pillars/holes`：crossing-pillars：seed 9 min_obstacle_clearance_m 0.241 < 0.400
- `crossing-pillars/holes`：crossing-pillars：合计到达率 arrival_rate 0.900 < 1.000
- `crossing-pillars/holes`：crossing-pillars：合计 collisions 1 > 0
- `crossing-pillars/scene_holes`：crossing-pillars：seed 0 min_obstacle_clearance_m 0.151 < 0.400
- `crossing-pillars/scene_holes`：crossing-pillars：seed 1 formation_error_p95_m 6.168 > 6.000
- `crossing-pillars/scene_holes`：crossing-pillars：seed 1 min_obstacle_clearance_m 0.062 < 0.400
- `crossing-pillars/scene_holes`：crossing-pillars：seed 2 formation_error_p95_m 7.899 > 6.000
- `crossing-pillars/scene_holes`：crossing-pillars：seed 2 min_obstacle_clearance_m 0.228 < 0.400
- `crossing-pillars/scene_holes`：crossing-pillars：seed 3 arrival_rate 0.000 < 1.000
- `crossing-pillars/scene_holes`：crossing-pillars：seed 3 collisions 1 > 0
- `crossing-pillars/scene_holes`：crossing-pillars：seed 3 formation_error_p95_m 9.369 > 6.000
- `crossing-pillars/scene_holes`：crossing-pillars：seed 3 min_obstacle_clearance_m -0.002 < 0.400
- `crossing-pillars/scene_holes`：crossing-pillars：seed 4 arrival_rate 0.000 < 1.000
- `crossing-pillars/scene_holes`：crossing-pillars：seed 4 collisions 1 > 0
- `crossing-pillars/scene_holes`：crossing-pillars：seed 4 formation_error_p95_m 7.372 > 6.000
- `crossing-pillars/scene_holes`：crossing-pillars：seed 4 min_obstacle_clearance_m -0.032 < 0.400
- `crossing-pillars/scene_holes`：crossing-pillars：seed 6 arrival_rate 0.000 < 1.000
- `crossing-pillars/scene_holes`：crossing-pillars：seed 6 collisions 1 > 0
- `crossing-pillars/scene_holes`：crossing-pillars：seed 6 min_obstacle_clearance_m -0.002 < 0.400
- `crossing-pillars/scene_holes`：crossing-pillars：seed 7 arrival_rate 0.000 < 1.000
- `crossing-pillars/scene_holes`：crossing-pillars：seed 7 collisions 1 > 0
- `crossing-pillars/scene_holes`：crossing-pillars：seed 7 formation_error_p95_m 6.944 > 6.000
- `crossing-pillars/scene_holes`：crossing-pillars：seed 7 min_obstacle_clearance_m -0.000 < 0.400
- `crossing-pillars/scene_holes`：crossing-pillars：seed 8 formation_error_p95_m 7.316 > 6.000
- `crossing-pillars/scene_holes`：crossing-pillars：seed 8 min_obstacle_clearance_m 0.001 < 0.400
- `crossing-pillars/scene_holes`：crossing-pillars：合计到达率 arrival_rate 0.600 < 1.000
- `crossing-pillars/scene_holes`：crossing-pillars：合计 collisions 4 > 0
- `crossing-pillars/scene_holes_gt`：crossing-pillars：seed 1 min_obstacle_clearance_m 0.375 < 0.400
- `crossing-pillars/scene_holes_gt`：crossing-pillars：seed 3 min_obstacle_clearance_m 0.256 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 0 min_obstacle_clearance_m 0.005 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 1 min_obstacle_clearance_m 0.228 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 2 arrival_rate 0.000 < 1.000
- `crossing-pillars/stress`：crossing-pillars：seed 2 collisions 1 > 0
- `crossing-pillars/stress`：crossing-pillars：seed 2 formation_error_p95_m 6.402 > 6.000
- `crossing-pillars/stress`：crossing-pillars：seed 2 min_obstacle_clearance_m -0.008 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 3 arrival_rate 0.000 < 1.000
- `crossing-pillars/stress`：crossing-pillars：seed 3 collisions 1 > 0
- `crossing-pillars/stress`：crossing-pillars：seed 3 min_obstacle_clearance_m -0.020 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 4 arrival_rate 0.000 < 1.000
- `crossing-pillars/stress`：crossing-pillars：seed 4 collisions 1 > 0
- `crossing-pillars/stress`：crossing-pillars：seed 4 formation_error_p95_m 6.244 > 6.000
- `crossing-pillars/stress`：crossing-pillars：seed 4 min_obstacle_clearance_m -0.006 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 5 min_obstacle_clearance_m 0.304 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 6 min_obstacle_clearance_m 0.385 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 7 arrival_rate 0.000 < 1.000
- `crossing-pillars/stress`：crossing-pillars：seed 7 collisions 1 > 0
- `crossing-pillars/stress`：crossing-pillars：seed 7 formation_error_p95_m 6.183 > 6.000
- `crossing-pillars/stress`：crossing-pillars：seed 7 min_obstacle_clearance_m -0.005 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 8 min_obstacle_clearance_m 0.245 < 0.400
- `crossing-pillars/stress`：crossing-pillars：seed 9 formation_error_p95_m 6.731 > 6.000
- `crossing-pillars/stress`：crossing-pillars：seed 9 min_obstacle_clearance_m 0.158 < 0.400
- `crossing-pillars/stress`：crossing-pillars：合计到达率 arrival_rate 0.600 < 1.000
- `crossing-pillars/stress`：crossing-pillars：合计 collisions 4 > 0
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 0 arrival_rate 0.667 < 1.000
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 1 arrival_rate 0.000 < 1.000
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 1 collisions 1 > 0
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 2 arrival_rate 0.000 < 1.000
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 2 collisions 1 > 0
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 3 arrival_rate 0.000 < 1.000
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 3 collisions 1 > 0
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 4 arrival_rate 0.000 < 1.000
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 4 collisions 1 > 0
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 5 arrival_rate 0.000 < 1.000
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 5 collisions 1 > 0
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 6 arrival_rate 0.000 < 1.000
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 6 collisions 1 > 0
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 7 arrival_rate 0.000 < 1.000
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 7 collisions 1 > 0
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 8 arrival_rate 0.000 < 1.000
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 8 collisions 1 > 0
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 9 arrival_rate 0.000 < 1.000
- `six-sparse/scene_holes`：crossing-6-forest-sparse：seed 9 collisions 1 > 0
- `six-sparse/scene_holes`：crossing-6-forest-sparse：合计到达率 arrival_rate 0.067 < 1.000
- `six-sparse/scene_holes`：crossing-6-forest-sparse：合计 collisions 9 > 0
- `six-sparse/scene_holes_gt`：crossing-6-forest-sparse：seed 1 arrival_rate 0.833 < 1.000
- `six-sparse/scene_holes_gt`：crossing-6-forest-sparse：seed 2 arrival_rate 0.833 < 1.000
- `six-sparse/scene_holes_gt`：crossing-6-forest-sparse：seed 3 arrival_rate 0.833 < 1.000
- `six-sparse/scene_holes_gt`：crossing-6-forest-sparse：seed 5 arrival_rate 0.833 < 1.000
- `six-sparse/scene_holes_gt`：crossing-6-forest-sparse：合计到达率 arrival_rate 0.933 < 1.000

E-D435 原表里空洞、全开、压测的已冻门失败留在 [e-d435-depth-noise.md](e-d435-depth-noise.md)。按同一套门重算，原表四张卡全部臂里有 6 组越门。这里不改那些门。

## 结论

下面的合计只含四张走 YOPO 的卡：g2 柱子、密度 0.14 单机、crossing-pillars、六机林带 sparse。「按飞机」把到达飞机数除以飞机×种子，六机卡权重大。「按卡」是四张卡到达率的简单平均。开阔三机编队不读深度，E-D435 里五条臂相同，本票不重跑。

原生传感器名义画幅 848×480，fx=426，基线 0.050 m，s=0.08。2 m 处公式 σ 是 15.0 mm。场景相机视场用同一像素角铺开，最近邻抬到这个网格，视差加噪之后按视差平均缩回场景分辨率；一个格子里有效原生像素不足一半，这个格子仍是空洞。

平壁 2 m、关掉飞点和空洞，种子 0：g2 柱子单机（320×240，fx=216.2）scene 实测 29.9 mm，native 重采样后 14.0 mm；密度 0.14 单机（160×96，fx=80.0）scene 实测 82.5 mm，native 重采样后 11.5 mm。

同一张合成图（半幅 1.5 m 柱、其余 6 m 墙），无效像素比例：g2 柱子单机 scene 全空洞 0.341、只留几何 0.028，native 全空洞 0.136、只留几何 0.022；密度 0.14 单机 scene 全空洞 0.340、只留几何 0.029，native 全空洞 0.070、只留几何 0.019。只留几何是关掉随机洞、块状洞和假近点。遮挡阴影和左侧无效带的角宽由基线决定，缩回场景分辨率之后和 scene 同量级。块状洞半径按像素计，scene 在 160 宽上是大洞，native 先在 fx≈426 的网格上挖，缩回来只占很少的格子。

E-D435 原表按飞机：基线 0.991，只开噪声 1.000，噪声+空洞/阴影 0.291，全开 0.227，压测 0.818。

E-D435 原表按卡：基线 0.975，只开噪声 1.000，噪声+空洞/阴影 0.417，全开 0.375，压测 0.683。

native 按飞机：只开噪声 0.991，噪声+空洞/阴影 0.955，全开 0.973，压测 0.845。

native 按卡：只开噪声 0.975，噪声+空洞/阴影 0.925，全开 0.925，压测 0.775。

按飞机，只开噪声：E-D435 1.000，native 0.991（变化 -0.009）。噪声+空洞/阴影：E-D435 0.291，native 0.955（变化 +0.664）。

native 分段：在只开噪声上再加空洞/阴影：0.991 → 0.955（变化 -0.036）；再加约 21 ms 延迟：0.955 → 0.973（变化 +0.018）。

native 全开相对 E-D435 基线，按飞机 0.991 → 0.973（变化 -0.018）。这仍是上游权重在仿真里的读数，传感器模型换成了原生 D435 的像素角。

native 压测（s=0.20，延迟固定 50 ms，丢帧率仍是 0.5%）相对 native 全开：0.973 → 0.845（变化 -0.127）。压测换了 s、延迟和每回合视差偏置，不是默认全开再加一档。

scene 合计（E-D435 那套焦距，本分支重跑）：NS inpaint 0.291，空洞填真值 0.873（变化 +0.582），只开噪声 1.000。真值填比 NS 高，离只开噪声还有距离。NS 填补是一部分；两种填法都留下的假近点也还在图里。

native 合计：NS inpaint 0.955，空洞填真值 0.973（变化 +0.018），只开噪声 0.991。这一档空洞没有把到达率打下去。

scene 按卡：g2 柱子单机 NS 10/10 (1.00)、障碍碰撞 0/10，真值填 10/10 (1.00)、障碍碰撞 0/10；密度 0.14 单机 NS 0/10 (0.00)、障碍碰撞 10/10，真值填 0/10 (0.00)、障碍碰撞 10/10；crossing-pillars 三机 NS 18/30 (0.60)、障碍碰撞 4/10，真值填 30/30 (1.00)、障碍碰撞 0/10；六机林带 sparse NS 4/60 (0.07)、障碍碰撞 9/10，真值填 56/60 (0.93)、障碍碰撞 0/10。

密度 0.14 两种填法都是 0/10 (0.00)，障碍碰撞都是 10/10。这张卡的掉点不来自 NS 和真值这两种填法的差别。两种填法都留下假近点，本票没有再把假近点单独关掉。crossing-pillars 从 18/30 (0.60)、障碍碰撞 4/10 回到 30/30 (1.00)、障碍碰撞 0/10。六机林带从 4/60 (0.07)、障碍碰撞 9/10 回到 56/60 (0.93)、障碍碰撞 0/10。没到达的 4 架没有记成障碍碰撞。crossing 和六机上，E-D435 的空洞掉点主要是网络侧 NS 填进空洞的深度。

native 按卡：g2 柱子单机 NS 10/10 (1.00)，真值填 10/10 (1.00)；密度 0.14 单机 NS 8/10 (0.80)，真值填 7/10 (0.70)；crossing-pillars 三机 NS 27/30 (0.90)，真值填 30/30 (1.00)；六机林带 sparse NS 60/60 (1.00)，真值填 60/60 (1.00)。

本分支重跑的 scene 空洞臂，四张卡的到达飞机数、障碍碰撞次数和最小净空与 E-D435 原表一致（净空差小于 0.001 m）。scene 路径在合并 origin/main 之后没有改飞行结果。真值填对照和原表空洞臂是同一条噪声，只是空洞像素的填法不同。

空洞/阴影这一档，按卡从 E-D435 到 native：g2 柱子单机 10/10 (1.00) → 10/10 (1.00)；密度 0.14 单机 0/10 (0.00) → 8/10 (0.80)；crossing-pillars 三机 18/30 (0.60) → 27/30 (0.90)；六机林带 sparse 4/60 (0.07) → 60/60 (1.00)。

逐粒记录在 [e-d435b-native-runs.json](e-d435b-native-runs.json)。

## 复现

```
python3 scripts/e-d435b-native.py --jobs 4
```

中断后原样再跑会读 `out/e-d435b-native/partial/`。`--fresh` 忽略已落盘的粒。
