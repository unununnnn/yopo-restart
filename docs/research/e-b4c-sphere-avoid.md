# E-B4c：后段 0.70 m 欧氏碰撞球

只报数。默认关。打开即偏离上游 `d5e1e60`。不冻结，不进 `scenes/suite.txt`，不设 gates，不改已有椭球开关的语义。

E-B3 收口（[e-b3-closeout.md](e-b3-closeout.md)）写过：若后段还要做机间避碰，另起新线，直接对准 0.70 m 欧氏碰撞球，不改已有椭球开关。本票就是这条新线。`swarm_penalty` 和 `swarm_segment_penalty` 仍默认关，代价和自由集在它们自己的开关下与加这项之前逐位相同。

数字来自 `scripts/e-b4c-sphere-avoid.py`，代码 `d97f755bad2d60234b8142f5bd338eeba5fcebc9`。逐粒记录在 [e-b4c-sphere-avoid-runs.json](e-b4c-sphere-avoid-runs.json)。

## 开关

写在 `policy_params.backend`，也可直接构造 `BackendConfig`。`peer_sphere_enabled` 缺省、false、字符串都是关。后段 `enabled` 已经是 JSON true 时，环境变量 `YOPO_PEER_SPHERE=1`（以及 `true` / `yes` / `on`）也能打开。半径和权重可用 `YOPO_PEER_SPHERE_RADIUS_M`、`YOPO_PEER_SPHERE_WEIGHT` 盖过。

```json
"policy_params": {
  "backend": {
    "enabled": true,
    "peer_sphere_enabled": true,
    "peer_sphere_radius_m": 0.85,
    "peer_sphere_weight": 8.0
  }
}
```

`0.85 m = 0.70 m 碰撞球 + 0.15 m 裕度`。0.70 m 是 L0 机间碰撞线（机体半径 0.35 m 的两倍）。权重 8 比障碍走廊的平方铰链（求解器里 λ_c = 40）小，末端三个控制点仍钉死，平滑项权重不动。

铰链只在时间对齐的 3D 欧氏距离 `d < R` 时亮。令 `s = R - d`，`b = R - 0.70`（默认 0.15 m）：

- `d >= R`：0。球面处函数值、一阶导、二阶导都是 0。
- `0 < s <= b`（默认 0.70–0.85 m）：`w · s³ / b`，三次过渡，避免刚进裕度就猛推。
- `s > b`（已经进了 0.70 m 球）：`w · (s² + b·s - b²)`，与三次段在接点上一阶导连续。

采样打在除最后一跨之外的段内点上（每段 6 个箱中点，梯度按均匀三次基函数送回控制点）。最后一跨留给平滑项。没有时间重叠的邻机采样不外推。重合时方向无定义，只加代价，不推梯度。

已有场景卡都不写 `peer_sphere_enabled`。对照脚本在进程里覆盖，不改 `scenes/`。

`policy.py` 和 `crossing.py` 本票不改。它们仍只在 `swarm_penalty` 为真时把邻机采样送进 `refine_yopo`。脚本在进程里包了一层：欧氏球打开时送采样，椭球项仍关。因此默认运行器在只写这个新字段时还不会把邻机送进后段；下面的闭环数字来自这层包装。单测直接把邻机采样传进 `refine_yopo`。

## 对照怎么跑

臂：

- `off`：默认全关
- `bspline_only`：只开后段平滑，不开机间
- `sphere`：后段 + `peer_sphere_enabled`
- `sphere+track_geom_multi`：`sphere` 再加 E-C5b 的 `geom_multi`（goto 才读这个开关）

种子：`swap-3-open`、`crossing-pillars`、`crossing-forest`、`ego-density-014-swap3` 是 0–4；`crossing-6-forest` 是 0–2。影子盾开，仿真时长用场景卡。后段预算仍是墙钟 0.02 s、最多 8 次迭代。带后段的臂两个进程。预算截断合计 8/5444，不是结果差异的原因。

```bash
.venv/bin/python scripts/e-b4c-sphere-avoid.py
```

每粒落在 `out/e-b4c-sphere-avoid/partial/`，可续跑。穿林和柱林不读 `track_clearance`，所以 `sphere+track_geom_multi` 在这三张卡上与 `sphere` 的到达、碰撞、最小机间和到达时间相同。

表上的最小机间距离：没撞时是全程采样最小；撞了则是运行器第一次中心距 ≤ 0.70 m 那一拍的采样，仿真在这一拍停下。障碍碰撞是 `TRUTH_POSITION`，机间碰撞是 `INTER_DRONE`。到达时间是三架（或六架）都进到达判据时的 `arrival_time_s`，没全到就是空。

## 结果

开阔三机对向（`swap-3-open`）上，全关和只开后段都是 5/5 机间碰撞、到达 0/15。欧氏球两臂都是 15/15 到达、机间 0、障碍 0，最小机间 0.849–0.899 m，到达时间 6.88–6.89 s。软铰链把同时刻距离停在 0.85 m 球面上，没有再穿进 0.70 m。

密度 0.14 三机对向（`ego-density-014-swap3`）上，全关和只开后段仍是 5/5 机间碰撞、到达 0/15、障碍 0。`sphere` 是 12/15 到达、机间 1、障碍 0。解开的四粒最小机间 0.845–0.962 m，到达时间 7.57–16.71 s。seed 1 仍是 `uav1-uav2:INTER_DRONE`，记下 0.694 m。`geom_multi` 叠上去没有把这一粒解开，到达时间在解开的种子上有变化（8.60–18.02 s），机间碰撞数不变。障碍碰撞没有增加。

柱林三机和六机穿林上，四臂都全到达、障碍 0、机间 0。柱林和六机穿林的 `sphere` 与 `bspline_only` 逐种子相同（到达时间、最小机间都对得上）。这两张卡的真值最近距离都在 0.85 m 以外：柱林 3.611–7.308 m，六机 0.956–1.171 m。三机穿林的最小机间也与 `bspline_only` 逐种子相同（4.086–7.410 m）；seed 0 的到达时间从 22.59 s 变成 23.39 s，seed 1 从 22.59 s 变成 21.79 s。后段平滑本身会改柱林和六机的到达时间（柱林全关 7.05–7.76 s，开后段 6.99–7.02 s；六机全关 26.08–28.13 s，开后段 30.56–35.06 s），那是 E-B2 的平滑，不是欧氏球。

| 场景 | 臂 | 到达 | 障碍碰撞 | 机间碰撞 | 超时 | 最小机间 m | 到达时间 s | 后段 p50 / p95 ms | 预算截断 |
|---|---|---|---|---|---|---|---|---|---|
| swap-3-open | off | 0/15 | 0 | 5 | 0 | 0.631–0.633 | — | — | 0/0 |
| swap-3-open | bspline_only | 0/15 | 0 | 5 | 0 | 0.656–0.657 | — | 3.6–3.8 / 4.2–4.6 | 0/330 |
| swap-3-open | sphere | 15/15 | 0 | 0 | 0 | 0.849–0.899 | 6.88–6.89 | 4.8–4.9 / 7.1–7.4 | 2/590 |
| swap-3-open | sphere+track_geom_multi | 15/15 | 0 | 0 | 0 | 0.849–0.886 | 6.88–6.89 | 4.8–4.9 / 7.1–7.5 | 1/589 |
| crossing-pillars | off | 15/15 | 0 | 0 | 0 | 3.491–7.307 | 7.05–7.76 | — | 0/0 |
| crossing-pillars | bspline_only | 15/15 | 0 | 0 | 0 | 3.611–7.308 | 6.99–7.02 | 4.6–5.0 / 6.7–8.0 | 0/90 |
| crossing-pillars | sphere | 15/15 | 0 | 0 | 0 | 3.611–7.308 | 6.99–7.02 | 4.7–5.0 / 6.5–8.0 | 0/90 |
| crossing-pillars | sphere+track_geom_multi | 15/15 | 0 | 0 | 0 | 3.611–7.308 | 6.99–7.02 | 4.6–5.2 / 6.5–7.8 | 0/90 |
| crossing-forest | off | 15/15 | 0 | 0 | 0 | 6.622–7.388 | 20.77–24.12 | — | 0/0 |
| crossing-forest | bspline_only | 15/15 | 0 | 0 | 0 | 4.086–7.410 | 22.09–23.33 | 5.4–5.5 / 6.4–10.0 | 2/228 |
| crossing-forest | sphere | 15/15 | 0 | 0 | 0 | 4.086–7.410 | 21.79–23.39 | 5.3–5.5 / 6.3–7.6 | 0/231 |
| crossing-forest | sphere+track_geom_multi | 15/15 | 0 | 0 | 0 | 4.086–7.410 | 21.79–23.39 | 5.4–5.6 / 6.5–7.8 | 0/231 |
| crossing-6-forest | off | 18/18 | 0 | 0 | 0 | 1.148–1.392 | 26.08–28.13 | — | 0/0 |
| crossing-6-forest | bspline_only | 18/18 | 0 | 0 | 0 | 0.956–1.171 | 30.56–35.06 | 5.1–5.7 / 7.0–8.2 | 1/278 |
| crossing-6-forest | sphere | 18/18 | 0 | 0 | 0 | 0.956–1.171 | 30.56–35.06 | 5.3–5.6 / 7.3–9.0 | 0/278 |
| crossing-6-forest | sphere+track_geom_multi | 18/18 | 0 | 0 | 0 | 0.956–1.171 | 30.56–35.06 | 5.1–5.8 / 6.9–7.7 | 0/278 |
| ego-density-014-swap3 | off | 0/15 | 0 | 5 | 0 | 0.622–0.698 | — | — | 0/0 |
| ego-density-014-swap3 | bspline_only | 0/15 | 0 | 5 | 0 | 0.616–0.690 | — | 5.0–5.1 / 6.9–7.6 | 0/405 |
| ego-density-014-swap3 | sphere | 12/15 | 0 | 1 | 0 | 0.694–0.962 | 7.57–16.71 | 5.9–6.9 / 8.6–9.3 | 0/906 |
| ego-density-014-swap3 | sphere+track_geom_multi | 12/15 | 0 | 1 | 0 | 0.694–0.943 | 8.60–18.02 | 5.9–6.5 / 8.6–9.2 | 2/830 |

### 开阔三机，分种子

| 臂 | seed | 结果 | 到达 | 障碍 | 机间 | 最小机间 m | 到达时间 s |
|---|---|---|---|---|---|---|---|
| off | 0 | COLLISION | 0/3 | 0 | 1 | 0.631 | — |
| off | 1 | COLLISION | 0/3 | 0 | 1 | 0.632 | — |
| off | 2 | COLLISION | 0/3 | 0 | 1 | 0.632 | — |
| off | 3 | COLLISION | 0/3 | 0 | 1 | 0.633 | — |
| off | 4 | COLLISION | 0/3 | 0 | 1 | 0.632 | — |
| bspline_only | 0 | COLLISION | 0/3 | 0 | 1 | 0.657 | — |
| bspline_only | 1 | COLLISION | 0/3 | 0 | 1 | 0.657 | — |
| bspline_only | 2 | COLLISION | 0/3 | 0 | 1 | 0.657 | — |
| bspline_only | 3 | COLLISION | 0/3 | 0 | 1 | 0.656 | — |
| bspline_only | 4 | COLLISION | 0/3 | 0 | 1 | 0.657 | — |
| sphere | 0 | SUCCESS | 3/3 | 0 | 0 | 0.849 | 6.88 |
| sphere | 1 | SUCCESS | 3/3 | 0 | 0 | 0.877 | 6.89 |
| sphere | 2 | SUCCESS | 3/3 | 0 | 0 | 0.853 | 6.89 |
| sphere | 3 | SUCCESS | 3/3 | 0 | 0 | 0.855 | 6.89 |
| sphere | 4 | SUCCESS | 3/3 | 0 | 0 | 0.899 | 6.89 |
| sphere+track_geom_multi | 0 | SUCCESS | 3/3 | 0 | 0 | 0.849 | 6.88 |
| sphere+track_geom_multi | 1 | SUCCESS | 3/3 | 0 | 0 | 0.877 | 6.89 |
| sphere+track_geom_multi | 2 | SUCCESS | 3/3 | 0 | 0 | 0.853 | 6.89 |
| sphere+track_geom_multi | 3 | SUCCESS | 3/3 | 0 | 0 | 0.886 | 6.89 |
| sphere+track_geom_multi | 4 | SUCCESS | 3/3 | 0 | 0 | 0.880 | 6.89 |

### 密度 0.14 三机，分种子

| 臂 | seed | 结果 | 到达 | 障碍 | 机间 | 最小机间 m | 到达时间 s |
|---|---|---|---|---|---|---|---|
| off | 0 | COLLISION | 0/3 | 0 | 1 | 0.634 | — |
| off | 1 | COLLISION | 0/3 | 0 | 1 | 0.639 | — |
| off | 2 | COLLISION | 0/3 | 0 | 1 | 0.629 | — |
| off | 3 | COLLISION | 0/3 | 0 | 1 | 0.698 | — |
| off | 4 | COLLISION | 0/3 | 0 | 1 | 0.622 | — |
| bspline_only | 0 | COLLISION | 0/3 | 0 | 1 | 0.616 | — |
| bspline_only | 1 | COLLISION | 0/3 | 0 | 1 | 0.683 | — |
| bspline_only | 2 | COLLISION | 0/3 | 0 | 1 | 0.662 | — |
| bspline_only | 3 | COLLISION | 0/3 | 0 | 1 | 0.690 | — |
| bspline_only | 4 | COLLISION | 0/3 | 0 | 1 | 0.680 | — |
| sphere | 0 | SUCCESS | 3/3 | 0 | 0 | 0.946 | 14.76 |
| sphere | 1 | COLLISION | 0/3 | 0 | 1 | 0.694 | — |
| sphere | 2 | SUCCESS | 3/3 | 0 | 0 | 0.962 | 16.71 |
| sphere | 3 | SUCCESS | 3/3 | 0 | 0 | 0.934 | 14.87 |
| sphere | 4 | SUCCESS | 3/3 | 0 | 0 | 0.845 | 7.57 |
| sphere+track_geom_multi | 0 | SUCCESS | 3/3 | 0 | 0 | 0.943 | 8.60 |
| sphere+track_geom_multi | 1 | COLLISION | 0/3 | 0 | 1 | 0.694 | — |
| sphere+track_geom_multi | 2 | SUCCESS | 3/3 | 0 | 0 | 0.839 | 10.06 |
| sphere+track_geom_multi | 3 | SUCCESS | 3/3 | 0 | 0 | 0.937 | 10.39 |
| sphere+track_geom_multi | 4 | SUCCESS | 3/3 | 0 | 0 | 0.845 | 18.02 |

## 单测

`tests/test_e_b4c_sphere_avoid.py`：默认关时，近邻机采样传进 `refine_yopo` 与不传逐位相同；远邻机打开开关也不改曲线。两机迎面、参考同时刻距离约 0.30 m（小于 0.70 m）时，打开开关后最小机间增加超过 0.15 m，两端 PVA 仍钉死，代价和梯度没有 NaN。

```bash
.venv/bin/python -m pytest -q tests/test_e_b4c_sphere_avoid.py tests/test_backend_bspline.py tests/test_backend_swarm.py
```

## L0-DET 确定性重跑（2026-11-15）

当前 main `046d25a`。后段缺省 `budget="evals"`（0.020 s → 10 次评估）。五张卡、四臂、票面种子与上面的命令相同；`--jobs-plain 2 --jobs-backend 2`。92 粒连跑两遍，去掉墙钟字段后逐位相同。全关五张卡与上文一致。开后段的三臂在柱林、三机穿林、六机穿林上仍彼此相同。

| 场景 | 臂 | 旧到达 | 新到达 | 旧机间碰撞 | 新机间碰撞 | 旧障碍 | 新障碍 | 旧最小机间 m | 新最小机间 m |
|---|---|---|---|---|---|---|---|---|---|
| swap-3-open | 只开后段 | 0/15 | 0/15 | 5 | 5 | 0 | 0 | 0.656–0.657 | 0.662 |
| swap-3-open | 欧氏球 | 15/15 | 15/15 | 0 | 0 | 0 | 0 | 0.849–0.899 | 0.862–0.870 |
| crossing-6-forest | 开后段三臂 | 18/18 | 17/18 | 0 | 0 | 0 | 0 | 0.956–1.171 | 0.967–1.157 |
| ego-density-014-swap3 | 欧氏球 | 12/15 | 14/15 | 1 | 0 | 0 | 1 | 0.694–0.962 | 0.894–0.948 |
| ego-density-014-swap3 | 欧氏球 + geom_multi | 12/15 | 15/15 | 1 | 0 | 0 | 0 | 0.694–0.943 | 0.885–0.948 |

开阔三机：欧氏球仍是 15/15、机间 0，到达时间仍是 6.88–6.89 s。只开后段仍是 5/5 机间碰撞。柱林和三机穿林仍全到达、球和只开后段不分。六机穿林 seeds 0–2 变了：seed 0 三臂都超时 5/6，不再是 6/6。014 三机 seeds 0–4 变了：上文 seed 1 两臂都是 0.694 m 的机间碰撞；这次欧氏球是 14/15，机间碰撞 0，多 1 次撞树，geom_multi 是 15/15。

结论变了两处。六机「开后段仍 18/18」不再成立，少的是 seed 0 超时，球和只开后段仍相同，所以这不是球单独拉开的。014「seed 1 停在 0.694 m、geom_multi 解不开」不再成立。开阔三机「球把 5 粒都解开、只开后段解不开」没变。柱林和三机穿林「球不改变只开后段的到达和碰撞」没变。
