# 票 E-B4d：014-swap3 seed 1 欧氏球未推开

report-only。只报数。不改 `yopo_l0/`、`core/`、`scenes/`，不冻结，不进 `scenes/suite.txt`。

report-only。只报数。不改 yopo_l0 后段、不冻结、不进 scenes/suite.txt，不设 gates。本票不拷贝 PR #94 的欧氏球实现。main 尚未带 peer_sphere_enabled 时跳过实飞，标 missing_feature。开关合入后再用同一脚本逐 tick 回答 014-swap3 seed 1（对照 seed 0）。

状态：`diagnosed`。实飞：`ran`。

## 引自 E-B4c 的原表

下面这张表是 PR #94 的原数字，不是这次重测。这次的四问和对照在后面。

| 臂 | seed | 结果 | 最小机间 m | 到达时间 s |
|---|---|---|---|---|
| sphere | 0 | SUCCESS | 0.946 | 14.76 |
| sphere | 1 | COLLISION | 0.694 | — |
| sphere+track_geom_multi | 0 | SUCCESS | 0.943 | 8.60 |
| sphere+track_geom_multi | 1 | COLLISION | 0.694 | — |

seed 1 两臂都是 `uav1-uav2` 相撞，最小距 0.694 m，贴着 0.70 m 碰撞球。叠 `geom_multi` 没有解开。

## 四问

1. 撞前 1.0 s 内后段是否收到邻机采样；第 0 个规划拍是否缺邻机

- sphere seed 1：第 0 个规划拍没有邻机。事件前 1.0 s 内后段收到邻机的调用 30/30。 第一次 plan 之前没有邻机采样。
- sphere seed 0：第 0 个规划拍没有邻机。事件前 1.0 s 内后段收到邻机的调用 30/30。 第一次 plan 之前没有邻机采样。
- geom_multi seed 1：第 0 个规划拍没有邻机。事件前 1.0 s 内后段收到邻机的调用 30/30。 第一次 plan 之前没有邻机采样。
- geom_multi seed 0：第 0 个规划拍没有邻机。事件前 1.0 s 内后段收到邻机的调用 26/26。 第一次 plan 之前没有邻机采样。

2. 铰链是否激活；激活时的三维距、代价、控制点位移、末端是否因钉死无法上推

- sphere seed 1：铰链激活 20 次，激活时最小三维距 0.242 m，最大代价 6.798，自由控制点最大位移 0.874 m。末端三个控制点钉死，末端位移最大 0.000 m。
- sphere seed 0：铰链激活 29 次，激活时最小三维距 0.304 m，最大代价 5.946，自由控制点最大位移 0.649 m。末端三个控制点钉死，末端位移最大 0.000 m。
- geom_multi seed 1：铰链激活 20 次，激活时最小三维距 0.242 m，最大代价 6.798，自由控制点最大位移 0.874 m。末端三个控制点钉死，末端位移最大 0.000 m。
- geom_multi seed 0：铰链激活 20 次，激活时最小三维距 0.304 m，最大代价 5.946，自由控制点最大位移 0.641 m。末端三个控制点钉死，末端位移最大 0.000 m。有 1 次最近点在被排除的最后一跨。

3. 碰撞在轨迹头/中/尾；是否一侧 CORRIDOR_EMPTY_HOVER / empty_brake、另一侧还在飞

- sphere seed 1：碰撞 t=2.64 s，距离 0.694 m。轨迹位置 uav1=head，uav2=head。规划码 uav1=OK，uav2=OK。不是一侧悬停、另一侧飞过来。empty_brake 关着。
- sphere seed 0：最近接近 t=2.67 s，距离 0.952 m。轨迹位置 uav1=head，uav2=head。规划码 uav1=OK，uav2=OK。不是一侧悬停、另一侧飞过来。empty_brake 关着。
- geom_multi seed 1：碰撞 t=2.64 s，距离 0.694 m。轨迹位置 uav1=head，uav2=head。规划码 uav1=OK，uav2=OK。不是一侧悬停、另一侧飞过来。empty_brake 关着。
- geom_multi seed 0：最近接近 t=2.67 s，距离 0.943 m。轨迹位置 uav1=head，uav2=head。规划码 uav1=OK，uav2=OK。不是一侧悬停、另一侧飞过来。empty_brake 关着。

4. 几何门放行的参考轨迹自身是否已经 < 0.70 m

- sphere seed 1：几何门在这次运行里没有打开过（geom_multi 要等邻机采样；第 0 拍没有）。 不论门开没开，参考轨迹自身最小 0.174 m，已经 < 0.70 m。
- sphere seed 0：几何门在这次运行里没有打开过（geom_multi 要等邻机采样；第 0 拍没有）。 不论门开没开，参考轨迹自身最小 0.146 m，已经 < 0.70 m。
- geom_multi seed 1：几何门放行的参考轨迹有 37 次同时刻距离已经 < 0.70 m，最小 0.174 m。 不论门开没开，参考轨迹自身最小 0.174 m，已经 < 0.70 m。
- geom_multi seed 0：几何门放行的参考轨迹有 27 次同时刻距离已经 < 0.70 m，最小 0.146 m。 不论门开没开，参考轨迹自身最小 0.146 m，已经 < 0.70 m。

## 对照

| 臂 | seed 1 结果 | seed 1 最小机间 m | seed 0 结果 | seed 0 最小机间 m |
|---|---|---|---|---|
| sphere | COLLISION | 0.694 | SUCCESS | 0.842 |
| sphere+track_geom_multi | COLLISION | 0.694 | SUCCESS | 0.943 |

## 开关可用时脚本记什么

进程内包一层，和 E-B4c 对照脚本一样：欧氏球打开时把邻机采样送进 `refine_yopo`，椭球项仍关。不改 `policy.py`。然后对 seed 0（成功参照）和 seed 1（`uav1-uav2`）各跑 `sphere` 与 `sphere+track_geom_multi`。

- 每次 `plan` 记下当时 `_neighbors` 是否为空。第 0 拍应为空：本 tick 的广播要等全部 `plan` 返回后才 `bus.poll`。
- 每次 `refine_yopo` 记下传入的邻机份数。撞前 1.0 s（没撞则是最近时刻前 1.0 s）里有没有非空采样。
- 铰链用后段自己的 `_peer_sphere_cost_grad` 量。记三维距、代价、自由控制点位移、末端三控制点位移。末端钉死、末端没有位移，并且球内最近点落在铰链不看的最后一跨时，才标这一跨推不到。邻近跨漏到钉死点上的小梯度不算。
- 碰撞时刻相对当时轨迹：头 / 中 / 尾。`CORRIDOR_EMPTY_HOVER` 且轨迹短于 0.05 s 标 `hover_hold`。本票的两臂都不开 `empty_brake`。
- `geom_multi` 只看参考轨迹对障碍表面的净空，不看机间距离。记下几何门打开时放行的参考多项式与邻机的同时刻最小距，看它是不是已经 < 0.70 m。

## 怎么重跑

```bash
.venv/bin/python scripts/e-b4d-sphere-miss.py --partial --write
.venv/bin/python scripts/e-b4d-sphere-miss.py --write
.venv/bin/python -m pytest tests/test_e_b4d_sphere_miss.py
```

`--partial` 永远不飞。第二条只在 `peer_sphere_enabled` 可用时跑完四粒并覆盖这份报告。

## L0-DET 确定性重跑（2026-11-15）

当前 main `046d25a`。后段缺省 `budget="evals"`（0.020 s → 10 次评估）。四粒仍是 014-swap3 的 seed 0 和 seed 1、`sphere` 与 `sphere+track_geom_multi`。命令是 `--write`，输出没有覆盖这份正文。连跑两遍，去掉墙钟字段后逐位相同。

| 臂 | 旧 seed 1 | 新 seed 1 | 旧 seed 1 最小机间 m | 新 seed 1 最小机间 m | 旧 seed 0 | 新 seed 0 | 新 seed 0 最小机间 m |
|---|---|---|---|---|---|---|---|
| sphere | COLLISION | SUCCESS | 0.694 | 0.917 | SUCCESS | SUCCESS | 0.948 |
| sphere+track_geom_multi | COLLISION | SUCCESS | 0.694 | 0.885 | SUCCESS | SUCCESS | 0.948 |

结论变了。这张票的题目是 seed 1 没被推开。确定性预算下两臂都把 seed 1 推过了 0.70 m，最近接近在 2.67 s，距离 0.917 m 和 0.885 m，规划码仍是 OK，不是一侧悬停。第 0 拍仍没有邻机，事件前 1.0 s 内后段仍收到邻机。铰链仍会激活，末端三个控制点仍钉死。这些机制还在，但「未推开、停在 0.694 m」不再成立。
