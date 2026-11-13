# 票 E-B4d：014-swap3 seed 1 欧氏球未推开

report-only。只报数。不改 `yopo_l0/`、`core/`、`scenes/`，不冻结，不进 `scenes/suite.txt`。

report-only。只报数。不改 yopo_l0 后段、不冻结、不进 scenes/suite.txt，不设 gates。本票不拷贝 PR #94 的欧氏球实现。main 尚未带 peer_sphere_enabled 时跳过实飞，标 missing_feature。开关合入后再用同一脚本逐 tick 回答 014-swap3 seed 1（对照 seed 0）。

状态：`missing_feature`。实飞：`skipped`。

## 为什么这次没有逐 tick 的数

本分支从 main 拉出。`yopo_l0.backend.config.BackendConfig` 没有 `peer_sphere_enabled`，`refine_yopo` 也不读这个字段，`config_from_params` 不会把它打开。按票面跳过 `ego-density-014-swap3` 的实飞，JSON 标 `missing_feature`。不把 PR [#94](https://github.com/unununnnn/yopo-restart/pull/94)（`yopo-restart-bot/e-b4c-sphere-avoid`）的后段源码拷进本 PR。

待该 PR 合入 main 之后，在合入后的树上去掉 `--partial` 重跑，才会记下 seed 1 和 seed 0 的逐 tick。

## 引自 E-B4c，本分支未重测

数字引自 PR #94 的 `docs/research/e-b4c-sphere-avoid.md`，本分支没有重测。

| 臂 | seed | 结果 | 最小机间 m | 到达时间 s |
|---|---|---|---|---|
| sphere | 0 | SUCCESS | 0.946 | 14.76 |
| sphere | 1 | COLLISION | 0.694 | — |
| sphere+track_geom_multi | 0 | SUCCESS | 0.943 | 8.60 |
| sphere+track_geom_multi | 1 | COLLISION | 0.694 | — |

seed 1 两臂都是 `uav1-uav2` 相撞，最小距 0.694 m，贴着 0.70 m 碰撞球。叠 `geom_multi` 没有解开。

## 四问

1. 撞前 1.0 s 内后段是否收到邻机采样；第 0 个规划拍是否缺邻机

状态：`deferred`。main 的 BackendConfig / refine_yopo 还没有可用的 peer_sphere_enabled。实飞诊断待 PR #94 合入后再跑（不要 --partial）。

2. 铰链是否激活；激活时的三维距、代价、控制点位移、末端是否因钉死无法上推

状态：`deferred`。main 的 BackendConfig / refine_yopo 还没有可用的 peer_sphere_enabled。实飞诊断待 PR #94 合入后再跑（不要 --partial）。

3. 碰撞在轨迹头/中/尾；是否一侧 CORRIDOR_EMPTY_HOVER / empty_brake、另一侧还在飞

状态：`deferred`。main 的 BackendConfig / refine_yopo 还没有可用的 peer_sphere_enabled。实飞诊断待 PR #94 合入后再跑（不要 --partial）。

4. 几何门放行的参考轨迹自身是否已经 < 0.70 m

状态：`deferred`。main 的 BackendConfig / refine_yopo 还没有可用的 peer_sphere_enabled。实飞诊断待 PR #94 合入后再跑（不要 --partial）。

## 本次冒烟

- 场景卡能读，飞机是 uav0、uav1、uav2，机体半径 0.35 m，碰撞线 0.70 m。
- 场景文件未改（sha256 `a46380a0e72437d64281c1aa7e0f4686a50ce2af6a802e784638628f4ffd005a`）。
- 运行器源码里，`plan` 在 `bus.poll` 之前。这是静态检查，不是 seed 1 的逐 tick 记录。
- 进程内卡片写了 `peer_sphere_enabled`，半径 0.85 m、权重 8。当前 `config_from_params` 不认这个字段时，不会拿它去飞。

## 开关可用时脚本记什么

进程内包一层，和 E-B4c 对照脚本一样：欧氏球打开时把邻机采样送进 `refine_yopo`，椭球项仍关。不改 `policy.py`。然后对 seed 0（成功参照）和 seed 1（`uav1-uav2`）各跑 `sphere` 与 `sphere+track_geom_multi`。

- 每次 `plan` 记下当时 `_neighbors` 是否为空。第 0 拍应为空：本 tick 的广播要等全部 `plan` 返回后才 `bus.poll`。
- 每次 `refine_yopo` 记下传入的邻机份数。撞前 1.0 s（没撞则是最近时刻前 1.0 s）里有没有非空采样。
- 铰链用后段自己的 `_peer_sphere_cost_grad` 量。记三维距、代价、自由控制点位移、末端三控制点位移。末端钉死且球内最近点落在被排除的最后一跨（或梯度打在钉死的控制点上）时，标末端推不动。
- 碰撞时刻相对当时轨迹：头 / 中 / 尾。`CORRIDOR_EMPTY_HOVER` 且轨迹短于 0.05 s 标 `hover_hold`。本票的两臂都不开 `empty_brake`。
- `geom_multi` 只看参考轨迹对障碍表面的净空，不看机间距离。记下几何门打开时放行的参考多项式与邻机的同时刻最小距，看它是不是已经 < 0.70 m。

## 怎么重跑

```bash
.venv/bin/python scripts/e-b4d-sphere-miss.py --partial --write
.venv/bin/python scripts/e-b4d-sphere-miss.py --write
.venv/bin/python -m pytest tests/test_e_b4d_sphere_miss.py
```

`--partial` 永远不飞。第二条只在 `peer_sphere_enabled` 可用时跑完四粒并覆盖这份报告。
