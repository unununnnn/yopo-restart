---
Status: accepted
Date: 2026-11-09
Scope: L0 upstream terminal rule
---

# L0 用 `stop_at_goal`，不用上游原样 `hold_here`

## Context

上游 YOPO-MINCO（`test_yopo_ros`）在进入到达半径后停止重规划，在当时的位置 `hold_here`。上游半径是 5 m。网络按远处目标训练，目标在机体系里截断到约 10 m，不会在目标点减速停住。

本项目的 G2 oracle 严于上游：距目标 ≤ 0.4 m、速度 ≤ 0.2 m/s、连续保持 1 s（[docs/SPEC.md](../SPEC.md) 票 02；`scenes/g2-pillar.json` 的 `criteria`）。机体从 `(0, 0, 1.5)` 飞到 `(5, 0, 1.5)`，起点到目标正好 5 m。

`UpstreamPolicy` 因此有两条终端规则，由场景卡 `policy_params.upstream` 选择：

- `hold_here`：进入半径后锁定当前位置，不再重规划（上游原样）。
- `stop_at_goal`：进入半径后，用一段五次多项式从期望状态（默认 `plan_from_reference=True` 时的参考 PVA）在 `stop_time_s`（默认 2 s）内停到目标点，速度和加速度为零，然后保持。

代码里的默认值是 `arrive_mode=stop_at_goal`、`arrive_radius_m=1.5`。`yopo_l0/policy.py` 的注释仍写着「待 frank 确认」；那句已经过时。2026-11-09 开发 bot 按 frank 的 2 分钟规则采纳 1.5 m，作为 L0 的终端半径。frank 以后可以改。本 ADR 只记录这个采纳，不改策略代码。对照数字仍留在下面。

对照跑在 commit `036d90761c13beb2ad5574fc685af5f138b996e3`，几何与 `scenes/g2-scene.json` 相同，影子盾开，仿真时长用场景卡的 45 s。`hold_here` 两行不是已提交的场景卡，只在 `policy_params.upstream` 上覆盖了终端参数。

| 终端 | 半径 | 场景卡哈希 | 到达 | 碰撞 | 终点误差 |
|---|---|---|---|---|---|
| `stop_at_goal`（L0 默认，已提交的 `g2-pillar`） | 1.5 m | `b00714a404ea47a64610ce4a057450b03326a2e574c6a2671ad9bf1338dd4f1d` | 10/10，平均 4.27 s | 0 | 0.016 m |
| `hold_here` | 1.5 m | `01a51d52f6f205424114bcfc95df215fe4bf09bb91677836a4000d9126351564` | 0/10，45 s 超时 | 0 | 1.43–1.46 m |
| `hold_here` | 5.0 m（上游半径） | `add5115e23baa97fb3272c5fbdfa2d3fa333cde8fd6c7db532ffa7c7dbf7742a` | 0/10，45 s 超时 | 0 | 5.000 m |

1.5 m 的 `hold_here` 与默认规则飞过同一段走廊（最小障碍距离都约 0.83 m，大于机体半径 0.35 m），进入半径后停在距目标约 1.4 m 处，速度归零，但到不了 0.4 m。5.0 m 的 `hold_here` 在第一次规划后就锁定：起点已经在上游半径上，终点误差保持 5 m，飞机留在起点。

逐 seed 表和复现命令见 [M0 样例](../results/m0-g2-pillar.md)。

## Decision

- L0 的 `upstream` 默认终端是项目规则 `stop_at_goal`，半径 1.5 m，停车段 2 s。不用上游原样的 `hold_here`。
- 原因：在 G2 oracle 0.4 m 下，`hold_here` 到不了终点。1.5 m 停在半径内侧约 1.4 m；5 m 在这张 5 m 的目标上等于不起飞。`stop_at_goal` 才让「上游选候选」和「项目到达口径」同时成立。10/10 到达、0 碰撞是这条终端规则下的数字。
- 这是相对上游到达行为的登记偏差，只发生在进入半径之后：一段不看场景几何的五次多项式，停到场景卡里的目标点。走廊阈值 0.05 m、最高分、无准入候选时本帧悬停、`plan_from_reference` 仍按上游。偏差不进 vendored `core/`，也不改网络和权重。
- `hold_here` 留在 `policy_params.upstream.arrive_mode`，用来复现上表。半径 1.5 m 已采纳为 L0 默认，不再悬着等确认。采纳的是终端参数，不是 M0 的过/不过门；到达数在判据冻结前仍只报数（见 [0008](0008-report-only-until-frozen.md)）。frank 以后可以改这个半径。

## Consequences

- 口头说「upstream 过了 G2」时，终端是 `stop_at_goal`，不是上游 5 m `hold_here`。样例表头必须带上终端规则。
- 若改回 `hold_here` 再冻结「必须到达」，按上表 M0 会是 0/10。那是规则选择，不是网络退步。
- 新增别的终端偏差要先改本 ADR 再改默认值，和 [0006](0006-yopo-minco-baseline.md) 的保真登记同一规矩。本 ADR 本身不改 `yopo_l0/policy.py`。
