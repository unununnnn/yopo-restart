---
Status: accepted
Date: 2026-11-10
Scope: L0 六机林带穿越（CrossingFleet，n ≥ 6 且卡上有 forest）的提案前自检；林带场景卡的进度门控开关
---

# 幽灵刹车停点：自检对齐邻机投票口径，林带暂关进度门控

编号是 0015。本决定不改 yopo-swarm，也不改 vendored core。

## Context

13a、13c、13d、13b 叠在 main（b289b2e）上之后，六机穿林 10 个 seed 的到达是 medium 0/60、sparse 9/60。全部在 50 s 超时，决策里 `LEADER_HOVER` 占多数。

打点结果（medium seed 1、sparse seed 9）：进度最低的那架（uav2）每扇窗口都 `CONFLICT_SKIP`，冲突对象是 uav0。但和它冲突的不是 uav0 当前的计划，而是 uav0 早已被替换掉的旧计划。medium seed 1 的 t=14.25 s，uav2 在 (12.85, 3.06)，冲突全部来自 uav0 在 6.25–12.25 s 提交的几份计划，也就是 uav0 在 (13.57, 2.02) 悬停时留下的。那时 uav0 的当前计划已经在 18.9 m 处，不冲突。

机制：

- 每份计划在 `t_start` 带一条刹车分支。分支停点的 `Hold` 一直占到 `inf`。
- 邻机提交新计划时，本机的 `peers[m].plans` 只把旧计划截到新计划的 `t_start`。截断只砍名义段，分支停点照旧到 `inf`，而且旧计划不会被删掉。
- 提案前的自检 `conflicts_with_neighbors()` 拿邻机的**全部**历史计划来比。邻机投票时（`_decide_proposal`）却只用自己的 `own_possible()`（committed + previous + in_flight）和已 ACK 的预留。提案方比投票方严得多，被替换的停点越攒越多，永远不会过期。
- 13b 让超前机在林子里原地悬停，悬停点就成了永久停点。13c 弹性槽位让侧向通道交叉，落后机正好在这些停点后面。落后机走不动，门控取 min，超前机继续悬停、继续留停点，于是整队冻住。

本地对照（六机 10 seed，全表见 [13e 结果](../results/13e-ghost-stops.md)）：只关门控（E）两档都是 47/60；只修自检（F）medium 2/60、sparse 36/60，门控已经不会冻住，但全队按 min 爬，50 s 内飞不完；两者一起（E+F）两档都是 60/60，碰撞 0。

## Decision

- 六机林带（`_forest_six()`）的提案前自检改为 `_conflicts_with_live()`：每个邻机只看 `peer.plans[-2:]`，也就是它的 committed + previous，加上它的预留（`delay_check_s > 0` 时再加 heard）。这与邻机投票时用的集合一致。三机穿林和柱林仍走 `conflicts_with_neighbors()`，行为不变。
- 投票、COMMIT、`own_possible()` 都不改。协议层的安全仍由投票方保证：提案方自检本来就只是预筛。
- 六机 medium / sparse 穿林卡写 `progress_gate=false`。进度门控（ADR 0013）的代码、默认值和场景卡开关都保留。三机穿林卡仍写 true，柱林仍默认关，开阔区编队仍固定关。
- 不改 13a 的承诺视界 1.75 s 和计划 a_max=6，不改 13d 的穿林 `delay_check_s=0`，`goal_bias` 仍是 elastic-slot。不带回 `_loose_goal`、`_LIMITS` 或 a_max=80。

## Consequences

- 六机穿林两档到达 60/60，碰撞 0，最小障碍净空 0.126 m（medium）/ 0.158 m（sparse）。判据仍按 [0008](0008-report-only-until-frozen.md) 只报数。
- 门控关掉后，官方队形误差 p95 对着墙钟参考，回到 18–26 m。与门控无关的相对 p95（每拍以全队平均位置为中心）中位数是 medium 9.3 m、sparse 6.2 m，比只关门控的 E（21.1 / 18.1 m）紧，但比门控开时（约 2.6–2.8 m）散。
- 重新打开林带门控之前，要先解决按 min 爬行太慢的问题：A+F（G）medium 4/60、sparse 33/60，门控能推进到 27–42 m，但飞不完。可选的方向有超前机改走短前缀而不是原地悬停，或放大 `_LEADER_MARGIN_M`，都还没有测。
- vendored core 里旧计划分支停点一直占到 `inf` 的问题仍在。其他用 `conflicts_with_neighbors()` 的路径（三机、柱林、开阔区）没有改；要改得回到 yopo-swarm 上游。
