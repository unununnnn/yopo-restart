---
Status: accepted
Date: 2026-11-10
Scope: L0 crossing virtual center; formation schedule unchanged
---

# 穿林进度门控，开阔区保持开环时刻表

> 票 13e 起，六机 medium / sparse 林带卡暂关本门控（`progress_gate=false`），原因和数字见 [ADR 0015](0015-ghost-brake-stops.md)。门控代码和场景卡开关保留。

编号是 0013。0011 已由票 13a 占用（`docs/adr/0011-brake-gap-horizon-and-amax.md`）。0012 留给票 13d。0014 留给票 13c。本决定不改 yopo-swarm。

## Context

票 11c 之后，六机穿林的虚拟中心仍按墙钟 × `speed_mps` 开环走。落后的飞机停在 `CONFLICT_SKIP` 或 `TREE_SKIP` 上时，槽位已经跑到十几到三十米前面，队形误差 p95 停在 15–28 m。时刻表上的槽位线还会穿过树干，把目标线段拉进树里。11c 在林带里改成了本机前方的 goto，撞树降到 0，队形误差还是那个量级，因为参考点没有停下来等落后机。

frank 2026-11-09 定：进度门控只在林带开，开阔区关。思想是进度协同：虚拟中心的弧长取全队沿中心投影的最小值或低分位，落后机把参考按住，超前机不再领跑。不换 YOPO，不引入外部求解器。判据仍按 [0008](0008-report-only-until-frozen.md) 只报数。

承诺视界和急刹 `a_max` 是票 13a / ADR 0011 的题目。本决定不抬 `commit_horizon_s`，也不把非超前机的计划契约从 `a_max=80` 改成动力学的 6。

## Decision

- 穿林（`CrossingFleet`，场景卡带 `forest`）默认 `progress_gate=true`。没有 `forest` 的柱林穿越默认关闭。场景卡上的布尔盖过默认。
- `progress_percentile` 默认 0，范围 [0, 100]。0 是全队最小值。弧长是机体 x 减去虚拟中心原点 x，再减去本机槽位的纵向偏移，然后取这个分位，夹在 `[0, length_m]`。
- 门控开时，虚拟中心不再用墙钟 × `speed_mps`。队形误差对着这个弧长上的槽位。林带外的槽位前瞻也锚在这个弧长上。六机林带里的前方 goto 仍是 11c 的那条，只给没有超前的飞机。
- 超前：本机刚观测到的投影比低分位大过 1 m。用观测投影，不用 `t_start` 上的计划头。计划头已经沿着上一条曲线往前探，拿它做比较会把落后机判成超前。
- 超前机只提案两样：按场景卡 `a_max` 刹停或原地保持（`LEADER_HOVER`），或者 0.3 s 的 YOPO 前缀、尾巴同样按场景卡 `a_max` 停（`LEADER_PREFIX`）。不提交盖住下一轮窗口的长曲线。已经进到达半径的仍走原来的终端停止。
- 没超前的飞机仍走原来的上游 YOPO（走廊准入、最高分、邻机冲突就跳过）。候选顺序不改。`safe_radius`、kp、kd、tau 不改。
- 开阔区 `FormationFleet` 的 `progress_gate` 固定为 false。卡上写成 true 直接拒绝。时刻表仍是墙钟上的开环最小急动度曲线，追赶和 06c 的窗口内重发都不读这个分位。
- 场景卡可以写 `task.progress_gate` 和 `task.progress_percentile`。六机 medium / sparse 穿林卡写明开、分位 0。开阔区编队卡写明关。

## Consequences

- 队形误差 p95 相对 11c 的开环时刻表下降。目标是明显下降，争取低于 5 m。撞树相对 11c 不回退。到达数一并写入表。这些数字未冻结，不作为合入门。全表在 [13b 结果](../results/13b-progress-gate.md)。
- 低分位上的飞机若被树和邻机悬停预留一起挡住，参考就不往前走，后面的种子会超时。这是门控的结果，不是把碰撞藏起来。表上照实写到达和碰撞。
- 柱林穿越和开阔区编队的开环时刻表保持原样。编队卡若打开进度门控，构造阶段就失败，不会悄悄改成另一套调度。
- 票 13a 若随后把承诺视界和契约 `a_max` 改掉，要在那条 ADR 上改，不在这里提前改非超前机的长前缀。
