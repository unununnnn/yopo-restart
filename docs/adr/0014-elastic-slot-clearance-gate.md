---
Status: accepted
Date: 2026-11-10
Scope: L0 six-drone forest slots and the pre-commit clearance gate
---

# 六机林带槽位是软目标，COMMIT 前用净空门过滤

## Context

票 11b（#18）的六机楔形穿 medium / sparse 林，规划参数与三机卡相同，避障仍是上游 YOPO，`safe_radius` 0.05 m。medium 到达 23/60、障碍碰撞 6；sparse 到达 40/60、障碍碰撞 1。首撞都是已经提交的计划。时刻表按 3.5 m/s 把槽位拉到机头前方十几到二十米，槽位线穿过树干；六机一轮窗口 1.5 s 长于承诺地平线 1.0 s，前缀结束后飞的是按计划契约 `a_max=80` 生成的末端急刹，L0 动力学 `a_max` 仍是 6，跟踪差把人推进树里。

frank 定的是弹性槽位，不是把走廊放进优化器。林带里槽位是软目标：侧向和纵向可以在界内松开，避障是硬约束。出了林带再收紧回刚性槽位。前方 goto 可以沿用 11c 的做法（目标留在机头前方，不追已经跑远的时刻表），这里把它收成「只瞄到还能走的那一截」。

COMMIT 之前要有一道硬净空门。思想来自 LSC：不可行走的走廊不进优化。本仓库不引入 CPLEX，也不改 YOPO 的候选排序。不够净空的多项式直接丢掉，不提案、不提交。

`safe_radius` 默认仍是上游 0.05 m。L0 跟踪的 kp、kd、tau、a_max 不改。计划契约里那条 `a_max=80` 的限幅也不在本 ADR 里改。承诺视界下界和急刹加速度对齐动力学是票 13a，编号 0011（`0011-brake-gap-horizon-and-amax.md`）留给那张票，本分支不写那个文件。

0006 正文里提到的 ADR-0011 是 yopo-swarm 的编号，不是本仓库这组 L0 ADR。本 ADR 不改 yopo-swarm。

## Decision

- 只对带 `forest` 且不少于六机的 `CrossingFleet` 生效。三机穿林和柱林仍用原来的槽位前瞻，规划器不读树。
- 林带内（前方 `longitudinal_slack_m` 已经探到林带，且人还没出林）槽位是软目标。侧向在槽位左右 `lateral_slack_m`（默认 2 m）内，一次横移不超过同一界，再剪到走廊的 y 范围。纵向只取 `longitudinal_slack_m`（默认等于 `min_ahead_m`）里、整段中心距都过净空门的那一截射线。能走的侧向优先贴槽位，再取更长的净空。射线缩短时目标仍停在同一条线上。
- 出林前 `exit_tighten_m`（默认 4 m）把松弛收到 0。收紧后的射线要是已经进树，就放回界内松弛，避障不让步。x 出了林带之后回到原来的刚性槽位前瞻。
- COMMIT 前对已拟合轨迹按 0.02 s 采样。中心到树干或矮枝表面的最小距离必须 ≥ `body_radius + clearance_margin_m`。margin 默认 0.15 m，场景卡 `task.clearance_margin_m` 可改，不能为负。不够就记 `CLEARANCE_SKIP`，不把这条多项式交给 `_as_plan`，也就不会提案、不会 COMMIT。刹车参考同样过这道门。这是过滤，不是优化。
- 六机且一轮窗口长于卡上的 `commit_horizon_s` 时，交出的 YOPO 前缀盖到一轮窗口（六机是 1.5 s），少飞末端急刹。三机仍用卡上的 1.0 s。不因此改计划契约的 `a_max`。
- 判据仍未冻结。到达和碰撞照实入表，不作为合入门。见 [0008](0008-report-only-until-frozen.md)。

## Consequences

- 同一张场景卡（medium `128eb741…`、sparse `6931bead…`，与 #18 相同）重跑 seeds 0–9：medium 到达 51/60、sparse 53/60，障碍碰撞和机间碰撞都是 0。最小净空 medium 0.119 m、sparse 0.129 m，离开 #18 的负净空，也离开 11c 在 medium seed 4 上的 0.017 m 贴边。全表在 [13c 结果](../results/13c-elastic-clearance.md)。
- 队形误差 p95 仍在约 19–29 m。时刻表继续以 3.5 m/s 走，软目标不追它，落后的飞机不会为了贴槽位穿树。
- `safe_radius` 仍是 0.05 m，结果里不标偏离上游。`PointMassLag` 仍是 kp 4、kd 3、tau 0.1、a_max 6。
- 本仓库 L0 ADR 的 0011、0012、0013 分别留给票 13a、13d、13b。本决定只占 0014。
