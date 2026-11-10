---
Status: accepted
Date: 2026-11-10
Scope: L0 formation protocol
---

# 编队新轨走 RMADER 的 Delay Check，不换 YOPO

## Context

frank 定了票 13d：在现有 ACK/COMMIT 上抄 RMADER 的协议语义。新轨要经过 PROPOSED → TENTATIVE（已广播）→ DELAY_CHECK → COMMITTED。上一份已提交的安全轨一直留着。Delay Check 期间如果收到和这条新轨冲突的邻机轨迹，包括迟到才到的，就作废新轨再提。不换 YOPO，也不引入 Gurobi。

RMADER（Kondo 等，arXiv:2303.06222）把鲁棒性放在三件事上：Delay Check（δ_DC ≥ 最大通信时延）、两段发布（先广播 trajopt，检查通过再广播 trajcomm）、把邻机的已提交轨和新轨都存下来再查。优化器是他们自己的，仿真里用 Gurobi。本仓库的运动更新仍是轮转窗口里的预留，编队轨迹是槽位时刻表，穿林避障是上游 YOPO。

票 06c（PR #20，写这篇时还没合进 main）说明六机 drop 20% 到达率掉下去，是因为提案和 ACK 在窗口里只发一次。本分支把那次窗口内重发一起带上，否则 13d 的「六档到达保持 1.000」对不上现有的丢包档。

## Decision

- 协议节点增加 `TrackPhase`。`make_proposal` 在返回 PROPOSE 之前依次进入 PROPOSED、TENTATIVE、DELAY_CHECK。TENTATIVE 就是这条消息已经交给调用方去广播。
- `ProtocolConfig.delay_check_s` 是 δ_DC，默认 0。默认时收齐 N-1 个绑定一致的 ACK 仍在当拍提交，离散事件测试和穿林、变阵的提交时序不变。
- 编队把 δ_DC 设为 0.05 s，等于 `scenes/formation-6-tiers.json` 里 reorder+delay / combined 的时延上界。收齐 ACK 并且当前时刻到达 `broadcast_t + δ_DC` 之后才 COMMIT。提交前 `committed` 不换，`executing()` 在 `t_start` 之前仍是上一份。票齐之后不再重发同一份提案，Delay Check 只等时间。
- Delay Check 开着的时候，收到与新轨冲突的邻机提案或提交就 RELEASE 新轨，原因是 `CONFLICT` 或 `LATE`。已提交轨不动。同一窗口如果还留得下一次完整的 Delay Check，可以再提一份新的 `plan_seq`。窗口内重发（06c）重发的是同一个 reservation，不换 `plan_seq`。
- 穿林和变阵显式保持 `delay_check_s = 0`。YOPO 的候选、解码和 `safe_radius` 都不改。

## 偏离

和 RMADER 论文不一致的地方：

1. 没有 trajopt 优化，也没有 Gurobi。新轨仍是编队槽位曲线或上游 YOPO 选出的多项式，再套原来的计划契约。
2. ACK 法定人数保留。RMADER 不收集 ACK，它靠 δ_DC 覆盖在途消息。这里两道都要过：票没齐不能提交，齐了也要等 Delay Check。
3. 提案权仍是轮转窗口，不是各机异步随时重规划。作废后的重提也要落在自己的窗口里，并且窗口剩余时间要够一次 δ_DC。
4. δ_DC 取故障档的时延上界 0.05 s，不是在网上测出来的 δ_max。论文要求 δ_DC ≥ δ_max；档位以外的更大时延不在这张票的安全声明里。
5. 穿林不打开等待。δ = 0 时状态机仍会走过这四个阶段，但 Delay Check 的长度是 0，收齐 ACK 当拍提交。这样林中的已提交轨和撞树次数不跟着编队的等待一起变。

和「收齐 ACK 立刻 COMMIT」不一致的地方只在 `delay_check_s > 0` 的编队：同一 reservation 的重发照旧，提交时刻推迟到 δ_DC。动力学要等 `t_start` 才跟踪新轨，所以推迟提交不改 `t_start` 之后的采样，只要提交发生在 `t_start` 之前。

## Consequences

- 六机开阔区六档、seeds 0–4 的数字在 `docs/results/13d-dual-track.md`。判据仍未冻结。
- 穿林要打开同样的等待，得另定 δ_DC，并重跑疏/中/密的撞树表。本 ADR 不把那次重跑算成已经做过。
