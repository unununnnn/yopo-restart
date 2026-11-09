---
Status: accepted
Date: 2026-11-09
Scope: L0 M0–M4 criteria
---

# M0–M4 判据冻结前只报数

## Context

[docs/SPEC.md](../SPEC.md)「判据建议值」写明：数值待 frank 冻结，冻结前只报数、不判过。建议列是到达数、碰撞数（建议必须为 0）、到达时间、墙钟。

同一份 SPEC 的票 02 验收清单另有一句「碰撞数为 0 才算通过」。两句在冻结前冲突。2026-11-09 父代理代定的是前一句，并写进 [docs/tickets.md](../tickets.md)：

> 结果表写到达数 / 碰撞数 / 到达时间，过/不过标「未冻结 / report-only」。碰撞数 = 0 作为硬事实如实报，但不据此判合 main。

场景卡 `scenes/g2-pillar.json` 的 `criteria.frozen` 为 `false`。到达口径仍是 G2 oracle：距目标 ≤ 0.4 m、速度 ≤ 0.2 m/s、连续保持 1 s。机体半径 0.35 m。碰撞用场景几何对真值位置判定，不走影子。

`yopo_l0.batch` 的退出码与此一致：hover 任务要全部 `SUCCESS`、确定性重跑一致、墙钟低于上限；goto 的到达数和碰撞数不改变退出码。

## Decision

- `criteria.frozen` 仍为 false 时，M0–M4（goto 以及其后的编队任务）只报数。表上的结论写成「未冻结 / report-only」，不写成通过或失败。
- 到达数、碰撞数、到达时间、最小障碍距离、终点误差每次都写。碰撞数是几何真值，有碰撞必须出现在表里。
- 在判据冻结之前，这些数字不作为合 main 的门，也不作为 `scripts/regress.sh` / `yopo_l0.batch` 对 goto 的退出码。
- 票 01 的 hover 任务保持硬判据：`SUCCESS` 才通过。确定性（墙钟除外）和 hover 的墙钟上限也保持为门。
- 与 SPEC 票 02 清单「碰撞数为 0 才算通过」的偏差：冻结前不执行该句。碰撞数继续计算、继续印在表上。frank 冻结并把 `criteria.frozen` 设为 true 之后，才允许批跑把到达和碰撞变成退出码。那次改动要另记，不在本 ADR 里提前改代码。

## Consequences

- certified 在 G2 柱子上 0/10 到达（见 [0010](0010-certified-env-unknown-reported.md)）可以和 upstream 的 10/10 放在同一张样例表里，回归仍然是绿的。这是测量，不是放水把门关掉。
- 冻结前，upstream 若撞上柱子，表上会有碰撞数，但当前回归脚本不会因此失败。读表的人要把碰撞列当成硬事实，不要把退出码当成「没有碰撞」。
- 墙钟写进表，供票 01「单次 < 60 s」和以后的夜间任务参考。墙钟不参与同 seed 的逐字段比较。
