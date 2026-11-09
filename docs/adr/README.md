# ADR

本目录记录 L0 重启线上已经落地、需要单独可引用的决定。编号接在 vendored 的 [0006](0006-yopo-minco-baseline.md) 之后；0006 里提到的 ADR-0011 等属于 yopo-swarm，不在本仓库。

0007、0008 是父代理按 [docs/tickets.md](../tickets.md)「已定」代定的，作用范围只有 L0。0009 的 `stop_at_goal` 半径 1.5 m 已采纳（2026-11-09，开发 bot 按 frank 的 2 分钟规则；frank 以后可以改）。0010 说明 certified 在 L0 上的 `ENV:UNKNOWN` 为什么只进表、不拦合入。票 03 之后影子盾会写入逐 tick 拒绝码；0007 末尾有修订。

| ADR | 决定 |
|---|---|
| [0007](0007-l0-default-upstream-shadow.md) | L0 默认 `upstream` + 影子盾开 |
| [0008](0008-report-only-until-frozen.md) | M0–M4 判据冻结前只报数 |
| [0009](0009-stop-at-goal-vs-hold-here.md) | 终端用 `stop_at_goal`，半径 1.5 m 已采纳 |
| [0010](0010-certified-env-unknown-reported.md) | certified 的 `ENV:UNKNOWN` 上报、不作为合入门 |

M0 样例表：[docs/results/m0-g2-pillar.md](../results/m0-g2-pillar.md)。
