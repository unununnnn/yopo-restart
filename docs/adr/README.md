# ADR

本目录记录 L0 重启线上已经落地、需要单独可引用的决定。编号接在 vendored 的 [0006](0006-yopo-minco-baseline.md) 之后。0006 正文里提到的 ADR-0011 属于 yopo-swarm，不是本目录的 [0011](0011-brake-gap-horizon-and-amax.md)。

0007、0008 是父代理按 [docs/tickets.md](../tickets.md)「已定」代定的，作用范围只有 L0。0009 的 `stop_at_goal` 半径 1.5 m 已采纳（2026-11-09，开发 bot 按 frank 的 2 分钟规则；frank 以后可以改）。0010 说明 certified 在 L0 上的 `ENV:UNKNOWN` 为什么只进表、不拦合入。0011 是票 13a：11c 的急刹空档来自承诺视界短于一轮窗口，以及计划 `a_max=80` 和 L0 动力学 `a_max=6` 不一致。票 03 之后影子盾会写入逐 tick 拒绝码；0007 末尾有修订。

| ADR | 决定 |
|---|---|
| [0007](0007-l0-default-upstream-shadow.md) | L0 默认 `upstream` + 影子盾开 |
| [0008](0008-report-only-until-frozen.md) | M0–M4 判据冻结前只报数 |
| [0009](0009-stop-at-goal-vs-hold-here.md) | 终端用 `stop_at_goal`，半径 1.5 m 已采纳 |
| [0010](0010-certified-env-unknown-reported.md) | certified 的 `ENV:UNKNOWN` 上报、不作为合入门 |
| [0011](0011-brake-gap-horizon-and-amax.md) | 承诺视界盖住一轮窗口；急刹 a_max 等于 L0 动力学 |

M0 样例表：[docs/results/m0-g2-pillar.md](../results/m0-g2-pillar.md)。
