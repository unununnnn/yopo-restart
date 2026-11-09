---
Status: accepted
Date: 2026-11-09
Scope: L0 only
---

# L0 默认上游等价策略，影子盾开

## Context

[docs/SPEC.md](../SPEC.md) 把「看见闭环」和「证明安全」拆开：L0 默认走 `upstream`（按 YOPO-MINCO 规则选候选），现行安全证明链以影子盾只记录、不改动作。同一节把「L0 默认 upstream + 影子盾」列为需要 frank 批准的决定，作用范围是 L0 和显式选择的 L1 抽检。L1 的安全清单数值不动。

2026-11-09 frank 的拍板卡片未回。父代理代定（见 [docs/tickets.md](../tickets.md)），只覆盖 L0：

1. L0 默认 `upstream`，影子盾开。`--policy` / `--no-shadow` 可以覆盖。
2. 影子盾开关写入结果记录。逐 tick 拒绝码留给票 03。

票 02 已按此落地：`RunConfig.policy` 默认 `"upstream"`，`RunConfig.shadow` 默认 `True`。结果记录带 `tier=L0`、`policy`、`shadow`。`shadow_reject_rate` 仍是 `null`。

## Decision

- L0 运行器的默认策略是 `upstream`，影子盾开关默认开。
- 一条运行用 `--policy hover|upstream|certified` 换策略，用 `--no-shadow` 关掉影子盾。两次覆盖都写进该次结果记录。
- 影子盾在票 03 之前只占结果里的开关位，不改变动作，也不产生拒绝率。`upstream` 路径本身不经过 SafetyGate。
- L1、vendored `core/` 的 certified 门、安全清单数值都不因本决定改动。

## Consequences

- 不带 `@策略` 的批跑（含 `scripts/regress.sh` 里的 `scenes/g2-pillar.json`）就是 upstream + 影子盾开。要看现行全部门，必须显式 `@certified`。
- 结果表上的「影子盾 开」表示开关为开，不表示已经有逐 tick 拒绝码。拒绝率表是票 03 的交付。
- 同 seed 下影子盾开/关的动作必须逐位一致。这条在票 03 实现记录时验收；本 ADR 不把未实现的记录算作已完成。
