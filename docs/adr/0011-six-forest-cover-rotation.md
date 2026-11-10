---
Status: accepted
Date: 2026-11-10
Scope: L0 six-drone forest only
---

# 六机穿林可以把点质量加速度上限提到 18

## Context

票 11b（`0328e24`，PR #18）里，六机楔形穿 medium 林 seeds 0–9 有 6 个 seed 撞树，机间碰撞是 0。同一片 medium 林上三机是 10/10 到达、0 撞树。队形误差 p95 是 17–33 m。

定位见 [docs/results/11c-six-forest.md](../results/11c-six-forest.md)。medium 的 6 次里有 5 次是：承诺的前缀已经结束，参考停在终端保持上，离树大约 1 m，机体比这个保持点超前 0.7–1.3 m，切进树干。目标点不在树里。三机一轮 0.75 s，短于 1.0 s 前缀，终端刹车还没被飞到。六机一轮 1.5 s，点质量 `a_max` 6 m/s² 跟不上这段刹车。

加长前缀、缩短轮转，都会让预留管子重叠，撞树更早。参考停住时改在机体处悬停，会停在预留管子外面，medium seed 0 出现机间碰撞。从机体重接计划，会把原来没撞的 seed 0 改成撞树。这三件都不采用。

## Decision

- L0 默认不变：策略 `upstream`，穿林准入仍是上游 safe_radius 0.05，动力学 `a_max` 仍是场景卡上的 6。
- `task.cover_rotation: true` 时，只把这一次运行的点质量 `a_max` 改成 18。计划、窗口、头状态来源、走廊准入都不改。
- 六机 medium / sparse 林卡打开这个开关。三机卡不加。

## Consequences

- 机体更跟得上终端刹车。medium seeds 0–9 的撞树从 6 降到 3，sparse 从 1 降到 0，机间碰撞仍是 0。剩下的 3 次里，跟踪差已经收到 0.1–0.35 m，计划本身离树只有 0.29–0.54 m。
- 开关不改变预留管子。判据仍未冻结，只报数。
