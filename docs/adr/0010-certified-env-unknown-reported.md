---
Status: accepted
Date: 2026-11-09
Scope: L0 certified reporting
---

# certified 在 L0 上的 `ENV:UNKNOWN` 只上报，不作为合入门

## Context

诊断和 [docs/SPEC.md](../SPEC.md) 的问题陈述：G2 单机绕柱在 PX4 闭环里一次都没飞成，主因是项目自己的安全证明门让这个场景无解。L0 的 `certified` 策略把这条门原样留下来当对照，参数与 L1 G2 相同（机体半径 0.35 m、跟踪裕度 0.15 m、`switch_delay` 2 s、起飞区半径 1.2 m），走 vendored `UavRuntime` + `Planner` + `SafetyGate`。

自由空间的规则不改：扫掠样本只有被深度证明为 FREE 才放行。球必须在相机前方、超出最小量程、投影包围盒整个在画面内、盒内像素有效且深度远于球的远端，才是 FREE；否则是 UNKNOWN。UNKNOWN 不当成 FREE。证书只覆盖已经批准过的扫掠和起飞区假设。

L0 的前视光线投射给不了这份证明。机身周围、相机后方、最小量程以内、视场外的样本都是 UNKNOWN。起飞区假设只盖住起点附近 1.2 m。候选轨迹一离开起飞区，扫掠就是 `ENV:UNKNOWN`，漏斗诊断是 `NO_ENV_SAFE_CANDIDATE`，运行时继续执行已经批准的初始保持。飞机留在起点。

这是门在拒绝，不是运行器把未知空间当成可飞。票 02 的代码（`036d90761c13beb2ad5574fc685af5f138b996e3`）上，`certified`、影子盾开、场景卡全时长 45 s、10 个 seed 的测量与诊断一致：到达 0/10，碰撞 0，结果都是 `TIMEOUT`，终点误差 5 m，最小障碍距离 1.5 m（停在起点）。每个 seed 的规划码相同：`BOOTSTRAP` 1、`WAIT_SWITCH` 20、`NO_ENV_SAFE_CANDIDATE` 429、`cand:ENV:UNKNOWN` 5148。十个 seed 的候选拒绝合计 `ENV:UNKNOWN` 51480，没有 `ENV:OCCUPIED` 或其他候选码。批跑退出码是 0。全表见 [M0 样例](../results/m0-g2-pillar.md)。

这组结果记录里的 `commit` 带 `-dirty`：跑的时候工作区里已跟踪的改动只有本分支的 README 链接，策略、场景卡和 `core/` 都没有改。upstream 主表是在那次 README 改动之前、干净工作区上跑的，`commit` 没有后缀。缩短的 seed 0、仿真 8 s（同一干净 commit、影子盾开）已经是同一形态的前 8 秒：`TIMEOUT`、到达 0、终点误差 5.0 m、最小障碍距离 1.5 m，规划码 `BOOTSTRAP` 1、`WAIT_SWITCH` 20、`NO_ENV_SAFE_CANDIDATE` 59、`cand:ENV:UNKNOWN` 708。

`scripts/regress.sh` 里的 certified 冒烟是另一个东西：单 seed、仿真 4 s、`--no-shadow`。4 s 里约一半还在等 `switch_delay`，不能拿它代替 10 seed 全时长。

## Decision

- L0 上 `certified` 的 `ENV:UNKNOWN` / `NO_ENV_SAFE_CANDIDATE` / 到达 0 写进 M0 表，不作为合 main 的失败，也不作为 `yopo_l0.batch` 或 `scripts/regress.sh` 的非零退出。
- 门本身继续拦截。UNKNOWN 候选不飞。不把 UNKNOWN 改成 FREE，不放宽 SafetyGate，不改 L0 深度适配去换一次到达。那样做会毁掉这组对照。
- 运行器只在真值碰撞时中止回合。`ENV:UNKNOWN` 累加到 `plan_codes`（`cand:` 前缀来自 certified 漏斗），回合跑到到达或超时。goto 在判据冻结前不影响退出码，见 [0008](0008-report-only-until-frozen.md)。
- 这样做是因为诊断已经预期对照臂到不了：全部门让 G2 无解。飞行臂是 `upstream`（终端规则见 [0009](0009-stop-at-goal-vs-hold-here.md)）。用对照臂的 0/10 去拦合入，等于把 main 再次冻在「门使场景无解」上。
- 这些 `cand:ENV:UNKNOWN` 计数是 certified 策略自己的漏斗，不是影子盾。影子盾开关按 [0007](0007-l0-default-upstream-shadow.md) 照常写入结果；逐 tick 影子拒绝率仍是票 03，`shadow_reject_rate` 保持 `null`。

## Consequences

- 回归为绿且 certified 一栏是 0/10、`ENV:UNKNOWN`，是符合本 ADR 的结果。
- 读表时不要把 certified 的到达 0 当成 L0 动力学或网络坏了。同一 commit 上 `upstream` + `stop_at_goal` 是 10/10、碰撞 0。
- 票 03 的影子拒绝率是另一列：upstream 动作不变，旁边记下「若开硬门会拒多少」。那一列也不在冻结前变成合入门。
