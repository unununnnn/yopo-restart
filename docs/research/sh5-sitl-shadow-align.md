# 票 SH5：SITL 重测影子对齐开关（只报数）

只报数。report-only。判据未冻结。不改 `core/`、`scenes/suite.txt`、已冻门、安全清单和 PX4 参数。影子盾仍然只记录，不改设定点。三个研究开关 `shadow_env_evidence`、`shadow_contract_align`、`shadow_tracking_align` 默认仍关；打开的结果标「偏离上游」。不放宽 `FlightOracle.tracking_bound`（0.15 m）。墙钟不进结论。

数字在 [sh5-sitl-shadow-align-runs.json](sh5-sitl-shadow-align-runs.json)。

## 结论

（交叉卡 seeds 0–4 跑完后填最终合计。下面方法与开关口径已定。）

背景：票 SH2b 在锁步交叉卡上打开环境证据后，环境拒绝率 0.931→0.013，`would_reject` 仍是 0.997，开臂 9759 tick 里有 6445 次 `TRACKING_BOUND`。票 SH3 在 L0 上把契约码从 `would_reject` 里拿掉，g2-pillar / crossing-pillars 的会拒率降到 0.0114 / 0.0176，但还没有在 SITL 上重测对齐开关。票 SH4 确认把 `TRACKING_BOUND` 当成计划危险是误报，并加上 `shadow_tracking_align`；当时没有新跑 SITL。

本票在隔离网络的锁步 SITL 上对照四臂，报 `would_reject`、环境拒绝率、`TRACKING_BOUND` 次数、到达与碰撞。

归因修正（本票）：`shadow_sitl.attribution` 在 tick 已有 `would_reject` 字段时改看该字段，不再「有拒绝码就拒」。否则打开 `shadow_tracking_align` 后，仍记在码表里的 `TRACKING_BOUND` 会被打回 `would_reject`。`shadow_reject_rate`（`shadow.aggregate`）本来就只看 `tick.would_reject`，与修正后一致。默认关时行为不变。

## 方法

场景卡至少 `scenes/crossing-pillars-sitl80.json`，policy `crossing`，seeds 0–4。入口是 `scripts/sitl/run_yopo_l0_sitl.sh --lockstep --fleet-shadow`，用官方脚本的默认隔离网络，镜像 `yopo-restart-sitl:yopo-81d084c04186`。不写 PX4 参数。

四臂都开 `fleet_shadow`，以便记下影子归因（与票 SH2 关臂相同）。`fleet_shadow` 本身不写 `upstream_deviation`。三个研究开关按臂打开：

| 臂 | `shadow_env_evidence` | `shadow_contract_align` | `shadow_tracking_align` |
|---|---|---|---|
| default | 关 | 关 | 关 |
| env | 开 | 关 | 关 |
| env_contract | 开 | 开 | 关 |
| env_contract_tracking | 开 | 开 | 开 |

打开契约对齐时，仍会从返回码里拿掉 `TRACKING_BOUND`（保持票 SH3 的行为）。再打开跟踪对齐后，0.15 m 超界留在直方图里，但不单独打满 `would_reject`。阈值本身不改。

## 锁步 SITL：crossing-pillars-sitl80 seeds 0–4

（待填逐趟与合计。种子 0–2 已跑完并推送；种子 3–4 等合入队列清空后继续。）

## pytest

（回归数字待填。）新增 `tests/test_sitl_sh5_shadow_align.py`：三个研究开关默认关、0.15 m 阈值不变、官方启动脚本四臂 dry-run 与本票命令行一致、汇总行会算 `would_reject` / 环境拒绝率 / `TRACKING_BOUND` 次数；归因在跟踪对齐下尊重 `tick.would_reject`。
