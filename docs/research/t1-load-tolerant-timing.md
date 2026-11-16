# 票 T1：计时测试在高负载下可过

本票改了一处会误杀合入的墙钟断言，并查了另一处高负载下偶发的同 seed 不一致。后一项没有改硬断言。不改后段迭代上限逻辑，也不改 gates/criteria.frozen、scenes/suite.txt。

## 1. `test_iteration_cap_and_time_budget`

`tests/test_backend_bspline.py::test_iteration_cap_and_time_budget` 以前断言 `cut["time_ms"] < 50.0`。`time_ms` 是 `refine_yopo` 的墙钟。#144（B2）第一次合入时，同机 SITL 把负载抬到约 13，这一项失败，尽管预算已经按评估次数截断。

缺省 `budget` 是 `"evals"`。测试里 `time_budget_s=1e-4` 按 1 ms/次折成 1 次评估（下限）。截断是否生效，看 `budget_cut` 和 `iterations <= max_iterations`，不看墙钟。墙钟预算由 `tests/test_l0_det_budget.py` 用假时钟覆盖，不在本测试设紧的墙钟上限。

本票没有把 50 ms 放宽成一个更大的墙钟阈值，也没有改 `yopo_l0/backend` 的截断逻辑。

## 2. 六机同 seed 重跑在高负载下偶发不一致

证据：

- B3 合入前回归：`tests/test_l0_six_batch.py::test_repeated_seed_stays_deterministic_with_tier_metadata` 对 `formation-6-open` 跑 0.3 s，同 seed 重跑报「不一致」。本票未改这张卡。单独跑、`-n 4` 跑整个文件都过；随后整套 L0 重跑也过。见 `b3-sitl-launch-tracking.md`。
- B5 第一次 `scripts/regress.sh`（2026-11-15 21:18–21:38，UTC-3，`out/regress/20261115T211832`）：批跑唯一失败是 `crossing-6-forest` seed 0 同 seed 重跑「不一致」。当时同机在跑 SITL，该卡单趟墙钟 234.9 s。21:42 低负载下单独重跑：一致。第二次回归（21:53–22:16）批跑通过。见 `b5-a-rerun-ramp-ref.md`。那两次失败的逐字段差分没有留下来。

先查是不是走了后段墙钟预算才不确定。结论：**不是。**

| 项 | formation-6-open（B3 的短时测试） | crossing-6-forest seed 0（B5 批跑） |
|---|---|---|
| 后段 `policy_params.backend` | 未写，保持关 | 未写，保持关 |
| 几何回退 | 未写，保持关 | 未写，保持关 |
| 缺省 `budget` | `"evals"`，本卡用不上 | `"evals"`，本卡用不上 |
| 影子盾 | 测试里 `--no-shadow` | 套件默认开 |
| 确定性比较已去掉的字段 | `wall_time_s`、`plan_wall_ms`、`batching_plan`、几何回退的 `check_time_ms` | 同左 |
| 另外比较的 `ticks` | 仿真状态（位置、计划码等），不含墙钟 | 同左 |

后段只有在 `budget="wall"`（或 `YOPO_BACKEND_BUDGET=wall`）时才按真墙钟截断求解，从而让轨迹随负载变。这两张卡后段是关的，走不到那条路径。L0-DET 把仿真缺省改成评估次数，正是为了避免这种不确定。

因此本票没有拿掉 `test_repeated_seed_stays_deterministic_with_tier_metadata` 的硬断言，也没有放宽批跑的确定性检查。只加了 `test_six_drone_det_cards_do_not_use_wall_budget`，避免以后误把原因安到墙钟预算上。

还没查清的是：去掉墙钟字段之后，轨迹或摘要为什么仍会在高负载下和低负载下不一样。失败现场的逐字段差分没有留下，本票没有在高负载下复现（写这篇时 #151 的 premerge 还在跑，没有叠全量回归去抢机器）。可能的方向包括进程内未钉死的 BLAS 线程数（不少研究脚本会把 `OMP_NUM_THREADS` / `OPENBLAS_NUM_THREADS` / `MKL_NUM_THREADS` 设成 1，`yopo_l0.runner` 和 `scripts/regress.sh` 没有），但没有差分就不能当结论。以后再遇到，应先把 `_strip` 前后不相同的键留下来，再决定要不要改比较或钉线程，不要先把硬断言拿掉。
