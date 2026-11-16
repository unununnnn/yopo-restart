# 票 T2：6 机同 seed 高负载不确定性

本票查的是 T1 留下的问题：高负载下 6 机卡同 seed 重跑偶发「不一致」。不改 gates/criteria.frozen，也不把硬断言改成 skip。T1 已经排除后段墙钟预算，见 `t1-load-tolerant-timing.md`。

## 1. 已复现的误报：比较了仓库 `commit` 脏标记

批跑的确定性比较是 `_strip(result)` 加上整段 `ticks`。`_strip` 原先只去掉 `wall_time_s`、`plan_wall_ms`、`batching_plan` 和几何回退的 `check_time_ms`。结果记录里的 `commit` 仍参与比较。

`yopo_l0.runner.git_commit()` 的实现是 `git rev-parse HEAD`，再加上 `git status --porcelain --untracked-files=no`：工作区有已跟踪文件改动时，返回 `sha-dirty`，否则返回 `sha`。这只是仓库快照，不是仿真状态。

同进程里两次 `run()` 之间，只要有别的进程改了本工作区的已跟踪文件，比较就会变成 `sha` 对 `sha-dirty`，从而报「不一致」，而 ticks 和去掉 `commit` 之后的结果仍然相同。

复现（2026-11-16，本工作区，UTC-3）：

1. 对 `formation-6-open` seed 0 跑 0.3 s（`--no-shadow`）。
2. 在两次 `run()` 之间给已跟踪的 `docs/research/INDEX.md` 追加一个换行。
3. 第一次 `commit` 是 `47da396…`，第二次是 `47da396…-dirty`。
4. `ticks` 逐字段相同；`_strip` 之前只在 `commit` 上不同。恢复该文件后仓库回到干净状态。

同一轮调查里还出现了一次未经安排的现场。`scripts/t2-six-drone-det.py` 对 `crossing-6-forest` seed 0 连续跑了 3 对 2.0 s（影子盾开，1 分钟负载大约 4.5–5.3）。前两对一致。第 3 对在 03:13（UTC-3）报不一致：墙钟 17.18 s / 17.67 s，`ticks` 相同，结果里只有 `commit` 从 `47da396…` 变成 `47da396…-dirty`。原因是调查进程正在改已跟踪的 `yopo_l0/batch.py` 和测试，跑批的进程还在用旧的 `_strip`（当时仍比较 `commit`）。这就是「同机有人改已跟踪文件」这条路径，不是轨迹分叉。

这与已有现场吻合：失败都出现在同机很忙的时候（B3 合入前回归、B5 第一次回归时同机在跑 SITL），单独重跑和低负载重跑都通过。B3 的卡是 `formation-6-open` 0.3 s，编队协议不进 YOPO、不读深度，开阔区也没有障碍；B5 的卡 `crossing-6-forest` seed 0 单趟墙钟 234.9 s，中间有足够时间让别的进程改到已跟踪文件。两次历史失败的逐字段差分都没有留下来，不能断言当时就是这一种误报，但这是一条已复现、且足够导致「不一致」的路径。

本票把 `commit` 从 `_strip` 里拿掉。结果记录仍写入 `commit`，只是不再用它判断确定性。失败时把剩余差分写到该 seed 目录的 `det_diff.json`，免得下次再没有现场。硬断言保留。

## 2. 轨迹层面：本票没有复现

T1 还提到过未钉死的 BLAS 线程。本环境里 NumPy 链的是 `scipy-openblas64`（OpenBLAS 0.3.34，默认 8 线程），`torch` 默认也是 8 线程。空载下 512×512 GEMM、以及 8×3×96×160 的 conv2d，1 线程和 8 线程的结果逐位相同。`yopo_l0.runner` 和 `scripts/regress.sh` 仍然没有钉 `OMP_NUM_THREADS`。因为没有看到轨迹差分，本票没有去钉线程，以免在没有证据时改动数值路径。

同 seed 重跑（脚本 `scripts/t2-six-drone-det.py`，比较已去掉墙钟和 `commit`）在 B7 的 SITL 仍占用机器时（1 分钟负载大约 4.9–6.5）：

| 卡 | 时长 | 影子盾 | 对数 | 不一致 |
|---|---:|---|---:|---:|
| formation-6-open seed 0 | 0.3 s | 关 | 38 | 0 |
| formation-6-open seed 0 | 2.0 s | 关 | 4 | 0 |
| crossing-6-forest seed 0 | 2.0 s | 开 | 3 | 1（仅 `commit`，ticks 相同） |

没有叠全时长 50 s 的 `crossing-6-forest` 去抢正在跑的 SITL。若以后再出现「不一致」，先看 `det_diff.json` 再决定要不要钉线程或改比较，不要先拿掉硬断言。

数字在 `t2-six-drone-det-runs.json`。
