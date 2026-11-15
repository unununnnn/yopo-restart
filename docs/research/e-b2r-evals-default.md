# 票 E-B2r：后段评估预算缺省 10→20

frank 拍板（2026-11-15，两分钟规则）。依据是票 E-B2q（PR #136，当时未合入；数字在分支 `yopo-restart-bot/e-b2q-evals-diag` 的 `docs/research/e-b2q-evals-diag.md`）：密林 10 次是 12/30，362 次调用里 130 次截断；20 次是 21/30，0 次截断；40 次与 20 次逐种子相同。014 对向在 E-A1 和 D1 固定时，20 次是 20/30，0 次截断，到齐种子与旧 combo 相同。

本票只改打开后段之后的缺省评估次数。研究开关仍默认关：`policy_params.backend.enabled` 不是 JSON true 时，`config_from_params` 返回 None，不读几何，也不拟合。`budget` 缺省仍是 `"evals"`，不改回墙钟。`max_iterations` 仍是 8。

`budget="evals"` 下 0.020 s 对应 20 次。名义每次评估 1 ms（`yopo_l0/backend/bspline.py` 的 `_IDLE_EVAL_COST_S` 从 0.002 改成 0.001）。这不是重新测过的墙钟。L0-DET 量到的大约 2 ms 仍是当时把 0.020 s 折成 10 次的依据；E-B2q 说明 10 次会截断还没跑完的 8 次迭代，20 次已经饱和。

调用者写上的 `time_budget_s` 按同一比例折算，不会被改成 20。显式 0.010 s 仍是 10 次，0.040 s 是 40 次，1e-4 s 仍是下限 1 次，5.0 s 从 2500 次变成 5000 次。要回旧的 10 次，显式传 0.010 s，不靠缺省。

不改 `core/`、场景卡、`scenes/suite.txt`、`criteria.frozen`、README、`scripts/premerge.sh`、`scripts/regress.sh`、`scripts/sitl/Dockerfile`、`docs/tickets.md`。旧研究文档里按 10 次报的数字不改。那些文档写的是当时的缺省。

数字在 [e-b2r-evals-default-runs.json](e-b2r-evals-default-runs.json)。跑数时 HEAD 是 `b7a5477`，工作区已经改了上面的缺省，尚未提交。干净的 `b7a5477` 仍是 10 次。`shadow_env_evidence` 没有打开。三个以上的进程，每个进程一个 OpenMP 线程。墙钟不参与轨迹。

## 冻结套件

`scenes/suite.txt` 里的卡都不写 `backend.enabled`。`docs/results/SUMMARY.json` 的场景是这些卡的子集：`g2-pillar`、`formation-hold`、`crossing-pillars`、`crossing-forest`、`formation-switch`、`formation-6-open`、`crossing-6-forest`、`crossing-6-forest-sparse`。逐张加载后 `config_from_params` 都是 None，运行器不调用 `refine_yopo`，评估次数上限读不到。两张打开后段的研究卡（`g2-pillar-bspline.json`、`crossing-forest-dense-bspline.json`）不在套件里，`criteria.frozen` 是 false。

所以这次缺省不改变 SUMMARY。`--check` 干跑核对了这件事，也核对了缺省是 20 次、显式 0.010 s 仍是 10 次，并且没有改写套件、SUMMARY、`core/` 和场景卡文件。

## 重跑

用新缺省，两张卡都没有写 `time_budget_s`，也没有把 `budget` 设成 `wall`。密林是 `scenes/crossing-forest-dense-bspline.json` 上已经打开的后段，种子 0–9，影子盾开。014 是 `scenes/ego-density-014-swap3.json`，E-A1（滤掉、阈值 0.70 m、惩罚 0）和 D1（`geometric_fallback`，机间 0.7 m，视界 1.0 s）按 `scripts/e-c4-density-l0.py` 的 combo 盖上，后段只写 `enabled=true`。打开后段标 report-only / 偏离上游，不是新的默认开关。

净空 = 机体中心到障碍表面的距离 − 0.35 m。撞树按运行器真值：中心距 ≤ 0.35 m 记 1。机间碰撞是中心距 ≤ 0.70 m。截断是这一次 `refine_yopo` 因为评估次数上限停下（`budget_cut`），不是仿真超时。

| 场景 | 到达 | 撞树 | 机间 | 超时 | 最小净空 m | 迭代 p50（均值） | 截断 | 单次评估最多 |
|---|---|---|---|---|---|---|---|---|
| 密林 | 21/30 | 3 | 0 | 0 | −0.007017 | 8（8.00） | 0/418 | 13 |
| 014 对向 | 20/30 | 2 | 2 | 0 | −0.000760 | 8（7.98） | 0/1471 | 15 |

与 E-B2q 的 20 次臂对照：到达、撞树、机间、调用次数、截断、单次评估最多，以及下面逐种子的到达、碰撞原因和 `cuts/calls`，都相同。40 次那一列 E-B2q 已经写过与 20 次逐种子相同，这里没有再跑。

### 密林

| seed | 到达 | 原因 | 截断 | 单次评估最多 |
|---|---|---|---|---|
| 0 | 0/3 | 撞树 uav1 | 0/34 | 11 |
| 1 | 3/3 | 到达，净空 0.0047 m | 0/47 | 12 |
| 2 | 3/3 | 到达 | 0/47 | 13 |
| 3 | 3/3 | 到达 | 0/45 | 12 |
| 4 | 0/3 | 撞树 uav0 | 0/28 | 12 |
| 5 | 0/3 | 撞树 uav0 | 0/28 | 11 |
| 6 | 3/3 | 到达 | 0/46 | 13 |
| 7 | 3/3 | 到达 | 0/47 | 12 |
| 8 | 3/3 | 到达 | 0/48 | 12 |
| 9 | 3/3 | 到达 | 0/48 | 11 |

机间碰撞是 0。超时是 0。撞树仍是 seed 0、4、5。seed 1 的净空是 0.004687 m。

### 014 对向

| seed | 到达 | 原因 | 截断 | 单次评估最多 |
|---|---|---|---|---|
| 0 | 3/3 | 到达 | 0/242 | 12 |
| 1 | 0/3 | 机间 uav1-uav2 | 0/84 | 12 |
| 2 | 3/3 | 到达 | 0/152 | 13 |
| 3 | 3/3 | 到达 | 0/153 | 12 |
| 4 | 3/3 | 到达 | 0/174 | 15 |
| 5 | 0/3 | 机间 uav1-uav2 | 0/84 | 12 |
| 6 | 3/3 | 到达 | 0/156 | 14 |
| 7 | 2/3 | 撞树 uav0 | 0/194 | 13 |
| 8 | 3/3 | 到达 | 0/151 | 14 |
| 9 | 0/3 | 撞树 uav0 | 0/81 | 15 |

到齐的是 seed 0、2、3、4、6、8。超时是 0。截断是 0。

## 复现

```bash
make venv
git rev-parse HEAD
.venv/bin/python scripts/e-b2r-evals-default.py --check
# 不要设 YOPO_BACKEND_BUDGET=wall。已有 partial 会接着跑。
.venv/bin/python scripts/e-b2r-evals-default.py --jobs 4
```

`--check` 不跑仿真。

```bash
PYTHONPATH=.:core/src .venv/bin/python -m pytest tests -q -n 4 -m "not slow"
```

这条命令在缺省改动提交之后的结果是 856 passed，28 skipped。随后 main 合入了 E-B2q 和 D6e。E-B2q 的干跑原来把缺省锁成 10 次，并把 0.040 s / 0.080 s 当成 20 / 40 次。诊断各臂改成显式秒数（6 次 0.006 s，10 次 0.010 s，20 次 0.020 s，40 次 0.040 s），缺省断言改成 20。`docs/research/e-b2q-evals-diag.md` 里按 10 次报的数字没有改。
