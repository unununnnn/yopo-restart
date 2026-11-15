# 票 S5b：三机编队保持的 PX4 SITL 实飞复验（只报数）

S5a 已经把 `scripts/sitl/yopo_l0_live.py --policy formation` 接到 `scenes/formation-hold-sitl80.json`，并留下 L0 同卡基线。本票在 box 上真跑 PX4 SITL，和同一张卡的 L0 对照。判据未冻结。SITL 的 `result.json` 仍是 `report_only=true`、`criteria_frozen=false`。下面的数字只报数，不拿 L0 gates 当 SITL 通过条件。

不改 `core/`、`yopo_l0/formation.py`、`yopo_l0/bus.py`、`scenes/suite.txt`、`scenes/formation-hold.json`，也不写 PX4 的 `COM_` / `NAV_` / `BAT_` 参数。本票没有改 `scripts/sitl/`：复验入口沿用 S5a，默认行为不变。

## 场景卡与入口

`scenes/formation-hold-sitl80.json` 与 `scenes/formation-hold.json` 只有名字、`task.duration_s`（40 → 80）和一句 note 不同。队形、槽位和 `criteria` 相同。不进 `scenes/suite.txt`。80 s 是任务钟上限，含起飞和降落。

```bash
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/formation-hold-sitl80.json --policy formation --seed 0
```

L0 对照是 `scenes/formation-hold.json`、同一组 seed、`RunConfig(policy="formation")`（影子盾开，零故障）。仓库里 `python -m yopo_l0.run --policy` 的选项没有 `formation`，编队由场景卡 `task.type` 走 `FormationFleet`。本票按票面调用 `RunConfig`，不改 CLI。

SITL 到达和 L0 比较时，用任务钟到达减去 `formation_t0_s`（全体进入 PLAN 的那一拍，编队钟记 0）。`takeoff_handover_s` 是各机交接的最大值，比 `formation_t0_s` 早一拍（0.02 s）。`result.json` 里的 `arrival_after_handover_s` 是各机「本机到达 − 本机交接」的最大值；某机先交接时，这个值会大于「任务钟到达 − formation_t0_s」，不能直接拿来和 L0 比。

定位 `default`。`px4_parameters_written` 全程是空列表。SITL 编队入口不调用 `UpstreamPolicy`，`shadow` 为 false，没有影子拒绝率。

## 环境

box 镜像沿用 S6 重建的标签（`/workspace/yopo-ops/setup-sitl-box.sh`，本票未再重建）：

| 标签 | 角色 |
|---|---|
| `yopo-restart-sitl:s1b-23bc262f44d8` | S1b 基础镜像（PX4 v1.17.0 SITL + Gazebo Harmonic） |
| `yopo-restart-sitl:yopo-81d084c04186` | S4c YOPO L0 分层 |

L0 在 2026-11-15 09:23:01–09:23:28 UTC-3 跑完，环境是 `/workspace/ys-venv`（Python 3.12，numpy 2.5.4，torch 2.4.1+cpu），`PYTHONPATH=.:core/src`，提交 `765e2cb`。SITL 在 09:23:41–09:29:05 UTC-3 串行跑完，seeds 0–4 全部退出码 0。

## 对照表（seeds 0–4）

L0 五个 seed 的轨迹数字完全相同。SITL 五次都是 `SUCCESS` / `GOAL_REACHED_AND_HELD`，3/3 到达并落地，碰撞 0。

| seed | L0 到达 | SITL 到达 | L0 碰撞 | SITL 碰撞 | L0 队形 p95 / m | SITL 队形 p95 / m | L0 最小机间 / m | SITL 最小机间 / m | L0 到达 / s | SITL 任务钟到达 / s | SITL formation_t0_s / s | SITL 交接后到达 / s | SITL 落地 | SITL RTF | SITL px4_parameters_written |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---|
| 0 | 3/3 | 3/3 | 0 | 0 | 0.027 | 0.320 | 1.987 | 1.870 | 18.63 | 28.876 | 9.876 | 19.000 | 是 | 0.994 | `[]` |
| 1 | 3/3 | 3/3 | 0 | 0 | 0.027 | 0.359 | 1.987 | 1.880 | 18.63 | 29.132 | 10.148 | 18.984 | 是 | 0.997 | `[]` |
| 2 | 3/3 | 3/3 | 0 | 0 | 0.027 | 0.310 | 1.987 | 1.894 | 18.63 | 28.832 | 9.964 | 18.868 | 是 | 0.998 | `[]` |
| 3 | 3/3 | 3/3 | 0 | 0 | 0.027 | 0.361 | 1.987 | 1.916 | 18.63 | 28.528 | 9.684 | 18.844 | 是 | 0.997 | `[]` |
| 4 | 3/3 | 3/3 | 0 | 0 | 0.027 | 0.350 | 1.987 | 1.881 | 18.63 | 28.996 | 10.092 | 18.904 | 是 | 0.999 | `[]` |

汇总：

| 项 | L0 | SITL |
|---|---|---|
| 到达 | 15/15 | 15/15，全部落地 |
| 碰撞 | 0 | 0 |
| 到达时刻 | 全部 18.63 s | 任务钟 28.528–29.132 s（含起飞）；`formation_t0_s` 9.684–10.148 s；交接后 18.844–19.000 s（均值 18.920 s） |
| 队形 p95 | 0.027 m | 0.310–0.361 m |
| 最小机间 | 1.987 m | 1.870–1.916 m |
| 落地 | 不降落 | 5/5 落地 |
| RTF | 点质量墙钟比约 3.2–3.8，不是锁步 | 0.994–0.999 |
| `px4_parameters_written` | 不适用 | 全程 `[]` |

去掉起飞交接之后，SITL 到达比 L0 的 18.63 s 慢约 0.21–0.37 s。队形 p95 从 0.027 m 升到 0.31–0.36 m，最小机间从 1.987 m 降到 1.870–1.916 m。这是 PX4 跟踪和点质量不是同一套动力学，只报数，不是失败门。

seed 3 的 `arrival_after_handover_s` 是 20.332 s，因为 uav2 在 8.196 s 先交接；按编队钟零点算仍是 18.844 s。原始结果和逐机交接时刻在 [s5b-sitl-formation-verify-runs.json](s5b-sitl-formation-verify-runs.json)。SITL 产物目录在 `/workspace/yopo-ops/sitl/runs/20261115T092341-yopo-n3` 起的 5 趟。

## 本环境实测

```bash
PYTHONPATH=".:core/src" .venv/bin/python -m pytest tests/test_sitl_s5b_formation_verify.py tests/test_sitl*.py -q --tb=line
bash scripts/sitl/run_yopo_l0_sitl.sh --dry-run --n 3 --scene scenes/formation-hold-sitl80.json --policy formation
```

| 命令 | 结果 |
|---|---|
| `pytest tests/test_sitl*.py -q --tb=line` | **99 passed** in 41.10 s |
| 上面的 shell dry-run | 退出码 0，`policy=formation`，`scene=scenes/formation-hold-sitl80.json`，镜像 `yopo-restart-sitl:yopo-81d084c04186` |
| L0 seeds 0–4 | 5/5 `SUCCESS`，退出码 0，数字与 S5a seed 0 基线一致 |
| SITL seeds 0–4 | 5/5 `SUCCESS`，退出码 0，全部落地，`px4_parameters_written=[]` |
| `scripts/regress.sh` | L0 pytest **852 passed / 27 skipped**，core pytest **453 passed / 3 skipped**，批跑通过；总计 1384.939 s |

`tests/test_sitl_s5b_formation_verify.py` 检查：对照卡只改了名字、时长和 note；dry-run 打印 `policy=formation` 且 `px4_parameters_written` 为空；`s5b-sitl-formation-verify-runs.json` 是 report-only，SITL 各行都不写 PX4 参数。没有在单元测试里拉起 Gazebo。
