# 票 S5a：SITL 实飞入口跑 formation-hold（只报数）

依赖票 S4e（PR #120，分支 `yopo-restart-bot/s4e-sitl-fleet-frames`，尚未进 main）。S4e 已经让 PX4 SITL 三机在 `scenes/open-3drone-sitl40.json` 上和 L0 对齐。本票把同一条实飞入口接上编队策略，先飞已有的三机编队卡。判据未冻结。SITL 的 `result.json` 仍是 `report_only=true`、`criteria_frozen=false`。下面的 L0 数字是同卡基线；box 上的 SITL 数字等复验再填。

不改 `core/`、`yopo_l0/formation.py`、`yopo_l0/bus.py`、`scenes/suite.txt`、`scenes/formation-hold.json`，也不写 PX4 的 `COM_` / `NAV_` / `BAT_` 参数。

## 接法

`scripts/sitl/yopo_l0_live.py --policy formation` 和 `LiveFleetSession` 在 `policy=formation` 时不再调用 `UpstreamPolicy`。PLAN 阶段用场景卡上的 `FormationFleet`：`exchange` 走进程内 `MessageBus`（与 `yopo_l0.runner` 同一个总线盐 `0xB05`，零故障时本拍投递）。不发布协议话题，不设 `FormationFleet.transport_override`。

编队钟从全体进入 PLAN 的那一拍记 0。起飞、解锁不计入 `t_go`（formation-hold 的 `t_go` 仍是 1.5 s）。仿真钟一次跳过多个 `physics_dt` 时，按 0.02 s 把协议补拍，避免整窗提案被跳过；设定点只在本拍的编队钟上采样一次。心跳仍是 S4e 那条：PLAN 阶段按仿真钟沿已提交计划往前采样，速度和加速度保留，不调用 `sample()`，所以 20 Hz 心跳不会把航向多转一拍。其他阶段只守位置和航向。

到达截住沿用场景卡口径：距编队终点槽位 ≤ 0.4 m、速度 ≤ 0.2 m/s、连续保持 1 s。全体到达后发 `MAV_CMD_NAV_LAND`（命令号 21）。编队 PLAN 超时从进入 PLAN 起算 60 s（上游缺省仍是 45 s），不含起飞。

SITL 步进是 50 Hz。L0 物理步是 100 Hz。协议按时间走，不按拍数走。队形误差只在编队钟上对实测位置记，起飞段不进 p95。

## SITL 卡

`scenes/formation-hold-sitl80.json` 与 `scenes/formation-hold.json` 只有名字、`task.duration_s`（40 → 80）和一句 note 不同。队形、槽位、`criteria`（含 gates）相同。不进 `scenes/suite.txt`。

80 s 是任务钟上限，含起飞。S4e 在 `open-3drone-sitl40` 上的到达约 27–29 s，同任务 L0 是 3.68 s，起飞和预热大约吃掉 24 s。本卡 L0 到达 18.63 s，加上这段起飞和落地余量，40 s 的原时长盖不住。PLAN 阶段本身仍按编队时刻表走，大约 19 s。

## L0 同卡基线

`scenes/formation-hold.json`，seed 0，`RunConfig(policy="formation")`（影子盾开，零故障）。本环境 `/tmp/yopo-venv`（Python 3.12，numpy 2.5.4，torch 2.4.1+cpu），`PYTHONPATH=.:core/src`。

| 项 | 值 |
|---|---|
| 结果 | `SUCCESS` / `GOAL_REACHED_AND_HELD` |
| 到达 | 3/3，到达率 1.0，到达时刻 18.63 s |
| 队形 p95 | 0.02698217847566754 m |
| 队形最大误差 | 0.04098926463108861 m |
| 最小机间 | 1.9865290876661743 m |
| 终点目标误差 | 0.010180195825787185 m |
| 提案 / ACK / NACK | 3 / 6 / 0，每机 1 次 commit |
| `t_go` / 运动时长 | 1.5 s / 18.75 s |

终点槽位（ENU，dry-run 用同一份 `FormationFleet.goal_of`）：uav0 `[10.884254957696518, 10.884254957698905, 1.5]`，uav1 `[8.884254957696518, 10.884254957698905, 1.5]`，uav2 `[12.884254957696518, 10.884254957698905, 1.5]`。

## box 复验

云端没有 Docker / Gazebo，下面这条在 box 上跑。镜像沿用 S4e 那层带 YOPO 的标签（本环境 dry-run 只能看到基础镜像 `yopo-restart-sitl:s1b-23bc262f44d8`）。

```bash
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/formation-hold-sitl80.json --policy formation
```

只看话题、不连 ROS：

```bash
scripts/sitl/run_yopo_l0_sitl.sh --dry-run --n 3 --scene scenes/formation-hold-sitl80.json --policy formation
PYTHONPATH=".:core/src" .venv/bin/python scripts/sitl/yopo_l0_live.py --dry-run \
  --scene scenes/formation-hold-sitl80.json --n 3 --policy formation --out /tmp/s5a-dry.json
```

`--policy` 缺省是 `upstream`。编队卡不带 `--policy formation` 会退出 2。`--dry-run` 打印 `policy=formation`，`px4_parameters_written` 是空列表，话题里没有协议名。机间消息不进 ROS。

复验时看 `result.json`：`outcome`、`arrived`、`formation_error_p95_m`、`min_inter_drone_distance_m`、`formation_t0_s`（编队钟零点在任务钟上的位置；任务钟到达减去它，才和 L0 的 18.63 s 比）、`landed`、`px4_parameters_written`。本票不拿 L0 的 gates 当 SITL 的通过条件。

## 本环境实测

```bash
PYTHONPATH=".:core/src" /tmp/yopo-venv/bin/python -m pytest tests/test_sitl_s5a_formation.py tests/test_sitl*.py -q --tb=line
PYTHONPATH=".:core/src" /tmp/yopo-venv/bin/python -m pytest -n 4 -q --tb=line
bash scripts/sitl/run_yopo_l0_sitl.sh --dry-run --n 3 --scene scenes/formation-hold-sitl80.json --policy formation
```

| 命令 | 结果 |
|---|---|
| `pytest tests/test_sitl_s5a_formation.py tests/test_sitl*.py -q --tb=line` | **89 passed** in 36.64s |
| `pytest -n 4 -q --tb=line` | **763 passed, 28 skipped** in 421.45s |
| 上面的 shell dry-run | 退出码 0，`policy=formation` |

`tests/test_sitl_s5a_formation.py` 用假 PX4 走命令序列：同步出发前设定点停在起点；编队钟 6 s 时三机都已 commit，设定点沿直线段离开起点，跳步和 0.02 s 连拍的位置一致；心跳采样更靠前的时刻并保留速度；假 PX4 积分 8 s，直线段最小机间大于 1.5 m，队形 p95 小于 0.5 m；停在终点槽位满 1 s 后发 `command=21`，上锁后 `SUCCESS`。没有拉起 PX4 / Gazebo。
