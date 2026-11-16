# 票 BAT1：锁步电池 unhealthy（不改 `BAT_` 参数，只诊断）

本票查 B1 第二版锁步交叉里出现的 `Battery unhealthy`。只报数，判据未冻结，不改 `core/`、`scenes/suite.txt`、`criteria.frozen`、`README.md`、`scripts/premerge.sh`、`scripts/regress.sh`、`scripts/sitl/Dockerfile`、`docs/tickets.md`，不改镜像，不写任何 `COM_` / `NAV_` / `BAT_` 参数。逐趟数字在 `bat1-lockstep-battery-runs.json`。时间是 2026-11-16 UTC-3。本票没有重跑 SITL：全部数字来自已有运行目录的 `px4-*.log` 和 `.ulg`。

结论先说：这不是起步电压低，也不是电池模型按墙钟放电，更不是锁步才有的时钟供给错误。根因是部分 PX4 实例在**解锁之后**停止发布 `battery_status`，指挥官按仿真钟过 5 s 判超时，打出 `Preflight Fail: Battery unhealthy`。电压一直是满电 16.2 V。第二版锁步没有因此触发 failsafe。要修只能改 PX4 的 posix 工作队列 / `battery_simulator`，或改镜像；这两种都在禁区，本票停在诊断。

## 不是什么

已有启动脚本会在 YOPO 实飞时注入 SITL 组参数 `SIM_BAT_DRAIN=86400`、`SIM_BAT_MIN_PCT=100`（`scripts/sitl/sitl-battery.env`），不碰 `BAT_` / `COM_` / `NAV_`。ulog 里受影响和不受影响的实例，发布期间电压都是 16.2 V，剩余电量约 0.989–1.000，`warning=0`、`connected=1`、`faults=0`。`result.json` 的 `px4_parameters_written` 仍是空列表。

PX4 `batteryCheck.cpp` 把 `battery_unhealthy` 设为真的条件是：

```
hrt_elapsed_time(&oldest_update) > 5_s
|| 缺了要求的电池 || 有 fault || warning == WARNING_FAILED
```

控制台那行 `Preflight Fail: Battery unhealthy` 只在「不健康、但不是缺电池、也没有 fault」时打印。本票看到的全部病例都属于第一条：最新 `battery_status` 的时间戳超过 5 s 没更新。

锁步第一步版（`run_to_sim_time`）另有世界失控后的 `Failsafe activated` 和 `battery warning (fast)`，B1 已作废，本票不再把它和第二版的 unhealthy 算成同一件事。

## 根因（解锁后模拟器停发）

`battery_simulator` 是 `hp_default` 工作队列上的 `ScheduledWorkItem`，100 Hz 一次，用 `hrt_absolute_time()` 打时间戳并发布。指挥官同样用 hrt（锁步下就是 gz 仿真钟）做 5 s 超时。两边用的是同一套钟。

对照同一趟第二版锁步交叉 `20261115T132622-yopo-n3`（`t_engage_s=15.408`，三机都在绝对时刻 21.408 s 解锁）：

| 实例 | `battery_status` 最后一条（相对/绝对 s） | `battery_unhealthy` 首次为 1 | 控制台 unhealthy | 电压 | `vehicle_status.failsafe` |
|---|---|---|---|---|---|
| px4-1 | 49.600 / 54.572（飞到收尾） | 从未置位 | 无 | 16.2 V | 0 |
| px4-2 | 11.620 / 21.416（紧贴解锁） | 16.688 / 26.484（解锁后 5.076 s） | 2 行 | 16.2 V | 0 |
| px4-3 | 6.656 / 21.412（紧贴解锁） | 11.720 / 26.476（解锁后 5.068 s） | 2 行 | 16.2 V | 0 |

px4-2 / px4-3 在解锁那一瞬间停止发布，指挥官继续跑，过 5 s 打旗标；第一次控制台告警就在旗标翻起来的时刻，第二次多在降落附近。任务照常飞完，没有 failsafe。

这不是日志漏记：同一份 ulog 里 `sensor_combined` 和 `failsafe_flags` 一直记到收尾，只有 `battery_status` 在解锁处切断，而且指挥官的旗标随后翻起来，说明 uORB 上确实没有新样本。

## 数字

B1 文档写的「第二版 10 趟三机交叉里有 5 趟出现 `Battery unhealthy`，没有一趟触发 failsafe」，用 `b1-sitl-lockstep-runs.json` 里 `v2-cross-lock` + `v2-cross-lock-delay50` 的控制台再数一遍，仍然是 **5/10 趟、8/30 机次、failsafe 0**。同批变阵 `v2-switch-lock` 是 5/5 趟、7/15 机次，同样没有 failsafe。单机 g2 锁步 0/3。

把 `/workspace/yopo-ops` 里带 ulog 的已有运行一并核对（775 机次、273 趟）：

| 集合 | 至少一机 unhealthy 的趟数 | 机次 | 停在解锁且随后 unhealthy | failsafe（uORB） |
|---|---|---|---|---|
| 锁步第二版（pause+multi_step） | 72/207 | 91/615 | 91，一一对应 | 0 |
| 锁步第一版（已作废） | 14/27 | 19/67 | 19，一一对应 | 21（世界失控，另案） |
| 自由跑 / 早期 S7 等（result 里无 lockstep 字段） | 9/37 | 10/91 | 10，一一对应 | 3 |

自由跑里同样有这个模式：例如 `20261115T105324-yopo-n3`（编队变阵，无锁步字段）的 px4-1 在解锁时刻停发，5.068 s 后 unhealthy；同趟另外两机电池一直发到收尾。所以这不是「只有锁步才会把电池模型和仿真钟拆开」。交叉卡锁步第二版合计 66/196 趟出现过，是间歇的，不是每趟必现。

120 个置位了 `battery_unhealthy` 的实例上，`unhealthy 时刻 − 解锁时刻` 中位 5.076 s，范围 5.016–5.100 s，和源码里的 5 s 门槛对齐。没有「停发却不报」或「没停发却报」的反例。

## 为什么脚本里修不了

B1 的锁步已经让 gz 每步把 `/world/default/clock` 交给 PX4：`GZBridge::clockCallback` 调 `px4_clock_settime`，posix `hrt` 走 `LockstepScheduler`。`battery_simulator` 本身就吃这条仿真钟，不是墙钟放电。再改启动脚本里的时钟供给途径，改变不了解锁后工作队列项停跑这件事。

可行的修法都在禁区：

1. 改 PX4：让 `battery_simulator` / `hp_default` 在解锁后继续按 `ScheduleOnInterval` 跑，或查 posix 工作队列在解锁瞬间的丢调度。这要改源码和镜像，不能改 `scripts/sitl/Dockerfile`。
2. 用 `CBRK_SUPPLY_CHK` 关掉供电检查：只是把告警压掉，不恢复发布，而且是安全检查旁路，本票不做。
3. 改 `BAT_` / `COM_LOW_BAT_ACT` 一类参数：门槛本来就不是电压，改了也无效，且在禁区。

因此本票不改启动脚本的默认行为，也不加「偏离上游」开关。收集脚本是 `scripts/sitl/bat1_collect.py`，单测不起 Gazebo。

## 回归

没有重跑 SITL，也没有改规划或锁步进给。单测：`PYTHONPATH=$PWD` 下 `tests/test_sitl_bat1_battery.py`（不起仿真）。全量 `scripts/regress.sh` 交给合入队列跑。
