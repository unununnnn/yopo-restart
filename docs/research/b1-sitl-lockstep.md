# 票 B1：多机 PX4 SITL 仿真钟锁步（方向 B，只报数）

本票启动方向 B：让三机 PX4 SITL 和我们的代理链按 Gazebo 仿真钟一步一步走（锁步），代理算多久世界就停多久。第一张验收卡是 `crossing-pillars-sitl80`。背景是 S7（`s7-sitl-crossing-switch.md`）：自由跑时三机交叉 0/10，机间里程计时间差最高 251 s，主机一忙就坏。

只报数，判据未冻结。SITL `result.json` 仍是 `report_only=true`、`criteria_frozen=false`。锁步开关默认关，开时 `result.json` 带 `deviation: "偏离上游：仿真钟锁步"`。不改 `core/`、`scenes/suite.txt`、`criteria.frozen`、`README.md`、`scripts/premerge.sh`、`scripts/regress.sh`、`scripts/sitl/Dockerfile`、`docs/tickets.md`，不改镜像，不写 PX4 的 `COM_` / `NAV_` / `BAT_` 参数，也不写任何别的 PX4 参数：每趟 `px4_parameters_written` 都是空列表。逐趟数字在 `b1-sitl-lockstep-runs.json`。所有时间都是 2026-11-15 UTC-3。

## 锁步机制

PX4 v1.17 的 `GZBridge::clockCallback` 每收到一条 `/world/default/clock` 就 `px4_clock_settime`，PX4 posix 的 lockstep 调度器本来就跟着 gz 时间走；问题是 gz 自己在自由跑。所以只需要在 gz 一侧把世界管住，PX4 不用改：

1. 起飞前先暂停世界，等 clock 和各机电机计数都 50 ms 不变再开始（`engage`）。
2. 每个物理步（world `max_step_size` 0.004 s）发一次 `WorldControl(pause=True, multi_step=1)`：走一步，之后仍停着。
3. 握手：clock 到了目标时刻，并且每个 PX4 实例在这次请求之后都新发了一条 `/x500_i/command/motor_speed`（250 Hz，每物理步一条），这一步才算走完。最慢的那台决定节拍。
4. 代理控制拍 0.02 s = 5 物理步。代理在世界停住时算；算完发设定点、OCM、vehicle_command（锁步时由主循环发，心跳线程不发），再走下一拍。每拍后最多等 50 ms 墙钟拿到新里程计。
5. 失败关闭：clock 或握手 10 s 不到就抛 `TimeoutError`，这一趟写 `LOCKSTEP_TIMEOUT` 失败，不丢掉慢的那台继续跑。请求回复丢了不要紧（gz python 请求回复常丢但已生效），以 clock 为准；200 ms clock 一点没动才重发。
6. 三机共用一个 gz server 和一个世界钟，机间不存在「各走各的」。

开关：`scripts/sitl/run_yopo_l0_sitl.sh --lockstep`（或环境变量 `YOPO_SITL_LOCKSTEP=1`），`--physics-step`，以及对照用的 `--agent-delay-s`（`YOPO_AGENT_DELAY_S`，每个控制拍让代理多睡这么久）。不开时主循环和 S7 一样。代码在 `yopo_l0/sitl_lockstep.py`（`LockstepStepper`、`SimStampSkewMeter`、`count_sensor_timeouts`），接线在 `scripts/sitl/yopo_l0_live.py`。

### 第一版为什么作废

第一版每步发 `WorldControl(pause=False, run_to_sim_time=t_k)`。它在负载下会失控：目标一旦落在当前 clock 之前，gz 直接放开世界自由跑，之后每个请求的目标都已过去，世界就一直自由跑。证据是 `clock_overshoots ≈ steps`（例如 7118/7130），以及代理收到里程计时的仿真时刻反而比步进器早 40–480 ms（`odom_sim_age_max_s` 为负）。第一版 27 趟锁步里 11 趟是这样，g2 加人工延迟的 4 趟全在其中，所以第一版报过的「锁步 + 延迟 g2 0/4」不是锁步的结果。`runs.json` 里不带 `v2-` 前缀的臂都是第一版，只留作反例。第二版改成 `pause=True, multi_step=1`，世界不可能自己跑起来；另外每步前查 clock 是否已经跑到步进器前面（`clock_ahead`），有就记账并从实际时刻接着走。第二版 18 趟 `clock_ahead` 全 0，`clock_overshoots` 最多 1（单步内多走一格），握手超时 0，重发 0。

## 时钟偏差的量法

S7 的 `ClockSkewMeter` 用 PX4 消息里的时间戳。uXRCE 的 `UXRCE_DDS_SYNCT=1` 会把 PX4 时间戳映射到代理的墙钟纪元，锁步下那个差会到 1.79e9 s，没法用。本票新增 `odom_sim_skew`：每条里程计到达时记当时的 gz 仿真时刻，算各机「里程计在仿真时间上的年龄」和三机最新里程计之间的仿真时间差（`inter_vehicle_spread`）。

## 结果（第二版，13:04–13:43）

L0 和 S7 自由跑的数字取自 `s7-sitl-crossing-switch-runs.json`。到达时间：L0 从 PLAN 起算，SITL 取 `arrival − formation_t0`。

### crossing-pillars-sitl80，seeds 0–4

| 臂 | 成功 | 到达 / 机次 | 碰撞 | 到达时间 s 中位（范围） | 编队 p95 m | 最小机间 m 中位（最小） | RTF 中位（范围） | 三机里程计仿真时差 最大 s | 传感器 TIMEOUT 合计 |
|---|---|---|---|---|---|---|---|---|---|
| L0 | 5/5 | 15/15 | 0 | 7.06（7.05–7.76） | 4.34 | 7.28（3.49） | — | — | — |
| S7 自由跑（按需深度） | 0/5 | — | — | — | 19.53 | 7.93 | 0.726（0.463–0.990） | PX4 戳差 12.5–251 | — |
| B1 锁步 | **4/5** | 12/15 | 1 | 11.47（9.08–13.14） | 4.71 | 5.95（0.97） | 0.453（0.439–0.506） | 0.012–0.016 | 4 |
| B1 锁步 + 代理每拍多睡 50 ms | 3/5 | 9/15 | 2 | 12.16（9.44–16.70） | 4.84 | 5.76（1.09） | 0.216（0.196–0.231） | 0.008–0.016 | 2 |

两臂合计锁步交叉 7/10，S7 自由跑 0/10。三次失败（锁步 seed 3 `uav0`，加延迟 seed 2 `uav0`、seed 4 `uav1`）都是 `OBSTACLE`，而且同一个样子：PLAN 段最小净空 −0.010、−0.015、−0.015 m，也就是代理自己规划出来的轨迹就擦着柱子，不是 PX4 跟不上、也不是时钟问题。（更正，见 `b2-sitl-plan-clearance.md`：这个量是 PLAN 段里程计位置的净空，不是规划轨迹；B2 的逐拍记录显示设定点净空 ≥ 0.81 m，撞柱的是没跟上横移的飞机。）这三趟都没有 `Failsafe activated`、没有握手超时、`clock_ahead` 为 0。成功的 7 趟 PLAN 段最小净空 0.31–1.03 m。到达时间比 L0 长 2–5 s，编队 p95 和 L0 同量级（4.7 对 4.3 m），不再是 S7 的 19.5 m。

### formation-switch-sitl80（不能变差），seeds 0–4

| 臂 | 成功 | 到达时间 s 中位（范围） | 编队 p95 m 中位（范围） | 最小机间 m 中位（最小） | RTF 中位 | 三机里程计仿真时差 最大 s |
|---|---|---|---|---|---|---|
| L0 | 5/5 | 27.0 | 0.036 | 1.536 | — | — |
| S7 自由跑 | 5/5 | 27.22（27.19–27.27） | 0.322（0.310–0.347） | 1.372（1.288） | 0.997 | PX4 戳差 0.59–1.93 |
| B1 锁步 | **5/5** | 27.14（27.14–27.14） | 0.178（0.174–0.193） | 1.448（1.404） | 0.536 | 0.012–0.016 |

没有变差：编队 p95 从 0.32 降到 0.18 m，最小机间从 1.37 升到 1.45 m，更靠近 L0。

### 人造负载对照（单机 g2-pillar）

| 臂 | 成功 | 交接后到达 s | RTF | 里程计仿真年龄最大 s |
|---|---|---|---|---|
| 锁步 | 1/1 | 5.58 | 0.429 | 0.012 |
| 锁步 + 每拍多睡 50 ms（seeds 0、1） | 2/2 | 5.50、5.58 | 0.195–0.201 | 0.012 |
| 自由跑 + 每拍多睡 50 ms（第一版时段，seeds 0、1） | 2/2 | 7.78、8.28 | 0.997 | 0.93–0.98 |

加 50 ms 延迟后，锁步下代理看到的里程计仍然只有 12 ms 仿真年龄，到达时间和不加延迟一样，代价只是 RTF 从 0.43 掉到 0.20。自由跑同样的延迟让里程计老到将近 1 s，到达慢了约 2.5 s。这就是锁步要的性质：代理慢不改变仿真里发生的事，只改变墙钟。

### 开销

每 4 ms 物理步墙钟中位 6.1–6.9 ms（三机）、5.7 ms（单机）。三机锁步 RTF 约 0.44–0.54，主机 1 min loadavg 在 6–43 之间（8 核共享机，同时有别人的 `ego-teacher` 容器在跑）。

## 已知问题与没做完的

1. **规划擦柱子**：交叉卡锁步后还剩 3/10 失败，都是规划轨迹净空略小于 0。这是代理侧（YOPO 规划 / 安全半径）的问题，L0 同卡没出现，原因可能是 SITL 起飞后实际位置、速度和 L0 不同。建议另开票 B2 查这三趟（run 目录见 `runs.json`）。
2. **PX4 HRT 周期任务**：PX4 posix 的 `hrt_thread` 在 lockstep 下用 `px4_usleep`，解锁后 `battery_simulator`、`system_power_simulator`、`cpuload` 这类周期任务会停，5 s 后报 `Battery unhealthy`。第一版有一趟因此 `Failsafe activated` 进 RTL；S7 自由跑的日志里也有这行。第二版 18 趟没有 failsafe，但 10 趟三机交叉里有 5 趟出现 unhealthy 行。修它要改 PX4 或镜像，超出本票；按规则也不能用 `COM_` / `BAT_` 参数压掉。
3. **传感器 TIMEOUT**：每趟 0–2 条 `[sensors] ... fail: TIMEOUT`，多数在 engage 之前的启动段。
4. **握手是近似**：用「每台新发一条 motor_speed」代表 PX4 算完了这一步，依赖 PX4 每物理步出一条电机指令（250 Hz 对 4 ms 步长）。换步长或换机型要重看。
5. **退出码 134**：个别失败趟收尾时 `terminate called without an active exception`（rclpy 线程收尾）。`result.json` 已先写好，不影响数字。
6. 方向 B 还没做的：锁步下的深度渲染时刻对齐（现在深度帧按代理拍的仿真时刻和里程计位姿生成，和 S7 一样）、更多 seed、把锁步跑进 premerge。

## 回归

合入 origin/main（含 #139）后，在 `b7518b0` 上 13:55–14:24 跑 `scripts/regress.sh`：rc=0，pytest=通过，批跑=通过（`out/regress/20261115T135549`）。

| 项 | 结果 |
|---|---|
| L0 pytest（`-n 4`） | 906 passed、27 skipped，952.6 s |
| core pytest | 453 passed、3 skipped、4 deselected，259.8 s |
| 批跑（suite.txt） | 通过，748.3 s |
| certified 冒烟 | g2-pillar 0/1 TIMEOUT（未冻结、只报数，不计入通过判定） |
| `pytest -n 4 tests/test_sitl*.py` | 128 passed，30.6 s |

## 复现

```bash
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/crossing-pillars-sitl80.json --policy crossing --seed 0 --lockstep
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/crossing-pillars-sitl80.json --policy crossing --seed 0 --lockstep --agent-delay-s 0.05
scripts/sitl/run_yopo_l0_sitl.sh --n 3 --scene scenes/formation-switch-sitl80.json --policy switch --seed 0 --lockstep
python3 scripts/sitl/b1_collect.py <index.txt>   # 汇总成 runs.json 的 rows
```

单测 `tests/test_sitl_b1_lockstep.py` 不起 Gazebo：假世界 + 假墙钟，测步进网格、等最慢的 PX4、丢请求重发、死掉的 PX4 失败关闭、世界跑到前面要记账、慢代理不改变仿真时间表、开关默认关、shell 透传、不写任何 PX4 参数。
