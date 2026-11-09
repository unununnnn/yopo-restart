# yopo-restart

yopo 编队项目重启。规格见 [docs/SPEC.md](docs/SPEC.md)，票见 [docs/tickets.md](docs/tickets.md)。

## L0（box CPU，无界面，锁步确定性）

```bash
make venv                     # 一次（Python 3.12 + torch 2.4.1 CPU，uv）
.venv/bin/python -m yopo_l0.run --scene scenes/g2-hover.json --seed 0   # 单次：out/g2-hover/seed0/{result.json,ticks.jsonl}
.venv/bin/python -m yopo_l0.run --scene scenes/g2-pillar.json --seed 0 --policy upstream    # 单机过 G2 柱子
.venv/bin/python -m yopo_l0.run --scene scenes/open-3drone.json --seed 0 --policy hover      # 三机开阔区，各自飞向 goal_enu
.venv/bin/python -m yopo_l0.batch scenes/g2-pillar.json@upstream scenes/g2-pillar.json@certified --jobs 8  # M0 表
scripts/regress.sh            # 合 main 前回归：pytest + 批跑 + Markdown 表
```

L0 默认 `upstream` + 影子盾开（`--policy hover|certified`、`--shadow off` / `--no-shadow` 可覆盖）。影子盾只记录会否拒绝，不改批准动作。M0–M4 判据先报数后冻结（碰撞数如实报）。`upstream` 的终端规则是项目的 `stop_at_goal`，半径 1.5 m 已采纳（[ADR 0009](docs/adr/0009-stop-at-goal-vs-hold-here.md)）。上游 5 m `hold_here` 到不了 G2 oracle 的 0.4 m。

决策记录见 [docs/adr/](docs/adr/README.md)。M0 样例表（commit、场景卡哈希、复现命令）见 [docs/results/m0-g2-pillar.md](docs/results/m0-g2-pillar.md)。

YOPO 推理走 vendored `core/`（yopo-swarm core@00b85bf，见 core/VENDORED.md）：钉定上游 d5e1e60 + 权重 sha256 校验，CPU 单线程，与金样一致。

接缝：场景卡（`scenes/*.json`）→ `yopo_l0.runner.run(card, RunConfig)` → 结果记录 + 逐 tick 日志。
- 世界：轴对齐盒子 + 竖直圆柱 + 地面；G2 几何原样取自 yopo-swarm `sim/px4/g2-scene.json`。
- 传感器：光线投射深度，内参由分辨率 + 视场算，量程/噪声/帧率可配，时间戳 = 仿真时钟。
- 动力学：`step(state, approved_plan, dt)`，点质量 + 一阶加速度滞后。
- 锁步：每 tick 全体传感 → 全体规划 → 进程内总线投递 → 动力学 → 碰撞真值（障碍用机体半径 0.35 m，机间用两倍半径）；墙钟不进仿真时间。
- 规划器只拿 `Observation`（深度帧、里程计、目标、时间），不拿几何、不拿他机状态。每机一个策略实例。策略在 `yopo_l0.policy.POLICIES` 注册：`hover` / `upstream` / `certified`。
- 机间消息只走 `yopo_l0.bus.MessageBus`（payload 深拷贝）。旋钮 `drop` / `duplicate` / `delay` 对齐 G3 `FaultPlan`，`reorder` 对齐 G3 explore 的乱序（插到当前队首）。默认全 0。场景卡可写 `bus`，或用 `--bus-drop` 等覆盖。本机计划对象留在私有槽，总线只送令牌；他机只收到不含对象身份的公告。

结果记录里的 `tier` 总是 `L0`：这是假动力学，不是 PX4 结果。

## 结果记录

`result.json`（墙钟 `wall_time_s` 不参与确定性比较）：

| 字段 | 开（默认，`--shadow on`） | 关（`--shadow off` 或 `--no-shadow`） |
|---|---|---|
| `shadow` | `true` | `false` |
| `shadow_reject_rate` | 全体 tick 里会拒的比例。一步都不拒是 `0.0` | `null` |
| `shadow_reject_histogram` | 全体拒绝码 → tick 次数。一 tick 多个码各计一次。不拒是 `{}` | `null` |
| `shadow_drones` | 每机一行：`id`、`shadow_reject_rate`、`shadow_reject_histogram`。顺序与场景卡一致 | `null` |
| `shadow_gates` | 四个门的 `wired` / `impl` / `note` | `null` |

关的时候拒绝率是 `null` 而不是 `0`：`0.0` 只表示盾开着且没有会拒的 tick。拒绝不改 `outcome`。碰撞仍是硬失败，不进拒绝码。批跑表的「影子盾」列读 `shadow`，「影子拒绝率」列读 `shadow_reject_rate`（关着是 —）。

`ticks.jsonl` 每行一个 tick：

| 字段 | 含义 |
|---|---|
| `cmd_p` `cmd_v` `cmd_a` | 本 tick 批准计划的位置采样（不经 `sample`，所以不推进航向）。开/关逐位相同 |
| `would_reject` | 开：`true`/`false`。关：`null` |
| `reject_codes` | 开：拒绝码列表，空列表表示查了且不拒。关：`null` |

每机一份影子记录。选择门（证据、计划契约、SafetyGate）在该机规划时刷新，并抄到它执行该选择的物理 tick 上。oracle 跟踪/落后用本 tick 实际交给动力学的批准计划（总线尚未投递时是原地保持）。码和缺口见 `yopo_l0/shadow.py`。

## 场景套件、视频、nightly（票 04）

`scenes/suite.txt` 是 regress 与 nightly 共用的列表（`卡@策略`）：

- `g2-hover` / `g2-hover-lift` @ `hover`
- `g2-pillar` @ `upstream`（票 02 的场景卡和策略；本票不改策略。判据未冻结，只报数）
- `pillar-forest` / `narrow-corridor` @ `hover`（新几何，悬停占位）
- `open-3drone` @ `hover`（票 05 的三机卡；几何是空的 `open-area.json`）

G2 几何仍只在 `scenes/g2-scene.json`。柱林穿越是票 07，编队是票 06。

```bash
.venv/bin/python -m yopo_l0.batch scenes/g2-pillar.json@upstream scenes/open-3drone.json@hover --jobs 4 --out out/suite
.venv/bin/python -m yopo_l0.render --log out/suite/g2-pillar@upstream/seed0/ticks.jsonl --scene scenes/g2-pillar.json --out out/suite/g2-pillar@upstream/seed0/preview.mp4
scripts/nightly.sh
```

`scripts/nightly.sh` 把 JSON、Markdown、逐 tick 日志和每张卡首个 seed 的俯视+侧视视频写到 **`out/nightly/YYYYMMDD/`**。播放时长 ≤60 s。同一天再跑会先清空该目录。失败时 `ticks.jsonl` 留在目录里，用上面的 `render` 命令重放。合 main 前的门仍是 `scripts/regress.sh`（另含 certified 冒烟）。不接 GitHub Actions；box 上把 `scripts/nightly.sh` 放进 cron 即可。系统需要 `ffmpeg`。
