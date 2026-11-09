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

## ROS 2 形态（票 10）

世界一个节点，每机一个节点。世界按 L0 的 tick 往 `/clock` 发仿真时间（各节点 `use_sim_time`），锁步仍是传感 → 规划 → 总线 → 动力学 → 碰撞真值，和 `yopo_l0.runner` 同一条循环。规划在该机节点里调用已有的 `yopo_l0.policy`。命名空间是场景卡里的无人机 id：`/uav0/drone`、`/uav1/drone`……影子盾也在这条循环里记，所以 `shadow_reject_rate` 和逐 tick 的 `would_reject` 与进程内形态相同。

编队（M1，`scenes/formation-hold.json`）的机间协议消息不进进程内 `MessageBus` 队列。一次运行里 `FormationFleet.transport_override` 指向 ROS 传输：没被故障丢掉的信封按锁步走完三跳，`poll` 才看得到话题上回来的那一份。

1. `/{src}/bus_tx`：世界把信封交给源无人机节点
2. `/{dest}/bus_rx`：源节点转发给目的无人机
3. `/{dest}/bus_done`：目的节点交回世界

单线程执行器，一条消息的分片按序走完才发下一条，DDS 谁先到不进仿真结果。丢包、重复、时延、乱序仍用和进程内相同的 `BusFaults` 抽样（时延只改 `deliver_at`，不睡墙钟）。被丢掉的消息不上话题。非编队场景的他机公告仍走进程内总线。

```bash
scripts/setup_ros2.sh    # Ubuntu 24.04 → Jazzy，22.04 → Humble；ros-*-ros-base + rclpy
.venv/bin/python -m yopo_l0.ros2_run --scene scenes/open-3drone.json --seed 0 --policy hover
.venv/bin/python -m yopo_l0.ros2_run --scene scenes/g2-pillar.json --seed 0 --policy upstream
.venv/bin/python -m yopo_l0.ros2_run --scene scenes/formation-hold.json --seed 0 --policy formation
```

输出和 `yopo_l0.run` 一样：`out/<场景>/seed<N>/{result.json,ticks.jsonl}`，结果里多一个 `"form": "ros2"`。`--policy`、`--seed`、`--shadow on|off` / `--no-shadow`、`--duration`、`--out`、`--bus-*` 与进程内启动器相同。`--policy formation` 只接受 `task.type` 为 `formation` 的卡。节点仍按卡里的 `drones` 建，id 即命名空间。

`rclpy` 导入不了时，`tests/test_ros2_form.py` 和 `tests/test_ros2_formation.py` 里的 ROS 用例跳过，原因里写明要跑 `scripts/setup_ros2.sh`。进程内回归不依赖 ROS。自己开解释器时再 `source /opt/ros/<jazzy|humble>/setup.bash`；启动器会自己把已安装的发行版加进模块路径。

批准计划对象留在本进程。动力学要 `sample` 策略交出来的那一个（上游偏航和 certified 运行时都挂在上面），ROS 消息里是观测和计划令牌。编队协议载荷用 `float.hex` 往返，计划哈希和占用规范形式不变。

## 场景套件、视频、nightly（票 04）

`scenes/suite.txt` 是 regress 与 nightly 共用的列表（`卡@策略`）：

- `g2-hover` / `g2-hover-lift` @ `hover`
- `g2-pillar` @ `upstream`（票 02 的场景卡和策略；本票不改策略。判据未冻结，只报数）
- `pillar-forest` / `narrow-corridor` @ `hover`（新几何，悬停占位）
- `open-3drone` @ `hover`（票 05 的三机卡；几何是空的 `open-area.json`）
- `formation-switch` @ `switch`（票 08：窄门触发横排↔纵列，判据未冻结，只报数）

G2 几何仍只在 `scenes/g2-scene.json`。柱林穿越是票 07，编队是票 06。

```bash
.venv/bin/python -m yopo_l0.batch scenes/g2-pillar.json@upstream scenes/open-3drone.json@hover --jobs 4 --out out/suite
.venv/bin/python -m yopo_l0.render --log out/suite/g2-pillar@upstream/seed0/ticks.jsonl --scene scenes/g2-pillar.json --out out/suite/g2-pillar@upstream/seed0/preview.mp4
scripts/nightly.sh
```

`scripts/nightly.sh` 把 JSON、Markdown、逐 tick 日志和每张卡首个 seed 的俯视+侧视视频写到 **`out/nightly/YYYYMMDD/`**。播放时长 ≤60 s。同一天再跑会先清空该目录。失败时 `ticks.jsonl` 留在目录里，用上面的 `render` 命令重放。合 main 前的门仍是 `scripts/regress.sh`（另含 certified 冒烟）。不接 GitHub Actions；box 上把 `scripts/nightly.sh` 放进 cron 即可。系统需要 `ffmpeg`。

## L1 抽检（票 09，box 侧）

真 PX4 SITL + Gazebo 在 frank 的 WSL 或 Orin 上跑。box 只做三件事：把场景卡导出成 yopo-swarm 现有栈能吃的目录、事后把日志收成和 L0 同一张 `result.json`（`tier=L1`，多起飞交接和 RTF）、以及同卡的 L0/L1 对照。不在这里启动 PX4，也不改 PX4 参数和 `g2_run.py` 里的安全清单数值。

路径引自 yopo-swarm @00b85bf 已经抄进本仓库的说明（本环境读不到那个私有仓库本身）：

- 启动：`ros2 launch yopo_swarm_px4 uavs.launch.py`，文件 `ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py`（`docs/ref/minco-port-map.md` 第 23 行）
- 世界：`sim/px4/g2-scene.json` 的格式（`world`、`goal_enu`、轴对齐 `boxes`）。圆柱和别的几何直接拒绝
- 安全清单数值：`g2_run.py`（`yopo_l0/policy.py` 里 certified 默认值注明与它对齐）。导出不把这些数写进启动参数

抽检机上若能读到那份 launch 文件，参数名按文件里的 `DeclareLaunchArgument` 对齐；对不上 `scene` / `seed` / `policy` / `shadow` / `start_enu` / `goal_enu` 就失败，不会把不认识的参数塞给 `ros2 launch`。

```bash
.venv/bin/python -m yopo_l1.export --scene scenes/g2-pillar.json --seed 0 --policy upstream --shadow on --out out/l1/bundle
# WSL / Orin：
YOPO_SWARM_ROOT=/path/to/yopo-swarm PX4_PARAM_DUMP_CMD='param show' \
  scripts/l1_spotcheck.sh --scene scenes/g2-pillar.json --seed 0 --out out/l1/g2-pillar
.venv/bin/python -m yopo_l1.report --l0 out/g2-pillar/seed0 --l1 out/l1/g2-pillar/log
```

`PX4_PARAM_DUMP_CMD` 必须是只读的（`param show` 可以，`param set` / `save` / `load` / `reset` / `import` 会被拒绝）。脚本在 launch 前后各 dump 一次参数，并给 `g2_run.py` 和文件名含 safety / checklist 的文本做 sha256；有差异就失败，不把这次写进成功结果。批跑表末尾两列是起飞交接和 RTF，L0 留空。RTF = 仿真秒 / 墙钟秒。

## 合 main（premerge）

不接 GitHub Actions。每个要进 main 的 PR 在本机跑：

```bash
scripts/premerge.sh 123            # 查 MERGEABLE，临时 worktree 里跑 regress.sh，查 Co-authored-by，在 PR 留一条评论
scripts/premerge.sh 123 --merge    # 上面全部通过后：gh pr ready，再 gh pr merge --merge --match-head-commit <检验过的 sha>
scripts/premerge.sh 123 --squash   # 同样只在全部通过后合并；说明由各提交拼成，并去掉 Co-authored-by 行
```

不可合并、regress 非 0、解析不到 pytest 计数或批跑表、任一提交带 `Co-authored-by` trailer、评论或合并失败，退出码都不是 0。评论里有 head sha、L0 与 core 的 passed/skipped、regress 退出码、总退出码，以及批跑 Markdown 表。

不给 `--merge` / `--squash` 时脚本不合并。它不 `git push`，也不 force-push。worktree 用完即删。

`make venv` 在 `.venv/bin/python` 已在、且 `requirements.txt` / `pyproject.toml` / `core/pyproject.toml` 的哈希未变时直接返回，避免 `uv venv` 报 virtual environment already exists。第三方装在 `~/.cache/yopo-restart/deps/<哈希>`（本目录只挂 editable）；wheel 走 uv 缓存，所以新 worktree 不重装 torch。
