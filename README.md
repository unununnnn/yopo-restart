# yopo-restart

yopo 编队项目重启。规格见 [docs/SPEC.md](docs/SPEC.md)，票见 [docs/tickets.md](docs/tickets.md)。

## L0（box CPU，无界面，锁步确定性）

```bash
make venv                     # 一次（Python 3.12 + torch 2.4.1 CPU，uv）
.venv/bin/python -m yopo_l0.run --scene scenes/g2-hover.json --seed 0   # 单次：out/g2-hover/seed0/{result.json,ticks.jsonl}
.venv/bin/python -m yopo_l0.run --scene scenes/g2-pillar.json --seed 0 --policy upstream    # 单机过 G2 柱子
.venv/bin/python -m yopo_l0.batch scenes/g2-pillar.json@upstream scenes/g2-pillar.json@certified --jobs 8  # M0 表
scripts/regress.sh            # 合 main 前回归：pytest + 批跑 + Markdown 表
```

L0 默认 `upstream` + 影子盾开（`--policy hover|certified`、`--no-shadow` 可覆盖；影子盾真正记录留给票 03）。M0–M4 判据先报数后冻结（碰撞数如实报）。
YOPO 推理走 vendored `core/`（yopo-swarm core@00b85bf，见 core/VENDORED.md）：钉定上游 d5e1e60 + 权重 sha256 校验，CPU 单线程，与金样一致。

接缝：场景卡（`scenes/*.json`）→ `yopo_l0.runner.run(card, RunConfig)` → 结果记录 + 逐 tick 日志。
- 世界：轴对齐盒子 + 竖直圆柱 + 地面；G2 几何原样取自 yopo-swarm `sim/px4/g2-scene.json`。
- 传感器：光线投射深度，内参由分辨率 + 视场算，量程/噪声/帧率可配，时间戳 = 仿真时钟。
- 动力学：`step(state, approved_plan, dt)`，点质量 + 一阶加速度滞后。
- 锁步：每 tick 传感器 → 规划 → 动力学 → 碰撞真值（机体半径 0.35 m）；墙钟不进仿真时间。
- 规划器只拿 `Observation`（深度帧、里程计、目标、时间），不拿几何。策略在 `yopo_l0.policy.POLICIES` 注册：`hover` / `upstream` / `certified`。

结果记录里的 `tier` 总是 `L0`：这是假动力学，不是 PX4 结果。
