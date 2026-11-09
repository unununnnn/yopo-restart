# 票列表（来源：docs/SPEC.md「Tickets」）

状态：`todo` / `in-progress` / `done`。依赖未完成的票不开工。

## 已定（2026-11-09）

frank 拍板卡片未回；父代理代定（仅 L0，L1 不动）：

1. **L0 默认 `upstream` 策略 + 影子盾开**（`--policy` / `--no-shadow` 可覆盖）。逐 tick 拒绝码在票 03：每机一条，全体另汇总。
2. **M0–M4 判据先报数、后冻结**：结果表写到达数 / 碰撞数 / 到达时间，过/不过标「未冻结 / report-only」。碰撞数 = 0 作为硬事实如实报，但不据此判合 main。

| 票 | 标题 | 被谁阻塞 | 状态 |
|---|---|---|---|
| 01 | L0 单机悬停纵切（世界 + 深度 + 动力学 + 锁步 + 结果记录） | 无 | done |
| 02 | 上游等价策略 + 单机过 G2 柱子（M0 核心） | 01 | done |
| 03 | 影子盾（真正记录实现） | 02 | done（每机一条影子记录；总拒绝率+直方图；判据先报数） |
| 04 | 场景卡套件 + 结果表 + 视频 + nightly | 02 | done |
| 05 | L0 多机骨架 + 消息总线 + 故障注入 | 02 | done |
| 06 | 三机编队保持（M1） | 05 | done |
| 06b | 编队同步出发丢 ACK 后的追赶 | 06 | todo |

06b：票 06 在同步出发那一轮收不齐 ACK 时，本机放弃提案并继续悬停已批准计划，不再追赶已经开走的虚拟中心。追赶不在 06 里做。
| 07 | 三机交叉避障（M2） | 06, 04 | todo |
| 08 | 一种变阵（M3 前半） | 07 | todo |
| 09 | L1 抽检通道（M3 后半）。Orin 驱动是 `scripts/run_g2.sh nominal`，不是 `ros2 launch`。L1 策略记 `deployed-certified` | 03, 04 | in-progress |
| 10 | ROS 2 形态 | 06 | done（M1 三机同 seed：到达/碰撞/队形误差 p95/机间距离/影子拒绝率与进程内一致；机间消息走 ROS 话题） |
| 11 | 六机 + 故障注入（M4） | 07, 08, 10 | todo |
| 12 | 交付包（M5） | 09, 11 | todo |

09：Orin 工单由 `scripts/l1_spotcheck.sh --prepare-only --scene scenes/g2-pillar.json --seed 0 --out out/l1/g2-pillar` 写到 `out/l1/g2-pillar/job-card.json`（schema `yopo-restart/l1-job-card/1`）。全量命令是 `YOPO_SWARM_ROOT=<yopo-swarm> PX4_PARAM_DUMP_CMD='<只读 param show>' scripts/l1_spotcheck.sh --scene scenes/g2-pillar.json --seed 0 --out out/l1/g2-pillar`。脚本内部执行 `YOPO_SIM_SEED=<seed> bash "$YOPO_SWARM_ROOT/scripts/run_g2.sh" nominal`，超时 1800 s。产物是 `bundle/scene.json`、`bundle/run.json`、`log/orin-result.json`、`log/events/`、`log/result.json`、`guard/` 里的参数表和安全文件哈希。过：`run_g2.sh` 退出码 0，参数表和安全哈希前后一致，且收下了 `result.json`。不过：退出码非 0、参数或安全文件有差异、没有 `result.json`，或命令里出现 param set/save/load/reset/import。已知缺口：这条路径仍是已部署的 certified 栈，不接受 `--policy upstream`；对照表写 `policy=deployed-certified`。把 upstream 移植进 L1 要等以后的 yopo-swarm 票，本仓库不改那个仓库。宿主上不要 `make venv`（aarch64 没有 torch cpu wheel）；export/import/report 只用 numpy，缺解释器时用已有镜像 `yopo-swarm:orin-infer-cu13`。

可并行前沿：01 → 02；02 后 03 / 04 / 05 可并行；06 后 07 / 10 可并行；09 在 03、04 完成后可与多机线并行。

每张票的验收清单见 docs/SPEC.md 对应小节。所有票都通过同一接缝验收：场景卡 → 运行器 → 结果。
