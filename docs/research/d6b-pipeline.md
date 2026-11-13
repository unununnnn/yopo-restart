# D6b：1.0 m 与 1.5 m 教师混合转换、训练、双高度评测

本票准备全量混合，不在这里飞教师，也不报闭环成绩。只新增四个文件：

- `scripts/distill/d6/pipeline.py`
- `scripts/distill/d6/eval_altitudes.py`
- `tests/test_d6b_pipeline.py`
- 本说明

不改 `yopo_l0/policy.py`、`yopo_l0/sensor.py`、`yopo_l0/distill.py`、`yopo_l0/student.py`、`yopo_l0/student_fit.py`、`scripts/distill/d6/feasibility.py`、`core/`、`scenes/`、`docs/tickets.md`、`README.md`、`scripts/regress.sh`。学生不选时仍是 D5。`StudentPolicy.DEFAULTS` 仍是 `knots=network`、`yaw_source=calculate_yaw`、`accel_source=state`、`vertical=network`。`track_clearance` 的库默认仍是 false。

地图边长 20.0 m、障碍半径 0.20 m 是假设，不是论文复现。见 [assumptions.py](../../scripts/teacher/assumptions.py)。

`scripts/distill/convert_d2b.py` 只读 2026-11-12 01:26:35（UTC-3）那份冻结快照。[feasibility.py](../../scripts/distill/d6/feasibility.py) 只把仓库里的 1.0 m 小样本做 z 平移。这两条路都不读 box 上正在变长的目录。本票读目录里已经写完的 `record.json`。

## 1. 数据从哪来

| 高度 | 目录 | 编号 |
| --- | --- | --- |
| 1.0 m | `/workspace/yopo-teacher-data/d2b/runs` | `d014-n3-headon-s0` 这种，没有 `z` |
| 1.5 m | `/workspace/yopo-teacher-data/d6-z150/runs` | `d014-n3-headon-z150-s0` |

一场一个目录，里面是 `yopo-teacher-record/1`：`record.json`、`trajectory.npz`、`scene.json`、`geometry.json`。和 d2b 相同。高度优先用记录里的 `scenario.flight_z_m`。记录没写、编号里有 `z150` 就用编号。两个都没有，按 1.0 m，和 d2b 的飞行高度一致。记录和编号对不上时仍用记录里的数，manifest 里 `z_mismatch` 为 true。

## 2. 口径

### 2.1 D5b r4

训练调用 `yopo_l0.student_fit.train_from_paths`，`FitConfig` 与 [train_r4.py](../../scripts/distill/d5b/train_r4.py) 相同：

| 项 | 值 |
| --- | --- |
| `planar` | true，位置和速度损失只算 x/y |
| `near_weight` | 3.0 |
| `near_depth_m` | 1.2 |
| `shuffle_batches` | 64 |
| 学习率 / 批大小 / epoch / 种子 | 1e-3 / 64 / 5 / 0 |
| 损失权重 位置、速度、偏航 | 1.0、0.25、0.05 |
| 线程 | 2 |

闭环四项不是 `FitConfig` 的字段，写在检查点 `meta.d6b.eval_params`，评测时写到内存场景卡的 `policy_params.student`：

| 项 | 值 |
| --- | --- |
| `knots` | `position` |
| `yaw_source` | `ego_lookahead` |
| `accel_source` | `reference` |
| `vertical` | `hold_goal_z` |

`--epochs 1` 只给冒烟。全量跑不要改 epoch，默认就是 5。检查点里 `recipe_fit` 是上面这张表，`fit_used` 是这次实际用的数。

### 2.2 高度配比

留出规则与 D5 相同：`seed % 5 == 0` 进留出。编号里的场景键不含高度，所以同一颗种子的 1.0 m 和 1.5 m 进同一边。

| `--balance` | 行为 |
| --- | --- |
| `full`（默认） | 训练名单里的场全部进入，每场权重 1 |
| `altitude` | 每个高度的训练场数抽到一样多。场数少的一边全留，多的一边用训练种子抽样。抽完若两边窗口数仍不同，用 `path_weights` 把两边的窗口总权重调成相等 |

全量谁多谁少要等 d6-z150 飞完才知道。两边场数差一截时用 `altitude`，免得 1.5 m 被 1.0 m 的窗口数盖住。两边已经接近时用 `full`。

### 2.3 净空过滤可选

`--min-clearance-m 0`（默认）或 `off`：只收 `schema == yopo-teacher-record/1` 且 `metrics.success == true`。教师成功线本身是净空 ≥ 0，设 0 不会把这些成功场再丢掉。

设成正数（例如 `0.05`）时，还要求 `metrics.min_obstacle_clearance_m >=` 该值。缺字段、写坏、或低于门槛的成功场记入 `n_filtered`，不转窗口。manifest 里 `clearance_filter` 标明这道门开了没有。按高度、按场景各有成功数、过滤数、转换场数、窗口数。

### 2.4 窗口缓存

每场一个 `windows/<场次编号>.npz`，走现成的 `convert_run` / `save_batch`。`--jobs` 并行。`convert-status.jsonl` 记 sha。文件还在、sha 没变、`--max-windows` 也没变，就跳过。中断后用同一条命令接着跑。

一场缺轨迹或转换抛错，写进 manifest 的 `failures`，其余场照常转完。退出码仍是 0。目录本身不存在才是非 0。

检查点是 `train/student.pt`。`meta.d6b` 里有 r4 配方、这次的拟合参数、训练/留出的高度分布、manifest 摘要（成功数、窗口数、过滤数、分高度分场景，不含逐文件 sha）。旁边的 `train/d6b-train.json` 记最终 sha256。权重不进 git。

## 3. 命令

转换和训练：

```bash
.venv/bin/python scripts/distill/d6/pipeline.py all \
    --teacher-dirs /workspace/yopo-teacher-data/d2b/runs \
                   /workspace/yopo-teacher-data/d6-z150/runs \
    --out /workspace/yopo-teacher-data/d6b \
    --jobs 2 \
    --min-clearance-m 0 \
    --balance altitude
```

只转换、只训练：

```bash
.venv/bin/python scripts/distill/d6/pipeline.py --only-convert \
    --teacher-dirs /workspace/yopo-teacher-data/d2b/runs \
                   /workspace/yopo-teacher-data/d6-z150/runs \
    --out /workspace/yopo-teacher-data/d6b --jobs 2

.venv/bin/python scripts/distill/d6/pipeline.py --only-train \
    --out /workspace/yopo-teacher-data/d6b --balance altitude
```

双高度评测。五个场景都跑，种子 0–9，时长用场景卡自己的。`off` 是学生这次的数字。`geom_multi` 是对照臂：

```bash
.venv/bin/python scripts/distill/d6/eval_altitudes.py \
    --weights /workspace/yopo-teacher-data/d6b/train/student.pt \
    --out /workspace/yopo-teacher-data/d6b/eval \
    --altitudes 1.0 1.5 \
    --seeds 0 1 2 3 4 5 6 7 8 9 \
    --track-clearance off \
    --jobs 2
```

两臂一起看：

```bash
.venv/bin/python scripts/distill/d6/eval_altitudes.py \
    --weights /workspace/yopo-teacher-data/d6b/train/student.pt \
    --out /workspace/yopo-teacher-data/d6b/eval-track \
    --altitudes 1.0 1.5 \
    --seeds 0 1 2 3 4 \
    --track-clearance off geom_multi \
    --jobs 2
```

输出 `eval-altitudes.json` 和 `eval-altitudes.md`。两份开头都有 report-only。表是场景 × 高度 × `track_clearance`：到达/分母、撞障场、机间场、超时场、最小净空、终点误差中位数。最小净空是障碍距离减去机体半径 0.35 m。

`geom_multi` 只把 `policy_params.upstream.track_clearance=true`、`track_clearance_mode=geom_multi`、`track_margin_m=0.08` 写在这次的内存卡上，不写回 `scenes/`。学生策略不读这项。库默认仍是 false。两臂的到达若相同，是因为飞法没变，不是评测没跑。

场景是 `open-3drone`、`swap-3-open`、`g2-pillar`、`ego-density-014-solo`、`ego-density-014-swap3`。`--scenes` 可以只跑其中几个。`--duration-s` 盖过卡上的时长，只给冒烟。

## 4. d6-z150 飞完之后

1. 确认 `/workspace/yopo-teacher-data/d6-z150/runs/<场次>/record.json` 的 schema 是 `yopo-teacher-record/1`，`scenario.flight_z_m` 是 1.5，编号带 `z150`。没写完的场没有 `metrics.success`，扫描时计入未完成，不转换。
2. 先 `--only-convert`。打开 `convert-manifest.json`，看 `by_altitude` 的 1.0 和 1.5：成功数、过滤数、窗口数。`failures` 应是空的，或只剩缺轨迹的场。
3. 净空门保持 0，除非要故意丢掉贴障的成功场。要加门就换一个 `--out`，或把 `--min-clearance-m` 设成正数再转；被滤掉的场不会进 `files`，磁盘上旧的 npz 也不进训练名单。
4. `--only-train --balance altitude`（场数差得大）或 `--balance full`（两边已经接近）。epoch 用默认 5。看 `train/d6b-train.json` 里两边的训练场数和窗口数，记下 `weights_sha256`。
5. 评测先 `--seeds 0 1 2 3 4 --jobs 2`，再决定要不要补到 0–9。1.0 m 和 1.5 m 都要有一行。`track-clearance` 先用 `off`。
6. 权重、窗口、评测 json 留在 `/workspace/yopo-teacher-data/d6b/`，不进这个仓库。成绩另写结果说明，本文件只记入口。

## 5. 单测

合成的 `yopo-teacher-record/1` 放在临时目录，含 1.0 m 和 1.5 m，其中一场成功但净空为负、一场成功但没有轨迹、一场 schema 不对、一场失败。

```bash
.venv/bin/python -m pytest -q tests/test_d6b_pipeline.py
```

覆盖：`0` 不过滤、正门槛会计过滤数、按高度抽到同样场数、每场张量形状、manifest 分高度分场景的计数、断点续跑 sha 不变、检查点能加载且带 r4 与高度分布、1.0 m 和 1.5 m 的评测 json / markdown 带 report-only。跑完 `StudentPolicy.DEFAULTS` 和场景卡文件的字节不变。

## 6. 局限

- d6-z150 还在飞。这里没有全量窗口数，也没有闭环到达数。
- 高度平衡先按场数抽齐，再用路径权重补窗口数的差。不是把窗口打散后逐条抽。
- `hold_goal_z` 是评测时定高，不是网络学出来的高度。
- `geom_multi` 这臂不改变学生飞法。它只证明对照开关没有写进库默认。
- 不读 2026-11-12 的 cutoff 文件。`d2b/runs` 里现在有的完成场都会进转换，包括冻结之后续上的场。
- `train_from_paths` 把 `loss_weights` 元组交给 `json.dumps`。本脚本只在这次调用期间让 `json` 接受元组，调用结束恢复，不改 `student_fit.py`。
