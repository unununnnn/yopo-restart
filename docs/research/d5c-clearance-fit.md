# D5c：学生训练侧 L0 机体半径净空过滤与安全加权（report-only）

本票只加训练选项和一份过滤统计。新选项全部默认关。不传时 `FitConfig()` 与 D5/D5b 走同一条 `trajectory_loss`，现有学生测试的损失路径不变。不改 `yopo_l0/policy.py`、`student.py`、`sensor.py`、`backend/`、`geom_fallback.py`、`core/`、`scenes/suite.txt`、`docs/tickets.md`、`README`。不跑闭环，不改已冻门，权重和教师数据仍在仓库外。

地图边长 20.0 m、障碍半径 0.20 m、教师机体半径 0.20 m 是假设，不是论文复现。见 [assumptions.py](../../scripts/teacher/assumptions.py)。L0 机体半径 0.35 m 是场景卡上的数。

## 1. 和 D5b 差在哪

[D5b](d5b-student-fix.md) 的 r4 在有障碍的场景全擦碰。g2 最小间隙 −0.005 m，密度 0.14 单机 −0.006 m，都在机体里面不到 1 cm。教师按 0.20 m 规划。重标注过滤是「未来 1.6 s 内表面净空 ≥ 0，而且这个净空已经减去了 0.20 m」。表面距离只要 ≥ 0.20 m 就留下。L0 的机体是 0.35 m，这 0.15 m 的差足够让「教师认为没碰上」的窗口，对学生来说已经在身体里。

D5b 第 7 节写过：净空过滤只删未来 1.6 s 里真碰上的窗口，删不掉贴着树飞的习惯。本票在训练侧补三件事，都不进飞行环：

1. 硬过滤按 L0 半径。表面距离 ≥ `0.35 m + margin_m` 才留下。`margin_m = 0` 就是刚好贴着 0.35 m 的身体；再大是额外安全裕度，数值是假设，本票没有在闭环里调过。
2. 软加权。净空落在 `[min_clearance_m, min_clearance_m + clearance_soft_margin_m)` 的窗口损失乘 `clearance_low_weight`（小于 1），避免把贴树当成该学的动作。
3. `endpoint_weight > 0` 时，1.6 s 视界后一半（8 个结点里的后 4 个，含末点）的位置误差乘 `1 + endpoint_weight`，再按步权重归一。对着 D5b 里开阔三机对向飞远（终点误差中位数 30.72 m）那种偏离，后段拉得更重。默认 0，位置项与原来逐元素相同。

这三项目前没有新的闭环表。D5b 已经说明单次训练会翻，本票不拿一次冒烟代替那个表。

## 2. 训练开关

都在 [student_fit.py](../../yopo_l0/student_fit.py) 的 `FitConfig`。默认值：

| 字段 | 默认 | 打开之后 |
|---|---|---|
| `min_clearance_m` | `None` | 训练和留出加载时，窗口自带的 `min_surface_clearance_m`（没有就用 `clearance`）必须 ≥ 该值。非有限值丢掉。`None` 时不读这个数组 |
| `clearance_soft_margin_m` | `0.0` | 与下面的权重一起，才对贴边窗口降权 |
| `clearance_low_weight` | `1.0` | 软带宽度 > 0 且 `0 ≤ weight < 1` 时，贴边窗口乘这个数。等于 1 时不乘 |
| `endpoint_weight` | `0.0` | `> 0` 时后半段位置误差加权。速度和偏航不加权 |
| `clearance_missing` | `"keep"` | 窗口没有净空数组时：`keep` 整场保持原样，`skip` 整场丢掉，`error` 报错 |

软带的下沿：设了 `min_clearance_m` 就用它，没设就从 0 算。数组里的数应是净空（表面距离减去比较用的机体半径），不是再减一次 0.35 m。

只开 `min_clearance_m`、其他损失开关仍是默认时，留下来的窗口还是 D5 的 `trajectory_loss`。`planar`、`near_weight`、文件 `path_weights` 的行为和 D5b 一样，贴边权重乘在近障权重后面。留出 MSE 只按硬门槛缩小样本，不把软权重算进评估。`planar=True` 时选最好一轮仍看水平位置 MSE。

## 3. 过滤统计

[filter_and_fit.py](../../scripts/distill/d5c/filter_and_fit.py) 的 `filter` 只写一份 JSON。它不改输入 npz，也不改仓库外已有的大数据集。

净空 = 未来 1.6 s 内到障碍表面的最小距离 − `body_radius_m`（默认 0.35）。保留 `净空 ≥ margin_m`，也就是表面距离 ≥ `0.35 + margin_m`。表面距离用场景卡上的圆柱、盒子和地面，公式与 `yopo_l0.scene` 相同。

几何从哪来，按这个顺序：

1. 清单或 `--trajectory-root/<场次>/trajectory.npz` 里，窗口 `time_s` 到 `time_s + 1.6` 的记录采样（含视界末端插值）。时间段与 D5b 重标注过滤相同。采样点之间的连续轨迹不看，快穿会漏。
2. 窗口里的 `position_world`，形状 `(N, K, 3)`，一般是 8 个 0.2 s 结点。
3. `origin_m`、`yaw_rad` 加上 `position_body`，用和 `R_yaw` 相同的矩阵变回世界系。
4. 以上都没有时，直接读已存的 `min_surface_clearance_m` 或 `clearance`，不再减半径。
5. 仍然没有，记成 `n_unscored`，不假装它被留下或被丢掉。

D5/D5b 转出来的窗口只有机体系标签，没有世界系未来位置。要对那些文件做 L0 重算，清单得能找到对应的 `trajectory.npz` 和 `scene.json`。`--trajectory-root` 按场次编号找 `<root>/<id>/trajectory.npz` 和 `scene.json`。

## 4. D6 双高度数据就绪之后

下面两条都是 report-only。数据目录是仓库外的约定路径，本票没有这些文件，也没有启动教师容器。第一条只写统计。第二条把权重写到 `--out`，仍然不覆盖输入 npz。

窗口 npz 里要先有 `min_surface_clearance_m`（表面距离减 0.35 m，未来 1.6 s 的最小值），`fit` 才滤得动。没有这个数组时，默认 `clearance_missing=keep`，等于不过滤。不要改 D6 原来的文件；把数组写进一份新拷贝，清单指向新路径。过滤统计本身可以不靠这个数组，它会从轨迹重算。

```bash
.venv/bin/python scripts/distill/d5c/filter_and_fit.py filter \
  --manifest /workspace/yopo-teacher-data/d6/convert-manifest.json \
  --trajectory-root /workspace/yopo-teacher-data/d6/runs \
  --body-radius-m 0.35 \
  --margin-m 0.02 \
  --out /workspace/yopo-teacher-data/d5c/filter-stats.json

.venv/bin/python scripts/distill/d5c/filter_and_fit.py fit \
  --manifest /workspace/yopo-teacher-data/d5c/convert-manifest.json \
  --out /workspace/yopo-teacher-data/d5c/student \
  --min-clearance-m 0.0 \
  --soft-margin-m 0.05 \
  --low-weight 0.25 \
  --endpoint-weight 1.0 \
  --clearance-missing error
```

`fit` 默认 `planar=True`（只学水平，和 D5b r4 一样）。这是这条命令的默认，不是 `FitConfig` 的默认；`FitConfig.planar` 仍然是 `False`。`--margin-m 0.02`、软带 0.05 m、低权重 0.25、末段权重 1.0 都是起点假设，不是闭环调出来的。D5b 的 `near_weight=3`、`shuffle_batches=64` 本命令不默认打开，需要时再传 `--near-weight 3 --shuffle-batches 64`。

冒烟不碰真实数据：

```bash
.venv/bin/python scripts/distill/d5c/filter_and_fit.py --smoke
```

## 5. 局限

- 没有新的飞行数。−0.005 m 到 −0.009 m 是 D5b r4 的测量，本票没有重飞。
- 硬门槛只去掉未来 1.6 s 里净空不够的教师窗口。跟踪误差仍可能把 0.35 m 外面的计划飞进身体。0.02 m 盖不住 D5b 里看到的几厘米跟踪差。
- 轨迹重算用的是记录采样，不是连续曲线。
- 打开地面的场景卡会把地面算进表面距离。1.5 m 高的空场净空约 1.15 m，不会被 0.35 m 的身体滤掉。
- `endpoint_weight` 过大时，后段贴标签会压过前面的绕行。默认关。
- 教师在密集格自己会擦树（D5b 第 7 节，[D6](d6-teacher-altitude.md) 沿用 0.20 m 的成功线）。把半径改成 0.35 m 之后，这些格能留下的窗口会更少。少到什么程度，要等双高度轨迹在仓库外跑完，用上面的 `filter` 读 `n_kept` / `n_dropped`。那份 JSON 不进 git。
