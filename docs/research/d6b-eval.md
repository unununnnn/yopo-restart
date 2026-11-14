# 票 D6b-E：D6b 双高度闭环评测结果

只报数。report-only。本文件入库 2026-11-13 15:27（UTC-3）生成的闭环评测结果：200/200 行，`eval.exit=0`。不改 `yopo_l0/`、`core/`、`scenes/`、`scripts/`、`docs/tickets.md`、README。权重不进 git。数字在 [d6b-eval-runs.json](d6b-eval-runs.json)。流水线说明见 [d6b-pipeline.md](d6b-pipeline.md)。

- 权重 `/workspace/yopo-teacher-data/d6b/train/student.pt`
- 权重 sha256 `e4bd211662b478066f3611ac8ca6e9cbc6b13445dfdaef82440c04bc53e1f3f0`
- 训练摘要来自 `/workspace/yopo-teacher-data/d6b/train/train-summary.json`：5 epoch，留出 500 场、216140 窗口，留出 `position_xy_rmse` 0.0658 m，`position_rmse` 0.117 m
- 评测时学生接口 `{"knots": "position", "yaw_source": "ego_lookahead", "accel_source": "reference", "vertical": "hold_goal_z"}`
- 库默认仍是 `{"student_knots": "network", "student_yaw_source": "calculate_yaw", "student_accel_source": "state", "student_vertical": "network", "track_clearance": false}`。评测不修改这些默认。`geom_multi` 只写在这次评测的内存场景卡上。学生策略不读 `policy_params.upstream.track_clearance`，这一臂不改变飞法，用来对照默认没被改写。

## 复现

评测命令（`/workspace/yopo-teacher-data/d6b/eval/eval.cmd`）：

```
# started from /workspace/yopo-restart
# America/Argentina/San_Juan
setsid bash -c '
  echo $$ > /workspace/yopo-teacher-data/d6b/eval/eval.pid
  export PYTHONUNBUFFERED=1
  nice -n 12 .venv/bin/python -u scripts/distill/d6/eval_altitudes.py \
    --weights /workspace/yopo-teacher-data/d6b/train/student.pt \
    --out /workspace/yopo-teacher-data/d6b/eval \
    --altitudes 1.0 1.5 \
    --seeds 0 1 2 3 4 5 6 7 8 9 \
    --track-clearance off geom_multi \
    --jobs 2
  echo $? > /workspace/yopo-teacher-data/d6b/eval/eval.exit
'
```

## 主表

| 场景 | 高度 m | track_clearance | 到达 | 撞障场 | 机间场 | 超时场 | 最小净空 m | 终点误差中位数 m |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 开阔三机平行 | 1.0 | off | 0/30 | 0 | 0 | 10 | +0.650 | 4.042 |
| 开阔三机对向 | 1.0 | off | 10/30 | 0 | 0 | 10 | +0.650 | 6.313 |
| g2 柱子 | 1.0 | off | 0/10 | 10 | 0 | 0 | -0.002 | 2.682 |
| 密度 0.14 单机 | 1.0 | off | 0/10 | 10 | 0 | 0 | -0.001 | 15.916 |
| 密度 0.14 三机对向 | 1.0 | off | 0/30 | 10 | 0 | 0 | -0.008 | 18.628 |
| 开阔三机平行 | 1.5 | off | 0/30 | 0 | 0 | 10 | +1.150 | 4.042 |
| 开阔三机对向 | 1.5 | off | 10/30 | 0 | 0 | 10 | +1.150 | 6.313 |
| g2 柱子 | 1.5 | off | 0/10 | 10 | 0 | 0 | -0.003 | 2.681 |
| 密度 0.14 单机 | 1.5 | off | 0/10 | 10 | 0 | 0 | -0.011 | 15.912 |
| 密度 0.14 三机对向 | 1.5 | off | 0/30 | 10 | 0 | 0 | -0.001 | 18.633 |
| 开阔三机平行 | 1.0 | geom_multi | 0/30 | 0 | 0 | 10 | +0.650 | 4.042 |
| 开阔三机对向 | 1.0 | geom_multi | 10/30 | 0 | 0 | 10 | +0.650 | 6.313 |
| g2 柱子 | 1.0 | geom_multi | 0/10 | 10 | 0 | 0 | -0.002 | 2.682 |
| 密度 0.14 单机 | 1.0 | geom_multi | 0/10 | 10 | 0 | 0 | -0.001 | 15.916 |
| 密度 0.14 三机对向 | 1.0 | geom_multi | 0/30 | 10 | 0 | 0 | -0.008 | 18.628 |
| 开阔三机平行 | 1.5 | geom_multi | 0/30 | 0 | 0 | 10 | +1.150 | 4.042 |
| 开阔三机对向 | 1.5 | geom_multi | 10/30 | 0 | 0 | 10 | +1.150 | 6.313 |
| g2 柱子 | 1.5 | geom_multi | 0/10 | 10 | 0 | 0 | -0.003 | 2.681 |
| 密度 0.14 单机 | 1.5 | geom_multi | 0/10 | 10 | 0 | 0 | -0.011 | 15.912 |
| 密度 0.14 三机对向 | 1.5 | geom_multi | 0/30 | 10 | 0 | 0 | -0.001 | 18.633 |

## 高度对照

| 场景 | track_clearance | 1.0 m | 1.5 m |
| --- | --- | --- | --- |
| 开阔三机平行 | off | 0/30，撞 0，超时 10，净空 +0.650 | 0/30，撞 0，超时 10，净空 +1.150 |
| 开阔三机对向 | off | 10/30，撞 0，超时 10，净空 +0.650 | 10/30，撞 0，超时 10，净空 +1.150 |
| g2 柱子 | off | 0/10，撞 10，超时 0，净空 -0.002 | 0/10，撞 10，超时 0，净空 -0.003 |
| 密度 0.14 单机 | off | 0/10，撞 10，超时 0，净空 -0.001 | 0/10，撞 10，超时 0，净空 -0.011 |
| 密度 0.14 三机对向 | off | 0/30，撞 10，超时 0，净空 -0.008 | 0/30，撞 10，超时 0，净空 -0.001 |
| 开阔三机平行 | geom_multi | 0/30，撞 0，超时 10，净空 +0.650 | 0/30，撞 0，超时 10，净空 +1.150 |
| 开阔三机对向 | geom_multi | 10/30，撞 0，超时 10，净空 +0.650 | 10/30，撞 0，超时 10，净空 +1.150 |
| g2 柱子 | geom_multi | 0/10，撞 10，超时 0，净空 -0.002 | 0/10，撞 10，超时 0，净空 -0.003 |
| 密度 0.14 单机 | geom_multi | 0/10，撞 10，超时 0，净空 -0.001 | 0/10，撞 10，超时 0，净空 -0.011 |
| 密度 0.14 三机对向 | geom_multi | 0/30，撞 10，超时 0，净空 -0.008 | 0/30，撞 10，超时 0，净空 -0.001 |

## 结论

开环留出误差小，但闭环 20 组里只有开阔三机对向拿到 10/30，有障碍的三个场景（g2 柱子、密度 0.14 单机、三机对向）在 1.0 m 和 1.5 m 都是每颗种子撞障碍，开阔三机平行 0/30 全超时；geom_multi 与 off 完全相同（学生不读该参数，符合预期）；1.0 m 与 1.5 m 结果几乎一致。
