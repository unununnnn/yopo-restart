# 票 S6：upstream + 影子盾接到单机 PX4 SITL，与 M0 g2-pillar 对照（只报数）

LIMITATIONS「朝真实仿真的下一步」第 1 条：把本仓库 `upstream` + 影子盾接到 PX4 SITL，用同一张 M0 场景卡出 L0 / SITL 对照。判据未冻结。SITL 的 `result.json` 仍是 `report_only=true`、`criteria_frozen=false`。下面的数字只报数。

不改 `core/`、`scenes/suite.txt`、套件里的通过门，也不写 PX4 的 `COM_` / `NAV_` / `BAT_` 参数。影子盾只记录 `would_reject`，不改批准动作。

## 场景卡与入口

`scenes/g2-pillar-sitl80.json` 与 `scenes/g2-pillar.json` 只有名字、`task.duration_s`（45 → 80）和一句 note 不同。几何、起点、终点和 `criteria` 相同。不进 `scenes/suite.txt`。80 s 是任务钟上限，含起飞和降落。

```bash
scripts/sitl/run_yopo_l0_sitl.sh --n 1 --scene scenes/g2-pillar-sitl80.json --policy upstream --seed 0
```

`--policy` 缺省就是 `upstream`。影子盾默认开，只读。深度走场景卡相机（`depth_sources=scene-card`），不依赖 Gazebo 里有没有柱子模型。定位 `default`。`px4_parameters_written` 全程是空列表。

L0 对照是同一张卡、同一组 seed，`python -m yopo_l0.run --scene scenes/g2-pillar-sitl80.json --policy upstream`（影子盾开）。L0 没有起飞段，到达时刻对应 SITL 的 `arrival_after_handover_s`（任务钟到达减去起飞交接）。

## 环境

box 重建走 `/workspace/yopo-ops/setup-sitl-box.sh`（2026-11-14 22:48–23:02 UTC-3）。镜像：

| 标签 | 角色 |
|---|---|
| `yopo-restart-sitl:s1b-23bc262f44d8` | S1b 基础镜像（PX4 v1.17.0 SITL + Gazebo Harmonic） |
| `yopo-restart-sitl:yopo-81d084c04186` | S4c YOPO L0 分层 |

## 对照表（seeds 0–9）

L0 在 22:51–22:54 UTC-3 跑完，10/10 成功。SITL 在 23:02–23:09 UTC-3 串行跑完，10/10 退出码 0，全部 `SUCCESS` / `GOAL_REACHED_AND_HELD` 并落地。

| seed | L0 到达 / s | SITL 交接 / s | SITL 交接后到达 / s | SITL 任务钟到达 / s | L0 碰撞 | SITL 碰撞 | L0 影子 would_reject | SITL 影子 would_reject | L0 最小净空 / m | SITL 最小净空 / m | SITL RTF |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 4.260 | 9.868 | 6.512 | 16.380 | 0 | 0 | 1.000 | 1.000 | 0.4790 | 0.0021 | 0.999 |
| 1 | 4.270 | 9.516 | 6.664 | 16.180 | 0 | 0 | 1.000 | 1.000 | 0.4789 | 0.0054 | 0.999 |
| 2 | 4.260 | 9.916 | 6.468 | 16.384 | 0 | 0 | 1.000 | 1.000 | 0.4789 | 0.0036 | 0.999 |
| 3 | 4.260 | 9.932 | 7.080 | 17.012 | 0 | 0 | 1.000 | 1.000 | 0.4787 | 0.0058 | 0.999 |
| 4 | 4.270 | 10.000 | 6.872 | 16.872 | 0 | 0 | 1.000 | 1.000 | 0.4794 | 0.0054 | 0.999 |
| 5 | 4.280 | 9.936 | 6.328 | 16.264 | 0 | 0 | 1.000 | 1.000 | 0.4780 | 0.0020 | 0.999 |
| 6 | 4.280 | 10.012 | 7.208 | 17.220 | 0 | 0 | 1.000 | 1.000 | 0.4778 | 0.0018 | 0.998 |
| 7 | 4.280 | 9.380 | 6.164 | 15.544 | 0 | 0 | 1.000 | 1.000 | 0.4775 | 0.0014 | 1.000 |
| 8 | 4.280 | 9.784 | 7.108 | 16.892 | 0 | 0 | 1.000 | 1.000 | 0.4777 | 0.0105 | 0.999 |
| 9 | 4.270 | 9.512 | 6.280 | 15.792 | 0 | 0 | 1.000 | 1.000 | 0.4789 | 0.0009 | 1.000 |

汇总：

| 项 | L0 | SITL |
|---|---|---|
| 到达 | 10/10 | 10/10，全部落地 |
| 碰撞 | 0 | 0 |
| 到达时刻 | 4.260–4.280 s（均值 4.271 s） | 任务钟 15.544–17.220 s（含起飞）；起飞交接 9.380–10.012 s；交接后 6.164–7.208 s（均值 6.669 s） |
| 影子 would_reject 率 | 1.000 | 1.000 |
| 最小表面净空 | 0.4775–0.4794 m | 0.0009–0.0105 m（见下） |
| RTF | 点质量墙钟比约 0.30，不是锁步 | 0.998–1.000 |

SITL 比 L0 多出约 9.4–10.0 s 的起飞交接。去掉这段之后，SITL 到达仍比 L0 慢约 1.9–2.9 s（PX4 + 一阶点质量不是同一套动力学）。影子拒绝率两边都是 1.000，和 LIMITATIONS 里「L0 上开盾时多为 1.000，不能当成失败门」的口径一致；SITL 直方图里除了 `ENV:UNKNOWN`，还有 `TRACKING_BOUND`、`LAG_VIOLATION` 等跟踪项，L0 没有后两项。

SITL 的 `min_surface_clearance_m` 不能直接和 L0 比。会话在降落段仍记账：高度刚超过机体半径 0.35 m 时，到地面的距离减去半径会把最小值压到接近 0。碰撞判定本身只在 `TAKEOFF` / `PLAN` 里做，这 10 次都是 0。L0 不降落，全程在 1.5 m，净空反映的是过柱。原始结果和逐 seed 直方图在 [s6-sitl-upstream-m0-runs.json](s6-sitl-upstream-m0-runs.json)。SITL 产物目录在 `/workspace/yopo-ops/sitl/runs/20261114T230215-yopo-n1` 起的 10 趟。

## 本环境实测

```bash
PYTHONPATH=".:core/src" /workspace/ys-venv/bin/python -m pytest tests/test_sitl_s6_upstream_m0.py tests/test_sitl*.py -q --tb=line
bash scripts/sitl/run_yopo_l0_sitl.sh --dry-run --n 1 --scene scenes/g2-pillar-sitl80.json --policy upstream
```

| 命令 | 结果 |
|---|---|
| `pytest tests/test_sitl*.py -q --tb=line` | **95 passed** in 45.42 s |
| 上面的 shell dry-run | 退出码 0，`policy=upstream`，`scene=scenes/g2-pillar-sitl80.json`，镜像 `yopo-restart-sitl:yopo-81d084c04186` |
| L0 seeds 0–9 | 10/10 `SUCCESS`，退出码 0 |
| SITL seeds 0–9 | 10/10 `SUCCESS`，退出码 0 |

`tests/test_sitl_s6_upstream_m0.py` 检查：对照卡只改了名字、时长和 note；dry-run 默认 `upstream` 且影子盾开；假 PX4 走完起飞后记下交接时刻和影子拒绝率；`shadow=False` 时拒绝率为空。没有在单元测试里拉起 Gazebo。
