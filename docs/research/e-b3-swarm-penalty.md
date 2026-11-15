# E-B3：后段机间椭球惩罚

收口说明见 [e-b3-closeout.md](e-b3-closeout.md)：这条实现线已收口，不再开后续调参或叠加欧氏铰链的实现票。

在已合入的 E-B2 / E-B2p 均匀三次 B 样条后段上加邻机项。邻机轨迹来自 L0 总线已提交计划的采样（`yopo_l0/bus.py` 的 `peer_plan`）。代价按论文式 (5)(6) 的角色重写：椭球距离上的平方铰链，不复制 `ego-planner-swarm`（GPLv3）。

开关默认关。关上时不广播采样、不读邻机、不改后段代价，全仓结果应与加这项之前逐位相同。不改 `core/`、已冻门、`scenes/suite.txt`、默认开关。不冻结新判据。

## 假设（不是论文复现）

论文正文只写椭球系数 `c > 1`、间隙 `C` 由用户定，没有给数值。本票自定如下，并在场景覆盖和结果里标注「这是假设，不是论文复现」：

| 符号 | 取值 | 含义 |
|---|---|---|
| `C`（`swarm_clearance_m`） | 0.80 m | 铰链门槛。椭球距离 `d < C` 时加罚 |
| `c`（`swarm_c`） | 2.0 | 竖直压缩。`d = sqrt(dx² + dy² + (c·dz)²)`，竖直半轴是 `C/c = 0.40` m，水平半轴是 `C` |
| `λ_w` | 40 | 写在求解器里，不进场景卡。与后段障碍项 `λ_c = 40` 同一量级。论文 `λ_w = 0.5` 的距离量纲对不上米制铰链，对不上，不冒充论文表 |

机体中心距 ≤ 0.70 m（两倍机体半径 0.35 m）仍是 L0 的机间碰撞线，本票不改这条线。`C = 0.80` m 只让惩罚在碰撞线之外就开始推。

控制点 `i` 对应时刻 `t_start + (i - 1)·dt`（均匀三次样条的 Greville 横坐标），夹到本条曲线区间。只罚可动的中间控制点。邻机采样没有时间重叠时不外推。

## 开关

写在 `policy_params.backend`。`enabled` 和 `swarm_penalty` 都必须是 JSON true 才打开椭球项。只开 `enabled` 就是原来的 E-B2，没有机间项。

```json
"policy_params": {
  "backend": {
    "enabled": true,
    "swarm_penalty": true,
    "swarm_clearance_m": 0.80,
    "swarm_c": 2.0
  }
}
```

已提交的场景卡都不写这些字段。对照脚本在进程里覆盖，不改 `scenes/ego-density-014-swap3.json` 和 `scenes/swap-8-open.json`。

goto / upstream：打开 E-B3 时，运行器才把已提交轨迹采样放进既有的 `peer_plan`；策略收到后交给后段。穿林舰队从协议节点里已提交运动段取样，不另开话题。E-A1 和 E-B3 都关时，载荷与以前相同。

## 对照怎么跑

小批量只报数，种子 0–9。臂：

- `off`：默认全关
- `b2`：只开后段，用来把 B3 的作用和后段本身分开
- `b3`：后段打开，并打开机间椭球

```bash
.venv/bin/python scripts/e-b3-swarm-penalty.py \
  --scenes d014-swap3,swap8 --arms off,b2,b3 --seeds 0,1,2,3,4,5,6,7,8,9 \
  --jobs-plain 2 --jobs-backend 1
```

每粒结果落在 `out/e-b3-swarm-penalty/partial/`，可续跑。墙钟不参与轨迹。`backend_budget_cuts` 是后段 0.02 s 墙钟预算把迭代截断的次数。原始行见 [e-b3-swarm-penalty-runs.json](e-b3-swarm-penalty-runs.json)。

目标场景来自已有对照：密度 0.14 三机对向在 #59 里两臂都是 10/10 次 uav1–uav2 机间碰撞，最小 0.634 m；八机对向用 `scenes/swap-8-open.json`。

## 结果

<!-- e-b3-table:start -->
完成 60/60 粒。C=0.8 m，c=2.0，这是假设，不是论文复现。C、c 是假设，不是论文复现。墙钟不参与轨迹。backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。

## 开 vs 关（scripts/e-b3-swarm-penalty.py）

| 场景 | 臂 | 到达 | 撞树 | 机间碰撞 | 超时 | 最小机间 m | 后段 p50 / p95 ms | 预算截断 |
|---|---|---|---|---|---|---|---|---|
| d014-swap3 | off | 0/30 | 0 | 10 | 0 | 0.622–0.698 | — | 0/0 |
| d014-swap3 | b2 | 0/30 | 0 | 10 | 0 | 0.608–0.690 | 9.4–11.9 / 23.1–29.1 | 85/810 |
| d014-swap3 | b3 | 0/30 | 0 | 10 | 0 | 0.629–0.697 | 9.1–12.5 / 18.8–29.5 | 68/810 |
| swap8 | off | 0/80 | 0 | 10 | 0 | 0.695–0.696 | — | 0/0 |
| swap8 | b2 | 0/80 | 0 | 10 | 0 | 0.659–0.699 | 7.0–23.4 / 17.6–36.4 | 508/1680 |
| swap8 | b3 | 0/80 | 0 | 10 | 0 | 0.674–0.698 | 11.4–23.7 / 29.1–35.5 | 743/1744 |

### 分种子

| 场景 | 臂 | seed | 结果 | 到达 | 撞树 | 机间 | 最小机间 m | 后段 p95 ms |
|---|---|---|---|---|---|---|---|---|
| d014-swap3 | off | 0 | COLLISION | 0/3 | 0 | 1 | 0.634 | — |
| d014-swap3 | off | 1 | COLLISION | 0/3 | 0 | 1 | 0.639 | — |
| d014-swap3 | off | 2 | COLLISION | 0/3 | 0 | 1 | 0.629 | — |
| d014-swap3 | off | 3 | COLLISION | 0/3 | 0 | 1 | 0.698 | — |
| d014-swap3 | off | 4 | COLLISION | 0/3 | 0 | 1 | 0.622 | — |
| d014-swap3 | off | 5 | COLLISION | 0/3 | 0 | 1 | 0.684 | — |
| d014-swap3 | off | 6 | COLLISION | 0/3 | 0 | 1 | 0.683 | — |
| d014-swap3 | off | 7 | COLLISION | 0/3 | 0 | 1 | 0.658 | — |
| d014-swap3 | off | 8 | COLLISION | 0/3 | 0 | 1 | 0.666 | — |
| d014-swap3 | off | 9 | COLLISION | 0/3 | 0 | 1 | 0.629 | — |
| d014-swap3 | b2 | 0 | COLLISION | 0/3 | 0 | 1 | 0.631 | 26.5 |
| d014-swap3 | b2 | 1 | COLLISION | 0/3 | 0 | 1 | 0.669 | 23.1 |
| d014-swap3 | b2 | 2 | COLLISION | 0/3 | 0 | 1 | 0.609 | 29.1 |
| d014-swap3 | b2 | 3 | COLLISION | 0/3 | 0 | 1 | 0.686 | 25.8 |
| d014-swap3 | b2 | 4 | COLLISION | 0/3 | 0 | 1 | 0.608 | 26.9 |
| d014-swap3 | b2 | 5 | COLLISION | 0/3 | 0 | 1 | 0.690 | 25.5 |
| d014-swap3 | b2 | 6 | COLLISION | 0/3 | 0 | 1 | 0.683 | 25.1 |
| d014-swap3 | b2 | 7 | COLLISION | 0/3 | 0 | 1 | 0.661 | 25.8 |
| d014-swap3 | b2 | 8 | COLLISION | 0/3 | 0 | 1 | 0.663 | 26.6 |
| d014-swap3 | b2 | 9 | COLLISION | 0/3 | 0 | 1 | 0.635 | 25.6 |
| d014-swap3 | b3 | 0 | COLLISION | 0/3 | 0 | 1 | 0.697 | 18.8 |
| d014-swap3 | b3 | 1 | COLLISION | 0/3 | 0 | 1 | 0.680 | 26.5 |
| d014-swap3 | b3 | 2 | COLLISION | 0/3 | 0 | 1 | 0.629 | 21.2 |
| d014-swap3 | b3 | 3 | COLLISION | 0/3 | 0 | 1 | 0.651 | 19.9 |
| d014-swap3 | b3 | 4 | COLLISION | 0/3 | 0 | 1 | 0.635 | 22.0 |
| d014-swap3 | b3 | 5 | COLLISION | 0/3 | 0 | 1 | 0.639 | 21.6 |
| d014-swap3 | b3 | 6 | COLLISION | 0/3 | 0 | 1 | 0.633 | 19.4 |
| d014-swap3 | b3 | 7 | COLLISION | 0/3 | 0 | 1 | 0.630 | 27.2 |
| d014-swap3 | b3 | 8 | COLLISION | 0/3 | 0 | 1 | 0.679 | 29.4 |
| d014-swap3 | b3 | 9 | COLLISION | 0/3 | 0 | 1 | 0.668 | 29.5 |
| swap8 | off | 0 | COLLISION | 0/8 | 0 | 1 | 0.695 | — |
| swap8 | off | 1 | COLLISION | 0/8 | 0 | 1 | 0.695 | — |
| swap8 | off | 2 | COLLISION | 0/8 | 0 | 1 | 0.695 | — |
| swap8 | off | 3 | COLLISION | 0/8 | 0 | 1 | 0.696 | — |
| swap8 | off | 4 | COLLISION | 0/8 | 0 | 1 | 0.695 | — |
| swap8 | off | 5 | COLLISION | 0/8 | 0 | 1 | 0.695 | — |
| swap8 | off | 6 | COLLISION | 0/8 | 0 | 1 | 0.696 | — |
| swap8 | off | 7 | COLLISION | 0/8 | 0 | 1 | 0.695 | — |
| swap8 | off | 8 | COLLISION | 0/8 | 0 | 1 | 0.695 | — |
| swap8 | off | 9 | COLLISION | 0/8 | 0 | 1 | 0.695 | — |
| swap8 | b2 | 0 | COLLISION | 0/8 | 0 | 1 | 0.661 | 24.7 |
| swap8 | b2 | 1 | COLLISION | 0/8 | 0 | 1 | 0.699 | 27.4 |
| swap8 | b2 | 2 | COLLISION | 0/8 | 0 | 1 | 0.660 | 18.9 |
| swap8 | b2 | 3 | COLLISION | 0/8 | 0 | 1 | 0.660 | 17.6 |
| swap8 | b2 | 4 | COLLISION | 0/8 | 0 | 1 | 0.659 | 17.9 |
| swap8 | b2 | 5 | COLLISION | 0/8 | 0 | 1 | 0.666 | 32.6 |
| swap8 | b2 | 6 | COLLISION | 0/8 | 0 | 1 | 0.680 | 36.4 |
| swap8 | b2 | 7 | COLLISION | 0/8 | 0 | 1 | 0.678 | 35.5 |
| swap8 | b2 | 8 | COLLISION | 0/8 | 0 | 1 | 0.675 | 34.7 |
| swap8 | b2 | 9 | COLLISION | 0/8 | 0 | 1 | 0.696 | 34.0 |
| swap8 | b3 | 0 | COLLISION | 0/8 | 0 | 1 | 0.693 | 35.5 |
| swap8 | b3 | 1 | COLLISION | 0/8 | 0 | 1 | 0.686 | 33.5 |
| swap8 | b3 | 2 | COLLISION | 0/8 | 0 | 1 | 0.695 | 33.4 |
| swap8 | b3 | 3 | COLLISION | 0/8 | 0 | 1 | 0.698 | 35.4 |
| swap8 | b3 | 4 | COLLISION | 0/8 | 0 | 1 | 0.687 | 31.3 |
| swap8 | b3 | 5 | COLLISION | 0/8 | 0 | 1 | 0.683 | 35.4 |
| swap8 | b3 | 6 | COLLISION | 0/8 | 0 | 1 | 0.680 | 31.5 |
| swap8 | b3 | 7 | COLLISION | 0/8 | 0 | 1 | 0.685 | 29.7 |
| swap8 | b3 | 8 | COLLISION | 0/8 | 0 | 1 | 0.690 | 34.7 |
| swap8 | b3 | 9 | COLLISION | 0/8 | 0 | 1 | 0.674 | 29.1 |
<!-- e-b3-table:end -->

## 测试

`tests/test_backend_swarm.py`：开关缺省关；已有场景卡都不开这项；关上时把邻机采样传进 `refine_yopo` 与不传逐位相同；打开但没有邻机、或邻机很远，与关上相同；近邻机把曲线推开；椭球项的解析梯度与有限差分一致；现有对向卡在关闭时总线 `peer_plan` 不含采样，打开后才带采样并写入 `backend` 块。

## L0-DET 确定性重跑（2026-11-15）

当前 main `046d25a`。后段缺省 `budget="evals"`（0.020 s → 10 次评估）。卡、臂、种子与上面的命令相同；`--jobs-plain 2 --jobs-backend 2`。连跑两遍，去掉墙钟字段后逐位相同。全关臂与上文逐种子相同。

| 场景 | 臂 | 旧到达 | 新到达 | 旧机间碰撞 | 新机间碰撞 | 旧最小机间 m | 新最小机间 m | 新预算截断 |
|---|---|---|---|---|---|---|---|---|
| d014-swap3 | b2 | 0/30 | 0/30 | 10 | 10 | 0.608–0.690 | 0.601–0.691 | 265/810 |
| d014-swap3 | b3 | 0/30 | 0/30 | 10 | 10 | 0.629–0.697 | 0.633–0.681 | 261/810 |
| swap8 | b2 | 0/80 | 0/80 | 10 | 10 | 0.659–0.699 | 0.666–0.667 | 265/1680 |
| swap8 | b3 | 0/80 | 0/80 | 10 | 10 | 0.674–0.698 | 0.677–0.697 | 791/1760 |

结论没变。两张卡、三臂仍是机间碰撞 10/10，到达是 0。椭球项没有把对向撞机降下来。变的是各粒的最小机间距离：swap8 的 b2 从 0.659–0.699 m 收到 0.666–0.667 m，十粒都贴在同一处，不再有上文接近 0.699 m 的那几粒。
