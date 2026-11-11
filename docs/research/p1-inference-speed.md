# P1：YOPO 单次推理耗时

结论先写在这里。

E-C3 的 `plan_wall_ms` 只包住 `BaselineModel.forward`。盒子（8 vCPU、无 GPU）上单机 `g2-pillar` seed 0 的均值 10.5 ms、p95 12.5 ms，就是这一段。本机 4 核上同一口径的基线是均值 11.63 ms、p50 11.58 ms、p95 13.24 ms。拆开之后大约 93% 是 ResNet18 骨干。深度渲染、预处理、解码和打分都不在这 10.5 ms 里面。渲染一帧大约 36 ms，比前向还长，但是另一段墙钟。

默认只打开逐位相同的 `torch.inference_mode`，仍然是 1 个 intra-op 线程。五张会调用网络的场景上，轨迹和基线逐位相同。场景内的均值差落在单次运行的噪声里，不能当成稳定的加速。

4 线程和 ONNX Runtime（4 线程）能把同一批输入的前向从大约 8 ms 降到 3.9 ms 和 1.7 ms。单机和三机上位置差在 1e-5 m 到 2 mm。六机中等林会换成另一条轨迹，位置差 16–18 m，到达时刻也变。疏林上 ORT 的位置差到 1.5 m。这两档留在 `YOPO_INFER_ACCEL` 里，默认不打开。权重、冻结门、`suite.txt`、README、`docs/tickets.md` 都没改。

CUDA：未测量。本机 `torch.cuda.is_available()` 为 False，没有 GPU。

## 机器

和 E-C1 那次 EGO-Swarm 基线是同一类虚拟机。EGO 的数字在分支 `cursor/ego-swarm-baseline-cbba` 的 `docs/research/ego-swarm-baseline.md`：401 次重规划，均值 1.72 ms，p95 6.3 ms。那份文件不在本分支，这里不做成链接。

| 项 | 值 |
|---|---|
| CPU | `Intel(R) Xeon(R) Processor`，GenuineIntel，family 6，model 207，stepping 2。管理程序把品牌字符串收成这一句 |
| 核 | 1 路 4 核，每核 1 线程，KVM |
| 内存 | 15 GiB |
| 系统 | Linux 6.12.113+，x86_64 |
| Python | 3.12.3 |
| torch | 2.4.1+cpu |
| numpy | 2.5.4 |
| onnxruntime | 1.31.1，只在测 `ort` 时装上，没有写进 `requirements.txt` |
| GPU | 无。CUDA 未测量 |

盒子上的 10.5 / 12.5 是 E-C3 的记录，不是这次重测的。本机核数少一倍，基线均值高大约 1 ms，用来看比例，不把两个均值当成同一次实验。

## 1. 时间花在哪

`yopo_l0/net.py` 的 `forward_raw` 用 `plan_wall.measure()` 包住一次 `run_forward`。渲染在 `DepthCamera.capture`，预处理是 `network_depth` 和 `body_obs`，解码打分是 `decode_params`、`corridor_mu_lo`、`select_action`、`minco_two_piece`。这些都不在 `plan_wall_ms` 里。

下面是 `g2-pillar` seed 0、基线（`no_grad`、1 线程）、影子盾开、飞完到达。21 次网络调用，53 帧出图。前向内部用了几只 `perf_counter`，合计 11.70 ms；同一条轨迹上不插探针的 `plan_wall_ms` 均值是 11.63 ms。探针大约 0.1 ms，比例按 11.70 ms 算。

| 环节 | 均值 ms | p50 | p95 | 在不在 plan_wall 里 |
|---|---|---|---|---|
| 深度渲染（每帧） | 35.62 | 35.35 | 37.25 | 否。320×240，6 个盒子加地面 |
| 深度预处理 `network_depth` | 0.181 | 0.177 | 0.237 | 否 |
| 观测 `body_obs` | 0.023 | 0.022 | 0.025 | 否 |
| 网络前向 | 11.70 | 11.75 | 12.52 | 是。这就是 E-C3 的那一项 |
| 解码 `decode_params` | 0.096 | 0.093 | 0.121 | 否 |
| 走廊加选优 | 0.049 | — | — | 否。走廊 0.021，选优 0.029 |
| 两段 MINCO | 0.075 | 0.074 | 0.092 | 否 |
| `plan()` 里其余 Python | 0.33 | 0.34 | 0.39 | 否。机体到世界的旋转和构造轨迹对象 |

前向内部：

| 子项 | 均值 ms | 占前向 |
|---|---|---|
| ResNet18 骨干 | 10.92 | 93.3% |
| 状态变换 `prepare_input` | 0.359 | 3.1% |
| 头，以及 tanh / softplus / sigmoid | 0.283 | 2.4% |
| 输出 permute 和拷回 numpy | 0.057 | 0.5% |
| numpy 转 tensor | 0.038 | 0.3% |
| 前向里其余 Python | 0.043 | 0.4% |

把同一批 21 帧从飞行里抽出来连着重放，1 线程前向均值是 8.45 ms（p50 8.37，p95 8.98）。比场景里的 11.6 ms 少大约 3 ms。少掉的不是另一段 Python，是渲染刚走过缓存之后，同一次卷积变慢了。第 2 节的对照表把 OpenMP 线程钉成 1，那一列的基线是 8.14 ms，和 8.45 ms 是两次隔离重放，不是场景内的 `plan_wall_ms`。

悬停和编队不调用这个网络，`plan_wall_ms` 的 count 是 0，没有可拆的 10.5 ms。

## 2. 试过、而且不改权重的做法

隔离重放用的是上面那 21 帧，进程里 `OMP_NUM_THREADS=1`、interop 线程 1。和基线比的是网络输出的最大绝对差。g2 上这 21 次的最小分数间隙是 1.51e-2，单次分数误差在 1e-6 量级时不会换候选。

| 做法 | 隔离均值 ms | 相对 1 线程 no_grad | 闭环 |
|---|---|---|---|
| `no_grad`，1 线程 | 8.14 | 逐位相同，这是基线 | 基线 |
| `inference_mode`，1 线程 | 8.05 | 逐位相同 | 五张场景都逐位相同。默认打开 |
| `inference_mode`，2 线程 | 5.43 | 分数 2.9e-6，轨迹参数 3.9e-7，argmax 没有翻 | 没有单独跑闭环。4 线程已经代表这一档 |
| `inference_mode`，4 线程 | 3.88 | 分数 2.4e-6，轨迹参数 6.0e-7，argmax 没有翻。连跑两次逐位相同 | 见第 3 节。六机中等林换了轨迹 |
| channels last，1 线程 | 15.38 | 更慢，也不是逐位 | 不留 |
| channels last，4 线程 | 5.04 | 比 NCHW 的 4 线程慢 | 不留 |
| 21 帧叠成一批，1 线程，摊到每帧 | 6.11（整批 128 ms） | 分数 4.3e-6，不是逐位 | 在线一次只有一帧。15 个候选已经在同一次前向的 3×5 格子里，没有第二遍可并 |
| 21 帧叠成一批，4 线程，摊到每帧 | 2.44（整批 51 ms） | 同上 | 不接入飞行循环 |
| JIT trace + freeze + optimize_for_inference，4 线程 | 2.56 | 分数 4.8e-6 | g2 闭环候选翻了 2 次，位置差 9.7 cm，到达 4.28 s，基线是 4.26 s。不留 |
| ONNX Runtime，1 线程 | 5.49 | 分数 4.8e-6，轨迹参数 9.2e-7 | 没有单独跑。4 线程是测过闭环的那一档 |
| ONNX Runtime，4 线程 | 1.72 | 同上 | 见第 3 节。六机两张林都换了轨迹 |
| CUDA | 未测量 | — | 本机没有 GPU |

缓存和预处理：

- 权重和 `StateTransform` 的格点本来就只建一次。
- `network_depth` 整段 0.18 ms，里面的缩放索引没有单独再砍。
- 相机射线网格每次重建要 1.55 ms，占 36 ms 渲染的一小段。把六个盒子的 slab 改成复用缓冲，在一个位姿上和原来逐位相同，微基准从 41 ms 到 34 ms。射线求交仍是大头，而且这段不在 `plan_wall_ms` 里，所以没有改渲染器。

容差按这个定：同一 seed、tick 数相同、到达时刻相同，并且全部位置分量的最大绝对差 ≤ 1e-4 m。理由是金样 atol 1e-5 这一档的分数误差，积出来的位置差应该远小于机体半径 0.35 m 和到达门 0.4 m。超过这个，或者到达时刻变了，就写成不是同一条轨迹。

## 3. 各场景的前后

seed 0，影子盾开，场景卡上的全长，不缩短。`plan_wall_ms` 是场景内每次前向的墙钟。位置差是相对基线的分量最大绝对差。`infer` 是这次默认打开的那一档。

悬停、编队不调用网络，count 为 0，前后一样，不进表。

### 默认：`inference_mode`，1 线程

| 场景 | 次数 | 基线 均值 / p50 / p95 | 默认 均值 / p50 / p95 | 位置差 | 到达 |
|---|---|---|---|---|---|
| g2-pillar | 21 | 11.63 / 11.58 / 13.24 | 11.00 / 11.05 / 11.62 | 0 | 都是 4.26 s |
| crossing-pillars | 18 | 12.60 / 12.33 / 14.52 | 12.66 / 12.27 / 14.82 | 0 | 都是 7.06 s |
| crossing-forest | 45 | 12.16 / 12.03 / 12.93 | 13.05 / 12.16 / 21.31 | 0 | 都是 23.32 s |
| crossing-6-forest | 82 | 13.80 / 12.47 / 14.86 | 12.40 / 12.02 / 14.32 | 0 | 都是 26.08 s |
| crossing-6-forest-sparse | 85 | 12.20 / 11.93 / 14.10 | 12.30 / 11.89 / 14.64 | 0 | 都是 25.40 s |

轨迹逐位相同，所以均值差是墙钟噪声。三机林的 p95 从 12.93 到 21.31，六机中等林基线有一次 96 ms 的尖峰把均值拉到 13.80（p50 仍是 12.47）。看 p50，默认和基线差在 0.5 ms 以内。

### 开关：4 线程，以及 ONNX Runtime 4 线程

这两列的墙钟可以看，但六机上的样本已经不是基线那条轨迹，不能当成同一条飞行的加速比。隔离重放才是同一批输入上的速度。

| 场景 | 4 线程 均值 / p50 / p95 | 位置差 | 到达 | ORT 均值 / p50 / p95 | 位置差 | 到达 |
|---|---|---|---|---|---|---|
| g2-pillar | 6.46 / 6.66 / 7.60 | 9.0e-6 m | 同为 4.26 s | 3.32 / 3.41 / 3.54 | 5.8e-6 m | 同为 4.26 s |
| crossing-pillars | 7.51 / 6.32 / 11.45 | 2.1e-5 m | 同为 7.06 s | 3.00 / 3.01 / 3.15 | 3.3e-5 m | 同为 7.06 s |
| crossing-forest | 7.59 / 6.79 / 12.25 | 2.2e-3 m | 同为 23.32 s | 3.81 / 3.50 / 5.35 | 1.2e-5 m | 同为 23.32 s |
| crossing-6-forest | 7.67 / 6.70 / 12.98 | 17.8 m | 25.21 s，基线 26.08 s | 3.35 / 3.03 / 4.49 | 16.4 m | 30.49 s，基线 26.08 s |
| crossing-6-forest-sparse | 6.97 / 6.13 / 10.56 | 8.1e-4 m | 同为 25.40 s | 3.52 / 3.21 / 4.69 | 1.53 m | 时刻仍是 25.40 s，轨迹不是同一条 |

4 线程在三机林上是 2.2 mm，疏林是 0.8 mm，到达时刻没变。即便把容差放到 1 cm，六机中等林的 18 m 仍然过不了。ORT 在疏林上偏 1.5 m，中等林偏 16 m。单次前向的 argmax 在 g2 的 21 帧上没有翻，换路发生在后面的帧：分数间隙比 1e-2 更紧时，1e-6 的分数误差就够换一个候选，后面的深度跟着变。

所以 4 线程和 ORT 不作为通过一致性检查的加速。开关留着，是为了按下面的命令把这张表再跑出来。

## 4. 复现

```bash
.venv/bin/python scripts/p1-infer-speed.py scenes/g2-pillar.json --seed 0
YOPO_INFER_ACCEL=baseline .venv/bin/python scripts/p1-infer-speed.py scenes/g2-pillar.json --seed 0
YOPO_INFER_ACCEL=threads YOPO_INFER_THREADS=4 .venv/bin/python scripts/p1-infer-speed.py scenes/crossing-6-forest.json --seed 0
YOPO_INFER_ACCEL=ort YOPO_INFER_THREADS=4 .venv/bin/python scripts/p1-infer-speed.py scenes/crossing-6-forest.json --seed 0
```

`ort` 会在第一次构造推理源时把网络导出到缓存目录，导出不进 `plan_wall_ms`。没装 `onnxruntime` 时这一档直接报错。默认不导出。
