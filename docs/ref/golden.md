# G0 基线金样（#3）

金样文件：`core/tests/golden/minco_d5e1e60_cpu.npz`

| 项 | 值 |
|---|---|
| 上游 | https://github.com/TJU-Aerial-Robotics/YOPO.git ，分支 YOPO-MINCO，提交 `d5e1e60f83cc991602f142d9b6b96204a24d6b9f`（ADR-0006） |
| 权重 | `YOPO/saved/YOPO_1/epoch50.pth`，sha256 `2d85e712c8874d11cd4e523777698eff78914fdb8f1ffb5068b037f8f972f62f`（ADR-0007）；不提交进本仓库，运行时从上游 git 取到缓存 |
| config_hash | `18de07db2c1482948b858ee32b64ddc5782d6c53a1e0ca2eb9dce5b7863281d8`（上游 `config.config.cfg` 全部字段的规范 JSON sha256） |
| torch | 2.4.1+cpu（`download.pytorch.org/whl/cpu`），numpy 1.26.4，Python 3.10 |
| 机器 | box：Debian 13 容器，8 vCPU，无 GPU；生成时 `torch.set_num_threads(1)` |
| 生成日期 | 2026-10-24 |

## 固定输入

- 深度：`numpy.random.Generator(PCG64(20261024)).random((1, 1, 96, 160), float32)`，取值 [0,1]（上游已归一化的深度张量格式）。不依赖 torch 随机数。
- 状态（9 维，机体系，未归一化）：速度 (1.0, 0.2, -0.1)、加速度 (0.1, 0, 0)、局部目标 (8.0, 1.0, 0.5)。`fixed_inputs` 返回的是这一组。
- 状态经上游 `StateTransform.normalize_obs` + `prepare_input` 后送入 `YopoNetwork.forward`，与上游 `inference()` 相同。
- npz 的 `obs` 键是 `forward` 返回前的同一块数组。`normalize_obs` 通过 `torch.from_numpy` 就地写入，速度和加速度除以 `UPSTREAM_DECODER` 的 6，目标除以 `max(||goal||, tail_radio_max=10)`。`compare_golden` 比对的是这份就地归一化后的 `obs`，不是未归一化的 `fixed_inputs` 返回值。`core/tests/test_golden_orin_record.py` 锁定这个关系。

## 输出与容差

- `traj_params` (1, 15, 14)：网络原始轨迹参数（tanh 后、未解码；解码在 #6）
- `score` (1, 15)：softplus 分数，越大越好
- `radius` (1, 15, 20)：预测走廊 μ 与置信 b，只作排序/告警
- 展平顺序与上游 `inference()` 一致：`permute(0, 2, 3, 1)`（vertical 行优先，再 horizon）
- 容差：`rtol=1e-4, atol=1e-5`。实测同机 1 线程与 8 线程最大绝对差 1.9e-6（score，量级 5.5），轨迹参数 4.2e-7。

## 重新生成

```bash
cd core
pip install -e '.[upstream,dev]'   # torch CPU 版请用 --index-url https://download.pytorch.org/whl/cpu
python -m yopo_swarm_core.baseline --write-golden
```

只有在有意变更上游 SHA、权重或固定输入时才重新生成，并在提交说明里写明原因。
平时运行 `python -m yopo_swarm_core.baseline` 会打印 SHA、权重哈希、config_hash 并比对金样；
`pytest -m upstream` 跑同样的检查（首次会把上游浅取到 `~/.cache/yopo-swarm`，可用 `YOPO_SWARM_CACHE` 改位置）。
离线时用 `pytest -m "not upstream"`。

## 解码金样（#6）

`core/tests/golden/minco_decode_d5e1e60.npz` 由 `PYTHONPATH=core/src python -m yopo_swarm_core.candidates --write-golden` 生成：

- 输入：上面 `minco_d5e1e60_cpu.npz` 里的 `traj_params / score / radius`（不重新前向）。
- 参考实现：上游 `StateTransform.pred_to_traj_params`（图像顺序）+ `MincoTraj.solve`；机体→世界按 `test_yopo_ros.py` 的方式锚定（固定 yaw 30°、位置 (1,2,3)、固定头 PVA），每条候选在 `[0, T)` 上取 11 个时刻的 p/v/a。
- 选择参考：`_corridor_mu_lo` + `select_action`（topk=1）逐行转写（上游节点依赖 rospy，无法直接调用）。
- 还存了 `decoder_config`（解码相关的上游配置项），离线测试用它核对 `UPSTREAM_DECODER`。
- 容差：rtol 1e-4、atol 1e-5（上游解码是 float32，本仓库 float64；实测最大差 ≈ 6e-7）。

## G3 双机有界穷举记录（#10）

- 文件：`docs/ref/g3-explore-2uav.json`（场景、计划配置、不变量清单、穷举范围与结果一起入库）。
- 场景：A (0,0,0)→(4,0,0)，B (2,-2,0)→(2,2,0)，两机航线在 (2,0,0) 交叉；真实计划占用
  （运动前缀 + 固定分支点制动分支 + 终端保持，ADR-0002），冲突判定走 #4 连续时间检查。
- 范围：max_steps=10、每机 1 个提案、队列深度 3、故障 drop/duplicate/reorder、时间步 0.5 s。
  队列超深的分支被剪掉并计数（`pruned_queue_depth`），这是有界枚举，不是证明。
- 重新生成：`PYTHONPATH=core/src python -m yopo_swarm_core.protocol.plan_occupancy`（在仓库根目录）。
  `tests/test_protocol_real.py::test_saved_exhaustive_record_is_reproducible` 校验入库记录与当前代码一致。
- 反例回归（同文件）：B01 COMMIT 部分丢失、超时释放、制动分支交叉、终点占用、已发 ACK 约束；
  每个都配一个去掉对应规则的变异体，参考检查器必须报违反。

## Python 3.12 兼容性（ADR-0010，2026-10-24，box）

| 环境 | 结果 |
|---|---|
| Python 3.12.15 + torch 2.4.1+cpu + numpy 1.26.4 + scipy 1.17.1 | `ruff check`、`ruff format --check` 通过；全量 `pytest` 301 passed（3 分 42 秒） |
| Python 3.12.15 + torch 2.12.0+cpu + numpy 1.26.4 + scipy 1.17.1 | 全量 `pytest -W default` 301 passed（3 分 37 秒），1 个 ResourceWarning 来自上游 `traj_opt.yaml` 未关闭文件，与本仓库代码无关；`python -m yopo_swarm_core.baseline` 金样 PASS；1 线程下 `traj_params` / `score` / `radius` 与金样最大绝对差均为 0（逐位一致） |

逐位一致只代表 x86 CPU 单线程；aarch64、CUDA、TensorRT 仍按 `docs/deploy/orin-nano.md` §4 的阶梯比容差。

## Orin GPU 金样阶梯（#11）

记录日期：**2026-10-24**（台北时间，UTC+8）；采集 UTC 时间见本机 `docker/orin/out/run_info.txt`（2026-10-24 15:20 起）。
原始报告：[golden_gpu_orin.json](golden_gpu_orin.json)，由最终镜像的 `validate` 输出原样复制。
版本取自同轮 `docker/orin/out/manifest.json`。

| 项 | 实测 / 构建值 |
|---|---|
| 镜像 | `yopo-swarm:orin-infer-cu13`，基础镜像 `nvidia/cuda:13.2.1-cudnn-runtime-ubuntu24.04`（aarch64），不装 ROS |
| 宿主 / GPU | L4T R39.2.1，驱动 595.78，Orin / sm_87；功耗模式 MAXN_SUPER，未修改时钟设置 |
| Python | 3.12.3 |
| torch | 2.12.0+cu132 |
| CUDA（torch） | 13.2 |
| cuDNN（torch 版本码） | 92000（9.20.0） |
| TensorRT | 10.16.1.11 |
| numpy | 1.26.4 |

完整 `validate --out /out` 退出码 0：manifest PASS，pytest **299 passed / 2 deselected**（排除 slow），CPU 基线 PASS，GPU 阶梯 PASS。

`max_abs` 顺序为 traj_params / score / radius；延迟单位 ms：

| 级别 | 状态 | max_abs | p50 / p99 |
|---|---|---|---|
| L0_cpu_fp32 | PASS | 7.4505806e-07 / 1.90734863e-06 / 2.38418579e-07 | 54.812512 / 60.305009 |
| L1_cuda_fp32 | PASS | 5.06639481e-07 / 3.09944153e-06 / 2.38418579e-07 | 7.809522 / 10.171237 |
| L2_trt_fp32 | PASS | 6.97374344e-06 / 1.90734863e-06 / 2.38418579e-07 | 2.288300 / 2.538983 |
| L3_trt_fp16 | PASS | 0.00242966413 / 0.0104341507 / 0.000888675451 | 0.806330 / 0.836722 |

- L0/L1/L2 均通过原有 `rtol=1e-4 / atol=1e-5`，未放宽 FP32 容差。ONNX 导出与 ONNX CPU 对照也 PASS。
- **FP16 决策级等价：true**，该固定输入的最高分候选索引与 CPU 金样一致。
  FP16 数值参考判据 `rtol=1e-2 / atol=1e-3` 的 `pass_tol` 也为 true；硬判据仍是最高分候选一致，仅代表这个固定输入。
- 计时：batch=1，深度 96×160，预热 1 次后取 50 次同步墙钟采样；CPU 单线程。
  L0 不含预处理；L1 含每次输入 CPU→GPU 搬运；L2/L3 使用预分配 GPU 输入输出，不含 engine 构建和输入输出搬运。
  这些数字不代表完整规划链路延迟。
- 首轮发现上游格点单例自动选 CUDA，使 CPU 输入与 CUDA 变换常量混用，出现 4 项测试失败和阶梯入口异常。
  已在 CPU 测试/L0 的独立进程隐藏 GPU，并在阶梯内将预处理缓存放到 CPU；L1/L2/L3 仍在真实 GPU 上执行。
  没有改上游源码、YOPO 网络、权重或金样。
- torch 2.12.0 发出 sm_87 支持警告，但本次 CUDA 前向和 TRT 两级实跑均通过，保留该版本；TensorRT 10.16.1.11 也无需回退或 API 补丁。
- 构建日志：`~/yopo-work/logs/build_infer.log`；本机完整产物与遥测在 `docker/orin/out/`，首轮失败证据在其 `attempt-initial/` 子目录。

补充检查：`pip check` **FAIL（rc=1）**，报告 `nvidia-cusparselt-cu13 0.8.1 is not supported on this platform`。
实读该包 `WHEEL` 为 `Tag: py3-none-manylinux2014_sbsa`，pip 支持的是 aarch64 标签；库文件 ELF `e_machine=183`（AArch64）。
这是发行包标签不匹配；本次 L0–L3 实跑通过，保留指定版本且不修改第三方元数据。cuSPARSELt 稀疏算子未单独验证。
