# P1b：深度相机求交加速（默认关）

结论先写在这里。

`YOPO_SENSOR_ACCEL` 默认是 `"0"`。只有 `"1"` 才走批量求交。关掉时仍是原来的逐障碍循环，深度和关掉之前逐位相同。射线网格按 `CameraConfig` 缓存，两边共用，数组只读。斜圆柱和地面不开批量，仍走原来的函数。噪声还是每帧整幅一次 `rng.normal`，抽取次数不随几何变。

这次测下来，开和关的闭环位置差是 0，到达时刻不变。渲染本身：`g2-pillar` 每帧 p50 从 34.52 ms 到 6.98 ms；三机林从 34.01 ms 到 12.17 ms；六机林从 33.88 ms 到 12.59 ms。回合墙钟在两张林上大约减半（96 s 到 42 s，218 s 到 109 s）。`g2-pillar` 只有 53 帧，墙钟从 15.5 s 到 13.1 s，渲染不是这 15 s 的大头。

开关默认关。下面的数字是 report-only：不改权重，不改冻结门，不改 `suite.txt`、`README.md`、`docs/tickets.md`、`scripts/regress.sh`。不拿这些毫秒当合入门。

## 机器

和 [p1-inference-speed.md](p1-inference-speed.md) 同一类虚拟机。计时在项目的 `.venv` 里，参考 BLAS，没有把 OpenMP 线程数设出来。

| 项 | 值 |
|---|---|
| CPU | `Intel(R) Xeon(R) Processor`，GenuineIntel，family 6，model 207，stepping 2。管理程序把品牌字符串收成这一句 |
| 核 | 1 路 4 核，每核 1 线程，KVM |
| 内存 | 15.6 GiB |
| 系统 | Linux 6.12.113+，x86_64 |
| Python | 3.12.3 |
| numpy | 2.5.4 |
| torch | 2.4.1+cpu。闭环里网络把 intra-op 线程钉成 1。单帧计时发生在导入 torch 之前 |
| BLAS | `/usr/lib/x86_64-linux-gnu/blas/libblas.so.3.12.0`，参考实现，单线程 |
| 线程环境 | `OMP_NUM_THREADS`、`MKL_NUM_THREADS`、`OPENBLAS_NUM_THREADS` 都没有设置 |
| GPU | 本机没有用来跑这次计时 |

跑完时 1 分钟 loadavg 是 1.0。表里的 p50 和最小值贴得很近，不是被抢占拉出来的单次尖峰。

## 改了什么

只动 `yopo_l0/sensor.py`。

1. **射线网格缓存。** `_ray_dirs_cam(cfg)` 按冻结的 `CameraConfig` 放进 `lru_cache`（自带锁）。返回的数组 `writeable` 为假。世界系方向仍是每帧一次矩阵乘。关和开都走这份缓存，数值和当场 `meshgrid` 相同。
2. **`YOPO_SENSOR_ACCEL=1` 时批量求交。** 多个盒子用同一套 slab，三个轴分开归约，和逐盒 `_hit_box` 逐位相同。多个竖直圆柱用和 `_hit_cylinder` 竖直分支相同的二次式，一次算一批。大窗并成一次全图批量；小窗按图像上的保守包围盒合成几簇再批量。完全在相机后面的障碍直接跳过。窗算大了只是多做几个本来就是 inf 的像素。
3. **不进批量的。** 斜圆柱仍走 `_hit_cylinder_lean`。地面仍走 `_hit_ground`。`min_range` / `max_range` 的裁剪和 NaN 掩码还在求完交之后，和原来同一行。

`crossing-6-forest` 和 `crossing-forest` 是同一片 medium、seed 7 的林：16 个盒子（5 面墙加 11 根矮枝）、23 根竖直树干、3 根斜干、1 个地面。相机 160×96，噪声标准差 0.02 m。`g2-pillar` 是 6 个盒子加地面，相机 320×240，噪声同样 0.02 m。

## 1. 单帧

`scripts/p1b-sensor-bench.py` 对 `crossing-6-forest` 的障碍各拍 30 次。`woods` 在林带中段（x = 20 m，yaw = 0），`start` 是六机 `uav0` 的起点。两次都先热身 2 帧。关和开都已经命中射线缓存，所以这张表比的是求交，不是网格重建。

| 位姿 | 关 p50 / 均值 / p95 | 开 p50 / 均值 / p95 | p50 之比 |
|---|---|---|---|
| 中段 x=20 m | 36.61 / 36.81 / 37.81 | 15.90 / 15.94 / 16.84 | 2.3× |
| 起点 (6, 1.5, 1.5) | 39.13 / 39.25 / 40.02 | 15.65 / 15.68 / 15.90 | 2.5× |

中段比闭环里的平均帧慢，是因为这一帧正对着密的一段，小窗里的树干更多。斜干仍是逐根算的。

射线网格单独测，不放进上面的求交时间：

| 相机 | 重建 p50 | 缓存命中 p50 |
|---|---|---|
| g2，320×240 | 0.45 ms | < 0.001 ms |
| 林，160×96 | 0.071 ms | < 0.001 ms |

P1 写的网格重建是 1.55 ms，那次包的是 320×240 上的一次测量。这次 320×240 是 0.45 ms。两次数不是同一次计时。相对 35 ms 的求交，网格不是大头，所以缓存两边都开，不放进开关。

## 2. 闭环

seed 0，影子盾开，场景卡上的全长，跑到到达为止。渲染时间是包住 `DepthCamera.capture` 的墙钟，含噪声。回合墙钟是 `run` 的 `wall_time_s`。位置差是开相对关的分量最大绝对差。

| 场景 | 帧数 | 关 渲染 p50 / 均值 / p95 | 开 渲染 p50 / 均值 / p95 | 关墙钟 | 开墙钟 | 位置差 | 到达 |
|---|---|---|---|---|---|---|---|
| g2-pillar | 53 | 34.52 / 34.74 / 36.13 | 6.98 / 8.11 / 13.28 | 15.54 s | 13.11 s | 0 | 都是 4.26 s |
| crossing-forest | 2190 | 34.01 / 34.26 / 35.09 | 12.17 / 12.38 / 13.75 | 96.02 s | 41.51 s | 0 | 都是 23.32 s |
| crossing-6-forest | 4878 | 33.88 / 33.97 / 34.56 | 12.59 / 12.72 / 14.03 | 218.20 s | 108.62 s | 0 | 都是 26.08 s |

三张都是 `SUCCESS`。到达时刻和 P1 那次基线相同。位置差 0 是逐位相同，不是容差里凑出来的。

`g2-pillar` 的渲染大约快 5 倍，但 53 帧省下的大约 1.5 s 落在 15 s 的回合里，墙钟只少 2 s 出头。两张林的渲染是回合墙钟的大头，开开关之后大约少一半。六机林 4878 帧，关 34 ms、开 13 ms，光渲染就差大约 100 s，和墙钟从 218 s 到 109 s 对得上。

## 3. 复现

```bash
.venv/bin/python scripts/p1b-sensor-bench.py
.venv/bin/python scripts/p1b-sensor-bench.py --partial
.venv/bin/python -m pytest -q tests/test_p1b_sensor_accel.py
```

`--partial` 把单帧重复降到 5 次，闭环只飞 0.4 s，用来确认脚本还能跑。上面的表不是 partial 的结果。

默认不设置 `YOPO_SENSOR_ACCEL` 就是关。要再跑一趟开的闭环：

```bash
YOPO_SENSOR_ACCEL=1 .venv/bin/python -m yopo_l0.run --scene scenes/g2-pillar.json --seed 0
```
