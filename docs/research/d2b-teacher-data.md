# D2b：加种后的 EGO-Swarm 教师数据（中途收口）

全量补齐后的清单和统计见 [d2c-teacher-data.md](d2c-teacher-data.md)。本文和 [d2b-teacher-manifest.json](d2b-teacher-manifest.json) 仍以收口时的 663 场为准，数字不改。

在 D2 的教师管线上加种子，密度只取 0、0.14、0.28。几何假设沿用 [assumptions.py](../../scripts/teacher/assumptions.py)，不改那份文件，也没有改仓库外的 [ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 提交 `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20`。源码在 `~/ext/ego-planner-swarm`，不进本仓库，也不加子模块。

大文件在仓库外 `/workspace/yopo-teacher-data/d2b/`。本仓库只交脚本、sha256 清单和这篇文档。没有改 `core/`、冻结门、`scenes/suite.txt`、`docs/tickets.md`、README，也没有改 `yopo_l0/student.py` 和 `scripts/distill/`。

地图边长 20.0 m、障碍半径 0.20 m **是假设，不是论文复现**。见 assumptions.py 文件头。测不到的量写「未知」。

## 1. 这一批跑了什么

计划网格：3 个密度 × 单机和三机 × 2 种图案 × 128 个种子（0 到 127），再加上密度 0 的六机 2 × 128，共 1792 场。命令是：

```bash
scripts/teacher/d2b.sh run --include-six-open --seed-count 128 --jobs 3 --work /workspace/yopo-teacher-data/d2b
```

收口时刻是 2026-11-12 01:26:35 −0300（America/Argentina/San_Juan，UTC-3）。当时后台仿真仍在跑，没有杀掉，已完成的 `record.json` 一共 663 场。清单和本文的数字都以这一批冻结的 663 场为准。还没跑完的格子见第 3 节，留给后续补票接着 `resume`，不要重跑已经有结论的场。

成功条数 635，相对 D2 小批量的 90 场是 7.06 倍，没有达到本票「至少一个数量级」（至少 900 场）的目标。差在还没跑完的格子，不是把已有成功场作废。

## 2. 已跑完的格子

格子是「成功数 / 有结论的场景数」。下面五行是冻结时已经跑满 128 个种子的格子。

| 密度 obs/m² | 机数 | 图案 | 成功 | 结论场 | 障碍碰撞 | 机间碰撞 |
|---|---|---|---|---|---|---|
| 0 | 1 | 平行 | 128 | 128 | 0 | 无此列（单机） |
| 0 | 1 | 对向 | 128 | 128 | 0 | 无此列（单机） |
| 0 | 3 | 平行 | 128 | 128 | 0 | 0 |
| 0 | 3 | 对向 | 128 | 128 | 0 | 0 |
| 0.14 | 1 | 平行 | 105 | 128 | 23 | 无此列（单机） |

密度 0 的四档合计 512/512。密度 0.14 单机平行 105/128。

按密度和机数汇总（六机在冻结时还没有场，记未知）：

| 密度 obs/m² | 1 架 | 3 架 | 6 架 | 该密度合计 |
|---|---|---|---|---|
| 0.0 | 256/256 | 256/256 | 未知 | 512/512 |
| 0.14 | 123/151 | 未知 | 未知 | 123/151 |
| 0.28 | 未知 | 未知 | 未知 | 未知 |

0.14 单机那一格是 105 场已跑完的平行，加上冻结时已经写出记录的 23 场对向（18 成功、5 失败），对向这一格并没有跑满，见下一节。

## 3. 还没跑完的格子（后补）

计划每格 128 个种子。冻结时不足 128 的格子如下。0/128 表示还一行记录都没有。

| 密度 obs/m² | 机数 | 图案 | 已写出记录 | 其中成功 | 其中失败 | 还缺 |
|---|---|---|---|---|---|---|
| 0.14 | 1 | 对向 | 23 | 18 | 5 | 105 |
| 0.14 | 3 | 平行 | 0 | 0 | 0 | 128 |
| 0.14 | 3 | 对向 | 0 | 0 | 0 | 128 |
| 0.28 | 1 | 平行 | 0 | 0 | 0 | 128 |
| 0.28 | 1 | 对向 | 0 | 0 | 0 | 128 |
| 0.28 | 3 | 平行 | 0 | 0 | 0 | 128 |
| 0.28 | 3 | 对向 | 0 | 0 | 0 | 128 |
| 0 | 6 | 平行 | 0 | 0 | 0 | 128 |
| 0 | 6 | 对向 | 0 | 0 | 0 | 128 |

还缺合计 1129 场。后台那次 `run` 在收口后继续跑，数据仍写到 `/workspace/yopo-teacher-data/d2b/runs/`。补票时用同一条命令（默认 resume），跳过已有结论的 `record.json`。

## 4. 碰撞和到达时间

28 场失败全部在密度 0.14 的单机：平行 23、对向 5。这 28 场都已经进入到达门，但表面距离按假设的 0.20 m 机体半径记成障碍碰撞。机间碰撞是 0。未知是 0。

到达时间是本管线的定义，论文没有到达半径：第一段每架都在目标 0.5 m 内、速度也不超过 0.3 m/s，并且随后保持满 1.0 s（允许少一拍）的起始时刻，相对第一拍。未到达则未知。

成功场到达时间：

| 范围 | 场数 | 均值 | 中位 | 最小 | 最大 |
|---|---|---|---|---|---|
| 全部成功场 | 635 | 16.665 s | 15.900 s | 15.097 s | 20.275 s |
| 1 架成功场 | 379 | 15.722 s | 15.700 s | 15.097 s | 17.800 s |
| 3 架成功场 | 256 | 18.060 s | 17.750 s | 15.300 s | 20.275 s |
| 密度 0 成功场 | 512 | 16.836 s | 15.801 s | 15.097 s | 20.275 s |
| 密度 0.14 成功场 | 123 | 15.950 s | 16.100 s | 15.100 s | 17.800 s |

密度 0、三机、对向的成功场中位到达时间是 19.800 s，比同密度平行三机的 16.599 s 长，这和空场对向要交换车道一致。

663 场的单场墙钟合计 12634.451 s。这次收口没有整批墙钟（仿真还在跑），场景/小时记未知，不拿单场墙钟之和去除以并行度去外推。

逐场数字在仓库外 `/workspace/yopo-teacher-data/d2b/snapshots/20261112T012635/d2b-teacher-batch.json`。npz 不进 git。清单在 [d2b-teacher-manifest.json](d2b-teacher-manifest.json)，含上述 663 场的 `trajectory.npz`、`record.json`、`scene.json`、`geometry.json` 的 sha256，共 2655 个文件条目。

## 5. 管线

新脚本在 `scripts/teacher/d2b.sh`，沿用 `scripts/teacher/` 里 D2 的场景生成、launch、记录和判据。最多 3 路并行，各自一个容器，不用 host 网络。进程 `nice -n 10`。

镜像仍是 D2 用过的 `osrf/ros:noetic-desktop-focal`，digest `sha256:85ff4e2f691f7eb5673835afe7e70cd49e552e3cdb8b1938b7c54b647d93b99a`。本机拉镜像和装 ROS 依赖时，访问 `snapshots.ros.org` 的 HTTP 会被网关打回 500，HTTPS 证书主机名也对不上。只在教师容器里把该源改成 HTTPS，并关掉 apt 的证书校验。没有改 ego-planner-swarm 源码。编译之后 `git status` 只有被 gitignore 的 `build/`、`devel/`。

第一次并行跑到密度 0 三机对向后段时，容器 `--pids-limit 512` 被打满。PID 1 当时是 `sleep infinity`，不收割僵尸线程，后续出现 `boost::thread_resource_error` / `can't start new thread`，部分场 `no_odom` 或超时。编排器被停掉后，给工人容器加了 Docker `--init`（tini 收割僵尸），把 `--pids-limit` 提到 8192，再按 resume 继续。已有成功结论的场没有重跑。按日志判定为基础设施失败、或 `success` 为未知的 22 场删掉记录后重跑：其中 21 场是密度 0 三机对向，重跑后全部到达且成功；另外 1 场是 `d014-n1-parallel-s48`（`no_odom`），重跑后进入该格的正常统计。密度 0.14 单机平行里因障碍碰撞失败的场保留原结论，没有重跑。

## 6. 机器

| 项 | 值 |
|---|---|
| 机器型号 | 未知。`/sys/class/dmi/id` 的 product_name 为空或没有 |
| 虚拟化 | KVM，full（`lscpu` 的 Hypervisor vendor、Virtualization type） |
| CPU | Intel(R) Xeon(R) Processor，1 路 8 核，每核 1 线程，`cpu MHz` 2400.000，BogoMIPS 4800.00 |
| 内存 | MemTotal 16386416 kB |
| 宿主机 | Debian GNU/Linux 13 (trixie)，内核 `6.12.113+`，`x86_64` |
| Docker | 29.1.3。没有 systemd，守护进程是手工拉起的静态二进制 |
| 容器 | `osrf/ros:noetic-desktop-focal`，digest `sha256:85ff4e2f691f7eb5673835afe7e70cd49e552e3cdb8b1938b7c54b647d93b99a` |
| 教师提交 | `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20` |

论文的基准机是 i7-9700KF。和这台 CPU 的对应关系未知。

## 7. 再跑和续跑

```bash
scripts/teacher/d2b.sh --help
scripts/teacher/d2b.sh all --dry-run --include-six-open --seed-count 128
scripts/teacher/d2b.sh run --include-six-open --seed-count 128 --jobs 3 --work /workspace/yopo-teacher-data/d2b
scripts/teacher/d2b.sh summarize --work /workspace/yopo-teacher-data/d2b
scripts/teacher/d2b.sh manifest --work /workspace/yopo-teacher-data/d2b
```

`run` 默认 resume：已有结论（成功或失败）的 `record.json` 会跳过；`success` 为未知的会重跑。本脚本不启动 dockerd。
