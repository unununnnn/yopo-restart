# D2c：登记 D2b 补齐后的全量教师数据

D2b 在 2026-11-12 01:26:35 −0300（America/Argentina/San_Juan，UTC-3）中途收口，清单冻结在 663 场，后台 `d2b.sh run` 当时没有杀掉。这次后台已经跑完。本票用 main 上 D2b 的管线和清单格式，把补齐后的全量数据登记入库：重新生成 sha256 清单，按格统计，查清 `run-wall.json` 里 543 个 skip 的原因。没有重跑仿真，没有改 `assumptions.py`，也没有改仓库外的 [ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 提交 `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20`。

大文件仍在仓库外 `/workspace/yopo-teacher-data/d2b/`。本仓库只交这篇说明、全量 sha256 清单 [d2c-teacher-manifest.json](d2c-teacher-manifest.json)，以及对应测试。D2b 收口时的说明和清单不改，见 [d2b-teacher-data.md](d2b-teacher-data.md) 与 [d2b-teacher-manifest.json](d2b-teacher-manifest.json)。没有改 `core/`、冻结门、`scenes/suite.txt`、`docs/tickets.md`、README，也没有改 `scripts/distill/` 和 `yopo_l0/student.py`。

地图边长 20.0 m、障碍半径 0.20 m **是假设，不是论文复现**。见 assumptions.py 文件头。测不到的量写「未知」。密度 0.14、0.28 的六机不在本计划里，下面记「未计划」，不要和「还没跑」混淆。

## 1. 这一批跑了什么

计划网格与 D2b 相同：3 个密度 × 单机和三机 × 2 种图案 × 128 个种子（0 到 127），再加上密度 0 的六机 2 × 128，共 1792 场。命令是：

```bash
scripts/teacher/d2b.sh run --include-six-open --seed-count 128 --jobs 3 --work /workspace/yopo-teacher-data/d2b
```

续跑在 2026-11-12 01:11:17 −0300 启动（`logs/d2b-run.pid` 的 mtime），默认 resume。`run-wall.json` 在 04:48:19 −0300 写出，记录 `n=1792`、`ok=1249`、`skip=543`、`fail=0`、`jobs=3`、`batch_wall_s=13019.918`。仓库外 `runs/` 现有 1792 份 `record.json`，与计划场数一致。

官方 `d2b.sh summarize` 的合计：成功 1205，失败 553，未知 34。成功 1205 场相对 D2 小批量的 90 场，达到 D2b「至少一个数量级」（至少 900 场）的目标。逐场数字在仓库外 `/workspace/yopo-teacher-data/d2b/d2c-teacher-batch.json`。npz 不进 git。

## 2. 按格统计

格子是「成功数 / 有结论的场景数」。有结论 = 成功或失败，不含未知。跑出数是该格已写出的 `record.json` 份数。失败分类按记录里的 `metrics` 和 `run.status` 计：障碍碰撞指 `collision_obstacle` 为真且已到达；「障碍碰撞且超时」指同一场既记了障碍碰撞、状态又是 `timeout`；「超时（未记碰撞）」指状态是 `timeout` 但没有记碰撞。机间碰撞全程是 0，表里不再单列。

| 密度 obs/m² | 机数 | 图案 | 计划 | 跑出 | 成功 | 失败 | 未知 | 失败分类 |
|---|---|---|---|---|---|---|---|---|
| 0 | 1 | 平行 | 128 | 128 | 128 | 0 | 0 | 无 |
| 0 | 1 | 对向 | 128 | 128 | 128 | 0 | 0 | 无 |
| 0 | 3 | 平行 | 128 | 128 | 128 | 0 | 0 | 无 |
| 0 | 3 | 对向 | 128 | 128 | 128 | 0 | 0 | 无 |
| 0 | 6 | 平行 | 128 | 128 | 110 | 7 | 11 | 超时（未记碰撞）7；launch_failed 11 |
| 0 | 6 | 对向 | 128 | 128 | 103 | 9 | 16 | 超时（未记碰撞）9；launch_failed 14；no_odom 2 |
| 0.14 | 1 | 平行 | 128 | 128 | 105 | 23 | 0 | 障碍碰撞 23（均已到达） |
| 0.14 | 1 | 对向 | 128 | 128 | 95 | 33 | 0 | 障碍碰撞 33（均已到达） |
| 0.14 | 3 | 平行 | 128 | 128 | 33 | 95 | 0 | 障碍碰撞 95（均已到达） |
| 0.14 | 3 | 对向 | 128 | 128 | 58 | 70 | 0 | 障碍碰撞 70（均已到达） |
| 0.14 | 6 | 平行 / 对向 | 未计划 | 0 | — | — | — | — |
| 0.28 | 1 | 平行 | 128 | 128 | 90 | 38 | 0 | 障碍碰撞 37（均已到达）；障碍碰撞且超时 1 |
| 0.28 | 1 | 对向 | 128 | 128 | 81 | 47 | 0 | 障碍碰撞 47（均已到达） |
| 0.28 | 3 | 平行 | 128 | 128 | 4 | 124 | 0 | 障碍碰撞 119（均已到达）；障碍碰撞且超时 3；超时（未记碰撞）2 |
| 0.28 | 3 | 对向 | 128 | 128 | 14 | 107 | 7 | 障碍碰撞 98（均已到达）；障碍碰撞且超时 8；超时（未记碰撞）1；launch_failed 6；no_odom 1 |
| 0.28 | 6 | 平行 / 对向 | 未计划 | 0 | — | — | — | — |

合计：计划 1792，跑出 1792，成功 1205，失败 553，未知 34。失败 553 场里：障碍碰撞（已到达）522，障碍碰撞且超时 12，超时（未记碰撞）19。障碍碰撞合计 534，与官方汇总表一致。机间碰撞 0。

按密度和机数汇总（官方 `format_d2b_table` 对空格打印「未知」；本表把未计划的六机格改写成「未计划」）：

| 密度 obs/m² | 1 架 | 3 架 | 6 架 | 该密度合计 |
|---|---|---|---|---|
| 0.0 | 256/256 | 256/256 | 213/229 | 725/741 |
| 0.14 | 200/256 | 91/256 | 未计划 | 291/512 |
| 0.28 | 171/256 | 18/249 | 未计划 | 189/505 |

密度 0 的六机格分母是 229 而不是 128×2=256，因为有 27 场未知。密度 0.28 的三机格分母是 249，因为有 7 场未知。

## 3. 543 个 skip 的原因

`run-wall.json` 的 `skip=543` **全部是续跑时按设计跳过已有结论的场，不是出错，也不是网格里预先剔除的场。**

依据如下。

1. `scripts/teacher/d2b.py` 里 `_should_skip`：默认 `--resume`，且默认重试 `success` 为未知的场。已有 `record.json` 并且 `success` 不是 `None` 的场直接返回 `"skip"`，不进容器。
2. 续跑在 2026-11-12 01:11:17 −0300 启动。当时磁盘上已有 543 份 `success` 非空的 `record.json`，这些文件的 mtime 全部早于 01:11:19；这次续跑新写出的 1249 份 `record.json`，mtime 全部晚于 01:11:19。进度表第一行（01:11:19 −0300）也是 `n=543 success=533 fail=10 unknown=0`。
3. 这 543 场的分布：密度 0 单机平行 128、单机对向 128、三机平行 128、三机对向 107，全部成功；密度 0.14 单机平行 52（成功 42，障碍碰撞 10）。没有六机，也没有密度 0.28。
4. 续跑实际执行并写出记录的是另外 1249 场，`ok=1249`。`543 + 1249 = 1792`。
5. `fail=0` 只表示编排器没有未捕获异常。任务失败（碰撞、超时）和 34 场未知都记在这 1249 场的 `record.json` 里，返回值仍是 `"ok"`。日志里有 15 行 `fail … docker_exec=-9`，这些场也被写成了 `launch_failed` 记录，计入 `ok`，不计入 `run-wall` 的 `fail`。

日志里部分 `skip` 行因 3 路并发打印粘在同一行。按场景 id 用正则解析，`skip` 仍是 543 个互不重复的 id，与上述 543 份续跑前已有结论的记录一一对应。

D2b 收口时刻（01:26:35 −0300）的 663 场，等于这 543 场再加上续跑开始后已经写完的 120 场。那 120 场记在本次 `ok=1249` 里，不在 `skip` 里。

## 4. 34 场未知

未知全部是这次续跑新写出的记录，`success` 为 `None`。默认 resume 会重试未知场，但这些场是续跑过程中新出现的，当时没有再排一次。本票只登记，不重跑。

| 密度 obs/m² | 机数 | 图案 | launch_failed | no_odom | 合计 |
|---|---|---|---|---|---|
| 0 | 6 | 平行 | 11 | 0 | 11 |
| 0 | 6 | 对向 | 14 | 2 | 16 |
| 0.28 | 3 | 对向 | 6 | 1 | 7 |

`launch_failed` 共 31 场：目录里没有 `raw_record.json`，也没有 `done.json`，管线按缺原始记录写成这个状态。其中 15 场在 `d2b-run.log` 里有 `docker_exec=-9`（`d000-n6-headon` 的 s40、s41、s42、s63、s65、s124、s126；`d000-n6-parallel` 的 s39、s40、s74、s75、s76；`d028-n3-headon` 的 s55、s56、s57）。其余 16 场同样没有 `raw_record.json`，日志没有打印 `docker_exec` 行，具体退出码未知。这 31 场的 `roslaunch.log` 并不都是空的，有的能看到规划器曾进入 `EXEC_TRAJ`，但记录进程没有写出原始样本。本票不把它们改记成超时或碰撞。

`no_odom` 共 3 场：`d000-n6-headon-s64`、`d000-n6-headon-s67`、`d028-n3-headon-s6`。这 3 场有 `done.json`（`{"status": "no_odom"}`）和 `raw_record.json`。

完整 id 见第 8 节。若以后要补跑，同一条 resume 命令会重试 `success` 为未知的场，不会重跑已有成功或失败结论的场。

## 5. 碰撞和到达时间

到达时间沿用 D2b / D2 管线的定义，论文没有到达半径：第一段每架都在目标 0.5 m 内、速度也不超过 0.3 m/s，并且随后保持满 1.0 s（允许少一拍）的起始时刻，相对第一拍。未到达则未知。

官方 `d2b.sh summarize` 打印的表（六机空格原文是「未知」，下面按第 2 节改成「未计划」；其余数字与命令输出一致）：

| 密度 obs/m² | 1 架 | 3 架 | 6 架 | 该密度合计 |
|---|---|---|---|---|
| 0.0 | 256/256 | 256/256 | 213/229 | 725/741 |
| 0.14 | 200/256 | 91/256 | 未计划 | 291/512 |
| 0.28 | 171/256 | 18/249 | 未计划 | 189/505 |

| 机数 | 成功 | 障碍碰撞 | 机间碰撞 | 到达时间（成功场） |
|---|---|---|---|---|
| 1 架 | 627/768 | 141 | 0 | 均值 15.891 s，中位 15.800 s，范围 14.900–18.600 s，n=627 |
| 3 架 | 365/761 | 393 | 0 | 均值 18.274 s，中位 19.300 s，范围 15.300–27.172 s，n=365 |
| 6 架 | 213/229 | 0 | 0 | 均值 18.903 s，中位 17.704 s，范围 16.498–24.500 s，n=213 |

| 图案 | 成功率 |
|---|---|
| 平行 | 598/885 |
| 对向 | 607/873 |

场景/小时: 495.49（口径 `batch_wall_s`）。这个墙钟是续跑这一段的 `13019.918` s，分子按官方脚本用全部 1792 场（含 543 个瞬间跳过的场），不是从 2026-11-11 23:54 第一场开始的整段墙钟。整段没有单独计时，不拿单场墙钟之和去除以并行度去外推。1792 场的单场墙钟合计 `38802.379` s。

合计成功: 1205/1758，未知 34。D2 成功 90 场，D2b 目标至少 900 场。是否达到一个数量级: 是。
障碍碰撞 534，机间碰撞 0。
成功场到达时间: 均值 17.145 s，中位 16.500 s，范围 14.900–27.172 s，n=1205。

## 6. 管线

沿用 D2b 的 `scripts/teacher/d2b.sh`，没有改脚本。最多 3 路并行，各自一个容器，不用 host 网络。进程 `nice -n 10`。镜像仍是 `osrf/ros:noetic-desktop-focal`，digest `sha256:85ff4e2f691f7eb5673835afe7e70cd49e552e3cdb8b1938b7c54b647d93b99a`。教师提交仍是 `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20`。本票只做 `summarize` 和 `manifest`，没有再启动 dockerd，也没有重编译。

清单由 `scripts/teacher/d2b.sh manifest --work /workspace/yopo-teacher-data/d2b --out docs/research/d2c-teacher-manifest.json` 生成，格式与 D2b 相同（`schema` 仍是 `yopo-teacher-d2b-manifest/1`），再补上完成时刻、成功/失败/未知条数和续跑字段。文件条目的 sha256 按管线原样保留。全量清单含 1792 场的 `trajectory.npz`、`record.json`、`scene.json`、`geometry.json`，以及仓库外的 `scenarios.jsonl`、`build-stamp.json`、`run-wall.json`，共 7171 个文件条目。

## 7. 机器

下面数字来自这次 `summarize` 当场采集的 `machine_info` 和 `build-stamp.json`。机器型号：未知（`product_name` 为 null）。

| 项 | 值 |
|---|---|
| 机器型号 | 未知 |
| 虚拟化 | KVM，full（`lscpu` 的 Hypervisor vendor、Virtualization type） |
| CPU | Intel(R) Xeon(R) Processor，1 路 8 核，每核 1 线程，BogoMIPS 4800.00 |
| 内存 | MemTotal 16386416 kB |
| 内核 | `Linux-6.12.113+-x86_64-with-glibc2.41`（`uname`） |
| 教师提交 | `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20` |
| 容器镜像 | `osrf/ros:noetic-desktop-focal`，digest `sha256:85ff4e2f691f7eb5673835afe7e70cd49e552e3cdb8b1938b7c54b647d93b99a` |

论文的基准机是 i7-9700KF。和这台 CPU 的对应关系未知。

## 8. 未知场的 id

密度 0、六机、平行，`launch_failed` 11 场：`d000-n6-parallel-s39`、`s40`、`s42`、`s43`、`s52`、`s74`、`s75`、`s76`、`s77`、`s78`、`s83`。

密度 0、六机、对向，`launch_failed` 14 场：`d000-n6-headon-s40`、`s41`、`s42`、`s43`、`s48`、`s57`、`s63`、`s65`、`s66`、`s68`、`s69`、`s124`、`s126`、`s127`。`no_odom` 2 场：`d000-n6-headon-s64`、`s67`。

密度 0.28、三机、对向，`launch_failed` 6 场：`d028-n3-headon-s55`、`s56`、`s57`、`s58`、`s59`、`s64`。`no_odom` 1 场：`d028-n3-headon-s6`。

## 9. 再跑和续跑

```bash
scripts/teacher/d2b.sh --help
scripts/teacher/d2b.sh summarize --work /workspace/yopo-teacher-data/d2b --out /workspace/yopo-teacher-data/d2b/d2c-teacher-batch.json
scripts/teacher/d2b.sh manifest --work /workspace/yopo-teacher-data/d2b --out docs/research/d2c-teacher-manifest.json
scripts/teacher/d2b.sh run --include-six-open --seed-count 128 --jobs 3 --work /workspace/yopo-teacher-data/d2b
```

`run` 默认 resume：已有结论（成功或失败）的 `record.json` 会跳过；`success` 为未知的会重跑。本脚本不启动 dockerd。本票没有执行最后一条。
