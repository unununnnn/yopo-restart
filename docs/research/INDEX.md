# 研究目录索引

<!-- 由 scripts/research_index.py 生成；请勿手工编辑。 -->

| 文件名 | 票号 | 标题首行 | report-only / 冻结标记 |
|---|---|---|---|
| d1-geometric-fallback.md | d1 | 票 D1：提交前的一次性几何回退 | report-only、冻结 |
| d2-teacher-batch.json | d2 | （无说明字段） | 无 |
| d2-teacher-data.md | d2 | D2：EGO-Swarm 教师数据 | 冻结 |
| d2b-teacher-data.md | d2b | D2b：加种后的 EGO-Swarm 教师数据（中途收口） | 冻结 |
| d2b-teacher-manifest.json | d2b | （无说明字段） | 无 |
| d2c-teacher-data.md | d2c | D2c：登记 D2b 补齐后的全量教师数据 | 冻结 |
| d2c-teacher-manifest.json | d2c | （无说明字段） | 无 |
| d3-multihead.md | d3 | D3：单机多头连续输出学生（脚手架） | 冻结 |
| d4-neighbour-input.md | d4 | D4：学生干路加邻机输入（脚手架） | 冻结 |
| d5-student-loop-runs.json | d5 | 只报数，不冻结新判据。密度场的地图边长和障碍半径是假设，不是论文复现。 | report-only、冻结 |
| d5-student-loop.md | d5 | D5：学生网络接进 L0 飞行闭环 | report-only、冻结 |
| d5-student-train.json | d5 | （无说明字段） | 无 |
| d5b-student-fix-runs.json | d5b | （无说明字段） | 无 |
| d5b-student-fix.md | d5b | D5b：学生闭环接口修正、DAgger 与改训练（report-only） | report-only、冻结 |
| d5c-clearance-fit.md | d5c | D5c：学生训练侧 L0 机体半径净空过滤与安全加权（report-only） | report-only、冻结 |
| d6-teacher-altitude-runs.json | d6 | （无说明字段） | 无 |
| d6-teacher-altitude.md | d6 | D6：教师飞行高度扩到 {1.0, 1.5} 的规划，以及 z 平移可行性（report-only） | report-only、冻结 |
| d6b-pipeline.md | d6b | D6b：1.0 m 与 1.5 m 教师混合转换、训练、双高度评测 | report-only、冻结 |
| e-a1-inter-drone-filter.md | e-a1 | 票 E-A1：机间过滤开关，默认关 | report-only、冻结 |
| e-a2-inter-soft-yield-runs.json | e-a2 | 只报数。report-only。不改默认 inter_drone_soft_yield false、empty_brake false、track_clearance false。四档都打开 E-A1（阈值 1.0 m、惩罚 0）和 empty_brake；geom 两档再打开 track_clearance mode=geom、margin 0.08 m。全部偏离上游。soft_yield 默认关。hard_min_m 0.75 m 高于 0.70 m 机间碰撞球。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。未冻结，不进 scenes/suite.txt，不设 gates。 | report-only |
| e-a2-inter-soft-yield.md | e-a2 | 票 E-A2：机间过滤防死锁软让行，默认关 | report-only、冻结 |
| e-b1-backend-seam.md | e-b1 | 后段接缝（E-B1） | 冻结 |
| e-b2-bspline-prototype.md | e-b2 | 单机 B 样条原型（E-B2） | 冻结 |
| e-b2-bspline-runs.json | e-b2 | （无说明字段） | 无 |
| e-b2p-backend-speed.md | e-b2p | E-B2p：B 样条后段提速 | report-only、冻结 |
| e-b3-closeout.md | e-b3 | E-B3 收口 | 冻结 |
| e-b3-swarm-penalty-runs.json | e-b3 | 这是假设，不是论文复现 | 无 |
| e-b3-swarm-penalty.md | e-b3 | E-B3：后段机间椭球惩罚 | report-only、冻结 |
| e-b3c-segment-penalty-runs.json | e-b3c | 这是假设，不是论文复现 | 无 |
| e-b3c-segment-penalty.md | e-b3c | E-B3c：机间椭球改到段内采样 | 冻结 |
| e-b4-peer-collision-diag-runs.json | e-b4 | 修复臂是进程内原型（本脚本的 _proto_brake + E-A1 阈值 1.0 m），不是已合入的开关；只报数 | report-only |
| e-b4-peer-collision-diag.md | e-b4 | 票 E-B4：机间碰撞为什么在所有臂上都「只差一点」（诊断，只报数） | report-only、冻结 |
| e-b4b-brake-switch-runs.json | e-b4b | 开臂 = E-A1 1.0 m + empty_brake（偏离上游） | 无 |
| e-b4b-brake-switch.md | e-b4b | 票 E-B4b：无解时从参考头刹停（开关，默认关） | report-only、冻结 |
| e-b4c-sphere-avoid-runs.json | e-b4c | 只报数。peer_sphere_enabled 默认关，不改 swarm_penalty / swarm_segment_penalty 的语义。打开欧氏球或 track_clearance 即偏离上游 d5e1e60。未冻结，不进 scenes/suite.txt，不设 gates。半径 0.85 m = 0.70 m 碰撞球 + 0.15 m 裕度。这不是论文复现。 | report-only |
| e-b4c-sphere-avoid.md | e-b4c | E-B4c：后段 0.70 m 欧氏碰撞球 | report-only、冻结 |
| e-b4d-sphere-miss-runs.json | e-b4d | report-only。只报数。不改 yopo_l0 后段、不冻结、不进 scenes/suite.txt，不设 gates。本票不拷贝 PR #94 的欧氏球实现。main 尚未带 peer_sphere_enabled 时跳过实飞，标 missing_feature。开关合入后再用同一脚本逐 tick 回答 014-swap3 seed 1（对照 seed 0）。 | report-only、冻结 |
| e-b4d-sphere-miss.md | e-b4d | 票 E-B4d：014-swap3 seed 1 欧氏球未推开 | report-only、冻结 |
| e-b5-sphere-sweep-runs.json | e-b5 | 只报数。report-only。peer_sphere_enabled 默认关，不改 swarm_penalty / swarm_segment_penalty 的语义。打开欧氏球、加大半径或权重、track_clearance、empty_brake，或使用脚本内末端放行，即偏离上游 d5e1e60。off 不打开这些开关。未冻结，不进 scenes/suite.txt，不设 gates。默认半径 0.85 m = 0.70 m 碰撞球 + 0.15 m 裕度，权重 8，与已合入的 E-B4c 相同。1.00 m 是 0.70 m 再加 0.30 m 裕度。权重 16 是默认的两倍。track_margin 0.08 m 沿用 E-C5 的假设。末端放行只存在于本脚本进程，不改 yopo_l0。六机开阔和 014 六机对向的卡只在内存里，不写入 scenes/。这不是论文复现。 | report-only |
| e-b5-sphere-sweep.md | e-b5 | E-B5：欧氏碰撞球参数扫描（只报数） | report-only |
| e-b7-fault-and-swap8-sphere-runs.json | e-b7 | 只报数。report-only。peer_sphere_enabled 默认关，不改 swarm_penalty / swarm_segment_penalty 的语义。打开欧氏球即偏离上游 d5e1e60。未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。E-B4c 半径 0.85 m = 0.70 m 碰撞球 + 0.15 m 裕度，权重 8。E-B5 的 1.00 m / 权重 8，以及 0.85 m / 权重 16，同样只在本进程的卡上。这不是论文复现。 | report-only、冻结 |
| e-b7-fault-and-swap8-sphere.md | e-b7 | E-B7：八机对向与 M4 六档通讯故障下的欧氏碰撞球 | report-only、冻结 |
| e-c4-density-l0-runs.json | e-c4 | 只报数，不改默认，不冻结。墙钟不参与轨迹。backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。 | report-only、冻结 |
| e-c4-density-l0.md | e-c4 | 票 E-C4 密度场 L0 seed 0，只报数 | report-only、冻结 |
| e-c4-eight-swap-density.md | e-c4 | 票 E-C4：八机开阔对向，以及一张标明假设的密度场 | report-only、冻结 |
| e-c4b-density-seeds-runs.json | e-c4b | 只报数，不改默认，不冻结。墙钟不参与轨迹。backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。radius035 是 safe_radius 0.35 m，偏离上游 d5e1e60。combo（E-A1+D1+E-B2）60 次已写入 rows。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。 | report-only、冻结 |
| e-c4b-density-seeds.md | e-c4b | 票 E-C4b：密度场 L0 补跑 seeds 0–9 与失败归因，只报数 | report-only、冻结 |
| e-c5-corridor-radius-runs.json | e-c5 | 只报数。不改默认 safe_radius 0.05 m。0.20 m 与 0.35 m 偏离上游 d5e1e60。刹停是 scripts/e-b4-peer-diag.py 的进程内原型，不是已合入开关。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。 | report-only |
| e-c5-corridor-radius.md | e-c5 | 票 E-C5：走廊准入 safe_radius 对照，只报数 | report-only、冻结 |
| e-c5b-track-clearance-runs.json | e-c5b | 只报数。不改默认 safe_radius 0.05 m、empty_brake false、track_clearance false。E-A1、empty_brake、track_clearance 打开即偏离上游 d5e1e60。track_margin 0.08 m 是假设，用来盖已观测的 0.058–0.073 m 跟踪差。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。未冻结，不进 scenes/suite.txt，不设 gates。 | report-only |
| e-c5b-track-clearance.md | e-c5b | 票 E-C5b：走廊跟踪误差补偿，只报数 | report-only、冻结 |
| e-c5c-blindspot-dense-runs.json | e-c5c | 只报数（report-only）。不改默认 safe_radius 0.05 m、empty_brake false、track_clearance false。E-A1、empty_brake、track_clearance 打开即偏离上游。track_margin 0.08 m 是假设，用来盖 E-C5 已观测的 0.058–0.073 m 跟踪差。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。未冻结，不进 scenes/suite.txt，不设 gates。密度 0.28 的五档对照只报数，不据此改规划器。 | report-only |
| e-c5c-blindspot-dense.md | e-c5c | 票 E-C5c：地面盲区归因与密度 0.28 对照，只报数 | report-only |
| e-c5d-ground-guard-runs.json | e-c5d | 只报数（report-only）。不改默认 safe_radius 0.05 m、empty_brake false、track_clearance false、ground_guard false。四档都打开了 track_clearance，偏离上游。ground_guard 打开也偏离上游。未冻结，不进 scenes/suite.txt，不设 gates。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。 | report-only |
| e-c5d-ground-guard.md | e-c5d | 票 E-C5d：地面净空闸门与刹停高度保护 | report-only |
| e-c6-six-density-runs.json | e-c6 | 只报数。report-only。不改默认 safe_radius 0.05 m、empty_brake false、track_clearance false、inter_drone_filter 关。E-A1 阈值 1.0 m、empty_brake、track_clearance 打开即偏离上游 d5e1e60。track_margin 0.08 m 是假设，用来盖已观测的 0.058–0.073 m 跟踪差。六机 0.14 对向卡只存在于进程内存，不写入 scenes/。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。 | report-only、冻结 |
| e-c6-six-density.md | e-c6 | 票 E-C6：六机密度场 0.14，只报数 | report-only、冻结 |
| e-c7-empty-hover-runs.json | e-c7 | 只报数。report-only。不改默认 safe_radius 0.05 m、empty_brake false、track_clearance false、inter_drone_filter 关。三档都在进程里打开 E-A1（阈值 1.0 m、惩罚 0）、empty_brake，以及 track_clearance margin 0.08 m，全部偏离上游。geom_soft 的软让行不是 main 上的开关：本脚本只在本进程里包住 inter_pick，硬过滤判空后留下中心距 ≥ 0.75 m 的候选，让行权重 2.0，用来和硬过滤对照。不改 yopo_l0/inter_drone.py，不改 yopo_l0/policy.py，不改 scenes/。0.08 m 是假设，用来盖已观测的跟踪差。0.40 m 的贴地/贴树线是机体半径 0.35 m 再加 0.05 m。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。 | report-only、冻结 |
| e-c7-empty-hover.md | e-c7 | 票 E-C7：六机 014 geom 空走廊悬停与超时归因 | report-only、冻结 |
| e-c8-escape-replan-runs.json | e-c8 | 只报数。report-only。偏离上游。不改 yopo_l0/、core/、scenes/、docs/tickets.md、README、scripts/premerge.sh、scripts/regress.sh。不改 safe_radius 0.05 m、E-A1 阈值 1.0 m、track_margin 0.08 m、empty_brake 的过滤门槛。脱困只改这一拍送进规划器的临时目标，走廊重开后回到原目标。E-B4c 的 peer_sphere_enabled 默认关；打开即偏离上游。六机 0.14 对向卡只存在于进程内存，不写入 scenes/。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。 | report-only、冻结 |
| e-c8-escape-replan.md | e-c8 | 票 E-C8：六机 014 连续空走廊悬停脱困重规划 | report-only、冻结 |
| e-combo-a1-d1-b2-runs.json |  | 三机两张卡的规划器 commit 是 616909b；合并 origin/main 之后全关 seed 0 复跑轨迹相同。八机只收 seed 0，跑在合并之后。影子盾开。全关、E-A1、D1 两个进程；E-B2、三个一起、八机单独一个进程。backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。密林上这个计数等于调用次数，seed 0 再跑轨迹会变。 | 无 |
| e-combo-a1-d1-b2.md |  | E-A1、D1、E-B2 三个开关一起量 | report-only、冻结 |
| e-combo-six-forest-runs.json |  | 墙钟不参与轨迹，也不进已冻门。backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。整表 report-only。 | report-only、冻结 |
| e-combo-six-forest.md |  | 六机林带上把 E-A1、D1、E-B2 三个开关一起量 | report-only、冻结 |
| e-combo-swap8-seeds-runs.json |  | 墙钟不参与轨迹。combo 的 backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。 | 无 |
| e-combo-swap8-seeds.md |  | 八机开阔对向补跑种子 1–9 | report-only、冻结 |
| e-d435-depth-noise-runs.json | e-d435 | 只报数。report-only。``d435_depth`` 默认关，场景卡、``scenes/suite.txt``、已冻门都没改。打开即偏离上游：上游权重在真值深度上训练（右目关闭）。无效像素保持 NaN，网络侧用上游 NS inpaint；本表没有用 inpaint 预填，也没有填最大量程。飞点概率 0.3、掠射角阈值 75°、视差噪声相关长度 3 px、丢帧率 0.5%，以及约 21 ms 里 Orin 额外的约 5 ms，置信度低。仿真相机不是 848×480。噪声用该相机的 fx，米制 σ 比原生 D435 大。编队任务不读深度，轨迹不该因这只开关改变；影子盾仍会看着深度图。影子盾只记录 would_reject，不改批准的计划。墙钟不进结论。 | report-only、冻结 |
| e-d435-depth-noise.md | e-d435 | 票 E-D435：D435 深度噪声与延迟下的 YOPO L0 | report-only、冻结 |
| e-so3-dynamics-gap-runs.json |  | 只报数。report-only。SO3 两臂是偏离默认动力学（L0 点质量不变）。不改 yopo_l0/ 的默认动力学，不改 core/、scenes/、docs/tickets.md、README.md、scripts/premerge.sh、scripts/regress.sh。规划与策略保持场景卡默认：upstream（柱群和穿林由 CrossingFleet 走上游 YOPO），影子盾开。外环 kp/kd/a_max 与 PointMassLag 相同。τ_att、τ_thrust 和倾角是票面给定的假设，不是本仓库的 SITL 辨识。未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。 | report-only、冻结 |
| e-so3-dynamics-gap.md |  | 票 E-SO3：L0 SO3 四旋翼滞后 vs 点质量，只报数 | report-only、冻结 |
| ego-c2-swap3.md |  | 票 E-C2：三机开阔对向（只报数） | report-only、冻结 |
| ego-swarm-baseline.md |  | EGO-Swarm 外部基线（E-C1） | 无 |
| ego-swarm-comparison.md |  | yopo-restart 与 EGO-Swarm 对照 | report-only、冻结 |
| ego-swarm-density.json |  | （无说明字段） | 无 |
| m5-a1-closeout-audit.md | m5 | M5-A1：口径 A 第一期收口前置核验 | report-only |
| p1-inference-speed.md | p1 | P1：YOPO 单次推理耗时 | 冻结 |
| p1b-sensor-speed.md | p1b | P1b：深度相机求交加速（默认关） | report-only、冻结 |
| s4-sitl-bridge.md | s4 | 票 S4：YOPO L0 与 PX4 SITL 的 ENU/NED 闭环桥 | report-only |
| s4b-sitl-live.md | s4b | 票 S4b：YOPO L0 规划桥接进 PX4 / Gazebo 容器 | report-only |
| s4c-sitl-yopo-image.md | s4c | 票 S4c：SITL 容器内 YOPO L0 运行时分层镜像 | report-only |
| v2-clearance-replay.md | v2 | 票 V2：穿林跟踪净空门前后对比回放，只出图 | 冻结 |
| v3-sphere-replay.md | v3 | 票 V3：E-B4c 欧氏碰撞球开关对比回放，只出图 | 无 |
