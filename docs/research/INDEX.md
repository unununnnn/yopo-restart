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
| d6-teacher-altitude-runs.json | d6 | （无说明字段） | 无 |
| d6-teacher-altitude.md | d6 | D6：教师飞行高度扩到 {1.0, 1.5} 的规划，以及 z 平移可行性（report-only） | report-only、冻结 |
| d6b-pipeline.md | d6b | D6b：1.0 m 与 1.5 m 教师混合转换、训练、双高度评测 | report-only、冻结 |
| e-a1-inter-drone-filter.md | e-a1 | 票 E-A1：机间过滤开关，默认关 | report-only、冻结 |
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
| e-c6-six-density-runs.json | e-c6 | 只报数。report-only。不改默认 safe_radius 0.05 m、empty_brake false、track_clearance false、inter_drone_filter 关。E-A1 阈值 1.0 m、empty_brake、track_clearance 打开即偏离上游 d5e1e60。track_margin 0.08 m 是假设，用来盖已观测的 0.058–0.073 m 跟踪差。六机 0.14 对向卡只存在于进程内存，不写入 scenes/。这是假设，不是论文复现（障碍半径 0.20 m，边长 20.0 m）。未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。 | report-only、冻结 |
| e-c6-six-density.md | e-c6 | 票 E-C6：六机密度场 0.14，只报数 | report-only、冻结 |
| e-combo-a1-d1-b2-runs.json |  | 三机两张卡的规划器 commit 是 616909b；合并 origin/main 之后全关 seed 0 复跑轨迹相同。八机只收 seed 0，跑在合并之后。影子盾开。全关、E-A1、D1 两个进程；E-B2、三个一起、八机单独一个进程。backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。密林上这个计数等于调用次数，seed 0 再跑轨迹会变。 | 无 |
| e-combo-a1-d1-b2.md |  | E-A1、D1、E-B2 三个开关一起量 | report-only、冻结 |
| e-combo-six-forest-runs.json |  | 墙钟不参与轨迹，也不进已冻门。backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。整表 report-only。 | report-only、冻结 |
| e-combo-six-forest.md |  | 六机林带上把 E-A1、D1、E-B2 三个开关一起量 | report-only、冻结 |
| e-combo-swap8-seeds-runs.json |  | 墙钟不参与轨迹。combo 的 backend_budget_cuts 是后段 0.02 s 墙钟预算把迭代截断的次数。 | 无 |
| e-combo-swap8-seeds.md |  | 八机开阔对向补跑种子 1–9 | report-only、冻结 |
| e-pg2-forest-gate-switch-runs.json |  | 整表 report-only。open 偏离当前场景卡（卡上 progress_gate=false，没有 leader_margin_m）。余量写在进程内的 task.leader_margin_m，不写回场景卡。墙钟不进门。 | report-only、冻结 |
| e-pg2-forest-gate-switch.md |  | 票 E-PG2：六机林带进度门控的超前余量做成开关 | report-only、冻结 |
| ego-c2-swap3.md |  | 票 E-C2：三机开阔对向（只报数） | report-only、冻结 |
| ego-swarm-baseline.md |  | EGO-Swarm 外部基线（E-C1） | 无 |
| ego-swarm-comparison.md |  | yopo-restart 与 EGO-Swarm 对照 | report-only、冻结 |
| ego-swarm-density.json |  | （无说明字段） | 无 |
| p1-inference-speed.md | p1 | P1：YOPO 单次推理耗时 | 冻结 |
| p1b-sensor-speed.md | p1b | P1b：深度相机求交加速（默认关） | report-only、冻结 |
| s4-sitl-bridge.md | s4 | 票 S4：YOPO L0 与 PX4 SITL 的 ENU/NED 闭环桥 | report-only |
| v2-clearance-replay.md | v2 | 票 V2：穿林跟踪净空门前后对比回放，只出图 | 冻结 |
