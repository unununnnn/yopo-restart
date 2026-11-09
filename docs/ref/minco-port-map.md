# YOPO-MINCO 移植映射表（ADR-0006 修订 2026-10-26 的附表）

本文是 [ADR-0006](../adr/0006-yopo-minco-baseline.md) 2026-10-26 修订的附表，属于 Y0（#33）。
后续 Y1–Y9（#34–#42）的 PR 都以本表为契约：改变某一行的目标或胜出方时，必须同时修改本表和 ADR-0006 的保真登记。

**钉定版本：**

| 简称 | 仓库与版本 |
|---|---|
| MINCO | `TJU-Aerial-Robotics/YOPO@d5e1e60f83cc991602f142d9b6b96204a24d6b9f`（YOPO-MINCO 分支）；权重 `YOPO/saved/YOPO_1/epoch50.pth`，sha256 `2d85e712c8874d11cd4e523777698eff78914fdb8f1ffb5068b037f8f972f62f` |
| dc | `dairycowp/YOPO-ROS2@d2fc4fc` |
| uu | `unununnnn/yopo-ros2@wk-edbot 5c4043e`；uu-lo 指分支 `cursor/learnopt-px4-yopo-854e` |
| ys | 本仓库 |

**钉定检查：**
- `core/tests/test_upstream_pin.py` 离线核对所有钉定常量和金样元数据。
- 带 `upstream` 标记的用例联网确认上游分支 head 仍是 d5e1e60。

工单与 issue：Y0 #33 · Y1 #34 · Y2 #35 · Y3 #36 · Y4 #37 · Y5 #38 · Y6 #39 · Y7 #40 · Y8 #41 · Y9 #42。

## 1. Baseline gap (what exists vs what upstream MINCO has)

| # | Upstream MINCO element | yopo-swarm today | Gap | Ticket |
|---|---|---|---|---|
| G-a | Network forward, weights epoch50 sha 2d85e712… | `baseline.py` + CPU golden `minco_d5e1e60_cpu.npz` | none | — |
| G-b | `pred_to_traj_params`, `MincoTraj.solve` | `candidates.decode_params`, `minco_two_piece` + `minco_decode_d5e1e60.npz` | none | — |
| G-c | Depth preprocessing (`prepare_input`, NS inpaint r=1, nearest resize) | `depth.network_depth` (bit-exact) | none | — |
| G-d | TRT inference (`yopo_trt_transfer.py`, torch2trt FP16, 3 outputs) | Offline ladder only (`docker/common/golden_gpu.py`): L2 FP32 2.29 ms, max_abs 7e-6; L3 FP16 0.81 ms | **No runtime TRT backend in `infer_ipc` serve** | Y2 (#35) |
| G-e | `select_action`: admit μ_lo ≥ safe_mu → argmax score → topk continuity; none admitted → BRAKE | `rank_candidates`: admitted first, non-admitted still flyable, CORRIDOR_EMPTY flag | **Semantics differ** (we are looser) | ✅ Y1 (#34)：严格准入 |
| G-f | `calculate_yaw` (rate ≤ 0.5π rad/s, goal-weighted blend) | Takeoff alignment at 0.5 rad/s only | **Missing in flight** | ✅ Y1 (#34)：`yaw.py` + `UavRuntime.command(goal_w=…)` |
| G-g | `process_odom` z-priority goal clip to goal_length 10 | Raw goal offset, uniform normalization | **Different obs** whenever ‖g‖ > 10 m with z offset | ✅ Y1 (#34)：`goal_obs` |
| G-h | Arrive within 5 m → hold | Own goal/hold logic | Semantics to align (no z=2 hard-code) | 🟡 Y1 (#34)：`arrived()` 已移植并锁定；到达保持未接入节点 |
| G-i | Node timing: 50 Hz fixed-dt replay | Sim-time sampling + `timing.py` | ours stricter; add 30 Hz slot gating, duplicate-stamp liveness, depth isolation (unununnnn fixes) | Y3 (#36) |
| G-j | Lazy MarkerArray visuals | none | add, subscription-gated | Y3 (#36) |
| G-k | Training: `minco/`, `loss/*`, trainer, dataset, expert | none | **Missing** | Y6 (#39) |
| G-l | CUDA sensor sim, simsense stereo, dataset_generator | none (Gazebo depth) | **Missing** (offline only) | Y7 (#40) |
| G-m | — (upstream has no env-safety proof) | `freespace.py` ENV check → NO_ENV_SAFE_CANDIDATE at step 0 | G2 blocker, our own layer | Y4 (#37)，落在 #23 分支 |

## 2. Decisions (resolved, with recommendation)

| ID | Decision | Recommendation (adopted) | Why |
|---|---|---|---|
| D1 | Port spine | Upstream MINCO math runs as **ROS-free core functions** (yopo-swarm `core/`, the equivalent of dairycowp's untouched `YOPO/` dir). ROS 2 is a thin rclpy layer (dairycowp shape) hardened with unununnnn's timing/stamp fixes. | dairycowp proves "keep algorithm files untouched, change only the node" is the minimal-diff port. unununnnn proves which node defects bite on PX4 (K1–K13 in dairycowp survey). |
| D2 | Head state | Keep `plan_from_reference=True` equivalent (parent plan PVA at t_switch). | Upstream itself says to use True with a position controller; PX4 TrajectorySetpoint is one. |
| D3 | Corridor selection | **Adopt upstream strict admission**: non-admitted candidates are never flown; empty → brake branch (our certified brake, not "hold last command"). Keep the `Funnel` ordering for the admitted set; the ENV gate and collision checker stay after it. | Faithful *and* strictly safer. Upstream's hold-last-command is replaced by our brake branch because PX4 needs a certified stop. Measure the admission rate in Y5 (#38); if CORRIDOR_EMPTY > 5 % of steps, report it, but do not loosen. |
| D4 | Inference backend | `infer_ipc --backend trt-fp32` is the flight default; `trt-fp16` and `torch-cuda` remain selectable. The engine is built once on the Orin, serialized, and pinned by sha256 in a manifest (weights sha + TRT 10.16.1.11 + precision + L4T R39.2.1). | FP32 is within 7e-6 of CPU; FP16 is only decision-equivalent on one input. Replay logs the raw network outputs, so determinism does not depend on rebuilding engines. Latency is not the bottleneck: the planner step is dominated by freespace. |
| D5 | Speed | Set `velocity` explicitly to the G2 cruise speed (start at 2.0 m/s, i.e. ratio 1/3 of 6 m/s) using upstream's ratio scaling in `DecoderParams.from_config`. | Upstream trained at 6 m/s and scales by `velocity`. The current `UPSTREAM_DECODER` vel_max 6.0 is a test default, not a flight choice. |
| D6 | Yaw | Port the MINCO `calculate_yaw` verbatim (0.5·π rad/s). The takeoff alignment rate stays separate. | Fidelity. The yaw rate is independent of the position safety envelope. |
| D7 | Packages and messages | No new message package. Keep `px4_msgs/TrajectorySetpoint` (dairycowp's `quadrotor_msgs/PositionCommand` and unununnnn's `yopo_msgs/TrajectoryCommand` are both rejected). Keep the single `yopo_swarm_px4` package plus launch. Visuals use `visualization_msgs/MarkerArray`, namespaced per UAV. | ADR-0011 mandates PX4 messages. Evidence already lives in JSONL; a message package adds an ABI without a consumer. |
| D8 | Training | Run upstream training code **pinned at d5e1e60 via a thin shim** (like `baseline.py`); rewrite nothing unless it fails on aarch64. | Fidelity by construction; less code. |
| D9 | Sensor sim | Build upstream `Simulator` ROS-free in the CUDA 13.2 inference container (`LANGUAGES CUDA`, sm_87). Use dairycowp's CMake port as the template, minus the gcc-11 and `CUDA 12.8 EXACT` pins. The ament/rclcpp node is not needed. | Humble containers have no CUDA (split-container rule); `dataset_generator` is ROS-free anyway. |
| D10 | Retime | Only if Y5 (#38) shows KIN rejections > 10 % of candidates: re-solve MINCO with durations × k (bisection k ∈ [1, 2], as in unununnnn 11135ae), keeping head PVA. The result re-enters the full gate. | It is a deviation from upstream, so add it only with evidence. |

## 3. Mapping table (MINCO file → dairycowp pattern → unununnnn pattern → yopo-swarm target)

Repos and versions:
- MINCO = `TJU-Aerial-Robotics/YOPO@d5e1e60`.
- dc = `dairycowp/YOPO-ROS2@d2fc4fc`.
- uu = `unununnnn/yopo-ros2@wk-edbot 5c4043e`; "uu-lo" = branch `cursor/learnopt-px4-yopo-854e`.
- ys = `yopo-swarm`.

| # | MINCO file (path:lines) | dc pattern | uu pattern | ys target (status) | Wins / why |
|---|---|---|---|---|---|
| 1 | `YOPO/policy/yopo_network.py`, `policy/models/*` | algorithm dir copied as-is, imported by script | `yopo_planner/policy_runner.py` loads upstream root, sha-checks checkpoint, `InputContractError` | `core/baseline.py` fetch-at-SHA (✅) | **uu**: pinned, verified loading beats a vendored copy; ys already does it. |
| 2 | `YOPO/policy/state_transform.py` (`pred_to_traj_params`, `prepare_input`, `normalize_obs`) | Simple version copied | Simple version, wrapped | `candidates.decode_params`, `depth.network_depth` (✅ goldens) | **MINCO itself** (golden); neither reference has MINCO decode. |
| 3 | `YOPO/policy/primitive.py` (lattice, device singleton) | same singleton bug (K9) | n/a | CPU preprocessing workaround in `infer_ipc` (✅) | **ys**. |
| 4 | `YOPO/policy/poly_solver.py:19-136` `MincoTraj` | Poly5Solver (Simple) | `_bounded_poly5` retime ×1.15 + bisection (`planner.py`, 11135ae) | `candidates.minco_two_piece` (✅); retime only under D10 | **MINCO** for the solve; **uu** for the retime pattern if needed. |
| 5 | `poly_solver.py:139+` `wrap_to_pi`, `calculate_yaw` | Simple `calculate_yaw` copied | heading check at P0 (53c6869) | `core/yaw.py` (✅ Y1 (#34)) | **MINCO** verbatim (fidelity); uu's check reused in takeoff only. |
| 6 | `YOPO/test_yopo_ros.py:133-176` `callback_odometry`/`process_odom` (z-priority clip L164-170) | rclpy `callback_odometry`/`process_odom` (`test_yopo_ros.py:110-146`) | `ros_node.py` source-stamp handling; `rflysim_bridge/adapters.py:94-113` EstimatorLocalFrame, `:150-182` monotonic stamps | `infer_ipc.body_obs` + `goal_obs()` (✅ Y1 (#34)) | **MINCO** math; **uu** stamp contract (dc uses `now()`, K3). |
| 7 | `test_yopo_ros.py:177-235` `callback_depth`/`_process_depth` | rclpy Image callback, single-thread spin | `depth_contract.py` SourceTimestampMapper; callback isolation 5d6f5c7 | `depth.network_depth` (✅) + isolated depth callback group (Y3 (#36)) | **uu**: dc's single-thread executor starves control (K1). |
| 8 | `test_yopo_ros.py:258-263` `_corridor_mu_lo`, `:338-351` `select_action` | `argmin(score)` (Simple; n/a) | ordered candidate loop with floor checks (`planner.py:1201-1265`) | `candidates.rank_candidates` → strict admission (✅ Y1 (#34), D3) | **MINCO** semantics; uu's ordered loop is already the ys `Funnel`. |
| 9 | `test_yopo_ros.py:236-257` `_solve_best_traj` (world-frame inner/tail) | n/a | `plan_from_reference` handoff (`planner.py:1151-1166`) | `planner.py` handoff at t_switch (✅) | **ys/uu** (D2). |
| 10 | `test_yopo_ros.py:295-337` `control_pub` (50 Hz, ctrl_time += dt) | `create_timer(ctrl_dt)` wall clock; dilation under load (K1) | 30 Hz slot gating 7f795d5; steady timers 4ea53a2; tracking latch 6a31840 | `uav.command(now)` on sim time + `timing.py` (✅); slot gating + latch (Y3 (#36)) | **uu**: fixed-dt replay drifts under a slow executor. |
| 11 | PositionCommand publish (`quadrotor_msgs`) | vendored `quadrotor_msgs` + `YOPO/control_msg/_PositionCommand.py` | `yopo_msgs/msg/TrajectoryCommand.msg` (MAVROS); uu-lo `learnopt_yopo_px4/offboard.py` px4_msgs | `px4.py` TrajectorySetpoint, NED↔ENU only there (✅) | **uu-lo/ys** (ADR-0011); dc's message is ROS1-shaped. |
| 12 | `test_yopo_ros.py:128-132,146-149` `callback_set_goal`, arrive 5 m, z = 2 | same, z = 2 hard-coded (K7) | `GoalCapture` (`planner.py:111-190`), waypoint pre-advance (`ros_nodes.py:467-498`) | 3D goal param + `arrived()` (🟡 Y1 (#34)：hold 未接入) | **uu** for capture (no z hard-code); **MINCO** for the 5 m arrive radius. |
| 13 | `test_yopo_ros.py:264-294,372-511` visuals (MarkerArray, corridor lazy) | PointCloud2 always on (K12) | `EvidenceRecorder` (`ros_runtime.py`) | JSONL evidence (✅) + gated MarkerArray (Y3 (#36)) | **MINCO** lazy-publish pattern; dc's always-on visuals cost CPU. |
| 14 | `YOPO/yopo_trt_transfer.py` (torch2trt FP16, 3 outputs) | `yopo_trt_transfer.py` + `TRTModule` load (`test_yopo_ros.py:26,80`) | none (CPU inference 35–38 ms) | `docker/common/golden_gpu.py` ONNX→TRT (✅ offline) → `infer_ipc --backend trt-*` (❌ → Y2 (#35)) | **ys**: torch2trt is unmaintained for TRT 10 (K11); ONNX→TRT already passes on the Orin. |
| 15 | `YOPO/config/traj_opt.yaml`, `config/config.py` | same global cfg | `PlannerConfig.at_inference_speed` (`planner.py:241-254`) | `DecoderParams.from_config` + config_hash 18de07db… (✅) + explicit `velocity` (D5) | **uu**: an explicit speed object equals upstream ratio scaling but is visible. |
| 16 | `YOPO/train_yopo.py`, `policy/yopo_trainer.py`, `policy/yopo_dataset.py` | Simple versions copied | none | `train/` shim on pinned upstream (❌ → Y6 (#39)) | **MINCO** pinned (D8). |
| 17 | `YOPO/loss/{safety,feasible,guidance,smoothness}_loss.py`, `loss_function.py`, `YOPO/minco/*` | Simple losses | none | via shim (❌ → Y6 (#39)) | **MINCO**. |
| 18 | `YOPO/expert/test_expert.py` | none | none | diagnostic CLI (❌ → Y6 (#39)) | **MINCO**. |
| 19 | `Simulator/src/src/sensor_simulator.cu`, `test_simulator_cuda.cpp`, `config/config.yaml` | ament_cmake `sensor_simulator`, rclcpp wall timers, gcc-11 + `CUDA 12.8 EXACT` | none (RflySim) | ROS-free build in the infer container, CUDA 13.2, sm_87 (❌ → Y7 (#40)) | **dc** CMake template, minus its pins (D9). |
| 20 | `Simulator/src/simsense/*` | absent (Simple era) | none | optional stereo path (❌ → Y7 (#40)) | **MINCO**. |
| 21 | `Simulator/src/src/{dataset_generator.cpp,maps.cpp}` | ported (ROS-free) | `tools/probe_forest_route.py` route screening | datagen binary + same-seed Gazebo SDF via `scripts/build_g2_world.py` (❌ → Y7 (#40)) | **dc** build; **uu** screening idea. |
| 22 | `Controller/src/so3_*`, `NetworkControl.h` (takeoff_altitude) | ament port + `.launch.py` | MAVROS controller state machine (`rflysim_bridge/controller.py:28-36,797-840`) | PX4 v1.17 `px4.py`, `takeoff.py` (✅) | **PX4/ys** (ADR-0011); SO(3) not ported. |
| 23 | `Controller/.../launch/simulator_attitude_control.launch` | `.launch.py` | `yopo_bringup/launch/closed_loop.launch.py` | `ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py` + #27 N-UAV (✅) | **uu/ys**: per-namespace bringup is needed for multi-drone. |
| 24 | — (upstream has no EV/time contract) | none | `adapters.py:150-182`; duplicate stamps don't refresh liveness 706e694; `StartHoldGate` `ros_nodes.py:250-330` | `px4.py`, `lio_emulator.py`, `takeoff.py` (✅, stricter per ADR-0012/0013) + duplicate-stamp rule (Y3 (#36)) | **ys**, plus one **uu** fix. |
| 25 | — (upstream has no environment proof) | none | `tools/probe_forest_route.py:53-109` per-ray exit depth (69a163c); `lidar_clearance.py` relative-motion bound | `freespace.py` (🟡 → Y4 (#37) inside #23) | **uu**. |
| 26 | `YOPO/requirements.txt` (torch 2.4.1+cu118, open3d 0.19) | same; CUDA mismatch (K8) | `requirements-ci.txt` | infer image torch 2.12.0+cu132, TRT 10.16.1.11; CPU golden uses torch 2.4.1 (✅) | **ys**. |
| 27 | `YOPO/saved/YOPO_1/epoch50.pth` | different (Simple) weights | Simple weights | ADR-0007 sha 2d85e712… (✅) | **MINCO**. |
