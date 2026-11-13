"""票 S1：单机 PX4 SITL 闭环——起飞、悬停 10 s、飞到一个点、降落（report-only）。

在 scripts/sitl/Dockerfile 构建的容器里运行（ROS 2 Humble + px4_msgs release/1.17）。
所有 ENU↔NED 转换与 px4_msgs 字段构造只走 ``yopo_swarm_core.px4``（ADR-0011），本文件不自写转换。
任务逻辑（``Mission``）不依赖 rclpy，可在 box 上用 pytest 驱动；ROS 胶水只在 ``main()`` 里导入。

不发任何 PX4 参数修改命令；降落用 PX4 自己的 NAV_LAND（VehicleCommand 21）。
SITL airframe 默认 NAV_DLL_ACT=2（数据链丢失要处置），没有地面站时预检报
「No connection to the GCS」、拒绝解锁。这里不改该参数，而是像 QGC 一样向 PX4 的 GCS
MAVLink 端口（SITL 实例 0：UDP 18570）每秒发一个 GCS HEARTBEAT（只发心跳，不发其他 MAVLink 消息）。
"""

from __future__ import annotations

import argparse
import os
import dataclasses
import json
import math
import sys
import time
from dataclasses import dataclass, field

import numpy as np

from yopo_swarm_core import px4

GCS_UDP_PORT = 18570  # PX4 SITL px4-rc.mavlink 的 udp_gcs_port_local（实例 0）
VEHICLE_CMD_NAV_LAND = 21  # MAV_CMD_NAV_LAND
NAV_STATE_OFFBOARD = 14  # px4_msgs/VehicleStatus.NAVIGATION_STATE_OFFBOARD
ARMING_STATE_ARMED = 2  # px4_msgs/VehicleStatus.ARMING_STATE_ARMED


@dataclass(frozen=True)
class MissionConfig:
    takeoff_alt_m: float = 2.5
    hover_s: float = 10.0
    goal_offset_enu: tuple[float, float, float] = (5.0, 3.0, 0.0)  # 相对悬停点
    land_at_hover: bool = False  # True：悬停采样结束后直接 NAV_LAND，不飞 GOTO（票 S2）
    warmup_s: float = 1.0  # 切 offboard 前先连续发心跳和设定点
    settle_pos_m: float = 0.2
    settle_vel_mps: float = 0.3
    settle_hold_s: float = 1.0
    goto_measure_s: float = 2.0  # 到点稳住后再停这么久，到点误差取这段的平均
    arm_retry_s: float = 1.0
    arm_timeout_s: float = 60.0
    takeoff_timeout_s: float = 30.0
    goto_timeout_s: float = 30.0
    land_timeout_s: float = 60.0


@dataclass
class VehicleState:
    """规划核心 ENU 状态 + PX4 状态位。"""

    t: float  # 仿真时间 s
    position: np.ndarray
    velocity: np.ndarray
    armed: bool = False
    offboard: bool = False
    landed: bool = False
    truth: np.ndarray | None = None  # Gazebo 真值位置（世界系本就是 ENU），只用于评估


@dataclass
class Action:
    setpoint_pva: np.ndarray | None = None  # (3,3) ENU，None = 不发设定点
    yaw_enu: float | None = None
    commands: list[str] = field(default_factory=list)  # "offboard" / "arm" / "land"
    heartbeat: bool = True


@dataclass
class Metrics:
    takeoff_s: float | None = None
    hover_alt_err_rms_m: float | None = None
    hover_alt_err_max_m: float | None = None
    hover_horiz_drift_max_m: float | None = None
    hover_samples: int = 0
    goto_s: float | None = None
    goto_err_m: float | None = None
    hover_alt_err_truth_rms_m: float | None = None
    hover_alt_err_truth_max_m: float | None = None
    goto_err_truth_m: float | None = None
    hover_est_truth_alt_bias_mean_m: float | None = None
    hover_est_truth_alt_bias_rms_m: float | None = None
    hover_est_truth_alt_bias_max_m: float | None = None
    land_s: float | None = None
    landed_and_disarmed: bool = False
    phase_reached: str = "WARMUP"
    failure: str | None = None


class Mission:
    """仿真时间驱动的状态机：WARMUP → ARMING → TAKEOFF → HOVER → GOTO → GOTO_HOLD → LAND → DONE / FAIL。

    误差两套口径：*_m 相对 PX4 自己的估计（VehicleOdometry，飞控实际跟踪的量）；
    *_truth_* 相对 Gazebo 真值（起飞前真值位置 + 指令偏移），含估计器误差。
    hover_est_truth_alt_bias_* 是悬停段估计高度减真值高度（同一世界系，不经各自 home）。
    """

    def __init__(self, cfg: MissionConfig | None = None):
        self.cfg = cfg or MissionConfig()
        self.phase = "WARMUP"
        self.t_phase = None  # 当前阶段开始的仿真时间
        self.home: np.ndarray | None = None
        self.yaw_enu: float | None = None
        self.hover_sp: np.ndarray | None = None
        self.goal_sp: np.ndarray | None = None
        self.t_last_arm = -math.inf
        self.t_settle: float | None = None
        self.settle_err: list[float] = []
        self.hover_alt_err: list[float] = []
        self.hover_horiz: list[float] = []
        self.hover_truth: list[float] = []
        self.hover_est_truth_dz: list[float] = []
        self.goto_err: list[float] = []
        self.goto_truth: list[float] = []
        self.truth_home: np.ndarray | None = None
        self.land_sent = False
        self.metrics = Metrics()

    @property
    def done(self) -> bool:
        return self.phase in ("DONE", "FAIL")

    def _enter(self, phase: str, t: float) -> None:
        self.phase = phase
        self.t_phase = t
        self.t_settle = None
        self.settle_err = []
        if phase not in ("DONE", "FAIL"):
            self.metrics.phase_reached = phase

    def _fail(self, why: str, t: float) -> Action:
        self.metrics.failure = f"{self.phase}: {why}"
        self._enter("FAIL", t)
        return Action(heartbeat=False)

    @staticmethod
    def _pva(p: np.ndarray) -> np.ndarray:
        return np.stack([np.asarray(p, dtype=float), np.zeros(3), np.zeros(3)])

    def _settled(self, s: VehicleState, target: np.ndarray) -> bool:
        """在目标附近连续 settle_hold_s 才算到；记录这段的位置误差。"""
        err = float(np.linalg.norm(s.position - target))
        ok = err < self.cfg.settle_pos_m and float(np.linalg.norm(s.velocity)) < self.cfg.settle_vel_mps
        if not ok:
            self.t_settle = None
            self.settle_err = []
            return False
        if self.t_settle is None:
            self.t_settle = s.t
        self.settle_err.append(err)
        return s.t - self.t_settle >= self.cfg.settle_hold_s

    def step(self, s: VehicleState) -> Action:
        c = self.cfg
        if self.t_phase is None:
            self.t_phase = s.t
        if self.home is None:
            self.home = np.asarray(s.position, dtype=float).copy()
            self.hover_sp = self.home + np.array([0.0, 0.0, c.takeoff_alt_m])
            self.goal_sp = self.hover_sp + np.asarray(c.goal_offset_enu, dtype=float)
            self.yaw_enu = None
        if self.truth_home is None and s.truth is not None and self.phase in ("WARMUP", "ARMING"):
            self.truth_home = np.asarray(s.truth, dtype=float).copy()
        dt = s.t - self.t_phase

        if self.phase == "WARMUP":
            if dt >= c.warmup_s:
                self._enter("ARMING", s.t)
            return Action(self._pva(self.home), self.yaw_enu)

        if self.phase == "ARMING":
            if s.armed and s.offboard:
                self._enter("TAKEOFF", s.t)
                return Action(self._pva(self.hover_sp), self.yaw_enu)
            if dt > c.arm_timeout_s:
                return self._fail(f"armed={s.armed} offboard={s.offboard} after {c.arm_timeout_s}s", s.t)
            cmds = []
            if s.t - self.t_last_arm >= c.arm_retry_s:
                self.t_last_arm = s.t
                if not s.offboard:
                    cmds.append("offboard")
                if not s.armed:
                    cmds.append("arm")
            return Action(self._pva(self.home), self.yaw_enu, cmds)

        if self.phase in ("TAKEOFF", "HOVER", "GOTO", "GOTO_HOLD") and not s.armed:
            return self._fail("disarmed in flight", s.t)
        if self.phase in ("TAKEOFF", "HOVER", "GOTO", "GOTO_HOLD") and not s.offboard:
            return self._fail("left offboard", s.t)

        if self.phase == "TAKEOFF":
            if self._settled(s, self.hover_sp):
                self.metrics.takeoff_s = dt
                self._enter("HOVER", s.t)
            elif dt > c.takeoff_timeout_s:
                return self._fail("not settled at takeoff altitude", s.t)
            return Action(self._pva(self.hover_sp), self.yaw_enu)

        if self.phase == "HOVER":
            self.hover_alt_err.append(float(s.position[2] - self.hover_sp[2]))
            self.hover_horiz.append(float(np.linalg.norm(s.position[:2] - self.hover_sp[:2])))
            if s.truth is not None and self.truth_home is not None:
                self.hover_truth.append(float(s.truth[2] - (self.truth_home[2] + c.takeoff_alt_m)))
            if s.truth is not None:
                self.hover_est_truth_dz.append(float(s.position[2] - s.truth[2]))
            if dt >= c.hover_s:
                a = np.asarray(self.hover_alt_err)
                self.metrics.hover_alt_err_rms_m = float(np.sqrt(np.mean(a * a)))
                self.metrics.hover_alt_err_max_m = float(np.max(np.abs(a)))
                self.metrics.hover_horiz_drift_max_m = float(np.max(self.hover_horiz))
                self.metrics.hover_samples = len(a)
                if self.hover_truth:
                    b = np.asarray(self.hover_truth)
                    self.metrics.hover_alt_err_truth_rms_m = float(np.sqrt(np.mean(b * b)))
                    self.metrics.hover_alt_err_truth_max_m = float(np.max(np.abs(b)))
                if self.hover_est_truth_dz:
                    d = np.asarray(self.hover_est_truth_dz)
                    self.metrics.hover_est_truth_alt_bias_mean_m = float(np.mean(d))
                    self.metrics.hover_est_truth_alt_bias_rms_m = float(np.sqrt(np.mean(d * d)))
                    self.metrics.hover_est_truth_alt_bias_max_m = float(np.max(np.abs(d)))
                if c.land_at_hover:
                    self._enter("LAND", s.t)
                    return Action(self._pva(self.hover_sp), self.yaw_enu, ["land"])
                self._enter("GOTO", s.t)
                return Action(self._pva(self.goal_sp), self.yaw_enu)
            return Action(self._pva(self.hover_sp), self.yaw_enu)

        if self.phase == "GOTO":
            if self._settled(s, self.goal_sp):
                self.metrics.goto_s = dt
                self._enter("GOTO_HOLD", s.t)
            elif dt > c.goto_timeout_s:
                err = float(np.linalg.norm(s.position - self.goal_sp))
                self.metrics.goto_err_m = err
                return self._fail(f"not settled at goal, err={err:.3f} m", s.t)
            return Action(self._pva(self.goal_sp), self.yaw_enu)

        if self.phase == "GOTO_HOLD":
            self.goto_err.append(float(np.linalg.norm(s.position - self.goal_sp)))
            if s.truth is not None and self.truth_home is not None:
                goal_truth = self.truth_home + np.array([0.0, 0.0, c.takeoff_alt_m]) + np.asarray(c.goal_offset_enu)
                self.goto_truth.append(float(np.linalg.norm(s.truth - goal_truth)))
            if dt >= c.goto_measure_s:
                self.metrics.goto_err_m = float(np.mean(self.goto_err))
                if self.goto_truth:
                    self.metrics.goto_err_truth_m = float(np.mean(self.goto_truth))
                self._enter("LAND", s.t)
                return Action(self._pva(self.goal_sp), self.yaw_enu, ["land"])
            return Action(self._pva(self.goal_sp), self.yaw_enu)

        if self.phase == "LAND":
            # NAV_LAND 之后 PX4 退出 offboard；不再发设定点和心跳。
            if s.landed and not s.armed:
                self.metrics.land_s = dt
                self.metrics.landed_and_disarmed = True
                self._enter("DONE", s.t)
                return Action(heartbeat=False)
            if dt > c.land_timeout_s:
                return self._fail(f"landed={s.landed} armed={s.armed}", s.t)
            return Action(heartbeat=False)

        return Action(heartbeat=False)


# ---------------------------------------------------------------- ROS 胶水（只在容器里跑）


def _fill(msg, fields: dict):
    for k, v in fields.items():
        setattr(msg, k, v)
    return msg


def main(argv: list[str] | None = None) -> int:  # pragma: no cover - 需要 rclpy/px4_msgs/gz
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--wall-timeout", type=float, default=900.0)
    ap.add_argument("--world", default="default")
    ap.add_argument("--model", default="x500_0", help="Gazebo 里的机体模型名（PX4 gz_x500 实例 0）")
    ap.add_argument("--preflight-timeout", type=float, default=240.0, help="墙钟 s：预检一直不过就失败退出")
    args = ap.parse_args(argv)

    import threading

    import rclpy
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzNode
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLandDetected,
        VehicleOdometry,
        VehicleStatus,
    )
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    from pymavlink import mavutil

    lock = threading.Lock()
    sim = {"t": None, "truth": None}
    stop = threading.Event()

    def gcs_heartbeat():
        conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{GCS_UDP_PORT}", source_system=255, source_component=190)
        while not stop.is_set():
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE
            )
            stop.wait(1.0)

    threading.Thread(target=gcs_heartbeat, daemon=True).start()

    def on_clock(msg):
        with lock:
            sim["t"] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_pose(msg):
        for p in msg.pose:
            if p.name == args.model:
                with lock:
                    sim["truth"] = np.array([p.position.x, p.position.y, p.position.z])
                return

    gz = GzNode()
    if not gz.subscribe(Clock, f"/world/{args.world}/clock", on_clock):
        print("gz clock subscribe failed", file=sys.stderr)
        return 2
    if not gz.subscribe(Pose_V, f"/world/{args.world}/pose/info", on_pose):
        print("gz pose subscribe failed", file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node("yopo_s1_offboard")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    pub_ocm = node.create_publisher(OffboardControlMode, px4.topic("in", "offboard_control_mode"), qos)
    pub_sp = node.create_publisher(TrajectorySetpoint, px4.topic("in", "trajectory_setpoint"), qos)
    pub_cmd = node.create_publisher(VehicleCommand, px4.topic("in", "vehicle_command"), qos)
    st = {"odom": None, "status": None, "land": None}
    node.create_subscription(VehicleOdometry, px4.topic("out", "vehicle_odometry"), lambda m: st.__setitem__("odom", m), qos)
    node.create_subscription(VehicleStatus, px4.topic("out", "vehicle_status"), lambda m: st.__setitem__("status", m), qos)
    node.create_subscription(
        VehicleLandDetected, px4.topic("out", "vehicle_land_detected"), lambda m: st.__setitem__("land", m), qos
    )

    def now_us() -> int:
        return node.get_clock().now().nanoseconds // 1000

    mission = Mission()
    wall0 = time.monotonic()
    wall_ready = sim_ready = None
    trace = []
    preflight_seen = False
    try:
        while not mission.done:
            rclpy.spin_once(node, timeout_sec=0.02)
            wall = time.monotonic() - wall0
            if wall > args.wall_timeout:
                mission.metrics.failure = f"{mission.phase}: wall timeout {args.wall_timeout}s"
                break
            with lock:
                t_sim, truth = sim["t"], sim["truth"]
            o, vs, ld = st["odom"], st["status"], st["land"]
            if t_sim is None or o is None or vs is None or ld is None:
                if wall > args.preflight_timeout:
                    missing = [k for k, v in (("gz_clock", t_sim), ("odometry", o), ("status", vs), ("land", ld)) if v is None]
                    mission.metrics.failure = f"PREFLIGHT: no {','.join(missing)} after {args.preflight_timeout}s wall"
                    break
                continue
            preflight_seen = preflight_seen or bool(vs.pre_flight_checks_pass)
            if wall_ready is None:
                if not vs.pre_flight_checks_pass:
                    if wall > args.preflight_timeout:
                        mission.metrics.failure = f"PREFLIGHT: pre_flight_checks_pass=false after {args.preflight_timeout}s wall"
                        break
                    continue
                wall_ready, sim_ready = wall, t_sim
            try:
                enu = px4.odometry_to_enu(o.position, o.q, o.velocity, o.timestamp, o.pose_frame, o.velocity_frame)
            except ValueError:
                continue
            s = VehicleState(
                t=t_sim,
                position=enu.position,
                velocity=enu.velocity,
                armed=vs.arming_state == ARMING_STATE_ARMED,
                offboard=vs.nav_state == NAV_STATE_OFFBOARD,
                landed=bool(ld.landed),
                truth=truth,
            )
            act = mission.step(s)
            ts = now_us()
            if act.heartbeat:
                pub_ocm.publish(_fill(OffboardControlMode(), px4.offboard_control_mode(ts)))
            if act.setpoint_pva is not None:
                pub_sp.publish(_fill(TrajectorySetpoint(), px4.trajectory_setpoint(act.setpoint_pva, act.yaw_enu, ts)))
            for c in act.commands:
                if c == "offboard":
                    f = px4.offboard_mode_command(ts)
                elif c == "arm":
                    f = px4.arm_command(ts)
                else:
                    f = px4.vehicle_command(VEHICLE_CMD_NAV_LAND, ts, param4=math.nan)
                pub_cmd.publish(_fill(VehicleCommand(), f))
            if not trace or t_sim - trace[-1][0] >= 0.1:
                tz = [None] * 3 if truth is None else [round(float(x), 4) for x in truth]
                trace.append([round(t_sim, 3), mission.phase, *[round(float(x), 4) for x in s.position], *tz])
            # 约 50 Hz 墙钟；PX4 要求心跳 > 2 Hz
            time.sleep(0.02)
    finally:
        stop.set()
        wall_end = time.monotonic() - wall0
        with lock:
            t_end = sim["t"]
        node.destroy_node()
        rclpy.try_shutdown()

    m = mission.metrics
    mission_wall = None if wall_ready is None else wall_end - wall_ready
    mission_sim = None if sim_ready is None or t_end is None else t_end - sim_ready
    out = {
        "schema": "yopo-restart/s1-sitl-smoke/1",
        "report_only": True,
        "positioning": os.environ.get("YOPO_POSITIONING", "default"),
        "ok": mission.phase == "DONE",
        "phase": mission.phase,
        "config": dataclasses.asdict(mission.cfg),
        "metrics": dataclasses.asdict(m),
        "wall_to_preflight_ok_s": wall_ready,
        "preflight_checks_pass_seen": preflight_seen,
        "mission_wall_s": mission_wall,
        "mission_sim_s": mission_sim,
        "rtf": (mission_sim / mission_wall) if mission_wall and mission_sim is not None else None,
        "home_enu": None if mission.home is None else mission.home.tolist(),
        "goal_enu": None if mission.goal_sp is None else mission.goal_sp.tolist(),
        "trace": trace,
    }
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps({k: v for k, v in out.items() if k != "trace"}, indent=1))
    return 0 if out["ok"] else 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
