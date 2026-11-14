"""票 S4b：容器内把 YOPO L0 规划桥接到 PX4 ROS 2 话题（report-only）。

订阅每机 ``VehicleOdometry``（``/fmu/out/vehicle_odometry`` 或 ``/px4_N/...``）、
可选的深度图 ``/yopo/depth_image`` 和障碍摘要 ``/yopo/obstacle_summary``。
ENU↔NED 只走 ``yopo_swarm_core.px4``。设定点是 NED 的 ``TrajectorySetpoint``。

``rclpy`` 只在真正开环时导入。``--dry-run`` 只打印话题表，不起飞，不写飞控参数。
默认离线假 PX4 仍走 ``run_yopo_l0.py``。本文件不改 ``offboard_mission.py``。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

from yopo_l0.scene import load_card
from yopo_l0.sitl_bridge import (
    LiveFleetConfig,
    VehicleInbox,
    live_topic_plan,
    make_live_session,
    normalize_positioning,
    outcome_exit_code,
    vehicle_topics,
)

_REPO = Path(__file__).resolve().parents[2]
ARMING_STATE_ARMED = 2
NAV_STATE_OFFBOARD = 14
# PX4 v1.17 ``COM_OF_LOSS_T`` 默认 1 s。规划一步会堵住主循环，设定点要在这段时间里继续发。
SETPOINT_PERIOD_S = 0.05


class MissionClock:
    """任务时间从第一次拿到仿真钟开始。

    Gazebo ``/clock`` 是开机以来的绝对时间。PX4 启动先走掉二十多秒，
    再拿它和场景卡 ``duration_s`` 比较，会话里实际飞不够这个时长。
    """

    def __init__(self, max_s: float):
        limit = float(max_s)
        if not limit > 0.0:
            raise ValueError(f"max_s 必须为正，收到 {max_s!r}")
        self.max_s = limit
        self.origin: float | None = None

    def elapsed(self, t_sim: float) -> float:
        t = float(t_sim)
        if self.origin is None:
            self.origin = t
        return t - self.origin

    def expired(self, t_sim: float) -> bool:
        return self.elapsed(t_sim) > self.max_s


def hold_setpoint(setpoint: dict | None) -> dict | None:
    """心跳重发只守位置和航向。速度、加速度前馈清零。

    规划一步常常超过 1 s。若把上一拍的 velocity 按 20 Hz 再发，PX4 会把这口
    前馈一直积下去，三机都顺着这口速度漂。位置设定点仍是已经转到本地 NED 的那一份。
    """
    if setpoint is None:
        return None
    held = dict(setpoint)
    held["velocity"] = [0.0, 0.0, 0.0]
    held["acceleration"] = [0.0, 0.0, 0.0]
    return held


def streamed_setpoint(command, tracked: dict | None = None) -> tuple[dict | None, dict | None]:
    """心跳发什么。

    PLAN 阶段 ``tracked`` 是轨迹在当前仿真钟上的设定点，速度和加速度保留。
    这口速度属于这一时刻，不是把 1 s 前的前馈再积一遍。
    没有轨迹时，以及其他阶段，仍只守位置和航向。
    """
    if not getattr(command, "heartbeat", False):
        return None, None
    if tracked is None:
        return offboard_stream(command)
    mode = command.offboard
    stamp = tracked.get("timestamp")
    if mode is not None and stamp is not None:
        mode = dict(mode)
        mode["timestamp"] = stamp
    return mode, tracked


def offboard_stream(command) -> tuple[dict | None, dict | None]:
    """持续心跳只带 OffboardControlMode 和位置保持设定点。

    解锁、降落这类 VehicleCommand 不在这里重复发。设定点已是本地 NED，这里不再做坐标变换。
    """
    if not getattr(command, "heartbeat", False):
        return None, None
    return command.offboard, hold_setpoint(command.setpoint)


class PreflightLatch:
    """启动预检只要求第一次全体通过，通过后锁存。

    状态：
    - ``WAIT``：还没有一拍是每架都有样本且 ``preflight_ok``。这一拍不 step。
    - ``LATCHED``：解锁 / 起飞前已经通过一次。之后继续 step。
    - ``FAIL``：锁存前墙钟超过 ``--preflight-timeout``。这才是启动预检失败。

    PX4 用 ``canArm(当前模式)`` 刷新 ``pre_flight_checks_pass``，着陆上锁后会把它拉低。
    锁存之后这种失败留在 ``LATCHED``：不停 step，也不记成启动 ``PREFLIGHT``。
    """

    WAIT = "WAIT"
    LATCHED = "LATCHED"
    FAIL = "FAIL"

    def __init__(self, timeout_s: float):
        timeout = float(timeout_s)
        if not timeout > 0.0:
            raise ValueError(f"preflight timeout 必须为正，收到 {timeout_s!r}")
        self.timeout_s = timeout
        self.state = self.WAIT
        self.failure = ""

    def allow_step(self, boxes: dict, wall_s: float) -> bool:
        """``boxes`` 的值是带 ``preflight_ok`` 的样本；``None`` 表示这一拍还没齐。"""
        if self.state == self.FAIL:
            return False
        if self.state == self.LATCHED:
            return True
        ready = bool(boxes) and all(box is not None and bool(box.preflight_ok) for box in boxes.values())
        if ready:
            self.state = self.LATCHED
            return True
        if float(wall_s) > self.timeout_s:
            missing = [ns or "/" for ns, box in boxes.items() if box is None]
            self.failure = "PREFLIGHT " + (",".join(missing) if missing else "checks")
            self.state = self.FAIL
            return False
        return False


class PreflightLatch:
    """启动预检只要求第一次全体通过，通过后锁存。

    状态：
    - ``WAIT``：还没有一拍是每架都有样本且 ``preflight_ok``。这一拍不 step。
    - ``LATCHED``：解锁 / 起飞前已经通过一次。之后继续 step。
    - ``FAIL``：锁存前墙钟超过 ``--preflight-timeout``。这才是启动预检失败。

    PX4 用 ``canArm(当前模式)`` 刷新 ``pre_flight_checks_pass``，着陆上锁后会把它拉低。
    锁存之后这种失败留在 ``LATCHED``：不停 step，也不记成启动 ``PREFLIGHT``。
    """

    WAIT = "WAIT"
    LATCHED = "LATCHED"
    FAIL = "FAIL"

    def __init__(self, timeout_s: float):
        timeout = float(timeout_s)
        if not timeout > 0.0:
            raise ValueError(f"preflight timeout 必须为正，收到 {timeout_s!r}")
        self.timeout_s = timeout
        self.state = self.WAIT
        self.failure = ""

    def allow_step(self, boxes: dict, wall_s: float) -> bool:
        """``boxes`` 的值是带 ``preflight_ok`` 的样本；``None`` 表示这一拍还没齐。"""
        if self.state == self.FAIL:
            return False
        if self.state == self.LATCHED:
            return True
        ready = bool(boxes) and all(box is not None and bool(box.preflight_ok) for box in boxes.values())
        if ready:
            self.state = self.LATCHED
            return True
        if float(wall_s) > self.timeout_s:
            missing = [ns or "/" for ns, box in boxes.items() if box is None]
            self.failure = "PREFLIGHT " + (",".join(missing) if missing else "checks")
            self.state = self.FAIL
            return False
        return False


def _scene_path(raw: str) -> Path:
    path = Path(raw)
    if path.is_file():
        return path
    alt = _REPO / raw
    if alt.is_file():
        return alt
    raise SystemExit(f"找不到场景卡：{raw}")


def _write(path: str, payload: dict) -> None:
    text = json.dumps(payload, indent=1, ensure_ascii=False) + "\n"
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    sys.stdout.write(text)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scene", default="scenes/g2-pillar.json")
    ap.add_argument("--n", type=int, default=None, help="必须和场景卡里的飞机数一致")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--wall-timeout", type=float, default=900.0)
    ap.add_argument("--max-s", type=float, default=None, help="仿真时间上限，默认用场景卡 duration_s")
    ap.add_argument("--positioning", choices=("default", "rtk"), default=None)
    ap.add_argument("--world", default="default")
    ap.add_argument("--preflight-timeout", type=float, default=240.0)
    ap.add_argument("--dry-run", action="store_true", help="只写话题和定位模式，不连 ROS")
    return ap.parse_args(argv)


def _positioning(arg: str | None) -> str:
    if arg:
        return normalize_positioning(arg)
    return normalize_positioning(os.environ.get("YOPO_POSITIONING", "default"))


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.wall_timeout > 0.0:
        print("--wall-timeout 必须为正", file=sys.stderr)
        return 2
    positioning = _positioning(args.positioning)
    card = load_card(_scene_path(args.scene))
    n = len(card.card["drones"])
    if args.n is not None and int(args.n) != n:
        print(f"--n {args.n} 和场景 {card.name} 的飞机数 {n} 不一致", file=sys.stderr)
        return 2
    if args.dry_run:
        _write(args.out, live_topic_plan(card, positioning=positioning, seed=args.seed))
        return 0
    try:
        return _run_ros(args, card, positioning)
    except ModuleNotFoundError as exc:
        missing = exc.name or "依赖"
        print(f"实飞入口停住：当前解释器没有 {missing}。不写飞控参数。", file=sys.stderr)
        return 2


def _fill(msg, fields: dict):
    for key, value in fields.items():
        setattr(msg, key, value)
    return msg


def _run_ros(args, card, positioning: str) -> int:  # pragma: no cover - 需要 rclpy / px4_msgs
    import rclpy
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLandDetected,
        VehicleOdometry,
        VehicleStatus,
    )
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    task = card.card.get("task") or {}
    max_s = float(args.max_s) if args.max_s is not None else float(task.get("duration_s", 45.0))
    session = make_live_session(
        card,
        seed=args.seed,
        live=LiveFleetConfig(positioning=positioning, depth_source="auto", initial_phase="WARMUP"),
    )
    ports = [int(slot.ident["gcs_port"]) for slot in session.slots]
    stop = threading.Event()

    def gcs_heartbeat() -> None:
        from pymavlink import mavutil

        conns = [
            mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=255, source_component=190)
            for port in ports
        ]
        while not stop.is_set():
            for conn in conns:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
            stop.wait(1.0)

    threading.Thread(target=gcs_heartbeat, daemon=True).start()

    lock = threading.Lock()
    sim = {"t": None}

    def on_clock(msg) -> None:
        with lock:
            sim["t"] = msg.sim.sec + msg.sim.nsec * 1e-9

    try:
        from gz.msgs10.clock_pb2 import Clock
        from gz.transport13 import Node as GzNode

        gz = GzNode()
        gz.subscribe(Clock, f"/world/{args.world}/clock", on_clock)
    except Exception:
        gz = None

    rclpy.init()
    node = rclpy.create_node("yopo_l0_sitl_live")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    pubs = {}
    slots = {}
    for slot in session.slots:
        ns = slot.ident["namespace"]
        topics = vehicle_topics(ns)
        state = {"odom": None, "status": None, "land": None, "depth": None, "summary": None}
        slots[ns] = state
        pubs[ns] = {
            "ocm": node.create_publisher(OffboardControlMode, topics["offboard_control_mode"], qos),
            "sp": node.create_publisher(TrajectorySetpoint, topics["trajectory_setpoint"], qos),
            "cmd": node.create_publisher(VehicleCommand, topics["vehicle_command"], qos),
        }
        node.create_subscription(VehicleOdometry, topics["vehicle_odometry"], lambda msg, box=state: box.__setitem__("odom", msg), qos)
        node.create_subscription(VehicleStatus, topics["vehicle_status"], lambda msg, box=state: box.__setitem__("status", msg), qos)
        node.create_subscription(
            VehicleLandDetected, topics["vehicle_land_detected"], lambda msg, box=state: box.__setitem__("land", msg), qos
        )
        node.create_subscription(Image, topics["depth_image"], lambda msg, box=state: box.__setitem__("depth", msg), sensor_qos)
        node.create_subscription(
            String, topics["obstacle_summary"], lambda msg, box=state: box.__setitem__("summary", msg.data), sensor_qos
        )

    def inbox_for(ns: str) -> VehicleInbox | None:
        state = slots[ns]
        odom, status, land = state["odom"], state["status"], state["land"]
        if odom is None or status is None or land is None:
            return None
        summary = state["summary"]
        return VehicleInbox(
            odometry=odom,
            depth=state["depth"],
            obstacle_summary=summary if summary else None,
            armed=status.arming_state == ARMING_STATE_ARMED,
            offboard=status.nav_state == NAV_STATE_OFFBOARD,
            landed=bool(land.landed),
            preflight_ok=bool(status.pre_flight_checks_pass),
        )

    wall0 = time.monotonic()
    last_elapsed = None
    physics_dt = float(session.slots[0].bridge.cfg.physics_dt)
    failure = ""
    preflight = PreflightLatch(args.preflight_timeout)
    clock = MissionClock(max_s)
    streamed = {"commands": []}
    stream_lock = threading.Lock()

    def publish_stream() -> None:
        # 规划（场景卡相机 + 网络）会占住主循环超过 COM_OF_LOSS_T。
        # PLAN 阶段按仿真钟沿当前轨迹往前采样，速度是这一时刻的前馈。
        # 其他阶段只重复位置和航向。解锁、降落不在这里重发。
        while not stop.is_set():
            with lock:
                t_sim = sim["t"]
            origin = clock.origin
            elapsed = None if t_sim is None or origin is None else float(t_sim) - float(origin)
            with stream_lock:
                commands = list(streamed["commands"])
            for command in commands:
                tracked = None
                if elapsed is not None and getattr(command, "phase", "") == "PLAN":
                    tracked = session.stream_local_setpoint(command.namespace, elapsed)
                mode, setpoint = streamed_setpoint(command, tracked)
                pub = pubs[command.namespace]
                if mode is not None:
                    pub["ocm"].publish(_fill(OffboardControlMode(), mode))
                if setpoint is not None:
                    pub["sp"].publish(_fill(TrajectorySetpoint(), setpoint))
            stop.wait(SETPOINT_PERIOD_S)

    threading.Thread(target=publish_stream, daemon=True).start()
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            wall = time.monotonic() - wall0
            if wall > args.wall_timeout:
                failure = f"WALL_TIMEOUT {args.wall_timeout}s"
                break
            with lock:
                t_sim = sim["t"]
            boxes = {ns: inbox_for(ns) for ns in slots}
            if not preflight.allow_step(boxes, wall):
                if preflight.failure:
                    failure = preflight.failure
                    break
                time.sleep(0.02)
                continue
            if t_sim is None:
                present = next((box for box in boxes.values() if box is not None), None)
                if present is None:
                    if last_elapsed is None:
                        time.sleep(0.02)
                        continue
                    elapsed = last_elapsed
                else:
                    elapsed = clock.elapsed(float(present.odometry.timestamp) * 1e-6)
            else:
                elapsed = clock.elapsed(float(t_sim))
            if elapsed > max_s and session.result()["outcome"] == "RUNNING":
                failure = f"MAX_S {max_s}"
                break
            fresh = last_elapsed is None or float(elapsed) + 1e-9 >= float(last_elapsed) + physics_dt
            if fresh:
                commands = session.step(float(elapsed), boxes)
                last_elapsed = float(elapsed)
            else:
                # 仿真时钟还没走到下一拍时，只重复上一拍的心跳和设定点，避免把解锁/降落再发一遍。
                commands = session.step(float(last_elapsed), boxes)
            with stream_lock:
                streamed["commands"] = list(commands)
            for command in commands:
                if fresh:
                    pub = pubs[command.namespace]
                    for fields in command.vehicle_commands:
                        pub["cmd"].publish(_fill(VehicleCommand(), fields))
            snapshot = session.result()
            if snapshot["ok"] or snapshot.get("terminal"):
                break
            time.sleep(0.02)
    finally:
        stop.set()
        node.destroy_node()
        rclpy.try_shutdown()

    payload = session.result(adapter="px4-ros")
    # 锁存后的 preflight_ok=false 不会写入 failure，着陆上锁不能把已飞完的结果改成启动预检失败。
    if failure and not payload["ok"]:
        payload["outcome"] = "FAIL"
        payload["reason"] = failure
        payload["ok"] = False
    payload["positioning"] = positioning
    payload["mission_t0_s"] = clock.origin
    _write(args.out, payload)
    return outcome_exit_code(payload)


if __name__ == "__main__":
    sys.exit(main())
