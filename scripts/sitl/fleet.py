"""票 S2：多机 PX4 SITL 的布局与起飞–悬停–降落任务（report-only）。

布局不写飞控参数。instance、MAV_SYS_ID、uXRCE namespace 按 PX4 v1.17 的 posix rcS：

- ``MAV_SYS_ID = px4_instance + 1``，``UXRCE_DDS_KEY`` 同这个数（rcS 自己写）
- instance 0 没有 namespace；instance > 0 时 namespace 是 ``px4_<instance>``
- 因此 n 架从 instance 1 起，namespace 才是 ``px4_1 .. px4_n``
- 全部实例共用一个 XRCE Agent 端口、一个 DDS 域（容器里的 ``ROS_DOMAIN_ID``）

``layout`` 子命令只用标准库，方便宿主机在没有 numpy 时校验 ``--n``。
任务状态机复用 ``offboard_mission.Mission``（``land_at_hover``），ROS 胶水只在 ``run`` 里导入。
"""

from __future__ import annotations

import argparse
import dataclasses
import importlib.util
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# 与 yopo_swarm_core.px4.XRCE_DEFAULT_PORT 保持一致；布局路径不导入 px4（它依赖 numpy）。
XRCE_PORT = 8888
DDS_DOMAIN_ID = 0
MAX_N = 3
GCS_PORT_BASE = 18570  # px4-rc.mavlink：udp_gcs_port_local = 18570 + px4_instance
SPAWN_SPACING_M = 3.0
TAKEOFF_ALT_M = 1.5
HOVER_S = 10.0  # 仿真时间，只用来采样估计/真值误差，不是冻结判据
SCHEMA = "yopo-restart/s2-sitl-fleet/1"


@dataclass(frozen=True)
class VehicleSpec:
    index: int
    instance: int
    mav_sys_id: int
    namespace: str
    model: str
    gcs_port: int
    pose: str  # PX4_GZ_MODEL_POSE；空字符串表示不覆盖模型自带的离地高度
    spawn_xy_m: tuple[float, float]
    standalone: bool


def fleet_specs(n: int) -> list[VehicleSpec]:
    if isinstance(n, bool) or not isinstance(n, int):
        raise ValueError(f"n 必须是整数，收到 {n!r}")
    if n < 1 or n > MAX_N:
        raise ValueError(f"n 只能是 1..{MAX_N}，收到 {n}")
    if n == 1:
        # S1 单机：instance 0、无 namespace、MAV_SYS_ID 1、Gazebo 模型 x500_0。不改生成点。
        return [VehicleSpec(1, 0, 1, "", "x500_0", GCS_PORT_BASE, "", (0.0, 0.0), False)]
    specs = []
    for i in range(1, n + 1):
        y = (i - 1) * SPAWN_SPACING_M
        pose = "0,0" if y == 0 else f"0,{y:g}"
        specs.append(
            VehicleSpec(
                index=i,
                instance=i,
                mav_sys_id=i + 1,
                namespace=f"px4_{i}",
                model=f"x500_{i}",
                gcs_port=GCS_PORT_BASE + i,
                pose=pose,
                spawn_xy_m=(0.0, float(y)),
                standalone=i != 1,
            )
        )
    domains = {DDS_DOMAIN_ID}
    if len(domains) != 1:
        raise ValueError("多机必须在同一个 DDS 域")
    return specs


def layout_dict(n: int) -> dict:
    specs = fleet_specs(n)
    return {
        "n": n,
        "dds_domain_id": DDS_DOMAIN_ID,
        "xrce_port": XRCE_PORT,
        "takeoff_alt_m": TAKEOFF_ALT_M,
        "hover_s": HOVER_S,
        "report_only": True,
        "criteria_frozen": False,
        "vehicles": [
            {
                "index": s.index,
                "instance": s.instance,
                "mav_sys_id": s.mav_sys_id,
                "namespace": s.namespace,
                "model": s.model,
                "gcs_port": s.gcs_port,
                "pose": s.pose,
                "spawn_xy_m": list(s.spawn_xy_m),
                "standalone": s.standalone,
            }
            for s in specs
        ],
    }


def format_shell(n: int) -> str:
    data = layout_dict(n)
    lines = [
        f"n={data['n']}",
        f"dds_domain_id={data['dds_domain_id']}",
        f"xrce_port={data['xrce_port']}",
    ]
    for v in data["vehicles"]:
        lines.append("---")
        lines.append(f"instance={v['instance']}")
        lines.append(f"mav_sys_id={v['mav_sys_id']}")
        lines.append(f"namespace={v['namespace']}")
        lines.append(f"model={v['model']}")
        lines.append(f"gcs_port={v['gcs_port']}")
        lines.append(f"pose={v['pose']}")
        lines.append(f"standalone={1 if v['standalone'] else 0}")
    return "\n".join(lines) + "\n"


def _om():
    path = Path(__file__).with_name("offboard_mission.py")
    name = "yopo_sitl_offboard_mission"
    mod = sys.modules.get(name)
    if mod is None:
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
    return mod


def fleet_mission_config():
    om = _om()
    return om.MissionConfig(
        takeoff_alt_m=TAKEOFF_ALT_M,
        hover_s=HOVER_S,
        goal_offset_enu=(0.0, 0.0, 0.0),
        land_at_hover=True,
    )


class Fleet:
    """每机一台 Mission，共用仿真时钟。ok 只表示都落地并上锁，不看误差大小。"""

    def __init__(self, n: int, cfg=None):
        om = _om()
        self.specs = fleet_specs(n)
        self.cfg = cfg or fleet_mission_config()
        self.missions = [om.Mission(self.cfg) for _ in self.specs]

    @property
    def done(self) -> bool:
        return all(m.done for m in self.missions)

    def step(self, states: list):
        if len(states) != len(self.missions):
            raise ValueError(f"需要 {len(self.missions)} 架的状态，收到 {len(states)}")
        return [m.step(s) for m, s in zip(self.missions, states)]


def command_message(kind: str, timestamp_us: int, target_system: int) -> dict:
    """VehicleCommand 字段。target_system 用该机的 MAV_SYS_ID，不广播。"""
    import math

    from yopo_swarm_core import px4

    om = _om()
    if kind == "offboard":
        return px4.offboard_mode_command(timestamp_us, target_system)
    if kind == "arm":
        return px4.arm_command(timestamp_us, target_system)
    if kind == "land":
        return px4.vehicle_command(om.VEHICLE_CMD_NAV_LAND, timestamp_us, target_system, param4=math.nan)
    raise ValueError(f"未知命令 {kind}")


def ros_topics(spec: VehicleSpec) -> dict[str, str]:
    from yopo_swarm_core import px4

    ns = spec.namespace
    return {
        "offboard_control_mode": px4.topic("in", "offboard_control_mode", ns),
        "trajectory_setpoint": px4.topic("in", "trajectory_setpoint", ns),
        "vehicle_command": px4.topic("in", "vehicle_command", ns),
        "vehicle_odometry": px4.topic("out", "vehicle_odometry", ns),
        "vehicle_status": px4.topic("out", "vehicle_status", ns),
        "vehicle_land_detected": px4.topic("out", "vehicle_land_detected", ns),
    }


def result_payload(
    fleet: Fleet,
    *,
    positioning: str,
    wall_to_preflight_ok_s,
    preflight_checks_pass_seen: bool,
    mission_wall_s,
    mission_sim_s,
    traces: dict | None = None,
) -> dict:
    traces = traces or {}
    vehicles = []
    for spec, mission in zip(fleet.specs, fleet.missions):
        hover = None if mission.hover_sp is None else [float(x) for x in mission.hover_sp]
        vehicles.append(
            {
                "index": spec.index,
                "instance": spec.instance,
                "mav_sys_id": spec.mav_sys_id,
                "namespace": spec.namespace,
                "model": spec.model,
                "gcs_port": spec.gcs_port,
                "pose": spec.pose,
                "spawn_xy_m": list(spec.spawn_xy_m),
                "hover_enu": hover,
                "ok": mission.phase == "DONE",
                "phase": mission.phase,
                "metrics": dataclasses.asdict(mission.metrics),
                "home_enu": None if mission.home is None else mission.home.tolist(),
                "trace": traces.get(spec.instance, []),
            }
        )
    ok = bool(vehicles) and all(v["ok"] for v in vehicles)
    rtf = (mission_sim_s / mission_wall_s) if mission_wall_s and mission_sim_s is not None else None
    return {
        "schema": SCHEMA,
        "report_only": True,
        "criteria_frozen": False,
        "n": len(fleet.specs),
        "dds_domain_id": DDS_DOMAIN_ID,
        "xrce_port": XRCE_PORT,
        "positioning": positioning,
        "ok": ok,
        "phase": "DONE" if ok else "FAIL",
        "config": dataclasses.asdict(fleet.cfg),
        "vehicles": vehicles,
        "wall_to_preflight_ok_s": wall_to_preflight_ok_s,
        "preflight_checks_pass_seen": preflight_checks_pass_seen,
        "mission_wall_s": mission_wall_s,
        "mission_sim_s": mission_sim_s,
        "rtf": rtf,
    }


def _fail_open(fleet: Fleet, why: str) -> None:
    for mission in fleet.missions:
        if mission.done:
            continue
        if mission.metrics.failure is None:
            mission.metrics.failure = f"{mission.phase}: {why}"
        mission.phase = "FAIL"


def run_ros(n: int, out_path: str, wall_timeout: float, world: str, preflight_timeout: float) -> int:
    """容器内：订阅每机 uXRCE 话题和 Gazebo 真值，写出 result.json。"""
    import threading

    try:
        import numpy as np
        import rclpy
    except ImportError as exc:
        print(f"run 需要容器里的 ROS 2 / numpy（{exc}）", file=sys.stderr)
        return 2
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
    from pymavlink import mavutil
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    from yopo_swarm_core import px4

    om = _om()
    if px4.XRCE_DEFAULT_PORT != XRCE_PORT:
        print(f"XRCE 端口与 core 不一致：布局 {XRCE_PORT}，core {px4.XRCE_DEFAULT_PORT}", file=sys.stderr)
        return 2
    specs = fleet_specs(n)
    fleet = Fleet(n)
    positioning = os.environ.get("YOPO_POSITIONING", "default")

    lock = threading.Lock()
    sim = {"t": None, "truth": {s.model: None for s in specs}}
    stop = threading.Event()

    def gcs_heartbeat():
        conns = [
            mavutil.mavlink_connection(
                f"udpout:127.0.0.1:{s.gcs_port}", source_system=255, source_component=190
            )
            for s in specs
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

    def on_clock(msg):
        with lock:
            sim["t"] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_pose(msg):
        with lock:
            for p in msg.pose:
                if p.name in sim["truth"]:
                    sim["truth"][p.name] = np.array([p.position.x, p.position.y, p.position.z])

    gz = GzNode()
    if not gz.subscribe(Clock, f"/world/{world}/clock", on_clock):
        print("gz clock subscribe failed", file=sys.stderr)
        return 2
    if not gz.subscribe(Pose_V, f"/world/{world}/pose/info", on_pose):
        print("gz pose subscribe failed", file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node("yopo_s2_offboard")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def fill(msg, fields: dict):
        for k, v in fields.items():
            setattr(msg, k, v)
        return msg

    pubs = {}
    st = {}
    for spec in specs:
        topics = ros_topics(spec)
        st[spec.instance] = {"odom": None, "status": None, "land": None}
        pubs[spec.instance] = {
            "ocm": node.create_publisher(OffboardControlMode, topics["offboard_control_mode"], qos),
            "sp": node.create_publisher(TrajectorySetpoint, topics["trajectory_setpoint"], qos),
            "cmd": node.create_publisher(VehicleCommand, topics["vehicle_command"], qos),
        }
        slot = st[spec.instance]
        node.create_subscription(VehicleOdometry, topics["vehicle_odometry"], lambda m, s=slot: s.__setitem__("odom", m), qos)
        node.create_subscription(VehicleStatus, topics["vehicle_status"], lambda m, s=slot: s.__setitem__("status", m), qos)
        node.create_subscription(
            VehicleLandDetected, topics["vehicle_land_detected"], lambda m, s=slot: s.__setitem__("land", m), qos
        )

    def now_us() -> int:
        return node.get_clock().now().nanoseconds // 1000

    wall0 = time.monotonic()
    wall_ready = sim_ready = None
    traces = {s.instance: [] for s in specs}
    preflight_seen = False
    try:
        while not fleet.done:
            rclpy.spin_once(node, timeout_sec=0.02)
            wall = time.monotonic() - wall0
            if wall > wall_timeout:
                _fail_open(fleet, f"wall timeout {wall_timeout}s")
                break
            with lock:
                t_sim = sim["t"]
                truths = {k: (None if v is None else v.copy()) for k, v in sim["truth"].items()}
            missing = []
            if t_sim is None:
                missing.append("gz_clock")
            slots = []
            for spec in specs:
                slot = st[spec.instance]
                if slot["odom"] is None or slot["status"] is None or slot["land"] is None:
                    missing.append(spec.namespace or spec.model)
                slots.append(slot)
            if missing:
                if wall > preflight_timeout:
                    _fail_open(fleet, f"PREFLIGHT: no {','.join(missing)} after {preflight_timeout}s wall")
                    break
                continue
            armed_ok = all(slot["status"].pre_flight_checks_pass for slot in slots)
            preflight_seen = preflight_seen or armed_ok
            if wall_ready is None:
                if not armed_ok:
                    if wall > preflight_timeout:
                        _fail_open(fleet, f"PREFLIGHT: pre_flight_checks_pass=false after {preflight_timeout}s wall")
                        break
                    continue
                wall_ready, sim_ready = wall, t_sim
            states = []
            bad = False
            for spec, slot in zip(specs, slots):
                o, vs, ld = slot["odom"], slot["status"], slot["land"]
                try:
                    enu = px4.odometry_to_enu(o.position, o.q, o.velocity, o.timestamp, o.pose_frame, o.velocity_frame)
                except ValueError:
                    bad = True
                    break
                states.append(
                    om.VehicleState(
                        t=t_sim,
                        position=enu.position,
                        velocity=enu.velocity,
                        armed=vs.arming_state == om.ARMING_STATE_ARMED,
                        offboard=vs.nav_state == om.NAV_STATE_OFFBOARD,
                        landed=bool(ld.landed),
                        truth=truths.get(spec.model),
                    )
                )
            if bad:
                continue
            actions = fleet.step(states)
            ts = now_us()
            for spec, act, s in zip(specs, actions, states):
                pub = pubs[spec.instance]
                if act.heartbeat:
                    pub["ocm"].publish(fill(OffboardControlMode(), px4.offboard_control_mode(ts)))
                if act.setpoint_pva is not None:
                    pub["sp"].publish(
                        fill(TrajectorySetpoint(), px4.trajectory_setpoint(act.setpoint_pva, act.yaw_enu, ts))
                    )
                for c in act.commands:
                    pub["cmd"].publish(fill(VehicleCommand(), command_message(c, ts, spec.mav_sys_id)))
                tr = traces[spec.instance]
                if not tr or t_sim - tr[-1][0] >= 0.1:
                    truth = truths.get(spec.model)
                    tz = [None] * 3 if truth is None else [round(float(x), 4) for x in truth]
                    tr.append([round(t_sim, 3), fleet.missions[spec.index - 1].phase, *[round(float(x), 4) for x in s.position], *tz])
            time.sleep(0.02)
    finally:
        stop.set()
        wall_end = time.monotonic() - wall0
        with lock:
            t_end = sim["t"]
        node.destroy_node()
        rclpy.try_shutdown()

    mission_wall = None if wall_ready is None else wall_end - wall_ready
    mission_sim = None if sim_ready is None or t_end is None else t_end - sim_ready
    out = result_payload(
        fleet,
        positioning=positioning,
        wall_to_preflight_ok_s=wall_ready,
        preflight_checks_pass_seen=preflight_seen,
        mission_wall_s=mission_wall,
        mission_sim_s=mission_sim,
        traces=traces,
    )
    with open(out_path, "w") as f:
        json.dump(out, f, indent=1)
    brief = {k: v for k, v in out.items() if k != "vehicles"}
    brief["vehicles"] = [{k: v for k, v in veh.items() if k != "trace"} for veh in out["vehicles"]]
    print(json.dumps(brief, indent=1))
    return 0 if out["ok"] else 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="三机 PX4 SITL 布局 / 任务（report-only）")
    sub = ap.add_subparsers(dest="cmd")
    lay = sub.add_parser("layout", help="打印机群布局，不起飞")
    lay.add_argument("--n", type=int, required=True)
    lay.add_argument("--format", choices=("shell", "json"), default="json")
    run = sub.add_parser("run", help="容器内跑起飞到 1.5 m 悬停再降落")
    run.add_argument("--n", type=int, required=True)
    run.add_argument("--out", required=True)
    run.add_argument("--wall-timeout", type=float, default=900.0)
    run.add_argument("--world", default="default")
    run.add_argument("--preflight-timeout", type=float, default=240.0)
    # 兼容：`fleet.py --n 3 --format shell`（up.sh / in_container.sh 用这个）
    ap.add_argument("--n", type=int)
    ap.add_argument("--format", choices=("shell", "json"), default="json")
    args = ap.parse_args(argv)
    try:
        if args.cmd == "run":
            return run_ros(args.n, args.out, args.wall_timeout, args.world, args.preflight_timeout)
        n = args.n
        fmt = args.format
        if n is None:
            ap.error("需要 --n")
        if fmt == "shell":
            sys.stdout.write(format_shell(n))
        else:
            json.dump(layout_dict(n), sys.stdout, indent=1)
            sys.stdout.write("\n")
        return 0
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
