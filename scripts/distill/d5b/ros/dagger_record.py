#!/usr/bin/env python3
"""在 ego-planner 容器里发布圆柱点云，并记下里程计和广播 B 样条。

只依赖 ROS 和标准库。几何来自场景 JSON，不读本仓库的 numpy 代码。
加速度取自 odom.twist.angular：该提交的 fake_drone 把指令加速度写在那里。
偏航取自 PositionCommand.yaw；还没有指令时记 0。
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import time
from pathlib import Path


def cylinder_points(cx, cy, radius, z_lo, z_hi, resolution):
    pts = []
    n = int(math.ceil(radius / resolution))
    r2 = radius * radius
    z0 = int(math.floor(z_lo / resolution))
    z1 = int(math.ceil(z_hi / resolution))
    for ix in range(-n, n + 1):
        for iy in range(-n, n + 1):
            x = cx + ix * resolution
            y = cy + iy * resolution
            if (x - cx) ** 2 + (y - cy) ** 2 > r2:
                continue
            for iz in range(z0, z1 + 1):
                z = iz * resolution
                if z < z_lo - 1e-9 or z > z_hi + 1e-9:
                    continue
                pts.append((x, y, z))
    return pts


def cloud_points(scenario, resolution, sentinel):
    pts = []
    for cyl in scenario["cylinders"]:
        pts.extend(
            cylinder_points(
                cyl["center_xy"][0],
                cyl["center_xy"][1],
                cyl["radius"],
                cyl["z_lo"],
                cyl["z_hi"],
                resolution,
            )
        )
    if not pts:
        pts.append(tuple(sentinel))
    return pts


def pack_cloud(points, frame_id, stamp):
    from sensor_msgs.msg import PointCloud2, PointField

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    buf = bytearray()
    for x, y, z in points:
        buf += struct.pack("<fff", float(x), float(y), float(z))
    msg.data = bytes(buf)
    return msg


def yaw_of_quat(x, y, z, w):
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def main():
    parser = argparse.ArgumentParser(description="记录一次教师飞行")
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--timeout", type=float, default=50.0)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--goal-radius", type=float, default=0.5)
    parser.add_argument("--goal-speed", type=float, default=0.3)
    parser.add_argument("--goal-hold", type=float, default=1.0)
    args = parser.parse_args()

    import rospy
    from nav_msgs.msg import Odometry
    from quadrotor_msgs.msg import PositionCommand
    from sensor_msgs.msg import PointCloud2

    scenario = json.loads(Path(args.scenario).read_text(encoding="utf-8"))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    drones = scenario["drones"]
    latest = {}
    yaw_cmd = {}
    bsplines = []
    bspline_import = False
    try:
        from traj_utils.msg import Bspline

        bspline_import = True
    except Exception:
        Bspline = None

    rospy.init_node("teacher_record", anonymous=False)
    points = cloud_points(scenario, 0.1, (0.0, 0.0, -10.0))
    pub = rospy.Publisher("/map_generator/global_cloud", PointCloud2, queue_size=1, latch=True)
    cloud = pack_cloud(points, "world", rospy.Time.now())
    pub.publish(cloud)

    def on_odom(ros_id, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        a = msg.twist.twist.angular
        q = msg.pose.pose.orientation
        latest[ros_id] = {
            "p": [p.x, p.y, p.z],
            "v": [v.x, v.y, v.z],
            "a": [a.x, a.y, a.z],
            "quat_yaw": yaw_of_quat(q.x, q.y, q.z, q.w),
            "stamp": msg.header.stamp.to_sec(),
            "wall": time.time(),
        }

    SEED_ID = 987654321
    handover = {}
    def on_cmd(ros_id, msg):
        yaw_cmd[ros_id] = float(msg.yaw)
        if int(msg.trajectory_id) != SEED_ID and ros_id not in handover:
            handover[ros_id] = rospy.Time.now().to_sec()

    for d in drones:
        rid = int(d["ros_id"])
        rospy.Subscriber(
            "/drone_%d_visual_slam/odom" % rid,
            Odometry,
            lambda m, rid=rid: on_odom(rid, m),
            queue_size=10,
        )
        rospy.Subscriber(
            "/drone_%d_planning/pos_cmd" % rid,
            PositionCommand,
            lambda m, rid=rid: on_cmd(rid, m),
            queue_size=10,
        )
        if Bspline is not None:
            rospy.Subscriber(
                "/drone_%d_planning/bspline" % rid,
                Bspline,
                lambda m, rid=rid: on_bspline("local", m),
                queue_size=50,
            )
    if Bspline is not None:
        rospy.Subscriber("/broadcast_bspline", Bspline, lambda m: on_bspline("broadcast", m), queue_size=100)

    def on_bspline(topic, msg):
        bsplines.append(
            {
                "topic": topic,
                "drone_id": int(msg.drone_id),
                "order": int(msg.order),
                "traj_id": int(msg.traj_id),
                "start_time_ros_s": float(msg.start_time.to_sec()),
                "receive_ros_s": float(rospy.Time.now().to_sec()),
                "knots": [float(k) for k in msg.knots],
                "pos_pts": [[float(p.x), float(p.y), float(p.z)] for p in msg.pos_pts],
                "yaw_pts": [float(y) for y in msg.yaw_pts],
                "yaw_dt": float(msg.yaw_dt),
            }
        )

    seed_pubs = {}
    for d in drones:
        if d.get("seed_state") is not None:
            seed_pubs[int(d["ros_id"])] = (rospy.Publisher("/drone_%d_planning/pos_cmd" % int(d["ros_id"]), PositionCommand, queue_size=5), d["seed_state"])
    def publish_seeds():
        for rid, (pb, st) in seed_pubs.items():
            if rid in handover:
                continue
            m = PositionCommand()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "world"
            m.position.x, m.position.y, m.position.z = (float(x) for x in st["p"])
            m.velocity.x, m.velocity.y, m.velocity.z = (float(x) for x in st["v"])
            m.acceleration.x, m.acceleration.y, m.acceleration.z = (float(x) for x in st["a"])
            m.yaw = float(st["yaw"])
            m.trajectory_id = SEED_ID
            pb.publish(m)
    seed_timer = rospy.Timer(rospy.Duration(0.02), lambda _e: publish_seeds()) if seed_pubs else None
    after_s = float(scenario.get("record_after_handover_s", 1e9))
    time_s = []
    position = []
    velocity = []
    acceleration = []
    yaw = []
    state = {"t0_ros": None, "status": "timeout", "wall0": time.time()}
    hold_since = None
    rate = rospy.Rate(args.sample_hz)
    deadline = state["wall0"] + args.timeout

    def save():
        payload = {
            "run_status": state["status"] if time_s else "no_odom",
            "wall_s": time.time() - state["wall0"],
            "t0_ros_s": state["t0_ros"],
            "sample_hz": args.sample_hz,
            "bspline_import": bspline_import,
            "yaw_source": "PositionCommand.yaw；没有指令时为 0",
            "acceleration_source": "odom.twist.angular（fake_drone 写入的指令加速度）",
            "n_cloud_points": len(points),
            "time_s": time_s,
            "position_m": position,
            "velocity_mps": velocity,
            "acceleration_mps2": acceleration,
            "yaw_rad": yaw,
            "bsplines": bsplines,
            "handover_ros_s": {str(k): v for k, v in handover.items()},
            "seed_trajectory_id": SEED_ID,
        }
        if not time_s and state["status"] == "timeout":
            payload["run_status"] = "no_odom"
        (out / "raw_record.json").write_text(json.dumps(payload), encoding="utf-8")
        (out / "done.json").write_text(json.dumps({"status": payload["run_status"]}), encoding="utf-8")

    try:
        while not rospy.is_shutdown() and time.time() < deadline:
            now_wall = time.time()
            fresh = len(latest) == len(drones) and max(
                now_wall - latest[int(d["ros_id"])]["wall"] for d in drones
            ) <= 0.3
            if fresh:
                ros_now = rospy.Time.now().to_sec()
                if state["t0_ros"] is None:
                    state["t0_ros"] = ros_now
                row_p, row_v, row_a, row_y = [], [], [], []
                all_in = True
                for d in drones:
                    item = latest[int(d["ros_id"])]
                    row_p.append(item["p"])
                    row_v.append(item["v"])
                    row_a.append(item["a"])
                    if int(d["ros_id"]) in yaw_cmd:
                        row_y.append(yaw_cmd[int(d["ros_id"])])
                    else:
                        row_y.append(0.0)
                    dist = math.sqrt(
                        (item["p"][0] - d["goal_enu"][0]) ** 2
                        + (item["p"][1] - d["goal_enu"][1]) ** 2
                        + (item["p"][2] - d["goal_enu"][2]) ** 2
                    )
                    speed = math.sqrt(item["v"][0] ** 2 + item["v"][1] ** 2 + item["v"][2] ** 2)
                    if dist > args.goal_radius or speed > args.goal_speed:
                        all_in = False
                time_s.append(ros_now - state["t0_ros"])
                position.append(row_p)
                velocity.append(row_v)
                acceleration.append(row_a)
                yaw.append(row_y)
                if seed_pubs and len(handover) == len(drones) and ros_now - max(handover.values()) >= after_s:
                    state["status"] = "window_done"
                    break
                if all_in:
                    if hold_since is None:
                        hold_since = now_wall
                    elif now_wall - hold_since >= args.goal_hold:
                        state["status"] = "arrived"
                        break
                else:
                    hold_since = None
            rate.sleep()
    except KeyboardInterrupt:
        if state["status"] == "timeout":
            state["status"] = "stopped"
    finally:
        save()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
