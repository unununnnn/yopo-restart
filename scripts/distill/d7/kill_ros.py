
import os, signal, sys
me = os.getpid()
keys = ("ego_planner_node","traj_server","pcl_render_node","poscmd_2_odom","odom_visualization","rosmaster","roscore","rosout","roslaunch","ros_record.py","teacher_record","rosbag")
for pid in os.listdir("/proc"):
    if not pid.isdigit() or int(pid) == me:
        continue
    try:
        cmd = open("/proc/%s/cmdline" % pid, "rb").read().replace(b"\x00", b" ").decode("utf-8", "replace")
    except Exception:
        continue
    if any(k in cmd for k in keys):
        try:
            os.kill(int(pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
