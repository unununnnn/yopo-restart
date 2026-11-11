#!/usr/bin/env bash
# 票 E-C1：在本仓库之外编译并跑 ego-planner-swarm（README 提交 92fe9f7）。
# 不把上游源码写进本仓库，不加子模块。
#
# 记录见 docs/research/ego-swarm-baseline.md。
# 宿主机需要能跑 Docker。镜像是 osrf/ros:noetic-desktop-focal，不是 README 写的桌面全量包。
set -euo pipefail

COMMIT=92fe9f7227b2da819133eb8e0e8c7fc000f6ae20
IMAGE=osrf/ros:noetic-desktop-focal
CONTAINER=ego-noetic
DEST="${EGO_SWARM_DIR:-$HOME/ext/ego-planner-swarm}"
SECONDS_RUN=120
DO_BUILD=0
DO_RUN=0
REBUILD=0
DRY_RUN=0

usage() {
  cat <<'EOF'
在仓库外编译并运行 ZJU-FAST-Lab/ego-planner-swarm，提交 92fe9f7。

用法:
  scripts/ext/ego_swarm_baseline.sh [build|run|all] [选项]

子命令:
  build   克隆（若还没有）并在容器里 catkin_make -j1
  run     无头跑 ego_planner swarm.launch，并把 total time 与里程计统计打到标准输出
  all     先 build 再 run（默认）

选项:
  --dir PATH        克隆目录，默认 $HOME/ext/ego-planner-swarm 或环境变量 EGO_SWARM_DIR
  --seconds N       run 在 roslaunch 起来之后等待的秒数，默认 120
  --rebuild         即使已经编译过也再 catkin_make 一次
  --dry-run         只打印将要执行的步骤，不克隆、不编译、不启动
  -h, --help        显示这段说明

示例:
  scripts/ext/ego_swarm_baseline.sh --help
  scripts/ext/ego_swarm_baseline.sh build --dir "$HOME/ext/ego-planner-swarm"
  scripts/ext/ego_swarm_baseline.sh run --seconds 120
  scripts/ext/ego_swarm_baseline.sh all --dry-run

Docker 守护进程没在跑时，脚本退出并给出错误。这台记录用的虚拟机没有 systemd，
当时是手工执行 dockerd。脚本不会自己启动 dockerd。
EOF
}

die() {
  echo "错误: $*" >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    build|run|all)
      if [[ "$1" == "build" || "$1" == "all" ]]; then DO_BUILD=1; fi
      if [[ "$1" == "run" || "$1" == "all" ]]; then DO_RUN=1; fi
      shift
      ;;
    --dir)
      [[ $# -ge 2 ]] || die "缺少 --dir 的路径。示例: scripts/ext/ego_swarm_baseline.sh build --dir \$HOME/ext/ego-planner-swarm"
      DEST=$2
      shift 2
      ;;
    --seconds)
      [[ $# -ge 2 ]] || die "缺少 --seconds 的整数。示例: scripts/ext/ego_swarm_baseline.sh run --seconds 120"
      SECONDS_RUN=$2
      shift 2
      ;;
    --rebuild) REBUILD=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *)
      die "无法识别的参数: $1
  scripts/ext/ego_swarm_baseline.sh --help"
      ;;
  esac
done

if [[ "$DO_BUILD" == 0 && "$DO_RUN" == 0 ]]; then
  DO_BUILD=1
  DO_RUN=1
fi

if ! [[ "$SECONDS_RUN" =~ ^[0-9]+$ ]]; then
  die "--seconds 需要非负整数，得到: $SECONDS_RUN"
fi

if [[ "$DRY_RUN" == 1 ]]; then
  echo "dry-run"
  echo "commit: $COMMIT"
  echo "image: $IMAGE"
  echo "dir: $DEST"
  echo "build: $DO_BUILD rebuild: $REBUILD"
  echo "run: $DO_RUN seconds: $SECONDS_RUN"
  echo "container: $CONTAINER network=host"
  echo "launch: roslaunch ego_planner swarm.launch"
  exit 0
fi

if ! command -v docker >/dev/null 2>&1; then
  die "找不到 docker。示例: 先安装 docker.io，再 scripts/ext/ego_swarm_baseline.sh build"
fi

docker_cmd() {
  if docker info >/dev/null 2>&1 </dev/null; then
    docker "$@"
  elif sudo docker info >/dev/null 2>&1 </dev/null; then
    sudo docker "$@"
  else
    die "连不上 Docker 守护进程。本脚本不启动 dockerd。
  有 systemd 时: sudo systemctl start docker
  没有 systemd 时，记录里用的是: sudo dockerd --host=unix:///var/run/docker.sock --iptables=false --storage-driver=overlay2
  然后: scripts/ext/ego_swarm_baseline.sh build"
  fi
}

ensure_clone() {
  if [[ -d "$DEST/.git" ]]; then
    local head
    head=$(git -C "$DEST" rev-parse HEAD)
    if [[ "$head" != "$COMMIT" ]]; then
      die "目录 $DEST 的 HEAD 是 $head，不是 $COMMIT。换一个 --dir，或自己 checkout 到该提交。脚本不会改一个已经存在的其它提交。"
    fi
    echo "已有克隆: $DEST @ $COMMIT"
    return
  fi
  mkdir -p "$(dirname "$DEST")"
  git clone https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git "$DEST"
  git -C "$DEST" checkout "$COMMIT"
}

ensure_container() {
  if docker_cmd inspect "$CONTAINER" >/dev/null 2>&1; then
    docker_cmd start "$CONTAINER" >/dev/null
  else
    docker_cmd pull "$IMAGE"
    docker_cmd run -d --name "$CONTAINER" --network=host \
      -v "$DEST":/ego -w /ego "$IMAGE" sleep infinity >/dev/null
  fi
}

ensure_deps() {
  docker_cmd exec "$CONTAINER" bash -lc '
    set -e
    export DEBIAN_FRONTEND=noninteractive
    missing=0
    for p in libarmadillo-dev ros-noetic-pcl-ros ros-noetic-pcl-conversions \
             ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf \
             ros-noetic-laser-geometry ros-noetic-nodelet ros-noetic-cmake-modules \
             libpcl-dev libeigen3-dev; do
      dpkg -s "$p" >/dev/null 2>&1 || missing=1
    done
    if [[ "$missing" == 1 ]]; then
      apt-get update -qq
      apt-get install -y -qq libarmadillo-dev ros-noetic-pcl-ros ros-noetic-pcl-conversions \
        ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf \
        ros-noetic-laser-geometry ros-noetic-nodelet ros-noetic-cmake-modules \
        libpcl-dev libeigen3-dev
    fi
  '
}

do_build() {
  ensure_clone
  ensure_container
  ensure_deps
  if [[ "$REBUILD" == 0 && -f "$DEST/devel/setup.bash" ]]; then
    echo "已有 $DEST/devel/setup.bash，跳过编译。要重编加 --rebuild。"
    return
  fi
  docker_cmd exec "$CONTAINER" bash -lc '
    set +u
    set -e
    source /opt/ros/noetic/setup.bash
    cd /ego
    if ! catkin_make -j1; then
      echo "第一次 catkin_make -j1 失败，先生成 multi_map_server 的 C++ 头再编一次。"
      catkin_make -j1 multi_map_server_generate_messages_cpp
      catkin_make -j1
    fi
  '
  echo "编译结束: $DEST"
}

write_harness() {
  local host_dir
  host_dir=$(mktemp -d)
  cat >"$host_dir/linebuf.c" <<'EOF'
#include <stdio.h>
__attribute__((constructor)) static void ego_linebuf(void) {
  setvbuf(stdout, NULL, _IOLBF, 0);
  setvbuf(stderr, NULL, _IOLBF, 0);
}
EOF
  cat >"$host_dir/ego_odom_watch.py" <<'PY'
#!/usr/bin/env python3
import json, math, time
import rospy
from nav_msgs.msg import Odometry

N = 10
INIT = [(-20.0, -9.0 + 2.0 * i, 0.1) for i in range(N)]
TARGET = [(20.0, 9.0 - 2.0 * i, 1.0) for i in range(N)]
latest = {}
stats = {
    "samples": 0,
    "flight_samples": 0,
    "min_pair_m": None,
    "min_pair_ids": None,
    "min_pair_ros_time": None,
    "min_pair_flight_m": None,
    "min_pair_flight_ids": None,
    "min_pair_flight_ros_time": None,
    "min_goal_m": [None] * N,
    "final": [None] * N,
    "max_disp_m": [0.0] * N,
}

def dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

def cb(i, msg):
    p = msg.pose.pose.position
    latest[i] = (p.x, p.y, p.z, msg.header.stamp.to_sec(), time.time())

def dump():
    with open("/tmp/ego-odom-stats.json", "w") as f:
        json.dump(stats, f)

def main():
    rospy.init_node("ego_odom_watch", anonymous=True)
    for i in range(N):
        rospy.Subscriber("/drone_%d_visual_slam/odom" % i, Odometry, lambda m, i=i: cb(i, m), queue_size=10)
    rate = rospy.Rate(20)
    t_print = time.time()
    while not rospy.is_shutdown():
        now = time.time()
        if len(latest) == N and max(now - latest[i][4] for i in range(N)) <= 0.2:
            pts = [latest[i][:3] for i in range(N)]
            stamps = [latest[i][3] for i in range(N)]
            stats["samples"] += 1
            moved = False
            for i in range(N):
                dg = dist(pts[i], TARGET[i])
                if stats["min_goal_m"][i] is None or dg < stats["min_goal_m"][i]:
                    stats["min_goal_m"][i] = dg
                dd = dist(pts[i], INIT[i])
                if dd > stats["max_disp_m"][i]:
                    stats["max_disp_m"][i] = dd
                if dd > 1.0:
                    moved = True
                stats["final"][i] = list(pts[i])
            best, best_ids = None, None
            for a in range(N):
                for b in range(a + 1, N):
                    d = dist(pts[a], pts[b])
                    if best is None or d < best:
                        best, best_ids = d, [a, b]
            tmid = sum(stamps) / N
            if stats["min_pair_m"] is None or best < stats["min_pair_m"]:
                stats["min_pair_m"] = best
                stats["min_pair_ids"] = best_ids
                stats["min_pair_ros_time"] = tmid
            if moved:
                stats["flight_samples"] += 1
                if stats["min_pair_flight_m"] is None or best < stats["min_pair_flight_m"]:
                    stats["min_pair_flight_m"] = best
                    stats["min_pair_flight_ids"] = best_ids
                    stats["min_pair_flight_ros_time"] = tmid
        if now - t_print > 5.0:
            t_print = now
            dump()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    finally:
        dump()
PY
  docker_cmd cp "$host_dir/linebuf.c" "$CONTAINER":/tmp/linebuf.c
  docker_cmd cp "$host_dir/ego_odom_watch.py" "$CONTAINER":/tmp/ego_odom_watch.py
  rm -rf "$host_dir"
  docker_cmd exec "$CONTAINER" gcc -shared -fPIC -o /tmp/linebuf.so /tmp/linebuf.c
}

summarize_in_container() {
  docker_cmd exec -i "$CONTAINER" python3 - <<'PY'
import math, re
from decimal import Decimal
from pathlib import Path
log = Path("/tmp/ego-swarm.log").read_text(errors="replace")
raws = re.findall(r"total time:\x1b\[42m([0-9.eE+-]+)", log)
print("total_time_count", len(raws))
if raws:
    vals = [Decimal(s) for s in raws]
    n = len(vals)
    total = sum(vals, Decimal(0))
    ordered = sorted(vals)
    idx = math.ceil(0.95 * n) - 1
    print("total_time_sum_s", total)
    print("total_time_mean_s", total / n)
    print("total_time_p95_s", ordered[idx])
    print("total_time_min_raw", min(raws, key=Decimal))
    print("total_time_max_raw", max(raws, key=Decimal))
print("plan_success_0", log.count("plan_success=0"))
print("plan_success_1", log.count("plan_success=1"))
print("close_to_goal", log.count("Close to goal"))
print("emergency_stop_line", log.count("Suddenly discovered obstacles. emergency stop!"))
print("depth_lost", log.count("Depth Lost!"))
print("to_emergency", len(re.findall(r"to EMERGENCY_STOP", log)))
stats = Path("/tmp/ego-odom-stats.json")
if stats.exists():
    print("odom_json", stats.read_text())
PY
}

do_run() {
  [[ -f "$DEST/devel/setup.bash" ]] || die "还没有编译。先运行: scripts/ext/ego_swarm_baseline.sh build --dir $DEST"
  ensure_container
  write_harness
  # 清掉上一次留下的节点，避免和这次的 rosmaster 抢 11311。
  docker_cmd exec "$CONTAINER" python3 -c '
import os, signal
keys = ("ego_planner_node","traj_server","random_forest","pcl_render_node","poscmd_2_odom","odom_visualization","rosmaster","rosout","roslaunch","ego_odom_watch")
for pid in os.listdir("/proc"):
    if not pid.isdigit() or int(pid) == os.getpid():
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
'
  docker_cmd exec "$CONTAINER" bash -lc "
    set +u
    source /opt/ros/noetic/setup.bash
    source /ego/devel/setup.bash
    export LD_PRELOAD=/tmp/linebuf.so
    rm -f /tmp/ego-swarm.log /tmp/ego-odom-stats.json
    python3 -u /tmp/ego_odom_watch.py >/tmp/ego-watch.log 2>&1 &
    watch_pid=\$!
    roslaunch ego_planner swarm.launch >/tmp/ego-swarm.log 2>&1 &
    launch_pid=\$!
    sleep ${SECONDS_RUN}
    rosnode list >/tmp/ego-rosnode.txt 2>&1 || true
    kill -INT \$watch_pid 2>/dev/null || true
    kill -INT \$launch_pid 2>/dev/null || true
    for i in 1 2 3 4 5 6 7 8 9 10 11 12; do
      kill -0 \$launch_pid 2>/dev/null || break
      sleep 1
    done
    kill -TERM \$launch_pid 2>/dev/null || true
    kill -TERM \$watch_pid 2>/dev/null || true
    echo nodes:
    cat /tmp/ego-rosnode.txt
  "
  summarize_in_container
}

if [[ "$DO_BUILD" == 1 ]]; then
  do_build
fi
if [[ "$DO_RUN" == 1 ]]; then
  do_run
fi
