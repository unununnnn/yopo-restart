#!/usr/bin/env bash
# 容器内入口（由 smoke.sh 以 nice 15 调起）：XRCE Agent + 单个 PX4 SITL 实例（gz_x500，headless）+ 任务驱动。
# PX4 参数：全新空状态目录 → 只用官方 airframe 默认值；本脚本不 param set、不设 PX4_PARAM_*。
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/ros_ws/install/setup.bash
BUILD=/opt/PX4-Autopilot/build/px4_sitl_default
source "$BUILD/rootfs/gz_env.sh"
set -u
OUT=/out
mkdir -p "$OUT/px4-state"
export HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=default
export PYTHONPATH=/repo/core/src${PYTHONPATH:+:$PYTHONPATH}

cleanup() { kill $(jobs -pr) 2>/dev/null || true; pkill -f 'gz sim' 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT TERM INT

MicroXRCEAgent udp4 -p 8888 > "$OUT/agent.log" 2>&1 &
( cd "$OUT/px4-state" && exec "$BUILD/bin/px4" -d "$BUILD/etc" ) > "$OUT/px4.log" 2>&1 &
PX4_PID=$!
for _ in $(seq 1 1200); do
  grep -q 'Startup script returned successfully' "$OUT/px4.log" 2>/dev/null && break
  kill -0 "$PX4_PID" 2>/dev/null || { echo "PX4 进程提前退出，见 px4.log" >&2; exit 3; }
  sleep 0.1
done
echo "px4_startup_wall_s $SECONDS"
# 只读：记下与默认值不同的参数（应为空或只有 airframe/SITL 自己设的），作为没改参数的证据。
"$BUILD/bin/px4-param" --instance 0 show -c > "$OUT/px4-params-changed.txt" 2>&1 || true
set +e
python3 /repo/scripts/sitl/offboard_mission.py --out "$OUT/result.json" --wall-timeout "${YOPO_SITL_WALL_TIMEOUT:-900}"
rc=$?
set -e
# 卡住时最有用的几行：PX4 自己报的预检 / 解锁失败原因（去重）
grep -E 'Preflight Fail|Arming denied|arming denied|Failsafe|failsafe' "$OUT/px4.log" | sort | uniq -c | sort -rn | head -10 || true
exit $rc
