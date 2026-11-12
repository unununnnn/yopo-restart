#!/usr/bin/env bash
# 票 S1：headless 单机 PX4 SITL 冒烟（report-only，不进 scenes/suite.txt）。
#   scripts/sitl/smoke.sh          前台跑一次，打印结果表
#   scripts/sitl/smoke.sh --bg     setsid nohup 后台跑，打印 PID 文件与日志路径后立即返回
# 流程：起飞到 2.5 m → 悬停 10 s（仿真时间）→ 飞到悬停点 +(5, 3, 0) m（ENU）→ NAV_LAND。
# 输出：悬停高度误差（RMS / 最大）、到点误差、墙钟、RTF。镜像不存在时先跑 scripts/sitl/up.sh。
# 只开 1 个 PX4 实例，全部进程 nice 15；只用 PX4 官方默认参数。
set -euo pipefail
# shellcheck source=common.sh
. "$(dirname "$0")/common.sh"
mkdir -p "$SITL_STATE/runs"
PIDF="$SITL_STATE/smoke.pid"
NAME=yopo-restart-s1-sitl

if [[ "${1:-}" == "--bg" ]]; then
  if pid_alive "$PIDF"; then echo "running pid=$(cat "$PIDF")"; exit 0; fi
  LOG="$SITL_STATE/runs/smoke-$(date +%Y%m%dT%H%M%S).log"
  setsid nohup "$0" >"$LOG" 2>&1 < /dev/null &
  echo $! > "$PIDF"
  echo "started pid=$! pidfile=$PIDF log=$LOG"
  exit 0
fi

if ! image_exists; then
  echo "镜像 $SITL_IMAGE 不存在：先跑 scripts/sitl/up.sh（或 --bg）" >&2
  exit 2
fi
if "${DOCKER[@]}" ps -q -f "name=^${NAME}$" | grep -q .; then
  echo "已有一个 $NAME 在跑（只允许 1 个 PX4 实例）" >&2
  exit 3
fi
RUN="$SITL_STATE/runs/$(date +%Y%m%dT%H%M%S)"
mkdir -p "$RUN"
# ROS_DOMAIN_ID=0：XRCE Agent 不读 ROS 环境变量，PX4 实体落在 UXRCE_DDS_DOM_ID 的默认域 0；
# 不改这个参数，ROS 侧跟着用 0。容器走自己的 bridge 网络命名空间，不和宿主 ROS 混。
t0=$(date +%s.%N)
set +e
"${DOCKER[@]}" run --rm --name "$NAME" \
  -e ROS_DOMAIN_ID=0 -e YOPO_SITL_WALL_TIMEOUT="${YOPO_SITL_WALL_TIMEOUT:-900}" \
  -v "$REPO_ROOT:/repo:ro" -v "$RUN:/out" --shm-size 1g \
  "$SITL_IMAGE" nice -n "$SITL_NICE" bash /repo/scripts/sitl/in_container.sh 2>&1 | tee "$RUN/console.log"
rc=${PIPESTATUS[0]}
set -e
t1=$(date +%s.%N)
# 容器里以 root 写的文件交还给当前用户，方便之后读和清理
"${DOCKER[@]}" run --rm -v "$RUN:/out" "$SITL_IMAGE" chown -R "$(id -u):$(id -g)" /out >/dev/null 2>&1 || true
python3 "$SITL_DIR/summarize.py" "$RUN" --total-wall "$(python3 -c "print($t1 - $t0)")" --image "$SITL_IMAGE" | tee "$RUN/summary.md"
echo "run_dir $RUN rc=$rc"
exit "$rc"
