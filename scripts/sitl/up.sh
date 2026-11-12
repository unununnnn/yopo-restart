#!/usr/bin/env bash
# 票 S1：一键构建单机 PX4 SITL 镜像（幂等）。
#   scripts/sitl/up.sh          前台构建；镜像已存在直接退出 0
#   scripts/sitl/up.sh --bg     setsid nohup 后台构建，打印 PID 文件与日志路径后立即返回
#   scripts/sitl/up.sh --status 只报状态：ready / building(PID) / missing
# 构建用 nice 15、PX4 编译并行 4；不改任何 PX4 参数。
set -euo pipefail
# shellcheck source=common.sh
. "$(dirname "$0")/common.sh"
mkdir -p "$SITL_STATE"
PIDF="$SITL_STATE/build.pid"
LOG="$SITL_STATE/build-$SITL_HASH.log"

case "${1:-}" in
  --status)
    if image_exists; then echo "ready $SITL_IMAGE"
    elif pid_alive "$PIDF"; then echo "building pid=$(cat "$PIDF") log=$LOG"
    else echo "missing $SITL_IMAGE"; fi
    exit 0 ;;
  --bg)
    if image_exists; then echo "ready $SITL_IMAGE"; exit 0; fi
    if pid_alive "$PIDF"; then echo "building pid=$(cat "$PIDF") log=$LOG"; exit 0; fi
    setsid nohup "$0" >"$LOG" 2>&1 < /dev/null &
    echo $! > "$PIDF"
    echo "started pid=$! pidfile=$PIDF log=$LOG"
    exit 0 ;;
  "") ;;
  *) echo "用法：$0 [--bg|--status]" >&2; exit 2 ;;
esac

if image_exists; then echo "ready $SITL_IMAGE"; exit 0; fi
t0=$(date +%s)
echo "build $SITL_IMAGE start $(date -Is)"
DOCKER_BUILDKIT=1 nice -n "$SITL_NICE" "${DOCKER[@]}" build \
  --build-arg JOBS="$SITL_JOBS" --build-arg NICE="$SITL_NICE" \
  -t "$SITL_IMAGE" -f "$SITL_DIR/Dockerfile" "$SITL_DIR"
t1=$(date +%s)
size=$("${DOCKER[@]}" image inspect -f '{{.Size}}' "$SITL_IMAGE")
printf 'build_seconds\t%s\nimage\t%s\nimage_bytes\t%s\n' "$((t1 - t0))" "$SITL_IMAGE" "$size" \
  | tee "$SITL_STATE/build-$SITL_HASH.tsv"
echo "build done $(date -Is)"
