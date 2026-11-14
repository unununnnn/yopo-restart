#!/usr/bin/env bash
# 票 S4c：在已有的 S1/S1b 镜像上构建 YOPO L0 运行时分层（幂等）。
# 不调用 scripts/sitl/Dockerfile，不重编 PX4。基础镜像不在就退出，提示先跑 up.sh。
#   scripts/sitl/build_yopo_image.sh                 前台构建；镜像已存在直接退出 0
#   scripts/sitl/build_yopo_image.sh --bg            setsid 后台构建，PID 和日志写到 $SITL_STATE
#   scripts/sitl/build_yopo_image.sh --status        ready / building(PID) / missing
#   scripts/sitl/build_yopo_image.sh --print-image   只打印分层标签
# 标签：yopo-restart-sitl:yopo-<Dockerfile 与 Dockerfile.yopo 拼接后的 sha256 前 12 位>
set -euo pipefail
# shellcheck source=common.sh
. "$(dirname "$0")/common.sh"

YOPO_HASH="$(cat "$SITL_DIR/Dockerfile" "$SITL_DIR/Dockerfile.yopo" | sha256sum | cut -c1-12)"
SITL_YOPO_IMAGE="yopo-restart-sitl:yopo-${YOPO_HASH}"

PRINT=0
BG=0
STATUS=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --print-image) PRINT=1; shift ;;
    --bg) BG=1; shift ;;
    --status) STATUS=1; shift ;;
    -h|--help)
      echo "用法：$0 [--print-image|--status|--bg]"
      exit 0
      ;;
    *) echo "用法：$0 [--print-image|--status|--bg]" >&2; exit 2 ;;
  esac
done

if [[ "$PRINT" == 1 ]]; then
  printf '%s\n' "$SITL_YOPO_IMAGE"
  exit 0
fi

mkdir -p "$SITL_STATE"
PIDF="$SITL_STATE/yopo-build.pid"
LOG="$SITL_STATE/yopo-build-$YOPO_HASH.log"

yopo_image_exists() { "${DOCKER[@]}" image inspect "$SITL_YOPO_IMAGE" >/dev/null 2>&1; }

image_status() {
  if yopo_image_exists; then
    echo "ready $SITL_YOPO_IMAGE"
  elif pid_alive "$PIDF"; then
    echo "building pid=$(cat "$PIDF") log=$LOG"
  else
    echo "missing $SITL_YOPO_IMAGE"
  fi
}

require_base_image() {
  if image_exists; then
    return 0
  fi
  echo "基础镜像 $SITL_IMAGE 不存在：先跑 scripts/sitl/up.sh（本脚本不重编 PX4）" >&2
  exit 2
}

build_image() {
  if yopo_image_exists; then
    echo "ready $SITL_YOPO_IMAGE"
    return 0
  fi
  require_base_image
  local t0 t1 size
  t0=$(date +%s)
  echo "build $SITL_YOPO_IMAGE start $(date -Is) base=$SITL_IMAGE"
  DOCKER_BUILDKIT=1 nice -n "$SITL_NICE" "${DOCKER[@]}" build \
    --build-arg "BASE_IMAGE=$SITL_IMAGE" \
    -t "$SITL_YOPO_IMAGE" \
    -f "$SITL_DIR/Dockerfile.yopo" \
    "$SITL_DIR"
  t1=$(date +%s)
  size=$("${DOCKER[@]}" image inspect -f '{{.Size}}' "$SITL_YOPO_IMAGE")
  printf 'build_seconds\t%s\nimage\t%s\nbase\t%s\nimage_bytes\t%s\n' \
    "$((t1 - t0))" "$SITL_YOPO_IMAGE" "$SITL_IMAGE" "$size" \
    | tee "$SITL_STATE/yopo-build-$YOPO_HASH.tsv"
  echo "build done $(date -Is)"
}

if [[ "$STATUS" == 1 ]]; then
  image_status
  exit 0
fi

if [[ "$BG" == 1 ]]; then
  if yopo_image_exists; then echo "ready $SITL_YOPO_IMAGE"; exit 0; fi
  if pid_alive "$PIDF"; then echo "building pid=$(cat "$PIDF") log=$LOG"; exit 0; fi
  require_base_image
  setsid nohup "$0" >"$LOG" 2>&1 < /dev/null &
  printf '%s\n' "$!" > "$PIDF"
  echo "started pid=$! pidfile=$PIDF log=$LOG"
  exit 0
fi

build_image
exit 0
