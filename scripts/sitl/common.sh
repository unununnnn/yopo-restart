# scripts/sitl 公共变量；被 up.sh / smoke.sh source。
SITL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SITL_DIR/../.." && pwd)"
# 镜像标签随 Dockerfile 内容变：内容不变就复用，变了就重建（幂等）。
SITL_HASH="$(sha256sum "$SITL_DIR/Dockerfile" | cut -c1-12)"
SITL_IMAGE="${YOPO_SITL_IMAGE:-yopo-restart-sitl:s1b-$SITL_HASH}"
# box 共享机器：状态与日志放 /workspace/yopo-ops/sitl（不进仓库，不进 /tmp）。
SITL_STATE="${YOPO_SITL_STATE:-/workspace/yopo-ops/sitl}"
SITL_NICE="${YOPO_SITL_NICE:-15}"
SITL_JOBS="${YOPO_SITL_JOBS:-4}"
if docker info >/dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo -n docker)
fi
image_exists() { "${DOCKER[@]}" image inspect "$SITL_IMAGE" >/dev/null 2>&1; }
pid_alive() { [[ -f "$1" ]] && kill -0 "$(cat "$1")" 2>/dev/null; }
# n=1 沿用 S1 容器名。n>1 是票 S2 的一趟仿真（容器里有 n 个 PX4 进程）。
# YOPO_SITL_CONTAINER_NAME（缺省空）：给这一趟换个容器名，和别人的仿真并排跑（各自独立网络命名空间）。
sitl_container_name() {
  if [[ -n "${YOPO_SITL_CONTAINER_NAME:-}" ]]; then
    printf '%s\n' "$YOPO_SITL_CONTAINER_NAME"
  elif [[ "$1" == "1" ]]; then
    printf '%s\n' "yopo-restart-s1-sitl"
  else
    printf '%s\n' "yopo-restart-sitl-n$1"
  fi
}
sitl_running_names() {
  "${DOCKER[@]}" ps --format '{{.Names}}' 2>/dev/null \
    | grep -E '^yopo-restart-s(1-sitl|itl-n[0-9]+)$' || true
}

# 票 SITL-NET：缺省为这一趟建隔离 docker 网络（前缀 yopo-sitl-），
# 避免和别的仿真在默认网桥上串话（DDS / gz-transport 组播）。
# YOPO_SITL_DOCKER_NETWORK 已给则用用户的网络，不创建、不删除。
# 自定容器名却没给网络：调用方应直接退出，这里不建网。
SITL_NET_PREFIX="yopo-sitl-"
SITL_RUN_NETWORK=""
SITL_RUN_NETWORK_OWNED=0

sitl_custom_name_needs_network() {
  if [[ -n "${YOPO_SITL_CONTAINER_NAME:-}" && -z "${YOPO_SITL_DOCKER_NETWORK:-}" ]]; then
    echo "YOPO_SITL_CONTAINER_NAME 要和 YOPO_SITL_DOCKER_NETWORK 一起给：默认网桥上会和别的仿真串话" >&2
    return 2
  fi
  return 0
}

sitl_prepare_run_network() {
  SITL_RUN_NETWORK=""
  SITL_RUN_NETWORK_OWNED=0
  if [[ -n "${YOPO_SITL_DOCKER_NETWORK:-}" ]]; then
    SITL_RUN_NETWORK="$YOPO_SITL_DOCKER_NETWORK"
    return 0
  fi
  local i name
  for i in 1 2 3; do
    name="${SITL_NET_PREFIX}$(date +%Y%m%dT%H%M%S)-$$-${RANDOM}"
    if "${DOCKER[@]}" network create --driver bridge "$name" >/dev/null; then
      SITL_RUN_NETWORK="$name"
      SITL_RUN_NETWORK_OWNED=1
      return 0
    fi
  done
  echo "无法创建隔离网络 ${SITL_NET_PREFIX}<id>" >&2
  return 3
}

sitl_cleanup_run_network() {
  local name="${SITL_RUN_NETWORK:-}" owned="${SITL_RUN_NETWORK_OWNED:-0}"
  SITL_RUN_NETWORK_OWNED=0
  if [[ "$owned" != 1 || -z "$name" ]]; then
    return 0
  fi
  # 只删本趟自建、且带 yopo-sitl- 前缀的网络。不删用户指定的，也不扫别人的。
  case "$name" in
    "${SITL_NET_PREFIX}"*) ;;
    *) return 0 ;;
  esac
  local i
  for i in 1 2 3 4 5; do
    if "${DOCKER[@]}" network rm "$name" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  if [[ -n "${NAME:-}" ]]; then
    "${DOCKER[@]}" network disconnect -f "$name" "$NAME" >/dev/null 2>&1 || true
  fi
  "${DOCKER[@]}" network rm "$name" >/dev/null 2>&1 || true
  return 0
}
