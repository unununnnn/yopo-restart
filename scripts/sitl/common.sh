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
