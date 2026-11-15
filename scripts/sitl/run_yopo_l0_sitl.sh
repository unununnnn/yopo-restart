#!/usr/bin/env bash
# 票 S4b：一键把 YOPO L0 规划桥接进 PX4 SITL + Gazebo 容器。
# 只传场景、机数、种子和定位模式。不改安全参数，不调用 offboard_mission.py。
#   scripts/sitl/run_yopo_l0_sitl.sh --n 1
#   scripts/sitl/run_yopo_l0_sitl.sh --n 3 --positioning rtk
#   scripts/sitl/run_yopo_l0_sitl.sh --scene scenes/g2-pillar.json --n 1 --dry-run
# 1 机默认 scenes/g2-pillar.json（空 namespace）。3 机默认 scenes/swap-3-open.json（px4_1..px4_3）。
# 容器已经由 up.sh 拉起时，在里面补跑；否则自己起一趟仿真，任务段只跑 yopo_l0_live.py。
# 镜像：--image 优先，其次 YOPO_SITL_IMAGE，再其次本机已有的 YOPO 分层标签，否则 S1b 基础镜像。
# 真正 docker run 之前检查选中的标签在不在。--dry-run 不要求镜像存在。
set -euo pipefail
# shellcheck disable=SC1091
. "$(dirname "$0")/common.sh"

SCENE=""
N="1"
POSITIONING="${YOPO_POSITIONING:-default}"
POLICY="${YOPO_L0_POLICY:-upstream}"
SEED="0"
DRY=0
IMAGE_ARG=""
usage() {
  echo "用法：$0 [--scene 场景卡] [--n 1|2|3] [--policy upstream|formation|crossing|switch] [--positioning default|rtk] [--seed N] [--image 镜像] [--dry-run]"
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --scene) SCENE="${2:-}"; shift 2 ;;
    --scene=*) SCENE="${1#*=}"; shift ;;
    --n) N="${2:-}"; shift 2 ;;
    --n=*) N="${1#*=}"; shift ;;
    --positioning) POSITIONING="${2:-}"; shift 2 ;;
    --positioning=*) POSITIONING="${1#*=}"; shift ;;
    --policy) POLICY="${2:-}"; shift 2 ;;
    --policy=*) POLICY="${1#*=}"; shift ;;
    --seed) SEED="${2:-}"; shift 2 ;;
    --seed=*) SEED="${1#*=}"; shift ;;
    --image)
      IMAGE_ARG="${2:-}"
      if [[ -z "$IMAGE_ARG" || "$IMAGE_ARG" == --* ]]; then
        echo "--image 需要镜像标签" >&2
        exit 2
      fi
      shift 2
      ;;
    --image=*)
      IMAGE_ARG="${1#*=}"
      if [[ -z "$IMAGE_ARG" ]]; then
        echo "--image 需要镜像标签" >&2
        exit 2
      fi
      shift
      ;;
    --dry-run) DRY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
if [[ "$POSITIONING" != "default" && "$POSITIONING" != "rtk" ]]; then
  echo "未知 --positioning=$POSITIONING（只要 default 或 rtk）" >&2
  exit 2
fi
case "$POLICY" in
  upstream|formation|crossing|switch) ;;
  *)
  echo "未知 --policy=$POLICY（只要 upstream、formation、crossing 或 switch）" >&2
  exit 2
  ;;
esac
case "$N" in
  1|2|3) ;;
  *) echo "未知 --n=$N（只要 1、2 或 3）" >&2; exit 2 ;;
esac
if [[ -z "$SCENE" ]]; then
  if [[ "$N" == "3" ]]; then
    SCENE="scenes/swap-3-open.json"
  elif [[ "$N" == "1" ]]; then
    SCENE="scenes/g2-pillar.json"
  else
    echo "--n 2 需要显式 --scene" >&2
    exit 2
  fi
fi
SITL_YOPO_IMAGE="$(bash "$SITL_DIR/build_yopo_image.sh" --print-image)"
tag_exists() { "${DOCKER[@]}" image inspect "$1" >/dev/null 2>&1; }
choose_runtime_image() {
  if [[ -n "$IMAGE_ARG" ]]; then
    printf '%s\n' "$IMAGE_ARG"
    return 0
  fi
  if [[ -n "${YOPO_SITL_IMAGE:-}" ]]; then
    printf '%s\n' "$SITL_IMAGE"
    return 0
  fi
  if tag_exists "$SITL_YOPO_IMAGE"; then
    printf '%s\n' "$SITL_YOPO_IMAGE"
    return 0
  fi
  printf '%s\n' "$SITL_IMAGE"
}
RUNTIME_IMAGE="$(choose_runtime_image)"
if [[ "$DRY" == "1" ]]; then
  printf 'planner=yopo-l0\nscene=%s\nn=%s\npolicy=%s\npositioning=%s\nseed=%s\nimage=%s\n' \
    "$SCENE" "$N" "$POLICY" "$POSITIONING" "$SEED" "$RUNTIME_IMAGE"
  exit 0
fi
python3 "$SITL_DIR/fleet.py" layout --n "$N" --format json >/dev/null
if ! tag_exists "$RUNTIME_IMAGE"; then
  if [[ "$RUNTIME_IMAGE" == "$SITL_YOPO_IMAGE" ]]; then
    echo "分层镜像 $RUNTIME_IMAGE 不存在：先跑 scripts/sitl/build_yopo_image.sh" >&2
  elif [[ -z "$IMAGE_ARG" && -z "${YOPO_SITL_IMAGE:-}" ]]; then
    echo "镜像 $RUNTIME_IMAGE 不存在：先跑 scripts/sitl/up.sh" >&2
  else
    echo "镜像 $RUNTIME_IMAGE 不存在：先准备该标签（基础镜像用 scripts/sitl/up.sh，分层镜像用 scripts/sitl/build_yopo_image.sh）" >&2
  fi
  exit 2
fi

NAME="$(sitl_container_name "$N")"
others="$(sitl_running_names | grep -v "^${NAME}$" || true)"
if [[ -n "$others" ]]; then
  echo "已有仿真容器在跑（一次只起一趟）：$others" >&2
  exit 3
fi

CACHE="${YOPO_SWARM_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/yopo-swarm}"
cache_env=()
cache_mount=()
if [[ -d "$CACHE" ]]; then
  # 只读挂进容器。属主是宿主机用户，容器以 root 跑。
  cache_env=(-e YOPO_SWARM_CACHE=/yopo-cache)
  cache_mount=(-v "$CACHE:/yopo-cache:ro")
fi
# git 把「root 访问别人的仓库」当成 dubious ownership，rev-parse 失败。
# fetch_upstream 就以为上游没取到，去写 <sha>.partial，只读挂载上是 Errno 30。
# safe.directory=* 只放宽这次容器里的目录检查，不改缓存内容。
git_safe=(
  -e GIT_CONFIG_COUNT=1
  -e GIT_CONFIG_KEY_0=safe.directory
  -e GIT_CONFIG_VALUE_0='*'
)

chown_out() {
  # 容器里的 root 写出 /out。和 smoke.sh 一样，结束时把属主还给调用者。
  local run=$1
  if "${DOCKER[@]}" ps -q -f "name=^${NAME}$" | grep -q .; then
    "${DOCKER[@]}" exec "$NAME" chown -R "$(id -u):$(id -g)" /out >/dev/null 2>&1 || true
  elif [[ -n "$run" && -d "$run" ]]; then
    "${DOCKER[@]}" run --rm -v "$run:/out" "$RUNTIME_IMAGE" chown -R "$(id -u):$(id -g)" /out >/dev/null 2>&1 || true
  fi
}

if "${DOCKER[@]}" ps -q -f "name=^${NAME}$" | grep -q .; then
  stack="$(printf '%s\n' "$SITL_STATE/stack-n${N}.txt")"
  if [[ ! -f "$stack" ]]; then
    echo "容器 $NAME 在跑，但没有 $stack" >&2
    exit 3
  fi
  RUN="$(cat "$stack")"
  set +e
  "${DOCKER[@]}" exec \
    -e YOPO_SITL_N="$N" -e YOPO_SITL_PLANNER=yopo-l0 -e YOPO_L0_SCENE="$SCENE" \
    -e YOPO_L0_POLICY="$POLICY" \
    -e YOPO_L0_SEED="$SEED" -e YOPO_POSITIONING="$POSITIONING" \
    -e YOPO_SITL_WALL_TIMEOUT="${YOPO_SITL_WALL_TIMEOUT:-900}" \
    "${cache_env[@]}" \
    "${git_safe[@]}" \
    "$NAME" nice -n "$SITL_NICE" bash /repo/scripts/sitl/exec_mission.sh
  rc=$?
  set -e
  chown_out "$RUN"
  exit "$rc"
fi

RUN="$SITL_STATE/runs/$(date +%Y%m%dT%H%M%S)-yopo-n${N}"
mkdir -p "$RUN"
"${DOCKER[@]}" rm -f "$NAME" >/dev/null 2>&1 || true
shm=1g
[[ "$N" != "1" ]] && shm=2g
set +e
"${DOCKER[@]}" run --rm --name "$NAME" \
  -e ROS_DOMAIN_ID=0 -e YOPO_SITL_N="$N" -e YOPO_SITL_MISSION=1 \
  -e YOPO_SITL_PLANNER=yopo-l0 -e YOPO_L0_SCENE="$SCENE" -e YOPO_L0_POLICY="$POLICY" \
  -e YOPO_L0_SEED="$SEED" \
  -e YOPO_SITL_WALL_TIMEOUT="${YOPO_SITL_WALL_TIMEOUT:-900}" \
  -e YOPO_POSITIONING="$POSITIONING" \
  "${git_safe[@]}" \
  "${cache_env[@]}" \
  "${cache_mount[@]}" \
  -v "$REPO_ROOT:/repo:ro" -v "$RUN:/out" --shm-size "$shm" \
  "$RUNTIME_IMAGE" nice -n "$SITL_NICE" bash /repo/scripts/sitl/in_container.sh
rc=$?
set -e
chown_out "$RUN"
echo "run_dir $RUN rc=$rc"
exit "$rc"
