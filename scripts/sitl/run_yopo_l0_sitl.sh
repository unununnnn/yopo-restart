#!/usr/bin/env bash
# 票 S4b：一键把 YOPO L0 规划桥接进 PX4 SITL + Gazebo 容器。
# 只传场景、机数、种子和定位模式。不改安全参数，不调用 offboard_mission.py。
#   scripts/sitl/run_yopo_l0_sitl.sh --n 1
#   scripts/sitl/run_yopo_l0_sitl.sh --n 3 --positioning rtk
#   scripts/sitl/run_yopo_l0_sitl.sh --scene scenes/g2-pillar.json --n 1 --dry-run
# 1 机默认 scenes/g2-pillar.json（空 namespace）。3 机默认 scenes/swap-3-open.json（px4_1..px4_3）。
# 容器已经由 up.sh 拉起时，在里面补跑；否则自己起一趟仿真，任务段只跑 yopo_l0_live.py。
set -euo pipefail
# shellcheck disable=SC1091
. "$(dirname "$0")/common.sh"

SCENE=""
N="1"
POSITIONING="${YOPO_POSITIONING:-default}"
SEED="0"
DRY=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --scene) SCENE="${2:-}"; shift 2 ;;
    --scene=*) SCENE="${1#*=}"; shift ;;
    --n) N="${2:-}"; shift 2 ;;
    --n=*) N="${1#*=}"; shift ;;
    --positioning) POSITIONING="${2:-}"; shift 2 ;;
    --positioning=*) POSITIONING="${1#*=}"; shift ;;
    --seed) SEED="${2:-}"; shift 2 ;;
    --seed=*) SEED="${1#*=}"; shift ;;
    --dry-run) DRY=1; shift ;;
    -h|--help)
      echo "用法：$0 [--scene 场景卡] [--n 1|2|3] [--positioning default|rtk] [--seed N] [--dry-run]"
      exit 0
      ;;
    *) echo "用法：$0 [--scene 场景卡] [--n 1|2|3] [--positioning default|rtk] [--seed N] [--dry-run]" >&2; exit 2 ;;
  esac
done
if [[ "$POSITIONING" != "default" && "$POSITIONING" != "rtk" ]]; then
  echo "未知 --positioning=$POSITIONING（只要 default 或 rtk）" >&2
  exit 2
fi
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
if [[ "$DRY" == "1" ]]; then
  printf 'planner=yopo-l0\nscene=%s\nn=%s\npositioning=%s\nseed=%s\n' "$SCENE" "$N" "$POSITIONING" "$SEED"
  exit 0
fi
python3 "$SITL_DIR/fleet.py" layout --n "$N" --format json >/dev/null
if ! image_exists; then
  echo "镜像 $SITL_IMAGE 不存在：先跑 scripts/sitl/up.sh" >&2
  exit 2
fi

NAME="$(sitl_container_name "$N")"
others="$(sitl_running_names | grep -v "^${NAME}$" || true)"
if [[ -n "$others" ]]; then
  echo "已有仿真容器在跑（一次只起一趟）：$others" >&2
  exit 3
fi

CACHE="${YOPO_SWARM_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/yopo-swarm}"
cache_args=()
if [[ -d "$CACHE" ]]; then
  cache_args=(-e YOPO_SWARM_CACHE=/yopo-cache -v "$CACHE:/yopo-cache:ro")
fi

if "${DOCKER[@]}" ps -q -f "name=^${NAME}$" | grep -q .; then
  stack="$(printf '%s\n' "$SITL_STATE/stack-n${N}.txt")"
  if [[ ! -f "$stack" ]]; then
    echo "容器 $NAME 在跑，但没有 $stack" >&2
    exit 3
  fi
  RUN="$(cat "$stack")"
  "${DOCKER[@]}" exec \
    -e YOPO_SITL_N="$N" -e YOPO_SITL_PLANNER=yopo-l0 -e YOPO_L0_SCENE="$SCENE" \
    -e YOPO_L0_SEED="$SEED" -e YOPO_POSITIONING="$POSITIONING" \
    -e YOPO_SITL_WALL_TIMEOUT="${YOPO_SITL_WALL_TIMEOUT:-900}" \
    "${cache_args[@]}" \
    "$NAME" nice -n "$SITL_NICE" bash /repo/scripts/sitl/exec_mission.sh
  exit $?
fi

RUN="$SITL_STATE/runs/$(date +%Y%m%dT%H%M%S)-yopo-n${N}"
mkdir -p "$RUN"
"${DOCKER[@]}" rm -f "$NAME" >/dev/null 2>&1 || true
shm=1g
[[ "$N" != "1" ]] && shm=2g
set +e
"${DOCKER[@]}" run --rm --name "$NAME" \
  -e ROS_DOMAIN_ID=0 -e YOPO_SITL_N="$N" -e YOPO_SITL_MISSION=1 \
  -e YOPO_SITL_PLANNER=yopo-l0 -e YOPO_L0_SCENE="$SCENE" -e YOPO_L0_SEED="$SEED" \
  -e YOPO_SITL_WALL_TIMEOUT="${YOPO_SITL_WALL_TIMEOUT:-900}" \
  -e YOPO_POSITIONING="$POSITIONING" \
  "${cache_args[@]}" \
  -v "$REPO_ROOT:/repo:ro" -v "$RUN:/out" --shm-size "$shm" \
  "$SITL_IMAGE" nice -n "$SITL_NICE" bash /repo/scripts/sitl/in_container.sh
rc=$?
set -e
echo "run_dir $RUN rc=$rc"
exit "$rc"
