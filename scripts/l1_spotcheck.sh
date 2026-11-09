#!/usr/bin/env bash
# 票 09 L1 抽检运行手册。在 frank 的 WSL 或 Orin 上跑，本脚本不去连那些机器。
# 启动的是 yopo-swarm 现有的 ros2 launch yopo_swarm_px4 uavs.launch.py。
# 不调用会改 PX4 参数的子命令，也不改 g2_run.py 和安全清单文件。
# 跑前跑后各做一次只读 dump 和清单哈希，有差异就失败。
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"

# 这些子命令会改 PX4 参数。出现在 dump 命令里就拒绝执行。
FORBIDDEN_PARAM_CMDS=(
  "param set"
  "param save"
  "param load"
  "param reset"
  "param import"
)

usage() {
  cat <<'EOF'
用法：
  scripts/l1_spotcheck.sh --scene scenes/g2-pillar.json --seed 0 --out out/l1/g2-pillar
  scripts/l1_spotcheck.sh --prepare-only --scene scenes/g2-pillar.json --out out/l1/bundle
  scripts/l1_spotcheck.sh --check-params before.txt after.txt
  scripts/l1_spotcheck.sh --check-safety before.sha after.sha
  scripts/l1_spotcheck.sh --dump-params out/params.txt

全量抽检需要：
  YOPO_SWARM_ROOT   yopo-swarm 检出（含 ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py）
  PX4_PARAM_DUMP_CMD  只读，标准输出是参数表。例如已经连上的 pxh 里执行 param show。
可选：
  L1_LAUNCH_FILE    用来对齐 DeclareLaunchArgument 的 launch 文件
  L1_LOG_SRC        栈自己写日志的目录；结束后只复制 jsonl / flight.json，不改源文件
EOF
}

reject_writer() {
  local cmd lower bad
  cmd="$1"
  lower=$(printf '%s' "$cmd" | tr '[:upper:]' '[:lower:]')
  for bad in "${FORBIDDEN_PARAM_CMDS[@]}"; do
    case "$lower" in
      *"$bad"*)
        echo "拒绝：命令含「${bad}」，会改 PX4 参数。抽检只做只读 dump。" >&2
        exit 2
        ;;
    esac
  done
}

normalize_params() {
  grep -v '^[[:space:]]*#' "$1" | sed '/^[[:space:]]*$/d' | sort
}

check_pair() {
  local before=$1 after=$2 what=$3
  local nb na
  if [[ ! -f "$before" || ! -f "$after" ]]; then
    echo "缺少 ${what} 的前或后快照：$before / $after" >&2
    exit 1
  fi
  nb=$(mktemp)
  na=$(mktemp)
  normalize_params "$before" >"$nb"
  normalize_params "$after" >"$na"
  if ! cmp -s "$nb" "$na"; then
    echo "${what}在抽检前后不一致。本脚本没有去改它；差异如下：" >&2
    diff -u "$nb" "$na" >&2 || true
    rm -f "$nb" "$na"
    exit 3
  fi
  rm -f "$nb" "$na"
}

collect_safety_files() {
  local root=$1
  [[ -d "$root" ]] || return 0
  find "$root" -type f \
    \( -name 'g2_run.py' -o -name '*checklist*' -o -name '*safety*' \) \
    ! -path '*/.git/*' ! -path '*/build/*' ! -path '*/install/*' ! -path '*/log/*' \
    ! -path '*/__pycache__/*' \
    | sort
}

hash_safety() {
  local root=$1 dest=$2
  local f
  : >"$dest"
  while IFS= read -r f; do
    [[ -n "$f" ]] || continue
    sha256sum "$f"
  done < <(collect_safety_files "$root") | sort >"$dest"
}

dump_params() {
  local dest=$1
  if [[ -z "${PX4_PARAM_DUMP_CMD:-}" ]]; then
    echo "需要只读环境变量 PX4_PARAM_DUMP_CMD（标准输出即参数表）。本脚本不会执行会改参数的子命令。" >&2
    exit 2
  fi
  reject_writer "$PX4_PARAM_DUMP_CMD"
  bash -o pipefail -c "$PX4_PARAM_DUMP_CMD" >"$dest"
}

SCENE=""
SEED=0
POLICY=upstream
SHADOW=on
OUT=""
MODE=run
BEFORE=""
AFTER=""
DUMP_DEST=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scene) SCENE=$2; shift 2 ;;
    --seed) SEED=$2; shift 2 ;;
    --policy) POLICY=$2; shift 2 ;;
    --shadow) SHADOW=$2; shift 2 ;;
    --no-shadow) SHADOW=off; shift ;;
    --out) OUT=$2; shift 2 ;;
    --prepare-only) MODE=prepare; shift ;;
    --check-params) MODE=params; BEFORE=$2; AFTER=$3; shift 3 ;;
    --check-safety) MODE=safety; BEFORE=$2; AFTER=$3; shift 3 ;;
    --dump-params) MODE=dump; DUMP_DEST=$2; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数 $1" >&2; usage >&2; exit 1 ;;
  esac
done

case "$MODE" in
  params) check_pair "$BEFORE" "$AFTER" "PX4 参数"; exit 0 ;;
  safety) check_pair "$BEFORE" "$AFTER" "安全清单"; exit 0 ;;
  dump) dump_params "$DUMP_DEST"; exit 0 ;;
esac

if [[ -z "$SCENE" ]]; then
  echo "需要 --scene" >&2
  exit 1
fi
if [[ "$SCENE" != /* ]]; then
  SCENE="$ROOT/$SCENE"
fi
if [[ -z "$OUT" ]]; then
  OUT="$ROOT/out/l1-spotcheck"
fi
if [[ "$OUT" != /* ]]; then
  OUT="$ROOT/$OUT"
fi
mkdir -p "$OUT"

if [[ -x "$ROOT/.venv/bin/python" ]]; then
  PY=$ROOT/.venv/bin/python
else
  PY=python3
fi

launch_file_args=()
if [[ -n "${L1_LAUNCH_FILE:-}" ]]; then
  launch_file_args=(--launch-file "$L1_LAUNCH_FILE")
elif [[ -n "${YOPO_SWARM_ROOT:-}" && -f "$YOPO_SWARM_ROOT/ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py" ]]; then
  launch_file_args=(--launch-file "$YOPO_SWARM_ROOT/ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py")
fi

"$PY" -m yopo_l1.export \
  --scene "$SCENE" --seed "$SEED" --policy "$POLICY" --shadow "$SHADOW" \
  --out "$OUT/bundle" "${launch_file_args[@]}"

cmd="ros2 launch yopo_swarm_px4 uavs.launch.py $(paste -sd ' ' "$OUT/bundle/launch-args.txt")"
echo "$cmd"

if [[ "$MODE" == prepare ]]; then
  exit 0
fi

if [[ -z "${YOPO_SWARM_ROOT:-}" ]]; then
  echo "全量抽检需要 YOPO_SWARM_ROOT 指向 yopo-swarm 检出。box 上用 --prepare-only，不要去连 WSL 或 Orin。" >&2
  exit 1
fi
if [[ ! -f "$YOPO_SWARM_ROOT/ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py" ]]; then
  echo "没有找到 $YOPO_SWARM_ROOT/ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py" >&2
  exit 1
fi

mkdir -p "$OUT/guard" "$OUT/log"
cp "$OUT/bundle/run.json" "$OUT/bundle/scene.json" "$OUT/log/"
hash_safety "$YOPO_SWARM_ROOT" "$OUT/guard/safety.before"
dump_params "$OUT/guard/px4-params.before"

set +e
start_ns=$(date +%s%N)
(
  set +u
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f "$YOPO_SWARM_ROOT/ros2_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$YOPO_SWARM_ROOT/ros2_ws/install/setup.bash"
  fi
  set -u
  cd "$YOPO_SWARM_ROOT"
  mapfile -t LAUNCH_ARGS <"$OUT/bundle/launch-args.txt"
  ros2 launch yopo_swarm_px4 uavs.launch.py "${LAUNCH_ARGS[@]}"
) >"$OUT/log/launch.stdout" 2>"$OUT/log/launch.stderr"
launch_rc=$?
end_ns=$(date +%s%N)
set -e
"$PY" -c 'import sys; print((int(sys.argv[1]) - int(sys.argv[2])) / 1e9)' "$end_ns" "$start_ns" >"$OUT/log/wall_time_s.txt"

if [[ -n "${L1_LOG_SRC:-}" ]]; then
  find "$L1_LOG_SRC" -type f \
    \( -name '*.jsonl' -o -name '*.jsonl.gz' -o -name 'flight.json' -o -name 'l1-flight.json' -o -name 'ticks.jsonl' \) \
    -exec cp -a {} "$OUT/log/" \;
fi

dump_params "$OUT/guard/px4-params.after"
hash_safety "$YOPO_SWARM_ROOT" "$OUT/guard/safety.after"
check_pair "$OUT/guard/px4-params.before" "$OUT/guard/px4-params.after" "PX4 参数"
check_pair "$OUT/guard/safety.before" "$OUT/guard/safety.after" "安全清单"

if [[ "$launch_rc" -ne 0 ]]; then
  echo "ros2 launch 退出 ${launch_rc}。日志在 $OUT/log。参数和安全清单没有变。" >&2
  exit 4
fi

"$PY" -m yopo_l1.importer "$OUT/log"
echo "结果 $OUT/log/result.json"
