#!/usr/bin/env bash
# 票 09b L1 抽检。Orin 上的驱动是 yopo-swarm 的 scripts/run_g2.sh nominal。
# 不走 uavs.launch.py，不创建宿主虚拟环境，不安装 torch。
# 不调用会改 PX4 参数的子命令，也不改 g2_run.py 和安全清单文件。
# 跑前跑后各做一次只读 dump 和清单哈希，有差异就失败。
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
# 只给本仓库的 export/import/collect 用。不要导出到 run_g2.sh，避免盖住 swarm 自己的包。
L1_PYTHONPATH="$ROOT:$ROOT/core/src${PYTHONPATH:+:$PYTHONPATH}"

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
  scripts/l1_spotcheck.sh --prepare-only --scene scenes/g2-pillar.json --seed 0 --out out/l1/g2-pillar
  YOPO_SWARM_ROOT=/path/to/yopo-swarm PX4_PARAM_DUMP_CMD='param show' \
    scripts/l1_spotcheck.sh --scene scenes/g2-pillar.json --seed 0 --out out/l1/g2-pillar
  scripts/l1_spotcheck.sh --check-params before.txt after.txt
  scripts/l1_spotcheck.sh --check-safety before.sha after.sha
  scripts/l1_spotcheck.sh --dump-params out/params.txt

--prepare-only 写 bundle/ 和 job-card.json。全量跑在 Orin 上执行
  YOPO_SIM_SEED=<seed> bash "$YOPO_SWARM_ROOT/scripts/run_g2.sh" nominal
种子只经 YOPO_SIM_SEED。不往 uavs.launch.py 传 scene/seed/policy/shadow/start/goal。

全量抽检需要：
  YOPO_SWARM_ROOT     yopo-swarm 检出（含 scripts/run_g2.sh）
  PX4_PARAM_DUMP_CMD  只读，标准输出是参数表
可选：
  YOPO_L1_PYTHON      已能 import numpy 的解释器。不设置时用已有 .venv、python3，
                      或镜像 yopo-swarm:orin-infer-cu13。不会去装依赖。
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

py_ok() {
  PYTHONPATH="$L1_PYTHONPATH" "$1" -c 'import numpy, yopo_l1' >/dev/null 2>&1
}

choose_py() {
  if [[ -n "${YOPO_L1_PYTHON:-}" ]]; then
    if py_ok "$YOPO_L1_PYTHON"; then
      PY=$YOPO_L1_PYTHON
      return
    fi
    echo "YOPO_L1_PYTHON=$YOPO_L1_PYTHON 不能 import numpy 和 yopo_l1。" >&2
    exit 1
  fi
  if [[ -x "$ROOT/.venv/bin/python" ]] && py_ok "$ROOT/.venv/bin/python"; then
    PY=$ROOT/.venv/bin/python
    return
  fi
  if command -v python3 >/dev/null 2>&1 && py_ok python3; then
    PY=python3
    return
  fi
  if command -v docker >/dev/null 2>&1 && docker image inspect yopo-swarm:orin-infer-cu13 >/dev/null 2>&1; then
    PY=docker-infer
    return
  fi
  echo "没有可用的 Python。抽检不创建宿主虚拟环境，也不安装 torch。" >&2
  echo "需要能 import numpy 的 python3，或已有镜像 yopo-swarm:orin-infer-cu13。" >&2
  exit 1
}

run_py() {
  if [[ "$PY" == "docker-infer" ]]; then
    local -a mounts=(-v "$ROOT:$ROOT")
    if [[ "$OUT" != "$ROOT" && "$OUT" != "$ROOT"/* ]]; then
      mounts+=(-v "$OUT:$OUT")
    fi
    if [[ -n "${YOPO_SWARM_ROOT:-}" && "$YOPO_SWARM_ROOT" != "$ROOT" && "$YOPO_SWARM_ROOT" != "$ROOT"/* ]]; then
      mounts+=(-v "$YOPO_SWARM_ROOT:$YOPO_SWARM_ROOT:ro")
    fi
    docker run --rm "${mounts[@]}" -w "$ROOT" -e PYTHONPATH="$ROOT:$ROOT/core/src" \
      yopo-swarm:orin-infer-cu13 python3 "$@"
  else
    PYTHONPATH="$L1_PYTHONPATH" "$PY" "$@"
  fi
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

choose_py

launch_file_args=()
if [[ -n "${L1_LAUNCH_FILE:-}" ]]; then
  launch_file_args=(--launch-file "$L1_LAUNCH_FILE")
elif [[ -n "${YOPO_SWARM_ROOT:-}" && -f "$YOPO_SWARM_ROOT/ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py" ]]; then
  launch_file_args=(--launch-file "$YOPO_SWARM_ROOT/ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py")
fi

run_py -m yopo_l1.export \
  --scene "$SCENE" --seed "$SEED" --policy "$POLICY" --shadow "$SHADOW" \
  --backend run_g2 --spot "$OUT" --out "$OUT/bundle" "${launch_file_args[@]}"

cp "$OUT/bundle/job-card.json" "$OUT/job-card.json"
echo "job-card $OUT/job-card.json"
run_py -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["command"])' "$OUT/job-card.json"

if [[ "$MODE" == prepare ]]; then
  exit 0
fi

if [[ -z "${YOPO_SWARM_ROOT:-}" ]]; then
  echo "全量抽检需要 YOPO_SWARM_ROOT 指向 yopo-swarm 检出。box 上用 --prepare-only。" >&2
  exit 1
fi
if [[ ! -f "$YOPO_SWARM_ROOT/scripts/run_g2.sh" ]]; then
  echo "没有找到 $YOPO_SWARM_ROOT/scripts/run_g2.sh" >&2
  exit 1
fi

mkdir -p "$OUT/guard" "$OUT/log"
cp "$OUT/bundle/run.json" "$OUT/bundle/scene.json" "$OUT/log/"
if git -C "$YOPO_SWARM_ROOT" rev-parse HEAD >/dev/null 2>&1; then
  git -C "$YOPO_SWARM_ROOT" rev-parse HEAD >"$OUT/guard/swarm-sha.txt"
fi
hash_safety "$YOPO_SWARM_ROOT" "$OUT/guard/safety.before"
dump_params "$OUT/guard/px4-params.before"

printf '%s\n' "YOPO_SIM_SEED=$SEED bash \"$YOPO_SWARM_ROOT/scripts/run_g2.sh\" nominal" >"$OUT/log/command.txt"
start_s=$(date +%s)
set +e
(
  cd "$YOPO_SWARM_ROOT"
  export YOPO_SIM_SEED="$SEED"
  bash "$YOPO_SWARM_ROOT/scripts/run_g2.sh" nominal
) >"$OUT/log/run_g2.stdout" 2>"$OUT/log/run_g2.stderr"
launch_rc=$?
run_py -m yopo_l1.collect "$YOPO_SWARM_ROOT" "$OUT/log" --since "$start_s" >"$OUT/log/collect.json"
collect_rc=$?
end_ns=$(date +%s%N)
set -e
run_py -c 'import sys; print((int(sys.argv[1]) - int(sys.argv[2])) / 1e9)' "$end_ns" "$((start_s * 1000000000))" >"$OUT/log/wall_time_s.txt"

dump_params "$OUT/guard/px4-params.after"
hash_safety "$YOPO_SWARM_ROOT" "$OUT/guard/safety.after"
check_pair "$OUT/guard/px4-params.before" "$OUT/guard/px4-params.after" "PX4 参数"
check_pair "$OUT/guard/safety.before" "$OUT/guard/safety.after" "安全清单"

if [[ "$launch_rc" -ne 0 ]]; then
  echo "run_g2.sh 退出 ${launch_rc}。日志在 $OUT/log。参数和安全清单没有变。" >&2
  exit 4
fi
if [[ "$collect_rc" -ne 0 ]]; then
  echo "没有收下本次 run_g2 的 result.json。日志在 $OUT/log。" >&2
  exit 5
fi

run_py -m yopo_l1.importer "$OUT/log"
echo "结果 $OUT/log/result.json"
