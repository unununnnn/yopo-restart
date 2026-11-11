#!/usr/bin/env bash
# 票 12c：在当前检出上渲染各里程碑 seed 0 的交付视频（播放 ≤60 s）。
#
# 用法：
#   scripts/render_delivery.sh
#     跑下面 7 条场景（seed 0），用票 04 的离线渲染器出俯视+侧视 mp4，
#     叠上里程碑、层级、commit、seed，写 docs/results/videos.md，
#     并复制到 /workspace/yopo-delivery-videos/。
#   scripts/render_delivery.sh --from-log LOG --scene CARD --milestone M0 \
#       --policy upstream --seed 0 --out-mp4 PATH [--commit SHA] [--index FILE] [--copy-to DIR]
#
# 视频不进仓库：out/ 已在 .gitignore。成片默认写在 out/delivery/。
# 需要系统里的 ffmpeg。
set -euo pipefail
cd "$(dirname "$0")/.."
if ! .venv/bin/python -c 'import torch, yopo_swarm_core, yopo_l0, matplotlib' 2>/dev/null; then
  make venv
fi
PY=.venv/bin/python

FROM_LOG=""
SCENE=""
MILESTONE=""
POLICY=""
SEED=0
OUT_MP4=""
OUT_DIR="out/delivery"
LOGS_OUT="out/delivery/logs"
INDEX=""
WRITE_INDEX=""
COPY_TO="/workspace/yopo-delivery-videos"
COMMIT=""
MAX_SECONDS=60
JOBS=4
REUSE_LOGS=0
FONT=""
for cand in \
  /usr/share/fonts/truetype/dejavu/DejaVuSans.ttf \
  /usr/share/fonts/truetype/droid/DroidSans.ttf \
  /usr/share/fonts/truetype/sand-box/custom/Droid\ Sans/DroidSans.ttf
do
  if [[ -f "$cand" ]]; then
    FONT=$cand
    break
  fi
done

usage() {
  sed -n '2,16p' "$0" | sed 's/^# \?//'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --from-log) FROM_LOG=$2; shift 2 ;;
    --scene) SCENE=$2; shift 2 ;;
    --milestone) MILESTONE=$2; shift 2 ;;
    --policy) POLICY=$2; shift 2 ;;
    --seed) SEED=$2; shift 2 ;;
    --out-mp4) OUT_MP4=$2; shift 2 ;;
    --out) OUT_DIR=$2; shift 2 ;;
    --logs-out) LOGS_OUT=$2; shift 2 ;;
    --index) INDEX=$2; WRITE_INDEX=1; shift 2 ;;
    --no-index) WRITE_INDEX=0; shift ;;
    --copy-to) COPY_TO=$2; shift 2 ;;
    --no-copy) COPY_TO=""; shift ;;
    --commit) COMMIT=$2; shift 2 ;;
    --max-seconds) MAX_SECONDS=$2; shift 2 ;;
    --jobs) JOBS=$2; shift 2 ;;
    --reuse-logs) REUSE_LOGS=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf '未知参数 %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if ! command -v ffmpeg >/dev/null 2>&1 || ! command -v ffprobe >/dev/null 2>&1; then
  printf '渲染 mp4 需要 ffmpeg 和 ffprobe 在 PATH 里\n' >&2
  exit 2
fi
if [[ -z "$FONT" ]]; then
  printf '找不到可叠加字幕的字体\n' >&2
  exit 2
fi

if [[ -z "$COMMIT" ]]; then
  COMMIT=$(git rev-parse HEAD)
fi
COMMIT_SHORT=${COMMIT:0:7}

# max_seconds 必须落在 (0, 60]，与 yopo_l0.render 一致。
$PY - "$MAX_SECONDS" <<'PY'
import sys
v = float(sys.argv[1])
if not (0 < v <= 60):
    sys.exit("视频时长上限必须在 (0, 60] 秒")
PY

# 交付清单：里程碑、场景卡、策略、字幕里的场景名。
# crossing-forest 场景卡本身就是 medium 档。
DELIVERY_ROWS=(
  "M0|scenes/g2-pillar.json|upstream|g2-pillar@upstream"
  "M1|scenes/formation-hold.json|formation|formation-hold"
  "M2|scenes/crossing-pillars.json|crossing|crossing-pillars"
  "M2|scenes/crossing-forest.json|crossing|crossing-forest-medium"
  "M3|scenes/formation-switch.json|switch|formation-switch"
  "M4|scenes/formation-6-open.json|formation|formation-6-open"
  "M4|scenes/crossing-6-forest.json|crossing|crossing-6-forest"
)

slug_of() {
  # $1 milestone $2 label → 文件名主干（@ 换成 -，避免文件名带特殊字符）
  local label=${2//@/-}
  printf '%s-%s-seed%s' "$1" "$label" "$SEED"
}

fmt_size() {
  $PY -c 'import sys
n=int(sys.argv[1])
if n >= 1048576:
    print(f"{n/1048576:.1f} MiB")
elif n >= 1024:
    print(f"{n/1024:.1f} KiB")
else:
    print(f"{n} B")' "$1"
}

probe_duration() {
  ffprobe -v error -show_entries format=duration -of default=nw=1:nk=1 "$1"
}

caption_text() {
  # $1 milestone $2 label $3 policy
  local label=$2
  if [[ -n "$3" ]]; then
    printf '%s  L0  %s  %s  seed %s' "$1" "$label" "$COMMIT_SHORT" "$SEED"
  else
    printf '%s  L0  %s  %s  seed %s' "$1" "$label" "$COMMIT_SHORT" "$SEED"
  fi
}

burn_overlay() {
  # $1 raw mp4 $2 out mp4 $3 caption
  local raw=$1 out=$2 caption=$3
  local cap tmp
  cap=$(mktemp)
  tmp=$(mktemp --suffix=.mp4)
  printf '%s\n' "$caption" >"$cap"
  # 字幕放在画面底部，避开渲染器顶部的场景名和仿真时间。
  # textfile 避免 caption 里的冒号、空格把 drawtext 拆开。
  if ! ffmpeg -hide_banner -loglevel error -y -i "$raw" \
    -vf "drawtext=fontfile=${FONT}:textfile=${cap}:fontcolor=white:fontsize=16:box=1:boxcolor=black@0.65:boxborderw=8:x=16:y=h-th-16" \
    -an -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p -movflags +faststart \
    "$tmp" </dev/null; then
    rm -f "$cap" "$tmp"
    printf 'ffmpeg 叠字幕失败\n' >&2
    return 1
  fi
  rm -f "$cap"
  mv "$tmp" "$out"
}

render_one() {
  # $1 log $2 scene $3 milestone $4 policy $5 label $6 out mp4
  local log=$1 scene=$2 milestone=$3 policy=$4 label=$5 out=$6
  local raw caption dur size
  if [[ ! -f "$log" ]]; then
    printf '缺少日志 %s\n' "$log" >&2
    return 1
  fi
  mkdir -p "$(dirname "$out")"
  raw="${out%.mp4}.raw.mp4"
  $PY scripts/_render_delivery_clip.py --log "$log" --scene "$scene" --out "$raw" --max-seconds "$MAX_SECONDS"
  caption=$(caption_text "$milestone" "$label" "$policy")
  burn_overlay "$raw" "$out" "$caption"
  rm -f "$raw"
  dur=$(probe_duration "$out")
  $PY - "$dur" "$MAX_SECONDS" <<'PY'
import sys
d, cap = float(sys.argv[1]), float(sys.argv[2])
if d > cap + 0.5:
    sys.exit(f"成片时长 {d:.3f}s 超过上限 {cap}s")
PY
  size=$(stat -c%s "$out")
  printf '写入 %s（%.2fs，%s，%s）\n' "$out" "$dur" "$(fmt_size "$size")" "$caption"
}

write_index() {
  # stdin: rows "file|dur|bytes|scene|commit|milestone|label"
  local path=$1
  mkdir -p "$(dirname "$path")"
  {
    printf '%s\n' "# 交付视频（票 12c）"
    printf '\n'
    printf '%s\n' "视频不进仓库：\`out/\` 已在 \`.gitignore\`。成片写在 \`${OUT_DIR}/\`，本表指向这些路径。用 \`scripts/render_delivery.sh\` 在当前检出上重渲。"
    if [[ -n "$COPY_TO" ]]; then
      printf '%s\n' "另有一份复制在 \`${COPY_TO}/\`。"
    fi
    printf '\n'
    printf '%s\n' "| 项 | 值 |"
    printf '%s\n' "|---|---|"
    printf '| 代码 commit | `%s` |\n' "$COMMIT"
    printf '%s\n' "| 层级 | L0 |"
    printf '| seed | %s |\n' "$SEED"
    printf '| 播放时长上限 | %s s |\n' "$MAX_SECONDS"
    printf '%s\n' "| 渲染器 | \`python -m yopo_l0.render\`（票 04）。字幕叠里程碑、层级、commit、seed。 |"
    printf '\n'
    printf '%s\n' "| 文件 | 时长 s | 大小 | 场景 | commit |"
    printf '%s\n' "|---|---|---|---|---|"
    local file dur bytes scene commit milestone label
    while IFS='|' read -r file dur bytes scene commit milestone label; do
      printf '| `%s` | %.2f | %s | %s | `%s` |\n' \
        "$file" "$dur" "$(fmt_size "$bytes")" "$scene" "$commit"
    done
    printf '\n'
    printf '%s\n' "每条都是 seed ${SEED}、层级 L0。M2、M4 各有两张场景卡，各渲一条。"
  } >"$path"
  printf '写入索引 %s\n' "$path"
}

copy_video() {
  local src=$1
  if [[ -z "$COPY_TO" ]]; then
    return 0
  fi
  mkdir -p "$COPY_TO"
  cp -f "$src" "$COPY_TO/$(basename "$src")"
}

if [[ -n "$FROM_LOG" ]]; then
  if [[ -z "$SCENE" || -z "$MILESTONE" || -z "$OUT_MP4" ]]; then
    printf '--from-log 需要同时给 --scene、--milestone、--out-mp4\n' >&2
    exit 2
  fi
  if [[ -z "$POLICY" ]]; then
    POLICY="upstream"
  fi
  label=$(basename "${SCENE%.json}")
  if [[ -n "$POLICY" ]]; then
    label="${label}@${POLICY}"
  fi
  if [[ -z "$WRITE_INDEX" ]]; then
    WRITE_INDEX=0
  fi
  if [[ -z "$INDEX" ]]; then
    INDEX=$(dirname "$OUT_MP4")/videos.md
  fi
  render_one "$FROM_LOG" "$SCENE" "$MILESTONE" "$POLICY" "$label" "$OUT_MP4"
  copy_video "$OUT_MP4"
  if [[ "$WRITE_INDEX" == "1" ]]; then
    dur=$(probe_duration "$OUT_MP4")
    size=$(stat -c%s "$OUT_MP4")
    printf '%s|%s|%s|%s|%s|%s|%s\n' \
      "$(basename "$OUT_MP4")" "$dur" "$size" "$SCENE${POLICY:+@$POLICY}" "$COMMIT" "$MILESTONE" "$label" \
      | write_index "$INDEX"
  fi
  exit 0
fi

if [[ -n "$OUT_MP4" ]]; then
  printf '整包渲染不要带 --out-mp4；单条请用 --from-log\n' >&2
  exit 2
fi
if [[ -z "$WRITE_INDEX" ]]; then
  WRITE_INDEX=1
fi
if [[ -z "$INDEX" ]]; then
  INDEX=docs/results/videos.md
fi

mkdir -p "$OUT_DIR" "$LOGS_OUT"

# 缺日志时才批跑；--reuse-logs 要求 7 条 ticks.jsonl 都在。
need_batch=0
specs=()
for row in "${DELIVERY_ROWS[@]}"; do
  IFS='|' read -r _ms card policy _label <<<"$row"
  name=$($PY -c 'import json,sys; print(json.load(open(sys.argv[1]))["name"])' "$card")
  log="$LOGS_OUT/${name}@${policy}/seed${SEED}/ticks.jsonl"
  specs+=("${card}@${policy}")
  if [[ ! -f "$log" ]]; then
    need_batch=1
  fi
done
if [[ "$REUSE_LOGS" -eq 1 && "$need_batch" -eq 1 ]]; then
  printf '--reuse-logs 但 %s 里日志不齐\n' "$LOGS_OUT" >&2
  exit 1
fi
if [[ "$need_batch" -eq 1 || "$REUSE_LOGS" -eq 0 ]]; then
  echo "## 交付批跑 seed ${SEED} → $LOGS_OUT"
  "$PY" -m yopo_l0.batch "${specs[@]}" --seeds "$SEED" --no-determinism --jobs "$JOBS" --out "$LOGS_OUT"
fi

index_rows=""
for row in "${DELIVERY_ROWS[@]}"; do
  IFS='|' read -r ms card policy label <<<"$row"
  name=$($PY -c 'import json,sys; print(json.load(open(sys.argv[1]))["name"])' "$card")
  log="$LOGS_OUT/${name}@${policy}/seed${SEED}/ticks.jsonl"
  slug=$(slug_of "$ms" "$label")
  out="$OUT_DIR/${slug}.mp4"
  render_one "$log" "$card" "$ms" "$policy" "$label" "$out"
  copy_video "$out"
  dur=$(probe_duration "$out")
  size=$(stat -c%s "$out")
  index_rows+="$(printf '%s|%s|%s|%s|%s|%s|%s' \
    "${slug}.mp4" "$dur" "$size" "${card}@${policy}" "$COMMIT" "$ms" "$label")"$'\n'
done

if [[ "$WRITE_INDEX" == "1" ]]; then
  printf '%s' "$index_rows" | write_index "$INDEX"
fi
