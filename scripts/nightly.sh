#!/usr/bin/env bash
# 票 04：box 上每晚跑的场景套件。不接 GitHub Actions。
# 套件见 scenes/suite.txt，含 g2-pillar@upstream（策略实现在票 02，这里只调用）。
# 结果：out/nightly/YYYYMMDD/
#   summary.md、results.json、每组 <场景>@<策略>/seed<N>/{result.json,ticks.jsonl}
#   以及该组首个 seed 的 preview.mp4（俯视+侧视，播放 ≤60s）
# 同一天重跑会先清空该目录。失败时 ticks.jsonl 仍留下，重放：
#   python -m yopo_l0.render --log <ticks.jsonl> --scene <场景卡.json> --out preview.mp4
# 影子拒绝率读结果 JSON（默认盾开）。合 main 前的门仍是 scripts/regress.sh（另含 certified 冒烟）。
# 用法：scripts/nightly.sh
set -euo pipefail
cd "$(dirname "$0")/.."
if ! .venv/bin/python -c 'import torch, yopo_swarm_core, yopo_l0, matplotlib' 2>/dev/null; then
  make venv
fi
PY=.venv/bin/python
DAY=$(date +%Y%m%d)
OUT=out/nightly/$DAY
case "$OUT" in
  out/nightly/[0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]) ;;
  *) echo "拒绝写入意外路径 $OUT" >&2; exit 1 ;;
esac
rm -rf "$OUT"
mkdir -p "$OUT"
mapfile -t SUITE < <(sed -e 's/[[:space:]]*#.*//' -e '/^[[:space:]]*$/d' scenes/suite.txt)
echo "## 场景套件 → $OUT"
set +e
$PY -m yopo_l0.batch "${SUITE[@]}" --jobs 4 --out "$OUT" 2>"$OUT/batch.err"
BATCH=$?
RENDER=0
for spec in "${SUITE[@]}"; do
  if [[ "$spec" == *@* ]]; then
    card="${spec%@*}"
    policy="${spec##*@}"
  else
    card="$spec"
    policy="upstream"
  fi
  name=$($PY -c 'import json,sys; print(json.load(open(sys.argv[1]))["name"])' "$card")
  seed=$($PY -c 'import json,sys; print(json.load(open(sys.argv[1])).get("seeds",[0])[0])' "$card")
  log="$OUT/${name}@${policy}/seed${seed}/ticks.jsonl"
  vid="$OUT/${name}@${policy}/seed${seed}/preview.mp4"
  if [[ -f "$log" ]]; then
    if ! $PY -m yopo_l0.render --log "$log" --scene "$card" --out "$vid" --max-seconds 60; then
      RENDER=1
    fi
  else
    echo "缺少日志 $log" >&2
    RENDER=1
  fi
done
set -e
echo "nightly 批跑=$([ "$BATCH" -eq 0 ] && echo 通过 || echo 失败) 视频=$([ "$RENDER" -eq 0 ] && echo 通过 || echo 失败) 输出 $OUT"
echo "失败重放：python -m yopo_l0.render --log $OUT/<场景>@<策略>/seed<N>/ticks.jsonl --scene scenes/<卡>.json --out preview.mp4"
if [[ "$BATCH" -ne 0 || "$RENDER" -ne 0 ]]; then
  exit 1
fi
