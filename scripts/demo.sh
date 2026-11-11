#!/usr/bin/env bash
# 票 12a：一条命令跑单机 + 三机 + 六机 demo。
# 用法：scripts/demo.sh
# 产物：out/demo/<场景>@<策略>/seed0/{result.json,ticks.jsonl} 和 summary.md
# 失败时退出非 0，不放宽已冻门槛。
set -euo pipefail
cd "$(dirname "$0")/.."
if ! .venv/bin/python -c 'import torch, yopo_swarm_core, yopo_l0' 2>/dev/null; then
  make venv
fi
OUT=out/demo
mkdir -p "$OUT"
echo "commit $(git rev-parse HEAD 2>/dev/null || echo unknown)  dirty=$(git status --porcelain 2>/dev/null | wc -l)  out=$OUT"
.venv/bin/python -m yopo_l0.batch \
  scenes/g2-pillar.json@upstream@0 \
  scenes/formation-hold.json@formation@0 \
  scenes/formation-6-open.json@formation@0 \
  --jobs 4 --out "$OUT"
echo "demo 写在 $OUT（单机 g2-pillar、三机 formation-hold、六机 formation-6-open，各 seed 0）"
