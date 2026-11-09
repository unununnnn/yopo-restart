#!/usr/bin/env bash
# 合 main 前的回归：pytest + 场景/seed 批跑，打印通过率和关键指标表（Markdown）。
# 用法：scripts/regress.sh [额外场景卡...]
set -euo pipefail
cd "$(dirname "$0")/.."
if ! .venv/bin/python -c 'import torch, yopo_swarm_core, yopo_l0, matplotlib' 2>/dev/null; then
  make venv
fi
PY=.venv/bin/python
OUT=out/regress/$(date +%Y%m%dT%H%M%S)
mkdir -p "$OUT"
echo "## pytest（L0）"
set +e
$PY -m pytest -q -m "not slow" tests 2>&1 | tee "$OUT/pytest-l0.log" | tail -2
P1=${PIPESTATUS[0]}
echo "## pytest（core，vendored，含上游金样；不含 slow 与联网的上游 head 检查）"
(cd core && ../$PY -m pytest -q -p no:cacheprovider -m "not slow" \
  --deselect tests/test_upstream_pin.py::test_upstream_branch_head_has_not_moved) 2>&1 | tee "$OUT/pytest-core.log" | tail -2
P2=${PIPESTATUS[0]}
PYTEST=$(( P1 || P2 ))
echo
echo "## 批跑（suite.txt：hover 判过；g2-pillar@upstream 与 open-3drone@hover 判据未冻结只报数）"
mapfile -t SUITE < <(sed -e 's/[[:space:]]*#.*//' -e '/^[[:space:]]*$/d' scenes/suite.txt)
$PY -m yopo_l0.batch "${SUITE[@]}" "$@" --jobs 4 --out "$OUT" 2>/dev/null
BATCH=$?
echo
echo "## certified 冒烟（单 seed、仿真 4 s；全部门每步约 2.4 s 墙钟）"
$PY -m yopo_l0.batch scenes/g2-pillar.json@certified --seeds 0 --duration 4 --no-determinism --no-shadow \
  --out "$OUT/certified-smoke" 2>/dev/null
BATCH=$(( BATCH || $? ))
set -e
echo
echo "pytest=$([ $PYTEST -eq 0 ] && echo 通过 || echo 失败) 批跑=$([ $BATCH -eq 0 ] && echo 通过 || echo 失败) 输出 $OUT"
[ $PYTEST -eq 0 ] && [ $BATCH -eq 0 ]
