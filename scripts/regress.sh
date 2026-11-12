#!/usr/bin/env bash
# 合 main 前的回归：pytest + 场景/seed 批跑，打印通过率和关键指标表（Markdown）。
# 用法：scripts/regress.sh [额外场景卡...]
set -euo pipefail
cd "$(dirname "$0")/.."
if ! .venv/bin/python -c 'import torch, yopo_swarm_core, yopo_l0, yopo_l1, matplotlib' 2>/dev/null; then
  make venv
fi
PY=.venv/bin/python
OUT=out/regress/$(date +%Y%m%dT%H%M%S)
mkdir -p "$OUT"
TOTAL_START=${EPOCHREALTIME/./}
duration_s() {
  local us=$(( $2 - $1 ))
  printf '%d.%03d' "$(( us / 1000000 ))" "$(( (us % 1000000) / 1000 ))"
}

echo "## pytest（L0）"
set +e
PYTEST_START=${EPOCHREALTIME/./}
(
  $PY -m pytest -q -m "not slow" tests 2>&1 | tee "$OUT/pytest-l0.log" >/dev/null
  exit "${PIPESTATUS[0]}"
) &
P1_PID=$!
(
  (cd core && ../$PY -m pytest -q -p no:cacheprovider -m "not slow" \
    --deselect tests/test_upstream_pin.py::test_upstream_branch_head_has_not_moved) 2>&1 | tee "$OUT/pytest-core.log" >/dev/null
  exit "${PIPESTATUS[0]}"
) &
P2_PID=$!
wait "$P1_PID"
P1=$?
wait "$P2_PID"
P2=$?
PYTEST_END=${EPOCHREALTIME/./}
tail -2 "$OUT/pytest-l0.log"
echo "## pytest（core，vendored，含上游金样；不含 slow 与联网的上游 head 检查）"
tail -2 "$OUT/pytest-core.log"
PYTEST=$(( P1 || P2 ))
echo
echo "## 批跑（suite.txt：hover 判过；g2-pillar@upstream 与 open-3drone@hover 判据未冻结只报数）"
mapfile -t SUITE < <(sed -e 's/[[:space:]]*#.*//' -e '/^[[:space:]]*$/d' scenes/suite.txt)
JOBS=${REGRESS_JOBS:-$(( $(nproc) - 2 ))}
if [ "$JOBS" -lt 1 ]; then JOBS=1; fi
BATCH_START=${EPOCHREALTIME/./}
$PY -m yopo_l0.batch "${SUITE[@]}" "$@" --jobs "$JOBS" --out "$OUT" 2>/dev/null
BATCH=$?
BATCH_END=${EPOCHREALTIME/./}
echo
echo "## certified 冒烟（单 seed、仿真 4 s；全部门每步约 2.4 s 墙钟）"
SMOKE_START=${EPOCHREALTIME/./}
$PY -m yopo_l0.batch scenes/g2-pillar.json@certified --seeds 0 --duration 4 --no-determinism --no-shadow \
  --out "$OUT/certified-smoke" 2>/dev/null
BATCH=$(( BATCH || $? ))
SMOKE_END=${EPOCHREALTIME/./}
TOTAL_END=${EPOCHREALTIME/./}
set -e
echo
{
  echo '| 阶段 | 耗时（秒） |'
  echo '| --- | ---: |'
  printf '| pytest（L0 + core） | %s |\n' "$(duration_s "$PYTEST_START" "$PYTEST_END")"
  printf '| 批跑 | %s |\n' "$(duration_s "$BATCH_START" "$BATCH_END")"
  printf '| certified 冒烟 | %s |\n' "$(duration_s "$SMOKE_START" "$SMOKE_END")"
  printf '| 总计 | %s |\n' "$(duration_s "$TOTAL_START" "$TOTAL_END")"
} | tee "$OUT/timing.md"
echo "pytest=$([ $PYTEST -eq 0 ] && echo 通过 || echo 失败) 批跑=$([ $BATCH -eq 0 ] && echo 通过 || echo 失败) 输出 $OUT"
[ $PYTEST -eq 0 ] && [ $BATCH -eq 0 ]
