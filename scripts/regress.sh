#!/usr/bin/env bash
# 合 main 前的回归：pytest + 场景/seed 批跑，打印通过率和关键指标表（Markdown）。
# 用法：scripts/regress.sh [额外场景卡...]
set -euo pipefail
cd "$(dirname "$0")/.."
if [ ! -x .venv/bin/python ]; then
  python3 -m venv .venv
  .venv/bin/pip -q install -e '.[test]'
fi
PY=.venv/bin/python
OUT=out/regress/$(date +%Y%m%dT%H%M%S)
mkdir -p "$OUT"
echo "## pytest"
set +e
$PY -m pytest -q -m "not slow" 2>&1 | tee "$OUT/pytest.log" | tail -3
PYTEST=${PIPESTATUS[0]}
echo
echo "## 批跑"
$PY -m yopo_l0.batch scenes/g2-hover.json scenes/g2-hover-lift.json "$@" --out "$OUT"
BATCH=$?
set -e
echo
echo "pytest=$([ $PYTEST -eq 0 ] && echo 通过 || echo 失败) 批跑=$([ $BATCH -eq 0 ] && echo 通过 || echo 失败) 输出 $OUT"
[ $PYTEST -eq 0 ] && [ $BATCH -eq 0 ]
