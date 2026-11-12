#!/usr/bin/env bash
# D2b 教师扩种。几何假设见 assumptions.py（这是假设，不是论文复现）。
# 大文件默认写到 /workspace/yopo-teacher-data/d2b/，不进本仓库。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export PYTHONPATH="${ROOT}/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PATH="/usr/local/bin:${PATH}"
export PYTHONUNBUFFERED=1
if [ -x "${ROOT}/.venv/bin/python" ]; then
  exec "${ROOT}/.venv/bin/python" -m teacher.d2b "$@"
fi
exec python3 -m teacher.d2b "$@"
