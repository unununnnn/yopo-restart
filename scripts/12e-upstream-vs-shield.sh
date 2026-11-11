#!/usr/bin/env bash
# 票 12e：上游策略（影子盾只记录）vs certified（硬门）对照。
# 转给同目录的 Python 脚本。在仓库根目录执行：
#   scripts/12e-upstream-vs-shield.sh --jobs 2
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
if [[ -x .venv/bin/python ]]; then
  py=.venv/bin/python
else
  echo "先在仓库根目录执行 make venv" >&2
  exit 2
fi
exec "$py" "$root/scripts/12e-upstream-vs-shield.py" "$@"
