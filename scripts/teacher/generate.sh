#!/usr/bin/env bash
# 教师数据一条命令。障碍半径 0.20 m、地图边长 20 m 是假设，见 assumptions.py。
# 源码在 ~/ext/ego-planner-swarm，不进本仓库。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export PYTHONPATH="${ROOT}/scripts${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m teacher.generate "$@"
