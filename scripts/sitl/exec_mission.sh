#!/usr/bin/env bash
# 已由 up.sh --n 拉起的容器里补跑任务（docker exec）。不起第二个 PX4。
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/ros_ws/install/setup.bash
export PYTHONPATH=/repo/core/src${PYTHONPATH:+:$PYTHONPATH}
set +e
python3 /repo/scripts/sitl/fleet.py run --out /out/result.json --n "${YOPO_SITL_N:?}" --wall-timeout "${YOPO_SITL_WALL_TIMEOUT:-900}"
rc=$?
set -e
if [[ $rc -ne 0 && -f /out/result.json ]]; then
  if python3 -c 'import json,sys; sys.exit(0 if json.load(open("/out/result.json")).get("ok") else 1)'; then
    echo "mission ok, python rc=$rc (teardown); treating as success"
    rc=0
  fi
fi
exit "$rc"
