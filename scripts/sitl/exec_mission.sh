#!/usr/bin/env bash
# 已由 up.sh --n 拉起的容器里补跑任务（docker exec）。不起第二个 PX4。
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/ros_ws/install/setup.bash
export PYTHONPATH=/repo/core/src${PYTHONPATH:+:$PYTHONPATH}
set +e
if [[ "${YOPO_SITL_PLANNER:-offboard}" == "yopo-l0" ]]; then
  PYTHONPATH="/repo:/repo/core/src${PYTHONPATH:+:$PYTHONPATH}" \
    python3 /repo/scripts/sitl/yopo_l0_live.py \
      --out /out/result.json \
      --scene "${YOPO_L0_SCENE:-scenes/g2-pillar.json}" \
      --n "${YOPO_SITL_N:?}" \
      --seed "${YOPO_L0_SEED:-0}" \
      --wall-timeout "${YOPO_SITL_WALL_TIMEOUT:-900}" \
      --positioning "${YOPO_POSITIONING:-default}"
  rc=$?
else
  python3 /repo/scripts/sitl/fleet.py run --out /out/result.json --n "${YOPO_SITL_N:?}" --wall-timeout "${YOPO_SITL_WALL_TIMEOUT:-900}"
  rc=$?
fi
set -e
if [[ $rc -ne 0 && -f /out/result.json ]]; then
  if python3 -c 'import json,sys; sys.exit(0 if json.load(open("/out/result.json")).get("ok") else 1)'; then
    echo "mission ok, python rc=$rc (teardown); treating as success"
    rc=0
  fi
fi
exit "$rc"
