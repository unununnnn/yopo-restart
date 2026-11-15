"""票 B1：把一批 SITL 运行目录汇总成 runs.json 的一行一趟（只读，不改运行目录）。

    python3 scripts/sitl/b1_collect.py <index.txt> > rows.json

index.txt 是 ``/workspace/b1lab/arm.sh`` 写的「done <tag> seed=<s> rc=<rc> run=<dir>」行。
"""

from __future__ import annotations

import json
import re
import statistics
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO))

from yopo_l0.sitl_lockstep import count_sensor_timeouts  # noqa: E402

DONE = re.compile(r"done (?P<tag>\S+) seed=(?P<seed>\d+) rc=(?P<rc>-?\d+) run=(?P<run>\S*)")


def _r(x, nd=4):
    return None if x is None else round(float(x), nd)


def row(tag: str, seed: int, rc: int, run: str, load_file: Path | None = None) -> dict:
    out = {"tag": tag, "seed": seed, "rc": rc, "run_dir": Path(run).name if run else None}
    res = Path(run) / "result.json" if run else None
    if res is None or not res.is_file():
        out["outcome"] = "NO_RESULT"
        return out
    d = json.loads(res.read_text())
    lock = d.get("lockstep") or {}
    skew = d.get("clock_skew") or {}
    sim_skew = d.get("odom_sim_skew") or {}
    timeouts: dict[str, int] = {}
    for log in sorted(Path(run).glob("px4*.log")):
        for k, v in count_sensor_timeouts(log.read_text(errors="replace")).items():
            timeouts[k] = timeouts.get(k, 0) + v
    arrival = d.get("arrival_time_s")
    t0 = d.get("formation_t0_s")
    out.update(
        outcome=d.get("outcome"),
        reason=d.get("reason"),
        arrived=d.get("arrived"),
        n=d.get("n_drones"),
        collisions=d.get("collisions"),
        collision_reason=d.get("collision_reason"),
        arrival_s=_r(arrival, 3),
        formation_t0_s=_r(t0, 3),
        arrival_minus_formation_t0_s=None if arrival is None or t0 is None else _r(arrival - t0, 3),
        arrival_after_handover_s=_r(d.get("arrival_after_handover_s"), 3),
        formation_p95_m=_r(d.get("formation_error_p95_m")),
        min_inter_m=_r(d.get("min_inter_drone_distance_m")),
        min_inter_plan_m=_r(d.get("min_inter_drone_distance_plan_m")),
        min_clear_plan_m=_r(d.get("min_surface_clearance_plan_m")),
        landed=d.get("landed"),
        sim_s=_r(d.get("sim_time_s"), 3),
        wall_s=_r(d.get("wall_time_s"), 2),
        rtf=_r(d.get("rtf")),
        px4_parameters_written=d.get("px4_parameters_written"),
        sensor_timeouts=timeouts,
        sensor_timeouts_total=sum(timeouts.values()),
        lockstep=bool(lock.get("enabled")),
        lockstep_steps=lock.get("steps"),
        lockstep_handshake_timeouts=lock.get("handshake_timeouts_total"),
        lockstep_resends=lock.get("resends"),
        
        lockstep_step_wall_ms_median=_r(lock.get("step_wall_ms_median"), 2),
        lockstep_agent_wall_s=lock.get("agent_wall_s"),
        lockstep_stepping_wall_s=lock.get("stepping_wall_s"),
        odom_wait_timeouts=lock.get("odom_wait_timeouts"),
        agent_delay_s=d.get("agent_delay_s"),
        odom_stamp_spread_max_s=_r(skew.get("inter_vehicle_spread_max_s")),
        odom_stamp_spread_median_s=_r(skew.get("inter_vehicle_spread_median_s")),
        odom_sim_spread_max_s=_r(sim_skew.get("inter_vehicle_spread_max_s")),
        odom_sim_spread_p95_s=_r(sim_skew.get("inter_vehicle_spread_p95_s")),
        odom_sim_age_max_s=_r(max((v["age_max_s"] for v in (sim_skew.get("per_vehicle") or {}).values()), default=None)),
        proposals=(d.get("bus") or {}).get("proposals"),
        nacks=(d.get("bus") or {}).get("nacks"),
    )
    if load_file is not None and load_file.is_file():
        vals = [float(x) for x in load_file.read_text().split() if x.strip()]
        if vals:
            out["host_load_median"] = round(statistics.median(vals), 2)
            out["host_load_max"] = round(max(vals), 2)
    return out


def main(argv: list[str]) -> int:
    index = Path(argv[1])
    rows = []
    for line in index.read_text().splitlines():
        m = DONE.search(line)
        if not m:
            continue
        tag, seed = m["tag"], int(m["seed"])
        rows.append(row(tag, seed, int(m["rc"]), m["run"], index.parent / f"{tag}-s{seed}.load"))
    json.dump(rows, sys.stdout, indent=1, ensure_ascii=False)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
