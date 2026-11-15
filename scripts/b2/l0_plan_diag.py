#!/usr/bin/env python3
"""票 B2：L0 交叉卡上装同一个规划记录器，给 SITL 锁步做对照。只读，不改选择。

    python3 scripts/b2/l0_plan_diag.py --seeds 0 1 2 3 4 [--safe-radius 0.35] > out.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from yopo_l0 import crossing  # noqa: E402
from yopo_l0.b2_plan_diag import PlanDiag  # noqa: E402
from yopo_l0.runner import RunConfig, run  # noqa: E402
from yopo_l0.scene import load_card  # noqa: E402


def one(card, seed: int, safe_radius: float | None) -> dict:
    holder: dict = {}
    original = crossing.CrossingFleet.from_card.__func__

    def from_card(cls, card_dict, physics_dt, *, record_inputs=False):
        fleet = original(cls, card_dict, physics_dt, record_inputs=record_inputs)
        holder["diag"] = PlanDiag(fleet, card.min_distance, float(card.card["criteria"]["body_radius_m"]))
        return fleet

    crossing.CrossingFleet.from_card = classmethod(from_card)
    try:
        out = run(card, RunConfig(seed=int(seed), shadow=True, safe_radius_m=safe_radius))
    finally:
        crossing.CrossingFleet.from_card = classmethod(original)
    res = out.result
    drones = res.get("drones") or []
    return {
        "seed": int(seed),
        "safe_radius_m": safe_radius,
        "outcome": res.get("outcome"),
        "reason": res.get("reason"),
        "arrived": res.get("arrived", sum(1 for d in drones if d.get("arrival_time_s") is not None)),
        "min_obstacle_m": res.get("min_obstacle_distance_m"),
        "result_keys_min": {k: res.get(k) for k in res if "min" in k and not isinstance(res.get(k), (dict, list))},
        "b2_plan_diag": holder["diag"].summary() if "diag" in holder else None,
    }


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=str(REPO / "scenes/crossing-pillars.json"))
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    ap.add_argument("--safe-radius", type=float, default=None)
    args = ap.parse_args(argv)
    card = load_card(args.scene)
    rows = [one(card, s, args.safe_radius) for s in args.seeds]
    json.dump({"scene": card.name, "rows": rows}, sys.stdout, indent=1, ensure_ascii=False)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
