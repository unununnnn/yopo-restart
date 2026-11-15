#!/usr/bin/env python3
"""票 B2：L0 交叉卡上装同一个规划记录器，给 SITL 锁步做对照。只读，不改选择。

    python3 scripts/b2/l0_plan_diag.py --seeds 0 1 2 3 4 [--safe-radius 0.35] > out.json
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from yopo_l0 import crossing  # noqa: E402
from yopo_l0.b2_plan_diag import PlanDiag, apply_b2_overrides, b3_from_fix, fix_deviations, parse_fix  # noqa: E402
from yopo_l0.runner import RunConfig, run  # noqa: E402
from yopo_l0.scene import load_card  # noqa: E402


def one(card, seed: int, safe_radius: float | None) -> dict:
    holder: dict = {}
    original = crossing.CrossingFleet.from_card.__func__

    def from_card(cls, card_dict, physics_dt, *, record_inputs=False):
        fleet = original(cls, card_dict, physics_dt, record_inputs=record_inputs)
        holder["fleet"] = fleet
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
        "b3": dict(holder["fleet"].b3_counts) if getattr(holder.get("fleet"), "b3", None) else None,
        "arrival_time_s": res.get("arrival_time_s"),
    }


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=str(REPO / "scenes/crossing-pillars.json"))
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    ap.add_argument("--safe-radius", type=float, default=None)
    ap.add_argument("--odomhead", type=float, default=None, help="policy_params.b2.head_from_odom_m（偏离上游）")
    ap.add_argument(
        "--dynamics", default="",
        help="只在本进程里改场景卡 dynamics，如 tau_s=0.3,kp=2（对照用，偏离上游）",
    )
    ap.add_argument("--fix", default="", help="同 YOPO_B2_FIX，例如 launch=3:2.5,relat=0.4（票 B3，偏离上游）")
    args = ap.parse_args(argv)
    fix = parse_fix(args.fix)
    card, deviations = apply_b2_overrides(
        load_card(args.scene), head_from_odom_m=args.odomhead, b3=b3_from_fix(fix)
    )
    deviations += [d for d in fix_deviations(fix) if "里程计" in d or "起步" in d]
    if args.dynamics:
        dyn = dict(card.card["dynamics"])
        for item in args.dynamics.split(","):
            key, _, value = item.partition("=")
            dyn[key.strip()] = float(value)
        card = dataclasses.replace(card, card={**card.card, "dynamics": dyn})
        deviations.append(f"偏离上游：L0 动力学 {args.dynamics}")
    rows = [one(card, s, args.safe_radius) for s in args.seeds]
    json.dump({"scene": card.name, "deviations": deviations, "odomhead": args.odomhead, "rows": rows}, sys.stdout, indent=1, ensure_ascii=False)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
