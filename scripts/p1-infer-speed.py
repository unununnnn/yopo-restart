"""打印一个场景、一个 seed 的 plan_wall，以及轨迹位置的 sha256。

加速开关靠环境变量，不改场景卡、不改冻结门：

    YOPO_INFER_ACCEL=baseline python scripts/p1-infer-speed.py scenes/g2-pillar.json --seed 0
    YOPO_INFER_ACCEL=ort python scripts/p1-infer-speed.py scenes/g2-pillar.json --seed 0

位置摘要用来对「选中的轨迹」做前后比较。整段最大绝对差要另跑两次再减，
本脚本只负责把同一种开关下的墙钟和位置钉下来。
"""

from __future__ import annotations

import argparse
import hashlib
import json

import numpy as np

from yopo_l0.infer_accel import accel_name, accel_threads
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("scene")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--policy", default="upstream")
    args = ap.parse_args()
    card = load_card(args.scene)
    out = run(card, RunConfig(seed=args.seed, policy=args.policy))
    pos = np.asarray([row["p"] for row in out.ticks], dtype=np.float64)
    digest = hashlib.sha256(np.ascontiguousarray(pos).tobytes()).hexdigest()
    wall = out.result["plan_wall_ms"]
    report = {
        "scene": card.name,
        "seed": args.seed,
        "policy": out.result["policy"],
        "accel": accel_name(),
        "threads": accel_threads(),
        "outcome": out.result["outcome"],
        "arrival_time_s": out.result["arrival_time_s"],
        "ticks": len(out.ticks),
        "pos_sha256": digest,
        "plan_wall_ms": wall,
    }
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
