"""单次运行：python -m yopo_l0.run --scene scenes/g2-hover.json --seed 0"""

from __future__ import annotations

import argparse
import json

from .policy import POLICIES
from .runner import RunConfig, run, write
from .scene import load_card


def main(argv=None):
    ap = argparse.ArgumentParser(description="L0 单次运行（CPU、无界面）")
    ap.add_argument("--scene", required=True, help="场景卡 JSON")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--policy", default="upstream", choices=sorted(POLICIES))
    ap.add_argument("--shadow", dest="shadow", action="store_true", default=True)
    ap.add_argument("--no-shadow", dest="shadow", action="store_false")
    ap.add_argument("--duration", type=float, default=None, help="仿真秒数，默认取场景卡")
    ap.add_argument("--out", default=None, help="输出目录，默认 out/<scene>/seed<N>")
    a = ap.parse_args(argv)
    card = load_card(a.scene)
    out = run(card, RunConfig(seed=a.seed, policy=a.policy, shadow=a.shadow, duration_s=a.duration))
    d = write(out, a.out or f"out/{card.name}/seed{a.seed}")
    print(json.dumps(out.result, ensure_ascii=False))
    print(f"写入 {d}/result.json, {d}/ticks.jsonl")
    return 0 if out.result["outcome"] == "SUCCESS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
