"""单次运行：python -m yopo_l0.run --scene scenes/g2-hover.json --seed 0"""

from __future__ import annotations

import argparse
import json

from .bus import BusFaults
from .policy import POLICIES
from .runner import RunConfig, run, write
from .scene import load_card


def _bus(a) -> BusFaults | None:
    """任一 --bus-* 出现就覆盖场景卡。未出现的旋钮为 0。"""
    if a.bus_drop is None and a.bus_duplicate is None and a.bus_delay is None and a.bus_reorder is None:
        return None
    if a.bus_delay:
        lo, hi = a.bus_delay.split(",")
        delay = (float(lo), float(hi))
    else:
        delay = (0.0, 0.0)
    return BusFaults(
        drop=0.0 if a.bus_drop is None else a.bus_drop,
        duplicate=0.0 if a.bus_duplicate is None else a.bus_duplicate,
        delay=delay,
        reorder=0.0 if a.bus_reorder is None else a.bus_reorder,
    )


def main(argv=None):
    ap = argparse.ArgumentParser(description="L0 单次运行（CPU、无界面）")
    ap.add_argument("--scene", required=True, help="场景卡 JSON")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--policy", default="upstream", choices=sorted(POLICIES))
    ap.add_argument("--shadow", dest="shadow", action="store_true", default=True)
    ap.add_argument("--no-shadow", dest="shadow", action="store_false")
    ap.add_argument("--duration", type=float, default=None, help="仿真秒数，默认取场景卡")
    ap.add_argument("--out", default=None, help="输出目录，默认 out/<scene>/seed<N>")
    ap.add_argument("--bus-drop", type=float, default=None, help="丢包概率，对齐 G3 FaultPlan.drop")
    ap.add_argument("--bus-duplicate", type=float, default=None, help="额外一份拷贝的概率，对齐 FaultPlan.duplicate")
    ap.add_argument("--bus-delay", default=None, help="时延秒数 lo,hi，对齐 FaultPlan.delay，例如 0,0.2")
    ap.add_argument("--bus-reorder", type=float, default=None, help="插到队首的概率，对齐 G3 explore 的 reorder")
    a = ap.parse_args(argv)
    card = load_card(a.scene)
    out = run(card, RunConfig(
        seed=a.seed, policy=a.policy, shadow=a.shadow, duration_s=a.duration, bus=_bus(a),
    ))
    d = write(out, a.out or f"out/{card.name}/seed{a.seed}")
    print(json.dumps(out.result, ensure_ascii=False))
    print(f"写入 {d}/result.json, {d}/ticks.jsonl")
    return 0 if out.result["outcome"] == "SUCCESS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
