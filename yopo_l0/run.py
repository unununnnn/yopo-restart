"""单次运行：python -m yopo_l0.run --scene scenes/g2-hover.json --seed 0"""

from __future__ import annotations

import argparse
import json

from .bus import BusFaults
from .policy import POLICIES
from .runner import RunConfig, run, write
from .scene import load_card


def _normalize_shadow(argv):
    """``--shadow`` / ``--shadow on`` 开，``--shadow off`` 关。裸 ``--shadow`` 不吞掉下一个参数。"""
    if argv is None:
        return None
    out = []
    i = 0
    while i < len(argv):
        if argv[i] == "--shadow" and i + 1 < len(argv) and argv[i + 1] in ("on", "off"):
            out.extend(("--shadow", argv[i + 1]))
            i += 2
        elif argv[i] == "--shadow":
            out.extend(("--shadow", "on"))
            i += 1
        else:
            out.append(argv[i])
            i += 1
    return out


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
    ap.add_argument("--shadow", choices=("on", "off"), default="on",
                    help="影子盾记录：on（默认）或 off。只记录会否拒绝，不改批准动作")
    ap.add_argument("--no-shadow", action="store_true", help="同 --shadow off")
    ap.add_argument("--duration", type=float, default=None, help="仿真秒数，默认取场景卡")
    ap.add_argument("--out", default=None, help="输出目录，默认 out/<scene>/seed<N>")
    ap.add_argument("--bus-drop", type=float, default=None, help="丢包概率，对齐 G3 FaultPlan.drop")
    ap.add_argument("--bus-duplicate", type=float, default=None, help="额外一份拷贝的概率，对齐 FaultPlan.duplicate")
    ap.add_argument("--bus-delay", default=None, help="时延秒数 lo,hi，对齐 FaultPlan.delay，例如 0,0.2")
    ap.add_argument("--bus-reorder", type=float, default=None, help="插到队首的概率，对齐 G3 explore 的 reorder")
    ap.add_argument(
        "--safe-radius", type=float, default=None,
        help="覆盖 upstream / crossing 的走廊准入半径（米）。缺省用场景卡，场景卡再缺省 0.05（上游 d5e1e60）",
    )
    ap.add_argument(
        "--geometric-fallback", dest="geometric_fallback", action="store_const", const=True, default=None,
        help="打开提交前的一次性几何回退。缺省跟场景卡，场景卡再缺省关闭",
    )
    ap.add_argument(
        "--no-geometric-fallback", dest="geometric_fallback", action="store_const", const=False,
        help="关掉几何回退，即使场景卡写了 enabled",
    )
    ap.add_argument(
        "--shadow-env-evidence", action="store_true",
        help="研究开关，默认关。打开后用场景卡几何的光线投射给影子盾补环境证据，只记录，不改批准动作。打开即偏离上游",
    )
    a = ap.parse_args(_normalize_shadow(argv))
    card = load_card(a.scene)
    out = run(card, RunConfig(
        seed=a.seed,
        policy=a.policy,
        shadow=a.shadow == "on" and not a.no_shadow,
        duration_s=a.duration,
        bus=_bus(a),
        safe_radius_m=a.safe_radius,
        geometric_fallback=a.geometric_fallback,
        shadow_env_evidence=bool(a.shadow_env_evidence),
    ))
    d = write(out, a.out or f"out/{card.name}/seed{a.seed}")
    print(json.dumps(out.result, ensure_ascii=False))
    print(f"写入 {d}/result.json, {d}/ticks.jsonl")
    return 0 if out.result["outcome"] == "SUCCESS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
