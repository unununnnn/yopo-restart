#!/usr/bin/env python3
"""票 T2：同 seed 重跑差分与复现。不放宽冻结判据。"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from yopo_l0.batch import _strip
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card


def _walk_diff(a, b, prefix=""):
    if type(a) is not type(b) and not (
        isinstance(a, (int, float)) and isinstance(b, (int, float))
    ):
        yield prefix, a, b
        return
    if isinstance(a, dict):
        keys = set(a) | set(b)
        for k in sorted(keys, key=str):
            p = f"{prefix}.{k}" if prefix else str(k)
            if k not in a:
                yield p, "<missing>", b[k]
            elif k not in b:
                yield p, a[k], "<missing>"
            else:
                yield from _walk_diff(a[k], b[k], p)
        return
    if isinstance(a, (list, tuple)):
        if len(a) != len(b):
            yield prefix + ".len", len(a), len(b)
            n = min(len(a), len(b))
        else:
            n = len(a)
        for i in range(n):
            if a[i] != b[i]:
                yield from _walk_diff(a[i], b[i], f"{prefix}[{i}]")
                if i > 20 and n > 40:
                    yield prefix + ".further", f"stopped after {i}", f"n={n}"
                    break
        return
    if a != b:
        yield prefix, a, b


def _summarize(diffs, limit=40):
    rows = []
    for i, (path, left, right) in enumerate(diffs):
        if i >= limit:
            break
        def short(x):
            s = repr(x)
            return s if len(s) < 180 else s[:177] + "..."
        rows.append({"path": path, "a": short(left), "b": short(right)})
    return rows


def pair_run(card, seed: int, duration: float | None, shadow: bool):
    cfg = RunConfig(seed=seed, policy="formation" if card.card["task"]["type"] == "formation" else "upstream",
                    duration_s=duration, shadow=shadow)
    # policy name is ignored for fleets; still set reasonably
    a = run(card, cfg)
    b = run(card, cfg)
    result_same = _strip(a.result) == _strip(b.result)
    ticks_same = a.ticks == b.ticks
    result_diffs = list(_walk_diff(_strip(a.result), _strip(b.result))) if not result_same else []
    tick_diffs = list(_walk_diff(a.ticks, b.ticks)) if not ticks_same else []
    return {
        "result_same": result_same,
        "ticks_same": ticks_same,
        "same": result_same and ticks_same,
        "wall_s": [a.result.get("wall_time_s"), b.result.get("wall_time_s")],
        "commit": [a.result.get("commit"), b.result.get("commit")],
        "result_diff_n": len(result_diffs),
        "tick_diff_n": len(tick_diffs),
        "result_diffs": _summarize(result_diffs),
        "tick_diffs": _summarize(tick_diffs),
        "outcome": [a.result.get("outcome"), b.result.get("outcome")],
        "reason": [a.result.get("reason"), b.result.get("reason")],
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--card", default="scenes/formation-6-open.json")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--duration", type=float, default=0.3)
    ap.add_argument("--pairs", type=int, default=5)
    ap.add_argument("--shadow", action="store_true")
    ap.add_argument("--no-shadow", action="store_true")
    ap.add_argument("--out", default="out/t2/pairs.jsonl")
    args = ap.parse_args(argv)
    shadow = True if args.shadow else False if args.no_shadow else False
    card = load_card(ROOT / args.card)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    n_fail = 0
    with out_path.open("a") as f:
        for i in range(args.pairs):
            rec = pair_run(card, args.seed, args.duration, shadow)
            rec.update({
                "i": i,
                "card": args.card,
                "seed": args.seed,
                "duration_s": args.duration,
                "shadow": shadow,
                "loadavg": os.getloadavg(),
                "pid": os.getpid(),
            })
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            f.flush()
            status = "OK" if rec["same"] else "DIFF"
            if not rec["same"]:
                n_fail += 1
            print(
                f"[{status}] {args.card} seed={args.seed} dur={args.duration} pair={i} "
                f"wall={rec['wall_s']} result_diffs={rec['result_diff_n']} "
                f"tick_diffs={rec['tick_diff_n']} load={rec['loadavg']}",
                flush=True,
            )
            if not rec["same"]:
                for row in rec["result_diffs"][:8]:
                    print("  result", row, flush=True)
                for row in rec["tick_diffs"][:8]:
                    print("  tick", row, flush=True)
    print(f"done pairs={args.pairs} fail={n_fail}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
