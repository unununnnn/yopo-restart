"""批跑：python -m yopo_l0.batch scenes/g2-hover.json [--seeds 0,1,2] [--out out/batch]

每张卡按 seed 跑，写 result.json / ticks.jsonl，打印 Markdown 结果表；每张卡的首个 seed 重跑一次做确定性检查。
退出码：全部 SUCCESS 且确定性一致为 0。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .runner import WALL_FIELDS, RunConfig, run, write
from .scene import load_card


def _strip(r):
    return {k: v for k, v in r.items() if k not in WALL_FIELDS}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("cards", nargs="+")
    ap.add_argument("--seeds", default=None, help="逗号分隔；默认取场景卡 seeds")
    ap.add_argument("--policy", default="hover")
    ap.add_argument("--out", default="out/batch")
    a = ap.parse_args(argv)
    rows, det = [], []
    for path in a.cards:
        card = load_card(path)
        seeds = [int(s) for s in a.seeds.split(",")] if a.seeds else card.card.get("seeds", [0])
        for i, seed in enumerate(seeds):
            cfg = RunConfig(seed=seed, policy=a.policy)
            out = run(card, cfg)
            write(out, Path(a.out) / card.name / f"seed{seed}")
            r = out.result
            rows.append(r)
            if i == 0:
                again = run(card, cfg)
                same = _strip(again.result) == _strip(r) and again.ticks == out.ticks
                det.append((card.name, seed, same))
    n_ok = sum(r["outcome"] == "SUCCESS" for r in rows)
    lines = [
        f"**L0 回归**：{n_ok}/{len(rows)} 通过（{100 * n_ok / max(len(rows), 1):.0f}%），commit `{rows[0]['commit'][:12]}`",
        "",
        "| 场景 | seed | 层级 | 策略 | 影子盾 | 结果 | 最小障碍距离 m | 最大悬停误差 m | 仿真 s | 墙钟 s |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        lines.append(
            f"| {r['scene']} | {r['seed']} | {r['tier']} | {r['policy']} | {'开' if r['shadow'] else '关'} | {r['outcome']} | "
            f"{r['min_obstacle_distance_m']:.3f} | {r['max_hover_error_m']:.4f} | {r['sim_duration_s']:.1f} | {r['wall_time_s']:.2f} |"
        )
    walls = [r["wall_time_s"] for r in rows]
    lines += ["", f"墙钟：最大 {max(walls):.2f} s，平均 {sum(walls) / len(walls):.2f} s（上限 60 s）"]
    lines += [f"确定性（同 seed 重跑，墙钟字段除外）：{name} seed{s} {'一致' if ok else '不一致'}" for name, s, ok in det]
    md = "\n".join(lines)
    Path(a.out).mkdir(parents=True, exist_ok=True)
    (Path(a.out) / "summary.md").write_text(md + "\n")
    (Path(a.out) / "results.json").write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n")
    print(md)
    ok = n_ok == len(rows) and all(x for *_, x in det) and max(walls) < 60
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
