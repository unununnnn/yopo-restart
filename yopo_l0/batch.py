"""批跑：python -m yopo_l0.batch CARD[@POLICY] ... [--seeds 0,1,2] [--duration S] [--jobs N] [--out out/batch]

每个 (卡, 策略) 按 seed 跑，写 result.json / ticks.jsonl，打印 Markdown 结果表；首个 seed 重跑一次做确定性检查。
判定：
- hover 任务：SUCCESS 才算通过（票 01 判据）。
- goto 任务：判据未冻结（criteria.frozen=false）→ 只报数「未冻结 / report-only」；碰撞数作为硬事实单列。
退出码：hover 全部通过、确定性一致、单次墙钟 < 上限 时为 0；goto 的到达与碰撞不影响退出码（只报数）。
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
from pathlib import Path

from .runner import WALL_FIELDS, RunConfig, run, write
from .scene import load_card


def _strip(r):
    return {k: v for k, v in r.items() if k not in WALL_FIELDS}


def _one(job):
    path, policy, seed, duration, out_dir, check, shadow = job
    card = load_card(path)
    cfg = RunConfig(seed=seed, policy=policy, duration_s=duration, shadow=shadow)
    out = run(card, cfg)
    write(out, Path(out_dir) / f"{card.name}@{policy}" / f"seed{seed}")
    same = None
    if check:
        again = run(card, cfg)
        same = _strip(again.result) == _strip(out.result) and again.ticks == out.ticks
    return out.result, same


def _fmt(x, nd=2):
    return "—" if x is None else f"{x:.{nd}f}"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("specs", nargs="+", help="场景卡路径，可带 @策略（默认 --policy）")
    ap.add_argument("--seeds", default=None, help="逗号分隔；默认取场景卡 seeds")
    ap.add_argument("--policy", default="upstream")
    ap.add_argument("--shadow", dest="shadow", action="store_true", default=True)
    ap.add_argument("--no-shadow", dest="shadow", action="store_false")
    ap.add_argument("--duration", type=float, default=None, help="覆盖仿真时长（秒）")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--no-determinism", action="store_true", help="跳过首 seed 重跑")
    ap.add_argument("--wall-limit", type=float, default=60.0)
    ap.add_argument("--out", default="out/batch")
    a = ap.parse_args(argv)
    jobs, groups = [], []
    for spec in a.specs:
        path, _, policy = spec.partition("@")
        policy = policy or a.policy
        card = load_card(path)
        seeds = [int(s) for s in a.seeds.split(",")] if a.seeds else card.card.get("seeds", [0])
        groups.append((card, policy, seeds))
        for i, seed in enumerate(seeds):
            jobs.append((path, policy, seed, a.duration, a.out, i == 0 and not a.no_determinism, a.shadow))
    if a.jobs > 1:
        with mp.get_context("spawn").Pool(a.jobs) as pool:
            res = pool.map(_one, jobs, chunksize=1)
    else:
        res = [_one(j) for j in jobs]

    rows = [r for r, _ in res]
    det = [(r["scene"], r["policy"], r["seed"], same) for r, same in res if same is not None]
    lines = [f"commit `{rows[0]['commit'][:12]}`，层级 L0", ""]
    gate_ok = True
    for card, policy, seeds in groups:
        rs = [r for r in rows if r["scene"] == card.name and r["policy"] == policy]
        task = card.card["task"]["type"]
        frozen = bool(card.card["criteria"].get("frozen", task == "hover"))
        n_col = sum(r["outcome"] == "COLLISION" for r in rs)
        walls = [r["wall_time_s"] for r in rs]
        if task == "hover":
            n_ok = sum(r["outcome"] == "SUCCESS" for r in rs)
            verdict = f"{n_ok}/{len(rs)} 通过"
            gate_ok &= n_ok == len(rs)
        else:
            arr = [r["arrival_time_s"] for r in rs if r["arrival_time_s"] is not None]
            mean_arr = _fmt(sum(arr) / len(arr)) if arr else "—"
            verdict = (f"到达 {len(arr)}/{len(rs)}，平均到达时间 {mean_arr} s，碰撞 {n_col}"
                       f"（{'判据已冻结' if frozen else '未冻结 / report-only'}）")
        gate_ok &= max(walls) < a.wall_limit or task != "hover"
        lines += [
            f"### {card.name} @ {policy}（{task}，影子盾 {'开' if rs[0]['shadow'] else '关'}）：{verdict}",
            f"墙钟 最大 {max(walls):.1f} s / 平均 {sum(walls) / len(walls):.1f} s",
            "",
            "| seed | 结果 | 到达时间 s | 最小障碍距离 m | 最小机间距离 m | 终点误差 m | 仿真 s | 墙钟 s | 规划码 |",
            "|---|---|---|---|---|---|---|---|---|",
        ]
        for r in rs:
            codes = ", ".join(f"{k}:{v}" for k, v in r["plan_codes"].items() if not k.startswith("cand:"))
            lines.append(
                f"| {r['seed']} | {r['outcome']} | {_fmt(r['arrival_time_s'])} | {_fmt(r['min_obstacle_distance_m'], 3)} | "
                f"{_fmt(r.get('min_inter_drone_distance_m'), 3)} | {_fmt(r['final_goal_error_m'], 3)} | "
                f"{r['sim_duration_s']:.1f} | {r['wall_time_s']:.1f} | {codes} |"
            )
        cand = {}
        for r in rs:
            for k, v in r["plan_codes"].items():
                if k.startswith("cand:"):
                    cand[k[5:]] = cand.get(k[5:], 0) + v
        if cand:
            lines.append("")
            lines.append("候选拒绝原因合计：" + "，".join(f"{k} {v}" for k, v in sorted(cand.items(), key=lambda x: -x[1])))
        lines.append("")
    for name, policy, s, ok in det:
        lines.append(f"确定性（同 seed 重跑，墙钟除外）：{name}@{policy} seed{s} {'一致' if ok else '不一致'}")
        gate_ok &= ok
    md = "\n".join(lines)
    Path(a.out).mkdir(parents=True, exist_ok=True)
    (Path(a.out) / "summary.md").write_text(md + "\n")
    (Path(a.out) / "results.json").write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n")
    print(md)
    return 0 if gate_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
