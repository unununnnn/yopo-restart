"""批跑：python -m yopo_l0.batch CARD[@POLICY] ... [--seeds 0,1,2] [--duration S] [--jobs N] [--out out/batch]

每个 (卡, 策略) 按 seed 跑，写 result.json / ticks.jsonl，打印 Markdown 结果表；首个 seed 重跑一次做确定性检查。
判定：
- hover 任务：SUCCESS 才算通过（票 01 判据）。
- goto / formation 任务：判据未冻结（criteria.frozen=false）→ 只报数「未冻结 / report-only」；碰撞数作为硬事实单列。
退出码：hover 全部通过、确定性一致、单次墙钟 < 上限 时为 0；goto 与 formation 的到达与碰撞不影响退出码（只报数）。
结果表列出场景卡哈希、seed、层级、策略、影子盾、最小机间距离。影子拒绝率读结果里的 shadow_reject_rate（关着是 null，表中写 —）。单机的机间距离为 —。
起飞交接和 RTF 是 L1 字段（票 09）；L0 结果里没有这两列，表上留空。

票 11a 在上面加两档，不改原来的表：
- ``--form inprocess|ros2``：进程内运行器，或 ROS 2 形态（``yopo_l0.ros2_run``）。默认 inprocess。
- ``--tiers none,drop-5,...`` 或 ``--tiers scenes/formation-6-tiers.json``：每个故障档位一张表，
  列是到达率、碰撞、最小机间距离、队形误差 p95、影子拒绝率、单次墙钟。
  ``--wall-cap`` 默认 120 秒。某次运行的墙钟超过上限时，把分批方案写进该次结果和 ``batching_plan.json``，
  不因此把退出码打成失败。
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
from pathlib import Path

from .bus import BusFaults
from .runner import WALL_FIELDS, RunConfig, run, write
from .scene import load_card

# 分批方案里带有当次墙钟，不能拿去比确定性。
_STRIP = WALL_FIELDS | {"batching_plan"}
_TIER_FILE = Path(__file__).resolve().parents[1] / "scenes" / "formation-6-tiers.json"


def _strip(r):
    return {k: v for k, v in r.items() if k not in _STRIP}


def _fmt(x, nd=2):
    return "—" if x is None else f"{x:.{nd}f}"


def _reject_summary(r):
    hist = r.get("shadow_reject_histogram")
    if not hist:
        return "—"
    top = sorted(hist.items(), key=lambda kv: (-kv[1], kv[0]))[:3]
    return ", ".join(f"{k}:{v}" for k, v in top)


def _read_tier_file(path: Path) -> list[dict]:
    raw = json.loads(path.read_text())
    tiers = raw["tiers"] if isinstance(raw, dict) else raw
    if not tiers:
        raise SystemExit(f"档位清单是空的：{path}")
    ids = [str(t["id"]) for t in tiers]
    if len(ids) != len(set(ids)):
        raise SystemExit(f"档位 id 重复：{path}")
    return list(tiers)


def resolve_tiers(text: str) -> list[dict]:
    """``all``、逗号分隔的 id，或一份档位 JSON。id 从 scenes/formation-6-tiers.json 查。"""
    raw = text.strip()
    path = Path(raw)
    if raw.endswith(".json") or path.is_file():
        found = path if path.is_file() else _TIER_FILE.parent / raw
        if not found.is_file():
            found = _TIER_FILE.parents[0] / raw
        if not found.is_file():
            raise SystemExit(f"找不到档位清单 {raw}")
        return _read_tier_file(found)
    catalog = {str(t["id"]): t for t in _read_tier_file(_TIER_FILE)}
    if raw in ("all", "*"):
        return list(catalog.values())
    ids = [part.strip() for part in raw.split(",") if part.strip()]
    if not ids:
        raise SystemExit("`--tiers` 是空的")
    missing = [i for i in ids if i not in catalog]
    if missing:
        known = ", ".join(catalog)
        raise SystemExit(f"未知档位 {', '.join(missing)}。已知：{known}。清单：{_TIER_FILE.name}")
    return [catalog[i] for i in ids]


def _arrival_rate(r: dict):
    if r.get("arrival_rate") is not None:
        return r["arrival_rate"]
    n = r.get("n_drones") or 0
    if not n or r.get("arrived") is None:
        return None
    return r["arrived"] / n


def _collisions(r: dict) -> int:
    return 1 if r.get("outcome") == "COLLISION" else 0


def _over_cap_plan(result: dict, wall_cap: float, scene_path: str) -> dict:
    form = result.get("form") or "inprocess"
    tier = result.get("fault_tier")
    seed = result["seed"]
    policy = result.get("policy") or "formation"
    cmd = (
        f"python -m yopo_l0.batch {scene_path}@{policy} --form {form} "
        f"--tiers {tier} --seeds {seed} --wall-cap {wall_cap:g} --jobs 1 --no-determinism"
    )
    return {
        "wall_cap_s": wall_cap,
        "scene": result["scene"],
        "scene_path": scene_path,
        "fault_tier": tier,
        "seed": seed,
        "form": form,
        "wall_time_s": result["wall_time_s"],
        "split": "按 seed 分批，每次只跑这一个 seed。ROS 2 形态保持 --jobs 1。",
        "if_still_over": (
            "若这一条仍超过墙钟上限，把场景卡 rates_hz.camera 从 30 降到 10，"
            "或把 task.duration_s 收到到达所需时长，再单 seed 重跑。"
            "超限档位不要放进 scenes/suite.txt，改放到 scripts/nightly.sh。"
        ),
        "commands": [cmd],
    }


def _aggregate_plan(plans: list[dict], wall_cap: float) -> dict:
    return {
        "wall_cap_s": wall_cap,
        "exceeded": [
            {
                "scene": p["scene"],
                "scene_path": p["scene_path"],
                "fault_tier": p["fault_tier"],
                "seed": p["seed"],
                "form": p["form"],
                "wall_time_s": p["wall_time_s"],
            }
            for p in plans
        ],
        "split": "按 seed 分批，每次只跑一个 seed。ROS 2 形态保持 --jobs 1，不要并行起多份 rclpy。",
        "if_still_over": plans[0]["if_still_over"],
        "commands": [cmd for p in plans for cmd in p["commands"]],
    }


def _one(job):
    path, policy, seed, duration, out_dir, check, shadow, form, bus, tier_id, wall_cap = job
    card = load_card(path)
    cfg = RunConfig(seed=seed, policy=policy, duration_s=duration, shadow=shadow, bus=bus)
    if form == "ros2":
        from .ros2_run import execute

        out = execute(card, cfg)
    else:
        out = run(card, cfg)
    # 档位标签和分批方案是批跑后写上的，不参与确定性比较。
    same = None
    if check:
        if form == "ros2":
            from .ros2_run import execute

            again = execute(card, cfg)
        else:
            again = run(card, cfg)
        same = _strip(again.result) == _strip(out.result) and again.ticks == out.ticks
    if tier_id is not None:
        out.result["form"] = "ros2" if form == "ros2" else "inprocess"
        out.result["fault_tier"] = tier_id
        if wall_cap is not None and out.result["wall_time_s"] > wall_cap:
            out.result["batching_plan"] = _over_cap_plan(out.result, wall_cap, path)
    if tier_id is None:
        dest = Path(out_dir) / f"{card.name}@{policy}" / f"seed{seed}"
    else:
        tag = "" if form == "inprocess" else "@ros2"
        dest = Path(out_dir) / f"{card.name}@{policy}{tag}" / tier_id / f"seed{seed}"
    write(out, dest)
    return out.result, same


def _hovering_without_commit(r: dict) -> bool:
    commits = (r.get("formation") or {}).get("local_commits") or {}
    return any(v == 0 for v in commits.values())


def _tier_section(card, policy: str, tier: dict, rows: list[dict], form: str, wall_cap: float) -> list[str]:
    label = tier.get("label") or tier["id"]
    task = card.card["task"]["type"]
    frozen = bool(card.card["criteria"].get("frozen", task == "hover"))
    verdict = "判据已冻结" if frozen else "未冻结 / report-only"
    form_word = "ROS 2" if form == "ros2" else "进程内"
    rates = [_arrival_rate(r) for r in rows]
    known = [x for x in rates if x is not None]
    dists = [r.get("min_inter_drone_distance_m") for r in rows]
    known_d = [x for x in dists if x is not None]
    p95s = [r.get("formation_error_p95_m") for r in rows]
    known_p = [x for x in p95s if x is not None]
    shadows = [r.get("shadow_reject_rate") for r in rows]
    known_s = [x for x in shadows if x is not None]
    walls = [r["wall_time_s"] for r in rows]
    n_col = sum(_collisions(r) for r in rows)
    mean_rate = sum(known) / len(known) if known else None
    lines = [
        f"### {card.name} @ {policy} / {label}（形态 {form_word}，{verdict}，"
        f"墙钟上限 {wall_cap:g} s，场景卡 `{card.hash}`）",
        (
            f"档位合计：到达率 {_fmt(mean_rate, 3)}，碰撞 {n_col}，"
            f"最小机间距离 {_fmt(min(known_d) if known_d else None, 3)} m，"
            f"队形误差 p95 最大 {_fmt(max(known_p) if known_p else None, 3)} m，"
            f"影子拒绝率均值 {_fmt(sum(known_s) / len(known_s) if known_s else None, 3)}，"
            f"单次墙钟最大 {max(walls):.1f} s"
        ),
        "",
        "| seed | 到达率 | 碰撞 | 最小机间距离 m | 队形误差 p95 m | 影子拒绝率 | 单次墙钟 s |",
        "|---|---|---|---|---|---|---|",
    ]
    for r, rate in zip(rows, rates):
        lines.append(
            f"| {r['seed']} | {_fmt(rate, 3)} | {_collisions(r)} | "
            f"{_fmt(r.get('min_inter_drone_distance_m'), 3)} | {_fmt(r.get('formation_error_p95_m'), 3)} | "
            f"{_fmt(r.get('shadow_reject_rate'), 3)} | {r['wall_time_s']:.1f} |"
        )
    lines.append(
        f"| 合计 | {_fmt(mean_rate, 3)} | {n_col} | "
        f"{_fmt(min(known_d) if known_d else None, 3)} | {_fmt(max(known_p) if known_p else None, 3)} | "
        f"{_fmt(sum(known_s) / len(known_s) if known_s else None, 3)} | {max(walls):.1f} |"
    )
    if any(_hovering_without_commit(r) for r in rows):
        lines.append("")
        lines.append(
            "有飞机的 `local_commits` 仍为 0，没有飞出起点，队形误差因此变大。"
            "06b 会在后续窗口沿本槽追赶；这一档里这些飞机没有形成提交。"
        )
    over = [r for r in rows if r.get("batching_plan")]
    if over:
        lines.append("")
        lines.append(
            f"墙钟超过 {wall_cap:g} s 的 seed：{', '.join(str(r['seed']) for r in over)}。"
            "分批方案已写入该次 `result.json` 的 `batching_plan`，批跑不因此失败。"
        )
    lines.append("")
    return lines


def _legacy_section(card, policy: str, rs: list[dict], wall_limit: float) -> tuple[list[str], bool]:
    task = card.card["task"]["type"]
    frozen = bool(card.card["criteria"].get("frozen", task == "hover"))
    n_col = sum(r["outcome"] == "COLLISION" for r in rs)
    walls = [r["wall_time_s"] for r in rs]
    gate_ok = True
    if task == "hover":
        n_ok = sum(r["outcome"] == "SUCCESS" for r in rs)
        verdict = f"{n_ok}/{len(rs)} 通过"
        gate_ok &= n_ok == len(rs)
    else:
        arr = [r["arrival_time_s"] for r in rs if r["arrival_time_s"] is not None]
        mean_arr = _fmt(sum(arr) / len(arr)) if arr else "—"
        verdict = (
            f"到达 {len(arr)}/{len(rs)}，平均到达时间 {mean_arr} s，碰撞 {n_col}"
            f"（{'判据已冻结' if frozen else '未冻结 / report-only'}）"
        )
    gate_ok &= max(walls) < wall_limit or task != "hover"
    lines = [
        f"### {card.name} @ {policy}（{task}，层级 {rs[0]['tier']}，影子盾 {'开' if rs[0]['shadow'] else '关'}，"
        f"场景卡 `{card.hash}`）：{verdict}",
        f"墙钟 最大 {max(walls):.1f} s / 平均 {sum(walls) / len(walls):.1f} s",
        "",
        "| seed | 层级 | 策略 | 影子盾 | 场景卡哈希 | 结果 | 到达时间 s | 最小障碍距离 m | 最小机间距离 m | 终点误差 m | 仿真 s | 墙钟 s | 影子拒绝率 | 主要拒绝码 | 规划码 | 起飞交接 s | RTF |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rs:
        codes = ", ".join(f"{k}:{v}" for k, v in r["plan_codes"].items() if not k.startswith("cand:"))
        lines.append(
            f"| {r['seed']} | {r['tier']} | {r['policy']} | {'开' if r['shadow'] else '关'} | `{r['scene_card_hash']}` | "
            f"{r['outcome']} | {_fmt(r['arrival_time_s'])} | {_fmt(r['min_obstacle_distance_m'], 3)} | "
            f"{_fmt(r.get('min_inter_drone_distance_m'), 3)} | {_fmt(r['final_goal_error_m'], 3)} | "
            f"{r['sim_duration_s']:.1f} | {r['wall_time_s']:.1f} | "
            f"{_fmt(r.get('shadow_reject_rate'), 3)} | {_reject_summary(r)} | {codes} | "
            f"{_fmt(r.get('takeoff_handoff_s'), 3)} | {_fmt(r.get('rtf'), 3)} |"
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
    return lines, gate_ok


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
    ap.add_argument("--wall-limit", type=float, default=60.0, help="hover 任务的单次墙钟上限，超过则失败")
    ap.add_argument("--form", choices=("inprocess", "ros2"), default="inprocess",
                    help="inprocess：进程内。ros2：每机一个 ROS 2 节点")
    ap.add_argument("--tiers", default=None,
                    help="故障档位：逗号分隔的 id、all，或档位 JSON。默认清单 scenes/formation-6-tiers.json")
    ap.add_argument("--wall-cap", type=float, default=120.0,
                    help="档位批跑的单次墙钟上限。超过则写入分批方案，不因此失败")
    ap.add_argument("--out", default="out/batch")
    a = ap.parse_args(argv)
    tiers = resolve_tiers(a.tiers) if a.tiers else None
    jobs = []
    groups = []
    for spec in a.specs:
        path, _, rest = spec.partition("@")
        policy, _, seedpart = rest.partition("@")
        policy = policy or a.policy
        card = load_card(path)
        if tiers is not None and card.card["task"]["type"] == "formation":
            policy = "formation"
        if a.seeds:
            seeds = [int(s) for s in a.seeds.split(",")]
        elif seedpart:
            seeds = [int(s) for s in seedpart.split(",") if s != ""]
        else:
            seeds = card.card.get("seeds", [0])
        if tiers is None:
            groups.append((card, policy, None, seeds))
            for i, seed in enumerate(seeds):
                jobs.append((
                    path, policy, seed, a.duration, a.out,
                    i == 0 and not a.no_determinism, a.shadow,
                    a.form, None, None, None,
                ))
        else:
            for tier in tiers:
                groups.append((card, policy, tier, seeds))
                bus = BusFaults.from_mapping(tier.get("bus"))
                for i, seed in enumerate(seeds):
                    jobs.append((
                        path, policy, seed, a.duration, a.out,
                        i == 0 and not a.no_determinism, a.shadow,
                        a.form, bus, str(tier["id"]), a.wall_cap,
                    ))
    if a.form == "ros2" or a.jobs <= 1:
        res = [_one(j) for j in jobs]
    else:
        with mp.get_context("spawn").Pool(a.jobs) as pool:
            res = pool.map(_one, jobs, chunksize=1)

    rows = [r for r, _ in res]
    det = [(r["scene"], r["policy"], r.get("fault_tier"), r["seed"], same) for r, same in res if same is not None]
    lines = [f"commit `{rows[0]['commit'][:12]}`，层级 {rows[0]['tier']}", ""]
    if a.form == "ros2":
        lines.append("形态 ROS 2。单进程顺序执行，避免多个 rclpy 抢同一个 DDS 域。")
        lines.append("")
    gate_ok = True
    plans: list[dict] = []
    if tiers is None:
        for card, policy, _tier, seeds in groups:
            rs = [r for r in rows if r["scene"] == card.name and r["policy"] == policy]
            section, ok = _legacy_section(card, policy, rs, a.wall_limit)
            lines += section
            gate_ok &= ok
    else:
        for card, policy, tier, seeds in groups:
            rs = [
                r for r in rows
                if r["scene"] == card.name and r["policy"] == policy and r.get("fault_tier") == tier["id"]
            ]
            lines += _tier_section(card, policy, tier, rs, a.form, a.wall_cap)
            plans.extend(r["batching_plan"] for r in rs if r.get("batching_plan"))
            if card.card["task"]["type"] == "hover":
                n_ok = sum(r["outcome"] == "SUCCESS" for r in rs)
                gate_ok &= n_ok == len(rs)
                gate_ok &= max(r["wall_time_s"] for r in rs) < a.wall_limit
    for name, policy, tier, s, ok in det:
        tag = f"/{tier}" if tier else ""
        lines.append(f"确定性（同 seed 重跑，墙钟除外）：{name}@{policy}{tag} seed{s} {'一致' if ok else '不一致'}")
        gate_ok &= ok
    out_dir = Path(a.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    if plans:
        plan = _aggregate_plan(plans, a.wall_cap)
        (out_dir / "batching_plan.json").write_text(json.dumps(plan, indent=2, ensure_ascii=False) + "\n")
        lines += [
            "",
            f"### 分批方案（单次墙钟超过 {a.wall_cap:g} s，不因此失败）",
            "",
            plan["split"],
            plan["if_still_over"],
            "",
        ]
        for cmd in plan["commands"]:
            lines.append(f"- `{cmd}`")
        lines.append("")
    if any(r.get("task") == "crossing" for r in rows):
        from .crossing import format_report

        lines += ["", format_report(rows)]
    if any(r.get("task") == "switch" for r in rows):
        from .switch import format_report as switch_report

        lines += ["", switch_report(rows)]
    md = "\n".join(lines)
    (out_dir / "summary.md").write_text(md + "\n")
    (out_dir / "results.json").write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n")
    print(md)
    return 0 if gate_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
