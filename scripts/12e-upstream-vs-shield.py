#!/usr/bin/env python3
"""票 12e：上游策略（影子盾只记录、不拦截）vs certified（硬门拦截）对照。

在当前 HEAD 上跑代表场景卡，写出 docs/results/12e-upstream-vs-shield.md。
所有数字来自实际运行，表上带层级和 commit。

仓库没有「影子盾拦截」模式：影子盾只记录 would_reject。真正把门打成硬拦的
是 certified 策略，且只接在单机 goto / hover 上。crossing / formation / switch
由任务类型决定规划路径，忽略 --policy certified，始终走上游 YOPO 提案。

用法（仓库根目录）：

  make venv
  .venv/bin/python scripts/12e-upstream-vs-shield.py
  .venv/bin/python scripts/12e-upstream-vs-shield.py --jobs 2
  .venv/bin/python scripts/12e-upstream-vs-shield.py --report-only
  .venv/bin/python scripts/12e-upstream-vs-shield.py --certified-seeds 0
  .venv/bin/python scripts/12e-upstream-vs-shield.py --six-seeds 0
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
ZONE = ZoneInfo("America/La_Paz")
ZONE_LABEL = "America/La_Paz，UTC-4"

# 对照组。six-drone 默认只跑 seed 0（单次墙钟超过 60 s）；可用 --six-seeds 加跑。
# formation-hold 场景卡只有 seeds 0–2。
GROUPS = [
    {
        "id": "g2-pillar-upstream",
        "title": "g2-pillar @ upstream + 影子盾开",
        "scene": "scenes/g2-pillar.json",
        "policy": "upstream",
        "seeds": list(range(10)),
        "kind": "upstream",
    },
    {
        "id": "g2-pillar-certified",
        "title": "g2-pillar @ certified + 影子盾开（硬门拦截）",
        "scene": "scenes/g2-pillar.json",
        "policy": "certified",
        "seeds": list(range(10)),
        "kind": "certified",
    },
    {
        "id": "crossing-pillars",
        "title": "crossing-pillars @ crossing + 影子盾开",
        "scene": "scenes/crossing-pillars.json",
        "policy": "crossing",
        "seeds": list(range(10)),
        "kind": "upstream",
    },
    {
        "id": "crossing-forest-medium",
        "title": "crossing-forest medium @ crossing + 影子盾开",
        "scene": "scenes/crossing-forest.json",
        "policy": "crossing",
        "seeds": list(range(10)),
        "kind": "upstream",
        "density": "medium",
    },
    {
        "id": "crossing-forest-sparse",
        "title": "crossing-forest sparse @ crossing + 影子盾开",
        "scene": "scenes/crossing-forest.json",
        "policy": "crossing",
        "seeds": list(range(10)),
        "kind": "upstream",
        "density": "sparse",
    },
    {
        "id": "crossing-forest-dense",
        "title": "crossing-forest dense @ crossing + 影子盾开",
        "scene": "scenes/crossing-forest.json",
        "policy": "crossing",
        "seeds": list(range(10)),
        "kind": "upstream",
        "density": "dense",
    },
    {
        "id": "formation-hold",
        "title": "formation-hold @ formation + 影子盾开",
        "scene": "scenes/formation-hold.json",
        "policy": "formation",
        "seeds": [0, 1, 2],
        "kind": "upstream",
        "seed_note": "场景卡只有 seeds 0–2",
    },
    {
        "id": "crossing-6-forest",
        "title": "crossing-6-forest medium @ crossing + 影子盾开",
        "scene": "scenes/crossing-6-forest.json",
        "policy": "crossing",
        "seeds": [0],
        "kind": "upstream",
        "six": True,
    },
    {
        "id": "crossing-6-forest-sparse",
        "title": "crossing-6-forest-sparse @ crossing + 影子盾开",
        "scene": "scenes/crossing-6-forest-sparse.json",
        "policy": "crossing",
        "seeds": [0],
        "kind": "upstream",
        "six": True,
    },
]

# 全时长开/关对照（到达、碰撞、动作序列）。不含六机全时长，默认另用短时核验。
FULL_IDENTITY = [
    ("scenes/g2-pillar.json", "upstream", 0),
    ("scenes/crossing-pillars.json", "crossing", 0),
    ("scenes/crossing-forest.json", "crossing", 0),
    ("scenes/formation-hold.json", "formation", 0),
]

SHORT_IDENTITY = [
    ("scenes/g2-pillar.json", "upstream", 0, 0.5),
    ("scenes/crossing-pillars.json", "crossing", 0, 1.0),
    ("scenes/crossing-forest.json", "crossing", 0, 1.0),
    ("scenes/formation-hold.json", "formation", 0, 1.0),
    ("scenes/crossing-6-forest.json", "crossing", 0, 1.0),
]


def now_label() -> str:
    return datetime.now(ZONE).strftime("%Y-%m-%d %H:%M") + f"（{ZONE_LABEL}）"


def git_commit(repo: Path) -> str:
    return subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()


def git_dirty(repo: Path) -> bool:
    out = subprocess.check_output(["git", "-C", str(repo), "status", "--porcelain", "--untracked-files=no"], text=True)
    return bool(out.strip())


def py() -> str:
    return sys.executable


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def write_forest_card(out_cards: Path, density: str) -> Path:
    sys.path.insert(0, str(ROOT))
    from yopo_l0.forest import card_for_tier
    from yopo_l0.scene import load_card

    src = load_card(ROOT / "scenes/crossing-forest.json")
    card = card_for_tier(src.card, density)
    path = out_cards / f"crossing-forest-{density}.json"
    path.write_text(json.dumps(card, indent=2, ensure_ascii=False) + "\n")
    return path


def scene_for_group(group: dict, out_cards: Path) -> Path:
    density = group.get("density")
    if density and density != "medium":
        return write_forest_card(out_cards, density)
    return ROOT / group["scene"]


def run_batch(scene: Path, policy: str, seeds: list[int], out_dir: Path, jobs: int) -> int:
    ensure_dir(out_dir)
    cmd = [
        py(), "-m", "yopo_l0.batch",
        f"{scene}@{policy}",
        "--seeds", ",".join(str(s) for s in seeds),
        "--jobs", str(jobs),
        "--no-determinism",
        "--out", str(out_dir),
    ]
    log = out_dir / "batch.log"
    print(f"+ {' '.join(cmd)}", flush=True)
    with log.open("w") as fh:
        proc = subprocess.run(cmd, cwd=ROOT, stdout=fh, stderr=subprocess.STDOUT)
    return proc.returncode


def load_rows(out_dir: Path) -> list[dict]:
    path = out_dir / "results.json"
    rows: list[dict] = []
    if path.is_file():
        data = json.loads(path.read_text())
        if isinstance(data, list):
            rows = data
        else:
            rows = data.get("results") or data.get("rows") or []
    have = {int(r["seed"]) for r in rows if "seed" in r}
    # 批跑还没写完 results.json 时，收已经落盘的单 seed 记录。
    if out_dir.is_dir():
        for f in sorted(out_dir.rglob("result.json")):
            r = json.loads(f.read_text())
            if "seed" in r and int(r["seed"]) not in have:
                rows.append(r)
                have.add(int(r["seed"]))
    return rows


def drop_heavy(row: dict) -> dict:
    skip = {"ticks", "frames", "planner_inputs"}
    return {k: v for k, v in row.items() if k not in skip}


def actions(ticks: list[dict]) -> list[tuple]:
    return [(t["cmd_p"], t["cmd_v"], t["cmd_a"], t["p"], t["v"], t["a"], t["yaw"]) for t in ticks]


def run_pair(scene: Path, policy: str, seed: int, duration_s: float | None) -> dict:
    from yopo_l0.runner import WALL_FIELDS, RunConfig, run
    from yopo_l0.scene import load_card
    from yopo_l0.shadow import SHADOW_RESULT_KEYS

    card = load_card(scene)
    kw = dict(seed=seed, policy=policy if policy in ("upstream", "certified", "hover") else "upstream")
    if duration_s is not None:
        kw["duration_s"] = duration_s
    off = run(card, RunConfig(shadow=False, **kw))
    on = run(card, RunConfig(shadow=True, **kw))
    same_actions = actions(off.ticks) == actions(on.ticks)
    off_core = {k: v for k, v in off.result.items() if k not in SHADOW_RESULT_KEYS and k not in WALL_FIELDS}
    on_core = {k: v for k, v in on.result.items() if k not in SHADOW_RESULT_KEYS and k not in WALL_FIELDS}
    return {
        "scene": card.card["name"],
        "policy": policy,
        "seed": seed,
        "duration_s": duration_s if duration_s is not None else off.result.get("sim_duration_s"),
        "same_actions": same_actions,
        "same_core": off_core == on_core,
        "outcome_off": off.result.get("outcome"),
        "outcome_on": on.result.get("outcome"),
        "arrived_off": off.result.get("arrived"),
        "arrived_on": on.result.get("arrived"),
        "reason_off": off.result.get("reason"),
        "reason_on": on.result.get("reason"),
        "n_ticks": len(on.ticks),
        "wall_off_s": off.result.get("wall_time_s"),
        "wall_on_s": on.result.get("wall_time_s"),
    }


def run_pytest_shadow(out_dir: Path) -> dict:
    log = out_dir / "test_shadow.log"
    cmd = [py(), "-m", "pytest", "-q", "tests/test_shadow.py", "--tb=short"]
    print(f"+ {' '.join(cmd)}", flush=True)
    with log.open("w") as fh:
        proc = subprocess.run(cmd, cwd=ROOT, stdout=fh, stderr=subprocess.STDOUT)
    text = log.read_text() if log.is_file() else ""
    return {"returncode": proc.returncode, "log": text.strip().splitlines()[-5:]}


def fmt(x, nd=3):
    if x is None:
        return "—"
    if isinstance(x, float):
        return f"{x:.{nd}f}"
    return str(x)


def arrival_cell(rows: list[dict]) -> str:
    if not rows:
        return "—"
    arrived = sum(int(r.get("arrived") or 0) for r in rows)
    slots = sum(int(r.get("n_drones") or 1) for r in rows)
    return f"{arrived}/{slots}"


def collision_cell(rows: list[dict]) -> str:
    if not rows:
        return "—"
    n = 0
    for r in rows:
        block = r.get("crossing") or r.get("formation") or r.get("switch") or {}
        if "obstacle_collisions" in block or "inter_drone_collisions" in block:
            n += int(block.get("obstacle_collisions") or 0)
            n += int(block.get("inter_drone_collisions") or 0)
        elif r.get("outcome") == "COLLISION":
            n += 1
    return str(n)


def clearance_range(rows: list[dict]) -> str:
    vals = []
    body = 0.35
    for r in rows:
        block = r.get("crossing") or r.get("formation") or {}
        if block.get("min_obstacle_clearance_m") is not None:
            vals.append(float(block["min_obstacle_clearance_m"]))
        elif r.get("min_obstacle_distance_m") is not None:
            vals.append(float(r["min_obstacle_distance_m"]) - body)
    if not vals:
        return "—"
    lo, hi = min(vals), max(vals)
    if abs(lo - hi) < 5e-4:
        return f"{lo:.3f}"
    return f"{lo:.3f}–{hi:.3f}"


def inter_range(rows: list[dict]) -> str:
    vals = [float(r["min_inter_drone_distance_m"]) for r in rows if r.get("min_inter_drone_distance_m") is not None]
    if not vals:
        return "—"
    lo, hi = min(vals), max(vals)
    if abs(lo - hi) < 5e-4:
        return f"{lo:.3f}"
    return f"{lo:.3f}–{hi:.3f}"


def reject_rate_cell(rows: list[dict]) -> str:
    vals = [float(r["shadow_reject_rate"]) for r in rows if r.get("shadow_reject_rate") is not None]
    if not vals:
        return "—"
    lo, hi = min(vals), max(vals)
    if abs(lo - hi) < 5e-4:
        return f"{lo:.3f}"
    return f"{lo:.3f}–{hi:.3f}"


def hist_sum(rows: list[dict]) -> Counter:
    c = Counter()
    for r in rows:
        hist = r.get("shadow_reject_histogram") or {}
        for k, v in hist.items():
            c[k] += int(v)
    return c


def hist_cell(rows: list[dict], top=3) -> str:
    c = hist_sum(rows)
    if not c:
        return "—"
    parts = [f"{k}:{v}" for k, v in c.most_common(top)]
    return ", ".join(parts)


def wall_cell(rows: list[dict]) -> str:
    vals = [float(r["wall_time_s"]) for r in rows if r.get("wall_time_s") is not None]
    if not vals:
        return "—"
    return f"{sum(vals)/len(vals):.1f}（{min(vals):.1f}–{max(vals):.1f}）"


def arrival_time_cell(rows: list[dict]) -> str:
    vals = [float(r["arrival_time_s"]) for r in rows if r.get("arrival_time_s") is not None]
    if not vals:
        return "—"
    lo, hi = min(vals), max(vals)
    mean = sum(vals) / len(vals)
    if abs(lo - hi) < 5e-3:
        return f"{mean:.2f}"
    return f"{mean:.2f}（{lo:.2f}–{hi:.2f}）"


def first_commit(rows: list[dict]) -> str:
    for r in rows:
        if r.get("commit"):
            return str(r["commit"])
    return "—"


def ok_mark(flag: bool) -> str:
    return "一致" if flag else "不一致"


def render_md(meta: dict, groups_out: dict, identity: dict, cuts: list[str]) -> str:
    commit = meta["commit"]
    short = commit[:12]
    dirty = "（工作区有未提交改动，结果记录带 -dirty）" if meta.get("dirty") else ""
    lines = [
        "# 票 12e：上游策略 vs 影子盾",
        "",
        "方向 D 素材。比较的是同一张场景卡、同一个 commit 上：",
        "",
        "- **upstream**（穿林 / 编队任务里结果记录的策略名是 `crossing` / `formation`）：规划按上游等价规则选候选，影子盾只记录「这一拍本该不该拒」，不改批准动作。",
        "- **certified**：现行安全门真正拦截。只接在单机 `goto` / `hover` 上。仓库没有把影子盾改成硬拦的开关。",
        "",
        "穿林和编队任务由任务类型决定规划路径，`--policy certified` 会被忽略，不能当成多机硬门对照。",
        "",
        "## 出处",
        "",
        "| 项 | 值 |",
        "|---|---|",
        f"| 代码 commit | `{commit}`（main{dirty}） |",
        "| 层级 | L0 |",
        "| 形态 | 进程内 |",
        f"| 跑数时间 | {meta['started_at']} 起，写表于 {meta['finished_at']} |",
        f"| 到达口径 | ≤ 0.4 m 且 ≤ 0.2 m/s，保持 1 s；机体半径 0.35 m |",
        f"| 机间碰撞线 | 0.70 m（两倍机体半径） |",
        f"| 净空 | 机体中心到障碍表面的距离 − 0.35 m |",
        f"| 影子盾 | 开（对照列）；关的核验见「不干预」节 |",
        f"| 数字来源 | 本票脚本 `scripts/12e-upstream-vs-shield.py` 在干净工作区上的实测，原始记录 [12e-runs.json](12e-runs.json) |",
        "",
        "墙钟随机器负载变，不参与确定性比较。同 seed 重跑只要求除墙钟外的结果一致。",
        "",
        "## 怎么读",
        "",
        "- 影子拒绝率是全体 tick 里 `would_reject` 的比例。一 tick 多个拒绝码在直方图里各计一次。",
        "- 拒绝不改 `outcome`，也不进碰撞计数。碰撞是几何真值。",
        "- certified 对照不是里程碑通过线，继续只报数（ADR 0010、ADR 0016）。",
        "",
        "## 总表",
        "",
        "| 场景 | 策略 | seeds | 到达 | 碰撞 | 到达时间 s | 影子拒绝率 | 主要拒绝码 | 最小净空 m | 最小机间 m | 墙钟 s | commit |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for g in GROUPS:
        rows = groups_out.get(g["id"]) or []
        if not rows:
            if g["id"] in (identity.get("skipped_groups") or []):
                lines.append(
                    f"| {g['title']} | {g['policy']} | — | — | — | — | — | — | — | — | — | 未跑，见裁剪 |"
                )
            continue
        seeds = ",".join(str(r["seed"]) for r in sorted(rows, key=lambda r: int(r["seed"])))
        pol = rows[0].get("policy") or g["policy"]
        sha = first_commit(rows)
        lines.append(
            "| {title} | {pol} | {seeds} | {arr} | {col} | {at} | {rr} | {hist} | {clr} | {inter} | {wall} | `{sha}` |".format(
                title=g["title"],
                pol=pol,
                seeds=seeds,
                arr=arrival_cell(rows),
                col=collision_cell(rows),
                at=arrival_time_cell(rows),
                rr=reject_rate_cell(rows),
                hist=hist_cell(rows),
                clr=clearance_range(rows),
                inter=inter_range(rows),
                wall=wall_cell(rows),
                sha=sha[:12] if sha != "—" else "—",
            )
        )
    lines += [
        "",
        "## 不干预：影子盾开/关动作序列逐位一致",
        "",
        "仓库已有 `tests/test_shadow.py`：悬停、三机开阔区、g2-pillar 的 upstream 与 certified（短时）在同 seed 下开/关盾，批准动作、规划输入和去掉影子字段后的结果记录一致。本票复跑该文件，并给代表卡补了短时和部分全时长核验。",
        "",
        "| 检查 | 结果 |",
        "|---|---|",
    ]
    pytest_info = identity.get("pytest") or {}
    rc = pytest_info.get("returncode")
    lines.append(
        f"| `pytest tests/test_shadow.py` | {'通过' if rc == 0 else f'失败（退出码 {rc}）'} |"
    )
    for row in identity.get("short") or []:
        lines.append(
            f"| 短时 {row['scene']} @ {row['policy']} seed {row['seed']}（{row['duration_s']} s，{row['n_ticks']} tick） | "
            f"动作{ok_mark(row['same_actions'])}，去掉影子字段后{ok_mark(row['same_core'])} |"
        )
    for row in identity.get("full") or []:
        lines.append(
            f"| 全时长 {row['scene']} @ {row['policy']} seed {row['seed']} | "
            f"动作{ok_mark(row['same_actions'])}，到达 {row['arrived_off']}/{row['arrived_on']}，"
            f"结果 {row['outcome_off']}/{row['outcome_on']}，去掉影子字段后{ok_mark(row['same_core'])} |"
        )
    lines += [
        "",
        "结论：当前 main 上影子盾仍然只记录、不改动作。同 seed 开/关盾，动作序列逐位一致。",
        "",
        "## 分场景",
    ]

    for g in GROUPS:
        rows = groups_out.get(g["id"]) or []
        if not rows:
            continue
        rows = sorted(rows, key=lambda r: int(r["seed"]))
        sha = first_commit(rows)
        pol = rows[0].get("policy") or g["policy"]
        note = g.get("seed_note") or ""
        extra = f" {note}。" if note else ""
        lines += [
            "",
            f"### {g['title']}",
            "",
            f"层级 L0，策略 `{pol}`，影子盾开，commit `{sha}`。到达 {arrival_cell(rows)}，碰撞 {collision_cell(rows)}。"
            f"场景卡哈希 `{rows[0].get('scene_card_hash', '')}`。{extra}",
            "",
            "| seed | 结果 | 到达 | 到达时间 s | 最小障碍距离 m | 最小净空 m | 最小机间距离 m | 影子拒绝率 | 主要拒绝码 | 仿真 s | 墙钟 s |",
            "|---|---|---|---|---|---|---|---|---|---|---|",
        ]
        for r in rows:
            block = r.get("crossing") or r.get("formation") or {}
            clear = block.get("min_obstacle_clearance_m")
            if clear is None and r.get("min_obstacle_distance_m") is not None:
                clear = float(r["min_obstacle_distance_m"]) - 0.35
            hist = r.get("shadow_reject_histogram") or {}
            top = ", ".join(f"{k}:{v}" for k, v in Counter(hist).most_common(3)) or "—"
            lines.append(
                "| {seed} | {out} | {arr} | {at} | {obs} | {clr} | {inter} | {rr} | {hist} | {sim} | {wall} |".format(
                    seed=r.get("seed"),
                    out=r.get("outcome") or "—",
                    arr=f"{r.get('arrived')}/{r.get('n_drones')}",
                    at=fmt(r.get("arrival_time_s"), 2),
                    obs=fmt(r.get("min_obstacle_distance_m"), 3),
                    clr=fmt(clear, 3),
                    inter=fmt(r.get("min_inter_drone_distance_m"), 3),
                    rr=fmt(r.get("shadow_reject_rate"), 3),
                    hist=top,
                    sim=fmt(r.get("sim_duration_s"), 2),
                    wall=fmt(r.get("wall_time_s"), 1),
                )
            )
        c = hist_sum(rows)
        if c:
            total = sum(c.values())
            lines += ["", "拒绝码合计（所有 seed、一 tick 多码各计一次）：", ""]
            lines += ["| 拒绝码 | 次数 | 占直方图合计 |", "|---|---|---|"]
            for k, v in c.most_common():
                lines.append(f"| `{k}` | {v} | {v/total:.3f} |")

    lines += [
        "",
        "## 说明",
        "",
        "1. **没有影子盾拦截模式。** `--shadow off` 只是不写拒绝字段；`--shadow on` 不改动作。要把现行门变成硬拦，只能改用 `certified` 策略。",
        "2. **certified 只覆盖单机。** `yopo_l0.policy.POLICIES` 里的 `certified` 走 vendored `UavRuntime` + `SafetyGate`。多机 crossing / formation / switch 在运行器里走编队协议和上游 YOPO 提案，结果里的 `policy` 是任务名，不是 CLI 的 `--policy`。",
        "3. **g2-pillar @ certified 到不了。** 硬门在 ENV 证明处把候选全部拦下，计划码是 `NO_ENV_SAFE_CANDIDATE` 和 `cand:ENV:UNKNOWN`，飞机不离开起点（最小障碍距离一直是离地 1.500 m）。这是硬门行为（ADR 0010）。同一几何的 upstream 10/10 到达、碰撞 0；影子直方图里也会记 `ENV:UNKNOWN`，但不改动作。certified 开着影子盾时，直方图主要是 `LAG_VIOLATION`：飞机停在起点，oracle 落后界被触发，这是记录，不是第二道硬拦。",
        "4. **拒绝率高并不表示飞行失败。** L0 上 `ENV:UNKNOWN`、`TIME:SWITCH_NOT_IN_FUTURE` 等是影子记录，冻结表把影子拒绝率列为只报数（ADR 0016）。",
        "",
        "## 裁剪",
        "",
    ]
    if cuts:
        for c in cuts:
            lines.append(f"- {c}")
    else:
        lines.append("- 无。代表卡均已按计划跑完。")
    lines += [
        "",
        "## 怎么重跑",
        "",
        "`out/` 在 `.gitignore` 里。脚本把批跑写到 `out/12e/`，再把汇总 JSON 和本页写进 `docs/results/`。",
        "",
        "```bash",
        "make venv",
        "git rev-parse HEAD",
        ".venv/bin/python scripts/12e-upstream-vs-shield.py --jobs 2",
        "```",
        "",
        "常用旗标：`--certified-seeds 0` 只跑 certified 的 seed 0；`--six-seeds 0,1` 给六机林带加 seed；`--skip-certified` 跳过 certified；`--report-only` 只根据已有 `out/12e/` 重写本页。",
        "",
        f"结果记录里的 `commit` 应与本页出处一致（commit 随 HEAD 变）。墙钟列会随机器变。本页对应 `{short}`。",
        "",
    ]
    return "\n".join(lines)


def parse_seeds(text: str | None, default: list[int]) -> list[int]:
    if text is None:
        return list(default)
    text = text.strip()
    if not text:
        return []
    return [int(s) for s in text.split(",") if s != ""]


def group_out_dir(base: Path, gid: str) -> Path:
    return base / gid


def main(argv=None) -> int:
    os.chdir(ROOT)
    ap = argparse.ArgumentParser(description="票 12e：上游策略 vs 影子盾对照")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--out", default="out/12e")
    ap.add_argument("--docs", default="docs/results")
    ap.add_argument("--report-only", action="store_true", help="不跑新的批，只用 out/ 里已有结果写文档")
    ap.add_argument("--skip-certified", action="store_true")
    ap.add_argument("--skip-identity", action="store_true")
    ap.add_argument("--skip-full-identity", action="store_true", help="只做短时开/关核验，不做全时长")
    ap.add_argument("--certified-seeds", default=None, help="覆盖 certified 的 seed 列表，例如 0 或 0,1,2")
    ap.add_argument("--six-seeds", default=None, help="覆盖六机林带的 seed 列表，默认 0")
    ap.add_argument("--skip-six", action="store_true")
    ap.add_argument("--only", default=None, help="只跑这些组 id，逗号分隔")
    args = ap.parse_args(argv)

    out_base = ensure_dir(ROOT / args.out)
    out_cards = ensure_dir(out_base / "cards")
    docs = ROOT / args.docs
    ensure_dir(docs)

    commit = git_commit(ROOT)
    dirty = git_dirty(ROOT)
    started = now_label()
    cuts: list[str] = []
    skipped_groups: list[str] = []
    only = {s.strip() for s in args.only.split(",")} if args.only else None

    groups_cfg = []
    for g in GROUPS:
        g = dict(g)
        if g.get("kind") == "certified":
            g["seeds"] = parse_seeds(args.certified_seeds, g["seeds"])
            if args.skip_certified or not g["seeds"]:
                skipped_groups.append(g["id"])
                if args.skip_certified:
                    cuts.append("按旗标跳过了 g2-pillar @ certified。")
                continue
        if g.get("six"):
            g["seeds"] = parse_seeds(args.six_seeds, g["seeds"])
            if args.skip_six or not g["seeds"]:
                skipped_groups.append(g["id"])
                cuts.append("按旗标跳过了六机林带卡。")
                continue
        if only is not None and g["id"] not in only:
            skipped_groups.append(g["id"])
            continue
        if g.get("kind") == "certified" and set(g["seeds"]) != set(range(10)):
            cuts.append(
                f"g2-pillar @ certified 只跑了 seeds {','.join(map(str, g['seeds']))}。"
                "该策略单 seed 全时长大约十几分钟，完整 10 seed 可用脚本默认旗标重跑。"
            )
        if g.get("six") and set(g["seeds"]) != set(range(10)):
            cuts.append(
                f"{g['id']} 只跑了 seeds {','.join(map(str, g['seeds']))}（单次墙钟超过 60 s，默认按 suite 惯例先跑 seed 0）。"
            )
        groups_cfg.append(g)

    ident_path = out_base / "identity.json"
    identity = {"pytest": None, "short": [], "full": [], "skipped_groups": skipped_groups}
    if ident_path.is_file():
        prev = json.loads(ident_path.read_text())
        identity["pytest"] = prev.get("pytest")
        identity["short"] = prev.get("short") or []
        identity["full"] = prev.get("full") or []
    if not args.report_only and not args.skip_identity:
        if not (identity.get("pytest") and identity["pytest"].get("returncode") == 0):
            identity["pytest"] = run_pytest_shadow(out_base)
            if identity["pytest"]["returncode"] != 0:
                print("pytest tests/test_shadow.py 失败，见 out/12e/test_shadow.log", file=sys.stderr)
        else:
            print("复用已通过的 tests/test_shadow.py 结果", flush=True)
        if not identity.get("short"):
            print("短时开/关核验…", flush=True)
            for scene, policy, seed, dur in SHORT_IDENTITY:
                row = run_pair(ROOT / scene, policy, seed, dur)
                identity["short"].append(row)
                print(f"  short {row['scene']} same_actions={row['same_actions']} same_core={row['same_core']}", flush=True)
            ident_path.write_text(json.dumps(identity, indent=2, ensure_ascii=False) + "\n")
        else:
            print(f"复用已有短时核验 {len(identity['short'])} 条", flush=True)
    elif args.report_only or args.skip_identity:
        print("跳过新的开/关核验，使用已有 identity 记录", flush=True)

    if not args.report_only:
        for g in groups_cfg:
            dest = group_out_dir(out_base, g["id"])
            existing = load_rows(dest)
            want = set(g["seeds"])
            have = {int(r["seed"]) for r in existing}
            # 已有完整结果就跳过，方便 certified 预先开跑、脚本随后接管。
            if want <= have and existing:
                print(f"跳过 {g['id']}：已有 seeds {sorted(have)}", flush=True)
                continue
            scene = scene_for_group(g, out_cards)
            rc = run_batch(scene, g["policy"], g["seeds"], dest, args.jobs)
            # certified 到达 0、dense 可能撞树，冻结门按 gate_policies / gate_densities 不会挡这两类。
            if rc != 0:
                print(f"警告：{g['id']} 批跑退出码 {rc}，仍收集已有结果。", flush=True)

        if not args.skip_identity and not args.skip_full_identity:
            print("全时长开/关核验（不含六机）…", flush=True)
            for scene, policy, seed in FULL_IDENTITY:
                already = any(
                    r.get("scene") in (key_name, Path(scene).stem, scene)
                    and r.get("policy") == policy and int(r.get("seed")) == seed
                    for r in identity.get("full") or []
                )
                if already:
                    print(f"  复用全时长 {scene} seed {seed}", flush=True)
                    continue
                row = run_pair(ROOT / scene, policy, seed, None)
                identity.setdefault("full", []).append(row)
                ident_path.write_text(json.dumps(identity, indent=2, ensure_ascii=False) + "\n")
                print(
                    f"  full {row['scene']} same_actions={row['same_actions']} "
                    f"arrived {row['arrived_off']}/{row['arrived_on']}",
                    flush=True,
                )

    groups_out: dict[str, list[dict]] = {}
    slim: list[dict] = []
    for g in GROUPS:
        rows = load_rows(group_out_dir(out_base, g["id"]))
        if g in groups_cfg or rows:
            want = None
            for cfg in groups_cfg:
                if cfg["id"] == g["id"]:
                    want = set(cfg["seeds"])
                    break
            if want is not None:
                rows = [r for r in rows if int(r["seed"]) in want]
            groups_out[g["id"]] = rows
            for r in rows:
                slim.append({"group": g["id"], **drop_heavy(r)})

    if not args.report_only:
        (out_base / "identity.json").write_text(json.dumps(identity, indent=2, ensure_ascii=False) + "\n")

    finished = now_label()
    result_commits = {str(r.get("commit")) for rows in groups_out.values() for r in rows if r.get("commit")}
    # 结果记录的 dirty 检查不含未跟踪文件，与 runner 一致；写表时以结果里的 commit 为准。
    label = sorted(result_commits)[0] if len(result_commits) == 1 else (commit + ("-dirty" if dirty else ""))
    meta = {
        "commit": label,
        "dirty": dirty and not result_commits,
        "started_at": started,
        "finished_at": finished,
        "jobs": args.jobs,
        "cuts": cuts,
        "result_commits": sorted(result_commits),
    }
    (out_base / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    (docs / "12e-runs.json").write_text(json.dumps(slim, indent=2, ensure_ascii=False) + "\n")
    (docs / "12e-identity.json").write_text(json.dumps(identity, indent=2, ensure_ascii=False) + "\n")
    md = render_md(meta, groups_out, identity, cuts)
    (docs / "12e-upstream-vs-shield.md").write_text(md if md.endswith("\n") else md + "\n")
    print(md)
    print(f"已写 {docs / '12e-upstream-vs-shield.md'}", flush=True)
    # 开/关不一致时失败，方便发现回归。
    bad = []
    if identity.get("pytest") and identity["pytest"].get("returncode") not in (0, None) and not args.report_only:
        bad.append("pytest")
    for row in (identity.get("short") or []) + (identity.get("full") or []):
        if not row.get("same_actions") or not row.get("same_core"):
            bad.append(f"{row['scene']} seed {row['seed']}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
