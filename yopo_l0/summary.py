"""把批跑 results.json 汇总成 M0–M4 结果总表。

只聚合已经存在的运行结果，不编造数字。未跑到的行在「本批未覆盖」列出，不填数。
冻结 / 只报数状态按场景卡 criteria.frozen 与 criteria.gates 判定（ADR 0016）。
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone, timedelta
from collections import defaultdict
from pathlib import Path

from .criteria import _clearance, active_gates, collision_count, metric_value
from .plan_wall import p95_ms as plan_wall_p95_ms
from .scene import load_card

SCHEMA = "yopo-restart/results-summary/1"

# 总表只收 M0–M4 场景。hover 等套件卡不进这一张。
_MILESTONE = {
    "g2-pillar": "M0",
    "formation-hold": "M1",
    "formation-drop": "M1",
    "formation-duplicate": "M1",
    "formation-delay": "M1",
    "formation-reorder": "M1",
    "crossing-pillars": "M2",
    "crossing-forest": "M2",
    "formation-switch": "M3",
    "formation-6-open": "M4",
    "crossing-6-forest": "M4",
    "crossing-6-forest-sparse": "M4",
}

# 已知应在总表出现、但本批结果里没有的行。只写说明，不填数。
_UNMEASURED = [
    {
        "milestone": "M0",
        "scene": "g2-pillar",
        "detail": "certified，10 seed 全程",
        "note": "继续只报数。本批没有收齐全 10 个 seed 的全程结果，不填数。合入前回归里的 certified 冒烟是 4 s 前缀，不能代替全程。",
    },
    {
        "milestone": "M2",
        "scene": "crossing-forest",
        "detail": "sparse / dense",
        "note": "继续只报数；本批结果没有这两档，不填数。",
    },
    {
        "milestone": "M3",
        "scene": "formation-switch",
        "detail": "L1 抽检",
        "note": "票 09 已关闭，没有新方案对照。继续只报数。",
    },
    {
        "milestone": "M4",
        "scene": "formation-6-open",
        "detail": "ROS 2 全矩阵",
        "note": "继续只报数；本批结果没有 ROS 2 形态，全矩阵另开票。",
    },
    {
        "milestone": "M4",
        "scene": "六机柱林",
        "detail": "",
        "note": "未测，不进套件。继续只报数。",
    },
]

_OPEN_AREA = {"formation-hold", "formation-drop", "formation-duplicate", "formation-delay", "formation-reorder", "formation-6-open"}

_SCENE_ORDER = (
    "g2-pillar",
    "formation-hold",
    "formation-drop",
    "formation-duplicate",
    "formation-delay",
    "formation-reorder",
    "crossing-pillars",
    "crossing-forest",
    "formation-switch",
    "formation-6-open",
    "crossing-6-forest",
    "crossing-6-forest-sparse",
)

_TIER_ORDER = ("none", "drop-5", "drop-20", "duplicate-10", "reorder-delay", "combined")
_DENSITY_ORDER = ("sparse", "medium", "dense")

_METRIC_LABELS = {
    "arrivals": "到达",
    "collisions": "碰撞",
    "arrival_time_s": "到达时间",
    "formation_error_p95_m": "队形误差 p95",
    "error_p95_before_m": "变阵前 p95",
    "error_p95_after_m": "变阵后 p95",
    "min_inter_drone_distance_m": "最小机间距离",
    "min_inter_drone_transition_m": "过渡最小机间距离",
    "min_obstacle_clearance_m": "最小净空",
    "transition_collisions": "过渡期碰撞",
}


def collect_result_files(inputs: list[Path]) -> list[Path]:
    """目录下优先用各批跑的 results.json，避免再把 seed 目录里的 result.json 算一遍。"""
    found: list[Path] = []
    for raw in inputs:
        path = Path(raw)
        if path.is_file():
            found.append(path.resolve())
            continue
        if not path.is_dir():
            raise FileNotFoundError(f"找不到输入 {path}")
        batch = sorted(p.resolve() for p in path.rglob("results.json"))
        found.extend(batch)
        covered = [p.parent for p in batch]
        for seed in sorted(p.resolve() for p in path.rglob("result.json")):
            if any(seed.is_relative_to(root) for root in covered):
                continue
            found.append(seed)
    unique: list[Path] = []
    seen: set[Path] = set()
    for path in found:
        if path in seen:
            continue
        seen.add(path)
        unique.append(path)
    if not unique:
        raise FileNotFoundError("输入里没有 results.json 或 result.json")
    return unique


def load_rows(files: list[Path]) -> list[dict]:
    rows: list[dict] = []
    for path in files:
        raw = json.loads(path.read_text())
        if isinstance(raw, dict):
            raw = [raw]
        if not isinstance(raw, list):
            raise ValueError(f"{path} 不是结果列表")
        for item in raw:
            if not isinstance(item, dict):
                raise ValueError(f"{path} 里有不是对象的结果")
            item = dict(item)
            item["_source"] = str(path)
            rows.append(item)
    return rows


def _identity(row: dict) -> tuple:
    return (
        row.get("scene"),
        row.get("policy"),
        row.get("form") or "inprocess",
        row.get("fault_tier"),
        row.get("forest_density"),
        bool(row.get("shadow", True)),
        row.get("seed"),
        row.get("tier") or "L0",
    )


def _metrics_payload(row: dict) -> dict:
    keys = (
        "outcome",
        "arrived",
        "n_drones",
        "arrival_rate",
        "arrival_time_s",
        "min_inter_drone_distance_m",
        "min_obstacle_distance_m",
        "formation_error_p95_m",
        "scene_card_hash",
        "commit",
        "crossing",
        "switch",
        "formation",
    )
    return {k: row.get(k) for k in keys}


def dedupe_rows(rows: list[dict]) -> list[dict]:
    """同一身份只留一条。数字不一致就报错，避免两份结果悄悄混在一起。"""
    by_id: dict[tuple, dict] = {}
    for row in rows:
        key = _identity(row)
        prev = by_id.get(key)
        if prev is None:
            by_id[key] = row
            continue
        if _metrics_payload(prev) != _metrics_payload(row):
            raise ValueError(f"同一身份出现两份不同结果：{key}")
    return list(by_id.values())


def _group_key(row: dict) -> tuple:
    return (
        _MILESTONE.get(str(row.get("scene") or ""), ""),
        row.get("scene"),
        row.get("policy"),
        row.get("tier") or "L0",
        row.get("form") or "inprocess",
        row.get("fault_tier"),
        row.get("forest_density"),
        bool(row.get("shadow", True)),
    )


def _find_card(scenes_dir: Path, name: str) -> Path | None:
    path = scenes_dir / f"{name}.json"
    return path if path.is_file() else None


def _range(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {
        "min": min(values),
        "max": max(values),
        "mean": sum(values) / len(values),
    }


def _fmt_range(span: dict[str, float] | None, nd: int = 3) -> str:
    if not span:
        return "—"
    lo, hi = span["min"], span["max"]
    if abs(hi - lo) < 0.5 * 10 ** (-nd):
        return f"{lo:.{nd}f}"
    return f"{lo:.{nd}f}–{hi:.{nd}f}"


def _fmt_opt(value: float | None, nd: int = 3) -> str:
    return "—" if value is None else f"{value:.{nd}f}"


def _fmt_seeds(seeds: list[int]) -> str:
    if not seeds:
        return "—"
    if len(seeds) == 1:
        return str(seeds[0])
    if seeds == list(range(seeds[0], seeds[-1] + 1)):
        return f"{seeds[0]}–{seeds[-1]}"
    return ",".join(str(s) for s in seeds)


def _display_source(path: Path | str) -> str:
    text = str(path)
    for marker in ("/out/m5/", "/out/"):
        idx = text.find(marker)
        if idx >= 0:
            return text[idx + 1 :]
    return text


def _compact_sources(paths: list[Path] | list[str]) -> list[str]:
    """同一次批跑底下的多份 seed result.json 收成一条，避免输入清单比总表还长。"""
    displayed = [_display_source(p) for p in paths]
    out: list[str] = []
    seen_roots: set[str] = set()
    counts: dict[str, int] = {}
    for text in displayed:
        parts = text.split("/")
        if text.endswith("/result.json") and len(parts) >= 3 and parts[-2].startswith("seed"):
            root = "/".join(parts[:-3]) + "/"
            counts[root] = counts.get(root, 0) + 1
    for text in displayed:
        parts = text.split("/")
        if text.endswith("/result.json") and len(parts) >= 3 and parts[-2].startswith("seed"):
            root = "/".join(parts[:-3]) + "/"
            if root in seen_roots:
                continue
            seen_roots.add(root)
            n = counts[root]
            out.append(f"{root}（{n} 个 seed 的 result.json）" if n > 1 else text)
        else:
            out.append(text)
    return out


def _short(sha: str | None, n: int = 12) -> str:
    if not sha:
        return "—"
    return sha[:n]


def _form_label(form: str, fault: str | None) -> str:
    word = "ROS 2" if form == "ros2" else "进程内"
    if fault:
        return f"{word} / {fault}"
    return word


def _shadow_label(shadow: bool) -> str:
    return "开" if shadow else "关"


def _status_phrase(frozen: bool, report_only_metrics: list[str]) -> str:
    if not frozen:
        return "未冻结 / report-only"
    if not report_only_metrics:
        return "判据已冻结"
    labels = "、".join(_METRIC_LABELS.get(name, name) for name in report_only_metrics)
    return f"判据已冻结（{labels}只报数）"


def _shown_metrics(row: dict, scene: str) -> list[str]:
    names = ["arrivals", "collisions"]
    if metric_value(row, "arrival_time_s") is not None:
        names.append("arrival_time_s")
    if row.get("formation_error_p95_m") is not None:
        names.append("formation_error_p95_m")
    switch = row.get("switch") or {}
    if switch.get("error_p95_before_m") is not None:
        names.append("error_p95_before_m")
    if switch.get("error_p95_after_m") is not None:
        names.append("error_p95_after_m")
    if switch.get("min_inter_drone_transition_m") is not None:
        names.append("min_inter_drone_transition_m")
    if switch.get("transition_collisions") is not None:
        names.append("transition_collisions")
    if row.get("min_inter_drone_distance_m") is not None:
        names.append("min_inter_drone_distance_m")
    if _clearance(row) is not None:
        names.append("min_obstacle_clearance_m")
    return names


def aggregate_group(rows: list[dict], *, scenes_dir: Path) -> dict:
    rows = sorted(rows, key=lambda r: int(r.get("seed") or 0))
    sample = rows[0]
    scene = str(sample["scene"])
    policy = str(sample.get("policy") or "")
    form = str(sample.get("form") or "inprocess")
    density = sample.get("forest_density")
    if density is None and any(r.get("forest_density") for r in rows):
        density = next(r.get("forest_density") for r in rows if r.get("forest_density"))
    card_path = _find_card(scenes_dir, scene)
    card = load_card(card_path) if card_path is not None else None
    gates = None
    if card is not None:
        gates = active_gates(card, policy=policy, form=form, density=density)
    frozen = gates is not None
    arrived = sum(int(r.get("arrived") or 0) for r in rows)
    slots = sum(int(r.get("n_drones") or 0) for r in rows)
    collisions = sum(collision_count(r) for r in rows)
    p95s = [float(r["formation_error_p95_m"]) for r in rows if r.get("formation_error_p95_m") is not None]
    inters = [
        float(r["min_inter_drone_distance_m"])
        for r in rows
        if r.get("min_inter_drone_distance_m") is not None
    ]
    clears = [v for r in rows if (v := _clearance(r)) is not None]
    times = [float(r["arrival_time_s"]) for r in rows if r.get("arrival_time_s") is not None]
    walls = [float(r["wall_time_s"]) for r in rows if r.get("wall_time_s") is not None]
    plan_walls = [v for r in rows if (v := plan_wall_p95_ms(r)) is not None]
    before = [float(v) for r in rows if (v := (r.get("switch") or {}).get("error_p95_before_m")) is not None]
    after = [float(v) for r in rows if (v := (r.get("switch") or {}).get("error_p95_after_m")) is not None]
    trans = [
        float(v)
        for r in rows
        if (v := (r.get("switch") or {}).get("min_inter_drone_transition_m")) is not None
    ]
    trans_col = sum(int((r.get("switch") or {}).get("transition_collisions") or 0) for r in rows)
    commits = sorted({str(r.get("commit") or "") for r in rows})
    hashes = sorted({str(r.get("scene_card_hash") or "") for r in rows})
    if card is not None:
        card_mismatch = [h for h in hashes if h and h != card.hash]
    else:
        card_mismatch = []
    shown = _shown_metrics(sample, scene)
    metric_status: dict[str, str] = {}
    for name in shown:
        gate_name = "arrival_rate" if name == "arrivals" else name
        if frozen and gates is not None and gate_name in gates:
            metric_status[name] = "frozen"
        else:
            metric_status[name] = "report-only"
    report_only = [name for name in shown if metric_status[name] == "report-only"]
    # 变阵卡冻的是变阵前后 p95 和过渡机间距离；全程 p95 / 全程机间不要再在状态里当成漏冻。
    if "error_p95_before_m" in metric_status or "error_p95_after_m" in metric_status:
        report_only = [name for name in report_only if name != "formation_error_p95_m"]
    if "min_inter_drone_transition_m" in metric_status:
        report_only = [name for name in report_only if name != "min_inter_drone_distance_m"]
    open_area = scene in _OPEN_AREA
    seeds = [int(r["seed"]) for r in rows]
    return {
        "milestone": _MILESTONE[scene],
        "scene": scene,
        "scene_path": str(card_path.as_posix()) if card_path is not None else None,
        "policy": policy,
        "tier": str(sample.get("tier") or "L0"),
        "form": form,
        "fault_tier": sample.get("fault_tier"),
        "forest_density": density,
        "shadow": bool(sample.get("shadow", True)),
        "commit": commits[0] if len(commits) == 1 else None,
        "commits": commits,
        "scene_card_hash": hashes[0] if len(hashes) == 1 else None,
        "scene_card_hashes": hashes,
        "card_hash_mismatch": card_mismatch,
        "seeds": seeds,
        "n_seeds": len(seeds),
        "arrived": arrived,
        "arrival_slots": slots,
        "arrivals": f"{arrived}/{slots}" if slots else "—",
        "arrival_rate": None if not slots else arrived / slots,
        "collisions": collisions,
        "formation_error_p95_m": _range(p95s),
        "error_p95_before_m": _range(before),
        "error_p95_after_m": _range(after),
        "min_inter_drone_distance_m": None if not inters else min(inters),
        "min_inter_drone_transition_m": None if not trans else min(trans),
        "transition_collisions": trans_col if sample.get("task") == "switch" or sample.get("switch") else None,
        "min_obstacle_clearance_m": None if not clears else min(clears),
        "clearance_is_ground": open_area,
        "arrival_time_s": _range(times),
        "wall_time_s": _range(walls),
        "plan_wall_ms": _range(plan_walls),
        "sources": sorted({str(r.get("_source")) for r in rows if r.get("_source")}),
        "frozen": frozen,
        "gates": gates,
        "metric_status": metric_status,
        "status": _status_phrase(frozen, report_only),
    }


def build_summary(
    rows: list[dict],
    *,
    scenes_dir: Path,
    sources: list[Path] | None = None,
    reproduce: list[str] | None = None,
    generated_at: str | None = None,
) -> dict:
    kept = [r for r in rows if r.get("scene") in _MILESTONE]
    grouped: dict[tuple, list[dict]] = defaultdict(list)
    for row in kept:
        grouped[_group_key(row)].append(row)
    def _sort_key(item: tuple[tuple, list[dict]]) -> tuple:
        key, _rows = item
        milestone, scene, policy, tier, form, fault, density, shadow = key
        policy_rank = {"upstream": 0, "formation": 1, "crossing": 2, "switch": 3, "certified": 9}.get(str(policy), 5)
        form_rank = 0 if form != "ros2" else 1
        shadow_rank = 0 if shadow else 1
        scene_rank = _SCENE_ORDER.index(scene) if scene in _SCENE_ORDER else len(_SCENE_ORDER)
        dens_rank = _DENSITY_ORDER.index(density) if density in _DENSITY_ORDER else -1
        tier_rank = _TIER_ORDER.index(fault) if fault in _TIER_ORDER else len(_TIER_ORDER)
        return (milestone, scene_rank, str(scene), dens_rank, form_rank, tier_rank, str(fault or ""), shadow_rank, policy_rank, str(policy), str(tier))

    table = [aggregate_group(group, scenes_dir=scenes_dir) for _, group in sorted(grouped.items(), key=_sort_key)]
    present = {(item["scene"], item["form"], item.get("forest_density"), item.get("fault_tier")) for item in table}
    unmeasured = []
    for item in _UNMEASURED:
        if item["scene"] == "g2-pillar" and "certified" in (item.get("detail") or ""):
            if any(row["scene"] == "g2-pillar" and row["policy"] == "certified" for row in table):
                continue
            unmeasured.append(item)
            continue
        if item["scene"] == "crossing-forest":
            densities = {d for scene, _form, d, _tier in present if scene == "crossing-forest"}
            missing = [d for d in ("sparse", "dense") if d not in densities]
            if not missing:
                continue
            unmeasured.append({**item, "detail": " / ".join(missing)})
            continue
        if item["scene"] == "formation-6-open" and "ROS 2" in item["detail"]:
            if any(scene == "formation-6-open" and form == "ros2" for scene, form, _d, _t in present):
                continue
        if item["scene"] == "六机柱林":
            unmeasured.append(item)
            continue
        if item["detail"] == "L1 抽检":
            if any(row.get("tier") == "L1" and row.get("scene") == "formation-switch" for row in rows):
                continue
            unmeasured.append(item)
            continue
        unmeasured.append(item)
    commits = sorted({c for item in table if (c := item.get("commit"))})
    return {
        "schema": SCHEMA,
        "adr": "0016",
        "commit": commits[0] if len(commits) == 1 else None,
        "commits": commits,
        "sources": _compact_sources(sources or []),
        "reproduce": list(reproduce or []),
        "generated_at": generated_at,
        "rows": table,
        "unmeasured": unmeasured,
    }


def render_markdown(summary: dict) -> str:
    commit = summary.get("commit")
    commits = summary.get("commits") or []
    commit_cell = _short(commit) if commit else "、".join(_short(c) for c in commits) or "—"
    lines = [
        "# M0–M4 结果总表",
        "",
        (
            f"数字全部来自下面列出的批跑结果，对应 commit `{commit_cell}`。"
            "没有从冻结建议表抄数，也没有编数。缺的行只在「本批未覆盖」里说明，不填数。"
            "结果记录里的 commit 是跑那一次时的 HEAD；若后来只合进文档或脚本、没有改 `yopo_l0` 或场景卡，这些数字对当前 main 仍然有效。"
        ),
        (
            "层级默认 L0，策略默认 `upstream` + 影子盾开。"
            "冻结口径见 [ADR 0016](../adr/0016-m0-m4-freeze-recommendation.md) 和场景卡 `criteria.frozen` / `criteria.gates`。"
            "已冻指标失败时批跑退出非 0；未写入 gates 的指标只报数，不挡退出码。碰撞数始终作为硬事实入表。"
            "已知限制见 [LIMITATIONS.md](../LIMITATIONS.md)，交付视频索引见 [videos.md](videos.md)。"
        ),
        "",
    ]
    if summary.get("sources"):
        lines.append("输入：")
        lines.append("")
        for src in summary["sources"]:
            lines.append(f"- `{src}`")
        lines.append("")
    if len(commits) > 1:
        lines.append("注意：这批结果里出现了不止一个 commit，表中按行写出，没有混成一个数。")
        lines.append("")
    mismatch = [row for row in summary["rows"] if row.get("card_hash_mismatch")]
    if mismatch:
        lines.append("注意：有的结果里的场景卡哈希和当前仓库里的卡不一致，表里保留结果自己的哈希。")
        lines.append("")
    lines += [
        "## 总表",
        "",
        "| 里程碑 | 场景卡 | 策略 | 层级 | 形态 / 档位 | 影子盾 | commit | 场景卡哈希 | seeds | 到达 | 碰撞 | 队形误差 p95 m | 最小机间距离 m | 最小净空 m | 规划墙钟 p95 ms（只报数） | 状态 |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for row in summary["rows"]:
        clear = _fmt_opt(row["min_obstacle_clearance_m"])
        if row.get("clearance_is_ground") and row["min_obstacle_clearance_m"] is not None:
            clear = f"离地 {clear}"
        p95 = _fmt_range(row["formation_error_p95_m"])
        if row.get("error_p95_before_m") or row.get("error_p95_after_m"):
            p95 = (
                f"前 {_fmt_range(row.get('error_p95_before_m'))} / "
                f"后 {_fmt_range(row.get('error_p95_after_m'))}（全程 {p95}）"
            )
        inter = _fmt_opt(row["min_inter_drone_distance_m"])
        if row.get("min_inter_drone_transition_m") is not None:
            inter = f"{inter}（过渡 {_fmt_opt(row['min_inter_drone_transition_m'])}）"
        hash_cell = "—"
        if row.get("scene_card_hash"):
            hash_cell = f"`{_short(row['scene_card_hash'])}`"
        elif row.get("scene_card_hashes"):
            hash_cell = "、".join(f"`{_short(h)}`" for h in row["scene_card_hashes"])
        commit_row = "—"
        if row.get("commit"):
            commit_row = f"`{_short(row['commit'])}`"
        elif row.get("commits"):
            commit_row = "、".join(f"`{_short(c)}`" for c in row["commits"])
        density = row.get("forest_density")
        scene_cell = row["scene"]
        if density:
            scene_cell = f"{scene_cell}（{density}）"
        lines.append(
            "| {milestone} | {scene} | {policy} | {tier} | {form} | {shadow} | {commit} | {hash} | {seeds} | {arrivals} | {collisions} | {p95} | {inter} | {clear} | {plan_wall} | {status} |".format(
                milestone=row["milestone"],
                scene=scene_cell,
                policy=row["policy"],
                tier=row["tier"],
                form=_form_label(row["form"], row.get("fault_tier")),
                shadow=_shadow_label(row["shadow"]),
                commit=commit_row,
                hash=hash_cell,
                seeds=_fmt_seeds(row["seeds"]),
                arrivals=row["arrivals"],
                collisions=row["collisions"],
                p95=p95,
                inter=inter,
                clear=clear,
                plan_wall=_fmt_range(row.get("plan_wall_ms")),
                status=row["status"],
            )
        )
    lines += [
        "",
        "队形误差 p95 取结果里的 `formation_error_p95_m`（官方口径）；多个 seed 写出最小到最大。"
        "最小机间距离和最小净空取各 seed 里最紧的值。净空 = 机体中心到障碍表面的距离 − 0.35 m。"
        "开阔区的净空是离地，不是避障净空，只报数。"
        "规划墙钟是每次网络前向（或后段优化）的墙钟 p95，单位毫秒，只报数，不进 gates；"
        "没有这一字段的旧结果写 —。"
        "整回路墙钟 `wall_time_s` 随负载变化，含光线投射和动力学，不进这张表，也不进 gates。",
        "",
    ]
    if summary.get("unmeasured"):
        lines += [
            "## 本批未覆盖（不填数）",
            "",
            "| 里程碑 | 场景 | 说明 |",
            "|---|---|---|",
        ]
        for item in summary["unmeasured"]:
            scene = item["scene"]
            if item.get("detail"):
                scene = f"{scene}（{item['detail']}）"
            lines.append(f"| {item['milestone']} | {scene} | {item['note']} |")
        lines.append("")
    if summary.get("reproduce"):
        lines += ["## 复现", "", "```bash"]
        lines += list(summary["reproduce"])
        lines += ["```", ""]
    if summary.get("generated_at"):
        lines.append(f"生成时间：{summary['generated_at']}（America/La_Paz，UTC-4）。")
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def write_summary(summary: dict, *, md_path: Path, json_path: Path) -> None:
    md_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    md_path.write_text(render_markdown(summary))
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")


def _now_lapaz() -> str:
    # 固定用 UTC-4，不依赖系统时区数据库里有没有 America/La_Paz。
    now = datetime.now(timezone(timedelta(hours=-4)))
    return now.strftime("%Y-%m-%d %H:%M")


def summarize(
    inputs: list[Path],
    *,
    scenes_dir: Path,
    md_path: Path,
    json_path: Path,
    reproduce: list[str] | None = None,
) -> dict:
    files = collect_result_files(inputs)
    rows = dedupe_rows(load_rows(files))
    summary = build_summary(
        rows,
        scenes_dir=scenes_dir,
        sources=files,
        reproduce=reproduce,
        generated_at=_now_lapaz(),
    )
    write_summary(summary, md_path=md_path, json_path=json_path)
    return summary


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="把批跑结果汇总成 docs/results/SUMMARY.md 和 JSON 副本")
    ap.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help="results.json、result.json，或包含它们的目录",
    )
    ap.add_argument("--scenes-dir", type=Path, default=None, help="场景卡目录，默认仓库 scenes/")
    ap.add_argument("--md", type=Path, default=Path("docs/results/SUMMARY.md"))
    ap.add_argument("--json", type=Path, default=Path("docs/results/SUMMARY.json"))
    ap.add_argument("--repro", action="append", default=[], help="写入「复现」节的命令，可重复")
    args = ap.parse_args(argv)
    scenes_dir = args.scenes_dir
    if scenes_dir is None:
        scenes_dir = Path(__file__).resolve().parents[1] / "scenes"
    summarize(
        args.inputs,
        scenes_dir=scenes_dir,
        md_path=args.md,
        json_path=args.json,
        reproduce=args.repro,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
