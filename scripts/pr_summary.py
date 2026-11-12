#!/usr/bin/env python3
"""把 regress OUT 汇成可贴进 PR 的确定性 Markdown。

用法：scripts/pr_summary.py <regress OUT 目录> [--base <另一个 OUT 目录>]
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PYTEST_FIELDS = ("passed", "failed", "skipped", "deselected", "errors", "xfailed", "xpassed", "warnings")
PYTEST_NAMES = ("通过", "失败", "跳过", "取消选择", "错误", "预期失败", "意外通过", "警告")
COUNT_RE = re.compile(r"(\d+)\s+(passed|failed|skipped|deselected|errors?|xfailed|xpassed|warnings?)\b")
HEADING_RE = re.compile(r"^### (.+?) @ (.+?)（")
FAILURE_RE = re.compile(r"^- (.+?) @ (.+?)：")
LOWER_GATES = {"arrival_rate", "min_inter_drone_distance_m", "min_inter_drone_transition_m", "min_obstacle_clearance_m"}


def pytest_counts(path: Path) -> dict[str, int]:
    """只取 pytest -q 最后一条汇总行，忽略进度和警告正文。"""
    for line in reversed(path.read_text(encoding="utf-8").splitlines()):
        if " in " not in line:
            continue
        found = COUNT_RE.findall(line)
        if found:
            counts = dict.fromkeys(PYTEST_FIELDS, 0)
            for number, name in found:
                counts[{"error": "errors", "warning": "warnings"}.get(name, name)] = int(number)
            return counts
        if "no tests ran" in line:
            return dict.fromkeys(PYTEST_FIELDS, 0)
    raise ValueError(f"找不到 pytest 汇总行：{path}")


def summary_verdicts(path: Path) -> tuple[dict[tuple[str, str], bool], set[tuple[str, str]]]:
    """summary.md 保留了 batch 对前缀跑和未冻结场景的实际判定。"""
    states: dict[tuple[str, str], bool] = {}
    failures: set[tuple[str, str]] = set()
    if not path.is_file():
        return states, failures
    in_failures = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("### 冻结判据未通过"):
            in_failures = True
            continue
        if line.startswith("### "):
            in_failures = False
            match = HEADING_RE.match(line)
            if match:
                key = match.group(1), match.group(2)
                if "未冻结 / report-only" in line:
                    states[key] = True
                elif "判据已冻结" in line:
                    states[key] = False
        elif in_failures:
            match = FAILURE_RE.match(line)
            if match:
                failures.add((match.group(1), match.group(2)))
    return states, failures


def arrival_rate(row: dict) -> float | None:
    if row.get("arrival_rate") is not None:
        return float(row["arrival_rate"])
    n = row.get("n_drones")
    arrived = row.get("arrived")
    return None if not n or arrived is None else float(arrived) / float(n)


def collisions(row: dict) -> int | None:
    block = row.get("crossing") or {}
    if "obstacle_collisions" in block or "inter_drone_collisions" in block:
        return int(block.get("obstacle_collisions") or 0) + int(block.get("inter_drone_collisions") or 0)
    outcome = row.get("outcome")
    if outcome is None:
        return None
    return int(outcome == "COLLISION")


def gate_value(row: dict, name: str) -> float | None:
    if name == "arrival_rate":
        return arrival_rate(row)
    if name == "collisions":
        value = collisions(row)
        return None if value is None else float(value)
    if name == "transition_collisions":
        return float((row.get("switch") or {}).get("transition_collisions") or 0)
    if name in {"error_p95_before_m", "error_p95_after_m", "min_inter_drone_transition_m"}:
        value = (row.get("switch") or {}).get(name)
    elif name == "min_obstacle_clearance_m":
        block = row.get("crossing") or {}
        value = block.get(name)
        if value is None and row.get("min_obstacle_distance_m") is not None:
            radius = block.get("body_radius_m") or row.get("body_radius_m") or 0.35
            value = float(row["min_obstacle_distance_m"]) - float(radius)
    else:
        value = row.get(name)
    return None if value is None else float(value)


def active_gates(scene: str, rows: list[dict]) -> dict[str, float] | None:
    """没有 summary.md 时，从本仓库同名场景卡读取冻结门。"""
    path = ROOT / "scenes" / f"{scene}.json"
    if not path.is_file():
        return None
    card = json.loads(path.read_text(encoding="utf-8"))
    criteria = card.get("criteria") or {}
    gates = criteria.get("gates")
    if not criteria.get("frozen") or not gates:
        return None
    policy = rows[0].get("policy")
    form = rows[0].get("form") or "inprocess"
    density = rows[0].get("forest_density") or (card.get("forest") or {}).get("density")
    for field, value in (("gate_policies", policy), ("gate_forms", form), ("gate_densities", density)):
        allowed = criteria.get(field)
        if allowed is not None and value not in allowed:
            return None
    return {name: float(value) for name, value in gates.items()}


def gates_pass(rows: list[dict], gates: dict[str, float]) -> bool:
    for row in rows:
        for name, threshold in gates.items():
            value = gate_value(row, name)
            if value is None:
                return False
            if name in LOWER_GATES:
                if value + 1e-9 < threshold:
                    return False
            elif value - 1e-9 > threshold:
                return False
    if "arrival_rate" in gates:
        rates = [arrival_rate(row) for row in rows]
        if any(rate is None for rate in rates):
            return False
        mean = sum(rates) / len(rates)
        arrived = sum(int(row.get("arrived") or 0) for row in rows)
        slots = sum(int(row.get("n_drones") or 0) for row in rows)
        overall = arrived / slots if slots else None
        if mean + 1e-9 < gates["arrival_rate"] and (overall is None or overall + 1e-9 < gates["arrival_rate"]):
            return False
    if "collisions" in gates and sum(collisions(row) or 0 for row in rows) - 1e-9 > gates["collisions"]:
        return False
    return True


def group_data(rows: list[dict], report_only: bool, passed: bool | None) -> dict:
    seeds = sorted({row.get("seed") for row in rows if row.get("seed") is not None})
    seed_text = ",".join(str(seed) for seed in seeds) if seeds else "—"
    rates = [arrival_rate(row) for row in rows]
    arrival = None
    arrival_text = "—"
    if rows[0].get("task") != "hover" and all(rate is not None for rate in rates):
        if all(row.get("arrived") is not None and row.get("n_drones") for row in rows):
            arrived = sum(int(row["arrived"]) for row in rows)
            slots = sum(int(row["n_drones"]) for row in rows)
            arrival = arrived / slots
            arrival_text = f"{arrived}/{slots}（{arrival * 100:.2f}%）"
        else:
            arrival = sum(rates) / len(rates)
            arrival_text = f"{arrival * 100:.2f}%"
    counts = [collisions(row) for row in rows]
    collision_total = sum(counts) if all(count is not None for count in counts) else None
    distances = [float(row["min_inter_drone_distance_m"]) for row in rows if row.get("min_inter_drone_distance_m") is not None]
    return {
        "seeds": seed_text,
        "report_only": report_only,
        "passed": passed,
        "arrival": arrival,
        "arrival_text": arrival_text,
        "collisions": collision_total,
        "distance": min(distances) if distances else None,
    }


def load_groups(path: Path) -> dict[tuple[str, str], dict]:
    rows = json.loads((path / "results.json").read_text(encoding="utf-8"))
    grouped: dict[tuple[str, str], list[dict]] = {}
    for row in rows:
        grouped.setdefault((str(row["scene"]), str(row["policy"])), []).append(row)
    states, failures = summary_verdicts(path / "summary.md")
    result = {}
    for key, group in sorted(grouped.items()):
        task = group[0].get("task")
        if task == "hover":
            report_only = False
            passed = None if any(row.get("outcome") is None for row in group) else all(
                row["outcome"] == "SUCCESS" for row in group
            )
        else:
            gates = active_gates(key[0], group)
            # 自定义场景卡不在仓库时，先看结果中的冻结标记，再退回 outcome。
            card_exists = (ROOT / "scenes" / f"{key[0]}.json").is_file()
            embedded_frozen = (group[0].get(task) or {}).get("frozen") if task in {"formation", "crossing", "switch"} else None
            report_only = states.get(
                key,
                not bool(gates) if card_exists else (embedded_frozen is False),
            )
            if any(row.get("report_only") is True for row in group):
                report_only = True
            if report_only:
                passed = None
            elif key in states:
                passed = key not in failures
            elif gates:
                passed = gates_pass(group, gates)
            else:
                passed = None if any(row.get("outcome") is None for row in group) else all(
                    row["outcome"] == "SUCCESS" for row in group
                )
        result[key] = group_data(group, report_only, passed)
    return result


def load_run(path: Path) -> dict:
    smoke = path / "certified-smoke" / "results.json"
    return {
        "pytest": {
            "L0": pytest_counts(path / "pytest-l0.log"),
            "core": pytest_counts(path / "pytest-core.log"),
        },
        "groups": load_groups(path),
        "smoke": load_groups(smoke.parent) if smoke.is_file() else {},
    }


def signed(value: float | int | None, digits: int = 0, suffix: str = "") -> str:
    return "—" if value is None else f"{value:+.{digits}f}{suffix}"


def rate(groups: dict) -> tuple[int, int, int]:
    judged = [group for group in groups.values() if not group["report_only"]]
    return sum(group["passed"] is True for group in judged), len(judged), len(groups) - len(judged)


def render_pytest(current: dict, base: dict | None) -> list[str]:
    labels = " | ".join(PYTEST_NAMES)
    delta_labels = " | ".join(f"Δ{label}" for label in PYTEST_NAMES)
    header = f"| 套件 | {labels} |"
    if base:
        header = f"| 套件 | {labels} | {delta_labels} |"
    lines = ["## pytest", "", header, "|---" + "|---" * (len(PYTEST_FIELDS) * (2 if base else 1)) + "|"]
    for suite in ("L0", "core", "合计"):
        now = current["pytest"]
        old = base["pytest"] if base else None
        counts = {field: sum(now[name][field] for name in ("L0", "core")) for field in PYTEST_FIELDS} if suite == "合计" else now[suite]
        cells = [str(counts[field]) for field in PYTEST_FIELDS]
        if old:
            before = {field: sum(old[name][field] for name in ("L0", "core")) for field in PYTEST_FIELDS} if suite == "合计" else old[suite]
            cells.extend(f"{counts[field] - before[field]:+d}" for field in PYTEST_FIELDS)
        lines.append(f"| {suite} | " + " | ".join(cells) + " |")
    return lines


def render_groups(current: dict, base: dict | None) -> list[str]:
    head = "| 场景@策略 | seed | 是否通过 | 到达 | 碰撞 | 最小机间距 m | report-only |"
    if base is not None:
        head = "| 场景@策略 | seed | 是否通过 | 到达 | Δ到达（百分点） | 碰撞 | Δ碰撞 | 最小机间距 m | Δ最小机间距 m | report-only | 变动 |"
    lines = [head, "|" + "---|" * (head.count("|") - 1)]
    keys = sorted(set(current) | (set(base) if base is not None else set()))
    for key in keys:
        group = current.get(key)
        before = base.get(key) if base is not None else None
        name = f"{key[0]}@{key[1]}"
        if group is None:
            cells = [name, "—", "—", "—", "—", "—", "—"]
            if base is not None:
                cells = [name, "—", "—", "—", "—", "—", "—", "—", "—", "—", "缺失"]
        else:
            status = "—" if group["passed"] is None else ("是" if group["passed"] else "否")
            count = group["collisions"]
            distance = group["distance"]
            flag = "是" if group["report_only"] else "否"
            cells = [name, group["seeds"], status, group["arrival_text"]]
            if base is not None:
                delta_arrival = None if before is None or group["arrival"] is None or before["arrival"] is None else (group["arrival"] - before["arrival"]) * 100
                cells.append(signed(delta_arrival, 2))
            cells.append("—" if count is None else str(count))
            if base is not None:
                cells.append("—" if before is None or count is None or before["collisions"] is None else f"{count - before['collisions']:+d}")
            cells.append("—" if distance is None else f"{distance:.3f}")
            if base is not None:
                delta_distance = None if before is None or distance is None or before["distance"] is None else distance - before["distance"]
                cells.append(signed(delta_distance, 3))
            cells.append(flag)
            if base is not None:
                cells.append("新增" if before is None else "—")
        lines.append("| " + " | ".join(cells) + " |")
    return lines


def render(current: dict, base: dict | None = None) -> str:
    lines = render_pytest(current, base)
    passed, total, reports = rate(current["groups"])
    percentage = "—" if not total else f"{passed / total * 100:.2f}%"
    rate_line = f"批跑通过率（按场景）：{passed}/{total}（{percentage}）；report-only {reports} 场，不计入分母。"
    if base is not None:
        old_passed, old_total, old_reports = rate(base["groups"])
        old_percentage = "—" if not old_total else f"{old_passed / old_total * 100:.2f}%"
        delta = None if not total or not old_total else (passed / total - old_passed / old_total) * 100
        rate_line += f" 对照 {old_passed}/{old_total}（{old_percentage}），Δ {signed(delta, 2)} 个百分点；对照 report-only {old_reports} 场。"
    lines += ["", "## 批跑", "", rate_line, ""]
    lines += render_groups(current["groups"], base["groups"] if base is not None else None)
    if current["smoke"] or (base is not None and base["smoke"]):
        lines += ["", "## certified-smoke", "", "冒烟结果单列，不计入批跑通过率。", ""]
        lines += render_groups(current["smoke"], base["smoke"] if base is not None else None)
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path, help="regress OUT 目录")
    parser.add_argument("--base", type=Path, help="对照的 regress OUT 目录")
    args = parser.parse_args(argv)
    try:
        current = load_run(args.out)
        base = load_run(args.base) if args.base else None
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
        parser.error(str(error))
    print(render(current, base), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
