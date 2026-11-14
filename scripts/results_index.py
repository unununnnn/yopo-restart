#!/usr/bin/env python3
"""Build the one-level index for result reports."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RESULTS = REPO_ROOT / "docs" / "results"
JSON_EXPLANATION_KEYS = ("title", "disclaimer", "description", "summary", "note")


def ticket_from_name(name: str) -> str:
    # Ticket IDs are the first filename component when it is either a number
    # optionally followed by letters (06c, 13d, 11) or m + number (m0).
    # Generic artifacts such as SUMMARY.json and videos.md intentionally have
    # no inferred ticket ID.
    match = re.match(r"(?i)^((?:\d+[a-z]*|m\d+))(?:-|\.)", name)
    return match.group(1).lower() if match else ""


def markdown_title(text: str) -> str:
    """Return the first non-empty Markdown ATX heading, without its # marks."""
    for line in text.splitlines():
        match = re.match(r"^\s{0,3}#{1,6}\s+(.+?)\s*#*\s*$", line)
        if match:
            return match.group(1).strip()
    return "（无标题）"


def json_title(value: Any) -> str:
    """Choose the first non-empty title/disclaimer/explanation, in that order."""
    candidates = value if isinstance(value, list) else [value]
    for key in JSON_EXPLANATION_KEYS:
        for candidate in candidates:
            if isinstance(candidate, dict):
                item = candidate.get(key)
                if isinstance(item, str) and item.strip():
                    return item.strip().splitlines()[0]
    return "（无说明字段）"


def marker(value: Any) -> str:
    if isinstance(value, str):
        text = value
    else:
        text = json.dumps(value, ensure_ascii=False, sort_keys=True)
    report_only = "report-only" in text.lower() or "只报数" in text
    # “未冻结” is explicitly not a frozen marker.  Accept the existing
    # affirmative forms 已冻结/已冻 and a standalone 冻结 token.
    without_negative = re.sub(r"(?:尚|还)?未冻结", "", text)
    frozen = bool(re.search(r"已冻结|已冻|(?<!未)冻结", without_negative))
    labels = [label for yes, label in ((report_only, "report-only"), (frozen, "冻结")) if yes]
    return "、".join(labels) if labels else "无"


def escape_cell(value: str) -> str:
    return value.replace("|", r"\|").replace("\n", " ")


def research_ticket_from_name(name: str) -> str:
    # Research tickets are either a letter/number component (d2b, p1) or an
    # E-series prefix plus that component (e-a1, e-c5b).  Descriptive names
    # such as ego-swarm-* and e-combo-* intentionally do not infer a ticket.
    match = re.match(r"(?i)^((?:e-)?[a-z]\d+[a-z]*)(?:-|\.)", name)
    return match.group(1).lower() if match else ""


def render(results_dir: Path, *, research: bool = False) -> str:
    rows: list[tuple[str, str, str, str]] = []
    files = sorted(
        (*results_dir.glob("*.md"), *results_dir.glob("*.json")),
        key=lambda path: path.name,
    )
    for path in files:
        if path.name == "INDEX.md":
            continue
        text = path.read_text(encoding="utf-8")
        if path.suffix == ".md":
            title = markdown_title(text)
            parsed: Any = text
        else:
            parsed = json.loads(text)
            title = json_title(parsed)
        ticket = research_ticket_from_name(path.name) if research else ticket_from_name(path.name)
        rows.append((path.name, ticket, title, marker(parsed)))

    lines = [
        "# 研究目录索引" if research else "# 结果目录索引",
        "",
        (
            "<!-- 由 scripts/research_index.py 生成；请勿手工编辑。 -->"
            if research
            else "<!-- 由 scripts/results_index.py 生成；请勿手工编辑。 -->"
        ),
        "",
        "| 文件名 | 票号 | 标题首行 | report-only / 冻结标记 |",
        "|---|---|---|---|",
    ]
    lines.extend("| " + " | ".join(escape_cell(cell) for cell in row) + " |" for row in rows)
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_RESULTS, help="results directory")
    parser.add_argument("--research", action="store_true", help="use research index headings and ticket IDs")
    parser.add_argument("--check", action="store_true", help="fail if INDEX.md is absent or stale")
    args = parser.parse_args(argv)

    expected = render(args.root, research=args.research)
    index = args.root / "INDEX.md"
    if args.check:
        if not index.is_file() or index.read_text(encoding="utf-8") != expected:
            print(f"results index is missing or stale: {index}", file=sys.stderr)
            return 1
        return 0
    index.write_text(expected, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
