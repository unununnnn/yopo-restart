#!/usr/bin/env python3
"""检查 Markdown 相对链接是否指向存在的文件。

用法：
  .venv/bin/python scripts/check_doc_links.py
  .venv/bin/python scripts/check_doc_links.py docs/LIMITATIONS.md
  .venv/bin/python scripts/check_doc_links.py --root /path/to/repo

默认检查仓库根目录的 README.md 和 docs/ 下全部 .md。
http(s)、mailto 和只含页内锚点的链接跳过。相对路径按源文件所在目录解析。
跳过围栏代码块和行内代码里的文字，避免把示例路径当成链接。
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

FENCE_RE = re.compile(r"```.*?```|~~~.*?~~~", re.DOTALL)
INLINE_CODE_RE = re.compile(r"`[^`]+`")
# [text](url) 或 [text](<url>)，允许标题。
# 角括号是可选包装；不能用 (<?) 做条件组，因为空捕获仍算“已参与”。
_LINK_DEST = r"(?:<([^>\n]+)>|([^)\s]+))"
_LINK_TITLE = r"(?:\s+(?:\"[^\"]*\"|'[^']*'|\([^)]*\)))?"
INLINE_LINK_RE = re.compile(
    r"(?<!\!)\[(?:[^\]]|\\.)*?\]\(\s*" + _LINK_DEST + _LINK_TITLE + r"\s*\)"
)
IMAGE_LINK_RE = re.compile(
    r"!\[(?:[^\]]|\\.)*?\]\(\s*" + _LINK_DEST + _LINK_TITLE + r"\s*\)"
)
REF_DEF_RE = re.compile(
    r"^\s*\[(?:[^\]]+)\]:\s*" + _LINK_DEST + _LINK_TITLE + r"\s*$",
    re.MULTILINE,
)

SKIP_SCHEMES = ("http://", "https://", "mailto:", "ftp://", "irc:", "data:")


def strip_markup(text: str) -> str:
    text = FENCE_RE.sub("", text)
    return INLINE_CODE_RE.sub("``", text)


def link_url(match: re.Match) -> str:
    url = match.group(1) or match.group(2) or ""
    return url.strip()


def extract_urls(text: str) -> list[str]:
    cleaned = strip_markup(text)
    urls: list[str] = []
    for rx in (INLINE_LINK_RE, IMAGE_LINK_RE, REF_DEF_RE):
        for match in rx.finditer(cleaned):
            urls.append(link_url(match))
    return urls


def is_external(url: str) -> bool:
    lower = url.lower()
    if lower.startswith(SKIP_SCHEMES):
        return True
    # 协议相对
    if lower.startswith("//"):
        return True
    return False


def resolve_target(source: Path, url: str) -> Path | None:
    """返回应对应到本地文件的路径。外部链接和纯锚点返回 None（表示不检查文件）。"""
    if is_external(url):
        return None
    path, _, _frag = url.partition("#")
    if not path:
        return None
    return (source.parent / path).resolve()


def iter_markdown(root: Path, paths: list[Path] | None) -> list[Path]:
    if paths:
        return [p.resolve() for p in paths]
    files = [root / "README.md"]
    docs = root / "docs"
    if docs.is_dir():
        files.extend(sorted(docs.rglob("*.md")))
    return [p for p in files if p.is_file()]


def check_file(path: Path) -> list[tuple[str, str]]:
    """返回 (url, reason) 列表。"""
    text = path.read_text(encoding="utf-8")
    broken: list[tuple[str, str]] = []
    seen: set[str] = set()
    for url in extract_urls(text):
        if url in seen:
            continue
        seen.add(url)
        target = resolve_target(path, url)
        if target is None:
            continue
        if not target.exists():
            broken.append((url, f"不存在：{target}"))
    return broken


# 仓库里已经存在、本脚本不修的坏链。只允许这一条。
KNOWN_BROKEN: dict[str, set[str]] = {
    "docs/adr/0006-yopo-minco-baseline.md": {
        "../deploy/ros2-uav-node.md",
    },
}


def check_tree(root: Path, paths: list[Path] | None = None, *, use_allowlist: bool = True):
    findings = []
    for path in iter_markdown(root, paths):
        rel = path.relative_to(root).as_posix() if path.is_relative_to(root) else str(path)
        allowed = KNOWN_BROKEN.get(rel, set()) if use_allowlist else set()
        for url, reason in check_file(path):
            if url in allowed:
                continue
            findings.append((rel, url, reason))
    return findings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", type=Path, help="要检查的 Markdown 文件；默认 README.md 与 docs/")
    parser.add_argument("--root", type=Path, default=ROOT, help="仓库根目录")
    parser.add_argument("--no-allowlist", action="store_true", help="连已知坏链一并报出来")
    args = parser.parse_args(argv)
    root = args.root.resolve()
    paths = [p if p.is_absolute() else root / p for p in args.paths]
    findings = check_tree(root, paths or None, use_allowlist=not args.no_allowlist)
    if not findings:
        print("相对链接全部可解析")
        return 0
    print(f"有 {len(findings)} 条相对链接无法解析：", file=sys.stderr)
    for rel, url, reason in findings:
        print(f"  {rel}: [{url}] {reason}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
