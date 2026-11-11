"""文档相对链接必须指向存在的文件。"""

import importlib.util
import stat
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_doc_links.py"


def _load():
    spec = importlib.util.spec_from_file_location("check_doc_links", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_limitations_relative_links_resolve():
    check = _load()
    path = ROOT / "docs" / "LIMITATIONS.md"
    assert path.is_file()
    urls = check.extract_urls(path.read_text(encoding="utf-8"))
    relative = [u for u in urls if not check.is_external(u) and not u.startswith("#")]
    assert relative, "docs/LIMITATIONS.md 里没有可检查的相对链接（空文档不能算通过）"
    broken = check.check_file(path)
    assert broken == [], "docs/LIMITATIONS.md 有无法解析的相对链接：\n" + "\n".join(
        f"  {url} ({reason})" for url, reason in broken
    )


def test_repo_docs_relative_links_resolve():
    check = _load()
    findings = check.check_tree(ROOT)
    assert findings == [], "文档有无法解析的相对链接：\n" + "\n".join(
        f"  {rel}: {url} ({reason})" for rel, url, reason in findings
    )


def test_checker_reports_missing_relative_target(tmp_path):
    check = _load()
    src = tmp_path / "note.md"
    src.write_text("见 [缺页](missing.md) 和 [外链](https://example.com/x)。\n", encoding="utf-8")
    broken = check.check_file(src)
    assert len(broken) == 1
    assert broken[0][0] == "missing.md"


def test_checker_accepts_existing_relative_target(tmp_path):
    check = _load()
    (tmp_path / "target.md").write_text("# t\n", encoding="utf-8")
    src = tmp_path / "note.md"
    src.write_text(
        "见 [页](target.md#t)、[括号](<target.md>)、[本页](#here)。\n\n[ref]: target.md\n",
        encoding="utf-8",
    )
    assert check.check_file(src) == []


def test_checker_ignores_links_in_code_fences(tmp_path):
    check = _load()
    src = tmp_path / "note.md"
    src.write_text("```\n[假](nowhere.md)\n```\n还有 ` [也假](nowhere.md) `。\n", encoding="utf-8")
    assert check.check_file(src) == []


def test_script_is_executable_bit_optional_but_runs():
    # 脚本本身要能作为文件被加载；不强制 chmod +x，避免各 worktree 的 umask 差。
    assert SCRIPT.is_file()
    mode = SCRIPT.stat().st_mode
    assert stat.S_ISREG(mode)
