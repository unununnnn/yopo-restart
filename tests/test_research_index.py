from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from scripts import results_index


RESEARCH_SCRIPT = Path(__file__).parents[1] / "scripts" / "research_index.py"


def run_research(root: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(RESEARCH_SCRIPT), "--root", str(root), *args],
        text=True,
        capture_output=True,
        check=False,
    )


def test_research_fields_ticket_rules_and_stable_sort(tmp_path: Path) -> None:
    (tmp_path / "e-c5b-track-clearance.md").write_text(
        "# 票 E-C5b：走廊\n\n只报数。\n", encoding="utf-8"
    )
    (tmp_path / "d2b-teacher-data.md").write_text(
        "\n# D2b：教师数据\n", encoding="utf-8"
    )
    (tmp_path / "e-a1-inter-drone-filter.md").write_text(
        "# 票 E-A1：机间过滤\n\nreport-only\n", encoding="utf-8"
    )
    (tmp_path / "ego-swarm-baseline.md").write_text("# 基线\n", encoding="utf-8")

    assert run_research(tmp_path).returncode == 0
    index = (tmp_path / "INDEX.md").read_text(encoding="utf-8")
    assert index.index("d2b-teacher-data.md") < index.index("e-a1-inter-drone-filter.md")
    assert "| d2b-teacher-data.md | d2b | D2b：教师数据 | 无 |" in index
    assert "| e-a1-inter-drone-filter.md | e-a1 | 票 E-A1：机间过滤 | report-only |" in index
    assert "| e-c5b-track-clearance.md | e-c5b | 票 E-C5b：走廊 | report-only |" in index
    assert "| ego-swarm-baseline.md |  | 基线 | 无 |" in index


def test_research_check_current_missing_stale_and_byte_stable(tmp_path: Path) -> None:
    (tmp_path / "d1-report.md").write_text("# D1 报告\n", encoding="utf-8")
    assert run_research(tmp_path, "--check").returncode != 0
    assert run_research(tmp_path).returncode == 0
    generated = (tmp_path / "INDEX.md").read_bytes()
    assert run_research(tmp_path, "--check").returncode == 0
    assert run_research(tmp_path).returncode == 0
    assert (tmp_path / "INDEX.md").read_bytes() == generated
    (tmp_path / "INDEX.md").write_text("stale\n", encoding="utf-8")
    assert run_research(tmp_path, "--check").returncode != 0


def test_research_hint_current_has_no_warning(tmp_path: Path) -> None:
    (tmp_path / "d1-report.md").write_text("# D1 报告\n", encoding="utf-8")
    assert run_research(tmp_path).returncode == 0

    hinted = run_research(tmp_path, "--hint")

    assert hinted.returncode == 0
    assert hinted.stderr == ""


def test_research_hint_missing_and_stale_warn_without_failing(tmp_path: Path) -> None:
    (tmp_path / "d1-report.md").write_text("# D1 报告\n", encoding="utf-8")

    missing = run_research(tmp_path, "--hint")
    assert missing.returncode == 0
    assert str(tmp_path / "INDEX.md") in missing.stderr
    assert "missing or stale" in missing.stderr
    assert "does not block merging" in missing.stderr

    (tmp_path / "INDEX.md").write_text("stale\n", encoding="utf-8")
    stale = run_research(tmp_path, "--hint")
    assert stale.returncode == 0
    assert str(tmp_path / "INDEX.md") in stale.stderr
    assert "missing or stale" in stale.stderr
    assert "does not block merging" in stale.stderr


def test_default_results_render_is_locked_byte_for_byte(tmp_path: Path) -> None:
    (tmp_path / "06c-report.md").write_text(
        "# 票 06c：结果 | 样例\n\nreport-only，已冻结。\n", encoding="utf-8"
    )
    expected = (
        "# 结果目录索引\n\n"
        "<!-- 由 scripts/results_index.py 生成；请勿手工编辑。 -->\n\n"
        "| 文件名 | 票号 | 标题首行 | report-only / 冻结标记 |\n"
        "|---|---|---|---|\n"
        "| 06c-report.md | 06c | 票 06c：结果 \\| 样例 | report-only、冻结 |\n"
    )
    assert results_index.render(tmp_path) == expected
