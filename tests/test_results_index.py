from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from scripts import results_index


SCRIPT = Path(__file__).parents[1] / "scripts" / "results_index.py"


def run_index(root: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--root", str(root), *args],
        text=True,
        capture_output=True,
        check=False,
    )


def test_generates_sorted_fields_and_stable_inference(tmp_path: Path) -> None:
    (tmp_path / "13d-dual-track.md").write_text(
        "\n# 票 13d 双轨\n\n判据未冻结 / report-only。\n", encoding="utf-8"
    )
    (tmp_path / "06c-drop20.md").write_text(
        "# 票 06c：六机 | drop 20%\n\n本项已冻结。\n", encoding="utf-8"
    )
    (tmp_path / "m0-g2-pillar.md").write_text(
        "# M0 样例：g2-pillar\n\n碰撞只报数。\n", encoding="utf-8"
    )
    (tmp_path / "12e-data.json").write_text(
        '{"disclaimer": "仅为说明", "title": "JSON 标题", "note": "已冻结"}',
        encoding="utf-8",
    )
    (tmp_path / "SUMMARY.json").write_text('{"schema": "example/1"}', encoding="utf-8")

    completed = run_index(tmp_path)

    assert completed.returncode == 0
    index = (tmp_path / "INDEX.md").read_text(encoding="utf-8")
    assert index.index("06c-drop20.md") < index.index("12e-data.json") < index.index("13d-dual-track.md")
    assert "| 06c-drop20.md | 06c | 票 06c：六机 \\| drop 20% | 冻结 |" in index
    assert "| 12e-data.json | 12e | JSON 标题 | 冻结 |" in index
    assert "| 13d-dual-track.md | 13d | 票 13d 双轨 | report-only |" in index
    assert "| m0-g2-pillar.md | m0 | M0 样例：g2-pillar | report-only |" in index
    assert "| SUMMARY.json |  | （无说明字段） | 无 |" in index


def test_check_accepts_current_index_and_generation_is_byte_stable(tmp_path: Path) -> None:
    (tmp_path / "11-report.md").write_text("# 报告\n\n无标记。\n", encoding="utf-8")
    assert run_index(tmp_path).returncode == 0
    first = (tmp_path / "INDEX.md").read_bytes()
    assert run_index(tmp_path, "--check").returncode == 0
    assert run_index(tmp_path).returncode == 0
    assert (tmp_path / "INDEX.md").read_bytes() == first


def test_check_rejects_missing_and_stale_index(tmp_path: Path) -> None:
    (tmp_path / "07c-report.md").write_text("# 报告\n", encoding="utf-8")
    missing = run_index(tmp_path, "--check")
    assert missing.returncode != 0
    assert "missing or stale" in missing.stderr

    (tmp_path / "INDEX.md").write_text("stale\n", encoding="utf-8")
    stale = run_index(tmp_path, "--check")
    assert stale.returncode != 0
    assert "missing or stale" in stale.stderr


def test_root_keeps_output_isolated_to_tmp_directory(tmp_path: Path) -> None:
    outside = tmp_path / "outside"
    results = tmp_path / "fixture-results"
    outside.mkdir()
    results.mkdir()
    (results / "06c-only.md").write_text("# 独立夹具\n", encoding="utf-8")

    completed = subprocess.run(
        [sys.executable, str(SCRIPT), "--root", str(results)],
        cwd=outside,
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 0
    assert (results / "INDEX.md").is_file()
    assert not (outside / "INDEX.md").exists()
