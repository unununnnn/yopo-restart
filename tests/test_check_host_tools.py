"""Tests for the host-tool preflight check with an isolated PATH."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "check-host-tools.py"


def _run_with_tools(tmp_path: Path, *tools: str) -> subprocess.CompletedProcess[str]:
    for tool in tools:
        executable = tmp_path / tool
        executable.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        executable.chmod(0o755)

    env = os.environ.copy()
    env["PATH"] = str(tmp_path)
    return subprocess.run(
        [sys.executable, str(SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )


def test_all_required_and_optional_tools_are_found(tmp_path):
    result = _run_with_tools(tmp_path, "make", "python3", "bash", "shellcheck", "docker")

    assert result.returncode == 0
    for tool in ("make", "python3", "bash", "shellcheck", "docker"):
        assert f"{tool}: found" in result.stdout
    assert "all required tools found" in result.stdout


def test_missing_make_fails_and_names_it(tmp_path):
    result = _run_with_tools(tmp_path, "python3", "bash")

    assert result.returncode != 0
    assert "make: MISSING" in result.stdout
    assert "missing required: make" in result.stdout


def test_missing_optional_tools_are_skipped(tmp_path):
    result = _run_with_tools(tmp_path, "make", "python3", "bash")

    assert result.returncode == 0
    assert "shellcheck: skip (not found)" in result.stdout
    assert "docker: skip (not found)" in result.stdout
