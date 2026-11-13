"""Static checks for the shell scripts covered by the C10 ticket."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
EXCLUDED_SCRIPTS = {
    REPO_ROOT / "scripts" / "premerge.sh",
    REPO_ROOT / "scripts" / "regress.sh",
}
SHELL_SCRIPTS = sorted(
    script
    for pattern in ("scripts/*.sh", "scripts/distill/**/*.sh")
    for script in REPO_ROOT.glob(pattern)
    if script not in EXCLUDED_SCRIPTS
    and (REPO_ROOT / "scripts" / "sitl") not in script.parents
)


@pytest.mark.parametrize(
    "script", SHELL_SCRIPTS, ids=lambda script: str(script.relative_to(REPO_ROOT))
)
def test_bash_syntax(script: Path) -> None:
    result = subprocess.run(
        ["bash", "-n", str(script)], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize(
    "script", SHELL_SCRIPTS, ids=lambda script: str(script.relative_to(REPO_ROOT))
)
def test_shellcheck_errors(script: Path) -> None:
    shellcheck = shutil.which("shellcheck")
    if shellcheck is None:
        pytest.skip("未安装 shellcheck")

    result = subprocess.run(
        [shellcheck, "-S", "error", str(script)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
