import json
import subprocess
import sys
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "check_scenes.py"


def _run(directory: Path, *, strict: bool = False) -> subprocess.CompletedProcess[str]:
    command = [sys.executable, str(SCRIPT)]
    if strict:
        command.append("--strict")
    command.append(str(directory))
    return subprocess.run(command, capture_output=True, text=True, check=False)


def _write(directory: Path, name: str, card: object) -> None:
    (directory / name).write_text(json.dumps(card), encoding="utf-8")


def test_valid_card_passes_default_and_strict(tmp_path: Path) -> None:
    _write(
        tmp_path,
        "valid.json",
        {"name": "hover", "drones": [{"id": 1}], "task": {}, "criteria": {}},
    )

    for strict in (False, True):
        result = _run(tmp_path, strict=strict)
        assert result.returncode == 0
        assert "warning:" not in result.stderr


@pytest.mark.parametrize(
    "card, expected",
    [
        ({"name": "hover", "drones": [{}], "task": {}}, "missing criteria"),
        (
            {"id": "hover", "vehicles": ["not-an-object"], "task": [], "criteria": {}},
            "vehicles must contain only objects",
        ),
    ],
)
def test_bad_card_warns_by_default_and_fails_strict(
    tmp_path: Path, card: object, expected: str
) -> None:
    _write(tmp_path, "bad.json", card)

    default = _run(tmp_path)
    assert default.returncode == 0
    assert "warning:" in default.stderr
    assert expected in default.stderr

    strict = _run(tmp_path, strict=True)
    assert strict.returncode != 0
    assert "warning:" in strict.stderr
    assert expected in strict.stderr


def test_geometry_json_is_not_treated_as_flight_card(tmp_path: Path) -> None:
    _write(tmp_path, "geometry.json", {"world": "empty", "boxes": []})
    result = _run(tmp_path, strict=True)
    assert result.returncode == 0
    assert result.stderr == ""
