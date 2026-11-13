#!/usr/bin/env python3
"""Check the read-only structural contract of flight scene JSON files."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


# A flight card is distinguished from geometry JSON by having at least one of
# the flight-only keys below.  Flight cards must then contain all four concepts:
# a non-empty string ``name`` or ``id``; a non-empty list of objects in
# ``drones`` or ``vehicles``; and object-valued ``task`` and ``criteria``.
ID_KEYS = ("name", "id")
VEHICLE_KEYS = ("drones", "vehicles")
FLIGHT_KEYS = (*VEHICLE_KEYS, "task", "criteria")


def _alias_value(card: dict[str, Any], aliases: tuple[str, ...]) -> tuple[str, Any] | None:
    for key in aliases:
        if key in card:
            return key, card[key]
    return None


def validate_card(card: Any) -> list[str]:
    """Return structural errors for a flight card, or none for geometry JSON."""
    if not isinstance(card, dict):
        return ["top-level JSON value must be an object"]
    if not any(key in card for key in FLIGHT_KEYS):
        return []

    errors: list[str] = []
    scene_id = _alias_value(card, ID_KEYS)
    if scene_id is None:
        errors.append("missing scene id (name or id)")
    elif not isinstance(scene_id[1], str) or not scene_id[1].strip():
        errors.append(f"{scene_id[0]} must be a non-empty string")

    vehicles = _alias_value(card, VEHICLE_KEYS)
    if vehicles is None:
        errors.append("missing vehicles (drones or vehicles)")
    elif not isinstance(vehicles[1], list) or not vehicles[1]:
        errors.append(f"{vehicles[0]} must be a non-empty list of objects")
    elif any(not isinstance(vehicle, dict) for vehicle in vehicles[1]):
        errors.append(f"{vehicles[0]} must contain only objects")

    for key in ("task", "criteria"):
        if key not in card:
            errors.append(f"missing {key}")
        elif not isinstance(card[key], dict):
            errors.append(f"{key} must be an object")
    return errors


def check_directory(directory: Path) -> list[tuple[Path, str]]:
    warnings: list[tuple[Path, str]] = []
    for path in sorted(directory.glob("*.json")):
        try:
            card = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            warnings.append((path, f"cannot read JSON: {exc}"))
            continue
        warnings.extend((path, error) for error in validate_card(card))
    return warnings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "directory",
        nargs="?",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "scenes",
        help="directory whose top-level *.json files are checked (default: scenes/)",
    )
    parser.add_argument("--strict", action="store_true", help="fail when warnings are found")
    args = parser.parse_args(argv)

    warnings = check_directory(args.directory)
    for path, message in warnings:
        print(f"warning: {path}: {message}", file=sys.stderr)
    return 1 if args.strict and warnings else 0


if __name__ == "__main__":
    raise SystemExit(main())
