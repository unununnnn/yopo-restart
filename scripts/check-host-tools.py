#!/usr/bin/env python3
"""Check host commands needed by the merge regression workflow."""

from __future__ import annotations

import shutil


REQUIRED_TOOLS = ("make", "python3", "bash")
OPTIONAL_TOOLS = ("shellcheck", "docker")


def main() -> int:
    missing = []

    for tool in REQUIRED_TOOLS:
        path = shutil.which(tool)
        if path is None:
            missing.append(tool)
            print(f"required: {tool}: MISSING")
        else:
            print(f"required: {tool}: found ({path})")

    for tool in OPTIONAL_TOOLS:
        path = shutil.which(tool)
        if path is None:
            print(f"optional: {tool}: skip (not found)")
        else:
            print(f"optional: {tool}: found ({path})")

    if missing:
        print(f"host tool check failed; missing required: {', '.join(missing)}")
        return 1

    print("host tool check passed; all required tools found")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
