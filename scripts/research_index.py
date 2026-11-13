#!/usr/bin/env python3
"""Build the one-level index for research reports."""

from __future__ import annotations

import sys
from pathlib import Path

from results_index import main


REPO_ROOT = Path(__file__).resolve().parents[1]


if __name__ == "__main__":
    raise SystemExit(main(["--research", "--root", str(REPO_ROOT / "docs" / "research"), *sys.argv[1:]]))
