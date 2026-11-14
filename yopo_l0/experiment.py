"""Side-effect-free helpers shared by command-line experiment scripts."""

from __future__ import annotations

import json
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import Callable, Iterable, Sequence, TypeVar

T = TypeVar("T")
R = TypeVar("R")


def comma_list(value: str) -> list[str]:
    return [item for item in value.split(",") if item]


def comma_ints(value: str) -> list[int]:
    return [int(item) for item in value.split(",") if item != ""]


def comma_floats(value: str) -> list[float]:
    return [float(item) for item in value.split(",") if item]


def unknown_values(values: Iterable[T], allowed: Iterable[T]) -> list[T]:
    """Return unknown selections in their original command-line order."""
    known = tuple(allowed)
    return [value for value in values if value not in known]


def run_spawn_jobs(specs: Sequence[T], worker: Callable[[T], R], jobs: int) -> list[R]:
    """Run ordered jobs using the spawn behavior required by CPU-heavy experiments."""
    if jobs <= 1 or len(specs) <= 1:
        return [worker(spec) for spec in specs]
    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(worker, specs, chunksize=1)


def run_completion_jobs(specs: Sequence[T], worker: Callable[[T], R], jobs: int) -> list[R]:
    """Run jobs and return them in completion order, preserving legacy behavior."""
    if jobs <= 1:
        return [worker(spec) for spec in specs]
    rows: list[R] = []
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        futures = [pool.submit(worker, spec) for spec in specs]
        for future in as_completed(futures):
            rows.append(future.result())
    return rows


def write_json(path: Path, payload: object) -> None:
    """Write canonical experiment JSON: UTF-8, indented, with a final newline."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json_text(payload), encoding="utf-8")


def json_text(payload: object, *, indent: int | None = 2, final_newline: bool = True) -> str:
    """Render JSON while allowing legacy scripts to retain their exact indentation."""
    text = json.dumps(payload, ensure_ascii=False, indent=indent)
    return text + ("\n" if final_newline else "")


def markdown_table(header: str, separator: str, rows: Sequence[str]) -> str:
    """Join preformatted table rows without changing experiment-specific wording."""
    return "\n".join([header, separator, *rows])
