from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

from yopo_l0.experiment import (
    comma_floats,
    comma_ints,
    comma_list,
    markdown_table,
    run_completion_jobs,
    run_spawn_jobs,
    unknown_values,
    write_json,
)

ROOT = Path(__file__).resolve().parents[1]


def _load_script(filename: str):
    spec = importlib.util.spec_from_file_location(filename.replace("-", "_"), ROOT / "scripts" / filename)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_comma_parsers_preserve_legacy_empty_item_behavior():
    assert comma_list("a,,b,") == ["a", "b"]
    assert comma_ints("0,,2,") == [0, 2]
    assert comma_floats(".05,,.2,") == [0.05, 0.2]
    assert unknown_values(["ok", "bad"], ["ok"]) == ["bad"]
    with pytest.raises(ValueError):
        comma_ints("not-an-int")


def test_json_and_markdown_rendering_are_byte_exact(tmp_path: Path):
    dest = tmp_path / "nested" / "result.json"
    write_json(dest, {"中文": [1, 2]})
    assert dest.read_bytes() == b'{\n  "\xe4\xb8\xad\xe6\x96\x87": [\n    1,\n    2\n  ]\n}\n'
    assert json.loads(dest.read_text(encoding="utf-8")) == {"中文": [1, 2]}
    assert markdown_table("| A | B |", "|---|---|", ["| 1 | 2 |"]) == "| A | B |\n|---|---|\n| 1 | 2 |"


def test_serial_job_shells_keep_order():
    assert run_spawn_jobs([3, 1], str, 1) == ["3", "1"]
    assert run_completion_jobs([3, 1], str, 1) == ["3", "1"]


def test_migrated_scripts_locked_small_rendering():
    c5 = _load_script("e-c5-corridor-radius.py")
    assert c5.render_summary([]) == (
        "| 卡 | 臂 | 到达 | 撞树 | 机间碰撞 | 超时 | 最小树净空 m | 最小机间 m |\n"
        "|---|---|---|---|---|---|---|---|"
    )
    b3c = _load_script("e-b3c-segment-penalty.py")
    assert b3c.summarize([], title="锁定") == (
        "## 锁定\n\n"
        "| 场景 | 臂 | 到达 | 撞树 | 机间碰撞 | 超时 | 最小机间 m | 后段 p50 / p95 ms | 预算截断 |\n"
        "|---|---|---|---|---|---|---|---|---|\n\n"
        "### 分种子\n\n"
        "| 场景 | 臂 | seed | 结果 | 到达 | 撞树 | 机间 | 最小机间 m | 后段 p50 ms | 后段 p95 ms |\n"
        "|---|---|---|---|---|---|---|---|---|---|\n"
    )
