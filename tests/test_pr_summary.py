"""PR 摘要只读取回归产物，不运行批跑或仿真。"""

import json
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/pr_summary.py"
SAMPLE = Path(
    "/workspace/wt/yopo-restart/yopo-restart-bot-s1b-rtk-positioning/out/regress/20261112T075129"
)


def row(scene, policy, task, seed, outcome, arrived, n_drones, distance=None, crossing=None):
    """字段按 batch.py 写出的 results.json 逐 seed 结构。"""
    result = {
        "scene": scene,
        "policy": policy,
        "task": task,
        "seed": seed,
        "outcome": outcome,
        "arrived": arrived,
        "n_drones": n_drones,
        "arrival_time_s": 4.0 if outcome == "SUCCESS" and task != "hover" else None,
        "min_inter_drone_distance_m": distance,
    }
    if crossing is not None:
        result["crossing"] = crossing
    if task == "crossing":
        result["arrival_rate"] = arrived / n_drones
        result["formation_error_p95_m"] = 4.0
        result["min_obstacle_distance_m"] = 0.9
    return result


def out_dir(path, rows, *, l0, core, smoke=None):
    path.mkdir()
    (path / "pytest-l0.log").write_text(l0, encoding="utf-8")
    (path / "pytest-core.log").write_text(core, encoding="utf-8")
    (path / "results.json").write_text(json.dumps(rows, ensure_ascii=False), encoding="utf-8")
    if smoke is not None:
        smoke_dir = path / "certified-smoke"
        smoke_dir.mkdir()
        (smoke_dir / "results.json").write_text(json.dumps(smoke, ensure_ascii=False), encoding="utf-8")
    return path


def run_summary(*args):
    return subprocess.run(
        [sys.executable, str(SCRIPT), *(str(arg) for arg in args)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout


def test_summary_is_deterministic_and_excludes_report_only(tmp_path):
    rows = [
        row("g2-hover", "hover", "hover", 1, "SUCCESS", 0, 1),
        row("g2-hover", "hover", "hover", 0, "SUCCESS", 0, 1),
        row("g2-pillar", "upstream", "goto", 0, "SUCCESS", 1, 1),
        row("formation-drop", "formation", "formation", 0, "SUCCESS", 3, 3, 1.8),
        row(
            "crossing-pillars", "crossing", "crossing", 0, "COLLISION", 2, 3, 1.5,
            {"obstacle_collisions": 1, "inter_drone_collisions": 0},
        ),
    ]
    smoke = [row("g2-pillar", "certified", "goto", 0, "TIMEOUT", 0, 1)]
    path = out_dir(
        tmp_path / "now", rows,
        l0="...\n3 passed, 1 skipped, 2 deselected in 0.12s\n",
        core="...\n2 passed, 1 failed, 1 warning in 0.20s\n",
        smoke=smoke,
    )
    output = run_summary(path)
    assert output == run_summary(path)
    assert "| L0 | 3 | 0 | 1 | 2 |" in output
    assert "| core | 2 | 1 | 0 | 0 |" in output
    assert "| 合计 | 5 | 1 | 1 | 2 |" in output
    assert "批跑通过率（按场景）：2/3（66.67%）；report-only 1 场" in output
    assert "| crossing-pillars@crossing | 0 | 否 | 2/3（66.67%） | 1 | 1.500 | 否 |" in output
    assert "| formation-drop@formation | 0 | — | 3/3（100.00%） | 0 | 1.800 | 是 |" in output
    assert "| g2-hover@hover | 0,1 | 是 | — | 0 | — | 否 |" in output
    assert "## certified-smoke" in output
    assert "| g2-pillar@certified | 0 | — | 0/1（0.00%） | 0 | — | 是 |" in output


def test_base_deltas_and_scene_set_changes(tmp_path):
    now = out_dir(
        tmp_path / "now",
        [
            row("g2-hover", "hover", "hover", 0, "SUCCESS", 0, 1),
            row("g2-pillar", "upstream", "goto", 0, "SUCCESS", 1, 1),
            row(
                "crossing-pillars", "crossing", "crossing", 0, "COLLISION", 2, 3, 1.5,
                {"obstacle_collisions": 1, "inter_drone_collisions": 0},
            ),
            row("formation-drop", "formation", "formation", 0, "SUCCESS", 3, 3),
        ],
        l0="3 passed, 1 skipped in 0.12s\n",
        core="2 passed, 1 failed in 0.20s\n",
    )
    base = out_dir(
        tmp_path / "base",
        [
            row("g2-hover", "hover", "hover", 0, "TIMEOUT", 0, 1),
            row("g2-pillar", "upstream", "goto", 0, "COLLISION", 0, 1),
            row(
                "crossing-pillars", "crossing", "crossing", 0, "COLLISION", 1, 3, 1.2,
                {"obstacle_collisions": 2, "inter_drone_collisions": 0},
            ),
            row("narrow-corridor", "hover", "hover", 0, "SUCCESS", 0, 1),
        ],
        l0="2 passed, 2 skipped in 0.11s\n",
        core="2 passed in 0.18s\n",
    )
    output = run_summary(now, "--base", base)
    assert "| L0 | 3 | 0 | 1 | 0 | 0 | 0 | 0 | 0 | +1 | +0 | -1 |" in output
    assert "批跑通过率（按场景）：2/3（66.67%）" in output
    assert "对照 1/4（25.00%），Δ +41.67 个百分点" in output
    assert "| crossing-pillars@crossing | 0 | 否 | 2/3（66.67%） | +33.33 | 1 | -1 | 1.500 | +0.300 | 否 | — |" in output
    assert "| formation-drop@formation | 0 | — | 3/3（100.00%） | — | 0 | — | — | — | 是 | 新增 |" in output
    assert "| narrow-corridor@hover | — | — | — | — | — | — | — | — | — | 缺失 |" in output


@pytest.mark.skipif(not SAMPLE.is_dir(), reason="只读回归样例不存在")
def test_real_regress_sample():
    output = run_summary(SAMPLE)
    assert "批跑通过率（按场景）：12/12（100.00%）；report-only 5 场" in output
    assert "## certified-smoke" in output
    assert "| g2-pillar@certified | 0 | — | 0/1（0.00%） | 0 | — | 是 |" in output
