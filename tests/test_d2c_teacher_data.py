"""票 D2c：全量教师清单覆盖 1792 场，并保留 D2b 收口清单和假设标注。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from teacher.d2b import sha256_file
from teacher.d2b_grid import DEFAULT_SEED_COUNT, grid_size, iter_d2b_scenarios

_REPO = Path(__file__).resolve().parents[1]
_D2C = _REPO / "docs" / "research" / "d2c-teacher-manifest.json"
_D2B = _REPO / "docs" / "research" / "d2b-teacher-manifest.json"
_D2C_DOC = _REPO / "docs" / "research" / "d2c-teacher-data.md"
_WORK = Path("/workspace/yopo-teacher-data/d2b")


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_d2c_manifest_covers_the_planned_grid_and_keeps_the_assumption():
    manifest = _load(_D2C)
    assert manifest["schema"] == "yopo-teacher-d2b-manifest/1"
    assert "不是论文复现" in manifest["assumption"]
    assert manifest["planned_n"] == 1792
    assert manifest["n_completed_runs"] == 1792
    assert manifest["n_success"] == 1205
    assert manifest["n_fail"] == 553
    assert manifest["n_unknown"] == 34
    assert manifest["live_batch_left_running"] is False
    assert manifest["n_files"] == len(manifest["files"])
    assert manifest["n_files"] == 7171
    assert manifest["resume"]["skip"] == 543
    assert manifest["resume"]["ok"] == 1249
    assert manifest["resume"]["fail"] == 0
    assert "不是编排器出错" in manifest["resume"]["skip_meaning"]

    planned = {row["id"] for row in iter_d2b_scenarios(seed_count=DEFAULT_SEED_COUNT, include_six_open=True)}
    assert len(planned) == grid_size(DEFAULT_SEED_COUNT, True) == 1792

    run_files = [row for row in manifest["files"] if row["path"].startswith("runs/")]
    ids = {row["path"].split("/")[1] for row in run_files}
    kinds_by_id = {}
    for row in run_files:
        kinds_by_id.setdefault(row["path"].split("/")[1], set()).add(row["kind"])
    assert ids == planned
    assert all(kinds == {"trajectory.npz", "record.json", "scene.json", "geometry.json"} for kinds in kinds_by_id.values())
    assert all(len(row["sha256"]) == 64 and row["bytes"] >= 0 for row in manifest["files"])


def test_d2b_midrun_manifest_is_left_unchanged():
    manifest = _load(_D2B)
    assert manifest["n_completed_runs"] == 663
    assert manifest["n_success"] == 635
    assert manifest["n_fail"] == 28
    assert manifest["n_unknown"] == 0
    assert manifest["n_files"] == 2655
    assert manifest["n_files"] == len(manifest["files"])
    assert manifest["live_batch_left_running"] is True
    assert "不是论文复现" in manifest["assumption"]


def test_d2c_doc_keeps_the_assumption_and_explains_skips():
    text = _D2C_DOC.read_text(encoding="utf-8")
    assert "这是假设，不是论文复现" in text or "是假设，不是论文复现" in text
    assert "skip=543" in text
    assert "不是出错" in text
    assert "1205" in text
    assert "未计划" in text


@pytest.mark.skipif(not (_WORK / "runs").is_dir(), reason="仓库外教师数据不在这台机器上")
def test_d2c_manifest_hashes_match_workspace_files():
    manifest = _load(_D2C)
    work = Path(manifest["work"])
    assert work == _WORK
    missing = []
    mismatched = []
    for row in manifest["files"]:
        path = work / row["path"]
        if not path.is_file():
            missing.append(row["path"])
            continue
        digest = sha256_file(path)
        size = path.stat().st_size
        if digest != row["sha256"] or size != row["bytes"]:
            mismatched.append(row["path"])
    assert missing == []
    assert mismatched == []
