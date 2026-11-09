"""票 09：同一张卡的 L0 / L1 对照，以及批跑表上 L0 的两列留空。"""

import json
from pathlib import Path

import pytest

from yopo_l0.batch import main as batch_main
from yopo_l0.scene import load_card

from yopo_l1.errors import L1ReportError
from yopo_l1.report import l0_vs_l1

ROOT = Path(__file__).resolve().parents[1]


def _write(directory: Path, **fields):
    directory.mkdir(parents=True)
    base = {
        "scene": "g2-pillar",
        "scene_card_hash": "abc",
        "seed": 0,
        "tier": "L0",
        "policy": "upstream",
        "shadow": True,
        "task": "goto",
        "n_drones": 1,
        "arrived": 1,
        "arrival_time_s": 8.2,
        "outcome": "SUCCESS",
        "shadow_reject_rate": 0.25,
        "takeoff_handoff_s": None,
        "rtf": None,
    }
    base.update(fields)
    (directory / "result.json").write_text(json.dumps(base), encoding="utf-8")


def test_report_lists_the_five_metrics(tmp_path):
    card = load_card(ROOT / "scenes/g2-pillar.json")
    _write(tmp_path / "l0", scene_card_hash=card.hash, arrival_time_s=8.2, shadow_reject_rate=0.25)
    _write(
        tmp_path / "l1",
        scene_card_hash=card.hash,
        tier="L1",
        arrival_time_s=12.4,
        shadow_reject_rate=1.0,
        takeoff_handoff_s=6.5,
        rtf=0.45,
        outcome="SUCCESS",
        arrived=1,
    )
    text = l0_vs_l1(tmp_path / "l0", tmp_path / "l1", tmp_path / "diff.md")
    assert (tmp_path / "diff.md").read_text() == text
    assert "| 到达时间 s | 8.20 | 12.40 | 4.20 |" in text
    assert "| 碰撞 | 0 | 0 | 0 |" in text
    assert "| 影子拒绝率 | 0.250 | 1.000 | 0.750 |" in text
    assert "| 起飞交接 s | — | 6.500 | — |" in text
    assert "| RTF | — | 0.450 | — |" in text
    assert "交给 frank" in text
    assert "方向 B" in text


def test_report_rejects_different_cards(tmp_path):
    _write(tmp_path / "l0", scene_card_hash="aaa")
    _write(tmp_path / "l1", scene_card_hash="bbb", tier="L1")
    with pytest.raises(L1ReportError, match="不是同一张场景卡"):
        l0_vs_l1(tmp_path / "l0", tmp_path / "l1")


def test_batch_table_leaves_l1_columns_empty_for_l0(tmp_path):
    rc = batch_main([
        "scenes/narrow-corridor.json@hover",
        "--seeds", "0",
        "--duration", "0.2",
        "--no-determinism",
        "--no-shadow",
        "--out", str(tmp_path),
    ])
    assert rc == 0
    md = (tmp_path / "summary.md").read_text()
    assert "起飞交接 s" in md and "RTF" in md
    row = next(line for line in md.splitlines() if line.startswith("| 0 |"))
    cells = [c.strip() for c in row.strip("|").split("|")]
    assert cells[-2:] == ["—", "—"]
