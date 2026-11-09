"""票 09：L1 日志收成和 L0 同一套 result.json。规划拒绝码来自 vendored 的 yopo-swarm 日志。"""

import json
import shutil
from pathlib import Path

from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card

from yopo_l1.export import export_l1
from yopo_l1.importer import import_l1

ROOT = Path(__file__).resolve().parents[1]
REAL_LOG = ROOT / "docs/ref/ros2-node/px4_1.jsonl.gz"
SYNTHETIC = Path(__file__).parent / "fixtures/l1/flight-synthetic.json"


def _bundle(tmp_path, shadow=True):
    card = load_card(ROOT / "scenes/g2-pillar.json")
    export_l1(card, RunConfig(seed=3, policy="upstream", shadow=shadow), tmp_path / "bundle")
    log = tmp_path / "log"
    log.mkdir()
    shutil.copy(tmp_path / "bundle" / "run.json", log / "run.json")
    shutil.copy(tmp_path / "bundle" / "scene.json", log / "scene.json")
    shutil.copy(REAL_LOG, log / "px4_1.jsonl.gz")
    return card, log


def test_real_planner_log_fills_schema_and_reject_histogram(tmp_path):
    card, log = _bundle(tmp_path, shadow=True)
    result = import_l1(log)
    saved = json.loads((log / "result.json").read_text())
    assert saved == result
    assert result["tier"] == "L1"
    assert result["scene"] == "g2-pillar"
    assert result["scene_card_hash"] == card.hash
    assert result["seed"] == 3 and result["policy"] == "upstream" and result["shadow"] is True
    assert result["dynamics"] == "px4_sitl"
    assert result["outcome"] == "TIMEOUT"
    assert result["arrival_time_s"] is None
    assert result["min_obstacle_distance_m"] < 0.01
    assert result["shadow_reject_rate"] == 1.0
    assert result["shadow_reject_histogram"]["NO_ENV_SAFE_CANDIDATE"] == 17
    assert result["shadow_reject_histogram"]["STALE_FRAME"] == 1
    assert result["shadow_drones"][0]["id"] == "uav0"
    assert result["plan_codes"]["NO_ENV_SAFE_CANDIDATE"] == 17
    assert result["takeoff_handoff_s"] is None
    assert result["rtf"] is None
    assert result["network"]["upstream_sha"].startswith("d5e1e60")
    l0 = run(load_card(ROOT / "scenes/g2-hover.json"), RunConfig(seed=0, policy="hover", shadow=False, duration_s=0.2))
    assert set(result) == set(l0.result) | {"takeoff_handoff_s", "rtf"}


def test_shadow_off_keeps_reject_rate_null(tmp_path):
    _card, log = _bundle(tmp_path, shadow=False)
    result = import_l1(log)
    assert result["shadow"] is False
    assert result["shadow_reject_rate"] is None
    assert result["shadow_reject_histogram"] is None
    assert result["plan_codes"]["STALE_FRAME"] == 1


def test_synthetic_flight_overlay_keeps_real_reject_codes(tmp_path):
    raw = json.loads(SYNTHETIC.read_text())
    assert raw["label"] == "synthetic"
    assert raw["built_from"].endswith("px4_1.jsonl.gz")
    _card, log = _bundle(tmp_path, shadow=True)
    shutil.copy(SYNTHETIC, log / "l1-flight.json")
    result = import_l1(log)
    assert result["outcome"] == "SUCCESS"
    assert result["arrival_time_s"] == 12.4
    assert result["takeoff_handoff_s"] == 6.5
    assert result["rtf"] == 0.45
    assert result["min_obstacle_distance_m"] == 0.62
    assert result["shadow_reject_histogram"]["NO_ENV_SAFE_CANDIDATE"] == 17
    assert result["tier"] == "L1"
    assert "label" not in result
