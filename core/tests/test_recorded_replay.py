import gzip
import json
from pathlib import Path

from yopo_swarm_core.replay import config_from_log, replay


def test_recorded_humble_snapshot_replays_bitwise_with_self_describing_config():
    path = Path(__file__).resolve().parents[2] / "docs/ref/ros2-node/px4_1.jsonl.gz"
    cfg = config_from_log(path)
    with gzip.open(path, "rt") as f:
        rows = [json.loads(line) for line in f]
    assert cfg.config_hash() == rows[0]["config_hash"]
    assert replay(path, cfg) == [row["decision"] for row in rows[1:]]
