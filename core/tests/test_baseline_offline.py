"""#3 不依赖网络和 torch 的部分：权重校验、config_hash、候选数推导、无 ROS 导入。"""

import hashlib
import importlib
import sys

import pytest

from yopo_swarm_core import baseline
from yopo_swarm_core.baseline import (
    WeightsVerificationError,
    candidate_count,
    config_hash,
    verify_weights,
)


def test_pins_match_adr():
    assert baseline.UPSTREAM_SHA == "d5e1e60f83cc991602f142d9b6b96204a24d6b9f"
    assert baseline.WEIGHTS_SHA256 == (
        "2d85e712c8874d11cd4e523777698eff78914fdb8f1ffb5068b037f8f972f62f"
    )


def test_missing_weights_rejected_with_clear_error(tmp_path):
    with pytest.raises(WeightsVerificationError, match="not found"):
        verify_weights(tmp_path / "nope.pth")


def test_wrong_hash_rejected_with_clear_error(tmp_path):
    p = tmp_path / "epoch50.pth"
    p.write_bytes(b"not the real weights")
    with pytest.raises(WeightsVerificationError, match="sha256 mismatch") as e:
        verify_weights(p)
    assert hashlib.sha256(b"not the real weights").hexdigest() in str(e.value)


def test_matching_hash_accepted(tmp_path):
    p = tmp_path / "w.pth"
    p.write_bytes(b"abc")
    assert verify_weights(p, expected=hashlib.sha256(b"abc").hexdigest()) == (
        hashlib.sha256(b"abc").hexdigest()
    )


def test_load_refuses_bad_weights_before_touching_torch(tmp_path):
    p = tmp_path / "epoch50.pth"
    p.write_bytes(b"tampered")
    with pytest.raises(WeightsVerificationError):
        baseline.load_verified_model(upstream_root=tmp_path, weights_path=p)


CFG = {"horizon_num": 5, "vertical_num": 3, "radius_num": 10, "v_mean_unit": [0.4, 0.0, 0.0]}


def test_config_hash_stable_and_order_independent():
    shuffled = dict(reversed(list(CFG.items())))
    assert config_hash(CFG) == config_hash(dict(CFG)) == config_hash(shuffled)
    assert len(config_hash(CFG)) == 64


@pytest.mark.parametrize(
    "key,value",
    [("horizon_num", 6), ("vertical_num", 4), ("radius_num", 11), ("v_mean_unit", [0.4, 0.0, 0.1])],
)
def test_config_hash_changes_when_any_field_changes(key, value):
    assert config_hash({**CFG, key: value}) != config_hash(CFG)


def test_config_hash_distinguishes_added_field_and_types():
    assert config_hash({**CFG, "extra": 1}) != config_hash(CFG)
    assert config_hash({**CFG, "radius_num": 10.0}) != config_hash(CFG)


def test_candidate_count_derived_from_grid():
    assert candidate_count({"horizon_num": 5, "vertical_num": 3}) == 15
    assert candidate_count({"horizon_num": 4, "vertical_num": 2}) == 8


def test_baseline_module_imports_no_ros():
    importlib.import_module("yopo_swarm_core.baseline")
    ros = [m for m in sys.modules if m.split(".")[0] in {"rclpy", "rospy", "rosgraph", "roslib"}]
    assert ros == []
