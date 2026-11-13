"""Fast boundary tests for the public inter-drone helpers."""

from types import SimpleNamespace

import math
import numpy as np

from yopo_l0.inter_drone import (
    InterDroneConfig,
    as_sample_array,
    clearance_m,
    committed_neighbor_samples,
    inter_drone_config,
    inter_pick,
    sample_motion,
)


def test_empty_neighbors_do_not_penalize_or_replace_argmax():
    candidate = np.array([[0.0, 1.0, 2.0, 3.0]])
    assert math.isinf(clearance_m(candidate, []))
    assert committed_neighbor_samples(SimpleNamespace(peers={}), 0.0) == []

    pick = inter_pick(
        [1.0, 4.0, 2.0],
        [True, True, True],
        [math.inf, math.inf, math.inf],
        InterDroneConfig(1.0, 3.0),
    )
    assert pick.kind == "unchanged"
    assert pick.action is None and pick.order is None
    assert pick.filtered == 0 and pick.penalized == 0


def test_coincident_and_exact_threshold_follow_strict_boundary():
    cfg = InterDroneConfig(threshold_m=1.0, penalty=0.0)
    pick = inter_pick([10.0, 2.0], [True, True], [0.0, 1.0], cfg)
    assert pick.kind == "adjusted"
    assert pick.action == 1 and pick.order == (1,)
    assert pick.filtered == 1

    candidate = np.array([[0.0, 2.0, 3.0, 4.0]])
    neighbor = np.array([[0.0, 2.0, 3.0, 4.0]])
    assert clearance_m(candidate, [neighbor]) == 0.0
    exact = inter_pick([1.0], [True], [1.0], cfg)
    assert exact.kind == "unchanged"  # implementation uses clearance < threshold


def test_nan_samples_are_rejected_and_nan_clearance_is_ignored():
    row = [[0.0, np.nan, 0.0, 0.0]]
    assert as_sample_array(row) is None
    assert math.isinf(clearance_m(np.asarray(row), [np.array([[0.0, 0.0, 0.0, 0.0]])]))

    pick = inter_pick([1.0, 2.0], [True, True], [np.nan, 2.0], InterDroneConfig(1.0, 0.0))
    assert pick.kind == "unchanged"


def test_disabled_config_does_not_sample_or_adjust():
    assert inter_drone_config(None) is None
    assert inter_drone_config({"upstream": {"inter_drone_filter": False}}) is None

    class ExplodingMotion:
        t_start = 0.0
        t_end = 1.0

        def evaluate(self, _t, _order):
            raise AssertionError("disabled caller must not sample")

    node = SimpleNamespace(
        peers={"uav1": SimpleNamespace(plans=[])},
        untouched_motion=ExplodingMotion(),
    )
    assert committed_neighbor_samples(node, 0.0) == []
    scores = np.array([1.0, 5.0])
    assert int(np.argmax(scores)) == 1


def test_sample_motion_preserves_nan_returned_by_motion():
    motion = SimpleNamespace(t_start=0.0, t_end=0.0, evaluate=lambda _t, _order: [np.nan, 1.0, 2.0])
    sampled = sample_motion(motion, 0.0, horizon=0.02, dt=0.02)
    assert sampled.shape == (1, 4)
    assert np.isnan(sampled[0, 1])
