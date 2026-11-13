"""Fast boundary tests for geometric-fallback public APIs."""

import numpy as np

from yopo_l0.geom_fallback import GeomSession, REASON_INTER, interp_position


def _card(enabled: bool) -> dict:
    return {
        "criteria": {"body_radius_m": 0.35},
        "dynamics": {"a_max": 6.0},
        "geometric_fallback": {"enabled": enabled, "inter_drone_distance_m": 0.7},
    }


def _session(enabled: bool = True, obstacles=()) -> GeomSession:
    return GeomSession.from_card(
        _card(enabled), members=("uav0", "uav1"), horizon=0.02, obstacles=obstacles
    )


def test_empty_neighbors_do_not_report_inter_drone_conflict():
    session = _session()
    assert session.classify("uav0", [[0.0, 0.0, 0.0, 1.0]]) == []
    assert session.checks == 0 and session.fallbacks == 0
    assert interp_position([], 0.0) is None


def test_coincident_position_conflicts_but_exact_threshold_does_not():
    session = _session()
    session._heard["uav1"] = [[0.0, 0.0, 0.0, 1.0]]
    assert session.classify("uav0", [[0.0, 0.0, 0.0, 1.0]]) == [REASON_INTER]
    assert session.classify("uav0", [[0.0, 0.7, 0.0, 1.0]]) == []


def test_nan_positions_and_neighbor_samples_do_not_report_conflict():
    session = _session()
    session._heard["uav1"] = [[0.0, np.nan, 0.0, 1.0]]
    assert session.classify("uav0", [[0.0, 0.0, 0.0, 1.0]]) == []
    session._heard["uav1"] = [[0.0, 0.0, 0.0, 1.0]]
    assert session.classify("uav0", [[0.0, np.nan, 0.0, 1.0]]) == []


def test_disabled_session_returns_original_without_sampling_or_state_changes():
    session = _session(enabled=False)

    class Plan:
        @property
        def pva(self):
            raise AssertionError("disabled fallback must not inspect or sample the trajectory")

    proposed = Plan()
    before = dict(session.__dict__)
    assert session.review_goto("uav0", proposed, 0.0) is proposed
    assert session.checks == 0 and session.fallbacks == 0
    assert session._outbox == [] and session._plans == {} and session._meta == {}
    assert session.__dict__ == before


def test_disabled_commit_review_accepts_without_reading_node():
    session = _session(enabled=False)

    class Node:
        @property
        def in_flight(self):
            raise AssertionError("disabled fallback must not read the proposed trajectory")

    assert session.review_commit(Node(), 0.0) == "accept"
    assert session.checks == 0 and session._pending == {}
