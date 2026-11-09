import numpy as np

from yopo_swarm_core.oracle import FlightOracle, Outcome
from yopo_swarm_core.synthetic import Box


def make(**kwargs):
    return FlightOracle(
        (Box((4.5, -0.5, 0.5), (5.5, 0.5, 1.5)),),
        np.array([0.0, 0.0, 1.0]),
        np.array([10.0, 0.0, 1.0]),
        0.0,
        **kwargs,
    )


def test_collision_uses_actual_px4_position_not_nominal_reference():
    oracle = make(tracking_bound=100.0)
    assert oracle.update(0.1, np.array([0.0, 0.0, 1.0]), 0.1, np.array([5.0, 0.0, 1.0])) is None
    assert (
        oracle.update(0.2, np.array([5.0, 0.0, 1.0]), 0.2, np.array([0.0, 0.0, 1.0]))
        is Outcome.COLLISION
    )


def test_tracking_violation_is_oracle_failure_not_success():
    oracle = make()
    assert (
        oracle.update(0.1, np.array([0.0, 0.0, 1.0]), 0.1, np.array([0.3, 0.0, 1.0]))
        is Outcome.ORACLE_PAUSE
    )
    assert oracle.report()["reason"] == "TRACKING_BOUND"


def test_stale_odometry_and_lag_are_distinct_failures():
    assert (
        make().update(1.0, np.array([0.0, 0.0, 1.0]), 0.0, np.array([0.0, 0.0, 1.0]))
        is Outcome.ORACLE_PAUSE
    )
    assert (
        make().update(11.0, np.array([0.0, 0.0, 1.0]), 11.0, np.array([0.0, 0.0, 1.0]))
        is Outcome.LAG_VIOLATION
    )


def test_goal_needs_actual_armed_hold_and_terminal_speed():
    oracle = make(tracking_bound=100.0)
    goal = oracle.goal
    assert oracle.update(1.0, goal, 1.0, goal, armed=False) is None
    assert oracle.update(2.0, goal, 2.0, goal, armed=True, speed=1.0) is None
    assert oracle.update(3.0, goal, 3.0, goal, armed=True) is None
    assert oracle.update(4.0, goal, 4.0, goal, armed=True) is Outcome.SUCCESS


def test_ground_support_is_bounded_and_cannot_be_reentered_after_liftoff():
    p = np.array([0.0, 0.0, 0.24])

    def grounded():
        return make(ground_z=0.0, support_center=p, support_min_z=0.23)

    oracle = grounded()
    assert oracle.update(0.1, p, 0.1, p, mission_ready=False) is None
    outside = p + np.array([0.16, 0.0, 0.0])
    assert grounded().update(0.1, outside, 0.1, outside, mission_ready=False) is Outcome.COLLISION
    below = np.array([0.0, 0.0, 0.22])
    assert grounded().update(0.1, below, 0.1, below, mission_ready=False) is Outcome.COLLISION
    high = np.array([0.0, 0.0, 0.6])
    assert oracle.update(0.2, high, 0.2, high, mission_ready=False) is None
    assert oracle.update(0.3, p, 0.3, p, mission_ready=False) is Outcome.COLLISION
    assert grounded().update(0.1, p, 0.1, p, mission_ready=True) is Outcome.COLLISION


def test_p0_reference_starts_only_after_takeoff_contract():
    oracle = make(ground_z=0.0)
    p = oracle.start
    assert oracle.update(10.0, p, 10.0, p, mission_ready=False) is None
    assert oracle.update(20.0, p, 20.0, p, mission_ready=True) is None
    assert oracle.mission_start_time == 20.0
    assert oracle.update(31.0, p, 31.0, p) is Outcome.LAG_VIOLATION
