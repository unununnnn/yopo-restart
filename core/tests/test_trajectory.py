import math

import numpy as np
import pytest

from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment, TrajectoryExpired


def line(p0, v, t0, duration):
    """Straight line p(t) = p0 + v (t - t0) on [t0, t0+duration]."""
    coeffs = np.array([[p0[k], v[k]] for k in range(3)], dtype=float)
    return PiecewisePolynomial(t0, [PolySegment(coeffs, duration)])


def test_evaluates_position_and_derivatives_in_absolute_time():
    # x(τ) = 1 + 2τ + 3τ², segment starts at absolute t0 = 10
    coeffs = np.array([[1.0, 2.0, 3.0], [0, 0, 0], [0, 0, 0]])
    traj = PiecewisePolynomial(10.0, [PolySegment(coeffs, 2.0)])
    assert traj.evaluate(11.0)[0] == pytest.approx(6.0)
    assert traj.evaluate(11.0, order=1)[0] == pytest.approx(8.0)
    assert traj.evaluate(11.0, order=2)[0] == pytest.approx(6.0)
    assert traj.t_start == 10.0 and traj.t_end == 12.0


def test_second_segment_uses_its_own_local_time():
    s1 = PolySegment(np.array([[0.0, 1.0], [0, 0], [0, 0]]), 1.0)
    s2 = PolySegment(np.array([[1.0, -1.0], [0, 0], [0, 0]]), 1.0)
    traj = PiecewisePolynomial(0.0, [s1, s2])
    assert traj.evaluate(1.5)[0] == pytest.approx(0.5)
    assert traj.boundaries() == pytest.approx([0.0, 1.0, 2.0])


@pytest.mark.parametrize("t", [-0.001, 2.001, 100.0])
def test_evaluation_outside_validity_raises_expired_instead_of_clamping(t):
    traj = line((0, 0, 0), (1, 0, 0), 0.0, 2.0)
    with pytest.raises(TrajectoryExpired):
        traj.evaluate(t)


@pytest.mark.parametrize("order", [0, 1, 2, 3])
def test_evaluation_outside_validity_raises_for_derivatives(order):
    """有效期外是过期，包括仍落在多项式支撑内的时刻；不返回端点值。"""
    traj = line((0, 0, 0), (1, 2, 3), 0.0, 2.0)
    start = traj.evaluate(0.0, order)
    end = traj.evaluate(2.0, order)
    assert not np.allclose(start, end) or order > 0
    for t in (-1e-6, 2.0 + 1e-6, 100.0, math.nan):
        with pytest.raises(TrajectoryExpired, match="outside validity"):
            traj.evaluate(t, order)
    short = traj.with_validity(0.4, 1.2)
    held = short.evaluate(1.2, order)
    with pytest.raises(TrajectoryExpired, match="outside validity"):
        short.evaluate(1.6, order)  # 1.6 仍在支撑 [0, 2] 内，夹断则会返回端点
    assert np.all(np.isfinite(held))
    assert short.evaluate(1.2, order) is not None


def test_validity_window_can_be_shorter_than_polynomial_support():
    traj = line((0, 0, 0), (1, 0, 0), 0.0, 2.0).with_validity(0.5, 1.5)
    assert traj.evaluate(1.0)[0] == pytest.approx(1.0)
    with pytest.raises(TrajectoryExpired):
        traj.evaluate(1.6)


@pytest.mark.parametrize(
    "seg",
    [
        PolySegment(np.array([[math.nan, 1.0], [0, 0], [0, 0]]), 1.0),
        PolySegment(np.array([[math.inf, 1.0], [0, 0], [0, 0]]), 1.0),
        PolySegment(np.zeros((3, 2)), 0.0),
        PolySegment(np.zeros((3, 2)), -1.0),
        PolySegment(np.zeros((3, 2)), math.nan),
    ],
)
def test_malformed_segments_are_constructible_but_flagged(seg):
    traj = PiecewisePolynomial(0.0, [seg])
    assert not traj.is_well_formed()


def test_shifted_moves_trajectory_in_time():
    traj = line((0, 0, 0), (1, 0, 0), 0.0, 2.0).shifted(5.0)
    assert traj.t_start == 5.0
    assert traj.evaluate(6.0)[0] == pytest.approx(1.0)
