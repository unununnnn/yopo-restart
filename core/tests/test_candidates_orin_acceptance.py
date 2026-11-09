"""#6 Orin 验收补证：每条候选的绝对有效期、分轴连续性拒绝、走廊不充当环境放行。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from yopo_swarm_core.candidates import (
    UPSTREAM_DECODER,
    CandidateReject,
    Timing,
    build_candidates,
    head_continuity_ok,
    minco_two_piece,
    rank_candidates,
)
from yopo_swarm_core.plan import (
    CONTINUITY_TOL,
    KinematicLimits,
    PlanConfig,
    PlanReject,
    make_plan,
    quintic,
)
from yopo_swarm_core.trajectory import PiecewisePolynomial, TrajectoryExpired

GOLDEN = Path(__file__).parent / "golden" / "minco_d5e1e60_cpu.npz"


def golden_batch():
    g = np.load(GOLDEN)
    return g["traj_params"][0], g["score"][0], g["radius"][0]


def parent_branch(t0=0.0):
    seg = quintic(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.2, -0.1], [0.1, 0.0, 0.0]]),
        np.array([[6.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        4.0,
    )
    return PiecewisePolynomial(t0, [seg])


def test_every_candidate_validity_is_absolute_and_outside_is_expired():
    """有效期是绝对时刻。窗外的位置/速度/加速度求值抛 TrajectoryExpired，端点本身仍可求。"""
    params, score, radius = golden_batch()
    t_switch = 12.5
    parent = parent_branch(t0=10.0)
    timing = Timing(
        image_capture=12.00,
        state_sample=12.01,
        inference_done=12.05,
        proposal=12.06,
        effective=t_switch,
    )
    res = build_candidates(
        params,
        score,
        radius,
        UPSTREAM_DECODER,
        parent,
        timing,
        np.eye(3),
        parent.evaluate(12.01),
    )
    assert not isinstance(res, CandidateReject)
    assert len(res) == UPSTREAM_DECODER.count
    parent_pva = np.array([parent.evaluate(t_switch, k) for k in range(3)])
    for cand in res:
        assert cand.traj.t_start == pytest.approx(t_switch)
        assert cand.traj.t_end == pytest.approx(t_switch + float(np.sum(cand.durations)))
        assert cand.traj.t_start > 10.0
        for order in (0, 1, 2):
            np.testing.assert_allclose(
                cand.traj.evaluate(t_switch, order), parent_pva[order], atol=1e-9
            )
            at_end = cand.traj.evaluate(cand.traj.t_end, order)
            assert at_end.shape == (3,) and np.all(np.isfinite(at_end))
            for t in (t_switch - 1e-4, cand.traj.t_end + 1e-4):
                with pytest.raises(TrajectoryExpired, match="outside validity"):
                    cand.traj.evaluate(t, order)
        assert head_continuity_ok(cand.traj, parent, t_switch)


@pytest.mark.parametrize("row", [0, 1, 2], ids=["pos", "vel", "acc"])
def test_pva_component_discontinuity_is_rejected(row):
    """只错位置、只错速度或只错加速度，都过不了 I8。偏移 1e-5，大于 CONTINUITY_TOL。"""
    t_switch = 0.5
    parent = parent_branch()
    true = np.array([parent.evaluate(t_switch, k) for k in range(3)])
    offset = 1e-5
    assert offset > CONTINUITY_TOL
    bad = np.array(true, copy=True)
    bad[row, 0] = bad[row, 0] + offset
    traj = minco_two_piece(
        t_switch,
        bad,
        np.array([[8.0, 0.0, 1.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        np.array([4.0, 0.0, 0.5]),
        (1.0, 1.0),
    )
    got = np.array([traj.evaluate(t_switch, k) for k in range(3)])
    for k in range(3):
        if k == row:
            assert abs(float(got[k, 0] - true[k, 0])) > CONTINUITY_TOL
        else:
            np.testing.assert_allclose(got[k], true[k], atol=1e-9)
    assert not head_continuity_ok(traj, parent, t_switch)
    res = make_plan(
        "u0",
        1,
        "p",
        traj,
        t_switch,
        true,
        [0.6],
        PlanConfig(limits=KinematicLimits(v_max=50, a_max=200, j_max=5000)),
    )
    assert isinstance(res, list) and PlanReject.HEAD_DISCONTINUITY in res


def test_corridor_admission_is_strict_and_never_an_environment_pass():
    """走廊严格准入（D3，ADR-0006 2026-10-26 修订）：未准入者不出现在候选里，全部未准入时整帧拒绝。
    准入与否都不把 env_checked 设为真，也不改轨迹。"""
    params, score, radius = golden_batch()
    parent = parent_branch()
    p_ref = parent.evaluate(0.01)
    timing = Timing(
        image_capture=0.00, state_sample=0.01, inference_done=0.05, proposal=0.06, effective=0.5
    )

    def build(rad):
        return build_candidates(
            params, score, rad, UPSTREAM_DECODER, parent, timing, np.eye(3), p_ref
        )

    base = build(radius)
    assert isinstance(base, list)
    assert len(base) == UPSTREAM_DECODER.count
    assert all(c.corridor_ok for c in base)
    assert all(c.env_checked is False for c in base)

    nr = UPSTREAM_DECODER.radius_num
    best = int(np.argmax(score))
    mixed = np.array(radius, copy=True)
    mixed[best, :nr] = 0.0
    mixed_out = build(mixed)
    assert isinstance(mixed_out, list)
    assert {c.index for c in mixed_out} == set(range(UPSTREAM_DECODER.count)) - {best}
    assert all(c.corridor_ok for c in mixed_out)
    assert all(c.env_checked is False for c in mixed_out)
    mixed_rank = rank_candidates(score, mixed, UPSTREAM_DECODER)
    assert mixed_rank.preferred != best
    assert mixed_rank.preferred == mixed_out[0].index
    assert mixed_rank.warning is None
    assert not mixed_rank.corridor_empty

    blocked = np.zeros_like(radius)
    assert build(blocked) is CandidateReject.CORRIDOR_EMPTY
    blocked_rank = rank_candidates(score, blocked, UPSTREAM_DECODER)
    assert blocked_rank.corridor_empty
    assert blocked_rank.warning == "CORRIDOR_EMPTY"
    assert blocked_rank.order == ()

    by_base = {c.index: c for c in base}
    for other in (mixed_out,):
        for cand in other:
            src = by_base[cand.index]
            np.testing.assert_array_equal(cand.inner_w, src.inner_w)
            np.testing.assert_array_equal(cand.tail_pva_w, src.tail_pva_w)
            np.testing.assert_array_equal(cand.durations, src.durations)
            for frac in (0.0, 0.5, 1.0):
                t = src.traj.t_start + frac * (src.traj.t_end - src.traj.t_start)
                np.testing.assert_array_equal(cand.traj.evaluate(t), src.traj.evaluate(t))
