"""#6：网络输出 → 世界系 MINCO 候选（头状态在 t_switch 重构，I15/I8）+ 六时刻 + 有效期。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from yopo_swarm_core.candidates import (
    UPSTREAM_DECODER,
    CandidateReject,
    DecoderParams,
    Timing,
    build_candidates,
    decode_params,
    head_continuity_ok,
    minco_two_piece,
    rank_candidates,
)
from yopo_swarm_core.plan import KinematicLimits, PlanConfig, PlanReject, make_plan, quintic
from yopo_swarm_core.trajectory import PiecewisePolynomial, TrajectoryExpired

GOLDEN = Path(__file__).parent / "golden" / "minco_d5e1e60_cpu.npz"
DECODE_GOLDEN = Path(__file__).parent / "golden" / "minco_decode_d5e1e60.npz"


def golden_batch():
    g = np.load(GOLDEN)
    return g["traj_params"][0], g["score"][0], g["radius"][0]


def parent_branch(t0=0.0):
    """父计划：从原点以 1 m/s 沿 x 加速到停在 (6,0,0)。"""
    seg = quintic(
        np.array([[0, 0, 0], [1.0, 0.2, -0.1], [0.1, 0, 0]]),
        np.array([[6, 0, 0], [0, 0, 0], [0, 0, 0]]),
        4.0,
    )
    return PiecewisePolynomial(t0, [seg])


TIMING = Timing(
    image_capture=0.00, state_sample=0.01, inference_done=0.05, proposal=0.06, effective=0.5
)


# --- 解码与上游一致（离线金样） -------------------------------------------------


def test_decoder_params_match_pinned_upstream_config():
    g = np.load(DECODE_GOLDEN)
    cfg = json.loads(str(g["decoder_config"]))
    assert DecoderParams.from_config(cfg) == UPSTREAM_DECODER


def test_decode_matches_upstream_golden():
    g = np.load(DECODE_GOLDEN)
    params, _, _ = golden_batch()
    inner, tail, dur = decode_params(params, UPSTREAM_DECODER)
    np.testing.assert_allclose(inner, g["inner_pos_b"], rtol=1e-4, atol=1e-5)
    np.testing.assert_allclose(tail, g["tail_pva_b"], rtol=1e-4, atol=1e-5)
    np.testing.assert_allclose(dur, g["durations"], rtol=1e-4, atol=1e-5)


def test_minco_matches_upstream_minco_traj_numerically():
    g = np.load(DECODE_GOLDEN)
    for n in range(g["inner_w"].shape[0]):
        tr = minco_two_piece(
            0.0, g["head_pva_w"], g["tail_pva_w"][n], g["inner_w"][n], g["durations"][n]
        )
        for k, key in enumerate(("pos", "vel", "acc")):
            got = np.array([tr.evaluate(float(t), k) for t in g["times"][n]])
            np.testing.assert_allclose(got, g[key][n], rtol=1e-4, atol=1e-5)


def test_ranking_is_argmax_score_among_corridor_admitted():
    _params, score, radius = golden_batch()
    g = np.load(DECODE_GOLDEN)
    r = rank_candidates(score, radius, UPSTREAM_DECODER)
    assert r.preferred == int(g["upstream_action_id"])
    assert r.corridor_empty == bool(g["upstream_brake"])
    assert sorted(r.order) == list(np.flatnonzero(r.admitted))  # 严格准入：只排准入候选


def test_corridor_filter_excludes_best_score_like_upstream_select_action():
    _params, score, radius = golden_batch()
    nr = UPSTREAM_DECODER.radius_num
    best = int(np.argmax(score))
    r2 = radius.copy()
    r2[best, :nr] = 0.0  # 最高分候选走廊不达标
    r = rank_candidates(score, r2, UPSTREAM_DECODER)
    mu_lo = (r2[:, :nr] - r2[:, nr:]).min(axis=1)
    cand = np.flatnonzero(mu_lo >= UPSTREAM_DECODER.safe_mu)
    assert r.preferred == int(cand[np.argmax(score[cand])]) != best
    assert not r.corridor_empty
    assert best not in r.order  # 严格准入（D3）：未准入的最高分候选不可达
    assert r.order[1:] == tuple(sorted(r.order[1:], key=lambda i: (-score[i], i)))


def test_corridor_empty_gives_explicit_result():
    _params, score, radius = golden_batch()
    nr = UPSTREAM_DECODER.radius_num
    bad = radius.copy()
    bad[:, :nr] = 0.0  # μ = 0 → 没有候选通过 0.05 m 走廊
    r = rank_candidates(score, bad, UPSTREAM_DECODER)
    assert r.corridor_empty
    assert r.order == () and r.preferred is None  # 没有可飞候选；上游此时 brake=True
    assert r.warning == "CORRIDOR_EMPTY"


# --- 头状态重构（I15 / I8） -----------------------------------------------------


def test_reconstructed_head_matches_parent_pva_at_t_switch():
    params, score, radius = golden_batch()
    parent = parent_branch()
    res = build_candidates(
        params, score, radius, UPSTREAM_DECODER, parent, TIMING, np.eye(3), parent.evaluate(0.01)
    )
    assert not isinstance(res, CandidateReject)
    for c in res:
        assert c.traj.t_start == pytest.approx(0.5)
        for k in range(3):
            np.testing.assert_allclose(c.traj.evaluate(0.5, k), parent.evaluate(0.5, k), atol=1e-9)
        assert head_continuity_ok(c.traj, parent, 0.5)


def test_timestamp_shift_without_reconstruction_is_rejected():
    """只把 t_state 时刻的候选平移到 t_switch，而不重构头状态：连续性检查和计划契约都拒绝。"""
    params, _, _ = golden_batch()
    parent = parent_branch()
    inner, tail, dur = decode_params(params, UPSTREAM_DECODER)
    p0 = parent.evaluate(0.01)
    tail_w = tail[0].copy()
    tail_w[0] += p0
    head_old = np.array([parent.evaluate(0.01, k) for k in range(3)])
    naive = minco_two_piece(0.01, head_old, tail_w, inner[0] + p0, dur[0]).shifted(0.49)
    assert not head_continuity_ok(naive, parent, 0.5)
    lim = KinematicLimits(v_max=50, a_max=200, j_max=5000)
    res = make_plan(
        "u0",
        1,
        "p",
        naive,
        0.5,
        np.array([parent.evaluate(0.5, k) for k in range(3)]),
        [0.6],
        PlanConfig(limits=lim),
    )
    assert isinstance(res, list) and PlanReject.HEAD_DISCONTINUITY in res


def test_build_requires_parent_branch():
    params, score, radius = golden_batch()
    with pytest.raises(TypeError):
        build_candidates(
            params, score, radius, UPSTREAM_DECODER, None, TIMING, np.eye(3), np.zeros(3)
        )


def test_parent_must_cover_t_switch():
    params, score, radius = golden_batch()
    parent = parent_branch().with_validity(0.0, 0.3)
    res = build_candidates(
        params, score, radius, UPSTREAM_DECODER, parent, TIMING, np.eye(3), np.zeros(3)
    )
    assert res == CandidateReject.PARENT_EXPIRED


# --- 六时刻与有效期 -------------------------------------------------------------


def test_state_image_skew_beyond_bound_is_rejected():
    t = Timing(
        image_capture=0.0, state_sample=0.08, inference_done=0.1, proposal=0.11, effective=0.5
    )
    assert t.validate(max_skew=0.03) == CandidateReject.STATE_IMAGE_SKEW
    assert TIMING.validate(max_skew=0.03) is None
    params, score, radius = golden_batch()
    parent = parent_branch()
    res = build_candidates(
        params, score, radius, UPSTREAM_DECODER, parent, t, np.eye(3), np.zeros(3), max_skew=0.03
    )
    assert res == CandidateReject.STATE_IMAGE_SKEW


@pytest.mark.parametrize(
    ("kw", "reason"),
    [
        ({"proposal": 0.04}, CandidateReject.TIME_ORDER),  # 提案早于推理完成
        ({"effective": 0.06}, CandidateReject.TIME_ORDER),  # 生效不晚于提案
        ({"inference_done": float("nan")}, CandidateReject.MALFORMED),
    ],
)
def test_timing_order_and_finiteness(kw, reason):
    base = {
        "image_capture": 0.0,
        "state_sample": 0.01,
        "inference_done": 0.05,
        "proposal": 0.06,
        "effective": 0.5,
    }
    base.update(kw)
    assert Timing(**base).validate(max_skew=0.03) == reason


def test_actual_switch_is_recorded_separately():
    t = TIMING.with_actual_switch(0.52)
    assert t.actual_switch == 0.52 and t.effective == 0.5
    assert t.validate(max_skew=0.03) is None


def test_candidate_has_absolute_validity_and_expires():
    params, score, radius = golden_batch()
    parent = parent_branch()
    res = build_candidates(
        params, score, radius, UPSTREAM_DECODER, parent, TIMING, np.eye(3), parent.evaluate(0.01)
    )
    c = res[0]
    assert c.traj.t_start == 0.5 and c.traj.t_end == pytest.approx(0.5 + float(np.sum(c.durations)))
    with pytest.raises(TrajectoryExpired):
        c.traj.evaluate(c.traj.t_end + 1e-3)
    with pytest.raises(TrajectoryExpired):
        c.traj.evaluate(0.49)


def test_corridor_is_only_metadata_not_a_safety_pass():
    params, score, radius = golden_batch()
    parent = parent_branch()
    res = build_candidates(
        params, score, radius, UPSTREAM_DECODER, parent, TIMING, np.eye(3), parent.evaluate(0.01)
    )
    for c in res:
        assert c.env_checked is False  # 必须另走 #7 环境检查与动力学检查
        assert isinstance(c.corridor_ok, bool)


def test_non_finite_network_output_is_rejected():
    params, score, radius = golden_batch()
    p = params.copy()
    p[3, 5] = np.nan
    parent = parent_branch()
    res = build_candidates(
        p, score, radius, UPSTREAM_DECODER, parent, TIMING, np.eye(3), np.zeros(3)
    )
    assert res == CandidateReject.MALFORMED


def test_world_rotation_applied_to_offsets():
    params, score, radius = golden_batch()
    _, tail, _ = decode_params(params, UPSTREAM_DECODER)
    rz = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # 机体朝 +y
    parent = parent_branch()
    p_ref = parent.evaluate(0.01)
    res = build_candidates(params, score, radius, UPSTREAM_DECODER, parent, TIMING, rz, p_ref)
    c = res[0]
    np.testing.assert_allclose(c.tail_pva_w[0], rz @ tail[c.index, 0] + p_ref, atol=1e-12)
    np.testing.assert_allclose(
        c.traj.evaluate(c.traj.t_end), rz @ tail[c.index, 0] + p_ref, atol=1e-6
    )
