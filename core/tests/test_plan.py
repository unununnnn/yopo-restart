"""#8：计划安全契约（ADR-0002）——运动前缀 + 固定分支点制动分支 + 终端占用 + 占用集合。"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest

from yopo_swarm_core.collision import Verdict, check_derivative_bound
from yopo_swarm_core.plan import (
    HOLD_TOL,
    BrakeBranch,
    Hold,
    KinematicLimits,
    OccupancySet,
    Plan,
    PlanConfig,
    PlanReject,
    brake_branch,
    canonical_digest,
    conflict_witness,
    conflicts,
    make_plan,
    quintic,
    validate_plan,
)
from yopo_swarm_core.trajectory import (
    PiecewisePolynomial,
    PolySegment,
    TrajectoryExpired,
)

LIM = KinematicLimits(v_max=4.0, a_max=6.0, j_max=40.0)
CFG = PlanConfig(limits=LIM, max_branch_points=4, body_radius=0.2, tracking_margin=0.1)


def pva(p, v=(0, 0, 0), a=(0, 0, 0)):
    return np.array([p, v, a], dtype=float)


def straight(t0, p0, p1, T, v0=(0, 0, 0)):
    """从 (p0, v0, 0) 到 (p1, 0, 0) 的单段五次运动前缀。"""
    return PiecewisePolynomial(t0, [quintic(pva(p0, v0), pva(p1), T)])


def plan_from(motion, t_switch, head, times, owner="u0", cfg=CFG):
    res = make_plan(owner, 1, "parent", motion, t_switch, head, times, cfg)
    assert isinstance(res, Plan), res
    return res


# --- 构造与校验 ---------------------------------------------------------------


def test_quintic_matches_boundary_conditions():
    seg = quintic(pva((0, 0, 0), (1, 0, 0), (0, 1, 0)), pva((3, 2, 1), (0, 0, 0)), 2.0)
    for k, want in enumerate([(0, 0, 0), (1, 0, 0), (0, 1, 0)]):
        np.testing.assert_allclose(seg.evaluate_local(0.0, k), want, atol=1e-12)
    for k, want in enumerate([(3, 2, 1), (0, 0, 0), (0, 0, 0)]):
        np.testing.assert_allclose(seg.evaluate_local(2.0, k), want, atol=1e-9)


def test_valid_plan_has_feasible_continuous_brake_branches():
    m = straight(0.0, (0, 0, 0), (6, 0, 0), 3.0)
    p = plan_from(m, 0.0, pva((0, 0, 0)), [0.5, 1.0, 1.5])
    assert validate_plan(p, CFG) == []
    assert len(p.branches) == 3
    for br in p.branches:
        for k in range(3):  # I8：分支点处 PVA 连续
            np.testing.assert_allclose(
                br.traj.evaluate(br.t_branch, k), m.evaluate(br.t_branch, k), atol=1e-9
            )
        np.testing.assert_allclose(br.traj.evaluate(br.traj.t_end, 1), 0, atol=1e-9)
        np.testing.assert_allclose(br.traj.evaluate(br.traj.t_end, 2), 0, atol=1e-9)


def test_plan_without_brake_branch_is_rejected():
    m = straight(0.0, (0, 0, 0), (6, 0, 0), 3.0)
    res = make_plan("u0", 1, "p", m, 0.0, pva((0, 0, 0)), [], CFG)
    assert res == [PlanReject.NO_BRAKE_BRANCH]


def test_branch_point_count_is_capped():
    cfg = PlanConfig(limits=LIM, max_branch_points=2, body_radius=0.2, tracking_margin=0.1)
    m = straight(0.0, (0, 0, 0), (6, 0, 0), 3.0)
    res = make_plan("u0", 1, "p", m, 0.0, pva((0, 0, 0)), [0.5, 1.0, 1.5], cfg)
    assert PlanReject.TOO_MANY_BRANCH_POINTS in res


def test_head_discontinuity_is_rejected():
    """I8：新计划在 t_switch 的 PVA 必须等于父计划给出的头状态。"""
    m = straight(1.0, (0, 0, 0), (6, 0, 0), 3.0)
    res = make_plan("u0", 1, "p", m, 1.0, pva((0, 0, 0), (0.5, 0, 0)), [1.5], CFG)
    assert PlanReject.HEAD_DISCONTINUITY in res


def test_hover_is_not_a_valid_brake_branch_while_moving():
    m = straight(0.0, (0, 0, 0), (6, 0, 0), 3.0)
    p = plan_from(m, 0.0, pva((0, 0, 0)), [1.0])
    assert validate_plan(p, CFG) == []
    speed = float(np.linalg.norm(m.evaluate(1.0, 1)))
    assert speed > HOLD_TOL
    pos = m.evaluate(1.0)
    hover = PiecewisePolynomial(1.0, [quintic(pva(pos), pva(pos), 1.0)])
    assert float(np.linalg.norm(hover.evaluate(1.0, 1))) <= HOLD_TOL
    bad = p.replace_branches([BrakeBranch(1.0, hover)])
    assert validate_plan(bad, CFG) == [PlanReject.HOVER_AS_BRAKE]


def test_branch_discontinuity_is_rejected():
    m = straight(0.0, (0, 0, 0), (6, 0, 0), 3.0)
    p = plan_from(m, 0.0, pva((0, 0, 0)), [1.0])
    pos, vel = m.evaluate(1.0), m.evaluate(1.0, 1)
    jump = PiecewisePolynomial(1.0, [quintic(pva(pos + 0.3, vel), pva(pos + 2.0), 2.0)])
    bad = p.replace_branches([type(p.branches[0])(1.0, jump)])
    assert PlanReject.BRANCH_DISCONTINUITY in validate_plan(bad, CFG)


def test_infeasible_brake_branch_is_rejected_by_conservative_check():
    m = straight(0.0, (0, 0, 0), (6, 0, 0), 3.0)
    p = plan_from(m, 0.0, pva((0, 0, 0)), [1.0])
    pos, vel, acc = m.evaluate(1.0), m.evaluate(1.0, 1), m.evaluate(1.0, 2)
    slam = PiecewisePolynomial(1.0, [quintic(pva(pos, vel, acc), pva(pos + 0.1 * vel), 0.15)])
    bad = p.replace_branches([type(p.branches[0])(1.0, slam)])
    assert PlanReject.BRANCH_KINEMATICS in validate_plan(bad, CFG)


def test_brake_branch_builder_respects_limits():
    v = np.array([3.5, 0.0, 0.0])
    br = brake_branch(2.0, pva((1, 1, 1), v, (0.5, 0, 0)), LIM)
    assert br is not None
    np.testing.assert_allclose(br.evaluate(br.t_end, 1), 0, atol=1e-9)


def test_motion_must_end_holdable():
    seg = quintic(pva((0, 0, 0)), pva((6, 0, 0), (1.0, 0, 0)), 3.0)
    m = PiecewisePolynomial(0.0, [seg])
    res = make_plan("u0", 1, "p", m, 0.0, pva((0, 0, 0)), [1.0], CFG)
    assert PlanReject.TERMINAL_NOT_HOLDABLE in res


def test_kinematic_violation_in_motion_is_rejected():
    m = straight(0.0, (0, 0, 0), (20, 0, 0), 2.0)  # 峰值速度 ≈ 18.75 > 4
    res = make_plan("u0", 1, "p", m, 0.0, pva((0, 0, 0)), [0.5], CFG)
    assert PlanReject.MOTION_KINEMATICS in res


def test_non_finite_input_is_malformed():
    m = straight(0.0, (0, 0, 0), (6, 0, 0), 3.0)
    res = make_plan("u0", 1, "p", m, 0.0, pva((np.nan, 0, 0)), [1.0], CFG)
    assert res == [PlanReject.MALFORMED]


# --- 占用集合与冲突 -----------------------------------------------------------


def test_terminal_occupancy_persists_after_polynomial_expires():
    """多项式过期后不能删除仍在原地的无人机。"""
    a = plan_from(straight(0.0, (0, 0, 0), (4, 0, 0), 2.0), 0.0, pva((0, 0, 0)), [1.0])
    later = straight(50.0, (4, -3, 0), (4, 3, 0), 3.0)  # t=50 穿过 A 的终点
    b = plan_from(later, 50.0, pva((4, -3, 0)), [50.5], owner="u1")
    assert conflicts(a.occupancy(CFG), b.occupancy(CFG))
    far = plan_from(
        straight(50.0, (9, -3, 0), (9, 3, 0), 3.0), 50.0, pva((9, -3, 0)), [50.5], owner="u1"
    )
    assert not conflicts(a.occupancy(CFG), far.occupancy(CFG))


def test_brake_branch_intersection_is_a_conflict_even_if_nominal_paths_are_clear():
    """I14：名义轨迹时间上错开，但制动分支 + 终端保持与另一机相交。"""
    a = plan_from(straight(0.0, (0, 0, 0), (8, 0, 0), 4.0), 0.0, pva((0, 0, 0)), [1.8])
    stop = a.branches[0].traj.evaluate(a.branches[0].traj.t_end)
    # B 在 A 名义已经离开、A 已经到 x=8 之前穿过 A 的制动停点
    b_motion = straight(10.0, (stop[0], -3, 0), (stop[0], 3, 0), 3.0)
    b = plan_from(b_motion, 10.0, pva((stop[0], -3, 0)), [10.5], owner="u1")
    nominal_only = conflicts(a.nominal_occupancy(CFG), b.nominal_occupancy(CFG))
    assert not nominal_only
    assert conflicts(a.occupancy(CFG), b.occupancy(CFG))
    witness = conflict_witness(a.occupancy(CFG), b.occupancy(CFG))
    assert witness is not None and witness[0].startswith("branch")


def test_brake_tube_conflicts_without_the_stop_hold():
    """I14 的制动一半：冲突来自制动量管子，不是制动结束后的停点保持。"""
    motion = straight(0.0, (0, 0, 0), (8, 0, 0), 4.0)
    base = plan_from(motion, 0.0, pva((0, 0, 0)), [1.0])
    head = np.array([motion.evaluate(1.0, k) for k in range(3)])
    stop = head[0] + np.array([0.0, 2.5, 0.0])
    brake = PiecewisePolynomial(
        1.0, [quintic(head, np.array([stop, np.zeros(3), np.zeros(3)]), 2.5)]
    )
    owner = base.replace_branches([BrakeBranch(1.0, brake)])
    assert validate_plan(owner, CFG) == []
    point = brake.evaluate(2.0)
    start = point + np.array([0.0, 0.0, -0.8])
    end = point + np.array([0.0, 0.0, 0.8])
    intr = plan_from(
        straight(1.2, start, end, 1.6),
        1.2,
        pva(start),
        [1.6],
        owner="u1",
    )
    assert not conflicts(owner.nominal_occupancy(CFG), intr.nominal_occupancy(CFG))
    assert conflict_witness(owner.occupancy(CFG), intr.occupancy(CFG)) == ("branch0", "motion")
    without_tube = OccupancySet(
        tuple(e for e in owner.occupancy(CFG).elements if e.label != "branch0")
    )
    assert not conflicts(without_tube, intr.occupancy(CFG))


def test_union_of_old_and_new_plan_occupancy():
    old = plan_from(straight(0.0, (0, 0, 0), (4, 0, 0), 2.0), 0.0, pva((0, 0, 0)), [1.0])
    new = plan_from(straight(5.0, (4, 0, 0), (4, 5, 0), 3.0), 5.0, pva((4, 0, 0)), [5.5])
    both = old.occupancy(CFG).union(new.occupancy(CFG))
    # 探针在 t=1.0 经过 (2,0,0)，此时旧计划正好在那里
    probe = plan_from(
        straight(0.1, (2, -1.5, 0), (2, 1.5, 0), 1.8), 0.1, pva((2, -1.5, 0)), [0.5], owner="u1"
    )
    assert conflicts(both, probe.occupancy(CFG))
    assert not conflicts(new.occupancy(CFG), probe.occupancy(CFG))
    assert len(both.elements) == len(old.occupancy(CFG).elements) + len(new.occupancy(CFG).elements)


# --- plan_hash ---------------------------------------------------------------


def _trim(br, dt):
    return type(br)(br.t_branch, br.traj.with_validity(br.traj.t_start, br.traj.t_end - dt))


def _base():
    return plan_from(straight(0.0, (0, 0, 0), (6, 0, 0), 3.0), 0.0, pva((0, 0, 0)), [1.0])


def test_plan_hash_is_deterministic_and_field_order_independent():
    a, b = _base(), _base()
    assert a.plan_hash(CFG) == b.plan_hash(CFG)
    canon = a.canonical(CFG)
    shuffled = dict(reversed(list(canon.items())))
    from yopo_swarm_core.plan import canonical_digest

    assert canonical_digest(shuffled) == a.plan_hash(CFG)


@pytest.mark.parametrize(
    "mutate",
    [
        lambda p, c: (p.retimed(1e-9), c),  # 绝对时间
        lambda p, c: (p.with_owner("u9"), c),
        lambda p, c: (
            p,
            PlanConfig(
                limits=LIM,
                max_branch_points=4,
                body_radius=0.2,
                tracking_margin=0.1,
                frame="world-NED",
            ),
        ),
        lambda p, c: (
            p,
            PlanConfig(
                limits=LIM, max_branch_points=4, body_radius=0.2, tracking_margin=0.1, units="mm"
            ),
        ),
        lambda p, c: (
            p,
            PlanConfig(limits=LIM, max_branch_points=4, body_radius=0.21, tracking_margin=0.1),
        ),
        lambda p, c: (p.replace_branches([_trim(p.branches[0], 1e-6)]), c),  # 分支时长
    ],
)
def test_plan_hash_sensitive_to_safety_fields(mutate):
    p = _base()
    q, c = mutate(p, CFG)
    assert q.plan_hash(c) != p.plan_hash(CFG)


def test_plan_hash_sensitive_to_segment_duration():
    a = plan_from(straight(0.0, (0, 0, 0), (6, 0, 0), 3.0), 0.0, pva((0, 0, 0)), [1.0])
    b = plan_from(straight(0.0, (0, 0, 0), (6, 0, 0), 3.0 + 1e-9), 0.0, pva((0, 0, 0)), [1.0])
    assert a.plan_hash(CFG) != b.plan_hash(CFG)


# --- 计划执行状态与悬停计划（规划器与协议共用的唯一实现） -----------------------


def test_hover_plan_is_valid_and_holds_in_place():
    from yopo_swarm_core.plan import hover_plan, state_at

    p0 = np.array([1.0, -2.0, 0.5])
    h = hover_plan("u0", p0, 3.0, CFG)
    assert validate_plan(h, CFG) == []
    assert h.t_switch == 3.0 and h.parent_plan_hash == "genesis"
    for t in (3.0, 3.2, 100.0):
        np.testing.assert_allclose(state_at(h, t), [p0, np.zeros(3), np.zeros(3)])
    assert hover_plan("u0", p0, 0.0, CFG, parent="").parent_plan_hash == ""


def test_state_at_follows_motion_then_terminal_rest_and_branches():
    from yopo_swarm_core.plan import state_at

    plan = _base()
    m = plan.motion
    t_mid = 0.5 * (m.t_start + m.t_end)
    np.testing.assert_allclose(state_at(plan, t_mid)[0], m.evaluate(t_mid))
    rest = state_at(plan, m.t_end + 10.0)
    np.testing.assert_allclose(rest[0], plan.terminal_point())
    np.testing.assert_allclose(rest[1:], 0.0, atol=1e-12)
    b = plan.branches[0]
    t_b = b.t_branch + 0.01
    np.testing.assert_allclose(state_at(plan, t_b, active_branch=0)[0], b.traj.evaluate(t_b))


def _mono(coeffs, duration):
    c = np.zeros((3, len(coeffs)))
    c[0, : len(coeffs)] = coeffs
    return PiecewisePolynomial(0.0, [PolySegment(c, duration)])


def test_every_approved_brake_branch_passes_conservative_bounds():
    m = straight(0.0, (0, 0, 0), (6, 0, 0), 3.0)
    p = plan_from(m, 0.0, pva((0, 0, 0)), [0.5, 1.0, 1.5])
    assert len(p.branches) >= 1
    for traj in (p.motion, *(b.traj for b in p.branches)):
        for order, limit in CFG.limits.by_order():
            assert check_derivative_bound(traj, order, limit).verdict is Verdict.SAFE
    for br in p.branches:
        for k in range(3):
            np.testing.assert_allclose(
                br.traj.evaluate(br.t_branch, k), m.evaluate(br.t_branch, k), atol=1e-9
            )
        assert float(np.linalg.norm(m.evaluate(br.t_branch, 1))) > HOLD_TOL


def test_each_derivative_order_alone_rejects_the_branch():
    """只超速度、只超加速度、只超 jerk 都要被 #4 的保守检查拒绝。"""
    cases = [
        # p = 5 t，v = 5 > 4，a = j = 0
        (_mono([0.0, 5.0], 1.0), 1),
        # p = 5 t^2，a = 10 > 6；T=0.2 时 v<=2、j=0
        (_mono([0.0, 0.0, 5.0], 0.2), 2),
        # j = 50 > 40；T=0.05 时 a<=2.5、v<=0.0625。c3 = 50/6
        (_mono([0.0, 0.0, 0.0, 50.0 / 6.0], 0.05), 3),
    ]
    base = plan_from(straight(0.0, (0, 0, 0), (6, 0, 0), 3.0), 0.0, pva((0, 0, 0)), [0.8, 1.6])
    assert validate_plan(base, CFG) == []
    for traj, violated in cases:
        verdicts = {
            order: check_derivative_bound(traj, order, limit).verdict
            for order, limit in CFG.limits.by_order()
        }
        assert verdicts[violated] is Verdict.UNSAFE
        for order, value in verdicts.items():
            if order != violated:
                assert value is Verdict.SAFE
        shifted = traj.shifted(base.branches[1].t_branch - traj.t_start)
        bad = base.replace_branches(
            [base.branches[0], BrakeBranch(base.branches[1].t_branch, shifted)]
        )
        assert PlanReject.BRANCH_KINEMATICS in validate_plan(bad, CFG)


def test_branch_count_exactly_at_cap_is_accepted():
    cfg = PlanConfig(limits=LIM, max_branch_points=2, body_radius=0.2, tracking_margin=0.1)
    m = straight(0.0, (0, 0, 0), (6, 0, 0), 3.0)
    ok = make_plan("u0", 1, "p", m, 0.0, pva((0, 0, 0)), [0.5, 1.5], cfg)
    assert isinstance(ok, Plan)
    assert validate_plan(ok, cfg) == []
    assert len(ok.branches) == 2
    over = make_plan("u0", 1, "p", m, 0.0, pva((0, 0, 0)), [0.5, 1.0, 1.5], cfg)
    assert PlanReject.TOO_MANY_BRANCH_POINTS in over


@pytest.mark.parametrize(
    "head",
    [
        pva((0.2, 0, 0)),
        pva((0, 0, 0), (0, 0, 0), (0.5, 0, 0)),
    ],
)
def test_head_position_and_acceleration_discontinuity(head):
    m = straight(1.0, (0, 0, 0), (6, 0, 0), 3.0)
    res = make_plan("u0", 1, "p", m, 1.0, head, [1.5], CFG)
    assert PlanReject.HEAD_DISCONTINUITY in res


def test_branch_discontinuity_each_derivative():
    m = straight(0.0, (0, 0, 0), (6, 0, 0), 3.0)
    p = plan_from(m, 0.0, pva((0, 0, 0)), [1.0])
    pos, vel, acc = m.evaluate(1.0), m.evaluate(1.0, 1), m.evaluate(1.0, 2)
    cases = [
        pva(pos + (0.3, 0, 0), vel, acc),
        pva(pos, vel + (0.2, 0, 0), acc),
        pva(pos, vel, acc + (0.4, 0, 0)),
    ]
    for start in cases:
        jump = PiecewisePolynomial(1.0, [quintic(start, pva(pos + (1.0, 0, 0)), 2.0)])
        bad = p.replace_branches([BrakeBranch(1.0, jump)])
        assert PlanReject.BRANCH_DISCONTINUITY in validate_plan(bad, CFG)
        assert PlanReject.HOVER_AS_BRAKE not in validate_plan(bad, CFG)


def test_terminal_hold_survives_expiry_until_explicit_replacement():
    """多项式过期不能删掉仍停在终点的无人机；换成后继计划才释放。"""
    motion = straight(0.0, (0, 0, 0), (10, 0, 0), 5.0)
    old = plan_from(motion, 0.0, pva((0, 0, 0)), [0.75])
    with pytest.raises(TrajectoryExpired, match="outside validity"):
        old.motion.evaluate(100.0)
    holds = [
        e for e in old.occupancy(CFG).elements if isinstance(e, Hold) and e.label == "terminal"
    ]
    assert len(holds) == 1
    assert holds[0].t_from == old.t_end
    assert math.isinf(holds[0].t_until)
    intr = plan_from(
        straight(100.0, (10, -3, 0), (10, 3, 0), 3.0),
        100.0,
        pva((10, -3, 0)),
        [100.4],
        owner="u1",
    )
    assert conflict_witness(old.occupancy(CFG), intr.occupancy(CFG))[0] == "terminal"
    without = OccupancySet(tuple(e for e in old.occupancy(CFG).elements if e.label != "terminal"))
    assert not conflicts(without, intr.occupancy(CFG))
    # 时间截断不去掉仍在持续的终端保持。
    assert conflicts(old.occupancy(CFG).truncated_after(50.0), intr.occupancy(CFG))
    successor = plan_from(
        straight(5.0, (10, 0, 0), (10, 8, 0), 4.0),
        5.0,
        pva((10, 0, 0)),
        [8.2],
        owner="u0",
    )
    assert not conflicts(successor.occupancy(CFG), intr.occupancy(CFG))
    assert conflicts(old.occupancy(CFG), intr.occupancy(CFG))


def _reverse_keys(obj):
    if isinstance(obj, dict):
        return {k: _reverse_keys(v) for k, v in reversed(list(obj.items()))}
    if isinstance(obj, list):
        return [_reverse_keys(v) for v in obj]
    return obj


def test_plan_hash_ignores_nested_key_order():
    p = _base()
    assert canonical_digest(_reverse_keys(p.canonical(CFG))) == p.plan_hash(CFG)


def test_branch_time_input_order_does_not_change_hash():
    m = straight(0.0, (0, 0, 0), (6, 0, 0), 3.0)
    a = plan_from(m, 0.0, pva((0, 0, 0)), [0.5, 1.5])
    b = plan_from(m, 0.0, pva((0, 0, 0)), [1.5, 0.5])
    assert a.plan_hash(CFG) == b.plan_hash(CFG)


def test_segment_list_order_changes_plan_hash():
    base = _base()
    first = quintic(pva((0, 0, 0)), pva((1, 0, 0)), 1.0)
    second = quintic(pva((1, 0, 0)), pva((1, 2, 0)), 1.0)
    left = replace(base, motion=PiecewisePolynomial(0.0, [first, second]))
    right = replace(base, motion=PiecewisePolynomial(0.0, [second, first]))
    assert left.plan_hash(CFG) != right.plan_hash(CFG)


def _bump_motion_coeff(p, cfg):
    seg = p.motion.segments[0]
    coef = np.array(seg.coeffs, copy=True)
    coef[0, 0] = np.nextafter(coef[0, 0], np.inf)
    segs = (PolySegment(coef, seg.duration),) + p.motion.segments[1:]
    motion = PiecewisePolynomial(p.motion.t0, segs, p.motion.t_start, p.motion.t_end)
    return replace(p, motion=motion), cfg


def _bump_duration(p, cfg):
    seg = p.motion.segments[0]
    segs = (PolySegment(seg.coeffs, np.nextafter(seg.duration, np.inf)),) + p.motion.segments[1:]
    return replace(p, motion=PiecewisePolynomial(p.motion.t0, segs)), cfg


def _bump_validity(p, cfg):
    motion = p.motion.with_validity(p.motion.t_start, np.nextafter(p.motion.t_end, 0.0))
    return replace(p, motion=motion), cfg


def _bump_t0(p, cfg):
    motion = PiecewisePolynomial(np.nextafter(p.motion.t0, np.inf), p.motion.segments)
    return replace(p, motion=motion), cfg


def _bump_switch(p, cfg):
    return replace(p, t_switch=np.nextafter(p.t_switch, np.inf)), cfg


def _bump_head(p, cfg):
    head = np.array(p.head_pva, copy=True)
    head[0, 0] = np.nextafter(head[0, 0], np.inf)
    return replace(p, head_pva=head), cfg


def _bump_branch_time(p, cfg):
    b0 = p.branches[0]
    nxt = BrakeBranch(float(np.nextafter(b0.t_branch, np.inf)), b0.traj)
    return p.replace_branches((nxt, *p.branches[1:])), cfg


def _bump_branch_coeff(p, cfg):
    b0 = p.branches[0]
    seg = b0.traj.segments[0]
    coef = np.array(seg.coeffs, copy=True)
    coef[0, 0] = np.nextafter(coef[0, 0], np.inf)
    traj = PiecewisePolynomial(b0.traj.t0, [PolySegment(coef, seg.duration)])
    return p.replace_branches((BrakeBranch(b0.t_branch, traj), *p.branches[1:])), cfg


@pytest.mark.parametrize(
    "mutate",
    [
        _bump_motion_coeff,
        _bump_duration,
        _bump_validity,
        _bump_t0,
        _bump_switch,
        _bump_head,
        _bump_branch_time,
        _bump_branch_coeff,
        lambda p, c: (p, replace(c, frame="world-NED")),
        lambda p, c: (p, replace(c, units="mm")),
        lambda p, c: (p, replace(c, limits=KinematicLimits(np.nextafter(4.0, np.inf), 6.0, 40.0))),
        lambda p, c: (p, replace(c, limits=KinematicLimits(4.0, np.nextafter(6.0, np.inf), 40.0))),
        lambda p, c: (p, replace(c, limits=KinematicLimits(4.0, 6.0, np.nextafter(40.0, np.inf)))),
        lambda p, c: (p, replace(c, body_radius=np.nextafter(c.body_radius, np.inf))),
        lambda p, c: (p, replace(c, tracking_margin=np.nextafter(c.tracking_margin, np.inf))),
        lambda p, c: (p, replace(c, max_branch_points=c.max_branch_points + 1)),
        lambda p, c: (replace(p, parent_plan_hash=p.parent_plan_hash + "x"), c),
        lambda p, c: (replace(p, plan_seq=p.plan_seq + 1), c),
        lambda p, c: (p.with_owner(p.owner + "x"), c),
    ],
)
def test_plan_hash_sensitive_to_each_safety_field(mutate):
    p = _base()
    q, cfg = mutate(p, CFG)
    assert q.plan_hash(cfg) != p.plan_hash(CFG)
