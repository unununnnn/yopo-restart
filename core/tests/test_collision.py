"""成对连续时间冲突检查。UNSAFE 用例都由解析构造给出真实最近距离；密集采样只作交叉检验。"""

import math
from itertools import pairwise

import mpmath
import numpy as np
import pytest
from _collision_pre_g1 import (
    check_derivative_bound as check_derivative_bound_pre,
)
from _collision_pre_g1 import (
    check_separation as check_separation_pre,
)
from hypothesis import given, settings
from hypothesis import seed as hypothesis_seed
from hypothesis import strategies as st

from yopo_swarm_core.collision import (
    DEFAULT_TOL,
    ConstantMargin,
    LinearMargin,
    Reason,
    Verdict,
    check_derivative_bound,
    check_separation,
)
from yopo_swarm_core.trajectory import PiecewisePolynomial, PolySegment


def line(p0, v, t0, duration):
    coeffs = np.array([[p0[k], v[k]] for k in range(3)], dtype=float)
    return PiecewisePolynomial(t0, [PolySegment(coeffs, duration)])


def polyline(t0, points, durations):
    """分段直线，经过 points[i]，第 i 段时长 durations[i]。"""
    segs = []
    for p, q, d in zip(points[:-1], points[1:], durations):
        p, q = np.asarray(p, float), np.asarray(q, float)
        segs.append(PolySegment(np.stack([p, (q - p) / d], axis=1), d))
    return PiecewisePolynomial(t0, segs)


def dist(a, b, t):
    return float(np.linalg.norm(a.evaluate(t) - b.evaluate(t)))


def dense_values(traj, ts, order=0):
    """独立于被测检查器的向量化求值，段边界语义与 evaluate 一致（取后一段）。"""
    ts = np.asarray(ts, float)
    bounds = np.asarray(traj.boundaries())
    idx = np.clip(np.searchsorted(bounds, ts, side="right") - 1, 0, len(traj.segments) - 1)
    out = np.empty((len(ts), 3))
    for i, seg in enumerate(traj.segments):
        mask = idx == i
        if mask.any():
            tau = ts[mask] - bounds[i]
            dc = seg.derivative_coeffs(order)
            out[mask] = np.stack([np.polynomial.polynomial.polyval(tau, row) for row in dc], 1)
    return out


def dense_times(lo, hi, *trajs):
    extra = [t for tr in trajs for t in tr.boundaries() if lo <= t <= hi]
    return np.concatenate([np.linspace(lo, hi, 20001), extra])


def assert_witness(result, a, b, margin):
    assert result.verdict is Verdict.UNSAFE
    assert result.witness_time is not None
    assert dist(a, b, result.witness_time) < margin.at(result.witness_time)


# 高精度检查直接吃存放的 power-basis 系数（IEEE 位型），不经过 evaluate。
# 每个原子区间上 ||r||^2 的临界点是多项式极小；断点再用右连续段的值补一次，
# 和检查器「段边界归入后一段」一致。密集采样不是这里的判定依据。
_HP_DPS = 40
DENSE_PROPERTY_SEED = 20261025
DENSE_PROPERTY_EXAMPLES = 500
HP_PROPERTY_SEED = 20261025
HP_PROPERTY_EXAMPLES = 500
TOL_BAND_DIFF_SEED = 20261025
TOL_BAND_DIFF_PAIRS = 48
TOL_BAND_DIFF_TRAJS = 24


def _mpf_from_float(x):
    """binary64 → mpmath，按整数尾数，不走十进制。"""
    fv = float(x)
    if fv == 0.0:
        return mpmath.mpf("-0") if math.copysign(1.0, fv) < 0 else mpmath.mpf(0)
    ui = int(np.float64(fv).view(np.uint64))
    sign = -1 if ui >> 63 else 1
    exp = int((ui >> 52) & 0x7FF)
    mant = int(ui & ((1 << 52) - 1))
    if exp == 0x7FF:
        raise ValueError(f"non-finite value in high-precision check: {fv}")
    if exp == 0:
        return mpmath.mpf(sign * mant) * mpmath.power(2, -1074)
    return mpmath.mpf(sign * (mant + (1 << 52))) * mpmath.power(2, exp - 1075)


def _prepare_mp():
    mpmath.mp.dps = _HP_DPS


def _pval(coeffs, u):
    acc = mpmath.mpf(0)
    pwr = mpmath.mpf(1)
    for c in coeffs:
        acc += c * pwr
        pwr *= u
    return acc


def _pmul(a, b):
    out = [mpmath.mpf(0)] * (len(a) + len(b) - 1)
    for i, ai in enumerate(a):
        if ai == 0:
            continue
        for j, bj in enumerate(b):
            out[i + j] += ai * bj
    return out


def _padd(a, b):
    n = max(len(a), len(b))
    out = [mpmath.mpf(0)] * n
    for i, v in enumerate(a):
        out[i] += v
    for i, v in enumerate(b):
        out[i] += v
    return out


def _reparam_mp(row, tau0):
    """q(u) = p(tau0 + u)，u 从 0 到区间长度。"""
    out = [mpmath.mpf(0)] * len(row)
    for k, ck in enumerate(row):
        if ck == 0:
            continue
        for j in range(k + 1):
            out[j] += ck * mpmath.binomial(k, j) * (tau0 ** (k - j))
    return out


def _diff_rows(rows, order):
    cur = rows
    for _ in range(order):
        nxt = []
        for row in cur:
            if len(row) <= 1:
                nxt.append([mpmath.mpf(0)])
            else:
                nxt.append([i * row[i] for i in range(1, len(row))])
        cur = nxt
    return cur


def _segment_at(traj, t):
    bounds = traj.boundaries()
    last = len(traj.segments) - 1
    for i, seg in enumerate(traj.segments):
        if t < bounds[i + 1] or i == last:
            return seg, bounds[i]
    raise AssertionError(t)


def _rows_of(seg):
    return [[_mpf_from_float(v) for v in row] for row in seg.coeffs]


def _norm2(rows):
    acc = [mpmath.mpf(0)]
    for row in rows:
        acc = _padd(acc, _pmul(row, row))
    return acc


def _critical_us(d2, h):
    der = [i * d2[i] for i in range(1, len(d2))]
    if not der:
        return []
    scale = max(abs(c) for c in der)
    if scale == 0:
        return []
    while len(der) > 1 and abs(der[-1]) <= scale * mpmath.mpf("1e-24"):
        der.pop()
        scale = max((abs(c) for c in der), default=mpmath.mpf(0))
        if scale == 0:
            return []
    if len(der) <= 1:
        return []
    roots = mpmath.polyroots(list(reversed(der)), maxsteps=200, extraprec=20)
    found = []
    for rt in roots:
        if abs(rt.imag) > mpmath.mpf("1e-12"):
            continue
        u = rt.real
        if mpmath.mpf(0) <= u <= h:
            found.append(u)
    return found


def _candidate_norms(rows, h):
    """端点和 ||q||^2 临界点上的范数。极小、极大都在这组点里。"""
    d2 = _norm2(rows)
    found = []
    for u in (mpmath.mpf(0), h, *_critical_us(d2, h)):
        val = _pval(d2, u)
        if val < 0:
            if abs(val) > mpmath.mpf("1e-12"):
                raise AssertionError(f"negative squared norm {val}")
            val = mpmath.mpf(0)
        found.append(mpmath.sqrt(val))
    return found


def _breakpoints(lo, hi, *trajs):
    pts = [lo, hi]
    for tr in trajs:
        for t in tr.boundaries():
            if lo < t < hi:
                pts.append(t)
    return sorted(set(pts))


def _interior_mid(s, e):
    mid = s + 0.5 * (e - s)
    if not s < mid < e:
        mid = math.nextafter(s, e)
    return mid


def hp_norm_at(traj, t, order=0):
    _prepare_mp()
    seg, start = _segment_at(traj, t)
    tau = _mpf_from_float(t) - _mpf_from_float(start)
    rows = _diff_rows(_rows_of(seg), order)
    acc = mpmath.mpf(0)
    for row in rows:
        acc += _pval(row, tau) ** 2
    if acc < 0:
        acc = mpmath.mpf(0)
    return mpmath.sqrt(acc)


def hp_peak_norm(traj, order, lo, hi):
    """区间上 ||第 order 阶导数|| 的高精度上确界（临界点 + 右连续断点）。"""
    _prepare_mp()
    pts = _breakpoints(lo, hi, traj)
    best = mpmath.mpf(0)
    for s, e in pairwise(pts):
        seg, start = _segment_at(traj, _interior_mid(s, e))
        tau0 = _mpf_from_float(s) - _mpf_from_float(start)
        h = _mpf_from_float(e) - _mpf_from_float(s)
        rows = [_reparam_mp(row, tau0) for row in _diff_rows(_rows_of(seg), order)]
        best = max(best, max(_candidate_norms(rows, h)))
    for t in pts:
        best = max(best, hp_norm_at(traj, t, order))
    return best


def hp_min_distance(a, b, lo, hi):
    _prepare_mp()
    pts = _breakpoints(lo, hi, a, b)
    best = None

    def consider(val):
        nonlocal best
        best = val if best is None else min(best, val)

    for s, e in pairwise(pts):
        mid = _interior_mid(s, e)
        h = _mpf_from_float(e) - _mpf_from_float(s)
        rel = []
        width = 0
        for tr in (a, b):
            seg, start = _segment_at(tr, mid)
            tau0 = _mpf_from_float(s) - _mpf_from_float(start)
            rows = [_reparam_mp(row, tau0) for row in _rows_of(seg)]
            rel.append(rows)
            width = max(width, max(len(row) for row in rows))
        diff = []
        for axis in range(3):
            ra = rel[0][axis] + [mpmath.mpf(0)] * (width - len(rel[0][axis]))
            rb = rel[1][axis] + [mpmath.mpf(0)] * (width - len(rel[1][axis]))
            diff.append([x - y for x, y in zip(ra, rb)])
        consider(min(_candidate_norms(diff, h)))
    for t in pts:
        dacc = mpmath.mpf(0)
        sa, a0 = _segment_at(a, t)
        sb, b0 = _segment_at(b, t)
        pa = _rows_of(sa)
        pb = _rows_of(sb)
        for axis in range(3):
            va = _pval(pa[axis], _mpf_from_float(t) - _mpf_from_float(a0))
            vb = _pval(pb[axis], _mpf_from_float(t) - _mpf_from_float(b0))
            dacc += (va - vb) ** 2
        if dacc < 0:
            dacc = mpmath.mpf(0)
        consider(mpmath.sqrt(dacc))
    return best


def assert_hp_distance(a, b, expected, atol=1e-9):
    lo, hi = max(a.t_start, b.t_start), min(a.t_end, b.t_end)
    got = hp_min_distance(a, b, lo, hi)
    assert abs(got - mpmath.mpf(expected)) <= atol, got
    return got


def test_far_apart_parallel_lines_are_safe():
    a = line((0, 0, 0), (1, 0, 0), 0.0, 10.0)
    b = line((0, 5, 0), (1, 0, 0), 0.0, 10.0)
    r = check_separation(a, b, ConstantMargin(1.0))
    assert r.verdict is Verdict.SAFE
    assert r.lower_bound is not None and r.lower_bound > 1.0


def test_near_miss_between_ten_samples_is_unsafe():
    # a 沿 x 以 20 m/s 穿过 b 旁边，最近距离 0.9 发生在 t*=0.555。
    # 十点采样 t=0,0.1,…,1.0 时所有距离都 > 1.0（t=0.5/0.6 处 dx=1.1/0.9）。
    a = line((-11.1, 0.9, 0), (20, 0, 0), 0.0, 1.0)
    b = line((0, 0, 0), (0, 0, 0), 0.0, 1.0)
    samples = [dist(a, b, t) for t in np.linspace(0, 1, 11)]
    assert min(samples) > 1.0  # 采样会漏掉
    m = ConstantMargin(1.0)
    r = check_separation(a, b, m)
    assert_witness(r, a, b, m)
    assert abs(r.witness_time - 0.555) < 0.05
    assert_hp_distance(a, b, "0.9")


def test_crossing_lines_unsafe_with_witness_near_crossing_time():
    a = line((-5, 0, 0), (1, 0, 0), 0.0, 10.0)
    b = line((0, -5, 0), (0, 1, 0), 0.0, 10.0)
    m = ConstantMargin(1.0)
    r = check_separation(a, b, m)
    assert_witness(r, a, b, m)
    assert_hp_distance(a, b, 0)


def test_grazing_tangency_is_never_safe():
    # 最近距离恰为 1.0 = 要求间距
    a = line((-5, 1.0, 0), (1, 0, 0), 0.0, 10.0)
    b = line((0, 0, 0), (0, 0, 0), 0.0, 10.0)
    r = check_separation(a, b, ConstantMargin(1.0))
    assert r.verdict is not Verdict.SAFE
    assert_hp_distance(a, b, 1)


def test_very_short_segment_collision_is_found():
    eps = 1e-6
    # a 在 [1, 1+eps] 这一极短段内经过原点
    a = polyline(0.0, [(-1, 0, 0), (0, 0, 0), (eps, 0, 0), (1, 0, 0)], [1.0, eps, 1.0])
    b = line((0, 0, 0), (0, 0, 0), 0.0, 3.0)
    m = ConstantMargin(0.1)
    r = check_separation(a, b, m)
    assert_witness(r, a, b, m)
    assert_hp_distance(a, b, 0)


def test_very_short_common_window_still_decides():
    a = line((0, 0, 0), (1, 0, 0), 0.0, 1.0)
    b = line((0, 5, 0), (1, 0, 0), 1.0 - 1e-9, 1.0)
    r = check_separation(a, b, ConstantMargin(1.0))
    assert r.verdict is Verdict.SAFE


def test_non_overlapping_windows_are_unknown():
    a = line((0, 0, 0), (1, 0, 0), 0.0, 1.0)
    b = line((0, 0, 0), (1, 0, 0), 2.0, 1.0)
    r = check_separation(a, b, ConstantMargin(1.0))
    assert r.verdict is Verdict.UNKNOWN
    assert r.reason is Reason.NO_COMMON_WINDOW


def test_requested_window_not_covered_by_both_is_unknown():
    a = line((0, 0, 0), (1, 0, 0), 0.0, 1.0)
    b = line((0, 5, 0), (1, 0, 0), 0.0, 3.0)
    r = check_separation(a, b, ConstantMargin(1.0), window=(0.0, 2.0))
    assert r.verdict is Verdict.UNKNOWN
    assert r.reason is Reason.NO_COMMON_WINDOW


def test_collision_exactly_at_segment_boundary():
    # a 在 t=1 到达原点并折返（V 形），速度在边界处不连续
    a = polyline(0.0, [(-1, 0, 0), (0, 0, 0), (-1, 1, 0)], [1.0, 1.0])
    b = line((0, 0, 0), (0, 0, 0), 0.0, 2.0)
    m = ConstantMargin(0.5)
    r = check_separation(a, b, m)
    assert_witness(r, a, b, m)
    assert_hp_distance(a, b, 0)


def test_discontinuity_at_segment_boundary_is_not_hidden():
    # b 在 t=1 处从 z=1 跳到原点（段边界取后一段的值）；a 静止在原点
    up = PolySegment(np.array([[0.0], [0.0], [1.0]]), 1.0)
    down = PolySegment(np.zeros((3, 1)), 1.0)
    b = PiecewisePolynomial(0.0, [up, down])
    a = line((0, 0, 0), (0, 0, 0), 0.0, 1.0)
    m = ConstantMargin(0.5)
    r = check_separation(a, b, m)
    assert_witness(r, a, b, m)
    assert_hp_distance(a, b, 0)


def test_clock_shift_of_both_preserves_verdict_and_shifts_witness():
    a = line((-5, 0, 0), (1, 0, 0), 0.0, 10.0)
    b = line((0, -5, 0), (0, 1, 0), 0.0, 10.0)
    m = ConstantMargin(1.0)
    r0 = check_separation(a, b, m)
    shifted_a, shifted_b = a.shifted(1000.0), b.shifted(1000.0)
    r1 = check_separation(shifted_a, shifted_b, m)
    assert r1.verdict is Verdict.UNSAFE
    assert r1.witness_time == pytest.approx(r0.witness_time + 1000.0, abs=1e-6)
    assert_hp_distance(shifted_a, shifted_b, 0)


def test_clock_shift_of_one_changes_geometry():
    # b 晚 3 s：解析最近距离 sqrt(4.5)≈2.12 > 1
    a = line((-5, 0, 0), (1, 0, 0), 0.0, 20.0)
    b = line((0, -5, 0), (0, 1, 0), 0.0, 20.0).shifted(3.0)
    r = check_separation(a, b, ConstantMargin(1.0))
    assert r.verdict is Verdict.SAFE


@pytest.mark.parametrize(
    "bad",
    [
        PolySegment(np.array([[math.nan, 1.0], [0, 0], [0, 0]]), 1.0),
        PolySegment(np.array([[0.0, math.inf], [0, 0], [0, 0]]), 1.0),
        PolySegment(np.array([[-math.inf, 0.0], [0, 0], [0, 0]]), 1.0),
        PolySegment(np.zeros((3, 2)), 0.0),
        PolySegment(np.zeros((3, 2)), -1.0),
        PolySegment(np.zeros((3, 2)), math.nan),
        PolySegment(np.zeros((3, 2)), math.inf),
        PolySegment(np.zeros((3, 2)), -math.inf),
    ],
)
def test_malformed_input_is_unknown_not_exception(bad):
    a = PiecewisePolynomial(0.0, [bad])
    b = line((0, 50, 0), (0, 0, 0), -10.0, 20.0)
    r = check_separation(a, b, ConstantMargin(1.0))
    assert r.verdict is Verdict.UNKNOWN
    assert r.verdict is not Verdict.SAFE
    assert r.reason is Reason.MALFORMED
    for order in (1, 2, 3):
        d = check_derivative_bound(a, order, 1.0)
        assert d.verdict is Verdict.UNKNOWN
        assert d.verdict is not Verdict.SAFE
        assert d.reason is Reason.MALFORMED


def test_non_finite_margin_is_unknown():
    a = line((0, 0, 0), (1, 0, 0), 0.0, 1.0)
    b = line((0, 5, 0), (1, 0, 0), 0.0, 1.0)
    for margin in (
        ConstantMargin(math.nan),
        ConstantMargin(math.inf),
        ConstantMargin(-math.inf),
        LinearMargin(0.0, math.nan, 1.0),
        LinearMargin(0.0, 1.0, math.inf),
    ):
        r = check_separation(a, b, margin)
        assert r.verdict is Verdict.UNKNOWN
        assert r.verdict is not Verdict.SAFE
    assert check_derivative_bound(a, 1, math.nan).verdict is Verdict.UNKNOWN
    nan_t0 = PiecewisePolynomial(math.nan, [PolySegment(np.zeros((3, 2)), 1.0)])
    assert check_separation(nan_t0, b, ConstantMargin(1.0)).verdict is Verdict.UNKNOWN
    assert check_derivative_bound(nan_t0, 1, 1.0).verdict is Verdict.UNKNOWN


class TestTimeVaryingMargin:
    a = line((0, 0, 0), (0, 0, 0), 0.0, 1.0)
    b = line((0, 2.0, 0), (0, 0, 0), 0.0, 1.0)  # 恒定距离 2.0

    def test_margin_growing_past_distance_is_unsafe_late(self):
        m = LinearMargin(t0=0.0, value0=1.0, rate=2.0)  # 1 → 3
        r = check_separation(self.a, self.b, m)
        assert_witness(r, self.a, self.b, m)
        assert r.witness_time > 0.5
        assert_hp_distance(self.a, self.b, 2)

    def test_margin_staying_below_distance_is_safe(self):
        m = LinearMargin(t0=0.0, value0=1.0, rate=0.9)  # 1 → 1.9
        assert check_separation(self.a, self.b, m).verdict is Verdict.SAFE

    def test_margin_reaching_distance_at_endpoint_is_not_safe(self):
        m = LinearMargin(t0=0.0, value0=1.0, rate=1.0)  # 恰在 t=1 达到 2.0
        assert check_separation(self.a, self.b, m).verdict is not Verdict.SAFE
        assert_hp_distance(self.a, self.b, 2)


def test_extra_split_times_are_accepted():
    a = line((-5, 0, 0), (1, 0, 0), 0.0, 10.0)
    b = line((0, -5, 0), (0, 1, 0), 0.0, 10.0)
    m = ConstantMargin(1.0)
    r = check_separation(a, b, m, split_times=[2.5, 7.0])
    assert_witness(r, a, b, m)
    assert_hp_distance(a, b, 0)


def test_tiny_budget_returns_unknown_rather_than_safe():
    # 斜向近擦边（解析最近距离 1.0001），包围盒在粗分辨率下很松：极小预算下只能 UNKNOWN
    c = math.sqrt(2) * 1.0001
    a = line((-5, 0, 0), (1, 0, 0), 0.0, 10.0)
    b = line((0, -5 + c, 0), (0, 1, 0), 0.0, 10.0)
    r = check_separation(a, b, ConstantMargin(1.0), max_depth=2)
    assert r.verdict is Verdict.UNKNOWN
    assert r.reason is Reason.BUDGET_EXHAUSTED
    # 真最小距离是 |c|/sqrt(2) = 1.0001，大于间距；UNKNOWN 来自包络预算，不是漏报碰撞。
    got = hp_min_distance(a, b, max(a.t_start, b.t_start), min(a.t_end, b.t_end))
    assert abs(got - _mpf_from_float(c) / mpmath.sqrt(2)) <= mpmath.mpf("1e-9")
    assert got > 1


# --- 动力学约束 -------------------------------------------------------------


def velocity_bump(peak=10.0, t_peak=0.55):
    # v_x(t) = peak - 20 (t - t_peak)^2，峰值在两个采样点 0.5/0.6 之间（采样最大 9.95）
    k = 20.0
    c = np.zeros((3, 4))
    c[0] = [0.0, peak - k * t_peak**2, k * t_peak, -k / 3]
    return PiecewisePolynomial(0.0, [PolySegment(c, 1.0)])


def test_velocity_peak_between_samples_violation_detected():
    traj = velocity_bump()
    samples = np.linspace(0, 1, 11)
    assert max(abs(traj.evaluate(t, 1)[0]) for t in samples) < 9.97
    assert max(hp_norm_at(traj, float(t), 1) for t in samples) < 9.97
    peak = hp_peak_norm(traj, 1, 0.0, 1.0)
    assert peak > 9.97
    assert abs(peak - 10) <= mpmath.mpf("1e-9")
    r = check_derivative_bound(traj, order=1, limit=9.97)
    assert r.verdict is Verdict.UNSAFE
    assert hp_norm_at(traj, r.witness_time, 1) > 9.97
    assert min(abs(r.witness_time - t) for t in samples) > 1e-3


def test_velocity_within_limit_is_safe():
    assert check_derivative_bound(velocity_bump(), order=1, limit=10.5).verdict is Verdict.SAFE


def test_velocity_exactly_at_limit_is_not_safe():
    traj = velocity_bump()
    assert abs(hp_peak_norm(traj, 1, 0.0, 1.0) - 10) <= mpmath.mpf("1e-9")
    assert check_derivative_bound(traj, order=1, limit=10.0).verdict is not Verdict.SAFE


@pytest.mark.parametrize("order,limit", [(2, 25.0), (3, 30.0)])
def test_acceleration_and_jerk_bounds(order, limit):
    # a_x = -40 (t - 0.55)，|a| 最大 22 在端点 t=0；jerk 恒为 -40（采样点上也超限）。
    traj = velocity_bump()
    peak = hp_peak_norm(traj, order, 0.0, 1.0)
    if order == 2:
        assert peak < limit
    else:
        assert peak > limit
    r = check_derivative_bound(traj, order=order, limit=limit)
    expect = Verdict.SAFE if order == 2 else Verdict.UNSAFE
    assert r.verdict is expect


def test_derivative_bound_on_malformed_is_unknown():
    traj = PiecewisePolynomial(0.0, [PolySegment(np.zeros((3, 2)), -1.0)])
    assert check_derivative_bound(traj, 1, 1.0).verdict is Verdict.UNKNOWN


def _quadratic_bump(order, peak=10.02, k=24.0, tp=0.53125):
    """第 order 阶导数为 peak - k (t-tp)^2，峰值在 11 等分采样点之间。"""
    c0 = peak - k * tp**2
    c1 = 2 * k * tp
    c2 = -k
    # p^{(order)} = c0 + c1 t + c2 t^2，积分 order 次得到位置系数。
    deriv = [c0, c1, c2]
    pos = deriv
    for _ in range(order):
        pos = [0.0] + [pos[i] / (i + 1) for i in range(len(pos))]
    coeffs = np.zeros((3, len(pos)))
    coeffs[0] = pos
    return PiecewisePolynomial(0.0, [PolySegment(coeffs, 1.0)])


@pytest.mark.parametrize("order", [2, 3])
def test_derivative_peak_between_eleven_samples_is_unsafe(order):
    limit = 10.0
    traj = _quadratic_bump(order)
    samples = np.linspace(0, 1, 11)
    sample_peak = max(hp_norm_at(traj, float(t), order) for t in samples)
    peak = hp_peak_norm(traj, order, 0.0, 1.0)
    assert sample_peak < limit < peak
    r = check_derivative_bound(traj, order=order, limit=limit)
    assert r.verdict is Verdict.UNSAFE
    assert hp_norm_at(traj, r.witness_time, order) > limit
    assert min(abs(r.witness_time - float(t)) for t in samples) > 1e-3
    assert (
        check_derivative_bound(traj, order=order, limit=float(peak) + 0.5).verdict is Verdict.SAFE
    )


def test_tol_band_midpoint_does_not_hide_interior_collision():
    """端点和中点都贴着间距，原点穿越发生在 t=0.25。不能因此返回 UNKNOWN。"""
    a = line((-1, 0, 0), (4, 0, 0), 0.0, 1.0)
    b = line((0, 0, 0), (0, 0, 0), 0.0, 1.0)
    assert_hp_distance(a, b, 0)
    r = check_separation(a, b, ConstantMargin(1.0))
    assert r.verdict is Verdict.UNSAFE


def test_tol_band_midpoint_does_not_hide_interior_speed_peak():
    traj = velocity_bump()
    limit = float(hp_norm_at(traj, 0.5625, 1))
    peak = hp_peak_norm(traj, 1, 0.0, 1.0)
    assert limit < peak
    r = check_derivative_bound(traj, order=1, limit=limit)
    assert r.verdict is Verdict.UNSAFE
    assert hp_norm_at(traj, r.witness_time, 1) > limit


def test_constant_gap_envelope_respects_tolerance_boundary():
    """静止间距取自存放的系数。容差带内不得 SAFE，带外可 SAFE，带内深一点必须 UNSAFE。"""
    from yopo_swarm_core.collision import DEFAULT_TOL

    gap = 1.0
    a = line((0, gap, 0), (0, 0, 0), 0.0, 1.0)
    b = line((0, 0, 0), (0, 0, 0), 0.0, 1.0)
    stored = hp_min_distance(a, b, 0.0, 1.0)
    assert stored == mpmath.mpf(gap)
    tol = DEFAULT_TOL
    inside = check_separation(a, b, ConstantMargin(gap - 0.5 * tol))
    assert inside.verdict is not Verdict.SAFE
    clear = check_separation(a, b, ConstantMargin(gap - 100 * tol))
    assert clear.verdict is Verdict.SAFE
    assert clear.lower_bound is not None and clear.lower_bound > gap - 100 * tol
    violated = check_separation(a, b, ConstantMargin(gap + 2 * tol))
    assert violated.verdict is Verdict.UNSAFE


def test_loose_aabb_envelope_is_not_declared_safe_without_subdivision():
    """对角线相对运动：端点包围盒下界为 0，真最小距离是 sqrt(2)。"""
    a = line((2, 0, 0), (-2, 2, 0), 0.0, 1.0)
    b = line((0, 0, 0), (0, 0, 0), 0.0, 1.0)
    p0, p1 = np.array([2.0, 0.0, 0.0]), np.array([0.0, 2.0, 0.0])
    box_lo, box_hi = np.minimum(p0, p1), np.maximum(p0, p1)
    gap = np.maximum(np.maximum(box_lo, -box_hi), 0.0)
    assert np.linalg.norm(gap) == 0.0
    got = hp_min_distance(a, b, 0.0, 1.0)
    assert abs(got - mpmath.sqrt(2)) <= mpmath.mpf("1e-12")
    assert check_separation(a, b, ConstantMargin(1.0), max_depth=0).verdict is Verdict.UNKNOWN
    proven = check_separation(a, b, ConstantMargin(1.0))
    assert proven.verdict is Verdict.SAFE
    assert proven.lower_bound is not None and proven.lower_bound > 1.0
    tangent = check_separation(a, b, ConstantMargin(float(mpmath.sqrt(2))))
    assert tangent.verdict is not Verdict.SAFE
    violated = check_separation(a, b, ConstantMargin(2.0))
    assert violated.verdict is Verdict.UNSAFE


# --- 属性测试 ---------------------------------------------------------------

coef = st.floats(min_value=-3.0, max_value=3.0, allow_nan=False, allow_infinity=False)


@st.composite
def random_traj(draw):
    nseg = draw(st.integers(1, 3))
    segs = []
    for _ in range(nseg):
        c = np.array(draw(st.lists(coef, min_size=12, max_size=12))).reshape(3, 4)
        d = draw(st.floats(min_value=0.05, max_value=1.5))
        segs.append(PolySegment(c, d))
    t0 = draw(st.floats(min_value=-0.5, max_value=0.5))
    return PiecewisePolynomial(t0, segs)


@hypothesis_seed(DENSE_PROPERTY_SEED)
@settings(max_examples=DENSE_PROPERTY_EXAMPLES, deadline=None)
@given(random_traj(), random_traj(), st.floats(min_value=0.1, max_value=3.0))
def test_property_safe_verdict_never_contradicted_by_dense_check(a, b, required):
    m = ConstantMargin(required)
    r = check_separation(a, b, m)
    if r.verdict is Verdict.SAFE:
        lo, hi = max(a.t_start, b.t_start), min(a.t_end, b.t_end)
        ts = dense_times(lo, hi, a, b)
        d = np.linalg.norm(dense_values(a, ts) - dense_values(b, ts), axis=1)
        assert d.min() >= required
    elif r.verdict is Verdict.UNSAFE:
        assert dist(a, b, r.witness_time) < required


@hypothesis_seed(DENSE_PROPERTY_SEED)
@settings(max_examples=DENSE_PROPERTY_EXAMPLES, deadline=None)
@given(random_traj(), st.integers(1, 3), st.floats(min_value=0.5, max_value=40.0))
def test_property_derivative_safe_never_contradicted(traj, order, limit):
    r = check_derivative_bound(traj, order, limit)
    ts = dense_times(traj.t_start, traj.t_end, traj)
    peak = np.linalg.norm(dense_values(traj, ts, order), axis=1).max()
    if r.verdict is Verdict.SAFE:
        assert peak <= limit
    elif r.verdict is Verdict.UNSAFE:
        assert np.linalg.norm(traj.evaluate(r.witness_time, order)) > limit


def test_near_miss_minimum_confirmed_with_high_precision():
    """十进制构造的驻点是 0.9；存放系数的高精度最小值同样低于间距。"""
    mpmath.mp.dps = 50
    r2 = lambda t: (mpmath.mpf("-11.1") + 20 * t) ** 2 + mpmath.mpf("0.81")
    t_star = mpmath.findroot(lambda t: mpmath.diff(r2, t), mpmath.mpf("0.5"))
    assert abs(t_star - mpmath.mpf("0.555")) < mpmath.mpf("1e-40")
    assert abs(mpmath.sqrt(r2(t_star)) - mpmath.mpf("0.9")) < mpmath.mpf("1e-40")
    a = line((-11.1, 0.9, 0), (20, 0, 0), 0.0, 1.0)
    b = line((0, 0, 0), (0, 0, 0), 0.0, 1.0)
    stored = hp_min_distance(a, b, 0.0, 1.0)
    assert stored < 1
    assert abs(stored - mpmath.mpf("0.9")) <= mpmath.mpf("1e-9")


def _shift_x(traj, dx):
    segs = []
    for seg in traj.segments:
        coef = np.array(seg.coeffs, dtype=float, copy=True)
        coef[0, 0] += dx
        segs.append(PolySegment(coef, seg.duration))
    return PiecewisePolynomial(traj.t0, segs)


def _pos_radius(traj):
    rad = 0.0
    for seg in traj.segments:
        acc = 0.0
        dur = float(seg.duration)
        for row in seg.coeffs:
            bound = sum(abs(float(row[k])) * dur**k for k in range(len(row)))
            acc += bound * bound
        rad = max(rad, math.sqrt(acc))
    return rad


def test_property_safe_pairs_have_no_high_precision_point_below_separation():
    """凡判 SAFE 的轨迹对，mpmath 临界点都不得低于间距。种子和例数固定。"""
    rng = np.random.Generator(np.random.PCG64(HP_PROPERTY_SEED))
    safe_checked = 0
    worst_slack = mpmath.mpf("inf")
    for i in range(HP_PROPERTY_EXAMPLES):
        a = _rng_traj(rng)
        b = _rng_traj(rng)
        required = float(rng.uniform(0.1, 3.0))
        shifted = i % 2 == 0
        if shifted:
            # 平移到同一起点，保证有公共时间窗；再沿 x 拉开到半径和之外。
            b = _shift_x(b, _pos_radius(a) + _pos_radius(b) + required + 1.0)
            b = b.shifted(a.t_start - b.t_start)
        lo, hi = max(a.t_start, b.t_start), min(a.t_end, b.t_end)
        if not hi > lo:
            assert check_separation(a, b, ConstantMargin(required)).verdict is not Verdict.SAFE
            continue
        result = check_separation(a, b, ConstantMargin(required))
        if shifted:
            assert result.verdict is Verdict.SAFE
        if result.verdict is Verdict.SAFE:
            got = hp_min_distance(a, b, lo, hi)
            slack = got - mpmath.mpf(required)
            assert slack >= 0, (i, got, required)
            worst_slack = min(worst_slack, slack)
            safe_checked += 1
        elif result.verdict is Verdict.UNSAFE:
            assert dist(a, b, result.witness_time) < required
    assert safe_checked >= HP_PROPERTY_EXAMPLES // 2
    print(
        f"HP_PROPERTY seed={HP_PROPERTY_SEED} examples={HP_PROPERTY_EXAMPLES} "
        f"safe_checked={safe_checked} worst_slack={float(worst_slack):.16e}"
    )


def _rng_traj(rng):
    nseg = int(rng.integers(1, 4))
    segs = []
    for _ in range(nseg):
        coef = rng.uniform(-3.0, 3.0, size=(3, 4))
        segs.append(PolySegment(coef, float(rng.uniform(0.05, 1.5))))
    return PiecewisePolynomial(float(rng.uniform(-0.5, 0.5)), segs)


def test_tol_band_fix_adds_no_safe_verdicts():
    """55cc533 只增严：新 SAFE 蕴含 e9762b1 冻结检查器也是 SAFE。

    容差带中点不再提前结束细分，所以旧 UNKNOWN 可以变成新 UNSAFE。
    随机轨迹复用 `_rng_traj`，PCG64 种子固定。两侧用同一组参数。
    """
    rows = []

    def record(label, new, old):
        if new is Verdict.SAFE:
            assert old is Verdict.SAFE, (label, old, new)
        rows.append((label, old, new))

    def sep(label, a, b, margin):
        record(
            label,
            check_separation(a, b, margin).verdict,
            check_separation_pre(a, b, margin).verdict,
        )

    def deriv(label, traj, order, limit):
        record(
            label,
            check_derivative_bound(traj, order, limit).verdict,
            check_derivative_bound_pre(traj, order, limit).verdict,
        )

    gap = 1.0
    still_a = line((0, gap, 0), (0, 0, 0), 0.0, 1.0)
    still_b = line((0, 0, 0), (0, 0, 0), 0.0, 1.0)
    for k in (-100, -2, -1, -0.5, 0, 0.5, 1, 2, 100):
        sep(
            f"constant-gap k={k}",
            still_a,
            still_b,
            ConstantMargin(gap + k * DEFAULT_TOL),
        )
    for value0, rate in (
        (gap - 2 * DEFAULT_TOL, 0.0),
        (gap - DEFAULT_TOL, 0.4),
        (gap, 0.5),
        (0.5, 1.0),
    ):
        sep(
            f"constant-gap linear rate={rate}",
            still_a,
            still_b,
            LinearMargin(0.0, value0, rate),
        )

    cross_a = line((-1, 0, 0), (4, 0, 0), 0.0, 1.0)
    cross_b = line((0, 0, 0), (0, 0, 0), 0.0, 1.0)
    sep("separation-tol-band-midpoint", cross_a, cross_b, ConstantMargin(1.0))
    for k in (-2, -0.5, 0.5, 2):
        sep(
            f"separation-tol-band k={k}",
            cross_a,
            cross_b,
            ConstantMargin(1.0 + k * DEFAULT_TOL),
        )
    for margin in (0.5, 1.5, 2.0):
        sep(
            f"separation-tol-band margin={margin}",
            cross_a,
            cross_b,
            ConstantMargin(margin),
        )

    graze_a = line((-5, 1.0, 0), (1, 0, 0), 0.0, 10.0)
    graze_b = line((0, 0, 0), (0, 0, 0), 0.0, 10.0)
    for k in (-100, -2, -0.5, 0, 0.5, 2, 100):
        sep(
            f"grazing k={k}",
            graze_a,
            graze_b,
            ConstantMargin(1.0 + k * DEFAULT_TOL),
        )
    for margin in (0.5, 1.5, 2.0):
        sep(f"grazing margin={margin}", graze_a, graze_b, ConstantMargin(margin))

    bump = velocity_bump()
    for order, peak in ((1, 10.0), (2, 22.0), (3, 40.0)):
        for k in (-100, -2, -0.5, 0, 0.5, 2, 100):
            deriv(f"limit-eq order={order} k={k}", bump, order, peak + k * DEFAULT_TOL)
        for limit in (0.5 * peak, peak + 1.0):
            deriv(f"limit-eq order={order} limit={limit}", bump, order, limit)
    band_limit = float(np.linalg.norm(bump.evaluate(0.5625, 1)))
    deriv("derivative-tol-band-midpoint", bump, 1, band_limit)
    for k in (-2, -0.5, 0.5, 2):
        deriv(
            f"derivative-tol-band k={k}",
            bump,
            1,
            band_limit + k * DEFAULT_TOL,
        )
    for limit in (0.5 * band_limit, band_limit + 1.0):
        deriv(f"derivative-tol-band limit={limit}", bump, 1, limit)

    rng = np.random.Generator(np.random.PCG64(TOL_BAND_DIFF_SEED))
    for i in range(TOL_BAND_DIFF_PAIRS):
        a = _rng_traj(rng)
        b = _rng_traj(rng)
        for required in (0.2, 1.0, 2.5):
            sep(f"random-pair {i} margin={required}", a, b, ConstantMargin(required))
        sep(
            f"random-pair {i} linear",
            a,
            b,
            LinearMargin(a.t_start, 1.0, 0.25),
        )
    for i in range(TOL_BAND_DIFF_TRAJS):
        traj = _rng_traj(rng)
        for order in (1, 2, 3):
            for limit in (0.5, 8.0, 30.0):
                deriv(f"random-traj {i} order={order} limit={limit}", traj, order, limit)

    unknown_to_unsafe = [
        label for label, old, new in rows if old is Verdict.UNKNOWN and new is Verdict.UNSAFE
    ]
    nonsafe_to_safe = [
        label for label, old, new in rows if old is not Verdict.SAFE and new is Verdict.SAFE
    ]
    assert nonsafe_to_safe == []
    assert len(unknown_to_unsafe) >= 1
    assert "separation-tol-band-midpoint" in unknown_to_unsafe
    assert "derivative-tol-band-midpoint" in unknown_to_unsafe
