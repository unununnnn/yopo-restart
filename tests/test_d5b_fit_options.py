"""票 D5b：训练选项（只学水平、逐窗口加权）。默认关闭时与 D5 训练逐位相同。"""

from __future__ import annotations

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from yopo_l0.student_fit import FitConfig, trajectory_loss, weighted_trajectory_loss, window_weights


def _heads(n=4, seed=0):
    g = torch.Generator().manual_seed(seed)
    mk = lambda *s: torch.randn(*s, generator=g)
    return mk(n, 8, 3), mk(n, 8, 3), mk(n, 8), mk(n, 8, 3), mk(n, 8, 3), mk(n, 8)


def test_defaults_are_off():
    c = FitConfig()
    assert c.planar is False and c.near_weight == 1.0 and c.shuffle_batches == 8


def test_unit_weights_match_unweighted_loss():
    p, v, y, ph, vh, yh = _heads()
    ref, ref_pos, _, _ = trajectory_loss(p, v, y, ph, vh, yh, weights=(1.0, 0.25, 0.05))
    got, got_pos = weighted_trajectory_loss(p, v, y, ph, vh, yh, torch.ones(4), weights=(1.0, 0.25, 0.05))
    assert float(got) == pytest.approx(float(ref), rel=1e-6)
    assert float(got_pos) == pytest.approx(float(ref_pos), rel=1e-6)


def test_planar_loss_ignores_z():
    p, v, y, ph, vh, yh = _heads()
    a, _ = weighted_trajectory_loss(p, v, y, ph, vh, yh, torch.ones(4), planar=True)
    ph2, vh2 = ph.clone(), vh.clone()
    ph2[..., 2] += 10.0
    vh2[..., 2] -= 10.0
    b, _ = weighted_trajectory_loss(p, v, y, ph2, vh2, yh, torch.ones(4), planar=True)
    assert float(a) == pytest.approx(float(b), rel=1e-7)


def test_sample_weights_emphasise_heavy_windows():
    p, v, y, ph, vh, yh = _heads()
    w = torch.tensor([1.0, 0.0, 0.0, 0.0])
    got, _ = weighted_trajectory_loss(p, v, y, ph, vh, yh, w)
    one, _, _, _ = trajectory_loss(p[:1], v[:1], y[:1], ph[:1], vh[:1], yh[:1])
    assert float(got) == pytest.approx(float(one), rel=1e-6)


class _B:
    def __init__(self, depth_m, valid, max_range=20.0):
        self.n = len(depth_m)
        self.max_range_m = max_range
        d = np.zeros((self.n, 2, 4, 4), np.float32)
        for i, (dm, ok) in enumerate(zip(depth_m, valid)):
            d[i, 0] = dm / max_range
            d[i, 1] = 1.0 if ok else 0.0
        self.depth = d


def test_window_weights_mark_near_obstacle_windows():
    b = _B([0.8, 5.0, 0.5], [True, True, False])
    w = window_weights(b, 2.0, FitConfig(near_weight=3.0, near_depth_m=1.2))
    np.testing.assert_allclose(w, [6.0, 2.0, 2.0])
    np.testing.assert_allclose(window_weights(b, 1.0, FitConfig()), [1.0, 1.0, 1.0])
