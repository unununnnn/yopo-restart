"""P1：推理开关不改权重。默认 inference_mode 与 no_grad 逐位相同。"""

import numpy as np
import pytest

from yopo_l0.infer_accel import accel_name, accel_threads, run_forward
from yopo_l0.net import load_model
from yopo_swarm_core.baseline import fixed_inputs


@pytest.fixture
def restore_threads():
    yield
    import torch

    from yopo_l0 import infer_accel

    torch.set_num_threads(1)
    infer_accel._THREADS_APPLIED = None


def test_accel_name_defaults_and_aliases(monkeypatch):
    monkeypatch.delenv("YOPO_INFER_ACCEL", raising=False)
    assert accel_name() == "infer"
    assert accel_threads() == 1
    monkeypatch.setenv("YOPO_INFER_ACCEL", "baseline")
    assert accel_name() == "baseline"
    assert accel_threads() == 1
    monkeypatch.setenv("YOPO_INFER_ACCEL", "off")
    assert accel_name() == "baseline"
    monkeypatch.setenv("YOPO_INFER_ACCEL", "threads")
    monkeypatch.delenv("YOPO_INFER_THREADS", raising=False)
    assert accel_threads() == 4
    monkeypatch.setenv("YOPO_INFER_THREADS", "2")
    assert accel_threads() == 2
    monkeypatch.setenv("YOPO_INFER_ACCEL", "nope")
    with pytest.raises(ValueError, match="YOPO_INFER_ACCEL"):
        accel_name()


def test_infer_mode_matches_baseline_bitwise(monkeypatch, restore_threads):
    model = load_model()
    depth, obs = fixed_inputs(model.config)
    monkeypatch.setenv("YOPO_INFER_ACCEL", "baseline")
    base = run_forward(model, depth.copy(), obs.copy())
    monkeypatch.setenv("YOPO_INFER_ACCEL", "infer")
    fast = run_forward(model, depth.copy(), obs.copy())
    assert np.array_equal(fast.traj_params, base.traj_params)
    assert np.array_equal(fast.score, base.score)
    assert np.array_equal(fast.radius, base.radius)


def test_threads_keep_the_same_argmax_within_golden_tolerance(monkeypatch, restore_threads):
    """单次前向：4 线程落在金样容差 rtol 1e-4 / atol 1e-5 内，且 argmax 不变。

    这不是闭环。六机林上多线程会换成另一条轨迹，见 docs/research/p1-inference-speed.md。
    """
    model = load_model()
    depth, obs = fixed_inputs(model.config)
    monkeypatch.setenv("YOPO_INFER_ACCEL", "baseline")
    base = run_forward(model, depth.copy(), obs.copy())
    monkeypatch.setenv("YOPO_INFER_ACCEL", "threads")
    monkeypatch.setenv("YOPO_INFER_THREADS", "4")
    alt = run_forward(model, depth.copy(), obs.copy())
    np.testing.assert_allclose(alt.traj_params, base.traj_params, rtol=1e-4, atol=1e-5)
    np.testing.assert_allclose(alt.score, base.score, rtol=1e-4, atol=1e-5)
    np.testing.assert_allclose(alt.radius, base.radius, rtol=1e-4, atol=1e-5)
    assert int(np.argmax(alt.score)) == int(np.argmax(base.score))


def test_ort_keeps_the_same_argmax_when_installed(monkeypatch, restore_threads):
    """单次前向。闭环是否还是同一条轨迹，以研究记录里的场景表为准。"""
    pytest.importorskip("onnxruntime")
    model = load_model()
    depth, obs = fixed_inputs(model.config)
    monkeypatch.setenv("YOPO_INFER_ACCEL", "baseline")
    base = run_forward(model, depth.copy(), obs.copy())
    monkeypatch.setenv("YOPO_INFER_ACCEL", "ort")
    monkeypatch.setenv("YOPO_INFER_THREADS", "4")
    alt = run_forward(model, depth.copy(), obs.copy())
    np.testing.assert_allclose(alt.traj_params, base.traj_params, rtol=1e-4, atol=1e-5)
    np.testing.assert_allclose(alt.score, base.score, rtol=1e-4, atol=1e-5)
    np.testing.assert_allclose(alt.radius, base.radius, rtol=1e-4, atol=1e-5)
    assert int(np.argmax(alt.score)) == int(np.argmax(base.score))
