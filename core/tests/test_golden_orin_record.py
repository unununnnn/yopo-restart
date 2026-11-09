"""Lock the committed CPU golden shapes and the recorded Orin FP32 compare.

Does not import torch and does not start a container. L0/L1 numbers are the
2026-10-24 infer-cu13 validate report in docs/ref/golden_gpu_orin.json.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from yopo_swarm_core.baseline import (
    GOLDEN_ATOL,
    GOLDEN_PATH,
    UPSTREAM_SHA,
    WEIGHTS_SHA256,
    candidate_count,
    fixed_inputs,
)
from yopo_swarm_core.candidates import UPSTREAM_DECODER

GPU_RECORD = Path(__file__).resolve().parents[2] / "docs" / "ref" / "golden_gpu_orin.json"


def test_cpu_golden_provenance_and_shapes():
    golden = np.load(GOLDEN_PATH)
    assert str(golden["upstream_sha"]) == UPSTREAM_SHA
    assert str(golden["weights_sha256"]) == WEIGHTS_SHA256
    assert str(golden["torch_version"]) == "2.4.1+cpu"
    assert str(golden["config_hash"]) == (
        "18de07db2c1482948b858ee32b64ddc5782d6c53a1e0ca2eb9dce5b7863281d8"
    )
    count = candidate_count({"horizon_num": 5, "vertical_num": 3})
    assert count == 15  # 5×3：只作断言值
    assert golden["traj_params"].shape == (1, count, 14)
    assert golden["score"].shape == (1, count)
    assert golden["radius"].shape == (1, count, 20)
    assert np.isfinite(golden["traj_params"]).all()
    assert np.isfinite(golden["score"]).all()
    assert np.isfinite(golden["radius"]).all()


def test_golden_obs_matches_inplace_normalize_of_fixed_input():
    """npz obs is the fixed state after normalize_obs, which writes through from_numpy."""
    golden = np.load(GOLDEN_PATH)
    _depth, raw = fixed_inputs({"horizon_num": 5, "vertical_num": 3, "downsample_factor": 32})
    assert np.array_equal(_depth, golden["depth"])
    assert not np.array_equal(raw, golden["obs"])
    expected = raw.copy()
    expected[:, 0:3] /= UPSTREAM_DECODER.vel_max
    expected[:, 3:6] /= UPSTREAM_DECODER.acc_max
    goal_norm = np.linalg.norm(expected[:, 6:9], axis=1, keepdims=True)
    expected[:, 6:9] /= np.maximum(goal_norm, UPSTREAM_DECODER.tail_radio_max)
    np.testing.assert_allclose(golden["obs"], expected, rtol=0, atol=1e-6)


def test_recorded_orin_fp32_within_cpu_golden_atol():
    report = json.loads(GPU_RECORD.read_text())
    assert report["torch"] == "2.12.0+cu132"
    for level in ("L0_cpu_fp32", "L1_cuda_fp32", "L2_trt_fp32"):
        row = report[level]
        assert row["status"] == "PASS"
        assert row["pass_tol"] is True
        assert row["argmax_equal"] is True
        for key in ("traj_params", "score", "radius"):
            assert row["max_abs"][key] <= GOLDEN_ATOL
    fp16 = report["L3_trt_fp16"]
    assert fp16["status"] == "PASS"
    assert fp16["pass_tol"] is True
    assert fp16["argmax_equal"] is True
