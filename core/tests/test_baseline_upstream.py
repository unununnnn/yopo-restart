"""#3 需要拉取上游源码与权重的部分（pytest -m upstream；缓存于 YOPO_SWARM_CACHE）。

金样：core/tests/golden/minco_d5e1e60_cpu.npz，由 `python -m yopo_swarm_core.baseline
--write-golden` 生成，生成方式见 docs/ref/golden.md。
"""

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from yopo_swarm_core import baseline

pytestmark = pytest.mark.upstream

GOLDEN = Path(__file__).parent / "golden" / "minco_d5e1e60_cpu.npz"


@pytest.fixture(scope="module")
def upstream_root():
    return baseline.fetch_upstream()


@pytest.fixture(scope="module")
def model(upstream_root):
    return baseline.load_verified_model(upstream_root)


def test_fetch_pins_exact_sha(upstream_root):
    head = subprocess.run(
        ["git", "-C", str(upstream_root), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    assert head == baseline.UPSTREAM_SHA


def test_model_reports_provenance(model):
    assert model.upstream_sha == baseline.UPSTREAM_SHA
    assert model.weights_sha256 == baseline.WEIGHTS_SHA256
    assert len(model.config_hash) == 64


def test_candidate_batch_shapes(model):
    depth, obs = baseline.fixed_inputs(model.config)
    out = model.forward(depth, obs)
    n = baseline.candidate_count(model.config)
    assert n == 15  # 5×3：只作断言值
    nr = int(model.config["radius_num"])
    assert out.traj_params.shape == (1, n, 14)
    assert out.score.shape == (1, n)
    assert out.radius.shape == (1, n, 2 * nr)
    assert np.all(np.isfinite(out.traj_params))
    assert np.all(out.score >= 0)  # softplus


def test_forward_matches_golden_on_cpu(model):
    golden = np.load(GOLDEN)
    assert str(golden["upstream_sha"]) == baseline.UPSTREAM_SHA
    assert str(golden["weights_sha256"]) == baseline.WEIGHTS_SHA256
    assert str(golden["config_hash"]) == model.config_hash
    depth, obs = baseline.fixed_inputs(model.config)
    np.testing.assert_array_equal(depth, golden["depth"])
    out = model.forward(depth, obs)
    tol = {"rtol": baseline.GOLDEN_RTOL, "atol": baseline.GOLDEN_ATOL}
    np.testing.assert_allclose(out.traj_params, golden["traj_params"], **tol)
    np.testing.assert_allclose(out.score, golden["score"], **tol)
    np.testing.assert_allclose(out.radius, golden["radius"], **tol)


def test_cli_prints_provenance_and_passes(upstream_root):
    src = str(Path(baseline.__file__).resolve().parents[1])
    env = {**os.environ, "PYTHONPATH": src + os.pathsep + os.environ.get("PYTHONPATH", "")}
    r = subprocess.run(
        [sys.executable, "-m", "yopo_swarm_core.baseline"],
        check=False,
        capture_output=True,
        text=True,
        timeout=600,
        env=env,
    )
    assert r.returncode == 0, r.stderr
    assert baseline.UPSTREAM_SHA in r.stdout
    assert baseline.WEIGHTS_SHA256 in r.stdout
    assert "config_hash" in r.stdout
    assert "golden: PASS" in r.stdout
