"""#6 与上游 YOPO-MINCO 的在线对照（pytest -m upstream）。"""

import numpy as np
import pytest

from yopo_swarm_core import baseline
from yopo_swarm_core.candidates import UPSTREAM_DECODER, DecoderParams, upstream_reference

pytestmark = pytest.mark.upstream


@pytest.fixture(scope="module")
def model():
    return baseline.load_verified_model(baseline.fetch_upstream())


def test_decoder_params_from_live_upstream_config(model):
    assert DecoderParams.from_config(model.config) == UPSTREAM_DECODER


def test_live_upstream_reference_matches_committed_decode_golden(model):
    from pathlib import Path

    g = np.load(Path(__file__).parent / "golden" / "minco_decode_d5e1e60.npz")
    ref = upstream_reference(model)
    for key in ("inner_pos_b", "tail_pva_b", "durations", "pos", "vel", "acc"):
        np.testing.assert_allclose(ref[key], g[key], rtol=1e-4, atol=1e-5)
    assert int(ref["upstream_action_id"]) == int(g["upstream_action_id"])
