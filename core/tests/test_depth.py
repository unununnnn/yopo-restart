import json
from pathlib import Path

import numpy as np
import pytest

from yopo_swarm_core.depth import T_BODY_OPTICAL, DepthSample, decode_image, network_depth
from yopo_swarm_core.freespace import FreeSpaceConfig, SpaceVerdict

DATA = Path(__file__).parent / "data"
SAMPLE = Path(__file__).resolve().parents[2] / "docs/ref/px4-depth/single-camera.npz"


def test_camera_extrinsics_match_resolved_sdf_merge_frame_graph():
    pose = json.loads((SAMPLE.parent / "resolved-camera-pose.json").read_text())
    assert np.allclose(T_BODY_OPTICAL[:3, 3], pose["depth_sensor_in_base_link_flu"][:3], atol=1e-12)
    assert pose["base_link_in_model"][2] == 0.24


@pytest.mark.parametrize("case", range(5))
def test_network_input_matches_upstream_opencv_reference(case):
    with np.load(DATA / "depth_opencv_4_10.npz") as z:
        assert np.array_equal(network_depth(z[f"raw_{case}"]), z[f"expected_{case}"])


def test_raw_sample_is_readable_by_freespace_without_ros():
    sample = DepthSample.load(SAMPLE)
    frame = sample.evidence(FreeSpaceConfig())
    assert frame.depth.shape == (480, 640)
    assert frame.stamp == sample.stamp_ns / 1e9
    assert np.array_equal(
        frame.valid, np.isfinite(sample.raw_m) & (sample.raw_m >= 0.3) & (sample.raw_m <= 30)
    )
    # 不把无回波视为 20 m 自由空间。
    invalid = ~np.isfinite(sample.raw_m)
    assert np.isnan(frame.depth[invalid]).all()
    assert frame.classify_ball(frame.o, 0.2, 0.3) is SpaceVerdict.UNKNOWN
    raw = sample.raw_m.copy()
    network_depth(raw)
    assert np.array_equal(raw, sample.raw_m, equal_nan=True)


def test_image_padding_endianness_and_mm_units():
    a = np.array([[1000, 2000, 999], [0, 500, 999]], dtype=">u2")
    actual = decode_image(a.tobytes(), 2, 2, 6, "16UC1", True)
    assert np.array_equal(actual, [[1, 2], [0, 0.5]])
    with pytest.raises(ValueError):
        decode_image(b"", 2, 2, 8, "rgb8")


def test_clipping_is_network_only():
    raw = np.full((96, 160), 25.0, np.float32)
    assert np.all(network_depth(raw) == 1)
    assert np.all(raw == 25)
