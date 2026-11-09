"""Y6（#39）训练薄壳：PLY 替身、设备单例、合成数据、对上游 d5e1e60 的损失/权重金样。"""

import importlib.util
import struct
import zlib

import numpy as np
import pytest

from yopo_swarm_core import train_golden, train_shim, train_synth

TRAIN_DEPS = ("cv2", "sklearn", "rich", "matplotlib", "tensorboard")
_missing = [m for m in TRAIN_DEPS if importlib.util.find_spec(m) is None]
needs_train_deps = pytest.mark.skipif(bool(_missing), reason=f"训练依赖缺失: {_missing}")


# --- PLY 替身 ---------------------------------------------------------------------


def test_read_ply_binary_roundtrip(tmp_path):
    pts = np.array([[0.0, 1.5, -2.25], [3.0, 4.0, 5.0]], np.float32)
    train_synth.write_ply(tmp_path / "a.ply", pts)
    got = train_shim.read_ply_points(tmp_path / "a.ply")
    assert got.dtype == np.float64
    np.testing.assert_array_equal(got, pts.astype(np.float64))


def test_read_ply_ascii_with_extra_props(tmp_path):
    (tmp_path / "b.ply").write_text(
        "ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\n"
        "property float z\nproperty uchar red\nelement face 0\n"
        "property list uchar int vertex_indices\nend_header\n1 2 3 255\n-1 0.5 7 0\n"
    )
    got = train_shim.read_ply_points(tmp_path / "b.ply")
    np.testing.assert_array_equal(got, [[1, 2, 3], [-1, 0.5, 7]])


def test_open3d_stub_matches_interface(tmp_path):
    pts = np.array([[0, 0, 0], [1, -2, 3]], np.float32)
    train_synth.write_ply(tmp_path / "c.ply", pts)
    stub = train_shim.make_open3d_stub()
    pc = stub.io.read_point_cloud(str(tmp_path / "c.ply"))
    np.testing.assert_array_equal(np.asarray(pc.points), pts)
    np.testing.assert_array_equal(pc.get_min_bound(), [0, -2, 0])
    np.testing.assert_array_equal(pc.get_max_bound(), [1, 0, 3])


def test_stub_agrees_with_real_open3d(tmp_path):
    o3d = pytest.importorskip("open3d")
    if getattr(o3d, "__yopo_swarm_stub__", False):
        pytest.skip("open3d 已是替身")
    train_synth.write_ply(tmp_path / "d.ply", train_synth.forest_points(train_synth.forest(0)))
    real = o3d.io.read_point_cloud(str(tmp_path / "d.ply"))
    stub = train_shim.make_open3d_stub().io.read_point_cloud(str(tmp_path / "d.ply"))
    np.testing.assert_array_equal(np.asarray(real.points), stub.points)
    np.testing.assert_array_equal(real.get_min_bound(), stub.get_min_bound())
    np.testing.assert_array_equal(real.get_max_bound(), stub.get_max_bound())


# --- 设备单例 ---------------------------------------------------------------------


def test_device_singleton_rejects_switch(monkeypatch):
    pytest.importorskip("torch")
    monkeypatch.setattr(train_shim, "_DEVICE", None)
    assert train_shim.select_device("cpu") == "cpu"
    assert train_shim.select_device() == "cpu"
    with pytest.raises(RuntimeError, match="one device per process"):
        train_shim.select_device("cuda")


def test_pinned_device_masks_cuda_and_restores(monkeypatch):
    torch = pytest.importorskip("torch")
    monkeypatch.setattr(train_shim, "_DEVICE", None)
    real = torch.cuda.is_available
    with train_shim.pinned_device("cpu") as dev:
        assert dev == "cpu"
        assert torch.cuda.is_available() is False
    assert torch.cuda.is_available is real


# --- 合成数据 ---------------------------------------------------------------------


def _png16_decode(path):
    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos, idat, w, h = 8, b"", None, None
    while pos < len(data):
        (n,) = struct.unpack(">I", data[pos : pos + 4])
        tag, body = data[pos + 4 : pos + 8], data[pos + 8 : pos + 8 + n]
        if tag == b"IHDR":
            w, h, depth, ctype = struct.unpack(">IIBB", body[:10])
            assert (depth, ctype) == (16, 0)
        elif tag == b"IDAT":
            idat += body
        pos += 12 + n
    raw = zlib.decompress(idat)
    rows = [raw[r * (2 * w + 1) + 1 : (r + 1) * (2 * w + 1)] for r in range(h)]
    return np.frombuffer(b"".join(rows), ">u2").reshape(h, w)


def test_synthetic_dataset_layout_and_determinism(tmp_path):
    a = train_synth.write_dataset(tmp_path / "a", maps=2, frames=3, width=40, height=24)
    b = train_synth.write_dataset(tmp_path / "b", maps=2, frames=3, width=40, height=24)
    files = sorted(p.relative_to(a) for p in a.rglob("*") if p.is_file())
    assert [str(p) for p in files] == [
        "0/img_0.png", "0/img_1.png", "0/img_2.png", "1/img_0.png", "1/img_1.png",
        "1/img_2.png", "pointcloud-0.ply", "pointcloud-1.ply", "pose-0.csv", "pose-1.csv",
    ]  # fmt: skip
    for p in files:
        assert (a / p).read_bytes() == (b / p).read_bytes()
    poses = np.loadtxt(a / "pose-0.csv", delimiter=",", skiprows=1)
    assert poses.shape == (3, 7)
    np.testing.assert_allclose(np.linalg.norm(poses[:, 3:], axis=1), 1.0, atol=1e-8)
    img = _png16_decode(a / "0" / "img_0.png")
    assert img.shape == (24, 40)
    assert 0 < img.min() and img.max() <= 65535


def test_render_depth_hits_cylinder_in_front():
    f = train_synth.Forest(np.array([[5.0, 0.0]]), 0.5, 6.0, (0, 10, -5, 5))
    d = train_synth.render_depth(f, np.array([0.0, 0.0, 2.0]), 0.0, 41, 21)
    assert d[10, 20] == pytest.approx(4.5, abs=1e-6)  # 光轴正中：圆柱前表面
    assert d[0, 0] == pytest.approx(train_synth.DEPTH_MAX)  # 左上角朝天：无命中


def test_png16_matches_cv2(tmp_path):
    cv2 = pytest.importorskip("cv2")
    img = (np.arange(24 * 40).reshape(24, 40) * 61 % 65536).astype(np.uint16)
    train_synth.write_png16(tmp_path / "x.png", img)
    np.testing.assert_array_equal(cv2.imread(str(tmp_path / "x.png"), -1), img)


# --- 对上游金样 -------------------------------------------------------------------


@pytest.fixture(scope="module")
def golden():
    return train_golden.load_golden()


@pytest.fixture(scope="module")
def fixture_ds(tmp_path_factory):
    return train_golden.write_fixture_dataset(tmp_path_factory.mktemp("y6ds"))


def _inputs(g):
    return {k: v for k, v in g.items() if k.startswith("in/")}


def test_two_step_bound_off_reference_follows_adamw(monkeypatch):
    # 参考环境：一律 1e-5 验收门。其他平台：AdamW 每步 |Δw| ≤ lr（偏置校正后 m̂/√v̂ ≤ 1.0013），
    # 近零梯度在两平台上符号可以相反 → 参数差 ≤ 2·步数·lr；BN 统计量随特征漂移，按其移动量的 1%。
    monkeypatch.setattr(train_golden, "reference_env", lambda: True)
    assert train_golden.two_step_bound("conv.weight", 0.3) == 1e-5
    assert train_golden.two_step_bound("bn.running_var", 0.3) == 1e-5
    monkeypatch.setattr(train_golden, "reference_env", lambda: False)
    adam = 2 * 2 * 1.5e-4 * 1.01
    assert train_golden.two_step_bound("conv.weight", 0.3) == pytest.approx(adam)
    assert train_golden.two_step_bound("bn.bias", 3e-4) == pytest.approx(adam)
    assert train_golden.two_step_bound("bn.running_mean", 0.25) == pytest.approx(2.5e-3)
    assert train_golden.two_step_bound("bn.running_var", 0.2) == pytest.approx(2e-3)


def test_acceptance_gate_is_reference_env_only(monkeypatch):
    # 验收门（1e-6 / 1e-5）只在金样生成环境里生效；其他平台用宽一档的诊断界
    assert train_golden.REFERENCE_ENV == ("x86_64", "2.4.1")
    monkeypatch.setattr(train_golden, "reference_env", lambda: True)
    assert (train_golden.loss_rtol(), train_golden.weight_atol()) == (1e-6, 1e-5)
    monkeypatch.setattr(train_golden, "reference_env", lambda: False)
    assert (train_golden.loss_rtol(), train_golden.weight_atol()) == (1e-4, 1e-4)


@pytest.mark.upstream
def test_loss_terms_match_upstream_golden(golden, fixture_ds):
    pytest.importorskip("torch")
    pytest.importorskip("scipy")
    got = train_golden.loss_terms(_inputs(golden), fixture_ds)
    keys = [f"cost/{k}" for k in train_golden.COST_KEYS] + ["aux/radius_dist"]
    errs = train_golden.relative_errors(got, golden, keys)
    assert max(errs.values()) <= train_golden.loss_rtol(), errs
    np.testing.assert_array_equal(got["aux/collided"], golden["aux/collided"])
    assert golden["aux/collided"].any() and not golden["aux/collided"].all()  # 金样两类都有


@pytest.mark.upstream
def test_float32_rounding_noise_explains_the_gate_split(golden, fixture_ds):
    # 根因回归：1 ulp 输入扰动的影响已超过 1e-6（只有逐位相同的内核能过严格门），
    # 但比跨平台诊断界小 5 倍以上（界不是随手放宽的）。
    pytest.importorskip("torch")
    pytest.importorskip("scipy")
    noise = max(train_golden.ulp_sensitivity(_inputs(golden), fixture_ds).values())
    # x86 实测 1.13e-5，Orin aarch64 2.08e-5：诊断界 1e-4 ≥ 4× 噪声
    assert train_golden.LOSS_RTOL < noise < train_golden.CROSS_PLATFORM_LOSS_RTOL / 4, noise


@pytest.mark.upstream
@needs_train_deps
def test_trainer_terms_match_upstream_golden(golden, fixture_ds):
    terms = {k: golden[k] for k in golden if k.startswith(("cost/", "aux/"))}
    got = train_golden.trainer_terms(_inputs(golden), terms, fixture_ds)
    keys = [f"trainer/{k}" for k in train_golden.TRAINER_KEYS]
    errs = train_golden.relative_errors(got, golden, keys)
    assert max(errs.values()) <= train_golden.loss_rtol(), errs


@pytest.mark.upstream
def test_golden_pins_cpu_on_cuda_host(golden, fixture_ds, monkeypatch):
    # Orin 回归：有 CUDA 时 configure() 不能先按缺省把单例钉到 cuda，金样必须整程 CPU
    torch = pytest.importorskip("torch")
    pytest.importorskip("scipy")
    monkeypatch.setattr(train_shim, "_DEVICE", None)
    monkeypatch.setattr(torch.cuda, "is_available", lambda: True)
    train_golden.loss_terms(_inputs(golden), fixture_ds)
    assert train_shim._DEVICE == "cpu"


@pytest.mark.upstream
@pytest.mark.slow
@needs_train_deps
def test_two_training_steps_match_upstream_cpu(golden, fixture_ds):
    init, two = train_golden.two_step_weights(fixture_ds)
    names = sorted(k[3:] for k in golden if k.startswith("w2/"))
    assert sorted(two) == names
    init_err = max(float(np.max(np.abs(init[n] - golden[f"w0/{n}"]))) for n in names)
    moved = {n: float(np.max(np.abs(golden[f"w2/{n}"] - golden[f"w0/{n}"]))) for n in names}
    over = {}
    for n in names:
        err = float(np.max(np.abs(two[n] - golden[f"w2/{n}"])))
        if err > train_golden.two_step_bound(n, moved[n]):
            over[n] = (err, moved[n])
    assert init_err <= train_golden.weight_atol(), init_err
    assert not over, over
    assert max(moved.values()) > 10 * train_golden.WEIGHT_ATOL  # 2 步确实改了权重，比较有分辨力
