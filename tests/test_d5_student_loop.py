"""票 D5：学生接到 L0 闭环，默认不选；不选时飞行路径不跑网络。"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from yopo_l0.distill import convert_run, is_holdout_seed, parse_run_id, split_runs_by_seed
from yopo_l0.policy import POLICIES, PolicyContext, StudentPolicy
from yopo_l0.runner import RunConfig, WALL_FIELDS, run
from yopo_l0.scene import load_card
from yopo_l0.student import (
    StudentConfig,
    checkpoint_sha256,
    depth_from_hw,
    load_checkpoint,
    pack_ego_state,
    resize_depth,
    save_checkpoint,
    train_smoke,
)

from conftest import ROOT

FIXTURE = ROOT / "tests/fixtures/d3/d014-n1-headon-s0"
PILLAR = ROOT / "scenes/g2-pillar.json"
HOVER = ROOT / "scenes/g2-hover.json"


def strip_wall(r):
    return {k: v for k, v in r.items() if k not in WALL_FIELDS}


def test_policies_default_is_still_upstream():
    assert "student" in POLICIES
    assert set(POLICIES) >= {"hover", "upstream", "certified", "student"}
    assert RunConfig().policy == "upstream"


def test_import_policy_does_not_import_torch():
    code = (
        "import sys\n"
        "from yopo_l0.policy import POLICIES\n"
        "from yopo_l0.runner import RunConfig\n"
        "assert 'student' in POLICIES\n"
        "assert RunConfig().policy == 'upstream'\n"
        "assert 'torch' not in sys.modules\n"
        "assert 'yopo_l0.student' not in sys.modules\n"
    )
    p = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, timeout=60)
    assert p.returncode == 0, p.stdout + p.stderr


def test_upstream_run_does_not_import_student_or_torch():
    code = (
        "import sys\n"
        "from yopo_l0.runner import RunConfig, run\n"
        "from yopo_l0.scene import load_card\n"
        "run(load_card('scenes/g2-hover.json'), RunConfig(policy='upstream', duration_s=0.2, shadow=False, seed=0))\n"
        "assert 'yopo_l0.student' not in sys.modules\n"
    )
    p = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, timeout=120)
    assert p.returncode == 0, p.stdout + p.stderr


def test_split_by_seed_does_not_leak_the_same_run():
    ids = [
        "d000-n1-headon-s0",
        "d000-n1-headon-s1",
        "d000-n1-headon-s5",
        "d014-n3-parallel-s12",
        "d014-n1-headon-s20",
    ]
    train, hold = split_runs_by_seed(ids)
    assert set(train).isdisjoint(set(hold))
    assert set(train) | set(hold) == set(ids)
    assert "d000-n1-headon-s0" in hold
    assert "d000-n1-headon-s5" in hold
    assert "d000-n1-headon-s1" in train
    assert "d014-n3-parallel-s12" in train
    assert "d014-n1-headon-s20" in hold
    info = parse_run_id("d014-n1-headon-s20")
    assert info["density_per_m2"] == pytest.approx(0.14)
    assert info["n_drones"] == 1
    assert info["pattern"] == "headon"
    assert info["seed"] == 20
    assert info["scene_key"] == "d014-n1-headon"
    assert is_holdout_seed(0) and not is_holdout_seed(1)


def test_resize_depth_block_average_and_nans():
    depth = np.arange(16, dtype=np.float32).reshape(4, 4)
    out = resize_depth(depth, 2, 2)
    assert out.shape == (2, 2)
    np.testing.assert_allclose(out[0, 0], np.mean([0, 1, 4, 5]))
    np.testing.assert_allclose(out[1, 1], np.mean([10, 11, 14, 15]))
    same = np.ones((24, 40), dtype=np.float32)
    got = resize_depth(same, 24, 40)
    np.testing.assert_array_equal(got, same)
    broken = depth.copy()
    broken[0, 0] = np.nan
    out2 = resize_depth(broken, 2, 2)
    np.testing.assert_allclose(out2[0, 0], np.mean([1, 4, 5]))


def test_pack_ego_state_matches_body_frame_formula():
    p = np.array([1.0, 2.0, 1.5])
    v = np.array([3.0, 0.0, 0.0])
    a = np.array([0.0, 1.0, 0.0])
    goal = np.array([5.0, 2.0, 1.5])
    yaw = np.pi / 2
    state = pack_ego_state(p, v, a, goal, yaw)
    assert state.shape == (9,)
    # 偏航 π/2：机体系 x 是世界 y，机体系 y 是世界 -x。
    np.testing.assert_allclose(state[0:3], [0.0, -3.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(state[3:6], [1.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(state[6:9], [0.0, -4.0, 0.0], atol=1e-6)


def test_depth_from_hw_matches_student_shape():
    raw = np.full((96, 160), 10.0, dtype=np.float32)
    raw[0, 0] = np.nan
    packed = depth_from_hw(raw, 20.0)
    assert packed.shape == (2, 24, 40)
    assert packed.dtype == np.float32
    np.testing.assert_allclose(packed[0, 1:, :], 10.0 / 20.0)
    assert packed[1, 1, 1] == 1.0


def test_checkpoint_roundtrip_and_sha(tmp_path):
    batch = convert_run(FIXTURE, max_windows=4)
    report = train_smoke(
        batch.depth,
        batch.state,
        batch.position_body,
        batch.velocity_body,
        batch.yaw_rel,
        batch.neighbor_rel,
        batch.neighbor_mask,
        enabled=True,
        steps=40,
        seed=0,
        scenario_id=batch.scenario_id,
    )
    assert report.n_windows == 4
    from yopo_l0.student import MultiHeadStudent

    # train_smoke 不返回模型。另存一个刚训过的网络：再跑很短会改变权重。
    # 这里只验证保存格式，用打开的空网络即可。
    model = MultiHeadStudent(StudentConfig(enabled=True))
    path = tmp_path / "student.pt"
    _, sha = save_checkpoint(model, path, {"note": "test"})
    assert sha == checkpoint_sha256(path)
    loaded, meta = load_checkpoint(path, enabled=True)
    assert loaded.cfg.enabled is True
    assert meta["weights_sha256"] == sha
    for a, b in zip(model._module().state_dict().values(), loaded._module().state_dict().values(), strict=True):
        np.testing.assert_allclose(a.detach().cpu().numpy(), b.detach().cpu().numpy())


def test_student_policy_refuses_without_weights():
    ctx = PolicyContext("uav0", 0.01, {})
    with pytest.raises(ValueError, match="YOPO_STUDENT_WEIGHTS"):
        StudentPolicy(ctx)


def test_student_policy_can_plan_a_short_run(tmp_path, monkeypatch):
    batch = convert_run(FIXTURE, max_windows=4)
    report = train_smoke(
        batch.depth,
        batch.state,
        batch.position_body,
        batch.velocity_body,
        batch.yaw_rel,
        batch.neighbor_rel,
        batch.neighbor_mask,
        enabled=True,
        steps=80,
        seed=0,
        scenario_id=batch.scenario_id,
    )
    from yopo_l0.student import MultiHeadStudent

    # 用冒烟同款监督再存一份可加载权重：直接把 fixture 上训过的网络存下来。
    # train_smoke 内部的模型不对外。这里再训一个很短的，只为证明闭环能走。
    assert report.n_windows == 4
    model = MultiHeadStudent(StudentConfig(enabled=True))
    import torch

    torch.manual_seed(0)
    net = model._module()
    opt = torch.optim.Adam(net.parameters(), lr=1e-2)
    depth = torch.from_numpy(batch.depth)
    state = torch.from_numpy(batch.state)
    pos = torch.from_numpy(batch.position_body)
    vel = torch.from_numpy(batch.velocity_body)
    yaw = torch.from_numpy(batch.yaw_rel)
    neigh = torch.from_numpy(batch.neighbor_rel)
    mask = torch.from_numpy(batch.neighbor_mask)
    from yopo_l0.student import trajectory_loss

    for _ in range(80):
        opt.zero_grad(set_to_none=True)
        ph, vh, yh = net(depth, state, neigh, mask)
        total, _, _, _ = trajectory_loss(pos, vel, yaw, ph, vh, yh)
        total.backward()
        opt.step()
    path, sha = save_checkpoint(model, tmp_path / "student.pt")
    monkeypatch.setenv("YOPO_STUDENT_WEIGHTS", str(path))
    card = load_card(PILLAR)
    a = run(card, RunConfig(seed=0, policy="student", duration_s=0.5, shadow=False))
    assert a.result["policy"] == "student"
    assert a.result["network"]["name"] == "student"
    assert a.result["network"]["weights_sha256"] == sha
    assert a.result["student_infer"]["n"] >= 1
    assert a.result["outcome"] in {"SUCCESS", "TIMEOUT", "COLLISION"}
    b = run(card, RunConfig(seed=0, policy="student", duration_s=0.5, shadow=False))
    ra, rb = strip_wall(a.result), strip_wall(b.result)
    ra.pop('student_infer', None)
    rb.pop('student_infer', None)
    assert ra == rb
    assert a.ticks == b.ticks


def test_eval_scenes_exist_and_are_not_in_suite():
    suite = (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    for rel in (
        "scenes/g2-pillar.json",
        "scenes/swap-3-open.json",
        "scenes/open-3drone.json",
        "scenes/ego-density-014-solo.json",
    ):
        assert (ROOT / rel).is_file()
    assert "swap-3-open" not in suite
    assert "ego-density-014-solo" not in suite
    assert "d5-student" not in suite
