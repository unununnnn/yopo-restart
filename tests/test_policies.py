"""票 02：策略开关、上游等价策略过 G2 柱子、certified 不变、CPU 推理对金样。"""

import json
import subprocess
import sys
from hashlib import sha256
from pathlib import Path

import numpy as np
import pytest

from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import load_card

from conftest import ROOT

PILLAR = ROOT / "scenes/g2-pillar.json"
pytestmark = pytest.mark.upstream


def strip_wall(r):
    return {k: v for k, v in r.items() if k not in WALL_FIELDS}


def test_cpu_inference_matches_golden():
    from yopo_swarm_core import baseline as B

    from yopo_l0.net import load_model

    golden = np.load(ROOT / "core/tests/golden/minco_d5e1e60_cpu.npz")
    model = load_model()
    assert str(golden["weights_sha256"]) == model.weights_sha256 == B.WEIGHTS_SHA256
    assert str(golden["upstream_sha"]) == model.upstream_sha
    depth, obs = B.fixed_inputs(model.config)
    out = model.forward(depth, obs)
    tol = {"rtol": B.GOLDEN_RTOL, "atol": B.GOLDEN_ATOL}
    np.testing.assert_allclose(out.traj_params, golden["traj_params"], **tol)
    np.testing.assert_allclose(out.score, golden["score"], **tol)
    np.testing.assert_allclose(out.radius, golden["radius"], **tol)


def test_vendored_core_unchanged():
    # certified 走的 vendored 代码、测试和金样与 vendor 时逐字节一致
    for line in (ROOT / "core/VENDORED.sha256").read_text().splitlines():
        digest, rel = line.split(maxsplit=1)
        assert sha256((ROOT / "core" / rel).read_bytes()).hexdigest() == digest, rel


def test_l0_default_is_upstream_with_shadow_switch(tmp_path):
    # L0 默认 upstream + 影子盾开（开关就绪，记录实现留给 03）；可用 --no-shadow / --policy 覆盖
    p = subprocess.run(
        [sys.executable, "-m", "yopo_l0.run", "--scene", str(PILLAR), "--duration", "0.5", "--out", str(tmp_path / "a")],
        cwd=ROOT, capture_output=True, text=True, timeout=180,
    )
    assert p.returncode in (0, 1), p.stderr + p.stdout
    r = json.loads((tmp_path / "a" / "result.json").read_text())
    assert r["policy"] == "upstream" and r["shadow"] is True
    assert r["network"]["upstream_sha"].startswith("d5e1e60")
    hover = ROOT / "scenes/g2-hover.json"
    q = subprocess.run(
        [sys.executable, "-m", "yopo_l0.run", "--scene", str(hover), "--duration", "0.5",
         "--policy", "hover", "--no-shadow", "--out", str(tmp_path / "b")],
        cwd=ROOT, capture_output=True, text=True, timeout=120,
    )
    assert q.returncode == 0, q.stderr
    r2 = json.loads((tmp_path / "b" / "result.json").read_text())
    assert r2["policy"] == "hover" and r2["shadow"] is False and r2["network"] is None


def test_upstream_passes_g2_pillar_deterministically():
    card = load_card(PILLAR)
    a = run(card, RunConfig(seed=0, policy="upstream", record_planner_inputs=True))
    r = a.result
    assert r["outcome"] == "SUCCESS" and r["arrived"] == 1
    assert r["min_obstacle_distance_m"] > 0.35
    assert r["final_goal_error_m"] <= 0.4
    assert r["network"]["upstream_sha"].startswith("d5e1e60")
    b = run(card, RunConfig(seed=0, policy="upstream", record_planner_inputs=True))
    assert strip_wall(a.result) == strip_wall(b.result) and a.ticks == b.ticks
    assert a.planner_inputs == b.planner_inputs


def test_upstream_inputs_have_no_scene_geometry(make_card, g2_geometry):
    base = json.loads(PILLAR.read_text())
    keep = {k: base[k] for k in ("name", "drones", "task", "criteria", "camera", "rates_hz", "policy_params")}
    hidden = {**g2_geometry, "boxes": g2_geometry["boxes"] + [
        {"name": "hidden", "lo": [-2.9, -1.0, 0.0], "hi": [-2.6, 1.0, 2.0]}]}
    a = run(make_card(geometry=g2_geometry, **keep), RunConfig(policy="upstream", record_planner_inputs=True))
    b = run(make_card(geometry=hidden, **keep), RunConfig(policy="upstream", record_planner_inputs=True))
    assert a.planner_inputs == b.planner_inputs
    assert a.ticks == b.ticks


def test_certified_runs_vendored_gates_deterministically():
    card = load_card(PILLAR)
    cfg = RunConfig(seed=0, policy="certified", duration_s=2.5)
    a = run(card, cfg)
    codes = a.result["plan_codes"]
    assert codes.get("BOOTSTRAP") == 1  # 初始保持经 SafetyGate 批准
    assert sum(v for k, v in codes.items() if k not in ("BOOTSTRAP", "WAIT_SWITCH") and not k.startswith("cand:")) > 0
    assert a.result["outcome"] in ("TIMEOUT", "SUCCESS")  # 判据未冻结：只要求跑通、不碰撞
    b = run(card, cfg)
    assert strip_wall(a.result) == strip_wall(b.result) and a.ticks == b.ticks
