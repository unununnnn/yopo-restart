"""票 SH1：场景证据开关默认关。关时不加字段；开和关的轨迹逐位相同。"""

import json
import subprocess
import sys

import numpy as np

from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import Box, Ground
from yopo_l0.sensor import CameraConfig, DepthCamera
from yopo_l0.shadow import ENV_OCCUPIED, ENV_UNKNOWN, DroneShadow
from yopo_swarm_core.plan import quintic
from yopo_swarm_core.trajectory import PiecewisePolynomial

from conftest import G2_CARD, ROOT
from test_shadow import actions, strip_shadow_result, strip_shadow_ticks, strip_wall

PILLAR = (
    Box("pillar", (2.6, -0.4, 0.0), (3.2, 0.4, 2.8)),
    Ground(0.0),
)
ENV_KEYS = (
    "shadow_env_evidence",
    "shadow_env_views",
    "shadow_env_clearance_evals",
    "shadow_env_certificate_centers",
)


def _traj(p0, p1, duration=1.2):
    z = np.zeros(3)
    start = np.stack([np.asarray(p0, dtype=float), z, z])
    end = np.stack([np.asarray(p1, dtype=float), z, z])
    return PiecewisePolynomial(0.0, [quintic(start, end, duration)])


class _Wrap:
    def __init__(self, traj):
        self.traj = traj


class _State:
    def __init__(self, p):
        self.p = np.asarray(p, dtype=float)
        self.yaw = 0.0


def _frame(origin):
    cam = DepthCamera(
        CameraConfig(width=160, height=120, hfov_deg=80.0, vfov_deg=60.0, noise_std_m=0.0),
        PILLAR,
        np.random.default_rng(0),
    )
    return cam.capture(np.asarray(origin, dtype=float), 0.0, 0.2)


def _select(obstacles, origin, dest):
    shadow = DroneShadow("uav0", origin, dest, {}, scene_obstacles=obstacles)
    shadow.on_selection(_Wrap(_traj(origin, dest)), _frame(origin), _State(origin), 0.2)
    return shadow.selection


def test_switch_defaults_off_and_adds_no_fields():
    assert RunConfig().shadow_env_evidence is False
    card_path = G2_CARD
    from yopo_l0.scene import load_card

    card = load_card(card_path)
    kw = dict(seed=0, policy="hover", duration_s=0.3, shadow=True)
    plain = run(card, RunConfig(**kw))
    explicit = run(card, RunConfig(shadow_env_evidence=False, **kw))
    assert "shadow_env_evidence" not in plain.result
    assert "shadow_env_evidence" not in explicit.result
    for key in ENV_KEYS:
        assert key not in plain.result
        assert key not in explicit.result
    assert strip_wall(plain.result) == strip_wall(explicit.result)
    assert plain.ticks == explicit.ticks


def test_switch_does_not_change_trajectory():
    from yopo_l0.scene import load_card

    card = load_card(G2_CARD)
    kw = dict(seed=1, policy="hover", record_planner_inputs=True, duration_s=0.4, shadow=True)
    off = run(card, RunConfig(shadow_env_evidence=False, **kw))
    on = run(card, RunConfig(shadow_env_evidence=True, **kw))
    assert on.result["shadow_env_evidence"] is True
    assert on.result["upstream_deviation"] is True
    assert on.result["shadow_env_views"] >= 1
    assert on.result["shadow_env_clearance_evals"] > 0
    assert actions(off.ticks) == actions(on.ticks)
    assert off.planner_inputs == on.planner_inputs
    assert strip_shadow_ticks(off.ticks) == strip_shadow_ticks(on.ticks)
    assert off.result["outcome"] == on.result["outcome"]
    assert off.result["min_obstacle_distance_m"] == on.result["min_obstacle_distance_m"]
    dropped = set(ENV_KEYS) | {"upstream_deviation"}
    kept = {k: v for k, v in on.result.items() if k not in dropped}
    assert strip_shadow_result(off.result) == strip_shadow_result(kept)


def test_scene_evidence_frees_clear_sweep_and_keeps_occupied():
    clear = _select(PILLAR, (0.0, 2.2, 1.5), (1.6, 2.2, 1.5))
    hit = _select(PILLAR, (0.0, 0.0, 1.5), (5.0, 0.0, 1.5))
    blind_clear = _select(None, (0.0, 2.2, 1.5), (1.6, 2.2, 1.5))
    blind_hit = _select(None, (0.0, 0.0, 1.5), (5.0, 0.0, 1.5))
    # 前视深度能把穿柱扫掠标成 OCCUPIED，但给不出旁边那条的 FREE，所以仍是 UNKNOWN。
    assert ENV_UNKNOWN in blind_clear
    assert ENV_OCCUPIED in blind_hit
    assert ENV_UNKNOWN not in clear
    assert ENV_OCCUPIED not in clear
    assert ENV_OCCUPIED in hit


def test_cli_switch_off_omits_field_and_on_matches_path(tmp_path):
    common = [
        sys.executable, "-m", "yopo_l0.run", "--scene", str(G2_CARD), "--seed", "0",
        "--policy", "hover", "--duration", "0.2", "--shadow", "on",
    ]
    off_dir, on_dir = tmp_path / "off", tmp_path / "on"
    p_off = subprocess.run(
        [*common, "--out", str(off_dir)], cwd=ROOT, capture_output=True, text=True, timeout=120,
    )
    p_on = subprocess.run(
        [*common, "--shadow-env-evidence", "--out", str(on_dir)],
        cwd=ROOT, capture_output=True, text=True, timeout=120,
    )
    assert p_off.returncode == 0, p_off.stderr
    assert p_on.returncode == 0, p_on.stderr
    r_off = json.loads((off_dir / "result.json").read_text())
    r_on = json.loads((on_dir / "result.json").read_text())
    assert "shadow_env_evidence" not in r_off
    assert r_on["shadow_env_evidence"] is True
    ticks_off = [json.loads(line) for line in (off_dir / "ticks.jsonl").read_text().splitlines()]
    ticks_on = [json.loads(line) for line in (on_dir / "ticks.jsonl").read_text().splitlines()]
    assert actions(ticks_off) == actions(ticks_on)
