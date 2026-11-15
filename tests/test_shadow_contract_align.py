"""票 SH3：契约码修正开关默认关。关时不加字段；打开只改影子记录，不改批准动作。"""

import sys

import numpy as np

from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import Box, Ground, load_card
from yopo_l0.sensor import CameraConfig, DepthCamera
from yopo_l0.shadow import (
    ENV_OCCUPIED,
    TRACKING_BOUND,
    DroneShadow,
)
from yopo_swarm_core.plan import quintic
from yopo_swarm_core.trajectory import PiecewisePolynomial

from conftest import G2_CARD
from test_shadow import actions, strip_shadow_result, strip_shadow_ticks, strip_wall
from test_shadow_env_evidence import ENV_KEYS, PILLAR, _State, _Wrap, _frame, _traj


def _pva(p, v=None, a=None):
    return np.stack([
        np.asarray(p, dtype=float),
        np.zeros(3) if v is None else np.asarray(v, dtype=float),
        np.zeros(3) if a is None else np.asarray(a, dtype=float),
    ])


def _poly(t0, head, end, duration):
    return PiecewisePolynomial(float(t0), [quintic(np.asarray(head, dtype=float), np.asarray(end, dtype=float), duration)])


class _Tick:
    def __init__(self, p, t=1.0):
        self.p = np.asarray(p, dtype=float)
        self.t = float(t)
        self.yaw = 0.0


def _codes(align, traj, now=0.2, origin=None, dest=None, obstacles=None, actual=None):
    origin = np.zeros(3) if origin is None else np.asarray(origin, dtype=float)
    dest = np.array([5.0, 0.0, 0.0]) if dest is None else np.asarray(dest, dtype=float)
    shadow = DroneShadow("uav0", origin, dest, {}, scene_obstacles=obstacles, contract_align=align)
    start = np.asarray(traj.evaluate(traj.t_start), dtype=float)
    shadow.on_selection(_Wrap(traj), None, _State(start), now)
    pos = start if actual is None else np.asarray(actual, dtype=float)
    return shadow.on_tick(_Tick(pos, now), start, now)


def test_switch_defaults_off_and_adds_no_fields():
    assert RunConfig().shadow_contract_align is False
    card = load_card(G2_CARD)
    kw = dict(seed=0, policy="hover", duration_s=0.3, shadow=True)
    plain = run(card, RunConfig(**kw))
    explicit = run(card, RunConfig(shadow_contract_align=False, **kw))
    assert "shadow_contract_align" not in plain.result
    assert "shadow_contract_align" not in explicit.result
    assert strip_wall(plain.result) == strip_wall(explicit.result)
    assert plain.ticks == explicit.ticks


def test_switch_does_not_change_trajectory_and_tags_deviation():
    card = load_card(G2_CARD)
    kw = dict(seed=1, policy="hover", record_planner_inputs=True, duration_s=0.4, shadow=True)
    off = run(card, RunConfig(shadow_contract_align=False, **kw))
    on = run(card, RunConfig(shadow_contract_align=True, **kw))
    assert on.result["shadow_contract_align"] is True
    assert on.result["upstream_deviation"] is True
    assert "shadow_contract_align" not in off.result
    assert actions(off.ticks) == actions(on.ticks)
    assert off.planner_inputs == on.planner_inputs
    assert strip_shadow_ticks(off.ticks) == strip_shadow_ticks(on.ticks)
    assert off.result["outcome"] == on.result["outcome"]
    assert off.result["min_obstacle_distance_m"] == on.result["min_obstacle_distance_m"]
    dropped = {"shadow_contract_align", "upstream_deviation"}
    kept = {k: v for k, v in on.result.items() if k not in dropped}
    assert strip_shadow_result(off.result) == strip_shadow_result(kept)
    # 悬停在 now 复查，认证门会打 TIME:SWITCH；对齐后不再把这条算进 would_reject。
    assert off.result["shadow_reject_histogram"].get("TIME:SWITCH_NOT_IN_FUTURE", 0) > 0
    assert on.result["shadow_reject_histogram"].get("TIME:SWITCH_NOT_IN_FUTURE", 0) == 0


def test_time_switch_and_terminal_and_head_are_false_positives():
    hover_hold = _poly(0.0, _pva([0, 0, 0]), _pva([0, 0, 0]), 0.5)
    off = _codes(False, hover_hold, now=0.0)
    on = _codes(True, hover_hold, now=0.0)
    assert "TIME:SWITCH_NOT_IN_FUTURE" in off
    assert "TIME:SWITCH_NOT_IN_FUTURE" not in on

    moving = _poly(0.0, _pva([0, 0, 0]), _pva([3, 0, 0], v=[1, 0, 0]), 3.0)
    off = _codes(False, moving, now=0.2)
    on = _codes(True, moving, now=0.2)
    assert "TRANSITION:TERMINAL_NOT_HOLDABLE" in off
    assert "TRANSITION:TERMINAL_NOT_HOLDABLE" not in on

    mid = _poly(1.0, _pva([2, 0, 0], v=[1, 0, 0]), _pva([6, 0, 0]), 8.0)
    off = _codes(False, mid, now=1.5, origin=np.zeros(3))
    on = _codes(True, mid, now=1.5, origin=np.zeros(3))
    assert "TRANSITION:HEAD_DISCONTINUITY" in off
    assert "TRANSITION:HEAD_DISCONTINUITY" not in on


def test_kinematics_uses_upstream_limits_not_a_looser_certified_bound():
    # 5 m/s：超过认证 v_max=4，不超过上游 v_max=6。
    mid_v = _poly(0.0, _pva([0, 0, 0], v=[5, 0, 0]), _pva([10, 0, 0]), 4.0)
    off = _codes(False, mid_v, now=0.2)
    on = _codes(True, mid_v, now=0.2)
    assert "TRANSITION:MOTION_KINEMATICS" in off
    assert "TRANSITION:MOTION_KINEMATICS" not in on

    # 7 m/s：连上游限幅也超过，对齐后仍应拒绝。不是靠放宽阈值盖掉。
    over = _poly(0.0, _pva([0, 0, 0], v=[7, 0, 0]), _pva([14, 0, 0]), 4.0)
    on_over = _codes(True, over, now=0.2)
    assert "TRANSITION:MOTION_KINEMATICS" in on_over


def test_branch_unreachable_under_cert_may_clear_with_upstream_limits():
    mid_v = _poly(0.0, _pva([0, 0, 0], v=[5, 0, 0]), _pva([10, 0, 0]), 4.0)
    off = _codes(False, mid_v, now=0.2)
    on = _codes(True, mid_v, now=0.2)
    # 起点已超认证 v_max 时，制动分支常常一并失败。
    assert "TRANSITION:BRANCH_UNREACHABLE" in off
    assert "TRANSITION:BRANCH_UNREACHABLE" not in on

    # 20 m/s 即使用上游限幅也刹不住，这是真问题，必须留下。
    wild = _poly(0.0, _pva([0, 0, 0], v=[20, 0, 0]), _pva([40, 0, 0]), 4.0)
    assert "TRANSITION:BRANCH_UNREACHABLE" in _codes(True, wild, now=0.2) or "TRANSITION:MOTION_KINEMATICS" in _codes(True, wild, now=0.2)


def test_tracking_bound_not_counted_when_aligned_but_threshold_unchanged():
    hold = _poly(0.0, _pva([0, 0, 0]), _pva([0, 0, 0]), 0.5)
    off = _codes(False, hold, now=1.0, actual=[0.16, 0, 0])
    on = _codes(True, hold, now=1.0, actual=[0.16, 0, 0])
    assert TRACKING_BOUND in off
    assert TRACKING_BOUND not in on


def test_align_still_rejects_trajectory_through_pillar():
    origin = np.array([0.0, 0.0, 1.5])
    dest = np.array([5.0, 0.0, 1.5])
    traj = _traj(origin, dest)
    shadow = DroneShadow(
        "uav0", origin, dest, {}, scene_obstacles=PILLAR, contract_align=True,
    )
    shadow.on_selection(_Wrap(traj), _frame(origin), _State(origin), 0.2)
    codes = shadow.on_tick(_Tick(origin, 0.2), origin, 0.2)
    assert ENV_OCCUPIED in shadow.selection
    assert ENV_OCCUPIED in codes


def test_align_keeps_true_head_discontinuity_between_motions():
    first = _poly(0.0, _pva([0, 0, 0]), _pva([2, 0, 0]), 2.0)
    jump = _poly(1.0, _pva([5, 0, 0], v=[1, 0, 0]), _pva([9, 0, 0]), 4.0)
    shadow = DroneShadow("uav0", np.zeros(3), np.array([9.0, 0, 0]), {}, contract_align=True)
    shadow.on_selection(_Wrap(first), None, _State([0, 0, 0]), 0.2)
    shadow.on_selection(_Wrap(jump), None, _State([5, 0, 0]), 1.0)
    assert "TRANSITION:HEAD_DISCONTINUITY" in shadow.selection


def test_cli_switch_off_omits_field_and_on_marks_deviation(tmp_path):
    import json
    import subprocess

    common = [
        sys.executable, "-m", "yopo_l0.run", "--scene", str(G2_CARD), "--seed", "0",
        "--policy", "hover", "--duration", "0.2", "--shadow", "on",
    ]
    off = subprocess.run(common + ["--out", str(tmp_path / "off")], check=True, capture_output=True, text=True)
    on = subprocess.run(
        common + ["--shadow-contract-align", "--out", str(tmp_path / "on")],
        check=True, capture_output=True, text=True,
    )
    off_r = json.loads((tmp_path / "off" / "result.json").read_text())
    on_r = json.loads((tmp_path / "on" / "result.json").read_text())
    assert "shadow_contract_align" not in off_r
    assert on_r["shadow_contract_align"] is True
    assert on_r["upstream_deviation"] is True
