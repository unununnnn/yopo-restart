"""票 B3：起步加速度上限、横向偏差重规划。两个开关默认关，打开即偏离上游。"""

import json
import os
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from conftest import ROOT
from yopo_l0 import crossing
from yopo_l0.b2_plan_diag import apply_b2_overrides, b3_from_fix, fix_deviations, parse_fix
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card
from yopo_swarm_core.plan import KinematicLimits

CROSS = ROOT / "scenes/crossing-pillars.json"
CROSS_SITL = ROOT / "scenes/crossing-pillars-sitl80.json"
ON = {"launch_a_max_mps2": 3.0, "launch_s": 2.5, "replan_lateral_m": 0.5}


def _fleet(params=None):
    card = dict(load_card(CROSS).card)
    if params is not None:
        card["policy_params"] = params
    return crossing.CrossingFleet.from_card(card, 0.01)


def test_b3_is_off_by_default_and_keeps_the_task_hash():
    off = _fleet()
    assert off.b3 == {} and off._launch_limits is None
    assert _fleet({}).cfg.task_hash == off.cfg.task_hash
    on = _fleet({"b3": dict(ON)})
    assert on.b3 == ON
    assert on.cfg.task_hash != off.cfg.task_hash  # 打开才进 task_hash
    with pytest.raises(ValueError):
        _fleet({"b3": {"launch_a_max_mps2": 7.0}})  # 不能高于动力学 a_max
    with pytest.raises(ValueError):
        _fleet({"b3": {"replan_lateral_m": 0.0}})


def test_lateral_replan_ignores_along_lag_and_fires_on_lateral_error():
    fleet = _fleet({"b3": {"replan_lateral_m": 0.5}})
    head = np.array([[2.0, 0.0, 1.5], [5.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    frame = SimpleNamespace(stamp=0.0, depth=np.zeros((2, 2)))
    # 沿向落后 1 m（PX4 的固定滞后）：不换头。
    fleet.ingest("uav0", frame, SimpleNamespace(p=np.array([1.0, 0.1, 1.5]), v=np.array([5.0, 0.0, 0.0]), yaw=0.0))
    assert np.array_equal(fleet._lateral_odom_head("uav0", head, 0.0, 0.0), head)
    # 横向偏 0.6 m：换成里程计状态，加速度清零。
    fleet.ingest("uav0", frame, SimpleNamespace(p=np.array([2.0, 0.6, 1.5]), v=np.array([5.0, 0.3, 0.0]), yaw=0.0))
    out = fleet._lateral_odom_head("uav0", head, 0.0, 0.1)
    assert np.allclose(out[0], [2.5, 0.63, 1.5]) and np.allclose(out[1], [5.0, 0.3, 0.0])
    assert np.allclose(out[2], 0.0) and fleet.b3_counts["replan_lateral"] == 1


def test_launch_cap_time_scales_the_curve_and_keeps_the_head():
    fleet = _fleet({"b3": {"launch_a_max_mps2": 3.0}})
    head = np.array([[0.0, 0.0, 1.5], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    tail = np.array([[3.0, 0.5, 1.5], [3.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    inner = np.array([1.5, 0.3, 1.5])
    lim = fleet.pcfg.limits
    cap = KinematicLimits(lim.v_max, 3.0, lim.j_max)
    fleet._launch_limits = None
    raw0 = fleet._launch_minco(0.0, head, tail, inner, [0.7, 0.7])
    assert not fleet._launch_prefix_ok(raw0, cap)  # 原曲线要飞的前缀超过 3 m/s²
    fleet._launch_limits = cap
    raw = fleet._launch_minco(0.0, head, tail, inner, [0.7, 0.7])
    assert fleet._launch_prefix_ok(raw, cap)
    assert fleet._fit_yopo(raw) is not None  # 拟合、停止尾仍按场景 a_max
    assert fleet.b3_counts["launch_scaled"] == 1
    assert np.allclose(raw.evaluate(0.0, 0), head[0]) and np.allclose(raw.evaluate(0.0, 1), 0.0)


def test_fix_string_maps_to_policy_params_and_marks_deviation():
    fix = parse_fix("launch=3:2.5,relat=0.5")
    assert b3_from_fix(fix) == ON
    assert b3_from_fix(parse_fix("")) is None
    assert fix_deviations(fix) == [
        "偏离上游：起步 2.5 s 内计划加速度上限 3 m/s²",
        "偏离上游：横向偏差超过 0.5 m 时从里程计起规划",
    ]
    card = load_card(CROSS)
    same, dev = apply_b2_overrides(card, b3=None)
    assert same is card and dev == []
    on, _ = apply_b2_overrides(card, b3=b3_from_fix(fix))
    assert on.card["policy_params"]["b3"] == ON
    for bad in ("launch=0:2", "launch=3:-1", "relat=0"):
        with pytest.raises(ValueError):
            parse_fix(bad)


def test_dry_run_reports_b3_deviation(tmp_path):
    e = {k: v for k, v in os.environ.items() if not k.startswith(("YOPO_B2_", "YOPO_SITL_", "YOPO_AGENT_"))}
    e.update({"YOPO_B2_FIX": "launch=3:2.5,relat=0.5", "PYTHONPATH": f"{ROOT}:{ROOT / 'core/src'}"})
    out = tmp_path / "p.json"
    r = subprocess.run(
        [sys.executable, str(ROOT / "scripts/sitl/yopo_l0_live.py"), "--scene", str(CROSS_SITL),
         "--policy", "crossing", "--dry-run", "--out", str(out)],
        cwd=ROOT, env=e, capture_output=True, text=True, check=False,
    )
    assert r.returncode == 0, r.stderr
    b2 = json.loads(out.read_text())["b2"]
    assert any("起步" in d for d in b2["deviations"]) and any("横向" in d for d in b2["deviations"])


def test_l0_short_flight_with_both_switches_on():
    card, _ = apply_b2_overrides(load_card(CROSS), b3=dict(ON))
    holder = {}
    original = crossing.CrossingFleet.from_card.__func__

    def from_card(cls, card_dict, physics_dt, *, record_inputs=False):
        holder["f"] = original(cls, card_dict, physics_dt, record_inputs=record_inputs)
        return holder["f"]

    crossing.CrossingFleet.from_card = classmethod(from_card)
    try:
        out = run(card, RunConfig(seed=0, shadow=False, duration_s=3.0))
    finally:
        crossing.CrossingFleet.from_card = classmethod(original)
    assert out.result["collisions"] == 0 if "collisions" in out.result else out.result["outcome"] != "COLLISION"
    assert holder["f"].b3_counts["launch_plans"] > 0


def test_container_name_override_is_opt_in():
    script = f'source "{ROOT}/scripts/sitl/common.sh"; sitl_container_name 3'
    env = {k: v for k, v in os.environ.items() if k != "YOPO_SITL_CONTAINER_NAME"}
    default = subprocess.run(["bash", "-c", script], env=env, capture_output=True, text=True, check=True)
    assert default.stdout.strip() == "yopo-restart-sitl-n3"
    env["YOPO_SITL_CONTAINER_NAME"] = "yopo-restart-b3-sitl-n3"
    named = subprocess.run(["bash", "-c", script], env=env, capture_output=True, text=True, check=True)
    assert named.stdout.strip() == "yopo-restart-b3-sitl-n3"
