"""票 SH4：TRACKING_BOUND 修正开关默认关。关时不加字段；打开只改影子记录，不改批准动作，不放宽 0.15 m。"""

import json
import subprocess
import sys

import numpy as np

from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card
from yopo_l0.shadow import (
    ENV_OCCUPIED,
    LAG_VIOLATION,
    ORACLE_TRACKING_BOUND,
    TRACKING_BOUND,
    DroneShadow,
    codes_would_reject,
    record_oracle_track_lag,
)

from conftest import G2_CARD, ROOT
from test_shadow import actions, strip_shadow_result, strip_shadow_ticks, strip_wall
from test_shadow_env_evidence import PILLAR, _State, _Wrap, _frame, _traj
from test_shadow_contract_align import _Tick

LIFT = ROOT / "scenes/g2-hover-lift.json"


def _oracle_tick(*, tracking=False, now=1.0, actual, desired=(0, 0, 0), start=(0, 0, 0), goal=(5, 0, 0)):
    """只记 oracle，不跑选择门，避免包装码和环境码把 would_reject 打满。"""
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    shadow = DroneShadow("uav0", start, goal, {}, tracking_align=tracking)
    codes = shadow.on_tick(_Tick(actual, now), np.asarray(desired, dtype=float), now)
    return codes, codes_would_reject(codes, tracking_align=tracking)


def test_switch_defaults_off_and_adds_no_fields():
    assert RunConfig().shadow_tracking_align is False
    assert ORACLE_TRACKING_BOUND == 0.15
    card = load_card(G2_CARD)
    kw = dict(seed=0, policy="hover", duration_s=0.3, shadow=True)
    plain = run(card, RunConfig(**kw))
    explicit = run(card, RunConfig(shadow_tracking_align=False, **kw))
    assert "shadow_tracking_align" not in plain.result
    assert "shadow_tracking_align" not in explicit.result
    assert strip_wall(plain.result) == strip_wall(explicit.result)
    assert plain.ticks == explicit.ticks


def test_switch_does_not_change_trajectory_and_tags_deviation():
    card = load_card(G2_CARD)
    kw = dict(seed=1, policy="hover", record_planner_inputs=True, duration_s=0.4, shadow=True)
    off = run(card, RunConfig(shadow_tracking_align=False, **kw))
    on = run(card, RunConfig(shadow_tracking_align=True, **kw))
    assert on.result["shadow_tracking_align"] is True
    assert on.result["upstream_deviation"] is True
    assert "shadow_tracking_align" not in off.result
    assert actions(off.ticks) == actions(on.ticks)
    assert off.planner_inputs == on.planner_inputs
    assert strip_shadow_ticks(off.ticks) == strip_shadow_ticks(on.ticks)
    assert off.result["outcome"] == on.result["outcome"]
    assert off.result["min_obstacle_distance_m"] == on.result["min_obstacle_distance_m"]
    dropped = {"shadow_tracking_align", "upstream_deviation"}
    kept = {k: v for k, v in on.result.items() if k not in dropped}
    assert strip_shadow_result(off.result) == strip_shadow_result(kept)


def test_bound_stays_015_and_align_drops_only_would_reject():
    assert record_oracle_track_lag(
        1.0, 1.0, (0.15, 0, 0), (0, 0, 0), (0, 0, 0), (5, 0, 0), 0.0,
    ) == ()
    assert record_oracle_track_lag(
        1.0, 1.0, (0.16, 0, 0), (0, 0, 0), (0, 0, 0), (5, 0, 0), 0.0,
    ) == (TRACKING_BOUND,)

    off_codes, off_wr = _oracle_tick(tracking=False, actual=(0.16, 0, 0))
    on_codes, on_wr = _oracle_tick(tracking=True, actual=(0.16, 0, 0))
    assert off_codes == (TRACKING_BOUND,)
    assert on_codes == (TRACKING_BOUND,)
    assert off_wr is True
    assert on_wr is False
    assert ORACLE_TRACKING_BOUND == 0.15


def test_align_still_rejects_lag_beyond_upstream_and_path_through_pillar():
    # 沿路径落后 3 m（参考速度 0.15 m/s × 20 s），超过 lag_limit 1.5 m。不是把 0.15 m 放宽。
    codes, wr = _oracle_tick(tracking=True, now=20.0, actual=(0, 0, 0), desired=(0, 0, 0), start=(0, 0, 0), goal=(10, 0, 0))
    assert LAG_VIOLATION in codes
    assert wr is True

    origin = np.array([0.0, 0.0, 1.5])
    dest = np.array([5.0, 0.0, 1.5])
    traj = _traj(origin, dest, duration=4.0)
    shadow = DroneShadow(
        "uav0", origin, dest, {}, scene_obstacles=PILLAR, contract_align=True, tracking_align=True,
    )
    shadow.on_selection(_Wrap(traj), _frame(origin), _State(origin), 0.2)
    codes = shadow.on_tick(_Tick(origin, 0.2), origin, 0.2)
    assert shadow.selection == (ENV_OCCUPIED,)
    assert ENV_OCCUPIED in codes
    assert codes_would_reject(codes, tracking_align=True) is True


def test_lift_keeps_tracking_measurement_when_contract_also_aligned():
    card = load_card(LIFT)
    kw = dict(seed=1, policy="hover", duration_s=1.0, shadow=True)
    base = run(card, RunConfig(**kw))
    contract = run(card, RunConfig(shadow_contract_align=True, **kw))
    both = run(card, RunConfig(shadow_contract_align=True, shadow_tracking_align=True, **kw))
    n_track = base.result["shadow_reject_histogram"].get(TRACKING_BOUND, 0)
    assert n_track > 0
    # SH3 单独打开会从码表拿掉跟踪测量。SH4 打开后 0.15 m 超界仍记在直方图里，但不打 would_reject。
    assert contract.result["shadow_reject_histogram"].get(TRACKING_BOUND, 0) == 0
    assert contract.result["shadow_reject_rate"] == 0.0
    assert both.result["shadow_reject_histogram"].get(TRACKING_BOUND, 0) == n_track
    assert both.result["shadow_reject_rate"] == 0.0
    assert both.result["shadow_tracking_align"] is True
    assert actions(base.ticks) == actions(contract.ticks) == actions(both.ticks)
    assert base.result["outcome"] == both.result["outcome"] == "SUCCESS"


def test_cli_switch_off_omits_field_and_on_marks_deviation(tmp_path):
    common = [
        sys.executable, "-m", "yopo_l0.run", "--scene", str(G2_CARD), "--seed", "0",
        "--policy", "hover", "--duration", "0.2", "--shadow", "on",
    ]
    subprocess.run(common + ["--out", str(tmp_path / "off")], check=True, capture_output=True, text=True, cwd=ROOT)
    subprocess.run(
        common + ["--shadow-tracking-align", "--out", str(tmp_path / "on")],
        check=True, capture_output=True, text=True, cwd=ROOT,
    )
    off_r = json.loads((tmp_path / "off" / "result.json").read_text())
    on_r = json.loads((tmp_path / "on" / "result.json").read_text())
    assert "shadow_tracking_align" not in off_r
    assert on_r["shadow_tracking_align"] is True
    assert on_r["upstream_deviation"] is True


def test_sitl_dry_run_prints_tracking_align_flag():
    SITL = ROOT / "scripts" / "sitl"
    on = subprocess.run(
        [
            "bash", str(SITL / "run_yopo_l0_sitl.sh"),
            "--dry-run", "--n", "3",
            "--scene", "scenes/crossing-pillars-sitl80.json",
            "--policy", "crossing", "--lockstep",
            "--shadow-env-evidence", "--fleet-shadow",
            "--shadow-contract-align", "--shadow-tracking-align",
        ],
        cwd=ROOT, capture_output=True, text=True, timeout=30,
    )
    assert on.returncode == 0, on.stderr
    assert "shadow_tracking_align=1" in on.stdout
    assert "shadow_contract_align=1" in on.stdout
    off = subprocess.run(
        [
            "bash", str(SITL / "run_yopo_l0_sitl.sh"),
            "--dry-run", "--n", "3",
            "--scene", "scenes/crossing-pillars-sitl80.json",
            "--policy", "crossing",
        ],
        cwd=ROOT, capture_output=True, text=True, timeout=30,
    )
    assert off.returncode == 0, off.stderr
    assert "shadow_tracking_align=0" in off.stdout
    assert "shadow_contract_align=0" in off.stdout
