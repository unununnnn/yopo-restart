"""票 03：影子盾只记录。同 seed 下开/关，批准动作逐位一致，只有影子字段不同。"""

import json
import subprocess
import sys

from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import load_card
from yopo_l0.shadow import (
    SHADOW_RESULT_KEYS,
    SHADOW_TICK_KEYS,
    TRACKING_BOUND,
    record_oracle_track_lag,
)

from conftest import G2_CARD, ROOT

LIFT = ROOT / "scenes/g2-hover-lift.json"
PILLAR = ROOT / "scenes/g2-pillar.json"
OPEN3 = ROOT / "scenes/open-3drone.json"


def strip_wall(r):
    return {k: v for k, v in r.items() if k not in WALL_FIELDS}


def strip_shadow_result(r):
    return {k: v for k, v in r.items() if k not in SHADOW_RESULT_KEYS and k not in WALL_FIELDS}


def strip_shadow_ticks(ticks):
    return [{k: v for k, v in t.items() if k not in SHADOW_TICK_KEYS} for t in ticks]


def actions(ticks):
    return [(t["cmd_p"], t["cmd_v"], t["cmd_a"], t["p"], t["v"], t["a"], t["yaw"]) for t in ticks]


def test_shadow_off_is_null_and_on_matches_actions():
    card = load_card(G2_CARD)
    kw = dict(seed=0, policy="hover", record_planner_inputs=True, duration_s=0.5)
    off = run(card, RunConfig(shadow=False, **kw))
    on = run(card, RunConfig(shadow=True, **kw))
    assert off.result["shadow"] is False and on.result["shadow"] is True
    assert off.result["shadow_reject_rate"] is None
    assert off.result["shadow_reject_histogram"] is None
    assert off.result["shadow_gates"] is None
    assert all(t["would_reject"] is None and t["reject_codes"] is None for t in off.ticks)
    assert isinstance(on.result["shadow_reject_rate"], float)
    assert isinstance(on.result["shadow_reject_histogram"], dict)
    assert [g["gate"] for g in on.result["shadow_gates"]] == [
        "env_evidence", "stop_select", "plan_contract", "oracle_track_lag",
    ]
    assert all(g["wired"] for g in on.result["shadow_gates"])
    assert all(isinstance(t["would_reject"], bool) and isinstance(t["reject_codes"], list) for t in on.ticks)
    assert strip_shadow_result(off.result) == strip_shadow_result(on.result)
    assert strip_shadow_ticks(off.ticks) == strip_shadow_ticks(on.ticks)
    assert actions(off.ticks) == actions(on.ticks)
    assert off.planner_inputs == on.planner_inputs
    assert off.result["outcome"] == on.result["outcome"] == "SUCCESS"
    # 悬停命令 t_switch == now，认证门要求切换在未来。
    assert on.result["shadow_reject_histogram"].get("TIME:SWITCH_NOT_IN_FUTURE", 0) > 0


def test_shadow_on_same_seed_is_deterministic():
    card = load_card(G2_CARD)
    cfg = RunConfig(seed=4, policy="hover", shadow=True, record_planner_inputs=True, duration_s=0.4)
    a = run(card, cfg)
    b = run(card, cfg)
    assert strip_wall(a.result) == strip_wall(b.result)
    assert a.ticks == b.ticks
    assert a.planner_inputs == b.planner_inputs


def test_lift_records_tracking_bound_without_changing_outcome():
    card = load_card(LIFT)
    kw = dict(seed=1, policy="hover", duration_s=1.0)
    off = run(card, RunConfig(shadow=False, **kw))
    on = run(card, RunConfig(shadow=True, **kw))
    hist = on.result["shadow_reject_histogram"]
    assert hist.get(TRACKING_BOUND, 0) > 0
    assert "COLLISION" not in hist
    assert off.result["outcome"] == on.result["outcome"] == "SUCCESS"
    assert actions(off.ticks) == actions(on.ticks)
    assert strip_shadow_result(off.result) == strip_shadow_result(on.result)
    rejected = sum(t["would_reject"] for t in on.ticks)
    assert on.result["shadow_reject_rate"] == rejected / len(on.ticks)
    for code, n in hist.items():
        assert n == sum(code in t["reject_codes"] for t in on.ticks)


def test_open_3drone_shadow_is_per_drone_and_does_not_change_actions():
    card = load_card(OPEN3)
    kw = dict(seed=0, policy="hover", record_planner_inputs=True, duration_s=0.4)
    off = run(card, RunConfig(shadow=False, **kw))
    on = run(card, RunConfig(shadow=True, **kw))
    assert on.result["n_drones"] == 3
    assert [row["id"] for row in on.result["shadow_drones"]] == ["uav0", "uav1", "uav2"]
    assert off.result["shadow_drones"] is None
    assert actions(off.ticks) == actions(on.ticks)
    assert off.planner_inputs == on.planner_inputs
    assert strip_shadow_result(off.result) == strip_shadow_result(on.result)
    assert off.result["min_inter_drone_distance_m"] == on.result["min_inter_drone_distance_m"]
    rejected = 0
    for row in on.result["shadow_drones"]:
        subset = [t for t in on.ticks if t["uav"] == row["id"]]
        assert subset and len(subset) * 3 == len(on.ticks)
        n = sum(bool(t["would_reject"]) for t in subset)
        rejected += n
        assert row["shadow_reject_rate"] == n / len(subset)
        for code, count in row["shadow_reject_histogram"].items():
            assert count == sum(code in t["reject_codes"] for t in subset)
    assert on.result["shadow_reject_rate"] == rejected / len(on.ticks)


def test_upstream_shadow_does_not_change_actions():
    card = load_card(PILLAR)
    kw = dict(seed=0, policy="upstream", record_planner_inputs=True, duration_s=0.4)
    off = run(card, RunConfig(shadow=False, **kw))
    on = run(card, RunConfig(shadow=True, **kw))
    assert actions(off.ticks) == actions(on.ticks)
    assert off.planner_inputs == on.planner_inputs
    assert strip_shadow_result(off.result) == strip_shadow_result(on.result)
    assert off.result["outcome"] == on.result["outcome"]
    assert on.result["shadow_reject_histogram"]  # 上游选择过了认证门，会留下拒绝码
    assert all(t["would_reject"] is None for t in off.ticks)


def test_certified_shadow_does_not_change_actions():
    card = load_card(PILLAR)
    kw = dict(seed=0, policy="certified", duration_s=0.4)
    off = run(card, RunConfig(shadow=False, **kw))
    on = run(card, RunConfig(shadow=True, **kw))
    assert actions(off.ticks) == actions(on.ticks)
    assert strip_shadow_result(off.result) == strip_shadow_result(on.result)
    assert off.result["plan_codes"] == on.result["plan_codes"]


def test_oracle_lag_and_tracking_codes_match_flight_oracle_bounds():
    far = record_oracle_track_lag(
        20.0, 20.0, (0.0, 0.0, 1.0), (10.0, 0.0, 1.0), (0.0, 0.0, 1.0), (10.0, 0.0, 1.0), 0.0,
    )
    assert far == ("TRACKING_BOUND", "LAG_VIOLATION")
    lag_only = record_oracle_track_lag(
        20.0, 20.0, (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (10.0, 0.0, 1.0), 0.0,
    )
    assert lag_only == ("LAG_VIOLATION",)
    assert record_oracle_track_lag(
        0.0, 0.0, (float("nan"), 0.0, 1.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), 0.0,
    ) == ("ODOMETRY_AGE_OR_NONFINITE",)


def test_cli_shadow_on_off(tmp_path):
    common = [
        sys.executable, "-m", "yopo_l0.run", "--scene", str(G2_CARD), "--seed", "0",
        "--policy", "hover", "--duration", "0.2",
    ]
    on_dir, off_dir, bare_dir = tmp_path / "on", tmp_path / "off", tmp_path / "bare"
    p_on = subprocess.run(
        [*common, "--shadow", "on", "--out", str(on_dir)], cwd=ROOT, capture_output=True, text=True, timeout=120,
    )
    p_off = subprocess.run(
        [*common, "--shadow", "off", "--out", str(off_dir)], cwd=ROOT, capture_output=True, text=True, timeout=120,
    )
    p_bare = subprocess.run(
        [*common, "--no-shadow", "--out", str(bare_dir)], cwd=ROOT, capture_output=True, text=True, timeout=120,
    )
    assert p_on.returncode == 0, p_on.stderr
    assert p_off.returncode == 0, p_off.stderr
    assert p_bare.returncode == 0, p_bare.stderr
    r_on = json.loads((on_dir / "result.json").read_text())
    r_off = json.loads((off_dir / "result.json").read_text())
    r_bare = json.loads((bare_dir / "result.json").read_text())
    assert r_on["shadow"] is True and isinstance(r_on["shadow_reject_histogram"], dict)
    assert r_off["shadow"] is False and r_off["shadow_reject_rate"] is None
    assert r_bare["shadow"] is False and r_bare["shadow_reject_histogram"] is None
    ticks_on = [json.loads(line) for line in (on_dir / "ticks.jsonl").read_text().splitlines()]
    ticks_off = [json.loads(line) for line in (off_dir / "ticks.jsonl").read_text().splitlines()]
    assert ticks_on and all(isinstance(t["reject_codes"], list) for t in ticks_on)
    assert all(t["would_reject"] is None and t["reject_codes"] is None for t in ticks_off)
    assert actions(ticks_on) == actions(ticks_off)
