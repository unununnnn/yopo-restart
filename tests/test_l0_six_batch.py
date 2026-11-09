"""票 11a：档位批跑每个档位一张表；墙钟超过上限时写入分批方案，退出码仍成功。"""

import json

import pytest

from yopo_l0.batch import _strip, main as batch_main, resolve_tiers
from yopo_l0.scene import load_card

from conftest import ROOT

CARD = "scenes/formation-6-open.json"
LABELS = ["none", "drop 5%", "drop 20%", "duplicate 10%", "reorder+delay", "combined"]


def test_unknown_tier_is_a_usage_error():
    with pytest.raises(SystemExit, match="未知档位"):
        resolve_tiers("drop-5,nope")


def test_strip_drops_wall_clock_inside_the_batching_plan():
    a = {"seed": 0, "wall_time_s": 1.0, "batching_plan": {"wall_time_s": 1.0}}
    b = {"seed": 0, "wall_time_s": 50.0, "batching_plan": {"wall_time_s": 50.0, "commands": ["x"]}}
    assert _strip(a) == _strip(b)


def _fake_result(job):
    path, policy, seed, _duration, _out, check, _shadow, form, bus, tier_id, wall_cap = job
    result = {
        "commit": "abc123def4567890",
        "scene": "formation-6-open",
        "scene_card_hash": "deadbeef",
        "seed": seed,
        "tier": "L0",
        "policy": policy,
        "shadow": True,
        "task": "formation",
        "outcome": "SUCCESS",
        "reason": "GOAL_REACHED_AND_HELD",
        "n_drones": 6,
        "arrived": 6,
        "arrival_rate": 1.0,
        "arrival_time_s": 21.0,
        "min_obstacle_distance_m": 1.5,
        "min_inter_drone_distance_m": 2.98,
        "final_goal_error_m": 0.01,
        "formation_error_p95_m": 0.066,
        "shadow_reject_rate": 1.0,
        "shadow_reject_histogram": {},
        "plan_codes": {},
        "sim_duration_s": 21.0,
        "wall_time_s": 14.0,
        "bus": {"drop": bus.drop if bus else 0.0},
        "formation": {"local_commits": {f"uav{i}": 1 for i in range(6)}},
        "form": "ros2" if form == "ros2" else "inprocess",
        "fault_tier": tier_id,
    }
    if wall_cap is not None and result["wall_time_s"] > wall_cap:
        result["batching_plan"] = {
            "wall_cap_s": wall_cap,
            "scene": result["scene"],
            "scene_path": path,
            "fault_tier": tier_id,
            "seed": seed,
            "form": result["form"],
            "wall_time_s": result["wall_time_s"],
            "split": "按 seed 分批",
            "if_still_over": "降相机帧率",
            "commands": [f"python -m yopo_l0.batch {path} --tiers {tier_id} --seeds {seed}"],
        }
    return result, True if check else None


def test_each_tier_gets_its_own_table(tmp_path, monkeypatch):
    monkeypatch.setattr("yopo_l0.batch._one", _fake_result)
    out = tmp_path / "m4"
    rc = batch_main([
        CARD, "--form", "ros2", "--tiers", "scenes/formation-6-tiers.json",
        "--seeds", "0,1", "--no-determinism", "--wall-cap", "120", "--out", str(out),
    ])
    assert rc == 0
    md = (out / "summary.md").read_text()
    assert "形态 ROS 2" in md
    assert "未冻结 / report-only" in md
    for label in LABELS:
        assert f"/ {label}（" in md
    assert md.count("| seed | 到达率 | 碰撞 | 最小机间距离 m | 队形误差 p95 m | 影子拒绝率 | 单次墙钟 s |") == 6
    assert "| 0 | 1.000 | 0 | 2.980 | 0.066 | 1.000 | 14.0 |" in md
    assert "| 合计 |" in md
    assert not (out / "batching_plan.json").exists()
    saved = json.loads((out / "results.json").read_text())
    assert isinstance(saved, list) and len(saved) == 12
    assert {row["fault_tier"] for row in saved} == {t["id"] for t in resolve_tiers("all")}
    assert all(row["form"] == "ros2" for row in saved)


def test_wall_cap_writes_a_plan_and_still_exits_zero(tmp_path, monkeypatch):
    monkeypatch.setattr("yopo_l0.batch._one", _fake_result)
    out = tmp_path / "over"
    rc = batch_main([
        CARD, "--form", "inprocess", "--tiers", "none,drop-5",
        "--seeds", "0", "--no-determinism", "--wall-cap", "10", "--out", str(out),
    ])
    assert rc == 0
    md = (out / "summary.md").read_text()
    assert "分批方案" in md
    assert "不因此失败" in md
    plan = json.loads((out / "batching_plan.json").read_text())
    assert plan["wall_cap_s"] == 10
    assert [row["fault_tier"] for row in plan["exceeded"]] == ["none", "drop-5"]
    assert len(plan["commands"]) == 2
    saved = json.loads((out / "results.json").read_text())
    assert all("batching_plan" in row for row in saved)
    assert all(row["batching_plan"]["wall_time_s"] == 14.0 for row in saved)


def test_repeated_seed_stays_deterministic_with_tier_metadata(tmp_path):
    out = tmp_path / "det"
    rc = batch_main([
        CARD, "--form", "inprocess", "--tiers", "none",
        "--seeds", "0", "--duration", "0.3", "--no-shadow", "--out", str(out),
    ])
    assert rc == 0
    md = (out / "summary.md").read_text()
    assert "formation-6-open@formation/none seed0 一致" in md


def test_real_run_over_a_zero_cap_records_the_plan(tmp_path):
    out = tmp_path / "real"
    rc = batch_main([
        CARD, "--form", "inprocess", "--tiers", "none",
        "--seeds", "0", "--duration", "0.2", "--no-determinism", "--no-shadow",
        "--wall-cap", "0", "--out", str(out),
    ])
    assert rc == 0
    saved = json.loads((out / "results.json").read_text())
    assert saved[0]["n_drones"] == 6
    assert saved[0]["fault_tier"] == "none"
    assert saved[0]["form"] == "inprocess"
    assert saved[0]["wall_time_s"] > 0
    plan = saved[0]["batching_plan"]
    assert plan["wall_cap_s"] == 0
    assert plan["seed"] == 0
    assert "commands" in plan and "formation-6-open" in plan["commands"][0]
    on_disk = json.loads((out / "formation-6-open@formation" / "none" / "seed0" / "result.json").read_text())
    assert on_disk["batching_plan"]["fault_tier"] == "none"
    card = load_card(ROOT / CARD)
    assert card.hash in (out / "summary.md").read_text()
