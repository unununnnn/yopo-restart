"""票 11a：档位批跑每个档位一张表；墙钟超过上限时写入分批方案，退出码仍成功。"""

import json

import pytest

from yopo_l0.batch import _strip, det_mismatch, main as batch_main, resolve_tiers
from yopo_l0.scene import load_card

from conftest import ROOT

CARD = "scenes/formation-6-open.json"
LABELS = ["none", "drop 5%", "drop 20%", "duplicate 10%", "reorder+delay", "combined"]


def test_unknown_tier_is_a_usage_error():
    with pytest.raises(SystemExit, match="未知档位"):
        resolve_tiers("drop-5,nope")


def test_strip_drops_wall_clock_inside_the_batching_plan():
    a = {
        "seed": 0,
        "wall_time_s": 1.0,
        "plan_wall_ms": {"count": 3, "mean": 1.0, "p50": 1.0, "p95": 2.0, "max": 2.0},
        "batching_plan": {"wall_time_s": 1.0},
    }
    b = {
        "seed": 0,
        "wall_time_s": 50.0,
        "plan_wall_ms": {"count": 3, "mean": 9.0, "p50": 8.0, "p95": 12.0, "max": 20.0},
        "batching_plan": {"wall_time_s": 50.0, "commands": ["x"]},
    }
    assert _strip(a) == _strip(b)
    assert "plan_wall_ms" not in _strip(a)


def test_strip_ignores_repo_commit_dirty_flag():
    """同机改已跟踪文件时，两次重跑的 commit 会从 sha 变成 sha-dirty，轨迹仍一致。"""
    a = {
        "seed": 0,
        "commit": "47da396b00e5190c8de1585fc392b39122a32108",
        "arrived": 6,
        "wall_time_s": 1.0,
    }
    b = {
        "seed": 0,
        "commit": "47da396b00e5190c8de1585fc392b39122a32108-dirty",
        "arrived": 6,
        "wall_time_s": 50.0,
    }
    assert _strip(a) == _strip(b)
    assert "commit" not in _strip(a)


def test_det_mismatch_records_trajectory_fields_not_commit(tmp_path):
    left = {
        "commit": "abc123",
        "arrived": 6,
        "wall_time_s": 1.0,
        "plan_wall_ms": {"count": 1, "mean": 1.0, "p50": 1.0, "p95": 1.0, "max": 1.0},
    }
    right = {
        "commit": "abc123-dirty",
        "arrived": 5,
        "wall_time_s": 9.0,
        "plan_wall_ms": {"count": 1, "mean": 8.0, "p50": 8.0, "p95": 8.0, "max": 8.0},
    }
    assert det_mismatch(left, [{"p": [0.0]}], right, [{"p": [0.0]}]) is not None
    mismatch = det_mismatch(left, [{"p": [0.0]}], right, [{"p": [1.0]}])
    assert mismatch is not None
    paths = [row["path"] for row in mismatch["result_diffs"]]
    assert "commit" not in paths
    assert "arrived" in paths
    assert mismatch["result_equal"] is False
    assert mismatch["ticks_equal"] is False
    assert mismatch["tick_diffs"]
    # 只有 commit / 墙钟不同时，比较仍算一致，不写差分。
    assert det_mismatch(left, [{"p": [0.0]}], {**right, "arrived": 6}, [{"p": [0.0]}]) is None


def test_repeated_seed_stays_deterministic_when_git_commit_flips(tmp_path, monkeypatch):
    values = iter(["abc123def4567890", "abc123def4567890-dirty"])
    monkeypatch.setattr("yopo_l0.runner.git_commit", lambda: next(values, "abc123def4567890-dirty"))
    out = tmp_path / "det-dirty"
    rc = batch_main([
        CARD, "--form", "inprocess", "--tiers", "none",
        "--seeds", "0", "--duration", "0.3", "--no-shadow", "--out", str(out),
    ])
    assert rc == 0
    md = (out / "summary.md").read_text()
    assert "formation-6-open@formation/none seed0 一致" in md
    seed_dir = out / "formation-6-open@formation" / "none" / "seed0"
    assert not (seed_dir / "det_diff.json").exists()


def _fake_result(job):
    path, policy, seed, _duration, _out, check, _shadow, form, bus, tier_id, wall_cap, _safe_radius, *_rest = job
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
    assert md.count("| seed | 到达率 | 碰撞 | 最小机间距离 m | 队形误差 p95 m | 影子拒绝率 | 单次墙钟 s | 规划墙钟 p95 ms（只报数） |") == 6
    assert "| 0 | 1.000 | 0 | 2.980 | 0.066 | 1.000 | 14.0 | — |" in md
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


def test_six_drone_det_cards_do_not_use_wall_budget():
    """B3/B5 在高负载下看到的同 seed 不一致，不是后段墙钟预算造成的。

    ``formation-6-open``（本文件的短时确定性测试）和 ``crossing-6-forest``
    （B5 批跑 seed 0）都没有打开后段，也没有打开几何回退。缺省预算是按评估
    次数，不读墙钟。比较时已经去掉 ``wall_time_s``、``plan_wall_ms``、
    ``batching_plan``、``commit`` 和几何回退的 ``check_time_ms``。因此不要为了
    让测试常绿把硬断言拿掉。仓库脏标记见 ``docs/research/t2-six-drone-det.md``。
    """
    from yopo_l0.backend.config import config_from_params

    for rel in (CARD, "scenes/crossing-6-forest.json"):
        raw = json.loads((ROOT / rel).read_text())
        assert config_from_params(raw.get("policy_params"), a_max=6.0) is None, rel
        assert not (raw.get("geometric_fallback") or {}).get("enabled"), rel


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
