"""票 E-C8b：六机 014 对向的内存场景、六臂开关、脱困目标和断点续跑。"""

import importlib.util
import json
import subprocess
import sys

import numpy as np
import pytest

from yopo_l0.backend.config import config_from_params
from yopo_l0.dynamics import State
from yopo_l0.policy import read_empty_brake, read_track_clearance, read_track_clearance_mode
from yopo_l0.sensor import DepthFrame

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-c8b-ablation.py"
    spec = importlib.util.spec_from_file_location("e_c8b_ablation", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _watched():
    return [
        ROOT / "scenes/ego-density-014-swap3.json",
        ROOT / "scenes/ego-density-014.json",
        ROOT / "scenes/suite.txt",
        ROOT / "docs/tickets.md",
        ROOT / "README.md",
        ROOT / "scripts/regress.sh",
        ROOT / "scripts/premerge.sh",
        ROOT / "yopo_l0/policy.py",
        ROOT / "yopo_l0/runner.py",
        ROOT / "yopo_l0/backend/bspline.py",
        ROOT / "yopo_l0/backend/config.py",
        ROOT / "core/src/yopo_swarm_core/baseline.py",
    ]


def _blank_depth(fill=np.nan) -> np.ndarray:
    return np.full((96, 160), fill, dtype=np.float32)


def test_swap6_reuses_swap3_geometry_and_ec6_placement():
    mod = _script()
    raw = json.loads((ROOT / "scenes/ego-density-014-swap3.json").read_text(encoding="utf-8"))
    assert raw["geometry"] == "ego-density-014.json"
    geom = mod.swap3_geometry()
    assert geom["header"]["target_density_per_m2"] == pytest.approx(0.14)
    assert len(geom["cylinders"]) == 56
    ours = mod.swap6_placement()
    theirs = mod.load_ec6().swap6_placement()
    assert [d["id"] for d in ours] == [f"uav{i}" for i in range(6)]
    for a, b in zip(ours, theirs):
        assert a["start_enu"] == pytest.approx(b["start_enu"])
        assert a["goal_enu"] == pytest.approx(b["goal_enu"])
        assert a["radius_m"] == pytest.approx(b["radius_m"])
        assert a["pushed_outside"] is b["pushed_outside"]
    assert ours[0]["start_enu"] == pytest.approx([8.0, 0.0, 1.5])
    assert ours[0]["goal_enu"] == pytest.approx([-8.0, 0.0, 1.5])
    assert ours[2]["pushed_outside"] is True
    card = mod.build_card("A")
    assert card.name == "ego-density-014-swap6"
    assert card.card["task"]["type"] == "goto"
    assert not (ROOT / "scenes/ego-density-014-swap6.json").exists()
    mod.assert_endpoints_clear(card)


def test_six_arms_only_touch_the_in_memory_card(monkeypatch):
    mod = _script()
    for key in ("YOPO_PEER_SPHERE", "YOPO_PEER_SPHERE_RADIUS_M", "YOPO_PEER_SPHERE_WEIGHT"):
        monkeypatch.delenv(key, raising=False)
    before = {path: path.read_bytes() for path in _watched()}
    for arm in mod.ARMS:
        card = mod.build_card(arm)
        params = card.card["policy_params"]
        cfg = mod.arm_config(arm)
        spec = mod.arm_spec(arm)
        assert params["research"]["report_only"] is True
        assert params["research"]["escape_replan"] is spec["escape"]
        assert "inter_drone_filter" not in params.get("upstream", {})
        assert "safe_radius" not in params.get("upstream", {})
        assert read_empty_brake(params) is spec["brake"]
        assert read_track_clearance(params) is spec["geom"]
        if spec["geom"]:
            assert read_track_clearance_mode(params) == "geom"
            assert params["upstream"]["track_margin_m"] == pytest.approx(0.08)
        else:
            assert "upstream" not in params or "track_clearance" not in params["upstream"]
        backend = config_from_params(params, a_max=6.0)
        if spec["sphere"] is None:
            assert backend is None
            assert "backend" not in params
        else:
            radius, weight = spec["sphere"]
            assert backend is not None
            assert backend.peer_sphere_enabled is True
            assert backend.swarm_penalty is False
            assert backend.peer_sphere_radius_m == pytest.approx(radius)
            assert backend.peer_sphere_weight == pytest.approx(weight)
        assert cfg["deviates_from_upstream"] is True
        assert cfg["report_only"] is True
        assert mod.arm_deviates(arm) is True
        assert "偏离上游" in mod.arm_label(arm)
        assert "report-only" in mod.arm_label(arm)
    assert mod.arm_spec("C")["geom"] is False and mod.arm_spec("C")["brake"] is False
    assert mod.arm_spec("C")["escape"] is False
    assert mod.arm_spec("B")["escape"] is False
    assert mod.arm_spec("B")["sphere"] == (0.85, 8.0)
    assert mod.arm_spec("E")["sphere"][0] == pytest.approx(1.0)
    assert mod.arm_spec("E")["sphere"][1] == pytest.approx(8.0)
    assert mod.arm_spec("F")["sphere"] == (0.85, 16.0)
    assert mod.arm_spec("A")["sphere"] is None and mod.arm_spec("A")["escape"] is False
    assert mod.ESCAPE_TRIGGER_TICKS == 10
    assert mod.ESCAPE_HOLD_TICKS == 10
    assert mod.ESCAPE_FORWARD_M == pytest.approx(4.0)
    assert mod.ESCAPE_LATERAL_LO_M == pytest.approx(1.2)
    assert mod.ESCAPE_LATERAL_HI_M == pytest.approx(1.5)
    after = {path: path.read_bytes() for path in before}
    assert after == before
    assert "ego-density-014-swap6.json" not in {p.name for p in (ROOT / "scenes").glob("*.json")}


def test_open_sector_goal_is_bounded_forward_and_lateral():
    mod = _script()
    assert mod.horizon_rows(96) == (32, 64)
    position = np.array([0.0, 0.0, 1.5])
    straight = mod.temp_goal_from_depth(_blank_depth(), position, 0.0)
    fwd, lat = mod.world_offset_body(position, 0.0, straight)
    assert fwd == pytest.approx(4.0)
    assert lat == pytest.approx(1.2)
    assert straight[2] == pytest.approx(1.5)

    right = _blank_depth(1.0)
    right[:, 142:] = np.nan
    goal_r = mod.temp_goal_from_depth(right, position, 0.0)
    fwd_r, lat_r = mod.world_offset_body(position, 0.0, goal_r)
    assert fwd_r == pytest.approx(4.0)
    assert lat_r < 0.0
    assert abs(lat_r) == pytest.approx(1.2 + 0.3 * (40.0 / 45.0))
    assert mod.ESCAPE_LATERAL_LO_M <= abs(lat_r) <= mod.ESCAPE_LATERAL_HI_M

    left = _blank_depth(1.0)
    left[:, :17] = np.nan
    goal_l = mod.temp_goal_from_depth(left, position, 0.0)
    fwd_l, lat_l = mod.world_offset_body(position, 0.0, goal_l)
    assert fwd_l == pytest.approx(4.0)
    assert lat_l > 0.0
    assert lat_l == pytest.approx(-lat_r)

    yaw = np.pi / 2.0
    origin = np.array([1.0, 2.0, 1.5])
    turned = mod.temp_goal_from_depth(None, origin, yaw)
    assert turned == pytest.approx([-0.2, 6.0, 1.5])
    fwd_y, lat_y = mod.world_offset_body(origin, yaw, turned)
    assert fwd_y == pytest.approx(4.0)
    assert lat_y == pytest.approx(1.2)


def test_escape_tracker_arms_after_ten_and_returns_on_reopen():
    mod = _script()

    def make():
        return np.array([4.0, 1.2, 1.5]), np.array([4.0, 1.2, 0.0])

    tracker = mod.EscapeTracker()
    for _ in range(9):
        tracker.observe("CORRIDOR_EMPTY_HOVER", make)
    assert tracker.triggers == 0 and tracker.active() is False
    tracker.observe("OK", make)
    for _ in range(9):
        tracker.observe("CORRIDOR_EMPTY_HOVER", make)
    assert tracker.triggers == 0

    tracker.observe("CORRIDOR_EMPTY_HOVER", make)
    assert tracker.triggers == 1
    assert tracker.held == 0
    assert tracker.active() is True
    assert tracker.temp_goal == pytest.approx([4.0, 1.2, 1.5])

    tracker.observe("OK", make)
    assert tracker.reopened == 1
    assert tracker.held == 1
    assert tracker.active() is False
    tracker.observe("CORRIDOR_EMPTY_HOVER", make)
    assert tracker.triggers == 1
    assert tracker.streak == 1

    expired = mod.EscapeTracker()
    for _ in range(10):
        expired.observe("CORRIDOR_EMPTY_HOVER", make)
    for _ in range(10):
        expired.observe("CORRIDOR_EMPTY_HOVER", make)
    assert expired.held == 10
    assert expired.expired == 1
    assert expired.active() is False
    assert expired.triggers == 1


def test_wrapper_keeps_mission_goal_until_the_tenth_empty_plan():
    mod = _script()
    mod._reset_escape_stats()
    from yopo_l0.policy import Observation

    mission = np.array([8.0, 0.0, 1.5])
    state = State(0.0, np.array([0.0, 0.0, 1.5]), np.zeros(3), np.zeros(3), 0.0)
    frame = DepthFrame(0.0, _blank_depth(), (1.0, 1.0, 80.0, 48.0))
    codes = ["CORRIDOR_EMPTY_HOVER"] * 10 + ["OK"] + ["CORRIDOR_EMPTY_HOVER"]
    seen = []

    class Fake:
        ctx = type("Ctx", (), {"owner": "uav0", "params": {}})()
        p = {"arrive_radius_m": 1.5}
        arrived = False
        last_code = ""

        def orig(self, obs):
            seen.append(np.array(obs.goal, dtype=float).copy())
            self.last_code = codes.pop(0)
            if self.last_code == "ARRIVED_HOLD":
                self.arrived = True
            return "plan"

    fake = Fake()
    for _ in range(12):
        obs = Observation(frame, state, mission.copy(), 0.0)
        mod._plan_with_escape(fake, obs, Fake.orig)
    assert seen[0] == pytest.approx(mission)
    assert seen[9] == pytest.approx(mission)
    assert seen[10] == pytest.approx([4.0, 1.2, 1.5])
    assert seen[11] == pytest.approx(mission)
    stats = mod.escape_totals()
    assert stats["triggers"] == 1
    assert stats["held_ticks"] == 1
    assert stats["reopened"] == 1
    assert stats["by_drone"]["uav0"]["triggers"] == 1

    far = np.array([0.0, 0.0, 1.5])
    temp = np.array([4.0, 1.2, 1.5])
    aim = np.array([4.0, 1.2, 0.0])
    assert mod.planning_goal(far, temp, aim, 1.5) == pytest.approx(temp)
    close = np.array([3.6, 1.2, 1.5])
    pushed = mod.planning_goal(close, temp, aim, 1.5)
    assert float(np.linalg.norm(pushed - close)) == pytest.approx(2.0)
    assert float(np.linalg.norm(pushed - close)) > 1.5


def test_counts_and_partial_resume(tmp_path, monkeypatch):
    mod = _script()
    ticks = [
        {"uav": "uav0", "v": [3.0, 0.0, 0.0], "k": 0, "plan_code": "CORRIDOR_EMPTY_HOVER"},
        {"uav": "uav1", "v": [0.0, 0.0, 0.0], "k": 0, "plan_code": "OK"},
        {"uav": "uav0", "v": [float("nan"), 0.0, 0.0], "k": 1, "plan_code": "CORRIDOR_EMPTY_HOVER"},
        {"uav": "uav0", "v": [0.0, 4.0, 0.0], "k": 10, "plan_code": "CORRIDOR_EMPTY_HOVER"},
    ]
    assert mod.avg_speed_mps(ticks) == pytest.approx((3.0 + 4.0) / 2.0)
    assert mod.avg_speed_mps([]) is None
    assert mod.corridor_empty_hover(ticks) == 2
    assert mod._is_plan_tick(0) and mod._is_plan_tick(10) and not mod._is_plan_tick(1)

    monkeypatch.setattr(mod, "PARTIAL", tmp_path)
    stored = {
        "row_version": mod.ROW_VERSION,
        "scene": mod.SCENE,
        "arm": "D",
        "seed": 3,
        "outcome": "SUCCESS",
        "arrived": 6,
        "n_drones": 6,
        "avg_speed_mps": 1.25,
        "escape_triggers": 2,
    }
    mod.save_partial(stored)
    monkeypatch.setattr(mod, "_simulate", lambda *_a, **_k: (_ for _ in ()).throw(AssertionError("不应重跑")))
    got = mod.run_one("D", 3, resume=True)
    assert got["avg_speed_mps"] == pytest.approx(1.25)
    assert got["escape_triggers"] == 2
    fresh = dict(stored)
    fresh["avg_speed_mps"] = 2.5
    monkeypatch.setattr(mod, "_simulate", lambda *_a, **_k: fresh)
    assert mod.run_one("D", 3, resume=False)["avg_speed_mps"] == pytest.approx(2.5)
    stale = dict(stored)
    stale["row_version"] = mod.ROW_VERSION + 1
    stale["seed"] = 1
    mod.save_partial(stale)
    assert mod.load_partial("D", 1) is None


def test_check_grid_does_not_touch_upstream_files():
    mod = _script()
    before = {path: path.read_bytes() for path in _watched()}
    assert mod.ARMS == ("A", "B", "C", "D", "E", "F")
    assert mod.SEEDS == (0, 1, 2, 3, 4)
    assert len(mod.job_specs(mod.ARMS, mod.SEEDS)) == 30
    assert "report-only" in mod.DISCLAIMER and "偏离上游" in mod.DISCLAIMER
    assert mod.main(["--check"]) == 0
    with pytest.raises(SystemExit):
        mod.main(["--jobs", "5", "--seeds", "0"])
    import yopo_l0.runner as runner_mod
    from yopo_l0.backend.config import swarm_penalty_enabled as real_swarm
    from yopo_l0.policy import UpstreamPolicy

    # 别的研究脚本可能已经包过 runner。这里只要求本票装上之后能拆干净，回到装之前的那个函数。
    runner_before = runner_mod.swarm_penalty_enabled
    plan_before = UpstreamPolicy.plan
    try:
        mod.install_peer_delivery()
        mod.install_escape()
        sphere = {"backend": {"enabled": True, "peer_sphere_enabled": True}}
        assert runner_mod.swarm_penalty_enabled(sphere) is True
        assert real_swarm(sphere) is False
        after = {path: path.read_bytes() for path in before}
        assert after == before
    finally:
        mod.uninstall_peer_delivery()
        mod.uninstall_escape()
    assert UpstreamPolicy.plan is plan_before
    assert runner_mod.swarm_penalty_enabled is runner_before
    tickets = (ROOT / "docs/tickets.md").read_text(encoding="utf-8")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "E-C8b" not in tickets and "E-C8b" not in readme


def test_import_does_not_pin_blas_threads():
    code = """
import importlib.util
import os
import sys
from pathlib import Path
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.pop(key, None)
path = Path("scripts/e-c8b-ablation.py")
spec = importlib.util.spec_from_file_location("e_c8b_fresh", path)
mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)
assert os.environ.get("OMP_NUM_THREADS") is None, os.environ.get("OMP_NUM_THREADS")
"""
    proc = subprocess.run(
        [sys.executable, "-c", code],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr


def test_committed_runs_match_tables_when_present():
    path = ROOT / "docs/research/e-c8b-ablation-runs.json"
    md_path = ROOT / "docs/research/e-c8b-ablation.md"
    if not path.is_file() or not md_path.is_file():
        pytest.skip("报告还没写入")
    mod = _script()
    data = json.loads(path.read_text(encoding="utf-8"))
    rows = data["rows"]
    assert len(rows) == 30
    assert data["incomplete"] == 0
    assert data["report_only"] is True and data["frozen"] is False
    assert data["inter_drone_filter"] is False
    assert "偏离上游" in data["disclaimer"] and "report-only" in data["disclaimer"]
    assert data["geom_sha256"] == mod.GEOM_SHA256
    assert data["scene_source"] == "scenes/ego-density-014-swap3.json"
    assert len(data["swap6"]) == 6
    md = md_path.read_text(encoding="utf-8")
    assert mod.render_summary(rows) in md
    assert mod.render_seeds(rows) in md
    assert mod.render_placement() in md
    assert "只报数" in md and "偏离上游" in md and "report-only" in md
    for row in rows:
        assert row["arm"] in mod.ARMS
        assert row["seed"] in range(5)
        assert row["report_only"] is True and row["frozen"] is False
        assert row["deviation_mark"] == "偏离上游 / report-only"
        assert row["upstream_deviation"] is True
        assert row["inter_drone_filter"] is False
        assert float(row["safe_radius_m"]) == pytest.approx(0.05)
        spec = mod.arm_spec(row["arm"])
        assert row["empty_brake"] is spec["brake"]
        assert row["track_clearance"] is spec["geom"]
        assert row["escape_replan"] is spec["escape"]
        if spec["sphere"] is None:
            assert row["peer_sphere_enabled"] is False
        else:
            assert row["peer_sphere_radius_m"] == pytest.approx(spec["sphere"][0])
            assert row["peer_sphere_weight"] == pytest.approx(spec["sphere"][1])
        if not spec["escape"]:
            assert int(row["escape_triggers"]) == 0
            assert int(row["escape_held_ticks"]) == 0
        kinds = (
            int(row["pillar_collision"])
            + int(row["ground_collision"])
            + int(row["box_collision"])
            + int(row["inter_drone_collision"])
        )
        assert kinds <= 1
        assert f"{sum(int(r['arrived']) for r in rows if r['arm'] == row['arm'])}/30" in md
