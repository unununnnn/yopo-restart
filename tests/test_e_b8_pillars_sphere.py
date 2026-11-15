"""E-B8：柱林 seed 3 / 6 开球越门归因。只检查脚本和已提交的报数，不改默认开关。"""

import importlib.util
import json
import sys

import pytest

from conftest import ROOT

DISCLAIMER_BITS = ("只报数", "report-only", "偏离上游")
ARMS = ("baseline", "bspline_only", "b4c", "b4c-no-obstacle")


def _script():
    name = "e_b8_pillars_sphere"
    cached = sys.modules.get(name)
    if cached is not None and getattr(cached, "ARMS", None):
        return cached
    path = ROOT / "scripts/e-b8-pillars-sphere.py"
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def _src() -> str:
    return (ROOT / "scripts/e-b8-pillars-sphere.py").read_text(encoding="utf-8")


def _arm_row(
    *,
    crossed: bool,
    door_min: float,
    hinge: int = 0,
    cuts: int = 0,
) -> dict:
    return {
        "door_crossed": crossed,
        "door_min": door_min,
        "sphere_hinge_applied": hinge,
        "backend_budget_cuts": cuts,
    }


def _quad(*, base, smooth, sphere, naked) -> dict:
    return {
        "baseline": base,
        "bspline_only": smooth,
        "b4c": sphere,
        "b4c-no-obstacle": naked,
    }


def test_defaults_name_the_two_seeds_and_leave_frozen_files_alone():
    mod = _script()
    src = _src()
    assert mod.ARMS == ARMS
    assert mod.SEEDS == (3, 6)
    assert mod.DOOR_OF == {3: "clearance", 6: "inter"}
    assert mod.CLEARANCE_DOOR_M == pytest.approx(0.40)
    assert mod.INTER_DOOR_M == pytest.approx(2.0)
    assert mod.WINDOW_S == pytest.approx(1.0)
    assert mod.SPHERE["peer_sphere_radius_m"] == pytest.approx(0.85)
    assert mod.SPHERE["peer_sphere_weight"] == pytest.approx(8.0)
    assert mod.DEVIATION_LABEL == "report-only / 偏离上游"
    for bit in DISCLAIMER_BITS:
        assert bit in mod.DISCLAIMER
    for name in ("docs/tickets.md", "README.md", "scripts/premerge.sh", "scripts/regress.sh", "yopo_l0/", "scenes/"):
        assert name in src
    assert "e-b8" not in (ROOT / "docs/tickets.md").read_text(encoding="utf-8")
    assert "e-b8" not in (ROOT / "README.md").read_text(encoding="utf-8")
    assert "e-b8" not in (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    with pytest.raises(SystemExit):
        mod.main(["--jobs", "2", "--summarize-only"])


def test_cards_stay_on_disk_and_rear_opens_without_the_sphere():
    mod = _script()
    from yopo_l0.backend.config import config_from_params, peer_sphere_enabled

    disk = ROOT / mod.SCENE_PATH
    before = disk.read_text(encoding="utf-8")
    base = mod.build_card("baseline")
    assert "peer_sphere" not in json.dumps(base.card)
    assert "backend" not in (base.card.get("policy_params") or {})
    assert peer_sphere_enabled(base.card.get("policy_params")) is False
    assert mod.deviation_label("baseline") == "否"
    assert mod.sphere_on("baseline") is False
    assert mod.backend_on("baseline") is False

    rear = mod.build_card("bspline_only")
    backend = rear.card["policy_params"]["backend"]
    assert backend["enabled"] is True
    assert "peer_sphere_enabled" not in backend
    assert peer_sphere_enabled(rear.card["policy_params"]) is False
    cfg = config_from_params(rear.card["policy_params"], a_max=6.0, clearance_m=0.35)
    assert cfg is not None and cfg.peer_sphere_enabled is False
    assert mod.deviation_label("bspline_only") == "report-only / 偏离上游"
    assert rear.card["criteria"] == base.card["criteria"]
    assert rear.card["task"] == base.card["task"]

    sphere = mod.build_card("b4c")
    naked = mod.build_card("b4c-no-obstacle")
    assert sphere.card == naked.card
    block = sphere.card["policy_params"]["backend"]
    assert block["enabled"] is True
    assert block["peer_sphere_enabled"] is True
    assert block["peer_sphere_radius_m"] == pytest.approx(0.85)
    assert block["peer_sphere_weight"] == pytest.approx(8.0)
    assert "swarm_penalty" not in block
    assert peer_sphere_enabled(sphere.card["policy_params"]) is True
    assert mod.obstacle_term_on("b4c") is True
    assert mod.obstacle_term_on("b4c-no-obstacle") is False
    assert mod.sphere_on("b4c-no-obstacle") is True
    assert disk.read_text(encoding="utf-8") == before


def test_obstacle_weight_returns_to_the_value_it_found():
    mod = _script()
    import yopo_l0.backend.bspline as bspline

    original = float(bspline._LAMBDA_C)
    assert original == pytest.approx(40.0)
    try:
        prev = mod.push_obstacle_term(False)
        assert prev == pytest.approx(original)
        assert bspline._LAMBDA_C == pytest.approx(0.0)
        try:
            raise RuntimeError("臂中途失败")
        except RuntimeError:
            mod.pop_obstacle_term(prev)
        assert bspline._LAMBDA_C == pytest.approx(original)
        mod.push_obstacle_term(True)
        assert bspline._LAMBDA_C == pytest.approx(original)
    finally:
        bspline._LAMBDA_C = original


def test_crossing_wrapper_feeds_neighbors_only_for_the_sphere(monkeypatch):
    mod = _script()
    import yopo_l0.backend.bspline as bspline
    import yopo_l0.crossing as crossing
    import yopo_l0.inter_drone as inter

    sentinel = [{"peer": True}]
    calls = []

    def fake_refine(traj, cfg, obstacles, goal, neighbors=None):
        calls.append(neighbors)
        return traj, {"applied": True, "budget_cut": False}

    monkeypatch.setattr(bspline, "refine_yopo", fake_refine)
    monkeypatch.setattr(inter, "committed_neighbor_samples", lambda node, t0: sentinel)
    mod.install_peer_delivery()
    try:
        fleet = type("F", (), {})()
        fleet._backend = type("C", (), {"swarm_penalty": False, "peer_sphere_enabled": True})()
        fleet._score_node = object()
        fleet._backend_obstacles = ()
        fleet.backend_log = []
        crossing.CrossingFleet._apply_backend(fleet, type("T", (), {"t_start": 1.25})(), "goal")
        assert calls == [sentinel]

        calls.clear()
        fleet._backend.peer_sphere_enabled = False
        crossing.CrossingFleet._apply_backend(fleet, type("T", (), {"t_start": 0.0})(), "goal")
        assert calls == [None]
    finally:
        mod.restore_peer_delivery()
    assert crossing.CrossingFleet._apply_backend is mod._PEER_ORIG["cross"]


def test_window_is_one_second_either_side_and_keeps_the_tick_fields():
    mod = _script()
    frames = []
    for i in range(0, 800):
        t = (i + 1) * 0.01
        frames.append(
            {
                "k": i,
                "t": t,
                "truth_clearance_m": 0.50 if t < 3.0 else 0.30,
                "truth_inter_m": 3.0 if t < 4.0 else 1.2,
                "truth_inter_pair": "uav0-uav1",
                "drones": [
                    {
                        "uav": "uav0",
                        "plan_code": "OK",
                        "ref_clearance_m": 0.42,
                        "ref_peer_m": 2.5,
                        "budget_cut": None,
                        "sphere_hinge": None,
                    }
                ],
            }
        )
    event = mod.choose_event(frames, "clearance", 0.40, 0.30)
    assert event["crossed"] is True
    assert event["event_t"] == pytest.approx(3.0)
    window = mod.slice_window(frames, event["event_t"])
    assert window[0]["t"] == pytest.approx(2.0)
    assert window[-1]["t"] == pytest.approx(4.0)
    assert len(window) == 201
    held_frames = [
        {"t": 0.01, "truth_clearance_m": 0.80, "truth_inter_m": 4.0},
        {"t": 2.50, "truth_clearance_m": 0.51, "truth_inter_m": 4.0},
        {"t": 5.00, "truth_clearance_m": 0.70, "truth_inter_m": 4.0},
    ]
    held = mod.choose_event(held_frames, "clearance", 0.40, 0.51)
    assert held["crossed"] is False
    assert held["event_t"] == pytest.approx(2.50)
    missed = mod.slice_window(frames, None)
    assert missed == []


def test_refine_flags_follow_the_last_applied_call():
    mod = _script()
    frames = [
        {
            "k": 0,
            "t": 1.0,
            "drones": [{"uav": "uav0"}],
        },
        {
            "k": 1,
            "t": 2.0,
            "drones": [{"uav": "uav0"}],
        },
    ]
    events = [
        {"drone": "uav0", "t_sim": 0.5, "t_start": 0.5, "applied": True, "budget_cut": False, "sphere_hinge": False, "obstacle_hinge": False},
        {"drone": "uav0", "t_sim": 1.5, "t_start": 1.5, "applied": False, "budget_cut": True, "sphere_hinge": True, "obstacle_hinge": True},
        {"drone": "uav0", "t_sim": 1.8, "t_start": 1.8, "applied": True, "budget_cut": True, "sphere_hinge": True, "obstacle_hinge": False},
    ]
    mod.attach_refine(frames, events)
    assert frames[0]["drones"][0]["budget_cut"] is False
    assert frames[0]["drones"][0]["sphere_hinge"] is False
    assert frames[1]["drones"][0]["budget_cut"] is True
    assert frames[1]["drones"][0]["sphere_hinge"] is True
    assert frames[1]["drones"][0]["refine_t"] == pytest.approx(1.8)


def test_attribution_names_rear_sphere_budget_and_obstacle_shift():
    mod = _script()
    rear = mod.attribute_seed(
        _quad(
            base=_arm_row(crossed=False, door_min=0.90),
            smooth=_arm_row(crossed=True, door_min=0.365),
            sphere=_arm_row(crossed=True, door_min=0.365),
            naked=_arm_row(crossed=True, door_min=0.365),
        )
    )
    assert rear["cause"] == "rear"
    assert rear["cause_zh"] == "越门来自后段平滑。"
    assert rear["obstacle_moves"] is False
    assert rear["bspline_matches_b4c"] is True

    shifted = mod.attribute_seed(
        _quad(
            base=_arm_row(crossed=False, door_min=0.90),
            smooth=_arm_row(crossed=True, door_min=0.365),
            sphere=_arm_row(crossed=True, door_min=0.365),
            naked=_arm_row(crossed=True, door_min=0.300),
        )
    )
    assert shifted["cause"] == "rear"
    assert shifted["obstacle_moves"] is True

    sphere = mod.attribute_seed(
        _quad(
            base=_arm_row(crossed=False, door_min=3.5),
            smooth=_arm_row(crossed=False, door_min=2.4),
            sphere=_arm_row(crossed=True, door_min=1.2, hinge=4),
            naked=_arm_row(crossed=True, door_min=1.2, hinge=4),
        )
    )
    assert sphere["cause"] == "sphere"
    assert sphere["cause_zh"] == "越门来自球项。"

    budget = mod.attribute_seed(
        _quad(
            base=_arm_row(crossed=False, door_min=3.5),
            smooth=_arm_row(crossed=False, door_min=2.4, cuts=0),
            sphere=_arm_row(crossed=True, door_min=1.2, hinge=0, cuts=2),
            naked=_arm_row(crossed=True, door_min=1.2, hinge=0, cuts=2),
        )
    )
    assert budget["cause"] == "budget"

    mixed = mod.attribute_seed(
        _quad(
            base=_arm_row(crossed=False, door_min=0.90),
            smooth=_arm_row(crossed=True, door_min=0.365),
            sphere=_arm_row(crossed=True, door_min=0.300, hinge=2),
            naked=_arm_row(crossed=True, door_min=0.300, hinge=2),
        )
    )
    assert mixed["cause"] == "mixed"

    quiet = mod.attribute_seed(
        _quad(
            base=_arm_row(crossed=False, door_min=0.90),
            smooth=_arm_row(crossed=False, door_min=0.80),
            sphere=_arm_row(crossed=False, door_min=0.70),
            naked=_arm_row(crossed=False, door_min=0.70),
        )
    )
    assert quiet["cause"] == "none"

    unclear = mod.attribute_seed(
        _quad(
            base=_arm_row(crossed=False, door_min=0.90),
            smooth=_arm_row(crossed=False, door_min=0.50, cuts=0),
            sphere=_arm_row(crossed=True, door_min=0.30, hinge=0, cuts=0),
            naked=_arm_row(crossed=False, door_min=0.50, cuts=0),
        )
    )
    assert unclear["cause"] == "unclear"
    assert unclear["obstacle_moves"] is True


def test_committed_runs_quote_the_window_and_the_conclusion():
    mod = _script()
    path = ROOT / "docs/research/e-b8-pillars-sphere-runs.json"
    doc_path = ROOT / "docs/research/e-b8-pillars-sphere.md"
    data = json.loads(path.read_text(encoding="utf-8"))
    report = doc_path.read_text(encoding="utf-8")
    assert data["report_only"] is True
    assert data["incomplete"] == 0
    assert data["completed"] == data["expected"] == 8
    assert data["deviation_label"] == "report-only / 偏离上游"
    assert data["window_s"] == pytest.approx(1.0)
    assert "report-only / 偏离上游" in report
    assert "只报数" in report
    rendered = mod.render_markdown(data)
    # 原文仍是这次提交记录的渲染结果。L0-DET 重跑只加在文末，不改原结论段。
    assert report.startswith(rendered)
    assert "\n## L0-DET 确定性重跑（2026-11-15）\n" in report
    rows = data["rows"]
    assert [(int(row["seed"]), row["arm"]) for row in rows] == [
        (seed, arm) for seed in mod.SEEDS for arm in mod.ARMS
    ]
    for row in rows:
        assert row["report_only"] is True
        assert row["scene_kind"] == "crossing-pillars"
        kind, limit = mod.door_limit(int(row["seed"]))
        assert row["door"] == kind
        assert row["door_limit_m"] == pytest.approx(limit)
        assert row["deviation_label"] == mod.deviation_label(row["arm"])
        if row["arm"] == "baseline":
            assert row["deviates_from_upstream"] is False
            assert row["peer_sphere_enabled"] is False
            assert row["backend_enabled"] is False
            assert row["obstacle_term"] is True
        else:
            assert row["deviates_from_upstream"] is True
            assert row["deviation_label"] == "report-only / 偏离上游"
            assert row["backend_enabled"] is True
        if row["arm"] in ("b4c", "b4c-no-obstacle"):
            assert row["peer_sphere_enabled"] is True
            assert row["peer_sphere_radius_m"] == pytest.approx(0.85)
            assert row["peer_sphere_weight"] == pytest.approx(8.0)
        if row["arm"] == "b4c-no-obstacle":
            assert row["obstacle_term"] is False
        if row["arm"] == "bspline_only":
            assert row["peer_sphere_enabled"] is False
            assert row["obstacle_term"] is True
        assert row["window_coverage"] in {"full", "start", "end", "both"}
        times = [float(frame["t"]) for frame in row["window"]]
        assert times
        assert times[0] >= float(row["event_t"]) - 1.0 - 1e-6
        assert times[-1] <= float(row["event_t"]) + 1.0 + 1e-6
        assert any(abs(float(frame["t"]) - float(row["event_t"])) <= 0.01 + 1e-6 for frame in row["window"]) or float(row["event_t"]) < times[0]
        if row["window_coverage"] == "full":
            assert abs((times[-1] - times[0]) - 2.0) < 0.02
        for left, right in zip(times, times[1:]):
            assert abs((right - left) - 0.01) < 1e-6
        for frame in row["window"]:
            assert {drone["uav"] for drone in frame["drones"]} == {"uav0", "uav1", "uav2"}
            for drone in frame["drones"]:
                for key in ("plan_code", "budget_cut", "sphere_hinge", "ref_clearance_m", "ref_peer_m", "ref_obstacle_m"):
                    assert key in drone
                if row["arm"] == "baseline":
                    assert drone["budget_cut"] is None
                    assert drone["sphere_hinge"] is None
                else:
                    assert drone["budget_cut"] is None or isinstance(drone["budget_cut"], bool)
                    assert drone["sphere_hinge"] is None or isinstance(drone["sphere_hinge"], bool)
        summary = row["window_summary"]
        assert summary["n_ticks"] == len(row["window"])
        assert "plan_codes" in summary
        if row.get("tight_t") is not None and abs(float(row["tight_t"]) - float(row["event_t"])) > 1.0 + 1e-6:
            tight = row.get("tight_window") or []
            assert tight
            assert any(abs(float(frame["t"]) - float(row["tight_t"])) <= 0.011 for frame in tight)
    assert data["attribution"] == mod.attribute_rows(rows)
    for seed, block in data["attribution"].items():
        assert block["cause"] in mod.CAUSE_ZH
        assert block["cause_zh"] in report
        assert f"seed {seed}" in report
    for line in mod.format_outcome_table(rows).splitlines():
        if line.startswith("| 3") or line.startswith("| 6") or line.startswith("| seed"):
            assert line in report
    for line in mod.format_window_table(rows).splitlines():
        if line.startswith("| 3") or line.startswith("| 6"):
            assert line in report
