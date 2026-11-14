"""E-B6：常规 M2/M4 套件上的欧氏球对照。只检查脚本，不改默认开关。"""

import importlib.util
import json
import sys
from types import SimpleNamespace

import pytest

from conftest import ROOT

DISCLAIMER_BITS = ("只报数", "report-only", "偏离上游")
SCENES = (
    "crossing-pillars",
    "crossing-forest",
    "crossing-6-forest",
    "crossing-6-forest-sparse",
)


def _script():
    name = "e_b6_suite_sphere"
    cached = sys.modules.get(name)
    if cached is not None and getattr(cached, "ARMS", None):
        return cached
    path = ROOT / "scripts/e-b6-suite-sphere.py"
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def _src() -> str:
    return (ROOT / "scripts/e-b6-suite-sphere.py").read_text(encoding="utf-8")


def test_suite_defaults_and_does_not_touch_frozen_files():
    mod = _script()
    src = _src()
    assert mod.ARMS == ("baseline", "b4c", "b5-radius", "b5-weight")
    assert mod.ARM_SPEC["b4c"] == {"radius_m": 0.85, "weight": 8.0, "level": "E-B4c 默认球"}
    assert mod.ARM_SPEC["b5-radius"]["radius_m"] == pytest.approx(1.0)
    assert mod.ARM_SPEC["b5-radius"]["weight"] == pytest.approx(8.0)
    assert mod.ARM_SPEC["b5-weight"]["radius_m"] == pytest.approx(0.85)
    assert mod.ARM_SPEC["b5-weight"]["weight"] == pytest.approx(16.0)
    assert mod.SCENE_SEEDS["crossing-pillars"] == tuple(range(10))
    assert mod.SCENE_SEEDS["crossing-forest"] == tuple(range(10))
    assert mod.SCENE_SEEDS["crossing-6-forest"] == tuple(range(5))
    assert mod.SCENE_SEEDS["crossing-6-forest-sparse"] == tuple(range(5))
    assert mod.SCENE_TIER["crossing-pillars"] == "M2"
    assert mod.SCENE_TIER["crossing-6-forest-sparse"] == "M4"
    for bit in DISCLAIMER_BITS:
        assert bit in mod.DISCLAIMER
    assert mod.DEVIATION_LABEL == "report-only / 偏离上游"
    assert "不改" in src
    for name in ("docs/tickets.md", "README.md", "scripts/premerge.sh", "scripts/regress.sh"):
        assert name in src
    suite = (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    tickets = (ROOT / "docs/tickets.md").read_text(encoding="utf-8")
    assert "peer_sphere" not in suite
    assert "e-b6" not in tickets
    assert "e-b6" not in (ROOT / "README.md").read_text(encoding="utf-8")


def test_cards_stay_on_disk_and_only_sphere_arms_deviate():
    mod = _script()
    from yopo_l0.backend.config import peer_sphere_enabled

    before = {name: (ROOT / mod.SCENE_PATH[name]).read_text(encoding="utf-8") for name in SCENES}
    for scene in SCENES:
        base = mod.build_card(scene, "baseline")
        raw_disk = json.loads(before[scene])
        assert "peer_sphere" not in json.dumps(base.card)
        assert peer_sphere_enabled(base.card.get("policy_params")) is False
        assert base.card.get("criteria") == json.loads(json.dumps(raw_disk["criteria"]))
        hashes = {base.hash}
        for arm, spec in mod.ARM_SPEC.items():
            card = mod.build_card(scene, arm)
            backend = card.card["policy_params"]["backend"]
            assert backend["enabled"] is True
            assert backend["peer_sphere_enabled"] is True
            assert backend["peer_sphere_radius_m"] == pytest.approx(spec["radius_m"])
            assert backend["peer_sphere_weight"] == pytest.approx(spec["weight"])
            assert "swarm_penalty" not in backend
            assert "swarm_segment_penalty" not in backend
            assert peer_sphere_enabled(card.card["policy_params"]) is True
            assert card.card["task"] == base.card["task"]
            assert card.card["criteria"] == base.card["criteria"]
            assert card.card["drones"] == base.card["drones"]
            assert card.hash not in hashes
            hashes.add(card.hash)
            assert mod.deviation_label(arm) == "report-only / 偏离上游"
            assert mod.sphere_on(arm) is True
        assert mod.sphere_on("baseline") is False
        assert mod.deviation_label("baseline") == "否"
        assert (ROOT / mod.SCENE_PATH[scene]).read_text(encoding="utf-8") == before[scene]


def test_same_scene_arms_share_seeds_and_jobs_stay_capped():
    mod = _script()
    jobs = mod.build_jobs(list(SCENES), list(mod.ARMS), None)
    by_scene: dict[str, set[int]] = {name: set() for name in SCENES}
    counts: dict[tuple[str, str], list[int]] = {}
    for scene, arm, seed in jobs:
        counts.setdefault((scene, arm), []).append(seed)
        by_scene[scene].add(seed)
    for scene in SCENES:
        expect = list(mod.SCENE_SEEDS[scene])
        for arm in mod.ARMS:
            assert counts[(scene, arm)] == expect
        assert sorted(by_scene[scene]) == expect
    overridden = mod.build_jobs(["crossing-pillars", "crossing-6-forest"], list(mod.ARMS), "0,1")
    for scene in ("crossing-pillars", "crossing-6-forest"):
        got = {arm: [seed for s, arm2, seed in overridden if s == scene and arm2 == arm] for arm in mod.ARMS}
        assert all(seeds == [0, 1] for seeds in got.values())
    with pytest.raises(SystemExit):
        mod.main(["--jobs-plain", "3", "--seeds", "0", "--scenes", "crossing-pillars", "--arms", "baseline"])
    with pytest.raises(SystemExit):
        mod.main(["--jobs-backend", "3", "--seeds", "0", "--scenes", "crossing-pillars", "--arms", "baseline"])
    src = _src()
    assert "YOPO_PEER_SPHERE_RADIUS_M" in src
    assert "committed_neighbor_samples" in src


def test_crossing_wrapper_feeds_same_tick_samples_only_when_sphere_or_swarm(monkeypatch):
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
        fleet = SimpleNamespace(
            _backend=SimpleNamespace(swarm_penalty=False, peer_sphere_enabled=True),
            _score_node=object(),
            _backend_obstacles=(),
            backend_log=[],
        )
        crossing.CrossingFleet._apply_backend(fleet, SimpleNamespace(t_start=1.25), "goal")
        assert calls == [sentinel]
        assert fleet.backend_log == [{"applied": True, "budget_cut": False}]

        calls.clear()
        off = SimpleNamespace(
            _backend=SimpleNamespace(swarm_penalty=False, peer_sphere_enabled=False),
            _score_node=object(),
            _backend_obstacles=(),
            backend_log=[],
        )
        crossing.CrossingFleet._apply_backend(off, SimpleNamespace(t_start=0.0), "goal")
        assert calls == [None]

        calls.clear()
        swarm = SimpleNamespace(
            _backend=SimpleNamespace(swarm_penalty=True, peer_sphere_enabled=False),
            _score_node=object(),
            _backend_obstacles=(),
            backend_log=[],
        )
        crossing.CrossingFleet._apply_backend(swarm, SimpleNamespace(t_start=0.5), "goal")
        assert calls == [sentinel]
    finally:
        mod.restore_peer_delivery()
    assert crossing.CrossingFleet._apply_backend is mod._ORIG["cross"]


def test_summary_table_has_the_eight_metrics_and_labels():
    mod = _script()
    rows = []
    for arm, radius, weight, deviate in (
        ("baseline", None, None, False),
        ("b4c", 0.85, 8.0, True),
        ("b5-radius", 1.0, 8.0, True),
        ("b5-weight", 0.85, 16.0, True),
    ):
        rows.append(
            {
                "scene_kind": "crossing-pillars",
                "tier": "M2",
                "arm": arm,
                "seed": 0,
                "arrived": 3,
                "n_drones": 3,
                "obstacle_collisions": 0,
                "inter_drone_collisions": 0,
                "timeout": 0,
                "formation_error_p95_m": 4.2,
                "min_inter_drone_distance_m": 3.5,
                "min_obstacle_clearance_m": 0.6,
                "arrival_time_s": 7.1,
                "deviates_from_upstream": deviate,
                "deviation_label": mod.deviation_label(arm),
                "peer_sphere_radius_m": radius,
                "peer_sphere_weight": weight,
            }
        )
    text = mod.format_summary(rows)
    for header in (
        "到达",
        "障碍碰撞",
        "机间碰撞",
        "超时",
        "队形误差 p95 m",
        "最小机间 m",
        "最小障碍净空 m",
        "平均到达时间 s",
    ):
        assert header in text
    assert "report-only / 偏离上游" in text
    assert "| crossing-pillars | baseline | 否 | 3/3 | 0 | 0 | 0 | 4.200 | 3.500 | 0.600 | 7.10 |" in text
    stats = mod.summarize_stats(rows)
    sphere = [item for item in stats if item["arm"] != "baseline"]
    assert sphere and all(item["zero_regression_vs_baseline"] for item in sphere)
    assert all(item["frozen_gates"]["passes_frozen_gates"] for item in stats)


def test_clear_sphere_env_drops_radius_override(monkeypatch):
    mod = _script()
    monkeypatch.setenv("YOPO_PEER_SPHERE", "1")
    monkeypatch.setenv("YOPO_PEER_SPHERE_RADIUS_M", "9")
    monkeypatch.setenv("YOPO_PEER_SPHERE_WEIGHT", "99")
    mod.clear_sphere_env()
    for key in mod.ENV_KEYS:
        assert key not in __import__("os").environ


def test_committed_runs_align_seeds_and_report_quotes_them():
    mod = _script()
    data = json.loads((ROOT / "docs/research/e-b6-suite-sphere-runs.json").read_text(encoding="utf-8"))
    report = (ROOT / "docs/research/e-b6-suite-sphere.md").read_text(encoding="utf-8")
    assert data["report_only"] is True
    assert data["incomplete"] == 0
    assert data["completed"] == data["expected"]
    assert data["deviation_label"] == "report-only / 偏离上游"
    assert "偏离上游" in report and "report-only" in report
    rows = data["rows"]
    for scene, seeds in data["seeds_by_scene"].items():
        assert seeds == list(mod.SCENE_SEEDS[scene])
        assert scene in report
        for arm in mod.ARMS:
            got = [row for row in rows if row["scene_kind"] == scene and row["arm"] == arm]
            assert sorted(int(row["seed"]) for row in got) == list(seeds)
            for row in got:
                assert row["report_only"] is True
                for key in (
                    "arrived",
                    "obstacle_collisions",
                    "inter_drone_collisions",
                    "timeout",
                    "formation_error_p95_m",
                    "min_inter_drone_distance_m",
                    "min_obstacle_clearance_m",
                    "arrival_time_s",
                ):
                    assert key in row
                if arm == "baseline":
                    assert row["deviates_from_upstream"] is False
                    assert row["peer_sphere_enabled"] is False
                    assert row["deviation_label"] == "否"
                else:
                    assert row["deviates_from_upstream"] is True
                    assert row["deviation_label"] == "report-only / 偏离上游"
                    assert row["peer_sphere_enabled"] is True
                    assert row["peer_sphere_radius_m"] == pytest.approx(mod.ARM_SPEC[arm]["radius_m"])
                    assert row["peer_sphere_weight"] == pytest.approx(mod.ARM_SPEC[arm]["weight"])
    table = mod.format_summary(rows)
    for line in table.splitlines():
        if line.startswith("| crossing"):
            assert line in report
    by = {(item["scene"], item["arm"]): item for item in data["summary"]}
    assert by[("crossing-pillars", "baseline")]["frozen_gates"]["passes_frozen_gates"] is True
    assert by[("crossing-pillars", "b4c")]["zero_regression_vs_baseline"] is False
    assert by[("crossing-pillars", "b5-radius")]["zero_regression_vs_baseline"] is False
    assert by[("crossing-pillars", "b5-weight")]["zero_regression_vs_baseline"] is False
    for scene in ("crossing-forest", "crossing-6-forest", "crossing-6-forest-sparse"):
        for arm in mod.ARMS:
            assert by[(scene, arm)]["frozen_gates"]["passes_frozen_gates"] is True
            assert by[(scene, arm)]["obstacle_collisions"] == 0
            assert by[(scene, arm)]["inter_drone_collisions"] == 0
            assert by[(scene, arm)]["timeout"] == 0
