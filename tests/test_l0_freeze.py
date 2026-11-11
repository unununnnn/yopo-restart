"""票 11d：冻结判据失败时批跑退出非 0；只报数的失败不挡退出码。"""

from yopo_l0.batch import main as batch_main
from yopo_l0.criteria import active_gates, evaluate_gates
from yopo_l0.scene import load_card

from conftest import ROOT


def _base(job, **extra):
    path, policy, seed, _duration, _out, check, _shadow, form, _bus, tier_id, _wall_cap, _safe, *_rest = job
    card = load_card(path)
    task = card.card["task"]["type"]
    n = len(card.card["drones"])
    density = (card.card.get("forest") or {}).get("density")
    row = {
        "commit": "abc123def4567890",
        "scene": card.name,
        "scene_card_hash": card.hash,
        "seed": seed,
        "tier": "L0",
        "policy": policy if task not in ("crossing", "formation", "switch") else task,
        "shadow": True,
        "task": task,
        "outcome": "SUCCESS",
        "reason": "GOAL_REACHED_AND_HELD",
        "n_drones": n,
        "arrived": n,
        "arrival_rate": 1.0,
        "arrival_time_s": 4.27 if task == "goto" else 20.0,
        "min_obstacle_distance_m": 0.83,
        "min_inter_drone_distance_m": 3.0,
        "final_goal_error_m": 0.01,
        "formation_error_p95_m": 0.03,
        "shadow_reject_rate": 1.0,
        "shadow_reject_histogram": {},
        "plan_codes": {},
        "sim_duration_s": 21.0,
        "wall_time_s": 5.0,
        "form": "ros2" if form == "ros2" else "inprocess",
        "fault_tier": tier_id,
        "forest_density": density,
        "crossing": {
            "frozen": True,
            "body_radius_m": 0.35,
            "obstacle_collisions": 0,
            "inter_drone_collisions": 0,
            "min_obstacle_clearance_m": 0.50,
        },
        "switch": {
            "frozen": True,
            "transition_collisions": 0,
            "min_inter_drone_transition_m": 1.54,
            "error_p95_before_m": 0.025,
            "error_p95_after_m": 0.025,
        },
        "formation": {"local_commits": {f"uav{i}": 1 for i in range(n)}, "frozen": True},
    }
    row.update(extra)
    return row, True if check else None


def _run(tmp_path, monkeypatch, specs, extra=None, **job_overrides):
    def fake(job):
        return _base(job, **(job_overrides if not callable(job_overrides) else {}))

    if callable(extra):
        def fake(job):
            row, same = _base(job)
            extra(row, job)
            return row, same
    elif extra:
        def fake(job):
            row, same = _base(job)
            row.update(extra)
            return row, same

    monkeypatch.setattr("yopo_l0.batch._one", fake)
    out = tmp_path / "freeze"
    args = list(specs) + ["--no-determinism", "--out", str(out)]
    return batch_main(args), out


def test_scene_cards_carry_the_approved_gates():
    pillar = load_card(ROOT / "scenes/g2-pillar.json")
    assert active_gates(pillar, policy="upstream") == {
        "arrival_rate": 1.0, "collisions": 0.0, "arrival_time_s": 5.0,
    }
    assert active_gates(pillar, policy="certified") is None

    hold = load_card(ROOT / "scenes/formation-hold.json")
    assert active_gates(hold, policy="formation") == {
        "arrival_rate": 1.0, "collisions": 0.0,
        "formation_error_p95_m": 0.05, "min_inter_drone_distance_m": 1.50,
    }

    pillars = load_card(ROOT / "scenes/crossing-pillars.json")
    assert active_gates(pillars, policy="crossing")["formation_error_p95_m"] == 6.0
    assert active_gates(pillars, policy="crossing")["min_obstacle_clearance_m"] == 0.40

    forest = load_card(ROOT / "scenes/crossing-forest.json")
    medium = active_gates(forest, policy="crossing", density="medium")
    assert medium["formation_error_p95_m"] == 6.0
    assert "min_obstacle_clearance_m" not in medium
    assert active_gates(forest, policy="crossing", density="dense") is None
    assert active_gates(forest, policy="crossing", density="sparse") is None

    six_open = load_card(ROOT / "scenes/formation-6-open.json")
    assert active_gates(six_open, policy="formation", form="inprocess")["formation_error_p95_m"] == 0.10
    assert active_gates(six_open, policy="formation", form="ros2") is None

    switch = load_card(ROOT / "scenes/formation-switch.json")
    gates = active_gates(switch, policy="switch")
    assert gates["error_p95_before_m"] == 0.05 and gates["min_inter_drone_transition_m"] == 1.20
    assert gates["transition_collisions"] == 0.0

    for rel in ("scenes/crossing-6-forest.json", "scenes/crossing-6-forest-sparse.json"):
        card = load_card(ROOT / rel)
        gates = active_gates(card, policy="crossing")
        assert gates == {"arrival_rate": 1.0, "collisions": 0.0, "min_inter_drone_distance_m": 0.70}
        assert "formation_error_p95_m" not in gates
        assert "min_obstacle_clearance_m" not in gates

    drop = load_card(ROOT / "scenes/formation-drop.json")
    assert active_gates(drop, policy="formation") is None


def test_evaluate_gates_reports_a_miss_and_passes_a_hit():
    ok = {"seed": 0, "arrived": 3, "n_drones": 3, "arrival_rate": 1.0, "outcome": "SUCCESS",
          "formation_error_p95_m": 4.818, "min_inter_drone_distance_m": 4.6}
    assert evaluate_gates([ok], {"arrival_rate": 1.0, "formation_error_p95_m": 6.0, "min_inter_drone_distance_m": 2.0}) == []
    miss = dict(ok, formation_error_p95_m=6.001)
    msgs = evaluate_gates([miss], {"formation_error_p95_m": 6.0})
    assert msgs and "formation_error_p95_m" in msgs[0]


def test_frozen_arrival_miss_exits_nonzero(tmp_path, monkeypatch):
    rc, out = _run(
        tmp_path, monkeypatch, ["scenes/g2-pillar.json@upstream"],
        extra={"arrived": 0, "arrival_rate": 0.0, "arrival_time_s": None, "outcome": "TIMEOUT", "reason": "TIMEOUT"},
    )
    assert rc == 1
    text = (out / "summary.md").read_text()
    assert "冻结判据未通过" in text
    assert "arrival_rate" in text


def test_report_only_arrival_miss_exits_zero(tmp_path, monkeypatch):
    rc, out = _run(
        tmp_path, monkeypatch, ["scenes/g2-pillar.json@certified"],
        extra={"arrived": 0, "arrival_rate": 0.0, "arrival_time_s": None, "outcome": "TIMEOUT", "reason": "TIMEOUT"},
    )
    assert rc == 0
    text = (out / "summary.md").read_text()
    assert "冻结判据未通过" not in text
    assert "未冻结 / report-only" in text


def test_report_only_collision_on_unfrozen_card_exits_zero(tmp_path, monkeypatch):
    rc, _ = _run(
        tmp_path, monkeypatch, ["scenes/formation-drop.json@formation"],
        extra={"outcome": "COLLISION", "reason": "uav0:TRUTH_POSITION", "arrived": 0, "arrival_rate": 0.0},
    )
    assert rc == 0


def test_short_duration_does_not_apply_frozen_gates(tmp_path, monkeypatch):
    def fake(job):
        row, same = _base(job)
        row.update(arrived=0, arrival_rate=0.0, arrival_time_s=None, outcome="TIMEOUT")
        return row, same

    monkeypatch.setattr("yopo_l0.batch._one", fake)
    rc = batch_main([
        "scenes/crossing-pillars.json@crossing",
        "--seeds", "0", "--duration", "0.3", "--no-determinism",
        "--out", str(tmp_path / "short"),
    ])
    assert rc == 0


def test_m2_formation_gate_is_six_meters(tmp_path, monkeypatch):
    rc_ok, _ = _run(
        tmp_path, monkeypatch, ["scenes/crossing-pillars.json@crossing", "--seeds", "0"],
        extra={"formation_error_p95_m": 6.0, "min_obstacle_distance_m": 0.75,
               "crossing": {"frozen": True, "body_radius_m": 0.35, "obstacle_collisions": 0,
                            "inter_drone_collisions": 0, "min_obstacle_clearance_m": 0.40}},
    )
    assert rc_ok == 0
    rc_bad, out = _run(
        tmp_path, monkeypatch, ["scenes/crossing-pillars.json@crossing", "--seeds", "0"],
        extra={"formation_error_p95_m": 6.001, "min_obstacle_distance_m": 0.75,
               "crossing": {"frozen": True, "body_radius_m": 0.35, "obstacle_collisions": 0,
                            "inter_drone_collisions": 0, "min_obstacle_clearance_m": 0.40}},
    )
    assert rc_bad == 1
    assert "formation_error_p95_m" in (out / "summary.md").read_text()


def test_six_forest_p95_stays_report_only_but_min_gap_is_frozen(tmp_path, monkeypatch):
    wide = {
        "formation_error_p95_m": 26.0,
        "min_inter_drone_distance_m": 0.70,
        "crossing": {
            "frozen": True, "body_radius_m": 0.35,
            "obstacle_collisions": 0, "inter_drone_collisions": 0,
            "min_obstacle_clearance_m": 0.05,
        },
    }
    rc, _ = _run(tmp_path, monkeypatch, ["scenes/crossing-6-forest.json@crossing", "--seeds", "0"], extra=wide)
    assert rc == 0
    tight = dict(wide, min_inter_drone_distance_m=0.699)
    rc_bad, out = _run(
        tmp_path, monkeypatch, ["scenes/crossing-6-forest.json@crossing", "--seeds", "0"], extra=tight,
    )
    assert rc_bad == 1
    assert "min_inter_drone_distance_m" in (out / "summary.md").read_text()


def test_ros2_form_stays_report_only(tmp_path, monkeypatch):
    rc, out = _run(
        tmp_path, monkeypatch,
        ["scenes/formation-6-open.json", "--form", "ros2", "--tiers", "none", "--seeds", "0"],
        extra={"arrived": 0, "arrival_rate": 0.0, "formation_error_p95_m": 9.0, "outcome": "TIMEOUT"},
    )
    assert rc == 0
    assert "未冻结 / report-only" in (out / "summary.md").read_text()


def test_inprocess_m4_p95_gate_blocks(tmp_path, monkeypatch):
    rc, out = _run(
        tmp_path, monkeypatch,
        ["scenes/formation-6-open.json", "--form", "inprocess", "--tiers", "none", "--seeds", "0"],
        extra={"formation_error_p95_m": 0.101, "min_inter_drone_distance_m": 2.98},
    )
    assert rc == 1
    assert "formation_error_p95_m" in (out / "summary.md").read_text()


def test_three_forest_dense_stays_report_only(tmp_path, monkeypatch):
    def tweak(row, job):
        row["forest_density"] = "dense"
        row["arrived"] = 0
        row["arrival_rate"] = 0.0
        row["outcome"] = "COLLISION"
        row["crossing"] = {
            "frozen": False, "body_radius_m": 0.35,
            "obstacle_collisions": 1, "inter_drone_collisions": 0,
            "min_obstacle_clearance_m": -0.1,
        }

    rc, _ = _run(tmp_path, monkeypatch, ["scenes/crossing-forest.json@crossing", "--seeds", "0"], extra=tweak)
    assert rc == 0
