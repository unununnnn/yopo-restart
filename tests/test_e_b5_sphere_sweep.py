"""E-B5：参数扫描只改进程内的卡和包装，不改库，也不把六机卡写进 scenes/。"""

import importlib.util
import sys

import numpy as np
import pytest

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-b5-sphere-sweep.py"
    spec = importlib.util.spec_from_file_location("e_b5_sphere_sweep", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _watched():
    return [
        ROOT / "yopo_l0/backend/config.py",
        ROOT / "yopo_l0/backend/bspline.py",
        ROOT / "yopo_l0/policy.py",
        ROOT / "yopo_l0/geom_fallback.py",
        ROOT / "yopo_l0/inter_drone.py",
        ROOT / "yopo_l0/runner.py",
        ROOT / "scenes/ego-density-014-swap3.json",
        ROOT / "scenes/swap-3-open.json",
        ROOT / "scenes/ego-density-014.json",
        ROOT / "scenes/suite.txt",
        ROOT / "docs/tickets.md",
        ROOT / "README.md",
        ROOT / "scripts/premerge.sh",
        ROOT / "scripts/regress.sh",
    ]


def test_disclaimer_is_report_only_and_marks_deviation():
    mod = _script()
    text = mod.DISCLAIMER
    assert "只报数" in text
    assert "report-only" in text
    assert "偏离上游" in text
    assert mod.arm_deviates("off") is False
    assert mod.arm_deviates("sphere") is True
    assert mod.arm_deviates("end_free") is True
    assert "末端" in mod.END_FREE_NOTE


def test_ticket_matrix_covers_swap3_and_six():
    mod = _script()
    assert mod.SCENE_SEEDS["d014-swap3"] == tuple(range(10))
    assert mod.SCENE_SEEDS["swap6-open"] == tuple(range(5))
    assert mod.SCENE_SEEDS["d014-swap6"] == tuple(range(5))
    swap3 = mod.SCENE_ARMS["d014-swap3"]
    for arm in ("off", "sphere", "r100", "w16", "r100w16", "geom_multi", "empty_brake", "end_free"):
        assert arm in swap3
    for scene in ("swap6-open", "d014-swap6"):
        assert mod.SCENE_ARMS[scene] == ("off", "sphere", "geom")
    assert mod.arm_spec("sphere")["radius_m"] == pytest.approx(0.85)
    assert mod.arm_spec("sphere")["weight"] == pytest.approx(8.0)
    assert mod.arm_spec("r100")["radius_m"] == pytest.approx(1.0)
    assert mod.arm_spec("w16")["weight"] == pytest.approx(16.0)
    assert mod.arm_spec("r100w16")["radius_m"] == pytest.approx(1.0)
    assert mod.arm_spec("r100w16")["weight"] == pytest.approx(16.0)
    assert mod.arm_spec("end_free")["end_free"] is True
    assert mod.arm_spec("sphere")["end_free"] is False


def test_cards_only_set_public_fields_and_do_not_touch_disk():
    mod = _script()
    before = {path: path.read_bytes() for path in _watched()}
    names_before = sorted(p.name for p in (ROOT / "scenes").glob("*.json"))
    from yopo_l0.backend.config import config_from_params, peer_sphere_enabled
    from yopo_l0.policy import read_empty_brake, read_track_clearance, read_track_clearance_mode, read_track_margin_m

    off = mod.build_card("d014-swap3", "off")
    assert "policy_params" not in off.card
    assert peer_sphere_enabled(off.card.get("policy_params")) is False

    sphere = mod.build_card("d014-swap3", "sphere")
    end_free = mod.build_card("d014-swap3", "end_free")
    wider = mod.build_card("d014-swap3", "r100w16")
    multi = mod.build_card("d014-swap3", "geom_multi")
    brake = mod.build_card("d014-swap3", "empty_brake")
    both = mod.build_card("d014-swap3", "geom_multi+empty_brake")
    geom = mod.build_card("swap6-open", "geom")

    assert sphere.card["policy_params"]["backend"] == end_free.card["policy_params"]["backend"]
    for card, radius, weight in (
        (sphere, 0.85, 8.0),
        (wider, 1.0, 16.0),
        (multi, 0.85, 8.0),
        (brake, 0.85, 8.0),
        (both, 0.85, 8.0),
        (geom, 0.85, 8.0),
    ):
        params = card.card["policy_params"]
        assert peer_sphere_enabled(params) is True
        cfg = config_from_params(params, a_max=6.0)
        assert cfg is not None
        assert cfg.peer_sphere_radius_m == pytest.approx(radius)
        assert cfg.peer_sphere_weight == pytest.approx(weight)
        assert cfg.swarm_penalty is False
        assert cfg.swarm_segment_penalty is False
        assert "swarm_penalty" not in params["backend"]
        assert "safe_radius" not in params.get("upstream", {})

    assert read_track_clearance(sphere.card["policy_params"]) is False
    assert read_empty_brake(sphere.card["policy_params"]) is False
    assert read_track_clearance(multi.card["policy_params"]) is True
    assert read_track_clearance_mode(multi.card["policy_params"]) == "geom_multi"
    assert read_track_margin_m(multi.card["policy_params"]) == pytest.approx(0.08)
    assert read_empty_brake(multi.card["policy_params"]) is False
    assert read_empty_brake(brake.card["policy_params"]) is True
    assert read_track_clearance(brake.card["policy_params"]) is False
    assert read_track_clearance(both.card["policy_params"]) is True
    assert read_empty_brake(both.card["policy_params"]) is True
    assert read_track_clearance_mode(geom.card["policy_params"]) == "geom"

    open_off = mod.build_card("swap6-open", "off")
    assert open_off.card["name"] == "swap-6-open"
    assert len(open_off.card["drones"]) == 6
    assert open_off.card["geometry"].get("cylinders", []) == []
    assert "policy_params" not in open_off.card
    starts = [np.asarray(d["start_enu"], dtype=float) for d in open_off.card["drones"]]
    for i, start in enumerate(starts):
        goal = np.asarray(open_off.card["drones"][i]["goal_enu"], dtype=float)
        assert goal[:2] == pytest.approx(-start[:2])
        assert float(np.linalg.norm(start[:2])) == pytest.approx(8.0)
    for i, a in enumerate(starts):
        for b in starts[i + 1 :]:
            assert float(np.linalg.norm(a - b)) > 0.70

    dense = mod.build_card("d014-swap6", "off")
    assert dense.card["name"] == "ego-density-014-swap6"
    assert len(dense.card["geometry"]["cylinders"]) == 56
    assert not (ROOT / "scenes/ego-density-014-swap6.json").exists()
    assert not (ROOT / "scenes/swap-6-open.json").exists()
    after = {path: path.read_bytes() for path in before}
    assert after == before
    assert sorted(p.name for p in (ROOT / "scenes").glob("*.json")) == names_before


def test_last_span_cost_is_local_and_default_hook_matches_library():
    mod = _script()
    import yopo_l0.backend.bspline as bspline

    before = (ROOT / "yopo_l0/backend/bspline.py").read_bytes()
    q = np.zeros((9, 3))
    q[:, 2] = 1.5
    tail = np.array([[1.0, 0.5, 0.0, 1.5], [1.2, 0.5, 0.0, 1.5]])
    library = bspline._peer_sphere_cost_grad
    lib_cost, lib_grad = library(q, 0.2, 0.0, [tail], 0.85, 8.0)
    local_cost, local_grad = mod.peer_sphere_cost_including_last_span(q, 0.2, 0.0, [tail], 0.85, 8.0)
    assert lib_cost == pytest.approx(0.0)
    assert not np.any(lib_grad)
    sample, _ = bspline._peer_sphere_sample(np.array([0.5]), 0.85, 8.0, 0.15)
    assert local_cost == pytest.approx(float(sample[0]) * 6)
    assert local_cost > 0.0
    assert np.all(np.isfinite(local_grad))
    assert np.any(local_grad)

    mod.install_end_free()
    mod.STATE["end_free"] = False
    hooked = bspline._peer_sphere_cost_grad(q, 0.2, 0.0, [tail], 0.85, 8.0)
    assert hooked[0] == pytest.approx(0.0)
    mod.STATE["end_free"] = True
    opened = bspline._peer_sphere_cost_grad(q, 0.2, 0.0, [tail], 0.85, 8.0)
    assert opened[0] == pytest.approx(local_cost)
    zeros_q = np.ones((9, 3))
    seg_cost, seg_grad = bspline._swarm_segment_cost_grad(zeros_q, 0.2, 0.0, [], 0.8, 2.0)
    assert seg_cost == pytest.approx(0.0)
    assert seg_grad.shape == zeros_q.shape
    assert not np.any(seg_grad)
    cfg = mod.config_with_end_free(
        __import__("yopo_l0.backend.config", fromlist=["BackendConfig"]).BackendConfig(
            peer_sphere_enabled=True, peer_sphere_radius_m=0.85, peer_sphere_weight=8.0
        )
    )
    assert cfg.swarm_segment_penalty is True
    assert cfg.peer_sphere_enabled is True
    assert cfg.peer_sphere_radius_m == pytest.approx(0.85)
    mod.STATE["end_free"] = False
    assert (ROOT / "yopo_l0/backend/bspline.py").read_bytes() == before


def test_approach_reports_crossing_and_closing_speed():
    mod = _script()
    ticks = []
    for k in range(0, 21):
        t = k / 100.0
        half = (100 - 2 * k) / 200.0
        for uav, sign, vx in (("uav0", 1.0, -1.0), ("uav1", -1.0, 1.0)):
            ticks.append(
                {
                    "t": t,
                    "uav": uav,
                    "p": [sign * half, 0.0, 1.5],
                    "v": [vx, 0.0, 0.0],
                }
            )
    app = mod.approach_from_ticks(ticks, reason="uav0-uav1:INTER_DRONE")
    assert app["kind"] == "collision"
    assert app["pair"] == "uav0-uav1"
    assert app["t_s"] == pytest.approx(0.15)
    assert app["distance_m"] == pytest.approx(0.70)
    assert app["closing_speed_mps"] == pytest.approx(2.0)
    prev = next(s for s in app["samples"] if abs(s["dt_s"] + 0.01) < 1e-9)
    assert prev["distance_m"] == pytest.approx(0.72)
    assert prev["speed_mps"]["uav0"] == pytest.approx(1.0)
    nxt = next(s for s in app["samples"] if abs(s["dt_s"] - 0.01) < 1e-9)
    assert nxt["distance_m"] == pytest.approx(0.68)


def test_seed1_headline_says_whether_0694_remains():
    mod = _script()
    hit = {
        "scene_kind": "d014-swap3",
        "seed": 1,
        "arm": "sphere",
        "outcome": "COLLISION",
        "reason": "uav1-uav2:INTER_DRONE",
        "inter_drone_collisions": 1,
        "min_inter_drone_distance_m": 0.694,
    }
    cleared = {
        "scene_kind": "d014-swap3",
        "seed": 1,
        "arm": "r100",
        "outcome": "SUCCESS",
        "reason": "GOAL_REACHED_AND_HELD",
        "inter_drone_collisions": 0,
        "min_inter_drone_distance_m": 0.91,
    }
    text = mod.seed1_headline([hit, cleared])
    assert "0.694 m 没有消除" in text
    page = mod.render_markdown({"rows": []})
    assert "report-only" in page
    assert "偏离上游" in page
    assert "0.694 m" in page
    gone = dict(hit)
    gone["outcome"] = "SUCCESS"
    gone["reason"] = "GOAL_REACHED_AND_HELD"
    gone["inter_drone_collisions"] = 0
    gone["min_inter_drone_distance_m"] = 0.86
    assert "没有出现" in mod.seed1_headline([gone])
