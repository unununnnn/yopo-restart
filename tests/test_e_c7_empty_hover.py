"""票 E-C7：空走廊悬停归因。只报数，不改默认开关，不改场景文件。"""

import importlib.util
import json
import sys

import numpy as np
import pytest

from yopo_l0.dynamics import State
from yopo_l0.inter_drone import InterDroneConfig, inter_pick
from yopo_l0.policy import Observation, PolicyContext, UpstreamPolicy
from yopo_l0.scene import Cylinder, Ground
from yopo_l0.sensor import DepthFrame
from yopo_swarm_core.candidates import UPSTREAM_DECODER
from yopo_swarm_core.plan import hold_traj

from conftest import ROOT

GEOM_SHA256 = "5a1ed08922513039c0cf8022722457139df056a28a4a1f0e74d21c389c183cd9"


def _script():
    name = "e_c7_empty_hover"
    path = ROOT / "scripts/e-c7-empty-hover.py"
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_check_and_scene_files_stay_put():
    mod = _script()
    watched = [
        ROOT / "scenes/ego-density-014.json",
        ROOT / "scenes/ego-density-014-swap3.json",
        ROOT / "scenes/suite.txt",
        ROOT / "yopo_l0/inter_drone.py",
        ROOT / "yopo_l0/policy.py",
        ROOT / "docs/tickets.md",
        ROOT / "README.md",
    ]
    before = {path: path.read_bytes() for path in watched}
    assert mod.main(["--check"]) == 0
    assert {path: path.read_bytes() for path in watched} == before
    assert "report-only" in mod.DISCLAIMER
    assert "只报数" in mod.DISCLAIMER
    assert mod.TOUCH_M == pytest.approx(0.40)
    assert mod.HARD_MIN_M == pytest.approx(0.75)
    assert mod.PROBE_KEY not in (ROOT / "yopo_l0/inter_drone.py").read_text(encoding="utf-8")
    assert "ec7" not in (ROOT / "scenes/suite.txt").read_text(encoding="utf-8")
    digest = __import__("hashlib").sha256((ROOT / "scenes/ego-density-014.json").read_bytes()).hexdigest()
    assert digest == GEOM_SHA256


def test_swap6_matches_e_c6_azimuths_and_swap3():
    mod = _script()
    placed = mod.place_swap6(mod.density_centers())
    assert [d["id"] for d in placed] == [f"uav{i}" for i in range(6)]
    disk = json.loads((ROOT / "scenes/ego-density-014-swap3.json").read_text(encoding="utf-8"))
    for src, idx in ((0, 0), (1, 2), (2, 4)):
        got = mod._public_drone(placed[idx])
        for key in ("start_enu", "goal_enu"):
            assert got[key] == pytest.approx(disk["drones"][src][key])
    assert placed[1]["pushed_outside"] is False
    assert placed[2]["pushed_outside"] is True
    assert placed[5]["pushed_outside"] is True
    for d in placed:
        assert d["axis_distance_start"] >= 0.70 - 1e-9
        assert d["axis_distance_goal"] >= 0.70 - 1e-9
        start = np.asarray(d["start_enu"], dtype=float)
        goal = np.asarray(d["goal_enu"], dtype=float)
        assert np.allclose(goal[:2], -start[:2])
        assert goal[2] == pytest.approx(start[2])
    for scene in mod.SCENES:
        for arm in mod.ARMS:
            card = mod.build_card(scene, arm)
            assert "e-c7" not in json.dumps(card.card["policy_params"]) or arm == "geom_soft"
            text = (ROOT / "scenes/ego-density-014-swap3.json").read_text(encoding="utf-8")
            assert "ec7_soft_yield_probe" not in text
            assert "track_clearance" not in text


def test_classify_cause_four_ways():
    mod = _script()
    assert mod.classify_cause({"select_brake": True}) == "corridor_empty"
    assert mod.classify_cause({
        "select_brake": False,
        "geom_called": True,
        "geom_brake": True,
        "geom_corridor_empty": True,
        "geom_index_missing": False,
        "geom_ok": 0,
    }) == "corridor_empty"
    assert mod.classify_cause({
        "select_brake": False,
        "geom_called": True,
        "geom_brake": True,
        "geom_corridor_empty": False,
        "geom_index_missing": False,
        "geom_ok": 0,
    }) == "geom_reject"
    assert mod.classify_cause({
        "select_brake": False,
        "geom_called": True,
        "geom_brake": True,
        "geom_corridor_empty": False,
        "geom_index_missing": True,
        "geom_ok": 0,
    }) == "other"
    assert mod.classify_cause({
        "select_brake": False,
        "geom_called": True,
        "geom_brake": False,
        "inter_called": True,
        "inter_empty_delta": 1,
    }) == "inter_empty"
    assert mod.classify_cause({"select_brake": False, "geom_called": False, "inter_called": False}) == "other"


def test_soft_yield_keeps_above_hard_min_and_reranks():
    mod = _script()
    cfg = InterDroneConfig(1.0, 0.0)
    # 0.90 m 减分更少，同分时它排到前面。0.50 m 低于 0.75 m，滤掉。
    pick = mod.soft_yield_pick(
        np.array([1.0, 1.0, 9.0]),
        np.array([True, True, True]),
        np.array([0.90, 0.80, 0.50]),
        cfg,
    )
    assert pick is not None
    assert pick.kind == "adjusted"
    assert pick.action == 0
    assert pick.filtered == 1
    assert pick.penalized == 2
    # 分数差得够大时，近一点的仍能赢。
    closer = mod.soft_yield_pick(
        np.array([0.0, 1.0]),
        np.array([True, True]),
        np.array([0.90, 0.80]),
        cfg,
    )
    assert closer.action == 1
    # 测不到的 +inf 留下，并且不减分。低于下限的那个被滤掉。
    unknown = mod.soft_yield_pick(
        np.array([0.1, 5.0]),
        np.array([True, True]),
        np.array([np.inf, 0.20]),
        cfg,
    )
    assert unknown.action == 0
    assert unknown.penalized == 0
    assert unknown.filtered == 1
    assert mod.soft_yield_pick(np.array([1.0]), np.array([True]), np.array([0.74]), cfg) is None
    # 探针没装上时，硬过滤仍是整票判空。
    empty = inter_pick(np.array([1.0, 2.0]), np.array([True, True]), np.array([0.2, 0.3]), cfg)
    assert empty.kind == "empty"
    assert empty.filtered == 2


def test_touch_line_and_clearance_split():
    mod = _script()
    assert mod.is_touch(0.40) is True
    assert mod.is_touch(0.40 + 1e-6) is False
    assert mod.is_touch(None) is False
    assert mod.is_touch(float("inf")) is False
    ground, tree = mod.split_clearance(
        (Cylinder("c", (0.0, 0.0), 0.2, 0.0, 3.0), Ground(0.0)),
        np.array([[0.56, 0.0, 1.5], [1.0, 0.0, 0.2]]),
    )
    assert ground == pytest.approx(0.2)
    assert tree == pytest.approx(0.36)


def test_yield_coupling_counts_the_other_drone_on_the_next_tick():
    mod = _script()
    events = [
        {"t": 0.0, "uav": "uav0", "cause": None, "yield_release": True},
        {"t": 0.0, "uav": "uav1", "cause": "corridor_empty", "yield_release": False},
        {"t": 0.1, "uav": "uav1", "cause": "geom_reject", "yield_release": False},
        {"t": 0.1, "uav": "uav0", "cause": "geom_reject", "yield_release": False},
        {"t": 0.2, "uav": "uav1", "cause": "geom_reject", "yield_release": False},
    ]
    got = mod.yield_geom_coupling(events)
    # 0.1 s：uav1 的拒绝算在 uav0 放行之后；uav0 自己上一拍被放行，不算「另一架」。
    # 0.2 s 的上一拍没有放行。
    assert got == {"geom_reject": 3, "geom_reject_after_other_yield": 1}


def test_terminal_stop_duration_and_partial_roundtrip(tmp_path):
    mod = _script()
    events = [
        {"t": 1.0, "uav": "uav0", "code": "OK", "cause": None, "p": [0.0, 0.0, 1.5]},
        {"t": 2.0, "uav": "uav0", "code": mod.HOVER, "cause": "geom_reject", "p": [1.0, 0.0, 1.5]},
        {"t": 3.0, "uav": "uav0", "code": "OK", "cause": None, "p": [1.2, 0.0, 1.5]},
        {"t": 4.0, "uav": "uav0", "code": mod.HOVER, "cause": "inter_empty", "p": [1.3, 0.0, 1.4]},
        {"t": 5.0, "uav": "uav0", "code": mod.HOVER, "cause": "inter_empty", "p": [1.3, 0.0, 1.4]},
        {"t": 4.0, "uav": "uav1", "code": "ARRIVED_HOLD", "cause": None, "p": [8.0, 0.0, 1.5]},
    ]
    ticks = []
    for k, (uav, p, err, speed) in enumerate((
        ("uav0", [1.3, 0.0, 1.4], 6.0, 0.0),
        ("uav1", [8.0, 0.0, 1.5], 0.1, 0.0),
    )):
        ticks.append({
            "k": k, "t": 1.0 + k, "uav": uav, "p": p, "v": [speed, 0.0, 0.0],
            "goal_error_m": err, "plan_code": "OK",
        })
    # uav1 在目标里停满 1 s 以上才算到达。这里只有一拍，所以两架都没到达。
    crit = {"goal_tolerance_m": 0.4, "goal_speed_mps": 0.2, "goal_hold_s": 1.0}
    assert mod.arrived_ids(ticks, crit) == set()
    held = []
    for i in range(15):
        held.append({
            "k": i, "t": float(i) * 0.1, "uav": "uav1", "p": [8.0, 0.0, 1.5],
            "v": [0.0, 0.0, 0.0], "goal_error_m": 0.1, "plan_code": "ARRIVED_HOLD",
        })
    assert mod.arrived_ids(held, crit) == {"uav1"}
    stops = mod.terminal_stops(events, ticks, crit, "TIMEOUT", 6.0)
    by_id = {row["uav"]: row for row in stops}
    assert by_id["uav0"]["terminal_hover_start_s"] == pytest.approx(4.0)
    assert by_id["uav0"]["terminal_hover_duration_s"] == pytest.approx(2.0)
    assert by_id["uav0"]["terminal_hover_n"] == 2
    assert by_id["uav0"]["hover_continuous_to_timeout"] is False
    assert by_id["uav0"]["first_hover_s"] == pytest.approx(2.0)
    assert by_id["uav0"]["terminal_cause_counts"]["inter_empty"] == 2
    assert by_id["uav0"]["final_p"] == pytest.approx([1.3, 0.0, 1.4])
    assert mod.terminal_stops(events, ticks, crit, "SUCCESS", 6.0) == []

    row = {
        "row_version": mod.ROW_VERSION,
        "scene": "d014-swap6",
        "arm": "geom",
        "seed": 3,
        "duration_s_override": None,
        "report_only": True,
        "corridor_empty_hover": 2,
    }
    mod.save_partial(row, tmp_path)
    assert mod.load_partial("d014-swap6", "geom", 3, None, tmp_path)["corridor_empty_hover"] == 2
    assert mod.load_partial("d014-swap6", "geom", 3, 1.0, tmp_path) is None
    short = dict(row, duration_s_override=1.0, seed=4)
    mod.save_partial(short, tmp_path)
    assert mod.load_partial("d014-swap6", "geom", 4, 1.0, tmp_path)["duration_s_override"] == 1.0


def test_planner_hover_count_skips_repeated_physics_ticks():
    mod = _script()
    ticks = []
    for k in range(20):
        code = mod.HOVER if k >= 10 else "OK"
        ticks.append({"k": k, "uav": "uav0", "plan_code": code})
        ticks.append({"k": k, "uav": "uav1", "plan_code": "OK"})
    # physics 100、planner 10：k = 0 和 k = 10 是规划拍。k=10 起 uav0 才悬停，所以只计 1。
    assert mod.planner_hover_count(ticks, 100, 10) == 1


def test_markdown_is_report_only_and_names_the_four_causes():
    mod = _script()
    payload = {
        "commit": "abc",
        "rows": [{
            "scene": "d014-swap6",
            "arm": "geom",
            "seed": 0,
            "n_drones": 6,
            "arrived": 5,
            "pillar": 0,
            "ground": 0,
            "inter": 0,
            "timeout": 1,
            "avg_speed_mps": 0.564,
            "mean_speed_mps": 0.2,
            "speed_samples": 10,
            "corridor_empty_hover": 4,
            "cause_counts": {"corridor_empty": 1, "geom_reject": 2, "inter_empty": 1, "other": 0},
            "empty_brake_applied": 1,
            "empty_brake_hold_fallback": 3,
            "pos_near_ground": 0,
            "traj_near_ground": 0,
            "pos_near_tree": 0,
            "traj_near_tree": 1,
            "prev_min_z_min": 1.2,
            "prev_min_z_max": 1.5,
            "hover_min_inter_min": 1.1,
            "hover_pos_z_median": 1.45,
            "yield_releases": 0,
            "yield_coupling": {"geom_reject": 2, "geom_reject_after_other_yield": 0},
            "min_inter_drone_distance_m": 1.187,
            "outcome": "TIMEOUT",
            "stops": [{
                "uav": "uav2",
                "final_p": [0.1, 0.2, 1.4],
                "final_speed_mps": 0.0,
                "final_goal_error_m": 8.0,
                "terminal_hover_start_s": 12.0,
                "terminal_hover_duration_s": 28.0,
                "terminal_hover_n": 280,
                "terminal_cause_counts": {"corridor_empty": 1, "geom_reject": 2, "inter_empty": 1, "other": 0},
                "hover_continuous_to_timeout": True,
            }],
        }],
    }
    text = mod.render_markdown(payload)
    assert "只报数" in text
    assert "report-only" in text
    assert "走廊准入为空" in text
    assert "geom 净空门拒绝" in text
    assert "inter_pick" in text
    assert "uav2" in text
    assert "没有改 `yopo_l0/inter_drone.py`" in text
    assert "1 / 2 / 1 / 0" in text


def _frame():
    return DepthFrame(0.5, np.zeros((4, 4), np.float32), (1.0, 1.0, 1.0, 1.0))


def _obs(p=(0.0, 0.0, 1.5)):
    state = State(0.5, np.array(p, float), np.zeros(3), np.zeros(3), 0.0)
    return Observation(_frame(), state, np.array([30.0, 0.0, 1.5]), 0.5)


def _stub(monkeypatch, positions, scores, mu, head0):
    positions = np.asarray(positions, float)
    head0 = np.asarray(head0, float)
    inner_b = positions - head0
    n = len(positions)

    def fake_forward(*_a, **_k):
        return (
            np.zeros((n, 14), np.float32),
            np.asarray(scores, np.float64),
            np.zeros((n, 2), np.float32),
        )

    def fake_decode(_params, _dec):
        return inner_b.copy(), np.zeros((n, 3, 3)), np.ones((n, 2))

    def fake_mu(_radius, _dec):
        return np.asarray(mu, np.float64)

    def fake_minco(t0, _head, _tail, inner, _durations):
        return hold_traj(np.asarray(inner, float).reshape(3), float(t0), float(t0) + 0.2)

    monkeypatch.setattr("yopo_swarm_core.candidates.decode_params", fake_decode)
    monkeypatch.setattr("yopo_swarm_core.candidates.corridor_mu_lo", fake_mu)
    monkeypatch.setattr("yopo_swarm_core.candidates.minco_two_piece", fake_minco)
    return fake_forward


def _policy(monkeypatch, *, mode, mu, xs, neighbor=None, soft=False):
    n = int(UPSTREAM_DECODER.count)
    positions = np.zeros((n, 3))
    positions[:, 2] = 1.5
    positions[:, 0] = 20.0
    score = np.full(n, -1.0)
    admitted = np.zeros(n)
    for i, (x, ok) in enumerate(zip(xs, mu)):
        positions[i, 0] = 0.2 + float(x)
        score[i] = 5.0 - i
        admitted[i] = 1.0 if ok else 0.0
    extra = {
        "inter_drone_filter": True,
        "inter_drone_threshold_m": 1.0,
        "inter_drone_penalty": 0,
        "empty_brake": True,
    }
    if soft:
        extra["ec7_soft_yield_probe"] = True
    params = {"upstream": {
        "track_clearance": True,
        "track_clearance_mode": mode,
        "track_margin_m": 0.08,
        **extra,
    }}
    obstacles = (Cylinder("c", (0.0, 0.0), 0.2, 0.0, 3.0), Ground(0.0))
    pol = UpstreamPolicy(PolicyContext("uav0", 0.01, params, obstacles=obstacles, a_max=6.0))
    pol.source.forward_raw = _stub(monkeypatch, positions, score, admitted, head0=(0.0, 0.0, 1.5))
    if neighbor is not None:
        pol.note_neighbor_samples("uav1", neighbor)
    return pol


def test_probe_names_corridor_geom_inter_and_a_soft_release(monkeypatch):
    mod = _script()
    mod.install_probe()
    try:
        mod.take_log()
        corridor = _policy(monkeypatch, mode="geom", mu=(False, False), xs=(0.36, 0.50))
        corridor.plan(_obs())
        logged = mod.take_log()
        assert corridor.last_code == "CORRIDOR_EMPTY_HOVER"
        assert logged[-1]["cause"] == "corridor_empty"
        assert logged[-1]["empty_brake_hold_fallback"] is True
        assert logged[-1]["empty_brake_applied"] is False

        geom = _policy(monkeypatch, mode="geom", mu=(True, True), xs=(0.36, 0.36))
        geom.plan(_obs())
        logged = mod.take_log()
        assert geom.last_code == "CORRIDOR_EMPTY_HOVER"
        assert logged[-1]["cause"] == "geom_reject"
        assert logged[-1]["pos_near_tree"] is True

        neighbor = [[0.0, 2.2, 0.0, 1.5], [2.0, 2.2, 0.0, 1.5]]
        inter = _policy(monkeypatch, mode="geom", mu=(True, True), xs=(2.0, 2.0), neighbor=neighbor)
        inter.plan(_obs())
        logged = mod.take_log()
        assert inter.last_code == "CORRIDOR_EMPTY_HOVER"
        assert logged[-1]["cause"] == "inter_empty"
        assert inter._inter_empty == 1

        # 候选在 x = 2.2，邻机在 x = 3.0，中心距 0.80 m：硬过滤会判空，软让行留下。
        released = [[0.0, 3.0, 0.0, 1.5], [2.0, 3.0, 0.0, 1.5]]
        soft = _policy(
            monkeypatch, mode="geom", mu=(True, True), xs=(2.0, 2.0), neighbor=released, soft=True,
        )
        soft.plan(_obs())
        logged = mod.take_log()
        assert soft.last_code == "OK"
        assert logged[-1]["yield_release"] is True
        assert logged[-1]["cause"] is None
        assert soft._inter_empty == 0
    finally:
        mod.uninstall_probe()
        mod.take_log()
