"""票 S3：三机协同 goto / swap 的路点、状态机和只报数汇总（不需要 docker / rclpy / px4_msgs）。"""

import importlib.util
import json
import os
import shutil
import subprocess
import sys

import numpy as np
import pytest

from conftest import ROOT

SITL = ROOT / "scripts" / "sitl"
PHASES = ("WARMUP", "ARMING", "TAKEOFF", "HOVER", "GOTO", "GOTO_HOLD", "LAND", "DONE")


def _load(name):
    spec = importlib.util.spec_from_file_location(f"sitl_s3_{name}", SITL / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


fleet = _load("fleet")
om = _load("offboard_mission")
summ = _load("summarize")


class _Fake:
    def __init__(self, origin, arm_ok=True, bias_z=0.0, truth_dz=0.05, tau=0.6):
        self.p = np.array(origin, dtype=float)
        self.v = np.zeros(3)
        self.armed = self.offboard = self.landing = False
        self.arm_ok, self.tau, self.bias_z, self.truth_dz = arm_ok, tau, bias_z, truth_dz
        self.sp = self.p.copy()

    def state(self, t):
        landed = self.p[2] <= 1e-3
        truth = self.p + np.array([0.0, 0.0, self.truth_dz])
        return om.VehicleState(t, self.p.copy(), self.v.copy(), self.armed, self.offboard, landed, truth)

    def apply(self, act, dt):
        for c in act.commands:
            if c == "offboard":
                self.offboard = True
            elif c == "arm" and self.arm_ok:
                self.armed = True
            elif c == "land":
                self.landing, self.offboard = True, False
        if act.setpoint_pva is not None and not self.landing:
            self.sp = act.setpoint_pva[0] + np.array([0.0, 0.0, self.bias_z])
        if not self.armed:
            self.v = np.zeros(3)
            return
        if self.landing:
            self.v = np.array([0.0, 0.0, -0.7])
        else:
            self.v = (self.sp - self.p) / self.tau
        self.p = self.p + self.v * dt
        if self.p[2] <= 0.0 and self.landing:
            self.p[2] = 0.0
            self.v = np.zeros(3)
            self.armed = False


def _fly(n=3, mission="hover", fakes=None, dt=0.02, t_max=80.0, trace_dt=0.1):
    group = fleet.Fleet(n, mission=mission)
    if fakes is None:
        fakes = [_Fake([s.spawn_xy_m[0], s.spawn_xy_m[1], 0.0]) for s in group.specs]
    phases = [[] for _ in group.missions]
    entered = {}
    traces = {s.instance: [] for s in group.specs}
    dense = []
    t = 0.0
    while not group.done and t < t_max:
        states = [f.state(t) for f in fakes]
        actions = group.step(states)
        dense.append([(float(s.truth[0]), float(s.truth[1]), float(s.truth[2])) for s in states])
        for i, mission_i in enumerate(group.missions):
            if not phases[i] or phases[i][-1] != mission_i.phase:
                phases[i].append(mission_i.phase)
                entered.setdefault(mission_i.phase, []).append(t)
        for spec, state in zip(group.specs, states):
            tr = traces[spec.instance]
            if not tr or t - tr[-1][0] >= trace_dt:
                truth = state.truth
                tr.append(
                    [round(t, 3), group.missions[spec.index - 1].phase,
                     *[float(x) for x in state.position], *[float(x) for x in truth]]
                )
        for fake, act in zip(fakes, actions):
            fake.apply(act, dt)
        t += dt
    return group, fakes, phases, entered, traces, dense


def _payload(group, traces=None, positioning="default"):
    return fleet.result_payload(
        group,
        positioning=positioning,
        wall_to_preflight_ok_s=2.0,
        preflight_checks_pass_seen=True,
        mission_wall_s=30.0,
        mission_sim_s=24.0,
        traces=traces,
    )


def test_s1_defaults_still_fly_the_original_goto():
    cfg = om.MissionConfig()
    assert (cfg.takeoff_alt_m, cfg.hover_s, cfg.goal_offset_enu, cfg.land_at_hover) == (
        2.5, 10.0, (5.0, 3.0, 0.0), False,
    )
    assert cfg.mission_mode == "s1" and cfg.goto_target_enu is None
    assert om.mission_config_from_args(None, None) == cfg
    assert om.mission_config_from_args("s1", None) == cfg
    fake = _Fake([1.0, -2.0, 0.0], truth_dz=0.05)
    mission = om.Mission()
    t = 0.0
    while not mission.done and t < 80.0:
        fake.apply(mission.step(fake.state(t)), 0.02)
        t += 0.02
    assert mission.phase == "DONE", mission.metrics
    np.testing.assert_allclose(mission.goal_sp, [6.0, 1.0, 2.5])
    assert mission.metrics.goto_err_m < 0.05
    assert mission.metrics.hover_est_truth_alt_bias_mean_m == pytest.approx(-0.05, abs=1e-6)


def test_s2_hover_defaults_skip_goto():
    cfg = fleet.fleet_mission_config()
    assert cfg.takeoff_alt_m == 1.5 and cfg.hover_s == 10.0
    assert cfg.goal_offset_enu == (0.0, 0.0, 0.0) and cfg.land_at_hover is True
    assert cfg.mission_mode == "hover" and cfg.goto_target_enu is None
    group, _, phases, _, traces, _ = _fly(3, "hover")
    assert all(p == ["WARMUP", "ARMING", "TAKEOFF", "HOVER", "LAND", "DONE"] for p in phases)
    assert all("GOTO" not in p for p in phases)
    payload = _payload(group, traces)
    assert payload["schema"] == "yopo-restart/s2-sitl-fleet/1"
    assert payload["mission"] == "hover"
    assert payload["report_only"] is True and payload["criteria_frozen"] is False
    assert payload["min_inter_drone_m"] == pytest.approx(3.0)
    for veh in payload["vehicles"]:
        assert veh["metrics"]["goto_err_m"] is None and veh["metrics"]["goto_s"] is None
        assert veh["metrics"]["hover_est_truth_alt_bias_mean_m"] == pytest.approx(-0.05, abs=1e-6)
        assert veh["ok"] is True


def test_hover_goto_swap_layouts_and_separation():
    hover = fleet.vehicle_plans(3, "hover")
    assert [p.goal_enu for p in hover] == [p.hover_enu for p in hover]
    assert fleet.min_geometric_separation_m(hover) == pytest.approx(3.0)

    goto = fleet.vehicle_plans(3, "goto")
    assert [p.goto_target_enu for p in goto] == [fleet.GOTO_OFFSET_ENU] * 3
    assert [p.goal_enu for p in goto] == [(6.0, 0.0, 1.5), (6.0, 3.0, 1.5), (6.0, 6.0, 1.5)]
    assert fleet.min_geometric_separation_m(goto) == pytest.approx(3.0)
    assert om.DEFAULT_GOTO_OFFSET_ENU == fleet.GOTO_OFFSET_ENU
    single = om.mission_config_from_args("goto", None)
    assert single.goto_target_enu == fleet.GOTO_OFFSET_ENU
    assert single.takeoff_alt_m == 1.5 and single.land_at_hover is False

    swap = fleet.vehicle_plans(3, "swap")
    assert [p.takeoff_alt_m for p in swap] == [1.5, 2.2, 2.9]
    assert [p.hover_enu for p in swap] == [(0.0, 0.0, 1.5), (0.0, 3.0, 2.2), (0.0, 6.0, 2.9)]
    assert [p.goal_enu for p in swap] == [(0.0, 6.0, 1.5), (0.0, 3.0, 2.2), (0.0, 0.0, 2.9)]
    assert fleet.min_geometric_separation_m(swap) == pytest.approx(0.7, abs=1e-6)
    data = fleet.layout_dict(3, "swap")
    assert data["mission"] == "swap" and data["report_only"] is True and data["criteria_frozen"] is False
    assert data["min_geometric_separation_m"] == pytest.approx(0.7, abs=1e-6)
    assert data["takeoff_alt_m"] == 1.5  # 表头仍是 S2 的基准高度，各层在 waypoints 里

    same_alt = [
        fleet.VehiclePlan(1, (0.0, 0.0, 0.0), (0.0, 0.0, 1.5), (6.0, 0.0, 1.5), 1.5, (6.0, 0.0, 0.0)),
        fleet.VehiclePlan(2, (6.0, 0.0, 0.0), (6.0, 0.0, 1.5), (0.0, 0.0, 1.5), 1.5, (-6.0, 0.0, 0.0)),
    ]
    assert fleet.min_geometric_separation_m(same_alt) == pytest.approx(0.0, abs=1e-9)
    with pytest.raises(ValueError, match="间隔"):
        fleet.assert_plan_separation(same_alt)
    for n in (1, 2, 4):
        with pytest.raises(ValueError):
            fleet.vehicle_plans(n, "swap")
    with pytest.raises(ValueError):
        fleet.parse_mission("cruise")


def test_shell_layout_stays_s2_when_mission_changes():
    hover = subprocess.run(
        [sys.executable, str(SITL / "fleet.py"), "--n", "3", "--format", "shell"],
        capture_output=True, text=True, check=False,
    )
    swap = subprocess.run(
        [sys.executable, str(SITL / "fleet.py"), "--n", "3", "--format", "shell", "--mission", "swap"],
        capture_output=True, text=True, check=False,
    )
    assert hover.returncode == 0 and swap.returncode == 0, (hover.stderr, swap.stderr)
    assert hover.stdout == swap.stdout
    assert "mission=" not in hover.stdout and "PX4_PARAM_" not in hover.stdout
    env = os.environ.copy()
    env["YOPO_SITL_MISSION_MODE"] = "swap"
    from_env = subprocess.run(
        [sys.executable, str(SITL / "fleet.py"), "layout", "--n", "3"],
        capture_output=True, text=True, check=False, env=env,
    )
    assert from_env.returncode == 0, from_env.stderr
    assert json.loads(from_env.stdout)["mission"] == "swap"
    override = subprocess.run(
        [sys.executable, str(SITL / "fleet.py"), "layout", "--n", "3", "--mission", "goto"],
        capture_output=True, text=True, check=False, env=env,
    )
    assert json.loads(override.stdout)["mission"] == "goto"


def test_three_vehicles_run_goto_and_swap_together():
    for mode, min_sep in (("goto", 3.0), ("swap", 0.7)):
        group, _, phases, entered, traces, dense = _fly(3, mode)
        assert group.done and all(m.phase == "DONE" for m in group.missions)
        for seen in phases:
            assert tuple(seen) == PHASES, seen
        assert len(set(round(t, 5) for t in entered["GOTO"])) == 1
        payload = _payload(group, traces)
        assert payload["schema"] == "yopo-restart/s3-sitl-crossing/1"
        assert payload["mission"] == mode
        assert payload["report_only"] is True and payload["criteria_frozen"] is False
        assert payload["min_geometric_separation_m"] == pytest.approx(min_sep, abs=1e-6)
        assert payload["min_inter_drone_m"] is not None
        assert payload["min_inter_drone_m"] >= min_sep - 0.02
        for veh in payload["vehicles"]:
            metrics = veh["metrics"]
            assert metrics["failure"] is None and metrics["goto_err_m"] < 0.05
            assert metrics["goto_err_truth_m"] == pytest.approx(metrics["goto_err_m"], abs=1e-6)
            assert metrics["hover_est_truth_alt_bias_mean_m"] == pytest.approx(-0.05, abs=1e-6)
        dense_tracks = []
        for i in range(3):
            dense_tracks.append({"trace": [[k * 0.02, "", 0, 0, 0, *dense[k][i]] for k in range(len(dense))]})
        dense_min = fleet.min_inter_drone_m_from_vehicles(dense_tracks)
        assert dense_min == pytest.approx(min_sep, abs=0.02)


def test_summarize_reports_goto_error_and_min_distance(tmp_path):
    group, _, _, _, traces, _ = _fly(3, "swap", trace_dt=0.02)
    payload = _payload(group, traces)
    (tmp_path / "result.json").write_text(json.dumps(payload))
    fleet.write_trajectory_csvs(tmp_path, traces)
    text = summ.render_fleet(payload, 40.0, "img")
    assert "report-only" in text and "判据未冻结" in text
    assert "report_only: true" in text and "criteria_frozen: false" in text
    assert "已冻结" not in text
    assert "px4_1" in text and "px4_3" in text
    assert "| RTF | 0.80 |" in text
    assert f"{payload['vehicles'][0]['metrics']['goto_err_m']:.3f}" in text
    assert f"{payload['min_inter_drone_m']:.3f}" in text
    assert "-0.050" in text
    assert summ.main([str(tmp_path), "--total-wall", "40", "--image", "img"]) == 0

    bare = tmp_path / "csv-only"
    bare.mkdir()
    copied = json.loads(json.dumps(payload))
    copied["min_inter_drone_m"] = None
    for veh in copied["vehicles"]:
        veh["trace"] = []
    (bare / "result.json").write_text(json.dumps(copied))
    fleet.write_trajectory_csvs(bare, traces)
    assert summ.min_inter_drone_m(copied, bare) == pytest.approx(payload["min_inter_drone_m"], abs=1e-9)
    assert summ.main([str(bare)]) == 0


def test_shell_scripts_accept_mission_and_keep_parsing():
    for name in ("smoke.sh", "in_container.sh"):
        subprocess.run(["bash", "-n", str(SITL / name)], check=True)
    if shutil.which("shellcheck"):
        subprocess.run(["shellcheck", str(SITL / "smoke.sh"), str(SITL / "in_container.sh")], check=True)
    help_smoke = subprocess.run(["bash", str(SITL / "smoke.sh"), "--help"], capture_output=True, text=True)
    assert help_smoke.returncode == 0 and "--mission" in help_smoke.stdout and "--n" in help_smoke.stdout
    bad = subprocess.run(["bash", str(SITL / "smoke.sh"), "--mission", "cruise"], capture_output=True, text=True)
    assert bad.returncode == 2
    swap_one = subprocess.run(["bash", str(SITL / "smoke.sh"), "--n", "1", "--mission", "swap"], capture_output=True, text=True)
    assert swap_one.returncode == 2
    container = (SITL / "in_container.sh").read_text()
    assert 'YOPO_SITL_MISSION=0' in container or "YOPO_SITL_MISSION" in container
    assert 'offboard_mission.py --out "$OUT/result.json" --wall-timeout' in container
    assert '--mission "$MISSION_MODE"' in container
    assert 'px4" -d -i' in container
    smoke = (SITL / "smoke.sh").read_text()
    assert "--mission" in smoke and "YOPO_SITL_MISSION_MODE" in smoke
