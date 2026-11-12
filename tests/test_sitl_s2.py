"""票 S2：三机 PX4 SITL 骨架的布局、任务和 result.json（不需要 docker / rclpy / px4_msgs）。"""

import importlib.util
import json
import re
import subprocess
import sys

import numpy as np
import pytest

from yopo_swarm_core import px4

from conftest import ROOT

SITL = ROOT / "scripts/sitl"


def _load(name):
    spec = importlib.util.spec_from_file_location(f"sitl_s2_{name}", SITL / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


fleet = _load("fleet")
summ = _load("summarize")


class _Fake:
    """一阶跟踪。truth_dz 是真值相对估计的固定高度差。bias_z 是跟踪偏差。"""

    def __init__(self, origin, arm_ok=True, bias_z=0.0, truth_dz=0.05, tau=0.6):
        self.p = np.array(origin, dtype=float)
        self.v = np.zeros(3)
        self.armed = self.offboard = self.landing = False
        self.arm_ok, self.tau, self.bias_z, self.truth_dz = arm_ok, tau, bias_z, truth_dz
        self.sp = self.p.copy()

    def state(self, t):
        landed = self.p[2] <= 1e-3
        truth = self.p + np.array([0.0, 0.0, self.truth_dz])
        return fleet._om().VehicleState(
            t, self.p.copy(), self.v.copy(), self.armed, self.offboard, landed, truth
        )

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


def _fly(n=3, fakes=None, t_max=300.0, dt=0.02):
    group = fleet.Fleet(n)
    if fakes is None:
        fakes = [_Fake(np.array([s.spawn_xy_m[0], s.spawn_xy_m[1], 0.0])) for s in group.specs]
    t = 0.0
    while not group.done and t < t_max:
        actions = group.step([f.state(t) for f in fakes])
        for fake, act in zip(fakes, actions):
            fake.apply(act, dt)
        t += dt
    return group, fakes


def _code_lines(path):
    out = []
    for line in path.read_text().splitlines():
        s = line.strip()
        if s.startswith("#"):
            continue
        out.append(s)
    return "\n".join(out)


def test_three_vehicle_layout_is_px4_1_to_3_on_one_domain():
    specs = fleet.fleet_specs(3)
    assert [s.instance for s in specs] == [1, 2, 3]
    assert [s.mav_sys_id for s in specs] == [2, 3, 4]
    assert [s.namespace for s in specs] == ["px4_1", "px4_2", "px4_3"]
    assert [s.model for s in specs] == ["x500_1", "x500_2", "x500_3"]
    assert [s.gcs_port for s in specs] == [18571, 18572, 18573]
    assert [s.standalone for s in specs] == [False, True, True]
    ys = [s.spawn_xy_m[1] for s in specs]
    assert ys == [0.0, 3.0, 6.0]
    assert min(abs(ys[i] - ys[j]) for i in range(3) for j in range(i)) >= 3.0
    data = fleet.layout_dict(3)
    assert data["dds_domain_id"] == 0
    assert data["xrce_port"] == px4.XRCE_DEFAULT_PORT == fleet.XRCE_PORT
    assert data["report_only"] is True and data["criteria_frozen"] is False
    assert data["takeoff_alt_m"] == 1.5
    shell = fleet.format_shell(3)
    assert shell.count("dds_domain_id=0") == 1
    assert shell.count("xrce_port=8888") == 1
    assert "param set" not in shell and "PX4_PARAM_" not in shell and "PX4_UXRCE_DDS_NS" not in shell
    accepted = (
        "---", "n=", "dds_domain_id=", "xrce_port=", "mav_sys_id=", "namespace=",
        "model=", "gcs_port=", "instance=", "pose=", "standalone=",
    )
    for line in shell.splitlines():
        assert line == "" or line.startswith(accepted), line
    container = (SITL / "in_container.sh").read_text()
    assert 'px4" -d -i' in container


def test_single_vehicle_layout_stays_s1_instance_zero():
    s = fleet.fleet_specs(1)[0]
    assert (s.instance, s.mav_sys_id, s.namespace, s.model, s.gcs_port, s.pose, s.standalone) == (
        0, 1, "", "x500_0", 18570, "", False,
    )


def test_layout_rejects_n_outside_1_to_3():
    for n in (0, 4, -1):
        with pytest.raises(ValueError):
            fleet.fleet_specs(n)
    for n in ("4", "0", "x"):
        proc = subprocess.run([sys.executable, str(SITL / "fleet.py"), "--n", n], capture_output=True, text=True)
        assert proc.returncode == 2, proc.stderr


def test_commands_and_topics_use_each_sys_id_and_namespace():
    for spec in fleet.fleet_specs(3):
        topics = fleet.ros_topics(spec)
        assert topics["trajectory_setpoint"] == f"/{spec.namespace}/fmu/in/trajectory_setpoint"
        assert topics["vehicle_status"] == f"/{spec.namespace}/fmu/out/vehicle_status_v1"
        assert topics["vehicle_odometry"] == f"/{spec.namespace}/fmu/out/vehicle_odometry"
        arm = fleet.command_message("arm", 10, spec.mav_sys_id)
        land = fleet.command_message("land", 10, spec.mav_sys_id)
        offboard = fleet.command_message("offboard", 10, spec.mav_sys_id)
        assert arm["command"] == 400 and arm["target_system"] == spec.mav_sys_id
        assert land["command"] == 21 and land["target_system"] == spec.mav_sys_id
        assert offboard["command"] == 176 and offboard["target_system"] == spec.mav_sys_id
        assert arm["target_system"] != 0


def test_three_vehicles_hover_at_own_1_5m_then_land_and_record_errors():
    specs = fleet.fleet_specs(3)
    biases = (0.0, 0.05, 0.1)
    truths = (0.02, 0.05, -0.01)
    fakes = [
        _Fake(np.array([s.spawn_xy_m[0], s.spawn_xy_m[1], 0.0]), bias_z=b, truth_dz=t)
        for s, b, t in zip(specs, biases, truths)
    ]
    group, _ = _fly(3, fakes)
    assert group.done and all(m.phase == "DONE" for m in group.missions)
    payload = fleet.result_payload(
        group,
        positioning="default",
        wall_to_preflight_ok_s=1.0,
        preflight_checks_pass_seen=True,
        mission_wall_s=20.0,
        mission_sim_s=16.0,
    )
    assert payload["schema"] == "yopo-restart/s2-sitl-fleet/1"
    assert payload["report_only"] is True and payload["criteria_frozen"] is False
    assert payload["ok"] is True and payload["n"] == 3 and payload["dds_domain_id"] == 0
    assert payload["phase"] == "DONE"
    assert [v["namespace"] for v in payload["vehicles"]] == ["px4_1", "px4_2", "px4_3"]
    for spec, veh, bias, truth_dz in zip(specs, payload["vehicles"], biases, truths):
        hover = veh["hover_enu"]
        assert hover[0] == pytest.approx(spec.spawn_xy_m[0])
        assert hover[1] == pytest.approx(spec.spawn_xy_m[1])
        assert hover[2] == pytest.approx(1.5)
        m = veh["metrics"]
        assert m["failure"] is None and m["landed_and_disarmed"]
        assert m["goto_s"] is None and m["goto_err_m"] is None
        assert m["hover_samples"] >= 400
        assert m["hover_alt_err_rms_m"] == pytest.approx(bias, abs=0.02)
        assert m["hover_est_truth_alt_bias_mean_m"] == pytest.approx(-truth_dz, abs=1e-6)
        assert veh["ok"] is True
    # 误差大小不决定 ok：0.1 m 的跟踪偏差仍然算飞完
    assert payload["vehicles"][2]["ok"] is True


def test_one_vehicle_arming_refusal_fails_the_fleet_only():
    specs = fleet.fleet_specs(3)
    fakes = []
    for i, s in enumerate(specs):
        fakes.append(_Fake(np.array([s.spawn_xy_m[0], s.spawn_xy_m[1], 0.0]), arm_ok=(i != 1)))
    group, _ = _fly(3, fakes)
    payload = fleet.result_payload(
        group,
        positioning="rtk",
        wall_to_preflight_ok_s=None,
        preflight_checks_pass_seen=False,
        mission_wall_s=None,
        mission_sim_s=None,
    )
    assert payload["ok"] is False and payload["criteria_frozen"] is False
    assert payload["vehicles"][0]["ok"] and payload["vehicles"][2]["ok"]
    failed = payload["vehicles"][1]
    assert failed["ok"] is False
    assert failed["metrics"]["failure"].startswith("ARMING")
    assert failed["namespace"] == "px4_2"


def test_summarize_fleet_is_report_only(tmp_path):
    specs = fleet.fleet_specs(3)
    fakes = [_Fake(np.array([s.spawn_xy_m[0], s.spawn_xy_m[1], 0.0]), bias_z=0.0) for s in specs]
    group, _ = _fly(3, fakes)
    payload = fleet.result_payload(
        group,
        positioning="default",
        wall_to_preflight_ok_s=3.0,
        preflight_checks_pass_seen=True,
        mission_wall_s=30.0,
        mission_sim_s=24.0,
    )
    (tmp_path / "result.json").write_text(json.dumps(payload))
    text = summ.render_fleet(payload, 40.0, "img")
    assert "report-only" in text and "判据未冻结" in text
    assert "px4_1" in text and "px4_2" in text and "px4_3" in text
    assert "| RTF | 0.80 |" in text
    assert "已冻结" not in text
    assert summ.main([str(tmp_path), "--total-wall", "40", "--image", "img"]) == 0
    written = (tmp_path / "summary.md")
    assert not written.exists()  # main 只打印，不自己落盘


def test_multi_vehicle_scripts_add_no_params_outside_whitelist():
    for p in sorted(SITL.rglob("*")):
        if not p.is_file() or p.name == "positioning-rtk.env":
            continue
        if p.suffix not in (".sh", ".py", ".env"):
            continue
        assert "PX4_PARAM_" not in _code_lines(p), p
    for name in ("up.sh", "smoke.sh", "in_container.sh", "exec_mission.sh", "common.sh", "fleet.py"):
        code = _code_lines(SITL / name)
        assert "param set" not in code, name
        assert not re.search(r"px4-param[^\n]*\bset\b", code), name
        assert not re.search(r"PX4_PARAM_[A-Z]", code), name
    container = _code_lines(SITL / "in_container.sh")
    assert "export PX4_UXRCE_DDS_NS" not in container
    assert "fleet.py" in container
    assert "--n" in (SITL / "up.sh").read_text()
    assert "--n" in (SITL / "smoke.sh").read_text()


def test_shell_scripts_parse_and_reject_bad_n():
    for name in ("up.sh", "smoke.sh", "in_container.sh", "common.sh", "compare.sh", "exec_mission.sh"):
        subprocess.run(["bash", "-n", str(SITL / name)], check=True)
    help_up = subprocess.run(["bash", str(SITL / "up.sh"), "--help"], capture_output=True, text=True)
    assert help_up.returncode == 0 and "--n" in help_up.stdout
    help_smoke = subprocess.run(["bash", str(SITL / "smoke.sh"), "--help"], capture_output=True, text=True)
    assert help_smoke.returncode == 0 and "--n" in help_smoke.stdout
    for script in ("up.sh", "smoke.sh"):
        bad = subprocess.run(["bash", str(SITL / script), "--n", "4"], capture_output=True, text=True)
        assert bad.returncode == 2, (script, bad.stdout, bad.stderr)
