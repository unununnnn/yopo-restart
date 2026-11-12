"""票 S1：box 上单机 PX4 SITL 冒烟的纯逻辑部分（不需要 docker / rclpy / px4_msgs）。"""

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
    spec = importlib.util.spec_from_file_location(f"sitl_{name}", SITL / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


om = _load("offboard_mission")
summ = _load("summarize")


class FakePx4:
    """一阶跟踪设定点的假飞控：offboard/arm 命令立刻生效，NAV_LAND 后匀速下降、落地自动上锁。"""

    def __init__(self, arm_ok=True, tau=0.6, bias_z=0.0):
        self.p = np.array([1.0, -2.0, 0.0])
        self.v = np.zeros(3)
        self.armed = self.offboard = self.landing = False
        self.arm_ok, self.tau, self.bias_z = arm_ok, tau, bias_z
        self.sp = self.p.copy()

    def state(self, t):
        landed = self.p[2] <= 1e-3
        # 真值 = 估计 + 固定偏差（模拟估计器高度偏 0.05 m）
        truth = self.p + np.array([0.0, 0.0, 0.05])
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


def _fly(fake, cfg=None, t_max=300.0, dt=0.02):
    m = om.Mission(cfg)
    t = 0.0
    while not m.done and t < t_max:
        fake.apply(m.step(fake.state(t)), dt)
        t += dt
    return m


def test_mission_completes_takeoff_hover_goto_land():
    m = _fly(FakePx4())
    assert m.phase == "DONE", m.metrics
    x = m.metrics
    assert x.landed_and_disarmed and x.failure is None
    assert x.hover_samples >= 400  # 10 s × 50 Hz
    assert x.hover_alt_err_rms_m < 0.05 and x.hover_alt_err_max_m < 0.2
    assert x.goto_err_m < 0.05  # 稳住后再停 2 s 取平均，不含进场段
    # 起飞前就有的固定偏差在真值口径里抵消
    assert x.hover_alt_err_truth_rms_m == pytest.approx(x.hover_alt_err_rms_m, abs=1e-9)
    assert x.goto_err_truth_m == pytest.approx(x.goto_err_m, abs=1e-9)
    np.testing.assert_allclose(m.goal_sp, [6.0, 1.0, 2.5])


def test_hover_error_reports_altitude_bias():
    # 跟踪偏高 0.1 m：仍落在 settle 门内，悬停高度误差应如实报出约 0.1 m
    m = _fly(FakePx4(bias_z=0.1))
    assert m.phase == "DONE"
    assert m.metrics.hover_alt_err_rms_m == pytest.approx(0.1, abs=0.01)


def test_mission_fails_when_arming_refused():
    cfg = om.MissionConfig(arm_timeout_s=3.0)
    m = _fly(FakePx4(arm_ok=False), cfg)
    assert m.phase == "FAIL"
    assert m.metrics.failure.startswith("ARMING")
    assert m.metrics.phase_reached == "ARMING"


def test_setpoints_go_through_core_px4_ned():
    m = om.Mission()
    s = om.VehicleState(0.0, np.zeros(3), np.zeros(3))
    act = m.step(s)
    f = px4.trajectory_setpoint(act.setpoint_pva, act.yaw_enu, 0)
    assert f["position"] == [0.0, 0.0, 0.0]
    m.home = np.zeros(3)
    pva = m._pva(np.array([5.0, 3.0, 2.5]))
    assert px4.trajectory_setpoint(pva, None, 0)["position"] == [3.0, 5.0, -2.5]


def _code_lines(path):
    out = []
    for line in path.read_text().splitlines():
        s = line.strip()
        if s.startswith("#"):
            continue
        out.append(s)
    return "\n".join(out)


def test_sitl_scripts_never_set_px4_params():
    for p in sorted(SITL.iterdir()):
        if not p.is_file() or p.suffix not in (".sh", ".py", "") or p.name == "Dockerfile":
            continue
        code = _code_lines(p)
        assert not re.search(r"PX4_PARAM_[A-Z]", code), p
        assert not re.search(r"px4-param[^\n]*\bset\b", code), p
        assert "param set" not in code, p


def test_dockerfile_pins_match_core_px4():
    d = (SITL / "Dockerfile").read_text()
    assert px4.PX4_VERSION == "v1.17.0" and "PX4_REF=d6f12ad1c4f70ad3230afd7d86e971421e02fef4" in d
    assert px4.PX4_MSGS_BRANCH == "release/1.17" and "PX4_MSGS_REF=86d8239e962f6939e05c3737784f60c02fa884db" in d
    assert f"XRCE_AGENT_REF={px4.XRCE_AGENT_VERSION}" in d


def test_shell_scripts_parse():
    for name in ("up.sh", "smoke.sh", "in_container.sh", "common.sh"):
        subprocess.run(["bash", "-n", str(SITL / name)], check=True)


def test_sitl_not_in_suite():
    assert "sitl" not in (ROOT / "scenes/suite.txt").read_text().lower()


def test_summarize_renders_metrics(tmp_path):
    res = {
        "ok": True,
        "phase": "DONE",
        "metrics": {"hover_alt_err_rms_m": 0.031, "hover_alt_err_max_m": 0.08, "goto_err_m": 0.12,
                    "takeoff_s": 6.0, "goto_s": 8.0, "land_s": 9.0},
        "mission_wall_s": 50.0,
        "mission_sim_s": 40.0,
        "rtf": 0.8,
    }
    (tmp_path / "result.json").write_text(json.dumps(res))
    text = summ.render(res, 120.0, "img")
    assert "report-only" in text and "| RTF | 0.80 |" in text and "| 到点误差 (m) | 0.120 |" in text
    assert summ.main([str(tmp_path)]) == 0
    assert summ.main([str(tmp_path / "missing")]) == 1
