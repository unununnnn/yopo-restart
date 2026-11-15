"""票 B1：仿真钟锁步。假世界、假墙钟，不起 Gazebo，也不连 ROS。"""

import json
import os
import subprocess
import sys

import pytest

from conftest import ROOT
from yopo_l0.sitl_lockstep import (
    LockstepStepper,
    SimStampSkewMeter,
    count_sensor_timeouts,
    model_name,
    motor_topic,
    quantize,
    split_time,
)

STEP = 0.004
CROSS_SITL = ROOT / "scenes/crossing-pillars-sitl80.json"
G2 = ROOT / "scenes/g2-pillar.json"


class VirtualWall:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += max(float(dt), 1e-4)


class FakeWorld:
    """gz 世界 + 若干 PX4：run_to_sim_time 生效后 clock 跳到目标，每机按自己的滞后出电机指令。

    ``lose_every``：每隔几次请求丢一次（请求没生效，要靠重发）。
    ``lag_polls``：某机出电机指令前要多少次 motor_counts() 轮询（模拟 PX4 算得慢）。
    ``dead``：这些机永远不出电机指令。
    ``free_run``：暂停请求生效前 clock 还会再走这么多步（engage 要等它停稳）。
    """

    def __init__(self, namespaces, *, t0=12.0, lose_every=0, lag_polls=None, dead=(), free_run=0):
        self.t = t0
        self.target = t0
        self.paused = False
        self.free_run = free_run
        self.requests = 0
        self.lose_every = lose_every
        self.counts = {ns: 0 for ns in namespaces}
        self.owed = {ns: 0 for ns in namespaces}
        self.lag_polls = dict(lag_polls or {})
        self.wait = {ns: 0 for ns in namespaces}
        self.dead = set(dead)
        self.history = []

    def sim_time(self):
        if self.free_run > 0:  # 暂停请求生效前 clock 还会再走几步
            self.free_run -= 1
            self.t = round(self.t + STEP, 9)
            for ns in self.owed:
                self.owed[ns] += 1
        return self.t

    def motor_counts(self):
        for ns in self.counts:
            if ns in self.dead or self.owed[ns] == 0:
                continue
            self.wait[ns] += 1
            if self.wait[ns] > self.lag_polls.get(ns, 0):
                self.counts[ns] += 1
                self.owed[ns] -= 1
                self.wait[ns] = 0
        return dict(self.counts)

    def _lost(self):
        self.requests += 1
        return self.lose_every and self.requests % self.lose_every == 0

    def request_pause(self):
        if not self._lost():
            self.paused = True

    def request_run_to(self, t):
        if self._lost():
            return
        self.paused = True
        if t > self.t + 1e-12:
            steps = int(round((t - self.t) / STEP))
            self.t = round(t, 9)
            for ns in self.owed:
                self.owed[ns] += steps
        self.history.append(t)

    def request_resume(self):
        self.paused = False


def _stepper(world, ns, **kw):
    wall = VirtualWall()
    return LockstepStepper(world, ns, physics_step_s=STEP, now=wall.now, sleep=wall.sleep, **kw), wall


def test_engage_waits_for_the_clock_to_stop_then_steps_exactly_on_the_grid():
    ns = ["px4_1", "px4_2", "px4_3"]
    world = FakeWorld(ns, t0=10.0, free_run=7)
    stepper, _ = _stepper(world, ns)
    t0 = stepper.engage()
    assert world.paused and t0 == pytest.approx(10.0 + 7 * STEP)
    for _ in range(50):
        stepper.advance(0.02)
    assert stepper.t == pytest.approx(t0 + 1.0)
    assert world.t == pytest.approx(t0 + 1.0)
    # 每次只走一个物理步：每个目标比上一个多 4 ms
    diffs = {round(b - a, 9) for a, b in zip(world.history, world.history[1:])}
    assert diffs == {STEP}
    s = stepper.stats.summary()
    assert s["steps"] == 250 and s["sim_stepped_s"] == pytest.approx(1.0)
    assert s["handshake_timeouts_total"] == 0
    # 暂停前在途的 7 条先收完再记基线，之后每步恰好一条
    assert world.counts == {n: 7 + 250 for n in ns}


def test_each_step_waits_for_the_slowest_px4():
    ns = ["px4_1", "px4_2"]
    world = FakeWorld(ns, lag_polls={"px4_2": 30})
    stepper, _ = _stepper(world, ns)
    stepper.engage()
    t0 = world.t
    stepper.advance(0.02)
    # 慢的那架每一步都出齐了，世界才往下走
    assert world.counts["px4_2"] == 5 and world.counts["px4_1"] == 5
    assert world.t == pytest.approx(t0 + 0.02)
    assert stepper.stats.handshake_timeouts == {}


def test_extra_motor_messages_do_not_let_a_later_step_through_early():
    ns = ["px4_1"]
    world = FakeWorld(ns)
    stepper, _ = _stepper(world, ns, handshake_timeout_s=0.01)
    stepper.engage()
    stepper.advance(STEP)
    world.counts["px4_1"] += 10
    world.dead.add("px4_1")
    with pytest.raises(TimeoutError, match="PX4 电机握手超时"):
        stepper.advance(STEP)


def test_lost_requests_are_resent_and_never_overshoot():
    ns = ["px4_1"]
    world = FakeWorld(ns, lose_every=3)
    stepper, _ = _stepper(world, ns)
    stepper.engage()
    t0 = world.t
    for _ in range(20):
        stepper.advance(0.02)
    assert stepper.stats.resends > 0
    assert stepper.stats.clock_overshoots == 0
    assert world.t == pytest.approx(t0 + 0.4)
    assert world.counts["px4_1"] == 100


def test_dead_px4_fails_closed_without_advancing_the_others_again():
    ns = ["px4_1", "px4_2"]
    world = FakeWorld(ns, dead={"px4_2"})
    stepper, _ = _stepper(world, ns, handshake_timeout_s=0.01)
    stepper.engage()
    t0 = world.t
    with pytest.raises(TimeoutError, match="px4_2"):
        stepper.advance(0.02)
    assert world.t == pytest.approx(t0 + STEP)
    assert stepper.stats.handshake_timeouts == {"px4_2": 1}
    assert stepper.stats.steps == 0


def test_slow_agent_changes_wall_time_but_not_the_sim_schedule():
    """代理每拍多睡 0.5 s：墙钟拉长、RTF 下降，仿真钟上的步进序列一模一样。"""
    ns = ["px4_1", "px4_2", "px4_3"]
    runs = []
    for agent_s in (0.0, 0.5):
        world = FakeWorld(ns, t0=20.0)
        stepper, wall = _stepper(world, ns)
        stepper.engage()
        for _ in range(25):
            wall.sleep(agent_s)  # 代理算这一拍
            stepper.advance(0.02)
        runs.append((list(world.history), stepper.stats.summary()))
    (h0, s0), (h1, s1) = runs
    assert h0 == h1
    assert s0["steps"] == s1["steps"] == 125
    assert s1["agent_wall_s"] > s0["agent_wall_s"] + 10.0
    assert s1["rtf_locked"] < s0["rtf_locked"]


def test_advance_before_engage_is_an_error():
    stepper, _ = _stepper(FakeWorld(["a"]), ["a"])
    with pytest.raises(RuntimeError):
        stepper.advance(0.02)


def test_grid_and_time_helpers():
    assert quantize(12.0 + 3 * STEP + 1e-12, STEP) == pytest.approx(12.012)
    assert split_time(12.012) == (12, 12_000_000)
    with pytest.raises(ValueError):
        split_time(-1.0)
    assert model_name(1) == "x500_1"
    assert motor_topic(0) == "/x500_0/command/motor_speed"


def test_sensor_timeout_lines_from_px4_log():
    log = (
        "INFO  [commander] Ready for takeoff!\n"
        "ERROR [sensors] Accel #0 fail:  TIMEOUT!\t\n"
        "ERROR [sensors] Accel #0 fail:  TIMEOUT!\t\n"
        "WARN  [sensors] Gyro #0 fail: TIMEOUT!\n"
    )
    assert count_sensor_timeouts(log) == {"Accel #0": 2, "Gyro #0": 1}
    assert count_sensor_timeouts("") == {}


def test_sim_stamp_skew_uses_receipt_sim_time_only():
    m = SimStampSkewMeter()
    m.note(None, {"a": 1.0})
    m.note(10.0, {"a": 9.99, "b": 9.97, "c": None})
    m.note(10.02, {"a": 10.01, "b": 10.01})
    s = m.summary()
    assert s["inter_vehicle_spread_max_s"] == pytest.approx(0.02)
    assert s["per_vehicle"]["b"]["age_max_s"] == pytest.approx(0.03)
    assert "c" not in s["per_vehicle"]


def _live(*argv, env=None):
    e = dict(os.environ)
    e.pop("YOPO_SITL_LOCKSTEP", None)
    e.pop("YOPO_AGENT_DELAY_S", None)
    e.update(env or {})
    e["PYTHONPATH"] = f"{ROOT}:{ROOT / 'core/src'}"
    return subprocess.run(
        [sys.executable, str(ROOT / "scripts/sitl/yopo_l0_live.py"), *argv],
        cwd=ROOT, env=e, capture_output=True, text=True, check=False,
    )


def test_lockstep_is_off_by_default_in_the_dry_run(tmp_path):
    out = tmp_path / "plan.json"
    r = _live("--scene", str(G2), "--dry-run", "--out", str(out))
    assert r.returncode == 0, r.stderr
    plan = json.loads(out.read_text())
    assert plan["lockstep"] == {"enabled": False, "agent_delay_s": 0.0}


@pytest.mark.parametrize("how", ["flag", "env"])
def test_lockstep_dry_run_lists_the_handshake_topics_and_writes_no_px4_params(tmp_path, how):
    out = tmp_path / "plan.json"
    argv = ["--scene", str(CROSS_SITL), "--policy", "crossing", "--n", "3", "--dry-run", "--out", str(out)]
    env = None
    if how == "flag":
        argv.append("--lockstep")
    else:
        env = {"YOPO_SITL_LOCKSTEP": "1", "YOPO_AGENT_DELAY_S": "0.05"}
    r = _live(*argv, env=env)
    assert r.returncode == 0, r.stderr
    lock = json.loads(out.read_text())["lockstep"]
    assert lock["enabled"] is True
    assert lock["handshake_topics"] == [f"/x500_{i}/command/motor_speed" for i in (1, 2, 3)]
    assert lock["px4_parameters_written"] == []
    assert lock["physics_step_s"] == STEP
    assert "偏离上游" in lock["deviation"]
    assert lock["agent_delay_s"] == (0.05 if how == "env" else 0.0)


def test_live_rejects_negative_agent_delay(tmp_path):
    r = _live("--scene", str(G2), "--dry-run", "--out", str(tmp_path / "x.json"), "--agent-delay-s", "-1")
    assert r.returncode == 2


def _shell(*argv):
    e = dict(os.environ)
    e.pop("YOPO_SITL_LOCKSTEP", None)
    e.pop("YOPO_AGENT_DELAY_S", None)
    return subprocess.run(
        ["bash", str(ROOT / "scripts/sitl/run_yopo_l0_sitl.sh"), "--dry-run", *argv],
        cwd=ROOT, env=e, capture_output=True, text=True, check=False,
    )


def test_shell_entry_defaults_off_and_passes_lockstep_through():
    off = _shell("--n", "1")
    assert off.returncode == 0, off.stderr
    assert "lockstep=0" in off.stdout and "agent_delay_s=0" in off.stdout
    on = _shell("--n", "3", "--scene", "scenes/crossing-pillars-sitl80.json", "--policy", "crossing",
                "--lockstep", "--agent-delay-s", "0.05")
    assert on.returncode == 0, on.stderr
    assert "lockstep=1" in on.stdout and "agent_delay_s=0.05" in on.stdout
    assert _shell("--agent-delay-s", "abc").returncode == 2


def test_lockstep_does_not_touch_px4_parameters_or_frozen_files():
    """锁步只在代理侧控制 gz 世界：入口脚本不出现 param set / PX4_PARAM_，也不碰安全参数前缀。"""
    for rel in ("yopo_l0/sitl_lockstep.py", "scripts/sitl/yopo_l0_live.py"):
        text = (ROOT / rel).read_text()
        assert "param set" not in text
        assert "PX4_PARAM_" not in text
        for prefix in ("COM_", "NAV_", "BAT_"):
            assert f'"{prefix}' not in text and f"'{prefix}" not in text
