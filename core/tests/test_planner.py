"""#9：单机规划器离线纵切（感知 → 候选 → 几何验证 → 停止回退），只测入口的外部可观察行为。"""

from __future__ import annotations

import dataclasses
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from yopo_swarm_core import planner as P
from yopo_swarm_core import replay as R
from yopo_swarm_core.freespace import CameraIntrinsics, DepthFrame, FreeSpaceConfig
from yopo_swarm_core.plan import (
    Hold,
    KinematicLimits,
    OccupancySet,
    PlanConfig,
    hover_plan,
    state_at,
    validate_plan,
)
from yopo_swarm_core.synthetic import (
    Box,
    ScriptedLatticeSource,
    camera_pose_looking_x,
    render_depth,
)

K = CameraIntrinsics(fx=80.0, fy=83.0, cx=80.0, cy=48.0, width=160, height=96)
ROOM = (
    Box((-5.0, -20.0, -3.0), (40.0, 20.0, -1.5)),  # 地面
    Box((-5.0, -20.0, 4.0), (40.0, 20.0, 6.0)),  # 顶
    Box((-5.0, -20.0, -3.0), (40.0, -8.0, 6.0)),  # 右墙
    Box((-5.0, 8.0, -3.0), (40.0, 20.0, 6.0)),  # 左墙
    Box((25.0, -20.0, -3.0), (40.0, 20.0, 6.0)),  # 尽头
)
PILLAR = Box((4.0, -0.6, -1.5), (5.0, 0.6, 4.0))  # 正前方柱子
CFG = P.PlannerConfig(
    plan=PlanConfig(
        limits=KinematicLimits(v_max=4.0, a_max=8.0, j_max=80.0),
        max_branch_points=2,
        body_radius=0.15,
        tracking_margin=0.05,
    ),
    space=FreeSpaceConfig(
        body_radius=0.15,
        margin=0.05,
        max_age=0.5,
        min_range=0.1,
        max_range=30.0,
        sample_spacing=0.05,
    ),
    switch_delay=0.125,
)
GOAL = np.array([20.0, 0.0, 0.0])
DT = 0.125  # 二进制可精确表示：k·DT + switch_delay 恰好落在下一步


def make_obs(t, boxes, approved, peers=(), stamp=None, depth=None, frame_id="world-ENU", **kw):
    pos = state_at(approved, t)[0]
    T = camera_pose_looking_x(pos)
    d = render_depth(boxes, K, T, CFG.space.max_range) if depth is None else depth
    frame = DepthFrame.from_raw(d, K, T, t if stamp is None else stamp, CFG.space)
    pva = state_at(approved, t)
    state = P.VehicleState(pva[0], pva[1], pva[2], np.eye(3), t)
    return P.Observation(frame, state, GOAL, approved, tuple(peers), t, frame_id, **kw)


def new_planner(source=None, log=None):
    pl = P.Planner(CFG, source or ScriptedLatticeSource(), log=log)
    pl.assume_free(np.zeros(3), 1.0, "takeoff-pad")
    return pl


def run(boxes, steps, planner=None, log=None):
    pl = planner or new_planner(log=log)
    approved = hover_plan("uav0", np.zeros(3), 0.0, CFG.plan)
    decisions = []
    for k in range(steps):
        t = k * DT
        d = pl.step(make_obs(t, boxes, approved))
        decisions.append(d)
        if d.approved is not None:
            approved = d.approved.plan
    return decisions, approved


def clearance(plan, boxes):
    trs = [plan.motion] + [b.traj for b in plan.branches]
    return min(
        bx.distance(tr.evaluate(float(t)))
        for tr in trs
        for t in np.linspace(tr.t_start, tr.t_end, 200)
        for bx in boxes
    )


# --- 合成场景序列 ----------------------------------------------------------------


@pytest.fixture(scope="module")
def pillar_run():
    return run((*ROOM, PILLAR), 20)


def test_scenario_sequence_all_approved_plans_pass_every_check(pillar_run):
    decisions, _ = pillar_run
    approved = [d for d in decisions if d.approved is not None]
    assert len(approved) >= 10
    for d in approved:
        plan = d.approved.plan
        assert validate_plan(plan, CFG.plan) == []
        # 测试侧用地图真值复核（规划器拿不到）：所有分支离障碍 ≥ 机体半径
        assert clearance(plan, (*ROOM, PILLAR)) >= CFG.plan.body_radius
    final = approved[-1].approved.plan
    assert final.terminal_point()[0] > 1.0  # 确实在前进


def test_pillar_forces_rejections_of_some_candidates(pillar_run):
    decisions, _ = pillar_run
    codes = {c.code for d in decisions for c in d.candidates}
    assert any(code.startswith("ENV:") for code in codes)
    assert "OK" in codes


def test_funnel_counts_monotone_every_step(pillar_run):
    decisions, _ = pillar_run
    for d in decisions:
        f = d.funnel
        assert f.decoded >= f.transition_ok >= f.env_safe >= f.peer_safe >= f.approved
        assert f.approved == (1 if d.approved is not None else 0)
        assert f.decoded == len(d.candidates)


def test_every_candidate_carries_stable_code(pillar_run):
    decisions, _ = pillar_run
    allowed = {"OK", "NOT_SELECTED", "ENV:OCCUPIED", "ENV:UNKNOWN", "PEER:CONFLICT"}
    allowed |= {f"TRANSITION:{r.value}" for r in P.PlanReject}
    for d in decisions:
        for c in d.candidates:
            assert c.code in allowed


# --- 过期 / 空 / 无解 ----------------------------------------------------------


def started(boxes=ROOM):
    pl = new_planner()
    decisions, approved = run(boxes, 5, planner=pl)
    assert decisions[-1].approved is not None
    return pl, approved


def test_stale_frame_rejected_and_falls_back_to_brake_branch():
    pl, approved = started()
    t = 5 * DT
    d = pl.step(make_obs(t, ROOM, approved, stamp=t - 0.6))
    assert d.diagnostic is P.Diagnostic.STALE_FRAME
    assert d.approved is None
    assert d.fallback.kind == "BRAKE_BRANCH"
    assert d.fallback.plan_hash == approved.plan_hash(CFG.plan)
    assert any(abs(b.t_branch - d.fallback.t_branch) < 1e-12 for b in approved.branches)
    assert d.fallback.t_branch >= t


def test_corridor_empty_rejected_and_falls_back_to_brake_branch():
    """D3 / 保真登记 (b)：没有候选通过预测走廊准入 → 整帧拒绝，走已认证刹车分支，不飞任何候选。"""
    pl, approved = started()
    inner = pl.source

    def blocked(obs, dec):
        params, score, radius = inner(obs, dec)
        return params, score, np.zeros_like(radius)  # μ = b = 0 → μ_lo = 0 < safe_mu

    pl.source = blocked
    t = 5 * DT
    d = pl.step(make_obs(t, ROOM, approved))
    assert d.diagnostic is P.Diagnostic.CORRIDOR_EMPTY
    assert d.approved is None and d.candidates == () and d.funnel.decoded == 0
    assert d.fallback.kind == "BRAKE_BRANCH"
    assert d.fallback.plan_hash == approved.plan_hash(CFG.plan)
    assert d.fallback.t_branch >= t


def test_empty_frame_rejected_with_legal_fallback():
    pl, approved = started()
    d = pl.step(make_obs(5 * DT, ROOM, approved, depth=np.full((K.height, K.width), np.nan)))
    assert d.diagnostic is P.Diagnostic.EMPTY_FRAME
    assert d.approved is None and d.fallback.kind in {"BRAKE_BRANCH", "KEEP_APPROVED"}


def test_no_solution_wall_rejected_and_keeps_approved_plan():
    wall = Box((0.9, -20.0, -3.0), (1.2, 20.0, 6.0))
    pl = new_planner()
    approved = hover_plan("uav0", np.zeros(3), 0.0, CFG.plan)
    d = pl.step(make_obs(0.0, (*ROOM, wall), approved))
    assert d.diagnostic is P.Diagnostic.NO_ENV_SAFE_CANDIDATE
    assert d.approved is None and d.fallback.kind == "KEEP_APPROVED"
    assert d.funnel.transition_ok > 0 and d.funnel.env_safe == 0


def test_peer_blocking_everything_gives_no_peer_safe_candidate():
    pl = new_planner()
    approved = hover_plan("uav0", np.zeros(3), 0.0, CFG.plan)
    blocker = OccupancySet((Hold((1.0, 0.0, 0.0), 3.0, 0.0),))
    d = pl.step(make_obs(0.0, ROOM, approved, peers=[P.PeerOccupancy("uav1", 1, blocker)]))
    assert d.diagnostic is P.Diagnostic.NO_PEER_SAFE_CANDIDATE
    assert d.funnel.env_safe > 0 and d.funnel.peer_safe == 0


def test_infeasible_dynamics_gives_no_transition_candidate():
    tight = dataclasses.replace(
        CFG, plan=dataclasses.replace(CFG.plan, limits=KinematicLimits(0.05, 0.05, 0.05))
    )
    pl = P.Planner(tight, ScriptedLatticeSource())
    pl.assume_free(np.zeros(3), 1.0, "takeoff-pad")
    approved = hover_plan("uav0", np.zeros(3), 0.0, tight.plan)
    d = pl.step(make_obs(0.0, ROOM, approved))
    assert d.diagnostic is P.Diagnostic.NO_TRANSITION_CANDIDATE
    assert d.funnel.transition_ok == 0


# --- 输入校验 -------------------------------------------------------------------


def test_unknown_frame_rejected():
    pl = new_planner()
    approved = hover_plan("uav0", np.zeros(3), 0.0, CFG.plan)
    d = pl.step(make_obs(0.0, ROOM, approved, frame_id="map"))
    assert d.diagnostic is P.Diagnostic.UNKNOWN_FRAME and d.approved is None


@pytest.mark.parametrize("field", ["position", "velocity", "goal"])
def test_non_finite_input_rejected(field):
    pl = new_planner()
    approved = hover_plan("uav0", np.zeros(3), 0.0, CFG.plan)
    obs = make_obs(0.0, ROOM, approved)
    if field == "goal":
        obs = dataclasses.replace(obs, goal=np.array([np.nan, 0, 0]))
    else:
        bad = getattr(obs.state, field).copy()
        bad[0] = np.inf
        obs = dataclasses.replace(obs, state=dataclasses.replace(obs.state, **{field: bad}))
    d = pl.step(obs)
    assert d.diagnostic is P.Diagnostic.INPUT_MALFORMED and d.approved is None


def test_non_finite_network_output_rejected():
    pl = new_planner(source=ScriptedLatticeSource(poison=True))
    approved = hover_plan("uav0", np.zeros(3), 0.0, CFG.plan)
    d = pl.step(make_obs(0.0, ROOM, approved))
    assert d.diagnostic is P.Diagnostic.NET_OUTPUT_MALFORMED and d.approved is None


def test_ground_truth_map_has_no_entry_point():
    names = {f.name for f in dataclasses.fields(P.Observation)}
    assert names == {
        "frame",
        "state",
        "goal",
        "approved",
        "peers",
        "now",
        "frame_id",
        "active_branch",
    }
    with pytest.raises(TypeError):
        P.Observation(None, None, GOAL, None, (), 0.0, obstacles=[PILLAR])  # type: ignore[call-arg]
    src = Path(P.__file__).read_text(encoding="utf-8")
    assert "synthetic" not in src and "Box" not in src


def test_core_planner_does_not_import_rclpy():
    code = "import sys, yopo_swarm_core.planner; print('rclpy' in sys.modules)"
    src = str(Path(P.__file__).resolve().parents[1])
    env = {**os.environ, "PYTHONPATH": src}
    out = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=True, env=env
    )
    assert out.stdout.strip() == "False"


# --- I7：没有绕过批准门的路径 ----------------------------------------------------


def test_approved_plan_cannot_be_constructed_outside_gate():
    plan = hover_plan("uav0", np.zeros(3), 0.0, CFG.plan)
    with pytest.raises(PermissionError):
        P.ApprovedPlan(object(), plan, "x")


def test_decision_refuses_raw_plan():
    plan = hover_plan("uav0", np.zeros(3), 0.0, CFG.plan)
    with pytest.raises(TypeError):
        P.Decision(0, P.Diagnostic.APPROVED, plan, None, P.Funnel(1, 1, 1, 1, 1), ())


def test_gate_rejection_blocks_output_even_if_candidates_pass():
    class RejectAll(P.SafetyGate):
        def validate(self, *a, **k):
            return P.GateReject(("TEST_REJECT",))

    pl = P.Planner(CFG, ScriptedLatticeSource(), gate=RejectAll(CFG))
    pl.assume_free(np.zeros(3), 1.0, "takeoff-pad")
    approved = hover_plan("uav0", np.zeros(3), 0.0, CFG.plan)
    d = pl.step(make_obs(0.0, ROOM, approved))
    assert d.approved is None and d.diagnostic is P.Diagnostic.GATE_REJECTED
    assert d.funnel.peer_safe > 0 and d.funnel.approved == 0


def test_gate_independently_rejects_unsafe_plan():
    """门自己重查：给它一个穿柱子的计划，它拒绝（不信任调用方的前序检查）。"""
    pl = new_planner()
    approved = hover_plan("uav0", np.zeros(3), 0.0, CFG.plan)
    obs = make_obs(0.0, (*ROOM, PILLAR), approved)
    pl.step(obs)  # 让门看到这一帧
    bad = P.straight_plan(approved, obs.now + CFG.switch_delay, np.array([6.0, 0, 0]), CFG)
    assert not isinstance(bad, list)
    v = pl.gate.validate(bad, approved, None, pl.space, (), obs.now)
    assert isinstance(v, P.GateReject)
    assert {"ENV:OCCUPIED", "ENV:UNKNOWN"} & set(v.codes)


# --- I9：日志与重放 --------------------------------------------------------------


def test_replay_is_bitwise_identical_and_header_records_provenance(tmp_path):
    path = tmp_path / "run.jsonl"
    with R.ReplayLog(path, CFG, code_sha="deadbeef", source=ScriptedLatticeSource()) as log:
        run((*ROOM, PILLAR), 8, log=log)
    lines = path.read_text(encoding="utf-8").splitlines()
    header = json.loads(lines[0])
    assert header["kind"] == "header"
    assert header["code_sha"] == "deadbeef"
    assert header["config_hash"] == CFG.config_hash()
    assert "takeoff-pad" in json.dumps(header["assumptions"], ensure_ascii=False)
    assert "固定成员" in header["assumptions"]["run"]
    assert header["source"]["name"] == "scripted-lattice"
    replayed = R.replay(path, CFG)
    logged = [json.loads(x)["decision"] for x in lines[1:]]
    assert [json.dumps(r, sort_keys=True) for r in replayed] == [
        json.dumps(x, sort_keys=True) for x in logged
    ]
    assert len(replayed) == 8
    # 日志能恢复决策依据：每步有每个候选的错误码与漏斗
    assert all("candidates" in x and "funnel" in x for x in logged)


def test_replay_detects_config_mismatch(tmp_path):
    path = tmp_path / "run.jsonl"
    with R.ReplayLog(path, CFG, code_sha="x", source=ScriptedLatticeSource()) as log:
        run(ROOM, 2, log=log)
    other = dataclasses.replace(CFG, switch_delay=0.2)
    with pytest.raises(ValueError, match="config_hash"):
        R.replay(path, other)
