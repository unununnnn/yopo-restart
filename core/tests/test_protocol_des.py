"""#5：无 ROS DES 测试器 + 有界穷举（V0.3 §7；V0.2 I1/I3/I4/I5/I6/I9）。"""

from __future__ import annotations

import json
import math

import pytest

from yopo_swarm_core.protocol import (
    AbstractOccupancy,
    Decision,
    ProtocolConfig,
    ProtocolNode,
    Reason,
    Span,
)
from yopo_swarm_core.protocol.des import (
    FAULT_CATEGORIES,
    FAULT_CLASSES,
    FaultPlan,
    Simulator,
    explore,
    fault_matrix,
)
from yopo_swarm_core.protocol.node import NodeMode


def cfg(n: int, **kw) -> ProtocolConfig:
    return ProtocolConfig(members=tuple(f"u{i}" for i in range(n)), **kw)


# --- 变异体：用来证明检查器真的会抓错 -----------------------------------------


class TimeoutReleaseNode(ProtocolNode):
    """违反 B01：本地超时就释放预留（未收到 COMMIT/RELEASE）。"""

    def on_tick(self, now):
        for peer in self.peers.values():
            for rid, r in list(peer.reservations.items()):
                if now >= r.t_start + self.cfg.window_len:
                    del peer.reservations[rid]
        return super().on_tick(now)


def greedy_planner(node, now, t_start):
    return AbstractOccupancy.of(Span(t_start, math.inf, "R0"))


class AckAllNode(ProtocolNode):
    """违反 I4/I10：投票不做冲突检查，规划不看邻机。"""

    def __init__(self, node_id, c):
        super().__init__(node_id, c, planner=greedy_planner)

    def _on_proposal(self, p, now):
        rec_cls = type(self.committed)
        rec = rec_cls(
            p.proposer_id, p.plan_seq, p.plan_hash, p.parent_plan_hash, p.t_start, p.occupancy
        )
        self.peers[p.proposer_id].reservations[p.reservation_id] = rec
        return self._vote(p, Decision.ACK, Reason.OK)


# --- 确定性与可重放 -----------------------------------------------------------


def test_same_seed_same_trace_bytes():
    faults = FaultPlan(drop=0.2, duplicate=0.2, delay=(0.0, 0.5))
    a = Simulator(cfg(3), faults, seed=7).run(until=12.0)
    b = Simulator(cfg(3), faults, seed=7).run(until=12.0)
    assert a.trace_bytes() == b.trace_bytes()
    c = Simulator(cfg(3), faults, seed=8).run(until=12.0)
    assert c.trace_bytes() != a.trace_bytes()


@pytest.mark.parametrize("n", [3, 6])
def test_smoke_no_faults_everyone_commits(n):
    r = Simulator(cfg(n), seed=1).run(until=3.0 * n + 1)
    assert r.violations == []
    for node in r.nodes.values():
        assert node.mode is NodeMode.ACTIVE
        assert node.committed.plan_seq >= 2  # 每轮自己的窗口都提交成功


def test_log_recovers_versions_proposals_and_nack_reasons():
    """I9：只读 trace 就能还原各成员状态版本、提案，以及 NACK 决策和原因。"""
    c = cfg(3)
    # 固定时延把窗口 0 的提案拖出窗口，投票方必须回 NACK_STALE。
    r = Simulator(c, FaultPlan(delay=(2.0, 2.0)), seed=0).run(until=6.0)
    last = {}
    for line in r.trace:
        if "snapshot" in line and line.get("state") in c.members:
            last[line["state"]] = json.loads(line["snapshot"])
    assert set(last) == set(c.members)
    for m, node in r.nodes.items():
        live = json.loads(node.snapshot())
        assert last[m]["boot"] == live["boot"]
        assert last[m]["clock_epoch"] == live["clock_epoch"]
        assert last[m]["task"] == live["task"]
        assert last[m]["plan_seq"] == live["plan_seq"]
        assert last[m]["committed"]["plan_hash"] == live["committed"]["plan_hash"]
    proposals = [x for x in r.trace if x.get("event") == "propose"]
    assert proposals
    assert {p["node"] for p in proposals} <= set(c.members)
    assert all({"rid", "plan_hash", "t_start"} <= p.keys() for p in proposals)
    nacks = [
        x["msg"]
        for x in r.trace
        if x.get("send")
        and x.get("msg", {}).get("kind") == "VOTE"
        and str(x["msg"].get("decision", "")).startswith("NACK")
    ]
    assert nacks
    assert {n["decision"] for n in nacks} >= {"NACK_STALE"}
    assert {n["reason"] for n in nacks} >= {"WINDOW"}
    assert all(n.get("reservation_id") and n.get("plan_hash") for n in nacks)


def test_replay_reconstructs_commits_from_log():
    """I9：仅凭日志可重建每次提交的输入（提案）与全部 ACK。"""
    c = cfg(3)
    r = Simulator(c, FaultPlan(drop=0.15, delay=(0.0, 0.3)), seed=3).run(until=15.0)
    commits = [x for x in r.trace if x.get("event") == "commit"]
    assert commits
    for cm in commits:
        owner = cm["node"]
        proposals = [
            x
            for x in r.trace
            if x.get("send") == owner
            and x["msg"]["kind"] == "PROPOSE"
            and x["msg"]["reservation_id"] == cm["rid"]
        ]
        assert len(proposals) == 1 and proposals[0]["msg"]["plan_hash"] == cm["plan_hash"]
        ackers = {
            x["msg"]["ctx"]["sender_id"]
            for x in r.trace
            if x.get("recv") == owner
            and x["msg"]["kind"] == "VOTE"
            and x["msg"]["reservation_id"] == cm["rid"]
            and x["msg"]["decision"] == "ACK"
            and x["t"] <= cm["t"]
        }
        assert ackers == set(c.members) - {owner}
    last = {}
    for cm in commits:
        last[cm["node"]] = cm["plan_hash"]
    for m, h in last.items():
        assert r.nodes[m].committed.plan_hash == h


# --- 故障注入 ---------------------------------------------------------------


@pytest.mark.parametrize("seed", range(25))
def test_random_fault_soak_holds_invariants(seed):
    faults = FaultPlan(drop=0.15, duplicate=0.15, delay=(0.0, 0.8))
    r = Simulator(cfg(3), faults, seed=seed).run(until=20.0)
    assert r.violations == []


def test_fault_matrix_has_every_class_and_category_and_no_collisions():
    table = fault_matrix(cfg(3), seeds=range(4), until=15.0)
    assert set(table) == set(FAULT_CLASSES)
    for name, row in table.items():
        assert set(row) == set(FAULT_CATEGORIES), name
        assert row["collision"] == 0, name
        assert row["oracle_pause"] == 0, name
    assert table["duplicate"]["detected"] > 0  # 重复被 seq 检出
    assert table["clock_reset"]["detected"] > 0
    assert table["task_reset"]["detected"] > 0
    assert table["partition"]["blocked_new_plans"] > 0
    assert table["restart"]["legal_fallback"] > 0
    assert table["none"]["blocked_new_plans"] == 0


def test_partition_halts_new_progress_within_bound():
    """I5：全分区后在有界时间内停止新的任务进度（不再提交，全部 HALTED）。"""
    c = cfg(3, max_failed_rounds=2)
    t0 = 2.0
    parts = tuple((t0, math.inf, frozenset({m})) for m in c.members)
    r = Simulator(c, FaultPlan(partitions=parts), seed=0).run(until=t0 + 10.0)
    assert r.violations == []
    late_commits = [x for x in r.trace if x.get("event") == "commit" and x["t"] > t0 + c.window_len]
    assert late_commits == []
    bound = t0 + (c.max_failed_rounds * len(c.members) + 2) * c.window_len
    halts = {x["node"]: x["t"] for x in r.trace if x.get("event") == "halt"}
    assert set(halts) == set(c.members)
    assert max(halts.values()) <= bound


def test_restart_does_not_reuse_old_acks_and_others_stay_safe():
    c = cfg(3)
    r = Simulator(c, FaultPlan(restarts=((4.3, "u2"),), drop=0.1), seed=5).run(until=15.0)
    assert r.violations == []
    assert r.nodes["u2"].mode is NodeMode.HALTED


def test_mutant_timeout_release_is_caught_by_simulator():
    c = cfg(3)
    found = False
    for seed in range(30):
        sim = Simulator(
            c, FaultPlan(drop=0.5, delay=(0.0, 0.3)), seed=seed, node_factory=TimeoutReleaseNode
        )
        r = sim.run(until=15.0)
        if any(v.invariant == "COVERAGE" for _, v in r.violations):
            found = True
            assert r.stats["oracle_pause"] == 1
            break
    assert found


# --- 有界穷举 ---------------------------------------------------------------


def test_exhaustive_two_nodes_all_faults_complete_and_safe(tmp_path):
    s = explore(cfg(2, resend_period=None), max_steps=9, max_plan_seq=1, queue_depth=3)
    assert s.violation is None
    assert s.complete
    assert s.states > 1000
    out = tmp_path / "summary.json"
    s.save(out)
    data = json.loads(out.read_text())
    for key in (
        "members",
        "max_steps",
        "max_plan_seq",
        "queue_depth",
        "faults",
        "states",
        "complete",
    ):
        assert key in data


def test_exhaustive_three_nodes_reorder_safe():
    s = explore(
        cfg(3, resend_period=None), max_steps=8, max_plan_seq=1, queue_depth=4, faults=("reorder",)
    )
    assert s.violation is None and s.complete


def test_exhaustive_finds_shortest_counterexample_for_timeout_release(tmp_path):
    c = cfg(2, resend_period=None)
    s = explore(
        c,
        max_steps=12,
        max_plan_seq=1,
        queue_depth=3,
        faults=("reorder",),
        node_factory=TimeoutReleaseNode,
    )
    assert s.violation is not None and s.violation["invariant"] == "COVERAGE"
    k = len(s.counterexample)
    assert k > 0 and any(a.startswith("DELIVER PROPOSE") for a in s.counterexample)
    out = tmp_path / "counterexample.json"
    s.save(out)
    saved = json.loads(out.read_text(encoding="utf-8"))
    assert saved["violation"]["invariant"] == "COVERAGE"
    assert saved["counterexample"] == s.counterexample
    assert saved["members"] == ["u0", "u1"]
    shorter = explore(
        c,
        max_steps=k - 1,
        max_plan_seq=1,
        queue_depth=3,
        faults=("reorder",),
        node_factory=TimeoutReleaseNode,
    )
    assert shorter.violation is None  # BFS：没有更短的反例


def test_exhaustive_finds_collision_for_ack_all_mutant():
    s = explore(
        cfg(2, resend_period=None),
        max_steps=12,
        max_plan_seq=1,
        queue_depth=3,
        faults=("reorder",),
        node_factory=AckAllNode,
    )
    # #10 新增的 I10/I11 不变量比 COLLISION 更早（BFS 更短）抓到同一缺陷
    assert s.violation is not None
    assert s.violation["invariant"] in {"ACK_BINDING", "DOUBLE_CONFIRM", "COLLISION"}


@pytest.mark.slow
@pytest.mark.parametrize(
    ("n", "steps", "plan_seq", "depth", "faults"),
    [
        (3, 14, 1, 4, ("reorder", "drop")),
        (2, 14, 2, 3, ("drop", "duplicate", "reorder")),
    ],
)
def test_exhaustive_larger_bounds(n, steps, plan_seq, depth, faults, tmp_path):
    s = explore(
        cfg(n, resend_period=None),
        max_steps=steps,
        max_plan_seq=plan_seq,
        queue_depth=depth,
        faults=faults,
    )
    s.save(tmp_path / "summary.json")
    assert s.violation is None and s.complete and s.states > 10_000


class ForgetfulProposerNode(ProtocolNode):
    """违反“每机最多一个在途提案”：不结案就丢掉在途提案，下一个窗口再提。"""

    def on_tick(self, now):
        if self.in_flight is not None and now >= self.in_flight.record.t_start - 0.5:
            self.in_flight = None
        return super().on_tick(now)


def test_single_inflight_invariant_catches_forgetful_proposer():
    c = cfg(2, resend_period=None)
    r = Simulator(c, FaultPlan(drop=1.0), seed=0, node_factory=ForgetfulProposerNode).run(
        until=10.0
    )
    assert any(v.invariant == "SINGLE_INFLIGHT" for _, v in r.violations)
    ok = Simulator(c, FaultPlan(drop=1.0), seed=0).run(until=10.0)
    assert ok.violations == []
