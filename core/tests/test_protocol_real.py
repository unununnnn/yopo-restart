"""#10：协议接入真实计划占用（ADR-0002）+ B01/B04 反例回归 + I10–I14 检查。

每个反例都配一个「变异体」对照：去掉对应安全规则后，同一脚本必须被参考检查器判为违反，
证明用例不是空转。
"""

from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from yopo_swarm_core.plan import KinematicLimits, OccupancySet, PlanConfig, state_at
from yopo_swarm_core.protocol import Commit, Decision, ProtocolConfig, ProtocolNode, Vote
from yopo_swarm_core.protocol.des import Simulator, check_invariants, explore
from yopo_swarm_core.protocol.node import PlanRecord, plan_hash_of
from yopo_swarm_core.protocol.plan_occupancy import (
    PlanConflictChecker,
    PlanOccupancy,
    RealScenario,
    hover_occupancy,
)

PCFG = PlanConfig(
    KinematicLimits(4.0, 6.0, 40.0), max_branch_points=2, body_radius=0.15, tracking_margin=0.05
)
R = PCFG.tube_radius
REF = PlanConflictChecker(physical=True)  # 参考检查器：总按计划本身的完整占用判定


def scenario(
    starts,
    routes,
    *,
    duration=4.0,
    offsets=(0.0, 0.5),
    members=None,
    occupancy_cls=PlanOccupancy,
    **kw,
) -> RealScenario:
    members = members or tuple(starts)
    cfg = ProtocolConfig(members=members, resend_period=None, **kw)
    return RealScenario(
        cfg,
        PCFG,
        starts,
        routes,
        duration=duration,
        branch_offsets=offsets,
        occupancy_cls=occupancy_cls,
    )


class Net:
    """脚本化网络：显式 tick / 投递 / 丢弃，每步跑参考不变量。"""

    def __init__(self, sc: RealScenario, node_cls=ProtocolNode, checker=None) -> None:
        self.sc = sc
        self.nodes = {m: sc.node(m, node_cls=node_cls, checker=checker) for m in sc.cfg.members}
        self.bag: list[tuple[str, object]] = []
        self.t = 0.0
        self.violations: list = []

    def _fan(self, sender, msgs):
        for m in msgs:
            dests = [m.dest] if getattr(m, "dest", None) else [x for x in self.nodes if x != sender]
            self.bag += [(d, m) for d in dests]

    def check(self):
        self.violations += check_invariants(self.nodes, self.t, checker=REF)

    def tick(self, t, who=None):
        self.t = t
        for m in who or self.nodes:
            self._fan(m, self.nodes[m].on_tick(t))
        self.check()

    def deliver(self, keep=lambda d, m: True):
        """投递当前队列中满足 keep 的消息，其余丢弃；直到队列为空。"""
        while self.bag:
            d, m = self.bag.pop(0)
            if keep(d, m):
                self._fan(d, self.nodes[d].on_message(m, self.t))
                self.check()

    def kinds(self):
        return [(d, m.kind) for d, m in self.bag]


A0, A1 = (0.0, 0.0, 0.0), (4.0, 0.0, 0.0)
C0, C1 = (2.0, -2.0, 0.0), (2.0, 2.0, 0.0)  # 与 A 航线在 (2,0,0) 交叉（时间上错开）
X0, X1 = (4.0, -2.0, 0.0), (4.0, 2.0, 0.0)  # 穿过 A 的终点 (4,0,0)：与 A 的终端保持冲突
FAR0, FAR1 = (0.0, 10.0, 0.0), (0.0, 12.0, 0.0)


# --- 占用适配 ---------------------------------------------------------------


def test_plan_occupancy_matches_plan_contract():
    sc = scenario({"A": A0, "B": C0}, {"A": [A1], "B": [C1]})
    a = sc.node("A")
    occ = sc.planner(a, 0.0, 1.0)
    assert isinstance(occ, PlanOccupancy)
    plan = occ.plan
    assert plan.t_switch == 1.0 and plan.owner == "A"
    assert [b.t_branch for b in plan.branches] == [1.0, 1.5]
    labels = {e.label for e in occ.occupancy_set().elements}
    assert {"motion", "branch0", "branch1", "branch0-hold", "branch1-hold", "terminal"} <= labels
    assert occ.end_time() == math.inf and occ.execution_end() == math.inf
    assert occ.is_finite()
    # 截断：名义运动只到 until，until 之前的分支整体保留（可能已经刹车），之后的分支与终端去掉
    cut = occ.truncated(1.2)
    labels = {e.label for e in cut.occupancy_set().elements}
    assert labels == {"motion", "branch0", "branch0-hold"}
    assert cut.execution_end() == 1.2 and cut.end_time() == math.inf
    assert cut.to_canonical() != occ.to_canonical()
    # 序列化稳定
    assert json.dumps(occ.to_canonical(), sort_keys=True) == json.dumps(
        sc.planner(sc.node("A"), 0.0, 1.0).to_canonical(), sort_keys=True
    )


def test_hover_terminal_holds_until_replaced():
    h = hover_occupancy("A", A0, PCFG)
    other = hover_occupancy("B", (0.3, 0.0, 0.0), PCFG)  # 0.3 < 2R=0.4
    far = hover_occupancy("B", (0.5, 0.0, 0.0), PCFG)
    chk = PlanConflictChecker()
    assert chk.conflicts(h, other)
    assert not chk.conflicts(h, far)


def test_non_finite_or_contract_violating_plan_is_malformed():
    sc = scenario({"A": A0, "B": C0}, {"A": [A1], "B": [C1]})
    occ = sc.planner(sc.node("A"), 0.0, 1.0)
    bad = replace(occ, plan=replace(occ.plan, head_pva=np.full((3, 3), np.nan)))
    assert not bad.is_finite()
    nobranch = replace(occ, plan=replace(occ.plan, branches=()))
    assert not nobranch.is_finite()  # 无制动分支 = 不满足 ADR-0002 契约


def test_checker_survives_deepcopy_and_caches():
    import copy

    chk = PlanConflictChecker()
    assert copy.deepcopy(chk) is chk
    sc = scenario({"A": A0, "B": C0}, {"A": [A1], "B": [C1]})
    occ = sc.planner(sc.node("A"), 0.0, 1.0)
    assert copy.deepcopy(occ) is occ


def test_checker_cache_is_bounded_and_stays_correct(monkeypatch):
    monkeypatch.setattr(PlanConflictChecker, "cache_max", 4)
    monkeypatch.setattr(PlanConflictChecker, "_cache", type(PlanConflictChecker._cache)())
    chk = PlanConflictChecker()
    h = hover_occupancy("A", A0, PCFG)
    for i in range(12):
        x = 0.3 if i % 2 else 0.5 + 0.1 * i  # 奇数冲突、偶数分离
        b = hover_occupancy("B", (x, 0.0, 0.0), PCFG)
        assert chk.conflicts(h, b) is (i % 2 == 1)
        assert len(PlanConflictChecker._cache) <= 4
    assert chk.conflicts(h, hover_occupancy("B", (0.3, 0.0, 0.0), PCFG))


# --- 正常流程：真实占用下双机交叉 -------------------------------------------


def test_crossing_pair_time_separated_both_commit():
    sc = scenario({"A": A0, "B": C0}, {"A": [A1], "B": [C1]})
    net = Net(sc)
    net.tick(0.0)  # 窗口 0 属 A
    assert net.kinds() == [("B", "PROPOSE")]
    net.deliver()
    a, b = net.nodes["A"], net.nodes["B"]
    assert a.committed.plan_seq == 1
    assert b.peers["A"].plans[-1].plan_hash == a.committed.plan_hash
    net.tick(1.0)  # 窗口 1 属 B：A 在 t=3 过交叉点，B 在 t=4 才到 → 分离，可批准
    net.deliver()
    assert b.committed.plan_seq == 1
    assert not REF.conflicts(a.committed.occupancy, b.committed.occupancy)
    assert not net.violations


def test_route_through_peer_terminal_is_skipped():
    sc = scenario({"A": A0, "B": X0}, {"A": [A1], "B": [X1]})
    net = Net(sc)
    net.tick(0.0)
    net.deliver()
    for k in range(1, 8):
        net.tick(float(k))
        net.deliver()
    b = net.nodes["B"]
    assert b.committed.plan_seq == 0 and b.in_flight is None
    assert any(e["event"] == "skip" for e in b.events)
    assert not net.violations


# --- B01：COMMIT 部分丢失（I12） --------------------------------------------


def _b01(node_cls=ProtocolNode):
    sc = scenario(
        {"A": A0, "B": FAR0, "C": X0}, {"A": [A1], "B": [FAR1], "C": [X1]}, max_failed_rounds=99
    )
    net = Net(sc, node_cls=node_cls)
    net.tick(0.0, who=["A"])
    # 投票全部送达；A 的 COMMIT 只到 B，丢给 C
    net.deliver(keep=lambda d, m: not (isinstance(m, Commit) and d == "C"))
    assert net.nodes["A"].committed.plan_seq == 1
    for k in range(1, 4):
        net.tick(float(k))
        net.deliver(keep=lambda d, m: not (isinstance(m, Commit) and d == "C"))
    return net


def test_b01_commit_partial_loss_c_never_approves_conflict():
    net = _b01()
    a, c = net.nodes["A"], net.nodes["C"]
    a_plan = a.committed
    # C 没收到 COMMIT，但仍把 A 的新计划当作可能执行（预留不释放）
    assert a_plan.plan_hash in c.possible_plan_hashes("A", 3.0)
    # C 在自己的窗口（t=2）没有提出/批准与 A 新计划任何分支冲突的计划
    assert not [e for e in c.events if e["event"] == "propose"]
    assert any(e["event"] == "skip" for e in c.events)
    assert c.committed.plan_seq == 0
    assert not REF.conflicts(c.committed.occupancy, a_plan.occupancy)
    assert not net.violations


class ForgetfulNode(ProtocolNode):
    """变异体：预留在 t_start 到达时按「锁超时」本地释放（B01/I13 禁止的做法）。"""

    def on_tick(self, now):
        for p in self.peers.values():
            p.reservations = {k: r for k, r in p.reservations.items() if r.t_start > now}
        return super().on_tick(now)


def test_b01_mutant_lock_timeout_is_caught():
    net = _b01(ForgetfulNode)
    kinds = {v.invariant for v in net.violations}
    assert "COVERAGE" in kinds


def _b01_vote(node_cls=ProtocolNode):
    """A 提交，COMMIT 只到 B；随后 B 提出穿过 A 终点的计划，交给没收到 COMMIT 的 C 投票。"""
    sc = scenario(
        {"A": A0, "B": X0, "C": FAR0}, {"A": [A1], "B": [X1], "C": [FAR1]}, max_failed_rounds=99
    )
    net = Net(sc, node_cls=node_cls)
    net.tick(0.0, who=["A"])
    net.deliver(keep=lambda d, m: not (isinstance(m, Commit) and d == "C"))
    b, c = net.nodes["B"], net.nodes["C"]
    t_start = 2.0  # B 的窗口 1 → 下一窗口开始
    occ = sc.planner._to(b, state_at(b.committed.occupancy.plan, t_start), t_start, np.array(X1))
    assert occ is not None and REF.conflicts(occ, net.nodes["A"].committed.occupancy)
    prop = b.make_proposal(1.0, occ)
    (vote,) = c.on_message(prop, 1.0)
    return vote


def test_b01_voter_without_commit_nacks_conflicting_proposal():
    vote = _b01_vote()
    assert isinstance(vote, Vote) and vote.decision is Decision.NACK_CONFLICT


def test_b01_voter_mutant_lock_timeout_acks_conflict():
    assert _b01_vote(ForgetfulVoter).decision is Decision.ACK


class ForgetfulVoter(ProtocolNode):
    """变异体：投票时把 t_start 已到的预留当作过期丢掉（B01 禁止）。"""

    def on_message(self, msg, now):
        for p in self.peers.values():
            p.reservations = {k: r for k, r in p.reservations.items() if r.t_start > now}
        return super().on_message(msg, now)


# --- 超时释放（I13） --------------------------------------------------------


def _timeout(node_cls=ProtocolNode):
    sc = scenario({"A": A0, "B": X0}, {"A": [A1], "B": [X1]}, max_failed_rounds=99)
    net = Net(sc, node_cls=node_cls)
    net.tick(0.0)
    never_commit = lambda d, m: not isinstance(m, Commit)
    net.deliver(keep=never_commit)
    for k in range(1, 6):
        net.tick(float(k))
        net.deliver(keep=never_commit)
    return net


def test_timeout_does_not_release_reservation_while_peer_may_execute():
    net = _timeout()
    a, b = net.nodes["A"], net.nodes["B"]
    assert a.committed.plan_seq == 1
    rid = "A/0/1"
    assert rid in b.peers["A"].reservations  # 远超 t_start 仍保留
    assert b.committed.plan_seq == 0  # B 的交叉直飞一直被挡住
    assert not net.violations


def test_timeout_mutant_is_caught():
    net = _timeout(ForgetfulNode)
    assert {"COVERAGE"} <= {v.invariant for v in net.violations}


# --- 回退交叉（I14）：名义轨迹分离、制动分支相交 ----------------------------


def _overshoot(sc: RealScenario, start, goal) -> float:
    n = sc.node("A")
    occ = sc.planner(n, 0.0, 1.0)
    # 名义终点 vs 末个制动分支停点在前进方向上的超越量
    d = np.subtract(goal, start) / np.linalg.norm(np.subtract(goal, start))
    stop = occ.plan.branches[-1].stop_point()
    return float(np.dot(stop - np.asarray(goal), d))


class NominalOnly(PlanOccupancy):
    """变异体：只声明名义运动 + 终端（漏掉制动分支）。"""

    def occupancy_set(self) -> OccupancySet:
        full = super().occupancy_set()
        return OccupancySet(tuple(e for e in full.elements if not e.label.startswith("branch")))


def _branch_cross(occ_cls=PlanOccupancy):
    # T=2 s，分支点在 τ=0.6：此时制动停点越过名义终点约 4 cm
    probe = scenario(
        {"A": A0, "B": (9.0, 0, 0)},
        {"A": [(4.0, 0, 0)], "B": [(9.0, 0, 0)]},
        duration=2.0,
        offsets=(0.0, 1.2),
    )
    o = _overshoot(probe, A0, (4.0, 0, 0))
    assert o > 0.02, o
    gap = o  # 名义终点间距 2R+gap（分离），两分支停点间距 2R+gap-2o < 2R（相交）
    a1 = (4.0, 0.0, 0.0)
    b1 = (4.0 + 2 * R + gap, 0.0, 0.0)
    b0 = (b1[0] + 4.0, 0.0, 0.0)
    sc = scenario(
        {"A": A0, "B": b0},
        {"A": [a1], "B": [b1]},
        duration=2.0,
        offsets=(0.0, 1.2),
        occupancy_cls=occ_cls,
    )
    net = Net(sc)
    net.tick(0.0, who=["A"])  # 窗口 0：A 提出向 +x 冲刺
    net.deliver()
    net.tick(1.0, who=["B"])  # 窗口 1：B 迎面冲刺到 A 终点外 2R+gap
    net.deliver()
    return net, (a1, b1)


def test_branch_crossing_pair_is_never_both_approved():
    net, (a1, b1) = _branch_cross()
    a, b = net.nodes["A"], net.nodes["B"]
    assert a.committed.plan_seq == 1
    assert b.committed.plan_seq == 0  # A 的分支已在 B 的视图里 → B 不提（或被拒）
    # 对照：两机名义轨迹 + 终端本身是分离的，冲突只来自制动分支
    probe = scenario(
        {"A": A0, "B": (b1[0] + 4.0, 0, 0)},
        {"A": [a1], "B": [b1]},
        duration=2.0,
        offsets=(0.0, 1.2),
        occupancy_cls=NominalOnly,
    )
    nb = probe.planner(probe.node("B"), 1.0, 2.0)
    na = NominalOnly(a.committed.occupancy.plan, PCFG)
    assert not PlanConflictChecker().conflicts(na, nb)  # 只看名义：分离
    assert REF.conflicts(na, nb)  # 含制动分支：相交
    assert not net.violations


def test_branch_crossing_mutant_nominal_only_is_caught():
    net, _ = _branch_cross(NominalOnly)
    assert net.nodes["B"].committed.plan_seq == 1  # 变异体：只看名义轨迹 → 双双批准
    kinds = {v.invariant for v in net.violations}
    assert "COLLISION" in kinds
    assert "DOUBLE_CONFIRM" in kinds  # I10：双方都把冲突计划当成已全员确认


# --- 终点占用（I14） ---------------------------------------------------------


class NoTerminal(PlanOccupancy):
    """变异体：漏掉终端保持。"""

    def occupancy_set(self) -> OccupancySet:
        full = super().occupancy_set()
        return OccupancySet(tuple(e for e in full.elements if e.label != "terminal"))


def _terminal(occ_cls=PlanOccupancy):
    # B 先从 (2,-2) 飞到 A 航线上的 (2,0) 并停住；之后 A 想直飞穿过
    sc = scenario(
        {"A": A0, "B": C0},
        {"A": [A1], "B": [(2.0, 0.0, 0.0)]},
        members=("B", "A"),
        occupancy_cls=occ_cls,
    )
    net = Net(sc)
    for k in range(9):
        net.tick(float(k))
        net.deliver()
    return net


def test_terminal_occupancy_blocks_conflicting_plan():
    net = _terminal()
    a, b = net.nodes["A"], net.nodes["B"]
    assert b.committed.plan_seq == 1 and b.committed.occupancy.plan.t_end <= 5.0
    assert a.committed.plan_seq == 0
    assert not net.violations


def test_terminal_mutant_is_caught():
    net = _terminal(NoTerminal)
    assert net.nodes["A"].committed.plan_seq == 1
    assert "COLLISION" in {v.invariant for v in net.violations}


# --- I11：已发出的 ACK 约束投票方 --------------------------------------------


def test_sent_ack_constrains_voter_motion_and_fallback():
    sc = scenario({"A": A0, "B": X0}, {"A": [A1], "B": [X1]})
    net = Net(sc)
    net.tick(0.0)
    # B 的 ACK 已发出，但 A 收不到任何东西（A 之后会超时 abort，RELEASE 也丢）
    net.deliver(keep=lambda d, m: d == "B")
    b = net.nodes["B"]
    assert "A/0/1" in b.peers["A"].reservations
    net.tick(1.0)  # B 的窗口：直飞与已 ACK 的预留冲突
    net.bag.clear()
    assert b.in_flight is None and b.committed.plan_seq == 0
    assert any(e["event"] == "skip" and e["reason"] == "NO_CANDIDATE" for e in b.events)
    assert not net.violations


def _i11_routes():
    """名义分离、制动相交的迎面目标，加上一个名义管就撞上预留的目标。"""
    probe = scenario(
        {"A": A0, "B": (9.0, 0.0, 0.0)},
        {"A": [(4.0, 0.0, 0.0)], "B": [(9.0, 0.0, 0.0)]},
        duration=2.0,
        offsets=(0.0, 1.2),
    )
    gap = _overshoot(probe, A0, (4.0, 0.0, 0.0))
    brake = (4.0 + 2 * R + gap, 0.0, 0.0)
    # 4.3 落在制动停点内侧：名义终点本身与 A 的预留相交，且 2 s 内运动学可生成。
    motion = (4.3, 0.0, 0.0)
    start = (brake[0] + 4.0, 0.0, 0.0)
    return brake, motion, start


def test_ack_blocks_motion_route_and_brake_only_route():
    """I11：同一份已发出的 ACK 既拒绝名义运动冲突，也拒绝只在制动分支上相交的回退。"""
    brake, motion, start = _i11_routes()
    sc = scenario(
        {"A": A0, "B": start},
        {"A": [(4.0, 0.0, 0.0)], "B": [brake, motion]},
        duration=2.0,
        offsets=(0.0, 1.2),
    )
    net = Net(sc)
    net.tick(0.0, who=["A"])
    net.deliver(keep=lambda d, m: d == "B")
    b = net.nodes["B"]
    rec = b.peers["A"].reservations["A/0/1"]
    head = state_at(b.committed.occupancy.plan, 2.0)
    brake_occ = sc.planner._to(b, head, 2.0, np.array(brake))
    motion_occ = sc.planner._to(b, head, 2.0, np.array(motion))
    assert brake_occ is not None and motion_occ is not None
    nominal = PlanConflictChecker()
    brake_nom = NominalOnly(brake_occ.plan, PCFG)
    motion_nom = NominalOnly(motion_occ.plan, PCFG)
    reserved_nom = NominalOnly(rec.occupancy.plan, PCFG)
    assert not nominal.conflicts(brake_nom, reserved_nom)
    assert REF.conflicts(brake_occ, rec.occupancy)
    assert nominal.conflicts(motion_nom, reserved_nom)
    assert b.conflicts_with_neighbors(brake_occ)
    assert b.conflicts_with_neighbors(motion_occ)
    net.tick(1.0, who=["B"])
    assert b.in_flight is None and b.committed.plan_seq == 0
    assert not REF.conflicts(b.committed.occupancy, rec.occupancy)
    assert not net.violations

    # 漏掉制动分支的变异体把第一条（名义分离）路线提出去，参考检查器仍判冲突。
    sc_bad = scenario(
        {"A": A0, "B": start},
        {"A": [(4.0, 0.0, 0.0)], "B": [brake, motion]},
        duration=2.0,
        offsets=(0.0, 1.2),
        occupancy_cls=NominalOnly,
    )
    bad = Net(sc_bad)
    bad.tick(0.0, who=["A"])
    bad.deliver(keep=lambda d, m: d == "B")
    bad.tick(1.0, who=["B"])
    inflight = bad.nodes["B"].in_flight
    assert inflight is not None
    assert np.linalg.norm(inflight.record.occupancy.plan.terminal_point() - np.array(brake)) < 1e-6
    reserved = bad.nodes["B"].peers["A"].reservations["A/0/1"]
    assert REF.conflicts(inflight.record.occupancy, reserved.occupancy)


class AckIgnoringNode(ProtocolNode):
    """变异体：规划时只看邻机已提交计划，忽略自己发出的 ACK。"""

    def conflicts_with_neighbors(self, occ):
        return any(
            self.checker.conflicts(occ, r.occupancy) for p in self.peers.values() for r in p.plans
        )


def test_ack_binding_mutant_is_caught():
    sc = scenario({"A": A0, "B": X0}, {"A": [A1], "B": [X1]})
    net = Net(sc, node_cls=AckIgnoringNode)
    net.tick(0.0)
    net.deliver(keep=lambda d, m: d == "B")
    net.tick(1.0)
    assert net.nodes["B"].in_flight is not None
    assert "ACK_BINDING" in {v.invariant for v in net.violations}


# --- I10 / I1 / I3 / I6 在真实占用下仍成立 ----------------------------------


def test_vote_binds_plan_hash_validity_and_dependency():
    sc = scenario({"A": A0, "B": C0}, {"A": [A1], "B": [C1]})
    net = Net(sc)
    net.tick(0.0)
    d, prop = net.bag.pop(0)
    ((_, vote),) = [(a, m) for a, m in [(d, x) for x in net.nodes[d].on_message(prop, 0.0)]]
    assert isinstance(vote, Vote)
    a = net.nodes["A"]
    p = a.in_flight.proposal
    assert vote.decision is Decision.ACK
    assert (vote.plan_hash, vote.validity, vote.dependency_digest) == (
        p.plan_hash,
        p.validity,
        p.dependency_digest,
    )
    # I6：plan_hash、有效期、邻机版本摘要任一被改都不能复用
    a.on_message(replace(vote, plan_hash="0" * 64), 0.1)
    assert a.committed.plan_seq == 0
    a.on_message(
        replace(
            vote,
            dependency_digest="0" * 64,
            ctx=replace(vote.ctx, message_seq=vote.ctx.message_seq + 1),
        ),
        0.12,
    )
    assert a.committed.plan_seq == 0 and a.rejections[-1].value == "VOTE_MISMATCH"
    a.on_message(
        replace(
            vote,
            validity=(vote.validity[0] + 1.0, vote.validity[1]),
            ctx=replace(vote.ctx, message_seq=vote.ctx.message_seq + 2),
        ),
        0.14,
    )
    assert a.committed.plan_seq == 0
    # I3：收齐 ACK 才提交；提交后 t_start 前仍执行旧计划
    a.on_message(replace(vote, ctx=replace(vote.ctx, message_seq=vote.ctx.message_seq + 3)), 0.2)
    assert a.committed.plan_seq == 1
    assert a.executing(0.5).plan_seq == 0 and a.executing(1.0).plan_seq == 1
    # I1：同一 member_epoch 内新旧计划名义执行期不重叠
    assert a.cfg.member_epoch == net.nodes["B"].cfg.member_epoch
    assert a.previous.occupancy.execution_end() <= a.committed.t_start
    assert not any(
        v.invariant == "SINGLE_COMMITTED" for v in check_invariants(net.nodes, 1.0, checker=REF)
    )


def test_real_missing_ack_is_not_executable():
    """I3：三机真实占用下，缺一票 ACK 不提交，也不执行该计划。"""
    sc = scenario(
        {"A": A0, "B": FAR0, "C": (0.0, -10.0, 0.0)},
        {"A": [A1], "B": [FAR1], "C": [(0.0, -12.0, 0.0)]},
    )
    net = Net(sc)
    net.tick(0.0, who=["A"])
    votes = []
    while net.bag:
        dest, msg = net.bag.pop(0)
        votes += net.nodes[dest].on_message(msg, net.t)
    assert len(votes) == 2 and {v.decision for v in votes} == {Decision.ACK}
    net.nodes["A"].on_message(votes[0], net.t)
    assert net.nodes["A"].committed.plan_seq == 0
    assert net.nodes["A"].executing(10.0).plan_seq == 0
    net.nodes["A"].on_message(votes[1], net.t)
    committed = net.nodes["A"].committed
    assert committed.plan_seq == 1
    assert net.nodes["A"].executing(committed.t_start - 1e-6).plan_seq == 0
    assert net.nodes["A"].executing(committed.t_start).plan_seq == 1


def test_real_overlapping_previous_is_single_committed():
    """I1 检查器在真实占用上会抓住伸进新计划的旧名义执行期。"""
    sc = scenario({"A": A0, "B": C0}, {"A": [A1], "B": [C1]})
    net = Net(sc)
    net.tick(0.0)
    net.deliver()
    a = net.nodes["A"]
    assert a.previous is not None
    assert a.previous.occupancy.execution_end() <= a.committed.t_start
    assert not any(v.invariant == "SINGLE_COMMITTED" for v in net.violations)
    a.previous = replace(a.previous, occupancy=replace(a.previous.occupancy, until=math.inf))
    viol = check_invariants(net.nodes, a.committed.t_start, checker=REF)
    assert any(v.invariant == "SINGLE_COMMITTED" for v in viol)


@pytest.mark.parametrize("seed", [0, 1, 2])
@pytest.mark.parametrize("faults", ["drop", "duplicate", "reorder", "restart"], ids=lambda f: f)
def test_random_faults_real_occupancy_no_violation(seed, faults):
    from yopo_swarm_core.protocol.des import FAULT_CLASSES

    sc = scenario({"A": A0, "B": C0}, {"A": [A1, A0], "B": [C1, C0]}, max_failed_rounds=99)
    sc = replace(sc, cfg=replace(sc.cfg, resend_period=1.0))
    res = Simulator(
        sc.cfg, FAULT_CLASSES[faults](sc.cfg), seed=seed, node_factory=sc.factory(), checker=REF
    ).run(12.0)
    assert not res.violations, res.violations[:3]
    assert not any(v.invariant == "DOUBLE_CONFIRM" for _, v in res.violations)
    assert res.stats["collision"] == 0


class SameSpotNode(ProtocolNode):
    """变异体：两机在同一次 tick 里把已提交计划改成同一点悬停，绕过投票。"""

    def on_tick(self, now):
        if self.plan_seq == 0:
            occ = hover_occupancy(self.id, (0.0, 0.0, 0.0), PCFG)
            parent = self.committed.plan_hash
            self.plan_seq = 1
            h = plan_hash_of(self.id, 1, parent, 0.0, occ, self.cfg)
            self.committed = PlanRecord(self.id, 1, h, parent, 0.0, occ)
        return []


def test_i10_double_confirm_fires_in_simulator_and_exhaustive():
    """I10：故障注入运行和有界穷举都会输出 DOUBLE_CONFIRM；诚实协议在两种模式下都没有。"""
    sc = scenario({"A": A0, "B": C0}, {"A": [A1], "B": [C1]})
    from yopo_swarm_core.protocol.des import FAULT_CLASSES

    honest = Simulator(
        sc.cfg, FAULT_CLASSES["reorder"](sc.cfg), seed=0, node_factory=sc.factory(), checker=REF
    ).run(4.0)
    assert not any(v.invariant == "DOUBLE_CONFIRM" for _, v in honest.violations)

    def mutant(m, c):
        return SameSpotNode(m, c, initial=sc.initial)

    # 先 tick 的那一机提交后，邻机尚未提交，COVERAGE 更早；继续跑才会看到双方都确认。
    bad = Simulator(sc.cfg, seed=0, node_factory=mutant, checker=REF, stop_on_violation=False).run(
        0.1
    )
    assert any(v.invariant == "DOUBLE_CONFIRM" for _, v in bad.violations)
    found = explore(
        sc.cfg,
        max_steps=3,
        max_plan_seq=1,
        queue_depth=2,
        faults=("reorder",),
        node_factory=mutant,
        checker=REF,
    )
    assert found.violation is not None and found.violation["invariant"] == "DOUBLE_CONFIRM"
    assert found.counterexample == ["TICK"]


def test_exhaustive_two_uav_bounded_no_violation_and_scope_saved(tmp_path):
    sc = scenario({"A": A0, "B": C0}, {"A": [A1], "B": [C1]})
    s = explore(
        sc.cfg,
        max_steps=7,
        max_plan_seq=1,
        queue_depth=3,
        faults=("drop", "duplicate", "reorder"),
        node_factory=sc.factory(),
        checker=REF,
    )
    assert s.violation is None and s.complete, s.counterexample
    assert s.states > 100
    s.save(tmp_path / "g3.json")
    saved = json.loads((tmp_path / "g3.json").read_text())
    for k in (
        "members",
        "max_steps",
        "max_plan_seq",
        "queue_depth",
        "time_step",
        "faults",
        "states",
        "complete",
    ):
        assert k in saved


def test_saved_exhaustive_record_is_reproducible():
    """docs/ref/g3-explore-2uav.json 与当前代码重跑结果一致（覆盖范围与结果一起入库）。"""
    from yopo_swarm_core.protocol.plan_occupancy import reference_exploration

    path = Path(__file__).resolve().parents[2] / "docs/ref/g3-explore-2uav.json"
    saved = json.loads(path.read_text(encoding="utf-8"))
    fresh = reference_exploration()
    assert fresh == saved


def test_ack_binding_and_double_confirm_hold_in_abstract_model_too():
    """新增不变量对 #5 抽象模型同样成立（回归）。"""
    cfg = ProtocolConfig(members=("A", "B"))
    s = explore(cfg, max_steps=6, max_plan_seq=1, queue_depth=3)
    assert s.violation is None
