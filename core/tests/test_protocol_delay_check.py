"""票 13d：PROPOSED → TENTATIVE → DELAY_CHECK → COMMITTED。

δ_DC = 0 时收齐 ACK 仍在同一拍提交。δ_DC > 0 时新轨先广播，Delay Check
没走完不替换已提交轨；期间收到冲突或迟到的邻机轨迹就作废新轨，本窗口可以再提。
"""

import dataclasses

from yopo_swarm_core.protocol import (
    AbstractOccupancy,
    Decision,
    ProtocolConfig,
    ProtocolNode,
    Reason,
    Span,
    TrackPhase,
)
from yopo_swarm_core.protocol.messages import Commit


def cfg(**kw):
    kw.setdefault("delay_check_s", 0.2)
    return ProtocolConfig(members=("u0", "u1"), **kw)


def pair(**kw):
    c = cfg(**kw)
    return c, {m: ProtocolNode(m, c) for m in c.members}


def deliver(msgs, ns, now, only=None):
    out = []
    for m in msgs:
        for name, node in ns.items():
            if name == m.ctx.sender_id or (only is not None and name not in only):
                continue
            out += node.on_message(m, now)
    return out


def phases(node):
    return [e["phase"] for e in node.events if e["event"] == "phase"]


def test_zero_delay_still_commits_on_the_ack_that_completes_quorum():
    c = ProtocolConfig(members=("u0", "u1"), delay_check_s=0.0)
    ns = {m: ProtocolNode(m, c) for m in c.members}
    t = 0.5
    prop = ns["u0"].on_tick(t)[0]
    assert phases(ns["u0"]) == ["PROPOSED", "TENTATIVE", "DELAY_CHECK"]
    votes = deliver([prop], ns, t)
    assert {v.decision for v in votes} == {Decision.ACK}
    out = deliver(votes, ns, t, only={"u0"})
    assert [m.kind for m in out] == ["COMMIT"]
    assert phases(ns["u0"])[-1] == "COMMITTED"
    assert ns["u0"].committed.plan_hash == prop.plan_hash
    assert ns["u0"].in_flight is None


def test_delay_check_keeps_the_committed_track_until_it_elapses():
    c, ns = pair()
    t = 0.5  # u0 的窗口，t_start = 1
    safe = ns["u0"].committed.plan_hash
    prop = ns["u0"].on_tick(t)[0]
    assert ns["u0"].in_flight.phase is TrackPhase.DELAY_CHECK
    assert ns["u0"].in_flight.delay_until == t + c.delay_check_s
    votes = deliver([prop], ns, t)
    assert deliver(votes, ns, t, only={"u0"}) == []
    assert ns["u0"].committed.plan_hash == safe
    assert ns["u0"].in_flight is not None
    assert ns["u0"].executing(t).plan_hash == safe
    assert ns["u0"].finalize_delay_check(t + 0.1) == []
    commits = ns["u0"].finalize_delay_check(t + c.delay_check_s)
    assert [m.kind for m in commits] == ["COMMIT"]
    assert ns["u0"].committed.plan_hash == prop.plan_hash
    assert ns["u0"].previous is not None and ns["u0"].previous.plan_hash == safe
    assert ns["u0"].executing(prop.t_start - 1e-6).plan_hash == safe
    assert ns["u0"].executing(prop.t_start).plan_hash == prop.plan_hash
    assert phases(ns["u0"]) == ["PROPOSED", "TENTATIVE", "DELAY_CHECK", "COMMITTED"]


def test_conflicting_proposal_during_delay_check_drops_the_new_track():
    c, ns = pair()
    t = 0.5
    prop = ns["u0"].on_tick(t)[0]
    safe = ns["u0"].committed.plan_hash
    region = prop.occupancy.spans[0].region
    # u1 不是本窗主人。这份轨迹迟到才到，但和 u0 的新轨占同一区域。
    late = dataclasses.replace(
        prop,
        ctx=ns["u1"].next_ctx(),
        reservation_id="u1/0/9",
        proposer_id="u1",
        plan_seq=9,
        parent_plan_hash=ns["u1"].committed.plan_hash,
        plan_hash="late-track",
        occupancy=AbstractOccupancy.of(Span(0.0, 10.0, region)),
    )
    # 哈希对不上会被当成 MALFORMED，先按节点规则重算。
    from yopo_swarm_core.protocol.node import plan_hash_of

    late = dataclasses.replace(
        late,
        plan_hash=plan_hash_of(
            "u1", 9, ns["u1"].committed.plan_hash, late.t_start, late.occupancy, c
        ),
    )
    out = ns["u0"].on_message(late, t)
    assert [m.kind for m in out] == ["RELEASE", "VOTE"]
    assert out[0].plan_hash == prop.plan_hash
    assert out[1].decision is Decision.NACK_STALE
    assert out[1].reason is Reason.WINDOW
    assert ns["u0"].committed.plan_hash == safe
    assert ns["u0"].in_flight is None
    assert ns["u0"].rejections == [] or ns["u0"].events[-2]["reason"] == "LATE"
    abort = [e for e in ns["u0"].events if e["event"] == "abort"][-1]
    assert abort["reason"] == "LATE" and abort["repropose"] is True
    # 同一窗口再提一份，躲开刚听到的区域。
    again = [m for m in ns["u0"].on_tick(t + 0.01) if m.kind == "PROPOSE"]
    assert len(again) == 1
    assert again[0].plan_seq == 2
    assert again[0].occupancy.spans[0].region != region
    assert ns["u0"].committed.plan_hash == safe


def test_late_proposal_past_t_start_also_drops_a_conflicting_new_track():
    """t_start 已经不晚于现在的邻机轨迹，只要和新轨冲突，Delay Check 就作废新轨。"""
    _, ns = pair(delay_check_s=0.4)
    occ = AbstractOccupancy.of(Span(0.0, 20.0, "X"))
    ns["u0"].make_proposal(0.9, occ)
    assert ns["u0"].in_flight is not None
    now = 1.05  # 已经过了 u0 那份的 t_start=1，但还没走到 on_tick 的截止
    late = dataclasses.replace(ns["u1"].make_proposal(now, occ), t_start=now)
    out = ns["u0"].on_message(late, now)
    assert [m.kind for m in out] == ["RELEASE", "VOTE"]
    assert out[1].decision is Decision.NACK_STALE
    assert out[1].reason is Reason.LATE
    assert ns["u0"].committed.plan_seq == 0
    assert ns["u0"].in_flight is None
    assert [e for e in ns["u0"].events if e["event"] == "abort"][-1]["reason"] == "LATE"


def test_commit_of_a_conflicting_neighbor_track_drops_the_new_one():
    c, ns = pair()
    # u1 的窗口里先占住区域 X，u0 记下预留。
    t1 = 1.5
    occ = AbstractOccupancy.of(Span(0.0, 50.0, "X"))
    u1_prop = ns["u1"].make_proposal(t1, occ)
    ack = ns["u0"].on_message(u1_prop, t1)
    assert ack[0].decision is Decision.ACK
    # u0 自己的窗口里强行提出一份撞上 X 的新轨（规划器不会这么做，这里只测作废）。
    t0 = 2.5
    ns["u0"].make_proposal(t0, occ)
    safe = ns["u0"].committed.plan_hash
    commit = Commit(
        ctx=ns["u1"].next_ctx(),
        reservation_id=u1_prop.reservation_id,
        plan_hash=u1_prop.plan_hash,
    )
    out = ns["u0"].on_message(commit, t0)
    assert [m.kind for m in out] == ["RELEASE"]
    assert ns["u0"].committed.plan_hash == safe
    assert ns["u0"].in_flight is None
    assert u1_prop.plan_hash in {r.plan_hash for r in ns["u0"].peers["u1"].plans}
