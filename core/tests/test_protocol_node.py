"""#5 协议节点单元测试：纯状态机，直接喂消息。"""

import dataclasses
import math

import pytest

from yopo_swarm_core.protocol import (
    AbstractOccupancy,
    Commit,
    Decision,
    NodeMode,
    ProtocolConfig,
    ProtocolNode,
    Reason,
    Span,
    Vote,
)
from yopo_swarm_core.protocol.des import check_invariants


def cfg(n=3, **kw):
    return ProtocolConfig(members=tuple(f"u{i}" for i in range(n)), **kw)


def nodes(n=3, **kw):
    c = cfg(n, **kw)
    return c, {m: ProtocolNode(m, c) for m in c.members}


def owner_time(c, member, k=0):
    """member 拥有窗口的某个时刻（窗口中段）。"""
    idx = c.members.index(member) + k * len(c.members)
    return (idx + 0.5) * c.window_len


def deliver(msgs, ns, now, only=None):
    out = []
    for m in msgs:
        for name, node in ns.items():
            if name == m.ctx.sender_id or (only is not None and name not in only):
                continue
            out += node.on_message(m, now)
    return out


def test_only_window_owner_proposes():
    c, ns = nodes()
    t = owner_time(c, "u1")
    assert ns["u0"].on_tick(t) == []
    props = ns["u1"].on_tick(t)
    assert len(props) == 1 and props[0].kind == "PROPOSE"


def test_proposal_outside_window_rejected_as_stale():
    c, ns = nodes()
    prop = ns["u1"].on_tick(owner_time(c, "u1"))[0]
    later = owner_time(c, "u2")  # 已到 u2 的窗口
    votes = ns["u0"].on_message(prop, later)
    assert votes[0].decision is Decision.NACK_STALE
    assert votes[0].reason is Reason.WINDOW


@pytest.mark.parametrize(
    "field,value,reason",
    [
        ("clock_epoch", 1, Reason.CLOCK_EPOCH),
        ("member_epoch", 9, Reason.MEMBER_EPOCH),
        ("task_hash", "other", Reason.TASK_HASH),
    ],
)
def test_context_mismatch_rejected_and_state_unchanged(field, value, reason):
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    bad = dataclasses.replace(prop, ctx=dataclasses.replace(prop.ctx, **{field: value}))
    before = ns["u0"].snapshot()
    out = ns["u0"].on_message(bad, t)
    assert ns["u0"].snapshot() == before
    assert ns["u0"].rejections[-1] == reason
    assert all(v.decision is not Decision.ACK for v in out)


def test_old_sequence_does_not_update_state():
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    ns["u0"].on_message(prop, t)
    before = ns["u0"].snapshot()
    replay = dataclasses.replace(
        prop, ctx=dataclasses.replace(prop.ctx, message_seq=prop.ctx.message_seq - 1)
    )
    ns["u0"].on_message(replay, t)
    assert ns["u0"].snapshot() == before
    assert ns["u0"].rejections[-1] is Reason.SEQ_REGRESSION


def test_non_finite_fields_rejected():
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    bad = dataclasses.replace(prop, t_start=math.nan)
    before = ns["u0"].snapshot()
    ns["u0"].on_message(bad, t)
    assert ns["u0"].snapshot() == before
    assert ns["u0"].rejections[-1] is Reason.MALFORMED


def test_full_round_commits_and_executes_only_after_t_start():
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    votes = deliver([prop], ns, t)
    assert {v.decision for v in votes} == {Decision.ACK}
    out = deliver(votes, ns, t, only={"u1"})
    commit = [m for m in out if m.kind == "COMMIT"]
    assert len(commit) == 1
    assert ns["u1"].committed.plan_hash == prop.plan_hash
    assert ns["u1"].executing(prop.t_start - 1e-6).plan_hash != prop.plan_hash  # I3
    assert ns["u1"].executing(prop.t_start).plan_hash == prop.plan_hash


def test_missing_one_ack_never_commits():
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    votes = deliver([prop], ns, t, only={"u0"})  # u2 的票丢失
    out = deliver(votes, ns, t, only={"u1"})
    assert not [m for m in out if m.kind == "COMMIT"]
    # 超过截止仍未收齐：放弃并发 RELEASE，不执行
    out = ns["u1"].on_tick(prop.t_start + 1e-3)
    assert [m.kind for m in out].count("RELEASE") == 1
    assert ns["u1"].executing(prop.t_start + 1.0).plan_hash != prop.plan_hash


def test_vote_bound_to_plan_hash_and_window_is_not_reusable():
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    votes = deliver([prop], ns, t)
    # 终端时刻是 +inf，inf+1 仍是 inf，必须改有限的起点才真的换了有效期。
    # 这一票是凑齐全员 ACK 的那一票：有效期检查被绕过的话这里就会提交。
    shifted = dataclasses.replace(
        votes[1], validity=(votes[1].validity[0] + 1.0, votes[1].validity[1])
    )
    out = deliver([votes[0], shifted], ns, t, only={"u1"})
    assert not [m for m in out if m.kind == "COMMIT"]
    assert ns["u1"].rejections[-1] is Reason.VOTE_MISMATCH
    rehashed = dataclasses.replace(
        votes[1],
        plan_hash="deadbeef",
        ctx=dataclasses.replace(votes[1].ctx, message_seq=votes[1].ctx.message_seq + 1),
    )
    out = deliver([rehashed], ns, t, only={"u1"})
    assert not [m for m in out if m.kind == "COMMIT"]
    assert ns["u1"].committed.plan_seq == 0


def test_vote_bound_to_dependency_digest_is_not_reusable():
    """I6：ACK 还绑定提案所参考的邻机版本，换一份摘要就不能用来提交。"""
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    votes = deliver([prop], ns, t)
    forged = dataclasses.replace(votes[0], dependency_digest="0" * 64)
    out = deliver([forged, votes[1]], ns, t, only={"u1"})
    assert not [m for m in out if m.kind == "COMMIT"]
    assert ns["u1"].committed.plan_seq == 0
    assert ns["u1"].rejections[-1] is Reason.VOTE_MISMATCH


def test_same_member_epoch_committed_versions_do_not_overlap():
    """I1：同一 member_epoch 内提交后，旧计划名义执行期不超过新 t_start。"""
    c, ns = nodes()
    assert len({n.cfg.member_epoch for n in ns.values()}) == 1
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    deliver(deliver([prop], ns, t), ns, t, only={"u1"})
    node = ns["u1"]
    assert node.committed.plan_seq == 1 and node.previous is not None
    assert node.previous.occupancy.execution_end() <= node.committed.t_start
    assert not any(v.invariant == "SINGLE_COMMITTED" for v in check_invariants(ns, prop.t_start))


def test_overlapping_committed_versions_reported_as_single_committed():
    """I1 检查器不是空转：旧计划名义期伸进新计划就会报 SINGLE_COMMITTED。"""
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    deliver(deliver([prop], ns, t), ns, t, only={"u1"})
    node = ns["u1"]
    stale = node.previous.occupancy.spans[0]
    node.previous = dataclasses.replace(
        node.previous,
        occupancy=AbstractOccupancy.of(Span(stale.t0, math.inf, stale.region)),
    )
    viol = check_invariants(ns, prop.t_start)
    assert any(v.invariant == "SINGLE_COMMITTED" and v.detail.startswith("u1") for v in viol)


def test_voter_never_acks_two_incompatible_proposals():
    c, ns = nodes()
    a = ns["u0"]
    # 构造两个占用相同区域的提案
    p1 = ns["u1"].make_proposal(owner_time(c, "u1"), AbstractOccupancy.of(Span(0, 100, "X")))
    v1 = a.on_message(p1, owner_time(c, "u1"))
    assert v1[0].decision is Decision.ACK
    p2 = ns["u2"].make_proposal(owner_time(c, "u2"), AbstractOccupancy.of(Span(0, 100, "X")))
    v2 = a.on_message(p2, owner_time(c, "u2"))
    assert v2[0].decision is Decision.NACK_CONFLICT


def test_at_most_one_in_flight_proposal():
    c, ns = nodes()
    t = owner_time(c, "u1")
    assert len(ns["u1"].on_tick(t)) == 1
    assert [m for m in ns["u1"].on_tick(t + 1e-3) if m.kind == "PROPOSE"] == []


def test_restart_new_boot_id_old_acks_not_reused():
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    votes = deliver([prop], ns, t)
    ns["u1"].restart(t)
    assert ns["u1"].ctx.sender_boot_id != prop.ctx.sender_boot_id
    out = deliver(votes, ns, t, only={"u1"})
    assert not [m for m in out if m.kind == "COMMIT"]
    assert ns["u1"].in_flight is None


def test_peer_restart_ends_task_for_listeners():
    _, ns = nodes()
    ns["u2"].restart(0.0)
    hb = ns["u2"].on_tick(0.01)  # 心跳会携带新 boot_id
    deliver(hb, ns, 0.01)
    assert ns["u0"].mode is NodeMode.HALTED
    assert ns["u0"].rejections[-1] is Reason.PEER_RESTARTED


def test_clock_reset_old_acks_not_reused():
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    votes = deliver([prop], ns, t)
    ns["u1"].clock_reset()
    out = deliver(votes, ns, t, only={"u1"})
    assert not [m for m in out if m.kind == "COMMIT"]


def test_task_reset_old_acks_not_reused():
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    votes = deliver([prop], ns, t)
    ns["u1"].task_reset("task-2")
    out = deliver(votes, ns, t, only={"u1"})
    assert not [m for m in out if m.kind == "COMMIT"]


def test_voter_keeps_reservation_without_commit_until_release():
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    votes = deliver([prop], ns, t)
    deliver(votes, ns, t, only={"u1"})  # u1 已提交，但 COMMIT 未送达 u0
    far = prop.t_start + 50 * c.window_len
    ns["u0"].on_tick(far)
    # 本地等待超时不释放：u0 仍把 u1 的新计划算作可能占用（I13）
    assert prop.plan_hash in ns["u0"].possible_plan_hashes("u1", far)


def test_commit_with_stale_reservation_ignored():
    c, ns = nodes()
    t = owner_time(c, "u1")
    prop = ns["u1"].on_tick(t)[0]
    bogus = Commit(ctx=ns["u1"].next_ctx(), reservation_id="nope", plan_hash=prop.plan_hash)
    ns["u0"].on_message(bogus, t)
    assert prop.plan_hash not in ns["u0"].possible_plan_hashes("u1", t)


def test_vote_type_is_constructible_for_injection():
    # Vote 字段齐全：reservation_id/plan_hash/validity/voter_occupancy_version/dependency_digest
    names = {f.name for f in dataclasses.fields(Vote)}
    assert {
        "reservation_id",
        "proposer_id",
        "plan_hash",
        "validity",
        "voter_occupancy_version",
        "dependency_digest",
        "decision",
        "reason",
    } <= names
