"""协议节点：ADR-0001 轮转提案窗口 + 预留 ACK / COMMIT / RELEASE（纯状态机，无 ROS、无 IO）。

票 13d 在 ACK/COMMIT 上加 RMADER 双轨（ADR 0012）：
PROPOSED → TENTATIVE（PROPOSE 已交出，即广播）→ DELAY_CHECK → COMMITTED。
Delay Check 没走完就提交不了。已提交的那一份一直留着，动力学在 t_start 之前仍飞它。
Delay Check 期间若收到与新轨冲突的邻机轨迹（含迟到才到的），作废新轨并允许本窗口再提一份。

安全规则（方案书 V0.3 B01/B02、V0.2 5.3）：
- 只有当前窗口拥有者能提出运动更新；每机最多一个在途提案。
- ACK = 对该提案的排他预留：绑定 reservation_id、plan_hash、有效期、依赖摘要；不会同时 ACK 两个不兼容提案。
- 预留只在收到 COMMIT（转为对端已提交计划）或 RELEASE（提案者明确放弃）时释放，
  绝不因本地等待超时释放；未收到 COMMIT 不等于对端没有执行。
- 收齐当前成员表内 N-1 个绑定一致的 ACK，并且 Delay Check 时间已到，才提交；提交的计划在 t_start 前不执行。
- 版本上下文不一致、seq 倒退、非有限数值的消息一律丢弃且不改状态；对端重启（新 boot_id）、
  本机 clock reset / task reset 直接结束本次任务（P0 不做重加入）。
"""

from __future__ import annotations

import json
import math
from collections.abc import Callable
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Any

from yopo_swarm_core.protocol.messages import (
    Commit,
    Decision,
    Heartbeat,
    Message,
    Proposal,
    Reason,
    Release,
    VersionContext,
    Vote,
    canonical,
    digest,
)
from yopo_swarm_core.protocol.occupancy import (
    AbstractConflictChecker,
    AbstractOccupancy,
    ConflictChecker,
    Span,
)


class NodeMode(Enum):
    ACTIVE = "ACTIVE"
    HALTED = "HALTED"  # 停止新的任务进度，只执行已批准计划（含终端保持）


class TrackPhase(Enum):
    """新轨在提交前的阶段。COMMITTED 只出现在日志里：提交后新轨变成 committed。"""

    PROPOSED = "PROPOSED"
    TENTATIVE = "TENTATIVE"  # PROPOSE 已交给调用方广播
    DELAY_CHECK = "DELAY_CHECK"
    COMMITTED = "COMMITTED"


@dataclass(frozen=True)
class ProtocolConfig:
    members: tuple[str, ...]
    window_len: float = 1.0
    member_epoch: int = 0
    task_hash: str = "task-1"
    max_failed_rounds: int = 3
    heartbeat_period: float | None = None  # None = 只在启动/重启后发一次
    resend_period: float | None = 1.0  # COMMIT/RELEASE 重发周期；None 关闭
    regions: tuple[str, ...] | None = None  # 抽象规划器可选区域
    max_proposals: int | None = None  # 有界穷举用：每个 boot 最多发起的提案数
    # RMADER δ_DC。0 表示收齐 ACK 当拍提交（原行为）。必须小于窗口长，否则本窗提不出提案。
    delay_check_s: float = 0.0

    def window_index(self, t: float) -> int:
        return math.floor(t / self.window_len)

    def owner(self, window_index: int) -> str:
        return self.members[window_index % len(self.members)]

    def region_set(self) -> tuple[str, ...]:
        return self.regions or tuple(f"R{i}" for i in range(len(self.members) + 1))


@dataclass(frozen=True)
class PlanRecord:
    owner: str
    plan_seq: int
    plan_hash: str
    parent_plan_hash: str
    t_start: float
    occupancy: Any

    def to_canonical(self) -> Any:
        return {
            "owner": self.owner,
            "plan_seq": self.plan_seq,
            "plan_hash": self.plan_hash,
            "t_start": canonical(self.t_start),
            "occupancy": self.occupancy.to_canonical(),
        }


def plan_hash_of(owner: str, plan_seq: int, parent: str, t_start: float, occ: Any, cfg: Any) -> str:
    return digest(
        {
            "owner": owner,
            "plan_seq": plan_seq,
            "parent": parent,
            "t_start": t_start,
            "occupancy": occ,
            "task_hash": cfg.task_hash,
            "member_epoch": cfg.member_epoch,
        }
    )


InitialOccupancy = Callable[[str, ProtocolConfig], Any]


def abstract_initial(owner: str, cfg: ProtocolConfig) -> AbstractOccupancy:
    return AbstractOccupancy.of(Span(0.0, math.inf, f"H{cfg.members.index(owner)}"))


def initial_plan(
    owner: str, cfg: ProtocolConfig, initial: InitialOccupancy = abstract_initial
) -> PlanRecord:
    occ = initial(owner, cfg)
    h = plan_hash_of(owner, 0, "", 0.0, occ, cfg)
    return PlanRecord(owner, 0, h, "", 0.0, occ)


@dataclass
class InFlight:
    proposal: Proposal
    record: PlanRecord
    acks: dict[str, Vote] = field(default_factory=dict)
    phase: TrackPhase = TrackPhase.PROPOSED
    broadcast_t: float = 0.0
    delay_until: float = 0.0


@dataclass
class PeerState:
    boot_id: int = 0
    last_seq: int = -1
    plans: list[PlanRecord] = field(default_factory=list)  # 已知已提交（旧的已截断）
    reservations: dict[str, PlanRecord] = field(default_factory=dict)  # 已 ACK 未结
    # 见过、尚未提交的邻机新轨。delay_check_s > 0 时规划会躲开它。
    heard: dict[str, PlanRecord] = field(default_factory=dict)


Planner = Callable[["ProtocolNode", float, float], Any]


def default_planner(node: ProtocolNode, now: float, t_start: float) -> Any | None:
    """抽象规划器：从 t_start 起换到一个与邻机可能占用都不冲突的区域（含终端保持）。"""
    regions = node.cfg.region_set()
    k = node.cfg.members.index(node.id) + node.plan_seq + 1
    current = {s.region for s in node.committed.occupancy.spans}
    for i in range(len(regions)):
        region = regions[(k + i) % len(regions)]
        if region in current:
            continue
        occ = AbstractOccupancy.of(Span(t_start, math.inf, region))
        if not node.conflicts_with_neighbors(occ):
            return occ
    return None


class ProtocolNode:
    def __init__(
        self,
        node_id: str,
        cfg: ProtocolConfig,
        checker: ConflictChecker | None = None,
        planner: Planner = default_planner,
        initial: InitialOccupancy = abstract_initial,
    ) -> None:
        if node_id not in cfg.members:
            raise ValueError(f"{node_id} not in members")
        self.id = node_id
        self.cfg = cfg
        self.checker = checker or AbstractConflictChecker()
        self.planner = planner
        self.initial = initial  # 各成员起始占用（全员一致的任务配置）
        self.boot_id = 0
        self.rejections: list[Reason] = []
        self.events: list[dict] = []
        self._fresh_state(task_hash=cfg.task_hash, clock_epoch=0)

    # --- 状态 -----------------------------------------------------------
    def _fresh_state(self, task_hash: str, clock_epoch: int) -> None:
        self.task_hash = task_hash
        self.clock_epoch = clock_epoch
        self.seq = 0
        self.mode = NodeMode.ACTIVE
        self.plan_seq = 0
        self.committed = initial_plan(self.id, self.cfg, self.initial)
        self.previous: PlanRecord | None = None
        self.in_flight: InFlight | None = None
        self.failed_rounds = 0
        self.last_window_proposed = -1
        self.hello_pending = False  # 只有重启后才需要宣告新 boot_id
        self.last_heartbeat = -math.inf
        self.last_resend = -math.inf
        self.resend: list[Message] = []  # 最近一次 COMMIT / RELEASE（重发用，构造时不带 ctx）
        self.peers = {
            m: PeerState(plans=[initial_plan(m, self.cfg, self.initial)])
            for m in self.cfg.members
            if m != self.id
        }

    def snapshot(self) -> str:
        """规范化状态（不含诊断记录）。用于确定性比对与穷举去重。"""
        state = {
            "id": self.id,
            "boot": self.boot_id,
            "task": self.task_hash,
            "clock_epoch": self.clock_epoch,
            "seq": self.seq,
            "mode": self.mode.value,
            "plan_seq": self.plan_seq,
            "committed": self.committed.to_canonical(),
            "previous": self.previous.to_canonical() if self.previous else None,
            "in_flight": None
            if self.in_flight is None
            else {
                "rid": self.in_flight.proposal.reservation_id,
                "acks": sorted(self.in_flight.acks),
                "phase": self.in_flight.phase.value,
                "delay_until": canonical(self.in_flight.delay_until),
            },
            "failed": self.failed_rounds,
            "last_window": self.last_window_proposed,
            "hello": self.hello_pending,
            "resend": [m.to_json()["reservation_id"] for m in self.resend],
            "peers": {
                m: {
                    "boot": p.boot_id,
                    "seq": p.last_seq,
                    "plans": [r.plan_hash for r in p.plans],
                    "plan_ends": [canonical(r.occupancy.execution_end()) for r in p.plans],
                    "res": sorted(p.reservations),
                    "heard": sorted(p.heard),
                }
                for m, p in sorted(self.peers.items())
            },
        }
        return json.dumps(state, sort_keys=True, separators=(",", ":"))

    @property
    def ctx(self) -> VersionContext:
        return VersionContext(
            self.cfg.member_epoch, self.task_hash, self.clock_epoch, self.boot_id, self.id, self.seq
        )

    def next_ctx(self) -> VersionContext:
        self.seq += 1
        return self.ctx

    def _log(self, event: str, **kw: Any) -> None:
        self.events.append({"node": self.id, "event": event, **canonical(kw)})

    def _reject(self, reason: Reason, msg: Message | None = None) -> None:
        self.rejections.append(reason)
        self._log("reject", reason=reason, msg_kind=msg.kind if msg else None)

    def _halt(self, reason: Reason) -> None:
        if self.mode is not NodeMode.HALTED:
            self.mode = NodeMode.HALTED
            self._log("halt", reason=reason)

    # --- 占用视图 -------------------------------------------------------
    def own_possible(self) -> list[Any]:
        occ = [self.committed.occupancy]
        if self.previous is not None:
            occ.append(self.previous.occupancy)
        if self.in_flight is not None:
            occ.append(self.in_flight.record.occupancy)
        return occ

    def neighbor_possible(self, exclude: str | None = None) -> list[tuple[str, PlanRecord]]:
        out = []
        for m, p in sorted(self.peers.items()):
            if m == exclude:
                continue
            out += [(m, r) for r in p.plans]
            out += [(m, r) for _, r in sorted(p.reservations.items())]
            if self.cfg.delay_check_s > 0:
                out += [(m, r) for _, r in sorted(p.heard.items())]
        return out

    def conflicts_with_neighbors(self, occ: Any) -> bool:
        return any(self.checker.conflicts(occ, r.occupancy) for _, r in self.neighbor_possible())

    def possible_plan_hashes(self, peer: str, t: float) -> set[str]:
        p = self.peers[peer]
        live = {r.plan_hash for r in p.plans if r.occupancy.end_time() > t}
        return live | {r.plan_hash for r in p.reservations.values()}

    def executing(self, t: float) -> PlanRecord:
        if t >= self.committed.t_start or self.previous is None:
            return self.committed
        return self.previous

    def dependency_digest(self) -> str:
        return digest({m: [r.plan_hash for r in p.plans] for m, p in sorted(self.peers.items())})

    # --- 故障注入入口（由测试器调用） ------------------------------------
    def restart(self, now: float) -> None:
        self.boot_id += 1
        self._fresh_state(self.task_hash, self.clock_epoch)
        self.hello_pending = True
        self._halt(Reason.SELF_RESET)
        self._log("restart", boot=self.boot_id, t=now)

    def clock_reset(self) -> None:
        self.clock_epoch += 1
        self.in_flight = None
        self._halt(Reason.SELF_RESET)
        self._log("clock_reset", clock_epoch=self.clock_epoch)

    def task_reset(self, task_hash: str) -> None:
        self.task_hash = task_hash
        self.in_flight = None
        self._halt(Reason.SELF_RESET)
        self._log("task_reset", task_hash=task_hash)

    # --- 发送 -----------------------------------------------------------
    def make_proposal(self, now: float, occupancy: Any) -> Proposal:
        w = self.cfg.window_index(now)
        t_start = (w + 1) * self.cfg.window_len
        self.plan_seq += 1
        parent = self.committed.plan_hash
        h = plan_hash_of(self.id, self.plan_seq, parent, t_start, occupancy, self.cfg)
        rid = f"{self.id}/{self.boot_id}/{self.plan_seq}"
        prop = Proposal(
            ctx=self.next_ctx(),
            reservation_id=rid,
            proposer_id=self.id,
            plan_seq=self.plan_seq,
            plan_hash=h,
            parent_plan_hash=parent,
            window_index=w,
            t_start=t_start,
            validity=(t_start, occupancy.end_time()),
            occupancy=occupancy,
            dependency_digest=self.dependency_digest(),
        )
        record = PlanRecord(self.id, self.plan_seq, h, parent, t_start, occupancy)
        self.in_flight = InFlight(prop, record, phase=TrackPhase.PROPOSED, broadcast_t=now)
        self.last_window_proposed = w
        self._log("phase", phase=TrackPhase.PROPOSED, rid=rid)
        # 返回值就是要广播的 PROPOSE。TENTATIVE = 已交给调用方。
        self.in_flight.phase = TrackPhase.TENTATIVE
        self.in_flight.broadcast_t = now
        self._log("phase", phase=TrackPhase.TENTATIVE, rid=rid)
        self.in_flight.phase = TrackPhase.DELAY_CHECK
        self.in_flight.delay_until = now + float(self.cfg.delay_check_s)
        self._log("phase", phase=TrackPhase.DELAY_CHECK, rid=rid, until=self.in_flight.delay_until)
        self._log("propose", rid=rid, plan_hash=h, t_start=t_start, window=w)
        return prop

    def _quorum(self, flight: InFlight) -> bool:
        return set(flight.acks) == set(self.peers)

    def _delay_check_open(self) -> bool:
        flight = self.in_flight
        return (
            flight is not None
            and flight.phase is TrackPhase.DELAY_CHECK
            and self.cfg.delay_check_s > 0
        )

    def _abort(self, reason: Reason, *, repropose: bool = False) -> list[Message]:
        assert self.in_flight is not None
        window = self.in_flight.proposal.window_index
        rid, h = self.in_flight.proposal.reservation_id, self.in_flight.record.plan_hash
        self.in_flight = None
        self.failed_rounds += 1
        if repropose:
            # 同一窗口还能再提一份新轨。06c 重发的是同一 reservation，这里是新的 plan_seq。
            self.last_window_proposed = window - 1
        self._log("abort", rid=rid, reason=reason, repropose=repropose)
        rel = Release(ctx=self.next_ctx(), reservation_id=rid, plan_hash=h)
        self.resend = [rel]
        if self.failed_rounds >= self.cfg.max_failed_rounds:
            self._halt(Reason.TOO_MANY_FAILURES)
        return [rel]

    def _commit(self) -> list[Message]:
        assert self.in_flight is not None
        rec = self.in_flight.record
        self.previous = replace(
            self.committed, occupancy=self.committed.occupancy.truncated(rec.t_start)
        )
        self.committed = rec
        self.in_flight = None
        self.failed_rounds = 0
        rid = f"{self.id}/{self.boot_id}/{rec.plan_seq}"
        self._log("phase", phase=TrackPhase.COMMITTED, rid=rid)
        self._log("commit", rid=rid, plan_hash=rec.plan_hash)
        c = Commit(
            ctx=self.next_ctx(),
            reservation_id=f"{self.id}/{self.boot_id}/{rec.plan_seq}",
            plan_hash=rec.plan_hash,
        )
        self.resend = [c]
        return [c]

    def finalize_delay_check(self, now: float) -> list[Message]:
        """本拍消息已经投递完之后调用。收齐 ACK 且 δ_DC 已到，才把新轨提交。

        同一拍上 on_tick 不提交（`now > delay_until` 才在下一拍兜底），这样
        deliver_at == now 的迟到邻机轨迹能先被看见。
        """
        flight = self.in_flight
        if flight is None or not self._quorum(flight):
            return []
        if now + 1e-12 < flight.delay_until:
            return []
        if now >= flight.record.t_start:
            return self._abort(Reason.DEADLINE)
        return self._commit()

    def on_tick(self, now: float) -> list[Message]:
        out: list[Message] = []
        if self.in_flight is not None and now >= self.in_flight.record.t_start:
            out += self._abort(Reason.DEADLINE)
        elif (
            self.in_flight is not None
            and self._quorum(self.in_flight)
            and now > self.in_flight.delay_until + 1e-12
            and now < self.in_flight.record.t_start
        ):
            out += self._commit()
        hb_due = self.hello_pending or (
            self.cfg.heartbeat_period is not None
            and now - self.last_heartbeat >= self.cfg.heartbeat_period
        )
        if hb_due:
            self.hello_pending = False
            self.last_heartbeat = now
            out.append(Heartbeat(ctx=self.next_ctx()))
        if (
            self.resend
            and self.cfg.resend_period is not None
            and now - self.last_resend >= self.cfg.resend_period
            and not out
        ):
            self.last_resend = now
            out += [replace(m, ctx=self.next_ctx()) for m in self.resend]
        w = self.cfg.window_index(now)
        if (
            self.mode is NodeMode.ACTIVE
            and self.in_flight is None
            and self.cfg.owner(w) == self.id
            and self.last_window_proposed < w
            and (self.cfg.max_proposals is None or self.plan_seq < self.cfg.max_proposals)
        ):
            t_start = (w + 1) * self.cfg.window_len
            if now + self.cfg.delay_check_s >= t_start - 1e-12:
                self.last_window_proposed = w
                self._log("skip", window=w, reason=Reason.DEADLINE)
                return out
            occ = self.planner(self, now, t_start)
            if occ is None:
                self.last_window_proposed = w
                self._log("skip", window=w, reason=Reason.NO_CANDIDATE)
            else:
                out.append(self.make_proposal(now, occ))
        return out

    # --- 接收 -----------------------------------------------------------
    def _validate(self, msg: Message) -> Reason | None:
        c = msg.ctx
        if c.sender_id not in self.peers:
            return Reason.UNKNOWN_SENDER
        if c.member_epoch != self.cfg.member_epoch:
            return Reason.MEMBER_EPOCH
        if c.task_hash != self.task_hash:
            return Reason.TASK_HASH
        if c.clock_epoch != self.clock_epoch:
            return Reason.CLOCK_EPOCH
        peer = self.peers[c.sender_id]
        if c.sender_boot_id < peer.boot_id:
            return Reason.STALE_BOOT
        if c.sender_boot_id > peer.boot_id:
            return Reason.PEER_RESTARTED
        if c.message_seq <= peer.last_seq:
            return Reason.SEQ_REGRESSION
        if not all(math.isfinite(x) for x in msg.numbers()):
            return Reason.MALFORMED
        if isinstance(msg, Proposal) and not msg.occupancy.is_finite():
            return Reason.MALFORMED
        return None

    def on_message(self, msg: Message, now: float) -> list[Message]:
        dest = getattr(msg, "dest", None)
        if dest is not None and dest != self.id:
            return []
        reason = self._validate(msg)
        if reason is Reason.PEER_RESTARTED:
            self.peers[msg.ctx.sender_id].boot_id = msg.ctx.sender_boot_id
            self.peers[msg.ctx.sender_id].last_seq = msg.ctx.message_seq
            self._reject(reason, msg)
            self._halt(reason)  # P0 不做重加入：结束本次任务
            return []
        if reason is not None:
            self._reject(reason, msg)
            return []
        self.peers[msg.ctx.sender_id].last_seq = msg.ctx.message_seq
        if isinstance(msg, Proposal):
            return self._on_proposal(msg, now)
        if isinstance(msg, Vote):
            return self._on_vote(msg)
        if isinstance(msg, Commit):
            return self._on_commit(msg, now)
        if isinstance(msg, Release):
            return self._on_release(msg)
        return []

    def _vote(self, p: Proposal, decision: Decision, reason: Reason) -> list[Message]:
        self._log("vote", rid=p.reservation_id, decision=decision, reason=reason)
        return [
            Vote(
                ctx=self.next_ctx(),
                reservation_id=p.reservation_id,
                proposer_id=p.proposer_id,
                plan_hash=p.plan_hash,
                validity=p.validity,
                voter_occupancy_version=self.committed.plan_hash,
                dependency_digest=p.dependency_digest,
                decision=decision,
                reason=reason,
                dest=p.proposer_id,
            )
        ]

    def _conflicts_with_new_track(self, occ: Any) -> bool:
        if not self._delay_check_open():
            return False
        if occ is None or not occ.is_finite():
            return False
        flight = self.in_flight
        assert flight is not None
        return self.checker.conflicts(flight.record.occupancy, occ)

    def _remember(self, p: Proposal) -> None:
        """delay_check_s > 0 时记下邻机新轨，重提时躲开。δ = 0 不记，避免改原来的规划视图。"""
        if self.cfg.delay_check_s <= 0 or not p.occupancy.is_finite():
            return
        self.peers[p.ctx.sender_id].heard[p.reservation_id] = PlanRecord(
            p.proposer_id, p.plan_seq, p.plan_hash, p.parent_plan_hash, p.t_start, p.occupancy
        )

    def _yield_new_track(self, p: Proposal, reason: Reason) -> list[Message]:
        if not self._conflicts_with_new_track(p.occupancy):
            return []
        self._remember(p)
        return self._abort(reason, repropose=True)

    def _on_proposal(self, p: Proposal, now: float) -> list[Message]:
        if self.mode is NodeMode.HALTED:
            return self._vote(p, Decision.NACK_STALE, Reason.HALTED)
        w = self.cfg.window_index(now)
        if (
            p.window_index != w
            or self.cfg.owner(w) != p.proposer_id
            or p.proposer_id != p.ctx.sender_id
        ):
            return self._yield_new_track(p, Reason.LATE) + self._vote(
                p, Decision.NACK_STALE, Reason.WINDOW
            )
        if p.t_start <= now:
            return self._yield_new_track(p, Reason.LATE) + self._vote(
                p, Decision.NACK_STALE, Reason.LATE
            )
        expect = plan_hash_of(
            p.proposer_id, p.plan_seq, p.parent_plan_hash, p.t_start, p.occupancy, self.cfg
        )
        if expect != p.plan_hash or p.validity != (p.t_start, p.occupancy.end_time()):
            self._reject(Reason.MALFORMED, p)
            return self._vote(p, Decision.NACK_STALE, Reason.MALFORMED)
        self._remember(p)
        flight = self.in_flight
        if flight is not None and self._conflicts_with_new_track(p.occupancy):
            reason = Reason.LATE if p.t_start <= flight.broadcast_t else Reason.CONFLICT
            return self._abort(reason, repropose=True) + self._decide_proposal(p)
        return self._decide_proposal(p)

    def _decide_proposal(self, p: Proposal) -> list[Message]:
        mine = self.own_possible()
        # 已为第三方（或该提案者的其他提案）ACK 的预留都要排他：不同时 ACK 两个不兼容提案
        reserved = [
            r.occupancy
            for m, peer in self.peers.items()
            for rid, r in peer.reservations.items()
            if not (m == p.proposer_id and rid == p.reservation_id)
        ]
        if any(self.checker.conflicts(p.occupancy, o) for o in mine + reserved):
            return self._vote(p, Decision.NACK_CONFLICT, Reason.CONFLICT)
        rec = PlanRecord(
            p.proposer_id, p.plan_seq, p.plan_hash, p.parent_plan_hash, p.t_start, p.occupancy
        )
        self.peers[p.proposer_id].reservations[p.reservation_id] = rec
        return self._vote(p, Decision.ACK, Reason.OK)

    def _on_vote(self, v: Vote) -> list[Message]:
        f = self.in_flight
        if f is None:
            return []
        p = f.proposal
        bound = (
            v.reservation_id == p.reservation_id
            and v.proposer_id == self.id
            and v.plan_hash == p.plan_hash
            and v.validity == p.validity
            and v.dependency_digest == p.dependency_digest
        )
        if not bound:
            self._reject(Reason.VOTE_MISMATCH, v)
            return []
        if v.decision is not Decision.ACK:
            return self._abort(Reason.NACKED)
        f.acks[v.ctx.sender_id] = v
        # δ_DC > 0 时不在这张 ACK 上提交。同一拍里还可能有迟到邻机轨迹，
        # 由 finalize_delay_check 在投递完之后提交。δ = 0 保持收齐即提交。
        if self._quorum(f) and self.cfg.delay_check_s <= 0:
            return self._commit()
        return []

    def _on_commit(self, c: Commit, _now: float) -> list[Message]:
        peer = self.peers[c.ctx.sender_id]
        rec = peer.reservations.get(c.reservation_id)
        if rec is None or rec.plan_hash != c.plan_hash:
            if not any(r.plan_hash == c.plan_hash for r in peer.plans):
                self._reject(Reason.UNKNOWN_RESERVATION, c)
            return []
        del peer.reservations[c.reservation_id]
        peer.heard.pop(c.reservation_id, None)
        peer.plans = [replace(r, occupancy=r.occupancy.truncated(rec.t_start)) for r in peer.plans]
        peer.plans = [r for r in peer.plans if not r.occupancy.is_empty()] + [rec]
        self._log("learn_commit", peer=c.ctx.sender_id, plan_hash=rec.plan_hash)
        if self._conflicts_with_new_track(rec.occupancy):
            flight = self.in_flight
            assert flight is not None
            reason = Reason.LATE if rec.t_start <= flight.broadcast_t else Reason.CONFLICT
            return self._abort(reason, repropose=True)
        return []

    def _on_release(self, r: Release) -> list[Message]:
        peer = self.peers[r.ctx.sender_id]
        peer.heard.pop(r.reservation_id, None)
        rec = peer.reservations.get(r.reservation_id)
        if rec is not None and rec.plan_hash == r.plan_hash:
            del peer.reservations[r.reservation_id]
            self._log("release", peer=r.ctx.sender_id, rid=r.reservation_id)
        return []
