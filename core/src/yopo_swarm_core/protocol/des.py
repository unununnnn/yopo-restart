"""无 ROS 离散事件测试器：确定性调度协议节点 + 可注入故障的网络模型 + 每步不变量检查。

两种模式：
- Simulator：给定种子的随机故障运行（丢失、重复、乱序、分区、重启、时钟暂停/回拨、任务重置），
  同一种子逐字节可重放。
- explore：有界穷举（BFS），在给定机数、计划版本数、队列深度、步数上界内枚举
  投递/丢失/重复/乱序与时间推进的全部交错；发现违反时返回最短反例。

不变量（每步检查）：
- SINGLE_INFLIGHT（仅 Simulator，按日志核对）：每机最多一个在途提案。
- SINGLE_COMMITTED (I1)：同一成员已提交的新旧计划名义执行期不重叠（回退分支可延续）。
- DOUBLE_CONFIRM (I10)：两机各自认为已获全员确认的计划（committed）互不冲突。
- ACK_BINDING (I11)：每机自身可能执行/在途的计划不与它已 ACK 而未结的预留冲突。
- COLLISION (I10/I14)：任意两机实际可能执行的计划占用不冲突。用参考检查器判定；
  真实占用（#10）下参考检查器按计划完整占用（运动 + 制动分支 + 终端保持）计算，不信任节点声明。
- COVERAGE (I12/I13/I3)：任一机仍可能执行的已提交计划，所有仍在运行的邻机都记得
  （作为已提交计划或仍持有的预留）——即未收齐 ACK 不会提交，预留不会过早释放。
- 本模型是有界枚举，不是任意故障下的证明。
"""

from __future__ import annotations

import copy
import heapq
import json
import random
from collections import Counter, deque
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from yopo_swarm_core.protocol.messages import Message, canonical
from yopo_swarm_core.protocol.node import NodeMode, PlanRecord, ProtocolConfig, ProtocolNode
from yopo_swarm_core.protocol.occupancy import AbstractConflictChecker, ConflictChecker

NodeFactory = Callable[[str, ProtocolConfig], ProtocolNode]


# --- 不变量 -----------------------------------------------------------------


@dataclass(frozen=True)
class Violation:
    invariant: str
    detail: str


def physical_plans(node: ProtocolNode) -> list[PlanRecord]:
    out = [node.committed]
    if node.previous is not None:
        out.append(node.previous)
    return out


def check_invariants(
    nodes: dict[str, ProtocolNode],
    t: float,
    physical: dict[str, list[PlanRecord]] | None = None,
    frozen: Iterable[str] = (),
    checker: ConflictChecker | None = None,
) -> list[Violation]:
    """physical：各机实际可能执行的计划（默认取节点自身的 committed/previous）。
    frozen：已重启的成员（其内存状态不再代表实际运动，由 physical 记录冻结）。"""
    frozen = set(frozen)
    checker = checker or AbstractConflictChecker()  # 参考检查器，独立于被测节点
    phys = physical or {m: physical_plans(n) for m, n in nodes.items()}
    out: list[Violation] = []
    for m, n in nodes.items():
        if m in frozen:
            continue
        if n.previous is not None and n.previous.occupancy.execution_end() > n.committed.t_start:
            out.append(Violation("SINGLE_COMMITTED", f"{m} previous overlaps committed"))
        reserved = [r for p in n.peers.values() for _, r in sorted(p.reservations.items())]
        for own in n.own_possible():
            for r in reserved:
                if checker.conflicts(own, r.occupancy):
                    out.append(Violation("ACK_BINDING", f"{m} own plan x acked {r.plan_hash[:8]}"))
    names = sorted(nodes)
    live = [m for m in names if m not in frozen]
    for i, a in enumerate(live):
        for b in live[i + 1 :]:
            if checker.conflicts(nodes[a].committed.occupancy, nodes[b].committed.occupancy):
                out.append(Violation("DOUBLE_CONFIRM", f"{a} and {b} committed conflicting plans"))
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            for ra in phys[a]:
                for rb in phys[b]:
                    if checker.conflicts(ra.occupancy, rb.occupancy):
                        out.append(
                            Violation(
                                "COLLISION", f"{a}:{ra.plan_hash[:8]} x {b}:{rb.plan_hash[:8]}"
                            )
                        )
    for owner in names:
        for rec in phys[owner]:
            if rec.occupancy.end_time() <= t:
                continue
            for m in names:
                n = nodes[m]
                if m == owner or m in frozen or n.mode is not NodeMode.ACTIVE:
                    continue
                if rec.plan_hash not in n.possible_plan_hashes(owner, t):
                    out.append(
                        Violation("COVERAGE", f"{m} forgot live plan {owner}:{rec.plan_hash[:8]}")
                    )
    return out


# --- 随机故障运行 -----------------------------------------------------------


@dataclass(frozen=True)
class FaultPlan:
    drop: float = 0.0
    duplicate: float = 0.0
    delay: tuple[float, float] = (0.01, 0.05)  # 随机时延，天然产生乱序
    partitions: tuple[tuple[float, float, frozenset[str]], ...] = ()  # [t0,t1) 内该组与其余断开
    restarts: tuple[tuple[float, str], ...] = ()
    clock_pauses: tuple[tuple[float, float, str], ...] = ()
    clock_resets: tuple[tuple[float, str, float], ...] = ()  # (t, node, 回拨秒数)
    task_resets: tuple[tuple[float, str, str], ...] = ()


FAULT_CATEGORIES = (
    "detected",
    "blocked_new_plans",
    "legal_fallback",
    "out_of_envelope",
    "oracle_pause",
    "collision",
)


@dataclass
class RunResult:
    trace: list[dict]
    violations: list[tuple[float, Violation]]
    nodes: dict[str, ProtocolNode]
    stats: dict[str, int]
    rejections: dict[str, int]

    def trace_bytes(self) -> bytes:
        return "\n".join(
            json.dumps(x, sort_keys=True, separators=(",", ":")) for x in self.trace
        ).encode()


class Simulator:
    def __init__(
        self,
        cfg: ProtocolConfig,
        faults: FaultPlan | None = None,
        seed: int = 0,
        tick: float = 0.1,
        node_factory: NodeFactory = ProtocolNode,
        stop_on_violation: bool = True,
        checker: ConflictChecker | None = None,
    ) -> None:
        self.cfg = cfg
        self.checker = checker
        self.faults = faults or FaultPlan()
        self.rng = random.Random(seed)
        self.tick = tick
        self.nodes = {m: node_factory(m, cfg) for m in cfg.members}
        self.stop_on_violation = stop_on_violation
        self._q: list[tuple[float, int, str, Any]] = []
        self._order = 0
        self.offset = dict.fromkeys(cfg.members, 0.0)
        self.paused_at: dict[str, float | None] = dict.fromkeys(cfg.members, None)
        self.frozen: set[str] = set()
        self.timeline: dict[str, list[PlanRecord]] = {
            m: physical_plans(n) for m, n in self.nodes.items()
        }
        self.trace: list[dict] = []
        self.violations: list[tuple[float, Violation]] = []
        self.network = Counter()
        self.open_proposal: dict[str, str | None] = {}

    def _push(self, t: float, kind: str, payload: Any) -> None:
        self._order += 1
        heapq.heappush(self._q, (t, self._order, kind, payload))

    def local_time(self, m: str, t: float) -> float:
        base = self.paused_at[m] if self.paused_at[m] is not None else t
        return base - self.offset[m]

    def _partitioned(self, a: str, b: str, t: float) -> bool:
        for t0, t1, group in self.faults.partitions:
            if t0 <= t < t1 and ((a in group) != (b in group)):
                return True
        return False

    def _send(self, t: float, sender: str, msgs: list[Message]) -> None:
        for msg in msgs:
            self.trace.append({"t": round(t, 9), "send": sender, "msg": msg.to_json()})
            dests = (
                [msg.dest]
                if getattr(msg, "dest", None)
                else [m for m in self.cfg.members if m != sender]
            )
            for d in dests:
                if self._partitioned(sender, d, t):
                    self.network["partition_drop"] += 1
                    continue
                if self.rng.random() < self.faults.drop:
                    self.network["drop"] += 1
                    continue
                copies = 2 if self.rng.random() < self.faults.duplicate else 1
                self.network["duplicate"] += copies - 1
                for _ in range(copies):
                    lo, hi = self.faults.delay
                    self._push(t + self.rng.uniform(lo, hi), "deliver", (d, msg))

    def _drain_events(self, t: float, node: ProtocolNode) -> list[Violation]:
        """记录节点事件，并按日志独立核对“每机最多一个在途提案”。"""
        out = []
        for e in node.events:
            self.trace.append({"t": round(t, 9), **e})
            ev = e["event"]
            if ev == "propose":
                if self.open_proposal.get(node.id) is not None:
                    out.append(
                        Violation(
                            "SINGLE_INFLIGHT",
                            f"{node.id} proposed while {self.open_proposal[node.id]} open",
                        )
                    )
                self.open_proposal[node.id] = e["rid"]
            elif ev in ("commit", "abort", "restart", "clock_reset", "task_reset"):
                self.open_proposal[node.id] = None
        node.events.clear()
        return out

    def _update_timeline(self, m: str) -> None:
        if m in self.frozen:
            return
        n = self.nodes[m]
        tl = self.timeline[m]
        if n.committed.plan_hash != tl[0].plan_hash:
            self.timeline[m] = physical_plans(n)

    def run(self, until: float) -> RunResult:
        n = len(self.cfg.members)
        for i, m in enumerate(self.cfg.members):
            self._push(i * self.tick / n, "tick", m)
        for t, m in self.faults.restarts:
            self._push(t, "restart", m)
        for t0, t1, m in self.faults.clock_pauses:
            self._push(t0, "pause", m)
            self._push(t1, "resume", m)
        for t, m, dt in self.faults.clock_resets:
            self._push(t, "clock_reset", (m, dt))
        for t, m, h in self.faults.task_resets:
            self._push(t, "task_reset", (m, h))

        while self._q:
            t, _, kind, payload = heapq.heappop(self._q)
            if t > until:
                break
            if kind == "tick":
                m = payload
                out = self.nodes[m].on_tick(self.local_time(m, t))
                self._push(t + self.tick, "tick", m)
            elif kind == "deliver":
                m, msg = payload
                self.trace.append({"t": round(t, 9), "recv": m, "msg": msg.to_json()})
                out = self.nodes[m].on_message(msg, self.local_time(m, t))
            elif kind == "restart":
                m = payload
                self.frozen.add(m)  # 实际运动按重启前已批准计划冻结（watchdog 假设）
                self.nodes[m].restart(t)
                out = []
            elif kind == "pause":
                m = payload
                self.paused_at[m] = t
                out = []
            elif kind == "resume":
                m = payload
                self.offset[m] += t - self.paused_at[m]
                self.paused_at[m] = None
                out = []
            elif kind == "clock_reset":
                m, dt = payload
                self.offset[m] += dt
                self.nodes[m].clock_reset()
                out = []
            else:  # task_reset
                m, h = payload
                self.nodes[m].task_reset(h)
                out = []
            node = self.nodes[m]
            self._update_timeline(m)
            viol = self._drain_events(t, node)
            self._send(t, m, out)
            self.trace.append({"t": round(t, 9), "state": m, "snapshot": node.snapshot()})
            viol += check_invariants(self.nodes, t, self.timeline, self.frozen, self.checker)
            if viol:
                self.violations += [(t, v) for v in viol]
                self.trace.append({"t": round(t, 9), "violation": [canonical(v) for v in viol]})
                if self.stop_on_violation:
                    break
        return self._result()

    def _result(self) -> RunResult:
        rej = Counter()
        aborts = skips = halts = 0
        for line in self.trace:
            ev = line.get("event")
            if ev == "reject":
                rej[line["reason"]] += 1
            elif ev == "abort":
                aborts += 1
            elif ev == "skip":
                skips += 1
            elif ev == "halt":
                halts += 1
        fallback = sum(1 for n in self.nodes.values() if n.mode is NodeMode.HALTED)
        collisions = sum(1 for _, v in self.violations if v.invariant == "COLLISION")
        stats = {
            "detected": sum(rej.values()),
            "blocked_new_plans": aborts + skips + halts,
            "legal_fallback": fallback,
            # 测试器假设完美跟踪（误差由 tracking_margin 覆盖），超包络只在 G4 物理仿真中可观测
            "out_of_envelope": 0,
            "oracle_pause": 1 if self.violations else 0,
            "collision": collisions,
        }
        return RunResult(self.trace, self.violations, self.nodes, stats, dict(rej))


FAULT_CLASSES: dict[str, Callable[[ProtocolConfig], FaultPlan]] = {
    "none": lambda c: FaultPlan(),
    "drop": lambda c: FaultPlan(drop=0.2),
    "duplicate": lambda c: FaultPlan(duplicate=0.3),
    "reorder": lambda c: FaultPlan(delay=(0.0, 0.6 * c.window_len)),
    "partition": lambda c: FaultPlan(
        partitions=((3 * c.window_len, 12 * c.window_len, frozenset(c.members[:1])),)
    ),
    "restart": lambda c: FaultPlan(restarts=((4.3 * c.window_len, c.members[-1]),)),
    "clock_pause": lambda c: FaultPlan(
        clock_pauses=((2.2 * c.window_len, 4.7 * c.window_len, c.members[0]),)
    ),
    "clock_reset": lambda c: FaultPlan(clock_resets=((3.1 * c.window_len, c.members[0], 0.7),)),
    "task_reset": lambda c: FaultPlan(task_resets=((3.1 * c.window_len, c.members[0], "task-2"),)),
}


def fault_matrix(
    cfg: ProtocolConfig,
    seeds: Iterable[int],
    until: float,
    node_factory: NodeFactory = ProtocolNode,
    checker: ConflictChecker | None = None,
) -> dict[str, dict[str, int]]:
    """按故障类别统计：检测、禁止新计划、合法回退、超包络、oracle 暂停、碰撞。"""
    table: dict[str, dict[str, int]] = {}
    seeds = list(seeds)
    for name, make in FAULT_CLASSES.items():
        total = Counter()
        for s in seeds:
            sim = Simulator(cfg, make(cfg), seed=s, node_factory=node_factory, checker=checker)
            total.update(sim.run(until).stats)
        table[name] = {k: total[k] for k in FAULT_CATEGORIES}
    return table


# --- 有界穷举 ---------------------------------------------------------------


@dataclass
class ExploreSummary:
    members: tuple[str, ...]
    max_steps: int
    max_plan_seq: int
    queue_depth: int
    time_step: float
    faults: tuple[str, ...]
    states: int = 0
    transitions: int = 0
    max_depth_reached: int = 0
    pruned_queue_depth: int = 0
    leaves_step_bound: int = 0
    complete: bool = False
    violation: dict | None = None
    counterexample: list[str] = field(default_factory=list)

    def to_json(self) -> dict:
        return canonical(self.__dict__) | {
            "members": list(self.members),
            "faults": list(self.faults),
        }

    def save(self, path: Path) -> None:
        Path(path).write_text(
            json.dumps(self.to_json(), indent=2, sort_keys=True, ensure_ascii=False)
        )


@dataclass
class _State:
    t: float
    nodes: dict[str, ProtocolNode]
    bag: list[tuple[str, Message]]  # (dest, msg)

    def key(self) -> tuple:
        bag = sorted(json.dumps([d, m.to_json()], sort_keys=True) for d, m in self.bag)
        return (
            round(self.t, 9),
            tuple(n.snapshot() for _, n in sorted(self.nodes.items())),
            tuple(bag),
        )


def _fanout(cfg: ProtocolConfig, sender: str, msgs: list[Message]) -> list[tuple[str, Message]]:
    out = []
    for m in msgs:
        dests = [m.dest] if getattr(m, "dest", None) else [x for x in cfg.members if x != sender]
        out += [(d, m) for d in dests]
    return out


def explore(
    cfg: ProtocolConfig,
    max_steps: int,
    max_plan_seq: int = 1,
    queue_depth: int = 4,
    time_step: float | None = None,
    faults: tuple[str, ...] = ("drop", "duplicate", "reorder"),
    node_factory: NodeFactory = ProtocolNode,
    max_states: int = 2_000_000,
    checker: ConflictChecker | None = None,
) -> ExploreSummary:
    """BFS 枚举全部交错，返回摘要；发现违反即停，counterexample 为最短动作序列。

    动作：TICK（全局时间前进 time_step，各节点按成员顺序 on_tick）、DELIVER i、
    DROP i（drop 故障）、DUP i（duplicate 故障）。不含 reorder 时只能投递队首。
    每个节点最多发起 max_plan_seq 个提案；网络中消息超过 queue_depth 的分支剪掉并计数。
    """
    dt = time_step if time_step is not None else cfg.window_len / 2
    bounded = replace(cfg, max_proposals=max_plan_seq)
    summary = ExploreSummary(cfg.members, max_steps, max_plan_seq, queue_depth, dt, tuple(faults))
    init = _State(0.0, {m: node_factory(m, bounded) for m in cfg.members}, [])
    start = init.key()
    parent: dict[tuple, tuple[tuple, str] | None] = {start: None}
    frontier = deque([(init, start, 0)])

    def path_to(key: tuple) -> list[str]:
        out = []
        while (link := parent[key]) is not None:
            key, action = link
            out.append(action)
        return out[::-1]

    def tick(s: _State) -> None:
        s.t += dt
        for m in cfg.members:
            s.bag += _fanout(cfg, m, s.nodes[m].on_tick(s.t))

    while frontier:
        state, skey, depth = frontier.popleft()
        summary.max_depth_reached = max(summary.max_depth_reached, depth)
        if depth >= max_steps:
            summary.leaves_step_bound += 1
            continue
        actions: list[tuple[str, Callable[[_State], None]]] = [("TICK", tick)]
        idxs = range(len(state.bag)) if "reorder" in faults else range(min(1, len(state.bag)))
        for i in idxs:
            d, m = state.bag[i]
            label = f"{m.kind}#{m.ctx.sender_id}.{m.ctx.message_seq}->{d}"

            def deliver(s: _State, i: int = i) -> None:
                dest, msg = s.bag.pop(i)
                s.bag += _fanout(cfg, dest, s.nodes[dest].on_message(msg, s.t))

            actions.append((f"DELIVER {label}", deliver))
            if "drop" in faults:
                actions.append((f"DROP {label}", lambda s, i=i: s.bag.pop(i)))
            if "duplicate" in faults:
                actions.append((f"DUP {label}", lambda s, i=i: s.bag.append(s.bag[i])))

        for label, act in actions:
            nxt = copy.deepcopy(state)
            act(nxt)
            for n in nxt.nodes.values():
                n.events.clear()
                n.rejections.clear()
            summary.transitions += 1
            if len(nxt.bag) > queue_depth:
                summary.pruned_queue_depth += 1
                continue
            k = nxt.key()
            if k in parent:
                continue
            parent[k] = (skey, label)
            summary.states += 1
            viol = check_invariants(nxt.nodes, nxt.t, checker=checker)
            if viol:
                summary.violation = canonical(viol[0])
                summary.counterexample = path_to(k)
                return summary
            if summary.states >= max_states:
                return summary
            frontier.append((nxt, k, depth + 1))
    summary.complete = True
    return summary


__all__ = [
    "FAULT_CATEGORIES",
    "FAULT_CLASSES",
    "ExploreSummary",
    "FaultPlan",
    "RunResult",
    "Simulator",
    "Violation",
    "check_invariants",
    "explore",
    "fault_matrix",
]
