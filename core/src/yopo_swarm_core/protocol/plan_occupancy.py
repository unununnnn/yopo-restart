"""#10：把 #8 的真实计划（运动前缀 + 固定分支点制动分支 + 终端保持，ADR-0002）接入协议节点。

- `PlanOccupancy`：实现协议的占用接口。被新计划在 `until` 替换后，名义运动截到 until；
  until 之前的制动分支整体保留（对端无法知道本机是否已刹车）；终端保持只覆盖 [t_end, until)。
- `PlanConflictChecker`：冲突判定走 #4 的连续时间分离检查（UNKNOWN 按冲突）。
  `physical=True` 时总按计划本身的完整占用判定（参考检查器，不信任声明的占用）。
- `RoutePlanner` / `RealScenario`：测试器用的确定性规划器与场景（候选目标按偏好顺序，
  与邻机可能占用及自己已发出的 ACK 冲突的候选一律不提）。
- `reference_exploration()`：入库的双机有界穷举记录（docs/ref/g3-explore-2uav.json）。

运行假设（与离散事件测试器一致）：车辆完美跟踪已批准分支（跟踪误差由 tracking_margin 覆盖），
因此本层的「超包络」计数恒为 0；跟踪误差外偏离属于 G4 物理仿真。
"""

from __future__ import annotations

import argparse
import json
import math
from collections import OrderedDict
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
from functools import cached_property
from pathlib import Path
from typing import Any, ClassVar

import numpy as np

from yopo_swarm_core.plan import (
    Hold,
    OccupancySet,
    Plan,
    PlanConfig,
    Tube,
    canonical_digest,
    conflicts,
    hover_plan,
    make_plan,
    quintic,
    state_at,
    validate_plan,
)
from yopo_swarm_core.protocol.node import ProtocolConfig, ProtocolNode
from yopo_swarm_core.trajectory import PiecewisePolynomial

Vec3 = tuple[float, float, float]


@dataclass(frozen=True, eq=False)
class PlanOccupancy:
    plan: Plan
    cfg: PlanConfig
    until: float = math.inf  # 名义运动在此刻被新计划替换

    def __deepcopy__(self, memo: dict) -> PlanOccupancy:
        return self  # 不可变；穷举时共享，避免复制多项式

    # --- 协议占用接口 ----------------------------------------------------
    def truncated(self, until: float) -> PlanOccupancy:
        return replace(self, until=min(self.until, float(until)))

    def end_time(self) -> float:
        ends = [
            e.traj.t_end if isinstance(e, Tube) else e.t_until
            for e in self.occupancy_set().elements
        ]
        return max(ends, default=-math.inf)

    def execution_end(self) -> float:
        return self.until

    def is_empty(self) -> bool:
        return not self.occupancy_set().elements

    def is_finite(self) -> bool:
        """有限且满足 ADR-0002 计划契约（validate_plan 无拒绝）。"""
        return self._contract_ok

    def to_canonical(self) -> Any:
        u = "inf" if self.until == math.inf else float(self.until).hex()
        return {"plan": self.plan.canonical(self.cfg), "until": u}

    # --- 占用集合 --------------------------------------------------------
    def occupancy_set(self) -> OccupancySet:
        """本节点声明的占用（变异体测试会覆盖此方法）。"""
        return self.physical_set()

    def physical_set(self) -> OccupancySet:
        return self._physical

    @cached_property
    def _physical(self) -> OccupancySet:
        r, u, m = self.cfg.tube_radius, self.until, self.plan.motion
        els: list[Tube | Hold] = []
        if m.t_start < u:
            tr = m if u >= m.t_end else m.with_validity(m.t_start, u)
            els.append(Tube(tr, r, "motion"))
        for i, b in enumerate(self.plan.branches):
            if b.t_branch < u:
                els.append(Tube(b.traj, r, f"branch{i}"))
                stop = tuple(float(x) for x in b.stop_point())
                els.append(Hold(stop, r, b.traj.t_end, math.inf, f"branch{i}-hold"))
        if self.plan.t_end < u:
            term = tuple(float(x) for x in self.plan.terminal_point())
            els.append(Hold(term, r, self.plan.t_end, u, "terminal"))
        return OccupancySet(tuple(els))

    @cached_property
    def _contract_ok(self) -> bool:
        if math.isnan(self.until):
            return False
        try:
            return not validate_plan(self.plan, self.cfg)
        except (ValueError, np.linalg.LinAlgError, FloatingPointError):
            return False

    @cached_property
    def digest(self) -> str:
        return canonical_digest(self.to_canonical())


class PlanConflictChecker:
    """True = 冲突或无法证明分离。结果按占用摘要缓存（进程内，跨深拷贝共享，LRU 有界）。"""

    cache_max: ClassVar[int] = 200_000
    _cache: ClassVar[OrderedDict[tuple, bool]] = OrderedDict()

    def __init__(self, physical: bool = False) -> None:
        self.physical = physical

    def __deepcopy__(self, memo: dict) -> PlanConflictChecker:
        return self

    def _key(self, o: PlanOccupancy) -> str:
        return o.digest if self.physical else f"{type(o).__qualname__}:{o.digest}"

    def _set(self, o: PlanOccupancy) -> OccupancySet:
        return o.physical_set() if self.physical else o.occupancy_set()

    def conflicts(self, a: PlanOccupancy, b: PlanOccupancy) -> bool:
        ka, kb = self._key(a), self._key(b)
        key = (ka, kb) if ka <= kb else (kb, ka)
        cache = self._cache
        hit = cache.get(key)
        if hit is None:
            hit = cache[key] = conflicts(self._set(a), self._set(b))
            while len(cache) > self.cache_max:
                cache.popitem(last=False)
        else:
            cache.move_to_end(key)
        return hit


# --- 规划器与场景 ---------------------------------------------------------------


def hover_occupancy(
    owner: str, position: Sequence[float], cfg: PlanConfig, cls: type[PlanOccupancy] = PlanOccupancy
) -> PlanOccupancy:
    return cls(hover_plan(owner, np.asarray(position, float), 0.0, cfg, parent=""), cfg)


@dataclass(frozen=True)
class RealScenario:
    cfg: ProtocolConfig
    pcfg: PlanConfig
    starts: Mapping[str, Vec3]
    routes: Mapping[str, Sequence[Vec3]]  # 每机候选目标，按偏好顺序
    duration: float = 4.0
    branch_offsets: tuple[float, ...] = (0.0, 0.5)
    occupancy_cls: type[PlanOccupancy] = PlanOccupancy

    @property
    def planner(self) -> RoutePlanner:
        return RoutePlanner(self)

    def initial(self, owner: str, cfg: ProtocolConfig) -> PlanOccupancy:
        return hover_occupancy(owner, self.starts[owner], self.pcfg, self.occupancy_cls)

    def node(
        self,
        member: str,
        cfg: ProtocolConfig | None = None,
        node_cls: type[ProtocolNode] = ProtocolNode,
        checker: PlanConflictChecker | None = None,
    ) -> ProtocolNode:
        return node_cls(
            member,
            cfg or self.cfg,
            checker=checker or PlanConflictChecker(),
            planner=self.planner,
            initial=self.initial,
        )

    def factory(
        self, node_cls: type[ProtocolNode] = ProtocolNode
    ) -> Callable[[str, ProtocolConfig], ProtocolNode]:
        return lambda m, c: self.node(m, c, node_cls)


@dataclass(frozen=True)
class RoutePlanner:
    sc: RealScenario

    def __call__(self, node: ProtocolNode, now: float, t_start: float) -> PlanOccupancy | None:
        parent = node.committed.occupancy.plan
        head = state_at(parent, t_start)
        for target in self.sc.routes[node.id]:
            goal = np.asarray(target, float)
            if np.linalg.norm(goal - parent.terminal_point()) <= 1e-9:
                continue  # 当前计划已去往（或停在）该目标
            occ = self._to(node, head, t_start, goal)
            if occ is not None and not node.conflicts_with_neighbors(occ):
                return occ
        return None  # 没有安全候选：继续执行当前已批准计划（含其分支与终端保持）

    def _to(
        self, node: ProtocolNode, head: np.ndarray, t_start: float, goal: np.ndarray
    ) -> PlanOccupancy | None:
        end = np.array([goal, np.zeros(3), np.zeros(3)])
        motion = PiecewisePolynomial(t_start, [quintic(head, end, self.sc.duration)])
        res = make_plan(
            node.id,
            node.plan_seq + 1,
            node.committed.plan_hash,
            motion,
            t_start,
            head,
            [t_start + o for o in self.sc.branch_offsets],
            self.sc.pcfg,
        )
        return self.sc.occupancy_cls(res, self.sc.pcfg) if isinstance(res, Plan) else None


# --- 入库的有界穷举记录 ---------------------------------------------------------

REFERENCE_PCFG_ARGS = {"v_max": 4.0, "a_max": 6.0, "j_max": 40.0}
REFERENCE_SCOPE = {
    "max_steps": 10,
    "max_plan_seq": 1,
    "queue_depth": 3,
    "faults": ["drop", "duplicate", "reorder"],
}
RECORD_PATH = Path("docs/ref/g3-explore-2uav.json")


def reference_scenario() -> RealScenario:
    from yopo_swarm_core.plan import KinematicLimits

    pcfg = PlanConfig(
        KinematicLimits(**REFERENCE_PCFG_ARGS),
        max_branch_points=2,
        body_radius=0.15,
        tracking_margin=0.05,
    )
    cfg = ProtocolConfig(members=("A", "B"), resend_period=None)
    return RealScenario(
        cfg,
        pcfg,
        {"A": (0.0, 0.0, 0.0), "B": (2.0, -2.0, 0.0)},
        {"A": [(4.0, 0.0, 0.0)], "B": [(2.0, 2.0, 0.0)]},
    )


def reference_exploration() -> dict:
    """两机在 (2,0,0) 交叉；枚举投递/丢失/重复/乱序与时间推进。返回范围 + 结果。"""
    from yopo_swarm_core.protocol.des import explore

    sc = reference_scenario()
    s = explore(
        sc.cfg,
        max_steps=REFERENCE_SCOPE["max_steps"],
        max_plan_seq=REFERENCE_SCOPE["max_plan_seq"],
        queue_depth=REFERENCE_SCOPE["queue_depth"],
        faults=tuple(REFERENCE_SCOPE["faults"]),
        node_factory=sc.factory(),
        checker=PlanConflictChecker(physical=True),
    )
    return json.loads(
        json.dumps(
            {
                "scenario": {
                    "members": list(sc.cfg.members),
                    "starts": {k: list(v) for k, v in sc.starts.items()},
                    "routes": {k: [list(p) for p in v] for k, v in sc.routes.items()},
                    "duration": sc.duration,
                    "branch_offsets": list(sc.branch_offsets),
                    "plan_config": sc.pcfg.to_canonical(),
                    "window_len": sc.cfg.window_len,
                },
                "invariants": [
                    "SINGLE_COMMITTED(I1)",
                    "DOUBLE_CONFIRM(I10)",
                    "ACK_BINDING(I11)",
                    "COLLISION(I10/I14, 参考检查器按完整占用)",
                    "COVERAGE(I3/I12/I13)",
                ],
                "summary": s.to_json(),
            },
            sort_keys=True,
            ensure_ascii=False,
        )
    )


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="重跑并写入 G3 双机有界穷举记录")
    ap.add_argument("--out", type=Path, default=RECORD_PATH)
    args = ap.parse_args(argv)
    rec = reference_exploration()
    args.out.write_text(
        json.dumps(rec, indent=2, sort_keys=True, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(rec["summary"], ensure_ascii=False))
    return 0 if rec["summary"]["violation"] is None else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "PlanConflictChecker",
    "PlanOccupancy",
    "RealScenario",
    "RoutePlanner",
    "hover_occupancy",
    "reference_exploration",
]
