"""票 13e：被替换的旧计划留下的刹车停点不再挡六机林带提案。三机和柱林仍走 conflicts_with_neighbors。"""

import numpy as np
import pytest

from yopo_l0.crossing import CrossingFleet
from yopo_l0.scene import Cylinder, load_card
from yopo_swarm_core.plan import hover_plan, quintic
from yopo_swarm_core.protocol.node import PlanRecord
from yopo_swarm_core.protocol.plan_occupancy import PlanOccupancy
from yopo_swarm_core.trajectory import PiecewisePolynomial

from conftest import ROOT

SIX = ROOT / "scenes/crossing-6-forest.json"
THREE = ROOT / "scenes/crossing-forest.json"
PILLARS = ROOT / "scenes/crossing-pillars.json"

_Y = 40.0  # 远离起点和树的一条侧线，只让邻机的悬停记录参与冲突。
_T0 = 10.0


def _hover(owner: str, pos, t0: float, seq: int, pcfg, until: float | None = None) -> PlanRecord:
    occ = PlanOccupancy(hover_plan(owner, np.asarray(pos, dtype=float), t0, pcfg), pcfg)
    if until is not None:
        occ = occ.truncated(until)
    return PlanRecord(owner, seq, f"{owner}-{seq}", "", t0, occ)


def _setup(card_path, *, ghost_is_live: bool = False):
    fleet = CrossingFleet.from_card(load_card(card_path).card, 0.01)
    me, other = fleet.members[0], fleet.members[1]
    node = fleet.nodes[me]
    pcfg = fleet.pcfg
    # 只测机间自检。树和净空门有自己的测试。留一棵远处的树，六机林带分支照常生效。
    fleet._trees = (Cylinder("t-far", (0.0, -40.0), 0.2, 0.0, 6.0),)
    in_path = (22.0, _Y, 1.5)
    if ghost_is_live:
        plans = [_hover(other, in_path, 8.0, 3, pcfg)]
    else:
        # 邻机 5 s 时在路上悬停过，6 s、8 s 又提交了两份远处的计划。5 s 那份已被替换。
        plans = [
            _hover(other, in_path, 5.0, 1, pcfg, until=6.0),
            _hover(other, (60.0, _Y, 1.5), 6.0, 2, pcfg, until=8.0),
            _hover(other, (61.0, _Y, 1.5), 8.0, 3, pcfg),
        ]
    node.peers[other].plans = plans
    head = np.array([[20.0, _Y, 1.5], [2.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    end = np.array([[24.0, _Y, 1.5], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    motion = PiecewisePolynomial(_T0, [quintic(head, end, 2.0)])
    return fleet, node, head, motion


def test_superseded_brake_stop_no_longer_blocks_six_drone_forest_proposal():
    fleet, node, head, motion = _setup(SIX)
    assert fleet._forest_six()
    occ, reject = fleet._as_plan(node, _T0, head, motion)
    assert reject is None and occ is not None
    # 旧口径（全部历史计划）仍会被 5 s 那份计划的分支停点挡住。
    assert node.conflicts_with_neighbors(occ) is True


def test_live_hover_in_the_path_still_blocks_six_drone_forest_proposal():
    fleet, node, head, motion = _setup(SIX, ghost_is_live=True)
    occ, reject = fleet._as_plan(node, _T0, head, motion)
    assert occ is None and reject == "CONFLICT_SKIP"


@pytest.mark.parametrize("card", [THREE, PILLARS])
def test_three_drone_and_pillar_crossing_keep_conflicts_with_neighbors(card):
    fleet, node, head, motion = _setup(card)
    assert not fleet._forest_six()
    calls = []
    original = node.conflicts_with_neighbors

    def spy(occ):
        calls.append(occ)
        return original(occ)

    node.conflicts_with_neighbors = spy
    occ, reject = fleet._as_plan(node, _T0, head, motion)
    assert calls, "三机和柱林必须走 conflicts_with_neighbors"
    assert occ is None and reject == "CONFLICT_SKIP"
