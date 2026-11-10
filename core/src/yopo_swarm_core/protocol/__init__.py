"""G3 双机协议（ROS-free）：轮转提案窗口、预留 ACK/COMMIT/RELEASE。

占用可以是抽象占用（#5）或真实计划占用（#10，见 plan_occupancy）。
"""

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
)
from yopo_swarm_core.protocol.node import (
    NodeMode,
    PlanRecord,
    ProtocolConfig,
    ProtocolNode,
    TrackPhase,
    default_planner,
)
from yopo_swarm_core.protocol.occupancy import (
    AbstractConflictChecker,
    AbstractOccupancy,
    ConflictChecker,
    Span,
)

__all__ = [
    "AbstractConflictChecker",
    "AbstractOccupancy",
    "Commit",
    "ConflictChecker",
    "Decision",
    "Heartbeat",
    "Message",
    "NodeMode",
    "PlanRecord",
    "Proposal",
    "ProtocolConfig",
    "ProtocolNode",
    "Reason",
    "Release",
    "Span",
    "TrackPhase",
    "VersionContext",
    "Vote",
    "default_planner",
]
