"""票 10b：三机 M1 的 ROS 2 形态和进程内形态，同 seed 结论一致。

比的是到达、碰撞、队形误差 p95、最小机间距离、影子拒绝率。
机间协议消息要经各机话题往返；rclpy 导入不了时，带 ROS 的用例跳过。
编码往返、故障抽样和传输钩子不依赖 ROS，始终跑。
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from yopo_l0.bus import BusFaults, MessageBus, _Item
from yopo_l0.formation import FormationFleet
from yopo_l0.ros2_bus import Ros2MessageBus
from yopo_l0.ros2_codec import decode_bus_messages, encode_bus_item
from yopo_l0.ros2_env import ensure_ros_paths
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card
from yopo_swarm_core.protocol.messages import (
    Commit,
    Decision,
    Heartbeat,
    Reason,
    Release,
    VersionContext,
    Vote,
)
from yopo_swarm_core.protocol.node import plan_hash_of

from conftest import ROOT
from test_ros2_form import _record, needs_ros

FORMATION = ROOT / "scenes/formation-hold.json"


def _item(**kw) -> SimpleNamespace:
    base = dict(seq=1, src="uav0", dest="uav1", topic="protocol", sent_at=0.0, deliver_at=0.0, rank=1)
    base.update(kw)
    return SimpleNamespace(**base)


def _roundtrip(item):
    return decode_bus_messages(encode_bus_item(item))


def test_protocol_roundtrip_preserves_plan_hash_and_canonical():
    card = load_card(FORMATION)
    fleet = FormationFleet.from_card(card.card, 0.01)

    class Grab:
        def __init__(self):
            self.payloads = []

        def publish(self, now, *, src, dest, topic, payload):
            self.payloads.append(payload)

        def poll(self, now):
            return []

    grab = Grab()
    fleet.exchange(grab, 0.0)
    assert grab.payloads and grab.payloads[0]["kind"] == "protocol"
    msg = grab.payloads[0]["msg"]
    got = _roundtrip(_item(payload={"kind": "protocol", "msg": msg}, sent_at=0.25, deliver_at=0.5, rank=4, seq=9))
    new = got["payload"]["msg"]
    assert got["seq"] == 9 and got["src"] == "uav0" and got["dest"] == "uav1" and got["rank"] == 4
    assert new.to_json() == msg.to_json()
    assert new.occupancy.to_canonical() == msg.occupancy.to_canonical()
    assert new.plan_hash == msg.plan_hash
    assert plan_hash_of(
        new.proposer_id, new.plan_seq, new.parent_plan_hash, new.t_start, new.occupancy, fleet.cfg,
    ) == msg.plan_hash

    ctx = VersionContext(0, "task", 0, 0, "uav1", 3)
    others = [
        Vote(
            ctx=ctx, reservation_id="uav0/0/1", proposer_id="uav0", plan_hash="h",
            validity=(0.5, math.inf), voter_occupancy_version="p", dependency_digest="d",
            decision=Decision.ACK, reason=Reason.OK, dest="uav0",
        ),
        Commit(ctx=ctx, reservation_id="uav0/0/1", plan_hash="h", dest=None),
        Release(ctx=ctx, reservation_id="uav0/0/1", plan_hash="h"),
        Heartbeat(ctx=ctx, dest="uav2"),
    ]
    for message in others:
        back = _roundtrip(_item(payload={"kind": "protocol", "msg": message}))["payload"]["msg"]
        assert back.to_json() == message.to_json()


def test_bus_item_chunks_roundtrip_when_pieces_are_small(monkeypatch):
    import yopo_l0.ros2_codec as codec

    item = _item(payload={"kind": "self_plan", "token": 7}, topic="plan", sent_at=0.02, deliver_at=0.02)
    monkeypatch.setattr(codec, "OBS_CHUNK_BYTES", 16)
    messages = codec.encode_bus_item(item)
    assert len(messages) > 2
    got = codec.decode_bus_messages(messages)
    assert got["payload"] == {"kind": "self_plan", "token": 7}
    assert isinstance(got["payload"]["token"], int)
    peer = _roundtrip(_item(payload={"kind": "peer_plan", "code": "LOCAL_HOVER"}, topic="plan"))
    assert peer["payload"] == {"kind": "peer_plan", "code": "LOCAL_HOVER"}


def test_ros2_bus_faults_match_inprocess_queue():
    calls = {"n": 0}

    def relay(item):
        calls["n"] += 1
        data = decode_bus_messages(encode_bus_item(item))
        return _Item(
            deliver_at=data["deliver_at"],
            rank=data["rank"],
            seq=data["seq"],
            src=data["src"],
            dest=data["dest"],
            topic=data["topic"],
            payload=data["payload"],
            sent_at=data["sent_at"],
        )

    faults = BusFaults(drop=0.3, duplicate=0.25, delay=(0.0, 0.05), reorder=0.5)
    plain = MessageBus(faults, rng=np.random.default_rng(1))
    wired = Ros2MessageBus(faults, rng=np.random.default_rng(1), relay=relay)
    for i in range(40):
        now = i * 0.01
        payload = {"kind": "self_plan", "token": i}
        plain.publish(now, src="uav0", dest="uav1", topic="plan", payload=payload)
        wired.publish(now, src="uav0", dest="uav1", topic="plan", payload=payload)
        ea, eb = plain.poll(now), wired.poll(now)
        assert [(e.seq, e.src, e.dest, e.topic, e.deliver_at, e.sent_at, e.payload) for e in ea] == [
            (e.seq, e.src, e.dest, e.topic, e.deliver_at, e.sent_at, e.payload) for e in eb
        ]
    assert plain.counts() == wired.counts()
    assert plain.dropped > 0 and plain.duplicated > 0 and plain.reordered > 0
    appends = plain.published - plain.dropped + plain.duplicated
    assert calls["n"] == appends


def test_transport_override_is_what_exchange_publishes_on():
    seen = []

    class Transport:
        def carrying(self, bus):
            class Wired:
                def publish(self, now, *, src, dest, topic, payload):
                    seen.append((src, dest, topic, payload["kind"]))

                def poll(self, now):
                    return []

            return Wired()

    card = load_card(FORMATION)
    FormationFleet.transport_override = Transport()
    try:
        run(card, RunConfig(seed=0, policy="formation", duration_s=0.05, shadow=False))
    finally:
        FormationFleet.transport_override = None
    assert FormationFleet.transport_override is None
    assert seen
    assert {kind for *_, kind in seen} == {"protocol"}
    assert {topic for *_, topic, _ in seen} == {"protocol"}


def _view(result: dict) -> dict:
    return {
        "outcome": result["outcome"],
        "reason": result["reason"],
        "arrived": result["arrived"],
        "arrival_time_s": result["arrival_time_s"],
        "collision": result["outcome"] == "COLLISION",
        "formation_error_p95_m": result["formation_error_p95_m"],
        "min_inter_drone_distance_m": result["min_inter_drone_distance_m"],
        "shadow_reject_rate": result["shadow_reject_rate"],
    }


@needs_ros
def test_bus_topics_carry_peer_and_self_envelopes():
    from yopo_l0.ros2_bus import Ros2Transport
    from yopo_l0.ros2_nodes import L0WorldBridge, init_rclpy

    distro = ensure_ros_paths()
    if distro == "humble":
        os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    else:
        os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    os.environ.setdefault("RMW_FASTRTPS_USE_SHM", "0")
    os.environ.setdefault("RCUTILS_LOGGING_SEVERITY", "WARN")
    import rclpy

    card = load_card(FORMATION)
    transport = Ros2Transport()
    bridge = L0WorldBridge(card, transport=transport)
    init_rclpy(rclpy)
    try:
        bridge.bind([{"id": i, "policy": None} for i in ("uav0", "uav1", "uav2")])
        assert bridge.bus_topics == [
            "/uav0/bus_tx", "/uav0/bus_rx", "/uav0/bus_done",
            "/uav1/bus_tx", "/uav1/bus_rx", "/uav1/bus_done",
            "/uav2/bus_tx", "/uav2/bus_rx", "/uav2/bus_done",
        ]
        for seq, dest in ((1, "uav1"), (2, "uav0")):
            item = _Item(
                deliver_at=0.5, rank=seq, seq=seq, src="uav0", dest=dest, topic="protocol",
                payload={"kind": "self_plan", "token": seq}, sent_at=0.25,
            )
            got = transport.relay(item)
            assert got.payload == {"kind": "self_plan", "token": seq}
            assert got.dest == dest and got.src == "uav0" and got.seq == seq
            assert got.deliver_at == 0.5 and got.sent_at == 0.25
        assert bridge.bus_ros_deliveries == 2
    finally:
        bridge.close()
        if rclpy.ok():
            rclpy.shutdown()


@needs_ros
def test_ros2_formation_cli_accepts_policy(tmp_path: Path):
    out = tmp_path / "ros2"
    proc = subprocess.run(
        [
            sys.executable, "-m", "yopo_l0.ros2_run",
            "--scene", str(FORMATION), "--seed", "0", "--policy", "formation",
            "--duration", "1.0", "--out", str(out),
        ],
        cwd=ROOT, capture_output=True, text=True, timeout=180,
    )
    assert proc.returncode in (0, 1), proc.stderr + proc.stdout
    result = json.loads((out / "result.json").read_text())
    assert result["form"] == "ros2"
    assert result["policy"] == "formation" and result["task"] == "formation"
    assert result["n_drones"] == 3 and result["seed"] == 0
    assert result["bus"]["delivered"] > 0
    assert FormationFleet.transport_override is None


@needs_ros
@pytest.mark.parametrize("faults", [
    pytest.param(BusFaults(drop=1.0), id="drop"),
    pytest.param(BusFaults(reorder=1.0), id="reorder"),
])
def test_ros2_formation_faults_match_inprocess(faults):
    from yopo_l0.ros2_run import execute

    card = load_card(FORMATION)
    cfg = RunConfig(seed=0, policy="formation", duration_s=2.0, bus=faults)
    proc = run(card, cfg)
    info: dict = {}
    ros = execute(card, cfg, inspect=info)
    assert _view(proc.result) == _view(ros.result)
    assert proc.ticks == ros.ticks
    assert info["bus_ros_deliveries"] == ros.result["bus"]["delivered"] == proc.result["bus"]["delivered"]
    assert proc.result["bus"]["dropped"] == ros.result["bus"]["dropped"]
    assert proc.result["bus"]["reordered"] == ros.result["bus"]["reordered"]
    if faults.drop == 1.0:
        assert info["bus_ros_deliveries"] == 0
        assert proc.result["bus"]["dropped"] == proc.result["bus"]["published"] > 0
    else:
        assert info["bus_ros_deliveries"] > 0
        assert proc.result["bus"]["reordered"] > 0
    assert FormationFleet.transport_override is None


@pytest.mark.parametrize("seed", [0, 1, 2])
@needs_ros
def test_ros2_formation_matches_inprocess_for_seed(seed: int):
    from yopo_l0.ros2_run import execute

    card = load_card(FORMATION)
    cfg = RunConfig(seed=seed, policy="formation")
    proc = run(card, cfg)
    info: dict = {}
    ros = execute(card, cfg, inspect=info)
    assert info["clock_ok"], info["clock_error"]
    assert info["nodes"] == ["/l0_world", "/uav0/drone", "/uav1/drone", "/uav2/drone"]
    assert _view(proc.result) == _view(ros.result)
    assert proc.result["outcome"] == "SUCCESS"
    assert proc.result["arrived"] == 3
    assert proc.result["outcome"] != "COLLISION"
    assert proc.result["formation_error_p95_m"] is not None
    assert proc.result["min_inter_drone_distance_m"] is not None
    assert proc.result["shadow_reject_rate"] is not None
    assert info["bus_ros_deliveries"] == ros.result["bus"]["delivered"] == proc.result["bus"]["delivered"] > 0
    assert proc.ticks == ros.ticks
    assert ros.result["form"] == "ros2"
    assert FormationFleet.transport_override is None
    _record("formation-hold", "formation", seed, proc.result, ros.result, info["nodes"], info["clock_ok"])
