"""一个 L0 世界节点 + 每机一个节点。

世界在每个 tick 传感之前把 L0 时钟发到 `/clock`（各节点 use_sim_time）。
规划请求走 `/{id}/observation`，批准计划的令牌走 `/{id}/plan`。
令牌指向本进程里的计划对象：`_TrajPlan.sample` 会改偏航，certified 的运行时也挂在这个对象上，
动力学必须 sample 策略交出来的那一个。他机不拿这个对象。

编队的机间信封不进进程内队列。世界把信封发到 `/{src}/bus_tx`，源节点转到 `/{dest}/bus_rx`，
目的节点再发 `/{dest}/bus_done`。锁步队列只接收走完这三跳的那一份。节点仍按 bind 的 id 建命名空间。
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from .bus import _Item
from .policy import Observation
from .ros2_codec import (
    check_drone_id,
    decode_bus_messages,
    decode_observation_messages,
    encode_bus_item,
    encode_observation,
    same_float,
    same_observation,
    sim_time_ns,
)
from .scene import SceneCard

# 与 rclpy TimeSource 订阅 /clock 的 QoS 一致，否则 use_sim_time 接不上。
_CLOCK_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
# 一条规划请求的 header + 分片必须能在开 spin 之前全部进队列。
_LOCKSTEP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=64,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
_USE_SIM = Parameter("use_sim_time", Parameter.Type.BOOL, True)


class _Slots:
    """本机计划对象。ROS 消息里只出现整数令牌。"""

    def __init__(self) -> None:
        self._items: list = []

    def put(self, plan) -> int:
        self._items.append(plan)
        return len(self._items) - 1

    def get(self, token: int):
        return self._items[int(token)]


class _Assembler:
    """把一个 seq 的 header 和深度分片收齐。乱序或半包直接报错，不悄悄规划。"""

    def __init__(self) -> None:
        self.seq: int | None = None
        self.messages: list[str] = []
        self.need = 0

    def push(self, text: str) -> list[str] | None:
        data = json.loads(text)
        if data.get("kind") == "header":
            self.seq = int(data["seq"])
            self.need = int(data["n_chunks"])
            self.messages = [text]
        else:
            if self.seq is None or int(data.get("seq", -1)) != self.seq:
                raise RuntimeError(f"深度分片 seq={data.get('seq')} 没有对应的 header {self.seq}")
            self.messages.append(text)
        if len(self.messages) == 1 + self.need:
            done = self.messages
            self.messages = []
            self.need = 0
            return done
        return None


class _BusAssembler:
    """一个 seq 的 bus header 加分片。乱序或半包直接报错。"""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.seq: int | None = None
        self.messages: list[str] = []
        self.need = 0
        self.dest: str | None = None

    def push(self, text: str) -> list[str] | None:
        data = json.loads(text)
        kind = data.get("kind")
        if kind == "bus":
            self.seq = int(data["seq"])
            self.need = int(data["n_chunks"])
            self.dest = data["dest"]
            self.messages = [text]
        elif kind == "bus_chunk":
            if self.seq is None or int(data.get("seq", -1)) != self.seq:
                raise RuntimeError(f"总线分片 seq={data.get('seq')} 没有对应的 header {self.seq}")
            if data.get("dest") != self.dest:
                raise RuntimeError(f"总线分片目的地 {data.get('dest')} 和 header {self.dest} 不一致")
            self.messages.append(text)
        else:
            raise RuntimeError(f"不是机间总线消息：{kind}")
        if len(self.messages) == 1 + self.need:
            done = self.messages
            self.reset()
            return done
        return None


class DroneNode(Node):
    """包一层已有策略。策略对象就是运行器建的那一个。"""

    def __init__(self, drone_id: str, policy, slots: _Slots, peers: list[str] | None = None):
        check_drone_id(drone_id)
        super().__init__(
            "drone",
            namespace=drone_id,
            parameter_overrides=[_USE_SIM],
            enable_rosout=False,
        )
        self.drone_id = drone_id
        self.policy = policy
        self.slots = slots
        self._asm = _Assembler()
        self.create_subscription(String, "observation", self._on_observation, _LOCKSTEP_QOS)
        self._plan_pub = self.create_publisher(String, "plan", _LOCKSTEP_QOS)
        self.bus_rx_pubs = {}
        self.bus_done_pub = None
        if peers is not None:
            # 含自己：dest==src 时也从本机 bus_rx 走一跳，不在回调里短路。
            self.create_subscription(String, "bus_tx", self._on_bus_tx, _LOCKSTEP_QOS)
            self.create_subscription(String, "bus_rx", self._on_bus_rx, _LOCKSTEP_QOS)
            self.bus_done_pub = self.create_publisher(String, "bus_done", _LOCKSTEP_QOS)
            for peer in peers:
                self.bus_rx_pubs[peer] = self.create_publisher(String, f"/{peer}/bus_rx", _LOCKSTEP_QOS)

    def _on_bus_tx(self, msg: String) -> None:
        dest = json.loads(msg.data)["dest"]
        pub = self.bus_rx_pubs.get(dest)
        if pub is None:
            raise RuntimeError(f"{self.drone_id} 没有发往 {dest} 的 bus_rx")
        pub.publish(String(data=msg.data))

    def _on_bus_rx(self, msg: String) -> None:
        if self.bus_done_pub is None:
            raise RuntimeError(f"{self.drone_id} 没有 bus_done")
        self.bus_done_pub.publish(String(data=msg.data))

    def _on_observation(self, msg: String) -> None:
        done = self._asm.push(msg.data)
        if done is None:
            return
        obs = decode_observation_messages(done)
        header = json.loads(done[0])
        if header["drone_id"] != self.drone_id:
            raise RuntimeError(f"{self.drone_id} 收到发给 {header['drone_id']} 的观测")
        plan = self.policy.plan(obs)
        token = self.slots.put(plan)
        self._plan_pub.publish(String(data=json.dumps({
            "v": 1,
            "seq": int(header["seq"]),
            "token": token,
            "digest": obs.digest(),
            "state_t": float(obs.state.t).hex(),
        }, separators=(",", ":"))))


class L0WorldBridge:
    """世界节点。`run(..., bridge=this)` 每 tick 调进来，由这里把锁步挡住，直到该机节点规划完。"""

    def __init__(self, card: SceneCard, *, plan_timeout_s: float = 180.0, transport=None):
        self.card = card
        self.phys = int(card.card["rates_hz"]["physics"])
        self.plan_timeout_s = float(plan_timeout_s)
        self.transport = transport
        self.clock_ok = True
        self.clock_error = ""
        self.clock_samples = 0
        self.nodes_full_names: list[str] = []
        self.bus_ros_deliveries = 0
        self.bus_topics: list[str] = []
        self.last_now: float | None = None
        self._clock_misses = 0
        self._clock_broken = False
        self._closed = False
        self._executor: SingleThreadedExecutor | None = None
        self._owned: list[Node] = []
        self.world: Node | None = None
        self.drone_nodes: list[DroneNode] = []
        self._slots = _Slots()
        self._obs_pubs: dict = {}
        self._plan_subs: dict = {}
        self._tx_pubs: dict = {}
        self._done_subs: dict = {}
        self._bus_asm = _BusAssembler()
        self._bus_seen = 0
        self._bus_done_item: dict | None = None
        self._clock_pub = None
        self._waiting: tuple[str, int] | None = None
        self._token: int | None = None
        self._expect_digest = ""
        self._expect_state_t = ""
        self._seq = 0

    def bind(self, drones: list[dict]) -> None:
        ids = [d["id"] for d in drones]
        if len(ids) != len(set(ids)):
            raise ValueError(f"无人机 id 重复：{ids}")
        for drone_id in ids:
            check_drone_id(drone_id)
        self.world = Node(
            "l0_world",
            parameter_overrides=[_USE_SIM],
            enable_rosout=False,
        )
        self._owned.append(self.world)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.world)
        self._clock_pub = self.world.create_publisher(Clock, "/clock", _CLOCK_QOS)
        peers = ids if self.transport is not None else None
        if self.transport is not None:
            for drone_id in ids:
                self._tx_pubs[drone_id] = self.world.create_publisher(
                    String, f"/{drone_id}/bus_tx", _LOCKSTEP_QOS,
                )
                self._done_subs[drone_id] = self.world.create_subscription(
                    String, f"/{drone_id}/bus_done",
                    lambda msg, drone_id=drone_id: self._on_bus_done(drone_id, msg),
                    _LOCKSTEP_QOS,
                )
                self.bus_topics += [
                    f"/{drone_id}/bus_tx", f"/{drone_id}/bus_rx", f"/{drone_id}/bus_done",
                ]
            self.transport.relay = self.relay_item
            self.transport.topics = list(self.bus_topics)
        for dr in drones:
            node = DroneNode(dr["id"], dr["policy"], self._slots, peers=peers)
            self.drone_nodes.append(node)
            self._owned.append(node)
            self._executor.add_node(node)
            drone_id = dr["id"]
            self._obs_pubs[drone_id] = self.world.create_publisher(
                String, f"/{drone_id}/observation", _LOCKSTEP_QOS,
            )
            self._plan_subs[drone_id] = self.world.create_subscription(
                String, f"/{drone_id}/plan",
                lambda msg, drone_id=drone_id: self._on_plan(drone_id, msg),
                _LOCKSTEP_QOS,
            )
        self._wait_discovery()
        self._require_sim_clock()
        self.nodes_full_names = [self.world.get_fully_qualified_name()] + [
            n.get_fully_qualified_name() for n in self.drone_nodes
        ]

    def begin_tick(self, k: int, now: float) -> None:
        """传感之前。发布的是 tick 起点，和运行器里的 now = k / physics 是同一步。"""
        self.last_now = now
        total = self._publish_clock(k)
        self.clock_samples += 1
        if total == 0 or self._clock_broken:
            assert self._executor is not None
            self._executor.spin_once(timeout_sec=0)
            return
        if not self._wait_clock(total):
            self.clock_ok = False
            self._clock_misses += 1
            seen = [int(n.get_clock().now().nanoseconds) for n in self.drone_nodes]
            self.clock_error = f"k={k} 期望 /clock {total} ns，各机时钟 {seen}"
            if self._clock_misses >= 5:
                self._clock_broken = True

    def plan(self, drone_id: str, obs: Observation):
        self._seq += 1
        seq = self._seq
        messages = encode_observation(obs, drone_id=drone_id, seq=seq)
        wired = decode_observation_messages(messages)
        if not same_observation(wired, obs):
            raise RuntimeError(f"{drone_id} 的观测编码不是逐位往返")
        self._waiting = (drone_id, seq)
        self._token = None
        self._expect_digest = obs.digest()
        self._expect_state_t = float(obs.state.t).hex()
        pub = self._obs_pubs[drone_id]
        for text in messages:
            pub.publish(String(data=text))
        deadline = time.monotonic() + self.plan_timeout_s
        assert self._executor is not None
        while self._token is None:
            self._executor.spin_once(timeout_sec=0.05)
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"{drone_id} 在 {self.plan_timeout_s:.0f}s 内没有发回批准计划"
                    f"（seq={seq}，观测 {len(messages)} 条）"
                )
        return self._slots.get(self._token)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        exe = self._executor
        self._executor = None
        if exe is not None:
            exe.shutdown()
        for node in self._owned:
            node.destroy_node()
        self._owned.clear()

    def _publish_clock(self, k: int) -> int:
        total = sim_time_ns(k, self.phys)
        self._publish_ns(total)
        return total

    def _clocks_match(self, total: int) -> bool:
        return all(int(n.get_clock().now().nanoseconds) == total for n in self.drone_nodes)

    def _wait_clock(self, total: int) -> bool:
        """同一时刻可能要分给世界和每一架飞机，spin 一次只派一个回调。"""
        assert self._executor is not None
        readers = len(self.drone_nodes) + 2
        for i in range(readers * 4):
            if self._clocks_match(total):
                return True
            if i and i % readers == 0:
                self._publish_ns(total)
            self._executor.spin_once(timeout_sec=0)
        self._publish_ns(total)
        self._executor.spin_once(timeout_sec=0.02)
        return self._clocks_match(total)

    def _publish_ns(self, total: int) -> None:
        msg = Clock()
        msg.clock.sec = int(total // 1_000_000_000)
        msg.clock.nanosec = int(total % 1_000_000_000)
        self._clock_pub.publish(msg)

    def _wait_discovery(self) -> None:
        assert self._executor is not None and self._clock_pub is not None
        expect_clock = len(self.drone_nodes) + 1  # 每机 + 世界自己都订了 /clock
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            self._executor.spin_once(timeout_sec=0.05)
            clock_n = self._clock_pub.get_subscription_count()
            obs_ok = all(p.get_subscription_count() >= 1 for p in self._obs_pubs.values())
            plan_ok = all(s.get_publisher_count() >= 1 for s in self._plan_subs.values())
            if clock_n >= expect_clock and obs_ok and plan_ok and self._bus_matched():
                return
        raise TimeoutError(
            "ROS 2 发现没有完成："
            f" /clock 订阅 {self._clock_pub.get_subscription_count()}/{expect_clock}，"
            f"观测订阅 {[p.get_subscription_count() for p in self._obs_pubs.values()]}，"
            f"计划发布 {[s.get_publisher_count() for s in self._plan_subs.values()]}。"
            f"{self._bus_discovery_text()}"
            "若本机还有别的 ROS 图，换一个 ROS_DOMAIN_ID。"
        )

    def _bus_matched(self) -> bool:
        if self.transport is None:
            return True
        if not self._tx_pubs:
            return False
        tx_ok = all(p.get_subscription_count() >= 1 for p in self._tx_pubs.values())
        rx_ok = all(
            pub.get_subscription_count() >= 1
            for node in self.drone_nodes
            for pub in node.bus_rx_pubs.values()
        )
        done_ok = all(
            node.bus_done_pub is not None and node.bus_done_pub.get_subscription_count() >= 1
            for node in self.drone_nodes
        )
        return tx_ok and rx_ok and done_ok

    def _bus_discovery_text(self) -> str:
        if self.transport is None:
            return ""
        tx = [p.get_subscription_count() for p in self._tx_pubs.values()]
        rx = [pub.get_subscription_count() for node in self.drone_nodes for pub in node.bus_rx_pubs.values()]
        done = [
            0 if node.bus_done_pub is None else node.bus_done_pub.get_subscription_count()
            for node in self.drone_nodes
        ]
        return f" bus_tx 订阅 {tx}，bus_rx 订阅 {rx}，bus_done 订阅 {done}。"

    def relay_item(self, item) -> _Item:
        """一条没被丢掉的拷贝。分片按序走完三跳才返回，返回值只含话题上回来的字段。"""
        messages = encode_bus_item(item)
        self._bus_asm.reset()
        self._bus_seen = 0
        self._bus_done_item = None
        pub = self._tx_pubs[item.src]
        for index, text in enumerate(messages):
            pub.publish(String(data=text))
            self._wait_bus_seen(index + 1, item)
        got = self._bus_done_item
        if got is None:
            raise RuntimeError(f"机间消息 seq={item.seq} 分片齐了但没有解出信封")
        self._bus_done_item = None
        for key in ("seq", "src", "dest", "topic", "rank"):
            if got[key] != getattr(item, key):
                raise RuntimeError(
                    f"话题往返字段 {key}：发出 {getattr(item, key)!r}，收到 {got[key]!r}"
                )
        if not same_float(got["sent_at"], item.sent_at) or not same_float(got["deliver_at"], item.deliver_at):
            raise RuntimeError(
                f"总线时间戳经话题后不是逐位：sent {item.sent_at!r}->{got['sent_at']!r}，"
                f"deliver {item.deliver_at!r}->{got['deliver_at']!r}"
            )
        self.bus_ros_deliveries += 1
        return _Item(
            deliver_at=got["deliver_at"],
            rank=int(got["rank"]),
            seq=int(got["seq"]),
            src=got["src"],
            dest=got["dest"],
            topic=got["topic"],
            payload=got["payload"],
            sent_at=got["sent_at"],
        )

    def _wait_bus_seen(self, target: int, item) -> None:
        assert self._executor is not None
        deadline = time.monotonic() + min(self.plan_timeout_s, 30.0)
        spins = 0
        while self._bus_seen < target:
            timeout = 0.0 if spins < 30 else 0.02
            self._executor.spin_once(timeout_sec=timeout)
            spins += 1
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"机间消息 seq={item.seq} {item.src}->{item.dest} 在 30s 内没有经 "
                    f"/{item.src}/bus_tx → /{item.dest}/bus_rx → /{item.dest}/bus_done 回到世界"
                    f"（已回到世界 {self._bus_seen} 片，正在等第 {target} 片）"
                )

    def _on_bus_done(self, drone_id: str, msg: String) -> None:
        self._bus_seen += 1
        done = self._bus_asm.push(msg.data)
        if done is None:
            return
        decoded = decode_bus_messages(done)
        if decoded["dest"] != drone_id:
            raise RuntimeError(f"/{drone_id}/bus_done 收到目的地是 {decoded['dest']} 的信封")
        self._bus_done_item = decoded

    def _require_sim_clock(self) -> None:
        from rclpy.clock_type import ClockType

        for node in [self.world, *self.drone_nodes]:
            if node.get_clock().clock_type != ClockType.ROS_TIME:
                raise RuntimeError(f"{node.get_fully_qualified_name()} 的 use_sim_time 没有生效")

    def _on_plan(self, drone_id: str, msg: String) -> None:
        data = json.loads(msg.data)
        got = (drone_id, int(data["seq"]))
        if self._waiting != got:
            raise RuntimeError(f"锁步被打乱：收到 {got}，正在等 {self._waiting}")
        if data["digest"] != self._expect_digest or data["state_t"] != self._expect_state_t:
            raise RuntimeError(f"{drone_id} 规划用的观测和世界发出的不一致")
        self._token = int(data["token"])


def init_rclpy(rclpy_mod=None) -> None:
    mod = rclpy_mod or rclpy
    if mod.ok():
        return
    from rclpy.signals import SignalHandlerOptions

    mod.init(signal_handler_options=SignalHandlerOptions.NO)
