"""票 S4 / S4b：YOPO L0 与 PX4 Offboard 之间的 ENU/NED 闭环桥。

规划核心保持 ENU。PX4 ``VehicleOdometry`` / ``TrajectorySetpoint`` 是 NED。
两边的向量和航向只经过 ``yopo_swarm_core.px4``（``px4.py`` 模块头注明的 ADR-0011 边界）。
本文件不写第二套轴映射，也不导入 ``rclpy``。

离线闭环用 ``FakePx4Vehicle``：在 NED 里按设定点做点质量 + 一阶加速度滞后，
增益与 ``PointMassLag`` 相同。票 S4b 把同一份桥接接到 ROS 消息字段：
``VehicleOdometry``、深度图或障碍摘要进 ``Observation``，``TrajectorySetpoint`` 出去。
容器里的订阅和发布在 ``scripts/sitl/yopo_l0_live.py``，见 ``docs/research/s4b-sitl-live.md``。
"""

from __future__ import annotations

import json
import math
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from yopo_swarm_core import px4

from .dynamics import State
from .inter_drone import samples_from_pva
from .policy import Observation, PolicyContext, UpstreamPolicy
from .scene import SceneCard
from .sensor import R_BODY_CAM, CameraConfig, DepthCamera, DepthFrame

SCHEMA = "yopo-restart/s4-sitl-bridge/1"
SCHEMA_LIVE = "yopo-restart/s4b-sitl-live/1"
# ``--partial`` 只跑这么长的前缀，用来做离线校验，不等到达。
PARTIAL_PREFIX_S = 0.5
POSITIONING_MODES = ("default", "rtk")
VEHICLE_CMD_NAV_LAND = 21  # MAV_CMD_NAV_LAND，与 offboard_mission 同一条命令号
GCS_PORT_BASE = 18570
_REPO = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class SitlBridgeConfig:
    """一架飞机的桥接参数。``physics_dt`` 是设定点周期，``planner_dt`` 是重规划周期。"""

    physics_dt: float = 0.02
    planner_dt: float = 0.1
    hover_alt_m: float = 1.5
    goal_enu: tuple[float, float, float] | None = None
    policy_params: dict = field(default_factory=dict)
    drone_id: str = "uav0"
    seed: int = 0
    drone_index: int = 0
    n_drones: int = 1
    a_max: float = 6.0
    tau_s: float = 0.1
    kp: float = 4.0
    kd: float = 3.0
    px4_namespace: str = ""

    def __post_init__(self) -> None:
        if not self.physics_dt > 0.0 or not self.planner_dt > 0.0:
            raise ValueError(f"physics_dt 和 planner_dt 必须为正，收到 {self.physics_dt}, {self.planner_dt}")
        if self.n_drones < 1 or not 0 <= self.drone_index < self.n_drones:
            raise ValueError(f"drone_index={self.drone_index} 不在 0..{self.n_drones - 1}")
        if self.goal_enu is not None and len(self.goal_enu) != 3:
            raise ValueError(f"goal_enu 需要 3 个数，收到 {self.goal_enu!r}")


@dataclass(frozen=True)
class NeighborEnu:
    """邻机扩展口：世界系 ENU 位置/速度，以及可选的已提交轨迹采样（t, x, y, z）。"""

    drone_id: str
    position: np.ndarray
    velocity: np.ndarray
    samples: list | np.ndarray | None = None


@dataclass(frozen=True)
class BridgeOutput:
    """一次 ``command`` 的 ENU 期望和已经转成 NED 的 PX4 字段。"""

    now_s: float
    p_enu: np.ndarray
    v_enu: np.ndarray
    a_enu: np.ndarray
    yaw_enu: float
    setpoint: dict
    offboard: dict
    planned: bool
    plan_code: str
    roundtrip_error_m: float
    roundtrip_velocity_mps: float
    roundtrip_acceleration_mps2: float


@dataclass(frozen=True)
class OffboardCommand:
    """填进 px4_msgs 的字段。容器侧的 publisher 不在这个仓库模块里。"""

    trajectory_setpoint: dict
    offboard_control_mode: dict
    namespace: str = ""


@dataclass(frozen=True)
class FakePx4Config:
    """与 ``PointMassLag`` 相同的增益，积分在 NED 里做。"""

    tau_s: float = 0.1
    kp: float = 4.0
    kd: float = 3.0
    a_max: float = 6.0


def _as_vec3(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} 必须是有限的 3 维向量，收到 shape={arr.shape}")
    return arr


def roundtrip_abs(enu: np.ndarray) -> float:
    """``ned_to_enu(enu_to_ned(v))`` 与 ``v`` 的最大绝对分量差。"""
    src = np.asarray(enu, dtype=float)
    back = px4.ned_to_enu(px4.enu_to_ned(src))
    return float(np.max(np.abs(back - src)))


def enu_state_from_ned(
    position_ned,
    velocity_ned,
    now_s: float,
    *,
    q_ned_frd=None,
    yaw_ned: float | None = None,
    acceleration_ned=None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """VehicleOdometry 口径的 NED 位置/速度 → ENU 位置、速度、加速度和航向。

    给了四元数就走 ``px4.odometry_to_enu``（姿态里的航向）。否则用 ``yaw_ned``。
    两者都缺则拒绝，不猜朝向。
    """
    position_ned = _as_vec3(position_ned, "position_ned")
    velocity_ned = _as_vec3(velocity_ned, "velocity_ned")
    if q_ned_frd is not None:
        enu = px4.odometry_to_enu(
            position_ned,
            q_ned_frd,
            velocity_ned,
            int(round(float(now_s) * 1e6)),
        )
        p = np.asarray(enu.position, dtype=float)
        v = np.asarray(enu.velocity, dtype=float)
        yaw = math.atan2(float(enu.R_wb[1, 0]), float(enu.R_wb[0, 0]))
    elif yaw_ned is not None:
        if not math.isfinite(float(yaw_ned)):
            raise ValueError("yaw_ned 不是有限值")
        p = np.asarray(px4.ned_to_enu(position_ned), dtype=float)
        v = np.asarray(px4.ned_to_enu(velocity_ned), dtype=float)
        yaw = float(px4.yaw_ned_to_enu(float(yaw_ned)))
    else:
        raise ValueError("需要 q_ned_frd 或 yaw_ned")
    if acceleration_ned is None:
        a = np.zeros(3)
    else:
        a = np.asarray(px4.ned_to_enu(_as_vec3(acceleration_ned, "acceleration_ned")), dtype=float)
    return p, v, a, yaw


def trajectory_setpoint_ned(p_enu, v_enu, a_enu, yaw_enu: float, now_s: float) -> dict:
    """ENU 的 (p, v, a, yaw) → ``px4.trajectory_setpoint`` 字段（NED）。"""
    pva = np.stack([_as_vec3(p_enu, "p_enu"), _as_vec3(v_enu, "v_enu"), _as_vec3(a_enu, "a_enu")])
    return px4.trajectory_setpoint(pva, float(yaw_enu), int(round(float(now_s) * 1e6)))


def offboard_command(out: BridgeOutput, namespace: str = "") -> OffboardCommand:
    return OffboardCommand(out.setpoint, out.offboard, namespace)


def _yaw_ned_quaternion(yaw_ned: float) -> np.ndarray:
    """水平机体、NED 航向 ``yaw_ned`` 的 FRD→NED 四元数。旋转是在 NED 里构造的。"""
    c, s = math.cos(yaw_ned), math.sin(yaw_ned)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return px4.rot_to_quat(rot)


def _camera_rng(seed: int, index: int, n: int) -> np.random.Generator:
    """与 ``runner.run`` 一样：``SeedSequence(seed).spawn(n)[index]``。"""
    child = np.random.SeedSequence(int(seed)).spawn(int(n))[int(index)]
    return np.random.default_rng(child)


def _git_commit() -> str:
    try:
        sha = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=_REPO, capture_output=True, text=True, check=True
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=no"],
            cwd=_REPO,
            capture_output=True,
            text=True,
        ).stdout.strip()
        return sha + ("-dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


class YopoSitlBridge:
    """一架飞机：NED 里程计进来，NED ``TrajectorySetpoint`` 出去。

    ``command`` 每个 ``physics_dt`` 调一次。到了 ``planner_dt`` 才渲染深度并
    ``UpstreamPolicy.plan``；两次规划之间只对当前 ``_TrajPlan`` 采样。
    同一个 ``now_s`` 再调一次不重复采样（``_TrajPlan.sample`` 会推进偏航）。
    """

    def __init__(self, card: SceneCard, cfg: SitlBridgeConfig | None = None):
        self.cfg = cfg or SitlBridgeConfig()
        self.card = card
        drones = [d for d in card.card["drones"] if d["id"] == self.cfg.drone_id]
        if len(drones) != 1:
            raise ValueError(f"场景 {card.name} 里没有唯一的 {self.cfg.drone_id}")
        drone = drones[0]
        if self.cfg.goal_enu is not None:
            goal = np.asarray(self.cfg.goal_enu, dtype=float)
        else:
            raw = drone.get("goal_enu", drone.get("hover_enu", drone["start_enu"]))
            goal = np.asarray(raw, dtype=float)
        if goal.shape != (3,):
            raise ValueError(f"{self.cfg.drone_id} 的目标不是 3 维")
        self.goal = goal.copy()
        self.hover_alt_m = float(self.cfg.hover_alt_m)
        self.drone_id = self.cfg.drone_id
        self.start_enu = np.asarray(drone["start_enu"], dtype=float).copy()
        cam = CameraConfig.from_card(card.card["camera"])
        rng = _camera_rng(self.cfg.seed, self.cfg.drone_index, self.cfg.n_drones)
        self.camera = DepthCamera(cam, card.obstacles, rng)
        ctx = PolicyContext(
            self.drone_id,
            float(self.cfg.physics_dt),
            dict(self.cfg.policy_params),
            obstacles=card.obstacles,
            a_max=float(self.cfg.a_max),
        )
        self.policy = UpstreamPolicy(ctx)
        self.neighbors: tuple[NeighborEnu, ...] = ()
        self.neighbor_sample_deliveries = 0
        self._plan = None
        self._broadcast_samples: list | None = None
        self._last_plan_s: float | None = None
        self._last_sample_s: float | None = None
        self._last_output: BridgeOutput | None = None
        self.last_observation: Observation | None = None
        self.plan_counts: dict[str, int] = {}

    @property
    def plan(self):
        """当前 ``UpstreamPolicy`` 交回的 ``_TrajPlan``（还没规划时是 None）。"""
        return self._plan

    def set_neighbors(self, neighbors: tuple[NeighborEnu, ...] | list[NeighborEnu]) -> None:
        """记下邻机 ENU 状态。采样在下一次 ``plan`` 之前交给 ``note_neighbor_samples``。"""
        self.neighbors = tuple(neighbors)

    def broadcast_samples(self) -> list | None:
        """最近一次规划广播的 ENU 采样。没有 ``pva`` 的计划是 None。"""
        return self._broadcast_samples

    def _deliver_neighbors(self) -> None:
        note = getattr(self.policy, "note_neighbor_samples", None)
        if note is None:
            return
        for nb in self.neighbors:
            if nb.samples is None:
                continue
            note(nb.drone_id, nb.samples)
            self.neighbor_sample_deliveries += 1

    def _due(self, now_s: float) -> bool:
        if self._plan is None or self._last_plan_s is None:
            return True
        return now_s - self._last_plan_s + 1e-8 >= self.cfg.planner_dt

    def command(
        self,
        position_ned,
        velocity_ned,
        now_s: float,
        *,
        q_ned_frd=None,
        yaw_ned: float | None = None,
        acceleration_ned=None,
        depth_frame: DepthFrame | None = None,
    ) -> BridgeOutput:
        now_s = float(now_s)
        if self._last_output is not None and self._last_sample_s is not None:
            if abs(now_s - self._last_sample_s) <= 1e-12:
                return self._last_output
        if depth_frame is not None and not isinstance(depth_frame, DepthFrame):
            raise TypeError(f"depth_frame 必须是 DepthFrame，收到 {type(depth_frame).__name__}")
        p, v, a_state, yaw = enu_state_from_ned(
            position_ned,
            velocity_ned,
            now_s,
            q_ned_frd=q_ned_frd,
            yaw_ned=yaw_ned,
            acceleration_ned=acceleration_ned,
        )
        planned = False
        if self._due(now_s):
            self._deliver_neighbors()
            frame = self.camera.capture(p, yaw, now_s) if depth_frame is None else depth_frame
            state = State(now_s, p.copy(), v.copy(), a_state.copy(), float(yaw))
            obs = Observation(frame, state, self.goal.copy(), now_s)
            self.last_observation = obs
            self._plan = self.policy.plan(obs)
            code = str(self.policy.last_code)
            self.plan_counts[code] = self.plan_counts.get(code, 0) + 1
            self._broadcast_samples = samples_from_pva(self._plan, now_s)
            self._last_plan_s = now_s
            planned = True
        p_ref, v_ref, a_ref, yaw_ref = self._plan.sample(now_s)
        p_ref = np.asarray(p_ref, dtype=float).copy()
        v_ref = np.asarray(v_ref, dtype=float).copy()
        a_ref = np.asarray(a_ref, dtype=float).copy()
        yaw_ref = float(yaw_ref)
        setpoint = trajectory_setpoint_ned(p_ref, v_ref, a_ref, yaw_ref, now_s)
        offboard = px4.offboard_control_mode(setpoint["timestamp"])
        out = BridgeOutput(
            now_s=now_s,
            p_enu=p_ref,
            v_enu=v_ref,
            a_enu=a_ref,
            yaw_enu=yaw_ref,
            setpoint=setpoint,
            offboard=offboard,
            planned=planned,
            plan_code=str(self.policy.last_code),
            roundtrip_error_m=roundtrip_abs(p_ref),
            roundtrip_velocity_mps=roundtrip_abs(v_ref),
            roundtrip_acceleration_mps2=roundtrip_abs(a_ref),
        )
        self._last_sample_s = now_s
        self._last_output = out
        return out


class FakePx4Vehicle:
    """按 NED 设定点推进的点质量。加速度一阶滞后，速度、位置用新加速度积分。

    公式与 ``PointMassLag.step`` 相同，坐标已经是设定点所在的 NED，不再做一次轴映射。
    """

    def __init__(self, position_enu, velocity_enu=None, yaw_enu: float = 0.0, cfg: FakePx4Config | None = None):
        self.cfg = cfg or FakePx4Config()
        if not self.cfg.tau_s > 0.0 or not self.cfg.a_max > 0.0:
            raise ValueError("tau_s 和 a_max 必须为正")
        self.p_ned = np.asarray(px4.enu_to_ned(_as_vec3(position_enu, "position_enu")), dtype=float).copy()
        v0 = np.zeros(3) if velocity_enu is None else _as_vec3(velocity_enu, "velocity_enu")
        self.v_ned = np.asarray(px4.enu_to_ned(v0), dtype=float).copy()
        self.a_ned = np.zeros(3)
        if not math.isfinite(float(yaw_enu)):
            raise ValueError("yaw_enu 不是有限值")
        self.yaw_ned = float(px4.yaw_enu_to_ned(float(yaw_enu)))

    def quaternion(self) -> np.ndarray:
        return _yaw_ned_quaternion(self.yaw_ned)

    def odometry(self) -> dict:
        q = self.quaternion()
        return {
            "position_ned": self.p_ned.copy(),
            "velocity_ned": self.v_ned.copy(),
            "acceleration_ned": self.a_ned.copy(),
            "yaw_ned": float(self.yaw_ned),
            "q_ned_frd": q.copy(),
        }

    def step(self, setpoint: dict, dt: float) -> None:
        if not dt > 0.0:
            raise ValueError(f"dt 必须为正，收到 {dt}")
        p_sp = _as_vec3(setpoint["position"], "setpoint.position")
        v_sp = _as_vec3(setpoint["velocity"], "setpoint.velocity")
        a_sp = _as_vec3(setpoint["acceleration"], "setpoint.acceleration")
        a_cmd = a_sp + self.cfg.kp * (p_sp - self.p_ned) + self.cfg.kd * (v_sp - self.v_ned)
        n = float(np.linalg.norm(a_cmd))
        if n > self.cfg.a_max:
            a_cmd = a_cmd * (self.cfg.a_max / n)
        alpha = 1.0 - math.exp(-dt / self.cfg.tau_s)
        self.a_ned = self.a_ned + alpha * (a_cmd - self.a_ned)
        self.v_ned = self.v_ned + self.a_ned * dt
        self.p_ned = self.p_ned + self.v_ned * dt
        yaw = setpoint.get("yaw")
        if yaw is not None and math.isfinite(float(yaw)):
            self.yaw_ned = float(yaw)


def _vec_list(value) -> list[float]:
    return [float(x) for x in np.asarray(value, dtype=float).reshape(-1)]


def _hz(dt: float) -> float:
    hz = 1.0 / float(dt)
    rounded = round(hz)
    if abs(hz - rounded) <= 1e-6:
        return float(rounded)
    return hz


def run_closed_loop(
    card: SceneCard,
    *,
    seed: int = 0,
    max_s: float = 15.0,
    partial: bool = False,
    partial_s: float = PARTIAL_PREFIX_S,
    physics_dt: float = 0.02,
    planner_dt: float = 0.1,
    ros_messages: bool = False,
) -> dict:
    """场景卡上的全体飞机：假 PX4 NED 闭环。多机在每次规划前交换上一拍的采样。

    ``partial`` 把时域收成 ``min(max_s, partial_s)``。碰撞或全体按场景判据到达并保持后提前停。
    本结果始终 ``report_only``，不采用场景卡上的冻结门。
    """
    if not max_s > 0.0:
        raise ValueError(f"max_s 必须为正，收到 {max_s}")
    if partial and not partial_s > 0.0:
        raise ValueError(f"partial_s 必须为正，收到 {partial_s}")
    horizon = min(float(max_s), float(partial_s)) if partial else float(max_s)
    task = card.card.get("task") or {}
    if not partial and "duration_s" in task:
        horizon = min(horizon, float(task["duration_s"]))
    dyn = card.card.get("dynamics") or {}
    crit = card.card.get("criteria") or {}
    body = float(crit.get("body_radius_m", 0.35))
    track_goal = "goal_tolerance_m" in crit
    if track_goal:
        gtol = float(crit["goal_tolerance_m"])
        gspeed = float(crit["goal_speed_mps"])
        ghold = float(crit["goal_hold_s"])
    else:
        gtol = float(crit.get("hover_tolerance_m", 0.3))
        gspeed = math.inf
        ghold = 0.0
    drones = list(card.card["drones"])
    n = len(drones)
    params = dict(card.card.get("policy_params") or {})
    a_max = float(dyn.get("a_max", 6.0))
    tau_s = float(dyn.get("tau_s", 0.1))
    kp = float(dyn.get("kp", 4.0))
    kd = float(dyn.get("kd", 3.0))
    bridges: list[YopoSitlBridge] = []
    vehicles: list[FakePx4Vehicle] = []
    for i, drone in enumerate(drones):
        start = drone["start_enu"]
        cfg = SitlBridgeConfig(
            physics_dt=physics_dt,
            planner_dt=planner_dt,
            hover_alt_m=float(start[2]),
            goal_enu=tuple(drone.get("goal_enu", drone.get("hover_enu", start))),
            policy_params=params,
            drone_id=drone["id"],
            seed=int(seed),
            drone_index=i,
            n_drones=n,
            a_max=a_max,
            tau_s=tau_s,
            kp=kp,
            kd=kd,
            px4_namespace="" if n == 1 else f"px4_{i + 1}",
        )
        bridges.append(YopoSitlBridge(card, cfg))
        vehicles.append(
            FakePx4Vehicle(
                start, yaw_enu=0.0, cfg=FakePx4Config(tau_s=tau_s, kp=kp, kd=kd, a_max=a_max)
            )
        )

    wall0 = time.perf_counter()
    n_steps = int(round(horizon / physics_dt))
    rt_p = 0.0
    rt_v = 0.0
    rt_a = 0.0
    min_clear = math.inf
    min_obs = math.inf
    min_inter: float | None = None
    outcome = None
    reason = ""
    goal_since = [None] * n
    arrival = [None] * n
    final_p = [bridges[i].start_enu.copy() for i in range(n)]
    final_err = [float(np.linalg.norm(bridges[i].start_enu - bridges[i].goal)) for i in range(n)]
    ran = False
    k_last = -1
    start_positions = []
    for bridge in bridges:
        dist0 = float(card.min_distance(bridge.start_enu))
        min_obs = min(min_obs, dist0)
        min_clear = min(min_clear, dist0 - body)
        start_positions.append(np.asarray(bridge.start_enu, dtype=float))

    def _note_inter(positions: list[np.ndarray]) -> None:
        nonlocal min_inter, outcome, reason
        if n < 2:
            return
        for i in range(n):
            for j in range(i + 1, n):
                dist = float(np.linalg.norm(positions[i] - positions[j]))
                min_inter = dist if min_inter is None else min(min_inter, dist)
                if outcome is None and dist <= 2.0 * body:
                    outcome, reason = "COLLISION", f"{drones[i]['id']}-{drones[j]['id']}:INTER_DRONE"

    _note_inter(start_positions)
    for k in range(n_steps):
        if outcome is not None:
            break
        ran = True
        k_last = k
        now = round(k * physics_dt, 10)
        for i, bridge in enumerate(bridges):
            others = []
            for j, other in enumerate(bridges):
                if j == i:
                    continue
                odom_j = vehicles[j].odometry()
                if ros_messages:
                    decoded_j = decode_vehicle_odometry(
                        odometry_message_from_odometry(odom_j, int(round(now * 1e6))), now_s=now
                    )
                    pj, vj = decoded_j["position_enu"], decoded_j["velocity_enu"]
                else:
                    pj, vj, _, _ = enu_state_from_ned(
                        odom_j["position_ned"],
                        odom_j["velocity_ned"],
                        now,
                        q_ned_frd=odom_j["q_ned_frd"],
                    )
                others.append(NeighborEnu(other.drone_id, pj, vj, other.broadcast_samples()))
            bridge.set_neighbors(others)
        for i, bridge in enumerate(bridges):
            odom = vehicles[i].odometry()
            depth_frame = None
            position_ned = odom["position_ned"]
            velocity_ned = odom["velocity_ned"]
            q_ned = odom["q_ned_frd"]
            yaw_ned = odom["yaw_ned"]
            acceleration_ned = odom["acceleration_ned"]
            if ros_messages:
                decoded_i = decode_vehicle_odometry(
                    odometry_message_from_odometry(odom, int(round(now * 1e6))), now_s=now
                )
                position_ned = decoded_i["position_ned"]
                velocity_ned = decoded_i["velocity_ned"]
                q_ned = decoded_i["q_ned_frd"]
                acceleration_ned = decoded_i["acceleration_ned"]
                yaw_ned = None
                # 只有到了重规划拍才渲染。否则会多抽一次相机噪声，和直接 command 的随机序列错开。
                if bridge._due(now):
                    preview = bridge.camera.capture(decoded_i["position_enu"], decoded_i["yaw_enu"], now)
                    depth_frame = decode_depth_image(encode_depth_image(preview))
            out = bridge.command(
                position_ned,
                velocity_ned,
                now,
                q_ned_frd=q_ned,
                yaw_ned=yaw_ned,
                acceleration_ned=acceleration_ned,
                depth_frame=depth_frame,
            )
            vehicles[i].step(out.setpoint, physics_dt)
            odom_enu = px4.ned_to_enu(odom["position_ned"])
            rt_p = max(rt_p, out.roundtrip_error_m, roundtrip_abs(odom_enu))
            rt_v = max(rt_v, out.roundtrip_velocity_mps)
            rt_a = max(rt_a, out.roundtrip_acceleration_mps2)
        t_state = round((k + 1) * physics_dt, 10)
        positions = []
        for i, vehicle in enumerate(vehicles):
            odom = vehicle.odometry()
            p_enu, v_enu, _, _ = enu_state_from_ned(
                odom["position_ned"], odom["velocity_ned"], t_state, q_ned_frd=odom["q_ned_frd"]
            )
            positions.append(p_enu)
            dist = float(card.min_distance(p_enu))
            clearance = dist - body
            min_obs = min(min_obs, dist)
            min_clear = min(min_clear, clearance)
            err = float(np.linalg.norm(p_enu - bridges[i].goal))
            final_p[i] = p_enu
            final_err[i] = err
            if outcome is None and dist <= body:
                outcome, reason = "COLLISION", f"{bridges[i].drone_id}:OBSTACLE"
            elif outcome is None and arrival[i] is None:
                speed = float(np.linalg.norm(v_enu))
                if track_goal:
                    if err <= gtol and speed <= gspeed:
                        goal_since[i] = t_state if goal_since[i] is None else goal_since[i]
                        if t_state - goal_since[i] >= ghold - 1e-9:
                            arrival[i] = goal_since[i]
                    else:
                        goal_since[i] = None
                elif err <= gtol:
                    arrival[i] = t_state
        _note_inter(positions)
        if outcome is not None:
            break
        if all(t is not None for t in arrival):
            outcome, reason = "SUCCESS", "GOAL_REACHED_AND_HELD"
            break

    if outcome is None and track_goal:
        outcome, reason = ("PARTIAL", "PREFIX") if partial else ("TIMEOUT", "GOAL_NOT_REACHED")
    if outcome is None:
        worst = max(final_err) if final_err else 0.0
        outcome, reason = ("SUCCESS", "HOVER_HELD") if worst <= gtol else ("HOVER_DRIFT", "HOVER_TOLERANCE")

    arrived_n = sum(t is not None for t in arrival)
    drone_rows = []
    for i, bridge in enumerate(bridges):
        drone_rows.append(
            {
                "id": bridge.drone_id,
                "px4_namespace": bridge.cfg.px4_namespace,
                "start_enu": _vec_list(bridge.start_enu),
                "goal_enu": _vec_list(bridge.goal),
                "hover_alt_m": bridge.hover_alt_m,
                "arrived": arrival[i] is not None,
                "arrival_time_s": arrival[i],
                "final_position_enu": _vec_list(final_p[i]),
                "final_goal_error_m": float(final_err[i]),
                "plans": int(sum(bridge.plan_counts.values())),
                "plan_codes": dict(bridge.plan_counts),
                "neighbor_sample_deliveries": int(bridge.neighbor_sample_deliveries),
            }
        )
    clear_out = None if min_clear is math.inf else float(min_clear)
    obs_out = None if min_obs is math.inf else float(min_obs)
    return {
        "schema": SCHEMA,
        "report_only": True,
        "criteria_frozen": False,
        "scene_card_criteria_frozen": bool(crit.get("frozen", False)),
        "commit": _git_commit(),
        "scene": card.name,
        "scene_card_hash": card.hash,
        "seed": int(seed),
        "partial": bool(partial),
        "max_s": float(max_s),
        "horizon_s": float(horizon),
        "sim_duration_s": (k_last + 1) * physics_dt if ran else 0.0,
        "n_drones": n,
        "policy": "upstream",
        "adapter": "fake-px4",
        "coordinate_conversion": "yopo_swarm_core.px4",
        "physics_dt_s": float(physics_dt),
        "planner_dt_s": float(planner_dt),
        "setpoint_hz": _hz(physics_dt),
        "planner_hz": _hz(planner_dt),
        "outcome": outcome,
        "reason": reason,
        "arrived": arrived_n,
        "arrival_rate": float(arrived_n) / float(n),
        "arrival_time_s": max(arrival) if track_goal and arrived_n == n else None,
        "collisions": 0 if outcome != "COLLISION" else 1,
        "collision_reason": reason if outcome == "COLLISION" else None,
        "min_surface_clearance_m": clear_out,
        "min_obstacle_distance_m": obs_out,
        "min_inter_drone_distance_m": min_inter,
        "body_radius_m": body,
        "enu_ned_roundtrip_max_error_m": float(rt_p),
        "enu_ned_roundtrip_max_velocity_mps": float(rt_v),
        "enu_ned_roundtrip_max_acceleration_mps2": float(rt_a),
        "neighbor_sample_deliveries": int(sum(b.neighbor_sample_deliveries for b in bridges)),
        "drones": drone_rows,
        "wall_time_s": time.perf_counter() - wall0,
    }


def normalize_positioning(value: str | None) -> str:
    """定位模式只记进结果。``rtk`` 仍由容器启动时已有的白名单环境注入，桥不写参数。"""
    text = "default" if value is None or str(value).strip() == "" else str(value).strip().lower()
    if text not in POSITIONING_MODES:
        raise ValueError(f"positioning 只要 default 或 rtk，收到 {value!r}")
    return text


def vehicle_identity(drone_index: int, n_drones: int) -> dict:
    """与 ``fleet.fleet_specs`` 对齐的 namespace、MAV_SYS_ID 和 GCS 端口。

    1 架是 instance 0：空 namespace、system 1、端口 18570。
    多架从 instance 1 起：``px4_1`` 的 system 是 2，端口 18571。
    """
    if isinstance(n_drones, bool) or not isinstance(n_drones, int) or n_drones < 1:
        raise ValueError(f"n_drones 必须是正整数，收到 {n_drones!r}")
    if not 0 <= int(drone_index) < n_drones:
        raise ValueError(f"drone_index={drone_index} 不在 0..{n_drones - 1}")
    if n_drones == 1:
        return {"namespace": "", "mav_sys_id": 1, "gcs_port": GCS_PORT_BASE, "instance": 0}
    instance = int(drone_index) + 1
    return {
        "namespace": f"px4_{instance}",
        "mav_sys_id": instance + 1,
        "gcs_port": GCS_PORT_BASE + instance,
        "instance": instance,
    }


def _perception_topic(namespace: str, kind: str) -> str:
    if kind not in ("depth_image", "obstacle_summary"):
        raise ValueError(f"未知感知话题 {kind}")
    ns = namespace.strip("/")
    prefix = f"/{ns}" if ns else ""
    return f"{prefix}/yopo/{kind}"


def vehicle_topics(namespace: str) -> dict[str, str]:
    """一架飞机的 PX4 话题和本桥的感知话题。空 namespace 就是 ``/fmu/...``。"""
    return {
        "vehicle_odometry": px4.topic("out", "vehicle_odometry", namespace),
        "vehicle_status": px4.topic("out", "vehicle_status", namespace),
        "vehicle_land_detected": px4.topic("out", "vehicle_land_detected", namespace),
        "trajectory_setpoint": px4.topic("in", "trajectory_setpoint", namespace),
        "offboard_control_mode": px4.topic("in", "offboard_control_mode", namespace),
        "vehicle_command": px4.topic("in", "vehicle_command", namespace),
        "depth_image": _perception_topic(namespace, "depth_image"),
        "obstacle_summary": _perception_topic(namespace, "obstacle_summary"),
    }


def vehicle_command_message(kind: str, timestamp_us: int, target_system: int) -> dict:
    """解锁、offboard、降落。目标 system 用该机的 MAV_SYS_ID，不广播，也不改参数。"""
    if kind == "offboard":
        return px4.offboard_mode_command(timestamp_us, target_system)
    if kind == "arm":
        return px4.arm_command(timestamp_us, target_system, arm=True)
    if kind == "land":
        return px4.vehicle_command(
            VEHICLE_CMD_NAV_LAND, timestamp_us, target_system, param4=math.nan
        )
    raise ValueError(f"未知 VehicleCommand {kind}")


def land_status_done(command: dict, *, landed: bool, armed: bool) -> str:
    """降落命令发出后，只有「已落地且上锁」才算收尾完成。"""
    if int(command.get("command", -1)) != VEHICLE_CMD_NAV_LAND:
        return "LAND"
    if landed and not armed:
        return "DONE"
    return "LAND"


def _msg_get(msg, name: str, default=None):
    if msg is None:
        return default
    if isinstance(msg, dict):
        return msg[name] if name in msg else default
    return getattr(msg, name, default)


def odometry_message_from_odometry(odom: dict, timestamp_us: int) -> dict:
    """``FakePx4Vehicle.odometry`` → ``px4_msgs/VehicleOdometry`` 字段（仍是 NED）。"""
    msg = {
        "position": _vec_list(odom["position_ned"]),
        "velocity": _vec_list(odom["velocity_ned"]),
        "q": _vec_list(odom["q_ned_frd"]),
        "timestamp": int(timestamp_us),
        "pose_frame": int(px4.POSE_FRAME_NED),
        "velocity_frame": int(px4.VELOCITY_FRAME_NED),
    }
    if odom.get("acceleration_ned") is not None:
        msg["acceleration"] = _vec_list(odom["acceleration_ned"])
    if odom.get("yaw_ned") is not None and math.isfinite(float(odom["yaw_ned"])):
        msg["yaw"] = float(odom["yaw_ned"])
    return msg


def decode_vehicle_odometry(msg, now_s: float | None = None) -> dict:
    """``VehicleOdometry`` 字段 → ENU 位置/速度/加速度/航向。帧不对或有 NaN 时拒绝。"""
    position = _as_vec3(_msg_get(msg, "position"), "odometry.position")
    velocity = _as_vec3(_msg_get(msg, "velocity"), "odometry.velocity")
    quat = _msg_get(msg, "q")
    if quat is None:
        raise ValueError("VehicleOdometry 缺少 q")
    quat = np.asarray(quat, dtype=float).reshape(-1)
    if quat.shape != (4,) or not np.all(np.isfinite(quat)):
        raise ValueError("VehicleOdometry.q 必须是有限的 4 维四元数")
    pose_frame = int(_msg_get(msg, "pose_frame", px4.POSE_FRAME_NED))
    vel_frame = int(_msg_get(msg, "velocity_frame", px4.VELOCITY_FRAME_NED))
    timestamp = _msg_get(msg, "timestamp")
    if timestamp is None:
        if now_s is None:
            raise ValueError("VehicleOdometry 缺少 timestamp")
        timestamp_us = int(round(float(now_s) * 1e6))
    else:
        timestamp_us = int(timestamp)
    enu = px4.odometry_to_enu(position, quat, velocity, timestamp_us, pose_frame, vel_frame)
    raw_acc = _msg_get(msg, "acceleration")
    if raw_acc is None:
        acceleration_ned = None
        acceleration_enu = np.zeros(3)
    else:
        acceleration_ned = _as_vec3(raw_acc, "odometry.acceleration")
        acceleration_enu = np.asarray(px4.ned_to_enu(acceleration_ned), dtype=float)
    yaw = math.atan2(float(enu.R_wb[1, 0]), float(enu.R_wb[0, 0]))
    return {
        "position_enu": np.asarray(enu.position, dtype=float).copy(),
        "velocity_enu": np.asarray(enu.velocity, dtype=float).copy(),
        "acceleration_enu": acceleration_enu.copy(),
        "yaw_enu": float(yaw),
        "timestamp_us": timestamp_us,
        "position_ned": position.copy(),
        "velocity_ned": velocity.copy(),
        "acceleration_ned": None if acceleration_ned is None else acceleration_ned.copy(),
        "q_ned_frd": quat.copy(),
    }


def encode_depth_image(frame: DepthFrame) -> dict:
    """``DepthFrame`` → ``sensor_msgs/Image`` 的 32FC1 字段。字节是小端 float32。"""
    depth = np.ascontiguousarray(frame.depth, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError(f"深度必须是二维，收到 {depth.shape}")
    return {
        "height": int(depth.shape[0]),
        "width": int(depth.shape[1]),
        "encoding": "32FC1",
        "is_bigendian": 0,
        "step": int(depth.shape[1] * 4),
        "data": depth.tobytes(),
        "stamp_s": float(frame.stamp),
        "K": [float(x) for x in frame.K],
    }


def decode_depth_image(msg, *, fallback_K=None, stamp_s: float | None = None) -> DepthFrame:
    """``sensor_msgs/Image`` 字段 → ``DepthFrame``。32FC1 是米，16UC1 是毫米，0 视为无回波。"""
    if isinstance(msg, DepthFrame):
        return msg
    height = int(_msg_get(msg, "height"))
    width = int(_msg_get(msg, "width"))
    encoding = str(_msg_get(msg, "encoding", "32FC1"))
    if height < 1 or width < 1:
        raise ValueError(f"深度图尺寸非法：{height}x{width}")
    data = _msg_get(msg, "data")
    if isinstance(data, np.ndarray) and data.ndim == 2:
        depth = np.array(data, dtype=np.float32, copy=True)
        if depth.shape != (height, width):
            raise ValueError(f"深度数组形状 {depth.shape} 和 {height}x{width} 不一致")
    else:
        raw = bytes(data)
        big = int(_msg_get(msg, "is_bigendian", 0) or 0) == 1
        endian = ">" if big else "<"
        if encoding == "32FC1":
            expect = height * width * 4
            if len(raw) != expect:
                raise ValueError(f"32FC1 字节数 {len(raw)} != {expect}")
            depth = np.frombuffer(raw, dtype=f"{endian}f4").reshape(height, width).astype(np.float32, copy=True)
        elif encoding == "16UC1":
            expect = height * width * 2
            if len(raw) != expect:
                raise ValueError(f"16UC1 字节数 {len(raw)} != {expect}")
            mm = np.frombuffer(raw, dtype=f"{endian}u2").reshape(height, width)
            depth = mm.astype(np.float32) * np.float32(0.001)
            depth[mm == 0] = np.nan
        else:
            raise ValueError(f"不支持的深度编码 {encoding}")
    raw_k = _msg_get(msg, "K", fallback_K)
    if raw_k is None:
        raise ValueError("深度图缺少 K")
    intrinsics = tuple(float(x) for x in raw_k)
    if len(intrinsics) != 4 or not all(math.isfinite(x) for x in intrinsics):
        raise ValueError(f"K 需要 4 个有限数，收到 {raw_k!r}")
    stamp = float(stamp_s) if stamp_s is not None else float(_msg_get(msg, "stamp_s", 0.0))
    return DepthFrame(stamp, depth, intrinsics)  # type: ignore[arg-type]


def depth_from_obstacle_summary(summary, position_enu, yaw_enu: float, camera: CameraConfig, stamp: float) -> DepthFrame:
    """障碍摘要 → 一帧 z 深度。点用 ENU；只有距离时按机体航向排成水平扇面。

    投影和 ``DepthCamera`` 同一套光学轴（``R_BODY_CAM``）。摘要里没有的像素是 NaN。
    """
    if isinstance(summary, str):
        summary = json.loads(summary)
    if not isinstance(summary, dict):
        raise ValueError("障碍摘要必须是对象或 JSON 对象字符串")
    height, width = int(camera.height), int(camera.width)
    depth = np.full((height, width), np.nan, dtype=np.float32)
    points = summary.get("points_enu")
    if points is None and summary.get("ranges_m") is not None:
        ranges = np.asarray(summary["ranges_m"], dtype=float).reshape(-1)
        if "bearings_rad" in summary:
            bearings = np.asarray(summary["bearings_rad"], dtype=float).reshape(-1)
        else:
            half = math.radians(float(camera.hfov_deg)) / 2.0
            bearings = np.linspace(float(yaw_enu) - half, float(yaw_enu) + half, int(ranges.size))
        if bearings.shape != ranges.shape:
            raise ValueError("ranges_m 和 bearings_rad 长度不一致")
        if "elevations_rad" in summary:
            elevations = np.asarray(summary["elevations_rad"], dtype=float).reshape(-1)
            if elevations.shape != ranges.shape:
                raise ValueError("elevations_rad 和 ranges_m 长度不一致")
        else:
            elevations = np.zeros(ranges.size)
        origin = np.asarray(position_enu, dtype=float).reshape(3)
        points = np.stack(
            [
                origin[0] + ranges * np.cos(elevations) * np.cos(bearings),
                origin[1] + ranges * np.cos(elevations) * np.sin(bearings),
                origin[2] + ranges * np.sin(elevations),
            ],
            axis=1,
        )
    if points is not None:
        cloud = np.asarray(points, dtype=float).reshape(-1, 3)
        yaw = float(yaw_enu)
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        cam_pos = np.asarray(position_enu, dtype=float).reshape(3) + rotation @ np.asarray(camera.mount_xyz_body, dtype=float)
        world_cam = rotation @ R_BODY_CAM
        fx, fy, cx, cy = camera.intrinsics
        for point in cloud:
            if not np.all(np.isfinite(point)):
                continue
            rel = world_cam.T @ (point - cam_pos)
            z = float(rel[2])
            if not math.isfinite(z) or z < float(camera.min_range_m) or z > float(camera.max_range_m):
                continue
            uc = cx + fx * (float(rel[0]) / z)
            vc = cy + fy * (float(rel[1]) / z)
            ui, vi = int(math.floor(uc)), int(math.floor(vc))
            if ui < 0 or vi < 0 or ui >= width or vi >= height:
                continue
            current = float(depth[vi, ui])
            if not math.isfinite(current) or z < current:
                depth[vi, ui] = np.float32(z)
    return DepthFrame(float(stamp), depth, camera.intrinsics)


def resolve_depth_frame(
    bridge: YopoSitlBridge,
    decoded: dict,
    now_s: float,
    *,
    depth=None,
    obstacle_summary=None,
    depth_source: str = "auto",
) -> tuple[str, DepthFrame | None]:
    """选出这一拍的深度。返回的帧是 None 时，``command`` 仍用场景卡相机渲染。"""
    if depth_source not in ("auto", "depth", "summary", "scene-card"):
        raise ValueError(f"depth_source 只要 auto、depth、summary 或 scene-card，收到 {depth_source!r}")
    if depth_source in ("auto", "depth") and depth is not None:
        return "ros-depth", decode_depth_image(depth, fallback_K=bridge.camera.cfg.intrinsics, stamp_s=now_s)
    if depth_source in ("auto", "summary") and obstacle_summary is not None:
        frame = depth_from_obstacle_summary(
            obstacle_summary, decoded["position_enu"], decoded["yaw_enu"], bridge.camera.cfg, now_s
        )
        return "ros-obstacle", frame
    if depth_source in ("auto", "scene-card"):
        return "scene-card", None
    return "missing", None


def command_from_ros(
    bridge: YopoSitlBridge,
    odometry,
    now_s: float,
    *,
    depth=None,
    obstacle_summary=None,
    depth_source: str = "auto",
) -> tuple[BridgeOutput, str]:
    """一条里程计消息（加可选深度或障碍摘要）→ NED 设定点。"""
    decoded = decode_vehicle_odometry(odometry, now_s=now_s)
    source, frame = resolve_depth_frame(
        bridge, decoded, now_s, depth=depth, obstacle_summary=obstacle_summary, depth_source=depth_source
    )
    if source == "missing":
        raise ValueError(f"{bridge.cfg.px4_namespace or '/'} 这一拍没有 {depth_source} 感知")
    out = bridge.command(
        decoded["position_ned"],
        decoded["velocity_ned"],
        now_s,
        q_ned_frd=decoded["q_ned_frd"],
        acceleration_ned=decoded["acceleration_ned"],
        depth_frame=frame,
    )
    return out, source


def _goal_limits(card: SceneCard) -> tuple[float, float, float, float]:
    crit = card.card.get("criteria") or {}
    body = float(crit.get("body_radius_m", 0.35))
    if "goal_tolerance_m" in crit:
        return float(crit["goal_tolerance_m"]), float(crit["goal_speed_mps"]), float(crit["goal_hold_s"]), body
    return float(crit.get("hover_tolerance_m", 0.3)), math.inf, 0.0, body


@dataclass
class VehicleInbox:
    """一架飞机这一拍的 ROS 字段。布尔量由调用方从 VehicleStatus / LandDetected 译好。"""

    odometry: object | None = None
    depth: object | None = None
    obstacle_summary: object | None = None
    armed: bool | None = None
    offboard: bool | None = None
    landed: bool | None = None
    preflight_ok: bool = True


@dataclass(frozen=True)
class LiveCommand:
    namespace: str
    mav_sys_id: int
    gcs_port: int
    topics: dict
    phase: str
    heartbeat: bool
    setpoint: dict | None
    offboard: dict | None
    vehicle_commands: tuple
    planned: bool
    depth_source: str
    roundtrip_error_m: float
    position_enu: tuple | None


@dataclass
class LiveFleetConfig:
    positioning: str = "default"
    depth_source: str = "auto"
    initial_phase: str = "WARMUP"
    warmup_s: float = 1.0
    arm_timeout_s: float = 60.0
    arm_retry_s: float = 1.0
    takeoff_timeout_s: float = 30.0
    plan_timeout_s: float = 45.0
    land_timeout_s: float = 60.0
    settle_pos_m: float = 0.2
    settle_vel_mps: float = 0.3
    settle_hold_s: float = 1.0

    def __post_init__(self) -> None:
        self.positioning = normalize_positioning(self.positioning)
        if self.depth_source not in ("auto", "depth", "summary", "scene-card"):
            raise ValueError(f"depth_source 非法：{self.depth_source!r}")
        if self.initial_phase not in ("WARMUP", "PLAN"):
            raise ValueError(f"initial_phase 只要 WARMUP 或 PLAN，收到 {self.initial_phase!r}")
        for name in ("warmup_s", "arm_timeout_s", "takeoff_timeout_s", "plan_timeout_s", "land_timeout_s"):
            if not float(getattr(self, name)) > 0.0:
                raise ValueError(f"{name} 必须为正")


class _LiveSlot:
    def __init__(self, bridge: YopoSitlBridge, ident: dict, phase: str):
        self.bridge = bridge
        self.ident = ident
        self.phase = phase
        self.t_phase: float | None = None
        self.home: np.ndarray | None = None
        self.hover: np.ndarray | None = None
        self.goal_since: float | None = None
        self.arrival: float | None = None
        self.settle_since: float | None = None
        self.last_cmd_s = -math.inf
        self.depth_source = ""
        self.position: np.ndarray | None = None
        self.velocity: np.ndarray | None = None
        self.yaw = 0.0
        self.land_sent = False


class LiveFleetSession:
    """容器侧状态机：预热、解锁、起飞、按桥规划、全体到达后 NAV_LAND。

    不写 PX4 参数。每架只读自己 namespace 的消息。规划设定点仍来自 ``YopoSitlBridge``。
    """

    def __init__(self, card: SceneCard, bridges: list[YopoSitlBridge], idents: list[dict], cfg: LiveFleetConfig):
        if len(bridges) != len(idents) or not bridges:
            raise ValueError("bridges 和 idents 必须非空且一一对应")
        self.card = card
        self.cfg = cfg
        self.slots = [_LiveSlot(bridge, ident, cfg.initial_phase) for bridge, ident in zip(bridges, idents)]
        self.goal_tol, self.goal_speed, self.goal_hold, self.body = _goal_limits(card)
        self.fail_reason = ""
        self.px4_parameters_written: list = []
        self.land_commands_sent = 0
        self.max_roundtrip_m = 0.0
        self._last_now: float | None = None
        self._last_cmds: list[LiveCommand] = []

    def step(self, now_s: float, inbox_by_ns: dict[str, VehicleInbox]) -> list[LiveCommand]:
        now_s = float(now_s)
        if self._last_now is not None and abs(now_s - self._last_now) <= 1e-12:
            return list(self._last_cmds)
        decoded: dict[str, dict | None] = {}
        for slot in self.slots:
            ns = slot.ident["namespace"]
            inbox = inbox_by_ns.get(ns)
            if inbox is None or inbox.odometry is None:
                decoded[ns] = None
                continue
            try:
                decoded[ns] = decode_vehicle_odometry(inbox.odometry, now_s=now_s)
            except ValueError:
                decoded[ns] = None
        self._exchange(decoded)
        if self.fail_reason == "":
            self._note_separation(decoded, now_s)
        commands = [self._step_slot(slot, now_s, inbox_by_ns.get(slot.ident["namespace"]), decoded[slot.ident["namespace"]]) for slot in self.slots]
        if self.fail_reason == "" and all(slot.arrival is not None for slot in self.slots):
            for slot in self.slots:
                if slot.phase == "PLAN":
                    slot.phase = "LAND"
                    slot.t_phase = now_s
        self._last_now = now_s
        self._last_cmds = commands
        return list(commands)

    def _exchange(self, decoded: dict[str, dict | None]) -> None:
        for slot in self.slots:
            if slot.phase != "PLAN":
                continue
            others = []
            for other in self.slots:
                if other is slot:
                    continue
                state = decoded.get(other.ident["namespace"])
                if state is None:
                    continue
                others.append(
                    NeighborEnu(
                        other.bridge.drone_id,
                        state["position_enu"],
                        state["velocity_enu"],
                        other.bridge.broadcast_samples(),
                    )
                )
            slot.bridge.set_neighbors(others)

    def _note_separation(self, decoded: dict[str, dict | None], now_s: float) -> None:
        positions = []
        for slot in self.slots:
            state = decoded.get(slot.ident["namespace"])
            if state is None or slot.phase not in ("TAKEOFF", "PLAN"):
                continue
            position = state["position_enu"]
            if float(position[2]) <= self.body:
                continue
            if float(self.card.min_distance(position)) <= self.body:
                self._fail(f"{slot.bridge.drone_id}:OBSTACLE", now_s)
                return
            positions.append((slot, position))
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                dist = float(np.linalg.norm(positions[i][1] - positions[j][1]))
                if dist <= 2.0 * self.body:
                    left, right = positions[i][0].bridge.drone_id, positions[j][0].bridge.drone_id
                    self._fail(f"{left}-{right}:INTER_DRONE", now_s)
                    return

    def _fail(self, reason: str, now_s: float) -> None:
        self.fail_reason = reason
        for slot in self.slots:
            if slot.phase != "DONE":
                slot.phase = "FAIL"
                slot.t_phase = now_s
                slot.land_sent = False

    def _pack(
        self,
        slot: _LiveSlot,
        now_s: float,
        *,
        heartbeat: bool,
        setpoint: dict | None,
        commands: list[dict],
        planned: bool,
        depth_source: str,
        roundtrip: float,
        position,
    ) -> LiveCommand:
        offboard = None if setpoint is None else px4.offboard_control_mode(setpoint["timestamp"])
        if position is not None:
            self.max_roundtrip_m = max(self.max_roundtrip_m, float(roundtrip))
        for command in commands:
            if int(command.get("command", -1)) == VEHICLE_CMD_NAV_LAND:
                self.land_commands_sent += 1
        pos = None if position is None else tuple(float(x) for x in np.asarray(position, dtype=float).reshape(3))
        return LiveCommand(
            namespace=slot.ident["namespace"],
            mav_sys_id=int(slot.ident["mav_sys_id"]),
            gcs_port=int(slot.ident["gcs_port"]),
            topics=vehicle_topics(slot.ident["namespace"]),
            phase=slot.phase,
            heartbeat=heartbeat,
            setpoint=setpoint,
            offboard=offboard,
            vehicle_commands=tuple(commands),
            planned=planned,
            depth_source=depth_source,
            roundtrip_error_m=float(roundtrip),
            position_enu=pos,
        )

    def _hold(self, position, yaw: float, now_s: float) -> tuple[dict, float]:
        setpoint = trajectory_setpoint_ned(position, np.zeros(3), np.zeros(3), yaw, now_s)
        return setpoint, roundtrip_abs(position)

    def _lock_home(self, slot: _LiveSlot, state: dict) -> None:
        if slot.home is None:
            slot.home = np.asarray(state["position_enu"], dtype=float).copy()
            slot.hover = np.array([slot.home[0], slot.home[1], float(slot.bridge.hover_alt_m)], dtype=float)

    def _step_slot(self, slot: _LiveSlot, now_s: float, inbox: VehicleInbox | None, state: dict | None) -> LiveCommand:
        if state is not None:
            slot.position = np.asarray(state["position_enu"], dtype=float).copy()
            slot.velocity = np.asarray(state["velocity_enu"], dtype=float).copy()
            slot.yaw = float(state["yaw_enu"])
            self._lock_home(slot, state)
        if slot.phase == "DONE":
            return self._pack(slot, now_s, heartbeat=False, setpoint=None, commands=[], planned=False, depth_source="", roundtrip=0.0, position=slot.position)
        if state is None or slot.home is None:
            return self._pack(slot, now_s, heartbeat=False, setpoint=None, commands=[], planned=False, depth_source="", roundtrip=0.0, position=None)
        if slot.t_phase is None:
            slot.t_phase = now_s
        if slot.phase == "FAIL":
            return self._land_pack(slot, now_s, inbox)
        if slot.phase == "WARMUP":
            setpoint, roundtrip = self._hold(slot.position, slot.yaw, now_s)
            if now_s - slot.t_phase >= self.cfg.warmup_s:
                slot.phase = "ARMING"
                slot.t_phase = now_s
            return self._pack(slot, now_s, heartbeat=True, setpoint=setpoint, commands=[], planned=False, depth_source="", roundtrip=roundtrip, position=slot.position)
        if slot.phase == "ARMING":
            return self._arm(slot, now_s, inbox, state)
        if slot.phase == "TAKEOFF":
            return self._takeoff(slot, now_s, inbox, state)
        if slot.phase == "PLAN":
            return self._plan(slot, now_s, inbox, state)
        if slot.phase == "LAND":
            return self._land_pack(slot, now_s, inbox)
        return self._pack(slot, now_s, heartbeat=False, setpoint=None, commands=[], planned=False, depth_source="", roundtrip=0.0, position=slot.position)

    def _arm(self, slot: _LiveSlot, now_s: float, inbox: VehicleInbox | None, state: dict | None) -> LiveCommand:
        armed = bool(inbox and inbox.armed)
        offboard = bool(inbox and inbox.offboard)
        if armed and offboard:
            slot.phase = "TAKEOFF"
            slot.t_phase = now_s
            slot.settle_since = None
            return self._takeoff(slot, now_s, inbox, state)
        if now_s - float(slot.t_phase) > self.cfg.arm_timeout_s:
            self._fail(f"{slot.bridge.drone_id}:ARM_TIMEOUT", now_s)
            return self._land_pack(slot, now_s, inbox)
        commands = []
        if now_s - slot.last_cmd_s >= self.cfg.arm_retry_s:
            slot.last_cmd_s = now_s
            ts = int(round(now_s * 1e6))
            if not offboard:
                commands.append(vehicle_command_message("offboard", ts, slot.ident["mav_sys_id"]))
            if not armed:
                commands.append(vehicle_command_message("arm", ts, slot.ident["mav_sys_id"]))
        setpoint, roundtrip = self._hold(slot.home, slot.yaw, now_s)
        return self._pack(slot, now_s, heartbeat=True, setpoint=setpoint, commands=commands, planned=False, depth_source="", roundtrip=roundtrip, position=slot.position)

    def _takeoff(self, slot: _LiveSlot, now_s: float, inbox: VehicleInbox | None, state: dict | None) -> LiveCommand:
        target = slot.hover
        err = float(np.linalg.norm(slot.position - target))
        speed = float(np.linalg.norm(slot.velocity))
        if err < self.cfg.settle_pos_m and speed < self.cfg.settle_vel_mps:
            slot.settle_since = now_s if slot.settle_since is None else slot.settle_since
            if now_s - slot.settle_since >= self.cfg.settle_hold_s - 1e-9:
                slot.phase = "PLAN"
                slot.t_phase = now_s
                slot.goal_since = None
        else:
            slot.settle_since = None
        if slot.phase == "TAKEOFF" and now_s - float(slot.t_phase) > self.cfg.takeoff_timeout_s:
            self._fail(f"{slot.bridge.drone_id}:TAKEOFF_TIMEOUT", now_s)
            return self._land_pack(slot, now_s, inbox)
        if slot.phase == "PLAN":
            return self._plan(slot, now_s, inbox, state)
        setpoint = trajectory_setpoint_ned(target, np.zeros(3), np.zeros(3), slot.yaw, now_s)
        return self._pack(
            slot, now_s, heartbeat=True, setpoint=setpoint, commands=[], planned=False,
            depth_source="", roundtrip=roundtrip_abs(target), position=slot.position,
        )

    def _plan(self, slot: _LiveSlot, now_s: float, inbox: VehicleInbox | None, state: dict | None) -> LiveCommand:
        if state is None:
            setpoint, roundtrip = self._hold(slot.position, slot.yaw, now_s)
            return self._pack(slot, now_s, heartbeat=True, setpoint=setpoint, commands=[], planned=False, depth_source="missing", roundtrip=roundtrip, position=slot.position)
        if now_s - float(slot.t_phase) > self.cfg.plan_timeout_s and slot.arrival is None:
            self._fail(f"{slot.bridge.drone_id}:PLAN_TIMEOUT", now_s)
            return self._land_pack(slot, now_s, inbox)
        source, frame = resolve_depth_frame(
            slot.bridge,
            state,
            now_s,
            depth=None if inbox is None else inbox.depth,
            obstacle_summary=None if inbox is None else inbox.obstacle_summary,
            depth_source=self.cfg.depth_source,
        )
        slot.depth_source = source
        if source == "missing":
            setpoint, roundtrip = self._hold(slot.position, slot.yaw, now_s)
            return self._pack(slot, now_s, heartbeat=True, setpoint=setpoint, commands=[], planned=False, depth_source=source, roundtrip=roundtrip, position=slot.position)
        out = slot.bridge.command(
            state["position_ned"],
            state["velocity_ned"],
            now_s,
            q_ned_frd=state["q_ned_frd"],
            acceleration_ned=state["acceleration_ned"],
            depth_frame=frame,
        )
        err = float(np.linalg.norm(state["position_enu"] - slot.bridge.goal))
        speed = float(np.linalg.norm(state["velocity_enu"]))
        if err <= self.goal_tol and speed <= self.goal_speed:
            slot.goal_since = now_s if slot.goal_since is None else slot.goal_since
            if now_s - slot.goal_since >= self.goal_hold - 1e-9:
                slot.arrival = slot.goal_since
        else:
            slot.goal_since = None
        return self._pack(
            slot,
            now_s,
            heartbeat=True,
            setpoint=out.setpoint,
            commands=[],
            planned=out.planned,
            depth_source=source,
            roundtrip=out.roundtrip_error_m,
            position=state["position_enu"],
        )

    def _land_pack(self, slot: _LiveSlot, now_s: float, inbox: VehicleInbox | None) -> LiveCommand:
        landed = bool(inbox and inbox.landed)
        armed = True if inbox is None or inbox.armed is None else bool(inbox.armed)
        command = vehicle_command_message("land", int(round(now_s * 1e6)), slot.ident["mav_sys_id"])
        if land_status_done(command, landed=landed, armed=armed) == "DONE":
            slot.phase = "DONE"
            commands = [command] if not slot.land_sent else []
            slot.land_sent = True
            return self._pack(slot, now_s, heartbeat=False, setpoint=None, commands=commands, planned=False, depth_source=slot.depth_source, roundtrip=0.0, position=slot.position)
        commands = []
        if not slot.land_sent or now_s - slot.last_cmd_s >= self.cfg.arm_retry_s:
            slot.last_cmd_s = now_s
            slot.land_sent = True
            commands = [command]
        if slot.phase != "FAIL" and slot.t_phase is not None and now_s - float(slot.t_phase) > self.cfg.land_timeout_s:
            self._fail(f"{slot.bridge.drone_id}:LAND_TIMEOUT", now_s)
        return self._pack(slot, now_s, heartbeat=False, setpoint=None, commands=commands, planned=False, depth_source=slot.depth_source, roundtrip=0.0, position=slot.position)

    def result(self, *, adapter: str = "px4-ros") -> dict:
        n = len(self.slots)
        arrived = sum(slot.arrival is not None for slot in self.slots)
        if self.slots and all(slot.phase == "DONE" for slot in self.slots):
            outcome, reason, ok = "SUCCESS", "GOAL_REACHED_AND_HELD", True
        elif any(slot.phase == "FAIL" for slot in self.slots):
            outcome, reason, ok = "FAIL", self.fail_reason or "FAIL", False
        else:
            outcome, reason, ok = "RUNNING", self.slots[0].phase if self.slots else "", False
        drones = []
        for slot in self.slots:
            drones.append(
                {
                    "id": slot.bridge.drone_id,
                    "px4_namespace": slot.ident["namespace"],
                    "mav_sys_id": int(slot.ident["mav_sys_id"]),
                    "gcs_port": int(slot.ident["gcs_port"]),
                    "phase": slot.phase,
                    "goal_enu": _vec_list(slot.bridge.goal),
                    "arrived": slot.arrival is not None,
                    "arrival_time_s": slot.arrival,
                    "position_enu": None if slot.position is None else _vec_list(slot.position),
                    "depth_source": slot.depth_source,
                    "plans": int(sum(slot.bridge.plan_counts.values())),
                }
            )
        return {
            "schema": SCHEMA_LIVE,
            "report_only": True,
            "criteria_frozen": False,
            "adapter": adapter,
            "fake_px4": False,
            "scene": self.card.name,
            "n_drones": n,
            "positioning": self.cfg.positioning,
            "px4_parameters_written": list(self.px4_parameters_written),
            "coordinate_conversion": "yopo_swarm_core.px4",
            "namespaces": [slot.ident["namespace"] for slot in self.slots],
            "mav_sys_ids": [int(slot.ident["mav_sys_id"]) for slot in self.slots],
            "gcs_ports": [int(slot.ident["gcs_port"]) for slot in self.slots],
            "topics": {slot.ident["namespace"]: vehicle_topics(slot.ident["namespace"]) for slot in self.slots},
            "outcome": outcome,
            "reason": reason,
            "ok": ok,
            "arrived": arrived,
            "arrival_rate": float(arrived) / float(n),
            "landed": bool(self.slots) and all(slot.phase == "DONE" for slot in self.slots),
            "land_commands_sent": int(self.land_commands_sent),
            "enu_ned_roundtrip_max_error_m": float(self.max_roundtrip_m),
            "depth_sources": sorted({slot.depth_source for slot in self.slots if slot.depth_source}),
            "drones": drones,
        }


def make_live_session(
    card: SceneCard,
    *,
    seed: int = 0,
    physics_dt: float = 0.02,
    planner_dt: float = 0.1,
    live: LiveFleetConfig | None = None,
) -> LiveFleetSession:
    """按场景卡建一队桥。1 机 namespace 为空；多机是 ``px4_1`` 起。"""
    cfg = live or LiveFleetConfig()
    drones = list(card.card["drones"])
    n = len(drones)
    dyn = card.card.get("dynamics") or {}
    params = dict(card.card.get("policy_params") or {})
    bridges = []
    idents = []
    for i, drone in enumerate(drones):
        ident = vehicle_identity(i, n)
        idents.append(ident)
        start = drone["start_enu"]
        bridges.append(
            YopoSitlBridge(
                card,
                SitlBridgeConfig(
                    physics_dt=physics_dt,
                    planner_dt=planner_dt,
                    hover_alt_m=float(start[2]),
                    goal_enu=tuple(drone.get("goal_enu", drone.get("hover_enu", start))),
                    policy_params=params,
                    drone_id=drone["id"],
                    seed=int(seed),
                    drone_index=i,
                    n_drones=n,
                    a_max=float(dyn.get("a_max", 6.0)),
                    tau_s=float(dyn.get("tau_s", 0.1)),
                    kp=float(dyn.get("kp", 4.0)),
                    kd=float(dyn.get("kd", 3.0)),
                    px4_namespace=ident["namespace"],
                ),
            )
        )
    return LiveFleetSession(card, bridges, idents, cfg)


def live_topic_plan(card: SceneCard, *, positioning: str = "default", seed: int = 0) -> dict:
    """不起策略、不连 ROS。只列出每机话题、system id 和目标，给容器入口的 dry-run。"""
    positioning = normalize_positioning(positioning)
    drones = list(card.card["drones"])
    n = len(drones)
    idents = [vehicle_identity(i, n) for i in range(n)]
    return {
        "schema": SCHEMA_LIVE,
        "report_only": True,
        "criteria_frozen": False,
        "dry_run": True,
        "adapter": "px4-ros",
        "fake_px4": False,
        "scene": card.name,
        "seed": int(seed),
        "n_drones": n,
        "positioning": positioning,
        "px4_parameters_written": [],
        "coordinate_conversion": "yopo_swarm_core.px4",
        "namespaces": [ident["namespace"] for ident in idents],
        "mav_sys_ids": [ident["mav_sys_id"] for ident in idents],
        "gcs_ports": [ident["gcs_port"] for ident in idents],
        "drone_ids": [drone["id"] for drone in drones],
        "topics": {ident["namespace"]: vehicle_topics(ident["namespace"]) for ident in idents},
        "goals_enu": [
            _vec_list(drone.get("goal_enu", drone.get("hover_enu", drone["start_enu"]))) for drone in drones
        ],
    }


def run_mock_ros_closed_loop(
    card: SceneCard,
    *,
    positioning: str = "default",
    land_on_arrival: bool = True,
    **kwargs,
) -> dict:
    """假 PX4 上走 ROS 消息字段的闭环。默认在到达并保持之后补一条 NAV_LAND 收尾。

    物理仍是 ``FakePx4Vehicle``。``positioning`` 只写入结果，不产生任何参数写入。
    """
    positioning = normalize_positioning(positioning)
    result = dict(run_closed_loop(card, ros_messages=True, **kwargs))
    n = int(result["n_drones"])
    idents = [vehicle_identity(i, n) for i in range(n)]
    for row, ident in zip(result["drones"], idents):
        if row["px4_namespace"] != ident["namespace"]:
            raise RuntimeError(
                f"{row['id']} 的 namespace {row['px4_namespace']!r} 和 {ident['namespace']!r} 不一致"
            )
    commands: list[dict] = []
    landed = False
    finish = str(result["reason"])
    ok = False
    if land_on_arrival and result["outcome"] == "SUCCESS":
        stamp = int(round(float(result["sim_duration_s"]) * 1e6))
        commands = [vehicle_command_message("land", stamp, ident["mav_sys_id"]) for ident in idents]
        if commands and all(land_status_done(cmd, landed=True, armed=False) == "DONE" for cmd in commands):
            landed = True
            finish = "NAV_LAND"
            ok = True
    result.update(
        schema=SCHEMA_LIVE,
        adapter="mock-ros",
        fake_px4=True,
        positioning=positioning,
        px4_parameters_written=[],
        perception="ros-depth",
        namespaces=[ident["namespace"] for ident in idents],
        mav_sys_ids=[ident["mav_sys_id"] for ident in idents],
        gcs_ports=[ident["gcs_port"] for ident in idents],
        topics={ident["namespace"]: vehicle_topics(ident["namespace"]) for ident in idents},
        land_commands=commands,
        landed=landed,
        finish=finish,
        ok=ok,
        report_only=True,
        criteria_frozen=False,
    )
    return result
