"""票 S4：YOPO L0 与 PX4 Offboard 之间的 ENU/NED 闭环桥。

规划核心保持 ENU。PX4 ``VehicleOdometry`` / ``TrajectorySetpoint`` 是 NED。
两边的向量和航向只经过 ``yopo_swarm_core.px4``（``px4.py`` 模块头注明的 ADR-0011 边界）。
本文件不写第二套轴映射，也不导入 ``rclpy``。

离线闭环用 ``FakePx4Vehicle``：在 NED 里按设定点做点质量 + 一阶加速度滞后，
增益与 ``PointMassLag`` 相同。真 PX4 容器以后只替换读写端口，见
``docs/research/s4-sitl-bridge.md``。
"""

from __future__ import annotations

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
from .sensor import CameraConfig, DepthCamera

SCHEMA = "yopo-restart/s4-sitl-bridge/1"
# ``--partial`` 只跑这么长的前缀，用来做离线校验，不等到达。
PARTIAL_PREFIX_S = 0.5
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
    ) -> BridgeOutput:
        now_s = float(now_s)
        if self._last_output is not None and self._last_sample_s is not None:
            if abs(now_s - self._last_sample_s) <= 1e-12:
                return self._last_output
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
            frame = self.camera.capture(p, yaw, now_s)
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
            out = bridge.command(
                odom["position_ned"],
                odom["velocity_ned"],
                now,
                q_ned_frd=odom["q_ned_frd"],
                yaw_ned=odom["yaw_ned"],
                acceleration_ned=odom["acceleration_ned"],
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
