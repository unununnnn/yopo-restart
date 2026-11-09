"""单机 ROS 边界的可重放适配器；规划决策全部委托 #9，坐标转换全部委托 px4。

本模块只负责每机输入配对、批准凭证生命周期、按仿真时间采样及 IPC 候选源。
初始保持也通过 SafetyGate；未拿到批准凭证时不生成任何外部轨迹命令。
"""

from __future__ import annotations

import math
import re
from collections import deque
from dataclasses import dataclass

import numpy as np

from . import px4
from .baseline import UPSTREAM_SHA, WEIGHTS_SHA256
from .depth import DepthSample, network_depth, optical_pose
from .freespace import CameraIntrinsics, DepthFrame, FreeSpaceConfig
from .infer_ipc import IpcClient, body_obs
from .plan import KinematicLimits, PlanConfig, hover_plan, state_at
from .planner import ApprovedPlan, Decision, Observation, Planner, PlannerConfig, VehicleState
from .takeoff import TakeoffContract
from .yaw import CTRL_DT, calculate_yaw


def default_config(
    owner: str, body_radius=0.2, tracking_margin=0.1, switch_delay=0.5
) -> PlannerConfig:
    return PlannerConfig(
        owner=owner,
        plan=PlanConfig(
            KinematicLimits(4.0, 8.0, 80.0),
            body_radius=body_radius,
            tracking_margin=tracking_margin,
        ),
        space=FreeSpaceConfig(body_radius=body_radius, margin=tracking_margin),
        switch_delay=switch_delay,
    )


def namespace(value: str) -> str:
    value = value.strip("/")
    if not re.fullmatch(r"px4_[0-9]+", value):
        raise ValueError("each node requires its own /px4_N namespace")
    return value


class InputBuffer:
    """有限状态历史；按采集时间配对，不用接收时刻冒充采集时刻。"""

    def __init__(self, cfg: PlannerConfig):
        self.cfg = cfg
        self.states: deque[VehicleState] = deque(maxlen=256)

    def odometry(self, position, q, velocity, stamp_us, pose_frame=1, velocity_frame=1):
        s = px4.odometry_to_enu(position, q, velocity, stamp_us, pose_frame, velocity_frame)
        stamp = s.stamp_us / 1e6
        if self.states and stamp <= self.states[-1].stamp:
            return  # 重复/乱序不能改变当前状态或加速度估计。
        acceleration = np.zeros(3)
        if self.states:
            previous = self.states[-1]
            dt = stamp - previous.stamp
            if 0 < dt <= 0.1:
                acceleration = (s.velocity - previous.velocity) / dt
        self.states.append(VehicleState(s.position, s.velocity, acceleration, s.R_wb, stamp))

    def pair(self, raw_m: np.ndarray, k: CameraIntrinsics, stamp_ns: int):
        if not self.states:
            return None
        stamp = stamp_ns / 1e9
        state = min(self.states, key=lambda s: abs(s.stamp - stamp))
        if abs(state.stamp - stamp) > self.cfg.max_skew:
            return None
        sample = DepthSample(raw_m, k, optical_pose(state.position, state.R_wb), stamp_ns)
        return sample.evidence(self.cfg.space), state

    def stationary(self, now: float, duration: float = 1.0, radius: float = 0.02) -> bool:
        """接地初始参考须来自连续稳定的实际估计，避免把 EKF 启动漂移写成保持参考。"""
        states = [s for s in self.states if 0 <= now - s.stamp <= duration + 0.05]
        return bool(
            states
            and states[-1].stamp - states[0].stamp >= duration
            and 0 <= now - states[-1].stamp <= self.cfg.max_frame_age
            and all(
                np.linalg.norm(s.position - states[-1].position) <= radius
                and np.linalg.norm(s.velocity) <= 0.1
                for s in states
            )
        )


class IpcSource:
    name = "per-uav-ipc"

    def __init__(self, path: str, timeout: float = 2.0):
        self.path, self.timeout = path, timeout
        self.client: IpcClient | None = None
        self.last_header: dict = {}
        self._raw: np.ndarray | None = None
        self._stamp: float | None = None

    def bind_depth(self, raw_m: np.ndarray, stamp: float):
        self._raw, self._stamp = raw_m, stamp

    def describe(self):
        return {
            "upstream_sha": UPSTREAM_SHA,
            "weights_sha256": WEIGHTS_SHA256,
            "socket": self.path,
            "preprocess": "upstream-ns1-uint8-96x160",
        }

    def __call__(self, obs: Observation, dec):
        if self.client is None:
            self.client = IpcClient(self.path, self.timeout)
        s = obs.state
        o = body_obs(s.velocity, s.acceleration, obs.goal, s.position, s.R_wb)
        if self._raw is None or self._stamp != obs.frame.stamp:
            raise ValueError("network raw depth must be bound to this observation")
        try:
            result = self.client.forward(network_depth(self._raw), o)
            self.last_header = self.client.last_header.copy()
            # IPC 服务是 batch=1；#9 候选源接口接受无 batch 的 N×14 / N / N×2nr。
            return tuple(a[0] for a in result)
        except (OSError, RuntimeError):
            self.close()
            raise

    def close(self):
        if self.client is not None:
            self.client.close()
            self.client = None


@dataclass(frozen=True)
class Command:
    setpoint: dict
    mode: dict
    pva_enu: np.ndarray
    plan_hash: str


class UavRuntime:
    def __init__(self, planner: Planner, takeoff: TakeoffContract | None = None):
        self.planner = planner
        self.takeoff = takeoff
        self.current: ApprovedPlan | None = None
        self.pending: ApprovedPlan | None = None
        self.active_branch: int | None = None
        self.scheduled_branch: int | None = None
        self.last_clock: float | None = None
        self.clock_fault = False
        self.last_error = ""
        self.last_yaw: float | None = None  # 上游 last_yaw：上一条命令的航向（ENU）

    def bootstrap(
        self, frame: DepthFrame, state: VehicleState, now: float, assumption_radius: float
    ) -> bool:
        """显式起飞区假设下审批初始静止保持；只在近似静止、鲜活数据时尝试。"""
        cfg = self.planner.cfg
        if self.current or self.pending or self.clock_fault:
            return False
        if not (
            0 <= now - frame.stamp <= cfg.max_frame_age
            and abs(state.stamp - frame.stamp) <= cfg.max_skew
            and np.mean(frame.valid) >= cfg.min_valid_fraction
            and np.linalg.norm(state.velocity) <= 0.15
        ):
            self.last_error = "BOOTSTRAP_INPUT"
            return False
        seed = hover_plan(cfg.owner, state.position, now, cfg.plan)
        hold = hover_plan(
            cfg.owner,
            state.position,
            now + cfg.switch_delay,
            cfg.plan,
            parent=seed.plan_hash(cfg.plan),
        )
        space = self.planner.space
        if self.takeoff is not None:
            if self.takeoff.space is None and not self.takeoff.initialize(
                frame, state, cfg, assumption_radius
            ):
                self.last_error = "SUPPORT_NOT_VERIFIED"
                return False
            space = self.takeoff.space
        elif not space.assumptions:
            self.planner.assume_free(state.position, assumption_radius, "explicit-start-pad")
        space.add(frame)
        approved = self.planner.gate.validate(hold, seed, None, space, (), now)
        if not isinstance(approved, ApprovedPlan):
            self.last_error = ",".join(approved.codes)
            return False
        self.pending = approved
        if self.takeoff is not None:
            self.takeoff.approvals.append(approved)
        self.last_error = ""
        return True

    def update_takeoff(self, frame, state, now, armed, offboard):
        if self.takeoff is not None:
            self.takeoff.observe(self, state, frame.stamp, now, armed, offboard)
            parent = None if self.current is None else self.current.plan_hash
            token = self.takeoff.propose(self, frame, state, now, armed, offboard)
            if token is not None:
                self.takeoff.accept(self, token, parent, now, armed, offboard)

    def observation(self, frame, state, goal, now) -> Observation | None:
        if (
            self.current is None
            or self.pending is not None
            or self.scheduled_branch is not None
            or self.clock_fault
            or (self.takeoff is not None and not self.takeoff.ready)
        ):
            return None
        return Observation(
            frame,
            state,
            np.asarray(goal, float),
            self.current.plan,
            (),
            now,
            active_branch=self.active_branch,
        )

    def accept(self, decision: Decision, parent_hash: str, now: float) -> bool:
        self.last_error = ""
        if self.clock_fault or self.current is None or self.current.plan_hash != parent_hash:
            self.last_error = "PARENT_CHANGED"
            return False
        if decision.approved is not None:
            token = decision.approved
            if token.plan.parent_plan_hash != parent_hash or self.pending is not None:
                self.last_error = "PARENT_CHANGED"
                return False
            if token.plan.t_switch <= now:
                self.last_error = "MISSED_SWITCH"
                return False
            self.pending = token
            return True
        fb = decision.fallback
        if fb and fb.plan_hash == parent_hash and fb.kind == "BRAKE_BRANCH":
            if fb.t_branch is None or fb.t_branch < now:
                self.last_error = "MISSED_BRANCH"
                return False
            self.scheduled_branch = next(
                i for i, b in enumerate(self.current.plan.branches) if b.t_branch == fb.t_branch
            )
        return False

    def command(
        self,
        now: float,
        yaw_enu: float | None = None,
        goal_w: np.ndarray | None = None,
        position_w: np.ndarray | None = None,
    ) -> Command | None:
        """只有前进的仿真时间能生成新命令。回拨锁存故障，不自动复用旧批准。

        给出 goal_w 时按上游 `control_pub` 计算航向（#34 / Y1，`yaw.calculate_yaw`）：
        运动段用 (期望速度, goal − 期望位置)；执行制动分支时用上游 brake 形式
        (goal − 当前位置, 同上)，position_w 缺省时退回期望位置。yaw_enu 只作首条命令的
        last_yaw 种子（上游取第一帧里程计航向）。dt 取相邻命令的仿真时间差，首条取 CTRL_DT。
        不给 goal_w 时保持旧行为：yaw_enu 原样下发、不控 yawspeed。
        """
        if not math.isfinite(now) or self.clock_fault:
            return None
        prev_clock = self.last_clock
        if self.last_clock is not None:
            if now < self.last_clock:
                self.clock_fault = True
                self.last_error = "CLOCK_RESET"
                return None
            if now == self.last_clock:
                return None
        self.last_clock = now
        if self.pending is not None and now >= self.pending.plan.t_switch:
            self.current, self.pending = self.pending, None
            self.active_branch = self.scheduled_branch = None
        if self.current is None:
            return None
        if (
            self.scheduled_branch is not None
            and now >= self.current.plan.branches[self.scheduled_branch].t_branch
        ):
            self.active_branch, self.scheduled_branch = self.scheduled_branch, None
        pva = state_at(self.current.plan, now, self.active_branch)
        stamp_us = int(now * 1e6)
        yaw, yaw_dot = yaw_enu, None
        if goal_w is not None:
            seed = self.last_yaw if self.last_yaw is not None else yaw_enu
            if seed is not None:
                goal = np.asarray(goal_w, dtype=float)
                dt = CTRL_DT if prev_clock is None else now - prev_clock
                if self.active_branch is None:
                    yaw, yaw_dot = calculate_yaw(pva[1], goal - pva[0], seed, dt)
                else:
                    here = pva[0] if position_w is None else np.asarray(position_w, dtype=float)
                    gd = goal - here
                    yaw, yaw_dot = calculate_yaw(gd, gd, seed, dt)
                yaw, yaw_dot = float(yaw), float(yaw_dot)
                self.last_yaw = yaw
        return Command(
            px4.trajectory_setpoint(pva, yaw, stamp_us, yaw_dot),
            px4.offboard_control_mode(stamp_us),
            pva,
            self.current.plan_hash,
        )
