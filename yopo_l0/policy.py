"""规划策略。规划器只拿 Observation：深度帧 + 本机里程计 + 目标 + 仿真时间，没有任何场景几何。

策略开关（POLICIES）：
- `hover`：票 01 悬停。
- `upstream`：上游等价（YOPO-MINCO test_yopo_ros 规则）：从期望状态出发，候选按预测走廊（0.05 m）
  准入、取最高分；无准入候选时本帧悬停、下帧照常重规划。不经任何安全门。
- `certified`：现行全部门：vendored `UavRuntime` + `Planner` + `SafetyGate`，参数同 L1 G2（g2_run.py）。
- `student`：D5 学生网络。默认不选。只有 ``--policy student``（或同等显式选择）才构造网络、读权重。
两条路径共用 vendored 的解码（decode_params / minco_two_piece / corridor_mu_lo / select_action）和推理。
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field

import numpy as np

from .backend.config import config_from_params
from .dynamics import HoverPlan, Plan, State
from .sensor import DepthFrame


@dataclass(frozen=True)
class Observation:
    frame: DepthFrame | None
    state: State  # 里程计（L0 为真值）
    goal: np.ndarray
    now: float

    def digest(self) -> str:
        h = hashlib.sha256()
        h.update(np.float64(self.now).tobytes())
        h.update(np.asarray(self.goal, dtype=np.float64).tobytes())
        s = self.state
        for x in (s.p, s.v, s.a):
            h.update(np.asarray(x, dtype=np.float64).tobytes())
        h.update(np.float64(s.yaw).tobytes())
        if self.frame is not None:
            h.update(np.float64(self.frame.stamp).tobytes())
            h.update(np.asarray(self.frame.K, dtype=np.float64).tobytes())
            h.update(np.ascontiguousarray(self.frame.depth).tobytes())
        return h.hexdigest()


@dataclass(frozen=True)
class PolicyContext:
    """策略构造参数：只含本机身份、时钟步长和场景卡里的策略参数。

    ``obstacles`` / ``a_max`` 只给后段开关。开关关掉时策略不读它们。
    """

    owner: str
    physics_dt: float
    params: dict = field(default_factory=dict)
    obstacles: tuple = ()
    a_max: float = 6.0


def R_yaw(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class HoverPolicy:
    """票 01：在目标点悬停（悬停任务里目标 = 悬停点）。"""

    name = "hover"

    def __init__(self, ctx: PolicyContext):
        self._plan: Plan | None = None
        self.last_code = "HOVER"

    def plan(self, obs: Observation) -> Plan:
        if self._plan is None:
            self._plan = HoverPlan(tuple(float(x) for x in obs.goal), obs.state.yaw)
        return self._plan


class _TrajPlan:
    """跟踪一条 PiecewisePolynomial，结束后在终点静止；偏航按上游 control_pub 用 calculate_yaw 逐 tick 推进。"""

    def __init__(self, traj, goal, yaw0: float, dt: float, brake: bool = False):
        self.traj, self.goal, self.yaw, self.dt, self.brake = traj, np.asarray(goal, float), yaw0, dt, brake

    def pva(self, t: float):
        tr = self.traj
        tt = min(max(t, tr.t_start), tr.t_end)
        p, v, a = (np.asarray(tr.evaluate(tt, k), dtype=float) for k in range(3))
        if t > tr.t_end:
            v, a = np.zeros(3), np.zeros(3)
        return p, v, a

    def sample(self, t: float):
        from yopo_swarm_core.yaw import calculate_yaw

        p, v, a = self.pva(t)
        gd = self.goal - p
        vel_dir = gd if self.brake else v
        self.yaw = float(calculate_yaw(vel_dir, gd, self.yaw, self.dt)[0])
        return p, v, a, self.yaw


# 上游 test_yopo_ros.py 的 safe_radius。非这个值就是偏离 d5e1e60，结果表要标出来。
UPSTREAM_SAFE_RADIUS_M = 0.05
_EMPTY_SOLUTION = ("CORRIDOR_EMPTY_HOVER", "BRAKE_ONLY", "CONFLICT_BRAKE")


def upstream_decoder(safe_radius: float | None = None):
    """复制解码器只改准入半径。默认返回原来的 UPSTREAM_DECODER，金样比较不受影响。"""
    from dataclasses import replace

    from yopo_swarm_core.candidates import UPSTREAM_DECODER

    radius = UPSTREAM_SAFE_RADIUS_M if safe_radius is None else float(safe_radius)
    if not radius > 0.0:
        raise ValueError(f"safe_radius 必须为正，收到 {safe_radius}")
    if abs(radius - float(UPSTREAM_DECODER.safe_radius)) <= 1e-12:
        return UPSTREAM_DECODER
    return replace(UPSTREAM_DECODER, safe_radius=radius)


def read_safe_radius(params: dict | None) -> float:
    raw = ((params or {}).get("upstream") or {}).get("safe_radius", UPSTREAM_SAFE_RADIUS_M)
    radius = float(raw)
    if not radius > 0.0:
        raise ValueError(f"safe_radius 必须为正，收到 {raw}")
    return radius


def deviates_from_upstream(radius: float) -> bool:
    return abs(float(radius) - UPSTREAM_SAFE_RADIUS_M) > 1e-9


def empty_solution_count(plan_codes: dict | None) -> int:
    """无解后的悬停或刹车次数。不含正常的 OK / YOPO_ALT。"""
    codes = plan_codes or {}
    return sum(int(codes.get(name, 0)) for name in _EMPTY_SOLUTION)


class UpstreamPolicy:
    """参数（场景卡 policy_params.upstream）：
    - plan_from_reference（默认 True，上游注释「用位置控制器飞时设 True」；L0/L1 都跟踪位置设定点）
    - safe_radius：走廊准入半径，默认 0.05 m，与上游 d5e1e60 一致。0.35 m 是票 07d 的可选修法。
    - arrive_radius_m / arrive_mode：上游进入到达半径（上游 5 m）后停止重规划，`hold_here` 原地保持（上游原样）。
      上游网络不会在目标点减速停住（训练目标是远处目标，goal 截断到 10 m），按 G2 oracle 0.4 m / 0.2 m/s / 1 s
      判到达时几乎总是绕圈超时，所以加一条项目终端规则 `stop_at_goal`：进入半径后用一段五次多项式从期望状态
      停到目标点。默认 stop_at_goal + 1.5 m（待 frank 确认；hold_here 的数字一并报告）。
    """

    name = "upstream"
    DEFAULTS = {
        "plan_from_reference": True,
        "arrive_radius_m": 1.5,
        "arrive_mode": "stop_at_goal",
        "stop_time_s": 2.0,
        "safe_radius": UPSTREAM_SAFE_RADIUS_M,
    }

    def __init__(self, ctx: PolicyContext):
        from .net import InProcessSource

        self.ctx = ctx
        self.p = {**self.DEFAULTS, **ctx.params.get("upstream", {})}
        self.source = InProcessSource()
        self.plan_obj: _TrajPlan | None = None
        self.arrived = False
        self.last_code = ""
        # 缺省关。关掉时不读障碍，也不改这条多项式。
        self._backend = config_from_params(ctx.params, a_max=float(ctx.a_max))
        self.backend_log = [] if self._backend is not None else None
        self._obstacles = tuple(ctx.obstacles) if self._backend is not None else ()
        # 默认关。关时 _inter 为 None，plan() 不读邻机、不改 select_action 的结果。
        from .inter_drone import inter_drone_config

        self._inter = inter_drone_config(ctx.params)
        self._neighbors: dict[str, np.ndarray] = {}
        self._inter_filtered = 0
        self._inter_penalized = 0
        self._inter_empty = 0

    def plan(self, obs: Observation) -> Plan:
        from yopo_swarm_core.candidates import (
            corridor_mu_lo,
            decode_params,
            minco_two_piece,
            select_action,
        )
        from yopo_swarm_core.plan import hold_traj

        dec = upstream_decoder(self.p["safe_radius"])

        s, now = obs.state, obs.now
        yaw = s.yaw if self.plan_obj is None else self.plan_obj.yaw
        if self.arrived or np.linalg.norm(s.p - obs.goal) < self.p["arrive_radius_m"]:
            # 上游 arrive：不再重规划，保持（只在第一次到达时锁定保持点）
            if not self.arrived:
                self.arrived = True
                if self.p["arrive_mode"] == "hold_here":
                    traj = hold_traj(s.p, now, now + 1e-3)
                else:
                    from yopo_swarm_core.plan import quintic
                    from yopo_swarm_core.trajectory import PiecewisePolynomial

                    head = np.array(self.plan_obj.pva(now)) if self.plan_obj else np.array([s.p, s.v, np.zeros(3)])
                    end = np.array([np.asarray(obs.goal, float), np.zeros(3), np.zeros(3)])
                    traj = PiecewisePolynomial(now, [quintic(head, end, float(self.p["stop_time_s"]))])
                self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=True)
            self.last_code = "ARRIVED_HOLD"
            return self.plan_obj
        if self.plan_obj is None:  # 上游：首帧用里程计初始化期望状态
            head = np.array([s.p, s.v, np.zeros(3)])
        else:
            head = np.array(self.plan_obj.pva(now))
            if not self.p["plan_from_reference"]:
                head[0], head[1] = s.p, s.v
        R = R_yaw(s.yaw)  # 上游用里程计姿态做机体系
        params, score, radius = self.source.forward_raw(obs.frame.depth, head[1], head[2], obs.goal, head[0], R)
        inner_b, tail_b, durations = decode_params(params, dec)
        inner_w = inner_b @ R.T + head[0]
        tail_w = np.einsum("ij,nkj->nki", R, tail_b)
        tail_w[:, 0] += head[0]
        action, brake = select_action(score, inner_w, corridor_mu_lo(radius, dec), dec.safe_mu)
        if not brake and self._inter is not None and self._neighbors:
            action, brake = self._apply_inter(score, radius, dec, now, head, tail_w, inner_w, durations, action, brake)
        if brake:
            # SPEC：无准入候选时本帧悬停（停在当前位置），下帧照常重规划
            self.last_code = "CORRIDOR_EMPTY_HOVER"
            traj = hold_traj(s.p, now, now + 1e-3)
        else:
            self.last_code = "OK"
            traj = minco_two_piece(now, head, tail_w[action], inner_w[action], durations[action])
            if self._backend is not None:
                traj = self._apply_backend(traj, obs.goal)
        self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=brake)
        return self.plan_obj

    def _apply_backend(self, traj, goal):
        from .backend.bspline import refine_yopo

        out, rec = refine_yopo(traj, self._backend, self._obstacles, goal)
        self.backend_log.append(rec)
        return out

    def note_neighbor_samples(self, src: str, samples) -> None:
        """总线上邻机已提交轨迹的采样。开关关闭时丢掉，不进状态。"""
        if self._inter is None:
            return
        from .inter_drone import as_sample_array

        arr = as_sample_array(samples)
        if arr is not None:
            self._neighbors[str(src)] = arr

    def inter_drone_report(self) -> dict | None:
        if self._inter is None:
            return None
        from .inter_drone import report_block

        return report_block(self._inter, self._inter_filtered, self._inter_penalized, self._inter_empty)

    def _apply_inter(self, score, radius, dec, now, head, tail_w, inner_w, durations, action, brake):
        """阈值内的候选过滤或减分。没有可行候选时回到原来的悬停分支。"""
        from yopo_swarm_core.candidates import corridor_mu_lo, minco_two_piece

        from .inter_drone import clearance_m, inter_pick, sample_motion

        mu = corridor_mu_lo(radius, dec)
        admitted = np.asarray(mu, dtype=np.float64) >= np.float64(dec.safe_mu)
        clear = np.full(int(dec.count), np.inf)
        neighbors = list(self._neighbors.values())
        for i in np.flatnonzero(admitted):
            try:
                traj_i = minco_two_piece(now, head, tail_w[i], inner_w[i], durations[i])
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                continue
            clear[i] = clearance_m(sample_motion(traj_i, now), neighbors)
        pick = inter_pick(score, admitted, clear, self._inter)
        if pick.kind == "unchanged":
            return action, brake
        if pick.kind == "empty":
            # 和走廊为空同一条悬停。不在这里造一条绕开邻机的曲线。
            self._inter_empty += 1
            self._inter_filtered += pick.filtered
            return action, True
        self._inter_filtered += pick.filtered
        self._inter_penalized += pick.penalized
        return int(pick.action), False


class _RuntimePlan:
    """certified：每个物理 tick 向 UavRuntime 取命令（与 L1 的 TrajectorySetpoint 同语义）。"""

    def __init__(self, policy: CertifiedPolicy, hold: np.ndarray, yaw0: float):
        self.policy, self.hold, self.yaw0 = policy, np.asarray(hold, float), yaw0
        self.last = (self.hold, np.zeros(3), np.zeros(3), yaw0)

    def sample(self, t: float):
        rt = self.policy.runtime
        cmd = rt.command(t, self.yaw0, goal_w=self.policy.goal)
        if cmd is not None:
            p, v, a = (np.asarray(x, dtype=float) for x in cmd.pva_enu)
            self.last = (p, v, a, float(rt.last_yaw if rt.last_yaw is not None else self.yaw0))
        elif rt.current is None:
            self.last = (self.hold, np.zeros(3), np.zeros(3), self.yaw0)  # 尚无批准：原地保持
        return self.last


class CertifiedPolicy:
    name = "certified"
    DEFAULTS = {"body_radius": 0.35, "tracking_margin": 0.15, "switch_delay": 2.0, "start_pad_radius": 1.2}

    def __init__(self, ctx: PolicyContext):
        from yopo_swarm_core.planner import Planner
        from yopo_swarm_core.uav import UavRuntime, default_config

        from .net import InProcessSource

        p = {**self.DEFAULTS, **ctx.params.get("certified", {})}
        self.pad = float(p["start_pad_radius"])
        self.cfg = default_config(ctx.owner, p["body_radius"], p["tracking_margin"], p["switch_delay"])
        self.source = InProcessSource()
        self.planner = Planner(self.cfg, self.source)
        self.runtime = UavRuntime(self.planner)
        self.goal: np.ndarray | None = None
        self.plan_obj: _RuntimePlan | None = None
        self.last_code = ""
        self.extra_codes: dict[str, int] = {}

    def plan(self, obs: Observation) -> Plan:
        from yopo_swarm_core.depth import DepthSample, optical_pose
        from yopo_swarm_core.freespace import CameraIntrinsics
        from yopo_swarm_core.planner import VehicleState

        s, now, rt = obs.state, obs.now, self.runtime
        self.goal = np.asarray(obs.goal, float)
        if self.plan_obj is None:
            self.plan_obj = _RuntimePlan(self, s.p, s.yaw)
        f = obs.frame
        h, w = f.depth.shape
        R = R_yaw(s.yaw)
        k = CameraIntrinsics(*f.K, width=w, height=h)
        raw = f.depth.astype(np.float32)
        frame = DepthSample(raw, k, optical_pose(s.p, R), int(round(f.stamp * 1e9))).evidence(self.cfg.space)
        state = VehicleState(s.p.copy(), s.v.copy(), s.a.copy(), R, s.t)
        if rt.current is None:
            if rt.pending is None:
                ok = rt.bootstrap(frame, state, now, self.pad)
                self.last_code = "BOOTSTRAP" if ok else f"BOOTSTRAP_FAIL:{rt.last_error}"
            else:
                self.last_code = "WAIT_SWITCH"
            return self.plan_obj
        o = rt.observation(frame, state, self.goal, now)
        if o is None:
            self.last_code = "WAIT_SWITCH"
            return self.plan_obj
        self.source.bind_depth(raw, frame.stamp)
        parent = o.approved.plan_hash(self.cfg.plan)
        d = self.planner.step(o)
        rt.accept(d, parent, now)
        self.last_code = d.diagnostic.value
        for c in d.candidates:  # 候选漏斗：每个被拒候选的原因
            key = "cand:" + c.code
            self.extra_codes[key] = self.extra_codes.get(key, 0) + 1
        return self.plan_obj



class StudentPolicy:
    """D5：用多头学生网络给出一条连续轨迹。只有显式选 ``student`` 时才会构造。

    参数（policy_params.student，或环境变量 YOPO_STUDENT_WEIGHTS）：
    - weights：检查点路径，仓库外。缺了就拒绝构造。
    - max_range_m：深度归一化，默认 20.0，和教师转换一致。
    - arrive_radius_m / arrive_mode / stop_time_s：与上游相同的终端规则，不是网络学出来的。
    """

    name = "student"
    DEFAULTS = {
        "weights": None,
        "max_range_m": 20.0,
        "arrive_radius_m": 1.5,
        "arrive_mode": "stop_at_goal",
        "stop_time_s": 2.0,
    }

    def __init__(self, ctx: PolicyContext):
        import os

        self.ctx = ctx
        self.p = {**self.DEFAULTS, **ctx.params.get("student", {})}
        weights = self.p.get("weights") or os.environ.get("YOPO_STUDENT_WEIGHTS")
        if not weights:
            raise ValueError("学生策略需要 policy_params.student.weights 或环境变量 YOPO_STUDENT_WEIGHTS")
        from yopo_l0.student import load_checkpoint

        self.model, self.meta = load_checkpoint(weights, enabled=True)
        self.plan_obj: _TrajPlan | None = None
        self.arrived = False
        self.last_code = ""
        self._neighbors: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.infer_s: list[float] = []
        self.weights_sha256 = str(self.meta.get("weights_sha256") or "")

    def note_neighbor_odom(self, src: str, odom) -> None:
        """总线上邻机当前里程计。缺字段就丢掉这一条。"""
        if not isinstance(odom, dict) or "p" not in odom or "v" not in odom:
            return
        p = np.asarray(odom["p"], dtype=np.float64).reshape(3)
        v = np.asarray(odom["v"], dtype=np.float64).reshape(3)
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(v))):
            return
        self._neighbors[str(src)] = (p.copy(), v.copy())

    def infer_report(self) -> dict | None:
        if not self.infer_s:
            return {
                "n": 0,
                "mean_ms": None,
                "p50_ms": None,
                "p95_ms": None,
                "max_ms": None,
                "weights_sha256": self.weights_sha256,
            }
        arr = np.asarray(self.infer_s, dtype=np.float64) * 1e3
        return {
            "n": int(arr.size),
            "mean_ms": float(arr.mean()),
            "p50_ms": float(np.percentile(arr, 50)),
            "p95_ms": float(np.percentile(arr, 95)),
            "max_ms": float(arr.max()),
            "weights_sha256": self.weights_sha256,
        }

    def plan(self, obs: Observation) -> Plan:
        import time as time_mod

        from yopo_swarm_core.plan import hold_traj, quintic
        from yopo_swarm_core.trajectory import PiecewisePolynomial

        from yopo_l0.student import (
            HORIZON_DT_S,
            HORIZON_STEPS,
            depth_from_hw,
            pack_ego_state,
            pack_neighbor_features,
            predict,
        )

        s, now = obs.state, obs.now
        yaw = s.yaw if self.plan_obj is None else self.plan_obj.yaw
        if self.arrived or np.linalg.norm(s.p - obs.goal) < self.p["arrive_radius_m"]:
            if not self.arrived:
                self.arrived = True
                if self.p["arrive_mode"] == "hold_here":
                    traj = hold_traj(s.p, now, now + 1e-3)
                else:
                    head = np.array(self.plan_obj.pva(now)) if self.plan_obj else np.array([s.p, s.v, np.zeros(3)])
                    end = np.array([np.asarray(obs.goal, float), np.zeros(3), np.zeros(3)])
                    traj = PiecewisePolynomial(now, [quintic(head, end, float(self.p["stop_time_s"]))])
                self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=True)
            self.last_code = "ARRIVED_HOLD"
            return self.plan_obj
        if obs.frame is None:
            self.last_code = "NO_DEPTH_HOVER"
            traj = hold_traj(s.p, now, now + 1e-3)
            self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=True)
            return self.plan_obj

        depth = depth_from_hw(obs.frame.depth, float(self.p["max_range_m"]))
        state = pack_ego_state(s.p, s.v, s.a, obs.goal, s.yaw)
        if self._neighbors:
            ids = sorted(self._neighbors)
            nb_p = np.stack([self._neighbors[i][0] for i in ids], axis=0)
            nb_v = np.stack([self._neighbors[i][1] for i in ids], axis=0)
            feat, mask = pack_neighbor_features(s.p, s.v, s.yaw, nb_p, nb_v, k=self.model.cfg.n_neighbors)
        else:
            feat, mask = pack_neighbor_features(
                s.p, s.v, s.yaw, np.zeros((0, 3)), np.zeros((0, 3)), k=self.model.cfg.n_neighbors
            )
        t0 = time_mod.perf_counter()
        pred = predict(self.model, depth, state, feat, mask)
        self.infer_s.append(time_mod.perf_counter() - t0)
        if pred is None:
            self.last_code = "STUDENT_DISABLED_HOVER"
            traj = hold_traj(s.p, now, now + 1e-3)
            self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=True)
            return self.plan_obj

        rot = R_yaw(s.yaw)
        p_b = np.asarray(pred.position_body[0], dtype=np.float64)
        v_b = np.asarray(pred.velocity_body[0], dtype=np.float64)
        if p_b.shape != (HORIZON_STEPS, 3) or v_b.shape != (HORIZON_STEPS, 3) or not (
            np.all(np.isfinite(p_b)) and np.all(np.isfinite(v_b))
        ):
            self.last_code = "STUDENT_BAD_TRAJ_HOVER"
            traj = hold_traj(s.p, now, now + 1e-3)
            self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=True)
            return self.plan_obj
        p_w = s.p + (rot @ p_b.T).T
        v_w = (rot @ v_b.T).T
        head = np.array([np.asarray(s.p, float), np.asarray(s.v, float), np.zeros(3)])
        segs = []
        prev = head
        try:
            for i in range(HORIZON_STEPS):
                end = np.array([p_w[i], v_w[i], np.zeros(3)])
                segs.append(quintic(prev, end, float(HORIZON_DT_S)))
                prev = end
            traj = PiecewisePolynomial(now, segs)
        except (ValueError, np.linalg.LinAlgError, FloatingPointError):
            self.last_code = "STUDENT_FIT_HOVER"
            traj = hold_traj(s.p, now, now + 1e-3)
            self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=True)
            return self.plan_obj
        self.last_code = "OK"
        self.plan_obj = _TrajPlan(traj, obs.goal, yaw, self.ctx.physics_dt, brake=False)
        return self.plan_obj


POLICIES = {"hover": HoverPolicy, "upstream": UpstreamPolicy, "certified": CertifiedPolicy, "student": StudentPolicy}
