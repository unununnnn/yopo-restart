#!/usr/bin/env python3
"""票 E-SO3：L0 SO3 四旋翼滞后 vs PointMassLag。只报数。

不改 ``yopo_l0/dynamics.py``。``SO3LagModel`` 只在本进程里挂到
``yopo_l0.dynamics.MODELS["so3_lag"]``，场景卡只在内存里换 ``dynamics``。
默认仍是 ``PointMassLag``（``tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0``）。

SO3 两臂都标「report-only / 偏离默认动力学（L0 点质量不变）」。
规划保持场景卡默认：``upstream``，影子盾开。穿林和柱群仍由 ``CrossingFleet``
走上游 YOPO。不进 ``scenes/suite.txt``，不设 gates，不改已冻门。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-so3-dynamics-gap" / "partial"
JSON_PATH = ROOT / "docs" / "research" / "e-so3-dynamics-gap-runs.json"
MD_PATH = ROOT / "docs" / "research" / "e-so3-dynamics-gap.md"

# 模型方程版本。改积分或误差定义时一起改，旧的 partial 就不会被续跑拿来用。
MODEL_REV = "so3-lee-zeta707-2026-11-13"
ROW_VERSION = 1

# PX4 源码里的 CONSTANTS_ONE_G。
G = 9.80665
EZ = np.array([0.0, 0.0, 1.0], dtype=np.float64)
# 二阶姿态环阻尼。1/sqrt(2)。
ZETA = 1.0 / math.sqrt(2.0)
# docs.px4.io/main（2026-11-13）Multicopter Attitude Control。
ROLL_RATE_MAX_DEG_S = 220.0
PITCH_RATE_MAX_DEG_S = 220.0
YAW_RATE_MAX_DEG_S = 200.0
# 同一页 MC_YAW_WEIGHT。只乘在姿态误差的偏航分量上。
YAW_WEIGHT = 0.4

DISCLAIMER = (
    "只报数。report-only。SO3 两臂是偏离默认动力学（L0 点质量不变）。"
    "不改 yopo_l0/ 的默认动力学，不改 core/、scenes/、docs/tickets.md、README.md、"
    "scripts/premerge.sh、scripts/regress.sh。"
    "规划与策略保持场景卡默认：upstream（柱群和穿林由 CrossingFleet 走上游 YOPO），影子盾开。"
    "外环 kp/kd/a_max 与 PointMassLag 相同。τ_att、τ_thrust 和倾角是票面给定的假设，"
    "不是本仓库的 SITL 辨识。未冻结，不进 scenes/suite.txt，不设 gates，不改已冻门。"
)

POINT_MASS = {
    "model": "point_mass_lag",
    "tau_s": 0.1,
    "kp": 4.0,
    "kd": 3.0,
    "a_max": 6.0,
}

# 外环与点质量相同，差距只来自推力方向、倾角限幅、姿态时间常数和推力滞后。
_SO3_OUTER = {
    "kp": 4.0,
    "kd": 3.0,
    "a_max": 6.0,
    "attitude_order": 2,
    "yaw_weight": YAW_WEIGHT,
}

ARM_DYNAMICS = {
    "so3_nominal": {
        "model": "so3_lag",
        "tau_att_s": 0.12,
        "tau_thrust_s": 0.06,
        "max_tilt_deg": 35.0,
        **_SO3_OUTER,
    },
    "so3_slow": {
        "model": "so3_lag",
        "tau_att_s": 0.22,
        "tau_thrust_s": 0.10,
        "max_tilt_deg": 30.0,
        **_SO3_OUTER,
    },
}

ARMS = ("point_mass", "so3_nominal", "so3_slow")
SCENE_ORDER = (
    "g2-pillar",
    "crossing-pillars",
    "crossing-forest",
    "crossing-6-forest-sparse",
)
SCENES = {
    "g2-pillar": {
        "path": "scenes/g2-pillar.json",
        "seeds": tuple(range(10)),
        "label": "G2 柱子",
        "milestone": "M0",
        "note": "单机 goto",
    },
    "crossing-pillars": {
        "path": "scenes/crossing-pillars.json",
        "seeds": tuple(range(5)),
        "label": "三机柱群",
        "milestone": "M2",
        "note": "柱群",
    },
    "crossing-forest": {
        "path": "scenes/crossing-forest.json",
        "seeds": tuple(range(5)),
        "label": "三机中等林带",
        "milestone": "M2",
        "note": "中等林带",
    },
    "crossing-6-forest-sparse": {
        "path": "scenes/crossing-6-forest-sparse.json",
        "seeds": tuple(range(3)),
        "label": "六机稀疏林带",
        "milestone": "M4",
        "note": "稀疏林带",
    },
}

FORBIDDEN = (
    "yopo_l0/dynamics.py",
    "docs/tickets.md",
    "README.md",
    "scripts/premerge.sh",
    "scripts/regress.sh",
)


def install_so3_model():
    """把 SO3LagModel 挂进本进程的 MODELS。不写 dynamics.py。"""
    import yopo_l0.dynamics as dynamics

    current = dynamics.MODELS.get("point_mass_lag")
    if current is not dynamics.PointMassLag:
        raise RuntimeError("point_mass_lag 被换成了别的类，本票不改默认模型")
    dynamics.MODELS["so3_lag"] = SO3LagModel
    return dynamics


def arm_deviates(arm: str) -> bool:
    if arm not in ARMS:
        raise ValueError(arm)
    return arm != "point_mass"


def arm_label(arm: str) -> str:
    labels = {
        "point_mass": "PointMassLag 默认（tau_s=0.1, kp=4.0, kd=3.0, a_max=6.0）",
        "so3_nominal": "SO3 标称 PX4 响应档（report-only / 偏离默认动力学，L0 点质量不变）",
        "so3_slow": "SO3 慢姿态档（report-only / 偏离默认动力学，L0 点质量不变）",
    }
    if arm not in labels:
        raise ValueError(arm)
    return labels[arm]


def arm_mark(arm: str) -> str:
    if arm == "point_mass":
        return "默认动力学"
    if arm not in ARMS:
        raise ValueError(arm)
    return "report-only / 偏离默认动力学"


def hover_horizontal_accel(tilt_deg: float) -> float:
    """保持高度时，倾角对应的水平加速度 g·tan(θ)。"""
    return G * math.tan(math.radians(float(tilt_deg)))


def _hat(w: np.ndarray) -> np.ndarray:
    a, b, c = float(w[0]), float(w[1]), float(w[2])
    return np.array(
        [[0.0, -c, b], [c, 0.0, -a], [-b, a, 0.0]],
        dtype=np.float64,
    )


def _exp_so3(rotvec: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-12:
        return np.eye(3) + _hat(rotvec)
    K = _hat(rotvec / theta)
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def _project_so3(R: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(R)
    out = U @ Vt
    if float(np.linalg.det(out)) < 0.0:
        U = U.copy()
        U[:, -1] *= -1.0
        out = U @ Vt
    return out


def _vee_attitude_error(Rd: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Lee, Leok, McClamroch, CDC 2010：0.5 vee(Rd^T R − R^T Rd)。"""
    M = Rd.T @ R - R.T @ Rd
    return 0.5 * np.array([M[2, 1], M[0, 2], M[1, 0]], dtype=np.float64)


def _level_R(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _Rd(b3: np.ndarray, yaw: float) -> np.ndarray:
    """推力方向 b3 加期望偏航，组成水平机体系下的目标旋转。"""
    b1c = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)
    b2 = np.cross(b3, b1c)
    n = float(np.linalg.norm(b2))
    if n < 1e-8:
        # 推力方向和期望机头几乎平行时，换一条水平参考，避免除零。
        b1c = np.array([-math.sin(yaw), math.cos(yaw), 0.0], dtype=np.float64)
        b2 = np.cross(b3, b1c)
        n = float(np.linalg.norm(b2))
        if n < 1e-8:
            raise RuntimeError("目标推力方向无法配平偏航")
    b2 = b2 / n
    b1 = np.cross(b2, b3)
    return np.column_stack((b1, b2, b3))


def _yaw_of(R: np.ndarray) -> float:
    return math.atan2(float(R[1, 0]), float(R[0, 0]))


def _as3(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64).reshape(3)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} 必须是有限的 3 维向量")
    return arr


def _positive(name: str, value, *, allow_zero: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是数字")
    out = float(value)
    if not math.isfinite(out) or (out < 0.0 if allow_zero else out <= 0.0):
        raise ValueError(f"{name} 必须为{'非负' if allow_zero else '正'}有限数")
    return out


class _Vehicle:
    __slots__ = ("R", "omega", "thrust", "p_id", "stamp_p", "stamp_v", "stamp_yaw")

    def __init__(self, R: np.ndarray, thrust: float):
        self.R = R
        self.omega = np.zeros(3, dtype=np.float64)
        self.thrust = float(thrust)
        self.p_id: int | None = None
        self.stamp_p: np.ndarray | None = None
        self.stamp_v: np.ndarray | None = None
        self.stamp_yaw: float | None = None


def _stamp_matches(veh: _Vehicle, p: np.ndarray, v: np.ndarray, yaw: float) -> bool:
    if veh.stamp_p is None or veh.stamp_v is None or veh.stamp_yaw is None:
        return False
    return (
        veh.stamp_p.shape == p.shape
        and np.array_equal(veh.stamp_p, p)
        and np.array_equal(veh.stamp_v, v)
        and float(veh.stamp_yaw) == float(yaw)
    )


class SO3LagModel:
    """几何推力矢量跟踪 + 倾角限幅 + 姿态一阶/二阶滞后 + 推力一阶滞后。

    ``step(state, plan, dt) -> State`` 与 ``PointMassLag`` 相同。姿态、角速度和
    推力不在 ``State`` 里，按返回的位置数组身份记在模型上。运行器每个 tick 用
    ``dataclasses.replace`` 只改时间，位置数组还是上一次返回的那份，所以多机
    共用一个模型也不会串状态。

    外环加速度命令与 ``PointMassLag`` 同一式，并先按 ``a_max`` 限幅::

        a_cmd = clip(a_ref + kp (p_ref − p) + kd (v_ref − v), a_max)
        f = a_cmd + g e_z

    ``f`` 的方向限制在与天向夹角 ``max_tilt_deg`` 的锥里，再作为期望推力方向。
    姿态误差用 SO(3) 的 vee 映射。``attitude_order=1`` 时角速度直接等于
    ``−e_R / τ_att``（偏航分量再乘 ``yaw_weight``）。``attitude_order=2`` 时
    自然频率 ``ω_n = 1/τ_att``、阻尼 ``ζ = 1/√2``：速率命令增益
    ``k_R = 1/(τ_att √2)``，速率本身再做一个时间常数 ``τ_att/√2`` 的一阶滞后。
    推力比力（m/s²）对期望值做时间常数 ``τ_thrust`` 的一阶滞后。平动是
    ``a = T b_3 − g e_z``，再用新加速度积分速度和位置，顺序与点质量一致。

    L0 的深度相机只读 ``State.yaw`` 的水平航向，不读倾角。倾角只进平动。
    """

    def __init__(
        self,
        tau_att_s: float = 0.12,
        tau_thrust_s: float = 0.06,
        max_tilt_deg: float = 35.0,
        a_max: float = 6.0,
        kp: float = 4.0,
        kd: float = 3.0,
        attitude_order: int = 2,
        yaw_weight: float = YAW_WEIGHT,
        g: float = G,
        roll_rate_max_deg_s: float = ROLL_RATE_MAX_DEG_S,
        pitch_rate_max_deg_s: float = PITCH_RATE_MAX_DEG_S,
        yaw_rate_max_deg_s: float = YAW_RATE_MAX_DEG_S,
    ):
        self.tau_att_s = _positive("tau_att_s", tau_att_s)
        self.tau_thrust_s = _positive("tau_thrust_s", tau_thrust_s)
        self.max_tilt_deg = _positive("max_tilt_deg", max_tilt_deg)
        if self.max_tilt_deg >= 90.0:
            raise ValueError("max_tilt_deg 必须小于 90")
        self.max_tilt_rad = math.radians(self.max_tilt_deg)
        self.a_max = _positive("a_max", a_max)
        self.kp = _positive("kp", kp)
        self.kd = _positive("kd", kd, allow_zero=True)
        if isinstance(attitude_order, bool):
            raise ValueError("attitude_order 只能是 1 或 2")
        order = int(attitude_order)
        if float(attitude_order) != float(order) or order not in (1, 2):
            raise ValueError("attitude_order 只能是 1 或 2")
        self.attitude_order = order
        self.yaw_weight = _positive("yaw_weight", yaw_weight, allow_zero=True)
        if self.yaw_weight > 1.0:
            raise ValueError("yaw_weight 必须在 0 和 1 之间")
        self.g = _positive("g", g)
        self.roll_rate_max = math.radians(_positive("roll_rate_max_deg_s", roll_rate_max_deg_s))
        self.pitch_rate_max = math.radians(_positive("pitch_rate_max_deg_s", pitch_rate_max_deg_s))
        self.yaw_rate_max = math.radians(_positive("yaw_rate_max_deg_s", yaw_rate_max_deg_s))
        # ω_n = 1/τ，ζ = 1/√2 → k_R = ω_n / (2ζ) = 1/(τ √2)，τ_rate = τ/√2。
        # 一阶则没有速率滞后，k_R = 1/τ，小角度误差按 e^{-t/τ} 衰减。
        if self.attitude_order == 1:
            self.k_R = 1.0 / self.tau_att_s
            self.tau_rate_s = None
        else:
            self.k_R = 1.0 / (self.tau_att_s * math.sqrt(2.0))
            self.tau_rate_s = self.tau_att_s / math.sqrt(2.0)
        self._vehicles: list[_Vehicle] = []
        self._by_id: dict[int, _Vehicle] = {}

    @property
    def n_vehicles(self) -> int:
        return len(self._vehicles)

    def reset(self) -> None:
        self._vehicles.clear()
        self._by_id.clear()

    def desired_specific_force(self, a_cmd: np.ndarray) -> tuple[np.ndarray, float]:
        """已限幅的加速度命令 →（期望机体 z 轴，期望比力 m/s²）。"""
        f = _as3(a_cmd, "a_cmd") + self.g * EZ
        n = float(np.linalg.norm(f))
        if n < 1e-8:
            return EZ.copy(), 0.0
        b3 = f / n
        tilt = math.acos(float(np.clip(b3[2], -1.0, 1.0)))
        if tilt > self.max_tilt_rad:
            horiz = np.array([b3[0], b3[1], 0.0], dtype=np.float64)
            hn = float(np.linalg.norm(horiz))
            if hn < 1e-10:
                b3 = EZ.copy()
            else:
                horiz /= hn
                s = math.sin(self.max_tilt_rad)
                c = math.cos(self.max_tilt_rad)
                b3 = np.array([s * horiz[0], s * horiz[1], c], dtype=np.float64)
        thrust = float(np.dot(f, b3))
        if thrust < 0.0:
            thrust = 0.0
        cap = self.g + self.a_max
        if thrust > cap:
            thrust = cap
        return b3, thrust

    def internal(self, state) -> dict:
        """上一次 ``step`` 返回的状态所对应的姿态。找不到就抛 KeyError。"""
        p = _as3(state.p, "p")
        v = _as3(state.v, "v")
        yaw = float(state.yaw)
        veh = self._find(p, v, yaw, create=False)
        if veh is None:
            raise KeyError("没有这架飞机的 SO3 状态")
        b3 = veh.R[:, 2].copy()
        tilt = math.degrees(math.acos(float(np.clip(b3[2], -1.0, 1.0))))
        return {
            "tilt_deg": float(tilt),
            "yaw_rate_rad_s": float(veh.omega[2]),
            "omega_rad_s": veh.omega.copy(),
            "thrust_mps2": float(veh.thrust),
            "b3": b3,
        }

    def step(self, s, plan, dt: float):
        from yopo_l0.dynamics import State

        if isinstance(dt, bool) or not isinstance(dt, (int, float)):
            raise ValueError("dt 必须是正数")
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt 必须是正数")
        if not math.isfinite(float(s.t)) or not math.isfinite(float(s.yaw)):
            raise ValueError("状态里有非有限的时间或偏航")
        p = _as3(s.p, "p")
        v = _as3(s.v, "v")
        yaw = float(s.yaw)
        veh = self._find(p, v, yaw, create=True)
        assert veh is not None

        p_ref, v_ref, a_ref, yaw_ref = plan.sample(float(s.t))
        p_ref = _as3(p_ref, "p_ref")
        v_ref = _as3(v_ref, "v_ref")
        a_ref = _as3(a_ref, "a_ref")
        yaw_ref = float(yaw_ref)
        if not math.isfinite(yaw_ref):
            raise ValueError("计划偏航不是有限数")
        a_cmd = a_ref + self.kp * (p_ref - p) + self.kd * (v_ref - v)
        n = float(np.linalg.norm(a_cmd))
        if n > self.a_max:
            a_cmd = a_cmd * (self.a_max / n)

        b3_des, thrust_des = self.desired_specific_force(a_cmd)
        Rd = _Rd(b3_des, yaw_ref)
        e_R = _vee_attitude_error(Rd, veh.R)
        e_R = e_R.copy()
        e_R[2] *= self.yaw_weight
        omega_cmd = -self.k_R * e_R
        limits = (self.roll_rate_max, self.pitch_rate_max, self.yaw_rate_max)
        for i, limit in enumerate(limits):
            if omega_cmd[i] > limit:
                omega_cmd[i] = limit
            elif omega_cmd[i] < -limit:
                omega_cmd[i] = -limit
        if self.attitude_order == 1:
            omega = omega_cmd
        else:
            assert self.tau_rate_s is not None
            alpha_w = 1.0 - math.exp(-dt / self.tau_rate_s)
            omega = veh.omega + alpha_w * (omega_cmd - veh.omega)
        R = _project_so3(veh.R @ _exp_so3(omega * dt))
        alpha_T = 1.0 - math.exp(-dt / self.tau_thrust_s)
        thrust = veh.thrust + alpha_T * (thrust_des - veh.thrust)
        if thrust < 0.0:
            thrust = 0.0
        cap = self.g + self.a_max
        if thrust > cap:
            thrust = cap
        b3 = R[:, 2]
        a = thrust * b3 - self.g * EZ
        v_new = v + a * dt
        p_new = p + v_new * dt
        yaw_new = _yaw_of(R)
        if not np.all(np.isfinite(p_new)) or not np.all(np.isfinite(v_new)) or not math.isfinite(yaw_new):
            raise RuntimeError("SO3 积分出现非有限数")

        veh.R = R
        veh.omega = np.asarray(omega, dtype=np.float64).copy()
        veh.thrust = float(thrust)
        self._rekey(veh, p_new, v_new, yaw_new)
        return State(float(s.t) + dt, p_new, v_new, np.asarray(a, dtype=np.float64).copy(), yaw_new)

    def _find(self, p: np.ndarray, v: np.ndarray, yaw: float, *, create: bool) -> _Vehicle | None:
        key = id(p)
        veh = self._by_id.get(key)
        if veh is not None and _stamp_matches(veh, p, v, yaw):
            return veh
        if veh is not None:
            self._by_id.pop(key, None)
            if veh.p_id == key:
                veh.p_id = None
        for other in self._vehicles:
            if _stamp_matches(other, p, v, yaw):
                if other.p_id is not None and other.p_id != key:
                    self._by_id.pop(other.p_id, None)
                other.p_id = key
                self._by_id[key] = other
                return other
        if not create:
            return None
        veh = _Vehicle(_level_R(yaw), self.g)
        veh.p_id = key
        self._vehicles.append(veh)
        self._by_id[key] = veh
        return veh

    def _rekey(self, veh: _Vehicle, p: np.ndarray, v: np.ndarray, yaw: float) -> None:
        old = veh.p_id
        if old is not None:
            self._by_id.pop(old, None)
        key = id(p)
        veh.p_id = key
        veh.stamp_p = p.copy()
        veh.stamp_v = v.copy()
        veh.stamp_yaw = float(yaw)
        self._by_id[key] = veh


def params_fingerprint() -> str:
    blob = {
        "model_rev": MODEL_REV,
        "row_version": ROW_VERSION,
        "g": G,
        "zeta": ZETA,
        "yaw_weight": YAW_WEIGHT,
        "rate_max_deg_s": [ROLL_RATE_MAX_DEG_S, PITCH_RATE_MAX_DEG_S, YAW_RATE_MAX_DEG_S],
        "point_mass": POINT_MASS,
        "arms": {arm: ARM_DYNAMICS[arm] for arm in ARMS if arm != "point_mass"},
        "scenes": {name: list(SCENES[name]["seeds"]) for name in SCENE_ORDER},
    }
    raw = json.dumps(blob, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(raw).hexdigest()


def run_config(seed: int):
    from yopo_l0.runner import RunConfig

    # 不传 safe_radius / geometric_fallback，避免盖过场景卡。
    return RunConfig(seed=int(seed), policy="upstream", shadow=True)


def load_scene(scene: str):
    from yopo_l0.scene import load_card

    if scene not in SCENES:
        raise ValueError(scene)
    return load_card(ROOT / SCENES[scene]["path"])


def build_card(scene: str, arm: str):
    from yopo_l0.scene import SceneCard

    if arm not in ARMS:
        raise ValueError(arm)
    base = load_scene(scene)
    if arm == "point_mass":
        return base
    # 浅拷贝。几何 dict 和障碍来自同一次读取，林带不会被再抽样一次。
    raw = dict(base.card)
    raw["dynamics"] = json.loads(json.dumps(ARM_DYNAMICS[arm]))
    return SceneCard(base.path, raw, base.obstacles)


def file_card_hash(scene: str) -> str:
    return load_scene(scene).hash


def assert_disk_cards_are_point_mass() -> None:
    for scene in SCENE_ORDER:
        raw = json.loads((ROOT / SCENES[scene]["path"]).read_text(encoding="utf-8"))
        dyn = raw.get("dynamics") or {}
        if dyn.get("model") != "point_mass_lag":
            raise RuntimeError(f"{scene} 的动力学不是 point_mass_lag")
        for key, expect in POINT_MASS.items():
            if dyn.get(key) != expect:
                raise RuntimeError(f"{scene} 的 dynamics.{key} 是 {dyn.get(key)!r}，文件应保持 {expect!r}")


def _jsonable(obj):
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.floating):
        value = float(obj)
        return None if not math.isfinite(value) else value
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, float):
        return None if not math.isfinite(obj) else obj
    return obj


def positions_sha256(ticks: list[dict]) -> str:
    digest = hashlib.sha256()
    for row in ticks:
        digest.update(str(row["uav"]).encode())
        digest.update(np.asarray(row["p"], dtype=np.float64).tobytes())
        digest.update(np.float64(row["t"]).tobytes())
    return digest.hexdigest()


def collision_flags(result: dict) -> dict:
    reason = str(result.get("reason") or "")
    obstacle = inter = 0
    if result.get("outcome") == "COLLISION":
        if "INTER_DRONE" in reason:
            inter = 1
        else:
            obstacle = 1
    return {
        "collision": obstacle + inter,
        "obstacle_collision": obstacle,
        "inter_drone_collision": inter,
    }


def partial_path(scene: str, arm: str, seed: int) -> Path:
    return PARTIAL / f"{scene}-{arm}-{int(seed)}.json"


def load_partial(scene: str, arm: str, seed: int) -> dict | None:
    dest = partial_path(scene, arm, seed)
    if not dest.is_file():
        return None
    row = json.loads(dest.read_text(encoding="utf-8"))
    if int(row.get("row_version", 0)) != ROW_VERSION:
        return None
    if row.get("params_fingerprint") != params_fingerprint():
        return None
    return row


def save_partial(row: dict) -> None:
    dest = partial_path(row["scene"], row["arm"], int(row["seed"]))
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(_jsonable(row), ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)


def _simulate(scene: str, arm: str, seed: int) -> dict:
    from yopo_l0.runner import run

    install_so3_model()
    file_card = load_scene(scene)
    card = build_card(scene, arm)
    if arm == "point_mass":
        if card.card["dynamics"] != file_card.card["dynamics"]:
            raise RuntimeError("点质量臂改写了场景卡里的动力学")
        if card.hash != file_card.hash:
            raise RuntimeError("点质量臂的场景卡哈希和文件不一致")
    else:
        if card.card["dynamics"]["model"] != "so3_lag":
            raise RuntimeError("SO3 臂没有换成 so3_lag")
        if card.hash == file_card.hash:
            raise RuntimeError("SO3 臂的内存卡哈希不应与冻结文件相同")
    cfg = run_config(seed)
    if cfg.safe_radius_m is not None or cfg.geometric_fallback is not None:
        raise RuntimeError("对照不覆盖 safe_radius 或 geometric_fallback")
    t0 = time.perf_counter()
    out = run(card, cfg)
    res = out.result
    if bool(res.get("shadow")) is not True:
        raise RuntimeError("影子盾应开")
    if bool(res.get("upstream_deviation")):
        raise RuntimeError(f"{scene} {arm} 被标成偏离上游，本票只换动力学")
    got_radius = res.get("safe_radius_m")
    if got_radius is None or abs(float(got_radius) - 0.05) > 1e-12:
        raise RuntimeError(f"safe_radius 读回 {got_radius}，应保持 0.05 m")
    expect_model = "point_mass_lag" if arm == "point_mass" else "so3_lag"
    if res.get("dynamics") != expect_model:
        raise RuntimeError(f"运行器动力学是 {res.get('dynamics')}，应为 {expect_model}")
    if scene == "g2-pillar":
        if res.get("policy") != "upstream":
            raise RuntimeError(f"G2 策略读回 {res.get('policy')}")
    elif res.get("policy") != "crossing":
        raise RuntimeError(f"{scene} 策略读回 {res.get('policy')}")
    body = float(card.card["criteria"]["body_radius_m"])
    surface = float(res["min_obstacle_distance_m"])
    hits = collision_flags(res)
    dyn = dict(card.card["dynamics"])
    row = {
        "row_version": ROW_VERSION,
        "params_fingerprint": params_fingerprint(),
        "model_rev": MODEL_REV,
        "scene": scene,
        "arm": arm,
        "seed": int(seed),
        "report_only": True,
        "frozen": False,
        "dynamics_deviation": arm_deviates(arm),
        "mark": arm_mark(arm),
        "dynamics": res["dynamics"],
        "dynamics_params": dyn,
        "policy": res["policy"],
        "shadow": True,
        "upstream_deviation": False,
        "safe_radius_m": float(got_radius),
        "task": res["task"],
        "milestone": SCENES[scene]["milestone"],
        "file_card_hash": file_card.hash,
        "run_card_hash": card.hash,
        "commit": res.get("commit"),
        "outcome": res["outcome"],
        "reason": str(res["reason"]),
        "arrived": int(res["arrived"]),
        "n_drones": int(res["n_drones"]),
        "arrival_rate": float(res["arrived"]) / float(res["n_drones"]),
        "arrival_time_s": res["arrival_time_s"],
        "min_obstacle_distance_m": surface,
        "min_clearance_m": surface - body,
        "body_radius_m": body,
        "min_inter_drone_distance_m": res["min_inter_drone_distance_m"],
        "formation_error_p95_m": res.get("formation_error_p95_m"),
        "sim_duration_s": float(res["sim_duration_s"]),
        "positions_sha256": positions_sha256(out.ticks),
        "wall_time_s": float(res["wall_time_s"]),
        "worker_s": time.perf_counter() - t0,
        **hits,
    }
    return _jsonable(row)


def run_one(scene: str, arm: str, seed: int, *, resume: bool) -> dict:
    if resume:
        cached = load_partial(scene, arm, seed)
        if cached is not None:
            print(f"{scene} {arm} seed={seed} resume", flush=True)
            return cached
    row = _simulate(scene, arm, seed)
    save_partial(row)
    p95 = row.get("formation_error_p95_m")
    p95_s = "—" if p95 is None else f"{float(p95):.3f}"
    arrival = row.get("arrival_time_s")
    arrival_s = "—" if arrival is None else f"{float(arrival):.2f}"
    print(
        f"{scene} {arm_label(arm)} seed={seed} {row['outcome']} "
        f"{row['arrived']}/{row['n_drones']} coll={row['collision']} "
        f"clear={float(row['min_clearance_m']):.3f} p95={p95_s} "
        f"t={arrival_s} sim={float(row['sim_duration_s']):.2f} "
        f"wall={float(row['wall_time_s']):.1f}s",
        flush=True,
    )
    return row


def _worker(spec: tuple) -> dict:
    try:
        os.nice(10)
    except OSError:
        pass
    scene, arm, seed, resume = spec
    try:
        return run_one(str(scene), str(arm), int(seed), resume=bool(resume))
    except Exception:
        print(f"FAIL {scene} {arm} seed={seed}", flush=True)
        traceback.print_exc()
        raise


def seeds_for(scene: str, only: set[int] | None) -> tuple[int, ...]:
    seeds = SCENES[scene]["seeds"]
    if only is None:
        return seeds
    return tuple(s for s in seeds if s in only)


def job_specs(scenes, arms, seeds: set[int] | None, *, resume: bool) -> list[tuple]:
    specs = []
    for arm in arms:
        if arm not in ARMS:
            raise ValueError(arm)
        for scene in scenes:
            if scene not in SCENES:
                raise ValueError(scene)
            for seed in seeds_for(scene, seeds):
                specs.append((scene, arm, int(seed), bool(resume)))
    return specs


def full_specs() -> list[tuple]:
    return job_specs(SCENE_ORDER, ARMS, None, resume=True)


def _run_group(specs: list[tuple], jobs: int) -> list[dict]:
    if jobs <= 1 or len(specs) <= 1:
        return [_worker(spec) for spec in specs]
    import multiprocessing as mp

    with mp.get_context("spawn").Pool(jobs) as pool:
        return pool.map(_worker, specs, chunksize=1)


def _group(rows: list[dict], scene: str, arm: str) -> list[dict]:
    return [r for r in rows if r["scene"] == scene and r["arm"] == arm]


def _mean(values: list[float]) -> float | None:
    if not values:
        return None
    return float(sum(values) / len(values))


def _fmt(value, nd: int = 3) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{nd}f}"


def _signed(value, nd: int = 3) -> str:
    if value is None:
        return "—"
    return f"{float(value):+.{nd}f}"


def _rng(values: list[float], nd: int = 3) -> str:
    if not values:
        return "—"
    lo, hi = min(values), max(values)
    if abs(lo - hi) < 5 * 10 ** (-(nd + 1)):
        return f"{lo:.{nd}f}"
    sep = " 至 " if lo < 0.0 or hi < 0.0 else "–"
    return f"{lo:.{nd}f}{sep}{hi:.{nd}f}"


def aggregate(group: list[dict]) -> dict:
    clearances = [float(r["min_clearance_m"]) for r in group]
    p95 = [float(r["formation_error_p95_m"]) for r in group if r.get("formation_error_p95_m") is not None]
    times = [float(r["arrival_time_s"]) for r in group if r.get("arrival_time_s") is not None]
    slots = sum(int(r["n_drones"]) for r in group)
    arrived = sum(int(r["arrived"]) for r in group)
    return {
        "n": len(group),
        "slots": slots,
        "arrived": arrived,
        "collisions": sum(int(r["collision"]) for r in group),
        "obstacle": sum(int(r["obstacle_collision"]) for r in group),
        "inter": sum(int(r["inter_drone_collision"]) for r in group),
        "timeouts": sum(1 for r in group if r.get("outcome") == "TIMEOUT"),
        "min_clearance_m": None if not clearances else float(min(clearances)),
        "max_clearance_m": None if not clearances else float(max(clearances)),
        "p95_min_m": None if not p95 else float(min(p95)),
        "p95_max_m": None if not p95 else float(max(p95)),
        "mean_arrival_time_s": _mean(times),
        "n_arrival_times": len(times),
        "clearances": clearances,
        "p95s": p95,
        "times": times,
    }


def _hash_diff(rows: list[dict], scene: str, arm: str) -> int | None:
    base = {int(r["seed"]): r.get("positions_sha256") for r in _group(rows, scene, "point_mass")}
    other = _group(rows, scene, arm)
    if not base or not other:
        return None
    return sum(1 for r in other if base.get(int(r["seed"])) != r.get("positions_sha256"))


def render_summary(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 里程碑 | 臂 | 标记 | 到达率 | 碰撞 | 最小净空 m | 队形误差 p95 m | 平均到达时间 s |",
        "|---|---|---|---|---:|---:|---|---|---:|",
    ]
    for scene in SCENE_ORDER:
        for arm in ARMS:
            group = _group(rows, scene, arm)
            if not group:
                continue
            stats = aggregate(group)
            coll = f"{stats['collisions']}（障碍 {stats['obstacle']}，机间 {stats['inter']}）"
            n_time = stats["n_arrival_times"]
            if stats["mean_arrival_time_s"] is None:
                timing = "—"
            else:
                timing = f"{stats['mean_arrival_time_s']:.2f}（{n_time}/{stats['n']} 次有全体到达）"
            lines.append(
                f"| {SCENES[scene]['label']} | {SCENES[scene]['milestone']} | {arm_label(arm)} | "
                f"{arm_mark(arm)} | {stats['arrived']}/{stats['slots']} | {coll} | "
                f"{_rng(stats['clearances'])} | {_rng(stats['p95s'])} | {timing} |"
            )
    return "\n".join(lines)


def render_gap(rows: list[dict]) -> str:
    lines = [
        "| 场景 | SO3 臂 | 到达架次差 | 碰撞差 | 最紧净空差 m | 队形 p95 最差差 m | 平均到达时间差 s |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for scene in SCENE_ORDER:
        base = aggregate(_group(rows, scene, "point_mass")) if _group(rows, scene, "point_mass") else None
        if not base:
            continue
        for arm in ("so3_nominal", "so3_slow"):
            group = _group(rows, scene, arm)
            if not group:
                continue
            stats = aggregate(group)
            d_arr = stats["arrived"] - base["arrived"]
            d_col = stats["collisions"] - base["collisions"]
            d_clr = None
            if stats["min_clearance_m"] is not None and base["min_clearance_m"] is not None:
                d_clr = stats["min_clearance_m"] - base["min_clearance_m"]
            d_p95 = None
            if stats["p95_max_m"] is not None and base["p95_max_m"] is not None:
                d_p95 = stats["p95_max_m"] - base["p95_max_m"]
            d_t = None
            if stats["mean_arrival_time_s"] is not None and base["mean_arrival_time_s"] is not None:
                d_t = stats["mean_arrival_time_s"] - base["mean_arrival_time_s"]
            lines.append(
                f"| {SCENES[scene]['label']} | {arm_label(arm)} | {d_arr:+d} | {d_col:+d} | "
                f"{_signed(d_clr)} | {_signed(d_p95)} | {_signed(d_t, 2)} |"
            )
    return "\n".join(lines)


def render_seeds(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 臂 | seed | 结果 | 原因 | 到达 | 碰撞 | 最小净空 m | 队形 p95 m | 到达时间 s | 仿真 s |",
        "|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for scene in SCENE_ORDER:
        for arm in ARMS:
            for row in sorted(_group(rows, scene, arm), key=lambda r: int(r["seed"])):
                reason = str(row["reason"]).replace("|", "/")
                lines.append(
                    f"| {SCENES[scene]['label']} | {arm_label(arm)} | {int(row['seed'])} | "
                    f"{row['outcome']} | `{reason}` | {int(row['arrived'])}/{int(row['n_drones'])} | "
                    f"{int(row['collision'])} | {_fmt(row.get('min_clearance_m'))} | "
                    f"{_fmt(row.get('formation_error_p95_m'))} | {_fmt(row.get('arrival_time_s'), 2)} | "
                    f"{float(row['sim_duration_s']):.2f} |"
                )
    return "\n".join(lines)


def _gap_records(rows: list[dict]) -> list[dict]:
    out = []
    for scene in SCENE_ORDER:
        base_rows = _group(rows, scene, "point_mass")
        if not base_rows:
            continue
        base = aggregate(base_rows)
        for arm in ("so3_nominal", "so3_slow"):
            group = _group(rows, scene, arm)
            if not group:
                continue
            stats = aggregate(group)
            d_clr = d_p95 = d_t = None
            if stats["min_clearance_m"] is not None and base["min_clearance_m"] is not None:
                d_clr = stats["min_clearance_m"] - base["min_clearance_m"]
            if stats["p95_max_m"] is not None and base["p95_max_m"] is not None:
                d_p95 = stats["p95_max_m"] - base["p95_max_m"]
            if stats["mean_arrival_time_s"] is not None and base["mean_arrival_time_s"] is not None:
                d_t = stats["mean_arrival_time_s"] - base["mean_arrival_time_s"]
            out.append({
                "scene": scene,
                "arm": arm,
                "arrival_delta": stats["arrived"] - base["arrived"],
                "collision_delta": stats["collisions"] - base["collisions"],
                "min_clearance_delta_m": d_clr,
                "p95_worst_delta_m": d_p95,
                "mean_arrival_time_delta_s": d_t,
                "so3_arrived": stats["arrived"],
                "base_arrived": base["arrived"],
                "slots": stats["slots"],
                "so3_collisions": stats["collisions"],
                "base_collisions": base["collisions"],
                "so3_min_clearance_m": stats["min_clearance_m"],
                "base_min_clearance_m": base["min_clearance_m"],
                "hash_diff": _hash_diff(rows, scene, arm),
                "n_seeds": stats["n"],
            })
    return out


def render_conclusion(rows: list[dict]) -> str:
    gaps = _gap_records(rows)
    if not gaps:
        return "还没有可对照的行。"
    sentences = []
    for gap in gaps:
        scene = SCENES[gap["scene"]]["label"]
        arm = arm_label(gap["arm"])
        clr = "—" if gap["min_clearance_delta_m"] is None else _signed(gap["min_clearance_delta_m"])
        p95 = "—" if gap["p95_worst_delta_m"] is None else _signed(gap["p95_worst_delta_m"])
        timing = "—" if gap["mean_arrival_time_delta_s"] is None else _signed(gap["mean_arrival_time_delta_s"], 2)
        hash_diff = gap["hash_diff"]
        hash_s = "—" if hash_diff is None else f"{hash_diff}/{gap['n_seeds']}"
        sentences.append(
            f"{scene} / {arm}：到达 {gap['so3_arrived']}/{gap['slots']}，"
            f"点质量 {gap['base_arrived']}/{gap['slots']}，架次差 {gap['arrival_delta']:+d}；"
            f"碰撞 {gap['so3_collisions']}，点质量 {gap['base_collisions']}，差 {gap['collision_delta']:+d}；"
            f"最紧净空差 {clr} m；队形 p95 最差差 {p95} m；平均到达时间差 {timing} s；"
            f"位置哈希与点质量不同的种子 {hash_s}。"
        )
    arrival_drop = [g for g in gaps if g["arrival_delta"] < 0]
    new_collisions = [g for g in gaps if g["collision_delta"] > 0]
    clearances = [g["min_clearance_delta_m"] for g in gaps if g["min_clearance_delta_m"] is not None]
    times = [g["mean_arrival_time_delta_s"] for g in gaps if g["mean_arrival_time_delta_s"] is not None]
    p95s = [g["p95_worst_delta_m"] for g in gaps if g["p95_worst_delta_m"] is not None]
    lo = min(clearances) if clearances else None
    hi = max(clearances) if clearances else None
    parts = ["\n".join(f"{i}. {text}" for i, text in enumerate(sentences, start=1)), ""]
    if not arrival_drop and not new_collisions:
        parts.append(
            "两档 SO3 的到达架次和碰撞次数都与同一种子的点质量相同。"
        )
        if lo is not None and hi is not None:
            parts.append(
                f"最紧净空相对点质量的变化是 {lo:+.3f} 到 {hi:+.3f} m。"
                "负值是 SO3 更贴障碍。"
            )
            if lo >= -0.02 and hi <= 0.02:
                parts.append(
                    "变化落在 2 cm 以内。以这四张卡和这些种子衡量，点质量没有把到达率或净空算得明显更乐观。"
                )
            elif lo < -0.02:
                parts.append(
                    f"到达和碰撞没有变差，最紧净空最多紧了 {abs(lo):.3f} m。"
                    "点质量在到达率上与 SO3 对齐，在净空上松了这一截。"
                    "冻结表里的米数要重测之后才能当成 SO3 的净空。"
                )
            else:
                parts.append(
                    "这组种子上，SO3 的最紧净空没有比点质量更小。"
                )
    else:
        bits = []
        for gap in arrival_drop:
            bits.append(
                f"{SCENES[gap['scene']]['label']} / {arm_label(gap['arm'])} 少 {abs(gap['arrival_delta'])} 架次"
            )
        for gap in new_collisions:
            bits.append(
                f"{SCENES[gap['scene']]['label']} / {arm_label(gap['arm'])} 多 {gap['collision_delta']} 次碰撞"
            )
        parts.append(
            "点质量比至少一档 SO3 更乐观：" + "；".join(bits) + "。"
            "到达率或碰撞已经分开，冻结表里的点质量秒数和米数不能直接当成 SO3 的结果。"
        )
        if lo is not None and hi is not None:
            parts.append(f"最紧净空差的范围是 {lo:+.3f} 到 {hi:+.3f} m。")
    if times:
        parts.append(
            f"有全体到达时间的对照里，SO3 平均到达时间相对点质量是 {min(times):+.2f} 到 {max(times):+.2f} s。"
        )
    if p95s:
        parts.append(
            f"队形误差 p95 的最差种子相对点质量是 {min(p95s):+.3f} 到 {max(p95s):+.3f} m。"
            "单机 G2 没有队形误差。"
        )
    parts.append(
        "深度相机仍只吃水平偏航，倾角不进光线。上面的净空和到达差来自闭环里的位置、速度和偏航滞后。"
    )
    return "\n".join(parts)


def parameter_section() -> str:
    need = math.degrees(math.atan(6.0 / G))
    nom = ARM_DYNAMICS["so3_nominal"]
    slow = ARM_DYNAMICS["so3_slow"]
    lines = [
        "外环和 `PointMassLag` 用同一组数：`kp=4`、`kd=3`、`a_max=6`。",
        "PX4 当前 main 参数表（docs.px4.io/main，2026-11-13）里，`MPC_XY_P` 默认 0.95（米/秒每米），"
        "`MPC_XY_VEL_P_ACC` 默认 1.8（米/秒²每米/秒），`MPC_ACC_HOR` 默认 3 m/s²，`MPC_ACC_HOR_MAX` 默认 5 m/s²。",
        "这些数和 L0 的加速度外环不是同一个量纲。本票不把外环换成 PX4 默认，避免把增益差和滞后差混在一次对照里。",
        "",
        f"标称档：`tau_att_s={nom['tau_att_s']}`，`tau_thrust_s={nom['tau_thrust_s']}`，"
        f"`max_tilt_deg={nom['max_tilt_deg']:.0f}`，`a_max=6`，姿态阶数 {nom['attitude_order']}。",
        f"慢姿态档：`tau_att_s={slow['tau_att_s']}`，`tau_thrust_s={slow['tau_thrust_s']}`，"
        f"`max_tilt_deg={slow['max_tilt_deg']:.0f}`，`a_max=6`，姿态阶数 {slow['attitude_order']}。",
        "",
        "姿态是二阶环：自然频率 ω_n = 1/τ_att，阻尼 ζ = 1/√2。",
        "速率命令增益 k_R = 1/(τ_att √2)，速率再滞后 τ_att/√2。",
        "同一页参数表里 `MC_ROLL_P` 和 `MC_PITCH_P` 的默认是 4.0（弧度/秒每弧度）。",
        "速率环若理想，一阶读数是 1/4.0 = 0.25 s。旧版参数表常见 6.5，对应约 0.154 s。",
        "标称 0.12 s（ω_n ≈ 8.33 rad/s）比这两个倒数都快，是票面给定的小飞机标称，不是把某一个 PX4 参数取倒数。",
        "慢档 0.22 s（ω_n ≈ 4.55 rad/s）靠近当前默认 0.25 s 的一阶读数。",
        "PX4 没有单独的推力时间常数。0.06 s 和 0.10 s 是票面给定的一阶比力滞后，量级是小型多旋翼电机的几十毫秒。本仓库没有做 SITL 辨识。",
        "",
        f"`MPC_TILTMAX_AIR` 默认 45°。两档都更紧。",
        f"保持高度时，35° 的水平加速度上限是 g·tan(35°) = {hover_horizontal_accel(35):.2f} m/s²，高于 6。",
        f"30° 是 {hover_horizontal_accel(30):.2f} m/s²，低于 6。",
        f"水平加速度 6 m/s²、垂直加速度 0 需要的倾角约 {need:.2f}°。",
        "所以标称档在 a_max=6 的水平命令上不会被倾角锥截住，慢档会被截到大约 5.7 m/s²。",
        "",
        f"角速度限幅用同一页的默认：滚转/俯仰 {ROLL_RATE_MAX_DEG_S:.0f} °/s，偏航 {YAW_RATE_MAX_DEG_S:.0f} °/s。",
        f"`MC_YAW_WEIGHT` 默认 {YAW_WEIGHT}，两档 SO3 都乘在姿态误差的偏航分量上。点质量没有这个状态，它每个物理步把偏航设成计划值。",
        f"重力取 {G} m/s²。几何跟踪用 Lee, Leok, McClamroch, CDC 2010 的 SO(3) 姿态误差。",
        "平动是 a = T b_3 − g e_z。L0 深度相机只读 `State.yaw`，倾角不改变光线方向。",
    ]
    return "\n".join(lines)


def scene_table() -> str:
    lines = [
        "| 场景 | 文件 | 里程碑 | 种子 | 任务 |",
        "|---|---|---|---|---|",
    ]
    for scene in SCENE_ORDER:
        spec = SCENES[scene]
        seeds = spec["seeds"]
        span = f"{seeds[0]}–{seeds[-1]}" if len(seeds) > 1 else str(seeds[0])
        lines.append(
            f"| {spec['label']} | `{spec['path']}` | {spec['milestone']} | {span} | {spec['note']} |"
        )
    return "\n".join(lines)


def render_markdown(payload: dict) -> str:
    rows = payload["rows"]
    commit = payload.get("commit") or ""
    lines = [
        "# 票 E-SO3：L0 SO3 四旋翼滞后 vs 点质量，只报数",
        "",
        DISCLAIMER,
        "",
        "依据是 [SPEC](../SPEC.md) 第 47 行（动力学可换，SO3 后加）和第 73 行（接口是「给定当前状态和批准计划，推进 dt」；首版点质量加一阶加速度滞后），以及 [已知限制](../LIMITATIONS.md) 第 102 行：默认仍是点质量、`a_max=6`，换模型之后到达和净空要重测。",
        "",
        f"数字在 [e-so3-dynamics-gap-runs.json](e-so3-dynamics-gap-runs.json)。跑数 commit 是 `{commit}`。",
        "",
        "## 模型",
        "",
        parameter_section(),
        "",
        "SO3 状态（旋转、角速度、比力）记在脚本进程里，按 `step` 返回的位置数组认飞机。不写入 `State` 的新字段，也不改 `yopo_l0/dynamics.py`。注册键是 `so3_lag`。",
        "",
        "## 口径",
        "",
        "到达率按架次：该档全部 seed 的 `arrived` 之和 / `n_drones` 之和。碰撞按运行次数。仿真在第一次碰撞停下，所以一次运行最多 1。括号里分开障碍和机间。",
        "",
        "最小净空 = `min_obstacle_distance_m` − 机体半径 0.35 m。表里写该档各次的范围，左端是最紧的那一次。负值表示中心已经进了机体半径。",
        "",
        "队形误差 p95 是运行器的 `formation_error_p95_m`。多个 seed 写出最小到最大。单机 G2 没有这个字段，写 —。",
        "",
        "平均到达时间是该档里 `arrival_time_s` 非空的运行的算术平均。运行器在全体飞机都进入保持时才写这个字段，值是各机进入保持的时刻的最大值。有一机没进保持则为空，不进平均。括号里的分母是该档的运行次数。",
        "",
        "差距表是 SO3 减去点质量。最紧净空差为负，表示 SO3 更贴障碍。队形 p95 差用的是各档最差种子。平均到达时间差为正，表示 SO3 更晚。",
        "",
        "## 场景和臂",
        "",
        scene_table(),
        "",
        "三臂共用同一份场景文件。点质量臂不改 `dynamics`。SO3 只在进程内存里把 `dynamics.model` 换成 `so3_lag` 并写上该档参数。文件哈希见 JSON 的 `file_card_hash`。影子盾开，只记录。`safe_radius` 仍是 0.05 m，运行器的 `upstream_deviation` 保持 false。",
        "",
        "| 臂 | 动力学 | 标记 |",
        "|---|---|---|",
        f"| {arm_label('point_mass')} | `point_mass_lag`，τ=0.1 s | {arm_mark('point_mass')} |",
        f"| {arm_label('so3_nominal')} | `so3_lag`，τ_att=0.12 s，τ_thrust=0.06 s，倾角 35° | {arm_mark('so3_nominal')} |",
        f"| {arm_label('so3_slow')} | `so3_lag`，τ_att=0.22 s，τ_thrust=0.10 s，倾角 30° | {arm_mark('so3_slow')} |",
        "",
        "## 汇总",
        "",
        render_summary(rows),
        "",
        "## 相对点质量",
        "",
        render_gap(rows),
        "",
        "## 结论",
        "",
        render_conclusion(rows),
        "",
        "## 分种子",
        "",
        render_seeds(rows),
        "",
        "## 复现",
        "",
        "```",
        ".venv/bin/python scripts/e-so3-dynamics-gap.py --jobs 4",
        ".venv/bin/python -m pytest tests/test_e_so3_dynamics_gap.py -q",
        "python3 scripts/check_doc_links.py",
        "```",
        "",
        "`--partial` 从 `out/e-so3-dynamics-gap/partial/` 续跑已经完成的种子。缺省也续跑。`--no-resume` 忽略已有 partial。模型方程或臂参数变了，指纹对不上的 partial 不会被拿来用。",
        "",
    ]
    return "\n".join(lines)


def git_commit() -> str:
    import subprocess

    try:
        sha = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=no"],
            cwd=ROOT, capture_output=True, text=True,
        ).stdout.strip()
        return sha + ("-dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def build_payload(rows: list[dict], *, jobs: int) -> dict:
    commits = sorted({str(r.get("commit")) for r in rows if r.get("commit")})
    commit = commits[0] if len(commits) == 1 else git_commit()
    file_hashes = {}
    for scene in SCENE_ORDER:
        group = [r for r in rows if r["scene"] == scene]
        if group:
            file_hashes[scene] = group[0]["file_card_hash"]
    return {
        "disclaimer": DISCLAIMER,
        "report_only": True,
        "frozen": False,
        "commit": commit,
        "model_rev": MODEL_REV,
        "row_version": ROW_VERSION,
        "params_fingerprint": params_fingerprint(),
        "g": G,
        "zeta": ZETA,
        "yaw_weight": YAW_WEIGHT,
        "jobs": int(jobs),
        "arms": list(ARMS),
        "arm_dynamics": {"point_mass": POINT_MASS, **ARM_DYNAMICS},
        "scenes": [
            {
                "id": scene,
                "path": SCENES[scene]["path"],
                "milestone": SCENES[scene]["milestone"],
                "label": SCENES[scene]["label"],
                "seeds": list(SCENES[scene]["seeds"]),
                "file_card_hash": file_hashes.get(scene),
            }
            for scene in SCENE_ORDER
        ],
        "gaps": _gap_records(rows),
        "rows": rows,
    }


def write_docs(payload: dict) -> None:
    JSON_PATH.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n"
    tmp = JSON_PATH.with_suffix(".json.tmp")
    tmp.write_text(text, encoding="utf-8")
    tmp.replace(JSON_PATH)
    md = render_markdown(payload)
    tmp_md = MD_PATH.with_suffix(".md.tmp")
    tmp_md.write_text(md, encoding="utf-8")
    tmp_md.replace(MD_PATH)


def is_full_grid(scenes, arms, seeds: set[int] | None) -> bool:
    return (
        tuple(scenes) == SCENE_ORDER
        and tuple(arms) == ARMS
        and seeds is None
    )


def self_check() -> None:
    """开环检查：悬停、偏航滞后、倾角锥、多机不串状态。不跑场景。"""
    from dataclasses import replace

    from yopo_l0.dynamics import HoverPlan, PointMassLag, State

    install_so3_model()
    dynamics = sys.modules["yopo_l0.dynamics"]
    made = dynamics.make_model(dict(ARM_DYNAMICS["so3_nominal"]))
    if not isinstance(made, SO3LagModel):
        raise AssertionError("make_model 没有构造 SO3LagModel")
    if abs(made.tau_att_s - 0.12) > 1e-12 or made.attitude_order != 2:
        raise AssertionError("标称档参数没有进构造函数")

    # 倾角锥：水平 6 m/s² 在 35° 内原样通过，在 30° 上被截住。
    wide = SO3LagModel(max_tilt_deg=35, a_max=6, kp=4, kd=3)
    b3, thrust = wide.desired_specific_force(np.array([6.0, 0.0, 0.0]))
    tilt = math.degrees(math.acos(float(np.clip(b3[2], -1.0, 1.0))))
    expect = math.degrees(math.atan2(6.0, G))
    if abs(tilt - expect) > 1e-6:
        raise AssertionError(f"35° 档不应截住 6 m/s²，倾角 {tilt}")
    a = thrust * b3 - G * EZ
    if np.linalg.norm(a - np.array([6.0, 0.0, 0.0])) > 1e-6:
        raise AssertionError(f"未截断时加速度应回到命令，得到 {a}")
    narrow = SO3LagModel(max_tilt_deg=30, a_max=6, kp=4, kd=3)
    b3n, _thrust_n = narrow.desired_specific_force(np.array([6.0, 0.0, 0.0]))
    tilt_n = math.degrees(math.acos(float(np.clip(b3n[2], -1.0, 1.0))))
    if abs(tilt_n - 30.0) > 1e-6:
        raise AssertionError(f"30° 档应截到 30°，得到 {tilt_n}")
    if not (hover_horizontal_accel(35) > 6.0 > hover_horizontal_accel(30)):
        raise AssertionError("倾角和 a_max=6 的相对关系写反了")

    # 纯垂直命令保持直立，比力是 g+a。
    b3u, Tu = wide.desired_specific_force(np.array([0.0, 0.0, 6.0]))
    if abs(b3u[2] - 1.0) > 1e-9 or abs(Tu - (G + 6.0)) > 1e-6:
        raise AssertionError("垂直命令没有变成直立的 g+6")

    # 悬停不动。
    hover = SO3LagModel()
    s = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    plan = HoverPlan((0.0, 0.0, 0.0), 0.0)
    for _ in range(200):
        s = hover.step(s, plan, 0.01)
    if np.linalg.norm(s.p) > 1e-8 or np.linalg.norm(s.v) > 1e-8:
        raise AssertionError(f"悬停漂了 {s.p} {s.v}")

    # 点质量立刻采用计划偏航；SO3 只走出一部分，并且不把平动带偏。
    pm = PointMassLag()
    s0 = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    yaw_plan = HoverPlan((0.0, 0.0, 0.0), 0.4)
    sp = pm.step(s0, yaw_plan, 0.01)
    yaw_model = SO3LagModel(attitude_order=1, yaw_weight=1.0, tau_att_s=0.2, tau_thrust_s=0.05)
    ss = yaw_model.step(State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0), yaw_plan, 0.01)
    if abs(sp.yaw - 0.4) > 1e-12:
        raise AssertionError("点质量应立刻采用计划偏航")
    if not (0.0 < ss.yaw < 0.2):
        raise AssertionError(f"SO3 偏航应落后于计划，得到 {ss.yaw}")

    # 一阶小角度偏航，一个时间常数后误差大约剩下 1/e。
    tau = 0.25
    yaw_ref = 0.1
    first = SO3LagModel(
        attitude_order=1, yaw_weight=1.0, tau_att_s=tau, tau_thrust_s=0.05, max_tilt_deg=45,
    )
    s = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    hold = HoverPlan((0.0, 0.0, 0.0), yaw_ref)
    dt = 0.005
    for _ in range(int(round(tau / dt))):
        s = first.step(s, hold, dt)
    frac = (yaw_ref - s.yaw) / yaw_ref
    if not (0.30 < frac < 0.45):
        raise AssertionError(f"一阶偏航时间常数不对，剩余比例 {frac}")
    if np.linalg.norm(s.p) > 1e-5:
        raise AssertionError(f"纯偏航把位置带偏了 {s.p}")

    # 二阶起步比一阶慢。
    second = SO3LagModel(
        attitude_order=2, yaw_weight=1.0, tau_att_s=0.2, tau_thrust_s=0.05, max_tilt_deg=45,
    )
    a = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    b = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    one = SO3LagModel(
        attitude_order=1, yaw_weight=1.0, tau_att_s=0.2, tau_thrust_s=0.05, max_tilt_deg=45,
    )
    for _ in range(10):
        a = second.step(a, hold, 0.01)
        b = one.step(b, hold, 0.01)
    if not (0.0 < a.yaw < b.yaw):
        raise AssertionError(f"二阶起步应慢于一阶，{a.yaw} vs {b.yaw}")

    # 推力滞后：同样的向上阶跃，时间常数大的先爬得慢。
    up = HoverPlan((0.0, 0.0, 2.0), 0.0)
    fast_T = SO3LagModel(attitude_order=1, tau_att_s=0.05, tau_thrust_s=0.02, yaw_weight=1.0)
    slow_T = SO3LagModel(attitude_order=1, tau_att_s=0.05, tau_thrust_s=0.20, yaw_weight=1.0)
    sf = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    ss_ = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    for _ in range(8):
        sf = fast_T.step(sf, up, 0.01)
        ss_ = slow_T.step(ss_, up, 0.01)
    if not (sf.v[2] > ss_.v[2] > 0.0):
        raise AssertionError(f"推力滞后没有拉开垂直速度 {sf.v[2]} {ss_.v[2]}")

    # 水平阶跃：SO3 要先倾斜，早期位移小于点质量；慢档又小于标称档。
    far = HoverPlan((5.0, 0.0, 0.0), 0.0)
    pm_s = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    nom_s = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    slow_s = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    nom_m = SO3LagModel(**{k: v for k, v in ARM_DYNAMICS["so3_nominal"].items() if k != "model"})
    slow_m = SO3LagModel(**{k: v for k, v in ARM_DYNAMICS["so3_slow"].items() if k != "model"})
    pm_m = PointMassLag()
    for _ in range(40):
        pm_s = pm_m.step(pm_s, far, 0.01)
        nom_s = nom_m.step(nom_s, far, 0.01)
        slow_s = slow_m.step(slow_s, far, 0.01)
    if not (0.0 < slow_s.p[0] < nom_s.p[0] < pm_s.p[0]):
        raise AssertionError(
            f"水平滞后顺序不对：慢 {slow_s.p[0]} 标称 {nom_s.p[0]} 点质量 {pm_s.p[0]}"
        )

    # 多机 + replace 只改时间。两架飞机不能共用一套姿态。
    fleet = SO3LagModel()
    ua = State(0.0, np.array([0.0, 0.0, 0.0]), np.zeros(3), np.zeros(3), 0.0)
    ub = State(0.0, np.array([8.0, 0.0, 0.0]), np.zeros(3), np.zeros(3), 0.0)
    pa = HoverPlan((3.0, 0.0, 0.0), 0.0)
    pb = HoverPlan((8.0, 0.0, 0.0), 0.0)
    for k in range(80):
        ua = fleet.step(ua, pa, 0.01)
        ua = replace(ua, t=(k + 1) * 0.01)
        ub = fleet.step(ub, pb, 0.01)
        ub = replace(ub, t=(k + 1) * 0.01)
    if fleet.n_vehicles != 2:
        raise AssertionError(f"多机状态数是 {fleet.n_vehicles}")
    if ua.p[0] <= 0.2 or abs(ub.p[0] - 8.0) > 1e-4 or abs(ub.p[1]) > 1e-6:
        raise AssertionError(f"多机串状态：{ua.p} {ub.p}")
    info = fleet.internal(ua)
    if info["tilt_deg"] <= 1.0:
        raise AssertionError("跟踪水平目标时倾角没有起来")

    # 确定性。
    def roll(model):
        st = State(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.2)
        for _ in range(30):
            st = model.step(st, far, 0.01)
        return st

    left = roll(SO3LagModel())
    right = roll(SO3LagModel())
    if np.linalg.norm(left.p - right.p) > 0.0 or left.yaw != right.yaw:
        raise AssertionError("同输入的两次 SO3 积分不一致")


def run_check() -> int:
    install_so3_model()
    assert_disk_cards_are_point_mass()
    self_check()
    before = {path: (ROOT / path).read_bytes() for path in FORBIDDEN}
    for scene in SCENE_ORDER:
        disk = load_scene(scene)
        point = build_card(scene, "point_mass")
        if point.hash != disk.hash:
            raise RuntimeError(f"{scene} 点质量臂哈希变了")
        if point.card["dynamics"] != POINT_MASS:
            raise RuntimeError(f"{scene} 点质量参数不是默认值")
        for arm in ("so3_nominal", "so3_slow"):
            card = build_card(scene, arm)
            if card.card["dynamics"] != ARM_DYNAMICS[arm]:
                raise RuntimeError(f"{scene} {arm} 动力学没有写进内存卡")
            if card.hash == disk.hash:
                raise RuntimeError(f"{scene} {arm} 不应与文件哈希相同")
            if card.card["geometry"] != disk.card["geometry"]:
                raise RuntimeError("内存卡的几何和场景文件不一致")
            if card.card["drones"] != disk.card["drones"] or card.card["task"] != disk.card["task"]:
                raise RuntimeError("内存卡改了任务或飞机")
    after = {path: (ROOT / path).read_bytes() for path in before}
    if after != before:
        raise RuntimeError("检查过程改了禁止改动的文件")
    n = len(full_specs())
    if n != 69:
        raise RuntimeError(f"全网格应是 69 次运行，得到 {n}")
    print(f"check ok: {n} runs, fingerprint {params_fingerprint()[:12]}", flush=True)
    return 0


def _parse_seeds(text: str) -> set[int] | None:
    text = text.strip()
    if not text:
        return None
    return {int(part) for part in text.split(",") if part.strip() != ""}


def _parse_list(text: str, default: tuple[str, ...]) -> tuple[str, ...]:
    text = text.strip()
    if not text:
        return default
    return tuple(part.strip() for part in text.split(",") if part.strip())


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="E-SO3 点质量 vs SO3 滞后，只报数")
    parser.add_argument("--check", action="store_true", help="只检查模型和场景卡，不跑闭环")
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--scenes", default="", help="逗号分隔，缺省四张卡")
    parser.add_argument("--arms", default="", help="逗号分隔，缺省三臂")
    parser.add_argument("--seeds", default="", help="逗号分隔，与各卡自己的种子取交集")
    parser.add_argument("--no-resume", action="store_true")
    parser.add_argument("--no-write", action="store_true", help="跑完不写 docs/research")
    parser.add_argument("--render-only", action="store_true", help="用已有 partial 或 JSON 重写报告")
    args = parser.parse_args(argv)
    if args.jobs < 1:
        raise SystemExit("--jobs 至少为 1")
    if args.check:
        return run_check()

    scenes = _parse_list(args.scenes, SCENE_ORDER)
    arms = _parse_list(args.arms, ARMS)
    seeds = _parse_seeds(args.seeds)
    for scene in scenes:
        if scene not in SCENES:
            raise SystemExit(f"未知场景 {scene}")
    for arm in arms:
        if arm not in ARMS:
            raise SystemExit(f"未知臂 {arm}")

    if args.render_only:
        rows = []
        for scene, arm, seed, _resume in job_specs(scenes, arms, seeds, resume=True):
            cached = load_partial(scene, arm, seed)
            if cached is None:
                raise SystemExit(f"缺少 partial：{scene} {arm} seed={seed}")
            rows.append(cached)
        if not args.no_write and is_full_grid(scenes, arms, seeds):
            payload = build_payload(rows, jobs=args.jobs)
            write_docs(payload)
            print(f"wrote {MD_PATH} and {JSON_PATH}", flush=True)
        return 0

    specs = job_specs(scenes, arms, seeds, resume=not args.no_resume)
    print(f"E-SO3 runs={len(specs)} jobs={args.jobs} resume={not args.no_resume}", flush=True)
    rows = _run_group(specs, args.jobs)
    if is_full_grid(scenes, arms, seeds) and not args.no_write:
        # 续跑时本次返回的行已经是全网格。按场景、臂、种子排好再写。
        order = {(scene, arm, seed): i for i, (scene, arm, seed, _r) in enumerate(specs)}
        rows = sorted(rows, key=lambda r: order[(r["scene"], r["arm"], int(r["seed"]))])
        payload = build_payload(rows, jobs=args.jobs)
        write_docs(payload)
        print(f"wrote {MD_PATH} and {JSON_PATH}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
