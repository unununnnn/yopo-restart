"""L0 的 SO3 开关模型。默认不启用。

点质量仍是 ``yopo_l0/dynamics.py`` 的 ``PointMassLag``。本模块只在场景卡
``dynamics.model`` 写成 ``so3_lag``，或 ``RunConfig.so3 is True`` 时才被运行器调用。
关掉时运行器不导入本模块。

姿态是一阶环：角速度命令 ``ω = clip(−e_R / τ_att)``，没有 E-SO3 已报过的二阶速率滞后。
推力对比力命令做一阶滞后后再限幅，倾角限制在锥内，滚转/俯仰/偏航角速度按 PX4 默认限幅。
方程和 E-SO3 脚本里 ``attitude_order=1`` 的那一支相同，不重做那张二阶量化。

参数来源与置信度见 ``SOURCE``。外环 ``kp`` / ``kd`` / ``a_max`` 跟场景卡上的点质量走，
卡上没写就用 ``PointMassLag`` 的 4 / 3 / 6。这样打开开关只换姿态和推力通道。
"""

from __future__ import annotations

import math

import numpy as np

# PX4 源码 CONSTANTS_ONE_G。常数，置信度高。
G = 9.80665
EZ = np.array([0.0, 0.0, 1.0], dtype=np.float64)

# 打开开关、场景卡又没写 SO3 参数时用的这一组。
# attitude_order=1 是本票的一阶姿态环。E-SO3 公布的两档是 2，这里不重复那次量化。
SWITCH_DEFAULTS = {
    "model": "so3_lag",
    "tau_att_s": 0.12,
    "tau_thrust_s": 0.06,
    "max_tilt_deg": 35.0,
    "kp": 4.0,
    "kd": 3.0,
    "a_max": 6.0,
    "attitude_order": 1,
    "yaw_weight": 0.4,
    "roll_rate_max_deg_s": 220.0,
    "pitch_rate_max_deg_s": 220.0,
    "yaw_rate_max_deg_s": 200.0,
    "g": G,
}

# 每条都写来源和置信度。置信度不是拟合残差，是「这个数从哪来」。
SOURCE = {
    "tau_att_s": (
        "0.12 s，E-SO3 标称档（docs/research/e-so3-dynamics-gap.md）。"
        "票面假设，不是本仓库的 SITL 辨识，置信度低。"
        "PX4 当前 main（docs.px4.io/main，2026-11-13）MC_ROLL_P 与 MC_PITCH_P 默认 4.0，"
        "速率环若理想，一阶读数是 0.25 s。本开关不用 0.25 s，避免和已经用过的标称时间常数再换一套。"
    ),
    "tau_thrust_s": (
        "0.06 s，E-SO3 标称档的一阶比力滞后。票面假设，量级是小型多旋翼电机的几十毫秒，"
        "置信度低。PX4 没有单独的推力时间常数。"
    ),
    "max_tilt_deg": (
        "35°，E-SO3 标称档。PX4 MPC_TILTMAX_AIR 默认 45°。本开关比文档默认更紧，置信度低。"
        "保持高度时水平加速度上限 g·tan(35°)≈6.87 m/s²，高于 a_max=6，所以纯水平 6 m/s² 不会被这只锥截住。"
    ),
    "attitude_order": (
        "1。本票写明姿态环一阶滞后：ω = −e_R / τ_att，再做角速率限幅。"
        "E-SO3 已经量化过二阶（自然频率 1/τ、阻尼 1/√2）。本开关不重复那张表。"
    ),
    "yaw_weight": (
        "0.4，PX4 MC_YAW_WEIGHT 默认（docs.px4.io/main，2026-11-13）。"
        "只乘在姿态误差的偏航分量上。公开参数表，不是本机辨识，置信度中。"
    ),
    "rate_limits_deg_s": (
        "滚转/俯仰 220 °/s，偏航 200 °/s。同一页 MC_ROLLRATE_MAX、MC_PITCHRATE_MAX、MC_YAWRATE_MAX 的默认。"
        "公开参数表，不是本机辨识，置信度中。"
    ),
    "thrust_limit": (
        "比力限制在 [0, g + a_max]。这是 E-SO3 模型里的幅值帽，不是电机实测，置信度低。"
        "推力通道除此之外还有上面的一阶滞后。"
    ),
    "outer_loop": (
        "kp=4、kd=3、a_max=6 与 PointMassLag 相同，并优先沿用场景卡上的这三个数。"
        "PX4 的 MPC_XY_P / MPC_XY_VEL_P_ACC 量纲不同，本开关不把外环换成那一组，避免把增益差和滞后差混在一次对照里。"
    ),
    "g": "9.80665 m/s²，PX4 CONSTANTS_ONE_G。常数，置信度高。",
    "attitude_error": "Lee, Leok, McClamroch, CDC 2010 的 SO(3) vee 误差。几何定义，不是辨识参数。",
}

MARK = "偏离上游"


def switch_config(card_dyn: dict | None = None) -> dict:
    """场景卡仍是点质量时，打开开关用的动力学字典。不改传入的卡。"""
    out = dict(SWITCH_DEFAULTS)
    for key in ("kp", "kd", "a_max"):
        if card_dyn and key in card_dyn and card_dyn[key] is not None:
            out[key] = card_dyn[key]
    return out


def point_mass_config(card_dyn: dict | None = None) -> dict:
    """``RunConfig.so3 is False`` 且场景卡写成了 so3_lag 时，退回点质量。"""
    out = {
        "model": "point_mass_lag",
        "tau_s": 0.1,
        "kp": 4.0,
        "kd": 3.0,
        "a_max": 6.0,
    }
    for key in ("tau_s", "kp", "kd", "a_max"):
        if card_dyn and key in card_dyn and card_dyn[key] is not None:
            out[key] = card_dyn[key]
    return out


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
    b1c = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)
    b2 = np.cross(b3, b1c)
    n = float(np.linalg.norm(b2))
    if n < 1e-8:
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
    """几何推力矢量跟踪。``step`` 的签名与 ``PointMassLag.step`` 相同。

    姿态、角速度和比力不进 ``State``。运行器用 ``dataclasses.replace`` 只改时间，
    位置数组还是上一次返回的那份，所以多机共用一个模型也不会串状态。
    """

    def __init__(
        self,
        tau_att_s: float = SWITCH_DEFAULTS["tau_att_s"],
        tau_thrust_s: float = SWITCH_DEFAULTS["tau_thrust_s"],
        max_tilt_deg: float = SWITCH_DEFAULTS["max_tilt_deg"],
        a_max: float = SWITCH_DEFAULTS["a_max"],
        kp: float = SWITCH_DEFAULTS["kp"],
        kd: float = SWITCH_DEFAULTS["kd"],
        attitude_order: int = SWITCH_DEFAULTS["attitude_order"],
        yaw_weight: float = SWITCH_DEFAULTS["yaw_weight"],
        g: float = G,
        roll_rate_max_deg_s: float = SWITCH_DEFAULTS["roll_rate_max_deg_s"],
        pitch_rate_max_deg_s: float = SWITCH_DEFAULTS["pitch_rate_max_deg_s"],
        yaw_rate_max_deg_s: float = SWITCH_DEFAULTS["yaw_rate_max_deg_s"],
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
        self.roll_rate_max_deg_s = math.degrees(self.roll_rate_max)
        self.pitch_rate_max_deg_s = math.degrees(self.pitch_rate_max)
        self.yaw_rate_max_deg_s = math.degrees(self.yaw_rate_max)
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

    def desired_specific_force(self, a_cmd: np.ndarray) -> tuple[np.ndarray, float]:
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


def make_so3(cfg: dict) -> SO3LagModel:
    """注册 ``so3_lag`` 再交给 ``make_model``。只在开关打开时调用。"""
    from .dynamics import MODELS, make_model

    MODELS["so3_lag"] = SO3LagModel
    return make_model(dict(cfg))


def public_record(cfg: dict) -> dict:
    """写进结果记录的参数。只在开关打开时出现。"""
    model = SO3LagModel(**{k: v for k, v in cfg.items() if k != "model"})
    return {
        "model": "so3_lag",
        "attitude_order": model.attitude_order,
        "tau_att_s": model.tau_att_s,
        "tau_thrust_s": model.tau_thrust_s,
        "max_tilt_deg": model.max_tilt_deg,
        "kp": model.kp,
        "kd": model.kd,
        "a_max": model.a_max,
        "yaw_weight": model.yaw_weight,
        "roll_rate_max_deg_s": model.roll_rate_max_deg_s,
        "pitch_rate_max_deg_s": model.pitch_rate_max_deg_s,
        "yaw_rate_max_deg_s": model.yaw_rate_max_deg_s,
        "g": model.g,
        "mark": MARK,
    }
