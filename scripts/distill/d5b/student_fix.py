"""D5b 原型：学生策略的接口修正（仓库外，之后搬进 yopo_l0/policy.py）。

三个选项（policy_params.student）：
- knots: "network"（D5）| "position"：结点速度改由位置结点中心差分得到，和位置自洽。
- yaw_source: "calculate_yaw"（D5）| "ego_lookahead"：EGO-Swarm traj_server 的偏航，朝 1.0 s 后轨迹点，限速 π rad/s。
- accel_source: "state"（D5）| "reference"：状态里的加速度用当前计划的参考加速度（教师记录的是指令加速度）。
"""
import numpy as np
import yopo_l0.policy as P
from yopo_l0.policy import R_yaw, _TrajPlan, StudentPolicy


class EgoLookaheadPlan(_TrajPlan):
    LOOKAHEAD_S = 1.0
    YAW_RATE = np.pi

    def sample(self, t):
        p, v, a = self.pva(t)
        tr = self.traj
        tf = min(max(t + self.LOOKAHEAD_S, tr.t_start), tr.t_end)
        d = np.asarray(tr.evaluate(tf, 0), dtype=float) - p
        if float(np.linalg.norm(d)) > 0.1:
            diff = (float(np.arctan2(d[1], d[0])) - self.yaw + np.pi) % (2 * np.pi) - np.pi
            m = self.YAW_RATE * self.dt
            self.yaw = float((self.yaw + float(np.clip(diff, -m, m)) + np.pi) % (2 * np.pi) - np.pi)
        return p, v, a, self.yaw


class StudentPolicyB(StudentPolicy):
    DEFAULTS = {**StudentPolicy.DEFAULTS, "knots": "position", "yaw_source": "ego_lookahead", "accel_source": "reference"}

    def plan(self, obs):
        import time as time_mod
        from dataclasses import replace
        from yopo_swarm_core.plan import quintic
        from yopo_swarm_core.trajectory import PiecewisePolynomial
        from yopo_l0.student import HORIZON_DT_S, HORIZON_STEPS

        s, now = obs.state, obs.now
        knots = self.p["knots"]; ysrc = self.p["yaw_source"]; asrc = self.p["accel_source"]
        if asrc == "reference" and self.plan_obj is not None and not self.arrived:
            a_ref = np.asarray(self.plan_obj.pva(now)[2], dtype=float)
            obs = replace(obs, state=replace(s, a=a_ref))
        prev_plan = self.plan_obj
        out = super().plan(obs)
        if self.last_code != "OK":
            return out
        s = obs.state
        pred = self._last_pred
        rot = R_yaw(s.yaw)
        p_b = np.asarray(pred.position_body[0], dtype=float)
        p_w = s.p + (rot @ p_b.T).T
        if knots == "position":
            pts = np.vstack([np.asarray(s.p, float), p_w])
            v_w = np.empty_like(p_w)
            v_w[:-1] = (pts[2:] - pts[:-2]) / (2.0 * HORIZON_DT_S)
            v_w[-1] = (pts[-1] - pts[-2]) / HORIZON_DT_S
        else:
            v_w = (rot @ np.asarray(pred.velocity_body[0], float).T).T
        head = np.array([np.asarray(s.p, float), np.asarray(s.v, float), np.zeros(3)])
        segs = []; prev = head
        for i in range(HORIZON_STEPS):
            end = np.array([p_w[i], v_w[i], np.zeros(3)])
            segs.append(quintic(prev, end, float(HORIZON_DT_S))); prev = end
        traj = PiecewisePolynomial(now, segs)
        yaw0 = s.yaw if prev_plan is None else prev_plan.yaw
        if ysrc == "ego_lookahead":
            self.plan_obj = EgoLookaheadPlan(traj, obs.goal, yaw0, self.ctx.physics_dt, brake=False)
        else:
            self.plan_obj = _TrajPlan(traj, obs.goal, yaw0, self.ctx.physics_dt, brake=False)
        return self.plan_obj


def install():
    import yopo_l0.student as S
    orig = S.predict

    def predict(model, *a, **k):
        r = orig(model, *a, **k)
        _holder["pred"] = r
        return r
    _holder = {}
    S.predict = predict
    StudentPolicyB._last_pred = property(lambda self: _holder["pred"])
    P.POLICIES["student"] = StudentPolicyB
