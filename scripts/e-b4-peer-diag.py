#!/usr/bin/env python3
"""票 E-B4：机间碰撞只读诊断。

复现一粒对向运行，记下碰撞前窗口里每一个物理步的数据，用来逐个验证 H1–H4：

- 哪层出的指令（规划码、后段是否生效 / 被预算截断）；
- 各层看到的邻机距离与门槛（相机、影子盾、E-A1、D1、E-B3）；
- 邻机计划的年龄、预测分离与实际分离；
- 跟踪误差（真值位置与批准参考位置之差）。

只读：不改运行器、策略、后段或场景卡。进程内做三件事，跑完全部还原：

1. 把 ``POLICIES["upstream"]`` 换成一个只记日志的子类，选择仍走原来的 ``plan()``；
2. 把 ``runner._note_pairs`` 换成「记首次越线、不立刻停」的版本，越线后多飞
   ``--post-s`` 秒再停，用来看不截断时两机会不会穿过去。越线之前轨迹与原运行器相同；
3. 不往总线、Observation、动力学里写任何东西。

臂：``off`` 全关；``b2`` 只开后段；``b3`` 后段 + 机间椭球（只有当前代码带
``swarm_penalty`` 字段时可用）；``a1`` 只开 E-A1（阈值 0.70，过滤），作为 H1 的旁证；
``fix`` 见 ``--fix-arm``（默认无）。

用法：

    .venv/bin/python scripts/e-b4-peer-diag.py --scene swap8 --arm off --seed 0 --out out/e-b4/swap8-off-0.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
# --code-root：用另一份检出里的 yopo_l0（例如还没合入的 E-B3 worktree）跑同一套诊断。
if "--code-root" in sys.argv:
    ROOT = Path(sys.argv[sys.argv.index("--code-root") + 1]).resolve()
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from yopo_l0.experiment import json_text

SCENES = {
    "swap8": "scenes/swap-8-open.json",
    "d014-swap3": "scenes/ego-density-014-swap3.json",
}
ARMS = ("off", "b2", "b3", "a1")
BODY_R = 0.35
COLLIDE_M = 2.0 * BODY_R
# 与 E-B3 对照脚本同一组假设（不是论文复现）。
B3 = {"enabled": True, "swarm_penalty": True, "swarm_clearance_m": 0.80, "swarm_c": 2.0}
# 与 scripts/e-c4-density-l0.py、e-combo-a1-d1-b2.py 同一组 E-A1 字段。
A1 = {"inter_drone_filter": True, "inter_drone_threshold_m": 0.70, "inter_drone_penalty": 0}
SAMPLE_DT = 0.02
HORIZON = 4.0


def _deep(x):
    return json.loads(json.dumps(x))


def build_card(scene: str, arm: str, extra: dict | None = None):
    from yopo_l0.scene import SceneCard, load_card, parse_geometry

    path = ROOT / SCENES[scene]
    if arm == "off" and not extra:
        return load_card(path)
    raw = json.loads(path.read_text(encoding="utf-8"))
    params = dict(raw.get("policy_params") or {})
    if arm in ("b2", "b3"):
        backend = dict(params.get("backend") or {})
        if arm == "b2":
            backend["enabled"] = True
        else:
            from yopo_l0.backend.config import BackendConfig

            if "swarm_penalty" not in BackendConfig.__dataclass_fields__:
                raise SystemExit("当前代码没有 E-B3 的 swarm_penalty 字段，b3 臂不可用")
            backend.update(B3)
        params["backend"] = backend
    elif arm == "a1":
        up = dict(params.get("upstream") or {})
        up.update(A1)
        params["upstream"] = up
    for key, val in (extra or {}).items():
        blk = dict(params.get(key) or {})
        blk.update(val)
        params[key] = blk
    raw["policy_params"] = params
    card = _deep(raw)
    geom = card.get("geometry")
    if isinstance(geom, str):
        card["geometry"] = json.loads((ROOT / "scenes" / geom).read_text(encoding="utf-8"))
    return SceneCard(card["name"], card, parse_geometry(card["geometry"], card.get("ground_z")))


class _Log:
    proto_brake: bool = False
    dump: list | None = None

    def __init__(self):
        self.plans: list[dict] = []  # 每次规划一条
        self.recv: list[dict] = []  # 每次收到邻机采样一条
        self.cross: dict | None = None
        self.refine: list[dict] = []  # E-B3：后段前后对邻机的最小距离与反事实
        self.stop_after: float = 1.0


def _make_policy_cls(base, log: _Log):
    class DiagUpstream(base):
        def plan(self, obs):
            n_back = len(self.backend_log) if self.backend_log is not None else 0
            prev = self.plan_obj
            out = super().plan(obs)
            if log.proto_brake and self.last_code == "CORRIDOR_EMPTY_HOVER":
                out = _proto_brake(self, prev, obs)
            back = None
            if self.backend_log is not None and len(self.backend_log) > n_back:
                rec = self.backend_log[-1]
                back = {"applied": bool(rec.get("applied")), "reason": rec.get("reason"),
                        "budget_cut": bool(rec.get("budget_cut")), "iterations": int(rec.get("iterations", 0))}
            log.plans.append({
                "uav": self.ctx.owner,
                "now": float(obs.now),
                "code": self.last_code,
                "plan": out,
                "state_p": np.asarray(obs.state.p, float).copy(),
                "state_v": np.asarray(obs.state.v, float).copy(),
                "yaw": float(obs.state.yaw),
                "frame": obs.frame,
                "backend": back,
                "neighbors_held": sorted(getattr(self, "_neighbors", {}).keys()),
            })
            return out

        def note_neighbor_samples(self, src, samples):
            arr = np.asarray(samples, dtype=float)
            log.recv.append({"dest": self.ctx.owner, "src": str(src), "t0": float(arr[0, 0]), "arr": arr})
            return super().note_neighbor_samples(src, samples)

    DiagUpstream.__name__ = "DiagUpstream"
    return DiagUpstream


def _proto_brake(policy, prev, obs):
    """修复原型（只在本脚本进程里，不改 policy.py）：无解时不把参考点拉回当前位置，
    而是从上一份参考的 (p, v, a) 按 a_max 刹停，与 D1 的 _brake_from_head 同一个 core 函数。"""
    from yopo_swarm_core.plan import KinematicLimits, brake_branch, hold_traj

    from yopo_l0.policy import _TrajPlan

    s, now = obs.state, float(obs.now)
    if prev is not None:
        head = np.array(prev.pva(now), dtype=float)
        yaw = prev.yaw
    else:
        head = np.array([s.p, s.v, np.zeros(3)], dtype=float)
        yaw = s.yaw
    traj = None
    if float(np.linalg.norm(head[1])) > 1e-6 or float(np.linalg.norm(head[2])) > 1e-6:
        traj = brake_branch(now, head, KinematicLimits(v_max=20.0, a_max=float(policy.ctx.a_max), j_max=20000.0))
    if traj is None:
        traj = hold_traj(head[0], now, now + 1e-3)
    policy.plan_obj = _TrajPlan(traj, obs.goal, yaw, policy.ctx.physics_dt, brake=True)
    return policy.plan_obj


def _traj_vs_neighbors(traj, neighbors, c=2.0):
    """曲线与邻机采样同一时刻的最小中心距 / 椭球距离（只算时间重叠部分）。"""
    best_c, best_e = math.inf, math.inf
    t0, t1 = float(traj.t_start), float(traj.t_end)
    for nb in neighbors or ():
        arr = np.asarray(nb, float)
        ts = arr[:, 0]
        m = (ts >= t0) & (ts <= t1)
        if not m.any():
            continue
        pts = np.asarray([traj.evaluate(float(t), 0) for t in ts[m]], float)
        d = pts - arr[m, 1:4]
        best_c = min(best_c, float(np.linalg.norm(d, axis=1).min()))
        best_e = min(best_e, float(_ellip(d, c).min()))
    return (None if not math.isfinite(best_c) else best_c), (None if not math.isfinite(best_e) else best_e)


def _wrap_refine(log: _Log):
    """包住 refine_yopo：只记录，返回原结果。窗口里额外离线重算两个反事实（不计入那次调用的预算）。"""
    import yopo_l0.backend.bspline as bs

    orig = bs.refine_yopo

    def wrapped(traj, cfg, obstacles, goal=None, neighbors=None):
        out, rec = orig(traj, cfg, obstacles, goal, neighbors=neighbors)
        if neighbors and getattr(cfg, "swarm_penalty", False):
            pre = _traj_vs_neighbors(traj, neighbors, cfg.swarm_c)
            post = _traj_vs_neighbors(out, neighbors, cfg.swarm_c)
            row = {"t": float(traj.t_start), "duration_s": float(traj.t_end - traj.t_start),
                   "pre_center": pre[0], "pre_ellip": pre[1], "post_center": post[0], "post_ellip": post[1],
                   "applied": bool(rec.get("applied")), "budget_cut": bool(rec.get("budget_cut")),
                   "reason": rec.get("reason"), "iterations": int(rec.get("iterations", 0))}
            if log.cross is None and pre[1] is not None and pre[1] < float(cfg.swarm_clearance_m):
                from dataclasses import replace

                for tag, it in (("nobudget_8", int(cfg.max_iterations)), ("nobudget_100", 100)):
                    o2, r2 = orig(traj, replace(cfg, time_budget_s=60.0, max_iterations=it), obstacles, goal,
                                  neighbors=neighbors)
                    q = _traj_vs_neighbors(o2, neighbors, cfg.swarm_c)
                    row[tag] = {"center": q[0], "ellip": q[1], "applied": bool(r2.get("applied")),
                                "reason": r2.get("reason"), "iterations": int(r2.get("iterations", 0))}
                if log.dump is not None:
                    log.dump.append({"traj": traj, "cfg": cfg, "obstacles": tuple(obstacles or ()), "goal": goal,
                                     "neighbors": [np.asarray(n, float) for n in neighbors], "row": row})
            log.refine.append(row)
        return out, rec

    bs.refine_yopo = wrapped
    return orig


def _make_note_pairs(log: _Log):
    """首次中心距 <= 2r 时记下，不立刻判失败；越线后再飞 stop_after 秒才交回 COLLISION。"""

    def note_pairs(drones, radius, min_inter, outcome, reason):
        if len(drones) < 2:
            return min_inter, outcome, reason
        t = float(drones[0]["state"].t)
        for i, a in enumerate(drones):
            for b in drones[i + 1:]:
                dist = float(np.linalg.norm(a["state"].p - b["state"].p))
                if log.cross is None:
                    min_inter = dist if min_inter is None else min(min_inter, dist)
                if log.cross is None and dist <= 2.0 * radius:
                    log.cross = {"t": t, "pair": [a["id"], b["id"]], "dist": dist,
                                 "reason": f"{a['id']}-{b['id']}:INTER_DRONE"}
        if outcome is None and log.cross is not None and t >= log.cross["t"] + log.stop_after - 1e-9:
            outcome, reason = "COLLISION", log.cross["reason"]
        return min_inter, outcome, reason

    return note_pairs


def _plan_pos(plan, times):
    out = np.empty((len(times), 3))
    for i, t in enumerate(times):
        p, _v, _a = plan.pva(float(t))
        out[i] = np.asarray(p, float)
    return out


def _peer_in_depth(frame, own_p, yaw, peer_p, mount):
    """把邻机中心投到本机最近一帧深度图上。返回 (是否在视场内, 像素深度, 真 z 深度)。"""
    if frame is None:
        return None
    from yopo_l0.sensor import R_BODY_CAM

    c, s = math.cos(yaw), math.sin(yaw)
    R_wb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    cam = np.asarray(own_p) + R_wb @ np.asarray(mount)
    R_wc = R_wb @ R_BODY_CAM
    x = R_wc.T @ (np.asarray(peer_p) - cam)
    if x[2] <= 0.0:
        return {"in_fov": False, "depth_px": None, "z_true": float(x[2])}
    fx, fy, cx, cy = frame.K
    u, v = fx * x[0] / x[2] + cx, fy * x[1] / x[2] + cy
    h, w = frame.depth.shape
    if not (0 <= u < w and 0 <= v < h):
        return {"in_fov": False, "depth_px": None, "z_true": float(x[2])}
    d = float(frame.depth[int(v), int(u)])
    return {"in_fov": True, "depth_px": None if not math.isfinite(d) else d, "z_true": float(x[2])}


def _layers(card) -> dict:
    """各层的邻机门槛。数值从场景卡和代码缺省读出，不手写。"""
    from yopo_l0.backend.config import config_from_params
    from yopo_l0.geom_fallback import GeomSession
    from yopo_l0.inter_drone import inter_drone_config

    c = card.card
    params = c.get("policy_params") or {}
    a1 = inter_drone_config(params)
    back = config_from_params(params, a_max=float(c["dynamics"].get("a_max", 6.0)))
    geom = GeomSession.from_card(c, members=tuple(d["id"] for d in c["drones"]), physics_dt=0.01,
                                 obstacles=card.obstacles)
    return {
        "collision_line_m": 2.0 * float(c["criteria"]["body_radius_m"]),
        "camera": {"renders_peers": False, "note": "DepthCamera 只拿 card.obstacles（runner.py:207）"},
        "observation": {"has_peers": False, "note": "Observation 只有 frame/state/goal/now（policy.py:24-28）"},
        "shadow": {"peers": 0, "note": "SafetyGate.validate(..., peers=(), ...)（shadow.py:241）"},
        "e_a1": {"enabled": a1 is not None, "threshold_m": None if a1 is None else a1.threshold_m,
                 "metric": "同时刻中心距"},
        "d1": {"enabled": bool(geom.enabled), "threshold_m": float(geom.inter_drone_m),
               "metric": "同时刻中心距"},
        "e_b3": {
            "enabled": bool(back is not None and getattr(back, "swarm_penalty", False)),
            "clearance_m": None if back is None or not getattr(back, "swarm_penalty", False) else back.swarm_clearance_m,
            "c": None if back is None or not getattr(back, "swarm_penalty", False) else back.swarm_c,
            "metric": "椭球距离，控制点",
        },
        "backend": {"enabled": back is not None,
                    "time_budget_s": None if back is None else back.time_budget_s,
                    "max_iterations": None if back is None else back.max_iterations},
    }


def _ellip(delta, c=2.0):
    return np.sqrt(delta[:, 0] ** 2 + delta[:, 1] ** 2 + (c * delta[:, 2]) ** 2)


def diagnose(scene: str, arm: str, seed: int, *, window_s: float, post_s: float, extra: dict | None = None,
             proto_brake: bool = False, dump_refine: str | None = None) -> dict:
    import yopo_l0.policy as P
    import yopo_l0.runner as R

    card = build_card(scene, arm, extra)
    log = _Log()
    log.stop_after = float(post_s)
    log.proto_brake = bool(proto_brake)
    log.dump = [] if dump_refine else None
    orig_cls = P.POLICIES["upstream"]
    orig_pairs = R._note_pairs
    P.POLICIES["upstream"] = _make_policy_cls(orig_cls, log)
    R._note_pairs = _make_note_pairs(log)
    import yopo_l0.backend.bspline as bs

    orig_refine = _wrap_refine(log)
    try:
        out = R.run(card, R.RunConfig(seed=int(seed), shadow=True))
    finally:
        P.POLICIES["upstream"] = orig_cls
        R._note_pairs = orig_pairs
        bs.refine_yopo = orig_refine
    if dump_refine:
        import pickle

        Path(dump_refine).parent.mkdir(parents=True, exist_ok=True)
        with open(dump_refine, "wb") as fh:
            pickle.dump(log.dump, fh)

    res = out.result
    ids = [d["id"] for d in card.card["drones"]]
    starts = {d["id"]: np.asarray(d["start_enu"], float) for d in card.card["drones"]}
    goals = {d["id"]: np.asarray(d["goal_enu"], float) for d in card.card["drones"]}
    mount = card.card["camera"].get("mount_xyz_body", [0, 0, 0])
    # ticks：每 tick 每机一行，按 (k, uav) 建索引
    P_ = {u: {} for u in ids}
    for row in out.ticks:
        P_[row["uav"]][int(row["k"])] = row
    ks = sorted(P_[ids[0]].keys())
    phys = int(card.card["rates_hz"]["physics"])
    t_of = {k: (k + 1) / phys for k in ks}  # 行里的 p 是 step 之后的状态，时间 (k+1)/phys

    summary = {"scene": scene, "arm": arm, "seed": int(seed), "proto_brake": bool(proto_brake),
               "extra": extra, "layers": _layers(card),
               "outcome_after_post": res["outcome"], "sim_duration_s": res["sim_duration_s"]}
    if log.cross is None:
        summary["crossing"] = None
        # 没有越线：报全程最小机间
        summary["min_inter_drone_m"] = res["min_inter_drone_distance_m"]
        summary["arrived"] = res["arrived"]
        summary["outcome"] = res["outcome"]
        summary["reason"] = res["reason"]
        summary["min_obstacle_distance_m"] = res["min_obstacle_distance_m"]
        summary["plan_codes"] = res["plan_codes"]
        return summary
    a, b = log.cross["pair"]
    tc = float(log.cross["t"])
    kc = int(round(tc * phys)) - 1
    summary["crossing"] = {"t_s": tc, "pair": [a, b], "dist_m": log.cross["dist"],
                           "censored_min_inter_m": res["min_inter_drone_distance_m"]}

    def pos(u, k):
        return np.asarray(P_[u][k]["p"], float)

    def cmdp(u, k):
        return np.asarray(P_[u][k]["cmd_p"], float)

    # 截断：跨线前一个 tick 的距离和本 tick 的距离 → 每 tick 缩短量
    d_prev = float(np.linalg.norm(pos(a, kc - 1) - pos(b, kc - 1)))
    va, vb = np.asarray(P_[a][kc]["v"]), np.asarray(P_[b][kc]["v"])
    rel = pos(a, kc) - pos(b, kc)
    closing = float(-np.dot(rel / np.linalg.norm(rel), va - vb))
    post_ks = [k for k in ks if k >= kc]
    d_post = [float(np.linalg.norm(pos(a, k) - pos(b, k))) for k in post_ks]
    cmd_post = [float(np.linalg.norm(cmdp(a, k) - cmdp(b, k))) for k in post_ks]
    all_min = math.inf
    for k in post_ks:
        for i, u in enumerate(ids):
            for w in ids[i + 1:]:
                all_min = min(all_min, float(np.linalg.norm(pos(u, k) - pos(w, k))))
    n_pairs_under = 0
    for i, u in enumerate(ids):
        for w in ids[i + 1:]:
            if min(float(np.linalg.norm(pos(u, k) - pos(w, k))) for k in post_ks) <= COLLIDE_M:
                n_pairs_under += 1
    summary["censoring"] = {
        "dist_prev_tick_m": d_prev,
        "dist_cross_tick_m": float(np.linalg.norm(pos(a, kc) - pos(b, kc))),
        "closing_speed_mps": closing,
        "closing_per_tick_m": closing / phys,
        "post_s": float(post_s),
        "pair_min_after_cross_m": min(d_post),
        "pair_min_cmd_after_cross_m": min(cmd_post),
        "any_pair_min_after_cross_m": all_min,
        "pairs_under_line_after_cross": n_pairs_under,
        "n_pairs": len(ids) * (len(ids) - 1) // 2,
    }

    # 窗口逐 tick
    k0 = max(ks[0], kc - int(round(window_s * phys)) + 1)
    win = [k for k in ks if k0 <= k <= kc]

    def lateral(u, p):
        s, g = starts[u], goals[u]
        d = (g - s) / np.linalg.norm(g - s)
        r = p - s
        return float(np.linalg.norm(r - np.dot(r, d) * d))

    rows = []
    for k in win:
        pa, pb = pos(a, k), pos(b, k)
        ca, cb = cmdp(a, k), cmdp(b, k)
        rows.append({
            "t": t_of[k],
            "d_true": float(np.linalg.norm(pa - pb)),
            "d_cmd": float(np.linalg.norm(ca - cb)),
            "err_a": float(np.linalg.norm(pa - ca)), "err_b": float(np.linalg.norm(pb - cb)),
            "speed_a": float(np.linalg.norm(P_[a][k]["v"])), "speed_b": float(np.linalg.norm(P_[b][k]["v"])),
            "lat_a": lateral(a, pa), "lat_b": lateral(b, pb),
            "code_a": P_[a][k]["plan_code"], "code_b": P_[b][k]["plan_code"],
        })
    summary["window_ticks"] = rows
    t_lo = t_of[win[0]]

    # 规划事件：本机计划 vs 当时已知的邻机计划
    by_uav = {u: [p for p in log.plans if p["uav"] == u] for u in ids}
    broadcast = bool(log.recv)

    def known_peer(dest, src, now):
        """dest 在 now 规划时手里那份 src 的计划。

        有广播：最近一次已收到的采样（运行器在规划之后才 poll，所以是上一拍的）。
        无广播：重建同样时序下「如果广播」会拿到的那份：src 在 now 之前最近一次规划。
        """
        if broadcast:
            got = [r for r in log.recv if r["dest"] == dest and r["src"] == src and r["t0"] < now - 1e-9]
            if not got:
                return None, None
            r = got[-1]
            return r["t0"], ("samples", r["arr"])
        prev = [p for p in by_uav[src] if p["now"] < now - 1e-9]
        if not prev:
            return None, None
        return prev[-1]["now"], ("plan", prev[-1]["plan"])

    def peer_track(kind_obj, times):
        kind, obj = kind_obj
        if kind == "plan":
            return _plan_pos(obj, times)
        ts = obj[:, 0]
        return np.column_stack([np.interp(times, ts, obj[:, j]) for j in (1, 2, 3)])

    def truth_track(u, times):
        out = []
        for t in times:
            k = int(round(t * phys)) - 1
            k = min(max(k, ks[0]), ks[-1])
            out.append(pos(u, k))
        return np.asarray(out)

    t_last = t_of[ks[-1]]
    events = []
    for me, other in ((a, b), (b, a)):
        for ev in by_uav[me]:
            now = ev["now"]
            if now < t_lo - 1e-9 or now > tc + 1e-9:
                continue
            t_sent, kp = known_peer(me, other, now)
            times = now + np.arange(int(round(HORIZON / SAMPLE_DT))) * SAMPLE_DT
            own = _plan_pos(ev["plan"], times)
            rec = {
                "uav": me, "peer": other, "now": now, "code": ev["code"], "backend": ev["backend"],
                "peer_plan_age_s": None if t_sent is None else now - t_sent,
                "d_now_true": float(np.linalg.norm(ev["state_p"] - truth_track(other, [now])[0])),
            }
            # 预测分离只在已知邻机计划存在时可算
            if kp is not None:
                peer = peer_track(kp, times)
                dd = np.linalg.norm(own - peer, axis=1)
                i = int(np.argmin(dd))
                rec["pred_min_center_m"] = float(dd[i])
                rec["pred_min_t"] = float(times[i])
                rec["pred_min_ellipsoid_m"] = float(_ellip(own - peer).min())
            # 实际：两机真值在同一时段里的最小距离（只算到运行实际飞到的时刻）
            mask = times <= t_last + 1e-9
            if mask.any():
                ta = truth_track(me, times[mask])
                tb = truth_track(other, times[mask])
                da = np.linalg.norm(ta - tb, axis=1)
                j = int(np.argmin(da))
                rec["true_min_center_m"] = float(da[j])
                rec["true_min_t"] = float(times[mask][j])
                # 本机新计划对邻机真值：只看「邻机计划过时」这一项
                rec["own_plan_vs_peer_truth_min_m"] = float(np.linalg.norm(own[mask] - tb, axis=1).min())
            cam = _peer_in_depth(ev["frame"], ev["state_p"], ev["yaw"], truth_track(other, [now])[0], mount)
            rec["camera"] = cam
            events.append(rec)
    events.sort(key=lambda r: (r["now"], r["uav"]))
    summary["plan_events"] = events
    if log.refine:
        win_ref = [r for r in log.refine if t_lo - 1e-9 <= r["t"] <= tc + 1e-9]
        summary["e_b3_refine_window"] = win_ref

        def _r(key, sub=None):
            vals = [(r[key][sub] if sub else r[key]) for r in win_ref if key in r and (r[key] if not sub else r[key][sub]) is not None]
            return _rng(vals)

        summary["e_b3_refine_summary"] = {
            "calls_with_peer_inside_C": len([r for r in win_ref if r["pre_ellip"] is not None and r["pre_ellip"] < 0.8]),
            "calls": len(win_ref),
            "budget_cut": sum(1 for r in win_ref if r["budget_cut"]),
            "pre_ellip": _r("pre_ellip"), "post_ellip": _r("post_ellip"),
            "pre_center": _r("pre_center"), "post_center": _r("post_center"),
            "nobudget_8_center": _r("nobudget_8", "center"), "nobudget_100_center": _r("nobudget_100", "center"),
            "nobudget_100_ellip": _r("nobudget_100", "ellip"),
        }

    # 汇总
    errs = [max(r["err_a"], r["err_b"]) for r in rows]
    last = rows[-1]
    codes = {}
    for r in rows:
        for c in (r["code_a"], r["code_b"]):
            codes[c] = codes.get(c, 0) + 1
    backs = [e["backend"] for e in events if e["backend"] is not None]
    summary["window_summary"] = {
        "t_from_s": t_lo, "t_to_s": tc,
        "d_true_at_cross_m": last["d_true"], "d_cmd_at_cross_m": last["d_cmd"],
        "track_err_max_m": max(errs), "track_err_at_cross_m": [last["err_a"], last["err_b"]],
        "speed_start_mps": [rows[0]["speed_a"], rows[0]["speed_b"]],
        "speed_at_cross_mps": [last["speed_a"], last["speed_b"]],
        "lateral_max_m": [max(r["lat_a"] for r in rows), max(r["lat_b"] for r in rows)],
        "tick_codes": codes,
        "plan_events": len(events),
        "peer_plan_age_s": sorted({round(e["peer_plan_age_s"], 6) for e in events if e["peer_plan_age_s"] is not None}),
        "pred_min_center_m": _rng([e.get("pred_min_center_m") for e in events]),
        "true_min_center_m": _rng([e.get("true_min_center_m") for e in events]),
        "own_plan_vs_peer_truth_min_m": _rng([e.get("own_plan_vs_peer_truth_min_m") for e in events]),
        "camera_in_fov_events": sum(1 for e in events if e["camera"] and e["camera"]["in_fov"]),
        "camera_depth_hits_peer": sum(
            1 for e in events if e["camera"] and e["camera"]["in_fov"] and e["camera"]["depth_px"] is not None
            and abs(e["camera"]["depth_px"] - e["camera"]["z_true"]) < 0.5),
        "backend_calls": len(backs),
        "backend_applied": sum(1 for x in backs if x["applied"]),
        "backend_budget_cut": sum(1 for x in backs if x["budget_cut"]),
        "broadcast_samples": broadcast,
    }
    return summary


def compare_row(scene: str, arm: str, seed: int, *, extra: dict | None = None, proto_brake: bool = False) -> dict:
    """对照用：运行器原样判碰撞（不换 _note_pairs），只按需挂修复原型。"""
    import yopo_l0.policy as P
    import yopo_l0.runner as R

    card = build_card(scene, arm, extra)
    log = _Log()
    log.proto_brake = bool(proto_brake)
    orig_cls = P.POLICIES["upstream"]
    if proto_brake:
        P.POLICIES["upstream"] = _make_policy_cls(orig_cls, log)
    try:
        res = R.run(card, R.RunConfig(seed=int(seed), shadow=True)).result
    finally:
        P.POLICIES["upstream"] = orig_cls
    reason = str(res["reason"])
    return {
        "scene": scene, "arm": arm, "seed": int(seed), "proto_brake": bool(proto_brake), "extra": extra,
        "scene_card_hash": res["scene_card_hash"],
        "outcome": res["outcome"], "reason": reason, "arrived": res["arrived"], "n_drones": res["n_drones"],
        "inter_drone_collision": int(res["outcome"] == "COLLISION" and "INTER_DRONE" in reason),
        "tree_collision": int(res["outcome"] == "COLLISION" and "TRUTH_POSITION" in reason),
        "timeout": int(res["outcome"] == "TIMEOUT"),
        "min_inter_drone_distance_m": res["min_inter_drone_distance_m"],
        "min_obstacle_distance_m": res["min_obstacle_distance_m"],
        "sim_duration_s": res["sim_duration_s"],
        "arrival_time_s": res["arrival_time_s"],
        "plan_codes": res["plan_codes"],
        "inter_drone_filter": res.get("inter_drone_filter"),
        "proto_brakes": sum(1 for p in log.plans if p["code"] == "CORRIDOR_EMPTY_HOVER") if proto_brake else None,
        "wall_time_s": res["wall_time_s"],
    }


def analyze_dump(path: str) -> list[dict]:
    """离线复算 --dump-refine 存下的后段调用：最近点落在曲线哪一段、是否落在钉死的端点控制点上、
    椭球项在拟合初值上的代价和梯度有多大。只读 bspline 的内部函数，不改它。"""
    import pickle

    import yopo_l0.backend.bspline as bs

    rows = []
    with open(path, "rb") as fh:
        calls = pickle.load(fh)
    for call in calls:
        traj, cfg, nbs = call["traj"], call["cfg"], call["neighbors"]
        t0, t1 = float(traj.t_start), float(traj.t_end)
        n_span, dt = bs._choose_knots(t1 - t0)
        best = (math.inf, None)
        for nb in nbs:
            ts = nb[:, 0]
            m = (ts >= t0) & (ts <= t1)
            for t, q in zip(ts[m], nb[m, 1:4]):
                d = float(np.linalg.norm(np.asarray(traj.evaluate(float(t), 0)) - q))
                if d < best[0]:
                    best = (d, float(t))
        # 自由控制点的 Greville 时刻覆盖 [t0 + 2dt, t1 - 2dt]
        free_lo, free_hi = t0 + 2.0 * dt, t0 + (n_span - 2) * dt
        n_ctrl = n_span + 3
        p0, v0, a0 = bs._pva(traj, t0)
        p1, v1, a1 = bs._pva(traj, t1)
        q_fixed = np.zeros((n_ctrl, 3))
        q_fixed[0], q_fixed[1], q_fixed[2] = bs._boundary_controls(p0, v0, a0, dt)
        q_fixed[-3], q_fixed[-2], q_fixed[-1] = bs._boundary_controls(p1, v1, a1, dt)
        free = np.arange(3, n_ctrl - 3)
        q_fit, _rmse = bs._fit_free(traj, q_fixed, free, n_span, dt)
        sc, sg = bs._swarm_cost_grad(q_fit, dt, t0, tuple(nbs), float(cfg.swarm_clearance_m), float(cfg.swarm_c))
        idx, wts = bs._sample_layout(n_span)
        oc, og = bs._cost_grad(q_fit, q_fit, free, dt, call["obstacles"], float(cfg.clearance_m) + bs._CLEARANCE_BUFFER_M,
                               float(cfg.v_max), float(cfg.a_max), idx, wts)
        # 每个自由控制点到邻机同时刻位置的距离
        times = t0 + (free.astype(float) - 1.0) * dt
        ctrl_d = []
        for i, t in zip(free, times):
            dd = math.inf
            for nb in nbs:
                ts = nb[:, 0]
                if ts[0] <= t <= ts[-1]:
                    q = np.array([np.interp(t, ts, nb[:, j]) for j in (1, 2, 3)])
                    dd = min(dd, float(np.linalg.norm(q_fit[i] - q)))
            ctrl_d.append(None if not math.isfinite(dd) else dd)
        rows.append({
            "t_start": t0, "t_end": t1, "dt": dt, "n_span": n_span,
            "min_center_m": best[0], "min_t": best[1],
            "min_in_free_window": None if best[1] is None else bool(free_lo <= best[1] <= free_hi),
            "free_window": [free_lo, free_hi],
            "swarm_cost_at_fit": float(sc), "swarm_grad_norm_free": float(np.linalg.norm(sg[free])),
            "other_cost_at_fit": float(oc), "other_grad_norm_free": float(np.linalg.norm(og[free])),
            "free_ctrl_dist_to_peer_m": ctrl_d,
            "row": call["row"],
        })
    return rows


DISCLAIMER = "修复臂是进程内原型（本脚本的 _proto_brake + E-A1 阈值 1.0 m），不是已合入的开关；只报数"


def _round(x, nd=4):
    if isinstance(x, float):
        return round(x, nd)
    if isinstance(x, dict):
        return {k: _round(v, nd) for k, v in x.items()}
    if isinstance(x, list):
        return [_round(v, nd) for v in x]
    return x


def collect(diag_dir: str, out_json: str) -> dict:
    """把 out/ 下的诊断和对照结果收成一份 runs.json（浮点保留 4 位）。"""
    d = Path(diag_dir)
    diags, compare, analysis = {}, [], {}
    for f in sorted(d.glob("*.json")):
        obj = json.loads(f.read_text())
        if f.name.startswith("analysis-"):
            analysis[f.stem[len("analysis-"):]] = obj
            continue
        if f.name.startswith("cmp-"):
            obj.pop("wall_time_s", None)
            obj["label"] = f.stem.split("-")[-2]  # cmp-<卡>-<臂>-<seed>
            compare.append(obj)
        else:
            diags[f.stem] = obj
    compare.sort(key=lambda r: (r["scene"], r["label"], r["seed"]))
    data = {"disclaimer": DISCLAIMER, "diagnostics": _round(diags), "e_b3_refine_analysis": _round(analysis),
            "compare": _round(compare)}
    Path(out_json).write_text(json_text(data, indent=1), encoding="utf-8")
    return data


LABELS = {
    "off": "全关",
    "a1m": "E-A1 1.0 m",
    "a1b": "E-A1 0.70 m + 刹停原型",
    "fix": "修复：E-A1 1.0 m + 刹停原型",
}


def render_compare(rows: list[dict]) -> str:
    """runs.json 的 compare 段 → Markdown 两张表（汇总、分种子）。"""
    out = ["| 卡 | 臂 | seeds | 到达（架次） | 机间碰撞 | 撞树 | 超时 | 最小机间 m | 最小障碍 m | 刹停原型触发（规划拍） |",
           "|---|---|---|---|---|---|---|---|---|---|"]
    order = [k for k in LABELS]
    groups: dict = {}
    for r in rows:
        groups.setdefault((r["scene"], r["label"]), []).append(r)
    for scene in ("swap8", "d014-swap3"):
        for lab in order:
            g = sorted(groups.get((scene, lab), []), key=lambda r: r["seed"])
            if not g:
                continue
            seeds = ",".join(str(r["seed"]) for r in g)
            arr = sum(int(r["arrived"]) for r in g)
            tot = sum(int(r["n_drones"]) for r in g)
            mi = [r["min_inter_drone_distance_m"] for r in g if r["min_inter_drone_distance_m"] is not None]
            mo = [r["min_obstacle_distance_m"] for r in g]
            pb = [r["proto_brakes"] for r in g if r["proto_brakes"] is not None]
            out.append(
                f"| {scene} | {LABELS[lab]} | {seeds if len(g) < 10 else '0–9'} | {arr}/{tot} | "
                f"{sum(r['inter_drone_collision'] for r in g)} | {sum(r['tree_collision'] for r in g)} | "
                f"{sum(r['timeout'] for r in g)} | {min(mi):.3f}–{max(mi):.3f} | {min(mo):.3f}–{max(mo):.3f} | "
                f"{'—' if not pb else f'{min(pb)}–{max(pb)}'} |")
    out += ["", "| 卡 | 臂 | seed | 结果 | 原因 | 到达 | 最小机间 m | 最小障碍 m | 仿真 s |", "|---|---|---|---|---|---|---|---|---|"]
    for scene in ("swap8", "d014-swap3"):
        for lab in order:
            for r in sorted(groups.get((scene, lab), []), key=lambda r: r["seed"]):
                mi = r["min_inter_drone_distance_m"]
                out.append(
                    f"| {scene} | {LABELS[lab]} | {r['seed']} | {r['outcome']} | `{r['reason']}` | "
                    f"{r['arrived']}/{r['n_drones']} | {'—' if mi is None else f'{mi:.3f}'} | "
                    f"{r['min_obstacle_distance_m']:.3f} | {r['sim_duration_s']:.2f} |")
    return "\n".join(out)


def _rng(xs):
    xs = [x for x in xs if x is not None]
    return None if not xs else [float(min(xs)), float(max(xs))]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scene", choices=sorted(SCENES), default=None)
    ap.add_argument("--collect", default=None, help="把这个目录下的诊断 / 对照 json 收进 --out")
    ap.add_argument("--render-md", default=None, help="和 --collect 一起用：把对照表写进报告的标记之间")
    ap.add_argument("--analyze-dump", default=None, help="离线复算 --dump-refine 的 pickle（只读）")
    ap.add_argument("--arm", choices=ARMS, default="off")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--window-s", type=float, default=2.0)
    ap.add_argument("--post-s", type=float, default=1.0, help="越线后多飞多久（只用于看不截断时的最小距离）")
    ap.add_argument("--extra", default=None, help="额外 policy_params 覆盖（JSON），例如修复开关")
    ap.add_argument("--out", default=None)
    ap.add_argument("--dump-refine", default=None, help="把邻机落进 C 的后段调用输入存成 pickle（离线复算用）")
    ap.add_argument("--compare", action="store_true", help="只出一行对照结果（运行器原样判碰撞）")
    ap.add_argument("--proto-brake", action="store_true", help="修复原型：无解时按 a_max 从参考头刹停（进程内）")
    ap.add_argument("--code-root", default=None, help="从这份检出 import yopo_l0、读场景卡")
    args = ap.parse_args(argv)
    extra = json.loads(args.extra) if args.extra else None
    if args.collect:
        data = collect(args.collect, args.out)
        if args.render_md:
            md = Path(args.render_md)
            text = md.read_text(encoding="utf-8")
            a, b = "<!-- e-b4-compare:start -->", "<!-- e-b4-compare:end -->"
            head, rest = text.split(a, 1)
            _old, tail = rest.split(b, 1)
            md.write_text(head + a + "\n" + render_compare(data["compare"]) + "\n" + b + tail, encoding="utf-8")
        return 0
    if args.analyze_dump:
        rows = analyze_dump(args.analyze_dump)
        text = json_text(rows, indent=1, final_newline=False)
        if args.out:
            Path(args.out).write_text(text + "\n", encoding="utf-8")
        print(text)
        return 0
    if args.scene is None:
        ap.error("需要 --scene")
    if args.compare:
        row = compare_row(args.scene, args.arm, args.seed, extra=extra, proto_brake=args.proto_brake)
        text = json_text(row, indent=None, final_newline=False)
        if args.out:
            Path(args.out).parent.mkdir(parents=True, exist_ok=True)
            Path(args.out).write_text(text + "\n", encoding="utf-8")
        print(text)
        return 0
    s = diagnose(args.scene, args.arm, args.seed, window_s=args.window_s, post_s=args.post_s, extra=extra,
                 proto_brake=args.proto_brake, dump_refine=args.dump_refine)
    text = json_text(s, indent=1, final_newline=False)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text + "\n", encoding="utf-8")
    short = {k: v for k, v in s.items() if k not in ("window_ticks", "plan_events")}
    print(json_text(short, indent=1, final_newline=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
