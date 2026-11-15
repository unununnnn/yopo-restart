"""票 SH3：对照上游语义，诊断影子盾契约码的触发条件。只读，不改批准动作。"""
from __future__ import annotations

import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "core" / "src"))

from yopo_swarm_core.plan import (  # noqa: E402
    HOLD_TOL,
    KinematicLimits,
    PlanReject,
    brake_branch,
    make_plan,
    state_at,
    validate_plan,
)
from yopo_swarm_core.planner import GateReject, SafetyGate  # noqa: E402
from yopo_swarm_core.trajectory import PiecewisePolynomial  # noqa: E402
from yopo_swarm_core.uav import default_config  # noqa: E402

from yopo_l0.backend.config import BackendConfig  # noqa: E402
from yopo_l0.policy import CertifiedPolicy  # noqa: E402
from yopo_l0.runner import RunConfig, run  # noqa: E402
from yopo_l0.scene import load_card  # noqa: E402
from yopo_l0.shadow import (  # noqa: E402
    ORACLE_TRACKING_BOUND,
    DroneShadow,
    record_oracle_track_lag,
)

CERT = KinematicLimits(4.0, 8.0, 80.0)
UP = KinematicLimits(6.0, 6.0, 80.0)
UP_NO_J = KinematicLimits(6.0, 6.0, 20000.0)


def peaks(traj: PiecewisePolynomial, dt: float = 0.01) -> dict:
    t0, t1 = float(traj.t_start), float(traj.t_end)
    n = max(2, int(np.ceil((t1 - t0) / dt)) + 1)
    ts = np.linspace(t0, t1, n)
    out = {}
    for order, name in ((1, "v"), (2, "a"), (3, "j")):
        m = 0.0
        tw = t0
        for t in ts:
            val = float(np.linalg.norm(traj.evaluate(float(t), order)))
            if val > m:
                m, tw = val, float(t)
        out[name] = m
        out[f"{name}_t"] = tw
    end = np.array([traj.evaluate(t1, k) for k in range(3)])
    out["end_v"] = float(np.linalg.norm(end[1]))
    out["end_a"] = float(np.linalg.norm(end[2]))
    out["t_start"] = t0
    out["t_end"] = t1
    out["duration"] = t1 - t0
    return out


def inspect_wrap(shadow: DroneShadow, plan, now: float) -> dict:
    traj = getattr(plan, "traj", None)
    rec = {"now": float(now), "kind": type(plan).__name__}
    if not isinstance(traj, PiecewisePolynomial):
        rec["has_traj"] = False
        return rec
    rec["has_traj"] = True
    rec.update({f"peak_{k}": v for k, v in peaks(traj).items()})
    t_switch = float(traj.t_start)
    rec["t_switch"] = t_switch
    rec["switch_in_future"] = bool(t_switch > now)
    rec["switch_minus_now"] = t_switch - float(now)
    head = np.array([traj.evaluate(t_switch, k) for k in range(3)])
    parent_at = state_at(shadow.parent, t_switch, None)
    rec["head_vs_parent_maxabs"] = float(np.max(np.abs(head - parent_at)))
    rec["parent_seq"] = int(shadow.parent.plan_seq)
    rec["end_holdable"] = rec["peak_end_v"] <= HOLD_TOL and rec["peak_end_a"] <= HOLD_TOL
    rec["kin_cert_ok"] = all(
        # filled below via make_plan
        True
        for _ in ()
    )
    rec["v_gt_cert"] = rec["peak_v"] > CERT.v_max
    rec["v_gt_up"] = rec["peak_v"] > UP.v_max
    rec["a_gt_cert"] = rec["peak_a"] > CERT.a_max
    rec["a_gt_up"] = rec["peak_a"] > UP.a_max
    rec["j_gt_cert"] = rec["peak_j"] > CERT.j_max
    rec["j_gt_up_no_j"] = rec["peak_j"] > UP_NO_J.j_max

    branches = [t_switch + o for o in shadow.cfg.branch_offsets]
    rec["branch_times"] = branches
    rec["branch_in_prefix"] = [
        bool(traj.t_start <= tb <= traj.t_end) for tb in branches
    ]
    rec["brake"] = {}
    for name, lim in (("cert", CERT), ("up", UP), ("up_no_j", UP_NO_J)):
        ok = []
        for tb in branches:
            if not (traj.t_start <= tb <= traj.t_end):
                ok.append("outside")
                continue
            pva = np.array([traj.evaluate(tb, k) for k in range(3)])
            ok.append("ok" if brake_branch(tb, pva, lim) is not None else "unreachable")
        rec["brake"][name] = ok

    built = make_plan(
        shadow.owner, shadow.seq, shadow.parent.plan_hash(shadow.cfg.plan),
        traj, t_switch, head, branches, shadow.cfg.plan,
    )
    if isinstance(built, list):
        rec["make_plan"] = [r.value for r in built]
        rec["make_plan_ok"] = False
    else:
        rec["make_plan"] = []
        rec["make_plan_ok"] = True
        rec["validate"] = [r.value for r in validate_plan(built, shadow.cfg.plan)]
        res = shadow.gate.validate(built, shadow.parent, None, shadow.space, (), now)
        rec["gate"] = list(res.codes) if isinstance(res, GateReject) else "approved"

    # aligned make_plan with upstream limits, remaining-prefix branches at now
    from yopo_swarm_core.plan import PlanConfig
    aligned = PlanConfig(UP, body_radius=shadow.cfg.plan.body_radius, tracking_margin=shadow.cfg.plan.tracking_margin)
    rec["make_plan_up"] = []
    built_up = make_plan(
        shadow.owner, shadow.seq, shadow.parent.plan_hash(aligned),
        traj, t_switch, head, [tb for tb in branches if traj.t_start <= tb <= traj.t_end] or [t_switch],
        aligned,
    )
    rec["make_plan_up"] = [r.value for r in built_up] if isinstance(built_up, list) else []

    aligned2 = PlanConfig(UP_NO_J, body_radius=shadow.cfg.plan.body_radius, tracking_margin=shadow.cfg.plan.tracking_margin)
    built2 = make_plan(
        shadow.owner, shadow.seq, shadow.parent.plan_hash(aligned2),
        traj, t_switch, head, [tb for tb in branches if traj.t_start <= tb <= traj.t_end] or [t_switch],
        aligned2,
    )
    rec["make_plan_up_no_j"] = [r.value for r in built2] if isinstance(built2, list) else []
    return rec


def run_card(scene: str, seed: int, every: int = 20) -> dict:
    card = load_card(scene)
    samples = []
    counts = Counter()
    seen_codes = Counter()

    orig = DroneShadow._recheck

    def hooked(self, plan, now):
        rec = inspect_wrap(self, plan, now)
        env, contract, stop = orig(self, plan, now)
        codes = tuple(dict.fromkeys((*env, *contract, *stop)))
        rec["codes"] = list(codes)
        rec["env"] = list(env)
        rec["contract"] = list(contract)
        rec["stop"] = list(stop)
        counts["selections"] += 1
        for c in codes:
            seen_codes[c] += 1
        if counts["selections"] <= 3 or counts["selections"] % every == 0:
            rec["selection_i"] = counts["selections"]
            samples.append(rec)
        return env, contract, stop

    DroneShadow._recheck = hooked
    try:
        out = run(card, RunConfig(seed=seed, shadow=True, shadow_env_evidence=False))
    finally:
        DroneShadow._recheck = orig

    result = dict(out.result)
    return {
        "scene": card.name,
        "seed": seed,
        "outcome": result.get("outcome"),
        "arrived": result.get("arrived") or result.get("n_arrived"),
        "collisions": result.get("collisions") or result.get("n_collisions") or result.get("collision"),
        "shadow_reject_rate": result.get("shadow_reject_rate"),
        "histogram": result.get("shadow_reject_histogram"),
        "selections": counts["selections"],
        "selection_codes": dict(seen_codes),
        "samples": samples,
        "backend": {
            "v_max": BackendConfig().v_max,
            "a_max": BackendConfig().a_max,
        },
        "cert_limits": {"v": CERT.v_max, "a": CERT.a_max, "j": CERT.j_max},
        "oracle_tracking_bound": ORACLE_TRACKING_BOUND,
    }


def min_repros() -> dict:
    """不跑场景，只构造最小多项式，标出每个码的触发位置。"""
    from yopo_swarm_core.plan import hover_plan, quintic
    from yopo_swarm_core.trajectory import PiecewisePolynomial

    cfg = default_config("uav0")
    out = {}

    # TIME:SWITCH_NOT_IN_FUTURE — hover at t=0, now=0; SafetyGate requires t_switch > now
    hover = hover_plan("uav0", np.zeros(3), 0.0, cfg.plan)
    gate = SafetyGate(cfg)
    from yopo_swarm_core.freespace import FreeSpace
    res = gate.validate(hover, hover, None, FreeSpace(cfg.space), (), now=0.0)
    out["TIME:SWITCH_NOT_IN_FUTURE"] = {
        "where": "core/src/yopo_swarm_core/planner.py SafetyGate.validate: if not plan.t_switch > now",
        "wrap": "yopo_l0/shadow.py DroneShadow._build: t_switch = traj.t_start；对已在执行的轨迹复查时 t_switch 不会大于 now",
        "repro": "hover_plan(..., t0=0) 在 now=0 交给 SafetyGate",
        "t_switch": hover.t_switch,
        "now": 0.0,
        "codes": list(res.codes) if isinstance(res, GateReject) else "approved",
    }

    # HEAD_DISCONTINUITY (SafetyGate parent): new traj starts away from hover parent
    t0 = 0.0
    head = np.array([[1.0, 0, 0], [1.0, 0, 0], [0, 0, 0]])  # already moving, not at origin
    end = np.array([[5.0, 0, 0], [0, 0, 0], [0, 0, 0]])
    # long enough + stop so make_plan can succeed under cert limits if v<=4
    # v starts at 1, ends at 0. Use a gentle traj
    traj = PiecewisePolynomial(t0, [quintic(np.array([[0,0,0],[0,0,0],[0,0,0]]), end, 4.0)])
    # wait this starts at origin at rest — parent matches.
    # Use mid-flight: t_start=1, position at 2m, parent hover still at 0
    head2 = np.array([[2.0, 0, 0], [1.0, 0, 0], [0, 0, 0]])
    end2 = np.array([[6.0, 0, 0], [0, 0, 0], [0, 0, 0]])
    traj2 = PiecewisePolynomial(1.0, [quintic(head2, end2, 8.0)])
    parent = hover_plan("uav0", np.zeros(3), 0.0, cfg.plan)
    built = make_plan("uav0", 1, parent.plan_hash(cfg.plan), traj2, 1.0, head2, [1.0, 1.5], cfg.plan)
    if isinstance(built, list):
        gate_codes = [r.value for r in built]
        make_ok = False
    else:
        make_ok = True
        res = gate.validate(built, parent, None, FreeSpace(cfg.space), (), now=1.5)
        gate_codes = list(res.codes) if isinstance(res, GateReject) else "approved"
        parent_at = state_at(parent, 1.0)
        gap = float(np.max(np.abs(head2 - parent_at)))
    out["TRANSITION:HEAD_DISCONTINUITY"] = {
        "where_validate": "core/.../plan.py validate_plan: motion 起点 PVA 与声明 head 不一致",
        "where_gate": "core/.../planner.py SafetyGate: plan.head_pva 与 parent 在 t_switch 的状态差过 CONTINUITY_TOL",
        "wrap": "shadow._build 把 head 取成 traj 在 t_start 的 PVA，validate 这条通常不过；但 parent 卡在初始悬停（门因为 TIME:SWITCH 从未批准），中途计划的 head 对不上",
        "repro": "t_start=1 的运动从 x=2 出发，parent 是原点悬停，now=1.5 交给 SafetyGate",
        "make_plan_ok": make_ok,
        "codes": gate_codes,
        "head_vs_parent_maxabs": gap if make_ok else None,
    }

    # TERMINAL_NOT_HOLDABLE: end with leftover velocity
    head3 = np.array([[0,0,0],[0,0,0],[0,0,0]])
    end3 = np.array([[3,0,0],[1.0,0,0],[0,0,0]])  # end v=1
    traj3 = PiecewisePolynomial(0.0, [quintic(head3, end3, 3.0)])
    built3 = make_plan("uav0", 1, parent.plan_hash(cfg.plan), traj3, 0.0, head3, [0.0, 0.5], cfg.plan)
    out["TRANSITION:TERMINAL_NOT_HOLDABLE"] = {
        "where": "core/.../plan.py validate_plan: 运动终点 |v| 或 |a| > HOLD_TOL (1e-6)",
        "wrap": "上游 YOPO 轨迹是滚动时域片段，并不保证在 t_end 收成悬停",
        "repro": "quintic 从静止到 x=3、终点速度 1 m/s",
        "codes": [r.value for r in built3] if isinstance(built3, list) else "plan",
        "end_v": 1.0,
        "HOLD_TOL": HOLD_TOL,
    }

    # MOTION_KINEMATICS: v=5 > cert 4, < upstream 6
    # distance 10m in 2s with start/end rest would have high v... 
    # simpler: start at 5 m/s, go forward, end at rest slowly
    head4 = np.array([[0,0,0],[5.0,0,0],[0,0,0]])
    end4 = np.array([[10.0,0,0],[0,0,0],[0,0,0]])
    traj4 = PiecewisePolynomial(0.0, [quintic(head4, end4, 4.0)])
    pk = peaks(traj4)
    built4 = make_plan("uav0", 1, parent.plan_hash(cfg.plan), traj4, 0.0, head4, [0.0, 0.5], cfg.plan)
    from yopo_swarm_core.plan import PlanConfig
    up_cfg = PlanConfig(UP, body_radius=cfg.plan.body_radius, tracking_margin=cfg.plan.tracking_margin)
    built4u = make_plan("uav0", 1, parent.plan_hash(up_cfg), traj4, 0.0, head4, [0.0, 0.5], up_cfg)
    out["TRANSITION:MOTION_KINEMATICS"] = {
        "where": "core/.../plan.py validate_plan -> _kinematics_ok；限幅来自 default_config KinematicLimits(4,8,80)",
        "wrap": "上游 backend 默认 v_max=6、a_max=6；影子却用认证默认 v_max=4",
        "repro": "起点速度 5 m/s 的五次多项式，超过认证 4、不超过上游 6",
        "peaks": pk,
        "make_plan_cert": [r.value for r in built4] if isinstance(built4, list) else "plan",
        "make_plan_up": [r.value for r in built4u] if isinstance(built4u, list) else "plan",
    }

    # BRANCH_UNREACHABLE: brake_branch returns None
    # high speed with tight a_max, or state already over limits so brake can't satisfy kinematics
    head5 = np.array([[0,0,0],[20.0,0,0],[0,0,0]])  # 20 m/s, cert v_max=4
    end5 = np.array([[40.0,0,0],[0,0,0],[0,0,0]])
    traj5 = PiecewisePolynomial(0.0, [quintic(head5, end5, 4.0)])
    br = brake_branch(0.0, head5, CERT)
    built5 = make_plan("uav0", 1, parent.plan_hash(cfg.plan), traj5, 0.0, head5, [0.0, 0.5], cfg.plan)
    out["TRANSITION:BRANCH_UNREACHABLE"] = {
        "where": "core/.../plan.py make_plan: brake_branch(...) is None",
        "wrap": "影子按 t_switch+{0,0.5} 现造制动分支；若当前速度已超认证 v_max，分支很难在限幅内刹住",
        "repro": "起点 20 m/s，认证限幅下 brake_branch 返回 None",
        "brake_cert": br is not None,
        "codes": [r.value for r in built5] if isinstance(built5, list) else "plan",
        "peaks": peaks(traj5),
    }

    # TRACKING_BOUND
    start = np.zeros(3)
    goal = np.array([10.0, 0, 0])
    actual = np.array([0.16, 0, 0])
    desired = np.zeros(3)
    codes = record_oracle_track_lag(1.0, 1.0, actual, desired, start, goal, 0.0)
    out["TRACKING_BOUND"] = {
        "where": "yopo_l0/shadow.py record_oracle_track_lag: ||actual-desired|| > FlightOracle.tracking_bound (0.15 m)",
        "wrap": "界取认证 FlightOracle 默认值。L0 点质量跟踪很紧；SITL 上 PX4 常超过 0.15 m，但上游/PX4 并未承诺这条界",
        "repro": "actual 与 desired 相距 0.16 m",
        "bound": ORACLE_TRACKING_BOUND,
        "error": 0.16,
        "codes": list(codes),
    }
    return out


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "all"
    dest = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("/workspace/yopo-ops/sh3/diagnose.json")
    payload = {"repros": min_repros()}
    if cmd in ("all", "l0"):
        scenes = ["scenes/g2-pillar.json", "scenes/crossing-pillars.json"]
        payload["l0"] = [run_card(s, 0) for s in scenes]
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload, ensure_ascii=False, indent=2, default=str))
    print(f"wrote {dest}")
    print(json.dumps({"repros_ok": list(payload["repros"]), "l0": [
        {k: r[k] for k in ("scene", "outcome", "shadow_reject_rate", "histogram", "selections") if k in r}
        for r in payload.get("l0", [])
    ]}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
