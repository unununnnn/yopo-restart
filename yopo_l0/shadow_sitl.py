"""票 SH2：把影子盾环境证据接到 SITL 桥。默认关，打开即偏离上游。

``sitl_bridge`` 只调用这里的钩子：要不要开影子、要不要把场景障碍交给
``DroneShadow``、编队类策略用哪份计划和哪座钟、结果里写哪些字段。
逻辑不进桥接主循环。影子盾仍然只记录，不改设定点。
"""

from __future__ import annotations

import numpy as np

from .dynamics import State
from .shadow import DroneShadow, approved_pva, codes_would_reject

ENV_RESULT_KEYS = (
    "shadow_env_evidence",
    "shadow_env_views",
    "shadow_env_clearance_evals",
    "shadow_env_certificate_centers",
)

# 拒绝码的触发条件。只解释，不改判据。数值来自 core 现有缺省。
CODE_CONDITIONS = {
    "ENV:NO_FRAME": "这一拍影子没有深度帧。",
    "ENV:NONFINITE": "深度或位姿里有非有限值，证据加不进 FreeSpace。",
    "ENV:OCCUPIED": "classify_sweep 认为扫掠球碰到已观测到的表面。",
    "ENV:UNKNOWN": "扫掠里有样本既不是 FREE 也不是 OCCUPIED。关证据时，相机后方和最小量程以内通常落在这里。",
    "TIME:SWITCH_NOT_IN_FUTURE": "包装后的计划 t_switch 不大于当前影子时钟。上游多项式的 t_switch 取轨迹起点，计划一开始跑就会记下这个码。",
    "TRANSITION:HEAD_DISCONTINUITY": "运动段起点的位置/速度/加速度与声明的 head_pva 对不上，或与父计划在 t_switch 的状态差超过 1e-6。",
    "TRANSITION:TERMINAL_NOT_HOLDABLE": "运动段末端速度或加速度的绝对值超过 1e-6，不能当悬停收住。",
    "TRANSITION:MOTION_KINEMATICS": "运动段超过计划运动学限制（缺省 v_max=4 m/s、a_max=8 m/s²、j_max=80 m/s³）。",
    "TRANSITION:BRANCH_UNREACHABLE": "在某个制动分支时刻，按同样限制生成不出刹停曲线。",
    "TRACKING_BOUND": "机体位置与批准位置的欧氏距离超过 FlightOracle.tracking_bound（0.15 m）。",
    "LAG_VIOLATION": "沿起点到终点的进度落后超过 FlightOracle.lag_limit（1.5 m）；参考速度 0.15 m/s。",
    "ODOMETRY_AGE_OR_NONFINITE": "里程计时间戳与影子时钟相差超过 0.2 s，或位置不是有限三维向量。",
}


def shadow_enabled(cfg, policy: str) -> bool:
    """upstream 默认开影子盾（S6）。编队类策略默认不开，打开 fleet_shadow 或证据开关才记。"""
    if not bool(getattr(cfg, "shadow", True)):
        return False
    if policy == "upstream":
        return True
    return bool(getattr(cfg, "fleet_shadow", False) or getattr(cfg, "shadow_env_evidence", False))


def scene_obstacles(card, cfg):
    """关着返回 None，DroneShadow 不读几何、不导入 shadow_env。"""
    if not bool(getattr(cfg, "shadow_env_evidence", False)):
        return None
    return card.obstacles


def bind_slot_shadow(slot, card, cfg, params: dict) -> None:
    slot.shadow = DroneShadow(
        slot.bridge.drone_id,
        slot.bridge.start_enu,
        slot.bridge.goal,
        params,
        scene_obstacles(card, cfg),
        contract_align=bool(getattr(cfg, "shadow_contract_align", False)),
        tracking_align=bool(getattr(cfg, "shadow_tracking_align", False)),
    )


def observe(session, slot, now_s: float, state: dict, *, planned: bool) -> None:
    """只读复验本拍计划和跟踪点。不改设定点。"""
    if slot.shadow is None or not session.shadow:
        return
    if session.fleet is not None:
        _observe_fleet(session, slot, now_s, state, planned=planned)
        return
    _observe_upstream(session, slot, now_s, state, planned=planned)


def family_of(code: str) -> str:
    raw = str(code)
    if raw.startswith("ENV:"):
        return "ENV"
    if raw.startswith("TIME:"):
        return "TIME"
    if raw.startswith("TRANSITION:"):
        return "TRANSITION"
    if raw in ("TRACKING_BOUND", "LAG_VIOLATION", "ODOMETRY_AGE_OR_NONFINITE"):
        return "TRACKING"
    return "OTHER"


def attribution(ticks: list[dict]) -> dict:
    """按契约码和跟踪码计 tick。一 tick 多码各计一次。不改判据。"""
    n = len(ticks)
    hist: dict[str, int] = {}
    family_ticks = {"ENV": 0, "TIME": 0, "TRANSITION": 0, "TRACKING": 0, "OTHER": 0}
    reject = env_reject = 0
    env_only = non_env = 0
    for tick in ticks:
        codes = [str(c) for c in (tick.get("reject_codes") or [])]
        # 跟踪对齐时 tick.would_reject 已忽略单独的 TRACKING_BOUND；不能再用「有码就拒」把测量打回去。
        if "would_reject" in tick:
            wr = bool(tick.get("would_reject"))
        else:
            wr = bool(codes)
        fams: set[str] = set()
        for code in codes:
            hist[code] = hist.get(code, 0) + 1
            fams.add(family_of(code))
        if wr:
            reject += 1
        if "ENV" in fams:
            env_reject += 1
        if wr and fams and fams <= {"ENV"}:
            env_only += 1
        if wr and (fams - {"ENV"}):
            non_env += 1
        for fam in fams:
            family_ticks[fam] += 1
    rate = 0.0 if n == 0 else reject / n
    return {
        "ticks": n,
        "would_reject": reject,
        "would_reject_rate": rate,
        "env_reject": env_reject,
        "env_reject_rate": None if n == 0 else env_reject / n,
        "reject_only_env": env_only,
        "reject_with_non_env": non_env,
        # 去掉环境码之后，剩下的码是否仍能把 would_reject 打满。
        "non_env_covers_would_reject": (reject > 0 and non_env == reject) or reject == 0,
        "histogram": dict(sorted(hist.items())),
        "family_ticks": family_ticks,
        "conditions": {code: CODE_CONDITIONS[code] for code in hist if code in CODE_CONDITIONS},
    }


def annotate_result(session, payload: dict) -> None:
    """关着不加字段。打开证据开关即偏离上游。编队类影子打开时附带拒绝码归因。"""
    cfg = session.cfg
    research = bool(getattr(cfg, "shadow_env_evidence", False) or getattr(cfg, "fleet_shadow", False))
    if research and session.shadow:
        payload["shadow_attribution"] = attribution(session._shadow_ticks)
    if bool(getattr(cfg, "shadow_contract_align", False)):
        payload["shadow_contract_align"] = True
        payload["upstream_deviation"] = True
    if bool(getattr(cfg, "shadow_tracking_align", False)):
        payload["shadow_tracking_align"] = True
        payload["upstream_deviation"] = True
    if not bool(getattr(cfg, "shadow_env_evidence", False)):
        return
    payload["shadow_env_evidence"] = True
    payload["upstream_deviation"] = True
    scenes = [
        slot.shadow._scene
        for slot in session.slots
        if slot.shadow is not None and slot.shadow._scene is not None
    ]
    payload["shadow_env_views"] = 0 if not scenes else int(scenes[0].n_views)
    payload["shadow_env_clearance_evals"] = int(sum(s.clearance_evals for s in scenes))
    payload["shadow_env_certificate_centers"] = int(sum(s.certificate_centers for s in scenes))


def _observe_upstream(session, slot, now_s: float, state: dict, *, planned: bool) -> None:
    obs = slot.bridge.last_observation
    if planned and obs is not None and slot.bridge.plan is not None:
        slot.shadow.on_selection(slot.bridge.plan, obs.frame, obs.state, float(now_s))
    if slot.bridge.plan is None:
        return
    body = _body_state(state, slot, float(now_s))
    cmd_p, _, _ = approved_pva(slot.bridge.plan, float(now_s))
    _tick(session, slot, body, cmd_p, float(now_s))


def _observe_fleet(session, slot, now_s: float, state: dict, *, planned: bool) -> None:
    # 编队钟从全体进入 PLAN 起算，计划和 L0 同一套时间。起飞段没有提交计划，不记。
    if session._plan_origin is None or session._formation_t is None:
        return
    now = float(now_s) - float(session._plan_origin)
    if now < -1e-12:
        return
    view = None
    shadow_view = getattr(session.fleet, "shadow_view", None)
    if callable(shadow_view):
        view = shadow_view(slot.bridge.drone_id)
    if view is not None:
        frame = _frame_at(slot.fleet_frame, now, cache_on=slot)
        body = _body_state(state, slot, now)
        slot.shadow.on_selection(view, frame, body, now)
    sampler = slot.formation_sampler
    if sampler is None:
        return
    body = _body_state(state, slot, now)
    cmd_p, _, _ = approved_pva(sampler, now)
    _tick(session, slot, body, cmd_p, now)


def _tick(session, slot, body: State, cmd_p, now: float) -> None:
    codes = slot.shadow.on_tick(body, cmd_p, now)
    session._shadow_ticks.append(
        {
            "uav": slot.bridge.drone_id,
            "t": float(now),
            "would_reject": codes_would_reject(
                codes, tracking_align=bool(getattr(session.cfg, "shadow_tracking_align", False)),
            ),
            "reject_codes": list(codes),
        }
    )


def _body_state(state: dict, slot, now: float) -> State:
    accel = state.get("acceleration_enu")
    if accel is None:
        accel = np.zeros(3)
    return State(
        float(now),
        np.asarray(state["position_enu"], dtype=float).copy(),
        np.asarray(state["velocity_enu"], dtype=float).copy(),
        np.asarray(accel, dtype=float).reshape(3).copy(),
        float(state.get("yaw_enu", slot.yaw)),
    )


class _StampedFrame:
    """同一张深度，时间戳改成影子时钟。不重新渲染。"""

    __slots__ = ("depth", "K", "stamp")

    def __init__(self, depth, k, stamp: float):
        self.depth = depth
        self.K = k
        self.stamp = float(stamp)


def _frame_at(frame, stamp: float, cache_on=None):
    if frame is None:
        src = getattr(cache_on, "_shadow_frame", None) if cache_on is not None else None
        return None if src is None else _StampedFrame(src.depth, src.K, stamp)
    # 只读已经渲染好的帧。不触发 LazyDepthFrame 的首次渲染：
    # 那会消耗场景卡相机的噪声序列，可能改变后续提案，也就不再是只读了。
    rendered = getattr(frame, "_frame", None)
    if rendered is None and hasattr(frame, "rendered") and not frame.rendered:
        src = getattr(cache_on, "_shadow_frame", None) if cache_on is not None else None
        return None if src is None else _StampedFrame(src.depth, src.K, stamp)
    src = rendered if rendered is not None else frame
    try:
        depth = src.depth
        k = src.K
    except (AttributeError, TypeError, ValueError):
        return None
    if cache_on is not None:
        cache_on._shadow_frame = src
    return _StampedFrame(depth, k, stamp)


__all__ = [
    "CODE_CONDITIONS",
    "ENV_RESULT_KEYS",
    "annotate_result",
    "attribution",
    "bind_slot_shadow",
    "family_of",
    "observe",
    "scene_obstacles",
    "shadow_enabled",
]
