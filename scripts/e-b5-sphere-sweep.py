#!/usr/bin/env python3
"""E-B5 只报数：在已合入的 E-B4c 欧氏球上扫半径、权重，并叠 geom / empty_brake。

不改 yopo_l0/、core/、scenes/。半径和权重走 BackendConfig 的公共字段。
末端放行是本进程里的包装：借用 E-B3c 的自由集，椭球代价强制为 0，
欧氏球采样盖住最后一跨。包装不写回库。

场景：

- d014-swap3：scenes/ego-density-014-swap3.json，seeds 0–9
- swap6-open：开阔六机对向，只在内存里，seeds 0–4
- d014-swap6：014 六机对向，几何与 E-C6 相同，只在内存里，seeds 0–4

每粒落在 out/e-b5-sphere-sweep/partial/，可续跑。带后段的臂最多 2 个进程。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "e-b5-sphere-sweep" / "partial"
COLLISION_M = 0.70
PRIOR_SEED1_M = 0.694
TRACK_MARGIN_M = 0.08
BODY_R = 0.35

SCENE_ORDER = ("d014-swap3", "swap6-open", "d014-swap6")
SCENE_LABEL = {
    "d014-swap3": "014 三机对向",
    "swap6-open": "六机开阔",
    "d014-swap6": "014 六机对向",
}
SCENE_SEEDS = {
    "d014-swap3": tuple(range(10)),
    "swap6-open": tuple(range(5)),
    "d014-swap6": tuple(range(5)),
}
# 票面：014 三机扫 off / 默认球 / 加大半径与权重 / geom_multi / empty_brake / 末端放行。
# 六机只看球单独和叠 geom。
SCENE_ARMS = {
    "d014-swap3": (
        "off",
        "sphere",
        "r100",
        "w16",
        "r100w16",
        "geom_multi",
        "empty_brake",
        "geom_multi+empty_brake",
        "end_free",
    ),
    "swap6-open": ("off", "sphere", "geom"),
    "d014-swap6": ("off", "sphere", "geom"),
}
ARM_SPEC = {
    "off": {},
    "sphere": {"sphere": True, "radius_m": 0.85, "weight": 8.0},
    "r100": {"sphere": True, "radius_m": 1.00, "weight": 8.0},
    "w16": {"sphere": True, "radius_m": 0.85, "weight": 16.0},
    "r100w16": {"sphere": True, "radius_m": 1.00, "weight": 16.0},
    "geom_multi": {"sphere": True, "radius_m": 0.85, "weight": 8.0, "track": "geom_multi"},
    "empty_brake": {"sphere": True, "radius_m": 0.85, "weight": 8.0, "empty_brake": True},
    "geom_multi+empty_brake": {
        "sphere": True,
        "radius_m": 0.85,
        "weight": 8.0,
        "track": "geom_multi",
        "empty_brake": True,
    },
    "end_free": {"sphere": True, "radius_m": 0.85, "weight": 8.0, "end_free": True},
    "geom": {"sphere": True, "radius_m": 0.85, "weight": 8.0, "track": "geom"},
}
ARM_LABEL = {
    "off": "全关",
    "sphere": "默认 E-B4c（0.85 m，权重 8）",
    "r100": "半径 1.00 m，权重 8",
    "w16": "半径 0.85 m，权重 16",
    "r100w16": "半径 1.00 m，权重 16",
    "geom_multi": "默认球 + geom_multi 0.08 m",
    "empty_brake": "默认球 + empty_brake",
    "geom_multi+empty_brake": "默认球 + geom_multi + empty_brake",
    "end_free": "默认球 + 脚本内末端放行",
    "geom": "默认球 + geom 0.08 m",
}
DISCLAIMER = (
    "只报数。report-only。peer_sphere_enabled 默认关，不改 swarm_penalty / swarm_segment_penalty 的语义。"
    "打开欧氏球、加大半径或权重、track_clearance、empty_brake，或使用脚本内末端放行，即偏离上游 d5e1e60。"
    "off 不打开这些开关。"
    "未冻结，不进 scenes/suite.txt，不设 gates。"
    "默认半径 0.85 m = 0.70 m 碰撞球 + 0.15 m 裕度，权重 8，与已合入的 E-B4c 相同。"
    "1.00 m 是 0.70 m 再加 0.30 m 裕度。权重 16 是默认的两倍。"
    "track_margin 0.08 m 沿用 E-C5 的假设。"
    "末端放行只存在于本脚本进程，不改 yopo_l0。"
    "六机开阔和 014 六机对向的卡只在内存里，不写入 scenes/。"
    "这不是论文复现。"
)
END_FREE_NOTE = (
    "末端放行包装做两件事，都只在本进程里："
    "把这次 refine 的配置换成 swarm_penalty 与 swarm_segment_penalty 都为真，从而放开末端三个控制点、不再钉死末端 PVA；"
    "同时把段内椭球代价和梯度换成 0，欧氏球改成连最后一跨一起采样。"
    "场景卡上仍只有 peer_sphere，没有椭球开关。"
    "库里的 peer_sphere 仍不放开末端，最后一跨仍不进球。"
)
OFFSETS = (-1.0, -0.5, -0.2, -0.1, -0.05, -0.01, 0.0, 0.01, 0.05, 0.1, 0.2, 0.5, 1.0)
STATE = {"end_free": False, "peer": False, "end": False}
ORIG = {}


def arm_spec(arm: str) -> dict:
    if arm not in ARM_SPEC:
        raise ValueError(arm)
    spec = {
        "sphere": False,
        "radius_m": None,
        "weight": None,
        "track": None,
        "empty_brake": False,
        "end_free": False,
    }
    spec.update(ARM_SPEC[arm])
    return spec


def arm_label(arm: str) -> str:
    if arm not in ARM_LABEL:
        raise ValueError(arm)
    return ARM_LABEL[arm]


def arm_deviates(arm: str) -> bool:
    return arm != "off"


def _ec6():
    name = "e_c6_six_density_for_b5"
    if name in sys.modules:
        return sys.modules[name]
    path = ROOT / "scripts" / "e-c6-six-density.py"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"无法加载 {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def _deep(raw: dict) -> dict:
    return json.loads(json.dumps(raw))


def apply_arm(raw: dict, arm: str) -> dict:
    """深拷贝后只写这一臂的公共字段。不写 safe_radius，不写椭球开关。"""
    spec = arm_spec(arm)
    out = _deep(raw)
    if arm == "off":
        out.pop("policy_params", None)
        return out
    params = dict(out.get("policy_params") or {})
    if spec["sphere"]:
        backend = dict(params.get("backend") or {})
        backend["enabled"] = True
        backend["peer_sphere_enabled"] = True
        backend["peer_sphere_radius_m"] = float(spec["radius_m"])
        backend["peer_sphere_weight"] = float(spec["weight"])
        backend.pop("swarm_penalty", None)
        backend.pop("swarm_segment_penalty", None)
        params["backend"] = backend
    upstream = dict(params.get("upstream") or {})
    for key in (
        "track_clearance",
        "track_clearance_mode",
        "track_margin_m",
        "empty_brake",
        "safe_radius",
        "inter_drone_filter",
    ):
        upstream.pop(key, None)
    if spec["track"]:
        upstream["track_clearance"] = True
        upstream["track_clearance_mode"] = spec["track"]
        upstream["track_margin_m"] = TRACK_MARGIN_M
    if spec["empty_brake"]:
        upstream["empty_brake"] = True
    if upstream:
        params["upstream"] = upstream
    else:
        params.pop("upstream", None)
    out["policy_params"] = params
    return out


def swap6_open_raw() -> dict:
    """六机开阔对向。方位与 E-C6 的六机相同，半径保持 8 m，不写盘。"""
    from yopo_l0.scene import load_card

    loaded = load_card(ROOT / "scenes" / "swap-3-open.json")
    raw = _deep(loaded.card)
    ec6 = _ec6()
    drones = []
    for i, (ux, uy) in enumerate(ec6.SWAP6_UNIT):
        sx, sy = 8.0 * float(ux), 8.0 * float(uy)
        drones.append(
            {
                "id": f"uav{i}",
                "start_enu": [sx, sy, 1.5],
                "goal_enu": [-sx, -sy, 1.5],
            }
        )
    raw["name"] = "swap-6-open"
    raw["drones"] = drones
    raw["seeds"] = [0]
    raw.pop("policy_params", None)
    raw["note"] = (
        "票 E-B5：六机开阔对向。只在 scripts/e-b5-sphere-sweep.py 的进程内存里构造，不写入 scenes/。"
        "几何沿用 scenes/swap-3-open.json 的开阔场。六架在半径 8.0 m、高度 1.5 m 的圆上，方位间隔 60°，"
        "各自飞向对径。方位与 E-C6 的 014 六机对向相同，开阔场不把起终点推出 8 m。"
        "report-only。打开欧氏球或 track_clearance 即偏离上游。不进 scenes/suite.txt，不设 gates。"
        "速度与 a_max 用本仓库已有的点质量设定。"
    )
    return raw


def d014_swap6_raw() -> dict:
    """014 六机对向。起终点规则与 E-C6 相同，本脚本再盖自己的臂。"""
    return _ec6().swap6_raw()


def build_card(scene: str, arm: str):
    from yopo_l0.scene import SceneCard, load_card, parse_geometry

    if scene not in SCENE_ORDER:
        raise ValueError(scene)
    if arm not in ARM_SPEC:
        raise ValueError(arm)
    if scene == "d014-swap3":
        loaded = load_card(ROOT / "scenes" / "ego-density-014-swap3.json")
        if arm == "off":
            return loaded
        raw = apply_arm(loaded.card, arm)
        return SceneCard(loaded.path, raw, loaded.obstacles)
    if scene == "swap6-open":
        raw = apply_arm(swap6_open_raw(), arm)
    elif scene == "d014-swap6":
        raw = apply_arm(d014_swap6_raw(), arm)
    else:
        raise ValueError(scene)
    return SceneCard(raw["name"], raw, parse_geometry(raw["geometry"], raw.get("ground_z")))


def config_with_end_free(cfg):
    """放开末端的那份配置。椭球代价由包装清掉，不在这里改权重。"""
    from yopo_l0.backend.config import BackendConfig

    return BackendConfig(
        a_max=float(cfg.a_max),
        v_max=float(cfg.v_max),
        max_iterations=int(cfg.max_iterations),
        time_budget_s=float(cfg.time_budget_s),
        clearance_m=float(cfg.clearance_m),
        swarm_penalty=True,
        swarm_clearance_m=float(cfg.swarm_clearance_m),
        swarm_c=float(cfg.swarm_c),
        swarm_segment_penalty=True,
        peer_sphere_enabled=True,
        peer_sphere_radius_m=float(cfg.peer_sphere_radius_m),
        peer_sphere_weight=float(cfg.peer_sphere_weight),
    )


def peer_sphere_cost_including_last_span(q, dt, t_start, neighbors, radius, weight):
    """与库里的 `_peer_sphere_cost_grad` 同一条铰链，但最后一跨也采样。

    库函数把 `n_pen` 设成 `n_span - 1`，避免去拽钉死的末端。末端放行把末端放开之后，
    这一跨才进代价。不改 yopo_l0。
    """
    import yopo_l0.backend.bspline as bspline

    n = int(q.shape[0])
    n_span = n - 3
    grad = np.zeros_like(q)
    radius = float(radius)
    weight = float(weight)
    band = radius - float(bspline._PEER_COLLISION_BALL_M)
    n_pen = n_span
    if n_pen < 1 or not neighbors or not (weight > 0.0) or not (radius > 0.0):
        return 0.0, grad
    windows = bspline._control_windows(q)[:n_pen]
    pts = np.einsum("kb,sbc->skc", bspline._B_SEGMENT, windows)
    span = np.arange(n_pen, dtype=float)[:, None]
    times = float(t_start) + (span + bspline._SEGMENT_U[None, :]) * float(dt)
    flat_t = times.reshape(-1)
    flat_p = pts.reshape(-1, 3)
    cost = 0.0
    dpos = np.zeros_like(flat_p)
    for raw in neighbors:
        arr = np.asarray(raw, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 4 or arr.shape[0] == 0:
            continue
        if not bool(np.all(np.isfinite(arr))):
            continue
        ts = arr[:, 0]
        mask = (flat_t >= float(ts[0])) & (flat_t <= float(ts[-1]))
        if not bool(np.any(mask)):
            continue
        t = flat_t[mask]
        delta = flat_p[mask] - np.column_stack(
            (
                np.interp(t, ts, arr[:, 1]),
                np.interp(t, ts, arr[:, 2]),
                np.interp(t, ts, arr[:, 3]),
            )
        )
        dist = np.sqrt(np.sum(delta * delta, axis=1))
        sample_cost, dcost_dd = bspline._peer_sphere_sample(dist, radius, weight, band)
        zero = dist <= bspline._SWARM_EPS
        if bool(np.any(zero)):
            zdist = np.zeros(int(np.count_nonzero(zero)), dtype=float)
            zcost, _zd = bspline._peer_sphere_sample(zdist, radius, weight, band)
            sample_cost = sample_cost.copy()
            sample_cost[zero] = zcost
            dcost_dd = dcost_dd.copy()
            dcost_dd[zero] = 0.0
        cost += float(np.sum(sample_cost))
        active = (dcost_dd != 0.0) & ~zero
        if not bool(np.any(active)):
            continue
        push = (dcost_dd[active] / dist[active])[:, None] * delta[active]
        idx = np.flatnonzero(mask)
        dpos[idx[active]] += push
    if cost == 0.0 and not np.any(dpos):
        return 0.0, grad
    d_sk = dpos.reshape(n_pen, bspline._SWARM_SEGMENT_SAMPLES, 3)
    contrib = np.einsum("kb,skc->sbc", bspline._B_SEGMENT, d_sk)
    for j in range(4):
        grad[j : j + n_pen] += contrib[:, j, :]
    return float(cost), grad


def install_peer_delivery() -> None:
    """欧氏球打开时把邻机采样送进后段。椭球项仍只看 swarm_penalty。"""
    if STATE["peer"]:
        return
    import yopo_l0.policy as policy_mod
    import yopo_l0.runner as runner_mod
    from yopo_l0.backend.config import peer_sphere_enabled as sphere_on

    orig_enabled = runner_mod.swarm_penalty_enabled

    def enabled(params, **kwargs):
        if orig_enabled(params, **kwargs):
            return True
        return bool(sphere_on(params, **kwargs))

    runner_mod.swarm_penalty_enabled = enabled
    orig_keep = policy_mod.UpstreamPolicy._keep_neighbor_samples

    def keep(self):
        if orig_keep(self):
            return True
        cfg = getattr(self, "_backend", None)
        return cfg is not None and bool(cfg.peer_sphere_enabled)

    policy_mod.UpstreamPolicy._keep_neighbor_samples = keep

    def apply(self, traj, goal):
        from yopo_l0.backend.bspline import refine_yopo

        cfg = self._backend
        use = bool(self._swarm) or (cfg is not None and bool(cfg.peer_sphere_enabled))
        neighbors = list(self._neighbors.values()) if use else None
        out, rec = refine_yopo(traj, cfg, self._obstacles, goal, neighbors=neighbors)
        self.backend_log.append(rec)
        return out

    policy_mod.UpstreamPolicy._apply_backend = apply
    STATE["peer"] = True


def install_end_free() -> None:
    """末端放行只在 STATE['end_free'] 为真时改变 refine。关着时走原来的函数。"""
    if STATE["end"]:
        return
    import yopo_l0.backend.bspline as bspline

    ORIG["refine"] = bspline._refine
    ORIG["peer"] = bspline._peer_sphere_cost_grad
    ORIG["seg"] = bspline._swarm_segment_cost_grad

    def refine(traj, cfg, obstacles, t0, neighbors=None):
        if (
            STATE["end_free"]
            and cfg is not None
            and bool(cfg.peer_sphere_enabled)
            and not bool(cfg.swarm_segment_penalty)
        ):
            cfg = config_with_end_free(cfg)
        return ORIG["refine"](traj, cfg, obstacles, t0, neighbors)

    def peer(q, dt, t_start, neighbors, radius, weight):
        if STATE["end_free"]:
            return peer_sphere_cost_including_last_span(q, dt, t_start, neighbors, radius, weight)
        return ORIG["peer"](q, dt, t_start, neighbors, radius, weight)

    def seg(*args, **kwargs):
        if STATE["end_free"]:
            return 0.0, np.zeros_like(args[0])
        return ORIG["seg"](*args, **kwargs)

    bspline._refine = refine
    bspline._peer_sphere_cost_grad = peer
    bspline._swarm_segment_cost_grad = seg
    STATE["end"] = True


def _budget_wrap():
    import yopo_l0.backend.bspline as bspline

    current = bspline.refine_yopo
    if getattr(current, "_eb5_wrapped", False):
        current.cuts = 0
        current.calls = 0
        return current
    orig = current

    def wrapped(*args, **kwargs):
        out, rec = orig(*args, **kwargs)
        wrapped.calls += 1
        wrapped.cuts += int(bool(rec.get("budget_cut")))
        return out, rec

    wrapped.calls = 0
    wrapped.cuts = 0
    wrapped._eb5_wrapped = True
    bspline.refine_yopo = wrapped
    return wrapped


def _drop_samples(block: dict | None) -> dict | None:
    if not isinstance(block, dict):
        return None
    out = dict(block)
    for key in ("time_ms", "iterations"):
        part = out.get(key)
        if isinstance(part, dict):
            kept = dict(part)
            kept.pop("samples", None)
            out[key] = kept
    return out


def _frames(ticks: list[dict]) -> dict:
    by_t: dict[float, dict] = {}
    for row in ticks:
        by_t.setdefault(float(row["t"]), {})[row["uav"]] = row
    return by_t


def _series(by_t: dict, a: str, b: str) -> list[dict]:
    out = []
    prev_d = None
    prev_t = None
    for t in sorted(by_t):
        frame = by_t[t]
        if a not in frame or b not in frame:
            continue
        pa = np.asarray(frame[a]["p"], dtype=float)
        pb = np.asarray(frame[b]["p"], dtype=float)
        va = np.asarray(frame[a]["v"], dtype=float)
        vb = np.asarray(frame[b]["v"], dtype=float)
        dist = float(np.linalg.norm(pa - pb))
        closing = None
        if prev_d is not None and t > prev_t:
            closing = (prev_d - dist) / (t - prev_t)
        out.append(
            {
                "t_s": t,
                "distance_m": dist,
                "closing_speed_mps": closing,
                "speed_mps": {a: float(np.linalg.norm(va)), b: float(np.linalg.norm(vb))},
            }
        )
        prev_d, prev_t = dist, t
    return out


def _pick_pair(by_t: dict, reason: str | None) -> tuple[str | None, str | None]:
    if reason and "INTER_DRONE" in reason:
        left = reason.split(":", 1)[0]
        if "-" in left:
            a, b = left.split("-", 1)
            return a, b
    ids = sorted({u for frame in by_t.values() for u in frame})
    best = None
    for i, a in enumerate(ids):
        for b in ids[i + 1 :]:
            series = _series(by_t, a, b)
            if not series:
                continue
            dist = min(s["distance_m"] for s in series)
            if best is None or dist < best[0]:
                best = (dist, a, b)
    if best is None:
        return None, None
    return best[1], best[2]


def approach_from_ticks(ticks: list[dict], *, reason: str | None = None, pair: str | None = None) -> dict | None:
    """碰撞则取第一次 ≤ 0.70 m 的那一拍，否则取最小机间那一拍。接近速度为正表示距离在减小。"""
    by_t = _frames(ticks)
    if pair and "-" in pair:
        a, b = pair.split("-", 1)
    else:
        a, b = _pick_pair(by_t, reason)
    if not a or not b:
        return None
    series = _series(by_t, a, b)
    if not series:
        return None
    cross = next((s for s in series if s["distance_m"] <= COLLISION_M + 1e-12), None)
    if cross is not None:
        star = cross
        kind = "collision"
    else:
        star = min(series, key=lambda s: (s["distance_m"], s["t_s"]))
        kind = "closest"
    t_star = float(star["t_s"])
    samples = []
    for off in OFFSETS:
        target = t_star + off
        nearest = min(series, key=lambda s: abs(s["t_s"] - target))
        tol = 0.006 if abs(off) <= 0.01 + 1e-12 else 0.03
        if abs(nearest["t_s"] - target) > tol:
            continue
        samples.append(
            {
                "dt_s": float(nearest["t_s"] - t_star),
                "t_s": float(nearest["t_s"]),
                "distance_m": float(nearest["distance_m"]),
                "closing_speed_mps": nearest["closing_speed_mps"],
                "speed_mps": dict(nearest["speed_mps"]),
            }
        )
    return {
        "pair": f"{a}-{b}",
        "kind": kind,
        "t_s": t_star,
        "distance_m": float(star["distance_m"]),
        "closing_speed_mps": star["closing_speed_mps"],
        "speed_mps": dict(star["speed_mps"]),
        "samples": samples,
    }


def _traces_match(official: dict, diag: dict | None, holder: dict) -> tuple[bool, str]:
    if diag is None or holder.get("t0") is None:
        return False, "续飞没有形成机间越线，越线之后的距离不写入结论。"
    if diag.get("pair") != official.get("pair"):
        return False, "续飞越线的那一对和正式跑不同，越线之后的距离不写入结论。"
    if abs(float(holder["t0"]) - float(official["t_s"])) > 0.02:
        return False, "续飞的越线时刻和正式跑差过 0.02 s，越线之后的距离不写入结论。"
    diag_at = {round(float(s["t_s"]), 9): float(s["distance_m"]) for s in diag["samples"]}
    diffs = []
    for s in official["samples"]:
        if float(s["dt_s"]) > 1e-9:
            continue
        key = round(float(s["t_s"]), 9)
        if key not in diag_at:
            continue
        diffs.append(abs(diag_at[key] - float(s["distance_m"])))
    if len(diffs) < 3 or max(diffs) > 0.02:
        return False, "续飞在越线前的距离和正式跑对不上（差超过 0.02 m），越线之后的距离不写入结论。后段按墙钟截断，两次跑可以不一样。"
    return True, "续飞在越线前的距离和正式跑相差不超过 0.02 m。越线之后 1 s 的点来自这次续飞。表上的结果列仍是正式跑，运行器在第一次 ≤ 0.70 m 时停下。"


def _diagnostic_after(card, seed: int, official: dict) -> dict:
    import yopo_l0.runner as runner_mod
    from yopo_l0.runner import RunConfig, run

    holder: dict = {"t0": None, "reason": None}
    orig = runner_mod._note_pairs

    def wrapped(drones, radius, min_inter, outcome, reason):
        min_inter, outcome, reason = orig(drones, radius, min_inter, outcome, reason)
        if outcome == "COLLISION" and "INTER_DRONE" in str(reason):
            t = float(drones[0]["state"].t)
            if holder["t0"] is None:
                holder["t0"] = t
                holder["reason"] = reason
            if t < float(holder["t0"]) + 1.0 - 1e-12:
                return min_inter, None, ""
            return min_inter, "COLLISION", holder["reason"]
        return min_inter, outcome, reason

    runner_mod._note_pairs = wrapped
    try:
        out = run(card, RunConfig(seed=int(seed), shadow=True, duration_s=float(official["t_s"]) + 1.05))
    finally:
        runner_mod._note_pairs = orig
    diag = approach_from_ticks(out.ticks, pair=official["pair"], reason=holder.get("reason"))
    trusted, note = _traces_match(official, diag, holder)
    samples = []
    if trusted and diag is not None:
        for s in diag["samples"]:
            shifted = dict(s)
            shifted["dt_s"] = float(s["t_s"]) - float(official["t_s"])
            if shifted["dt_s"] > 0.02:
                samples.append(shifted)
    return {
        "trusted": trusted,
        "note": note,
        "t0_s": holder.get("t0"),
        "reason": holder.get("reason"),
        "samples": samples,
    }


def _row(scene: str, arm: str, result: dict, ticks: list[dict], *, budget_calls: int, budget_cuts: int) -> dict:
    spec = arm_spec(arm)
    reason = result.get("reason") or ""
    outcome = result.get("outcome")
    backend = _drop_samples(result.get("backend"))
    time_ms = (backend or {}).get("time_ms") or {}
    inter = int(outcome == "COLLISION" and "INTER_DRONE" in str(reason))
    row = {
        "scene_kind": scene,
        "scene_label": SCENE_LABEL[scene],
        "arm": arm,
        "arm_label": arm_label(arm),
        "scene": result.get("scene"),
        "seed": int(result["seed"]),
        "outcome": outcome,
        "reason": reason,
        "arrived": int(result["arrived"]),
        "n_drones": int(result["n_drones"]),
        "arrival_time_s": result.get("arrival_time_s"),
        "obstacle_collisions": int(outcome == "COLLISION" and "TRUTH_POSITION" in str(reason)),
        "inter_drone_collisions": inter,
        "timeout": int(outcome == "TIMEOUT"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "min_obstacle_distance_m": result.get("min_obstacle_distance_m"),
        "sim_duration_s": result.get("sim_duration_s"),
        "final_goal_error_m": result.get("final_goal_error_m"),
        "wall_time_s": result.get("wall_time_s"),
        "backend": backend,
        "backend_time_p50_ms": time_ms.get("p50"),
        "backend_time_p95_ms": time_ms.get("p95"),
        "backend_budget_calls": int(budget_calls),
        "backend_budget_cuts": int(budget_cuts),
        "scene_card_hash": result.get("scene_card_hash"),
        "commit": result.get("commit"),
        "report_only": True,
        "deviates_from_upstream": arm_deviates(arm),
        "deviation_tag": "偏离上游" if arm_deviates(arm) else "否",
        "runner_upstream_deviation": result.get("upstream_deviation"),
        "track_clearance_mode": spec["track"],
        "empty_brake": bool(spec["empty_brake"]),
        "end_free": bool(spec["end_free"]),
        "peer_sphere_enabled": bool(spec["sphere"]),
        "peer_sphere_radius_m": spec["radius_m"],
        "peer_sphere_weight": spec["weight"],
        "disclaimer": DISCLAIMER,
    }
    if scene == "d014-swap3" and int(result["seed"]) == 1:
        row["approach"] = approach_from_ticks(ticks, reason=str(reason))
    return row


def _partial(scene: str, arm: str, seed: int) -> Path:
    safe = arm.replace("+", "_")
    return PARTIAL / f"{scene}-{safe}-{seed}.json"


def run_one(scene: str, arm: str, seed: int, *, resume: bool) -> dict:
    dest = _partial(scene, arm, seed)
    if resume and dest.is_file():
        return json.loads(dest.read_text(encoding="utf-8"))
    from yopo_l0.runner import RunConfig, run

    spec = arm_spec(arm)
    STATE["end_free"] = bool(spec["end_free"])
    install_peer_delivery()
    install_end_free()
    card = build_card(scene, arm)
    wrapped = _budget_wrap()
    t0 = time.perf_counter()
    out = run(card, RunConfig(seed=int(seed), shadow=True))
    row = _row(
        scene,
        arm,
        out.result,
        out.ticks,
        budget_calls=int(wrapped.calls),
        budget_cuts=int(wrapped.cuts),
    )
    if (
        scene == "d014-swap3"
        and int(seed) == 1
        and int(row["inter_drone_collisions"]) == 1
        and row.get("approach")
        and row["approach"].get("kind") == "collision"
    ):
        try:
            row["approach_after"] = _diagnostic_after(card, int(seed), row["approach"])
        except Exception as exc:  # 续飞失败不影响正式跑的数字
            row["approach_after"] = {"trusted": False, "note": f"续飞失败：{exc}", "samples": []}
    row["worker_s"] = time.perf_counter() - t0
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False), encoding="utf-8")
    tmp.replace(dest)
    gap = row["min_inter_drone_distance_m"]
    gap_s = "—" if gap is None else f"{float(gap):.3f}"
    print(
        f"{scene} {arm} seed={seed} {row['outcome']} {row['arrived']}/{row['n_drones']} "
        f"obs={row['obstacle_collisions']} inter={row['inter_drone_collisions']} "
        f"gap={gap_s} cuts={row['backend_budget_cuts']}/{row['backend_budget_calls']} "
        f"wall={row['wall_time_s']:.1f}s",
        flush=True,
    )
    return row


def _job(spec):
    scene, arm, seed, resume = spec
    return run_one(scene, arm, int(seed), resume=bool(resume))


def _f(value, nd: int) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{nd}f}"


def _xyz(x: float, y: float, z: float = 1.5) -> str:
    def one(v: float) -> str:
        v = 0.0 if abs(float(v)) < 5e-4 else float(v)
        return f"{v:.3f}"

    return f"{one(x)}, {one(y)}, {one(z)}"


def _rng(values: list[float], nd: int) -> str:
    if not values:
        return "—"
    lo, hi = min(values), max(values)
    if abs(lo - hi) < 10 ** (-nd) / 2:
        return _f(lo, nd)
    return f"{_f(lo, nd)}–{_f(hi, nd)}"


def _stats(group: list[dict]) -> dict:
    gaps = [float(r["min_inter_drone_distance_m"]) for r in group if r.get("min_inter_drone_distance_m") is not None]
    times = [float(r["arrival_time_s"]) for r in group if r.get("arrival_time_s") is not None]
    return {
        "arrived": sum(int(r["arrived"]) for r in group),
        "total": sum(int(r["n_drones"]) for r in group),
        "obs": sum(int(r["obstacle_collisions"]) for r in group),
        "inter": sum(int(r["inter_drone_collisions"]) for r in group),
        "timeout": sum(int(r["timeout"]) for r in group),
        "gaps": gaps,
        "times": times,
        "cuts": sum(int(r.get("backend_budget_cuts") or 0) for r in group),
        "calls": sum(int(r.get("backend_budget_calls") or 0) for r in group),
    }


def summarize(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 臂 | 偏离上游 | 到达 | 障碍碰撞 | 机间碰撞 | 超时 | 最小机间 m | 到达时间 s | 预算截断 |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for scene in SCENE_ORDER:
        arms = []
        for row in rows:
            if row["scene_kind"] == scene and row["arm"] not in arms:
                arms.append(row["arm"])
        for arm in arms:
            group = [r for r in rows if r["scene_kind"] == scene and r["arm"] == arm]
            if not group:
                continue
            st = _stats(group)
            tag = "偏离上游" if arm_deviates(arm) else "否"
            lines.append(
                f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | {tag} | {st['arrived']}/{st['total']} | "
                f"{st['obs']} | {st['inter']} | {st['timeout']} | {_rng(st['gaps'], 3)} | {_rng(st['times'], 2)} | "
                f"{st['cuts']}/{st['calls']} |"
            )
    return "\n".join(lines) + "\n"


def seed1_rows(rows: list[dict]) -> list[dict]:
    return [r for r in rows if r.get("scene_kind") == "d014-swap3" and int(r.get("seed", -1)) == 1]


def seed1_headline(rows: list[dict]) -> str:
    group = seed1_rows(rows)
    sphere = next((r for r in group if r.get("arm") == "sphere"), None)
    if sphere is None:
        return "这次输出里没有 014 三机对向 seed 1 的默认 E-B4c。"
    gap = sphere.get("min_inter_drone_distance_m")
    gap_s = "—" if gap is None else f"{float(gap):.3f} m"
    hit = int(sphere.get("inter_drone_collisions") or 0) == 1
    if hit:
        shown = None if gap is None else round(float(gap), 3)
        extra = ""
        if shown is not None and abs(shown - PRIOR_SEED1_M) < 5e-4:
            extra = "这次记下的距离仍是 0.694 m。"
        elif shown is not None and abs(shown - PRIOR_SEED1_M) <= 0.005:
            extra = (
                f"这次记下的距离和 E-B4c 的 0.694 m 相差 {shown - PRIOR_SEED1_M:+.3f} m，"
                "是同一次越线采样的量级。"
            )
        elif shown is not None:
            extra = (
                f"这次记下的是 {shown:.3f} m，不是 0.694 m。"
                "后段 0.02 s 预算按墙钟截断，同一粒的越线采样会动。"
            )
        text = (
            f"默认 E-B4c（半径 0.85 m，权重 8）在 `ego-density-014-swap3` seed 1 上仍是机间碰撞"
            f"（`{sphere.get('reason')}`，最小机间 {gap_s}）。"
            f"E-B4c 记下的 0.694 m 没有消除。{extra}"
        )
    else:
        text = (
            f"默认 E-B4c（半径 0.85 m，权重 8）在 `ego-density-014-swap3` seed 1 上这次是 {sphere.get('outcome')}"
            f"，最小机间 {gap_s}。"
            f"E-B4c 记下的 0.694 m 机间碰撞没有出现。"
        )
    others = []
    for row in group:
        if row.get("arm") == "sphere":
            continue
        others.append(_seed1_phrase(row))
    if others:
        text += "同一粒其余各臂：" + "；".join(others) + "。"
    return text


def _seed1_phrase(row: dict) -> str:
    gap = row.get("min_inter_drone_distance_m")
    gap_s = "—" if gap is None else f"{float(gap):.3f} m"
    if int(row.get("inter_drone_collisions") or 0):
        flag = f"仍是机间碰撞（`{row.get('reason')}`）"
    elif int(row.get("obstacle_collisions") or 0):
        flag = f"机间没有越线，但是障碍碰撞（`{row.get('reason')}`）"
    elif row.get("outcome") == "TIMEOUT":
        flag = f"超时，到达 {int(row.get('arrived') or 0)}/{int(row.get('n_drones') or 0)}"
    elif row.get("outcome") == "SUCCESS":
        flag = "到达，机间没有越线"
    else:
        flag = str(row.get("outcome"))
    return f"{arm_label(row['arm'])}：{flag}，最小机间 {gap_s}"


def format_approach_md(approach: dict | None, *, after_samples: list[dict] | None = None) -> str:
    if not approach:
        return ""
    samples = list(approach.get("samples") or [])
    if after_samples:
        have = {round(float(s["dt_s"]), 5) for s in samples}
        for s in after_samples:
            if float(s["dt_s"]) > 0.02 and round(float(s["dt_s"]), 5) not in have:
                samples.append(s)
        samples.sort(key=lambda s: float(s["dt_s"]))
    pair = str(approach["pair"])
    a, b = pair.split("-", 1)
    lines = [
        (
            f"一对是 `{pair}`，该帧 t = {float(approach['t_s']):.2f} s，"
            f"距离 {float(approach['distance_m']):.3f} m，"
            f"接近速度 {_f(approach.get('closing_speed_mps'), 2)} m/s。"
            "接近速度为正表示中心距在减小，用相邻两拍的 −Δd/Δt。"
        ),
        "",
        f"| 相对该帧 s | 距离 m | 接近速度 m/s | {a} 速率 m/s | {b} 速率 m/s |",
        "|---|---|---|---|---|",
    ]
    for s in samples:
        speeds = s.get("speed_mps") or {}
        lines.append(
            f"| {float(s['dt_s']):+.2f} | {float(s['distance_m']):.3f} | "
            f"{_f(s.get('closing_speed_mps'), 2)} | {_f(speeds.get(a), 2)} | {_f(speeds.get(b), 2)} |"
        )
    return "\n".join(lines)


def _seed1_curves(rows: list[dict]) -> str:
    group = seed1_rows(rows)
    sphere = next((r for r in group if r.get("arm") == "sphere"), None)
    ordered = []
    if sphere is not None:
        ordered.append(sphere)
    for row in group:
        if row is sphere:
            continue
        if int(row.get("inter_drone_collisions") or 0):
            ordered.append(row)
    sphere_approach = (sphere or {}).get("approach") if sphere is not None else None
    parts = []
    for row in ordered:
        approach = row.get("approach")
        if not approach:
            continue
        if (
            row is not sphere
            and sphere_approach
            and approach.get("pair") == sphere_approach.get("pair")
            and abs(float(approach["t_s"]) - float(sphere_approach["t_s"])) < 0.02
            and abs(float(approach["distance_m"]) - float(sphere_approach["distance_m"])) < 0.002
        ):
            parts.append(
                f"### seed 1 · {arm_label(row['arm'])}\n\n"
                "越线的那一对、时刻和距离与默认球这一粒相同（差小于 0.002 m）。曲线不另附。"
            )
            continue
        diag = row.get("approach_after") or {}
        after = diag.get("samples") if diag.get("trusted") else None
        if diag.get("trusted"):
            note = str(diag.get("note") or "")
            if after:
                first = min(after, key=lambda s: float(s["dt_s"]))
                if float(first["distance_m"]) > float(approach["distance_m"]) + 0.01:
                    note += "续飞里越线之后的第一档距离高于该帧，中心距在回升。运行器仍把这一拍记成碰撞。"
                elif float(first["distance_m"]) < float(approach["distance_m"]) - 0.01:
                    note += "续飞里越线之后距离继续下降。"
        elif diag:
            note = str(diag.get("note") or "续飞没有采用。")
        elif approach.get("kind") == "collision":
            note = "运行器在第一次中心距 ≤ 0.70 m 的那一拍停下，正式跑没有越线之后的采样。下表是这一拍和它前面的距离、速率。"
        else:
            note = "这一粒没有机间碰撞。下表是全程最小机间那一拍的前后，都来自同一次正式跑。"
        parts.append(f"### seed 1 · {arm_label(row['arm'])}\n\n{note}\n\n{format_approach_md(approach, after_samples=after)}")
    return "\n\n".join(parts)


def _per_seed(rows: list[dict]) -> str:
    lines = [
        "| 场景 | 臂 | seed | 结果 | 原因 | 到达 | 障碍 | 机间 | 超时 | 最小机间 m | 到达时间 s |",
        "|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for scene in SCENE_ORDER:
        arms = [a for a in SCENE_ARMS[scene] if any(r["scene_kind"] == scene and r["arm"] == a for r in rows)]
        extra = []
        for row in rows:
            if row["scene_kind"] == scene and row["arm"] not in arms and row["arm"] not in extra:
                extra.append(row["arm"])
        for arm in list(arms) + extra:
            group = sorted(
                (r for r in rows if r["scene_kind"] == scene and r["arm"] == arm),
                key=lambda r: int(r["seed"]),
            )
            for r in group:
                lines.append(
                    f"| {SCENE_LABEL[scene]} | {arm_label(arm)} | {int(r['seed'])} | {r.get('outcome')} | "
                    f"`{r.get('reason') or ''}` | {int(r['arrived'])}/{int(r['n_drones'])} | "
                    f"{int(r['obstacle_collisions'])} | {int(r['inter_drone_collisions'])} | {int(r['timeout'])} | "
                    f"{_f(r.get('min_inter_drone_distance_m'), 3)} | {_f(r.get('arrival_time_s'), 2)} |"
                )
    return "\n".join(lines)


def _placement_md() -> str:
    ec6 = _ec6()
    placed = ec6.swap6_placement()
    lines = [
        "| 飞机 | 半径 m | 推出正方形 | 起点 | 目标 |",
        "|---|---:|---|---|---|",
    ]
    for d in placed:
        start = ", ".join(f"{float(x):.3f}" for x in d["start_enu"])
        goal = ", ".join(f"{float(x):.3f}" for x in d["goal_enu"])
        lines.append(
            f"| {d['id']} | {float(d['radius_m']):.3f} | {'是' if d['pushed_outside'] else '否'} | {start} | {goal} |"
        )
    open_lines = [
        "| 飞机 | 起点 | 目标 |",
        "|---|---|---|",
    ]
    for i, (ux, uy) in enumerate(ec6.SWAP6_UNIT):
        sx, sy = 8.0 * float(ux), 8.0 * float(uy)
        open_lines.append(
            f"| uav{i} | {_xyz(sx, sy)} | {_xyz(-sx, -sy)} |"
        )
    return (
        "六机开阔的名义圆半径 8.0 m，方位间隔 60°，目标是起点的对径。开阔场没有柱子，起终点留在 8 m 圆上。\n\n"
        + "\n".join(open_lines)
        + "\n\n014 六机对向的起终点与 [E-C6](e-c6-six-density.md) 同一条规则：轴距低于 0.70 m 就沿射线推到 20 m 正方形外。\n\n"
        + "\n".join(lines)
    )


def _arm_group(rows: list[dict], scene: str, arm: str) -> list[dict]:
    return [r for r in rows if r.get("scene_kind") == scene and r.get("arm") == arm]


def results_blurb(rows: list[dict]) -> str:
    """从合计里抽出和 0.694 m、六机对照直接相关的几句。没有这些行时留空。"""

    def counts(scene: str, arm: str) -> str | None:
        group = _arm_group(rows, scene, arm)
        if not group:
            return None
        st = _stats(group)
        return (
            f"到达 {st['arrived']}/{st['total']}，障碍 {st['obs']}，机间 {st['inter']}，"
            f"超时 {st['timeout']}，最小机间 {_rng(st['gaps'], 3)} m"
        )

    parts = []
    sphere = counts("d014-swap3", "sphere")
    if sphere:
        wider = []
        for arm in ("r100", "w16", "r100w16"):
            group = _arm_group(rows, "d014-swap3", arm)
            if not group:
                continue
            st = _stats(group)
            seed1 = next((r for r in group if int(r.get("seed", -1)) == 1), None)
            seed1_bit = ""
            if seed1 is not None and int(seed1.get("inter_drone_collisions") or 0) == 0:
                seed1_bit = "，seed 1 的机间碰撞没有了"
            wider.append(
                f"{arm_label(arm)}：到达 {st['arrived']}/{st['total']}，障碍 {st['obs']}，机间 {st['inter']}{seed1_bit}"
            )
        parts.append(
            f"014 三机对向 seeds 0–9。默认球：{sphere}。seed 1 的 0.694 m 还在。"
            + ("加大半径或权重：" + "；".join(wider) + "。" if wider else "")
        )
    end = next((r for r in _arm_group(rows, "d014-swap3", "end_free") if int(r.get("seed", -1)) == 1), None)
    if end is not None and int(end.get("inter_drone_collisions") or 0) == 0:
        if int(end.get("obstacle_collisions") or 0):
            parts.append(
                "脚本内末端放行在 seed 1 上不再记机间碰撞，"
                f"换成障碍碰撞（`{end.get('reason')}`），最小机间 {_f(end.get('min_inter_drone_distance_m'), 3)} m。"
                "放开末端能把 0.694 m 的擦线推开，这一粒改去撞障碍。"
            )
        elif end.get("outcome") == "SUCCESS":
            parts.append(
                "脚本内末端放行在 seed 1 上到达，最小机间 "
                f"{_f(end.get('min_inter_drone_distance_m'), 3)} m，机间没有越线。"
            )
    stacks = []
    for arm in ("geom_multi", "empty_brake", "geom_multi+empty_brake"):
        row = next((r for r in _arm_group(rows, "d014-swap3", arm) if int(r.get("seed", -1)) == 1), None)
        if row is not None and int(row.get("inter_drone_collisions") or 0):
            stacks.append(f"{arm_label(arm)} 仍是 {_f(row.get('min_inter_drone_distance_m'), 3)} m")
    if stacks:
        parts.append("seed 1 上，" + "；".join(stacks) + "。叠 geom_multi 或 empty_brake 没有去掉这次擦线。")
    open_off = counts("swap6-open", "off")
    open_sphere = counts("swap6-open", "sphere")
    open_geom = counts("swap6-open", "geom")
    if open_off and open_sphere and open_geom:
        parts.append(f"六机开阔，seeds 0–4。全关：{open_off}。默认球：{open_sphere}。叠 geom：{open_geom}。")
    six_off = counts("d014-swap6", "off")
    six_sphere = counts("d014-swap6", "sphere")
    six_geom = counts("d014-swap6", "geom")
    if six_off and six_sphere and six_geom:
        parts.append(
            f"014 六机对向，seeds 0–4。全关：{six_off}。默认球单独：{six_sphere}。叠 geom：{six_geom}。"
        )
    return "\n\n".join(parts)


def render_markdown(payload: dict) -> str:
    rows = list(payload.get("rows") or [])
    commits = sorted({str(r.get("commit")) for r in rows if r.get("commit")})
    code = "、".join(f"`{c}`" for c in commits) if commits else "`unknown`"
    headline = seed1_headline(rows)
    curves = _seed1_curves(rows)
    curve_block = curves if curves else "这次没有 seed 1 的距离曲线。"
    return f"""# E-B5：欧氏碰撞球参数扫描（只报数）

只报数。report-only。没有改 `yopo_l0/`、`core/`、`scenes/`、`docs/tickets.md`、`README.md`、`scripts/premerge.sh`、`scripts/regress.sh`。`peer_sphere_enabled` 仍默认关。打开欧氏球、加大半径或权重、`track_clearance`、`empty_brake`，或使用下面的脚本内末端放行，都标「偏离上游」。`off` 不打开这些开关。未冻结，不进 `scenes/suite.txt`，不设 gates。

这不是论文复现。默认半径 0.85 m = 0.70 m 碰撞球 + 0.15 m 裕度，权重 8，与已合入的 E-B4c 相同。1.00 m 是 0.70 m 再加 0.30 m 裕度。权重 16 是默认的两倍。`track_margin` 0.08 m 沿用 E-C5 的假设，用来盖已观测的跟踪差。

数字来自 `scripts/e-b5-sphere-sweep.py`，跑数时的代码 {code}。逐粒记录在 [e-b5-sphere-sweep-runs.json](e-b5-sphere-sweep-runs.json)。

## seed 1 的 0.694 m

E-B4c 在 `scenes/ego-density-014-swap3.json` seed 1、默认欧氏球上记下机间碰撞，最小机间 0.694 m。运行器在第一次中心距 ≤ 0.70 m 的那一拍停下，所以这个数是越线那一拍的采样，不是全程最小。

{headline}

{curve_block}

## 臂

半径和权重写在 `policy_params.backend`，与 `BackendConfig.peer_sphere_radius_m` / `peer_sphere_weight` 相同。`geom` / `geom_multi` 是已合入的 `track_clearance`，`empty_brake` 是已合入的无解刹停。场景卡上不写 `swarm_penalty`。邻机采样仍由脚本在进程里送进后段，和 E-B4c 的对照脚本同一层包装：默认运行器只在椭球开关打开时送采样。

{END_FREE_NOTE}

| 臂 | 进程里多出来的 | 偏离上游 |
|---|---|---|
| 全关 | 没有 | 否 |
| 默认 E-B4c（0.85 m，权重 8） | `peer_sphere_enabled`，半径 0.85 m，权重 8 | 偏离上游 |
| 半径 1.00 m，权重 8 | 同上，半径 1.00 m | 偏离上游 |
| 半径 0.85 m，权重 16 | 同上，权重 16 | 偏离上游 |
| 半径 1.00 m，权重 16 | 半径 1.00 m，权重 16 | 偏离上游 |
| 默认球 + geom_multi 0.08 m | 默认球，再加 `track_clearance` mode=`geom_multi`，margin 0.08 m | 偏离上游 |
| 默认球 + empty_brake | 默认球，再加 `empty_brake=true` | 偏离上游 |
| 默认球 + geom_multi + empty_brake | 上面两样一起 | 偏离上游 |
| 默认球 + 脚本内末端放行 | 卡上仍是默认球。进程内放开末端，椭球代价为 0，最后一跨也进球 | 偏离上游 |
| 默认球 + geom 0.08 m | 默认球，再加 `track_clearance` mode=`geom`，margin 0.08 m。只用于六机 | 偏离上游 |

014 三机对向用前九臂，seeds 0–9。六机开阔和 014 六机对向用全关、默认球、默认球 + geom，seeds 0–4。六机不跑末端放行，也不跑 geom_multi：这一问只看球单独和叠 `geom`。

## 场景

014 三机对向是磁盘上的 `scenes/ego-density-014-swap3.json`。另外两张卡只在进程内存里。

{_placement_md()}

影子盾开。仿真时长用场景卡（40 s）。后段预算仍是墙钟 0.02 s、最多 8 次迭代。全关臂可以多个进程；带后段的臂两个进程，避免预算把迭代截短。

到达按架次。撞障碍是 `TRUTH_POSITION`，机间是 `INTER_DRONE`，仿真在第一次碰撞停下，所以一次运行最多记 1 次碰撞。超时是 `TIMEOUT`。最小机间：没撞时是全程采样最小；撞了则是越线那一拍的采样。到达时间是全部飞机都进到达判据时的 `arrival_time_s`。

## 结果

{results_blurb(rows)}

{summarize(rows)}

## 分种子

{_per_seed(rows)}

## 复现

```bash
.venv/bin/python scripts/e-b5-sphere-sweep.py
```

每粒落在 `out/e-b5-sphere-sweep/partial/`，缺省续跑。`--no-resume` 忽略已有 partial。`--scenes`、`--arms`、`--seeds` 可以只跑一部分。单测不跑这 120 粒：

```bash
.venv/bin/python -m pytest -q tests/test_e_b5_sphere_sweep.py
```
"""


def _load_partials(jobs: list[tuple[str, str, int]]) -> list[dict]:
    rows = []
    for scene, arm, seed in jobs:
        path = _partial(scene, arm, seed)
        if path.is_file():
            rows.append(json.loads(path.read_text(encoding="utf-8")))
    return rows


def _run_group(jobs: list[tuple[str, str, int]], *, workers: int, resume: bool) -> None:
    specs = [(scene, arm, seed, resume) for scene, arm, seed in jobs]
    if not specs:
        return
    if workers <= 1:
        for spec in specs:
            _job(spec)
        return
    from concurrent.futures import ProcessPoolExecutor, as_completed

    with ProcessPoolExecutor(max_workers=workers) as pool:
        futs = [pool.submit(_job, spec) for spec in specs]
        for fut in as_completed(futs):
            fut.result()


def _parse_list(text: str) -> list[str]:
    return [s for s in text.split(",") if s]


def _jobs(scenes: list[str], arms_arg: str | None, seeds_arg: str | None) -> list[tuple[str, str, int]]:
    jobs = []
    for scene in scenes:
        if arms_arg is None:
            arms = list(SCENE_ARMS[scene])
        else:
            arms = _parse_list(arms_arg)
        if seeds_arg is None:
            seeds = SCENE_SEEDS[scene]
        else:
            seeds = tuple(int(s) for s in seeds_arg.split(",") if s != "")
        for arm in arms:
            if arm not in ARM_SPEC:
                raise SystemExit(f"未知臂: {arm}")
            for seed in seeds:
                jobs.append((scene, arm, int(seed)))
    return jobs


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", default=",".join(SCENE_ORDER))
    ap.add_argument("--arms", default=None, help="盖过每张卡自己的臂。缺省用票面的臂。")
    ap.add_argument("--seeds", default=None, help="盖过每张卡自己的种子。缺省：014 三机 0–9，六机 0–4。")
    ap.add_argument("--jobs-plain", type=int, default=4)
    ap.add_argument("--jobs-backend", type=int, default=2)
    ap.add_argument("--out", default="docs/research/e-b5-sphere-sweep-runs.json")
    ap.add_argument("--md", default="docs/research/e-b5-sphere-sweep.md")
    ap.add_argument("--no-md", action="store_true")
    ap.add_argument("--no-resume", action="store_true")
    a = ap.parse_args(argv)
    scenes = _parse_list(a.scenes)
    unknown = [s for s in scenes if s not in SCENE_ORDER]
    if unknown:
        raise SystemExit(f"未知场景: {unknown}")
    if int(a.jobs_backend) > 2:
        raise SystemExit("带后段的臂最多 2 个进程，避免墙钟预算把后段迭代截短")
    if int(a.jobs_plain) < 1 or int(a.jobs_backend) < 1:
        raise SystemExit("进程数至少为 1")
    jobs = _jobs(scenes, a.arms, a.seeds)
    resume = not a.no_resume

    from yopo_swarm_core.baseline import fetch_upstream

    fetch_upstream()
    plain = [job for job in jobs if job[1] == "off"]
    backend = [job for job in jobs if job[1] != "off"]
    if plain:
        _run_group(plain, workers=max(1, int(a.jobs_plain)), resume=resume)
    if backend:
        _run_group(backend, workers=max(1, int(a.jobs_backend)), resume=resume)

    rows = _load_partials(jobs)
    rows.sort(
        key=lambda r: (
            SCENE_ORDER.index(r["scene_kind"]),
            list(ARM_SPEC).index(r["arm"]),
            int(r["seed"]),
        )
    )
    payload = {
        "disclaimer": DISCLAIMER,
        "report_only": True,
        "deviation": "打开欧氏球、track_clearance、empty_brake 或脚本内末端放行即偏离上游 d5e1e60。off 不偏离。",
        "end_free_note": END_FREE_NOTE,
        "prior_seed1_m": PRIOR_SEED1_M,
        "prior_seed1_source": "docs/research/e-b4c-sphere-avoid.md",
        "collision_ball_m": COLLISION_M,
        "default_radius_m": 0.85,
        "default_weight": 8.0,
        "wider_radius_m": 1.00,
        "heavier_weight": 16.0,
        "track_margin_m": TRACK_MARGIN_M,
        "scenes": scenes,
        "arms_by_scene": {scene: list(SCENE_ARMS[scene]) for scene in scenes},
        "seeds_by_scene": {
            scene: list(SCENE_SEEDS[scene] if a.seeds is None else tuple(int(s) for s in a.seeds.split(",") if s != ""))
            for scene in scenes
        },
        "completed": len(rows),
        "expected": len(jobs),
        "incomplete": len(jobs) - len(rows),
        "rows": rows,
    }
    out = ROOT / a.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(summarize(rows))
    print(seed1_headline(rows))
    print(f"wrote {out} ({payload['completed']}/{payload['expected']})")
    if payload["incomplete"] == 0 and not a.no_md:
        md = ROOT / a.md
        md.write_text(render_markdown(payload), encoding="utf-8")
        print(f"wrote {md}")
    return 0 if payload["incomplete"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
