"""E-A1：上游候选打分阶段的机间项。默认关。

只在走廊已经准入的候选上，用邻机已提交轨迹的采样做同时刻中心距。
距离小于场景卡阈值时：惩罚权重为 0 则从可行集去掉，权重大于 0 则从上游分数里减一笔。
可行集上的第一名仍是 argmax（同分取索引较小者），和 `select_action` 同一条规则。

可行集空时不新增几何回退。调用方走原来的无候选路径（goto 的 `hold_traj`，
穿林的 `CORRIDOR_EMPTY_HOVER` 以及其后已有的刹车或保持上一份计划）。
提交层、Delay Check、B 样条后段都不在这里。

E-A1 原文写椭球距离。论文没有给椭球系数，椭球惩罚归 E-B3。本票用场景卡上的
阈值做中心距，避免和后段抢同一套公式。

开关关闭时，本模块的配置函数返回 None。运行器不采样、不改分数、不往结果里加字段。

E-A2：硬过滤把可行集清空时的软让行，默认关。`inter_drone_soft_yield` 缺省或 false 时，
配置和 `inter_pick` 与上面这段逐位相同。打开、且惩罚权重为 0、所有准入候选都在阈值内时，
保留中心距不小于 `inter_drone_hard_min_m`（默认 0.75 m）的候选，含测不到的 +inf。
0.75 m 高于机间碰撞球 0.70 m（机体半径 0.35 m 的两倍），低于这一下限的仍然滤掉。
保底集里按 `score - yield_penalty * (阈值 − d) / 阈值` 重排，同分取索引较小者。
连下限都没有候选时，才回到原来的空集。不新做几何回退。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# 与穿林净空门同一个采样步长。4 s 盖住上游两段时长（segment_time 10/6 s × 2）。
SAMPLE_DT_S = 0.02
SAMPLE_HORIZON_S = 4.0
_N = int(round(SAMPLE_HORIZON_S / SAMPLE_DT_S))

FALLBACK_CODE = "CORRIDOR_EMPTY_HOVER"


@dataclass(frozen=True)
class InterDroneConfig:
    threshold_m: float
    penalty: float
    # E-A2 默认关。关时 inter_pick 不读后面两项，对象与只传前两项相同。
    soft_yield: bool = False
    hard_min_m: float = 0.75
    yield_penalty: float = 2.0


def inter_drone_config(params: dict | None) -> InterDroneConfig | None:
    """`policy_params` 上的开关。缺省或 false 时返回 None，不读阈值。"""
    raw = ((params or {}).get("upstream") or {})
    flag = raw.get("inter_drone_filter", False)
    if flag is False:
        return None
    if flag is not True:
        raise ValueError(f"inter_drone_filter 必须是布尔值，收到 {flag!r}")
    missing = [k for k in ("inter_drone_threshold_m", "inter_drone_penalty") if k not in raw]
    if missing:
        raise ValueError("打开 inter_drone_filter 时，场景卡必须写 " + " 和 ".join(missing))
    threshold = float(raw["inter_drone_threshold_m"])
    penalty = float(raw["inter_drone_penalty"])
    if not math.isfinite(threshold) or threshold <= 0.0:
        raise ValueError(f"inter_drone_threshold_m 必须为正，收到 {raw['inter_drone_threshold_m']!r}")
    if not math.isfinite(penalty) or penalty < 0.0:
        raise ValueError(f"inter_drone_penalty 必须为非负，收到 {raw['inter_drone_penalty']!r}")
    soft = raw.get("inter_drone_soft_yield", False)
    if soft is not True and soft is not False:
        raise ValueError(f"inter_drone_soft_yield 必须是布尔值，收到 {soft!r}")
    hard_min = 0.75
    yield_penalty = 2.0
    # 关掉时不读下限和让行权重，返回值与 E-A1 只传阈值、惩罚相同。
    if soft is True:
        if "inter_drone_hard_min_m" in raw:
            hard_min = _positive_finite("inter_drone_hard_min_m", raw["inter_drone_hard_min_m"])
        if "inter_drone_yield_penalty" in raw:
            yield_penalty = _nonnegative_finite("inter_drone_yield_penalty", raw["inter_drone_yield_penalty"])
    return InterDroneConfig(
        threshold_m=threshold,
        penalty=penalty,
        soft_yield=soft,
        hard_min_m=hard_min,
        yield_penalty=yield_penalty,
    )


def _positive_finite(name: str, value) -> float:
    """正实数。布尔值不能借 int 混进来（True 会变成 1.0）。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须为正实数，收到 {value!r}")
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} 必须为正实数，收到 {value!r}")
    return number


def _nonnegative_finite(name: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须为非负实数，收到 {value!r}")
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{name} 必须为非负实数，收到 {value!r}")
    return number


@dataclass(frozen=True)
class InterPick:
    """`unchanged`：没有候选进入阈值，调用方保持原来的 argmax。

    `adjusted`：可行集非空，`action` / `order` 是减分或过滤之后的结果。
    `empty`：过滤后没有可行候选。调用方走 `FALLBACK_CODE`，不要另做几何回退。
    """

    kind: str
    action: int | None = None
    order: tuple[int, ...] | None = None
    filtered: int = 0
    penalized: int = 0

    @property
    def empty(self) -> bool:
        return self.kind == "empty"


def inter_pick(
    score,
    admitted,
    clearances,
    cfg: InterDroneConfig,
) -> InterPick:
    """在已准入候选上应用机间项。`clearances[i]` 为 +inf 表示量不到，不罚也不滤。"""
    score = np.asarray(score, dtype=np.float64)
    admitted = np.asarray(admitted, dtype=bool)
    clearances = np.asarray(clearances, dtype=np.float64)
    if score.shape != admitted.shape or score.shape != clearances.shape:
        raise ValueError("score、admitted、clearances 长度必须相同")
    within = admitted & np.isfinite(clearances) & (clearances < cfg.threshold_m)
    n_within = int(within.sum())
    if n_within == 0:
        return InterPick("unchanged")
    adjusted = score.copy()
    feasible = admitted.copy()
    filtered = 0
    penalized = 0
    if cfg.penalty == 0.0:
        feasible[within] = False
        filtered = n_within
    else:
        scale = (cfg.threshold_m - clearances) / cfg.threshold_m
        adjusted[within] = score[within] - cfg.penalty * scale[within]
        penalized = n_within
    if not bool(feasible.any()):
        # 只有惩罚为 0 的硬过滤才会把准入候选全部拿掉。软让行只接这一条，避免和减分路径抢。
        if cfg.soft_yield and cfg.penalty == 0.0:
            return _soft_yield_pick(score, admitted, clearances, cfg, filtered)
        return InterPick("empty", filtered=filtered, penalized=penalized)
    # 与 select_action 相同：可行者里 argmax，同分取 flatnonzero 的第一个（索引较小）。
    cand = np.flatnonzero(feasible)
    first = int(cand[int(np.argmax(adjusted[cand]))])
    rest = sorted((int(i) for i in cand.tolist() if int(i) != first), key=lambda i: (-float(adjusted[i]), i))
    return InterPick("adjusted", action=first, order=(first, *rest), filtered=filtered, penalized=penalized)


def _soft_yield_pick(score, admitted, clearances, cfg: InterDroneConfig, filtered: int) -> InterPick:
    """硬过滤已经清空可行集。留下不低于 hard_min_m 的准入候选（含 +inf），按让行权重重排。

    低于下限的仍然算滤掉。一个都不剩时返回 empty，调用方走原来的悬停或刹停。
    """
    keep = admitted & (clearances >= float(cfg.hard_min_m))
    if not bool(keep.any()):
        return InterPick("empty", filtered=filtered, penalized=0)
    adjusted = np.array(score, dtype=np.float64, copy=True)
    finite = keep & np.isfinite(clearances)
    penalized = 0
    if bool(finite.any()):
        scale = (float(cfg.threshold_m) - clearances[finite]) / float(cfg.threshold_m)
        adjusted[finite] = score[finite] - float(cfg.yield_penalty) * scale
        penalized = int(np.count_nonzero(finite & (clearances < float(cfg.threshold_m))))
    still = int(np.count_nonzero(
        admitted & np.isfinite(clearances) & (clearances < float(cfg.hard_min_m))
    ))
    cand = np.flatnonzero(keep)
    first = int(cand[int(np.argmax(adjusted[cand]))])
    rest = sorted((int(i) for i in cand.tolist() if int(i) != first), key=lambda i: (-float(adjusted[i]), i))
    return InterPick("adjusted", action=first, order=(first, *rest), filtered=still, penalized=penalized)


def sample_motion(motion, t0: float, horizon: float = SAMPLE_HORIZON_S, dt: float = SAMPLE_DT_S) -> np.ndarray:
    """绝对时间上的 (t, x, y, z)。有效期外夹到端点，端点之后视为停住。"""
    t0 = float(t0)
    n = int(round(float(horizon) / float(dt)))
    times = t0 + np.arange(n, dtype=np.float64) * float(dt)
    pos = np.empty((n, 3), dtype=np.float64)
    lo = float(motion.t_start)
    hi = float(motion.t_end)
    for i, t in enumerate(times.tolist()):
        tt = lo if t < lo else hi if t > hi else t
        pos[i] = np.asarray(motion.evaluate(tt, 0), dtype=np.float64)
    return np.column_stack((times, pos))


def samples_from_pva(plan, now: float) -> list[list[float]] | None:
    """给总线载荷用的采样。没有 `pva` 的计划（悬停令牌等）不附带。"""
    pva = getattr(plan, "pva", None)
    if pva is None:
        return None
    now = float(now)
    rows: list[list[float]] = []
    for i in range(_N):
        t = now + i * SAMPLE_DT_S
        p, _v, _a = pva(t)
        pp = np.asarray(p, dtype=np.float64)
        rows.append([t, float(pp[0]), float(pp[1]), float(pp[2])])
    return rows


def as_sample_array(samples) -> np.ndarray | None:
    arr = np.asarray(samples, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 4 or arr.shape[0] == 0:
        return None
    if not bool(np.all(np.isfinite(arr))):
        return None
    return arr


def clearance_m(cand: np.ndarray, neighbors: list[np.ndarray]) -> float:
    """候选采样与邻机采样在同一时刻的最近中心距。没有时间重叠则为 +inf。不外推。"""
    if cand.size == 0 or not neighbors:
        return math.inf
    ct = cand[:, 0]
    cp = cand[:, 1:4]
    best = math.inf
    for nb in neighbors:
        if nb.size == 0:
            continue
        ts = nb[:, 0]
        lo = float(ts[0])
        hi = float(ts[-1])
        mask = (ct >= lo) & (ct <= hi)
        if not bool(np.any(mask)):
            continue
        t = ct[mask]
        p = cp[mask]
        x = np.interp(t, ts, nb[:, 1])
        y = np.interp(t, ts, nb[:, 2])
        z = np.interp(t, ts, nb[:, 3])
        delta = p - np.column_stack((x, y, z))
        dist = float(np.sqrt(np.einsum("ij,ij->i", delta, delta)).min())
        if dist < best:
            best = dist
    return best


def committed_neighbor_samples(node, t0: float) -> list[np.ndarray]:
    """邻机最后一份已提交运动段的采样。

    `peer.plans` 里更早的计划被截断后，刹车停点仍可能留到 inf。把那些停点也算进来，
    会把已经替换掉的位置继续当成占用（ADR 0015 的幽灵停点）。这里只读最后一份。
    不读预留，也不改提交。
    """
    out: list[np.ndarray] = []
    peers = getattr(node, "peers", None) or {}
    for _mid, peer in sorted(peers.items()):
        plans = getattr(peer, "plans", None) or []
        if not plans:
            continue
        motion = plans[-1].occupancy.plan.motion
        out.append(sample_motion(motion, t0))
    return out


def report_block(cfg: InterDroneConfig, filtered: int, penalized: int, empty: int) -> dict:
    return {
        "enabled": True,
        "threshold_m": float(cfg.threshold_m),
        "penalty": float(cfg.penalty),
        "filtered_candidates": int(filtered),
        "penalized_candidates": int(penalized),
        "empty_fallbacks": int(empty),
        # 可行集空时的动作就是原来的无候选路径，不是新的安全层。
        "fallback": FALLBACK_CODE,
    }
