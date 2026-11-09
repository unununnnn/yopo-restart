"""G1 候选解码（#6）：网络原始输出 → 世界系、绝对时间、头状态在 t_switch 重构的 MINCO 候选。

与上游 YOPO-MINCO（d5e1e60）对应关系：
- `decode_params`  ≙ `StateTransform.pred_to_traj_params`（图像顺序，14 维参数）
- `minco_two_piece` ≙ `MincoTraj.solve`（2 段五次、内点 C4 连续），结果转成 PiecewisePolynomial
- `corridor_mu_lo` / `select_action` ≙ `test_yopo_ros.py` 的 `_corridor_mu_lo` / `select_action`（原样移植，
  由 `tests/golden/minco_node_d5e1e60.npz` 锁定）
- `rank_candidates`：严格准入（ADR-0006 2026-10-26 修订，D3）。只有 μ_lo ≥ safe_mu 的候选进入排序，
  首位就是上游 `select_action` 的选择；准入集为空时整帧拒绝（CORRIDOR_EMPTY），由规划器走已认证刹车分支

与上游的有意差异（I15/I8）：上游从“当前/期望状态”直接接上；这里头状态一律取父计划
（已批准的制动分支或运动段）在 t_switch 的 PVA，候选从 t_switch 起生效，有绝对有效期。
网络给出的内点/终点是相对“状态采样时刻机体位姿 (R_wb, p_ref)”的偏移，世界系锚定后不随
t_switch 平移——它们描述的是环境里的点。预测走廊决定准入，但不是安全证明：候选
`env_checked` 恒为 False，必须再走 #7 环境检查与 #8 计划契约（保真登记 (c)）。
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from enum import Enum
from pathlib import Path
from typing import Any

import numpy as np

from .plan import CONTINUITY_TOL
from .trajectory import PiecewisePolynomial, PolySegment

DURATION_MIN = 0.1  # 上游 StateTransform.duration_min
SAFE_RADIUS = 0.05  # 上游 test_yopo_ros.py safe_radius（m）
DEFAULT_MAX_SKEW = 0.03  # 状态采样与图像曝光允许的最大时间差（s）

DECODER_CONFIG_KEYS = (
    "train",
    "velocity",
    "vel_max_train",
    "acc_max_train",
    "tail_radio_max",
    "horizon_num",
    "vertical_num",
    "horizon_camera_fov",
    "vertical_camera_fov",
    "horizon_anchor_fov",
    "vertical_anchor_fov",
    "radius_num",
    "radius_warp_lambda",
)


class CandidateReject(Enum):
    MALFORMED = "MALFORMED"  # 非有限值、形状不对、旋转矩阵非法
    STATE_IMAGE_SKEW = "STATE_IMAGE_SKEW"  # 状态采样与图像曝光时间差超界
    TIME_ORDER = "TIME_ORDER"  # 六时刻先后关系不成立
    PARENT_EXPIRED = "PARENT_EXPIRED"  # 父计划不覆盖 t_switch，无法重构头状态
    CORRIDOR_EMPTY = "CORRIDOR_EMPTY"  # 没有候选的预测走廊达到 safe_mu（上游 brake=True）


@dataclass(frozen=True)
class DecoderParams:
    vel_max: float
    acc_max: float
    segment_time: float
    tail_radio_max: float
    horizon_num: int
    vertical_num: int
    horizon_fov_deg: float
    vertical_fov_deg: float
    horizon_anchor_fov_deg: float
    vertical_anchor_fov_deg: float
    radius_num: int
    radius_warp_lambda: float
    safe_radius: float = SAFE_RADIUS

    @classmethod
    def from_config(cls, cfg: Mapping[str, Any]) -> DecoderParams:
        """按上游 LatticeParam 推导（ratio = 1 训练时，否则 velocity / vel_max_train）。"""
        ratio = 1.0 if cfg["train"] else float(cfg["velocity"]) / float(cfg["vel_max_train"])
        sgm_time = float(cfg["tail_radio_max"]) / float(cfg["vel_max_train"])
        return cls(
            vel_max=ratio * float(cfg["vel_max_train"]),
            acc_max=ratio * ratio * float(cfg["acc_max_train"]),
            segment_time=sgm_time / ratio,
            tail_radio_max=float(cfg["tail_radio_max"]),
            horizon_num=int(cfg["horizon_num"]),
            vertical_num=int(cfg["vertical_num"]),
            horizon_fov_deg=float(cfg["horizon_camera_fov"]),
            vertical_fov_deg=float(cfg["vertical_camera_fov"]),
            horizon_anchor_fov_deg=float(cfg["horizon_anchor_fov"]),
            vertical_anchor_fov_deg=float(cfg["vertical_anchor_fov"]),
            radius_num=int(cfg["radius_num"]),
            radius_warp_lambda=float(cfg["radius_warp_lambda"]),
        )

    @property
    def count(self) -> int:
        return self.horizon_num * self.vertical_num

    @property
    def safe_mu(self) -> float:
        return 1.0 - math.exp(-self.safe_radius / self.radius_warp_lambda)

    def grid_angles(self) -> tuple[np.ndarray, np.ndarray]:
        """图像顺序的锚点 (yaw, pitch)。上游按 lattice 顺序（自下而上、自右向左）生成后整体翻转。"""
        h, v = self.horizon_num, self.vertical_num
        dyaw = 0.0 if h == 1 else math.radians(self.horizon_fov_deg) / h
        dpitch = 0.0 if v == 1 else math.radians(self.vertical_fov_deg) / v
        yaw, pitch = [], []
        for i in range(v):
            for j in range(h):
                yaw.append(-dyaw * (h - 1) / 2 + j * dyaw)
                pitch.append(-dpitch * (v - 1) / 2 + i * dpitch)
        return np.array(yaw)[::-1], np.array(pitch)[::-1]


# 上游 d5e1e60 config/traj_opt.yaml + config.py 推导值；测试会与金样里的配置及在线上游核对。
UPSTREAM_DECODER = DecoderParams(
    vel_max=6.0,
    acc_max=6.0,
    segment_time=10.0 / 6.0,
    tail_radio_max=10.0,
    horizon_num=5,
    vertical_num=3,
    horizon_fov_deg=90.0,
    vertical_fov_deg=60.0,
    horizon_anchor_fov_deg=30.0,
    vertical_anchor_fov_deg=30.0,
    radius_num=10,
    radius_warp_lambda=2.0,
)


def _spherical(yaw: np.ndarray, pitch: np.ndarray, r: np.ndarray) -> np.ndarray:
    cp = np.cos(pitch)
    return np.stack([cp * np.cos(yaw) * r, cp * np.sin(yaw) * r, np.sin(pitch) * r], axis=-1)


def decode_params(
    traj_params: np.ndarray, dec: DecoderParams
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(N,14) 图像顺序原始参数 → inner_pos_b (N,3)、tail_pva_b (N,3,3)、durations (N,2)。"""
    p = np.asarray(traj_params, dtype=float)
    if p.shape != (dec.count, 14):
        raise ValueError(f"traj_params shape {p.shape} != ({dec.count}, 14)")
    yaw_g, pitch_g = dec.grid_angles()
    yaw_diff = 0.5 * math.radians(dec.horizon_anchor_fov_deg)
    pitch_diff = 0.5 * math.radians(dec.vertical_anchor_fov_deg)
    inner = _spherical(
        yaw_g + p[:, 0] * yaw_diff,
        pitch_g + p[:, 1] * pitch_diff,
        (p[:, 2] + 1.0) * 0.5 * dec.tail_radio_max,
    )
    tail_pos = _spherical(
        p[:, 3] * math.radians(dec.horizon_fov_deg) / 2.0,
        p[:, 4] * math.radians(dec.vertical_fov_deg) / 2.0,
        (p[:, 5] + 1.0) * 0.5 * dec.tail_radio_max,
    )
    tail = np.stack([tail_pos, p[:, 6:9] * dec.vel_max, p[:, 9:12] * dec.acc_max], axis=-2)
    durations = np.maximum((p[:, 12:14] + 1.0) * (0.5 * dec.segment_time), DURATION_MIN)
    return inner, tail, durations


def minco_two_piece(
    t0: float,
    head_pva: np.ndarray,
    tail_pva: np.ndarray,
    inner_pos: np.ndarray,
    durations: Sequence[float],
) -> PiecewisePolynomial:
    """2 段五次（MincoS3NU）：头/尾 PVA、内点位置、内点处 C4 连续。与上游 12×12 线性系统相同。"""
    T0, T1 = (float(x) for x in durations)
    a = np.zeros((12, 12))
    a[0, 0], a[1, 1], a[2, 2] = 1.0, 1.0, 2.0
    pw = [T0**k for k in range(6)]
    a[3, 0:6] = pw
    a[4, 0:6] = pw
    a[4, 6] = -1.0
    a[5, 1:6] = [1, 2 * pw[1], 3 * pw[2], 4 * pw[3], 5 * pw[4]]
    a[5, 7] = -1.0
    a[6, 2:6] = [2, 6 * pw[1], 12 * pw[2], 20 * pw[3]]
    a[6, 8] = -2.0
    a[7, 3:6] = [6, 24 * pw[1], 60 * pw[2]]
    a[7, 9] = -6.0
    a[8, 4:6] = [24, 120 * pw[1]]
    a[8, 10] = -24.0
    pw = [T1**k for k in range(6)]
    a[9, 6:12] = pw
    a[10, 7:12] = [1, 2 * pw[1], 3 * pw[2], 4 * pw[3], 5 * pw[4]]
    a[11, 8:12] = [2, 6 * pw[1], 12 * pw[2], 20 * pw[3]]
    b = np.zeros((12, 3))
    b[0:3] = np.asarray(head_pva, dtype=float)
    b[3] = np.asarray(inner_pos, dtype=float)
    b[9:12] = np.asarray(tail_pva, dtype=float)
    c = np.linalg.solve(a, b)
    return PiecewisePolynomial(t0, [PolySegment(c[0:6].T, T0), PolySegment(c[6:12].T, T1)])


# --- 时间 -----------------------------------------------------------------------


@dataclass(frozen=True)
class Timing:
    """六时刻（ADR-0003 共享时间基准下的绝对时间，秒）。actual_switch 在执行后另行记录。"""

    image_capture: float
    state_sample: float
    inference_done: float
    proposal: float
    effective: float  # = t_switch
    actual_switch: float | None = None

    def with_actual_switch(self, t: float) -> Timing:
        return replace(self, actual_switch=float(t))

    def validate(self, max_skew: float = DEFAULT_MAX_SKEW) -> CandidateReject | None:
        ts = [
            self.image_capture,
            self.state_sample,
            self.inference_done,
            self.proposal,
            self.effective,
        ]
        if self.actual_switch is not None:
            ts.append(self.actual_switch)
        if not all(math.isfinite(float(t)) for t in ts):
            return CandidateReject.MALFORMED
        if abs(self.state_sample - self.image_capture) > max_skew:
            return CandidateReject.STATE_IMAGE_SKEW
        if not (
            max(self.image_capture, self.state_sample)
            <= self.inference_done
            <= self.proposal
            < self.effective
        ):
            return CandidateReject.TIME_ORDER
        return None


# --- 排序 -----------------------------------------------------------------------


@dataclass(frozen=True)
class Ranking:
    order: tuple[
        int, ...
    ]  # 只含准入候选：首位 = 上游 select_action 的选择，其余按分数降序（同分按索引）
    mu_lo: np.ndarray = field(repr=False)
    admitted: np.ndarray = field(repr=False)
    corridor_empty: bool

    @property
    def preferred(self) -> int | None:
        return self.order[0] if self.order else None

    @property
    def warning(self) -> str | None:
        return "CORRIDOR_EMPTY" if self.corridor_empty else None


def corridor_mu_lo(radius: np.ndarray, dec: DecoderParams) -> np.ndarray:
    """每个候选最紧的保守走廊：min over nr (μ − b)。radius: (N, 2·nr)，前 nr 为 μ，后 nr 为 b。

    保留输入浮点精度（网络输出为 float32 时按 float32 相减），与上游 `_corridor_mu_lo` 逐位一致。
    """
    r = np.asarray(radius)
    if not np.issubdtype(r.dtype, np.floating):
        r = r.astype(float)
    nr = dec.radius_num
    return (r[:, :nr] - r[:, nr:]).min(axis=1)


def select_action(
    score: np.ndarray,
    inner_w: np.ndarray | None,
    mu_lo: np.ndarray,
    safe_mu: float,
    topk: int = 1,
    last_inner_w: np.ndarray | None = None,
) -> tuple[int, bool]:
    """上游 `select_action`（test_yopo_ros.py:338-351）原样移植，返回 (action_id, brake)。

    准入（μ_lo ≥ safe_mu）者里取最高分；topk > 1 且有上一条选中轨迹时，在准入者的前 K 名里取
    内点离上次内点最近的。无人准入时 brake=True，action_id 是全体最高分（上游仍会飞它，
    本仓库不飞：见 `rank_candidates` 与保真登记 (b)）。
    """
    score = np.asarray(score)
    # 上游 safe_mu 是 np.float64（1 − np.exp(...)），NumPy ≥ 2 下 float32 μ_lo 与之按 float64 比较；
    # 这里显式升到 float64，避免 Python float 弱类型把比较降到 float32（恰在 safe_mu 处会多准入 1 ulp）。
    mu_lo = np.asarray(mu_lo, dtype=np.float64)
    cand = np.flatnonzero(mu_lo >= np.float64(safe_mu))
    if cand.size == 0:
        return int(np.argmax(score)), True
    if topk <= 1 or last_inner_w is None:
        return int(cand[np.argmax(score[cand])]), False
    k = min(topk, cand.size)
    topk_ids = cand[np.argpartition(score[cand], -k)[-k:]]
    dists = np.linalg.norm(np.asarray(inner_w)[topk_ids] - last_inner_w, axis=1)
    return int(topk_ids[np.argmin(dists)]), False


def rank_candidates(
    score: np.ndarray,
    radius: np.ndarray,
    dec: DecoderParams,
    inner_w: np.ndarray | None = None,
    topk: int = 1,
    last_inner_w: np.ndarray | None = None,
) -> Ranking:
    """严格准入（D3）：只排准入候选；首位 = `select_action`，其余准入者按分数降序作为安全层的后备。

    未准入的候选不出现在 `order` 里，因此不可能被解码、检查或执行；准入集为空时 `order` 为空、
    `corridor_empty` 为真，调用方必须整帧拒绝并走刹车分支。
    """
    s = np.asarray(score, dtype=float)
    mu_lo = corridor_mu_lo(radius, dec)
    admitted = np.asarray(mu_lo, dtype=np.float64) >= np.float64(dec.safe_mu)  # 同 select_action
    if not admitted.any():
        return Ranking((), mu_lo, admitted, corridor_empty=True)
    first, _ = select_action(s, inner_w, mu_lo, dec.safe_mu, topk, last_inner_w)
    rest = sorted(
        (i for i in range(len(s)) if admitted[i] and i != first), key=lambda i: (-s[i], i)
    )
    return Ranking((first, *rest), mu_lo, admitted, corridor_empty=False)


# --- 候选 -----------------------------------------------------------------------


@dataclass(frozen=True)
class Candidate:
    index: int  # 图像顺序索引
    score: float
    traj: PiecewisePolynomial  # 世界系，[t_switch, t_switch + T0 + T1] 绝对有效期
    durations: np.ndarray
    inner_w: np.ndarray
    tail_pva_w: np.ndarray
    corridor_mu_lo: float
    corridor_ok: bool  # 严格准入后恒为 True；保留字段供日志与断言
    timing: Timing

    @property
    def env_checked(self) -> bool:
        """候选本身从不代表通过环境检查；预测走廊只决定准入，不是环境证明。"""
        return False


def _pva(tr: PiecewisePolynomial, t: float) -> np.ndarray:
    return np.array([tr.evaluate(t, k) for k in range(3)])


def head_continuity_ok(
    traj: PiecewisePolynomial,
    parent: PiecewisePolynomial,
    t_switch: float,
    tol: float = CONTINUITY_TOL,
) -> bool:
    """I8：候选从 t_switch 起生效，且其头 PVA 与父计划在 t_switch 的 PVA 一致。"""
    if abs(traj.t_start - t_switch) > tol or not parent.covers(t_switch, t_switch):
        return False
    return float(np.max(np.abs(_pva(traj, traj.t_start) - _pva(parent, t_switch)))) <= tol


def _is_rotation(r: np.ndarray) -> bool:
    return (
        r.shape == (3, 3)
        and bool(np.all(np.isfinite(r)))
        and np.allclose(r @ r.T, np.eye(3), atol=1e-6)
        and abs(np.linalg.det(r) - 1.0) < 1e-6
    )


def build_candidates(
    traj_params: np.ndarray,
    score: np.ndarray,
    radius: np.ndarray,
    dec: DecoderParams,
    parent: PiecewisePolynomial,
    timing: Timing,
    R_wb: np.ndarray,
    p_ref: np.ndarray,
    max_skew: float = DEFAULT_MAX_SKEW,
) -> list[Candidate] | CandidateReject:
    """解码一帧候选；返回按 `rank_candidates` 顺序排列的准入候选，或一个拒绝原因。

    没有候选通过走廊准入时返回 CandidateReject.CORRIDOR_EMPTY（上游 brake=True）。

    parent：已批准计划在 t_switch 处生效的轨迹（运动段或制动分支），必填——没有“只平移时间戳”的路径。
    R_wb, p_ref：状态采样时刻的机体姿态与位置（世界系），网络偏移据此锚定到世界系。
    """
    if not isinstance(parent, PiecewisePolynomial):
        raise TypeError("parent trajectory is required to reconstruct the head state at t_switch")
    params = np.asarray(traj_params, dtype=float)
    s = np.asarray(score, dtype=float)
    rad = np.asarray(radius, dtype=float)
    rot = np.asarray(R_wb, dtype=float)
    p0 = np.asarray(p_ref, dtype=float)
    if (
        params.shape != (dec.count, 14)
        or s.shape != (dec.count,)
        or rad.shape != (dec.count, 2 * dec.radius_num)
        or p0.shape != (3,)
        or not all(np.all(np.isfinite(x)) for x in (params, s, rad, p0))
        or not _is_rotation(rot)
    ):
        return CandidateReject.MALFORMED
    bad = timing.validate(max_skew)
    if bad is not None:
        return bad
    t_switch = float(timing.effective)
    if not parent.covers(t_switch, t_switch):
        return CandidateReject.PARENT_EXPIRED
    head = _pva(parent, t_switch)

    inner_b, tail_b, durations = decode_params(params, dec)
    inner_w = inner_b @ rot.T + p0
    tail_w = np.einsum("ij,nkj->nki", rot, tail_b)
    tail_w[:, 0] += p0
    ranking = rank_candidates(s, rad, dec)
    if ranking.corridor_empty:
        return CandidateReject.CORRIDOR_EMPTY
    out = []
    for i in ranking.order:
        tr = minco_two_piece(t_switch, head, tail_w[i], inner_w[i], durations[i])
        out.append(
            Candidate(
                index=i,
                score=float(s[i]),
                traj=tr,
                durations=durations[i].copy(),
                inner_w=inner_w[i].copy(),
                tail_pva_w=tail_w[i].copy(),
                corridor_mu_lo=float(ranking.mu_lo[i]),
                corridor_ok=bool(ranking.admitted[i]),
                timing=timing,
            )
        )
    return out


# --- 上游参考（需要上游源码与 torch；只给金样生成与 upstream 测试用） ------------------

DECODE_GOLDEN_PATH = (
    Path(__file__).resolve().parents[2] / "tests" / "golden" / "minco_decode_d5e1e60.npz"
)
_REF_YAW = math.radians(30.0)
_REF_ROT = np.array(
    [
        [math.cos(_REF_YAW), -math.sin(_REF_YAW), 0.0],
        [math.sin(_REF_YAW), math.cos(_REF_YAW), 0.0],
        [0, 0, 1.0],
    ]
)
_REF_POS = np.array([1.0, 2.0, 3.0])
_REF_HEAD = np.array([[1.0, 2.0, 3.0], [1.0, 0.2, -0.1], [0.1, 0.0, 0.0]])
_REF_SAMPLES = 11


def upstream_reference(model: Any) -> dict[str, np.ndarray]:
    """用上游 StateTransform 与 MincoTraj 对 #3 金样参数解码、求解、采样（偏移按 test_yopo_ros 锚定）。

    选择逻辑在上游 ROS 节点里（依赖 rospy），这里按 test_yopo_ros.py 的
    `_corridor_mu_lo`/`select_action`（topk=1）逐行转写。
    """
    import torch
    from policy.poly_solver import MincoTraj  # type: ignore[import-not-found]

    from .baseline import GOLDEN_PATH

    g = np.load(GOLDEN_PATH)
    dec = DecoderParams.from_config(model.config)
    v, h = dec.vertical_num, dec.horizon_num
    params = g["traj_params"]  # (1, N, 14) 图像顺序
    endstate = torch.from_numpy(params).reshape(1, v, h, 14).permute(0, 3, 1, 2).contiguous()
    with torch.no_grad():
        inner_b, tail_b, dur, _ = model._st.pred_to_traj_params(endstate)
    inner_b, tail_b, dur = (x[0].double().numpy() for x in (inner_b, tail_b, dur))

    inner_w = inner_b @ _REF_ROT.T + _REF_POS
    tail_w = np.einsum("ij,...kj->...ki", _REF_ROT, tail_b)
    tail_w[:, 0] += _REF_POS
    n = inner_w.shape[0]
    times = np.zeros((n, _REF_SAMPLES))
    pos, vel, acc = (np.zeros((n, _REF_SAMPLES, 3)) for _ in range(3))
    for i in range(n):
        traj = MincoTraj().solve(_REF_HEAD, tail_w[i], inner_w[i], durations=dur[i])
        times[i] = np.linspace(0.0, traj.total_time - 1e-6, _REF_SAMPLES)
        pos[i], vel[i], acc[i] = (
            traj.position(times[i]),
            traj.velocity(times[i]),
            traj.acceleration(times[i]),
        )

    m = g["radius"][0].T.reshape(2 * dec.radius_num, -1)  # 上游 (2nr, N) 布局
    mu_lo = (m[: dec.radius_num] - m[dec.radius_num :]).min(axis=0)
    safe_mu = 1.0 - np.exp(-SAFE_RADIUS / float(model.config["radius_warp_lambda"]))
    score = g["score"][0]
    cand = np.flatnonzero(mu_lo >= safe_mu)
    if cand.size == 0:
        action_id, brake = int(np.argmax(score)), True
    else:
        action_id, brake = int(cand[np.argmax(score[cand])]), False

    return {
        "decoder_config": np.array(
            json.dumps({k: model.config[k] for k in DECODER_CONFIG_KEYS}, sort_keys=True)
        ),
        "inner_pos_b": inner_b,
        "tail_pva_b": tail_b,
        "durations": dur,
        "head_pva_w": _REF_HEAD,
        "inner_w": inner_w,
        "tail_pva_w": tail_w,
        "times": times,
        "pos": pos,
        "vel": vel,
        "acc": acc,
        "upstream_action_id": np.array(action_id),
        "upstream_brake": np.array(brake),
    }


def main(argv: list[str] | None = None) -> int:
    import argparse

    from .baseline import fetch_upstream, load_verified_model

    ap = argparse.ArgumentParser(
        description="#6 解码金样（上游 StateTransform + MincoTraj 参考输出）"
    )
    ap.add_argument("--write-golden", action="store_true")
    args = ap.parse_args(argv)
    ref = upstream_reference(load_verified_model(fetch_upstream()))
    if args.write_golden:
        np.savez(DECODE_GOLDEN_PATH, **ref)
        print(f"wrote {DECODE_GOLDEN_PATH}")
    else:
        print(json.dumps({k: v.tolist() for k, v in ref.items() if v.ndim == 0}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
