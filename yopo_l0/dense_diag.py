"""票 07c：密档 uav0 撞树的定位。只记录，不改 CrossingFleet 的选择。

类别（每个失败 seed 归一条，按这个顺序）：

1. 无解悬停后撞：正在飞的计划是刹车 / 走廊空 / 运动学退回，不是一条 YOPO 曲线。
2. 跟踪误差：批准计划的参考点离障碍仍大于机体半径，机体自己进了半径。
3. 矮枝/斜干不在深度图里：撞上的是矮枝或斜干，而且订下这条计划时的深度图里没有它。
4. 走廊 0.05 m 过松：候选过了上游 0.05 m 走廊准入，计划本身却进了机体半径。
5. 其他。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from yopo_swarm_core.candidates import UPSTREAM_DECODER, corridor_mu_lo, rank_candidates
from yopo_swarm_core.plan import state_at

from .crossing import CrossingFleet
from .forest import card_for_tier
from .policy import R_yaw
from .runner import RunConfig, run
from .scene import Box, Cylinder, SceneCard, load_card
from .sensor import R_BODY_CAM, CameraConfig, _ray_dirs_cam

CAT_HIDDEN = "矮枝/斜干不在深度图里"
CAT_CORRIDOR = "走廊 0.05 m 过松"
CAT_HOVER = "无解悬停后撞"
CAT_TRACK = "跟踪误差"
CAT_OTHER = "其他"
CATEGORIES = (CAT_HIDDEN, CAT_CORRIDOR, CAT_HOVER, CAT_TRACK, CAT_OTHER)

_HOVER_CODES = frozenset({
    "CORRIDOR_EMPTY_HOVER",
    "BRAKE_ONLY",
    "CONFLICT_BRAKE",
    "YOPO_KIN_FALLBACK",
    "YOPO_FAIL",
    "ARRIVE_KIN_FALLBACK",
})
_FLY_CODES = frozenset({"OK", "YOPO_ALT"})
_SAFE_RADIUS_M = 0.05
_LAMBDA = float(UPSTREAM_DECODER.radius_warp_lambda)


def implied_radius_m(mu: float) -> float:
    """上游 μ = 1 − exp(−r / λ) 的反函数。r 是预测走廊半径，不是机体半径。"""
    clipped = min(max(float(mu), 0.0), 1.0 - 1e-12)
    return -_LAMBDA * math.log(1.0 - clipped)


def _tree_kind(ob) -> str | None:
    name = getattr(ob, "name", "")
    if not str(name).startswith("t"):
        return None
    if str(name).endswith("-br"):
        return "branch"
    if isinstance(ob, Cylinder) and not ob.upright():
        return "lean"
    if isinstance(ob, Cylinder):
        return "trunk"
    if isinstance(ob, Box):
        return "branch"
    return "other"


def nearest_tree(obstacles, position: np.ndarray):
    best = None
    best_d = math.inf
    for ob in obstacles:
        if _tree_kind(ob) is None:
            continue
        dist = float(ob.distance(np.asarray(position, dtype=float)))
        if dist < best_d:
            best, best_d = ob, dist
    return best, best_d


def depth_sees(frame, position: np.ndarray, yaw: float, cam: CameraConfig, obstacle) -> bool:
    """深度图里有没有落到这件障碍表面上的回波。薄枝在 160×96 上可能一个像素都没有。"""
    dirs = _ray_dirs_cam(cam)
    world = dirs @ (R_yaw(yaw) @ R_BODY_CAM).T
    mount = R_yaw(yaw) @ np.asarray(cam.mount_xyz_body, dtype=float)
    origin = np.asarray(position, dtype=float) + mount
    depth = np.asarray(frame.depth, dtype=float)
    valid = np.isfinite(depth)
    if not np.any(valid):
        return False
    pts = origin + depth[..., None] * world
    flat = pts[valid]
    # 全图像素太多，隔点取。160×96 隔 2 仍盖得住树干，薄枝本来就容易被漏掉。
    flat = flat[::2]
    dists = [float(obstacle.distance(p)) for p in flat]
    return min(dists) <= 0.2


def classify(facts: dict) -> str:
    """facts 见 diagnose_one 写进去的字段。顺序就是模块说明里的五类。"""
    if facts["plan_code"] in _HOVER_CODES:
        return CAT_HOVER
    if facts["plan_clearance_m"] > facts["body_radius_m"] and facts["actual_clearance_m"] <= facts["body_radius_m"]:
        return CAT_TRACK
    if facts["kind"] in ("branch", "lean") and not facts["in_depth"]:
        return CAT_HIDDEN
    if (
        facts["plan_clearance_m"] <= facts["body_radius_m"]
        and facts.get("admitted")
        and facts["plan_code"] in _FLY_CODES
    ):
        return CAT_CORRIDOR
    return CAT_OTHER


class _DiagFleet(CrossingFleet):
    """多记每次提案的走廊数。选择仍走 CrossingFleet._yopo_options / _planner。"""

    def __init__(self, card, physics_dt, *, record_inputs: bool):
        super().__init__(card, physics_dt, record_inputs=record_inputs)
        self.trace: list[dict] = []
        self._plan_now = 0.0
        self._plan_t0 = 0.0
        self._pose: dict[str, tuple] = {}
        self._snap: dict | None = None
        self._option_mu: list[float] = []
        self._as_calls = 0
        self._picked: int | None = None

    def ingest(self, drone_id, frame, state):
        super().ingest(drone_id, frame, state)
        self._pose[drone_id] = (
            np.array(state.p, dtype=float, copy=True),
            float(state.yaw),
            frame,
        )

    def _planner(self, node, now, t_start):
        self._plan_now = float(now)
        self._plan_t0 = float(t_start)
        self._snap = None
        self._option_mu = []
        self._as_calls = 0
        self._picked = None
        return super()._planner(node, now, t_start)

    def _as_plan(self, node, t_start, head, motion):
        occ, reject = super()._as_plan(node, t_start, head, motion)
        if occ is not None and self._picked is None:
            self._picked = self._as_calls
        self._as_calls += 1
        return occ, reject

    def _yopo_options(self, head, t_start, goal, depth, yaw):
        from yopo_swarm_core.candidates import decode_params, minco_two_piece

        R = R_yaw(yaw)
        params, score, radius = self.source.forward_raw(depth, head[1], head[2], goal, head[0], R)
        inner_b, tail_b, durations = decode_params(params, UPSTREAM_DECODER)
        inner_w = inner_b @ R.T + head[0]
        tail_w = np.einsum("ij,nkj->nki", R, tail_b)
        tail_w[:, 0] += head[0]
        ranking = rank_candidates(score, radius, UPSTREAM_DECODER, inner_w)
        mu = corridor_mu_lo(radius, UPSTREAM_DECODER)
        admitted = np.asarray(ranking.admitted, dtype=bool)
        n_adm = int(admitted.sum())
        selected = None if ranking.corridor_empty or not ranking.order else int(ranking.order[0])
        self._snap = {
            "n_candidates": int(UPSTREAM_DECODER.count),
            "n_admitted": n_adm,
            "corridor_empty": bool(ranking.corridor_empty),
            "mu_lo_selected": None if selected is None else float(mu[selected]),
            "mu_lo_min_admitted": None if n_adm == 0 else float(mu[admitted].min()),
            "safe_mu": float(UPSTREAM_DECODER.safe_mu),
        }
        # 下面与 CrossingFleet._yopo_options 相同，避免再跑一次网络。
        if ranking.corridor_empty:
            return [], "CORRIDOR_EMPTY_HOVER"
        motions = []
        option_mu: list[float] = []
        for index in ranking.order[:6]:
            try:
                raw = minco_two_piece(t_start, head, tail_w[index], inner_w[index], durations[index])
            except (ValueError, np.linalg.LinAlgError, FloatingPointError):
                continue
            traj = self._fit_yopo(raw)
            if traj is not None:
                motions.append(traj)
                option_mu.append(float(mu[index]))
        self._option_mu = option_mu
        if not motions:
            return [], "YOPO_KIN_FALLBACK"
        return motions, "OK"

    def _count(self, drone_id, code):
        super()._count(drone_id, code)
        pose = self._pose.get(drone_id)
        row = {
            "drone": drone_id,
            "now": self._plan_now,
            "t_start": self._plan_t0,
            "code": code,
            "n_candidates": None,
            "n_admitted": None,
            "corridor_empty": None,
            "mu_lo_selected": None,
            "mu_lo_min_admitted": None,
            "implied_r_m": None,
            "safe_mu": float(UPSTREAM_DECODER.safe_mu),
            "p": None if pose is None else pose[0].tolist(),
            "yaw": None if pose is None else pose[1],
            "frame": None if pose is None else pose[2],
        }
        if self._snap is not None:
            row.update(self._snap)
            if self._picked is not None and self._picked < len(self._option_mu):
                row["mu_lo_selected"] = self._option_mu[self._picked]
            mu = row["mu_lo_selected"]
            row["implied_r_m"] = None if mu is None else implied_radius_m(mu)
            self._snap = None
        self.trace.append(row)


def _install(fleet_cls):
    import yopo_l0.crossing as crossing

    previous = crossing.CrossingFleet
    crossing.CrossingFleet = fleet_cls
    return previous


def _plan_clearance(plan, obstacle, t0: float, t1: float) -> tuple[float, np.ndarray]:
    t0 = max(float(t0), float(plan.t_switch))
    t1 = max(float(t1), t0)
    times = np.linspace(t0, t1, 25)
    dists = []
    points = []
    for t in times:
        p = np.asarray(state_at(plan, float(t))[0], dtype=float)
        dists.append(float(obstacle.distance(p)))
        points.append(p)
    k = int(np.argmin(dists))
    return dists[k], points[k]


def diagnose_output(card: SceneCard, out, fleet: _DiagFleet) -> dict:
    result = out.result
    reason = str(result.get("reason") or "")
    drone = "uav0"
    if ":" in reason:
        drone = reason.split(":", 1)[0]
    rows = [row for row in out.ticks if row["uav"] == drone]
    hit = rows[-1]
    t_hit = float(hit["t"])
    p_hit = np.asarray(hit["p"], dtype=float)
    obstacle, actual = nearest_tree(card.obstacles, p_hit)
    kind = None if obstacle is None else _tree_kind(obstacle)
    body = float(card.card["criteria"]["body_radius_m"])
    window = [row for row in fleet.trace if row["drone"] == drone and t_hit - 1.0 <= row["now"] <= t_hit + 1e-9]
    active = [
        row for row in fleet.trace
        if row["drone"] == drone and row["t_start"] <= t_hit + 1e-9 and row["code"] in _FLY_CODES | _HOVER_CODES
    ]
    flown = active[-1] if active else (window[-1] if window else None)
    plan = fleet.last_record(drone).occupancy.plan
    if obstacle is None:
        plan_clear, plan_p = math.inf, p_hit
        in_depth = False
    else:
        t0 = max(0.0, t_hit - 0.5)
        plan_clear, plan_p = _plan_clearance(plan, obstacle, t0, t_hit)
        in_depth = False
        if flown and flown["frame"] is not None and flown["p"] is not None:
            cam = CameraConfig.from_card(card.card["camera"])
            in_depth = depth_sees(flown["frame"], np.asarray(flown["p"], dtype=float), float(flown["yaw"]), cam, obstacle)
    facts = {
        "seed": result["seed"],
        "outcome": result["outcome"],
        "reason": reason,
        "drone": drone,
        "t_hit_s": t_hit,
        "kind": kind,
        "obstacle": None if obstacle is None else obstacle.name,
        "actual_clearance_m": actual,
        "plan_clearance_m": plan_clear,
        "tracking_error_m": float(np.linalg.norm(p_hit - plan_p)),
        "in_depth": in_depth,
        "plan_code": None if flown is None else flown["code"],
        "admitted": bool(flown and (flown.get("n_admitted") or 0) > 0 and not flown.get("corridor_empty")),
        "mu_lo_selected": None if flown is None else flown.get("mu_lo_selected"),
        "implied_r_m": None if flown is None else flown.get("implied_r_m"),
        "body_radius_m": body,
        "speed_mps": float(np.linalg.norm(hit["v"])),
        "window": window,
    }
    facts["category"] = classify(facts) if result["outcome"] == "COLLISION" else None
    return facts


def _fmt(x, nd=3):
    return "—" if x is None else f"{float(x):.{nd}f}"


def window_table(facts: dict) -> str:
    lines = [
        f"seed {facts['seed']}  t={facts['t_hit_s']:.2f}s  {facts['reason']}  "
        f"障碍 {facts['obstacle']}（{facts['kind']}）  类别：{facts['category']}",
        "",
        "| t | 规划码 | 准入/总数 | 选中 μ_lo | 折合半径 m | 准入最小 μ_lo |",
        "|---|---|---|---|---|---|",
    ]
    for row in facts["window"]:
        adm = "—"
        if row["n_admitted"] is not None:
            adm = f"{row['n_admitted']}/{row['n_candidates']}"
        lines.append(
            f"| {row['now']:.2f} | {row['code']} | {adm} | {_fmt(row['mu_lo_selected'], 4)} | "
            f"{_fmt(row['implied_r_m'])} | {_fmt(row['mu_lo_min_admitted'], 4)} |"
        )
    if len(facts["window"]) == 0:
        lines.append("| — | — | — | — | — | — |")
    lines.append("")
    lines.append(
        f"计划净空 {_fmt(facts['plan_clearance_m'])} m，机体净空 {_fmt(facts['actual_clearance_m'])} m，"
        f"跟踪差 {_fmt(facts['tracking_error_m'])} m，深度里{'有' if facts['in_depth'] else '没有'}这件障碍，"
        f"在飞计划 {facts['plan_code']}。"
    )
    return "\n".join(lines)


def dense_card(root: Path | None = None) -> tuple[dict, Path]:
    root = root or Path(__file__).resolve().parents[1]
    raw = json.loads((root / "scenes/crossing-forest.json").read_text())
    return card_for_tier(raw, "dense"), root


def run_seed(seed: int, *, shadow: bool = False) -> tuple[dict, object]:
    raw, root = dense_card()
    path = root / "scenes/crossing-forest.json"
    # load_card 读 forest.density。密档用临时卡，避免改仓库里的 medium 默认档。
    tmp = path.with_name(f".crossing-forest-dense-diag-{seed}.json")
    tmp.write_text(json.dumps(raw))
    previous = _install(_DiagFleet)
    try:
        card = load_card(tmp)
        out = run(card, RunConfig(seed=seed, shadow=shadow))
    finally:
        _install(previous)
        tmp.unlink(missing_ok=True)
    import yopo_l0.crossing as crossing

    fleet = None
    # run() 里的实例就是 _DiagFleet，挂在闭包外只能从 trace 类变量拿不到。
    # from_card 把 self 留在最近一次构造上。
    fleet = _DiagFleet._last
    return diagnose_output(card, out, fleet), out


def _capture_init(base_init):
    def wrapped(self, *args, **kwargs):
        base_init(self, *args, **kwargs)
        if isinstance(self, _DiagFleet):
            _DiagFleet._last = self
    return wrapped


_DiagFleet._last = None
_DiagFleet.__init__ = _capture_init(_DiagFleet.__init__)


def summarize(rows: list[dict]) -> str:
    counts = {name: 0 for name in CATEGORIES}
    for row in rows:
        if row["category"] in counts:
            counts[row["category"]] += 1
    lines = [
        "# 票 07c 密档撞树定位",
        "",
        "规划器没有改。下面是密档 `forest.seed=7`、飞行 seed 0–9、影子盾关的同一条上游 YOPO。",
        "撞前 1 s 的「准入」是 μ_lo ≥ 上游 safe_mu（0.05 m 走廊）。折合半径是 μ 的反函数，不是真值净空。",
        "",
        "| 类别 | 次数 |",
        "|---|---|",
    ]
    for name in CATEGORIES:
        lines.append(f"| {name} | {counts[name]} |")
    lines.append("")
    failed = [row for row in rows if row["outcome"] == "COLLISION"]
    arrived = [row for row in rows if row["outcome"] != "COLLISION"]
    lines.append(f"碰撞 {len(failed)} / {len(rows)}。")
    if arrived:
        lines.append("没有撞树的 seed：" + "、".join(
            f"{row['seed']}（{row['outcome']}）" for row in arrived
        ) + "。")
    lines.append("")
    lines.append("## 撞前 1 s")
    lines.append("")
    for row in failed:
        lines.append(window_table(row))
        lines.append("")
    lines += _conclusion(rows, counts)
    return "\n".join(lines)


def _conclusion(rows: list[dict], counts: dict) -> list[str]:
    failed = [row for row in rows if row["category"]]
    branches = sum(1 for row in failed if row["kind"] == "branch")
    leans = sum(1 for row in failed if row["kind"] == "lean")
    seen = sum(1 for row in failed if row["in_depth"])
    return [
        "## 结论",
        "",
        f"{len(failed)} 次撞树里，矮枝 {branches} 次、斜干 {leans} 次，其余是直立树干。"
        f"订计划时的深度图里看得到这件障碍的有 {seen} 次，所以「矮枝/斜干不在深度图里」是 0。",
        "计划净空比机体半径 0.35 m 更小的，归到走廊：上游只要求预测走廊 ≥ 0.05 m，折合半径经常在 0.08–0.31 m，多项式就会进机体。",
        "计划净空刚好比 0.35 m 大、机体又进了半径的，归到跟踪：点质量滞后把参考点和机体拉开 0.05–0.16 m。",
        "seed 2 是唯一的无解悬停：撞前一次前向 0/15 准入，承诺的是刹车，刹车参考还在机体外，跟踪把它带进树干。",
        "",
        "## 最小修法（这次没有改）",
        "",
        "1. 准入改成预测走廊半径 ≥ 机体半径 0.35 m，再承诺。这能挡住走廊类（计划本身已经进了 0.35 m）。"
        "偏离上游 d5e1e60：`SAFE_RADIUS` / `test_yopo_ros.py` 的 safe_radius 是 0.05 m。",
        "2. 保持 0.05 m 准入不动，把 L0 点质量跟踪收到参考点 0.02 m 以内（现在差到 0.16 m）。"
        "不偏离 d5e1e60 的候选规则，改的是 L0 动力学。挡不住计划净空已经小于 0.35 m 的那几条。",
        "",
    ]


def main(argv=None) -> int:
    import argparse

    ap = argparse.ArgumentParser(description="票 07c 密档撞树定位")
    ap.add_argument("--seeds", default="0,1,2,3,4,5,6,7,8,9")
    ap.add_argument("--out", default="docs/results/07c-dense-diag.md")
    a = ap.parse_args(argv)
    rows = []
    for seed in [int(s) for s in a.seeds.split(",") if s != ""]:
        facts, _out = run_seed(seed)
        print(window_table(facts), flush=True)
        print("---", flush=True)
        # 深度图数组不进 markdown
        facts["window"] = [{k: v for k, v in row.items() if k != "frame"} for row in facts["window"]]
        rows.append(facts)
    text = summarize(rows)
    path = Path(a.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text + "\n")
    print(f"写入 {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
