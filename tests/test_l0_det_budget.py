"""票 L0-DET：后段时间预算在仿真里按评估次数算，机器忙不忙不改结果。

本票之前 ``_minimize`` 用 0.02 s 墙钟截断 L-BFGS。同一张 bspline 卡、同一个 seed，
box 上连跑两遍迭代数就不一样（4.7 对 6.0），轨迹和到达时间跟着变。
这里用假时钟把机器“拖慢”，不靠真负载：每读一次钟走 50 ms。
"""

import numpy as np
import pytest
from yopo_swarm_core.plan import quintic
from yopo_swarm_core.trajectory import PiecewisePolynomial

import yopo_l0.backend.bspline as bspline
from yopo_l0.backend.bspline import _eval_cap, refine_yopo
from yopo_l0.backend.config import BackendConfig, config_from_params
from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import Cylinder, load_card

from conftest import ROOT


class _SlowClock:
    """每读一次 perf_counter 走 step 秒，模拟被别的进程抢走 CPU 的机器。"""

    def __init__(self, step: float = 0.05):
        self.t = 0.0
        self.step = step

    def perf_counter(self) -> float:
        self.t += self.step
        return self.t


def _line() -> PiecewisePolynomial:
    p0 = np.array([-2.0, 0.0, 1.5])
    head = np.array([p0, np.array([2.0, 0.0, 0.0]), np.zeros(3)])
    end = np.array([p0 + head[1] * 1.6, head[1].copy(), np.zeros(3)])
    return PiecewisePolynomial(0.0, [quintic(head, end, 1.6)])


def _samples(traj) -> np.ndarray:
    ts = np.linspace(traj.t_start, traj.t_end, 33)
    return np.array([traj.evaluate(float(t), k) for t in ts for k in range(3)])


def _refine(cfg):
    cyl = Cylinder("trunk", (0.0, 0.15), 0.45, 0.0, 4.0)
    return refine_yopo(_line(), cfg, (cyl,))


def test_default_budget_is_eval_count_and_wall_is_opt_in(monkeypatch):
    monkeypatch.delenv("YOPO_BACKEND_BUDGET", raising=False)
    cfg = config_from_params({"backend": {"enabled": True}}, a_max=6.0)
    assert cfg.budget == "evals" and _eval_cap(cfg) == 10
    assert config_from_params({"backend": {"enabled": True, "budget": "wall"}}, a_max=6.0).budget == "wall"
    monkeypatch.setenv("YOPO_BACKEND_BUDGET", "wall")
    assert config_from_params({"backend": {"enabled": True}}, a_max=6.0).budget == "wall"
    with pytest.raises(ValueError):
        BackendConfig(budget="cpu")
    assert _eval_cap(BackendConfig(time_budget_s=1e-4)) == 1
    assert _eval_cap(BackendConfig(time_budget_s=5.0)) == 2500


def test_slow_machine_gives_the_same_refinement(monkeypatch):
    cfg = BackendConfig(a_max=6.0, v_max=6.0, max_iterations=8, clearance_m=0.35)
    out_fast, rec_fast = _refine(cfg)
    monkeypatch.setattr(bspline, "time", _SlowClock())
    out_slow, rec_slow = _refine(cfg)
    assert rec_fast["applied"] is True, rec_fast
    assert rec_slow["iterations"] == rec_fast["iterations"]
    assert rec_slow["budget_cut"] == rec_fast["budget_cut"]
    assert np.array_equal(_samples(out_slow), _samples(out_fast))


def test_wall_budget_still_cuts_on_a_slow_clock(monkeypatch):
    cfg = BackendConfig(a_max=6.0, v_max=6.0, max_iterations=8, clearance_m=0.35, budget="wall")
    _out, fast = _refine(cfg)
    monkeypatch.setattr(bspline, "time", _SlowClock())
    _out, slow = _refine(cfg)
    assert slow["budget_cut"] is True
    assert slow["iterations"] < fast["iterations"] or not fast["budget_cut"]


def _strip(result: dict) -> dict:
    out = {k: v for k, v in result.items() if k not in WALL_FIELDS}
    if isinstance(out.get("backend"), dict):
        out["backend"] = {k: v for k, v in out["backend"].items() if k != "time_ms"}
    return out


def test_bspline_card_is_deterministic_on_a_slow_machine(monkeypatch):
    card = load_card(ROOT / "scenes/g2-pillar-bspline.json")
    cfg = RunConfig(seed=0, policy="upstream", duration_s=2.0)
    fast = run(card, cfg)
    assert fast.result["backend"]["calls"] > 0
    monkeypatch.setattr(bspline, "time", _SlowClock())
    slow = run(load_card(ROOT / "scenes/g2-pillar-bspline.json"), cfg)
    assert fast.ticks == slow.ticks
    assert _strip(fast.result) == _strip(slow.result)
