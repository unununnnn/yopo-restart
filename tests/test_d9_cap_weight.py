"""D9：统计与编排的纯函数检查（不跑闭环）。"""
import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("t_d9_cap_weight", ROOT / "scripts/distill/d9/cap_weight.py")
M = importlib.util.module_from_spec(spec)
sys.modules["t_d9_cap_weight"] = M
spec.loader.exec_module(M)


def test_sign_test_exact():
    assert M.sign_test([1, 1, 1, 1, 1])["p_two_sided"] == pytest.approx(0.0625)
    assert M.sign_test([1, -1, 0])["p_two_sided"] == pytest.approx(1.0)
    r = M.sign_test([0, 0])
    assert r["n_tie"] == 2 and r["p_two_sided"] == 1.0


def test_bootstrap_ci_brackets_mean():
    r = M.bootstrap_ci([0, 10, 20, 10, 10], n_boot=2000)
    assert r["mean"] == pytest.approx(10.0)
    assert r["ci95"][0] <= 10.0 <= r["ci95"][1]
    assert M.bootstrap_ci([5, 5, 5], n_boot=100)["ci95"] == [5.0, 5.0]


def test_eval_seeds_disjoint_from_d7_train_seeds():
    d7 = M.d8().d7()
    for k, seeds in d7.TRAIN_SEEDS.items():
        for scene, ss in seeds.items():
            assert not set(ss) & set(M.d8().EVAL_SEEDS[scene])


def test_drone_timing_reads_radius_and_hold():
    class Card:
        card = {"criteria": {"goal_tolerance_m": 0.4, "goal_speed_mps": 0.2},
                "drones": [{"id": "uav2", "goal_enu": [0.0, 0.0, 1.0]}]}

    ticks = [{"uav": "uav2", "t": t / 10, "p": [max(2.0 - t / 10, 0.3), 0.0, 1.0], "v": [1.0 if t < 18 else 0.0, 0, 0]}
             for t in range(0, 31)]
    (d,) = M.d8().drone_timing(Card(), ticks)
    assert d["t_arrive_radius_s"] == pytest.approx(0.5)
    assert d["t_goal_tol_s"] == pytest.approx(1.6)
    assert d["longest_hold_s"] == pytest.approx(1.2)
