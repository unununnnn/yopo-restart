"""节点决策金样（#34 / Y1）：本仓库移植的节点逻辑与上游 YOPO-MINCO d5e1e60 `test_yopo_ros.py` 逐帧一致。

金样由 `scripts/make_node_golden.py` 以桩 rospy 直接运行上游节点生成（≥ 200 帧，含 G2 真实网络输出、
上游网络前向、合成边界）。本测试离线，只读 npz。

- 选择与 brake 判定：精确相等；
- 目标截断 / yaw / yaw_dot：float64 ≤ 1e-12；
- 保真登记 (b)：上游 brake 帧在本仓库不可能飞任何候选（`order == ()`，`CORRIDOR_EMPTY`）。
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from yopo_swarm_core.candidates import (
    UPSTREAM_DECODER,
    corridor_mu_lo,
    rank_candidates,
    select_action,
)
from yopo_swarm_core.infer_ipc import ARRIVE_RADIUS, GOAL_LENGTH, arrived, body_obs, goal_obs
from yopo_swarm_core.yaw import CTRL_DT, MAX_YAW_RATE, calculate_yaw

GOLDEN = Path(__file__).parent / "golden" / "minco_node_d5e1e60.npz"
DEC = UPSTREAM_DECODER


@pytest.fixture(scope="module")
def g():
    with np.load(GOLDEN) as z:
        return {k: z[k] for k in z.files}


def _frames(g):
    return range(len(g["src"]))


def _last(g, i):
    li = g["last_inner_w"][i]
    return None if np.isnan(li).any() else li


def test_golden_provenance_and_coverage(g):
    meta = json.loads(str(g["meta"]))
    assert meta["upstream_sha"] == "d5e1e60f83cc991602f142d9b6b96204a24d6b9f"
    assert meta["frames"] == len(g["src"]) >= 200
    assert meta["safe_mu"] == DEC.safe_mu
    assert meta["goal_length"] == GOAL_LENGTH
    assert meta["ctrl_dt"] == CTRL_DT
    assert meta["plan_from_reference"] is True  # D2
    src = [s.split(":")[0] for s in g["src"]]
    assert {"g2", "network", "synthetic"} <= set(src)
    # 覆盖面：截断、|gz| ≥ 10、brake、到达、topk 连续性都必须出现
    rel = g["goal"] - g["desire_pos"]
    assert (np.linalg.norm(rel, axis=1) > GOAL_LENGTH).sum() >= 50
    assert (np.abs(rel[:, 2]) >= GOAL_LENGTH).sum() >= 5
    assert 20 <= g["brake"].sum() < len(src)
    assert 5 <= g["arrive"].sum() < len(src)
    assert ((g["topk"] > 1) & ~np.isnan(g["last_inner_w"]).any(axis=1)).sum() >= 20


def test_corridor_mu_lo_matches(g):
    for i in _frames(g):
        np.testing.assert_allclose(
            corridor_mu_lo(g["radius"][i], DEC), g["mu_lo"][i], rtol=0, atol=1e-7
        )


def test_admission_boundary_is_float64_like_upstream(g):
    """μ_lo 恰为 float32(safe_mu) 的帧：上游按 float64 比较不准入，本仓库必须同样不准入。"""
    hit = 0
    for i in _frames(g):
        m = g["mu_lo"][i]
        edge = m == np.float32(DEC.safe_mu)
        if edge.any():
            hit += 1
            r = rank_candidates(g["score"][i], g["radius"][i], DEC)
            assert not r.admitted[edge].any(), i
    assert hit >= 10


def test_select_action_is_exact(g):
    for i in _frames(g):
        got = select_action(
            g["score"][i],
            g["inner_w"][i],
            g["mu_lo"][i],
            DEC.safe_mu,
            int(g["topk"][i]),
            _last(g, i),
        )
        assert got == (int(g["action_id"][i]), bool(g["brake"][i])), (i, g["src"][i])


def test_rank_candidates_first_is_upstream_choice_and_brake_never_flies(g):
    for i in _frames(g):
        r = rank_candidates(
            g["score"][i], g["radius"][i], DEC, g["inner_w"][i], int(g["topk"][i]), _last(g, i)
        )
        if g["brake"][i]:
            # 保真登记 (b)：上游会飞全体最高分，本仓库整帧拒绝
            assert r.corridor_empty and r.order == () and r.preferred is None, i
        else:
            assert r.preferred == int(g["action_id"][i]), (i, g["src"][i])
            assert all(r.admitted[j] for j in r.order)
            assert sorted(r.order) == sorted(np.flatnonzero(r.admitted).tolist())


def test_goal_obs_and_body_obs_match(g):
    for i in _frames(g):
        gw = goal_obs(g["goal"][i], g["desire_pos"][i], GOAL_LENGTH)
        np.testing.assert_allclose(gw, g["goal_local_w"][i], rtol=0, atol=1e-12)
        assert np.linalg.norm(gw) <= GOAL_LENGTH + 1e-9
        # D2：plan_from_reference=True，速度/加速度/目标基点都取参考（期望）状态
        obs = body_obs(
            g["desire_vel"][i], g["desire_acc"][i], g["goal"][i], g["desire_pos"][i], g["R_wc"][i]
        )
        np.testing.assert_allclose(obs[0], g["obs_raw"][i], rtol=1e-6, atol=1e-5)


def test_arrival_matches(g):
    for i in _frames(g):
        assert arrived(g["odom_pos"][i], g["goal"][i], ARRIVE_RADIUS) == bool(g["arrive"][i]), i


@pytest.mark.parametrize("form", ["fly", "brake"])
def test_calculate_yaw_matches(g, form):
    vel = g["desire_vel"] if form == "fly" else g[f"yaw_goal_{form}"]
    for i in _frames(g):
        yaw, yaw_dot = calculate_yaw(vel[i], g[f"yaw_goal_{form}"][i], g["last_yaw"][i], CTRL_DT)
        assert abs(yaw - g[f"yaw_{form}"][i]) <= 1e-12, i
        assert abs(yaw_dot - g[f"yawdot_{form}"][i]) <= 1e-12, i
        assert abs(yaw_dot) <= MAX_YAW_RATE * np.pi + 1e-12
