"""票 01 验收：只看结果记录 / 规划器输入摘要，不看内部调用顺序。"""

import json
import resource
import subprocess
import sys

from yopo_l0.runner import WALL_FIELDS, RunConfig, run
from yopo_l0.scene import load_card

from conftest import G2_CARD, ROOT


def strip_wall(r):
    return {k: v for k, v in r.items() if k not in WALL_FIELDS}


def test_g2_geometry_is_l1_copy(g2_geometry):
    # 几何与 L1 的 g2-scene.json 同一份：墙、顶、柱子都在，柱子位置不变
    names = {b["name"] for b in g2_geometry["boxes"]}
    assert {"ceiling", "left_wall", "right_wall", "end_wall", "back_wall", "pillar"} <= names
    pillar = next(b for b in g2_geometry["boxes"] if b["name"] == "pillar")
    assert pillar["lo"] == [2.7, -0.4, 0.0] and pillar["hi"] == [3.3, 0.4, 2.8]


def test_hover_succeeds_and_records_fields():
    out = run(load_card(G2_CARD), RunConfig(seed=0, policy="hover", shadow=False))
    r = out.result
    assert r["outcome"] == "SUCCESS"
    for key in ("commit", "scene_card_hash", "seed", "tier", "policy", "shadow",
                "min_obstacle_distance_m", "wall_time_s", "sim_duration_s"):
        assert key in r
    assert r["tier"] == "L0" and r["policy"] == "hover" and r["shadow"] is False
    assert r["sim_duration_s"] == 10.0
    assert len(out.ticks) == r["ticks"] == 1000


def test_same_seed_is_deterministic():
    card = load_card(G2_CARD)
    a = run(card, RunConfig(seed=3, policy="hover", record_planner_inputs=True))
    b = run(card, RunConfig(seed=3, policy="hover", record_planner_inputs=True))
    assert strip_wall(a.result) == strip_wall(b.result)
    assert a.ticks == b.ticks
    assert a.planner_inputs == b.planner_inputs


def test_different_seed_changes_depth_noise_only():
    card = load_card(G2_CARD)
    a = run(card, RunConfig(seed=0, policy="hover", record_planner_inputs=True))
    b = run(card, RunConfig(seed=1, policy="hover", record_planner_inputs=True))
    assert a.planner_inputs != b.planner_inputs  # 深度噪声随 seed 变


def test_planner_inputs_have_no_scene_geometry(make_card, g2_geometry):
    # 在相机看不到（身后、视场外）的地方加一个盒子：几何变了，规划器输入必须逐位不变
    behind = {**g2_geometry, "boxes": g2_geometry["boxes"] + [
        {"name": "hidden", "lo": [-2.5, -1.0, 0.0], "hi": [-2.0, 1.0, 2.0]}]}
    a = make_card(geometry=g2_geometry)
    b = make_card(geometry=behind)
    assert a.hash != b.hash
    ra = run(a, RunConfig(seed=0, policy="hover", record_planner_inputs=True))
    rb = run(b, RunConfig(seed=0, policy="hover", record_planner_inputs=True))
    assert ra.planner_inputs == rb.planner_inputs
    # 对照：在视场内加障碍，输入随深度变（证明上一条断言不是空的）
    front = {**g2_geometry, "boxes": g2_geometry["boxes"] + [
        {"name": "front", "lo": [1.5, -0.5, 1.0], "hi": [1.7, 0.5, 2.0]}]}
    rc = run(make_card(geometry=front), RunConfig(seed=0, policy="hover", record_planner_inputs=True))
    assert rc.planner_inputs != ra.planner_inputs


def test_collision_truth_uses_035_body_radius(make_card):
    # 柱子 x 面在 2.7：机心距柱 0.34 m → 碰撞；0.36 m → 不碰
    hit = run(make_card(drones=[{"id": "uav0", "start_enu": [2.36, 0.0, 1.5]}]), RunConfig(policy="hover"))
    assert hit.result["outcome"] == "COLLISION"
    assert hit.result["min_obstacle_distance_m"] <= 0.35
    ok = run(make_card(drones=[{"id": "uav0", "start_enu": [2.34, 0.0, 1.5]}]), RunConfig(policy="hover"))
    assert ok.result["outcome"] == "SUCCESS"
    assert 0.35 < ok.result["min_obstacle_distance_m"] < 0.37
    # 地面同样按 0.35 m 判
    low = run(make_card(drones=[{"id": "uav0", "start_enu": [0.0, 0.0, 0.3]}]), RunConfig(policy="hover"))
    assert low.result["outcome"] == "COLLISION"


def test_dynamics_converge_to_hover_point(make_card):
    card = make_card(drones=[{"id": "uav0", "start_enu": [0.0, 0.0, 1.2], "hover_enu": [0.0, 0.0, 1.5]}])
    out = run(card, RunConfig(policy="hover"))
    assert out.result["outcome"] == "SUCCESS"
    assert out.ticks[-1]["goal_error_m"] < 0.01
    assert out.ticks[0]["goal_error_m"] > 0.29  # 一阶滞后：第一 tick 几乎没动


def test_cli_single_run_under_60s(tmp_path):
    before = resource.getrusage(resource.RUSAGE_CHILDREN)
    p = subprocess.run(
        [sys.executable, "-m", "yopo_l0.run", "--scene", str(G2_CARD), "--seed", "0", "--policy", "hover", "--out", str(tmp_path)],
        cwd=ROOT, capture_output=True, text=True, timeout=120,
    )
    assert p.returncode == 0, p.stderr
    r = json.loads((tmp_path / "result.json").read_text())
    after = resource.getrusage(resource.RUSAGE_CHILDREN)
    # 票 L0-DET：墙钟随机器负载走，只验它有值。60 s 口径改看子进程 CPU 时间。
    assert r["wall_time_s"] > 0
    assert (after.ru_utime - before.ru_utime) + (after.ru_stime - before.ru_stime) < 60
    assert sum(1 for _ in (tmp_path / "ticks.jsonl").open()) == r["ticks"]
