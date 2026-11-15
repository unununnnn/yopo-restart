"""票 D7：DAgger 编排里的纯函数。不起容器，不跑闭环。"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "distill" / "d7" / "dagger.py"


def _mod():
    spec = importlib.util.spec_from_file_location("d7_dagger_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules["d7_dagger_test"] = module
    spec.loader.exec_module(module)
    return module


def test_train_and_eval_seeds_are_disjoint():
    mod = _mod()
    for scene, eval_seeds in mod.EVAL_SEEDS.items():
        seen = set()
        for k in (1, 2, 3):
            train = set(mod.TRAIN_SEEDS[k][scene])
            assert not (train & set(eval_seeds))
            assert not (train & seen)
            seen |= train
    assert mod.EVAL_SEEDS["open-3drone"] == list(range(10))
    assert mod.EVAL_SEEDS["g2-pillar"] == list(range(5))
    assert mod.ALTITUDES == (1.0, 1.5)


def test_teacher_scenario_seeds_state_and_g2_cylinder():
    mod = _mod()
    job = {
        "id": "r1-g2-pillar-z1.5-s5-t1.0", "round": 1, "scene": "g2-pillar", "altitude_m": 1.5, "seed": 5, "t": 1.0,
        "drones": [{"ros_id": 0, "id": "uav0", "p": [1.0, 0.1, 1.5], "v": [1.0, 0.0, 0.0], "a": [0.0, 0.0, 0.0],
                    "yaw": 0.1, "goal": [5.0, 0.0, 1.5]}],
    }
    scn = mod.teacher_scenario(job)
    assert scn["cylinders"] == mod.G2_TEACHER_CYLINDERS
    assert scn["cylinders"][0]["radius"] == 0.5
    d = scn["drones"][0]
    assert d["start_enu"] == [1.0, 0.1, 1.5]
    assert d["seed_state"]["v"] == [1.0, 0.0, 0.0]
    assert scn["record_after_handover_s"] == mod.AFTER_S
    open_job = dict(job, scene="open-3drone")
    assert mod.teacher_scenario(open_job)["cylinders"] == []


def test_make_jobs_samples_every_half_second_and_filters(tmp_path):
    mod = _mod()
    rdir = tmp_path / "r1" / "rollouts"
    rdir.mkdir(parents=True)
    states = []
    for n in range(0, 81):
        t = round(n * 0.1, 2)
        x = min(6.0, 0.8 * t)
        states.append({"t": t, "id": "uav0", "p": [x, 0.0, 1.5], "v": [0.8, 0.0, 0.0], "a": [0.0, 0.0, 0.0],
                       "yaw": 0.0, "goal": [6.0, 0.0, 1.5], "min_obstacle_m": 1.5})
    body = {"scene": "open-3drone", "altitude_m": 1.5, "seed": 10, "drone_ids": ["uav0"], "states": states}
    (rdir / "open-3drone-z1.5-s10.json").write_text(json.dumps(body))
    jobs = mod.make_jobs(tmp_path, 1)
    ts = [j["t"] for j in jobs]
    assert ts[0] == 0.5
    assert all(abs((t - 0.5) / 0.5 - round((t - 0.5) / 0.5)) < 1e-6 for t in ts)
    assert len(jobs) <= mod.CAP_PER_ROLLOUT
    # 0.8 m/s 走到 6.875 s 时离目标 0.5 m：之后的点不取。
    assert max(ts) < 6.875
    assert (tmp_path / "r1" / "jobs.jsonl").is_file()


def test_state_filter_rejects_near_obstacle_and_out_of_corridor():
    mod = _mod()
    ok = {"p": [1.0, 0.0, 1.5], "goal": [5.0, 0.0, 1.5], "min_obstacle_m": 1.0}
    assert mod._state_ok("g2-pillar", ok)
    assert not mod._state_ok("g2-pillar", dict(ok, min_obstacle_m=0.40))
    assert not mod._state_ok("g2-pillar", dict(ok, p=[1.0, 3.1, 1.5]))
    assert not mod._state_ok("open-3drone", dict(ok, p=[1.0, 0.0, 0.3]))
    assert mod._state_ok("open-3drone", dict(ok, p=[1.0, 3.1, 1.5]))
