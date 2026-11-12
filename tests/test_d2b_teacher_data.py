"""票 D2b：扩种网格、到达时间、sha256 清单。不改 assumptions.py，不改 D2 的 192 格。"""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from teacher.assumptions import SEEDS
from teacher.d2b import arrival_time_s, build_manifest, format_d2b_table, sha256_file, summarize_d2b
from teacher.d2b_grid import (
    DEFAULT_SEED_COUNT,
    DENSITIES_PER_M2,
    DRONE_COUNTS,
    D2_SUCCESS_TOTAL,
    TARGET_MIN_SUCCESS,
    grid_size,
    iter_d2b_scenarios,
)
from teacher.scenarios import iter_scenarios, make_scenario


def test_d2_grid_is_unchanged():
    assert SEEDS == tuple(range(8))
    rows = list(iter_scenarios())
    assert len(rows) == 192
    assert any(r["density_per_m2"] == 0.42 for r in rows)


def test_d2b_grid_default_size_and_has_no_density_042():
    rows = list(iter_d2b_scenarios())
    assert len(rows) == grid_size()
    assert len(rows) == 3 * 2 * 2 * DEFAULT_SEED_COUNT
    assert len({r["id"] for r in rows}) == len(rows)
    assert DENSITIES_PER_M2 == (0.0, 0.14, 0.28)
    assert DRONE_COUNTS == (1, 3)
    assert all(r["density_per_m2"] in DENSITIES_PER_M2 for r in rows)
    assert all(r["n_drones"] in DRONE_COUNTS for r in rows)
    assert all(r["density_per_m2"] != 0.42 for r in rows)
    assert {r["seed"] for r in rows} == set(range(DEFAULT_SEED_COUNT))


def test_six_open_is_only_density_zero_and_appended():
    base = list(iter_d2b_scenarios(seed_count=4, include_six_open=False))
    rows = list(iter_d2b_scenarios(seed_count=4, include_six_open=True))
    assert len(rows) == grid_size(4, True)
    assert len(base) == 3 * 2 * 2 * 4
    extra = rows[len(base) :]
    assert len(extra) == 2 * 4
    assert all(r["n_drones"] == 6 and r["density_per_m2"] == 0.0 for r in extra)
    assert all(r["n_drones"] != 6 for r in base)


def test_high_seed_is_deterministic_and_placement_completes():
    a = make_scenario(0.28, 3, "headon", 100)
    b = make_scenario(0.28, 3, "headon", 100)
    c = make_scenario(0.28, 3, "headon", 101)
    assert a["id"] == "d028-n3-headon-s100"
    assert a["cylinders"] == b["cylinders"]
    assert a["cylinders"] != c["cylinders"]
    assert a["placement_complete"] is True
    assert a["n_obstacles"] == 112
    assert a["assumption"]["text"].startswith("地图边长和障碍半径是假设")


def test_target_is_an_order_of_magnitude_above_d2():
    assert D2_SUCCESS_TOTAL == 90
    assert TARGET_MIN_SUCCESS == 900


def test_arrival_time_matches_the_hold_window():
    time_s = np.linspace(0.0, 12.0, 121)
    pos = np.zeros((121, 1, 3))
    pos[:, 0, 0] = np.linspace(-9.0, 9.0, 121)
    pos[:, 0, 2] = 1.0
    # 从 t=10 s 起停在目标处。
    pos[100:, 0] = (9.0, 0.0, 1.0)
    vel = np.zeros_like(pos)
    vel[:100, 0, 0] = 1.8
    goals = np.array([[9.0, 0.0, 1.0]])
    assert arrival_time_s(time_s, pos, vel, goals) == pytest.approx(10.0)
    short = arrival_time_s(time_s[:103], pos[:103], vel[:103], goals)
    assert short is None


def test_manifest_sha256_roundtrip(tmp_path):
    run = tmp_path / "runs" / "d000-n1-parallel-s0"
    run.mkdir(parents=True)
    payload = b"d2b-teacher-fixture\n"
    (run / "trajectory.npz").write_bytes(payload)
    (run / "record.json").write_text("{}\n", encoding="utf-8")
    (run / "scene.json").write_text("{}\n", encoding="utf-8")
    (run / "geometry.json").write_text("{}\n", encoding="utf-8")
    manifest = build_manifest(tmp_path)
    assert manifest["schema"] == "yopo-teacher-d2b-manifest/1"
    assert "不是论文复现" in manifest["assumption"]
    assert manifest["n_files"] == 4
    found = {row["kind"]: row for row in manifest["files"]}
    assert found["trajectory.npz"]["sha256"] == hashlib.sha256(payload).hexdigest()
    assert found["trajectory.npz"]["bytes"] == len(payload)
    assert found["trajectory.npz"]["sha256"] == sha256_file(run / "trajectory.npz")


def test_summarize_table_records_success_collision_and_arrival(tmp_path):
    run = tmp_path / "runs" / "d014-n1-parallel-s0"
    run.mkdir(parents=True)
    time_s = np.linspace(0.0, 2.0, 21)
    pos = np.zeros((21, 1, 3))
    pos[:, 0] = (9.0, 0.0, 1.0)
    vel = np.zeros_like(pos)
    np.savez_compressed(
        run / "trajectory.npz",
        time_s=time_s,
        position_m=pos,
        velocity_mps=vel,
        goal_m=np.array([[9.0, 0.0, 1.0]]),
    )
    record = {
        "scenario": {
            "id": "d014-n1-parallel-s0",
            "density_per_m2": 0.14,
            "n_drones": 1,
            "pattern": "parallel",
            "seed": 0,
        },
        "run": {"status": "arrived", "wall_s": 12.0},
        "metrics": {
            "success": True,
            "arrived": True,
            "collision_obstacle": False,
            "collision_inter_drone": False,
            "min_inter_drone_m": None,
            "min_obstacle_surface_m": 1.0,
            "arrival_time_s": 0.0,
        },
    }
    (run / "record.json").write_text(json.dumps(record), encoding="utf-8")
    summary = summarize_d2b(tmp_path / "runs")
    assert summary["overall"]["success"] == 1
    assert summary["target_min_success"] == 900
    text = format_d2b_table(summary)
    assert "这是假设，不是论文复现" in text or "不是论文复现" in text
    assert "0.42" not in text
    assert "1/1" in text
    assert "到达时间" in text
