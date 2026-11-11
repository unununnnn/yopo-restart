"""票 E-C4：八机开阔对向只报数；密度场几何单独成文件，不改林带三档。"""

import importlib.util
import json
import math
import sys

import numpy as np
import pytest

from yopo_l0.criteria import active_gates
from yopo_l0.forest import TIERS, generate_forest
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import Cylinder, Ground, load_card

from conftest import ROOT

CARD = ROOT / "scenes/swap-8-open.json"
RADIUS_M = 8.0
HEIGHT_M = 1.5
DISCLAIMER = "这是假设，不是论文复现"
DENSITY_FILES = (
    ("ego-density-014.json", 0.14, 56),
    ("ego-density-028.json", 0.28, 112),
    ("ego-density-042.json", 0.42, 168),
)


def _density_script():
    path = ROOT / "scripts/ego-c4-density-field.py"
    spec = importlib.util.spec_from_file_location("ego_c4_density_field", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_swap8_card_is_report_only_antipodal_circle():
    raw = json.loads(CARD.read_text())
    assert raw["name"] == "swap-8-open"
    assert raw["geometry"] == "open-area.json"
    assert raw["task"]["type"] == "goto"
    assert raw["seeds"] == [0]
    assert raw["criteria"]["frozen"] is False
    assert "gates" not in raw["criteria"]
    assert "policy_params" not in raw
    assert "geometric_fallback" not in raw
    assert "forest" not in raw
    assert "20.3" in raw["note"] and "14.3" in raw["note"]
    assert "8.0" in raw["note"]
    assert "speed_mps" not in raw["task"]
    assert raw["dynamics"]["a_max"] == 6.0
    starts = []
    for i, drone in enumerate(raw["drones"]):
        assert drone["id"] == f"uav{i}"
        start = np.asarray(drone["start_enu"], dtype=float)
        goal = np.asarray(drone["goal_enu"], dtype=float)
        assert start[2] == HEIGHT_M and goal[2] == HEIGHT_M
        assert math.hypot(start[0], start[1]) == pytest.approx(RADIUS_M)
        assert math.hypot(goal[0], goal[1]) == pytest.approx(RADIUS_M)
        assert np.allclose(start[:2] + goal[:2], 0.0)
        starts.append(start[:2])
    assert len(raw["drones"]) == 8
    angles = sorted(math.atan2(float(p[1]), float(p[0])) % (2.0 * math.pi) for p in starts)
    for i in range(8):
        step = (angles[(i + 1) % 8] - angles[i]) % (2.0 * math.pi)
        assert step == pytest.approx(math.pi / 4)
    chord = 2.0 * RADIUS_M * math.sin(math.pi / 8)
    for i in range(8):
        d = float(np.linalg.norm(starts[i] - starts[(i + 1) % 8]))
        assert d == pytest.approx(chord)

    card = load_card(CARD)
    assert all(isinstance(o, Ground) for o in card.obstacles)
    assert active_gates(card, policy="upstream") is None
    assert active_gates(card, policy="hover") is None
    listed = (ROOT / "scenes/suite.txt").read_text()
    assert "swap-8-open" not in listed
    assert "ego-density" not in listed


def test_swap8_card_hash_stable_and_short_run_records_gap():
    card = load_card(CARD)
    again = load_card(CARD)
    assert card.hash == again.hash and len(card.hash) == 64
    out = run(card, RunConfig(seed=0, policy="upstream", shadow=False, duration_s=0.2))
    assert out.result["scene"] == "swap-8-open"
    assert out.result["scene_card_hash"] == card.hash
    assert out.result["n_drones"] == 8
    assert out.result["tier"] == "L0"
    assert out.result["policy"] == "upstream"
    assert "inter_drone_filter" not in out.result
    assert "geometric_fallback" not in out.result
    # 起点弦长约 6.12 m，短跑还没飞到圆心。
    assert out.result["min_inter_drone_distance_m"] > 2 * 0.35
    assert out.result["outcome"] != "COLLISION"


def test_density_fields_match_stated_assumptions_and_load(tmp_path):
    mod = _density_script()
    assert mod.OBSTACLE_RADIUS_M == 0.20
    assert mod.MAP_SIDE_M == 20.0
    assert mod.DISCLAIMER == DISCLAIMER
    area = 20.0 * 20.0
    for name, target, n in DENSITY_FILES:
        path = ROOT / "scenes" / name
        raw = json.loads(path.read_text())
        assert raw == mod.build(target, n)
        assert raw["note"].startswith(DISCLAIMER)
        header = raw["header"]
        assert header["disclaimer"] == DISCLAIMER
        assert header["obstacle_radius_m"] == 0.20
        assert header["map_side_m"] == 20.0
        assert header["map_area_m2"] == area
        assert header["target_density_per_m2"] == target
        assert header["n_obstacles"] == n
        assert header["realized_density_per_m2"] == n / area
        assert header["realized_density_per_m2"] == target
        assert len(raw["cylinders"]) == n
        assert raw["boxes"] == []
        lo = -10.0 + 0.20
        hi = 10.0 - 0.20
        centers = []
        for cyl in raw["cylinders"]:
            assert cyl["radius"] == 0.20
            assert cyl["z_lo"] == 0.0 and cyl["z_hi"] == 2.5
            x, y = cyl["center_xy"]
            assert lo <= x <= hi and lo <= y <= hi
            centers.append((x, y))
        for i, (x, y) in enumerate(centers):
            for ox, oy in centers[:i]:
                assert (x - ox) ** 2 + (y - oy) ** 2 >= (0.40 - 1e-9) ** 2

        # 加载器读的是场景卡上的 geometry 路径。几何文件本身不是场景卡。
        (tmp_path / name).write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
        wrapper = tmp_path / f"load-{name}"
        wrapper.write_text(
            json.dumps(
                {
                    "name": f"load-{name}",
                    "geometry": name,
                    "ground_z": 0.0,
                    "drones": [],
                    "task": {"type": "hover", "duration_s": 1.0},
                    "criteria": {"frozen": False},
                }
            ),
            encoding="utf-8",
        )
        card = load_card(wrapper)
        cylinders = [o for o in card.obstacles if isinstance(o, Cylinder)]
        assert len(cylinders) == n
        assert any(isinstance(o, Ground) for o in card.obstacles)
        assert card.card["geometry"]["header"]["disclaimer"] == DISCLAIMER


def test_forest_tiers_unchanged_by_density_files():
    assert TIERS == ("sparse", "medium", "dense")
    _geom, stats = generate_forest({"density": "dense", "seed": 7})
    assert stats["surface_gap_spec_m"] == 1.7
    assert stats["n_trunks"] == 45
    assert stats["n_middle"] == 35
