"""票 V2：穿林净空回放只出图。场景卡不被改写，--partial 能写出 gif、png 和 summary.json。"""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

from conftest import ROOT

SCRIPT = ROOT / "scripts" / "v2-clearance-replay.py"
REQUIRED = (
    "outcome",
    "arrived",
    "min_obstacle_clearance_m",
    "min_ground_z_m",
    "min_inter_drone_distance_m",
    "files",
)


def load_mod():
    spec = importlib.util.spec_from_file_location("v2_clearance_replay", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 {SCRIPT}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_arms_do_not_rewrite_scene_files_and_frames_carry_the_recorded_fields():
    mod = load_mod()
    solo = ROOT / "scenes" / "ego-density-014-solo.json"
    swap3 = ROOT / "scenes" / "ego-density-014-swap3.json"
    before = {path: path.read_bytes() for path in (solo, swap3)}

    base = mod.build_card(solo, mod.arm_by_id("base"))
    assert "policy_params" not in base.card
    geom = mod.build_card(solo, mod.arm_by_id("geom"))
    upstream = geom.card["policy_params"]["upstream"]
    assert upstream["track_clearance"] is True
    assert upstream["track_clearance_mode"] == "geom"
    assert "empty_brake" not in upstream
    brake = mod.build_card(swap3, mod.arm_by_id("geom+empty_brake"))
    turned = brake.card["policy_params"]["upstream"]
    assert turned["track_clearance"] is True
    assert turned["track_clearance_mode"] == "geom"
    assert turned["empty_brake"] is True
    assert before == {path: path.read_bytes() for path in (solo, swap3)}

    ticks = [
        {"k": 0, "t": 0.01, "uav": "uav0", "p": [0, 0, 1.5], "v": [1, 0, 0], "plan_code": "OK", "min_obstacle_m": 0.50},
        {"k": 0, "t": 0.01, "uav": "uav1", "p": [0, 3, 1.4], "v": [0, 0, 0], "plan_code": "HOVER", "min_obstacle_m": 0.80},
        {"k": 1, "t": 0.02, "uav": "uav1", "p": [0.1, 2.5, 1.4], "v": [0, -1, 0], "plan_code": "OK", "min_obstacle_m": 0.90},
        {"k": 1, "t": 0.02, "uav": "uav0", "p": [0.1, 0, 1.2], "v": [1, 0, -0.2], "plan_code": "OK", "min_obstacle_m": 0.40},
    ]
    frames = mod.frames_from_ticks(ticks)
    assert [row["uav"] for row in frames[0]["drones"]] == ["uav0", "uav1"]
    drone = frames[0]["drones"][0]
    assert drone["p"] == pytest.approx([0, 0, 1.5])
    assert drone["v"] == pytest.approx([1, 0, 0])
    assert drone["plan_code"] == "OK"
    assert drone["min_surface_clearance_m"] == pytest.approx(0.50)
    assert frames[0]["min_surface_clearance_m"] == pytest.approx(0.50)
    assert frames[0]["min_inter_drone_distance_m"] == pytest.approx((9.0 + 0.01) ** 0.5)
    stats = mod.metrics_from_frames(frames)
    assert stats["min_obstacle_clearance_m"] == pytest.approx(0.40)
    assert stats["min_ground_z_m"] == pytest.approx(1.2)
    assert stats["min_inter_drone_distance_m"] == pytest.approx(2.507987)
    solo_frames = mod.frames_from_ticks([ticks[0], ticks[3]])
    assert solo_frames[0]["min_inter_drone_distance_m"] is None
    assert mod.metrics_from_frames(solo_frames)["min_inter_drone_distance_m"] is None


def test_partial_replay_writes_gif_png_and_summary(tmp_path):
    out = tmp_path / "viz"
    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--partial",
            "--out",
            str(out),
            "--scenes",
            "solo,swap3",
            "--seed",
            "0",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        timeout=240,
    )
    assert proc.returncode == 0, proc.stderr[-4000:] + proc.stdout[-2000:]
    summary_path = out / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["report_only"] is True
    assert summary["viz_only"] is True
    assert summary["frozen"] is False
    assert summary["partial"] is True
    assert summary["seed"] == 0
    assert summary["arms"] == ["base", "geom", "geom+empty_brake"]
    assert summary["scenes"] == ["solo", "swap3"]
    runs = summary["runs"]
    assert len(runs) == 6
    by_key = {(row["scene"], row["arm"]): row for row in runs}
    assert set(by_key) == {
        ("solo", "base"),
        ("solo", "geom"),
        ("solo", "geom+empty_brake"),
        ("swap3", "base"),
        ("swap3", "geom"),
        ("swap3", "geom+empty_brake"),
    }
    seen_gif = set()
    seen_png = set()
    for row in runs:
        for key in REQUIRED:
            assert key in row, key
        assert isinstance(row["outcome"], str) and row["outcome"]
        assert isinstance(row["arrived"], int)
        assert isinstance(row["min_obstacle_clearance_m"], float)
        assert isinstance(row["min_ground_z_m"], float)
        if row["scene"] == "solo":
            assert row["min_inter_drone_distance_m"] is None
        else:
            assert isinstance(row["min_inter_drone_distance_m"], float)
            assert row["min_inter_drone_distance_m"] > 0
        files = row["files"]
        for kind in ("gif", "png", "frames"):
            path = out / files[kind]
            assert path.is_file(), path
            assert path.stat().st_size > 0, path
        if kind_mp4 := files.get("mp4"):
            mp4 = out / kind_mp4
            assert mp4.is_file() and mp4.stat().st_size > 0
        gif = out / files["gif"]
        png = out / files["png"]
        assert gif.read_bytes()[:3] == b"GIF"
        assert png.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
        seen_gif.add(gif)
        seen_png.add(png)
        frame = json.loads((out / files["frames"]).read_text(encoding="utf-8").splitlines()[0])
        drone = frame["drones"][0]
        assert {"p", "v", "plan_code", "min_surface_clearance_m"} <= set(drone)
        assert len(drone["p"]) == 3 and len(drone["v"]) == 3
        assert "min_inter_drone_distance_m" in frame
        assert "min_surface_clearance_m" in frame
    assert len(seen_gif) == 2
    assert len(seen_png) == 2
