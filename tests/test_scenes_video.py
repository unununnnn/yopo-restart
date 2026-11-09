"""票 04：场景卡能加载，结果表带哈希/层级/策略/影子盾，日志能渲成 ≤60s 视频。"""

import hashlib
import json
import subprocess
import sys

import numpy as np
import pytest

from yopo_l0.batch import main as batch_main
from yopo_l0.render import render_video, select_frame_indices
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import Cylinder, Ground, load_card

from conftest import ROOT

G2_SCENE_SHA256 = "b820e3b1e6989c10384a4dee6aad7d49a9ef9d87a1ced4580b79916581d8818e"
NEW_CARDS = (
    "scenes/pillar-forest.json",
    "scenes/narrow-corridor.json",
)


def suite_specs():
    lines = []
    for ln in (ROOT / "scenes/suite.txt").read_text().splitlines():
        ln = ln.split("#", 1)[0].strip()
        if ln:
            lines.append(ln)
    return lines


def write_ticks(path, n, dt, n_uav=1):
    with path.open("w") as f:
        for k in range(n):
            for i in range(n_uav):
                row = {
                    "k": k, "t": round(k * dt, 9), "uav": f"uav{i}",
                    "p": [0.01 * k, 0.2 * i, 1.5], "v": [0, 0, 0], "a": [0, 0, 0],
                    "frame_stamp": None, "min_obstacle_m": 1.0,
                    "goal_error_m": 0.0, "yaw": 0.0, "plan_code": "HOVER",
                }
                f.write(json.dumps(row) + "\n")


def ffprobe(path):
    p = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=codec_name,width,height",
         "-show_entries", "format=duration", "-of", "json", str(path)],
        capture_output=True, text=True, check=True,
    )
    meta = json.loads(p.stdout)
    return meta["streams"][0], float(meta["format"]["duration"])


def test_g2_scene_bytes_unchanged_and_pillar_card_reuses_it():
    digest = hashlib.sha256((ROOT / "scenes/g2-scene.json").read_bytes()).hexdigest()
    assert digest == G2_SCENE_SHA256
    pillar = json.loads((ROOT / "scenes/g2-pillar.json").read_text())
    assert pillar["geometry"] == "g2-scene.json"
    card = load_card(ROOT / "scenes/g2-pillar.json")
    assert card.name == "g2-pillar"
    assert any(getattr(o, "name", "") == "pillar" for o in card.obstacles)


def test_suite_includes_g2_pillar_upstream_and_new_hover_cards():
    specs = suite_specs()
    assert specs == [
        "scenes/g2-hover.json@hover",
        "scenes/g2-hover-lift.json@hover",
        "scenes/g2-pillar.json@upstream",
        "scenes/pillar-forest.json@hover",
        "scenes/narrow-corridor.json@hover",
        "scenes/open-3drone.json@hover",
        "scenes/formation-hold.json@formation",
        "scenes/crossing-pillars.json@crossing",
        "scenes/crossing-forest.json@crossing@0",
        "scenes/formation-switch.json@switch",
    ]
    for spec in specs:
        load_card(ROOT / spec.split("@", 1)[0])


def test_new_cards_match_hover_schema_and_clear_the_body():
    hover = json.loads((ROOT / "scenes/g2-hover.json").read_text())
    hashes = []
    for rel in NEW_CARDS:
        raw = json.loads((ROOT / rel).read_text())
        assert set(raw) == set(hover)
        assert raw["camera"] == hover["camera"]
        assert raw["dynamics"] == hover["dynamics"]
        assert raw["rates_hz"] == hover["rates_hz"]
        assert raw["criteria"] == hover["criteria"]
        assert raw["task"]["type"] == "hover"
        assert raw["geometry"] != "g2-scene.json"
        card = load_card(ROOT / rel)
        again = load_card(ROOT / rel)
        assert card.hash == again.hash and len(card.hash) == 64
        hashes.append(card.hash)
        start = np.asarray(card.card["drones"][0]["start_enu"], dtype=float)
        assert card.min_distance(start) > card.card["criteria"]["body_radius_m"]
    assert len(set(hashes)) == len(NEW_CARDS)


def test_pillar_forest_cylinders_match_g2_pillar_height(g2_geometry):
    card = load_card(ROOT / "scenes/pillar-forest.json")
    cyls = [o for o in card.obstacles if isinstance(o, Cylinder)]
    assert len(cyls) == 12
    g2_top = next(b for b in g2_geometry["boxes"] if b["name"] == "pillar")["hi"][2]
    assert all(c.radius == 0.3 and c.z_hi == g2_top for c in cyls)
    start = np.array([0.0, 0.0, 1.5])
    assert min(c.distance(start) for c in cyls) > 2.0


def test_narrow_corridor_is_narrower_than_g2_and_still_passable(g2_geometry):
    g2_left = next(b for b in g2_geometry["boxes"] if b["name"] == "left_wall")
    g2_right = next(b for b in g2_geometry["boxes"] if b["name"] == "right_wall")
    g2_gap = g2_left["lo"][1] - g2_right["hi"][1]
    geom = json.loads((ROOT / "scenes/narrow-corridor-scene.json").read_text())
    left = next(b for b in geom["boxes"] if b["name"] == "left_wall")
    right = next(b for b in geom["boxes"] if b["name"] == "right_wall")
    gap = left["lo"][1] - right["hi"][1]
    assert gap == pytest.approx(1.6)
    assert 0.7 < gap < g2_gap


def test_open_area_is_ticket05_empty_world():
    # 票 05 的开阔区是空几何；三机卡引用它，不另建一份。
    geom = json.loads((ROOT / "scenes/open-area.json").read_text())
    assert geom["boxes"] == [] and geom["cylinders"] == []
    raw = json.loads((ROOT / "scenes/open-3drone.json").read_text())
    assert raw["geometry"] == "open-area.json" and len(raw["drones"]) == 3
    card = load_card(ROOT / "scenes/open-3drone.json")
    assert all(isinstance(o, Ground) for o in card.obstacles)
    for drone in card.card["drones"]:
        start = np.asarray(drone["start_enu"], dtype=float)
        assert card.min_distance(start) > card.card["criteria"]["body_radius_m"]
    out = run(card, RunConfig(seed=0, policy="hover", shadow=False, duration_s=0.2))
    assert out.result["n_drones"] == 3
    assert out.result["outcome"] != "COLLISION"
    assert out.result["min_inter_drone_distance_m"] > 2 * 0.35
    assert out.result["shadow_reject_rate"] is None


@pytest.mark.parametrize("rel", NEW_CARDS)
def test_new_card_hover_seam(rel):
    card = load_card(ROOT / rel)
    out = run(card, RunConfig(seed=0, policy="hover", shadow=False, duration_s=0.2))
    assert out.result["outcome"] == "SUCCESS"
    assert out.result["scene_card_hash"] == card.hash
    assert out.result["tier"] == "L0"
    assert out.result["policy"] == "hover"
    assert out.result["shadow"] is False
    assert out.result["shadow_reject_rate"] is None
    assert out.ticks and "p" in out.ticks[0]


def test_batch_table_lists_hash_seed_tier_policy_shadow(tmp_path):
    rc = batch_main([
        "scenes/narrow-corridor.json@hover",
        "--seeds", "0",
        "--duration", "0.2",
        "--no-determinism",
        "--no-shadow",
        "--out", str(tmp_path),
    ])
    assert rc == 0
    card = load_card(ROOT / "scenes/narrow-corridor.json")
    md = (tmp_path / "summary.md").read_text()
    assert card.hash in md
    assert "层级 L0" in md
    assert "@ hover" in md
    assert "影子盾 关" in md
    assert "最小机间距离" in md
    assert "| 0 | L0 | hover | 关 |" in md
    assert "—" in md  # 单机机间距离与影子拒绝率占位
    saved = json.loads((tmp_path / "results.json").read_text())
    assert saved[0]["scene_card_hash"] == card.hash
    assert saved[0]["seed"] == 0 and saved[0]["tier"] == "L0"
    assert saved[0]["policy"] == "hover" and saved[0]["shadow"] is False
    assert saved[0]["min_inter_drone_distance_m"] is None
    assert (tmp_path / "narrow-corridor@hover" / "seed0" / "ticks.jsonl").is_file()


def test_select_frame_indices_caps_playback_at_60s_and_keeps_short_logs_realtime():
    long = select_frame_indices(20_000, 200.0, fps=10, max_seconds=60)
    assert len(long) / 10 <= 60
    assert long[0] == 0 and long[-1] == 19_999
    short = select_frame_indices(1000, 10.0, fps=10, max_seconds=60)
    assert len(short) / 10 == pytest.approx(10, abs=0.15)
    with pytest.raises(ValueError):
        select_frame_indices(10, 1.0, fps=10, max_seconds=61)


def test_render_top_and_side_under_60s(tmp_path):
    log = tmp_path / "ticks.jsonl"
    write_ticks(log, 50, 0.1, n_uav=2)  # 4.9 s 仿真，两机
    info = render_video(
        log, tmp_path / "preview.mp4",
        scene=ROOT / "scenes/narrow-corridor.json", fps=10, max_seconds=1,
    )
    assert info.duration_s <= 1
    assert info.path.is_file() and info.path.stat().st_size > 500
    stream, duration = ffprobe(info.path)
    assert stream["codec_name"] == "h264"
    assert int(stream["width"]) % 2 == 0 and int(stream["height"]) % 2 == 0
    assert duration <= 1.25
    png = tmp_path / "frame.png"
    subprocess.run(
        ["ffmpeg", "-y", "-i", str(info.path), "-frames:v", "1", str(png)],
        check=True, capture_output=True,
    )
    import matplotlib.pyplot as plt
    img = plt.imread(png)
    half = img.shape[0] // 2
    assert img[:half].std() > 0.005
    assert img[half:].std() > 0.005


def test_render_cli(tmp_path):
    log = tmp_path / "ticks.jsonl"
    write_ticks(log, 5, 0.1)
    out = tmp_path / "preview.mp4"
    ok = subprocess.run(
        [sys.executable, "-m", "yopo_l0.render", "--log", str(log), "--out", str(out), "--max-seconds", "60"],
        cwd=ROOT, capture_output=True, text=True,
    )
    assert ok.returncode == 0, ok.stderr
    assert out.is_file()
    bad = subprocess.run(
        [sys.executable, "-m", "yopo_l0.render", "--log", str(log), "--out", str(tmp_path / "x.mp4"),
         "--max-seconds", "61"],
        cwd=ROOT, capture_output=True, text=True,
    )
    assert bad.returncode != 0
    empty = tmp_path / "empty.jsonl"
    empty.write_text("")
    nope = subprocess.run(
        [sys.executable, "-m", "yopo_l0.render", "--log", str(empty), "--out", str(tmp_path / "y.mp4")],
        cwd=ROOT, capture_output=True, text=True,
    )
    assert nope.returncode != 0
