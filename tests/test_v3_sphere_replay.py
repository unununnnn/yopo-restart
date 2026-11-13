"""票 V3：欧氏球回放只出图。缺开关时完整对比失败，--partial 仍写出结构和图。"""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

from conftest import ROOT

SCRIPT = ROOT / "scripts" / "v3-sphere-replay.py"
REQUIRED = (
    "status",
    "outcome",
    "arrived",
    "n_drones",
    "min_inter_drone_distance_m",
    "collision_pairs",
    "seed",
    "arm",
    "scene",
    "files",
)


def load_mod():
    spec = importlib.util.spec_from_file_location("v3_sphere_replay", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 {SCRIPT}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _ticks(arm_id: str) -> list[dict]:
    rows = []
    for k, t in enumerate((0.0, 0.5, 1.0, 1.5, 2.0)):
        gap = 1.2 if arm_id == "sphere" else max(0.4, 2.0 - t)
        positions = {"uav0": [0.0, 0.0, 1.5], "uav1": [gap, 0.0, 1.5], "uav2": [10.0, 10.0, 1.5]}
        for uav, p in positions.items():
            rows.append(
                {
                    "k": k,
                    "t": t,
                    "uav": uav,
                    "p": p,
                    "v": [0.0, 0.0, 0.0],
                    "plan_code": "OK",
                    "min_obstacle_m": 1.0,
                }
            )
    return rows


def _result(arm_id: str) -> dict:
    collided = arm_id != "sphere"
    return {
        "outcome": "COLLISION" if collided else "TIMEOUT",
        "reason": "uav0-uav1:INTER_DRONE" if collided else "GOAL_NOT_REACHED",
        "arrived": 0,
        "n_drones": 3,
        "sim_duration_s": 2.0,
        "min_inter_drone_distance_m": 0.4 if collided else 1.2,
    }


def _fake_run(mod):
    calls = []

    def run_sim(scene_path, arm, seed, duration, log_dir):
        calls.append((Path(scene_path).name, arm["id"], int(seed), duration))
        return _result(arm["id"]), _ticks(arm["id"]), 0.01

    mod.run_sim = run_sim
    return calls


def _summary_files(out: Path, row: dict) -> None:
    files = row["files"]
    for kind in ("gif", "png", "overview"):
        path = out / files[kind]
        assert path.is_file(), path
        assert path.stat().st_size > 0, path
    gif = out / files["gif"]
    png = out / files["png"]
    overview = out / files["overview"]
    assert gif.read_bytes()[:3] == b"GIF"
    assert png.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
    assert overview.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
    if files.get("mp4"):
        mp4 = out / files["mp4"]
        assert mp4.is_file() and mp4.stat().st_size > 0


def test_default_cases_are_the_ticket_seeds():
    mod = load_mod()
    cases = mod.default_cases()
    assert [(row["scene"], row["seed"]) for row in cases] == [
        ("swap-3-open", 0),
        ("ego-density-014-swap3", 0),
        ("ego-density-014-swap3", 1),
    ]
    for row in cases:
        assert (ROOT / row["path"]).is_file()


def test_build_card_does_not_rewrite_scene_files(monkeypatch):
    mod = load_mod()
    swap = ROOT / "scenes" / "swap-3-open.json"
    dense = ROOT / "scenes" / "ego-density-014-swap3.json"
    before = {path: path.read_bytes() for path in (swap, dense)}
    monkeypatch.setattr(mod, "peer_sphere_available", lambda: False)
    base = mod.build_card(swap, mod.BASE_ARM)
    backend = base.card["policy_params"]["backend"]
    assert backend["enabled"] is True
    assert "peer_sphere_enabled" not in backend
    assert "swarm_penalty" not in backend
    with pytest.raises(mod.SphereSwitchMissing):
        mod.build_card(dense, mod.SPHERE_ARM)

    monkeypatch.setattr(mod, "peer_sphere_available", lambda: True)
    base_on = mod.build_card(swap, mod.BASE_ARM)
    sphere = mod.build_card(dense, mod.SPHERE_ARM)
    assert base_on.card["policy_params"]["backend"]["peer_sphere_enabled"] is False
    turned = sphere.card["policy_params"]["backend"]
    assert turned["enabled"] is True
    assert turned["peer_sphere_enabled"] is True
    assert turned["peer_sphere_radius_m"] == pytest.approx(0.85)
    assert turned["peer_sphere_weight"] == pytest.approx(8.0)
    assert "swarm_penalty" not in turned
    assert before == {path: path.read_bytes() for path in (swap, dense)}


def test_analyze_inter_reports_the_collision_pair():
    mod = load_mod()
    times, uavs, tracks = mod.align_ticks(_ticks("base"))
    report = mod.analyze_inter(times, uavs, tracks, "uav1-uav0:INTER_DRONE", 0.70)
    assert report["min_inter_drone_distance_m"] == pytest.approx(0.4)
    assert report["collision_pairs"][0]["pair"] == ["uav0", "uav1"]
    assert report["collision_pairs"][0]["min_distance_m"] == pytest.approx(0.4)
    assert len(report["collision_pairs"]) == 1
    far = mod.analyze_inter(*mod.align_ticks(_ticks("sphere")), "GOAL_NOT_REACHED", 0.70)
    assert far["collision_pairs"] == []
    assert far["min_inter_drone_distance_m"] == pytest.approx(1.2)
    assert mod.pair_from_reason("uav1-uav2:INTER_DRONE") == ["uav1", "uav2"]
    assert mod.pair_from_reason("GOAL_NOT_REACHED") is None


def test_full_compare_errors_when_switch_is_missing(tmp_path, monkeypatch, capsys):
    mod = load_mod()
    monkeypatch.setattr(mod, "peer_sphere_available", lambda: False)
    calls = _fake_run(mod)
    rc = mod.main(["--out", str(tmp_path / "viz")])
    assert rc == 2
    assert calls == []
    err = capsys.readouterr().err
    assert "peer_sphere_enabled" in err
    assert "94" in err
    assert not (tmp_path / "viz" / "summary.json").exists()


def test_partial_marks_missing_sphere_and_writes_plots(tmp_path, monkeypatch):
    mod = load_mod()
    monkeypatch.setattr(mod, "peer_sphere_available", lambda: False)
    calls = _fake_run(mod)
    swap = ROOT / "scenes" / "swap-3-open.json"
    before = swap.read_bytes()
    out = tmp_path / "viz"
    rc = mod.main(["--partial", "--out", str(out), "--cases", "swap-3-open:0", "--fps", "4"])
    assert rc == 0, "partial 应该在缺开关时成功"
    assert calls == [("swap-3-open.json", "base", 0, 2.0)]
    assert swap.read_bytes() == before
    summary = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    assert summary["ticket"] == "V3"
    assert summary["report_only"] is True
    assert summary["viz_only"] is True
    assert summary["frozen"] is False
    assert summary["partial"] is True
    assert summary["peer_sphere_available"] is False
    assert summary["peer_sphere_radius_m"] == pytest.approx(0.85)
    assert summary["collision_m"] == pytest.approx(0.70)
    assert summary["arms"] == ["base", "sphere"]
    assert summary["cases"] == [
        {
            "scene": "swap-3-open",
            "scene_path": "scenes/swap-3-open.json",
            "seed": 0,
            "title": "开阔三机对向",
        }
    ]
    assert "missing" in summary["note"]
    by_arm = {row["arm"]: row for row in summary["runs"]}
    assert set(by_arm) == {"base", "sphere"}
    base = by_arm["base"]
    for key in REQUIRED:
        assert key in base
    assert base["status"] == "ok"
    assert base["outcome"] == "COLLISION"
    assert base["arrived"] == 0
    assert base["n_drones"] == 3
    assert base["min_inter_drone_distance_m"] == pytest.approx(0.4)
    assert base["collision_pairs"][0]["pair"] == ["uav0", "uav1"]
    assert base["peer_sphere_enabled"] is False
    assert base["backend_enabled"] is True
    _summary_files(out, base)
    sphere = by_arm["sphere"]
    for key in REQUIRED:
        assert key in sphere
    assert sphere["status"] == "missing"
    assert sphere["outcome"] is None
    assert sphere["arrived"] is None
    assert sphere["min_inter_drone_distance_m"] is None
    assert sphere["collision_pairs"] is None
    assert sphere["peer_sphere_enabled"] is None
    assert "peer_sphere_enabled" in sphere["missing"]
    _summary_files(out, sphere)


def test_partial_with_switch_runs_both_arms(tmp_path, monkeypatch):
    mod = load_mod()
    monkeypatch.setattr(mod, "peer_sphere_available", lambda: True)
    monkeypatch.setattr(mod, "install_peer_delivery", lambda: True)
    calls = _fake_run(mod)
    out = tmp_path / "viz"
    rc = mod.main(
        ["--partial", "--out", str(out), "--cases", "ego-density-014-swap3:1", "--fps", "4", "--duration", "1.5"]
    )
    assert rc == 0
    assert calls == [
        ("ego-density-014-swap3.json", "base", 1, 1.5),
        ("ego-density-014-swap3.json", "sphere", 1, 1.5),
    ]
    summary = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    assert summary["partial"] is True
    assert summary["duration_s"] == pytest.approx(1.5)
    assert summary["peer_sphere_available"] is True
    assert summary["note"] is None
    by_arm = {row["arm"]: row for row in summary["runs"]}
    assert by_arm["base"]["status"] == "ok"
    assert by_arm["base"]["collision_pairs"][0]["pair"] == ["uav0", "uav1"]
    assert by_arm["sphere"]["status"] == "ok"
    assert by_arm["sphere"]["peer_sphere_enabled"] is True
    assert by_arm["sphere"]["peer_sphere_radius_m"] == pytest.approx(0.85)
    assert by_arm["sphere"]["collision_pairs"] == []
    assert by_arm["sphere"]["min_inter_drone_distance_m"] == pytest.approx(1.2)
    assert by_arm["sphere"]["outcome"] == "TIMEOUT"
    assert by_arm["sphere"]["seed"] == 1
    _summary_files(out, by_arm["sphere"])


def test_plot_interface_writes_gif_and_png(tmp_path):
    mod = load_mod()
    case = mod.default_cases()[0]
    scene = mod.scene_file(case)
    base = mod.clip_from_run(case, mod.BASE_ARM, scene, _result("base"), _ticks("base"), tmp_path / "logs" / "base", 0.01)
    sphere = mod.missing_clip(case, mod.SPHERE_ARM, scene)
    png = tmp_path / "still.png"
    gif = tmp_path / "clip.gif"
    overview = tmp_path / "overview.png"
    mod.render_still([base, sphere], png)
    mod.render_overview([[base, sphere]], overview)
    _gif, mp4, play_s = mod.render_animation([base, sphere], gif, tmp_path / "clip.mp4", fps=4, max_seconds=15)
    assert png.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
    assert overview.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
    assert gif.read_bytes()[:3] == b"GIF"
    assert play_s <= 15
    assert _gif == gif
    if mp4 is not None:
        assert mp4.is_file() and mp4.stat().st_size > 0
    assert base.collision_pairs[0]["pair"] == ["uav0", "uav1"]
    assert sphere.status == "missing"
    assert sphere.collision_pairs is None
