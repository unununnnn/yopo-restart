"""票 E-C5c：地面盲区归因与密度 0.28 对照。只报数，不改默认。"""

import importlib.util
import json
import sys

import numpy as np
import pytest

from conftest import ROOT


def _script():
    path = ROOT / "scripts/e-c5c-blindspot-dense.py"
    name = "e_c5c_blindspot_dense"
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def _toy_card():
    from yopo_l0.scene import SceneCard, parse_geometry

    geom = {
        "boxes": [],
        "cylinders": [
            {"name": "obs0", "center_xy": [5.0, 0.0], "radius": 0.2, "z_lo": 0.0, "z_hi": 2.5},
        ],
    }
    raw = {
        "name": "toy",
        "camera": {
            "width": 160,
            "height": 96,
            "hfov_deg": 90.0,
            "vfov_deg": 60.0,
            "min_range_m": 0.2,
            "max_range_m": 20.0,
            "noise_std_m": 0.0,
            "mount_xyz_body": [0.13, 0.0, 0.02],
        },
        "criteria": {"body_radius_m": 0.35},
        "ground_z": 0.0,
        "geometry": geom,
    }
    return SceneCard("toy", raw, parse_geometry(geom, 0.0))


def _ground_case(mod):
    card = _toy_card()
    ticks = [{
        "uav": "uav1",
        "t": 4.56,
        "p": [0.0, 0.0, 0.349],
        "v": [1.0, 0.0, -0.4],
        "yaw": 0.2,
    }]
    plans = [{
        "t": 4.50,
        "plan_code": "OK",
        "geom_gate_active": True,
        "n_neighbors": 2,
        "ref_is_brake": False,
        "ref_min_z_m": 0.52,
        "ref_ground_clearance_m": 0.50,
        "ref_cylinder_clearance_m": 0.46,
        "ref_surface_clearance_m": 0.46,
        "depth_min_valid_m": 1.2,
        "depth_n_valid": 80,
        "ground_pixels": 4,
        "ground_pixel_stride": 2,
        "ec5_cylinder_name": "obs0",
        "ec5_cylinder_distance_m": 4.8,
        "ec5_cylinder_in_depth": False,
    }]
    diag = mod.diagnosis_from_run(card, ticks, plans, drone="uav1")
    return {
        "scene": "d014-swap3",
        "arm": "geom",
        "seed": 2,
        "drone": "uav1",
        "samples": diag["samples"],
    }


def test_script_does_not_import_in_flight_helpers():
    src = (ROOT / "scripts/e-c5c-blindspot-dense.py").read_text(encoding="utf-8")
    code = []
    skipping = False
    for line in src.splitlines():
        stripped = line.strip()
        if skipping:
            if stripped.endswith('"""') or stripped.endswith("'''"):
                skipping = False
            continue
        if stripped.startswith('"""') or stripped.startswith("'''"):
            if stripped.count('"""') >= 2 or stripped.count("'''") >= 2:
                continue
            skipping = True
            continue
        if stripped.startswith("#"):
            continue
        code.append(line)
    body = "\n".join(code)
    assert "importlib" not in body
    assert "e-c5-corridor-radius" not in body
    assert "e-c5b-track-clearance" not in body
    assert "yopo_l0.experiment" not in body


def test_five_arms_match_ec5b_and_do_not_touch_files():
    mod = _script()
    assert mod.ARMS == ("off", "base", "pred", "geom", "geom_multi")
    assert mod.SEEDS == tuple(range(10))
    assert mod.DENSE_ORDER == ("d028-swap3", "d028-solo")
    assert mod.DIAG_SEEDS == (2, 9)
    assert mod.DIAG_DRONE == "uav1"
    assert mod.decoder_radius_m("off") == pytest.approx(0.05)
    assert mod.decoder_radius_m("pred") == pytest.approx(0.13)
    assert mod.decoder_radius_m("geom") == pytest.approx(0.05)
    assert mod.arm_deviates("off") is False
    assert mod.arm_deviates("geom_multi") is True
    assert "偏离上游" not in mod.arm_label("off")
    assert "偏离上游" in mod.arm_label("base")
    assert "偏离上游" in mod.arm_label("pred")
    assert "偏离上游" in mod.arm_label("geom")
    assert "偏离上游" in mod.arm_label("geom_multi")
    paths = [ROOT / rel for rel in mod.SCENES.values()]
    before = {path: path.read_bytes() for path in paths}
    for scene in ("d028-swap3", "d028-solo", "d014-swap3"):
        off = mod.build_card(scene, "off")
        assert "policy_params" not in off.card
        base = mod.build_card(scene, "base")
        up = base.card["policy_params"]["upstream"]
        assert up["empty_brake"] is True
        assert up["inter_drone_filter"] is True
        assert up["inter_drone_threshold_m"] == pytest.approx(1.0)
        assert up["inter_drone_penalty"] == 0
        assert "track_clearance" not in up
        assert "safe_radius" not in up
        for arm in ("pred", "geom", "geom_multi"):
            card = mod.build_card(scene, arm)
            block = card.card["policy_params"]["upstream"]
            assert block["track_clearance"] is True
            assert block["track_clearance_mode"] == arm
            assert block["track_margin_m"] == pytest.approx(0.08)
            assert block["empty_brake"] is True
            assert "safe_radius" not in block
    assert {path: path.read_bytes() for path in paths} == before
    specs = mod.job_specs(list(mod.DENSE_ORDER), list(mod.ARMS), list(mod.SEEDS), diagnose=True)
    assert len(specs) == 104
    diag = [item for item in specs if item[3] is True]
    assert {(item[0], item[1], item[2]) for item in diag} == {
        ("d014-swap3", "geom", 2),
        ("d014-swap3", "geom", 9),
        ("d014-swap3", "geom_multi", 2),
        ("d014-swap3", "geom_multi", 9),
    }
    assert mod.main(["--check"]) == 0
    with pytest.raises(SystemExit):
        mod.main(["--jobs", "5", "--seeds", "0"])


def test_nearest_ground_low_trunk_and_collision_flags():
    mod = _script()
    card = _toy_card()
    query = mod.NearQuery(card.obstacles)
    ground = query.nearest(np.array([0.0, 0.0, 0.349]))
    assert ground["nearest_kind"] == "Ground"
    assert ground["nearest_distance_m"] == pytest.approx(0.349)
    assert query.cylinder_distances(np.array([0.0, 0.0, 0.349]))[0] > 0.35
    low = query.nearest(np.array([5.45, 0.0, 0.30]))
    assert low["nearest_raw_kind"] == "cylinder"
    assert low["nearest_kind"] == "低处树干"
    assert low["closest_z_m"] == pytest.approx(0.30)
    high = query.nearest(np.array([5.45, 0.0, 1.2]))
    assert high["nearest_kind"] == "圆柱"
    flags = mod.collision_flags(
        {"outcome": "COLLISION", "reason": "uav1:TRUTH_POSITION"},
        [{"uav": "uav1", "p": [0.0, 0.0, 0.349]}],
        query,
    )
    assert flags == {
        "pillar_collision": 0,
        "ground_collision": 1,
        "inter_drone_collision": 0,
        "timeout": 0,
    }
    both = mod.collision_flags(
        {"outcome": "COLLISION", "reason": "uav0:TRUTH_POSITION"},
        [{"uav": "uav0", "p": [5.45, 0.0, 0.30]}],
        query,
    )
    assert both["pillar_collision"] == 1 and both["ground_collision"] == 1
    inter = mod.collision_flags(
        {"outcome": "COLLISION", "reason": "uav1-uav2:INTER_DRONE"},
        [{"uav": "uav1", "p": [0.0, 0.0, 0.30]}],
        query,
    )
    assert inter["inter_drone_collision"] == 1
    assert inter["ground_collision"] == 0


def test_preimpact_samples_pitch_vfov_and_depth_fields():
    mod = _script()
    case = _ground_case(mod)
    impact = case["samples"][-1]
    assert impact["nearest_kind"] == "Ground"
    assert impact["ground_clearance_m"] == pytest.approx(0.349)
    assert impact["cylinder_clearance_m"] > 0.35
    assert impact["attitude_pitch_rad"] == pytest.approx(0.0)
    assert impact["camera_uses_pitch"] is False
    assert impact["flight_path_pitch_rad"] == pytest.approx(np.atan2(-0.4, 1.0))
    assert impact["closest_body_z_m"] == pytest.approx(-0.349)
    assert impact["nadir_in_vfov"] is False
    assert impact["closest_in_vfov"] is False
    assert impact["vfov_deg"] == pytest.approx(60.0)
    assert impact["lower_edge_elevation_deg"] == pytest.approx(-30.0)
    assert impact["ground_enters_horizontal_m"] == pytest.approx((0.349 + 0.02) / np.tan(np.deg2rad(30.0)))
    assert impact["depth_min_valid_m"] == pytest.approx(1.2)
    assert impact["ec5_cylinder_in_depth"] is False
    assert impact["plan_code"] == "OK"
    assert impact["ref_ground_clearance_m"] == pytest.approx(0.50)
    assert impact["plan_t"] == pytest.approx(4.50)
    info = mod.camera_attitude()
    assert info["camera_uses_pitch"] is False
    assert info["state_has_pitch"] is False
    assert info["capture_params"] == ["self", "position", "yaw", "stamp"]


def test_depth_image_counts_ground_and_not_the_far_cylinder():
    mod = _script()
    from yopo_l0.policy import R_yaw
    from yopo_l0.sensor import R_BODY_CAM, CameraConfig, DepthFrame, _ray_dirs_cam

    card = _toy_card()
    cam = CameraConfig.from_card(card.card["camera"])
    query = mod.NearQuery(card.obstacles)
    position = np.array([0.0, 0.0, 1.0])
    yaw = 0.0
    dirs = _ray_dirs_cam(cam)
    row, col = 95, 80
    assert (row * cam.width + col) % 2 == 0
    world = dirs[row, col] @ (R_yaw(yaw) @ R_BODY_CAM).T
    origin = mod.camera_origin(position, yaw, cam)
    assert world[2] < 0.0
    depth_m = (0.0 - origin[2]) / world[2]
    image = np.full((cam.height, cam.width), np.nan, dtype=np.float32)
    image[row, col] = np.float32(depth_m)
    frame = DepthFrame(0.0, image, cam.intrinsics)
    report = mod.analyze_depth(frame, position, yaw, cam, query)
    assert report["depth_n_valid"] == 1
    assert report["depth_min_valid_m"] == pytest.approx(depth_m, rel=1e-5)
    assert report["ground_pixels"] >= 1
    assert report["ec5_cylinder_in_depth"] is False
    assert report["ec5_cylinder_name"] == "obs0"


def test_gate_probe_checks_ground_without_a_separate_floor():
    mod = _script()
    probe = mod.geom_gate_sees_ground()
    assert probe["includes_ground"] is True
    assert probe["separate_z_min"] is False
    assert probe["low_with_ground_m"] == pytest.approx(0.40)
    assert probe["low_without_ground_m"] > 10.0
    assert probe["pass_clearance_m"] == pytest.approx(0.50)
    assert probe["low_rejected"] is True
    assert probe["pass_accepted"] is True


def test_explain_names_ground_vfov_and_not_a_missing_pitch():
    mod = _script()
    ground = _ground_case(mod)
    brake = {
        "scene": "d014-swap3",
        "arm": "geom_multi",
        "seed": 9,
        "drone": "uav1",
        "samples": [{
            **ground["samples"][-1],
            "plan_code": "CORRIDOR_EMPTY_HOVER",
            "ref_is_brake": True,
            "ref_ground_clearance_m": 0.31,
            "flight_path_pitch_rad": -0.3,
        }],
    }
    out = mod.explain_blindspot(
        [ground, brake],
        mod.camera_attitude(),
        mod.geom_gate_sees_ground(),
    )
    assert out["causes"]["attitude_pitch_insufficient"] is False
    assert out["causes"]["vfov_misses_body_nadir"] is True
    assert out["causes"]["geom_ignores_ground"] is False
    assert out["causes"]["geom_checks_ground"] is True
    assert out["causes"]["separate_z_min"] is False
    assert out["causes"]["depth_flag_targets_cylinder_not_image"] is True
    assert "Ground" in out["headline"]
    assert "不是俯仰不够" in out["text"]
    assert "VFOV" in out["text"]
    assert "z_min" in out["text"]
    assert "差在跟踪" in out["text"]
    assert "刹车轨迹不经过几何门" in out["text"]
    assert "几何门不检查这条刹车轨迹" in out["text"]
    trunk = {
        "arm": "geom",
        "seed": 2,
        "drone": "uav1",
        "scene": "d014-swap3",
        "samples": [{
            **ground["samples"][-1],
            "nearest_kind": "低处树干",
            "cylinder_clearance_m": 0.20,
        }],
    }
    other = mod.explain_blindspot([trunk], mod.camera_attitude(), mod.geom_gate_sees_ground())
    assert "低处树干" in other["headline"]
    assert other["causes"]["depth_flag_targets_cylinder_not_image"] is False


def test_resume_reads_partial_and_markdown_json_shape(tmp_path, monkeypatch):
    mod = _script()
    monkeypatch.setattr(mod, "PARTIAL", tmp_path)
    stored = {"scene": "d028-solo", "arm": "off", "seed": 3, "marker": 7}
    dest = mod._partial_path("d028-solo", "off", 3)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(stored), encoding="utf-8")

    def boom(*_a, **_k):
        raise AssertionError("断点还在，不应该重跑")

    monkeypatch.setattr(mod, "_execute", boom)
    assert mod.run_one("d028-solo", "off", 3, resume=True)["marker"] == 7

    def fresh(scene, arm, seed, *, diagnose=False):
        return {
            "scene": scene,
            "arm": arm,
            "seed": int(seed),
            "ran": True,
            "diagnosis": {"samples": []} if diagnose else None,
        }

    monkeypatch.setattr(mod, "_execute", fresh)
    made = mod.run_one("d028-swap3", "geom", 1, diagnose=False, resume=True)
    assert made["ran"] is True
    assert mod._partial_path("d028-swap3", "geom", 1).is_file()
    monkeypatch.setattr(mod, "_execute", boom)
    assert mod.run_one("d028-swap3", "geom", 1, resume=True)["ran"] is True

    ground = _ground_case(mod)
    rows = []
    for scene in mod.DENSE_ORDER:
        for arm in mod.ARMS:
            rows.append({
                "scene": scene,
                "arm": arm,
                "seed": 0,
                "upstream_deviation": arm != "off",
                "report_only": True,
                "frozen": False,
                "outcome": "SUCCESS",
                "reason": "GOAL_REACHED_AND_HELD",
                "arrived": 3 if scene.endswith("swap3") else 1,
                "n_drones": 3 if scene.endswith("swap3") else 1,
                "arrival_time_s": 12.5,
                "pillar_collision": 0,
                "ground_collision": 0,
                "inter_drone_collision": 0,
                "timeout": 0,
                "brake_plans": 0 if arm == "off" else 2,
                "corridor_empty_hover_plans": 0 if arm == "off" else 2,
                "min_clearance_m": 0.41,
                "min_cylinder_clearance_m": 0.48,
                "sim_duration_s": 16.0,
            })
    rows.append({
        "scene": "d014-swap3",
        "arm": "geom",
        "seed": 2,
        "outcome": "COLLISION",
        "reason": "uav1:TRUTH_POSITION",
        "arrived": 0,
        "n_drones": 3,
        "pillar_collision": 0,
        "ground_collision": 1,
        "min_clearance_m": 0.349,
        "min_cylinder_clearance_m": 0.45,
        "brake_plans": 4,
        "sim_duration_s": 4.56,
        "diagnosis": {"samples": ground["samples"]},
    })
    payload = mod.assemble_payload(
        rows,
        commit="abc",
        jobs=2,
        scenes=list(mod.DENSE_ORDER),
        arms=list(mod.ARMS),
        seeds=[0],
    )
    assert payload["schema"] == "e-c5c/1"
    assert payload["report_only"] is True
    assert payload["frozen"] is False
    assert "偏离上游" in payload["disclaimer"]
    assert "report-only" in payload["disclaimer"]
    assert len(payload["diagnosis"]) == 1
    assert len(payload["rows"]) == 10
    assert payload["answer"]["causes"]["geom_checks_ground"] is True
    md = mod.render_markdown(payload)
    assert "只报数" in md and "report-only" in md and "偏离上游" in md
    assert "CORRIDOR_EMPTY_HOVER 刹停" in md
    assert "不是俯仰不够" in md
    assert "VFOV" in md
    assert "z_min" in md
    assert "| 028 单机 | 默认全关 | 否 |" in md
    assert "028 单机 geom_multi 与 base 逐粒相同" in md
    blob = json.dumps(payload, ensure_ascii=False)
    assert '"report_only": true' in blob
    assert '"frozen": false' in blob


def test_saved_report_covers_the_full_grid():
    payload = json.loads((ROOT / "docs/research/e-c5c-blindspot-dense-runs.json").read_text(encoding="utf-8"))
    md = (ROOT / "docs/research/e-c5c-blindspot-dense.md").read_text(encoding="utf-8")
    assert payload["schema"] == "e-c5c/1"
    assert payload["report_only"] is True
    assert payload["frozen"] is False
    assert "偏离上游" in payload["disclaimer"]
    assert len(payload["rows"]) == 100
    assert len(payload["diagnosis"]) == 4
    assert {(row["scene"], row["arm"]) for row in payload["rows"]} == {
        (scene, arm)
        for scene in ("d028-swap3", "d028-solo")
        for arm in ("off", "base", "pred", "geom", "geom_multi")
    }
    causes = payload["answer"]["causes"]
    assert causes["attitude_pitch_insufficient"] is False
    assert causes["vfov_misses_body_nadir"] is True
    assert causes["geom_checks_ground"] is True
    assert causes["geom_ignores_ground"] is False
    assert causes["separate_z_min"] is False
    assert causes["depth_flag_targets_cylinder_not_image"] is True
    for needle in ("只报数", "report-only", "偏离上游", "不是俯仰不够", "VFOV", "z_min", "CORRIDOR_EMPTY_HOVER 刹停", "Ground"):
        assert needle in md
