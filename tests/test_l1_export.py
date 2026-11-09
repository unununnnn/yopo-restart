"""票 09：场景卡导出成 yopo-swarm SITL 能吃的目录。几何不对就拒绝。"""

import json
from pathlib import Path

import pytest

from yopo_l0.runner import RunConfig
from yopo_l0.scene import load_card

from yopo_l1.errors import L1GeometryError, L1LaunchError
from yopo_l1.export import export_l1

ROOT = Path(__file__).resolve().parents[1]


def _cfg(**kw):
    base = dict(seed=0, policy="upstream", shadow=True)
    base.update(kw)
    return RunConfig(**base)


def test_g2_pillar_exports_scene_schema_and_launch_args(tmp_path):
    card = load_card(ROOT / "scenes/g2-pillar.json")
    out = export_l1(card, _cfg(), tmp_path)
    scene = json.loads((out / "scene.json").read_text())
    original = json.loads((ROOT / "scenes/g2-scene.json").read_text())
    assert scene["world"] == "g2"
    assert scene["boxes"] == original["boxes"]
    assert scene["goal_enu"] == [5.0, 0.0, 1.5]
    assert set(scene) == {"world", "goal_enu", "boxes"}
    run = json.loads((out / "run.json").read_text())
    assert run["tier"] == "L1"
    assert run["scene_card_hash"] == card.hash
    assert run["policy"] == "upstream" and run["shadow"] is True
    assert run["drones"][0]["namespace"] == "px4_1"
    assert not (out / "launch-args.txt").exists()
    assert run["policy_requested"] == "upstream"
    assert run["policy_executed"] == "deployed-certified"
    assert run["drones"][0]["start_enu"] == [0.0, 0.0, 1.5]
    assert run["drones"][0]["goal_enu"] == [5.0, 0.0, 1.5]
    card_obj = json.loads((out / "job-card.json").read_text())
    assert card_obj["schema"] == "yopo-restart/l1-job-card/1"
    assert card_obj["mode"] == "nominal"
    assert card_obj["argv"] == ["nominal"]
    assert card_obj["driver"] == "scripts/run_g2.sh"
    assert card_obj["command"] == 'YOPO_SIM_SEED=0 bash "$YOPO_SWARM_ROOT/scripts/run_g2.sh" nominal'
    assert card_obj["env"] == {"YOPO_SIM_SEED": "0"}
    assert card_obj["policy_executed"] == "deployed-certified"
    assert card_obj["uavs_launch_py"]["used"] is False
    blob = card_obj["command"] + "\n" + json.dumps(card_obj["env"])
    for forbidden in ("body_radius", "tracking_margin", "switch_delay", "start_pad", "MPC_", "param set", "scene:=", "policy:="):
        assert forbidden not in blob
    assert run["launch"]["launch_path"] == "scripts/run_g2.sh"
    assert run["launch"]["world_schema"] == "sim/px4/g2-scene.json"
    assert run["launch"]["args"] == {}


def test_orin_launch_file_does_not_require_missing_args(tmp_path):
    launch = tmp_path / "uavs.launch.py"
    launch.write_text(
        "\n".join(
            f"DeclareLaunchArgument('{name}')"
            for name in ("vehicles", "ipc_root", "log_root", "body_radius", "tracking_margin", "code_sha")
        )
    )
    card = load_card(ROOT / "scenes/g2-pillar.json")
    out = export_l1(card, _cfg(), tmp_path / "out", launch_file=launch, backend="run_g2")
    job = json.loads((out / "job-card.json").read_text())
    assert job["uavs_launch_py"]["declared_in_file"] == [
        "vehicles",
        "ipc_root",
        "log_root",
        "body_radius",
        "tracking_margin",
        "code_sha",
    ]
    assert job["uavs_launch_py"]["used"] is False
    assert "scene" not in job["env"]
    assert not (out / "launch-args.txt").exists()


def test_launch_file_renames_args(tmp_path):
    launch = tmp_path / "uavs.launch.py"
    launch.write_text(
        "\n".join(
            [
                "DeclareLaunchArgument('scene_file', default_value='sim/px4/g2-scene.json')",
                "DeclareLaunchArgument('seed', default_value='0')",
                "DeclareLaunchArgument('policy', default_value='certified')",
                "DeclareLaunchArgument('shadow', default_value='off')",
                "DeclareLaunchArgument('start_enu', default_value='0,0,0')",
                "DeclareLaunchArgument('goal_enu', default_value='0,0,0')",
            ]
        )
    )
    card = load_card(ROOT / "scenes/g2-pillar.json")
    out = export_l1(card, _cfg(policy="certified", shadow=False), tmp_path / "out", launch_file=launch)
    keys = [line.split(":=", 1)[0] for line in (out / "launch-args.txt").read_text().splitlines()]
    assert keys[0] == "scene_file"
    assert "scene" not in keys
    assert "shadow:=off" in (out / "launch-args.txt").read_text()


def test_partial_launch_file_falls_back_to_run_g2(tmp_path):
    launch = tmp_path / "uavs.launch.py"
    launch.write_text("DeclareLaunchArgument('scene', default_value='')\nDeclareLaunchArgument('seed', default_value='0')\n")
    card = load_card(ROOT / "scenes/g2-pillar.json")
    out = export_l1(card, _cfg(), tmp_path / "out", launch_file=launch)
    assert (out / "job-card.json").is_file()
    assert not (out / "launch-args.txt").exists()


def test_ros2_backend_missing_policy_fails_closed(tmp_path):
    launch = tmp_path / "uavs.launch.py"
    launch.write_text("DeclareLaunchArgument('scene', default_value='')\nDeclareLaunchArgument('seed', default_value='0')\n")
    card = load_card(ROOT / "scenes/g2-pillar.json")
    with pytest.raises(L1LaunchError, match="policy"):
        export_l1(card, _cfg(), tmp_path / "out", launch_file=launch, backend="ros2")


def test_cylinders_are_refused():
    card = load_card(ROOT / "scenes/pillar-forest.json")
    with pytest.raises(L1GeometryError, match="圆柱"):
        export_l1(card, _cfg(), "/tmp/should-not-write")


def test_hover_policy_is_refused(tmp_path):
    card = load_card(ROOT / "scenes/g2-pillar.json")
    with pytest.raises(L1LaunchError, match="upstream 或 certified"):
        export_l1(card, _cfg(policy="hover"), tmp_path)


def test_safety_number_drift_is_refused(tmp_path, make_card):
    card = make_card(policy_params={"certified": {"body_radius": 0.35, "tracking_margin": 0.15, "switch_delay": 1.0, "start_pad_radius": 1.2}})
    with pytest.raises(L1GeometryError, match="switch_delay"):
        export_l1(card, _cfg(), tmp_path)


def test_nonzero_ground_and_extra_geometry_are_refused(tmp_path, make_card):
    with pytest.raises(L1GeometryError, match="地面"):
        export_l1(make_card(ground_z=1.0), _cfg(), tmp_path / "a")
    with pytest.raises(L1GeometryError, match="不认识字段"):
        export_l1(make_card(geometry={"world": "g2", "boxes": [], "spheres": []}), _cfg(), tmp_path / "b")


def test_narrow_corridor_boxes_export(tmp_path):
    card = load_card(ROOT / "scenes/narrow-corridor.json")
    out = export_l1(card, _cfg(policy="certified", shadow=False), tmp_path)
    scene = json.loads((out / "scene.json").read_text())
    assert [b["name"] for b in scene["boxes"]] == ["ceiling", "left_wall", "right_wall", "end_wall", "back_wall"]


def test_three_drones_join_poses(tmp_path):
    card = load_card(ROOT / "scenes/open-3drone.json")
    out = export_l1(card, _cfg(), tmp_path)
    run = json.loads((out / "run.json").read_text())
    assert [d["start_enu"] for d in run["drones"]] == [[0.0, 0.0, 1.5], [0.0, 5.0, 1.5], [0.0, -5.0, 1.5]]
    assert [d["goal_enu"] for d in run["drones"]] == [[6.0, 0.0, 1.5], [6.0, 5.0, 1.5], [6.0, -5.0, 1.5]]
    assert not (out / "launch-args.txt").exists()
    assert [d["namespace"] for d in run["drones"]] == ["px4_1", "px4_2", "px4_3"]
