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
    args = (out / "launch-args.txt").read_text().splitlines()
    assert "policy:=upstream" in args
    assert "shadow:=on" in args
    assert any(line.startswith("scene:=") and line.endswith("scene.json") for line in args)
    assert "start_enu:=0,0,1.5" in args
    assert "goal_enu:=5,0,1.5" in args
    blob = "\n".join(args)
    for forbidden in ("body_radius", "tracking_margin", "switch_delay", "start_pad", "MPC_", "param set"):
        assert forbidden not in blob
    assert run["launch"]["launch_path"] == "ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py"
    assert run["launch"]["world_schema"] == "sim/px4/g2-scene.json"


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


def test_launch_file_missing_policy_fails_closed(tmp_path):
    launch = tmp_path / "uavs.launch.py"
    launch.write_text("DeclareLaunchArgument('scene', default_value='')\nDeclareLaunchArgument('seed', default_value='0')\n")
    card = load_card(ROOT / "scenes/g2-pillar.json")
    with pytest.raises(L1LaunchError, match="policy"):
        export_l1(card, _cfg(), tmp_path / "out", launch_file=launch)


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
    text = (out / "launch-args.txt").read_text()
    assert "start_enu:=0,0,1.5;0,5,1.5;0,-5,1.5" in text
    assert "goal_enu:=6,0,1.5;6,5,1.5;6,-5,1.5" in text
    run = json.loads((out / "run.json").read_text())
    assert [d["namespace"] for d in run["drones"]] == ["px4_1", "px4_2", "px4_3"]
