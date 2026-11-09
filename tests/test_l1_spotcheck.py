"""票 09：抽检脚本只读参数，前后不一致就失败。不启动 PX4。"""

import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/l1_spotcheck.sh"


def _run(args, env=None, check=False):
    merged = os.environ.copy()
    merged.pop("YOPO_SWARM_ROOT", None)
    merged.pop("PX4_PARAM_DUMP_CMD", None)
    merged.pop("L1_LAUNCH_FILE", None)
    if env:
        merged.update(env)
    return subprocess.run(
        ["bash", str(SCRIPT), *args],
        cwd=ROOT,
        env=merged,
        text=True,
        capture_output=True,
        check=check,
    )


def test_script_mentions_each_writer_only_in_the_denylist():
    text = SCRIPT.read_text()
    for phrase in ("param set", "param save", "param load", "param reset", "param import"):
        assert text.count(phrase) == 1


def test_check_params_ignores_order_and_fails_on_value_change(tmp_path):
    before = tmp_path / "before.txt"
    after = tmp_path / "after.txt"
    before.write_text("MPC_Z_VEL_MAX [0, 8] : 3.0000\nNAV_DLL_ACT [0, 1] : 0\n")
    after.write_text("# comment\n\nNAV_DLL_ACT [0, 1] : 0\nMPC_Z_VEL_MAX [0, 8] : 3.0000\n")
    ok = _run(["--check-params", str(before), str(after)])
    assert ok.returncode == 0, ok.stderr
    after.write_text("MPC_Z_VEL_MAX [0, 8] : 4.0000\nNAV_DLL_ACT [0, 1] : 0\n")
    bad = _run(["--check-params", str(before), str(after)])
    assert bad.returncode == 3
    assert "MPC_Z_VEL_MAX" in bad.stderr
    assert "不一致" in bad.stderr


def test_dump_rejects_param_set_without_running_it(tmp_path):
    marker = tmp_path / "ran"
    dest = tmp_path / "params.txt"
    proc = _run(
        ["--dump-params", str(dest)],
        env={"PX4_PARAM_DUMP_CMD": f"param set MPC_XY_VEL_MAX 1; touch {marker}"},
    )
    assert proc.returncode == 2
    assert "param set" in proc.stderr
    assert not marker.exists()
    assert not dest.exists()


def test_prepare_only_exports_without_px4(tmp_path):
    proc = _run([
        "--prepare-only",
        "--scene", "scenes/g2-pillar.json",
        "--seed", "1",
        "--policy", "upstream",
        "--shadow", "on",
        "--out", str(tmp_path / "spot"),
    ])
    assert proc.returncode == 0, proc.stderr
    assert "ros2 launch yopo_swarm_px4 uavs.launch.py" in proc.stdout
    assert "policy:=upstream" in proc.stdout
    assert "shadow:=on" in proc.stdout
    assert "param set" not in proc.stdout
    assert (tmp_path / "spot/bundle/scene.json").is_file()
    assert not (tmp_path / "spot/log").exists()


def test_prepare_refuses_cylinders(tmp_path):
    proc = _run([
        "--prepare-only",
        "--scene", "scenes/pillar-forest.json",
        "--out", str(tmp_path / "spot"),
    ])
    assert proc.returncode != 0
    assert "圆柱" in proc.stderr


def test_full_run_refuses_to_start_without_swarm_root(tmp_path):
    proc = _run([
        "--scene", "scenes/g2-pillar.json",
        "--out", str(tmp_path / "spot"),
    ])
    assert proc.returncode == 1
    assert "YOPO_SWARM_ROOT" in proc.stderr
    assert (tmp_path / "spot/bundle/run.json").is_file()
    assert not (tmp_path / "spot/guard").exists()
