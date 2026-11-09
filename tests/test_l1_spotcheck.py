"""票 09b：抽检脚本只读参数，Orin 驱动是 run_g2.sh nominal。不启动 PX4，不装 torch。"""

import json
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


def test_script_does_not_install_a_host_venv_or_torch():
    text = SCRIPT.read_text()
    for phrase in ("make venv", "uv venv", "uv pip", "pip install", "ros2 launch"):
        assert phrase not in text


def test_prepare_only_writes_run_g2_job_card(tmp_path):
    proc = _run([
        "--prepare-only",
        "--scene", "scenes/g2-pillar.json",
        "--seed", "1",
        "--policy", "upstream",
        "--shadow", "on",
        "--out", str(tmp_path / "spot"),
    ])
    assert proc.returncode == 0, proc.stderr
    assert 'YOPO_SIM_SEED=1 bash "$YOPO_SWARM_ROOT/scripts/run_g2.sh" nominal' in proc.stdout
    assert "ros2 launch" not in proc.stdout
    assert "param set" not in proc.stdout
    assert (tmp_path / "spot/bundle/scene.json").is_file()
    card = json.loads((tmp_path / "spot/job-card.json").read_text())
    assert card["mode"] == "nominal"
    assert card["argv"] == ["nominal"]
    assert card["driver"] == "scripts/run_g2.sh"
    assert card["env"]["YOPO_SIM_SEED"] == "1"
    assert card["policy_requested"] == "upstream"
    assert card["policy_executed"] == "deployed-certified"
    assert card["timeout_s"] == 1800
    assert "pass" in card["pass_fail"] and "fail" in card["pass_fail"]
    assert card["uavs_launch_py"]["used"] is False
    assert not (tmp_path / "spot/log").exists()


def test_prepare_refuses_cylinders(tmp_path):
    proc = _run([
        "--prepare-only",
        "--scene", "scenes/pillar-forest.json",
        "--out", str(tmp_path / "spot"),
    ])
    assert proc.returncode != 0
    assert "圆柱" in proc.stderr


def _fake_swarm(root: Path, body: str) -> None:
    (root / "scripts").mkdir(parents=True)
    (root / "scripts" / "g2_run.py").write_text("# frozen\nBODY_RADIUS = 0.35\n")
    script = root / "scripts" / "run_g2.sh"
    script.write_text(body)
    script.chmod(0o755)
    launch = root / "ros2_ws/src/yopo_swarm_px4/launch/uavs.launch.py"
    launch.parent.mkdir(parents=True)
    launch.write_text(
        "\n".join(
            f"DeclareLaunchArgument('{name}')"
            for name in ("vehicles", "ipc_root", "log_root", "body_radius", "tracking_margin", "code_sha")
        )
        + "\n"
    )


def test_full_run_calls_run_g2_nominal_and_ingests_result(tmp_path):
    swarm = tmp_path / "swarm"
    _fake_swarm(
        swarm,
        """#!/bin/bash
set -euo pipefail
test "${1:-}" = nominal
test -n "${YOPO_SIM_SEED:-}"
mkdir -p sim/px4/last
printf '%s\\n' "$1" > sim/px4/last/mode.txt
printf '%s\\n' "$YOPO_SIM_SEED" > sim/px4/last/seed.txt
cat > sim/px4/last/result.json <<'EOF'
{"outcome":"SUCCESS","collisions":0,"tracking":{"error_m_max":0.04},"wall_time_s":11.0,"takeoff_handoff_s":2.5,"sim_duration_s":22.0}
EOF
printf '%s\\n' '{"kind":"event"}' > sim/px4/last/events.jsonl
""",
    )
    spot = tmp_path / "spot"
    proc = _run(
        ["--scene", "scenes/g2-pillar.json", "--seed", "0", "--policy", "upstream", "--out", str(spot)],
        env={
            "YOPO_SWARM_ROOT": str(swarm),
            "PX4_PARAM_DUMP_CMD": "printf '%s\\n' 'MPC_Z_VEL_MAX [0, 8] : 3.0000'",
        },
    )
    assert proc.returncode == 0, proc.stderr + proc.stdout
    assert (swarm / "sim/px4/last/mode.txt").read_text().strip() == "nominal"
    assert (swarm / "sim/px4/last/seed.txt").read_text().strip() == "0"
    assert "run_g2.sh" in (spot / "log/command.txt").read_text()
    assert "nominal" in (spot / "log/command.txt").read_text()
    result = json.loads((spot / "log/result.json").read_text())
    assert result["tier"] == "L1"
    assert result["policy"] == "deployed-certified"
    assert result["policy_requested"] == "upstream"
    assert result["outcome"] == "SUCCESS"
    assert result["collisions"] == 0
    assert result["tracking_error_m"] == 0.04
    assert result["wall_time_s"] == 11.0
    assert result["takeoff_handoff_s"] == 2.5
    assert result["rtf"] == 2.0
    assert result["shadow"] is False
    assert (spot / "log/orin-result.json").is_file()
    assert list((spot / "log/events").glob("*events.jsonl"))
    assert "ros2 launch" not in proc.stdout


def test_full_run_fails_when_safety_file_changes(tmp_path):
    swarm = tmp_path / "swarm"
    _fake_swarm(
        swarm,
        """#!/bin/bash
set -euo pipefail
printf '\\n# touched\\n' >> scripts/g2_run.py
mkdir -p sim/px4/last
echo '{"outcome":"SUCCESS","collisions":0}' > sim/px4/last/result.json
""",
    )
    proc = _run(
        ["--scene", "scenes/g2-pillar.json", "--seed", "0", "--out", str(tmp_path / "spot")],
        env={
            "YOPO_SWARM_ROOT": str(swarm),
            "PX4_PARAM_DUMP_CMD": "printf '%s\\n' 'MPC_Z_VEL_MAX [0, 8] : 3.0000'",
        },
    )
    assert proc.returncode == 3, proc.stderr
    assert "安全清单" in proc.stderr


def test_full_run_refuses_to_start_without_swarm_root(tmp_path):
    proc = _run([
        "--scene", "scenes/g2-pillar.json",
        "--out", str(tmp_path / "spot"),
    ])
    assert proc.returncode == 1
    assert "YOPO_SWARM_ROOT" in proc.stderr
    assert (tmp_path / "spot/bundle/run.json").is_file()
    assert not (tmp_path / "spot/guard").exists()
