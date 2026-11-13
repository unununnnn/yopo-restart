"""票 S4c：YOPO L0 运行时分层镜像。不启动 Docker / PX4。"""

from __future__ import annotations

import hashlib
import os
import re
import subprocess
import time
from pathlib import Path

from conftest import ROOT

SITL = ROOT / "scripts" / "sitl"

_DOCKER = r"""#!/bin/bash
printf '%s\n' "$*" >> "$DOCKER_LOG"
if [[ "$1" == "info" ]]; then
  exit 0
fi
if [[ "$1" == "image" && "$2" == "inspect" ]]; then
  name="$3"
  show_size=0
  if [[ "$name" == "-f" ]]; then
    show_size=1
    name="$5"
  fi
  if grep -qxF "$name" "$DOCKER_OK" || grep -q '^build ' "$DOCKER_LOG"; then
    if [[ "$show_size" == 1 ]]; then
      printf '1\n'
    fi
    exit 0
  fi
  exit 1
fi
exit 0
"""


def _sha12(*paths: Path) -> str:
    data = b"".join(path.read_bytes() for path in paths)
    return hashlib.sha256(data).hexdigest()[:12]


def _base_image() -> str:
    return f"yopo-restart-sitl:s1b-{_sha12(SITL / 'Dockerfile')}"


def _yopo_image() -> str:
    return f"yopo-restart-sitl:yopo-{_sha12(SITL / 'Dockerfile', SITL / 'Dockerfile.yopo')}"


def _install_docker(tmp_path: Path, present: list[str]) -> tuple[Path, Path]:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    script = bin_dir / "docker"
    script.write_text(_DOCKER)
    script.chmod(0o755)
    log = tmp_path / "docker.log"
    ok = tmp_path / "inspect-ok.txt"
    ok.write_text("".join(f"{name}\n" for name in present))
    log.write_text("")
    return bin_dir, log


def _env(tmp_path: Path, bin_dir: Path) -> dict[str, str]:
    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}:{env.get('PATH', '')}"
    env["HOME"] = str(tmp_path / "home")
    (tmp_path / "home").mkdir()
    env["YOPO_SITL_STATE"] = str(tmp_path / "state")
    env["DOCKER_LOG"] = str(tmp_path / "docker.log")
    env["DOCKER_OK"] = str(tmp_path / "inspect-ok.txt")
    for key in ("YOPO_SITL_IMAGE", "YOPO_SWARM_CACHE", "XDG_CACHE_HOME", "YOPO_SITL_N"):
        env.pop(key, None)
    return env


def _run(args: list[str], env: dict[str, str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, capture_output=True, text=True, env=env, check=False)


def _build_lines(log: Path) -> list[str]:
    return [line for line in log.read_text().splitlines() if line.startswith("build ")]


def test_dockerfile_yopo_layers_runtime_without_touching_base():
    text = (SITL / "Dockerfile.yopo").read_text()
    assert "ARG BASE_IMAGE=yopo-restart-sitl:s1b-placeholder" in text
    assert "FROM ${BASE_IMAGE}" in text
    assert "pip3 install --no-cache-dir" in text
    assert "--extra-index-url https://download.pytorch.org/whl/cpu" in text
    assert "torch==2.4.1+cpu" in text
    assert "numpy>=1.26,<2.3" in text
    assert "numpy>=1.26,<2" in text
    assert "scipy>=1.10,<1.16" in text
    assert "ruamel.yaml>=0.17" in text
    assert "matplotlib>=3.9" in text
    smoke = 'RUN python3 -c "import torch, numpy, scipy, ruamel.yaml; print(torch.__version__)"'
    assert smoke in text
    assert text.index("pip3 install") < text.index(smoke)
    assert "apt-get" not in text
    assert "\nCOPY " not in text and not text.startswith("COPY ")
    assert "git clone" not in text
    base = (SITL / "Dockerfile").read_text()
    assert "torch" not in base
    assert "Dockerfile.yopo" not in base
    committed = subprocess.check_output(["git", "show", "HEAD:scripts/sitl/Dockerfile"], cwd=ROOT)
    assert (SITL / "Dockerfile").read_bytes() == committed
    common = (SITL / "common.sh").read_text()
    assert 'SITL_IMAGE="${YOPO_SITL_IMAGE:-yopo-restart-sitl:s1b-$SITL_HASH}"' in common
    assert "Dockerfile.yopo" not in common
    for name in ("up.sh", "smoke.sh"):
        body = (SITL / name).read_text()
        assert "Dockerfile.yopo" not in body
        assert "build_yopo_image.sh" not in body


def test_sitl_shells_parse():
    scripts = sorted(SITL.glob("*.sh"))
    assert (SITL / "build_yopo_image.sh") in scripts
    for script in scripts:
        proc = subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True, check=False)
        assert proc.returncode == 0, proc.stderr


def test_print_image_is_combined_hash_without_build(tmp_path):
    bin_dir, log = _install_docker(tmp_path, [])
    env = _env(tmp_path, bin_dir)
    blocked = tmp_path / "not-a-directory"
    blocked.write_text("x")
    env["YOPO_SITL_STATE"] = str(blocked)
    proc = _run(["bash", str(SITL / "build_yopo_image.sh"), "--print-image"], env)
    assert proc.returncode == 0, proc.stderr
    assert proc.stdout.strip() == _yopo_image()
    assert _build_lines(log) == []


def test_status_reports_missing_ready_and_building(tmp_path):
    image = _yopo_image()
    bin_dir, _log = _install_docker(tmp_path, [])
    env = _env(tmp_path, bin_dir)
    missing = _run(["bash", str(SITL / "build_yopo_image.sh"), "--status"], env)
    assert missing.returncode == 0, missing.stderr
    assert missing.stdout.strip() == f"missing {image}"

    ok = tmp_path / "inspect-ok.txt"
    ok.write_text(image + "\n")
    ready = _run(["bash", str(SITL / "build_yopo_image.sh"), "--status"], env)
    assert ready.returncode == 0, ready.stderr
    assert ready.stdout.strip() == f"ready {image}"

    ok.write_text("")
    state = tmp_path / "state"
    state.mkdir(exist_ok=True)
    (state / "yopo-build.pid").write_text(str(os.getpid()) + "\n")
    building = _run(["bash", str(SITL / "build_yopo_image.sh"), "--status"], env)
    assert building.returncode == 0, building.stderr
    assert building.stdout.startswith(f"building pid={os.getpid()} log=")
    assert str(state / f"yopo-build-{image.rsplit('-', 1)[-1]}.log") in building.stdout


def test_foreground_build_refuses_missing_base_and_is_idempotent(tmp_path):
    bin_dir, log = _install_docker(tmp_path, [])
    env = _env(tmp_path, bin_dir)
    script = str(SITL / "build_yopo_image.sh")
    missing = _run(["bash", script], env)
    assert missing.returncode == 2
    assert "scripts/sitl/up.sh" in missing.stderr
    assert "不重编 PX4" in missing.stderr
    assert _build_lines(log) == []

    (tmp_path / "inspect-ok.txt").write_text(_base_image() + "\n")
    built = _run(["bash", script], env)
    assert built.returncode == 0, built.stderr + built.stdout
    lines = _build_lines(log)
    assert len(lines) == 1
    line = lines[0]
    assert f"BASE_IMAGE={_base_image()}" in line
    assert f"-t {_yopo_image()}" in line
    dockerfile = line.split("-f ", 1)[1].split()[0]
    assert dockerfile.endswith("Dockerfile.yopo")
    assert "build done" in built.stdout

    (tmp_path / "inspect-ok.txt").write_text(_yopo_image() + "\n")
    log.write_text("")
    again = _run(["bash", script], env)
    assert again.returncode == 0, again.stderr
    assert again.stdout.strip() == f"ready {_yopo_image()}"
    assert _build_lines(log) == []


def test_background_build_records_pid_without_implicit_base_rebuild(tmp_path):
    bin_dir, log = _install_docker(tmp_path, [])
    env = _env(tmp_path, bin_dir)
    script = str(SITL / "build_yopo_image.sh")
    refused = _run(["bash", script, "--bg"], env)
    assert refused.returncode == 2
    assert "scripts/sitl/up.sh" in refused.stderr
    assert not (tmp_path / "state" / "yopo-build.pid").exists()

    (tmp_path / "inspect-ok.txt").write_text(_base_image() + "\n")
    started = _run(["bash", script, "--bg"], env)
    assert started.returncode == 0, started.stderr
    match = re.search(r"pid=(\d+)", started.stdout)
    assert match is not None
    pid = int(match.group(1))
    pidfile = tmp_path / "state" / "yopo-build.pid"
    assert pidfile.read_text().strip() == str(pid)
    assert f"log={tmp_path / 'state' / ('yopo-build-' + _yopo_image().rsplit('-', 1)[-1] + '.log')}" in started.stdout
    deadline = time.time() + 10
    while time.time() < deadline:
        try:
            os.kill(pid, 0)
        except OSError:
            break
        time.sleep(0.05)
    else:
        os.kill(pid, 15)
        raise AssertionError("后台构建没有退出")
    assert _build_lines(log), log.read_text()
    assert f"BASE_IMAGE={_base_image()}" in _build_lines(log)[0]


def test_dry_run_stays_compatible_and_selects_layered_image(tmp_path):
    bin_dir, log = _install_docker(tmp_path, [])
    env = _env(tmp_path, bin_dir)
    script = str(SITL / "run_yopo_l0_sitl.sh")
    dry = _run(["bash", script, "--dry-run", "--n", "3", "--positioning", "rtk"], env)
    assert dry.returncode == 0, dry.stderr
    assert "planner=yopo-l0" in dry.stdout
    assert "scene=scenes/swap-3-open.json" in dry.stdout
    assert "n=3" in dry.stdout
    assert "positioning=rtk" in dry.stdout
    assert "seed=0" in dry.stdout
    assert f"image={_base_image()}" in dry.stdout
    one = _run(["bash", script, "--dry-run", "--n", "1"], env)
    assert one.returncode == 0, one.stderr
    assert "scene=scenes/g2-pillar.json" in one.stdout
    assert "positioning=default" in one.stdout
    bad = _run(["bash", script, "--dry-run", "--n", "4"], env)
    assert bad.returncode == 2
    assert _build_lines(log) == []

    (tmp_path / "inspect-ok.txt").write_text(_yopo_image() + "\n")
    layered = _run(["bash", script, "--dry-run", "--n", "1"], env)
    assert layered.returncode == 0, layered.stderr
    assert f"image={_yopo_image()}" in layered.stdout
    assert f"image={_base_image()}" not in layered.stdout

    env["YOPO_SITL_IMAGE"] = "explicit:tag"
    forced = _run(["bash", script, "--dry-run", "--n", "1"], env)
    assert forced.returncode == 0, forced.stderr
    assert "image=explicit:tag" in forced.stdout
    cli = _run(["bash", script, "--dry-run", "--n", "1", "--image", "cli:tag"], env)
    assert cli.returncode == 0, cli.stderr
    assert "image=cli:tag" in cli.stdout
    assert "image=explicit:tag" not in cli.stdout


def test_run_checks_tag_before_start_and_prefers_layer(tmp_path):
    bin_dir, log = _install_docker(tmp_path, [])
    env = _env(tmp_path, bin_dir)
    script = str(SITL / "run_yopo_l0_sitl.sh")
    missing = _run(["bash", script, "--n", "1"], env)
    assert missing.returncode == 2
    assert "scripts/sitl/up.sh" in missing.stderr
    assert not any(line.startswith("run ") for line in log.read_text().splitlines())

    env["YOPO_SITL_IMAGE"] = "missing:tag"
    explicit = _run(["bash", script, "--n", "1"], env)
    assert explicit.returncode == 2
    assert "missing:tag" in explicit.stderr
    assert "scripts/sitl/build_yopo_image.sh" in explicit.stderr
    env.pop("YOPO_SITL_IMAGE")

    (tmp_path / "inspect-ok.txt").write_text(_yopo_image() + "\n")
    log.write_text("")
    started = _run(["bash", script, "--n", "1", "--seed", "0"], env)
    assert started.returncode == 0, started.stderr + started.stdout
    assert "run_dir " in started.stdout
    run_lines = [line for line in log.read_text().splitlines() if line.startswith("run ")]
    assert len(run_lines) == 1
    assert _yopo_image() in run_lines[0]
    assert _base_image() not in run_lines[0]
    assert "in_container.sh" in run_lines[0]
    assert _build_lines(log) == []


def test_research_note_says_layer_is_offline():
    text = (ROOT / "docs/research/s4c-sitl-yopo-image.md").read_text()
    assert "report-only" in text
    assert "Dockerfile.yopo" in text
    assert "没有在云端实起 Docker" in text
    assert "PX4" in text
    readme = (SITL / "README.md").read_text()
    assert "s4c-sitl-yopo-image.md" in readme
    assert "build_yopo_image.sh" in readme
