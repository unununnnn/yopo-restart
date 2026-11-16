"""票 SITL-NET：SITL 缺省走隔离 docker 网络。不启动真实 Docker / PX4。"""

from __future__ import annotations

import hashlib
import os
import re
import subprocess
from pathlib import Path

from conftest import ROOT

SITL = ROOT / "scripts" / "sitl"
NET_RE = re.compile(r"^yopo-sitl-[0-9]{8}T[0-9]{6}-[0-9]+-[0-9]+$")

_DOCKER = r"""#!/bin/bash
printf '%s\n' "$*" >> "$DOCKER_LOG"
if [[ "$1" == "info" ]]; then
  exit 0
fi
if [[ "$1" == "image" && "$2" == "inspect" ]]; then
  name="$3"
  if [[ "$name" == "-f" ]]; then
    name="$5"
  fi
  if grep -qxF "$name" "$DOCKER_OK" || grep -q '^build ' "$DOCKER_LOG"; then
    exit 0
  fi
  exit 1
fi
if [[ "$1" == "network" && "$2" == "create" ]]; then
  name="${*: -1}"
  printf '%s\n' "$name" >> "$DOCKER_NETS"
  if [[ "${DOCKER_FAIL_NET_CREATE:-}" == 1 ]]; then
    exit 1
  fi
  exit 0
fi
if [[ "$1" == "network" && "$2" == "rm" ]]; then
  name="${*: -1}"
  printf '%s\n' "$name" >> "$DOCKER_NETS_RM"
  if [[ -f "$DOCKER_NETS" ]]; then
    grep -vxF "$name" "$DOCKER_NETS" > "$DOCKER_NETS.tmp" || true
    mv "$DOCKER_NETS.tmp" "$DOCKER_NETS"
  fi
  exit 0
fi
if [[ "$1" == "run" && "${DOCKER_FAIL_RUN:-}" == 1 && "$*" == *in_container.sh* ]]; then
  exit 7
fi
exit 0
"""


def _sha12(*paths: Path) -> str:
    data = b"".join(path.read_bytes() for path in paths)
    return hashlib.sha256(data).hexdigest()[:12]


def _yopo_image() -> str:
    return f"yopo-restart-sitl:yopo-{_sha12(SITL / 'Dockerfile', SITL / 'Dockerfile.yopo')}"


def _install_docker(tmp_path: Path) -> tuple[Path, Path]:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    script = bin_dir / "docker"
    script.write_text(_DOCKER)
    script.chmod(0o755)
    log = tmp_path / "docker.log"
    ok = tmp_path / "inspect-ok.txt"
    ok.write_text(_yopo_image() + "\n")
    log.write_text("")
    (tmp_path / "nets.txt").write_text("")
    (tmp_path / "nets-rm.txt").write_text("")
    return bin_dir, log


def _env(tmp_path: Path, bin_dir: Path) -> dict[str, str]:
    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}:{env.get('PATH', '')}"
    env["HOME"] = str(tmp_path / "home")
    (tmp_path / "home").mkdir()
    env["YOPO_SITL_STATE"] = str(tmp_path / "state")
    env["DOCKER_LOG"] = str(tmp_path / "docker.log")
    env["DOCKER_OK"] = str(tmp_path / "inspect-ok.txt")
    env["DOCKER_NETS"] = str(tmp_path / "nets.txt")
    env["DOCKER_NETS_RM"] = str(tmp_path / "nets-rm.txt")
    for key in (
        "YOPO_SITL_IMAGE",
        "YOPO_SWARM_CACHE",
        "XDG_CACHE_HOME",
        "YOPO_SITL_N",
        "YOPO_SITL_CONTAINER_NAME",
        "YOPO_SITL_DOCKER_NETWORK",
        "DOCKER_FAIL_NET_CREATE",
        "DOCKER_FAIL_RUN",
    ):
        env.pop(key, None)
    return env


def _run(args: list[str], env: dict[str, str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, capture_output=True, text=True, env=env, check=False, cwd=ROOT)


def _lines(path: Path, prefix: str = "") -> list[str]:
    rows = path.read_text().splitlines() if path.exists() else []
    if prefix:
        return [row for row in rows if row.startswith(prefix)]
    return [row for row in rows if row]


def _mission_line(log: Path) -> str:
    rows = [row for row in _lines(log, "run ") if "in_container.sh" in row]
    assert len(rows) == 1, log.read_text()
    return rows[0]


def test_custom_container_name_without_network_exits_before_touching_docker(tmp_path):
    bin_dir, log = _install_docker(tmp_path)
    env = _env(tmp_path, bin_dir)
    env["YOPO_SITL_CONTAINER_NAME"] = "yopo-restart-b3-sitl-n3"
    proc = _run(["bash", str(SITL / "run_yopo_l0_sitl.sh"), "--n", "3"], env)
    assert proc.returncode == 2, proc.stderr + proc.stdout
    assert "YOPO_SITL_CONTAINER_NAME 要和 YOPO_SITL_DOCKER_NETWORK 一起给" in proc.stderr
    # common.sh 在 source 时会调 docker info 决定要不要 sudo；除此之外不应碰容器或网络。
    assert all(row == "info" for row in _lines(log)), log.read_text()
    assert _lines(tmp_path / "nets.txt") == []
    assert _lines(tmp_path / "nets-rm.txt") == []
    assert not (tmp_path / "state").exists() or list((tmp_path / "state").glob("**/*")) == []


def test_default_run_creates_and_removes_isolated_network(tmp_path):
    bin_dir, log = _install_docker(tmp_path)
    env = _env(tmp_path, bin_dir)
    proc = _run(["bash", str(SITL / "run_yopo_l0_sitl.sh"), "--n", "1"], env)
    assert proc.returncode == 0, proc.stderr + proc.stdout
    create_rows = [row for row in _lines(log) if row.startswith("network create ")]
    assert len(create_rows) == 1
    created = create_rows[0].rsplit(" ", 1)[-1]
    removed = _lines(tmp_path / "nets-rm.txt")
    assert removed == [created]
    assert _lines(tmp_path / "nets.txt") == []
    assert NET_RE.match(created), created
    mission = _mission_line(log)
    assert f"--network {created}" in mission
    assert "run_dir " in proc.stdout
    assert f"sitl_network {created} owned=1" in proc.stdout
    run_dir = Path(proc.stdout.split("run_dir ", 1)[1].split()[0])
    meta = (run_dir / "docker_network.txt").read_text()
    assert f"network={created}" in meta and "owned=1" in meta
    assert create_rows == [f"network create --driver bridge {created}"]


def test_user_network_is_used_and_not_removed(tmp_path):
    bin_dir, log = _install_docker(tmp_path)
    env = _env(tmp_path, bin_dir)
    env["YOPO_SITL_CONTAINER_NAME"] = "yopo-restart-sitl-sh2b"
    env["YOPO_SITL_DOCKER_NETWORK"] = "yopo-sh2b-sitl"
    proc = _run(["bash", str(SITL / "run_yopo_l0_sitl.sh"), "--n", "3"], env)
    assert proc.returncode == 0, proc.stderr + proc.stdout
    assert _lines(tmp_path / "nets.txt") == []
    assert _lines(tmp_path / "nets-rm.txt") == []
    mission = _mission_line(log)
    assert "--network yopo-sh2b-sitl" in mission
    assert "--name yopo-restart-sitl-sh2b" in mission
    assert "sitl_network yopo-sh2b-sitl owned=0" in proc.stdout
    assert not any(row.startswith("network create ") or row.startswith("network rm ") for row in _lines(log))
    run_dir = Path(proc.stdout.split("run_dir ", 1)[1].split()[0])
    meta = (run_dir / "docker_network.txt").read_text()
    assert "network=yopo-sh2b-sitl" in meta and "owned=0" in meta


def test_isolated_network_is_removed_when_docker_run_fails(tmp_path):
    bin_dir, log = _install_docker(tmp_path)
    env = _env(tmp_path, bin_dir)
    env["DOCKER_FAIL_RUN"] = "1"
    proc = _run(["bash", str(SITL / "run_yopo_l0_sitl.sh"), "--n", "1"], env)
    assert proc.returncode == 7, proc.stderr + proc.stdout
    created = _lines(tmp_path / "nets.txt")
    removed = _lines(tmp_path / "nets-rm.txt")
    # cleanup 已删网，nets.txt 应变空；rm 记录仍在。
    assert created == []
    assert len(removed) == 1 and NET_RE.match(removed[0])
    assert f"--network {removed[0]}" in _mission_line(log)
    assert f"sitl_network {removed[0]} owned=1" in proc.stdout
    assert "run_dir " in proc.stdout


def test_prepare_retries_then_fails_without_leaving_a_network(tmp_path):
    bin_dir, log = _install_docker(tmp_path)
    env = _env(tmp_path, bin_dir)
    env["DOCKER_FAIL_NET_CREATE"] = "1"
    proc = _run(["bash", str(SITL / "run_yopo_l0_sitl.sh"), "--n", "1"], env)
    assert proc.returncode == 3, proc.stderr + proc.stdout
    assert "无法创建隔离网络" in proc.stderr
    assert not any("in_container.sh" in row for row in _lines(log))
    assert _lines(tmp_path / "nets-rm.txt") == []
    creates = [row for row in _lines(log) if row.startswith("network create ")]
    assert len(creates) == 3
    names = [row.rsplit(" ", 1)[-1] for row in creates]
    assert all(NET_RE.match(name) for name in names)
    assert len(set(names)) == 3


def test_cleanup_helper_only_deletes_owned_yopo_sitl_prefix(tmp_path):
    bin_dir, _log = _install_docker(tmp_path)
    env = _env(tmp_path, bin_dir)
    script = r"""
set -euo pipefail
. "$SITL/common.sh"
assert_no_rm() {
  [[ ! -s "$DOCKER_NETS_RM" ]]
}
# 用户网、别人的网，哪怕误把 OWNED 写成 1，也不删。
for name in bridge yopo-b3-sitl yopo-sh2b-sitl docker0; do
  SITL_RUN_NETWORK="$name"
  SITL_RUN_NETWORK_OWNED=1
  sitl_cleanup_run_network
  assert_no_rm
done
# 自建前缀但 OWNED=0：不删。
SITL_RUN_NETWORK="yopo-sitl-should-keep"
SITL_RUN_NETWORK_OWNED=0
sitl_cleanup_run_network
assert_no_rm
# 本趟自建：只删这一个。
printf '%s\n' "yopo-sitl-this-run" "yopo-sitl-someone-else" > "$DOCKER_NETS"
SITL_RUN_NETWORK="yopo-sitl-this-run"
SITL_RUN_NETWORK_OWNED=1
sitl_cleanup_run_network
grep -qxF "yopo-sitl-this-run" "$DOCKER_NETS_RM"
grep -qxF "yopo-sitl-someone-else" "$DOCKER_NETS"
! grep -qxF "yopo-sitl-this-run" "$DOCKER_NETS"
"""
    proc = _run(["bash", "-c", script], {**env, "SITL": str(SITL)})
    assert proc.returncode == 0, proc.stderr + proc.stdout
