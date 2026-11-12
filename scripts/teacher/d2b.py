"""票 D2b：在 D2 管线上加密度 0 / 0.14 / 0.28 的教师轨迹。

几何假设沿用 assumptions.py，不改那份文件，也不改外部 ego-planner-swarm。
大文件写到 /workspace/yopo-teacher-data/d2b/。仓库只交脚本、sha256 清单和文档。
仿真最多 3 路并行，各自一个容器，不用 host 网络，避免 roscore 抢端口。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import queue
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import numpy as np

from teacher.assumptions import (
    EGO_COMMIT,
    EGO_IMAGE,
    GOAL_HOLD_S,
    GOAL_RADIUS_M,
    GOAL_SPEED_MPS,
    SAMPLE_HZ,
    TIMEOUT_S,
)
from teacher.dataset import write_dataset
from teacher.d2b_grid import (
    DEFAULT_SEED_COUNT,
    DENSITIES_PER_M2,
    DRONE_COUNTS,
    D2_SUCCESS_TOTAL,
    TARGET_MIN_SUCCESS,
    grid_size,
    iter_d2b_scenarios,
)
from teacher.launch_xml import launch_xml
from teacher.scenarios import make_scenario, read_scenarios, write_scenarios
from teacher.summarize import machine_info

_REPO = Path(__file__).resolve().parents[2]
_DEFAULT_WORK = Path("/workspace/yopo-teacher-data/d2b")
_EGO_IMAGE_DIGEST = "sha256:85ff4e2f691f7eb5673835afe7e70cd49e552e3cdb8b1938b7c54b647d93b99a"
_MAX_JOBS = 3
_WORKER_PREFIX = "ego-teacher-d2b"
_BUILD_CONTAINER = "ego-teacher-d2b-build"
_WORKER_IMAGE = "yopo-teacher-d2b:92fe9f7"
_MANIFEST_NAME = "d2b-teacher-manifest.json"
_HASH_KINDS = ("trajectory.npz", "record.json", "scene.json", "geometry.json")


def _die(message: str, example: str) -> None:
    print(f"错误: {message}\n  {example}", file=sys.stderr)
    raise SystemExit(2)


def _docker_bin() -> str:
    env = os.environ.get("DOCKER", "").strip()
    if env:
        return env
    for candidate in ("docker", "/usr/local/bin/docker", "/usr/bin/docker"):
        path = candidate if candidate.startswith("/") else shutil.which(candidate)
        if path and Path(path).exists():
            return path
    return "docker"


def _docker(*args: str, check: bool = True, capture: bool = False) -> subprocess.CompletedProcess:
    cmd = ["sudo", "-n", _docker_bin(), *args]
    return subprocess.run(cmd, check=check, text=True, capture_output=capture)


def _dest() -> Path:
    return Path(os.environ.get("EGO_SWARM_DIR", str(Path.home() / "ext" / "ego-planner-swarm")))


def _image_ref() -> str:
    return f"{EGO_IMAGE}@{_EGO_IMAGE_DIGEST}"


def _ensure_clone(dest: Path) -> None:
    if (dest / ".git").is_dir():
        head = subprocess.check_output(["git", "-C", str(dest), "rev-parse", "HEAD"], text=True).strip()
        if head != EGO_COMMIT:
            _die(
                f"{dest} 的 HEAD 是 {head}，不是 {EGO_COMMIT}。",
                'scripts/teacher/d2b.sh build --dir "$HOME/ext/ego-planner-swarm"',
            )
        print(f"已有克隆: {dest} @ {EGO_COMMIT}")
        return
    dest.parent.mkdir(parents=True, exist_ok=True)
    subprocess.check_call(["git", "clone", "https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git", str(dest)])
    subprocess.check_call(["git", "-C", str(dest), "checkout", EGO_COMMIT])


def _image_digest() -> str | None:
    proc = _docker("image", "inspect", EGO_IMAGE, "--format", "{{json .RepoDigests}}", check=False, capture=True)
    if proc.returncode != 0:
        return None
    try:
        digests = json.loads(proc.stdout.strip() or "[]")
    except json.JSONDecodeError:
        return None
    return digests[0] if digests else None


def _worker_image() -> str:
    inspect = _docker("image", "inspect", _WORKER_IMAGE, check=False, capture=True)
    return _WORKER_IMAGE if inspect.returncode == 0 else _image_ref()


def _ensure_container(name: str, dest: Path, work: Path, *, memory: str | None = "2200m", cpus: str | None = "2", image: str | None = None) -> str:
    work.mkdir(parents=True, exist_ok=True)
    wanted = image or _image_ref()
    inspect = _docker("inspect", name, check=False, capture=True)
    if inspect.returncode == 0:
        cfg = _docker("inspect", name, "--format", "{{.Config.Image}}", check=False, capture=True)
        current = (cfg.stdout or "").strip()
        if current == wanted or current.startswith(wanted):
            _docker("start", name, check=False)
            return name
        _docker("rm", "-f", name, check=False)
    args = [
        "run",
        "-d",
        "--init",
        "--name",
        name,
        "-v",
        f"{dest}:/ego",
        "-v",
        f"{_REPO}:/ws:ro",
        "-v",
        f"{work}:/teacher-runs",
        "-w",
        "/ego",
    ]
    if memory:
        args.extend(["--memory", memory])
    if cpus:
        args.extend(["--cpus", cpus])
    args.extend(["--pids-limit", "8192", wanted, "sleep", "infinity"])
    _docker(*args)
    return name


def _prepare_apt_in(container: str) -> None:
    """这台机器访问 snapshots.ros.org 的 HTTP 会被网关打回 500，HTTPS 证书主机名也对不上。

    只在教师容器里改 apt 配置，不改 ego-planner-swarm 源码。
    """
    script = r"""
set -eu
if [ -f /etc/apt/sources.list ]; then
  sed -i -E 's|http://([a-z0-9.-]*ubuntu.com)|https://\1|g' /etc/apt/sources.list
fi
if [ -f /etc/apt/sources.list.d/ros1-snapshots.list ]; then
  sed -i 's|http://snapshots.ros.org|https://snapshots.ros.org|g' /etc/apt/sources.list.d/ros1-snapshots.list
fi
cat >/etc/apt/apt.conf.d/99d2b-https-workaround <<EOF
Acquire::https::Verify-Peer "false";
Acquire::https::Verify-Host "false";
EOF
"""
    proc = _docker("exec", container, "bash", "-lc", script, check=False, capture=True)
    if proc.returncode != 0:
        _die("容器里改 apt 源失败。", "scripts/teacher/d2b.sh build")


def cmd_build(args: argparse.Namespace) -> None:
    dest = Path(args.dir) if args.dir else _dest()
    work = Path(args.work)
    if args.dry_run:
        print("dry-run")
        print(f"commit: {EGO_COMMIT}")
        print(f"image: {EGO_IMAGE}")
        print(f"image_digest: {_EGO_IMAGE_DIGEST}")
        print(f"dir: {dest}")
        print(f"container: {_BUILD_CONTAINER}")
        print("launch: 本管线生成的 teacher.launch，不启动 rviz")
        print("note: 地图边长和障碍半径是假设，不是论文复现。")
        return
    probe = _docker("info", check=False, capture=True)
    if probe.returncode != 0:
        _die(
            "连不上 Docker 守护进程。本脚本不启动 dockerd。",
            "sudo dockerd --host=unix:///var/run/docker.sock --iptables=false --storage-driver=overlay2",
        )
    _ensure_clone(dest)
    pull = _docker("pull", _image_ref(), check=False, capture=True)
    if pull.returncode != 0:
        print(pull.stdout or "", end="")
        print(pull.stderr or "", end="", file=sys.stderr)
        _die("拉取镜像失败。", f"sudo docker pull {_image_ref()}")
    _docker("tag", _image_ref(), EGO_IMAGE, check=False)
    _ensure_container(_BUILD_CONTAINER, dest, work, memory="4g", cpus="2")
    if not args.rebuild and (dest / "devel" / "setup.bash").is_file():
        print(f"已有 {dest}/devel/setup.bash，跳过编译。要重编加 --rebuild。")
        inspect = _docker("image", "inspect", _WORKER_IMAGE, check=False, capture=True)
        if inspect.returncode != 0:
            _prepare_apt_in(_BUILD_CONTAINER)
            _docker("commit", _BUILD_CONTAINER, _WORKER_IMAGE)
            print(f"worker_image: {_WORKER_IMAGE}")
        return
    _prepare_apt_in(_BUILD_CONTAINER)
    script = r"""
set -eu
export DEBIAN_FRONTEND=noninteractive
missing=0
for p in build-essential gcc libarmadillo-dev ros-noetic-pcl-ros ros-noetic-pcl-conversions \
         ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf \
         ros-noetic-laser-geometry ros-noetic-nodelet ros-noetic-cmake-modules \
         libpcl-dev libeigen3-dev; do
  dpkg -s "$p" >/dev/null 2>&1 || missing=1
done
if [ "$missing" = 1 ]; then
  apt-get update -qq
  apt-get install -y -qq build-essential gcc libarmadillo-dev ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf \
    ros-noetic-laser-geometry ros-noetic-nodelet ros-noetic-cmake-modules libpcl-dev libeigen3-dev
fi
cat >/tmp/linebuf.c <<'EOF'
#include <stdio.h>
__attribute__((constructor)) static void ego_linebuf(void) {
  setvbuf(stdout, NULL, _IOLBF, 0);
  setvbuf(stderr, NULL, _IOLBF, 0);
}
EOF
gcc -shared -fPIC -o /tmp/linebuf.so /tmp/linebuf.c
cp /tmp/linebuf.so /teacher-runs/linebuf.so
set +u
source /opt/ros/noetic/setup.bash
cd /ego
# E-C1 记下第一次 catkin_make 会缺 MultiOccupancyGrid.h。这里先生成该头，再全量编译。
catkin_make -j2 multi_map_server_generate_messages_cpp
catkin_make -j2
"""
    print("开始编译 ego-planner-swarm")
    proc = subprocess.run(["sudo", "-n", _docker_bin(), "exec", _BUILD_CONTAINER, "bash", "-lc", script])
    if proc.returncode != 0:
        _die("容器内编译失败。", "scripts/teacher/d2b.sh build --rebuild")
    digest = _image_digest() or f"{EGO_IMAGE}@{_EGO_IMAGE_DIGEST}"
    stamp = {
        "commit": EGO_COMMIT,
        "image": EGO_IMAGE,
        "image_digest": digest,
        "pinned_digest": _EGO_IMAGE_DIGEST,
        "container": _BUILD_CONTAINER,
        "dir": str(dest),
        "assumption": "地图边长和障碍半径是假设，不是论文复现。见 scripts/teacher/assumptions.py。",
    }
    work.mkdir(parents=True, exist_ok=True)
    (work / "build-stamp.json").write_text(json.dumps(stamp, indent=2) + "\n", encoding="utf-8")
    _docker("commit", _BUILD_CONTAINER, _WORKER_IMAGE)
    print(f"编译结束: {dest}")
    print(f"image_digest: {digest}")
    print(f"worker_image: {_WORKER_IMAGE}")


def _parse_id(item: str) -> dict:
    parts = item.split("-")
    if len(parts) != 4 or not parts[1].startswith("n") or not parts[3].startswith("s"):
        _die(f"无法识别的场景 id: {item}", "scripts/teacher/d2b.sh run --id d000-n1-parallel-s0")
    try:
        density = {"d000": 0.0, "d014": 0.14, "d028": 0.28, "d042": 0.42}[parts[0]]
        pattern = parts[2]
        n = int(parts[1][1:])
        seed = int(parts[3][1:])
    except (KeyError, ValueError):
        _die(f"无法识别的场景 id: {item}", "scripts/teacher/d2b.sh run --id d000-n1-parallel-s0")
    return make_scenario(density, n, pattern, seed)


def _load_selected(args: argparse.Namespace) -> list[dict]:
    if args.id:
        return [_parse_id(item) for item in args.id]
    path = Path(args.scenarios) if getattr(args, "scenarios", None) else Path(args.work) / "scenarios.jsonl"
    if path.is_file():
        rows = read_scenarios(path)
    elif args.dry_run:
        rows = list(
            iter_d2b_scenarios(
                seed_count=int(args.seed_count),
                include_six_open=bool(args.include_six_open),
            )
        )
    else:
        _die(f"还没有场景列表 {path}", "scripts/teacher/d2b.sh scenarios")
    if args.limit is not None:
        rows = rows[: args.limit]
    return rows


def cmd_scenarios(args: argparse.Namespace) -> None:
    path = Path(args.out) if args.out else Path(args.work) / "scenarios.jsonl"
    rows = list(
        iter_d2b_scenarios(
            seed_count=int(args.seed_count),
            include_six_open=bool(args.include_six_open),
        )
    )
    if args.dry_run:
        print(f"dry-run scenarios: {len(rows)}")
        print(f"first: {rows[0]['id']}")
        print(f"last: {rows[-1]['id']}")
        print(f"out: {path}")
        print("assumption: 地图边长和障碍半径是假设，不是论文复现。")
        return
    n = write_scenarios(path, rows)
    print(f"scenarios: {n}")
    print(f"out: {path}")


_KILL = r"""
import os, signal, sys
me = os.getpid()
keys = ("ego_planner_node","traj_server","pcl_render_node","poscmd_2_odom","odom_visualization","rosmaster","roscore","rosout","roslaunch","ros_record.py","teacher_record","rosbag")
for pid in os.listdir("/proc"):
    if not pid.isdigit() or int(pid) == me:
        continue
    try:
        cmd = open("/proc/%s/cmdline" % pid, "rb").read().replace(b"\x00", b" ").decode("utf-8", "replace")
    except Exception:
        continue
    if any(k in cmd for k in keys):
        try:
            os.kill(int(pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
"""


def _kill_helper(work: Path) -> str:
    path = work / "kill_ros.py"
    path.write_text(_KILL, encoding="utf-8")
    return "/teacher-runs/kill_ros.py"


def arrival_time_s(time_s, position_m, velocity_mps, goals) -> float | None:
    """第一段满足到达门并保持满 GOAL_HOLD_S（允许少一拍）的起始时刻。未到达则未知。

    到达门与 metrics._held 相同：每架距目标 <= 0.5 m，速度 <= 0.3 m/s。
    这是本管线的定义，论文没有到达半径。
    """
    time_s = np.asarray(time_s, dtype=float)
    position_m = np.asarray(position_m, dtype=float)
    velocity_mps = np.asarray(velocity_mps, dtype=float)
    goals = np.asarray(goals, dtype=float)
    if time_s.size == 0 or position_m.size == 0:
        return None
    n = position_m.shape[1]
    count = int(time_s.shape[0])
    inside = []
    for i in range(count):
        ok = True
        for k in range(n):
            dist = float(np.linalg.norm(position_m[i, k] - goals[k]))
            speed = float(np.linalg.norm(velocity_mps[i, k]))
            if dist > GOAL_RADIUS_M or speed > GOAL_SPEED_MPS:
                ok = False
                break
        inside.append(ok)
    need = GOAL_HOLD_S - (1.0 / SAMPLE_HZ)
    i = 0
    while i < count:
        if not inside[i]:
            i += 1
            continue
        j = i
        while j + 1 < count and inside[j + 1]:
            j += 1
        if float(time_s[j] - time_s[i]) + 1e-9 >= need:
            return float(time_s[i])
        i = j + 1
    return None


def arrival_time_from_run(run_dir: Path) -> float | None:
    npz_path = run_dir / "trajectory.npz"
    if not npz_path.is_file():
        return None
    with np.load(npz_path) as npz:
        return arrival_time_s(npz["time_s"], npz["position_m"], npz["velocity_mps"], npz["goal_m"])


def _should_skip(run_dir: Path, resume: bool, retry_unknown: bool) -> bool:
    record_path = run_dir / "record.json"
    if not resume or not record_path.is_file():
        return False
    if retry_unknown:
        try:
            record = json.loads(record_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            return False
        if record.get("metrics", {}).get("success") is None:
            return False
    return True


def _run_one(scenario: dict, work: Path, timeout_s: float, resume: bool, retry_unknown: bool, container: str) -> str:
    run_dir = work / "runs" / scenario["id"]
    run_dir.mkdir(parents=True, exist_ok=True)
    if _should_skip(run_dir, resume, retry_unknown):
        print(f"skip {scenario['id']}")
        return "skip"
    (run_dir / "scenario.json").write_text(json.dumps(scenario), encoding="utf-8")
    (run_dir / "teacher.launch").write_text(launch_xml(scenario), encoding="utf-8")
    kill_py = _kill_helper(work)
    if not (resume and (run_dir / "raw_record.json").is_file() and not retry_unknown):
        remote = f"/teacher-runs/runs/{scenario['id']}"
        preload = ""
        if (work / "linebuf.so").is_file():
            preload = "export LD_PRELOAD=/teacher-runs/linebuf.so\n"
        deadline = int(timeout_s) + 25
        script = f"""#!/bin/bash
set +u
source /opt/ros/noetic/setup.bash
source /ego/devel/setup.bash
set -u
export PYTHONUNBUFFERED=1
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
{preload}python3 {kill_py}
rm -f {remote}/done.json {remote}/raw_record.json {remote}/roslaunch.log
roscore > {remote}/roscore.log 2>&1 &
for _i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25; do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.2
done
python3 -u /ws/scripts/teacher/ros_record.py --scenario {remote}/scenario.json --out {remote} --timeout {timeout_s} > {remote}/record.log 2>&1 &
rec=$!
sleep 1
roslaunch {remote}/teacher.launch > {remote}/roslaunch.log 2>&1 &
launch=$!
deadline=$((SECONDS + {deadline}))
while [ "$SECONDS" -lt "$deadline" ]; do
  if [ -f {remote}/done.json ]; then
    break
  fi
  kill -0 "$launch" 2>/dev/null || break
  sleep 0.5
done
kill -INT "$rec" 2>/dev/null || true
for _i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
  [ -f {remote}/done.json ] && break
  sleep 0.5
done
kill -INT "$launch" 2>/dev/null || true
sleep 2
kill -KILL "$rec" "$launch" 2>/dev/null || true
python3 {kill_py}
chmod -R a+rwX {remote} || true
"""
        script_path = run_dir / "run.sh"
        script_path.write_text(script, encoding="utf-8")
        script_path.chmod(0o755)
        wrap = int(timeout_s) + 60
        try:
            proc = subprocess.run(
                ["sudo", "-n", _docker_bin(), "exec", container, "bash", "-lc", f"nice -n 10 bash {remote}/run.sh"],
                timeout=wrap,
            )
            code = proc.returncode
        except subprocess.TimeoutExpired:
            _docker("exec", container, "python3", kill_py, check=False)
            code = -9
        if code != 0 and not (run_dir / "raw_record.json").is_file():
            print(f"fail {scenario['id']} docker_exec={code}", file=sys.stderr)
    if not os.access(run_dir, os.W_OK):
        subprocess.run(["sudo", "-n", "chmod", "-R", "a+rwX", str(run_dir)], check=False)
    raw_path = run_dir / "raw_record.json"
    if raw_path.is_file():
        raw = json.loads(raw_path.read_text(encoding="utf-8"))
    else:
        raw = {
            "run_status": "launch_failed",
            "wall_s": None,
            "t0_ros_s": None,
            "sample_hz": None,
            "bspline_import": None,
            "yaw_source": None,
            "acceleration_source": None,
            "time_s": [],
            "position_m": [],
            "velocity_mps": [],
            "acceleration_mps2": [],
            "yaw_rad": [],
            "bsplines": [],
        }
    log_path = run_dir / "roslaunch.log"
    log_text = log_path.read_text(encoding="utf-8", errors="replace") if log_path.is_file() else ""
    record = write_dataset(run_dir, scenario, raw, log_text)
    arrived_s = arrival_time_from_run(run_dir)
    record["metrics"]["arrival_time_s"] = arrived_s
    (run_dir / "record.json").write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    success = record["metrics"]["success"]
    print(f"done {scenario['id']} status={record['run']['status']} success={success} arrival_time_s={arrived_s}")
    return "ok"


def cmd_run(args: argparse.Namespace) -> None:
    rows = _load_selected(args)
    jobs = int(args.jobs)
    if jobs < 1 or jobs > _MAX_JOBS:
        _die(f"--jobs 只能是 1 到 {_MAX_JOBS}，收到 {jobs}。", "scripts/teacher/d2b.sh run --jobs 3")
    if args.dry_run:
        print(f"dry-run run: {len(rows)}")
        if rows:
            print(f"first: {rows[0]['id']}")
            print(f"last: {rows[-1]['id']}")
        print(f"timeout_s: {args.timeout}")
        print(f"jobs: {jobs}")
        print(f"work: {args.work}")
        return
    dest = _dest()
    if not (dest / "devel" / "setup.bash").is_file():
        _die("还没有编译。", "scripts/teacher/d2b.sh build")
    work = Path(args.work)
    work.mkdir(parents=True, exist_ok=True)
    workers = [
        _ensure_container(f"{_WORKER_PREFIX}-{slot}", dest, work, image=_worker_image())
        for slot in range(jobs)
    ]
    slots: queue.Queue[str] = queue.Queue()
    for name in workers:
        slots.put(name)
    resume = not args.no_resume
    retry_unknown = not args.no_retry_unknown
    timeout_s = float(args.timeout)

    def work_one(scenario: dict) -> str:
        name = slots.get()
        try:
            return _run_one(scenario, work, timeout_s, resume, retry_unknown, name)
        finally:
            slots.put(name)

    wall0 = time.time()
    ok = skip = fail = 0
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        futs = {pool.submit(work_one, scenario): scenario["id"] for scenario in rows}
        for fut in as_completed(futs):
            ident = futs[fut]
            try:
                status = fut.result()
            except Exception as exc:  # noqa: BLE001
                print(f"fail {ident} {exc!r}", file=sys.stderr)
                fail += 1
                continue
            if status == "skip":
                skip += 1
            else:
                ok += 1
    elapsed = time.time() - wall0
    (work / "run-wall.json").write_text(
        json.dumps(
            {
                "batch_wall_s": elapsed,
                "n": len(rows),
                "ok": ok,
                "skip": skip,
                "fail": fail,
                "jobs": jobs,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"run_wall_s: {elapsed:.3f}")
    print(f"ok={ok} skip={skip} fail={fail}")
    if elapsed > 0:
        print(f"scenarios_per_hour: {len(rows) / (elapsed / 3600.0):.3f}")
    if fail:
        raise SystemExit(1)


def _cell(rows: list[dict]) -> dict:
    success = sum(1 for r in rows if r["success"] is True)
    fail = sum(1 for r in rows if r["success"] is False)
    unknown = sum(1 for r in rows if r["success"] is None)
    measured = success + fail
    obstacle = sum(1 for r in rows if r["collision_obstacle"] is True)
    inter = sum(1 for r in rows if r["collision_inter_drone"] is True)
    arrived = sum(1 for r in rows if r["arrived"] is True)
    times = [r["arrival_time_s"] for r in rows if isinstance(r["arrival_time_s"], (int, float))]
    success_times = [
        r["arrival_time_s"]
        for r in rows
        if r["success"] is True and isinstance(r["arrival_time_s"], (int, float))
    ]

    def _stats(vals: list[float]) -> dict:
        if not vals:
            return {"n": 0, "mean_s": None, "median_s": None, "min_s": None, "max_s": None}
        ordered = sorted(vals)
        mid = len(ordered) // 2
        if len(ordered) % 2:
            median = ordered[mid]
        else:
            median = 0.5 * (ordered[mid - 1] + ordered[mid])
        return {
            "n": len(ordered),
            "mean_s": float(sum(ordered) / len(ordered)),
            "median_s": float(median),
            "min_s": float(ordered[0]),
            "max_s": float(ordered[-1]),
        }

    return {
        "n": len(rows),
        "success": success,
        "fail": fail,
        "unknown": unknown,
        "rate": None if measured == 0 else success / measured,
        "arrived": arrived,
        "collision_obstacle": obstacle,
        "collision_inter_drone": inter,
        "arrival_time_arrived": _stats(times),
        "arrival_time_success": _stats(success_times),
    }


def _format_rate(cell: dict) -> str:
    if cell["n"] == 0 or cell["rate"] is None:
        return "未知"
    return f"{cell['success']}/{cell['success'] + cell['fail']}"


def _format_time(stats: dict) -> str:
    if not stats or not stats.get("n"):
        return "未知"
    return f"均值 {stats['mean_s']:.3f} s，中位 {stats['median_s']:.3f} s，范围 {stats['min_s']:.3f}–{stats['max_s']:.3f} s，n={stats['n']}"


def summarize_d2b(run_dir: Path, *, batch_wall_s: float | None = None) -> dict:
    rows = []
    if run_dir.is_dir():
        for record_path in sorted(run_dir.glob("*/record.json")):
            record = json.loads(record_path.read_text(encoding="utf-8"))
            scenario = record["scenario"]
            metrics = record["metrics"]
            arrived_s = metrics.get("arrival_time_s")
            if arrived_s is None:
                arrived_s = arrival_time_from_run(record_path.parent)
            rows.append(
                {
                    "id": scenario["id"],
                    "density_per_m2": scenario["density_per_m2"],
                    "n_drones": scenario["n_drones"],
                    "pattern": scenario["pattern"],
                    "seed": scenario["seed"],
                    "status": record["run"].get("status"),
                    "success": metrics.get("success"),
                    "arrived": metrics.get("arrived"),
                    "collision_obstacle": metrics.get("collision_obstacle"),
                    "collision_inter_drone": metrics.get("collision_inter_drone"),
                    "min_inter_drone_m": metrics.get("min_inter_drone_m"),
                    "min_obstacle_surface_m": metrics.get("min_obstacle_surface_m"),
                    "arrival_time_s": arrived_s,
                    "wall_s": record["run"].get("wall_s"),
                }
            )
    densities = list(DENSITIES_PER_M2)
    drones = list(DRONE_COUNTS)
    if any(r["n_drones"] == 6 for r in rows):
        if 6 not in drones:
            drones.append(6)
    by_density = {str(d): _cell([r for r in rows if r["density_per_m2"] == d]) for d in densities}
    by_drones = {str(n): _cell([r for r in rows if r["n_drones"] == n]) for n in drones}
    by_pattern = {
        "parallel": _cell([r for r in rows if r["pattern"] == "parallel"]),
        "headon": _cell([r for r in rows if r["pattern"] == "headon"]),
    }
    grid = {}
    for density in densities:
        grid[str(density)] = {}
        for n in drones:
            grid[str(density)][str(n)] = _cell(
                [r for r in rows if r["density_per_m2"] == density and r["n_drones"] == n]
            )
    walls = [r["wall_s"] for r in rows if isinstance(r["wall_s"], (int, float))]
    per_hour = None
    if batch_wall_s and batch_wall_s > 0 and rows:
        per_hour = len(rows) / (batch_wall_s / 3600.0)
    overall = _cell(rows)
    return {
        "null_means": "未知",
        "assumption": "地图边长和障碍半径是假设，不是论文复现。见 scripts/teacher/assumptions.py。",
        "d2_success_total": D2_SUCCESS_TOTAL,
        "target_min_success": TARGET_MIN_SUCCESS,
        "order_of_magnitude_met": overall["success"] >= TARGET_MIN_SUCCESS,
        "n": len(rows),
        "overall": overall,
        "by_density": by_density,
        "by_drones": by_drones,
        "by_pattern": by_pattern,
        "by_density_and_drones": grid,
        "batch_wall_s": batch_wall_s,
        "sum_scenario_wall_s": None if not walls else float(sum(walls)),
        "scenarios_per_hour": per_hour,
        "scenarios_per_hour_basis": "batch_wall_s" if batch_wall_s else "sum_scenario_wall_s",
        "machine": machine_info(),
        "runs": rows,
    }


def format_d2b_table(summary: dict) -> str:
    lines = [
        "地图边长和障碍半径是假设，不是论文复现。",
        "",
        "| 密度 obs/m² | 1 架 | 3 架 | 6 架 | 该密度合计 |",
        "|---|---|---|---|---|",
    ]
    grid = summary["by_density_and_drones"]
    for density in ("0.0", "0.14", "0.28"):
        if density not in grid:
            continue
        row = [density]
        for n in ("1", "3", "6"):
            cell = grid[density].get(n) or {"n": 0, "rate": None, "success": 0, "fail": 0}
            row.append(_format_rate(cell))
        row.append(_format_rate(summary["by_density"][density]))
        lines.append("| " + " | ".join(row) + " |")
    lines.append("")
    lines.append("| 机数 | 成功 | 障碍碰撞 | 机间碰撞 | 到达时间（成功场） |")
    lines.append("|---|---|---|---|---|")
    for n, label in (("1", "1 架"), ("3", "3 架"), ("6", "6 架")):
        if n not in summary["by_drones"]:
            continue
        cell = summary["by_drones"][n]
        lines.append(
            f"| {label} | {_format_rate(cell)} | {cell['collision_obstacle']} | "
            f"{cell['collision_inter_drone']} | {_format_time(cell['arrival_time_success'])} |"
        )
    lines.append("")
    lines.append("| 图案 | 成功率 |")
    lines.append("|---|---|")
    for pattern, label in (("parallel", "平行"), ("headon", "对向")):
        lines.append(f"| {label} | {_format_rate(summary['by_pattern'][pattern])} |")
    overall = summary["overall"]
    rate = summary["scenarios_per_hour"]
    speed = "未知" if rate is None else f"{rate:.2f}"
    lines.append("")
    lines.append(f"场景/小时: {speed}（口径 {summary['scenarios_per_hour_basis']}）")
    lines.append(
        f"合计成功: {_format_rate(overall)}，未知 {overall['unknown']}。"
        f"D2 成功 {summary['d2_success_total']} 场，本票目标至少 {summary['target_min_success']} 场。"
        f"是否达到一个数量级: {'是' if summary['order_of_magnitude_met'] else '否'}。"
    )
    lines.append(f"障碍碰撞 {overall['collision_obstacle']}，机间碰撞 {overall['collision_inter_drone']}。")
    lines.append(f"成功场到达时间: {_format_time(overall['arrival_time_success'])}")
    return "\n".join(lines) + "\n"


def cmd_summarize(args: argparse.Namespace) -> None:
    work = Path(args.work)
    wall = None
    wall_path = work / "run-wall.json"
    if wall_path.is_file():
        wall = json.loads(wall_path.read_text(encoding="utf-8")).get("batch_wall_s")
    summary = summarize_d2b(work / "runs", batch_wall_s=wall)
    stamp = work / "build-stamp.json"
    if stamp.is_file():
        summary["build"] = json.loads(stamp.read_text(encoding="utf-8"))
    out = Path(args.out) if args.out else work / "d2b-teacher-batch.json"
    text = format_d2b_table(summary)
    if args.dry_run:
        print(text)
        print(f"out: {out}")
        return
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = dict(summary)
    # 仓库里的文档只摘表。逐场记录只留在仓库外的 batch json。
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(text)
    print(f"out: {out}")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_manifest(work: Path) -> dict:
    runs = work / "runs"
    files = []
    for run_dir in sorted(p for p in runs.glob("*") if p.is_dir()) if runs.is_dir() else []:
        for name in _HASH_KINDS:
            path = run_dir / name
            if not path.is_file():
                continue
            rel = path.relative_to(work).as_posix()
            files.append(
                {
                    "path": rel,
                    "sha256": sha256_file(path),
                    "bytes": path.stat().st_size,
                    "kind": name,
                }
            )
    for extra in ("scenarios.jsonl", "build-stamp.json", "run-wall.json", "d2b-teacher-batch.json"):
        path = work / extra
        if path.is_file():
            files.append(
                {
                    "path": extra,
                    "sha256": sha256_file(path),
                    "bytes": path.stat().st_size,
                    "kind": extra,
                }
            )
    stamp = {}
    stamp_path = work / "build-stamp.json"
    if stamp_path.is_file():
        stamp = json.loads(stamp_path.read_text(encoding="utf-8"))
    return {
        "schema": "yopo-teacher-d2b-manifest/1",
        "assumption": "地图边长和障碍半径是假设，不是论文复现。见 scripts/teacher/assumptions.py。",
        "work": str(work),
        "ego_commit": stamp.get("commit", EGO_COMMIT),
        "image": stamp.get("image", EGO_IMAGE),
        "image_digest": stamp.get("image_digest") or stamp.get("pinned_digest") or _EGO_IMAGE_DIGEST,
        "n_files": len(files),
        "files": files,
    }


def cmd_manifest(args: argparse.Namespace) -> None:
    work = Path(args.work)
    out = Path(args.out) if args.out else _REPO / "docs" / "research" / _MANIFEST_NAME
    manifest = build_manifest(work)
    if args.dry_run:
        print(f"dry-run manifest files: {manifest['n_files']}")
        print(f"out: {out}")
        return
    out.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(manifest, ensure_ascii=False, indent=2) + "\n"
    out.write_text(text, encoding="utf-8")
    also = work / _MANIFEST_NAME
    also.write_text(text, encoding="utf-8")
    print(f"manifest_files: {manifest['n_files']}")
    print(f"out: {out}")


def cmd_all(args: argparse.Namespace) -> None:
    if args.dry_run:
        cmd_build(args)
        cmd_scenarios(args)
        cmd_run(args)
        cmd_summarize(args)
        cmd_manifest(args)
        return
    cmd_build(args)
    cmd_scenarios(args)
    cmd_run(args)
    cmd_summarize(args)
    cmd_manifest(args)


def _parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--work", default=str(_DEFAULT_WORK), help="运行目录，默认 /workspace/yopo-teacher-data/d2b")
    common.add_argument("--dry-run", action="store_true", help="只打印将要做的事")
    common.add_argument("--seed-count", type=int, default=DEFAULT_SEED_COUNT, help="每种格子的种子数，从 0 起")
    common.add_argument("--include-six-open", action="store_true", help="额外加上密度 0 的六机")

    parser = argparse.ArgumentParser(
        prog="scripts/teacher/d2b.sh",
        description="在仓库外的 ego-planner-swarm 上为 D2b 扩种教师轨迹。",
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    build = sub.add_parser("build", parents=[common], help="克隆并在容器里编译 92fe9f7")
    build.add_argument("--dir", default=None, help="克隆目录，默认 $EGO_SWARM_DIR 或 ~/ext/ego-planner-swarm")
    build.add_argument("--rebuild", action="store_true")
    build.set_defaults(func=cmd_build)

    scenarios = sub.add_parser("scenarios", parents=[common], help="写出 D2b 场景 jsonl")
    scenarios.add_argument("--out", default=None)
    scenarios.set_defaults(func=cmd_scenarios)

    run = sub.add_parser("run", parents=[common], help="并行无头跑教师并写成 npz")
    run.add_argument("--scenarios", default=None)
    run.add_argument("--limit", type=int, default=None)
    run.add_argument("--timeout", type=float, default=TIMEOUT_S)
    run.add_argument("--id", action="append", default=[])
    run.add_argument("--no-resume", action="store_true")
    run.add_argument("--no-retry-unknown", action="store_true")
    run.add_argument("--jobs", type=int, default=_MAX_JOBS, help="并行仿真路数，最多 3")
    run.set_defaults(func=cmd_run)

    summary = sub.add_parser("summarize", parents=[common], help="按密度和机数汇总成功、碰撞、到达时间")
    summary.add_argument("--out", default=None, help="默认写到仓库外 work/d2b-teacher-batch.json")
    summary.set_defaults(func=cmd_summarize)

    manifest = sub.add_parser("manifest", parents=[common], help="写出带 sha256 的清单")
    manifest.add_argument("--out", default=None, help="默认 docs/research/d2b-teacher-manifest.json")
    manifest.set_defaults(func=cmd_manifest)

    entire = sub.add_parser("all", parents=[common], help="build、生成场景、跑、汇总、清单")
    entire.add_argument("--dir", default=None)
    entire.add_argument("--rebuild", action="store_true")
    entire.add_argument("--scenarios", default=None)
    entire.add_argument("--limit", type=int, default=None)
    entire.add_argument("--timeout", type=float, default=TIMEOUT_S)
    entire.add_argument("--id", action="append", default=[])
    entire.add_argument("--no-resume", action="store_true")
    entire.add_argument("--no-retry-unknown", action="store_true")
    entire.add_argument("--jobs", type=int, default=_MAX_JOBS)
    entire.add_argument("--out", default=None)
    entire.set_defaults(func=cmd_all)
    return parser


def main(argv: list[str] | None = None) -> None:
    parser = _parser()
    args = parser.parse_args(argv)
    if getattr(args, "seed_count", None) is not None and args.seed_count < 1:
        _die("--seed-count 需要正整数。", "scripts/teacher/d2b.sh scenarios --seed-count 128")
    if getattr(args, "limit", None) is not None and args.limit < 0:
        _die("--limit 需要非负整数。", "scripts/teacher/d2b.sh run --limit 10")
    if getattr(args, "timeout", None) is not None and args.timeout < 0:
        _die("--timeout 需要非负秒数。", "scripts/teacher/d2b.sh run --timeout 50")
    args.func(args)


if __name__ == "__main__":
    main()
