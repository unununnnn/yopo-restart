"""一条命令生成 EGO-Swarm 教师数据。

源码留在仓库外（默认 ~/ext/ego-planner-swarm，提交 92fe9f7）。
不把那个仓库拷进本仓库，也不加子模块。

示例:
  scripts/teacher/generate.sh --help
  scripts/teacher/generate.sh all --dry-run
  scripts/teacher/generate.sh all --limit 192
  scripts/teacher/generate.sh run --id d000-n1-parallel-s0
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

from .assumptions import CONTAINER, EGO_COMMIT, EGO_IMAGE, TIMEOUT_S
from .dataset import write_dataset
from .launch_xml import launch_xml
from .scenarios import iter_scenarios, make_scenario, read_scenarios, write_scenarios
from .summarize import format_table, summarize

_REPO = Path(__file__).resolve().parents[2]
_DEFAULT_WORK = Path("/tmp/teacher-d2")
_DEFAULT_SAMPLE = _REPO / "data" / "teacher" / "sample"


def _die(message: str, example: str) -> None:
    print(f"错误: {message}\n  {example}", file=sys.stderr)
    raise SystemExit(2)


def _docker(*args: str, check: bool = True, capture: bool = False) -> subprocess.CompletedProcess:
    cmd = ["sudo", "-n", "docker", *args]
    return subprocess.run(cmd, check=check, text=True, capture_output=capture)


def _dest() -> Path:
    return Path(os.environ.get("EGO_SWARM_DIR", str(Path.home() / "ext" / "ego-planner-swarm")))


def _ensure_clone(dest: Path) -> None:
    if (dest / ".git").is_dir():
        head = subprocess.check_output(["git", "-C", str(dest), "rev-parse", "HEAD"], text=True).strip()
        if head != EGO_COMMIT:
            _die(
                f"{dest} 的 HEAD 是 {head}，不是 {EGO_COMMIT}。",
                "scripts/teacher/generate.sh build --dir \"$HOME/ext/ego-planner-swarm\"",
            )
        print(f"已有克隆: {dest} @ {EGO_COMMIT}")
        return
    dest.parent.mkdir(parents=True, exist_ok=True)
    subprocess.check_call(["git", "clone", "https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git", str(dest)])
    subprocess.check_call(["git", "-C", str(dest), "checkout", EGO_COMMIT])


def _ensure_container(dest: Path, work: Path) -> None:
    work.mkdir(parents=True, exist_ok=True)
    inspect = _docker("inspect", CONTAINER, check=False, capture=True)
    if inspect.returncode == 0:
        _docker("start", CONTAINER)
        return
    _docker("pull", EGO_IMAGE)
    _docker(
        "run",
        "-d",
        "--name",
        CONTAINER,
        "--network=host",
        "-v",
        f"{dest}:/ego",
        "-v",
        f"{_REPO}:/ws",
        "-v",
        f"{work}:/teacher-runs",
        "-w",
        "/ego",
        EGO_IMAGE,
        "sleep",
        "infinity",
    )


def _image_digest() -> str | None:
    proc = _docker("image", "inspect", EGO_IMAGE, "--format", "{{json .RepoDigests}}", check=False, capture=True)
    if proc.returncode != 0:
        return None
    try:
        digests = json.loads(proc.stdout.strip() or "[]")
    except json.JSONDecodeError:
        return None
    return digests[0] if digests else None


def cmd_build(args: argparse.Namespace) -> None:
    dest = Path(args.dir) if args.dir else _dest()
    work = Path(args.work)
    if args.dry_run:
        print("dry-run")
        print(f"commit: {EGO_COMMIT}")
        print(f"image: {EGO_IMAGE}")
        print(f"dir: {dest}")
        print(f"container: {CONTAINER}")
        print("launch: 本管线生成的 teacher.launch，不启动 rviz")
        return
    if shutil.which("docker") is None and not Path("/usr/bin/docker").exists():
        _die("找不到 docker。", "sudo apt-get install -y docker.io && scripts/teacher/generate.sh build")
    probe = _docker("info", check=False, capture=True)
    if probe.returncode != 0:
        _die(
            "连不上 Docker 守护进程。本脚本不启动 dockerd。",
            "sudo dockerd --host=unix:///var/run/docker.sock --iptables=false --storage-driver=overlay2",
        )
    _ensure_clone(dest)
    _ensure_container(dest, work)
    if not args.rebuild and (dest / "devel" / "setup.bash").is_file():
        print(f"已有 {dest}/devel/setup.bash，跳过编译。要重编加 --rebuild。")
        return
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
set +u
source /opt/ros/noetic/setup.bash
cd /ego
# E-C1 记下第一次 catkin_make 会缺 MultiOccupancyGrid.h。这里先生成该头，再全量编译。
catkin_make -j2 multi_map_server_generate_messages_cpp
catkin_make -j2
"""
    print("开始编译 ego-planner-swarm")
    proc = subprocess.run(["sudo", "-n", "docker", "exec", CONTAINER, "bash", "-lc", script])
    if proc.returncode != 0:
        _die("容器内编译失败。", "scripts/teacher/generate.sh build --rebuild")
    digest = _image_digest()
    stamp = {
        "commit": EGO_COMMIT,
        "image": EGO_IMAGE,
        "image_digest": digest,
        "container": CONTAINER,
        "dir": str(dest),
    }
    (work / "build-stamp.json").write_text(json.dumps(stamp, indent=2) + "\n", encoding="utf-8")
    print(f"编译结束: {dest}")
    if digest:
        print(f"image_digest: {digest}")


def _load_selected(args: argparse.Namespace) -> list[dict]:
    if args.id:
        rows = []
        for item in args.id:
            parts = item.split("-")
            # d014-n3-parallel-s0
            if len(parts) != 4 or not parts[1].startswith("n") or not parts[3].startswith("s"):
                _die(f"无法识别的场景 id: {item}", "scripts/teacher/generate.sh run --id d000-n1-parallel-s0")
            try:
                density = {"d000": 0.0, "d014": 0.14, "d028": 0.28, "d042": 0.42}[parts[0]]
                pattern = parts[2]
                n = int(parts[1][1:])
                seed = int(parts[3][1:])
            except (KeyError, ValueError):
                _die(f"无法识别的场景 id: {item}", "scripts/teacher/generate.sh run --id d000-n1-parallel-s0")
            rows.append(make_scenario(density, n, pattern, seed))
        return rows
    path = Path(args.scenarios) if args.scenarios else Path(args.work) / "scenarios.jsonl"
    if path.is_file():
        rows = read_scenarios(path)
    elif args.dry_run:
        rows = list(iter_scenarios())
    else:
        _die(f"还没有场景列表 {path}", "scripts/teacher/generate.sh scenarios --work /tmp/teacher-d2")
    if args.limit is not None:
        rows = rows[: args.limit]
    return rows


def cmd_scenarios(args: argparse.Namespace) -> None:
    path = Path(args.out) if args.out else Path(args.work) / "scenarios.jsonl"
    if args.dry_run:
        rows = list(iter_scenarios())
        print(f"dry-run scenarios: {len(rows)}")
        print(f"first: {rows[0]['id']}")
        print(f"last: {rows[-1]['id']}")
        print(f"out: {path}")
        return
    n = write_scenarios(path)
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


def _run_one(scenario: dict, work: Path, timeout_s: float, resume: bool) -> str:
    run_dir = work / "runs" / scenario["id"]
    run_dir.mkdir(parents=True, exist_ok=True)
    record_path = run_dir / "record.json"
    if resume and record_path.is_file():
        print(f"skip {scenario['id']}")
        return "skip"
    (run_dir / "scenario.json").write_text(json.dumps(scenario), encoding="utf-8")
    (run_dir / "teacher.launch").write_text(launch_xml(scenario), encoding="utf-8")
    kill_py = _kill_helper(work)
    if not (resume and (run_dir / "raw_record.json").is_file()):
        remote = f"/teacher-runs/runs/{scenario['id']}"
        script = f"""#!/bin/bash
set +u
source /opt/ros/noetic/setup.bash
source /ego/devel/setup.bash
set -u
export PYTHONUNBUFFERED=1
python3 {kill_py}
rm -f {remote}/done.json {remote}/raw_record.json {remote}/roslaunch.log
roscore > {remote}/roscore.log 2>&1 &
for _i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25; do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.2
done
python3 -u /ws/scripts/teacher/ros_record.py --scenario {remote}/scenario.json --out {remote} --timeout {timeout_s} > {remote}/record.log 2>&1 &
rec=$!
sleep 1
if [ -f /tmp/linebuf.so ]; then
  LD_PRELOAD=/tmp/linebuf.so roslaunch {remote}/teacher.launch > {remote}/roslaunch.log 2>&1 &
else
  roslaunch {remote}/teacher.launch > {remote}/roslaunch.log 2>&1 &
fi
launch=$!
deadline=$((SECONDS + {int(timeout_s) + 25}))
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
        proc = subprocess.run(
            ["sudo", "-n", "docker", "exec", CONTAINER, "bash", f"{remote}/run.sh"],
        )
        if proc.returncode != 0 and not (run_dir / "raw_record.json").is_file():
            print(f"fail {scenario['id']} docker_exec={proc.returncode}", file=sys.stderr)
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
    success = record["metrics"]["success"]
    print(f"done {scenario['id']} status={record['run']['status']} success={success}")
    return "ok"


def cmd_run(args: argparse.Namespace) -> None:
    rows = _load_selected(args)
    if args.dry_run:
        print(f"dry-run run: {len(rows)}")
        if rows:
            print(f"first: {rows[0]['id']}")
            print(f"timeout_s: {args.timeout}")
        return
    if not (_dest() / "devel" / "setup.bash").is_file():
        _die("还没有编译。", "scripts/teacher/generate.sh build")
    _ensure_container(_dest(), Path(args.work))
    wall0 = time.time()
    for scenario in rows:
        _run_one(scenario, Path(args.work), float(args.timeout), not args.no_resume)
    elapsed = time.time() - wall0
    (Path(args.work) / "run-wall.json").write_text(
        json.dumps({"batch_wall_s": elapsed, "n": len(rows)}) + "\n",
        encoding="utf-8",
    )
    print(f"run_wall_s: {elapsed:.3f}")
    if elapsed > 0:
        print(f"scenarios_per_hour: {len(rows) / (elapsed / 3600.0):.3f}")


def _copy_sample(work: Path, sample_dir: Path, seed: int) -> int:
    if sample_dir.exists():
        shutil.rmtree(sample_dir)
    sample_dir.mkdir(parents=True)
    n = 0
    runs = work / "runs"
    if not runs.is_dir():
        return 0
    for record_path in sorted(runs.glob("*/record.json")):
        record = json.loads(record_path.read_text(encoding="utf-8"))
        if int(record["scenario"]["seed"]) != seed:
            continue
        dest = sample_dir / record["scenario"]["id"]
        dest.mkdir()
        for name in ("trajectory.npz", "record.json", "scene.json", "geometry.json"):
            src = record_path.parent / name
            if src.is_file():
                shutil.copy2(src, dest / name)
        n += 1
    schema = Path(__file__).resolve().parent / "schema.json"
    shutil.copy2(schema, sample_dir / "schema.json")
    manifest = {
        "schema": "yopo-teacher-record/1",
        "note": "小样本。全量在仓库外，用 scripts/teacher/generate.sh all 重跑。",
        "n": n,
        "seed": seed,
    }
    (sample_dir / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return n


def cmd_convert(args: argparse.Namespace) -> None:
    work = Path(args.work)
    sample = Path(args.sample_dir)
    if args.dry_run:
        print(f"dry-run convert sample_dir={sample} seed={args.sample_seed}")
        return
    n = _copy_sample(work, sample, int(args.sample_seed))
    print(f"sample_scenarios: {n}")
    print(f"sample_dir: {sample}")


def cmd_summarize(args: argparse.Namespace) -> None:
    work = Path(args.work)
    wall = None
    wall_path = work / "run-wall.json"
    if wall_path.is_file():
        wall = json.loads(wall_path.read_text(encoding="utf-8")).get("batch_wall_s")
    summary = summarize(work / "runs", batch_wall_s=wall)
    stamp = work / "build-stamp.json"
    if stamp.is_file():
        summary["build"] = json.loads(stamp.read_text(encoding="utf-8"))
    out = Path(args.out) if args.out else _REPO / "docs" / "research" / "d2-teacher-batch.json"
    if args.dry_run:
        print(format_table(summary))
        print(f"out: {out}")
        return
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(format_table(summary))
    print(f"out: {out}")


def cmd_all(args: argparse.Namespace) -> None:
    if args.dry_run:
        cmd_build(args)
        cmd_scenarios(args)
        cmd_run(args)
        cmd_convert(args)
        cmd_summarize(args)
        return
    cmd_build(args)
    cmd_scenarios(args)
    cmd_run(args)
    cmd_convert(args)
    cmd_summarize(args)


def _parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--work", default=str(_DEFAULT_WORK), help="运行目录，默认 /tmp/teacher-d2")
    common.add_argument("--dry-run", action="store_true", help="只打印将要做的事")

    parser = argparse.ArgumentParser(
        prog="scripts/teacher/generate.sh",
        description="在仓库外的 ego-planner-swarm 上批量生成教师轨迹，并写成 L0 场景卡。",
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    build = sub.add_parser("build", parents=[common], help="克隆并在容器里编译 92fe9f7")
    build.add_argument("--dir", default=None, help="克隆目录，默认 $EGO_SWARM_DIR 或 ~/ext/ego-planner-swarm")
    build.add_argument("--rebuild", action="store_true", help="即使已有 devel/setup.bash 也重编")
    build.set_defaults(func=cmd_build)

    scenarios = sub.add_parser("scenarios", parents=[common], help="写出 192 个场景的 jsonl")
    scenarios.add_argument("--out", default=None, help="默认 <work>/scenarios.jsonl")
    scenarios.set_defaults(func=cmd_scenarios)

    run = sub.add_parser("run", parents=[common], help="逐个无头跑教师并写成 npz")
    run.add_argument("--scenarios", default=None, help="场景 jsonl，默认 <work>/scenarios.jsonl")
    run.add_argument("--limit", type=int, default=None, help="只跑列表前 N 个")
    run.add_argument("--timeout", type=float, default=TIMEOUT_S, help=f"单场景秒数上限，默认 {TIMEOUT_S}")
    run.add_argument("--id", action="append", default=[], help="只跑这个场景 id，可重复")
    run.add_argument("--no-resume", action="store_true", help="已有 record.json 也重跑")
    run.set_defaults(func=cmd_run)

    convert = sub.add_parser("convert", parents=[common], help="把一个种子的运行拷进仓库作为小样本")
    convert.add_argument("--sample-dir", default=str(_DEFAULT_SAMPLE))
    convert.add_argument("--sample-seed", type=int, default=0)
    convert.set_defaults(func=cmd_convert)

    summary = sub.add_parser("summarize", parents=[common], help="按密度和机数汇总成功率")
    summary.add_argument("--out", default=None, help="默认 docs/research/d2-teacher-batch.json")
    summary.set_defaults(func=cmd_summarize)

    entire = sub.add_parser("all", parents=[common], help="build、生成场景、跑、拷小样本、汇总")
    entire.add_argument("--dir", default=None)
    entire.add_argument("--rebuild", action="store_true")
    entire.add_argument("--scenarios", default=None)
    entire.add_argument("--limit", type=int, default=192)
    entire.add_argument("--timeout", type=float, default=TIMEOUT_S)
    entire.add_argument("--id", action="append", default=[])
    entire.add_argument("--no-resume", action="store_true")
    entire.add_argument("--sample-dir", default=str(_DEFAULT_SAMPLE))
    entire.add_argument("--sample-seed", type=int, default=0)
    entire.add_argument("--out", default=None)
    entire.set_defaults(func=cmd_all)
    return parser


def main(argv: list[str] | None = None) -> None:
    parser = _parser()
    args = parser.parse_args(argv)
    if getattr(args, "limit", None) is not None and args.limit < 0:
        _die("--limit 需要非负整数。", "scripts/teacher/generate.sh all --limit 192")
    if getattr(args, "timeout", None) is not None and args.timeout < 0:
        _die("--timeout 需要非负秒数。", "scripts/teacher/generate.sh run --timeout 50")
    args.func(args)


if __name__ == "__main__":
    main()
