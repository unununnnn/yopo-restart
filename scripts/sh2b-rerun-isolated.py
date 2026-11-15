#!/usr/bin/env python3
"""票 SH2b：把被默认网桥串话污染的 4 趟 SH2 锁步 SITL，在独立 docker 网络上重跑。只报数。

原趟：/workspace/yopo-ops/sh2/sitl/runs 下
  20261115T163856（seed 3 关）、20261115T164011（seed 3 开）、
  20261115T164143（seed 4 关）、20261115T164247（seed 4 开）。
16:39:53–16:45:02 UTC-3 期间 B3 与 SH2 的容器同在默认网桥上，DDS/gz 互相发现。

重跑用同一镜像、同一命令，容器名 yopo-restart-sitl-sh2b，网络 yopo-sh2b-sitl。
不写 PX4 参数。只删本票起的容器和网络。
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

ROOT = Path(__file__).resolve().parents[1]
OPS = Path("/workspace/yopo-ops/sh2b")
STATE = OPS / "sitl"
OUT = ROOT / "out" / "sh2b-rerun-isolated"
DOC_RUNS = ROOT / "docs" / "research" / "sh2-sitl-shadow-evidence-runs.json"
NETWORK = "yopo-sh2b-sitl"
CONTAINER = "yopo-restart-sitl-sh2b"
IMAGE = "yopo-restart-sitl:yopo-81d084c04186"
SCENE = "scenes/crossing-pillars-sitl80.json"
JOBS = (
    (3, False, "20261115T163856-yopo-n3"),
    (3, True, "20261115T164011-yopo-n3"),
    (4, False, "20261115T164143-yopo-n3"),
    (4, True, "20261115T164247-yopo-n3"),
)


def _docker() -> list[str]:
    if subprocess.run(["docker", "info"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
        return ["docker"]
    return ["sudo", "-n", "docker"]


def _run(cmd: list[str], **kw):
    return subprocess.run(cmd, check=True, text=True, **kw)


def _write(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(row, ensure_ascii=False, indent=2) + "\n")


def ensure_network() -> None:
    docker = _docker()
    inspect = subprocess.run(docker + ["network", "inspect", NETWORK], capture_output=True, text=True)
    if inspect.returncode == 0:
        print(f"network {NETWORK} already exists", flush=True)
        return
    _run(docker + ["network", "create", "--driver", "bridge", NETWORK])
    print(f"created network {NETWORK}", flush=True)


def write_isolated_launcher() -> Path:
    """官方脚本在 main 上还没有自定容器名/网络（B3 尚未合入）。
    这里写一份只给本票重跑用的副本：容器名必须带网络，避免再走默认网桥。
    """
    src = (ROOT / "scripts/sitl/run_yopo_l0_sitl.sh").read_text()
    # 自定容器名，不管别人的缺省仿真是不是在跑。
    old = 'NAME="$(sitl_container_name "$N")"\nothers="$(sitl_running_names | grep -v "^${NAME}$" || true)"\n'
    new = (
        'NAME="$(sitl_container_name "$N")"\n'
        'if [[ -n "${YOPO_SITL_CONTAINER_NAME:-}" ]]; then\n'
        '  NAME="$YOPO_SITL_CONTAINER_NAME"\n'
        'fi\n'
        'others=""\n'
        '[[ -z "${YOPO_SITL_CONTAINER_NAME:-}" ]] && others="$(sitl_running_names | grep -v "^${NAME}$" || true)"\n'
    )
    if old not in src:
        raise SystemExit("run_yopo_l0_sitl.sh 的 NAME 段对不上，拒绝改写")
    src = src.replace(old, new, 1)
    old = '"${DOCKER[@]}" run --rm --name "$NAME" \\\n'
    new = (
        'if [[ -n "${YOPO_SITL_CONTAINER_NAME:-}" && -z "${YOPO_SITL_DOCKER_NETWORK:-}" ]]; then\n'
        '  echo "YOPO_SITL_CONTAINER_NAME 要和 YOPO_SITL_DOCKER_NETWORK 一起给：默认网桥上会和别的仿真串话" >&2\n'
        '  exit 2\n'
        'fi\n'
        'net_args=()\n'
        '[[ -n "${YOPO_SITL_DOCKER_NETWORK:-}" ]] && net_args=(--network "$YOPO_SITL_DOCKER_NETWORK")\n'
        '"${DOCKER[@]}" run --rm --name "$NAME" "${net_args[@]}" \\\n'
    )
    if old not in src:
        raise SystemExit("run_yopo_l0_sitl.sh 的 docker run 段对不上，拒绝改写")
    src = src.replace(old, new, 1)
    dest_dir = OPS / "sitl-scripts"
    dest_dir.mkdir(parents=True, exist_ok=True)
    # 启动脚本用 dirname 去 source common.sh，把官方副本放在一起。
    shutil.copy2(ROOT / "scripts/sitl/common.sh", dest_dir / "common.sh")
    dest = dest_dir / "run_yopo_l0_sitl.sh"
    dest.write_text(src)
    dest.chmod(0o755)
    return dest


def sitl_cmd(launcher: Path, seed: int, evidence: bool) -> list[str]:
    cmd = [
        "bash",
        str(launcher),
        "--n", "3",
        "--scene", SCENE,
        "--policy", "crossing",
        "--seed", str(seed),
        "--lockstep",
        "--fleet-shadow",
        "--image", IMAGE,
    ]
    if evidence:
        cmd.append("--shadow-env-evidence")
    return cmd


def _row_from_result(seed: int, evidence: bool, voided: str, run_dir: str, rc: int, result: dict) -> dict:
    attr = result.get("shadow_attribution") or {}
    hist = attr.get("histogram") or result.get("shadow_reject_histogram") or {}
    return {
        "seed": seed,
        "arm": "on" if evidence else "off",
        "shadow_env_evidence": bool(evidence),
        "voided_run": voided,
        "run_dir": run_dir,
        "rc": rc,
        "outcome": result.get("outcome"),
        "reason": result.get("reason"),
        "arrived": result.get("arrived"),
        "n_drones": result.get("n_drones"),
        "collisions": result.get("collisions"),
        "min_surface_clearance_plan_m": result.get("min_surface_clearance_plan_m"),
        "px4_parameters_written": result.get("px4_parameters_written"),
        "clock_ahead": (result.get("lockstep") or {}).get("clock_ahead"),
        "shadow_reject_rate": result.get("shadow_reject_rate"),
        "shadow_attribution": attr,
        "ticks": attr.get("ticks"),
        "env_reject_rate": attr.get("env_reject_rate"),
        "histogram": hist,
        "network": NETWORK,
        "container": CONTAINER,
    }


def cmd_run(args) -> int:
    ensure_network()
    launcher = write_isolated_launcher()
    STATE.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["YOPO_SITL_STATE"] = str(STATE)
    env["YOPO_SITL_WALL_TIMEOUT"] = str(args.wall_timeout)
    env["YOPO_SITL_CONTAINER_NAME"] = CONTAINER
    env["YOPO_SITL_DOCKER_NETWORK"] = NETWORK
    docker = _docker()
    rows = []
    for seed, evidence, voided in JOBS:
        tag = "on" if evidence else "off"
        dest = OUT / f"crossing-pillars-sitl80-seed{seed}-{tag}.json"
        if dest.exists() and not args.force:
            print(f"skip {dest.name}", flush=True)
            rows.append(json.loads(dest.read_text()))
            continue
        # 只动自己的容器。
        subprocess.run(docker + ["rm", "-f", CONTAINER], capture_output=True)
        print(f"sitl seed {seed} {tag} start (voided {voided})", flush=True)
        proc = subprocess.run(
            sitl_cmd(launcher, seed, evidence),
            cwd=ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=args.wall_timeout + 180,
        )
        log = dest.with_suffix(".log")
        log.write_text((proc.stdout or "") + "\n--- stderr ---\n" + (proc.stderr or ""))
        run_dir = None
        for line in (proc.stdout or "").splitlines():
            if line.startswith("run_dir "):
                run_dir = line.split()[1]
                break
        src = None if run_dir is None else Path(run_dir) / "result.json"
        if src is None or not src.is_file():
            raise RuntimeError(
                f"SITL seed {seed} {tag} 没有 result.json rc={proc.returncode} run_dir={run_dir}\n"
                f"stdout tail:\n{(proc.stdout or '')[-2000:]}\nstderr tail:\n{(proc.stderr or '')[-2000:]}"
            )
        result = json.loads(src.read_text())
        if result.get("px4_parameters_written"):
            raise RuntimeError(f"seed {seed} {tag} 写了 PX4 参数：{result.get('px4_parameters_written')}")
        if proc.returncode != 0:
            print(
                f"note sitl seed {seed} {tag} rc={proc.returncode} "
                f"outcome={result.get('outcome')} reason={result.get('reason')}",
                flush=True,
            )
        row = _row_from_result(seed, evidence, voided, run_dir, proc.returncode, result)
        _write(dest, row)
        rows.append(row)
        print(
            f"done sitl seed {seed} {tag} outcome={row['outcome']} "
            f"would_reject={row['shadow_reject_rate']} env={row['env_reject_rate']} "
            f"ticks={row['ticks']} hist={row['histogram']}",
            flush=True,
        )
    payload = {
        "note": "票 SH2b：4 趟被默认网桥串话污染的锁步 SITL，在 yopo-sh2b-sitl 上重跑。只报数。",
        "generated": time.strftime("%Y-%m-%d %H:%M:%S UTC-3"),
        "network": NETWORK,
        "container": CONTAINER,
        "image": IMAGE,
        "voided_window_utc3": "2026-11-15 16:39:53–16:45:02",
        "rows": rows,
    }
    _write(OUT / "summary.json", payload)
    print(json.dumps({"rows": [
        {k: r[k] for k in ("seed", "arm", "outcome", "reason", "arrived", "collisions",
                           "shadow_reject_rate", "env_reject_rate", "ticks", "run_dir")}
        for r in rows
    ]}, ensure_ascii=False, indent=2))
    return 0


def cmd_cleanup(_args) -> int:
    docker = _docker()
    rm = subprocess.run(docker + ["rm", "-f", CONTAINER], capture_output=True, text=True)
    print(f"rm container {CONTAINER} rc={rm.returncode}")
    # 只删本票网络。先确认上面没有别人的容器。
    inspect = subprocess.run(docker + ["network", "inspect", NETWORK], capture_output=True, text=True)
    if inspect.returncode != 0:
        print(f"network {NETWORK} already gone")
        return 0
    info = json.loads(inspect.stdout)[0]
    leftover = [c.get("Name") for c in (info.get("Containers") or {}).values()]
    if leftover:
        raise SystemExit(f"网络 {NETWORK} 上还有容器，不删：{leftover}")
    _run(docker + ["network", "rm", NETWORK])
    print(f"removed network {NETWORK}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="票 SH2b：隔离网络重跑被串话的 4 趟 SITL")
    sub = ap.add_subparsers(dest="cmd", required=True)
    run = sub.add_parser("run")
    run.add_argument("--wall-timeout", type=int, default=900)
    run.add_argument("--force", action="store_true")
    run.set_defaults(func=cmd_run)
    c = sub.add_parser("cleanup")
    c.set_defaults(func=cmd_cleanup)
    args = ap.parse_args(argv)
    return int(args.func(args) or 0)


if __name__ == "__main__":
    raise SystemExit(main())
