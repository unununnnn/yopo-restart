#!/usr/bin/env python3
"""票 SH5：锁步 SITL 上重测 SH3/SH4 影子对齐开关。只报数，不改默认。

四臂都开 fleet_shadow，以便记下影子归因（与票 SH2 关臂相同）。
三个研究开关按臂打开，默认仍关；打开的结果标「偏离上游」。
不改 0.15 m 阈值，不改设定点，不写 PX4 参数。墙钟不进结论。
SITL 用官方启动脚本的默认隔离网络。
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "sh5-sitl-shadow-align"
DOC_RUNS = ROOT / "docs" / "research" / "sh5-sitl-shadow-align-runs.json"
OPS = Path("/workspace/yopo-ops/sh5")
DEFAULT_STATE = OPS / "sitl"
IMAGE = "yopo-restart-sitl:yopo-81d084c04186"
PREMERGE_PID_DIR = Path("/workspace/premerge-repro-logs")
QUEUE_FILE = Path("/workspace/yopo-ops/queue.txt")
DAEMON_PID = Path("/workspace/yopo-ops/daemon.pid")

ARMS = (
    ("default", False, False, False),
    ("env", True, False, False),
    ("env_contract", True, True, False),
    ("env_contract_tracking", True, True, True),
)
ARM_FLAGS = {name: (ev, co, tr) for name, ev, co, tr in ARMS}

SCENES = {
    "crossing-pillars-sitl80": {
        "path": "scenes/crossing-pillars-sitl80.json",
        "policy": "crossing",
        "n": 3,
        "seeds": list(range(5)),
    },
    "g2-pillar-sitl80": {
        "path": "scenes/g2-pillar-sitl80.json",
        "policy": "upstream",
        "n": 1,
        "seeds": list(range(5)),
    },
    "formation-hold-sitl80": {
        "path": "scenes/formation-hold-sitl80.json",
        "policy": "formation",
        "n": 3,
        "seeds": list(range(5)),
    },
}

SITL_CONTAINER_PREFIXES = ("yopo-restart-sitl", "yopo-restart-s1-sitl")
HEAVY_CMD_MARKERS = (
    "scripts/regress.sh",
    "scripts/premerge.sh",
    "scripts/sitl/run_yopo_l0_sitl.sh",
    "yopo_l0_live.py",
)


def _write(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(row, ensure_ascii=False, indent=2) + "\n")


def docker_cmd() -> list[str]:
    if subprocess.run(["docker", "info"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
        return ["docker"]
    return ["sudo", "-n", "docker"]


def sitl_argv(scene: str, seed: int, arm: str, *, image: str = IMAGE) -> list[str]:
    info = SCENES[scene]
    evidence, contract, tracking = ARM_FLAGS[arm]
    cmd = [
        "bash",
        str(ROOT / "scripts/sitl/run_yopo_l0_sitl.sh"),
        "--n",
        str(info["n"]),
        "--scene",
        info["path"],
        "--policy",
        info["policy"],
        "--seed",
        str(seed),
        "--lockstep",
        "--fleet-shadow",
        "--image",
        image,
    ]
    if evidence:
        cmd.append("--shadow-env-evidence")
    if contract:
        cmd.append("--shadow-contract-align")
    if tracking:
        cmd.append("--shadow-tracking-align")
    return cmd


def row_dest(scene: str, seed: int, arm: str, dest_dir: Path | None = None) -> Path:
    base = dest_dir if dest_dir is not None else (OUT / "sitl")
    return base / f"{scene}-seed{seed}-{arm}.json"


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except (OSError, ProcessLookupError):
        return False


def _pid_cmdline(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return ""
    return raw.replace(b"\x00", b" ").decode("utf-8", "replace").strip()


def _alive_pidfile(path: Path) -> int | None:
    try:
        text = path.read_text().strip()
        pid = int(text.split()[0])
    except (OSError, ValueError, IndexError):
        return None
    return pid if _pid_alive(pid) else None


def _scan_heavy_procs() -> list[str]:
    found: list[str] = []
    proc = Path("/proc")
    daemon = _alive_pidfile(DAEMON_PID)
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if daemon is not None and pid == daemon:
            continue
        cmd = _pid_cmdline(pid)
        if not cmd:
            continue
        if any(mark in cmd for mark in HEAVY_CMD_MARKERS):
            found.append(f"pid {pid}: {cmd[:180]}")
    return found


def _sitl_containers() -> list[str]:
    proc = subprocess.run(
        docker_cmd() + ["ps", "--format", "{{.Names}}"],
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        return [f"docker ps 失败：{(proc.stderr or proc.stdout or '').strip()[:200]}"]
    names = []
    for line in proc.stdout.splitlines():
        name = line.strip()
        if name.startswith(SITL_CONTAINER_PREFIXES):
            names.append(name)
    return names


def busy_reasons() -> list[str]:
    reasons: list[str] = []
    if QUEUE_FILE.is_file() and QUEUE_FILE.read_text().strip():
        reasons.append(f"合入队列非空：{QUEUE_FILE}")
    if PREMERGE_PID_DIR.is_dir():
        for path in sorted(PREMERGE_PID_DIR.glob("*.pid")):
            pid = _alive_pidfile(path)
            if pid is not None:
                reasons.append(f"premerge 仍在跑：{path.name} pid={pid}")
    for path in sorted(Path("/workspace/yopo-ops").glob("*/regress.pid")):
        pid = _alive_pidfile(path)
        if pid is not None:
            cmd = _pid_cmdline(pid)
            if "regress.sh" in cmd or "premerge.sh" in cmd or not cmd:
                reasons.append(f"regress 仍在跑：{path} pid={pid}")
    for path in sorted(Path("/workspace/yopo-ops").glob("*/sitl.pid")):
        pid = _alive_pidfile(path)
        if pid is not None:
            reasons.append(f"SITL 记录进程仍在跑：{path} pid={pid}")
    reasons.extend(f"重负载进程 {item}" for item in _scan_heavy_procs())
    for name in _sitl_containers():
        if name.startswith("docker ps"):
            reasons.append(name)
        else:
            reasons.append(f"已有 SITL 容器：{name}")
    return reasons


def wait_until_clear(*, max_wait_s: float = 300.0, poll_s: float = 15.0) -> list[str]:
    """最多等 max_wait_s。清空则返回空列表，超时则返回仍在的原因。"""
    deadline = time.monotonic() + max_wait_s
    while True:
        reasons = busy_reasons()
        if not reasons:
            return []
        remain = deadline - time.monotonic()
        if remain <= 0:
            return reasons
        print(f"机器忙，等待 {min(poll_s, remain):.0f}s：{reasons}", flush=True)
        time.sleep(min(poll_s, remain))


def compact_row(
    *,
    scene: str,
    seed: int,
    arm: str,
    run_dir: str | None,
    rc: int,
    result: dict,
    network: str | None = None,
) -> dict:
    evidence, contract, tracking = ARM_FLAGS[arm]
    attr = result.get("shadow_attribution") or {}
    hist = attr.get("histogram") or result.get("shadow_reject_histogram") or {}
    lock = result.get("lockstep") or {}
    ticks = attr.get("ticks")
    would = attr.get("would_reject")
    rate = attr.get("would_reject_rate", result.get("shadow_reject_rate"))
    # shadow_reject_rate 走 shadow.aggregate，只看 tick.would_reject，是跟踪对齐下的准数。
    official = result.get("shadow_reject_rate")
    if official is not None and ticks:
        rate = float(official)
        would = int(round(rate * int(ticks)))
    elif would is None and ticks is not None and rate is not None:
        would = int(round(float(rate) * int(ticks)))
    return {
        "layer": "sitl-lockstep",
        "scene": scene,
        "scene_path": SCENES[scene]["path"],
        "policy": SCENES[scene]["policy"],
        "n": SCENES[scene]["n"],
        "seed": seed,
        "arm": arm,
        "fleet_shadow": True,
        "shadow_env_evidence": bool(evidence),
        "shadow_contract_align": bool(contract),
        "shadow_tracking_align": bool(tracking),
        "run_dir": run_dir,
        "network": network,
        "image": IMAGE,
        "rc": rc,
        "outcome": result.get("outcome"),
        "reason": result.get("reason"),
        "arrived": result.get("arrived"),
        "n_drones": result.get("n_drones"),
        "collisions": result.get("collisions"),
        "min_surface_clearance_plan_m": result.get("min_surface_clearance_plan_m"),
        "px4_parameters_written": result.get("px4_parameters_written"),
        "clock_ahead": lock.get("clock_ahead"),
        "upstream_deviation": bool(result.get("upstream_deviation")),
        "result_shadow_env_evidence": bool(result.get("shadow_env_evidence", False)),
        "result_shadow_contract_align": bool(result.get("shadow_contract_align", False)),
        "result_shadow_tracking_align": bool(result.get("shadow_tracking_align", False)),
        "ticks": ticks,
        "would_reject": would,
        "would_reject_rate": rate,
        "env_reject": attr.get("env_reject"),
        "env_reject_rate": attr.get("env_reject_rate"),
        "histogram": hist,
        "family_ticks": attr.get("family_ticks"),
        "tracking_bound_count": int(hist.get("TRACKING_BOUND", 0) or 0),
    }


def parse_launcher_output(stdout: str) -> tuple[str | None, str | None]:
    run_dir = None
    network = None
    for line in (stdout or "").splitlines():
        if line.startswith("run_dir "):
            run_dir = line.split()[1]
        elif line.startswith("sitl_network "):
            network = line.split()[1]
    return run_dir, network


def aggregate_rows(rows: list[dict]) -> dict:
    ticks = sum(int(r["ticks"] or 0) for r in rows)
    would = sum(int(r["would_reject"] or 0) for r in rows)
    env = sum(int(r["env_reject"] or 0) for r in rows)
    tracking = sum(int(r.get("tracking_bound_count") or 0) for r in rows)
    arrived_trips = sum(1 for r in rows if r.get("outcome") == "SUCCESS")
    arrived_drones = sum(int(r["arrived"] or 0) for r in rows)
    drones = sum(int(r["n_drones"] or 0) for r in rows)
    collisions = sum(int(r["collisions"] or 0) for r in rows)
    hist: dict[str, int] = {}
    for r in rows:
        for code, n in (r.get("histogram") or {}).items():
            hist[code] = hist.get(code, 0) + int(n)
    return {
        "trips": len(rows),
        "success_trips": arrived_trips,
        "arrived_drones": arrived_drones,
        "drones": drones,
        "collisions": collisions,
        "ticks": ticks,
        "would_reject": would,
        "would_reject_rate": None if ticks == 0 else would / ticks,
        "env_reject": env,
        "env_reject_rate": None if ticks == 0 else env / ticks,
        "tracking_bound_count": tracking,
        "histogram": dict(sorted(hist.items())),
    }


def collect_rows(dest_dir: Path | None = None) -> list[dict]:
    base = dest_dir if dest_dir is not None else (OUT / "sitl")
    if not base.is_dir():
        return []
    rows = []
    for path in sorted(base.glob("*-seed*-*.json")):
        if path.name.endswith("-summary.json"):
            continue
        rows.append(json.loads(path.read_text()))
    rows.sort(key=lambda r: (r.get("scene") or "", int(r.get("seed") or 0), r.get("arm") or ""))
    return rows


def build_payload(rows: list[dict]) -> dict:
    by_scene: dict[str, dict] = {}
    for row in rows:
        scene = row["scene"]
        slot = by_scene.setdefault(scene, {"scene": scene, "seeds": set(), "arms": {}})
        slot["seeds"].add(int(row["seed"]))
        slot["arms"].setdefault(row["arm"], []).append(row)
    scenes = {}
    for scene, slot in by_scene.items():
        scenes[scene] = {
            "scene": scene,
            "scene_path": SCENES[scene]["path"],
            "policy": SCENES[scene]["policy"],
            "n": SCENES[scene]["n"],
            "seeds": sorted(slot["seeds"]),
            "arms": {
                arm: {"rows": sorted(items, key=lambda r: int(r["seed"])), "summary": aggregate_rows(items)}
                for arm, items in slot["arms"].items()
            },
        }
    return {
        "note": "票 SH5：锁步 SITL 上重测 SH3/SH4 影子对齐开关。只报数。打开研究开关偏离上游。不放宽 0.15 m。",
        "generated": time.strftime("%Y-%m-%d %H:%M:%S UTC-3"),
        "image": IMAGE,
        "arms": [
            {
                "arm": name,
                "fleet_shadow": True,
                "shadow_env_evidence": ev,
                "shadow_contract_align": co,
                "shadow_tracking_align": tr,
            }
            for name, ev, co, tr in ARMS
        ],
        "scenes": scenes,
        "rows": rows,
    }


def write_runs(rows: list[dict], dest: Path | None = None) -> dict:
    payload = build_payload(rows)
    path = dest if dest is not None else DOC_RUNS
    _write(path, payload)
    _write(OUT / "merged.json", payload)
    return payload


def cmd_sitl(args) -> int:
    scenes = list(args.scenes) if args.scenes else ["crossing-pillars-sitl80"]
    for scene in scenes:
        if scene not in SCENES:
            raise SystemExit(f"未知场景 {scene}，可选：{', '.join(SCENES)}")
    arms = [name for name, ev, co, tr in ARMS if args.arm in ("all", name)]
    if not arms:
        raise SystemExit(f"未知臂 {args.arm}")
    dest_dir = OUT / "sitl"
    dest_dir.mkdir(parents=True, exist_ok=True)
    DEFAULT_STATE.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["YOPO_SITL_STATE"] = str(Path(args.sitl_state).resolve())
    env["YOPO_SITL_WALL_TIMEOUT"] = str(int(args.wall_timeout))
    env.pop("YOPO_SITL_CONTAINER_NAME", None)
    env.pop("YOPO_SITL_DOCKER_NETWORK", None)
    # 用默认隔离网络，不自定容器名。
    for scene in scenes:
        info = SCENES[scene]
        seeds = args.seeds if args.seeds else list(info["seeds"])
        for seed in seeds:
            for arm in arms:
                dest = row_dest(scene, seed, arm, dest_dir)
                if dest.exists() and not args.force:
                    print(f"skip {dest.name}", flush=True)
                    continue
                reasons = wait_until_clear(max_wait_s=args.wait_s)
                if reasons:
                    raise SystemExit(f"等待超过 {args.wait_s:.0f}s，仍有负载，本趟未开：{reasons}")
                print(f"sitl {scene} seed {seed} {arm} start", flush=True)
                proc = subprocess.run(
                    sitl_argv(scene, seed, arm, image=args.image),
                    cwd=ROOT,
                    env=env,
                    capture_output=True,
                    text=True,
                    timeout=float(args.wall_timeout) + 180,
                )
                log = dest.with_suffix(".log")
                log.write_text((proc.stdout or "") + "\n--- stderr ---\n" + (proc.stderr or ""))
                run_dir, network = parse_launcher_output(proc.stdout or "")
                src = None if run_dir is None else Path(run_dir) / "result.json"
                if src is None or not src.is_file():
                    raise RuntimeError(
                        f"SITL {scene} seed {seed} {arm} 没有 result.json "
                        f"rc={proc.returncode} run_dir={run_dir}\n"
                        f"stdout tail:\n{(proc.stdout or '')[-2000:]}\n"
                        f"stderr tail:\n{(proc.stderr or '')[-2000:]}"
                    )
                result = json.loads(src.read_text())
                written = result.get("px4_parameters_written")
                if written:
                    raise RuntimeError(f"{scene} seed {seed} {arm} 写了 PX4 参数：{written}")
                if proc.returncode != 0:
                    print(
                        f"note {scene} seed {seed} {arm} rc={proc.returncode} "
                        f"outcome={result.get('outcome')} reason={result.get('reason')}",
                        flush=True,
                    )
                row = compact_row(
                    scene=scene,
                    seed=seed,
                    arm=arm,
                    run_dir=run_dir,
                    rc=proc.returncode,
                    result=result,
                    network=network,
                )
                _write(dest, row)
                write_runs(collect_rows(dest_dir))
                print(
                    f"done {scene} seed {seed} {arm} outcome={row['outcome']} "
                    f"arrived={row['arrived']}/{row['n_drones']} collisions={row['collisions']} "
                    f"would_reject={row['would_reject_rate']} env={row['env_reject_rate']} "
                    f"TRACKING_BOUND={row['tracking_bound_count']} ticks={row['ticks']}",
                    flush=True,
                )
    return 0


def cmd_merge(_args) -> int:
    rows = collect_rows()
    payload = write_runs(rows)
    for scene, slot in payload["scenes"].items():
        print(f"## {scene}", flush=True)
        for arm, ev, co, tr in ARMS:
            if arm not in slot["arms"]:
                continue
            s = slot["arms"][arm]["summary"]
            wr = s["would_reject_rate"]
            er = s["env_reject_rate"]
            print(
                f"{arm}: trips {s['success_trips']}/{s['trips']} "
                f"arrived {s['arrived_drones']}/{s['drones']} collisions={s['collisions']} "
                f"would_reject={None if wr is None else round(wr, 4)} "
                f"env={None if er is None else round(er, 4)} "
                f"TRACKING_BOUND={s['tracking_bound_count']} ticks={s['ticks']}",
                flush=True,
            )
    print(f"merged {DOC_RUNS} rows={len(rows)}", flush=True)
    return 0


def cmd_busy(_args) -> int:
    reasons = busy_reasons()
    if not reasons:
        print("clear")
        return 0
    for item in reasons:
        print(item)
    return 1


def cmd_cleanup(_args) -> int:
    docker = docker_cmd()
    # 只清本票默认容器名。不碰 ego-teacher-d2b-build，不扫别人的网络。
    for name in ("yopo-restart-sitl-n3", "yopo-restart-sitl-n1", "yopo-restart-s1-sitl"):
        proc = subprocess.run(docker + ["rm", "-f", name], capture_output=True, text=True)
        print(f"rm container {name} rc={proc.returncode}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sitl = sub.add_parser("sitl", help="跑锁步 SITL 四臂对照")
    sitl.add_argument("--scenes", nargs="*", default=None, choices=sorted(SCENES))
    sitl.add_argument("--seeds", nargs="*", type=int, default=None)
    sitl.add_argument("--arm", choices=("all",) + tuple(name for name, *_ in ARMS), default="all")
    sitl.add_argument("--force", action="store_true")
    sitl.add_argument("--wall-timeout", type=float, default=900.0)
    sitl.add_argument("--wait-s", type=float, default=300.0)
    sitl.add_argument("--sitl-state", default=str(DEFAULT_STATE))
    sitl.add_argument("--image", default=IMAGE)
    sitl.set_defaults(func=cmd_sitl)
    merge = sub.add_parser("merge", help="把已跑行合成研究 JSON")
    merge.set_defaults(func=cmd_merge)
    busy = sub.add_parser("busy", help="检查是否有别人的 SITL / 回归 / premerge")
    busy.set_defaults(func=cmd_busy)
    cleanup = sub.add_parser("cleanup", help="只删本票默认容器名")
    cleanup.set_defaults(func=cmd_cleanup)
    args = ap.parse_args(argv)
    return int(args.func(args) or 0)


if __name__ == "__main__":
    raise SystemExit(main())
