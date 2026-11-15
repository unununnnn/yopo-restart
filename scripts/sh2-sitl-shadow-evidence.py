#!/usr/bin/env python3
"""票 SH2：L0 g2/crossing 与锁步 SITL crossing-pillars-sitl80 的拒绝码归因。只报数。

默认关的那一臂不读场景几何。打开的一臂把场景卡光线投射交给影子盾，偏离上游。
不改判据，不改设定点。墙钟不进结论。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "sh2-sitl-shadow"
L0_SCENES = (
    ("g2-pillar", "scenes/g2-pillar.json", list(range(10))),
    ("crossing-pillars", "scenes/crossing-pillars.json", list(range(10))),
)
SITL_SCENE = "scenes/crossing-pillars-sitl80.json"
SITL_SEEDS = list(range(5))


def _traj_digest(ticks: list[dict]) -> str:
    h = hashlib.sha256()
    for t in ticks:
        blob = json.dumps(
            [t["uav"], t["p"], t["v"], t["a"], t["yaw"], t["cmd_p"], t["cmd_v"], t["cmd_a"]],
            separators=(",", ":"),
        )
        h.update(blob.encode())
    return h.hexdigest()


def _write(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(row, ensure_ascii=False, indent=2) + "\n")


def _collisions(result: dict, body: float) -> dict:
    gap = result.get("min_inter_drone_distance_m")
    obs = int(float(result["min_obstacle_distance_m"]) <= body + 1e-9)
    inter = int(gap is not None and float(gap) <= 2.0 * body + 1e-9)
    return {"obstacle": obs, "inter": inter, "outcome_collision": int(result["outcome"] == "COLLISION")}


def _arm_record(result: dict, ticks: list[dict], body: float) -> dict:
    from yopo_l0.shadow_sitl import attribution

    attr = attribution(ticks)
    return {
        "outcome": result["outcome"],
        "reason": result["reason"],
        "arrived": result["arrived"],
        "n_drones": result["n_drones"],
        "arrival_time_s": result["arrival_time_s"],
        "min_obstacle_distance_m": result["min_obstacle_distance_m"],
        "min_inter_drone_distance_m": result["min_inter_drone_distance_m"],
        "collisions": _collisions(result, body),
        "plan_codes": result.get("plan_codes"),
        "upstream_deviation": bool(result.get("upstream_deviation")),
        "shadow_env_evidence": bool(result.get("shadow_env_evidence", False)),
        "shadow_env_views": result.get("shadow_env_views"),
        "shadow_env_clearance_evals": result.get("shadow_env_clearance_evals"),
        "shadow_env_certificate_centers": result.get("shadow_env_certificate_centers"),
        "traj_sha256": _traj_digest(ticks),
        "attribution": attr,
        "by_uav": _by_uav(ticks),
    }


def _by_uav(ticks: list[dict]) -> dict[str, dict]:
    from yopo_l0.shadow_sitl import attribution

    ids = []
    for t in ticks:
        uid = str(t.get("uav"))
        if uid not in ids:
            ids.append(uid)
    return {uid: attribution([t for t in ticks if t.get("uav") == uid]) for uid in ids}


def run_l0_pair(scene_name: str, scene_path: str, seed: int) -> dict:
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    from yopo_l0.runner import RunConfig, run
    from yopo_l0.scene import load_card

    card = load_card(ROOT / scene_path)
    body = float(card.card["criteria"]["body_radius_m"])
    off = run(card, RunConfig(seed=seed, shadow=True, shadow_env_evidence=False))
    on = run(card, RunConfig(seed=seed, shadow=True, shadow_env_evidence=True))
    same_traj = _traj_digest(off.ticks) == _traj_digest(on.ticks)
    same_flight = (
        off.result["outcome"] == on.result["outcome"]
        and off.result["arrived"] == on.result["arrived"]
        and off.result["arrival_time_s"] == on.result["arrival_time_s"]
        and off.result["min_obstacle_distance_m"] == on.result["min_obstacle_distance_m"]
        and off.result["min_inter_drone_distance_m"] == on.result["min_inter_drone_distance_m"]
        and off.result["plan_codes"] == on.result["plan_codes"]
    )
    if not (same_traj and same_flight):
        raise RuntimeError(f"{scene_name} seed {seed} 两臂轨迹或到达/碰撞不一致")
    return {
        "layer": "l0",
        "scene": scene_name,
        "scene_path": scene_path,
        "scene_card_hash": card.hash,
        "seed": seed,
        "body_radius_m": body,
        "arms_match": True,
        "off": _arm_record(off.result, off.ticks, body),
        "on": _arm_record(on.result, on.ticks, body),
    }


def _aggregate_pairs(rows: list[dict]) -> dict:
    from collections import Counter

    from yopo_l0.shadow_sitl import CODE_CONDITIONS

    def merge(arm: str) -> dict:
        hist: Counter[str] = Counter()
        family: Counter[str] = Counter()
        ticks = reject = env_reject = env_only = non_env = 0
        arrivals = []
        would_rates = []
        env_rates = []
        covers = []
        for row in rows:
            a = row[arm]["attribution"]
            hist.update(a["histogram"])
            family.update(a["family_ticks"])
            ticks += a["ticks"]
            reject += a["would_reject"]
            env_reject += a["env_reject"]
            env_only += a["reject_only_env"]
            non_env += a["reject_with_non_env"]
            arrivals.append(row[arm]["arrival_time_s"])
            would_rates.append(a["would_reject_rate"])
            env_rates.append(a["env_reject_rate"])
            covers.append(a["non_env_covers_would_reject"])
        return {
            "seeds": len(rows),
            "ticks": ticks,
            "would_reject": reject,
            "would_reject_rate_min": min(would_rates) if would_rates else None,
            "would_reject_rate_max": max(would_rates) if would_rates else None,
            "env_reject": env_reject,
            "env_reject_rate_min": min(env_rates) if env_rates else None,
            "env_reject_rate_max": max(env_rates) if env_rates else None,
            "reject_only_env": env_only,
            "reject_with_non_env": non_env,
            "non_env_covers_all_seeds": all(covers) if covers else True,
            "histogram": dict(sorted(hist.items())),
            "family_ticks": dict(family),
            "conditions": {code: CODE_CONDITIONS[code] for code in hist if code in CODE_CONDITIONS},
            "arrival_s_min": min((a for a in arrivals if a is not None), default=None),
            "arrival_s_max": max((a for a in arrivals if a is not None), default=None),
        }

    return {"off": merge("off"), "on": merge("on")}


def cmd_l0(args) -> int:
    jobs = []
    for name, path, seeds in L0_SCENES:
        if args.scenes and name not in args.scenes:
            continue
        for seed in seeds:
            if args.seeds and seed not in args.seeds:
                continue
            jobs.append((name, path, seed))
    dest_dir = OUT / "l0"
    dest_dir.mkdir(parents=True, exist_ok=True)
    pending = []
    done_rows: list[dict] = []
    for name, path, seed in jobs:
        dest = dest_dir / f"{name}-seed{seed}.json"
        if dest.exists() and not args.force:
            print(f"skip {dest.name}", flush=True)
            done_rows.append(json.loads(dest.read_text()))
            continue
        pending.append((name, path, seed, dest))
    if pending:
        with ProcessPoolExecutor(max_workers=max(1, args.jobs)) as pool:
            futs = {pool.submit(run_l0_pair, n, p, s): (n, s, dest) for n, p, s, dest in pending}
            for fut in as_completed(futs):
                name, seed, dest = futs[fut]
                try:
                    row = fut.result()
                except Exception:
                    print(f"FAIL {name} seed {seed}\n{traceback.format_exc()}", flush=True)
                    raise
                _write(dest, row)
                done_rows.append(row)
                off_a, on_a = row["off"]["attribution"], row["on"]["attribution"]
                print(
                    f"done {name} seed {seed} would_reject {off_a['would_reject_rate']:.3f} -> "
                    f"{on_a['would_reject_rate']:.3f} env {off_a['env_reject_rate']:.3f} -> "
                    f"{on_a['env_reject_rate']:.3f} non_env_covers={on_a['non_env_covers_would_reject']}",
                    flush=True,
                )
    by_scene: dict[str, list] = {}
    for row in done_rows:
        by_scene.setdefault(row["scene"], []).append(row)
    summary = {
        "note": "票 SH2 L0 拒绝码归因。只报数。打开 shadow_env_evidence 偏离上游。",
        "scenes": {name: _aggregate_pairs(sorted(rows, key=lambda r: r["seed"])) for name, rows in by_scene.items()},
        "rows": sorted(done_rows, key=lambda r: (r["scene"], r["seed"])),
    }
    _write(OUT / "l0-summary.json", summary)
    print(f"L0 summary {OUT / 'l0-summary.json'}", flush=True)
    return 0


def _sitl_cmd(seed: int, evidence: bool, out_parent: Path) -> list[str]:
    cmd = [
        "bash",
        str(ROOT / "scripts/sitl/run_yopo_l0_sitl.sh"),
        "--n",
        "3",
        "--scene",
        SITL_SCENE,
        "--policy",
        "crossing",
        "--seed",
        str(seed),
        "--lockstep",
        "--fleet-shadow",
        "--image",
        "yopo-restart-sitl:yopo-81d084c04186",
    ]
    if evidence:
        cmd.append("--shadow-env-evidence")
    return cmd


def cmd_sitl(args) -> int:
    seeds = args.seeds if args.seeds else SITL_SEEDS
    arms = []
    if args.arm in ("off", "both"):
        arms.append(False)
    if args.arm in ("on", "both"):
        arms.append(True)
    dest_dir = OUT / "sitl"
    dest_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["YOPO_SITL_STATE"] = str(Path(args.sitl_state).resolve())
    env["YOPO_SITL_WALL_TIMEOUT"] = str(args.wall_timeout)
    for seed in seeds:
        for evidence in arms:
            tag = "on" if evidence else "off"
            dest = dest_dir / f"crossing-pillars-sitl80-seed{seed}-{tag}.json"
            if dest.exists() and not args.force:
                print(f"skip {dest.name}", flush=True)
                continue
            print(f"sitl seed {seed} {tag} start", flush=True)
            proc = subprocess.run(
                _sitl_cmd(seed, evidence, dest_dir),
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
                    parts = line.split()
                    run_dir = parts[1]
                    break
            src = None if run_dir is None else Path(run_dir) / "result.json"
            if src is None or not src.is_file():
                raise RuntimeError(
                    f"SITL seed {seed} {tag} 没有 result.json rc={proc.returncode} run_dir={run_dir}\n"
                    f"stdout tail:\n{(proc.stdout or '')[-2000:]}\nstderr tail:\n{(proc.stderr or '')[-2000:]}"
                )
            result = json.loads(src.read_text())
            # 任务失败（例如撞障碍）也留下影子直方图。基础设施没写出结果才中止。
            if proc.returncode != 0:
                print(
                    f"note sitl seed {seed} {tag} rc={proc.returncode} "
                    f"outcome={result.get('outcome')} reason={result.get('reason')}",
                    flush=True,
                )
            row = {
                "layer": "sitl-lockstep",
                "scene": "crossing-pillars-sitl80",
                "scene_path": SITL_SCENE,
                "seed": seed,
                "arm": tag,
                "shadow_env_evidence": bool(evidence),
                "run_dir": run_dir,
                "rc": proc.returncode,
                "result": result,
            }
            _write(dest, row)
            hist = (result.get("shadow_attribution") or {}).get("histogram") or result.get("shadow_reject_histogram")
            print(
                f"done sitl seed {seed} {tag} outcome={result.get('outcome')} "
                f"would_reject={result.get('shadow_reject_rate')} hist={hist}",
                flush=True,
            )
    return 0


def cmd_merge(_args) -> int:
    l0 = OUT / "l0-summary.json"
    sitl_rows = []
    for path in sorted((OUT / "sitl").glob("crossing-pillars-sitl80-seed*-*.json")):
        sitl_rows.append(json.loads(path.read_text()))
    sitl_pairs: dict[int, dict] = {}
    for row in sitl_rows:
        sitl_pairs.setdefault(int(row["seed"]), {})[row["arm"]] = row
    payload = {
        "note": "票 SH2：影子盾 ENV 证据接 SITL 与拒绝码归因。只报数。打开即偏离上游。",
        "generated": time.strftime("%Y-%m-%d %H:%M:%S UTC-3"),
        "l0": json.loads(l0.read_text()) if l0.exists() else None,
        "sitl_lockstep": {
            "scene": "crossing-pillars-sitl80",
            "scene_path": SITL_SCENE,
            "seeds": sorted(sitl_pairs),
            "pairs": sitl_pairs,
        },
        "code_conditions": None,
    }
    from yopo_l0.shadow_sitl import CODE_CONDITIONS

    payload["code_conditions"] = dict(CODE_CONDITIONS)
    dest = ROOT / "docs/research/sh2-sitl-shadow-evidence-runs.json"
    _write(dest, payload)
    _write(OUT / "merged.json", payload)
    print(f"merged {dest}", flush=True)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    l0 = sub.add_parser("l0", help="跑 L0 g2 与 crossing 两臂对照")
    l0.add_argument("--scenes", nargs="*", default=None)
    l0.add_argument("--seeds", nargs="*", type=int, default=None)
    l0.add_argument("--jobs", type=int, default=4)
    l0.add_argument("--force", action="store_true")
    l0.set_defaults(func=cmd_l0)
    sitl = sub.add_parser("sitl", help="跑锁步 SITL crossing-pillars-sitl80 seeds 0–4")
    sitl.add_argument("--seeds", nargs="*", type=int, default=None)
    sitl.add_argument("--arm", choices=("off", "on", "both"), default="both")
    sitl.add_argument("--force", action="store_true")
    sitl.add_argument("--wall-timeout", type=float, default=900.0)
    sitl.add_argument("--sitl-state", default="/workspace/yopo-ops/sh2/sitl")
    sitl.set_defaults(func=cmd_sitl)
    merge = sub.add_parser("merge", help="把 L0 与 SITL 行合成研究 JSON")
    merge.set_defaults(func=cmd_merge)
    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
