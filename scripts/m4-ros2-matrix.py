#!/usr/bin/env python3
"""票 M4-ROS2：formation-6-open 的 ROS 2 六档 × seeds 0–4。只报数。

进程内六档的对照读 docs/results/SUMMARY.json（与 SUMMARY.md 同一批）。
不改进程内冻结判据，不改场景卡。ROS 2 不在 gate_forms 里，这张矩阵不挡已冻门。

每粒单独起进程，写到 out/m4-ros2-matrix/partial/<tier>/seed<N>.json。
指纹（场景卡哈希、档位、sim_rev）相符就跳过，中断后可续跑。
不写 ticks.jsonl。
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PARTIAL = ROOT / "out" / "m4-ros2-matrix" / "partial"
RUNS_JSON = ROOT / "docs" / "research" / "m4-ros2-matrix-runs.json"
SCENE = "scenes/formation-6-open.json"
TIERS_FILE = "scenes/formation-6-tiers.json"
SEEDS = (0, 1, 2, 3, 4)
# 改了会影响轨迹或对账字段时再加。只改说明文字不要动它，否则续跑会把已落盘的粒丢掉。
SIM_REV = "m4-ros2-matrix-1"
DISCLAIMER = (
    "只报数。report-only。M4 formation-6-open 的 ROS 2 形态（票 10：L0 世界、锁步时钟、"
    "每机规划在 ROS 2 节点里，/clock 发仿真时间）。不改 core/、已冻门、criteria.frozen、"
    "scenes/suite.txt、场景卡默认值、README.md、scripts/premerge.sh、scripts/regress.sh、"
    "scripts/sitl/Dockerfile、docs/tickets.md。进程内六档已冻结，本矩阵不拿 ROS 2 的差去改那些门。"
    "未冻结，不进 scenes/suite.txt，不设 gates。"
)

# SUMMARY.md 的队形误差 p95、最小机间用三位小数；到达时间在 SUMMARY.json 里是两位（20.13）。
_P95_ND = 3
_INTER_ND = 3
_TIME_ND = 2
_RAW_ATOL = 1e-6


def _python() -> str:
    venv = ROOT / ".venv" / "bin" / "python"
    if venv.is_file():
        return str(venv)
    return sys.executable


def _env() -> dict[str, str]:
    env = os.environ.copy()
    extra = f"{ROOT}{os.pathsep}{ROOT / 'core' / 'src'}"
    old = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = extra if not old else extra + os.pathsep + old
    env.setdefault("OMP_NUM_THREADS", "1")
    env.setdefault("OPENBLAS_NUM_THREADS", "1")
    env.setdefault("MKL_NUM_THREADS", "1")
    return env


def _partial(tier: str, seed: int) -> Path:
    return PARTIAL / tier / f"seed{seed}.json"


def _load(path: Path) -> dict | None:
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text())
    except json.JSONDecodeError:
        return None
    return data if isinstance(data, dict) else None


def _atomic(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    tmp.replace(path)


def _scene_hash() -> str:
    sys.path.insert(0, str(ROOT))
    sys.path.insert(0, str(ROOT / "core" / "src"))
    from yopo_l0.scene import load_card

    return load_card(ROOT / SCENE).hash


def _usable(row: dict | None, scene_hash: str) -> bool:
    if not row or row.get("ok") is not True:
        return False
    if row.get("sim_rev") != SIM_REV:
        return False
    if row.get("scene_card_hash") != scene_hash:
        return False
    if row.get("form") != "ros2":
        return False
    if row.get("outcome") is None:
        return False
    return True


def _fmt(value: float | None, nd: int) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{nd}f}"


def _fmt_range(values: list[float], nd: int) -> str:
    if not values:
        return "—"
    lo, hi = min(values), max(values)
    if abs(hi - lo) < 0.5 * 10 ** (-nd):
        return f"{lo:.{nd}f}"
    return f"{lo:.{nd}f}–{hi:.{nd}f}"


def _near(a, b) -> bool:
    if a is None or b is None:
        return False
    return abs(float(a) - float(b)) <= _RAW_ATOL


def _baseline() -> dict[str, dict]:
    data = json.loads((ROOT / "docs" / "results" / "SUMMARY.json").read_text())
    found: dict[str, dict] = {}
    for row in data["rows"]:
        if row.get("scene") == "formation-6-open" and row.get("form") == "inprocess":
            found[str(row["fault_tier"])] = row
    return found


def _qos_note() -> dict:
    from yopo_l0.ros2_nodes import _CLOCK_QOS, _LOCKSTEP_QOS

    def one(profile) -> dict:
        return {
            "reliability": profile.reliability.name,
            "history": profile.history.name,
            "depth": int(profile.depth),
            "durability": profile.durability.name,
        }

    return {"clock": one(_CLOCK_QOS), "lockstep": one(_LOCKSTEP_QOS)}


def run_grain(tier_id: str, seed: int) -> int:
    os.chdir(ROOT)
    sys.path.insert(0, str(ROOT))
    sys.path.insert(0, str(ROOT / "core" / "src"))
    from yopo_l0.batch import resolve_tiers
    from yopo_l0.bus import BusFaults
    from yopo_l0.criteria import collision_count
    from yopo_l0.ros2_env import ensure_ros_paths
    from yopo_l0.ros2_run import execute
    from yopo_l0.runner import RunConfig
    from yopo_l0.scene import load_card

    distro = ensure_ros_paths()
    if distro is None:
        raise SystemExit("rclpy 不可导入。先运行 scripts/setup_ros2.sh")
    catalog = {str(t["id"]): t for t in resolve_tiers(str(ROOT / TIERS_FILE))}
    if tier_id not in catalog:
        raise SystemExit(f"未知档位 {tier_id}")
    tier = catalog[tier_id]
    card = load_card(ROOT / SCENE)
    bus = BusFaults.from_mapping(tier.get("bus"))
    cfg = RunConfig(seed=seed, policy="formation", shadow=True, bus=bus)
    info: dict = {}
    t0 = time.perf_counter()
    try:
        out = execute(card, cfg, inspect=info)
    except Exception as exc:
        _atomic(
            _partial(tier_id, seed),
            {
                "ok": False,
                "sim_rev": SIM_REV,
                "tier": tier_id,
                "seed": seed,
                "form": "ros2",
                "scene_card_hash": card.hash,
                "error": f"{type(exc).__name__}: {exc}",
                "traceback": traceback.format_exc(),
                "wall_time_s": time.perf_counter() - t0,
                "ros_distro": distro,
            },
        )
        return 1
    result = out.result
    formation = result.get("formation") or {}
    bus_counts = result.get("bus") or {}
    row = {
        "ok": True,
        "sim_rev": SIM_REV,
        "tier": tier_id,
        "tier_label": tier.get("label") or tier_id,
        "seed": seed,
        "form": "ros2",
        "policy": result.get("policy"),
        "shadow": result.get("shadow"),
        "tier_level": result.get("tier"),
        "commit": result.get("commit"),
        "scene": result.get("scene"),
        "scene_card_hash": result.get("scene_card_hash"),
        "ros_distro": distro,
        "outcome": result.get("outcome"),
        "reason": result.get("reason"),
        "arrived": result.get("arrived"),
        "n_drones": result.get("n_drones"),
        "arrival_rate": result.get("arrival_rate"),
        "arrival_time_s": result.get("arrival_time_s"),
        "collisions": collision_count(result),
        "outcome_collision": result.get("outcome") == "COLLISION",
        "obstacle_collisions": formation.get("obstacle_collisions"),
        "inter_drone_collisions": formation.get("inter_drone_collisions"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "formation_error_p95_m": result.get("formation_error_p95_m"),
        "min_obstacle_distance_m": result.get("min_obstacle_distance_m"),
        "sim_duration_s": result.get("sim_duration_s"),
        "wall_time_s": result.get("wall_time_s"),
        "shadow_reject_rate": result.get("shadow_reject_rate"),
        "bus": {
            "drop": bus_counts.get("drop"),
            "duplicate": bus_counts.get("duplicate"),
            "delay_s": bus_counts.get("delay_s"),
            "reorder": bus_counts.get("reorder"),
            "published": bus_counts.get("published"),
            "dropped": bus_counts.get("dropped"),
            "duplicated": bus_counts.get("duplicated"),
            "delivered": bus_counts.get("delivered"),
            "reordered": bus_counts.get("reordered"),
        },
        "bus_ros_deliveries": info.get("bus_ros_deliveries"),
        "clock_ok": info.get("clock_ok"),
        "clock_error": info.get("clock_error"),
        "clock_samples": info.get("clock_samples"),
        "local_commits": formation.get("local_commits"),
        "local_aborts": formation.get("local_aborts"),
        "nacks": formation.get("nacks"),
        "proposals": formation.get("proposals"),
        "acks": formation.get("acks"),
        "committed_t_switch_s": formation.get("committed_t_switch_s"),
        "committed_plan_hash": formation.get("committed_plan_hash"),
        "catchup_time_s": formation.get("catchup_time_s"),
        "qos": _qos_note(),
        "runner_wall_s": time.perf_counter() - t0,
    }
    _atomic(_partial(tier_id, seed), row)
    print(
        f"{tier_id} seed {seed}: {row['outcome']} arrived {row['arrived']}/{row['n_drones']} "
        f"t={row['arrival_time_s']} p95={row['formation_error_p95_m']} "
        f"inter={row['min_inter_drone_distance_m']} col={row['collisions']} "
        f"clock_ok={row['clock_ok']} wall={row['wall_time_s']:.1f}s",
        flush=True,
    )
    return 0


def _seed_vs(row: dict, base: dict) -> dict:
    p95_target = (base.get("formation_error_p95_m") or {}).get("min")
    time_target = (base.get("arrival_time_s") or {}).get("min")
    inter_target = base.get("min_inter_drone_distance_m")
    n_seeds = int(base["n_seeds"])
    arrived_target = int(base["arrived"]) // n_seeds
    drones_target = arrived_target  # 进程内每粒满到达，架数等于该粒到达数
    return {
        "arrived_ok": row.get("arrived") == arrived_target and row.get("n_drones") == drones_target,
        "collisions_ok": int(row.get("collisions") or 0) == 0 and int(base["collisions"]) == 0,
        "p95_display": _fmt(row.get("formation_error_p95_m"), _P95_ND),
        "p95_raw_ok": _near(row.get("formation_error_p95_m"), p95_target),
        "inter_display": _fmt(row.get("min_inter_drone_distance_m"), _INTER_ND),
        "inter_raw_ok": _near(row.get("min_inter_drone_distance_m"), inter_target),
        "arrival_time_display": _fmt(row.get("arrival_time_s"), _TIME_ND),
        "arrival_time_raw_ok": _near(row.get("arrival_time_s"), time_target),
        "inprocess_p95_m": p95_target,
        "inprocess_min_inter_m": inter_target,
        "inprocess_arrival_time_s": time_target,
        "inprocess_arrived_per_seed": arrived_target,
    }


def _aggregate(rows: list[dict], base: dict) -> dict:
    arrived = sum(int(r.get("arrived") or 0) for r in rows)
    slots = sum(int(r.get("n_drones") or 0) for r in rows)
    collisions = sum(int(r.get("collisions") or 0) for r in rows)
    p95s = [float(r["formation_error_p95_m"]) for r in rows if r.get("formation_error_p95_m") is not None]
    inters = [
        float(r["min_inter_drone_distance_m"])
        for r in rows
        if r.get("min_inter_drone_distance_m") is not None
    ]
    times = [float(r["arrival_time_s"]) for r in rows if r.get("arrival_time_s") is not None]
    base_p95 = base.get("formation_error_p95_m") or {}
    base_time = base.get("arrival_time_s") or {}
    p95_display = _fmt_range(p95s, _P95_ND)
    inter_display = _fmt(min(inters) if inters else None, _INTER_ND)
    time_display = _fmt_range(times, _TIME_ND)
    base_p95_display = _fmt_range(
        [float(base_p95["min"]), float(base_p95["max"])] if base_p95 else [],
        _P95_ND,
    )
    base_inter_display = _fmt(base.get("min_inter_drone_distance_m"), _INTER_ND)
    base_time_display = _fmt_range(
        [float(base_time["min"]), float(base_time["max"])] if base_time else [],
        _TIME_ND,
    )
    arrivals = f"{arrived}/{slots}"
    time_complete = len(times) == len(rows) and len(rows) == int(base["n_seeds"])
    return {
        "n": len(rows),
        "arrivals": arrivals,
        "arrivals_ok": arrivals == base.get("arrivals"),
        "collisions": collisions,
        "collisions_ok": collisions == int(base["collisions"]),
        "formation_error_p95_display": p95_display,
        "formation_error_p95_ok": p95_display == base_p95_display and len(p95s) == len(rows),
        "min_inter_display": inter_display,
        "min_inter_ok": inter_display == base_inter_display and len(inters) == len(rows),
        "arrival_time_display": time_display if time_complete else "—",
        "arrival_time_ok": time_complete and time_display == base_time_display,
        "inprocess_arrivals": base.get("arrivals"),
        "inprocess_collisions": int(base["collisions"]),
        "inprocess_p95_display": base_p95_display,
        "inprocess_min_inter_display": base_inter_display,
        "inprocess_arrival_time_display": base_time_display,
        "p95_raw_min": None if not p95s else min(p95s),
        "p95_raw_max": None if not p95s else max(p95s),
        "inter_raw_min": None if not inters else min(inters),
        "arrival_time_raw_min": None if not times else min(times),
        "arrival_time_raw_max": None if not times else max(times),
        "all_seeds_raw_ok": all(
            (r.get("vs_inprocess") or {}).get("p95_raw_ok")
            and (r.get("vs_inprocess") or {}).get("inter_raw_ok")
            and (r.get("vs_inprocess") or {}).get("arrival_time_raw_ok")
            and (r.get("vs_inprocess") or {}).get("arrived_ok")
            and (r.get("vs_inprocess") or {}).get("collisions_ok")
            for r in rows
        ),
    }


def assemble(path: Path | None = None) -> dict:
    scene_hash = _scene_hash()
    base = _baseline()
    sys.path.insert(0, str(ROOT))
    from yopo_l0.batch import resolve_tiers

    order = [str(t["id"]) for t in resolve_tiers(str(ROOT / TIERS_FILE))]
    rows = []
    missing = []
    failed = []
    for tier in order:
        for seed in SEEDS:
            row = _load(_partial(tier, seed))
            if not _usable(row, scene_hash):
                if row and row.get("ok") is False:
                    failed.append({
                        "tier": tier,
                        "seed": seed,
                        "error": row.get("error"),
                    })
                else:
                    missing.append({"tier": tier, "seed": seed})
                continue
            assert row is not None
            if tier not in base:
                raise SystemExit(f"SUMMARY.json 没有进程内档位 {tier}")
            row = dict(row)
            row["vs_inprocess"] = _seed_vs(row, base[tier])
            rows.append(row)
    by_tier: dict[str, list[dict]] = {tier: [] for tier in order}
    for row in rows:
        by_tier[row["tier"]].append(row)
    tiers = []
    for tier in order:
        got = by_tier[tier]
        if len(got) != len(SEEDS):
            cell = None
        else:
            cell = _aggregate(got, base[tier])
        tiers.append({
            "tier": tier,
            "label": (got[0]["tier_label"] if got else tier),
            "complete": len(got) == len(SEEDS),
            "cell": cell,
        })
    commits = sorted({r.get("commit") for r in rows if r.get("commit")})
    distros = sorted({r.get("ros_distro") for r in rows if r.get("ros_distro")})
    payload = {
        "title": "M4-ROS2：formation-6-open 六档 × seeds 0–4 的 ROS 2 矩阵",
        "disclaimer": DISCLAIMER,
        "report_only": True,
        "frozen": False,
        "sim_rev": SIM_REV,
        "scene": "formation-6-open",
        "scene_path": SCENE,
        "scene_card_hash": scene_hash,
        "policy": "formation",
        "shadow": True,
        "form": "ros2",
        "seeds": list(SEEDS),
        "tiers_file": TIERS_FILE,
        "baseline": {
            "summary_md": "docs/results/SUMMARY.md",
            "summary_json": "docs/results/SUMMARY.json",
            "summary_commit": "5524cab42584ea54df54bbf38fc77e26117066dd",
            "note": (
                "进程内六档来自 SUMMARY.json 里 form=inprocess 的 formation-6-open 行，"
                "与 SUMMARY.md 同一批。到达、碰撞、队形误差 p95、最小机间用 SUMMARY.md 的显示口径"
                "（p95 与机间三位小数，到达是架次比）。"
                "SUMMARY.md 表上没有到达时间列；到达时间用同一 json 的 arrival_time_s，两位小数。"
                "该 json 六档的 p95 与到达时间都是 min=max，所以逐种子对照使用这个共同值。"
                "最小机间对照的是各档 seeds 里的最小值，与总表一致。"
                "原始值差超过 1e-6 记为 raw 不一致，即使三位小数看起来相同。"
            ),
        },
        "commits": commits,
        "ros_distro": distros,
        "n_expected": len(order) * len(SEEDS),
        "n_ok": len(rows),
        "missing": missing,
        "failed": failed,
        "tiers": tiers,
        "rows": rows,
    }
    dest = path or RUNS_JSON
    dest.parent.mkdir(parents=True, exist_ok=True)
    _atomic(dest, payload)
    return payload


def _print_table(payload: dict) -> None:
    print(f"ok {payload['n_ok']}/{payload['n_expected']} missing={len(payload['missing'])} failed={len(payload['failed'])}")
    print("| 档位 | 到达 | 碰撞 | p95 | 最小机间 | 到达时间 | 原始值 |")
    for tier in payload["tiers"]:
        cell = tier.get("cell")
        if not cell:
            print(f"| {tier['tier']} | 未齐 | | | | | |")
            continue
        raw = "一致" if cell["all_seeds_raw_ok"] else "有种子与进程内原始值不同"
        flags = "".join(
            name
            for name, ok in (
                ("到达", cell["arrivals_ok"]),
                ("碰撞", cell["collisions_ok"]),
                ("p95", cell["formation_error_p95_ok"]),
                ("机间", cell["min_inter_ok"]),
                ("时间", cell["arrival_time_ok"]),
            )
            if not ok
        )
        mark = "格一致" if not flags else f"格不一致:{flags}"
        print(
            f"| {tier['tier']} | {cell['arrivals']} | {cell['collisions']} | "
            f"{cell['formation_error_p95_display']} | {cell['min_inter_display']} | "
            f"{cell['arrival_time_display']} | {mark}；{raw} |"
        )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="M4 ROS 2 六档矩阵，逐粒落盘可续跑")
    ap.add_argument("--grain", nargs=2, metavar=("TIER", "SEED"), help="只跑一粒")
    ap.add_argument("--no-resume", action="store_true", help="忽略已落盘的粒")
    ap.add_argument("--assemble", action="store_true", help="只根据 partial 写 runs json")
    ap.add_argument("--out", default=str(RUNS_JSON))
    ap.add_argument("--timeout-s", type=float, default=600.0)
    a = ap.parse_args(argv)
    if a.grain:
        return run_grain(a.grain[0], int(a.grain[1]))
    if a.assemble:
        payload = assemble(Path(a.out))
        _print_table(payload)
        print(f"写入 {a.out}")
        return 0 if not payload["missing"] and not payload["failed"] else 1

    scene_hash = _scene_hash()
    from yopo_l0.batch import resolve_tiers

    order = [str(t["id"]) for t in resolve_tiers(str(ROOT / TIERS_FILE))]
    pending: list[tuple[str, int]] = []
    for tier in order:
        for seed in SEEDS:
            path = _partial(tier, seed)
            row = None if a.no_resume else _load(path)
            if _usable(row, scene_hash):
                print(f"skip {tier} seed {seed}", flush=True)
                continue
            pending.append((tier, seed))
    py = _python()
    env = _env()
    rc = 0
    for index, (tier, seed) in enumerate(pending, start=1):
        print(f"run {index}/{len(pending)} {tier} seed {seed}", flush=True)
        try:
            proc = subprocess.run(
                [py, str(Path(__file__).resolve()), "--grain", tier, str(seed)],
                cwd=ROOT,
                env=env,
                timeout=a.timeout_s,
            )
            if proc.returncode != 0:
                rc = 1
                print(f"grain exit {proc.returncode}: {tier} seed {seed}", flush=True)
        except subprocess.TimeoutExpired:
            rc = 1
            _atomic(
                _partial(tier, seed),
                {
                    "ok": False,
                    "sim_rev": SIM_REV,
                    "tier": tier,
                    "seed": seed,
                    "form": "ros2",
                    "scene_card_hash": scene_hash,
                    "error": f"TimeoutExpired: {a.timeout_s:.0f}s",
                },
            )
            print(f"timeout {tier} seed {seed}", flush=True)
    payload = assemble(Path(a.out))
    _print_table(payload)
    print(f"写入 {a.out}")
    return rc if rc else (0 if not payload["missing"] and not payload["failed"] else 1)


if __name__ == "__main__":
    raise SystemExit(main())
