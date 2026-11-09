"""export_l1(card, RunConfig) -> 目录。

目录里是记录和 Orin 工单，不是一份新的 PX4 参数表：

- ``scene.json``：``sim/px4/g2-scene.json`` 的字段（world / goal_enu / boxes）
- ``run.json``：种子、请求的策略、实际会跑的 deployed-certified、每机起点终点、场景卡哈希
- ``job-card.json``：默认后端。调度器执行 ``scripts/run_g2.sh nominal``，种子经 ``YOPO_SIM_SEED``

显式 ``--backend ros2`` 且 launch 文件声明了 scene/seed/policy/shadow/start/goal 时，
才写 ``launch-args.txt``。Orin 上那份 uavs.launch.py 没有这些声明，不再因此失败。

不把 body_radius、tracking_margin、switch_delay、start_pad_radius 写进要执行的命令，
也不写任何 MPC_/NAV_ 参数。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from yopo_l0.runner import RunConfig, git_commit
from yopo_l0.scene import SceneCard, load_card

from .geometry import check_card
from .jobcard import build_job_card, write_job_card
from .launch_contract import (
    BACKEND_ROS2,
    BACKEND_RUN_G2,
    EXECUTED_POLICY,
    LAUNCH_FILE,
    LAUNCH_PATH,
    PACKAGE,
    POLICY_GAP,
    RUN_G2_MODE,
    RUN_G2_SCRIPT,
    SAFETY_SOURCE,
    SEED_ENV,
    WORLD_SCHEMA,
    bind_launch_args,
    declared_arguments,
    resolve_backend,
)


def _pose(p: list[float]) -> str:
    return ",".join(f"{float(x):.6g}" for x in p)


def _poses(rows: list[list[float]]) -> str:
    return ";".join(_pose(p) for p in rows)


def _dump(path: Path, obj) -> None:
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def export_l1(
    card: SceneCard,
    cfg: RunConfig,
    out_dir: str | Path | None = None,
    launch_file: str | Path | None = None,
    backend: str | None = None,
    spot_dir: str | Path | None = None,
) -> Path:
    spec = check_card(card, cfg.policy)
    out = Path(out_dir) if out_dir is not None else Path("out/l1") / card.name / f"seed{cfg.seed}"
    out.mkdir(parents=True, exist_ok=True)
    scene = {"world": spec["world"], "goal_enu": spec["goal_enu"], "boxes": spec["boxes"]}
    _dump(out / "scene.json", scene)
    scene_path = str((out / "scene.json").resolve())
    semantic = {
        "scene": scene_path,
        "seed": str(int(cfg.seed)),
        "policy": cfg.policy,
        "shadow": "on" if cfg.shadow else "off",
        "start_enu": _poses([m["start_enu"] for m in spec["missions"]]),
        "goal_enu": _poses([m["goal_enu"] for m in spec["missions"]]),
    }
    declared = None
    if launch_file is not None:
        declared = declared_arguments(Path(launch_file))
    chosen = resolve_backend(backend, declared)
    c = card.card
    duration = float(cfg.duration_s if cfg.duration_s is not None else c["task"]["duration_s"])
    drones = []
    for i, m in enumerate(spec["missions"]):
        drones.append({**m, "namespace": f"px4_{i + 1}"})
    run = {
        "schema": "yopo-restart/l1-run/1",
        "tier": "L1",
        "scene": card.name,
        "scene_card_hash": card.hash,
        "seed": int(cfg.seed),
        "policy": cfg.policy,
        "policy_requested": cfg.policy,
        "policy_executed": EXECUTED_POLICY if chosen == BACKEND_RUN_G2 else cfg.policy,
        "shadow": bool(cfg.shadow),
        "shadow_requested": bool(cfg.shadow),
        "shadow_executed": None if chosen == BACKEND_RUN_G2 else bool(cfg.shadow),
        "task": spec["task"],
        "duration_s": duration,
        "ground_z": 0.0,
        "criteria": c["criteria"],
        "drones": drones,
        "scene_file": "scene.json",
        "commit": git_commit(),
        "safety_note": "不改 g2_run.py 的安全清单数值，也不下发 PX4 param set",
    }
    launch_args_path = out / "launch-args.txt"
    job_card_path = out / "job-card.json"
    if chosen == BACKEND_ROS2:
        args = bind_launch_args(semantic, declared)
        run["launch"] = {
            "backend": BACKEND_ROS2,
            "package": PACKAGE,
            "launch_file": LAUNCH_FILE,
            "launch_path": LAUNCH_PATH,
            "world_schema": WORLD_SCHEMA,
            "safety_source": SAFETY_SOURCE,
            "args": args,
            "args_source": "declare" if declared is not None else "canonical",
        }
        launch_args_path.write_text("".join(f"{k}:={v}\n" for k, v in args.items()), encoding="utf-8")
        if job_card_path.exists():
            job_card_path.unlink()
    else:
        if launch_args_path.exists():
            launch_args_path.unlink()
        run["launch"] = {
            "backend": BACKEND_RUN_G2,
            "package": None,
            "launch_file": None,
            "launch_path": RUN_G2_SCRIPT,
            "world_schema": WORLD_SCHEMA,
            "safety_source": SAFETY_SOURCE,
            "args": {},
            "args_source": "run_g2",
            "mode": RUN_G2_MODE,
            "seed_env": SEED_ENV,
            "note": POLICY_GAP,
        }
        run["driver"] = {
            "backend": BACKEND_RUN_G2,
            "script": RUN_G2_SCRIPT,
            "mode": RUN_G2_MODE,
            "argv": [RUN_G2_MODE],
            "seed_env": SEED_ENV,
            "world_schema": WORLD_SCHEMA,
            "safety_source": SAFETY_SOURCE,
            "launch_binding": "skipped",
            "launch_file_declared": declared,
        }
    _dump(out / "run.json", run)
    if chosen == BACKEND_RUN_G2:
        card_obj = build_job_card(
            run,
            bundle_dir=out,
            spot_dir=Path(spot_dir) if spot_dir is not None else None,
            scene_path=card.path,
            declared=declared,
        )
        write_job_card(out / "job-card.json", card_obj)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="把 L0 场景卡导出成 Orin run_g2 抽检目录")
    ap.add_argument("--scene", required=True)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--policy", default="upstream", choices=["upstream", "certified"])
    ap.add_argument("--shadow", choices=("on", "off"), default="on")
    ap.add_argument("--no-shadow", action="store_true")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--spot", default=None, help="抽检根目录。工单里的日志目的地是 <spot>/log")
    ap.add_argument("--backend", choices=(BACKEND_RUN_G2, BACKEND_ROS2), default=None)
    ap.add_argument("--launch-file", default=None, help="uavs.launch.py；ros2 后端才按 DeclareLaunchArgument 对齐")
    a = ap.parse_args(argv)
    card = load_card(a.scene)
    out = export_l1(
        card,
        RunConfig(
            seed=a.seed,
            policy=a.policy,
            shadow=a.shadow == "on" and not a.no_shadow,
            duration_s=a.duration,
        ),
        a.out,
        a.launch_file,
        backend=a.backend,
        spot_dir=a.spot,
    )
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
