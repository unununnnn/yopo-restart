"""export_l1(card, RunConfig) -> 目录。

目录里是 yopo-swarm SITL 启动要吃的东西，不是一份新的 PX4 参数表：

- ``scene.json``：``sim/px4/g2-scene.json`` 的字段（world / goal_enu / boxes）
- ``run.json``：种子、策略、影子盾、每机起点终点、场景卡哈希。给 import_l1 对回这张卡
- ``launch-args.txt``：``key:=value``，一行一个，交给 ``ros2 launch yopo_swarm_px4 uavs.launch.py``

不写 body_radius、tracking_margin、switch_delay、start_pad_radius，也不写任何 MPC_/NAV_ 参数。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from yopo_l0.runner import RunConfig, git_commit
from yopo_l0.scene import SceneCard, load_card

from .geometry import check_card
from .launch_contract import LAUNCH_FILE, LAUNCH_PATH, PACKAGE, SAFETY_SOURCE, WORLD_SCHEMA, bind_launch_args, declared_arguments


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
    source = "canonical"
    if launch_file is not None:
        declared = declared_arguments(Path(launch_file))
        source = "declare"
    args = bind_launch_args(semantic, declared)
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
        "shadow": bool(cfg.shadow),
        "task": spec["task"],
        "duration_s": duration,
        "ground_z": 0.0,
        "criteria": c["criteria"],
        "drones": drones,
        "scene_file": "scene.json",
        "commit": git_commit(),
        "launch": {
            "package": PACKAGE,
            "launch_file": LAUNCH_FILE,
            "launch_path": LAUNCH_PATH,
            "world_schema": WORLD_SCHEMA,
            "safety_source": SAFETY_SOURCE,
            "args": args,
            "args_source": source,
        },
        "safety_note": "不改 g2_run.py 的安全清单数值，也不下发 PX4 param set",
    }
    _dump(out / "run.json", run)
    (out / "launch-args.txt").write_text("".join(f"{k}:={v}\n" for k, v in args.items()), encoding="utf-8")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="把 L0 场景卡导出成 yopo-swarm SITL 启动目录")
    ap.add_argument("--scene", required=True)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--policy", default="upstream", choices=["upstream", "certified"])
    ap.add_argument("--shadow", choices=("on", "off"), default="on")
    ap.add_argument("--no-shadow", action="store_true")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--launch-file", default=None, help="uavs.launch.py；给出就按 DeclareLaunchArgument 对齐参数名")
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
    )
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
