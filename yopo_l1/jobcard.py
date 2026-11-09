"""Orin 调度器可以直接执行的抽检工单。

工单把 ``run_g2.sh nominal`` 的命令、环境、产物、超时和过/不过写死，
调度器不用猜 launch 参数。不改 PX4 参数，也不改 yopo-swarm。
"""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

from .launch_contract import (
    CONTAINERS,
    EXECUTED_POLICY,
    FROZEN_SAFETY,
    JOB_CARD_SCHEMA,
    JOB_TIMEOUT_S,
    LAUNCH_PATH,
    ORIN_LAUNCH_ARGS,
    POLICY_GAP,
    PYTHON_CHOICE,
    RESTART_REPO,
    RUN_G2_MODE,
    RUN_G2_SCRIPT,
    SAFETY_SOURCE,
    SEED_ENV,
    SWARM_REPO,
    WORLD_SCHEMA,
)

RESTART_ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN_PARAM_CMDS = ("param set", "param save", "param load", "param reset", "param import")


def swarm_head(root: str | Path | None = None) -> str | None:
    """只读。root 缺省时看 YOPO_SWARM_ROOT。不是 git 检出就返回 None。"""
    if root is None:
        root = os.environ.get("YOPO_SWARM_ROOT")
    if not root:
        return None
    try:
        proc = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    sha = proc.stdout.strip()
    return sha or None


def build_job_card(
    run: dict,
    *,
    bundle_dir: Path,
    spot_dir: Path | None = None,
    scene_path: str | None = None,
    declared: list[str] | None = None,
) -> dict:
    bundle_dir = Path(bundle_dir)
    spot = Path(spot_dir) if spot_dir is not None else bundle_dir.parent
    seed = int(run["seed"])
    requested = run.get("policy_requested", run.get("policy"))
    shadow_requested = bool(run.get("shadow_requested", run.get("shadow")))
    command = f'{SEED_ENV}={seed} bash "$YOPO_SWARM_ROOT/{RUN_G2_SCRIPT}" {RUN_G2_MODE}'
    scene_arg = scene_path or run.get("scene") or ""
    out_arg = str(spot)
    log_dir = str(spot / "log")
    return {
        "schema": JOB_CARD_SCHEMA,
        "ticket": "09b",
        "backend": "run_g2",
        "repo": SWARM_REPO,
        "restart_repo": RESTART_REPO,
        "restart_sha": run.get("commit"),
        "swarm_root_env": "YOPO_SWARM_ROOT",
        "swarm_sha": swarm_head(),
        "swarm_sha_command": 'git -C "$YOPO_SWARM_ROOT" rev-parse HEAD',
        "driver": RUN_G2_SCRIPT,
        "mode": RUN_G2_MODE,
        "argv": [RUN_G2_MODE],
        "cwd": "$YOPO_SWARM_ROOT",
        "command": command,
        "env": {SEED_ENV: str(seed)},
        "env_note": (
            "只追加 YOPO_SIM_SEED。不要设置 PX4 参数，不要 param set/save/load/reset/import，"
            "不要改 g2_run.py 或安全清单文件。"
        ),
        "forbidden_param_commands": list(FORBIDDEN_PARAM_CMDS),
        "policy_requested": requested,
        "policy_executed": EXECUTED_POLICY,
        "policy_gap": POLICY_GAP,
        "shadow_requested": shadow_requested,
        "shadow_executed": None,
        "shadow_note": "run_g2.sh 没有影子盾开关。不要把 L0 的影子盾当成已在 L1 运行。",
        "seed": seed,
        "scene": run.get("scene"),
        "scene_card_hash": run.get("scene_card_hash"),
        "scene_card_path": scene_arg,
        "bundle": {
            "dir": str(bundle_dir),
            "scene_json": str(bundle_dir / "scene.json"),
            "run_json": str(bundle_dir / "run.json"),
        },
        "geometry": {
            "world_schema": WORLD_SCHEMA,
            "note": (
                "scene.json 是记录，字段与 sim/px4/g2-scene.json 相同（world / goal_enu / boxes）。"
                "nominal 用栈里已有的 G2 世界，不把 scene/seed/policy/shadow/start_enu/goal_enu 传给 uavs.launch.py。"
            ),
        },
        "safety": {
            "source": SAFETY_SOURCE,
            "frozen": dict(FROZEN_SAFETY),
            "do_not_edit": ["scripts/g2_run.py", "*checklist*", "*safety*"],
            "do_not_pass": list(ORIN_LAUNCH_ARGS),
        },
        "uavs_launch_py": {
            "path": LAUNCH_PATH,
            "declared_on_orin": list(ORIN_LAUNCH_ARGS),
            "declared_in_file": list(declared) if declared is not None else None,
            "used": False,
        },
        "containers": list(CONTAINERS),
        "host_python": PYTHON_CHOICE,
        "python": {
            "needs": ["numpy"],
            "never": ["make venv", "pip install torch"],
            "fallback_image": "yopo-swarm:orin-infer-cu13",
        },
        "artifacts": {
            "search_root": "$YOPO_SWARM_ROOT",
            "newer_than": "run_g2.sh start",
            "result_names": ["result.json"],
            "event_names": ["events.jsonl", "events.json", "events.jsonl.gz"],
            "dest_dir": log_dir,
            "orin_result_name": "orin-result.json",
            "normalized_result": str(Path(log_dir) / "result.json"),
            "events_dir": str(Path(log_dir) / "events"),
            "guards": [
                str(spot / "guard" / "px4-params.before"),
                str(spot / "guard" / "px4-params.after"),
                str(spot / "guard" / "safety.before"),
                str(spot / "guard" / "safety.after"),
            ],
        },
        "guards": {
            "px4_param_dump": "只读，环境变量 PX4_PARAM_DUMP_CMD，标准输出即参数表",
            "safety_hash": "g2_run.py 以及文件名含 safety / checklist 的文本，sha256",
            "fail_on_any_change": True,
        },
        "scheduler": {
            "cwd": str(RESTART_ROOT),
            "argv": [
                "bash",
                str(RESTART_ROOT / "scripts" / "l1_spotcheck.sh"),
                "--scene",
                scene_arg,
                "--seed",
                str(seed),
                "--policy",
                str(requested),
                "--shadow",
                "on" if shadow_requested else "off",
                "--out",
                out_arg,
            ],
            "env": {
                "YOPO_SWARM_ROOT": "<orin 上的 yopo-swarm 检出>",
                "PX4_PARAM_DUMP_CMD": "<只读 param show，不要带 set/save/load/reset/import>",
            },
            "timeout_s": JOB_TIMEOUT_S,
        },
        "timeout_s": JOB_TIMEOUT_S,
        "pass_fail": {
            "pass": (
                "run_g2.sh 退出码 0；PX4 只读参数表前后一致；安全文件 sha256 前后一致；"
                "log/orin-result.json 已收下。"
            ),
            "fail": (
                "run_g2.sh 非 0，或参数/安全文件有差异，或没有收下 result.json，"
                "或命令里出现 param set/save/load/reset/import。"
            ),
        },
    }


def write_job_card(path: Path, card: dict) -> Path:
    path = Path(path)
    path.write_text(json.dumps(card, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path
