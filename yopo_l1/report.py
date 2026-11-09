"""同一张场景卡的 L0 目录和 L1 目录，打一张差异表。

列：到达、碰撞、拒绝率、起飞交接耗时、RTF。L0 没有后两项，格子是 —。
是否启动方向 B 交给 frank，这里只把数摆出来。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .errors import L1ReportError
from .launch_contract import EXECUTED_POLICY, POLICY_GAP


def _load(directory: Path) -> dict:
    path = Path(directory) / "result.json"
    if not path.is_file():
        raise L1ReportError(f"{directory} 里没有 result.json")
    return json.loads(path.read_text(encoding="utf-8"))


def _cell(x, nd=3) -> str:
    if x is None:
        return "—"
    if isinstance(x, str):
        return x
    return f"{float(x):.{nd}f}"


def _delta(a, b, nd=3) -> str:
    if a is None or b is None:
        return "—"
    return f"{float(b) - float(a):.{nd}f}"


def _arrived_cell(r: dict) -> str:
    if r.get("arrived") is None:
        return "—"
    return f"{r.get('arrived')}/{r.get('n_drones')}"


def _arrival_cell(r: dict) -> str:
    if r.get("arrival_time_s") is not None:
        return _cell(r["arrival_time_s"], 2)
    if r.get("task") == "hover":
        return "—"
    if r.get("outcome") == "SUCCESS" or r.get("outcome") is None:
        return "—"
    return "未到达"


def _collision(r: dict) -> int | None:
    if r.get("collisions") is not None:
        return int(r["collisions"])
    if "outcome" not in r or r.get("outcome") is None:
        return None
    return 1 if r["outcome"] == "COLLISION" else 0


def _policy_phrase(r: dict) -> str:
    pol = r.get("policy")
    req = r.get("policy_requested")
    if req and req != pol:
        return f"{pol}（请求 {req}，未在 L1 执行）"
    return str(pol)


def l0_vs_l1(l0_dir: str | Path, l1_dir: str | Path, out: str | Path | None = None) -> str:
    a = _load(Path(l0_dir))
    b = _load(Path(l1_dir))
    if a.get("scene_card_hash") != b.get("scene_card_hash") or not a.get("scene_card_hash"):
        raise L1ReportError(
            "不是同一张场景卡："
            f"L0 {a.get('scene')} {a.get('scene_card_hash')}，"
            f"L1 {b.get('scene')} {b.get('scene_card_hash')}"
        )
    if a.get("scene") and b.get("scene") and a["scene"] != b["scene"]:
        raise L1ReportError(f"场景卡名不一致：{a['scene']} 与 {b['scene']}")
    ha = a.get("scene_card_hash")
    col_a, col_b = _collision(a), _collision(b)
    arr_a, arr_b = a.get("arrival_time_s"), b.get("arrival_time_s")
    rej_a, rej_b = a.get("shadow_reject_rate"), b.get("shadow_reject_rate")
    hand_a, hand_b = a.get("takeoff_handoff_s"), b.get("takeoff_handoff_s")
    rtf_a, rtf_b = a.get("rtf"), b.get("rtf")
    wall_a, wall_b = a.get("wall_time_s"), b.get("wall_time_s")
    tr_a, tr_b = a.get("tracking_error_m"), b.get("tracking_error_m")
    lines = [
        f"# L0 / L1 对照：{a.get('scene')}",
        "",
        f"场景卡 `{ha}`。",
        f"L0 seed {a.get('seed')}，策略 {a.get('policy')}，影子盾 {'开' if a.get('shadow') else '关'}，层级 {a.get('tier')}。",
        f"L1 seed {b.get('seed')}，策略 {_policy_phrase(b)}，影子盾 {'开' if b.get('shadow') else '关'}，层级 {b.get('tier')}。",
        "",
        "| 指标 | L0 | L1 | 差 (L1−L0) |",
        "|---|---|---|---|",
        f"| 结果 | {a.get('outcome') or '—'} | {b.get('outcome') or '—'} | — |",
        f"| 到达时间 s | {_arrival_cell(a)} | {_arrival_cell(b)} | {_delta(arr_a, arr_b, 2)} |",
        f"| 到达数 | {_arrived_cell(a)} | {_arrived_cell(b)} | "
        f"{_delta(a.get('arrived'), b.get('arrived'), 0) if a.get('arrived') is not None and b.get('arrived') is not None else '—'} |",
        f"| 碰撞 | {_cell(col_a, 0)} | {_cell(col_b, 0)} | {_delta(col_a, col_b, 0)} |",
        f"| 跟踪误差 m | {_cell(tr_a, 3)} | {_cell(tr_b, 3)} | {_delta(tr_a, tr_b, 3)} |",
        f"| 影子拒绝率 | {_cell(rej_a, 3)} | {_cell(rej_b, 3)} | {_delta(rej_a, rej_b, 3)} |",
        f"| 起飞交接 s | {_cell(hand_a, 3)} | {_cell(hand_b, 3)} | {_delta(hand_a, hand_b, 3)} |",
        f"| 墙钟 s | {_cell(wall_a, 1)} | {_cell(wall_b, 1)} | {_delta(wall_a, wall_b, 1)} |",
        f"| RTF | {_cell(rtf_a, 3)} | {_cell(rtf_b, 3)} | {_delta(rtf_a, rtf_b, 3)} |",
        "",
    ]
    if b.get("policy") == EXECUTED_POLICY or b.get("policy_note"):
        lines += [
            "## L1 策略",
            "",
            str(b.get("policy_note") or POLICY_GAP),
            "",
        ]
    lines += [
        "## 方向 B",
        "",
        "差异已列出。是否启动方向 B（PX4 锁步多实例）交给 frank。本报告不代判。",
        "",
    ]
    text = "\n".join(lines)
    if out is not None:
        Path(out).write_text(text, encoding="utf-8")
    return text


def ensure_l0(scene: str, seed: int, dest: str | Path, *, policy: str = "upstream", shadow: bool = True, duration_s: float | None = None) -> Path:
    """在本机（x86 box）重跑 L0。Orin 上不要走这里，那里没有宿主 torch。"""
    from yopo_l0.runner import RunConfig, run, write
    from yopo_l0.scene import load_card

    dest = Path(dest)
    card = load_card(scene)
    write(run(card, RunConfig(seed=seed, policy=policy, shadow=shadow, duration_s=duration_s)), dest)
    return dest


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="同一张场景卡的 L0 / L1 对照表")
    ap.add_argument("--l0", default=None, help="已有的 L0 结果目录（里面有 result.json）")
    ap.add_argument("--scene", default=None, help="没有 --l0 时，在本机用这张卡重跑 L0（默认 seed 0、策略 upstream）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--policy", default="upstream", help="重跑 L0 时的策略。L1 不读这个值")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--no-shadow", action="store_true")
    ap.add_argument("--l0-out", default=None, help="重跑 L0 的输出目录，默认 out/l0-for-l1/<卡名>/seed<N>")
    ap.add_argument("--l1", required=True)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    l0 = a.l0
    if not l0:
        if not a.scene:
            ap.error("需要 --l0 结果目录，或 --scene 以便在 x86 上重跑 L0")
        from yopo_l0.scene import load_card

        name = load_card(a.scene).name
        l0 = a.l0_out or str(Path("out/l0-for-l1") / name / f"seed{a.seed}")
        ensure_l0(a.scene, a.seed, l0, policy=a.policy, shadow=not a.no_shadow, duration_s=a.duration)
    print(l0_vs_l1(l0, a.l1, a.out), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
