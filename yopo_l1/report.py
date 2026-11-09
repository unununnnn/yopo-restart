"""同一张场景卡的 L0 目录和 L1 目录，打一张差异表。

列：到达、碰撞、拒绝率、起飞交接耗时、RTF。L0 没有后两项，格子是 —。
是否启动方向 B 交给 frank，这里只把数摆出来。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .errors import L1ReportError


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
    if r.get("arrived") is None and r.get("n_drones") is None:
        return "—"
    return f"{r.get('arrived')}/{r.get('n_drones')}"


def _arrival_cell(r: dict) -> str:
    if r.get("arrival_time_s") is not None:
        return _cell(r["arrival_time_s"], 2)
    if r.get("task") == "hover":
        return "—"
    return "未到达"


def _collision(r: dict) -> int | None:
    if "outcome" not in r or r.get("outcome") is None:
        return None
    return 1 if r["outcome"] == "COLLISION" else 0


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
    lines = [
        f"# L0 / L1 对照：{a.get('scene')}",
        "",
        f"场景卡 `{ha}`。",
        f"L0 seed {a.get('seed')}，策略 {a.get('policy')}，影子盾 {'开' if a.get('shadow') else '关'}，层级 {a.get('tier')}。",
        f"L1 seed {b.get('seed')}，策略 {b.get('policy')}，影子盾 {'开' if b.get('shadow') else '关'}，层级 {b.get('tier')}。",
        "",
        "| 指标 | L0 | L1 | 差 (L1−L0) |",
        "|---|---|---|---|",
        f"| 到达时间 s | {_arrival_cell(a)} | {_arrival_cell(b)} | {_delta(arr_a, arr_b, 2)} |",
        f"| 到达数 | {_arrived_cell(a)} | {_arrived_cell(b)} | "
        f"{_delta(a.get('arrived'), b.get('arrived'), 0) if a.get('arrived') is not None and b.get('arrived') is not None else '—'} |",
        f"| 碰撞 | {_cell(col_a, 0)} | {_cell(col_b, 0)} | {_delta(col_a, col_b, 0)} |",
        f"| 影子拒绝率 | {_cell(rej_a, 3)} | {_cell(rej_b, 3)} | {_delta(rej_a, rej_b, 3)} |",
        f"| 起飞交接 s | {_cell(hand_a, 3)} | {_cell(hand_b, 3)} | {_delta(hand_a, hand_b, 3)} |",
        f"| RTF | {_cell(rtf_a, 3)} | {_cell(rtf_b, 3)} | {_delta(rtf_a, rtf_b, 3)} |",
        "",
        "## 方向 B",
        "",
        "差异已列出。是否启动方向 B（PX4 锁步多实例）交给 frank。本报告不代判。",
        "",
    ]
    text = "\n".join(lines)
    if out is not None:
        Path(out).write_text(text, encoding="utf-8")
    return text


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="同一张场景卡的 L0 / L1 对照表")
    ap.add_argument("--l0", required=True)
    ap.add_argument("--l1", required=True)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    print(l0_vs_l1(a.l0, a.l1, a.out), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
