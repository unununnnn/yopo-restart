"""把 scripts/sitl/smoke.sh 一次运行的 result.json 摘成一张 Markdown 表（report-only）。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def fmt(v, nd=3):
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def render(res: dict, total_wall: float | None, image: str | None) -> str:
    m = res.get("metrics") or {}
    rows = [
        ("结果", "通过" if res.get("ok") else f"未完成（{m.get('failure') or res.get('phase')}）"),
        ("悬停高度误差 RMS (m)", fmt(m.get("hover_alt_err_rms_m"))),
        ("悬停高度误差最大 (m)", fmt(m.get("hover_alt_err_max_m"))),
        ("悬停水平漂移最大 (m)", fmt(m.get("hover_horiz_drift_max_m"))),
        ("到点误差 (m)", fmt(m.get("goto_err_m"))),
        ("悬停高度误差 RMS / 最大，Gazebo 真值 (m)", f"{fmt(m.get('hover_alt_err_truth_rms_m'))} / {fmt(m.get('hover_alt_err_truth_max_m'))}"),
        ("到点误差，Gazebo 真值 (m)", fmt(m.get("goto_err_truth_m"))),
        ("估计相对真值的高度偏差 均值 / RMS / 最大 (m)",
         f"{fmt(m.get('hover_est_truth_alt_bias_mean_m'))} / {fmt(m.get('hover_est_truth_alt_bias_rms_m'))} / {fmt(m.get('hover_est_truth_alt_bias_max_m'))}"),
        ("起飞 / 到点 / 降落用时（仿真 s）", " / ".join(fmt(m.get(k), 1) for k in ("takeoff_s", "goto_s", "land_s"))),
        ("任务墙钟 (s)", fmt(res.get("mission_wall_s"), 1)),
        ("任务仿真时长 (s)", fmt(res.get("mission_sim_s"), 1)),
        ("RTF", fmt(res.get("rtf"), 2)),
        ("启动到预检通过墙钟 (s)", fmt(res.get("wall_to_preflight_ok_s"), 1)),
        ("总墙钟含容器 (s)", fmt(total_wall, 1)),
    ]
    pos = res.get("positioning") or "default"
    out = ["## S1/S1b 单机 PX4 SITL 冒烟（report-only，判据未冻结）", "",
           f"定位源：`{pos}`（default = 官方米级 GNSS 模型；rtk = 厘米级，默认关）。",
           "误差默认相对 PX4 自身估计（VehicleOdometry）；标「Gazebo 真值」的行含估计器误差。",
           "「估计相对真值的高度偏差」是悬停段 est_z − truth_z（同一世界系）。", ""]
    if image:
        out += [f"镜像：`{image}`", ""]
    out += ["| 项 | 值 |", "|---|---|"] + [f"| {k} | {v} |" for k, v in rows]
    return "\n".join(out) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--total-wall", type=float)
    ap.add_argument("--image")
    a = ap.parse_args(argv)
    p = Path(a.run_dir) / "result.json"
    if not p.exists():
        print(f"没有 {p}：容器没跑到任务结束，看 {a.run_dir}/console.log 与 px4.log")
        return 1
    print(render(json.loads(p.read_text()), a.total_wall, a.image), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


def render_compare(rows: list[dict]) -> str:
    """rows: dicts with keys positioning, run, metrics, ok."""
    lines = [
        "## S1b 默认定位 vs RTK 厘米级定位（同一任务，各 2 次，report-only，判据未冻结）",
        "",
        "误差：估计 = 相对 PX4 VehicleOdometry；真值 = 相对 Gazebo。",
        "高度偏差 = 悬停段估计高度 − 真值高度（同一世界系）的均值 / RMS / 最大。",
        "",
        "| 定位 | 运行 | 结果 | 悬停估计误差 RMS / 最大 m | 悬停真值误差 RMS / 最大 m | 到点估计 / 真值 m | 估计相对真值高度偏差 均值 / RMS / 最大 m |",
        "|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        m = r.get("metrics") or {}
        pos = r.get("positioning") or "—"
        run = r.get("run") or "—"
        ok = "通过" if r.get("ok") else f"未完成（{(m.get('failure') or r.get('phase'))}）"
        est = f"{fmt(m.get('hover_alt_err_rms_m'))} / {fmt(m.get('hover_alt_err_max_m'))}"
        tru = f"{fmt(m.get('hover_alt_err_truth_rms_m'))} / {fmt(m.get('hover_alt_err_truth_max_m'))}"
        goto = f"{fmt(m.get('goto_err_m'))} / {fmt(m.get('goto_err_truth_m'))}"
        bias = (
            f"{fmt(m.get('hover_est_truth_alt_bias_mean_m'))} / "
            f"{fmt(m.get('hover_est_truth_alt_bias_rms_m'))} / "
            f"{fmt(m.get('hover_est_truth_alt_bias_max_m'))}"
        )
        lines.append(f"| {pos} | {run} | {ok} | {est} | {tru} | {goto} | {bias} |")
    return "\n".join(lines) + "\n"
