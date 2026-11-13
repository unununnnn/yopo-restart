"""把 scripts/sitl/smoke.sh 一次运行的 result.json 摘成一张 Markdown 表（report-only）。"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path


def fmt(v, nd=3):
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def _fleet():
    path = Path(__file__).with_name("fleet.py")
    name = "yopo_sitl_fleet_for_summary"
    mod = sys.modules.get(name)
    if mod is None:
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
    return mod


def _rows_from_csv(path: Path) -> list:
    rows = []
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines:
        return rows
    for line in lines[1:]:
        if not line.strip():
            continue
        cols = line.split(",")
        def cell(i):
            if i >= len(cols) or cols[i] == "":
                return None
            if i == 1:
                return cols[i]
            return float(cols[i])
        rows.append([cell(0), cell(1), cell(2), cell(3), cell(4), cell(5), cell(6), cell(7)])
    return rows


def trajectory_csv_paths(run_dir: Path) -> list[Path]:
    paths = sorted(Path(run_dir).glob("px4_*/trajectory.csv"))
    if paths:
        return paths
    return sorted(Path(run_dir).glob("px4-*/trajectory.csv"))


def min_inter_drone_m(res: dict | None = None, run_dir: Path | None = None) -> float | None:
    """优先用 result 里已经算好的值，其次 trajectory.csv，再次 result.json 里的 trace。"""
    if res and res.get("min_inter_drone_m") is not None:
        return float(res["min_inter_drone_m"])
    fleet = _fleet()
    if run_dir is not None:
        paths = trajectory_csv_paths(run_dir)
        if len(paths) >= 2:
            dist = fleet.min_inter_drone_m_from_vehicles([{"trace": _rows_from_csv(p)} for p in paths])
            if dist is not None:
                return dist
    if res:
        return fleet.min_inter_drone_m_from_vehicles(res.get("vehicles") or [])
    return None


def _with_separation(res: dict, run_dir: Path | None) -> dict:
    if res.get("min_inter_drone_m") is not None or not res.get("vehicles"):
        return res
    dist = min_inter_drone_m(res, run_dir)
    if dist is None:
        return res
    copied = dict(res)
    copied["min_inter_drone_m"] = dist
    return copied


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
           "report_only: true",
           "criteria_frozen: false",
           f"定位源：`{pos}`（default = 官方米级 GNSS 模型；rtk = 厘米级，默认关）。",
           "误差默认相对 PX4 自身估计（VehicleOdometry）；标「Gazebo 真值」的行含估计器误差。",
           "「估计相对真值的高度偏差」是悬停段 est_z − truth_z（同一世界系）。", ""]
    if image:
        out += [f"镜像：`{image}`", ""]
    out += ["| 项 | 值 |", "|---|---|"] + [f"| {k} | {v} |" for k, v in rows]
    return "\n".join(out) + "\n"


def render_fleet(res: dict, total_wall: float | None, image: str | None) -> str:
    """多机冒烟表。数字只罗列，判据未冻结，不根据误差大小写通过。"""
    vehicles = res.get("vehicles") or []
    mode = res.get("mission") or "hover"
    if mode == "goto":
        task = "每机在自己起飞点上方 1.5 m 悬停后，飞向悬停点 +(6, 0, 0) m，再降落。"
    elif mode == "swap":
        task = "三机错高 1.5 / 2.2 / 2.9 m 交换两端位置（中间一架保持 y，只换高度层），再降落。"
    else:
        task = "每机悬停在自己起飞点上方 1.5 m，采样后降落。"
    lines = [
        "## S2/S3 多机 PX4 SITL 冒烟（report-only，判据未冻结）",
        "",
        "report_only: true",
        "criteria_frozen: false",
        f"任务：`{mode}`。机数：{res.get('n', len(vehicles))}。DDS 域：`{res.get('dds_domain_id')}`。"
        f"XRCE 端口：`{res.get('xrce_port')}`（各机共用）。",
        f"定位源：`{res.get('positioning') or 'default'}`（default = 官方米级 GNSS 模型；rtk = 厘米级，默认关）。",
        task + "误差默认相对 PX4 自身估计；"
        "标「真值」的列相对 Gazebo。高度偏差是悬停段 est_z − truth_z（同一世界系）。",
        "「全程最小机间距离」用各机同一时刻的 Gazebo 真值；没有真值时退回估计位置。",
        "「结果」只表示有没有落地并上锁，不是误差门。",
        "",
    ]
    if image:
        lines += [f"镜像：`{image}`", ""]
    lines += [
        "| 机 | namespace | MAV_SYS_ID | 结果 | 悬停估计高度误差 RMS / 最大 m | 悬停真值高度误差 RMS / 最大 m | 估计相对真值高度偏差 均值 / RMS / 最大 m | 到点估计 / 真值 m | 降落用时 仿真 s |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for v in vehicles:
        m = v.get("metrics") or {}
        ns = v.get("namespace") or "—"
        ok = "飞完" if v.get("ok") else f"未完成（{m.get('failure') or v.get('phase')}）"
        est = f"{fmt(m.get('hover_alt_err_rms_m'))} / {fmt(m.get('hover_alt_err_max_m'))}"
        tru = f"{fmt(m.get('hover_alt_err_truth_rms_m'))} / {fmt(m.get('hover_alt_err_truth_max_m'))}"
        bias = (
            f"{fmt(m.get('hover_est_truth_alt_bias_mean_m'))} / "
            f"{fmt(m.get('hover_est_truth_alt_bias_rms_m'))} / "
            f"{fmt(m.get('hover_est_truth_alt_bias_max_m'))}"
        )
        goto = f"{fmt(m.get('goto_err_m'))} / {fmt(m.get('goto_err_truth_m'))}"
        lines.append(
            f"| {v.get('index', '—')} | {ns} | {v.get('mav_sys_id', '—')} | {ok} | {est} | {tru} | {bias} | {goto} | {fmt(m.get('land_s'), 1)} |"
        )
    lines += [
        "",
        "| 项 | 值 |",
        "|---|---|",
        ("| 整趟结果 | 飞完 |" if res.get("ok") else f"| 整趟结果 | 未完成（{res.get('phase')}） |"),
        f"| 全程最小机间距离 真值 (m) | {fmt(res.get('min_inter_drone_m'))} |",
        f"| 计划几何最小间隔 (m) | {fmt(res.get('min_geometric_separation_m'))} |",
        f"| 任务墙钟 (s) | {fmt(res.get('mission_wall_s'), 1)} |",
        f"| 任务仿真时长 (s) | {fmt(res.get('mission_sim_s'), 1)} |",
        f"| RTF | {fmt(res.get('rtf'), 2)} |",
        f"| 启动到预检通过墙钟 (s) | {fmt(res.get('wall_to_preflight_ok_s'), 1)} |",
        f"| 总墙钟含容器 (s) | {fmt(total_wall, 1)} |",
    ]
    return "\n".join(lines) + "\n"


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
    res = json.loads(p.read_text())
    res = _with_separation(res, Path(a.run_dir))
    if res.get("vehicles"):
        text = render_fleet(res, a.total_wall, a.image)
    else:
        text = render(res, a.total_wall, a.image)
    print(text, end="")
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
