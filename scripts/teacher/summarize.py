"""把一批 record.json 收成按密度、按机数的成功率。测不到的格子是 null。"""

from __future__ import annotations

import json
import platform
import subprocess
from pathlib import Path

from .assumptions import DENSITIES_PER_M2, DRONE_COUNTS, PATTERNS


def _cell(rows: list[dict]) -> dict:
    success = sum(1 for r in rows if r["metrics"]["success"] is True)
    fail = sum(1 for r in rows if r["metrics"]["success"] is False)
    unknown = sum(1 for r in rows if r["metrics"]["success"] is None)
    measured = success + fail
    return {
        "n": len(rows),
        "success": success,
        "fail": fail,
        "unknown": unknown,
        "rate": None if measured == 0 else success / measured,
    }


def _index(run_dir: Path) -> list[dict]:
    rows = []
    if not run_dir.is_dir():
        return rows
    for record_path in sorted(run_dir.glob("*/record.json")):
        record = json.loads(record_path.read_text(encoding="utf-8"))
        scenario = record["scenario"]
        metrics = record["metrics"]
        rows.append(
            {
                "id": scenario["id"],
                "density_per_m2": scenario["density_per_m2"],
                "n_drones": scenario["n_drones"],
                "pattern": scenario["pattern"],
                "seed": scenario["seed"],
                "metrics": metrics,
                "wall_s": record["run"].get("wall_s"),
                "status": record["run"].get("status"),
            }
        )
    return rows


def machine_info() -> dict:
    cpu = Path("/proc/cpuinfo").read_text(encoding="utf-8", errors="replace")
    model = "未知"
    for line in cpu.splitlines():
        if line.lower().startswith("model name"):
            model = line.split(":", 1)[1].strip()
            break
    mem_kb = None
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        if line.startswith("MemTotal:"):
            mem_kb = int(line.split()[1])
            break
    product = None
    product_path = Path("/sys/class/dmi/id/product_name")
    if product_path.exists():
        product = product_path.read_text(encoding="utf-8").strip() or None
    hypervisor = "未知"
    virt = subprocess.run(["lscpu"], check=False, capture_output=True, text=True)
    for line in virt.stdout.splitlines():
        if "Hypervisor vendor" in line or "Virtualization type" in line:
            hypervisor = virt.stdout
            break
    return {
        "product_name": product,
        "cpu_model": model,
        "mem_total_kb": mem_kb,
        "uname": platform.platform(),
        "machine": platform.machine(),
        "lscpu": virt.stdout.strip(),
        "hypervisor_note": hypervisor if isinstance(hypervisor, str) else None,
    }


def summarize(run_dir: Path, *, batch_wall_s: float | None = None) -> dict:
    rows = _index(run_dir)
    by_density = {}
    for density in DENSITIES_PER_M2:
        picked = [r for r in rows if r["density_per_m2"] == density]
        by_density[str(density)] = _cell(picked)
    by_drones = {}
    for n in DRONE_COUNTS:
        by_drones[str(n)] = _cell([r for r in rows if r["n_drones"] == n])
    by_pattern = {}
    for pattern in PATTERNS:
        by_pattern[pattern] = _cell([r for r in rows if r["pattern"] == pattern])
    grid = {}
    for density in DENSITIES_PER_M2:
        grid[str(density)] = {}
        for n in DRONE_COUNTS:
            grid[str(density)][str(n)] = _cell(
                [r for r in rows if r["density_per_m2"] == density and r["n_drones"] == n]
            )
    walls = [r["wall_s"] for r in rows if isinstance(r["wall_s"], (int, float))]
    per_hour = None
    if batch_wall_s and batch_wall_s > 0 and rows:
        per_hour = len(rows) / (batch_wall_s / 3600.0)
    elif walls and sum(walls) > 0:
        per_hour = len(walls) / (sum(walls) / 3600.0)
    times = []
    for r in rows:
        vals = r["metrics"].get("planning_time_s") or []
        times.extend(vals)
    planning_mean = None if not times else float(sum(times) / len(times))
    return {
        "null_means": "未知",
        "n": len(rows),
        "overall": _cell(rows),
        "by_density": by_density,
        "by_drones": by_drones,
        "by_pattern": by_pattern,
        "by_density_and_drones": grid,
        "planning_time_mean_s": planning_mean,
        "planning_time_count": len(times),
        "batch_wall_s": batch_wall_s,
        "sum_scenario_wall_s": None if not walls else float(sum(walls)),
        "scenarios_per_hour": per_hour,
        "scenarios_per_hour_basis": "batch_wall_s" if batch_wall_s else "sum_scenario_wall_s",
        "machine": machine_info(),
        "runs": [
            {
                "id": r["id"],
                "density_per_m2": r["density_per_m2"],
                "n_drones": r["n_drones"],
                "pattern": r["pattern"],
                "seed": r["seed"],
                "status": r["status"],
                "success": r["metrics"]["success"],
                "arrived": r["metrics"]["arrived"],
                "collision_obstacle": r["metrics"]["collision_obstacle"],
                "collision_inter_drone": r["metrics"]["collision_inter_drone"],
                "min_inter_drone_m": r["metrics"]["min_inter_drone_m"],
                "min_obstacle_surface_m": r["metrics"]["min_obstacle_surface_m"],
                "planning_time_mean_s": r["metrics"]["planning_time_mean_s"],
                "planning_time_count": r["metrics"]["planning_time_count"],
                "wall_s": r["wall_s"],
            }
            for r in rows
        ],
    }


def format_rate(cell: dict) -> str:
    if cell["n"] == 0 or cell["rate"] is None:
        return "未知"
    return f"{cell['success']}/{cell['success'] + cell['fail']}"


def format_table(summary: dict) -> str:
    lines = ["| 密度 obs/m² | 1 架 | 3 架 | 6 架 | 该密度合计 |", "|---|---|---|---|---|"]
    grid = summary["by_density_and_drones"]
    for density in ("0.0", "0.14", "0.28", "0.42"):
        row = [density]
        for n in ("1", "3", "6"):
            row.append(format_rate(grid[density][n]))
        row.append(format_rate(summary["by_density"][density]))
        lines.append("| " + " | ".join(row) + " |")
    lines.append("")
    lines.append("| 机数 | 成功率 |")
    lines.append("|---|---|")
    for n in ("1", "3", "6"):
        lines.append(f"| {n} | {format_rate(summary['by_drones'][n])} |")
    lines.append("")
    lines.append("| 图案 | 成功率 |")
    lines.append("|---|---|")
    for pattern, label in (("parallel", "平行"), ("headon", "对向")):
        lines.append(f"| {label} | {format_rate(summary['by_pattern'][pattern])} |")
    rate = summary["scenarios_per_hour"]
    speed = "未知" if rate is None else f"{rate:.2f}"
    lines.append("")
    lines.append(f"场景/小时: {speed}（口径 {summary['scenarios_per_hour_basis']}）")
    lines.append(f"合计: {format_rate(summary['overall'])}，未知 {summary['overall']['unknown']}")
    return "\n".join(lines) + "\n"
