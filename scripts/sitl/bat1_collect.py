"""票 BAT1：从已有 SITL 运行目录统计 Battery unhealthy（只读，不改运行目录）。

    python3 scripts/sitl/bat1_collect.py <run_dir> [<run_dir> ...]

不读、不写任何 PX4 的 COM_ / NAV_ / BAT_ 参数。ulog 是可选的：有 pyulog 且目录里有
.ulg 时，再核对 battery_status 是否在解锁时刻停发，以及 unhealthy 旗标相对解锁的延迟。
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

UNHEALTHY_RE = re.compile(r"Battery unhealthy")
FAILSAFE_RE = re.compile(r"Failsafe activated", re.I)
BAT_WARN_RE = re.compile(r"battery warning", re.I)
# PX4 batteryCheck.cpp：hrt_elapsed_time(&oldest_update) > 5_s 则 battery_unhealthy。
STALE_BATTERY_S = 5.0


def count_console(text: str) -> dict:
    return {
        "battery_unhealthy": len(UNHEALTHY_RE.findall(text)),
        "failsafe_activated": len(FAILSAFE_RE.findall(text)),
        "battery_warning": len(BAT_WARN_RE.findall(text)),
    }


def stopped_at_arm(arm_s: float | None, bat_last_s: float | None, duration_s: float | None) -> bool:
    """ulog 里 battery_status 最后一条紧贴解锁，且飞行在解锁后再走至少 5 s。"""
    if arm_s is None or bat_last_s is None or duration_s is None:
        return False
    return bat_last_s <= arm_s + 0.05 and duration_s >= arm_s + STALE_BATTERY_S


def _try_ulog(path: Path) -> dict | None:
    try:
        from pyulog import ULog
        import numpy as np
    except ImportError:
        return None
    ulg = ULog(str(path))
    t0 = int(ulg.start_timestamp)
    duration = (int(ulg.last_timestamp) - t0) / 1e6

    def get(name: str):
        try:
            return ulg.get_dataset(name)
        except Exception:
            return None

    def rel(ts):
        return (np.asarray(ts, dtype=np.int64) - t0) / 1e6

    arm = None
    vs = get("vehicle_status")
    failsafe = False
    if vs is not None and "arming_state" in vs.data:
        t = rel(vs.data["timestamp"])
        arming = vs.data["arming_state"]
        for i in np.where(np.diff(arming) != 0)[0]:
            if int(arming[i + 1]) == 2:
                arm = float(t[i + 1])
                break
        if "failsafe" in vs.data:
            failsafe = bool(np.any(vs.data["failsafe"] != 0))
    bat_n = 0
    bat_last = None
    voltage = None
    remaining = None
    warning = None
    connected = None
    faults = None
    bs = get("battery_status")
    if bs is not None:
        t = rel(bs.data["timestamp"])
        bat_n = int(len(t))
        if bat_n:
            bat_last = float(t[-1])
        if "voltage_v" in bs.data:
            voltage = {"min": float(bs.data["voltage_v"].min()), "max": float(bs.data["voltage_v"].max())}
        if "remaining" in bs.data:
            remaining = {"min": float(bs.data["remaining"].min()), "max": float(bs.data["remaining"].max())}
        if "warning" in bs.data:
            warning = [int(x) for x in sorted(set(bs.data["warning"].tolist()))]
        if "connected" in bs.data:
            connected = [int(x) for x in sorted(set(bs.data["connected"].tolist()))]
        if "faults" in bs.data:
            faults = [int(x) for x in sorted(set(bs.data["faults"].tolist()))]
    unh = False
    unh_t = None
    ff = get("failsafe_flags")
    if ff is not None and "battery_unhealthy" in ff.data:
        t = rel(ff.data["timestamp"])
        arr = ff.data["battery_unhealthy"]
        hit = np.where(arr == 1)[0]
        if len(hit):
            unh = True
            unh_t = float(t[hit[0]])
    lag = None if (unh_t is None or arm is None) else round(unh_t - arm, 4)
    return {
        "ulog": path.name,
        "duration_s": round(duration, 3),
        "arm_s": None if arm is None else round(arm, 3),
        "battery_status_n": bat_n,
        "battery_status_last_s": None if bat_last is None else round(bat_last, 3),
        "stopped_at_arm": stopped_at_arm(arm, bat_last, duration),
        "battery_unhealthy_flag": unh,
        "battery_unhealthy_flag_s": None if unh_t is None else round(unh_t, 3),
        "unhealthy_minus_arm_s": lag,
        "failsafe_uorb": failsafe,
        "voltage_v": voltage,
        "remaining": remaining,
        "warning": warning,
        "connected": connected,
        "faults": faults,
    }


def collect_run(run: Path) -> dict:
    run = run.resolve()
    out: dict = {
        "run_dir": run.name,
        "vehicles": [],
        "console_unhealthy_vehicles": 0,
        "console_failsafe_vehicles": 0,
    }
    result = run / "result.json"
    if result.is_file():
        meta = json.loads(result.read_text())
        out["scene"] = meta.get("scene")
        out["outcome"] = meta.get("outcome") or meta.get("ok")
        out["px4_parameters_written"] = meta.get("px4_parameters_written")
        ls = meta.get("lockstep")
        if isinstance(ls, dict):
            out["lockstep_enabled"] = bool(ls.get("enabled"))
            out["lockstep_mechanism"] = ls.get("mechanism")
            out["t_engage_s"] = ls.get("t_engage_s")
        elif isinstance(ls, bool):
            out["lockstep_enabled"] = ls
    logs = sorted(run.glob("px4*.log"))
    ulogs = {p.parent.parent.parent.name if "px4-state" in p.parts else p.parent.name: p for p in run.rglob("*.ulg")}
    unh = 0
    fs = 0
    for log in logs:
        text = log.read_text(errors="replace")
        console = count_console(text)
        inst = log.stem.replace("px4-", "").replace("px4", "1")
        rec = {"log": log.name, "console": console}
        # 多机目录：px4-1.log 对应 px4-state/1；单机 px4.log 对应 px4-state。
        ulog = None
        if inst in ulogs:
            ulog = ulogs[inst]
        elif len(ulogs) == 1 and len(logs) == 1:
            ulog = next(iter(ulogs.values()))
        if ulog is not None:
            rec["ulog"] = _try_ulog(ulog)
        out["vehicles"].append(rec)
        if console["battery_unhealthy"]:
            unh += 1
        if console["failsafe_activated"]:
            fs += 1
    out["console_unhealthy_vehicles"] = unh
    out["console_failsafe_vehicles"] = fs
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="统计一次 SITL 运行的 Battery unhealthy")
    parser.add_argument("runs", nargs="+", type=Path)
    args = parser.parse_args(argv)
    rows = [collect_run(p) for p in args.runs]
    json.dump(rows if len(rows) != 1 else rows[0], sys.stdout, ensure_ascii=False, indent=2)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
