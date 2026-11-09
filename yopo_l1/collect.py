"""把一次 run_g2 写下的 result.json 和事件日志收进抽检目录。

只复制启动之后新出现的文件，不改 yopo-swarm 里的原件。
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

SKIP_DIRS = {".git", "__pycache__", ".venv", "node_modules", "build", "install"}
EVENT_NAMES = {"events.jsonl", "events.json", "events.jsonl.gz"}


def _is_event(name: str) -> bool:
    if name in EVENT_NAMES:
        return True
    return name.endswith(".jsonl") and "event" in name


def collect_g2_artifacts(swarm_root: str | Path, dest: str | Path, since: float, slack: float = 2.0) -> dict:
    swarm_root = Path(swarm_root)
    dest = Path(dest)
    if not swarm_root.is_dir():
        raise FileNotFoundError(f"没有 yopo-swarm 目录 {swarm_root}")
    dest.mkdir(parents=True, exist_ok=True)
    cutoff = float(since) - float(slack)
    dest_res = dest.resolve()
    results: list[Path] = []
    events: list[Path] = []
    for dirpath, dirnames, filenames in os.walk(swarm_root):
        kept = []
        for d in dirnames:
            if d in SKIP_DIRS:
                continue
            if (Path(dirpath) / d).resolve() == dest_res:
                continue
            kept.append(d)
        dirnames[:] = kept
        for name in filenames:
            path = Path(dirpath) / name
            try:
                if path.resolve().is_relative_to(dest_res):
                    continue
                if path.stat().st_mtime < cutoff:
                    continue
            except OSError:
                continue
            if name == "result.json":
                results.append(path)
            elif _is_event(name):
                events.append(path)
    results.sort(key=lambda p: (p.stat().st_mtime, str(p)), reverse=True)
    copied_result = None
    sources = [str(p) for p in results]
    if results:
        copied_result = dest / "orin-result.json"
        shutil.copy2(results[0], copied_result)
        (dest / "orin-result-source.txt").write_text(str(results[0]) + "\n", encoding="utf-8")
    ev_dir = dest / "events"
    copied_events = []
    for i, ev in enumerate(sorted(events, key=str)):
        ev_dir.mkdir(parents=True, exist_ok=True)
        target = ev_dir / f"{i:02d}-{ev.name}"
        shutil.copy2(ev, target)
        copied_events.append(str(target))
    return {"result": None if copied_result is None else str(copied_result), "events": copied_events, "sources": sources}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="收下 run_g2 写出的 result.json 和事件日志")
    ap.add_argument("swarm_root")
    ap.add_argument("dest")
    ap.add_argument("--since", type=float, required=True, help="run_g2.sh 启动前的 unix 秒")
    a = ap.parse_args(argv)
    try:
        info = collect_g2_artifacts(a.swarm_root, a.dest, a.since)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print(json.dumps(info, ensure_ascii=False))
    if not info["result"]:
        print(f"{a.swarm_root} 在本次运行之后没有新的 result.json", file=sys.stderr)
        return 5
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
