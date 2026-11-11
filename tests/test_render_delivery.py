"""票 12c：交付视频脚本能从日志渲出 ≤60 s 的 mp4。"""

import json
import subprocess
from pathlib import Path

from conftest import ROOT


def write_ticks(path: Path, n: int, dt: float, n_uav: int = 1) -> None:
    with path.open("w") as f:
        for k in range(n):
            for i in range(n_uav):
                row = {
                    "k": k,
                    "t": round(k * dt, 9),
                    "uav": f"uav{i}",
                    "p": [0.01 * k, 0.2 * i, 1.5],
                    "v": [0, 0, 0],
                    "a": [0, 0, 0],
                    "frame_stamp": None,
                    "min_obstacle_m": 1.0,
                    "goal_error_m": 0.0,
                    "yaw": 0.0,
                    "plan_code": "HOVER",
                }
                f.write(json.dumps(row) + "\n")


def ffprobe_duration(path: Path) -> float:
    p = subprocess.run(
        [
            "ffprobe", "-v", "error",
            "-show_entries", "format=duration",
            "-of", "default=nw=1:nk=1",
            str(path),
        ],
        capture_output=True, text=True, check=True,
    )
    return float(p.stdout.strip())


def test_render_delivery_script_produces_video_at_most_60s(tmp_path):
    log = tmp_path / "ticks.jsonl"
    # 短日志按实时播放，成片必须 ≤60 s。
    write_ticks(log, 30, 0.1, n_uav=2)
    out = tmp_path / "M0-g2-pillar-upstream-seed0.mp4"
    index = tmp_path / "videos.md"
    copy = tmp_path / "copy"
    proc = subprocess.run(
        [
            str(ROOT / "scripts/render_delivery.sh"),
            "--from-log", str(log),
            "--scene", str(ROOT / "scenes/g2-pillar.json"),
            "--milestone", "M0",
            "--policy", "upstream",
            "--seed", "0",
            "--commit", "deadbeefcafebabe",
            "--max-seconds", "60",
            "--out-mp4", str(out),
            "--index", str(index),
            "--copy-to", str(copy),
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr + "\n" + proc.stdout
    assert out.is_file() and out.stat().st_size > 500
    duration = ffprobe_duration(out)
    assert duration <= 60.5
    copied = copy / out.name
    assert copied.is_file() and copied.stat().st_size == out.stat().st_size
    text = index.read_text()
    assert "deadbeefcafebabe" in text
    assert "g2-pillar" in text
    assert "M0-g2-pillar-upstream-seed0.mp4" in text
    assert "60 s" in text
    assert "deadbee" in proc.stdout
    assert "seed 0" in proc.stdout
