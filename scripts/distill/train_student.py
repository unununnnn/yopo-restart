#!/usr/bin/env python3
"""用 D2b 转换后的窗口训练学生，权重写到仓库外。

    .venv/bin/python scripts/distill/train_student.py \\
        --data /workspace/yopo-teacher-data/d5 --out /workspace/yopo-teacher-data/d5
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="训练 D5 学生（权重不进仓库）")
    parser.add_argument("--data", type=Path, default=Path("/workspace/yopo-teacher-data/d5"))
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--epochs", type=int, default=5)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--threads", type=int, default=2)
    args = parser.parse_args(argv)
    data = Path(args.data)
    out = Path(args.out) if args.out is not None else data
    manifest_path = data / "convert-manifest.json"
    if not manifest_path.is_file():
        print(f"缺少 {manifest_path}，先跑 convert_d2b.py", file=sys.stderr)
        return 1
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    win = data / "windows"
    train_paths = [win / f"{i}.npz" for i in manifest["train_ids"]]
    hold_paths = [win / f"{i}.npz" for i in manifest["hold_ids"]]
    missing = [str(p) for p in train_paths + hold_paths if not p.is_file()]
    if missing:
        print(f"缺窗口文件 {len(missing)} 个，例如 {missing[:3]}", file=sys.stderr)
        return 1
    from yopo_l0.student_fit import FitConfig, train_from_paths

    summary = train_from_paths(
        train_paths,
        hold_paths,
        out,
        cfg=FitConfig(
            lr=float(args.lr),
            batch_size=int(args.batch_size),
            epochs=int(args.epochs),
            seed=int(args.seed),
            num_threads=int(args.threads),
        ),
    )
    print(
        json.dumps(
            {
                "weights": summary.get("weights"),
                "weights_sha256": summary.get("weights_sha256"),
                "holdout": summary.get("holdout"),
                "epochs_done": summary.get("epochs_done"),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
