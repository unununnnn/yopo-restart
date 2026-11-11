#!/usr/bin/env python3
"""把一条教师记录转成学生窗口张量。

全量样本不在本分支。PR #49（分支 cursor/d2-teacher-data-fa95）合入之后：

    .venv/bin/python scripts/distill/teacher_to_tensors.py \\
        --run data/teacher/sample/d014-n1-headon-s0 \\
        --out /tmp/d014-windows.npz

本仓库里能直接转的是冒烟夹具 tests/fixtures/d3/d014-n1-headon-s0。
成功不是 true 时退出码 2，不写文件。多机记录退出码 1。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from yopo_l0.distill import DistillError, convert_run, save_batch


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="教师记录 → 单机学生窗口张量")
    parser.add_argument("--run", type=Path, required=True, help="含 trajectory.npz、record.json、scene.json 的目录")
    parser.add_argument("--out", type=Path, required=True, help="写出的 npz")
    parser.add_argument("--max-windows", type=int, default=None, help="沿轨迹均匀抽取的窗口数。缺省是全部合法窗口")
    args = parser.parse_args(argv)
    try:
        batch = convert_run(args.run, max_windows=args.max_windows)
    except DistillError as exc:
        print(f"转换失败：{exc}", file=sys.stderr)
        return 1
    if batch.n == 0:
        print(f"{batch.scenario_id}：没有可训练窗口（{batch.skipped}）", file=sys.stderr)
        return 2
    save_batch(batch, args.out)
    print(
        f"{batch.scenario_id}：{batch.n} 个窗口，深度 {tuple(batch.depth.shape)}，"
        f"写到 {args.out}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
