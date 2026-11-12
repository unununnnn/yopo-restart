#!/usr/bin/env python3
"""把一条教师记录转成学生窗口张量。

单机和多机都可以转。多机按每一架的视点出窗口，并带上最近 K 架邻机特征。

    .venv/bin/python scripts/distill/teacher_to_tensors.py \\
        --run data/teacher/sample/d000-n3-headon-s0 \\
        --out /tmp/d000-n3-windows.npz

本仓库里还能直接转冒烟夹具 tests/fixtures/d3/d014-n1-headon-s0。
成功不是 true 时退出码 2，不写文件。缺字段或对不齐时退出码 1。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from yopo_l0.distill import DistillError, convert_run, save_batch


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="教师记录 → 学生窗口张量（含邻机特征）")
    parser.add_argument("--run", type=Path, required=True, help="含 trajectory.npz、record.json、scene.json 的目录")
    parser.add_argument("--out", type=Path, required=True, help="写出的 npz")
    parser.add_argument("--max-windows", type=int, default=None, help="沿（时刻，视点）均匀抽取的窗口数。缺省是全部合法窗口")
    parser.add_argument(
        "--altitude-shift",
        type=float,
        default=0.0,
        help="把位置和目标的 z 平移这么多米，再按 L0 相机重渲深度。默认 0，与不传逐位相同。",
    )
    args = parser.parse_args(argv)
    try:
        shift = float(args.altitude_shift)
        batch = convert_run(
            args.run,
            max_windows=args.max_windows,
            **({} if shift == 0.0 else {"altitude_shift_m": shift}),
        )
    except DistillError as exc:
        print(f"转换失败：{exc}", file=sys.stderr)
        return 1
    if batch.n == 0:
        print(f"{batch.scenario_id}：没有可训练窗口（{batch.skipped}）", file=sys.stderr)
        return 2
    save_batch(batch, args.out)
    print(
        f"{batch.scenario_id}：{batch.n} 个窗口，深度 {tuple(batch.depth.shape)}，"
        f"邻机 {tuple(batch.neighbor_rel.shape)}，写到 {args.out}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
