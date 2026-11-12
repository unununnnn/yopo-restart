#!/usr/bin/env python3
"""在一条成功的教师轨迹上过拟合几个窗口。CPU，几十到几百步。

这是脚手架冒烟，不保存可供飞行使用的权重，也不进闭环。
调用这个脚本就是一次显式训练。库函数 ``train_smoke`` 本身默认不训练。
多机记录会把邻机特征一起喂进去。

    .venv/bin/python scripts/distill/train_smoke.py \\
        --run tests/fixtures/d3/d014-n1-headon-s0

    .venv/bin/python scripts/distill/train_smoke.py \\
        --run data/teacher/sample/d000-n3-headon-s0

位置均方误差降不到阈值时退出码 1。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from yopo_l0.distill import DistillError, convert_run
from yopo_l0.student import train_smoke


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="学生冒烟：过拟合少量成功窗口（含邻机特征）")
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--windows", type=int, default=4)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--lr", type=float, default=1e-2)
    args = parser.parse_args(argv)
    try:
        batch = convert_run(args.run, max_windows=args.windows)
    except DistillError as exc:
        print(f"转换失败：{exc}", file=sys.stderr)
        return 1
    if batch.n == 0:
        print(f"{batch.scenario_id}：没有可训练窗口（{batch.skipped}）", file=sys.stderr)
        return 2
    report = train_smoke(
        batch.depth,
        batch.state,
        batch.position_body,
        batch.velocity_body,
        batch.yaw_rel,
        batch.neighbor_rel,
        batch.neighbor_mask,
        enabled=True,
        steps=args.steps,
        seed=args.seed,
        lr=args.lr,
        scenario_id=batch.scenario_id,
    )
    print(
        f"{report.scenario_id}：窗口 {report.n_windows}，步数 {report.steps}，"
        f"损失 {report.loss_initial:.6g} → {report.loss_final:.6g}，"
        f"位置 MSE {report.position_mse_final:.6g}，"
        f"速度 MSE {report.velocity_mse_final:.6g}，"
        f"偏航 MSE {report.yaw_mse_final:.6g}，"
        f"过拟合 {'是' if report.overfit else '否'}"
    )
    return 0 if report.overfit else 1


if __name__ == "__main__":
    raise SystemExit(main())
