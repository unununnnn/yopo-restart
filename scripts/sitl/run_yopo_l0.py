"""票 S4 / S4b：假 PX4 NED 上的 YOPO L0 离线闭环（report-only）。

单机直接用场景卡（``scenes/g2-pillar.json``）。多机卡（如 ``scenes/swap-3-open.json``
三机对向）为每架飞机建一个 ``YopoSitlBridge``，规划前互传上一拍的 ENU 采样。

不导入 rclpy。默认和 ``--fake-px4`` 都是进程内的 ``FakePx4Vehicle``。
``--mock-ros`` 仍用这架假飞机，但里程计和深度先编成 ROS 消息再解回桥。
真 PX4 容器入口是 ``scripts/sitl/run_yopo_l0_sitl.sh``，见 ``docs/research/s4b-sitl-live.md``。

用法：
  .venv/bin/python scripts/sitl/run_yopo_l0.py --scene scenes/g2-pillar.json --seed 0 --max-s 15
  .venv/bin/python scripts/sitl/run_yopo_l0.py --fake-px4 --scene scenes/g2-pillar.json --partial
  .venv/bin/python scripts/sitl/run_yopo_l0.py --mock-ros --positioning rtk --scene scenes/swap-3-open.json --partial
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from yopo_l0.scene import load_card
from yopo_l0.sitl_bridge import normalize_positioning, run_closed_loop, run_mock_ros_closed_loop

_REPO = Path(__file__).resolve().parents[2]


def _scene_path(raw: str) -> Path:
    path = Path(raw)
    if path.is_file():
        return path
    alt = _REPO / raw
    if alt.is_file():
        return alt
    raise SystemExit(f"找不到场景卡：{raw}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scene", default="scenes/g2-pillar.json", help="场景卡路径")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-s", type=float, default=15.0, dest="max_s", help="仿真时域上限（秒）")
    ap.add_argument(
        "--partial",
        action="store_true",
        help="只跑 0.5 s 前缀（仍不超过 --max-s），用于离线校验，不等到达",
    )
    ap.add_argument("--out", default=None, help="把 result.json 写到这个路径；同时打印到 stdout")
    ap.add_argument(
        "--fake-px4",
        action="store_true",
        help="假 PX4 离线闭环。缺省就是这个模式；写上只是为了和容器实飞入口区分",
    )
    ap.add_argument(
        "--mock-ros",
        action="store_true",
        help="仍用假 PX4，但里程计和深度先编成 ROS 消息再进桥（schema s4b）",
    )
    ap.add_argument(
        "--positioning",
        choices=("default", "rtk"),
        default="default",
        help="只写入 --mock-ros 的 result。不改飞控参数",
    )
    args = ap.parse_args(argv)
    if not args.max_s > 0.0:
        ap.error("--max-s 必须为正")
    card = load_card(_scene_path(args.scene))
    if args.mock_ros:
        result = run_mock_ros_closed_loop(
            card,
            seed=args.seed,
            max_s=args.max_s,
            partial=args.partial,
            positioning=normalize_positioning(args.positioning),
        )
    else:
        result = run_closed_loop(card, seed=args.seed, max_s=args.max_s, partial=args.partial)
    text = json.dumps(result, indent=1, ensure_ascii=False) + "\n"
    if args.out:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text)
    sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
