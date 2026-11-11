"""票 12c 内部助手：选一套中文和拉丁都有的字体，再调用票 04 的离线渲染器。

本机的 DroidSansFallbackFull 缺拉丁字母，渲染器若落到那套字体，轴标和标题会变成方框。
这里先把字体设好，并挡住 render._ensure_cjk_font 再去选那套残缺字体。不改 yopo_l0/render.py。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

import yopo_l0.render as R

_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)


def _prepare_font() -> None:
    for path in _FONT_CANDIDATES:
        if not Path(path).is_file():
            continue
        try:
            font_manager.fontManager.addfont(path)
            name = font_manager.FontProperties(fname=path).get_name()
        except (OSError, RuntimeError, ValueError):
            continue
        plt.rcParams["font.family"] = "sans-serif"
        plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        R._FONT_READY = True
        return


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="交付视频单条渲染（内部）")
    ap.add_argument("--log", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-seconds", type=float, default=60)
    a = ap.parse_args(argv)
    _prepare_font()
    try:
        info = R.render_video(a.log, a.out, scene=a.scene, max_seconds=a.max_seconds)
    except (ValueError, RuntimeError) as e:
        print(e, file=sys.stderr)
        return 2
    print(f"写入 {info.path}（{info.n_frames} 帧，{info.duration_s:.2f}s）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
