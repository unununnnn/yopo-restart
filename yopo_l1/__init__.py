"""L1 抽检的 box 侧适配：场景卡导出、日志收回、L0/L1 对照。

不启动 PX4，不改 PX4 参数，不改安全清单数值。真 SITL 由 scripts/l1_spotcheck.sh
在 WSL 或 Orin 上跑。
"""

from .export import export_l1
from .importer import import_l1
from .report import l0_vs_l1

__all__ = ["export_l1", "import_l1", "l0_vs_l1"]
