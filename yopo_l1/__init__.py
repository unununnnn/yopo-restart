"""L1 抽检的 box 侧适配：场景卡导出、日志收回、L0/L1 对照。

不启动 PX4，不改 PX4 参数，不改安全清单数值。Orin 上的真 SITL 由
scripts/l1_spotcheck.sh 调用 yopo-swarm 的 scripts/run_g2.sh nominal。
不在宿主上创建 venv，也不安装 torch。
"""

from .export import export_l1
from .importer import import_l1
from .report import l0_vs_l1

__all__ = ["export_l1", "import_l1", "l0_vs_l1"]
