"""L1 适配的对外错误。调用方只看这些，消息里写清为什么拒绝。"""


class L1GeometryError(ValueError):
    """现有 PX4 SITL 世界表达不了这张卡的几何。"""


class L1LaunchError(ValueError):
    """启动文件对不上抽检要传的参数，或者策略不在 L1 的选择里。"""


class L1LogError(ValueError):
    """日志目录里没有能收成 result.json 的记录。"""


class L1ReportError(ValueError):
    """两份结果不是同一张场景卡，或目录里没有 result.json。"""
