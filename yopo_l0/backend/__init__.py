"""YOPO 后段。默认不导入求解器。

``config`` 只解析场景卡开关。``bspline`` 在开关打开、真的要优化时才导入。
"""

from .config import BackendConfig, config_from_params

__all__ = ["BackendConfig", "config_from_params"]
