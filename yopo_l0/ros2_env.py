"""让当前解释器找得到已安装的 ROS 2，而不要求调用方先 source setup.bash。

`scripts/setup_ros2.sh` 把 `/opt/ros/<distro>/lib` 登记进 ldconfig。这里再把
该发行版的 site-packages 放进 `sys.path`，并把 `LD_LIBRARY_PATH` 等变量补进
环境，子进程（例如测试里再起的启动器）从一开始就能加载 rclpy 的动态库。
"""

from __future__ import annotations

import importlib
import importlib.util
import os
import subprocess
import sys
from pathlib import Path

_CACHED: str | None | bool = False  # False = 还没找过

_KEEP_PREFIXES = ("ROS_", "AMENT_", "COLCON_", "CMAKE_", "RMW_")
# PATH 不能整段换成 setup.bash 的值，否则子进程会丢 .venv。PYTHONPATH 单独拼。
_SKIP_APPLY = {"PATH", "PYTHONPATH", "PWD", "OLDPWD", "SHLVL", "_"}


def _installed(*names: str) -> str | None:
    forced = os.environ.get("ROS_DISTRO")
    ordered = ([forced] if forced else []) + [n for n in names if n != forced]
    for name in ordered:
        if name and (Path("/opt/ros") / name / "setup.bash").is_file():
            return name
    return None


def _source_env(setup: Path) -> dict[str, str]:
    script = f"set +u; source '{setup}'; env -0"
    out = subprocess.check_output(["bash", "-c", script])
    env: dict[str, str] = {}
    for item in out.split(b"\0"):
        if not item or b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        name = key.decode(errors="replace")
        if not name.isidentifier():
            continue
        env[name] = value.decode(errors="replace")
    return env


def _merge(distro: str) -> None:
    prefix = Path("/opt/ros") / distro
    env = _source_env(prefix / "setup.bash")
    ros_py = [p for p in env.get("PYTHONPATH", "").split(":") if p.startswith(str(prefix))]
    for p in reversed(ros_py):
        if p not in sys.path:
            sys.path.insert(0, p)
    # rclpy 依赖发行版的 python3-yaml 等，装在 /usr 而不是 .venv。
    # 追加在末尾，venv 里的 numpy/torch 仍然优先。
    minor = f"{sys.version_info.major}.{sys.version_info.minor}"
    for extra in (f"/usr/lib/python3/dist-packages", f"/usr/local/lib/python{minor}/dist-packages"):
        if Path(extra).is_dir() and extra not in sys.path:
            sys.path.append(extra)
    old = os.environ.get("PYTHONPATH", "")
    parts = [p for p in ros_py if p and p not in old.split(":")]
    if parts:
        os.environ["PYTHONPATH"] = ":".join(parts + ([old] if old else []))
    for key, value in env.items():
        if key in _SKIP_APPLY:
            continue
        if key.startswith(_KEEP_PREFIXES) or f"/opt/ros/{distro}" in value:
            os.environ[key] = value
    importlib.invalidate_caches()


def ensure_ros_paths() -> str | None:
    """返回发行版名（jazzy / humble / 已在路径里的 ROS_DISTRO）。没安装时返回 None。"""
    global _CACHED
    if _CACHED is not False:
        return _CACHED  # type: ignore[return-value]
    distro = _installed("jazzy", "humble")
    if distro is None:
        if importlib.util.find_spec("rclpy") is not None:
            _CACHED = os.environ.get("ROS_DISTRO") or "preloaded"
            return _CACHED
        _CACHED = None
        return None
    _merge(distro)
    _CACHED = distro
    return distro
