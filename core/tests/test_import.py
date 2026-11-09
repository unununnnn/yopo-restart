import importlib
import sys


def test_core_imports_without_ros():
    mod = importlib.import_module("yopo_swarm_core")
    assert mod.__version__ == "0.0.0"
    assert "rclpy" not in sys.modules
