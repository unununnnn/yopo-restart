"""YOPO-Swarm ROS-free 算法核心。

G0–G3 的 CPU 离线部分：baseline（上游固定与金样）、collision（连续时间冲突检查）、
candidates（候选解码）、freespace（深度证据）、plan（计划契约）、planner / replay（单机规划器与
重放日志）、protocol（离散事件协议）。不得依赖 rclpy 或任何 ROS 消息类型。
"""

__version__ = "0.0.0"
