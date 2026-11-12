# syntax=docker/dockerfile:1
# 票 S1：box 上 headless 的单机 PX4 SITL + Gazebo Harmonic + uXRCE-DDS + px4_msgs。
# 版本与 core/src/yopo_swarm_core/px4.py（ADR-0011 的钉版）一致：
#   PX4 v1.17.0 (d6f12ad) / px4_msgs release/1.17 (86d8239) / Micro-XRCE-DDS-Agent v2.4.3 / ROS 2 Humble。
# 不改任何 PX4 参数：镜像里只编译官方源码，参数全用 airframe 自带默认值。
# 由 scripts/sitl/up.sh 构建；不要直接 docker build（up.sh 负责 nice、标签和幂等）。
FROM ros:humble-ros-base-jammy

ARG PX4_REF=d6f12ad1c4f70ad3230afd7d86e971421e02fef4
ARG PX4_MSGS_REF=86d8239e962f6939e05c3737784f60c02fa884db
ARG XRCE_AGENT_REF=v2.4.3
ARG JOBS=4
ARG NICE=15

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 PIP_NO_CACHE_DIR=1
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# packages.ros.org 在 box 上证书不匹配（2026-11-12 实测），ROS 已在基础镜像里，去掉它的 apt 源。
RUN rm -f /etc/apt/sources.list.d/ros2*.sources /etc/apt/sources.list.d/ros2*.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends sudo lsb-release wget git ca-certificates gnupg \
 && rm -rf /var/lib/apt/lists/*

# PX4 源码（钉提交，浅克隆子模块）
RUN git clone --filter=blob:none --no-checkout https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
 && cd /opt/PX4-Autopilot && git checkout -q "${PX4_REF}" \
 && git submodule update --init --recursive --depth 1 --jobs 4 \
 && git describe --tags --always > /opt/PX4-Autopilot/.yopo-px4-describe

# PX4 官方依赖脚本（不装 NuttX 工具链）。仿真依赖不让脚本装：它写的 Gazebo 源是 http://，
# box 的出口代理对 packages.osrfoundation.org 的 http 返回 500（2026-11-12 实测），https 正常。
RUN cd /opt/PX4-Autopilot && bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools \
 && rm -rf /var/lib/apt/lists/* /root/.cache

# 与 ubuntu.sh 的「Simulation tools」段同一份包表（v1.17.0），只把 Gazebo 源换成 https；另加 gz 的 Python 绑定（读仿真时钟）
RUN wget -q https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends bc dmidecode gz-harmonic libunwind-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    libeigen3-dev libgstreamer-plugins-base1.0-dev libimage-exiftool-perl libopencv-dev libxml2-utils pkg-config protobuf-compiler \
    python3-gz-transport13 python3-gz-msgs10 \
 && rm -rf /var/lib/apt/lists/*

# 编译 SITL（nice，限并行）
RUN cd /opt/PX4-Autopilot && nice -n "${NICE}" make px4_sitl_default j="${JOBS}"

# Micro-XRCE-DDS-Agent
RUN git clone --depth 1 -b "${XRCE_AGENT_REF}" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
 && cmake -S /tmp/agent -B /tmp/agent/build -DCMAKE_BUILD_TYPE=Release \
 && nice -n "${NICE}" cmake --build /tmp/agent/build -j "${JOBS}" \
 && cmake --install /tmp/agent/build && ldconfig && rm -rf /tmp/agent

# px4_msgs
RUN mkdir -p /opt/ros_ws/src && git clone https://github.com/PX4/px4_msgs.git /opt/ros_ws/src/px4_msgs \
 && git -C /opt/ros_ws/src/px4_msgs checkout -q "${PX4_MSGS_REF}" \
 && source /opt/ros/humble/setup.bash && cd /opt/ros_ws \
 && MAKEFLAGS="-j${JOBS}" nice -n "${NICE}" colcon build --parallel-workers 1 --event-handlers console_direct- \
 && rm -rf /opt/ros_ws/build /opt/ros_ws/log

LABEL org.yopo.s1.px4="${PX4_REF}" org.yopo.s1.px4_msgs="${PX4_MSGS_REF}" org.yopo.s1.xrce_agent="${XRCE_AGENT_REF}"
WORKDIR /opt
