# 票 S4c：SITL 容器内 YOPO L0 运行时分层镜像

S4b 已经写明：[`scripts/sitl/Dockerfile`](../../scripts/sitl/Dockerfile) 里只有 ROS 2 Humble（Python 3.10 的 `rclpy`）、PX4 v1.17.0、Gazebo Harmonic、Micro-XRCE-DDS-Agent 和 `px4_msgs`，没有 `torch` / `scipy` / `ruamel.yaml` / `matplotlib`。容器里的系统 `python3` 因此导不进 `yopo_l0`，跑不了 [`scripts/sitl/yopo_l0_live.py`](../../scripts/sitl/yopo_l0_live.py)。

本票只准备这一层运行时和离线测试。判据未冻结，**report-only**。没有在云端实起 Docker，也没有实起 PX4。

## 不改什么

- [`scripts/sitl/Dockerfile`](../../scripts/sitl/Dockerfile) 不改。S1/S1b 标签仍是 `yopo-restart-sitl:s1b-<该文件 sha256 前 12 位>`。
- [`scripts/sitl/common.sh`](../../scripts/sitl/common.sh) 里 `SITL_IMAGE` 的默认值不改。`up.sh` / `smoke.sh` 不读分层镜像，也不会因为这层去重编 PX4。
- 不改 `yopo_l0/`、`core/`、`scenes/`、`offboard_mission.py`。`YOPO_SITL_PLANNER` 缺省仍是 `offboard`。

## 分层镜像

[`scripts/sitl/Dockerfile.yopo`](../../scripts/sitl/Dockerfile.yopo)：

- `ARG BASE_IMAGE`，默认 `yopo-restart-sitl:s1b-placeholder`，构建时由脚本传入真正的 `$SITL_IMAGE`。
- `FROM ${BASE_IMAGE}`。
- `pip3 install --no-cache-dir --extra-index-url https://download.pytorch.org/whl/cpu` 安装 `torch==2.4.1+cpu`，以及 `numpy>=1.26,<2.3`、`scipy>=1.10,<1.16`、`ruamel.yaml>=0.17`、`matplotlib>=3.9`。
- 安装后再单独跑 `python3 -c "import torch, numpy, scipy, ruamel.yaml; print(torch.__version__)"`。

NumPy 上界在安装行里收成 `numpy>=1.26,<2`，同时保留票面的 `numpy>=1.26,<2.3`。`<2.3` 是为了躲开不再提供 Python 3.10 轮子的 2.3；再收到 `<2` 是因为 Humble 的 `rclpy` 扩展按 NumPy 1.x ABI 编译，装进 `/usr/local` 的 NumPy 2 会让同一个 `python3` 里 `import rclpy` 失败。1.26 仍支持 Python 3.10，也能和 `torch==2.4.1+cpu` 一起解析。pip 只往 `/usr/local` 装，不 `apt-get` 动系统 Python，不覆盖 `/opt/ros` 里的 `rclpy`。

在本机用 pip 26 对 Python 3.10、`--only-binary=:all:` 做了解析（没有构建镜像）：`torch==2.4.1+cpu`（cp310 轮子）、`numpy==1.26.4`、`scipy==1.15.3`、`ruamel.yaml==0.19.1`、`matplotlib==3.10.9`。

## 标签和入口

[`scripts/sitl/build_yopo_image.sh`](../../scripts/sitl/build_yopo_image.sh) 把 `Dockerfile` 和 `Dockerfile.yopo` 按字节拼起来做 sha256，取前 12 位：

`yopo-restart-sitl:yopo-<哈希>`

- 默认前台构建。镜像已经存在就打印 `ready` 并退出 0。
- `$SITL_IMAGE` 不存在就退出 2，提示先跑 `scripts/sitl/up.sh`。不调用基础 Dockerfile，不重编 PX4。
- `--bg` 用 `setsid` 把构建放后台。PID 写到 `$SITL_STATE/yopo-build.pid`，日志写到 `$SITL_STATE/yopo-build-<哈希>.log`。`$SITL_STATE` 默认是 `/workspace/yopo-ops/sitl`，可用 `YOPO_SITL_STATE` 改。这个 PID 文件和 `up.sh` 的 `build.pid` 不是同一个。
- `--status` 打印 `ready` / `building` / `missing`。
- `--print-image` 只打印标签，不创建状态目录。

[`scripts/sitl/run_yopo_l0_sitl.sh`](../../scripts/sitl/run_yopo_l0_sitl.sh) 选择运行镜像的顺序：

1. `--image`
2. 环境变量 `YOPO_SITL_IMAGE`（`common.sh` 会把它放进 `SITL_IMAGE`）
3. 本机已经有上面的分层标签时，用分层镜像
4. 否则仍用 S1b 基础镜像

`docker run` 之前检查选中的标签在不在。`--dry-run` 不要求镜像存在。原有的 `planner`、`scene`、`n`、`positioning`、`seed` 行保留，末尾多一行 `image=`。容器已经由 `up.sh` 拉起时，仍是进现有容器补跑，不会为了换镜像把仿真停掉。

## 本环境实测

没有 Docker，没有 PX4。单测用假的 `docker` 可执行文件看参数，不连接守护进程。

```bash
PYTHONPATH=".:core/src" .venv/bin/python -m pytest tests/test_sitl*.py -q
bash -n scripts/sitl/*.sh
python3 scripts/check_doc_links.py
```

解释器是 `.venv` 里的 Python 3.12（numpy 2.5.4，torch 2.4.1+cpu）。`PYTHONPATH` 指向仓库根和 `core/src`。容器分层要装的是 Python 3.10 约束，和这台机器上跑单测的 3.12 不是同一套轮子。

| 命令 | 结果 |
|---|---|
| `pytest tests/test_sitl*.py -q` | **59 passed** in 23.86s |
| `bash -n scripts/sitl/*.sh` | 退出码 0 |
| `python3 scripts/check_doc_links.py` | 退出码 0，相对链接全部可解析 |
