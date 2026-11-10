# vendored：yopo-swarm core

- 来源：https://github.com/unununnnn/yopo-swarm `core/`，commit `00b85bf`（Merge PR #48，box 上 /workspace/yopo-swarm 的 main）。
- 原样复制 `src/yopo_swarm_core/`、`tests/`（含 golden）、`pyproject.toml`；包名保持 `yopo_swarm_core`，以便原测试逐字不改。
- 唯一删除：`tests/test_g0_pending_dry_run.py`（测的是旧仓库 `scripts/g0_orin_pending.py`，Orin 运维脚本，本仓库不带）。
- 测试用到的仓库参考文件一并原样复制：`docs/adr/0006-yopo-minco-baseline.md`、`docs/ref/{minco-port-map.md,upstream.md,golden.md,g3-explore-2uav.json,golden_gpu_orin.json}`、`docs/ref/px4-depth/{single-camera.npz,resolved-camera-pose.json}`、`docs/ref/ros2-node/px4_1.jsonl.gz`。
- 环境：Python 3.12、torch 2.4.1+cpu、numpy 2.5.4（同旧仓库实际跑测试的 ys-venv；`core/pyproject.toml` 写的 `numpy<2` 与实际不符，旧仓库 acceptance-g1-orin.md 已记录）。在 numpy 1.26.4 下 `test_node_golden.py::test_goal_obs_and_body_obs_match` 差 9.5e-10，超出其 1e-12 容差。
- 上游金样：`pytest -m upstream` 会按钉定 SHA d5e1e60 把 YOPO 上游取到 `~/.cache/yopo-swarm`，校验权重 sha256 后在 CPU 上跑前向并和 `tests/golden/minco_d5e1e60_cpu.npz` 比对。
- `test_upstream_branch_head_has_not_moved` 联网查上游分支 head；2026-11-09 上游已推进到 d81e9c0（钉定仍是 d5e1e60）。按 ADR-0006 这需要先修订 ADR 再决定是否跟进，不在本仓库自动处理；回归里排除这一条。
- 规则：改 vendored 代码要在本文件记一笔；`certified` 策略下这些测试和金样必须保持不变。
- `VENDORED.sha256`：记录下来的 sha256，`tests/test_policies.py::test_vendored_core_unchanged` 校验。改了 core 就要同时改这份清单。
- 2026-11-10 票 13d：`protocol/node.py` 与 `protocol/__init__.py` 增加 Delay Check 状态机（ADR 0011）。`delay_check_s` 默认 0，certified 规划、金样和 YOPO 权重没动。新测试在 `core/tests/test_protocol_delay_check.py`，不在原始 vendor 清单里。
