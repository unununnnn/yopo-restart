# M0 样例：g2-pillar

票 02 合入 main 之后，在本分支上重跑的 L0 结果。策略代码没有改。判据未冻结，表上的到达和碰撞只报数（[ADR 0008](../adr/0008-report-only-until-frozen.md)）。

## 出处

| 项 | 值 |
|---|---|
| 代码 commit | `036d90761c13beb2ad5574fc685af5f138b996e3`（main，票 02） |
| 层级 | L0 |
| 场景卡 | `scenes/g2-pillar.json` |
| 场景卡哈希 | `b00714a404ea47a64610ce4a057450b03326a2e574c6a2671ad9bf1338dd4f1d` |
| 场景卡文件 sha256 | `1780f4f5de7fbdeb8666f34a79a02eca10a3072916d86b42770643324b5512ff` |
| 几何文件 sha256 | `scenes/g2-scene.json` `b820e3b1e6989c10384a4dee6aad7d49a9ef9d87a1ced4580b79916581d8818e` |
| 网络 | 上游 `d5e1e60f83cc991602f142d9b6b96204a24d6b9f`，权重 sha256 `2d85e712c8874d11cd4e523777698eff78914fdb8f1ffb5068b037f8f972f62f`，`torch-cpu-fp32` |
| 默认终端 | `stop_at_goal`，半径 1.5 m，停车 2 s（[ADR 0009](../adr/0009-stop-at-goal-vs-hold-here.md)） |
| 影子盾 | 开（开关已记录；逐 tick 拒绝码是票 03，[ADR 0007](../adr/0007-l0-default-upstream-shadow.md)） |
| 到达口径 | ≤ 0.4 m 且 ≤ 0.2 m/s，保持 1 s；机体半径 0.35 m |

场景卡哈希是场景卡加上内联几何之后的规范 JSON 的 sha256，和结果记录里的 `scene_card_hash` 相同。文件 sha256 是仓库里那两个文件的原始字节。

墙钟取自这次 4 核机器，不参与确定性比较。同 seed 重跑只要求除墙钟外的结果记录和 tick 日志一致。

upstream 主表跑在干净工作区上，结果里的 `commit` 没有 `-dirty`。certified 全时长批跑开始时，已跟踪的改动只有本分支的 README 链接，所以那 10 条记录的 `commit` 是同一个 sha 加上 `-dirty`。策略、场景卡和 `core/` 在两次批跑之间没有改动。

原始记录：

- [m0-g2-pillar-upstream.json](m0-g2-pillar-upstream.json) — upstream 10 条 `result`（不含 tick）
- [m0-g2-pillar-certified.json](m0-g2-pillar-certified.json) — certified 10 条 `result`（不含 tick）

## g2-pillar @ upstream（默认）

到达 10/10，平均到达时间 4.27 s，碰撞 0。未冻结 / report-only。影子盾开。seed 0 确定性重跑一致。

| seed | 结果 | 到达时间 s | 最小障碍距离 m | 终点误差 m | 仿真 s | 墙钟 s | 规划码 |
|---|---|---|---|---|---|---|---|
| 0 | SUCCESS | 4.26 | 0.829 | 0.016 | 5.3 | 4.4 | ARRIVED_HOLD:32, OK:21 |
| 1 | SUCCESS | 4.27 | 0.829 | 0.016 | 5.3 | 4.3 | ARRIVED_HOLD:32, OK:21 |
| 2 | SUCCESS | 4.26 | 0.829 | 0.016 | 5.3 | 5.2 | ARRIVED_HOLD:32, OK:21 |
| 3 | SUCCESS | 4.26 | 0.829 | 0.016 | 5.3 | 4.4 | ARRIVED_HOLD:32, OK:21 |
| 4 | SUCCESS | 4.27 | 0.829 | 0.016 | 5.3 | 2.6 | ARRIVED_HOLD:32, OK:21 |
| 5 | SUCCESS | 4.28 | 0.828 | 0.016 | 5.3 | 2.7 | ARRIVED_HOLD:32, OK:21 |
| 6 | SUCCESS | 4.28 | 0.828 | 0.016 | 5.3 | 2.8 | ARRIVED_HOLD:32, OK:21 |
| 7 | SUCCESS | 4.28 | 0.828 | 0.016 | 5.3 | 2.6 | ARRIVED_HOLD:32, OK:21 |
| 8 | SUCCESS | 4.28 | 0.828 | 0.016 | 5.3 | 2.6 | ARRIVED_HOLD:32, OK:21 |
| 9 | SUCCESS | 4.27 | 0.829 | 0.016 | 5.3 | 2.7 | ARRIVED_HOLD:32, OK:21 |

十个 seed 的路径很接近：深度噪声没有换掉 argmax，最小障碍距离只在 0.828 m 和 0.829 m 之间。都大于机体半径 0.35 m。

## g2-pillar @ certified（对照，全时长）

到达 0/10，碰撞 0。未冻结 / report-only。影子盾开。批跑退出码 0（到达数不拦，见 [ADR 0010](../adr/0010-certified-env-unknown-reported.md)）。没有做 seed 0 的第二次全时长重跑。

结果记录里的 commit 是 `036d90761c13beb2ad5574fc685af5f138b996e3-dirty`。`-dirty` 只因为 README 已有未提交改动。场景卡哈希与 upstream 主表相同。

| seed | 结果 | 到达时间 s | 最小障碍距离 m | 终点误差 m | 仿真 s | 墙钟 s | 规划码 |
|---|---|---|---|---|---|---|---|
| 0 | TIMEOUT | — | 1.500 | 5.000 | 45.0 | 866.0 | BOOTSTRAP:1, NO_ENV_SAFE_CANDIDATE:429, WAIT_SWITCH:20 |
| 1 | TIMEOUT | — | 1.500 | 5.000 | 45.0 | 864.4 | BOOTSTRAP:1, NO_ENV_SAFE_CANDIDATE:429, WAIT_SWITCH:20 |
| 2 | TIMEOUT | — | 1.500 | 5.000 | 45.0 | 871.4 | BOOTSTRAP:1, NO_ENV_SAFE_CANDIDATE:429, WAIT_SWITCH:20 |
| 3 | TIMEOUT | — | 1.500 | 5.000 | 45.0 | 873.3 | BOOTSTRAP:1, NO_ENV_SAFE_CANDIDATE:429, WAIT_SWITCH:20 |
| 4 | TIMEOUT | — | 1.500 | 5.000 | 45.0 | 839.5 | BOOTSTRAP:1, NO_ENV_SAFE_CANDIDATE:429, WAIT_SWITCH:20 |
| 5 | TIMEOUT | — | 1.500 | 5.000 | 45.0 | 841.2 | BOOTSTRAP:1, NO_ENV_SAFE_CANDIDATE:429, WAIT_SWITCH:20 |
| 6 | TIMEOUT | — | 1.500 | 5.000 | 45.0 | 838.9 | BOOTSTRAP:1, NO_ENV_SAFE_CANDIDATE:429, WAIT_SWITCH:20 |
| 7 | TIMEOUT | — | 1.500 | 5.000 | 45.0 | 845.1 | BOOTSTRAP:1, NO_ENV_SAFE_CANDIDATE:429, WAIT_SWITCH:20 |
| 8 | TIMEOUT | — | 1.500 | 5.000 | 45.0 | 825.6 | BOOTSTRAP:1, NO_ENV_SAFE_CANDIDATE:429, WAIT_SWITCH:20 |
| 9 | TIMEOUT | — | 1.500 | 5.000 | 45.0 | 823.3 | BOOTSTRAP:1, NO_ENV_SAFE_CANDIDATE:429, WAIT_SWITCH:20 |

批跑把 `cand:` 计数单独汇总，不放进上表的规划码列。十个 seed 这一列相同，每个 seed `cand:ENV:UNKNOWN` 5148，合计 51480。没有其他候选拒绝码。飞机没有离开起点，所以最小障碍距离等于起点离地 1.5 m。

## 怎么重跑

`out/` 在 `.gitignore` 里。批跑写到 `out/`，再把 `summary.md` 和 `results.json` 抄进 `docs/results/`。已跟踪文件有改动时，运行器会在 commit 后面加 `-dirty`（本页 certified 表就是这样：当时只改了 README）。在干净工作区重跑，后缀会消失，墙钟会变，到达、碰撞和规划码不该变。本页飞行结果对应票 02 的代码；本分支上的提交只动文档。要给表盖上新的 sha，在干净的 HEAD 上重跑即可。

```bash
make venv
git rev-parse HEAD
.venv/bin/python -c 'from yopo_l0.scene import load_card; print(load_card("scenes/g2-pillar.json").hash)'

# M0 主表：场景卡自带的 10 个 seed，默认 upstream + 影子盾开，终端 stop_at_goal
.venv/bin/python -m yopo_l0.batch scenes/g2-pillar.json@upstream --jobs 4 \
  --out out/m0-upstream

# 对照：certified 全时长（场景卡 45 s）。全部门单步大约 2 s 墙钟，单 seed 十几分钟。
# --no-determinism 省掉 seed 0 的第二次全时长；短时确定性见 tests/test_policies.py。
.venv/bin/python -m yopo_l0.batch scenes/g2-pillar.json@certified --jobs 4 --no-determinism \
  --out out/m0-certified

# 合 main 回归：pytest + hover + 上面的 upstream 10 seed + certified 单 seed 仿真 4 s 冒烟。
# 4 s 冒烟覆盖不了「10 seed 全时长到达 0」；那一行用上面的 certified 批跑。
scripts/regress.sh
```

`results.json` 里每条记录的 `commit` 和 `scene_card_hash` 应与本页出处一致（commit 随 HEAD 变）。墙钟列会随机器变。

`hold_here` 对照不在已提交的场景卡里。把 `scenes/g2-pillar.json` 拷到临时目录（同目录放一份 `g2-scene.json`），只改 `name` 和 `policy_params.upstream`：

```json
"upstream": {
  "plan_from_reference": true,
  "arrive_mode": "hold_here",
  "arrive_radius_m": 1.5,
  "stop_time_s": 2.0
}
```

半径 5.0 再跑一版，得到 [ADR 0009](../adr/0009-stop-at-goal-vs-hold-here.md) 的两行。覆盖后的场景卡哈希分别是 `01a51d52f6f205424114bcfc95df215fe4bf09bb91677836a4000d9126351564`（1.5 m）和 `add5115e23baa97fb3272c5fbdfa2d3fa333cde8fd6c7db532ffa7c7dbf7742a`（5.0 m）。

## hold_here 对照（未提交的参数覆盖）

同一几何、同一 commit、影子盾开、仿真 45 s。两行都是 0/10 到达、碰撞 0。

### 半径 1.5 m

进入半径后停住，终点误差约 1.4 m，进不了 0.4 m。最小障碍距离与默认 `stop_at_goal` 相同，走廊是飞过的。

| seed | 结果 | 到达时间 s | 最小障碍距离 m | 终点误差 m | 仿真 s | 规划码 |
|---|---|---|---|---|---|---|
| 0 | TIMEOUT | — | 0.829 | 1.461 | 45.0 | ARRIVED_HOLD:429, OK:21 |
| 1 | TIMEOUT | — | 0.829 | 1.460 | 45.0 | ARRIVED_HOLD:429, OK:21 |
| 2 | TIMEOUT | — | 0.829 | 1.460 | 45.0 | ARRIVED_HOLD:429, OK:21 |
| 3 | TIMEOUT | — | 0.829 | 1.460 | 45.0 | ARRIVED_HOLD:429, OK:21 |
| 4 | TIMEOUT | — | 0.829 | 1.460 | 45.0 | ARRIVED_HOLD:429, OK:21 |
| 5 | TIMEOUT | — | 0.828 | 1.430 | 45.0 | ARRIVED_HOLD:429, OK:21 |
| 6 | TIMEOUT | — | 0.828 | 1.431 | 45.0 | ARRIVED_HOLD:429, OK:21 |
| 7 | TIMEOUT | — | 0.828 | 1.430 | 45.0 | ARRIVED_HOLD:429, OK:21 |
| 8 | TIMEOUT | — | 0.828 | 1.430 | 45.0 | ARRIVED_HOLD:429, OK:21 |
| 9 | TIMEOUT | — | 0.829 | 1.459 | 45.0 | ARRIVED_HOLD:429, OK:21 |

### 半径 5.0 m（上游到达半径）

起点到目标正好 5 m。第一次规划之后就 `hold_here`，终点误差保持 5 m，飞机留在起点。

| seed | 结果 | 到达时间 s | 最小障碍距离 m | 终点误差 m | 仿真 s | 规划码 |
|---|---|---|---|---|---|---|
| 0 | TIMEOUT | — | 1.497 | 5.000 | 45.0 | ARRIVED_HOLD:449, OK:1 |
| 1 | TIMEOUT | — | 1.497 | 5.000 | 45.0 | ARRIVED_HOLD:449, OK:1 |
| 2 | TIMEOUT | — | 1.497 | 5.000 | 45.0 | ARRIVED_HOLD:449, OK:1 |
| 3 | TIMEOUT | — | 1.497 | 5.000 | 45.0 | ARRIVED_HOLD:449, OK:1 |
| 4 | TIMEOUT | — | 1.497 | 5.000 | 45.0 | ARRIVED_HOLD:449, OK:1 |
| 5 | TIMEOUT | — | 1.497 | 5.000 | 45.0 | ARRIVED_HOLD:449, OK:1 |
| 6 | TIMEOUT | — | 1.497 | 5.000 | 45.0 | ARRIVED_HOLD:449, OK:1 |
| 7 | TIMEOUT | — | 1.497 | 5.000 | 45.0 | ARRIVED_HOLD:449, OK:1 |
| 8 | TIMEOUT | — | 1.497 | 5.000 | 45.0 | ARRIVED_HOLD:449, OK:1 |
| 9 | TIMEOUT | — | 1.497 | 5.000 | 45.0 | ARRIVED_HOLD:449, OK:1 |

原始记录：[m0-hold-here.json](m0-hold-here.json)。
