# 票 06c drop 20% 首个 COMMIT

口径与票 11a/11b 重跑开阔区相同：`scenes/formation-6-open.json`，进程内，影子盾开，seeds 0–4，档位 `scenes/formation-6-tiers.json`。判据未冻结。机间碰撞按中心距 ≤ 2×机体半径（0.70 m）计。

```bash
.venv/bin/python -m yopo_l0.batch scenes/formation-6-open.json@formation \
  --form inprocess --tiers scenes/formation-6-tiers.json --jobs 4 --no-determinism
```

补齐前是 `3d91e5275ced`（06b 已在树上）。补齐前的结果里没有 `fill_requests` 字段。

## 原因

不是重试上限，也不是 NACK。`max_failed_rounds` 在编队里是 10000，节点保持 ACTIVE，没有 `halt`。投票里的 NACK 数是 0。放弃原因全部是 `DEADLINE`：提案只发一次，缺的 ACK 没有第二张，本机一直等到 `t_start` 才 RELEASE。下一轮要等整圈窗口（六机 × 0.5 s = 3 s）。

drop 20%、seed 0（与上表到达 3/6、`local_commits` 一致；总线种子不随影子盾改变）：

| 飞机 | 事件 | 提交 t_start s | 结果 |
|---|---|---|---|
| uav0 | propose 0.5 → commit | 0.5 | 按时出发 |
| uav1 | 9 次 propose/DEADLINE，然后 commit | 28.0 | 提交了，40 s 内未到终点 |
| uav2 | 4 次 DEADLINE，然后 commit | 13.5 | 到达 |
| uav3 | 2 次 DEADLINE，然后 commit | 8.0 | 到达 |
| uav4 | 7 次 DEADLINE，然后 commit | 23.5 | 提交了，40 s 内未到终点 |
| uav5 | propose 3.0 / 6.0 / 9.0 均 DEADLINE，之后 10 个窗口 `skip NO_CANDIDATE` | — | `local_commits` 0，停在起点 |

uav5 在 `t_start` 6 s 和 9 s 时追赶还能产出轨迹，ACK 仍被 DEADLINE 丢掉。到 12 s 起 `iter_catchup` 一条都产不出：本槽时刻表的世界系速度峰值约 6.3 m/s，追赶用的软限幅是 4 m/s、3 m/s²，接入点全部超限。所以不是邻机冲突把提案判掉，是补齐窗口已经没有可提的计划。外槽错过同步出发后就补不回首个 COMMIT。

## 修复

窗口内对还没 ACK 的邻机重发**同一份**在途提案（同一个 `reservation_id`）。先等 100 ms（盖住故障档时延上界 0.05 s 的一来一回），再按 20 ms 起、40 ms 封顶退避，最多 8 次，截止前留 20 ms。不新开预留。对端预留仍只在 COMMIT 或 RELEASE 时释放，不因本地超时释放。动力学仍只跟踪本机 `executing()` 的已提交计划。这一轮补齐全部失败时，仍走 06b 的 DEADLINE RELEASE 和下一窗口追赶。

## 前后对比

六档合计。碰撞列是 seed 上的碰撞次数之和。最小机间距离取各 seed 的最小。队形误差 p95 取各 seed 的最大。

| 档位 | 到达率 前 | 到达率 后 | 碰撞 前 | 碰撞 后 | 最小机间距离 m 前 | 后 | 队形误差 p95 m 前 | 后 |
|---|---|---|---|---|---|---|---|---|
| none | 1.000 | 1.000 | 0 | 0 | 2.980 | 2.980 | 0.066 | 0.066 |
| drop 5% | 1.000 | 1.000 | 0 | 0 | 1.618 | 2.980 | 11.163 | 0.066 |
| drop 20% | 0.533 | 1.000 | 0 | 0 | 1.179 | 2.980 | 24.676 | 0.066 |
| duplicate 10% | 1.000 | 1.000 | 0 | 0 | 2.980 | 2.980 | 0.066 | 0.066 |
| reorder+delay | 1.000 | 1.000 | 0 | 0 | 2.980 | 2.980 | 0.066 | 0.066 |
| combined | 0.967 | 1.000 | 0 | 0 | 1.179 | 2.980 | 13.266 | 0.066 |

drop 20% 分 seed：

| seed | 到达率 前 | 后 | 碰撞 | `local_commits` = 0 前 | 后 | 补齐请求 后 |
|---|---|---|---|---|---|---|
| 0 | 0.500 | 1.000 | 0 | uav5 | 无 | 8 |
| 1 | 0.667 | 1.000 | 0 | uav0 uav2 | 无 | 13 |
| 2 | 0.667 | 1.000 | 0 | uav3 uav5 | 无 | 3 |
| 3 | 0.333 | 1.000 | 0 | uav5 | 无 | 5 |
| 4 | 0.500 | 1.000 | 0 | uav1 uav3 | 无 | 10 |

补齐后六个 seed、六档的提交时刻都是同步出发的 0.5…3.0 s，`deadline_aborts` 为 0。零故障、duplicate、reorder+delay 的补齐请求是 0，消息序列不因这条路径增加。影子拒绝率六档前后都是 1.000（零故障也是），直方图主要是 `ENV:UNKNOWN`，只报数，不是机间碰撞。
