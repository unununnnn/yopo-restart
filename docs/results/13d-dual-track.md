# 票 13d 双轨 Delay Check

判据未冻结 / report-only。碰撞数照实写。不换 YOPO，不引入 Gurobi。偏离写在 [ADR 0012](../adr/0012-rmader-dual-track.md)。

新轨在现有 ACK/COMMIT 上多一段等待：PROPOSED → TENTATIVE（PROPOSE 已广播）→ DELAY_CHECK → COMMITTED。编队 `delay_check_s = 0.05` s，等于六档里时延上界。没走完 Delay Check 不替换 `committed`。Delay Check 期间若邻机轨迹和新轨冲突（含迟到才到的），RELEASE 新轨，本窗口再提一份新的 `plan_seq`。

窗口内重发来自已合入的票 06c（同一 reservation，0.02 s 一次，截止前 0.10 s 静默）。票齐之后不再重发，Delay Check 只等时间；等到 δ_DC 才提交。

穿林和变阵的 `delay_check_s` 保持 0。收齐 ACK 仍当拍提交，YOPO 候选和跟踪参数不动。

## 开阔区六档

`scenes/formation-6-open.json`，档位 `scenes/formation-6-tiers.json`，seeds 0–4，进程内，影子盾开。每机 `local_commits = 1`，`local_aborts = 0`。机间碰撞是 `INTER_DRONE`，这 30 次里是 0。

| 档位 | 到达率 | 机间碰撞 | 最小机间距离 m | 队形误差 p95 m |
|---|---|---|---|---|
| none | 1.000 | 0 | 2.980 | 0.066 |
| drop 5% | 1.000 | 0 | 2.980 | 0.066 |
| drop 20% | 1.000 | 0 | 2.980 | 0.066 |
| duplicate 10% | 1.000 | 0 | 2.980 | 0.066 |
| reorder+delay | 1.000 | 0 | 2.980 | 0.066 |
| combined | 1.000 | 0 | 2.980 | 0.066 |

## 林

`CrossingFleet.cfg.delay_check_s` 是 0，YOPO 和 `safe_radius` 0.05 m 没动。`scripts/regress.sh` 里的 `crossing-forest` medium、seed 0：到达 3/3，障碍碰撞 0，机间碰撞 0，最小障碍净空 0.351 m。这落在票 07d 中档净空 0.275–0.591 m 里。密档原有的撞树不在这条套件里，也不由 Delay Check 改写。
