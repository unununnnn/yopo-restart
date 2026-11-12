#!/usr/bin/env python3
"""D5b 收口：从落盘结果生成 docs/research/d5b-student-fix.md 和 d5b-student-fix-runs.json。只读真实结果文件。"""
import hashlib, json, os, sys
from pathlib import Path
WT = Path(sys.argv[1]); D = Path("/workspace/yopo-teacher-data/d5b")
J = lambda p: json.loads(Path(p).read_text())
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
base = J(D / "convert-base-manifest.json")
EV = [("D5 原样（D5 文档）", None), ("d5-fixed：D5 权重 + 接口修正", "d5-fixed"), ("r0：全量成功场", "r0"), ("r1", "r1"), ("r2", "r2"),
      ("r3：DAgger 三轮", "r3"), ("r3 + 定高（只换动作，不重训）", "r3-planar"), ("r4：只学水平 + DART + 加权（本票改法）", "r4-planar")]
D5DOC = {"g2-pillar": "0/10，撞 10，−0.037", "swap-3-open": "0/30，撞 10，−0.004", "open-3drone": "0/30，超时 10", "ego-density-014-solo": "0/10，撞 10，−0.007"}
SC = [("g2-pillar", "g2 柱子"), ("swap-3-open", "开阔三机对向"), ("open-3drone", "开阔三机平行"), ("ego-density-014-solo", "密度 0.14 单机")]
summ = {k: J(D / "eval" / k / "summary.json") for _, k in EV if k and (D / "eval" / k / "summary.json").is_file()}
def cell(row):
    if row is None: return "—"
    s = f"{row['arrived']}/{row['denom']}，撞障 {row['obstacle']}，超时 {row['timeout']}"
    if row.get("inter"): s += f"，机间 {row['inter']}"
    if row.get("min_clearance_m") is not None: s += f"，{row['min_clearance_m']:+.3f}"
    return s
L = []
A = L.append
A("# D5b：学生闭环接口修正、DAgger 与改训练（report-only）\n")
A("本票只报数。新选项全部默认关；不选时 `StudentPolicy` 与 D5 逐位相同，`student_fit` 默认训练与 D5 逐位相同。不改 `core/`、已冻门、`scenes/suite.txt`、`docs/tickets.md`、`README`、`yopo_l0/backend/`。权重和数据在仓库外 `/workspace/yopo-teacher-data/d5b/`，仓库只记 sha256。地图边长和障碍半径是假设，不是论文复现。\n")
A("评测四个场景，seeds 0–9。表里每格是“到达架次/总架次，撞障场数，超时场数，最小障碍间隙 m”。最小间隙是到障碍表面的距离减机体半径 0.35 m，负数就是碰上了。\n")
A("## 1. 结论\n")
A("- 接口：没有坐标、单位、归一化错误。闭环飞丢是三处语义不一致造成的：偏航来源、位置头和速度头不自洽、加速度含义。修正后速度不再发散，但 D5 权重仍到不了。")
A("- DAgger r1–r3 没有改善闭环，r3 四个场景全部撞障。留出 RMSE 和训练损失都在降，网络在碰撞前 0.5 s 的状态上也比 r0 更贴近 EGO 重标注。问题出在高度：教师重标注自己保留了竖直动量，学生飞在训练里没有的 1.5 m 高度，竖直速度一路累积，最后撞地或撞树顶。")
A("- 改训练选了 (1) 低维动作 + (2) DART 扰动重标注 + (3) 安全加权：网络只管水平，高度按目标高度指数保持；训练只算 x/y 损失；DAgger 和 DART 窗口 ×4，近障窗口 ×3。结果见第 6 节。")
def brief(k):
    if k not in summ: return "（未跑完）"
    return "、".join(f"{lab} {r['arrived']}/{r['denom']}" for (sc, lab), r in zip(SC, [{x['scene']: x for x in summ[k]['rows']}.get(sc) for sc, _ in SC]) if r)
A(f"- r4 到达：{brief('r4-planar')}；r3 是 {brief('r3')}；只给 r3 换定高动作、不重训：{brief('r3-planar')}。有障碍的两个场景（g2、密度 0.14）仍然全部擦碰，最小间隙在 −0.01 m 以内，是贴着障碍过去的。")
A("- 教师本身在密集格不可靠（第 7 节）。密度 0.14 的三机格里，DART 重标注有 27/118 场 EGO-Swarm 自己的最小间隙小于 0；重标注只能靠净空过滤兜底。\n")
A("## 2. 接口诊断（仓库外脚本，复制在 `scripts/distill/d5b/`）\n")
A("- (a) 输入：闭环里重建的状态和邻机特征与训练张量最大差 0.0；深度平均绝对差 0.0015（归一化单位）；网络输出差不超过 5e-5 m（d000）。")
A("- (b) 输出：教师强制回放下，计划与教师轨迹的位置 RMSE 在 0.1 s 时 0.005–0.012 m，0.5 s 时 0.019–0.064 m。")
A("- (c) 闭环（open-3drone seed 0）：`_TrajPlan.sample` 用 YOPO `calculate_yaw`（速度归一化、无死区、0.5π rad/s），0.6 s 偏航已转 −0.86 rad；网络是机体系输入，偏航一转就照旧往机体前方飞，7.8 s 时 29.6 m/s，8 s 末离终点 116 m。教师偏航来自 EGO-Swarm traj_server（看 1.0 s 后的点、π rad/s、位移 ≤ 0.1 m 保持），教师加速度是 fake drone 写入的指令加速度。\n")
A("新选项（`policy_params.student`，默认值即 D5 行为）：\n")
A("| 键 | 默认（D5） | 新值 |\n|---|---|---|")
A("| `knots` | `network`：位置头、速度头各自转世界系 | `position`：速度由位置结点差分得到，两者自洽 |")
A("| `yaw_source` | `calculate_yaw` | `ego_lookahead`：按 traj_server 规则 |")
A("| `accel_source` | `state`：用 L0 实际加速度起段 | `reference`：用上一条计划的参考加速度 |")
A("| `vertical` | `network`：结点高度取网络 z | `hold_goal_z`：结点高度按 `vertical_tau_s`（默认 1.0 s）指数趋近目标高度，网络只管水平 |\n")
A("## 3. 数据\n")
A(f"D2b 全量 {base['n_runs']} 场，成功 {base['n_success']} 场（D2c 统计：成功 1205、失败 553、未知 34），只转成功场，转成 {base['n_converted']} 场、{base['n_windows']} 个窗口。跳过 1 场：`d000-n6-headon-s60`（偏航相邻样本跳变超过 π）。D5 冻结 663 场里成功的 635 场全部在内，另外新增 {base['vs_d5']['new_success_not_in_d5']} 场。留出规则同 D5：`seed % 5 == 0`。清单（逐文件 sha256）在 `/workspace/yopo-teacher-data/d5b/convert-base-manifest.json`，sha256 `{sha(D / 'convert-base-manifest.json')}`。\n")
A("| 格 | 场次 | 成功 | 转换 | 窗口 | 其中 D5 已有 |\n|---|---|---|---|---|---|")
for k, c in base["per_cell"].items(): A(f"| {k} | {c['runs']} | {c['success']} | {c['converted']} | {c['windows']} | {c['in_d5']} |")
A("")
A("与 D5 的区别：多了六机格、密度 0.14 三机格、密度 0.28 格；这些格的成功率低（d014-n3-parallel 33/128，d028-n3-parallel 4/128），留下的是教师侥幸成功的场次。\n")
A("重标注窗口（DAgger r1–r3、DART）一律过净空过滤：未来 1.6 s 内到障碍表面净空 ≥ 0（已扣机体半径 0.20 m），机间 ≥ 0.40 m。每格用量：\n")
A("| 来源 | 格 | 重标注场次 | 可用场次 | 窗口 | 过滤后 |\n|---|---|---|---|---|---|")
dm = {}
for r in ("r1", "r2", "r3", "dart"):
    p = D / "windows-dagger" / r / "manifest.json"
    if not p.is_file(): continue
    dm[r] = J(p)
    for k, c in sorted(dm[r]["per_cell"].items()): A(f"| {r} | {k} | {c['runs']} | {c['usable_runs']} | {c['windows_all']} | {c['windows_kept']} |")
A("")
A("## 4. DAgger r0–r3\n")
A("| 轮 | 训练窗口文件 | 新增重标注窗口 | 留出位置 RMSE m | 训练损失（5 个 epoch） |\n|---|---|---|---|---|")
tr = {}
for r in ("r0", "r1", "r2", "r3"):
    t = J(D / r / "d5b-train.json"); tr[r] = t
    hist = [round(h.get("train_loss", 0), 5) for h in t.get("history") or []]
    A(f"| {r} | {t['data']['base_train_runs']} + {t['data'].get('dagger_files', 0)} | {dm[r]['n_windows'] if r in dm else 0} | {t['holdout']['position_rmse_m']:.4f} | {' → '.join(map(str, hist))} |")
A("")
A("## 5. DAgger 为什么没用（数字）\n")
A("- 数据量：三轮重标注一共约 27.8k 个窗口（r1 13805、r2 2838、r3 11114），基础训练集 420340 个，约 6.6%，等权混入。r2 只有 66 个作业，因为 r1 学生几乎场场在 2 s 内撞上，能截的状态少。")
A("- 重标注质量：EGO 从学生状态接手的速度误差中位数 0.009–0.016 m/s，接手没问题。净空过滤几乎没删窗口（r1 13995→13805，r3 11230→11114），但带障碍的重标注场里教师自己最小间隙小于 0 的有：r1 15/57、r2 4/29、r3 10/37。")
A("- 损失在降：留出位置 RMSE r0 0.0580 → r3 0.0566，每轮训练损失单调下降（r3：0.00996 → 0.00564）。所以不是没学会监督目标。")
A("- 碰撞时刻对比（`scripts/distill/d5b/diag_collide.py`，取 r3 碰撞前 0.5 s 的状态，用 EGO 重标注，比较 1.6 s 处位置误差）：open3 r0 1.77 m → r3 1.56 m，swap 2.79 → 0.68 m，density 0.64 → 0.31 m。r3 更像教师了。但教师在这些状态下自己也保持竖直动量：1.6 s 内高度变化 open3 −0.48 m、swap −0.67 m、density +0.61 m。")
A("- 失败形态：r3 的碰撞全是竖直方向。open3 uav2 在 1.5 s 时 z 0.52 m、vz −3.14 m/s；swap z 0.54 m、vz −2.39；g2 z 0.57 m、vz −3.24；density 爬到 z 2.39 m、vz +2.32 撞树顶（树高 2.5 m）。r0 高度一直在 1.5 m 附近。")
A("- 原因：教师数据全部飞在 1.0 m，评测场景飞 1.5 m，深度图里的地面线索在分布外；网络 z 头和 L0 一阶滞后叠加，竖直速度自己喂大，DAgger 标签又忠实地保留这个动量。r0→r1 只换了约 6% 的数据，闭环就从 open3 20/30、swap 29/30 变成全撞，说明闭环对重训非常敏感，单轮评测的好坏有很大偶然成分。")
A("- 训练细节：`train_from_paths` 每攒约 8 个 batch 才洗一次，一个 batch 基本来自 1–2 场，DAgger 窗口很难真正混进梯度。r4 把它调成 64（新选项 `shuffle_batches`，默认仍是 8）。\n")
A("## 6. 改训练：低维动作 + DART + 安全加权\n")
A("训练选项（`FitConfig`，默认全关）：`planar`（位置、速度损失只算 x/y）、`near_weight` / `near_depth_m`（深度图最近有效深度 < 1.2 m 的窗口 ×3）、`shuffle_batches`；`train_from_paths(path_weights=...)` 给 DAgger 与 DART 窗口 ×4。`planar` 时按留出 x/y 位置 MSE 选最好的 epoch。\n")
A("DART 数据（`scripts/distill/d5b/dart_jobs.py`）：从基础训练场的教师轨迹上随机取时刻，加扰动后交给 EGO-Swarm 从扰动状态接手：横向位置 σ 0.4 m，横向速度 σ 0.6 m/s，纵向速度 σ 0.3 m/s，偏航 σ 0.35 rad；起终点高度随机取 1.0 或 1.5 m。8 个格（d000/d014 × 单机/三机 × 平行/对向）各 60 场，共 480 个作业。\n")
r4 = D / "r4" / "d5b-train.json"
if r4.is_file():
    t4 = J(r4)
    A(f"r4 训练：基础 {t4['data']['base_train_runs']} 场 + 重标注文件 {t4['data']['extra_files']}，墙钟 {t4.get('wall_s', 0):.0f} s。留出 x/y 位置 RMSE {t4['holdout']['position_xy_rmse_m']:.4f} m（三维 {t4['holdout']['position_rmse_m']:.4f} m，z 不再训练，三维数字没有意义）。")
    xy3 = D / "r3" / "holdout-xy.json"
    if xy3.is_file(): A(f"同一留出集上 r3 的 x/y RMSE 是 {J(xy3)['position_xy_rmse_m']:.4f} m。")
    A(f"权重 `/workspace/yopo-teacher-data/d5b/r4/student.pt`，sha256 `{t4.get('weights_sha256')}`。\n")
A("评测时 r4 和对照组都用 `knots=position`、`yaw_source=ego_lookahead`、`accel_source=reference`、`vertical=hold_goal_z`；D5 文档一行是 D5 默认接口，d5-fixed 到 r3 是前三项修正、高度仍由网络给。\n")
A("| 设置 | " + " | ".join(l for _, l in SC) + " |\n|---|" + "---|" * len(SC))
for label, k in EV:
    if k is None:
        A(f"| {label} | " + " | ".join(D5DOC[s] for s, _ in SC) + " |"); continue
    if k not in summ: continue
    rows = {r["scene"]: r for r in summ[k]["rows"]}
    A(f"| {label} | " + " | ".join(cell(rows.get(s)) for s, _ in SC) + " |")
if "r4-planar" in summ:
    rr = {x["scene"]: x for x in summ["r4-planar"]["rows"]}
    A("\nr4 终点误差中位数：" + "，".join(f"{lab} {rr[sc]['final_goal_err_median_m']:.2f} m" for sc, lab in SC if sc in rr and rr[sc].get('final_goal_err_median_m') is not None) + "。开阔三机对向每场都有 1 架没到（20/30），终点误差中位数说明它是飞远了，不是停在附近。")
A("\n上游 EGO 风格基线（D5 文档）：g2 10/10，open3 30/30，density 9/10。\n")
A("## 7. 教师在密集格不可靠\n")
A("D2c：1792 场里失败 553 场，几乎都是撞障碍。成功率按格：d014-n3-headon 58/128，d014-n3-parallel 33/128，d028-n3-headon 14/128，d028-n3-parallel 4/128。重标注时 EGO-Swarm 从学生（或扰动）状态接手，同样经常擦树：")
import glob
for r in ("r1", "r2", "r3", "dart"):
    recs = [J(f) for f in glob.glob(str(D / "relabel" / "runs" / f"{r}-*" / "record.json"))]
    by = {}
    for x in recs:
        m = x.get("metrics") or {}; c = x["scenario"]["base"].rsplit("-s", 1)[0]
        if m.get("min_clearance_m") is None: continue
        b = by.setdefault(c, [0, 0]); b[0] += 1; b[1] += m["min_clearance_m"] < 0
    if by: A(f"- {r}：" + "；".join(f"{c} {v[1]}/{v[0]}" for c, v in sorted(by.items())) + "（教师最小间隙 < 0 的场次 / 带障碍的重标注场次）")
A("\n所以密集格里的重标注标签本身就包含擦树的动作。净空过滤只删掉未来 1.6 s 内真碰上的窗口，删不掉“贴着树飞”的习惯。\n")
A("## 8. 复现\n")
A("脚本是仓库外运维脚本的副本，路径写死为 box 上的目录，放在 `scripts/distill/d5b/`：`convert_full.py`（全量成功场转窗口）、`dagger.sh`（r0–r3 编排）、`relabel.py` + `ros/dagger_record.py`（EGO 重标注容器）、`dagger_convert.py`（净空过滤）、`dart_jobs.py`、`train_r4.py`、`planar.sh`（r4 编排）、`rollout.py` / `eval.py`（评测）、`teacher_forced.py`、`diag_collide.py`。逐场评测结果见 `docs/research/d5b-student-fix-runs.json`。\n")
A("## 9. 局限\n")
A("- 每个设置只训练一次、评测 10 个 seed；L0 噪声小，同一权重下各 seed 结果几乎一样，10 个 seed 不能代表重训方差。r0→r1 的翻转说明单次训练结论不稳。")
A("- `hold_goal_z` 是人为规定的高度，不是网络学出来的；它只适合定高飞行的场景。")
A("- 评测场景飞 1.5 m，教师基础数据全在 1.0 m，DART 只补了 1.5 m 的一部分扰动样本。\n")
(WT / "docs/research/d5b-student-fix.md").write_text("\n".join(L) + "\n", encoding="utf-8")
runs = {k: {"weights": v["weights"], "variant": v.get("variant"), "student_params": v.get("student_params"), "rows": v["rows"]} for k, v in summ.items()}
runs["_weights_sha256"] = {r: (J(D / r / "d5b-train.json").get("weights_sha256")) for r in ("r0", "r1", "r2", "r3", "r4") if (D / r / "d5b-train.json").is_file()}
runs["_dagger_manifest_sha256"] = {r: sha(D / "windows-dagger" / r / "manifest.json") for r in dm}
(WT / "docs/research/d5b-student-fix-runs.json").write_text(json.dumps(runs, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
print("doc written")
