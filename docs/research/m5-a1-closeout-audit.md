# M5-A1：口径 A 第一期收口前置核验

本票只加只读核验，供口径 A 的第一期封版做前置核对。第一期是 L0 可回归闭环加上 M0–M5 交付包。SITL 实飞和蒸馏归二期。

新增文件只有 `scripts/check_phase1_closeout.py`、`tests/test_check_phase1_closeout.py` 和本记录。没有改 `yopo_l0/`、`core/`、`scenes/`、`docs/SPEC.md`、`docs/LIMITATIONS.md`、`docs/tickets.md`、`docs/results/SUMMARY.md`、`README.md`、`scripts/premerge.sh`、`scripts/regress.sh`。

## 怎么跑

```bash
python scripts/check_phase1_closeout.py
pytest tests/test_check_phase1_closeout.py
```

默认是静态检查：场景卡和 `scenes/suite.txt` 的默认开关、总表 / 视频索引 / 已知限制里的场景卡哈希和相对链接、`core/VENDORED.sha256` 的字节。不跑全量批跑。

`--with-smoke` 才额外跑 `scenes/g2-hover.json` 0.2 秒，确认这次进程的默认仍是 `upstream` 且影子盾开。这次记录用的是上面的默认静态命令。

退出码 0 表示没有硬破坏。场景卡把研究开关写成 JSON true、相对链接或套件路径断了、钉定哈希缺失或对不上、运行器默认不再是 upstream + 影子盾开，退出码才是 1。旧 commit 和二期外延写在清单里，不挡退出码。

## 这次 main

在 HEAD `f15354a556acbec51c19a5df5b791b6d41a4e3fc` 上，`python scripts/check_phase1_closeout.py` 退出码 0。`pytest tests/test_check_phase1_closeout.py` 24 通过。已跟踪文件没有未提交改动（核验当时工作区里只有本票的新文件，未计入）。

通过的底座：

- 运行器、`yopo_l0.run`、批跑和 ROS 2 入口的默认仍是 `upstream` + 影子盾开。`scripts/regress.sh` 的套件批跑没有 `--no-shadow`。certified 冒烟那一行显式关掉影子盾，只作对照。
- `scenes/*.json` 38 份，其中飞行卡 28 张。`track_clearance`、`empty_brake`、`peer_sphere_enabled`、`swarm_penalty`、`swarm_segment_penalty` 都不是 JSON true。影子盾没有被写成关。写了 `safe_radius` 的卡仍是 0.05 m。
- `scenes/suite.txt` 17 条都能落到文件。任务策略是 hover、upstream、formation、crossing、switch。套件里没有 certified 或 student。
- `core/VENDORED.md` 指向 `core/VENDORED.sha256`，59 个钉定文件字节一致。清单之外是 `core/tests/test_protocol_delay_check.py`，VENDORED.md 已写明它不在原始清单。
- `docs/results/SUMMARY.md`、`docs/results/videos.md`、`docs/LIMITATIONS.md` 的相对链接有效，文档里写到的 `scenes/` 路径都存在。总表 8 张场景卡的全长哈希与当前 `yopo_l0.scene.load_card` 一致，SUMMARY.md 里的 12 位短哈希是其前缀。

待刷新：SUMMARY.md 总表 13 行、文首和复现命令，以及 videos.md 的 8 条，commit 仍是 `f5985e4335fc`（完整 `f5985e4335fcc00a4e19c861dcd1a05e0113f797`）。这 13 行都匹配旧提交示例 `f5985e4`。场景卡哈希仍然对得上，要刷新的是出处 commit。正式封版若要让总表 commit 等于 HEAD，需要重跑或改写出处。本票不改这些文档。

二期外延：SITL 实飞（票 09 已关，S1–S4 只报数）、蒸馏（`student` 默认不选），以及套件外的三张研究卡。`scenes/g2-pillar-bspline.json` 和 `scenes/crossing-forest-dense-bspline.json` 打开后段，`scenes/swap-3-geom-fallback.json` 打开几何回退。三张都不在 `suite.txt`，五个命名研究开关仍然关着。

硬破坏：无。

## 脚本输出

下面是同一次 `python scripts/check_phase1_closeout.py --format md` 从「通过项」起的原文。

## 通过项

- **运行器默认是 upstream + 影子盾开**（`runner-default`）。RunConfig.policy 是 upstream，RunConfig.shadow 是 True。yopo_l0.run、批跑和 ROS 2 入口的默认与这一条一致。
- **合入前套件批跑保持影子盾默认开**（`regress-suite`）。套件批跑没有 --no-shadow，沿用运行器默认。紧随的 certified 冒烟显式 --no-shadow，只作对照，不改变套件默认。
- **场景卡没有打开五个研究开关**（`research-switches-off`）。track_clearance、empty_brake、peer_sphere_enabled、swarm_penalty、swarm_segment_penalty 都不是 JSON true。影子盾没有被写成关。写了 safe_radius 的卡仍是 0.05 m，没写的沿用上游缺省。
- **suite.txt 指向存在的场景卡，且没有打开研究开关**（`suite`）。共 17 条。任务策略：crossing、formation、hover、switch、upstream。不写 @策略时批跑默认 upstream。套件里没有 certified 或 student。
- **core 钉定哈希完好**（`vendored-pin`）。VENDORED.md 指向 VENDORED.sha256，59 个文件字节一致。清单之外还有 1 个文件（含票 13d 写明不在原始清单里的测试），不改写钉定字节。
- **交付文档的相对链接和场景路径有效**（`doc-links`）。SUMMARY.md、videos.md、LIMITATIONS.md 的相对链接都能落到文件。这三份加上 SUMMARY.json 里写到的 scenes/ 路径都存在。
- **总表场景卡哈希与当前场景卡一致**（`scene-card-hashes`）。8 张卡的全长哈希等于 SUMMARY.json，SUMMARY.md 的短哈希是其前缀。

## 待刷新项

- **交付索引仍引用旧 commit**（`stale-commits`）。SUMMARY.md 总表有 13 行的 commit 不是当前 HEAD（其中 13 行匹配旧提交示例 f5985e4）。SUMMARY.json 的 commit 是 `f5985e4335fcc00a4e19c861dcd1a05e0113f797`。正文或复现命令还引用了 `f5985e4335fc`、`f5985e4335fcc00a4e19c861dcd1a05e0113f797`。videos.md 有 8 条仍停在旧 commit。这些行的场景卡哈希仍与当前卡一致，所以要刷新的是出处 commit，不是哈希断档。

| 行 | 里程碑 | 场景 | commit | 哈希仍匹配 | 旧提交示例 f5985e4 |
|---|---|---|---|---|---|
| 18 | M0 | g2-pillar | `f5985e4335fc` | 是 | 是 |
| 19 | M1 | formation-hold | `f5985e4335fc` | 是 | 是 |
| 20 | M2 | crossing-pillars | `f5985e4335fc` | 是 | 是 |
| 21 | M2 | crossing-forest | `f5985e4335fc` | 是 | 是 |
| 22 | M3 | formation-switch | `f5985e4335fc` | 是 | 是 |
| 23 | M4 | formation-6-open | `f5985e4335fc` | 是 | 是 |
| 24 | M4 | formation-6-open | `f5985e4335fc` | 是 | 是 |
| 25 | M4 | formation-6-open | `f5985e4335fc` | 是 | 是 |
| 26 | M4 | formation-6-open | `f5985e4335fc` | 是 | 是 |
| 27 | M4 | formation-6-open | `f5985e4335fc` | 是 | 是 |
| 28 | M4 | formation-6-open | `f5985e4335fc` | 是 | 是 |
| 29 | M4 | crossing-6-forest | `f5985e4335fc` | 是 | 是 |
| 30 | M4 | crossing-6-forest-sparse | `f5985e4335fc` | 是 | 是 |


| 行 | 场景 | commit | 旧提交示例 f5985e4 |
|---|---|---|---|
| 8 | 代码 commit | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` | 是 |
| 16 | scenes/g2-pillar.json@upstream | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` | 是 |
| 17 | scenes/formation-hold.json@formation | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` | 是 |
| 18 | scenes/crossing-pillars.json@crossing | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` | 是 |
| 19 | scenes/crossing-forest.json@crossing | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` | 是 |
| 20 | scenes/formation-switch.json@switch | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` | 是 |
| 21 | scenes/formation-6-open.json@formation | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` | 是 |
| 22 | scenes/crossing-6-forest.json@crossing | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` | 是 |


## 二期外延项

- **SITL 实飞归二期**（`sitl-flight`）。口径 A 的第一期是 L0 可回归闭环和 M0–M5 交付包。实飞，以及把本仓库 upstream 接到 PX4 SITL 的闭环抽检，归二期。票 09 已关闭。S1–S4 的冒烟和桥只报数，不作为本次硬门。
- **蒸馏归二期**（`distill`）。student 策略默认不选，不进 scenes/suite.txt，也不改 RunConfig 的默认。权重和教师数据在仓库外。第一期封版不把蒸馏跑数当成硬门。
- **研究开关实验臂不进第一期默认基线**（`research-arms`）。这五个开关的打开态、student 蒸馏和 SITL 实飞都不进第一期默认套件。后段或几何回退若只出现在套件外的研究卡上，不把它们当成默认基线。当前套件外的研究卡：scenes/crossing-forest-dense-bspline.json（backend.enabled）；scenes/g2-pillar-bspline.json（backend.enabled）；scenes/swap-3-geom-fallback.json（geometric_fallback.enabled）。

## 硬破坏

- （无）


## JSON

同一次 `python scripts/check_phase1_closeout.py --format json`。`root` 写成 `.`，表示脚本所在仓库。

```json
{
  "schema": "yopo-restart/phase1-closeout/1",
  "ok": true,
  "exit_code": 0,
  "head": "f15354a556acbec51c19a5df5b791b6d41a4e3fc",
  "tracked_dirty": false,
  "mode": "static",
  "root": ".",
  "note": "退出码 0 表示没有硬破坏。待刷新（旧 commit、哈希漂移）和二期外延不挡退出码。",
  "passed": [
    {
      "id": "runner-default",
      "title": "运行器默认是 upstream + 影子盾开",
      "detail": "RunConfig.policy 是 upstream，RunConfig.shadow 是 True。yopo_l0.run、批跑和 ROS 2 入口的默认与这一条一致。"
    },
    {
      "id": "regress-suite",
      "title": "合入前套件批跑保持影子盾默认开",
      "detail": "套件批跑没有 --no-shadow，沿用运行器默认。紧随的 certified 冒烟显式 --no-shadow，只作对照，不改变套件默认。"
    },
    {
      "id": "research-switches-off",
      "title": "场景卡没有打开五个研究开关",
      "detail": "track_clearance、empty_brake、peer_sphere_enabled、swarm_penalty、swarm_segment_penalty 都不是 JSON true。影子盾没有被写成关。写了 safe_radius 的卡仍是 0.05 m，没写的沿用上游缺省。",
      "scene_json": 38,
      "flight_cards": 28
    },
    {
      "id": "suite",
      "title": "suite.txt 指向存在的场景卡，且没有打开研究开关",
      "detail": "共 17 条。任务策略：crossing、formation、hover、switch、upstream。不写 @策略时批跑默认 upstream。套件里没有 certified 或 student。",
      "entries": 17,
      "policies": [
        "crossing",
        "formation",
        "hover",
        "switch",
        "upstream"
      ]
    },
    {
      "id": "vendored-pin",
      "title": "core 钉定哈希完好",
      "detail": "VENDORED.md 指向 VENDORED.sha256，59 个文件字节一致。清单之外还有 1 个文件（含票 13d 写明不在原始清单里的测试），不改写钉定字节。",
      "pinned": 59,
      "extras": [
        "tests/test_protocol_delay_check.py"
      ]
    },
    {
      "id": "doc-links",
      "title": "交付文档的相对链接和场景路径有效",
      "detail": "SUMMARY.md、videos.md、LIMITATIONS.md 的相对链接都能落到文件。这三份加上 SUMMARY.json 里写到的 scenes/ 路径都存在。"
    },
    {
      "id": "scene-card-hashes",
      "title": "总表场景卡哈希与当前场景卡一致",
      "detail": "8 张卡的全长哈希等于 SUMMARY.json，SUMMARY.md 的短哈希是其前缀。",
      "scenes": [
        "crossing-6-forest",
        "crossing-6-forest-sparse",
        "crossing-forest",
        "crossing-pillars",
        "formation-6-open",
        "formation-hold",
        "formation-switch",
        "g2-pillar"
      ]
    }
  ],
  "refresh": [
    {
      "id": "stale-commits",
      "title": "交付索引仍引用旧 commit",
      "detail": "SUMMARY.md 总表有 13 行的 commit 不是当前 HEAD（其中 13 行匹配旧提交示例 f5985e4）。SUMMARY.json 的 commit 是 `f5985e4335fcc00a4e19c861dcd1a05e0113f797`。正文或复现命令还引用了 `f5985e4335fc`、`f5985e4335fcc00a4e19c861dcd1a05e0113f797`。videos.md 有 8 条仍停在旧 commit。这些行的场景卡哈希仍与当前卡一致，所以要刷新的是出处 commit，不是哈希断档。",
      "summary_rows": [
        {
          "source": "docs/results/SUMMARY.md",
          "line": 18,
          "milestone": "M0",
          "scene": "g2-pillar",
          "policy": "upstream",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "9d385f4cfce5",
          "scene_card_hash_current": "9d385f4cfce543922961865d06ceeedd44afdddb52c706bf6e3da9929431d8ae",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/SUMMARY.md",
          "line": 19,
          "milestone": "M1",
          "scene": "formation-hold",
          "policy": "formation",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "1b1894f5864a",
          "scene_card_hash_current": "1b1894f5864a2f5f5fb0e06b7d44ec490acf8d012957829f0f599e19951c365e",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/SUMMARY.md",
          "line": 20,
          "milestone": "M2",
          "scene": "crossing-pillars",
          "policy": "crossing",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "1d39afbd7807",
          "scene_card_hash_current": "1d39afbd780740b9e925569a70b27b670d73184d9b487d3adb2473fdf1ed279e",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/SUMMARY.md",
          "line": 21,
          "milestone": "M2",
          "scene": "crossing-forest",
          "policy": "crossing",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "e66b39061158",
          "scene_card_hash_current": "e66b39061158c23e4c8e8c3fd53bae40c799b3dabef6ef32dad0b94eb265fa0c",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/SUMMARY.md",
          "line": 22,
          "milestone": "M3",
          "scene": "formation-switch",
          "policy": "switch",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "4bff8a51c881",
          "scene_card_hash_current": "4bff8a51c881f676cc9ce8c83290d43354b861fd911acabb51acba55a0ec7ae3",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/SUMMARY.md",
          "line": 23,
          "milestone": "M4",
          "scene": "formation-6-open",
          "policy": "formation",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "a803d8021c77",
          "scene_card_hash_current": "a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/SUMMARY.md",
          "line": 24,
          "milestone": "M4",
          "scene": "formation-6-open",
          "policy": "formation",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "a803d8021c77",
          "scene_card_hash_current": "a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/SUMMARY.md",
          "line": 25,
          "milestone": "M4",
          "scene": "formation-6-open",
          "policy": "formation",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "a803d8021c77",
          "scene_card_hash_current": "a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/SUMMARY.md",
          "line": 26,
          "milestone": "M4",
          "scene": "formation-6-open",
          "policy": "formation",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "a803d8021c77",
          "scene_card_hash_current": "a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/SUMMARY.md",
          "line": 27,
          "milestone": "M4",
          "scene": "formation-6-open",
          "policy": "formation",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "a803d8021c77",
          "scene_card_hash_current": "a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/SUMMARY.md",
          "line": 28,
          "milestone": "M4",
          "scene": "formation-6-open",
          "policy": "formation",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "a803d8021c77",
          "scene_card_hash_current": "a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/SUMMARY.md",
          "line": 29,
          "milestone": "M4",
          "scene": "crossing-6-forest",
          "policy": "crossing",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "657efeb6fae1",
          "scene_card_hash_current": "657efeb6fae1a62ddc1b061d4cf04a3013f660d2e57b28f70a42d5c23c1db9ee",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/SUMMARY.md",
          "line": 30,
          "milestone": "M4",
          "scene": "crossing-6-forest-sparse",
          "policy": "crossing",
          "shadow": "开",
          "commit": "f5985e4335fc",
          "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "scene_card_hash_cited": "7e3328908444",
          "scene_card_hash_current": "7e3328908444d4615c34e217fbb871fec2bed1e2fcb451f8738c13ba2b24762a",
          "hash_matches_current": true,
          "matches_stale_example": true,
          "head_matches": false
        }
      ],
      "video_rows": [
        {
          "source": "docs/results/videos.md",
          "line": 8,
          "scene": "代码 commit",
          "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/videos.md",
          "line": 16,
          "scene": "scenes/g2-pillar.json@upstream",
          "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/videos.md",
          "line": 17,
          "scene": "scenes/formation-hold.json@formation",
          "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/videos.md",
          "line": 18,
          "scene": "scenes/crossing-pillars.json@crossing",
          "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/videos.md",
          "line": 19,
          "scene": "scenes/crossing-forest.json@crossing",
          "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/videos.md",
          "line": 20,
          "scene": "scenes/formation-switch.json@switch",
          "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/videos.md",
          "line": 21,
          "scene": "scenes/formation-6-open.json@formation",
          "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "matches_stale_example": true,
          "head_matches": false
        },
        {
          "source": "docs/results/videos.md",
          "line": 22,
          "scene": "scenes/crossing-6-forest.json@crossing",
          "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
          "matches_stale_example": true,
          "head_matches": false
        }
      ],
      "prose_commits": [
        "f5985e4335fc"
      ],
      "reproduce_checkouts": [
        "f5985e4335fcc00a4e19c861dcd1a05e0113f797"
      ],
      "summary_commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797"
    }
  ],
  "phase2": [
    {
      "id": "sitl-flight",
      "title": "SITL 实飞归二期",
      "detail": "口径 A 的第一期是 L0 可回归闭环和 M0–M5 交付包。实飞，以及把本仓库 upstream 接到 PX4 SITL 的闭环抽检，归二期。票 09 已关闭。S1–S4 的冒烟和桥只报数，不作为本次硬门。"
    },
    {
      "id": "distill",
      "title": "蒸馏归二期",
      "detail": "student 策略默认不选，不进 scenes/suite.txt，也不改 RunConfig 的默认。权重和教师数据在仓库外。第一期封版不把蒸馏跑数当成硬门。"
    },
    {
      "id": "research-arms",
      "title": "研究开关实验臂不进第一期默认基线",
      "detail": "这五个开关的打开态、student 蒸馏和 SITL 实飞都不进第一期默认套件。后段或几何回退若只出现在套件外的研究卡上，不把它们当成默认基线。当前套件外的研究卡：scenes/crossing-forest-dense-bspline.json（backend.enabled）；scenes/g2-pillar-bspline.json（backend.enabled）；scenes/swap-3-geom-fallback.json（geometric_fallback.enabled）。",
      "cards": [
        {
          "path": "scenes/crossing-forest-dense-bspline.json",
          "flags": [
            "backend.enabled"
          ],
          "in_suite": false
        },
        {
          "path": "scenes/g2-pillar-bspline.json",
          "flags": [
            "backend.enabled"
          ],
          "in_suite": false
        },
        {
          "path": "scenes/swap-3-geom-fallback.json",
          "flags": [
            "geometric_fallback.enabled"
          ],
          "in_suite": false
        }
      ]
    }
  ],
  "hard_failures": [],
  "summary_rows_on_old_commit": [
    {
      "source": "docs/results/SUMMARY.md",
      "line": 18,
      "milestone": "M0",
      "scene": "g2-pillar",
      "policy": "upstream",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "9d385f4cfce5",
      "scene_card_hash_current": "9d385f4cfce543922961865d06ceeedd44afdddb52c706bf6e3da9929431d8ae",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/SUMMARY.md",
      "line": 19,
      "milestone": "M1",
      "scene": "formation-hold",
      "policy": "formation",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "1b1894f5864a",
      "scene_card_hash_current": "1b1894f5864a2f5f5fb0e06b7d44ec490acf8d012957829f0f599e19951c365e",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/SUMMARY.md",
      "line": 20,
      "milestone": "M2",
      "scene": "crossing-pillars",
      "policy": "crossing",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "1d39afbd7807",
      "scene_card_hash_current": "1d39afbd780740b9e925569a70b27b670d73184d9b487d3adb2473fdf1ed279e",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/SUMMARY.md",
      "line": 21,
      "milestone": "M2",
      "scene": "crossing-forest",
      "policy": "crossing",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "e66b39061158",
      "scene_card_hash_current": "e66b39061158c23e4c8e8c3fd53bae40c799b3dabef6ef32dad0b94eb265fa0c",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/SUMMARY.md",
      "line": 22,
      "milestone": "M3",
      "scene": "formation-switch",
      "policy": "switch",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "4bff8a51c881",
      "scene_card_hash_current": "4bff8a51c881f676cc9ce8c83290d43354b861fd911acabb51acba55a0ec7ae3",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/SUMMARY.md",
      "line": 23,
      "milestone": "M4",
      "scene": "formation-6-open",
      "policy": "formation",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "a803d8021c77",
      "scene_card_hash_current": "a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/SUMMARY.md",
      "line": 24,
      "milestone": "M4",
      "scene": "formation-6-open",
      "policy": "formation",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "a803d8021c77",
      "scene_card_hash_current": "a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/SUMMARY.md",
      "line": 25,
      "milestone": "M4",
      "scene": "formation-6-open",
      "policy": "formation",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "a803d8021c77",
      "scene_card_hash_current": "a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/SUMMARY.md",
      "line": 26,
      "milestone": "M4",
      "scene": "formation-6-open",
      "policy": "formation",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "a803d8021c77",
      "scene_card_hash_current": "a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/SUMMARY.md",
      "line": 27,
      "milestone": "M4",
      "scene": "formation-6-open",
      "policy": "formation",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "a803d8021c77",
      "scene_card_hash_current": "a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/SUMMARY.md",
      "line": 28,
      "milestone": "M4",
      "scene": "formation-6-open",
      "policy": "formation",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "a803d8021c77",
      "scene_card_hash_current": "a803d8021c7774b3915c4b4f7c574b03ed73de67e3d7cb5ac1880cf0c9cca95d",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/SUMMARY.md",
      "line": 29,
      "milestone": "M4",
      "scene": "crossing-6-forest",
      "policy": "crossing",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "657efeb6fae1",
      "scene_card_hash_current": "657efeb6fae1a62ddc1b061d4cf04a3013f660d2e57b28f70a42d5c23c1db9ee",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/SUMMARY.md",
      "line": 30,
      "milestone": "M4",
      "scene": "crossing-6-forest-sparse",
      "policy": "crossing",
      "shadow": "开",
      "commit": "f5985e4335fc",
      "commit_full": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "scene_card_hash_cited": "7e3328908444",
      "scene_card_hash_current": "7e3328908444d4615c34e217fbb871fec2bed1e2fcb451f8738c13ba2b24762a",
      "hash_matches_current": true,
      "matches_stale_example": true,
      "head_matches": false
    }
  ],
  "video_rows_on_old_commit": [
    {
      "source": "docs/results/videos.md",
      "line": 8,
      "scene": "代码 commit",
      "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/videos.md",
      "line": 16,
      "scene": "scenes/g2-pillar.json@upstream",
      "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/videos.md",
      "line": 17,
      "scene": "scenes/formation-hold.json@formation",
      "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/videos.md",
      "line": 18,
      "scene": "scenes/crossing-pillars.json@crossing",
      "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/videos.md",
      "line": 19,
      "scene": "scenes/crossing-forest.json@crossing",
      "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/videos.md",
      "line": 20,
      "scene": "scenes/formation-switch.json@switch",
      "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/videos.md",
      "line": 21,
      "scene": "scenes/formation-6-open.json@formation",
      "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "matches_stale_example": true,
      "head_matches": false
    },
    {
      "source": "docs/results/videos.md",
      "line": 22,
      "scene": "scenes/crossing-6-forest.json@crossing",
      "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
      "matches_stale_example": true,
      "head_matches": false
    }
  ],
  "smoke": null,
  "counts": {
    "passed": 7,
    "refresh": 1,
    "phase2": 3,
    "hard": 0,
    "summary_rows_on_old_commit": 13,
    "video_rows_on_old_commit": 8
  }
}
```
