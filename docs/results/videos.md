# 交付视频（票 12c）

视频不进仓库：`out/` 已在 `.gitignore`。成片写在 `out/delivery/`，本表指向这些路径。用 `scripts/render_delivery.sh` 在当前检出上重渲。
另有一份复制在 `/workspace/yopo-delivery-videos/`。

| 项 | 值 |
|---|---|
| 代码 commit | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` |
| 层级 | L0 |
| seed | 0 |
| 播放时长上限 | 60 s |
| 渲染器 | `python -m yopo_l0.render`（票 04）。字幕叠里程碑、层级、commit、seed。 |

| 文件 | 时长 s | 大小 | 场景 | 层级 | commit |
|---|---|---|---|---|---|
| `M0-g2-pillar-upstream-seed0.mp4` | 5.20 | 34.7 KiB | scenes/g2-pillar.json@upstream | L0 | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` |
| `M1-formation-hold-seed0.mp4` | 19.60 | 73.4 KiB | scenes/formation-hold.json@formation | L0 | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` |
| `M2-crossing-pillars-seed0.mp4` | 8.00 | 55.0 KiB | scenes/crossing-pillars.json@crossing | L0 | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` |
| `M2-crossing-forest-medium-seed0.mp4` | 24.30 | 94.0 KiB | scenes/crossing-forest.json@crossing | L0 | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` |
| `M3-formation-switch-seed0.mp4` | 28.00 | 116.1 KiB | scenes/formation-switch.json@switch | L0 | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` |
| `M4-formation-6-open-seed0.mp4` | 21.10 | 95.6 KiB | scenes/formation-6-open.json@formation | L0 | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` |
| `M4-crossing-6-forest-seed0.mp4` | 27.10 | 181.6 KiB | scenes/crossing-6-forest.json@crossing | L0 | `f5985e4335fcc00a4e19c861dcd1a05e0113f797` |

每条都是 seed 0、层级 L0。M2、M4 各有两张场景卡，各渲一条。
