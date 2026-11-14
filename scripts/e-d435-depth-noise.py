#!/usr/bin/env python3
"""票 E-D435：D435 深度噪声下的 YOPO L0 回归。只报数。

默认开关保持关闭。五条臂都在进程里写进内存卡的 ``task.d435_depth``，
不改 ``scenes/``、已冻门、``core/``、``yopo_l0/crossing.py``、``yopo_l0/sitl_bridge.py``。

- off：关。与现 main 同一条深度路径。
- noise：只开视差域噪声（轴向、横向、飞点、量化、每回合视差偏置）。
- holes：噪声 + 空洞/阴影（左侧无效带、遮挡阴影、远距离掉点、掠射、随机洞、假近点）。
- all：再加延迟队列（模型默认，大约 21 ms，另有丢帧和尖峰）。
- stress：全开，子像素噪声 s=0.20，延迟固定 50 ms（调研表上界）。

代表卡：g2 柱子单机、密度 0.14 单机、crossing-pillars 三机、
crossing-6-forest-sparse 六机、开阔三机编队（formation-hold）。seeds 0–9。

无效像素用 NaN。网络入口仍走上游 ``network_depth`` 的 NS inpaint。
影子盾只记录，不改计划。介入次数是 ``would_reject`` 的 tick 数。

``--jobs`` 用 spawn 并行。逐粒写到 ``out/e-d435-depth-noise/partial/``，重跑会接着做。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from yopo_l0.criteria import evaluate_card_rows
from yopo_l0.experiment import comma_ints, comma_list, markdown_table, run_spawn_jobs, unknown_values, write_json
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import SceneCard, load_card

PARTIAL = ROOT / "out" / "e-d435-depth-noise" / "partial"
ROW_VERSION = 1
SEEDS = tuple(range(10))
BODY_FALLBACK_M = 0.35
YOPO_SCENES = ("g2-pillar", "d014-solo", "crossing-pillars", "six-sparse")

SCENES = (
    {"id": "g2-pillar", "path": "scenes/g2-pillar.json", "label": "g2 柱子单机", "policy": "upstream"},
    {"id": "d014-solo", "path": "scenes/ego-density-014-solo.json", "label": "密度 0.14 单机", "policy": "upstream"},
    {"id": "crossing-pillars", "path": "scenes/crossing-pillars.json", "label": "crossing-pillars 三机", "policy": "crossing"},
    {"id": "six-sparse", "path": "scenes/crossing-6-forest-sparse.json", "label": "六机林带 sparse", "policy": "crossing"},
    {"id": "formation-hold", "path": "scenes/formation-hold.json", "label": "开阔三机编队", "policy": "formation"},
)
SCENE_BY_ID = {item["id"]: item for item in SCENES}

ARMS = ("off", "noise", "holes", "all", "stress")
ARM_SPEC = {
    "off": None,
    "noise": {"enabled": True, "noise": True, "holes": False, "latency": False},
    "holes": {"enabled": True, "noise": True, "holes": True, "latency": False},
    "all": {"enabled": True, "noise": True, "holes": True, "latency": True},
    "stress": {
        "enabled": True,
        "noise": True,
        "holes": True,
        "latency": True,
        "subpix": 0.20,
        "latency_ms": 50.0,
    },
}
ARM_LABEL = {
    "off": "基线（关）",
    "noise": "只开噪声",
    "holes": "噪声 + 空洞/阴影",
    "all": "全开（含延迟）",
    "stress": "压测 s=0.20、延迟 50 ms",
}

DISCLAIMER = (
    "只报数。report-only。``d435_depth`` 默认关，场景卡、``scenes/suite.txt``、已冻门都没改。"
    "打开即偏离上游：上游权重在真值深度上训练（右目关闭）。"
    "无效像素保持 NaN，网络侧用上游 NS inpaint；本表没有用 inpaint 预填，也没有填最大量程。"
    "飞点概率 0.3、掠射角阈值 75°、视差噪声相关长度 3 px、丢帧率 0.5%，"
    "以及约 21 ms 里 Orin 额外的约 5 ms，置信度低。"
    "仿真相机不是 848×480。噪声用该相机的 fx，米制 σ 比原生 D435 大。"
    "编队任务不读深度，轨迹不该因这只开关改变；影子盾仍会看着深度图。"
    "影子盾只记录 would_reject，不改批准的计划。墙钟不进结论。"
)


def focal_px(card: dict) -> float:
    camera = card["camera"]
    return (float(camera["width"]) / 2.0) / math.tan(math.radians(float(camera["hfov_deg"])) / 2.0)


def sigma_at_2m(fx: float, subpix: float = 0.08) -> float:
    return (2.0 ** 2) * subpix / (fx * 0.05)


def with_arm(card: SceneCard, arm: str) -> SceneCard:
    spec = ARM_SPEC[arm]
    if spec is None:
        return card
    raw = dict(card.card)
    task = dict(raw["task"])
    task["d435_depth"] = dict(spec)
    raw["task"] = task
    return SceneCard(card.path, raw, card.obstacles)


def _block(result: dict) -> dict:
    for key in ("crossing", "formation", "switch"):
        block = result.get(key)
        if isinstance(block, dict):
            return block
    return {}


def collision_fields(result: dict) -> dict:
    block = _block(result)
    radius = float(block.get("body_radius_m") or BODY_FALLBACK_M)
    if "obstacle_collisions" in block:
        obstacle = int(block["obstacle_collisions"])
        inter = int(block["inter_drone_collisions"])
    else:
        dist = result.get("min_obstacle_distance_m")
        gap = result.get("min_inter_drone_distance_m")
        obstacle = int(dist is not None and float(dist) <= radius + 1e-9)
        inter = int(gap is not None and float(gap) <= 2.0 * radius + 1e-9)
    clearance = block.get("min_obstacle_clearance_m")
    if clearance is None and result.get("min_obstacle_distance_m") is not None:
        clearance = float(result["min_obstacle_distance_m"]) - radius
    return {
        "body_radius_m": radius,
        "obstacle_collision": obstacle,
        "inter_collision": inter,
        "clearance_m": None if clearance is None else float(clearance),
    }


def slim_result(result: dict, ticks: list[dict]) -> dict:
    fields = collision_fields(result)
    shadow = sum(1 for row in ticks if row.get("would_reject"))
    kept = {
        "scene": result.get("scene"),
        "seed": result.get("seed"),
        "policy": result.get("policy"),
        "task": result.get("task"),
        "outcome": result.get("outcome"),
        "reason": result.get("reason"),
        "arrived": int(result.get("arrived") or 0),
        "n_drones": int(result.get("n_drones") or 0),
        "arrival_time_s": result.get("arrival_time_s"),
        "min_obstacle_distance_m": result.get("min_obstacle_distance_m"),
        "min_inter_drone_distance_m": result.get("min_inter_drone_distance_m"),
        "formation_error_p95_m": result.get("formation_error_p95_m"),
        "upstream_deviation": bool(result.get("upstream_deviation")),
        "safe_radius_m": result.get("safe_radius_m"),
        "forest_density": result.get("forest_density"),
        "shadow_reject_rate": result.get("shadow_reject_rate"),
        "shadow_reject_count": int(shadow),
        "wall_time_s": result.get("wall_time_s"),
        "d435_depth": result.get("d435_depth"),
    }
    block = _block(result)
    if block:
        slim = {
            key: block[key]
            for key in (
                "obstacle_collisions",
                "inter_drone_collisions",
                "min_obstacle_clearance_m",
                "body_radius_m",
                "error_p95_m",
                "frozen",
            )
            if key in block
        }
        kind = "crossing" if "crossing" in result else "formation" if "formation" in result else "switch"
        kept[kind] = slim
    kept.update(fields)
    return kept


def partial_path(scene: str, arm: str, seed: int) -> Path:
    return PARTIAL / f"{scene}__{arm}__s{seed}.json"


def read_partial(path: Path, duration_s: float | None) -> dict | None:
    if not path.is_file():
        return None
    try:
        row = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    if row.get("row_version") != ROW_VERSION or row.get("complete") is not True:
        return None
    if row.get("duration_s") != duration_s:
        return None
    return row


def write_partial(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(row, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(path)


def eval_row(row: dict) -> dict:
    """给冻结门槛用的一行。只留判据会读的字段。"""
    out = {
        "seed": row["seed"],
        "policy": row["policy"],
        "outcome": row["outcome"],
        "arrived": row["arrived"],
        "n_drones": row["n_drones"],
        "arrival_time_s": row["arrival_time_s"],
        "min_obstacle_distance_m": row["min_obstacle_distance_m"],
        "min_inter_drone_distance_m": row["min_inter_drone_distance_m"],
        "formation_error_p95_m": row["formation_error_p95_m"],
        "forest_density": row.get("forest_density"),
        "form": "inprocess",
    }
    for key in ("crossing", "formation", "switch"):
        if key in row:
            out[key] = row[key]
    return out


def run_one(spec: tuple) -> dict:
    scene_id, arm, seed, duration_s = spec
    path = partial_path(scene_id, arm, seed)
    cached = read_partial(path, duration_s)
    if cached is not None:
        print(f"resume {scene_id} {arm} seed {seed}", flush=True)
        return cached
    meta = SCENE_BY_ID[scene_id]
    card = load_card(ROOT / meta["path"])
    tuned = with_arm(card, arm)
    started = time.perf_counter()
    output = run(
        tuned,
        RunConfig(seed=int(seed), policy=meta["policy"], shadow=True, duration_s=duration_s),
    )
    row = slim_result(output.result, output.ticks)
    row.update({
        "row_version": ROW_VERSION,
        "complete": True,
        "scene_id": scene_id,
        "arm": arm,
        "card_path": meta["path"],
        "base_scene_hash": card.hash,
        "run_scene_hash": output.result.get("scene_card_hash"),
        "duration_s": duration_s,
        "deviates": arm != "off",
    })
    write_partial(path, row)
    elapsed = time.perf_counter() - started
    print(
        f"done {scene_id} {arm} seed {seed} {row['outcome']} "
        f"arrived {row['arrived']}/{row['n_drones']} {elapsed:.1f}s",
        flush=True,
    )
    return row


def aggregate(rows: list[dict]) -> dict:
    drones = sum(int(row["n_drones"]) for row in rows)
    arrived = sum(int(row["arrived"]) for row in rows)
    clearances = [float(row["clearance_m"]) for row in rows if row.get("clearance_m") is not None]
    gaps = [float(row["min_inter_drone_distance_m"]) for row in rows if row.get("min_inter_drone_distance_m") is not None]
    times = [float(row["arrival_time_s"]) for row in rows if row.get("arrival_time_s") is not None]
    return {
        "runs": len(rows),
        "arrived_drones": arrived,
        "drones": drones,
        "arrival_rate": None if not drones else arrived / drones,
        "obstacle_runs": sum(int(row["obstacle_collision"]) for row in rows),
        "inter_runs": sum(int(row["inter_collision"]) for row in rows),
        "min_clearance_m": None if not clearances else min(clearances),
        "median_clearance_m": None if not clearances else float(np.median(clearances)),
        "min_inter_m": None if not gaps else min(gaps),
        "mean_arrival_time_s": None if not times else float(np.mean(times)),
        "arrived_runs": len(times),
        "shadow_reject_count": sum(int(row["shadow_reject_count"]) for row in rows),
    }


def scene_mean_rate(groups: dict, scenes: tuple[str, ...], arm: str) -> float | None:
    rates = [
        float(groups[scene][arm]["arrival_rate"])
        for scene in scenes
        if groups[scene][arm]["arrival_rate"] is not None
    ]
    if not rates:
        return None
    return float(np.mean(rates))


def pooled_rate(groups: dict, scenes: tuple[str, ...], arm: str) -> float | None:
    arrived = drones = 0
    for scene in scenes:
        block = groups[scene][arm]
        arrived += int(block["arrived_drones"])
        drones += int(block["drones"])
    if not drones:
        return None
    return arrived / drones


def _fmt(value, digits=3) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{digits}f}"


def _rate_cell(block: dict) -> str:
    if not block["drones"]:
        return "—"
    return f"{block['arrived_drones']}/{block['drones']} ({block['arrival_rate']:.2f})"


def _runs_cell(count: int, runs: int) -> str:
    return f"{count}/{runs}"


def gate_text(card: SceneCard, rows: list[dict], policy: str) -> tuple[str, list[str]]:
    present, failures = evaluate_card_rows(card, [eval_row(row) for row in rows], policy=policy, label=card.name)
    if present is None:
        return "未冻结", []
    if not failures:
        return "否", []
    return f"是（{len(failures)}）", list(failures)


def scene_notes(cards: dict[str, SceneCard]) -> list[str]:
    lines = [
        "| 卡 | 任务 | 机数 | 时长 s | 分辨率 | fx px | 2 m 处 σ mm（s=0.08） | 2 m 处 σ mm（s=0.20） |",
        "|---|---|---:|---:|---|---:|---:|---:|",
    ]
    for meta in SCENES:
        if meta["id"] not in cards:
            continue
        card = cards[meta["id"]]
        raw = card.card
        camera = raw["camera"]
        fx = focal_px(raw)
        lines.append(
            "| {label} | {task} | {n} | {dur:.0f} | {w}×{h} | {fx:.1f} | {s08:.1f} | {s20:.1f} |".format(
                label=meta["label"],
                task=raw["task"]["type"],
                n=len(raw["drones"]),
                dur=float(raw["task"]["duration_s"]),
                w=camera["width"],
                h=camera["height"],
                fx=fx,
                s08=sigma_at_2m(fx, 0.08) * 1000.0,
                s20=sigma_at_2m(fx, 0.20) * 1000.0,
            )
        )
    return lines


def _step_sentence(before: float, after: float, name: str) -> str:
    return f"{name}：{before:.3f} → {after:.3f}（变化 {after - before:+.3f}）"


def _extreme(steps: tuple) -> str:
    """steps 是 (名字, 前, 后)。下降最多的一段；若都没下降，就说变化最大的一段。"""
    drop = max(steps, key=lambda item: item[1] - item[2])
    if drop[1] - drop[2] > 1e-9:
        return (
            f"下降最多的是「{drop[0]}」，{drop[1]:.3f} → {drop[2]:.3f}"
            f"（变化 {drop[2] - drop[1]:+.3f}）"
        )
    rise = max(steps, key=lambda item: item[2] - item[1])
    return (
        f"没有一段在下降。变化最大的是「{rise[0]}」，{rise[1]:.3f} → {rise[2]:.3f}"
        f"（变化 {rise[2] - rise[1]:+.3f}）"
    )


def conclusion(groups: dict) -> list[str]:
    """只根据这张表的到达率写。编队不读 YOPO，单独说。"""
    lines = []
    yopo = tuple(scene for scene in YOPO_SCENES if scene in groups and "off" in groups[scene])
    if len(yopo) < len(YOPO_SCENES):
        lines.append("这不是四张 YOPO 卡的完整网格，下面只根据已经落盘的卡说话。")
    off = pooled_rate(groups, yopo, "off") if yopo else None
    def _rate(arm: str):
        have = tuple(scene for scene in yopo if arm in groups[scene])
        if len(have) != len(yopo):
            return None
        return pooled_rate(groups, have, arm)

    noise = _rate("noise")
    holes = _rate("holes")
    full = _rate("all")
    stress = _rate("stress")
    lines.append(
        "下面两套到达率都只含四张走 YOPO 的卡：g2 柱子、密度 0.14 单机、"
        "crossing-pillars、六机林带 sparse。开阔三机编队不读深度，不进合计。"
        "「按飞机」把到达飞机数除以飞机×种子，六机卡权重更大。"
        "「按卡」是四张卡到达率的简单平均，每张卡一票。"
    )
    lines.append(
        "按飞机：基线 {off}，只开噪声 {noise}，噪声+空洞/阴影 {holes}，全开 {full}，压测 {stress}。".format(
            off=_fmt(off), noise=_fmt(noise), holes=_fmt(holes), full=_fmt(full), stress=_fmt(stress),
        )
    )
    if yopo and all(arm in groups[scene] for scene in yopo for arm in ("off", "noise", "holes", "all", "stress")):
        lines.append(
            "按卡：基线 {off}，只开噪声 {noise}，噪声+空洞/阴影 {holes}，全开 {full}，压测 {stress}。".format(
                off=_fmt(scene_mean_rate(groups, yopo, "off")),
                noise=_fmt(scene_mean_rate(groups, yopo, "noise")),
                holes=_fmt(scene_mean_rate(groups, yopo, "holes")),
                full=_fmt(scene_mean_rate(groups, yopo, "all")),
                stress=_fmt(scene_mean_rate(groups, yopo, "stress")),
            )
        )
    if None not in (off, noise, holes, full):
        steps = (
            ("只加视差噪声", off, noise),
            ("再加空洞/阴影", noise, holes),
            ("再加约 21 ms 延迟", holes, full),
        )
        lines.append("按飞机分段：" + "；".join(_step_sentence(a, b, name) for name, a, b in steps) + "。")
        lines.append("三段里按飞机到达率" + _extreme(steps) + "。")
        card_steps = (
            ("只加视差噪声", scene_mean_rate(groups, yopo, "off"), scene_mean_rate(groups, yopo, "noise")),
            ("再加空洞/阴影", scene_mean_rate(groups, yopo, "noise"), scene_mean_rate(groups, yopo, "holes")),
            ("再加约 21 ms 延迟", scene_mean_rate(groups, yopo, "holes"), scene_mean_rate(groups, yopo, "all")),
        )
        if all(item[1] is not None and item[2] is not None for item in card_steps):
            lines.append(
                "按卡分段：" + "；".join(_step_sentence(a, b, name) for name, a, b in card_steps) + "。"
                "按卡到达率" + _extreme(card_steps) + "。"
            )
        lines.append(
            f"全开相对基线，按飞机到达率 {off:.3f} → {full:.3f}（变化 {full - off:+.3f}）。"
            "这是上游权重在这些仿真相机和这套 D435 模型下的掉幅，不是 848×480 实机的直接读数。"
            "2 m 处的米制噪声见上表：160×96、90° 的卡大约 80 mm，g2 的 320×240、73° 大约 30 mm；"
            "原生 D435 在 fx≈426 时是 15 mm。"
        )
    if stress is not None and full is not None:
        lines.append(
            f"压测（s=0.20，延迟固定 50 ms，丢帧率仍是 0.5%）相对全开："
            f"{full:.3f} → {stress:.3f}（变化 {stress - full:+.3f}）。"
        )
        if "d014-solo" in groups and "six-sparse" in groups and "all" in groups["d014-solo"] and "stress" in groups["six-sparse"]:
            lines.append(
                "压测不是默认全开再加一档噪声：s、延迟和每回合视差偏置都换了。"
                f"密度 0.14 全开 {_rate_cell(groups['d014-solo']['all'])}，压测 {_rate_cell(groups['d014-solo']['stress'])}。"
                f"六机林带 sparse 全开 {_rate_cell(groups['six-sparse']['all'])}，压测 {_rate_cell(groups['six-sparse']['stress'])}。"
                "合计回升不能读成噪声越大越安全。"
            )
    if "formation-hold" in groups and all(arm in groups["formation-hold"] for arm in ARMS):
        form_bits = []
        for arm in ARMS:
            block = groups["formation-hold"][arm]
            form_bits.append(f"{ARM_LABEL[arm]} {_rate_cell(block)}")
        lines.append("开阔三机编队（formation-hold，不读深度）：" + "；".join(form_bits) + "。")
        off_shadow = groups["formation-hold"]["off"]["shadow_reject_count"]
        full_shadow = groups["formation-hold"]["all"]["shadow_reject_count"]
        if off_shadow == full_shadow:
            lines.append(
                f"这张编队卡基线和全开的影子盾会拒 tick 都是 {off_shadow}，"
                "到达、净空、机间、平均到达时间也相同。编队规划不读深度。"
                "开阔场景上，这次加噪没有改到影子盾记下的拒绝次数。"
            )
        else:
            lines.append(
                f"这张编队卡的影子盾会拒 tick：基线 {off_shadow}，全开 {full_shadow}。"
                "计划不读深度，所以到达不该变；次数不同只说明影子在用加噪后的深度图复验。"
            )
    return lines


def render_markdown(cards: dict[str, SceneCard], groups: dict, failures: dict, rows: list[dict]) -> str:
    header = (
        "| 卡 | 臂 | 偏离上游 | 到达 | 障碍碰撞 | 机间碰撞 | 最小净空 m | 中位净空 m | 最小机间 m | 越门 | 平均到达时间 s | 影子盾介入 |"
    )
    rule = "|---|---|---|---:|---:|---:|---:|---:|---:|---|---:|---:|"
    body = []
    for meta in SCENES:
        if meta["id"] not in groups:
            continue
        for arm in ARMS:
            if arm not in groups[meta["id"]]:
                continue
            block = groups[meta["id"]][arm]
            body.append(
                "| {label} | {arm} | {dev} | {arr} | {obs} | {inter} | {clr} | {med} | {gap} | {gate} | {time} | {shadow} |".format(
                    label=meta["label"],
                    arm=ARM_LABEL[arm],
                    dev="否" if arm == "off" else "偏离上游",
                    arr=_rate_cell(block),
                    obs=_runs_cell(block["obstacle_runs"], block["runs"]),
                    inter=_runs_cell(block["inter_runs"], block["runs"]),
                    clr=_fmt(block["min_clearance_m"]),
                    med=_fmt(block["median_clearance_m"]),
                    gap=_fmt(block["min_inter_m"]),
                    gate=block["gate"],
                    time=_fmt(block["mean_arrival_time_s"]),
                    shadow=block["shadow_reject_count"],
                )
            )
    failure_lines = []
    for key in sorted(failures):
        for item in failures[key]:
            failure_lines.append(f"- `{key}`：{item}")
    if not failure_lines:
        failure_lines.append("- 没有已冻门被这批评判为失败。未冻结的卡写在表里，不算越门。")
    notes = "\n".join(scene_notes(cards))
    closing = "\n\n".join(conclusion(groups))
    seeds = sorted({int(row["seed"]) for row in rows})
    return "\n".join([
        "# 票 E-D435：D435 深度噪声与延迟下的 YOPO L0",
        "",
        DISCLAIMER,
        "",
        "## 接在哪里",
        "",
        "深度只在 `yopo_l0/sensor.py` 的 `render` 进策略之前改一次。",
        "`d435 is None` 时仍是原来的高斯噪声三行，随机数抽取次数不变。",
        "运行器读 `task.d435_depth`。缺省、`false`、或 `enabled: false` 都不建噪声状态，结果里也不加字段。",
        "名字登记在 `yopo_l0/experiment.py` 的 `RESEARCH_SWITCHES`，收口脚本据此拒绝场景卡把该键写成 JSON true。",
        "本票没有改 `yopo_l0/crossing.py` 和 `yopo_l0/sitl_bridge.py`。交叉任务读的仍是相机交出去的那一帧。",
        "",
        "## 模型和开关",
        "",
        "- 轴向噪声在视差域：`σ_Z = Z² · s / (fx · B)`，`B = 0.05` m，默认 `s = 0.08`。`fx` 用场景卡相机，不把图放大到 848×480 再缩回来。",
        "- 每架飞机每个回合抽一次视差偏置 `N(0, 0.1)` px，截断到 ±0.25 px。噪声、空洞、延迟用三条独立随机流。",
        "- 空洞/阴影：最小距离 0.195 m、左侧无效带、前景遮挡阴影、4–8 m 掉点、10 m 截断、掠射角、1% 随机洞、块状洞、0.3% 假近点。",
        "- 延迟：捕获时刻入队，送达后再给策略。默认 `max(10, N(18, 3)) + 0.5×8` ms，0.5% 尖峰加 20–60 ms，0.5% 丢帧。帧龄是锁步里的采样保持，不再另抽 `U(0, 1/fps)`。",
        "- 压测臂把输送延迟固定成 50 ms（抖动和尖峰为 0，丢帧率仍是 0.5%），`s = 0.20`。",
        "- 无效像素：`nan`。`network_depth` 对 NaN 和小于 0.04 m 的像素做上游 NS radius-1 inpaint。安全证据和影子盾仍把 NaN 当未知。`inpaint` 和 `max_range` 两个填充留在 `yopo_l0/depth_noise.py`，这次没打开。",
        "",
        "置信度低、仍然用了调研表默认值的参数：飞点概率 `p_flying = 0.3`，掠射角阈值 75°，视差噪声相关长度 3 px，丢帧率 0.5%，Orin 额外延迟约 5 ms（含在大约 21 ms 的合计里）。",
        "",
        "原生 D435 在 848×480、`fx ≈ 426` 时，2 m 处 σ ≈ 15 mm。下表是这些场景卡自己的焦距，噪声按这个焦距算。",
        "",
        notes,
        "",
        "## 结果",
        "",
        f"seeds {seeds[0]}–{seeds[-1]}，每格 {len(seeds)} 个种子。到达是到达飞机 / 飞机数。"
        "障碍碰撞、机间碰撞是发生了碰撞的运行次数 / 种子数（仿真在第一次碰撞停下，一次运行最多记 1）。"
        "净空 = 机体中心到障碍表面的距离 − 机体半径。最小净空是种子里最差的一次，中位净空是这些种子的中位数。"
        "平均到达时间只平均已经整队到达的种子。影子盾介入是 `would_reject` 的 tick 合计。",
        "",
        markdown_table(header, rule, body),
        "",
        "### 已冻门",
        "",
        *failure_lines,
        "",
        "## 结论",
        "",
        closing,
        "",
        "逐粒记录在 [e-d435-depth-noise-runs.json](e-d435-depth-noise-runs.json)。",
        "",
        "## 复现",
        "",
        "```",
        "python3 scripts/e-d435-depth-noise.py --jobs 4",
        "```",
        "",
        "中断后原样再跑会读 `out/e-d435-depth-noise/partial/`。`--fresh` 忽略已落盘的粒。",
        "",
    ])


def group_rows(rows: list[dict], cards: dict[str, SceneCard]) -> tuple[dict, dict]:
    groups = {}
    failures = {}
    for meta in SCENES:
        if meta["id"] not in cards:
            continue
        groups[meta["id"]] = {}
        card = cards[meta["id"]]
        for arm in ARMS:
            chosen = [row for row in rows if row["scene_id"] == meta["id"] and row["arm"] == arm]
            if not chosen:
                continue
            block = aggregate(chosen)
            text, failed = gate_text(card, chosen, meta["policy"])
            block["gate"] = text
            groups[meta["id"]][arm] = block
            if failed:
                failures[f"{meta['id']}/{arm}"] = failed
    return groups, failures


def expected_specs(scenes: list[str], arms: list[str], seeds: list[int], duration_s: float | None) -> list[tuple]:
    return [(scene, arm, seed, duration_s) for scene in scenes for arm in arms for seed in seeds]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="E-D435 深度噪声 L0 回归，只报数")
    parser.add_argument("--scenes", default=",".join(item["id"] for item in SCENES))
    parser.add_argument("--arms", default=",".join(ARMS))
    parser.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--duration", type=float, default=None, help="短于场景卡时只做冒烟，不写正式报告")
    parser.add_argument("--fresh", action="store_true")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--out", default="docs/research/e-d435-depth-noise-runs.json")
    parser.add_argument("--md", default="docs/research/e-d435-depth-noise.md")
    args = parser.parse_args(argv)
    scenes = comma_list(args.scenes)
    arms = comma_list(args.arms)
    seeds = comma_ints(args.seeds)
    unknown = unknown_values(scenes, SCENE_BY_ID) + unknown_values(arms, ARMS)
    if unknown:
        raise SystemExit("未知场景或臂：" + ",".join(map(str, unknown)))
    if args.jobs < 1:
        raise SystemExit("--jobs 至少为 1")
    cards = {meta["id"]: load_card(ROOT / meta["path"]) for meta in SCENES if meta["id"] in scenes}
    if args.check:
        for meta in SCENES:
            if meta["id"] not in scenes:
                continue
            card = cards[meta["id"]]
            fx = focal_px(card.card)
            print(f"{meta['id']} policy={meta['policy']} fx={fx:.2f} sigma2m={sigma_at_2m(fx)*1000:.1f}mm")
            for arm in arms:
                tuned = with_arm(card, arm)
                if arm == "off":
                    assert "d435_depth" not in tuned.card["task"]
                else:
                    assert tuned.card["task"]["d435_depth"]["enabled"] is True
        print("check ok")
        return 0
    specs = expected_specs(scenes, arms, seeds, args.duration)
    if args.fresh:
        for scene, arm, seed, _duration in specs:
            path = partial_path(scene, arm, seed)
            if path.is_file():
                path.unlink()
    pending = [spec for spec in specs if args.fresh or read_partial(partial_path(*spec[:3]), spec[3]) is None]
    print(f"grains {len(specs)} pending {len(pending)} jobs {args.jobs}", flush=True)
    if pending:
        run_spawn_jobs(pending, run_one, args.jobs)
    rows = []
    missing = []
    for spec in specs:
        row = read_partial(partial_path(*spec[:3]), spec[3])
        if row is None:
            missing.append(spec)
        else:
            rows.append(row)
    if missing:
        raise SystemExit(f"还有 {len(missing)} 粒没有落盘，不写报告")
    full_grid = (
        args.duration is None
        and scenes == [item["id"] for item in SCENES]
        and arms == list(ARMS)
        and seeds == list(SEEDS)
    )
    groups, failures = group_rows(rows, cards)
    payload = {
        "ticket": "E-D435",
        "row_version": ROW_VERSION,
        "disclaimer": DISCLAIMER,
        "invalid_fill": "nan",
        "arms": {arm: ARM_SPEC[arm] for arm in ARMS},
        "scenes": [meta["id"] for meta in SCENES],
        "seeds": list(SEEDS),
        "complete_grid": full_grid,
        "groups": groups,
        "gate_failures": failures,
        "rows": rows,
    }
    out = ROOT / args.out
    md = ROOT / args.md
    if not full_grid:
        print("不是完整网格，不覆盖 docs/research 里的正式报告", flush=True)
        out = PARTIAL / "partial-summary.json"
        md = PARTIAL / "partial-summary.md"
    write_json(out, payload)
    md.parent.mkdir(parents=True, exist_ok=True)
    md.write_text(render_markdown(cards, groups, failures, rows), encoding="utf-8")
    print(f"wrote {md} and {out}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
