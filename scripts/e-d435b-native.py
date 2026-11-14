#!/usr/bin/env python3
"""票 E-D435b：原生 D435 焦距下的深度噪声。只报数。

基于票 E-D435。``d435_depth`` 默认关，``profile`` 缺省 ``scene``，和 E-D435 同一条路径。
``profile=native`` 在原生角分辨率上加视差噪声和空洞，再重采样回场景相机分辨率。

四张走 YOPO 的卡，seeds 0–9。native 四档：只噪声、噪声+空洞/阴影、全开、压测。
另有对照：空洞用真值填，对 scene 和 native 各一档，用来分开「空洞像素里的值」和网络侧 NS inpaint。
不改 ``scenes/``、已冻门、``core/``。打开即偏离上游。

``--jobs`` 用 spawn 并行。逐粒写到 ``out/e-d435b-native/partial/``，重跑会接着做。
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
from yopo_l0.depth_noise import DEF, d435_depth, incidence_cos, native_d435_depth, native_sensor
from yopo_l0.experiment import comma_ints, comma_list, markdown_table, run_spawn_jobs, unknown_values, write_json
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import SceneCard, load_card

PARTIAL = ROOT / "out" / "e-d435b-native" / "partial"
REFERENCE = ROOT / "docs" / "research" / "e-d435-depth-noise-runs.json"
ROW_VERSION = 1
SEEDS = tuple(range(10))
BODY_FALLBACK_M = 0.35
REF_ARMS = ("off", "noise", "holes", "all", "stress")
TIER_ARMS = ("noise", "holes", "all", "stress")

SCENES = (
    {"id": "g2-pillar", "path": "scenes/g2-pillar.json", "label": "g2 柱子单机", "policy": "upstream"},
    {"id": "d014-solo", "path": "scenes/ego-density-014-solo.json", "label": "密度 0.14 单机", "policy": "upstream"},
    {"id": "crossing-pillars", "path": "scenes/crossing-pillars.json", "label": "crossing-pillars 三机", "policy": "crossing"},
    {"id": "six-sparse", "path": "scenes/crossing-6-forest-sparse.json", "label": "六机林带 sparse", "policy": "crossing"},
)
SCENE_BY_ID = {item["id"]: item for item in SCENES}
YOPO_SCENES = tuple(item["id"] for item in SCENES)

ARMS = ("noise", "holes", "all", "stress", "holes_gt", "scene_holes", "scene_holes_gt")
ARM_SPEC = {
    "noise": {"enabled": True, "noise": True, "holes": False, "latency": False, "profile": "native"},
    "holes": {"enabled": True, "noise": True, "holes": True, "latency": False, "profile": "native"},
    "all": {"enabled": True, "noise": True, "holes": True, "latency": True, "profile": "native"},
    "stress": {
        "enabled": True,
        "noise": True,
        "holes": True,
        "latency": True,
        "subpix": 0.20,
        "latency_ms": 50.0,
        "profile": "native",
    },
    "holes_gt": {
        "enabled": True,
        "noise": True,
        "holes": True,
        "latency": False,
        "profile": "native",
        "invalid": "gt",
    },
    "scene_holes": {"enabled": True, "noise": True, "holes": True, "latency": False, "profile": "scene"},
    "scene_holes_gt": {
        "enabled": True,
        "noise": True,
        "holes": True,
        "latency": False,
        "profile": "scene",
        "invalid": "gt",
    },
}
ARM_LABEL = {
    "off": "基线（关）",
    "noise": "只开噪声",
    "holes": "噪声 + 空洞/阴影",
    "all": "全开（含延迟）",
    "stress": "压测 s=0.20、延迟 50 ms",
    "holes_gt": "噪声 + 空洞，空洞填真值",
    "scene_holes": "scene 噪声 + 空洞/阴影（NS inpaint）",
    "scene_holes_gt": "scene 噪声 + 空洞，空洞填真值",
}
REF_LABEL = {
    "off": "E-D435 基线（关）",
    "noise": "E-D435 只开噪声",
    "holes": "E-D435 噪声 + 空洞/阴影",
    "all": "E-D435 全开（含延迟）",
    "stress": "E-D435 压测",
}

DISCLAIMER = (
    "只报数。report-only。``d435_depth`` 默认关，``profile`` 缺省 ``scene``。"
    "场景卡、``scenes/suite.txt``、已冻门都没改。打开即偏离上游："
    "上游权重在真值深度上训练（右目关闭）。"
    "``profile=native`` 按原生 D435（848×480，fx≈426 px，B=0.05 m，s=0.08）在视差域加噪，"
    "再重采样到场景相机分辨率。无效像素默认仍是 NaN，网络侧走上游 NS inpaint。"
    "真值填是对照：空洞像素写成这一帧的真值，网络不再把这些像素当洞。"
    "假近点两种填法都留着。飞点概率 0.3、掠射角阈值 75°、相关长度 3 px、丢帧率 0.5%，"
    "以及约 21 ms 里 Orin 额外的约 5 ms，置信度低。"
    "影子盾只记录 would_reject，不改批准的计划。墙钟不进结论。"
)


def focal_px(width: float, fov_deg: float) -> float:
    return (float(width) / 2.0) / math.tan(math.radians(float(fov_deg)) / 2.0)


def camera_k(camera: dict) -> tuple[int, int, float, float, tuple[float, float, float, float]]:
    width = int(camera["width"])
    height = int(camera["height"])
    fx = focal_px(width, camera["hfov_deg"])
    fy = focal_px(height, camera["vfov_deg"])
    return width, height, fx, fy, (fx, fy, width / 2.0, height / 2.0)


def sigma_formula(fx: float, subpix: float = 0.08) -> float:
    return (2.0 ** 2) * subpix / (fx * float(DEF["baseline"]))


def _quiet(**overrides) -> dict:
    params = dict(
        DEF,
        p_flying=0.0,
        lateral_px=0.0,
        p_hole=0.0,
        blob_holes=0,
        p_outlier=0.0,
        far_drop_z0=50.0,
        far_drop_z1=60.0,
        disp_offset=0.0,
    )
    params.update(overrides)
    return params


def _center_std(depth: np.ndarray, margin: int = 8) -> float:
    center = depth[margin:-margin, margin:-margin]
    center = center[np.isfinite(center) & (center > 0)]
    if center.size < 10:
        return float("nan")
    return float(center.std())


def measure_sigma(intrinsics, height: int, width: int, profile: str, subpix: float) -> float:
    image = np.full((height, width), 2.0)
    params = _quiet(subpix=subpix)
    rng = np.random.default_rng(0)
    if profile == "native":
        out = native_d435_depth(image, intrinsics, rng, params=params, noise=True, holes=False)
    else:
        out = d435_depth(image, intrinsics, rng, params=params, noise=True, holes=False)
    return _center_std(out)


def measure_holes(intrinsics, height: int, width: int, profile: str, geometric_only: bool) -> float:
    ground = np.full((height, width), 6.0)
    ground[:, width // 2 : width // 2 + max(4, width // 8)] = 1.5
    params = _quiet() if geometric_only else dict(DEF)
    rng = np.random.default_rng(0)
    holes_rng = np.random.default_rng(1)
    if profile == "native":
        out = native_d435_depth(
            ground, intrinsics, rng, holes_rng=holes_rng, params=params, noise=False, holes=True,
        )
    else:
        out = d435_depth(
            ground,
            intrinsics,
            rng,
            holes_rng=holes_rng,
            params=params,
            noise=False,
            holes=True,
            normals_cos=incidence_cos(ground, *intrinsics),
        )
    return float((~np.isfinite(out) | (out <= 0)).mean())


def geometry_rows(cards: dict[str, SceneCard]) -> list[dict]:
    seen = []
    rows = []
    for meta in SCENES:
        if meta["id"] not in cards:
            continue
        card = cards[meta["id"]]
        width, height, fx, fy, intrinsics = camera_k(card.card["camera"])
        key = (width, height, round(fx, 3), round(fy, 3))
        if key in seen:
            continue
        seen.append(key)
        rows.append({
            "label": meta["label"],
            "resolution": f"{width}×{height}",
            "fx": fx,
            "fy": fy,
            "sigma_scene_mm": measure_sigma(intrinsics, height, width, "scene", 0.08) * 1000.0,
            "sigma_native_mm": measure_sigma(intrinsics, height, width, "native", 0.08) * 1000.0,
            "sigma_scene_stress_mm": measure_sigma(intrinsics, height, width, "scene", 0.20) * 1000.0,
            "sigma_native_stress_mm": measure_sigma(intrinsics, height, width, "native", 0.20) * 1000.0,
            "sigma_formula_scene_mm": sigma_formula(fx, 0.08) * 1000.0,
            "sigma_formula_native_mm": sigma_formula(426.0, 0.08) * 1000.0,
            "hole_scene": measure_holes(intrinsics, height, width, "scene", False),
            "hole_scene_geom": measure_holes(intrinsics, height, width, "scene", True),
            "hole_native": measure_holes(intrinsics, height, width, "native", False),
            "hole_native_geom": measure_holes(intrinsics, height, width, "native", True),
        })
    return rows


def with_arm(card: SceneCard, arm: str) -> SceneCard:
    raw = dict(card.card)
    task = dict(raw["task"])
    task["d435_depth"] = dict(ARM_SPEC[arm])
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
        "shadow_reject_count": int(shadow),
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
        "deviates": True,
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
    dists = [
        float(row["min_obstacle_distance_m"])
        for row in rows
        if row.get("min_obstacle_distance_m") is not None
    ]
    gaps = [
        float(row["min_inter_drone_distance_m"])
        for row in rows
        if row.get("min_inter_drone_distance_m") is not None
    ]
    times = [float(row["arrival_time_s"]) for row in rows if row.get("arrival_time_s") is not None]
    return {
        "runs": len(rows),
        "arrived_drones": arrived,
        "drones": drones,
        "arrival_rate": None if not drones else arrived / drones,
        "obstacle_runs": sum(int(row["obstacle_collision"]) for row in rows),
        "inter_runs": sum(int(row["inter_collision"]) for row in rows),
        "min_obstacle_m": None if not dists else min(dists),
        "min_clearance_m": None if not clearances else min(clearances),
        "median_clearance_m": None if not clearances else float(np.median(clearances)),
        "min_inter_m": None if not gaps else min(gaps),
        "mean_arrival_time_s": None if not times else float(np.mean(times)),
        "arrived_runs": len(times),
        "shadow_reject_count": sum(int(row.get("shadow_reject_count") or 0) for row in rows),
    }


def pooled_rate(groups: dict, scenes: tuple[str, ...], arm: str) -> float | None:
    arrived = drones = 0
    for scene in scenes:
        block = groups.get(scene, {}).get(arm)
        if block is None:
            return None
        arrived += int(block["arrived_drones"])
        drones += int(block["drones"])
    if not drones:
        return None
    return arrived / drones


def scene_mean_rate(groups: dict, scenes: tuple[str, ...], arm: str) -> float | None:
    rates = []
    for scene in scenes:
        block = groups.get(scene, {}).get(arm)
        if block is None or block["arrival_rate"] is None:
            return None
        rates.append(float(block["arrival_rate"]))
    return float(np.mean(rates))


def _fmt(value, digits=3) -> str:
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "—"
    return f"{float(value):.{digits}f}"


def _rate_cell(block: dict | None) -> str:
    if not block or not block["drones"]:
        return "—"
    return f"{block['arrived_drones']}/{block['drones']} ({block['arrival_rate']:.2f})"


def _runs_cell(block: dict | None, key: str) -> str:
    if not block:
        return "—"
    return f"{block[key]}/{block['runs']}"


def gate_text(card: SceneCard, rows: list[dict], policy: str) -> tuple[str, list[str]]:
    present, failures = evaluate_card_rows(card, [eval_row(row) for row in rows], policy=policy, label=card.name)
    if present is None:
        return "未冻结", []
    if not failures:
        return "否", []
    return f"是（{len(failures)}）", list(failures)


def group_rows(rows: list[dict], cards: dict[str, SceneCard], arms: tuple[str, ...]) -> tuple[dict, dict]:
    groups = {}
    failures = {}
    for meta in SCENES:
        if meta["id"] not in cards:
            continue
        groups[meta["id"]] = {}
        card = cards[meta["id"]]
        for arm in arms:
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


def load_reference_rows() -> list[dict]:
    payload = json.loads(REFERENCE.read_text(encoding="utf-8"))
    rows = []
    for row in payload["rows"]:
        if row.get("scene_id") not in SCENE_BY_ID:
            continue
        if row.get("arm") not in REF_ARMS:
            continue
        rows.append(row)
    return rows


def _step(before: float, after: float, name: str) -> str:
    return f"{name}：{before:.3f} → {after:.3f}（变化 {after - before:+.3f}）"


def _fill_sentence(ns: float | None, gt: float | None, noise: float | None, where: str) -> str:
    if ns is None or gt is None:
        return f"{where}的真值填对照还没齐。"
    gap = gt - ns
    if noise is None:
        return f"{where}：NS inpaint 到达率 {ns:.3f}，空洞填真值 {gt:.3f}（变化 {gap:+.3f}）。"
    if gap > 0.05 and gt + 0.05 >= noise:
        return (
            f"{where}：NS inpaint {ns:.3f}，空洞填真值 {gt:.3f}（变化 {gap:+.3f}），"
            f"只开噪声 {noise:.3f}。真值填回到只开噪声附近，掉点来自网络侧 NS 填上的深度，"
            f"空洞像素换成真值之后这组种子不再掉这么多。"
        )
    if gap > 0.05:
        return (
            f"{where}：NS inpaint {ns:.3f}，空洞填真值 {gt:.3f}（变化 {gap:+.3f}），"
            f"只开噪声 {noise:.3f}。真值填比 NS 高，离只开噪声还有距离。"
            f"NS 填补是一部分；两种填法都留下的假近点也还在图里。"
        )
    if ns + 0.05 < noise:
        return (
            f"{where}：NS inpaint {ns:.3f}，空洞填真值 {gt:.3f}（变化 {gap:+.3f}），"
            f"只开噪声 {noise:.3f}。两种填法的到达率接近，而且都低于只开噪声。"
            f"差别不在 NS 和真值这两种填法之间。假近点两种都保留。"
        )
    return (
        f"{where}：NS inpaint {ns:.3f}，空洞填真值 {gt:.3f}（变化 {gap:+.3f}），"
        f"只开噪声 {noise:.3f}。这一档空洞没有把到达率打下去。"
    )


def conclusion(groups: dict, ref: dict, geom: list[dict]) -> list[str]:
    lines = [
        "下面的合计只含四张走 YOPO 的卡：g2 柱子、密度 0.14 单机、crossing-pillars、六机林带 sparse。"
        "「按飞机」把到达飞机数除以飞机×种子，六机卡权重大。"
        "「按卡」是四张卡到达率的简单平均。开阔三机编队不读深度，E-D435 里五条臂相同，本票不重跑。",
    ]
    sensor = native_sensor()
    lines.append(
        f"原生传感器名义画幅 {sensor['width']}×{sensor['height']}，fx={sensor['fx']:.0f}，"
        f"基线 {sensor['baseline_m']:.3f} m，s={sensor['subpix']:.2f}。"
        "2 m 处公式 σ 是 15.0 mm。场景相机视场用同一像素角铺开，最近邻抬到这个网格，"
        "视差加噪之后按视差平均缩回场景分辨率；一个格子里有效原生像素不足一半，这个格子仍是空洞。"
    )
    if geom:
        bits = []
        for row in geom:
            bits.append(
                f"{row['label']}（{row['resolution']}，fx={row['fx']:.1f}）"
                f"scene 实测 {_fmt(row['sigma_scene_mm'], 1)} mm，"
                f"native 重采样后 {_fmt(row['sigma_native_mm'], 1)} mm"
            )
        lines.append("平壁 2 m、关掉飞点和空洞，种子 0：" + "；".join(bits) + "。")
        bits = []
        for row in geom:
            bits.append(
                f"{row['label']} scene 全空洞 {row['hole_scene']:.3f}、只留几何 {row['hole_scene_geom']:.3f}，"
                f"native 全空洞 {row['hole_native']:.3f}、只留几何 {row['hole_native_geom']:.3f}"
            )
        lines.append(
            "同一张合成图（半幅 1.5 m 柱、其余 6 m 墙），无效像素比例："
            + "；".join(bits)
            + "。只留几何是关掉随机洞、块状洞和假近点。遮挡阴影和左侧无效带的角宽由基线决定，"
            "缩回场景分辨率之后和 scene 同量级。块状洞半径按像素计，scene 在 160 宽上是大洞，"
            "native 先在 fx≈426 的网格上挖，缩回来只占很少的格子。"
        )
    off = pooled_rate(ref, YOPO_SCENES, "off")
    ref_noise = pooled_rate(ref, YOPO_SCENES, "noise")
    ref_holes = pooled_rate(ref, YOPO_SCENES, "holes")
    ref_all = pooled_rate(ref, YOPO_SCENES, "all")
    ref_stress = pooled_rate(ref, YOPO_SCENES, "stress")
    nat_noise = pooled_rate(groups, YOPO_SCENES, "noise")
    nat_holes = pooled_rate(groups, YOPO_SCENES, "holes")
    nat_all = pooled_rate(groups, YOPO_SCENES, "all")
    nat_stress = pooled_rate(groups, YOPO_SCENES, "stress")
    if None not in (off, ref_noise, ref_holes, ref_all, ref_stress):
        lines.append(
            "E-D435 原表按飞机：基线 {off}，只开噪声 {noise}，噪声+空洞/阴影 {holes}，全开 {full}，压测 {stress}。".format(
                off=_fmt(off), noise=_fmt(ref_noise), holes=_fmt(ref_holes), full=_fmt(ref_all), stress=_fmt(ref_stress),
            )
        )
        lines.append(
            "E-D435 原表按卡：基线 {off}，只开噪声 {noise}，噪声+空洞/阴影 {holes}，全开 {full}，压测 {stress}。".format(
                off=_fmt(scene_mean_rate(ref, YOPO_SCENES, "off")),
                noise=_fmt(scene_mean_rate(ref, YOPO_SCENES, "noise")),
                holes=_fmt(scene_mean_rate(ref, YOPO_SCENES, "holes")),
                full=_fmt(scene_mean_rate(ref, YOPO_SCENES, "all")),
                stress=_fmt(scene_mean_rate(ref, YOPO_SCENES, "stress")),
            )
        )
    if None not in (nat_noise, nat_holes, nat_all, nat_stress):
        lines.append(
            "native 按飞机：只开噪声 {noise}，噪声+空洞/阴影 {holes}，全开 {full}，压测 {stress}。".format(
                noise=_fmt(nat_noise), holes=_fmt(nat_holes), full=_fmt(nat_all), stress=_fmt(nat_stress),
            )
        )
        lines.append(
            "native 按卡：只开噪声 {noise}，噪声+空洞/阴影 {holes}，全开 {full}，压测 {stress}。".format(
                noise=_fmt(scene_mean_rate(groups, YOPO_SCENES, "noise")),
                holes=_fmt(scene_mean_rate(groups, YOPO_SCENES, "holes")),
                full=_fmt(scene_mean_rate(groups, YOPO_SCENES, "all")),
                stress=_fmt(scene_mean_rate(groups, YOPO_SCENES, "stress")),
            )
        )
    if None not in (ref_holes, nat_holes, ref_noise, nat_noise):
        lines.append(
            "按飞机，只开噪声：E-D435 {a:.3f}，native {b:.3f}（变化 {c:+.3f}）。"
            "噪声+空洞/阴影：E-D435 {d:.3f}，native {e:.3f}（变化 {f:+.3f}）。".format(
                a=ref_noise, b=nat_noise, c=nat_noise - ref_noise,
                d=ref_holes, e=nat_holes, f=nat_holes - ref_holes,
            )
        )
    if None not in (nat_noise, nat_holes, nat_all):
        lines.append(
            "native 分段："
            + "；".join((
                _step(nat_noise, nat_holes, "在只开噪声上再加空洞/阴影"),
                _step(nat_holes, nat_all, "再加约 21 ms 延迟"),
            ))
            + "。"
        )
    if off is not None and nat_all is not None:
        lines.append(
            f"native 全开相对 E-D435 基线，按飞机 {off:.3f} → {nat_all:.3f}（变化 {nat_all - off:+.3f}）。"
            "这仍是上游权重在仿真里的读数，传感器模型换成了原生 D435 的像素角。"
        )
    if nat_stress is not None and nat_all is not None:
        lines.append(
            f"native 压测（s=0.20，延迟固定 50 ms，丢帧率仍是 0.5%）相对 native 全开："
            f"{nat_all:.3f} → {nat_stress:.3f}（变化 {nat_stress - nat_all:+.3f}）。"
            "压测换了 s、延迟和每回合视差偏置，不是默认全开再加一档。"
        )
    scene_ns = pooled_rate(groups, YOPO_SCENES, "scene_holes")
    scene_gt = pooled_rate(groups, YOPO_SCENES, "scene_holes_gt")
    native_gt = pooled_rate(groups, YOPO_SCENES, "holes_gt")
    lines.append(_fill_sentence(scene_ns, scene_gt, ref_noise, "scene 合计（E-D435 那套焦距，本分支重跑）"))
    lines.append(_fill_sentence(nat_holes, native_gt, nat_noise, "native 合计"))
    card_bits = []
    for meta in SCENES:
        ns_block = groups.get(meta["id"], {}).get("scene_holes")
        gt_block = groups.get(meta["id"], {}).get("scene_holes_gt")
        if ns_block is None or gt_block is None:
            continue
        card_bits.append(
            f"{meta['label']} NS {_rate_cell(ns_block)}、障碍碰撞 {_runs_cell(ns_block, 'obstacle_runs')}，"
            f"真值填 {_rate_cell(gt_block)}、障碍碰撞 {_runs_cell(gt_block, 'obstacle_runs')}"
        )
    if card_bits:
        lines.append("scene 按卡：" + "；".join(card_bits) + "。")
        d_ns = groups.get("d014-solo", {}).get("scene_holes")
        d_gt = groups.get("d014-solo", {}).get("scene_holes_gt")
        c_ns = groups.get("crossing-pillars", {}).get("scene_holes")
        c_gt = groups.get("crossing-pillars", {}).get("scene_holes_gt")
        s_ns = groups.get("six-sparse", {}).get("scene_holes")
        s_gt = groups.get("six-sparse", {}).get("scene_holes_gt")
        bits = []
        if d_ns and d_gt and d_ns["arrival_rate"] == 0 and d_gt["arrival_rate"] == 0:
            bits.append(
                f"密度 0.14 两种填法都是 {_rate_cell(d_gt)}，障碍碰撞都是 {_runs_cell(d_gt, 'obstacle_runs')}。"
                "这张卡的掉点不来自 NS 和真值这两种填法的差别。两种填法都留下假近点，本票没有再把假近点单独关掉。"
            )
        if c_ns and c_gt and c_gt["arrived_drones"] > c_ns["arrived_drones"] and c_gt["obstacle_runs"] < c_ns["obstacle_runs"]:
            bits.append(
                f"crossing-pillars 从 {_rate_cell(c_ns)}、障碍碰撞 {_runs_cell(c_ns, 'obstacle_runs')} "
                f"回到 {_rate_cell(c_gt)}、障碍碰撞 {_runs_cell(c_gt, 'obstacle_runs')}。"
            )
        if s_ns and s_gt and s_gt["arrived_drones"] > s_ns["arrived_drones"] and s_gt["obstacle_runs"] == 0:
            missed = int(s_gt["drones"]) - int(s_gt["arrived_drones"])
            bits.append(
                f"六机林带从 {_rate_cell(s_ns)}、障碍碰撞 {_runs_cell(s_ns, 'obstacle_runs')} "
                f"回到 {_rate_cell(s_gt)}、障碍碰撞 {_runs_cell(s_gt, 'obstacle_runs')}。"
                + (f"没到达的 {missed} 架没有记成障碍碰撞。" if missed else "")
            )
        if bits:
            lines.append("".join(bits) + "crossing 和六机上，E-D435 的空洞掉点主要是网络侧 NS 填进空洞的深度。")
    native_bits = []
    for meta in SCENES:
        ns_block = groups.get(meta["id"], {}).get("holes")
        gt_block = groups.get(meta["id"], {}).get("holes_gt")
        if ns_block is None or gt_block is None:
            continue
        native_bits.append(
            f"{meta['label']} NS {_rate_cell(ns_block)}，真值填 {_rate_cell(gt_block)}"
        )
    if native_bits:
        lines.append("native 按卡：" + "；".join(native_bits) + "。")
    if scene_ns is not None and ref_holes is not None:
        same = True
        for scene in YOPO_SCENES:
            left = ref.get(scene, {}).get("holes")
            right = groups.get(scene, {}).get("scene_holes")
            if left is None or right is None:
                same = False
                break
            if left["arrived_drones"] != right["arrived_drones"] or left["obstacle_runs"] != right["obstacle_runs"]:
                same = False
                break
            if abs(float(left["min_clearance_m"]) - float(right["min_clearance_m"])) > 1e-3:
                same = False
                break
        if same:
            lines.append(
                "本分支重跑的 scene 空洞臂，四张卡的到达飞机数、障碍碰撞次数和最小净空与 E-D435 原表一致"
                "（净空差小于 0.001 m）。scene 路径在合并 origin/main 之后没有改飞行结果。"
                "真值填对照和原表空洞臂是同一条噪声，只是空洞像素的填法不同。"
            )
        else:
            lines.append(
                "本分支重跑的 scene 空洞臂与 E-D435 原表有差。填法对照以本分支重跑的 scene 空洞臂为 NS 一侧，"
                "原表数字仍列在对照表里。"
            )
    per = []
    for meta in SCENES:
        ref_h = ref.get(meta["id"], {}).get("holes")
        nat_h = groups.get(meta["id"], {}).get("holes")
        if ref_h and nat_h:
            per.append(f"{meta['label']} {_rate_cell(ref_h)} → {_rate_cell(nat_h)}")
    if per:
        lines.append("空洞/阴影这一档，按卡从 E-D435 到 native：" + "；".join(per) + "。")
    return lines


def _result_row(label: str, arm_label: str, block: dict) -> str:
    return (
        "| {label} | {arm} | 偏离上游 | {arr} | {obs} | {inter} | {dist} | {clr} | {med} | {gate} |".format(
            label=label,
            arm=arm_label,
            arr=_rate_cell(block),
            obs=_runs_cell(block, "obstacle_runs"),
            inter=_runs_cell(block, "inter_runs"),
            dist=_fmt(block["min_obstacle_m"]),
            clr=_fmt(block["min_clearance_m"]),
            med=_fmt(block["median_clearance_m"]),
            gate=block["gate"],
        )
    )


def render_markdown(groups: dict, ref: dict, failures: dict, ref_failures: dict, geom: list[dict], seeds: list[int]) -> str:
    sensor = native_sensor()
    geo_header = (
        "| 相机 | 分辨率 | fx px | scene 公式 σ mm | scene 实测 σ mm | native 公式 σ mm | native 重采样后 σ mm | "
        "scene 空洞比例 | scene 只几何 | native 空洞比例 | native 只几何 |"
    )
    geo_rule = "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"
    geo_body = []
    for row in geom:
        geo_body.append(
            "| {label} | {res} | {fx:.1f} | {fs:.1f} | {ms:.1f} | {fn:.1f} | {mn:.1f} | {hs:.3f} | {hg:.3f} | {hn:.3f} | {hng:.3f} |".format(
                label=row["label"],
                res=row["resolution"],
                fx=row["fx"],
                fs=row["sigma_formula_scene_mm"],
                ms=row["sigma_scene_mm"],
                fn=row["sigma_formula_native_mm"],
                mn=row["sigma_native_mm"],
                hs=row["hole_scene"],
                hg=row["hole_scene_geom"],
                hn=row["hole_native"],
                hng=row["hole_native_geom"],
            )
        )
    header = "| 卡 | 臂 | 偏离上游 | 到达 | 障碍碰撞 | 机间碰撞 | 最小障碍距离 m | 最小净空 m | 中位净空 m | 越门 |"
    rule = "|---|---|---|---:|---:|---:|---:|---:|---:|---|"
    native_body = []
    for meta in SCENES:
        for arm in ("noise", "holes", "all", "stress", "holes_gt"):
            block = groups.get(meta["id"], {}).get(arm)
            if block is None:
                continue
            native_body.append(_result_row(meta["label"], ARM_LABEL[arm], block))
    side_header = (
        "| 卡 | 臂 | E-D435 到达 | native 到达 | E-D435 障碍碰撞 | native 障碍碰撞 | "
        "E-D435 最小障碍距离 m | native 最小障碍距离 m | E-D435 最小净空 m | native 最小净空 m |"
    )
    side_rule = "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"
    side_body = []
    for meta in SCENES:
        for arm in ("off",) + TIER_ARMS:
            left = ref.get(meta["id"], {}).get(arm)
            right = None if arm == "off" else groups.get(meta["id"], {}).get(arm)
            if left is None and right is None:
                continue
            side_body.append(
                "| {label} | {arm} | {la} | {ra} | {lo} | {ro} | {ld} | {rd} | {lc} | {rc} |".format(
                    label=meta["label"],
                    arm=REF_LABEL[arm] if arm == "off" else ARM_LABEL[arm],
                    la=_rate_cell(left),
                    ra="—" if arm == "off" else _rate_cell(right),
                    lo=_runs_cell(left, "obstacle_runs"),
                    ro="—" if arm == "off" else _runs_cell(right, "obstacle_runs"),
                    ld=_fmt(None if left is None else left["min_obstacle_m"]),
                    rd="—" if arm == "off" else _fmt(None if right is None else right["min_obstacle_m"]),
                    lc=_fmt(None if left is None else left["min_clearance_m"]),
                    rc="—" if arm == "off" else _fmt(None if right is None else right["min_clearance_m"]),
                )
            )
    fill_body = []
    for meta in SCENES:
        for arm in ("scene_holes", "scene_holes_gt", "holes", "holes_gt"):
            block = groups.get(meta["id"], {}).get(arm)
            if block is None:
                continue
            fill_body.append(_result_row(meta["label"], ARM_LABEL[arm], block))
    failure_lines = []
    for key in sorted(failures):
        for item in failures[key]:
            failure_lines.append(f"- `{key}`：{item}")
    if not failure_lines:
        failure_lines.append("- 本票重跑没有已冻门被判失败。未冻结的卡写在表里，不算越门。")
    closing = "\n\n".join(conclusion(groups, ref, geom))
    return "\n".join([
        "# 票 E-D435b：原生 D435 焦距下的深度噪声",
        "",
        DISCLAIMER,
        "",
        "依赖票 E-D435（PR #122，分支 `yopo-restart-bot/e-d435-depth-noise`）。本分支先合并了 `origin/main`。",
        "E-D435 的原表在 [e-d435-depth-noise.md](e-d435-depth-noise.md)。",
        "",
        "## 模型和开关",
        "",
        "- `task.d435_depth` 缺省关。关时不建噪声状态，高斯随机流与加这只开关之前相同。",
        "- `profile` 缺省 `scene`：视差噪声和按像素计的空洞用场景相机的 fx。不写 `profile` 时，深度与 E-D435 的 scene 路径逐元素相同。",
        f"- `profile=native`：名义传感器 {sensor['width']}×{sensor['height']}、fx={sensor['fx']:.0f}、B={sensor['baseline_m']:.3f} m。"
        "同一视场按这个像素角抬成细网格，调用原来的视差噪声和空洞，再把视差平均重采样回场景分辨率。",
        "- 重采样时，格子里有效原生像素不足一半，输出仍是空洞。块状洞、1% 随机洞、相关长度 3 px 都在细网格的像素上，缩回来变小。",
        "- 遮挡阴影宽度约 `fx · B · (1/z_near − 1/z_far)` 像素，角宽是 `B · (1/z_near − 1/z_far)`，跟把 fx 换成场景焦距再在粗网格上算，是同一个角。",
        "- 无效像素默认 `nan`。`network_depth` 对 NaN 和小于 0.04 m 的像素做上游 NS radius-1 inpaint。",
        "- `invalid=gt`：空洞像素改成这一帧的真值，再交给策略。这些像素是有限深度，NS 不会把它们当洞。假近点（约 0.3%）不在这次填补里。",
        "- 名字仍登记在 `yopo_l0/experiment.py` 的 `RESEARCH_SWITCHES`。场景卡没有把 `d435_depth` 写成 JSON true。",
        "",
        "## 平壁 σ 和合成图上的空洞比例",
        "",
        "σ 是 2 m 平壁、关掉飞点和空洞、视差偏置为 0、种子 0，裁掉四周 8 像素后的标准差。"
        "公式 σ 是 `Z² · s / (fx · B)`，native 公式用 fx=426，还没有做重采样平均。"
        "空洞比例是半幅 1.5 m、其余 6 m 的合成图，种子固定。三张 160×96、90° 的卡共用第二行的相机。",
        "",
        markdown_table(geo_header, geo_rule, geo_body),
        "",
        "## native 飞行结果",
        "",
        f"seeds {seeds[0]}–{seeds[-1]}。到达是到达飞机 / 飞机数。"
        "障碍碰撞、机间碰撞是发生了碰撞的运行次数 / 种子数（仿真在第一次碰撞停下，一次运行最多记 1）。"
        "最小障碍距离是种子里最小的机体中心到障碍表面的距离。最小净空是该距离减去机体半径后再取种子里最差的一次。"
        "这四张卡的机体半径是 0.35 m。中位净空是各种子最小净空的中位数。越门只报告，本票不改已冻门。",
        "",
        markdown_table(header, rule, native_body),
        "",
        "## 与 E-D435 原表对照",
        "",
        "E-D435 列来自 [e-d435-depth-noise-runs.json](e-d435-depth-noise-runs.json)，profile 是 scene。"
        "native 列是本票。基线没有 native 臂。最小障碍距离是各种子 `min_obstacle_distance_m` 的最小值。",
        "",
        markdown_table(side_header, side_rule, side_body),
        "",
        "## 空洞填真值，对照 NS inpaint",
        "",
        "scene 两行是 E-D435 的焦距，本分支重跑。native 两行是原生像素角。"
        "NS 是 `invalid=nan`：空洞留给 `network_depth` 做 NS inpaint。真值填是 `invalid=gt`。"
        "噪声、遮挡、随机洞、块状洞、假近点两边相同。真值填只替换深度 ≤ 0 的像素。",
        "",
        markdown_table(header, rule, fill_body),
        "",
        "### 已冻门",
        "",
        *failure_lines,
        "",
        "E-D435 原表里空洞、全开、压测的已冻门失败留在 [e-d435-depth-noise.md](e-d435-depth-noise.md)。"
        f"按同一套门重算，原表四张卡全部臂里有 {len(ref_failures)} 组越门。这里不改那些门。",
        "",
        "## 结论",
        "",
        closing,
        "",
        "逐粒记录在 [e-d435b-native-runs.json](e-d435b-native-runs.json)。",
        "",
        "## 复现",
        "",
        "```",
        "python3 scripts/e-d435b-native.py --jobs 4",
        "```",
        "",
        "中断后原样再跑会读 `out/e-d435b-native/partial/`。`--fresh` 忽略已落盘的粒。",
        "",
    ])


def expected_specs(scenes: list[str], arms: list[str], seeds: list[int], duration_s: float | None) -> list[tuple]:
    return [(scene, arm, seed, duration_s) for scene in scenes for arm in arms for seed in seeds]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="E-D435b 原生 D435 深度噪声，只报数")
    parser.add_argument("--scenes", default=",".join(item["id"] for item in SCENES))
    parser.add_argument("--arms", default=",".join(ARMS))
    parser.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--duration", type=float, default=None, help="短于场景卡时只做冒烟，不写正式报告")
    parser.add_argument("--fresh", action="store_true")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--out", default="docs/research/e-d435b-native-runs.json")
    parser.add_argument("--md", default="docs/research/e-d435b-native.md")
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
            _width, _height, fx, _fy, _k = camera_k(card.card["camera"])
            print(f"{meta['id']} policy={meta['policy']} fx={fx:.2f} formula={sigma_formula(fx)*1000:.1f}mm")
            for arm in arms:
                tuned = with_arm(card, arm)
                spec = tuned.card["task"]["d435_depth"]
                assert spec["enabled"] is True
                assert spec["profile"] == ARM_SPEC[arm]["profile"]
        assert d435_from_task_default_off(cards)
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
    groups, failures = group_rows(rows, cards, tuple(arms))
    ref_rows = load_reference_rows()
    ref_cards = {meta["id"]: load_card(ROOT / meta["path"]) for meta in SCENES}
    ref_groups, ref_failures = group_rows(ref_rows, ref_cards, REF_ARMS)
    geom = geometry_rows(cards)
    payload = {
        "ticket": "E-D435b",
        "row_version": ROW_VERSION,
        "disclaimer": DISCLAIMER,
        "depends_on": "E-D435 PR #122",
        "profile": "native",
        "invalid_fill": "nan",
        "arms": {arm: ARM_SPEC[arm] for arm in ARMS},
        "scenes": [meta["id"] for meta in SCENES if meta["id"] in scenes],
        "seeds": seeds,
        "complete_grid": full_grid,
        "native_sensor": native_sensor(),
        "geometry": geom,
        "groups": groups,
        "reference_groups": ref_groups,
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
    md.write_text(
        render_markdown(groups, ref_groups, failures, ref_failures, geom, seeds),
        encoding="utf-8",
    )
    print(f"wrote {md} and {out}", flush=True)
    return 0


def d435_from_task_default_off(cards: dict[str, SceneCard]) -> bool:
    for card in cards.values():
        if "d435_depth" in card.card.get("task", {}):
            return False
    return True


if __name__ == "__main__":
    raise SystemExit(main())
