#!/usr/bin/env python3
"""D5c：按 L0 机体半径 0.35 m + margin 重算未来 1.6 s 的表面净空，只写统计 JSON。

不改输入 npz，也不改仓库外已有的教师数据集。训练子命令调用
``train_from_paths``，``planar=True``，并带上净空过滤 / 贴边降权 / 末段加权。
这些开关在 ``FitConfig`` 里默认关；本脚本的 fit 子命令才默认打开 ``planar``。

    .venv/bin/python scripts/distill/d5c/filter_and_fit.py filter \\
        --manifest /workspace/yopo-teacher-data/d6/convert-manifest.json \\
        --trajectory-root /workspace/yopo-teacher-data/d6/runs \\
        --margin-m 0.02 \\
        --out /workspace/yopo-teacher-data/d5c/filter-stats.json

    .venv/bin/python scripts/distill/d5c/filter_and_fit.py fit \\
        --manifest /workspace/yopo-teacher-data/d5c/convert-manifest.json \\
        --out /workspace/yopo-teacher-data/d5c/student \\
        --min-clearance-m 0.0 --soft-margin-m 0.05 --low-weight 0.25 \\
        --endpoint-weight 1.0 --clearance-missing error

    .venv/bin/python scripts/distill/d5c/filter_and_fit.py --smoke

report-only。这里不跑闭环，也不把结果写进已冻门。
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

L0_BODY_RADIUS_M = 0.35
HORIZON_S = 1.6
SCHEMA = "yopo-d5c-clearance-filter/1"


def _json_float(value) -> float | None:
    if value is None:
        return None
    number = float(value)
    if not np.isfinite(number):
        return None
    return number


def _abs(path: str | Path, base: Path) -> Path:
    p = Path(path)
    if not p.is_absolute():
        p = base / p
    return p.resolve()


def surface_rows(points: np.ndarray, obstacles) -> np.ndarray:
    """每个点到障碍表面的距离，形状 (P,)。公式与 ``yopo_l0.scene`` 的 distance 相同。

    直立圆柱、盒子、地面一次算完。斜圆柱逐点走 ``Cylinder.distance``。
    """
    from yopo_l0.scene import Box, Cylinder, Ground

    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    best = np.full(pts.shape[0], np.inf, dtype=np.float64)
    if pts.shape[0] == 0 or not obstacles:
        return best
    upright = [o for o in obstacles if isinstance(o, Cylinder) and o.upright()]
    if upright:
        center = np.asarray([o.center_xy for o in upright], dtype=np.float64)
        radius = np.asarray([o.radius for o in upright], dtype=np.float64)
        z_lo = np.asarray([o.z_lo for o in upright], dtype=np.float64)
        z_hi = np.asarray([o.z_hi for o in upright], dtype=np.float64)
        radial = np.hypot(pts[:, None, 0] - center[None, :, 0], pts[:, None, 1] - center[None, :, 1]) - radius
        radial = np.maximum(radial, 0.0)
        dz = np.maximum(np.maximum(z_lo - pts[:, None, 2], 0.0), pts[:, None, 2] - z_hi)
        best = np.minimum(best, np.hypot(radial, dz).min(axis=1))
    for ob in obstacles:
        if isinstance(ob, Ground):
            best = np.minimum(best, np.maximum(pts[:, 2] - float(ob.z), 0.0))
        elif isinstance(ob, Box):
            lo = np.asarray(ob.lo, dtype=np.float64)
            hi = np.asarray(ob.hi, dtype=np.float64)
            delta = np.maximum(np.maximum(lo - pts, 0.0), pts - hi)
            best = np.minimum(best, np.linalg.norm(delta, axis=1))
        elif isinstance(ob, Cylinder) and not ob.upright():
            for i, p in enumerate(pts):
                best[i] = min(best[i], float(ob.distance(p)))
    return best


def min_surface_nk(points: np.ndarray, obstacles) -> np.ndarray:
    """``points`` 形状 (N, K, 3)。返回每个窗口在 K 个点上的最小表面距离。"""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 3 or pts.shape[-1] != 3:
        raise ValueError(f"未来位置应为 (N, K, 3)，收到 {pts.shape}")
    n, k, _ = pts.shape
    if n == 0 or k == 0:
        return np.full(n, np.inf, dtype=np.float64)
    return surface_rows(pts.reshape(n * k, 3), obstacles).reshape(n, k).min(axis=1)


def body_to_world(origin: np.ndarray, yaw: np.ndarray, body: np.ndarray) -> np.ndarray:
    """机体系未来位置变回世界系。``R_yaw`` 的同一矩阵：p_w = origin + R @ p_b。"""
    origin = np.asarray(origin, dtype=np.float64)
    yaw = np.asarray(yaw, dtype=np.float64).reshape(-1)
    body = np.asarray(body, dtype=np.float64)
    if body.ndim == 2:
        body = body[:, None, :]
    if body.ndim != 3 or body.shape[-1] != 3:
        raise ValueError(f"position_body 应为 (N, K, 3)，收到 {body.shape}")
    n = int(body.shape[0])
    if origin.shape != (n, 3) or yaw.shape != (n,):
        raise ValueError(f"origin {origin.shape}、yaw {yaw.shape} 与窗口数 {n} 对不齐")
    c = np.cos(yaw)[:, None]
    s = np.sin(yaw)[:, None]
    out = np.empty_like(body, dtype=np.float64)
    out[:, :, 0] = origin[:, None, 0] + c * body[:, :, 0] - s * body[:, :, 1]
    out[:, :, 1] = origin[:, None, 1] + s * body[:, :, 0] + c * body[:, :, 1]
    out[:, :, 2] = origin[:, None, 2] + body[:, :, 2]
    return out


def horizon_samples(time_s: np.ndarray, position: np.ndarray, t0: float, horizon_s: float) -> np.ndarray:
    """一架在 ``[t0, t0 + horizon]`` 里的采样点，含视界末端的插值（记录盖得到的话）。

    与 D5b 重标注过滤同一段时间。点与点之间的连续轨迹不采样，快穿会漏。
    """
    time_s = np.asarray(time_s, dtype=np.float64)
    position = np.asarray(position, dtype=np.float64)
    t0 = float(t0)
    t1 = t0 + float(horizon_s)
    if time_s.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    sel = (time_s >= t0 - 1e-6) & (time_s <= t1 + 1e-6)
    pts = position[sel]
    if time_s.size >= 2 and float(time_s[0]) - 1e-9 <= t1 <= float(time_s[-1]) + 1e-9:
        end = np.array([np.interp(t1, time_s, position[:, c]) for c in range(3)], dtype=np.float64)
        if pts.size == 0 or float(np.linalg.norm(pts[-1] - end)) > 1e-6:
            pts = np.vstack([pts, end]) if pts.size else end.reshape(1, 3)
    return np.asarray(pts, dtype=np.float64)


def _as_nk3(points: np.ndarray, n: int) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim == 2 and pts.shape == (n, 3):
        pts = pts[:, None, :]
    if pts.ndim != 3 or pts.shape[0] != n or pts.shape[-1] != 3:
        raise ValueError(f"未来世界位置应为 ({n}, K, 3)，收到 {pts.shape}")
    return pts


def _obstacles(scene_path: Path | None):
    if scene_path is None:
        return None
    from yopo_l0.scene import load_card

    if not Path(scene_path).is_file():
        raise FileNotFoundError(f"缺少场景卡：{scene_path}")
    return load_card(scene_path).obstacles


def score_file(
    path: Path,
    scene: Path | None,
    trajectory: Path | None,
    *,
    body_radius_m: float,
    margin_m: float,
    horizon_s: float,
) -> dict:
    """对一个窗口文件重算或读取净空。不写文件。

    优先级：轨迹采样 > npz 的 position_world > origin_m + yaw_rad + position_body > 已存的净空数组。
    重算结果是表面距离减去 ``body_radius_m``。已存数组当作净空直接比，不再减一次半径。
    """
    from yopo_l0.student_fit import read_window_clearance

    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"缺少窗口：{path}")
    with np.load(path, allow_pickle=False) as npz:
        files = set(npz.files)
        if "state" not in files:
            raise ValueError(f"{path} 没有 state，不像学生窗口")
        n = int(np.asarray(npz["state"]).shape[0])
        time_s = np.asarray(npz["time_s"], dtype=np.float64).copy() if "time_s" in files else None
        ego = np.asarray(npz["ego_index"], dtype=np.int64).reshape(-1).copy() if "ego_index" in files else None
        position_world = np.asarray(npz["position_world"], dtype=np.float64).copy() if "position_world" in files else None
        origin = np.asarray(npz["origin_m"], dtype=np.float64).copy() if "origin_m" in files else None
        yaw = np.asarray(npz["yaw_rad"], dtype=np.float64).copy() if "yaw_rad" in files else None
        body = np.asarray(npz["position_body"], dtype=np.float64).copy() if "position_body" in files else None

    obstacles = None
    source = "missing"
    clearance = None
    if scene is not None:
        obstacles = _obstacles(scene)

    if (
        obstacles is not None
        and trajectory is not None
        and Path(trajectory).is_file()
        and time_s is not None
        and ego is not None
    ):
        with np.load(trajectory, allow_pickle=False) as traj:
            if "time_s" not in traj.files or "position_m" not in traj.files:
                raise ValueError(f"{trajectory} 缺少 time_s 或 position_m")
            tt = np.asarray(traj["time_s"], dtype=np.float64)
            pos = np.asarray(traj["position_m"], dtype=np.float64)
        if time_s.shape[0] != n or ego.shape[0] != n:
            raise ValueError(f"{path} 的 time_s / ego_index 与窗口数 {n} 不一致")
        if pos.ndim != 3 or pos.shape[0] != tt.shape[0] or pos.shape[-1] != 3:
            raise ValueError(f"{trajectory} 的 position_m 形状是 {pos.shape}")
        per = np.empty(n, dtype=np.float64)
        for i in range(n):
            d = int(ego[i])
            if d < 0 or d >= pos.shape[1]:
                raise ValueError(f"{path} 第 {i} 个窗口的 ego_index={d} 超出轨迹机数 {pos.shape[1]}")
            samples = horizon_samples(tt, pos[:, d, :], float(time_s[i]), horizon_s)
            if samples.size == 0:
                per[i] = np.inf
            else:
                per[i] = float(surface_rows(samples, obstacles).min())
        clearance = per - float(body_radius_m)
        source = "trajectory"
    elif obstacles is not None and position_world is not None:
        clearance = min_surface_nk(_as_nk3(position_world, n), obstacles) - float(body_radius_m)
        source = "position_world"
    elif obstacles is not None and origin is not None and yaw is not None and body is not None:
        world = body_to_world(origin, yaw, body)
        clearance = min_surface_nk(world, obstacles) - float(body_radius_m)
        source = "origin_yaw"

    stored = read_window_clearance(path)
    stored_diff = None
    if clearance is None:
        if stored is None:
            source = "missing"
        else:
            if int(stored.shape[0]) != n:
                raise ValueError(f"{path} 的净空长度 {stored.shape[0]} 与窗口数 {n} 不一致")
            clearance = stored
            source = "stored"
    elif stored is not None and int(stored.shape[0]) == n:
        both = np.isfinite(clearance) & np.isfinite(stored)
        if bool(both.any()):
            stored_diff = float(np.max(np.abs(clearance[both] - stored[both])))

    if clearance is None:
        kept = dropped = 0
        unscored = n
        finite = np.zeros(0, dtype=np.float64)
    else:
        keep = ~np.isnan(clearance) & (clearance >= float(margin_m))
        kept = int(np.count_nonzero(keep))
        dropped = int(n - kept)
        unscored = 0
        finite = clearance[np.isfinite(clearance)]
    return {
        "id": path.stem,
        "path": str(path),
        "scene": None if scene is None else str(scene),
        "trajectory": None if trajectory is None else str(trajectory),
        "source": source,
        "n": n,
        "n_kept": kept,
        "n_dropped": dropped,
        "n_unscored": unscored,
        "min_clearance_m": _json_float(np.min(finite)) if finite.size else None,
        "stored_max_abs_diff_m": stored_diff,
        "clearance": clearance,
    }


def _histogram(values: np.ndarray) -> dict:
    if values.size == 0:
        return {}
    edges = (
        ("<0", values < 0.0),
        ("[0,0.02)", (values >= 0.0) & (values < 0.02)),
        ("[0.02,0.05)", (values >= 0.02) & (values < 0.05)),
        ("[0.05,0.15)", (values >= 0.05) & (values < 0.15)),
        (">=0.15", values >= 0.15),
    )
    return {name: int(np.count_nonzero(mask)) for name, mask in edges}


def filter_entries(
    entries: list[dict],
    *,
    body_radius_m: float = L0_BODY_RADIUS_M,
    margin_m: float = 0.0,
    horizon_s: float = HORIZON_S,
) -> dict:
    """重算或读取净空，汇总成统计。不修改任何输入文件。"""
    if float(body_radius_m) <= 0.0 or not np.isfinite(body_radius_m):
        raise ValueError(f"body_radius_m 必须为正，收到 {body_radius_m}")
    if float(margin_m) < 0.0 or not np.isfinite(margin_m):
        raise ValueError(f"margin_m 不能为负，收到 {margin_m}")
    if float(horizon_s) <= 0.0 or not np.isfinite(horizon_s):
        raise ValueError(f"horizon_s 必须为正，收到 {horizon_s}")
    rows = []
    chunks = []
    for ent in entries:
        scored = score_file(
            Path(ent["path"]),
            None if ent.get("scene") is None else Path(ent["scene"]),
            None if ent.get("trajectory") is None else Path(ent["trajectory"]),
            body_radius_m=float(body_radius_m),
            margin_m=float(margin_m),
            horizon_s=float(horizon_s),
        )
        if ent.get("id"):
            scored["id"] = str(ent["id"])
        clearance = scored.pop("clearance")
        if clearance is not None:
            finite = clearance[np.isfinite(clearance)]
            if finite.size:
                chunks.append(finite)
        rows.append(scored)
    all_values = np.concatenate(chunks) if chunks else np.zeros(0, dtype=np.float64)
    mins = [r["min_clearance_m"] for r in rows if r["min_clearance_m"] is not None]
    return {
        "schema": SCHEMA,
        "report_only": True,
        "inputs_modified": False,
        "body_radius_m": float(body_radius_m),
        "margin_m": float(margin_m),
        "horizon_s": float(horizon_s),
        "threshold_surface_m": float(body_radius_m) + float(margin_m),
        "rule": (
            "净空 = 未来视界内到障碍表面的最小距离 − body_radius_m；"
            "保留净空 >= margin_m，也就是表面距离 >= body_radius_m + margin_m。"
            "已存的 clearance / min_surface_clearance_m 视为净空，不再减半径。"
        ),
        "note": "只写这份统计 JSON，不改输入 npz，也不改仓库外已有数据集。",
        "n_files": len(rows),
        "n_windows": int(sum(r["n"] for r in rows)),
        "n_kept": int(sum(r["n_kept"] for r in rows)),
        "n_dropped": int(sum(r["n_dropped"] for r in rows)),
        "n_unscored": int(sum(r["n_unscored"] for r in rows)),
        "min_clearance_m": None if not mins else float(min(mins)),
        "histogram": _histogram(all_values),
        "files": rows,
    }


def collect_entries(
    windows: list[Path] | None,
    *,
    scene: Path | None = None,
    manifest: Path | None = None,
    trajectory_root: Path | None = None,
) -> list[dict]:
    entries: list[dict] = []
    if manifest is not None:
        manifest = Path(manifest)
        base = manifest.resolve().parent
        payload = json.loads(manifest.read_text(encoding="utf-8"))
        files = payload.get("files", payload if isinstance(payload, list) else None)
        if isinstance(files, dict):
            items = list(files.items())
        elif isinstance(files, list):
            items = [(None, item) for item in files]
        else:
            raise ValueError(f"{manifest} 没有 files 列表或字典")
        for key, rec in items:
            entries.append(_resolve_entry(key, rec, base, scene, trajectory_root))
    for window in windows or []:
        entries.append(_resolve_entry(None, {"path": str(window)}, Path.cwd(), scene, trajectory_root))
    if not entries:
        raise ValueError("没有窗口。传 --windows 或 --manifest")
    return entries


def _resolve_entry(key, rec, base: Path, scene: Path | None, trajectory_root: Path | None) -> dict:
    if isinstance(rec, str):
        rec = {"path": rec}
    if not isinstance(rec, dict) or not rec.get("path"):
        raise ValueError(f"manifest 条目缺少 path：{key or rec!r}")
    path = _abs(rec["path"], base)
    scene_path = _abs(rec["scene"], base) if rec.get("scene") else None
    traj = _abs(rec["trajectory"], base) if rec.get("trajectory") else None
    run_id = str(key or rec.get("id") or path.stem)
    if trajectory_root is not None:
        root = Path(trajectory_root)
        if scene_path is None and (root / run_id / "scene.json").is_file():
            scene_path = (root / run_id / "scene.json").resolve()
        if traj is None and (root / run_id / "trajectory.npz").is_file():
            traj = (root / run_id / "trajectory.npz").resolve()
    if scene_path is None and scene is not None:
        scene_path = Path(scene).resolve()
    return {"id": run_id, "path": path, "scene": scene_path, "trajectory": traj}


def split_manifest(manifest: Path) -> tuple[list[Path], list[Path]]:
    manifest = Path(manifest)
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    if "train_ids" not in payload or "hold_ids" not in payload or "files" not in payload:
        raise ValueError(f"{manifest} 需要 files、train_ids、hold_ids")
    base = manifest.resolve().parent
    files = payload["files"]

    def one(run_id: str) -> Path:
        rec = files[run_id]
        raw = rec["path"] if isinstance(rec, dict) else rec
        return _abs(raw, base)

    return [one(i) for i in payload["train_ids"]], [one(i) for i in payload["hold_ids"]]


def fit_paths(train: list[Path], hold: list[Path], out: Path, cfg) -> dict:
    from yopo_l0.student_fit import train_from_paths

    return train_from_paths([Path(p) for p in train], [Path(p) for p in hold], Path(out), cfg=cfg)


def _write_smoke_window(path: Path, clearance: np.ndarray, position_world: np.ndarray) -> None:
    n = int(clearance.shape[0])
    k = int(position_world.shape[1])
    meta = {
        "tensor_schema": "yopo-student-window/2",
        "scenario_id": "d5c-smoke",
        "horizon_dt_s": 0.2,
        "max_range_m": 20.0,
        "n_neighbors": 3,
        "skipped": None,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        depth=np.zeros((n, 2, 24, 40), np.float32),
        state=np.zeros((n, 9), np.float32),
        position_body=np.zeros((n, k, 3), np.float32),
        velocity_body=np.zeros((n, k, 3), np.float32),
        yaw_rel=np.zeros((n, k), np.float32),
        neighbor_rel=np.zeros((n, 3, 6), np.float32),
        neighbor_mask=np.zeros((n, 3), np.float32),
        time_s=np.zeros(n, np.float64),
        sample_index=np.arange(n, dtype=np.int64),
        ego_index=np.zeros(n, np.int64),
        meta=np.asarray(json.dumps(meta, ensure_ascii=False)),
        position_world=np.asarray(position_world, dtype=np.float64),
        min_surface_clearance_m=np.asarray(clearance, dtype=np.float64),
    )


def _sha256(path: Path) -> str:
    import hashlib

    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def run_smoke() -> dict:
    """合成一棵树和四个窗口，过滤并训练一个 epoch。临时目录用完即删。"""
    from yopo_l0.scene import load_card
    from yopo_l0.student_fit import FitConfig

    with tempfile.TemporaryDirectory(prefix="d5c-smoke-") as tmp:
        root = Path(tmp)
        geom = {
            "boxes": [],
            "cylinders": [
                {"name": "tree", "center_xy": [1.0, 0.0], "radius": 0.2, "z_lo": 0.0, "z_hi": 3.0},
            ],
        }
        (root / "geom.json").write_text(json.dumps(geom), encoding="utf-8")
        scene = {
            "name": "d5c-smoke",
            "geometry": "geom.json",
            "ground_z": 0.0,
            "drones": [{"id": "uav0", "start_enu": [0.0, 0.0, 1.5], "goal_enu": [5.0, 0.0, 1.5]}],
        }
        scene_path = root / "scene.json"
        scene_path.write_text(json.dumps(scene), encoding="utf-8")
        card = load_card(scene_path)
        # 圆柱在 x=1、半径 0.2。x=0.50 表面 0.30、净空 -0.05；x=0.45 表面 0.35、净空 0；远处只剩地面 1.15。
        specs = (
            ("brush", np.array([0.50, 0.0, 1.5])),
            ("edge", np.array([0.45, 0.0, 1.5])),
            ("far", np.array([4.0, 0.0, 1.5])),
        )
        expected = []
        worlds = []
        for _name, point in specs:
            surface = min(float(ob.distance(point)) for ob in card.obstacles)
            expected.append(surface - L0_BODY_RADIUS_M)
            worlds.append(np.repeat(point.reshape(1, 1, 3), 8, axis=1))
        expected_arr = np.asarray(expected, dtype=np.float64)
        train_path = root / "train.npz"
        hold_path = root / "hold.npz"
        _write_smoke_window(train_path, expected_arr, np.concatenate(worlds, axis=0))
        _write_smoke_window(hold_path, expected_arr[2:3], worlds[2])
        before = {p: _sha256(p) for p in (train_path, hold_path, scene_path)}
        report = filter_entries(
            [
                {"id": "train", "path": train_path, "scene": scene_path, "trajectory": None},
                {"id": "hold", "path": hold_path, "scene": scene_path, "trajectory": None},
            ],
            body_radius_m=L0_BODY_RADIUS_M,
            margin_m=0.0,
            horizon_s=HORIZON_S,
        )
        after = {p: _sha256(p) for p in before}
        if after != before:
            raise RuntimeError("过滤改动了输入文件")
        scored = score_file(
            train_path,
            scene_path,
            None,
            body_radius_m=L0_BODY_RADIUS_M,
            margin_m=0.0,
            horizon_s=HORIZON_S,
        )
        got = scored["clearance"]
        if got is None or got.shape != expected_arr.shape:
            raise RuntimeError(f"重算净空形状不对：{None if got is None else got.shape}")
        err = float(np.max(np.abs(got - expected_arr)))
        if err > 1e-9:
            raise RuntimeError(f"重算净空与场景几何不一致，最大差 {err}")
        if report["n_windows"] != 4 or report["n_dropped"] != 1 or report["n_kept"] != 3:
            raise RuntimeError(
                f"过滤计数不对：windows={report['n_windows']} kept={report['n_kept']} dropped={report['n_dropped']}"
            )
        if report["inputs_modified"]:
            raise RuntimeError("统计里 inputs_modified 应为 false")
        cfg = FitConfig(
            epochs=1,
            batch_size=4,
            num_threads=1,
            seed=0,
            planar=True,
            min_clearance_m=0.0,
            clearance_soft_margin_m=0.05,
            clearance_low_weight=0.25,
            endpoint_weight=1.0,
            shuffle_batches=1,
        )
        summary = fit_paths([train_path], [hold_path], root / "student", cfg)
        seen = int(summary["history"][-1]["n_train_windows_seen"]) if summary.get("history") else 0
        if seen != 2:
            raise RuntimeError(f"训练应丢掉贴树窗口，看到的窗口数是 {seen}")
        hold_n = int((summary.get("holdout") or {}).get("n_windows") or 0)
        if hold_n != 1:
            raise RuntimeError(f"留出应只剩安全窗口，n_windows={hold_n}")
        return {
            "report_only": True,
            "smoke": True,
            "inputs_modified": False,
            "max_recompute_error_m": err,
            "filter": {
                "n_windows": report["n_windows"],
                "n_kept": report["n_kept"],
                "n_dropped": report["n_dropped"],
                "n_unscored": report["n_unscored"],
                "body_radius_m": report["body_radius_m"],
                "margin_m": report["margin_m"],
                "horizon_s": report["horizon_s"],
                "min_clearance_m": report["min_clearance_m"],
                "inputs_modified": False,
            },
            "fit": {
                "planar": True,
                "min_clearance_m": 0.0,
                "clearance_soft_margin_m": 0.05,
                "clearance_low_weight": 0.25,
                "endpoint_weight": 1.0,
                "epochs_done": summary.get("epochs_done"),
                "n_train_windows_seen": seen,
                "holdout_n": hold_n,
                "weights_sha256": summary.get("weights_sha256"),
            },
        }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="D5c：L0 净空过滤统计，以及 planar 学生训练（report-only）")
    parser.add_argument("--smoke", action="store_true", help="用合成小样本跑通过滤和 1 个 epoch")
    sub = parser.add_subparsers(dest="cmd")

    filt = sub.add_parser("filter", help="重算未来 1.6 s 净空，只写统计 JSON")
    filt.add_argument("--windows", type=Path, nargs="*", default=[])
    filt.add_argument("--manifest", type=Path, default=None)
    filt.add_argument("--scene", type=Path, default=None, help="所有窗口共用的场景卡；清单里写了 scene 时以清单为准")
    filt.add_argument("--trajectory-root", type=Path, default=None, help="按场次编号找 <root>/<id>/trajectory.npz 和 scene.json")
    filt.add_argument("--body-radius-m", type=float, default=L0_BODY_RADIUS_M)
    filt.add_argument("--margin-m", type=float, default=0.0)
    filt.add_argument("--horizon-s", type=float, default=HORIZON_S)
    filt.add_argument("--out", type=Path, required=True)

    fit = sub.add_parser("fit", help="planar=True，并按窗口里的净空数组过滤 / 加权后训练")
    fit.add_argument("--manifest", type=Path, default=None, help="含 files、train_ids、hold_ids 的转换清单")
    fit.add_argument("--train", type=Path, nargs="*", default=[])
    fit.add_argument("--hold", type=Path, nargs="*", default=[])
    fit.add_argument("--out", type=Path, required=True)
    fit.add_argument("--epochs", type=int, default=5)
    fit.add_argument("--batch-size", type=int, default=64)
    fit.add_argument("--lr", type=float, default=1e-3)
    fit.add_argument("--seed", type=int, default=0)
    fit.add_argument("--threads", type=int, default=2)
    fit.add_argument("--min-clearance-m", type=float, default=None)
    fit.add_argument("--soft-margin-m", type=float, default=0.0)
    fit.add_argument("--low-weight", type=float, default=1.0)
    fit.add_argument("--endpoint-weight", type=float, default=0.0)
    fit.add_argument("--clearance-missing", choices=("keep", "skip", "error"), default="keep")
    fit.add_argument("--near-weight", type=float, default=1.0)
    fit.add_argument("--near-depth-m", type=float, default=1.2)
    fit.add_argument("--shuffle-batches", type=int, default=8)
    fit.add_argument("--no-planar", action="store_true", help="关闭只学水平。FitConfig 默认仍是关，这里默认开")
    return parser


def _compact_fit(summary: dict) -> dict:
    return {
        "report_only": True,
        "weights": summary.get("weights"),
        "weights_sha256": summary.get("weights_sha256"),
        "epochs_done": summary.get("epochs_done"),
        "holdout": summary.get("holdout"),
        "fit": summary.get("fit"),
        "note": "权重写在 --out。不改输入 npz。闭环数字不在本命令里。",
    }


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--smoke" in argv:
        extra = [arg for arg in argv if arg != "--smoke"]
        if extra:
            print("`--smoke` 单独使用，不和其他参数一起", file=sys.stderr)
            return 2
        try:
            report = run_smoke()
        except Exception as exc:
            print(f"smoke 失败：{exc}", file=sys.stderr)
            return 1
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0

    parser = _parser()
    args = parser.parse_args(argv)
    if args.cmd is None:
        parser.print_help(sys.stderr)
        return 2
    try:
        if args.cmd == "filter":
            entries = collect_entries(
                list(args.windows or []),
                scene=args.scene,
                manifest=args.manifest,
                trajectory_root=args.trajectory_root,
            )
            report = filter_entries(
                entries,
                body_radius_m=float(args.body_radius_m),
                margin_m=float(args.margin_m),
                horizon_s=float(args.horizon_s),
            )
            out = Path(args.out)
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(
                json.dumps(
                    {
                        "out": str(out),
                        "n_windows": report["n_windows"],
                        "n_kept": report["n_kept"],
                        "n_dropped": report["n_dropped"],
                        "n_unscored": report["n_unscored"],
                        "inputs_modified": False,
                        "report_only": True,
                    },
                    ensure_ascii=False,
                )
            )
            return 0
        if args.manifest is not None and (args.train or args.hold):
            print("fit 的 --manifest 与 --train/--hold 只能选一种", file=sys.stderr)
            return 2
        if args.manifest is not None:
            train, hold = split_manifest(args.manifest)
        else:
            train, hold = list(args.train or []), list(args.hold or [])
        if not train or not hold:
            print("fit 需要训练窗口和留出窗口", file=sys.stderr)
            return 2
        from yopo_l0.student_fit import FitConfig

        cfg = FitConfig(
            lr=float(args.lr),
            batch_size=int(args.batch_size),
            epochs=int(args.epochs),
            seed=int(args.seed),
            num_threads=int(args.threads),
            planar=not bool(args.no_planar),
            near_weight=float(args.near_weight),
            near_depth_m=float(args.near_depth_m),
            shuffle_batches=int(args.shuffle_batches),
            min_clearance_m=None if args.min_clearance_m is None else float(args.min_clearance_m),
            clearance_soft_margin_m=float(args.soft_margin_m),
            clearance_low_weight=float(args.low_weight),
            endpoint_weight=float(args.endpoint_weight),
            clearance_missing=str(args.clearance_missing),
        )
        summary = fit_paths(train, hold, args.out, cfg)
    except Exception as exc:
        print(f"{args.cmd} 失败：{exc}", file=sys.stderr)
        return 1
    print(json.dumps(_compact_fit(summary), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
