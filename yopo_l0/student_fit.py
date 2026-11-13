"""训练学生网络：按种子划留出，写检查点，可续跑。飞行环不导入本模块。"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np

from yopo_l0.distill import load_batch
from yopo_l0.student import (
    MultiHeadStudent,
    StudentConfig,
    load_checkpoint,
    save_checkpoint,
    trajectory_loss,
)


@dataclass(frozen=True)
class FitConfig:
    lr: float = 1e-3
    batch_size: int = 64
    epochs: int = 5
    seed: int = 0
    num_threads: int = 2
    loss_weights: tuple[float, float, float] = (1.0, 0.25, 0.05)
    log_every: int = 50
    # D5b 训练改法，默认全关（关时与 D5 逐位相同）。
    planar: bool = False  # 位置、速度损失只算水平 x/y，不学 z
    near_weight: float = 1.0  # 深度图里最近有效深度 < near_depth_m 的窗口，损失乘这个倍数
    near_depth_m: float = 1.2
    shuffle_batches: int = 8  # 攒多少个 batch 再洗；D5 是 8
    # D5c：L0 机体半径净空。默认全关。不传时损失仍走 trajectory_loss，与 D5/D5b 逐位相同。
    # min_clearance_m：窗口自带的 clearance / min_surface_clearance_m 不低于这个值才进训练和评估。
    # 软带：clearance < min_clearance_m + clearance_soft_margin_m 的贴边窗口乘 clearance_low_weight。
    # endpoint_weight：视界后一半（含末点）的位置误差乘 1+该值，减轻飞离目标。
    min_clearance_m: float | None = None
    clearance_soft_margin_m: float = 0.0
    clearance_low_weight: float = 1.0
    endpoint_weight: float = 0.0
    # 窗口没有净空数组时：keep 保持原样，skip 整场丢掉，error 报错。默认 keep，等于不动。
    clearance_missing: str = "keep"

    def __post_init__(self) -> None:
        if self.clearance_missing not in ("keep", "skip", "error"):
            raise ValueError(
                f"clearance_missing 必须是 keep、skip 或 error，收到 {self.clearance_missing!r}"
            )
        if not np.isfinite(self.clearance_soft_margin_m) or float(self.clearance_soft_margin_m) < 0.0:
            raise ValueError(
                f"clearance_soft_margin_m 必须是非负有限数，收到 {self.clearance_soft_margin_m!r}"
            )
        if not np.isfinite(self.clearance_low_weight) or float(self.clearance_low_weight) < 0.0:
            raise ValueError(
                f"clearance_low_weight 必须是非负有限数，收到 {self.clearance_low_weight!r}"
            )
        if not np.isfinite(self.endpoint_weight) or float(self.endpoint_weight) < 0.0:
            raise ValueError(f"endpoint_weight 必须是非负有限数，收到 {self.endpoint_weight!r}")
        if self.min_clearance_m is not None and not np.isfinite(self.min_clearance_m):
            raise ValueError(f"min_clearance_m 必须是有限数或 None，收到 {self.min_clearance_m!r}")


_TORCH_THREADS_SET = False
_CLEARANCE_KEYS = ("min_surface_clearance_m", "clearance")


def _soft_weight_active(cfg: FitConfig) -> bool:
    """贴边降权打开：软带宽度为正，且低净空权重落在 [0, 1)。"""
    return float(cfg.clearance_soft_margin_m) > 0.0 and 0.0 <= float(cfg.clearance_low_weight) < 1.0


def uses_unweighted_d5_loss(cfg: FitConfig, path_weights: dict | None = None) -> bool:
    """D5b/D5c 的损失改动都关着时为真。训练步这时调用 trajectory_loss，不走逐窗口加权。

    只设 min_clearance_m 时仍为真：过滤改的是样本集合，留下来的窗口损失公式不变。
    """
    if cfg.planar or cfg.near_weight != 1.0 or float(cfg.endpoint_weight) > 0.0 or _soft_weight_active(cfg):
        return False
    return all(float(v) == 1.0 for v in (path_weights or {}).values())


def read_window_clearance(path: Path) -> np.ndarray | None:
    """窗口 npz 里的逐窗口净空。优先 min_surface_clearance_m，否则 clearance。没有就返回 None。

    多出来的轴收成每个窗口一个数，取视界内最紧的那一点。
    """
    path = Path(path)
    with np.load(path, allow_pickle=False) as npz:
        key = next((name for name in _CLEARANCE_KEYS if name in npz.files), None)
        if key is None:
            return None
        arr = np.asarray(npz[key], dtype=np.float64)
    if arr.ndim == 0:
        arr = arr.reshape(1)
    elif arr.ndim > 1:
        arr = np.min(arr, axis=tuple(range(1, arr.ndim)))
    return np.reshape(arr, (-1,))


def _prepare_clearance(path: Path, n: int, cfg: FitConfig) -> tuple[np.ndarray | None, np.ndarray | None]:
    """返回 (keep 掩码或 None, 与原窗口对齐的净空或 None)。

    keep 为 None 表示这一场一个窗口都不丢。两个新开关都关时直接返回，不读文件。
    """
    if n == 0 or (cfg.min_clearance_m is None and not _soft_weight_active(cfg)):
        return None, None
    raw = read_window_clearance(path)
    if raw is None:
        if cfg.clearance_missing == "keep":
            return None, None
        if cfg.clearance_missing == "skip":
            return np.zeros(n, dtype=bool), None
        raise ValueError(
            f"{path} 没有 clearance 或 min_surface_clearance_m，FitConfig.clearance_missing='error'"
        )
    if int(raw.shape[0]) != int(n):
        raise ValueError(f"{path} 的净空长度 {raw.shape[0]} 与窗口数 {n} 不一致")
    if cfg.min_clearance_m is None:
        return None, raw
    keep = np.isfinite(raw) & (raw >= float(cfg.min_clearance_m))
    return keep, raw


def _take_windows(batch, keep: np.ndarray | None):
    if keep is None:
        return batch
    keep = np.asarray(keep, dtype=bool)
    if keep.shape != (batch.n,):
        raise ValueError(f"净空掩码长度 {keep.shape} 与窗口数 {batch.n} 不一致")
    if batch.n == 0 or bool(keep.all()):
        return batch
    return replace(
        batch,
        depth=batch.depth[keep],
        state=batch.state[keep],
        position_body=batch.position_body[keep],
        velocity_body=batch.velocity_body[keep],
        yaw_rel=batch.yaw_rel[keep],
        neighbor_rel=batch.neighbor_rel[keep],
        neighbor_mask=batch.neighbor_mask[keep],
        time_s=batch.time_s[keep],
        sample_index=batch.sample_index[keep],
        ego_index=batch.ego_index[keep],
    )


def _torch(num_threads: int | None = None):
    import torch

    global _TORCH_THREADS_SET
    if not _TORCH_THREADS_SET and num_threads is not None:
        torch.set_num_threads(max(1, int(num_threads)))
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            pass
        _TORCH_THREADS_SET = True
    return torch


def iter_run_windows(paths: list[Path]):
    for path in paths:
        batch = load_batch(path)
        if batch.n == 0:
            continue
        yield path, batch


def _stack_or_none(rows: list[np.ndarray]) -> np.ndarray | None:
    if not rows:
        return None
    return np.concatenate(rows, axis=0)


def evaluate_paths(
    model: MultiHeadStudent,
    paths: list[Path],
    *,
    batch_size: int = 256,
    cfg: FitConfig | None = None,
) -> dict:
    """留出或训练窗口上的均方误差。不把全部窗口一次装进内存。"""
    torch = _torch()
    net = model._module()
    net.eval()
    sums = {"n": 0, "position_sse": 0.0, "velocity_sse": 0.0, "yaw_sse": 0.0, "loss_sum": 0.0, "position_xy_sse": 0.0}
    weights = (1.0, 0.25, 0.05)
    for path, batch in iter_run_windows(paths):
        if cfg is not None and cfg.min_clearance_m is not None:
            keep, _clr = _prepare_clearance(path, batch.n, cfg)
            if keep is not None:
                batch = _take_windows(batch, keep)
        if batch.n == 0:
            continue
        n = batch.n
        for start in range(0, n, batch_size):
            sl = slice(start, min(n, start + batch_size))
            with torch.no_grad():
                ph, vh, yh = net(
                    torch.from_numpy(batch.depth[sl]),
                    torch.from_numpy(batch.state[sl]),
                    torch.from_numpy(batch.neighbor_rel[sl]),
                    torch.from_numpy(batch.neighbor_mask[sl]),
                )
                total, pos, vel, yaw_l = trajectory_loss(
                    torch.from_numpy(batch.position_body[sl]),
                    torch.from_numpy(batch.velocity_body[sl]),
                    torch.from_numpy(batch.yaw_rel[sl]),
                    ph,
                    vh,
                    yh,
                    weights=weights,
                )
            k = int(batch.depth[sl].shape[0])
            sums["position_xy_sse"] += float(((ph - torch.from_numpy(batch.position_body[sl]))[..., :2] ** 2).mean()) * k
            sums["n"] += k
            sums["position_sse"] += float(pos) * k
            sums["velocity_sse"] += float(vel) * k
            sums["yaw_sse"] += float(yaw_l) * k
            sums["loss_sum"] += float(total) * k
    n = max(sums["n"], 1)
    pos = sums["position_sse"] / n
    return {
        "n_windows": int(sums["n"]),
        "n_runs": len(paths),
        "loss": sums["loss_sum"] / n if sums["n"] else None,
        "position_mse": pos if sums["n"] else None,
        "velocity_mse": sums["velocity_sse"] / n if sums["n"] else None,
        "yaw_mse": sums["yaw_sse"] / n if sums["n"] else None,
        "position_rmse_m": float(np.sqrt(pos)) if sums["n"] else None,
        "position_xy_mse": sums["position_xy_sse"] / n if sums["n"] else None,
        "position_xy_rmse_m": float(np.sqrt(sums["position_xy_sse"] / n)) if sums["n"] else None,
    }


def window_weights(
    batch,
    path_weight: float,
    cfg: FitConfig,
    clearance: np.ndarray | None = None,
) -> np.ndarray:
    """每个窗口的损失倍数：文件倍数 × 近障倍数 × 贴边净空倍数。

    近障按深度图里最近的有效深度判断（米）。贴边净空只在软带打开、并且调用方传入了
    净空数组时才乘上去。不传净空、或软带关着时，与 D5b 逐位相同。
    """
    w = np.full((batch.n,), float(path_weight), dtype=np.float32)
    if cfg.near_weight != 1.0 and batch.n:
        depth = batch.depth[:, 0] * float(batch.max_range_m)
        valid = batch.depth[:, 1] > 0.5
        nearest = np.where(valid, depth, np.inf).reshape(batch.n, -1).min(axis=1)
        w = np.where(nearest < float(cfg.near_depth_m), w * float(cfg.near_weight), w).astype(np.float32)
    if _soft_weight_active(cfg) and clearance is not None and batch.n:
        clearance = np.asarray(clearance, dtype=np.float64).reshape(-1)
        if clearance.shape[0] != batch.n:
            raise ValueError(f"净空长度 {clearance.shape[0]} 与窗口数 {batch.n} 不一致")
        floor = 0.0 if cfg.min_clearance_m is None else float(cfg.min_clearance_m)
        band = floor + float(cfg.clearance_soft_margin_m)
        low = ~np.isfinite(clearance) | (clearance < band)
        w = np.where(low, w * np.float32(cfg.clearance_low_weight), w).astype(np.float32)
    return w


def weighted_trajectory_loss(
    position,
    velocity,
    yaw,
    position_hat,
    velocity_hat,
    yaw_hat,
    sample_weight,
    *,
    weights=(1.0, 0.25, 0.05),
    planar: bool = False,
    endpoint_weight: float = 0.0,
):
    """逐窗口加权的轨迹损失。planar 时位置、速度只算 x/y。返回 (总损失, 未加权的位置 MSE)。

    endpoint_weight 为 0 时，位置项仍是各元素均方，和 D5b 逐位相同。大于 0 时，视界后一半
    （含 1.6 s 末点）的位置误差乘 ``1 + endpoint_weight``，再按步权重归一。返回的位置 MSE
    始终是未加权的，日志还能和以前比。
    """
    torch = _torch()
    dims = slice(0, 2) if planar else slice(0, 3)
    if endpoint_weight:
        pos_sq = (position_hat - position)[..., dims] ** 2
        pos_e = pos_sq.mean(dim=(1, 2))
        steps = int(pos_sq.shape[1])
        step_w = torch.ones(steps, dtype=pos_sq.dtype, device=pos_sq.device)
        step_w[steps // 2 :] = 1.0 + float(endpoint_weight)
        pos_for_loss = (pos_sq * step_w.view(1, -1, 1)).sum(dim=(1, 2)) / (step_w.sum() * pos_sq.shape[-1])
    else:
        pos_e = ((position_hat - position)[..., dims] ** 2).mean(dim=(1, 2))
        pos_for_loss = pos_e
    vel_e = ((velocity_hat - velocity)[..., dims] ** 2).mean(dim=(1, 2))
    yaw_e = ((yaw_hat - yaw) ** 2).mean(dim=1)
    per = float(weights[0]) * pos_for_loss + float(weights[1]) * vel_e + float(weights[2]) * yaw_e
    w = sample_weight.to(per.dtype)
    total = (w * per).sum() / torch.clamp(w.sum(), min=1e-12)
    return total, pos_e.mean()


def train_from_paths(
    train_paths: list[Path],
    hold_paths: list[Path],
    out_dir: Path,
    *,
    cfg: FitConfig | None = None,
    resume: Path | None = None,
    path_weights: dict | None = None,
) -> dict:
    """按场次文件训练。每个 epoch 写检查点，中断后可从 last.pt 续跑。

    ``path_weights``：{窗口文件路径: 损失倍数}，缺省 1。D5b 用它给 DAgger / 扰动重标注窗口加权。
    """
    weights_by_path = {str(Path(k)): float(v) for k, v in (path_weights or {}).items()}
    cfg = cfg or FitConfig()
    weighted = not uses_unweighted_d5_loss(cfg, weights_by_path)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ckpt_dir = out_dir / "checkpoints"
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "train-log.jsonl"
    state_path = out_dir / "train-state.json"

    torch = _torch(cfg.num_threads)
    torch.manual_seed(int(cfg.seed))
    np.random.seed(int(cfg.seed))

    start_epoch = 0
    history: list[dict] = []
    if resume is None and (ckpt_dir / "last.pt").is_file() and state_path.is_file():
        resume = ckpt_dir / "last.pt"
    if resume is not None and Path(resume).is_file():
        model, meta = load_checkpoint(resume, enabled=True)
        start_epoch = int(meta.get("epoch", 0))
        if state_path.is_file():
            history = list(json.loads(state_path.read_text(encoding="utf-8")).get("history") or [])
    else:
        model = MultiHeadStudent(StudentConfig(enabled=True))

    net = model._module()
    opt = torch.optim.Adam(net.parameters(), lr=float(cfg.lr))
    opt_path = ckpt_dir / "last-opt.pt"
    if resume is not None and opt_path.is_file() and start_epoch > 0:
        opt.load_state_dict(torch.load(opt_path, map_location="cpu", weights_only=False))

    rng = np.random.default_rng(int(cfg.seed) + start_epoch * 1009)
    wall0 = time.perf_counter()
    best_hold = None
    best_path = out_dir / "student.pt"

    for epoch in range(start_epoch, int(cfg.epochs)):
        order = np.arange(len(train_paths))
        rng.shuffle(order)
        net.train()
        running = {"n": 0, "loss": 0.0, "position_mse": 0.0}
        steps = 0
        pending = {k: [] for k in ("depth", "state", "position", "velocity", "yaw", "neighbor", "mask", "weight")}

        def _flush():
            nonlocal steps
            n = len(pending["depth"])
            if n == 0:
                return
            depth = np.concatenate(pending["depth"], axis=0)
            state = np.concatenate(pending["state"], axis=0)
            pos = np.concatenate(pending["position"], axis=0)
            vel = np.concatenate(pending["velocity"], axis=0)
            yaw = np.concatenate(pending["yaw"], axis=0)
            neigh = np.concatenate(pending["neighbor"], axis=0)
            mask = np.concatenate(pending["mask"], axis=0)
            wts = np.concatenate(pending["weight"], axis=0) if weighted else None
            for k in pending:
                pending[k].clear()
            bs = int(cfg.batch_size)
            idx = np.arange(depth.shape[0])
            rng.shuffle(idx)
            for start in range(0, idx.size, bs):
                sl = idx[start : start + bs]
                opt.zero_grad(set_to_none=True)
                ph, vh, yh = net(
                    torch.from_numpy(depth[sl]),
                    torch.from_numpy(state[sl]),
                    torch.from_numpy(neigh[sl]),
                    torch.from_numpy(mask[sl]),
                )
                if weighted:
                    total, pos_l = weighted_trajectory_loss(
                        torch.from_numpy(pos[sl]),
                        torch.from_numpy(vel[sl]),
                        torch.from_numpy(yaw[sl]),
                        ph,
                        vh,
                        yh,
                        torch.from_numpy(wts[sl]),
                        weights=cfg.loss_weights,
                        planar=bool(cfg.planar),
                        endpoint_weight=float(cfg.endpoint_weight),
                    )
                else:
                    total, pos_l, _, _ = trajectory_loss(
                        torch.from_numpy(pos[sl]),
                        torch.from_numpy(vel[sl]),
                        torch.from_numpy(yaw[sl]),
                        ph,
                        vh,
                        yh,
                        weights=cfg.loss_weights,
                    )
                total.backward()
                opt.step()
                k = int(sl.size)
                running["n"] += k
                running["loss"] += float(total.detach()) * k
                running["position_mse"] += float(pos_l.detach()) * k
                steps += 1

        for j in order:
            path = train_paths[int(j)]
            batch = load_batch(path)
            clearance = None
            if cfg.min_clearance_m is not None or _soft_weight_active(cfg):
                keep, clearance = _prepare_clearance(path, batch.n, cfg)
                if keep is not None:
                    if clearance is not None:
                        clearance = clearance[keep]
                    batch = _take_windows(batch, keep)
            if batch.n == 0:
                continue
            pending["depth"].append(batch.depth)
            pending["state"].append(batch.state)
            pending["position"].append(batch.position_body)
            pending["velocity"].append(batch.velocity_body)
            pending["yaw"].append(batch.yaw_rel)
            pending["neighbor"].append(batch.neighbor_rel)
            pending["mask"].append(batch.neighbor_mask)
            if weighted:
                pending["weight"].append(
                    window_weights(batch, weights_by_path.get(str(Path(path)), 1.0), cfg, clearance)
                )
            # 攒到大约 shuffle_batches 个 batch 再洗一次，避免一场就把内存撑满。
            if sum(x.shape[0] for x in pending["depth"]) >= int(cfg.batch_size) * int(cfg.shuffle_batches):
                _flush()
        _flush()

        train_loss = running["loss"] / max(running["n"], 1)
        train_pos = running["position_mse"] / max(running["n"], 1)
        hold = evaluate_paths(model, hold_paths, batch_size=max(int(cfg.batch_size), 128), cfg=cfg)
        rec = {
            "epoch": epoch + 1,
            "steps": steps,
            "n_train_windows_seen": running["n"],
            "train_loss": train_loss,
            "train_position_mse": train_pos,
            "holdout": hold,
            "wall_s": time.perf_counter() - wall0,
        }
        history.append(rec)
        meta = {
            "epoch": epoch + 1,
            "fit": asdict(cfg),
            "train_runs": len(train_paths),
            "hold_runs": len(hold_paths),
            "last": rec,
        }
        last_pt, last_sha = save_checkpoint(model, ckpt_dir / "last.pt", meta)
        torch.save(opt.state_dict(), opt_path)
        epoch_pt, _ = save_checkpoint(model, ckpt_dir / f"epoch-{epoch + 1}.pt", meta)
        hold_mse = hold.get("position_xy_mse" if cfg.planar else "position_mse")
        if hold_mse is not None and (best_hold is None or hold_mse < best_hold):
            best_hold = hold_mse
            save_checkpoint(model, best_path, {**meta, "best_hold_position_mse": best_hold})
        with log_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        state_path.write_text(
            json.dumps({"history": history, "best_hold_position_mse": best_hold}, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        del last_pt, last_sha, epoch_pt

    if not best_path.is_file() and (ckpt_dir / "last.pt").is_file():
        model, meta = load_checkpoint(ckpt_dir / "last.pt", enabled=True)
        save_checkpoint(model, best_path, meta)

    final_hold = evaluate_paths(model if best_path.is_file() else model, hold_paths, cfg=cfg)
    if best_path.is_file():
        best_model, best_meta = load_checkpoint(best_path, enabled=True)
        final_hold = evaluate_paths(best_model, hold_paths, cfg=cfg)
        sha = best_meta["weights_sha256"]
    else:
        sha = None
        best_meta = {}
    summary = {
        "weights": str(best_path) if best_path.is_file() else None,
        "weights_sha256": sha,
        "epochs_done": history[-1]["epoch"] if history else start_epoch,
        "fit": asdict(cfg),
        "history": history,
        "holdout": final_hold,
        "wall_s": time.perf_counter() - wall0,
    }
    (out_dir / "train-summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return summary
