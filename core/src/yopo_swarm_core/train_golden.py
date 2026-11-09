"""Y6 金样：固定 batch 上逐项损失、2 步训练后的权重指纹（上游 d5e1e60 训练代码，CPU）。

- 损失项（``loss_terms``）只依赖上游 ``loss/*`` + ``minco/*`` + 配置，CI 也能跑。
- 训练层损失（rank / inner_reg / radius / total）与 2 步训练依赖上游 ``policy/yopo_trainer.py``，
  它会 import matplotlib / rich / tensorboard / cv2 / sklearn（训练依赖，见 docker/infer-cu13/）。
- 输入轨迹在生成金样时用种子化网络前向得到并存进 npz，比较时不再依赖网络数值。
"""

from __future__ import annotations

import contextlib
import os
import random
import tempfile
from collections.abc import Iterator
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from . import train_shim, train_synth

GOLDEN_PATH = Path(__file__).resolve().parents[2] / "tests" / "golden" / "minco_loss_d5e1e60.npz"
LOSS_RTOL = 1e-6  # 验收：各项相对误差 ≤ 1e-6（按该项最大幅值归一）
WEIGHT_ATOL = 1e-5  # 验收：2 步后权重 max_abs ≤ 1e-5
# 验收按金样生成环境（box x86_64 CPU，torch 2.4.1）定义，上面两条只在该环境里作门。
# 根因：固定 batch 的 MINCO 损失在 float32 下条件数大，输入扰动 1 ulp 就让 Acc 项相对变化
# ~1.1e-5（box 实测，见 ulp_sensitivity）。1e-6 只有逐位相同的内核才能达到；Orin aarch64 的
# GEMM 归约顺序不同，实测 1.33e-5，落在同一个 float32 舍入噪声量级。其他平台用宽一档的界
# 做回归诊断（≥ 4× 1-ulp 敏感度；Orin 实测 2.08e-5），不代表验收通过。
REFERENCE_ENV = ("x86_64", "2.4.1")
CROSS_PLATFORM_LOSS_RTOL = 1e-4
CROSS_PLATFORM_WEIGHT_ATOL = 1e-4  # 只用于初始化权重（未经训练，Orin 实测 0）
# 2 步训练后，非参考平台上的权重差由优化器决定，不由 float32 舍入决定（Orin 实测，见 two_step_bound）。
LEARNING_RATE = 1.5e-4  # 上游 YopoTrainer 默认（AdamW，fused）
TWO_STEPS = 2
ADAM_STEP_RATIO = 1.01  # 2 步时偏置校正后 |m̂/√v̂| 的上确界 1.0013，取 1.01
BN_STAT_RTOL = 0.01  # BN running_mean/var：按其 2 步移动量的比例；Orin 实测最大 0.32%
BATCH = 2
TWO_STEP_FRAMES = 5  # train_test_split(test_size=0.1) → 4 train / 1 valid → batch 2 恰好 2 步
FINGERPRINT_PER_TENSOR = 64
COST_KEYS = ("Smooth", "Acc", "Safety", "Goal", "Feasible", "Time")
TRAINER_KEYS = ("rank_loss", "inner_reg_loss", "radius_loss", "loss")


def seed_everything(seed: int = 0) -> None:
    """同上游 train_yopo.configure_random_seed（CPU 部分）。"""
    import torch

    random.seed(seed)
    os.environ["PYTHONHASHSEED"] = str(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def write_fixture_dataset(root: Path, frames: int = TWO_STEP_FRAMES) -> Path:
    return train_synth.write_dataset(root, maps=1, frames=frames)


# --- 损失项 -----------------------------------------------------------------------


def reference_env() -> bool:
    import platform

    import torch

    return (platform.machine(), torch.__version__.split("+")[0]) == REFERENCE_ENV


def loss_rtol() -> float:
    return LOSS_RTOL if reference_env() else CROSS_PLATFORM_LOSS_RTOL


def weight_atol() -> float:
    return WEIGHT_ATOL if reference_env() else CROSS_PLATFORM_WEIGHT_ATOL


def two_step_bound(name: str, moved: float) -> float:
    """2 步训练后单个张量指纹的允许差。参考环境是验收门 WEIGHT_ATOL。

    其他平台（Orin aarch64 实测）：AdamW 把梯度归一化成约 ±lr 的步长，近零梯度分量的符号会因
    舍入而相反（如 layer2.1.bn1.bias：移动 3.0e-4，差 5.5e-4），参数差上界是 2·步数·lr；
    BN 的 running 统计量跟着已分叉的特征走，按其移动量的比例约束。
    """
    if reference_env():
        return WEIGHT_ATOL
    if "running_" in name:
        return BN_STAT_RTOL * moved
    return 2 * TWO_STEPS * LEARNING_RATE * ADAM_STEP_RATIO


def ulp_sensitivity(inp: dict[str, np.ndarray], dataset: Path, seed: int = 0) -> dict[str, float]:
    """把所有 float32 输入随机推 1 ulp，返回各损失项的相对变化（float32 舍入噪声的量级）。"""
    rng = np.random.default_rng(seed)

    def nudge(v: np.ndarray) -> np.ndarray:
        if v.dtype != np.float32:
            return v
        toward = rng.choice(np.array([-np.inf, np.inf], np.float32), size=v.shape)
        return np.nextafter(v, toward).astype(np.float32)

    base = loss_terms(inp, dataset)
    moved = loss_terms({k: nudge(v) for k, v in inp.items()}, dataset)
    keys = [f"cost/{k}" for k in COST_KEYS] + ["aux/radius_dist"]
    return relative_errors(moved, base, keys)


def fixed_observation(cfg) -> dict[str, np.ndarray]:
    """BATCH 帧固定观测：合成地图 0 的解析深度 + 手写的体系 vel/acc/goal。"""
    f = train_synth.forest(0)
    w, h = int(cfg["image_width"]), int(cfg["image_height"])
    pos = np.array([[0.5, 0.0, 1.2], [1.25, 0.1, 1.5]], dtype=np.float32)
    yaw = np.array([0.0, 0.25])
    depth = np.stack(
        [
            train_synth.render_depth(f, p.astype(float), y, w, h)[None] / train_synth.DEPTH_MAX
            for p, y in zip(pos, yaw)
        ]
    ).astype(np.float32)
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.zeros((BATCH, 3, 3), np.float32)
    rot[:, 0, 0], rot[:, 0, 1], rot[:, 1, 0], rot[:, 1, 1], rot[:, 2, 2] = c, -s, s, c, 1.0
    obs = np.array(
        [
            [3.0, 0.2, -0.1, 0.5, 0.0, 0.1, 10.0, 1.0, 0.3],
            [5.0, -0.5, 0.0, -1.0, 0.5, 0.0, 6.0, -2.0, 0.0],
        ],
        dtype=np.float32,
    )
    return {
        "depth": depth,
        "pos": pos,
        "rot": rot,
        "obs_b": obs,
        "map_id": np.zeros(BATCH, np.int64),
    }


def predict_inputs(dataset: Path) -> dict[str, np.ndarray]:
    """生成金样输入：种子 0 初始化的上游网络 + 上游 ``_predict_world``（需训练依赖）。"""
    import torch

    cfg = train_shim.configure(dataset, device="cpu")
    with train_shim.pinned_device("cpu"):
        from policy.yopo_network import YopoNetwork
        from policy.yopo_trainer import YopoTrainer

        seed_everything(0)
        net = YopoNetwork()
    obs = fixed_observation(cfg)
    t = {k: torch.from_numpy(v) for k, v in obs.items()}
    ns = SimpleNamespace(batch_size=BATCH, traj_num=int(cfg["traj_num"]), policy=net)
    with torch.no_grad():
        traj = YopoTrainer._predict_world(ns, t["depth"], t["pos"], t["rot"], t["obs_b"])
    out = {f"in/{k}": v.numpy() for k, v in traj.items()}
    out["in/obs_b_normalized"] = t["obs_b"].numpy()  # inference 原地归一化后的 obs_b
    out["in/map_id"] = obs["map_id"]
    return out


def _tensors(inp: dict[str, np.ndarray]):
    import torch

    return {k.split("/", 1)[1]: torch.from_numpy(np.asarray(v)) for k, v in inp.items()}


def loss_terms(inp: dict[str, np.ndarray], dataset: Path) -> dict[str, np.ndarray]:
    """上游 YOPOLoss 在固定输入上的逐项（已加权）代价与辅助量（不需要训练依赖）。"""
    train_shim.configure(dataset, device="cpu")
    with train_shim.pinned_device("cpu"):
        from loss.loss_function import YOPOLoss

        loss = YOPOLoss()
    t = _tensors(inp)
    costs, aux = loss(
        t["head_w"], t["tail_w"], t["inner_w"], t["goal_w"], t["map_id"], durations=t["durations"]
    )
    out = {f"cost/{k}": costs[k].detach().numpy() for k in COST_KEYS}
    out["aux/collided"] = aux["collided"].numpy()
    out["aux/radius_dist"] = aux["radius_dist"].numpy()
    return out


def trainer_terms(
    inp: dict[str, np.ndarray], terms: dict[str, np.ndarray], dataset: Path
) -> dict[str, np.ndarray]:
    """上游 YopoTrainer.loss_aggregation + _radius_loss（需训练依赖）。"""
    import torch

    cfg = train_shim.configure(dataset, device="cpu")
    with train_shim.pinned_device("cpu"):
        from policy.yopo_trainer import YopoTrainer
    t = _tensors(inp)
    ns = SimpleNamespace(
        batch_size=BATCH,
        traj_num=int(cfg["traj_num"]),
        rank_weight=float(cfg["rank_weight"]),
        inner_reg_weight=float(cfg["w_inner_reg"]),
        radius_lambda=float(cfg["radius_warp_lambda"]),
        radius_over_weight=float(cfg["radius_over_weight"]),
    )
    total = sum(torch.from_numpy(terms[f"cost/{k}"]) for k in COST_KEYS)
    collided = torch.from_numpy(terms["aux/collided"])
    loss, rank, inner = YopoTrainer.loss_aggregation(
        ns, total, t["score"], collided, t["inner_dr"], t["obs_b_normalized"]
    )
    radius = YopoTrainer._radius_loss(ns, t["radius"], torch.from_numpy(terms["aux/radius_dist"]))
    loss = loss + float(cfg["w_radius"]) * radius
    vals = {"rank_loss": rank, "inner_reg_loss": inner, "radius_loss": radius, "loss": loss}
    return {f"trainer/{k}": np.asarray(v.detach().numpy()) for k, v in vals.items()}


def relative_errors(got: dict, want: dict, keys) -> dict[str, float]:
    """每项 max|got − want| / max(max|want|, 1e-12)。"""
    errs = {}
    for k in keys:
        g = np.asarray(got[k], dtype=np.float64)
        w = np.asarray(want[k], dtype=np.float64)
        if g.shape != w.shape:
            errs[k] = float("inf")
            continue
        errs[k] = float(
            np.max(np.abs(g - w), initial=0.0) / max(np.max(np.abs(w), initial=0.0), 1e-12)
        )
    return errs


# --- 2 步训练 ----------------------------------------------------------------------


@contextlib.contextmanager
def dataloader_overrides(num_workers: int = 0, pin_memory: bool = False) -> Iterator[None]:
    """上游 YopoTrainer 写死 num_workers=8、pin_memory=True；这里只改这两个装载参数。"""
    import policy.yopo_trainer as yt

    real = yt.DataLoader

    def loader(*a, **k):
        k["num_workers"], k["pin_memory"] = num_workers, pin_memory
        return real(*a, **k)

    yt.DataLoader = loader
    try:
        yield
    finally:
        yt.DataLoader = real


def make_trainer(
    dataset: Path,
    logdir: Path,
    device: str = "cpu",
    batch_size: int = BATCH,
    learning_rate: float = LEARNING_RATE,
    num_workers: int = 0,
    seed: int = 0,
):
    """上游 YopoTrainer（上游超参：lr 1.5e-4；seed 0），只改路径、设备与装载参数。"""
    train_shim.configure(dataset, device=device)
    with train_shim.pinned_device(device):
        import policy.yopo_trainer as yt

        seed_everything(seed)
        with dataloader_overrides(num_workers, pin_memory=device == "cuda"):
            return yt.YopoTrainer(
                learning_rate=learning_rate,
                batch_size=batch_size,
                tensorboard_path=str(logdir),
                checkpoint_path="",
                save_on_exit=False,
            )


def fingerprint(state_dict) -> dict[str, np.ndarray]:
    out = {}
    for name, v in state_dict.items():
        a = v.detach().cpu().numpy().reshape(-1)
        if a.dtype.kind != "f" or a.size == 0:
            continue
        idx = np.unique(np.linspace(0, a.size - 1, min(a.size, FINGERPRINT_PER_TENSOR)).astype(int))
        out[name] = a[idx].astype(np.float32)
    return out


def two_step_weights(dataset: Path) -> tuple[dict, dict]:
    """返回 (初始化指纹, 上游 train_one_epoch 恰好 2 步后的指纹)。"""
    with tempfile.TemporaryDirectory() as logdir:
        tr = make_trainer(dataset, Path(logdir))
        init = fingerprint(tr.policy.state_dict())
        assert len(tr.train_dataloader) == 2, len(tr.train_dataloader)
        tr.policy.train()
        with tr.progress_log:
            task = tr.progress_log.add_task("golden", total=1)
            tr.train_one_epoch(0, task)
        tr.tensorboard_log.close()
        return init, fingerprint(tr.policy.state_dict())


# --- 金样读写 ----------------------------------------------------------------------


def build_golden(workdir: Path) -> dict[str, np.ndarray]:
    ds = write_fixture_dataset(Path(workdir) / "ds")
    inp = predict_inputs(ds)
    terms = loss_terms(inp, ds)
    out = {**inp, **terms, **trainer_terms(inp, terms, ds)}
    init, two = two_step_weights(ds)
    out.update({f"w0/{k}": v for k, v in init.items()})
    out.update({f"w2/{k}": v for k, v in two.items()})
    return out


def load_golden(path: Path = GOLDEN_PATH) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k] for k in z.files}


def main(argv: list[str] | None = None) -> int:
    import argparse

    ap = argparse.ArgumentParser(description="生成 Y6 训练金样（box CPU, torch 2.4.1）")
    ap.add_argument("--out", type=Path, default=GOLDEN_PATH)
    args = ap.parse_args(argv)
    import torch

    with tempfile.TemporaryDirectory() as d:
        g = build_golden(Path(d))
    g["meta/torch"] = np.array(torch.__version__)
    np.savez_compressed(args.out, **g)
    print(f"wrote {args.out} ({len(g)} arrays)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
