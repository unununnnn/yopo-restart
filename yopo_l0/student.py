"""多头连续轨迹学生。默认关。不选 student 策略时，飞行环不导入本模块。

YOPO 在离散候选格子上输出末端参数、分数和走廊，飞行时对过走廊的分数做 argmax。
这里没有格子，也没有分数头。共享干路后面三个头回归同一条连续曲线在固定时刻上的
位置、速度和相对偏航，监督来自 EGO-Swarm 教师轨迹。

邻机特征是最近 K 架在机体系的相对位置和相对速度。缺位补零，并带存在掩码。
K 默认 3。特征来源是位置和速度，和总线上能广播的量同类，不是 B 样条系数。

``StudentConfig.enabled`` 默认 False。``predict`` 在关着的时候直接返回 None，不跑网络。
打开只发生在显式构造 ``StudentConfig(enabled=True)`` 的地方，以及 ``--policy student`` 选中学生策略时。
不选 ``student`` 时，飞行环不构造网络、不读权重。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 飞行环不读这个常量。库的默认就是关。
DEFAULT_ENABLED = False

DEPTH_CHANNELS = 2
STATE_DIM = 9
HORIZON_STEPS = 8
HORIZON_DT_S = 0.2
DEPTH_HEIGHT = 24
DEPTH_WIDTH = 40
NEIGHBOR_K = 3
NEIGHBOR_FEAT_DIM = 6


@dataclass(frozen=True)
class StudentConfig:
    """学生的形状和开关。数值是本票为了 CPU 冒烟定的，不是论文参数，也不是 YOPO 的格子。"""

    enabled: bool = False
    depth_height: int = DEPTH_HEIGHT
    depth_width: int = DEPTH_WIDTH
    horizon_steps: int = HORIZON_STEPS
    horizon_dt_s: float = HORIZON_DT_S
    state_dim: int = STATE_DIM
    n_neighbors: int = NEIGHBOR_K

    def __post_init__(self) -> None:
        if int(self.n_neighbors) < 1:
            raise ValueError(f"n_neighbors 至少为 1，收到 {self.n_neighbors}")

    @property
    def horizon_s(self) -> float:
        return float(self.horizon_steps) * float(self.horizon_dt_s)


def pack_neighbor_features(
    ego_p,
    ego_v,
    ego_yaw: float,
    neighbor_p,
    neighbor_v,
    *,
    k: int = NEIGHBOR_K,
):
    """最近 K 架邻机的机体系相对位置、相对速度。缺位补零，掩码 1 表示有邻机。

    输入是世界系位置和速度，和 ``yopo_l0.bus`` 上能广播的量同类，不是 B 样条系数。
    相对位置是 ``R_yaw(yaw).T @ (p_j - p_i)``，相对速度是 ``R_yaw(yaw).T @ (v_j - v_i)``。
    按相对位置的欧氏距离升序取最近 k 架；距离相同按下标升序。
    """
    from yopo_l0.policy import R_yaw

    if int(k) < 1:
        raise ValueError(f"k 至少为 1，收到 {k}")
    feat = np.zeros((int(k), NEIGHBOR_FEAT_DIM), dtype=np.float32)
    mask = np.zeros((int(k),), dtype=np.float32)
    neighbor_p = np.asarray(neighbor_p, dtype=np.float64)
    neighbor_v = np.asarray(neighbor_v, dtype=np.float64)
    if neighbor_p.size == 0:
        return feat, mask
    if neighbor_p.ndim == 1:
        neighbor_p = neighbor_p.reshape(1, 3)
        neighbor_v = neighbor_v.reshape(1, 3)
    if neighbor_p.ndim != 2 or neighbor_p.shape[1] != 3:
        raise ValueError(f"neighbor_p 应为 (M, 3)，收到 {neighbor_p.shape}")
    if neighbor_v.shape != neighbor_p.shape:
        raise ValueError(f"neighbor_v 形状应与 neighbor_p 相同，收到 {neighbor_v.shape}")
    world_to_body = R_yaw(float(ego_yaw)).T
    ego_p = np.asarray(ego_p, dtype=np.float64).reshape(3)
    ego_v = np.asarray(ego_v, dtype=np.float64).reshape(3)
    rel_p = (world_to_body @ (neighbor_p - ego_p).T).T
    rel_v = (world_to_body @ (neighbor_v - ego_v).T).T
    dist = np.linalg.norm(rel_p, axis=1)
    order = np.lexsort((np.arange(rel_p.shape[0]), dist))
    for slot, j in enumerate(order[: min(int(k), rel_p.shape[0])]):
        feat[slot, 0:3] = rel_p[j]
        feat[slot, 3:6] = rel_v[j]
        mask[slot] = 1.0
    return feat, mask


def pack_ego_state(p, v, a, goal, yaw: float):
    """机体坐标系：速度 3、加速度 3、目标相对位置 3。和教师窗口的 state 同一套。"""
    from yopo_l0.policy import R_yaw

    world_to_body = R_yaw(float(yaw)).T
    p = np.asarray(p, dtype=np.float64).reshape(3)
    v = np.asarray(v, dtype=np.float64).reshape(3)
    a = np.asarray(a, dtype=np.float64).reshape(3)
    goal = np.asarray(goal, dtype=np.float64).reshape(3)
    return np.concatenate([world_to_body @ v, world_to_body @ a, world_to_body @ (goal - p)]).astype(np.float32)


def resize_depth(depth, height: int, width: int):
    """把深度图缩到学生的空间尺寸。无回波是 NaN，块平均只统计有效像素。

    源尺寸能整除目标时按块平均；否则按面积加权（双线性），无效像素不进入平均。
    某个输出像素没有任何有效来源时，仍是 NaN。
    """
    depth = np.asarray(depth, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError(f"depth 应为 (H, W)，收到 {depth.shape}")
    h1, w1 = int(height), int(width)
    if h1 < 1 or w1 < 1:
        raise ValueError(f"目标尺寸必须为正，收到 {h1}×{w1}")
    h0, w0 = int(depth.shape[0]), int(depth.shape[1])
    if (h0, w0) == (h1, w1):
        return depth.copy()
    if h0 % h1 == 0 and w0 % w1 == 0:
        sh, sw = h0 // h1, w0 // w1
        blocks = depth.reshape(h1, sh, w1, sw)
        valid = np.isfinite(blocks)
        acc = np.where(valid, blocks, 0.0).sum(axis=(1, 3))
        cnt = valid.sum(axis=(1, 3))
        out = np.full((h1, w1), np.nan, dtype=np.float32)
        np.divide(acc, cnt, out=out, where=cnt > 0)
        return out
    # 面积加权：把每个源像素按覆盖比例加到目标格子。
    out_acc = np.zeros((h1, w1), dtype=np.float64)
    out_cnt = np.zeros((h1, w1), dtype=np.float64)
    ys = np.linspace(0.0, h1, h0 + 1)
    xs = np.linspace(0.0, w1, w0 + 1)
    finite = np.isfinite(depth)
    for i in range(h0):
        y0, y1 = ys[i], ys[i + 1]
        r0, r1 = int(np.floor(y0)), int(np.ceil(y1) - 1)
        r0 = max(0, r0)
        r1 = min(h1 - 1, r1)
        if r0 > r1:
            continue
        for j in range(w0):
            if not finite[i, j]:
                continue
            x0, x1 = xs[j], xs[j + 1]
            c0, c1 = int(np.floor(x0)), int(np.ceil(x1) - 1)
            c0 = max(0, c0)
            c1 = min(w1 - 1, c1)
            if c0 > c1:
                continue
            val = float(depth[i, j])
            for r in range(r0, r1 + 1):
                ya, yb = max(y0, r), min(y1, r + 1)
                if yb <= ya:
                    continue
                for c in range(c0, c1 + 1):
                    xa, xb = max(x0, c), min(x1, c + 1)
                    if xb <= xa:
                        continue
                    wgt = (yb - ya) * (xb - xa)
                    out_acc[r, c] += val * wgt
                    out_cnt[r, c] += wgt
    out = np.full((h1, w1), np.nan, dtype=np.float32)
    np.divide(out_acc, out_cnt, out=out, where=out_cnt > 0)
    return out


def depth_from_hw(depth_hw, max_range_m: float, *, height: int = DEPTH_HEIGHT, width: int = DEPTH_WIDTH):
    """观测深度 → 学生输入 (2, H, W)。先缩尺寸，再按教师转换同样的方式归一化。"""
    from yopo_l0.distill import pack_depth

    return pack_depth(resize_depth(depth_hw, height, width), float(max_range_m))


def checkpoint_sha256(path) -> str:
    import hashlib
    from pathlib import Path

    data = Path(path).read_bytes()
    return hashlib.sha256(data).hexdigest()


def save_checkpoint(model: MultiHeadStudent, path, meta: dict | None = None):
    """写出权重和形状。路径由调用方决定；仓库默认不提交这个文件。"""
    from pathlib import Path

    torch = _torch()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": "yopo-student-checkpoint/1",
        "state_dict": model._module().state_dict(),
        "cfg": {
            "depth_height": int(model.cfg.depth_height),
            "depth_width": int(model.cfg.depth_width),
            "horizon_steps": int(model.cfg.horizon_steps),
            "horizon_dt_s": float(model.cfg.horizon_dt_s),
            "state_dim": int(model.cfg.state_dim),
            "n_neighbors": int(model.cfg.n_neighbors),
        },
        "meta": dict(meta or {}),
    }
    torch.save(payload, path)
    return path, checkpoint_sha256(path)


def load_checkpoint(path, *, enabled: bool = True) -> tuple[MultiHeadStudent, dict]:
    """读回权重。默认打开配置，因为飞行和评估需要能前向。"""
    from pathlib import Path

    torch = _torch()
    path = Path(path)
    payload = torch.load(path, map_location="cpu", weights_only=False)
    if not isinstance(payload, dict) or payload.get("schema") != "yopo-student-checkpoint/1":
        raise ValueError(f"{path} 不是 yopo-student-checkpoint/1")
    raw_cfg = dict(payload.get("cfg") or {})
    cfg = StudentConfig(enabled=bool(enabled), **raw_cfg)
    model = MultiHeadStudent(cfg)
    model._module().load_state_dict(payload["state_dict"])
    model._module().eval()
    meta = dict(payload.get("meta") or {})
    meta["weights_sha256"] = checkpoint_sha256(path)
    meta["weights_path"] = str(path)
    return model, meta


@dataclass(frozen=True)
class ContinuousTrajectory:
    """一条连续轨迹，不是候选下标。

    position_body、velocity_body：(B, K, 3)，机体坐标系，时刻 (k+1) * dt。
    yaw_rel：(B, K)，相对当前偏航的弧度。
    """

    position_body: np.ndarray
    velocity_body: np.ndarray
    yaw_rel: np.ndarray


@dataclass(frozen=True)
class SmokeReport:
    steps: int
    loss_initial: float
    loss_final: float
    position_mse_final: float
    velocity_mse_final: float
    yaw_mse_final: float
    overfit: bool
    n_windows: int
    scenario_id: str


def _torch():
    import torch

    return torch


class MultiHeadStudent:
    """延迟建 torch 模块，避免「导入本文件」就改全局线程或跑前向。

    真正的 ``nn.Module`` 在第一次需要计算图时才创建。关着的 ``predict`` 不会创建它。
    """

    def __init__(self, cfg: StudentConfig | None = None):
        self.cfg = cfg or StudentConfig()
        self._net = None

    def _module(self):
        if self._net is None:
            self._net = _StudentNet(self.cfg)
        return self._net

    def forward(self, depth, state, neighbors=None, neighbor_mask=None):
        """depth (B, 2, H, W)，state (B, 9)，邻机 (B, K, 6) 和掩码 (B, K)。关着时拒绝，不计算。"""
        if not self.cfg.enabled:
            raise RuntimeError(
                "学生默认关。前向要 StudentConfig(enabled=True)。飞行环不调用这里。"
            )
        torch = _torch()
        net = self._module()
        depth_t = depth if isinstance(depth, torch.Tensor) else torch.as_tensor(depth)
        state_t = state if isinstance(state, torch.Tensor) else torch.as_tensor(state)
        neigh_t, mask_t = _neighbor_tensors(neighbors, neighbor_mask, depth_t.shape[0], self.cfg, torch)
        return net(depth_t, state_t, neigh_t, mask_t)


def _as_batched(depth: np.ndarray, state: np.ndarray, cfg: StudentConfig):
    depth = np.asarray(depth, dtype=np.float32)
    state = np.asarray(state, dtype=np.float32)
    if depth.ndim == 3:
        depth = depth[None]
    if state.ndim == 1:
        state = state[None]
    if depth.ndim != 4 or depth.shape[1] != DEPTH_CHANNELS:
        raise ValueError(f"depth 应为 (B, 2, H, W)，收到 {depth.shape}")
    if depth.shape[2] != cfg.depth_height or depth.shape[3] != cfg.depth_width:
        raise ValueError(
            f"depth 空间尺寸应为 {cfg.depth_height}×{cfg.depth_width}，收到 {depth.shape[2]}×{depth.shape[3]}"
        )
    if state.ndim != 2 or state.shape[1] != cfg.state_dim or state.shape[0] != depth.shape[0]:
        raise ValueError(f"state 应为 (B, {cfg.state_dim}) 且与 depth 同批，收到 {state.shape}")
    return depth, state


def _as_neighbors(neighbors, neighbor_mask, batch: int, cfg: StudentConfig):
    k = int(cfg.n_neighbors)
    if neighbors is None and neighbor_mask is None:
        return (
            np.zeros((batch, k, NEIGHBOR_FEAT_DIM), dtype=np.float32),
            np.zeros((batch, k), dtype=np.float32),
        )
    if neighbors is None or neighbor_mask is None:
        raise ValueError("neighbors 和 neighbor_mask 必须成对提供，或者都省略（按缺位补零）")
    neighbors = np.asarray(neighbors, dtype=np.float32)
    neighbor_mask = np.asarray(neighbor_mask, dtype=np.float32)
    if neighbors.ndim == 2:
        neighbors = neighbors[None]
    if neighbor_mask.ndim == 1:
        neighbor_mask = neighbor_mask[None]
    if neighbors.shape != (batch, k, NEIGHBOR_FEAT_DIM):
        raise ValueError(f"neighbors 应为 {(batch, k, NEIGHBOR_FEAT_DIM)}，收到 {neighbors.shape}")
    if neighbor_mask.shape != (batch, k):
        raise ValueError(f"neighbor_mask 应为 {(batch, k)}，收到 {neighbor_mask.shape}")
    return neighbors, neighbor_mask


def _neighbor_tensors(neighbors, neighbor_mask, batch: int, cfg: StudentConfig, torch):
    if neighbors is None and neighbor_mask is None:
        return (
            torch.zeros(batch, cfg.n_neighbors, NEIGHBOR_FEAT_DIM, dtype=torch.float32),
            torch.zeros(batch, cfg.n_neighbors, dtype=torch.float32),
        )
    if neighbors is None or neighbor_mask is None:
        raise ValueError("neighbors 和 neighbor_mask 必须成对提供，或者都省略（按缺位补零）")
    neigh_t = neighbors if isinstance(neighbors, torch.Tensor) else torch.as_tensor(neighbors)
    mask_t = neighbor_mask if isinstance(neighbor_mask, torch.Tensor) else torch.as_tensor(neighbor_mask)
    if neigh_t.ndim == 2:
        neigh_t = neigh_t[None]
    if mask_t.ndim == 1:
        mask_t = mask_t[None]
    k = int(cfg.n_neighbors)
    if tuple(neigh_t.shape) != (batch, k, NEIGHBOR_FEAT_DIM):
        raise ValueError(f"neighbors 应为 {(batch, k, NEIGHBOR_FEAT_DIM)}，收到 {tuple(neigh_t.shape)}")
    if tuple(mask_t.shape) != (batch, k):
        raise ValueError(f"neighbor_mask 应为 {(batch, k)}，收到 {tuple(mask_t.shape)}")
    return neigh_t, mask_t


def predict(
    model: MultiHeadStudent | None,
    depth,
    state,
    neighbors=None,
    neighbor_mask=None,
    *,
    enabled: bool | None = None,
) -> ContinuousTrajectory | None:
    """推理入口。默认关：返回 None，不建网络。

    ``enabled`` 为 None 时用模型配置。模型是 None 且没有显式打开时也是关。
    ``enabled=False`` 即使模型配置打开也直接返回。
    关着的时候不检查邻机形状。
    """
    if enabled is None:
        flag = False if model is None else bool(model.cfg.enabled)
    else:
        flag = bool(enabled)
    if not flag:
        return None
    if model is None:
        raise RuntimeError("学生被显式打开，但没有模型。")
    if not model.cfg.enabled:
        # 显式 enabled=True 仍尊重「配置本身是关」：调用方要改配置，而不是在 predict 里把开关翻开。
        return None
    torch = _torch()
    depth_b, state_b = _as_batched(depth, state, model.cfg)
    neigh_b, mask_b = _as_neighbors(neighbors, neighbor_mask, depth_b.shape[0], model.cfg)
    net = model._module()
    net.eval()
    with torch.no_grad():
        pos, vel, yaw = net(
            torch.from_numpy(depth_b),
            torch.from_numpy(state_b),
            torch.from_numpy(neigh_b),
            torch.from_numpy(mask_b),
        )
    return ContinuousTrajectory(
        position_body=pos.numpy().copy(),
        velocity_body=vel.numpy().copy(),
        yaw_rel=yaw.numpy().copy(),
    )


def trajectory_loss(position, velocity, yaw, position_hat, velocity_hat, yaw_hat, weights=(1.0, 0.25, 0.05)):
    """三个头各自的均方误差，再按权重相加。没有分数，也没有候选下标。"""
    torch = _torch()
    pos = torch.mean((position_hat - position) ** 2)
    vel = torch.mean((velocity_hat - velocity) ** 2)
    yaw_l = torch.mean((yaw_hat - yaw) ** 2)
    total = float(weights[0]) * pos + float(weights[1]) * vel + float(weights[2]) * yaw_l
    return total, pos, vel, yaw_l


def train_smoke(
    depth: np.ndarray,
    state: np.ndarray,
    position: np.ndarray,
    velocity: np.ndarray,
    yaw: np.ndarray,
    neighbors: np.ndarray | None = None,
    neighbor_mask: np.ndarray | None = None,
    *,
    enabled: bool = False,
    steps: int = 200,
    seed: int = 0,
    lr: float = 1e-2,
    scenario_id: str = "",
    position_mse_max: float = 1e-3,
) -> SmokeReport:
    """在调用方给的几个窗口上过拟合。默认不训练。

    这是训练集误差，没有留出集。用来证明转换和损失能记住少量成功样本，不是生产模型。
    """
    if not enabled:
        raise RuntimeError("train_smoke 默认不训练。要跑就传 enabled=True。飞行环不调用它。")
    if steps < 1:
        raise ValueError(f"steps 至少为 1，收到 {steps}")
    torch = _torch()
    torch.manual_seed(int(seed))
    cfg = StudentConfig(enabled=True)
    depth_b, state_b = _as_batched(np.asarray(depth, np.float32), np.asarray(state, np.float32), cfg)
    pos_b = np.asarray(position, dtype=np.float32)
    vel_b = np.asarray(velocity, dtype=np.float32)
    yaw_b = np.asarray(yaw, dtype=np.float32)
    if pos_b.shape != (depth_b.shape[0], cfg.horizon_steps, 3):
        raise ValueError(f"position 应为 {(depth_b.shape[0], cfg.horizon_steps, 3)}，收到 {pos_b.shape}")
    if vel_b.shape != pos_b.shape:
        raise ValueError(f"velocity 形状应与 position 相同，收到 {vel_b.shape}")
    if yaw_b.shape != (depth_b.shape[0], cfg.horizon_steps):
        raise ValueError(f"yaw 应为 {(depth_b.shape[0], cfg.horizon_steps)}，收到 {yaw_b.shape}")

    neigh_b, mask_b = _as_neighbors(neighbors, neighbor_mask, depth_b.shape[0], cfg)
    model = MultiHeadStudent(cfg)
    net = model._module()
    opt = torch.optim.Adam(net.parameters(), lr=float(lr))
    depth_t = torch.from_numpy(depth_b)
    state_t = torch.from_numpy(state_b)
    neigh_t = torch.from_numpy(neigh_b)
    mask_t = torch.from_numpy(mask_b)
    pos_t = torch.from_numpy(pos_b)
    vel_t = torch.from_numpy(vel_b)
    yaw_t = torch.from_numpy(yaw_b)

    def _eval():
        net.eval()
        with torch.no_grad():
            ph, vh, yh = net(depth_t, state_t, neigh_t, mask_t)
            total, pos, vel, yaw_l = trajectory_loss(pos_t, vel_t, yaw_t, ph, vh, yh)
        return float(total), float(pos), float(vel), float(yaw_l)

    loss0, _, _, _ = _eval()
    for _ in range(int(steps)):
        net.train()
        opt.zero_grad(set_to_none=True)
        ph, vh, yh = net(depth_t, state_t, neigh_t, mask_t)
        total, _, _, _ = trajectory_loss(pos_t, vel_t, yaw_t, ph, vh, yh)
        total.backward()
        opt.step()
    loss1, pos1, vel1, yaw1 = _eval()
    overfit = bool(np.isfinite(loss1) and pos1 < float(position_mse_max) and loss1 < loss0 * 0.1)
    return SmokeReport(
        steps=int(steps),
        loss_initial=loss0,
        loss_final=loss1,
        position_mse_final=pos1,
        velocity_mse_final=vel1,
        yaw_mse_final=yaw1,
        overfit=overfit,
        n_windows=int(depth_b.shape[0]),
        scenario_id=scenario_id,
    )


class _StudentNet:
    """占位，真正的 nn.Module 在 __new__ 里换成子类，这样本文件顶层不导入 torch。"""

    def __new__(cls, cfg: StudentConfig):
        torch = _torch()

        class Net(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.cfg = cfg
                self.conv = torch.nn.Sequential(
                    torch.nn.Conv2d(DEPTH_CHANNELS, 8, kernel_size=3, stride=2, padding=1),
                    torch.nn.ReLU(inplace=True),
                    torch.nn.Conv2d(8, 16, kernel_size=3, stride=2, padding=1),
                    torch.nn.ReLU(inplace=True),
                    torch.nn.AdaptiveAvgPool2d((4, 4)),
                )
                self.depth_fc = torch.nn.Linear(16 * 4 * 4, 64)
                self.state_fc = torch.nn.Sequential(
                    torch.nn.Linear(cfg.state_dim, 64),
                    torch.nn.ReLU(inplace=True),
                    torch.nn.Linear(64, 64),
                    torch.nn.ReLU(inplace=True),
                )
                self.neighbor_fc = torch.nn.Sequential(
                    torch.nn.Linear(NEIGHBOR_FEAT_DIM, 32),
                    torch.nn.ReLU(inplace=True),
                    torch.nn.Linear(32, 64),
                    torch.nn.ReLU(inplace=True),
                )
                self.fuse = torch.nn.Sequential(
                    torch.nn.Linear(192, 128),
                    torch.nn.ReLU(inplace=True),
                )
                k = cfg.horizon_steps
                self.position_head = torch.nn.Linear(128, k * 3)
                self.velocity_head = torch.nn.Linear(128, k * 3)
                self.yaw_head = torch.nn.Linear(128, k)

            def forward(self, depth, state, neighbors, neighbor_mask):
                x = self.conv(depth)
                x = self.depth_fc(torch.flatten(x, 1))
                s = self.state_fc(state)
                b = depth.shape[0]
                k_nb = self.cfg.n_neighbors
                n = self.neighbor_fc(neighbors.reshape(b * k_nb, NEIGHBOR_FEAT_DIM))
                n = n.reshape(b, k_nb, 64) * neighbor_mask.reshape(b, k_nb, 1)
                n = n.sum(dim=1)
                h = self.fuse(torch.cat([x, s, n], dim=1))
                k = self.cfg.horizon_steps
                return (
                    self.position_head(h).reshape(b, k, 3),
                    self.velocity_head(h).reshape(b, k, 3),
                    self.yaw_head(h).reshape(b, k),
                )

        return Net()
