"""单机多头连续轨迹学生。默认关。飞行环不导入本模块。

YOPO 在离散候选格子上输出末端参数、分数和走廊，飞行时对过走廊的分数做 argmax。
这里没有格子，也没有分数头。共享干路后面三个头回归同一条连续曲线在固定时刻上的
位置、速度和相对偏航，监督来自 EGO-Swarm 教师轨迹。

``StudentConfig.enabled`` 默认 False。``predict`` 在关着的时候直接返回 None，不跑网络。
打开只发生在显式构造 ``StudentConfig(enabled=True)`` 或 ``predict(..., enabled=True)`` 的地方。
D5 才允许接到闭环，本票不接。
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


@dataclass(frozen=True)
class StudentConfig:
    """学生的形状和开关。数值是本票为了 CPU 冒烟定的，不是论文参数，也不是 YOPO 的格子。"""

    enabled: bool = False
    depth_height: int = DEPTH_HEIGHT
    depth_width: int = DEPTH_WIDTH
    horizon_steps: int = HORIZON_STEPS
    horizon_dt_s: float = HORIZON_DT_S
    state_dim: int = STATE_DIM

    @property
    def horizon_s(self) -> float:
        return float(self.horizon_steps) * float(self.horizon_dt_s)


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

    def forward(self, depth, state):
        """depth (B, 2, H, W)，state (B, 9)。关着时拒绝，不计算。"""
        if not self.cfg.enabled:
            raise RuntimeError(
                "学生默认关。前向要 StudentConfig(enabled=True)。飞行环不调用这里。"
            )
        torch = _torch()
        net = self._module()
        depth_t = depth if isinstance(depth, torch.Tensor) else torch.as_tensor(depth)
        state_t = state if isinstance(state, torch.Tensor) else torch.as_tensor(state)
        return net(depth_t, state_t)


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


def predict(model: MultiHeadStudent | None, depth, state, *, enabled: bool | None = None) -> ContinuousTrajectory | None:
    """推理入口。默认关：返回 None，不建网络。

    ``enabled`` 为 None 时用模型配置。模型是 None 且没有显式打开时也是关。
    ``enabled=False`` 即使模型配置打开也直接返回。
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
    net = model._module()
    net.eval()
    with torch.no_grad():
        pos, vel, yaw = net(
            torch.from_numpy(depth_b),
            torch.from_numpy(state_b),
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

    model = MultiHeadStudent(cfg)
    net = model._module()
    opt = torch.optim.Adam(net.parameters(), lr=float(lr))
    depth_t = torch.from_numpy(depth_b)
    state_t = torch.from_numpy(state_b)
    pos_t = torch.from_numpy(pos_b)
    vel_t = torch.from_numpy(vel_b)
    yaw_t = torch.from_numpy(yaw_b)

    def _eval():
        net.eval()
        with torch.no_grad():
            ph, vh, yh = net(depth_t, state_t)
            total, pos, vel, yaw_l = trajectory_loss(pos_t, vel_t, yaw_t, ph, vh, yh)
        return float(total), float(pos), float(vel), float(yaw_l)

    loss0, _, _, _ = _eval()
    for _ in range(int(steps)):
        net.train()
        opt.zero_grad(set_to_none=True)
        ph, vh, yh = net(depth_t, state_t)
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
                self.fuse = torch.nn.Sequential(
                    torch.nn.Linear(128, 128),
                    torch.nn.ReLU(inplace=True),
                )
                k = cfg.horizon_steps
                self.position_head = torch.nn.Linear(128, k * 3)
                self.velocity_head = torch.nn.Linear(128, k * 3)
                self.yaw_head = torch.nn.Linear(128, k)

            def forward(self, depth, state):
                x = self.conv(depth)
                x = self.depth_fc(torch.flatten(x, 1))
                s = self.state_fc(state)
                h = self.fuse(torch.cat([x, s], dim=1))
                b = depth.shape[0]
                k = self.cfg.horizon_steps
                return (
                    self.position_head(h).reshape(b, k, 3),
                    self.velocity_head(h).reshape(b, k, 3),
                    self.yaw_head(h).reshape(b, k),
                )

        return Net()
