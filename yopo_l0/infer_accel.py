"""单次前向的加速开关。不改权重，不改候选打分。

默认 `infer`：`torch.inference_mode` + 1 个 intra-op 线程。和 `torch.no_grad`
在这条网络上逐位相同，所以直接打开。

`YOPO_INFER_ACCEL`：

- `baseline`：`no_grad` + 1 线程。E-C3 记 `plan_wall_ms` 时的路径。
- `infer`（默认）：`inference_mode` + 1 线程。
- `threads`：`inference_mode`，线程数看 `YOPO_INFER_THREADS`（默认 4）。
  卷积累加顺序和 1 线程不同，不是逐位相同。单次前向的 argmax 往往不变，
  但闭环会漂：六机林 seed 0 的位置最大偏差到十几米，到达时刻也变。
- `ort`：ONNX Runtime CPU，线程数同样看 `YOPO_INFER_THREADS`（默认 4）。
  需要已安装的 `onnxruntime`，不写进 requirements。单次前向的误差和 `threads`
  同一量级，六机林闭环同样对不上基线轨迹。

这两档留在开关里，是为了按研究记录复现，不是可以默默打开的加速。
数字和容差写在 `docs/research/p1-inference-speed.md`。
开关在进程里第一次构造 `InProcessSource` 之前就得设好。
"""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from yopo_swarm_core.baseline import BaselineModel, CandidateBatch

_MODES = ("baseline", "infer", "threads", "ort")
_THREADS_APPLIED: int | None = None
_ORT: tuple[int, object] | None = None


def accel_name() -> str:
    raw = os.environ.get("YOPO_INFER_ACCEL", "infer").strip().lower()
    if raw in {"", "default", "infer"}:
        return "infer"
    if raw in {"baseline", "0", "off"}:
        return "baseline"
    if raw in _MODES:
        return raw
    raise ValueError(f"YOPO_INFER_ACCEL 必须是 baseline、infer、threads、ort 之一，收到 {raw!r}")


def accel_threads() -> int:
    """baseline / infer 固定 1 线程。threads / ort 默认 4，可用 YOPO_INFER_THREADS 改。"""
    if accel_name() not in {"threads", "ort"}:
        return 1
    raw = os.environ.get("YOPO_INFER_THREADS", "4").strip()
    try:
        n = int(raw)
    except ValueError as exc:
        raise ValueError(f"YOPO_INFER_THREADS 必须是正整数，收到 {raw!r}") from exc
    if n < 1:
        raise ValueError(f"YOPO_INFER_THREADS 必须是正整数，收到 {n}")
    return n


def _ensure_torch_threads(n: int) -> None:
    global _THREADS_APPLIED
    if _THREADS_APPLIED == n:
        return
    import torch

    torch.set_num_threads(n)
    _THREADS_APPLIED = n


def configure(model: BaselineModel) -> None:
    """构造推理源时调用一次：把线程数设对，ort 模式把会话建好（建会话不进 plan_wall）。"""
    name = accel_name()
    if name == "ort":
        _ensure_torch_threads(1)
        _ort_session(model, accel_threads())
        return
    _ensure_torch_threads(accel_threads())


def run_forward(model: BaselineModel, depth: np.ndarray, obs: np.ndarray) -> CandidateBatch:
    name = accel_name()
    if name == "ort":
        _ensure_torch_threads(1)
        return _ort_forward(model, depth, obs, accel_threads())
    _ensure_torch_threads(accel_threads())
    if name == "baseline":
        return model.forward(depth, obs)
    import torch

    # 开关在 yopo_l0 外侧包一层。vendored BaselineModel.forward 仍走内部的
    # no_grad，不改 core/。inference_mode 包在外层时，内部 no_grad 不会退出它。
    with torch.inference_mode():
        return model.forward(depth, obs)


def _ort_session(model: BaselineModel, threads: int):
    global _ORT
    if _ORT is not None and _ORT[0] == threads:
        return _ORT[1]
    try:
        import onnxruntime as ort
    except ImportError as exc:
        raise RuntimeError("YOPO_INFER_ACCEL=ort 需要已安装的 onnxruntime") from exc

    import torch
    from yopo_swarm_core.baseline import default_cache_dir

    cfg = model.config
    vertical = int(cfg["vertical_num"])
    horizon = int(cfg["horizon_num"])
    down = int(cfg["downsample_factor"])
    depth = torch.zeros(1, 1, vertical * down, horizon * down)
    obs = torch.zeros(1, 9, vertical, horizon)
    path = default_cache_dir() / "ort" / model.weights_sha256 / "opset17-v1.onnx"
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.is_file():
        torch.onnx.export(
            model._net,
            (depth, obs),
            str(path),
            input_names=["depth", "obs"],
            output_names=["endstate", "score", "radius"],
            opset_version=17,
            dynamo=False,
        )
    options = ort.SessionOptions()
    options.intra_op_num_threads = threads
    options.inter_op_num_threads = 1
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
    # 第一次图执行算在构造里，不进单次规划墙钟。
    session.run(
        None,
        {
            "depth": np.zeros((1, 1, vertical * down, horizon * down), np.float32),
            "obs": np.zeros((1, 9, vertical, horizon), np.float32),
        },
    )
    _ORT = (threads, session)
    return session


def _ort_forward(model: BaselineModel, depth: np.ndarray, obs: np.ndarray, threads: int) -> CandidateBatch:
    import torch
    from yopo_swarm_core.baseline import CandidateBatch

    session = _ort_session(model, threads)
    d = np.ascontiguousarray(depth, dtype=np.float32)
    o = np.ascontiguousarray(obs, dtype=np.float32)
    with torch.inference_mode():
        prepared = model._st.prepare_input(model._st.normalize_obs(torch.from_numpy(o))).numpy()
    endstate, score, radius = session.run(None, {"depth": d, "obs": np.ascontiguousarray(prepared)})
    b = endstate.shape[0]
    params = np.ascontiguousarray(endstate.transpose(0, 2, 3, 1).reshape(b, -1, endstate.shape[1]))
    flat_score = np.ascontiguousarray(score.reshape(b, -1))
    flat_radius = np.ascontiguousarray(radius.transpose(0, 2, 3, 1).reshape(b, -1, radius.shape[1]))
    return CandidateBatch(traj_params=params.copy(), score=flat_score.copy(), radius=flat_radius.copy())
