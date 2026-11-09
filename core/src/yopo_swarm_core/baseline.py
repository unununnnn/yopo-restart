"""G0 基线：按固定 SHA 取 YOPO-MINCO 上游源码与权重，校验后在 CPU 上跑前向（ROS-free）。

用法：python -m yopo_swarm_core.baseline [--cache DIR] [--write-golden]
每次运行打印上游 SHA、权重 sha256、config_hash，并与仓库金样比对。

torch 与上游包只在 load_verified_model 内延迟导入；本模块顶层不依赖 torch。
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
import subprocess
import sys
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

UPSTREAM_URL = "https://github.com/TJU-Aerial-Robotics/YOPO.git"
UPSTREAM_SHA = "d5e1e60f83cc991602f142d9b6b96204a24d6b9f"  # ADR-0006
WEIGHTS_RELPATH = "YOPO/saved/YOPO_1/epoch50.pth"
WEIGHTS_SHA256 = "2d85e712c8874d11cd4e523777698eff78914fdb8f1ffb5068b037f8f972f62f"  # ADR-0007
PARAMS_PER_CANDIDATE = 14

# 金样容差：同一 CPU 上逐次运行结果逐位一致；跨机器/BLAS/torch 小版本的 float32 差异
# 实测远小于此值。容差放宽到这里以上的差异视为回归。
GOLDEN_RTOL = 1e-4
GOLDEN_ATOL = 1e-5
GOLDEN_PATH = Path(__file__).resolve().parents[2] / "tests" / "golden" / "minco_d5e1e60_cpu.npz"
FIXED_INPUT_SEED = 20261024


class WeightsVerificationError(RuntimeError):
    """权重文件缺失或 sha256 不符，拒绝加载。"""


class UpstreamFetchError(RuntimeError):
    pass


# --- 纯函数 -----------------------------------------------------------------


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def verify_weights(path: os.PathLike | str, expected: str = WEIGHTS_SHA256) -> str:
    """校验权重 sha256，返回实际哈希；缺失或不符时抛 WeightsVerificationError。"""
    p = Path(path)
    if not p.is_file():
        raise WeightsVerificationError(f"weights file not found: {p}")
    actual = _sha256_file(p)
    if actual != expected:
        raise WeightsVerificationError(
            f"weights sha256 mismatch for {p}: expected {expected}, got {actual}"
        )
    return actual


def _plain(obj: Any) -> Any:
    if isinstance(obj, Mapping):
        return {str(k): _plain(v) for k, v in obj.items()}
    if isinstance(obj, list | tuple):
        return [_plain(v) for v in obj]
    if isinstance(obj, np.generic):
        return obj.item()
    return obj


def config_hash(cfg: Mapping[str, Any]) -> str:
    """配置的规范 JSON（键排序）sha256。整数与浮点（10 vs 10.0）视为不同。"""
    text = json.dumps(_plain(cfg), sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def candidate_count(cfg: Mapping[str, Any]) -> int:
    return int(cfg["horizon_num"]) * int(cfg["vertical_num"])


def fixed_inputs(cfg: Mapping[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """金样用的固定输入：深度 (1,1,H,W) ∈ [0,1]，状态 (1,9) = 速度、加速度、局部目标（机体系）。

    使用 numpy PCG64（跨 numpy 版本稳定），不依赖 torch 随机数。
    """
    h = int(cfg["vertical_num"]) * int(cfg["downsample_factor"])
    w = int(cfg["horizon_num"]) * int(cfg["downsample_factor"])
    rng = np.random.Generator(np.random.PCG64(FIXED_INPUT_SEED))
    depth = rng.random((1, 1, h, w), dtype=np.float32)
    obs = np.array([[1.0, 0.2, -0.1, 0.1, 0.0, 0.0, 8.0, 1.0, 0.5]], dtype=np.float32)
    return depth, obs


# --- 上游获取 ---------------------------------------------------------------


def default_cache_dir() -> Path:
    env = os.environ.get("YOPO_SWARM_CACHE")
    if env:
        return Path(env)
    return Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "yopo-swarm"


def _git(*args: str, cwd: Path) -> str:
    r = subprocess.run(["git", *args], check=False, cwd=cwd, capture_output=True, text=True)
    if r.returncode != 0:
        raise UpstreamFetchError(f"git {' '.join(args)} failed: {r.stderr.strip()}")
    return r.stdout.strip()


def _head(path: Path) -> str | None:
    if not (path / ".git").exists():
        return None
    try:
        return _git("rev-parse", "HEAD", cwd=path)
    except UpstreamFetchError:
        return None


def fetch_upstream(
    cache_dir: Path | None = None, url: str = UPSTREAM_URL, sha: str = UPSTREAM_SHA
) -> Path:
    """把上游固定提交浅取到缓存目录，返回工作树根。幂等：已存在且 HEAD 正确则直接返回。"""
    dest = (cache_dir or default_cache_dir()) / "upstream" / sha
    if _head(dest) == sha:
        return dest
    tmp = dest.with_name(dest.name + ".partial")
    if tmp.exists():
        subprocess.run(["rm", "-rf", str(tmp)], check=True)  # 仅清理本函数自己的临时目录
    tmp.mkdir(parents=True)
    _git("init", "-q", cwd=tmp)
    _git("fetch", "-q", "--depth", "1", url, sha, cwd=tmp)
    _git("checkout", "-q", "--detach", "FETCH_HEAD", cwd=tmp)
    if _head(tmp) != sha:
        raise UpstreamFetchError(f"fetched HEAD {_head(tmp)} != pinned {sha}")
    if dest.exists():
        raise UpstreamFetchError(f"{dest} exists but HEAD is not {sha}; refusing to overwrite")
    tmp.rename(dest)
    return dest


# --- 模型 -------------------------------------------------------------------


@dataclass(frozen=True)
class CandidateBatch:
    """一次前向的全部候选（展平顺序与上游 inference 一致：vertical 行优先，再 horizon）。

    traj_params: (B, N, 14) 网络原始轨迹参数（tanh 后，未解码）
    score:       (B, N)     候选分数（softplus，越大越好）
    radius:      (B, N, 2·radius_num) 预测走廊 μ 与置信 b（严格准入 D3：μ_lo ≥ safe_mu 才可执行；仍不是障碍证明）
    """

    traj_params: np.ndarray
    score: np.ndarray
    radius: np.ndarray


class BaselineModel:
    def __init__(self, network: Any, state_transform: Any, config: dict, meta: dict) -> None:
        self._net = network
        self._st = state_transform
        self.config = config
        self.upstream_sha: str = meta["upstream_sha"]
        self.weights_sha256: str = meta["weights_sha256"]
        self.config_hash: str = config_hash(config)

    def forward(self, depth: np.ndarray, obs: np.ndarray) -> CandidateBatch:
        import torch

        with torch.no_grad():
            d = torch.from_numpy(np.ascontiguousarray(depth, dtype=np.float32))
            o = torch.from_numpy(np.ascontiguousarray(obs, dtype=np.float32))
            o = self._st.prepare_input(self._st.normalize_obs(o))
            endstate, score, radius = self._net.forward(d, o)
            b = endstate.shape[0]
            params = endstate.permute(0, 2, 3, 1).reshape(b, -1, endstate.shape[1])
            radius = radius.permute(0, 2, 3, 1).reshape(b, -1, radius.shape[1])
            return CandidateBatch(
                traj_params=params.numpy().copy(),
                score=score.reshape(b, -1).numpy().copy(),
                radius=radius.numpy().copy(),
            )


def load_verified_model(
    upstream_root: Path, weights_path: Path | None = None, expected_sha256: str = WEIGHTS_SHA256
) -> BaselineModel:
    """先校验权重哈希（不通过则在导入 torch 之前拒绝），再加载上游网络到 CPU。"""
    upstream_root = Path(upstream_root)
    wpath = Path(weights_path) if weights_path else upstream_root / WEIGHTS_RELPATH
    actual = verify_weights(wpath, expected_sha256)
    head = _head(upstream_root)
    if head != UPSTREAM_SHA:
        raise UpstreamFetchError(f"upstream at {upstream_root} is {head}, expected {UPSTREAM_SHA}")

    pkg_root = str(upstream_root / "YOPO")
    loaded = sys.modules.get("policy.yopo_network")
    if loaded is not None and not str(getattr(loaded, "__file__", "")).startswith(pkg_root):
        raise RuntimeError(f"upstream 'policy' already imported from {loaded.__file__}")
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)

    import torch

    # 上游导入时会打印参数横幅，转到 stderr 以保持 stdout 只有本工具的输出
    with contextlib.redirect_stdout(sys.stderr):
        from config.config import cfg  # type: ignore[import-not-found]
        from policy.state_transform import StateTransform  # type: ignore[import-not-found]
        from policy.yopo_network import YopoNetwork  # type: ignore[import-not-found]

        net = YopoNetwork()
    state = torch.load(wpath, map_location="cpu", weights_only=True)
    net.load_state_dict(state)
    net.to("cpu").eval()
    return BaselineModel(
        net,
        StateTransform(),
        _plain(dict(cfg._data)),
        {"upstream_sha": head, "weights_sha256": actual},
    )


# --- CLI --------------------------------------------------------------------


def _golden_arrays(model: BaselineModel) -> dict[str, np.ndarray]:
    import torch

    depth, obs = fixed_inputs(model.config)
    out = model.forward(depth, obs)
    return {
        "depth": depth,
        "obs": obs,
        "traj_params": out.traj_params,
        "score": out.score,
        "radius": out.radius,
        "upstream_sha": np.array(model.upstream_sha),
        "weights_sha256": np.array(model.weights_sha256),
        "config_hash": np.array(model.config_hash),
        "torch_version": np.array(torch.__version__),
    }


def compare_golden(model: BaselineModel, golden_path: Path = GOLDEN_PATH) -> list[str]:
    """返回不一致项列表（空表示通过）。"""
    g = np.load(golden_path)
    cur = _golden_arrays(model)
    problems = []
    for key in ("upstream_sha", "weights_sha256", "config_hash"):
        if str(g[key]) != str(cur[key]):
            problems.append(f"{key}: golden {g[key]} != current {cur[key]}")
    if not np.array_equal(g["depth"], cur["depth"]) or not np.array_equal(g["obs"], cur["obs"]):
        problems.append("fixed inputs differ from golden")
    for key in ("traj_params", "score", "radius"):
        if not np.allclose(cur[key], g[key], rtol=GOLDEN_RTOL, atol=GOLDEN_ATOL):
            err = float(np.max(np.abs(cur[key] - g[key])))
            problems.append(f"{key}: max abs diff {err:.3e} exceeds tolerance")
    return problems


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m yopo_swarm_core.baseline", description=__doc__)
    ap.add_argument("--cache", type=Path, default=None, help="上游缓存目录")
    ap.add_argument("--golden", type=Path, default=GOLDEN_PATH, help="金样 .npz 路径")
    ap.add_argument("--write-golden", action="store_true", help="重新生成金样（覆盖 --golden）")
    args = ap.parse_args(argv)

    import torch

    torch.set_num_threads(1)
    root = fetch_upstream(args.cache)
    model = load_verified_model(root)
    print(f"upstream_sha: {model.upstream_sha}")
    print(f"weights_sha256: {model.weights_sha256}")
    print(f"config_hash: {model.config_hash}")
    print(f"torch: {torch.__version__}")
    print(f"candidates: {candidate_count(model.config)}")
    if args.write_golden:
        args.golden.parent.mkdir(parents=True, exist_ok=True)
        np.savez(args.golden, **_golden_arrays(model))
        print(f"golden: WRITTEN {args.golden}")
        return 0
    problems = compare_golden(model, args.golden)
    if problems:
        print("golden: FAIL")
        for p in problems:
            print(f"  - {p}")
        return 1
    print("golden: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
