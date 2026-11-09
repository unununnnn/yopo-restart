"""单机规划器的重放日志（I9）：JSONL 序列化、`ReplayLog` 与 `replay`。

日志头记录运行假设、候选源来源、代码 SHA 与 config_hash；每步记录完整观测、候选源原始输出与
决策（含每个候选的稳定错误码）。浮点数用 `float.hex`、数组用 dtype+shape+base64，保证逐位还原。
`replay` 用日志里的候选源输出重跑规划器，得到逐位相同的决策。
"""

from __future__ import annotations

import argparse
import base64
import gzip
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, final

import numpy as np

from .candidates import DecoderParams
from .freespace import CameraIntrinsics, DepthFrame, FreeSpaceConfig
from .plan import BrakeBranch, Hold, KinematicLimits, OccupancySet, Plan, PlanConfig, Tube
from .planner import (
    CandidateSource,
    Decision,
    Observation,
    PeerOccupancy,
    Planner,
    PlannerConfig,
    VehicleState,
)
from .trajectory import PiecewisePolynomial, PolySegment

__all__ = ["ReplayLog", "replay"]


# --- 序列化（重放日志） ------------------------------------------------------------


def _arr(x: Any) -> dict:
    a = np.ascontiguousarray(np.asarray(x))
    return {
        "dtype": a.dtype.str,
        "shape": list(a.shape),
        "b64": base64.b64encode(a.tobytes()).decode(),
    }


def _unarr(d: dict) -> np.ndarray:
    a = np.frombuffer(base64.b64decode(d["b64"]), dtype=np.dtype(d["dtype"]))
    return a.reshape(d["shape"]).copy()


def _hex(x: float) -> str:
    return float(x).hex()


def _unhex(s: str) -> float:
    return float.fromhex(s)


def _traj_to(tr: PiecewisePolynomial) -> dict:
    return {
        "t0": _hex(tr.t0),
        "valid": [_hex(tr.t_start), _hex(tr.t_end)],
        "segments": [{"duration": _hex(s.duration), "coeffs": _arr(s.coeffs)} for s in tr.segments],
    }


def _traj_from(d: dict) -> PiecewisePolynomial:
    segs = [PolySegment(_unarr(s["coeffs"]), _unhex(s["duration"])) for s in d["segments"]]
    return PiecewisePolynomial(_unhex(d["t0"]), segs, _unhex(d["valid"][0]), _unhex(d["valid"][1]))


def _plan_to(p: Plan) -> dict:
    return {
        "owner": p.owner,
        "plan_seq": p.plan_seq,
        "parent_plan_hash": p.parent_plan_hash,
        "t_switch": _hex(p.t_switch),
        "head_pva": _arr(np.asarray(p.head_pva, dtype=float)),
        "motion": _traj_to(p.motion),
        "branches": [{"t_branch": _hex(b.t_branch), "traj": _traj_to(b.traj)} for b in p.branches],
    }


def _plan_from(d: dict) -> Plan:
    return Plan(
        d["owner"],
        d["plan_seq"],
        d["parent_plan_hash"],
        _unhex(d["t_switch"]),
        _unarr(d["head_pva"]),
        _traj_from(d["motion"]),
        tuple(BrakeBranch(_unhex(b["t_branch"]), _traj_from(b["traj"])) for b in d["branches"]),
    )


def _occ_to(o: OccupancySet) -> list:
    out = []
    for e in o.elements:
        if isinstance(e, Tube):
            out.append({"tube": _traj_to(e.traj), "radius": _hex(e.radius), "label": e.label})
        else:
            out.append(
                {
                    "hold": [_hex(x) for x in e.position],
                    "radius": _hex(e.radius),
                    "from": _hex(e.t_from),
                    "until": _hex(e.t_until),
                    "label": e.label,
                }
            )
    return out


def _occ_from(items: list) -> OccupancySet:
    els: list[Tube | Hold] = []
    for e in items:
        if "tube" in e:
            els.append(Tube(_traj_from(e["tube"]), _unhex(e["radius"]), e["label"]))
        else:
            pos = tuple(_unhex(x) for x in e["hold"])
            els.append(
                Hold(pos, _unhex(e["radius"]), _unhex(e["from"]), _unhex(e["until"]), e["label"])
            )
    return OccupancySet(tuple(els))


def _obs_to(o: Observation) -> dict:
    f, s = o.frame, o.state
    return {
        "frame": {
            "depth": _arr(f.depth),
            "valid": _arr(f.valid),
            "K": vars(f.k).copy(),
            "T": _arr(f.T),
            "stamp": _hex(f.stamp),
        },
        "state": {
            "position": _arr(np.asarray(s.position, dtype=float)),
            "velocity": _arr(np.asarray(s.velocity, dtype=float)),
            "acceleration": _arr(np.asarray(s.acceleration, dtype=float)),
            "R_wb": _arr(np.asarray(s.R_wb, dtype=float)),
            "stamp": _hex(s.stamp),
        },
        "goal": _arr(np.asarray(o.goal, dtype=float)),
        "approved": _plan_to(o.approved),
        "peers": [
            {"owner": p.owner, "version": p.version, "occupancy": _occ_to(p.occupancy)}
            for p in o.peers
        ],
        "now": _hex(o.now),
        "frame_id": o.frame_id,
        "active_branch": o.active_branch,
    }


def _obs_from(d: dict) -> Observation:
    f, s = d["frame"], d["state"]
    k = {kk: (float(v) if isinstance(v, float) else v) for kk, v in f["K"].items()}
    frame = DepthFrame(
        _unarr(f["depth"]),
        _unarr(f["valid"]),
        CameraIntrinsics(**k),
        _unarr(f["T"]),
        _unhex(f["stamp"]),
    )
    state = VehicleState(
        _unarr(s["position"]),
        _unarr(s["velocity"]),
        _unarr(s["acceleration"]),
        _unarr(s["R_wb"]),
        _unhex(s["stamp"]),
    )
    peers = tuple(
        PeerOccupancy(p["owner"], p["version"], _occ_from(p["occupancy"])) for p in d["peers"]
    )
    return Observation(
        frame,
        state,
        _unarr(d["goal"]),
        _plan_from(d["approved"]),
        peers,
        _unhex(d["now"]),
        d["frame_id"],
        d["active_branch"],
    )


@final
class ReplayLog:
    """JSONL 重放日志。第一步写入前写日志头（此时规划器的运行假设已确定）。"""

    SCHEMA = "yopo-swarm/planner-log/1"

    def __init__(self, path: Path, cfg: PlannerConfig, code_sha: str, source: CandidateSource):
        self.path = Path(path)
        self.cfg = cfg
        self.code_sha = code_sha
        self.source = source
        self._fh = None
        self._header_written = False

    def __enter__(self) -> ReplayLog:
        self._fh = self.path.open("w", encoding="utf-8")
        return self

    def __exit__(self, *exc: object) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None

    def _write(self, obj: dict) -> None:
        assert self._fh is not None, "ReplayLog must be used as a context manager"
        self._fh.write(json.dumps(obj, sort_keys=True, ensure_ascii=False) + "\n")

    def write_step(self, planner: Planner, k: int, obs: Observation, net, decision: Decision):
        if not self._header_written:
            self._write(
                {
                    "kind": "header",
                    "schema": self.SCHEMA,
                    "assumptions": planner.assumptions(),
                    "source": self.source.describe() | {"name": self.source.name},
                    "code_sha": self.code_sha,
                    "config": self.cfg.to_canonical(),
                    "config_hash": self.cfg.config_hash(),
                }
            )
            self._header_written = True
        try:
            obs_d = _obs_to(obs)
        except (AttributeError, TypeError, ValueError):
            obs_d = None  # 输入本身不可序列化（已被拒绝为 INPUT_MALFORMED）
        self._write(
            {
                "kind": "step",
                "step": k,
                "observation": obs_d,
                "net": None if net is None else [_arr(x) for x in net],
                "decision": decision.canonical(),
            }
        )


@dataclass
class _LoggedSource:
    outputs: list
    name: str = "logged"
    _i: int = field(default=0)

    def describe(self) -> dict:
        return {"name": self.name}

    def __call__(self, obs: Observation, dec: DecoderParams):
        out = self.outputs[self._i]
        self._i += 1
        if out is None:
            raise RuntimeError("replay asked for a source output the log does not have")
        return tuple(_unarr(x) for x in out)


def _rows(path: Path):
    path = Path(path)
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


def config_from_log(path: Path) -> PlannerConfig:
    """从规范日志头恢复配置，box 不必手工抄写浮点参数。"""
    header = next(_rows(path))
    c = header["config"]

    def floats(d):
        return {
            k: float.fromhex(v) if isinstance(v, str) and v.startswith(("0x", "-0x")) else v
            for k, v in d.items()
        }

    p = floats(c["plan"])
    p["limits"] = KinematicLimits(*(float.fromhex(x) for x in p["limits"]))
    kwargs = floats(
        {k: v for k, v in c.items() if k not in ("plan", "space", "decoder", "branch_offsets")}
    )
    return PlannerConfig(
        plan=PlanConfig(**p),
        space=FreeSpaceConfig(**floats(c["space"])),
        decoder=DecoderParams(**floats(c["decoder"])),
        branch_offsets=tuple(float.fromhex(x) for x in c["branch_offsets"]),
        **kwargs,
    )


def replay(path: Path, cfg: PlannerConfig) -> list[dict]:
    """流式读完整快照，只保留一行深度，避免在 8GB 设备上积累整段原始图像。"""
    rows = iter(_rows(path))
    header = next(rows)
    if header.get("kind") != "header" or header.get("schema") != ReplayLog.SCHEMA:
        raise ValueError("not a planner replay log")
    if header["config_hash"] != cfg.config_hash():
        raise ValueError("config_hash mismatch between log and replay config")
    src = _LoggedSource([])
    pl = Planner(cfg, src)
    for a in header["assumptions"]["free_space"]:
        pl.assume_free(np.array([_unhex(x) for x in a["center"]]), _unhex(a["radius"]), a["label"])
    out = []
    for s in rows:
        if s["observation"] is None:
            raise ValueError(f"step {s['step']}: observation was not serializable")
        src.outputs, src._i = [s["net"]], 0
        out.append(pl.step(_obs_from(s["observation"])).canonical())
    return out


def main():
    ap = argparse.ArgumentParser(description="只重放规划决策，不启动 ROS/PX4/推理服务")
    ap.add_argument("path", type=Path)
    args = ap.parse_args()
    cfg = config_from_log(args.path)
    actual = replay(args.path, cfg)
    rows = iter(_rows(args.path))
    header = next(rows)
    expected = [row["decision"] for row in rows]
    mismatches = [i for i, (a, b) in enumerate(zip(actual, expected, strict=True)) if a != b]
    print(
        json.dumps(
            {
                "steps": len(actual),
                "bitwise_equal": not mismatches,
                "mismatches": mismatches,
                "config_hash": cfg.config_hash(),
                "code_sha": header["code_sha"],
            },
            indent=2,
        )
    )
    return int(bool(mismatches))


if __name__ == "__main__":
    raise SystemExit(main())
