"""票 D6b：合成的 1.0 m / 1.5 m 教师记录走通扫描、转换、混合训练和双高度评测。"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

ROOT = Path(__file__).resolve().parents[1]
PIPELINE = ROOT / "scripts" / "distill" / "d6" / "pipeline.py"
EVAL = ROOT / "scripts" / "distill" / "d6" / "eval_altitudes.py"
PY = sys.executable

UNTOUCHED = (
    "yopo_l0/policy.py",
    "yopo_l0/sensor.py",
    "yopo_l0/distill.py",
    "yopo_l0/student.py",
    "yopo_l0/student_fit.py",
    "scripts/distill/d6/feasibility.py",
    "scripts/regress.sh",
    "docs/tickets.md",
    "README.md",
    "scenes/open-3drone.json",
    "scenes/swap-3-open.json",
    "scenes/g2-pillar.json",
    "scenes/ego-density-014-solo.json",
    "scenes/ego-density-014-swap3.json",
)


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_run(
    directory: Path,
    *,
    flight_z: float = 1.0,
    success: bool = True,
    clearance: float | None = 0.8,
    schema: str = "yopo-teacher-record/1",
    with_trajectory: bool = True,
    write_flight_z: bool = True,
    duration_s: float = 2.0,
) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    times = np.arange(0.0, duration_s + 1e-9, 0.1, dtype=np.float64)
    position = np.zeros((times.size, 1, 3), dtype=np.float64)
    position[:, 0, 0] = times * 2.0
    position[:, 0, 2] = flight_z
    velocity = np.zeros_like(position)
    velocity[:, 0, 0] = 2.0
    goal = np.array([[20.0, 0.0, flight_z]], dtype=np.float64)
    if with_trajectory:
        np.savez_compressed(
            directory / "trajectory.npz",
            time_s=times,
            position_m=position,
            velocity_mps=velocity,
            acceleration_mps2=np.zeros_like(position),
            yaw_rad=np.zeros((times.size, 1), dtype=np.float64),
            goal_m=goal,
        )
    scenario = {"id": directory.name, "n_drones": 1}
    if write_flight_z:
        scenario["flight_z_m"] = flight_z
    metrics = {"success": success}
    if clearance is not None:
        metrics["min_obstacle_clearance_m"] = clearance
    record = {
        "schema": schema,
        "scenario": scenario,
        "metrics": metrics,
        "run": {"status": "arrived" if success else "stopped"},
    }
    (directory / "record.json").write_text(json.dumps(record), encoding="utf-8")
    (directory / "geometry.json").write_text(json.dumps({"boxes": [], "cylinders": []}), encoding="utf-8")
    scene = {
        "name": directory.name,
        "geometry": "geometry.json",
        "ground_z": 0.0,
        "drones": [{"id": "uav0", "start_enu": [0.0, 0.0, flight_z], "goal_enu": [20.0, 0.0, flight_z]}],
        "task": {"type": "goto", "duration_s": 5.0},
        "criteria": {"frozen": False, "body_radius_m": 0.2},
        "camera": {
            "width": 160,
            "height": 96,
            "hfov_deg": 90.0,
            "vfov_deg": 60.0,
            "min_range_m": 0.2,
            "max_range_m": 20.0,
            "noise_std_m": 0.0,
            "mount_xyz_body": [0.0, 0.0, 0.0],
        },
        "dynamics": {"model": "point_mass_lag", "tau_s": 0.1, "kp": 4.0, "kd": 3.0, "a_max": 6.0},
        "rates_hz": {"physics": 100, "camera": 30, "planner": 10},
    }
    (directory / "scene.json").write_text(json.dumps(scene), encoding="utf-8")


def _build_corpus(tmp_path: Path) -> tuple[Path, Path]:
    low = tmp_path / "d2b" / "runs"
    high = tmp_path / "d6-z150" / "runs"
    # 1.0 m。s0 / s5 的种子进留出。s3 不写 flight_z_m，应落到默认 1.0。
    _write_run(low / "d000-n1-parallel-s0", flight_z=1.0, clearance=0.8)
    _write_run(low / "d000-n1-parallel-s1", flight_z=1.0, clearance=0.8)
    _write_run(low / "d000-n1-parallel-s2", flight_z=1.0, clearance=0.4)
    _write_run(low / "d000-n1-parallel-s3", flight_z=1.0, clearance=0.5, write_flight_z=False)
    _write_run(low / "d000-n1-parallel-s4", flight_z=1.0, success=False, clearance=-0.1)
    _write_run(low / "d000-n1-parallel-s6", flight_z=1.0, clearance=0.8, with_trajectory=False)
    _write_run(low / "d000-n1-parallel-s7", flight_z=1.0, clearance=-0.02)
    _write_run(low / "d000-n1-parallel-s8", flight_z=1.0, schema="not-a-teacher-record")
    # 1.5 m。编号带 z150。
    _write_run(high / "d000-n1-parallel-z150-s1", flight_z=1.5, clearance=0.9)
    _write_run(high / "d000-n1-parallel-z150-s2", flight_z=1.5, clearance=0.7)
    _write_run(high / "d000-n1-parallel-z150-s5", flight_z=1.5, clearance=0.7)
    _write_run(high / "d014-n1-headon-z150-s6", flight_z=1.5, clearance=0.3)
    return low, high


def _run(args: list[str]) -> None:
    proc = subprocess.run([PY, str(PIPELINE), *args], cwd=ROOT, text=True, capture_output=True)
    if proc.returncode != 0:
        raise AssertionError(f"exit {proc.returncode}\n{proc.stdout}\n{proc.stderr}")


def test_clearance_zero_keeps_success_and_a_positive_threshold_filters(tmp_path):
    pipe = _load("d6b_pipeline_filter", PIPELINE)
    assert pipe.parse_min_clearance("0") is None
    assert pipe.parse_min_clearance("off") is None
    assert pipe.parse_min_clearance("0.2") == pytest.approx(0.2)
    height, source, mismatch = pipe.resolve_flight_z({"scenario": {"flight_z_m": 1.5}}, "d000-n1-parallel-s1")
    assert (height, source, mismatch) == (1.5, "record", False)
    height, source, mismatch = pipe.resolve_flight_z({"scenario": {}}, "d000-n1-parallel-z150-s1")
    assert (height, source, mismatch) == (1.5, "run_id", False)
    height, source, mismatch = pipe.resolve_flight_z({"scenario": {}}, "d000-n1-parallel-s1")
    assert (height, source, mismatch) == (1.0, "default_1.0", False)
    height, _source, mismatch = pipe.resolve_flight_z(
        {"scenario": {"flight_z_m": 1.0}}, "d014-n1-headon-z150-s0"
    )
    assert height == 1.0 and mismatch is True

    root = tmp_path / "runs"
    _write_run(root / "d000-n1-parallel-s1", clearance=-0.05)
    _write_run(root / "d000-n1-parallel-s2", clearance=0.4)
    off = tmp_path / "off"
    gated = tmp_path / "gated"
    _run([
        "--only-convert", "--teacher-dirs", str(root), "--out", str(off),
        "--jobs", "1", "--min-clearance-m", "0", "--max-windows", "1",
    ])
    _run([
        "--only-convert", "--teacher-dirs", str(root), "--out", str(gated),
        "--jobs", "1", "--min-clearance-m", "0.2", "--max-windows", "1",
    ])
    loose = json.loads((off / "convert-manifest.json").read_text(encoding="utf-8"))
    strict = json.loads((gated / "convert-manifest.json").read_text(encoding="utf-8"))
    assert loose["clearance_filter"] is False
    assert loose["n_filtered"] == 0
    assert loose["n_converted"] == 2
    assert strict["clearance_filter"] is True
    assert strict["n_filtered"] == 1
    assert strict["n_converted"] == 1
    assert strict["min_clearance_m"] == pytest.approx(0.2)
    assert list(strict["files"]) == ["d000-n1-parallel-s2"]


def test_balance_samples_equal_runs_per_altitude():
    pipe = _load("d6b_pipeline_balance", PIPELINE)
    files = {
        "a": {"flight_z_m": 1.0, "n": 4},
        "b": {"flight_z_m": 1.0, "n": 4},
        "c": {"flight_z_m": 1.0, "n": 4},
        "d": {"flight_z_m": 1.5, "n": 2},
    }
    kept, info = pipe.balance_train_ids(["a", "b", "c", "d"], files, seed=0)
    assert info["applied"] is True
    assert info["runs_per_altitude"] == 1
    assert sum(1 for run_id in kept if files[run_id]["flight_z_m"] == 1.0) == 1
    assert "d" in kept
    assert kept == sorted(kept)
    assert pipe.resolve_mode("all", False, False) == "all"
    assert pipe.resolve_mode(None, True, False) == "convert"
    with pytest.raises(SystemExit):
        pipe.resolve_mode(None, True, True)


def test_end_to_end_convert_train_and_dual_altitude_smoke(tmp_path):
    from yopo_l0.distill import load_batch
    from yopo_l0.policy import StudentPolicy
    from yopo_l0.student import (
        DEPTH_HEIGHT,
        DEPTH_WIDTH,
        HORIZON_STEPS,
        NEIGHBOR_FEAT_DIM,
        NEIGHBOR_K,
        STATE_DIM,
        load_checkpoint,
    )

    untouched = {rel: _sha(ROOT / rel) for rel in UNTOUCHED}
    defaults = dict(StudentPolicy.DEFAULTS)
    low, high = _build_corpus(tmp_path)
    out = tmp_path / "d6b"
    _run([
        "all",
        "--teacher-dirs", str(low), str(high),
        "--out", str(out),
        "--jobs", "2",
        "--min-clearance-m", "0",
        "--max-windows", "2",
        "--balance", "altitude",
        "--epochs", "1",
        "--threads", "1",
    ])
    manifest = json.loads((out / "convert-manifest.json").read_text(encoding="utf-8"))
    assert manifest["schema"] == "yopo-student-d6b-convert/1"
    assert manifest["clearance_filter"] is False
    assert manifest["n_records"] == 12
    assert manifest["n_success"] == 10
    assert manifest["n_filtered"] == 0
    assert manifest["n_converted"] == 9
    assert manifest["n_windows"] == 18
    assert manifest["n_convert_failed"] == 1
    assert manifest["n_bad_schema"] == 1
    assert manifest["n_not_success"] == 1
    assert manifest["n_eligible"] == 10
    assert manifest["failures"][0]["id"] == "d000-n1-parallel-s6"
    assert manifest["by_altitude"]["1.0"]["n_success"] == 6
    assert manifest["by_altitude"]["1.0"]["n_converted"] == 5
    assert manifest["by_altitude"]["1.0"]["n_windows"] == 10
    assert manifest["by_altitude"]["1.5"]["n_success"] == 4
    assert manifest["by_altitude"]["1.5"]["n_converted"] == 4
    assert manifest["by_altitude"]["1.5"]["n_windows"] == 8
    assert manifest["by_scene"]["d000-n1-parallel"]["1.0"]["n_records"] == 8
    assert manifest["by_scene"]["d014-n1-headon"]["1.5"]["n_converted"] == 1
    assert manifest["files"]["d000-n1-parallel-s3"]["flight_z_source"] == "default_1.0"
    assert manifest["files"]["d000-n1-parallel-s3"]["flight_z_m"] == pytest.approx(1.0)
    assert manifest["files"]["d014-n1-headon-z150-s6"]["flight_z_m"] == pytest.approx(1.5)
    assert "d000-n1-parallel-s0" in manifest["hold_ids"]
    assert "d000-n1-parallel-z150-s5" in manifest["hold_ids"]
    assert "d000-n1-parallel-s1" in manifest["train_ids"]
    assert manifest["n_converted"] + manifest["n_convert_failed"] == manifest["n_eligible"]
    sha_before = manifest["files"]["d000-n1-parallel-s1"]["sha256"]

    assert len(list((out / "windows").glob("*.npz"))) == 9
    for run_id, meta in manifest["files"].items():
        batch = load_batch(meta["path"])
        n = batch.n
        assert n == 2
        assert meta["n"] == n
        assert batch.depth.shape == (n, 2, DEPTH_HEIGHT, DEPTH_WIDTH)
        assert batch.state.shape == (n, STATE_DIM)
        assert batch.position_body.shape == (n, HORIZON_STEPS, 3)
        assert batch.velocity_body.shape == (n, HORIZON_STEPS, 3)
        assert batch.yaw_rel.shape == (n, HORIZON_STEPS)
        assert batch.neighbor_rel.shape == (n, NEIGHBOR_K, NEIGHBOR_FEAT_DIM)
        assert batch.neighbor_mask.shape == (n, NEIGHBOR_K)
        assert batch.time_s.shape == (n,)
        assert batch.sample_index.shape == (n,)
        assert batch.ego_index.shape == (n,)
        assert batch.depth.dtype == np.float32
        assert batch.state.dtype == np.float32
        assert run_id == meta["path"].rsplit("/", 1)[-1].removesuffix(".npz")

    _run([
        "--only-convert",
        "--teacher-dirs", str(low), str(high),
        "--out", str(out),
        "--jobs", "2",
        "--min-clearance-m", "0",
        "--max-windows", "2",
    ])
    resumed = json.loads((out / "convert-manifest.json").read_text(encoding="utf-8"))
    assert resumed["n_reused"] == resumed["n_converted"] == 9
    assert resumed["n_windows"] == 18
    assert resumed["files"]["d000-n1-parallel-s1"]["sha256"] == sha_before

    weights = out / "train" / "student.pt"
    _model, ckpt = load_checkpoint(weights, enabled=True)
    block = ckpt["d6b"]
    assert block["recipe_name"] == "d5b-r4"
    assert block["eval_params"] == {
        "knots": "position",
        "yaw_source": "ego_lookahead",
        "accel_source": "reference",
        "vertical": "hold_goal_z",
    }
    assert block["fit_used"]["planar"] is True
    assert block["fit_used"]["near_weight"] == pytest.approx(3.0)
    assert block["fit_used"]["epochs"] == 1
    assert block["recipe_fit"]["epochs"] == 5
    assert block["recipe_fit"]["planar"] is True
    assert block["recipe_fit"]["near_weight"] == pytest.approx(3.0)
    assert block["balance"]["applied"] is True
    assert block["balance"]["runs_per_altitude"] == 3
    assert block["altitude_distribution"]["train"]["1.0"]["runs"] == 3
    assert block["altitude_distribution"]["train"]["1.5"]["runs"] == 3
    assert block["altitude_distribution"]["train"]["1.0"]["windows"] == 6
    assert block["altitude_distribution"]["train"]["1.5"]["windows"] == 6
    assert block["altitude_distribution"]["hold"]["1.0"]["runs"] == 1
    assert block["altitude_distribution"]["hold"]["1.5"]["runs"] == 1
    assert block["manifest_summary"]["n_converted"] == 9
    assert block["manifest_summary"]["by_altitude"]["1.5"]["n_windows"] == 8
    train_note = json.loads((out / "train" / "d6b-train.json").read_text(encoding="utf-8"))
    assert train_note["weights_sha256"] == ckpt["weights_sha256"]
    assert sum(int(parameter.numel()) for parameter in _model._module().parameters()) > 0

    missing = subprocess.run(
        [PY, str(PIPELINE), "--only-convert", "--teacher-dirs", str(tmp_path / "missing"), "--out", str(tmp_path / "unused")],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )
    assert missing.returncode != 0

    eval_mod = _load("d6b_eval_smoke", EVAL)
    assert [item[0] for item in eval_mod.SCENES] == [
        "open-3drone",
        "swap-3-open",
        "g2-pillar",
        "ego-density-014-solo",
        "ego-density-014-swap3",
    ]
    from yopo_l0.scene import load_card

    original = load_card(ROOT / "scenes" / "open-3drone.json")
    retargeted = eval_mod.retarget_card(original, 1.0, "geom_multi", str(weights))
    assert original.card["drones"][0]["goal_enu"][2] == 1.5
    assert retargeted.card["drones"][0]["start_enu"][2] == 1.0
    assert retargeted.card["drones"][0]["goal_enu"][2] == 1.0
    assert retargeted.card["policy_params"]["student"]["vertical"] == "hold_goal_z"
    assert retargeted.card["policy_params"]["upstream"]["track_clearance"] is True
    assert retargeted.card["policy_params"]["upstream"]["track_clearance_mode"] == "geom_multi"
    off_card = eval_mod.retarget_card(original, 1.5, "off", str(weights))
    assert "track_clearance" not in (off_card.card["policy_params"].get("upstream") or {})

    assert eval_mod.main([
        "--weights", str(weights),
        "--out", str(out / "eval"),
        "--altitudes", "1.0", "1.5",
        "--seeds", "0",
        "--scenes", "open-3drone",
        "--track-clearance", "off", "geom_multi",
        "--duration-s", "0.15",
        "--jobs", "1",
        "--no-shadow",
    ]) == 0
    report = json.loads((out / "eval" / "eval-altitudes.json").read_text(encoding="utf-8"))
    markdown = (out / "eval" / "eval-altitudes.md").read_text(encoding="utf-8")
    assert report["report_only"] is True
    assert "report-only" in report["disclaimer"]
    assert "report-only" in markdown
    assert report["altitudes_m"] == [1.0, 1.5]
    assert report["track_clearance"] == ["off", "geom_multi"]
    assert report["library_defaults"]["student_knots"] == "network"
    assert report["library_defaults"]["student_vertical"] == "network"
    assert report["library_defaults"]["track_clearance"] is False
    assert report["library_defaults"]["changed"] is False
    assert report["student_params"]["knots"] == "position"
    assert report["student_params"]["yaw_source"] == "ego_lookahead"
    assert report["student_params"]["accel_source"] == "reference"
    assert report["student_params"]["vertical"] == "hold_goal_z"
    seen = {(row["altitude_m"], row["track_clearance"]) for row in report["rows"]}
    assert seen == {(1.0, "off"), (1.0, "geom_multi"), (1.5, "off"), (1.5, "geom_multi")}
    for row in report["rows"]:
        assert row["n_drones"] == 3
        assert row["report_only"] is True
        assert row["student_reads_track_clearance"] is False
        assert isinstance(row["min_clearance_m"], float)
        assert row["student_params"]["knots"] == "position"
        if row["track_clearance"] == "off":
            assert row["track_clearance_on_card"] is False
        else:
            assert row["track_clearance_on_card"] is True
            assert row["track_clearance_mode_on_card"] == "geom_multi"
    altitudes_in_table = {group["altitude_m"] for group in report["groups"]}
    assert altitudes_in_table == {1.0, 1.5}
    assert "| 开阔三机平行 |" in markdown
    assert dict(StudentPolicy.DEFAULTS) == defaults
    for rel, digest in untouched.items():
        assert _sha(ROOT / rel) == digest, rel
