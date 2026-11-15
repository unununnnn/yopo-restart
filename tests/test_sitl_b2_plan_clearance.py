"""票 B2：交叉卡规划净空归因。不起 Gazebo，也不连 ROS。"""

import json
import os
import subprocess
import sys

import numpy as np
import pytest

from conftest import ROOT
from yopo_l0 import crossing
from yopo_l0.b2_plan_diag import PlanDiag, _split, apply_b2_overrides
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card

CROSS = ROOT / "scenes/crossing-pillars.json"
CROSS_SITL = ROOT / "scenes/crossing-pillars-sitl80.json"


def _run_l0(seconds, *, diag):
    card = load_card(CROSS)
    holder = {}
    original = crossing.CrossingFleet.from_card.__func__

    def from_card(cls, card_dict, physics_dt, *, record_inputs=False):
        fleet = original(cls, card_dict, physics_dt, record_inputs=record_inputs)
        if diag:
            holder["d"] = PlanDiag(fleet, card.min_distance, 0.35)
        return fleet

    crossing.CrossingFleet.from_card = classmethod(from_card)
    try:
        out = run(card, RunConfig(seed=0, shadow=False, duration_s=seconds))
    finally:
        crossing.CrossingFleet.from_card = classmethod(original)
    return out, holder.get("d")


def test_recorder_does_not_change_the_flight():
    """装上记录器，L0 每拍位置逐位相同；记录器确实记到了提案和每拍。"""
    plain, _ = _run_l0(2.5, diag=False)
    seen, d = _run_l0(2.5, diag=True)
    assert [r["p"] for r in plain.ticks] == [r["p"] for r in seen.ticks]
    s = d.summary()
    assert s["planner_calls"] > 0 and s["proposals"] > 0
    assert s["per_drone"]["uav0"]["ticks"] > 0
    assert s["plan_clear_min_m"] is not None and s["plan_clear_min_m"] > 0.0
    # L0 深度帧和规划同一拍，年龄不超过一个 30 Hz 周期。
    assert s["at_plan_frame_age_s"]["max"] <= 1.0 / 30.0 + 1e-9


def test_split_along_and_lateral():
    along, lat = _split(np.array([1.0, 2.0, 0.0]), np.array([1.0, 0.0]))
    assert along == pytest.approx(1.0) and lat == pytest.approx(2.0)
    along, lat = _split(np.array([0.0, -0.5, 0.0]), np.array([0.0, 0.0]))
    assert along == 0.0 and lat == pytest.approx(0.5)


def test_overrides_default_off_keep_card_hash():
    card = load_card(CROSS_SITL)
    same, dev = apply_b2_overrides(card)
    assert same is card and dev == []
    wide, dev = apply_b2_overrides(card, safe_radius=0.35)
    assert wide.card["policy_params"]["upstream"]["safe_radius"] == 0.35
    assert dev and dev[0].startswith("偏离上游")
    assert card.hash == load_card(CROSS_SITL).hash  # 原卡没被改
    with pytest.raises(ValueError):
        apply_b2_overrides(card, safe_radius=0.0)


def _live(*argv, env=None):
    e = {k: v for k, v in os.environ.items() if not k.startswith(("YOPO_B2_", "YOPO_SITL_", "YOPO_AGENT_"))}
    e.update(env or {})
    e["PYTHONPATH"] = f"{ROOT}:{ROOT / 'core/src'}"
    return subprocess.run(
        [sys.executable, str(ROOT / "scripts/sitl/yopo_l0_live.py"), *argv],
        cwd=ROOT, env=e, capture_output=True, text=True, check=False,
    )


def test_dry_run_b2_switches_default_off(tmp_path):
    out = tmp_path / "p.json"
    r = _live("--scene", str(CROSS_SITL), "--policy", "crossing", "--dry-run", "--out", str(out))
    assert r.returncode == 0, r.stderr
    assert json.loads(out.read_text())["b2"] == {"plan_diag": False, "deviations": []}
    r = _live("--scene", str(CROSS_SITL), "--policy", "crossing", "--dry-run", "--out", str(out),
              env={"YOPO_B2_PLAN_DIAG": "1", "YOPO_B2_SAFE_RADIUS": "0.35"})
    assert r.returncode == 0, r.stderr
    b2 = json.loads(out.read_text())["b2"]
    assert b2["plan_diag"] is True and b2["deviations"] == ["偏离上游：safe_radius=0.35"]
    bad = _live("--scene", str(CROSS_SITL), "--policy", "crossing", "--dry-run", "--out", str(out), "--safe-radius", "-1")
    assert bad.returncode == 2


def test_shell_entry_passes_b2_env_through():
    text = (ROOT / "scripts/sitl/run_yopo_l0_sitl.sh").read_text()
    # sudo docker 不继承调用者环境：必须显式带值。
    assert text.count('-e YOPO_B2_PLAN_DIAG="${YOPO_B2_PLAN_DIAG:-}"') == 2
    assert text.count('-e YOPO_B2_FIX="${YOPO_B2_FIX:-}"') == 2


def test_b2_does_not_touch_px4_parameters():
    for rel in ("yopo_l0/b2_plan_diag.py", "scripts/b2/l0_plan_diag.py"):
        text = (ROOT / rel).read_text()
        assert "param set" not in text and "PX4_PARAM_" not in text
        for prefix in ("COM_", "NAV_", "BAT_"):
            assert f'"{prefix}' not in text and f"'{prefix}" not in text
