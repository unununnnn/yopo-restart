"""票 07d：safe_radius 默认 0.05 m，与上游 d5e1e60 同一解码器；0.35 m 才少准入。"""

import math

import numpy as np
import pytest

from yopo_l0.batch import _radius_cells, format_safe_radius_comparison
from yopo_l0.batch import main as batch_main
from yopo_l0.crossing import CrossingFleet
from yopo_l0.dynamics import PointMassLag
from yopo_l0.policy import (
    UPSTREAM_SAFE_RADIUS_M,
    read_safe_radius,
    upstream_decoder,
)
from yopo_l0.runner import RunConfig, run
from yopo_l0.scene import load_card
from yopo_swarm_core.candidates import UPSTREAM_DECODER, select_action

PILLARS = "scenes/crossing-pillars.json"
GOTO = "scenes/g2-pillar.json"


def test_default_decoder_is_the_upstream_object():
    assert UPSTREAM_SAFE_RADIUS_M == pytest.approx(0.05)
    assert upstream_decoder(None) is UPSTREAM_DECODER
    assert upstream_decoder(0.05) is UPSTREAM_DECODER
    assert read_safe_radius({}) == pytest.approx(0.05)
    assert read_safe_radius({"upstream": {"safe_radius": 0.05}}) == pytest.approx(0.05)


def test_body_radius_threshold_rejects_a_candidate_upstream_would_fly():
    loose = upstream_decoder(0.05)
    tight = upstream_decoder(0.35)
    assert tight is not UPSTREAM_DECODER
    assert tight.safe_mu == pytest.approx(1.0 - math.exp(-0.35 / tight.radius_warp_lambda))
    assert tight.safe_mu > loose.safe_mu
    mid = 0.5 * (loose.safe_mu + tight.safe_mu)
    mu = np.array([mid, mid - 0.2])
    score = np.array([2.0, 0.1])
    action, brake_loose = select_action(score, None, mu, loose.safe_mu)
    _action, brake_tight = select_action(score, None, mu, tight.safe_mu)
    assert action == 0 and brake_loose is False
    assert brake_tight is True


def test_non_positive_radius_is_rejected():
    with pytest.raises(ValueError):
        upstream_decoder(0.0)
    with pytest.raises(ValueError):
        read_safe_radius({"upstream": {"safe_radius": -0.1}})


def test_scene_card_reaches_the_crossing_fleet_and_default_stays_upstream():
    card = load_card(PILLARS).card
    fleet = CrossingFleet.from_card(card, 0.01)
    assert fleet.decoder is UPSTREAM_DECODER
    assert fleet.safe_radius_m == pytest.approx(0.05)
    wide = {**card, "policy_params": {"upstream": {"safe_radius": 0.35}}}
    other = CrossingFleet.from_card(wide, 0.01)
    assert other.safe_radius_m == pytest.approx(0.35)
    assert other.decoder.safe_mu > fleet.decoder.safe_mu


def test_tracking_gains_stay_at_the_l0_defaults():
    model = PointMassLag()
    assert (model.kp, model.kd, model.tau_s, model.a_max) == (4.0, 3.0, 0.1, 6.0)


def test_explicit_upstream_radius_matches_the_omitted_default():
    card = load_card(GOTO)
    common = dict(seed=0, policy="upstream", duration_s=0.3, shadow=False)
    omitted = run(card, RunConfig(**common))
    explicit = run(card, RunConfig(**common, safe_radius_m=0.05))
    assert omitted.result["safe_radius_m"] == pytest.approx(0.05)
    assert omitted.result["upstream_deviation"] is False
    assert explicit.result["upstream_deviation"] is False
    assert omitted.ticks == explicit.ticks
    assert omitted.result["plan_codes"] == explicit.result["plan_codes"]


def test_override_is_marked_as_a_deviation_without_changing_the_card_hash():
    card = load_card(GOTO)
    out = run(card, RunConfig(
        seed=0, policy="upstream", duration_s=0.3, shadow=False, safe_radius_m=0.35,
    ))
    assert out.result["scene_card_hash"] == card.hash
    assert out.result["safe_radius_m"] == pytest.approx(0.35)
    assert out.result["upstream_deviation"] is True
    assert "empty_solution_count" in out.result


def test_batch_table_marks_deviation_and_keeps_l1_columns_last(tmp_path):
    assert _radius_cells({"safe_radius_m": 0.35, "upstream_deviation": True}) == "0.35 | 偏离上游"
    assert _radius_cells({"safe_radius_m": 0.05, "upstream_deviation": False}) == "0.05 | 否"
    assert _radius_cells({}) == "— | —"
    rc = batch_main([
        "scenes/narrow-corridor.json@hover",
        "--seeds", "0",
        "--duration", "0.2",
        "--no-determinism",
        "--no-shadow",
        "--out", str(tmp_path),
    ])
    assert rc == 0
    md = (tmp_path / "summary.md").read_text()
    assert "safe_radius m" in md and "偏离上游" in md
    row = next(line for line in md.splitlines() if line.startswith("| 0 |"))
    cells = [c.strip() for c in row.strip("|").split("|")]
    assert cells[-4:-2] == ["—", "—"]
    assert cells[-2:] == ["—", "—"]


def test_crossing_batch_without_an_explicit_policy_still_prints(tmp_path):
    rc = batch_main([
        "scenes/crossing-pillars.json",
        "--seeds", "0",
        "--duration", "0.3",
        "--no-determinism",
        "--no-shadow",
        "--safe-radius", "0.35",
        "--out", str(tmp_path),
    ])
    assert rc == 0
    md = (tmp_path / "summary.md").read_text()
    assert "crossing-pillars @ crossing" in md
    assert "0.35" in md and "偏离上游" in md
    row = next(line for line in md.splitlines() if line.startswith("| 0 |"))
    cells = [c.strip() for c in row.strip("|").split("|")]
    assert cells[-4:-2] == ["0.35", "偏离上游"]
    assert cells[-2:] == ["—", "—"]


def test_comparison_table_flags_non_upstream_radius():
    rows = [
        {
            "forest_density": "dense",
            "safe_radius_m": 0.05,
            "upstream_deviation": False,
            "arrival_time_s": None,
            "reason": "uav0:TRUTH_POSITION",
            "empty_solution_count": 2,
            "crossing": {"min_obstacle_clearance_m": -0.02},
        },
        {
            "forest_density": "dense",
            "safe_radius_m": 0.35,
            "upstream_deviation": True,
            "arrival_time_s": 12.5,
            "reason": "GOAL_REACHED_AND_HELD",
            "empty_solution_count": 4,
            "crossing": {"min_obstacle_clearance_m": 0.11},
        },
    ]
    text = format_safe_radius_comparison(rows)
    assert "偏离上游" in text
    assert "| dense | 0.05 | 否 | 0/1 | 1 | -0.020–-0.020 | 2 | — |" in text
    assert "| dense | 0.35 | 偏离上游 | 1/1 | 0 | 0.110–0.110 | 4 | 12.50（12.50–12.50） |" in text
