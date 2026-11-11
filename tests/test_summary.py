"""票 12b：结果总表只聚合实跑数字，冻结状态按 ADR 0016 的 gates 判定。"""

import json
from pathlib import Path

import pytest

from yopo_l0.scene import load_card
from yopo_l0.summary import SCHEMA, dedupe_rows, main, summarize

from conftest import ROOT


def _row(card, *, policy, seed, arrived, n_drones=None, **extra):
    task = card.card["task"]["type"]
    n = n_drones if n_drones is not None else len(card.card["drones"])
    row = {
        "commit": "f5985e4335fcc00a4e19c861dcd1a05e0113f797",
        "scene": card.name,
        "scene_card_hash": card.hash,
        "seed": seed,
        "tier": "L0",
        "policy": policy if task not in ("crossing", "formation", "switch") else task,
        "shadow": True,
        "task": task,
        "outcome": "SUCCESS" if arrived == n else "TIMEOUT",
        "n_drones": n,
        "arrived": arrived,
        "arrival_rate": arrived / n if n else None,
        "arrival_time_s": 4.27 if arrived == n and task == "goto" else (20.0 if arrived == n else None),
        "min_obstacle_distance_m": 0.83,
        "min_inter_drone_distance_m": None if n == 1 else 1.987,
        "formation_error_p95_m": None if n == 1 else 0.027,
        "wall_time_s": 5.0,
        "form": "inprocess",
        "fault_tier": None,
        "forest_density": (card.card.get("forest") or {}).get("density"),
    }
    row.update(extra)
    return row


def _write_results(path: Path, rows: list[dict]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n")
    return path


def test_summary_uses_only_input_numbers_and_marks_frozen_gates(tmp_path):
    scenes = ROOT / "scenes"
    pillar = load_card(scenes / "g2-pillar.json")
    hold = load_card(scenes / "formation-hold.json")
    forest6 = load_card(scenes / "crossing-6-forest.json")
    rows = [
        _row(pillar, policy="upstream", seed=0, arrived=1, arrival_time_s=4.26, min_obstacle_distance_m=0.829),
        _row(pillar, policy="upstream", seed=1, arrived=1, arrival_time_s=4.28, min_obstacle_distance_m=0.828),
        _row(pillar, policy="certified", seed=0, arrived=0, arrival_time_s=None, outcome="TIMEOUT", min_obstacle_distance_m=0.90),
        _row(pillar, policy="certified", seed=1, arrived=0, arrival_time_s=None, outcome="TIMEOUT", min_obstacle_distance_m=0.91),
        _row(hold, policy="formation", seed=0, arrived=3, n_drones=3, formation_error_p95_m=0.026982, min_inter_drone_distance_m=1.9865),
        _row(
            forest6,
            policy="crossing",
            seed=0,
            arrived=6,
            n_drones=6,
            formation_error_p95_m=23.472,
            min_inter_drone_distance_m=1.293,
            crossing={
                "body_radius_m": 0.35,
                "obstacle_collisions": 0,
                "inter_drone_collisions": 0,
                "min_obstacle_clearance_m": 0.306,
            },
        ),
        _row(
            forest6,
            policy="crossing",
            seed=1,
            arrived=6,
            n_drones=6,
            formation_error_p95_m=26.298,
            min_inter_drone_distance_m=1.111,
            crossing={
                "body_radius_m": 0.35,
                "obstacle_collisions": 0,
                "inter_drone_collisions": 0,
                "min_obstacle_clearance_m": 0.126,
            },
        ),
    ]
    src = _write_results(tmp_path / "batch" / "results.json", rows)
    md_path = tmp_path / "SUMMARY.md"
    json_path = tmp_path / "SUMMARY.json"
    summary = summarize([src], scenes_dir=scenes, md_path=md_path, json_path=json_path)

    assert summary["schema"] == SCHEMA
    assert summary["commit"] == "f5985e4335fcc00a4e19c861dcd1a05e0113f797"
    by_key = {
        (r["scene"], r["policy"], r.get("fault_tier"), r["form"]): r
        for r in summary["rows"]
    }
    upstream = by_key[("g2-pillar", "upstream", None, "inprocess")]
    assert upstream["arrivals"] == "2/2"
    assert upstream["collisions"] == 0
    assert upstream["frozen"] is True
    assert upstream["metric_status"]["arrivals"] == "frozen"
    assert upstream["metric_status"]["collisions"] == "frozen"
    assert upstream["metric_status"]["arrival_time_s"] == "frozen"
    assert upstream["status"] == "判据已冻结（最小净空只报数）"
    assert upstream["arrival_time_s"]["min"] == pytest.approx(4.26)
    assert upstream["arrival_time_s"]["max"] == pytest.approx(4.28)

    certified = by_key[("g2-pillar", "certified", None, "inprocess")]
    assert certified["arrivals"] == "0/2"
    assert certified["collisions"] == 0
    assert certified["frozen"] is False
    assert certified["status"] == "未冻结 / report-only"
    assert certified["gates"] is None

    hold_row = by_key[("formation-hold", "formation", None, "inprocess")]
    assert hold_row["arrivals"] == "3/3"
    assert hold_row["formation_error_p95_m"]["min"] == pytest.approx(0.026982)
    assert hold_row["min_inter_drone_distance_m"] == pytest.approx(1.9865)
    assert hold_row["frozen"] is True
    assert hold_row["metric_status"]["formation_error_p95_m"] == "frozen"
    assert hold_row["clearance_is_ground"] is True

    six = by_key[("crossing-6-forest", "crossing", None, "inprocess")]
    assert six["arrivals"] == "12/12"
    assert six["collisions"] == 0
    assert six["min_inter_drone_distance_m"] == pytest.approx(1.111)
    assert six["min_obstacle_clearance_m"] == pytest.approx(0.126)
    assert six["formation_error_p95_m"]["min"] == pytest.approx(23.472)
    assert six["formation_error_p95_m"]["max"] == pytest.approx(26.298)
    assert six["frozen"] is True
    assert six["metric_status"]["arrivals"] == "frozen"
    assert six["metric_status"]["collisions"] == "frozen"
    assert six["metric_status"]["min_inter_drone_distance_m"] == "frozen"
    assert six["metric_status"]["formation_error_p95_m"] == "report-only"
    assert six["metric_status"]["min_obstacle_clearance_m"] == "report-only"
    assert six["metric_status"]["arrival_time_s"] == "report-only"
    assert six["status"].startswith("判据已冻结（")
    assert "队形误差 p95" in six["status"]
    assert "最小净空" in six["status"]

    # 没有跑过的行只说明、不填数；表里也不能出现输入里没有的 60/60。
    notes = " ".join(item["note"] for item in summary["unmeasured"])
    details = " ".join(item.get("detail") or "" for item in summary["unmeasured"])
    assert "sparse" in details and "dense" in details
    assert any(item["detail"] == "L1 抽检" for item in summary["unmeasured"])
    assert any("ROS 2" in (item.get("detail") or "") for item in summary["unmeasured"])
    assert any(item["scene"] == "六机柱林" for item in summary["unmeasured"])
    md = md_path.read_text()
    payload = json.loads(json_path.read_text())
    assert {r["arrivals"] for r in payload["rows"]} >= {"2/2", "0/2", "3/3", "12/12"}
    assert "60/60" not in md
    assert "2/2" in md
    assert "0/2" in md
    assert "12/12" in md
    assert "未冻结 / report-only" in md
    assert "本批未覆盖（不填数）" in md
    assert "里程碑" in md and "场景卡哈希" in md and "队形误差 p95" in md
    assert "最小机间距离" in md and "最小净空" in md
    # 脚本不能给未跑的 ROS 2 行填上进程内的 1.000。
    assert "ROS 2 全矩阵" in md
    assert "| M4 | formation-6-open（ROS 2 全矩阵） |" in md or "ROS 2 全矩阵" in md


def test_conflicting_duplicates_are_rejected(tmp_path):
    card = load_card(ROOT / "scenes/g2-pillar.json")
    a = _row(card, policy="upstream", seed=0, arrived=1)
    b = _row(card, policy="upstream", seed=0, arrived=0, outcome="TIMEOUT", arrival_time_s=None)
    with pytest.raises(ValueError, match="同一身份出现两份不同结果"):
        dedupe_rows([a, b])


def test_cli_writes_markdown_and_json(tmp_path):
    card = load_card(ROOT / "scenes/formation-hold.json")
    src = _write_results(tmp_path / "results.json", [_row(card, policy="formation", seed=0, arrived=3, n_drones=3)])
    md_path = tmp_path / "out" / "SUMMARY.md"
    json_path = tmp_path / "out" / "SUMMARY.json"
    assert main([
        str(src),
        "--scenes-dir", str(ROOT / "scenes"),
        "--md", str(md_path),
        "--json", str(json_path),
    ]) == 0
    md = md_path.read_text()
    data = json.loads(json_path.read_text())
    assert data["rows"][0]["scene"] == "formation-hold"
    assert data["rows"][0]["arrivals"] == "3/3"
    assert "| M1 |" in md
    assert "`f5985e4335fc`" in md


def test_collects_seed_results_next_to_other_batch_summaries(tmp_path):
    """同一棵目录里，有 results.json 的批跑不要把旁边只有 seed 文件的批跑丢掉。"""
    from yopo_l0.summary import collect_result_files

    hold = load_card(ROOT / "scenes/formation-hold.json")
    forest = load_card(ROOT / "scenes/crossing-6-forest.json")
    root = tmp_path / "m5"
    _write_results(
        root / "core" / "results.json",
        [_row(hold, policy="formation", seed=0, arrived=3, n_drones=3)],
    )
    seed_row = _row(
        forest,
        policy="crossing",
        seed=0,
        arrived=6,
        n_drones=6,
        formation_error_p95_m=23.0,
        min_inter_drone_distance_m=1.2,
        crossing={
            "body_radius_m": 0.35,
            "obstacle_collisions": 0,
            "inter_drone_collisions": 0,
            "min_obstacle_clearance_m": 0.2,
        },
    )
    seed_path = root / "m4-forest" / "crossing-6-forest@crossing" / "seed0" / "result.json"
    seed_path.parent.mkdir(parents=True, exist_ok=True)
    seed_path.write_text(json.dumps(seed_row, indent=2, ensure_ascii=False) + "\n")

    files = collect_result_files([root])
    assert {p.name for p in files} == {"results.json", "result.json"}
    summary = summarize(
        [root],
        scenes_dir=ROOT / "scenes",
        md_path=tmp_path / "S.md",
        json_path=tmp_path / "S.json",
    )
    scenes = {row["scene"] for row in summary["rows"]}
    assert scenes == {"formation-hold", "crossing-6-forest"}
    # 没有 certified 全程时，只在未覆盖节说明，不填数。
    assert any("certified" in (item.get("detail") or "") for item in summary["unmeasured"])
    assert "certified" not in tmp_path.joinpath("S.md").read_text().split("## 总表")[1].split("## 本批未覆盖")[0]
