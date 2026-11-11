"""冻结判据：场景卡 criteria.gates 里的指标失败时，批跑退出非 0。

criteria.frozen 为 false，或本跑不在适用范围内时，只报数，不改退出码。
适用范围可选：gate_policies、gate_forms、gate_densities。
`--duration` 短于场景卡 task.duration_s 的前缀跑也不套这些门（短跑到不了终点）。
墙钟和单次规划墙钟从不进 gates。未写入 gates 的指标（例如六机林带的 p95、净空）继续只报数。
"""

from __future__ import annotations

# 方向：lower = 不得低于门槛；upper = 不得高于门槛。
_GATE_KIND = {
    "arrival_rate": "lower",
    "collisions": "upper",
    "transition_collisions": "upper",
    "arrival_time_s": "upper",
    "formation_error_p95_m": "upper",
    "error_p95_before_m": "upper",
    "error_p95_after_m": "upper",
    "min_inter_drone_distance_m": "lower",
    "min_inter_drone_transition_m": "lower",
    "min_obstacle_clearance_m": "lower",
}

_EPS = 1e-9


def _card_dict(card) -> dict:
    if hasattr(card, "card"):
        return card.card
    return card


def _task_duration_s(card) -> float:
    raw = _card_dict(card).get("task") or {}
    try:
        return float(raw.get("duration_s") or 0.0)
    except (TypeError, ValueError):
        return 0.0


def active_gates(
    card,
    *,
    policy: str | None = None,
    form: str = "inprocess",
    density: str | None = None,
    duration_s: float | None = None,
) -> dict[str, float] | None:
    """本跑要执行的门槛。不适用则返回 None（只报数）。"""
    crit = _card_dict(card).get("criteria") or {}
    if not bool(crit.get("frozen", False)):
        return None
    gates = crit.get("gates")
    if not gates:
        return None
    if duration_s is not None:
        task_dur = _task_duration_s(card)
        if task_dur > 0.0 and float(duration_s) < task_dur:
            return None
    allowed = crit.get("gate_policies")
    if allowed is not None and policy not in allowed:
        return None
    forms = crit.get("gate_forms")
    if forms is not None and form not in forms:
        return None
    densities = crit.get("gate_densities")
    if densities is not None:
        dens = density
        if dens is None:
            dens = (_card_dict(card).get("forest") or {}).get("density")
        if dens not in densities:
            return None
    unknown = [name for name in gates if name not in _GATE_KIND]
    if unknown:
        raise ValueError("未知冻结门槛：" + ", ".join(unknown))
    return {str(name): float(value) for name, value in gates.items()}


def frozen_for_card(card, **kwargs) -> bool:
    return active_gates(card, **kwargs) is not None


def arrival_rate(row: dict) -> float | None:
    if row.get("arrival_rate") is not None:
        return float(row["arrival_rate"])
    n = row.get("n_drones") or 0
    if not n or row.get("arrived") is None:
        return None
    return float(row["arrived"]) / float(n)


def collision_count(row: dict) -> int:
    block = row.get("crossing") or {}
    if "obstacle_collisions" in block or "inter_drone_collisions" in block:
        return int(block.get("obstacle_collisions") or 0) + int(block.get("inter_drone_collisions") or 0)
    if row.get("outcome") == "COLLISION":
        return 1
    return 0


def transition_collision_count(row: dict) -> int:
    return int((row.get("switch") or {}).get("transition_collisions") or 0)


def _clearance(row: dict) -> float | None:
    block = row.get("crossing") or {}
    if block.get("min_obstacle_clearance_m") is not None:
        return float(block["min_obstacle_clearance_m"])
    dist = row.get("min_obstacle_distance_m")
    if dist is None:
        return None
    radius = float(block.get("body_radius_m") or (row.get("body_radius_m") or 0.35))
    return float(dist) - radius


def metric_value(row: dict, name: str) -> float | None:
    if name == "arrival_rate":
        return arrival_rate(row)
    if name == "collisions":
        return float(collision_count(row))
    if name == "transition_collisions":
        return float(transition_collision_count(row))
    if name == "arrival_time_s":
        raw = row.get("arrival_time_s")
        return None if raw is None else float(raw)
    if name == "formation_error_p95_m":
        raw = row.get("formation_error_p95_m")
        return None if raw is None else float(raw)
    if name in ("error_p95_before_m", "error_p95_after_m", "min_inter_drone_transition_m"):
        raw = (row.get("switch") or {}).get(name)
        return None if raw is None else float(raw)
    if name == "min_inter_drone_distance_m":
        raw = row.get("min_inter_drone_distance_m")
        return None if raw is None else float(raw)
    if name == "min_obstacle_clearance_m":
        return _clearance(row)
    raise ValueError(f"未知冻结门槛：{name}")


def _holds(kind: str, value: float, threshold: float) -> bool:
    if kind == "lower":
        return value + _EPS >= threshold
    return value - _EPS <= threshold


def _fmt(value: float, name: str) -> str:
    if name in {"collisions", "transition_collisions"}:
        return f"{int(round(value))}"
    if name == "arrival_rate":
        return f"{value:.3f}"
    return f"{value:.3f}"


def _compare_phrase(kind: str, value: float, threshold: float, name: str) -> str:
    op = "<" if kind == "lower" else ">"
    return f"{name} {_fmt(value, name)} {op} {_fmt(threshold, name)}"


def evaluate_gates(rows: list[dict], gates: dict[str, float], *, label: str = "") -> list[str]:
    """检查一组结果。返回失败说明；空列表表示通过。"""
    if not gates or not rows:
        return []
    failures: list[str] = []
    prefix = f"{label}：" if label else ""
    for row in rows:
        seed = row.get("seed")
        seed_tag = f"seed {seed}" if seed is not None else "seed ?"
        for name, threshold in gates.items():
            kind = _GATE_KIND[name]
            value = metric_value(row, name)
            if value is None or not _holds(kind, value, threshold):
                shown = "缺失" if value is None else _compare_phrase(kind, value, threshold, name)
                failures.append(f"{prefix}{seed_tag} {shown}")
    # 整组到达率和碰撞再核一次，避免「每 seed 凑巧过、合计不够」被漏掉。
    if "arrival_rate" in gates:
        rates = [arrival_rate(row) for row in rows]
        if any(r is None for r in rates) or not _holds(
            "lower", sum(r for r in rates if r is not None) / len(rows), gates["arrival_rate"]
        ):
            arrived = sum(int(row.get("arrived") or 0) for row in rows)
            slots = sum(int(row.get("n_drones") or 0) for row in rows)
            overall = None if not slots else arrived / slots
            if overall is None or not _holds("lower", overall, gates["arrival_rate"]):
                msg = f"{prefix}合计到达率 " + (
                    "缺失" if overall is None else _compare_phrase("lower", overall, gates["arrival_rate"], "arrival_rate")
                )
                if msg not in failures:
                    failures.append(msg)
    if "collisions" in gates:
        total = sum(collision_count(row) for row in rows)
        if not _holds("upper", float(total), gates["collisions"]):
            msg = f"{prefix}合计 {_compare_phrase('upper', float(total), gates['collisions'], 'collisions')}"
            if msg not in failures:
                failures.append(msg)
    return failures


def evaluate_card_rows(
    card,
    rows: list[dict],
    *,
    policy: str | None = None,
    form: str = "inprocess",
    duration_s: float | None = None,
    label: str = "",
) -> tuple[dict[str, float] | None, list[str]]:
    """按行判断门槛是否适用（密度可能不同），返回 (所用门槛或 None, 失败说明)。"""
    failures: list[str] = []
    applied: dict[str, float] | None = None
    grouped: dict[tuple, list[dict]] = {}
    for row in rows:
        density = row.get("forest_density")
        row_policy = row.get("policy") or policy
        row_form = row.get("form") or form
        gates = active_gates(
            card, policy=row_policy, form=row_form, density=density, duration_s=duration_s,
        )
        if not gates:
            continue
        key = tuple(sorted(gates.items()))
        grouped.setdefault(key, []).append(row)
        applied = gates
    for key, group in grouped.items():
        gates = dict(key)
        failures.extend(evaluate_gates(group, gates, label=label))
        applied = gates
    return applied, failures
