"""D6：把教师飞行高度扩到 1.0 m 和 1.5 m 时，密集格要补多少场。

不启动容器。成功率用 D2c 已经跑完的 1.0 m 网格
（docs/research/d2c-teacher-data.md 第 2 节）。把这个比例用到 1.5 m
是假设：树高 2.5 m，1.5 m 仍穿过树干，水平几何不变，但深度里的地面和树顶会变，
真实成功率未知。

净空合格 = metrics.success 为 true。D2 的成功线就是
min_obstacle_clearance_m >= 0（表面距离减去机体半径 0.20 m，
碰撞判断带 1e-9 m 的余量，见 scripts/teacher/metrics.py）。
密集格指密度 0.14 和 0.28。空场不重采样。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil


# D2c 续跑实测。543 个 skip 是瞬间跳过，不进分子。
D2C_EXECUTED_SCENES = 1249
D2C_RESUME_WALL_S = 13019.918
D2C_SCENE_WALL_SUM_S = 38802.379
D2C_SCENE_COUNT = 1792
# 规划配额：每一密集格、每一高度留下这么多条净空合格的轨迹。
DEFAULT_QUOTA = 128
# 单格尝试上限（含已经跑过的种子）。不设上限时 d028-n3-parallel 要几千场。
DEFAULT_CAP = 512


@dataclass(frozen=True)
class Cell:
    density: float
    n_drones: int
    pattern: str
    planned: int
    success: int
    fail: int
    unknown: int

    @property
    def dense(self) -> bool:
        return self.density > 0.0

    @property
    def concluded(self) -> int:
        return self.success + self.fail

    @property
    def p_clean(self) -> float:
        """有结论的场里净空合格的比例。未知不进分母。"""
        if self.concluded <= 0:
            raise ValueError(f"{self.key} 没有有结论的场")
        return self.success / self.concluded

    @property
    def key(self) -> str:
        tag = f"d{int(round(self.density * 100)):03d}"
        return f"{tag}-n{self.n_drones}-{self.pattern}"


# 与 d2c-teacher-data.md 第 2 节的表一致。
D2C_CELLS: tuple[Cell, ...] = (
    Cell(0.0, 1, "parallel", 128, 128, 0, 0),
    Cell(0.0, 1, "headon", 128, 128, 0, 0),
    Cell(0.0, 3, "parallel", 128, 128, 0, 0),
    Cell(0.0, 3, "headon", 128, 128, 0, 0),
    Cell(0.0, 6, "parallel", 128, 110, 7, 11),
    Cell(0.0, 6, "headon", 128, 103, 9, 16),
    Cell(0.14, 1, "parallel", 128, 105, 23, 0),
    Cell(0.14, 1, "headon", 128, 95, 33, 0),
    Cell(0.14, 3, "parallel", 128, 33, 95, 0),
    Cell(0.14, 3, "headon", 128, 58, 70, 0),
    Cell(0.28, 1, "parallel", 128, 90, 38, 0),
    Cell(0.28, 1, "headon", 128, 81, 47, 0),
    Cell(0.28, 3, "parallel", 128, 4, 124, 0),
    Cell(0.28, 3, "headon", 128, 14, 107, 7),
)


def scenes_per_hour() -> float:
    """D2c 续跑 jobs=3 的日历吞吐：实飞场数 / 续跑墙钟。"""
    return D2C_EXECUTED_SCENES / D2C_RESUME_WALL_S * 3600.0


def mean_scene_wall_s() -> float:
    """1792 场单场墙钟之和的平均。这是各场 wall 的均值，不是日历时间。"""
    return D2C_SCENE_WALL_SUM_S / D2C_SCENE_COUNT


def keep_scene(metrics: dict, *, margin_m: float = 0.0) -> bool:
    """净空过滤。默认关在调度器里；这里只判断一条已经写好的记录要不要留下。

    margin_m = 0 时与「成功」同一条线：min_obstacle_clearance_m >= 0。
    """
    if metrics.get("success") is not True:
        return False
    clearance = metrics.get("min_obstacle_clearance_m")
    if clearance is None:
        return False
    return float(clearance) >= float(margin_m)


def _expected_attempts(need_clean: int, cell: Cell) -> int:
    if need_clean <= 0:
        return 0
    if cell.success <= 0:
        raise ValueError(f"{cell.key} 成功数为 0，无法外推重采样")
    return int(ceil(need_clean * cell.concluded / cell.success))


def runs_for_cell(cell: Cell, *, quota: int, fresh: bool, cap: int | None) -> int:
    """这一格还要启动多少次容器。

    fresh：新高度，从 0 条合格开始。空场只排 planned 个种子，不重采样。
    不是 fresh：1.0 m 已经跑过 planned 个种子，合格条数是 success，只补差额。
    cap：这一格的总尝试上限，含已经跑过的种子。None 表示不封顶。
    """
    if quota < 1:
        raise ValueError(f"配额至少为 1，收到 {quota}")
    if not cell.dense:
        return cell.planned if fresh else 0
    have = 0 if fresh else cell.success
    expected = _expected_attempts(max(0, quota - have), cell)
    if cap is None:
        return expected
    if cap < 1:
        raise ValueError(f"上限至少为 1，收到 {cap}")
    already = 0 if fresh else cell.planned
    return min(expected, max(0, int(cap) - already))


def plan_runs(*, quota: int = DEFAULT_QUOTA, cap: int | None = None) -> dict:
    """新增容器场数。new 是 1.5 m 整档，backfill 是 1.0 m 密集格补到配额。"""
    new = sum(runs_for_cell(c, quota=quota, fresh=True, cap=cap) for c in D2C_CELLS)
    backfill = sum(runs_for_cell(c, quota=quota, fresh=False, cap=cap) for c in D2C_CELLS)
    rate = scenes_per_hour()
    total = new + backfill
    return {
        "quota": quota,
        "cap": cap,
        "new_altitude_runs": new,
        "backfill_runs": backfill,
        "total_runs": total,
        "scenes_per_hour": rate,
        "wall_hours": total / rate,
        "new_altitude_hours": new / rate,
        "backfill_hours": backfill / rate,
    }
