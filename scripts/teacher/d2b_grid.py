"""D2b 的扩种网格。几何假设沿用 assumptions.py，不改那份文件。

密度只取 0、0.14、0.28。D2 在密度 0.42 上三机、六机成功数为 0，本票不加该档。
默认单机和三机。六机只在密度 0（空场）上可选追加，排在队列末尾。
种子从 0 起连续编号，含 D2 已跑过的 0–7，方便这份数据自成一套。
"""

from __future__ import annotations

from teacher.assumptions import FLIGHT_Z_M, PATTERNS
from teacher.scenarios import make_scenario

# 这是假设，不是论文复现。见 assumptions.py 文件头。
DENSITIES_PER_M2 = (0.0, 0.14, 0.28)
DRONE_COUNTS = (1, 3)
DEFAULT_SEED_COUNT = 128
# D2 小批量合计成功 90 场，见 docs/research/d2-teacher-data.md。
D2_SUCCESS_TOTAL = 90
# 比 D2 成功条数至少多一个数量级。
TARGET_MIN_SUCCESS = 10 * D2_SUCCESS_TOTAL


def seed_range(seed_count: int) -> tuple[int, ...]:
    if seed_count < 1:
        raise ValueError(f"种子数至少为 1，收到 {seed_count}")
    return tuple(range(int(seed_count)))


def iter_d2b_scenarios(
    *,
    seed_count: int = DEFAULT_SEED_COUNT,
    include_six_open: bool = False,
    flight_zs: tuple[float, ...] = (FLIGHT_Z_M,),
):
    """默认只有 1.0 m，场次编号和 D2c 的 1792 场一致。

    ``flight_zs=(1.0, 1.5)`` 时每个种子再出一档 1.5 m，编号带 ``z150``。
    圆柱只看水平锚点，两档高度的障碍是同一批。
    """
    seeds = seed_range(seed_count)
    zs = tuple(float(z) for z in flight_zs)
    if not zs:
        raise ValueError("flight_zs 至少要有一个高度")
    for z in zs:
        for density in DENSITIES_PER_M2:
            for n_drones in DRONE_COUNTS:
                for pattern in PATTERNS:
                    for seed in seeds:
                        yield make_scenario(density, n_drones, pattern, seed, flight_z_m=z)
        if include_six_open:
            for pattern in PATTERNS:
                for seed in seeds:
                    yield make_scenario(0.0, 6, pattern, seed, flight_z_m=z)


def grid_size(
    seed_count: int = DEFAULT_SEED_COUNT,
    include_six_open: bool = False,
    flight_zs: tuple[float, ...] = (FLIGHT_Z_M,),
) -> int:
    n_z = len(tuple(flight_zs))
    n = len(DENSITIES_PER_M2) * len(DRONE_COUNTS) * len(PATTERNS) * seed_count * n_z
    if include_six_open:
        n += len(PATTERNS) * seed_count * n_z
    return n
