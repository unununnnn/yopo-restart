"""OpenCV INPAINT_NS 的单通道 uint8 / radius=1 特化，numpy + 标准库。

算法转写自 OpenCV 4.10.0 modules/photo/src/inpaint.cpp，许可证见
仓库 LICENSES/opencv-inpaint.txt。仅网络图使用，绝不生成安全证据。
"""

import heapq
import math

import numpy as np


def ns_radius_one(image: np.ndarray, holes: np.ndarray) -> np.ndarray:
    out = np.asarray(image, dtype=np.uint8).copy()
    if not np.any(holes) or np.all(holes):
        return out
    mask = np.pad(np.asarray(holes, dtype=bool), 1)
    band = np.zeros_like(mask)
    band[1:] |= mask[:-1]
    band[:-1] |= mask[1:]
    band[:, 1:] |= mask[:, :-1]
    band[:, :-1] |= mask[:, 1:]
    band &= ~mask
    band[[0, -1]] = False
    band[:, [0, -1]] = False
    f = np.where(mask, 2, band.astype(np.uint8))
    t = np.full(mask.shape, 1e6, dtype=np.float32)
    t[band] = 0
    heap = [(0.0, n, int(i), int(j)) for n, (i, j) in enumerate(np.argwhere(band))]
    heapq.heapify(heap)
    order = len(heap)
    rows, cols = f.shape
    F = np.float32

    def solve(i1, j1, i2, j2):
        a, b = float(t[i1, j1]), float(t[i2, j2])
        if f[i1, j1] != 2:
            if f[i2, j2] != 2:
                return F(
                    1 + min(a, b)
                    if abs(a - b) >= 1
                    else (a + b + math.sqrt(2 - (a - b) ** 2)) * 0.5
                )
            return F(1 + a)
        return F(1 + b if f[i2, j2] != 2 else 1 + min(a, b))

    def value(y, x):
        return int(out[y, x])

    while heap:
        _, _, ii, jj = heapq.heappop(heap)
        f[ii, jj] = 0
        for i, j in ((ii - 1, jj), (ii, jj - 1), (ii + 1, jj), (ii, jj + 1)):
            if not (0 < i < rows - 1 and 0 < j < cols - 1) or f[i, j] != 2:
                continue
            dist = min(
                solve(i - 1, j, i, j - 1),
                solve(i + 1, j, i, j - 1),
                solve(i - 1, j, i, j + 1),
                solve(i + 1, j, i, j + 1),
            )
            t[i, j] = dist
            total, weight = F(0), F(1e-20)
            # radius=1 的圆只含四邻域（中心仍 INSIDE）。
            for k, l in ((i - 1, j), (i, j - 1), (i, j + 1), (i + 1, j)):
                if not (0 < k < rows - 1 and 0 < l < cols - 1) or f[k, l] == 2:
                    continue
                km, kp = k - 1 + (k == 1), k - 1 - (k == rows - 2)
                lm, lp = l - 1 + (l == 1), l - 1 - (l == cols - 2)
                if f[k + 1, l] != 2:
                    gx = abs(value(kp + 1, lm) - value(kp, lm))
                    gx = gx + abs(value(kp, lm) - value(km - 1, lm)) if f[k - 1, l] != 2 else gx * 2
                else:
                    gx = abs(value(kp, lm) - value(km - 1, lm)) * 2 if f[k - 1, l] != 2 else 0
                if f[k, l + 1] != 2:
                    gy = abs(value(km, lp + 1) - value(km, lm))
                    gy = gy + abs(value(km, lm) - value(km, lm - 1)) if f[k, l - 1] != 2 else gy * 2
                else:
                    gy = abs(value(km, lm) - value(km, lm - 1)) * 2 if f[k, l - 1] != 2 else 0
                gx = -gx
                rx, ry = j - l, i - k
                direction = rx * gx + ry * gy
                direction = (
                    F(1e-6)
                    if abs(direction) <= 0.01
                    else F(abs(direction / math.sqrt((rx * rx + ry * ry) * (gx * gx + gy * gy))))
                )
                w = F(F(0.5) * direction)
                # aarch64 OpenCV 使用 float32 FMA；float64 中乘加后一次舍入复现它。
                total = F(float(total) + float(w) * value(k - 1, l - 1))
                weight = F(weight + w)
            out[i - 1, j - 1] = np.uint8(np.clip(np.rint(float(total) / float(weight)), 0, 255))
            f[i, j] = 1
            heapq.heappush(heap, (float(dist), order, i, j))
            order += 1
    return out
