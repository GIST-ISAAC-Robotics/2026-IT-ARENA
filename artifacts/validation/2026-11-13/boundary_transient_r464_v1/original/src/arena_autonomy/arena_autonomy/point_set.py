"""같은 2차원 표본 집합·사전식 순서를 보존하는 중복 제거."""
import numpy as np


def unique_xy(points):
    """유한 float64 좌표의 structured-array 정렬 비용만 줄인다.

    반올림/격자 축소/근접점 병합은 없다. signed zero나 비유한 값처럼
    대표 원소 선택이 영향을 줄 수 있는 경우는 기존 NumPy 경로를 쓴다.
    """
    values = np.asarray(points)
    if (values.ndim != 2 or values.shape[1] != 2 or values.dtype != np.dtype('float64')
            or not np.all(np.isfinite(values))
            or np.any((values == 0.) & np.signbit(values))):
        return np.unique(values, axis=0)
    if len(values) < 2:
        return values.copy()
    ordered = values[np.lexsort((values[:, 1], values[:, 0]))]
    keep = np.empty(len(ordered), dtype=bool)
    keep[0] = True
    keep[1:] = np.any(ordered[1:] != ordered[:-1], axis=1)
    return ordered[keep]
