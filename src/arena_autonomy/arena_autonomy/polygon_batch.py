"""볼록 다각형 반평면 교차의 배열 계산. 꼭짓점/연산 순서와 닫힌 경계를 유지한다."""
import numpy as np


def clip_polygons(polygons, counts, origins, directions, sign):
    """행별 polygon의 clip_halfplane과 같은 순서. 빈 행과 경계 중복점도 보존한다."""
    polygons = np.asarray(polygons, dtype=float)
    counts = np.asarray(counts, dtype=np.int64)
    n, width, _ = polygons.shape
    if not n:
        return np.empty((0, 0, 2)), counts.copy()
    if not width:
        return polygons.copy(), counts.copy()
    origins = np.broadcast_to(origins, (n, 2))
    directions = np.broadcast_to(directions, (n, 2))
    rows = np.arange(n)
    previous = polygons[rows, np.maximum(0, counts-1)]
    pv = sign*(directions[:, 0]*(previous[:, 1]-origins[:, 1])-
               directions[:, 1]*(previous[:, 0]-origins[:, 0]))
    # 볼록성의 부동소수점 경계까지 포함해 한 꼭짓점당 최대 두 출력을 허용한다.
    result = np.empty((n, 2*width, 2))
    used = np.zeros(n, dtype=np.int64)
    for index in range(width):
        current = polygons[:, index]
        valid = index < counts
        cv = sign*(directions[:, 0]*(current[:, 1]-origins[:, 1])-
                   directions[:, 1]*(current[:, 0]-origins[:, 0]))
        cross = np.flatnonzero(valid & ((cv >= 0) != (pv >= 0)))
        if len(cross):
            t = pv[cross]/(pv[cross]-cv[cross])
            result[cross, used[cross]] = previous[cross]+t[:, None]*(current[cross]-previous[cross])
            used[cross] += 1
        inside = np.flatnonzero(valid & (cv >= 0))
        result[inside, used[inside]] = current[inside]
        used[inside] += 1
        previous, pv = current, cv
    # 미사용 칸은 0으로 채워 후속 계산의 비활성 행이 미초기화 값을 읽지 않게 한다.
    result = result[:, :int(used.max()), :]
    if result.shape[1]: result[np.arange(result.shape[1])[None, :] >= used[:, None]] = 0.
    return result, used


def exterior_queries(polygons, counts, length, width):
    """outside_body와 같은 네 조각, area>1e-14, 꼭짓점 평균. 질의 순서는 무관하다."""
    results = []; intersections = 0
    for origin, direction, sign in (((length/2, 0.), (0., 1.), -1),
                                    ((-length/2, 0.), (0., 1.), 1),
                                    ((0., width/2), (1., 0.), 1),
                                    ((0., -width/2), (1., 0.), -1)):
        pieces, sizes = clip_polygons(polygons, counts, origin, direction, sign)
        sums = np.zeros(len(sizes))
        for index in range(pieces.shape[1]):
            active = np.flatnonzero(index < sizes)
            a = pieces[active, index]
            b = pieces[active, (index+1) % sizes[active]]
            # 기존 Python sum과 같은 왼쪽부터의 덧셈 순서를 유지한다.
            sums[active] += a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0]
        keep = (sizes >= 3) & (abs(sums)*.5 > 1e-14)
        intersections += int(keep.sum())
        for count in np.unique(sizes[keep]):
            part = pieces[keep & (sizes == count), :count]
            results.extend((part.reshape(-1, 2), np.mean(part, axis=1)))
    return (np.vstack(results) if results else np.empty((0, 2))), intersections
