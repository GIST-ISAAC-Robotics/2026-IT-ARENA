"""GUARD 후보의 좁은 후반부 무반환 위험 분류. 자유 공간/거리 반환을 생성하지 않는다.

기존 좁은 결측의 3도·양옆 유효 반환 원칙을 후반부에 한정해 비교한다. 새 외곽이 현재
차체 후반 옆면에서 1 mm 이내일 때만 후보이며, 이는 수치 오차 보정이 아니라 미검증
운용 위험 상한이다. 얇은 돌출물·반사 실패·차체 추정 오차와 동적 접근은 배제하지 못한다.
"""
import math
import numpy as np

from arena_autonomy.observation_support import source_valid


def bounded_rear_fringe(scene, raw_scan, queries, padding, length, width):
    result = {'accepted': False, 'reason': 'invalid_input', 'unknown_filled': False,
              'residual_hidden_object_risk': True, 'maximum_exterior_m': .001,
              'maximum_gap_rad': math.radians(3), 'minimum_neighbor_range_m': .6}

    def finish(reason, **values):
        return {**result, 'reason': reason, **values}

    if raw_scan is None:
        return finish('raw_scan_required')
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    pad = np.asarray(padding, dtype=float)
    if not len(q) or not np.all(np.isfinite(q)) or not source_valid(scene):
        return finish('invalid_scene_or_queries')
    if pad.shape != (len(scene.rays),) or not np.all(np.isfinite(pad)) or np.any(pad < 0):
        return finish('invalid_padding')
    # 전방/뒤 범퍼를 넘는 진행 영역은 제외. 현재 차체 후반 옆면의 얇은 띠만 본다.
    exterior = np.abs(q[:, 1])-width/2
    result.update(queries=len(q), exterior_max_m=float(exterior.max()))
    if (np.any(q[:, 0] < -length/2) or np.any(q[:, 0] > 0) or np.any(exterior <= 0)
            or np.any(exterior > result['maximum_exterior_m'])):
        return finish('outside_rear_fringe')
    try:
        values = np.asarray(raw_scan.ranges, dtype=float)
        first = raw_scan.header.stamp.sec+raw_scan.header.stamp.nanosec*1e-9
        step = float(raw_scan.time_increment)
        if (raw_scan.header.frame_id != scene.source_frame_id or len(values) != len(scene.rays)
                or abs(first-scene.source_first_s) > 1e-9 or not math.isfinite(step) or step <= 0
                or any(abs(r.acquired_at_s-(first+i*step)) > 1e-9 for i, r in enumerate(scene.rays))):
            return finish('raw_scene_mismatch')
    except (AttributeError, TypeError, ValueError):
        return finish('raw_scene_mismatch')

    origins = np.asarray([r.ray_origin_xy_m for r in scene.rays])
    directions = np.asarray([r.direction_xy for r in scene.rays])
    statuses = np.asarray([r.status for r in scene.rays])
    valid = statuses == 'return'
    hits = np.asarray([r.hit_xy_m for r in scene.rays if r.status == 'return'])
    # 가까운 실제 점유는 결측 이웃의 먼 반환으로 덮지 않는다. 기존 나이별 점유 여유 그대로다.
    if len(hits) and np.any(np.linalg.norm(q[:, None, :]-hits[None, :, :], axis=2) <= pad[valid]):
        return finish('known_occupied_padding')
    depths = np.asarray([math.dist(r.ray_origin_xy_m, r.hit_xy_m) if r.status == 'return' else -1.
                         for r in scene.rays])
    checked = set()
    groups = {}
    for point in q:
        delta = point-origins
        radius = np.linalg.norm(delta, axis=1)
        if np.any(radius <= 1e-9):
            return finish('query_at_sensor_origin')
        angles = np.arctan2(directions[:, 0]*delta[:, 1]-directions[:, 1]*delta[:, 0],
                            np.sum(directions*delta, axis=1))
        lo = int(np.argmin(np.where(angles >= -1e-10, np.maximum(angles, 0.), np.inf)))
        hi = int(np.argmin(np.where(angles <= 1e-10, np.maximum(-angles, 0.), np.inf)))
        if (angles[lo] < -1e-10 or angles[hi] > 1e-10 or abs(lo-hi) > 1
                or abs(angles[lo])+abs(angles[hi]) > .025):
            return finish('unbracketed_query')
        pair = (lo, hi)
        if not any(statuses[i] == 'unknown' for i in pair):
            return finish('not_nonreturn_only')
        for i in pair:
            if statuses[i] == 'masked':
                return finish('mask_boundary')
            if statuses[i] == 'return':
                if depths[i]-radius[i]-pad[i] <= 0:
                    return finish('near_return')
                continue
            if i in checked:
                continue
            start = end = i
            while start > 0 and statuses[start-1] == 'unknown':
                start -= 1
            while end+1 < len(statuses) and statuses[end+1] == 'unknown':
                end += 1
            a, b = start-1, end+1
            if a < 0 or b >= len(statuses) or statuses[a] != 'return' or statuses[b] != 'return':
                return finish('unbounded_nonreturn')
            # NaN·유한한 최소거리 미만·음의 무한대는 무반환의 양의 무한대와 구분한다.
            if not np.all(np.isposinf(values[start:end+1])):
                return finish('invalid_near_or_nonfinite_return')
            gap = abs(math.atan2(float(directions[a, 0]*directions[b, 1]-directions[a, 1]*directions[b, 0]),
                                 float(np.dot(directions[a], directions[b]))))
            if gap > result['maximum_gap_rad']:
                return finish('wide_nonreturn')
            if min(depths[a], depths[b]) < result['minimum_neighbor_range_m']:
                return finish('near_gap_neighbor')
            if (not np.all(np.isfinite(values[[a, b]]))
                    or max(abs(values[a]-depths[a]), abs(values[b]-depths[b])) > 1e-6):
                return finish('raw_scene_mismatch')
            groups[(start, end)] = {'first': start, 'last': end, 'gap_rad': gap,
                                    'neighbor_ranges_m': [float(depths[a]), float(depths[b])]}
            checked.update(range(start, end+1))
    if sum(g['gap_rad'] for g in groups.values()) > math.radians(6):
        return finish('many_nonreturn_gaps')
    return finish('bounded_rear_nonreturn_risk', accepted=True, gaps=list(groups.values()))
