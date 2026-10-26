"""측방 접근 사건의 평가 전용 진단. 정답은 제어기에 전달하지 않는다.

기존 간격 통과선을 대신하지 않고, 상대차의 뒤→옆→앞 이동과 자차의
횡이동·정지/재개를 따로 표시한다. 정지한 상대 가정의 반사실 궤적이나
회피 가능한 충돌의 책임을 이 기록만으로 단정하지 않는다.
"""
from bisect import bisect_right
import math

import g1_traffic_eval as ev


def encounter_metrics(ego, actor, centerline, first, last, *, minimum_gap_m=.15):
    if (not math.isfinite(minimum_gap_m) or minimum_gap_m < 0
            or not math.isfinite(first) or not math.isfinite(last) or last <= first):
        raise ValueError('invalid encounter window/gap')
    if not ev._pose_rows_valid(ego) or not ev._pose_rows_valid(actor):
        return {'valid': False, 'reason': 'missing_or_invalid_pose_stream'}
    rows = ev.window_tuples(ego, first, last)
    coverage = {name: ev.stream_coverage([r[0] for r in records], first, last, ev.POSE_MAX_GAP_S)
                for name, records in (('ego', rows), ('actor', ev.window_tuples(actor, first, last)))}
    actor_times = [row[0] for row in actor]
    joined = []
    missing = 0
    for time, x, y, yaw in rows:
        index = bisect_right(actor_times, time)-1
        if index < 0 or time-actor_times[index] > ev.POSE_MAX_GAP_S:
            missing += 1
            continue
        _, ax, ay, ayaw = actor[index]
        c, s = math.cos(yaw), math.sin(yaw)
        longitudinal = c*(ax-x)+s*(ay-y)
        lateral = -s*(ax-x)+c*(ay-y)
        along, _ = centerline.nearest_s(x, y)
        cx, cy, heading = centerline.pose(along)
        road_lateral = -(x-cx)*math.sin(heading)+(y-cy)*math.cos(heading)
        gap = ev.rectangle(x, y, yaw, .20, .15).distance(ev.rectangle(ax, ay, ayaw, .20, .15))
        joined.append({'t': time, 'ego_x': x, 'ego_y': y, 'actor_x': ax, 'actor_y': ay,
            'longitudinal_m': longitudinal, 'lateral_m': lateral, 'ego_road_lateral_m': road_lateral,
            'gap_m': gap})
    if not joined:
        return {'valid': False, 'reason': 'no_joined_samples', 'missing': missing}
    behind = next((r for r in joined if r['longitudinal_m'] <= -.20), None)
    ahead = next((r for r in joined if behind and r['t'] > behind['t'] and r['longitudinal_m'] >= .20), None)
    alongside = [r for r in joined if abs(r['longitudinal_m']) < .20]
    below = [r for r in joined if r['gap_m'] < minimum_gap_m]
    initial_lat = joined[0]['ego_road_lateral_m']
    # 간격 기준을 처음 넘긴 한 구간만 병기한다. 운동의 책임/제어의 좋고 나쁨 판정은 아니다.
    change = None
    if below:
        r = below[0]
        index = next(i for i, v in enumerate(joined) if v is r)
        if index > 0:
            previous = joined[index-1]
            change = {'interval_s': r['t']-previous['t'],
                'ego_translation_m': math.hypot(r['ego_x']-previous['ego_x'], r['ego_y']-previous['ego_y']),
                'actor_translation_m': math.hypot(r['actor_x']-previous['actor_x'], r['actor_y']-previous['actor_y']),
                'ego_road_lateral_change_m': r['ego_road_lateral_m']-previous['ego_road_lateral_m']}
    return {'valid': missing == 0 and all(v['valid'] for v in coverage.values()),
        'coverage': coverage, 'samples': len(joined), 'missing_actor_matches': missing,
        'minimum_gap_m': min(r['gap_m'] for r in joined), 'registered_gap_m': minimum_gap_m,
        'first_below_gap_s': below[0]['t'] if below else None, 'first_below_gap_motion': change,
        'behind_observed_s': behind['t'] if behind else None, 'ahead_after_behind_s': ahead['t'] if ahead else None,
        'behind_to_ahead_observed': behind is not None and ahead is not None,
        'alongside_samples': len(alongside),
        'initial_ego_road_lateral_m': initial_lat,
        'maximum_ego_road_lateral_change_m': max(abs(r['ego_road_lateral_m']-initial_lat) for r in joined),
        'maximum_absolute_ego_road_lateral_m': max(abs(r['ego_road_lateral_m']) for r in joined),
        'scope': 'sampled encounter diagnosis only; fixed 20x15 cm actor; not continuous collision or blame proof'}
