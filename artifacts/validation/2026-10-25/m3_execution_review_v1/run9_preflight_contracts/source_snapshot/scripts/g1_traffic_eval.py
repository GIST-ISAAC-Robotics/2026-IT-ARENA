"""G1-b Gazebo 상대차 시험의 평가 전용 순수 함수.

정답 자세·actor 일정·공식 중심선은 이 모듈과 평가기에서만 사용한다. 제어기/보호층
입력으로 전달하지 않는다. 표본 사이 연속 충돌은 보수적 외곽 하한으로만 다루며,
물리 접촉 증명은 별도의 접촉 센서 기록과 구분한다.
"""
from bisect import bisect_left, bisect_right
import math

import numpy as np
from shapely.geometry import MultiPoint, Polygon

EGO_LENGTH_M, EGO_WIDTH_M = .20, .15
EVALUATOR_VERSION = 'g1d_eval_v7_dense_stop_evidence'
LATCHED_STATES = ('FAULT_LATCHED', 'ESTOP_LATCHED')
# 누락을 성공으로 해석하지 않는 평가 계측 조건. 기존 행동 판정선은 그대로다.
POSE_MAX_GAP_S = .05  # 기존 actor_pose_stream_continuous 기준을 자차/짝맞춤에도 적용
TRACE_MAX_GAP_S = .20  # 10 Hz trace의 두 발행 주기; 그보다 긴 공백을 연결하지 않음


def _finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def kinematic_speed_segments(ego):
    """평가용 실제 pose의 평면 구간 평균속도. 센서/순간 속도 추정값은 아니다."""
    if not all(len(row) == 4 and all(_finite(v) for v in row) for row in ego):
        raise ValueError('invalid ego pose shape/values')
    result = []
    for (t0, x0, y0, _), (t1, x1, y1, _) in zip(ego, ego[1:]):
        if t1 <= t0:
            raise ValueError('invalid/nonincreasing ego pose')
        result.append({'t0': t0, 't1': t1, 'average_speed_mps': math.hypot(x1-x0, y1-y0)/(t1-t0)})
    return result


def pose_window_coverage(segments, first, end):
    """STOP의 기존 계측 간격(.05초)과 속도(.01 m/s) 검사. 구간 양 끝도 요구한다."""
    if not (_finite(first) and _finite(end) and end > first):
        return dict(complete=False, passed=False, reason='invalid_window', segment_count=0,
                    maximum_speed_mps=None)
    selected = [s for s in segments if s['t1'] > first+1e-9 and s['t0'] < end-1e-9]
    begin = selected[0]['t0'] if selected else None
    last = selected[-1]['t1'] if selected else None
    gap = max((s['t1']-s['t0'] for s in selected), default=None)
    valid = all(all(_finite(s.get(k)) for k in ('t0', 't1', 'average_speed_mps'))
                and 0 < s['t1']-s['t0'] and s['average_speed_mps'] >= 0 for s in selected)
    contiguous = all(abs(b['t0']-a['t1']) <= 1e-9 for a, b in zip(selected, selected[1:]))
    complete = bool(selected and valid and contiguous and begin <= first+1e-9 and last >= end-1e-9
                    and gap <= POSE_MAX_GAP_S+1e-9)
    peak = max((s['average_speed_mps'] for s in selected), default=None)
    return dict(complete=complete, passed=bool(complete and peak < .01),
                reason='covered' if complete else 'incomplete_pose_window',
                required_first_s=first, required_last_s=end, covered_first_s=begin,
                covered_last_s=last, segment_count=len(selected), maximum_gap_s=gap,
                maximum_allowed_gap_s=POSE_MAX_GAP_S, maximum_speed_mps=peak)


def stop_pose_evidence(ego, end_s, hold_s):
    """실행기와 사후 감사가 공유하는 평가-only STOP 증거. 구동으로 전달하지 않는다."""
    if not _finite(hold_s) or hold_s <= 0 or not _finite(end_s):
        raise ValueError('finite stop time and positive hold duration required')
    return pose_window_coverage(kinematic_speed_segments(ego), end_s-hold_s, end_s)


def fresh_clock_ns(spin, state, monotonic, timeout_s=1.):
    """STOP 요청 직전 평가기 /clock을 새로 받아 정수 ns로 돌려준다.

    평가기는 사례 뒤 그래프 감사 동안 spin하지 않아 보관한 /clock이 오래될 수 있다(v3 실행에서
    요청 직후 자세 시각보다 0.118 s 이전). 기준 시점 뒤 /clock 콜백이 두 번 처리되어야 한다.
    첫 번째는 수신 대기열에 남아 있던 이전 표본일 수 있기 때문이다. 제한 시간 안에 받지 못하면
    None을 돌려 STOP 전 고정 판정을 불가로 실패시킨다. 판정선이 아니라 대기 상한이다.
    """
    mark, start = monotonic(), state.get('clock_receipts', 0)
    while True:
        received = state.get('clock_receipts', 0)-start
        if received >= 2 and type(state.get('clock_ns')) is int:
            return state['clock_ns'], {'fresh': True, 'receipts_after_mark': received, 'wait_wall_s': monotonic()-mark}
        if monotonic()-mark > timeout_s:
            return None, {'fresh': False, 'receipts_after_mark': received, 'wait_wall_s': monotonic()-mark}
        spin()


def stop_latch_timing(actuation_trace, request_clock_ns, stop_state='ESTOP_LATCHED', stop_reason='software_stop'):
    """명시 STOP 요청 직전의 평가기 /clock(정수 ns)과 MCU `now_us`(정수 µs)를 정수로 비교한다.

    v2는 요청 뒤 기록한 자세 시각(float 초)과 `now_us*1e-6`을 비교해, 같은 ms의 명시 STOP
    응답도 STOP 전으로 셀 수 있었다. 요청 µs보다 1 µs라도 이른 고정은 사유와 무관하게 STOP
    전으로 센다. 같은 µs에서는 설정한 STOP 상태·사유의 표본만 요청 응답으로 보고, FAULT·다른
    사유는 STOP 전으로 센다. 요청 시각 미기록, 정수가 아니거나 음수인 표본 시각은 판정 불가로
    실패시킨다. 노드마다 /clock을 받는 시점은 다르므로 인과 순서를 증명하지는 않는다.
    """
    request_valid = type(request_clock_ns) is int and request_clock_ns >= 0
    request_us = request_clock_ns//1000 if request_valid else None
    invalid = before = same_stop = same_other = 0
    first = None
    for sample in actuation_trace:
        now = sample.get('now_us') if isinstance(sample, dict) else None
        if type(now) is not int or now < 0:
            invalid += 1
            continue
        if not request_valid or sample.get('state') not in LATCHED_STATES:
            continue
        if first is None:
            first = {key: sample.get(key) for key in ('now_us', 'state', 'reason')}
        if now < request_us:
            before += 1
        elif now == request_us:
            if sample.get('state') == stop_state and sample.get('reason') == stop_reason:
                same_stop += 1
            else:
                same_other += 1
    return {'valid': request_valid and invalid == 0, 'request_clock_ns': request_clock_ns if request_valid else None,
            'request_clock_us': request_us, 'invalid_time_samples': invalid, 'latched_before_request': before,
            'same_instant_stop_response': same_stop, 'same_instant_other_latch': same_other,
            'latched_before_stop_samples': before+same_other, 'first_latched': first,
            'limits': ['request time is the evaluator /clock freshly received (two callbacks after a mark) just before the STOP call; '
                       'nodes receive /clock at different times, so causal order is not proven',
                       'equal microseconds are attributed to the STOP response only for the configured STOP state and reason',
                       'latched status received before the STOP call is still caught by the case-loop abort in wall order']}


def _pose_rows_valid(rows):
    return all(len(row) == 4 and all(_finite(v) for v in row) for row in rows) and all(
        b[0] > a[0] for a, b in zip(rows, rows[1:]))


def stream_coverage(times, first, last, max_gap):
    """양 끝 포함 수신 범위와 내부 공백을 확인한다. 빈/한 표본은 연속 증거가 아니다."""
    valid = bool(times) and all(_finite(t) for t in times) and all(b > a for a, b in zip(times, times[1:]))
    gaps = [b-a for a, b in zip(times, times[1:])] if valid else []
    covered = (valid and len(times) > 1 and times[0] <= first+max_gap
               and times[-1] >= last-max_gap and max(gaps) <= max_gap+1e-9)
    return {'valid': bool(covered), 'samples': len(times), 'first_s': times[0] if times else None,
            'last_s': times[-1] if times else None, 'required_first_s': first, 'required_last_s': last,
            'max_interval_s': max(gaps) if gaps else None, 'maximum_allowed_interval_s': max_gap}


def contact_connection_valid(connection):
    """구독자만 있는 토픽을 접촉 발행자 연결 증거로 쓰지 않는다."""
    output = connection.get('output', '')
    publisher_block = output.split('Publishers', 1)[-1].split('Subscribers', 1)[0] if 'Publishers' in output else ''
    return (connection.get('returncode') == 0 and connection.get('reader_alive_at_graph_audit') is True
            and 'gz.msgs.Contacts' in publisher_block and 'tcp://' in publisher_block)


def rectangle(x, y, yaw, length, width):
    if not all(_finite(v) for v in (x, y, yaw, length, width)) or length <= 0 or width <= 0:
        raise ValueError('rectangle requires finite pose and positive finite size')
    c, s = math.cos(yaw), math.sin(yaw)
    corners = ((length/2, width/2), (-length/2, width/2), (-length/2, -width/2), (length/2, -width/2))
    return Polygon([(x+c*a-s*b, y+s*a+c*b) for a, b in corners])


def wrap_angle(value):
    return (value+math.pi) % math.tau-math.pi


class Centerline:
    """공식 중심선의 호길이 보간. 평가기 전용이다."""

    def __init__(self, x, y, s, lap_length):
        self.xy = np.column_stack((np.asarray(x, float), np.asarray(y, float)))
        self.s = np.asarray(s, float)
        self.lap = float(lap_length)
        if (len(self.s) < 3 or len(self.xy) != len(self.s) or not np.all(np.isfinite(self.xy))
                or not np.all(np.isfinite(self.s)) or not _finite(self.lap)
                or np.any(np.diff(self.s) <= 0) or not self.lap > self.s[-1]):
            raise ValueError('invalid centerline')

    def pose(self, s_value):
        s_value = float(s_value) % self.lap
        closed_s = np.r_[self.s, self.lap]
        closed = np.vstack((self.xy, self.xy[:1]))
        index = min(max(int(np.searchsorted(closed_s, s_value, side='right'))-1, 0), len(closed_s)-2)
        fraction = (s_value-closed_s[index])/(closed_s[index+1]-closed_s[index])
        point = closed[index]+fraction*(closed[index+1]-closed[index])
        delta = closed[index+1]-closed[index]
        return float(point[0]), float(point[1]), math.atan2(delta[1], delta[0])

    def nearest_s(self, x, y):
        index = int(np.argmin(np.sum((self.xy-np.array([x, y]))**2, axis=1)))
        return float(self.s[index]), float(np.linalg.norm(self.xy[index]-np.array([x, y])))

    def unwrap(self, s_value, reference):
        """reference 근처의 연속 호길이로 바꾼다(랩 경계 처리)."""
        return reference+((s_value-reference+self.lap/2) % self.lap-self.lap/2)


def actor_travel_m(schedule, elapsed_s):
    """[from_s, to_s) 구간별 등속 일정의 누적 이동거리. 시작 전은 0."""
    if elapsed_s <= 0:
        return 0.
    return sum(max(0., min(elapsed_s, row['to_s'])-row['from_s'])*row['speed_mps'] for row in schedule)


def actor_speed_mps(schedule, elapsed_s):
    for row in schedule:
        if row['from_s'] <= elapsed_s < row['to_s']:
            return row['speed_mps']
    return 0.


def validate_schedule(schedule, duration_s):
    if not _finite(duration_s) or duration_s <= 0:
        raise ValueError('duration must be positive and finite')
    last = 0.
    for row in schedule:
        if (not all(_finite(row[k]) for k in ('from_s', 'to_s', 'speed_mps'))
                or not math.isclose(row['from_s'], last, abs_tol=1e-9) or not row['to_s'] > row['from_s']
                or row['speed_mps'] < 0):
            raise ValueError('schedule must be contiguous, increasing and non-negative')
        last = row['to_s']
    if schedule and not math.isclose(last, duration_s, abs_tol=1e-9):
        raise ValueError('schedule must cover the case duration')


def _latest_index(times, t):
    return bisect_right(times, t)-1


def sampled_gaps(ego, actor, actor_length, actor_width, max_actor_age_s=None):
    """각 ego 표본 시각의 최신 actor 자세와의 차체 간 최소 거리."""
    times = [row[0] for row in actor]
    rows = []
    for t, x, y, yaw in ego:
        index = _latest_index(times, t)
        if index < 0 or (max_actor_age_s is not None and t-times[index] > max_actor_age_s+1e-9):
            rows.append({'t': t, 'gap_m': None, 'overlap_area_m2': None,
                         'actor_pose_age_s': None if index < 0 else t-times[index],
                         'reason': 'actor_pose_missing' if index < 0 else 'actor_pose_stale'})
            continue
        _, ax, ay, ayaw = actor[index]
        e = rectangle(x, y, yaw, EGO_LENGTH_M, EGO_WIDTH_M)
        a = rectangle(ax, ay, ayaw, actor_length, actor_width)
        rows.append({'t': t, 'gap_m': e.distance(a), 'overlap_area_m2': e.intersection(a).area,
                     'actor_pose_age_s': t-actor[index][0]})
    return rows


def swept_gap_lower_bounds(ego, actor, actor_length, actor_width, actor_pose_period_s, max_curvature_per_m,
                           max_actor_age_s=None):
    """연속 ego 두 표본 사이의 차체 점유 외곽과 그 시간창의 모든 actor 자세 외곽의 거리 하한.

    가정: 표본 사이 ego는 강체 평면 운동이며 yaw가 단조 변화하고 경로 곡률이
    max_curvature_per_m 이하다. 그 조건에서 두 끝 차체의 볼록 외곽에서 벗어나는 거리를
    2·r·|Δyaw| + κ·d²/8로 위에서 누른다(r=반대각선). actor는 순간이동하는 정적
    모델이므로 창 앞의 최신 자세부터 발행 주기만큼 뒤까지의 모든 자세를 합친다.
    물리 접촉 기록이 아니다.
    """
    times = [row[0] for row in actor]
    half_diagonal = math.hypot(EGO_LENGTH_M, EGO_WIDTH_M)/2
    out = []
    for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(ego, ego[1:]):
        if t1 <= t0:
            continue
        first = max(_latest_index(times, t0), 0)
        last = bisect_right(times, t1+actor_pose_period_s)
        poses = actor[first:last]
        end_index = _latest_index(times, t1)
        stale = max_actor_age_s is not None and (not poses or end_index < 0
            or t0-poses[0][0] > max_actor_age_s+1e-9 or t1-times[end_index] > max_actor_age_s+1e-9
            or any(b[0]-a[0] > max_actor_age_s+1e-9 for a, b in zip(poses, poses[1:])))
        if not poses or poses[0][0] > t0 or stale:
            # 창 시작 전 actor 자세가 없으면 판단하지 않고 표시만 남긴다.
            out.append({'t0': t0, 't1': t1, 'lower_bound_m': None,
                        'reason': 'actor_pose_stale' if stale else 'actor_pose_missing'})
            continue
        ego_hull = MultiPoint(list(rectangle(x0, y0, a0, EGO_LENGTH_M, EGO_WIDTH_M).exterior.coords)
                              + list(rectangle(x1, y1, a1, EGO_LENGTH_M, EGO_WIDTH_M).exterior.coords)).convex_hull
        actor_points = []
        for _, ax, ay, ayaw in poses:
            actor_points += list(rectangle(ax, ay, ayaw, actor_length, actor_width).exterior.coords)
        actor_hull = MultiPoint(actor_points).convex_hull
        travel = math.hypot(x1-x0, y1-y0)
        inflation = 2*half_diagonal*abs(wrap_angle(a1-a0))+max_curvature_per_m*travel**2/8+1e-6
        out.append({'t0': t0, 't1': t1, 'lower_bound_m': ego_hull.distance(actor_hull)-inflation,
                    'inflation_m': inflation, 'actor_poses_in_window': len(poses)})
    return out


def longest_true_interval(rows, predicate, key='t', max_gap_s=None):
    """연속 표본에서 조건이 참인 구간의 첫·끝 시각 차(표본 개수가 아님). 한 점은 0초."""
    best, start, previous = 0., None, None
    for row in rows:
        if previous is not None and max_gap_s is not None and row[key]-previous > max_gap_s+1e-9:
            start = previous = None
        if predicate(row):
            start = row[key] if start is None else start
            previous = row[key]
            best = max(best, previous-start)
        else:
            start = previous = None
    return best


def progress_series(centerline, ego, start_s=None):
    """중심선 투영 누적 진행. 1 m 넘는 투영 점프는 평가 오류로 남긴다."""
    total, last, rows, jumps = 0., start_s, [], 0
    for t, x, y, _ in ego:
        s, error = centerline.nearest_s(x, y)
        if last is not None:
            delta = (s-last+centerline.lap/2) % centerline.lap-centerline.lap/2
            if abs(delta) > 1.:
                jumps += 1
                delta = 0.
            total += delta
        last = s
        rows.append({'t': t, 'progress_m': total, 'centerline_error_m': error, 's_m': s})
    return rows, jumps


def interval_contacts(contacts, ego_name='arena_car'):
    """접촉 센서 기록 중 자차 충돌체가 포함된 메시지 시각."""
    hits = []
    for row in contacts:
        names = [name for pair in row.get('pairs', []) for name in pair]
        if any(name.split('::')[0] == ego_name for name in names):
            hits.append(row['t'])
    return hits


def actor_tracking_errors(observed, schedule_pose):
    """관측 actor 자세와 같은 시각 일정 자세의 차이. 이동 표적 구현 오차 평가용."""
    rows = []
    for t, x, y, _ in observed:
        tx, ty = schedule_pose(t)
        rows.append({'t': t, 'error_m': math.hypot(x-tx, y-ty)})
    return rows


def window(rows, first, last, key='t'):
    keys = [row[key] for row in rows]
    return rows[bisect_left(keys, first):bisect_right(keys, last)]


def _value_at(rows, t, key):
    """t 이하의 마지막 표본 값."""
    previous = None
    for row in rows:
        if row['t'] > t:
            break
        previous = row[key]
    return previous


def _minimum(values):
    values = [v for v in values if v is not None]
    return min(values) if values and all(_finite(v) for v in values) else None


def evaluate_run(*, config, case, mode, t_green, trace, ego, actor, contacts, commands, stop_start,
                 schedule_pose, centerline, s0, road, runtime_world, contact_evidence=None):
    """기록을 사전 등록 판정선과 대조한다. 결과 해석은 판정 항목별로 분리한다."""
    acceptance, actor_cfg = config['acceptance'], config['actor']
    harness = config['acceptance']['harness']
    result = {'t_green': t_green, 'evaluator_version': EVALUATOR_VERSION}
    if t_green is None or not _finite(t_green):
        result['gates'] = {'green_observed': False}
        return result
    t_end = t_green+case['duration_s']
    numeric_checks = {
        'ego_finite_ordered': _pose_rows_valid(ego),
        'actor_finite_ordered': _pose_rows_valid(actor),
        'trace_finite_ordered': all(all(_finite(r.get(k)) for k in ('t', 'ground_speed_mps', 'safe_speed_mps'))
            and isinstance(r.get('autonomy'), dict) and r['ground_speed_mps'] >= 0
            and all(_finite(r[k]) for k in ('x', 'y', 'yaw', 'drive_speed_mps', 'sampled_gap_m') if k in r)
            for r in trace) and all(b['t'] > a['t'] for a, b in zip(trace, trace[1:])),
        'commands_finite_ordered': all(_finite(c.get('requested_sim_s')) for c in commands) and all(
            b['requested_sim_s'] > a['requested_sim_s'] for a, b in zip(commands, commands[1:])),
        'contacts_finite': all(_finite(r.get('t')) and isinstance(r.get('pairs'), list)
            and all(isinstance(p, list) and len(p) == 2 and all(isinstance(n, str) for n in p)
                    for p in r['pairs']) for r in contacts),
    }
    result['data_checks'] = numeric_checks
    if not all(numeric_checks.values()):
        result['harness'] = {'checks': numeric_checks, 'valid': False}
        result['gates'] = {'data_finite_ordered': False}
        return result
    last_time = max([row[0] for row in ego] or [t_end])
    actor_times_all = [r[0] for r in actor]
    actor_start = max(_latest_index(actor_times_all, t_green), 0)
    actor_end = bisect_right(actor_times_all, last_time+1/actor_cfg['pose_publish_hz'])
    coverage = {
        'ego': stream_coverage([r[0] for r in window_tuples(ego, t_green, last_time)], t_green, t_end, POSE_MAX_GAP_S),
        'actor': stream_coverage([r[0] for r in actor[actor_start:actor_end]], t_green, last_time, POSE_MAX_GAP_S),
        'trace': stream_coverage([r['t'] for r in window(trace, t_green, t_end)], t_green, t_end, TRACE_MAX_GAP_S),
    }
    result['coverage'] = coverage
    # --- 하네스: actor가 실제 실행 사본에 있고 일정대로 움직였는가 ---
    moving = [row for row in actor_tracking_errors(window_tuples(actor, t_green, t_end), schedule_pose)]
    static_rows, moving_rows = [], []
    for row in moving:
        elapsed = row['t']-t_green
        recent = [actor_speed_mps(case['schedule'], elapsed-dt) for dt in (0., .1, .2)]
        (static_rows if max(recent) == 0 else moving_rows).append(row)
    command_times = [c['requested_sim_s'] for c in commands if t_green <= c['requested_sim_s'] <= t_end]
    intervals = np.diff(command_times) if len(command_times) > 1 else np.array([])
    pose_times = [row[0] for row in actor if t_green <= row[0] <= last_time]
    harness_checks = {
        'complete_finite_data': all(numeric_checks.values()),
        'ego_window_covered': coverage['ego']['valid'],
        'actor_window_covered': coverage['actor']['valid'],
        'trace_window_covered': coverage['trace']['valid'],
        'case_reached_stop_request': _finite(stop_start) and stop_start >= t_end,
        'actor_in_runtime_world': f'name="{actor_cfg["model_name"]}"' in runtime_world or
                                  f"name='{actor_cfg['model_name']}'" in runtime_world,
        'actor_pose_before_green': bool(actor) and actor[0][0] <= t_green,
        'actor_pose_stream_continuous': len(pose_times) > 1 and float(np.max(np.diff(pose_times))) <= .05,
        'commands_acknowledged': (bool(commands) or not case['schedule']) and all(c.get('acknowledged') is True for c in commands),
        'command_interval': (not case['schedule']) or stream_coverage(command_times, t_green, t_end, harness['actor_command_max_interval_s'])['valid'],
        'moving_tracking_error': (bool(moving_rows) or not any(r['speed_mps'] > 0 for r in case['schedule']))
            and all(_finite(r['error_m']) and r['error_m'] <= harness['actor_moving_tracking_error_max_m'] for r in moving_rows),
        'static_error': (bool(static_rows) or (bool(case['schedule']) and all(r['speed_mps'] > 0 for r in case['schedule'])))
            and all(_finite(r['error_m']) and r['error_m'] <= harness['actor_static_error_max_m'] for r in static_rows),
    }
    result['harness'] = {'checks': harness_checks, 'valid': all(harness_checks.values()),
        'commands': len(commands), 'max_command_interval_s': float(np.max(intervals)) if len(intervals) else None,
        'max_moving_error_m': max((r['error_m'] for r in moving_rows), default=None),
        'max_static_error_m': max((r['error_m'] for r in static_rows), default=None),
        'max_actor_pose_gap_s': float(np.max(np.diff(pose_times))) if len(pose_times) > 1 else None}
    # --- 간격·접촉: 출발 전부터 정지 확인 끝까지 ---
    span = window_tuples(ego, t_green-1., last_time)
    sampled = sampled_gaps(span, actor, actor_cfg['length_m'], actor_cfg['width_m'], POSE_MAX_GAP_S)
    swept = swept_gap_lower_bounds(span, actor, actor_cfg['length_m'], actor_cfg['width_m'],
                                   1/actor_cfg['pose_publish_hz'], acceptance['swept_max_curvature_per_m'], POSE_MAX_GAP_S)
    lower = [row['lower_bound_m'] for row in swept if row['lower_bound_m'] is not None]
    hits = interval_contacts(contacts)
    result['gap'] = {
        'ego_samples': len(span), 'min_sampled_gap_m': _minimum(r['gap_m'] for r in sampled),
        'sampled_overlap_samples': sum(r['overlap_area_m2'] is not None and r['overlap_area_m2'] > 1e-9 for r in sampled),
        'sampled_missing_actor_samples': sum(r['gap_m'] is None for r in sampled),
        'sampled_finite_complete': bool(span) and len(sampled) == len(span) and all(
            _finite(r['gap_m']) and _finite(r['overlap_area_m2']) for r in sampled),
        'min_swept_lower_bound_m': min(lower) if lower else None,
        'swept_nonpositive_intervals': sum(v <= 0 for v in lower),
        'swept_missing_actor_intervals': sum(row['lower_bound_m'] is None for row in swept),
        'max_ego_sample_interval_s': float(np.max(np.diff([r[0] for r in span]))) if len(span) > 1 else None,
        'actor_contact_messages_with_ego': len(hits), 'first_contact_sim_s': hits[0] if hits else None,
        'scope': 'sampled gap + conservative swept lower bound between pose samples; event-only contact evidence is separate',
    }
    result['contact_evidence'] = {**(contact_evidence or {}), 'observed_messages': len(contacts),
        'observed_contact_events_with_ego': len(hits),
        'coverage': 'event-only; zero messages is not continuous no-contact proof',
        'limits': 'static set_pose actor is an infinite-mass teleported blocker; no vehicle dynamics or real safety claim'}
    # --- 노면: 기존 보수적 차체 외곽 기준 ---
    outside = None
    if road is not None:
        outside = sum(rectangle(x, y, yaw, EGO_LENGTH_M, EGO_WIDTH_M).difference(road).area > 1e-6
                      for _, x, y, yaw in window_tuples(ego, t_green, last_time))
    result['road'] = {'outside_samples': outside, 'scope': '20x15 cm body vs SDF road union, every ego pose sample'}
    # --- 진행·뒤따름 ---
    progress, jumps = progress_series(centerline, window_tuples(ego, t_green, last_time))
    actor_s = []
    for t, x, y, _ in actor:
        s, _ = centerline.nearest_s(x, y)
        actor_s.append((t, centerline.unwrap(s, s0)))
    behind, behind_samples = True, 0
    actor_times = [row[0] for row in actor_s]
    for t, x, y, _ in window_tuples(ego, t_green, last_time):
        index = _latest_index(actor_times, t)
        if index < 0:
            continue
        if t-actor_s[index][0] > POSE_MAX_GAP_S+1e-9:
            continue
        behind_samples += 1
        s, _ = centerline.nearest_s(x, y)
        behind &= centerline.unwrap(s, s0) <= actor_s[index][1]-(EGO_LENGTH_M+actor_cfg['length_m'])/2
    def progress_at(t):
        value = _value_at(progress, t, 'progress_m')
        return value if value is not None else 0.
    behind = bool(behind and behind_samples == len(window_tuples(ego, t_green, last_time)) and behind_samples > 0)
    gap_at_end = _value_at(sampled, t_end, 'gap_m')
    result['progress'] = {'at_window_end_m': progress_at(t_end), 'projection_jumps': jumps,
                          'ego_stays_behind_actor': behind, 'sampled_gap_at_window_end_m': gap_at_end}
    # --- 정지·재진행 (10 Hz 상태 표본) ---
    rows = [r for r in trace if r['t'] >= t_green]

    def stationary(row):
        held = row['ground_speed_mps'] < .01 and row['safe_speed_mps'] == 0
        return held and (mode not in ('g1', 'r0plus', 'm3') or row['autonomy'].get('behavior') == 'HOLD')
    case_metrics, gates = {}, {}
    rules = acceptance.get(case['name'], {})
    behavior = case.get('behavior_kind', case['name'])
    if behavior == 'stopped_center':
        hold = longest_true_interval(window(rows, t_green, t_end), stationary, max_gap_s=TRACE_MAX_GAP_S)
        case_metrics.update(longest_hold_s=hold)
        gates.update(hold=hold >= rules['minimum_continuous_hold_s'],
                     approached=gap_at_end is not None and gap_at_end <= rules['maximum_final_gap_m'])
    elif behavior == 'slow_lead':
        moved = [r['t'] for r in window(rows, t_green, t_end) if r['ground_speed_mps'] > .10]
        stall = (longest_true_interval(window(rows, moved[0], t_end), lambda r: r['ground_speed_mps'] < .05, max_gap_s=TRACE_MAX_GAP_S)
                 if moved else None)
        case_metrics.update(first_motion_sim_s=moved[0] if moved else None, longest_stationary_while_lead_moves_s=stall)
        gates.update(progress=progress_at(t_end) >= rules['minimum_progress_m'],
                     no_permanent_stall=stall is not None and stall <= rules['maximum_stationary_while_lead_moves_s'],
                     final_gap=gap_at_end is not None and gap_at_end <= rules['maximum_final_gap_m'])
    elif behavior == 'stop_restart':
        first, last = (t_green+v for v in rules['hold_window_s'])
        hold = longest_true_interval(window(rows, first, last), stationary, max_gap_s=TRACE_MAX_GAP_S)
        restart = [r['t'] for r in window(rows, last, t_end) if r['ground_speed_mps'] > rules['restart_speed_mps']]
        delay = restart[0]-last if restart else None
        after = progress_at(t_end)-progress_at(last)
        case_metrics.update(longest_hold_in_window_s=hold, restart_delay_s=delay, progress_after_restart_m=after,
                            restart_resolution_s=.10)
        gates.update(hold=hold >= rules['minimum_hold_s'],
                     restart=delay is not None and delay <= rules['maximum_restart_delay_s'],
                     progress_after_restart=after >= rules['minimum_progress_after_restart_m'],
                     final_gap=gap_at_end is not None and gap_at_end <= rules['maximum_final_gap_m'])
    elif behavior == 'side_occlusion':
        if case.get('route_kind') != 'side_grass_straight':
            raise ValueError('side behavior is restricted to registered off-road actors')
        actor_outside_road = road is not None and bool(actor) and all(
            rectangle(x, y, yaw, actor_cfg['length_m'], actor_cfg['width_m']).intersection(road).area <= 1e-6
            for _, x, y, yaw in window_tuples(actor, t_green, t_end))
        active = window_tuples(ego, t_green, t_end)
        signed = []
        for t, x, y, yaw in active:
            if progress_at(t) < rules['alignment_progress_m']:
                continue
            s, _ = centerline.nearest_s(x, y)
            cx, cy, heading = centerline.pose(s)
            signed.append((t, -(x-cx)*math.sin(heading)+(y-cy)*math.cos(heading)))
        moved = [r['t'] for r in window(rows, t_green, t_end) if r['ground_speed_mps'] > .10]
        stall = (longest_true_interval(window(rows, moved[0], t_end), lambda r: r['ground_speed_mps'] < .05,
                                     max_gap_s=TRACE_MAX_GAP_S) if moved else None)
        final_ego_s = centerline.unwrap(centerline.nearest_s(*active[-1][1:3])[0], s0) if active else None
        final_actor_s = s0+actor_travel_m(case['schedule'], case['duration_s'])
        maximum_lateral = max((abs(v) for _, v in signed), default=None)
        case_metrics.update(max_centerline_lateral_error_after_alignment_m=maximum_lateral,
                            lateral_samples_after_alignment=len(signed), longest_stationary_s=stall,
                            ego_final_ahead_m=final_ego_s-final_actor_s if final_ego_s is not None else None)
        gates.update(progress=progress_at(t_end) >= rules['minimum_progress_m'],
                     actor_stays_offroad=actor_outside_road,
                     no_permanent_stall=stall is not None and stall <= rules['maximum_stationary_s'],
                     keeps_original_line=maximum_lateral is not None and maximum_lateral <= rules['maximum_lateral_error_m'],
                     clears_side_actor=final_ego_s is not None and final_ego_s-final_actor_s >= rules['minimum_final_ahead_m'])
    else:
        raise ValueError('unknown case')
    if case.get('route_kind') == 'first_curve':
        # 단순히 시험 시간이 길었다는 이유로 코너 통과를 주장하지 않는다.
        # 사전 등록한 출구를 실제 ego가 지나고 방향도 바뀌었는지 정답으로만 평가한다.
        route_rows = window_tuples(ego, t_green, t_end)
        exit_s = centerline.unwrap(rules['required_exit_s_m'], s0)
        if exit_s < s0:
            exit_s += centerline.lap
        end_s = centerline.unwrap(centerline.nearest_s(*route_rows[-1][1:3])[0], s0) if route_rows else None
        turn = (abs(float(np.unwrap([r[3] for r in route_rows])[-1]-route_rows[0][3]))
                if route_rows else None)
        case_metrics.update(ego_end_unwrapped_s_m=end_s, required_exit_unwrapped_s_m=exit_s,
                            ego_net_heading_change_deg=math.degrees(turn) if turn is not None else None)
        gates.update(curve_exit_reached=end_s is not None and end_s >= exit_s,
                     curve_heading_changed=turn is not None and math.degrees(turn) >= rules['minimum_heading_change_deg'])
    gap = result['gap']
    gates.update(
        green_observed=True,
        data_coverage=all(c['valid'] for c in coverage.values()),
        sampled_gap=gap['sampled_finite_complete'] and gap['min_sampled_gap_m'] is not None and gap['min_sampled_gap_m'] >= acceptance['minimum_body_gap_m'],
        swept_gap=(gap['min_swept_lower_bound_m'] is not None and gap['swept_missing_actor_intervals'] == 0
                   and gap['min_swept_lower_bound_m'] >= acceptance['minimum_body_gap_m']),
        no_sampled_overlap=gap['sampled_finite_complete'] and gap['sampled_overlap_samples'] == 0,
        no_actor_contact=gap['actor_contact_messages_with_ego'] <= acceptance['actor_contact_messages_with_ego'],
        inside_road=outside is not None and outside <= acceptance['outside_road_samples'],
    )
    # 도로 밖 물체를 본래 주행선으로 지나가는 G1 가림 시험에만 뒤따름 조건이 적용되지 않는다.
    # 기존 앞차·코너 시험의 뒤따름 조건은 유지하며, 새 사례는 도로 유지/간격/실제 통과를 따로 검사한다.
    result['applicability'] = {'ego_stays_behind': 'not_applicable_offroad_actor' if behavior == 'side_occlusion' else 'required'}
    if behavior != 'side_occlusion':
        gates['ego_stays_behind'] = behind
    statuses = [r['autonomy'] for r in rows if r['autonomy'].get('behavior') is not None]
    result['controller_mode'] = {
        'behavior_rows': len(statuses),
        'g1_rows': sum(s.get('mode') == 'g1_follow' and s.get('passing_enabled') is False for s in statuses),
        'r0plus_rows': sum(s.get('mode') == 'r0_plus_candidate' and s.get('passing_enabled') is False for s in statuses),
        'm3_rows': sum(s.get('mode') == 'm3_execution_trial' and s.get('passing_enabled') is False
                       and s.get('simulation_trial_only') is True for s in statuses),
        'behavior_counts': {name: sum(s.get('behavior') == name for s in statuses) for name in ('CRUISE', 'FOLLOW', 'HOLD')},
        'hold_reasons': sorted({s.get('reason') for s in statuses if s.get('behavior') == 'HOLD'}),
    }
    result['case_metrics'] = case_metrics
    result['gates'] = gates
    return result


def window_tuples(rows, first, last):
    times = [row[0] for row in rows]
    return rows[bisect_left(times, first):bisect_right(times, last)]


def finalize_verdict(report, mode):
    """실행 유효성(하네스·무결성·종료)과 행동 판정을 나눠 기록한다."""
    evaluation = report.get('evaluation', {})
    harness = evaluation.get('harness', {})
    controller = evaluation.get('controller_mode', {})
    if mode not in ('g1', 'r0', 'r0plus', 'm3'):
        raise ValueError('unsupported controller mode')
    rows = controller.get('behavior_rows', 0)
    mode_matches = ((rows > 0 and controller.get('g1_rows') == rows) if mode == 'g1' else
                    (rows > 0 and controller.get('r0plus_rows') == rows) if mode == 'r0plus' else
                    (rows > 0 and controller.get('m3_rows') == rows) if mode == 'm3' else
                    controller.get('g1_rows', 0) == 0 and controller.get('r0plus_rows', 0) == 0
                    and controller.get('m3_rows', 0) == 0)
    integrity = {
        'no_runtime_error': 'error' not in report and 'error' not in evaluation,
        'evaluation_readers_valid': report.get('evaluation_readers', {}).get('valid') is True,
        'contact_source_instrumented': report.get('contact_evidence', {}).get('validated_source') is True,
        'sensor_only_controller_inputs': report.get('sensor_only_inputs') is True,
        'sensor_only_safety_inputs': report.get('safety_sensor_only_inputs') is True,
        'actuation_sensor_only_inputs': report.get('actuation', {}).get('sensor_only_inputs') is True,
        'actor_pose_only_evaluator': report.get('actor_pose_subscribers') == ['g1b_traffic_validator'],
        'drive_publisher_controller_only': report.get('drive_publishers') == ['local_pursuit'],
        'safe_drive_publisher_safety_only': report.get('safe_drive_publishers') == ['lidar_safety'],
        'sequential_acquisition': report.get('sequential_acquisition_verified') is True,
        'lidar_interface': report.get('lidar_nominal_interface_passed') is True,
        'source_unchanged': report.get('source_unchanged') is True,
        'signal_sequence': {'red', 'yellow', 'green'}.issubset(report.get('signal_states', [])),
        'no_false_start': report.get('false_start') is False,
        'controller_mode_matches': mode_matches,
    }
    stop = report.get('stop', {})
    actuation = report.get('actuation', {})
    stop_checks = {
        'model_stop_stable': stop.get('stable') is True,
        'dense_pose_stop_evidence': (stop.get('pose_evidence') or {}).get('passed') is True,
        'explicit_mcu_stop': report.get('actuation_stop_passed') is True,
        'no_mcu_latch_before_stop': ((actuation.get('stop_latch_timing') or {}).get('valid') is True
                                     and actuation.get('latched_before_stop_samples', 1) == 0 and report.get('abort') is None),
        'no_deadline_diagnostic': report.get('actuation_diagnostics', {}).get('control_deadline_samples', 1) == 0,
        'no_wall_collision': report.get('wall_collision_samples') == 0 and stop.get('wall_collision_samples') == 0,
    }
    behavior = dict(evaluation.get('gates', {}))
    report['harness_valid'] = harness.get('valid') is True
    report['integrity'] = integrity
    report['stop_checks'] = stop_checks
    report['gates'] = behavior
    report['run_valid'] = report['harness_valid'] and all(integrity.values()) and report.get('shutdown_clean') is True
    report['behavior_gates_passed'] = bool(behavior) and all(behavior.values()) and all(stop_checks.values())
    if mode in ('g1', 'r0plus', 'm3'):
        report['passed'] = report['run_valid'] and report['behavior_gates_passed']
    else:
        # R0는 비교 기준이다. 실행 유효성만 통과 조건이며 G1 판정선 충족 여부는 따로 남긴다.
        report['passed'] = report['run_valid']
        report['r0_meets_g1_gates'] = report['behavior_gates_passed']
    return report
