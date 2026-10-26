"""M4 중간 STOP/같은 경기 재개 사후 평가. 정답은 이 평가 함수에서만 사용한다."""
import math

import g1_traffic_eval as ev


ORDER = ['initial_ack', 'initial_arm', 'policy_stop', 'mcu_stop', 'resume_ack', 'clear_stop', 'rearm']


def evaluate_trial(trial, report, ego, actuation):
    operator, samples = trial['operator'], trial['samples']
    new_race = operator.get('new_race') is True
    order = ['new_race' if k == 'policy_stop' else k for k in ORDER] if new_race else ORDER
    requests = [e for e in operator['events'] if e['event'] == 'request']
    responses = [e for e in operator['events'] if e['event'] == 'response']
    result = dict(passed=False, original_traffic_passed=report.get('passed'),
        scope='registered intermediate STOP only; original traffic verdict is preserved; no hardware claim')
    if [e['kind'] for e in requests] != order or len(responses) != len(order) or not ego:
        return {**result, 'reason': 'incomplete_operator_sequence', 'operator_phase': operator['phase'],
                'failure': operator.get('failure')}
    times = {e['kind']: e['time_s'] for e in requests}
    replies = {e['kind']: e['time_s'] for e in responses}
    stop, ack, rearm = times['mcu_stop'], replies['resume_ack'], times['rearm']
    segments = ev.kinematic_speed_segments(ego)
    before = [s for s in segments if stop-.2 <= s['t0'] and s['t1'] <= stop]
    before_cover = ev.pose_window_coverage(segments, stop-.2, stop)
    stop_pose = ev.pose_window_coverage(segments, ack-.5, ack)
    waiting_pose = ev.pose_window_coverage(segments, ack, rearm)
    # 한 표본의 속도 튐 대신 밀집 자세 구간의 실제 이동을 요구한다.
    moving_before = bool(before) and sum((s['t1']-s['t0'])*s['average_speed_mps'] for s in before) >= .02
    resume_segments = [s for s in segments if s['t0'] >= rearm and s['average_speed_mps'] >= .1
                       and s['t1']-s['t0'] <= ev.POSE_MAX_GAP_S]
    resume_delay = resume_segments[0]['t1']-rearm if resume_segments else None
    resume_cover = ev.pose_window_coverage(segments, rearm, min(rearm+2.5, ego[-1][0]))
    end = report.get('case_window', {}).get('end_sim_s', ego[-1][0])
    after = [p for p in ego if rearm <= p[0] <= end]
    displacement = math.dist(after[0][1:3], after[-1][1:3]) if len(after) > 1 else 0.
    waiting = [s for s in samples if ack+.03 <= s['time_s'] <= rearm-.03]
    cover = ev.stream_coverage([s['time_s'] for s in waiting], ack+.03, rearm-.03, .05)
    same = tuple(operator['identity'])
    same_epoch = bool(samples) and all((s['race'].get('session'), s['race'].get('race_epoch'),
        s['race'].get('vision_epoch')) == same for s in samples)
    preserved = [s for s in samples if replies['new_race' if new_race else 'policy_stop']+.1 <= s['time_s'] <= end]
    before_final = []
    unexpected = []
    final_us = report.get('stop_request_clock_ns')
    final_us = final_us//1000 if type(final_us) is int else None
    stop_identity = tuple(operator['stop_identity'] or ())
    # 자차 pose 수신 시각은 MCU의 원본 시각보다 뒤처질 수 있다. CLEAR 경계는 동일한
    # MCU 스트림에서 관측한 더 높은 세대의 DISARMED로 정한다. 같은 us의 STOP→CLEAR도
    # 수신 순서와 generation으로 구별하며, CLEAR 뒤 옛 STOP은 제외하지 않는다.
    clear_index = next((i for i,row in enumerate(actuation)
        if len(stop_identity) == 2 and row.get('boot') == stop_identity[0]
        and row.get('state') == 'DISARMED' and row.get('generation', -1) > stop_identity[1]
        and type(row.get('now_us')) is int and row['now_us'] >= round(times['clear_stop']*1e6)), None)
    for index,row in enumerate(actuation):
        if type(row.get('now_us')) is not int:
            unexpected.append(row)
            continue
        if row.get('state') not in ev.LATCHED_STATES:
            continue
        final = (final_us is not None and row['now_us'] >= final_us and
                 row.get('state') == 'ESTOP_LATCHED' and row.get('reason') == 'software_stop')
        intermediate = (row.get('state') == 'ESTOP_LATCHED' and row.get('reason') == 'software_stop'
            and row.get('estop_held') is False
            and (row.get('boot'), row.get('generation')) == stop_identity
            and row['now_us'] >= round(stop*1e6) and clear_index is not None and index < clear_index)
        if not (final or intermediate):
            unexpected.append(row)
        if not final:
            before_final.append(row)
    stop_checks = {k:v for k,v in report.get('stop_checks', {}).items() if k != 'no_mcu_latch_before_stop'}
    gates = dict(
        sequence_complete=operator['phase'] == 'done' and operator.get('failure') is None,
        exact_requests=[e['kind'] for e in responses] == order,
        actual_motion_before_stop=before_cover['complete'] and moving_before,
        intermediate_dense_stop=stop_pose['passed'] and .5 <= ack-stop <= 6.,
        ack_and_clear_do_not_move=waiting_pose['passed'] and cover['valid'] and bool(waiting) and
            all(s['safe_speed'] == 0. and s['race'].get('policy_allowed_if_sensors_ready') is False
                and s['race'].get('rearm_required') is True for s in waiting),
        # 운영 상태는 변화 시 또는 0.2초마다 발행한다. 평가 수신 정합성에 0.05초 여유를 둔다.
        # 제어기 내부 MCU/센서 신선도 한도를 바꾸는 값이 아니다.
        wait_status_fresh=bool(waiting) and all(abs(s['time_s']-s['autonomy_stamp_s']) < .25 for s in waiting),
        same_race_and_vision_epoch=same_epoch,
        visual_fact_preserved=bool(preserved) and all(s['race'].get('visual_start_fact') is True
            and s['race'].get('started') is True for s in preserved),
        mcu_rearm_witnessed=operator.get('done_s') is not None and operator['done_s'] >= rearm,
        actual_restart=resume_cover['complete'] and resume_delay is not None and resume_delay <= 2.5,
        progress_after_rearm=displacement >= 1.,
        only_registered_intermediate_latch=bool(before_final) and clear_index is not None and not unexpected,
        original_integrity=report.get('run_valid') is True,
        original_traffic_behavior=bool(report.get('gates')) and all(report['gates'].values()),
        original_final_stop= len(stop_checks) == 5 and all(stop_checks.values()) and report.get('abort') is None)
    if new_race:
        # 새 경기 시험은 움직이지 않음을 판정한다. 원본 추종 HOLD/진행 gate는 고치지 않는다.
        for key in ('same_race_and_vision_epoch', 'visual_fact_preserved', 'actual_restart',
                    'progress_after_rearm', 'original_traffic_behavior'):
            del gates[key]
        after_arm = [s for s in samples if rearm+.1 <= s['time_s'] <= end]
        stationary_after_arm = ev.pose_window_coverage(segments, rearm, end)
        initial = tuple(operator['initial_identity'])
        safety_names = ('green_observed','data_coverage','sampled_gap','swept_gap','no_sampled_overlap',
                        'no_actor_contact','inside_road','ego_stays_behind')
        gates.update(
            race_and_vision_advanced_once=(same == (initial[0], initial[1]+1, initial[2]+1) and
                bool(preserved) and all((s['race'].get('session'),s['race'].get('race_epoch'),
                    s['race'].get('vision_epoch')) == same for s in preserved)),
            old_start_fact_not_reused=bool(preserved) and all(s['race'].get('started') is False
                and s['race'].get('visual_start_fact') is False for s in preserved),
            armed_but_waiting_for_new_signal=bool(after_arm) and all(s['race'].get('rearm_required') is False
                and s['race'].get('policy_reason') == 'wait_visual_start'
                and s['race'].get('policy_allowed_if_sensors_ready') is False
                and s['mcu'].get('state') in ('ARMED','ACTIVE') and s['safe_speed'] == 0. for s in after_arm),
            wait_for_new_signal_covered=ev.stream_coverage([s['time_s'] for s in after_arm], rearm+.1,end,.05)['valid'],
            actual_no_motion_without_new_signal=stationary_after_arm['passed'] and end-rearm >= 1.,
            original_geometry_and_coverage=all(report.get('gates',{}).get(k) is True for k in safety_names))
        result['after_arm_stationary'] = stationary_after_arm
        result['test_kind'] = 'new_race_waits_for_new_visual_start'
    result.update(gates=gates, passed=all(gates.values()),
        intermediate_stop=stop_pose, before_stop_motion=dict(covered=before_cover['complete'],
            actual_motion=moving_before, maximum_speed_mps=before_cover['maximum_speed_mps'],
            window_s=[stop-.2,stop]), waiting_pose=waiting_pose,
        restart_delay_s=resume_delay, progress_after_rearm_m=displacement,
        clear_transition=None if clear_index is None else dict(index=clear_index,
            source_us=actuation[clear_index]['now_us'], generation=actuation[clear_index]['generation']),
        waiting_status_coverage=cover, unexpected_latches=unexpected[:5],
        registered_intermediate_latched_samples=len(before_final), times=times, response_times=replies)
    return result
