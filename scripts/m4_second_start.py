"""M4 격리 시험 전용 두 번째 렌즈 신호. 제어기 출발 사실/주행 명령을 만들지 않는다."""
import math
import time

from m4_operation_trial import TrialError


class SecondStartTrial:
    def __init__(self):
        self.phase = 'wait_rearm'
        self.pending = None
        self.identity = self.boot = None
        self.begin_s = self.red_s = self.green_s = self.done_s = None
        self.green_request_s = None
        self.events = []
        self.failure = None

    def fail(self, reason):
        self.failure = reason
        raise TrialError(reason)

    def request(self, color, now):
        self.pending = {'color': color, 'time_s': now}
        if color == 'green':
            self.green_request_s = now
        self.events.append({'event': 'request', **self.pending})
        return dict(self.pending)

    def complete(self, response, now):
        if not self.pending:
            self.fail('signal_response_without_request')
        self.events.append({'event': 'response', 'color': self.pending['color'], 'time_s': now,
                            'response': response})
        if response.get('success') is not True:
            self.fail('signal_service_refused')
        self.phase = 'await_'+self.pending['color']
        self.pending = None

    def tick(self, now, operator, race, mcu, light):
        if self.failure:
            self.fail(self.failure)
        if not math.isfinite(now) or now <= 0:
            return None
        if operator.get('phase') != 'done':
            return None
        if operator.get('new_race') is not True or operator.get('failure'):
            self.fail('second_start_requires_new_race_wait')
        if self.begin_s is None:
            self.begin_s = now
            self.identity = tuple(operator['identity'])
            self.boot = operator['boot']
        if (not race or (race.get('session'), race.get('race_epoch'), race.get('vision_epoch')) != self.identity
                or mcu.get('boot') != self.boot or mcu.get('state') not in ('ARMED', 'ACTIVE')
                or race.get('latch') is not None or race.get('rearm_required') is not False
                or race.get('intervention') != operator['token']):
            self.fail('second_start_identity_or_permission_changed')
        if self.green_request_s is None and (race.get('started') or race.get('visual_start_fact')
                                             or race.get('policy_allowed_if_sensors_ready')):
            self.fail('second_start_before_new_signal')
        if self.phase == 'done':
            return None
        if now-self.begin_s > 12.:
            self.fail('second_start_deadline')
        if self.pending:
            if now-self.pending['time_s'] > 2.:
                self.fail('signal_response_timeout')
            return None
        stamp = light.get('sim_time_s')
        if (not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or not 0 <= now-stamp <= .5
                or light.get('error')):
            return None
        if self.phase == 'wait_rearm' and now-self.begin_s >= 1.5:
            return self.request('red', now)
        if self.phase == 'await_red' and light.get('applied') == 'red':
            requested = next(e['time_s'] for e in self.events if e['event'] == 'request' and e['color'] == 'red')
            if stamp >= requested:
                self.red_s, self.phase = now, 'red_hold'
                self.events.append({'event': 'applied', 'color': 'red', 'time_s': now, 'source_time_s': stamp})
        if self.phase == 'red_hold':
            if light.get('applied') != 'red':
                self.fail('second_red_hold_interrupted')
            if now-self.red_s >= 2.:
                return self.request('green', now)
        if self.phase == 'await_green' and light.get('applied') == 'green' and stamp >= self.green_request_s:
            self.green_s, self.phase = now, 'wait_pixels'
            self.events.append({'event': 'applied', 'color': 'green', 'time_s': now, 'source_time_s': stamp})
        if self.phase == 'wait_pixels' and (race.get('started') is True and race.get('visual_start_fact') is True
                                            and race.get('policy_allowed_if_sensors_ready') is True):
            self.done_s, self.phase = now, 'done'
            self.events.append({'event': 'visual_start', 'time_s': now, 'race': dict(race)})
        return None

    def snapshot(self):
        return dict(vars(self))


class RosSecondStart:
    def __init__(self, node, Trigger):
        self.logic = SecondStartTrial()
        self.Trigger = Trigger
        self.services = {color: node.create_client(Trigger, '/sim/traffic_light/set_'+color)
                         for color in ('red', 'green')}
        self.future = None
        self.sent_wall = None

    def tick(self, now, operator, race, mcu, light):
        if self.sent_wall is not None and time.monotonic()-self.sent_wall > 15.:
            self.logic.fail('signal_service_wall_timeout')
        if self.future is not None and self.future.done():
            result = self.future.result()
            self.logic.complete({'success': result.success, 'message': result.message}, now)
            self.future = self.sent_wall = None
        action = self.logic.tick(now, operator, race, mcu, light)
        if action:
            service = self.services[action['color']]
            if not service.service_is_ready():
                self.logic.fail('signal_service_not_ready')
            self.sent_wall = time.monotonic()
            self.future = service.call_async(self.Trigger.Request())

    def snapshot(self):
        return self.logic.snapshot()


def evaluate_second_start(trial, report, ego, actuation):
    """새 초록 요청 전 대기와 이후 실제 출발을 구분한다. 원본 판정/입력은 수정하지 않는다."""
    import copy
    import g1_traffic_eval as ev
    from m4_operation_eval import evaluate_trial

    signal = trial.get('second_start') or {}
    green_request, green_seen = signal.get('green_request_s'), signal.get('green_s')
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (green_request, green_seen)):
        return {'passed': False, 'reason': 'second_green_not_applied', 'signal_failure': signal.get('failure')}
    # 기존 부정 시험의 판정 함수는 그대로 사용하되 명시적으로 새 신호 이전 구간에 한정한다.
    prefix_trial = copy.deepcopy(trial)
    prefix_trial['samples'] = [s for s in trial['samples'] if s['time_s'] <= green_request]
    prefix_report = copy.deepcopy(report)
    prefix_report['case_window']['end_sim_s'] = green_request
    prefix = evaluate_trial(prefix_trial, prefix_report, [p for p in ego if p[0] <= green_request], actuation)
    events = signal.get('events', [])
    requests = [e for e in events if e['event'] == 'request']
    responses = [e for e in events if e['event'] == 'response']
    applied = [e for e in events if e['event'] == 'applied']
    visuals = [e for e in events if e['event'] == 'visual_start']
    end = report['case_window']['end_sim_s']
    segments = ev.kinematic_speed_segments(ego)
    moving = [s for s in segments if green_seen <= s['t0'] and s['t1'] <= end and
              s['average_speed_mps'] >= .1 and s['t1']-s['t0'] <= ev.POSE_MAX_GAP_S]
    delay = moving[0]['t1']-green_seen if moving else None
    covered = ev.pose_window_coverage(segments, green_seen, end)
    after = [p for p in ego if green_seen <= p[0] <= end]
    progress = math.dist(after[0][1:3], after[-1][1:3]) if len(after) > 1 else 0.
    identity = tuple(trial['operator']['identity'])
    samples = [s for s in trial['samples'] if green_seen <= s['time_s'] <= end]
    sample_coverage = ev.stream_coverage([s['time_s'] for s in samples], green_seen, end, .05)
    red_time = signal.get('red_s')
    event_times = [e.get('time_s') for e in events]
    timing_consistent = (len(requests) == len(responses) == len(applied) == 2 and len(visuals) == 1 and
        all(isinstance(t, (int, float)) and math.isfinite(t) for t in event_times) and
        all(a <= b for a, b in zip(event_times, event_times[1:])) and
        green_request == requests[1].get('time_s') and green_seen == applied[1].get('time_s') and
        red_time == applied[0].get('time_s') and signal.get('done_s') == visuals[0].get('time_s'))
    gates = {
        'new_race_wait_before_second_signal': prefix['passed'],
        'second_signal_completed': signal.get('phase') == 'done' and signal.get('failure') is None,
        'one_red_one_green_request': [e.get('color') for e in requests] == ['red', 'green'],
        'both_services_acknowledged': [e.get('color') for e in responses] == ['red', 'green'] and
            all(e.get('response', {}).get('success') is True for e in responses),
        'both_colors_applied': [e.get('color') for e in applied] == ['red', 'green'],
        'signal_event_timing_consistent': timing_consistent,
        'red_held_before_green': isinstance(red_time, (int, float)) and green_request-red_time >= 2.,
        'new_generation_visual_start': len(visuals) == 1 and visuals[0]['time_s'] >= green_seen and
            (visuals[0]['race'].get('session'), visuals[0]['race'].get('race_epoch'),
             visuals[0]['race'].get('vision_epoch')) == identity and
            visuals[0]['race'].get('visual_start_fact') is True and visuals[0]['race'].get('started') is True,
        'second_start_status_covered': sample_coverage['valid'] and bool(samples) and
            all(abs(s['time_s']-s['autonomy_stamp_s']) < .25 and
                (s['race'].get('session'), s['race'].get('race_epoch'), s['race'].get('vision_epoch')) == identity
                for s in samples),
        'actual_second_start': covered['complete'] and delay is not None and delay <= 2.5,
        'actual_second_progress': progress >= 1.,
    }
    return {'passed': all(gates.values()), 'gates': gates, 'test_kind': 'new_race_second_visual_start',
        'original_traffic_passed': report.get('passed'), 'prefix_scope_end_s': green_request,
        'pre_signal_wait': prefix, 'green_applied_observed_s': green_seen, 'restart_delay_s': delay,
        'progress_after_second_green_m': progress, 'second_pose_coverage': covered,
        'second_status_coverage': sample_coverage,
        'scope': 'temporary red/green rendering and actual motion; not official four-red, grid, or hardware proof'}
