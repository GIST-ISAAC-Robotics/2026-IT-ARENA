"""격리 Gazebo 시험에서만 운영자의 고정 STOP/확인/CLEAR/ARM 순서를 재현한다.

실제 운영용 자동 승인기가 아니다. --race-operation-test에서만 생성하며 지도/정답 자세를
받지 않는다. 실제 이동/정지는 m4_operation_eval의 사후 평가로 별도 판정한다.
"""
import json
import math
import time


class TrialError(RuntimeError):
    pass


class OperatorTrial:
    def __init__(self, new_race=False):
        self.new_race = new_race
        self.initial_identity = None
        self.phase = 'initial'
        self.pending = None
        self.events = []
        self.identity = None
        self.token = None
        self.boot = None
        self.first_s = None
        self.initial_arm_ok = False
        self.stop_request_s = None
        self.stop_identity = None
        self.stop_closed_s = None
        self.stop_ack_s = None
        self.ack_s = None
        self.clear_seen_s = None
        self.rearm_s = None
        self.done_s = None
        self.failure = None

    def fail(self, reason):
        self.failure = reason
        raise TrialError(reason)

    def emit(self, kind, now, race):
        action = dict(kind=kind, time_s=now, request=f'm4-eval-{len(self.events)}')
        if kind in ('initial_ack', 'resume_ack'):
            action['payload'] = dict(v=1, request=action['request'], op='acknowledge',
                session=race['session'], race_epoch=race['race_epoch'], intervention=self.token,
                hazards_cleared=True, operator_reviewed=True)
        elif kind == 'new_race':
            action['payload'] = dict(v=1, request=action['request'], op='new_race',
                                    session=race['session'], race_epoch=race['race_epoch'])
        elif kind == 'policy_stop':
            action['payload'] = dict(v=1, request=action['request'], op='stop')
        if kind == 'mcu_stop':
            self.stop_request_s = now
        if kind == 'rearm':
            self.rearm_s = now
        self.pending = action
        self.events.append(dict(event='request', **action))
        return action

    def complete(self, response, now):
        if not self.pending:
            self.fail('response_without_pending')
        a = self.pending
        self.events.append(dict(event='response', kind=a['kind'], time_s=now, response=response))
        if 'payload' in a:
            p = a['payload']
            if (response.get('request') != p['request'] or response.get('op') != p['op']
                    or response.get('accepted') is not True):
                self.fail('race_response_rejected_or_mismatched')
            expected = (self.identity[0], self.identity[1]+1) if a['kind'] == 'new_race' else self.identity[:2]
            if (response.get('session'), response.get('race_epoch')) != expected:
                self.fail('race_response_session_changed')
            if a['kind'] in ('policy_stop', 'new_race'):
                expected_latch = 'new_race' if a['kind'] == 'new_race' else 'explicit_stop'
                if response.get('latch') != expected_latch or response.get('intervention', 0) <= self.token:
                    self.fail('policy_stop_token_invalid')
                self.token = response['intervention']
                if a['kind'] == 'new_race':
                    self.identity = (expected[0], expected[1], self.identity[2]+1)
            elif response.get('intervention') != self.token or response.get('latch') is not None:
                self.fail('acknowledgement_token_invalid')
        elif response.get('success') is not True:
            self.fail('MCU_service_refused:'+str(response))
        self.pending = None
        transitions = dict(initial_ack='initial_arm', initial_arm='running', policy_stop='stop_mcu', new_race='stop_mcu',
                           mcu_stop='stop_hold', resume_ack='ack_hold', clear_stop='clear_hold', rearm='resume')
        self.phase = transitions[a['kind']]
        if a['kind'] == 'initial_arm':
            self.initial_arm_ok = True
        elif a['kind'] == 'mcu_stop':
            self.stop_ack_s = now
        elif a['kind'] == 'resume_ack':
            self.ack_s = now

    def tick(self, now, race, mcu, green_s):
        if self.failure:
            self.fail(self.failure)
        if not math.isfinite(now) or now <= 0 or not race or not mcu:
            return None
        self.first_s = now if self.first_s is None else self.first_s
        if self.identity is None:
            if race.get('mode') != 'b3_candidate' or race.get('latch') != 'process_start':
                self.fail('expected_B3_process_start_latch')
            self.identity = (race['session'], race['race_epoch'], race['vision_epoch'])
            self.initial_identity = self.identity
            self.token, self.boot = race['intervention'], mcu['boot']
        incoming_identity = (race.get('session'), race.get('race_epoch'), race.get('vision_epoch'))
        awaiting_new = self.pending and self.pending['kind'] == 'new_race' and incoming_identity == (
            self.identity[0], self.identity[1]+1, self.identity[2]+1)
        # 명령 응답이 0.2초 주기 상태보다 먼저 올 수 있다. 이 제한 전이에서는 이전 상태를
        # 새 허가에 쓰지 않고 새 세대 상태를 기다린다. MCU 고장 검사와 전체 기한은 유지한다.
        awaiting_new_status = self.new_race and self.phase == 'stop_mcu' and incoming_identity == self.initial_identity
        if incoming_identity != self.identity and not (awaiting_new or awaiting_new_status):
            self.fail('race_or_vision_epoch_changed')
        if mcu.get('boot') != self.boot:
            self.fail('MCU_boot_changed')
        if mcu.get('state') == 'FAULT_LATCHED':
            self.fail('unexpected_MCU_fault:'+str(mcu.get('reason')))
        if mcu.get('state') == 'ESTOP_LATCHED':
            key = (mcu.get('boot'), mcu.get('generation'))
            if (self.stop_request_s is None or self.stop_closed_s is not None or
                    mcu.get('reason') != 'software_stop' or mcu.get('estop_held') is not False or
                    mcu.get('now_us', -1)*1e-6 < self.stop_request_s or
                    (self.stop_identity is not None and key != self.stop_identity)):
                self.fail('unexpected_MCU_STOP')
            self.stop_identity = key
        if self.pending:
            if now-self.pending['time_s'] > 2.:
                self.fail('operation_response_timeout')
            return None
        if self.phase not in ('initial', 'stop_mcu') and race.get('intervention') != self.token:
            self.fail('new_intervention_not_authorized')
        if self.phase == 'initial':
            if (now-self.first_s >= 1. and race.get('rearm_baseline') and
                    mcu.get('state') == 'DISARMED' and mcu.get('stationary_ready') and mcu.get('output_ready')):
                return self.emit('initial_ack', now, race)
        elif self.phase == 'initial_arm':
            if race.get('latch') is None:
                return self.emit('initial_arm', now, race)
        elif self.phase == 'running':
            if green_s is not None and now-green_s >= 2.:
                if not (race.get('started') and race.get('visual_start_fact') and
                        race.get('policy_allowed_if_sensors_ready') and not race.get('rearm_required')):
                    self.fail('not_running_at_registered_stop_time')
                return self.emit('new_race' if self.new_race else 'policy_stop', now, race)
        elif self.phase == 'stop_mcu':
            if race.get('latch') == ('new_race' if self.new_race else 'explicit_stop') and race.get('intervention') == self.token:
                return self.emit('mcu_stop', now, race)
        elif self.phase == 'stop_hold':
            if now-self.stop_ack_s >= 1.5 and mcu.get('stationary_ready') and self.stop_identity:
                if self.new_race and (race.get('visual_start_fact') or race.get('started')):
                    self.fail('new_race_reused_old_visual_start')
                if not self.new_race and (not race.get('visual_start_fact') or not race.get('started')):
                    self.fail('same_race_visual_fact_lost')
                return self.emit('resume_ack', now, race)
        elif self.phase == 'ack_hold':
            if now-self.ack_s >= .5 and race.get('latch') is None:
                return self.emit('clear_stop', now, race)
        elif self.phase == 'clear_hold':
            if mcu.get('state') == 'DISARMED':
                if self.clear_seen_s is None:
                    self.clear_seen_s = self.stop_closed_s = now
                if now-self.clear_seen_s >= .5 and mcu.get('stationary_ready') and mcu.get('output_ready'):
                    return self.emit('rearm', now, race)
        elif self.phase == 'resume':
            if self.new_race:
                if race.get('started') or race.get('visual_start_fact') or race.get('policy_allowed_if_sensors_ready'):
                    self.fail('new_race_moved_without_new_signal')
                if (mcu.get('state') in ('ARMED', 'ACTIVE') and not race.get('rearm_required') and
                        race.get('policy_reason') == 'wait_visual_start'):
                    self.done_s, self.phase = now, 'done'
                    self.events.append(dict(event='rearm_witness', time_s=now, race=race))
                return None
            if (mcu.get('state') in ('ARMED', 'ACTIVE') and not race.get('rearm_required') and
                    race.get('policy_allowed_if_sensors_ready') and race.get('started') and
                    race.get('visual_start_fact')):
                self.done_s, self.phase = now, 'done'
                self.events.append(dict(event='rearm_witness', time_s=now, race=race))
        if self.phase != 'done' and green_s is not None and now-green_s > 10.:
            self.fail('operator_sequence_deadline')
        return None

    def snapshot(self):
        return {key: value for key, value in vars(self).items() if key != 'pending'}


class RosOperatorTrial:
    """시험기 소유 노드만 사용. 주행/보호 토픽에는 발행자를 만들지 않는다."""
    def __init__(self, node, Trigger, String, new_race=False):
        self.logic = OperatorTrial(new_race=new_race)
        self.node, self.Trigger, self.String = node, Trigger, String
        self.publisher = node.create_publisher(String, '/race/command', 10)
        self.subscription = node.create_subscription(String, '/race/command_result', self.receive, 20)
        self.services = {name: node.create_client(Trigger, '/actuation/'+name)
                         for name in ('arm', 'stop', 'clear_stop')}
        self.future = None
        self.responses = []
        self.samples = []
        self.now = 0.
        self.sent_wall = None

    def receive(self, message):
        response = json.loads(message.data)
        self.responses.append(response)
        pending = self.logic.pending
        if pending and 'payload' in pending and response.get('request') == pending['request']:
            try:
                self.logic.complete(response, self.now)
            except TrialError:
                # 다음 tick에서 시험 루프가 중단하고 기존 마지막 STOP 경로로 간다.
                pass
            self.sent_wall = None

    def tick(self, now, race, mcu, green, safe_speed, autonomy_stamp):
        self.now = now
        if self.sent_wall is not None and time.monotonic()-self.sent_wall > 15.:
            self.logic.fail('operation_wall_timeout')
        if self.future is not None and self.future.done():
            result = self.future.result()
            self.logic.complete(dict(success=result.success, message=result.message), now)
            self.future, self.sent_wall = None, None
        if not self.samples or now-self.samples[-1]['time_s'] >= .02:
            self.samples.append(dict(time_s=now, race=race, mcu=dict(mcu), safe_speed=safe_speed,
                                     autonomy_stamp_s=autonomy_stamp))
        if not all(s.service_is_ready() for s in self.services.values()):
            return
        action = self.logic.tick(now, race, mcu, green)
        if not action:
            return
        self.sent_wall = time.monotonic()
        if 'payload' in action:
            message = self.String()
            message.data = json.dumps(action['payload'])
            self.publisher.publish(message)
        else:
            service = {'initial_arm': 'arm', 'rearm': 'arm', 'mcu_stop': 'stop', 'clear_stop': 'clear_stop'}[action['kind']]
            self.future = self.services[service].call_async(self.Trigger.Request())

    def snapshot(self):
        return dict(operator=self.logic.snapshot(), samples=self.samples, responses=self.responses,
                    scope='evaluation-only scripted operator; not production auto-acknowledgement')
