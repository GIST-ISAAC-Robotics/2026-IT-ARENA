"""B3 선택형 운영 연결: 경기 허가·영상 세대·MCU 상태를 한 호출 지점에서 묶는다.

ROS 발행이나 MCU 명령을 하지 않는다. 사람의 해제는 상위 정책 래치만 풀며
MCU STOP/CLEAR/ARM을 대신하거나 자동 ARM하지 않는다. 목표 0·모터 평균 0은
실제 차량 정지의 증명이 아니다. 상위 래치는 프로세스 수명 동안만 유지된다. 프로세스 시작도
래치로 시작해 이전 ARMED를 재사용하지 않지만, 실제 정지 유지는 MCU STOP으로 별도 확보해야 한다.

같은 경기 복구: `recovery_reset`은 출발 사실을 보존하고 입력 이력만 버린다. 일반 명시 STOP 뒤에는
별도 RESET 버튼 없이 START할 수 있다. `resume_start`는
새 입력 경계를 만들 뿐 주행 허가가 아니며, 경계 뒤 센서·경로·MCU 재ARM이 모두 갖춰질 때까지
래치를 유지한다. 취소된 START는 큐에 남기지 않는다. new_race/legacy_reset과 섞지 않는다.
"""
from collections import OrderedDict, deque
import math
import uuid

from arena_autonomy.race_permission import RacePermission

ARMED_STATES = ('ARMED', 'ACTIVE')
LATCHED_STATES = ('ESTOP_LATCHED', 'FAULT_LATCHED')
KNOWN_STATES = ('DISARMED',) + ARMED_STATES + LATCHED_STATES
COMMAND_KEYS = {
    'new_race': {'v', 'request', 'op', 'session', 'race_epoch'},
    'acknowledge': {'v', 'request', 'op', 'session', 'race_epoch', 'intervention',
                    'hazards_cleared', 'operator_reviewed'},
    'recovery_reset': {'v', 'request', 'op', 'session', 'race_epoch', 'intervention'},
    'resume_start': {'v', 'request', 'op', 'session', 'race_epoch', 'intervention'},
    'cancel_start': {'v', 'request', 'op', 'session', 'race_epoch', 'start_request', 'reason'},
    'restore_race': {'v', 'request', 'op', 'session', 'race_epoch', 'intervention', 'race_id', 'side',
                     'same_race_confirmed'},
}
RESUME_INPUTS = ('scan', 'rgb', 'gyro', 'encoder')


def _request_id(value):
    return type(value) is str and 1 <= len(value) <= 64


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class McuObserver:
    """`/actuation/status`의 (boot, generation) 순서와 수신 신선도를 판정한다.

    가상 MCU의 `now_us`는 같은 /clock 기반이라는 시험 가정으로 수신 시각과 대조한다.
    실물 MCU는 host가 MCU 시계를 변환해 중계해야 하며 이 관측기는 그 변환을 하지 않는다.
    """
    def __init__(self, *, max_age_s=.1, wall_timeout_s=3., max_skew_s=.1, retired_limit=8):
        if not all(_finite(v) and v > 0 for v in (max_age_s, wall_timeout_s, max_skew_s)):
            raise ValueError('MCU status limits must be finite and positive')
        self.max_age_s, self.wall_timeout_s, self.max_skew_s = max_age_s, wall_timeout_s, max_skew_s
        self.latest = None
        self.retired = deque(maxlen=retired_limit)
        self.counts = dict(accepted=0, invalid=0, late=0, retired_boot=0, clock_mismatch=0, boot_changes=0)

    def identity(self):
        return None if self.latest is None else (self.latest['boot'], self.latest['generation'])

    def update(self, status, now_s, wall_s):
        valid = (type(status) is dict and status.get('v') == 1 and _request_id(status.get('boot')) and
                 type(status.get('generation')) is int and status['generation'] >= 0 and
                 status.get('state') in KNOWN_STATES and type(status.get('now_us')) is int and
                 status['now_us'] >= 0 and type(status.get('estop_held')) is bool and
                 _finite(now_s) and _finite(wall_s))
        if not valid:
            self.counts['invalid'] += 1
            return 'invalid'
        boot, generation, source_s = status['boot'], status['generation'], status['now_us']*1e-6
        if boot in self.retired:
            self.counts['retired_boot'] += 1
            return 'retired_boot'
        if abs(source_s-now_s) >= self.max_skew_s:
            self.counts['clock_mismatch'] += 1
            return 'clock_mismatch'
        event = 'accepted'
        latest = self.latest
        if latest is not None and boot == latest['boot']:
            if generation < latest['generation'] or status['now_us'] < latest['now_us']:
                self.counts['late'] += 1
                return 'late'
        elif latest is not None:
            self.retired.append(latest['boot'])
            self.counts['boot_changes'] += 1
            event = 'boot_changed'
        refreshed = latest is None or boot != latest['boot'] or status['now_us'] > latest['now_us']
        self.latest = dict(boot=boot, generation=generation, state=status['state'],
            reason=str(status.get('reason')), estop_held=status['estop_held'], now_us=status['now_us'],
            received_s=now_s if refreshed else latest['received_s'],
            received_wall=wall_s if refreshed else latest['received_wall'])
        self.counts['accepted'] += 1
        return event

    def fresh(self, now_s, wall_s):
        latest = self.latest
        return (latest is not None and 0 <= now_s-latest['received_s'] < self.max_age_s and
                0 <= wall_s-latest['received_wall'] < self.wall_timeout_s)


class RaceSupervisor:
    """새 경기·명시 STOP·사람 해제의 단일 호출 지점. 부모 단일 executor 전용.

    프로세스 시작·새 경기·STOP·고장마다 운영 개입 래치를 건다. 해제에는 같은 session·경기·
    개입 번호의 사람 확인과, 래치 시각 이후에 일어난 별도 MCU ARM의 목격이 모두 필요하다.
    출발 사실(묶인 영상 세대의 픽셀 판정)과 움직임 권한(`decide`)은 별개다.
    """
    def __init__(self, vision, *, session=None, mcu=None, image_max_age_s=1., history=64, now_s=None,
                 fence_margin_s=.02, resume_timeout_s=3., resume_wall_timeout_s=10., link_timeout_s=1.):
        if not _finite(image_max_age_s) or image_max_age_s <= 0:
            raise ValueError('image_max_age_s must be positive')
        if not _finite(fence_margin_s) or fence_margin_s < 0:
            raise ValueError('fence_margin_s must be non-negative')
        if not all(_finite(v) and v > 0 for v in (resume_timeout_s, resume_wall_timeout_s, link_timeout_s)):
            raise ValueError('resume limits must be finite and positive')
        # 팀 초안값. 입력 갱신을 기다리는 상한이며 보정·안정 대기 시간이 아니다.
        self.resume_timeout_s, self.resume_wall_timeout_s = resume_timeout_s, resume_wall_timeout_s
        self.link_timeout_s = link_timeout_s
        # 같은 /clock을 받는 두 프로세스의 배송 차이를 흡수하는 시험 가정 여유. 실측값이 아니다.
        self.fence_margin_s = fence_margin_s
        self.session = session or uuid.uuid4().hex
        self.race_id = uuid.uuid4().hex  # 프로세스 재시작을 넘어 같은 경기를 가리키는 식별자
        self.start_source = None
        self.recovery_intervention = None  # 복구 RESET을 마친 개입 번호. 새 STOP이면 무효가 된다.
        self.resume = None
        self.last_resume = None
        self.boundary_s = None
        self.boundary_count = 0
        self.permission = RacePermission()
        self.vision = vision
        self.mcu = mcu or McuObserver()
        self.image_max_age_s = image_max_age_s
        self.bound_vision_epoch = vision.epoch
        self.rearm_required = False
        self.rearm_fence_s = None
        self.rearm_baseline = {}
        self.mcu_stop_seen = False
        self.converted_mcu_latch = None
        self.vision_fault_latched = False
        self.processed = deque(maxlen=history)
        self.stop_requests = OrderedDict()
        self.history = history
        self.last_command = None
        self.events = deque(maxlen=32)
        self.counts = dict(visual_rejected_epoch=0, visual_accepted=0, commands=0, duplicates=0, rejected=0,
                           rearm_ignored_before_fence=0)
        # 프로세스 시작도 fail-closed: 이전 프로세스가 남긴 ARMED를 재사용하지 않는다.
        self._latch('process_start', now_s)

    def _event(self, kind, **data):
        self.events.append(dict(kind=kind, race_epoch=self.permission.race_epoch,
                                intervention=self.permission.intervention, **data))

    def _resolve_fence(self, now_s):
        # 시계를 아직 받지 못한 시작 직후의 래치는 첫 유효 시각을 fence로 쓴다.
        if self.rearm_required and self.rearm_fence_s is None and _finite(now_s) and now_s > 0:
            self.rearm_fence_s = now_s

    def _latch(self, reason, now_s):
        token = self.permission.stop(reason)
        self.rearm_required = True
        self.rearm_fence_s = None
        self.rearm_baseline = {}
        self.mcu_stop_seen = False
        self._resolve_fence(now_s)
        self._event('latch', reason=reason, fence_s=self.rearm_fence_s)
        self._close_resume('cancelled', reason)
        return token

    def _close_resume(self, outcome, reason):
        """대기 중인 START를 끝낸다. 취소된 요청은 재연결 뒤에도 실행하지 않는다."""
        resume, self.resume = self.resume, None
        if resume is None:
            return None
        self.last_resume = dict(request=resume['request'], outcome=outcome, reason=reason, phase=resume['phase'],
                                intervention=resume['intervention'], boundary_s=resume['boundary_s'],
                                missing=resume['missing'])
        self._event('resume_'+outcome, request=resume['request'], reason=reason, phase=resume['phase'])
        return self.last_resume

    def _input_boundary(self, now_s):
        """경기 세대는 유지하고 영상 세대만 올린다. 호출자가 스캔·운동·경로 기억을 같은 콜백에서 버린다."""
        self.boundary_s = now_s
        self.boundary_count += 1
        self.vision.begin_input_boundary(now_s)
        self.bound_vision_epoch = self.vision.epoch
        return now_s

    def _reset_vision(self):
        # 경기 세대와 영상 세대를 같은 호출에서 바꾼다. 진행 중 결과는 영상 세대로 거절된다.
        self.vision.reset()
        self.bound_vision_epoch = self.vision.epoch

    def stop(self, reason='explicit_stop', now_s=None):
        return self._latch(reason, now_s)

    def _new_race(self, reason, now_s):
        self.permission.new_race()
        self.race_id = uuid.uuid4().hex
        self.start_source = None
        self.recovery_intervention = None
        self._reset_vision()
        return self._latch(reason, now_s)

    def new_race(self, now_s=None):
        """새 영상 세대와 새 개입 래치. 사람 확인·별도 MCU 재ARM·새 신호 시퀀스가 필요하다."""
        self._new_race('new_race', now_s)
        return self.permission.race_epoch

    def clock_reset(self, now_s=None):
        return self._new_race('clock_reset', now_s)

    def legacy_reset(self, now_s=None):
        """구형 `/autonomy/reset`: 우회 경로가 되지 않도록 새 경기와 래치를 함께 적용한다."""
        return self._new_race('legacy_reset', now_s)

    def poll_faults(self, now_s=None):
        if self.vision.fault and not self.vision_fault_latched:
            self.vision_fault_latched = True
            self._latch('fault:vision', now_s)

    def observe_vision(self, result, vision_epoch, now_s):
        """출발 사실을 정책에 채택한다. 래치 중에는 채택하지 않지만 같은 영상 세대의 검출기 사실은 남는다."""
        if vision_epoch != self.bound_vision_epoch or self.vision.epoch != self.bound_vision_epoch:
            self.counts['visual_rejected_epoch'] += 1
            return False
        accepted = self.permission.observe_start(race_epoch=self.permission.race_epoch,
            visual_confirmed=self.vision.signal.started is True, capture_s=result['stamp'],
            now_s=now_s, max_age_s=self.image_max_age_s)
        self.counts['visual_accepted'] += accepted
        if accepted and self.start_source is None:
            self.start_source = 'visual'
        return accepted

    def markers_apply(self):
        """출발 뒤 STOP 중(운반 중일 수 있음)의 영상은 기준 벽을 바꾸지 않는다. START 경계 뒤부터 다시 쓴다."""
        return not (self.permission.started and self.permission.latch is not None and self.resume is None)

    def observe_mcu(self, status, now_s, wall_s):
        event = self.mcu.update(status, now_s, wall_s)
        if event not in ('accepted', 'boot_changed'):
            return event
        latest, identity = self.mcu.latest, self.mcu.identity()
        if event == 'boot_changed' and self.permission.latch is None:
            self._latch('mcu_boot_changed', now_s)
        elif event == 'boot_changed':
            self._close_resume('cancelled', 'mcu_boot_changed')
        if latest['state'] in LATCHED_STATES and identity != self.converted_mcu_latch:
            self.converted_mcu_latch = identity
            if self.permission.latch is None:
                self._latch('mcu:'+latest['state']+':'+latest['reason'], now_s)
            else:
                self.mcu_stop_seen = True
                self._event('mcu_latched_during_intervention', mcu_identity=identity)
        self._resolve_fence(now_s)
        if self.rearm_required:
            self._witness(latest)
        return event

    def _witness(self, latest):
        """래치 시각 이후 원본 시각의 상태만 쓴다. 그 boot의 첫 상태 세대보다 높은 ARMED만 재ARM이다.

        ARMED↔ACTIVE는 DRIVE로 바뀌며 generation을 올리지 않는다. generation은 ARM·DISARM·CLEAR·
        STOP·고장 같은 수신기 상태 전환에서만 오르므로, 첫 관측이 이미 ARMED여도 그 ARM이
        래치 전일 수 있다고 보고 세대 증가를 다시 요구한다. 같은 /clock 가정의 가상 MCU 계약이다.
        """
        fence = self.rearm_fence_s
        if fence is None or latest['now_us']*1e-6 <= fence+self.fence_margin_s:
            self.counts['rearm_ignored_before_fence'] += 1
            return
        baseline = self.rearm_baseline.setdefault(latest['boot'], latest['generation'])
        if latest['state'] in ARMED_STATES and latest['generation'] > baseline:
            self.rearm_required = False
            self._event('mcu_rearm_witnessed', mcu_identity=(latest['boot'], latest['generation']),
                        baseline_generation=baseline, fence_s=fence)

    def actuator_status(self, now_s, wall_s):
        latest = self.mcu.latest
        if latest is None:
            return False, 'mcu_status_missing'
        if not self.mcu.fresh(now_s, wall_s):
            return False, 'mcu_status_stale'
        if latest['estop_held']:
            return False, 'mcu_physical_stop_held'
        if latest['state'] not in ARMED_STATES:
            return False, 'mcu_not_armed'
        if self.rearm_required:
            return False, 'mcu_rearm_required'
        return True, 'mcu_armed'

    def decide(self, *, sensors_ready, now_s, wall_s):
        self.poll_faults(now_s)
        self._resolve_fence(now_s)
        ready, actuator_reason = self.actuator_status(now_s, wall_s)
        decision = self.permission.decide(sensors_ready=sensors_ready, actuator_ready=ready)
        return dict(allowed=decision.allowed, reason=decision.reason, actuator_reason=actuator_reason)

    def operator_link(self, request, wall_s):
        """대기 중인 같은 START의 운영 연결 유지 신호. 다른 요청 번호는 무시한다."""
        if self.resume is not None and request == self.resume['request'] and _finite(wall_s):
            self.resume['link_wall'] = wall_s

    def advance_resume(self, now_s, wall_s, *, inputs, route):
        """대기 중인 START를 한 단계 평가한다. 정상 입력 갱신을 기다리는 것 외의 대기는 없다."""
        resume = self.resume
        if resume is None:
            return None
        missing = [name for name in RESUME_INPUTS if inputs.get(name) is not True]
        resume['missing'], resume['route'] = missing, route
        if wall_s-resume['link_wall'] >= self.link_timeout_s:
            return self._close_resume('cancelled', 'operator_link_lost')
        if self.mcu.latest is not None and self.mcu.latest['estop_held']:
            return self._close_resume('cancelled', 'physical_stop_held')
        if (now_s-resume['boundary_s'] >= self.resume_timeout_s or
                wall_s-resume['boundary_wall'] >= self.resume_wall_timeout_s):
            return self._close_resume('cancelled', 'input_timeout:'+resume['phase'])
        if missing:
            resume['phase'] = 'inputs'
            return None
        if not (route and route.get('ready') is True):
            resume['phase'] = 'route'
            return None
        ready, resume['actuator_reason'] = self.actuator_status(now_s, wall_s)
        if not ready:
            resume['phase'] = 'mcu'
            return None
        if not self.permission.acknowledge(race_epoch=self.permission.race_epoch,
                intervention=resume['intervention'], hazards_cleared=True, operator_reviewed=True):
            return self._close_resume('cancelled', 'intervention_changed')
        self.recovery_intervention = None
        return self._close_resume('permitted', 'race_permitted')

    def _recovery_reset(self, request, now_s):
        if not self.permission.started:
            return self._result(request, 'recovery_reset', False, 'race_not_started')
        self._close_resume('cancelled', 'recovery_reset')
        self.recovery_intervention = self.permission.intervention
        self._event('recovery_reset', request=request)
        return self._result(request, 'recovery_reset', True, 'recovery_reset_held_start_required',
                            boundary_s=self._input_boundary(now_s))

    def _ordinary_stop(self):
        """정상 운영 STOP만 RESET 버튼을 생략한다. 고장 해제·물리 정지 복구와 구별한다."""
        mcu = self.mcu.latest
        return (self.permission.latch == 'explicit_stop' and mcu is not None
                and not mcu['estop_held'] and not self.vision.fault
                and (mcu['state'] in ('DISARMED', *ARMED_STATES)
                     or mcu['state'] == 'ESTOP_LATCHED' and mcu['reason'] == 'software_stop'))

    def _resume_start(self, request, now_s, wall_s):
        permission = self.permission
        if not permission.started:
            return self._result(request, 'resume_start', False, 'race_not_started')
        if self.resume is not None:
            return self._result(request, 'resume_start', False, 'start_already_pending')
        reset_done = self.recovery_intervention == permission.intervention
        if not reset_done and not self._ordinary_stop():
            return self._result(request, 'resume_start', False, 'recovery_reset_required')
        if self.mcu.latest is None:
            return self._result(request, 'resume_start', False, 'mcu_status_missing')
        if not _finite(now_s) or now_s <= 0 or not _finite(wall_s):
            return self._result(request, 'resume_start', False, 'clock_unavailable')
        # 이전 ARM을 재사용하지 않는다. 이 요청의 입력 경계 뒤 ARM만 목격한다.
        self.rearm_required, self.rearm_fence_s, self.rearm_baseline = True, now_s, {}
        boundary = self._input_boundary(now_s)
        self.resume = dict(request=request, intervention=permission.intervention, boundary_s=boundary,
                           boundary_wall=wall_s, link_wall=wall_s, phase='inputs',
                           missing=list(RESUME_INPUTS), route=None, actuator_reason=None)
        self._event('resume_requested', request=request, boundary_s=boundary,
                    preparation='recovery_reset' if reset_done else 'ordinary_stop')
        return self._result(request, 'resume_start', True, 'preparing_not_motion_permission', boundary_s=boundary)

    def _restore_race(self, request, payload, now_s):
        """재시작한 프로세스의 사람 확인 복원. 호출자는 저장 기록을 읽어 넘길 뿐 자동 재사용하지 않는다."""
        permission = self.permission
        if (payload['same_race_confirmed'] is not True or not _request_id(payload['race_id'])
                or payload['side'] not in ('left', 'right')):
            return self._result(request, 'restore_race', False, 'schema')
        if permission.started or permission.latch != 'process_start' or permission.race_epoch != 0:
            return self._result(request, 'restore_race', False, 'restore_requires_fresh_process')
        permission.started = True
        self.race_id, self.start_source = payload['race_id'], 'restored_operator_confirmed'
        self.recovery_intervention = permission.intervention
        self._event('race_restored', request=request, race_id=self.race_id)
        return self._result(request, 'restore_race', True, 'restored_held_start_required',
                            side=payload['side'], boundary_s=self._input_boundary(now_s))

    def _result(self, request, op, accepted, reason, **extra):
        result = dict(v=1, request=request, op=op, accepted=accepted, reason=reason, session=self.session,
                      race_epoch=self.permission.race_epoch, intervention=self.permission.intervention,
                      latch=self.permission.latch, **extra)
        self.last_command = result
        if not accepted:
            self.counts['rejected'] += 1
        return result

    def _remember(self, request):
        if request not in self.processed:
            self.processed.append(request)

    def handle_command(self, payload, now_s=None, wall_s=None):
        """명령 결과를 반환한다. STOP은 형식이 불완전하거나 요청 번호가 다른 명령과 겹쳐도 적용한다."""
        self.counts['commands'] += 1
        if type(payload) is not dict:
            return self._result(None, None, False, 'invalid_payload')
        request, op = payload.get('request'), payload.get('op')
        if op == 'stop':
            request = request if _request_id(request) else None
            if (request is not None and self.permission.latch is not None and
                    self.stop_requests.get(request) == self.permission.intervention):
                # 현재 활성 개입을 만든 같은 STOP의 재전송만 새 개입 없이 수락한다.
                self.counts['duplicates'] += 1
                return self._result(request, op, True, 'duplicate_stop_already_applied')
            token = self._latch('explicit_stop', now_s)
            if request is not None:
                self.stop_requests[request] = token
                self.stop_requests.move_to_end(request)
                while len(self.stop_requests) > self.history:
                    self.stop_requests.popitem(last=False)
                self._remember(request)
            return self._result(request, op, True, 'stop_latched', stop_token=token)
        if payload.get('v') != 1 or not _request_id(request):
            return self._result(request if _request_id(request) else None, op, False, 'schema')
        if request in self.processed:
            self.counts['duplicates'] += 1
            return self._result(request, op, False, 'duplicate_request')
        if op not in COMMAND_KEYS or set(payload) != COMMAND_KEYS[op]:
            return self._result(request, op, False, 'schema')
        self._remember(request)
        if payload['session'] != self.session:
            return self._result(request, op, False, 'session_mismatch')
        if type(payload['race_epoch']) is not int or payload['race_epoch'] != self.permission.race_epoch:
            return self._result(request, op, False, 'race_epoch_mismatch')
        if op == 'cancel_start':
            if self.resume is None or payload['start_request'] != self.resume['request']:
                return self._result(request, op, False, 'no_matching_pending_start')
            self._close_resume('cancelled', str(payload['reason'])[:64])
            return self._result(request, op, True, 'pending_start_cancelled')
        if op == 'new_race':
            self.new_race(now_s)
            return self._result(request, op, True, 'new_race_latched_ack_rearm_and_visual_start_required')
        if self.vision.fault:
            return self._result(request, op, False, 'fixed_fault_present')
        if self.mcu.latest is not None and self.mcu.latest['estop_held']:
            return self._result(request, op, False, 'physical_stop_held')
        if op in ('recovery_reset', 'resume_start', 'restore_race'):
            if type(payload['intervention']) is not int or payload['intervention'] != self.permission.intervention:
                return self._result(request, op, False, 'intervention_mismatch')
            if self.permission.latch is None:
                return self._result(request, op, False, 'no_active_intervention')
            if op == 'recovery_reset':
                return self._recovery_reset(request, now_s)
            if op == 'resume_start':
                return self._resume_start(request, now_s, wall_s)
            return self._restore_race(request, payload, now_s)
        if self.resume is not None or self.recovery_intervention == self.permission.intervention:
            # 복구 RESET 뒤의 해제는 입력 경계를 지나는 START 한 경로뿐이다.
            return self._result(request, op, False, 'recovery_requires_start')
        prepare_late = (not self.permission.started and
                        getattr(self.vision, 'signal_mode', None) == 'official_marker_late_candidate')
        if prepare_late and (not _finite(now_s) or now_s < 0):
            return self._result(request, op, False, 'invalid_preparation_stamp')
        if not self.permission.acknowledge(race_epoch=payload['race_epoch'],
                intervention=payload['intervention'], hazards_cleared=payload['hazards_cleared'],
                operator_reviewed=payload['operator_reviewed']):
            return self._result(request, op, False, 'acknowledge_rejected')
        if prepare_late:
            # 준비 전 시험 녹색을 이후 출발 사실로 재사용하지 않는다.
            # 영상 작업 세대와 촬영시각 경계만 변경하며 MCU/주행을 허가하지 않는다.
            self._reset_vision()
            self.vision.input_floor_s = now_s
            self._event('visual_start_prepared', capture_boundary_s=now_s)
        self._event('acknowledged')
        return self._result(request, op, True, 'policy_latch_cleared_mcu_and_sensors_still_required')

    def snapshot(self, now_s, wall_s):
        decision = self.decide(sensors_ready=True, now_s=now_s, wall_s=wall_s)
        latest, resume = self.mcu.latest, self.resume
        return dict(mode='b3_candidate', session=self.session, race_id=self.race_id,
            race_epoch=self.permission.race_epoch,
            vision_epoch=self.bound_vision_epoch, intervention=self.permission.intervention,
            latch=self.permission.latch, started=self.permission.started, start_source=self.start_source,
            recovery_ready=(self.permission.latch is not None and resume is None
                            and self.recovery_intervention == self.permission.intervention),
            resume_start_ready=(self.permission.started and self.permission.latch is not None and resume is None
                                and (self.recovery_intervention == self.permission.intervention
                                     or self._ordinary_stop())),
            resume=None if resume is None else {key: value for key, value in resume.items()
                                                if key not in ('boundary_wall', 'link_wall')},
            last_resume=self.last_resume, input_boundary_s=self.boundary_s,
            visual_start_fact=self.vision.epoch == self.bound_vision_epoch and self.vision.signal.started is True,
            policy_allowed_if_sensors_ready=decision['allowed'], policy_reason=decision['reason'],
            actuator_reason=decision['actuator_reason'], rearm_required=self.rearm_required,
            rearm_fence_s=self.rearm_fence_s, rearm_baseline=dict(self.rearm_baseline),
            mcu_stop_observed_during_intervention=self.mcu_stop_seen,
            mcu=None if latest is None else dict(boot=latest['boot'], generation=latest['generation'],
                state=latest['state'], reason=latest['reason'], estop_held=latest['estop_held'],
                now_us=latest['now_us'], fresh=self.mcu.fresh(now_s, wall_s)),
            mcu_counts=dict(self.mcu.counts), counts=dict(self.counts), last_command=self.last_command,
            interpretation='visual start fact and motion permission are separate; upper latch only; '
                           'MCU STOP delivery/ACK and physical stop are separate evidence')
