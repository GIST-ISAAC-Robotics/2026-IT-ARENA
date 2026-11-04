"""경기 운영 어댑터의 순수 상태. ROS·HTTP·디스크를 직접 다루지 않는다.

표시값의 의미를 고정한다. 계획은 /drive, 보호 후는 /drive/safe, MCU 적용 목표는 MCU 상태의
target_speed_mps, 모터축 환산은 MCU 상태의 mean_wheel_mps(단일 모터축 카운트 환산)다.
어느 값도 차체 실제 속도가 아니다. 요청 수락은 MCU 정지나 주행 허가의 증명이 아니다.
"""
from collections import deque
import copy
import threading

OPERATIONS = ('stop', 'recovery_reset', 'resume_start', 'prepare_start', 'new_race', 'restore_race')
BOUND = tuple(op for op in OPERATIONS if op != 'stop')
PHYSICAL_STOP = '미확인'
MAX_COMMANDS = 32
MAX_EVENTS = 64
MAX_ROWS = 6000
LATCHED_MCU = ('ESTOP_LATCHED', 'FAULT_LATCHED')
ARMED_MCU = ('ARMED', 'ACTIVE')
HOLD_STATES = ('RACE_LATCHED', 'ACTUATOR_WAIT', 'WAIT_GREEN', 'RESUME_PREPARING', 'DISABLED')
SENSOR_LIMITS_S = dict(scan=.45, rgb=1., gyro=.2, encoder=.2)
LABELS = dict(signal='신호 대기', running='주행', yield_hold='일시 양보·정체', operator_stop='운영자 STOP',
              recovery='복구 준비', resumable='재개 가능', fault='입력·구동 고장', setup='경기 준비 필요',
              unknown='상태 없음')


def token(value):
    return type(value) is str and 1 <= len(value) <= 64 and all(c.isalnum() or c in '-_' for c in value)


def side_of(status):
    return status.get('side') if status.get('side') in ('left', 'right') else None


def display_state(status, mcu):
    """화면 상태. 제어에 쓰지 않는다."""
    race = status.get('race') if status else None
    if not race:
        return 'unknown'
    latch = race.get('latch') or ''
    fault = (latch.startswith(('fault:', 'mcu:')) or (status.get('vision') or {}).get('fault')
             or (mcu or {}).get('state') == 'FAULT_LATCHED')
    if race.get('resume'):
        return 'recovery'
    if fault:
        return 'fault'
    if latch:
        if race.get('recovery_ready'):
            return 'resumable'
        return 'operator_stop' if race.get('started') else 'setup'
    if not race.get('started'):
        return 'signal'
    state = status.get('state')
    if state in ('SENSOR_STOP', 'MOTION_STOP', 'SCAN_INVALID'):
        return 'fault'
    return 'running' if state == 'RUNNING' else 'yield_hold'


class OpsCore:
    """HTTP 스레드와 ROS 콜백이 공유한다. 모든 공개 메서드는 잠금 안에서 끝나며 외부 호출을 하지 않는다."""
    def __init__(self, vehicle_id, *, status_stale_s=1., client_timeout_s=2., record_max_age_s=7200.):
        self.vehicle_id = vehicle_id
        self.status_stale_s, self.client_timeout_s = status_stale_s, client_timeout_s
        self.record_max_age_s = record_max_age_s
        self.lock = threading.Lock()
        self.status = self.status_wall = None
        self.mcu = self.mcu_wall = None
        self.safety = self.safety_wall = None
        self.drive = {}  # topic -> (speed, steering, wall)
        self.commands = {}
        self.order = deque()
        self.actions = deque()
        self.events = deque(maxlen=MAX_EVENTS)
        self.rows = deque(maxlen=MAX_ROWS)
        self.unlogged = deque(maxlen=MAX_ROWS)
        self.clients = {}
        self.record = None
        self.previous = {}
        self.seq = 0
        self.log_state = {}

    # 입력 ---------------------------------------------------------------
    def update_status(self, status, wall):
        with self.lock:
            self.status, self.status_wall = status, wall
            self.seq += 1
            return self._observe(status, wall)

    def update_mcu(self, status, wall):
        with self.lock:
            previous = self.mcu
            self.mcu, self.mcu_wall = status, wall
            if previous is None or previous.get('state') != status.get('state'):
                self._event(wall, 'mcu', f"MCU {status.get('state')} ({status.get('reason')})", source='mcu')

    def update_safety(self, status, wall):
        with self.lock:
            previous = self.safety
            self.safety, self.safety_wall = status, wall
            if previous is None or previous.get('reason') != status.get('reason'):
                self._event(wall, 'safety', f"보호층 {status.get('reason')}", source='safety')

    def update_drive(self, topic, speed, steering, wall):
        with self.lock:
            self.drive[topic] = (speed, steering, wall)

    def _event(self, wall, kind, text, **extra):
        event = dict(wall_s=wall, kind=kind, text=text,
                     sim_s=None if not self.status else self.status.get('sim_time_s'), **extra)
        self.events.append(event)
        self._row(dict(row='event', **event))
        return event

    def _row(self, row):
        self.rows.append(row)
        self.unlogged.append(row)

    def _observe(self, status, wall):
        """상태 변화에서 사건을 만들고 시간축 표본을 남긴다. 판단을 만들지 않는다."""
        race = status.get('race') or {}
        keys = dict(state=status.get('state'), reason=status.get('reason'), behavior=status.get('behavior'),
                    handoff=status.get('handoff_state'), side=side_of(status), latch=race.get('latch'),
                    started=race.get('started'))
        names = dict(state='상태', reason='이유', behavior='거동', handoff='전환', side='기준 벽',
                     latch='래치', started='출발 사실')
        for key, value in keys.items():
            if self.previous.get(key, value) != value:
                self._event(wall, key, f"{names[key]}: {self.previous[key]} → {value}", source='autonomy')
        speed = status.get('speed_command_mps')
        before = self.previous.get('speed')
        if type(speed) in (int, float) and type(before) in (int, float) and before >= .3 and speed <= before*.5:
            self._event(wall, 'decel', f'계획 속도 {before:.2f} → {speed:.2f} m/s, 이유 {keys["reason"]}',
                        source='autonomy', speed_before=before, speed_after=speed)
        self.previous = {**keys, 'speed': speed}
        row = self._sample(status, wall)
        self._row(row)
        return row

    def _sample(self, status, wall):
        race = status.get('race') or {}
        plan, safe = self.drive.get('/drive'), self.drive.get('/drive/safe')
        mcu = self.mcu or {}
        inputs = status.get('inputs') or {}
        return dict(row='sample', wall_s=wall, sim_s=status.get('sim_time_s'), state=status.get('state'),
                    reason=status.get('reason'), behavior=status.get('behavior'), side=side_of(status),
                    side_marker=inputs.get('side_marker'), marker_ids=status.get('marker_ids'),
                    plan_mps=None if plan is None else plan[0], protected_mps=None if safe is None else safe[0],
                    steering_rad=status.get('steering_command_rad'), mcu_target_mps=mcu.get('target_speed_mps'),
                    motor_shaft_equiv_mps=mcu.get('mean_wheel_mps'), mcu_state=mcu.get('state'),
                    safety_reason=None if self.safety is None else self.safety.get('reason'),
                    clearance_m=None if self.safety is None else self.safety.get('swept_clearance_m'),
                    input_age_s=inputs.get('age_s'), before_boundary=inputs.get('before_boundary'),
                    latch=race.get('latch'), race_id=race.get('race_id'),
                    intervention=race.get('intervention'))

    # 명령 ---------------------------------------------------------------
    def _status_stale(self, wall):
        return self.status is None or wall-self.status_wall >= self.status_stale_s

    def _controls(self, wall):
        race = (self.status or {}).get('race')
        stale = self._status_stale(wall)
        result = dict(stop=(True, None))
        if stale or not race:
            return {**result, **{op: (False, 'status_stale') for op in BOUND}}
        latch, started = race.get('latch'), race.get('started')
        offer = self._restore_offer(wall)
        result.update(
            recovery_reset=(bool(latch and started and not race.get('resume')),
                            None if latch and started else 'stop_required' if started else 'race_not_started'),
            resume_start=(bool(race.get('resume_start_ready', race.get('recovery_ready'))),
                          'recovery_reset_required' if started else 'race_not_started'),
            prepare_start=(bool(latch and not started), 'race_already_started' if started else 'no_active_intervention'),
            new_race=(bool(latch), 'stop_required'),
            restore_race=(offer is not None, 'no_restorable_record'))
        return result

    def submit(self, op, request, view, client, confirm, wall):
        """HTTP 스레드에서 호출. 같은 요청 번호는 다시 실행하지 않고 기존 기록을 돌려준다."""
        with self.lock:
            if request in self.commands:
                return copy.deepcopy(self.commands[request])
            reason = self._refuse(op, view, confirm, wall)
            entry = dict(request=request, op=op, client=client, view=view, received_wall=wall, sent_wall=None,
                         stage='rejected_by_adapter' if reason else 'queued', adapter_reason=reason, policy=None,
                         mcu=None, mcu_steps=[], physical_stop=PHYSICAL_STOP if op == 'stop' else None)
            self.commands[request] = entry
            self.order.append(request)
            while len(self.order) > MAX_COMMANDS:
                self.commands.pop(self.order.popleft(), None)
            if reason is None:
                self.actions.append(entry)
            self._event(wall, 'operator', f'{op} 요청 {request[:8]}' +
                        (f' 거절: {reason}' if reason else ''), source='operator', request=request)
            return copy.deepcopy(entry)

    def _refuse(self, op, view, confirm, wall):
        if op not in OPERATIONS:
            return 'unknown_operation'
        if op == 'stop':
            return None
        race = (self.status or {}).get('race')
        if self._status_stale(wall) or not race:
            return 'status_stale'
        if type(view) is not dict or any(view.get(key) != race.get(key)
                                         for key in ('session', 'race_id', 'race_epoch', 'intervention')):
            return 'stale_view'
        if op == 'new_race' and confirm != 'NEW_RACE':
            return 'new_race_confirmation_required'
        if op == 'restore_race' and confirm != 'SAME_RACE':
            return 'same_race_confirmation_required'
        allowed, reason = self._controls(wall)[op]
        return None if allowed else reason

    def next_action(self):
        with self.lock:
            return self.actions.popleft() if self.actions else None

    def mark(self, entry, wall=None, **fields):
        with self.lock:
            entry.update(fields)
            if wall is not None and 'stage' in fields:
                self._event(wall, 'operator', f"{entry['op']} {entry['request'][:8]}: {fields['stage']}",
                            source='operator', request=entry['request'])

    def result(self, payload, wall):
        with self.lock:
            entry = self.commands.get(payload.get('request'))
            if entry is None or entry['op'] == 'prepare_start' and payload.get('op') != 'acknowledge':
                return None
            accepted = payload.get('accepted') is True
            entry['policy'] = dict(accepted=accepted, reason=payload.get('reason'), wall_s=wall)
            entry['stage'] = 'policy_accepted' if accepted else 'policy_rejected'
            self._event(wall, 'operator', f"{entry['op']} {entry['request'][:8]}: "
                        f"{'정책 수락' if accepted else '정책 거절'} {payload.get('reason')}",
                        source='operator', request=entry['request'])
            return entry

    def expire(self, wall, timeout_s=3.):
        """응답 없는 명령은 성공/실패를 추측하지 않는다. 자동 재전송도 없다."""
        with self.lock:
            for entry in self.commands.values():
                if entry['stage'] == 'sent' and wall-entry['sent_wall'] >= timeout_s:
                    entry['stage'] = 'unknown_after_send'
                    self._event(wall, 'operator', f"{entry['op']} {entry['request'][:8]}: 응답 없음(결과 미확인)",
                                source='operator', request=entry['request'])

    def client_seen(self, client, wall):
        with self.lock:
            self.clients[client] = wall
            for key in [k for k, v in self.clients.items() if wall-v > 60.]:
                del self.clients[key]

    def client_fresh(self, client, wall):
        with self.lock:
            return client in self.clients and wall-self.clients[client] < self.client_timeout_s

    # 저장 기록 -----------------------------------------------------------
    def set_record(self, record):
        with self.lock:
            self.record = record

    def record_payload(self, wall_time):
        """같은 경기 복원에 필요한 최소 사실. 경로·운동 기하는 저장하지 않는다."""
        with self.lock:
            status = self.status or {}
            race = status.get('race') or {}
            if not race.get('started') or side_of(status) is None:
                return None
            return dict(v=1, vehicle_id=self.vehicle_id, race_id=race.get('race_id'), session=race.get('session'),
                        race_epoch=race.get('race_epoch'), start_source=race.get('start_source'),
                        side=side_of(status), side_marker=(status.get('inputs') or {}).get('side_marker'),
                        saved_wall_time=wall_time)

    def _restore_offer(self, wall_time=None):
        """사람이 같은 경기임을 확인할 후보만 보여 준다. 새 프로세스의 출발 전 상태에서만 제시한다."""
        record, race = self.record, (self.status or {}).get('race')
        if not record or not race or race.get('started') or race.get('latch') != 'process_start':
            return None
        if race.get('race_epoch') != 0 or record.get('session') == race.get('session'):
            return None
        if record.get('vehicle_id') != self.vehicle_id or record.get('side') not in ('left', 'right'):
            return None
        return record

    def restore_offer(self, wall_time):
        with self.lock:
            record = self._restore_offer()
            if record is None or not 0 <= wall_time-record.get('saved_wall_time', -1) <= self.record_max_age_s:
                return None
            return dict(record)

    # 표시 ---------------------------------------------------------------
    def view(self, wall, wall_time):
        with self.lock:
            status, mcu = self.status or {}, self.mcu or {}
            race = status.get('race') or {}
            inputs = status.get('inputs') or {}
            age = lambda at: None if at is None else wall-at
            state = display_state(self.status, self.mcu)
            plan, safe = self.drive.get('/drive'), self.drive.get('/drive/safe')
            ages = inputs.get('age_s') or {}
            sensors = [dict(name=name, age_s=ages.get(name), limit_s=limit,
                            ok=ages.get(name) is not None and 0 <= ages[name] < limit)
                       for name, limit in SENSOR_LIMITS_S.items()]
            controls = self._controls(wall)
            offer = self._restore_offer()
            if offer is not None and not 0 <= wall_time-offer.get('saved_wall_time', -1) <= self.record_max_age_s:
                offer = None
                controls['restore_race'] = (False, 'record_too_old')
            return dict(v=1, vehicle_id=self.vehicle_id, seq=self.seq, state=state, state_label=LABELS[state],
                connection=dict(status_age_s=age(self.status_wall), status_stale=self._status_stale(wall),
                                mcu_age_s=age(self.mcu_wall), safety_age_s=age(self.safety_wall)),
                binding={key: race.get(key) for key in ('session', 'race_id', 'race_epoch', 'intervention')},
                race={key: race.get(key) for key in ('latch', 'started', 'start_source', 'recovery_ready', 'resume_start_ready', 'resume',
                    'last_resume', 'policy_reason', 'actuator_reason', 'rearm_required', 'input_boundary_s',
                    'visual_start_fact')},
                speeds=dict(plan_mps=None if plan is None else plan[0], plan_age_s=None if plan is None else age(plan[2]),
                            protected_mps=None if safe is None else safe[0],
                            protected_age_s=None if safe is None else age(safe[2]),
                            mcu_target_mps=mcu.get('target_speed_mps'), motor_shaft_equiv_mps=mcu.get('mean_wheel_mps'),
                            motor_feedback_valid=mcu.get('feedback_valid'), mcu_age_s=age(self.mcu_wall)),
                mcu={key: mcu.get(key) for key in ('state', 'reason', 'estop_held', 'stationary_ready', 'output_ready',
                                                   'generation')},
                sensors=sensors,
                driving=dict(state=status.get('state'), reason=status.get('reason'), behavior=status.get('behavior'),
                             handoff=status.get('handoff_state'), side=side_of(status),
                             side_marker=inputs.get('side_marker'), marker_ids=status.get('marker_ids'),
                             route=inputs.get('route'), sim_s=status.get('sim_time_s'),
                             safety_reason=None if self.safety is None else self.safety.get('reason'),
                             clearance_m=None if self.safety is None else self.safety.get('swept_clearance_m')),
                controls={op: dict(allowed=flag, reason=None if flag else why) for op, (flag, why) in controls.items()},
                restore_offer=offer,
                commands=[copy.deepcopy(self.commands[key]) for key in list(self.order)[-8:]][::-1],
                events=list(self.events)[-24:][::-1], log=dict(self.log_state),
                limits=dict(physical_stop=PHYSICAL_STOP, wifi_stop_is_not_hardware_stop=True))

    def export_rows(self, seconds, wall):
        with self.lock:
            return [dict(row) for row in self.rows if wall-row['wall_s'] <= seconds]

    def drain_rows(self):
        with self.lock:
            rows, self.unlogged = list(self.unlogged), deque(maxlen=MAX_ROWS)
            return rows

    def set_log_state(self, state):
        with self.lock:
            self.log_state = state
