"""B3 경기 감독 후보의 순수 결합 회귀. ROS·보호층·실제 출력·차량 운동은 없다.

실제 픽셀 인식(`VisionPipeline`/`recognize`), `RaceSupervisor`, Python 참조 A1 수신기를 함께 호출한다.
엔코더는 고정 0카운트이고 MCU 목표 0은 실제 정지 증거가 아니다. 시각 간격과 같은 시계는 시험 가정이다.
"""
from types import SimpleNamespace

import pytest

from arena_autonomy.race_session import McuObserver, RaceSupervisor
from arena_autonomy.vision_process import VisionPipeline, recognize
from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, ContractConfig, State
from audit_start_signal_layout import light_image

WALL = 1000.


class InlineWorker:
    def __init__(self):
        self.result = None
        self.error = None

    def submit(self, request, data):
        self.result = recognize(request, data)

    def poll(self):
        if self.error:
            raise RuntimeError(self.error)
        result, self.result = self.result, None
        return result


class Bench:
    def __init__(self, boot_id='fixture-mcu', receiver=None, now_us=0, feedback_seq=0, host=None):
        self.receiver = receiver or self.make_receiver(boot_id)
        # 호스트 송신기는 제어기와 별도 프로세스이므로 제어기 재시작 시험에서는 그대로 쓴다.
        self.host = host or CommandProducer(client_id='fixture-host')
        self.vision = VisionPipeline(InlineWorker(), signal_mode='official_4red_candidate')
        self.now_us = now_us
        self.race = RaceSupervisor(self.vision, session='fixture-session', now_s=self.now_s)
        self.feedback_seq = feedback_seq
        self.sensors_ready = True
        self.statuses = []
        if receiver is None:
            self.receiver.update_motor_encoder(0, sequence=0, capture_us=0, motor_counts=0)
        self.advance(220_000)

    @staticmethod
    def make_receiver(boot_id):
        return ActuationReceiver(ContractConfig(feedback_mode='drive_motor_shaft', ticks_per_rev=28,
            gear_ratio=15., wheel_radius_m=.0325), boot_id=boot_id)

    @property
    def now_s(self):
        return self.now_us/1e6

    def status(self):
        return {**self.receiver.status(), 'now_us': self.now_us}

    def publish(self, status=None):
        status = self.status() if status is None else status
        self.statuses.append(status)
        return self.race.observe_mcu(status, self.now_s, self.now_s+WALL)

    def decision(self):
        return self.race.decide(sensors_ready=self.sensors_ready, now_s=self.now_s, wall_s=self.now_s+WALL)

    def advance(self, duration_us=10_000, *, publish=True, host=True):
        assert duration_us > 0 and duration_us % 10_000 == 0
        for _ in range(duration_us//10_000):
            self.now_us += 10_000
            self.feedback_seq += 1
            self.receiver.update_motor_encoder(self.now_us, sequence=self.feedback_seq,
                capture_us=self.now_us, motor_counts=0)
            self.receiver.tick(self.now_us)
            if publish:
                self.publish()
            if host and self.receiver.armed and self.receiver.owner in (None, self.host.client):
                # A1 참조 송신일 뿐 /drive/safe·독립 보호층을 실행하지 않는다.
                offer = self.receiver.offer(self.now_us)
                speed = .3 if self.decision()['allowed'] else 0.
                packet = self.host.drive(offer, speed_mps=speed, steering_rad=.1,
                    source_us=self.now_us+3_000_000, host_now_us=self.now_us+3_000_000)
                assert self.receiver.receive(packet, self.now_us)[0]

    def control(self, kind, *, publish=True):
        packet = self.host.control(self.receiver.offer(self.now_us), kind)
        result = self.receiver.receive(packet, self.now_us)
        if publish:
            self.publish()
        return result

    def mcu_stop(self):
        assert self.receiver.receive({'v': 1, 'kind': 'STOP'}, self.now_us)[0]
        self.publish()

    def image(self, reds=0, green=False):
        image = light_image(reds=reds, green=green, diameter=12, paired=True)
        sec, nanosec = divmod((self.now_us-1000)*1000, 1_000_000_000)
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
            data=image.tobytes(), height=image.shape[0], width=image.shape[1], step=image.shape[1]*3,
            encoding='rgb8')

    def frame(self, reds=0, green=False, *, submit_only=False):
        self.advance(100_000)
        self.vision.offer(self.image(reds, green), wall_now=self.now_s+WALL)
        self.vision.pump(self.now_s, wall_now=self.now_s+WALL)
        if submit_only:
            return
        accepted = self.vision.pump(self.now_s, wall_now=self.now_s+WALL)
        self.race.poll_faults(self.now_s)
        if accepted is not None and not self.vision.fault:
            self.race.observe_vision(accepted, self.vision.epoch, self.now_s)
        return accepted

    def lights(self):
        for n in (0, 1, 2, 3, 4, 4):
            self.frame(n)
        for _ in range(3):
            self.frame(green=True)
        assert self.vision.signal.started

    def command(self, op, request, **fields):
        return self.race.handle_command(dict(v=1, request=request, op=op, **fields), self.now_s)

    def ack(self, request=None, **override):
        self.requests = getattr(self, 'requests', 0)+1
        fields = dict(session=self.race.session, race_epoch=self.race.permission.race_epoch,
                      intervention=self.race.permission.intervention, hazards_cleared=True,
                      operator_reviewed=True)
        fields.update(override)
        return self.command('acknowledge', request or 'ack-%d' % self.requests, **fields)

    def rearm(self):
        # 래치와 같은 순간(fence 여유 20 ms 안)의 ARM은 구별하지 않는다. 사람 조작 간격을 둔다.
        self.advance(50_000)
        assert self.control('DISARM')[0] and self.control('ARM')[0]

    def running(self):
        self.advance()
        return self.decision()['allowed'] and self.receiver.state == State.ACTIVE


def started_bench():
    b = Bench()
    assert b.ack()['accepted'] and b.control('ARM')[0]
    b.lights()
    assert b.race.permission.started and b.running()
    return b


@pytest.mark.parametrize('order', ['ack_arm_lights', 'lights_arm_ack', 'arm_lights_ack'])
def test_process_start_is_latched_and_needs_ack_post_start_arm_and_lights(order):
    b = Bench()
    assert b.race.permission.latch == 'process_start' and b.race.permission.intervention == 1
    steps = dict(ack=lambda: b.ack()['accepted'], arm=lambda: b.control('ARM')[0], lights=lambda: b.lights() or True)
    done = []
    for step in order.split('_'):
        assert not b.running() and b.receiver.target_speed == 0.
        assert steps[step]()
        done.append(step)
    if order != 'ack_arm_lights':
        b.frame()  # 래치 중 확인된 같은 영상 세대의 출발 사실을 해제 뒤 정책에 채택
    assert b.running()


def test_process_restart_does_not_reuse_previous_armed_generation():
    old = started_bench()
    receiver = old.receiver  # MCU는 이전 제어기 프로세스의 ARMED를 유지한다.
    b = Bench(receiver=receiver, now_us=old.now_us, feedback_seq=old.feedback_seq, host=old.host)
    assert receiver.state in (State.ARMED, State.ACTIVE)
    assert b.ack()['accepted']
    b.lights()
    assert b.race.permission.started and b.decision()['actuator_reason'] == 'mcu_rearm_required'
    assert receiver.target_speed == 0.
    b.rearm()
    assert b.running()


@pytest.mark.parametrize('hold', ['sensor', 'mcu_status_gap'])
def test_temporary_holds_recover_without_latch_or_rearm(hold):
    b = started_bench()
    generation, intervention = b.receiver.generation, b.race.permission.intervention
    if hold == 'sensor':
        b.sensors_ready = False
        b.advance(40_000)
        assert b.decision()['reason'] == 'sensor_hold'
        b.sensors_ready = True
    else:
        b.advance(150_000, publish=False)
        assert b.decision()['actuator_reason'] == 'mcu_status_stale'
    assert b.race.permission.latch is None and b.receiver.generation == generation
    assert b.running() and b.race.permission.intervention == intervention


def test_upper_stop_without_mcu_stop_cannot_resume_until_actual_rearm():
    b = started_bench()
    assert b.command('stop', 'stop-1')['accepted'] and b.race.permission.latch == 'explicit_stop'
    b.advance(50_000)
    assert b.receiver.state == State.ARMED and b.receiver.target_speed == 0.
    assert b.ack(intervention=1)['reason'] == 'acknowledge_rejected'
    assert b.ack()['accepted']
    b.advance()
    assert b.decision()['actuator_reason'] == 'mcu_rearm_required' and b.receiver.target_speed == 0.
    b.rearm()
    assert b.running()  # 같은 경기: 신호를 다시 보지 않는다.


def test_delayed_pre_latch_disarm_arm_states_are_not_rearm_evidence():
    """래치 전 DISARM→ARM 상태가 늦게 와도 재ARM이 아니다. 이전 규칙(비무장 뒤 ARMED)은 이를 오인한다."""
    b = started_bench()
    b.advance(30_000, publish=False)
    assert b.control('DISARM', publish=False)[0]
    delayed = [b.status()]
    b.advance(10_000, publish=False)
    assert b.control('ARM', publish=False)[0]
    delayed.append(b.status())
    b.advance(10_000, publish=False)
    assert b.command('stop', 'stop-1')['accepted']
    fence = b.race.rearm_fence_s
    assert all(s['now_us']*1e-6 < fence for s in delayed)
    for status in delayed:
        assert b.publish(status) == 'accepted'  # 순서·시각 검사는 통과하지만 fence 이전이다.
    assert b.race.rearm_required and b.race.counts['rearm_ignored_before_fence'] >= 2
    assert b.ack()['accepted']
    b.advance(50_000)
    assert b.decision()['actuator_reason'] == 'mcu_rearm_required' and b.receiver.target_speed == 0.
    b.rearm()
    assert b.running()


def test_first_post_fence_armed_status_needs_a_new_generation():
    b = started_bench()
    b.advance(30_000, publish=False)
    b.rearm()  # 래치 직전 재ARM, 상태는 아직 늦게 도착하지 않음
    b.command('stop', 'stop-1')
    b.advance(50_000)  # 래치 뒤 첫 상태가 이미 ARMED: 그 ARM이 래치 전일 수 있다
    assert b.ack()['accepted']
    b.advance()
    assert b.decision()['actuator_reason'] == 'mcu_rearm_required'
    b.rearm()
    assert b.running()


@pytest.mark.parametrize('order', ['ack_first', 'clear_arm_first'])
def test_mcu_stop_becomes_intervention_and_needs_ack_plus_clear_arm(order):
    b = started_bench()
    before = b.race.permission.intervention
    b.mcu_stop()
    assert b.race.permission.latch.startswith('mcu:ESTOP_LATCHED') and b.race.permission.intervention == before+1
    b.advance(220_000)
    assert b.race.permission.intervention == before+1  # 같은 MCU 세대의 반복 상태는 새 개입이 아니다.
    if order == 'ack_first':
        assert b.ack()['accepted']
        b.advance()
        assert b.decision()['actuator_reason'] == 'mcu_not_armed'
        assert b.control('CLEAR')[0]
        b.advance()
        assert b.receiver.state == State.DISARMED and not b.decision()['allowed']
        assert b.control('ARM')[0]
    else:
        assert b.control('CLEAR')[0] and b.control('ARM')[0]
        b.advance()
        assert b.decision()['reason'] == b.race.permission.latch and b.receiver.target_speed == 0.
        assert b.ack()['accepted']
    assert b.running()


def test_combined_upper_and_mcu_stop_is_one_intervention_and_requires_clear_arm():
    b = started_bench()
    token = b.command('stop', 'stop-1')['stop_token']
    b.mcu_stop()
    assert b.race.permission.intervention == token and b.race.mcu_stop_seen
    assert b.ack()['accepted']
    b.advance(220_000)
    assert not b.decision()['allowed']
    assert b.control('CLEAR')[0] and b.control('ARM')[0]
    assert b.running()


def test_late_old_generation_mcu_status_is_ignored_after_rearm():
    b = started_bench()
    b.mcu_stop()
    old = b.status()
    b.advance(220_000)
    assert b.ack()['accepted'] and b.control('CLEAR')[0] and b.control('ARM')[0]
    assert b.running()
    assert b.publish(dict(old, now_us=b.now_us)) == 'late'  # 시각만 새로워도 이전 세대의 ESTOP
    assert b.race.permission.latch is None and b.running()


def test_stop_request_id_rules_fail_closed():
    b = started_bench()
    used_by_ack = 'ack-shared'
    assert b.command('stop', 'stop-1')['reason'] == 'stop_latched'
    assert b.ack(used_by_ack)['accepted']
    b.rearm()
    assert b.running()
    # 다른 op가 쓴 요청 번호와 겹쳐도 STOP은 실제로 고정한다.
    result = b.command('stop', used_by_ack)
    assert result['reason'] == 'stop_latched' and b.race.permission.latch == 'explicit_stop'
    active = b.race.permission.intervention
    assert b.command('stop', used_by_ack)['reason'] == 'duplicate_stop_already_applied'
    assert b.race.permission.intervention == active
    assert b.ack()['accepted']
    b.rearm()
    assert b.running()
    # 이미 해제된 과거 STOP 번호의 재전송은 새로 고정한다.
    assert b.command('stop', 'stop-1')['reason'] == 'stop_latched'
    assert b.race.permission.intervention == active+1 and not b.running()
    # 다른 개입이 이어진 뒤의 재전송도 현재 개입이 아니므로 고정한다.
    b.mcu_stop()
    assert b.ack()['accepted']
    b.advance(220_000)
    b.command('stop', 'stop-2')
    current = b.race.permission.intervention
    assert b.command('stop', 'stop-1')['reason'] == 'stop_latched'
    assert b.race.permission.intervention == current+1
    # STOP 번호를 해제 요청에 재사용할 수는 없다.
    assert b.ack('stop-2')['reason'] == 'duplicate_request'


def test_stale_ack_duplicate_and_wrong_session_cannot_release_new_stop():
    b = started_bench()
    first = b.command('stop', 'stop-1')['stop_token']
    old_ack = dict(session=b.race.session, race_epoch=0, intervention=first,
                   hazards_cleared=True, operator_reviewed=True)
    b.command('stop', 'stop-2')
    assert b.command('acknowledge', 'late-ack', **old_ack)['reason'] == 'acknowledge_rejected'
    assert b.ack('ack-x', session='old-process')['reason'] == 'session_mismatch'
    assert b.ack('ack-y', hazards_cleared=False)['reason'] == 'acknowledge_rejected'
    assert b.ack('ack-ok')['accepted']
    assert b.ack('ack-ok')['reason'] == 'duplicate_request'
    assert b.ack('ack-z')['reason'] == 'acknowledge_rejected'  # 이미 해제됨


def test_malformed_stop_still_latches_but_malformed_release_is_rejected():
    b = started_bench()
    assert b.race.handle_command({'op': 'stop'})['accepted']
    assert b.race.permission.latch == 'explicit_stop'
    assert not b.race.handle_command({'op': 'acknowledge'})['accepted']
    assert not b.race.handle_command(None)['accepted']
    assert not b.race.handle_command(dict(v=1, request='a', op='acknowledge', session=b.race.session,
        race_epoch=0, intervention=1, hazards_cleared=True, operator_reviewed=True, extra=1))['accepted']


def test_new_race_latches_resets_video_and_needs_ack_rearm_and_new_lights():
    b = started_bench()
    before = b.race.permission.intervention
    b.frame(green=True, submit_only=True)  # 이전 경기 작업이 진행 중
    result = b.command('new_race', 'race-2', session=b.race.session, race_epoch=0)
    assert result['accepted'] and result['race_epoch'] == 1
    assert b.race.permission.latch == 'new_race' and b.race.permission.intervention == before+1
    assert b.vision.epoch == b.race.bound_vision_epoch and not b.vision.signal.started
    assert b.vision.pump(b.now_s, wall_now=b.now_s+WALL) is None
    assert b.vision.stats['rejected_epoch'] == 1 and not b.race.permission.started
    assert b.ack()['accepted']
    for _ in range(3):
        b.frame()  # 전체 소등: 10/23 반례에서는 이전 출발이 넘어왔다.
        b.frame(green=True)
    assert not b.race.permission.started
    b.lights()
    assert b.race.permission.started
    b.advance()
    assert b.decision()['actuator_reason'] == 'mcu_rearm_required' and b.receiver.target_speed == 0.
    b.rearm()
    assert b.running()


def test_late_duplicate_or_stale_new_race_cannot_reset_following_race():
    b = started_bench()
    assert b.command('new_race', 'race-2', session=b.race.session, race_epoch=0)['accepted']
    assert b.ack()['accepted']
    b.rearm()
    b.lights()
    assert b.running()
    assert b.command('new_race', 'race-2', session=b.race.session, race_epoch=0)['reason'] == 'duplicate_request'
    assert b.command('new_race', 'race-2b', session=b.race.session, race_epoch=0)['reason'] == 'race_epoch_mismatch'
    assert b.command('new_race', 'race-2c', session='old', race_epoch=1)['reason'] == 'session_mismatch'
    assert b.race.permission.started and b.race.permission.latch is None and b.running()


def test_new_race_during_existing_latch_keeps_latched_and_old_epoch_ack_fails():
    b = started_bench()
    token = b.command('stop', 'stop-1')['stop_token']
    assert b.command('new_race', 'race-2', session=b.race.session, race_epoch=0)['accepted']
    assert b.race.permission.latch == 'new_race' and b.race.permission.intervention == token+1
    assert not b.command('acknowledge', 'old', session=b.race.session, race_epoch=0, intervention=token,
                         hazards_cleared=True, operator_reviewed=True)['accepted']
    assert b.ack()['accepted']
    b.rearm()
    b.advance()
    assert b.decision()['reason'] == 'wait_visual_start'
    b.lights()
    assert b.running()


@pytest.mark.parametrize('operation', ['clock_reset', 'legacy_reset'])
def test_clock_and_legacy_reset_are_fail_closed_new_race_with_latch(operation):
    b = started_bench()
    getattr(b.race, operation)(b.now_s)
    assert b.race.permission.race_epoch == 1 and b.race.permission.latch == operation
    assert not b.vision.signal.started and not b.race.permission.started
    b.lights()  # 래치 중: 출발 사실은 검출기에 남지만 정책 채택·움직임은 없다.
    snap = b.race.snapshot(b.now_s, b.now_s+WALL)
    assert snap['visual_start_fact'] and not snap['started'] and not snap['policy_allowed_if_sensors_ready']
    assert b.ack()['accepted']
    b.frame()
    assert b.race.permission.started
    assert b.decision()['actuator_reason'] == 'mcu_rearm_required'
    b.rearm()
    assert b.running()


def test_vision_fault_latches_and_cannot_be_acknowledged_or_reset_away():
    b = started_bench()
    b.vision.worker.error = 'fixture worker exited'
    b.frame()
    assert b.vision.fault and b.race.permission.latch == 'fault:vision'
    assert b.ack()['reason'] == 'fixed_fault_present'
    b.race.legacy_reset(b.now_s)
    assert b.vision.fault and b.ack()['reason'] == 'fixed_fault_present'
    b.rearm()
    b.advance()
    assert not b.decision()['allowed'] and b.receiver.target_speed == 0.


def test_physical_stop_held_blocks_ack_and_release_is_not_arm():
    b = started_bench()
    b.receiver.set_estop(True, b.now_us)
    b.publish()
    assert b.race.permission.latch.startswith('mcu:ESTOP_LATCHED')
    b.advance(220_000)
    assert b.ack()['reason'] == 'physical_stop_held'
    b.receiver.set_estop(False, b.now_us)
    b.advance()
    assert b.ack()['accepted'] and not b.decision()['allowed']
    assert b.receiver.state == State.ESTOP


def test_mcu_reboot_and_initially_latched_mcu_become_interventions():
    b = started_bench()
    b.receiver = Bench.make_receiver('rebooted-mcu')
    b.feedback_seq += 1
    b.receiver.update_motor_encoder(b.now_us, sequence=b.feedback_seq, capture_us=b.now_us, motor_counts=0)
    b.advance(220_000)
    assert b.race.permission.latch == 'mcu_boot_changed'
    assert b.publish(dict(b.statuses[0], now_us=b.now_us)) == 'retired_boot'
    assert b.ack()['accepted'] and b.control('ARM')[0]
    assert b.running()
    fresh = Bench()
    fresh.mcu_stop()
    restart = RaceSupervisor(VisionPipeline(InlineWorker(), signal_mode='official_4red_candidate'),
                             now_s=fresh.now_s)
    restart.observe_mcu(fresh.status(), fresh.now_s, fresh.now_s+WALL)
    assert restart.permission.latch == 'process_start' and restart.mcu_stop_seen


def test_mcu_observer_rejects_invalid_skewed_and_replayed_status():
    observer = McuObserver()
    good = dict(v=1, boot='b', generation=2, state='ARMED', now_us=1_000_000, estop_held=False)
    assert observer.update(dict(good, generation=-1), 1., WALL) == 'invalid'
    assert observer.update(dict(good, state='RUNNING'), 1., WALL) == 'invalid'
    assert observer.update(dict(good, estop_held=0), 1., WALL) == 'invalid'
    assert observer.update(good, 1.2, WALL) == 'clock_mismatch'
    assert observer.update(good, 1., WALL) == 'accepted' and observer.fresh(1.05, WALL+.05)
    assert observer.update(dict(good, generation=1, now_us=1_010_000), 1.01, WALL) == 'late'
    assert observer.update(dict(good, now_us=999_000), 1.01, WALL) == 'late'
    assert observer.update(good, 1.09, WALL+.09) == 'accepted'
    assert not observer.fresh(1.11, WALL+.11)  # 같은 원본 시각의 재전송은 신선도를 늘리지 않는다.
    assert not observer.fresh(.99, WALL)  # 소비자 시계 되감기
    assert not observer.fresh(1.05, WALL+5)  # 벽시계 정체 감시


def test_supervisor_rejects_invalid_fence_margin():
    with pytest.raises(ValueError, match='fence_margin'):
        RaceSupervisor(VisionPipeline(InlineWorker()), fence_margin_s=-1.)
