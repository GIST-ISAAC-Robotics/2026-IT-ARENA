"""B3/A1 combined event-contract bench; no ROS, guard bypass, plant or real output.

The harness is a proposed caller ordering, not an operational controller. Real pixel
recognition, race policy and A1 receiver are used; camera scheduling/encoder inputs
are synthetic. Receiver target values are NOT physical speed/stop measurements.
"""
from types import SimpleNamespace

import pytest

from arena_autonomy.race_permission import RacePermission
from arena_autonomy.vision_process import VisionPipeline, recognize
from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, ContractConfig, State
from audit_start_signal_layout import light_image


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


class CoupledBench:
    def __init__(self):
        self.receiver = ActuationReceiver(ContractConfig(feedback_mode='drive_motor_shaft',
            ticks_per_rev=28, gear_ratio=15., wheel_radius_m=.0325), boot_id='fixture-mcu')
        self.host = CommandProducer(client_id='fixture-host')
        self.race = RacePermission()
        self.vision = VisionPipeline(InlineWorker(), signal_mode='official_4red_candidate')
        self.now_us = 0
        self.feedback_seq = 0
        self.sensors_ready = True
        self.hold = False
        self.desired_speed = .3
        self.trace = []
        self.receiver.update_motor_encoder(0, sequence=0, capture_us=0, motor_counts=0)
        self.advance(220_000)

    def decision(self):
        return self.race.decide(sensors_ready=self.sensors_ready and self.vision.fault is None,
            actuator_ready=self.receiver.armed, temporary_hold=self.hold)

    def snapshot(self, event):
        decision = self.decision()
        self.trace.append(dict(event=event, now_us=self.now_us, race_epoch=self.race.race_epoch,
            vision_epoch=self.vision.epoch, intervention=self.race.intervention,
            started=self.race.started, policy_allowed=decision.allowed, policy_reason=decision.reason,
            mcu_state=self.receiver.state.value, mcu_reason=self.receiver.reason,
            mcu_generation=self.receiver.generation, target_speed_mps=self.receiver.target_speed))

    def advance(self, duration_us=10_000, *, feedback=True, host=True):
        assert duration_us > 0 and duration_us % 10_000 == 0
        for _ in range(duration_us//10_000):
            self.now_us += 10_000
            if feedback:
                self.feedback_seq += 1
                # Deliberately fixed count; this bench does not model actual vehicle movement.
                self.receiver.update_motor_encoder(self.now_us,sequence=self.feedback_seq,
                    capture_us=self.now_us,motor_counts=0)
            self.receiver.tick(self.now_us)
            if host and self.receiver.armed:
                # A1 reference only, NOT /drive/safe publication or independent LiDAR protection.
                offer = self.receiver.offer(self.now_us)
                speed = self.desired_speed if self.decision().allowed else 0.
                packet = self.host.drive(offer,speed_mps=speed,steering_rad=.1,
                    source_us=self.now_us+3_000_000,host_now_us=self.now_us+3_000_000)
                assert self.receiver.receive(packet,self.now_us)[0]

    def control(self, kind):
        packet = self.host.control(self.receiver.offer(self.now_us),kind)
        return self.receiver.receive(packet,self.now_us)

    def image(self, reds=0, green=False):
        image = light_image(reds=reds,green=green,diameter=12,paired=True)
        # Synthetic 1 ms capture-to-receipt delay, not a measured camera latency.
        # Avoid representing a zero-age image as future data after float conversion.
        ns=(self.now_us-1000)*1000
        sec,nanosec=divmod(ns,1_000_000_000)
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec,nanosec=nanosec)),
            data=image.tobytes(),height=image.shape[0],width=image.shape[1],step=image.shape[1]*3,encoding='rgb8')

    def frame(self, reds=0, green=False, *, submit_only=False):
        self.advance(100_000)
        now=self.now_us/1e6
        epoch=self.race.race_epoch  # Bind at submission, not when the result comes back.
        self.vision.offer(self.image(reds,green),wall_now=now+1000.)
        self.vision.pump(now,wall_now=now+1000.)
        if submit_only:
            return epoch
        accepted=self.vision.pump(now,wall_now=now+1000.)
        assert accepted is not None and self.vision.fault is None
        self.race.observe_start(race_epoch=epoch,visual_confirmed=self.vision.signal.started,
            capture_s=accepted['stamp'],now_s=now,max_age_s=self.vision.image_timeout)

    def start_pixels(self):
        for n in (0,1,2,3,4,4):
            self.frame(n)
            assert not self.race.started
        for i in range(3):
            self.frame(green=True)
            assert self.race.started is (i==2)

    def stop(self, *, deliver=True, reason='explicit_stop'):
        token=self.race.stop(reason)
        if deliver:
            assert self.receiver.receive({'v':1,'kind':'STOP'},self.now_us)[0]
        self.snapshot('stop_delivered' if deliver else 'stop_not_delivered')
        return token

    def acknowledge(self, token):
        return self.race.acknowledge(race_epoch=self.race.race_epoch,intervention=token,
            hazards_cleared=True,operator_reviewed=True)

    def new_race(self):
        # Test-only caller ordering: stop first, reset BOTH policy and visual accumulator.
        self.stop(reason='new_race')
        self.race.new_race()
        self.vision.reset()
        self.snapshot('new_race')


@pytest.mark.parametrize('arm_first',[True,False])
def test_pixels_and_separate_arm_are_both_required(arm_first):
    b=CoupledBench()
    if arm_first:
        assert b.control('ARM')[0]
        b.advance()
        assert b.receiver.target_speed==0. and not b.race.started
    b.start_pixels()
    if not arm_first:
        b.advance()
        assert b.receiver.state==State.DISARMED and b.receiver.target_speed==0.
        assert b.control('ARM')[0]
    b.advance()
    assert b.decision().allowed and b.receiver.state==State.ACTIVE


@pytest.mark.parametrize('hold_kind',['sensor','traffic'])
def test_temporary_hold_keeps_start_fact_and_recovers_without_arm(hold_kind):
    b=CoupledBench()
    b.start_pixels()
    assert b.control('ARM')[0]
    b.advance()
    generation=b.receiver.generation
    if hold_kind=='sensor': b.sensors_ready=False
    else: b.hold=True
    b.advance(40_000)
    assert b.receiver.state==State.ARMED and b.receiver.target_speed==0. and b.race.started
    assert b.receiver.generation==generation and b.race.latch is None
    b.sensors_ready=True
    b.hold=False
    b.advance()
    assert b.receiver.state==State.ACTIVE and b.receiver.generation==generation


@pytest.mark.parametrize('first',['race_ack','mcu_clear_arm'])
def test_stop_needs_race_ack_and_separate_mcu_clear_arm(first):
    b=CoupledBench()
    b.start_pixels()
    assert b.control('ARM')[0]
    b.advance()
    token=b.stop()
    b.advance(220_000)
    if first=='race_ack':
        assert b.acknowledge(token)
        b.advance()
        assert b.receiver.state==State.ESTOP and not b.decision().allowed
        assert b.control('CLEAR')[0]
        assert b.receiver.state==State.DISARMED and not b.decision().allowed
        assert b.control('ARM')[0]
    else:
        assert b.control('CLEAR')[0] and b.control('ARM')[0]
        b.advance()
        assert b.receiver.target_speed==0. and b.receiver.state==State.ARMED
        assert b.acknowledge(token)
    b.advance()
    assert b.receiver.state==State.ACTIVE and b.race.started


def test_delayed_positive_and_old_ack_cannot_undo_new_stop():
    b=CoupledBench()
    b.start_pixels()
    assert b.control('ARM')[0]
    b.advance()
    old=b.host.drive(b.receiver.offer(b.now_us),speed_mps=.3,steering_rad=.1,
        source_us=b.now_us+3_000_001,host_now_us=b.now_us+3_000_001)
    token=b.stop()
    b.stop()
    assert not b.acknowledge(token)
    assert b.receiver.receive(old,b.now_us)==(False,'session')
    assert b.receiver.state==State.ESTOP and b.receiver.target_speed==0.


def test_policy_stop_alone_is_not_mcu_stop_acknowledgement():
    b=CoupledBench()
    b.start_pixels()
    assert b.control('ARM')[0]
    b.advance()
    b.stop(deliver=False)
    assert not b.decision().allowed and b.receiver.target_speed>0.
    b.advance(110_000,host=False)
    assert b.receiver.state==State.FAULT and b.receiver.reason=='command_expired'
    assert b.receiver.target_speed==0.  # Command state only, not physical stop evidence.


def test_encoder_recovery_does_not_clear_a1_fault():
    b=CoupledBench()
    b.start_pixels()
    assert b.control('ARM')[0]
    b.advance()
    b.advance(60_000,feedback=False)
    assert b.receiver.state==State.FAULT and b.receiver.reason=='encoder_timeout'
    b.advance(220_000)
    assert b.receiver.state==State.FAULT and not b.decision().allowed
    assert b.control('CLEAR')[0] and b.control('ARM')[0]
    b.advance()
    assert b.receiver.state==State.ACTIVE


def test_physical_stop_release_is_not_clear_or_arm():
    b=CoupledBench()
    b.start_pixels()
    assert b.control('ARM')[0]
    b.advance()
    token=b.stop()
    b.receiver.set_estop(True,b.now_us)
    b.advance(220_000)
    assert b.acknowledge(token)
    assert b.control('CLEAR')==(False,'not_ready')
    b.receiver.set_estop(False,b.now_us)
    b.advance()
    assert b.receiver.state==State.ESTOP and not b.decision().allowed


def test_naive_policy_only_new_race_can_reuse_latched_visual_start_counterexample():
    b=CoupledBench()
    b.start_pixels()
    b.race.new_race()  # Deliberately wrong composition; no operational path uses this harness.
    assert not b.race.started
    b.frame()  # Entire signal dark, but old visual started latch is still true.
    assert b.race.started and b.vision.signal.started
    assert b.receiver.state==State.DISARMED  # Counterexample never emits real output.


def test_paired_new_race_reset_rejects_old_visual_work_and_green_only():
    b=CoupledBench()
    b.start_pixels()
    b.frame(submit_only=True)
    b.new_race()
    now=b.now_us/1e6
    assert b.vision.pump(now,wall_now=now+1000.) is None
    assert b.vision.stats['rejected_epoch']==1 and not b.race.started
    token=b.race.intervention
    b.advance(220_000)
    assert b.acknowledge(token) and b.control('CLEAR')[0] and b.control('ARM')[0]
    for _ in range(3):
        b.frame(green=True)
    assert not b.race.started and b.receiver.target_speed==0.
    b.start_pixels()
    b.advance()
    assert b.receiver.state==State.ACTIVE


def test_vision_fault_blocks_even_after_policy_ack_and_mcu_arm():
    b=CoupledBench()
    b.start_pixels()
    assert b.control('ARM')[0]
    b.advance()
    b.vision.worker.error='fixture worker exited'
    assert b.vision.pump(b.now_us/1e6,wall_now=b.now_us/1e6+1000.) is None
    token=b.stop(reason='fault:vision')
    b.advance(220_000)
    assert b.acknowledge(token) and b.control('CLEAR')[0] and b.control('ARM')[0]
    b.advance()
    assert b.vision.fault and b.receiver.target_speed==0. and not b.decision().allowed
    b.vision.reset()
    assert b.vision.fault  # Resetting frame state must not resurrect a dead worker.
