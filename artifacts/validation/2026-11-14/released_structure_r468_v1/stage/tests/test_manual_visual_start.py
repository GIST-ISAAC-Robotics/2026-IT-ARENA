"""운영자 신호 확인의 순수 계약 회귀. ROS/실차 구동과 실제 출발 신호는 실행하지 않는다."""
import copy
from types import SimpleNamespace

import pytest

from arena_autonomy.race_session import RaceSupervisor
from arena_autonomy.race_ops_core import OpsCore
from arena_autonomy.vision_process import VisionPipeline, recognize
from test_race_recovery import FakeVision, status, bound
from test_vision_process import WorkerDouble, begin, message


def waiting():
    vision = FakeVision()
    vision.signal.started = False
    vision.signal_detection_enabled = True
    race = RaceSupervisor(vision, session='s', now_s=.1)
    race.observe_mcu(status(0, 'DISARMED', .2), .2, 1.)
    race.observe_mcu(status(1, 'ARMED', .3), .3, 1.)
    assert race.handle_command(bound(race, 'acknowledge', 'prepare',
        hazards_cleared=True, operator_reviewed=True), .3, 1.)['accepted']
    return race


def command(race, request='manual', check=lambda: True, **changes):
    payload = bound(race, 'manual_visual_start', request, signal_confirmed=True)
    return race.handle_command({**payload, **changes}, .32, 1., manual_start_check=check)


def test_manual_records_operator_fact_without_pixel_or_arm_or_epoch_change():
    race = waiting()
    before = race.permission.race_epoch, race.permission.intervention, race.mcu.identity(), race.vision.epoch
    assert command(race)['accepted']
    snap = race.snapshot(.32, 1.)
    assert race.start_source == 'operator_confirmed' and race.permission.started
    assert not snap['visual_start_fact'] and not race.vision.signal.started
    assert race.permission.last_visual_stamp is None and race.counts['visual_accepted'] == 0
    assert not race.vision.signal_detection_enabled
    assert before == (race.permission.race_epoch, race.permission.intervention, race.mcu.identity(), race.vision.epoch)
    assert any(e['kind'] == 'manual_visual_start' and e['request'] == 'manual' for e in race.events)


@pytest.mark.parametrize('changes,reason', [
    ({'session': 'old'}, 'session_mismatch'), ({'race_epoch': 4}, 'race_epoch_mismatch'),
    ({'intervention': 0}, 'intervention_mismatch'), ({'signal_confirmed': False}, 'signal_confirmation_required'),
    ({'signal_confirmed': 1}, 'signal_confirmation_required'), ({'extra': True}, 'schema'),
])
def test_manual_uses_existing_exact_schema_and_identity(changes, reason):
    race = waiting()
    assert command(race, **changes)['reason'] == reason
    assert not race.permission.started and race.vision.signal_detection_enabled


@pytest.mark.parametrize('case', ['stop', 'fault', 'physical_stop', 'running', 'stale_state', 'no_check', 'old_vision'])
def test_manual_cannot_release_stops_or_accept_nonwaiting_state(case):
    race, check = waiting(), lambda: True
    if case == 'stop': race.stop(now_s=.31)
    if case == 'fault': race.vision.fault = 'failed'
    if case == 'physical_stop': race.mcu.latest['estop_held'] = True
    if case == 'running': race.permission.started = True
    if case == 'stale_state': check = lambda: False
    if case == 'no_check': check = None
    if case == 'old_vision': race.vision.epoch += 1
    before = race.permission.latch, race.permission.started, race.rearm_required
    assert not command(race, check=check)['accepted']
    assert before == (race.permission.latch, race.permission.started, race.rearm_required)
    assert race.vision.signal_detection_enabled


def test_replay_and_second_request_never_create_second_start():
    race = waiting()
    assert command(race)['accepted']
    assert command(race)['reason'] == 'duplicate_request'
    assert command(race, 'second')['reason'] == 'race_already_started'
    assert sum(e['kind'] == 'manual_visual_start' for e in race.events) == 1


def test_start_fact_does_not_bypass_sensor_or_mcu_gate():
    race = waiting()
    race.rearm_required = True  # WAIT_GREEN can precede MCU readiness; this is a fact, not ARM.
    assert command(race)['accepted']
    assert not race.decide(sensors_ready=True, now_s=.32, wall_s=1.)['allowed']
    assert race.rearm_required
    race.rearm_required = False
    assert not race.decide(sensors_ready=False, now_s=.32, wall_s=1.)['allowed']
    assert race.decide(sensors_ready=True, now_s=.32, wall_s=1.)['allowed']


def test_manual_same_race_stop_start_keeps_fact_but_new_race_clears_it():
    race = waiting()
    assert command(race)['accepted']
    race.stop(now_s=.4)
    assert race.permission.started and race.start_source == 'operator_confirmed'
    assert not command(race, 'while-stopped')['accepted']
    result = race.handle_command(bound(race, 'resume_start', 'resume'), .5, 1.)
    assert result['accepted'] and race.resume is not None
    assert race.permission.started and not race.vision.signal_detection_enabled
    assert race.start_source == 'operator_confirmed' and not race.vision.signal.started
    race.new_race(.6)
    assert not race.permission.started and race.start_source is None
    assert race.permission.latch == 'new_race'


def ops_waiting():
    core = OpsCore('car1')
    binding = dict(session='s', race_id='r', race_epoch=0, intervention=2)
    core.update_status(dict(state='WAIT_GREEN', race=dict(binding, latch=None, started=False, resume=None)), 10.)
    return core, binding


@pytest.mark.parametrize('state', ['WAIT_GREEN', 'RUNNING', 'SENSOR_STOP', 'ACTUATOR_WAIT', 'DISABLED', 'RACE_LATCHED'])
def test_button_is_available_only_in_wait_green(state):
    core, view = ops_waiting()
    core.status['state'] = state
    assert core.view(10.1, 10.)['controls']['manual_visual_start']['allowed'] is (state == 'WAIT_GREEN')


def test_manual_uses_existing_confirmation_replay_and_operation_log():
    core, view = ops_waiting()
    assert core.submit('manual_visual_start', 'missing', view, 'c', None, 10.1)['adapter_reason'] == 'signal_confirmation_required'
    assert core.submit('manual_visual_start', 'old', {**view, 'race_epoch': 9}, 'c', 'SIGNAL_CONFIRMED', 10.1)['adapter_reason'] == 'stale_view'
    first = core.submit('manual_visual_start', 'manual', view, 'c', 'SIGNAL_CONFIRMED', 10.1)
    assert first['stage'] == 'queued'
    assert core.submit('manual_visual_start', 'manual', view, 'c', 'SIGNAL_CONFIRMED', 10.2) == first
    assert core.next_action()['request'] == 'manual' and core.next_action() is None
    result = core.result(dict(request='manual', op='manual_visual_start', accepted=True,
        reason='operator_confirmed_start_fact', start_source='operator_confirmed'), 10.2)
    assert result['policy']['start_source'] == 'operator_confirmed'
    assert any(e.get('request') == 'manual' and 'operator_confirmed' in e['text'] for e in core.events)
    assert not core.view(12., 12.)['controls']['manual_visual_start']['allowed']


@pytest.mark.parametrize('mode', ['temporary_3light', 'official_4red_candidate',
                                 'official_4red_local_candidate', 'official_marker_late_candidate'])
def test_operator_start_skips_signal_work_but_keeps_markers(monkeypatch, mode):
    import arena_autonomy.vision_process as vp
    class Signal:
        def __init__(self): self.started = False
        def update(self, *args): raise AssertionError('signal detector must stay idle')
    seen = []
    monkeypatch.setattr(vp, 'make_start_signal', lambda _: Signal())
    monkeypatch.setattr(vp, 'marker_ids', lambda *args: seen.append('markers') or [30])
    monkeypatch.setattr(vp, 'marker_observations', lambda *args: seen.append('markers') or [(30, None)])
    msg = message()
    request = dict(epoch=0, sequence=1, stamp=1., signal={}, signal_mode=mode, signal_detection_enabled=False,
                   image={key: getattr(msg, key) for key in ('height', 'width', 'step', 'encoding')})
    result = recognize(request, msg.data)
    assert seen == ['markers'] and result['ids'] == [30] and not result['signal']['started']


def test_inflight_signal_cannot_overwrite_manual_fact_and_same_race_reset_keeps_skip():
    worker = WorkerDouble()
    pipe = VisionPipeline(worker)
    begin(pipe)
    worker.finish()
    worker.result['signal']['started'] = True
    worker.result['ids'] = [30]
    pipe.signal_detection_enabled = False
    assert pipe.pump(1.02, 10.02) is not None
    assert not pipe.signal.started and pipe.ids == [30]
    pipe.begin_input_boundary(1.1)
    assert not pipe.signal_detection_enabled
    begin(pipe, 1.2, 10.2)
    assert worker.sent[-1][0]['signal_detection_enabled'] is False
    pipe.reset()
    assert pipe.signal_detection_enabled and not pipe.signal.started
