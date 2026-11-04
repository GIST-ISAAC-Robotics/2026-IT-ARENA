"""같은 경기 복구 RESET/START 상태기계. ROS·센서 없이 경계·취소·거절만 확인한다."""
from types import SimpleNamespace

import pytest

from arena_autonomy.race_session import RaceSupervisor, RESUME_INPUTS
from arena_autonomy.vision_process import VisionPipeline


class FakeVision:
    def __init__(self):
        self.epoch = 0
        self.fault = None
        self.signal = SimpleNamespace(started=True)
        self.boundaries = []

    def reset(self):
        self.epoch += 1
        self.signal = SimpleNamespace(started=False)

    def begin_input_boundary(self, boundary):
        self.epoch += 1
        self.boundaries.append(boundary)


ALL = {name: True for name in RESUME_INPUTS}
ROUTE = dict(ready=True, reason='local_follow')


@pytest.mark.parametrize('wrapped', [False, True])
def test_resume_accepts_current_front_gap_wait_but_keeps_zero_output(wrapped):
    from arena_autonomy.local_pursuit import LocalPursuit
    node = SimpleNamespace(race=SimpleNamespace(resume={}), side='left', now_s=lambda: 4.)
    detail = dict(reason='follow_gap_hold', path_wall='right', behavior='HOLD')
    if wrapped:
        detail = dict(reason='handoff_follow', handoff_state='FOLLOW', normal=detail, behavior='HOLD')
    output = LocalPursuit.prepared_output(node, (0., .1, detail))
    assert node.resume_route['ready']
    assert node.resume_route['route_reason'] == 'follow_gap_hold'
    assert node.resume_route['path_wall'] == 'right'
    assert output[:2] == (0., .1)
    assert output[2]['reason'] == 'resume_preparing'


@pytest.mark.parametrize('state,reason,normal', [
    ('FOLLOW', 'handoff_follow', {'reason': 'follow_input_stop'}),
    ('FOLLOW', 'handoff_follow', {'reason': 'boundary_unconfirmed'}),
    ('FOLLOW', 'handoff_follow', None),
    ('BYPASS', 'handoff_follow', {'reason': 'follow_gap_hold'}),
    ('REACQUIRE', 'handoff_reacquire', {'reason': 'follow_gap_hold'}),
    ('FOLLOW', 'handoff_entry', {'reason': 'follow_gap_hold'}),
])
def test_resume_does_not_use_unknown_path_or_old_nested_normal(state, reason, normal):
    from arena_autonomy.local_pursuit import LocalPursuit
    node = SimpleNamespace(race=SimpleNamespace(resume={}), side='left', now_s=lambda: 4.)
    detail = dict(reason=reason, handoff_state=state, normal=normal)
    assert LocalPursuit.prepared_output(node, (0., .1, detail))[0] == 0.
    assert not node.resume_route['ready']


def test_resume_adapter_does_not_change_ordinary_driving_output():
    from arena_autonomy.local_pursuit import LocalPursuit
    node = SimpleNamespace(race=SimpleNamespace(resume=None))
    output = (0., .1, {'reason': 'handoff_follow', 'normal': {'reason': 'follow_gap_hold'}})
    assert LocalPursuit.prepared_output(node, output) is output


def status(generation, state='ARMED', now=1., boot='b', held=False):
    return dict(v=1, boot=boot, generation=generation, state=state, reason='x', now_us=round(now*1e6), estop_held=held)


def started():
    """출발한 경기가 주행 중인 상태까지 만든다."""
    vision = FakeVision()
    race = RaceSupervisor(vision, session='s', now_s=.1)
    race.observe_mcu(status(0, 'DISARMED', .2), .2, 1.)
    race.observe_mcu(status(1, 'ARMED', .3), .3, 1.)
    assert race.handle_command(dict(v=1, request='ack', op='acknowledge', session='s', race_epoch=0, intervention=1,
                                    hazards_cleared=True, operator_reviewed=True), .3, 1.)['accepted']
    assert race.observe_vision(dict(stamp=.31), vision.epoch, .32)
    assert race.decide(sensors_ready=True, now_s=.35, wall_s=1.)['allowed']
    return race


def bound(race, op, request, **extra):
    return dict(v=1, request=request, op=op, session=race.session, race_epoch=race.permission.race_epoch,
                intervention=race.permission.intervention, **extra)


def stopped_and_reset():
    race = started()
    race.handle_command(dict(v=1, request='st', op='stop'), 1., 1.)
    assert race.handle_command(bound(race, 'recovery_reset', 'r1'), 2., 2.)['accepted']
    return race


def arm_after_boundary(race, now, wall=3.):
    race.observe_mcu(status(2, 'ESTOP_LATCHED', now), now, wall)
    race.observe_mcu(status(3, 'DISARMED', now+.03), now+.03, wall)
    race.observe_mcu(status(4, 'ARMED', now+.06), now+.06, wall)


def test_first_start_without_signal_is_rejected_and_keeps_latch():
    race = RaceSupervisor(FakeVision(), session='s', now_s=.1)
    result = race.handle_command(bound(race, 'resume_start', 'x'), 1., 1.)
    assert not result['accepted'] and result['reason'] == 'race_not_started'
    assert race.handle_command(bound(race, 'recovery_reset', 'y'), 1., 1.)['reason'] == 'race_not_started'
    assert race.permission.latch == 'process_start' and race.resume is None


def test_recovery_reset_keeps_race_and_only_bumps_vision_generation():
    race = started()
    epoch, vision_epoch = race.permission.race_epoch, race.bound_vision_epoch
    race.handle_command(dict(v=1, request='st', op='stop'), 1., 1.)
    assert not race.decide(sensors_ready=True, now_s=1.01, wall_s=1.)['allowed']
    result = race.handle_command(bound(race, 'recovery_reset', 'r1'), 2., 2.)
    assert result['accepted'] and result['boundary_s'] == 2.
    assert race.permission.started and race.permission.race_epoch == epoch and race.permission.latch == 'explicit_stop'
    assert race.bound_vision_epoch == vision_epoch+1 and race.vision.signal.started is True
    assert race.snapshot(2., 2.)['recovery_ready'] is True


def test_reset_requires_active_stop_and_exact_intervention():
    race = started()
    assert race.handle_command(bound(race, 'recovery_reset', 'a'), 1., 1.)['reason'] == 'no_active_intervention'
    race.handle_command(dict(v=1, request='st', op='stop'), 1., 1.)
    wrong = bound(race, 'recovery_reset', 'b')
    wrong['intervention'] -= 1
    assert race.handle_command(wrong, 1., 1.)['reason'] == 'intervention_mismatch'


def test_fault_requires_recovery_reset_of_current_intervention():
    race = started()
    race.handle_command(dict(v=1, request='st', op='stop'), 1., 1.)
    race.observe_mcu(status(2, 'FAULT_LATCHED', 1.1), 1.1, 1.1)
    assert race.handle_command(bound(race, 'resume_start', 'x'), 2., 2.)['reason'] == 'recovery_reset_required'
    race.handle_command(bound(race, 'recovery_reset', 'r1'), 2., 2.)
    race.handle_command(dict(v=1, request='st2', op='stop'), 2.1, 2.1)  # 새 STOP은 RESET을 무효화한다.
    assert race.handle_command(bound(race, 'resume_start', 'y'), 2.2, 2.2)['reason'] == 'recovery_reset_required'


@pytest.mark.parametrize('reset_first', [False, True])
def test_start_accept_is_not_permission_until_inputs_route_and_rearm(reset_first):
    race = stopped_and_reset() if reset_first else started()
    if not reset_first:
        race.handle_command(dict(v=1, request='st', op='stop'), 2.8, 2.8)
        race.observe_mcu({**status(2, 'ESTOP_LATCHED', 2.9), 'reason': 'software_stop'}, 2.9, 2.9)
        snap = race.snapshot(2.91, 2.91)
        assert snap['resume_start_ready'] and not snap['recovery_ready']
    race_id, epoch = race.race_id, race.permission.race_epoch
    result = race.handle_command(bound(race, 'resume_start', 'go'), 3., 3.)
    assert result['accepted'] and result['reason'] == 'preparing_not_motion_permission'
    assert race.permission.latch == 'explicit_stop' and race.rearm_required
    assert race.rearm_fence_s == 3. and race.vision.boundaries[-1] == 3.
    assert race.advance_resume(3.1, 3.1, inputs={**ALL, 'rgb': False}, route=ROUTE) is None
    assert race.resume['phase'] == 'inputs' and race.resume['missing'] == ['rgb']
    assert race.advance_resume(3.2, 3.2, inputs=ALL, route=None) is None and race.resume['phase'] == 'route'
    assert race.advance_resume(3.3, 3.3, inputs=ALL, route=ROUTE) is None and race.resume['phase'] == 'mcu'
    assert not race.decide(sensors_ready=True, now_s=3.3, wall_s=3.3)['allowed']
    arm_after_boundary(race, 3.4, 3.4)
    assert not race.rearm_required
    race.operator_link('go', 3.45)
    outcome = race.advance_resume(3.47, 3.47, inputs=ALL, route=ROUTE)
    assert outcome['outcome'] == 'permitted' and race.permission.latch is None and race.resume is None
    assert race.decide(sensors_ready=True, now_s=3.47, wall_s=3.47)['allowed']
    assert race.race_id == race_id and race.permission.race_epoch == epoch and race.permission.started


def test_rearm_before_boundary_does_not_count():
    race = stopped_and_reset()
    race.observe_mcu(status(2, 'ESTOP_LATCHED', 2.5), 2.5, 2.5)
    race.observe_mcu(status(3, 'DISARMED', 2.6), 2.6, 2.6)
    race.observe_mcu(status(4, 'ARMED', 2.7), 2.7, 2.7)
    race.handle_command(bound(race, 'resume_start', 'go'), 3., 3.)
    race.observe_mcu(status(4, 'ARMED', 3.1), 3.1, 3.1)
    assert race.rearm_required
    assert race.advance_resume(3.2, 3.2, inputs=ALL, route=ROUTE) is None and race.resume['phase'] == 'mcu'


@pytest.mark.parametrize('cause,reason', [
    (lambda r: r.handle_command(dict(v=1, request='new', op='stop'), 3.1, 3.1), 'explicit_stop'),
    (lambda r: r.advance_resume(6.1, 6.1, inputs={}, route=None), None),
    (lambda r: r.advance_resume(3.2, 4.2, inputs=ALL, route=ROUTE), 'operator_link_lost'),
    (lambda r: r.observe_mcu(status(9, 'ESTOP_LATCHED', 3.1, held=True), 3.1, 3.1) and
               r.advance_resume(3.2, 3.2, inputs=ALL, route=ROUTE), 'physical_stop_held'),
    (lambda r: r.observe_mcu(status(0, 'DISARMED', 3.1, boot='c'), 3.1, 3.1), 'mcu_boot_changed'),
    (lambda r: r.new_race(3.1), 'new_race'),
])
@pytest.mark.parametrize('reset_first', [False, True])
def test_pending_start_is_cancelled_and_never_permits_later(cause, reason, reset_first):
    race = stopped_and_reset() if reset_first else started()
    if not reset_first:
        race.handle_command(dict(v=1, request='st', op='stop'), 2.8, 2.8)
    race.handle_command(bound(race, 'resume_start', 'go'), 3., 3.)
    cause(race)
    assert race.resume is None and race.permission.latch is not None
    assert race.last_resume['outcome'] == 'cancelled'
    assert reason is None or race.last_resume['reason'] == reason
    # 뒤늦은 입력·연결 유지 신호가 도착해도 되살아나지 않는다.
    race.operator_link('go', 3.3)
    assert race.advance_resume(3.3, 3.3, inputs=ALL, route=ROUTE) is None
    assert not race.decide(sensors_ready=True, now_s=3.3, wall_s=3.3)['allowed']


@pytest.mark.parametrize('state,reason,held', [
    ('FAULT_LATCHED', 'control_deadline', False),
    ('ESTOP_LATCHED', 'physical_stop', False),
    ('ESTOP_LATCHED', 'software_stop', True),
])
def test_ordinary_start_does_not_stand_in_for_fault_or_physical_recovery(state, reason, held):
    race = started()
    race.handle_command(dict(v=1, request='st', op='stop'), 1., 1.)
    race.observe_mcu({**status(2, state, 1.1, held=held), 'reason': reason}, 1.1, 1.1)
    assert not race.snapshot(1.12, 1.12)['resume_start_ready']
    before = race.boundary_count
    assert not race.handle_command(bound(race, 'resume_start', 'go'), 1.12, 1.12)['accepted']
    assert race.boundary_count == before and race.resume is None


def test_cancel_start_matches_only_current_request_and_timeout_names_phase():
    race = stopped_and_reset()
    race.handle_command(bound(race, 'resume_start', 'go'), 3., 3.)
    cancel = dict(v=1, request='c1', op='cancel_start', session='s', race_epoch=0, start_request='old', reason='link')
    assert race.handle_command(cancel, 3.1, 3.1)['reason'] == 'no_matching_pending_start' and race.resume
    race.advance_resume(6.2, 3.2, inputs=ALL, route=None)
    assert race.last_resume['reason'].startswith('input_timeout')
    race.handle_command(bound(race, 'resume_start', 'again'), 7., 7.)
    assert race.resume['request'] == 'again'  # 취소 뒤에는 새 명시 요청만 가능
    assert race.handle_command({**cancel, 'request': 'c2', 'start_request': 'again'}, 7.1, 7.1)['accepted']
    assert race.resume is None


def test_acknowledge_cannot_bypass_recovery_boundary():
    race = stopped_and_reset()
    payload = bound(race, 'acknowledge', 'ack2', hazards_cleared=True, operator_reviewed=True)
    assert race.handle_command(payload, 3., 3.)['reason'] == 'recovery_requires_start'
    assert race.permission.latch == 'explicit_stop'


def test_physical_stop_rejects_start_before_boundary():
    race = stopped_and_reset()
    race.observe_mcu(status(5, 'ESTOP_LATCHED', 2.5, held=True), 2.5, 2.5)
    assert race.handle_command(bound(race, 'resume_start', 'go'), 2.6, 2.6)['reason'] == 'physical_stop_held'


def test_markers_frozen_while_stopped_after_start_but_active_during_resume():
    race = started()
    assert race.markers_apply()
    race.handle_command(dict(v=1, request='st', op='stop'), 1., 1.)
    assert not race.markers_apply()
    race.handle_command(bound(race, 'recovery_reset', 'r1'), 2., 2.)
    assert not race.markers_apply()
    race.handle_command(bound(race, 'resume_start', 'go'), 3., 3.)
    assert race.markers_apply()


def test_new_race_clears_recovery_and_is_separate():
    race = stopped_and_reset()
    first = race.race_id
    race.handle_command(dict(v=1, request='nr', op='new_race', session='s', race_epoch=0), 3., 3.)
    assert race.permission.started is False and race.race_id != first and race.start_source is None
    assert race.handle_command(bound(race, 'resume_start', 'go'), 3.1, 3.1)['reason'] == 'race_not_started'


def restore(race, request='rs', **change):
    return race.handle_command(bound(race, 'restore_race', request, race_id='oldrace', side='right',
                                     same_race_confirmed=True) | change, 1., 1.)


def test_restore_requires_fresh_process_and_explicit_confirmation():
    fresh = RaceSupervisor(FakeVision(), session='s2', now_s=.1)
    assert restore(fresh, same_race_confirmed=False)['reason'] == 'schema' and not fresh.permission.started
    assert restore(fresh, 'rs2', side='middle')['reason'] == 'schema'
    result = restore(fresh, 'rs3')
    assert result['accepted'] and result['side'] == 'right' and fresh.permission.started
    assert fresh.start_source == 'restored_operator_confirmed' and fresh.race_id == 'oldrace'
    assert fresh.permission.latch == 'process_start'  # 복원만으로 움직이지 않는다.
    assert fresh.vision.signal.started is True or fresh.vision.boundaries  # 경계가 새로 생겼다.
    assert restore(fresh, 'rs4')['reason'] == 'restore_requires_fresh_process'
    stopped = started()
    stopped.handle_command(dict(v=1, request='st', op='stop'), 1., 1.)
    assert restore(stopped, 'rs5')['reason'] == 'restore_requires_fresh_process'


def test_restored_race_still_needs_new_inputs_route_and_arm():
    race = RaceSupervisor(FakeVision(), session='s2', now_s=.1)
    race.observe_mcu(status(2, 'FAULT_LATCHED', .2), .2, 1.)
    restore(race)
    assert race.handle_command(bound(race, 'resume_start', 'go'), 3., 3.)['accepted']
    assert race.advance_resume(3.1, 3.1, inputs={}, route=None) is None
    assert not race.decide(sensors_ready=True, now_s=3.1, wall_s=3.1)['allowed']


class Worker:
    def __init__(self):
        self.submitted, self.result = [], None

    def poll(self):
        result, self.result = self.result, None
        return result

    def submit(self, request, data):
        self.submitted.append(request)

    def close(self):
        return None


def image(stamp):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=int(stamp), nanosec=round((stamp % 1)*1e9))),
                           height=2, width=2, step=6, encoding='rgb8', data=bytes(12))


def test_vision_boundary_drops_late_frames_and_inflight_result_but_keeps_signal():
    worker = Worker()
    vision = VisionPipeline(worker, signal_mode='temporary_3light')
    vision.signal.started = True
    vision.offer(image(1.), 10.)
    vision.pump(1.05, 10.05)
    request = worker.submitted[-1]
    vision.begin_input_boundary(2.)
    worker.result = dict(epoch=request['epoch'], sequence=request['sequence'], stamp=request['stamp'],
                         signal=dict(started=True), signal_mode='temporary_3light', ids=[30], stages_ms={}, duration_ms=1.)
    assert vision.pump(2.05, 10.1) is None and vision.ids == [] and vision.stats['rejected_epoch'] == 1
    vision.offer(image(1.9), 10.2)  # 경계 전 취득 영상이 경계 뒤에 도착
    assert vision.pending is None and vision.stats['before_boundary'] == 1
    vision.offer(image(2.2), 10.3)
    assert vision.pending is not None and vision.signal.started is True
