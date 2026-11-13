"""장치/ROS 없는 두 단일 슬롯 인계의 소유권·취소·정리 반례."""
import copy
import math

import pytest

from arena_autonomy.runtime_dual_delivery import RuntimeDualDelivery
from arena_autonomy.runtime_dual_state import RuntimeDualProcess


def unused_factory(stage, header):
    raise AssertionError('fake lanes must not create a calculation process')


class FakeLane:
    def __init__(self, stage, header, **kwargs):
        self.stage = stage
        self.inflight = self.completed = None
        self.connection = object()
        self.closed = False
        self.close_count = 0

    def offer(self, data, generation, sequence, reset_reason=None):
        assert self.inflight is None
        self.inflight = dict(data=copy.deepcopy(data), generation=generation, sequence=sequence,
                             stage=self.stage, reset_reason=reset_reason)

    def complete(self, output):
        assert self.inflight is not None
        self.completed = dict(stage=self.stage, generation=self.inflight['generation'],
                              sequence=self.inflight['sequence'], output=copy.deepcopy(output),
                              began_wall_s=1., completed_wall_s=1.01)

    def collect(self):
        result, self.completed = self.completed, None
        if result is not None:
            self.inflight = None
        return result

    def close(self):
        self.close_count += 1
        self.closed = True
        return dict(stage=self.stage, forced=False, exitcode=0, alive=False)


def make_worker(lane_factory=FakeLane):
    return RuntimeDualProcess({}, evaluator_factory=unused_factory, supported=lambda output: output[0] > 0,
                              lane_factory=lane_factory)


def snapshot(ref=1., first=.9):
    return dict(reference_s=ref, epoch=0, side='left', previous_steering_rad=.1, dt_s=.1,
                scan=dict(representation='native_measurements_v1',
                          measurements=dict(source_epoch=0, acquired_at_s=[first, first+.01])))


def plan(speed=.5):
    return [speed, .2, dict(handoff_state='FOLLOW', reason='handoff_follow', normal=dict(
        mode='g1_follow', preferred_wall='left', path_wall='left', selector_mode='SOLO',
        selector_choice='baseline', path_source_s=.9))]


def start_current(w):
    s = snapshot()
    assert w.offer_plan(s)
    w.plan.complete(plan())
    assert len(w.collect()) == 1
    assert w.offer_next_current(snapshot(1.1, 1.), now_s=1.11)
    return s


def test_plan_and_current_have_separate_owned_slots_without_queue():
    w = make_worker()
    s = start_current(w)
    assert w.offer_plan(snapshot(1.2, 1.1))
    assert w.current.inflight is not None and w.plan.inflight is not None
    assert not w.offer_plan(snapshot(1.3, 1.2))
    assert not w.offer_next_current(snapshot(1.2, 1.1), now_s=1.21)
    s['side'] = 'right'
    assert w.current.inflight['data']['submitted']['side'] == 'left'
    assert 'ranges_m' not in w.current.inflight['data']['submitted']['scan']['measurements']
    assert w.current.inflight['data']['current']['reference_s'] == 1.1
    w.close()


def test_stop_keeps_inflight_bytes_until_both_late_results_are_discarded():
    w = make_worker(); start_current(w)
    assert w.offer_plan(snapshot(1.2, 1.1))
    before = copy.deepcopy((w.plan.inflight, w.current.inflight))
    w.invalidate('explicit_stop')
    assert (w.plan.inflight, w.current.inflight) == before
    w.plan.complete(plan())
    w.current.complete(dict(speed_mps=.4, steering_rad=.2, reference_s=1.1))
    assert w.collect() == [] and w.old_generation == 2
    assert w.ready is w.saved_plan is w.retained is None
    assert w.offer_plan(snapshot(1.3, 1.2))
    assert w.plan.inflight['generation'] == 1
    w.close()


@pytest.mark.parametrize('now,ref,first', [(1.09, 1.1, 1.), (1.21, 1.1, 1.),
                                         (1.21, 1.15, .9), (math.nan, 1.1, 1.)])
def test_current_submission_rejects_clock_or_remaining_budget(now, ref, first):
    w = make_worker()
    assert w.offer_plan(snapshot())
    w.plan.complete(plan()); w.collect()
    assert not w.offer_next_current(snapshot(ref, first), now_s=now)
    assert w.current.inflight is None
    w.close()


def test_last_published_reference_is_not_new_current_evidence():
    w = make_worker()
    w.last_application = 1.1
    assert not w.offer_next_current(snapshot(1.1, 1.), now_s=1.11)
    w.close()


def test_native_submission_aligns_once_and_never_reuses_same_source():
    w = make_worker(); d = RuntimeDualDelivery(w, lambda: 1.12)
    assert not d.offer(snapshot(1., .9))
    assert d.offer(snapshot(1.1, 1.))
    w.plan.complete(plan(0.)); w.collect()
    w.discard_plan((w.saved_plan['generation'], w.saved_plan['sequence']))
    assert not d.offer(snapshot(1.2, 1.))
    assert d.counts['submitted'] == 1
    d.invalidate()
    assert not d.offer(snapshot(1.1, 1.))
    w.close()


def test_failure_of_one_lane_still_closes_both_and_reports_no_single_process():
    w = make_worker()
    def failed():
        w.current.close_count += 1
        raise RuntimeError('cleanup failed')
    w.current.close = failed
    report = w.close()
    assert report['forced'] and report['alive'] and report['exitcode'] is None
    assert w.plan.closed and w.plan.close_count == w.current.close_count == 1
    assert w.close() is report
    assert w.plan.close_count == w.current.close_count == 1
    assert not hasattr(w, 'process')


def test_current_startup_failure_closes_already_owned_plan():
    made = []
    def factory(stage, header, **kwargs):
        if stage == 'current':
            raise RuntimeError('current startup failed')
        lane = FakeLane(stage, header, **kwargs); made.append(lane); return lane
    with pytest.raises(RuntimeError, match='startup'):
        make_worker(factory)
    assert len(made) == 1 and made[0].closed


def test_fault_clears_permission_and_delivery_surfaces_it():
    w = make_worker(); start_current(w)
    w.retained = {'sentinel': 'must not survive'}
    def fail():
        raise RuntimeError('worker fault')
    w.current.collect = fail
    d = RuntimeDualDelivery(w, lambda: 1.12)
    result = d.poll(1.12, previous_steering_rad=.1, side='left', motion_epoch=0, sensor_epoch=0)
    assert result['kind'] == 'fault' and 'worker fault' in result['reason']
    assert w.ready is w.retained is None
    w.close()
