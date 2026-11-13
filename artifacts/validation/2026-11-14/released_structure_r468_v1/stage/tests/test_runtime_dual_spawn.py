"""작은 명시 evaluator의 실제 spawn/늦은 결과/종료 smoke. ROS·장치를 열지 않는다."""
import multiprocessing as mp
import os
import time

import pytest

from arena_autonomy.runtime_dual_process import Lane
from arena_autonomy.runtime_dual_state import RuntimeDualProcess


class SmokeEvaluator:
    def __init__(self, stage, header):
        self.stage, self.header = stage, header
        self.resets = 0

    def reset(self, reason):
        self.resets += 1

    def __call__(self, data):
        snapshot = data if self.stage == 'plan' else data['current']
        if snapshot.get('smoke_hold'):
            self.header[self.stage+'_started'].set()
            if not self.header['release'].wait(3.):
                raise RuntimeError('test release deadline')
        if self.stage == 'current':
            return dict(speed_mps=.4, steering_rad=.2, reference_s=snapshot['reference_s'])
        return [.5, .2, dict(handoff_state='FOLLOW', reason='handoff_follow', smoke_resets=self.resets,
            normal=dict(mode='g1_follow', preferred_wall='left', path_wall='left',
                        selector_mode='SOLO', selector_choice='baseline', path_source_s=.9))]


def smoke_factory(stage, header):
    return SmokeEvaluator(stage, header)


def exit_before_ready(stage, header):
    os._exit(23)


def supported(output):
    return output[0] > 0


def snapshot(ref, first, hold=False):
    return dict(reference_s=ref, epoch=0, side='left', previous_steering_rad=.1, dt_s=.1,
                smoke_hold=hold, scan=dict(representation='native_measurements_v1',
                    measurements=dict(source_epoch=0, acquired_at_s=[first, first+.01])))


def collect_until(worker, predicate):
    deadline = time.monotonic()+3.
    rows = []
    while not predicate():
        worker.connection.poll(.02)
        rows.extend(worker.collect())
        assert worker.fault is None, worker.fault
        assert time.monotonic() < deadline, 'bounded completion deadline'
    return rows


def test_spawn_two_slots_stop_discards_both_late_results_and_closes_normally():
    context = mp.get_context('spawn')
    header = {key: context.Event() for key in ('plan_started', 'current_started', 'release')}
    worker = RuntimeDualProcess(header, evaluator_factory=smoke_factory, supported=supported, work_timeout=5.)
    processes = [worker.plan.process, worker.current.process]
    try:
        assert processes[0].pid != processes[1].pid and all(p.is_alive() for p in processes)
        assert worker.offer_plan(snapshot(1., .9))
        first = collect_until(worker, lambda: worker.saved_plan is not None)
        assert first[0]['output'][2]['smoke_resets'] == 1
        assert worker.offer_next_current(snapshot(1.1, 1., True), now_s=1.11)
        assert worker.offer_plan(snapshot(1.2, 1.1, True))
        assert header['current_started'].wait(2.) and header['plan_started'].wait(2.)
        worker.invalidate('explicit_stop')
        header['release'].set()
        late = collect_until(worker, lambda: worker.old_generation == 2)
        assert late == []
        assert worker.ready is worker.retained is worker.saved_plan is None
        assert worker.offer_plan(snapshot(1.3, 1.2))
        next_rows = collect_until(worker, lambda: worker.saved_plan is not None)
        assert next_rows[0]['generation'] == 1 and next_rows[0]['output'][2]['smoke_resets'] == 2
    finally:
        header['release'].set()
        report = worker.close()
    assert not report['forced'] and not report['alive'] and report['exitcode'] == 0
    assert {row['stage'] for row in report['lanes']} == {'plan', 'current'}
    assert all(not p.is_alive() and p.exitcode == 0 for p in processes)
    assert worker.close() is report


def test_spawn_handshake_eof_reaps_failed_lane_and_closes_pipe():
    lane = Lane.__new__(Lane)
    with pytest.raises(EOFError):
        lane.__init__('current', {}, capacity=4096, timeout=3., evaluator_factory=exit_before_ready)
    assert lane.closed and lane.connection.closed
    assert not lane.process.is_alive() and lane.process.exitcode == 23
    assert lane.cleanup == dict(stage='current', forced=False, exitcode=23, alive=False)
