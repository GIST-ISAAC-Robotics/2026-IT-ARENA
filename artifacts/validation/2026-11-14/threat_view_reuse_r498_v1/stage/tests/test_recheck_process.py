"""r242 공용 구현의 실제 자식 프로세스의 입력 소유권·순서·STOP 세대 경계."""
import copy
import json
from pathlib import Path
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
A = ROOT/'artifacts/validation/2026-11-12/two_stage_recheck_r235_v1'
from arena_autonomy.recheck_process import TwoStageProcess, evaluate_recheck
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import equal_output


def inputs():
    manifest = json.loads((A/'bundle_manifest.json').read_bytes())
    events = [json.loads(line) for line in (ROOT/manifest['trace']).open()]
    return events[0]['data'], [event['data'] for event in events if event['kind'] == 'call'][:3]


def wait_result(worker):
    deadline = time.monotonic()+5
    while time.monotonic() < deadline:
        answer = worker.pump()
        assert worker.fault is None
        if answer is not None:
            return answer
        time.sleep(.001)
    raise AssertionError('result deadline')


def first_plan(worker, calls):
    for call in calls:
        worker.offer(call['input'])
        result = wait_result(worker)
    return result


def test_input_owned_output_identical_and_single_use():
    header, calls = inputs()
    worker = TwoStageProcess(header)
    try:
        result = first_plan(worker, calls)
        owner, motion = make_controller(header)
        current = copy.deepcopy(calls[-1]['current_rebase']['input'])
        expected = evaluate_recheck(result['output'], calls[-1]['input'], current, motion, owner.config)
        worker.offer_recheck(current, result)
        current['reference_s'] = -999
        with pytest.raises(RuntimeError, match='serial stage'):
            worker.offer(calls[0]['input'])
        answer = wait_result(worker)
        assert answer['stage'] == 'recheck'
        assert equal_output(answer['output'], expected)
        assert answer['output']['reference_s'] == calls[-1]['current_rebase']['input']['reference_s']
        with pytest.raises(ValueError, match='unused current-generation'):
            worker.offer_recheck(calls[-1]['current_rebase']['input'], result)
        assert worker.pump() is None
    finally:
        assert worker.close() == dict(forced=False, exitcode=0)


@pytest.mark.parametrize('stage', ('plan', 'recheck'))
@pytest.mark.parametrize('iteration', range(10))
def test_inflight_stop_never_adopts_old_result(stage, iteration):
    header, calls = inputs()
    worker = TwoStageProcess(header)
    try:
        if stage == 'plan':
            worker.offer(calls[0]['input'])
        else:
            result = first_plan(worker, calls)
            worker.offer_recheck(calls[-1]['current_rebase']['input'], result)
        assert worker.pump() is None
        assert worker.inflight is not None
        worker.invalidate()
        deadline = time.monotonic()+5
        while worker.inflight is not None:
            assert worker.pump() is None
            assert worker.fault is None and time.monotonic()<deadline
            time.sleep(.001)
        assert worker.stats['old_generation'] == 1
        assert worker.plan_token is None
        worker.offer(calls[0]['input'])
        fresh = wait_result(worker)
        assert fresh['generation'] == 1 and fresh['stage'] == 'plan'
        assert equal_output(fresh['output'], calls[0]['output'])
    finally:
        assert worker.close() == dict(forced=False, exitcode=0)


def test_invalidated_pending_recheck_does_not_start():
    header, calls = inputs()
    worker = TwoStageProcess(header)
    try:
        result = first_plan(worker, calls)
        worker.offer_recheck(calls[-1]['current_rebase']['input'], result)
        count = worker.stats['submitted']
        worker.invalidate()
        assert worker.pump() is None and worker.stats['submitted'] == count
        with pytest.raises(ValueError, match='unused current-generation'):
            worker.offer_recheck(calls[-1]['current_rebase']['input'], result)
    finally:
        assert worker.close() == dict(forced=False, exitcode=0)
