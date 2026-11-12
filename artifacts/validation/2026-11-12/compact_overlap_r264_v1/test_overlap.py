import copy
import time
import pytest
from test_recheck_process import inputs
from arena_autonomy.decision_process import make_controller
from arena_autonomy.decision_delivery import source_identity
from arena_autonomy.follow_trace import equal_output
from overlap import OverlapProcess, evaluate_current, application_reason
from overlap import plan_reference


def collect_until(worker, stages, timeout=8.):
    results={};end=time.monotonic()+timeout
    while not set(stages)<=set(results):
        for row in worker.collect():results[row['stage']]=row
        assert worker.fault is None
        assert time.monotonic()<end
        time.sleep(.001)
    return results


def prime(worker,calls):
    for i,call in enumerate(calls):
        assert worker.offer_plan(call['input'])
        row=collect_until(worker,['plan'])['plan']
        if i<len(calls)-1:worker.discard_plan((row['generation'],row['sequence']))
    return row


def clean(worker):
    status=worker.close()
    assert len(status)==2 and all(not r['forced'] and r['exitcode']==0 for r in status)


def test_actual_process_overlap_and_input_ownership():
    header,calls=inputs();worker=OverlapProcess(header,test_floor_s=.15)
    try:
        planned=prime(worker,calls)
        token=(planned['generation'],planned['sequence'])
        current=copy.deepcopy(calls[-1]['current_rebase']['input'])
        current['previous_steering_rad']+=.04
        owner,motion=make_controller(header)
        expected=evaluate_current(planned['output'],calls[-1]['input'],current,motion,owner.config)
        # 회수한 사본에 손대도 worker가 소유한 계획을 바꾸지 않는다.
        planned['output'][0]=999.
        assert worker.offer_current(token,current)
        current['previous_steering_rad']=999.
        assert worker.offer_plan(calls[-1]['input'])
        assert worker.current.inflight and worker.plan.inflight
        rows=collect_until(worker,['plan','current'])
        a,b=rows['plan'],rows['current']
        overlap=min(a['completed_wall_s'],b['completed_wall_s'])-max(a['began_wall_s'],b['began_wall_s'])
        assert overlap>0.05
        assert equal_output(b['output'],expected)
        assert b['input']['previous_steering_rad']!=999.
        assert b['plan']['output'][0]!=999.
        with pytest.raises(ValueError,match='token'):
            worker.offer_current(token,calls[-1]['current_rebase']['input'])
        assert not worker.offer_plan(calls[0]['input'])  # 완료 계획 슬롯은 덮어쓰지 않는다.
    finally:clean(worker)


@pytest.mark.parametrize('iteration',range(3))
def test_stop_invalidates_both_inflight_results_and_new_work_resets(iteration):
    header,calls=inputs();worker=OverlapProcess(header,test_floor_s=.10)
    try:
        row=prime(worker,calls)
        assert worker.offer_current((row['generation'],row['sequence']),calls[-1]['current_rebase']['input'])
        assert worker.offer_plan(calls[-1]['input'])
        worker.invalidate()
        end=time.monotonic()+5
        while worker.plan.inflight or worker.current.inflight:
            assert worker.collect()==[]
            assert worker.fault is None and time.monotonic()<end
            time.sleep(.001)
        assert worker.old_generation==2 and worker.saved_plan is None
        assert worker.offer_plan(calls[0]['input'])
        fresh=collect_until(worker,['plan'])['plan']
        assert fresh['generation']==1 and equal_output(fresh['output'],calls[0]['output'])
    finally:clean(worker)


@pytest.fixture(scope='module')
def checked():
    header,calls=inputs();worker=OverlapProcess(header)
    try:
        plan=prime(worker,calls)
        current=copy.deepcopy(calls[-1]['current_rebase']['input'])
        current['previous_steering_rad']+=.04
        assert worker.offer_current((plan['generation'],plan['sequence']),current)
        return collect_until(worker,['current'])['current']
    finally:clean(worker)


def kwargs(result):
    now=result['input'];return dict(generation=result['generation'],now_s=now['reference_s']+.001,
        previous_steering_rad=now['previous_steering_rad'],side=now['side'],motion_epoch=now['epoch'],
        sensor_epoch=source_identity(now)[0])


def test_application_checks_current_request_not_obsolete_plan_request(checked):
    assert checked['input']['previous_steering_rad']!=checked['plan']['input']['previous_steering_rad']
    assert application_reason(checked,**kwargs(checked)) is None


@pytest.mark.parametrize('field',('generation','previous_steering_rad','side','motion_epoch','sensor_epoch','now_s'))
def test_change_after_current_check_is_rejected(checked,field):
    options=kwargs(checked)
    if field=='side':options[field]='right' if options[field]=='left' else 'left'
    else:options[field]+=.4 if field=='now_s' else 1
    assert application_reason(checked,**options) is not None


def test_current_reference_cannot_be_restamped(checked):
    changed=copy.deepcopy(checked)
    changed['output']['reference_s']+=.001
    assert application_reason(changed,**kwargs(changed))=='current_reference_changed'


def test_compact_reference_keeps_geometry_identity_without_old_arrays():
    header,calls=inputs()
    original=copy.deepcopy(calls[-1]['input'])
    reference=plan_reference(original)
    assert reference['reference_s']==original['reference_s']
    assert reference['scan']['measurements']=={'source_epoch':original['scan']['measurements']['source_epoch']}
    reference['scan']['measurements']['source_epoch']+=1
    assert original==calls[-1]['input']
