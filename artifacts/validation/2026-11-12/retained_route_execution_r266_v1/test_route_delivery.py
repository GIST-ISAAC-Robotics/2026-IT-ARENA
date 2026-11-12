import copy
import time
import pytest
from test_recheck_process import inputs
from arena_autonomy.decision_delivery import source_identity
from route_delivery import RouteProcess


def collect(worker, stage):
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        rows=worker.collect()
        assert worker.fault is None
        for row in rows:
            if row['stage']==stage:return row
        time.sleep(.001)
    raise AssertionError('completion deadline')


def prime(worker,calls):
    for i,call in enumerate(calls):
        assert worker.offer_plan(call['input'])
        row=collect(worker,'plan')
        if i<len(calls)-1:worker.discard_plan((row['generation'],row['sequence']))
    return row


def state(row,extra=.001):
    cur=row['input']
    return dict(now_s=cur['reference_s']+extra,previous_steering_rad=cur['previous_steering_rad'],
                side=cur['side'],motion_epoch=cur['epoch'],sensor_epoch=source_identity(cur)[0])


def cleanup(worker):
    assert all(not s['forced'] and s['exitcode']==0 for s in worker.close())


@pytest.fixture
def ready():
    header,calls=inputs();worker=RouteProcess(header)
    try:
        prime(worker,calls)
        assert worker.retained is None
        assert worker.offer_next_current(calls[-1]['current_rebase']['input'])
        row=collect(worker,'current')
        assert row['output']['speed_mps']>0
        yield worker,row,calls
    finally:cleanup(worker)


def test_only_actual_commit_retains_original_route_and_is_single_use(ready):
    worker,row,_=ready
    before=copy.deepcopy(row['plan'])
    candidate=worker.prepared(**state(row))
    assert candidate['kind']=='candidate' and worker.retained is None
    candidate['result']['output']['speed_mps']=999
    assert worker.ready['output']['speed_mps']!=999
    worker.commit_published((row['generation'],row['sequence']),**state(row))
    assert worker.retained['input']==before['input'] and worker.retained['output']==before['output']
    with pytest.raises(ValueError):worker.commit_published((row['generation'],row['sequence']),**state(row))


@pytest.mark.parametrize('field',('now_s','previous_steering_rad','side','motion_epoch','sensor_epoch'))
def test_after_check_changes_do_not_apply(ready,field):
    worker,row,_=ready;options=state(row)
    options[field]=('right' if options[field]=='left' else 'left') if field=='side' else options[field]+1
    assert worker.prepared(**options)['kind']=='not_applied'
    assert worker.retained is None


def test_stop_between_ready_and_commit_blocks_it(ready):
    worker,row,_=ready;worker.invalidate('explicit_stop')
    assert worker.prepared(**state(row)) is None
    with pytest.raises(ValueError):worker.commit_published((row['generation'],row['sequence']),**state(row))


def test_retained_requires_new_scan_and_current_proof(ready):
    worker,row,calls=ready
    worker.commit_published((row['generation'],row['sequence']),**state(row))
    assert not worker.offer_next_current(row['input'])
    # 같은 저장 현재 입력을 다시 쓴 물리 시험이 아니다. 실제 원문 소유를 검사한다.
    worker.last_checked_source=None
    assert worker.offer_next_current(row['input'])
    checked=collect(worker,'current')
    assert checked['route_check']['retained']
    assert checked['output'].get('current_route_proof',{}).get('accepted')
    assert checked['plan']['input']==row['plan']['input']
    assert worker.prepared(**state(checked))['reason']=='positive_application_interval'


def test_retained_failure_has_no_new_command_and_does_not_extend_time(ready):
    worker,row,_=ready
    worker.commit_published((row['generation'],row['sequence']),**state(row))
    published=worker.last_application
    broken=copy.deepcopy(row['input']);broken['epoch']+=1
    worker.last_checked_source=None
    assert worker.offer_next_current(broken)
    checked=collect(worker,'current')
    assert checked['discard_reason']=='retained_route_not_proven'
    assert worker.ready is None and worker.last_application==published


def test_new_zero_plan_and_inflight_old_check_never_revive_retained(ready):
    worker,row,calls=ready
    worker.commit_published((row['generation'],row['sequence']),**state(row))
    worker.last_checked_source=None
    assert worker.offer_next_current(row['input'])
    # 실제 새 계획 결과 처리 경계. 0계획은 공통 상태 결정이며 보조보다 먼저 반영한다.
    from overlap import OverlapProcess
    from unittest.mock import patch
    zero=copy.deepcopy(row['plan']);zero.update(stage='plan',sequence=999)
    zero['output'][0]=0.
    with patch.object(OverlapProcess,'collect',return_value=[zero]):
        worker.collect()
    assert worker.retained is None
    late=collect(worker,'current')
    assert late['discard_reason']=='new_plan_superseded_current_check'
    assert worker.ready is None


def test_new_positive_plan_supersedes_retained_check(ready):
    worker,row,calls=ready
    worker.commit_published((row['generation'],row['sequence']),**state(row))
    worker.last_checked_source=None
    assert worker.offer_next_current(row['input'])
    from overlap import OverlapProcess
    from unittest.mock import patch
    new=copy.deepcopy(row['plan']);new.update(stage='plan',sequence=999)
    with patch.object(OverlapProcess,'collect',return_value=[new]):worker.collect()
    late=collect(worker,'current')
    assert late['discard_reason']=='new_plan_superseded_current_check'
    assert worker.ready is None


def test_stop_invalidates_both_actual_inflight_processes():
    header,calls=inputs();worker=RouteProcess(header,test_floor_s=.10)
    try:
        prime(worker,calls)
        assert worker.offer_next_current(calls[-1]['current_rebase']['input'])
        assert worker.offer_plan(calls[-1]['input'])
        worker.invalidate('explicit_stop')
        deadline=time.monotonic()+8
        while worker.plan.inflight or worker.current.inflight:
            assert worker.collect()==[]
            assert worker.fault is None and time.monotonic()<deadline
            time.sleep(.001)
        assert worker.old_generation==2 and worker.retained is None and worker.ready is None
    finally:cleanup(worker)


def test_new_plan_can_check_scan_used_by_old_retained_route(ready):
    worker,row,calls=ready
    worker.commit_published((row['generation'],row['sequence']),**state(row))
    assert not worker.offer_next_current(row['input'])
    assert worker.offer_plan(calls[-1]['input'])
    newer=collect(worker,'plan')
    assert newer['sequence']!=row['plan']['sequence']
    assert worker.offer_next_current(row['input'])
    checked=collect(worker,'current')
    assert not checked['route_check']['retained']


def test_geometry_expiry_during_publication_wait_cannot_be_renewed(ready):
    worker,row,_=ready
    # 계산 완료 때 499ms였던 기하가 발행 대기 중 500ms를 넘는 반례다.
    origin=row['input']['reference_s']-.499
    candidate=worker.ready
    candidate['plan']['output'][2]['normal']['path_source_s']=origin
    candidate['output']['current_route_proof']['original_path_source_s']=origin
    candidate['route_check']['retained']=True
    options=state(row,extra=.002)
    assert worker.prepared(**options)['reason']=='route_geometry_expired'
    assert worker.retained is None
