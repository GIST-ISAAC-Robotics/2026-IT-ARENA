"""공통 guard status의 actual control/기록 오류 계약. ROS transport는 기존 node fixture로 대체한다."""
import copy
import json
import threading
from types import SimpleNamespace as N

import pytest

from arena_autonomy import guard_status_payload as subject
import test_independent_guard_route as fixture

def method():
    return fixture.control_method()[0]


def node(kind='positive',context=None):
    n,outputs,statuses=fixture.node(partial=False,previous=0.)
    # 압축 대상 상세가 들어오는 수명만 고정한다. projection/sweep/braking/시각/출력은 실제 control이다.
    evidence=dict(scope='sensor_points_only_not_object_identity',witnesses=[dict(source_index=1)],
        reference_s=.4,correspondence_context=context if context is not None else dict(points=[[1.,2.]]))
    partial=dict(allowed=True,reason='retained_memory_current_sweep_crawl',speed_cap_mps=.1,
        unknown_filled=False,latch_time_s=.4,latch_reason='previous_point_in_missing_sweep',latch_evidence=evidence,
        retained_memory_command=dict(accepted=True,speed_cap_mps=.1,requested_speed_mps=.8))
    n.partial_policy=N(evaluate=lambda *a:dict(partial),reset=lambda:None)
    n.command_observation_priority=False
    if kind=='zero':n.command.drive.speed=0.
    elif kind=='stale':n.command.header.stamp=fixture.stamp(.3)
    elif kind=='missing':n.scan=None
    elif kind=='obstacle':n.scan.ranges[250]=.05
    return n,outputs,statuses,partial


def signature(message):
    return (message.drive.speed,message.drive.steering_angle,message.header.frame_id,
        message.header.stamp.sec,message.header.stamp.nanosec)


def test_full_factory_does_not_touch_files_or_start_writer(monkeypatch,tmp_path):
    def forbidden(*a,**k):raise AssertionError('default full must not construct archive')
    monkeypatch.setattr(subject,'DetailArchive',forbidden)
    assert subject.make_encoder('full',str(tmp_path/'untouched.jsonl'),forbidden) is None
    assert list(tmp_path.iterdir())==[]


@pytest.mark.parametrize('path',['','relative/record.jsonl'])
def test_compact_requires_run_absolute_path_but_invalid_path_falls_back(path):
    warnings=[]
    assert subject.make_encoder('compact_v1',path,warnings.append) is None
    assert len(warnings)==1


def test_same_path_restart_keeps_old_file_and_falls_back(tmp_path):
    path=tmp_path/'run1.jsonl';warnings=[]
    first=subject.make_encoder('compact_v1',str(path),warnings.append)
    first({'partial_observation':{'latch_evidence':{'correspondence_context':{'points':[[1.,2.]]}}}})
    first.archive.log.queue.join();first.close();original=path.read_bytes()
    assert subject.make_encoder('compact_v1',str(path),warnings.append) is None
    assert path.read_bytes()==original and len(warnings)==1


@pytest.mark.parametrize('kind',['positive','zero','stale','missing','obstacle'])
@pytest.mark.parametrize('mode',['full','compact_v1'])
def test_actual_control_output_stop_and_dynamic_status_equal(kind,mode,tmp_path):
    context=dict(points=[[1.,2.]])
    baseline,old_out,old_status,_=node(kind,context)
    stage,new_out,new_status,partial=node(kind,context)
    warnings=[];path=tmp_path/'unique_run.jsonl'
    encoder=subject.make_encoder(mode,str(path),warnings.append)
    stage.status_encoder=encoder
    try:
        if encoder is not None:
            encoder({'partial_observation':partial});encoder.archive.log.queue.join()
        method()(baseline)
        method()(stage)
        assert signature(new_out[-1])==signature(old_out[-1])
        assert new_out[-1].drive.speed==(.1 if kind=='positive' else 0.)
        if encoder is None:
            assert new_status[-1].data==old_status[-1].data and not path.exists()
        else:
            recovered=subject.restore_status(json.loads(new_status[-1].data),subject.load_details(path))
            assert recovered==json.loads(old_status[-1].data)
        assert warnings==[]
    finally:subject.close_encoder(encoder)


def test_compact_runtime_exception_keeps_actual_control_running_with_full_status(tmp_path):
    warnings=[];encoder=subject.make_encoder('compact_v1',str(tmp_path/'run.jsonl'),warnings.append)
    def failure(_):raise RuntimeError('synthetic encoder failure')
    encoder.encoder.encode=failure
    try:
        for kind in ['positive','zero']:
            baseline,old_out,old_status,_=node(kind)
            stage,new_out,new_status,_=node(kind);stage.status_encoder=encoder
            method()(baseline);method()(stage)
            assert signature(new_out[-1])==signature(old_out[-1])
            assert new_status[-1].data==old_status[-1].data
        assert len(warnings)==1 and 'synthetic encoder failure' in warnings[0]
    finally:encoder.close()


@pytest.mark.parametrize('failure',['write','flush'])
def test_background_disk_failure_keeps_actual_control_running_with_full_status(tmp_path,failure):
    warnings=[];encoder=subject.make_encoder('compact_v1',str(tmp_path/'run.jsonl'),warnings.append)
    stream=encoder.archive.stream
    def broken_write(_):raise OSError('synthetic disk failure')
    def broken_flush():raise OSError('synthetic flush failure')
    encoder.archive.stream=N(write=broken_write if failure=='write' else stream.write,
        flush=broken_flush if failure=='flush' else stream.flush,close=stream.close)
    context=dict(points=[[1.,2.]])
    stage,new_out,new_status,_=node('positive',context);stage.status_encoder=encoder
    try:
        method()(stage);encoder.archive.log.queue.join()
        baseline,old_out,old_status,_=node('positive',context)
        method()(baseline);method()(stage)
        assert signature(new_out[-1])==signature(old_out[-1])
        assert new_status[-1].data==old_status[-1].data
        assert encoder.archive.log.failed==1 and len(warnings)==1
        stage.command.drive.speed=0.
        method()(stage)
        assert new_out[-1].drive.speed==0. and len(warnings)==1
    finally:encoder.close()


def test_reference_is_not_published_before_first_flush_ack(tmp_path):
    path=tmp_path/'new_run.jsonl';encoder=subject.make_encoder('compact_v1',str(path))
    flushing,release=threading.Event(),threading.Event();stream=encoder.archive.stream
    def blocked_flush():
        flushing.set()
        if not release.wait(2.):raise TimeoutError('test flush acknowledgement not released')
        stream.flush()
    encoder.archive.stream=N(write=stream.write,flush=blocked_flush,close=stream.close)
    value=dict(reason='clear',safe_speed_mps=.1,reference_s=.6,partial_observation=dict(latch_time_s=.4,
        latch_evidence=dict(witnesses=[dict(source_index=1)],correspondence_context=dict(points=[[1.,2.]]))))
    original=copy.deepcopy(value)
    try:
        encoder(value);assert flushing.wait(1.)
        pending=json.loads(encoder(value))
        assert pending['partial_observation']==value['partial_observation']
        assert pending['status_detail']['context_storage']=='pending'
        release.set();encoder.archive.log.queue.join()
        ready=json.loads(encoder(value))
        assert ready['status_detail']['context_storage']=='written'
        assert subject.restore_status(ready,subject.load_details(path))==value==original
        assert encoder.archive.log.accepted==encoder.archive.log.written==1
    finally:
        release.set();encoder.close()


def test_queue_drop_and_warning_failure_return_exact_full(tmp_path,monkeypatch):
    def broken_warning(_):raise RuntimeError('synthetic logger failure')
    encoder=subject.make_encoder('compact_v1',str(tmp_path/'run.jsonl'),broken_warning)
    monkeypatch.setattr(encoder.archive.log,'submit',lambda _:False)
    _,_,_,partial=node()
    value=dict(reason='clear',safe_speed_mps=.1,partial_observation=partial)
    try:
        assert encoder(value)==json.dumps(value)
        assert encoder.disabled_reason=='RuntimeError: detail archive dropped'
        value['safe_speed_mps']=0.
        assert encoder(value)==json.dumps(value)
    finally:encoder.close()
