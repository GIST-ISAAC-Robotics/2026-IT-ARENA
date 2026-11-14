import copy
import hashlib
import json

import pytest

from stage import DetailArchive,StatusEncoder,load_details,restore_status


def row(context):
    return dict(reason='clear',safe_speed_mps=.1,reference_s=8.,command_stale=False,
        swept_clearance_m=.5,no_return_policy='unknown',partial_observation=dict(allowed=True,
        reason='retained_memory_current_sweep_crawl',latch_time_s=7.52,
        latch_evidence=dict(scope='sensor_points_only_not_object_identity',witnesses=[dict(x=.1)],
                            correspondence_context=context),
        retained_memory_command=dict(accepted=True,speed_cap_mps=.1)))


class MemoryArchive:
    def __init__(self,state='pending'):self.current_state=state;self.records={};self.calls=0
    def submit(self,context):
        self.calls+=1;data=json.dumps(context).encode()
        ref=dict(archive_id='test',record_id=self.calls,sha256=hashlib.sha256(data).hexdigest(),bytes=len(data))
        self.records[('test',self.calls)]={**ref,'data':copy.deepcopy(context)}
        return ref
    def state(self,ref):return self.current_state
    def snapshot(self):return dict(archive_id='test')


def test_full_default_is_byte_identical_without_archive():
    value=row(dict(points=[[1.,2.]]));original=copy.deepcopy(value)
    assert StatusEncoder().encode(value)==json.dumps(value)
    assert value==original
    with pytest.raises(ValueError):StatusEncoder('compact_v1')


@pytest.mark.parametrize('state',['pending','failed','dropped'])
def test_unconfirmed_archive_keeps_full_original_evidence(state):
    value=row(dict(points=[[1.,2.]]));archive=MemoryArchive(state)
    result=json.loads(StatusEncoder('compact_v1',archive).encode(value))
    assert result['partial_observation']==value['partial_observation']
    assert result['status_detail']['context_storage']==state
    assert restore_status(result,archive.records)==value


def test_context_is_once_dynamic_output_and_new_context_stay_current():
    context=dict(points=[[1.,2.]]);value=row(context);original=copy.deepcopy(value)
    archive=MemoryArchive('written');encoder=StatusEncoder('compact_v1',archive)
    compact=json.loads(encoder.encode(value))
    assert value==original and compact['partial_observation']['latch_evidence']['witnesses']==[dict(x=.1)]
    assert restore_status(compact,archive.records)==value
    value.update(reason='stale_input',safe_speed_mps=0.,reference_s=8.2,command_stale=True)
    current=json.loads(encoder.encode(value))
    assert archive.calls==1 and restore_status(current,archive.records)==value
    value['partial_observation']['latch_evidence']['correspondence_context']=dict(points=[[3.,4.]])
    replacement=json.loads(encoder.encode(value))
    assert archive.calls==2 and restore_status(replacement,archive.records)==value
    with pytest.raises(KeyError):restore_status(replacement,{})


def test_real_sidecar_write_ack_round_trip_and_integrity(tmp_path):
    path=tmp_path/'evidence.jsonl';archive=DetailArchive(path)
    try:
        value=row(dict(points=[[1.,2.]]));encoder=StatusEncoder('compact_v1',archive)
        encoder.encode(value);archive.log.queue.join()
        result=json.loads(encoder.encode(value))
        assert result['status_detail']['context_storage']=='written'
        assert restore_status(result,load_details(path))==value
    finally:
        assert archive.close()
    changed=json.loads(path.read_text());changed['data']['points'][0][0]=99.
    path.write_text(json.dumps(changed)+'\n')
    with pytest.raises(ValueError,match='integrity'):load_details(path)
