"""회수한 C1 원문을 새로 변환해 발행 기록·대기·출력 경계를 대조한다."""
from pathlib import Path
import json,sys
from collections import Counter
ROOT=Path(__file__).resolve().parents[4]; A=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src/arena_vehicle_interface'))
from arena_vehicle_interface.c1_sample_clock import C1SampleClock
from arena_vehicle_interface.c1_native_preview import C1NativePreview

run=ROOT/'build/r270_recovered/run'
read=lambda p:json.loads((run/p).read_bytes())
supervisor=read('supervisor.json');summary=read('c1/summary.json');node=read('node/summary.json')
raw=[json.loads(x) for x in (run/'c1/sdk_raw.jsonl').read_text().splitlines()]
records=[json.loads(x) for x in (run/'c1/converted.jsonl').read_text().splitlines()]
assert len(raw)==len(records)==summary['counts']['input_batches']
epoch=records[0]['diagnostic']['source_epoch']
clock=C1SampleClock(source_epoch=epoch);nominal=C1NativePreview(source_epoch=epoch)
counts=Counter();last=None;periods=[];requested=[];actual_wait=[]
for batch,record in zip(raw,records):
    args={k:record[k] for k in ('receipt_monotonic_ns','receipt_ros_ns')}
    p,d,due=clock.convert(batch,**args);original,_=nominal.convert(batch,**args)
    assert p==record['payload'] and due==record['ready_monotonic_ns']
    assert d==record['diagnostic']
    counts['raw_reconstruction_equal']+=1
    if p is None:counts['warmup_or_rejected']+=1;continue
    for k in ('ranges_m','angles_rad','source_frame_id','range_min_m','range_max_m','time_basis'):
        assert p[k]==original[k]
    assert p['time_basis']=='unverified_estimate' and not d['time_contract_verified']
    assert record['publication_monotonic_ns']>=due
    assert p['acquired_at_s'][-1]<=record['publication_ros_ns']/1e9
    if last is not None: assert p['acquired_at_s'][0]>last
    last=p['acquired_at_s'][-1]
    counts['published']+=1;periods.append(d['sample_period_ns'])
    requested.append(d['publication_wait_ns'])
    actual_wait.append(record['publication_monotonic_ns']-record['receipt_monotonic_ns'])
assert counts['published']==summary['counts']['published']==summary['counts']['received']
assert summary['pending']==0 and summary['sensor_stop_ack'] and not summary['forced_cleanup']
assert supervisor['ports_free'] and supervisor['source_unchanged'] and not supervisor['forced']
assert all(v==0 for v in supervisor['children'].values())
assert node['scans']==counts['published'] and node['nonzero_private_commands']==0
assert node['motion']['drive_motor']==node['motion']['wheels']==node['motion']['rejected']==0
assert node['motion_time_policy']=='host_estimated_v1' and node['time_contract_verified'] is False
assert node['worker_exitcode']==0 and not node['worker_alive']
result=dict(counts=dict(counts),clock=dict(clock.counts),
    nominal_overlap_batches=nominal.counts['nominal_overlap_batches'],
    nominal_future_batches=nominal.counts['nominal_after_receipt_batches'],
    tracking_overlap=0,tracking_future_at_publication=0,
    period_ns=dict(min=min(periods),max=max(periods)),maximum_requested_wait_ns=max(requested),
    maximum_receipt_to_publication_ns=max(actual_wait),
    source_geometry_unchanged=True,actual_acquisition_accuracy_verified=False,
    common_scans=node['scans'],common_gyro=node['motion']['gyro'],private_positive_commands=0,
    children_normal=True,ports_free=True)
(A/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
