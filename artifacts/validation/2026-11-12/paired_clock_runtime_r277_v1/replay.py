"""r276 실제 세 번 읽기와 SDK 원문을 새 공통 구현으로 재생한다."""
from pathlib import Path
import json,sys
R=Path(__file__).resolve().parents[4];A=Path(__file__).resolve().parent
sys.path.insert(0,str(R/'src/arena_vehicle_interface'))
from arena_vehicle_interface.host_clock_pair import sample_clocks
from arena_vehicle_interface.c1_sample_clock import C1SampleClock

run=R/'build/r276_recovered/run/c1'
raw=[json.loads(s) for s in (run/'sdk_raw.jsonl').read_text().splitlines()]
records=[json.loads(s) for s in (run/'converted.jsonl').read_text().splitlines()]
clock=C1SampleClock(source_epoch=records[0]['diagnostic']['source_epoch'])
assert len(raw)==len(records)==203
for batch,row in zip(raw,records):
    samples=row['clock_reads']['samples']
    mono=iter([t for s in samples for t in (s['before_monotonic_ns'],s['after_monotonic_ns'])])
    ros=iter([s['ros_ns'] for s in samples])
    m,r,detail=sample_clocks(lambda:next(mono),lambda:next(ros))
    assert (m,r,detail)==(row['receipt_monotonic_ns'],row['receipt_ros_ns'],row['clock_reads'])
    p,d,ready=clock.convert(batch,receipt_monotonic_ns=m,receipt_ros_ns=r)
    assert (p,d,ready)==(row['payload'],row['diagnostic'],row['ready_monotonic_ns'])
result=dict(original_clock_reads_equal=len(records),sdk_payload_diagnostic_ready_equal=len(raw),
            clock=dict(clock.counts),default_nominal_changed=False,new_capture=False)
(A/'replay.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
