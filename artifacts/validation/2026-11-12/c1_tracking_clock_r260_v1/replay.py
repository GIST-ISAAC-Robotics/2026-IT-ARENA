import json
from pathlib import Path
from collections import Counter
from sample_clock import C1SampleClock
from arena_autonomy.native_lidar_input import decode_measurements

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
reports=[]
for name in ('r249','r251','r259'):
    folder=ROOT/'build'/f'{name}_recovered'/('live' if name=='r259' else 'c1')
    clock=C1SampleClock(source_epoch=7);counts=Counter();last=None;max_wait=0;epochs=set()
    with (folder/'sdk_raw.jsonl').open() as f,(folder/'converted.jsonl').open() as g:
        for a,b in zip(f,g):
            raw=json.loads(a);record=json.loads(b)
            payload,diag,due=clock.convert(raw,receipt_monotonic_ns=record['receipt_monotonic_ns'],
                                          receipt_ros_ns=record['receipt_ros_ns'])
            counts['input']+=1
            if payload is None:counts['warmup_or_rejected']+=1;continue
            decoded=decode_measurements(json.dumps(payload),expected_time_basis='unverified_estimate')
            for key in ('ranges_m','angles_rad','source_frame_id','range_min_m','range_max_m','time_basis'):
                expected=record['payload']
                assert expected is not None and payload[key]==expected[key],key
            if last is not None:assert payload['acquired_at_s'][0]>last
            last=payload['acquired_at_s'][-1]
            assert decoded.time_basis=='unverified_estimate' and not diag['time_contract_verified']
            assert due>=record['receipt_monotonic_ns']
            epochs.add(payload['source_epoch']);counts['published_candidates']+=1
            max_wait=max(max_wait,due-record['receipt_monotonic_ns'])
    reports.append(dict(record=name,counts=dict(counts),clock=dict(clock.counts),epochs=sorted(epochs),
        maximum_wait_ns=max_wait,geometry_and_source_values_unchanged=True,time_contract_verified=False))
(A/'replay_report.json').write_text(json.dumps(reports,indent=2)+'\n');print(json.dumps(reports))
