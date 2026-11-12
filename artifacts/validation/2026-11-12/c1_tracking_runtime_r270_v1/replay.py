"""최근 raw 기록의 선택형 공통 시계를 이전 후보와 대조한다."""
import importlib.util
import json
from pathlib import Path
from collections import Counter
from time import perf_counter
import numpy as np
from arena_vehicle_interface.c1_sample_clock import C1SampleClock
from arena_autonomy.native_lidar_input import decode_measurements

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
old_path=A.parent/'c1_tracking_clock_r260_v1/sample_clock.py'
spec=importlib.util.spec_from_file_location('preserved_r260',old_path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
new=C1SampleClock(source_epoch=7);old=module.C1SampleClock(source_epoch=7)
folder=ROOT/'build/r269_recovered/run/c1'
raw=[json.loads(x) for x in (folder/'sdk_raw.jsonl').read_text().splitlines()]
records=[json.loads(x) for x in (folder/'converted.jsonl').read_text().splitlines()]
assert len(raw)==len(records)
counts=Counter();period=[];cost=[];wait=[];last=None
for batch,record in zip(raw,records):
    args={k:record[k] for k in ('receipt_monotonic_ns','receipt_ros_ns')}
    begin=perf_counter();p,d,ready=new.convert(batch,**args);cost.append((perf_counter()-begin)*1000)
    original,_,old_ready=old.convert(batch,**args)
    assert p==original and ready==old_ready
    counts['candidate_payload_and_ready_equal']+=1
    if p is None:counts['warmup_or_rejected']+=1;continue
    decode_measurements(json.dumps(p,allow_nan=False),expected_time_basis='unverified_estimate')
    for k in ('ranges_m','angles_rad','source_frame_id','range_min_m','range_max_m','time_basis'):
        assert p[k]==record['payload'][k]
    assert d['time_contract_verified'] is False and not d['timestamps_clipped']
    if last is not None:assert p['acquired_at_s'][0]>last
    last=p['acquired_at_s'][-1]
    assert ready>=record['receipt_monotonic_ns']
    wait.append(ready-record['receipt_monotonic_ns']);period.append(d['sample_period_ns'])
    counts['published_candidates']+=1
result=dict(counts=dict(counts),clock=dict(new.counts),
    sample_period_ns=dict(min=min(period),max=max(period)),maximum_wait_ns=max(wait),
    callback_cost_ms=dict(p50=float(np.percentile(cost,50)),p95=float(np.percentile(cost,95)),max=max(cost)),
    geometry_and_source_unchanged=True,time_contract_verified=False,
    scope='saved raw only; no new acquisition or absolute timing calibration')
(A/'replay_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
