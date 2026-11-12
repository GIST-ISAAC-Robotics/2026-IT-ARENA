"""보존된 실측 입력만 비교한다. 새 취득/구동/절대 시각 교정은 하지 않는다."""
import importlib.util
import json
import time
from collections import Counter
from pathlib import Path
import numpy as np
from sample_clock import C1SampleClock

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
spec=importlib.util.spec_from_file_location('frozen_clock',A.parent/'c1_sample_clock_r259_v1/sample_clock.py')
frozen=importlib.util.module_from_spec(spec)
spec.loader.exec_module(frozen)

def stats(values):
    return dict(min=float(min(values)),median=float(np.median(values)),
                p95=float(np.percentile(values,95)),max=float(max(values))) if values else None

results=[]
for name in ('r249','r251','r259'):
    folder=ROOT/'build'/f'{name}_recovered'/('live' if name=='r259' else 'c1')
    rows=list(zip(map(json.loads,(folder/'sdk_raw.jsonl').read_text().splitlines()),
                  map(json.loads,(folder/'converted.jsonl').read_text().splitlines())))
    for label,cls in [('fixed_prefix',frozen.C1SampleClock),('tracking',C1SampleClock)]:
        clock=cls(source_epoch=7)
        measurements={key:[] for key in ('anchor_residual_ms','publication_wait_ms',
                                          'sample_period_us','effective_period_us','pc_conversion_ms')}
        for raw,record in rows:
            start=time.perf_counter_ns()
            p,d,due=clock.convert(raw,receipt_monotonic_ns=record['receipt_monotonic_ns'],
                                 receipt_ros_ns=record['receipt_ros_ns'])
            elapsed=(time.perf_counter_ns()-start)/1e6
            if p is None:continue
            measurements['anchor_residual_ms'].append(d['sdk_anchor_residual_ns']/1e6)
            measurements['publication_wait_ms'].append(d['publication_wait_ns']/1e6)
            measurements['sample_period_us'].append(d['sample_period_ns']/1000)
            measurements['effective_period_us'].append(d.get('effective_sample_period_ns',d['sample_period_ns'])/1000)
            measurements['pc_conversion_ms'].append(elapsed)
        results.append(dict(record=name,variant=label,counts=dict(clock.counts),
                            metrics={k:stats(v) for k,v in measurements.items()}))
(A/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results))
