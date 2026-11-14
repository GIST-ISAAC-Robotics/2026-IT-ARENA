"""r523 저장 제어 receipt의 실제 drain 사용량을 집계한다."""
import json
from pathlib import Path

from analyze import quantiles

A = Path(__file__).resolve().parent
rows = json.loads((A/'remote_result/result/controller_controls.json').read_bytes())
rows = [r for r in rows if 7.62 <= r['control_reference_s'] < 27.52]
drained = [r for r in rows if r.get('pending_drain')]
expanded = [r for r in drained if r['pending_drain']['duration_ms'] > 2]
old_before = []
for r in drained:
    before = r['source_before_drain']
    ages = {key: (r['control_reference_s']-max(values))*1000
            for key, values in before.items() if values}
    if any(age > 30.000001 for age in ages.values()):
        old_before.append(dict(reference_s=r['control_reference_s'],
            before_age_ms=ages, after_gyro_available_s=r.get('gyro_available_source_s'),
            after_wheel_available_s=r.get('wheel_available_source_s'),
            state=r['state'], pending_drain=r['pending_drain']))
out = dict(active_interval_s=[7.62,27.52],
    drain_duration_ms=quantiles(r['pending_drain']['duration_ms'] for r in drained),
    maximum_consumed=max(r['pending_drain']['total'] for r in drained),
    calls_over_old2ms_budget=len(expanded),
    bounded_stop_calls=sum(r['pending_drain']['bounded_stop'] for r in drained),
    old_before_drain_calls=len(old_before),
    old_before_drain_followed_by_motion_stop=sum(r['state']=='MOTION_STOP' for r in old_before),
    interpretation='Before-age describes saved latest-source stamps against final control reference; it is not a simulated 2ms counterfactual.',
    expanded_call_receipts=expanded, old_before_receipts=old_before)
(A/'drain_outcome.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in out.items() if k not in ('expanded_call_receipts','old_before_receipts')},ensure_ascii=False))
