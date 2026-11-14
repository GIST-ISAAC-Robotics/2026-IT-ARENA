"""4개 독립 stage의 고정 20초 구간을 동일 기준으로 집계한다."""
import contextlib,gzip,hashlib,io,json
from collections import Counter
from pathlib import Path
from causal_analysis import analyze
from analyze import quantiles
A=Path(__file__).resolve().parent
stages={'baseline':A/'remote_result/result',
    'shm16':A/'candidate_shm16/remote_result/result',
    'drain':A/'candidate_shm16_drain/remote_result/result',
    'proof':A/'candidate_shm16_drain_proof/remote_result/result'}
results={}
for label,p in stages.items():
    with contextlib.redirect_stdout(io.StringIO()):analyze(p)
    read=lambda n:json.loads((p/n).read_bytes())
    causal=read('causal_summary.json')
    (A/(label+'_causal_summary.json')).write_text(json.dumps(causal,indent=2)+'\n',encoding='utf8')
    trace=(json.loads(line) for line in (p/'controller_trace.jsonl').open(encoding='utf8'))
    discarded=Counter();resets=Counter()
    for row in trace:
        if row['kind']=='call':
            delivery=row['data'].get('delivery',{})
            if delivery.get('kind')=='discarded':discarded[delivery.get('reason')]+=1
        elif row['kind']=='reset':resets[row['data'].get('reason')]+=1
    with gzip.open(p/'commands.jsonl.gz','rt',encoding='utf8') as f:commands=[json.loads(l) for l in f]
    with gzip.open(p/'guard_status.jsonl.gz','rt',encoding='utf8') as f:guards=[json.loads(l)['value'] for l in f]
    active=lambda t:7.62<=t<27.52
    metrics={}
    for role in ('controller','guard'):
        summary=read(role+'_summary.json');node=causal['nodes'][role]
        controls=[r for r in read(role+'_controls.json') if active(r['control_reference_s'])]
        drains=[r['pending_drain'] for r in controls if 'pending_drain' in r]
        metrics[role]=dict(reasons=node['reasons'],motion_stale=node['motion_stale_count'],
            stale_gyro=node['stale_gyro'],stale_wheel=node['stale_wheel'],
            wheel_delay_ms=node['callbacks']['wheels']['publication_to_callback_ms'],
            imu_delay_ms=node['callbacks']['imu']['publication_to_callback_ms'],
            callback_control_ms=node['callbacks']['control']['duration_ms'],
            motion_received=summary['motion_counts'],timing_dropped=summary['timing_export']['dropped'],
            pending_drain=dict(calls=len(drains),bounded_stop=sum(r['bounded_stop'] for r in drains),
                duration_ms=quantiles(r['duration_ms'] for r in drains),
                maximum_consumed=max((r['total'] for r in drains),default=0)))
    positive=[r for r in commands if r['topic']=='/drive' and r['speed']>0 and active(r['received_s'])]
    safe=[r for r in commands if r['topic']=='/drive/safe' and r['speed']>0 and active(r['received_s'])]
    active_guards=[r for r in guards if active(r['reference_s'])]
    results[label]=dict(nodes=metrics,positive_drive=len(positive),positive_safe=len(safe),
        positive_command_age_ms=quantiles((r['received_s']-r['source_s'])*1000 for r in positive),
        positive_command_gap_ms=quantiles((b['received_s']-a['received_s'])*1000 for a,b in zip(positive,positive[1:])),
        stale_input=sum(r['reason']=='stale_input' for r in active_guards),
        command_stale=sum(r.get('command_stale') is True for r in active_guards),
        delivery_discard=dict(discarded),trace_resets=dict(resets),
        execution=read('execution.json'),resource_return=read('resource_return.json'))
out=dict(interval_s=[7.62,27.52],requested_selection_s=[7.62,27.62],
    interval_note='common source-covered interval ends at final scan publication; motion source ends 27.515/27.514; EOF is separate',
    scope='simulated replay; each stage once; no hardware/host/MCU/B3 proof',stages=results)
(A/'stage_comparison.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf8')
for label,r in results.items():
    print(label,json.dumps(dict(motion_stale={k:v['motion_stale'] for k,v in r['nodes'].items()},
        positive_drive=r['positive_drive'],positive_safe=r['positive_safe'],command_stale=r['command_stale'],
        discard=r['delivery_discard'],positive_gap=r['positive_command_gap_ms'],positive_age=r['positive_command_age_ms'])))
