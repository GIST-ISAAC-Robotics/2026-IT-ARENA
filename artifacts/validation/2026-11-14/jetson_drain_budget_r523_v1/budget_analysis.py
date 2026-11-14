"""r522 실제 stale/소비 비용으로 r523 한정 예산을 산정한다."""
import gzip,json,math,statistics
from pathlib import Path
A=Path(__file__).resolve().parent
P=A.parent/'jetson_traceoff_continuity_r522_v1/remote_result/result'
controls=json.loads((P/'controller_controls.json').read_bytes())
mapping=json.loads((P/'clock_mapping.json').read_bytes())
with gzip.open(P/'publication_lateness.jsonl.gz','rt',encoding='utf8') as f:publication=[json.loads(l) for l in f]
active=[x for x in controls if 7.62<=x['control_reference_s']<27.52]
costs=sorted(x['pending_drain']['duration_ms']/x['pending_drain']['total'] for x in active if x['pending_drain']['total']>=8)
evidence=[]
for c in active:
    if c['state']!='MOTION_STOP':continue
    began=c['begin_monotonic_ns']/1e9-c['pending_drain']['duration_ms']/1000
    streams={}
    for label in ('wheels','gyro'):
        before=c['source_before_drain'][label][0]
        sent=[r['at'] for r in publication if r['kind']==label and r['at']>before+1e-10
            and mapping['monotonic_origin_s']+r['at']-mapping['simulation_origin_s']+r['late_ms']/1000<=began]
        needed=next((i+1 for i,t in enumerate(sent) if c['control_reference_s']-t<=.03),None)
        streams[label]=dict(before_source_s=before,published_pending=len(sent),published_stamps=sent,
            minimum_samples_for30ms=needed,latest_published_s=sent[-1])
    rounds=streams['gyro']['minimum_samples_for30ms']
    scheduled=rounds+min(rounds,streams['wheels']['published_pending'])
    evidence.append(dict(control=c,streams=streams,round_robin_samples_for_gyro_freshness=scheduled,
        estimated_ms_at_backlog_cost_p95=scheduled*costs[int(.95*(len(costs)-1))],
        estimated_ms_at_backlog_cost_max=scheduled*max(costs)))
result=dict(source='r522 one trace-OFF run',backlog_calls=len(costs),
    ms_per_consumed_sample=dict(median=statistics.median(costs),p95=costs[int(.95*(len(costs)-1))],maximum=max(costs)),
    failures=evidence,chosen_controller_budget_ms=6.,unchanged_guard_budget_ms=2.,unchanged_sample_limit=64,
    reason='20 samples at observed upper backlog cost need 4.75ms; 6ms leaves about1.25ms margin, exits early when empty; not a general bound',
    limitation='publication-before-drain does not directly measure DDS unread queue; callback receipt and limit termination are measured')
(A/'budget_evidence.json').write_bytes((json.dumps(result,indent=2)+'\n').encode('utf8'))
print(json.dumps({k:v for k,v in result.items() if k!='failures'}))
