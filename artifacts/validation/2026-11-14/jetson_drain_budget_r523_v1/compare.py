"""r522 2ms와 r523 controller6ms를 동일 원입력 유효 구간에서 비교한다."""
import gzip,json
from collections import Counter
from pathlib import Path
from analyze import quantiles
A=Path(__file__).resolve().parent
roots={'budget2ms':A.parent/'jetson_traceoff_continuity_r522_v1/remote_result/result',
       'budget6ms':A/'remote_result/result'}
active=lambda t:7.62<=t<27.52
comparison={}
for label,p in roots.items():
    read=lambda n:json.loads((p/n).read_bytes())
    def rows(name):
        with gzip.open(p/(name+'.jsonl.gz'),'rt',encoding='utf8') as f:return [json.loads(l) for l in f]
    summary=read('summary.json');causal=read('causal_summary.json')
    commands=rows('commands');all_guard=[r['value'] for r in rows('guard_status')]
    guard=[r for r in all_guard if active(r['reference_s'])]
    controllers=read('controller_controls.json');controls=[r for r in controllers if active(r['control_reference_s'])]
    timer=[json.loads(l) for l in (p/'local_pursuit_timing.jsonl').read_text(encoding='utf8').splitlines()]
    metrics={}
    for topic in ('/drive','/drive/safe'):
        selected=[r for r in commands if r['topic']==topic and active(r['received_s'])]
        positive=[r for r in selected if r['speed']>0]
        a,b=max(zip(positive,positive[1:]),key=lambda pair:pair[1]['received_s']-pair[0]['received_s'])
        metrics[topic]=dict(total=len(selected),zero=sum(r['speed']==0 for r in selected),positive=len(positive),
            all_command_gap_ms=quantiles((b['received_s']-a['received_s'])*1000 for a,b in zip(selected,selected[1:])),
            positive_command_gap_ms=quantiles((b['received_s']-a['received_s'])*1000 for a,b in zip(positive,positive[1:])),
            positive_source_age_ms=quantiles((r['received_s']-r['source_s'])*1000 for r in positive),
            positive_age_over200ms=sum(r['received_s']-r['source_s']>.2 for r in positive),
            largest_positive_gap=dict(before=a,after=b))
    failures=[]
    for c in controls:
        if c['state']!='MOTION_STOP':continue
        before=[r for r in controls if c['begin_monotonic_ns']-200000000<r['begin_monotonic_ns']<c['begin_monotonic_ns']]
        longest=max(before,key=lambda r:r['end_monotonic_ns']-r['begin_monotonic_ns'])
        failures.append(dict(control=c,preceding_longest_control=longest,
            preceding_longest_control_ms=(longest['end_monotonic_ns']-longest['begin_monotonic_ns'])/1e6))
    comparison[label]=dict(commands=metrics,active_guard_reasons=dict(Counter(r['reason'] for r in guard)),
        active_guard_command_stale=sum(r.get('command_stale') is True for r in guard),
        active_controller_states=dict(Counter(r['state'] for r in controls)),
        all_control_invocations_ms=quantiles((r['end_monotonic_ns']-r['begin_monotonic_ns'])/1e6 for r in controls),
        timer_only_control_ms=causal['nodes']['controller']['callbacks']['control']['duration_ms'],
        calculation_collected=summary['calculation'],input_publication_lateness_ms=summary['input_publication_lateness_ms'],
        http_request_ms=summary['http_request_ms'],all_guard_reasons=summary['guard_reasons'],
        all_controller_control_states=dict(Counter(r['state'] for r in controllers)),
        phases={phase:dict(
            controller_states=dict(Counter(r['state'] for r in controllers if predicate(r['control_reference_s']))),
            guard_reasons=dict(Counter(r['reason'] for r in all_guard if predicate(r['reference_s']))),
            guard_command_stale=sum(r.get('command_stale') is True for r in all_guard if predicate(r['reference_s'])))
            for phase,predicate in [('startup',lambda t:t<7.62),('source_covered',active),('source_end_and_eof',lambda t:t>=27.52)]},
        all_controller_status=summary['controller_states'],active_stale_evidence=failures,
        motion_counts={k:summary[k]['motion_counts'] for k in ('controller','guard')},
        timing_dropped={k:summary[k]['timing_export']['dropped'] for k in ('controller','guard')},
        full_trace_file_bytes=(p/'controller_trace.jsonl').stat().st_size,
        completion=summary['completion'])
out=dict(active_interval_s=[7.62,27.52],nominal_selection_s=[7.62,27.62],
    comparison_scope='r522 2ms versus r523 controller6ms, both trace-OFF; one run each; no r518/r521/current-common or hardware proof',
    stages=comparison)
(A/'comparison.json').write_bytes((json.dumps(out,indent=2,ensure_ascii=False)+'\n').encode('utf8'))
for label,r in comparison.items():
    print(label,json.dumps({k:v for k,v in r.items() if k in ('commands','active_guard_reasons','active_guard_command_stale',
        'active_controller_states','all_control_invocations_ms','timer_only_control_ms','motion_counts','timing_dropped')}))
