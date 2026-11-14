"""수신 나이·실제 자식 계산·명령 갱신 공백을 서로 분리해 집계한다."""
from collections import Counter
import gzip
import json
from pathlib import Path
import sys


def quantiles(values):
    values=sorted(values)
    if not values:return dict(n=0)
    def percentile(q):
        place=(len(values)-1)*q;lo=int(place);hi=min(lo+1,len(values)-1)
        return values[lo]+(values[hi]-values[lo])*(place-lo)
    return dict(n=len(values),median=percentile(.5),p95=percentile(.95),maximum=values[-1])


def analyze(out):
    read=lambda name:json.loads((out/name).read_bytes())
    def rows(name):
        with gzip.open(out/(name+'.jsonl.gz'),'rt',encoding='utf8') as stream:
            return [json.loads(line) for line in stream]
    execution=read('execution.json')
    commands=rows('commands');statuses=rows('autonomy_status');safety=rows('guard_status')
    http=rows('http');late=rows('publication_lateness')
    calculations=read('calculation_receipts.json') if (out/'calculation_receipts.json').exists() else []
    result=dict(scope='ARM installed ROS simulation-clock reconstructed saved inputs; no physical host/MCU/B3/real RGB',
        execution=execution,calculation={},commands={},
        controller_states=dict(Counter(r['value'].get('state') for r in statuses)),
        guard_reasons=dict(Counter(r['value'].get('reason') for r in safety)),
        guard_command_stale_count=sum(r['value'].get('command_stale') is True for r in safety),
        input_publication_lateness_ms={k:quantiles(r['late_ms'] for r in late if r['kind']==k) for k in ('scan','gyro','wheels')},
        http_request_ms=quantiles(r['request_ms'] for r in http),
        http_samples=len(http),http_status_stale_samples=sum(r['state']['connection']['status_stale'] for r in http))
    for stage in ('plan','current'):
        selected=[r for r in calculations if r['stage']==stage]
        result['calculation'][stage]=dict(
            child_wall_ms=quantiles((r['completed_wall_s']-r['began_wall_s'])*1000 for r in selected),
            submit_to_receipt_ms=quantiles((r['received_wall_s']-r['submitted_wall_s'])*1000 for r in selected))
    for topic in ('/drive','/drive/safe'):
        selected=[r for r in commands if r['topic']==topic]
        positive=[r for r in selected if r['speed']>0]
        distinct=[]
        for row in positive:
            if not distinct or row['source_s']!=distinct[-1]['source_s']:distinct.append(row)
        result['commands'][topic]=dict(count=len(selected),positive=len(positive),
            source_age_ms=quantiles((r['received_s']-r['source_s'])*1000 for r in selected),
            positive_source_age_ms=quantiles((r['received_s']-r['source_s'])*1000 for r in positive),
            positive_distinct_stamp_gaps_ms=quantiles((b['received_wall_s']-a['received_wall_s'])*1000 for a,b in zip(distinct,distinct[1:])),
            positive_age_over_200ms=sum(r['received_s']-r['source_s']>.2 for r in positive),
            maximum_speed=max((r['speed'] for r in selected),default=None))
    result['controller']=read('controller_summary.json')
    result['guard']=read('guard_summary.json')
    result['ops']=read('ops_summary.json')
    result['completion']=dict(normal_children=all(c['exitcode']==0 for c in execution['children'].values()),
        no_forced_or_residual=not execution['forced'] and not execution['residual_groups'],
        source_unchanged=execution['source_unchanged'],
        controller_children_stopped=not result['controller'].get('cleanup',{}).get('alive',True),
        physical_stop_verified=False)
    (out/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf8')
    print(json.dumps({k:result[k] for k in ('calculation','commands','controller_states','guard_reasons','completion')},ensure_ascii=False))


if __name__=='__main__':analyze(Path(sys.argv[1]))
