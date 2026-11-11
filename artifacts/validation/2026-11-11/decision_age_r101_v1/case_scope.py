"""전체610개 비용과 실제 경기 창598개를 분리한다. 원 집계는 보존한다."""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
run=HERE.parent/'control_10hz_r91_v1/two_actor_run4/run1'
cost_file=HERE.parent/'control_10hz_cost_r98_v1/jetson_stages.json'
paths=[run/'report.json',run/'follow_decisions.jsonl',cost_file]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
hashes={str(p.relative_to(ROOT)):sha(p) for p in paths}
meta=json.loads(paths[0].read_bytes());window=meta['case_window']
costs={r['seq']:sum(v['wall_ms'] for v in r['stages'].values())/1000.
       for r in json.loads(cost_file.read_bytes())['per_call_stages']}
groups={k:[] for k in ('case_window','before_case','after_case')}
with paths[1].open(encoding='utf8') as stream:
    for line in stream:
        e=json.loads(line)
        if e['kind']!='call':continue
        d=e['data']['input'];t=d['reference_s'];cost=costs[e['seq']]
        key='before_case' if t<window['green_sim_s'] else 'after_case' if t>window['end_sim_s'] else 'case_window'
        first=d['scan']['measurements']['acquired_at_s'][0]
        groups[key].append(dict(seq=e['seq'],cost=cost,apply_age=t-first+cost))
report=dict(scope='case window separation of existing stored timing analysis',case_window=window,
    stop_request_sim_s=meta['stop_request_sim_s'],groups={k:dict(count=len(v),
        over180ms=sum(r['cost']>.18 for r in v),over200ms=sum(r['cost']>.2 for r in v),
        stale300ms=sum(r['apply_age']>=.3 for r in v),
        maximum_cost_s=max((r['cost'] for r in v),default=None)) for k,v in groups.items()},
    source_sha256=hashes,input_unchanged=all(sha(ROOT/p)==h for p,h in hashes.items()))
with (HERE/'case_report.json').open('x',encoding='utf8') as stream:json.dump(report,stream,indent=2);stream.write('\n')
print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'}))
