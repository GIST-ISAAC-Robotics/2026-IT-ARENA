"""실제 취득 audit의 발행→판단 대기와 event-driven 비용 예산을 대조한다."""
import bisect
from collections import Counter
import hashlib
import json
from pathlib import Path
import math

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
cost_dir=HERE.parent/'control_10hz_cost_r98_v1'
manifest=json.loads((cost_dir/'bundle_manifest.json').read_bytes())
trace=ROOT/manifest['trace']
acquisition=HERE.parent/'control_10hz_r91_v1/two_actor_run4/run1/acquisition.json'
cost_file=cost_dir/'jetson_stages.json'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
hashes={str(p.relative_to(ROOT)):sha(p) for p in (trace,acquisition,cost_file)}
audits={round(x['first_ray_stamp_s'],6):x for x in json.loads(acquisition.read_bytes())}
costs={x['seq']:sum(v['wall_ms'] for v in x['stages'].values())/1000.
       for x in json.loads(cost_file.read_bytes())['per_call_stages']}
rows=[]
with trace.open(encoding='utf8') as stream:
    for line in stream:
        e=json.loads(line)
        if e['kind']!='call':continue
        x=e['data']['input'];first=x['scan']['measurements']['acquired_at_s'][0]
        audit=audits[round(first,6)];publish=audit['source_stamp_at_publish_s']
        rows.append(dict(seq=e['seq'],first=first,publish=publish,reference=x['reference_s'],cost=costs[e['seq']]))
assert len(rows)==610 and len({r['first'] for r in rows})==610
def stats(v):
    v=sorted(v)
    return dict(count=len(v),minimum=v[0],median=v[len(v)//2],maximum=v[-1])
# 같은 저장 판단의 계산비용을 그대로 사용한 최선 도착시각 비교이며,
# 바뀐 기준시각으로 실제 기하/명령을 재계산한 결과가 아니다.
ideal=[r['publish']-r['first']+r['cost'] for r in rows]
sources=[r['publish'] for r in rows]
now=sources[0];last=-1;used=[]
while now<=sources[-1]+1.:
    i=bisect.bisect_right(sources,now+1e-8)-1
    if i<=last:
        if last>=len(rows)-1:break
        now=math.ceil((sources[last+1]-1e-9)/.02)*.02
        continue
    r=rows[i];finish=now+r['cost'];collected=math.ceil((finish-1e-9)/.02)*.02
    used.append(dict(seq=r['seq'],source_index=i,submitted=now,completed=finish,collected=collected,
        apply_source_age=collected-r['first'],skipped=i-last-1))
    last=i;now=collected
out=dict(scope='stored publication times + fixed recorded per-seq cost; timing-only estimate, not resimulated decisions',
    calls=len(rows),publish_first_age_s=stats([r['publish']-r['first'] for r in rows]),
    publish_to_original_reference_s=stats([r['reference']-r['publish'] for r in rows]),
    ideal_immediate_apply_age_s=stats(ideal),ideal_outside_existing_300ms=sum(x>=.3 for x in ideal),
    single_inflight_latest_at_50hz=dict(processed=len(used),skipped=sum(r['skipped'] for r in used),
        apply_age_s=stats([r['apply_source_age'] for r in used]),
        outside_300ms=sum(r['apply_source_age']>=.3 for r in used)),
    source_sha256=hashes,input_unchanged=all(sha(ROOT/p)==h for p,h in hashes.items()))
with (HERE/'scan_wait_report.json').open('x',encoding='utf8') as stream:json.dump(out,stream,ensure_ascii=False,indent=2);stream.write('\n')
print(json.dumps({k:v for k,v in out.items() if k!='source_sha256'},ensure_ascii=False))
