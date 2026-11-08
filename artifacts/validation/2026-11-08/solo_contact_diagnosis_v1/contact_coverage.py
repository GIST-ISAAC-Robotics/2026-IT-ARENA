"""편측 감속 구간의 바퀴별 1 ms 접촉 메시지 범위. 미수신은 0 N이 아니다."""
from pathlib import Path
from collections import defaultdict,Counter
import json,statistics
HERE=Path(__file__).resolve().parent;run=HERE/'run1';review=json.loads((HERE/'force_review.json').read_bytes());slow=review['groups']['right_slow'];lo=round(slow['from_s']*1000);hi=round(slow['to_s']*1000)
intervals={'before':(lo-250,lo-1),'right_slow':(lo,hi),'after':(hi+1,hi+250)}
rows=defaultdict(set);heights=defaultdict(list);surfaces=defaultdict(Counter);off_grid=0
for line in (run/'wheel_contacts.jsonl').open('rb'):
    row=json.loads(line);s=row['header']['stamp'];ns=int(s.get('sec',0))*10**9+int(s.get('nsec',0));ms=round(ns/1e6)
    key=next((g for g,(a,b) in intervals.items() if a<=ms<=b),None)
    if key is None:continue
    if ns!=ms*1000000:off_grid+=1;continue
    for c in row.get('contact',[]):
        names=[c[f'collision{i}']['name'] for i in (1,2)];which=next((i for i,n in enumerate(names) if n.startswith('arena_car::') and '_wheel::' in n),None)
        if which is None:continue
        w=names[which].split('::')[1];rows[key,w].add(ms);heights[key,w].extend(v.get('z',0) for v in c['position']);surfaces[key,w][names[1-which]]+=1
result={}
for key,(a,b) in intervals.items():
    data={};allms=set(range(a,b+1));anyms=set()
    for w in ('front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel'):
        stamps=rows[key,w];anyms.update(stamps);x=0;missing=0
        for m in range(a,b+1):x=0 if m in stamps else x+1;missing=max(missing,x)
        data[w]=dict(stamps=len(stamps),longest_missing_ms=missing,median_contact_z_m=statistics.median(heights[key,w]) if stamps else None,surface_names=dict(surfaces[key,w]))
    result[key]=dict(first_ms=a,last_ms=b,expected_grid_slots=b-a+1,at_least_one_wheel_stamps=len(anyms),wheels=data)
out=dict(scope='Absence is no reported contact, not verified zero normal force or physical lift. Other wheel timestamps check global stream continuity.',off_grid_messages=off_grid,intervals=result)
(HERE/'contact_coverage.json').write_text(json.dumps(out,indent=1)+'\n');print(json.dumps({g:[r['expected_grid_slots'],r['at_least_one_wheel_stamps'],{w:q['stamps'] for w,q in r['wheels'].items()}] for g,r in result.items()}))
