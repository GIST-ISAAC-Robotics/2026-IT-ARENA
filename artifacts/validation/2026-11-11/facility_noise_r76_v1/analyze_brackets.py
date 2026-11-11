"""저장된 미관측 질의의 원시 양옆 광선 조건을 수치로 분리한다. 제어기 실행 없음."""
import collections,json,math
from pathlib import Path
HERE=Path(__file__).resolve().parent
rows=json.loads((HERE/'diagnosis_details.json').read_text(encoding='utf-8'))
reports=[]
for row in rows:
    for part in row['parts']:
        if part['stage']!='motion_region' or not part.get('unobserved_queries'):continue
        rays=part['rays'];padding=part['padding'];causes=collections.Counter();pairs=collections.Counter();examples=[]
        for q in part['unobserved_queries']:
            angles=[];radius=[]
            for r in rays:
                x,y=q[0]-r['origin'][0],q[1]-r['origin'][1];dx,dy=r['direction']
                angles.append(math.atan2(dx*y-dy*x,dx*x+dy*y));radius.append(math.hypot(x,y))
            lo=min(range(len(rays)),key=lambda i:max(angles[i],0.) if angles[i]>=-1e-10 else math.inf)
            hi=min(range(len(rays)),key=lambda i:max(-angles[i],0.) if angles[i]<=1e-10 else math.inf)
            margins=[math.dist(rays[i]['origin'],rays[i]['hit'])-radius[i]-padding[i] if rays[i]['hit'] is not None else None for i in [lo,hi]]
            reasons=[]
            if abs(lo-hi)>1:reasons.append('nonadjacent')
            if abs(angles[lo])+abs(angles[hi])>.025:reasons.append('wide_bracket')
            if any(rays[i]['status']=='unknown' for i in [lo,hi]):reasons.append('unknown')
            if any(rays[i]['status']=='masked' for i in [lo,hi]):reasons.append('masked')
            if any(v is not None and v<=0 for v in margins):reasons.append('return_padding')
            causes['+'.join(reasons) or 'other']+=1;pairs[f'{lo}:{hi}']+=1
            if len(examples)<3:examples.append(dict(q=q,pair=[lo,hi],status=[rays[i]['status'] for i in [lo,hi]],
                angles=angles[lo:lo+1]+angles[hi:hi+1],range_margin=margins,rays=[rays[lo],rays[hi]]))
        reports.append(dict(time_s=row['time_s'],causes=dict(causes),pairs=dict(pairs),examples=examples))
out=HERE/'bracket_summary.json'
if out.exists():raise SystemExit('preserve prior analysis')
out.write_text(json.dumps(reports,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps([{k:r[k] for k in ['time_s','causes','pairs']} for r in reports],ensure_ascii=False))
