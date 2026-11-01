"""직전 완주본과 같은 진행 위치의 시간·속도·판단 사유를 비교한다. 평가 전용."""
from collections import Counter
from hashlib import sha256
import json
import math
from pathlib import Path

import numpy as np

HERE=Path(__file__).resolve().parent


def load(folder):
    m=json.loads((folder/'run1_metrics.json').read_bytes())
    rows=[r for r in map(json.loads,(folder/'run1/trajectory.jsonl').read_bytes().splitlines()) if r['light']=='green']
    t=np.array([r['time'] for r in rows])-m['first_green_sample_s'];s=np.array([r['progress_m'] for r in rows])
    v=np.array([math.hypot(r['dynamics']['truth_longitudinal_mps'],r['dynamics']['truth_lateral_mps']) for r in rows])
    keep=np.r_[True,np.diff(s)>0]
    calls=[]
    for line in (folder/'run1/follow_decisions.jsonl').open(encoding='utf-8'):
        e=json.loads(line)
        if e['kind']!='call':continue
        i=e['data']['input'];o=e['data']['output']
        calls.append((i['reference_s']-m['first_green_sample_s'],o[0],o[2].get('reason'),o[2].get('observed_path_ahead_m'),o[2].get('swept_clearance_m')))
    return dict(t=t,s=s,v=v,ts=t[keep],ss=s[keep],calls=calls,lap=m['lap_s_from_first_green_sample'])


a=load(HERE.parent/'solo_preview_gate_v1');b=load(HERE)
sections=[]
edges=np.arange(0.,47.,2.)
ta=np.interp(edges,a['ss'],a['ts']);tb=np.interp(edges,b['ss'],b['ts'])
for i,(x0,x1) in enumerate(zip(edges,edges[1:])):
    row=dict(s0=float(x0),s1=float(x1),arc_s=float(ta[i+1]-ta[i]),ray_s=float(tb[i+1]-tb[i]),delta_s=float((tb[i+1]-tb[i])-(ta[i+1]-ta[i])),cum_delta_s=float(tb[i+1]-ta[i+1]))
    for tag,d in (('arc',a),('ray',b)):
        m=(d['s']>=x0)&(d['s']<x1)
        row[tag+'_v_min']=float(np.min(d['v'][m]));row[tag+'_v_max']=float(np.max(d['v'][m]))
        c=Counter()
        lo,hi=(ta[i],ta[i+1]) if tag=='arc' else (tb[i],tb[i+1])
        for call in d['calls']:
            if lo<=call[0]<=hi:c[call[2]]+=.02
        row[tag+'_reasons']={k:round(v,2) for k,v in c.most_common(4)}
    sections.append(row)
result=dict(scope='같은 진행 거리 비교. 단독 1회씩이며 반복 분산을 측정하지 않았다.',arc_lap_s=a['lap'],ray_lap_s=b['lap'],sections=sections,
            script_sha256=sha256(Path(__file__).read_bytes()).hexdigest())
(HERE/'lap_compare.json').write_text(json.dumps(result,ensure_ascii=False,indent=1),encoding='utf-8',newline='\n')
for r in sections:
    print(f"{r['s0']:4.0f} arc {r['arc_s']:.2f} ray {r['ray_s']:.2f} d {r['delta_s']:+.2f} cum {r['cum_delta_s']:+.2f} arcV {r['arc_v_min']:.2f}-{r['arc_v_max']:.2f} rayV {r['ray_v_min']:.2f}-{r['ray_v_max']:.2f} {r['arc_reasons'] if abs(r['delta_s'])>.15 else ''} {r['ray_reasons'] if abs(r['delta_s'])>.15 else ''}")
