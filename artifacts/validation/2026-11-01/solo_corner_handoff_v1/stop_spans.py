"""네 단독 완주본의 0명령 구간을 진행 위치와 사유로 나란히 센다. 평가 전용."""
from collections import Counter
from hashlib import sha256
import json
import math
from pathlib import Path

import numpy as np

HERE=Path(__file__).resolve().parent


def spans(folder):
    m=json.loads((folder/'run1_metrics.json').read_bytes())
    begin=m['first_green_sample_s'];end=begin+m['lap_s_from_first_green_sample']
    rows=[r for r in map(json.loads,(folder/'run1/trajectory.jsonl').read_bytes().splitlines()) if r['light']=='green']
    T=np.array([r['time'] for r in rows]);S=np.array([r['progress_m'] for r in rows])
    V=np.array([math.hypot(r['dynamics']['truth_longitudinal_mps'],r['dynamics']['truth_lateral_mps']) for r in rows])
    out=[];active=None;total=Counter()
    for line in (folder/'run1/follow_decisions.jsonl').open(encoding='utf-8'):
        e=json.loads(line)
        if e['kind']!='call':continue
        t=e['data']['input']['reference_s'];v,_,d=e['data']['output']
        if not begin+2.<=t<=end:continue
        if v<=.001:
            total[d.get('reason')]+=.02
            if active and t-active['end']<=.13:
                active['end']=t;active['reasons'][d.get('reason')]+=.02
            else:
                active=dict(start=t,end=t,reasons=Counter({d.get('reason'):.02}));out.append(active)
    for span in out:
        lo,hi=span['start'],span['end']+.3
        mask=(T>=lo)&(T<=hi)
        span.update(s=float(np.interp(lo,T,S)),seconds=span['end']-span['start']+.02,min_speed=float(np.min(V[mask])) if mask.any() else None,
                    reasons={k:round(v,2) for k,v in span['reasons'].items()})
    return dict(lap_s=m['lap_s_from_first_green_sample'],zero_command_s=round(sum(total.values()),2),by_reason={k:round(v,2) for k,v in total.items()},spans=out)


result=dict(scope='출발 2초 이후 랩 창의 0명령. 평가 전용이며 원인 판정이 아니다.',
    runs={name:spans(HERE.parent/name) for name in ('solo_corridor_arc_v1','solo_ray_join_v1','solo_preview_gate_v1','solo_corner_handoff_v1')},
    script_sha256=sha256(Path(__file__).read_bytes()).hexdigest())
(HERE/'stop_spans.json').write_text(json.dumps(result,ensure_ascii=False,indent=1),encoding='utf-8',newline='\n')
for name,run in result['runs'].items():
    print(name,run['lap_s'],run['zero_command_s'],run['by_reason'])
    for s in run['spans']:print('  s',round(s['s'],1),'dur',round(s['seconds'],2),'min',round(s['min_speed'],2),s['reasons'])
