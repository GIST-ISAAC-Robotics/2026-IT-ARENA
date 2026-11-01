"""현재 진행 원호의 기존 우회 여유 제한 안에서만 최근 세 반환의 전진 증거를 대조한다.
평가용. 기록된 0명령을 해제하거나 조향·통과 결정을 수정하지 않는다.
"""
from collections import Counter, deque
from hashlib import sha256
import json
from pathlib import Path
import argparse

import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import transport_points


def capped_hint(triple,hist,bound=.06,motion_bound=.5):
    if len(triple)!=3:return 0.
    a,b,c=triple
    if not (0.<b[0]-a[0]<=.25 and 0.<c[0]-b[0]<=.25):
        return 0.
    if not (a[1]<b[1]<c[1]):return 0.
    try:
        points=np.asarray([transport_points(np.asarray([xy]),hist,t,c[1])[0] if t!=c[1] else xy for _,t,xy in triple])
    except ValueError:return 0.
    ds=np.diff(points,axis=0);dts=np.diff([a[0],b[0],c[0]])
    # 정지 후보 누적의 6 cm를 스캔마다 넘어 이력이 사라진 경우만 본다.
    # 이미 쓰던 동일 조각 이동 거리·전방성 기준과 0.5 m/s 상한을 더 넓히지 않는다.
    if (not np.isfinite(ds).all() or np.any(ds[:,0]<=np.maximum(bound,np.abs(ds[:,1])))
            or np.any(np.linalg.norm(ds,axis=1)>bound+motion_bound*dts)):
        return 0.
    return min(motion_bound,float(np.min(ds[:,0]/dts)))


def analyze(run):
    report=json.loads((run/'report.json').read_bytes())
    green=report['case_window']['green_sim_s'];end=report['case_window']['end_sim_s']
    triple=deque(maxlen=3);key=None;hint=0.;prev=None
    affected=Counter();calls=sources=positive=0;integral=0.;peak=0.;examples=[];sha=sha256();new_stops=0
    with (run/'follow_decisions.jsonl').open('rb') as f:
        for raw in f:
            sha.update(raw);e=json.loads(raw);kind=e['kind'];d=e['data']
            if kind=='header':motion=MotionConfig(**d['motion_config'])
            if kind=='reset':triple.clear();key=None;hint=0.;prev=None
            if kind!='call':continue
            t=d['input']['reference_s'];old,_,q=d['output'];new=old
            if not green<=t<=end:continue
            p=q.get('persistence') or {};x=d['input'];scan=x['scan']
            k=(x['epoch'],scan['frame_id'],scan['sec']+scan['nanosec']*1e-9)
            if key!=k:
                if key and (key[:2]!=k[:2] or k[-1]<=key[-1] or k[-1]-key[-1]>.25):triple.clear()
                key=k;sources+=1;hint=0.
                if (q.get('handoff_state')!='FOLLOW' or not p.get('fresh') or not p.get('observed')
                        or p.get('front_median_xy') is None or not x.get('timing_verified',False)):
                    triple.clear()
                else:
                    triple.append((k[-1],t,p['front_median_xy']))
                    if len(triple)==3:
                        _,_,hist=restore_call(x,motion)
                        hint=capped_hint(triple,hist)
                    positive+=hint>0
            space=q.get('follow_entry_space') or {}
            if (hint>0 and old>.001 and q.get('handoff_state')=='FOLLOW'
                and space.get('reason')=='observed_entry_distance_reserve' and space.get('applied')
                and not (q.get('preparation') or {}).get('applied')
                and not (q.get('entry_space') or {}).get('hold') and abs(old-space['cap_mps'])<1e-8):
                new=min(space['before_mps'], old+hint)
            if new>old+1e-8:
                phase='first_4s' if t-green<4 else 'after_4s'
                affected[phase]+=1;peak=max(peak,new-old)
                if len(examples)<6:examples.append([t-green,old,new])
            if prev and 0<t-prev[0]<=.1:
                integral+=(t-prev[0])*prev[1]
            prev=t,new-old;calls+=1;new_stops+=int((new<=.001)!=(old<=.001))
    if sha.hexdigest()!=report['follow_decisions']['sha256']:raise ValueError('기록 SHA 불일치')
    return dict(sources=sources, calls=calls, positive_hint_scans=positive,affected_calls=dict(affected),
        max_extra_mps=peak, added_command_integral_m=integral, zero_changes=new_stops,examples=examples,
        source_sha256=sha.hexdigest(),scope='세 반환 중앙점의 평면 이송. 같은 물체 신원·차체 속도·현장 통과나 실차 안전 증명이 아니다.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('out',type=Path);a=p.parse_args()
    result=analyze(a.run)
    with a.out.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps({k:result[k] for k in ('sources','positive_hint_scans','affected_calls','added_command_integral_m')}))
